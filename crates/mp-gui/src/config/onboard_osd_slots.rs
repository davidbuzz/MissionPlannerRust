// Copyright (C) 2026 David "Buzz" Bussenschutt
//
// This file is part of MissionPlannerRust, a Rust implementation derived from
// Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,
// https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.
//
// MissionPlannerRust is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, version 3 of the License.
//
// MissionPlannerRust is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// MissionPlannerRust. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-only

//! The OSD5 and OSD6 parameter slots: `Utilities/OsdTuningSlotProvider.cs` - the
//! `OSD_PARAM_SHOW_CONFIG` and `OSD_PARAM_CONFIG` requests and their replies - and
//! `ExtLibs/OSDConfigurator/GUI/Osd56ItemsSetup`, the "OSD/Telemetry Parameters Setup" dialog the
//! Onboard OSD page's "OSD / Telmetry Slots Config" opens (`ConfigOSD.cs:89-205`).
//!
//! * [`Slots::fetch`] is `OsdTuningSlotGetter.LoadAll`: a `ParamShow` for each of the eighteen
//!   slots - screens 5 and 6, indexes 1 to 9 - 20 ms apart, the replies collected until every
//!   slot has one or ten seconds pass with none, and one more pass for the slots still unanswered
//!   (`retries` 1). A reply's name is its `param_id` up to the first zero, and only a successful
//!   reply has one (`OsdTuningSlotProvider.cs:62-93, 118-178`). The page asks for it when a
//!   caption of screen 5 or 6 first needs the names (`ConfigOSD.cs:222-224`, the `Lazy`) and when
//!   the dialog opens (`:91-93`), under "Fetching Param Names" with Cancel (`:180-205`);
//! * [`Slots::update`] is `UpdateOsdTuningSlots`: a `ParamSet` for each change, the replies
//!   counted until all are in or ten seconds pass with none, "Screen s Slot i: Update Failed" for
//!   a reply that is not `OSD_PARAM_SUCCESS`, "Got n responses of m requests" when some never
//!   came, every line said - on the status line here - and the page's Refresh after
//!   (`:120-178`, `:113-118`);
//! * [`Dialog`] is `SetupDialog`: a heading a screen and a row a slot - the parameter it is
//!   assigned, chosen from every parameter the vehicle lists, its type, and its minimum, maximum
//!   and increment, which only type None takes - and "Done && Refresh", which sends the rows that
//!   changed (`SetupDialog.cs`, `ItemControl.cs`).
// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::Lock as _;
use std::sync::{Arc, Mutex, PoisonError};
use web_time::{Duration, Instant};

use mp_link::inspector::PacketSubscription;
use mp_mavlink_dialects::all::{MavMessage, OsdParamConfig, OsdParamShowConfig};
use mp_vehicle::VehicleId;

use super::onboard_osd::{ITEM_TYPES, NOT_CONNECTED, Setting, create, parse_screen_and_index};
use super::servo_output::{Combo, ERROR_TITLE};
use crate::telemetry::{Telemetry, TelemetryView};
use crate::textfield::TextField;

/// `SetupDialog.Text`. `// C#: GUI/Osd56ItemsSetup/SetupDialog.Designer.cs:78`
pub const DIALOG_TITLE: &str = "OSD/Telemetry Parameters Setup";
/// `button1.Text`, with its ampersand doubled for WinForms. `// C#: SetupDialog.Designer.cs:45`
pub const DONE_REFRESH: &str = "Done & Refresh";
/// `GetOsdSlotInfoWithDialog`'s progress caption. `// C#: ConfigOSD.cs:186`
pub const FETCHING: &str = "Fetching Param Names";
/// `UpdateOsdTuningSlots`' progress caption. `// C#: ConfigOSD.cs:125`
pub const UPDATING: &str = "Updating Parameters...";
/// `Thread.Sleep(20)` between the show requests. `// C#: OsdTuningSlotProvider.cs:161`
pub const SEND_GAP: Duration = Duration::from_millis(20);
/// Ten seconds from the last reply, for both the fetch and the update.
/// `// C#: ConfigOSD.cs:150; OsdTuningSlotProvider.cs:164`
pub const SILENCE: Duration = Duration::from_secs(10);
/// `LoadAll(1, ...)`: one retry.
pub const RETRIES: u8 = 1;
/// `OSD_PARAM_SUCCESS`.
pub const SUCCESS: u8 = 0;

/// The eighteen slots `LoadAll` asks about: screens 5 and 6, indexes 1 to 9.
/// `// C#: Utilities/OsdTuningSlotProvider.cs:122-126`
#[must_use]
pub fn all_slots() -> Vec<(u8, u8)> {
    (1..=9)
        .map(|i| (5, i))
        .chain((1..=9).map(|i| (6, i)))
        .collect()
}

/// The names the slots are assigned, once fetched: `(Screen, Index, Name)`, the name `null` for
/// a slot the vehicle refused or named with no terminator.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SlotNames {
    fetched: bool,
    names: Vec<(u8, u8, Option<String>)>,
}

impl SlotNames {
    /// The name a slot is assigned, if the fetch brought one.
    #[must_use]
    pub fn name(&self, screen: u8, index: u8) -> Option<&str> {
        self.names
            .iter()
            .find(|(s, i, _)| *s == screen && *i == index)
            .and_then(|(_, _, name)| name.as_deref())
    }

    /// Whether a fetch has brought the names.
    #[must_use]
    pub const fn is_fetched(&self) -> bool {
        self.fetched
    }

    /// The names a fetch brought.
    pub fn set(&mut self, names: Vec<(u8, u8, Option<String>)>) {
        self.names = names;
        self.fetched = true;
    }

    /// `Activate`'s new `Lazy`: fetched again when next needed.
    pub fn forget(&mut self) {
        self.fetched = false;
        self.names.clear();
    }

    /// Every slot with a name, for the facts: "5:1=RTL_ALT 6:2=...".
    #[must_use]
    pub fn listing(&self) -> String {
        self.names
            .iter()
            .filter_map(|(s, i, name)| name.as_ref().map(|n| format!("{s}:{i}={n}")))
            .collect::<Vec<_>>()
            .join(" ")
    }
}

/// A reply the link thread heard, as the handlers read it.
#[derive(Debug, Clone, PartialEq)]
pub enum Reply {
    /// `OSD_PARAM_SHOW_CONFIG_REPLY`: the request, its result, and the name up to the first zero.
    /// `// C#: OsdTuningSlotProvider.cs:62-93`
    Show {
        request_id: u32,
        result: u8,
        name: Option<String>,
    },
    /// `OSD_PARAM_CONFIG_REPLY`: the request and whether it succeeded.
    /// `// C#: OsdTuningSlotProvider.cs:95-114`
    Set { request_id: u32, success: bool },
}

impl Reply {
    /// A packet's reply, if it is one.
    #[must_use]
    pub fn of(message: &MavMessage) -> Option<Self> {
        match message {
            MavMessage::OsdParamShowConfigReply(m) => Some(Self::Show {
                request_id: m.request_id,
                result: m.result,
                // `Array.IndexOf(param_id, 0) > 0`: a name needs a terminator after at least one
                // character; sixteen characters with none read as no name.
                name: (m.result == SUCCESS)
                    .then(|| {
                        let zero = m.param_id.iter().position(|&b| b == 0)?;
                        if zero == 0 {
                            return None;
                        }
                        String::from_utf8(m.param_id.get(..zero)?.to_vec()).ok()
                    })
                    .flatten(),
            }),
            MavMessage::OsdParamConfigReply(m) => Some(Self::Set {
                request_id: m.request_id,
                success: m.result == SUCCESS,
            }),
            _ => None,
        }
    }
}

/// What a fetch is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum After {
    /// The captions of screens 5 and 6: the names kept, nothing else.
    Captions,
    /// The dialog opened over the names.
    Dialog,
}

/// `OsdTuningSlotGetter.Load`: one pass of requests and the wait for their replies.
#[derive(Debug)]
pub struct Fetch {
    /// Why.
    pub after: After,
    /// The slots this pass asks about, in order; sent from the front.
    pub to_send: Vec<(u8, u8)>,
    /// What was sent: the request id and its slot.
    pub sent: Vec<(u32, u8, u8)>,
    /// The replies so far, over every pass.
    pub results: Vec<(u8, u8, Option<String>)>,
    /// How many of this pass's requests have been answered.
    pub answered: usize,
    /// `lastResponseTime`.
    pub last_response: Instant,
    /// When the last request went.
    pub last_send: Option<Instant>,
    /// Which pass this is, from 0.
    pub pass: u8,
    /// When it started, for the progress dialog's marquee.
    pub started: Instant,
    /// `++request`.
    pub request: u32,
}

/// `SetupDialog.Change`: a row's new assignment.
/// `// C#: GUI/Osd56ItemsSetup/SetupDialog.cs:9-18`
#[derive(Debug, Clone, PartialEq)]
pub struct Change {
    pub screen: u8,
    pub index: u8,
    pub param_name: String,
    pub kind: u8,
    pub min: f64,
    pub max: f64,
    pub increment: f64,
}

/// `UpdateOsdTuningSlots`: the changes sent and the replies counted.
#[derive(Debug)]
pub struct Update {
    pub changes: Vec<Change>,
    /// How many have been sent.
    pub sent: usize,
    /// The request id of each sent, with its slot.
    pub requests: Vec<(u32, u8, u8)>,
    /// `responseCount`.
    pub responses: usize,
    /// `errors`.
    pub errors: Vec<String>,
    pub last_response: Instant,
    pub started: Instant,
    pub cancelled: bool,
}

/// A row of the dialog: `ItemControl`.
/// `// C#: GUI/Osd56ItemsSetup/ItemControl.cs`
#[derive(Debug, Clone, PartialEq)]
pub struct Row {
    pub screen: u8,
    pub index: u8,
    /// The item's name, "PARAM1".
    pub item: String,
    /// `cbParamName.SelectedIndex` into the shared list of names, `-1` as `None`.
    pub name_index: Option<usize>,
    pub initial_name_index: Option<usize>,
    /// `cbType.SelectedIndex`.
    pub kind: usize,
    pub initial_type: usize,
    /// `tbMin`, `tbMax`, `tbIncrement` as typed, and as they started.
    pub min: String,
    pub max: String,
    pub increment: String,
    pub initial_min: String,
    pub initial_max: String,
    pub initial_increment: String,
}

impl Row {
    /// `IsDirty`: whether the row differs from its start, and the change it makes. A type other
    /// than None compares the name and the type alone; None the three numbers too, which must
    /// parse.
    /// `// C#: GUI/Osd56ItemsSetup/ItemControl.cs:86-115`
    #[must_use]
    pub fn change(&self, names: &[String]) -> Option<Change> {
        let kind = u8::try_from(self.kind).unwrap_or(0);
        let param_name = self
            .name_index
            .and_then(|i| names.get(i))
            .cloned()
            .unwrap_or_default();
        let dirty = if kind != 0 {
            self.name_index != self.initial_name_index || self.kind != self.initial_type
        } else {
            self.min != self.initial_min
                || self.max != self.initial_max
                || self.increment != self.initial_increment
                || self.name_index != self.initial_name_index
                || self.kind != self.initial_type
        };
        if !dirty {
            return None;
        }
        let number = |text: &str| text.trim().parse::<f64>().unwrap_or(0.0);
        Some(Change {
            screen: self.screen,
            index: self.index,
            param_name,
            kind,
            min: if kind == 0 { number(&self.min) } else { 0.0 },
            max: if kind == 0 { number(&self.max) } else { 0.0 },
            increment: if kind == 0 {
                number(&self.increment)
            } else {
                0.0
            },
        })
    }

    /// Whether the number boxes take typing: `cbType.SelectedIndex == 0`.
    /// `// C#: GUI/Osd56ItemsSetup/ItemControl.cs:79-84`
    #[must_use]
    pub const fn numbers_enabled(&self) -> bool {
        self.kind == 0
    }
}

/// Which of a row's boxes is being typed into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Box_ {
    Min,
    Max,
    Increment,
}

/// Which of a row's combos is dropped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComboKind {
    Name,
    Type,
}

/// The dialog.
#[derive(Debug)]
pub struct Dialog {
    /// Every parameter the vehicle lists, sorted: `cbParamName`'s items.
    pub names: Arc<Vec<String>>,
    /// The rows, in the order shown: screen 5's then screen 6's, index ascending.
    pub rows: Vec<Row>,
    /// The combo dropped, by row.
    pub open: Option<(usize, ComboKind)>,
    /// The scroll of the dropped list.
    pub top_index: usize,
    /// The box being typed into, by row.
    pub typing: Option<(usize, Box_)>,
    pub field: TextField,
}

impl Dialog {
    /// The dialog over the configuration's screens 5 and 6: a row an item, ordered as the C#
    /// docks them - screen by screen, index ascending.
    /// `// C#: GUI/Osd56ItemsSetup/SetupDialog.cs:38-84`
    #[must_use]
    pub fn new(settings: &[Setting], slots: &SlotNames, all_names: Vec<String>) -> Self {
        let config = create(settings, 5..=6);
        let mut names = all_names;
        names.sort();
        let mut rows = Vec::new();
        let value = |item: &super::onboard_osd::Item, suffix: &str| {
            item.ends_with(settings, suffix)
                .and_then(|i| settings.get(i))
                .map_or(0.0, Setting::value)
        };
        for screen in &config.screens {
            let mut these: Vec<Row> = screen
                .items
                .iter()
                .filter_map(|item| {
                    let first = item
                        .options
                        .first()
                        .and_then(|&i| settings.get(i))
                        .map(|s| s.name.as_str())?;
                    let (s, i) = parse_screen_and_index(first)?;
                    let assigned = slots.name(s, i);
                    let name_index = assigned.and_then(|n| names.iter().position(|held| held == n));
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    let kind = value(item, "_TYPE") as usize;
                    let min = five_places(value(item, "_MIN"));
                    let max = five_places(value(item, "_MAX"));
                    let increment = five_places(value(item, "_INCR"));
                    Some(Row {
                        screen: s,
                        index: i,
                        item: item.name.clone(),
                        name_index,
                        initial_name_index: name_index,
                        kind,
                        initial_type: kind,
                        min: min.clone(),
                        max: max.clone(),
                        increment: increment.clone(),
                        initial_min: min,
                        initial_max: max,
                        initial_increment: increment,
                    })
                })
                .collect();
            these.sort_by_key(|row| row.index);
            rows.extend(these);
        }
        Self {
            names: Arc::new(names),
            rows,
            open: None,
            top_index: 0,
            typing: None,
            field: TextField::new(""),
        }
    }

    /// `GetChangedItems`.
    #[must_use]
    pub fn changes(&self) -> Vec<Change> {
        self.rows
            .iter()
            .filter_map(|row| row.change(&self.names))
            .collect()
    }

    /// The screens the rows belong to, in order, for the headings "OSD n".
    #[must_use]
    pub fn screens(&self) -> Vec<u8> {
        let mut screens: Vec<u8> = Vec::new();
        for row in &self.rows {
            if !screens.contains(&row.screen) {
                screens.push(row.screen);
            }
        }
        screens
    }

    /// A row's name combo, for the shared drawing.
    #[must_use]
    pub fn name_combo(&self, row: usize) -> Combo {
        let held = self.rows.get(row);
        Combo {
            param: String::new(),
            options: self
                .names
                .iter()
                .enumerate()
                .map(|(i, n)| (i64::try_from(i).unwrap_or(0), n.clone()))
                .collect(),
            selected: held
                .and_then(|r| r.name_index)
                .and_then(|i| i64::try_from(i).ok()),
            enabled: true,
            top_index: if self.open == Some((row, ComboKind::Name)) {
                self.top_index
            } else {
                0
            },
        }
    }

    /// A row's type combo.
    #[must_use]
    pub fn type_combo(&self, row: usize) -> Combo {
        Combo {
            param: String::new(),
            options: ITEM_TYPES
                .iter()
                .enumerate()
                .map(|(i, n)| (i64::try_from(i).unwrap_or(0), (*n).to_owned()))
                .collect(),
            selected: self.rows.get(row).and_then(|r| i64::try_from(r.kind).ok()),
            enabled: true,
            top_index: 0,
        }
    }

    /// A combo's arrow: dropped, or taken up.
    pub fn toggle_combo(&mut self, row: usize, kind: ComboKind) {
        self.leave_box();
        if self.open == Some((row, kind)) {
            self.open = None;
        } else {
            self.open = Some((row, kind));
            let mut combo = match kind {
                ComboKind::Name => self.name_combo(row),
                ComboKind::Type => self.type_combo(row),
            };
            combo.open_list();
            self.top_index = combo.top_index;
        }
    }

    /// The dropped list scrolled.
    pub fn scroll(&mut self, lines: i32) {
        let Some((row, kind)) = self.open else {
            return;
        };
        let mut combo = match kind {
            ComboKind::Name => self.name_combo(row),
            ComboKind::Type => self.type_combo(row),
        };
        combo.top_index = self.top_index;
        combo.scroll_list(lines);
        self.top_index = combo.top_index;
    }

    /// A row of the dropped list chosen.
    pub fn choose(&mut self, row: usize, kind: ComboKind, chosen: i64) {
        let chosen = usize::try_from(chosen).unwrap_or(0);
        if let Some(held) = self.rows.get_mut(row) {
            match kind {
                ComboKind::Name => held.name_index = Some(chosen),
                ComboKind::Type => held.kind = chosen.min(ITEM_TYPES.len() - 1),
            }
        }
        self.open = None;
    }

    /// A number box clicked into, while the row's type is None.
    pub fn begin_box(&mut self, row: usize, which: Box_) {
        self.leave_box();
        let Some(held) = self.rows.get(row) else {
            return;
        };
        if !held.numbers_enabled() {
            return;
        }
        let text = match which {
            Box_::Min => &held.min,
            Box_::Max => &held.max,
            Box_::Increment => &held.increment,
        };
        let text = text.clone();
        self.field = TextField::new("");
        self.field.set(text);
        self.typing = Some((row, which));
        self.open = None;
    }

    /// A key in the box being typed into: only digits, a point once and a leading minus are
    /// taken (`TextBoxKeyPress`); Enter or Escape leave the box.
    /// `// C#: GUI/Osd56ItemsSetup/ItemControl.cs:60-77`
    pub fn key(&mut self, event: &gpui::KeyDownEvent) -> bool {
        use crate::textfield::KeyOutcome;
        if self.typing.is_none() {
            return false;
        }
        let before = self.field.value().to_owned();
        match self.field.key(event) {
            KeyOutcome::Submitted | KeyOutcome::Cancelled => {
                self.leave_box();
                true
            }
            KeyOutcome::Changed => {
                if !allowed_number_text(self.field.value()) {
                    self.field.set(before);
                }
                true
            }
            KeyOutcome::Ignored => false,
        }
    }

    /// The box left: its text kept as typed.
    pub fn leave_box(&mut self) {
        if let Some((row, which)) = self.typing.take()
            && let Some(held) = self.rows.get_mut(row)
        {
            let text = self.field.value().to_owned();
            match which {
                Box_::Min => held.min = text,
                Box_::Max => held.max = text,
                Box_::Increment => held.increment = text,
            }
        }
    }
}

/// What `TextBoxKeyPress` lets through: digits, one point, a minus first.
fn allowed_number_text(text: &str) -> bool {
    let mut seen_point = false;
    for (i, c) in text.chars().enumerate() {
        match c {
            '0'..='9' => {}
            '.' if !seen_point => seen_point = true,
            '-' if i == 0 => {}
            _ => return false,
        }
    }
    true
}

/// `ToString("0.#####")`: up to five decimals, trailing zeros dropped.
#[must_use]
pub fn five_places(value: f64) -> String {
    let text = format!("{value:.5}");
    let trimmed = text.trim_end_matches('0').trim_end_matches('.');
    if trimmed.is_empty() || trimmed == "-" {
        "0".to_owned()
    } else {
        trimmed.to_owned()
    }
}

/// What the slots are doing.
#[derive(Debug)]
pub enum Phase {
    Idle,
    Fetching(Fetch),
    Updating(Update),
}

/// The slots: their names, the fetch and update under way, and the dialog.
pub struct Slots {
    names: SlotNames,
    phase: Phase,
    /// The replies the link thread heard, for the phase to read.
    replies: Arc<Mutex<Vec<Reply>>>,
    subscription: Option<PacketSubscription>,
    /// The dialog, while open.
    pub dialog: Option<Dialog>,
    /// Whether an update ended and the page's Refresh is due.
    refresh_due: bool,
    /// The last update's change count, for the facts.
    last_changes: usize,
    /// The last words said, for the facts.
    last_words: String,
}

impl Default for Slots {
    fn default() -> Self {
        Self {
            names: SlotNames::default(),
            phase: Phase::Idle,
            replies: Arc::new(Mutex::new(Vec::new())),
            subscription: None,
            dialog: None,
            refresh_due: false,
            last_changes: 0,
            last_words: String::new(),
        }
    }
}

impl std::fmt::Debug for Slots {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Slots")
            .field("names", &self.names)
            .field("phase", &self.phase)
            .field("dialog", &self.dialog.is_some())
            .finish_non_exhaustive()
    }
}

impl Slots {
    /// The names.
    #[must_use]
    pub const fn names(&self) -> &SlotNames {
        &self.names
    }

    /// `Activate`: fetched again when next needed; nothing under way survives.
    pub fn forget(&mut self) {
        self.names.forget();
        self.phase = Phase::Idle;
        self.subscription = None;
        self.dialog = None;
    }

    /// Whether a fetch or an update runs: the progress dialog shows.
    #[must_use]
    pub const fn busy(&self) -> Option<(&'static str, Instant)> {
        match &self.phase {
            Phase::Idle => None,
            Phase::Fetching(fetch) => Some((FETCHING, fetch.started)),
            Phase::Updating(update) => Some((UPDATING, update.started)),
        }
    }

    /// The names wanted for a caption: the fetch started unless one has run or runs.
    pub fn want_names(&mut self, telemetry: &Telemetry, view: &TelemetryView, now: Instant) {
        if self.names.is_fetched() || !matches!(self.phase, Phase::Idle) {
            return;
        }
        if !view.connected {
            // `CheckConnected` fails: an empty list, no names.
            self.names.set(Vec::new());
            return;
        }
        self.fetch(After::Captions, telemetry, now);
    }

    /// `GetOsdSlotInfoWithDialog`, then `LoadAll(1, token)`: the first pass started.
    fn fetch(&mut self, after: After, telemetry: &Telemetry, now: Instant) {
        self.subscribe(telemetry);
        self.phase = Phase::Fetching(Fetch {
            after,
            to_send: all_slots(),
            sent: Vec::new(),
            results: Vec::new(),
            answered: 0,
            last_response: now,
            last_send: None,
            pass: 0,
            started: now,
            request: 0,
        });
    }

    /// The button: `CheckConnected`, then the names fetched for the dialog.
    /// `// C#: ConfigOSD.cs:89-118`
    pub fn click_config(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        now: Instant,
    ) -> Option<String> {
        if !matches!(self.phase, Phase::Idle) || self.dialog.is_some() {
            return None;
        }
        if !view.connected {
            return Some(format!("{ERROR_TITLE}: {NOT_CONNECTED}"));
        }
        self.fetch(After::Dialog, telemetry, now);
        None
    }

    /// The progress dialog's Cancel: a fetch for the dialog ends with none (the dialog does not
    /// open); an update sends no more and ends when its replies are in.
    /// `// C#: ConfigOSD.cs:134-135, 192-196`
    pub fn cancel(&mut self) {
        match &mut self.phase {
            Phase::Fetching(_) => {
                self.phase = Phase::Idle;
                self.subscription = None;
            }
            Phase::Updating(update) => update.cancelled = true,
            Phase::Idle => {}
        }
    }

    /// The link's packets, for the replies.
    fn subscribe(&mut self, telemetry: &Telemetry) {
        if self
            .subscription
            .as_ref()
            .is_some_and(|sub| telemetry.carries(sub))
        {
            return;
        }
        let replies = Arc::clone(&self.replies);
        self.subscription = telemetry.on_packet(move |packet| {
            if let Some(reply) = Reply::of(&packet.message)
                && let Ok(mut held) = replies.os_lock()
            {
                held.push(reply);
            }
        });
    }

    fn take_replies(&self) -> Vec<Reply> {
        let mut held = self
            .replies
            .os_lock()
            .unwrap_or_else(PoisonError::into_inner);
        std::mem::take(&mut *held)
    }

    /// "Done && Refresh": the changed rows sent as `UpdateOsdTuningSlots` sends them; none
    /// changed, the dialog just closes.
    /// `// C#: ConfigOSD.cs:107-118`
    pub fn done(&mut self, telemetry: &Telemetry, view: &TelemetryView, now: Instant) {
        let Some(mut dialog) = self.dialog.take() else {
            return;
        };
        dialog.leave_box();
        let changes = dialog.changes();
        self.last_changes = changes.len();
        if changes.is_empty() {
            return;
        }
        if !view.connected {
            self.last_words = format!("{ERROR_TITLE}: {NOT_CONNECTED}");
            return;
        }
        self.subscribe(telemetry);
        self.phase = Phase::Updating(Update {
            changes,
            sent: 0,
            requests: Vec::new(),
            responses: 0,
            errors: Vec::new(),
            last_response: now,
            started: now,
            cancelled: false,
        });
    }

    /// The dialog's close box: nothing sent.
    pub fn close_dialog(&mut self) {
        self.dialog = None;
    }

    /// Whether an update ended, once: the page refreshes its parameters after one.
    pub fn take_refresh(&mut self) -> bool {
        std::mem::take(&mut self.refresh_due)
    }

    /// Once a frame: the requests sent 20 ms apart, the replies read, a pass or an update ended
    /// when all are in or ten seconds pass with none; what the C# boxes at the end, for the
    /// status line.
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        now: Instant,
    ) -> Option<String> {
        let target = view.vehicle?;
        let replies = if matches!(self.phase, Phase::Idle) {
            Vec::new()
        } else {
            self.take_replies()
        };
        let mut words = None;
        let mut next: Option<Phase> = None;
        match &mut self.phase {
            Phase::Idle => {}
            Phase::Fetching(fetch) => {
                for reply in replies {
                    if let Reply::Show {
                        request_id, name, ..
                    } = reply
                        && let Some(&(_, screen, index)) =
                            fetch.sent.iter().find(|(id, _, _)| *id == request_id)
                    {
                        fetch.results.push((screen, index, name));
                        fetch.answered += 1;
                        fetch.last_response = now;
                    }
                }
                if let Some(slot) = fetch.to_send.first().copied()
                    && fetch
                        .last_send
                        .is_none_or(|sent| now.duration_since(sent) >= SEND_GAP)
                {
                    fetch.request += 1;
                    let message = show_config(target, fetch.request, slot);
                    telemetry.send(&message);
                    fetch.sent.push((fetch.request, slot.0, slot.1));
                    fetch.to_send.remove(0);
                    fetch.last_send = Some(now);
                    fetch.last_response = now;
                }
                let pass_done = fetch.to_send.is_empty()
                    && (fetch.answered >= fetch.sent.len()
                        || now.duration_since(fetch.last_response) >= SILENCE);
                if pass_done {
                    // The next pass for the slots with no reply, while retries remain.
                    let unanswered: Vec<(u8, u8)> = all_slots()
                        .into_iter()
                        .filter(|(s, i)| {
                            !fetch.results.iter().any(|(rs, ri, _)| rs == s && ri == i)
                        })
                        .collect();
                    if !unanswered.is_empty() && fetch.pass < RETRIES {
                        fetch.pass += 1;
                        fetch.to_send = unanswered;
                        fetch.sent.clear();
                        fetch.answered = 0;
                        fetch.request = 0;
                        fetch.last_send = None;
                    } else {
                        let results = std::mem::take(&mut fetch.results);
                        let after = fetch.after;
                        self.names.set(results);
                        next = Some(Phase::Idle);
                        if after == After::Dialog {
                            let settings =
                                super::onboard_osd::OnboardOsd::settings_of(&view.parameters);
                            let all: Vec<String> = view
                                .parameters
                                .iter()
                                .map(|(name, _)| name.clone())
                                .collect();
                            self.dialog = Some(Dialog::new(&settings, &self.names, all));
                        }
                    }
                }
            }
            Phase::Updating(update) => {
                for reply in replies {
                    if let Reply::Set {
                        request_id,
                        success,
                    } = reply
                        && let Some(&(_, screen, index)) =
                            update.requests.iter().find(|(id, _, _)| *id == request_id)
                    {
                        update.responses += 1;
                        update.last_response = now;
                        if !success {
                            update
                                .errors
                                .push(format!("Screen {screen} Slot {index}: Update Failed"));
                        }
                    }
                }
                if !update.cancelled
                    && let Some(change) = update.changes.get(update.sent)
                {
                    let request = u32::try_from(update.sent).unwrap_or(0) + 1;
                    telemetry.send(&param_config(target, request, change));
                    update.requests.push((request, change.screen, change.index));
                    update.sent += 1;
                    update.last_response = now;
                }
                let all_sent = update.cancelled || update.sent >= update.changes.len();
                if all_sent
                    && (update.responses >= update.changes.len()
                        || now.duration_since(update.last_response) >= SILENCE)
                {
                    if update.cancelled {
                        next = Some(Phase::Idle);
                    } else {
                        let mut errors = std::mem::take(&mut update.errors);
                        if update.responses != update.changes.len() {
                            errors.push(format!(
                                "Got {} responses of {} requests",
                                update.responses,
                                update.changes.len()
                            ));
                        }
                        if !errors.is_empty() {
                            words = Some(errors.join("; "));
                        }
                        self.refresh_due = true;
                        next = Some(Phase::Idle);
                    }
                }
            }
        }
        if let Some(phase) = next {
            self.phase = phase;
            self.subscription = None;
        }
        if let Some(said) = &words {
            self.last_words.clone_from(said);
        }
        words
    }

    /// The phase, in a word, for the facts.
    #[must_use]
    pub const fn phase_word(&self) -> &'static str {
        match self.phase {
            Phase::Idle => "idle",
            Phase::Fetching(_) => "fetching",
            Phase::Updating(_) => "updating",
        }
    }

    /// The last words said, for the facts.
    #[must_use]
    pub fn last_words(&self) -> &str {
        &self.last_words
    }
}

/// `ParamShow`: `OSD_PARAM_SHOW_CONFIG` to the vehicle.
/// `// C#: Utilities/OsdTuningSlotProvider.cs:36-45`
#[must_use]
pub fn show_config(target: VehicleId, request_id: u32, (screen, index): (u8, u8)) -> MavMessage {
    MavMessage::OsdParamShowConfig(OsdParamShowConfig {
        request_id,
        target_system: target.payload_target(),
        target_component: target.compid,
        osd_screen: screen,
        osd_index: index,
    })
}

/// `ParamSet`: `OSD_PARAM_CONFIG` to the vehicle, the name as its bytes.
/// `// C#: Utilities/OsdTuningSlotProvider.cs:47-57`
#[must_use]
pub fn param_config(target: VehicleId, request_id: u32, change: &Change) -> MavMessage {
    let mut param_id = [0_u8; 16];
    for (slot, byte) in param_id.iter_mut().zip(change.param_name.bytes()) {
        *slot = byte;
    }
    #[allow(clippy::cast_possible_truncation)]
    MavMessage::OsdParamConfig(OsdParamConfig {
        request_id,
        min_value: change.min as f32,
        max_value: change.max as f32,
        increment: change.increment as f32,
        target_system: target.payload_target(),
        target_component: target.compid,
        osd_screen: change.screen,
        osd_index: change.index,
        param_id,
        config_type: change.kind,
    })
}

/// The facts the GUI scripts read.
pub fn record_facts(slots: &Slots) {
    use crate::facts::record;
    record("config.onboard_osd.slots.phase", slots.phase_word());
    record("config.onboard_osd.slots.fetched", slots.names.is_fetched());
    record("config.onboard_osd.slots.names", slots.names.listing());
    record("config.onboard_osd.slots.dialog", slots.dialog.is_some());
    record(
        "config.onboard_osd.slots.rows",
        slots.dialog.as_ref().map_or(0, |d| d.rows.len()),
    );
    record(
        "config.onboard_osd.slots.row_values",
        slots.dialog.as_ref().map_or_else(String::new, |d| {
            d.rows
                .iter()
                .map(|r| {
                    format!(
                        "{}:{}={}/{}/{}/{}/{}",
                        r.screen,
                        r.index,
                        r.name_index
                            .and_then(|i| d.names.get(i))
                            .map_or("-", String::as_str),
                        r.kind,
                        r.min,
                        r.max,
                        r.increment
                    )
                })
                .collect::<Vec<_>>()
                .join(" ")
        }),
    );
    record("config.onboard_osd.slots.changes", slots.last_changes);
    record("config.onboard_osd.slots.words", slots.last_words());
}

#[cfg(test)]
mod tests {
    use super::*;
    use mp_mavlink_dialects::all::{OsdParamConfigReply, OsdParamShowConfigReply};

    #[test]
    fn a_reply_reads_the_name_up_to_the_first_zero_and_only_a_success_has_one() {
        let mut param_id = [0_u8; 16];
        param_id[..7].copy_from_slice(b"RTL_ALT");
        let show = |result: u8, param_id: [u8; 16]| {
            Reply::of(&MavMessage::OsdParamShowConfigReply(
                OsdParamShowConfigReply {
                    request_id: 3,
                    min_value: 0.0,
                    max_value: 1.0,
                    increment: 0.1,
                    result,
                    param_id,
                    config_type: 0,
                },
            ))
        };
        assert_eq!(
            show(SUCCESS, param_id),
            Some(Reply::Show {
                request_id: 3,
                result: 0,
                name: Some("RTL_ALT".to_owned())
            })
        );
        assert_eq!(
            show(2, param_id),
            Some(Reply::Show {
                request_id: 3,
                result: 2,
                name: None
            })
        );
        // Sixteen characters and no terminator: `IndexOf` is -1, no name.
        let full = *b"ABCDEFGHIJKLMNOP";
        assert_eq!(
            show(SUCCESS, full),
            Some(Reply::Show {
                request_id: 3,
                result: 0,
                name: None
            })
        );
        assert_eq!(
            Reply::of(&MavMessage::OsdParamConfigReply(OsdParamConfigReply {
                request_id: 9,
                result: 1
            })),
            Some(Reply::Set {
                request_id: 9,
                success: false
            })
        );
    }

    fn settings() -> Vec<Setting> {
        let mut list = Vec::new();
        for screen in [5_u8, 6] {
            list.push(Setting::new(format!("OSD{screen}_ENABLE"), 0.0));
            for index in [2_u8, 1] {
                let base = format!("OSD{screen}_PARAM{index}");
                list.push(Setting::new(format!("{base}_EN"), 1.0));
                list.push(Setting::new(format!("{base}_X"), 2.0));
                list.push(Setting::new(format!("{base}_Y"), f64::from(index)));
                list.push(Setting::new(format!("{base}_TYPE"), f64::from(index - 1)));
                list.push(Setting::new(format!("{base}_MIN"), 0.0));
                list.push(Setting::new(format!("{base}_MAX"), 100.5));
                list.push(Setting::new(format!("{base}_INCR"), 0.001));
            }
        }
        list
    }

    #[test]
    fn the_dialog_has_a_row_a_slot_in_screen_and_index_order_with_the_initial_values() {
        let mut names = SlotNames::default();
        names.set(vec![(5, 1, Some("RTL_ALT".to_owned())), (6, 2, None)]);
        let dialog = Dialog::new(
            &settings(),
            &names,
            vec![
                "WPNAV_SPEED".to_owned(),
                "RTL_ALT".to_owned(),
                "ACRO_RP_P".to_owned(),
            ],
        );
        assert_eq!(*dialog.names, ["ACRO_RP_P", "RTL_ALT", "WPNAV_SPEED"]);
        assert_eq!(dialog.screens(), [5, 6]);
        let order: Vec<(u8, u8)> = dialog.rows.iter().map(|r| (r.screen, r.index)).collect();
        assert_eq!(order, [(5, 1), (5, 2), (6, 1), (6, 2)]);
        let first = &dialog.rows[0];
        assert_eq!(first.name_index, Some(1));
        assert_eq!(
            (
                first.kind,
                first.min.as_str(),
                first.max.as_str(),
                first.increment.as_str()
            ),
            (0, "0", "100.5", "0.001")
        );
        assert!(first.numbers_enabled());
        let second = &dialog.rows[1];
        assert_eq!(second.name_index, None);
        assert_eq!(second.kind, 1);
        assert!(!second.numbers_enabled());
        assert!(dialog.changes().is_empty(), "nothing dirty yet");
        assert_eq!(five_places(0.000_001), "0");
        assert_eq!(five_places(-2.5), "-2.5");
        assert_eq!(five_places(3.0), "3");
    }

    #[test]
    fn a_row_is_dirty_by_the_type_s_rule_and_makes_its_change() {
        let mut names = SlotNames::default();
        names.set(vec![(5, 1, Some("RTL_ALT".to_owned()))]);
        let mut dialog = Dialog::new(
            &settings(),
            &names,
            vec!["RTL_ALT".to_owned(), "ACRO_RP_P".to_owned()],
        );
        // Type None: a number typed makes it dirty.
        dialog.begin_box(0, Box_::Max);
        dialog.field.set("200");
        dialog.leave_box();
        let changes = dialog.changes();
        assert_eq!(changes.len(), 1);
        assert_eq!(
            changes[0],
            Change {
                screen: 5,
                index: 1,
                param_name: "RTL_ALT".to_owned(),
                kind: 0,
                min: 0.0,
                max: 200.0,
                increment: 0.001
            }
        );
        // Another type: the numbers are ignored, the name and type compared.
        dialog.choose(1, ComboKind::Type, 2);
        let changes = dialog.changes();
        assert_eq!(changes.len(), 2);
        assert_eq!(
            (
                changes[1].kind,
                changes[1].min,
                changes[1].param_name.as_str()
            ),
            (2, 0.0, "")
        );
        dialog.choose(1, ComboKind::Type, 1);
        assert_eq!(dialog.changes().len(), 1, "back to its start");
        // A box of a typed row does not open.
        dialog.begin_box(1, Box_::Min);
        assert_eq!(dialog.typing, None);
        // The name chosen from the list.
        dialog.toggle_combo(2, ComboKind::Name);
        assert_eq!(dialog.open, Some((2, ComboKind::Name)));
        dialog.choose(2, ComboKind::Name, 0);
        assert_eq!(dialog.rows[2].name_index, Some(0));
        assert_eq!(dialog.changes().len(), 2);
        assert!(allowed_number_text("-1.5"));
        assert!(!allowed_number_text("1.5.2"));
        assert!(!allowed_number_text("1-"));
        assert!(!allowed_number_text("abc"));
    }

    #[test]
    fn the_messages_carry_the_slot_the_request_and_the_name_bytes() {
        let target = VehicleId::new(1, 1);
        let MavMessage::OsdParamShowConfig(show) = show_config(target, 7, (6, 3)) else {
            panic!("a show config")
        };
        assert_eq!(
            (show.request_id, show.osd_screen, show.osd_index),
            (7, 6, 3)
        );
        let MavMessage::OsdParamConfig(set) = param_config(
            target,
            2,
            &Change {
                screen: 5,
                index: 1,
                param_name: "RTL_ALT".to_owned(),
                kind: 0,
                min: 1.0,
                max: 2.0,
                increment: 0.5,
            },
        ) else {
            panic!("a config")
        };
        assert_eq!(&set.param_id[..8], b"RTL_ALT\0");
        assert_eq!(
            (set.min_value, set.max_value, set.increment, set.config_type),
            (1.0, 2.0, 0.5, 0)
        );
    }

    #[test]
    fn a_fetch_asks_the_eighteen_slots_and_retries_the_unanswered_once() {
        let telemetry = Telemetry::idle();
        let mut view = TelemetryView::disconnected("test");
        view.connected = true;
        view.vehicle = Some(VehicleId::new(1, 1));
        let mut slots = Slots::default();
        let start = Instant::now();
        slots.want_names(&telemetry, &view, start);
        assert_eq!(slots.phase_word(), "fetching");
        // Eighteen sends, 20 ms apart.
        let mut now = start;
        for _ in 0..18 {
            now += SEND_GAP;
            slots.tick(&telemetry, &view, now);
        }
        let Phase::Fetching(fetch) = &slots.phase else {
            panic!("fetching")
        };
        assert_eq!(fetch.sent.len(), 18);
        assert!(fetch.to_send.is_empty());
        // Two answered, one of them refused; ten seconds of silence end the pass; the second
        // pass asks the sixteen again.
        slots.replies.os_lock().unwrap().extend([
            Reply::Show {
                request_id: 1,
                result: 0,
                name: Some("RTL_ALT".to_owned()),
            },
            Reply::Show {
                request_id: 10,
                result: 1,
                name: None,
            },
        ]);
        now += Duration::from_millis(10);
        slots.tick(&telemetry, &view, now);
        now += SILENCE;
        slots.tick(&telemetry, &view, now);
        let Phase::Fetching(fetch) = &slots.phase else {
            panic!("the retry")
        };
        assert_eq!(fetch.pass, 1);
        assert_eq!(fetch.to_send.len(), 16);
        assert!(!fetch.to_send.contains(&(5, 1)));
        assert!(!fetch.to_send.contains(&(6, 1)), "a refusal is an answer");
        for _ in 0..16 {
            now += SEND_GAP;
            slots.tick(&telemetry, &view, now);
        }
        now += SILENCE;
        slots.tick(&telemetry, &view, now);
        assert_eq!(slots.phase_word(), "idle");
        assert!(slots.names().is_fetched());
        assert_eq!(slots.names().name(5, 1), Some("RTL_ALT"));
        assert_eq!(slots.names().name(6, 1), None);
        assert_eq!(slots.names().listing(), "5:1=RTL_ALT");
        // Fetched: not asked again.
        slots.want_names(&telemetry, &view, now);
        assert_eq!(slots.phase_word(), "idle");
        // Disconnected: nothing to ask, the names empty.
        let mut cold = Slots::default();
        let mut off = TelemetryView::disconnected("test");
        off.vehicle = Some(VehicleId::new(1, 1));
        cold.want_names(&telemetry, &off, now);
        assert!(cold.names().is_fetched());
        assert_eq!(
            cold.click_config(&telemetry, &off, now).as_deref(),
            Some("Error: Your are not connected")
        );
    }

    #[test]
    fn an_update_sends_the_changes_counts_the_replies_and_says_what_failed() {
        let telemetry = Telemetry::idle();
        let mut view = TelemetryView::disconnected("test");
        view.connected = true;
        view.vehicle = Some(VehicleId::new(1, 1));
        view.parameters = vec![("RTL_ALT".to_owned(), 1.0)].into();
        let mut slots = Slots::default();
        let mut names = SlotNames::default();
        names.set(vec![(5, 1, Some("RTL_ALT".to_owned()))]);
        slots.names = names;
        let mut dialog = Dialog::new(
            &settings(),
            slots.names(),
            vec!["RTL_ALT".to_owned(), "ACRO_RP_P".to_owned()],
        );
        dialog.choose(0, ComboKind::Name, 0);
        dialog.choose(2, ComboKind::Type, 3);
        slots.dialog = Some(dialog);
        let start = Instant::now();
        slots.done(&telemetry, &view, start);
        assert_eq!(slots.phase_word(), "updating");
        assert_eq!(slots.last_changes, 2);
        let mut now = start;
        for _ in 0..2 {
            now += Duration::from_millis(1);
            slots.tick(&telemetry, &view, now);
        }
        let Phase::Updating(update) = &slots.phase else {
            panic!("updating")
        };
        assert_eq!(update.sent, 2);
        assert_eq!(
            update.requests.iter().map(|r| r.0).collect::<Vec<_>>(),
            [1, 2]
        );
        slots.replies.os_lock().unwrap().push(Reply::Set {
            request_id: 2,
            success: false,
        });
        now += Duration::from_millis(1);
        assert_eq!(slots.tick(&telemetry, &view, now), None);
        now += SILENCE;
        let words = slots.tick(&telemetry, &view, now);
        assert_eq!(
            words.as_deref(),
            Some("Screen 6 Slot 1: Update Failed; Got 1 responses of 2 requests")
        );
        assert_eq!(slots.phase_word(), "idle");
        assert!(slots.take_refresh());
        assert!(!slots.take_refresh());
        // Nothing changed: the dialog just closes.
        slots.dialog = Some(Dialog::new(
            &settings(),
            slots.names(),
            vec!["RTL_ALT".to_owned()],
        ));
        slots.done(&telemetry, &view, now);
        assert!(slots.dialog.is_none());
        assert_eq!(slots.phase_word(), "idle");
        assert!(!slots.take_refresh());
    }
}
