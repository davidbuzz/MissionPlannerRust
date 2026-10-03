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

//! Frame Type for a copter older than 3.5: `GCSViews/ConfigurationView/ConfigFrameType.cs`, the
//! page Initial Setup lists under Mandatory Hardware for `isCopter && gotAllParams &&
//! !isCopter35plus` (`GCSViews/InitialSetup.cs:184-192`), where a newer vehicle gets
//! `ConfigFrameClassType` ([`crate::config::frame_type`]). Before 3.5 ArduCopter had one parameter,
//! `FRAME`, for the motor layout: 0 Plus, 1 X, 2 V, 3 H, 4 V Tail, 10 Y6B (`ArduPilot.Frame`).
//!
//! What it shows, as the `.resx` places it in a 787 x 426 page: "Frame Type", a group of six rows,
//! each a label, a radio button and a picture of the layout, the picture brightened for the frame
//! chosen and faded for the rest, with two notes; and beside it "Default Settings", the
//! `DefaultSettings` control.
//!
//! `Activate` disables the page for a vehicle without `FRAME`; otherwise it runs `DoChange` on
//! `FRAME`'s value, which checks that frame's radio button, brightens its picture and fades the
//! others, and writes the value back - which `setParam`'s "not modified as same" turns into
//! nothing on the wire. A radio button or a picture clicked runs `DoChange` for its frame, which
//! writes `FRAME` at once, with no confirmation; a write that times out says "Set FRAME Failed".
//!
//! The radio buttons are WinForms', and so are their events: checking one unchecks the others in
//! the group first, and each of those raises `CheckedChanged`, whose handler runs `DoChange` for
//! its own frame. `DoChange`'s `indochange` stops the calls it causes itself, but not the one a
//! click causes before it starts: clicking X while Plus is checked unchecks Plus, whose handler
//! runs `DoChange(Plus)` - checking Plus again and writing 0 - and then X's runs `DoChange(X)` and
//! writes 1. [`FrameTypeLegacy`] runs the same events in the same order
//! (`RadioButton.Checked`'s setter, `PerformAutoUpdates` over `groupBox2.Controls`), so the writes
//! are the C#'s: the frame that was checked, which the vehicle already holds, and then the one
//! clicked. A picture's click calls `DoChange` directly and writes its frame alone.
//!
//! What the C# does on the UI thread - `setParam`, blocking until the vehicle echoes the value -
//! is a queue here, advanced once a frame, one write at a time in the same order.
//!
//! The Default Settings group is `Controls/DefaultSettings.cs`, a control of its own
//! ([`super::default_settings`]): on `Load` it lists ArduPilot's `Tools/Frame_params` from GitHub,
//! and Load Params fetches the file chosen and opens `ParamCompare` over it; the form closed, the
//! control's `OnChange` runs this page's `Activate` again (`ConfigFrameType.cs:22, 41-44`).
//!
//! "Set FRAME Failed", the C#'s error box for a write that throws, is said on the status line
//! instead, by the owner's ruling of 2026-09-25 that an error the window can show as state gets
//! no message box.
//!
//! The pictures are the C#'s ([`crate::pictures`], resources named in [`ROWS`]), each zoomed in
//! its box at the opacity the C# gives it; V Tail's box has no image in the C# and is empty here.
//!
//! What is not ported, and why:
//!
//! * the fade, a 400 ms linear `Transition` (`ConfigFrameType.cs:169-174`): drawn at the opacity
//!   it ends on;
//! * `MainV2.comPort.giveComport = false` in `Deactivate`: the flag that stops the C#'s own reader
//!   while a blocking call reads the port; this application's link reads on its own thread.
//!
//! The colours are this application's.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;

use gpui::{AnyElement, Context, Div, SharedString, Window, div, prelude::*, px, rgb};
use mp_link::requests::RequestOutcome;

use crate::MissionPlanner;
use crate::config::default_settings::DefaultSettings;
use crate::config::flight_modes::{ParamWriter, Progress};
use crate::config::optional::{button, message_box};
use crate::config::param_compare;
use crate::config::servo_output::{
    Combo, ERROR_TITLE, Message, combo_box, dropdown, modal, value_of,
};
use crate::setup::Key;
use crate::telemetry::TelemetryView;
use crate::ui::{action, panel, theme};

/// The page's title in Initial Setup's list.
/// `// C#: GCSViews/InitialSetup.cs:188`
pub const TITLE: &str = "Frame Type";

/// The parameter the page reads and writes.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:27-33, 160`
pub const PARAM: &str = "FRAME";

/// `DisabledOpacity`.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:13`
pub const DISABLED_OPACITY: f32 = 0.2;
/// `EnabledOpacity`.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:14`
pub const ENABLED_OPACITY: f32 = 1.0;

/// What `SetFrameParam` shows when the write throws: `Strings.ErrorSetValueFailed` with "FRAME",
/// under `Strings.ERROR`, with the warning icon.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:156-167; ExtLibs/Strings/Strings.resx:171-173`
pub const FAILED: &str = "Set FRAME Failed";

/// `groupBox2.Text` and `groupBox1.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameType.resx groupBox2.Text, groupBox1.Text`
pub const TYPE_GROUP: &str = "Frame Type";
/// `groupBox1.Text`.
pub const DEFAULTS_GROUP: &str = "Default Settings";

/// `label4.Text`, beside the H row.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameType.resx label4.Text`
pub const NOTE_H: &str = "NOTE: X and H are NOT interchangable the prop rotation changes";
/// `label7.Text`, beside the Y6B row.
pub const NOTE_Y6B: &str = "NOTE: This is the Y6B and prop rotation changes from the old Y6A";

/// `DefaultSettings`' `textBox1.Text`, `CMB_paramfiles.Text` and `BUT_paramfileload.Text`.
/// `// C#: Controls/DefaultSettings.resx textBox1.Text, CMB_paramfiles.Text, BUT_paramfileload.Text`
pub const DEFAULTS_TEXT: &str = "Load the default settings for standard frame types";
/// `CMB_paramfiles.Text`.
pub const DEFAULTS_LOADING: &str = "Loading";
/// `BUT_paramfileload.Text`.
pub const DEFAULTS_LOAD: &str = "Load Params";

/// A control's `Location` and `Size`.
pub type Place = (f32, f32, f32, f32);

/// `groupBox2`, in the page.
pub const TYPE_GROUP_AT: Place = (3.0, 3.0, 504.0, 420.0);
/// `groupBox1`, in the page.
pub const DEFAULTS_GROUP_AT: Place = (513.0, 3.0, 263.0, 112.0);
/// `configDefaultSettings1`, in `groupBox1`.
pub const DEFAULTS_AT: Place = (6.0, 18.0, 255.0, 89.0);
/// `textBox1`, in `DefaultSettings`: multiline and read-only.
pub const DEFAULTS_TEXT_AT: Place = (5.0, 3.0, 246.0, 34.0);
/// `CMB_paramfiles`, in `DefaultSettings`.
pub const DEFAULTS_COMBO_AT: Place = (49.0, 43.0, 147.0, 21.0);
/// `BUT_paramfileload`, in `DefaultSettings`.
pub const DEFAULTS_BUTTON_AT: Place = (72.0, 70.0, 103.0, 21.0);
/// `label4` and `label7`, in `groupBox2`.
pub const NOTE_H_AT: Place = (200.0, 272.0, 131.0, 43.0);
/// `label7`.
pub const NOTE_Y6B_AT: Place = (200.0, 356.0, 142.0, 43.0);

/// A frame the page offers: `radioButton_<name>`, its label and its `PictureBoxWithPseudoOpacity`,
/// all in `groupBox2`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Row {
    /// The `ArduPilot.Frame` value, as `FRAME` holds it.
    pub frame: i32,
    /// The controls' name, after `radioButton_` and `pictureBox`.
    pub name: &'static str,
    /// The label's name and text.
    pub label: (&'static str, &'static str),
    /// The label's `Location`.
    pub label_at: (f32, f32),
    /// The radio button's `Location`; each is 14 by 13.
    pub radio_at: (f32, f32),
    /// The picture's `Location` and `Size`.
    pub picture: Place,
    /// The resource the C# draws there ([`crate::pictures`]); V Tail's has none.
    pub image: Option<&'static str>,
}

/// The six frames, in `DoChange`'s order.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:53-152; ConfigFrameType.Designer.cs:66-244 (pictureBox*.Image: 97, 106, 128, 150, 177); ConfigFrameType.resx; ExtLibs/ArduPilot/Frame.cs:4-12`
pub const ROWS: [Row; 6] = [
    Row {
        frame: 0,
        name: "Plus",
        label: ("label2", "'Plus'"),
        label_at: (25.0, 38.0),
        radio_at: (62.0, 38.0),
        picture: (83.0, 9.0, 248.0, 75.0),
        image: Some("frames_plus"),
    },
    Row {
        frame: 1,
        name: "X",
        label: ("label3", "'X' , 'Y6A'"),
        label_at: (8.0, 118.0),
        radio_at: (62.0, 118.0),
        picture: (83.0, 88.0, 406.0, 78.0),
        image: Some("frames_x"),
    },
    Row {
        frame: 2,
        name: "V",
        label: ("label6", "'V'"),
        label_at: (38.0, 203.0),
        radio_at: (62.0, 203.0),
        picture: (83.0, 173.0, 111.0, 78.0),
        image: Some("new_3DR_04"),
    },
    Row {
        frame: 3,
        name: "H",
        label: ("label1", "'H'"),
        label_at: (38.0, 287.0),
        radio_at: (62.0, 287.0),
        picture: (83.0, 257.0, 111.0, 78.0),
        image: Some("frames_h"),
    },
    Row {
        frame: 10,
        name: "Y",
        label: ("label5", "'Y6B'"),
        label_at: (19.0, 371.0),
        radio_at: (62.0, 371.0),
        picture: (83.0, 341.0, 111.0, 77.0),
        image: Some("y6b"),
    },
    Row {
        frame: 4,
        name: "VTail",
        label: ("label9", "'V Tail'"),
        label_at: (313.0, 203.0),
        radio_at: (357.0, 203.0),
        picture: (378.0, 173.0, 111.0, 78.0),
        image: None,
    },
];

/// Where each radio button is in `groupBox2.Controls`, the order `PerformAutoUpdates` unchecks
/// the others in: V Tail, Plus, X, Y, V, H, as the Designer adds them.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameType.Designer.cs:196-216`
pub const CONTROLS_ORDER: [i32; 6] = [4, 0, 1, 10, 2, 3];

/// The order `DoChange` sets the radio buttons in each case: V Tail, Plus, V, X, H, Y.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:62-67`
pub const DO_CHANGE_ORDER: [i32; 6] = [4, 0, 2, 1, 3, 10];

/// The radio buttons `DoChange`'s `default` unchecks: all but V Tail.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:145-151`
pub const DEFAULT_UNCHECKS: [i32; 5] = [0, 2, 1, 3, 10];

/// Where a frame's row is in [`ROWS`].
fn row_of(frame: i32) -> Option<usize> {
    ROWS.iter().position(|row| row.frame == frame)
}

/// `FRAME`'s value as `Enum.Parse(typeof(Frame), param.ToString())` reads it: a whole number
/// parses to that value whether or not `Frame` names it; anything else throws.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:33; ExtLibs/Mavlink/MAVLinkParam.cs:219-224`
fn parse(value: f64) -> Option<i32> {
    let whole = value.is_finite()
        && value.fract() == 0.0
        && value >= f64::from(i32::MIN)
        && value <= f64::from(i32::MAX);
    #[allow(clippy::cast_possible_truncation)] // checked whole and in range above
    whole.then_some(value as i32)
}

/// `SetFrameParam`'s writes, one at a time and in order, as the C#'s blocking `setParam` calls go.
#[derive(Debug)]
pub struct Writes<H> {
    queued: VecDeque<f64>,
    waiting: Option<(f64, H)>,
    last: Option<String>,
    sent: usize,
}

impl<H> Default for Writes<H> {
    fn default() -> Self {
        Self {
            queued: VecDeque::new(),
            waiting: None,
            last: None,
            sent: 0,
        }
    }
}

impl<H: Copy> Writes<H> {
    /// How many writes are still to be answered.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.queued.len() + usize::from(self.waiting.is_some())
    }

    /// The values written, and to be written, in order: the one waiting first.
    #[cfg(test)]
    #[must_use]
    pub fn values(&self) -> Vec<f64> {
        self.waiting
            .map(|(value, _)| value)
            .into_iter()
            .chain(self.queued.iter().copied())
            .collect()
    }

    /// Moves the writes on: collects an answered one and starts the next. A timeout or a refusal
    /// is the C#'s exception, and its message box; a parameter the vehicle lacks is `setParam`'s
    /// false, and says nothing.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:156-167; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1640-1651, 1765`
    fn advance<W: ParamWriter<Handle = H>>(
        &mut self,
        writer: &W,
        messages: &mut VecDeque<Message>,
    ) {
        loop {
            if let Some((value, handle)) = self.waiting {
                let failed = match writer.progress(handle) {
                    Progress::Waiting => return,
                    Progress::Lost => Some("lost".to_owned()),
                    Progress::Finished(outcome) => match outcome {
                        RequestOutcome::Accepted { value: echoed } => {
                            let echoed = echoed.map_or(value, |echoed| echoed.as_f64());
                            self.last = Some(format!("{PARAM} {echoed} accepted"));
                            None
                        }
                        RequestOutcome::Unchanged | RequestOutcome::Sent => {
                            self.last = Some(format!("{PARAM} {value} unchanged"));
                            None
                        }
                        RequestOutcome::UnknownParameter => {
                            self.last = Some(format!("{PARAM} {value} not on the vehicle"));
                            None
                        }
                        RequestOutcome::TimedOut => Some("timed out".to_owned()),
                        RequestOutcome::Rejected(result) => Some(format!("rejected {result}")),
                    },
                };
                self.waiting = None;
                if let Some(why) = failed {
                    self.fail(value, &why, messages);
                }
            }
            let Some(value) = self.queued.pop_front() else {
                return;
            };
            match writer.write(PARAM, value) {
                Some(handle) => {
                    self.sent += 1;
                    self.waiting = Some((value, handle));
                }
                None => self.fail(value, "no vehicle", messages),
            }
        }
    }

    /// A write that threw: the message box.
    fn fail(&mut self, value: f64, why: &str, messages: &mut VecDeque<Message>) {
        self.last = Some(format!("{PARAM} {value} failed: {why}"));
        messages.push_back(Message {
            title: ERROR_TITLE,
            text: FAILED.to_owned(),
        });
    }
}

/// The page object and what its controls show.
#[derive(Debug)]
pub struct FrameTypeLegacy<H = mp_link::RequestId> {
    /// The screen the page object belongs to; a different one is a new object.
    made_for: Option<Key>,
    /// Between `Activate` and `Deactivate`.
    active: bool,
    /// The page's `Enabled`: false once `Activate` finds no `FRAME`.
    enabled: bool,
    /// What `Activate` last read, as `Enum.Parse` read it.
    read: Option<i32>,
    /// Each radio button's `Checked`, in [`ROWS`]' order.
    checked: [bool; 6],
    /// Whether each picture is at [`ENABLED_OPACITY`] rather than [`DISABLED_OPACITY`].
    bright: [bool; 6],
    /// `indochange`.
    in_do_change: bool,
    /// `SetFrameParam`'s writes.
    writes: Writes<H>,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// `configDefaultSettings1`, the Default Settings group's control.
    defaults: DefaultSettings<H>,
}

impl<H> Default for FrameTypeLegacy<H> {
    /// As the Designer leaves it: enabled, no radio button checked, every picture opaque.
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            enabled: true,
            read: None,
            checked: [false; 6],
            bright: [true; 6],
            in_do_change: false,
            writes: Writes::default(),
            messages: VecDeque::new(),
            defaults: DefaultSettings::default(),
        }
    }
}

impl<H: Copy> FrameTypeLegacy<H> {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// The page's `Enabled`.
    #[must_use]
    pub const fn enabled(&self) -> bool {
        self.enabled
    }

    /// The frame whose radio button is checked, if one is.
    #[must_use]
    pub fn checked(&self) -> Vec<i32> {
        ROWS.iter()
            .zip(self.checked)
            .filter(|(_, on)| *on)
            .map(|(row, _)| row.frame)
            .collect()
    }

    /// The frames whose pictures are bright.
    #[must_use]
    pub fn bright(&self) -> Vec<i32> {
        ROWS.iter()
            .zip(self.bright)
            .filter(|(_, on)| *on)
            .map(|(row, _)| row.frame)
            .collect()
    }

    /// The writes.
    #[must_use]
    pub const fn writes(&self) -> &Writes<H> {
        &self.writes
    }

    /// The message box showing, if one is.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// Dismisses it.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// The Default Settings control.
    #[must_use]
    pub const fn defaults(&self) -> &DefaultSettings<H> {
        &self.defaults
    }

    /// The Default Settings control, for its clicks.
    pub const fn defaults_mut(&mut self) -> &mut DefaultSettings<H> {
        &mut self.defaults
    }

    /// The C#'s error boxes - "Set FRAME Failed", and the Default Settings control's - taken out
    /// of their queues: the status line's words for the last of them (the module's notes).
    pub fn take_link_errors(&mut self) -> Option<String> {
        let mut status = None;
        self.messages.retain(|message| {
            if message.title == ERROR_TITLE {
                status = Some(message.text.clone());
                false
            } else {
                true
            }
        });
        self.defaults.take_link_errors().or(status)
    }

    /// Starts the fetch the Default Settings control has asked for.
    pub fn dispatch(&mut self) {
        self.defaults.dispatch();
    }

    /// `Activate`, on a new page object if this screen has none: disables the page for a vehicle
    /// without `FRAME`; otherwise `DoChange` on its value. A value that is not a whole number stops
    /// it where `Enum.Parse` throws, and `BackstageView` shows the page as it is.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:25-34; ExtLibs/Controls/BackstageView/BackstageView.cs:495-503`
    pub fn activate(&mut self, parameters: &[(String, f64)], key: Key) {
        if self.made_for != Some(key) {
            *self = Self {
                writes: std::mem::take(&mut self.writes),
                messages: std::mem::take(&mut self.messages),
                ..Self::default()
            };
            self.made_for = Some(key);
        }
        self.active = true;
        // The Default Settings control's `Load`, the first time the page shows - whether or not
        // the page is then disabled. `// C#: Controls/DefaultSettings.cs:103-106`
        self.defaults.load();
        let Some(value) = value_of(parameters, PARAM) else {
            self.enabled = false;
            return;
        };
        let Some(frame) = parse(value) else {
            return;
        };
        self.read = Some(frame);
        self.do_change(frame);
    }

    /// `Deactivate`: hidden; the writes carry on.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:36-39`
    pub fn deactivate(&mut self) {
        self.active = false;
    }

    /// `DoChange`: for one of the six, brightens its picture alone, checks its radio button and
    /// unchecks the rest, and writes it; for any other value unchecks five of the six, all but
    /// V Tail, and writes nothing. Ignored while it runs already.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:46-154`
    fn do_change(&mut self, frame: i32) {
        if self.in_do_change {
            return;
        }
        self.in_do_change = true;
        match row_of(frame) {
            Some(index) => {
                self.bright = std::array::from_fn(|at| at == index);
                for radio in DO_CHANGE_ORDER {
                    self.set_checked(radio, radio == frame);
                }
                self.writes.queued.push_back(f64::from(frame));
            }
            None => {
                for radio in DEFAULT_UNCHECKS {
                    self.set_checked(radio, false);
                }
            }
        }
        self.in_do_change = false;
    }

    /// `RadioButton.Checked`'s setter: a change unchecks the other checked buttons in the group
    /// first (`PerformAutoUpdates`), then raises `CheckedChanged`, whose handler - every one of
    /// the six - runs `DoChange` for its frame.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:176-234`
    fn set_checked(&mut self, frame: i32, value: bool) {
        let Some(index) = row_of(frame) else {
            return;
        };
        if self.checked.get(index).copied() == Some(value) {
            return;
        }
        if let Some(checked) = self.checked.get_mut(index) {
            *checked = value;
        }
        if value {
            for other in CONTROLS_ORDER {
                let on = row_of(other).and_then(|at| self.checked.get(at).copied());
                if other != frame && on == Some(true) {
                    self.set_checked(other, false);
                }
            }
        }
        self.do_change(frame);
    }

    /// A radio button clicked: `AutoCheck` checks it. The checked one raises nothing.
    pub fn click_radio(&mut self, frame: i32) {
        if self.active && self.enabled {
            self.set_checked(frame, true);
        }
    }

    /// A picture clicked: its `Click` handler runs `DoChange` for its frame, checked or not.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:186-234`
    pub fn click_picture(&mut self, frame: i32) {
        if self.active && self.enabled && row_of(frame).is_some() {
            self.do_change(frame);
        }
    }

    /// Once a frame: a page object whose screen has gone is disposed, and the writes move on.
    pub fn tick<W: ParamWriter<Handle = H>>(
        &mut self,
        writer: &W,
        view: &TelemetryView,
        on_setup: bool,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            *self = Self {
                writes: std::mem::take(&mut self.writes),
                messages: std::mem::take(&mut self.messages),
                ..Self::default()
            };
        }
        self.writes.advance(writer, &mut self.messages);
        // `configDefaultSettings1_OnChange`: `Activate` again, which reads `FRAME` as the form
        // left it. `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:41-44`
        self.defaults.tick(writer, &view.parameters);
        if self.defaults.take_changed()
            && self.active
            && let Some(key) = self.made_for
        {
            self.activate(&view.parameters, key);
            self.writes.advance(writer, &mut self.messages);
        }
    }
}

/// Facts a UI test asserts on: whether the page shows, what it read, which radio buttons are
/// checked and which pictures bright, the writes, and `FRAME` as `params.value.FRAME`.
pub fn record_facts(frame: &FrameTypeLegacy, view: &TelemetryView) {
    use crate::facts::record;
    let joined = |values: Vec<i32>| {
        if values.is_empty() {
            "none".to_owned()
        } else {
            values
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(",")
        }
    };
    record("config.framelegacy.active", frame.is_active());
    record("config.framelegacy.enabled", frame.enabled());
    record(
        "config.framelegacy.read",
        frame
            .read
            .map_or_else(|| "none".to_owned(), |value| value.to_string()),
    );
    record("config.framelegacy.checked", joined(frame.checked()));
    record("config.framelegacy.bright", joined(frame.bright()));
    record("config.framelegacy.pending", frame.writes().pending());
    record("config.framelegacy.sent", frame.writes().sent);
    record(
        "config.framelegacy.write",
        frame.writes().last.as_deref().unwrap_or("none"),
    );
    record(
        "config.framelegacy.message",
        frame
            .message()
            .map_or("none", |message| message.text.as_str()),
    );
    // The Default Settings control: the combo box's text - "Loading" until the listing comes -
    // how many files it lists, whether its controls take clicks, `ParamCompare` and its writes.
    let defaults = frame.defaults();
    record(
        "config.framelegacy.defaults",
        if defaults.listed() {
            defaults.combo().text()
        } else {
            DEFAULTS_LOADING
        },
    );
    record("config.framelegacy.defaults.loaded", defaults.loaded());
    record(
        "config.framelegacy.defaults.files",
        defaults.combo().options.len(),
    );
    record(
        "config.framelegacy.defaults.enabled",
        frame.enabled() && defaults.button_enabled(),
    );
    record(
        "config.framelegacy.defaults.error",
        defaults.listing_error().unwrap_or("none"),
    );
    record(
        "config.framelegacy.defaults.compare",
        defaults
            .compare()
            .map_or_else(|| "none".to_owned(), |form| form.rows().len().to_string()),
    );
    record(
        "config.framelegacy.defaults.write",
        defaults.last_write().unwrap_or("none"),
    );
    record(
        "config.framelegacy.defaults.message",
        defaults
            .message()
            .map_or("none", |message| message.text.as_str()),
    );
    if let Some(value) = value_of(&view.parameters, PARAM) {
        record(format!("params.value.{PARAM}"), value);
    }
}

/// An absolutely placed box.
fn at((x, y, width, height): Place) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// A `GroupBox`: a border with its caption.
fn group(place: Place, title: &'static str, enabled: bool) -> Div {
    at(place)
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_sm()
        .child(
            div()
                .absolute()
                .left(px(6.0))
                .top(px(-8.0))
                .px_1()
                .bg(rgb(theme::PANEL))
                .text_xs()
                .text_color(rgb(if enabled { theme::DIM } else { theme::BORDER }))
                .child(title),
        )
}

/// Text at a place, wrapped to its width; dimmed with the page.
fn text(place: Place, words: &'static str, enabled: bool) -> Div {
    at(place)
        .text_xs()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(words)
}

/// One row: its label, its radio button and its picture.
fn row(
    frame: &FrameTypeLegacy,
    index: usize,
    option: Row,
    cx: &mut Context<MissionPlanner>,
) -> [AnyElement; 3] {
    let enabled = frame.enabled();
    let value = option.frame;
    let checked = frame.checked.get(index).copied().unwrap_or(false);
    let bright = frame.bright.get(index).copied().unwrap_or(true);

    let label = text(
        (option.label_at.0, option.label_at.1, 60.0, 13.0),
        option.label.1,
        enabled,
    )
    .into_any_element();

    let id = format!("framelegacy-radio-{value}");
    let radio = crate::probe::measured(
        id.clone(),
        at((option.radio_at.0, option.radio_at.1, 14.0, 13.0)),
    )
    .id(SharedString::from(id))
    .child(
        div()
            .size(px(13.0))
            .flex()
            .items_center()
            .justify_center()
            .rounded_full()
            .border_1()
            .border_color(rgb(if enabled {
                theme::ACCENT
            } else {
                theme::BORDER
            }))
            .bg(rgb(theme::BG))
            .children(checked.then(|| {
                div().size(px(5.0)).rounded_full().bg(rgb(if enabled {
                    theme::ACCENT
                } else {
                    theme::DIM
                }))
            })),
    );
    let radio = if enabled {
        radio
            .cursor_pointer()
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.frame_type_legacy.click_radio(value);
                cx.notify();
            }))
            .into_any_element()
    } else {
        radio.into_any_element()
    };

    let id = format!("framelegacy-picture-{value}");
    // Each `PictureBoxWithPseudoOpacity`'s `Image`, zoomed; V Tail's has none and is empty.
    // C#: GCSViews/ConfigurationView/ConfigFrameType.Designer.cs:97, 106, 128, 150, 177;
    // ConfigFrameType.resx (pictureBox*.SizeMode)
    let drawn = option
        .image
        .and_then(|resource| crate::pictures::image(resource, crate::pictures::Layout::ZoomImage));
    crate::pictures::record("framelegacy", &id, option.image.filter(|_| drawn.is_some()));
    let content = match (drawn, option.image) {
        (Some(image), _) => image,
        (None, Some(_)) => div()
            .size_full()
            .flex()
            .items_center()
            .justify_center()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .rounded_sm()
            .bg(rgb(theme::BG))
            .text_xs()
            .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
            .child(option.name)
            .into_any_element(),
        (None, None) => div().size_full().into_any_element(),
    };
    let picture = crate::probe::measured(id.clone(), at(option.picture))
        .id(SharedString::from(id))
        .opacity(if bright {
            ENABLED_OPACITY
        } else {
            DISABLED_OPACITY
        })
        .child(content);
    let picture = if enabled {
        picture
            .cursor_pointer()
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.frame_type_legacy.click_picture(value);
                cx.notify();
            }))
            .into_any_element()
    } else {
        picture.into_any_element()
    };
    [label, radio, picture]
}

/// The page, laid out as `ConfigFrameType.resx` lays it out, while it is showing.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameType.Designer.cs:31-222; Controls/DefaultSettings.Designer.cs:29-66`
pub fn page(frame: &FrameTypeLegacy, cx: &mut Context<MissionPlanner>) -> Option<AnyElement> {
    if !frame.is_active() {
        return None;
    }
    let enabled = frame.enabled();
    // `$this.Size` is 787 x 426.
    let mut body = div().relative().w(px(787.0)).h(px(426.0));

    let mut types = group(TYPE_GROUP_AT, TYPE_GROUP, enabled);
    for (index, option) in ROWS.into_iter().enumerate() {
        types = types.children(row(frame, index, option, cx));
    }
    types =
        types
            .child(text(NOTE_H_AT, NOTE_H, enabled))
            .child(text(NOTE_Y6B_AT, NOTE_Y6B, enabled));

    // `DefaultSettings`: its note, the combo box - "Loading", disabled, until the listing comes,
    // then the names listed - and Load Params; all disabled with the page.
    // `// C#: Controls/DefaultSettings.Designer.cs:29-66; Controls/DefaultSettings.resx`
    let settings = frame.defaults();
    let (combo_x, combo_y, combo_w, combo_h) = DEFAULTS_COMBO_AT;
    let combo = if settings.listed() {
        let shown = Combo {
            enabled: enabled && settings.combo().enabled,
            ..settings.combo().clone()
        };
        combo_box(
            "framelegacy-defaults-combo".to_owned(),
            &shown,
            DEFAULTS_COMBO_AT,
            |this| this.frame_type_legacy.defaults_mut().toggle_dropdown(),
            cx,
        )
    } else {
        crate::probe::measured(
            "framelegacy-defaults-combo",
            at((combo_x, combo_y, combo_w, combo_h)),
        )
        .flex()
        .items_center()
        .px_1()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .text_xs()
        .text_color(rgb(theme::DIM))
        .child(div().flex_1().child(DEFAULTS_LOADING))
        .child(div().text_size(px(7.0)).child("▼"))
        .into_any_element()
    };
    let mut defaults = at(DEFAULTS_AT)
        .child(
            crate::probe::measured("framelegacy-defaults-text", at(DEFAULTS_TEXT_AT))
                .p(px(2.0))
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::PANEL))
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(DEFAULTS_TEXT),
        )
        .child(combo)
        .child(button(
            "framelegacy-defaults-load",
            DEFAULTS_LOAD,
            DEFAULTS_BUTTON_AT,
            enabled && settings.button_enabled() && !settings.busy(),
            |this, _window, _cx| {
                // `Settings.GetUserDataDirectory()`.
                let user_data = mp_settings::user_data_directory()
                    .unwrap_or_else(|| std::env::temp_dir().join("MissionPlannerRust"));
                let _ = std::fs::create_dir_all(&user_data);
                this.frame_type_legacy.defaults_mut().click_load(&user_data);
            },
            cx,
        ));
    if enabled && settings.dropdown() {
        defaults = defaults.child(dropdown(
            "framelegacy-defaults-combo",
            settings.combo(),
            (combo_x, combo_y + combo_h, combo_w),
            |this, index| this.frame_type_legacy.defaults_mut().choose(index),
            |this, lines| this.frame_type_legacy.defaults_mut().scroll_list(lines),
            cx,
        ));
    }
    let defaults = group(DEFAULTS_GROUP_AT, DEFAULTS_GROUP, enabled).child(defaults);

    body = body.child(types).child(defaults);
    Some(panel(TITLE, body).into_any_element())
}

/// The box or form showing, drawn over the whole window: the page's own box, the Default
/// Settings control's, and `ParamCompare` under them, as the C#'s boxes show over the dialog
/// they came from.
pub fn overlay(
    frame: &FrameTypeLegacy,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(message) = frame.message() {
        let ok = action(
            "framelegacy-message-ok",
            "OK",
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.frame_type_legacy.dismiss_message();
                cx.notify();
            }),
        );
        return Some(modal(
            "framelegacy-message",
            message.title,
            &message.text,
            message.title == ERROR_TITLE,
            vec![ok],
            window,
        ));
    }
    let defaults = frame.defaults();
    if let Some(message) = defaults.message() {
        return Some(message_box(
            "framelegacy-defaults-message",
            "framelegacy-defaults-message-ok",
            message,
            window,
            |this| this.frame_type_legacy.defaults_mut().dismiss_message(),
            cx,
        ));
    }
    let form = defaults.compare()?;
    Some(param_compare::dialog(
        form,
        defaults.writing(),
        param_compare::Handlers {
            toggle_all: |this: &mut MissionPlanner| {
                this.frame_type_legacy.defaults_mut().toggle_all();
            },
            toggle_row: |this: &mut MissionPlanner, index: usize| {
                this.frame_type_legacy.defaults_mut().toggle_row(index);
            },
            save: |this: &mut MissionPlanner| this.frame_type_legacy.defaults_mut().click_save(),
            close: |this: &mut MissionPlanner| {
                this.frame_type_legacy.defaults_mut().close_compare();
            },
        },
        window,
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;

    use mp_link::ProtocolTimeouts;
    use mp_mavlink_dialects::all::MavMessage;

    use super::*;
    use crate::telemetry::scripted::{Vehicle, param, until};

    /// A writer that answers each write with the next outcome it was given, and records them.
    #[derive(Default)]
    struct Answers {
        outcomes: RefCell<VecDeque<Progress>>,
        written: RefCell<Vec<(String, f64)>>,
    }

    impl Answers {
        fn with(outcomes: &[Progress]) -> Self {
            Self {
                outcomes: RefCell::new(outcomes.iter().copied().collect()),
                written: RefCell::default(),
            }
        }

        fn values(&self) -> Vec<f64> {
            self.written
                .borrow()
                .iter()
                .map(|(_, value)| *value)
                .collect()
        }
    }

    impl ParamWriter for Answers {
        type Handle = usize;

        fn write(&self, name: &str, value: f64) -> Option<usize> {
            let mut written = self.written.borrow_mut();
            written.push((name.to_owned(), value));
            Some(written.len() - 1)
        }

        fn progress(&self, _handle: usize) -> Progress {
            self.outcomes
                .borrow_mut()
                .pop_front()
                .unwrap_or(Progress::Finished(RequestOutcome::Unchanged))
        }
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn view() -> TelemetryView {
        TelemetryView::disconnected("test")
    }

    fn opened(frame_value: f64) -> FrameTypeLegacy<usize> {
        let mut frame = FrameTypeLegacy::default();
        frame.activate(&[(PARAM.to_owned(), frame_value)], key());
        frame
    }

    /// Runs the writes to the end, every one answered as unchanged unless told otherwise.
    fn drain(frame: &mut FrameTypeLegacy<usize>, answers: &Answers) {
        for _ in 0..20 {
            frame.tick(answers, &view(), true);
        }
    }

    /// The `.resx`'s places and words, and the Designer's order of the radio buttons.
    #[test]
    fn the_rows_and_words_are_the_resx() {
        let (Some(resx), Some(designer), Some(defaults)) = (
            crate::config_coverage::source::csharp(
                "GCSViews/ConfigurationView/ConfigFrameType.resx",
            ),
            crate::config_coverage::source::csharp(
                "GCSViews/ConfigurationView/ConfigFrameType.Designer.cs",
            ),
            crate::config_coverage::source::csharp("Controls/DefaultSettings.resx"),
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let get = |key: String| values.get(&key).cloned();
        let pair = |(x, y): (f32, f32)| format!("{x}, {y}");
        for row in ROWS {
            let (label, words) = row.label;
            assert_eq!(get(format!("{label}.Text")).as_deref(), Some(words));
            assert_eq!(get(format!("{label}.Location")), Some(pair(row.label_at)));
            assert_eq!(
                get(format!("radioButton_{}.Location", row.name)),
                Some(pair(row.radio_at))
            );
            let (x, y, w, h) = row.picture;
            assert_eq!(
                get(format!("pictureBox{}.Location", row.name)),
                Some(pair((x, y)))
            );
            assert_eq!(
                get(format!("pictureBox{}.Size", row.name)),
                Some(pair((w, h)))
            );
        }
        for (name, (x, y, w, h)) in [
            ("groupBox2", TYPE_GROUP_AT),
            ("groupBox1", DEFAULTS_GROUP_AT),
            ("configDefaultSettings1", DEFAULTS_AT),
            ("label4", NOTE_H_AT),
            ("label7", NOTE_Y6B_AT),
        ] {
            assert_eq!(
                get(format!("{name}.Location")),
                Some(pair((x, y))),
                "{name}"
            );
            assert_eq!(get(format!("{name}.Size")), Some(pair((w, h))), "{name}");
        }
        assert_eq!(get("groupBox2.Text".into()).as_deref(), Some(TYPE_GROUP));
        assert_eq!(
            get("groupBox1.Text".into()).as_deref(),
            Some(DEFAULTS_GROUP)
        );
        assert_eq!(get("label4.Text".into()).as_deref(), Some(NOTE_H));
        assert_eq!(get("label7.Text".into()).as_deref(), Some(NOTE_Y6B));
        assert_eq!(get("$this.Size".into()).as_deref(), Some("787, 426"));

        // The order the group's controls are added, the radio buttons among them.
        let order: Vec<i32> = designer
            .lines()
            .filter_map(|line| {
                line.trim()
                    .strip_prefix("this.groupBox2.Controls.Add(this.radioButton_")
                    .and_then(|rest| rest.strip_suffix(");"))
            })
            .filter_map(|name| ROWS.iter().find(|row| row.name == name))
            .map(|row| row.frame)
            .collect();
        assert_eq!(order, CONTROLS_ORDER);

        let defaults = crate::config_coverage::source::resx(&defaults);
        let dget = |key: &str| defaults.get(key).cloned();
        assert_eq!(dget("textBox1.Text").as_deref(), Some(DEFAULTS_TEXT));
        assert_eq!(
            dget("CMB_paramfiles.Text").as_deref(),
            Some(DEFAULTS_LOADING)
        );
        assert_eq!(
            dget("BUT_paramfileload.Text").as_deref(),
            Some(DEFAULTS_LOAD)
        );
        assert_eq!(dget("BUT_paramfileload.Enabled").as_deref(), Some("False"));
        for (name, (x, y, w, h)) in [
            ("textBox1", DEFAULTS_TEXT_AT),
            ("CMB_paramfiles", DEFAULTS_COMBO_AT),
            ("BUT_paramfileload", DEFAULTS_BUTTON_AT),
        ] {
            assert_eq!(
                dget(&format!("{name}.Location")),
                Some(pair((x, y))),
                "{name}"
            );
            assert_eq!(dget(&format!("{name}.Size")), Some(pair((w, h))), "{name}");
        }
    }

    /// `ArduPilot.Frame`'s values, from the C#.
    #[test]
    fn the_frames_are_ardupilots() {
        let (Some(frame), Some(types)) = (
            crate::config_coverage::source::csharp("ExtLibs/ArduPilot/Frame.cs"),
            crate::config_coverage::source::csharp("ExtLibs/ArduPilot/motor_frame_type.cs"),
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        for (name, member) in [
            ("Plus", "PLUS"),
            ("X", "X"),
            ("V", "V"),
            ("H", "H"),
            ("VTail", "VTAIL"),
            ("Y", "Y6B"),
        ] {
            assert!(frame.contains(&format!(
                "{name} = motor_frame_type.MOTOR_FRAME_TYPE_{member}"
            )));
            let value = ROWS
                .iter()
                .find(|row| row.name == name)
                .map(|row| row.frame)
                .expect("a row");
            assert!(
                types.contains(&format!("MOTOR_FRAME_TYPE_{member} = {value},")),
                "{name} is {value}"
            );
        }
    }

    /// `Activate` checks the vehicle's frame, brightens its picture, and writes it back.
    #[test]
    fn activate_reads_the_frame_and_writes_it_back() {
        let mut frame = opened(1.0);
        assert!(frame.enabled());
        assert_eq!(frame.read, Some(1));
        assert_eq!(frame.checked(), [1]);
        assert_eq!(frame.bright(), [1]);
        assert_eq!(frame.writes().values(), [1.0]);
        let answers = Answers::default();
        drain(&mut frame, &answers);
        assert_eq!(answers.values(), [1.0]);
        assert_eq!(frame.writes().last.as_deref(), Some("FRAME 1 unchanged"));
    }

    #[test]
    fn a_vehicle_without_frame_disables_the_page() {
        let mut frame: FrameTypeLegacy<usize> = FrameTypeLegacy::default();
        frame.activate(&[], key());
        assert!(!frame.enabled());
        assert!(frame.checked().is_empty());
        assert_eq!(
            frame.bright(),
            [0, 1, 2, 3, 10, 4],
            "the Designer's opacity"
        );
        frame.click_radio(0);
        frame.click_picture(0);
        assert_eq!(frame.writes().pending(), 0);
    }

    /// A value `Frame` does not name unchecks every button but V Tail's and writes nothing; one
    /// that is not whole stops `Activate` where `Enum.Parse` throws.
    #[test]
    fn an_unnamed_frame_unchecks_all_but_v_tail() {
        let mut frame = opened(4.0);
        assert_eq!(frame.checked(), [4]);
        frame.writes.queued.clear();
        frame.activate(&[(PARAM.to_owned(), 7.0)], key());
        assert_eq!(frame.checked(), [4], "V Tail is left checked");
        assert_eq!(frame.bright(), [4], "and the pictures as they were");
        assert_eq!(frame.writes().pending(), 0);

        let mut frame = opened(0.0);
        frame.writes.queued.clear();
        frame.activate(&[(PARAM.to_owned(), 7.0)], key());
        assert!(frame.checked().is_empty());

        let frame = opened(1.5);
        assert_eq!(frame.read, None);
        assert!(frame.checked().is_empty());
        assert_eq!(frame.writes().pending(), 0);
    }

    /// Clicking X with Plus checked: Plus's own `CheckedChanged` runs first, checking Plus again
    /// and writing 0, then X's writes 1 - and X ends checked, alone.
    #[test]
    fn a_radio_click_writes_the_frame_that_was_checked_then_its_own() {
        let mut frame = opened(0.0);
        frame.writes.queued.clear();
        frame.click_radio(1);
        assert_eq!(frame.checked(), [1]);
        assert_eq!(frame.bright(), [1]);
        assert_eq!(frame.writes().values(), [0.0, 1.0]);

        // The checked one raises nothing.
        frame.writes.queued.clear();
        frame.click_radio(1);
        assert_eq!(frame.writes().pending(), 0);

        // From V Tail to Y6B: the same.
        let mut frame = opened(4.0);
        frame.writes.queued.clear();
        frame.click_radio(10);
        assert_eq!(frame.checked(), [10]);
        assert_eq!(frame.writes().values(), [4.0, 10.0]);
    }

    /// A picture's click is `DoChange` for its frame alone, checked already or not; V Tail's
    /// picture too, which on this page has its handler.
    #[test]
    fn a_picture_click_writes_its_frame_alone() {
        let mut frame = opened(0.0);
        frame.writes.queued.clear();
        frame.click_picture(3);
        assert_eq!(frame.checked(), [3]);
        assert_eq!(frame.bright(), [3]);
        assert_eq!(frame.writes().values(), [3.0]);
        frame.click_picture(3);
        assert_eq!(frame.writes().values(), [3.0, 3.0]);
        frame.click_picture(4);
        assert_eq!(frame.checked(), [4]);
        assert_eq!(frame.writes().values(), [3.0, 3.0, 4.0]);
    }

    /// With no radio button checked - a frame `Frame` does not name - a click writes its own
    /// frame alone.
    #[test]
    fn a_click_with_nothing_checked_writes_once() {
        let mut frame = opened(7.0);
        assert!(frame.checked().is_empty());
        frame.click_radio(2);
        assert_eq!(frame.checked(), [2]);
        assert_eq!(frame.writes().values(), [2.0]);
    }

    /// The writes go one at a time; a timeout says "Set FRAME Failed" and the next still goes.
    #[test]
    fn a_timed_out_write_says_so_and_the_next_goes() {
        let mut frame = opened(0.0);
        frame.writes.queued.clear();
        frame.click_radio(1);
        let answers = Answers::with(&[
            Progress::Waiting,
            Progress::Finished(RequestOutcome::TimedOut),
            Progress::Finished(RequestOutcome::Accepted { value: None }),
        ]);
        frame.tick(&answers, &view(), true);
        assert_eq!(answers.values(), [0.0], "one at a time");
        frame.tick(&answers, &view(), true);
        assert_eq!(answers.values(), [0.0, 1.0]);
        assert_eq!(frame.message().map(|m| m.text.as_str()), Some(FAILED));
        assert_eq!(frame.message().map(|m| m.title), Some(ERROR_TITLE));
        frame.tick(&answers, &view(), true);
        assert_eq!(frame.writes().last.as_deref(), Some("FRAME 1 accepted"));
        assert_eq!(frame.writes().pending(), 0);
    }

    /// The page object lives while its screen does: shown again, its radio buttons are as they
    /// were; the screen left, it is the Designer's again.
    #[test]
    fn the_page_object_lives_with_its_screen() {
        let mut frame = opened(0.0);
        let answers = Answers::default();
        drain(&mut frame, &answers);
        frame.deactivate();
        frame.tick(&answers, &view(), true);
        assert_eq!(frame.checked(), [0]);
        frame.tick(&answers, &view(), false);
        assert!(frame.checked().is_empty());
        assert_eq!(frame.read, None);
    }

    /// Through the real link: opening writes nothing the vehicle holds already, and a click puts
    /// the new frame on the wire.
    #[test]
    fn a_click_reaches_the_vehicle() {
        let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default().faster(20));
        vehicle.send(&param(PARAM, 0.0, 2));
        until("FRAME to be held", || telemetry.holds_parameter(PARAM));
        let view = telemetry.view();
        let mut frame: FrameTypeLegacy = FrameTypeLegacy::default();
        frame.activate(&view.parameters, Key::of(&view));
        frame.click_radio(3);
        let mut written = Vec::new();
        until("the writes", || {
            frame.tick(&telemetry, &telemetry.view(), true);
            for message in vehicle.read() {
                if let MavMessage::ParamSet(set) = message {
                    written.push(set.param_value);
                    vehicle.send(&param(PARAM, set.param_value, 2));
                }
            }
            frame.writes().pending() == 0
        });
        // 0, the vehicle's own, is "not modified as same" twice over and never sent.
        assert_eq!(written, [3.0]);
        assert_eq!(frame.writes().last.as_deref(), Some("FRAME 3 accepted"));
    }

    /// Every fact the script asserts on is one this page records.
    #[test]
    fn the_gui_script_names_facts_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-frametype-legacy.gui");
        let source = include_str!("frame_type_legacy.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            if let (Some("expect"), Some(key)) = (words.next(), words.next())
                && key.starts_with("config.framelegacy.")
            {
                assert!(source.contains(&format!("\"{key}\"")), "{key}");
                facts += 1;
            }
            assert!(
                !line.contains("click framelegacy-"),
                "SITL never lists the page: nothing on it can be clicked"
            );
        }
        assert!(facts >= 4, "{facts} facts");
    }

    /// The Default Settings control's `Load` runs with the page's first `Activate`, disabled page
    /// or not, and asks for the listing once.
    #[test]
    fn the_default_settings_load_with_the_page() {
        let frame = opened(1.0);
        assert!(frame.defaults().loaded());
        assert!(frame.defaults().busy(), "the listing is asked for");
        assert!(!frame.defaults().button_enabled());

        let mut frame: FrameTypeLegacy<usize> = FrameTypeLegacy::default();
        frame.activate(&[], key());
        assert!(!frame.enabled());
        assert!(
            frame.defaults().loaded(),
            "Load fires on a disabled page too"
        );
    }

    /// `ParamCompare` over a `.param` from `Tools/Frame_params`, closed: the control's `OnChange`
    /// runs the page's `Activate` again, which reads `FRAME` as the vehicle now holds it and
    /// writes it back - through the real `FRAME` of the vehicle's table.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameType.cs:22, 41-44`
    #[test]
    fn a_default_settings_change_runs_activate_again() {
        use crate::config::default_settings::Arrived;
        use mp_firmware::github::FileInfo;

        let dir = std::env::temp_dir().join(format!(
            "mp-gui-framelegacy-defaults-{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).expect("a scratch directory");
        let mut view = view();
        view.parameters = vec![(PARAM.to_owned(), 1.0)].into();
        let mut frame = opened(1.0);
        let answers = Answers::default();
        for _ in 0..5 {
            frame.tick(&answers, &view, true);
        }
        assert_eq!(answers.values(), [1.0], "Activate's own write");

        let defaults = frame.defaults_mut();
        defaults.arrive(
            Arrived::Listing(Ok(vec![FileInfo {
                name: "Y6B.param".into(),
                path: "Tools/Frame_params/Y6B.param".into(),
                size: 8,
            }])),
            &[],
        );
        defaults.arrive(
            Arrived::File {
                save_as: dir.join("Y6B.param"),
                bytes: Ok(Some(b"FRAME 10\n".to_vec())),
            },
            &view.parameters,
        );
        assert_eq!(
            frame.defaults().compare().map(|form| form.rows().len()),
            Some(1),
            "FRAME differs"
        );
        frame.defaults_mut().close_compare();
        for _ in 0..5 {
            frame.tick(&answers, &view, true);
        }
        assert_eq!(answers.values(), [1.0, 1.0], "Activate ran again");
        assert_eq!(frame.checked(), [1]);
        let _ = std::fs::remove_dir_all(dir);
    }

    /// "Set FRAME Failed" goes on the status line rather than in a box (the owner's ruling).
    #[test]
    fn a_failed_frame_write_is_a_status_line() {
        let mut frame = opened(0.0);
        let answers = Answers::with(&[Progress::Finished(RequestOutcome::TimedOut)]);
        frame.tick(&answers, &view(), true);
        frame.tick(&answers, &view(), true);
        assert_eq!(frame.take_link_errors().as_deref(), Some(FAILED));
        assert!(frame.message().is_none());
        assert_eq!(frame.take_link_errors(), None);
    }
}
