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

//! OSD: `GCSViews/ConfigurationView/ConfigHWOSD.cs`, an Optional Hardware page of Initial Setup
//! (`GCSViews/InitialSetup.cs:303-306`), listed once every parameter is in.
//!
//! What it shows: the heading, the MinimOSD's picture, "Enable Telemetry" and the note beside it.
//! `Activate` enables the page - it disables it without a link and then enables it again at
//! once, so it is always enabled (`ConfigHWOSD.cs:14-21`). "Enable Telemetry" sets the stream
//! rates a MinimOSD needs on the first, second and fourth serial ports: 24 `setParam` calls of
//! 2, `SR0_`, `SR1_` and `SR3_` each of `EXT_STAT`, `EXTRA1` to `EXTRA3`, `POSITION`, `RAW_CTRL`,
//! `RAW_SENS` and `RC_CHAN`, in one `try` whose `catch` says "Failed to set OSD rates." (`:23-68`).
//! A name the vehicle does not list is `setParam`'s false, which the handler ignores; a call
//! that times out ends the handler with the box.
//!
//! The layout is `ConfigHWOSD.resx`'s, every control at its `Location` in a 650 x 119 page, and
//! `pictureBox5` shows `Resources.MinimOSD`, zoomed, as its `BackgroundImage` ([`crate::pictures`]).

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;

use gpui::{AnyElement, Context, Window, div, prelude::*, px, rgb};

use super::optional::{Job, Set, SetQueue, at, button, heading, message_box, picture, plain, rule};
use crate::MissionPlanner;
use crate::config::servo_output::{Message, value_of};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{panel, theme};

/// The page's title in Initial Setup's list, `backstageViewPageosd.Text`.
/// `// C#: GCSViews/InitialSetup.resx:729-731`
pub const TITLE: &str = "OSD";

/// `label6.Text`, the heading.
/// `// C#: GCSViews/ConfigurationView/ConfigHWOSD.resx label6.Text`
pub const HEADING: &str = "OSD";

/// `BUT_osdrates.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigHWOSD.resx BUT_osdrates.Text`
pub const BUTTON: &str = "Enable Telemetry";

/// `label7.Text`, beside the button.
/// `// C#: GCSViews/ConfigurationView/ConfigHWOSD.resx label7.Text`
pub const NOTE: &str =
    "You only need to use this if you are having issue with your OSD not updating";

/// `BUT_osdrates_Click`'s `catch`: `CustomMessageBox.Show` with no caption.
/// `// C#: GCSViews/ConfigurationView/ConfigHWOSD.cs:67`
pub const FAILED: &str = "Failed to set OSD rates.";

/// The rate every call sets.
/// `// C#: GCSViews/ConfigurationView/ConfigHWOSD.cs:38-63`
pub const RATE: f64 = 2.0;

/// The parameters `BUT_osdrates_Click` sets, in its order.
/// `// C#: GCSViews/ConfigurationView/ConfigHWOSD.cs:38-63`
pub const RATES: [&str; 24] = [
    "SR0_EXT_STAT",
    "SR0_EXTRA1",
    "SR0_EXTRA2",
    "SR0_EXTRA3",
    "SR0_POSITION",
    "SR0_RAW_CTRL",
    "SR0_RAW_SENS",
    "SR0_RC_CHAN",
    "SR1_EXT_STAT",
    "SR1_EXTRA1",
    "SR1_EXTRA2",
    "SR1_EXTRA3",
    "SR1_POSITION",
    "SR1_RAW_CTRL",
    "SR1_RAW_SENS",
    "SR1_RC_CHAN",
    "SR3_EXT_STAT",
    "SR3_EXTRA1",
    "SR3_EXTRA2",
    "SR3_EXTRA3",
    "SR3_POSITION",
    "SR3_RAW_CTRL",
    "SR3_RAW_SENS",
    "SR3_RC_CHAN",
];

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigHWOSD.resx $this.Size`
const PAGE_SIZE: (f32, f32) = (650.0, 119.0);
/// `BUT_osdrates`: the `.resx`'s 60 x 23, which is narrower than its text - the text runs past
/// the box's ends as it is clipped in the C#.
const BUTTON_AT: (f32, f32, f32, f32) = (166.0, 61.0, 60.0, 23.0);
/// `label7`: a fixed 180 x 51, the note wrapped inside it.
const NOTE_AT: (f32, f32, f32, f32) = (245.0, 50.0, 180.0, 51.0);

/// The page object.
#[derive(Debug, Default)]
pub struct Osd {
    /// The screen the page object belongs to.
    made_for: Option<Key>,
    /// Whether the page is showing.
    active: bool,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// The writes.
    queue: SetQueue,
    /// How many times the button has been pressed on an enabled page.
    presses: usize,
}

impl Osd {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// The page's `Enabled`: `Activate` leaves it true whatever the link, so it is true
    /// whenever the page shows.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWOSD.cs:14-21`
    #[must_use]
    pub const fn enabled(&self) -> bool {
        true
    }

    /// The message box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// Dismisses it.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// Shows the page: a new page object for a new screen, then `Activate`, which leaves the
    /// page enabled.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWOSD.cs:14-21`
    pub fn activate(&mut self, key: Key) {
        if self.made_for != Some(key) {
            let messages = std::mem::take(&mut self.messages);
            let queue = std::mem::take(&mut self.queue);
            *self = Self {
                made_for: Some(key),
                messages,
                queue,
                presses: self.presses,
                ..Self::default()
            };
        }
        self.active = true;
    }

    /// The page hidden. `ConfigHWOSD` is `IActivate` only.
    pub fn hide(&mut self) {
        self.active = false;
    }

    /// "Enable Telemetry": the 24 rates in one `try`, a throw ending it with the box.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWOSD.cs:23-68`
    pub fn click_enable_telemetry(&mut self) -> Vec<Job> {
        if !self.active {
            return Vec::new();
        }
        self.presses += 1;
        let mut job = Job::new("rates", RATES.iter().map(|name| Set::plain(*name, RATE)));
        job.on_throw = Some(|_| plain(FAILED));
        vec![job]
    }

    /// Queues handlers' jobs.
    pub fn push(&mut self, jobs: Vec<Job>) {
        self.queue.push(jobs);
    }

    /// Once a frame: a page object whose screen has gone is let go, and the writes move on.
    pub fn tick(&mut self, telemetry: &Telemetry, view: &TelemetryView, on_setup: bool) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            self.made_for = None;
        }
        self.queue.advance(telemetry, &mut self.messages);
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(page: &Osd, view: &TelemetryView) {
    use crate::facts::record;
    record("config.osd.active", page.is_active());
    record("config.osd.enabled", page.enabled());
    record("config.osd.heading", HEADING);
    record("config.osd.button", BUTTON);
    record("config.osd.note", NOTE);
    record("config.osd.presses", page.presses);
    record("config.osd.write", page.queue.last().unwrap_or("none"));
    record("config.osd.writes.pending", page.queue.pending());
    record(
        "config.osd.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    for name in RATES {
        if let Some(value) = value_of(&view.parameters, name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

/// The page, laid out as `ConfigHWOSD.resx` lays it out.
pub fn page(osd: &Osd, cx: &mut Context<MissionPlanner>) -> AnyElement {
    if !osd.is_active() {
        return div().into_any_element();
    }
    let (x, y, width, height) = NOTE_AT;
    let body = div()
        .relative()
        .w(px(PAGE_SIZE.0))
        .h(px(PAGE_SIZE.1))
        .child(heading(7.0, 5.0, HEADING, true))
        .child(rule(3.0, 23.0, 644.0))
        // C#: GCSViews/ConfigurationView/ConfigHWOSD.Designer.cs:54; ConfigHWOSD.resx:177-178
        .child(picture(
            "osd",
            "MinimOSD",
            (11.0, 35.0, 75.0, 75.0),
            "MinimOSD",
            crate::pictures::Layout::Zoom,
        ))
        .child(button(
            "osd-enable-telemetry",
            BUTTON,
            BUTTON_AT,
            true,
            |this, _window, _cx| {
                let jobs = this.extra.osd.click_enable_telemetry();
                this.extra.osd.push(jobs);
            },
            cx,
        ))
        .child(
            at(x, y, width, height)
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(NOTE),
        );
    panel(TITLE, body).into_any_element()
}

/// The message box showing, over the whole window.
pub fn overlay(osd: &Osd, window: &Window, cx: &mut Context<MissionPlanner>) -> Option<AnyElement> {
    let message = osd.message()?;
    Some(message_box(
        "osd-message",
        "osd-message-ok",
        message,
        window,
        |this| this.extra.osd.dismiss_message(),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::tests::{Answering, drain};
    use mp_link::requests::RequestOutcome;

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    /// The `.resx`'s words and places, read from the tree when it is here.
    #[test]
    fn the_text_is_the_resx_text() {
        let Some(resx) =
            crate::config_coverage::source::csharp("GCSViews/ConfigurationView/ConfigHWOSD.resx")
        else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let get = |key: &str| values.get(key).map(String::as_str);
        assert_eq!(get("label6.Text"), Some(HEADING));
        assert_eq!(get("BUT_osdrates.Text"), Some(BUTTON));
        assert_eq!(get("BUT_osdrates.Location"), Some("166, 61"));
        assert_eq!(get("BUT_osdrates.Size"), Some("60, 23"));
        assert_eq!(get("label7.Text"), Some(NOTE));
        assert_eq!(get("label7.Location"), Some("245, 50"));
        assert_eq!(get("label7.Size"), Some("180, 51"));
        assert_eq!(get("pictureBox5.Location"), Some("11, 35"));
        assert_eq!(get("$this.Size"), Some("650, 119"));
    }

    /// The handler's names, read from the `.cs` when the tree is here: every `setParam` of the
    /// click, in its order, each to 2.
    #[test]
    fn the_rates_are_the_handlers() {
        let Some(cs) =
            crate::config_coverage::source::csharp("GCSViews/ConfigurationView/ConfigHWOSD.cs")
        else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let names: Vec<&str> = cs
            .lines()
            .filter(|line| line.contains("setParam("))
            .filter_map(|line| line.split('"').nth(1))
            .collect();
        assert_eq!(names, RATES);
        assert!(
            cs.lines()
                .filter(|line| line.contains("setParam("))
                .all(|line| line.trim_end().ends_with(", 2);"))
        );
    }

    #[test]
    fn the_page_is_enabled_whatever_the_link() {
        let mut page = Osd::default();
        page.activate(key());
        assert!(page.is_active() && page.enabled());
    }

    /// The press writes all 24, in order; the names SITL lacks are `false`, which the handler
    /// ignores - no box.
    #[test]
    fn enable_telemetry_sets_each_rate_to_two_in_order() {
        let mut page = Osd::default();
        page.activate(key());
        let jobs = page.click_enable_telemetry();
        let unknown = Progress::Finished(RequestOutcome::UnknownParameter);
        let link = Answering::new(&[("SR3_EXTRA1", unknown)]);
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        let messages = drain(&mut queue, &link);
        let written: Vec<String> = link.taken().into_iter().map(|(name, _)| name).collect();
        assert_eq!(written, RATES);
        assert!(
            link.taken()
                .iter()
                .all(|(_, value)| (*value - RATE).abs() < 1e-12)
        );
        assert!(messages.is_empty(), "{messages:?}");
        assert_eq!(queue.last(), Some("SR3_RC_CHAN 2 accepted"));
    }

    /// A timeout ends the handler: the calls after it are not made, and the `catch`'s box shows.
    #[test]
    fn a_timeout_ends_the_handler_with_its_box() {
        let mut page = Osd::default();
        page.activate(key());
        let jobs = page.click_enable_telemetry();
        let link = Answering::new(&[("SR1_EXTRA2", Progress::Finished(RequestOutcome::TimedOut))]);
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        let messages = drain(&mut queue, &link);
        assert_eq!(
            link.taken().len(),
            11,
            "SR0's eight, SR1_EXT_STAT to SR1_EXTRA2"
        );
        assert_eq!(messages, [plain(FAILED)]);
    }

    /// A hidden page takes no press.
    #[test]
    fn a_hidden_page_writes_nothing() {
        let mut page = Osd::default();
        assert!(page.click_enable_telemetry().is_empty());
        page.activate(key());
        page.hide();
        assert!(page.click_enable_telemetry().is_empty());
    }

    /// Every fact the GUI script asserts on is one this page records, and every control it
    /// clicks is one this page draws.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-osd.gui");
        let source = include_str!("osd.rs");
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.osd.") => {
                    assert!(
                        source.contains(&format!("\"{key}\"")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("osd-") => {
                    assert!(source.contains(&format!("\"{id}\"")), "{id} is not drawn");
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 5 && clicks >= 1, "{facts} facts, {clicks} clicks");
    }
}
