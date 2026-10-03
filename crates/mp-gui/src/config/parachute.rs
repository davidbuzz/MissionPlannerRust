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

//! Parachute: `GCSViews/ConfigurationView/ConfigHWParachute.cs`, an Optional Hardware page of
//! Initial Setup (`GCSViews/InitialSetup.cs:323-326`), listed once every parameter is in.
//!
//! What it shows: the heading, the parachute's picture, "Enable (AC3.3+)" (`CHUTE_ENABLED`), the
//! release type (`CHUTE_TYPE`, from the page's own list of four relays and a servo), the servo
//! output ("Servo Num", a plain `ComboBox` of RC9 to RC14), and the resting and deploy PWM and
//! the minimum altitude (`CHUTE_SERVO_OFF`, `CHUTE_SERVO_ON`, `CHUTE_ALT_MIN`), each a
//! `MavlinkNumericUpDown`. `Activate` sets them up each time the page is shown
//! (`ConfigHWParachute.cs:17-45`); each `Mavlink*` control writes its parameter when it changes, a
//! number 300 ms after.
//!
//! "Servo Num" is the page's own: `Activate` shows the output the vehicle already has on function
//! 27 (the first `*_FUNCTION` parameter holding 27, less its `_FUNCTION`), and choosing one runs
//! `ensureDisabled` - every other listed output on 27 set back to 0 - then sets the chosen one's
//! `_FUNCTION` to 27 (`:47-74`). Its items are the `RCn` names of firmware before Copter 3.5; a
//! vehicle that names its outputs `SERVOn` lists no `RCn_FUNCTION`, so `setParam` returns false
//! for each - ignored, as the C# ignores it - and the combo changes nothing on the vehicle. A name
//! the vehicle has on 27 that is not one of the six (`SERVO9`, say) selects nothing, as a
//! `DropDownList`'s `Text` does for text it does not list. The handler has no `try`: a call that
//! times out is the unhandled-exception box, and ends the handler.
//!
//! `startup` starts true and `Activate` sets it false after reading "Servo Num"'s selection, so the
//! first `Activate` writes nothing; a later one whose selection differs from the combo's runs the
//! handler, as the C#'s `Text` setter raises `SelectedIndexChanged` then.
//!
//! The layout is `ConfigHWParachute.resx`'s, every control at its `Location` in a 650 x 212 page;
//! `pictureBox3` shows `Resources.Parachute` over `Resources.sonar`, both zoomed
//! ([`crate::pictures`]).
//!
//! What differs, and why: the first `*_FUNCTION` on 27 is the first in name order, the order this application keeps the
//! vehicle's table in; the C#'s is the order the parameters arrived, which differs only for a
//! vehicle with two outputs on 27.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::time::Instant;

use gpui::{AnyElement, Context, KeyDownEvent, Window, div, prelude::*, px};

use super::optional::{Job, Set, SetQueue, heading, label, message_box, rule, value_of};
use super::rangefinder::unhandled;
use crate::MissionPlanner;
use crate::config::failsafe::Lookup;
use crate::config::servo_output::{
    Check, Combo, Message, NUMERIC_DEFAULTS, Number, NumberHandlers, OUT_OF_RANGE_TITLE, Question,
    Setup, Write, check_box, combo_box, dropdown, modal, number_box,
};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{action, panel, theme};

use super::extra_setup::Focus;

/// The page's title in Initial Setup's list, `backstageViewPageParachute.Text`.
/// `// C#: GCSViews/InitialSetup.resx:756-758`
pub const TITLE: &str = "Parachute";

/// `label1.Text`, the heading.
/// `// C#: GCSViews/ConfigurationView/ConfigHWParachute.resx label1.Text`
pub const HEADING: &str = "Parachute";

/// `mavlinkCheckBoxEnable.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigHWParachute.resx mavlinkCheckBoxEnable.Text`
pub const ENABLE: &str = "Enable (AC3.3+)";

/// `SERVOn_FUNCTION`'s value for a parachute release.
/// `// C#: GCSViews/ConfigurationView/ConfigHWParachute.cs:23, 52-54`
pub const PARACHUTE_FUNCTION: f64 = 27.0;

/// The type combo's list, which `Activate` builds.
/// `// C#: GCSViews/ConfigurationView/ConfigHWParachute.cs:34-40`
pub const TYPES: [(i64, &str); 5] = [
    (0, "First Relay"),
    (1, "Second Relay"),
    (2, "Third Relay"),
    (3, "Fourth Relay"),
    (10, "Servo"),
];

/// "Servo Num"'s items, from the Designer; keyed by their index.
/// `// C#: GCSViews/ConfigurationView/ConfigHWParachute.Designer.cs:103-109; ConfigHWParachute.resx mavlinkComboBoxServoNum.Items to Items5`
pub const SERVOS: [&str; 6] = ["RC9", "RC10", "RC11", "RC12", "RC13", "RC14"];

/// One of the three numbers: its parameter, its label, and the two `Location`s.
#[derive(Debug, Clone, Copy)]
pub struct NumberRow {
    /// The parameter.
    pub param: &'static str,
    /// `setup(Min, Max, Scale, Increment, ...)`.
    pub setup: Setup,
    /// The label's text.
    pub label: &'static str,
    /// The label's `Location`.
    pub label_at: (f32, f32),
    /// The box's `Location`.
    pub at: (f32, f32),
}

/// `setup(1000, 2000, 1, 1, ...)` for the two PWMs.
/// `// C#: GCSViews/ConfigurationView/ConfigHWParachute.cs:42-43`
const PWM: Setup = Setup {
    minimum: 1000.0,
    maximum: 2000.0,
    scale: 1.0,
    increment: 1.0,
};

/// The three numbers, in the `.cs`'s order.
/// `// C#: GCSViews/ConfigurationView/ConfigHWParachute.cs:42-44; ConfigHWParachute.resx label4-6, mavlinkNumericUpDown*`
pub const NUMBERS: [NumberRow; 3] = [
    NumberRow {
        param: "CHUTE_SERVO_OFF",
        setup: PWM,
        label: "Resting PWM",
        label_at: (102.0, 111.0),
        at: (186.0, 109.0),
    },
    NumberRow {
        param: "CHUTE_SERVO_ON",
        setup: PWM,
        label: "Deploy PWM",
        label_at: (102.0, 141.0),
        at: (186.0, 139.0),
    },
    NumberRow {
        param: "CHUTE_ALT_MIN",
        setup: Setup {
            minimum: 0.0,
            maximum: 32000.0,
            scale: 1.0,
            increment: 1.0,
        },
        label: "Min Alt (m)",
        label_at: (102.0, 168.0),
        at: (186.0, 166.0),
    },
];

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigHWParachute.resx $this.Size`
const PAGE_SIZE: (f32, f32) = (650.0, 212.0);
/// `mavlinkComboBoxType`.
const TYPE_AT: (f32, f32, f32, f32) = (186.0, 54.0, 121.0, 20.0);
/// `mavlinkComboBoxServoNum`.
const SERVO_AT: (f32, f32, f32, f32) = (186.0, 83.0, 121.0, 20.0);

/// The two combos.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Which {
    /// `mavlinkComboBoxType`, `CHUTE_TYPE`.
    Type,
    /// `mavlinkComboBoxServoNum`, the page's own.
    Servo,
}

/// The page object.
#[derive(Debug)]
pub struct Parachute {
    made_for: Option<Key>,
    active: bool,
    /// `startup`: true from the constructor until the first `Activate` has read the servo.
    startup: bool,
    /// `mavlinkCheckBoxEnable`.
    enable: Check,
    /// `mavlinkComboBoxType`.
    kind: Combo,
    /// `mavlinkComboBoxServoNum`: enabled from the Designer, bound to nothing.
    servo: Combo,
    /// The three numbers, in [`NUMBERS`]' order.
    numbers: [Number; 3],
    dropdown: Option<Which>,
    editing: Option<usize>,
    question: Option<(usize, Question)>,
    messages: VecDeque<Message>,
    queue: SetQueue,
}

impl Default for Parachute {
    /// `InitializeComponent` and the field initialisers: the `Mavlink*` controls disabled (`.resx`
    /// `Enabled = False`), "Servo Num" enabled with nothing selected, `startup` true.
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            startup: true,
            enable: Check::default(),
            kind: Combo::default(),
            servo: Combo {
                options: SERVOS
                    .iter()
                    .zip(0_i64..)
                    .map(|(name, index)| (index, (*name).to_owned()))
                    .collect(),
                enabled: true,
                ..Combo::default()
            },
            numbers: std::array::from_fn(|_| Number::new(NUMERIC_DEFAULTS)),
            dropdown: None,
            editing: None,
            question: None,
            messages: VecDeque::new(),
            queue: SetQueue::default(),
        }
    }
}

/// The first `*_FUNCTION` parameter holding 27, less its `_FUNCTION` - `item.Replace("_FUNCTION",
/// "")` - or `None`.
/// `// C#: GCSViews/ConfigurationView/ConfigHWParachute.cs:21-28`
#[must_use]
pub fn servo_on_parachute(parameters: &[(String, f64)]) -> Option<String> {
    parameters.iter().find_map(|(name, value)| {
        // `MAV.param[item].ToString() == "27"`: a value written as exactly 27.
        #[allow(clippy::float_cmp)]
        let on = *value == PARACHUTE_FUNCTION;
        (name.ends_with("_FUNCTION") && on).then(|| name.replace("_FUNCTION", ""))
    })
}

/// "Servo Num"'s handler for `text` chosen: `ensureDisabled` over the six items - each other one
/// whose `_FUNCTION` the vehicle lists and holds 27 set to 0 - then the chosen `_FUNCTION` to 27,
/// in one handler without a `try`.
/// `// C#: GCSViews/ConfigurationView/ConfigHWParachute.cs:47-74`
#[must_use]
pub fn servo_handler(parameters: &[(String, f64)], text: &str) -> Job {
    let mut sets = Vec::new();
    for item in SERVOS {
        let name = format!("{item}_FUNCTION");
        let Some(held) = value_of(parameters, &name) else {
            continue;
        };
        if item == text {
            continue;
        }
        // `(float)MainV2.comPort.MAV.param[item + "_FUNCTION"] == number`.
        #[allow(clippy::cast_possible_truncation, clippy::float_cmp)]
        let on = f64::from(held as f32) == PARACHUTE_FUNCTION;
        if on {
            sets.push(uncaught(&name, 0.0));
        }
    }
    sets.push(uncaught(&format!("{text}_FUNCTION"), PARACHUTE_FUNCTION));
    Job::new("servo", sets)
}

/// A `setParam` whose `bool` is ignored and whose exception nothing catches.
fn uncaught(param: &str, value: f64) -> Set {
    Set {
        on_throw: Some(unhandled(param)),
        ..Set::plain(param, value)
    }
}

impl Parachute {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// "Enable (AC3.3+)".
    #[must_use]
    pub const fn enable(&self) -> &Check {
        &self.enable
    }

    /// A combo.
    #[must_use]
    pub const fn combo(&self, which: Which) -> &Combo {
        match which {
            Which::Type => &self.kind,
            Which::Servo => &self.servo,
        }
    }

    /// The numbers.
    #[must_use]
    pub const fn numbers(&self) -> &[Number; 3] {
        &self.numbers
    }

    /// The message box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// The question waiting.
    #[must_use]
    pub fn question(&self) -> Option<&Question> {
        self.question.as_ref().map(|(_, question)| question)
    }

    /// Dismisses the message box.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// The page object disposed, returning what its numbers' timers held.
    fn dispose(&mut self) -> Vec<Write> {
        let pending = self.numbers.iter_mut().filter_map(Number::flush).collect();
        let messages = std::mem::take(&mut self.messages);
        let queue = std::mem::take(&mut self.queue);
        *self = Self {
            messages,
            queue,
            ..Self::default()
        };
        pending
    }

    /// Shows the page: a new page object for a new screen, then `Activate`. What it returns is
    /// the servo handler's calls, when a later `Activate` changes the servo's selection.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWParachute.cs:17-45`
    pub fn activate(&mut self, parameters: &[(String, f64)], key: Key, lookup: Lookup) -> Vec<Job> {
        let mut jobs = Vec::new();
        if self.made_for != Some(key) {
            jobs.extend(self.dispose().into_iter().map(Job::control));
            self.made_for = Some(key);
        }
        self.active = true;
        self.dropdown = None;
        // `// C#: :19-28` - `Text =` on a `DropDownList`: the item of that text, found ignoring
        // case, selected if it is not already; text it does not list changes nothing.
        if let Some(text) = servo_on_parachute(parameters) {
            let found = self
                .servo
                .options
                .iter()
                .find(|(_, item)| item.eq_ignore_ascii_case(&text))
                .map(|(index, _)| *index);
            if let Some(index) = found
                && self.servo.select(index)
                && !self.startup
            {
                jobs.push(servo_handler(parameters, self.servo.text()));
            }
        }
        self.startup = false;
        // `// C#: :32-44`
        self.enable.setup(1.0, 0.0, "CHUTE_ENABLED", parameters);
        let types = TYPES
            .iter()
            .map(|(value, text)| (*value, (*text).to_owned()))
            .collect();
        self.kind.setup(types, "CHUTE_TYPE", parameters);
        for (number, row) in self.numbers.iter_mut().zip(NUMBERS) {
            number.setup(row.setup, row.param, parameters, lookup);
        }
        jobs
    }

    /// The page hidden - it is `IActivate` only - which reads a number being typed into.
    pub fn hide(&mut self, now: Instant) {
        self.active = false;
        self.dropdown = None;
        self.leave(now);
    }

    /// A click on "Enable (AC3.3+)": the control's own write.
    /// `// C#: Controls/MavlinkCheckBox.cs:106-143`
    pub fn click_enable(&mut self, now: Instant) -> Vec<Job> {
        self.leave(now);
        self.dropdown = None;
        self.enable.click().map(Job::control).into_iter().collect()
    }

    /// Drops a combo's list down, or back up.
    pub fn toggle_dropdown(&mut self, which: Which, now: Instant) {
        self.leave(now);
        let enabled = self.combo(which).enabled;
        self.dropdown = if self.dropdown == Some(which) || !enabled {
            None
        } else {
            match which {
                Which::Type => self.kind.open_list(),
                Which::Servo => self.servo.open_list(),
            }
            Some(which)
        };
    }

    /// The wheel over the list.
    pub fn scroll_list(&mut self, which: Which, lines: i32) {
        if self.dropdown == Some(which) {
            match which {
                Which::Type => self.kind.scroll_list(lines),
                Which::Servo => self.servo.scroll_list(lines),
            }
        }
    }

    /// A row chosen: the type combo's own write, or "Servo Num"'s handler when its selection
    /// changes.
    /// `// C#: Controls/MavlinkComboBox.cs:133-200; GCSViews/ConfigurationView/ConfigHWParachute.cs:47-55`
    pub fn choose(&mut self, which: Which, key: i64, parameters: &[(String, f64)]) -> Vec<Job> {
        self.dropdown = None;
        match which {
            Which::Type => self
                .kind
                .choose(key)
                .map(Job::control)
                .into_iter()
                .collect(),
            Which::Servo => {
                if !self.servo.select(key) || self.startup {
                    return Vec::new();
                }
                vec![servo_handler(parameters, self.servo.text())]
            }
        }
    }

    /// Whether a number takes clicks.
    fn number_live(&self, index: usize) -> bool {
        self.numbers.get(index).is_some_and(|number| number.enabled)
    }

    /// A number clicked into.
    pub fn begin(&mut self, index: usize, now: Instant) {
        if self.editing == Some(index) {
            return;
        }
        self.leave(now);
        self.dropdown = None;
        if self.number_live(index) {
            self.editing = Some(index);
        }
    }

    /// The number being typed into loses the focus, which reads its text.
    pub fn leave(&mut self, now: Instant) {
        let Some(index) = self.editing.take() else {
            return;
        };
        if let Some(question) = self
            .numbers
            .get_mut(index)
            .and_then(|number| number.commit(now))
        {
            self.question = Some((index, question));
        }
    }

    /// A key for the number being typed into.
    pub fn key(&mut self, event: &KeyDownEvent, now: Instant) -> bool {
        let Some(index) = self.editing else {
            return false;
        };
        let Some(number) = self.numbers.get_mut(index) else {
            return false;
        };
        let (handled, question) = number.key(event, now);
        if let Some(question) = question {
            self.question = Some((index, question));
        }
        handled
    }

    /// A number's arrow.
    pub fn step(&mut self, index: usize, up: bool, now: Instant) {
        self.begin(index, now);
        if !self.number_live(index) {
            return;
        }
        if let Some(question) = self
            .numbers
            .get_mut(index)
            .and_then(|number| number.step(up, now))
        {
            self.question = Some((index, question));
        }
    }

    /// The out-of-range question answered.
    pub fn answer(&mut self, yes: bool, now: Instant) {
        let Some((index, question)) = self.question.take() else {
            return;
        };
        if let Some(number) = self.numbers.get_mut(index) {
            number.answer(&question, yes, now);
        }
    }

    /// Queues handlers' jobs.
    pub fn push(&mut self, jobs: Vec<Job>) {
        self.queue.push(jobs);
    }

    /// Once a frame: a page object whose screen has gone is disposed, a number that lost the
    /// focus is read, the timers write, and the writes move on.
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        on_setup: bool,
        focused: bool,
        now: Instant,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            let pending: Vec<Job> = self.dispose().into_iter().map(Job::control).collect();
            self.queue.push(pending);
        }
        if self.editing.is_some() && !focused {
            self.leave(now);
        }
        let due: Vec<Job> = self
            .numbers
            .iter_mut()
            .filter_map(|number| number.due(now))
            .map(Job::control)
            .collect();
        self.queue.push(due);
        self.queue.advance(telemetry, &mut self.messages);
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(page: &Parachute, view: &TelemetryView) {
    use crate::facts::record;
    record("config.parachute.active", page.is_active());
    record("config.parachute.heading", HEADING);
    record("config.parachute.enable", page.enable().state.key());
    record("config.parachute.enable.enabled", page.enable().enabled);
    for (which, key) in [(Which::Type, "type"), (Which::Servo, "servo")] {
        let combo = page.combo(which);
        record(
            format!("config.parachute.{key}"),
            combo
                .selected
                .map_or_else(|| "none".to_owned(), |value| value.to_string()),
        );
        record(format!("config.parachute.{key}.text"), combo.text());
        record(format!("config.parachute.{key}.enabled"), combo.enabled);
        record(
            format!("config.parachute.{key}.options"),
            combo.options.len(),
        );
    }
    for (number, row) in page.numbers().iter().zip(NUMBERS) {
        let key = row.param.to_ascii_lowercase();
        record(format!("config.parachute.{key}"), number.shown());
        record(format!("config.parachute.{key}.enabled"), number.enabled);
    }
    record(
        "config.parachute.write",
        page.queue.last().unwrap_or("none"),
    );
    record("config.parachute.writes.pending", page.queue.pending());
    record(
        "config.parachute.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    let names = ["CHUTE_ENABLED", "CHUTE_TYPE"]
        .into_iter()
        .chain(NUMBERS.iter().map(|row| row.param));
    for name in names {
        if let Some(value) = value_of(&view.parameters, name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

/// The page, laid out as `ConfigHWParachute.resx` lays it out.
pub fn page(
    parachute: &Parachute,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if !parachute.is_active() {
        return div().into_any_element();
    }
    let mut body = div()
        .relative()
        .w(px(PAGE_SIZE.0))
        .h(px(PAGE_SIZE.1))
        .child(heading(7.0, 5.0, HEADING, true))
        .child(rule(3.0, 21.0, 644.0))
        // `pictureBox3`: `Resources.sonar` as its `BackgroundImage`, zoomed, and
        // `Resources.Parachute` as its `Image`, zoomed, over it.
        // C#: GCSViews/ConfigurationView/ConfigHWParachute.Designer.cs:67, 69; ConfigHWParachute.resx:354-355, 414-415
        .child(crate::pictures::layered(
            "parachute",
            "Parachute",
            (11.0, 32.0, 75.0, 69.0),
            ("sonar", crate::pictures::Layout::Zoom),
            ("Parachute", crate::pictures::Layout::ZoomImage),
        ))
        .child(check_box(
            "parachute-CHUTE_ENABLED".to_owned(),
            &parachute.enable,
            ENABLE,
            (103.0, 32.0),
            |this| {
                let jobs = this.extra.parachute.click_enable(Instant::now());
                this.extra.parachute.push(jobs);
            },
            cx,
        ))
        .child(label(102.0, 57.0, "Type", true))
        .child(combo_box(
            "parachute-CHUTE_TYPE".to_owned(),
            &parachute.kind,
            TYPE_AT,
            |this| {
                this.extra
                    .parachute
                    .toggle_dropdown(Which::Type, Instant::now());
            },
            cx,
        ))
        .child(label(102.0, 86.0, "Servo Num", true))
        .child(combo_box(
            "parachute-servo".to_owned(),
            &parachute.servo,
            SERVO_AT,
            |this| {
                this.extra
                    .parachute
                    .toggle_dropdown(Which::Servo, Instant::now());
            },
            cx,
        ));
    for (index, (number, row)) in parachute.numbers.iter().zip(NUMBERS).enumerate() {
        let (label_x, label_y) = row.label_at;
        let (x, y) = row.at;
        body = body
            .child(label(label_x, label_y, row.label, true))
            .child(number_box(
                format!("parachute-{}", row.param),
                number,
                parachute.editing == Some(index),
                &focus.number,
                (x, y, 120.0, 21.0),
                NumberHandlers {
                    begin: move |this: &mut MissionPlanner| {
                        this.extra.parachute.begin(index, Instant::now());
                    },
                    key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
                        this.extra.parachute.key(event, Instant::now())
                    },
                    step: move |this: &mut MissionPlanner, up: bool| {
                        this.extra.parachute.step(index, up, Instant::now());
                    },
                },
                window,
                cx,
            ));
    }
    if let Some(which) = parachute.dropdown {
        let (id, (x, y, width, height)) = match which {
            Which::Type => ("parachute-CHUTE_TYPE", TYPE_AT),
            Which::Servo => ("parachute-servo", SERVO_AT),
        };
        body = body.child(dropdown(
            id,
            parachute.combo(which),
            (x, y + height, width),
            move |this, key| {
                let view = this.telemetry.view();
                let jobs = this.extra.parachute.choose(which, key, &view.parameters);
                this.extra.parachute.push(jobs);
            },
            move |this, lines| this.extra.parachute.scroll_list(which, lines),
            cx,
        ));
    }
    panel(TITLE, body).into_any_element()
}

/// The question or message box showing, over the whole window.
pub fn overlay(
    parachute: &Parachute,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(question) = parachute.question() {
        let buttons = vec![
            action(
                "parachute-question-yes",
                "Yes",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.extra.parachute.answer(true, Instant::now());
                    cx.notify();
                }),
            ),
            action(
                "parachute-question-no",
                "No",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.extra.parachute.answer(false, Instant::now());
                    cx.notify();
                }),
            ),
        ];
        return Some(modal(
            "parachute-question",
            OUT_OF_RANGE_TITLE,
            &question.text(),
            false,
            buttons,
            window,
        ));
    }
    let message = parachute.message()?;
    Some(message_box(
        "parachute-message",
        "parachute-message-ok",
        message,
        window,
        |this| this.extra.parachute.dismiss_message(),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::failsafe::CheckState;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::tests::{Answering, drain};
    use crate::config::optional::{Event, SetQueue};
    use crate::config::servo_output::WRITE_DELAY;
    use mp_link::requests::RequestOutcome;

    fn bundled(name: &str) -> Option<&'static mp_params::ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    fn table(entries: &[(&str, f64)]) -> Vec<(String, f64)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn run(jobs: Vec<Job>, link: &Answering) -> Vec<Message> {
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        drain(&mut queue, link)
    }

    /// A copter from before 3.5, parachute on RC10: the names "Servo Num" lists.
    fn old_copter() -> Vec<(String, f64)> {
        table(&[
            ("CHUTE_ENABLED", 1.0),
            ("CHUTE_TYPE", 10.0),
            ("CHUTE_SERVO_OFF", 1100.0),
            ("CHUTE_SERVO_ON", 1900.0),
            ("CHUTE_ALT_MIN", 10.0),
            ("RC9_FUNCTION", 0.0),
            ("RC10_FUNCTION", 27.0),
            ("RC11_FUNCTION", 0.0),
            ("RC12_FUNCTION", 27.0),
        ])
    }

    /// The `.resx`'s words and places, read from the tree when it is here.
    #[test]
    fn the_text_is_the_resx_text() {
        let Some(resx) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigHWParachute.resx",
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let get = |key: &str| values.get(key).map(String::as_str);
        assert_eq!(get("label1.Text"), Some(HEADING));
        assert_eq!(get("mavlinkCheckBoxEnable.Text"), Some(ENABLE));
        assert_eq!(get("mavlinkCheckBoxEnable.Location"), Some("103, 32"));
        assert_eq!(get("label2.Text"), Some("Type"));
        assert_eq!(get("mavlinkComboBoxType.Location"), Some("186, 54"));
        assert_eq!(get("label3.Text"), Some("Servo Num"));
        assert_eq!(get("mavlinkComboBoxServoNum.Location"), Some("186, 83"));
        for (index, item) in SERVOS.iter().enumerate() {
            let key = if index == 0 {
                "mavlinkComboBoxServoNum.Items".to_owned()
            } else {
                format!("mavlinkComboBoxServoNum.Items{index}")
            };
            assert_eq!(get(&key), Some(*item));
        }
        for (row, (label, number)) in NUMBERS.iter().zip([
            ("label4", "mavlinkNumericUpDownResting"),
            ("label5", "mavlinkNumericUpDownDeploy"),
            ("label6", "mavlinkNumericUpDownMinAlt"),
        ]) {
            assert_eq!(get(&format!("{label}.Text")), Some(row.label));
            let (x, y) = row.label_at;
            assert_eq!(
                get(&format!("{label}.Location")),
                Some(format!("{x}, {y}").as_str())
            );
            let (x, y) = row.at;
            assert_eq!(
                get(&format!("{number}.Location")),
                Some(format!("{x}, {y}").as_str())
            );
        }
        assert_eq!(get("pictureBox3.Location"), Some("11, 32"));
        assert_eq!(get("$this.Size"), Some("650, 212"));
    }

    /// SITL's copter: `CHUTE_ENABLED` 0 and none of the rest - the parachute's parameters show
    /// only once it is enabled - and `SERVOn` outputs, none on 27.
    #[test]
    fn activate_on_the_sitl_copter() {
        let mut page = Parachute::default();
        let parameters = table(&[
            ("CHUTE_ENABLED", 0.0),
            ("SERVO9_FUNCTION", 0.0),
            ("SERVO1_FUNCTION", 33.0),
        ]);
        let jobs = page.activate(&parameters, key(), bundled);
        assert!(jobs.is_empty());
        assert!(page.is_active());
        assert!(page.enable().enabled);
        assert_eq!(page.enable().state, CheckState::Unchecked);
        assert!(!page.combo(Which::Type).enabled);
        assert_eq!(page.combo(Which::Type).options.len(), 5);
        assert!(page.combo(Which::Servo).enabled, "the Designer's, always");
        assert_eq!(page.combo(Which::Servo).selected, None);
        assert!(page.numbers().iter().all(|number| !number.enabled));
    }

    /// An older copter with every parameter binds every control; "Servo Num" shows the first
    /// output on 27, and the first `Activate` writes nothing.
    #[test]
    fn activate_binds_every_control_and_reads_the_servo() {
        let mut page = Parachute::default();
        let jobs = page.activate(&old_copter(), key(), bundled);
        assert!(jobs.is_empty(), "startup is true for the first");
        assert_eq!(page.enable().state, CheckState::Checked);
        assert_eq!(page.combo(Which::Type).text(), "Servo");
        assert_eq!(page.combo(Which::Servo).text(), "RC10");
        let shown: Vec<&str> = page.numbers().iter().map(Number::shown).collect();
        assert_eq!(shown, ["1100", "1900", "10"]);
    }

    /// A name on 27 the combo does not list selects nothing.
    #[test]
    fn a_servo_the_list_lacks_selects_nothing() {
        let parameters = table(&[("SERVO9_FUNCTION", 27.0), ("CHUTE_ENABLED", 1.0)]);
        assert_eq!(servo_on_parachute(&parameters).as_deref(), Some("SERVO9"));
        let mut page = Parachute::default();
        page.activate(&parameters, key(), bundled);
        assert_eq!(page.combo(Which::Servo).selected, None);
    }

    /// Choosing RC11: RC12 - the other listed output on 27 - is set to 0, then RC11 to 27; RC10,
    /// on 27 too, is also cleared, as the C# clears every one but the chosen.
    #[test]
    fn choosing_a_servo_moves_the_parachute_there() {
        let mut page = Parachute::default();
        let parameters = old_copter();
        page.activate(&parameters, key(), bundled);
        page.toggle_dropdown(Which::Servo, Instant::now());
        let jobs = page.choose(Which::Servo, 2, &parameters);
        let link = Answering::new(&[]);
        let messages = run(jobs, &link);
        assert!(messages.is_empty());
        assert_eq!(
            link.taken(),
            [
                ("RC10_FUNCTION".to_owned(), 0.0),
                ("RC12_FUNCTION".to_owned(), 0.0),
                ("RC11_FUNCTION".to_owned(), 27.0)
            ]
        );
        // The same one again changes no selection: nothing.
        assert!(page.choose(Which::Servo, 2, &parameters).is_empty());
    }

    /// On firmware with `SERVOn` outputs the `RCn_FUNCTION` the handler sets is not listed:
    /// `setParam`'s false, ignored - no box.
    #[test]
    fn a_servo_on_new_firmware_writes_a_name_the_vehicle_lacks_silently() {
        let mut page = Parachute::default();
        let parameters = table(&[("CHUTE_ENABLED", 0.0), ("SERVO9_FUNCTION", 0.0)]);
        page.activate(&parameters, key(), bundled);
        let jobs = page.choose(Which::Servo, 0, &parameters);
        let unknown = Progress::Finished(RequestOutcome::UnknownParameter);
        let link = Answering::new(&[("RC9_FUNCTION", unknown)]);
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        let messages = drain(&mut queue, &link);
        assert!(messages.is_empty());
        assert_eq!(queue.last(), Some("RC9_FUNCTION 27 false"));
    }

    /// The handler has no `try`: a timeout is the unhandled-exception box and ends it.
    #[test]
    fn a_timeout_in_the_servo_handler_is_unhandled() {
        let mut page = Parachute::default();
        let parameters = old_copter();
        page.activate(&parameters, key(), bundled);
        let jobs = page.choose(Which::Servo, 0, &parameters);
        let link = Answering::new(&[(
            "RC10_FUNCTION",
            Progress::Finished(RequestOutcome::TimedOut),
        )]);
        let messages = run(jobs, &link);
        assert_eq!(link.taken().len(), 1, "nothing after the throw");
        assert_eq!(messages, [unhandled("RC10_FUNCTION")]);
    }

    /// A later `Activate` whose vehicle has the parachute elsewhere selects it - and, `startup`
    /// being false by then, runs the handler as the C#'s `Text` setter does.
    #[test]
    fn a_later_activate_that_moves_the_selection_runs_the_handler() {
        let mut page = Parachute::default();
        page.activate(&old_copter(), key(), bundled);
        page.hide(Instant::now());
        let moved = table(&[("RC9_FUNCTION", 27.0), ("RC10_FUNCTION", 0.0)]);
        let jobs = page.activate(&moved, key(), bundled);
        assert_eq!(page.combo(Which::Servo).text(), "RC9");
        let link = Answering::new(&[]);
        run(jobs, &link);
        assert_eq!(link.taken(), [("RC9_FUNCTION".to_owned(), 27.0)]);
        // And one that finds it where it is writes nothing.
        page.hide(Instant::now());
        assert!(page.activate(&moved, key(), bundled).is_empty());
    }

    /// "Enable" is the `MavlinkCheckBox`'s own write, "Set CHUTE_ENABLED Failed" on a timeout.
    #[test]
    fn enable_writes_chute_enabled() {
        let mut page = Parachute::default();
        page.activate(&table(&[("CHUTE_ENABLED", 0.0)]), key(), bundled);
        let jobs = page.click_enable(Instant::now());
        let link = Answering::new(&[]);
        assert!(run(jobs, &link).is_empty());
        assert_eq!(link.taken(), [("CHUTE_ENABLED".to_owned(), 1.0)]);
        let jobs = page.click_enable(Instant::now());
        let link = Answering::new(&[(
            "CHUTE_ENABLED",
            Progress::Finished(RequestOutcome::TimedOut),
        )]);
        assert_eq!(
            run(jobs, &link),
            [crate::config::optional::error("Set CHUTE_ENABLED Failed")]
        );
    }

    /// The type: the combo's own write, the chosen value.
    #[test]
    fn choosing_a_type_writes_it() {
        let mut page = Parachute::default();
        let parameters = old_copter();
        page.activate(&parameters, key(), bundled);
        page.toggle_dropdown(Which::Type, Instant::now());
        let jobs = page.choose(Which::Type, 1, &parameters);
        let link = Answering::new(&[]);
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        let mut messages = VecDeque::new();
        let events = queue.advance(&link, &mut messages);
        assert!(events.iter().any(|event| matches!(
            event,
            Event::Set { param, value, .. } if param == "CHUTE_TYPE" && (*value - 1.0).abs() < 1e-9
        )));
        assert_eq!(page.combo(Which::Type).text(), "Second Relay");
    }

    /// A number steps and writes 300 ms later, as `MavlinkNumericUpDown`'s timer does.
    #[test]
    fn a_number_writes_after_its_timer() {
        let mut page = Parachute::default();
        page.activate(&old_copter(), key(), bundled);
        let start = Instant::now();
        page.step(1, true, start);
        assert_eq!(page.numbers()[1].shown(), "1901");
        let mut numbers = page.numbers;
        assert!(numbers[1].due(start).is_none());
        let write = numbers[1].due(start + WRITE_DELAY).expect("due");
        assert_eq!(write.param, "CHUTE_SERVO_ON");
        assert!((write.value - 1901.0).abs() < 1e-9);
    }

    /// Every fact the GUI script asserts on is one this page records, and every control it
    /// clicks is one this page draws.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-parachute.gui");
        let source = include_str!("parachute.rs");
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.parachute.") => {
                    let generic = ["type", "servo"].iter().fold(key.to_owned(), |key, which| {
                        key.replace(
                            &format!("config.parachute.{which}"),
                            "config.parachute.{key}",
                        )
                    });
                    let numbered = NUMBERS.iter().fold(generic.clone(), |key, row| {
                        key.replace(
                            &format!("config.parachute.{}", row.param.to_ascii_lowercase()),
                            "config.parachute.{key}",
                        )
                    });
                    assert!(
                        source.contains(&format!("\"{key}\""))
                            || source.contains(&format!("\"{generic}\""))
                            || source.contains(&format!("\"{numbered}\"")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("parachute-") => {
                    let base = id
                        .strip_suffix("-up")
                        .or_else(|| id.strip_suffix("-down"))
                        .unwrap_or(id);
                    assert!(
                        source.contains(&format!("\"{base}"))
                            || NUMBERS
                                .iter()
                                .any(|row| base == format!("parachute-{}", row.param)),
                        "{id} is not drawn"
                    );
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 8 && clicks >= 2, "{facts} facts, {clicks} clicks");
    }
}
