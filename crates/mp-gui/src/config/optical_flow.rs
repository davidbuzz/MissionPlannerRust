//! Optical Flow: `GCSViews/ConfigurationView/ConfigHWOptFlow.cs`, an Optional Hardware page of
//! Initial Setup (`GCSViews/InitialSetup.cs:299-302`), listed once every parameter is in.
//!
//! What it shows depends on the firmware. Firmware old enough to have `FLOW_ENABLE` gets the
//! legacy page: "Enable" alone, the new-style controls hidden. Anything later gets the sensor type
//! (`FLOW_TYPE`), its yaw (`FLOW_ORIENT_YAW`, in centidegrees, shown in degrees), the two scale
//! factors (`FLOW_FXSCALER`, `FLOW_FYSCALER`), the sensor's position (`FLOW_POS_X/Y/Z`) and, on a
//! rover, the height override (`FLOW_HGT_OVR`) - "Enable" hidden (`ConfigHWOptFlow.cs:17-75`).
//! Each is a `Mavlink*` control and writes its parameter when it changes, a number 300 ms after.
//! "Enable" has the page's handler too, which runs first and writes `FLOW_ENABLE` (`:77-96`); the
//! type combo's handler shows or hides the height override by firmware (`:98-113`).
//!
//! The layout is `ConfigHWOptFlow.resx`'s, every control at its `Location` in a 650 x 360 page.
//!
//! What is not ported, and why:
//!
//! * `pictureBox2`'s image (`Resources.opticalflow`), a resource this application does not carry;
//! * the yaw box's `Minimum = -179` holding a yaw below -179 degrees: the C# sets its bounds after
//!   `setup` has attached the write, so a vehicle holding -179.5 has -179 written back the moment
//!   the page opens. Here the value is shown as the vehicle holds it and nothing is written that
//!   the pilot did not change; the bounds apply to what is typed and stepped.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::time::Instant;

use gpui::{AnyElement, Context, KeyDownEvent, Window, div, prelude::*, px};

use super::optional::{
    Focus, Job, Set, SetQueue, has, heading, label, message_box, picture, plain, rule, value_of,
};
use crate::MissionPlanner;
use crate::config::extra_setup::{link_error, take_link_errors};
use crate::config::failsafe::{CheckState, Lookup, options};
use crate::config::flight_modes::Firmware;
use crate::config::flight_modes::ParamWriter;
use crate::config::servo_output::{
    Check, Combo, Message, NUMERIC_DEFAULTS, Number, NumberHandlers, OUT_OF_RANGE_TITLE, Question,
    Setup, Write, check_box, combo_box, dropdown, modal, number_box,
};
use crate::setup::Key;
use crate::telemetry::TelemetryView;
use crate::ui::{action, panel, theme};

/// The page's title in Initial Setup's list, `backstageViewPageoptflow.Text`.
/// `// C#: GCSViews/InitialSetup.resx:702-704`
pub const TITLE: &str = "Optical Flow";

/// `label3.Text`, the heading, trailing spaces and all.
/// `// C#: GCSViews/ConfigurationView/ConfigHWOptFlow.resx label3.Text`
pub const HEADING: &str = "Optical Flow  ";

/// One of the seven numbers: its parameter, `setup`'s arguments, its `Location`, and the ids of
/// its labels.
#[derive(Debug, Clone, Copy)]
pub struct NumberRow {
    /// The parameter.
    pub param: &'static str,
    /// `setup(Min, Max, Scale, Increment, ...)`.
    pub setup: Setup,
    /// The box's `Location`; every box is 120 x 20.
    pub at: (f32, f32),
}

/// `mavlinkNumericUpDown_yaw`, `FX`, `FY`, `X`, `Y`, `Z` and `HGTOVR`, in that order.
/// `// C#: GCSViews/ConfigurationView/ConfigHWOptFlow.cs:53-64; ConfigHWOptFlow.resx *.Location`
pub const NUMBERS: [NumberRow; 7] = [
    NumberRow {
        param: "FLOW_ORIENT_YAW",
        setup: Setup {
            minimum: -179_000.0,
            maximum: 180_000.0,
            scale: 100.0,
            increment: 1.0,
        },
        at: (209.0, 92.0),
    },
    NumberRow {
        param: "FLOW_FXSCALER",
        setup: Setup {
            minimum: -200.0,
            maximum: 200.0,
            scale: 1.0,
            increment: 1.0,
        },
        at: (209.0, 140.0),
    },
    NumberRow {
        param: "FLOW_FYSCALER",
        setup: Setup {
            minimum: -200.0,
            maximum: 200.0,
            scale: 1.0,
            increment: 1.0,
        },
        at: (209.0, 166.0),
    },
    NumberRow {
        param: "FLOW_POS_X",
        setup: POSITION,
        at: (206.0, 236.0),
    },
    NumberRow {
        param: "FLOW_POS_Y",
        setup: POSITION,
        at: (206.0, 262.0),
    },
    NumberRow {
        param: "FLOW_POS_Z",
        setup: POSITION,
        at: (206.0, 288.0),
    },
    NumberRow {
        param: "FLOW_HGT_OVR",
        setup: Setup {
            minimum: 0.0,
            maximum: 2.0,
            scale: 1.0,
            increment: 0.01,
        },
        at: (206.0, 328.0),
    },
];

/// `setup(-5, 5, 1, 0.01F, ...)` for the three positions.
const POSITION: Setup = Setup {
    minimum: -5.0,
    maximum: 5.0,
    scale: 1.0,
    increment: 0.01,
};

/// The yaw's bounds and step, set after its `setup`.
/// `// C#: GCSViews/ConfigurationView/ConfigHWOptFlow.cs:54-56`
const YAW_MAXIMUM: f64 = 180.0;
/// Its minimum.
const YAW_MINIMUM: f64 = -179.0;

/// The height override's index in [`NUMBERS`].
const HEIGHT_OVERRIDE: usize = 6;

/// The labels, with their `Location`s: `label1`, `label2`, `label4` to `label14`.
/// `// C#: GCSViews/ConfigurationView/ConfigHWOptFlow.resx label*.Text, label*.Location`
const LABELS: [(f32, f32, &str); 13] = [
    (92.0, 68.0, "Type"),
    (92.0, 94.0, "Yaw Orientation"),
    (92.0, 142.0, "FX Scale"),
    (92.0, 168.0, "FY Scale"),
    (89.0, 238.0, "Position X"),
    (89.0, 264.0, "Position Y"),
    (89.0, 290.0, "Position Z"),
    (335.0, 97.0, "degrees, relative to vehicle"),
    (330.0, 238.0, "metres foward"),
    (330.0, 264.0, "metres right"),
    (330.0, 290.0, "metres down"),
    (163.0, 211.0, "Position of Sensor, relative to IMU"),
    (204.0, 121.0, "Scaling Factors"),
];

/// `label15` and `label16`, beside the height override, which hide with it.
const HEIGHT_LABELS: [(f32, f32, &str); 2] = [
    (88.0, 330.0, "Height Override"),
    (332.0, 330.0, "metres from ground"),
];

/// `CHK_enableoptflow_CheckedChanged`'s `catch`, a box with no caption.
/// `// C#: GCSViews/ConfigurationView/ConfigHWOptFlow.cs:94`
pub const FLOW_ENABLE_FAILED: &str = "Set FLOW_ENABLE Failed";

/// No documentation, for the yaw, whose bounds and step the page sets itself.
fn undocumented(_: &str) -> Option<&'static mp_params::ParamMeta> {
    None
}

/// The page object. `H` is what the link knows a write by: the vehicle's request, or a test's.
#[derive(Debug)]
pub struct OpticalFlow<H = mp_link::RequestId> {
    made_for: Option<Key>,
    active: bool,
    /// The page's `Enabled`.
    enabled: bool,
    /// `startup`.
    startup: bool,
    /// `CHK_enableoptflow`, and whether it shows.
    enable: Check,
    enable_visible: bool,
    /// `DROP_optflowtype`, and whether it shows.
    kind: Combo,
    kind_visible: bool,
    /// The seven numbers, in [`NUMBERS`]' order, and whether each shows.
    numbers: [Number; 7],
    visible: [bool; 7],
    /// `label15` and `label16`.
    height_labels_visible: bool,
    /// The firmware's name, `cs.firmware`, for "Not Available on".
    firmware: Firmware,
    /// Whether the version banner names ArduRover.
    rover: bool,
    dropdown: bool,
    editing: Option<usize>,
    question: Option<(usize, Question)>,
    messages: VecDeque<Message>,
    /// The last link failure the C# boxes, for the status line (the owner's ruling of
    /// 2026-09-25), until the holder takes it.
    status: Option<String>,
    queue: SetQueue<H>,
}

impl Default for OpticalFlow {
    /// `InitializeComponent`: every control disabled and showing; the numbers at `NumericUpDown`'s
    /// defaults.
    fn default() -> Self {
        Self::blank()
    }
}

impl<H: Copy> OpticalFlow<H> {
    /// `InitializeComponent`, for any link.
    #[must_use]
    pub fn blank() -> Self {
        Self {
            made_for: None,
            active: false,
            enabled: true,
            startup: false,
            enable: Check::default(),
            enable_visible: true,
            kind: Combo::default(),
            kind_visible: true,
            numbers: std::array::from_fn(|_| Number::new(NUMERIC_DEFAULTS)),
            visible: [true; 7],
            height_labels_visible: true,
            firmware: Firmware::ArduCopter2,
            rover: false,
            dropdown: false,
            editing: None,
            question: None,
            messages: VecDeque::new(),
            status: None,
            queue: SetQueue::default(),
        }
    }

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

    /// "Enable".
    #[must_use]
    pub const fn enable(&self) -> &Check {
        &self.enable
    }

    /// The type combo.
    #[must_use]
    pub const fn kind(&self) -> &Combo {
        &self.kind
    }

    /// The numbers.
    #[must_use]
    pub const fn numbers(&self) -> &[Number; 7] {
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

    /// The words of the last link failure since the holder last asked, for the status line.
    pub fn take_status(&mut self) -> Option<String> {
        self.status.take()
    }

    /// The page object disposed, returning what its numbers' timers held.
    fn dispose(&mut self) -> Vec<Write> {
        let pending = self.numbers.iter_mut().filter_map(Number::flush).collect();
        let messages = std::mem::take(&mut self.messages);
        let status = self.status.take();
        let queue = std::mem::take(&mut self.queue);
        *self = Self {
            messages,
            status,
            queue,
            ..Self::blank()
        };
        pending
    }

    /// Shows the page: a new page object for a new screen, then `Activate`.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWOptFlow.cs:17-75`
    pub fn activate(
        &mut self,
        parameters: &[(String, f64)],
        key: Key,
        connected: bool,
        banner: Option<&str>,
        firmware: Firmware,
        lookup: Lookup,
    ) {
        if self.made_for != Some(key) {
            let pending: Vec<Job> = self.dispose().into_iter().map(Job::control).collect();
            self.queue.push(pending);
            self.made_for = Some(key);
        }
        self.active = true;
        self.dropdown = false;
        self.firmware = firmware;
        self.rover = banner.is_some_and(|banner| banner.contains("ArduRover"));
        if !connected {
            self.enabled = false;
            return;
        }
        self.enabled = true;
        self.startup = true;

        // `// C#: :29-46`
        self.enable.setup(1.0, 0.0, "FLOW_ENABLE", parameters);
        self.enable_visible |= has(parameters, "FLOW_ENABLE");
        if self.enable.enabled {
            self.kind_visible = false;
            for visible in self.visible.iter_mut().take(HEIGHT_OVERRIDE) {
                *visible = false;
            }
            self.startup = false;
            return;
        }
        self.enable_visible = false;

        // `// C#: :50-64`
        self.kind
            .setup(options("FLOW_TYPE", lookup), "FLOW_TYPE", parameters);
        self.kind_visible |= has(parameters, "FLOW_TYPE");
        for (index, (number, row)) in self.numbers.iter_mut().zip(NUMBERS).enumerate() {
            let documented: Lookup = if index == 0 { undocumented } else { lookup };
            number.setup(row.setup, row.param, parameters, documented);
            if let Some(visible) = self.visible.get_mut(index) {
                *visible |= has(parameters, row.param);
            }
        }
        // `Maximum = 180; Minimum = -179; Increment = 1;` - the step is `setup`'s 1 already, the
        // yaw having been set up without its documentation's 10. See the module's notes.
        if let Some(yaw) = self.numbers.first_mut() {
            yaw.maximum = YAW_MAXIMUM;
            yaw.minimum = YAW_MINIMUM;
        }

        // `// C#: :66-72`
        if !self.rover {
            self.hide_height_override();
        }
        self.startup = false;
    }

    fn hide_height_override(&mut self) {
        if let Some(visible) = self.visible.get_mut(HEIGHT_OVERRIDE) {
            *visible = false;
        }
        self.height_labels_visible = false;
    }

    /// The page hidden - it is `IActivate` only - which reads a number being typed into.
    pub fn hide(&mut self, now: Instant) {
        self.active = false;
        self.dropdown = false;
        self.leave(now);
    }

    /// A click on "Enable": the page's handler, then the control's own write.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWOptFlow.cs:77-96`
    pub fn click_enable(&mut self, parameters: &[(String, f64)], now: Instant) -> Vec<Job> {
        self.leave(now);
        self.dropdown = false;
        if !self.enabled || !self.enable_visible {
            return Vec::new();
        }
        let Some(write) = self.enable.click() else {
            return Vec::new();
        };
        let mut jobs = Vec::new();
        if !self.startup {
            if value_of(parameters, "FLOW_ENABLE").is_none() {
                // Kept a box: not the link failing but the firmware lacking the parameter, which
                // nothing else in the window shows (see `tick`).
                jobs.push(Job::show(
                    "enable",
                    plain(format!("Not Available on {}", self.firmware.label())),
                ));
            } else {
                let checked = self.enable.state == CheckState::Checked;
                jobs.push(Job::new(
                    "enable",
                    [Set {
                        on_throw: Some(plain(FLOW_ENABLE_FAILED)),
                        ..Set::plain("FLOW_ENABLE", if checked { 1.0 } else { 0.0 })
                    }],
                ));
            }
        }
        jobs.push(Job::control(write));
        jobs
    }

    /// Drops the type's list down, or back up.
    pub fn toggle_dropdown(&mut self, now: Instant) {
        self.leave(now);
        self.dropdown = !self.dropdown && self.enabled && self.kind.enabled;
        if self.dropdown {
            self.kind.open_list();
        }
    }

    /// The wheel over the list.
    pub fn scroll_list(&mut self, lines: i32) {
        if self.dropdown {
            self.kind.scroll_list(lines);
        }
    }

    /// A type chosen: the page's handler - the height override shown on a rover, hidden
    /// otherwise - then the combo's own write.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWOptFlow.cs:98-113`
    pub fn choose(&mut self, key: i64) -> Vec<Job> {
        self.dropdown = false;
        if !self.enabled {
            return Vec::new();
        }
        let Some(write) = self.kind.choose(key) else {
            return Vec::new();
        };
        if self.rover {
            if let Some(visible) = self.visible.get_mut(HEIGHT_OVERRIDE) {
                *visible = true;
            }
            self.height_labels_visible = true;
        } else {
            self.hide_height_override();
        }
        vec![Job::control(write)]
    }

    /// Whether a number takes clicks: enabled, showing, on an enabled page.
    fn number_live(&self, index: usize) -> bool {
        self.enabled
            && self.visible.get(index).copied().unwrap_or(false)
            && self.numbers.get(index).is_some_and(|number| number.enabled)
    }

    /// A number clicked into.
    pub fn begin(&mut self, index: usize, now: Instant) {
        if self.editing == Some(index) {
            return;
        }
        self.leave(now);
        self.dropdown = false;
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
    pub fn tick<W: ParamWriter<Handle = H>>(
        &mut self,
        writer: &W,
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
        self.queue.advance(writer, &mut self.messages);
        // The owner's ruling of 2026-09-25 (PLAN.md §12): the C#'s boxes for the link failing go
        // on the status line - the handler's `catch`, "Set FLOW_ENABLE Failed", a box with no
        // caption (`ConfigHWOptFlow.cs:94`), and the controls' own `Strings.ERROR` boxes, "Set X
        // Failed" / "Set X Failed!" (`Controls/MavlinkCheckBox.cs:118, 124, 134, 140`,
        // `Controls/MavlinkComboBox.cs:182, 197`, `Controls/MavlinkNumericUpDown.cs:171, 175`).
        // "Not Available on" the firmware (`ConfigHWOptFlow.cs:85`) says why the click did
        // nothing on this vehicle, not that the link failed, and a number's out-of-range question
        // (`Controls/MavlinkNumericUpDown.cs:139`) is a question: they keep their boxes.
        let failure = |message: &Message| link_error(message) || message.text == FLOW_ENABLE_FAILED;
        if let Some(words) = take_link_errors(&mut self.messages, failure) {
            self.status = Some(words);
        }
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(page: &OpticalFlow, view: &TelemetryView) {
    use crate::facts::record;
    record("config.optflow.active", page.is_active());
    record("config.optflow.enabled", page.enabled());
    record("config.optflow.heading", HEADING.trim_end());
    record("config.optflow.enable", page.enable().state.key());
    record("config.optflow.enable.visible", page.enable_visible);
    record("config.optflow.enable.enabled", page.enable().enabled);
    record(
        "config.optflow.type",
        page.kind()
            .selected
            .map_or_else(|| "none".to_owned(), |value| value.to_string()),
    );
    record("config.optflow.type.text", page.kind().text());
    record("config.optflow.type.enabled", page.kind().enabled);
    record("config.optflow.type.visible", page.kind_visible);
    for ((number, row), visible) in page.numbers().iter().zip(NUMBERS).zip(page.visible) {
        let key = row.param.to_ascii_lowercase();
        record(format!("config.optflow.{key}"), number.shown());
        record(format!("config.optflow.{key}.enabled"), number.enabled);
        record(format!("config.optflow.{key}.visible"), visible);
    }
    record(
        "config.optflow.height.labels.visible",
        page.height_labels_visible,
    );
    record("config.optflow.write", page.queue.last().unwrap_or("none"));
    record("config.optflow.writes.pending", page.queue.pending());
    record(
        "config.optflow.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    let names = ["FLOW_ENABLE", "FLOW_TYPE"]
        .into_iter()
        .chain(NUMBERS.iter().map(|row| row.param));
    for name in names {
        if let Some(value) = value_of(&view.parameters, name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

/// The page, laid out as `ConfigHWOptFlow.resx` lays it out.
pub fn page(
    flow: &OpticalFlow,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if !flow.is_active() {
        return div().into_any_element();
    }
    let enabled = flow.enabled;
    let mut body = div()
        .relative()
        .w(px(650.0))
        .h(px(360.0))
        .child(heading(7.0, 5.0, HEADING, enabled))
        .child(rule(3.0, 23.0, 644.0))
        .child(picture("opticalflow", (11.0, 35.0, 75.0, 75.0)));
    for (x, y, text) in LABELS {
        body = body.child(label(x, y, text, enabled));
    }
    if flow.height_labels_visible {
        for (x, y, text) in HEIGHT_LABELS {
            body = body.child(label(x, y, text, enabled));
        }
    }
    if flow.enable_visible {
        let mut check = flow.enable.clone();
        check.enabled &= enabled;
        body = body.child(check_box(
            "optflow-FLOW_ENABLE".to_owned(),
            &check,
            "Enable",
            (92.0, 35.0),
            |this| {
                let view = this.telemetry.view();
                let jobs = this
                    .optional
                    .optflow
                    .click_enable(&view.parameters, Instant::now());
                this.optional.optflow.push(jobs);
            },
            cx,
        ));
    }
    if flow.kind_visible {
        let mut kind = flow.kind.clone();
        kind.enabled &= enabled;
        body = body.child(combo_box(
            "optflow-FLOW_TYPE".to_owned(),
            &kind,
            (209.0, 65.0, 121.0, 21.0),
            |this| this.optional.optflow.toggle_dropdown(Instant::now()),
            cx,
        ));
    }
    for (index, (number, row)) in flow.numbers.iter().zip(NUMBERS).enumerate() {
        if !flow.visible.get(index).copied().unwrap_or(false) {
            continue;
        }
        let (x, y) = row.at;
        let shown;
        let number = if enabled {
            number
        } else {
            shown = disabled_copy(number);
            &shown
        };
        body = body.child(number_box(
            format!("optflow-{}", row.param),
            number,
            flow.editing == Some(index),
            &focus.number,
            (x, y, 120.0, 20.0),
            NumberHandlers {
                begin: move |this: &mut MissionPlanner| {
                    this.optional.optflow.begin(index, Instant::now());
                },
                key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
                    this.optional.optflow.key(event, Instant::now())
                },
                step: move |this: &mut MissionPlanner, up: bool| {
                    this.optional.optflow.step(index, up, Instant::now());
                },
            },
            window,
            cx,
        ));
    }
    if flow.dropdown {
        body = body.child(dropdown(
            "optflow-FLOW_TYPE",
            &flow.kind,
            (209.0, 86.0, 121.0),
            |this, key| {
                let jobs = this.optional.optflow.choose(key);
                this.optional.optflow.push(jobs);
            },
            |this, lines| this.optional.optflow.scroll_list(lines),
            cx,
        ));
    }
    panel(TITLE, body).into_any_element()
}

/// A number as a disabled page shows it: its text, inert.
fn disabled_copy(number: &Number) -> Number {
    let mut copy = Number::new(crate::config::servo_output::Designer {
        minimum: number.minimum,
        maximum: number.maximum,
        value: number.shown().parse().unwrap_or(0.0),
        decimals: number.decimals,
    });
    copy.enabled = false;
    copy
}

/// The question or message box showing, over the whole window.
pub fn overlay(
    flow: &OpticalFlow,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(question) = flow.question() {
        let buttons = vec![
            action(
                "optflow-question-yes",
                "Yes",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.optional.optflow.answer(true, Instant::now());
                    cx.notify();
                }),
            ),
            action(
                "optflow-question-no",
                "No",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.optional.optflow.answer(false, Instant::now());
                    cx.notify();
                }),
            ),
        ];
        return Some(modal(
            "optflow-question",
            OUT_OF_RANGE_TITLE,
            &question.text(),
            false,
            buttons,
            window,
        ));
    }
    let message = flow.message()?;
    Some(message_box(
        "optflow-message",
        "optflow-message-ok",
        message,
        window,
        |this| this.optional.optflow.dismiss_message(),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::optional::tests::{Answering, drain};
    use crate::config::servo_output::WRITE_DELAY;
    use crate::telemetry::Telemetry;

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

    const COPTER: Option<&str> = Some("ArduCopter V4.5.7 (2a3dc4b7)");

    fn run(jobs: Vec<Job>, link: &Answering) -> Vec<Message> {
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        drain(&mut queue, link)
    }

    fn configured() -> Vec<(String, f64)> {
        table(&[
            ("FLOW_TYPE", 1.0),
            ("FLOW_ORIENT_YAW", 4500.0),
            ("FLOW_FXSCALER", 10.0),
            ("FLOW_FYSCALER", -5.0),
            ("FLOW_POS_X", 0.05),
            ("FLOW_POS_Y", 0.0),
            ("FLOW_POS_Z", -0.02),
        ])
    }

    #[test]
    fn the_text_is_the_resx_text() {
        let Some(resx) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigHWOptFlow.resx",
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let get = |key: &str| values.get(key).map(String::as_str);
        assert_eq!(get("label3.Text"), Some(HEADING));
        assert_eq!(get("CHK_enableoptflow.Location"), Some("92, 35"));
        assert_eq!(get("DROP_optflowtype.Location"), Some("209, 65"));
        let names = [
            "mavlinkNumericUpDown_yaw",
            "mavlinkNumericUpDownFX",
            "mavlinkNumericUpDownFY",
            "mavlinkNumericUpDownX",
            "mavlinkNumericUpDownY",
            "mavlinkNumericUpDownZ",
            "mavlinkNumericUpDownHGTOVR",
        ];
        for (name, row) in names.iter().zip(NUMBERS) {
            let (x, y) = row.at;
            assert_eq!(
                get(&format!("{name}.Location")),
                Some(format!("{x}, {y}").as_str()),
                "{name}"
            );
        }
        let texts: Vec<&str> = LABELS
            .iter()
            .chain(HEIGHT_LABELS.iter())
            .map(|(_, _, text)| *text)
            .collect();
        for index in [1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16] {
            let text = get(&format!("label{index}.Text")).expect("a label");
            assert!(texts.contains(&text), "label{index} {text:?}");
        }
    }

    /// SITL's copter: no `FLOW_ENABLE`, so the new-style page; `FLOW_TYPE` 0 and none of the
    /// sensor's other parameters, which the firmware lists only once a type is set.
    #[test]
    fn the_sitl_copter_gets_the_new_style_page() {
        let mut page = OpticalFlow::default();
        page.activate(
            &table(&[("FLOW_TYPE", 0.0)]),
            key(),
            true,
            COPTER,
            Firmware::ArduCopter2,
            bundled,
        );
        assert!(!page.enable_visible, "FLOW_ENABLE is only on old firmware");
        assert!(page.kind_visible && page.kind().enabled);
        assert_eq!(page.kind().text(), "None");
        assert!(page.numbers().iter().all(|number| !number.enabled));
        assert!(
            !page.visible[HEIGHT_OVERRIDE] && !page.height_labels_visible,
            "not a rover"
        );
        // The disabled boxes still show; only the height override hides.
        assert!(
            page.visible[..HEIGHT_OVERRIDE]
                .iter()
                .all(|visible| *visible)
        );
    }

    #[test]
    fn a_configured_sensor_shows_its_numbers_in_their_units() {
        let mut page = OpticalFlow::default();
        page.activate(
            &configured(),
            key(),
            true,
            COPTER,
            Firmware::ArduCopter2,
            bundled,
        );
        let shown: Vec<&str> = page.numbers().iter().map(Number::shown).collect();
        // The yaw in degrees, stepped by one between -179 and 180; the positions to two places.
        assert_eq!(shown[..6], ["45", "10", "-5", "0.05", "0.00", "-0.02"]);
        let yaw = &page.numbers()[0];
        assert!((yaw.minimum - YAW_MINIMUM).abs() < 1e-9);
        assert!((yaw.maximum - YAW_MAXIMUM).abs() < 1e-9);
        let now = Instant::now();
        page.step(0, true, now);
        assert_eq!(page.numbers()[0].shown(), "46");
        let write = page.numbers[0].due(now + WRITE_DELAY).expect("due");
        assert_eq!(write.param, "FLOW_ORIENT_YAW");
        assert!((write.value - 4600.0).abs() < 1e-9, "centidegrees again");
    }

    /// Old firmware: "Enable" alone, the new-style boxes hidden.
    #[test]
    fn flow_enable_gets_the_legacy_page() {
        let mut page = OpticalFlow::default();
        let parameters = table(&[("FLOW_ENABLE", 0.0)]);
        page.activate(
            &parameters,
            key(),
            true,
            COPTER,
            Firmware::ArduCopter2,
            bundled,
        );
        assert!(page.enable_visible && page.enable().enabled);
        assert!(!page.kind_visible);
        assert!(
            page.visible[..HEIGHT_OVERRIDE]
                .iter()
                .all(|visible| !*visible)
        );
        // The page's handler writes first, then the control.
        let jobs = page.click_enable(&parameters, Instant::now());
        let link = Answering::new(&[]);
        assert!(run(jobs, &link).is_empty());
        assert_eq!(
            link.taken(),
            [
                ("FLOW_ENABLE".to_owned(), 1.0),
                ("FLOW_ENABLE".to_owned(), 1.0)
            ]
        );
    }

    /// The handler asks the vehicle's table, not the box: a `FLOW_ENABLE` gone since `Activate`
    /// is "Not Available on" the firmware, and the control's own write still follows.
    #[test]
    fn a_missing_flow_enable_says_not_available() {
        let mut page = OpticalFlow::default();
        page.activate(
            &table(&[("FLOW_ENABLE", 0.0)]),
            key(),
            true,
            COPTER,
            Firmware::ArduCopter2,
            bundled,
        );
        let jobs = page.click_enable(&[], Instant::now());
        let link = Answering::new(&[]);
        let messages = run(jobs, &link);
        assert_eq!(messages, [plain("Not Available on ArduCopter2")]);
        assert_eq!(link.taken(), [("FLOW_ENABLE".to_owned(), 1.0)]);
    }

    /// On a rover the type's handler shows the height override; elsewhere it hides it.
    #[test]
    fn the_height_override_is_a_rovers() {
        let mut page = OpticalFlow::default();
        let mut parameters = configured();
        parameters.push(("FLOW_HGT_OVR".to_owned(), 0.3));
        page.activate(
            &parameters,
            key(),
            true,
            Some("ArduRover V4.5.0"),
            Firmware::ArduRover,
            bundled,
        );
        assert!(page.visible[HEIGHT_OVERRIDE] && page.height_labels_visible);
        assert_eq!(page.numbers()[HEIGHT_OVERRIDE].shown(), "0.30");
        let jobs = page.choose(5);
        assert!(page.visible[HEIGHT_OVERRIDE]);
        let link = Answering::new(&[]);
        run(jobs, &link);
        assert_eq!(link.taken(), [("FLOW_TYPE".to_owned(), 5.0)]);
        let mut copter = OpticalFlow::default();
        copter.activate(
            &parameters,
            key(),
            true,
            COPTER,
            Firmware::ArduCopter2,
            bundled,
        );
        assert!(!copter.visible[HEIGHT_OVERRIDE]);
        copter.choose(5);
        assert!(!copter.visible[HEIGHT_OVERRIDE] && !copter.height_labels_visible);
    }

    #[test]
    fn with_no_link_the_page_is_disabled() {
        let mut page = OpticalFlow::default();
        page.activate(
            &configured(),
            key(),
            false,
            COPTER,
            Firmware::ArduCopter2,
            bundled,
        );
        assert!(!page.enabled());
        assert!(page.choose(2).is_empty());
        page.step(1, true, Instant::now());
        assert_eq!(
            page.numbers()[1].shown(),
            "0",
            "the Designer's value, untouched"
        );
    }

    /// A pending number is written when the page object goes with its screen.
    #[test]
    fn leaving_the_screen_writes_what_a_timer_held() {
        let telemetry = Telemetry::idle();
        let view = telemetry.view();
        let mut page = OpticalFlow::default();
        page.activate(
            &configured(),
            Key::of(&view),
            true,
            COPTER,
            Firmware::ArduCopter2,
            bundled,
        );
        let now = Instant::now();
        page.step(1, true, now);
        page.hide(now);
        page.tick(&telemetry, &view, false, false, now);
        // No vehicle: the control's write finds no such name, and says so - on the status line,
        // not in a box (the owner's ruling of 2026-09-25).
        assert!(page.message().is_none(), "no box: {:?}", page.message());
        assert_eq!(
            page.take_status().as_deref(),
            Some("Set FLOW_FXSCALER Failed")
        );
    }

    /// A timeout, through the page's own tick as the application runs it: the handler's
    /// captionless "Set FLOW_ENABLE Failed" (`ConfigHWOptFlow.cs:94`) and the check box's own
    /// box are the status line's, not boxes - the owner's ruling of 2026-09-25.
    #[test]
    fn a_timeout_is_a_status_line_not_a_box() {
        use crate::config::flight_modes::Progress;
        use mp_link::requests::RequestOutcome;
        let view = TelemetryView::disconnected("test");
        let mut page = OpticalFlow::<usize>::blank();
        let parameters = table(&[("FLOW_ENABLE", 0.0)]);
        page.activate(
            &parameters,
            key(),
            true,
            COPTER,
            Firmware::ArduCopter2,
            bundled,
        );
        let now = Instant::now();
        let jobs = page.click_enable(&parameters, now);
        page.push(jobs);
        let link = Answering::new(&[("FLOW_ENABLE", Progress::Finished(RequestOutcome::TimedOut))]);
        for _ in 0..10 {
            page.tick(&link, &view, true, false, now);
        }
        assert_eq!(page.queue.pending(), 0);
        assert_eq!(
            link.taken().len(),
            2,
            "the handler's write, then the control's"
        );
        assert!(page.message().is_none(), "no box: {:?}", page.message());
        // The control's box came last; the handler's was on the line before it.
        assert_eq!(
            page.take_status().as_deref(),
            Some("Set FLOW_ENABLE Failed")
        );
        assert_eq!(page.take_status(), None, "taken once");

        // "Not Available on" is no failure of the link: it keeps its box.
        let jobs = page.click_enable(&[], now);
        page.push(jobs);
        let link = Answering::new(&[]);
        for _ in 0..10 {
            page.tick(&link, &view, true, false, now);
        }
        assert_eq!(page.message(), Some(&plain("Not Available on ArduCopter2")));
        assert_eq!(page.take_status(), None);
    }

    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-optflow.gui");
        let source = include_str!("optical_flow.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.optflow.") => {
                    let number = NUMBERS.iter().any(|row| {
                        let param = row.param.to_ascii_lowercase();
                        [
                            format!("config.optflow.{param}"),
                            format!("config.optflow.{param}.enabled"),
                            format!("config.optflow.{param}.visible"),
                        ]
                        .contains(&key.to_owned())
                    });
                    assert!(
                        number || source.contains(&format!("\"{key}\"")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("optflow-") => {
                    let base = id
                        .rsplit_once('-')
                        .filter(|(_, tail)| tail.chars().all(|c| c.is_ascii_digit()))
                        .map_or(id, |(base, _)| base);
                    assert!(source.contains(&format!("\"{base}")), "{id} is not drawn");
                }
                _ => {}
            }
        }
        assert!(facts > 8, "{facts} facts");
    }
}
