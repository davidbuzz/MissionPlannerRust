//! The ESC Calibration page of Initial Setup: `GCSViews/ConfigurationView/ConfigESCCalibration.cs`,
//! a Mandatory Hardware entry (`GCSViews/InitialSetup.cs:222-225`), listed once every parameter is
//! in.
//!
//! What it shows: its heading, "ESC Calibration (AC3.3+)"; the Calibrate ESCs button with the
//! instructions beside it, which begin "Remove Props!"; and under a rule, the ESC type
//! (`MOT_PWM_TYPE`) as a combo box and five numbers - `MOT_PWM_MIN`, `MOT_PWM_MAX`,
//! `MOT_SPIN_ARM`, `MOT_SPIN_MIN`, `MOT_SPIN_MAX` - each with its label and a note. `Activate`
//! sets the six controls up every time the page is shown (`ConfigESCCalibration.cs:15-26`); each
//! is a `Mavlink*` control, and writes its parameter itself when it changes, as on the Servo
//! Output page ([`crate::config::servo_output`]).
//!
//! The button is the page's one action: `setParam("ESC_CALIBRATION", 3)`, which makes the
//! autopilot calibrate the ESCs on its next power-up. The C# says "Set param error. Please ensure
//! your version is AC3.3+." when the set fails, and disables the button when it succeeds - for the
//! life of the page object, which is until the screen is left or loaded again
//! (`ConfigESCCalibration.cs:28-45`). There is no reboot prompt: the instructions tell the pilot to
//! disconnect and power up. Nothing is written until the button is pressed.
//!
//! The geometry is the `.resx`'s: every control at its `Location` and `Size` in a 595 x 349 page,
//! the three five-pixel group boxes drawn as the rules they look like. The colours are this
//! application's.
//!
//! What is not ported: the page has no `Deactivate`, and nothing it does is left out.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::time::Instant;

use gpui::{
    AnyElement, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px, rgb,
};
use mp_link::RequestId;
use mp_link::requests::RequestOutcome;

use crate::MissionPlanner;
use crate::config::failsafe::{Lookup, options};
use crate::config::servo_output::{
    Combo, ERROR_TITLE, Message, NUMERIC_DEFAULTS, Number, NumberHandlers, OUT_OF_RANGE_TITLE,
    Question, Setup, Write, Writes, combo_box, dropdown, label, modal, number_box, value_of,
};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{action, panel, theme};

/// The page's title in Initial Setup's list.
/// `// C#: GCSViews/InitialSetup.cs:224`
pub const TITLE: &str = "ESC Calibration";

/// `label1.Text`, the page's heading, in 12 point.
/// `// C#: GCSViews/ConfigurationView/ConfigESCCalibration.resx label1.Text, label1.Font`
pub const HEADING: &str = "ESC Calibration (AC3.3+)";

/// `label2.Text`, beside the button: the props warning and what to do after pressing it.
/// `// C#: GCSViews/ConfigurationView/ConfigESCCalibration.resx label2.Text`
pub const INSTRUCTIONS: &str = "Remove Props!\nAfter pushing this button:\n-Disconnect USB and \
                                battery\n-Plug in battery\n-when LEDs flash, push Saftey Switch \
                                (if present)\n-ESCs should beep as they are calibrated\n- restart \
                                flight controller normally";

/// `buttonStart.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigESCCalibration.resx buttonStart.Text`
pub const BUTTON: &str = "Calibrate ESCs";

/// What the button writes, and the value.
/// `// C#: GCSViews/ConfigurationView/ConfigESCCalibration.cs:32`
pub const CALIBRATION_PARAM: &str = "ESC_CALIBRATION";
/// `ESC_CALIBRATION` 3: calibrate on the next power-up.
pub const CALIBRATION_VALUE: f64 = 3.0;

/// What the button's handler says when the set fails, with no caption.
/// `// C#: GCSViews/ConfigurationView/ConfigESCCalibration.cs:34, 40`
pub const FAILED: &str = "Set param error. Please ensure your version is AC3.3+.";

/// `label3.Text`, left of the combo box.
/// `// C#: GCSViews/ConfigurationView/ConfigESCCalibration.resx label3.Text`
const ESC_TYPE: &str = "ESC Type:";

/// The ESC type's parameter.
/// `// C#: GCSViews/ConfigurationView/ConfigESCCalibration.cs:17-18`
pub const TYPE_PARAM: &str = "MOT_PWM_TYPE";

/// One of the five numbers: its parameter, its `setup` arguments, its label and note, and the
/// row its box sits on.
#[derive(Debug, Clone, Copy)]
pub struct NumberRow {
    /// The parameter.
    pub param: &'static str,
    /// `setup`'s `Min`, `Max`, `Scale`, `Increment`.
    pub setup: Setup,
    /// The label left of the box: `label4` to `label8`.
    pub label: &'static str,
    /// The note right of it: `label9` to `label13`.
    pub note: &'static str,
    /// The box's `Location.Y`; the labels sit seven pixels lower.
    pub y: f32,
}

/// `mavlinkNumericUpDown1` to `5`, with the labels beside them.
/// `// C#: GCSViews/ConfigurationView/ConfigESCCalibration.cs:20-25;
/// ConfigESCCalibration.resx mavlinkNumericUpDown1-5.Location, label4-13.Text`
pub const NUMBERS: [NumberRow; 5] = [
    NumberRow {
        param: "MOT_PWM_MIN",
        setup: Setup {
            minimum: 0.0,
            maximum: 1500.0,
            scale: 1.0,
            increment: 1.0,
        },
        label: "Output PWM Min",
        note: "Leave as 0 to use RX input range",
        y: 186.0,
    },
    NumberRow {
        param: "MOT_PWM_MAX",
        setup: Setup {
            minimum: 0.0,
            maximum: 2200.0,
            scale: 1.0,
            increment: 1.0,
        },
        label: "Output PWM Max",
        note: "Leave as 0 to use RX input range",
        y: 212.0,
    },
    NumberRow {
        param: "MOT_SPIN_ARM",
        setup: Setup {
            minimum: 0.0,
            maximum: 1.0,
            scale: 1.0,
            increment: 0.01,
        },
        label: "Spin when Armed",
        note: "speed when motors are armed but throttle is at zero (idle)",
        y: 238.0,
    },
    NumberRow {
        param: "MOT_SPIN_MIN",
        setup: Setup {
            minimum: 0.0,
            maximum: 1.0,
            scale: 1.0,
            increment: 0.01,
        },
        label: "Spin minimum",
        note: "minimum speed of motors while in flight (slightly higher than \"Spin when Armed\")",
        y: 264.0,
    },
    NumberRow {
        param: "MOT_SPIN_MAX",
        setup: Setup {
            minimum: 0.0,
            maximum: 1.0,
            scale: 1.0,
            increment: 0.01,
        },
        label: "Spin Maximum",
        note: "maximum speed of motors while in flight (almost all escs have a deadzone at the top)",
        y: 290.0,
    },
];

/// The page object and what it keeps.
#[derive(Debug)]
pub struct EscCalibration {
    /// The screen the page object belongs to; a different one is a new object.
    made_for: Option<Key>,
    /// Whether the page is showing.
    active: bool,
    /// `mavlinkComboBox1`, `MOT_PWM_TYPE`.
    esc_type: Combo,
    /// `mavlinkNumericUpDown1` to `5`, in [`NUMBERS`]' order.
    numbers: [Number; 5],
    /// `buttonStart.Enabled`: false once a press has set the parameter.
    button_enabled: bool,
    /// The press's write, while the link carries it.
    calibrating: Option<RequestId>,
    /// How many times the button has sent `ESC_CALIBRATION`.
    sent: usize,
    /// How the press's write ended.
    calibration: Option<String>,
    /// Whether the ESC type's list is dropped down.
    dropdown: bool,
    /// The number the keyboard is typing into.
    editing: Option<usize>,
    /// A number's out-of-range question, waiting for its answer.
    question: Option<(usize, Question)>,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// The controls' writes on their way.
    writes: Writes,
}

impl Default for EscCalibration {
    /// `InitializeComponent`: the combo box and the numbers disabled (`.resx` `Enabled = False`),
    /// the numbers at `NumericUpDown`'s defaults, the button enabled.
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            esc_type: Combo::default(),
            numbers: std::array::from_fn(|_| Number::new(NUMERIC_DEFAULTS)),
            button_enabled: true,
            calibrating: None,
            sent: 0,
            calibration: None,
            dropdown: false,
            editing: None,
            question: None,
            messages: VecDeque::new(),
            writes: Writes::default(),
        }
    }
}

impl EscCalibration {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// The ESC type's combo box.
    #[must_use]
    pub const fn esc_type(&self) -> &Combo {
        &self.esc_type
    }

    /// The five numbers, in [`NUMBERS`]' order.
    #[must_use]
    pub const fn numbers(&self) -> &[Number; 5] {
        &self.numbers
    }

    /// Whether the button can be pressed: enabled, and not waiting on its write - the C#'s set
    /// holds the page until it returns.
    #[must_use]
    pub const fn button_enabled(&self) -> bool {
        self.button_enabled && self.calibrating.is_none()
    }

    /// How many times the button has sent `ESC_CALIBRATION`.
    #[must_use]
    pub const fn sent(&self) -> usize {
        self.sent
    }

    /// How the button's write ended.
    #[must_use]
    pub fn calibration(&self) -> Option<&str> {
        self.calibration.as_deref()
    }

    /// Whether the ESC type's list is down.
    #[must_use]
    pub const fn dropdown(&self) -> bool {
        self.dropdown
    }

    /// The number being typed into.
    #[must_use]
    pub const fn editing(&self) -> Option<usize> {
        self.editing
    }

    /// The question waiting, if one is.
    #[must_use]
    pub fn question(&self) -> Option<&Question> {
        self.question.as_ref().map(|(_, question)| question)
    }

    /// The message box showing, if one is.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// How the controls' last write ended.
    #[must_use]
    pub fn last_write(&self) -> Option<&str> {
        self.writes.last()
    }

    /// Shows the page: a new page object if this screen has none, then `Activate`, which sets
    /// the combo box and the five numbers up from the vehicle's parameters.
    /// `// C#: GCSViews/ConfigurationView/ConfigESCCalibration.cs:10-26`
    pub fn activate(
        &mut self,
        telemetry: &Telemetry,
        parameters: &[(String, f64)],
        key: Key,
        lookup: Lookup,
    ) {
        if self.made_for != Some(key) {
            for write in self.dispose() {
                self.issue(telemetry, write);
            }
            self.made_for = Some(key);
        }
        self.esc_type
            .setup(options(TYPE_PARAM, lookup), TYPE_PARAM, parameters);
        for (number, row) in self.numbers.iter_mut().zip(NUMBERS) {
            number.setup(row.setup, row.param, parameters, lookup);
        }
        self.active = true;
    }

    /// The page hidden - it has no `Deactivate` - which takes the focus from a number being
    /// typed into, so its text is read.
    pub fn hide(&mut self, now: Instant) {
        self.active = false;
        self.dropdown = false;
        self.leave(now);
    }

    /// The page object disposed with its screen, returning what the numbers' timers still held.
    pub fn dispose(&mut self) -> Vec<Write> {
        let pending = self.numbers.iter_mut().filter_map(Number::flush).collect();
        *self = Self {
            sent: self.sent,
            messages: std::mem::take(&mut self.messages),
            writes: std::mem::take(&mut self.writes),
            ..Self::default()
        };
        pending
    }

    /// `buttonStart_Click`: `ESC_CALIBRATION` set to 3 through the retrying set. With no vehicle
    /// to send to, the set has failed already.
    /// `// C#: GCSViews/ConfigurationView/ConfigESCCalibration.cs:28-45`
    pub fn press(&mut self, telemetry: &Telemetry, now: Instant) {
        self.leave(now);
        self.dropdown = false;
        if !self.button_enabled() {
            return;
        }
        match telemetry.set_parameter_confirmed(CALIBRATION_PARAM, CALIBRATION_VALUE) {
            Some(id) => {
                self.calibrating = Some(id);
                self.sent += 1;
            }
            None => self.calibration_failed("no vehicle"),
        }
    }

    /// The button's set failed: its message box, with no caption.
    fn calibration_failed(&mut self, why: &str) {
        self.calibration = Some(format!(
            "{CALIBRATION_PARAM} {CALIBRATION_VALUE} failed: {why}"
        ));
        self.messages.push_back(Message {
            title: "",
            text: FAILED.to_owned(),
        });
    }

    /// Reads back how the button's write ended: `setParam`'s true disables the button, its false
    /// is the message box.
    fn settle_calibration(&mut self, telemetry: &Telemetry) {
        let Some(id) = self.calibrating else {
            return;
        };
        let Some(outcome) = telemetry.request(id).and_then(|request| request.outcome()) else {
            if telemetry.request(id).is_none() {
                self.calibrating = None;
            }
            return;
        };
        self.calibrating = None;
        match outcome {
            RequestOutcome::Accepted { value } => {
                let echoed = value.map_or(CALIBRATION_VALUE, |value| value.as_f64());
                self.calibration = Some(format!("{CALIBRATION_PARAM} {echoed} accepted"));
                self.button_enabled = false;
            }
            RequestOutcome::Unchanged | RequestOutcome::Sent => {
                self.calibration =
                    Some(format!("{CALIBRATION_PARAM} {CALIBRATION_VALUE} unchanged"));
                self.button_enabled = false;
            }
            RequestOutcome::UnknownParameter => self.calibration_failed("not on the vehicle"),
            RequestOutcome::TimedOut => self.calibration_failed("timed out"),
            RequestOutcome::Rejected(result) => {
                self.calibration_failed(&format!("rejected {result}"));
            }
        }
    }

    /// Drops the ESC type's list down, or back up.
    pub fn toggle_dropdown(&mut self, now: Instant) {
        self.leave(now);
        self.dropdown = !self.dropdown && self.esc_type.enabled;
        if self.dropdown {
            self.esc_type.open_list();
        }
    }

    /// The wheel over the dropped-down list.
    pub fn scroll_list(&mut self, lines: i32) {
        if self.dropdown {
            self.esc_type.scroll_list(lines);
        }
    }

    /// Chooses an ESC type from the list, closing it.
    pub fn choose(&mut self, key: i64) -> Option<Write> {
        self.dropdown = false;
        self.esc_type.choose(key)
    }

    /// A number clicked into.
    pub fn begin(&mut self, index: usize, now: Instant) {
        if self.editing == Some(index) {
            return;
        }
        self.leave(now);
        self.dropdown = false;
        if self.numbers.get(index).is_some_and(|number| number.enabled) {
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

    /// A number's up or down arrow.
    pub fn step(&mut self, index: usize, up: bool, now: Instant) {
        self.begin(index, now);
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

    /// Dismisses the message box showing.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// Sends a control's write.
    pub fn issue(&mut self, telemetry: &Telemetry, write: Write) {
        self.writes.issue(telemetry, write, &mut self.messages);
    }

    /// Once a frame: a page object whose screen has gone is disposed, a number that lost the
    /// focus has its text read, the timers that ran out write, and the writes are read back.
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
            for write in self.dispose() {
                self.issue(telemetry, write);
            }
        }
        if self.editing.is_some() && !focused {
            self.leave(now);
        }
        let due: Vec<Write> = self
            .numbers
            .iter_mut()
            .filter_map(|number| number.due(now))
            .collect();
        for write in due {
            self.issue(telemetry, write);
        }
        self.writes.settle(telemetry, &mut self.messages);
        self.settle_calibration(telemetry);
    }
}

/// Facts a UI test asserts on: the page's text, the button, the six controls, what was sent and
/// what the vehicle holds.
pub fn record_facts(esc: &EscCalibration, view: &TelemetryView) {
    use crate::facts::record;
    record("config.esc.active", esc.is_active());
    record("config.esc.heading", HEADING);
    record("config.esc.text", INSTRUCTIONS);
    record("config.esc.button", BUTTON);
    record("config.esc.button.enabled", esc.button_enabled());
    record("config.esc.sent", esc.sent());
    record(
        "config.esc.calibration",
        esc.calibration().unwrap_or("none"),
    );
    record(
        "config.esc.message",
        esc.message()
            .map_or("none", |message| message.text.as_str()),
    );
    record("config.esc.write", esc.last_write().unwrap_or("none"));
    record(
        "config.esc.type",
        esc.esc_type()
            .selected
            .map_or_else(|| "none".to_owned(), |key| key.to_string()),
    );
    record("config.esc.type.text", esc.esc_type().text());
    record("config.esc.type.enabled", esc.esc_type().enabled);
    for (number, row) in esc.numbers().iter().zip(NUMBERS) {
        let key = row.param.to_ascii_lowercase();
        record(format!("config.esc.{key}"), number.shown());
        record(format!("config.esc.{key}.enabled"), number.enabled);
    }
    for name in std::iter::once(CALIBRATION_PARAM)
        .chain(std::iter::once(TYPE_PARAM))
        .chain(NUMBERS.iter().map(|row| row.param))
    {
        if let Some(value) = value_of(&view.parameters, name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

/// A rule: one of the page's five-pixel-high group boxes, which draw as a line.
fn rule(x: f32, y: f32, width: f32) -> AnyElement {
    div()
        .absolute()
        .left(px(x))
        .top(px(y + 2.0))
        .w(px(width))
        .h(px(1.0))
        .bg(rgb(theme::BORDER))
        .into_any_element()
}

/// The page, laid out as `ConfigESCCalibration.resx` lays it out.
pub fn page(
    esc: &EscCalibration,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if !esc.is_active() {
        return None;
    }
    let mut body = div()
        .relative()
        .w(px(595.0))
        .h(px(349.0))
        .child(
            div()
                .absolute()
                .left(px(7.0))
                .top(px(5.0))
                .text_base()
                .text_color(rgb(theme::TEXT))
                .child(HEADING),
        )
        .child(rule(3.0, 23.0, 589.0))
        .child(rule(0.0, 147.0, 589.0))
        .child(rule(3.0, 316.0, 589.0));

    // buttonStart, at 11, 42, 93 x 25.
    let enabled = esc.button_enabled();
    let button = crate::probe::measured("esc-calibrate", div())
        .id("esc-calibrate")
        .size_full()
        .flex()
        .items_center()
        .justify_center()
        .rounded_sm()
        .border_1()
        .text_xs()
        .child(BUTTON);
    let button = if enabled {
        button
            .bg(rgb(theme::ACTION))
            .border_color(rgb(theme::WARN))
            .text_color(rgb(theme::WARN))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(cx.listener(|this, _event, _window, cx| {
                this.esc_calibration.press(&this.telemetry, Instant::now());
                cx.notify();
            }))
    } else {
        button
            .bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::DIM))
    };
    body = body.child(
        div()
            .absolute()
            .left(px(11.0))
            .top(px(42.0))
            .w(px(93.0))
            .h(px(25.0))
            .child(button),
    );
    // label2, at 110, 42: the instructions, a line to a line.
    body = body.child(
        crate::probe::measured("esc-instructions", div())
            .absolute()
            .left(px(110.0))
            .top(px(42.0))
            .flex()
            .flex_col()
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .children(
                INSTRUCTIONS
                    .lines()
                    .map(|line| div().child(SharedString::from(line))),
            ),
    );

    // label3 and the combo box.
    body = body.child(label(11.0, 165.0, ESC_TYPE)).child(combo_box(
        "esc-MOT_PWM_TYPE".to_owned(),
        esc.esc_type(),
        (113.0, 158.0, 120.0, 21.0),
        |this| this.esc_calibration.toggle_dropdown(Instant::now()),
        cx,
    ));

    for (index, (number, row)) in esc.numbers().iter().zip(NUMBERS).enumerate() {
        body = body
            .child(label(11.0, row.y + 7.0, row.label))
            .child(label(181.0, row.y + 7.0, row.note))
            .child(number_box(
                format!("esc-{}", row.param),
                number,
                esc.editing() == Some(index),
                handle,
                (113.0, row.y, 62.0, 20.0),
                NumberHandlers {
                    begin: move |this: &mut MissionPlanner| {
                        this.esc_calibration.begin(index, Instant::now());
                    },
                    key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
                        this.esc_calibration.key(event, Instant::now())
                    },
                    step: move |this: &mut MissionPlanner, up: bool| {
                        this.esc_calibration.step(index, up, Instant::now());
                    },
                },
                window,
                cx,
            ));
    }

    // The list last, over the numbers under it.
    if esc.dropdown() {
        body = body.child(dropdown(
            "esc-MOT_PWM_TYPE",
            esc.esc_type(),
            (113.0, 179.0, 120.0),
            |this, key| {
                if let Some(write) = this.esc_calibration.choose(key) {
                    this.esc_calibration.issue(&this.telemetry, write);
                }
            },
            |this, lines| this.esc_calibration.scroll_list(lines),
            cx,
        ));
    }
    Some(panel(TITLE, body).into_any_element())
}

/// The question or message box showing, drawn over the whole window.
pub fn overlay(
    esc: &EscCalibration,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(question) = esc.question() {
        let buttons = vec![
            action(
                "esc-question-yes",
                "Yes",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.esc_calibration.answer(true, Instant::now());
                    cx.notify();
                }),
            ),
            action(
                "esc-question-no",
                "No",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.esc_calibration.answer(false, Instant::now());
                    cx.notify();
                }),
            ),
        ];
        return Some(modal(
            "esc-question",
            OUT_OF_RANGE_TITLE,
            &question.text(),
            false,
            buttons,
            window,
        ));
    }
    let message = esc.message()?;
    let ok = action(
        "esc-message-ok",
        "OK",
        theme::ACCENT,
        true,
        cx.listener(|this, _event: &(), _window, cx| {
            this.esc_calibration.dismiss_message();
            cx.notify();
        }),
    );
    Some(modal(
        "esc-message",
        message.title,
        &message.text,
        message.title == ERROR_TITLE || message.text == FAILED,
        vec![ok],
        window,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::scripted::{Vehicle, param, until};
    use mp_link::ProtocolTimeouts;
    use mp_mavlink_dialects::all::MavMessage;
    use mp_params::ParamMeta;

    fn bundled(name: &str) -> Option<&'static ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    fn sitl() -> Vec<(String, f64)> {
        let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/params/sitl-copter.param");
        let file = mp_params::param_file::ParamFile::load(&fixture).expect("the SITL dump");
        file.iter()
            .map(|(name, value)| (name.to_owned(), value))
            .collect()
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn fast() -> ProtocolTimeouts {
        ProtocolTimeouts::default().faster(20)
    }

    /// The `.resx`'s words, read from the tree when it is here.
    #[test]
    fn the_text_is_the_resx_text() {
        let Some(resx) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigESCCalibration.resx",
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        assert_eq!(values.get("label1.Text").map(String::as_str), Some(HEADING));
        assert_eq!(
            values
                .get("label2.Text")
                .map(|text| text.replace("\r\n", "\n")),
            Some(INSTRUCTIONS.to_owned())
        );
        assert_eq!(
            values.get("buttonStart.Text").map(String::as_str),
            Some(BUTTON)
        );
        assert_eq!(
            values.get("label3.Text").map(String::as_str),
            Some(ESC_TYPE)
        );
        for (index, row) in NUMBERS.iter().enumerate() {
            assert_eq!(
                values
                    .get(&format!("label{}.Text", index + 4))
                    .map(String::as_str),
                Some(row.label)
            );
            assert_eq!(
                values
                    .get(&format!("label{}.Text", index + 9))
                    .map(String::as_str),
                Some(row.note)
            );
            assert_eq!(
                values
                    .get(&format!("mavlinkNumericUpDown{}.Location", index + 1))
                    .map(String::as_str),
                Some(format!("113, {}", row.y).as_str())
            );
        }
        assert!(INSTRUCTIONS.starts_with("Remove Props!"));
    }

    #[test]
    fn activate_reads_the_sitl_copters_motor_parameters() {
        let telemetry = Telemetry::idle();
        let mut esc = EscCalibration::default();
        assert!(!esc.esc_type().enabled, "disabled until Activate");
        esc.activate(&telemetry, &sitl(), key(), bundled);
        assert!(esc.is_active());
        assert!(esc.esc_type().enabled);
        assert_eq!(esc.esc_type().selected, Some(0));
        assert_eq!(esc.esc_type().text(), "Normal");
        let shown: Vec<&str> = esc.numbers().iter().map(Number::shown).collect();
        // The spins have two places, from setup's 0.01 increment.
        assert_eq!(shown, ["1000", "2000", "0.10", "0.15", "0.95"]);
        assert!(esc.numbers().iter().all(|number| number.enabled));
        assert!(esc.button_enabled());
        assert_eq!(esc.sent(), 0);
    }

    #[test]
    fn a_vehicle_without_the_parameters_leaves_the_controls_disabled() {
        let telemetry = Telemetry::idle();
        let mut esc = EscCalibration::default();
        esc.activate(&telemetry, &[], key(), bundled);
        assert!(!esc.esc_type().enabled);
        assert!(esc.numbers().iter().all(|number| !number.enabled));
        assert_eq!(esc.numbers()[2].shown(), "0.00");
        assert!(esc.choose(1).is_none());
    }

    /// Showing the page, using its controls and leaving it sends nothing to `ESC_CALIBRATION`:
    /// only the button does.
    #[test]
    fn nothing_is_sent_to_esc_calibration_unless_the_button_is_pressed() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&param("ESC_CALIBRATION", 0.0, 2));
        until("the parameter to be held", || {
            telemetry.holds_parameter("ESC_CALIBRATION")
        });
        let view = telemetry.view();
        let mut esc = EscCalibration::default();
        esc.activate(&telemetry, &view.parameters, Key::of(&view), bundled);
        for _ in 0..20 {
            esc.tick(&telemetry, &view, true, false, Instant::now());
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        esc.hide(Instant::now());
        esc.tick(&telemetry, &view, false, false, Instant::now());
        vehicle.read();
        assert_eq!(
            vehicle.count(|message| matches!(message, MavMessage::ParamSet(_))),
            0
        );
        assert_eq!(esc.sent(), 0);
    }

    /// The button sets `ESC_CALIBRATION` to 3 and, once the vehicle echoes it, is disabled.
    #[test]
    fn the_button_sets_esc_calibration_to_three_and_is_then_disabled() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&param("ESC_CALIBRATION", 0.0, 2));
        until("the parameter to be held", || {
            telemetry.holds_parameter("ESC_CALIBRATION")
        });
        let view = telemetry.view();
        let mut esc = EscCalibration::default();
        esc.activate(&telemetry, &view.parameters, Key::of(&view), bundled);
        esc.press(&telemetry, Instant::now());
        assert_eq!(esc.sent(), 1);
        assert!(!esc.button_enabled(), "held while the set is under way");
        let mut written = None;
        until("the PARAM_SET", || {
            for message in vehicle.read() {
                if let MavMessage::ParamSet(set) = message {
                    written = Some((mp_params::decode_param_id(&set.param_id), set.param_value));
                    vehicle.send(&param("ESC_CALIBRATION", set.param_value, 2));
                }
            }
            written.is_some()
        });
        assert_eq!(written, Some(("ESC_CALIBRATION".to_owned(), 3.0)));
        until("the echo", || {
            esc.tick(&telemetry, &telemetry.view(), true, false, Instant::now());
            esc.calibration().is_some()
        });
        assert_eq!(esc.calibration(), Some("ESC_CALIBRATION 3 accepted"));
        assert!(!esc.button_enabled());
        assert!(esc.message().is_none());
        // Pressed again, it does nothing.
        esc.press(&telemetry, Instant::now());
        assert_eq!(esc.sent(), 1);
    }

    /// A vehicle without the parameter: the set fails, the C#'s message, and the button stays.
    #[test]
    fn a_failed_set_says_so_and_leaves_the_button_enabled() {
        let telemetry = Telemetry::idle();
        let mut esc = EscCalibration::default();
        esc.activate(&telemetry, &sitl(), key(), bundled);
        esc.press(&telemetry, Instant::now());
        let message = esc.message().expect("a message box");
        assert_eq!(message.text, FAILED);
        assert_eq!(message.title, "");
        assert!(esc.button_enabled());
    }

    #[test]
    fn the_button_stays_disabled_for_the_life_of_the_page_object() {
        let telemetry = Telemetry::idle();
        let view = telemetry.view();
        let mut esc = EscCalibration::default();
        esc.activate(&telemetry, &sitl(), Key::of(&view), bundled);
        esc.button_enabled = false;
        esc.hide(Instant::now());
        esc.tick(&telemetry, &view, true, false, Instant::now());
        esc.activate(&telemetry, &sitl(), Key::of(&view), bundled);
        assert!(!esc.button_enabled(), "Activate does not enable it again");
        esc.hide(Instant::now());
        esc.tick(&telemetry, &view, false, false, Instant::now());
        esc.activate(&telemetry, &sitl(), Key::of(&view), bundled);
        assert!(esc.button_enabled(), "a new page object");
    }

    #[test]
    fn a_spin_steps_by_a_hundredth_and_is_written_as_the_float() {
        let telemetry = Telemetry::idle();
        let mut esc = EscCalibration::default();
        esc.activate(&telemetry, &sitl(), key(), bundled);
        let now = Instant::now();
        esc.step(2, true, now);
        assert_eq!(esc.numbers()[2].shown(), "0.11");
        let write = esc.numbers[2]
            .due(now + crate::config::servo_output::WRITE_DELAY)
            .expect("due");
        assert_eq!(write.param, "MOT_SPIN_ARM");
        assert!((write.value - f64::from(0.11_f32)).abs() < f64::EPSILON);
    }

    #[test]
    fn choosing_an_esc_type_writes_it() {
        let telemetry = Telemetry::idle();
        let mut esc = EscCalibration::default();
        esc.activate(&telemetry, &sitl(), key(), bundled);
        esc.toggle_dropdown(Instant::now());
        assert!(esc.dropdown());
        let write = esc.choose(4).expect("a change");
        assert!(!esc.dropdown());
        assert_eq!(write.param, "MOT_PWM_TYPE");
        assert!((write.value - 4.0).abs() < f64::EPSILON);
        assert_eq!(esc.esc_type().text(), "DShot150");
    }

    /// Every fact the GUI script asserts on is one this page records, and every control it
    /// clicks is one this page draws; and the script never presses the button.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-esc.gui");
        let source = include_str!("esc_calibration.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.esc.") => {
                    let recorded = source.contains(&format!("\"{key}\""))
                        || NUMBERS.iter().any(|row| {
                            let param = row.param.to_ascii_lowercase();
                            key == format!("config.esc.{param}")
                                || key == format!("config.esc.{param}.enabled")
                        });
                    assert!(recorded, "{key} is not recorded");
                    facts += 1;
                }
                (Some("click"), Some(id)) => {
                    assert_ne!(id, "esc-calibrate", "the script must not press the button");
                }
                _ => {}
            }
        }
        assert!(facts > 10, "{facts} facts");
    }
}
