//! Motor Test: `GCSViews/ConfigurationView/ConfigMotorTest.cs`, a page of Initial Setup's Optional
//! Hardware list (`GCSViews/InitialSetup.cs:315-318`), listed once every parameter is in.
//!
//! What it shows: one group box, "Motor Test", with a throttle percentage and a duration, the
//! frame's class and type, a button per motor - "Test motor A", "Test motor B"... in test order,
//! each labelled with the motor number and rotation the frame's layout gives that place - then
//! "Test all motors", "Stop all motors" and "Test all in Sequence", and on the right the Set Motor
//! Spin Arm and Spin Min buttons, a note to hold the vehicle down and a link to ArduPilot's motor
//! order diagrams. `Activate` builds the motor buttons from the frame each time the page is shown
//! (`ConfigMotorTest.cs:43-109`); how many, and their labels, are `mp_calibration::motor`.
//!
//! Every test button is `testMotor`: `MAV_CMD_DO_MOTOR_TEST` with the motor, the throttle as a
//! percentage, the throttle, the duration and a motor count, sent through the link's retrying
//! `doCommand` (`Telemetry::test_motor`). The C# waits on each in turn on the UI thread; here the
//! commands of one press are handed to the link together and it matches each `COMMAND_ACK` to the
//! oldest command still waiting for one, which is the order they went out in. A refusal or a
//! timeout is said on the status line, where this application says what the C# puts in a message
//! box.
//!
//! The Spin Arm and Spin Min buttons disable the page, ask for a percentage in an `InputBox` and
//! write `MOT_SPIN_ARM` or `MOT_SPIN_MIN` as a fraction, then enable it again
//! (`ConfigMotorTest.cs:341-396`). Their message boxes and the question are drawn over the page.
//!
//! The geometry is `ConfigMotorTest.resx`'s, each control at its `Location` in `groupBox1`, the
//! run-time buttons where `Activate` puts them. The colours are this application's.
//!
//! What is not ported, and why:
//!
//! * a picture of the motor order: the C# has none on this page - the link opens ArduPilot's
//!   diagrams in the browser, and the labels beside the buttons come from `APMotorLayout.json`,
//!   which is data and is here (`mp_calibration::motor_layouts`);
//! * `Activate` adding a second set of buttons each time the page is shown again - it never
//!   removes the first set, which stays on top of the new one. Here the buttons are rebuilt, so a
//!   frame changed between visits shows its own buttons rather than the first visit's;
//! * the page's `Enabled` persisting for the life of the page object: the C# never sets it back
//!   to true once `get_motormax` or a Spin handler that returned early set it false, until
//!   `MainV2` reloads the screen. Here each `Activate` sets it afresh;
//! * `linkLabel1`'s "Bad default system association" box, for a browser that cannot be started:
//!   gpui's `open_url` does not say whether one was;
//! * the Yes of `Program.handleException`'s "Report this Error???" box, which a Spin handler's
//!   uncaught exception reaches: this application sends no error reports, so the box has one
//!   button.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::time::Instant;

use gpui::{
    AnyElement, Context, Div, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px,
    rgb,
};
use mp_calibration::motor::{self, CMD_DO_MOTOR_TEST, MotorCommand};
use mp_calibration::motor_layouts::Layout;
use mp_link::RequestId;
use mp_link::requests::RequestOutcome;
use mp_mavlink_dialects::all::MavCmd;

use crate::MissionPlanner;
use crate::config::failsafe::{Lookup, options};
use crate::telemetry::{self, Telemetry, TelemetryView};
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::theme;

/// `groupBox1.Text`, the page's one group box.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.resx groupBox1.Text`
pub const TITLE: &str = "Motor Test";
/// `groupBox1.Size`, which `AutoSize` grows to fit the buttons `Activate` adds.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.resx groupBox1.Size`
const GROUP_SIZE: (f32, f32) = (528.0, 316.0);

/// `label1.Text`, left of the throttle box.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.resx label1.Text`
const THROTTLE_LABEL: &str = "Throttle %";
/// `label3.Text`, left of the duration box.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.resx label3.Text`
const DURATION_LABEL: &str = "Duration (s)";
/// `label2.Text`, the note beside the buttons.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.resx label2.Text`
pub const NOTE: &str = "NOTE: PLEASE HOLD DOWN YOUR UAV\nThis will test your motors are working.\nMotors are tested in a clockwise rotation \nstarting at the front right.";
/// `linkLabel1.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.resx linkLabel1.Text`
const LINK_TEXT: &str =
    "Please click here to see your motor numbers,\nscroll to the bottom of the page";
/// What `linkLabel1_LinkClicked` opens.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:333`
pub const MOTOR_ORDER_URL: &str =
    "https://ardupilot.org/copter/docs/connect-escs-and-motors.html#motor-order-diagrams";
/// `but_mot_spin_arm.Text` and `label4.Text` beside it.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.resx but_mot_spin_arm.Text, label4.Text`
const SPIN_ARM: (&str, &str) = (
    "Set Motor Spin Arm",
    "Set the min % that will be output when armed, but still on the ground",
);
/// `but_mot_spin_min.Text` and `label5.Text` beside it.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.resx but_mot_spin_min.Text, label5.Text`
const SPIN_MIN: (&str, &str) = (
    "Set Motor Spin Min",
    "Set the min % that will be output when flying",
);
/// `FrameClass.Text` and `FrameType.Text` until a frame is read.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.resx FrameClass.Text, FrameType.Text`
const UNKNOWN: (&str, &str) = ("Class: unknown", "Type: unknown");
/// The three buttons `Activate` adds under the motor buttons, in its order.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:84, 93, 102`
pub const TEST_ALL: &str = "Test all motors";
/// "Stop all motors".
pub const STOP_ALL: &str = "Stop all motors";
/// "Test all in Sequence".
pub const TEST_SEQUENCE: &str = "Test all in Sequence";

/// `Strings.ERROR`, the Spin handlers' message boxes' caption.
/// `// C#: ExtLibs/Strings/Strings.resx:130-132`
const ERROR_TITLE: &str = "Error";
/// `Strings.ChangeThrottle`, the `InputBox`'s caption.
/// `// C#: ExtLibs/Strings/Strings.resx:594-596`
const CHANGE_THROTTLE: &str = "Change Throttle";
/// What both Spin handlers say when the throttle box holds 20 or more.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:363, 392`
pub const TOO_HIGH: &str = "Throttle percent above 20, too high";
/// `Program.handleException`'s box, which an exception out of an `async void` handler reaches:
/// its caption, and the text before the exception.
/// `// C#: Program.cs:791-793`
const UNHANDLED: (&str, &str) = ("Send Error", "An error has occurred\n");

/// `NUM_thr_percent`: `Minimum` -100, `Maximum` the default 100, `Value` 5.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.Designer.cs:53-63`
const THROTTLE_BOX: (f64, f64, f64) = (5.0, -100.0, 100.0);
/// `NUM_duration`: `Minimum` the default 0, `Maximum` 999, `Value` 2.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.Designer.cs:85-95`
const DURATION_BOX: (f64, f64, f64) = (2.0, 0.0, 999.0);

/// Where `Activate` puts the first motor button, and how far down each next one goes.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:45-46, 80`
const BUTTONS_AT: (f32, f32, f32) = (6.0, 75.0, 25.0);
/// A `MyButton`'s default size, which the motor buttons keep: `Button.DefaultSize`.
const MOTOR_BUTTON: (f32, f32) = (75.0, 23.0);
/// The three buttons under them: 75 by 37, 39 apart.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:86, 90, 95, 99, 104`
const BIG_BUTTON: (f32, f32, f32) = (75.0, 37.0, 39.0);

/// A `NumericUpDown` with no decimal places and an increment of 1: its `Value`, its bounds, and
/// the text in its box, which is only read into `Value` when the box is validated - on leaving
/// it, on Enter, on an arrow, and whenever `Value` is read.
#[derive(Debug)]
pub struct NumericUpDown {
    /// The box's text.
    pub field: TextField,
    value: f64,
    minimum: f64,
    maximum: f64,
    /// `UserEdit`: typed into since it was last validated.
    edited: bool,
}

impl NumericUpDown {
    fn new((value, minimum, maximum): (f64, f64, f64)) -> Self {
        let mut field = TextField::new("");
        field.set(number_text(value));
        Self {
            field,
            value,
            minimum,
            maximum,
            edited: false,
        }
    }

    /// What `Value` reads now: the typed text if it parses, constrained to the bounds, else the
    /// value it held. `ParseEditText` swallows a text that does not parse.
    #[must_use]
    pub fn value(&self) -> f64 {
        if !self.edited {
            return self.value;
        }
        let text = self.field.value().trim();
        // `ParseEditText` leaves an empty box and a lone "-" alone.
        if text.is_empty() || text == "-" {
            return self.value;
        }
        text.parse::<f64>()
            .ok()
            .filter(|typed| typed.is_finite())
            .map_or(self.value, |typed| typed.clamp(self.minimum, self.maximum))
    }

    /// `ValidateEditText`: the text read into `Value`, and the box showing `Value` again.
    pub fn commit(&mut self) -> f64 {
        self.value = self.value();
        self.edited = false;
        self.field.set(number_text(self.value));
        self.value
    }

    /// `(int)NUM_thr_percent.Value`: validated, then truncated.
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(int)` of a value within -100..999
    pub fn int(&mut self) -> i32 {
        self.commit() as i32
    }

    /// `UpButton` and `DownButton`: validated, stepped by the increment, constrained.
    pub fn step(&mut self, delta: f64) {
        let value = self.commit();
        self.value = (value + delta).clamp(self.minimum, self.maximum);
        self.field.set(number_text(self.value));
    }

    /// A key in the box: the arrow keys step, Enter validates, and anything else is typing.
    pub fn key(&mut self, event: &KeyDownEvent) -> bool {
        match event.keystroke.key.as_str() {
            "up" => {
                self.step(1.0);
                return true;
            }
            "down" => {
                self.step(-1.0);
                return true;
            }
            _ => {}
        }
        match self.field.key(event) {
            KeyOutcome::Changed => {
                self.edited = true;
                true
            }
            KeyOutcome::Submitted => {
                self.commit();
                true
            }
            KeyOutcome::Cancelled | KeyOutcome::Ignored => false,
        }
    }

    /// Replaces the box's text as typing would.
    #[cfg(test)]
    fn type_text(&mut self, text: &str) {
        self.field.set(text);
        self.edited = true;
    }
}

/// `Value` as the box shows it at `DecimalPlaces` 0: `decimal.ToString("F0")`, which rounds a
/// half away from zero.
#[allow(clippy::cast_possible_truncation)] // a box's value, within -100..999
fn number_text(value: f64) -> String {
    format!("{}", value.round() as i64)
}

/// Which Spin button.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Spin {
    /// `but_mot_spin_arm`, writing `MOT_SPIN_ARM`.
    Arm,
    /// `but_mot_spin_min`, writing `MOT_SPIN_MIN`.
    Min,
}

impl Spin {
    /// The parameter it writes.
    #[must_use]
    pub const fn param(self) -> &'static str {
        match self {
            Self::Arm => "MOT_SPIN_ARM",
            Self::Min => "MOT_SPIN_MIN",
        }
    }

    /// The `InputBox`'s question.
    /// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:354, 382`
    #[must_use]
    pub const fn question(self) -> &'static str {
        match self {
            Self::Arm => "Enter arm throttle % (deadzone + 2%)",
            Self::Min => "Enter min spin throttle % (arm min + 3%)",
        }
    }
}

/// The `InputBox` a Spin handler is waiting on.
#[derive(Debug)]
pub struct Prompt {
    /// Which handler.
    pub spin: Spin,
    /// The answer box.
    pub field: TextField,
}

/// A message box: its caption and text.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Message {
    /// The caption.
    pub title: &'static str,
    /// The text.
    pub text: String,
}

/// One button press: what was pressed, what it sent, and how each went.
#[derive(Debug, Clone)]
struct Press {
    /// The button's text.
    button: String,
    /// Each `testMotor` call, its request, and its outcome once it has one.
    commands: Vec<(MotorCommand, Option<RequestId>, Option<String>)>,
    /// When it was pressed, for the link's pick-up grace.
    made: Instant,
    /// The newest message in the log when it was pressed, so the vehicle's answers are the lines
    /// after it.
    after: u64,
}

/// The page.
#[derive(Debug)]
pub struct MotorTest {
    /// Shown: between `Activate` and the list choosing another page.
    active: bool,
    /// The page's `Enabled`.
    enabled: bool,
    /// `motormax`.
    motormax: usize,
    /// `motor_layout`, kept between activations as the C#'s field is.
    layout: Option<&'static Layout>,
    /// `FrameClass.Text`, kept between activations: only a documented value changes it.
    class_text: String,
    /// `FrameType.Text`, likewise.
    type_text: String,
    /// `NUM_thr_percent`.
    pub throttle: NumericUpDown,
    /// `NUM_duration`.
    pub duration: NumericUpDown,
    /// The last press.
    press: Option<Press>,
    /// How many `testMotor` calls this page has made.
    sent: usize,
    /// The Spin handler's question, while it is asked.
    prompt: Option<Prompt>,
    /// The Spin handler's write, while it is awaited: which, the value, and its request.
    spin_write: Option<(Spin, f64, RequestId)>,
    /// How the last Spin write went, for a test.
    last_spin: Option<String>,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// Which number boxes had the focus last frame, to validate one the focus has left.
    focused: [bool; 2],
}

impl Default for MotorTest {
    fn default() -> Self {
        Self {
            active: false,
            enabled: true,
            motormax: 0,
            layout: None,
            class_text: UNKNOWN.0.to_owned(),
            type_text: UNKNOWN.1.to_owned(),
            throttle: NumericUpDown::new(THROTTLE_BOX),
            duration: NumericUpDown::new(DURATION_BOX),
            press: None,
            sent: 0,
            prompt: None,
            spin_write: None,
            last_spin: None,
            messages: VecDeque::new(),
            focused: [false; 2],
        }
    }
}

/// A parameter's value from the view's table.
fn value_of(view: &TelemetryView, name: &str) -> Option<f64> {
    view.parameters
        .iter()
        .find(|(held, _)| held == name)
        .map(|(_, value)| *value)
}

/// `Label.Text` for a documented value of `param`: `prefix` and the option's text, or `None` when
/// the documentation lists no such value and the label is left as it was.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:209-228`
fn option_text(param: &str, value: i32, prefix: &str, lookup: Lookup) -> Option<String> {
    options(param, lookup)
        .into_iter()
        .find(|(key, _)| *key == i64::from(value))
        .map(|(_, text)| format!("{prefix}{text}"))
}

impl MotorTest {
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

    /// `motormax`.
    #[must_use]
    pub const fn motor_count(&self) -> usize {
        self.motormax
    }

    /// `FrameClass.Text` and `FrameType.Text`.
    #[must_use]
    pub fn frame_text(&self) -> (&str, &str) {
        (&self.class_text, &self.type_text)
    }

    /// The motor buttons' texts, `A` first.
    #[must_use]
    pub fn buttons(&self) -> Vec<String> {
        (1..=self.motormax).map(motor::button_text).collect()
    }

    /// The labels beside motor button `a`.
    #[must_use]
    pub fn labels(&self, a: usize) -> Vec<String> {
        motor::labels(self.layout, a)
    }

    /// The message box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// The question being asked.
    #[must_use]
    pub const fn prompt(&self) -> Option<&Prompt> {
        self.prompt.as_ref()
    }

    /// The commands the last press sent.
    #[cfg(test)]
    #[must_use]
    pub fn last_commands(&self) -> Vec<MotorCommand> {
        self.press.as_ref().map_or_else(Vec::new, |press| {
            press
                .commands
                .iter()
                .map(|(command, ..)| *command)
                .collect()
        })
    }

    /// `Activate`: the motor count, the frame's labels and layout, and the buttons they make.
    /// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:43-109, 111-233`
    pub fn activate(&mut self, view: &TelemetryView, lookup: Lookup) {
        let aptype = view.state.as_ref().map_or(0, |state| state.vehicle_type);
        let count = motor::motor_count(|name| value_of(view, name), aptype, self.layout);
        if let Some(frame) = count.frame {
            if let Some(text) = option_text(frame.class_param, frame.class, "Class: ", lookup) {
                self.class_text = text;
            }
            if let Some(text) = option_text(frame.type_param, frame.frame_type, "Type: ", lookup) {
                self.type_text = text;
            }
        }
        self.motormax = count.count;
        self.layout = count.layout;
        self.enabled = count.enabled;
        self.active = true;
    }

    /// The list choosing another page. `ConfigMotorTest` has no `Deactivate`: nothing is stopped
    /// or written. The question and the message boxes are modal in the C#, so no other page can
    /// be chosen while they show; here they go with the page, the question as Cancel.
    pub fn deactivate(&mut self) {
        self.active = false;
        if self.prompt.take().is_some() {
            self.enabled = true;
        }
        self.messages.clear();
    }

    /// Sends one press's `testMotor` calls and keeps them for the facts.
    fn send(
        &mut self,
        button: String,
        commands: Vec<MotorCommand>,
        telemetry: &mut Telemetry,
        view: &TelemetryView,
    ) {
        let after = view.messages.last().map_or(0, |message| message.seq);
        let commands = commands
            .into_iter()
            .map(|command| (command, telemetry.test_motor(command), None))
            .collect::<Vec<_>>();
        self.sent += commands.len();
        self.press = Some(Press {
            button,
            commands,
            made: Instant::now(),
            after,
        });
    }

    /// A motor button, `but_Click`: `testMotor(Tag, speed, time)`.
    /// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:290-303`
    pub fn test_one(&mut self, a: usize, telemetry: &mut Telemetry, view: &TelemetryView) {
        if !self.enabled || a == 0 || a > self.motormax {
            return;
        }
        let speed = self.throttle.int();
        let time = self.duration.int();
        let motor = i32::try_from(a).unwrap_or(i32::MAX);
        self.send(
            motor::button_text(a),
            vec![motor::single(motor, speed, time)],
            telemetry,
            view,
        );
    }

    /// "Test all motors", `but_TestAll`.
    /// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:263-272`
    pub fn test_all(&mut self, telemetry: &mut Telemetry, view: &TelemetryView) {
        if !self.enabled {
            return;
        }
        let speed = self.throttle.int();
        let time = self.duration.int();
        let commands = motor::test_all(self.motormax, speed, time);
        self.send(TEST_ALL.to_owned(), commands, telemetry, view);
    }

    /// "Test all in Sequence", `but_TestAllSeq`.
    /// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:274-280`
    pub fn test_sequence(&mut self, telemetry: &mut Telemetry, view: &TelemetryView) {
        if !self.enabled {
            return;
        }
        let speed = self.throttle.int();
        let time = self.duration.int();
        let command = motor::sequence(self.motormax, speed, time);
        self.send(TEST_SEQUENCE.to_owned(), vec![command], telemetry, view);
    }

    /// "Stop all motors", `but_StopAll`. The boxes are not read.
    /// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:282-288`
    pub fn stop_all(&mut self, telemetry: &mut Telemetry, view: &TelemetryView) {
        if !self.enabled {
            return;
        }
        let commands = motor::stop_all(self.motormax);
        self.send(STOP_ALL.to_owned(), commands, telemetry, view);
    }

    /// `but_mot_spin_arm_Click` and `but_mot_spin_min_Click`, up to the `InputBox`: the page
    /// disabled; the parameter missing said and the page left disabled, as the C#'s early
    /// `return` leaves it; a throttle under 20 asks for the percentage - the throttle plus 2 for
    /// Spin Arm, the vehicle's `MOT_SPIN_MIN` cast to `int` plus 3 for Spin Min (a fraction, so
    /// 3); and anything else said, and the page enabled again.
    /// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:341-396`
    pub fn click_spin(&mut self, spin: Spin, view: &TelemetryView) {
        if !self.enabled {
            return;
        }
        self.enabled = false;
        let Some(current) = value_of(view, spin.param()) else {
            self.messages.push_back(Message {
                title: ERROR_TITLE,
                text: format!("param {} missing", spin.param()),
            });
            return;
        };
        if self.throttle.commit() < 20.0 {
            #[allow(clippy::cast_possible_truncation)] // the C#'s `(int)` casts
            let value = match spin {
                Spin::Arm => self.throttle.value() as i32 + 2,
                Spin::Min => current as i32 + 3,
            };
            let mut field = TextField::new("");
            field.set(value.to_string());
            self.prompt = Some(Prompt { spin, field });
        } else {
            self.messages.push_back(Message {
                title: ERROR_TITLE,
                text: TOO_HIGH.to_owned(),
            });
            self.enabled = true;
        }
    }

    /// OK on the question: `int.Parse` of the answer, and `setParamAsync` of it over 100 as a
    /// `float`. Text that is not a whole number is `int.Parse`'s `FormatException`, which escapes
    /// the `async void` handler to `Program.handleException`'s box and leaves the page disabled.
    /// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:354-358, 382-387; ExtLibs/Controls/InputBox.cs:21-27; Program.cs:791-793`
    pub fn answer(&mut self, telemetry: &Telemetry) {
        let Some(prompt) = self.prompt.take() else {
            return;
        };
        let Some(percent) = int_parse(prompt.field.value()) else {
            self.messages.push_back(Message {
                title: UNHANDLED.0,
                text: format!(
                    "{}System.FormatException: Input string was not in a correct format.",
                    UNHANDLED.1
                ),
            });
            return;
        };
        #[allow(clippy::cast_precision_loss)] // `(float)value`, a percentage
        let value = f64::from(percent as f32 / 100.0_f32);
        match telemetry.set_parameter_confirmed(prompt.spin.param(), value) {
            Some(id) => self.spin_write = Some((prompt.spin, value, id)),
            // No link: `setParamAsync` has nothing to write to and returns false, unread.
            None => self.enabled = true,
        }
    }

    /// Cancel on the question: nothing written, and the page enabled.
    /// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:354, 366`
    pub fn cancel(&mut self) {
        if self.prompt.take().is_some() {
            self.enabled = true;
        }
    }

    /// A key in the question's box.
    pub fn prompt_key(&mut self, event: &KeyDownEvent, telemetry: &Telemetry) -> bool {
        let Some(prompt) = self.prompt.as_mut() else {
            return false;
        };
        match prompt.field.key(event) {
            KeyOutcome::Submitted => self.answer(telemetry),
            KeyOutcome::Cancelled => self.cancel(),
            KeyOutcome::Changed => {}
            KeyOutcome::Ignored => return false,
        }
        true
    }

    /// Dismisses the message box showing.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// Once a frame: validates a number box the focus has left, reads how the last press's
    /// commands went, and ends a Spin handler once its write has.
    pub fn tick(&mut self, telemetry: &Telemetry, focused: [bool; 2]) {
        if self.focused[0] && !focused[0] {
            self.throttle.commit();
        }
        if self.focused[1] && !focused[1] {
            self.duration.commit();
        }
        self.focused = focused;

        if let Some(press) = self.press.as_mut() {
            for (_, request, outcome) in &mut press.commands {
                if outcome.is_some() {
                    continue;
                }
                let Some(id) = *request else {
                    *outcome = Some("not sent".to_owned());
                    continue;
                };
                *outcome = match telemetry.lookup(id, press.made) {
                    telemetry::Lookup::Found(request) => request.outcome().map(outcome_text),
                    telemetry::Lookup::PickingUp => None,
                    telemetry::Lookup::Gone => Some("lost".to_owned()),
                };
            }
        }

        if let Some((spin, value, id)) = self.spin_write {
            let Some(request) = telemetry.request(id) else {
                self.spin_write = None;
                self.last_spin = Some(format!("{} {value} lost", spin.param()));
                self.enabled = true;
                return;
            };
            let Some(outcome) = request.outcome() else {
                return;
            };
            self.spin_write = None;
            match outcome {
                // `setParamAsync`'s `TimeoutException`, out of the `async void` handler: the
                // page stays disabled (MAVLinkInterface.cs:1765; Program.cs:791-793).
                RequestOutcome::TimedOut => {
                    self.last_spin = Some(format!("{} {value} timed out", spin.param()));
                    self.messages.push_back(Message {
                        title: UNHANDLED.0,
                        text: format!(
                            "{}System.TimeoutException: Timeout on read - setParam {}",
                            UNHANDLED.1,
                            spin.param()
                        ),
                    });
                }
                // Its `bool` is not read: whatever it returned, the page is enabled again.
                RequestOutcome::Accepted { value: echoed } => {
                    self.last_spin = Some(format!(
                        "{} {} accepted",
                        spin.param(),
                        echoed.map_or(value, mp_params::ParamValue::as_f64)
                    ));
                    self.enabled = true;
                }
                other => {
                    self.last_spin =
                        Some(format!("{} {value} {}", spin.param(), outcome_text(other)));
                    self.enabled = true;
                }
            }
        }
    }
}

/// `int.Parse`: optional white space, an optional sign, and digits.
fn int_parse(text: &str) -> Option<i32> {
    let text = text.trim();
    let digits = text.strip_prefix(['+', '-']).unwrap_or(text);
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    text.parse().ok()
}

/// An outcome, in a word or two.
fn outcome_text(outcome: RequestOutcome) -> String {
    match outcome {
        RequestOutcome::Accepted { .. } => "accepted".to_owned(),
        RequestOutcome::Rejected(result) => {
            format!(
                "rejected {}",
                mp_link::messages::command_result_name(result)
            )
        }
        RequestOutcome::TimedOut => "timed out".to_owned(),
        RequestOutcome::Sent => "sent".to_owned(),
        RequestOutcome::UnknownParameter => "unknown parameter".to_owned(),
        RequestOutcome::Unchanged => "unchanged".to_owned(),
    }
}

/// The focus handles of the two number boxes and the question's box.
pub struct Focus {
    throttle: FocusHandle,
    duration: FocusHandle,
    prompt: FocusHandle,
}

impl Focus {
    /// New handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            throttle: cx.focus_handle(),
            duration: cx.focus_handle(),
            prompt: cx.focus_handle(),
        }
    }

    /// Whether the throttle and the duration box have the focus.
    #[must_use]
    pub fn focused(&self, window: &Window) -> [bool; 2] {
        [
            self.throttle.is_focused(window),
            self.duration.is_focused(window),
        ]
    }
}

/// The prefix of the message log's line for a `MAV_CMD_DO_MOTOR_TEST` acknowledgement.
fn ack_prefix() -> String {
    let name = MavCmd(u32::from(CMD_DO_MOTOR_TEST))
        .name()
        .map_or_else(|| format!("command {CMD_DO_MOTOR_TEST}"), ToOwned::to_owned);
    format!("{name}:")
}

/// Facts a UI test asserts on: what the page built, what the last press sent, what the link and
/// the vehicle said back, and the vehicle's motor outputs.
pub fn record_facts(test: &MotorTest, view: &TelemetryView) {
    use crate::facts::record;
    let joined = |values: Vec<String>, separator: &str| {
        if values.is_empty() {
            "none".to_owned()
        } else {
            values.join(separator)
        }
    };
    record("config.motortest.active", test.active);
    record("config.motortest.enabled", test.enabled());
    record("config.motortest.count", test.motor_count());
    record(
        "config.motortest.letters",
        joined(
            (1..=test.motormax)
                .map(|a| motor::letter(a).to_string())
                .collect(),
            ",",
        ),
    );
    record("config.motortest.buttons", joined(test.buttons(), ","));
    record(
        "config.motortest.labels",
        joined(
            (1..=test.motormax)
                .flat_map(|a| {
                    test.labels(a)
                        .into_iter()
                        .map(move |label| format!("{}={label}", motor::letter(a)))
                })
                .collect(),
            ";",
        ),
    );
    let (class_text, type_text) = test.frame_text();
    record("config.motortest.class", class_text);
    record("config.motortest.type", type_text);
    record("config.motortest.throttle", test.throttle.value());
    record("config.motortest.duration", test.duration.value());
    record("config.motortest.sent", test.sent);

    let press = test.press.as_ref();
    record(
        "config.motortest.press",
        press.map_or("none", |press| press.button.as_str()),
    );
    let last = press.and_then(|press| press.commands.last());
    let part = |pick: fn(&MotorCommand) -> i32| {
        last.map_or_else(
            || "none".to_owned(),
            |(command, ..)| pick(command).to_string(),
        )
    };
    record("config.motortest.last.motor", part(|c| c.motor));
    record("config.motortest.last.throttle", part(|c| c.throttle));
    record("config.motortest.last.duration", part(|c| c.duration));
    record("config.motortest.last.count", part(|c| c.count));
    record(
        "config.motortest.last.params",
        last.map_or_else(
            || "none".to_owned(),
            |(command, ..)| {
                command
                    .params()
                    .iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(" ")
            },
        ),
    );
    record(
        "config.motortest.commands",
        joined(
            press.map_or_else(Vec::new, |press| {
                press
                    .commands
                    .iter()
                    .map(|(c, ..)| format!("{}/{}/{}/{}", c.motor, c.throttle, c.duration, c.count))
                    .collect()
            }),
            ",",
        ),
    );
    record(
        "config.motortest.outcomes",
        joined(
            press.map_or_else(Vec::new, |press| {
                press
                    .commands
                    .iter()
                    .map(|(.., outcome)| outcome.clone().unwrap_or_else(|| "waiting".to_owned()))
                    .collect()
            }),
            ",",
        ),
    );
    // The vehicle's own answers since the press: the newest `COMMAND_ACK` line for the command,
    // and how many there have been.
    let prefix = ack_prefix();
    let acks: Vec<&str> = press.map_or_else(Vec::new, |press| {
        view.messages
            .iter()
            .filter(|message| message.seq > press.after && message.text.starts_with(&prefix))
            .map(|message| message.text.as_str())
            .collect()
    });
    record(
        "config.motortest.ack",
        acks.last().copied().unwrap_or("none"),
    );
    record("config.motortest.acks", acks.len());
    // What the vehicle is putting out on the motors' channels, `SERVO_OUTPUT_RAW` 1 to count.
    record(
        "config.motortest.outputs",
        joined(
            view.state.as_ref().map_or_else(Vec::new, |state| {
                state
                    .servo_outputs
                    .iter()
                    .take(test.motormax)
                    .map(ToString::to_string)
                    .collect()
            }),
            ",",
        ),
    );
    record(
        "config.motortest.message",
        test.message()
            .map_or("none", |message| message.text.as_str()),
    );
    record(
        "config.motortest.prompt",
        test.prompt()
            .map_or("none", |prompt| prompt.spin.question()),
    );
    record(
        "config.motortest.prompt.value",
        test.prompt().map_or("none", |prompt| prompt.field.value()),
    );
    record(
        "config.motortest.spin",
        test.last_spin.as_deref().unwrap_or("none"),
    );
    record("config.motortest.link", MOTOR_ORDER_URL);
    for name in [Spin::Arm.param(), Spin::Min.param()] {
        if let Some(value) = value_of(view, name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// An absolutely placed box, at a `.resx` `Location` and `Size`.
fn at(x: f32, y: f32, width: f32, height: f32) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// A label at its `Location`, as wide as `width`.
fn label(x: f32, y: f32, width: f32, text: impl Into<SharedString>, enabled: bool) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .text_xs()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(text.into())
}

/// A `MyButton` at its place: its text wrapped in its width, inert while the page is disabled.
fn button(
    id: String,
    (x, y, width, height): (f32, f32, f32, f32),
    text: impl Into<SharedString>,
    enabled: bool,
    on_click: impl Fn(&mut MissionPlanner, &mut Window, &mut Context<MissionPlanner>) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let body = crate::probe::measured(id.clone(), at(x, y, width, height))
        .id(SharedString::from(id))
        .flex()
        .items_center()
        .justify_center()
        .px_1()
        .rounded_sm()
        .border_1()
        .text_xs()
        .text_center()
        .child(text.into());
    if enabled {
        body.bg(rgb(theme::ACTION))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(cx.listener(move |this, _event, window, cx| {
                on_click(this, window, cx);
                cx.notify();
            }))
            .into_any_element()
    } else {
        body.bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::DIM))
            .into_any_element()
    }
}

/// Which number box.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BoxId {
    Throttle,
    Duration,
}

impl BoxId {
    const fn id(self) -> &'static str {
        match self {
            Self::Throttle => "motortest-throttle",
            Self::Duration => "motortest-duration",
        }
    }

    fn of(self, test: &mut MotorTest) -> &mut NumericUpDown {
        match self {
            Self::Throttle => &mut test.throttle,
            Self::Duration => &mut test.duration,
        }
    }
}

/// A `NumericUpDown` at its place: the text, typed into while it has the focus, and the up and
/// down arrows at its right.
fn number_box(
    which: BoxId,
    number: &NumericUpDown,
    handle: &FocusHandle,
    (x, y, width, height): (f32, f32, f32, f32),
    enabled: bool,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let focused = enabled && handle.is_focused(window);
    let text = crate::probe::measured(which.id(), div())
        .id(which.id())
        .flex_1()
        .h_full()
        .flex()
        .items_center()
        .px_1()
        .overflow_hidden()
        .text_xs()
        .child(number.field.value().to_owned())
        .children(focused.then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT))));
    let text = if enabled {
        text.track_focus(handle)
            .key_context("TextField")
            .cursor_text()
            .text_color(rgb(theme::TEXT))
            .on_key_down(cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                if which.of(&mut this.motor_test).key(event) {
                    cx.notify();
                }
            }))
    } else {
        text.text_color(rgb(theme::DIM))
    };
    let arrow = |suffix: &'static str, glyph: &'static str, delta: f64| {
        let id = format!("{}-{suffix}", which.id());
        let base = crate::probe::measured(id.clone(), div())
            .id(SharedString::from(id))
            .flex_1()
            .flex()
            .items_center()
            .justify_center()
            .text_size(px(7.0))
            .child(glyph);
        if enabled {
            base.text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    which.of(&mut this.motor_test).step(delta);
                    cx.notify();
                }))
                .into_any_element()
        } else {
            base.text_color(rgb(theme::DIM)).into_any_element()
        }
    };
    let arrows = div()
        .w(px(14.0))
        .h_full()
        .flex()
        .flex_col()
        .border_l_1()
        .border_color(rgb(theme::BORDER))
        .child(arrow("up", "▲", 1.0))
        .child(arrow("down", "▼", -1.0));
    at(x, y, width, height)
        .flex()
        .rounded_sm()
        .border_1()
        .border_color(rgb(if focused {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(if enabled { theme::ACTION } else { theme::PANEL }))
        .child(text)
        .child(arrows)
        .into_any_element()
}

/// The page, laid out as `ConfigMotorTest.resx` and `Activate` lay it out.
pub fn page(
    test: &MotorTest,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if !test.active {
        return None;
    }
    let enabled = test.enabled && test.prompt.is_none() && test.messages.is_empty();
    let dim = test.enabled;

    // The run-time buttons, down the left from 6, 75.
    let (x, mut y, step) = BUTTONS_AT;
    let mut runtime: Vec<AnyElement> = Vec::new();
    for a in 1..=test.motormax {
        let letter = motor::letter(a);
        runtime.push(button(
            format!("motortest-motor-{letter}"),
            (x, y, MOTOR_BUTTON.0, MOTOR_BUTTON.1),
            motor::button_text(a),
            enabled,
            move |this, _window, _cx| {
                let view = this.telemetry.view();
                this.motor_test.test_one(a, &mut this.telemetry, &view);
            },
            cx,
        ));
        for text in test.labels(a) {
            runtime.push(label(x + 85.0, y + 5.0, 150.0, text, dim).into_any_element());
        }
        y += step;
    }
    let (width, height, gap) = BIG_BUTTON;
    runtime.push(button(
        "motortest-all".to_owned(),
        (x, y, width, height),
        TEST_ALL,
        enabled,
        |this, _window, _cx| {
            let view = this.telemetry.view();
            this.motor_test.test_all(&mut this.telemetry, &view);
        },
        cx,
    ));
    y += gap;
    runtime.push(button(
        "motortest-stop".to_owned(),
        (x, y, width, height),
        STOP_ALL,
        enabled,
        |this, _window, _cx| {
            let view = this.telemetry.view();
            this.motor_test.stop_all(&mut this.telemetry, &view);
        },
        cx,
    ));
    y += gap;
    runtime.push(button(
        "motortest-sequence".to_owned(),
        (x, y, width, height),
        TEST_SEQUENCE,
        enabled,
        |this, _window, _cx| {
            let view = this.telemetry.view();
            this.motor_test.test_sequence(&mut this.telemetry, &view);
        },
        cx,
    ));
    let bottom = y + height;

    // groupBox1: AutoSize, so as tall as the buttons need.
    let group_height = GROUP_SIZE.1.max(bottom + 6.0);
    let spin_button = |spin: Spin, top: f32, cx: &mut Context<MissionPlanner>| {
        let (text, id) = match spin {
            Spin::Arm => (SPIN_ARM.0, "motortest-spin-arm"),
            Spin::Min => (SPIN_MIN.0, "motortest-spin-min"),
        };
        button(
            id.to_owned(),
            (282.0, top, 75.0, 41.0),
            text,
            enabled,
            move |this, window, cx| {
                let view = this.telemetry.view();
                this.motor_test.click_spin(spin, &view);
                if this.motor_test.prompt().is_some() {
                    this.motor_focus.prompt.focus(window, cx);
                }
            },
            cx,
        )
    };
    let link = crate::probe::measured("motortest-link", at(279.0, 157.0, 218.0, 26.0))
        .id("motortest-link")
        .text_xs()
        .text_color(rgb(if enabled { theme::ACCENT } else { theme::DIM }))
        .underline()
        .child(LINK_TEXT);
    let link = if enabled {
        link.cursor_pointer()
            .on_click(|_event, _window, cx| cx.open_url(MOTOR_ORDER_URL))
            .into_any_element()
    } else {
        link.into_any_element()
    };

    let group = at(0.0, 0.0, GROUP_SIZE.0, group_height)
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
                .text_color(rgb(if dim { theme::DIM } else { theme::BORDER }))
                .child(TITLE),
        )
        .child(label(6.0, 16.0, 60.0, THROTTLE_LABEL, dim))
        .child(number_box(
            BoxId::Throttle,
            &test.throttle,
            &focus.throttle,
            (66.0, 14.0, 54.0, 20.0),
            enabled,
            window,
            cx,
        ))
        .child(label(129.0, 16.0, 70.0, DURATION_LABEL, dim))
        .child(number_box(
            BoxId::Duration,
            &test.duration,
            &focus.duration,
            (201.0, 14.0, 54.0, 20.0),
            enabled,
            window,
            cx,
        ))
        .child(label(6.0, 45.0, 120.0, test.class_text.clone(), dim))
        .child(label(129.0, 45.0, 150.0, test.type_text.clone(), dim))
        .child(spin_button(Spin::Arm, 11.0, cx))
        .child(label(363.0, 11.0, 159.0, SPIN_ARM.1, dim))
        .child(spin_button(Spin::Min, 58.0, cx))
        .child(label(363.0, 58.0, 159.0, SPIN_MIN.1, dim))
        .child(label(279.0, 102.0, 214.0, NOTE, dim))
        .child(link)
        .children(runtime);

    let mut body = div()
        .relative()
        .w(px(GROUP_SIZE.0))
        .h(px(group_height))
        .child(group);
    if let Some(dialog) = dialog(test, focus, window, cx) {
        body = body.child(dialog);
    }
    Some(body.into_any_element())
}

/// The message box or the question showing, over the page: both are modal in the C#.
fn dialog(
    test: &MotorTest,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let card = |border: u32| {
        div()
            .flex()
            .flex_col()
            .gap_2()
            .w(px(340.0))
            .p_3()
            .bg(rgb(theme::PANEL))
            .border_1()
            .border_color(rgb(border))
            .rounded_md()
    };
    let dialog = if let Some(message) = test.message() {
        crate::probe::measured("motortest-message", card(theme::ALERT))
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(message.title),
            )
            .child(
                div()
                    .text_sm()
                    .text_color(rgb(theme::TEXT))
                    .child(message.text.clone()),
            )
            .child(div().flex().justify_end().child(crate::ui::action(
                "motortest-message-ok",
                "OK",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.motor_test.dismiss_message();
                    cx.notify();
                }),
            )))
    } else {
        let prompt = test.prompt()?;
        let focused = focus.prompt.is_focused(window);
        crate::probe::measured("motortest-prompt", card(theme::ACCENT))
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(CHANGE_THROTTLE),
            )
            .child(
                div()
                    .text_sm()
                    .text_color(rgb(theme::TEXT))
                    .child(prompt.spin.question()),
            )
            .child(crate::textfield::text_field(
                "motortest-prompt-value",
                &prompt.field,
                &focus.prompt,
                focused,
                px(310.0),
                cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                    if this.motor_test.prompt_key(event, &this.telemetry) {
                        cx.notify();
                    }
                }),
            ))
            .child(
                div()
                    .flex()
                    .justify_end()
                    .gap_2()
                    .child(crate::ui::action(
                        "motortest-prompt-ok",
                        "OK",
                        theme::ACCENT,
                        true,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.motor_test.answer(&this.telemetry);
                            cx.notify();
                        }),
                    ))
                    .child(crate::ui::action(
                        "motortest-prompt-cancel",
                        "Cancel",
                        theme::DIM,
                        true,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.motor_test.cancel();
                            cx.notify();
                        }),
                    )),
            )
    };
    Some(
        div()
            .id("motortest-dialog-backdrop")
            .absolute()
            .inset_0()
            .flex()
            .items_center()
            .justify_center()
            .occlude()
            .child(dialog)
            .into_any_element(),
    )
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;
    use crate::telemetry::scripted::{VEHICLE, Vehicle, ack, until};
    use mp_link::ProtocolTimeouts;
    use mp_link::requests::MAV_RESULT_ACCEPTED;
    use mp_mavlink_dialects::all::MavMessage;
    use mp_vehicle::{VehicleId, VehicleState};

    /// The bundled documentation.
    fn bundled(name: &str) -> Option<&'static mp_params::ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    /// A connected quad holding these parameters.
    fn view_with(parameters: &[(&str, f64)]) -> TelemetryView {
        let mut view = TelemetryView::disconnected("tcp:127.0.0.1:5760");
        view.connected = true;
        view.vehicle = Some(VehicleId::new(1, 1));
        let mut state = VehicleState::new(1, 1);
        state.vehicle_type = 2;
        view.state = Some(Arc::new(state));
        view.parameters = parameters
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect();
        view
    }

    /// SITL's copter: a quad plus, with the firmware's spin defaults.
    fn sitl() -> TelemetryView {
        view_with(&[
            ("FRAME_CLASS", 1.0),
            ("FRAME_TYPE", 0.0),
            ("MOT_SPIN_ARM", 0.1),
            ("MOT_SPIN_MIN", 0.15),
        ])
    }

    /// The link's waits, divided so a test runs the C#'s full retry ladder in a blink.
    fn fast() -> ProtocolTimeouts {
        ProtocolTimeouts::default().faster(20)
    }

    fn key(name: &str, character: Option<&str>) -> KeyDownEvent {
        KeyDownEvent {
            keystroke: gpui::Keystroke {
                modifiers: gpui::Modifiers::default(),
                key: name.to_owned(),
                key_char: character.map(ToOwned::to_owned),
            },
            is_held: false,
            prefer_character_input: false,
        }
    }

    #[test]
    fn activating_on_sitls_quad_builds_four_lettered_labelled_buttons() {
        let mut test = MotorTest::default();
        assert_eq!(test.frame_text(), UNKNOWN);
        test.activate(&sitl(), bundled);
        assert!(test.is_active());
        assert!(test.enabled());
        assert_eq!(test.motor_count(), 4);
        assert_eq!(
            test.buttons(),
            [
                "Test motor A",
                "Test motor B",
                "Test motor C",
                "Test motor D"
            ]
        );
        assert_eq!(test.labels(1), ["Motor Number: 3, CW"]);
        assert_eq!(test.labels(4), ["Motor Number: 2, CCW"]);
        assert_eq!(test.frame_text(), ("Class: Quad", "Type: Plus"));
    }

    #[test]
    fn a_value_the_documentation_does_not_list_leaves_the_label_as_it_was() {
        let mut test = MotorTest::default();
        test.activate(
            &view_with(&[("FRAME_CLASS", 99.0), ("FRAME_TYPE", 0.0)]),
            bundled,
        );
        assert_eq!(test.frame_text(), (UNKNOWN.0, "Type: Plus"));
    }

    #[test]
    fn a_vehicle_with_no_frame_disables_the_page() {
        let mut test = MotorTest::default();
        test.activate(&view_with(&[("ARSPD_TYPE", 1.0)]), bundled);
        assert!(!test.enabled());
        assert_eq!(test.motor_count(), 8);
        // A disabled page sends nothing.
        let mut telemetry = Telemetry::idle();
        test.test_one(1, &mut telemetry, &sitl());
        test.stop_all(&mut telemetry, &sitl());
        assert!(test.last_commands().is_empty());
    }

    #[test]
    fn the_boxes_start_at_the_designers_values_and_bounds() {
        let mut test = MotorTest::default();
        assert!((test.throttle.value() - 5.0).abs() < f64::EPSILON);
        assert!((test.duration.value() - 2.0).abs() < f64::EPSILON);
        test.throttle.type_text("150");
        assert_eq!(test.throttle.int(), 100);
        test.throttle.type_text("-150");
        assert_eq!(test.throttle.int(), -100);
        test.duration.type_text("1000");
        assert_eq!(test.duration.int(), 999);
        test.duration.type_text("-1");
        assert_eq!(test.duration.int(), 0);
        // Text that does not parse is dropped and the last value shown again.
        test.duration.type_text("soon");
        assert_eq!(test.duration.int(), 0);
        assert_eq!(test.duration.field.value(), "0");
        // A fraction is kept, shown rounded and read truncated, as a decimal Value is.
        test.throttle.type_text("7.5");
        assert_eq!(test.throttle.int(), 7);
        assert_eq!(test.throttle.field.value(), "8");
    }

    #[test]
    fn the_arrows_and_keys_step_by_one_within_the_bounds() {
        let mut test = MotorTest::default();
        test.throttle.step(1.0);
        assert_eq!(test.throttle.int(), 6);
        assert!(test.throttle.key(&key("down", None)));
        assert_eq!(test.throttle.int(), 5);
        for _ in 0..200 {
            test.throttle.step(1.0);
        }
        assert_eq!(test.throttle.int(), 100);
        // Typing, then Enter, validates.
        test.duration.key(&key("backspace", None));
        test.duration.key(&key("9", Some("9")));
        assert!((test.duration.value() - 9.0).abs() < f64::EPSILON);
        test.duration.key(&key("enter", None));
        assert_eq!(test.duration.field.value(), "9");
    }

    #[test]
    fn each_button_makes_the_csharps_calls() {
        let mut test = MotorTest::default();
        test.activate(&sitl(), bundled);
        let mut telemetry = Telemetry::idle();
        let view = sitl();
        test.test_one(3, &mut telemetry, &view);
        assert_eq!(test.last_commands(), [motor::single(3, 5, 2)]);
        test.test_all(&mut telemetry, &view);
        assert_eq!(test.last_commands(), motor::test_all(4, 5, 2));
        test.test_sequence(&mut telemetry, &view);
        assert_eq!(test.last_commands(), [motor::sequence(4, 5, 2)]);
        test.throttle.type_text("12");
        test.stop_all(&mut telemetry, &view);
        assert_eq!(test.last_commands(), motor::stop_all(4));
        // A button past the count is not one the page has.
        test.test_one(5, &mut telemetry, &view);
        assert_eq!(test.last_commands(), motor::stop_all(4));
        assert_eq!(test.sent, 10);
    }

    #[test]
    fn spin_arm_asks_for_the_throttle_plus_two() {
        let mut test = MotorTest::default();
        test.activate(&sitl(), bundled);
        test.click_spin(Spin::Arm, &sitl());
        assert!(!test.enabled(), "the handler disables the page");
        let prompt = test.prompt().expect("the InputBox");
        assert_eq!(prompt.field.value(), "7");
        assert_eq!(
            prompt.spin.question(),
            "Enter arm throttle % (deadzone + 2%)"
        );
        test.cancel();
        assert!(test.enabled());
        assert!(test.prompt().is_none());
    }

    #[test]
    fn spin_min_asks_for_the_parameter_cast_to_int_plus_three() {
        // (int)0.15 + 3: the fraction truncates to 0.
        let mut test = MotorTest::default();
        test.activate(&sitl(), bundled);
        test.click_spin(Spin::Min, &sitl());
        assert_eq!(test.prompt().map(|p| p.field.value()), Some("3"));
    }

    #[test]
    fn a_throttle_of_twenty_or_more_is_too_high_for_either() {
        let mut test = MotorTest::default();
        test.activate(&sitl(), bundled);
        test.throttle.type_text("20");
        test.click_spin(Spin::Min, &sitl());
        assert!(test.prompt().is_none());
        assert_eq!(test.message().map(|m| m.text.as_str()), Some(TOO_HIGH));
        assert!(
            test.enabled(),
            "the else branch falls through to Enabled = true"
        );
    }

    #[test]
    fn a_missing_parameter_is_said_and_leaves_the_page_disabled() {
        let mut test = MotorTest::default();
        test.activate(
            &view_with(&[("FRAME_CLASS", 1.0), ("FRAME_TYPE", 0.0)]),
            bundled,
        );
        test.click_spin(Spin::Arm, &sitl_without_spin());
        assert_eq!(
            test.message().map(|m| m.text.as_str()),
            Some("param MOT_SPIN_ARM missing")
        );
        assert!(!test.enabled(), "the C# returns before Enabled = true");
    }

    fn sitl_without_spin() -> TelemetryView {
        view_with(&[("FRAME_CLASS", 1.0), ("FRAME_TYPE", 0.0)])
    }

    #[test]
    fn an_answer_that_is_not_a_whole_number_escapes_as_a_format_exception() {
        let mut test = MotorTest::default();
        test.activate(&sitl(), bundled);
        test.click_spin(Spin::Arm, &sitl());
        test.prompt.as_mut().unwrap().field.set("7.5");
        test.answer(&Telemetry::idle());
        let message = test.message().expect("the unhandled exception's box");
        assert_eq!(message.title, "Send Error");
        assert!(message.text.contains("FormatException"), "{}", message.text);
        assert!(!test.enabled());
    }

    #[test]
    fn int_parse_takes_what_the_csharps_does() {
        assert_eq!(int_parse(" 7 "), Some(7));
        assert_eq!(int_parse("+7"), Some(7));
        assert_eq!(int_parse("-2"), Some(-2));
        assert_eq!(int_parse("7.0"), None);
        assert_eq!(int_parse(""), None);
        assert_eq!(int_parse("-"), None);
    }

    #[test]
    fn leaving_the_page_cancels_the_question_and_keeps_nothing_open() {
        let mut test = MotorTest::default();
        test.activate(&sitl(), bundled);
        test.click_spin(Spin::Arm, &sitl());
        test.deactivate();
        assert!(!test.is_active());
        assert!(test.prompt().is_none());
        assert!(test.enabled());
    }

    /// The path the product takes: a motor button through `Telemetry::test_motor` to the link's
    /// retrying command, the vehicle hearing `MAV_CMD_DO_MOTOR_TEST` with the C#'s parameters,
    /// and its `COMMAND_ACK` ending the request.
    #[test]
    fn a_motor_button_puts_do_motor_test_on_the_wire_and_the_ack_ends_it() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut test = MotorTest::default();
        test.activate(&sitl(), bundled);
        test.test_one(2, &mut telemetry, &sitl());

        let mut heard = Vec::new();
        until("the vehicle to hear the command", || {
            for message in vehicle.read() {
                if let MavMessage::CommandLong(long) = message {
                    heard.push(long);
                    vehicle.send(&ack(long.command, MAV_RESULT_ACCEPTED));
                }
            }
            !heard.is_empty()
        });
        let long = heard[0];
        assert_eq!(long.command, CMD_DO_MOTOR_TEST);
        assert_eq!(
            (long.target_system, long.target_component),
            (VEHICLE.sysid, VEHICLE.compid)
        );
        assert_eq!(
            [
                long.param1,
                long.param2,
                long.param3,
                long.param4,
                long.param5,
                long.param6,
                long.param7
            ],
            [2.0, 0.0, 5.0, 2.0, 0.0, 0.0, 0.0]
        );
        until("the request to end", || {
            test.tick(&telemetry, [false; 2]);
            test.press
                .as_ref()
                .is_some_and(|press| press.commands.iter().all(|(.., o)| o.is_some()))
        });
        let outcomes: Vec<Option<String>> = test
            .press
            .as_ref()
            .unwrap()
            .commands
            .iter()
            .map(|(.., o)| o.clone())
            .collect();
        assert_eq!(outcomes, [Some("accepted".to_owned())]);
    }

    /// Stop all motors: four commands at once, each answered in turn, each sent once.
    #[test]
    fn stop_all_sends_every_motor_and_each_ack_answers_one() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut test = MotorTest::default();
        test.activate(&sitl(), bundled);
        test.stop_all(&mut telemetry, &sitl());
        let mut heard = Vec::new();
        until("four commands", || {
            for message in vehicle.read() {
                if let MavMessage::CommandLong(long) = message {
                    heard.push((long.param1, long.param3, long.param4, long.confirmation));
                    vehicle.send(&ack(long.command, MAV_RESULT_ACCEPTED));
                }
            }
            heard.len() >= 4
        });
        assert_eq!(
            heard,
            [
                (1.0, 0.0, 0.0, 0),
                (2.0, 0.0, 0.0, 0),
                (3.0, 0.0, 0.0, 0),
                (4.0, 0.0, 0.0, 0)
            ]
        );
        until("every request to end", || {
            test.tick(&telemetry, [false; 2]);
            test.press
                .as_ref()
                .is_some_and(|press| press.commands.iter().all(|(.., o)| o.is_some()))
        });
        let press = test.press.as_ref().unwrap();
        assert!(
            press
                .commands
                .iter()
                .all(|(.., o)| o.as_deref() == Some("accepted")),
            "{:?}",
            press.commands
        );
    }

    /// The Spin Arm answer is written as a fraction through the retrying set, and the page is
    /// enabled once the vehicle echoes it.
    #[test]
    fn a_spin_arm_answer_is_written_over_a_hundred_and_the_echo_enables_the_page() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        // The vehicle lists MOT_SPIN_ARM, so setParam will send it.
        vehicle.send(&crate::telemetry::scripted::param("MOT_SPIN_ARM", 0.1, 9));
        until("the parameter to be held", || {
            telemetry.holds_parameter("MOT_SPIN_ARM")
        });
        let mut test = MotorTest::default();
        test.activate(&sitl(), bundled);
        test.click_spin(Spin::Arm, &sitl());
        test.answer(&telemetry);
        assert!(!test.enabled(), "disabled until the write ends");
        let mut written = None;
        until("the PARAM_SET", || {
            for message in vehicle.read() {
                if let MavMessage::ParamSet(set) = message {
                    written = Some(set.param_value);
                    vehicle.send(&crate::telemetry::scripted::param(
                        "MOT_SPIN_ARM",
                        set.param_value,
                        9,
                    ));
                }
            }
            written.is_some()
        });
        assert_eq!(written, Some(7.0_f32 / 100.0));
        until("the page to be enabled", || {
            test.tick(&telemetry, [false; 2]);
            test.enabled()
        });
        assert!(
            test.last_spin.as_deref().is_some_and(
                |spin| spin.starts_with("MOT_SPIN_ARM 0.07") && spin.ends_with("accepted")
            ),
            "{:?}",
            test.last_spin
        );
    }

    /// Every fact the GUI script asserts on is one this page records, and every control it
    /// clicks is one this page draws: a renamed fact or id fails here rather than as a script
    /// that cannot find what it looks for.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-motortest.gui");
        let source = include_str!("motor_test.rs");
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.motortest.") => {
                    assert!(
                        source.contains(&format!("\"{key}\"")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("motortest-") => {
                    let drawn = source.contains(&format!("\"{id}\""))
                        || (id.starts_with("motortest-motor-")
                            && source.contains("\"motortest-motor-{letter}\""))
                        || ["-up", "-down"].iter().any(|arrow| {
                            id.strip_suffix(arrow)
                                .is_some_and(|base| source.contains(&format!("\"{base}\"")))
                        });
                    assert!(drawn, "{id} is not drawn");
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 40 && clicks > 10, "{facts} facts, {clicks} clicks");
    }
}
