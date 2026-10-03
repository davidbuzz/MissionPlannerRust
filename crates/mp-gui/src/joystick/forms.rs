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

//! The button functions' settings forms, which a button row's Settings opens: `Joy_ChangeMode`,
//! `Joy_Mount_Mode`, `Joy_Do_Set_Relay`, `Joy_Do_Repeat_Relay`, `Joy_Do_Set_Servo`,
//! `Joy_Do_Repeat_Servo` and `Joy_Button_axis`.
//!
//! Each is a small dialog - `ShowDialog` - with a combo box or a few `NumericUpDown`s at their
//! Designer places, filled from the function's `JoyButton` in its constructor, and a handler per
//! control that writes what it holds back into the button at once. Nothing is saved: the
//! joystick object holds it until the page's Save.
//!
//! A `NumericUpDown` here is its text and its arrows. Its value is the text read and held to the
//! control's range - which is what the C#'s `Value` reads once the constructor has set `Text` -
//! so an arrow steps from what is shown, and a handler that writes every box's value writes each
//! as shown.
//! `// C#: Joystick/Joy_ChangeMode.cs; Joy_Mount_Mode.cs; Joy_Do_Set_Relay.cs; Joy_Do_Repeat_Relay.cs;
//! Joy_Do_Set_Servo.cs; Joy_Do_Repeat_Servo.cs; Joy_Button_axis.cs, and each .Designer.cs`

use mp_input::{ButtonFunction, JoyButton};

use crate::textfield::TextField;

/// A place: x, y, width, height.
pub type Place = (f32, f32, f32, f32);

/// Which form.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FormKind {
    /// `Joy_ChangeMode`.
    ChangeMode,
    /// `Joy_Mount_Mode`.
    MountMode,
    /// `Joy_Do_Set_Relay`.
    DoSetRelay,
    /// `Joy_Do_Repeat_Relay`.
    DoRepeatRelay,
    /// `Joy_Do_Set_Servo`.
    DoSetServo,
    /// `Joy_Do_Repeat_Servo`.
    DoRepeatServo,
    /// `Joy_Button_axis`, for both button axes.
    ButtonAxis,
}

impl FormKind {
    /// `but_settings_Click`'s switch: the form a function has, or `None` for "No settings to
    /// set".
    /// `// C#: Joystick/JoystickSetup.cs:453-487`
    #[must_use]
    pub const fn of(function: ButtonFunction) -> Option<Self> {
        Some(match function {
            ButtonFunction::ChangeMode => Self::ChangeMode,
            ButtonFunction::MountMode => Self::MountMode,
            ButtonFunction::DoRepeatRelay => Self::DoRepeatRelay,
            ButtonFunction::DoRepeatServo => Self::DoRepeatServo,
            ButtonFunction::DoSetRelay => Self::DoSetRelay,
            ButtonFunction::DoSetServo => Self::DoSetServo,
            ButtonFunction::ButtonAxis0 | ButtonFunction::ButtonAxis1 => Self::ButtonAxis,
            _ => return None,
        })
    }

    /// The form's `Text`, its caption - `Joy_Mount_Mode`'s Designer gives it `Joy_ChangeMode`'s.
    /// `// C#: Joystick/Joy_*.Designer.cs (this.Text)`
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::ChangeMode | Self::MountMode => "Joy_ChangeMode",
            Self::DoSetRelay => "Joy_Do_Set_Relay",
            Self::DoRepeatRelay => "Joy_Do_Repeat_Relay",
            Self::DoSetServo => "Joy_Do_Set_Servo",
            Self::DoRepeatServo => "Joy_Do_Repeat_Servo",
            Self::ButtonAxis => "Joy_Button_axis",
        }
    }

    /// The class, for the facts: two share a caption.
    #[must_use]
    pub const fn class(self) -> &'static str {
        match self {
            Self::ChangeMode => "Joy_ChangeMode",
            Self::MountMode => "Joy_Mount_Mode",
            Self::DoSetRelay => "Joy_Do_Set_Relay",
            Self::DoRepeatRelay => "Joy_Do_Repeat_Relay",
            Self::DoSetServo => "Joy_Do_Set_Servo",
            Self::DoRepeatServo => "Joy_Do_Repeat_Servo",
            Self::ButtonAxis => "Joy_Button_axis",
        }
    }

    /// `ClientSize`.
    #[must_use]
    pub const fn client(self) -> (f32, f32) {
        match self {
            Self::ChangeMode => (148.0, 43.0),
            Self::MountMode => (282.0, 120.0),
            Self::DoSetRelay => (145.0, 37.0),
            Self::DoRepeatRelay => (363.0, 37.0),
            Self::DoSetServo => (258.0, 36.0),
            Self::DoRepeatServo => (517.0, 33.0),
            Self::ButtonAxis => (298.0, 36.0),
        }
    }
}

/// `Joy_Mount_Mode`'s `label1`, from its `.resx`.
/// `// C#: Joystick/Joy_Mount_Mode.resx (label1.Text)`
pub const MOUNT_MODE_HELP: &str = "Retract: pull up mount to preset positions\nNeutral: load default position, ie forwards\nMavlink_targeting: point to a recevied roll, pitch, yaw\nrc_targeting: use rc channels to control axis's\ngps_point: used to track a single ground point";

/// `Joy_Mount_Mode`'s `comboBox1` at (70, 12), 121 by 21; `Joy_ChangeMode`'s at (13, 13).
const MOUNT_COMBO: Place = (70.0, 12.0, 121.0, 21.0);
const MODE_COMBO: Place = (13.0, 13.0, 121.0, 21.0);
/// `Joy_Mount_Mode`'s `label1`.
pub const MOUNT_HELP_AT: (f32, f32) = (12.0, 37.0);

/// The parameters `Joy_Mount_Mode` takes its list from, the first with any options.
/// `// C#: Joystick/Joy_Mount_Mode.cs:18-29`
pub const MOUNT_MODE_PARAMS: [&str; 3] = ["MNT1_DEFLT_MODE", "MNT_DEFLT_MODE", "MNT_MODE"];

/// Which of the button's `p` fields a box's handler writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Writes {
    /// Its own field alone: `p1` or `p2`.
    One(usize),
    /// Every box's value into `p1` onwards, as the handlers the Designer shares between them do.
    All,
}

/// One `NumericUpDown`: its label, its place, its range and what it holds.
#[derive(Debug)]
pub struct Number {
    /// The label beside it, and where.
    pub label: &'static str,
    /// The label's place.
    pub label_at: (f32, f32),
    /// The box's place.
    pub place: Place,
    /// `Minimum`.
    pub min: f64,
    /// `Maximum`.
    pub max: f64,
    /// The Designer's `Value`, which a text that does not read leaves it at.
    designer: f64,
    /// What is shown.
    pub text: TextField,
    writes: Writes,
}

impl Number {
    fn new(
        label: &'static str,
        label_at: (f32, f32),
        place: Place,
        (min, max, designer): (f64, f64, f64),
        writes: Writes,
        shown: f32,
    ) -> Self {
        let mut text = TextField::new("");
        // `numericUpDownN.Text = config.pN.ToString()`.
        text.set(format!("{shown}"));
        Self {
            label,
            label_at,
            place,
            min,
            max,
            designer,
            text,
            writes,
        }
    }

    /// `Value`: the text read and held to the range, or the Designer's value for text that does
    /// not read.
    #[must_use]
    pub fn value(&self) -> f64 {
        self.text
            .value()
            .trim()
            .parse::<f64>()
            .ok()
            .filter(|value| value.is_finite())
            .map_or(self.designer, |value| value.clamp(self.min, self.max))
    }

    /// The value as the box shows it once read: no decimal places.
    fn show(&mut self, value: f64) {
        self.text
            .set(crate::config::servo_output::decimal_text(value, 0));
    }
}

/// A form's combo box: its list and the row selected.
#[derive(Debug, Clone, PartialEq)]
pub struct FormCombo {
    /// The place.
    pub place: Place,
    /// `DataSource`: the values and their text.
    pub options: Vec<(i64, String)>,
    /// `SelectedIndex`.
    pub selected: Option<usize>,
}

impl FormCombo {
    /// The text shown.
    #[must_use]
    pub fn text(&self) -> &str {
        self.selected
            .and_then(|index| self.options.get(index))
            .map_or("", |(_, text)| text.as_str())
    }
}

/// An open form.
#[derive(Debug)]
pub struct Form {
    /// Which.
    pub kind: FormKind,
    /// `Tag`: the button's slot in `JoyButtons`.
    pub slot: usize,
    /// `comboBox1`, on the two forms that have one.
    pub combo: Option<FormCombo>,
    /// The `NumericUpDown`s, in the Designer's order.
    pub numbers: Vec<Number>,
}

impl Form {
    /// The constructor: the controls made, filled from `button`, and whatever the filling's own
    /// events write back - `Joy_ChangeMode` setting `Text` on its combo raises
    /// `SelectedIndexChanged` when that moves the selection, and so does `Joy_Mount_Mode` setting
    /// `SelectedValue`.
    ///
    /// `modes` is `Common.getModesList(cs.firmware)`; `mount_options` the first of
    /// [`MOUNT_MODE_PARAMS`] with documented options.
    /// `// C#: Joystick/Joy_ChangeMode.cs:8-25; Joy_Mount_Mode.cs:10-36; Joy_Do_Set_Relay.cs:8-19;
    /// Joy_Do_Repeat_Relay.cs:8-21; Joy_Do_Set_Servo.cs:8-20; Joy_Do_Repeat_Servo.cs:8-22;
    /// Joy_Button_axis.cs:8-20`
    #[must_use]
    pub fn open(
        kind: FormKind,
        slot: usize,
        button: &mut JoyButton,
        modes: Vec<(i64, String)>,
        mount_options: Vec<(i64, String)>,
    ) -> Self {
        let number = |label, label_at, place, range, writes, shown| {
            Number::new(label, label_at, place, range, writes, shown)
        };
        // `NumericUpDown`'s own range is 0 to 100 unless the Designer says otherwise.
        const PLAIN: (f64, f64, f64) = (0.0, 100.0, 0.0);
        let mut form = Self {
            kind,
            slot,
            combo: None,
            numbers: Vec::new(),
        };
        match kind {
            FormKind::ChangeMode => {
                // The list binds with its first row selected; `Text = config.mode` then selects
                // the row of that name, as `FindStringExact` finds it - ignoring case - or none.
                let found = button.mode.as_deref().and_then(|mode| {
                    modes
                        .iter()
                        .position(|(_, name)| name.eq_ignore_ascii_case(mode))
                });
                let before = (!modes.is_empty()).then_some(0);
                let combo = FormCombo {
                    place: MODE_COMBO,
                    options: modes,
                    selected: found,
                };
                if combo.selected != before {
                    // `comboBox1_SelectedIndexChanged`: function and mode written.
                    button.function = ButtonFunction::ChangeMode;
                    button.mode = Some(combo.text().to_owned());
                }
                form.combo = Some(combo);
            }
            FormKind::MountMode => {
                let empty = mount_options.is_empty();
                #[allow(clippy::cast_possible_truncation)] // `(int)config.p1`
                let wanted = i64::from(button.p1 as i32);
                let found = mount_options.iter().position(|(key, _)| *key == wanted);
                let combo = FormCombo {
                    place: MOUNT_COMBO,
                    options: mount_options,
                    // No `DataSource` without options, so nothing to select.
                    selected: if empty { None } else { found },
                };
                if !empty && found.is_some_and(|index| index != 0) {
                    button.function = ButtonFunction::MountMode;
                    #[allow(clippy::cast_precision_loss)] // mount modes are small integers
                    if let Some((key, _)) = combo.selected.and_then(|i| combo.options.get(i)) {
                        button.p1 = *key as f32;
                    }
                }
                // A `p1` the list does not hold: `SelectedValue` is null and the handler's `(int)`
                // throws. Nothing is written here.
                form.combo = Some(combo);
            }
            FormKind::DoSetRelay => {
                form.numbers = vec![number(
                    "Relay No#",
                    (12.0, 9.0),
                    (76.0, 7.0, 57.0, 20.0),
                    (0.0, 3.0, 0.0),
                    Writes::One(1),
                    button.p1,
                )];
            }
            FormKind::DoRepeatRelay => {
                form.numbers = vec![
                    number(
                        "Relay No#",
                        (12.0, 9.0),
                        (76.0, 7.0, 57.0, 20.0),
                        (0.0, 3.0, 0.0),
                        Writes::All,
                        button.p1,
                    ),
                    number(
                        "Repeat #",
                        (139.0, 9.0),
                        (197.0, 7.0, 57.0, 20.0),
                        PLAIN,
                        Writes::All,
                        button.p2,
                    ),
                    number(
                        "Time",
                        (260.0, 9.0),
                        (296.0, 7.0, 57.0, 20.0),
                        PLAIN,
                        Writes::All,
                        button.p3,
                    ),
                ];
            }
            FormKind::DoSetServo => {
                form.numbers = vec![
                    number(
                        "Servo No#",
                        (12.0, 9.0),
                        (77.0, 7.0, 47.0, 20.0),
                        PLAIN,
                        Writes::One(1),
                        button.p1,
                    ),
                    number(
                        "PWM",
                        (130.0, 9.0),
                        (171.0, 7.0, 78.0, 20.0),
                        (900.0, 2100.0, 1500.0),
                        Writes::One(2),
                        button.p2,
                    ),
                ];
            }
            FormKind::DoRepeatServo => {
                form.numbers = vec![
                    number(
                        "Servo No#",
                        (12.0, 9.0),
                        (77.0, 7.0, 57.0, 20.0),
                        PLAIN,
                        Writes::All,
                        button.p1,
                    ),
                    number(
                        "Pwm Value",
                        (140.0, 9.0),
                        (205.0, 7.0, 57.0, 20.0),
                        (900.0, 2100.0, 1500.0),
                        Writes::All,
                        button.p2,
                    ),
                    number(
                        "Rep Time",
                        (268.0, 9.0),
                        (327.0, 7.0, 57.0, 20.0),
                        PLAIN,
                        Writes::All,
                        button.p3,
                    ),
                    number(
                        "Delay (ms)",
                        (390.0, 9.0),
                        (452.0, 7.0, 57.0, 20.0),
                        PLAIN,
                        Writes::All,
                        button.p4,
                    ),
                ];
            }
            FormKind::ButtonAxis => {
                form.numbers = vec![
                    number(
                        "PWM 1",
                        (12.0, 9.0),
                        (77.0, 7.0, 47.0, 20.0),
                        (800.0, 2200.0, 800.0),
                        Writes::One(1),
                        button.p1,
                    ),
                    number(
                        "PWM 2",
                        (130.0, 9.0),
                        (193.0, 7.0, 78.0, 20.0),
                        (800.0, 2200.0, 1500.0),
                        Writes::One(2),
                        button.p2,
                    ),
                ];
            }
        }
        form
    }

    /// A row chosen from the combo: `comboBox1_SelectedIndexChanged`, when the selection moved.
    /// `// C#: Joystick/Joy_ChangeMode.cs:27-41; Joy_Mount_Mode.cs:38-52`
    pub fn choose(&mut self, index: usize, button: &mut JoyButton) -> bool {
        let kind = self.kind;
        let Some(combo) = self.combo.as_mut() else {
            return false;
        };
        if combo.selected == Some(index) || index >= combo.options.len() {
            return false;
        }
        combo.selected = Some(index);
        match kind {
            FormKind::ChangeMode => {
                button.function = ButtonFunction::ChangeMode;
                button.mode = Some(combo.text().to_owned());
            }
            FormKind::MountMode => {
                button.function = ButtonFunction::MountMode;
                #[allow(clippy::cast_precision_loss)] // mount modes are small integers
                if let Some((key, _)) = combo.options.get(index) {
                    button.p1 = *key as f32;
                }
            }
            _ => return false,
        }
        true
    }

    /// An arrow on a box: its value a step up or down, held to its range, and `ValueChanged`'s
    /// write if it moved.
    pub fn step(&mut self, index: usize, up: bool, button: &mut JoyButton) -> bool {
        let Some(number) = self.numbers.get_mut(index) else {
            return false;
        };
        let before = number.value();
        let after = (if up { before + 1.0 } else { before - 1.0 }).clamp(number.min, number.max);
        let shown_before = number.text.value().to_owned();
        number.show(after);
        if after == before && shown_before == number.text.value() {
            return false;
        }
        self.write(index, button);
        true
    }

    /// Typed text read into a box's value - Enter, or the box left - and `ValueChanged`'s write.
    pub fn commit(&mut self, index: usize, button: &mut JoyButton) -> bool {
        let Some(number) = self.numbers.get_mut(index) else {
            return false;
        };
        let value = number.value();
        number.show(value);
        self.write(index, button);
        true
    }

    /// The box's `ValueChanged` handler: its own field, or every box's into `p1` onwards.
    /// `// C#: Joystick/Joy_Do_Set_Relay.cs:21-28; Joy_Do_Repeat_Relay.cs:23-32;
    /// Joy_Do_Set_Servo.cs:22-38; Joy_Do_Repeat_Servo.cs:24-34; Joy_Button_axis.cs:22-38`
    fn write(&self, index: usize, button: &mut JoyButton) {
        let Some(number) = self.numbers.get(index) else {
            return;
        };
        #[allow(clippy::cast_possible_truncation)] // `(float)numericUpDown.Value`
        let field = |value: f64| value as f32;
        match number.writes {
            Writes::One(1) => button.p1 = field(number.value()),
            Writes::One(_) => button.p2 = field(number.value()),
            Writes::All => {
                let values: Vec<f32> = self.numbers.iter().map(|n| field(n.value())).collect();
                let slots = [
                    &mut button.p1,
                    &mut button.p2,
                    &mut button.p3,
                    &mut button.p4,
                ];
                for (slot, value) in slots.into_iter().zip(values) {
                    *slot = value;
                }
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::indexing_slicing)]
mod tests {
    use super::*;

    fn with(function: ButtonFunction) -> JoyButton {
        JoyButton {
            function,
            ..JoyButton::unassigned()
        }
    }

    /// Settings opens the form the function has, and none for the rest.
    #[test]
    fn each_function_opens_its_form() {
        assert_eq!(FormKind::of(ButtonFunction::DoSetServo), Some(FormKind::DoSetServo));
        assert_eq!(FormKind::of(ButtonFunction::ButtonAxis1), Some(FormKind::ButtonAxis));
        assert_eq!(FormKind::of(ButtonFunction::Arm), None);
        assert_eq!(FormKind::of(ButtonFunction::GimbalPntTrack), None);
        assert_eq!(FormKind::MountMode.title(), "Joy_ChangeMode", "the Designer's copy");
    }

    /// `Joy_Do_Set_Servo`: the box shows the button's `p1`, an arrow writes `p1` alone; the PWM
    /// box, 900 to 2100, reads the shown 0 as 900 and steps from there.
    #[test]
    fn the_servo_form_writes_each_box_to_its_own_field() {
        let mut button = with(ButtonFunction::DoSetServo);
        button.p1 = 4.0;
        let mut form = Form::open(FormKind::DoSetServo, 2, &mut button, Vec::new(), Vec::new());
        assert_eq!(form.numbers[0].text.value(), "4");
        assert_eq!(form.numbers[1].text.value(), "0");
        assert!(form.step(0, true, &mut button));
        assert!((button.p1 - 5.0).abs() < f32::EPSILON);
        assert!(button.p2.abs() < f32::EPSILON, "the other box writes nothing");
        assert!(form.step(1, true, &mut button));
        assert!((button.p2 - 901.0).abs() < f32::EPSILON);
        form.numbers[1].text.set("5000");
        assert!(form.commit(1, &mut button));
        assert!((button.p2 - 2100.0).abs() < f32::EPSILON, "held to the maximum");
        assert_eq!(form.numbers[1].text.value(), "2100");
    }

    /// `Joy_Do_Repeat_Servo`'s boxes share one handler, which writes all four.
    #[test]
    fn the_repeat_forms_write_every_box() {
        let mut button = with(ButtonFunction::DoRepeatServo);
        button.p2 = 1600.0;
        let mut form = Form::open(FormKind::DoRepeatServo, 0, &mut button, Vec::new(), Vec::new());
        assert!(form.step(3, true, &mut button));
        assert!((button.p2 - 1600.0).abs() < f32::EPSILON);
        assert!((button.p4 - 1.0).abs() < f32::EPSILON);
        let mut relay = with(ButtonFunction::DoRepeatRelay);
        relay.p1 = 3.0;
        let mut form = Form::open(FormKind::DoRepeatRelay, 0, &mut relay, Vec::new(), Vec::new());
        assert!(!form.step(0, true, &mut relay), "relay 3 is the maximum");
        assert!(form.step(1, true, &mut relay));
        assert!((relay.p2 - 1.0).abs() < f32::EPSILON);
    }

    /// `Joy_ChangeMode` selects the button's mode by name; choosing another writes it.
    #[test]
    fn the_mode_form_selects_and_writes_the_mode() {
        let modes = vec![
            (0, "Stabilize".to_owned()),
            (5, "Loiter".to_owned()),
            (6, "RTL".to_owned()),
        ];
        let mut button = with(ButtonFunction::ChangeMode);
        button.mode = Some("loiter".to_owned());
        let mut form = Form::open(FormKind::ChangeMode, 1, &mut button, modes.clone(), Vec::new());
        assert_eq!(form.combo.as_ref().unwrap().text(), "Loiter");
        assert_eq!(button.mode.as_deref(), Some("Loiter"), "the selection moved, so written");
        assert!(form.choose(2, &mut button));
        assert_eq!(button.mode.as_deref(), Some("RTL"));
        // No mode yet: `Text = null` selects nothing, and that is written as an empty mode.
        let mut fresh = with(ButtonFunction::ChangeMode);
        let form = Form::open(FormKind::ChangeMode, 1, &mut fresh, modes, Vec::new());
        assert_eq!(form.combo.unwrap().selected, None);
        assert_eq!(fresh.mode.as_deref(), Some(""));
    }

    /// `Joy_Mount_Mode` selects `p1`'s mode and writes the chosen one's value.
    #[test]
    fn the_mount_form_writes_the_modes_value() {
        let options = vec![(0, "Retracted".to_owned()), (3, "RC Targeting".to_owned())];
        let mut button = with(ButtonFunction::MountMode);
        let mut form = Form::open(FormKind::MountMode, 0, &mut button, Vec::new(), options);
        assert_eq!(form.combo.as_ref().unwrap().selected, Some(0));
        assert!(form.choose(1, &mut button));
        assert!((button.p1 - 3.0).abs() < f32::EPSILON);
        let empty = Form::open(FormKind::MountMode, 0, &mut button, Vec::new(), Vec::new());
        assert!(empty.combo.unwrap().options.is_empty());
    }
}
