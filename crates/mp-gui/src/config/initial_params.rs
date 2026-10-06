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

//! Initial Tune Parameter: `GCSViews/ConfigurationView/ConfigInitialParams.cs`, a Mandatory
//! Hardware page of Initial Setup for a copter or a quadplane (`GCSViews/InitialSetup.cs:
//! 235-238`), listed once every parameter is in.
//!
//! What it shows: the page's instructions, four boxes - the airscrew size in inches, the battery's
//! cell count and a cell's charged and discharged voltages - the battery chemistry, "Using T-Motor
//! Flame ESC?", "Add suggested settings for 4.0 and up (Battery failsafe and Fence) ?", the
//! Calculate Initial Parameters button, and the link to ArduPilot's tuning instructions.
//! `Activate` puts 9 and 4 in the first two boxes, clears both check boxes and selects LiPo
//! (`ConfigInitialParams.cs:54-69`); choosing a chemistry writes its cell voltages into the
//! other two (`:245-273`) - which selecting LiPo when LiPo is already selected does not.
//!
//! Calculate reads the four boxes (`ConvertToDouble`: `double.TryParse`, and an exception for
//! text that does not parse, which nothing catches), refuses a prop size of zero or less and a
//! cell count below one, then works out the parameters as `calc_values` does (`:89-121`), with
//! the T-Motor's expo, and names them for the firmware: `ATC_` and `MOT_` on a copter, `Q_A_` and
//! `Q_M_` on a plane; `*_ACCEL_*_MAX` where the vehicle has them, else `*_ACC_*_MAX` in hundredths;
//! the 4.x filter names on version 4, else the 3.x ones; the T-Motor's PWM range; and on a copter
//! of 4.x, when asked, the battery failsafe and fence settings (`:124-228`). It opens `ParamCompare`
//! ([`super::param_compare`]) over the vehicle's table and those values, its button renamed "Write
//! to FC"; when that closes with `OK` - every ticked parameter written - it says so, with what to
//! do after the test flight (`:230-241`).
//!
//! The layout is the Designer's, every control at its `Location` in a 612 x 523 page.
//!
//! What is not carried over, and why:
//!
//! * typing into the instructions box and the chemistry combo: the C#'s `textBox1` is an editable
//!   `TextBox` whose text nothing reads, and `cmb_batterytype` a `DropDown` whose typed text
//!   raises no `SelectedIndexChanged`; both are drawn read-only - the combo's list still chooses;
//! * the text boxes' selection, caret placement and clipboard: this application's text field
//!   takes typing at its end.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;

use gpui::{AnyElement, Context, KeyDownEvent, Window, div, prelude::*, px, rgb};

use super::extra_setup::Focus;
use super::optional::{Event, SetQueue, at, button, label, message_box, text_box, value_of};
use super::param_compare::{self, ParamCompare};
use crate::MissionPlanner;
use crate::config::failsafe::CheckState;
use crate::config::flight_modes::Firmware;
use crate::config::servo_output::{Check, Combo, Message, check_box, combo_box, dropdown};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{panel, theme};

/// The page's title in Initial Setup's list, `backstageViewPageInitialParams.Text`.
/// `// C#: GCSViews/InitialSetup.resx:162-164`
pub const TITLE: &str = "Initial Tune Parameter";

/// `textBox1.Text`, the instructions.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.resx textBox1.Text`
pub const INSTRUCTIONS: &str = "You have to set some parameters based on battery and prop size \
for a new copter setup.\nPlease make sure that before entering data here and updating \
parameters:\n\n  -- ALL INITIAL SETUPS ARE DONE (Calibrations, frame settings, motor tests)\n  -- \
BATTERY VOLTAGE MONITORING IS SET AND WORKING\n\nNote: INS_GYRO_FILTER with a value other than 20 \
is optional and probably only for small frames/props. At first, you can keep it at 20";

/// `cb_tmotor.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.Designer.cs:105`
pub const TMOTOR: &str = "Using T-Motor Flame ESC?";
/// `cb_suggested.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.Designer.cs:175`
pub const SUGGESTED: &str = "Add suggested settings for 4.0 and up (Battery failsafe and Fence) ?";
/// `btn_docalc.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.Designer.cs:145`
pub const CALCULATE: &str = "Calculate Initial Parameters";
/// `label6.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.Designer.cs:195`
pub const SEE_HERE: &str =
    "You can find a detailed description of initial parameter settings and tuning here.";
/// `linkLabel1.Text`, and what its click opens.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.Designer.cs:185; ConfigInitialParams.cs:277`
pub const TUNING_URL: &str = "https://ardupilot.org/copter/docs/tuning-process-instructions.html";
/// `label7.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.Designer.cs:204`
pub const READ_IT: &str = "PLEASE READ IT !";
/// `label5.Text`.
pub const CHEMISTRY: &str = "Battery Chemistry";

/// The chemistries, `cmb_batterytype.Items`, keyed by index.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.Designer.cs:150-153`
pub const CHEMISTRIES: [&str; 3] = ["LiPo", "LiPoHV", "LiIon"];

/// The two refusals' caption.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.cs:136, 143`
pub const REFUSAL_TITLE: &str = "ERROR!";
/// A prop size of zero or less.
pub const PROP_TOO_SMALL: &str = "Prop size must be larger than zero.";
/// A cell count below one.
pub const TOO_FEW_CELLS: &str = "Battery cell count must be at least 1.";
/// `ParamCompare`'s button, renamed.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.cs:233-234`
pub const WRITE_TO_FC: &str = "Write to FC";
/// The box when `ParamCompare` closes with `OK`: its caption.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.cs:240`
pub const DONE_TITLE: &str = "Initial parameter calculator";
/// Its text, each `\r\n` a line.
pub const DONE: &str = "Initial Parameters succesfully updated.\nCheck parameters before flight!\n\
\nAfter test flight :\n\tSet ATC_THR_MIX_MAN to 0.5\n\tSet PSC_ACCZ_P/PSC_D_ACC_P to \
MOT_THST_HOVER\n\tSet PSC_ACCZ_I/PSC_D_ACC_I to 2*MOT_THST_HOVER\n\nHappy flying!";

/// `Program.handleException`'s box for `ConvertToDouble`'s exception, which nothing catches.
/// `// C#: ExtLibs/Utilities/Extensions.cs:600-615; Program.cs:793-795`
pub const BAD_NUMBER: &str =
    "An error has occurred\nSystem.Exception: Bad Type System.String\n\nReport this Error???";

/// The job `ParamCompare`'s button makes.
const SAVE_TAG: &str = "initialparams-save";

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.Designer.cs:228`
const PAGE_SIZE: (f32, f32) = (612.0, 523.0);

/// The four boxes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Field {
    /// `t_prop`.
    Prop,
    /// `t_cellcount`.
    CellCount,
    /// `t_cellmax`.
    CellMax,
    /// `t_cellmin`.
    CellMin,
}

impl Field {
    /// In the Designer's order.
    pub const ALL: [Self; 4] = [Self::Prop, Self::CellCount, Self::CellMax, Self::CellMin];

    /// Its index.
    const fn index(self) -> usize {
        match self {
            Self::Prop => 0,
            Self::CellCount => 1,
            Self::CellMax => 2,
            Self::CellMin => 3,
        }
    }

    /// Its control id.
    const fn id(self) -> &'static str {
        match self {
            Self::Prop => "initialparams-prop",
            Self::CellCount => "initialparams-cellcount",
            Self::CellMax => "initialparams-cellmax",
            Self::CellMin => "initialparams-cellmin",
        }
    }

    /// Its label's text and `Location`, and its own `Location`; each box is 100 x 20.
    /// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.Designer.cs:61-138`
    const fn places(self) -> (&'static str, (f32, f32), (f32, f32)) {
        match self {
            Self::Prop => ("Airscrew size in inch:", (99.0, 186.0), (210.0, 186.0)),
            Self::CellCount => ("Battery cellcount:", (115.0, 217.0), (210.0, 218.0)),
            Self::CellMax => (
                "Battery cell fully charged voltage:",
                (41.0, 248.0),
                (210.0, 248.0),
            ),
            Self::CellMin => (
                "Battery cell fully discharged voltage:",
                (28.0, 279.0),
                (210.0, 279.0),
            ),
        }
    }

    /// The Designer's text.
    /// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.Designer.cs:113, 121, 129, 137`
    const fn designer(self) -> &'static str {
        match self {
            Self::Prop => "12",
            Self::CellCount => "4",
            Self::CellMax => "4.2",
            Self::CellMin => "3.3",
        }
    }
}

/// What `calc_values` works out.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.cs:15-44, 89-121`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Values {
    /// `acro_yaw_p`.
    pub acro_yaw_p: f64,
    /// `atc_accel_p_max`, which `atc_accel_r_max` equals.
    pub atc_accel_p_max: f64,
    /// `atc_accel_y_max`.
    pub atc_accel_y_max: f64,
    /// `ins_gyro_filter`.
    pub ins_gyro_filter: f64,
    /// `atc_rat_pit_fltd`, which the roll's and the pitch's and roll's FLTT equal.
    pub atc_rat_fltd: f64,
    /// `atc_rat_yaw_fltt`.
    pub atc_rat_yaw_fltt: f64,
    /// `mot_thst_expo`.
    pub mot_thst_expo: f64,
    /// `batt_arm_volt`.
    pub batt_arm_volt: f64,
    /// `batt_crt_volt`.
    pub batt_crt_volt: f64,
    /// `batt_low_volt`.
    pub batt_low_volt: f64,
    /// `mot_bat_volt_max`.
    pub mot_bat_volt_max: f64,
    /// `mot_bat_volt_min`.
    pub mot_bat_volt_min: f64,
}

/// `atc_rat_pit_flte` and `atc_rat_rll_flte`.
const RAT_FLTE: f64 = 0.0;
/// `atc_rat_yaw_fltd`.
const YAW_FLTD: f64 = 0.0;
/// `atc_rat_yaw_flte`.
const YAW_FLTE: f64 = 2.0;
/// `atc_thr_mix_man`.
const THR_MIX_MAN: f64 = 0.1;
/// `ins_accel_filter`.
const ACCEL_FILTER: f64 = 10.0;
/// `mot_thst_hover`.
const THST_HOVER: f64 = 0.2;

/// `RoundTo(value, precision)`: `Math.Round`, a half to the even, for places; for a negative
/// precision, to that power of ten - half of it added, then the remainder taken off.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.cs:74-86`
#[must_use]
pub fn round_to(value: f64, precision: i32) -> f64 {
    if precision >= 0 {
        return round_places(value, precision);
    }
    // `(int)Math.Pow(10, Math.Abs(precision))`, and `5 * precision / 10` in integers.
    let power = 10_i64.pow(precision.unsigned_abs());
    #[allow(clippy::cast_precision_loss)] // small powers of ten
    let (half, power) = ((5 * power / 10) as f64, power as f64);
    let value = value + half;
    (value - value % power).round_ties_even()
}

/// `Math.Round(value, digits)`: scaled, rounded a half to the even, scaled back.
fn round_places(value: f64, digits: i32) -> f64 {
    let power = 10_f64.powi(digits);
    (value * power).round_ties_even() / power
}

/// `calc_values`.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.cs:89-121`
#[must_use]
pub fn calc_values(prop_size: f64, cells: f64, cell_max: f64, cell_min: f64) -> Values {
    let atc_accel_y_max = f64::max(8000.0, round_to(-900.0 * prop_size + 36000.0, -2));
    let acro_yaw_p = 0.5 * atc_accel_y_max / 4500.0;
    let atc_accel_p_max = f64::max(
        10000.0,
        round_to(
            -2.613_267 * prop_size.powi(3) + 343.392_16 * prop_size.powi(2)
                - 15083.7121 * prop_size
                + 235_771.0,
            -2,
        ),
    );
    let ins_gyro_filter = f64::max(20.0, (289.22 * prop_size.powf(-0.838)).round_ties_even());
    let half = f64::max(10.0, ins_gyro_filter / 2.0);
    let mot_thst_expo = f64::min(round_places(0.15686 * prop_size.ln() + 0.23693, 2), 0.80);
    Values {
        acro_yaw_p,
        atc_accel_p_max,
        atc_accel_y_max,
        ins_gyro_filter,
        atc_rat_fltd: half,
        atc_rat_yaw_fltt: half,
        mot_thst_expo,
        batt_arm_volt: (cells - 1.0) * 0.1 + (cell_min + 0.3) * cells,
        batt_crt_volt: (cell_min + 0.2) * cells,
        batt_low_volt: (cell_min + 0.3) * cells,
        mot_bat_volt_max: cell_max * cells,
        mot_bat_volt_min: cell_min * cells,
    }
}

/// What `btn_docalc_Click` does with the four boxes' texts.
#[derive(Debug, Clone, PartialEq)]
pub enum Calculated {
    /// The parameters to compare, in the order the handler adds them.
    Params(Vec<(String, f64)>),
    /// A box, and nothing more.
    Refused(Message),
}

/// The vehicle as the handler reads it.
#[derive(Debug, Clone, Copy)]
pub struct Target<'a> {
    /// `MAV.param`.
    pub parameters: &'a [(String, f64)],
    /// `cs.firmware`.
    pub firmware: Firmware,
    /// `cs.version.Major`.
    pub major: u8,
}

/// The two check boxes.
#[derive(Debug, Clone, Copy, Default)]
pub struct Options {
    /// `cb_tmotor.Checked`.
    pub tmotor: bool,
    /// `cb_suggested.Checked`.
    pub suggested: bool,
}

/// `btn_docalc_Click` up to `ParamCompare`: the boxes read, checked, and the parameters named.
/// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.cs:124-228`
#[must_use]
pub fn calculate(texts: [&str; 4], options: Options, target: Target<'_>) -> Calculated {
    let mut numbers = [0.0; 4];
    for (number, text) in numbers.iter_mut().zip(texts) {
        // `ConvertToDouble`: `double.TryParse`, or an exception nothing catches.
        match mp_log::netfmt::parse_double(text) {
            Some(value) => *number = value,
            None => {
                return Calculated::Refused(Message {
                    title: "Send Error",
                    text: BAD_NUMBER.to_owned(),
                });
            }
        }
    }
    let [prop_size, cells, cell_max, cell_min] = numbers;
    if prop_size <= 0.0 {
        return Calculated::Refused(Message {
            title: REFUSAL_TITLE,
            text: PROP_TOO_SMALL.to_owned(),
        });
    }
    if cells < 1.0 {
        return Calculated::Refused(Message {
            title: REFUSAL_TITLE,
            text: TOO_FEW_CELLS.to_owned(),
        });
    }
    let mut values = calc_values(prop_size, cells, cell_max, cell_min);
    if options.tmotor {
        values.mot_thst_expo = 0.2;
    }
    let plane = target.firmware == Firmware::ArduPlane;
    let (atc, mot) = if plane {
        ("Q_A", "Q_M")
    } else {
        ("ATC", "MOT")
    };
    let mut params: Vec<(String, f64)> = Vec::new();
    let mut add = |name: String, value: f64| params.push((name, value));
    add("ACRO_YAW_P".to_owned(), values.acro_yaw_p);
    if value_of(target.parameters, &format!("{atc}_ACCEL_P_MAX")).is_some() {
        add(format!("{atc}_ACCEL_P_MAX"), values.atc_accel_p_max);
        add(format!("{atc}_ACCEL_R_MAX"), values.atc_accel_p_max);
        add(format!("{atc}_ACCEL_Y_MAX"), values.atc_accel_y_max);
    } else {
        add(format!("{atc}_ACC_P_MAX"), values.atc_accel_p_max / 100.0);
        add(format!("{atc}_ACC_R_MAX"), values.atc_accel_p_max / 100.0);
        add(format!("{atc}_ACC_Y_MAX"), values.atc_accel_y_max / 100.0);
    }
    // "Filters has different name in 4.x and in 3.x"
    if target.major == 4 {
        add(format!("{atc}_RAT_PIT_FLTD"), values.atc_rat_fltd);
        add(format!("{atc}_RAT_PIT_FLTE"), RAT_FLTE);
        add(format!("{atc}_RAT_PIT_FLTT"), values.atc_rat_fltd);
        add(format!("{atc}_RAT_RLL_FLTD"), values.atc_rat_fltd);
        add(format!("{atc}_RAT_RLL_FLTE"), RAT_FLTE);
        add(format!("{atc}_RAT_RLL_FLTT"), values.atc_rat_fltd);
        add(format!("{atc}_RAT_YAW_FLTD"), YAW_FLTD);
        add(format!("{atc}_RAT_YAW_FLTE"), YAW_FLTE);
        add(format!("{atc}_RAT_YAW_FLTT"), values.atc_rat_yaw_fltt);
    } else {
        add(format!("{atc}_RAT_PIT_FILT"), values.atc_rat_fltd);
        add(format!("{atc}_RAT_RLL_FILT"), values.atc_rat_fltd);
        add(format!("{atc}_RAT_YAW_FILT"), YAW_FLTE);
    }
    add(format!("{atc}_THR_MIX_MAN"), THR_MIX_MAN);
    add("INS_ACCEL_FILTER".to_owned(), ACCEL_FILTER);
    add("INS_GYRO_FILTER".to_owned(), values.ins_gyro_filter);
    add(format!("{mot}_THST_EXPO"), values.mot_thst_expo);
    add(format!("{mot}_THST_HOVER"), THST_HOVER);
    add("BATT_ARM_VOLT".to_owned(), values.batt_arm_volt);
    add("BATT_CRT_VOLT".to_owned(), values.batt_crt_volt);
    add("BATT_LOW_VOLT".to_owned(), values.batt_low_volt);
    add(format!("{mot}_BAT_VOLT_MAX"), values.mot_bat_volt_max);
    add(format!("{mot}_BAT_VOLT_MIN"), values.mot_bat_volt_min);
    if options.tmotor {
        add(format!("{mot}_PWM_MIN"), 1100.0);
        add(format!("{mot}_PWM_MAX"), 1940.0);
    }
    if options.suggested && target.major == 4 && !plane {
        for (name, value) in [
            ("BATT_FS_CRT_ACT", 1.0),
            ("BATT_FS_LOW_ACT", 2.0),
            ("FENCE_ACTION", 3.0),
            ("FENCE_ALT_MAX", 120.0),
            ("FENCE_ENABLE", 1.0),
            ("FENCE_RADIUS", 150.0),
            ("FENCE_TYPE", 7.0),
        ] {
            add(name.to_owned(), value);
        }
    }
    Calculated::Params(params)
}

/// The page object.
#[derive(Debug)]
pub struct InitialParams<H = mp_link::RequestId> {
    made_for: Option<Key>,
    active: bool,
    /// The four boxes, in [`Field::ALL`]'s order.
    fields: [TextField; 4],
    /// `cb_tmotor.Checked`.
    tmotor: bool,
    /// `cb_suggested.Checked`.
    suggested: bool,
    /// `cmb_batterytype`: nothing selected until `Activate`.
    chemistry: Combo,
    dropdown: bool,
    /// The box being typed into.
    editing: Option<Field>,
    /// `ParamCompare`, while it is open.
    compare: Option<ParamCompare>,
    messages: VecDeque<Message>,
    queue: SetQueue<H>,
}

impl<H> Default for InitialParams<H> {
    /// `InitializeComponent`: the Designer's texts, the check boxes clear, no chemistry.
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            fields: Field::ALL.map(|field| {
                let mut text = TextField::new("");
                text.set(field.designer());
                text
            }),
            tmotor: false,
            suggested: false,
            chemistry: Combo {
                options: CHEMISTRIES
                    .iter()
                    .zip(0_i64..)
                    .map(|(name, index)| (index, (*name).to_owned()))
                    .collect(),
                enabled: true,
                ..Combo::default()
            },
            dropdown: false,
            editing: None,
            compare: None,
            messages: VecDeque::new(),
            queue: SetQueue::default(),
        }
    }
}

impl InitialParams {
    /// Once a frame: a page object whose screen has gone is let go, a box that lost the focus is
    /// left, and the writes move on.
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        on_setup: bool,
        focused: bool,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            self.made_for = None;
            self.compare = None;
        }
        if self.editing.is_some() && !focused {
            self.editing = None;
        }
        let events = self.queue.advance(telemetry, &mut self.messages);
        self.after(&events);
    }
}

impl<H: Copy> InitialParams<H> {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// A box's text.
    #[must_use]
    pub fn text(&self, field: Field) -> &str {
        self.fields.get(field.index()).map_or("", TextField::value)
    }

    /// `cb_tmotor.Checked`.
    #[must_use]
    pub const fn tmotor(&self) -> bool {
        self.tmotor
    }

    /// `cb_suggested.Checked`.
    #[must_use]
    pub const fn suggested(&self) -> bool {
        self.suggested
    }

    /// The chemistry combo.
    #[must_use]
    pub const fn chemistry(&self) -> &Combo {
        &self.chemistry
    }

    /// `ParamCompare`, while it is open.
    #[must_use]
    pub const fn compare(&self) -> Option<&ParamCompare> {
        self.compare.as_ref()
    }

    /// Whether `ParamCompare`'s writes are running.
    #[must_use]
    pub fn writing(&self) -> bool {
        self.queue.pending() > 0
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

    /// Shows the page: a new page object for a new screen, then `Activate`.
    /// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.cs:54-69`
    pub fn activate(&mut self, key: Key) {
        if self.made_for != Some(key) {
            let messages = std::mem::take(&mut self.messages);
            let queue = std::mem::take(&mut self.queue);
            *self = Self {
                made_for: Some(key),
                messages,
                queue,
                ..Self::default()
            };
        }
        self.active = true;
        self.dropdown = false;
        self.editing = None;
        // `prop.ToString()` and `cellcount.ToString()`.
        self.set_text(Field::Prop, "9");
        self.set_text(Field::CellCount, "4");
        self.tmotor = false;
        self.suggested = false;
        // `cmb_batterytype.SelectedIndex = 0`: the handler only when that changes the selection.
        if self.chemistry.select(0) {
            self.chemistry_changed();
        }
    }

    /// The page hidden. `ConfigInitialParams` is `IActivate` only.
    pub fn hide(&mut self) {
        self.active = false;
        self.dropdown = false;
        self.editing = None;
    }

    fn set_text(&mut self, field: Field, text: &str) {
        if let Some(box_) = self.fields.get_mut(field.index()) {
            box_.set(text);
        }
    }

    /// `cmb_batterytype_SelectedIndexChanged`: the chemistry's cell voltages, `ToString()` of
    /// the doubles.
    /// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.cs:245-273`
    fn chemistry_changed(&mut self) {
        let (max, min) = match self.chemistry.text() {
            "LiPoHV" => ("4.35", "3.3"),
            "LiIon" => ("4.1", "2.8"),
            // "LiPo", and the default.
            _ => ("4.2", "3.3"),
        };
        self.set_text(Field::CellMax, max);
        self.set_text(Field::CellMin, min);
    }

    /// Drops the chemistry's list down, or back up.
    pub fn toggle_dropdown(&mut self) {
        self.editing = None;
        self.dropdown = !self.dropdown;
        if self.dropdown {
            self.chemistry.open_list();
        }
    }

    /// The wheel over the list.
    pub fn scroll_list(&mut self, lines: i32) {
        if self.dropdown {
            self.chemistry.scroll_list(lines);
        }
    }

    /// A chemistry chosen: its handler when the selection changes.
    pub fn choose(&mut self, key: i64) {
        self.dropdown = false;
        if self.chemistry.select(key) {
            self.chemistry_changed();
        }
    }

    /// A check box clicked.
    pub fn click_tmotor(&mut self) {
        self.editing = None;
        self.tmotor = !self.tmotor;
    }

    /// The other.
    pub fn click_suggested(&mut self) {
        self.editing = None;
        self.suggested = !self.suggested;
    }

    /// A box clicked into.
    pub fn begin(&mut self, field: Field) {
        self.dropdown = false;
        self.editing = Some(field);
    }

    /// A key for the box being typed into.
    pub fn key(&mut self, event: &KeyDownEvent) -> bool {
        let Some(field) = self.editing else {
            return false;
        };
        let Some(box_) = self.fields.get_mut(field.index()) else {
            return false;
        };
        !matches!(box_.key(event), KeyOutcome::Ignored)
    }

    /// Calculate Initial Parameters: the refusal's box, or `ParamCompare` opened.
    /// `// C#: GCSViews/ConfigurationView/ConfigInitialParams.cs:124-236`
    pub fn click_calculate(&mut self, target: Target<'_>) {
        self.editing = None;
        self.dropdown = false;
        let texts = Field::ALL.map(|field| self.text(field).to_owned());
        let options = Options {
            tmotor: self.tmotor,
            suggested: self.suggested,
        };
        let texts = texts.each_ref().map(String::as_str);
        match calculate(texts, options, target) {
            Calculated::Refused(message) => self.messages.push_back(message),
            Calculated::Params(params) => {
                self.compare =
                    Some(ParamCompare::new(target.parameters, &params).with_save_text(WRITE_TO_FC));
            }
        }
    }

    /// `ParamCompare`'s button: its writes, while none are running.
    pub fn click_save(&mut self) {
        if self.writing() {
            return;
        }
        if let Some(form) = &self.compare {
            let job = form.save(SAVE_TAG);
            self.queue.push([job]);
        }
    }

    /// A row's Use box.
    pub fn toggle_row(&mut self, index: usize) {
        if let Some(form) = &mut self.compare {
            form.toggle_row(index);
        }
    }

    /// "Check/Uncheck All".
    pub fn toggle_all(&mut self) {
        if let Some(form) = &mut self.compare {
            form.click_toggle_all();
        }
    }

    /// The form closed by its close box: `DialogResult.Cancel`, nothing said.
    pub fn close_compare(&mut self) {
        if !self.writing() {
            self.compare = None;
        }
    }

    /// The writes' end: without a throw, the form closes with `OK` and the page says so; with
    /// one, the form stays open under its error box.
    /// `// C#: Controls/paramcompare.cs:84-109; GCSViews/ConfigurationView/ConfigInitialParams.cs:238-241`
    fn after(&mut self, events: &[Event]) {
        for event in events {
            if let Event::Done {
                tag: SAVE_TAG,
                threw,
            } = event
                && !threw
            {
                self.compare = None;
                self.messages.push_back(Message {
                    title: DONE_TITLE,
                    text: DONE.to_owned(),
                });
            }
        }
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(page: &InitialParams) {
    use crate::facts::record;
    record("config.initialparams.active", page.is_active());
    record("config.initialparams.prop", page.text(Field::Prop));
    record(
        "config.initialparams.cellcount",
        page.text(Field::CellCount),
    );
    record("config.initialparams.cellmax", page.text(Field::CellMax));
    record("config.initialparams.cellmin", page.text(Field::CellMin));
    record("config.initialparams.chemistry", page.chemistry().text());
    record("config.initialparams.tmotor", page.tmotor());
    record("config.initialparams.suggested", page.suggested());
    record(
        "config.initialparams.compare.open",
        page.compare().is_some(),
    );
    record(
        "config.initialparams.compare.rows",
        page.compare().map_or(0, |form| form.rows().len()),
    );
    record(
        "config.initialparams.compare.names",
        page.compare().map_or_else(String::new, |form| {
            form.rows()
                .iter()
                .map(|row| row.name.as_str())
                .collect::<Vec<_>>()
                .join(",")
        }),
    );
    if let Some(form) = page.compare() {
        for row in form.rows() {
            record(
                format!("config.initialparams.compare.{}", row.name),
                format!("{} -> {}", row.value, row.new_value),
            );
        }
    }
    record(
        "config.initialparams.write",
        page.queue.last().unwrap_or("none"),
    );
    record("config.initialparams.writes.pending", page.queue.pending());
    record(
        "config.initialparams.message",
        page.message()
            .map_or("none", |message| message.text.lines().next().unwrap_or("")),
    );
}

/// A plain `CheckBox` drawn with the `MavlinkCheckBox`'s look.
fn plain_check(checked: bool) -> Check {
    let mut check = Check::default();
    check.state = if checked {
        CheckState::Checked
    } else {
        CheckState::Unchecked
    };
    check.enabled = true;
    check
}

/// The page, laid out as the Designer lays it out.
pub fn page(
    params: &InitialParams,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if !params.is_active() {
        return div().into_any_element();
    }
    let typing = focus.text.is_focused(window);
    let mut body = div()
        .relative()
        .w(px(PAGE_SIZE.0))
        .h(px(PAGE_SIZE.1))
        .child(
            crate::probe::measured("initialparams-instructions", at(37.0, 32.0, 553.0, 137.0))
                .p_1()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::PANEL))
                .text_xs()
                .font_weight(gpui::FontWeight::BOLD)
                .text_color(rgb(theme::TEXT))
                .children(
                    INSTRUCTIONS
                        .split('\n')
                        .map(|line| div().min_h(px(12.0)).child(line.to_owned())),
                ),
        );
    for field in Field::ALL {
        let (text, (lx, ly), (x, y)) = field.places();
        body = body.child(label(lx, ly, text, true)).child(text_box(
            field.id(),
            params.text(field),
            Some(&focus.text),
            typing && params.editing == Some(field),
            true,
            (x, y, 100.0, 20.0),
            move |this| this.extra.initial_params.begin(field),
            |this, event| this.extra.initial_params.key(event),
            cx,
        ));
    }
    body = body
        .child(label(386.0, 221.0, CHEMISTRY, true))
        .child(combo_box(
            "initialparams-chemistry".to_owned(),
            params.chemistry(),
            (389.0, 244.0, 121.0, 21.0),
            |this| this.extra.initial_params.toggle_dropdown(),
            cx,
        ))
        .child(check_box(
            "initialparams-tmotor".to_owned(),
            &plain_check(params.tmotor()),
            TMOTOR,
            (210.0, 314.0),
            |this| this.extra.initial_params.click_tmotor(),
            cx,
        ))
        .child(check_box(
            "initialparams-suggested".to_owned(),
            &plain_check(params.suggested()),
            SUGGESTED,
            (210.0, 337.0),
            |this| this.extra.initial_params.click_suggested(),
            cx,
        ))
        .child(button(
            "initialparams-calculate",
            CALCULATE,
            (185.0, 388.0, 236.0, 23.0),
            true,
            |this, _window, _cx| {
                let view = this.telemetry.view();
                let banner = this.telemetry.firmware_banner().map(str::to_owned);
                let vehicle = crate::setup::Vehicle::of(&view, banner.as_deref());
                let target = Target {
                    parameters: &view.parameters,
                    firmware: vehicle.firmware,
                    major: vehicle.version.0,
                };
                this.extra.initial_params.click_calculate(target);
            },
            cx,
        ))
        .child(label(115.0, 447.0, SEE_HERE, true))
        .child(
            crate::probe::measured("initialparams-link", at(145.0, 469.0, 316.0, 13.0))
                .id("initialparams-link")
                .text_xs()
                .whitespace_nowrap()
                .text_color(rgb(theme::ACCENT))
                .underline()
                .cursor_pointer()
                .child(TUNING_URL)
                .on_click(|_event, _window, cx| cx.open_url(TUNING_URL)),
        )
        .child(label(253.0, 492.0, READ_IT, true));
    if params.dropdown {
        body = body.child(dropdown(
            "initialparams-chemistry",
            params.chemistry(),
            (389.0, 265.0, 121.0),
            |this, key| this.extra.initial_params.choose(key),
            |this, lines| this.extra.initial_params.scroll_list(lines),
            cx,
        ));
    }
    panel(TITLE, body).into_any_element()
}

/// `ParamCompare` or the message box showing, over the whole window: the box above the form, as
/// the C#'s error box shows over the dialog it came from.
pub fn overlay(
    params: &InitialParams,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(message) = params.message() {
        return Some(message_box(
            "initialparams-message",
            "initialparams-message-ok",
            message,
            window,
            |this| this.extra.initial_params.dismiss_message(),
            cx,
        ));
    }
    let form = params.compare()?;
    Some(param_compare::dialog(
        form,
        params.writing(),
        param_compare::Handlers {
            toggle_all: |this: &mut MissionPlanner| this.extra.initial_params.toggle_all(),
            toggle_row: |this: &mut MissionPlanner, index: usize| {
                this.extra.initial_params.toggle_row(index);
            },
            save: |this: &mut MissionPlanner| this.extra.initial_params.click_save(),
            close: |this: &mut MissionPlanner| this.extra.initial_params.close_compare(),
        },
        window,
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::tests::Answering;
    use mp_link::requests::RequestOutcome;

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    /// SITL's copter, every line of its parameter dump.
    fn sitl() -> Vec<(String, f64)> {
        include_str!("../../../../testdata/params/sitl-copter.param")
            .lines()
            .filter_map(|line| {
                let (name, value) = line.split_once(',')?;
                Some((name.to_owned(), value.trim().parse().ok()?))
            })
            .collect()
    }

    fn copter(parameters: &[(String, f64)]) -> Target<'_> {
        Target {
            parameters,
            firmware: Firmware::ArduCopter2,
            major: 4,
        }
    }

    fn params_of(calculated: Calculated) -> Vec<(String, f64)> {
        match calculated {
            Calculated::Params(params) => params,
            Calculated::Refused(message) => panic!("refused: {message:?}"),
        }
    }

    fn value(params: &[(String, f64)], name: &str) -> f64 {
        params
            .iter()
            .find(|(held, _)| held == name)
            .map(|(_, value)| *value)
            .unwrap_or_else(|| panic!("{name} is not in {params:?}"))
    }

    /// The Designer's words, read from the tree when it is here.
    #[test]
    fn the_text_is_the_designers() {
        let Some(designer) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigInitialParams.Designer.cs",
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        for text in [TMOTOR, SUGGESTED, CALCULATE, TUNING_URL, READ_IT, CHEMISTRY] {
            assert!(designer.contains(&format!(".Text = \"{text}\";")), "{text}");
        }
        for field in Field::ALL {
            let (text, (lx, ly), (x, y)) = field.places();
            assert!(designer.contains(&format!(".Text = \"{text}\";")), "{text}");
            assert!(
                designer.contains(&format!("new System.Drawing.Point({lx}, {ly});")),
                "{text}"
            );
            assert!(designer.contains(&format!("new System.Drawing.Point({x}, {y});")));
            assert!(designer.contains(&format!(".Text = \"{}\";", field.designer())));
        }
        for chemistry in CHEMISTRIES {
            assert!(designer.contains(&format!("\"{chemistry}\"")));
        }
        assert!(designer.contains("this.Size = new System.Drawing.Size(612, 523);"));
        let Some(resx) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigInitialParams.resx",
        ) else {
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        assert_eq!(
            values
                .get("textBox1.Text")
                .map(|text| text.replace("\r\n", "\n")),
            Some(INSTRUCTIONS.to_owned())
        );
        let Some(cs) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigInitialParams.cs",
        ) else {
            return;
        };
        let done = DONE.replace('\n', "\\r\\n").replace('\t', "\\t");
        assert!(cs.contains(&done), "the success box's text");
        for text in [PROP_TOO_SMALL, TOO_FEW_CELLS, DONE_TITLE, WRITE_TO_FC] {
            assert!(cs.contains(&format!("\"{text}\"")), "{text}");
        }
    }

    /// `Activate`: 9 and 4, both boxes clear, LiPo - whose handler writes 4.2 and 3.3.
    #[test]
    fn activate_sets_the_defaults_and_lipo() {
        let mut page = InitialParams::<usize>::default();
        assert_eq!(page.text(Field::Prop), "12", "the Designer's");
        page.activate(key());
        assert_eq!(page.text(Field::Prop), "9");
        assert_eq!(page.text(Field::CellCount), "4");
        assert_eq!(page.text(Field::CellMax), "4.2");
        assert_eq!(page.text(Field::CellMin), "3.3");
        assert_eq!(page.chemistry().text(), "LiPo");
    }

    /// Choosing a chemistry writes its voltages; selecting LiPo again on `Activate` when it is
    /// already selected does not, so voltages typed since stay.
    #[test]
    fn a_chemistry_writes_its_voltages_only_when_it_changes() {
        let mut page = InitialParams::<usize>::default();
        page.activate(key());
        page.choose(2);
        assert_eq!(page.text(Field::CellMax), "4.1");
        assert_eq!(page.text(Field::CellMin), "2.8");
        page.choose(1);
        assert_eq!(page.text(Field::CellMax), "4.35");
        page.choose(0);
        page.set_text(Field::CellMax, "4.15");
        page.hide();
        page.activate(key());
        assert_eq!(
            page.text(Field::CellMax),
            "4.15",
            "no change of selection, no handler"
        );
        page.choose(1);
        page.hide();
        page.activate(key());
        assert_eq!(
            page.text(Field::CellMax),
            "4.2",
            "back to LiPo: its handler"
        );
    }

    /// `RoundTo` to hundreds, and `calc_values` for the page's own 9-inch, four-cell LiPo.
    #[test]
    fn the_values_for_a_nine_inch_four_cell_lipo() {
        assert!((round_to(27_900.0, -2) - 27_900.0).abs() < 1e-9);
        assert!((round_to(125_927.284, -2) - 125_900.0).abs() < 1e-9);
        assert!((round_to(149.0, -2) - 100.0).abs() < 1e-9);
        assert!((round_to(150.0, -2) - 200.0).abs() < 1e-9);
        let values = calc_values(9.0, 4.0, 4.2, 3.3);
        assert!((values.atc_accel_y_max - 27_900.0).abs() < 1e-9);
        assert!((values.acro_yaw_p - 3.1).abs() < 1e-9);
        assert!((values.atc_accel_p_max - 125_900.0).abs() < 1e-9);
        assert!((values.ins_gyro_filter - 46.0).abs() < 1e-9);
        assert!((values.atc_rat_fltd - 23.0).abs() < 1e-9);
        assert!((values.mot_thst_expo - 0.58).abs() < 1e-12);
        assert_eq!(mp_log::netfmt::double(values.batt_arm_volt), "14.7");
        assert_eq!(mp_log::netfmt::double(values.batt_crt_volt), "14");
        assert_eq!(mp_log::netfmt::double(values.batt_low_volt), "14.4");
        assert_eq!(mp_log::netfmt::double(values.mot_bat_volt_max), "16.8");
        assert_eq!(mp_log::netfmt::double(values.mot_bat_volt_min), "13.2");
        // A small prop: the floors hold.
        let small = calc_values(3.0, 1.0, 4.2, 3.3);
        assert!((small.atc_accel_y_max - 33_300.0).abs() < 1e-9);
        assert!(small.ins_gyro_filter > 20.0);
        // A large one: the yaw acceleration floor, the expo's ceiling.
        let large = calc_values(40.0, 12.0, 4.2, 3.3);
        assert!((large.atc_accel_y_max - 8000.0).abs() < 1e-9);
        assert!((large.ins_gyro_filter - 20.0).abs() < 1e-9);
        assert!((large.atc_rat_fltd - 10.0).abs() < 1e-9);
        assert!(large.mot_thst_expo <= 0.8);
    }

    /// A 4.0 copter, which has `ATC_ACCEL_P_MAX`: the 4.x names, the page's order.
    #[test]
    fn a_4x_copter_gets_the_4x_names() {
        let four = vec![("ATC_ACCEL_P_MAX".to_owned(), 110_000.0)];
        let params = params_of(calculate(
            ["9", "4", "4.2", "3.3"],
            Options::default(),
            copter(&four),
        ));
        let names: Vec<&str> = params.iter().map(|(name, _)| name.as_str()).collect();
        assert_eq!(
            names,
            [
                "ACRO_YAW_P",
                "ATC_ACCEL_P_MAX",
                "ATC_ACCEL_R_MAX",
                "ATC_ACCEL_Y_MAX",
                "ATC_RAT_PIT_FLTD",
                "ATC_RAT_PIT_FLTE",
                "ATC_RAT_PIT_FLTT",
                "ATC_RAT_RLL_FLTD",
                "ATC_RAT_RLL_FLTE",
                "ATC_RAT_RLL_FLTT",
                "ATC_RAT_YAW_FLTD",
                "ATC_RAT_YAW_FLTE",
                "ATC_RAT_YAW_FLTT",
                "ATC_THR_MIX_MAN",
                "INS_ACCEL_FILTER",
                "INS_GYRO_FILTER",
                "MOT_THST_EXPO",
                "MOT_THST_HOVER",
                "BATT_ARM_VOLT",
                "BATT_CRT_VOLT",
                "BATT_LOW_VOLT",
                "MOT_BAT_VOLT_MAX",
                "MOT_BAT_VOLT_MIN",
            ]
        );
        assert!((value(&params, "ATC_ACCEL_R_MAX") - 125_900.0).abs() < 1e-9);
        assert!((value(&params, "ATC_RAT_YAW_FLTE") - 2.0).abs() < 1e-9);
        // SITL's 4.5 has `ATC_ACC_P_MAX` instead: the hundredths.
        let sitl = sitl();
        let params = params_of(calculate(
            ["9", "4", "4.2", "3.3"],
            Options::default(),
            copter(&sitl),
        ));
        assert!((value(&params, "ATC_ACC_P_MAX") - 1259.0).abs() < 1e-9);
    }

    /// Without `ATC_ACCEL_P_MAX` the newer names in hundredths; a 3.x copter's filters; the
    /// T-Motor's expo and PWM range; the suggestions only for a 4.x copter.
    #[test]
    fn the_names_follow_the_vehicle() {
        let none: Vec<(String, f64)> = Vec::new();
        let options = Options {
            tmotor: true,
            suggested: true,
        };
        let three = Target {
            parameters: &none,
            firmware: Firmware::ArduCopter2,
            major: 3,
        };
        let params = params_of(calculate(["9", "4", "4.2", "3.3"], options, three));
        assert!((value(&params, "ATC_ACC_P_MAX") - 1259.0).abs() < 1e-9);
        assert!((value(&params, "ATC_ACC_Y_MAX") - 279.0).abs() < 1e-9);
        assert!((value(&params, "ATC_RAT_PIT_FILT") - 23.0).abs() < 1e-9);
        assert!((value(&params, "ATC_RAT_YAW_FILT") - 2.0).abs() < 1e-9);
        assert!((value(&params, "MOT_THST_EXPO") - 0.2).abs() < 1e-12);
        assert!((value(&params, "MOT_PWM_MIN") - 1100.0).abs() < 1e-9);
        assert!((value(&params, "MOT_PWM_MAX") - 1940.0).abs() < 1e-9);
        assert!(
            !params.iter().any(|(name, _)| name == "FENCE_ENABLE"),
            "3.x"
        );

        let copter = params_of(calculate(["9", "4", "4.2", "3.3"], options, copter(&none)));
        assert!((value(&copter, "FENCE_TYPE") - 7.0).abs() < 1e-9);
        assert!((value(&copter, "BATT_FS_LOW_ACT") - 2.0).abs() < 1e-9);

        let plane = Target {
            parameters: &none,
            firmware: Firmware::ArduPlane,
            major: 4,
        };
        let quadplane = params_of(calculate(["9", "4", "4.2", "3.3"], options, plane));
        assert!(quadplane.iter().any(|(name, _)| name == "Q_A_RAT_PIT_FLTD"));
        assert!(quadplane.iter().any(|(name, _)| name == "Q_M_THST_HOVER"));
        assert!(
            !quadplane.iter().any(|(name, _)| name == "FENCE_ENABLE"),
            "a plane"
        );
    }

    /// The refusals, and text that does not parse - `ConvertToDouble`'s unhandled exception.
    #[test]
    fn the_refusals() {
        let none: Vec<(String, f64)> = Vec::new();
        let refused = |texts: [&str; 4]| match calculate(texts, Options::default(), copter(&none)) {
            Calculated::Refused(message) => message,
            Calculated::Params(_) => panic!("not refused"),
        };
        assert_eq!(refused(["0", "4", "4.2", "3.3"]).text, PROP_TOO_SMALL);
        assert_eq!(refused(["9", "0.5", "4.2", "3.3"]).text, TOO_FEW_CELLS);
        assert_eq!(refused(["9", "4", "four", "3.3"]).text, BAD_NUMBER);
        assert_eq!(refused(["", "4", "4.2", "3.3"]).text, BAD_NUMBER);
    }

    /// Calculate on SITL opens the form over what differs; Write to FC writes each ticked row
    /// and, when every one returned, closes the form and says so.
    #[test]
    fn calculate_compare_and_write() {
        let sitl = sitl();
        let mut page = InitialParams::<usize>::default();
        page.activate(key());
        page.click_calculate(copter(&sitl));
        let form = page.compare().expect("ParamCompare");
        assert_eq!(form.save_text(), WRITE_TO_FC);
        let names: Vec<&str> = form.rows().iter().map(|row| row.name.as_str()).collect();
        assert!(names.contains(&"INS_GYRO_FILTER"), "{names:?}");
        assert!(
            names
                .windows(2)
                .all(|pair| { mp_log::netfmt::culture_compare(pair[0], pair[1]).is_lt() })
        );
        let rows = form.rows().len();
        page.click_save();
        assert!(page.writing());
        let link = Answering::new(&[]);
        let mut messages = VecDeque::new();
        for _ in 0..100 {
            let events = page.queue.advance(&link, &mut messages);
            page.after(&events);
            if !page.writing() {
                break;
            }
        }
        page.messages.extend(messages);
        assert_eq!(link.taken().len(), rows);
        assert!(page.compare().is_none());
        assert_eq!(
            page.message().map(|message| message.title),
            Some(DONE_TITLE)
        );
    }

    /// A timeout keeps the form open under `Strings.ErrorSettingParameter`.
    #[test]
    fn a_timeout_keeps_the_form_open() {
        let sitl = sitl();
        let mut page = InitialParams::<usize>::default();
        page.activate(key());
        page.click_calculate(copter(&sitl));
        page.click_save();
        assert_eq!(
            page.compare()
                .and_then(|form| form.rows().first())
                .map(|row| row.name.as_str()),
            Some("ATC_ACC_P_MAX")
        );
        let link = Answering::new(&[(
            "ATC_ACC_P_MAX",
            Progress::Finished(RequestOutcome::TimedOut),
        )]);
        let mut messages = VecDeque::new();
        for _ in 0..100 {
            let events = page.queue.advance(&link, &mut messages);
            page.after(&events);
            if !page.writing() {
                break;
            }
        }
        assert!(page.compare().is_some());
        assert_eq!(
            messages.front().map(|message| message.text.as_str()),
            Some(crate::config::compass::ERROR_SETTING_PARAMETER)
        );
    }

    /// Every fact the GUI script asserts on is one this page records, and every control it
    /// clicks is one this page draws.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-initialparams.gui");
        let source = include_str!("initial_params.rs");
        let compare = include_str!("param_compare.rs");
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.initialparams.") => {
                    let generic = if key.starts_with("config.initialparams.compare.")
                        && !["open", "rows", "names"]
                            .iter()
                            .any(|tail| key.ends_with(tail))
                    {
                        "config.initialparams.compare.{}"
                    } else {
                        key
                    };
                    assert!(
                        source.contains(&format!("\"{generic}\"")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id))
                    if id.starts_with("initialparams-") || id.starts_with("paramcompare-") =>
                {
                    let base = id
                        .strip_prefix("paramcompare-use-")
                        .map_or(id, |_| "paramcompare-use-");
                    let base = if base.starts_with("initialparams-chemistry-") {
                        "initialparams-chemistry"
                    } else {
                        base
                    };
                    assert!(
                        source.contains(&format!("\"{base}"))
                            || compare.contains(&format!("\"{base}")),
                        "{id} is not drawn"
                    );
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 8 && clicks >= 3, "{facts} facts, {clicks} clicks");
    }
}
