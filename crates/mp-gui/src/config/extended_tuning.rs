//! Extended Tuning: `GCSViews/ConfigurationView/ConfigArducopter.cs`, the CONFIG screen's page for a
//! copter (`GCSViews/SoftwareConfig.cs:169`, "Extended Tuning") and for a plane
//! (`:182`, "QP Extended Tuning", which `Activate` enables only with `Q_ENABLE` set).
//!
//! What it shows, at `ConfigArducopter.resx`'s places in a 729 x 673 page: sixteen group boxes of
//! `MavlinkNumericUpDown`s - the stabilize, rate, position, velocity, throttle, waypoint, filter
//! and notch gains - the channel-6 tuning combo with its low and high numbers, the `RCn_OPTION`
//! combos for channels 6 to 10, "Lock Pitch and Roll Values", Write Params and Refresh Screen,
//! and the 4.7 unit-change warning under them.
//!
//! How it binds: `Activate` sets each box up with the names the C# tries for it, in order - the
//! first the vehicle has is the one bound, and a box with none is left disabled, named for the
//! first ([`NumberSpec::bind`], `Controls/MavlinkNumericUpDown.cs:49-122`). The three rate IMAX
//! boxes pick their names and scale from whether the 3.4 names are there (`:80-105`).
//!
//! How it writes: every box and seven of the nine combos have their `ValueUpdated` wired to the
//! page (`ConfigArducopter.Designer.cs`), so a change is not written by the control: it goes into
//! the page's `changes`, the box is marked green, and "Lock Pitch and Roll Values" copies a roll
//! change to pitch and back (`ConfigArducopter.cs:260-351`). Write Params - or ctrl+S - writes
//! `changes` one name at a time, asking first about a value more than double the vehicle's
//! (`:353-411`). `CH9_OPTION` and `CH10_OPTION` have no `ValueUpdated` and write their parameter
//! the moment they change, as any `MavlinkComboBox` does (`Controls/MavlinkComboBox.cs:169-199`).
//! No box here goes through `MavlinkNumericUpDown`'s 300 ms timer: the timer runs only when
//! nothing is subscribed to `ValueUpdated` (`MavlinkNumericUpDown.cs:147-160`), and on this page
//! everything is.
//!
//! Every `setParam` goes through the link's retrying set, one after another as the C#'s blocking
//! calls go ([`SetQueue`]); Refresh Screen's `GetParam`s through its retrying read.
//!
//! What is not ported, and why:
//!
//! * `BUT_rerequestparams` ("Refresh Params") and its handler: the `.resx` makes the button
//!   invisible and nothing shows it, so it cannot be pressed;
//! * `AddNewLinesForTooltip` and `disableNumericUpDownControls`: nothing calls them;
//! * the tool tips' twenty-second `AutoPopDelay`: gpui's tool tips stay while the pointer does.
//!
//! Divergences, each at its site: `changes` is a `Hashtable`, whose keys Write Params walks in
//! bucket order, and here in the order they were first changed; the writes do not hold the window
//! while they go; a paired box is typed into through its own typing path; ctrl+S reaches the page
//! while one of its boxes has the keyboard.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::time::{Duration, Instant};

use gpui::{
    AnyElement, App, Context, FocusHandle, FontWeight, KeyDownEvent, Keystroke, Modifiers, Render,
    SharedString, Window, div, prelude::*, px, rgb,
};

use super::optional::{
    Event, Job, Set, SetQueue, button, error, group, has, label, message_box, set_failed, value_of,
};
use crate::MissionPlanner;
use crate::config::failsafe::{CheckState, Lookup, options};
use crate::config::flight_modes::{Firmware, ParamWriter, Progress};
use crate::config::servo_output::{
    Check, Combo, Designer, Message, NUMERIC_DEFAULTS, Number, NumberHandlers, OUT_OF_RANGE_TITLE,
    Question, Setup, check_box, combo_box, dropdown, modal, number_box,
};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{action, theme};

// ---------------------------------------------------------------------------------------------
// The Designer's controls, at their `.resx` places.
// ---------------------------------------------------------------------------------------------

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.resx ($this.Size)`
pub const PAGE: (f32, f32) = (729.0, 673.0);

/// A group box: its Designer name, `Location` and `Size` on the page, and `Text`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroupSpec {
    /// The Designer name.
    pub name: &'static str,
    /// `Location` and `Size`.
    pub at: (f32, f32, f32, f32),
    /// `Text`.
    pub caption: &'static str,
}

const fn grp(name: &'static str, at: (f32, f32, f32, f32), caption: &'static str) -> GroupSpec {
    GroupSpec { name, at, caption }
}

/// The sixteen group boxes.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.resx (groupBoxN.Location, .Size, .Text)`
#[rustfmt::skip]
pub const GROUPS: &[GroupSpec] = &[
    grp("groupBox22", (7.0, 5.0, 170.0, 68.0), "Stabilize Roll (Error to Rate)"),
    grp("groupBox21", (183.0, 5.0, 170.0, 68.0), "Stabilize Pitch (Error to Rate)"),
    grp("groupBox20", (359.0, 5.0, 170.0, 68.0), "Stabilize Yaw (Error to Rate)"),
    grp("groupBox19", (532.0, 5.0, 170.0, 68.0), "Position XY (Dist to Speed)"),
    grp("groupBox25", (7.0, 100.0, 170.0, 180.0), "Rate Roll"),
    grp("groupBox24", (183.0, 100.0, 170.0, 180.0), "Rate Pitch"),
    grp("groupBox23", (359.0, 100.0, 170.0, 180.0), "Rate Yaw"),
    grp("groupBox1", (532.0, 100.0, 170.0, 108.0), "Velocity XY (Vel to Accel)"),
    grp("groupBox3", (532.0, 210.0, 170.0, 70.0), "Basic Filters"),
    grp("groupBox2", (7.0, 287.0, 170.0, 122.0), "Throttle Accel (Accel to motor)"),
    grp("groupBox5", (183.0, 287.0, 170.0, 50.0), "Throttle Rate (VSpd to accel)"),
    grp("groupBox7", (359.0, 287.0, 170.0, 50.0), "Altitude Hold (Alt to climbrate)"),
    grp("groupBox4", (532.0, 287.0, 170.0, 138.0), "WPNav (cm's)"),
    grp("groupBox6", (7.0, 415.0, 344.0, 48.0), "Filter Logs"),
    grp("groupBox8", (7.0, 469.0, 170.0, 133.0), "Static Notch Filter"),
    grp("groupBox9", (186.0, 469.0, 342.0, 133.0), "Harmonic Notch Filter"),
];

/// A label: its Designer name, the group box it is in (`None` for the page), `Location` there and
/// `Text`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LabelSpec {
    /// The Designer name.
    pub name: &'static str,
    /// Its parent group box.
    pub group: Option<&'static str>,
    /// `Location` in the parent.
    pub at: (f32, f32),
    /// `Text`.
    pub text: &'static str,
}

const fn lbl(
    name: &'static str,
    group: Option<&'static str>,
    at: (f32, f32),
    text: &'static str,
) -> LabelSpec {
    LabelSpec {
        name,
        group,
        at,
        text,
    }
}

/// Every label but `lblUnitWarning`.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.resx (labelN.Location, .Text, .Parent)`
pub const LABELS: &[LabelSpec] = &[
    lbl("label46", Some("groupBox22"), (6.0, 16.0), "P"),
    lbl("label19", Some("groupBox22"), (6.0, 42.0), "ACCEL MAX"),
    lbl("label42", Some("groupBox21"), (6.0, 16.0), "P"),
    lbl("label20", Some("groupBox21"), (6.0, 42.0), "ACCEL MAX"),
    lbl("label35", Some("groupBox20"), (6.0, 16.0), "P"),
    lbl("label21", Some("groupBox20"), (6.0, 42.0), "ACCEL MAX"),
    lbl("label31", Some("groupBox19"), (6.0, 16.0), "P"),
    lbl("label23", Some("groupBox19"), (6.0, 42.0), "INPUT TC"),
    lbl("label91", Some("groupBox25"), (6.0, 16.0), "P"),
    lbl("label90", Some("groupBox25"), (6.0, 40.0), "I"),
    lbl("label17", Some("groupBox25"), (6.0, 63.0), "D"),
    lbl("label88", Some("groupBox25"), (6.0, 86.0), "IMAX"),
    lbl("label12", Some("groupBox25"), (6.0, 108.0), "FLTE"),
    lbl("P_FLTD", Some("groupBox25"), (6.0, 131.0), "FLTD"),
    lbl("label28", Some("groupBox25"), (7.0, 154.0), "FLTT"),
    lbl("label87", Some("groupBox24"), (6.0, 16.0), "P"),
    lbl("label86", Some("groupBox24"), (6.0, 40.0), "I"),
    lbl("label11", Some("groupBox24"), (6.0, 63.0), "D"),
    lbl("label84", Some("groupBox24"), (6.0, 86.0), "IMAX"),
    lbl("label14", Some("groupBox24"), (6.0, 108.0), "FLTE"),
    lbl("label29", Some("groupBox24"), (6.0, 131.0), "FLTD"),
    lbl("label30", Some("groupBox24"), (5.0, 156.0), "FLTT"),
    lbl("label82", Some("groupBox23"), (6.0, 16.0), "P"),
    lbl("label77", Some("groupBox23"), (6.0, 40.0), "I"),
    lbl("label10", Some("groupBox23"), (6.0, 63.0), "D"),
    lbl("label47", Some("groupBox23"), (6.0, 86.0), "IMAX"),
    lbl("label18", Some("groupBox23"), (6.0, 108.0), "FLTE"),
    lbl("label32", Some("groupBox23"), (6.0, 131.0), "FLTD"),
    lbl("label33", Some("groupBox23"), (6.0, 154.0), "FLTT"),
    lbl("label4", Some("groupBox1"), (6.0, 16.0), "P"),
    lbl("label3", Some("groupBox1"), (6.0, 40.0), "I"),
    lbl("label1", Some("groupBox1"), (6.0, 63.0), "D"),
    lbl("label2", Some("groupBox1"), (6.0, 87.0), "IMAX"),
    lbl("label24", Some("groupBox3"), (8.0, 16.0), "Gyro"),
    lbl("label26", Some("groupBox3"), (8.0, 39.0), "Accel"),
    lbl("label8", Some("groupBox2"), (6.0, 22.0), "P"),
    lbl("label7", Some("groupBox2"), (6.0, 46.0), "I"),
    lbl("label5", Some("groupBox2"), (6.0, 69.0), "D"),
    lbl("label6", Some("groupBox2"), (6.0, 92.0), "IMAX"),
    lbl("label25", Some("groupBox5"), (6.0, 16.0), "P"),
    lbl("label22", Some("groupBox7"), (6.0, 16.0), "P"),
    lbl("label16", Some("groupBox4"), (6.0, 16.0), "Speed "),
    lbl("label15", Some("groupBox4"), (6.0, 40.0), "Radius"),
    lbl("label27", Some("groupBox4"), (6.0, 63.0), "Speed Up"),
    lbl("label13", Some("groupBox4"), (6.0, 87.0), "Speed Dn"),
    lbl("label9", Some("groupBox4"), (6.0, 110.0), "Loiter Speed"),
    lbl("label34", Some("groupBox6"), (6.0, 16.0), "Mask"),
    lbl("label36", Some("groupBox6"), (182.0, 16.0), "Options"),
    lbl("label37", Some("groupBox8"), (8.0, 29.0), "Enabled"),
    lbl("label38", Some("groupBox8"), (6.0, 54.0), "Frequency"),
    lbl("label39", Some("groupBox8"), (8.0, 79.0), "BandWidth"),
    lbl("label40", Some("groupBox8"), (8.0, 104.0), "Attenuation"),
    lbl("label45", Some("groupBox9"), (6.0, 29.0), "Enabled"),
    lbl("label44", Some("groupBox9"), (6.0, 54.0), "Mode"),
    lbl("label43", Some("groupBox9"), (6.0, 79.0), "Reference"),
    lbl("label41", Some("groupBox9"), (6.0, 104.0), "Frequency"),
    lbl("label48", Some("groupBox9"), (158.0, 29.0), "Attenuation"),
    lbl("label49", Some("groupBox9"), (158.0, 54.0), "Bandwidth"),
    lbl("label50", Some("groupBox9"), (158.0, 79.0), "Options"),
    lbl("label51", Some("groupBox9"), (158.0, 104.0), "Harmonics"),
    lbl("myLabel2", None, (183.0, 332.0), "Tune"),
    lbl("myLabel3", None, (183.0, 357.0), "Min"),
    lbl("label52", None, (357.0, 334.0), "RC6 Opt"),
    lbl("myLabel1", None, (357.0, 361.0), "RC7 Opt"),
    lbl("myLabel4", None, (357.0, 388.0), "RC8 Opt"),
    lbl("myLabel5", None, (357.0, 415.0), "RC9 Opt"),
    lbl("myLabel6", None, (357.0, 442.0), "RC10 Opt"),
];

/// `setup`'s first four arguments `Activate` passes: `(0, 0, 1, 0.001f)`, the gains.
const GAIN: Setup = Setup {
    minimum: 0.0,
    maximum: 0.0,
    scale: 1.0,
    increment: 0.001,
};
/// `(0, 0, 1, 0.0001f)`, the D gains.
const D_GAIN: Setup = Setup {
    increment: 0.0001,
    ..GAIN
};
/// `(0, 0, 1, 1f)`: whole numbers.
const WHOLE: Setup = Setup {
    increment: 1.0,
    ..GAIN
};
/// `(0, 0, 10, 1f)`: the IMAXes shown a tenth of the parameter.
const TENTHS: Setup = Setup {
    scale: 10.0,
    ..WHOLE
};
/// `(0, 10000, 1, 0.01f)`: `TUNE_LOW` and `TUNE_HIGH`.
const TUNE_BOUNDS: Setup = Setup {
    minimum: 0.0,
    maximum: 10000.0,
    scale: 1.0,
    increment: 0.01,
};

/// How `Activate` sets a box up.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Bind {
    /// `setup(Min, Max, Scale, Increment, new[] { names }, ...)`.
    Names(Setup, &'static [&'static str]),
    /// A rate IMAX: `if (param.ContainsKey(when[0]) || param.ContainsKey(when[1]))` - "3.4
    /// changes scaling" - one call, else the other.
    Imax {
        /// The names whose presence picks `then` - the names `then` is set up with.
        when: &'static [&'static str],
        /// The call when one is present.
        then: (Setup, &'static [&'static str]),
        /// The call otherwise.
        otherwise: (Setup, &'static [&'static str]),
    },
}

impl Bind {
    /// The call `Activate` makes against this parameter table.
    #[must_use]
    pub fn call(self, parameters: &[(String, f64)]) -> (Setup, &'static [&'static str]) {
        match self {
            Self::Names(how, names) => (how, names),
            Self::Imax {
                when,
                then,
                otherwise,
            } => {
                if when.iter().any(|name| has(parameters, name)) {
                    then
                } else {
                    otherwise
                }
            }
        }
    }
}

/// A `MavlinkNumericUpDown`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NumberSpec {
    /// The Designer name, the control's `Name` until `setup` renames it to its parameter.
    pub name: &'static str,
    /// Its parent group box; `None` for the page.
    pub group: Option<&'static str>,
    /// `Location` in the parent, and `Size`.
    pub at: (f32, f32, f32, f32),
    /// The line of its `setup` call in `ConfigArducopter.cs`.
    pub line: u32,
    /// The call.
    pub bind: Bind,
}

const fn num(
    name: &'static str,
    group: Option<&'static str>,
    at: (f32, f32, f32, f32),
    line: u32,
    how: Setup,
    names: &'static [&'static str],
) -> NumberSpec {
    NumberSpec {
        name,
        group,
        at,
        line,
        bind: Bind::Names(how, names),
    }
}

/// A rate IMAX box.
const fn imax(
    name: &'static str,
    group: &'static str,
    at: (f32, f32, f32, f32),
    line: u32,
    when: &'static [&'static str],
    otherwise: &'static [&'static str],
) -> NumberSpec {
    NumberSpec {
        name,
        group: Some(group),
        at,
        line,
        bind: Bind::Imax {
            when,
            then: (WHOLE, when),
            otherwise: (TENTHS, otherwise),
        },
    }
}

/// A number's `Size`: 78 x 20, as all but three are.
const fn box78(x: f32, y: f32) -> (f32, f32, f32, f32) {
    (x, y, 78.0, 20.0)
}

/// The 59 boxes, in `Activate`'s order, with the names each is set up with in the order tried.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:60-166; ConfigArducopter.resx`
#[rustfmt::skip]
pub const NUMBERS: &[NumberSpec] = &[
    num("TUNE_LOW", None, (218.0, 360.0, 51.0, 20.0), 60, TUNE_BOUNDS, &["TUNE_LOW", "TUNE_MIN"]),
    num("TUNE_HIGH", None, (308.0, 360.0, 46.0, 20.0), 62, TUNE_BOUNDS, &["TUNE_HIGH", "TUNE_MAX"]),
    num("HLD_LAT_P", Some("groupBox19"), box78(80.0, 13.0), 65, GAIN, &["HLD_LAT_P", "POS_XY_P", "PSC_POSXY_P", "Q_P_POSXY_P"]),
    num("LOITER_LAT_D", Some("groupBox1"), box78(80.0, 60.0), 67, GAIN, &["LOITER_LAT_D", "PSC_VELXY_D", "Q_P_VELXY_D", "PSC_NE_VEL_D", "Q_P_NE_VEL_D"]),
    num("LOITER_LAT_I", Some("groupBox1"), box78(80.0, 37.0), 69, GAIN, &["LOITER_LAT_I", "VEL_XY_I", "PSC_VELXY_I", "Q_P_VELXY_I", "PSC_NE_VEL_I", "Q_P_NE_VEL_I"]),
    num("LOITER_LAT_IMAX", Some("groupBox1"), box78(80.0, 84.0), 71, TENTHS, &["LOITER_LAT_IMAX", "VEL_XY_IMAX", "PSC_VELXY_IMAX", "Q_P_VELXY_IMAX", "PSC_NE_VEL_IMAX", "Q_P_NE_VEL_IMAX"]),
    num("LOITER_LAT_P", Some("groupBox1"), box78(80.0, 13.0), 74, GAIN, &["LOITER_LAT_P", "VEL_XY_P", "PSC_VELXY_P", "Q_P_VELXY_P", "PSC_NE_VEL_P", "Q_P_NE_VEL_P"]),
    num("RATE_PIT_P", Some("groupBox24"), box78(80.0, 13.0), 77, GAIN, &["RATE_PIT_P", "ATC_RAT_PIT_P", "Q_A_RAT_PIT_P"]),
    num("RATE_PIT_I", Some("groupBox24"), box78(80.0, 37.0), 78, GAIN, &["RATE_PIT_I", "ATC_RAT_PIT_I", "Q_A_RAT_PIT_I"]),
    num("RATE_PIT_D", Some("groupBox24"), box78(80.0, 60.0), 79, D_GAIN, &["RATE_PIT_D", "ATC_RAT_PIT_D", "Q_A_RAT_PIT_D"]),
    imax("RATE_PIT_IMAX", "groupBox24", box78(80.0, 84.0), 80, &["ATC_RAT_PIT_IMAX", "Q_A_RAT_PIT_IMAX"], &["RATE_PIT_IMAX", "RATE_PIT_IMAX"]),
    num("RATE_PIT_FILT", Some("groupBox24"), box78(80.0, 106.0), 84, GAIN, &["RATE_PIT_FILT", "ATC_RAT_PIT_FILT", "ATC_RAT_PIT_FLTE", "Q_A_RAT_PIT_FLTE"]),
    num("ATC_RAT_PIT_FLTD", Some("groupBox24"), box78(80.0, 129.0), 85, WHOLE, &["ATC_RAT_PIT_FLTD", "Q_A_RAT_PIT_FLTD"]),
    num("ATC_RAT_PIT_FLTT", Some("groupBox24"), box78(80.0, 152.0), 86, WHOLE, &["ATC_RAT_PIT_FLTT", "Q_A_RAT_PIT_FLTT"]),
    num("RATE_RLL_P", Some("groupBox25"), box78(80.0, 13.0), 88, GAIN, &["RATE_RLL_P", "ATC_RAT_RLL_P", "Q_A_RAT_RLL_P"]),
    num("RATE_RLL_I", Some("groupBox25"), box78(80.0, 37.0), 89, GAIN, &["RATE_RLL_I", "ATC_RAT_RLL_I", "Q_A_RAT_RLL_I"]),
    num("RATE_RLL_D", Some("groupBox25"), box78(80.0, 60.0), 90, D_GAIN, &["RATE_RLL_D", "ATC_RAT_RLL_D", "Q_A_RAT_RLL_D"]),
    imax("RATE_RLL_IMAX", "groupBox25", box78(80.0, 83.0), 91, &["ATC_RAT_RLL_IMAX", "Q_A_RAT_RLL_IMAX"], &["RATE_RLL_IMAX"]),
    num("RATE_RLL_FILT", Some("groupBox25"), box78(80.0, 106.0), 95, GAIN, &["RATE_RLL_FILT", "ATC_RAT_RLL_FILT", "ATC_RAT_RLL_FLTE", "Q_A_RAT_RLL_FLTE"]),
    num("ATC_RAT_RLL_FLTD", Some("groupBox25"), box78(80.0, 129.0), 96, WHOLE, &["ATC_RAT_RLL_FLTD", "Q_A_RAT_RLL_FLTD"]),
    num("ATC_RAT_RLL_FLTT", Some("groupBox25"), box78(80.0, 152.0), 97, WHOLE, &["ATC_RAT_RLL_FLTT", "Q_A_RAT_RLL_FLTT"]),
    num("RATE_YAW_P", Some("groupBox23"), box78(80.0, 13.0), 99, GAIN, &["RATE_YAW_P", "ATC_RAT_YAW_P", "Q_A_RAT_YAW_P"]),
    num("RATE_YAW_I", Some("groupBox23"), box78(80.0, 37.0), 100, GAIN, &["RATE_YAW_I", "ATC_RAT_YAW_I", "Q_A_RAT_YAW_I"]),
    num("RATE_YAW_D", Some("groupBox23"), box78(80.0, 60.0), 101, D_GAIN, &["RATE_YAW_D", "ATC_RAT_YAW_D", "Q_A_RAT_YAW_D"]),
    imax("RATE_YAW_IMAX", "groupBox23", box78(80.0, 83.0), 102, &["ATC_RAT_YAW_IMAX", "Q_A_RAT_YAW_IMAX"], &["RATE_YAW_IMAX"]),
    num("RATE_YAW_FILT", Some("groupBox23"), box78(80.0, 106.0), 106, GAIN, &["RATE_YAW_FILT", "ATC_RAT_YAW_FILT", "ATC_RAT_YAW_FLTE", "Q_A_RAT_YAW_FLTE"]),
    num("ATC_RAT_YAW_FLTD", Some("groupBox23"), box78(79.0, 129.0), 107, WHOLE, &["ATC_RAT_YAW_FLTD", "Q_A_RAT_YAW_FLTD"]),
    num("ATC_RAT_YAW_FLTT", Some("groupBox23"), box78(79.0, 152.0), 108, WHOLE, &["ATC_RAT_YAW_FLTT", "Q_A_RAT_YAW_FLTT"]),
    num("STB_PIT_P", Some("groupBox21"), box78(80.0, 13.0), 110, GAIN, &["STB_PIT_P", "ATC_ANG_PIT_P", "Q_A_ANG_PIT_P"]),
    num("STB_RLL_P", Some("groupBox22"), box78(80.0, 13.0), 112, GAIN, &["STB_RLL_P", "ATC_ANG_RLL_P", "Q_A_ANG_RLL_P"]),
    num("STB_YAW_P", Some("groupBox20"), box78(80.0, 13.0), 114, GAIN, &["STB_YAW_P", "ATC_ANG_YAW_P", "Q_A_ANG_YAW_P"]),
    num("THR_ACCEL_P", Some("groupBox2"), box78(80.0, 19.0), 118, GAIN, &["THR_ACCEL_P", "ACCEL_Z_P", "PSC_ACCZ_P", "Q_P_ACCZ_P", "PSC_D_ACC_P", "Q_P_D_ACC_P"]),
    num("THR_ACCEL_I", Some("groupBox2"), box78(80.0, 43.0), 120, GAIN, &["THR_ACCEL_I", "ACCEL_Z_I", "PSC_ACCZ_I", "Q_P_ACCZ_I", "PSC_D_ACC_I", "Q_P_D_ACC_I"]),
    num("THR_ACCEL_D", Some("groupBox2"), box78(80.0, 66.0), 122, GAIN, &["THR_ACCEL_D", "ACCEL_Z_D", "PSC_ACCZ_D", "Q_P_ACCZ_D", "PSC_D_ACC_D", "Q_P_D_ACC_D"]),
    num("THR_ACCEL_IMAX", Some("groupBox2"), box78(80.0, 89.0), 124, TENTHS, &["THR_ACCEL_IMAX", "ACCEL_Z_IMAX", "PSC_ACCZ_IMAX", "Q_P_ACCZ_IMAX", "PSC_D_ACC_IMAX", "Q_P_D_ACC_IMAX"]),
    num("THR_ALT_P", Some("groupBox7"), box78(80.0, 13.0), 127, GAIN, &["THR_ALT_P", "POS_Z_P", "PSC_POSZ_P", "Q_P_POSZ_P", "PSC_D_POS_P", "Q_P_D_POS_P"]),
    num("THR_RATE_P", Some("groupBox5"), box78(80.0, 13.0), 129, GAIN, &["THR_RATE_P", "VEL_Z_P", "PSC_VELZ_P", "Q_P_VELZ_P", "PSC_D_VEL_P", "Q_P_D_VEL_P"]),
    num("WPNAV_LOIT_SPEED", Some("groupBox4"), box78(80.0, 107.0), 132, GAIN, &["WPNAV_LOIT_SPEED", "LOIT_SPEED", "Q_LOIT_SPEED", "LOIT_SPEED_MS", "Q_LOIT_SPEED_MS"]),
    num("WPNAV_RADIUS", Some("groupBox4"), box78(80.0, 37.0), 134, GAIN, &["WPNAV_RADIUS", "Q_WP_RADIUS", "WP_RADIUS_M", "Q_WP_RADIUS_M"]),
    num("WPNAV_SPEED", Some("groupBox4"), box78(80.0, 13.0), 135, GAIN, &["WPNAV_SPEED", "Q_WP_SPEED", "WP_SPD", "Q_WP_SPD"]),
    num("WPNAV_SPEED_DN", Some("groupBox4"), box78(80.0, 84.0), 136, GAIN, &["WPNAV_SPEED_DN", "Q_WP_SPEED_DN", "WP_SPD_DN", "Q_WP_SPD_DN"]),
    num("WPNAV_SPEED_UP", Some("groupBox4"), box78(80.0, 60.0), 137, GAIN, &["WPNAV_SPEED_UP", "Q_WP_SPEED_UP", "WP_SPD_UP", "Q_WP_SPD_UP"]),
    num("INS_GYRO_FILTER", Some("groupBox3"), (79.0, 14.0, 79.0, 20.0), 139, WHOLE, &["INS_GYRO_FILTER"]),
    num("INS_ACCEL_FILTER", Some("groupBox3"), box78(80.0, 37.0), 140, WHOLE, &["INS_ACCEL_FILTER"]),
    num("INS_LOG_BAT_OPT", Some("groupBox6"), box78(256.0, 14.0), 143, WHOLE, &["INS_LOG_BAT_OPT"]),
    num("INS_NOTCH_FREQ", Some("groupBox8"), box78(80.0, 52.0), 146, WHOLE, &["INS_NOTCH_FREQ"]),
    num("INS_NOTCH_BW", Some("groupBox8"), box78(80.0, 77.0), 147, WHOLE, &["INS_NOTCH_BW"]),
    num("INS_NOTCH_ATT", Some("groupBox8"), box78(80.0, 102.0), 148, WHOLE, &["INS_NOTCH_ATT"]),
    num("INS_HNTCH_MODE", Some("groupBox9"), box78(64.0, 52.0), 151, WHOLE, &["INS_HNTCH_MODE"]),
    num("INS_HNTCH_REF", Some("groupBox9"), box78(64.0, 77.0), 152, WHOLE, &["INS_HNTCH_REF"]),
    num("INS_HNTCH_FREQ", Some("groupBox9"), box78(64.0, 102.0), 153, WHOLE, &["INS_HNTCH_FREQ"]),
    num("INS_HNTCH_ATT", Some("groupBox9"), box78(229.0, 27.0), 154, WHOLE, &["INS_HNTCH_ATT"]),
    num("INS_HNTCH_BW", Some("groupBox9"), box78(229.0, 52.0), 155, WHOLE, &["INS_HNTCH_BW"]),
    num("INS_HNTCH_OPTS", Some("groupBox9"), box78(230.0, 77.0), 156, WHOLE, &["INS_HNTCH_OPTS"]),
    num("INS_HNTCH_HMNCS", Some("groupBox9"), box78(230.0, 102.0), 157, WHOLE, &["INS_HNTCH_HMNCS"]),
    num("mavlinkNumericUpDownatc_accel_r_max", Some("groupBox22"), box78(80.0, 39.0), 159, GAIN, &["ATC_ACCEL_R_MAX", "Q_A_ACCEL_R_MAX", "ATC_ACC_R_MAX", "Q_A_ACC_R_MAX"]),
    num("mavlinkNumericUpDownatc_accel_p_max", Some("groupBox21"), box78(80.0, 39.0), 161, GAIN, &["ATC_ACCEL_P_MAX", "Q_A_ACCEL_P_MAX", "ATC_ACC_P_MAX", "Q_A_ACC_P_MAX"]),
    num("mavlinkNumericUpDownatc_accel_y_max", Some("groupBox20"), box78(80.0, 39.0), 163, GAIN, &["ATC_ACCEL_Y_MAX", "Q_A_ACCEL_Y_MAX", "ATC_ACC_Y_MAX", "Q_A_ACC_Y_MAX"]),
    num("mavlinkNumericUpDownatc_input_tc", Some("groupBox19"), box78(80.0, 39.0), 165, GAIN, &["ATC_INPUT_TC", "Q_A_INPUT_TC"]),
];

/// The boxes in the order the page's `Controls` hold them - the Designer's `Controls.Add` order,
/// each group box's own in turn - which is the order Refresh Screen reads them in.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:457-477; ConfigArducopter.Designer.cs`
#[rustfmt::skip]
pub const CONTROLS_ORDER: [&str; 59] = [
    "INS_NOTCH_ATT", "INS_NOTCH_BW", "INS_NOTCH_FREQ",
    "INS_HNTCH_HMNCS", "INS_HNTCH_OPTS", "INS_HNTCH_BW", "INS_HNTCH_ATT", "INS_HNTCH_FREQ",
    "INS_HNTCH_REF", "INS_HNTCH_MODE",
    "INS_LOG_BAT_OPT",
    "INS_ACCEL_FILTER", "INS_GYRO_FILTER",
    "THR_ACCEL_D", "THR_ACCEL_IMAX", "THR_ACCEL_I", "THR_ACCEL_P",
    "LOITER_LAT_D", "LOITER_LAT_IMAX", "LOITER_LAT_I", "LOITER_LAT_P",
    "TUNE_LOW", "TUNE_HIGH",
    "THR_RATE_P",
    "WPNAV_SPEED_UP", "WPNAV_LOIT_SPEED", "WPNAV_SPEED_DN", "WPNAV_RADIUS", "WPNAV_SPEED",
    "THR_ALT_P",
    "mavlinkNumericUpDownatc_input_tc", "HLD_LAT_P",
    "mavlinkNumericUpDownatc_accel_y_max", "STB_YAW_P",
    "mavlinkNumericUpDownatc_accel_p_max", "STB_PIT_P",
    "mavlinkNumericUpDownatc_accel_r_max", "STB_RLL_P",
    "ATC_RAT_YAW_FLTT", "ATC_RAT_YAW_FLTD", "RATE_YAW_FILT", "RATE_YAW_D", "RATE_YAW_IMAX",
    "RATE_YAW_I", "RATE_YAW_P",
    "ATC_RAT_PIT_FLTT", "ATC_RAT_PIT_FLTD", "RATE_PIT_FILT", "RATE_PIT_D", "RATE_PIT_IMAX",
    "RATE_PIT_I", "RATE_PIT_P",
    "ATC_RAT_RLL_FLTT", "ATC_RAT_RLL_FLTD", "RATE_RLL_FILT", "RATE_RLL_D", "RATE_RLL_IMAX",
    "RATE_RLL_I", "RATE_RLL_P",
];

/// A number as the Designer leaves it: `NumericUpDown`'s defaults, and `THR_ACCEL_IMAX`'s
/// `Maximum = 1000`.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.Designer.cs (THR_ACCEL_IMAX.Maximum)`
fn designer_of(name: &str) -> Designer {
    if name == "THR_ACCEL_IMAX" {
        Designer {
            maximum: 1000.0,
            ..NUMERIC_DEFAULTS
        }
    } else {
        NUMERIC_DEFAULTS
    }
}

/// How `Activate` sets a combo up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComboBind {
    /// `setup(ParameterMetaDataRepository.GetParameterOptionsInt(name, ...).ToList(), name,
    /// param)`: the list bound whether or not the vehicle has the parameter.
    Options(&'static str),
    /// `setup(new[] { names }, param)`: the first name the vehicle has, or nothing at all.
    Names(&'static [&'static str]),
}

/// A `MavlinkComboBox`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ComboSpec {
    /// The Designer name.
    pub name: &'static str,
    /// Its parent group box; `None` for the page.
    pub group: Option<&'static str>,
    /// `Location` in the parent, and `Size`.
    pub at: (f32, f32, f32, f32),
    /// The line of its `setup` call.
    pub line: u32,
    /// The call.
    pub bind: ComboBind,
    /// Whether the Designer wires its `ValueUpdated` to `numeric_ValueUpdated`: a change then
    /// goes into `changes`; without it the control writes the parameter itself.
    pub value_updated: bool,
}

const fn cmb(
    name: &'static str,
    group: Option<&'static str>,
    at: (f32, f32, f32, f32),
    line: u32,
    bind: ComboBind,
    value_updated: bool,
) -> ComboSpec {
    ComboSpec {
        name,
        group,
        at,
        line,
        bind,
        value_updated,
    }
}

/// The nine combos. Every one's `DropDownWidth` is 170.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:49-58, 142, 145, 150;
/// ConfigArducopter.Designer.cs (CHn_OPTION, TUNE, INS_*.ValueUpdated); ConfigArducopter.resx`
#[rustfmt::skip]
pub const COMBOS: &[ComboSpec] = &[
    cmb("TUNE", None, (242.0, 332.0, 112.0, 21.0), 49, ComboBind::Options("TUNE"), true),
    cmb("CH6_OPTION", None, (416.0, 334.0, 112.0, 21.0), 54, ComboBind::Names(&["CH6_OPT", "CH6_OPTION", "RC6_OPTION", "RC6_OPTION"]), true),
    cmb("CH7_OPTION", None, (416.0, 361.0, 112.0, 21.0), 55, ComboBind::Names(&["CH7_OPT", "CH7_OPTION", "RC7_OPTION", "RC7_OPTION"]), true),
    cmb("CH8_OPTION", None, (416.0, 388.0, 112.0, 21.0), 56, ComboBind::Names(&["CH8_OPT", "CH8_OPTION", "RC8_OPTION", "RC8_OPTION"]), true),
    cmb("CH9_OPTION", None, (416.0, 415.0, 112.0, 21.0), 57, ComboBind::Names(&["CH9_OPT", "CH9_OPTION", "RC9_OPTION", "RC9_OPTION"]), false),
    cmb("CH10_OPTION", None, (416.0, 442.0, 112.0, 21.0), 58, ComboBind::Names(&["CH10_OPT", "CH10_OPTION", "RC10_OPTION", "RC10_OPTION"]), false),
    cmb("INS_LOG_BAT_MASK", Some("groupBox6"), (80.0, 13.0, 78.0, 21.0), 142, ComboBind::Names(&["INS_LOG_BAT_MASK"]), true),
    cmb("INS_NOTCH_ENABLE", Some("groupBox8"), (80.0, 26.0, 78.0, 21.0), 145, ComboBind::Names(&["INS_NOTCH_ENABLE"]), true),
    cmb("INS_HNTCH_ENABLE", Some("groupBox9"), (64.0, 26.0, 78.0, 21.0), 150, ComboBind::Names(&["INS_HNTCH_ENABLE"]), true),
];

/// `DropDownWidth`.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.Designer.cs (*.DropDownWidth = 170)`
const DROP_DOWN_WIDTH: f32 = 170.0;

/// A `MyButton`: its Designer name, `Location` and `Size`, `Text`, and whether it is `Visible`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ButtonSpec {
    /// The Designer name.
    pub name: &'static str,
    /// `Location` and `Size`.
    pub at: (f32, f32, f32, f32),
    /// `Text`.
    pub text: &'static str,
    /// `Visible`.
    pub visible: bool,
}

/// The three buttons. "Refresh Params" is `Visible = False` in the `.resx` and nothing shows it.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.resx (BUT_*.Location, .Size, .Text, .Visible)`
pub const BUTTONS: [ButtonSpec; 3] = [
    ButtonSpec {
        name: "BUT_writePIDS",
        at: (208.0, 608.0, 103.0, 25.0),
        text: "Write Params",
        visible: true,
    },
    ButtonSpec {
        name: "BUT_rerequestparams",
        at: (317.0, 608.0, 103.0, 25.0),
        text: "Refresh Params",
        visible: false,
    },
    ButtonSpec {
        name: "BUT_refreshpart",
        at: (426.0, 608.0, 103.0, 25.0),
        text: "Refresh Screen",
        visible: true,
    },
];

/// `CHK_lockrollpitch`: `Location`, `Size` and `Text`; `Checked = true` in the Designer.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.resx (CHK_lockrollpitch.*);
/// ConfigArducopter.Designer.cs (CHK_lockrollpitch.Checked)`
pub const LOCK_AT: (f32, f32, f32, f32) = (7.0, 79.0, 154.0, 17.0);
/// Its text.
pub const LOCK_TEXT: &str = "Lock Pitch and Roll Values";

/// `lblUnitWarning`: `Location`, `Size`, and its Designer `Text`, bold and red, `Visible = False`.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.resx (lblUnitWarning.*)`
pub const WARNING_AT: (f32, f32, f32, f32) = (4.0, 646.0, 151.0, 13.0);
/// Its Designer text, which never shows: the label is made visible only with a warning's text.
#[cfg(test)]
pub const WARNING_TEXT: &str = "Breaking change warning";

// ---------------------------------------------------------------------------------------------
// The handlers' words.
// ---------------------------------------------------------------------------------------------

/// Write Params' question's caption.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:363-364`
pub const LARGE_TITLE: &str = "Large Value";

/// Write Params' question.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:363`
#[must_use]
pub fn large_text(param: &str) -> String {
    format!("{param} has more than doubled the last input. Are you sure?")
}

/// Write Params without a link.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:385`
pub const NOT_CONNECTED: &str = "You are not connected";

/// `lblUnitWarning`'s text before a 4.7 change's description.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:493`
pub const CHANGE_IN_47: &str = "Change in 4.7: ";

/// `ParamChanges47._byNewParam`: each renamed parameter's new name and its warning.
/// `// C#: ExtLibs/Utilities/paramchanges47.cs:11-128`
#[rustfmt::skip]
pub const CHANGES_47: &[(&str, &str)] = &[
    ("ARMING_SKIPCHK", "ARMING_CHECK → ARMING_SKIPCHK: select which checks to disable"),
    ("ATC_ANGLE_MAX", "ANGLE_MAX → ATC_ANGLE_MAX: units changed from cd to deg"),
    ("ATC_ACC_R_MAX", "ATC_ACCEL_R_MAX → ATC_ACC_R_MAX: units changed from cm/s/s to m/s/s"),
    ("ATC_ACC_P_MAX", "ATC_ACCEL_P_MAX → ATC_ACC_P_MAX: units changed from cm/s/s to m/s/s"),
    ("ATC_ACC_Y_MAX", "ATC_ACCEL_Y_MAX → ATC_ACC_Y_MAX: units changed from cm/s/s to m/s/s"),
    ("ATC_RATE_WPY_MAX", "ATC_SLEW_YAW → ATC_RATE_WPY_MAX: units changed from cdeg/s to deg/s"),
    ("CIRCLE_RADIUS_M", "CIRCLE_RADIUS → CIRCLE_RADIUS_M: units changed from cm to m"),
    ("LAND_SPD_MS", "LAND_SPEED → LAND_SPD_MS: units changed from cm/s to m/s"),
    ("LAND_SPD_HIGH_MS", "LAND_SPEED_HIGH → LAND_SPD_HIGH_MS: units changed from cm/s to m/s"),
    ("LAND_ALT_LOW_M", "LAND_ALT_LOW → LAND_ALT_LOW_M: units changed from cm to m"),
    ("LOIT_SPEED_MS", "LOIT_SPEED → LOIT_SPEED_MS: units changed from cm/s to m/s"),
    ("LOIT_ACC_MAX_M", "LOIT_ACC_MAX → LOIT_ACC_MAX_M: units changed from cm/s/s to m/s/s"),
    ("LOIT_BRK_ACC_M", "LOIT_BRK_ACCEL → LOIT_BRK_ACC_M: units changed from cm/s/s to m/s/s"),
    ("LOIT_BRK_JRK_M", "LOIT_BRK_JERK → LOIT_BRK_JRK_M: units changed from cm/s/s/s to m/s/s/s"),
    ("PILOT_ACC_Z", "PILOT_ACCEL_Z → PILOT_ACC_Z: units changed from cm/s/s to m/s/s"),
    ("PILOT_SPD_UP", "PILOT_SPEED_UP → PILOT_SPD_UP: units changed from cm/s to m/s"),
    ("PILOT_SPD_DN", "PILOT_SPEED_DN → PILOT_SPD_DN: units changed from cm/s to m/s"),
    ("PILOT_TKO_ALT_M", "PILOT_TKOFF_ALT → PILOT_TKO_ALT_M: units changed from cm to m"),
    ("PHLD_BRK_ANGLE", "PHLD_BRAKE_ANGLE → PHLD_BRK_ANGLE: units changed from cd to deg"),
    ("PHLD_BRK_RATE", "PHLD_BRAKE_RATE → PHLD_BRK_RATE: no scaling change, name only"),
    ("PSC_NE_VEL_P", "PSC_VELXY_P → PSC_NE_VEL_P: no scaling change, name only"),
    ("PSC_NE_VEL_I", "PSC_VELXY_I → PSC_NE_VEL_I: no scaling change, name only"),
    ("PSC_NE_VEL_D", "PSC_VELXY_D → PSC_NE_VEL_D: no scaling change, name only"),
    ("PSC_NE_VEL_IMAX", "PSC_VELXY_IMAX → PSC_NE_VEL_IMAX: value is now 100x smaller"),
    ("PSC_NE_VEL_FILT", "PSC_VELXY_FILT → PSC_NE_VEL_FILT: no scaling change, name only"),
    ("PSC_NE_VEL_FF", "PSC_VELXY_FF → PSC_NE_VEL_FF: no scaling change, name only"),
    ("PSC_NE_VEL_FLTE", "PSC_VELXY_FLTE → PSC_NE_VEL_FLTE: no scaling change, name only"),
    ("PSC_NE_VEL_FLTD", "PSC_VELXY_FLTD → PSC_NE_VEL_FLTD: no scaling change, name only"),
    ("PSC_D_VEL_P", "PSC_VELZ_P → PSC_D_VEL_P: no scaling change, name only"),
    ("PSC_D_VEL_I", "PSC_VELZ_I → PSC_D_VEL_I: no scaling change, name only"),
    ("PSC_D_VEL_D", "PSC_VELZ_D → PSC_D_VEL_D: no scaling change, name only"),
    ("PSC_D_VEL_IMAX", "PSC_VELZ_IMAX → PSC_D_VEL_IMAX: value is now 100x smaller"),
    ("PSC_D_VEL_FILT", "PSC_VELZ_FILT → PSC_D_VEL_FILT: no scaling change, name only"),
    ("PSC_D_VEL_FF", "PSC_VELZ_FF → PSC_D_VEL_FF: no scaling change, name only"),
    ("PSC_D_VEL_FLTE", "PSC_VELZ_FLTE → PSC_D_VEL_FLTE: no scaling change, name only"),
    ("PSC_D_VEL_FLTD", "PSC_VELZ_FLTD → PSC_D_VEL_FLTD: no scaling change, name only"),
    ("PSC_D_ACC_P", "PSC_ACCZ_P → PSC_D_ACC_P: value is now 10x smaller"),
    ("PSC_D_ACC_I", "PSC_ACCZ_I → PSC_D_ACC_I: value is now 10x smaller"),
    ("PSC_D_ACC_D", "PSC_ACCZ_D → PSC_D_ACC_D: value is now 10x smaller"),
    ("PSC_D_ACC_IMAX", "PSC_ACCZ_IMAX → PSC_D_ACC_IMAX: no scaling change, name only"),
    ("PSC_D_ACC_FILT", "PSC_ACCZ_FILT → PSC_D_ACC_FILT: no scaling change, name only"),
    ("PSC_D_ACC_FF", "PSC_ACCZ_FF → PSC_D_ACC_FF: no scaling change, name only"),
    ("PSC_D_ACC_FLTE", "PSC_ACCZ_FLTE → PSC_D_ACC_FLTE: no scaling change, name only"),
    ("PSC_D_ACC_FLTD", "PSC_ACCZ_FLTD → PSC_D_ACC_FLTD: no scaling change, name only"),
    ("RTL_ALT_M", "RTL_ALT → RTL_ALT_M: units changed from cm to m"),
    ("RTL_SPEED_MS", "RTL_SPEED → RTL_SPEED_MS: units changed from cm/s to m/s"),
    ("RTL_ALT_FINAL_M", "RTL_ALT_FINAL → RTL_ALT_FINAL_M: units changed from cm to m"),
    ("RTL_CLIMB_MIN_M", "RTL_CLIMB_MIN → RTL_CLIMB_MIN_M: units changed from cm to m"),
    ("WP_ACC", "WPNAV_ACCEL → WP_ACC: units changed from cm/s/s to m/s/s"),
    ("WP_ACC_CNR", "WPNAV_ACCEL_C → WP_ACC_CNR: units changed from cm/s/s to m/s/s"),
    ("WP_ACC_Z", "WPNAV_ACCEL_Z → WP_ACC_Z: units changed from cm/s/s to m/s/s"),
    ("WP_RADIUS_M", "WPNAV_RADIUS → WP_RADIUS_M: units changed from cm to m"),
    ("WP_SPD", "WPNAV_SPEED → WP_SPD: units changed from cm/s to m/s"),
    ("WP_SPD_DN", "WPNAV_SPEED_DN → WP_SPD_DN: units changed from cm/s to m/s"),
    ("WP_SPD_UP", "WPNAV_SPEED_UP → WP_SPD_UP: units changed from cm/s to m/s"),
    ("Q_A_ANGLE_MAX", "Q_ANGLE_MAX → Q_A_ANGLE_MAX: units changed from cd to deg"),
    ("Q_A_ACC_R_MAX", "Q_A_ACCEL_R_MAX → Q_A_ACC_R_MAX: units changed from cm/s/s to m/s/s"),
    ("Q_A_ACC_P_MAX", "Q_A_ACCEL_P_MAX → Q_A_ACC_P_MAX: units changed from cm/s/s to m/s/s"),
    ("Q_A_ACC_Y_MAX", "Q_A_ACCEL_Y_MAX → Q_A_ACC_Y_MAX: units changed from cm/s/s to m/s/s"),
    ("Q_A_RATE_WPY_MAX", "Q_A_SLEW_YAW → Q_A_RATE_WPY_MAX: units changed from cdeg/s to deg/s"),
    ("Q_LOIT_SPEED_MS", "Q_LOIT_SPEED → Q_LOIT_SPEED_MS: units changed from cm/s to m/s"),
    ("Q_LOIT_ACC_MAX_M", "Q_LOIT_ACC_MAX → Q_LOIT_ACC_MAX_M: units changed from cm/s/s to m/s/s"),
    ("Q_LOIT_BRK_ACC_M", "Q_LOIT_BRK_ACCEL → Q_LOIT_BRK_ACC_M: units changed from cm/s/s to m/s/s"),
    ("Q_LOIT_BRK_JRK_M", "Q_LOIT_BRK_JERK → Q_LOIT_BRK_JRK_M: units changed from cm/s/s/s to m/s/s/s"),
    ("Q_P_NE_VEL_P", "Q_P_VELXY_P → Q_P_NE_VEL_P: no scaling change, name only"),
    ("Q_P_NE_VEL_I", "Q_P_VELXY_I → Q_P_NE_VEL_I: no scaling change, name only"),
    ("Q_P_NE_VEL_D", "Q_P_VELXY_D → Q_P_NE_VEL_D: no scaling change, name only"),
    ("Q_P_NE_VEL_IMAX", "Q_P_VELXY_IMAX → Q_P_NE_VEL_IMAX: value is now 100x smaller"),
    ("Q_P_NE_VEL_FILT", "Q_P_VELXY_FILT → Q_P_NE_VEL_FILT: no scaling change, name only"),
    ("Q_P_NE_VEL_FF", "Q_P_VELXY_FF → Q_P_NE_VEL_FF: no scaling change, name only"),
    ("Q_P_NE_VEL_FLTE", "Q_P_VELXY_FLTE → Q_P_NE_VEL_FLTE: no scaling change, name only"),
    ("Q_P_NE_VEL_FLTD", "Q_P_VELXY_FLTD → Q_P_NE_VEL_FLTD: no scaling change, name only"),
    ("Q_P_D_VEL_P", "Q_P_VELZ_P → Q_P_D_VEL_P: no scaling change, name only"),
    ("Q_P_D_VEL_I", "Q_P_VELZ_I → Q_P_D_VEL_I: no scaling change, name only"),
    ("Q_P_D_VEL_D", "Q_P_VELZ_D → Q_P_D_VEL_D: no scaling change, name only"),
    ("Q_P_D_VEL_IMAX", "Q_P_VELZ_IMAX → Q_P_D_VEL_IMAX: value is now 100x smaller"),
    ("Q_P_D_VEL_FILT", "Q_P_VELZ_FILT → Q_P_D_VEL_FILT: no scaling change, name only"),
    ("Q_P_D_VEL_FF", "Q_P_VELZ_FF → Q_P_D_VEL_FF: no scaling change, name only"),
    ("Q_P_D_VEL_FLTE", "Q_P_VELZ_FLTE → Q_P_D_VEL_FLTE: no scaling change, name only"),
    ("Q_P_D_VEL_FLTD", "Q_P_VELZ_FLTD → Q_P_D_VEL_FLTD: no scaling change, name only"),
    ("Q_P_D_ACC_P", "Q_P_ACCZ_P → Q_P_D_ACC_P: value is now 10x smaller"),
    ("Q_P_D_ACC_I", "Q_P_ACCZ_I → Q_P_D_ACC_I: value is now 10x smaller"),
    ("Q_P_D_ACC_D", "Q_P_ACCZ_D → Q_P_D_ACC_D: value is now 10x smaller"),
    ("Q_P_D_ACC_IMAX", "Q_P_ACCZ_IMAX → Q_P_D_ACC_IMAX: no scaling change, name only"),
    ("Q_P_D_ACC_FILT", "Q_P_ACCZ_FILT → Q_P_D_ACC_FILT: no scaling change, name only"),
    ("Q_P_D_ACC_FF", "Q_P_ACCZ_FF → Q_P_D_ACC_FF: no scaling change, name only"),
    ("Q_P_D_ACC_FLTE", "Q_P_ACCZ_FLTE → Q_P_D_ACC_FLTE: no scaling change, name only"),
    ("Q_P_D_ACC_FLTD", "Q_P_ACCZ_FLTD → Q_P_D_ACC_FLTD: no scaling change, name only"),
    ("Q_WP_ACC", "Q_WP_ACCEL → Q_WP_ACC: units changed from cm/s/s to m/s/s"),
    ("Q_WP_ACC_CNR", "Q_WP_ACCEL_C → Q_WP_ACC_CNR: units changed from cm/s/s to m/s/s"),
    ("Q_WP_ACC_Z", "Q_WP_ACCEL_Z → Q_WP_ACC_Z: units changed from cm/s/s to m/s/s"),
    ("Q_WP_RADIUS_M", "Q_WP_RADIUS → Q_WP_RADIUS_M: units changed from cm to m"),
    ("Q_WP_SPD", "Q_WP_SPEED → Q_WP_SPD: units changed from cm/s to m/s"),
    ("Q_WP_SPD_DN", "Q_WP_SPEED_DN → Q_WP_SPD_DN: units changed from cm/s to m/s"),
    ("Q_WP_SPD_UP", "Q_WP_SPEED_UP → Q_WP_SPD_UP: units changed from cm/s to m/s"),
];

/// `ParamChanges47.changedByNewParamWarning`.
/// `// C#: ExtLibs/Utilities/paramchanges47.cs:154-157`
#[must_use]
pub fn warning_47(name: &str) -> Option<&'static str> {
    CHANGES_47
        .iter()
        .find(|(new, _)| *new == name)
        .map(|(_, warning)| *warning)
}

/// `VersionDetection.GetVersion(MainV2.comPort.MAV.VersionString) >= new Version(4, 7)`, the
/// version string being the firmware's banner.
///
/// A banner with no version in it, or none yet, is taken as older: `GetVersion` throws there, out
/// of the `Enter` handler, and the handler does nothing else - so the label is left hidden here.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:487; ExtLibs/Utilities/VersionDetection.cs:15-58`
#[must_use]
pub fn at_least_47(banner: Option<&str>) -> bool {
    banner
        .and_then(mp_params::pdef::Version::from_banner)
        .is_some_and(|version| {
            version
                >= mp_params::pdef::Version {
                    major: 4,
                    minor: 7,
                    build: None,
                    revision: None,
                }
        })
}

/// `MavlinkNumericUpDown.setup`'s choice of name: "default to first item", then the first the
/// vehicle has.
/// `// C#: Controls/MavlinkNumericUpDown.cs:54-64`
#[must_use]
pub fn chosen(names: &[&'static str], parameters: &[(String, f64)]) -> &'static str {
    names
        .iter()
        .copied()
        .find(|name| has(parameters, name))
        .or_else(|| names.first().copied())
        .unwrap_or_default()
}

/// `double.ToString()` for a parameter's value, as Write Params' restore types it back into a box:
/// fifteen significant digits, and the exponent form below 1E-04 and from 1E+15.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:373`
#[must_use]
pub fn double_text(value: f64) -> String {
    if value == 0.0 || !value.is_finite() {
        return "0".to_owned();
    }
    let rounded: f64 = format!("{value:.14e}").parse().unwrap_or(value);
    #[allow(clippy::cast_possible_truncation)] // a double's exponent fits
    let exponent = rounded.abs().log10().floor() as i32;
    if (-4..15).contains(&exponent) {
        return format!("{rounded}");
    }
    let text = format!("{rounded:e}");
    let (mantissa, power) = text.split_once('e').unwrap_or((&text, "0"));
    let power: i32 = power.parse().unwrap_or(0);
    let sign = if power < 0 { '-' } else { '+' };
    format!("{mantissa}E{sign}{:02}", power.abs())
}

/// `Text = text` on a `NumericUpDown`: WinForms puts the text in the box and, it being typed
/// rather than the box's own, reads it at once (`UpDownBase.Text`'s setter validates) - which
/// raises `ValueChanged` when the value it reads differs. Done here through the box's own
/// typing: the line cleared, the text typed, Enter. A disabled box is not typed into; the C#
/// would put the text in it all the same, but no box the page pairs is disabled while its pair
/// is not - both are named alike, and a vehicle has both or neither.
fn set_text(number: &mut Number, text: &str, now: Instant) -> Option<Question> {
    let _ = number.key(&press("u", None, true), now);
    for c in text.chars() {
        let typed = c.to_string();
        let _ = number.key(&press(&typed, Some(typed.clone()), false), now);
    }
    number.key(&press("enter", None, false), now).1
}

/// A key pressed: its name, the character it types, and whether Control is held.
fn press(key: &str, key_char: Option<String>, control: bool) -> KeyDownEvent {
    KeyDownEvent {
        keystroke: Keystroke {
            modifiers: Modifiers {
                control,
                ..Modifiers::default()
            },
            key: key.to_owned(),
            key_char,
        },
        is_held: false,
        prefer_character_input: false,
    }
}

// ---------------------------------------------------------------------------------------------
// The page object.
// ---------------------------------------------------------------------------------------------

/// Something that reads single parameters and says how each read went: `GetParam` on the link,
/// or a test's stand-in.
pub trait ParamReader: ParamWriter {
    /// Starts one read; `None` when there is nothing to read from.
    fn read(&self, name: &str) -> Option<Self::Handle>;
}

impl ParamReader for Telemetry {
    fn read(&self, name: &str) -> Option<Self::Handle> {
        self.read_parameter(name)
    }
}

/// What `Activate` reads besides the parameters' documentation.
#[derive(Debug, Clone, Copy)]
pub struct Activation<'a> {
    /// `MainV2.comPort.MAV.param`.
    pub parameters: &'a [(String, f64)],
    /// The screen the page object belongs to.
    pub key: Key,
    /// `MainV2.comPort.BaseStream.IsOpen`.
    pub connected: bool,
    /// `MainV2.comPort.MAV.cs.firmware`.
    pub firmware: Firmware,
}

/// A control that raises `ValueUpdated`: a box or a combo, by its index in [`NUMBERS`] or
/// [`COMBOS`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Control {
    /// A `MavlinkNumericUpDown`.
    Number(usize),
    /// A `MavlinkComboBox`.
    Combo(usize),
}

/// Write Params' question, waiting for its answer.
#[derive(Debug, Clone, PartialEq)]
pub struct Large {
    /// The name.
    pub param: String,
    /// What `changes` holds for it.
    pub value: f32,
}

/// Write Params, part way through its keys.
#[derive(Debug, Default)]
struct WriteRun {
    /// The names still to come, from `changes.Clone()`.
    keys: VecDeque<String>,
    /// The name whose `setParam` is on its way.
    current: Option<String>,
}

/// Refresh Screen, part way through its reads.
#[derive(Debug)]
struct Refresh<H> {
    /// The names still to read.
    names: VecDeque<String>,
    /// The read on its way.
    waiting: Option<H>,
}

/// How deep a pair of paired boxes may answer each other. The C#'s handlers call each other until
/// the two boxes agree, which takes one round; this only stops a loop the C# would run until its
/// stack overflowed.
const PAIRING_DEPTH: u8 = 8;

/// The page object and what it keeps.
#[derive(Debug)]
pub struct ExtendedTuning<H = mp_link::RequestId> {
    /// The screen the page object belongs to; a different one is a new object.
    made_for: Option<Key>,
    /// Whether the page is showing.
    active: bool,
    /// The page's `Enabled`.
    enabled: bool,
    /// `startup`, true from the constructor.
    startup: bool,
    /// The boxes, by [`NUMBERS`] index.
    numbers: Vec<Number>,
    /// The scale each box was last set up with.
    scales: Vec<f32>,
    /// The combos, by [`COMBOS`] index.
    combos: Vec<Combo>,
    /// `CHK_lockrollpitch.Checked`.
    lock: bool,
    /// `changes`: name and value, in the order first changed.
    changes: Vec<(String, f32)>,
    /// The controls whose `BackColor` is green, by `Name`.
    marked: BTreeSet<String>,
    /// `toolTip1`'s text for each control `Activate` gave one, by Designer name.
    tips: BTreeMap<&'static str, String>,
    /// `lblUnitWarning`'s text while it is visible.
    warning: Option<String>,
    /// The combo whose list is down.
    dropdown: Option<usize>,
    /// The box the keyboard is typing into.
    editing: Option<usize>,
    /// A box's out-of-range question.
    question: Option<(usize, Question)>,
    /// Write Params' "Large Value" question.
    large: Option<Large>,
    /// Write Params, while it runs.
    writing: Option<WriteRun>,
    /// Refresh Screen, while it runs.
    refresh: Option<Refresh<H>>,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// The `setParam` calls.
    queue: SetQueue<H>,
}

impl<H> Default for ExtendedTuning<H> {
    /// `InitializeComponent`: every box and combo disabled, the lock checked, `startup` true.
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            enabled: true,
            startup: true,
            numbers: NUMBERS
                .iter()
                .map(|spec| Number::new(designer_of(spec.name)))
                .collect(),
            scales: vec![1.0; NUMBERS.len()],
            combos: vec![Combo::default(); COMBOS.len()],
            lock: true,
            changes: Vec::new(),
            marked: BTreeSet::new(),
            tips: BTreeMap::new(),
            warning: None,
            dropdown: None,
            editing: None,
            question: None,
            large: None,
            writing: None,
            refresh: None,
            messages: VecDeque::new(),
            queue: SetQueue::default(),
        }
    }
}

/// A box's index by its Designer name.
#[must_use]
pub fn number_index(name: &str) -> Option<usize> {
    NUMBERS.iter().position(|spec| spec.name == name)
}

/// A combo's index by its Designer name.
#[cfg(test)]
#[must_use]
pub fn combo_index(name: &str) -> Option<usize> {
    COMBOS.iter().position(|spec| spec.name == name)
}

/// `Hashtable[name] = value`: a name already there keeps its place.
fn put(changes: &mut Vec<(String, f32)>, name: &str, value: f32) {
    match changes.iter_mut().find(|(held, _)| held == name) {
        Some((_, held)) => *held = value,
        None => changes.push((name.to_owned(), value)),
    }
}

impl<H: Copy> ExtendedTuning<H> {
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

    /// A box, by [`NUMBERS`] index.
    #[must_use]
    pub fn number(&self, index: usize) -> Option<&Number> {
        self.numbers.get(index)
    }

    /// A combo, by [`COMBOS`] index.
    #[must_use]
    pub fn combo(&self, index: usize) -> Option<&Combo> {
        self.combos.get(index)
    }

    /// `CHK_lockrollpitch.Checked`.
    #[must_use]
    pub const fn locked(&self) -> bool {
        self.lock
    }

    /// `changes`, in the order first changed.
    #[must_use]
    pub fn changes(&self) -> &[(String, f32)] {
        &self.changes
    }

    /// Whether a control's `BackColor` is green, by its `Name`.
    #[must_use]
    pub fn marked(&self, name: &str) -> bool {
        self.marked.contains(name)
    }

    /// The tool tip `Activate` gave a control, by Designer name.
    #[must_use]
    pub fn tip(&self, name: &str) -> Option<&str> {
        self.tips.get(name).map(String::as_str)
    }

    /// `lblUnitWarning`'s text while it is visible.
    #[must_use]
    pub fn warning(&self) -> Option<&str> {
        self.warning.as_deref()
    }

    /// The combo whose list is down.
    #[must_use]
    pub const fn dropdown(&self) -> Option<usize> {
        self.dropdown
    }

    /// The box being typed into.
    #[must_use]
    pub const fn editing(&self) -> Option<usize> {
        self.editing
    }

    /// A box's out-of-range question.
    #[must_use]
    pub fn question(&self) -> Option<&Question> {
        self.question.as_ref().map(|(_, question)| question)
    }

    /// Write Params' question.
    #[must_use]
    pub const fn large(&self) -> Option<&Large> {
        self.large.as_ref()
    }

    /// Whether Write Params is running.
    #[must_use]
    pub const fn writing(&self) -> bool {
        self.writing.is_some()
    }

    /// Whether Refresh Screen is running: its button is disabled meanwhile.
    #[must_use]
    pub const fn refreshing(&self) -> bool {
        self.refresh.is_some()
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

    /// How the last `setParam` ended.
    #[must_use]
    pub fn last_write(&self) -> Option<&str> {
        self.queue.last()
    }

    /// How many `setParam` jobs are queued or running.
    #[must_use]
    pub fn writes_pending(&self) -> usize {
        self.queue.pending()
    }

    /// A control's `Name`: its Designer name until `setup` gives it its parameter's.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:66-67; Controls/MavlinkComboBox.cs:53-54, 83-84`
    #[must_use]
    pub fn name_of(&self, control: Control) -> &str {
        let (param, designer) = match control {
            Control::Number(index) => (
                self.numbers.get(index).map(|number| number.param.as_str()),
                NUMBERS.get(index).map(|spec| spec.name),
            ),
            Control::Combo(index) => (
                self.combos.get(index).map(|combo| combo.param.as_str()),
                COMBOS.get(index).map(|spec| spec.name),
            ),
        };
        param
            .filter(|param| !param.is_empty())
            .or(designer)
            .unwrap_or_default()
    }

    /// `Controls.Find(name, true)[0]` among the boxes, in the page's `Controls` order. No combo can
    /// bear a name the handlers look for: those are `RATE_`, `STB_`, `ACRO_`, `LOITER_LON_` and
    /// `NAV_LON_` names, and the combos are `TUNE`, the `RCn_OPTION`s and three `INS_` switches.
    fn find_number(&self, name: &str) -> Option<usize> {
        CONTROLS_ORDER
            .iter()
            .filter_map(|designer| number_index(designer))
            .find(|index| self.name_of(Control::Number(*index)) == name)
    }

    /// `Controls.Find(name, true)[0]` among every control that has a value.
    fn find_control(&self, name: &str) -> Option<Control> {
        self.find_number(name).map(Control::Number).or_else(|| {
            (0..COMBOS.len())
                .find(|index| self.name_of(Control::Combo(*index)) == name)
                .map(Control::Combo)
        })
    }

    /// A box's value as `NumericUpDown.Value` holds it: what it writes, over its scale.
    fn value_of_box(&self, index: usize) -> f64 {
        let scale = self.scales.get(index).copied().unwrap_or(1.0);
        self.numbers
            .get(index)
            .map_or(0.0, |number| number.written() / f64::from(scale))
    }

    /// Shows the page: a new page object for a new screen, then `Activate`.
    /// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:26-204`
    pub fn activate(&mut self, vehicle: &Activation, lookup: Lookup) {
        if self.made_for != Some(vehicle.key) {
            let messages = std::mem::take(&mut self.messages);
            let queue = std::mem::take(&mut self.queue);
            *self = Self {
                made_for: Some(vehicle.key),
                messages,
                queue,
                ..Self::default()
            };
        }
        self.active = true;
        self.dropdown = None;
        let parameters = vehicle.parameters;
        // `// C#: :28-32`
        if !vehicle.connected {
            self.enabled = false;
            return;
        }
        // `// C#: :34-42`
        let quadplane = value_of(parameters, "Q_ENABLE").is_some_and(|value| value != 0.0);
        if vehicle.firmware == Firmware::ArduCopter2 || quadplane {
            self.enabled = true;
        } else {
            self.enabled = false;
            return;
        }
        self.startup = true;
        self.changes.clear();
        // Each `setup` reads only the parameter table, so their order among themselves changes
        // nothing. `// C#: :48-58, 142, 145, 150; Controls/MavlinkComboBox.cs:37-99`
        for (combo, spec) in self.combos.iter_mut().zip(COMBOS) {
            match spec.bind {
                ComboBind::Options(name) => combo.setup(options(name, lookup), name, parameters),
                ComboBind::Names(names) => {
                    if let Some(name) = names.iter().find(|name| has(parameters, name)) {
                        combo.setup(options(name, lookup), name, parameters);
                    }
                }
            }
        }
        // `// C#: :60-166`
        for ((number, scale), spec) in self.numbers.iter_mut().zip(&mut self.scales).zip(NUMBERS) {
            let (how, names) = spec.bind.call(parameters);
            number.setup(how, chosen(names, parameters), parameters, lookup);
            *scale = how.scale;
        }
        // "unlock entries if they differ": `Value`s compared as the C# compares them.
        // `// C#: :169-180`
        let pairs = [
            ("RATE_RLL_P", "RATE_PIT_P"),
            ("RATE_RLL_I", "RATE_PIT_I"),
            ("RATE_RLL_D", "RATE_PIT_D"),
            ("RATE_RLL_IMAX", "RATE_PIT_IMAX"),
        ];
        #[allow(clippy::float_cmp)] // `decimal !=`
        let differ = pairs.iter().any(|(roll, pitch)| {
            let value = |name: &str| number_index(name).map(|index| self.value_of_box(index));
            value(roll) != value(pitch)
        });
        if differ || has(parameters, "H_SWASH_TYPE") {
            self.lock = false;
        }
        // The tool tips of the controls inside the group boxes: `ParamName`, and its
        // documentation. `// C#: :182-201`
        self.tips.clear();
        let tip = |param: &str| {
            let description = lookup(param).map_or("", |meta| meta.description);
            format!("{param}:\n{description}")
        };
        for (number, spec) in self.numbers.iter().zip(NUMBERS) {
            if spec.group.is_some() {
                self.tips.insert(spec.name, tip(&number.param));
            }
        }
        for (combo, spec) in self.combos.iter().zip(COMBOS) {
            if spec.group.is_some() {
                self.tips.insert(spec.name, tip(&combo.param));
            }
        }
        self.startup = false;
    }

    /// The page hidden: `ConfigArducopter` is `IActivate` only, so nothing runs but the box being
    /// typed into losing the focus, which reads its text.
    pub fn hide(&mut self, now: Instant) {
        self.active = false;
        self.dropdown = None;
        self.leave(now);
    }

    /// The page object disposed with its screen; the messages and the writes on their way stay.
    fn dispose(&mut self) {
        let messages = std::mem::take(&mut self.messages);
        let queue = std::mem::take(&mut self.queue);
        *self = Self {
            messages,
            queue,
            ..Self::default()
        };
    }

    /// A click on "Lock Pitch and Roll Values": a plain `CheckBox`, no handler.
    pub fn toggle_lock(&mut self, now: Instant) {
        self.leave(now);
        self.dropdown = None;
        if self.enabled {
            self.lock = !self.lock;
        }
    }

    /// `numeric_ValueUpdated`, `EEPROM_View_float_TextChanged`: the value put in `changes` under
    /// the control's `Name`, the control marked green, and with the lock checked a roll or pitch
    /// `RATE_`, `STB_` or `ACRO_` name copied to its pair; `NAV_LAT_` and `LOITER_LAT_` names are
    /// copied to their `LON` pairs whatever the lock says.
    /// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:260-351, 479-482`
    fn value_updated(&mut self, sender: Control, value: f32, now: Instant, depth: u8) {
        if self.startup || depth > PAIRING_DEPTH {
            return;
        }
        let name = self.name_of(sender).to_owned();
        put(&mut self.changes, &name, value);
        self.marked.insert(name.clone());
        let text = match sender {
            Control::Number(index) => self.numbers.get(index).map(|number| number.shown()),
            Control::Combo(index) => self.combos.get(index).map(Combo::text),
        }
        .unwrap_or_default()
        .to_owned();
        if self.lock
            && (name.starts_with("RATE_") || name.starts_with("STB_") || name.starts_with("ACRO_"))
        {
            if name.contains("_RLL_") {
                self.pair(&name.replace("_RLL_", "_PIT_"), value, &text, now, depth);
            } else if name.contains("_PIT_") {
                self.pair(&name.replace("_PIT_", "_RLL_"), value, &text, now, depth);
            }
        }
        if name.contains("NAV_LAT_") {
            self.pair(
                &name.replace("NAV_LAT_", "NAV_LON_"),
                value,
                &text,
                now,
                depth,
            );
        }
        if name.contains("LOITER_LAT_") {
            self.pair(
                &name.replace("LOITER_LAT_", "LOITER_LON_"),
                value,
                &text,
                now,
                depth,
            );
        }
    }

    /// One pairing: `changes[newname] = value`, and the control of that name, if there is one,
    /// given the sender's text - which raises its own `ValueUpdated` when its value changes - and
    /// marked green.
    /// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:295-345`
    fn pair(&mut self, newname: &str, value: f32, text: &str, now: Instant, depth: u8) {
        put(&mut self.changes, newname, value);
        let Some(index) = self.find_number(newname) else {
            return;
        };
        if let Some(number) = self.numbers.get_mut(index)
            && number.enabled
            && let Some(question) = set_text(number, text, now)
        {
            self.question = Some((index, question));
        }
        self.updated(index, now, depth + 1);
        self.marked.insert(newname.to_owned());
    }

    /// After a box has had its `ValueChanged`: `ValueUpdated`, if it was raised. The box's timer is
    /// what `servo_output::Number` starts for a change; this page's boxes, whose `ValueUpdated`
    /// has a subscriber, raise that instead, so the timer is taken as the event.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:147-152`
    fn updated(&mut self, index: usize, now: Instant, depth: u8) {
        if let Some(write) = self.numbers.get_mut(index).and_then(Number::flush) {
            #[allow(clippy::cast_possible_truncation)] // `write.value` is a float's value
            let value = write.value as f32;
            self.value_updated(Control::Number(index), value, now, depth);
        }
    }

    /// `OnEnter_NumUpDown`: on a 4.7 or later vehicle, a box whose parameter is one 4.7 renamed
    /// shows the change in red under the buttons; any other box hides it.
    /// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:484-501`
    fn enter(&mut self, index: usize, at_least_47: bool) {
        let param = self
            .numbers
            .get(index)
            .map(|number| number.param.as_str())
            .unwrap_or_default();
        self.warning = at_least_47
            .then(|| warning_47(param))
            .flatten()
            .map(|warning| format!("{CHANGE_IN_47}{warning}"));
    }

    /// A box clicked into: the one being typed into loses the focus first, and the new one's
    /// `Enter` runs.
    pub fn begin(&mut self, index: usize, now: Instant, at_least_47: bool) {
        if self.editing == Some(index) {
            return;
        }
        self.leave(now);
        self.dropdown = None;
        if self.enabled && self.numbers.get(index).is_some_and(|number| number.enabled) {
            self.editing = Some(index);
            self.enter(index, at_least_47);
        }
    }

    /// The box being typed into loses the focus, which reads its text.
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
        self.updated(index, now, 0);
    }

    /// A key for the box being typed into. Ctrl+S is `ProcessCmdKey`'s: Write Params.
    /// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:206-215`
    pub fn key(&mut self, event: &KeyDownEvent, now: Instant) -> bool {
        let Some(index) = self.editing else {
            return false;
        };
        let keystroke = &event.keystroke;
        if keystroke.modifiers.control && keystroke.key.eq_ignore_ascii_case("s") {
            self.write_params();
            return true;
        }
        let Some(number) = self.numbers.get_mut(index) else {
            return false;
        };
        let (handled, question) = number.key(event, now);
        if let Some(question) = question {
            self.question = Some((index, question));
        }
        self.updated(index, now, 0);
        handled
    }

    /// A box's up or down arrow: the box takes the focus, then steps.
    pub fn step(&mut self, index: usize, up: bool, now: Instant, at_least_47: bool) {
        self.begin(index, now, at_least_47);
        if !self.enabled {
            return;
        }
        if let Some(question) = self
            .numbers
            .get_mut(index)
            .and_then(|number| number.step(up, now))
        {
            self.question = Some((index, question));
        }
        self.updated(index, now, 0);
    }

    /// The out-of-range question answered.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:136-152`
    pub fn answer(&mut self, yes: bool, now: Instant) {
        let Some((index, question)) = self.question.take() else {
            return;
        };
        if let Some(number) = self.numbers.get_mut(index) {
            number.answer(&question, yes, now);
        }
        self.updated(index, now, 0);
    }

    /// Drops a combo's list down, or back up.
    pub fn toggle_dropdown(&mut self, index: usize, now: Instant) {
        self.leave(now);
        let enabled = self.enabled && self.combos.get(index).is_some_and(|combo| combo.enabled);
        self.dropdown = if self.dropdown == Some(index) || !enabled {
            None
        } else {
            if let Some(combo) = self.combos.get_mut(index) {
                combo.open_list();
            }
            Some(index)
        };
    }

    /// The wheel over a list.
    pub fn scroll_list(&mut self, index: usize, lines: i32) {
        if self.dropdown == Some(index)
            && let Some(combo) = self.combos.get_mut(index)
        {
            combo.scroll_list(lines);
        }
    }

    /// A row chosen from a combo's list. A combo with `ValueUpdated` wired raises it - into
    /// `changes` - and one without writes its parameter itself.
    /// `// C#: Controls/MavlinkComboBox.cs:133-200`
    pub fn choose(&mut self, index: usize, key: i64, now: Instant) -> Vec<Job> {
        self.dropdown = None;
        if !self.enabled {
            return Vec::new();
        }
        let Some(write) = self
            .combos
            .get_mut(index)
            .and_then(|combo| combo.choose(key))
        else {
            return Vec::new();
        };
        if COMBOS.get(index).is_some_and(|spec| spec.value_updated) {
            #[allow(clippy::cast_possible_truncation)] // `(float)(int)SelectedValue`
            let value = write.value as f32;
            self.value_updated(Control::Combo(index), value, now, 0);
            Vec::new()
        } else {
            vec![Job::control(write)]
        }
    }

    /// Queues handlers' jobs.
    pub fn push(&mut self, jobs: Vec<Job>) {
        self.queue.push(jobs);
    }

    /// `BUT_writePIDS_Click`, or ctrl+S: the names in `changes` taken, to be written one at a
    /// time. A second press while it runs is not taken: the C#'s handler holds the window until it
    /// returns.
    ///
    /// `changes` is a `Hashtable`, and the C# walks `changes.Clone().Keys` in its buckets' order,
    /// which .NET does not define; here the names go in the order they were first changed.
    /// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:353-357`
    pub fn write_params(&mut self) {
        if !self.enabled || self.writing.is_some() {
            return;
        }
        self.writing = Some(WriteRun {
            keys: self.changes.iter().map(|(name, _)| name.clone()).collect(),
            current: None,
        });
    }

    /// Moves Write Params on: each name in turn checked against the vehicle's value - a value
    /// more than double it asks first - then, connected, written.
    /// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:357-410`
    fn advance_write(&mut self, parameters: &[(String, f64)], connected: bool) {
        loop {
            let Some(run) = self.writing.as_mut() else {
                return;
            };
            if run.current.is_some() || self.large.is_some() {
                return;
            }
            let Some(param) = run.keys.pop_front() else {
                self.writing = None;
                return;
            };
            // `(float)changes[value] > (float)MainV2.comPort.MAV.param[value] * 2.0f`: a name
            // `changes` no longer holds, or the vehicle does not - a pair's `LON` name, say - is
            // a null cast, which the `catch` reports. `// C#: :361, 406-409`
            let value = self
                .changes
                .iter()
                .find(|(name, _)| *name == param)
                .map(|(_, value)| *value);
            let (Some(value), Some(held)) = (value, value_of(parameters, &param)) else {
                self.messages.push_back(error(set_failed(&param)));
                continue;
            };
            #[allow(clippy::cast_possible_truncation)] // `(float)`
            let held = held as f32;
            if value > held * 2.0 {
                self.large = Some(Large { param, value });
                return;
            }
            self.send(param, value, connected);
        }
    }

    /// The not-connected check, then `setParam`: its `bool` is not read, a throw is the
    /// `catch`'s "Set NAME Failed" and leaves the name in `changes`.
    /// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:383-404`
    fn send(&mut self, param: String, value: f32, connected: bool) {
        if !connected {
            self.messages.push_back(error(NOT_CONNECTED));
            self.writing = None;
            return;
        }
        let failed = set_failed(&param);
        self.queue.push([Job::new(
            "write",
            [Set::caught(param.clone(), f64::from(value), failed)],
        )]);
        if let Some(run) = self.writing.as_mut() {
            run.current = Some(param);
        }
    }

    /// Write Params' question answered. Yes writes; No puts the vehicle's value back in the
    /// control - typed into a box, which raises its `ValueUpdated` - clears its colour, and ends
    /// the run.
    /// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:361-381`
    pub fn answer_large(&mut self, yes: bool, vehicle: &Activation, now: Instant) {
        let Some(large) = self.large.take() else {
            return;
        };
        if yes {
            self.send(large.param, large.value, vehicle.connected);
            return;
        }
        // `textControls[0].Text = MainV2.comPort.MAV.param[value].Value.ToString()`: a box that
        // shows its parameter scaled is given the parameter's own value, as the C# gives it. A
        // combo's list has no row of that text, so a combo keeps its selection.
        match self.find_control(&large.param) {
            Some(Control::Number(index)) => {
                let text = double_text(value_of(vehicle.parameters, &large.param).unwrap_or(0.0));
                if let Some(number) = self.numbers.get_mut(index)
                    && number.enabled
                    && let Some(question) = set_text(number, &text, now)
                {
                    self.question = Some((index, question));
                }
                self.updated(index, now, 0);
                self.marked.remove(&large.param);
            }
            Some(Control::Combo(_)) => {
                self.marked.remove(&large.param);
            }
            None => {}
        }
        self.writing = None;
    }

    /// `BUT_refreshpart_Click`: every box's parameter read again, by its `Name`, then `Activate`.
    /// Only the `MavlinkNumericUpDown`s: `updateparam` asks for a control whose type is exactly
    /// `ComboBox`, which a `MavlinkComboBox` is not. The button is disabled while it runs.
    /// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:441-477`
    pub fn refresh(&mut self, connected: bool) {
        if !connected || !self.enabled || self.refresh.is_some() {
            return;
        }
        let names = CONTROLS_ORDER
            .iter()
            .filter_map(|designer| number_index(designer))
            .map(|index| self.name_of(Control::Number(index)).to_owned())
            .collect();
        self.refresh = Some(Refresh {
            names,
            waiting: None,
        });
    }

    /// Moves Refresh Screen on: one `GetParam` at a time, a failure passed over as the empty
    /// `catch` passes it; when the last has ended, `Activate`.
    fn advance_refresh<L: ParamReader<Handle = H>>(
        &mut self,
        link: &L,
        vehicle: &Activation,
        lookup: Lookup,
    ) {
        loop {
            let Some(refresh) = self.refresh.as_mut() else {
                return;
            };
            if let Some(handle) = refresh.waiting {
                if link.progress(handle) == Progress::Waiting {
                    return;
                }
                refresh.waiting = None;
            }
            match refresh.names.pop_front() {
                Some(name) => refresh.waiting = link.read(&name),
                None => {
                    self.refresh = None;
                    self.activate(vehicle, lookup);
                    return;
                }
            }
        }
    }

    /// Once a frame: a page object whose screen has gone is disposed, a box that lost the focus
    /// has its text read, and Write Params and Refresh Screen move on as the link answers.
    #[allow(clippy::too_many_arguments)]
    pub fn tick<L: ParamReader<Handle = H>>(
        &mut self,
        link: &L,
        vehicle: &Activation,
        on_config: bool,
        focused: bool,
        now: Instant,
        lookup: Lookup,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_config || self.made_for != Some(vehicle.key))
        {
            self.dispose();
        }
        if self.editing.is_some() && !focused {
            self.leave(now);
        }
        for event in self.queue.advance(link, &mut self.messages) {
            // Each `setParam` of Write Params: returned, the name leaves `changes` and its
            // control's colour is cleared; thrown, it stays. `// C#: :389-409`
            if let Event::Done {
                tag: "write",
                threw,
            } = event
                && let Some(param) = self.writing.as_mut().and_then(|run| run.current.take())
                && !threw
            {
                self.changes.retain(|(name, _)| *name != param);
                self.marked.remove(&param);
            }
        }
        self.advance_write(vehicle.parameters, vehicle.connected);
        self.advance_refresh(link, vehicle, lookup);
    }
}

// ---------------------------------------------------------------------------------------------
// Facts.
// ---------------------------------------------------------------------------------------------

/// Facts a UI test asserts on.
pub fn record_facts(page: &ExtendedTuning, view: &TelemetryView) {
    use crate::facts::record;
    let none = || "none".to_owned();
    record("config.extended.active", page.is_active());
    record("config.extended.enabled", page.enabled());
    record(
        "config.extended.lock",
        if page.locked() {
            "checked"
        } else {
            "unchecked"
        },
    );
    let changes: Vec<String> = page
        .changes()
        .iter()
        .map(|(name, value)| format!("{name}={value}"))
        .collect();
    record(
        "config.extended.changes",
        if changes.is_empty() {
            none()
        } else {
            changes.join(",")
        },
    );
    record("config.extended.warning", page.warning().unwrap_or("none"));
    record(
        "config.extended.editing",
        page.editing()
            .and_then(|index| NUMBERS.get(index))
            .map_or("none", |spec| spec.name),
    );
    record(
        "config.extended.question",
        page.question().map_or_else(none, Question::text),
    );
    record(
        "config.extended.large",
        page.large()
            .map_or_else(none, |large| large_text(&large.param)),
    );
    record(
        "config.extended.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    record("config.extended.write", page.last_write().unwrap_or("none"));
    record("config.extended.writes.pending", page.writes_pending());
    record("config.extended.writing", page.writing());
    record("config.extended.refreshing", page.refreshing());
    record(
        "config.extended.list.top",
        page.dropdown()
            .and_then(|index| page.combo(index))
            .map_or_else(none, |combo| combo.top_index.to_string()),
    );
    let mut bound = BTreeSet::new();
    for (index, spec) in NUMBERS.iter().enumerate() {
        let Some(number) = page.number(index) else {
            continue;
        };
        let name = spec.name;
        let param = page.name_of(Control::Number(index));
        record(format!("config.extended.number.{name}"), number.shown());
        record(format!("config.extended.number.{name}.param"), param);
        record(
            format!("config.extended.number.{name}.enabled"),
            page.enabled() && number.enabled,
        );
        record(
            format!("config.extended.number.{name}.marked"),
            page.marked(param),
        );
        record(
            format!("config.extended.tip.{name}"),
            page.tip(name).unwrap_or("none"),
        );
        bound.insert(param.to_owned());
    }
    for (index, spec) in COMBOS.iter().enumerate() {
        let Some(combo) = page.combo(index) else {
            continue;
        };
        let name = spec.name;
        let param = page.name_of(Control::Combo(index));
        record(
            format!("config.extended.combo.{name}"),
            combo.selected.map_or_else(none, |key| key.to_string()),
        );
        record(format!("config.extended.combo.{name}.text"), combo.text());
        record(format!("config.extended.combo.{name}.param"), param);
        record(
            format!("config.extended.combo.{name}.enabled"),
            page.enabled() && combo.enabled,
        );
        record(
            format!("config.extended.combo.{name}.marked"),
            page.marked(param),
        );
        record(
            format!("config.extended.tip.{name}"),
            page.tip(name).unwrap_or("none"),
        );
        bound.insert(param.to_owned());
    }
    for (name, _) in page.changes() {
        bound.insert(name.clone());
    }
    // Every parameter a control could bind, whether or not the page has been shown yet: a script
    // reads the vehicle's values on the parameter screen before opening the page, as
    // config-servo.gui does for SERVO*.
    for spec in NUMBERS {
        let lists: &[&[&str]] = match spec.bind {
            Bind::Names(_, names) => &[names],
            Bind::Imax {
                when, otherwise, ..
            } => &[when, otherwise.1],
        };
        for name in lists.iter().flat_map(|names| names.iter()) {
            bound.insert((*name).to_owned());
        }
    }
    for spec in COMBOS {
        if let ComboBind::Names(names) = spec.bind {
            for name in names {
                bound.insert((*name).to_owned());
            }
        }
    }
    for name in bound {
        if let Some(value) = value_of(&view.parameters, &name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// The page's keyboard focus: the box being typed into.
pub struct Focus {
    /// The `NumericUpDown` being typed into.
    pub number: FocusHandle,
}

impl Focus {
    /// A new handle.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            number: cx.focus_handle(),
        }
    }
}

/// A tool tip's text, `toolTip1`'s, a line to a line.
struct Tip {
    text: SharedString,
}

impl Render for Tip {
    fn render(&mut self, _window: &mut Window, _cx: &mut Context<Self>) -> impl IntoElement {
        div()
            .max_w(px(420.0))
            .p_1()
            .bg(rgb(theme::PANEL))
            .border_1()
            .border_color(rgb(theme::BORDER))
            .rounded_sm()
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .children(
                self.text
                    .lines()
                    .map(|line| div().child(SharedString::from(line.to_owned()))),
            )
    }
}

/// `toolTip1.InitialDelay`.
/// `// C#: GCSViews/ConfigurationView/ConfigArducopter.Designer.cs (toolTip1.InitialDelay = 500)`
const TIP_DELAY: Duration = Duration::from_millis(500);

/// A box at its place: a green ring while its `BackColor` is, and its tool tip.
fn placed(
    id: &str,
    (x, y, width, height): (f32, f32, f32, f32),
    marked: bool,
    tip: Option<&str>,
    control: AnyElement,
) -> AnyElement {
    let ring = marked.then(|| {
        div()
            .absolute()
            .left(px(-2.0))
            .top(px(-2.0))
            .w(px(width + 4.0))
            .h(px(height + 4.0))
            .rounded_sm()
            .border_2()
            .border_color(rgb(theme::OK))
    });
    let base = div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
        .children(ring)
        .child(control);
    match tip {
        Some(text) => {
            let text = SharedString::from(text.to_owned());
            base.id(SharedString::from(format!("{id}-tip")))
                .tooltip(move |_window: &mut Window, cx: &mut App| {
                    let text = text.clone();
                    cx.new(|_| Tip { text }).into()
                })
                .tooltip_show_delay(TIP_DELAY)
                .into_any_element()
        }
        None => base.into_any_element(),
    }
}

/// A control's id: `extended-` and its Designer name.
fn control_id(name: &str) -> String {
    format!("extended-{name}")
}

/// Where a control's parent puts it on the page.
fn origin(group: Option<&str>) -> (f32, f32) {
    group
        .and_then(|name| GROUPS.iter().find(|spec| spec.name == name))
        .map_or((0.0, 0.0), |spec| (spec.at.0, spec.at.1))
}

/// The page, laid out as `ConfigArducopter.resx` lays it out.
pub fn page(
    tuning: &ExtendedTuning,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if !tuning.is_active() {
        return div().into_any_element();
    }
    let enabled = tuning.enabled();
    let mut groups: Vec<gpui::Div> = GROUPS
        .iter()
        .map(|spec| group(spec.at, spec.caption, enabled))
        .collect();
    let mut body = div()
        .relative()
        .w(px(PAGE.0))
        .h(px(PAGE.1))
        .bg(rgb(theme::PANEL));
    // Each control goes into its group box, or onto the page.
    let mut place = |parent: Option<&str>, element: AnyElement| match parent
        .and_then(|name| GROUPS.iter().position(|spec| spec.name == name))
        .and_then(|index| groups.get_mut(index))
    {
        Some(group) => *group = std::mem::replace(group, div()).child(element),
        None => body = std::mem::replace(&mut body, div()).child(element),
    };
    for spec in LABELS {
        place(
            spec.group,
            label(spec.at.0, spec.at.1, spec.text, enabled).into_any_element(),
        );
    }
    for (index, spec) in NUMBERS.iter().enumerate() {
        let Some(number) = tuning.number(index) else {
            continue;
        };
        // A disabled page's boxes are disabled already: the page is disabled only by an
        // `Activate` that returned before setting any up, on a page object of its own.
        let id = control_id(spec.name);
        let (_, _, width, height) = spec.at;
        let control = number_box(
            id.clone(),
            number,
            tuning.editing() == Some(index),
            &focus.number,
            (0.0, 0.0, width, height),
            NumberHandlers {
                begin: move |this: &mut MissionPlanner| {
                    let recent = at_least_47(this.telemetry.firmware_banner());
                    this.extended_tuning.begin(index, Instant::now(), recent);
                },
                key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
                    this.extended_tuning.key(event, Instant::now())
                },
                step: move |this: &mut MissionPlanner, up: bool| {
                    let recent = at_least_47(this.telemetry.firmware_banner());
                    this.extended_tuning.step(index, up, Instant::now(), recent);
                },
            },
            window,
            cx,
        );
        let marked = tuning.marked(tuning.name_of(Control::Number(index)));
        place(
            spec.group,
            placed(&id, spec.at, marked, tuning.tip(spec.name), control),
        );
    }
    for (index, spec) in COMBOS.iter().enumerate() {
        let Some(combo) = tuning.combo(index) else {
            continue;
        };
        let id = control_id(spec.name);
        let mut shown = combo.clone();
        shown.enabled &= enabled;
        let (_, _, width, height) = spec.at;
        let control = combo_box(
            id.clone(),
            &shown,
            (0.0, 0.0, width, height),
            move |this| this.extended_tuning.toggle_dropdown(index, Instant::now()),
            cx,
        );
        let marked = tuning.marked(tuning.name_of(Control::Combo(index)));
        place(
            spec.group,
            placed(&id, spec.at, marked, tuning.tip(spec.name), control),
        );
    }
    let mut lock = Check::default();
    lock.state = if tuning.locked() {
        CheckState::Checked
    } else {
        CheckState::Unchecked
    };
    lock.enabled = enabled;
    place(
        None,
        check_box(
            control_id("CHK_lockrollpitch"),
            &lock,
            LOCK_TEXT,
            (LOCK_AT.0, LOCK_AT.1),
            |this| this.extended_tuning.toggle_lock(Instant::now()),
            cx,
        ),
    );
    for spec in BUTTONS.iter().filter(|spec| spec.visible) {
        let element = match spec.name {
            "BUT_writePIDS" => button(
                "extended-BUT_writePIDS",
                spec.text,
                spec.at,
                enabled,
                |this, _window, _cx| {
                    this.extended_tuning.leave(Instant::now());
                    this.extended_tuning.write_params();
                },
                cx,
            ),
            _ => button(
                "extended-BUT_refreshpart",
                spec.text,
                spec.at,
                enabled && !tuning.refreshing(),
                |this, _window, _cx| {
                    let view = this.telemetry.view();
                    let connected = view.connected && view.vehicle.is_some();
                    this.extended_tuning.leave(Instant::now());
                    this.extended_tuning.refresh(connected);
                },
                cx,
            ),
        };
        place(None, element);
    }
    if let Some(warning) = tuning.warning() {
        place(
            None,
            crate::probe::measured("extended-lblUnitWarning", div())
                .absolute()
                .left(px(WARNING_AT.0))
                .top(px(WARNING_AT.1))
                .whitespace_nowrap()
                .text_xs()
                .font_weight(FontWeight::BOLD)
                .text_color(rgb(theme::ALERT))
                .child(warning.to_owned())
                .into_any_element(),
        );
    }
    for group in groups {
        body = body.child(group);
    }
    // The list last, over what is under it.
    if let Some(index) = tuning.dropdown()
        && let (Some(spec), Some(combo)) = (COMBOS.get(index), tuning.combo(index))
    {
        let (x, y) = origin(spec.group);
        let (cx0, cy0, width, height) = spec.at;
        body = body.child(dropdown(
            &control_id(spec.name),
            combo,
            (x + cx0, y + cy0 + height, width.max(DROP_DOWN_WIDTH)),
            move |this, key| {
                let jobs = this.extended_tuning.choose(index, key, Instant::now());
                this.extended_tuning.push(jobs);
            },
            move |this, lines| this.extended_tuning.scroll_list(index, lines),
            cx,
        ));
    }
    body.into_any_element()
}

/// The question or message box showing, over the whole window.
pub fn overlay(
    tuning: &ExtendedTuning,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let yes_no =
        |yes: &'static str, no: &'static str, large: bool, cx: &mut Context<MissionPlanner>| {
            let answer = move |yes: bool| {
                move |this: &mut MissionPlanner| {
                    let now = Instant::now();
                    if large {
                        let view = this.telemetry.view();
                        let banner = this.telemetry.firmware_banner();
                        let vehicle = crate::setup::Vehicle::of(&view, banner);
                        let activation = Activation {
                            parameters: &view.parameters,
                            key: Key::of(&view),
                            connected: vehicle.connected,
                            firmware: vehicle.firmware,
                        };
                        this.extended_tuning.answer_large(yes, &activation, now);
                    } else {
                        this.extended_tuning.answer(yes, now);
                    }
                }
            };
            let (on_yes, on_no) = (answer(true), answer(false));
            vec![
                action(
                    yes,
                    "Yes",
                    theme::ACCENT,
                    true,
                    cx.listener(move |this, _event: &(), _window, cx| {
                        on_yes(this);
                        cx.notify();
                    }),
                ),
                action(
                    no,
                    "No",
                    theme::ACCENT,
                    true,
                    cx.listener(move |this, _event: &(), _window, cx| {
                        on_no(this);
                        cx.notify();
                    }),
                ),
            ]
        };
    if let Some(question) = tuning.question() {
        let buttons = yes_no("extended-question-yes", "extended-question-no", false, cx);
        return Some(modal(
            "extended-question",
            OUT_OF_RANGE_TITLE,
            &question.text(),
            false,
            buttons,
            window,
        ));
    }
    if let Some(large) = tuning.large() {
        let buttons = yes_no("extended-large-yes", "extended-large-no", true, cx);
        return Some(modal(
            "extended-large",
            LARGE_TITLE,
            &large_text(&large.param),
            false,
            buttons,
            window,
        ));
    }
    let message = tuning.message()?;
    Some(message_box(
        "extended-message",
        "extended-message-ok",
        message,
        window,
        |this| this.extended_tuning.dismiss_message(),
        cx,
    ))
}

// ---------------------------------------------------------------------------------------------
// Its part in the application.
// ---------------------------------------------------------------------------------------------

impl MissionPlanner {
    /// What `Activate` reads, from the link as it is now.
    fn extended_tuning_vehicle<'a>(&self, view: &'a TelemetryView) -> Activation<'a> {
        let vehicle = crate::setup::Vehicle::of(view, self.telemetry.firmware_banner());
        Activation {
            parameters: &view.parameters,
            key: Key::of(view),
            connected: vehicle.connected,
            firmware: vehicle.firmware,
        }
    }

    /// `Activate`, each time the page is chosen.
    /// `// C#: GCSViews/ConfigurationView/ConfigArducopter.cs:26-204`
    pub(crate) fn extended_tuning_activate(&mut self) {
        let view = self.telemetry.view();
        let vehicle = self.extended_tuning_vehicle(&view);
        self.extended_tuning
            .activate(&vehicle, crate::metadata::lookup);
    }

    /// The page hidden when another is chosen.
    pub(crate) fn extended_tuning_hide(&mut self) {
        self.extended_tuning.hide(Instant::now());
    }

    /// The page.
    pub(crate) fn extended_tuning_page(
        &self,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> AnyElement {
        page(&self.extended_tuning, &self.extended_focus, window, cx)
    }

    /// Once a frame.
    pub(crate) fn extended_tuning_tick(&mut self, view: &TelemetryView, window: &Window) {
        let on_config = self.screen == crate::Screen::Config;
        let focused = self.extended_focus.number.is_focused(window);
        let vehicle = self.extended_tuning_vehicle(view);
        self.extended_tuning.tick(
            &self.telemetry,
            &vehicle,
            on_config,
            focused,
            Instant::now(),
            crate::metadata::lookup,
        );
    }

    /// The question or message box the page is showing.
    pub(crate) fn extended_tuning_overlay(
        &self,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> Option<AnyElement> {
        overlay(&self.extended_tuning, window, cx)
    }
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;

    use mp_link::requests::RequestOutcome;
    use mp_params::ParamMeta;

    use super::*;
    use crate::config::optional::error;

    /// The bundled documentation, so no test depends on what another has installed.
    fn bundled(name: &str) -> Option<&'static ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    fn table(entries: &[(&str, f64)]) -> Vec<(String, f64)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    /// The whole of a real SITL copter's parameters, as `mpr param save` wrote them.
    fn sitl() -> Vec<(String, f64)> {
        let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/params/sitl-copter.param");
        let file = mp_params::param_file::ParamFile::load(&fixture).expect("the SITL dump");
        file.iter()
            .map(|(name, value)| (name.to_owned(), value))
            .collect()
    }

    /// A copter from before 3.4: the old names, roll and pitch alike.
    fn old_copter() -> Vec<(String, f64)> {
        table(&[
            ("RATE_RLL_P", 0.15),
            ("RATE_PIT_P", 0.15),
            ("RATE_RLL_I", 0.1),
            ("RATE_PIT_I", 0.1),
            ("RATE_RLL_D", 0.004),
            ("RATE_PIT_D", 0.004),
            ("RATE_RLL_IMAX", 5000.0),
            ("RATE_PIT_IMAX", 5000.0),
            ("STB_RLL_P", 4.5),
            ("STB_PIT_P", 4.5),
        ])
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn copter(parameters: &[(String, f64)]) -> Activation<'_> {
        Activation {
            parameters,
            key: key(),
            connected: true,
            firmware: Firmware::ArduCopter2,
        }
    }

    fn opened(parameters: &[(String, f64)]) -> ExtendedTuning<usize> {
        let mut page = ExtendedTuning::<usize>::default();
        page.activate(&copter(parameters), bundled);
        page
    }

    fn at(name: &str) -> usize {
        number_index(name).expect(name)
    }

    fn combo_at(name: &str) -> usize {
        combo_index(name).expect(name)
    }

    fn shown<H: Copy>(page: &ExtendedTuning<H>, name: &str) -> String {
        page.number(at(name)).expect(name).shown().to_owned()
    }

    fn param_of<H: Copy>(page: &ExtendedTuning<H>, name: &str) -> String {
        page.name_of(Control::Number(at(name))).to_owned()
    }

    fn changes(entries: &[(&str, f32)]) -> Vec<(String, f32)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    /// Typed into a box as the keyboard does: clicked into, the line cleared, the text, Enter.
    fn type_into<H: Copy>(page: &mut ExtendedTuning<H>, name: &str, text: &str) {
        let now = Instant::now();
        page.begin(at(name), now, false);
        page.key(&press("u", None, true), now);
        for c in text.chars() {
            let typed = c.to_string();
            page.key(&press(&typed, Some(typed.clone()), false), now);
        }
        page.key(&press("enter", None, false), now);
    }

    /// Where the reads' handles start, apart from the writes'.
    const READ: usize = 1_000_000;

    /// A link that answers each write as told, ends every read at once, and remembers both.
    #[derive(Default)]
    struct Link {
        writes: RefCell<Vec<(String, f64)>>,
        reads: RefCell<Vec<String>>,
        outcomes: BTreeMap<String, Progress>,
    }

    impl Link {
        fn answering(outcomes: &[(&str, RequestOutcome)]) -> Self {
            Self {
                outcomes: outcomes
                    .iter()
                    .map(|(name, outcome)| ((*name).to_owned(), Progress::Finished(*outcome)))
                    .collect(),
                ..Self::default()
            }
        }

        fn written(&self) -> Vec<(String, f64)> {
            self.writes.borrow().clone()
        }
    }

    impl ParamWriter for Link {
        type Handle = usize;

        fn write(&self, name: &str, value: f64) -> Option<usize> {
            let mut writes = self.writes.borrow_mut();
            writes.push((name.to_owned(), value));
            Some(writes.len() - 1)
        }

        fn progress(&self, handle: usize) -> Progress {
            if handle >= READ {
                return Progress::Finished(RequestOutcome::TimedOut);
            }
            let writes = self.writes.borrow();
            let Some((name, _)) = writes.get(handle) else {
                return Progress::Lost;
            };
            self.outcomes
                .get(name)
                .copied()
                .unwrap_or(Progress::Finished(RequestOutcome::Accepted { value: None }))
        }
    }

    impl ParamReader for Link {
        fn read(&self, name: &str) -> Option<usize> {
            let mut reads = self.reads.borrow_mut();
            reads.push(name.to_owned());
            Some(READ + reads.len() - 1)
        }
    }

    /// Ticks until Write Params, Refresh Screen and the writes have ended, or a question waits.
    fn settle(page: &mut ExtendedTuning<usize>, link: &Link, vehicle: &Activation) {
        for _ in 0..500 {
            page.tick(link, vehicle, true, true, Instant::now(), bundled);
            if page.large().is_some()
                || (!page.writing() && !page.refreshing() && page.writes_pending() == 0)
            {
                return;
            }
        }
    }

    // -----------------------------------------------------------------------------------------
    // The C#'s files.
    // -----------------------------------------------------------------------------------------

    fn csharp(file: &str) -> Option<String> {
        crate::config_coverage::source::csharp(&format!("GCSViews/ConfigurationView/{file}"))
    }

    /// The strings quoted in a piece of C#.
    fn quoted(text: &str) -> Vec<String> {
        text.split('"')
            .skip(1)
            .step_by(2)
            .map(ToOwned::to_owned)
            .collect()
    }

    /// The text between the `(` at `open` and the `)` that closes it.
    fn inside(source: &str, open: usize) -> &str {
        let mut depth = 0;
        let mut quote = false;
        for (at, c) in source[open..].char_indices() {
            match c {
                '"' => quote = !quote,
                '(' if !quote => depth += 1,
                ')' if !quote => {
                    depth -= 1;
                    if depth == 0 {
                        return &source[open + 1..open + at];
                    }
                }
                _ => {}
            }
        }
        &source[open + 1..]
    }

    /// Every `this.X.Y += new System.EventHandler(this.H);` in the Designer: (X, Y, H).
    fn wirings(designer: &str) -> Vec<(String, String, String)> {
        designer
            .lines()
            .filter_map(|line| {
                let rest = line.trim().strip_prefix("this.")?;
                let (target, handler) = rest.split_once(" += new System.EventHandler(this.")?;
                let (control, event) = target.split_once('.')?;
                let handler = handler.strip_suffix(");")?;
                Some((control.to_owned(), event.to_owned(), handler.to_owned()))
            })
            .collect()
    }

    // -----------------------------------------------------------------------------------------
    // The page is the Designer's.
    // -----------------------------------------------------------------------------------------

    /// Every control this page draws is at the `Location` and `Size` its `.resx` gives, in the
    /// parent it gives, with its text.
    #[test]
    fn every_control_is_where_the_resx_puts_it() {
        let Some(text) = csharp("ConfigArducopter.resx") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let resx = crate::config_coverage::source::resx(&text);
        let get = |key: &str| resx.get(key).map(String::as_str);
        let pair = |name: &str, property: &str| -> (f32, f32) {
            let value =
                get(&format!("{name}.{property}")).unwrap_or_else(|| panic!("{name}.{property}"));
            let (a, b) = value.split_once(',').expect("a pair");
            (a.trim().parse().unwrap(), b.trim().parse().unwrap())
        };
        let geometry = |name: &str| {
            let ((x, y), (width, height)) = (pair(name, "Location"), pair(name, "Size"));
            (x, y, width, height)
        };
        let parent = |name: &str| get(&format!("&gt;&gt;{name}.Parent"));
        let in_group = |group: Option<&'static str>| Some(group.unwrap_or("$this"));
        assert_eq!(get("$this.Size"), Some("729, 673"));
        assert_eq!((PAGE.0, PAGE.1), pair("$this", "Size"));
        for spec in GROUPS {
            assert_eq!(geometry(spec.name), spec.at, "{}", spec.name);
            assert_eq!(get(&format!("{}.Text", spec.name)), Some(spec.caption));
            assert_eq!(parent(spec.name), Some("$this"), "{}", spec.name);
        }
        for spec in LABELS {
            assert_eq!(pair(spec.name, "Location"), spec.at, "{}", spec.name);
            assert_eq!(get(&format!("{}.Text", spec.name)), Some(spec.text));
            assert_eq!(parent(spec.name), in_group(spec.group), "{}", spec.name);
        }
        for spec in NUMBERS {
            assert_eq!(geometry(spec.name), spec.at, "{}", spec.name);
            assert_eq!(parent(spec.name), in_group(spec.group), "{}", spec.name);
            assert_eq!(get(&format!("{}.Enabled", spec.name)), Some("False"));
        }
        for spec in COMBOS {
            assert_eq!(geometry(spec.name), spec.at, "{}", spec.name);
            assert_eq!(parent(spec.name), in_group(spec.group), "{}", spec.name);
            assert_eq!(get(&format!("{}.Enabled", spec.name)), Some("False"));
        }
        for spec in BUTTONS {
            assert_eq!(geometry(spec.name), spec.at, "{}", spec.name);
            assert_eq!(get(&format!("{}.Text", spec.name)), Some(spec.text));
            assert_eq!(
                get(&format!("{}.Visible", spec.name)) != Some("False"),
                spec.visible,
                "{}",
                spec.name
            );
        }
        assert_eq!(geometry("CHK_lockrollpitch"), LOCK_AT);
        assert_eq!(get("CHK_lockrollpitch.Text"), Some(LOCK_TEXT));
        assert_eq!(geometry("lblUnitWarning"), WARNING_AT);
        assert_eq!(get("lblUnitWarning.Text"), Some(WARNING_TEXT));
        assert_eq!(get("lblUnitWarning.Visible"), Some("False"));
        assert_eq!(
            get("lblUnitWarning.Font"),
            Some("Microsoft Sans Serif, 8.25pt, style=Bold")
        );
    }

    /// Every control the Designer declares is one of the page's tables, of the kind the table
    /// holds, and every table entry is one the Designer declares: the page draws the Designer's
    /// controls, all of them.
    #[test]
    fn every_designer_control_is_drawn() {
        let Some(designer) = csharp("ConfigArducopter.Designer.cs") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let mut declared = Vec::new();
        for line in designer.lines() {
            let Some(rest) = line.trim().strip_prefix("this.") else {
                continue;
            };
            let Some((name, kind)) = rest.split_once(" = new ") else {
                continue;
            };
            if name.contains('.') || name == "components" {
                continue;
            }
            let kind = kind.split('(').next().unwrap_or_default();
            let kind = kind.rsplit('.').next().unwrap_or_default();
            let ours = match kind {
                "MavlinkNumericUpDown" => number_index(name).is_some(),
                "MavlinkComboBox" => combo_index(name).is_some(),
                "GroupBox" => GROUPS.iter().any(|spec| spec.name == name),
                "Label" => name == "lblUnitWarning" || LABELS.iter().any(|spec| spec.name == name),
                "MyButton" => BUTTONS.iter().any(|spec| spec.name == name),
                "CheckBox" => name == "CHK_lockrollpitch",
                // The tool tips `Activate` sets, drawn on each control that has one.
                "ToolTip" => name == "toolTip1",
                _ => false,
            };
            assert!(ours, "{name} ({kind}) is not drawn");
            declared.push(name.to_owned());
        }
        let tables = NUMBERS
            .iter()
            .map(|spec| spec.name)
            .chain(COMBOS.iter().map(|spec| spec.name))
            .chain(GROUPS.iter().map(|spec| spec.name))
            .chain(LABELS.iter().map(|spec| spec.name))
            .chain(BUTTONS.iter().map(|spec| spec.name))
            .chain(["CHK_lockrollpitch", "lblUnitWarning", "toolTip1"]);
        let mut count = 0;
        for name in tables {
            assert!(declared.iter().any(|held| held == name), "{name}");
            count += 1;
        }
        assert_eq!(count, declared.len());
        assert_eq!((NUMBERS.len(), COMBOS.len(), GROUPS.len()), (59, 9, 16));
    }

    /// Every event the Designer wires - 128 - is handled here, or is the hidden button's.
    #[test]
    fn every_wiring_is_handled_or_is_the_hidden_buttons() {
        let Some(designer) = csharp("ConfigArducopter.Designer.cs") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let wired = wirings(&designer);
        assert_eq!(wired.len(), 128);
        for (control, event, handler) in &wired {
            let handled = match (event.as_str(), handler.as_str()) {
                // `numeric_ValueUpdated` → `value_updated`.
                ("ValueUpdated", "numeric_ValueUpdated") => {
                    number_index(control).is_some()
                        || combo_index(control)
                            .and_then(|index| COMBOS.get(index))
                            .is_some_and(|spec| spec.value_updated)
                }
                // `OnEnter_NumUpDown` → `enter`, from `begin`.
                ("Enter", "OnEnter_NumUpDown") => number_index(control).is_some(),
                ("Click", "BUT_writePIDS_Click") => control == "BUT_writePIDS",
                ("Click", "BUT_refreshpart_Click") => control == "BUT_refreshpart",
                // Dimmed: the `.resx` hides the button and nothing shows it.
                ("Click", "BUT_rerequestparams_Click") => {
                    control == "BUT_rerequestparams"
                        && BUTTONS
                            .iter()
                            .any(|spec| spec.name == control && !spec.visible)
                }
                _ => false,
            };
            assert!(handled, "{control}.{event} += {handler}");
        }
        // A combo the Designer does not wire writes its parameter itself.
        for spec in COMBOS {
            let has_event = wired
                .iter()
                .any(|(control, event, _)| control == spec.name && event == "ValueUpdated");
            assert_eq!(has_event, spec.value_updated, "{}", spec.name);
        }
        // Every box has both of its events.
        for spec in NUMBERS {
            for event in ["ValueUpdated", "Enter"] {
                assert!(
                    wired
                        .iter()
                        .any(|(control, held, _)| control == spec.name && held == event),
                    "{}.{event}",
                    spec.name
                );
            }
        }
        // And the handlers are here.
        let source = include_str!("extended_tuning.rs");
        for name in [
            "fn value_updated(",
            "fn enter(",
            "fn write_params(",
            "fn refresh(",
        ] {
            assert!(source.contains(name), "{name}");
        }
    }

    /// Each box and combo is set up with the arguments and the names `Activate` passes, the names
    /// in the order the C# lists them; each IMAX's `if` names the pair that picks its call.
    #[test]
    fn activate_binds_the_names_the_csharp_tries_in_order() {
        let Some(source) = csharp("ConfigArducopter.cs") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let lines: Vec<&str> = source.lines().collect();
        // Every `X.setup(...)`: the control, the line, the text between the brackets.
        let mut calls = Vec::new();
        for (offset, _) in source.match_indices(".setup(") {
            let start = source[..offset]
                .rfind(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
                .map_or(0, |at| at + 1);
            let control = &source[start..offset];
            let line = u32::try_from(source[..offset].matches('\n').count() + 1).unwrap();
            let inner = inside(&source, offset + ".setup".len());
            calls.push((control.to_owned(), line, inner.to_owned()));
        }
        let call = |control: &str, line: u32| {
            calls
                .iter()
                .find(|(held, at, _)| held == control && *at == line)
                .map(|(_, _, inner)| inner.as_str())
                .unwrap_or_else(|| panic!("{control}.setup at line {line}"))
        };
        let numbers = |inner: &str| -> Setup {
            let values: Vec<f32> = inner
                .split(',')
                .take(4)
                .map(|value| value.trim().trim_end_matches('f').parse().unwrap())
                .collect();
            Setup {
                minimum: values[0],
                maximum: values[1],
                scale: values[2],
                increment: values[3],
            }
        };
        let names = |inner: &str| -> Vec<String> {
            let open = inner.find('{').expect("a names array");
            let close = inner[open..].find('}').expect("its end") + open;
            quoted(&inner[open..close])
        };
        let check = |control: &str, line: u32, how: Setup, list: &[&str]| {
            let inner = call(control, line);
            assert_eq!(numbers(inner), how, "{control} at {line}");
            assert_eq!(names(inner), list, "{control} at {line}");
        };
        for spec in NUMBERS {
            match spec.bind {
                Bind::Names(how, list) => check(spec.name, spec.line, how, list),
                Bind::Imax {
                    when,
                    then,
                    otherwise,
                } => {
                    let condition = lines[spec.line as usize - 1].trim_start();
                    assert!(condition.starts_with("if ("), "{condition}");
                    assert_eq!(quoted(condition), when, "{}", spec.name);
                    check(spec.name, spec.line + 1, then.0, then.1);
                    assert!(lines[spec.line as usize + 1].trim() == "else");
                    check(spec.name, spec.line + 3, otherwise.0, otherwise.1);
                }
            }
        }
        for spec in COMBOS {
            let inner = call(spec.name, spec.line);
            match spec.bind {
                ComboBind::Options(name) => {
                    assert!(inner.contains(&format!("GetParameterOptionsInt(\"{name}\"")));
                    assert!(inner.contains(&format!("\"{name}\", MainV2.comPort.MAV.param")));
                }
                ComboBind::Names(list) => assert_eq!(names(inner), list, "{}", spec.name),
            }
        }
        // Every `setup` call in the file is one of these: the 59 boxes, the IMAXes' other
        // three, and the nine combos.
        assert_eq!(calls.len(), NUMBERS.len() + 3 + COMBOS.len());
    }

    /// Refresh Screen reads the boxes in the order the page's `Controls` hold them.
    #[test]
    fn the_controls_order_is_the_designers() {
        let Some(designer) = csharp("ConfigArducopter.Designer.cs") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let mut top = Vec::new();
        let mut children: BTreeMap<String, Vec<String>> = BTreeMap::new();
        for line in designer.lines() {
            let Some(rest) = line.trim().strip_prefix("this.") else {
                continue;
            };
            let Some((parent, child)) = rest.split_once(".Add(this.") else {
                continue;
            };
            let child = child.trim_end_matches(");").to_owned();
            match parent.strip_suffix(".Controls") {
                Some(group) => children.entry(group.to_owned()).or_default().push(child),
                None if parent == "Controls" => top.push(child),
                None => {}
            }
        }
        let order: Vec<String> = top
            .iter()
            .flat_map(|name| {
                children
                    .get(name)
                    .cloned()
                    .unwrap_or_else(|| vec![name.clone()])
            })
            .filter(|name| number_index(name).is_some())
            .collect();
        assert_eq!(order, CONTROLS_ORDER);
    }

    /// The 4.7 table is `ParamChanges47`'s, entry for entry.
    #[test]
    fn the_47_table_is_the_csharps() {
        let Some(source) =
            crate::config_coverage::source::csharp("ExtLibs/Utilities/paramchanges47.cs")
        else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let entries: Vec<(String, String)> = source
            .lines()
            .filter(|line| line.trim_start().starts_with("{ \""))
            .map(|line| {
                let strings = quoted(line);
                (strings[0].clone(), strings[1].clone())
            })
            .collect();
        let ours: Vec<(String, String)> = CHANGES_47
            .iter()
            .map(|(new, warning)| ((*new).to_owned(), (*warning).to_owned()))
            .collect();
        assert_eq!(ours, entries);
        assert_eq!(ours.len(), 95);
    }

    // -----------------------------------------------------------------------------------------
    // Activate.
    // -----------------------------------------------------------------------------------------

    /// SITL's copter: each box bound to the first of its names the vehicle has, the eleven it
    /// has none of disabled; the combos to their `RCn_OPTION`s.
    #[test]
    fn activate_binds_the_sitl_copter() {
        let parameters = sitl();
        let page = opened(&parameters);
        assert!(page.is_active() && page.enabled());
        #[rustfmt::skip]
        let expected = [
            ("TUNE_LOW", "TUNE_MIN", true), ("TUNE_HIGH", "TUNE_MAX", true),
            ("HLD_LAT_P", "HLD_LAT_P", false),
            ("LOITER_LAT_D", "PSC_NE_VEL_D", true), ("LOITER_LAT_I", "PSC_NE_VEL_I", true),
            ("LOITER_LAT_IMAX", "PSC_NE_VEL_IMAX", true), ("LOITER_LAT_P", "PSC_NE_VEL_P", true),
            ("RATE_PIT_P", "ATC_RAT_PIT_P", true), ("RATE_PIT_I", "ATC_RAT_PIT_I", true),
            ("RATE_PIT_D", "ATC_RAT_PIT_D", true), ("RATE_PIT_IMAX", "ATC_RAT_PIT_IMAX", true),
            ("RATE_PIT_FILT", "ATC_RAT_PIT_FLTE", true), ("ATC_RAT_PIT_FLTD", "ATC_RAT_PIT_FLTD", true),
            ("ATC_RAT_PIT_FLTT", "ATC_RAT_PIT_FLTT", true),
            ("RATE_RLL_P", "ATC_RAT_RLL_P", true), ("RATE_RLL_I", "ATC_RAT_RLL_I", true),
            ("RATE_RLL_D", "ATC_RAT_RLL_D", true), ("RATE_RLL_IMAX", "ATC_RAT_RLL_IMAX", true),
            ("RATE_RLL_FILT", "ATC_RAT_RLL_FLTE", true), ("ATC_RAT_RLL_FLTD", "ATC_RAT_RLL_FLTD", true),
            ("ATC_RAT_RLL_FLTT", "ATC_RAT_RLL_FLTT", true),
            ("RATE_YAW_P", "ATC_RAT_YAW_P", true), ("RATE_YAW_I", "ATC_RAT_YAW_I", true),
            ("RATE_YAW_D", "ATC_RAT_YAW_D", true), ("RATE_YAW_IMAX", "ATC_RAT_YAW_IMAX", true),
            ("RATE_YAW_FILT", "ATC_RAT_YAW_FLTE", true), ("ATC_RAT_YAW_FLTD", "ATC_RAT_YAW_FLTD", true),
            ("ATC_RAT_YAW_FLTT", "ATC_RAT_YAW_FLTT", true),
            ("STB_PIT_P", "ATC_ANG_PIT_P", true), ("STB_RLL_P", "ATC_ANG_RLL_P", true),
            ("STB_YAW_P", "ATC_ANG_YAW_P", true),
            ("THR_ACCEL_P", "PSC_D_ACC_P", true), ("THR_ACCEL_I", "PSC_D_ACC_I", true),
            ("THR_ACCEL_D", "PSC_D_ACC_D", true), ("THR_ACCEL_IMAX", "PSC_D_ACC_IMAX", true),
            ("THR_ALT_P", "PSC_D_POS_P", true), ("THR_RATE_P", "PSC_D_VEL_P", true),
            ("WPNAV_LOIT_SPEED", "LOIT_SPEED_MS", true), ("WPNAV_RADIUS", "WP_RADIUS_M", true),
            ("WPNAV_SPEED", "WP_SPD", true), ("WPNAV_SPEED_DN", "WP_SPD_DN", true),
            ("WPNAV_SPEED_UP", "WP_SPD_UP", true),
            ("INS_GYRO_FILTER", "INS_GYRO_FILTER", true), ("INS_ACCEL_FILTER", "INS_ACCEL_FILTER", true),
            ("INS_LOG_BAT_OPT", "INS_LOG_BAT_OPT", true),
            ("INS_NOTCH_FREQ", "INS_NOTCH_FREQ", false), ("INS_NOTCH_BW", "INS_NOTCH_BW", false),
            ("INS_NOTCH_ATT", "INS_NOTCH_ATT", false),
            ("INS_HNTCH_MODE", "INS_HNTCH_MODE", false), ("INS_HNTCH_REF", "INS_HNTCH_REF", false),
            ("INS_HNTCH_FREQ", "INS_HNTCH_FREQ", false), ("INS_HNTCH_ATT", "INS_HNTCH_ATT", false),
            ("INS_HNTCH_BW", "INS_HNTCH_BW", false), ("INS_HNTCH_OPTS", "INS_HNTCH_OPTS", false),
            ("INS_HNTCH_HMNCS", "INS_HNTCH_HMNCS", false),
            ("mavlinkNumericUpDownatc_accel_r_max", "ATC_ACC_R_MAX", true),
            ("mavlinkNumericUpDownatc_accel_p_max", "ATC_ACC_P_MAX", true),
            ("mavlinkNumericUpDownatc_accel_y_max", "ATC_ACC_Y_MAX", true),
            ("mavlinkNumericUpDownatc_input_tc", "ATC_INPUT_TC", true),
        ];
        assert_eq!(expected.len(), NUMBERS.len());
        for (name, param, enabled) in expected {
            assert_eq!(param_of(&page, name), param, "{name}");
            assert_eq!(page.number(at(name)).unwrap().enabled, enabled, "{name}");
        }
        assert_eq!(shown(&page, "RATE_RLL_P"), "0.135");
        assert_eq!(shown(&page, "RATE_PIT_P"), "0.135");
        assert_eq!(shown(&page, "INS_GYRO_FILTER"), "20");
        #[rustfmt::skip]
        let combos = [
            ("TUNE", "TUNE", true, Some(0)), ("CH6_OPTION", "RC6_OPTION", true, Some(0)),
            ("CH7_OPTION", "RC7_OPTION", true, Some(7)), ("CH8_OPTION", "RC8_OPTION", true, Some(0)),
            ("CH9_OPTION", "RC9_OPTION", true, Some(0)), ("CH10_OPTION", "RC10_OPTION", true, Some(0)),
            // A bitmask: the documentation lists no values, so nothing is selected.
            ("INS_LOG_BAT_MASK", "INS_LOG_BAT_MASK", true, None),
            // Not on this vehicle: left as the Designer made it, named for itself.
            ("INS_NOTCH_ENABLE", "INS_NOTCH_ENABLE", false, None),
            ("INS_HNTCH_ENABLE", "INS_HNTCH_ENABLE", true, Some(0)),
        ];
        for (name, param, enabled, selected) in combos {
            let combo = page.combo(combo_at(name)).unwrap();
            assert_eq!(
                page.name_of(Control::Combo(combo_at(name))),
                param,
                "{name}"
            );
            assert_eq!(combo.enabled, enabled, "{name}");
            assert_eq!(combo.selected, selected, "{name}");
        }
        assert_eq!(
            page.combo(combo_at("CH7_OPTION")).unwrap().text(),
            "Save WP"
        );
        // Roll and pitch alike, and no swashplate: the lock stays checked.
        assert!(page.locked());
        assert!(page.changes().is_empty());
    }

    /// `setup` binds the first name the vehicle has; with none, the first, disabled.
    #[test]
    fn the_first_name_the_vehicle_has_is_the_one_bound() {
        let page = opened(&table(&[("RATE_RLL_P", 0.1), ("ATC_RAT_RLL_P", 0.2)]));
        assert_eq!(param_of(&page, "RATE_RLL_P"), "RATE_RLL_P");
        let page = opened(&table(&[("ATC_RAT_RLL_P", 0.2), ("Q_A_RAT_RLL_P", 0.3)]));
        assert_eq!(param_of(&page, "RATE_RLL_P"), "ATC_RAT_RLL_P");
        let page = opened(&table(&[("Q_A_RAT_RLL_P", 0.3)]));
        assert_eq!(param_of(&page, "RATE_RLL_P"), "Q_A_RAT_RLL_P");
        let page = opened(&table(&[]));
        assert_eq!(param_of(&page, "RATE_RLL_P"), "RATE_RLL_P");
        assert!(!page.number(at("RATE_RLL_P")).unwrap().enabled);
        assert_eq!(chosen(&["A", "B"], &table(&[("B", 1.0)])), "B");
        assert_eq!(chosen(&["A", "B"], &[]), "A");
        // A combo with a names list: the first the vehicle has.
        let page = opened(&table(&[("CH6_OPT", 0.0), ("RC6_OPTION", 7.0)]));
        assert_eq!(
            page.name_of(Control::Combo(combo_at("CH6_OPTION"))),
            "CH6_OPT"
        );
    }

    /// A rate IMAX is the 3.4 name, shown as it is, when the vehicle has one; the old name,
    /// shown a tenth, otherwise - and a change is written back tenfold.
    #[test]
    fn a_rate_imax_takes_its_names_and_scale_from_the_34_names() {
        let old = table(&[("RATE_PIT_IMAX", 4000.0)]);
        let spec = NUMBERS[at("RATE_PIT_IMAX")];
        assert_eq!(
            spec.bind.call(&old),
            (TENTHS, &["RATE_PIT_IMAX", "RATE_PIT_IMAX"][..])
        );
        let mut page = opened(&old);
        assert!(
            !page.locked(),
            "roll's IMAX, which this vehicle lacks, differs"
        );
        assert_eq!(param_of(&page, "RATE_PIT_IMAX"), "RATE_PIT_IMAX");
        assert_eq!(shown(&page, "RATE_PIT_IMAX"), "400");
        type_into(&mut page, "RATE_PIT_IMAX", "300");
        assert_eq!(page.changes(), changes(&[("RATE_PIT_IMAX", 3000.0)]));

        let page = opened(&table(&[("ATC_RAT_PIT_IMAX", 0.5)]));
        assert_eq!(param_of(&page, "RATE_PIT_IMAX"), "ATC_RAT_PIT_IMAX");
        assert_eq!(page.number(at("RATE_PIT_IMAX")).unwrap().written(), 0.5);
        let page = opened(&table(&[("Q_A_RAT_PIT_IMAX", 0.4)]));
        assert_eq!(param_of(&page, "RATE_PIT_IMAX"), "Q_A_RAT_PIT_IMAX");
    }

    /// Activate unchecks the lock when roll and pitch differ, or for a helicopter; it never
    /// checks it again.
    #[test]
    fn activate_unchecks_the_lock_when_roll_and_pitch_differ_or_for_a_heli() {
        assert!(opened(&old_copter()).locked());
        let mut differ = old_copter();
        differ.push(("RATE_PIT_I".to_owned(), 0.2));
        differ.retain(|(name, value)| name != "RATE_PIT_I" || *value == 0.2);
        assert!(!opened(&differ).locked());
        let mut heli = sitl();
        heli.push(("H_SWASH_TYPE".to_owned(), 0.0));
        assert!(!opened(&heli).locked());
        let parameters = sitl();
        let mut page = opened(&parameters);
        page.toggle_lock(Instant::now());
        page.activate(&copter(&parameters), bundled);
        assert!(!page.locked());
    }

    /// No link, or a plane without `Q_ENABLE`: the page disabled, and nothing on it acts. A
    /// quadplane binds the `Q_` names.
    #[test]
    fn the_page_is_disabled_without_a_link_or_for_a_plane_without_q_enable() {
        let parameters = sitl();
        let mut page = ExtendedTuning::<usize>::default();
        page.activate(
            &Activation {
                connected: false,
                ..copter(&parameters)
            },
            bundled,
        );
        assert!(page.is_active() && !page.enabled());
        assert!(!page.number(at("RATE_RLL_P")).unwrap().enabled);
        page.write_params();
        assert!(!page.writing());
        page.toggle_lock(Instant::now());
        assert!(page.locked());
        page.refresh(true);
        assert!(!page.refreshing());

        let plane = |parameters| Activation {
            parameters,
            key: key(),
            connected: true,
            firmware: Firmware::ArduPlane,
        };
        let mut page = ExtendedTuning::<usize>::default();
        let without = table(&[("Q_ENABLE", 0.0), ("Q_A_RAT_RLL_P", 0.25)]);
        page.activate(&plane(&without), bundled);
        assert!(!page.enabled());
        let quadplane = table(&[
            ("Q_ENABLE", 1.0),
            ("Q_A_RAT_RLL_P", 0.25),
            ("Q_A_RAT_PIT_P", 0.25),
            ("Q_A_RAT_RLL_IMAX", 0.5),
            ("Q_A_RAT_PIT_IMAX", 0.5),
            ("Q_P_D_POS_P", 1.0),
        ]);
        let mut page = ExtendedTuning::<usize>::default();
        page.activate(&plane(&quadplane), bundled);
        assert!(page.enabled());
        assert_eq!(param_of(&page, "RATE_RLL_P"), "Q_A_RAT_RLL_P");
        assert_eq!(param_of(&page, "RATE_RLL_IMAX"), "Q_A_RAT_RLL_IMAX");
        assert_eq!(param_of(&page, "THR_ALT_P"), "Q_P_D_POS_P");
    }

    /// The controls inside the group boxes get `ParamName` and its documentation; the ones on the
    /// page itself get none - the C#'s loop goes one level into `Controls`.
    #[test]
    fn activate_gives_the_controls_in_group_boxes_their_tool_tips() {
        let page = opened(&sitl());
        let description = |name: &str| bundled(name).map_or("", |meta| meta.description);
        assert_eq!(
            page.tip("RATE_RLL_P"),
            Some(format!("ATC_RAT_RLL_P:\n{}", description("ATC_RAT_RLL_P")).as_str())
        );
        assert_eq!(
            page.tip("HLD_LAT_P"),
            Some(format!("HLD_LAT_P:\n{}", description("HLD_LAT_P")).as_str())
        );
        // A combo whose names the vehicle lacks has no `ParamName`.
        assert_eq!(page.tip("INS_NOTCH_ENABLE"), Some(":\n"));
        for name in ["TUNE", "TUNE_LOW", "TUNE_HIGH", "CH6_OPTION", "CH10_OPTION"] {
            assert_eq!(page.tip(name), None, "{name}");
        }
        assert_eq!(page.tips.len(), 57 + 3);
    }

    // -----------------------------------------------------------------------------------------
    // The handlers.
    // -----------------------------------------------------------------------------------------

    /// A typed change goes into `changes` and marks the box; nothing is written.
    #[test]
    fn a_change_goes_into_changes_and_marks_the_box() {
        let mut page = opened(&sitl());
        type_into(&mut page, "RATE_RLL_P", "0.14");
        assert_eq!(shown(&page, "RATE_RLL_P"), "0.140");
        assert_eq!(page.changes(), changes(&[("ATC_RAT_RLL_P", 0.14)]));
        assert!(page.marked("ATC_RAT_RLL_P"));
        assert!(page.writes_pending() == 0 && !page.writing());
        // The arrows too: one increment, the documented 0.005.
        page.step(at("RATE_RLL_P"), false, Instant::now(), false);
        assert_eq!(page.changes(), changes(&[("ATC_RAT_RLL_P", 0.135)]));
        // A box left with typing in it is read as it loses the focus - hidden with the page.
        page.begin(at("RATE_YAW_P"), Instant::now(), false);
        page.key(&press("u", None, true), Instant::now());
        for typed in ["0", ".", "4"] {
            page.key(&press(typed, Some(typed.to_owned()), false), Instant::now());
        }
        page.hide(Instant::now());
        assert_eq!(
            page.changes().last(),
            Some(&("ATC_RAT_YAW_P".to_owned(), 0.4))
        );
    }

    /// `startup`, true from the constructor and while `Activate` runs, drops every change.
    #[test]
    fn a_change_while_starting_up_is_not_taken() {
        assert!(ExtendedTuning::<usize>::default().startup);
        let mut page = opened(&sitl());
        assert!(!page.startup);
        page.startup = true;
        type_into(&mut page, "RATE_RLL_P", "0.14");
        assert!(page.changes().is_empty());
        assert!(!page.marked("ATC_RAT_RLL_P"));
    }

    /// A typed value above the maximum asks first; Yes takes it into `changes`, No the maximum.
    #[test]
    fn a_value_above_the_maximum_asks_first() {
        let mut page = opened(&sitl());
        type_into(&mut page, "RATE_RLL_P", "0.9");
        assert_eq!(
            page.question().map(Question::text).as_deref(),
            Some("ATC_RAT_RLL_P Value out of range\nDo you want to accept the new value?")
        );
        assert!(page.changes().is_empty());
        page.answer(true, Instant::now());
        assert_eq!(page.changes(), changes(&[("ATC_RAT_RLL_P", 0.9)]));
        let mut page = opened(&sitl());
        type_into(&mut page, "RATE_RLL_P", "0.9");
        page.answer(false, Instant::now());
        assert_eq!(page.changes(), changes(&[("ATC_RAT_RLL_P", 0.5)]));
    }

    /// With the lock checked, a roll `RATE_` or `STB_` change is typed into its pitch box and
    /// back, both go into `changes` and both are marked - the pair's by its own `ValueUpdated`,
    /// at its own scale.
    #[test]
    fn the_lock_copies_roll_to_pitch_and_pitch_to_roll_on_the_old_names() {
        let mut page = opened(&old_copter());
        assert!(page.locked());
        type_into(&mut page, "RATE_RLL_P", "0.1");
        assert_eq!(shown(&page, "RATE_PIT_P"), "0.100");
        assert_eq!(
            page.changes(),
            changes(&[("RATE_RLL_P", 0.1), ("RATE_PIT_P", 0.1)])
        );
        assert!(page.marked("RATE_RLL_P") && page.marked("RATE_PIT_P"));
        type_into(&mut page, "STB_PIT_P", "4");
        assert_eq!(shown(&page, "STB_RLL_P"), "4.000");
        assert!(page.changes().contains(&("STB_PIT_P".to_owned(), 4.0)));
        assert!(page.changes().contains(&("STB_RLL_P".to_owned(), 4.0)));
        type_into(&mut page, "RATE_RLL_IMAX", "400");
        assert_eq!(shown(&page, "RATE_PIT_IMAX"), "400");
        assert!(
            page.changes()
                .contains(&("RATE_RLL_IMAX".to_owned(), 4000.0))
        );
        assert!(
            page.changes()
                .contains(&("RATE_PIT_IMAX".to_owned(), 4000.0))
        );
        // A pair whose own maximum is below the value typed into it is held at that maximum, which
        // is the value it has: nothing changes, so it raises nothing - and `changes` keeps what
        // the pairing put there. (With no documented range the maximum is the vehicle's value.)
        let mut parameters = old_copter();
        for (name, value) in &mut parameters {
            if name == "RATE_PIT_D" {
                *value = 0.002;
            }
        }
        let mut page = opened(&parameters);
        assert!(!page.locked(), "Activate unchecks it: the D gains differ");
        page.toggle_lock(Instant::now());
        type_into(&mut page, "RATE_RLL_D", "0.003");
        assert!(page.question().is_none());
        assert_eq!(shown(&page, "RATE_RLL_D"), "0.0030");
        assert_eq!(shown(&page, "RATE_PIT_D"), "0.0020");
        assert_eq!(
            page.changes(),
            changes(&[("RATE_RLL_D", 0.003), ("RATE_PIT_D", 0.003)])
        );
    }

    /// Unchecked, a roll change stays on roll.
    #[test]
    fn unlocked_a_roll_change_stays_on_roll() {
        let mut page = opened(&old_copter());
        page.toggle_lock(Instant::now());
        assert!(!page.locked());
        type_into(&mut page, "RATE_RLL_P", "0.1");
        assert_eq!(shown(&page, "RATE_PIT_P"), "0.150");
        assert_eq!(page.changes(), changes(&[("RATE_RLL_P", 0.1)]));
    }

    /// The lock pairs only names that start `RATE_`, `STB_` or `ACRO_`: a vehicle's `ATC_RAT_`
    /// and `Q_A_` names are left alone, as the C#'s `StartsWith` leaves them.
    #[test]
    fn the_lock_pairs_only_rate_stb_and_acro_names() {
        let mut page = opened(&sitl());
        assert!(page.locked());
        type_into(&mut page, "RATE_RLL_P", "0.14");
        assert_eq!(shown(&page, "RATE_PIT_P"), "0.135");
        assert_eq!(page.changes(), changes(&[("ATC_RAT_RLL_P", 0.14)]));
        assert!(!page.marked("ATC_RAT_PIT_P"));
    }

    /// A `LOITER_LAT_` change goes to its `LOITER_LON_` name too, lock or no lock - there is no
    /// box of that name, so only `changes` has it - and Write Params writes both.
    #[test]
    fn a_loiter_lat_change_carries_to_loiter_lon() {
        let parameters = table(&[("LOITER_LAT_P", 1.0), ("LOITER_LON_P", 1.0)]);
        let mut page = opened(&parameters);
        page.toggle_lock(Instant::now());
        type_into(&mut page, "LOITER_LAT_P", "0.5");
        assert_eq!(
            page.changes(),
            changes(&[("LOITER_LAT_P", 0.5), ("LOITER_LON_P", 0.5)])
        );
        page.write_params();
        let link = Link::default();
        settle(&mut page, &link, &copter(&parameters));
        assert_eq!(
            link.written(),
            table(&[("LOITER_LAT_P", 0.5), ("LOITER_LON_P", 0.5)])
        );
        assert!(page.changes().is_empty());
        // A vehicle without the `LON` name: its null cast's box, and the rest still written.
        let parameters = table(&[("LOITER_LAT_P", 1.0)]);
        let mut page = opened(&parameters);
        type_into(&mut page, "LOITER_LAT_P", "0.5");
        page.write_params();
        let link = Link::default();
        settle(&mut page, &link, &copter(&parameters));
        assert_eq!(link.written(), table(&[("LOITER_LAT_P", 0.5)]));
        assert_eq!(page.message(), Some(&error("Set LOITER_LON_P Failed")));
        assert_eq!(page.changes(), changes(&[("LOITER_LON_P", 0.5)]));
    }

    /// The `OnEnter` warning: on 4.7 or later, a box whose parameter 4.7 renamed; hidden
    /// otherwise.
    #[test]
    fn entering_a_box_4_7_renamed_shows_the_change() {
        let mut page = opened(&sitl());
        let now = Instant::now();
        page.begin(at("mavlinkNumericUpDownatc_accel_r_max"), now, true);
        assert_eq!(
            page.warning(),
            Some(
                "Change in 4.7: ATC_ACCEL_R_MAX → ATC_ACC_R_MAX: units changed from cm/s/s to \
                 m/s/s"
            )
        );
        page.begin(at("RATE_RLL_P"), now, true);
        assert_eq!(page.warning(), None);
        page.step(at("WPNAV_SPEED"), true, now, true);
        assert_eq!(
            page.warning(),
            Some("Change in 4.7: WPNAV_SPEED → WP_SPD: units changed from cm/s to m/s")
        );
        page.begin(at("mavlinkNumericUpDownatc_accel_r_max"), now, false);
        assert_eq!(page.warning(), None);
        assert!(at_least_47(Some("ArduCopter V4.8.0-dev (cdec5df1)")));
        assert!(at_least_47(Some("ArduCopter V4.7.0 (abcdef12)")));
        assert!(!at_least_47(Some("ArduCopter V4.6.3 (abcdef12)")));
        assert!(!at_least_47(Some("no version here")));
        assert!(!at_least_47(None));
    }

    // -----------------------------------------------------------------------------------------
    // Write Params.
    // -----------------------------------------------------------------------------------------

    /// Write Params writes `changes` a name at a time, in the order changed, empties it and
    /// clears the colours.
    #[test]
    fn write_params_writes_changes_in_order_and_empties_them() {
        let parameters = sitl();
        let mut page = opened(&parameters);
        type_into(&mut page, "RATE_YAW_P", "0.35");
        type_into(&mut page, "RATE_RLL_P", "0.14");
        page.write_params();
        assert!(page.writing());
        let link = Link::default();
        settle(&mut page, &link, &copter(&parameters));
        assert_eq!(
            link.written(),
            [
                ("ATC_RAT_YAW_P".to_owned(), f64::from(0.35_f32)),
                ("ATC_RAT_RLL_P".to_owned(), f64::from(0.14_f32)),
            ]
        );
        assert!(page.changes().is_empty());
        assert!(!page.marked("ATC_RAT_YAW_P") && !page.marked("ATC_RAT_RLL_P"));
        assert!(page.message().is_none() && !page.writing());
    }

    /// Ctrl+S is Write Params.
    #[test]
    fn ctrl_s_is_write_params() {
        let mut page = opened(&sitl());
        type_into(&mut page, "RATE_RLL_P", "0.14");
        assert!(page.key(&press("s", Some("s".to_owned()), true), Instant::now()));
        assert!(page.writing());
    }

    /// A value more than double the vehicle's asks first. No types the vehicle's value back into
    /// the box - which puts it in `changes` - clears the colour and stops; Yes writes.
    #[test]
    fn a_value_more_than_double_asks_first() {
        let parameters = sitl();
        let vehicle = copter(&parameters);
        let mut page = opened(&parameters);
        type_into(&mut page, "RATE_RLL_P", "0.3");
        page.write_params();
        let link = Link::default();
        settle(&mut page, &link, &vehicle);
        assert_eq!(
            page.large(),
            Some(&Large {
                param: "ATC_RAT_RLL_P".to_owned(),
                value: 0.3
            })
        );
        assert_eq!(
            large_text("ATC_RAT_RLL_P"),
            "ATC_RAT_RLL_P has more than doubled the last input. Are you sure?"
        );
        page.answer_large(false, &vehicle, Instant::now());
        settle(&mut page, &link, &vehicle);
        assert!(link.written().is_empty());
        assert_eq!(shown(&page, "RATE_RLL_P"), "0.135");
        assert_eq!(page.changes(), changes(&[("ATC_RAT_RLL_P", 0.135)]));
        assert!(!page.marked("ATC_RAT_RLL_P"));
        assert!(!page.writing() && page.large().is_none());

        type_into(&mut page, "RATE_RLL_P", "0.3");
        page.write_params();
        settle(&mut page, &link, &vehicle);
        page.answer_large(true, &vehicle, Instant::now());
        settle(&mut page, &link, &vehicle);
        assert_eq!(
            link.written(),
            [("ATC_RAT_RLL_P".to_owned(), f64::from(0.3_f32))]
        );
        assert!(page.changes().is_empty());
    }

    /// A combo asked about keeps its choice on No: its list has no row of the vehicle's value's
    /// text.
    #[test]
    fn a_combo_answered_no_keeps_its_choice() {
        let parameters = sitl();
        let vehicle = copter(&parameters);
        let mut page = opened(&parameters);
        let ch8 = combo_at("CH8_OPTION");
        page.toggle_dropdown(ch8, Instant::now());
        assert!(page.choose(ch8, 9, Instant::now()).is_empty());
        assert_eq!(page.changes(), changes(&[("RC8_OPTION", 9.0)]));
        assert!(page.marked("RC8_OPTION"));
        page.write_params();
        let link = Link::default();
        settle(&mut page, &link, &vehicle);
        assert_eq!(
            page.large().map(|large| large.param.as_str()),
            Some("RC8_OPTION")
        );
        page.answer_large(false, &vehicle, Instant::now());
        assert_eq!(page.combo(ch8).unwrap().selected, Some(9));
        assert!(!page.marked("RC8_OPTION"));
        assert_eq!(page.changes(), changes(&[("RC8_OPTION", 9.0)]));
        assert!(link.written().is_empty() && !page.writing());
    }

    /// A timeout is the `catch`'s "Set NAME Failed" and leaves the name in `changes`; a `false`
    /// - the vehicle does not have the name - is not read, and the name goes.
    #[test]
    fn a_timeout_says_set_failed_and_keeps_the_name() {
        let parameters = sitl();
        let mut page = opened(&parameters);
        type_into(&mut page, "RATE_RLL_P", "0.14");
        type_into(&mut page, "RATE_YAW_P", "0.35");
        page.write_params();
        let link = Link::answering(&[
            ("ATC_RAT_RLL_P", RequestOutcome::TimedOut),
            ("ATC_RAT_YAW_P", RequestOutcome::UnknownParameter),
        ]);
        settle(&mut page, &link, &copter(&parameters));
        assert_eq!(link.written().len(), 2);
        assert_eq!(page.message(), Some(&error("Set ATC_RAT_RLL_P Failed")));
        page.dismiss_message();
        assert!(page.message().is_none());
        assert_eq!(page.changes(), changes(&[("ATC_RAT_RLL_P", 0.14)]));
        assert!(page.marked("ATC_RAT_RLL_P") && !page.marked("ATC_RAT_YAW_P"));
    }

    /// Without a link, Write Params says so and stops, writing nothing.
    #[test]
    fn without_a_link_write_params_says_so_and_stops() {
        let parameters = sitl();
        let mut page = opened(&parameters);
        type_into(&mut page, "RATE_RLL_P", "0.14");
        page.write_params();
        let link = Link::default();
        settle(
            &mut page,
            &link,
            &Activation {
                connected: false,
                ..copter(&parameters)
            },
        );
        assert_eq!(page.message(), Some(&error(NOT_CONNECTED)));
        assert!(link.written().is_empty() && !page.writing());
        assert_eq!(page.changes(), changes(&[("ATC_RAT_RLL_P", 0.14)]));
    }

    /// `CH9_OPTION` and `CH10_OPTION` have no `ValueUpdated`: each writes its parameter itself,
    /// failing with its own text. The other combos go into `changes`.
    #[test]
    fn ch9_and_ch10_write_at_once_and_the_other_combos_wait() {
        let parameters = sitl();
        let mut page = opened(&parameters);
        for name in ["CH9_OPTION", "CH10_OPTION"] {
            let index = combo_at(name);
            page.toggle_dropdown(index, Instant::now());
            assert_eq!(page.dropdown(), Some(index));
            let jobs = page.choose(index, 9, Instant::now());
            assert_eq!(jobs.len(), 1, "{name}");
            page.push(jobs);
        }
        assert!(page.changes().is_empty());
        let link = Link::answering(&[("RC9_OPTION", RequestOutcome::UnknownParameter)]);
        settle(&mut page, &link, &copter(&parameters));
        assert_eq!(
            link.written(),
            table(&[("RC9_OPTION", 9.0), ("RC10_OPTION", 9.0)])
        );
        assert_eq!(page.message(), Some(&error("Set RC9_OPTION Failed!")));
        for (name, param, key) in [
            ("TUNE", "TUNE", 1),
            ("CH7_OPTION", "RC7_OPTION", 9),
            ("INS_HNTCH_ENABLE", "INS_HNTCH_ENABLE", 1),
        ] {
            assert!(page.choose(combo_at(name), key, Instant::now()).is_empty());
            assert!(page.marked(param), "{name}");
        }
        assert_eq!(
            page.changes(),
            changes(&[
                ("TUNE", 1.0),
                ("RC7_OPTION", 9.0),
                ("INS_HNTCH_ENABLE", 1.0)
            ])
        );
    }

    /// A write goes through the link's retrying set as `setParam`, and the echo is what the page
    /// reports.
    #[test]
    fn a_write_goes_through_the_links_retrying_set() {
        use crate::telemetry::scripted::{Vehicle, param, until};
        use mp_link::ProtocolTimeouts;
        use mp_mavlink_dialects::all::MavMessage;
        let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default().faster(20));
        vehicle.send(&param("ATC_RAT_RLL_P", 0.135, 9));
        vehicle.send(&param("ATC_RAT_PIT_P", 0.135, 9));
        until("the parameters", || {
            telemetry.holds_parameter("ATC_RAT_RLL_P") && telemetry.holds_parameter("ATC_RAT_PIT_P")
        });
        let view = telemetry.view();
        let state = Activation {
            parameters: &view.parameters,
            key: Key::of(&view),
            connected: true,
            firmware: Firmware::ArduCopter2,
        };
        let mut page = ExtendedTuning::default();
        page.activate(&state, bundled);
        type_into(&mut page, "RATE_RLL_P", "0.14");
        page.write_params();
        let mut written = None;
        until("the PARAM_SET", || {
            page.tick(&telemetry, &state, true, true, Instant::now(), bundled);
            for message in vehicle.read() {
                if let MavMessage::ParamSet(set) = message {
                    written = Some(set.param_value);
                    vehicle.send(&param("ATC_RAT_RLL_P", set.param_value, 9));
                }
            }
            written.is_some()
        });
        assert_eq!(written, Some(0.14));
        until("the echo", || {
            let view = telemetry.view();
            let now = Activation {
                parameters: &view.parameters,
                ..state
            };
            page.tick(&telemetry, &now, true, true, Instant::now(), bundled);
            page.last_write() == Some("ATC_RAT_RLL_P 0.14 accepted")
        });
        assert!(page.changes().is_empty() && !page.writing());
        assert!(page.message().is_none());
    }

    // -----------------------------------------------------------------------------------------
    // Refresh Screen.
    // -----------------------------------------------------------------------------------------

    /// Refresh Screen reads every box's parameter by its `Name`, in the page's `Controls` order,
    /// none of the combos, then runs `Activate` on what came back.
    #[test]
    fn refresh_reads_every_box_by_its_name_then_activates() {
        let parameters = sitl();
        let mut page = opened(&parameters);
        page.refresh(false);
        assert!(!page.refreshing(), "not without a link");
        page.refresh(true);
        assert!(page.refreshing());
        let mut changed = parameters.clone();
        for (name, value) in &mut changed {
            if name == "ATC_RAT_RLL_P" {
                *value = 0.2;
            }
        }
        let link = Link::default();
        settle(&mut page, &link, &copter(&changed));
        assert!(!page.refreshing());
        let reads = link.reads.borrow().clone();
        let expected: Vec<String> = CONTROLS_ORDER
            .iter()
            .map(|designer| param_of(&page, designer))
            .collect();
        assert_eq!(reads, expected);
        assert_eq!(reads.first().map(String::as_str), Some("INS_NOTCH_ATT"));
        assert!(reads.iter().any(|name| name == "TUNE_MIN"));
        assert!(!reads.iter().any(|name| name == "RC7_OPTION"));
        assert_eq!(shown(&page, "RATE_RLL_P"), "0.200");
    }

    /// The page object goes with its screen: leaving CONFIG disposes of it, and the next is the
    /// Designer's again - the lock checked.
    #[test]
    fn the_page_object_goes_with_its_screen() {
        let parameters = sitl();
        let mut page = opened(&parameters);
        page.toggle_lock(Instant::now());
        page.hide(Instant::now());
        let link = Link::default();
        page.tick(
            &link,
            &copter(&parameters),
            false,
            false,
            Instant::now(),
            bundled,
        );
        assert!(page.locked());
        assert!(!page.number(at("RATE_RLL_P")).unwrap().enabled);
    }

    // -----------------------------------------------------------------------------------------
    // The rest.
    // -----------------------------------------------------------------------------------------

    /// `double.ToString()`'s text: fixed from 1E-04 to below 1E+15, the exponent form outside.
    #[test]
    fn double_text_is_the_csharps() {
        assert_eq!(double_text(0.135), "0.135");
        assert_eq!(double_text(7.0), "7");
        assert_eq!(double_text(0.0), "0");
        assert_eq!(double_text(-2.5), "-2.5");
        assert_eq!(double_text(0.0001), "0.0001");
        assert_eq!(double_text(0.00001), "1E-05");
        assert_eq!(double_text(0.000_015), "1.5E-05");
        assert_eq!(double_text(1e15), "1E+15");
    }

    /// Every fact the GUI script asserts is one this page records, and every control it clicks
    /// is one this page draws.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-extended-tuning.gui");
        let source = include_str!("extended_tuning.rs");
        let control = |rest: &str, suffixes: &[&str], known: &dyn Fn(&str) -> bool| {
            let (name, suffix) = rest
                .split_once('.')
                .map_or((rest, None), |(name, suffix)| (name, Some(suffix)));
            known(name) && suffix.is_none_or(|suffix| suffixes.contains(&suffix))
        };
        let (mut facts, mut clicks) = (0, 0);
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.extended.") => {
                    let rest = &key["config.extended.".len()..];
                    let recorded = source.contains(&format!("\"{key}\""))
                        || rest.strip_prefix("number.").is_some_and(|rest| {
                            control(rest, &["param", "enabled", "marked"], &|name| {
                                number_index(name).is_some()
                            })
                        })
                        || rest.strip_prefix("combo.").is_some_and(|rest| {
                            control(rest, &["text", "param", "enabled", "marked"], &|name| {
                                combo_index(name).is_some()
                            })
                        })
                        || rest.strip_prefix("tip.").is_some_and(|name| {
                            number_index(name).is_some() || combo_index(name).is_some()
                        });
                    assert!(recorded, "{key} is not recorded");
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("extended-") => {
                    let name = &id["extended-".len()..];
                    let row = name.rsplit_once('-').is_some_and(|(base, value)| {
                        combo_index(base).is_some() && value.parse::<i64>().is_ok()
                    });
                    let drawn = number_index(name).is_some()
                        || combo_index(name).is_some()
                        || row
                        || name == "CHK_lockrollpitch"
                        || BUTTONS.iter().any(|spec| spec.visible && spec.name == name)
                        || source.contains(&format!("\"{id}\""));
                    assert!(drawn, "{id} is not drawn");
                    clicks += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("config-page-") => {
                    let class = &id["config-page-".len()..];
                    assert!(
                        crate::setup::CONFIG_LIST
                            .iter()
                            .any(|entry| entry.class == class),
                        "{class} is not on the CONFIG list"
                    );
                }
                _ => {}
            }
        }
        assert!(facts > 50 && clicks >= 20, "{facts} facts, {clicks} clicks");
    }
}
