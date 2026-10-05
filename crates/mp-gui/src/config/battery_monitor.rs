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

//! Battery Monitor: `GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs`, a page of Initial
//! Setup's Optional Hardware list (`GCSViews/InitialSetup.cs:268-272`).
//!
//! What it shows: the monitor type (`BATT_MONITOR`, from the parameter's documented values), the
//! sensor (a fixed list of power modules, each a divider and an amps-per-volt), the board (which
//! analog pins the sensor is wired to, `BATT_VOLT_PIN` and `BATT_CURR_PIN`), the capacity, and a
//! Calibration box: a measured voltage and current typed in, the vehicle's own readings beside them,
//! and the divider and amps-per-volt those give - the voltage divider as
//! `measured x divider / reading`. A one-second timer refreshes the vehicle's readings.
//!
//! There is no save button, in the C# or here: each control writes when it changes, a text box
//! when it is left (`Validated`) - and the three calibration boxes on Enter too
//! (`PreviewKeyDown`). The C# calls `setParam` on the UI thread, one after another, each blocking
//! until the vehicle echoes or three retries go unanswered; here each handler's writes are a job,
//! the jobs are run one at a time in the order the handlers ran, and a write that times out ends
//! its job with the message the C#'s `catch` shows.
//!
//! Which parameter a number is written to is decided by asking the vehicle which names it has, the
//! C#'s `setParam(string[] names, value)`: the divider goes to `VOLT_DIVIDER` if the firmware still
//! has that name and `BATT_VOLT_MULT` otherwise, the amps-per-volt to `AMP_PER_VOLT`, else
//! `BATT_AMP_PERVOLT`, else `BATT_AMP_PERVLT`, three generations of ArduPilot. `Activate` reads
//! them in the opposite order, newest first, so the oldest name the vehicle has wins both ways.
//!
//! The layout is `ConfigBatteryMonitoring.resx`'s, every control at its `Location` in a 512 x 322
//! page. The colours are this application's. `pictureBox5`, a photograph of a power module held
//! in the `.resx`, is drawn, zoomed, from `Resources.BR_APMPWRDEAN_2`: the same bytes
//! ([`crate::pictures`]).
//!
//! The Sensor and HW Ver boxes are `DropDown` combos: their text takes typing, which no handler
//! reads - only `SelectedIndexChanged` is wired - and which leaves no row selected, as the native
//! combo box clears its list's selection when its edit text changes with the list closed; so
//! choosing any row afterwards is a change, puts that row's text back and runs the handler. The
//! arrow drops the list; the text is typed into.
//!
//! The `.cs` has `Validating` handlers for the three calibration boxes (`:268-272, 308-312,
//! 332-336`) that would hold the caret in a box whose text does not parse, but the Designer wires
//! none of them: leaving a box always runs `Validated`, whose `float.Parse` throws into its
//! `catch`.
//!
//! What a write's failure says - a `catch`'s "Set ... Failed" after `setParam` timed out, the
//! monitor combo's own "Set BATT_MONITOR Failed!" - goes on the status line, the owner's ruling of
//! 2026-09-25; so does what refuses what was typed - "Invalid number entered", text that does not
//! parse, the feature not enabled - since his word that evening, three times, on that box (a
//! `Job::Show` is an `Event::Status`). Each `InputBox` OK also keeps the answer as `InputBox`
//! does, under `InputBox<caption><question>` (`ExtLibs/Controls/InputBox.cs:73-84, 178-184`).
//!
//! "MP Alert on Low Battery" reads and writes `Settings.Instance` - Mission Planner's `config.xml`,
//! the dictionary [`Persisted`] is, which the Planner page and Battery Monitor 2 read and write
//! too: `Activate` ticks it from `speechbatteryenabled` and `speechenable`
//! (`ConfigBatteryMonitoring.cs:53-60`), and a click writes those two and, ticked, asks the three
//! `InputBox` questions whose answers are `speechbattery`, `speechbatteryvolt` and
//! `speechbatterypercent` (`ConfigBatteryMonitoring.cs:565-601`). Each change is in
//! the dictionary at once and in the file at the next `SaveConfig`; nothing here speaks them, as
//! nothing in this application has a speech engine.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use web_time::{Duration, Instant};

use gpui::{
    AnyElement, Context, Div, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px,
    rgb,
};
use mp_link::requests::RequestOutcome;
use mp_params::param_file::invariant_double;

use super::failsafe::{Lookup, Message, options};
use super::flight_modes::{ParamWriter, Progress};
use super::optional::{picture, remember_answer};
use super::servo_output::{Combo, dropdown};
use crate::MissionPlanner;
use crate::settings::Persisted;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action, panel, theme};

/// The page's name in Initial Setup's list, `backstageViewPagebatmon.Text`.
/// `// C#: GCSViews/InitialSetup.resx:243-245`
pub const TITLE: &str = "Battery Monitor";


/// `Strings.ErrorFeatureNotEnabled`.
/// `// C#: ExtLibs/Strings/Strings.resx:143-145`
pub const FEATURE_NOT_ENABLED: &str = "This feature is not enabled in your firmware.";

/// `Strings.InvalidNumberEntered`, newline and all.
/// `// C#: ExtLibs/Strings/Strings.resx:186-188`
pub const INVALID_NUMBER: &str = "Invalid number entered\n";

/// `timer1.Interval`.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.Designer.cs:130`
const TIMER_INTERVAL: Duration = Duration::from_millis(1000);

/// `TXT_battcapacity.Text` before `Activate` reads the vehicle's.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.resx TXT_battcapacity.Text`
const DESIGNER_CAPACITY: &str = "2200";

/// `CMB_batmonsensortype.Items`: the sensors, each named by its number.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.resx CMB_batmonsensortype.Items-Items9`
pub const SENSORS: [&str; 10] = [
    "0: Other",
    "1: AttoPilot 45A",
    "2: AttoPilot 90A",
    "3: AttoPilot 180A",
    "4: 3DR Power Module",
    "5: 3DR 4 in 1 ESC",
    "6: 3DR HV Power Module APM",
    "7: Cube HV Power Module",
    "8: CUAV HV PM",
    "9: Holybro Power Module",
];

/// `CMB_HWVersion.Items`: the boards, each named by its number.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.resx CMB_HWVersion.Items-Items10`
pub const HW_VERSIONS: [&str; 11] = [
    "0: CUAV V5/Pixhawk4 or APM1",
    "1: APM2 - 2.5 non 3DR",
    "2: APM2.5+/ZealotF427 - 3DR Power Module",
    "3: PX4",
    "4: The Cube or Pixhawk",
    "5: VR Brain 4.5 - 5",
    "6: VR Micro Brain 5",
    "7: VR Brain 4",
    "8: Cube Orange",
    "9: Durandal/ZealotH743",
    "10: Pixhawk 6C/Pix32 v6",
];

/// What each board writes, `(BATT_VOLT_PIN, BATT_CURR_PIN)`, by its number.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:492-557`
pub const HW_PINS: [(f64, f64); 11] = [
    (0.0, 1.0),
    (1.0, 2.0),
    (13.0, 12.0),
    (100.0, 101.0),
    (2.0, 3.0),
    (10.0, 11.0),
    (10.0, -1.0),
    (6.0, 7.0),
    (14.0, 15.0),
    (16.0, 17.0),
    (8.0, 4.0),
];

/// The divider's names, oldest first, as `setParam` tries them.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:296, 320`
pub const DIVIDER_NAMES: [&str; 2] = ["VOLT_DIVIDER", "BATT_VOLT_MULT"];

/// The amps-per-volt's names, oldest first, as `setParam` tries them.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:344, 643`
pub const AMPS_NAMES: [&str; 3] = ["AMP_PER_VOLT", "BATT_AMP_PERVOLT", "BATT_AMP_PERVLT"];

/// `(amps per volt, divider)` for sensors 1 to 9, as `Activate` recognises them: the texts of
/// these numbers, `(13.6612).ToString()` and so on, against the boxes' texts.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:64-103`
const DETECT: [(f64, f64); 9] = [
    (13.6612, 4.127115),
    (27.3224, 15.70105),
    (54.64481, 15.70105),
    (18.0018, 10.10101),
    (17.0, 12.02),
    (24.0, 12.02),
    (39.877, 12.02),
    (24.0, 18.0),
    (36.364, 18.182),
];

/// The monitor combo's handler's `catch`.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:264`
const MONITOR_FAILED: &str = "Set BATT_MONITOR,BATT_VOLT_PIN,BATT_CURR_PIN Failed";
/// `MavlinkComboBox`'s own write's failure, `"Set " + ParamName + " Failed!"`.
/// `// C#: Controls/MavlinkComboBox.cs:180-183, 197`
const COMBO_FAILED: &str = "Set BATT_MONITOR Failed!";
/// `TXT_battcapacity_Validated`'s `catch`.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:201`
const CAPACITY_FAILED: &str = "Set BATT_CAPACITY Failed";
/// The divider's `catch`, shown only for an analog monitor.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:303, 327`
const DIVIDER_FAILED: &str = "Set BATT_VOLT_MULT Failed";
/// The amps-per-volt's `catch`, shown only for an analog monitor.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:351, 650`
const AMPS_FAILED: &str = "Set BATT_AMP_PERVOLT Failed";
/// The board combo's `catch`.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:561`
const PINS_FAILED: &str = "Set BATT_????_PIN Failed";

/// The Low Battery alert's three questions: caption, question, the setting it fills, and what it
/// offers when the setting has nothing.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:577-599`
const SPEECH_PROMPTS: [(&str, &str, &str, &str); 3] = [
    (
        "Notification",
        "What do you want it to say?",
        "speechbattery",
        "WARNING, Battery at {batv} Volt, {batp} percent",
    ),
    (
        "Battery Level",
        "What Voltage do you want to warn at?",
        "speechbatteryvolt",
        "9.6",
    ),
    (
        "Battery Level",
        "What percentage do you want to warn at?",
        "speechbatterypercent",
        "20",
    ),
];

// ---------------------------------------------------------------------------------------------
// Numbers as the C# reads and writes them.
// ---------------------------------------------------------------------------------------------

/// `float.ToString()` as .NET Framework writes it: `G` at seven significant digits, fixed-point
/// for a power of ten from -5 (exclusive) to 7 (exclusive) and `1.234568E+07` otherwise, trailing
/// zeros dropped, and a negative zero written "0".
///
/// The seventh digit is rounded half away from zero on the float's exact decimal expansion, as
/// the CRT's `_ecvt` behind `Number.FormatSingle` rounds; a float lands exactly on a half at the
/// eighth digit only for a few short binary fractions, and none of this page's numbers.
#[must_use]
pub fn float_text(value: f32) -> String {
    if value.is_nan() {
        return "NaN".to_owned();
    }
    if value.is_infinite() {
        return if value > 0.0 { "Infinity" } else { "-Infinity" }.to_owned();
    }
    if value == 0.0 {
        return "0".to_owned();
    }
    // Forty digits hold a float's whole expansion for any number this page shows.
    let exact = format!("{:.40e}", f64::from(value).abs());
    let Some((mantissa, exponent)) = exact.split_once('e') else {
        return "0".to_owned();
    };
    let mut exponent: i32 = exponent.parse().unwrap_or(0);
    let digits: Vec<u8> = mantissa
        .bytes()
        .filter(u8::is_ascii_digit)
        .map(|digit| digit - b'0')
        .collect();
    let mut kept: Vec<u8> = digits.iter().copied().take(7).collect();
    if digits.get(7).is_some_and(|digit| *digit >= 5) {
        let mut carry = true;
        for digit in kept.iter_mut().rev() {
            if *digit == 9 {
                *digit = 0;
            } else {
                *digit += 1;
                carry = false;
                break;
            }
        }
        if carry {
            kept.insert(0, 1);
            kept.pop();
            exponent += 1;
        }
    }
    while kept.len() > 1 && kept.last() == Some(&0) {
        kept.pop();
    }
    let text: String = kept.iter().map(|digit| char::from(b'0' + digit)).collect();
    let sign = if value < 0.0 { "-" } else { "" };
    if exponent > -5 && exponent < 7 {
        let point = usize::try_from(exponent + 1).unwrap_or(0);
        if exponent >= 0 {
            let (whole, fraction) = if text.len() > point {
                (
                    text.get(..point).unwrap_or(""),
                    text.get(point..).unwrap_or(""),
                )
            } else {
                (text.as_str(), "")
            };
            let zeros = "0".repeat(point.saturating_sub(text.len()));
            if fraction.is_empty() {
                format!("{sign}{whole}{zeros}")
            } else {
                format!("{sign}{whole}.{fraction}")
            }
        } else {
            let zeros = "0".repeat(usize::try_from(-exponent - 1).unwrap_or(0));
            format!("{sign}0.{zeros}{text}")
        }
    } else {
        let (first, rest) = text.split_at(1);
        let point = if rest.is_empty() { "" } else { "." };
        let exponent_sign = if exponent < 0 { '-' } else { '+' };
        format!(
            "{sign}{first}{point}{rest}E{exponent_sign}{:02}",
            exponent.unsigned_abs()
        )
    }
}

/// `float.Parse` and `float.TryParse`, `NumberStyles.Float | AllowThousands` in an English
/// culture: white space either side, a leading sign, a point, an exponent, commas among the whole
/// digits, and the words `Infinity`, `-Infinity` and `NaN`. `None` is the `FormatException` - and
/// the `OverflowException` .NET Framework raises for a number past a float's range.
///
/// .NET Framework reads the text as a double and narrows it, so this does too.
#[must_use]
pub fn parse_float(text: &str) -> Option<f32> {
    let trimmed = text.trim_matches(|c: char| c == ' ' || ('\t'..='\r').contains(&c));
    match trimmed {
        "Infinity" => return Some(f32::INFINITY),
        "-Infinity" => return Some(f32::NEG_INFINITY),
        "NaN" => return Some(f32::NAN),
        _ => {}
    }
    if !trimmed
        .chars()
        .all(|c| c.is_ascii_digit() || matches!(c, '+' | '-' | '.' | 'e' | 'E' | ','))
    {
        return None;
    }
    // Group separators belong to the whole part only.
    let split = trimmed.find(['.', 'e', 'E']).unwrap_or(trimmed.len());
    let (whole, rest) = trimmed.split_at(split);
    if rest.contains(',') {
        return None;
    }
    let candidate = format!("{}{rest}", whole.replace(',', ""));
    let double: f64 = candidate.parse().ok()?;
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)double`
    let single = double as f32;
    single.is_finite().then_some(single)
}

/// A parameter's text: `MAVLinkParam.ToString()`, which for ArduPilot's floats is
/// `((float)Value).ToString()`.
/// `// C#: ExtLibs/Mavlink/MAVLinkParam.cs:219-224`
#[must_use]
pub fn param_text(value: f64) -> String {
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)this`
    let single = value as f32;
    float_text(single)
}

/// The divider or amps-per-volt a measurement gives: `(measured * factor) / reading`, in floats,
/// or `None` for a reading of zero, where the C# returns without writing.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:280-286, 627-633`
#[must_use]
pub fn calibrate(measured: f32, factor: f32, reading: f32) -> Option<f32> {
    if reading == 0.0 {
        return None;
    }
    Some((measured * factor) / reading)
}

/// A sensor's divider and amps-per-volt texts, as its branch of the sensor combo's handler sets
/// them; `None` for 0, "Other", which leaves the boxes alone.
///
/// The AttoPilots and the 3DR module are worked out from their full-scale readings in floats - the
/// voltage at full scale, `max * mV per unit / 1000`, divided into the maximum - and written as
/// the float's text; the rest are double constants and written as the double's.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:360-438`
#[must_use]
pub fn preset(sensor: usize) -> Option<(String, String)> {
    let computed = |max_volt: f32, max_amps: f32, mv_per_volt: f32, mv_per_amp: f32| {
        let top_volt = (max_volt * mv_per_volt) / 1000.0;
        let top_amps = (max_amps * mv_per_amp) / 1000.0;
        (
            float_text(max_volt / top_volt),
            float_text(max_amps / top_amps),
        )
    };
    let fixed = |divider: f64, amps: f64| (invariant_double(divider), invariant_double(amps));
    match sensor {
        // AttoPilot 45A.
        1 => Some(computed(13.6, 44.7, 242.3, 73.20)),
        // AttoPilot 90A.
        2 => Some(computed(50.0, 89.4, 63.69, 36.60)),
        // AttoPilot 180A.
        3 => Some(computed(50.0, 178.8, 63.69, 18.30)),
        // 3DR Power Module.
        4 => Some(computed(50.0, 90.0, 99.0, 55.55)),
        // 3DR 4 in 1 ESC.
        5 => Some(fixed(12.02, 17.0)),
        // 3DR HV Power Module APM.
        6 => Some(fixed(12.02, 24.0)),
        // Cube HV Power Module.
        7 => Some(fixed(12.02, 39.877)),
        // CUAV HV PM.
        8 => Some(fixed(18.0, 24.0)),
        // Holybro Power Module.
        9 => Some(fixed(18.182, 36.364)),
        _ => None,
    }
}

/// Which sensor `Activate` takes the boxes for: the first whose texts both match, else 0.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:62-103`
#[must_use]
pub fn detect_sensor(amps: &str, divider: &str) -> usize {
    DETECT
        .iter()
        .position(|(sensor_amps, sensor_divider)| {
            invariant_double(*sensor_amps) == amps && invariant_double(*sensor_divider) == divider
        })
        .map_or(0, |index| index + 1)
}

/// What `Activate` makes of `BATT_VOLT_PIN`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HwDetect {
    /// A board it names.
    Index(usize),
    /// A pin it has no branch for: the combo keeps what it had.
    Unmatched,
    /// Pin 10 without a `BATT_CURR_PIN`: the cast of the missing parameter throws, and
    /// `Activate` goes no further.
    Throws,
}

/// The board `BATT_VOLT_PIN` (and, for pin 10, `BATT_CURR_PIN`) says the vehicle is.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:105-164`
#[must_use]
pub fn detect_hw(volt_pin: f64, curr_pin: Option<f64>) -> HwDetect {
    // Exact comparisons of a parameter's value against whole numbers, as the C# makes them.
    #[allow(clippy::float_cmp)]
    let index = if volt_pin == 0.0 {
        0
    } else if volt_pin == 1.0 {
        1
    } else if volt_pin == 13.0 {
        2
    } else if volt_pin == 100.0 {
        3
    } else if volt_pin == 2.0 {
        4
    } else if volt_pin == 6.0 {
        7
    } else if volt_pin == 10.0 {
        match curr_pin {
            Some(11.0) => 5,
            Some(_) => 6,
            None => return HwDetect::Throws,
        }
    } else if volt_pin == 14.0 {
        8
    } else if volt_pin == 16.0 {
        9
    } else if volt_pin == 8.0 {
        10
    } else {
        return HwDetect::Unmatched;
    };
    HwDetect::Index(index)
}

/// A parameter's value in the vehicle's table.
fn value_of(parameters: &[(String, f64)], name: &str) -> Option<f64> {
    parameters
        .iter()
        .find(|(held, _)| held == name)
        .map(|(_, value)| *value)
}

// ---------------------------------------------------------------------------------------------
// The writes.
// ---------------------------------------------------------------------------------------------

/// One `setParam`: the names it tries, in order, and the value.
#[derive(Debug, Clone, PartialEq)]
pub struct Step {
    /// One name, or the generations of one, oldest first.
    pub names: Vec<&'static str>,
    /// The value, as the C# computes it.
    pub value: f64,
}

impl Step {
    fn one(name: &'static str, value: f64) -> Self {
        Self {
            names: vec![name],
            value,
        }
    }

    fn any(names: &[&'static str], value: f32) -> Self {
        Self {
            names: names.to_vec(),
            value: f64::from(value),
        }
    }
}

/// What a `catch` shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OnThrow {
    /// Always this message.
    Show(&'static str),
    /// This message, but only while `BATT_MONITOR` is 3 or 4, an analog monitor.
    ShowIfAnalog(&'static str),
}

/// One handler's work, as the C# does it in one go.
#[derive(Debug, Clone, PartialEq)]
pub enum Job {
    /// `setParam` calls inside one `try`.
    Writes {
        /// In order.
        steps: VecDeque<Step>,
        /// What the `catch` shows when one throws, which ends the job.
        on_throw: OnThrow,
        /// What shows when a step returns false - the vehicle has none of its names -
        /// `MavlinkComboBox`'s check; the page's own handlers do not look.
        on_false: Option<&'static str>,
        /// `getParamList` and `Activate` afterwards: `BATT_MONITOR` taken off 0.
        refresh: bool,
    },
    /// A `float.Parse` that threw inside the `try`, before any write.
    Throw(OnThrow),
    /// A message box outside any `try`.
    Show(&'static str),
}

impl Job {
    fn writes(steps: Vec<Step>, on_throw: OnThrow) -> Self {
        Self::Writes {
            steps: steps.into(),
            on_throw,
            on_false: None,
            refresh: false,
        }
    }

    /// The writes, in order, for a test to read.
    #[cfg(test)]
    #[must_use]
    pub fn steps(&self) -> Vec<Step> {
        match self {
            Self::Writes { steps, .. } => steps.iter().cloned().collect(),
            Self::Throw(_) | Self::Show(_) => Vec::new(),
        }
    }
}

/// What running the jobs produced.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// A step finished: the name that took it, the value asked for, and how.
    Written {
        /// The name the vehicle had.
        name: &'static str,
        /// The value sent.
        value: f64,
        /// The vehicle's answer.
        outcome: RequestOutcome,
    },
    /// A step every name of which the vehicle refused - `setParam`'s false - with what
    /// `MavlinkComboBox` says of it: a link failure.
    Refused(&'static str),
    /// A message the C# shows in a box outside any `try` ("Invalid number entered", the feature
    /// not enabled), on the status line here: an avoidable error never gets a box - the owner's
    /// rule of 2026-09-25, and his word again that evening on that box, three times.
    Status(&'static str),
    /// A `float.Parse` of what was typed threw into the `catch`; its message, if it shows one
    /// for the monitor the vehicle has.
    Unparsed(OnThrow),
    /// A write threw into the `catch` - a timeout, or no vehicle; its message, if it shows one
    /// for the monitor the vehicle has: a link failure.
    Threw {
        /// What was being written, for the record.
        what: String,
        /// The `catch`'s rule.
        on_throw: OnThrow,
    },
    /// `getParamList` then `Activate`.
    Refresh,
}

/// The job running, and where it is.
#[derive(Debug)]
struct Running<H> {
    job: Job,
    /// Which of the front step's names is being tried.
    name: usize,
    /// The write the link is carrying.
    waiting: Option<H>,
}

/// The handlers' jobs, run one at a time, one write at a time, as the C#'s blocking calls go.
#[derive(Debug)]
pub struct Runner<H> {
    queue: VecDeque<Job>,
    running: Option<Running<H>>,
}

impl<H> Default for Runner<H> {
    fn default() -> Self {
        Self {
            queue: VecDeque::new(),
            running: None,
        }
    }
}

impl<H: Copy> Runner<H> {
    /// Queues a handler's work.
    pub fn push(&mut self, jobs: impl IntoIterator<Item = Job>) {
        self.queue.extend(jobs);
    }

    /// How many jobs are queued or running.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.queue.len() + usize::from(self.running.is_some())
    }

    /// Moves the jobs on as far as the link's answers allow.
    ///
    /// A name the vehicle does not have is `setParam` returning false: the next name is tried,
    /// and with none left the step is over and the job goes on. A timeout, or a link that has
    /// forgotten the write, is the `TimeoutException`: the job ends there. A value the vehicle
    /// already holds is `setParam`'s "not modified as same", a success.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1609-1620, 1640-1651, 1765`
    pub fn advance<W: ParamWriter<Handle = H>>(&mut self, writer: &W) -> Vec<Event> {
        let mut events = Vec::new();
        loop {
            let Some(running) = self.running.as_mut() else {
                let Some(job) = self.queue.pop_front() else {
                    return events;
                };
                self.running = Some(Running {
                    job,
                    name: 0,
                    waiting: None,
                });
                continue;
            };
            let (steps, on_throw, on_false, refresh) = match &mut running.job {
                Job::Writes {
                    steps,
                    on_throw,
                    on_false,
                    refresh,
                } => (steps, *on_throw, *on_false, *refresh),
                &mut Job::Throw(on_throw) => {
                    events.push(Event::Unparsed(on_throw));
                    self.running = None;
                    continue;
                }
                &mut Job::Show(text) => {
                    events.push(Event::Status(text));
                    self.running = None;
                    continue;
                }
            };
            if let Some(handle) = running.waiting {
                let Some(step) = steps.front() else {
                    running.waiting = None;
                    continue;
                };
                let name = step.names.get(running.name).copied().unwrap_or("");
                match writer.progress(handle) {
                    Progress::Waiting => return events,
                    Progress::Finished(RequestOutcome::TimedOut) | Progress::Lost => {
                        events.push(Event::Threw {
                            what: format!("{name} {}", step.value),
                            on_throw,
                        });
                        self.running = None;
                        continue;
                    }
                    Progress::Finished(
                        RequestOutcome::UnknownParameter | RequestOutcome::Rejected(_),
                    ) => {
                        running.waiting = None;
                        running.name += 1;
                        continue;
                    }
                    Progress::Finished(outcome) => {
                        events.push(Event::Written {
                            name,
                            value: step.value,
                            outcome,
                        });
                        running.waiting = None;
                        running.name = 0;
                        steps.pop_front();
                        continue;
                    }
                }
            }
            let Some(step) = steps.front() else {
                if refresh {
                    events.push(Event::Refresh);
                }
                self.running = None;
                continue;
            };
            let Some(name) = step.names.get(running.name).copied() else {
                // Every name refused: `setParam` returned false.
                if let Some(text) = on_false {
                    events.push(Event::Refused(text));
                }
                running.name = 0;
                steps.pop_front();
                continue;
            };
            match writer.write(name, step.value) {
                Some(handle) => running.waiting = Some(handle),
                None => {
                    // No vehicle to write to: the C#'s send throws.
                    events.push(Event::Threw {
                        what: format!("{name} {}", step.value),
                        on_throw,
                    });
                    self.running = None;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The page.
// ---------------------------------------------------------------------------------------------

/// The text boxes that take typing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Field {
    /// `TXT_battcapacity`.
    Capacity,
    /// `TXT_measuredvoltage`.
    Measured,
    /// `TXT_divider_VOLT_MULT`.
    Divider,
    /// `txt_meascurrent`.
    MeasuredCurrent,
    /// `TXT_AMP_PERVLT`.
    AmpsPerVolt,
}

impl Field {
    /// Every one, in the order their focus handles are kept.
    pub const ALL: [Self; 5] = [
        Self::Capacity,
        Self::Measured,
        Self::Divider,
        Self::MeasuredCurrent,
        Self::AmpsPerVolt,
    ];

    /// The name a test clicks it by.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Capacity => "battery-capacity",
            Self::Measured => "battery-measured",
            Self::Divider => "battery-divider",
            Self::MeasuredCurrent => "battery-meascurrent",
            Self::AmpsPerVolt => "battery-ampspervolt",
        }
    }

    const fn index(self) -> usize {
        match self {
            Self::Capacity => 0,
            Self::Measured => 1,
            Self::Divider => 2,
            Self::MeasuredCurrent => 3,
            Self::AmpsPerVolt => 4,
        }
    }

    /// Whether Enter validates it: the three with a `PreviewKeyDown` handler.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:603-619`
    const fn validates_on_enter(self) -> bool {
        matches!(self, Self::Measured | Self::Divider | Self::AmpsPerVolt)
    }
}

/// The three combo boxes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComboId {
    /// `CMB_batmontype`.
    Monitor,
    /// `CMB_batmonsensortype`.
    Sensor,
    /// `CMB_HWVersion`.
    Hardware,
}

impl ComboId {
    const fn id(self) -> &'static str {
        match self {
            Self::Monitor => "battery-monitor",
            Self::Sensor => "battery-sensor",
            Self::Hardware => "battery-hw",
        }
    }
}

/// `Settings.Instance.GetBoolean(key)`: `bool.TryParse`, false when absent or not a boolean.
/// `// C#: ExtLibs/Utilities/Settings.cs:223-232`
fn get_boolean(settings: &Persisted, key: &str) -> bool {
    settings.get(key).is_some_and(|value| {
        value
            .trim_matches(|c: char| c.is_whitespace() || c == '\0')
            .eq_ignore_ascii_case("true")
    })
}

/// One of the Low Battery alert's `InputBox`es.
#[derive(Debug)]
pub struct Prompt {
    /// Which question, 0 to 2.
    stage: usize,
    /// The answer box.
    pub field: TextField,
}

impl Prompt {
    /// The `stage`th question, its box holding the setting or the handler's literal.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:577-599`
    fn at(stage: usize, settings: &Persisted) -> Option<Self> {
        let (_, _, key, default) = SPEECH_PROMPTS.get(stage)?;
        let mut field = TextField::new("");
        field.set(settings.get(key).unwrap_or(*default));
        Some(Self { stage, field })
    }

    /// The caption and the question.
    #[must_use]
    pub fn text(&self) -> (&'static str, &'static str) {
        SPEECH_PROMPTS
            .get(self.stage)
            .map_or(("", ""), |(title, question, _, _)| (*title, *question))
    }
}

/// Every control as the handlers leave it.
#[derive(Debug)]
pub struct Controls {
    /// The page's `Enabled`: false when `Activate` found no link or no `BATT_MONITOR`.
    pub enabled: bool,
    /// `startup`: the handlers write nothing while it is set.
    startup: bool,
    /// `CMB_batmontype`'s `DataSource`, `BATT_MONITOR`'s documented values.
    pub monitor_options: Vec<(i64, String)>,
    /// Its `SelectedValue`; `None` for a value the list does not hold.
    pub monitor: Option<i64>,
    /// Its `Enabled`.
    pub monitor_enabled: bool,
    /// `CMB_batmonsensortype.SelectedIndex`.
    pub sensor: Option<usize>,
    /// Its `Enabled`.
    pub sensor_enabled: bool,
    /// Its `Text` as typed into its edit box, until a row is chosen again; `None` while it
    /// shows the selected row.
    pub sensor_typed: Option<String>,
    /// `CMB_HWVersion.SelectedIndex`.
    pub hardware: Option<usize>,
    /// Its `Enabled`.
    pub hardware_enabled: bool,
    /// Its `Text` as typed.
    pub hardware_typed: Option<String>,
    /// `TXT_battcapacity`.
    pub capacity: TextField,
    /// `TXT_measuredvoltage`.
    pub measured: TextField,
    /// Its `Enabled`.
    pub measured_enabled: bool,
    /// `TXT_voltage.Text`: the vehicle's battery voltage.
    pub voltage: String,
    /// `TXT_voltage.Enabled`, which the monitor handler clears.
    pub voltage_enabled: bool,
    /// `TXT_divider_VOLT_MULT`.
    pub divider: TextField,
    /// Its `Enabled`.
    pub divider_enabled: bool,
    /// `txt_meascurrent`.
    pub measured_current: TextField,
    /// `txt_current.Text`: the vehicle's current.
    pub current: String,
    /// `TXT_AMP_PERVLT`.
    pub amps_per_volt: TextField,
    /// Its `Enabled`.
    pub amps_enabled: bool,
    /// `groupBox4.Enabled`, the Calibration box.
    pub calibration_enabled: bool,
    /// `CHK_speechbattery.Checked`.
    pub speech: bool,
}

impl Default for Controls {
    /// As the designer leaves them. `MavlinkComboBox` starts disabled; nothing else does.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.Designer.cs:36-277; Controls/MavlinkComboBox.cs:31-35`
    fn default() -> Self {
        let mut capacity = TextField::new("");
        capacity.set(DESIGNER_CAPACITY);
        Self {
            enabled: true,
            startup: false,
            monitor_options: Vec::new(),
            monitor: None,
            monitor_enabled: false,
            sensor: None,
            sensor_enabled: true,
            sensor_typed: None,
            hardware: None,
            hardware_enabled: true,
            hardware_typed: None,
            capacity,
            measured: TextField::new(""),
            measured_enabled: true,
            voltage: String::new(),
            voltage_enabled: true,
            divider: TextField::new(""),
            divider_enabled: true,
            measured_current: TextField::new(""),
            current: String::new(),
            amps_per_volt: TextField::new(""),
            amps_enabled: true,
            calibration_enabled: true,
            speech: false,
        }
    }
}

impl Controls {
    /// `Activate`: every control from the vehicle's parameters, then the monitor and sensor
    /// handlers run once as the C# calls them, which is where the page's first writes come from.
    ///
    /// `voltage` is `cs.battery_voltage`, which both voltage boxes start at.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:18-176`
    #[must_use]
    pub fn activate(
        view: &TelemetryView,
        lookup: Lookup,
        settings: &Persisted,
    ) -> (Self, Vec<Job>) {
        let mut controls = Self::default();
        let parameters = &view.parameters;
        let Some(monitor) = value_of(parameters, "BATT_MONITOR").filter(|_| view.connected) else {
            // `Enabled = false; return;` - the rest keep the designer's.
            controls.enabled = false;
            return (controls, Vec::new());
        };
        controls.startup = true;

        // `MavlinkComboBox.setup(source, "BATT_MONITOR", param)`. `// C#: Controls/MavlinkComboBox.cs:73-99`
        controls.monitor_options = options("BATT_MONITOR", lookup);
        controls.monitor_enabled = true;
        #[allow(clippy::cast_possible_truncation)] // `(int)paramlist[paramname].Value`
        let selected = monitor as i64;
        controls.monitor = controls
            .monitor_options
            .iter()
            .any(|(key, _)| *key == selected)
            .then_some(selected);

        if let Some(capacity) = value_of(parameters, "BATT_CAPACITY") {
            controls.capacity.set(param_text(capacity));
        }
        let voltage = voltage_text(view);
        controls.measured.set(voltage.clone());
        controls.voltage = voltage;

        // Newest names first, so the oldest the vehicle has is what the box ends with.
        // `// C#: :38-51`
        for (name, field) in [
            ("BATT_AMP_PERVLT", Field::AmpsPerVolt),
            ("BATT_VOLT_MULT", Field::Divider),
            ("BATT_AMP_PERVOLT", Field::AmpsPerVolt),
            ("VOLT_DIVIDER", Field::Divider),
            ("AMP_PER_VOLT", Field::AmpsPerVolt),
        ] {
            if let Some(value) = value_of(parameters, name) {
                controls.field_mut(field).set(param_text(value));
            }
        }

        // `// C#: :53-60`
        controls.speech =
            get_boolean(settings, "speechbatteryenabled") && get_boolean(settings, "speechenable");

        // Setting `SelectedIndex` raises the sensor handler, which has no `startup` check but
        // writes nothing while it is set. `// C#: :62-103`
        controls.sensor = Some(detect_sensor(
            controls.amps_per_volt.value(),
            controls.divider.value(),
        ));
        let during_startup = controls.sensor_changed();
        debug_assert!(during_startup.is_empty(), "startup writes nothing");

        // `// C#: :105-168`
        if let Some(volt_pin) = value_of(parameters, "BATT_VOLT_PIN") {
            controls.hardware_enabled = true;
            match detect_hw(volt_pin, value_of(parameters, "BATT_CURR_PIN")) {
                HwDetect::Index(index) => controls.hardware = Some(index),
                HwDetect::Unmatched => {}
                // The exception leaves `startup` set and the timer stopped.
                HwDetect::Throws => return (controls, Vec::new()),
            }
        } else {
            controls.hardware_enabled = false;
        }

        controls.startup = false;
        // `// C#: :172-173`
        let mut jobs = controls.monitor_changed(parameters);
        jobs.extend(controls.sensor_changed());
        (controls, jobs)
    }

    /// A text box.
    #[must_use]
    pub const fn field(&self, field: Field) -> &TextField {
        match field {
            Field::Capacity => &self.capacity,
            Field::Measured => &self.measured,
            Field::Divider => &self.divider,
            Field::MeasuredCurrent => &self.measured_current,
            Field::AmpsPerVolt => &self.amps_per_volt,
        }
    }

    const fn field_mut(&mut self, field: Field) -> &mut TextField {
        match field {
            Field::Capacity => &mut self.capacity,
            Field::Measured => &mut self.measured,
            Field::Divider => &mut self.divider,
            Field::MeasuredCurrent => &mut self.measured_current,
            Field::AmpsPerVolt => &mut self.amps_per_volt,
        }
    }

    /// Whether a text box takes typing and validates: its own `Enabled`, inside the
    /// Calibration box's for the five in it, inside the page's - `Control.Enabled` reads false
    /// under a disabled parent.
    #[must_use]
    pub const fn field_enabled(&self, field: Field) -> bool {
        let own = match field {
            Field::Capacity => return self.enabled,
            Field::Measured => self.measured_enabled,
            Field::Divider => self.divider_enabled,
            Field::MeasuredCurrent => true,
            Field::AmpsPerVolt => self.amps_enabled,
        };
        own && self.calibration_enabled && self.enabled
    }

    /// Whether a combo box takes clicks.
    #[must_use]
    pub const fn combo_enabled(&self, combo: ComboId) -> bool {
        self.enabled
            && match combo {
                ComboId::Monitor => self.monitor_enabled,
                ComboId::Sensor => self.sensor_enabled,
                ComboId::Hardware => self.hardware_enabled,
            }
    }

    /// A `DropDown` combo's `Text`: what was typed into it, else its selected row's.
    #[must_use]
    pub fn combo_text(&self, combo: ComboId) -> &str {
        let (typed, selected, items) = match combo {
            ComboId::Monitor => return self.monitor_text(),
            ComboId::Sensor => (&self.sensor_typed, self.sensor, SENSORS.as_slice()),
            ComboId::Hardware => (&self.hardware_typed, self.hardware, HW_VERSIONS.as_slice()),
        };
        typed.as_deref().unwrap_or_else(|| {
            selected
                .and_then(|index| items.get(index))
                .copied()
                .unwrap_or("")
        })
    }

    /// A key in a `DropDown` combo's edit box: its text edited, and with it the selection gone -
    /// `SelectedIndex` -1, and no `SelectedIndexChanged`, which is raised by the list alone. The
    /// monitor combo is a `DropDownList` and takes no typing. Whether the key was taken.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.Designer.cs:72-87, 107, 135-151`
    fn type_combo(&mut self, combo: ComboId, event: &KeyDownEvent) -> bool {
        if !self.combo_enabled(combo) || combo == ComboId::Monitor {
            return false;
        }
        let mut field = TextField::new("");
        field.set(self.combo_text(combo));
        if field.key(event) != KeyOutcome::Changed {
            return false;
        }
        let text = field.value().to_owned();
        match combo {
            ComboId::Sensor => {
                self.sensor = None;
                self.sensor_typed = Some(text);
            }
            ComboId::Hardware => {
                self.hardware = None;
                self.hardware_typed = Some(text);
            }
            ComboId::Monitor => {}
        }
        true
    }

    /// The monitor combo's text.
    #[must_use]
    pub fn monitor_text(&self) -> &str {
        self.monitor
            .and_then(|value| self.monitor_options.iter().find(|(key, _)| *key == value))
            .map_or("", |(_, text)| text.as_str())
    }

    /// `CMB_batmontype_SelectedIndexChanged`: the Calibration box and the other combos enabled for
    /// the monitor chosen, the pins cleared for none, and `BATT_MONITOR` written - with the
    /// parameters fetched again and the page activated afresh when it comes off 0.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:205-266`
    fn monitor_changed(&mut self, parameters: &[(String, f64)]) -> Vec<Job> {
        if self.startup {
            return Vec::new();
        }
        let Some(current) = value_of(parameters, "BATT_MONITOR") else {
            return vec![Job::Show(FEATURE_NOT_ENABLED)];
        };
        // `(int)CMB_batmontype.SelectedValue` throws on nothing selected.
        let Some(selection) = self.monitor else {
            return vec![Job::Throw(OnThrow::Show(MONITOR_FAILED))];
        };
        self.sensor_enabled = true;
        self.voltage_enabled = false;
        let mut steps = Vec::new();
        match selection {
            0 => {
                self.sensor_enabled = false;
                self.hardware_enabled = false;
                self.calibration_enabled = false;
                steps.push(Step::one("BATT_VOLT_PIN", -1.0));
                steps.push(Step::one("BATT_CURR_PIN", -1.0));
            }
            4 => {
                self.sensor_enabled = true;
                self.hardware_enabled = true;
                self.calibration_enabled = true;
                self.amps_enabled = true;
            }
            3 => {
                self.calibration_enabled = true;
                self.sensor_enabled = false;
                self.hardware_enabled = true;
                self.amps_enabled = false;
                self.measured_enabled = true;
                self.divider_enabled = true;
            }
            _ => {}
        }
        #[allow(clippy::cast_precision_loss)] // monitor types are small
        let value = selection as f64;
        steps.push(Step::one("BATT_MONITOR", value));
        vec![Job::Writes {
            steps: steps.into(),
            on_throw: OnThrow::Show(MONITOR_FAILED),
            on_false: None,
            refresh: current == 0.0 && selection != 0,
        }]
    }

    /// `CMB_batmonsensortype_SelectedIndexChanged`: the chosen sensor's divider and amps-per-volt
    /// into their boxes, both written, and the three calibration boxes left disabled for anything
    /// but "Other".
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:356-462`
    fn sensor_changed(&mut self) -> Vec<Job> {
        let selection = self.sensor.unwrap_or(0);
        if let Some((divider, amps)) = preset(selection) {
            self.divider.set(divider);
            self.amps_per_volt.set(amps);
        }
        self.divider_enabled = true;
        self.amps_enabled = true;
        self.measured_enabled = true;
        let mut jobs = self.amps_validated();
        jobs.extend(self.divider_validated());
        let other = selection == 0;
        self.divider_enabled = other;
        self.amps_enabled = other;
        self.measured_enabled = other;
        jobs
    }

    /// `CMB_apmversion_SelectedIndexChanged`: the board's two pins.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:483-563`
    fn hardware_changed(&self) -> Vec<Job> {
        if self.startup {
            return Vec::new();
        }
        let Some((volt, curr)) = self.hardware.and_then(|index| HW_PINS.get(index)) else {
            return Vec::new();
        };
        vec![Job::writes(
            vec![
                Step::one("BATT_VOLT_PIN", *volt),
                Step::one("BATT_CURR_PIN", *curr),
            ],
            OnThrow::Show(PINS_FAILED),
        )]
    }

    /// `TXT_battcapacity_Validated`.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:184-203`
    fn capacity_validated(&self, parameters: &[(String, f64)]) -> Vec<Job> {
        if self.startup || !self.field_enabled(Field::Capacity) {
            return Vec::new();
        }
        if value_of(parameters, "BATT_CAPACITY").is_none() {
            return vec![Job::Show(FEATURE_NOT_ENABLED)];
        }
        vec![parse_float(self.capacity.value()).map_or(
            Job::Throw(OnThrow::Show(CAPACITY_FAILED)),
            |value| {
                Job::writes(
                    vec![Step::any(&["BATT_CAPACITY"], value)],
                    OnThrow::Show(CAPACITY_FAILED),
                )
            },
        )]
    }

    /// `TXT_measuredvoltage_Validated`: the divider the measured voltage gives, into its box and
    /// written.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:274-306`
    fn measured_validated(&mut self) -> Vec<Job> {
        if self.startup || !self.field_enabled(Field::Measured) {
            return Vec::new();
        }
        let (Some(measured), Some(voltage), Some(divider)) = (
            parse_float(self.measured.value()),
            parse_float(&self.voltage),
            parse_float(self.divider.value()),
        ) else {
            return vec![Job::Show(INVALID_NUMBER)];
        };
        let Some(divider) = calibrate(measured, divider, voltage) else {
            return Vec::new();
        };
        self.divider.set(float_text(divider));
        vec![Self::write_text(
            self.divider.value(),
            &DIVIDER_NAMES,
            DIVIDER_FAILED,
        )]
    }

    /// `TXT_divider_Validated`.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:314-330`
    fn divider_validated(&self) -> Vec<Job> {
        if self.startup || !self.field_enabled(Field::Divider) {
            return Vec::new();
        }
        vec![Self::write_text(
            self.divider.value(),
            &DIVIDER_NAMES,
            DIVIDER_FAILED,
        )]
    }

    /// `TXT_ampspervolt_Validated`.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:338-354`
    fn amps_validated(&self) -> Vec<Job> {
        if self.startup || !self.field_enabled(Field::AmpsPerVolt) {
            return Vec::new();
        }
        vec![Self::write_text(
            self.amps_per_volt.value(),
            &AMPS_NAMES,
            AMPS_FAILED,
        )]
    }

    /// `txt_meascurrent_Validated`: the amps-per-volt the measured current gives, into its box
    /// and written.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:621-653`
    fn measured_current_validated(&mut self) -> Vec<Job> {
        if self.startup || !self.field_enabled(Field::MeasuredCurrent) {
            return Vec::new();
        }
        let (Some(measured), Some(current), Some(amps)) = (
            parse_float(self.measured_current.value()),
            parse_float(&self.current),
            parse_float(self.amps_per_volt.value()),
        ) else {
            return vec![Job::Show(INVALID_NUMBER)];
        };
        let Some(amps) = calibrate(measured, amps, current) else {
            return Vec::new();
        };
        self.amps_per_volt.set(float_text(amps));
        vec![Self::write_text(
            self.amps_per_volt.value(),
            &AMPS_NAMES,
            AMPS_FAILED,
        )]
    }

    /// `setParam(names, float.Parse(text))` in a `try` whose `catch` speaks only for an analog
    /// monitor.
    fn write_text(text: &str, names: &[&'static str], failed: &'static str) -> Job {
        parse_float(text).map_or(Job::Throw(OnThrow::ShowIfAnalog(failed)), |value| {
            Job::writes(vec![Step::any(names, value)], OnThrow::ShowIfAnalog(failed))
        })
    }

    /// A box's `Validated`, by which box.
    fn validated(&mut self, field: Field, parameters: &[(String, f64)]) -> Vec<Job> {
        match field {
            Field::Capacity => self.capacity_validated(parameters),
            Field::Measured => self.measured_validated(),
            Field::Divider => self.divider_validated(),
            Field::MeasuredCurrent => self.measured_current_validated(),
            Field::AmpsPerVolt => self.amps_validated(),
        }
    }

    /// Leaving a box: `Validated`. The `.cs`'s three `Validating` handlers, which would cancel
    /// it for text that does not parse, are wired to nothing in the Designer, so they never run.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:268-272, 308-312, 332-336;
    /// ConfigBatteryMonitoring.Designer.cs:103, 174-175, 192-193, 204-205, 250`
    fn left(&mut self, field: Field, parameters: &[(String, f64)]) -> Vec<Job> {
        self.validated(field, parameters)
    }

    /// `timer1_Tick`: the vehicle's voltage and current into their boxes.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:477-481`
    fn timer_tick(&mut self, view: &TelemetryView) {
        self.voltage = voltage_text(view);
        self.current = view.state.as_deref().map_or_else(
            || "0".to_owned(),
            |state| invariant_double(f64::from(state.battery.current)),
        );
    }
}

/// `cs.battery_voltage.ToString()`: a double, so fifteen significant digits.
///
/// The C# keeps its low-pass-filtered voltage as a double and this application's vehicle state
/// keeps it as a float, so past the seventh digit the text can differ; `float.Parse` of either
/// is the same float, so the calibration arithmetic cannot.
fn voltage_text(view: &TelemetryView) -> String {
    view.state.as_deref().map_or_else(
        || "0".to_owned(),
        |state| invariant_double(f64::from(state.battery.voltage)),
    )
}

/// The page, and what it keeps while it is open and after.
#[derive(Debug, Default)]
pub struct BatteryMonitor {
    /// The controls, while the page is open.
    controls: Option<Controls>,
    /// The combo whose list is down.
    dropdown: Option<ComboId>,
    /// The first row the dropped-down list shows, which the wheel moves.
    list_top: usize,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// The last write failure's words, for the status line.
    status: Option<String>,
    /// The Low Battery alert's question, if one is being asked.
    prompt: Option<Prompt>,
    /// The handlers' writes.
    runner: Runner<mp_link::RequestId>,
    /// How the last write went, for a test to read.
    last_write: Option<String>,
    /// Which boxes had the focus last frame.
    focused: [bool; 5],
    /// When the timer last ran; `None` while it is stopped.
    timer: Option<Instant>,
    /// How many times it has run since the page opened.
    ticks: u32,
    /// The vehicle's parameter count when a refresh was asked for.
    refresh_from: Option<u16>,
}

impl BatteryMonitor {
    /// Whether the page is open.
    #[must_use]
    pub const fn is_open(&self) -> bool {
        self.controls.is_some()
    }

    /// The controls, while the page is open.
    #[must_use]
    pub const fn controls(&self) -> Option<&Controls> {
        self.controls.as_ref()
    }

    /// The message box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// The question showing.
    #[must_use]
    pub const fn prompt(&self) -> Option<&Prompt> {
        self.prompt.as_ref()
    }

    /// The question showing, to type into, for a test.
    #[cfg(test)]
    pub fn prompt_mut(&mut self) -> Option<&mut Prompt> {
        self.prompt.as_mut()
    }

    /// How many handlers' writes are still to finish.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.runner.pending()
    }

    /// Opens the page: `Activate`, which reads the alert's settings from `Settings.Instance`.
    pub fn open(&mut self, view: &TelemetryView, lookup: Lookup, settings: &Persisted) {
        let (controls, jobs) = Controls::activate(view, lookup, settings);
        self.timer = (controls.enabled && !controls.startup).then(Instant::now);
        self.controls = Some(controls);
        self.dropdown = None;
        self.focused = [false; 5];
        self.ticks = 0;
        self.refresh_from = None;
        self.runner.push(jobs);
    }

    /// Closes the page: `Deactivate` stops the timer and sets `startup`, so a box left as the
    /// page goes writes nothing. Writes already queued go on, as the C#'s had finished.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:178-182`
    pub fn close(&mut self) {
        self.controls = None;
        self.dropdown = None;
        self.timer = None;
        self.prompt = None;
        self.refresh_from = None;
    }

    /// The page's entry on the setup screen: opens it, or closes it again.
    pub fn toggle(&mut self, telemetry: &Telemetry, settings: &Persisted) {
        if self.is_open() {
            self.close();
        } else {
            self.open(&telemetry.view(), crate::metadata::lookup, settings);
        }
    }

    /// Drops a combo's list down, or back up. A disabled combo does nothing.
    pub fn toggle_dropdown(&mut self, combo: ComboId) {
        let enabled = self
            .controls
            .as_ref()
            .is_some_and(|controls| controls.combo_enabled(combo));
        self.dropdown = if self.dropdown == Some(combo) || !enabled {
            None
        } else {
            Some(combo)
        };
        if let Some(mut list) = self.list() {
            list.open_list();
            self.list_top = list.top_index;
        }
    }

    /// The dropped-down list, as the shared drop-down draws it: the rows, the selected one, and
    /// the first showing.
    #[must_use]
    pub fn list(&self) -> Option<Combo> {
        let combo = self.dropdown?;
        let controls = self.controls.as_ref()?;
        let (options, selected) = match combo {
            ComboId::Monitor => (controls.monitor_options.clone(), controls.monitor),
            ComboId::Sensor => (
                numbered(&SENSORS),
                controls.sensor.and_then(|index| i64::try_from(index).ok()),
            ),
            ComboId::Hardware => (
                numbered(&HW_VERSIONS),
                controls
                    .hardware
                    .and_then(|index| i64::try_from(index).ok()),
            ),
        };
        Some(Combo {
            options,
            selected,
            enabled: true,
            top_index: self.list_top,
            ..Combo::default()
        })
    }

    /// The list back up, without a choice.
    pub fn close_list(&mut self) {
        self.dropdown = None;
    }

    /// The wheel over the dropped-down list.
    pub fn scroll_list(&mut self, lines: i32) {
        if let Some(mut list) = self.list() {
            list.scroll_list(lines);
            self.list_top = list.top_index;
        }
    }

    /// A key in the Sensor or HW Ver box's text: it takes the typing, and the list goes up.
    pub fn type_combo(&mut self, combo: ComboId, event: &KeyDownEvent) -> bool {
        let Some(controls) = self.controls.as_mut() else {
            return false;
        };
        let taken = controls.type_combo(combo, event);
        if taken {
            self.dropdown = None;
        }
        taken
    }

    /// The last write failure's words, taken for the status line.
    pub fn take_status(&mut self) -> Option<String> {
        self.status.take()
    }

    /// Chooses from a combo's list. `SelectedIndexChanged` only for a different row; the monitor
    /// combo then runs the page's handler and `MavlinkComboBox`'s own write, in that order.
    /// `// C#: Controls/MavlinkComboBox.cs:133-199`
    pub fn choose(&mut self, combo: ComboId, key: i64, parameters: &[(String, f64)]) {
        self.dropdown = None;
        let Some(controls) = self.controls.as_mut() else {
            return;
        };
        if !controls.combo_enabled(combo) {
            return;
        }
        let jobs = match combo {
            ComboId::Monitor => {
                if controls.monitor == Some(key)
                    || !controls
                        .monitor_options
                        .iter()
                        .any(|(value, _)| *value == key)
                {
                    return;
                }
                controls.monitor = Some(key);
                let mut jobs = controls.monitor_changed(parameters);
                #[allow(clippy::cast_precision_loss)] // `(float)(int)SelectedValue`
                let value = key as f64;
                jobs.push(Job::Writes {
                    steps: vec![Step::one("BATT_MONITOR", value)].into(),
                    on_throw: OnThrow::Show(COMBO_FAILED),
                    on_false: Some(COMBO_FAILED),
                    refresh: false,
                });
                jobs
            }
            ComboId::Sensor => {
                let Ok(index) = usize::try_from(key) else {
                    return;
                };
                if index >= SENSORS.len() || controls.sensor == Some(index) {
                    return;
                }
                controls.sensor = Some(index);
                controls.sensor_typed = None;
                controls.sensor_changed()
            }
            ComboId::Hardware => {
                let Ok(index) = usize::try_from(key) else {
                    return;
                };
                if index >= HW_VERSIONS.len() || controls.hardware == Some(index) {
                    return;
                }
                controls.hardware = Some(index);
                controls.hardware_typed = None;
                controls.hardware_changed()
            }
        };
        self.runner.push(jobs);
    }

    /// A key in a box. Enter validates the three calibration boxes that listen for it.
    pub fn key(
        &mut self,
        field: Field,
        event: &KeyDownEvent,
        parameters: &[(String, f64)],
    ) -> bool {
        let Some(controls) = self.controls.as_mut() else {
            return false;
        };
        if !controls.field_enabled(field) {
            return false;
        }
        match controls.field_mut(field).key(event) {
            KeyOutcome::Submitted if field.validates_on_enter() => {
                let jobs = controls.validated(field, parameters);
                self.runner.push(jobs);
                true
            }
            KeyOutcome::Changed => true,
            KeyOutcome::Submitted | KeyOutcome::Cancelled | KeyOutcome::Ignored => false,
        }
    }

    /// Enter in a box, for a test: what `PreviewKeyDown` does.
    #[cfg(test)]
    pub fn enter(&mut self, field: Field, parameters: &[(String, f64)]) {
        let Some(controls) = self.controls.as_mut() else {
            return;
        };
        if field.validates_on_enter() {
            let jobs = controls.validated(field, parameters);
            self.runner.push(jobs);
        }
    }

    /// A box losing the focus.
    pub fn leave(&mut self, field: Field, parameters: &[(String, f64)]) {
        let Some(controls) = self.controls.as_mut() else {
            return;
        };
        let jobs = controls.left(field, parameters);
        self.runner.push(jobs);
    }

    /// Replaces a box's text, for a test.
    #[cfg(test)]
    pub fn type_into(&mut self, field: Field, text: &str) {
        if let Some(controls) = self.controls.as_mut() {
            controls.field_mut(field).set(text);
        }
    }

    /// A click on "MP Alert on Low Battery": `Settings.Instance` changed and, when it ends
    /// checked, the first of the three questions asked.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:565-601`
    pub fn click_speech(&mut self, settings: &mut Persisted) {
        let Some(controls) = self.controls.as_mut() else {
            return;
        };
        if !controls.enabled {
            return;
        }
        controls.speech = !controls.speech;
        if controls.startup {
            return;
        }
        let checked = controls.speech;
        // `((CheckBox)sender).Checked.ToString()`: "True" or "False".
        settings.set(
            "speechbatteryenabled",
            if checked { "True" } else { "False" },
        );
        settings.set("speechenable", "True");
        self.prompt = if checked {
            Prompt::at(0, settings)
        } else {
            None
        };
    }

    /// OK on a question: `InputBox` keeping the answer in its list, the page keeping it in its
    /// setting in `Settings.Instance`, and the next asked.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:580-599; ExtLibs/Controls/InputBox.cs:178-184`
    pub fn answer(&mut self, settings: &mut Persisted) {
        let Some(prompt) = self.prompt.take() else {
            return;
        };
        let (title, question) = prompt.text();
        remember_answer(settings, title, question, prompt.field.value());
        if let Some((_, _, key, _)) = SPEECH_PROMPTS.get(prompt.stage) {
            settings.set(key, prompt.field.value());
        }
        self.prompt = Prompt::at(prompt.stage + 1, settings);
    }

    /// Cancel on a question: `return`, the questions after it unasked.
    pub fn cancel(&mut self) {
        self.prompt = None;
    }

    /// A key in the question's box.
    pub fn prompt_key(&mut self, event: &KeyDownEvent, settings: &mut Persisted) -> bool {
        let Some(prompt) = self.prompt.as_mut() else {
            return false;
        };
        match prompt.field.key(event) {
            KeyOutcome::Submitted => self.answer(settings),
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

    /// Reads what running the jobs produced into messages, the status line, the record, and a
    /// refresh. A write's failure - `setParam`'s false or a throw - is the status line's, the
    /// owner's ruling of 2026-09-25; a handler's box, and a `catch` reached by text that does not
    /// parse, are boxes.
    fn absorb(&mut self, events: Vec<Event>, telemetry: Option<&Telemetry>, view: &TelemetryView) {
        let analog = value_of(&view.parameters, "BATT_MONITOR")
            .is_some_and(|monitor| monitor == 3.0 || monitor == 4.0);
        let shown = |on_throw: OnThrow| match on_throw {
            OnThrow::Show(text) => Some(text),
            OnThrow::ShowIfAnalog(text) => analog.then_some(text),
        };
        for event in events {
            match event {
                Event::Written {
                    name,
                    value,
                    outcome,
                } => {
                    self.last_write = Some(match outcome {
                        RequestOutcome::Accepted { value: echoed } => format!(
                            "{name} {} accepted",
                            echoed.map_or(value, mp_params::ParamValue::as_f64)
                        ),
                        _ => format!("{name} {value} unchanged"),
                    });
                }
                Event::Status(text) => self.status = Some(text.trim().to_owned()),
                Event::Refused(text) => self.status = Some(text.to_owned()),
                // Text that does not parse is an avoidable error, and the status line's (the
                // owner's rule of 2026-09-25, and his word on this box that evening).
                Event::Unparsed(on_throw) => {
                    self.last_write = Some("a number that does not parse failed".to_owned());
                    if let Some(text) = shown(on_throw) {
                        self.status = Some(text.to_owned());
                    }
                }
                Event::Threw { what, on_throw } => {
                    self.last_write = Some(format!("{what} failed"));
                    if let Some(text) = shown(on_throw) {
                        self.status = Some(text.to_owned());
                    }
                }
                Event::Refresh => {
                    if let Some(telemetry) = telemetry {
                        telemetry.download_parameters();
                    }
                    self.refresh_from = Some(view.parameters_expected);
                }
            }
        }
    }

    /// Once a frame: closes the page when the setup screen goes, validates a box the focus has
    /// left, runs the timer, moves the writes on, and re-activates once a refresh has brought the
    /// vehicle's new parameters.
    ///
    /// The refresh is `getParamList` then `Activate`: here the download is asked for and the page
    /// activated again when the table is whole with a different count - the parameters
    /// `BATT_MONITOR` switches on. A refresh that brings none leaves the page as it is until it is
    /// opened again.
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        focused: [bool; 5],
        on_setup: bool,
        settings: &Persisted,
    ) {
        if !on_setup && self.is_open() {
            self.close();
        }
        let now = Instant::now();
        for field in Field::ALL {
            let index = field.index();
            let was = self.focused.get(index).copied().unwrap_or(false);
            let is = focused.get(index).copied().unwrap_or(false);
            if was && !is {
                self.leave(field, &view.parameters);
            }
        }
        self.focused = focused;
        if let (Some(last), Some(controls)) = (self.timer, self.controls.as_mut())
            && now.duration_since(last) >= TIMER_INTERVAL
        {
            controls.timer_tick(view);
            self.timer = Some(now);
            self.ticks = self.ticks.saturating_add(1);
        }
        let events = self.runner.advance(telemetry);
        self.absorb(events, Some(telemetry), view);
        if let Some(from) = self.refresh_from
            && self.is_open()
            && view.parameters_expected != from
            && !view.parameters.is_empty()
            && view.parameters.len() >= usize::from(view.parameters_expected)
        {
            self.refresh_from = None;
            self.open(view, crate::metadata::lookup, settings);
        }
    }
}

/// The text boxes' focus handles, in [`Field::ALL`] order, and the question's.
pub struct Focus {
    fields: [FocusHandle; 5],
    prompt: FocusHandle,
    /// The Sensor and HW Ver boxes' edit text.
    sensor: FocusHandle,
    hardware: FocusHandle,
}

impl Focus {
    /// New handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            fields: [
                cx.focus_handle(),
                cx.focus_handle(),
                cx.focus_handle(),
                cx.focus_handle(),
                cx.focus_handle(),
            ],
            prompt: cx.focus_handle(),
            sensor: cx.focus_handle(),
            hardware: cx.focus_handle(),
        }
    }

    /// A `DropDown` combo's edit text's handle.
    const fn combo(&self, combo: ComboId) -> Option<&FocusHandle> {
        match combo {
            ComboId::Sensor => Some(&self.sensor),
            ComboId::Hardware => Some(&self.hardware),
            ComboId::Monitor => None,
        }
    }

    /// Which boxes have the focus.
    #[must_use]
    pub fn focused(&self, window: &Window) -> [bool; 5] {
        self.fields
            .each_ref()
            .map(|handle| handle.is_focused(window))
    }
}

/// Facts a UI test asserts on: what the page shows, how its writes went, and what the vehicle
/// holds.
pub fn record_facts(battery: &BatteryMonitor, view: &TelemetryView) {
    use crate::facts::record;
    record("config.battery.open", battery.is_open());
    record(
        "config.battery.message",
        battery
            .message()
            .map_or("none", |message| message.text.as_str()),
    );
    record(
        "config.battery.prompt",
        battery.prompt().map_or("none", |prompt| prompt.text().1),
    );
    record(
        "config.battery.write",
        battery.last_write.as_deref().unwrap_or("none"),
    );
    record("config.battery.writes.pending", battery.pending());
    record("config.battery.readouts", battery.ticks);
    let controls = battery.controls();
    let text = |field: Field| {
        controls.map_or_else(String::new, |controls| {
            controls.field(field).value().to_owned()
        })
    };
    record(
        "config.battery.enabled",
        controls.is_some_and(|controls| controls.enabled),
    );
    record(
        "config.battery.monitor",
        controls
            .and_then(|controls| controls.monitor)
            .map_or_else(|| "none".to_owned(), |value| value.to_string()),
    );
    record(
        "config.battery.monitor.text",
        controls.map_or("", Controls::monitor_text),
    );
    record(
        "config.battery.monitor.options",
        controls.map_or(0, |controls| controls.monitor_options.len()),
    );
    record(
        "config.battery.sensor",
        controls
            .and_then(|controls| controls.sensor)
            .map_or_else(|| "none".to_owned(), |index| index.to_string()),
    );
    record(
        "config.battery.sensor.enabled",
        controls.is_some_and(|controls| controls.combo_enabled(ComboId::Sensor)),
    );
    record(
        "config.battery.sensor.text",
        controls.map_or("", |controls| controls.combo_text(ComboId::Sensor)),
    );
    record(
        "config.battery.hwversion.text",
        controls.map_or("", |controls| controls.combo_text(ComboId::Hardware)),
    );
    record(
        "config.battery.list",
        battery.dropdown.map_or("none", ComboId::id),
    );
    record(
        "config.battery.hwversion",
        controls
            .and_then(|controls| controls.hardware)
            .map_or_else(|| "none".to_owned(), |index| index.to_string()),
    );
    record(
        "config.battery.hwversion.enabled",
        controls.is_some_and(|controls| controls.combo_enabled(ComboId::Hardware)),
    );
    record("config.battery.capacity", text(Field::Capacity));
    record("config.battery.measured", text(Field::Measured));
    record("config.battery.divider", text(Field::Divider));
    record("config.battery.meascurrent", text(Field::MeasuredCurrent));
    record("config.battery.ampspervolt", text(Field::AmpsPerVolt));
    record(
        "config.battery.voltage",
        controls.map_or("", |controls| controls.voltage.as_str()),
    );
    record(
        "config.battery.current",
        controls.map_or("", |controls| controls.current.as_str()),
    );
    for field in [Field::Measured, Field::Divider, Field::AmpsPerVolt] {
        record(
            format!("config.battery.{}.enabled", field_key(field)),
            controls.is_some_and(|controls| controls.field_enabled(field)),
        );
    }
    record(
        "config.battery.calibration.enabled",
        controls.is_some_and(|controls| controls.enabled && controls.calibration_enabled),
    );
    record(
        "config.battery.speech",
        controls.is_some_and(|controls| controls.speech),
    );
    for name in [
        "BATT_MONITOR",
        "BATT_CAPACITY",
        "BATT_VOLT_PIN",
        "BATT_CURR_PIN",
        "BATT_VOLT_MULT",
        "BATT_AMP_PERVLT",
        "BATT_AMP_PERVOLT",
        "VOLT_DIVIDER",
        "AMP_PER_VOLT",
    ] {
        if let Some(value) = value_of(&view.parameters, name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

/// A box's key in the facts.
const fn field_key(field: Field) -> &'static str {
    match field {
        Field::Capacity => "capacity",
        Field::Measured => "measured",
        Field::Divider => "divider",
        Field::MeasuredCurrent => "meascurrent",
        Field::AmpsPerVolt => "ampspervolt",
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

/// A label at its `.resx` `Location`.
fn label(x: f32, y: f32, text: &'static str, enabled: bool) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .text_xs()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(text)
}

/// A combo box at its `.resx` place. The monitor's, a `DropDownList`, drops its list on a click
/// anywhere. The Sensor and HW Ver boxes, `DropDown`s, take typing in their text - `edit` is its
/// focus handle and whether it has the focus - and drop their list from the arrow,
/// `<id>-button`. A click that drops a list takes the focus from a text box first, as clicking a
/// WinForms combo does, so the box is validated before the choice; a click into the text takes
/// it too, and the box left is validated the same way.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.Designer.cs:72-87, 107-118, 135-151`
fn combo_box(
    combo: ComboId,
    text: String,
    enabled: bool,
    (x, y, width, height): (f32, f32, f32, f32),
    edit: Option<(&FocusHandle, bool)>,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let drop_list =
        move |this: &mut MissionPlanner, window: &mut Window, cx: &mut Context<MissionPlanner>| {
            window.blur(cx);
            this.battery_monitor.toggle_dropdown(combo);
            cx.notify();
        };
    let arrow = div().flex_shrink_0().px_1().child("▾");
    let base = crate::probe::measured(combo.id(), div())
        .id(combo.id())
        .size_full()
        .flex()
        .items_center()
        .justify_between()
        .rounded_sm()
        .border_1()
        .text_xs();
    let base = match edit {
        _ if !enabled => base
            .border_color(rgb(theme::BORDER))
            .bg(rgb(theme::PANEL))
            .text_color(rgb(theme::DIM))
            .child(div().px_1().truncate().child(text))
            .child(arrow),
        None => base
            .border_color(rgb(theme::BORDER))
            .bg(rgb(theme::ACTION))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.border_color(rgb(theme::ACCENT)))
            .child(div().px_1().truncate().child(text))
            .child(arrow)
            .on_click(cx.listener(move |this, _event, window, cx| drop_list(this, window, cx))),
        Some((handle, focused)) => {
            let focus = handle.clone();
            let button = format!("{}-button", combo.id());
            base.border_color(rgb(if focused {
                theme::ACCENT
            } else {
                theme::BORDER
            }))
            .bg(rgb(theme::ACTION))
            .text_color(rgb(theme::TEXT))
            .track_focus(handle)
            .key_context("TextField")
            .on_key_down(cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                if this.battery_monitor.type_combo(combo, event) {
                    cx.notify();
                }
            }))
            .child(
                div()
                    .flex_1()
                    .flex()
                    .items_center()
                    .px_1()
                    .overflow_hidden()
                    .whitespace_nowrap()
                    .cursor_text()
                    .child(text)
                    .children(focused.then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT))))
                    .on_mouse_down(
                        gpui::MouseButton::Left,
                        cx.listener(move |this, _event, window, cx| {
                            this.battery_monitor.close_list();
                            focus.focus(window, cx);
                            // Kept: the window's own focusable element takes the keyboard on the
                            // same press unless told not to (gpui's div.rs).
                            window.prevent_default();
                            cx.notify();
                        }),
                    ),
            )
            .child(
                crate::probe::measured(button.clone(), arrow)
                    .id(SharedString::from(button))
                    .cursor_pointer()
                    .hover(|style| style.text_color(rgb(theme::ACCENT)))
                    .on_click(
                        cx.listener(move |this, _event, window, cx| drop_list(this, window, cx)),
                    ),
            )
        }
    };
    at(x, y, width, height).child(base).into_any_element()
}

/// A text box at its `.resx` place: typing and a caret while it has the focus, dimmed and inert
/// while it is disabled. `read_only` for the two the vehicle fills.
fn text_box(
    id: &'static str,
    text: String,
    focus: Option<(&FocusHandle, Field)>,
    enabled: bool,
    (x, y, width, height): (f32, f32, f32, f32),
    window_focused: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let base = crate::probe::measured(id, div())
        .id(id)
        .size_full()
        .flex()
        .items_center()
        .px_1()
        .rounded_sm()
        .border_1()
        .text_xs()
        .overflow_hidden()
        // The C#'s double as it prints - "12.6000003814697", its filter's `0.4f` and `0.6f`
        // leaving the voltage unround (CurrentState.cs:1295-1301) - is longer than the 76-wide
        // box: its start shows, and an ellipsis says there is more, where it ran past the box's
        // edge (the layout guard on the owner's Mac, 2026-10-05).
        .child(
            div()
                .flex_1()
                .min_w(px(0.0))
                .overflow_hidden()
                .whitespace_nowrap()
                .text_ellipsis()
                .child(text),
        );
    let base = match focus {
        Some((handle, field)) if enabled => base
            .track_focus(handle)
            .key_context("TextField")
            .on_key_down(cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                let view = this.telemetry.view();
                if this.battery_monitor.key(field, event, &view.parameters) {
                    cx.notify();
                }
            }))
            .bg(rgb(theme::ACTION))
            .border_color(rgb(if window_focused {
                theme::ACCENT
            } else {
                theme::BORDER
            }))
            .text_color(rgb(theme::TEXT))
            .cursor_text()
            .children(window_focused.then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT)))),
        _ => base
            .bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM })),
    };
    at(x, y, width, height).child(base).into_any_element()
}

/// The page, laid out as `ConfigBatteryMonitoring.resx` lays it out.
pub fn page(
    battery: &BatteryMonitor,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let Some(controls) = battery.controls() else {
        return div().into_any_element();
    };
    let enabled = controls.enabled;
    let mut body = div().relative().w(px(512.0)).h(px(322.0));

    // `pictureBox5`'s photograph, held in the `.resx` as base64: the same bytes as
    // `Resources.BR_APMPWRDEAN_2` (`Resources/BR-APMPWRDEAN-2.jpg`), which is drawn, zoomed.
    // C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.Designer.cs:123; ConfigBatteryMonitoring.resx:315, 627-628
    body = body.child(picture(
        "battery",
        "pictureBox5",
        (3.0, 41.0, 97.0, 75.0),
        "BR_APMPWRDEAN_2",
        crate::pictures::Layout::Zoom,
    ));

    // The three combos and their labels, left column.
    let edit = |combo: ComboId| {
        focus
            .combo(combo)
            .map(|handle| (handle, handle.is_focused(window)))
    };
    body = body
        .child(label(106.0, 45.0, "Monitor", enabled))
        .child(combo_box(
            ComboId::Monitor,
            controls.combo_text(ComboId::Monitor).to_owned(),
            controls.combo_enabled(ComboId::Monitor),
            (160.0, 41.0, 162.0, 21.0),
            None,
            cx,
        ))
        .child(label(106.0, 71.0, "Sensor", enabled))
        .child(combo_box(
            ComboId::Sensor,
            controls.combo_text(ComboId::Sensor).to_owned(),
            controls.combo_enabled(ComboId::Sensor),
            (160.0, 68.0, 162.0, 20.0),
            edit(ComboId::Sensor),
            cx,
        ))
        .child(label(106.0, 98.0, "HW Ver", enabled))
        .child(combo_box(
            ComboId::Hardware,
            controls.combo_text(ComboId::Hardware).to_owned(),
            controls.combo_enabled(ComboId::Hardware),
            (160.0, 95.0, 162.0, 20.0),
            edit(ComboId::Hardware),
            cx,
        ));

    let focused = battery.focused;
    let field_box =
        |field: Field, place: (f32, f32, f32, f32), cx: &mut Context<MissionPlanner>| {
            let handle = focus.fields.get(field.index());
            text_box(
                field.id(),
                controls.field(field).value().to_owned(),
                handle.map(|handle| (handle, field)),
                controls.field_enabled(field),
                place,
                focused.get(field.index()).copied().unwrap_or(false),
                cx,
            )
        };

    // Capacity, and the Low Battery alert, right column.
    body = body
        .child(label(330.0, 44.0, "Battery Capacity", enabled))
        .child(field_box(Field::Capacity, (425.0, 45.0, 50.0, 19.0), cx))
        .child(label(481.0, 45.0, "mAh", enabled))
        .child(speech_box(controls.speech, enabled, cx));

    // groupBox4, "Calibration".
    let group_enabled = enabled && controls.calibration_enabled;
    let row = |x: f32, y: f32, text: &'static str| label(x, y, text, group_enabled);
    let group = at(19.0, 142.0, 276.0, 164.0)
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
                .text_color(rgb(theme::DIM))
                .child("Calibration"),
        )
        .child(row(5.0, 16.0, "1. Measured battery voltage:"))
        .child(field_box(Field::Measured, (183.0, 13.0, 76.0, 19.0), cx))
        .child(row(5.0, 38.0, "2. Battery voltage (Calced):"))
        .child(text_box(
            "battery-voltage",
            controls.voltage.clone(),
            None,
            group_enabled && controls.voltage_enabled,
            (183.0, 35.0, 76.0, 19.0),
            false,
            cx,
        ))
        .child(row(5.0, 59.0, "3. Voltage divider (Calced):"))
        .child(field_box(Field::Divider, (183.0, 56.0, 76.0, 19.0), cx))
        .child(row(6.0, 80.0, "4. Measured current:"))
        .child(field_box(
            Field::MeasuredCurrent,
            (183.0, 77.0, 76.0, 19.0),
            cx,
        ))
        .child(row(6.0, 101.0, "5. Current (Calced)"))
        .child(text_box(
            "battery-current",
            controls.current.clone(),
            None,
            group_enabled,
            (183.0, 98.0, 76.0, 19.0),
            false,
            cx,
        ))
        .child(row(6.0, 122.0, "6. Amperes per volt:"))
        .child(field_box(
            Field::AmpsPerVolt,
            (183.0, 119.0, 76.0, 19.0),
            cx,
        ));
    body = body.child(group);

    // A dropped-down list, deferred and anchored so it lies over the page, `DropDownWidth` 200.
    // C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.Designer.cs:72, 108, 135
    if let (Some(combo), Some(list)) = (battery.dropdown, battery.list()) {
        let y = match combo {
            ComboId::Monitor => 41.0 + 21.0,
            ComboId::Sensor => 68.0 + 20.0,
            ComboId::Hardware => 95.0 + 20.0,
        };
        body = body.child(dropdown(
            combo.id(),
            &list,
            (160.0, y, 200.0),
            move |this, key| {
                let view = this.telemetry.view();
                this.battery_monitor.choose(combo, key, &view.parameters);
            },
            |this, lines| this.battery_monitor.scroll_list(lines),
            cx,
        ));
    }

    panel(TITLE, body).into_any_element()
}

/// A fixed list as the combo's rows, each keyed by its index.
fn numbered(items: &[&str]) -> Vec<(i64, String)> {
    items
        .iter()
        .enumerate()
        .filter_map(|(index, text)| {
            i64::try_from(index)
                .ok()
                .map(|key| (key, (*text).to_owned()))
        })
        .collect()
}

/// `CHK_speechbattery`, "MP Alert on Low Battery", at 333,67.
fn speech_box(checked: bool, enabled: bool, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let square = div()
        .size(px(11.0))
        .flex_shrink_0()
        .rounded_sm()
        .border_1()
        .border_color(rgb(if enabled {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(if checked {
            if enabled { theme::ACCENT } else { theme::DIM }
        } else {
            theme::BG
        }));
    let body = crate::probe::measured("battery-speech", div())
        .id("battery-speech")
        .flex()
        .items_center()
        .gap_1()
        .text_xs()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(square)
        .child("MP Alert on Low Battery");
    let body = if enabled {
        body.cursor_pointer()
            .on_click(cx.listener(|this, _event, window, cx| {
                window.blur(cx);
                this.battery_monitor.click_speech(&mut this.persisted);
                if this.battery_monitor.prompt().is_some() {
                    this.battery_focus.prompt.focus(window, cx);
                }
                cx.notify();
            }))
            .into_any_element()
    } else {
        body.into_any_element()
    };
    div()
        .absolute()
        .left(px(333.0))
        .top(px(67.0))
        .child(body)
        .into_any_element()
}

/// The message box or the question showing, drawn over the whole window: both are modal.
pub fn overlay(
    battery: &BatteryMonitor,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let dialog = if let Some(message) = battery.message() {
        crate::probe::measured("battery-message", div())
            .flex()
            .flex_col()
            .gap_2()
            .w(px(340.0))
            .p_3()
            .bg(rgb(theme::PANEL))
            .border_1()
            .border_color(rgb(theme::ALERT))
            .rounded_md()
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
                    .child(message.text.trim_end().to_owned()),
            )
            .child(div().flex().justify_end().child(action(
                "battery-message-ok",
                "OK",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.battery_monitor.dismiss_message();
                    cx.notify();
                }),
            )))
    } else {
        let prompt = battery.prompt()?;
        let (title, question) = prompt.text();
        let focused = focus.prompt.is_focused(window);
        crate::probe::measured("battery-prompt", div())
            .flex()
            .flex_col()
            .gap_2()
            .w(px(340.0))
            .p_3()
            .bg(rgb(theme::PANEL))
            .border_1()
            .border_color(rgb(theme::ACCENT))
            .rounded_md()
            .child(div().text_xs().text_color(rgb(theme::DIM)).child(title))
            .child(div().text_sm().text_color(rgb(theme::TEXT)).child(question))
            .child(crate::textfield::text_field(
                "battery-prompt-value",
                &prompt.field,
                &focus.prompt,
                focused,
                px(310.0),
                cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                    if this.battery_monitor.prompt_key(event, &mut this.persisted) {
                        cx.notify();
                    }
                }),
            ))
            .child(
                div()
                    .flex()
                    .justify_end()
                    .gap_2()
                    .child(action(
                        "battery-prompt-ok",
                        "OK",
                        theme::ACCENT,
                        true,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.battery_monitor.answer(&mut this.persisted);
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "battery-prompt-cancel",
                        "Cancel",
                        theme::DIM,
                        true,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.battery_monitor.cancel();
                            cx.notify();
                        }),
                    )),
            )
    };
    let size = window.viewport_size();
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(
                    div()
                        .id("battery-dialog-backdrop")
                        .w(size.width)
                        .h(size.height)
                        .flex()
                        .items_center()
                        .justify_center()
                        .occlude()
                        .child(dialog),
                ),
        )
        .with_priority(2)
        .into_any_element(),
    )
}

#[cfg(test)]
mod tests {
    use mp_os::fs::FsExt as _;
    use std::cell::RefCell;
    use std::sync::Arc;

    use super::*;

    /// The bundled documentation, so no test depends on what another has fetched.
    fn bundled(name: &str) -> Option<&'static mp_params::ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    /// A connected vehicle holding these parameters, its battery at 12.6 V and 1.23 A.
    fn view_with(parameters: &[(&str, f64)]) -> TelemetryView {
        let mut view = TelemetryView::disconnected("tcp:127.0.0.1:5760");
        view.connected = true;
        view.vehicle = Some(mp_vehicle::VehicleId::new(1, 1));
        view.parameters = parameters
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect();
        view.parameters_expected = u16::try_from(view.parameters.len()).unwrap();
        let mut state = mp_vehicle::VehicleState::default();
        state.battery.voltage = 12.6;
        state.battery.current = 1.23;
        view.state = Some(Arc::new(state));
        view
    }

    /// SITL's copter, as `tools/sitl/params/copter.parm` and the firmware's defaults leave it.
    const SITL: [(&str, f64); 7] = [
        ("BATT_MONITOR", 4.0),
        ("BATT_CAPACITY", 3300.0),
        ("BATT_VOLT_PIN", 13.0),
        ("BATT_CURR_PIN", 12.0),
        ("BATT_VOLT_MULT", 10.1),
        ("BATT_AMP_PERVLT", 17.0),
        ("BATT_AMP_OFFSET", 0.0),
    ];

    fn open(parameters: &[(&str, f64)]) -> (BatteryMonitor, TelemetryView) {
        let view = view_with(parameters);
        let mut battery = BatteryMonitor::default();
        battery.open(&view, bundled, &Persisted::at(None));
        (battery, view)
    }

    /// The jobs the handlers have queued, taken off the page's runner.
    fn drain(battery: &mut BatteryMonitor) -> Vec<Job> {
        battery.runner.queue.drain(..).collect()
    }

    /// The writes the jobs would make, flattened: each step's names and value.
    fn steps(jobs: &[Job]) -> Vec<(Vec<&'static str>, f64)> {
        jobs.iter()
            .flat_map(Job::steps)
            .map(|step| (step.names, step.value))
            .collect()
    }

    /// A stand-in vehicle: it holds some names, answers a write to one of them at once, refuses
    /// the rest as `setParam` does, and times out on the one it is told to.
    struct Vehicle {
        held: Vec<&'static str>,
        timeout: Option<&'static str>,
        silent: bool,
        written: RefCell<Vec<(String, f64)>>,
    }

    impl Vehicle {
        fn holding(held: &[&'static str]) -> Self {
            Self {
                held: held.to_vec(),
                timeout: None,
                silent: false,
                written: RefCell::new(Vec::new()),
            }
        }

        fn written(&self) -> Vec<(String, f64)> {
            self.written.borrow().clone()
        }
    }

    impl ParamWriter for Vehicle {
        type Handle = usize;

        fn write(&self, name: &str, value: f64) -> Option<usize> {
            let mut written = self.written.borrow_mut();
            written.push((name.to_owned(), value));
            Some(written.len() - 1)
        }

        fn progress(&self, handle: usize) -> Progress {
            if self.silent {
                return Progress::Waiting;
            }
            let written = self.written.borrow();
            let Some((name, _)) = written.get(handle) else {
                return Progress::Lost;
            };
            if Some(name.as_str()) == self.timeout {
                Progress::Finished(RequestOutcome::TimedOut)
            } else if self.held.contains(&name.as_str()) {
                Progress::Finished(RequestOutcome::Accepted { value: None })
            } else {
                Progress::Finished(RequestOutcome::UnknownParameter)
            }
        }
    }

    /// Only what the vehicle took, in order.
    fn taken(vehicle: &Vehicle) -> Vec<(String, f64)> {
        vehicle
            .written()
            .into_iter()
            .filter(|(name, _)| vehicle.held.contains(&name.as_str()))
            .collect()
    }

    #[test]
    fn a_float_is_written_as_net_frameworks_seven_digits() {
        assert_eq!(float_text(4.127_115), "4.127115");
        assert_eq!(float_text(13.6612), "13.6612");
        assert_eq!(float_text(10.1), "10.1");
        assert_eq!(float_text(3300.0), "3300");
        assert_eq!(float_text(17.0), "17");
        assert_eq!(float_text(-2.5), "-2.5");
        assert_eq!(float_text(0.0), "0");
        assert_eq!(float_text(-0.0), "0", "Framework drops the sign of zero");
        assert_eq!(float_text(1_234_567.0), "1234567");
        assert_eq!(float_text(12_345_678.0), "1.234568E+07");
        assert_eq!(float_text(0.0001), "0.0001");
        assert_eq!(float_text(0.000_01), "1E-05");
        // 1.99999988: seven nines and an eight, which carries into a new leading digit.
        assert_eq!(float_text(f32::from_bits(0x3FFF_FFFF)), "2");
        assert_eq!(float_text(10_000_000.0), "1E+07");
        assert_eq!(float_text(0.1), "0.1", "0.100000001490116 to seven digits");
        // 2^-11 is 0.00048828125 exactly, a half at the eighth digit: away from zero.
        assert_eq!(float_text(0.000_488_281_25), "0.0004882813");
        assert_eq!(float_text(f32::INFINITY), "Infinity");
        assert_eq!(float_text(f32::NAN), "NaN");
    }

    #[test]
    fn text_is_read_as_float_parse_reads_it() {
        assert_eq!(parse_float("10.1"), Some(10.1));
        assert_eq!(parse_float(" 3,300 "), Some(3300.0), "AllowThousands");
        assert_eq!(parse_float("1.5e3"), Some(1500.0));
        assert_eq!(parse_float("-0.5"), Some(-0.5));
        assert_eq!(parse_float(".5"), Some(0.5));
        assert_eq!(parse_float("Infinity"), Some(f32::INFINITY));
        assert!(parse_float("NaN").is_some_and(f32::is_nan));
        for bad in [
            "", " ", "abc", "1.2.3", "inf", "nan", "1.000,5", "12V", "1e40",
        ] {
            assert_eq!(parse_float(bad), None, "{bad:?}");
        }
    }

    #[test]
    fn a_parameter_reads_as_its_float() {
        assert_eq!(param_text(10.1), "10.1");
        assert_eq!(param_text(3300.0), "3300");
        assert_eq!(param_text(-1.0), "-1");
        // What the link keeps for a float the vehicle echoed: seven significant digits of it.
        let echoed = mp_params::round_to_significant_digits(f64::from(13.6612_f32), 7);
        assert_eq!(param_text(echoed), "13.6612");
    }

    /// The sensor table, as the handler writes it into the boxes: divider, then amps per volt.
    #[test]
    fn the_sensor_presets_are_the_csharps() {
        let expected = [
            ("4.127115", "13.6612"),
            ("15.70105", "27.3224"),
            ("15.70105", "54.64481"),
            ("10.10101", "18.0018"),
            ("12.02", "17"),
            ("12.02", "24"),
            ("12.02", "39.877"),
            ("18", "24"),
            ("18.182", "36.364"),
        ];
        assert_eq!(preset(0), None, "Other leaves the boxes alone");
        for (index, (divider, amps)) in expected.iter().enumerate() {
            let sensor = index + 1;
            assert_eq!(
                preset(sensor),
                Some(((*divider).to_owned(), (*amps).to_owned())),
                "{}",
                SENSORS[sensor]
            );
        }
        assert_eq!(preset(10), None);
    }

    /// Every preset, written to the vehicle and read back by `Activate`, is recognised as the
    /// sensor it came from - the computed AttoPilot and 3DR values included, which is the test
    /// that the float arithmetic is the C#'s: its detection strings are those floats' texts.
    #[test]
    fn a_preset_written_and_read_back_is_recognised() {
        for (sensor, name) in SENSORS.iter().enumerate().skip(1) {
            let (divider, amps) = preset(sensor).unwrap();
            let echo = |text: &str| {
                let written = parse_float(text).unwrap();
                param_text(mp_params::round_to_significant_digits(
                    f64::from(written),
                    7,
                ))
            };
            assert_eq!(
                detect_sensor(&echo(&amps), &echo(&divider)),
                sensor,
                "{name}"
            );
        }
        assert_eq!(detect_sensor("17", "10.1"), 0, "SITL's is Other");
        assert_eq!(detect_sensor("", ""), 0);
    }

    #[test]
    fn each_board_writes_the_pins_that_read_back_as_it() {
        for (index, (volt, curr)) in HW_PINS.iter().enumerate() {
            assert_eq!(
                detect_hw(*volt, Some(*curr)),
                HwDetect::Index(index),
                "{}",
                HW_VERSIONS[index]
            );
        }
        assert_eq!(HW_PINS[2], (13.0, 12.0), "APM2.5+, SITL's pins");
        assert_eq!(HW_PINS[4], (2.0, 3.0), "The Cube or Pixhawk");
        assert_eq!(HW_PINS[10], (8.0, 4.0), "Pixhawk 6C");
        assert_eq!(detect_hw(10.0, Some(11.0)), HwDetect::Index(5));
        assert_eq!(detect_hw(10.0, Some(-1.0)), HwDetect::Index(6));
        assert_eq!(detect_hw(10.0, None), HwDetect::Throws);
        assert_eq!(detect_hw(5.0, Some(4.0)), HwDetect::Unmatched);
    }

    /// The divider from a measured voltage, and the amps per volt from a measured current, in
    /// floats - the expected texts worked out in IEEE single precision.
    #[test]
    fn the_calibration_arithmetic_is_in_floats() {
        let divider = |measured: &str, divider: &str, reading: &str| {
            calibrate(
                parse_float(measured).unwrap(),
                parse_float(divider).unwrap(),
                parse_float(reading).unwrap(),
            )
            .map(float_text)
        };
        assert_eq!(divider("12.5", "10.1", "12.6").as_deref(), Some("10.01984"));
        assert_eq!(
            divider("11.9", "10.1", "12.3456").as_deref(),
            Some("9.735453")
        );
        // The reading the voltage box shows for a 12.6 V float: the same divider back.
        assert_eq!(
            divider("12.6", "10.1", "12.6000003814697").as_deref(),
            Some("10.1")
        );
        assert_eq!(divider("12.6", "10.1", "0"), None, "no reading, no write");
        assert_eq!(divider("10", "17", "8.5").as_deref(), Some("20"));
        assert_eq!(
            divider("2.5", "17", "1.23000001907349").as_deref(),
            Some("34.55285")
        );
    }

    /// What `Activate` makes of SITL's copter, from a real parameter dump: an analog voltage and
    /// current monitor on the APM2.5 pins, "Other" sensor, and the writes of the two handlers it
    /// calls - `BATT_MONITOR`, then the amps per volt and the divider under the names the vehicle
    /// has.
    #[test]
    fn activating_on_the_sitl_dump_reads_what_the_gui_script_expects() {
        let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/params/sitl-copter.param");
        let file = mp_params::param_file::ParamFile::load(&fixture).expect("the SITL dump");
        let mut view = view_with(&[]);
        view.parameters = file
            .iter()
            .map(|(name, value)| (name.to_owned(), value))
            .collect();
        let (controls, jobs) = Controls::activate(&view, bundled, &Persisted::at(None));
        assert!(controls.enabled);
        assert!(!controls.startup);
        assert_eq!(controls.monitor, Some(4));
        assert_eq!(controls.monitor_text(), "Analog Voltage and Current");
        assert!(
            controls
                .monitor_options
                .contains(&(3, "Analog Voltage Only".to_owned()))
        );
        assert_eq!(controls.sensor, Some(0));
        assert_eq!(controls.hardware, Some(2));
        assert_eq!(controls.capacity.value(), "3300");
        assert_eq!(controls.divider.value(), "10.1");
        assert_eq!(controls.amps_per_volt.value(), "17");
        assert_eq!(controls.voltage, invariant_double(f64::from(12.6_f32)));
        assert_eq!(controls.measured.value(), controls.voltage);
        assert_eq!(controls.current, "", "until the timer first runs");
        assert!(!controls.voltage_enabled, "the monitor handler greys it");
        for field in Field::ALL {
            assert!(controls.field_enabled(field), "{field:?}");
        }
        for combo in [ComboId::Monitor, ComboId::Sensor, ComboId::Hardware] {
            assert!(controls.combo_enabled(combo), "{combo:?}");
        }
        assert_eq!(
            steps(&jobs),
            vec![
                (vec!["BATT_MONITOR"], 4.0),
                (AMPS_NAMES.to_vec(), 17.0),
                (DIVIDER_NAMES.to_vec(), f64::from(10.1_f32)),
            ]
        );

        // And through a vehicle with SITL's names, each lands on the one it has.
        let vehicle = Vehicle::holding(&["BATT_MONITOR", "BATT_AMP_PERVLT", "BATT_VOLT_MULT"]);
        let mut runner = Runner::default();
        runner.push(jobs);
        let events = runner.advance(&vehicle);
        assert_eq!(runner.pending(), 0);
        assert_eq!(
            taken(&vehicle),
            vec![
                ("BATT_MONITOR".to_owned(), 4.0),
                ("BATT_AMP_PERVLT".to_owned(), 17.0),
                ("BATT_VOLT_MULT".to_owned(), f64::from(10.1_f32)),
            ]
        );
        assert!(
            !events
                .iter()
                .any(|event| matches!(event, Event::Status(_)))
        );
    }

    /// Older firmwares' names, read in the C#'s order: the oldest name the vehicle has fills the
    /// box, and the write goes to the oldest name it has.
    #[test]
    fn each_generation_of_names_is_read_and_written() {
        // ArduPilot 3.x: VOLT_DIVIDER and AMP_PER_VOLT beside nothing newer.
        let (mut battery, view) = open(&[
            ("BATT_MONITOR", 4.0),
            ("VOLT_DIVIDER", 15.70105),
            ("AMP_PER_VOLT", 27.3224),
        ]);
        let controls = battery.controls().unwrap();
        assert_eq!(controls.divider.value(), "15.70105");
        assert_eq!(controls.amps_per_volt.value(), "27.3224");
        assert_eq!(controls.sensor, Some(2), "AttoPilot 90A");
        // No pins: Activate turns the board combo off and selects nothing, and the monitor
        // handler it then calls turns it back on for monitor 4 (:108-168, :231-237).
        assert_eq!(controls.hardware, None);
        assert!(controls.combo_enabled(ComboId::Hardware));
        drain(&mut battery);
        // A preset sensor leaves the boxes disabled, so choose Other to type.
        battery.choose(ComboId::Sensor, 0, &view.parameters);
        drain(&mut battery);
        battery.type_into(Field::Divider, "15.5");
        battery.enter(Field::Divider, &view.parameters);
        let old = Vehicle::holding(&["VOLT_DIVIDER", "AMP_PER_VOLT"]);
        let mut runner = Runner::default();
        runner.push(drain(&mut battery));
        runner.advance(&old);
        assert_eq!(taken(&old), vec![("VOLT_DIVIDER".to_owned(), 15.5)]);

        // A vehicle with the middle name and the newest: the middle one wins both ways.
        let (mut battery, view) = open(&[
            ("BATT_MONITOR", 4.0),
            ("BATT_AMP_PERVLT", 17.0),
            ("BATT_AMP_PERVOLT", 24.0),
            ("BATT_VOLT_MULT", 12.02),
        ]);
        assert_eq!(battery.controls().unwrap().amps_per_volt.value(), "24");
        assert_eq!(battery.controls().unwrap().sensor, Some(6));
        drain(&mut battery);
        battery.choose(ComboId::Sensor, 0, &view.parameters);
        let middle = Vehicle::holding(&["BATT_AMP_PERVOLT", "BATT_AMP_PERVLT", "BATT_VOLT_MULT"]);
        let mut runner = Runner::default();
        runner.push(drain(&mut battery));
        runner.advance(&middle);
        assert_eq!(
            taken(&middle),
            vec![
                ("BATT_AMP_PERVOLT".to_owned(), 24.0),
                ("BATT_VOLT_MULT".to_owned(), f64::from(12.02_f32)),
            ]
        );
    }

    /// Monitor 0: the pins cleared, and the Calibration box, the sensor and the board off - so the
    /// sensor handler's writes find their boxes disabled and make none.
    #[test]
    fn a_disabled_monitor_clears_the_pins_and_writes_nothing_else() {
        let (mut battery, _view) = open(&[
            ("BATT_MONITOR", 0.0),
            ("BATT_VOLT_PIN", 13.0),
            ("BATT_CURR_PIN", 12.0),
        ]);
        let controls = battery.controls().unwrap();
        assert!(!controls.calibration_enabled);
        assert!(!controls.combo_enabled(ComboId::Sensor));
        assert!(!controls.combo_enabled(ComboId::Hardware));
        assert!(controls.combo_enabled(ComboId::Monitor));
        for field in [
            Field::Measured,
            Field::Divider,
            Field::MeasuredCurrent,
            Field::AmpsPerVolt,
        ] {
            assert!(!controls.field_enabled(field), "{field:?}");
        }
        assert!(controls.field_enabled(Field::Capacity), "outside the box");
        let jobs = drain(&mut battery);
        assert_eq!(
            steps(&jobs),
            vec![
                (vec!["BATT_VOLT_PIN"], -1.0),
                (vec!["BATT_CURR_PIN"], -1.0),
                (vec!["BATT_MONITOR"], 0.0),
            ]
        );
    }

    /// A monitor the handler has no branch for - an SMBus battery, say - enables the sensor combo
    /// and leaves the rest as `Activate` left them: no pins, no board.
    #[test]
    fn an_unnamed_monitor_leaves_the_board_combo_as_activate_left_it() {
        let (mut battery, _view) = open(&[("BATT_MONITOR", 7.0), ("BATT_CAPACITY", 5000.0)]);
        let controls = battery.controls().unwrap();
        assert_eq!(controls.monitor_text(), "SMBus-Generic");
        assert!(!controls.combo_enabled(ComboId::Hardware));
        assert!(controls.combo_enabled(ComboId::Sensor));
        assert_eq!(controls.capacity.value(), "5000");
        let jobs = drain(&mut battery);
        assert_eq!(steps(&jobs), vec![(vec!["BATT_MONITOR"], 7.0)]);
        // The two calibration boxes are empty, so `float.Parse` throws for each, and the catch
        // is silent for a monitor that is not analog.
        assert_eq!(
            jobs.get(1..),
            Some(
                &[
                    Job::Throw(OnThrow::ShowIfAnalog(AMPS_FAILED)),
                    Job::Throw(OnThrow::ShowIfAnalog(DIVIDER_FAILED)),
                ][..]
            )
        );
    }

    /// Without a link, or without `BATT_MONITOR`, the page is disabled and keeps the designer's.
    #[test]
    fn with_no_monitor_parameter_the_page_is_disabled() {
        let (mut battery, view) = open(&[("BATT_CAPACITY", 5000.0)]);
        let controls = battery.controls().unwrap();
        assert!(!controls.enabled);
        assert_eq!(controls.capacity.value(), "2200", "the designer's text");
        assert!(drain(&mut battery).is_empty());
        assert!(battery.timer.is_none(), "the timer is not started");
        battery.leave(Field::Capacity, &view.parameters);
        battery.choose(ComboId::Monitor, 4, &view.parameters);
        assert!(
            drain(&mut battery).is_empty(),
            "a disabled page does nothing"
        );

        let mut view = view_with(&SITL);
        view.connected = false;
        let mut battery = BatteryMonitor::default();
        battery.open(&view, bundled, &Persisted::at(None));
        assert!(!battery.controls().unwrap().enabled);
    }

    /// Pin 10 without `BATT_CURR_PIN` throws in `Activate`: `startup` stays set, so nothing the
    /// page does afterwards writes, and the timer never starts.
    #[test]
    fn activation_stops_where_the_csharp_throws() {
        let (mut battery, view) = open(&[
            ("BATT_MONITOR", 4.0),
            ("BATT_CAPACITY", 3300.0),
            ("BATT_VOLT_PIN", 10.0),
        ]);
        assert!(drain(&mut battery).is_empty());
        assert!(battery.timer.is_none());
        assert_eq!(battery.controls().unwrap().hardware, None);
        battery.leave(Field::Capacity, &view.parameters);
        battery.choose(ComboId::Hardware, 4, &view.parameters);
        assert!(drain(&mut battery).is_empty());
    }

    /// A sensor chosen: its preset into both boxes, both written, the boxes then disabled; Other
    /// writes what the boxes hold and leaves them editable.
    #[test]
    fn choosing_a_sensor_writes_its_preset() {
        let (mut battery, view) = open(&SITL);
        drain(&mut battery);
        battery.choose(ComboId::Sensor, 4, &view.parameters);
        let controls = battery.controls().unwrap();
        assert_eq!(controls.divider.value(), "10.10101");
        assert_eq!(controls.amps_per_volt.value(), "18.0018");
        for field in [Field::Measured, Field::Divider, Field::AmpsPerVolt] {
            assert!(!controls.field_enabled(field), "{field:?} after a preset");
        }
        assert_eq!(
            steps(&drain(&mut battery)),
            vec![
                (AMPS_NAMES.to_vec(), f64::from(18.0018_f32)),
                (DIVIDER_NAMES.to_vec(), f64::from(10.101_01_f32)),
            ]
        );
        battery.choose(ComboId::Sensor, 4, &view.parameters);
        assert!(
            drain(&mut battery).is_empty(),
            "the same row raises nothing"
        );

        battery.choose(ComboId::Sensor, 0, &view.parameters);
        let controls = battery.controls().unwrap();
        assert!(controls.field_enabled(Field::Divider));
        assert_eq!(controls.divider.value(), "10.10101", "Other keeps the text");
        assert_eq!(drain(&mut battery).len(), 2);
    }

    /// The monitor combo: the page's handler, then `MavlinkComboBox`'s own write; off 0, a
    /// refresh after.
    #[test]
    fn choosing_a_monitor_writes_it_twice_and_refreshes_off_zero() {
        let (mut battery, view) = open(&SITL);
        drain(&mut battery);
        battery.choose(ComboId::Monitor, 0, &view.parameters);
        let jobs = drain(&mut battery);
        assert_eq!(
            steps(&jobs),
            vec![
                (vec!["BATT_VOLT_PIN"], -1.0),
                (vec!["BATT_CURR_PIN"], -1.0),
                (vec!["BATT_MONITOR"], 0.0),
                (vec!["BATT_MONITOR"], 0.0),
            ]
        );
        assert!(!battery.controls().unwrap().calibration_enabled);
        assert!(
            jobs.iter()
                .all(|job| !matches!(job, Job::Writes { refresh: true, .. }))
        );
        battery.choose(ComboId::Monitor, 0, &view.parameters);
        assert!(drain(&mut battery).is_empty());
        battery.choose(ComboId::Monitor, 99, &view.parameters);
        assert!(drain(&mut battery).is_empty(), "not an option");

        // From 0 on the vehicle to 3: a refresh, and the voltage-only boxes.
        let (mut battery, view) = open(&[("BATT_MONITOR", 0.0)]);
        drain(&mut battery);
        battery.choose(ComboId::Monitor, 3, &view.parameters);
        let jobs = drain(&mut battery);
        assert!(matches!(
            jobs.first(),
            Some(Job::Writes { refresh: true, .. })
        ));
        let controls = battery.controls().unwrap();
        assert!(controls.calibration_enabled);
        assert!(!controls.combo_enabled(ComboId::Sensor));
        assert!(controls.combo_enabled(ComboId::Hardware));
        assert!(!controls.field_enabled(Field::AmpsPerVolt));
        assert!(controls.field_enabled(Field::Divider));
        assert!(controls.field_enabled(Field::Measured));
    }

    #[test]
    fn choosing_a_board_writes_its_pins() {
        let (mut battery, view) = open(&SITL);
        drain(&mut battery);
        battery.choose(ComboId::Hardware, 4, &view.parameters);
        assert_eq!(
            steps(&drain(&mut battery)),
            vec![(vec!["BATT_VOLT_PIN"], 2.0), (vec!["BATT_CURR_PIN"], 3.0)]
        );
        battery.choose(ComboId::Hardware, 4, &view.parameters);
        assert!(drain(&mut battery).is_empty());
        battery.choose(ComboId::Hardware, 11, &view.parameters);
        assert!(drain(&mut battery).is_empty(), "no eleventh board");
    }

    /// Enter in the measured-voltage box: the divider worked out, shown and written.
    #[test]
    fn a_measured_voltage_sets_the_divider() {
        let (mut battery, view) = open(&SITL);
        drain(&mut battery);
        battery.type_into(Field::Measured, "12.5");
        battery.controls.as_mut().unwrap().voltage = "12.6".to_owned();
        battery.enter(Field::Measured, &view.parameters);
        assert_eq!(battery.controls().unwrap().divider.value(), "10.01984");
        assert_eq!(
            steps(&drain(&mut battery)),
            vec![(DIVIDER_NAMES.to_vec(), f64::from(10.019_84_f32))]
        );

        // Text that is not a number: the C#'s message, and no write.
        battery.type_into(Field::Measured, "twelve");
        battery.enter(Field::Measured, &view.parameters);
        assert_eq!(drain(&mut battery), vec![Job::Show(INVALID_NUMBER)]);
        // A reading of zero: nothing at all.
        battery.type_into(Field::Measured, "12.5");
        battery.controls.as_mut().unwrap().voltage = "0".to_owned();
        battery.enter(Field::Measured, &view.parameters);
        assert!(drain(&mut battery).is_empty());
    }

    /// Leaving the measured-current box: the amps per volt worked out from the vehicle's current.
    /// Before the timer has shown a current there is nothing to divide by, and the C# says so.
    #[test]
    fn a_measured_current_sets_the_amps_per_volt() {
        let (mut battery, view) = open(&SITL);
        drain(&mut battery);
        battery.type_into(Field::MeasuredCurrent, "2.5");
        battery.leave(Field::MeasuredCurrent, &view.parameters);
        assert_eq!(drain(&mut battery), vec![Job::Show(INVALID_NUMBER)]);
        battery.controls.as_mut().unwrap().current = "1.23000001907349".to_owned();
        battery.leave(Field::MeasuredCurrent, &view.parameters);
        assert_eq!(
            battery.controls().unwrap().amps_per_volt.value(),
            "34.55285"
        );
        assert_eq!(
            steps(&drain(&mut battery)),
            vec![(AMPS_NAMES.to_vec(), f64::from(34.552_85_f32))]
        );
    }

    /// The `Validating` handlers are wired to nothing: leaving a box whose text does not parse
    /// runs `Validated` as Enter does, whose `float.Parse` throws, and the `catch` speaks - in a
    /// box, for what was typed - only for an analog monitor.
    #[test]
    fn text_that_does_not_parse_is_reported_on_leaving_and_on_enter() {
        let Some(designer) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigBatteryMonitoring.Designer.cs",
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        assert!(!designer.contains("Validating"), "no Validating is wired");
        let (mut battery, view) = open(&SITL);
        drain(&mut battery);
        battery.type_into(Field::Divider, "ten");
        battery.leave(Field::Divider, &view.parameters);
        let left = drain(&mut battery);
        battery.enter(Field::Divider, &view.parameters);
        let jobs = drain(&mut battery);
        assert_eq!(left, jobs, "leaving is Enter");
        assert_eq!(
            jobs,
            vec![Job::Throw(OnThrow::ShowIfAnalog(DIVIDER_FAILED))]
        );
        // The measured voltage, left not parsing: "Invalid number entered".
        battery.type_into(Field::Measured, "twelve");
        battery.leave(Field::Measured, &view.parameters);
        assert_eq!(drain(&mut battery), vec![Job::Show(INVALID_NUMBER)]);

        let mut runner = Runner::<usize>::default();
        runner.push(jobs.clone());
        let events = runner.advance(&Vehicle::holding(&[]));
        battery.absorb(events, None, &view);
        assert!(battery.message().is_none(), "the status line, not a box");
        assert_eq!(
            battery.take_status().as_deref(),
            Some("Set BATT_VOLT_MULT Failed")
        );

        // With a monitor that is not analog, the same failure is silent.
        let quiet = view_with(&[("BATT_MONITOR", 7.0)]);
        let mut runner = Runner::<usize>::default();
        runner.push(jobs);
        let events = runner.advance(&Vehicle::holding(&[]));
        battery.absorb(events, None, &quiet);
        assert!(battery.message().is_none());
    }

    /// The capacity box: written when left; with no `BATT_CAPACITY`, the feature message.
    #[test]
    fn the_capacity_is_written_when_its_box_is_left() {
        let (mut battery, view) = open(&SITL);
        drain(&mut battery);
        battery.type_into(Field::Capacity, "3400");
        battery.leave(Field::Capacity, &view.parameters);
        assert_eq!(
            steps(&drain(&mut battery)),
            vec![(vec!["BATT_CAPACITY"], 3400.0)]
        );
        // No Validating on this box, so text that does not parse reaches the catch.
        battery.type_into(Field::Capacity, "lots");
        battery.leave(Field::Capacity, &view.parameters);
        assert_eq!(
            drain(&mut battery),
            vec![Job::Throw(OnThrow::Show(CAPACITY_FAILED))]
        );

        let (mut battery, view) = open(&[("BATT_MONITOR", 4.0)]);
        drain(&mut battery);
        battery.leave(Field::Capacity, &view.parameters);
        assert_eq!(drain(&mut battery), vec![Job::Show(FEATURE_NOT_ENABLED)]);
    }

    /// The runner: a name the vehicle lacks falls through to the next; a timeout ends its job
    /// with the job's message and the next job still runs; `MavlinkComboBox`'s false is a message.
    #[test]
    fn the_runner_goes_as_the_csharps_blocking_calls_go() {
        let mut vehicle = Vehicle::holding(&["BATT_AMP_PERVLT", "BATT_CAPACITY"]);
        vehicle.timeout = Some("BATT_VOLT_PIN");
        let mut runner = Runner::default();
        runner.push([
            Job::writes(vec![Step::any(&AMPS_NAMES, 17.0)], OnThrow::Show("amps")),
            Job::writes(
                vec![
                    Step::one("BATT_VOLT_PIN", 2.0),
                    Step::one("BATT_CURR_PIN", 3.0),
                ],
                OnThrow::Show(PINS_FAILED),
            ),
            Job::Writes {
                steps: vec![Step::one("BATT_MONITOR", 4.0)].into(),
                on_throw: OnThrow::Show(COMBO_FAILED),
                on_false: Some(COMBO_FAILED),
                refresh: true,
            },
            Job::writes(
                vec![Step::one("BATT_CAPACITY", 3400.0)],
                OnThrow::Show(CAPACITY_FAILED),
            ),
        ]);
        let events = runner.advance(&vehicle);
        let names: Vec<String> = vehicle
            .written()
            .into_iter()
            .map(|(name, _)| name)
            .collect();
        assert_eq!(
            names,
            [
                "AMP_PER_VOLT",
                "BATT_AMP_PERVOLT",
                "BATT_AMP_PERVLT",
                "BATT_VOLT_PIN",
                "BATT_MONITOR",
                "BATT_CAPACITY"
            ],
            "BATT_CURR_PIN never sent after the timeout"
        );
        assert!(events.contains(&Event::Threw {
            what: "BATT_VOLT_PIN 2".to_owned(),
            on_throw: OnThrow::Show(PINS_FAILED),
        }));
        assert!(events.contains(&Event::Refused(COMBO_FAILED)));
        assert!(
            events.contains(&Event::Refresh),
            "after the writes, even refused"
        );
        assert!(events.contains(&Event::Written {
            name: "BATT_CAPACITY",
            value: 3400.0,
            outcome: RequestOutcome::Accepted { value: None },
        }));
        assert_eq!(runner.pending(), 0);

        // While the vehicle has not answered, nothing more is sent.
        let mut silent = Vehicle::holding(&["BATT_CAPACITY"]);
        silent.silent = true;
        let mut runner = Runner::default();
        runner.push([
            Job::writes(vec![Step::one("BATT_CAPACITY", 1.0)], OnThrow::Show("")),
            Job::writes(vec![Step::one("BATT_CAPACITY", 2.0)], OnThrow::Show("")),
        ]);
        assert!(runner.advance(&silent).is_empty());
        assert!(runner.advance(&silent).is_empty());
        assert_eq!(silent.written().len(), 1);
        assert_eq!(runner.pending(), 2);
    }

    /// The page's own loop: a box the focus leaves is validated and its write goes to the link;
    /// with no vehicle to take it the C#'s `catch` speaks - on the status line, the owner's
    /// ruling of 2026-09-25, and not in a box.
    #[test]
    fn leaving_a_box_in_the_frame_loop_writes_it() {
        let telemetry = Telemetry::idle();
        let (mut battery, view) = open(&SITL);
        drain(&mut battery);
        battery.type_into(Field::Capacity, "3400");
        let mut focused = [false; 5];
        focused[Field::Capacity.index()] = true;
        battery.tick(&telemetry, &view, focused, true, &Persisted::at(None));
        assert!(battery.take_status().is_none(), "still in the box");
        battery.tick(&telemetry, &view, [false; 5], true, &Persisted::at(None));
        assert!(battery.message().is_none());
        assert_eq!(battery.take_status().as_deref(), Some(CAPACITY_FAILED));
        assert_eq!(
            battery.last_write.as_deref(),
            Some("BATT_CAPACITY 3400 failed")
        );
    }

    /// `MavlinkComboBox`'s false - a vehicle without the name - is a link failure too.
    #[test]
    fn a_refused_monitor_write_is_a_status_line() {
        let (mut battery, view) = open(&SITL);
        drain(&mut battery);
        battery.absorb(vec![Event::Refused(COMBO_FAILED)], None, &view);
        assert!(battery.message().is_none());
        assert_eq!(battery.take_status().as_deref(), Some(COMBO_FAILED));
        // A handler's own box is the status line too: an error the window can show as state
        // never gets a box (the owner's rule, 2026-09-25).
        battery.absorb(vec![Event::Status(FEATURE_NOT_ENABLED)], None, &view);
        assert!(battery.message().is_none());
        assert_eq!(
            battery.take_status().as_deref(),
            Some(FEATURE_NOT_ENABLED)
        );
    }

    /// The Sensor and HW Ver boxes are `DropDown`s: typing changes their text and clears the
    /// selection, runs no handler and writes nothing; a row chosen afterwards - even the one that
    /// was selected - is a change, puts its text back and runs the handler.
    #[test]
    fn typing_into_the_sensor_and_board_boxes_writes_nothing_until_a_row_is_chosen() {
        let (mut battery, view) = open(&SITL);
        drain(&mut battery);
        let typed = |key: &str| KeyDownEvent {
            keystroke: gpui::Keystroke {
                modifiers: gpui::Modifiers::default(),
                key: key.to_owned(),
                key_char: (key.chars().count() == 1).then(|| key.to_owned()),
            },
            is_held: false,
            prefer_character_input: false,
        };
        let controls = battery.controls().expect("open");
        assert_eq!(controls.combo_text(ComboId::Sensor), "0: Other");
        assert_eq!(
            controls.combo_text(ComboId::Hardware),
            "2: APM2.5+/ZealotF427 - 3DR Power Module"
        );
        // The monitor combo is a `DropDownList`: no typing.
        assert!(!battery.type_combo(ComboId::Monitor, &typed("x")));
        // A backspace and a letter: the text edited, no row selected, nothing queued.
        assert!(battery.type_combo(ComboId::Sensor, &typed("backspace")));
        assert!(battery.type_combo(ComboId::Sensor, &typed("x")));
        let controls = battery.controls().expect("open");
        assert_eq!(controls.combo_text(ComboId::Sensor), "0: Othex");
        assert_eq!(controls.sensor, None);
        assert!(battery.type_combo(ComboId::Hardware, &typed("7")));
        assert_eq!(battery.controls().and_then(|c| c.hardware), None);
        assert!(drain(&mut battery).is_empty(), "no handler ran");
        // Choosing the row that was selected is a change now: its text back, its pins written.
        battery.toggle_dropdown(ComboId::Hardware);
        battery.choose(ComboId::Hardware, 2, &view.parameters);
        let controls = battery.controls().expect("open");
        assert_eq!(
            controls.combo_text(ComboId::Hardware),
            "2: APM2.5+/ZealotF427 - 3DR Power Module"
        );
        assert_eq!(controls.hardware_typed, None);
        assert_eq!(
            steps(&drain(&mut battery)),
            vec![(vec!["BATT_VOLT_PIN"], 13.0), (vec!["BATT_CURR_PIN"], 12.0)]
        );
        battery.choose(ComboId::Sensor, 0, &view.parameters);
        assert_eq!(
            battery.controls().map(|c| c.combo_text(ComboId::Sensor)),
            Some("0: Other")
        );
        assert_eq!(
            steps(&drain(&mut battery)),
            vec![
                (AMPS_NAMES.to_vec(), 17.0),
                (DIVIDER_NAMES.to_vec(), f64::from(10.1_f32)),
            ]
        );
        // A disabled box takes nothing: monitor 0 disables the sensor combo.
        let (mut off, _) = open(&[("BATT_MONITOR", 0.0), ("BATT_VOLT_PIN", 13.0)]);
        assert!(!off.type_combo(ComboId::Sensor, &typed("x")));
    }

    /// Each OK keeps the answer as `InputBox` keeps it, under its caption and question.
    #[test]
    fn each_answer_is_kept_as_input_box_keeps_it() {
        let (mut battery, _) = open(&SITL);
        let mut settings = Persisted::at(None);
        battery.click_speech(&mut settings);
        for _ in SPEECH_PROMPTS {
            battery.answer(&mut settings);
        }
        assert!(battery.prompt().is_none());
        assert_eq!(
            settings.get("InputBoxNotificationWhatdoyouwantittosay"),
            Some("WARNING%2C+Battery+at+%7Bbatv%7D+Volt%2C+%7Bbatp%7D+percent")
        );
        assert_eq!(
            settings.get("InputBoxBatteryLevelWhatVoltagedoyouwanttowarnat"),
            Some("9.6")
        );
        assert_eq!(
            settings.get("InputBoxBatteryLevelWhatpercentagedoyouwanttowarnat"),
            Some("20")
        );
        for (title, question, _, _) in SPEECH_PROMPTS {
            let key = crate::config::optional::answers_key(title, question);
            assert!(crate::settings::PUBLISHED.contains(&key.as_str()), "{key}");
        }
    }

    /// The lists drop down through the shared drop-down: the rows, the selection, and the wheel.
    #[test]
    fn the_lists_are_the_shared_drop_down() {
        let (mut battery, _) = open(&SITL);
        assert!(battery.list().is_none());
        battery.toggle_dropdown(ComboId::Hardware);
        let list = battery.list().expect("down");
        assert_eq!(list.options.len(), HW_VERSIONS.len());
        assert_eq!(list.selected, Some(2));
        assert_eq!(list.top_index, 0);
        battery.scroll_list(1);
        assert_eq!(
            battery.list().map(|list| list.top_index),
            Some(0),
            "all rows show"
        );
        battery.close_list();
        assert!(battery.list().is_none());
    }

    /// Leaving the setup screen closes the page, as leaving Initial Setup deactivates it; a box
    /// left as it closes writes nothing, because `Deactivate` has set `startup`.
    #[test]
    fn leaving_the_setup_screen_closes_the_page_without_writing() {
        let telemetry = Telemetry::idle();
        let (mut battery, view) = open(&SITL);
        drain(&mut battery);
        let mut focused = [false; 5];
        focused[Field::Capacity.index()] = true;
        battery.tick(&telemetry, &view, focused, true, &Persisted::at(None));
        battery.tick(&telemetry, &view, [false; 5], false, &Persisted::at(None));
        assert!(!battery.is_open());
        assert!(battery.message().is_none());
        assert_eq!(battery.pending(), 0);
    }

    /// The timer puts the vehicle's voltage and current in their boxes once a second, as the
    /// doubles' text.
    #[test]
    fn the_timer_shows_the_vehicles_readings() {
        let telemetry = Telemetry::idle();
        let (mut battery, mut view) = open(&SITL);
        drain(&mut battery);
        let mut state = mp_vehicle::VehicleState::default();
        state.battery.voltage = 11.1;
        state.battery.current = 2.5;
        view.state = Some(Arc::new(state));
        battery.tick(&telemetry, &view, [false; 5], true, &Persisted::at(None));
        assert_eq!(battery.ticks, 0, "not a second yet");
        battery.timer = Instant::now().checked_sub(TIMER_INTERVAL);
        battery.tick(&telemetry, &view, [false; 5], true, &Persisted::at(None));
        assert_eq!(battery.ticks, 1);
        let controls = battery.controls().unwrap();
        assert_eq!(controls.voltage, invariant_double(f64::from(11.1_f32)));
        assert_eq!(controls.voltage, "11.1000003814697");
        assert_eq!(controls.current, "2.5");
        assert_eq!(
            controls.measured.value(),
            invariant_double(f64::from(12.6_f32)),
            "the measured box keeps what it opened with"
        );
    }

    /// `GetBoolean` over `Settings.Instance`: "True" in any case, blanks and NULs trimmed; absent or
    /// anything else is false.
    #[test]
    fn get_boolean_reads_the_dictionary_as_bool_try_parse_does() {
        let mut settings = Persisted::at(None);
        settings.set("speechbatteryenabled", "True");
        settings.set("speechenable", " true ");
        assert!(get_boolean(&settings, "speechbatteryenabled"));
        assert!(get_boolean(&settings, "speechenable"));
        assert!(!get_boolean(&settings, "speechbattery"), "absent");
        settings.set("speechenable", "yes");
        assert!(!get_boolean(&settings, "speechenable"), "not a boolean");
    }

    /// The check box reads and writes `Settings.Instance`: `Activate` ticks it from the
    /// dictionary; a click writes "True"/"False" and `speechenable`, and ticked asks the three
    /// questions, each offering the dictionary's value or its default and putting the answer in
    /// the dictionary; Cancel stops them.
    #[test]
    fn the_low_battery_alert_reads_and_writes_settings_instance() {
        let mut settings = Persisted::at(None);
        settings.set("speechbatteryenabled", "True");
        settings.set("speechenable", "True");
        settings.set("speechbatteryvolt", "10.5");
        let view = view_with(&SITL);
        let mut battery = BatteryMonitor::default();
        battery.open(&view, bundled, &settings);
        assert!(battery.controls().unwrap().speech, "read at Activate");
        battery.click_speech(&mut settings);
        assert!(!battery.controls().unwrap().speech);
        assert_eq!(settings.get("speechbatteryenabled"), Some("False"));
        assert!(battery.prompt().is_none(), "unchecking asks nothing");

        battery.click_speech(&mut settings);
        assert_eq!(settings.get("speechbatteryenabled"), Some("True"));
        assert_eq!(settings.get("speechenable"), Some("True"));
        let prompt = battery.prompt().unwrap();
        assert_eq!(
            prompt.text(),
            ("Notification", "What do you want it to say?")
        );
        assert_eq!(
            prompt.field.value(),
            "WARNING, Battery at {batv} Volt, {batp} percent"
        );
        battery.answer(&mut settings);
        assert_eq!(
            settings.get("speechbattery"),
            Some("WARNING, Battery at {batv} Volt, {batp} percent"),
            "in the dictionary at once"
        );
        let prompt = battery.prompt().unwrap();
        assert_eq!(prompt.text().1, "What Voltage do you want to warn at?");
        assert_eq!(
            prompt.field.value(),
            "10.5",
            "the setting, over the default"
        );
        battery.cancel();
        assert!(battery.prompt().is_none());
        assert_eq!(settings.get("speechbatteryvolt"), Some("10.5"));
        assert_eq!(settings.get("speechbatterypercent"), None);

        // Another page's write - Battery Monitor 2 and the Planner page share the dictionary - is
        // what the next `Activate` reads.
        settings.set("speechbatteryenabled", "False");
        battery.close();
        battery.open(&view, bundled, &settings);
        assert!(!battery.controls().unwrap().speech);
    }

    /// A change made on the page is in `Settings.Instance` at once, in `config.xml` after the next
    /// `SaveConfig` and not before, and read back by the page after a restart.
    #[test]
    fn a_speech_change_is_saved_with_config_xml_and_comes_back() {
        let dir =
            mp_os::temp_dir().join(format!("mp-gui-battery-speech-{}", mp_os::process_id()));
        let _ = mp_os::fs::remove_dir_all(&dir);
        let path = dir.join("MissionPlannerRust").join("config.xml");
        let mut settings = Persisted::at(Some(path.clone()));
        let view = view_with(&SITL);
        let mut battery = BatteryMonitor::default();
        battery.open(&view, bundled, &settings);
        assert!(!battery.controls().unwrap().speech);

        battery.click_speech(&mut settings);
        for answer in ["LOW {batv}", "11.1", "30"] {
            battery.prompt_mut().unwrap().field.set(answer);
            battery.answer(&mut settings);
        }
        assert!(battery.prompt().is_none(), "three questions");
        assert_eq!(settings.get("speechbattery"), Some("LOW {batv}"));
        assert!(!path.os_exists(), "nothing on disk before a save");

        settings
            .save_config(crate::settings::SaveEvent::FlightData)
            .expect("the save");
        let on_disk = mp_settings::Config::load(&path).expect("the file reads back");
        for (key, value) in [
            ("speechbatteryenabled", "True"),
            ("speechenable", "True"),
            ("speechbattery", "LOW {batv}"),
            ("speechbatteryvolt", "11.1"),
            ("speechbatterypercent", "30"),
        ] {
            assert_eq!(on_disk.get(key), Some(value), "{key}");
        }

        // A restart: the dictionary read from the file, the page ticked from it, and the saved
        // answer offered when the box is ticked again.
        let mut restarted = Persisted::at(Some(path));
        let mut again = BatteryMonitor::default();
        again.open(&view, bundled, &restarted);
        assert!(again.controls().unwrap().speech);
        again.click_speech(&mut restarted);
        again.click_speech(&mut restarted);
        assert_eq!(
            again.prompt().map(|prompt| prompt.field.value().to_owned()),
            Some("LOW {batv}".to_owned())
        );
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    /// Every fact `tests/gui/config-battery.gui` asserts on is one this page records or one of
    /// the dictionary's published keys; the script keeps a config.xml of its own, restarts to
    /// read it back, and the alert's box it clicks is one this page draws.
    #[test]
    fn the_gui_script_names_facts_this_page_and_the_dictionary_have() {
        let script = include_str!("../../../../tests/gui/config-battery.gui");
        let source = include_str!("battery_monitor.rs");
        let mut speech = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.battery.") => {
                    // A box's `.enabled` is recorded per box, by its key.
                    let per_box = key
                        .strip_prefix("config.battery.")
                        .and_then(|rest| rest.strip_suffix(".enabled"))
                        .is_some_and(|name| {
                            Field::ALL.iter().any(|field| field_key(*field) == name)
                        });
                    assert!(per_box || source.contains(&format!("\"{key}\"")), "{key}");
                }
                (Some("expect"), Some(key))
                    if key.starts_with("config.speech") || key.starts_with("config.InputBox") =>
                {
                    let name = key.trim_start_matches("config.");
                    assert!(crate::settings::PUBLISHED.contains(&name), "{key}");
                    speech += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("battery-") => {
                    // A `DropDown` combo's arrow is `<id>-button`, a list's row `<id>-<value>`.
                    let base = id.strip_suffix("-button").unwrap_or_else(|| {
                        id.rsplit_once('-')
                            .filter(|(_, row)| row.chars().all(|c| c.is_ascii_digit()))
                            .map_or(id, |(base, _)| base)
                    });
                    assert!(source.contains(&format!("\"{base}\"")), "{id}");
                }
                _ => {}
            }
        }
        assert!(speech >= 10, "{speech} speech facts");
        assert!(
            script
                .lines()
                .any(|line| line == "env MP_CONFIG_XML $WORK/config.xml")
        );
        assert!(script.lines().any(|line| line == "restart"));
        assert!(script.contains("click battery-speech"));
    }

    /// The combos drop down only when they are enabled, and one at a time.
    #[test]
    fn a_disabled_combo_does_not_drop_down() {
        let (mut battery, _view) = open(&[("BATT_MONITOR", 0.0)]);
        battery.toggle_dropdown(ComboId::Sensor);
        assert_eq!(battery.dropdown, None);
        battery.toggle_dropdown(ComboId::Monitor);
        assert_eq!(battery.dropdown, Some(ComboId::Monitor));
        battery.toggle_dropdown(ComboId::Monitor);
        assert_eq!(battery.dropdown, None);
    }
}
