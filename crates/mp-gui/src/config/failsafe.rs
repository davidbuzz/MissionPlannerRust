//! The FailSafe page of Initial Setup: `GCSViews/ConfigurationView/ConfigFailSafe.cs`.
//!
//! What it shows: the first eight radio inputs and servo outputs as bars, the flight mode, armed
//! state and GPS fix in large type, and the vehicle's failsafe parameters in three boxes - Battery,
//! Radio, GCS - each control bound to one parameter and writing it as soon as it is changed. There
//! is no save button in the C# and there is none here: a `MavlinkComboBox` or `MavlinkCheckBox`
//! writes from its change handler, a `MavlinkNumericUpDown` 300 ms after its last change.
//!
//! Which parameter each control is bound to is decided when the page opens (`Activate`), by
//! asking which names the vehicle lists: `BATT_FS_LOW_ACT` before `FS_BATT_ENABLE`, `LOW_VOLT`
//! before `FS_BATT_VOLTAGE` before `BATT_LOW_VOLT`, `FS_BATT_MAH` before `BATT_LOW_MAH`. A control
//! whose parameter the vehicle does not have stays hidden, which is how one page serves a copter
//! (`FS_THR_ENABLE`, `FS_GCS_ENABLE`) and a plane (`THR_FAILSAFE`, `FS_SHORT_ACTN`, ...) at once.
//!
//! The geometry is the `.resx`'s: every control at its `Location` and `Size` inside a 688 x 482
//! page, in its group box. The colours are this application's.
//!
//! What is not ported, and why:
//!
//! * typing a number: gpui has no numeric up-down, so the up and down arrows are the parameter
//!   editor's `-` and `+` steps, and the value cannot be typed. `MavlinkNumericUpDown`'s
//!   out-of-range prompt only fires on a typed value, so it goes with it;
//! * `mavlinkCheckBoxfs_gps_enable`, commented out in the C# "at randys request"
//!   (`ConfigFailSafe.cs:75-76`), and the `FS_SHORT_TIMEOUT`/`FS_LONG_TIMEOUT` a plane has, which
//!   the page never binds.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use gpui::{AnyElement, Context, Div, SharedString, Window, div, prelude::*, px, rgb};
use mp_link::requests::RequestOutcome;
use mp_params::ParamMeta;
use mp_vehicle::{VehicleFamily, VehicleState};

use crate::MissionPlanner;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{action, panel, theme};

/// Where parameter documentation comes from: [`crate::metadata::lookup`] on screen, the bundled
/// table in the tests.
pub type Lookup = fn(&str) -> Option<&'static ParamMeta>;

/// The message `Activate` shows every time the page opens.
/// `// C#: GCSViews/ConfigurationView/ConfigFailSafe.cs:91-92`
pub const PROPS_WARNING: &str = "Ensure your props are not on the Plane/Quad";

/// That message's title.
const PROPS_TITLE: &str = "FailSafe";

/// `Strings.ERROR`, the title of a failed write's message.
/// `// C#: ExtLibs/Strings/Strings.resx:130-132`
const ERROR_TITLE: &str = "Error";

/// How long a number waits after its last change before it is written: `MavlinkNumericUpDown`
/// restarts a 300 ms timer on every change and writes when it fires.
/// `// C#: Controls/MavlinkNumericUpDown.cs:154-180`
const WRITE_DELAY: Duration = Duration::from_millis(300);

/// `lbl_currentmode`'s text until the vehicle's mode first reaches it.
/// `// C#: GCSViews/ConfigurationView/ConfigFailSafe.resx lbl_currentmode.Text`
const DESIGNER_MODE: &str = "Manual";

/// The bars' scale: every one is `Minimum = 1000`, `Maximum = 2000`.
/// `// C#: GCSViews/ConfigurationView/ConfigFailSafe.Designer.cs:328-330`
const BAR_MINIMUM: i64 = 1000;
/// The top of that scale.
const BAR_MAXIMUM: i64 = 2000;

/// The rows the eight bars of each column sit on, from the `.resx`'s `Location`s.
/// `// C#: GCSViews/ConfigurationView/ConfigFailSafe.resx horizontalProgressBar1-16.Location`
const BAR_ROWS: [f32; 8] = [19.0, 70.0, 121.0, 172.0, 222.0, 273.0, 324.0, 375.0];

/// A write the page asks for: `MainV2.comPort.setParam(name, value)`, and the message the C#
/// shows if it fails.
#[derive(Debug, Clone, PartialEq)]
pub struct Write {
    /// The parameter.
    pub param: &'static str,
    /// The value, as the C# computes it.
    pub value: f64,
    /// What the C# shows when the set fails.
    pub failure: String,
}

impl Write {
    /// A combo box's write. Its failure text is not `Strings.ErrorSetValueFailed` but its own,
    /// with an exclamation mark. `// C#: Controls/MavlinkComboBox.cs:180-183, 197`
    fn combo(param: &'static str, value: f64) -> Self {
        Self {
            param,
            value,
            failure: format!("Set {param} Failed!"),
        }
    }

    /// A check box's or a number's write, which fails with `Strings.ErrorSetValueFailed`,
    /// "Set {0} Failed". `// C#: Controls/MavlinkCheckBox.cs:118, 134; ExtLibs/Strings/Strings.resx:171-173`
    fn other(param: &'static str, value: f64) -> Self {
        Self {
            param,
            value,
            failure: format!("Set {param} Failed"),
        }
    }
}

/// A modal message box, `CustomMessageBox.Show(text, title)`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Message {
    /// The caption.
    pub title: &'static str,
    /// The text.
    pub text: String,
}

/// A `MavlinkComboBox` set up with an option list: `setup(List<KeyValuePair<int, string>>, ...)`.
/// `// C#: Controls/MavlinkComboBox.cs:73-99`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Combo {
    /// `ParamName`, which `setup` also makes the control's `Name`.
    pub param: &'static str,
    /// `DataSource`: the values the documentation lists and their text,
    /// `ParameterMetaDataRepository.GetParameterOptionsInt`.
    pub options: Vec<(i64, String)>,
    /// `SelectedValue`. `None` is `SelectedIndex` -1: the vehicle holds a value that is not one
    /// of the options, and the box is blank.
    pub selected: Option<i64>,
    /// `Visible` and `Enabled`, which `setup` sets together and only for a parameter the vehicle
    /// has. Both start false in the `.resx`.
    pub shown: bool,
}

impl Combo {
    /// Sets the box up from the vehicle's parameters.
    fn setup(param: &'static str, parameters: &[(String, f64)], lookup: Lookup) -> Self {
        let value = value_of(parameters, param);
        let options = options(param, lookup);
        // `this.SelectedValue = (int)paramlist[paramname].Value`: a cast, so truncated, and a
        // value the list does not hold selects nothing.
        #[allow(clippy::cast_possible_truncation)]
        let selected = value
            .map(|value| value as i64)
            .filter(|value| options.iter().any(|(key, _)| key == value));
        Self {
            param,
            options,
            selected,
            shown: value.is_some(),
        }
    }

    /// The selected option's text, blank when nothing is selected.
    #[must_use]
    pub fn text(&self) -> &str {
        self.selected
            .and_then(|selected| self.options.iter().find(|(key, _)| *key == selected))
            .map_or("", |(_, text)| text.as_str())
    }

    /// Chooses an option. A write only when the selection changed, because that is when WinForms
    /// raises `SelectedIndexChanged`; the value is `(float)(int)SelectedValue`.
    /// `// C#: Controls/MavlinkComboBox.cs:169-199`
    fn choose(&mut self, key: i64) -> Option<Write> {
        if !self.shown || self.selected == Some(key) {
            return None;
        }
        if !self.options.iter().any(|(option, _)| *option == key) {
            return None;
        }
        self.selected = Some(key);
        #[allow(clippy::cast_precision_loss)] // option values are small integers
        let value = f64::from(key as f32);
        Some(Write::combo(self.param, value))
    }
}

/// The options a parameter's documentation lists, trimmed as the C# trims them.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepository.cs:76-102`
#[must_use]
pub fn options(param: &str, lookup: Lookup) -> Vec<(i64, String)> {
    lookup(param).map_or_else(Vec::new, |meta| {
        meta.values
            .iter()
            .map(|(key, text)| (*key, text.trim().to_owned()))
            .collect()
    })
}

/// A check box's three states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CheckState {
    /// Off.
    Unchecked,
    /// On.
    Checked,
    /// The vehicle holds neither the on value nor the off value.
    Indeterminate,
}

impl CheckState {
    /// The word a fact carries.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::Unchecked => "unchecked",
            Self::Checked => "checked",
            Self::Indeterminate => "indeterminate",
        }
    }
}

/// A `MavlinkCheckBox`: `setup(OnValue, OffValue, paramname, paramlist)`.
/// `// C#: Controls/MavlinkCheckBox.cs:60-98`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Check {
    /// `ParamName`.
    pub param: &'static str,
    /// What the box shows.
    pub state: CheckState,
    /// `Visible` and `Enabled`: set for a parameter the vehicle has. The `.resx` hides every
    /// check box on this page until then.
    pub shown: bool,
    /// `OnValue`.
    on: f64,
    /// `OffValue`.
    off: f64,
}

impl Check {
    /// Sets the box up: checked for the on value, unchecked for the off value, indeterminate for
    /// anything else.
    fn setup(on: f64, off: f64, param: &'static str, parameters: &[(String, f64)]) -> Self {
        let value = value_of(parameters, param);
        // `paramlist[paramname].Value == OnValue`: an exact comparison, as the C# makes it, of
        // the same float reading on both sides.
        #[allow(clippy::float_cmp)]
        let state = match value {
            Some(value) if value == on => CheckState::Checked,
            Some(value) if value == off => CheckState::Unchecked,
            Some(_) => CheckState::Indeterminate,
            None => CheckState::Unchecked,
        };
        Self {
            param,
            state,
            shown: value.is_some(),
            on,
            off,
        }
    }

    /// A click. WinForms takes a two-state box from checked or indeterminate to unchecked and
    /// from unchecked to checked, and every one of those changes `Checked`, so every click writes:
    /// `OnValue` when it ends checked, `OffValue` otherwise.
    /// `// C#: Controls/MavlinkCheckBox.cs:106-143`
    fn click(&mut self) -> Option<Write> {
        if !self.shown {
            return None;
        }
        self.state = match self.state {
            CheckState::Unchecked => CheckState::Checked,
            CheckState::Checked | CheckState::Indeterminate => CheckState::Unchecked,
        };
        let value = if self.state == CheckState::Checked {
            self.on
        } else {
            self.off
        };
        Some(Write::other(self.param, value))
    }
}

/// A decimal as .NET's `(decimal)float` makes one: seven significant digits, trailing zeros
/// dropped. Returned as a mantissa and the number of places after the point, which is what
/// `decimal.GetBits(...)[3]`'s scale byte reads back.
/// `// C#: Controls/MavlinkNumericUpDown.cs:90, 101-103`
#[must_use]
pub fn decimal_of(value: f32) -> (i64, u32) {
    if value == 0.0 || !value.is_finite() {
        return (0, 0);
    }
    let text = format!("{:.6e}", value.abs());
    let Some((mantissa, exponent)) = text.split_once('e') else {
        return (0, 0);
    };
    let exponent: i64 = exponent.parse().unwrap_or(0);
    let digits: String = mantissa.chars().filter(char::is_ascii_digit).collect();
    let digits = digits.trim_end_matches('0');
    let mut mantissa: i64 = digits.parse().unwrap_or(0);
    // The number of places: digits after the first, less the power of ten.
    let places = i64::try_from(digits.len()).unwrap_or(0) - 1 - exponent;
    let scale = if places < 0 {
        mantissa =
            mantissa.saturating_mul(10_i64.saturating_pow(u32::try_from(-places).unwrap_or(0)));
        0
    } else {
        u32::try_from(places).unwrap_or(0)
    };
    if value < 0.0 {
        mantissa = -mantissa;
    }
    (mantissa, scale)
}

/// Re-expresses a mantissa at more decimal places.
fn rescale(mantissa: i64, from: u32, to: u32) -> i64 {
    mantissa.saturating_mul(10_i64.saturating_pow(to.saturating_sub(from)))
}

/// A `MavlinkNumericUpDown`, set up: bounds, step and value held as the C#'s `decimal`s, in units
/// of its last decimal place.
/// `// C#: Controls/MavlinkNumericUpDown.cs:43-122`
#[derive(Debug, Clone, PartialEq)]
pub struct Numeric {
    /// `ParamName`.
    pub param: &'static str,
    /// Whether the vehicle has the parameter, so `setup` bound the box to it.
    pub bound: bool,
    /// `Visible`.
    pub shown: bool,
    /// `Enabled`.
    pub enabled: bool,
    /// `Minimum`.
    pub minimum: f64,
    /// `Maximum`.
    pub maximum: f64,
    /// `DecimalPlaces`.
    pub decimals: u32,
    /// `Increment`, in units of the last decimal place.
    increment: i64,
    /// `Value`, in units of the last decimal place.
    value: i64,
    /// `_scale`: the parameter is the shown value times this.
    scale: f32,
    /// When the value last changed and has not yet been written: the running timer.
    changed: Option<Instant>,
}

/// How a number is set up, in the order `setup`'s arguments come.
struct NumericSetup {
    /// `Min`.
    minimum: f32,
    /// `Max`.
    maximum: f32,
    /// `Scale`.
    scale: f32,
    /// `Increment`.
    increment: f32,
    /// The designer's `DecimalPlaces`, which `setup` compares the documented increment against.
    designer_decimals: u32,
    /// The designer's `Visible`, which `setup` leaves alone for a parameter the vehicle lacks.
    designer_shown: bool,
}

impl Numeric {
    /// `setup(Min, Max, Scale, Increment, paramname, paramlist)`.
    fn setup(
        how: &NumericSetup,
        param: &'static str,
        parameters: &[(String, f64)],
        lookup: Lookup,
    ) -> Self {
        let meta = lookup(param);
        // The documented range wins over the one passed in. `// C#: :69-75`
        let (mut minimum, mut maximum) = meta
            .and_then(|meta| meta.range)
            .unwrap_or((f64::from(how.minimum), f64::from(how.maximum)));
        // The documented increment wins only when it is larger than the designer's decimal
        // places - a comparison of a step against a count, and it is what the C# does. `// C#: :80-84`
        let mut increment = how.increment;
        if let Some(documented) = meta.and_then(|meta| meta.increment)
            && documented > f64::from(how.designer_decimals)
        {
            // The C# reads it as a float.
            #[allow(clippy::cast_possible_truncation)]
            let documented = documented as f32;
            increment = documented;
        }
        let (mut step, mut decimals) = decimal_of(increment);
        let mut units = 0;
        let value = value_of(parameters, param);
        let (shown, enabled) = match value {
            Some(value) => {
                // `(decimal)((float)paramlist[ParamName] / _scale)`. `// C#: :101-111`
                #[allow(clippy::cast_possible_truncation)]
                let shown_value = value as f32 / how.scale;
                let (mantissa, places) = decimal_of(shown_value);
                if places > decimals {
                    step = rescale(step, decimals, places);
                    decimals = places;
                }
                units = rescale(mantissa, places, decimals);
                let exact = f64::from(shown_value);
                if exact < minimum {
                    minimum = exact;
                }
                if exact > maximum {
                    maximum = exact;
                }
                (true, true)
            }
            None => (how.designer_shown, false),
        };
        Self {
            param,
            bound: value.is_some(),
            shown,
            enabled,
            minimum,
            maximum,
            decimals,
            increment: step,
            value: units,
            scale: how.scale,
            changed: None,
        }
    }

    /// `Value`.
    #[must_use]
    pub fn value(&self) -> f64 {
        #[allow(clippy::cast_precision_loss)]
        let units = self.value as f64;
        units / 10_f64.powi(i32::try_from(self.decimals).unwrap_or(0))
    }

    /// The text the box shows: `Value` to `DecimalPlaces` places.
    #[must_use]
    pub fn text(&self) -> String {
        format!("{:.*}", self.decimals as usize, self.value())
    }

    /// The up or down arrow: one `Increment`, held to `Minimum` and `Maximum`. Returns whether
    /// the value changed, which is when `ValueChanged` starts the write timer.
    fn step(&mut self, up: bool, now: Instant) -> bool {
        if !self.enabled {
            return false;
        }
        let power = 10_f64.powi(i32::try_from(self.decimals).unwrap_or(0));
        #[allow(clippy::cast_possible_truncation)]
        let (lowest, highest) = (
            (self.minimum * power).ceil() as i64,
            (self.maximum * power).floor() as i64,
        );
        let next = if up {
            self.value.saturating_add(self.increment).min(highest)
        } else {
            self.value.saturating_sub(self.increment).max(lowest)
        };
        if next == self.value {
            return false;
        }
        self.value = next;
        self.changed = Some(now);
        true
    }

    /// What the timer writes: `(float)base.Value * (float)_scale`. `// C#: :169`
    #[must_use]
    pub fn written(&self) -> f64 {
        #[allow(clippy::cast_possible_truncation)]
        let value = self.value() as f32;
        f64::from(value * self.scale)
    }

    /// The write, once the timer has run out.
    fn due(&mut self, now: Instant) -> Option<Write> {
        let changed = self.changed?;
        if now.duration_since(changed) < WRITE_DELAY {
            return None;
        }
        self.changed = None;
        Some(Write::other(self.param, self.written()))
    }

    /// The write, now, whatever the timer says.
    fn flush(&mut self) -> Option<Write> {
        self.changed.take()?;
        Some(Write::other(self.param, self.written()))
    }
}

/// One of the Battery box's labelled rows - `PNL_low_bat`, `pnlmah`, `pnltimer` - a panel holding
/// a label and a number. The panel is the number's `enabledisable` control, which `setup` makes
/// visible either way and enables only for a parameter the vehicle has.
/// `// C#: Controls/MavlinkNumericUpDown.cs:124-131`
#[derive(Debug, Clone, PartialEq)]
pub struct Row {
    /// The label: `label4`, `label5`, `label6`.
    pub label: &'static str,
    /// The panel's `Visible`; `false` in the `.resx` until `setup` runs.
    pub shown: bool,
    /// The panel's `Enabled`.
    pub enabled: bool,
    /// The number.
    pub number: Numeric,
}

impl Row {
    /// A row whose number has been set up.
    fn setup(label: &'static str, number: Numeric) -> Self {
        Self {
            label,
            shown: true,
            enabled: number.enabled,
            number,
        }
    }
}

/// Which combo box.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComboId {
    /// `mavlinkComboBox_fs_thr_enable`.
    Throttle,
    /// `mavlinkComboBoxfs_batt_enable`.
    Battery,
}

/// Which check box.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CheckId {
    /// `mavlinkCheckBoxFS_GCS_ENABLE`.
    Gcs,
    /// `mavlinkCheckBoxthr_fs`.
    PlaneThrottle,
    /// `mavlinkCheckBoxthr_fs_action`.
    PlaneThrottleAction,
    /// `mavlinkCheckBoxgcs_fs`.
    PlaneGcs,
    /// `mavlinkCheckBoxshort_fs`.
    PlaneShort,
    /// `mavlinkCheckBoxlong_fs`.
    PlaneLong,
}

/// Which number.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NumberId {
    /// `mavlinkNumericUpDownlow_voltage`.
    LowVoltage,
    /// `mavlinkNumericUpDownFS_BATT_MAH`.
    ReservedMah,
    /// `mavlinkNumericUpDownBATT_LOW_TIMER`.
    LowTimer,
    /// `mavlinkNumericUpDownfs_thr_value`.
    ThrottlePwm,
    /// `mavlinkNumericUpDownthr_fs_value`.
    PlaneThrottlePwm,
}

/// Every control bound to a parameter, as `Activate` leaves them.
#[derive(Debug, Clone, PartialEq)]
pub struct Controls {
    /// `PNL_low_bat`.
    pub low_voltage: Row,
    /// `pnlmah`.
    pub reserved_mah: Row,
    /// `pnltimer`.
    pub low_timer: Row,
    /// `mavlinkComboBoxfs_batt_enable`.
    pub battery: Combo,
    /// `mavlinkComboBox_fs_thr_enable`.
    pub throttle: Combo,
    /// `mavlinkNumericUpDownfs_thr_value`.
    pub throttle_pwm: Numeric,
    /// `mavlinkNumericUpDownthr_fs_value`.
    pub plane_throttle_pwm: Numeric,
    /// `mavlinkCheckBoxthr_fs`.
    pub plane_throttle: Check,
    /// `mavlinkCheckBoxthr_fs_action`.
    pub plane_throttle_action: Check,
    /// `mavlinkCheckBoxFS_GCS_ENABLE`.
    pub gcs: Check,
    /// `mavlinkCheckBoxgcs_fs`.
    pub plane_gcs: Check,
    /// `mavlinkCheckBoxshort_fs`.
    pub plane_short: Check,
    /// `mavlinkCheckBoxlong_fs`.
    pub plane_long: Check,
}

impl Controls {
    /// `Activate`: every control bound to the parameter the vehicle has, in the C#'s order.
    /// `// C#: GCSViews/ConfigurationView/ConfigFailSafe.cs:24-89`
    #[must_use]
    pub fn activate(parameters: &[(String, f64)], lookup: Lookup) -> Self {
        let has = |name: &str| value_of(parameters, name).is_some();

        // `// C#: :26-28`
        let throttle = Combo::setup("FS_THR_ENABLE", parameters, lookup);
        // Copter's newer name first. `// C#: :30-42`
        let battery = Combo::setup(
            if has("BATT_FS_LOW_ACT") {
                "BATT_FS_LOW_ACT"
            } else {
                "FS_BATT_ENABLE"
            },
            parameters,
            lookup,
        );
        // `// C#: :43`; the designer sets no DecimalPlaces, so 0, and hides it.
        let throttle_pwm = Numeric::setup(
            &NumericSetup {
                minimum: 800.0,
                maximum: 1200.0,
                scale: 1.0,
                increment: 1.0,
                designer_decimals: 0,
                designer_shown: false,
            },
            "FS_THR_VALUE",
            parameters,
            lookup,
        );
        // The three generations of the low-voltage name, oldest first, with the newest as the
        // name used when the vehicle has none of them. `// C#: :45-59`; designer
        // `DecimalPlaces = 1` (`ConfigFailSafe.Designer.cs:135`).
        let low_voltage_name = if has("LOW_VOLT") {
            "LOW_VOLT"
        } else if has("FS_BATT_VOLTAGE") {
            "FS_BATT_VOLTAGE"
        } else {
            "BATT_LOW_VOLT"
        };
        let low_voltage = Row::setup(
            "Low Battery",
            Numeric::setup(
                &NumericSetup {
                    minimum: 6.0,
                    maximum: 99.0,
                    scale: 1.0,
                    increment: 0.1,
                    designer_decimals: 1,
                    designer_shown: true,
                },
                low_voltage_name,
                parameters,
                lookup,
            ),
        );
        // `// C#: :61-68`; designer `DecimalPlaces = 1` (`ConfigFailSafe.Designer.cs:263`).
        let reserved_mah = Row::setup(
            "Reserved MAH",
            Numeric::setup(
                &NumericSetup {
                    minimum: 0.0,
                    maximum: 99999.0,
                    scale: 1.0,
                    increment: 1.0,
                    designer_decimals: 1,
                    designer_shown: true,
                },
                if has("FS_BATT_MAH") {
                    "FS_BATT_MAH"
                } else {
                    "BATT_LOW_MAH"
                },
                parameters,
                lookup,
            ),
        );
        // Set up only when the vehicle has it; otherwise the row keeps the `.resx`'s hidden
        // panel. `// C#: :70-73`; designer `DecimalPlaces = 1` (`ConfigFailSafe.Designer.cs:605`).
        let timer = Numeric::setup(
            &NumericSetup {
                minimum: 0.0,
                maximum: 120.0,
                scale: 1.0,
                increment: 1.0,
                designer_decimals: 1,
                designer_shown: true,
            },
            "BATT_LOW_TIMER",
            parameters,
            lookup,
        );
        let low_timer = if has("BATT_LOW_TIMER") {
            Row::setup("Low Timer", timer)
        } else {
            Row {
                label: "Low Timer",
                shown: false,
                enabled: true,
                number: timer,
            }
        };
        // `// C#: :77`
        let gcs = Check::setup(1.0, 0.0, "FS_GCS_ENABLE", parameters);

        // Plane. `THR_FAILSAFE` enables `THR_FS_VALUE`'s box when checked, and `THR_FS_VALUE`'s
        // own setup, which comes after, enables it again whenever the vehicle has the parameter.
        // `// C#: :79-85`
        let plane_throttle = Check::setup(1.0, 0.0, "THR_FAILSAFE", parameters);
        let plane_throttle_pwm = Numeric::setup(
            &NumericSetup {
                minimum: 800.0,
                maximum: 1200.0,
                scale: 1.0,
                increment: 1.0,
                designer_decimals: 0,
                designer_shown: false,
            },
            "THR_FS_VALUE",
            parameters,
            lookup,
        );
        let plane_throttle_action = Check::setup(1.0, 0.0, "THR_FS_ACTION", parameters);
        let plane_gcs = Check::setup(1.0, 0.0, "FS_GCS_ENABL", parameters);
        let plane_short = Check::setup(1.0, 0.0, "FS_SHORT_ACTN", parameters);
        let plane_long = Check::setup(1.0, 0.0, "FS_LONG_ACTN", parameters);

        Self {
            low_voltage,
            reserved_mah,
            low_timer,
            battery,
            throttle,
            throttle_pwm,
            plane_throttle_pwm,
            plane_throttle,
            plane_throttle_action,
            gcs,
            plane_gcs,
            plane_short,
            plane_long,
        }
    }

    /// A combo box, by name.
    #[must_use]
    pub const fn combo(&self, id: ComboId) -> &Combo {
        match id {
            ComboId::Throttle => &self.throttle,
            ComboId::Battery => &self.battery,
        }
    }

    const fn combo_mut(&mut self, id: ComboId) -> &mut Combo {
        match id {
            ComboId::Throttle => &mut self.throttle,
            ComboId::Battery => &mut self.battery,
        }
    }

    /// A check box, by name.
    #[must_use]
    pub const fn check(&self, id: CheckId) -> &Check {
        match id {
            CheckId::Gcs => &self.gcs,
            CheckId::PlaneThrottle => &self.plane_throttle,
            CheckId::PlaneThrottleAction => &self.plane_throttle_action,
            CheckId::PlaneGcs => &self.plane_gcs,
            CheckId::PlaneShort => &self.plane_short,
            CheckId::PlaneLong => &self.plane_long,
        }
    }

    const fn check_mut(&mut self, id: CheckId) -> &mut Check {
        match id {
            CheckId::Gcs => &mut self.gcs,
            CheckId::PlaneThrottle => &mut self.plane_throttle,
            CheckId::PlaneThrottleAction => &mut self.plane_throttle_action,
            CheckId::PlaneGcs => &mut self.plane_gcs,
            CheckId::PlaneShort => &mut self.plane_short,
            CheckId::PlaneLong => &mut self.plane_long,
        }
    }

    /// A number, by name.
    #[must_use]
    pub const fn number(&self, id: NumberId) -> &Numeric {
        match id {
            NumberId::LowVoltage => &self.low_voltage.number,
            NumberId::ReservedMah => &self.reserved_mah.number,
            NumberId::LowTimer => &self.low_timer.number,
            NumberId::ThrottlePwm => &self.throttle_pwm,
            NumberId::PlaneThrottlePwm => &self.plane_throttle_pwm,
        }
    }

    const fn number_mut(&mut self, id: NumberId) -> &mut Numeric {
        match id {
            NumberId::LowVoltage => &mut self.low_voltage.number,
            NumberId::ReservedMah => &mut self.reserved_mah.number,
            NumberId::LowTimer => &mut self.low_timer.number,
            NumberId::ThrottlePwm => &mut self.throttle_pwm,
            NumberId::PlaneThrottlePwm => &mut self.plane_throttle_pwm,
        }
    }

    /// Every number, for the timers.
    fn numbers_mut(&mut self) -> [&mut Numeric; 5] {
        [
            &mut self.low_voltage.number,
            &mut self.reserved_mah.number,
            &mut self.low_timer.number,
            &mut self.throttle_pwm,
            &mut self.plane_throttle_pwm,
        ]
    }

    /// The Radio box's number that shows: `FS_THR_VALUE` on a copter, `THR_FS_VALUE` on a plane.
    #[must_use]
    pub const fn pwm(&self) -> Option<&Numeric> {
        if self.throttle_pwm.shown {
            Some(&self.throttle_pwm)
        } else if self.plane_throttle_pwm.shown {
            Some(&self.plane_throttle_pwm)
        } else {
            None
        }
    }
}

/// A parameter's value in the vehicle's table.
fn value_of(parameters: &[(String, f64)], name: &str) -> Option<f64> {
    parameters
        .iter()
        .find(|(held, _)| held == name)
        .map(|(_, value)| *value)
}

/// Whether Initial Setup lists the page: connected, and every parameter the vehicle reported
/// received (`gotAllParams`: `TotalReceived >= TotalReported`).
///
/// One condition the C# does not have: a table that is not empty. Mission Planner downloads the
/// parameters as part of connecting, so an empty table while connected does not arise there;
/// here they are downloaded when asked for, and a page opened on an empty table would hide every
/// control it has.
/// `// C#: GCSViews/InitialSetup.cs:120-131, 230-233`
#[must_use]
pub fn available(view: &TelemetryView) -> bool {
    view.connected
        && view.vehicle.is_some()
        && !view.parameters.is_empty()
        && view.parameters.len() >= usize::from(view.parameters_expected)
}

/// `lbl_armed`'s text: the bound `True` or `False`, repainted as a word.
/// `// C#: GCSViews/ConfigurationView/ConfigFailSafe.cs:124-136`
#[must_use]
pub const fn armed_text(armed: bool) -> &'static str {
    if armed { "Armed" } else { "Disarmed" }
}

/// `lbl_gpslock`'s text for the bound `gpsstatus`. Blank past 3D: the paint handler assigns the
/// empty string it started with to every fix it does not name, which is every RTK and DGPS fix.
/// `// C#: GCSViews/ConfigurationView/ConfigFailSafe.cs:138-171`
#[must_use]
pub const fn gps_text(fix_type: u8) -> &'static str {
    match fix_type {
        0 => "GPS: No GPS",
        1 => "GPS: No Fix",
        // The C# says 3D for a 2D fix too.
        2 | 3 => "GPS: 3D Fix",
        _ => "",
    }
}

/// The mode as `lbl_currentmode` shows it, named as the flight screen names it.
#[must_use]
pub fn mode_text(state: &VehicleState) -> String {
    mp_vehicle::flight_mode_name(state.vehicle_type, state.custom_mode)
        .map_or_else(|| format!("mode {}", state.custom_mode), ToOwned::to_owned)
}

/// `ch1in` to `ch8in`: `RC_CHANNELS` as sent, zero before one has arrived.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3495-3509`
#[must_use]
pub fn radio_in(state: &VehicleState, channel: usize) -> i64 {
    if !state.rc.reported {
        return 0;
    }
    state
        .rc
        .values
        .get(channel.wrapping_sub(1))
        .map_or(0, |value| i64::from(*value))
}

/// `ch1out` to `ch8out`: `SERVO_OUTPUT_RAW` port 0.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3638-3647`
#[must_use]
pub fn servo_out(state: &VehicleState, channel: usize) -> i64 {
    state
        .servo_outputs
        .get(channel.wrapping_sub(1))
        .map_or(0, |value| i64::from(*value))
}

/// How full a bar is drawn: `HorizontalProgressBar.Value`'s clamp, one above the minimum at the
/// bottom and the maximum at the top, over the bar's 1000 to 2000.
/// `// C#: ExtLibs/Controls/HorizontalProgressBar.cs:74-115`
#[must_use]
pub fn bar_fraction(value: i64) -> f32 {
    let shown = if value <= BAR_MINIMUM {
        BAR_MINIMUM + 1
    } else if value >= BAR_MAXIMUM {
        BAR_MAXIMUM
    } else {
        value
    };
    #[allow(clippy::cast_precision_loss)]
    let fraction = (shown - BAR_MINIMUM) as f32 / (BAR_MAXIMUM - BAR_MINIMUM) as f32;
    fraction
}

/// Where `LNK_wiki` goes: the copter failsafe page for a copter, the plane page for anything
/// else. `// C#: GCSViews/ConfigurationView/ConfigFailSafe.cs:112-122`
#[must_use]
pub fn wiki_url(family: Option<VehicleFamily>) -> &'static str {
    if family == Some(VehicleFamily::Copter) {
        "https://ardupilot.org/copter/docs/failsafe-landing-page.html"
    } else {
        "https://ardupilot.org/plane/docs/advanced-failsafe-configuration.html"
    }
}

/// `lbl_currentmode`'s colour.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeColour {
    /// The control's own, until a mode change sets one.
    Default,
    /// `Color.White`: the throttle is at or above `FS_THR_VALUE`.
    White,
    /// `Color.Red`: the throttle is below `FS_THR_VALUE`.
    Red,
}

impl ModeColour {
    /// The word a fact carries.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::Default => "default",
            Self::White => "white",
            Self::Red => "red",
        }
    }
}

/// A write on its way to the vehicle.
#[derive(Debug, Clone)]
struct InFlight {
    id: mp_link::RequestId,
    write: Write,
}

/// The page, and everything it keeps while it is open.
#[derive(Debug, Default)]
pub struct FailSafe {
    /// The controls, while the page is open.
    controls: Option<Controls>,
    /// The combo box whose list is dropped down.
    dropdown: Option<ComboId>,
    /// Message boxes, the first showing. Modal, as `CustomMessageBox.Show` is.
    messages: VecDeque<Message>,
    /// Writes the link is carrying.
    in_flight: Vec<InFlight>,
    /// How the last write ended, for a test to read.
    last_write: Option<String>,
    /// `lbl_currentmode.Text`, once the vehicle's mode has reached it.
    mode: Option<String>,
    /// `lbl_currentmode.ForeColor`.
    mode_colour: Option<ModeColour>,
}

impl FailSafe {
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

    /// The message box showing, if one is.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// The combo box whose list is down.
    #[must_use]
    pub const fn dropdown(&self) -> Option<ComboId> {
        self.dropdown
    }

    /// How the last write ended.
    #[must_use]
    pub fn last_write(&self) -> Option<&str> {
        self.last_write.as_deref()
    }

    /// `lbl_currentmode`'s text.
    #[must_use]
    pub fn mode(&self) -> &str {
        self.mode.as_deref().unwrap_or(DESIGNER_MODE)
    }

    /// `lbl_currentmode`'s colour.
    #[must_use]
    pub fn mode_colour(&self) -> ModeColour {
        self.mode_colour.unwrap_or(ModeColour::Default)
    }

    /// Opens the page: `Activate`, which binds every control and then shows the warning.
    /// `// C#: GCSViews/ConfigurationView/ConfigFailSafe.cs:24-93`
    pub fn open(&mut self, parameters: &[(String, f64)], lookup: Lookup) {
        self.controls = Some(Controls::activate(parameters, lookup));
        self.dropdown = None;
        self.mode = None;
        self.mode_colour = None;
        self.messages.push_back(Message {
            title: PROPS_TITLE,
            text: PROPS_WARNING.to_owned(),
        });
    }

    /// Closes the page, returning what its number timers still had to write: the C#'s timers
    /// outlive the page, so a change made just before leaving still reaches the vehicle.
    pub fn close(&mut self) -> Vec<Write> {
        let pending = self
            .controls
            .as_mut()
            .map(|controls| {
                controls
                    .numbers_mut()
                    .into_iter()
                    .filter_map(Numeric::flush)
                    .collect()
            })
            .unwrap_or_default();
        self.controls = None;
        self.dropdown = None;
        self.mode = None;
        self.mode_colour = None;
        pending
    }

    /// Dismisses the message box showing.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// Drops a combo box's list down, or back up.
    pub fn toggle_dropdown(&mut self, id: ComboId) {
        self.dropdown = if self.dropdown == Some(id) {
            None
        } else {
            Some(id)
        };
    }

    /// Chooses an option from a combo box's list, closing it.
    pub fn choose(&mut self, id: ComboId, key: i64) -> Option<Write> {
        self.dropdown = None;
        self.controls.as_mut()?.combo_mut(id).choose(key)
    }

    /// Clicks a check box. `THR_FAILSAFE`'s box enables or disables `THR_FS_VALUE`'s as it goes.
    /// `// C#: Controls/MavlinkCheckBox.cs:100-104, 111-129`
    pub fn click(&mut self, id: CheckId) -> Option<Write> {
        let controls = self.controls.as_mut()?;
        let write = controls.check_mut(id).click()?;
        if id == CheckId::PlaneThrottle {
            controls.plane_throttle_pwm.enabled =
                controls.plane_throttle.state == CheckState::Checked;
        }
        Some(write)
    }

    /// Steps a number; the write follows once its timer runs out.
    pub fn step(&mut self, id: NumberId, up: bool, now: Instant) -> bool {
        self.controls
            .as_mut()
            .is_some_and(|controls| controls.number_mut(id).step(up, now))
    }

    /// The numbers whose timers have run out.
    pub fn due(&mut self, now: Instant) -> Vec<Write> {
        self.controls
            .as_mut()
            .map(|controls| {
                controls
                    .numbers_mut()
                    .into_iter()
                    .filter_map(|number| number.due(now))
                    .collect()
            })
            .unwrap_or_default()
    }

    /// `lbl_currentmode_TextChanged`: when the mode's text changes, red if the throttle is below
    /// `FS_THR_VALUE` and white otherwise, and nothing at all for a vehicle without that
    /// parameter. The colour holds until the next change.
    /// `// C#: GCSViews/ConfigurationView/ConfigFailSafe.cs:173-192`
    pub fn observe_mode(&mut self, text: &str, ch3in: i64, fs_thr_value: Option<f64>) {
        if self.mode() == text {
            return;
        }
        self.mode = Some(text.to_owned());
        if let Some(threshold) = fs_thr_value {
            #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
            let below = (ch3in as f32) < threshold as f32;
            self.mode_colour = Some(if below {
                ModeColour::Red
            } else {
                ModeColour::White
            });
        }
    }

    /// Sends a write through the link's retrying set. With no vehicle to send to it has failed
    /// already, and says so as the C# does.
    pub fn issue(&mut self, telemetry: &Telemetry, write: Write) {
        match telemetry.set_parameter_confirmed(write.param, write.value) {
            Some(id) => self.in_flight.push(InFlight { id, write }),
            None => self.failed(&write, "no vehicle"),
        }
    }

    /// A failed write: the C#'s message box.
    fn failed(&mut self, write: &Write, why: &str) {
        self.last_write = Some(format!("{} {} failed: {why}", write.param, write.value));
        self.messages.push_back(Message {
            title: ERROR_TITLE,
            text: write.failure.clone(),
        });
    }

    /// Reads back how each write the link is carrying ended.
    fn settle(&mut self, telemetry: &Telemetry) {
        let mut still = Vec::new();
        for flight in std::mem::take(&mut self.in_flight) {
            let Some(request) = telemetry.request(flight.id) else {
                continue;
            };
            match request.outcome() {
                None => still.push(flight),
                Some(RequestOutcome::Accepted { value }) => {
                    let echoed = value.map_or(flight.write.value, |value| value.as_f64());
                    self.last_write = Some(format!("{} {echoed} accepted", flight.write.param));
                }
                // "not modified as same": the C#'s setParam returns true.
                Some(RequestOutcome::Unchanged | RequestOutcome::Sent) => {
                    self.last_write = Some(format!(
                        "{} {} unchanged",
                        flight.write.param, flight.write.value
                    ));
                }
                Some(RequestOutcome::UnknownParameter) => {
                    self.failed(&flight.write, "not on the vehicle");
                }
                Some(RequestOutcome::TimedOut) => self.failed(&flight.write, "timed out"),
                Some(RequestOutcome::Rejected(result)) => {
                    self.failed(&flight.write, &format!("rejected {result}"));
                }
            }
        }
        self.in_flight = still;
    }

    /// Once a frame. Leaving the setup screen closes the page, as leaving Initial Setup disposes
    /// it; the timers that ran out are written, the writes in flight read back, and the mode's
    /// colour kept as `timer_Tick`'s rebinding would keep it.
    /// `// C#: GCSViews/ConfigurationView/ConfigFailSafe.cs:95-110`
    pub fn tick(&mut self, telemetry: &Telemetry, view: &TelemetryView, on_setup: bool) {
        let now = Instant::now();
        let mut writes = if on_setup || !self.is_open() {
            Vec::new()
        } else {
            self.close()
        };
        writes.extend(self.due(now));
        for write in writes {
            self.issue(telemetry, write);
        }
        self.settle(telemetry);
        if self.is_open()
            && let Some(state) = view.state.as_deref()
        {
            let fs_thr_value = view
                .parameters
                .iter()
                .find(|(name, _)| name == "FS_THR_VALUE")
                .map(|(_, value)| *value);
            self.observe_mode(&mode_text(state), radio_in(state, 3), fs_thr_value);
        }
    }

    /// The page's entry on the setup screen: opens it, or closes it again.
    pub fn toggle(&mut self, telemetry: &Telemetry) {
        if self.is_open() {
            for write in self.close() {
                self.issue(telemetry, write);
            }
        } else {
            self.open(&telemetry.view().parameters, crate::metadata::lookup);
        }
    }
}

/// Facts a UI test asserts on: what the page shows and what the vehicle holds.
pub fn record_facts(failsafe: &FailSafe, view: &TelemetryView) {
    use crate::facts::record;
    record("config.failsafe.available", available(view));
    record("config.failsafe.open", failsafe.is_open());
    record(
        "config.failsafe.message",
        failsafe
            .message()
            .map_or("none", |message| message.text.as_str()),
    );
    record(
        "config.failsafe.write",
        failsafe.last_write().unwrap_or("none"),
    );
    record("config.failsafe.writes.pending", failsafe.in_flight.len());

    let vehicle = |name: &str| {
        value_of(&view.parameters, name)
            .map_or_else(|| "none".to_owned(), |value| value.to_string())
    };
    let controls = failsafe.controls();
    let combo = |key: &str, combo: Option<&Combo>| {
        let shown = combo.filter(|combo| combo.shown);
        record(
            format!("config.failsafe.{key}.param"),
            shown.map_or("none", |combo| combo.param),
        );
        record(
            format!("config.failsafe.{key}"),
            shown
                .and_then(|combo| combo.selected)
                .map_or_else(|| "none".to_owned(), |value| value.to_string()),
        );
        record(
            format!("config.failsafe.{key}.text"),
            shown.map_or("", Combo::text),
        );
        record(
            format!("config.failsafe.{key}.options"),
            shown.map_or(0, |combo| combo.options.len()),
        );
        record(
            format!("config.failsafe.{key}.vehicle"),
            shown.map_or_else(|| "none".to_owned(), |combo| vehicle(combo.param)),
        );
    };
    combo("throttle", controls.map(|controls| &controls.throttle));
    combo("battery", controls.map(|controls| &controls.battery));

    let number = |key: &str, number: Option<&Numeric>| {
        let shown = number.filter(|number| number.bound);
        record(
            format!("config.failsafe.{key}.param"),
            shown.map_or("none", |number| number.param),
        );
        record(
            format!("config.failsafe.{key}"),
            shown.map_or_else(|| "none".to_owned(), Numeric::text),
        );
    };
    number(
        "voltage",
        controls.map(|controls| &controls.low_voltage.number),
    );
    number(
        "mah",
        controls.map(|controls| &controls.reserved_mah.number),
    );
    number(
        "timer",
        controls
            .filter(|controls| controls.low_timer.shown)
            .map(|controls| &controls.low_timer.number),
    );
    number("pwm", controls.and_then(Controls::pwm));

    let gcs = controls
        .map(|controls| &controls.gcs)
        .filter(|check| check.shown);
    record(
        "config.failsafe.gcs.param",
        gcs.map_or("none", |check| check.param),
    );
    record(
        "config.failsafe.gcs",
        gcs.map_or("none", |check| check.state.key()),
    );
    record(
        "config.failsafe.gcs.vehicle",
        gcs.map_or_else(|| "none".to_owned(), |check| vehicle(check.param)),
    );
    record(
        "config.failsafe.plane.shown",
        controls.map_or(0, |controls| {
            [
                controls.plane_throttle,
                controls.plane_throttle_action,
                controls.plane_gcs,
                controls.plane_short,
                controls.plane_long,
            ]
            .iter()
            .filter(|check| check.shown)
            .count()
        }),
    );

    record("config.failsafe.mode", failsafe.mode());
    record("config.failsafe.mode.colour", failsafe.mode_colour().key());
    let state = view.state.as_deref();
    record(
        "config.failsafe.armed",
        state.map_or("", |state| armed_text(state.armed)),
    );
    record(
        "config.failsafe.gps",
        state.map_or("", |state| gps_text(state.gps.fix_type)),
    );
    for channel in 1..=8 {
        record(
            format!("config.failsafe.ch{channel}"),
            state.map_or(0, |state| radio_in(state, channel)),
        );
        record(
            format!("config.failsafe.out{channel}"),
            state.map_or(0, |state| servo_out(state, channel)),
        );
    }
}

/// An absolutely placed box, at a `.resx` `Location` and `Size`.
fn at(x: f32, y: f32, width: f32, height: f32) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// One `HorizontalProgressBar`, with the label and the value its `drawlbl` puts under it.
/// `// C#: ExtLibs/Controls/HorizontalProgressBar.cs:166-179`
fn bar(x: f32, y: f32, label: String, value: i64) -> impl IntoElement {
    at(x, y, 170.0, 51.0)
        .flex()
        .flex_col()
        .child(
            div()
                .h(px(23.0))
                .w_full()
                .rounded_sm()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::BG))
                .child(
                    div()
                        .h_full()
                        .w(gpui::relative(bar_fraction(value)))
                        .bg(rgb(theme::OK)),
                ),
        )
        .child(
            div()
                .mt(px(2.0))
                .h(px(13.0))
                .flex()
                .justify_center()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(label),
        )
        .child(
            div()
                .h(px(13.0))
                .flex()
                .justify_center()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(value.to_string()),
        )
}

/// A `GroupBox`: a border with its caption.
fn group(x: f32, y: f32, width: f32, height: f32, title: &'static str) -> Div {
    at(x, y, width, height)
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
                .child(title),
        )
}

/// A combo box: the selected option's text, and a click to drop its list down.
fn combo_box(
    id: ComboId,
    combo: &Combo,
    x: f32,
    y: f32,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if !combo.shown {
        return None;
    }
    Some(
        at(x, y, 199.0, 20.0)
            .child(
                crate::probe::measured(format!("failsafe-{}", combo.param), div())
                    .id(SharedString::from(format!("failsafe-{}", combo.param)))
                    .size_full()
                    .flex()
                    .items_center()
                    .px_1()
                    .rounded_sm()
                    .border_1()
                    .border_color(rgb(theme::BORDER))
                    .bg(rgb(theme::ACTION))
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .cursor_pointer()
                    .hover(|style| style.border_color(rgb(theme::ACCENT)))
                    .child(combo.text().to_owned())
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        this.failsafe.toggle_dropdown(id);
                        cx.notify();
                    })),
            )
            .into_any_element(),
    )
}

/// A combo box's list, dropped down over whatever is under it as a WinForms list is.
fn dropdown(
    id: ComboId,
    combo: &Combo,
    x: f32,
    y: f32,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let mut list = div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(199.0))
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .occlude();
    for (key, text) in &combo.options {
        let key = *key;
        let selected = combo.selected == Some(key);
        let name = format!("failsafe-{}-{key}", combo.param);
        list = list.child(
            crate::probe::measured(name.clone(), div())
                .id(SharedString::from(name))
                .px_1()
                .py(px(1.0))
                .text_xs()
                .bg(rgb(if selected {
                    theme::BORDER
                } else {
                    theme::PANEL
                }))
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::ACTION)))
                .child(text.clone())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    if let Some(write) = this.failsafe.choose(id, key) {
                        this.failsafe.issue(&this.telemetry, write);
                    }
                    cx.notify();
                })),
        );
    }
    list.into_any_element()
}

/// A check box and its text.
fn check_box(
    id: CheckId,
    check: &Check,
    text: &'static str,
    x: f32,
    y: f32,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if !check.shown {
        return None;
    }
    let mark = match check.state {
        CheckState::Unchecked => None,
        CheckState::Checked => Some(div().size(px(6.0)).bg(rgb(theme::ACCENT))),
        CheckState::Indeterminate => Some(div().w(px(6.0)).h(px(2.0)).bg(rgb(theme::ACCENT))),
    };
    Some(
        div()
            .absolute()
            .left(px(x))
            .top(px(y))
            .child(
                crate::probe::measured(format!("failsafe-{}", check.param), div())
                    .id(SharedString::from(format!("failsafe-{}", check.param)))
                    .flex()
                    .items_center()
                    .gap_1()
                    .cursor_pointer()
                    .child(
                        div()
                            .size(px(12.0))
                            .flex()
                            .items_center()
                            .justify_center()
                            .border_1()
                            .border_color(rgb(theme::DIM))
                            .bg(rgb(theme::BG))
                            .children(mark),
                    )
                    .child(div().text_xs().text_color(rgb(theme::TEXT)).child(text))
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        if let Some(write) = this.failsafe.click(id) {
                            this.failsafe.issue(&this.telemetry, write);
                        }
                        cx.notify();
                    })),
            )
            .into_any_element(),
    )
}

/// A number with its arrows: the value, and a step down and a step up.
fn number_box(
    id: NumberId,
    number: &Numeric,
    width: f32,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let arrow = |up: bool, cx: &mut Context<MissionPlanner>| {
        let name = format!(
            "failsafe-{}-{}",
            number.param,
            if up { "up" } else { "down" }
        );
        let base = crate::probe::measured(name.clone(), div())
            .id(SharedString::from(name))
            .w(px(14.0))
            .h_full()
            .flex()
            .items_center()
            .justify_center()
            .border_l_1()
            .border_color(rgb(theme::BORDER))
            .text_xs()
            .child(if up { "+" } else { "-" });
        if number.enabled {
            base.text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.failsafe.step(id, up, Instant::now());
                    cx.notify();
                }))
                .into_any_element()
        } else {
            base.text_color(rgb(theme::DIM)).into_any_element()
        }
    };
    div()
        .w(px(width))
        .h(px(19.0))
        .flex()
        .items_center()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        .child(
            crate::probe::measured(format!("failsafe-{}", number.param), div())
                .flex_1()
                .px_1()
                .text_xs()
                .text_color(rgb(if number.enabled {
                    theme::TEXT
                } else {
                    theme::DIM
                }))
                .child(number.text()),
        )
        .child(arrow(false, cx))
        .child(arrow(true, cx))
        .into_any_element()
}

/// One of the Battery box's rows: its label, and its number at x 114.
fn battery_row(
    id: NumberId,
    row: &Row,
    y: f32,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if !row.shown {
        return None;
    }
    Some(
        at(6.0, y, 199.0, 21.0)
            .child(
                div()
                    .absolute()
                    .left(px(4.0))
                    .top(px(3.0))
                    .text_xs()
                    .text_color(rgb(if row.enabled { theme::TEXT } else { theme::DIM }))
                    .child(row.label),
            )
            .child(
                div()
                    .absolute()
                    .left(px(114.0))
                    .top(px(0.0))
                    .child(number_box(id, &row.number, 85.0, cx)),
            )
            .into_any_element(),
    )
}

/// The page, laid out as `ConfigFailSafe.resx` lays it out.
pub fn page(
    failsafe: &FailSafe,
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let Some(controls) = failsafe.controls() else {
        return div().into_any_element();
    };
    let state = view.state.as_deref();
    let family = state.and_then(|state| VehicleFamily::from_mav_type(state.vehicle_type));

    let mut body = div().relative().w(px(688.0)).h(px(482.0));

    // `label1`, `label2`, and the sixteen bars: radio in on the left, servo out beside it.
    body = body
        .child(
            div()
                .absolute()
                .left(px(84.0))
                .top(px(5.0))
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child("Radio IN"),
        )
        .child(
            div()
                .absolute()
                .left(px(342.0))
                .top(px(5.0))
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child("Servo/Motor OUT"),
        );
    for (index, y) in BAR_ROWS.iter().enumerate() {
        let channel = index + 1;
        body = body
            .child(bar(
                15.0,
                *y,
                format!("Radio {channel}"),
                state.map_or(0, |state| radio_in(state, channel)),
            ))
            .child(bar(
                297.0,
                *y,
                format!("Radio {channel}"),
                state.map_or(0, |state| servo_out(state, channel)),
            ));
    }

    // The three large readouts and the wiki link.
    let mode_colour = match failsafe.mode_colour() {
        ModeColour::Red => theme::ALERT,
        ModeColour::White | ModeColour::Default => theme::TEXT,
    };
    body = body
        .child(
            at(473.0, 19.0, 215.0, 32.0)
                .flex()
                .items_center()
                .justify_center()
                .text_3xl()
                .text_color(rgb(mode_colour))
                .child(failsafe.mode().to_owned()),
        )
        .child(
            at(473.0, 60.0, 215.0, 33.0)
                .flex()
                .items_center()
                .text_3xl()
                .text_color(rgb(theme::TEXT))
                .child(state.map_or("", |state| armed_text(state.armed))),
        )
        .child(
            at(473.0, 102.0, 215.0, 30.0)
                .flex()
                .items_center()
                .text_3xl()
                .text_color(rgb(theme::TEXT))
                .child(state.map_or("", |state| gps_text(state.gps.fix_type))),
        )
        .child(
            div().absolute().right(px(5.0)).top(px(5.0)).child(
                crate::probe::measured("failsafe-wiki", div())
                    .id("failsafe-wiki")
                    .text_xs()
                    .text_color(rgb(theme::ACCENT))
                    .cursor_pointer()
                    .hover(|style| style.underline())
                    .child("Wiki")
                    .on_click(cx.listener(move |_this, _event, _window, cx| {
                        cx.open_url(wiki_url(family));
                    })),
            ),
        );

    // groupBox4, "Battery".
    let battery = group(477.0, 146.0, 208.0, 121.0, "Battery")
        .children(battery_row(
            NumberId::LowTimer,
            &controls.low_timer,
            68.0,
            cx,
        ))
        .children(combo_box(
            ComboId::Battery,
            &controls.battery,
            6.0,
            92.0,
            cx,
        ))
        .children(battery_row(
            NumberId::LowVoltage,
            &controls.low_voltage,
            18.0,
            cx,
        ))
        .children(battery_row(
            NumberId::ReservedMah,
            &controls.reserved_mah,
            43.0,
            cx,
        ));

    // groupBox2, "Radio". `PNL_thr_fs_value` docks its label left and whichever number shows
    // right.
    let mut pwm = div().flex();
    if controls.throttle_pwm.shown {
        pwm = pwm.child(number_box(
            NumberId::ThrottlePwm,
            controls.number(NumberId::ThrottlePwm),
            93.0,
            cx,
        ));
    }
    if controls.plane_throttle_pwm.shown {
        pwm = pwm.child(number_box(
            NumberId::PlaneThrottlePwm,
            controls.number(NumberId::PlaneThrottlePwm),
            103.0,
            cx,
        ));
    }
    let radio = group(477.0, 273.0, 208.0, 96.0, "Radio")
        .children(combo_box(
            ComboId::Throttle,
            &controls.throttle,
            6.0,
            13.0,
            cx,
        ))
        .child(
            at(6.0, 35.0, 199.0, 23.0)
                .flex()
                .items_center()
                .justify_between()
                .child(div().text_xs().text_color(rgb(theme::TEXT)).child("FS Pwm"))
                .child(pwm),
        )
        .children(check_box(
            CheckId::PlaneThrottle,
            controls.check(CheckId::PlaneThrottle),
            "Throttle FailSafe",
            6.0,
            60.0,
            cx,
        ))
        .children(check_box(
            CheckId::PlaneThrottleAction,
            controls.check(CheckId::PlaneThrottleAction),
            "Throttle Failsafe Action",
            6.0,
            77.0,
            cx,
        ));

    // groupBox3, "GCS".
    let gcs = group(477.0, 375.0, 208.0, 73.0, "GCS")
        .children(check_box(
            CheckId::Gcs,
            controls.check(CheckId::Gcs),
            "GCS FS Enable",
            6.0,
            14.0,
            cx,
        ))
        .children(check_box(
            CheckId::PlaneGcs,
            controls.check(CheckId::PlaneGcs),
            "GCS FailSafe",
            6.0,
            29.0,
            cx,
        ))
        .children(check_box(
            CheckId::PlaneShort,
            controls.check(CheckId::PlaneShort),
            "FailSafe Short (1 sec)",
            6.0,
            43.0,
            cx,
        ))
        .children(check_box(
            CheckId::PlaneLong,
            controls.check(CheckId::PlaneLong),
            "FailSafe Long (20 sec)",
            6.0,
            58.0,
            cx,
        ));

    body = body.child(battery).child(radio).child(gcs);

    // A dropped-down list goes last, so it draws over the boxes below it.
    if let Some(id) = failsafe.dropdown() {
        let combo = controls.combo(id);
        let (x, y) = match id {
            ComboId::Battery => (477.0 + 6.0, 146.0 + 92.0 + 20.0),
            ComboId::Throttle => (477.0 + 6.0, 273.0 + 13.0 + 20.0),
        };
        if combo.shown {
            body = body.child(dropdown(id, combo, x, y, cx));
        }
    }

    panel("FailSafe", body).into_any_element()
}

/// The message box showing, drawn over the whole window as `CustomMessageBox.Show` is modal.
pub fn overlay(
    failsafe: &FailSafe,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let message = failsafe.message()?;
    let size = window.viewport_size();
    let dialog = crate::probe::measured("failsafe-message", div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(340.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(if message.title == ERROR_TITLE {
            theme::ALERT
        } else {
            theme::WARN
        }))
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
                .child(message.text.clone()),
        )
        .child(div().flex().justify_end().child(action(
            "failsafe-message-ok",
            "OK",
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.failsafe.dismiss_message();
                cx.notify();
            }),
        )));
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(
                    div()
                        .id("failsafe-message-backdrop")
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
    use super::*;

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

    /// A copter as SITL's defaults leave it: the names ArduCopter 4.x has.
    fn copter() -> Vec<(String, f64)> {
        table(&[
            ("FS_THR_ENABLE", 1.0),
            ("FS_THR_VALUE", 975.0),
            ("BATT_FS_LOW_ACT", 0.0),
            ("BATT_LOW_VOLT", 10.5),
            ("BATT_LOW_MAH", 0.0),
            ("BATT_LOW_TIMER", 10.0),
            ("FS_GCS_ENABLE", 0.0),
        ])
    }

    /// An older copter: the battery failsafe under its first names, and no low timer.
    fn old_copter() -> Vec<(String, f64)> {
        table(&[
            ("FS_THR_ENABLE", 2.0),
            ("FS_THR_VALUE", 975.0),
            ("FS_BATT_ENABLE", 1.0),
            ("FS_BATT_VOLTAGE", 10.5),
            ("FS_BATT_MAH", 1200.0),
            ("FS_GCS_ENABLE", 1.0),
        ])
    }

    /// A plane: its own throttle and GCS failsafe names, and none of a copter's.
    fn plane() -> Vec<(String, f64)> {
        table(&[
            ("THR_FAILSAFE", 1.0),
            ("THR_FS_VALUE", 950.0),
            ("THR_FS_ACTION", 0.0),
            ("FS_GCS_ENABL", 2.0),
            ("FS_SHORT_ACTN", 0.0),
            ("FS_LONG_ACTN", 1.0),
            ("BATT_FS_LOW_ACT", 0.0),
            ("BATT_LOW_VOLT", 0.0),
            ("BATT_LOW_MAH", 0.0),
        ])
    }

    #[test]
    fn a_current_copter_binds_the_battery_failsafe_names_it_has() {
        let controls = Controls::activate(&copter(), bundled);
        assert_eq!(controls.throttle.param, "FS_THR_ENABLE");
        assert!(controls.throttle.shown);
        assert_eq!(controls.battery.param, "BATT_FS_LOW_ACT");
        assert!(controls.battery.shown);
        assert_eq!(controls.low_voltage.number.param, "BATT_LOW_VOLT");
        assert_eq!(controls.reserved_mah.number.param, "BATT_LOW_MAH");
        assert_eq!(controls.low_timer.number.param, "BATT_LOW_TIMER");
        assert!(controls.low_timer.shown, "BATT_LOW_TIMER's row shows");
        assert_eq!(controls.gcs.param, "FS_GCS_ENABLE");
        assert!(controls.gcs.shown);
        assert_eq!(controls.pwm().map(|pwm| pwm.param), Some("FS_THR_VALUE"));
        // None of a plane's controls.
        for check in [
            controls.plane_throttle,
            controls.plane_throttle_action,
            controls.plane_gcs,
            controls.plane_short,
            controls.plane_long,
        ] {
            assert!(!check.shown, "{} shows on a copter", check.param);
        }
        assert!(!controls.plane_throttle_pwm.shown);
    }

    /// The whole of a real SITL copter's parameters, as `mpr param save` wrote them: the page
    /// binds what `config-failsafe.gui` expects to find on the live vehicle.
    #[test]
    fn the_sitl_copter_dump_binds_the_names_the_gui_script_expects() {
        let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/params/sitl-copter.param");
        let file = mp_params::param_file::ParamFile::load(&fixture).expect("the SITL dump");
        let parameters: Vec<(String, f64)> = file
            .iter()
            .map(|(name, value)| (name.to_owned(), value))
            .collect();
        assert!(parameters.len() > 1000, "{}", parameters.len());
        let controls = Controls::activate(&parameters, bundled);
        assert_eq!(controls.throttle.selected, Some(1));
        assert_eq!(controls.throttle.text(), "Enabled always RTL");
        assert_eq!(controls.battery.param, "BATT_FS_LOW_ACT");
        assert_eq!(controls.battery.text(), "None");
        assert_eq!(controls.low_voltage.number.param, "BATT_LOW_VOLT");
        assert_eq!(controls.low_voltage.number.text(), "10.5");
        assert_eq!(controls.reserved_mah.number.param, "BATT_LOW_MAH");
        assert!(controls.low_timer.shown);
        assert_eq!(controls.low_timer.number.text(), "10");
        assert_eq!(controls.pwm().map(Numeric::text), Some("975".to_owned()));
        assert_eq!(controls.gcs.state, CheckState::Unchecked);
    }

    #[test]
    fn an_older_copter_binds_the_first_names() {
        let controls = Controls::activate(&old_copter(), bundled);
        assert_eq!(controls.battery.param, "FS_BATT_ENABLE");
        assert_eq!(controls.low_voltage.number.param, "FS_BATT_VOLTAGE");
        assert_eq!(controls.reserved_mah.number.param, "FS_BATT_MAH");
        // No BATT_LOW_TIMER: setup is never called, and the panel keeps the .resx's Visible=False.
        assert!(!controls.low_timer.shown);
    }

    #[test]
    fn low_volt_comes_before_either_later_name() {
        let parameters = table(&[
            ("LOW_VOLT", 9.9),
            ("FS_BATT_VOLTAGE", 10.5),
            ("BATT_LOW_VOLT", 10.5),
        ]);
        let controls = Controls::activate(&parameters, bundled);
        assert_eq!(controls.low_voltage.number.param, "LOW_VOLT");
        assert_eq!(controls.low_voltage.number.text(), "9.9");
    }

    #[test]
    fn with_no_battery_names_the_rows_show_disabled_under_the_newest_names() {
        // MavlinkNumericUpDown.enableControl(false) makes the panel visible and disabled.
        let controls = Controls::activate(&table(&[("FS_THR_ENABLE", 1.0)]), bundled);
        assert_eq!(controls.low_voltage.number.param, "BATT_LOW_VOLT");
        assert_eq!(controls.reserved_mah.number.param, "BATT_LOW_MAH");
        assert!(controls.low_voltage.shown && !controls.low_voltage.enabled);
        assert!(controls.reserved_mah.shown && !controls.reserved_mah.enabled);
        assert!(!controls.low_voltage.number.enabled);
        // The combo boxes stay hidden without their parameter.
        assert_eq!(controls.battery.param, "FS_BATT_ENABLE");
        assert!(!controls.battery.shown);
        assert!(controls.pwm().is_none());
    }

    #[test]
    fn a_plane_binds_its_own_names() {
        let controls = Controls::activate(&plane(), bundled);
        assert!(!controls.throttle.shown, "no FS_THR_ENABLE on a plane");
        assert!(!controls.gcs.shown, "no FS_GCS_ENABLE on a plane");
        assert_eq!(controls.pwm().map(|pwm| pwm.param), Some("THR_FS_VALUE"));
        assert_eq!(controls.plane_throttle.state, CheckState::Checked);
        assert_eq!(controls.plane_throttle_action.state, CheckState::Unchecked);
        // FS_GCS_ENABL is 2: neither on nor off.
        assert_eq!(controls.plane_gcs.state, CheckState::Indeterminate);
        assert_eq!(controls.plane_short.state, CheckState::Unchecked);
        assert_eq!(controls.plane_long.state, CheckState::Checked);
        assert!(controls.plane_throttle_pwm.enabled);
    }

    #[test]
    fn the_throttle_options_are_the_documentations_and_the_value_selects_one() {
        let controls = Controls::activate(&copter(), bundled);
        let throttle = &controls.throttle;
        assert_eq!(throttle.options, options("FS_THR_ENABLE", bundled));
        assert!(throttle.options.contains(&(0, "Disabled".to_owned())));
        assert!(
            throttle
                .options
                .contains(&(3, "Enabled always Land".to_owned()))
        );
        assert_eq!(throttle.selected, Some(1));
        assert_eq!(throttle.text(), "Enabled always RTL");

        let battery = &controls.battery;
        assert!(battery.options.contains(&(2, "RTL".to_owned())));
        assert_eq!(battery.selected, Some(0));
        assert_eq!(battery.text(), "None");
    }

    #[test]
    fn a_value_that_is_not_an_option_selects_nothing() {
        // SelectedValue = 42 finds no row, so SelectedIndex is -1 and the box is blank.
        let controls = Controls::activate(&table(&[("FS_THR_ENABLE", 42.0)]), bundled);
        assert!(controls.throttle.shown);
        assert_eq!(controls.throttle.selected, None);
        assert_eq!(controls.throttle.text(), "");
    }

    #[test]
    fn the_selected_value_is_the_truncated_parameter() {
        // `(int)paramlist[paramname].Value`.
        let controls = Controls::activate(&table(&[("FS_THR_ENABLE", 3.9)]), bundled);
        assert_eq!(controls.throttle.selected, Some(3));
    }

    #[test]
    fn a_parameter_with_no_documented_options_has_an_empty_list() {
        fn nothing(_: &str) -> Option<&'static ParamMeta> {
            None
        }
        let controls = Controls::activate(&copter(), nothing);
        assert!(controls.throttle.shown);
        assert!(controls.throttle.options.is_empty());
        assert_eq!(controls.throttle.selected, None);
    }

    #[test]
    fn choosing_a_different_option_writes_it_and_the_same_one_does_not() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        failsafe.toggle_dropdown(ComboId::Throttle);
        assert_eq!(failsafe.dropdown(), Some(ComboId::Throttle));
        let write = failsafe.choose(ComboId::Throttle, 3).expect("a change");
        assert_eq!(failsafe.dropdown(), None, "choosing closes the list");
        assert_eq!(write.param, "FS_THR_ENABLE");
        assert!((write.value - 3.0).abs() < f64::EPSILON);
        assert_eq!(write.failure, "Set FS_THR_ENABLE Failed!");
        assert_eq!(
            failsafe.controls().map(|controls| controls.throttle.text()),
            Some("Enabled always Land")
        );
        assert!(
            failsafe.choose(ComboId::Throttle, 3).is_none(),
            "no SelectedIndexChanged for the same row"
        );
        assert!(
            failsafe.choose(ComboId::Throttle, 99).is_none(),
            "not an option"
        );
    }

    #[test]
    fn a_check_box_writes_on_every_click_and_indeterminate_goes_to_off() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&plane(), bundled);
        // FS_GCS_ENABL = 2: indeterminate, and a click unchecks it and writes OffValue.
        let write = failsafe.click(CheckId::PlaneGcs).expect("a write");
        assert_eq!(write.param, "FS_GCS_ENABL");
        assert!(write.value.abs() < f64::EPSILON);
        assert_eq!(write.failure, "Set FS_GCS_ENABL Failed");
        let write = failsafe.click(CheckId::PlaneGcs).expect("a write");
        assert!((write.value - 1.0).abs() < f64::EPSILON);
        assert_eq!(
            failsafe
                .controls()
                .map(|controls| controls.check(CheckId::PlaneGcs).state),
            Some(CheckState::Checked)
        );
        // A hidden box cannot be clicked.
        assert!(failsafe.click(CheckId::Gcs).is_none());
    }

    #[test]
    fn the_gcs_check_box_reads_fs_gcs_enable() {
        let controls = Controls::activate(&copter(), bundled);
        assert_eq!(controls.gcs.state, CheckState::Unchecked);
        let controls = Controls::activate(&old_copter(), bundled);
        assert_eq!(controls.gcs.state, CheckState::Checked);
        let controls = Controls::activate(&table(&[("FS_GCS_ENABLE", 5.0)]), bundled);
        assert_eq!(controls.gcs.state, CheckState::Indeterminate);
    }

    #[test]
    fn unchecking_thr_failsafe_disables_thr_fs_value() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&plane(), bundled);
        failsafe.click(CheckId::PlaneThrottle);
        let enabled = |failsafe: &FailSafe| {
            failsafe
                .controls()
                .is_some_and(|controls| controls.plane_throttle_pwm.enabled)
        };
        assert!(!enabled(&failsafe));
        assert!(
            !failsafe.step(NumberId::PlaneThrottlePwm, true, Instant::now()),
            "a disabled number does not step"
        );
        failsafe.click(CheckId::PlaneThrottle);
        assert!(enabled(&failsafe));
    }

    #[test]
    fn decimals_are_the_seven_digit_decimals_of_a_float() {
        assert_eq!(decimal_of(0.1), (1, 1));
        assert_eq!(decimal_of(1.0), (1, 0));
        assert_eq!(decimal_of(50.0), (50, 0));
        assert_eq!(decimal_of(10.5), (105, 1));
        // 3.3 is 3.2999999523 as a float, and seven digits of it are 3.3.
        assert_eq!(decimal_of(3.3), (33, 1));
        assert_eq!(decimal_of(975.0), (975, 0));
        assert_eq!(decimal_of(-2.5), (-25, 1));
        assert_eq!(decimal_of(0.0), (0, 0));
        assert_eq!(decimal_of(0.05), (5, 2));
        assert_eq!(decimal_of(99999.0), (99999, 0));
    }

    #[test]
    fn fs_thr_value_takes_the_documented_range_and_step() {
        let controls = Controls::activate(&copter(), bundled);
        let pwm = &controls.throttle_pwm;
        // The documentation says 910 to 1100 in steps of 1, over setup's 800 to 1200.
        assert!((pwm.minimum - 910.0).abs() < f64::EPSILON);
        assert!((pwm.maximum - 1100.0).abs() < f64::EPSILON);
        assert_eq!(pwm.decimals, 0);
        assert_eq!(pwm.text(), "975");
        assert!(pwm.shown && pwm.enabled);
    }

    #[test]
    fn the_low_voltage_shows_one_place_and_steps_by_a_tenth() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        let number = |failsafe: &FailSafe| {
            failsafe
                .controls()
                .map(|controls| controls.number(NumberId::LowVoltage).clone())
                .expect("open")
        };
        assert_eq!(number(&failsafe).text(), "10.5");
        assert_eq!(number(&failsafe).decimals, 1);
        let start = Instant::now();
        assert!(failsafe.step(NumberId::LowVoltage, true, start));
        assert_eq!(number(&failsafe).text(), "10.6");
        // The write waits for the timer, and goes as a float.
        assert!(failsafe.due(start).is_empty());
        let writes = failsafe.due(start + WRITE_DELAY);
        assert_eq!(writes.len(), 1);
        assert_eq!(writes[0].param, "BATT_LOW_VOLT");
        assert!((writes[0].value - f64::from(10.6_f32)).abs() < f64::EPSILON);
        assert!(
            failsafe.due(start + WRITE_DELAY * 2).is_empty(),
            "written once"
        );
    }

    #[test]
    fn a_documented_step_larger_than_the_designer_places_is_taken() {
        // BATT_LOW_MAH documents 50, which is more than the designer's one decimal place.
        let controls = Controls::activate(&copter(), bundled);
        let mah = &controls.reserved_mah.number;
        assert_eq!(mah.decimals, 0);
        assert_eq!(mah.text(), "0");
        let mut mah = mah.clone();
        assert!(mah.step(true, Instant::now()));
        assert_eq!(mah.text(), "50");
        // And the voltage's 0.1 is not more than one decimal place, so setup's 0.1 stands.
        assert_eq!(controls.low_voltage.number.increment, 1);
    }

    #[test]
    fn a_value_outside_the_range_widens_it() {
        // FS_THR_VALUE 850 is under the documented 910.
        let controls = Controls::activate(&table(&[("FS_THR_VALUE", 850.0)]), bundled);
        assert!((controls.throttle_pwm.minimum - 850.0).abs() < f64::EPSILON);
        assert_eq!(controls.throttle_pwm.text(), "850");
    }

    #[test]
    fn stepping_stops_at_the_bounds() {
        let controls = Controls::activate(&table(&[("FS_THR_VALUE", 1100.0)]), bundled);
        let mut pwm = controls.throttle_pwm;
        assert!(!pwm.step(true, Instant::now()), "already at the maximum");
        assert!(pwm.step(false, Instant::now()));
        assert_eq!(pwm.text(), "1099");
    }

    #[test]
    fn a_value_with_more_places_than_the_step_shows_them() {
        // 10.25 has two places; the box shows both, and steps by 0.1 in them.
        let mut number = Controls::activate(&table(&[("BATT_LOW_VOLT", 10.25)]), bundled)
            .low_voltage
            .number;
        assert_eq!(number.decimals, 2);
        assert_eq!(number.text(), "10.25");
        assert!(number.step(true, Instant::now()));
        assert_eq!(number.text(), "10.35");
    }

    #[test]
    fn closing_writes_what_the_timers_still_held() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        failsafe.step(NumberId::ThrottlePwm, true, Instant::now());
        let pending = failsafe.close();
        assert_eq!(pending.len(), 1);
        assert_eq!(pending[0].param, "FS_THR_VALUE");
        assert!((pending[0].value - 976.0).abs() < f64::EPSILON);
        assert!(!failsafe.is_open());
    }

    #[test]
    fn opening_warns_about_the_props_every_time() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        assert_eq!(
            failsafe.message().map(|message| message.text.as_str()),
            Some(PROPS_WARNING)
        );
        assert_eq!(
            failsafe.message().map(|message| message.title),
            Some("FailSafe")
        );
        failsafe.dismiss_message();
        assert!(failsafe.message().is_none());
        failsafe.close();
        failsafe.open(&copter(), bundled);
        assert!(failsafe.message().is_some());
    }

    #[test]
    fn a_write_with_no_vehicle_fails_with_the_csharp_message() {
        let telemetry = Telemetry::idle();
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        failsafe.dismiss_message();
        let write = failsafe.choose(ComboId::Throttle, 3).expect("a change");
        failsafe.issue(&telemetry, write);
        let message = failsafe.message().expect("a message box");
        assert_eq!(message.title, "Error");
        assert_eq!(message.text, "Set FS_THR_ENABLE Failed!");
        assert!(
            failsafe
                .last_write()
                .is_some_and(|text| text.contains("failed"))
        );
    }

    #[test]
    fn leaving_the_setup_screen_closes_the_page() {
        let telemetry = Telemetry::idle();
        let view = telemetry.view();
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        failsafe.tick(&telemetry, &view, true);
        assert!(failsafe.is_open());
        failsafe.tick(&telemetry, &view, false);
        assert!(!failsafe.is_open());
    }

    #[test]
    fn the_mode_turns_red_on_a_change_with_the_throttle_below_fs_thr_value() {
        let mut failsafe = FailSafe::default();
        assert_eq!(failsafe.mode(), "Manual");
        assert_eq!(failsafe.mode_colour(), ModeColour::Default);
        failsafe.observe_mode("Stabilize", 900, Some(975.0));
        assert_eq!(failsafe.mode(), "Stabilize");
        assert_eq!(failsafe.mode_colour(), ModeColour::Red);
        // The colour is decided on a change only: the throttle coming up does not repaint it.
        failsafe.observe_mode("Stabilize", 1500, Some(975.0));
        assert_eq!(failsafe.mode_colour(), ModeColour::Red);
        failsafe.observe_mode("AltHold", 1500, Some(975.0));
        assert_eq!(failsafe.mode_colour(), ModeColour::White);
        // Without FS_THR_VALUE the colour is left as it was.
        failsafe.observe_mode("Loiter", 900, None);
        assert_eq!(failsafe.mode_colour(), ModeColour::White);
    }

    #[test]
    fn a_mode_that_reads_as_the_designer_text_is_no_change() {
        let mut failsafe = FailSafe::default();
        failsafe.observe_mode("Manual", 900, Some(975.0));
        assert_eq!(failsafe.mode_colour(), ModeColour::Default);
    }

    #[test]
    fn the_readouts_say_what_the_csharp_paints() {
        assert_eq!(armed_text(true), "Armed");
        assert_eq!(armed_text(false), "Disarmed");
        assert_eq!(gps_text(0), "GPS: No GPS");
        assert_eq!(gps_text(1), "GPS: No Fix");
        assert_eq!(gps_text(2), "GPS: 3D Fix");
        assert_eq!(gps_text(3), "GPS: 3D Fix");
        assert_eq!(gps_text(4), "");
        assert_eq!(gps_text(6), "");
    }

    #[test]
    fn a_bar_is_held_one_above_its_minimum_and_at_its_maximum() {
        assert!((bar_fraction(0) - 0.001).abs() < 1e-6);
        assert!((bar_fraction(1000) - 0.001).abs() < 1e-6);
        assert!((bar_fraction(1500) - 0.5).abs() < 1e-6);
        assert!((bar_fraction(2000) - 1.0).abs() < 1e-6);
        assert!((bar_fraction(65535) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn the_radio_bars_read_rc_channels_and_zero_before_it() {
        let mut state = VehicleState::default();
        assert_eq!(radio_in(&state, 3), 0, "ch3in is 0 until RC_CHANNELS");
        state.rc.reported = true;
        if let Some(slot) = state.rc.values.get_mut(2) {
            *slot = 1000;
        }
        assert_eq!(radio_in(&state, 3), 1000);
        // An absent channel is what the wire says, as ch9in is in the C#.
        assert_eq!(radio_in(&state, 9), i64::from(mp_vehicle::rc::UNAVAILABLE));
        assert_eq!(radio_in(&state, 0), 0);
        if let Some(slot) = state.servo_outputs.get_mut(0) {
            *slot = 1100;
        }
        assert_eq!(servo_out(&state, 1), 1100);
        assert_eq!(servo_out(&state, 8), 0);
    }

    #[test]
    fn the_page_is_listed_once_every_parameter_is_in() {
        let mut view = TelemetryView::disconnected("tcp:127.0.0.1:5760");
        assert!(!available(&view));
        view.connected = true;
        view.vehicle = Some(mp_vehicle::VehicleId::new(1, 1));
        assert!(!available(&view), "no parameters yet");
        view.parameters = copter();
        view.parameters_expected = 1400;
        assert!(!available(&view), "7 of 1400 is not all of them");
        view.parameters_expected = 7;
        assert!(available(&view));
    }

    #[test]
    fn the_wiki_link_follows_the_firmware() {
        assert!(wiki_url(Some(VehicleFamily::Copter)).contains("/copter/"));
        assert!(wiki_url(Some(VehicleFamily::Plane)).contains("/plane/"));
        assert!(wiki_url(None).contains("/plane/"));
    }
}
