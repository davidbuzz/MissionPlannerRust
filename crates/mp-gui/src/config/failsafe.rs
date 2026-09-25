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
//! The numbers are the shared `MavlinkNumericUpDown` ([`Number`]): typed into or stepped by their
//! arrows, the typed text read when the box is left, on Enter and before an arrow, and a typed
//! value above the maximum asking "Out of range" first (`Controls/MavlinkNumericUpDown.cs:133-161`).
//! The `.resx`'s tooltips are on the controls that have one.
//!
//! The geometry is the `.resx`'s: every control at its `Location` and `Size` inside a 688 x 482
//! page, in its group box. The colours are this application's.
//!
//! A write the vehicle does not take - `Strings.ErrorSetValueFailed` in an error box in the C# -
//! goes on the status line instead, by the owner's ruling of 2026-09-25 that an error the window
//! can show as state gets no message box; the props warning `Activate` shows and the out-of-range
//! question keep their boxes.
//!
//! What is not ported, and why:
//!
//! * `mavlinkCheckBoxfs_gps_enable`, commented out in the C# "at randys request"
//!   (`ConfigFailSafe.cs:75-76`), and the `FS_SHORT_TIMEOUT`/`FS_LONG_TIMEOUT` a plane has, which
//!   the page never binds;
//! * the numbers' mouse wheel, as on every page that uses the shared control.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::time::Instant;

use gpui::{
    AnyElement, AnyView, Context, Div, FocusHandle, KeyDownEvent, SharedString, Window, div,
    prelude::*, px, rgb,
};
use mp_params::ParamMeta;
use mp_vehicle::{VehicleFamily, VehicleState};

use crate::MissionPlanner;
use crate::config::basic_tuning::Tip;
use crate::config::extra_setup::{link_error, status_words};
use crate::config::servo_output::{
    Designer, ERROR_TITLE, NUMERIC_DEFAULTS, Number, NumberHandlers, OUT_OF_RANGE_TITLE, Question,
    Setup, Writes, modal, number_box,
};
pub use crate::config::servo_output::{Message, Write};
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

/// The Battery box's three numbers as the Designer leaves them: `DecimalPlaces = 1`, `Value` 13.1
/// (`new decimal(131, 0, 0, 65536)`), and `NumericUpDown`'s own `Minimum` 0 and `Maximum` 100 -
/// the Designer's `Min` and `Max` are the control's own properties, which `setup` replaces. What
/// a box whose parameter the vehicle lacks goes on showing.
/// `// C#: GCSViews/ConfigurationView/ConfigFailSafe.Designer.cs:133-151, 261-277, 603-619`
const BATTERY_BOX: Designer = Designer {
    minimum: 0.0,
    maximum: 100.0,
    value: 13.1,
    decimals: 1,
};

/// The `.resx`'s tooltips (`toolTip1`), by control.
/// `// C#: GCSViews/ConfigurationView/ConfigFailSafe.resx:267-268, 303-304, 342-343, 381-382,
/// 420-422, 454-455, 487-488, 526-527, 559-560`
pub mod tips {
    /// `mavlinkNumericUpDownlow_voltage`.
    pub const LOW_VOLTAGE: &str = "Low Voltage Trigger";
    /// `mavlinkCheckBoxlong_fs` and `mavlinkCheckBoxshort_fs`.
    pub const SHORT_LONG: &str = "Off, no Action, On, RTL";
    /// `mavlinkCheckBoxgcs_fs`.
    pub const PLANE_GCS: &str = "Enable Failsafe on GCS loss of communication";
    /// `mavlinkCheckBoxthr_fs_action`.
    pub const THROTTLE_ACTION: &str = "Arducopter Auto: Off, no Action, On, RTL\nArducopter Other: \
                                       if have gps, RTL, Otherwise Land";
    /// `mavlinkNumericUpDownfs_thr_value` and `mavlinkNumericUpDownthr_fs_value`.
    pub const PWM: &str = "Trigger Throttle Pwm";
    /// `mavlinkCheckBoxthr_fs`.
    pub const THROTTLE: &str = "Enable Failsafe on low throttle pwm";
    /// `mavlinkComboBox_fs_thr_enable`.
    pub const THROTTLE_MODE: &str = "Failsafe mode on low pwm";
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

/// One of the page's `MavlinkNumericUpDown`s: the shared control, which types, steps, asks
/// "Out of range" and writes on its timer, and its `Visible` - which the `.resx` sets false for
/// the two throttle numbers until `setup` finds their parameter.
/// `// C#: Controls/MavlinkNumericUpDown.cs:43-181`
#[derive(Debug)]
pub struct Numeric {
    /// The control.
    pub number: Number,
    /// Whether the vehicle has the parameter, so `setup` bound the box to it.
    pub bound: bool,
    /// `Visible`.
    pub shown: bool,
}

impl Numeric {
    /// The box as the Designer leaves it, named for its parameter but never set up: what
    /// `BATT_LOW_TIMER`'s box is on a vehicle without it (`ConfigFailSafe.cs:70-73`).
    fn designer(designer: Designer, designer_shown: bool, param: &'static str) -> Self {
        let mut number = Number::new(designer);
        param.clone_into(&mut number.param);
        Self {
            number,
            bound: false,
            shown: designer_shown,
        }
    }

    /// `setup(Min, Max, Scale, Increment, paramname, paramlist)` on the box the Designer made. A
    /// parameter the vehicle has makes it visible and enabled; one it lacks leaves `Visible` as
    /// the `.resx` has it and disables it.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:49-122`
    fn setup(
        designer: Designer,
        designer_shown: bool,
        how: Setup,
        param: &'static str,
        parameters: &[(String, f64)],
        lookup: Lookup,
    ) -> Self {
        let mut numeric = Self::designer(designer, designer_shown, param);
        numeric.number.setup(how, param, parameters, lookup);
        numeric.bound = value_of(parameters, param).is_some();
        numeric.shown = numeric.bound || designer_shown;
        numeric
    }

    /// `ParamName`.
    #[must_use]
    pub fn param(&self) -> &str {
        &self.number.param
    }

    /// `Value` as the box shows it.
    #[must_use]
    pub fn text(&self) -> String {
        self.number.text()
    }

    /// `Enabled`.
    #[must_use]
    pub const fn enabled(&self) -> bool {
        self.number.enabled
    }
}

/// One of the Battery box's labelled rows - `PNL_low_bat`, `pnlmah`, `pnltimer` - a panel holding
/// a label and a number. The panel is the number's `enabledisable` control, which `setup` makes
/// visible either way and enables only for a parameter the vehicle has.
/// `// C#: Controls/MavlinkNumericUpDown.cs:124-131`
#[derive(Debug)]
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
            enabled: number.enabled(),
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
#[derive(Debug)]
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
        // `// C#: :43`; the Designer gives the box `NumericUpDown`'s defaults and the `.resx`
        // hides it (`ConfigFailSafe.Designer.cs:193-200`, `.resx:457-458`).
        let throttle_pwm = Numeric::setup(
            NUMERIC_DEFAULTS,
            false,
            Setup {
                minimum: 800.0,
                maximum: 1200.0,
                scale: 1.0,
                increment: 1.0,
            },
            "FS_THR_VALUE",
            parameters,
            lookup,
        );
        // The three generations of the low-voltage name, oldest first, with the newest as the
        // name used when the vehicle has none of them. `// C#: :45-59`
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
                BATTERY_BOX,
                true,
                Setup {
                    minimum: 6.0,
                    maximum: 99.0,
                    scale: 1.0,
                    increment: 0.1,
                },
                low_voltage_name,
                parameters,
                lookup,
            ),
        );
        // `// C#: :61-68`
        let reserved_mah = Row::setup(
            "Reserved MAH",
            Numeric::setup(
                BATTERY_BOX,
                true,
                Setup {
                    minimum: 0.0,
                    maximum: 99999.0,
                    scale: 1.0,
                    increment: 1.0,
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
        // panel and the Designer's box. `// C#: :70-73`
        let low_timer = if has("BATT_LOW_TIMER") {
            Row::setup(
                "Low Timer",
                Numeric::setup(
                    BATTERY_BOX,
                    true,
                    Setup {
                        minimum: 0.0,
                        maximum: 120.0,
                        scale: 1.0,
                        increment: 1.0,
                    },
                    "BATT_LOW_TIMER",
                    parameters,
                    lookup,
                ),
            )
        } else {
            Row {
                label: "Low Timer",
                shown: false,
                enabled: true,
                number: Numeric::designer(BATTERY_BOX, true, "BATT_LOW_TIMER"),
            }
        };
        // `// C#: :77`
        let gcs = Check::setup(1.0, 0.0, "FS_GCS_ENABLE", parameters);

        // Plane. `THR_FAILSAFE` enables `THR_FS_VALUE`'s box when checked, and `THR_FS_VALUE`'s
        // own setup, which comes after, enables it again whenever the vehicle has the parameter.
        // `// C#: :79-85`
        let plane_throttle = Check::setup(1.0, 0.0, "THR_FAILSAFE", parameters);
        let plane_throttle_pwm = Numeric::setup(
            NUMERIC_DEFAULTS,
            false,
            Setup {
                minimum: 800.0,
                maximum: 1200.0,
                scale: 1.0,
                increment: 1.0,
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

    /// Whether a number takes the focus and its arrows: shown, in a panel that is not hidden,
    /// and enabled.
    fn number_live(&self, id: NumberId) -> bool {
        let row_shown = match id {
            NumberId::LowVoltage => self.low_voltage.shown,
            NumberId::ReservedMah => self.reserved_mah.shown,
            NumberId::LowTimer => self.low_timer.shown,
            NumberId::ThrottlePwm | NumberId::PlaneThrottlePwm => true,
        };
        let number = self.number(id);
        row_shown && number.shown && number.enabled()
    }

    /// Every number, for the timers.
    fn numbers_mut(&mut self) -> [&mut Number; 5] {
        [
            &mut self.low_voltage.number.number,
            &mut self.reserved_mah.number.number,
            &mut self.low_timer.number.number,
            &mut self.throttle_pwm.number,
            &mut self.plane_throttle_pwm.number,
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

/// The page, and everything it keeps while it is open.
#[derive(Debug, Default)]
pub struct FailSafe {
    /// The controls, while the page is open.
    controls: Option<Controls>,
    /// The combo box whose list is dropped down.
    dropdown: Option<ComboId>,
    /// The number being typed into: the one with the keyboard focus.
    editing: Option<NumberId>,
    /// A number's "Out of range" question, modal until it is answered.
    question: Option<(NumberId, Question)>,
    /// Message boxes, the first showing. Modal, as `CustomMessageBox.Show` is.
    messages: VecDeque<Message>,
    /// Writes the link is carrying, and how the last ended.
    writes: Writes,
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

    /// The "Out of range" question showing, if one is.
    #[must_use]
    pub fn question(&self) -> Option<&Question> {
        self.question.as_ref().map(|(_, question)| question)
    }

    /// The combo box whose list is down.
    #[must_use]
    pub const fn dropdown(&self) -> Option<ComboId> {
        self.dropdown
    }

    /// The number being typed into.
    #[must_use]
    pub const fn editing(&self) -> Option<NumberId> {
        self.editing
    }

    /// How the last write ended.
    #[must_use]
    pub fn last_write(&self) -> Option<&str> {
        self.writes.last()
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
        self.editing = None;
        self.question = None;
        self.mode = None;
        self.mode_colour = None;
        self.messages.push_back(Message {
            title: PROPS_TITLE,
            text: PROPS_WARNING.to_owned(),
        });
    }

    /// Closes the page, returning what its number timers still had to write: the C#'s timers
    /// outlive the page, so a change made just before leaving still reaches the vehicle. A number
    /// being typed into loses the focus first, which reads its text; a question that raises has
    /// no page left to show over, and is taken as No - the value held to the maximum, which the
    /// handler then writes.
    pub fn close(&mut self, now: Instant) -> Vec<Write> {
        self.leave(now);
        self.answer(false, now);
        let pending = self
            .controls
            .as_mut()
            .map(|controls| {
                controls
                    .numbers_mut()
                    .into_iter()
                    .filter_map(Number::flush)
                    .collect()
            })
            .unwrap_or_default();
        self.controls = None;
        self.dropdown = None;
        self.editing = None;
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
            controls.plane_throttle_pwm.number.enabled =
                controls.plane_throttle.state == CheckState::Checked;
        }
        Some(write)
    }

    /// A number clicked into: the one being typed into before it loses the focus, which reads
    /// its text, and this one takes it - if it is showing and enabled.
    pub fn begin(&mut self, id: NumberId, now: Instant) {
        if self.editing == Some(id) {
            return;
        }
        self.leave(now);
        self.dropdown = None;
        if self
            .controls
            .as_ref()
            .is_some_and(|controls| controls.number_live(id))
        {
            self.editing = Some(id);
        }
    }

    /// The number being typed into loses the focus: `ValidateEditText` reads what was typed.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:133-161`
    pub fn leave(&mut self, now: Instant) {
        let Some(id) = self.editing.take() else {
            return;
        };
        if let Some(question) = self
            .controls
            .as_mut()
            .and_then(|controls| controls.number_mut(id).number.commit(now))
        {
            self.question = Some((id, question));
        }
    }

    /// A key for the number being typed into: the arrow keys step, Enter reads the text, the rest
    /// is typing. Whether it was taken.
    pub fn key(&mut self, event: &KeyDownEvent, now: Instant) -> bool {
        let Some(id) = self.editing else {
            return false;
        };
        let Some(controls) = self.controls.as_mut() else {
            return false;
        };
        let (handled, question) = controls.number_mut(id).number.key(event, now);
        if let Some(question) = question {
            self.question = Some((id, question));
        }
        handled
    }

    /// A number's up or down arrow: the box takes the focus, the typed text is read, and it steps
    /// one `Increment`; the write follows once its timer runs out.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:133-161`
    pub fn step(&mut self, id: NumberId, up: bool, now: Instant) {
        self.begin(id, now);
        let Some(controls) = self.controls.as_mut() else {
            return;
        };
        if !controls.number_live(id) {
            return;
        }
        if let Some(question) = controls.number_mut(id).number.step(up, now) {
            self.question = Some((id, question));
        }
    }

    /// The "Out of range" question answered: Yes takes what was typed as the new maximum and the
    /// value; either way the handler goes on to start the write timer.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:136-160`
    pub fn answer(&mut self, yes: bool, now: Instant) {
        let Some((id, question)) = self.question.take() else {
            return;
        };
        if let Some(controls) = self.controls.as_mut() {
            controls.number_mut(id).number.answer(&question, yes, now);
        }
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
        self.writes.issue(telemetry, write, &mut self.messages);
    }

    /// The C#'s error boxes for writes that failed, taken out of the queue: the status line's
    /// words for the last of them, by the owner's ruling (the module's notes). The props warning
    /// stays a box.
    pub fn take_link_errors(&mut self) -> Option<String> {
        let mut status = None;
        self.messages.retain(|message| {
            if link_error(message) {
                status = Some(status_words(message));
                false
            } else {
                true
            }
        });
        status
    }

    /// Once a frame. Leaving the setup screen closes the page, as leaving Initial Setup disposes
    /// it; a number the focus has left is read, the timers that ran out are written, the writes
    /// in flight read back, and the mode's colour kept as `timer_Tick`'s rebinding would keep it.
    /// Returns the status line's words for a write that failed.
    /// `// C#: GCSViews/ConfigurationView/ConfigFailSafe.cs:95-110`
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        on_setup: bool,
        focused: bool,
    ) -> Option<String> {
        let now = Instant::now();
        if self.editing.is_some() && !focused {
            self.leave(now);
        }
        let mut writes = if on_setup || !self.is_open() {
            Vec::new()
        } else {
            self.close(now)
        };
        writes.extend(self.due(now));
        for write in writes {
            self.issue(telemetry, write);
        }
        self.writes.settle(telemetry, &mut self.messages);
        if self.is_open()
            && let Some(state) = view.state.as_deref()
        {
            let fs_thr_value = value_of(&view.parameters, "FS_THR_VALUE");
            self.observe_mode(&mode_text(state), radio_in(state, 3), fs_thr_value);
        }
        self.take_link_errors()
    }

    /// The page's entry on the setup screen: opens it, or closes it again.
    pub fn toggle(&mut self, telemetry: &Telemetry) {
        if self.is_open() {
            for write in self.close(Instant::now()) {
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
    record("config.failsafe.writes.pending", failsafe.writes.pending());
    record(
        "config.failsafe.question",
        failsafe
            .question()
            .map_or_else(|| "none".to_owned(), Question::text),
    );

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
            shown.map_or("none", Numeric::param),
        );
        record(
            format!("config.failsafe.{key}"),
            shown.map_or_else(|| "none".to_owned(), Numeric::text),
        );
        // What the box shows, typed text included.
        record(
            format!("config.failsafe.{key}.shown"),
            shown.map_or("none", |number| number.number.shown()),
        );
        record(
            format!("config.failsafe.{key}.enabled"),
            shown.is_some_and(Numeric::enabled),
        );
        record(
            format!("config.failsafe.{key}.vehicle"),
            shown.map_or_else(|| "none".to_owned(), |number| vehicle(number.param())),
        );
    };
    record(
        "config.failsafe.editing",
        failsafe
            .editing()
            .and_then(|id| controls.map(|controls| controls.number(id).param().to_owned()))
            .unwrap_or_else(|| "none".to_owned()),
    );
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

/// A `ToolTip` over an element, as `toolTip1.SetToolTip` gives one: shown while it is enabled,
/// as WinForms shows none over a disabled control.
fn with_tip(
    element: gpui::Stateful<Div>,
    tip: Option<&'static str>,
    enabled: bool,
) -> gpui::Stateful<Div> {
    match tip {
        Some(tip) if enabled => element.tooltip(move |_window, cx| -> AnyView {
            cx.new(|_| Tip(SharedString::from(tip))).into()
        }),
        _ => element,
    }
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
    (x, y): (f32, f32),
    tip: Option<&'static str>,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if !combo.shown {
        return None;
    }
    let base = crate::probe::measured(format!("failsafe-{}", combo.param), div())
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
        .child(div().flex_1().child(combo.text().to_owned()))
        .child(div().text_size(px(7.0)).child("▼"))
        .on_click(cx.listener(move |this, _event, _window, cx| {
            this.failsafe.toggle_dropdown(id);
            cx.notify();
        }));
    Some(
        at(x, y, 199.0, 20.0)
            .child(with_tip(base, tip, true))
            .into_any_element(),
    )
}

/// A combo box's list, dropped down over the page: the shared list, drawn deferred and anchored
/// at the box so the page's scrolling area does not clip it. Each list here is a few rows, so it
/// never scrolls.
fn dropdown(
    id: ComboId,
    combo: &Combo,
    (x, y): (f32, f32),
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let shared = crate::config::servo_output::Combo {
        param: combo.param.to_owned(),
        options: combo.options.clone(),
        selected: combo.selected,
        enabled: combo.shown,
        top_index: 0,
    };
    crate::config::servo_output::dropdown(
        &format!("failsafe-{}", combo.param),
        &shared,
        (x, y, 199.0),
        move |this, key| {
            if let Some(write) = this.failsafe.choose(id, key) {
                this.failsafe.issue(&this.telemetry, write);
            }
        },
        |_this, _lines| {},
        cx,
    )
}

/// A check box and its text.
fn check_box(
    id: CheckId,
    check: &Check,
    text: &'static str,
    (x, y): (f32, f32),
    tip: Option<&'static str>,
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
    let base = crate::probe::measured(format!("failsafe-{}", check.param), div())
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
        }));
    Some(
        div()
            .absolute()
            .left(px(x))
            .top(px(y))
            .child(with_tip(base, tip, true))
            .into_any_element(),
    )
}

/// Where a page's number boxes take the keyboard and send their events.
struct Numbers<'a> {
    failsafe: &'a FailSafe,
    focus: &'a FocusHandle,
    window: &'a Window,
}

impl Numbers<'_> {
    /// A number at its place in its parent: the shared box - its text typed into while it has
    /// the focus, its arrows at its right, `failsafe-<param>`, `-up` and `-down` - under its
    /// tooltip.
    fn number(
        &self,
        id: NumberId,
        (x, y, width, height): (f32, f32, f32, f32),
        tip: Option<&'static str>,
        cx: &mut Context<MissionPlanner>,
    ) -> Option<AnyElement> {
        let numeric = self.failsafe.controls()?.number(id);
        if !numeric.shown {
            return None;
        }
        let name = format!("failsafe-{}", numeric.param());
        let wrapper = div()
            .id(SharedString::from(format!("{name}-tip")))
            .absolute()
            .left(px(x))
            .top(px(y))
            .w(px(width))
            .h(px(height));
        let wrapper = with_tip(wrapper, tip, numeric.enabled());
        Some(
            wrapper
                .child(number_box(
                    name,
                    &numeric.number,
                    self.failsafe.editing() == Some(id),
                    self.focus,
                    (0.0, 0.0, width, height),
                    NumberHandlers {
                        begin: move |this: &mut MissionPlanner| {
                            this.failsafe.begin(id, Instant::now());
                        },
                        key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
                            this.failsafe.key(event, Instant::now())
                        },
                        step: move |this: &mut MissionPlanner, up: bool| {
                            this.failsafe.step(id, up, Instant::now());
                        },
                    },
                    self.window,
                    cx,
                ))
                .into_any_element(),
        )
    }

    /// One of the Battery box's rows: its label at (4, 3), and its number at (114, 0), 85 by 19.
    /// `// C#: GCSViews/ConfigurationView/ConfigFailSafe.resx label4-6, PNL_low_bat, pnlmah,
    /// pnltimer and their numbers' Location and Size`
    fn battery_row(
        &self,
        id: NumberId,
        row: &Row,
        y: f32,
        tip: Option<&'static str>,
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
                .children(self.number(id, (114.0, 0.0, 85.0, 19.0), tip, cx))
                .into_any_element(),
        )
    }
}

/// The page, laid out as `ConfigFailSafe.resx` lays it out.
pub fn page(
    failsafe: &FailSafe,
    focus: &FocusHandle,
    view: &TelemetryView,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let Some(controls) = failsafe.controls() else {
        return div().into_any_element();
    };
    let numbers = Numbers {
        failsafe,
        focus,
        window,
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

    // groupBox4, "Battery", its controls in the Designer's order: `pnltimer`,
    // `mavlinkComboBoxfs_batt_enable`, `PNL_low_bat`, `pnlmah`.
    // `// C#: GCSViews/ConfigurationView/ConfigFailSafe.Designer.cs:583-591`
    let battery = group(477.0, 146.0, 208.0, 121.0, "Battery")
        .children(numbers.battery_row(NumberId::LowTimer, &controls.low_timer, 68.0, None, cx))
        .children(combo_box(
            ComboId::Battery,
            &controls.battery,
            (6.0, 92.0),
            None,
            cx,
        ))
        .children(numbers.battery_row(
            NumberId::LowVoltage,
            &controls.low_voltage,
            18.0,
            Some(tips::LOW_VOLTAGE),
            cx,
        ))
        .children(numbers.battery_row(
            NumberId::ReservedMah,
            &controls.reserved_mah,
            43.0,
            None,
            cx,
        ));

    // groupBox2, "Radio". `PNL_thr_fs_value` (6, 35, 199 x 23) docks `label3` left and each
    // number right: `FS_THR_VALUE`'s 93 wide, `THR_FS_VALUE`'s 103; only one shows.
    // `// C#: GCSViews/ConfigurationView/ConfigFailSafe.Designer.cs:292-305, 561-569; .resx:439-449, 472-482, 779-812`
    let pwm = at(6.0, 35.0, 199.0, 23.0)
        .child(
            div()
                .absolute()
                .left(px(0.0))
                .top(px(0.0))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child("FS Pwm"),
        )
        .children(numbers.number(
            NumberId::ThrottlePwm,
            (199.0 - 93.0, 0.0, 93.0, 19.0),
            Some(tips::PWM),
            cx,
        ))
        .children(numbers.number(
            NumberId::PlaneThrottlePwm,
            (199.0 - 103.0, 0.0, 103.0, 19.0),
            Some(tips::PWM),
            cx,
        ));
    let radio = group(477.0, 273.0, 208.0, 96.0, "Radio")
        .children(combo_box(
            ComboId::Throttle,
            &controls.throttle,
            (6.0, 13.0),
            Some(tips::THROTTLE_MODE),
            cx,
        ))
        .child(pwm)
        .children(check_box(
            CheckId::PlaneThrottle,
            controls.check(CheckId::PlaneThrottle),
            "Throttle FailSafe",
            (6.0, 60.0),
            Some(tips::THROTTLE),
            cx,
        ))
        .children(check_box(
            CheckId::PlaneThrottleAction,
            controls.check(CheckId::PlaneThrottleAction),
            "Throttle Failsafe Action",
            (6.0, 77.0),
            Some(tips::THROTTLE_ACTION),
            cx,
        ));

    // groupBox3, "GCS".
    let gcs = group(477.0, 375.0, 208.0, 73.0, "GCS")
        .children(check_box(
            CheckId::Gcs,
            controls.check(CheckId::Gcs),
            "GCS FS Enable",
            (6.0, 14.0),
            None,
            cx,
        ))
        .children(check_box(
            CheckId::PlaneGcs,
            controls.check(CheckId::PlaneGcs),
            "GCS FailSafe",
            (6.0, 29.0),
            Some(tips::PLANE_GCS),
            cx,
        ))
        .children(check_box(
            CheckId::PlaneShort,
            controls.check(CheckId::PlaneShort),
            "FailSafe Short (1 sec)",
            (6.0, 43.0),
            Some(tips::SHORT_LONG),
            cx,
        ))
        .children(check_box(
            CheckId::PlaneLong,
            controls.check(CheckId::PlaneLong),
            "FailSafe Long (20 sec)",
            (6.0, 58.0),
            Some(tips::SHORT_LONG),
            cx,
        ));

    body = body.child(battery).child(radio).child(gcs);

    // A dropped-down list, anchored under its box.
    if let Some(id) = failsafe.dropdown() {
        let combo = controls.combo(id);
        let at = match id {
            ComboId::Battery => (477.0 + 6.0, 146.0 + 92.0 + 20.0),
            ComboId::Throttle => (477.0 + 6.0, 273.0 + 13.0 + 20.0),
        };
        if combo.shown {
            body = body.child(dropdown(id, combo, at, cx));
        }
    }

    panel("FailSafe", body).into_any_element()
}

/// The question or message box showing, drawn over the whole window as `CustomMessageBox.Show`
/// is modal: "Out of range" first, as it is asked over whatever else is showing.
pub fn overlay(
    failsafe: &FailSafe,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(question) = failsafe.question() {
        let buttons = vec![
            action(
                "failsafe-question-yes",
                "Yes",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.failsafe.answer(true, Instant::now());
                    cx.notify();
                }),
            ),
            action(
                "failsafe-question-no",
                "No",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.failsafe.answer(false, Instant::now());
                    cx.notify();
                }),
            ),
        ];
        return Some(modal(
            "failsafe-question",
            OUT_OF_RANGE_TITLE,
            &question.text(),
            false,
            buttons,
            window,
        ));
    }
    let message = failsafe.message()?;
    let ok = action(
        "failsafe-message-ok",
        "OK",
        theme::ACCENT,
        true,
        cx.listener(|this, _event: &(), _window, cx| {
            this.failsafe.dismiss_message();
            cx.notify();
        }),
    );
    Some(modal(
        "failsafe-message",
        message.title,
        &message.text,
        message.title == ERROR_TITLE,
        vec![ok],
        window,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::servo_output::WRITE_DELAY;

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
        assert_eq!(controls.low_voltage.number.param(), "BATT_LOW_VOLT");
        assert_eq!(controls.reserved_mah.number.param(), "BATT_LOW_MAH");
        assert_eq!(controls.low_timer.number.param(), "BATT_LOW_TIMER");
        assert!(controls.low_timer.shown, "BATT_LOW_TIMER's row shows");
        assert_eq!(controls.gcs.param, "FS_GCS_ENABLE");
        assert!(controls.gcs.shown);
        assert_eq!(controls.pwm().map(Numeric::param), Some("FS_THR_VALUE"));
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

    /// The whole of a real SITL copter's parameters, as `headless-planner param save` wrote them: the page
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
        assert_eq!(controls.low_voltage.number.param(), "BATT_LOW_VOLT");
        assert_eq!(controls.low_voltage.number.text(), "10.5");
        assert_eq!(controls.reserved_mah.number.param(), "BATT_LOW_MAH");
        assert!(controls.low_timer.shown);
        assert_eq!(controls.low_timer.number.text(), "10");
        assert_eq!(controls.pwm().map(Numeric::text), Some("975".to_owned()));
        assert_eq!(controls.gcs.state, CheckState::Unchecked);
    }

    #[test]
    fn an_older_copter_binds_the_first_names() {
        let controls = Controls::activate(&old_copter(), bundled);
        assert_eq!(controls.battery.param, "FS_BATT_ENABLE");
        assert_eq!(controls.low_voltage.number.param(), "FS_BATT_VOLTAGE");
        assert_eq!(controls.reserved_mah.number.param(), "FS_BATT_MAH");
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
        assert_eq!(controls.low_voltage.number.param(), "LOW_VOLT");
        assert_eq!(controls.low_voltage.number.text(), "9.9");
    }

    #[test]
    fn with_no_battery_names_the_rows_show_disabled_under_the_newest_names() {
        // MavlinkNumericUpDown.enableControl(false) makes the panel visible and disabled.
        let controls = Controls::activate(&table(&[("FS_THR_ENABLE", 1.0)]), bundled);
        assert_eq!(controls.low_voltage.number.param(), "BATT_LOW_VOLT");
        assert_eq!(controls.reserved_mah.number.param(), "BATT_LOW_MAH");
        assert!(controls.low_voltage.shown && !controls.low_voltage.enabled);
        assert!(controls.reserved_mah.shown && !controls.reserved_mah.enabled);
        assert!(!controls.low_voltage.number.enabled());
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
        assert_eq!(controls.pwm().map(Numeric::param), Some("THR_FS_VALUE"));
        assert_eq!(controls.plane_throttle.state, CheckState::Checked);
        assert_eq!(controls.plane_throttle_action.state, CheckState::Unchecked);
        // FS_GCS_ENABL is 2: neither on nor off.
        assert_eq!(controls.plane_gcs.state, CheckState::Indeterminate);
        assert_eq!(controls.plane_short.state, CheckState::Unchecked);
        assert_eq!(controls.plane_long.state, CheckState::Checked);
        assert!(controls.plane_throttle_pwm.enabled());
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
                .is_some_and(|controls| controls.plane_throttle_pwm.enabled())
        };
        let text = |failsafe: &FailSafe| {
            failsafe
                .controls()
                .map(|controls| controls.plane_throttle_pwm.text())
        };
        assert!(!enabled(&failsafe));
        failsafe.step(NumberId::PlaneThrottlePwm, true, Instant::now());
        assert_eq!(
            text(&failsafe).as_deref(),
            Some("950"),
            "a disabled number does not step"
        );
        failsafe.begin(NumberId::PlaneThrottlePwm, Instant::now());
        assert_eq!(failsafe.editing(), None, "nor takes the focus");
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
        assert!((pwm.number.minimum - 910.0).abs() < f64::EPSILON);
        assert!((pwm.number.maximum - 1100.0).abs() < f64::EPSILON);
        assert_eq!(pwm.number.decimals, 0);
        assert_eq!(pwm.text(), "975");
        assert!(pwm.shown && pwm.enabled());
    }

    #[test]
    fn the_low_voltage_shows_one_place_and_steps_by_a_tenth() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        let text = |failsafe: &FailSafe| {
            failsafe
                .controls()
                .map(|controls| controls.number(NumberId::LowVoltage).text())
                .expect("open")
        };
        assert_eq!(text(&failsafe), "10.5");
        let start = Instant::now();
        failsafe.step(NumberId::LowVoltage, true, start);
        assert_eq!(text(&failsafe), "10.6");
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
        let mut controls = Controls::activate(&copter(), bundled);
        let mah = &mut controls.reserved_mah.number.number;
        assert_eq!(mah.decimals, 0);
        assert_eq!(mah.text(), "0");
        assert!(mah.step(true, Instant::now()).is_none());
        assert_eq!(mah.text(), "50");
        // And the voltage's 0.1 is not more than one decimal place, so setup's 0.1 stands.
        assert!((controls.low_voltage.number.number.increment() - 0.1).abs() < 1e-9);
    }

    #[test]
    fn a_value_outside_the_range_widens_it() {
        // FS_THR_VALUE 850 is under the documented 910.
        let controls = Controls::activate(&table(&[("FS_THR_VALUE", 850.0)]), bundled);
        assert!((controls.throttle_pwm.number.minimum - 850.0).abs() < f64::EPSILON);
        assert_eq!(controls.throttle_pwm.text(), "850");
    }

    #[test]
    fn stepping_stops_at_the_bounds() {
        let controls = Controls::activate(&table(&[("FS_THR_VALUE", 1100.0)]), bundled);
        let mut pwm = controls.throttle_pwm.number;
        pwm.step(true, Instant::now());
        assert_eq!(pwm.text(), "1100", "already at the maximum");
        pwm.step(false, Instant::now());
        assert_eq!(pwm.text(), "1099");
    }

    #[test]
    fn a_value_with_more_places_than_the_step_shows_them() {
        // 10.25 has two places; the box shows both, and steps by 0.1 in them.
        let mut number = Controls::activate(&table(&[("BATT_LOW_VOLT", 10.25)]), bundled)
            .low_voltage
            .number
            .number;
        assert_eq!(number.decimals, 2);
        assert_eq!(number.text(), "10.25");
        number.step(true, Instant::now());
        assert_eq!(number.text(), "10.35");
    }

    #[test]
    fn closing_writes_what_the_timers_still_held() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        failsafe.step(NumberId::ThrottlePwm, true, Instant::now());
        let pending = failsafe.close(Instant::now());
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
        failsafe.close(Instant::now());
        failsafe.open(&copter(), bundled);
        assert!(failsafe.message().is_some());
    }

    /// A write that fails is the C#'s "Set FS_THR_ENABLE Failed!", on the status line rather than
    /// in a box (the owner's ruling); the props warning keeps its box.
    #[test]
    fn a_write_with_no_vehicle_fails_on_the_status_line() {
        let telemetry = Telemetry::idle();
        let view = telemetry.view();
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        let write = failsafe.choose(ComboId::Throttle, 3).expect("a change");
        failsafe.issue(&telemetry, write);
        assert_eq!(
            failsafe.tick(&telemetry, &view, true, false).as_deref(),
            Some("Set FS_THR_ENABLE Failed!")
        );
        assert_eq!(
            failsafe.message().map(|message| message.text.as_str()),
            Some(PROPS_WARNING),
            "no error box, and the warning is left showing"
        );
        failsafe.dismiss_message();
        assert!(failsafe.message().is_none());
        assert!(
            failsafe
                .last_write()
                .is_some_and(|text| text.contains("failed"))
        );
        // A number's write fails with `Strings.ErrorSetValueFailed`.
        failsafe.step(NumberId::ThrottlePwm, true, Instant::now());
        for write in failsafe.close(Instant::now()) {
            failsafe.issue(&telemetry, write);
        }
        assert_eq!(
            failsafe.take_link_errors().as_deref(),
            Some("Set FS_THR_VALUE Failed")
        );
    }

    #[test]
    fn leaving_the_setup_screen_closes_the_page() {
        let telemetry = Telemetry::idle();
        let view = telemetry.view();
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        failsafe.tick(&telemetry, &view, true, false);
        assert!(failsafe.is_open());
        failsafe.tick(&telemetry, &view, false, false);
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
        view.parameters = copter().into();
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

    /// Types into a number as the keyboard would: the box takes the focus, and its text is
    /// replaced.
    fn type_into(failsafe: &mut FailSafe, id: NumberId, text: &str) {
        failsafe.begin(id, Instant::now());
        assert_eq!(failsafe.editing(), Some(id), "the box takes the focus");
        let controls = failsafe.controls.as_mut().expect("open");
        controls.number_mut(id).number.type_text(text);
    }

    fn shown(failsafe: &FailSafe, id: NumberId) -> String {
        failsafe
            .controls()
            .map(|controls| controls.number(id).text())
            .expect("open")
    }

    /// A typed low voltage is read when the box is left and written 300 ms later, as a float -
    /// `BATT_LOW_VOLT` read and written on a current copter.
    #[test]
    fn a_typed_low_voltage_is_written_when_the_box_is_left() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        type_into(&mut failsafe, NumberId::LowVoltage, "11.2");
        assert_eq!(
            shown(&failsafe, NumberId::LowVoltage),
            "10.5",
            "not read yet"
        );
        let start = Instant::now();
        failsafe.leave(start);
        assert_eq!(failsafe.editing(), None);
        assert_eq!(shown(&failsafe, NumberId::LowVoltage), "11.2");
        assert!(failsafe.question().is_none());
        assert!(failsafe.due(start).is_empty(), "the timer runs first");
        let writes = failsafe.due(start + WRITE_DELAY);
        assert_eq!(writes.len(), 1);
        assert_eq!(writes[0].param, "BATT_LOW_VOLT");
        assert!((writes[0].value - f64::from(11.2_f32)).abs() < f64::EPSILON);
        assert_eq!(writes[0].failure, "Set BATT_LOW_VOLT Failed");
    }

    /// The pwm typed on an older copter, `FS_THR_VALUE`, read by an arrow: the arrow reads the
    /// text first and then steps from it.
    #[test]
    fn an_arrow_reads_the_typed_text_before_it_steps() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&old_copter(), bundled);
        type_into(&mut failsafe, NumberId::ThrottlePwm, "1000");
        let start = Instant::now();
        failsafe.step(NumberId::ThrottlePwm, true, start);
        assert_eq!(shown(&failsafe, NumberId::ThrottlePwm), "1001");
        let writes = failsafe.due(start + WRITE_DELAY);
        assert_eq!(writes.len(), 1, "one timer for both changes");
        assert_eq!(writes[0].param, "FS_THR_VALUE");
        assert!((writes[0].value - 1001.0).abs() < f64::EPSILON);
    }

    /// A typed value above the maximum - `FS_THR_VALUE`'s documented 1100 - asks "Out of range";
    /// Yes takes it as the new maximum and writes it, No leaves the box at the maximum it was held
    /// to, and writes that.
    #[test]
    fn a_typed_value_above_the_maximum_asks_first() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        type_into(&mut failsafe, NumberId::ThrottlePwm, "1150");
        let start = Instant::now();
        failsafe.leave(start);
        assert_eq!(
            failsafe.question().map(Question::text).as_deref(),
            Some("FS_THR_VALUE Value out of range\nDo you want to accept the new value?")
        );
        assert!(
            failsafe.due(start + WRITE_DELAY).is_empty(),
            "nothing is written while it asks"
        );
        failsafe.answer(true, start);
        assert!(failsafe.question().is_none());
        assert_eq!(shown(&failsafe, NumberId::ThrottlePwm), "1150");
        let writes = failsafe.due(start + WRITE_DELAY);
        assert!((writes.first().map_or(0.0, |write| write.value) - 1150.0).abs() < f64::EPSILON);

        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        type_into(&mut failsafe, NumberId::ThrottlePwm, "1150");
        failsafe.leave(start);
        failsafe.answer(false, start);
        assert_eq!(shown(&failsafe, NumberId::ThrottlePwm), "1100");
        let writes = failsafe.due(start + WRITE_DELAY);
        assert!((writes.first().map_or(0.0, |write| write.value) - 1100.0).abs() < f64::EPSILON);
    }

    /// A typed value below the minimum is held to it without a question: the C#'s handler asks
    /// only about the maximum.
    #[test]
    fn a_typed_value_below_the_minimum_is_held_to_it() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        type_into(&mut failsafe, NumberId::ThrottlePwm, "5");
        failsafe.leave(Instant::now());
        assert!(failsafe.question().is_none());
        assert_eq!(shown(&failsafe, NumberId::ThrottlePwm), "910");
    }

    /// Text that is not a number is dropped, and the box shows its value again.
    #[test]
    fn text_that_does_not_parse_changes_nothing() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        type_into(&mut failsafe, NumberId::ReservedMah, "lots");
        let start = Instant::now();
        failsafe.leave(start);
        assert_eq!(shown(&failsafe, NumberId::ReservedMah), "0");
        assert!(failsafe.due(start + WRITE_DELAY).is_empty());
    }

    /// Closing the page with a number typed into reads it and writes it at once.
    #[test]
    fn closing_reads_the_box_being_typed_into() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&copter(), bundled);
        type_into(&mut failsafe, NumberId::LowTimer, "20");
        let pending = failsafe.close(Instant::now());
        assert_eq!(pending.len(), 1);
        assert_eq!(pending[0].param, "BATT_LOW_TIMER");
        assert!((pending[0].value - 20.0).abs() < f64::EPSILON);
    }

    /// A number whose parameter the vehicle lacks keeps the Designer's 13.1, disabled, to the
    /// places `setup`'s increment gives it; it takes no focus and no arrow.
    #[test]
    fn a_number_the_vehicle_lacks_shows_the_designer_value() {
        let mut failsafe = FailSafe::default();
        failsafe.open(&table(&[("FS_THR_ENABLE", 1.0)]), bundled);
        assert_eq!(shown(&failsafe, NumberId::LowVoltage), "13.1");
        // BATT_LOW_MAH's documented step of 50 leaves no decimal places.
        assert_eq!(shown(&failsafe, NumberId::ReservedMah), "13");
        // BATT_LOW_TIMER's box is never set up, and its row is hidden.
        assert_eq!(shown(&failsafe, NumberId::LowTimer), "13.1");
        failsafe.begin(NumberId::LowVoltage, Instant::now());
        assert_eq!(failsafe.editing(), None);
        failsafe.begin(NumberId::LowTimer, Instant::now());
        assert_eq!(failsafe.editing(), None);
        failsafe.step(NumberId::LowVoltage, true, Instant::now());
        assert_eq!(shown(&failsafe, NumberId::LowVoltage), "13.1");
    }

    /// The tooltips are the `.resx`'s, and the Battery box's Designer values its.
    #[test]
    fn the_tooltips_and_designer_values_are_the_csharps() {
        let (Some(resx), Some(designer)) = (
            crate::config_coverage::source::csharp(
                "GCSViews/ConfigurationView/ConfigFailSafe.resx",
            ),
            crate::config_coverage::source::csharp(
                "GCSViews/ConfigurationView/ConfigFailSafe.Designer.cs",
            ),
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let tip = |name: &str| values.get(&format!("{name}.ToolTip")).cloned();
        for (name, text) in [
            ("mavlinkNumericUpDownlow_voltage", tips::LOW_VOLTAGE),
            ("mavlinkCheckBoxlong_fs", tips::SHORT_LONG),
            ("mavlinkCheckBoxshort_fs", tips::SHORT_LONG),
            ("mavlinkCheckBoxgcs_fs", tips::PLANE_GCS),
            ("mavlinkCheckBoxthr_fs_action", tips::THROTTLE_ACTION),
            ("mavlinkNumericUpDownfs_thr_value", tips::PWM),
            ("mavlinkNumericUpDownthr_fs_value", tips::PWM),
            ("mavlinkCheckBoxthr_fs", tips::THROTTLE),
            ("mavlinkComboBox_fs_thr_enable", tips::THROTTLE_MODE),
        ] {
            assert_eq!(tip(name).as_deref(), Some(text), "{name}");
        }
        let with_tips = values
            .keys()
            .filter(|key| key.ends_with(".ToolTip"))
            .count();
        assert_eq!(with_tips, 9, "every tooltip the page has");
        // `Value = new decimal(new int[] { 131, 0, 0, 65536 })` and `DecimalPlaces = 1` on each of
        // the Battery box's numbers.
        for name in [
            "mavlinkNumericUpDownlow_voltage",
            "mavlinkNumericUpDownFS_BATT_MAH",
            "mavlinkNumericUpDownBATT_LOW_TIMER",
        ] {
            assert!(
                designer.contains(&format!("this.{name}.DecimalPlaces = 1;")),
                "{name}"
            );
            let value = designer
                .split(&format!("this.{name}.Value = new decimal(new int[] {{"))
                .nth(1)
                .and_then(|rest| rest.split("});").next())
                .map(|numbers| numbers.split_whitespace().collect::<String>());
            assert_eq!(value.as_deref(), Some("131,0,0,65536"), "{name}");
        }
        assert!((BATTERY_BOX.value - 13.1).abs() < f64::EPSILON);
        assert_eq!(BATTERY_BOX.decimals, 1);
    }

    /// The GUI script asserts facts this page records: each named whole, or made by the
    /// per-control `format!("config.failsafe.{key}...")` with a control this page names.
    #[test]
    fn the_gui_script_names_facts_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-failsafe.gui");
        let source = include_str!("failsafe.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            if let (Some("expect"), Some(key)) = (words.next(), words.next())
                && let Some(rest) = key.strip_prefix("config.failsafe.")
            {
                let known = source.contains(&format!("\"{key}\""))
                    || match rest.split_once('.') {
                        Some((control, field)) => {
                            source.contains(&format!("\"config.failsafe.{{key}}.{field}\""))
                                && source.contains(&format!("\"{control}\""))
                        }
                        None => {
                            // `ch<n>` and `out<n>`: the bars, one fact each.
                            let bar = ["ch", "out"].into_iter().find(|prefix| {
                                rest.strip_prefix(prefix).is_some_and(|digits| {
                                    !digits.is_empty() && digits.bytes().all(|b| b.is_ascii_digit())
                                })
                            });
                            match bar {
                                Some(prefix) => source
                                    .contains(&format!("\"config.failsafe.{prefix}{{channel}}\"")),
                                None => {
                                    source.contains("\"config.failsafe.{key}\"")
                                        && source.contains(&format!("\"{rest}\""))
                                }
                            }
                        }
                    };
                assert!(known, "{key}");
                facts += 1;
            }
        }
        assert!(facts >= 20, "{facts} facts");
    }
}
