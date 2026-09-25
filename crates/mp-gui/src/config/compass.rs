//! Compass, a page of Initial Setup under Mandatory Hardware, in the two forms the list offers
//! (`GCSViews/InitialSetup.cs:200-207`): `GCSViews/ConfigurationView/ConfigHWCompass2.cs` for a
//! vehicle that lists `COMPASS_PRIO1_ID` - ArduPilot 4.1 and later, where the parameter arrived
//! with compass ordering - and `ConfigHWCompass.cs` for one that does not: ArduPilot 4.0 and
//! earlier.
//!
//! `ConfigHWCompass2` shows, top to bottom as its Designer places it: the compass priority table -
//! every compass the vehicle lists, `COMPASS_PRIO1_ID` to `PRIO3_ID` first and then every other
//! nonzero `COMPASS_DEV_IDn`, each with its device id decoded into bus type, bus, address and
//! device type, whether it is missing (a priority naming a device no `DEV_ID` holds), whether it is
//! external and its orientation - with up and down buttons that reorder it and write the first
//! three back as the priorities; the Use Compass 1 to 3 and Automatically learn offsets check boxes
//! (`COMPASS_USE`, `COMPASS_USE2`, `COMPASS_USE3`, `COMPASS_LEARN`); Remove Missing; Reboot; the
//! Onboard Mag Calibration group - Start, Accept and Cancel over `MAV_CMD_DO_START_MAG_CAL`,
//! `DO_ACCEPT_MAG_CAL` and `DO_CANCEL_MAG_CAL`, a bar and a light per compass, the text box the
//! timer writes each `MAG_CAL_PROGRESS` and `MAG_CAL_REPORT` into, and the Fitness combo
//! (`COMPASS_CAL_FIT`); and Large Vehicle MagCal, `MAV_CMD_FIXED_MAG_CAL_YAW` with a heading asked
//! for. The External and Orientation columns only show: the table is `ReadOnly`, and the page
//! writes neither `COMPASS_EXTERN*` nor `COMPASS_ORIENT*`. It has no declination.
//!
//! `ConfigHWCompass` shows the declination (`COMPASS_DEC`, in degrees and minutes, and
//! `COMPASS_AUTODEC`), learn and the primary compass (`COMPASS_PRIMARY`); a group per compass
//! with its Use and External boxes, its orientation (`COMPASS_ORIENTn`) and its offsets and MOT
//! values; three quick-configure buttons for old boards; the same onboard calibration without the
//! lights; and Large Vehicle MagCal.
//!
//! The C# calls `setParam` and `doCommand` on the UI thread (the priority writes `await`
//! `setParamAsync`), each blocking until the vehicle answers or the retries run out, and shows a
//! message box when one fails. Here each handler's requests are a job, run one at a time in the
//! order the handlers ran, and a failure shows the C#'s message in a box over the window. The
//! pages' own questions - "Reboot required, reboot now?", "Reboot?", the heading for Large Vehicle
//! MagCal, the quick-configure firmware question - are boxes too, and what the C# does after the
//! answer is done after it here.
//!
//! The layouts are the Designer's and the `.resx`'s, every control at its `Location` in the 678 x
//! 582 and 650 x 474 pages. WinForms has a `DataGridView`, check boxes, combo boxes and progress
//! bars and gpui has none, so the table is rows of cells at the columns' widths, a check box is a
//! square, and a bar is a filled box. The colours are this application's.
//!
//! What is not ported, and why:
//!
//! * `ConfigHWCompass`'s Live Calibration button, `MagCalib.DoGUIMagCalib`: Mission Planner's own
//!   calibration from raw magnetometer samples, with its sphere display, is `MagCalib.cs`, not this
//!   page. The button is drawn and does nothing. Its group shows only for a plane from 3.7.1 or a
//!   vehicle without onboard calibration. Its fit is ported (`mp_calibration::magcalib`); what it
//!   still needs is its window - `ProgressReporterSphere`, three OpenTK `Sphere` controls
//!   (`ExtLibs/Controls/Sphere.cs`) drawing the samples in 3D as the vehicle is turned - and the
//!   live half of `MagCalib.cs:136-790`: the `RAW_IMU`/`SCALED_IMU2`/`SCALED_IMU3` subscriptions,
//!   the stream rates it sets and restores, the offsets zeroed first, the sphere-coverage test
//!   and `SaveOffsets` through `PREFLIGHT_SET_SENSOR_OFFSETS`;
//! * a "Log Calibration" button: neither page has one. `ConfigHWCompass.cs:362-373` still holds
//!   `BUT_MagCalibrationLog_Click` - "Min Throttle" asked, then `MagCalib.ProcessLog` - but its
//!   Designer makes no such button and wires nothing to it (only stale translations keep its
//!   text), so nothing calls it: dead C#, recorded and not ported (PLAN.md §12 D16).
//!   `ProcessLog`'s one live caller is the hidden Temp screen's `BUT_magfit2` ("mag calb log",
//!   `temp.cs:410-413`), which is not this page; its fit is `headless-planner magcal`;
//! * the table's row selection and its row headers' current-row arrow: WinForms' grid behaviour,
//!   not the page's - nothing the page does reads the selection;
//! * `HorizontalProgressBar.DrawLabel`'s value label, drawn 15 pixels below each bar where the
//!   next bar and the fitness row cover it: the percentage is written in the bar instead;
//! * the report dialog `Program.handleException` shows when a priority write times out: the
//!   `TimeoutException` escapes the `async void` handler, and that dialog sends a crash report to
//!   Mission Planner's server. The write stops there, as the C#'s does, and says nothing;
//! * the same for a Large Vehicle MagCal heading that does not parse: `double.Parse` throws out of
//!   the click handler, and nothing is sent;
//! * the stack trace `ex.ToString()` adds to a failed command's message: the exception's type and
//!   message are shown;
//! * `ConfigHWCompass` showing its onboard group, its "OR" and its Mission Planner group together
//!   when Control is held as the page opens: a page is opened by a click here, with no modifier
//!   keys to read.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use gpui::{
    AnyElement, Context, Div, FocusHandle, FontWeight, KeyDownEvent, SharedString, Window, div,
    prelude::*, px, rgb,
};
use mp_calibration::MagCalLog;
use mp_link::requests::RequestOutcome;

use super::battery_monitor::{float_text, param_text, parse_float};
use super::failsafe::{Lookup, options};
use super::flight_modes::{Firmware, Progress, firmware_of};
use crate::MissionPlanner;
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action_sized, theme};

/// The page's name in Initial Setup's list, `backstageViewPagecompass.Text`.
/// `// C#: GCSViews/InitialSetup.cs:203`
pub const TITLE: &str = "Compass";

/// `Strings.ERROR`, most message boxes' caption.
/// `// C#: ExtLibs/Strings/Strings.resx:130-132`
const ERROR_TITLE: &str = "Error";

/// `Strings.ErrorSettingParameter`, trailing space and all.
/// `// C#: ExtLibs/Strings/Strings.resx:168-170`
pub const ERROR_SETTING_PARAMETER: &str = "Error setting parameter ";

/// `Activate`'s message when a priority names a device no `DEV_ID` holds.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:141-144`
pub const MISSING_MESSAGE: &str =
    "Your compass configuration has changed, please review the missing compass";

/// `CheckReboot`'s question and its caption.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:161-162`
pub const REBOOT_REQUIRED: &str = "Reboot required, reboot now?";
/// That question's caption.
const REBOOT_TITLE: &str = "Reboot";

/// What `CheckReboot` says after a Yes when the reboot could not be sent.
///
/// **Deliberate divergence.** `doReboot` returns true when the reboot was sent, and the C# shows
/// this message when it returns true - so a reboot that went out is reported as failed, every
/// time. That is a bug in the C#, not a behaviour: the text asks for a manual reboot of hardware
/// that is already rebooting. Here it is shown only when nothing could be sent.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:166-169; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2553-2586`
pub const REBOOT_FAILED: &str = "Reboot failed. please manually reboot the hardware.";

/// `but_reboot_Click`'s question.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:502`
pub const REBOOT_QUESTION: &str = "Reboot?";

/// What the timer says when every compass it heard from has saved its calibration.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:457-463`
pub const PLEASE_REBOOT: &str = "Please reboot the autopilot";

/// `BUT_OBmagcalstart_Click`'s `catch`, before the exception.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:275-279`
pub const START_FAILED: &str =
    "Failed to start MAG CAL, check the autopilot is still responding.\n";

/// `doCommand`'s `TimeoutException`, as `ex.ToString()` begins.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2797`
pub const COMMAND_TIMEOUT: &str = "System.TimeoutException: Timeout on read - doCommand";

/// Large Vehicle MagCal's `InputBox`: title and prompt.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:479`
pub const MAGCAL_YAW_TITLE: &str = "MagCal Yaw";
/// The prompt.
pub const MAGCAL_YAW_PROMPT: &str =
    "Enter current heading in degrees\nNOTE: gps lock is required. Heading is true, not magnetic";

/// `timer1`'s interval: the designer leaves it at the `Timer` default, and a saved calibration
/// makes it a second.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.Designer.cs:311; ConfigHWCompass2.cs:452`
const TIMER_INTERVAL: Duration = Duration::from_millis(100);
/// The interval once a report says a calibration was saved.
const TIMER_SLOW: Duration = Duration::from_millis(1000);

/// The parameters the page reads or writes, for the facts.
const WATCHED: [&str; 12] = [
    "COMPASS_USE",
    "COMPASS_USE2",
    "COMPASS_USE3",
    "COMPASS_LEARN",
    "COMPASS_CAL_FIT",
    "COMPASS_PRIO1_ID",
    "COMPASS_PRIO2_ID",
    "COMPASS_PRIO3_ID",
    "COMPASS_EXTERNAL",
    "COMPASS_ORIENT",
    "COMPASS_DEV_ID",
    "COMPASS_DEV_ID2",
];

// ---------------------------------------------------------------------------------------------
// Device ids: `ExtLibs/Utilities/Device.cs` and `GCSViews/ConfigurationView/DeviceInfo.cs`.
// ---------------------------------------------------------------------------------------------

/// `Device.BusType`, as `DeviceInfo.BusType` writes it: the name without `BUS_TYPE_`, or the
/// number for the one three-bit value the enum does not name.
/// `// C#: ExtLibs/Utilities/Device.cs:16-25, 59; GCSViews/ConfigurationView/DeviceInfo.cs:23`
#[must_use]
pub fn bus_type_name(devid: u32) -> String {
    match devid & 0x7 {
        0 => "UNKNOWN",
        1 => "I2C",
        2 => "SPI",
        3 => "UAVCAN",
        4 => "SITL",
        5 => "MSP",
        6 => "SERIAL",
        other => return other.to_string(),
    }
    .to_owned()
}

/// `DeviceStructure.bus`: five bits, which instance of the bus type.
/// `// C#: ExtLibs/Utilities/Device.cs:60`
#[must_use]
pub const fn bus(devid: u32) -> u32 {
    (devid >> 3) & 0x1f
}

/// `DeviceStructure.address`: eight bits, the address on the bus.
/// `// C#: ExtLibs/Utilities/Device.cs:61`
#[must_use]
pub const fn address(devid: u32) -> u32 {
    (devid >> 8) & 0xff
}

/// `DeviceStructure.devtype`: eight bits, the driver's own device type.
/// `// C#: ExtLibs/Utilities/Device.cs:62`
#[must_use]
pub const fn devtype(devid: u32) -> u32 {
    (devid >> 16) & 0xff
}

/// `DeviceInfo.DevType` for a compass: `SENSOR_ID#` and the number on a DroneCAN bus, otherwise
/// `compass_type`'s name without `DEVTYPE_`, or the number for one it does not name. Every name the
/// page builds a row from contains "COMP", so the barometer, airspeed and IMU branches are never
/// taken here.
/// `// C#: GCSViews/ConfigurationView/DeviceInfo.cs:27-45; ExtLibs/Utilities/Device.cs:101-127`
#[must_use]
pub fn compass_dev_type(devid: u32) -> String {
    if devid & 0x7 == 3 {
        return format!("SENSOR_ID#{}", devtype(devid));
    }
    match devtype(devid) {
        0x01 => "HMC5883_OLD",
        0x07 => "HMC5883",
        0x02 => "LSM303D",
        0x04 => "AK8963",
        0x05 => "BMM150",
        0x06 => "LSM9DS1",
        0x08 => "LIS3MDL",
        0x09 => "AK09916",
        0x0A => "IST8310",
        0x0B => "ICM20948",
        0x0C => "MMC3416",
        0x0D => "QMC5883L",
        0x0E => "MAG3110",
        0x0F => "SITL",
        0x10 => "IST8308",
        0x11 => "RM3100",
        0x12 => "RM3100_2",
        0x13 => "MMC5883",
        0x14 => "AK09918",
        0x15 => "AK09915",
        0x16 => "QMC5883P",
        0x17 => "BMM350",
        0x18 => "IIS2MDC",
        0x19 => "LIS2MDL",
        other => return other.to_string(),
    }
    .to_owned()
}

/// A parameter's value, if the vehicle has listed it.
fn value_of(parameters: &[(String, f64)], name: &str) -> Option<f64> {
    parameters
        .iter()
        .find(|(held, _)| held == name)
        .map(|(_, value)| *value)
}

/// `MAVLinkParamList[string[] names]`: the one of the names the vehicle lists, or nothing when it
/// lists none of them - or more than one.
/// `// C#: ExtLibs/Mavlink/MAVLinkParamList.cs:71-91`
fn one_of<'a>(parameters: &[(String, f64)], names: &[&'a str]) -> Option<(&'a str, f64)> {
    let mut found = None;
    for name in names {
        if let Some(value) = value_of(parameters, name) {
            if found.is_some() {
                return None;
            }
            found = Some((*name, value));
        }
    }
    found
}

/// `(uint)value`: a device id read from the parameter's double, truncated, wrapping a negative as
/// the x86 conversion does.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn device_id(value: f64) -> u32 {
    value as i64 as u32
}

/// The `DEV_ID`, `ORIENT` and `EXTERN` names of the first three compasses, each with the
/// `COMPASSn_` spelling ArduPilot's master briefly had.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:33-43`
const INSTANCE_NAMES: [([&str; 2], [&str; 2], [&str; 2]); 3] = [
    (
        ["COMPASS_DEV_ID", "COMPASS1_DEV_ID"],
        ["COMPASS_ORIENT", "COMPASS1_ORIENT"],
        ["COMPASS_EXTERNAL", "COMPASS1_EXTERN"],
    ),
    (
        ["COMPASS_DEV_ID2", "COMPASS2_DEV_ID"],
        ["COMPASS_ORIENT2", "COMPASS2_ORIENT"],
        ["COMPASS_EXTERN2", "COMPASS2_EXTERN"],
    ),
    (
        ["COMPASS_DEV_ID3", "COMPASS3_DEV_ID"],
        ["COMPASS_ORIENT3", "COMPASS3_ORIENT"],
        ["COMPASS_EXTERN3", "COMPASS3_EXTERN"],
    ),
];

/// One row of the priority table: a `CompassDeviceInfo`.
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceRow {
    /// `ParamName`: the parameter the id came from.
    pub param: String,
    /// `DeviceStructure.devid`.
    pub devid: u32,
    /// `Orient`: the matching `COMPASS_ORIENTn`'s text, if the id is one of the first three's.
    pub orient: Option<String>,
    /// `External`: the matching `COMPASS_EXTERNn` is above zero.
    pub external: bool,
    /// `Missing`: a priority naming a device no `DEV_ID` holds.
    pub missing: bool,
}

impl DeviceRow {
    /// `new CompassDeviceInfo(index, ParamName, id)`: the orientation and externality of whichever
    /// of the first three compasses has this id, later ones winning.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:30-60`
    fn new(param: &str, devid: u32, parameters: &[(String, f64)]) -> Self {
        let mut orient = None;
        let mut external = false;
        for (ids, orients, externs) in INSTANCE_NAMES {
            // `id1 != null && id1?.Value == id`: the double against the id.
            #[allow(clippy::float_cmp)]
            let matches =
                one_of(parameters, &ids).is_some_and(|(_, value)| value == f64::from(devid));
            if matches {
                orient = one_of(parameters, &orients).map(|(_, value)| param_text(value));
                external = one_of(parameters, &externs).is_some_and(|(_, value)| value > 0.0);
            }
        }
        Self {
            param: param.to_owned(),
            devid,
            orient,
            external,
            missing: false,
        }
    }

    /// `DevID`: the id as an `int`.
    #[must_use]
    #[allow(clippy::cast_possible_wrap)]
    pub const fn dev_id(&self) -> i32 {
        self.devid as i32
    }

    /// The row's cells, as the table shows them: Priority (its place, `RowPostPaint`), DevID,
    /// BusType, Bus, Address, DevType, Missing, External and the Orientation's text.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:466-474; ConfigHWCompass2.Designer.cs:450-519`
    #[must_use]
    pub fn cells(&self, place: usize, orientations: &[(String, String)]) -> [String; 9] {
        [
            (place + 1).to_string(),
            self.dev_id().to_string(),
            bus_type_name(self.devid),
            bus(self.devid).to_string(),
            address(self.devid).to_string(),
            compass_dev_type(self.devid),
            self.missing.to_string(),
            self.external.to_string(),
            orientation_text(self.orient.as_deref(), orientations),
        ]
    }
}

/// The Orientation column's text: the `DataGridViewComboBoxColumn` shows the option whose key is
/// the value, and nothing - its `DataError` cancelled - for a value it does not list.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:130-139, 509-512`
fn orientation_text(orient: Option<&str>, orientations: &[(String, String)]) -> String {
    orient
        .and_then(|orient| orientations.iter().find(|(key, _)| key == orient))
        .map_or_else(String::new, |(_, text)| text.clone())
}

/// `Activate`'s table: the nonzero priorities by name, each marked missing if no nonzero `DEV_ID`
/// holds its id, then the nonzero `DEV_ID`s by name that no priority names. Returns whether any
/// was missing.
///
/// The C# orders by `ParamName` with the culture's comparison; for the names ArduPilot uses -
/// one prefix followed by digits - that is the ordinal order used here.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:88-119`
#[must_use]
pub fn device_list(parameters: &[(String, f64)]) -> (Vec<DeviceRow>, bool) {
    let rows = |keep: &dyn Fn(&str) -> bool| {
        let mut rows: Vec<DeviceRow> = parameters
            .iter()
            .filter(|(name, value)| keep(name) && *value != 0.0)
            .map(|(name, value)| DeviceRow::new(name, device_id(*value), parameters))
            .collect();
        rows.sort_by(|a, b| a.param.cmp(&b.param));
        rows
    };
    let mut list = rows(&|name: &str| name.starts_with("COMPASS") && name.contains("DEV_ID"));
    let mut prio = rows(&|name: &str| name.starts_with("COMPASS_PRIO"));

    let mut any_missing = false;
    for row in &mut prio {
        let found = row.dev_id() == 0 || list.iter().any(|b| b.dev_id() == row.dev_id());
        row.missing = !found;
        any_missing |= !found;
    }
    list.retain(|row| !prio.iter().any(|b| b.dev_id() == row.dev_id()));
    prio.extend(list);
    (prio, any_missing)
}

/// One of `UpdateFirst3`'s writes: the parameter, the value, and whether a `false` from
/// `setParamAsync` is said (the clearing writes' is not).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PriorityWrite {
    /// `COMPASS_PRIOn_ID`.
    pub param: &'static str,
    /// The device id, or zero to clear it.
    pub value: f64,
    /// Whether a refusal shows `Strings.ErrorSettingParameter`.
    pub checked: bool,
}

/// `UpdateFirst3`: the first row's id as `COMPASS_PRIO1_ID`; the second's as `PRIO2_ID`, or zero
/// with fewer than two rows; the third's as `PRIO3_ID`, or zero.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:205-262`
#[must_use]
pub fn first3_writes(rows: &[DeviceRow]) -> Vec<PriorityWrite> {
    let id = |index: usize| rows.get(index).map(|row| f64::from(row.dev_id()));
    let mut writes = Vec::new();
    if let Some(value) = id(0) {
        writes.push(PriorityWrite {
            param: "COMPASS_PRIO1_ID",
            value,
            checked: true,
        });
    }
    for (index, param) in [(1, "COMPASS_PRIO2_ID"), (2, "COMPASS_PRIO3_ID")] {
        writes.push(match id(index) {
            Some(value) => PriorityWrite {
                param,
                value,
                checked: true,
            },
            None => PriorityWrite {
                param,
                value: 0.0,
                checked: false,
            },
        });
    }
    writes
}

// ---------------------------------------------------------------------------------------------
// The bound controls: `Controls/MavlinkCheckBox.cs` and `Controls/MavlinkComboBox.cs`.
// ---------------------------------------------------------------------------------------------

/// A check box's three states.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CheckState {
    /// Off.
    #[default]
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

/// A `MavlinkCheckBox` set up with `OnValue` 1 and `OffValue` 0, as every one on this page is.
/// `// C#: Controls/MavlinkCheckBox.cs:24-98`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Check {
    /// `ParamName`, once `setup` found one the vehicle lists.
    pub param: Option<&'static str>,
    /// What the box shows.
    pub state: CheckState,
    /// `Enabled`: false in the designer, true once bound.
    pub enabled: bool,
}

impl Check {
    /// `setup(1, 0, names, paramlist)`: bound to the first name the vehicle lists, checked for 1,
    /// unchecked for 0 and indeterminate otherwise; left as the designer has it - disabled and
    /// unchecked - when it lists none.
    /// `// C#: Controls/MavlinkCheckBox.cs:47-98`
    fn setup(names: &[&'static str], parameters: &[(String, f64)]) -> Self {
        let Some((param, value)) = names
            .iter()
            .find_map(|name| value_of(parameters, name).map(|value| (*name, value)))
        else {
            return Self::default();
        };
        #[allow(clippy::float_cmp)] // `paramlist[paramname].Value == OnValue`
        let state = if value == 1.0 {
            CheckState::Checked
        } else if value == 0.0 {
            CheckState::Unchecked
        } else {
            CheckState::Indeterminate
        };
        Self {
            param: Some(param),
            state,
            enabled: true,
        }
    }

    /// A click. WinForms takes a two-state box from checked or indeterminate to unchecked and from
    /// unchecked to checked, each a change of `Checked`, so each writes: 1 when it ends checked, 0
    /// otherwise. A disabled box takes no click.
    /// `// C#: Controls/MavlinkCheckBox.cs:106-143`
    fn click(&mut self) -> Option<Job> {
        let param = self.param.filter(|_| self.enabled)?;
        self.state = match self.state {
            CheckState::Unchecked => CheckState::Checked,
            CheckState::Checked | CheckState::Indeterminate => CheckState::Unchecked,
        };
        let value = if self.state == CheckState::Checked {
            1.0
        } else {
            0.0
        };
        // `String.Format(Strings.ErrorSetValueFailed, ParamName)`, for a `false` and a throw alike.
        Some(Job::bound(param, value, format!("Set {param} Failed")))
    }

    /// `Checked`: on, or indeterminate.
    #[must_use]
    pub const fn checked(self) -> bool {
        !matches!(self.state, CheckState::Unchecked)
    }
}

/// A `MavlinkComboBox` set up with an option list.
/// `// C#: Controls/MavlinkComboBox.cs:31-99`
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Combo {
    /// `ParamName`.
    pub param: &'static str,
    /// `DataSource`: the documented values and their text.
    pub options: Vec<(i64, String)>,
    /// The selected option; `None` is `SelectedIndex` -1.
    pub selected: Option<i64>,
    /// `Enabled`: false in the designer, true once bound.
    pub enabled: bool,
}

impl Combo {
    /// `setup(source, paramname, paramlist)`. Setting `DataSource` selects the first option; a
    /// parameter the vehicle lists then sets `SelectedValue` to its `(int)` value, which selects
    /// nothing if the list does not hold it.
    /// `// C#: Controls/MavlinkComboBox.cs:73-99`
    fn setup(param: &'static str, parameters: &[(String, f64)], lookup: Lookup) -> Self {
        let options = options(param, lookup);
        let first = options.first().map(|(key, _)| *key);
        match value_of(parameters, param) {
            Some(value) => {
                #[allow(clippy::cast_possible_truncation)] // `(int)paramlist[paramname].Value`
                let value = value as i64;
                let selected = options
                    .iter()
                    .any(|(key, _)| *key == value)
                    .then_some(value);
                Self {
                    param,
                    options,
                    selected,
                    enabled: true,
                }
            }
            None => Self {
                param,
                options,
                selected: first,
                enabled: false,
            },
        }
    }

    /// The selected option's text, blank when nothing is selected.
    #[must_use]
    pub fn text(&self) -> &str {
        self.selected
            .and_then(|selected| self.options.iter().find(|(key, _)| *key == selected))
            .map_or("", |(_, text)| text.as_str())
    }

    /// Chooses an option: a write when the selection changed, `(float)(int)SelectedValue`, whose
    /// failure is "Set {name} Failed!".
    /// `// C#: Controls/MavlinkComboBox.cs:133-199`
    fn choose(&mut self, key: i64) -> Option<Job> {
        if !self.enabled || self.selected == Some(key) {
            return None;
        }
        if !self.options.iter().any(|(option, _)| *option == key) {
            return None;
        }
        self.selected = Some(key);
        #[allow(clippy::cast_precision_loss)] // option values are small integers
        let value = f64::from(key as f32);
        Some(Job::bound(
            self.param,
            value,
            format!("Set {} Failed!", self.param),
        ))
    }

    /// `SelectedIndex = index`: the option at that place, and its write if that changed the
    /// selection. The handler runs whether or not the box is enabled.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:639, 643, 664, 691`
    fn select_index(&mut self, index: usize) -> Option<Job> {
        let key = self.options.get(index).map(|(key, _)| *key)?;
        if self.selected == Some(key) {
            return None;
        }
        self.selected = Some(key);
        #[allow(clippy::cast_precision_loss)] // option values are small integers
        let value = f64::from(key as f32);
        Some(Job::bound(
            self.param,
            value,
            format!("Set {} Failed!", self.param),
        ))
    }
}

// ---------------------------------------------------------------------------------------------
// The Onboard Mag Calibration group and its timer.
// ---------------------------------------------------------------------------------------------

/// `ToString("0.0")` on a float, as .NET Framework writes it: the float's seven significant
/// digits, then one place, the half rounded away from zero, and a negative that rounds to zero
/// written without its sign.
#[must_use]
pub fn one_place(value: f32) -> String {
    let text = float_text(value);
    let Ok(exact) = text.parse::<f64>() else {
        return text;
    };
    let rounded = (exact * 10.0).round() / 10.0;
    let written = format!("{rounded:.1}");
    if written == "-0.0" {
        "0.0".to_owned()
    } else {
        written
    }
}

/// What the group shows and whether its timer is running.
#[derive(Debug, Clone, PartialEq)]
pub struct Onboard {
    /// `horizontalProgressBar1` to `3`'s `Value`.
    pub bars: [u8; 3],
    /// `pictureBox1` to `3`, green once their compass reports.
    pub green: [bool; 3],
    /// `lbl_obmagresult`'s text.
    pub text: String,
    /// `BUT_OBmagcalaccept.Enabled`, false in the designer.
    pub accept_enabled: bool,
    /// `BUT_OBmagcalcancel.Enabled`, false in the designer.
    pub cancel_enabled: bool,
    /// `timer1` is running.
    pub timer: bool,
    /// `timer1.Interval`.
    pub interval: Duration,
    /// When the timer next fires.
    next: Option<Instant>,
}

impl Default for Onboard {
    fn default() -> Self {
        Self {
            bars: [0; 3],
            green: [false; 3],
            text: String::new(),
            accept_enabled: false,
            cancel_enabled: false,
            timer: false,
            interval: TIMER_INTERVAL,
            next: None,
        }
    }
}

/// What one tick found: how many compasses the progress messages name and how many reports say
/// saved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TickCount {
    /// `compasscount`.
    pub compasses: usize,
    /// `completecount`.
    pub complete: usize,
}

impl Onboard {
    /// `timer1.Start()`: the first tick one interval from now.
    fn start_timer(&mut self, now: Instant) {
        self.timer = true;
        self.next = Some(now + self.interval);
    }

    /// `timer1.Stop()`.
    fn stop_timer(&mut self) {
        self.timer = false;
        self.next = None;
    }

    /// `timer1_Tick`, reading the calibration's messages: the text box rewritten, each compass's
    /// bar at its percentage and at 100 with its light green once it reports, and the timer slowed
    /// to a second by a saved report.
    ///
    /// `line_end` is what ends the progress line: `"\r\n"` on this page, `"\n"` on the older one.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:358-456; ConfigHWCompass.cs:515-600`
    pub fn tick_text(&mut self, log: &MagCalLog, line_end: &str) -> TickCount {
        self.text.clear();
        let mut count = TickCount {
            compasses: 0,
            complete: 0,
        };
        let mut message = String::new();
        for progress in &log.progress {
            if let Some(bar) = self.bars.get_mut(usize::from(progress.compass_id)) {
                *bar = progress.percent;
            }
            message.push_str(&format!(
                "id:{} {}% ",
                progress.compass_id, progress.percent
            ));
            count.compasses += 1;
        }
        self.text.push_str(&message);
        self.text.push_str(line_end);

        for report in &log.reports {
            let [x, y, z] = report.offsets;
            self.text.push_str(&format!(
                "id:{} x:{} y:{} z:{} fit:{} {}\n",
                report.compass_id,
                one_place(x),
                one_place(y),
                one_place(z),
                one_place(report.fitness),
                mp_calibration::mag_cal_status_name(report.status),
            ));
            let index = usize::from(report.compass_id);
            if let Some(bar) = self.bars.get_mut(index) {
                *bar = 100;
            }
            if let Some(light) = self.green.get_mut(index) {
                *light = true;
            }
            if report.autosaved {
                count.complete += 1;
                self.interval = TIMER_SLOW;
            }
        }
        count
    }

    /// The text box's lines, as a multi-line box breaks them.
    #[must_use]
    pub fn lines(&self) -> Vec<&str> {
        self.text
            .split('\n')
            .map(|line| line.trim_end_matches('\r'))
            .filter(|line| !line.is_empty())
            .collect()
    }
}

// ---------------------------------------------------------------------------------------------
// Dialogs and jobs.
// ---------------------------------------------------------------------------------------------

/// What to do once a message box is dismissed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Then {
    /// Nothing.
    Nothing,
    /// Carry on with Start: `CheckReboot` returned true.
    Start,
}

/// A modal box the page is showing.
#[derive(Debug)]
pub enum Dialog {
    /// `CustomMessageBox.Show(text, caption)`: OK.
    Message {
        /// The caption.
        title: &'static str,
        /// The text.
        text: String,
        /// What the handler does after it.
        then: Then,
    },
    /// `CheckReboot`'s Yes/No question.
    RebootRequired {
        /// Whether Start asked, and carries on after a Yes.
        then_start: bool,
    },
    /// `but_reboot_Click`'s "Reboot?", OK; Cancel stands for the box's close button.
    RebootQuestion,
    /// Large Vehicle MagCal's `InputBox`: the heading typed so far.
    MagCalYaw(TextField),
    /// `buttonQuickPixhawk_Click`'s Yes/No question about the firmware.
    QuickFirmware,
}

impl Dialog {
    /// A message box with nothing after it.
    fn message(title: &'static str, text: impl Into<String>) -> Self {
        Self::Message {
            title,
            text: text.into(),
            then: Then::Nothing,
        }
    }

    /// The caption and the text.
    #[must_use]
    pub fn words(&self) -> (&str, &str) {
        match self {
            Self::Message { title, text, .. } => (title, text.as_str()),
            Self::RebootRequired { .. } => (REBOOT_TITLE, REBOOT_REQUIRED),
            Self::RebootQuestion => ("", REBOOT_QUESTION),
            Self::MagCalYaw(_) => (MAGCAL_YAW_TITLE, MAGCAL_YAW_PROMPT),
            Self::QuickFirmware => ("", QUICK_FIRMWARE),
        }
    }

    /// The two buttons' words; an empty second is a box with one.
    #[must_use]
    pub const fn buttons(&self) -> (&'static str, &'static str) {
        match self {
            Self::Message { .. } => ("OK", ""),
            Self::RebootRequired { .. } | Self::QuickFirmware => ("Yes", "No"),
            Self::RebootQuestion | Self::MagCalYaw(_) => ("OK", "Cancel"),
        }
    }
}

/// A message box's caption and text.
pub type Say = (&'static str, String);

/// One thing a handler does, in its order.
#[derive(Debug, Clone, PartialEq)]
pub enum Step {
    /// `setParam`: what a `false` shows, what a throw shows, and whether a throw ends the handler
    /// (the rest of its `try`).
    Set {
        /// The parameter.
        param: &'static str,
        /// The value.
        value: f64,
        /// Shown for a `false`.
        refused: Option<Say>,
        /// Shown for a throw.
        threw: Option<Say>,
        /// Whether a throw ends the handler.
        abort: bool,
    },
    /// `UpdateFirst3`'s last line: `rebootrequired = true`.
    RebootRequired,
    /// Start's `doCommand` and what follows it.
    Start,
    /// Accept's.
    Accept,
    /// Cancel's.
    Cancel,
    /// Large Vehicle MagCal's.
    FixedYaw(f32),
    /// `buttonQuickPixhawk_Click`'s question, which the rest of the handler waits on.
    AskFirmware,
    /// `Activate()`, which the quick-configure handlers end with.
    Activate,
}

/// A handler's steps, run when every earlier handler's have finished; `on_abort` runs when a step
/// ends it early (what follows its `try`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Job {
    steps: VecDeque<Step>,
    on_abort: Vec<Step>,
}

impl Job {
    /// Steps in order.
    fn of(steps: impl IntoIterator<Item = Step>) -> Self {
        Self {
            steps: steps.into_iter().collect(),
            on_abort: Vec::new(),
        }
    }

    /// A bound control's own `setParam`: `failure` for a `false` and for a throw alike.
    fn bound(param: &'static str, value: f64, failure: String) -> Self {
        Self::of([Step::Set {
            param,
            value,
            refused: Some((ERROR_TITLE, failure.clone())),
            threw: Some((ERROR_TITLE, failure)),
            abort: false,
        }])
    }

    /// The steps, for a test.
    #[must_use]
    pub fn steps(&self) -> Vec<Step> {
        self.steps.iter().cloned().collect()
    }
}

/// How a request ended, as the C# sees it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Answer {
    /// `setParam` or `doCommand` returned true.
    True,
    /// It returned false.
    False,
    /// It threw: every retry went unanswered.
    Threw,
}

impl Answer {
    /// A finished request's outcome in the C#'s terms. A request the link has let go is taken as
    /// having thrown.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1640-1651, 1765, 2797, 2822-2833`
    #[must_use]
    pub const fn of(progress: Progress) -> Option<Self> {
        match progress {
            Progress::Waiting => None,
            Progress::Lost | Progress::Finished(RequestOutcome::TimedOut) => Some(Self::Threw),
            Progress::Finished(RequestOutcome::Rejected(_) | RequestOutcome::UnknownParameter) => {
                Some(Self::False)
            }
            Progress::Finished(
                RequestOutcome::Accepted { .. } | RequestOutcome::Sent | RequestOutcome::Unchanged,
            ) => Some(Self::True),
        }
    }

    /// The word a fact carries.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::True => "accepted",
            Self::False => "refused",
            Self::Threw => "timeout",
        }
    }
}

/// What the page needs of the vehicle: the link, or a test's stand-in.
pub trait Autopilot {
    /// What a request is known by.
    type Handle: Copy;
    /// `setParam` on the vehicle being flown; `None` without one.
    fn set_param(&mut self, name: &str, value: f64) -> Option<Self::Handle>;
    /// Start's `doCommand`.
    fn start(&mut self) -> Option<Self::Handle>;
    /// Accept's.
    fn accept(&mut self) -> Option<Self::Handle>;
    /// Cancel's.
    fn cancel(&mut self) -> Option<Self::Handle>;
    /// Large Vehicle MagCal's.
    fn fixed_yaw(&mut self, yaw_degrees: f32) -> Option<Self::Handle>;
    /// How a request is getting on.
    fn progress(&self, handle: Self::Handle) -> Progress;
    /// `doReboot`.
    /// Whether the reboot could be sent: `doReboot`'s return.
    fn reboot(&mut self) -> bool;
    /// `mprog.Clear()`, `mrep.Clear()`.
    fn clear_mag_cal(&mut self);
    /// `mprog` and `mrep`.
    fn mag_cal(&self) -> MagCalLog;
    /// `MAV.param`, for an `Activate()` a handler calls.
    fn parameters(&self) -> Vec<(String, f64)>;
}

impl Autopilot for Telemetry {
    type Handle = mp_link::RequestId;

    fn set_param(&mut self, name: &str, value: f64) -> Option<Self::Handle> {
        self.set_parameter_confirmed(name, value)
    }

    fn start(&mut self) -> Option<Self::Handle> {
        self.start_compass_calibration()
    }

    fn accept(&mut self) -> Option<Self::Handle> {
        self.accept_compass_calibration()
    }

    fn cancel(&mut self) -> Option<Self::Handle> {
        self.cancel_compass_calibration()
    }

    fn fixed_yaw(&mut self, yaw_degrees: f32) -> Option<Self::Handle> {
        self.fixed_mag_cal_yaw(yaw_degrees)
    }

    fn progress(&self, handle: Self::Handle) -> Progress {
        match self.request(handle) {
            None => Progress::Lost,
            Some(request) => request
                .outcome()
                .map_or(Progress::Waiting, Progress::Finished),
        }
    }

    fn reboot(&mut self) -> bool {
        Telemetry::reboot(self)
    }

    fn clear_mag_cal(&mut self) {
        self.clear_compass_calibration();
    }

    fn mag_cal(&self) -> MagCalLog {
        self.compass_calibration()
    }

    fn parameters(&self) -> Vec<(String, f64)> {
        self.view().parameters.to_vec()
    }
}

/// The job being run.
#[derive(Debug)]
struct Running<H> {
    job: Job,
    /// The step whose request is out, and the request.
    out: Option<(Step, H)>,
}

/// The last request a job made to end, for the facts: what, and the answer.
#[derive(Debug, Clone, PartialEq)]
pub struct Ended {
    /// The parameter or the command.
    pub what: String,
    /// The answer.
    pub answer: Answer,
}

// ---------------------------------------------------------------------------------------------
// The older page: `GCSViews/ConfigurationView/ConfigHWCompass.cs`.
// ---------------------------------------------------------------------------------------------

/// `MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION`.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:7097`
const CAPABILITY_COMPASS_CALIBRATION: u32 = 4096;

/// `THRESHOLD_OFS_RED` and `THRESHOLD_OFS_YELLOW`.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:15-16`
const THRESHOLD_OFS_RED: i64 = 600;
/// The yellow threshold.
const THRESHOLD_OFS_YELLOW: i64 = 400;

/// `Rotation.ROTATION_NONE`.
/// `// C#: ExtLibs/Utilities/Vector3.cs:519`
const ROTATION_NONE: usize = 0;
/// `Rotation.ROTATION_ROLL_180`.
/// `// C#: ExtLibs/Utilities/Vector3.cs:527`
const ROTATION_ROLL_180: usize = 8;

/// `buttonQuickPixhawk_Click`'s question.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:636`
pub const QUICK_FIRMWARE: &str =
    "is the FW version greater than APM:copter 3.01 or APM:Plane 2.74?";

/// `Strings.ErrorNotConnected`.
/// `// C#: ExtLibs/Strings/Strings.resx:155-157`
const ERROR_NOT_CONNECTED: &str = "You are not connected.";

/// `Strings.ErrorFeatureNotEnabled`.
/// `// C#: ExtLibs/Strings/Strings.resx:143-145`
const FEATURE_NOT_ENABLED: &str = "This feature is not enabled in your firmware.";

/// `Strings.InvalidNumberEntered`.
/// `// C#: ExtLibs/Strings/Strings.resx:186-188`
const INVALID_NUMBER: &str = "Invalid number entered\n";

/// `linkLabelmagdec`'s page.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:268`
const DECLINATION_SITE: &str = "http://www.magnetic-declination.com/";

/// `linkLabel1`'s video.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:412`
const YOUTUBE_EXAMPLE: &str = "https://www.youtube.com/watch?v=DmsueBS0J3E";

/// `MathHelper.rad2deg`.
/// `// C#: ExtLibs/Utilities/Math.cs:10`
const RAD2DEG: f64 = 180.0 / std::f64::consts::PI;
/// `MathHelper.deg2rad`.
/// `// C#: ExtLibs/Utilities/Math.cs:11`
const DEG2RAD: f64 = 1.0 / RAD2DEG;

/// `CompassNumber`, `CMB_primary_compass`'s `DataSource`.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:19-24`
pub const COMPASS_NUMBERS: [&str; 3] = ["Compass1", "Compass2", "Compass3"];

/// What the older page's `Activate` asks of the vehicle beyond its parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct VehicleInfo {
    /// `BaseStream.IsOpen`.
    pub open: bool,
    /// `cs.firmware`.
    pub firmware: Firmware,
    /// `cs.version`: major, minor and patch - `AUTOPILOT_VERSION`'s, else the banner's.
    pub version: (u8, u8, u8),
    /// `cs.capabilities`.
    pub capabilities: u32,
}

impl VehicleInfo {
    /// The vehicle a view shows, with the firmware's banner.
    #[must_use]
    pub fn of(view: &TelemetryView, banner: Option<&str>) -> Self {
        let state = view.state.as_deref();
        let reported = state
            .map(|state| state.autopilot_info.version)
            .filter(|version| version[..3] != [0, 0, 0])
            .map(|version| (version[0], version[1], version[2]));
        Self {
            open: view.connected,
            firmware: state.map_or(Firmware::ArduCopter2, |state| {
                firmware_of(state.autopilot, state.vehicle_type, banner)
            }),
            version: reported
                .or_else(|| banner.and_then(banner_version))
                .unwrap_or((0, 0, 0)),
            capabilities: state.map_or(0, |state| state.autopilot_info.capabilities),
        }
    }
}

/// The version a firmware banner names: `ArduCopter V4.0.7 (...)` is 4.0.7.
fn banner_version(banner: &str) -> Option<(u8, u8, u8)> {
    let (_, rest) = banner.split_once(" V")?;
    let mut numbers = rest.split(|c: char| !c.is_ascii_digit());
    let major = numbers.next()?.parse().ok()?;
    let minor = numbers.next()?.parse().ok()?;
    let patch = numbers
        .next()
        .and_then(|patch| patch.parse().ok())
        .unwrap_or(0);
    Some((major, minor, patch))
}

/// `ToString("0")` on a double: rounded half away from zero, and a negative that rounds to zero
/// written without its sign, as .NET Framework writes it.
fn whole_text(value: f64) -> String {
    let rounded = value.round();
    if rounded == 0.0 {
        "0".to_owned()
    } else {
        format!("{rounded:.0}")
    }
}

/// How an offsets label is coloured.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OffsetColour {
    /// Past 600, or every offset zero (not calibrated).
    Red,
    /// Past 400.
    Yellow,
    /// Otherwise.
    Green,
}

impl OffsetColour {
    /// The word a fact carries.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::Red => "red",
            Self::Yellow => "yellow",
            Self::Green => "green",
        }
    }
}

/// An offsets label's text and colour: `OFFSETS  X: x,   Y: y,   Z: z` with each offset `(int)`,
/// red beyond 600 or all zero, yellow beyond 400.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:103-120, 247-250`
#[must_use]
pub fn offsets_label(offsets: [f64; 3]) -> (String, OffsetColour) {
    #[allow(clippy::cast_possible_truncation)] // `(int)MainV2.comPort.MAV.param[...]`
    let [x, y, z] = offsets.map(|value| value as i64);
    let absmax = x.abs().max(y.abs()).max(z.abs());
    let colour = if absmax > THRESHOLD_OFS_RED {
        OffsetColour::Red
    } else if absmax > THRESHOLD_OFS_YELLOW {
        OffsetColour::Yellow
    } else if x == 0 && y == 0 && z == 0 {
        OffsetColour::Red
    } else {
        OffsetColour::Green
    };
    (format!("OFFSETS  X: {x},   Y: {y},   Z: {z}"), colour)
}

/// A MOT label's text, `MOT          X: x,   Y: y,   Z: z`.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:121-127`
#[must_use]
pub fn mot_label(mot: [f64; 3]) -> String {
    #[allow(clippy::cast_possible_truncation)] // `(int)MainV2.comPort.MAV.param[...]`
    let [x, y, z] = mot.map(|value| value as i64);
    format!("MOT          X: {x},   Y: {y},   Z: {z}")
}

/// One of the older page's three compass group boxes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LegacyCompass {
    /// The group box shows: `Hide()` for a vehicle without its `EXTERN` parameter.
    pub visible: bool,
    /// `CHK_compassN_use`.
    pub use_check: Check,
    /// `CHK_compassN_external`.
    pub external: Check,
    /// `CMB_compassN_orient`.
    pub orient: Combo,
    /// `LBL_compassN_offset`: text and colour, or the designer's `OFFSET`.
    pub offsets: Option<(String, OffsetColour)>,
    /// `LBL_compassN_mot`, or the designer's `MOT`.
    pub mot: Option<String>,
}

impl Default for LegacyCompass {
    fn default() -> Self {
        Self {
            visible: true,
            use_check: Check::default(),
            external: Check::default(),
            orient: Combo::default(),
            offsets: None,
            mot: None,
        }
    }
}

/// The older page's own controls.
#[derive(Debug)]
pub struct Legacy {
    /// The page's `Enabled`.
    pub enabled: bool,
    /// `startup`: handlers write nothing while it is set. `Activate` leaves it set where it
    /// returns early.
    startup: bool,
    /// `label1` and the three quick-configure buttons.
    pub quick_visible: bool,
    /// `groupBoxonboardcalib`.
    pub onboard_visible: bool,
    /// `label4`, "OR".
    pub or_visible: bool,
    /// `groupBoxmpcalib`.
    pub mpcalib_visible: bool,
    /// `CHK_autodec.Checked`.
    pub autodec: bool,
    /// `TXT_declination_deg`.
    pub degrees: TextField,
    /// `TXT_declination_min`.
    pub minutes: TextField,
    /// The three compasses.
    pub compasses: [LegacyCompass; 3],
    /// `CMB_primary_compass`'s selection, a `CompassNumber`.
    pub primary: Option<usize>,
    /// Its `Enabled`.
    pub primary_enabled: bool,
    /// Its and its label's `Visible`.
    pub primary_visible: bool,
}

impl Default for Legacy {
    fn default() -> Self {
        Self {
            enabled: true,
            startup: false,
            quick_visible: true,
            onboard_visible: true,
            or_visible: true,
            mpcalib_visible: true,
            autodec: false,
            degrees: TextField::new(""),
            minutes: TextField::new(""),
            compasses: [
                LegacyCompass::default(),
                LegacyCompass::default(),
                LegacyCompass::default(),
            ],
            // `DataSource = Enum.GetNames(...)` selects the first.
            primary: Some(0),
            primary_enabled: false,
            primary_visible: true,
        }
    }
}

impl Legacy {
    /// The declination boxes' `Enabled`: not while it is obtained automatically.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:377-387, 737-738`
    #[must_use]
    pub const fn declination_enabled(&self) -> bool {
        !self.autodec
    }

    /// `ShowRelevantFields`: each orientation shown only for an external compass and each offsets
    /// and MOT label only for one in use - both read from the boxes as they are drawn - and the
    /// primary only for a vehicle that has `COMPASS_PRIMARY`.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:735-756`
    fn show_relevant_fields(&mut self, parameters: &[(String, f64)]) {
        self.primary_visible = value_of(parameters, "COMPASS_PRIMARY").is_some();
    }
}

/// The names one of the older page's compasses reads.
#[derive(Debug, Clone, Copy)]
struct LegacyNames {
    /// `COMPASS_USEn`.
    use_name: &'static str,
    /// `COMPASS_EXTERNn`.
    extern_name: &'static str,
    /// `COMPASS_ORIENTn`.
    orient_name: &'static str,
    /// `COMPASS_OFSn_X`, `_Y`, `_Z`.
    offsets: [&'static str; 3],
    /// `COMPASS_MOTn_X`, `_Y`, `_Z`.
    mot: [&'static str; 3],
}

/// The three compasses' names.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:92-215`
const LEGACY_NAMES: [LegacyNames; 3] = [
    LegacyNames {
        use_name: "COMPASS_USE",
        extern_name: "COMPASS_EXTERNAL",
        orient_name: "COMPASS_ORIENT",
        offsets: ["COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z"],
        mot: ["COMPASS_MOT_X", "COMPASS_MOT_Y", "COMPASS_MOT_Z"],
    },
    LegacyNames {
        use_name: "COMPASS_USE2",
        extern_name: "COMPASS_EXTERN2",
        orient_name: "COMPASS_ORIENT2",
        offsets: ["COMPASS_OFS2_X", "COMPASS_OFS2_Y", "COMPASS_OFS2_Z"],
        mot: ["COMPASS_MOT2_X", "COMPASS_MOT2_Y", "COMPASS_MOT2_Z"],
    },
    LegacyNames {
        use_name: "COMPASS_USE3",
        extern_name: "COMPASS_EXTERN3",
        orient_name: "COMPASS_ORIENT3",
        offsets: ["COMPASS_OFS3_X", "COMPASS_OFS3_Y", "COMPASS_OFS3_Z"],
        mot: ["COMPASS_MOT3_X", "COMPASS_MOT3_Y", "COMPASS_MOT3_Z"],
    },
];

/// Three parameters, or `None` if any is missing - where the C#'s `(int)param[name]` throws.
fn three(parameters: &[(String, f64)], names: [&str; 3]) -> Option<[f64; 3]> {
    let [x, y, z] = names.map(|name| value_of(parameters, name));
    Some([x?, y?, z?])
}

/// Which of the older page's text boxes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Declination {
    /// `TXT_declination_deg`.
    Degrees,
    /// `TXT_declination_min`.
    Minutes,
}

/// Which combo box's list is dropped down.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComboId {
    /// `mavlinkComboBoxfitness`.
    Fitness,
    /// `CMB_compassN_orient`, zero-based.
    Orient(usize),
    /// `CMB_primary_compass`.
    Primary,
}

impl ComboId {
    /// The control id a script clicks.
    #[must_use]
    pub fn id(self) -> String {
        match self {
            Self::Fitness => "compass-fitness".to_owned(),
            Self::Orient(index) => format!("compass{}-orient", index + 1),
            Self::Primary => "compass-primary".to_owned(),
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The page.
// ---------------------------------------------------------------------------------------------

/// Which of the two classes the list shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Class {
    /// `ConfigHWCompass2`, for a vehicle with `COMPASS_PRIO1_ID`.
    #[default]
    Priority,
    /// `ConfigHWCompass`, for one without.
    Legacy,
}

impl Class {
    /// The class's name, as the list names it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Priority => "ConfigHWCompass2",
            Self::Legacy => "ConfigHWCompass",
        }
    }

    /// The class of a list entry.
    #[must_use]
    pub fn of(class: &str) -> Option<Self> {
        match class {
            "ConfigHWCompass2" => Some(Self::Priority),
            "ConfigHWCompass" => Some(Self::Legacy),
            _ => None,
        }
    }

    /// What the timer ends the progress line with.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:393; ConfigHWCompass.cs:549`
    const fn line_end(self) -> &'static str {
        match self {
            Self::Priority => "\r\n",
            Self::Legacy => "\n",
        }
    }
}

/// The page's state: a `ConfigHWCompass2` or `ConfigHWCompass` instance, which lives as long as
/// the SETUP screen that made it, shown and hidden by `Activate` and `Deactivate` as its entry is
/// chosen.
#[derive(Debug)]
pub struct Compass<H = mp_link::RequestId> {
    /// Between `Activate` and `Deactivate`.
    active: bool,
    /// Which class.
    class: Class,
    /// The screen the instance belongs to; a different one is a new instance.
    made_for: Option<Key>,
    /// Where the documentation comes from, for an `Activate()` a handler calls.
    lookup: Option<Lookup>,
    /// What `Activate` read of the vehicle.
    info: VehicleInfo,
    /// `list`, the priority table's rows.
    rows: Vec<DeviceRow>,
    /// The Orientation column's `DataSource`: each documented value as text, and its name.
    orientations: Vec<(String, String)>,
    /// `mavlinkComboBoxfitness`.
    fitness: Combo,
    /// The combo whose list is dropped down.
    open: Option<ComboId>,
    /// `mavlinkCheckBoxUseCompass1` to `3`.
    uses: [Check; 3],
    /// `CHK_compass_learn`.
    learn: Check,
    /// `rebootrequired`.
    reboot_required: bool,
    /// The Onboard Mag Calibration group.
    onboard: Onboard,
    /// The older page's own controls.
    legacy: Legacy,
    /// Which declination boxes had the focus last frame.
    focused: [bool; 2],
    /// The boxes showing, the front one on top.
    dialogs: VecDeque<Dialog>,
    /// Handlers' steps waiting their turn.
    jobs: VecDeque<Job>,
    /// The one being run.
    running: Option<Running<H>>,
    /// The last command to end.
    last_command: Option<Ended>,
    /// The last parameter write to end.
    last_write: Option<Ended>,
}

impl<H> Default for Compass<H> {
    fn default() -> Self {
        Self {
            active: false,
            class: Class::Priority,
            made_for: None,
            lookup: None,
            info: VehicleInfo::default(),
            rows: Vec::new(),
            orientations: Vec::new(),
            fitness: Combo::default(),
            open: None,
            uses: [Check::default(); 3],
            learn: Check::default(),
            reboot_required: false,
            onboard: Onboard::default(),
            legacy: Legacy::default(),
            focused: [false; 2],
            dialogs: VecDeque::new(),
            jobs: VecDeque::new(),
            running: None,
            last_command: None,
            last_write: None,
        }
    }
}

impl<H: Copy> Compass<H> {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// The box showing, if any.
    #[must_use]
    pub fn dialog(&self) -> Option<&Dialog> {
        self.dialogs.front()
    }

    /// How many jobs are waiting or running.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.jobs.len() + usize::from(self.running.is_some())
    }

    /// A new instance: what disposing the screen and making it again leaves. Boxes already
    /// showing, and requests already made, outlive it, as a modal box and an `await` outlive the
    /// control that started them.
    fn dispose(&mut self) {
        let dialogs = std::mem::take(&mut self.dialogs);
        let jobs = std::mem::take(&mut self.jobs);
        let running = self.running.take();
        let (last_command, last_write) = (self.last_command.take(), self.last_write.take());
        *self = Self {
            dialogs,
            jobs,
            running,
            last_command,
            last_write,
            ..Self::default()
        };
    }

    /// `Activate` for the class the list shows.
    pub fn activate(
        &mut self,
        class: Class,
        parameters: &[(String, f64)],
        key: Key,
        info: VehicleInfo,
        lookup: Lookup,
    ) {
        if self.made_for != Some(key) || self.class != class {
            self.dispose();
            self.made_for = Some(key);
            self.class = class;
        }
        self.active = true;
        self.open = None;
        self.read(parameters, info, lookup);
    }

    /// What `Activate` reads, for either class.
    fn read(&mut self, parameters: &[(String, f64)], info: VehicleInfo, lookup: Lookup) {
        self.lookup = Some(lookup);
        self.info = info;
        match self.class {
            Class::Priority => self.activate_priority(parameters, lookup),
            Class::Legacy => self.activate_legacy(parameters, lookup),
        }
    }

    /// `ConfigHWCompass2.Activate`: the table from the parameters, the fitness combo, the Use and
    /// learn boxes and the orientations, then the missing-compass message. Stops where the C#
    /// throws: with no single `COMPASS_ORIENT` the orientation list is not made, nor the message
    /// shown.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:86-145`
    fn activate_priority(&mut self, parameters: &[(String, f64)], lookup: Lookup) {
        let (rows, any_missing) = device_list(parameters);
        self.rows = rows;
        self.fitness = Combo::setup("COMPASS_CAL_FIT", parameters, lookup);
        self.uses = [
            Check::setup(&["COMPASS_USE", "COMPASS1_USE"], parameters),
            Check::setup(&["COMPASS_USE2", "COMPASS2_USE"], parameters),
            Check::setup(&["COMPASS_USE3", "COMPASS3_USE"], parameters),
        ];
        self.learn = Check::setup(&["COMPASS_LEARN"], parameters);
        let Some((orient, _)) = one_of(parameters, &["COMPASS_ORIENT", "COMPASS1_ORIENT"]) else {
            return;
        };
        self.orientations = options(orient, lookup)
            .into_iter()
            .map(|(key, text)| (key.to_string(), text))
            .collect();
        if any_missing {
            self.dialogs
                .push_back(Dialog::message(ERROR_TITLE, MISSING_MESSAGE));
        }
    }

    /// `ConfigHWCompass.Activate`, stopping where it returns or throws.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:31-243`
    fn activate_legacy(&mut self, parameters: &[(String, f64)], lookup: Lookup) {
        let info = self.info;
        let legacy = &mut self.legacy;
        if !info.open {
            legacy.enabled = false;
            return;
        }
        legacy.enabled = true;
        legacy.startup = true;

        // `// C#: :42-49`
        if info.version > (3, 2, 1) && info.firmware == Firmware::ArduCopter2 {
            legacy.quick_visible = false;
        }
        // Control held down is not asked: a page opens from a click, with no modifiers to hand
        // here. `// C#: :51-70`
        if info.version >= (3, 7, 1) && info.firmware == Firmware::ArduPlane {
            legacy.onboard_visible = true;
            legacy.or_visible = true;
            legacy.mpcalib_visible = true;
        } else if info.capabilities & CAPABILITY_COMPASS_CALIBRATION == 0 {
            legacy.onboard_visible = false;
            legacy.or_visible = false;
            legacy.mpcalib_visible = true;
        } else {
            legacy.onboard_visible = true;
            legacy.or_visible = false;
            legacy.mpcalib_visible = false;
        }

        // `// C#: :74-88`
        self.learn = Check::setup(&["COMPASS_LEARN"], parameters);
        if let Some(value) = value_of(parameters, "COMPASS_DEC") {
            let dec = value * RAD2DEG;
            #[allow(clippy::cast_possible_truncation)] // `(int)dec`
            let degrees = dec as i64;
            #[allow(clippy::cast_precision_loss)]
            let minutes = (dec - degrees as f64) * 60.0;
            legacy.degrees.set(degrees.to_string());
            legacy.minutes.set(whole_text(minutes));
        }
        if let Some(value) = value_of(parameters, "COMPASS_AUTODEC") {
            // `param.ToString() == "1"`; the change handler returns at `startup`.
            #[allow(clippy::float_cmp)]
            let on = value == 1.0;
            legacy.autodec = on;
        }

        // `// C#: :91-134`
        let [first, second, third] = &mut legacy.compasses;
        let [names, names2, names3] = LEGACY_NAMES;
        first.use_check = Check::setup(&[names.use_name], parameters);
        first.external = Check::setup(&[names.extern_name], parameters);
        first.orient = Combo::setup(names.orient_name, parameters, lookup);
        if value_of(parameters, "COMPASS_OFS_X").is_none() {
            legacy.enabled = false;
            return;
        }
        let Some(offsets) = three(parameters, names.offsets) else {
            return;
        };
        first.offsets = Some(offsets_label(offsets));
        if value_of(parameters, names.mot[0]).is_some() {
            let Some(mot) = three(parameters, names.mot) else {
                return;
            };
            first.mot = Some(mot_label(mot));
        }

        // `// C#: :136-228`
        for (names, (index, compass)) in [names2, names3].into_iter().zip([(1, second), (2, third)])
        {
            if value_of(parameters, names.extern_name).is_none() {
                compass.visible = false;
                continue;
            }
            compass.use_check = Check::setup(&[names.use_name], parameters);
            compass.external = Check::setup(&[names.extern_name], parameters);
            compass.orient = Combo::setup(names.orient_name, parameters, lookup);
            if index == 1 {
                // `setup(typeof(CompassNumber), "COMPASS_PRIMARY", ...)`: its name as the text,
                // which a number past the enum's selects nothing.
                // `// C#: :144; Controls/MavlinkComboBox.cs:102-125`
                if let Some(value) = value_of(parameters, "COMPASS_PRIMARY") {
                    legacy.primary_enabled = true;
                    #[allow(clippy::cast_possible_truncation)]
                    let number = value as i64;
                    legacy.primary = usize::try_from(number)
                        .ok()
                        .filter(|number| *number < COMPASS_NUMBERS.len());
                }
            }
            let Some(offsets) = three(parameters, names.offsets) else {
                return;
            };
            compass.offsets = Some(offsets_label(offsets));
            if value_of(parameters, names.mot[0]).is_some() {
                let Some(mot) = three(parameters, names.mot) else {
                    return;
                };
                compass.mot = Some(mot_label(mot));
            }
        }

        // `// C#: :230-242`
        self.fitness = Combo::setup("COMPASS_CAL_FIT", parameters, lookup);
        legacy.show_relevant_fields(parameters);
        legacy.startup = false;
    }

    /// `Deactivate`: the timer stops; on the newer page `CheckReboot` asks about a reboot the
    /// priorities need.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:147-152; ConfigHWCompass.cs:252-255`
    pub fn deactivate(&mut self, open: bool) {
        self.active = false;
        self.open = None;
        self.onboard.stop_timer();
        if self.class == Class::Priority {
            self.check_reboot(open, false);
        }
    }

    /// `CheckReboot`: with the link open and a reboot required, the question, whose answer decides
    /// the rest. Returns whether the caller carries on now, which it does only with the link
    /// closed.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:154-182`
    fn check_reboot(&mut self, open: bool, then_start: bool) -> bool {
        if !open {
            return true;
        }
        if self.reboot_required {
            self.dialogs
                .push_back(Dialog::RebootRequired { then_start });
        }
        false
    }

    /// Up (`up`) or Down on a row: moves it, if it can move, and writes the first three.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:184-203`
    pub fn move_row(&mut self, index: usize, up: bool) {
        if index >= self.rows.len() {
            return;
        }
        let target = if up {
            let Some(target) = index.checked_sub(1) else {
                return;
            };
            target
        } else {
            if index + 1 >= self.rows.len() {
                return;
            }
            index + 1
        };
        let row = self.rows.remove(index);
        self.rows.insert(target, row);
        self.update_first3();
    }

    /// Remove Missing: every missing row out of the table, then the first three written.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:519-532`
    pub fn remove_missing(&mut self) {
        self.rows.retain(|row| !row.missing);
        self.update_first3();
    }

    /// `UpdateFirst3`, queued: a checked write refused says `ErrorSettingParameter`, a throw ends
    /// it before `rebootrequired` is set.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:205-262`
    fn update_first3(&mut self) {
        let mut steps: Vec<Step> = first3_writes(&self.rows)
            .into_iter()
            .map(|write| Step::Set {
                param: write.param,
                value: write.value,
                refused: write
                    .checked
                    .then(|| (ERROR_TITLE, ERROR_SETTING_PARAMETER.to_owned())),
                threw: None,
                abort: true,
            })
            .collect();
        steps.push(Step::RebootRequired);
        self.jobs.push_back(Job::of(steps));
    }

    /// A check box clicked: `0`, `1`, `2` for the newer page's Use Compass 1 to 3, `3` for learn.
    ///
    /// On the older page learn has a handler of its own on the box's `CheckedChanged`, run before
    /// the box's own write: a `setParam` whose `false` it ignores and whose throw says "Set
    /// COMPASS_LEARN Failed". The box's write then finds the value already held.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:710-727; Controls/MavlinkCheckBox.cs:106-120`
    pub fn click_check(&mut self, which: usize) {
        let legacy = self.class == Class::Legacy;
        if legacy && !self.legacy.enabled {
            return;
        }
        let check = match which {
            0..=2 => self.uses.get_mut(which),
            3 => Some(&mut self.learn),
            _ => None,
        };
        let Some(mut job) = check.and_then(Check::click) else {
            return;
        };
        if legacy
            && which == 3
            && let Some(Step::Set { param, value, .. }) = job.steps.front().cloned()
        {
            job.steps.push_front(Step::Set {
                param,
                value,
                refused: None,
                threw: Some(("", "Set COMPASS_LEARN Failed".to_owned())),
                abort: false,
            });
        }
        self.jobs.push_back(job);
    }

    /// One of the older page's Use or External boxes clicked: its write, then
    /// `ShowRelevantFields`.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:730-733`
    pub fn click_legacy_check(
        &mut self,
        compass: usize,
        external: bool,
        parameters: &[(String, f64)],
    ) {
        if !self.legacy.enabled {
            return;
        }
        let Some(group) = self.legacy.compasses.get_mut(compass) else {
            return;
        };
        let check = if external {
            &mut group.external
        } else {
            &mut group.use_check
        };
        if let Some(job) = check.click() {
            self.jobs.push_back(job);
        }
        self.legacy.show_relevant_fields(parameters);
    }

    /// `CHK_autodec` clicked: the declination boxes follow it, and unless `startup` the parameter
    /// is written - a `false` ignored, a throw saying so - or, on a vehicle without it, "Not
    /// Available on" the firmware.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:375-406`
    pub fn click_autodec(&mut self, parameters: &[(String, f64)]) {
        if !self.legacy.enabled {
            return;
        }
        self.legacy.autodec = !self.legacy.autodec;
        if self.legacy.startup {
            return;
        }
        if value_of(parameters, "COMPASS_AUTODEC").is_none() {
            self.dialogs.push_back(Dialog::message(
                "",
                format!("Not Available on {}", self.info.firmware.label()),
            ));
            return;
        }
        self.jobs.push_back(Job::of([Step::Set {
            param: "COMPASS_AUTODEC",
            value: if self.legacy.autodec { 1.0 } else { 0.0 },
            refused: None,
            threw: Some(("", "Set COMPASS_AUTODEC Failed".to_owned())),
            abort: false,
        }]));
    }

    /// A key typed into a declination box.
    pub fn declination_key(&mut self, which: Declination, event: &KeyDownEvent) -> bool {
        if !self.legacy.enabled || !self.legacy.declination_enabled() {
            return false;
        }
        let field = match which {
            Declination::Degrees => &mut self.legacy.degrees,
            Declination::Minutes => &mut self.legacy.minutes,
        };
        matches!(field.key(event), KeyOutcome::Changed)
    }

    /// `TXT_declination_Validated`, as a declination box loses the focus: unless `startup`, the
    /// degrees and minutes as one angle - the minutes taken away from a negative one - written as
    /// radians.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:283-322`
    pub fn declination_validated(&mut self, parameters: &[(String, f64)]) {
        if self.legacy.startup {
            return;
        }
        if value_of(parameters, "COMPASS_DEC").is_none() {
            self.dialogs
                .push_back(Dialog::message(ERROR_TITLE, FEATURE_NOT_ENABLED));
            return;
        }
        let (Some(mut dec), Some(minutes)) = (
            parse_float(self.legacy.degrees.value()),
            parse_float(self.legacy.minutes.value()),
        ) else {
            self.dialogs
                .push_back(Dialog::message(ERROR_TITLE, INVALID_NUMBER));
            return;
        };
        if dec < 0.0 {
            dec -= minutes / 60.0;
        } else {
            dec += minutes / 60.0;
        }
        self.jobs.push_back(Job::of([Step::Set {
            param: "COMPASS_DEC",
            value: f64::from(dec) * DEG2RAD,
            refused: None,
            threw: Some((ERROR_TITLE, "Set COMPASS_DEC Failed".to_owned())),
            abort: false,
        }]));
    }

    /// A combo clicked: its list drops down, or folds away.
    pub fn toggle_combo(&mut self, combo: ComboId) {
        let enabled = match combo {
            ComboId::Fitness => self.fitness.enabled,
            ComboId::Orient(index) => self
                .legacy
                .compasses
                .get(index)
                .is_some_and(|group| group.orient.enabled),
            ComboId::Primary => self.legacy.primary_enabled,
        } && (self.class == Class::Priority || self.legacy.enabled);
        self.open = (enabled && self.open != Some(combo)).then_some(combo);
    }

    /// An option chosen from a combo's list.
    pub fn choose(&mut self, combo: ComboId, key: i64) {
        self.open = None;
        let job = match combo {
            ComboId::Fitness => self.fitness.choose(key),
            ComboId::Orient(index) => self
                .legacy
                .compasses
                .get_mut(index)
                .and_then(|group| group.orient.choose(key)),
            // `(float)(Int32)Enum.Parse(_source, this.Text)`, failing with
            // `Strings.ErrorSetValueFailed`. `// C#: Controls/MavlinkComboBox.cs:138-168`
            ComboId::Primary => {
                let index = usize::try_from(key)
                    .ok()
                    .filter(|index| *index < COMPASS_NUMBERS.len());
                match index {
                    Some(index) if self.legacy.primary != Some(index) => {
                        self.legacy.primary = Some(index);
                        #[allow(clippy::cast_precision_loss)]
                        let value = index as f64;
                        Some(Job::bound(
                            "COMPASS_PRIMARY",
                            value,
                            "Set COMPASS_PRIMARY Failed".to_owned(),
                        ))
                    }
                    _ => None,
                }
            }
        };
        if let Some(job) = job {
            self.jobs.push_back(job);
        }
    }

    /// The older page's three quick-configure buttons: `0` Pixhawk/PX4, `1` APM2.5, `2` APM and
    /// External. Each writes its set in a `try` whose `catch` says `ErrorSettingParameter`, then
    /// runs `Activate()`; Pixhawk asks about the firmware between the two. APM2.5 and External set
    /// the first orientation before writing, which writes it.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:612-708`
    pub fn click_quick(&mut self, which: usize, open: bool) {
        if !self.legacy.enabled {
            return;
        }
        if !open {
            self.dialogs
                .push_back(Dialog::message("", ERROR_NOT_CONNECTED));
            return;
        }
        let quiet = |param: &'static str, value: f64| Step::Set {
            param,
            value,
            refused: None,
            threw: Some((ERROR_TITLE, ERROR_SETTING_PARAMETER.to_owned())),
            abort: true,
        };
        let (orient, writes): (Option<usize>, Vec<Step>) = match which {
            0 => (
                None,
                vec![
                    quiet("COMPASS_USE", 1.0),
                    quiet("COMPASS_USE2", 1.0),
                    quiet("COMPASS_USE3", 0.0),
                    quiet("COMPASS_EXTERNAL", 1.0),
                    quiet("COMPASS_EXTERN2", 0.0),
                    quiet("COMPASS_EXTERN3", 0.0),
                    quiet("COMPASS_PRIMARY", 0.0),
                    quiet("COMPASS_LEARN", 1.0),
                    Step::AskFirmware,
                ],
            ),
            1 => (
                Some(ROTATION_NONE),
                vec![
                    quiet("COMPASS_USE1", 1.0),
                    quiet("COMPASS_USE2", 0.0),
                    quiet("COMPASS_USE3", 0.0),
                    quiet("COMPASS_EXTERNAL", 0.0),
                    quiet("COMPASS_EXTERN2", 0.0),
                    quiet("COMPASS_EXTERN3", 0.0),
                    quiet("COMPASS_PRIMARY", 0.0),
                    quiet("COMPASS_LEARN", 1.0),
                    Step::Activate,
                ],
            ),
            2 => (
                Some(ROTATION_ROLL_180),
                vec![
                    quiet("COMPASS_EXTERNAL", 1.0),
                    quiet("COMPASS_EXTERN2", 0.0),
                    quiet("COMPASS_EXTERN3", 0.0),
                    quiet("COMPASS_USE1", 1.0),
                    quiet("COMPASS_USE2", 0.0),
                    quiet("COMPASS_USE3", 0.0),
                    quiet("COMPASS_PRIMARY", 0.0),
                    quiet("COMPASS_LEARN", 1.0),
                    Step::Activate,
                ],
            ),
            _ => return,
        };
        let mut steps = Vec::new();
        if let Some(index) = orient
            && let Some(job) = self
                .legacy
                .compasses
                .first_mut()
                .and_then(|group| group.orient.select_index(index))
        {
            steps.extend(job.steps);
        }
        steps.extend(writes);
        let mut job = Job::of(steps);
        job.on_abort = vec![Step::Activate];
        self.jobs.push_back(job);
    }

    /// Start: on the newer page `CheckReboot` first when a reboot is required, then the command.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:264-269; ConfigHWCompass.cs:453-458`
    pub fn click_start(&mut self, open: bool) {
        if self.class == Class::Legacy && !self.legacy.enabled {
            return;
        }
        if self.class == Class::Priority && self.reboot_required && !self.check_reboot(open, true) {
            return;
        }
        self.jobs.push_back(Job::of([Step::Start]));
    }

    /// Accept.
    pub fn click_accept(&mut self) {
        if self.onboard.accept_enabled {
            self.jobs.push_back(Job::of([Step::Accept]));
        }
    }

    /// Cancel.
    pub fn click_cancel(&mut self) {
        if self.onboard.cancel_enabled {
            self.jobs.push_back(Job::of([Step::Cancel]));
        }
    }

    /// Reboot: the question.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:500-507`
    pub fn click_reboot(&mut self) {
        self.dialogs.push_back(Dialog::RebootQuestion);
    }

    /// Large Vehicle MagCal: the `InputBox`, holding `0`, the `double` the handler starts from.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:476-479; ExtLibs/Controls/InputBox.cs:29-35`
    pub fn click_large_magcal(&mut self) {
        if self.class == Class::Legacy && !self.legacy.enabled {
            return;
        }
        let mut field = TextField::new("");
        field.set("0");
        self.dialogs.push_back(Dialog::MagCalYaw(field));
    }

    /// A key for the box showing. Enter is its first button and Escape its second.
    pub fn dialog_key<A: Autopilot<Handle = H>>(
        &mut self,
        event: &KeyDownEvent,
        autopilot: &mut A,
    ) -> bool {
        let outcome = match self.dialogs.front_mut() {
            Some(Dialog::MagCalYaw(field)) => field.key(event),
            Some(_) => crate::fly::answer_key(event),
            None => return false,
        };
        match outcome {
            KeyOutcome::Submitted => self.answer(true, autopilot),
            KeyOutcome::Cancelled => self.answer(false, autopilot),
            KeyOutcome::Changed => {}
            KeyOutcome::Ignored => return false,
        }
        true
    }

    /// The box showing answered: `yes` is its first button - OK or Yes - and `false` its second.
    pub fn answer<A: Autopilot<Handle = H>>(&mut self, yes: bool, autopilot: &mut A) {
        let Some(dialog) = self.dialogs.pop_front() else {
            return;
        };
        match dialog {
            Dialog::Message { then, .. } => {
                if then == Then::Start {
                    self.jobs.push_back(Job::of([Step::Start]));
                }
            }
            // `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:159-178`
            Dialog::RebootRequired { then_start } => {
                if yes {
                    let sent = autopilot.reboot();
                    self.reboot_required = false;
                    if sent {
                        // The reboot went out. The C# would now say it failed (see
                        // REBOOT_FAILED); the calibration the question interrupted goes ahead.
                        if then_start {
                            self.jobs.push_back(Job::of([Step::Start]));
                        }
                    } else {
                        self.dialogs.push_back(Dialog::Message {
                            title: ERROR_TITLE,
                            text: REBOOT_FAILED.to_owned(),
                            then: if then_start {
                                Then::Start
                            } else {
                                Then::Nothing
                            },
                        });
                    }
                }
            }
            // `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:500-507`
            Dialog::RebootQuestion => {
                if yes {
                    autopilot.reboot();
                    self.reboot_required = false;
                }
            }
            // `double.Parse(answer)` runs whichever button was pressed - on the text only OK
            // keeps - and a heading that does not parse throws out of the handler.
            // `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:478-481; ExtLibs/Controls/InputBox.cs:29-35, 185-192`
            Dialog::MagCalYaw(field) => {
                if yes && let Some(yaw) = parse_float(field.value()) {
                    self.jobs.push_back(Job::of([Step::FixedYaw(yaw)]));
                }
            }
            // The first orientation set by index - which writes it - and, for an older firmware,
            // the first compass internal; then `Activate()`.
            // `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:635-651`
            Dialog::QuickFirmware => {
                let index = if yes {
                    ROTATION_NONE
                } else {
                    ROTATION_ROLL_180
                };
                let mut steps = self
                    .legacy
                    .compasses
                    .first_mut()
                    .and_then(|group| group.orient.select_index(index))
                    .map(|job| job.steps())
                    .unwrap_or_default();
                if !yes {
                    steps.push(Step::Set {
                        param: "COMPASS_EXTERNAL",
                        value: 0.0,
                        refused: None,
                        threw: Some((ERROR_TITLE, ERROR_SETTING_PARAMETER.to_owned())),
                        abort: true,
                    });
                }
                steps.push(Step::Activate);
                let mut job = Job::of(steps);
                job.on_abort = vec![Step::Activate];
                self.jobs.push_back(job);
            }
        }
    }

    /// Once a frame: a page whose screen has gone is disposed, a declination box that lost the
    /// focus is validated, the jobs move on, and the timer fires when it is due.
    pub fn tick<A: Autopilot<Handle = H>>(
        &mut self,
        autopilot: &mut A,
        view: &TelemetryView,
        on_setup: bool,
        focused: [bool; 2],
        now: Instant,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            self.dispose();
        }
        let was = std::mem::replace(&mut self.focused, focused);
        if self.active
            && self.class == Class::Legacy
            && was.iter().zip(focused).any(|(had, has)| *had && !has)
        {
            self.declination_validated(&view.parameters);
        }
        self.run_jobs(autopilot, now);
        if self.onboard.timer && self.onboard.next.is_some_and(|next| now >= next) {
            self.onboard.next = Some(now + self.onboard.interval);
            self.timer_tick(&autopilot.mag_cal());
        }
    }

    /// `timer1_Tick`, then its end: when every compass heard from has saved, the buttons off, the
    /// timer stopped, and "Please reboot the autopilot".
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:457-463; ConfigHWCompass.cs:602-608`
    pub fn timer_tick(&mut self, log: &MagCalLog) {
        let count = self.onboard.tick_text(log, self.class.line_end());
        if self.class == Class::Legacy {
            // The older page has no lights.
            self.onboard.green = [false; 3];
        }
        if count.compasses == count.complete && count.compasses != 0 {
            self.onboard.cancel_enabled = false;
            self.onboard.accept_enabled = false;
            self.onboard.stop_timer();
            self.dialogs.push_back(Dialog::message("", PLEASE_REBOOT));
        }
    }

    /// Runs the jobs: each step in turn, a request's answer handled as the C# handles the call
    /// returning.
    fn run_jobs<A: Autopilot<Handle = H>>(&mut self, autopilot: &mut A, now: Instant) {
        loop {
            let Some(mut running) = self.running.take() else {
                let Some(job) = self.jobs.pop_front() else {
                    return;
                };
                self.running = Some(Running { job, out: None });
                continue;
            };
            let (step, answer) = match running.out.take() {
                Some((step, handle)) => match Answer::of(autopilot.progress(handle)) {
                    Some(answer) => (step, answer),
                    None => {
                        running.out = Some((step, handle));
                        self.running = Some(running);
                        return;
                    }
                },
                None => {
                    let Some(step) = running.job.steps.pop_front() else {
                        // Finished: the next job, if any.
                        continue;
                    };
                    let made = match &step {
                        Step::Set { param, value, .. } => autopilot.set_param(param, *value),
                        Step::Start => autopilot.start(),
                        Step::Accept => autopilot.accept(),
                        Step::Cancel => autopilot.cancel(),
                        Step::FixedYaw(yaw) => autopilot.fixed_yaw(*yaw),
                        Step::RebootRequired => {
                            self.reboot_required = true;
                            self.running = Some(running);
                            continue;
                        }
                        Step::AskFirmware => {
                            // The rest of the handler follows the answer.
                            self.dialogs.push_back(Dialog::QuickFirmware);
                            continue;
                        }
                        Step::Activate => {
                            if let Some(lookup) = self.lookup {
                                let parameters = autopilot.parameters();
                                self.read(&parameters, self.info, lookup);
                            }
                            self.running = Some(running);
                            continue;
                        }
                    };
                    match made {
                        Some(handle) => {
                            running.out = Some((step, handle));
                            self.running = Some(running);
                            continue;
                        }
                        // No vehicle: `setParam` finds no such parameter and `doCommand` the port
                        // closed, and both return false.
                        None => (step, Answer::False),
                    }
                }
            };
            if self.answered(&step, answer, autopilot, now) {
                self.running = Some(running);
            } else {
                // The handler ends here; what follows its `try` still runs.
                let rest = std::mem::take(&mut running.job.on_abort);
                if !rest.is_empty() {
                    self.running = Some(Running {
                        job: Job::of(rest),
                        out: None,
                    });
                }
            }
        }
    }

    /// What a handler does once a call returns `answer`; `false` when that ends the handler.
    fn answered<A: Autopilot<Handle = H>>(
        &mut self,
        step: &Step,
        answer: Answer,
        autopilot: &mut A,
        now: Instant,
    ) -> bool {
        let command = match step {
            Step::Start => Some("DO_START_MAG_CAL"),
            Step::Accept => Some("DO_ACCEPT_MAG_CAL"),
            Step::Cancel => Some("DO_CANCEL_MAG_CAL"),
            Step::FixedYaw(_) => Some("FIXED_MAG_CAL_YAW"),
            _ => None,
        };
        if let Some(name) = command {
            self.last_command = Some(Ended {
                what: name.to_owned(),
                answer,
            });
        }
        match step {
            Step::Set {
                param,
                refused,
                threw,
                abort,
                ..
            } => {
                self.last_write = Some(Ended {
                    what: (*param).to_owned(),
                    answer,
                });
                let said = match answer {
                    Answer::True => None,
                    Answer::False => refused.clone(),
                    Answer::Threw => threw.clone(),
                };
                if let Some((title, text)) = said {
                    self.dialogs.push_back(Dialog::message(title, text));
                }
                !(answer == Answer::Threw && *abort)
            }
            // `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:271-293`
            Step::Start => {
                if answer == Answer::Threw {
                    self.dialogs.push_back(Dialog::message(
                        ERROR_TITLE,
                        format!("{START_FAILED}{COMMAND_TIMEOUT}"),
                    ));
                    return false;
                }
                autopilot.clear_mag_cal();
                self.onboard.bars = [0; 3];
                self.onboard.accept_enabled = true;
                self.onboard.cancel_enabled = true;
                self.onboard.start_timer(now);
                true
            }
            // `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:323-356`
            Step::Accept | Step::Cancel => {
                if answer == Answer::Threw {
                    self.dialogs
                        .push_back(Dialog::message(ERROR_TITLE, COMMAND_TIMEOUT));
                }
                self.onboard.stop_timer();
                true
            }
            // `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:481-496`
            Step::FixedYaw(_) => {
                self.dialogs.push_back(if answer == Answer::True {
                    Dialog::message(
                        crate::fly::strings::COMPLETED,
                        crate::fly::strings::COMPLETED,
                    )
                } else {
                    Dialog::message(ERROR_TITLE, crate::fly::strings::COMMAND_FAILED)
                });
                true
            }
            Step::RebootRequired | Step::AskFirmware | Step::Activate => true,
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Facts.
// ---------------------------------------------------------------------------------------------

/// Facts a UI test asserts on: the class, the table, the boxes, the calibration group, the box
/// showing, the last command and write to end, the older page's controls, and the vehicle's value
/// of each parameter the page touches.
pub fn record_facts<H: Copy>(compass: &Compass<H>, view: &TelemetryView) {
    use crate::facts::record;
    record("config.compass.active", compass.active);
    record("config.compass.class", compass.class.name());
    record("config.compass.rows", compass.rows.len());
    record(
        "config.compass.devids",
        compass
            .rows
            .iter()
            .map(|row| row.dev_id().to_string())
            .collect::<Vec<_>>()
            .join(","),
    );
    for (place, row) in compass.rows.iter().enumerate() {
        record(
            format!("config.compass.row.{}", place + 1),
            row.cells(place, &compass.orientations).join(" "),
        );
    }
    record(
        "config.compass.missing",
        compass.rows.iter().any(|row| row.missing),
    );
    for (index, check) in compass.uses.iter().enumerate() {
        record(
            format!("config.compass.use{}", index + 1),
            check.state.key(),
        );
        record(
            format!("config.compass.use{}.enabled", index + 1),
            check.enabled,
        );
    }
    record("config.compass.learn", compass.learn.state.key());
    record("config.compass.learn.enabled", compass.learn.enabled);
    record(
        "config.compass.fitness",
        compass
            .fitness
            .selected
            .map_or_else(|| "none".to_owned(), |key| key.to_string()),
    );
    record("config.compass.fitness.text", compass.fitness.text());
    record("config.compass.fitness.enabled", compass.fitness.enabled);
    record(
        "config.compass.open",
        compass.open.map_or_else(|| "none".to_owned(), ComboId::id),
    );
    record("config.compass.orientations", compass.orientations.len());
    record("config.compass.rebootrequired", compass.reboot_required);

    let onboard = &compass.onboard;
    let joined = |values: &mut dyn Iterator<Item = String>| values.collect::<Vec<_>>().join(",");
    record(
        "config.compass.cal.bars",
        joined(&mut onboard.bars.iter().map(u8::to_string)),
    );
    record(
        "config.compass.cal.green",
        joined(&mut onboard.green.iter().map(bool::to_string)),
    );
    let lines = onboard.lines();
    record(
        "config.compass.cal.progress",
        lines.first().copied().unwrap_or("").trim_end(),
    );
    record(
        "config.compass.cal.reports",
        lines.iter().skip(1).copied().collect::<Vec<_>>().join("; "),
    );
    record(
        "config.compass.cal.timer",
        if onboard.timer { "running" } else { "stopped" },
    );
    record("config.compass.cal.interval", onboard.interval.as_millis());
    record("config.compass.accept.enabled", onboard.accept_enabled);
    record("config.compass.cancel.enabled", onboard.cancel_enabled);

    record(
        "config.compass.dialog",
        compass.dialog().map_or_else(
            || "none".to_owned(),
            |dialog| dialog.words().1.trim_end().replace('\n', " "),
        ),
    );
    record("config.compass.jobs", compass.pending());
    let ended = |ended: Option<&Ended>| {
        ended.map_or_else(
            || "none".to_owned(),
            |ended| format!("{} {}", ended.what, ended.answer.key()),
        )
    };
    record(
        "config.compass.command",
        ended(compass.last_command.as_ref()),
    );
    record("config.compass.write", ended(compass.last_write.as_ref()));

    let legacy = &compass.legacy;
    record("config.compass.legacy.enabled", legacy.enabled);
    record("config.compass.legacy.quick", legacy.quick_visible);
    record("config.compass.legacy.onboard", legacy.onboard_visible);
    record("config.compass.legacy.mpcalib", legacy.mpcalib_visible);
    record("config.compass.legacy.autodec", legacy.autodec);
    record(
        "config.compass.legacy.declination",
        format!("{} {}", legacy.degrees.value(), legacy.minutes.value()),
    );
    record(
        "config.compass.legacy.primary",
        legacy
            .primary
            .and_then(|index| COMPASS_NUMBERS.get(index))
            .map_or("none", |name| name),
    );
    for (index, group) in legacy.compasses.iter().enumerate() {
        let key = format!("config.compass.legacy.{}", index + 1);
        record(format!("{key}.visible"), group.visible);
        record(format!("{key}.use"), group.use_check.state.key());
        record(format!("{key}.external"), group.external.state.key());
        record(format!("{key}.orient"), group.orient.text());
        record(
            format!("{key}.offsets"),
            group.offsets.as_ref().map_or_else(
                || "OFFSET".to_owned(),
                |(text, colour)| format!("{text} {}", colour.key()),
            ),
        );
        record(format!("{key}.mot"), group.mot.as_deref().unwrap_or("MOT"));
    }

    for name in WATCHED {
        if let Some(value) = value_of(&view.parameters, name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// The table's columns after its 20-pixel row headers: Priority, DevID, BusType, Bus, Address,
/// DevType, Missing, External, Orientation (the `DataGridViewColumn` default, 100), Up, Down.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.Designer.cs:442, 450-534`
const COLUMNS: [(&str, f32); 11] = [
    ("Priority", 50.0),
    ("DevID", 50.0),
    ("BusType", 60.0),
    ("Bus", 50.0),
    ("Address", 50.0),
    ("DevType", 150.0),
    ("Missing", 50.0),
    ("External", 50.0),
    ("Orientation", 100.0),
    ("Up", 40.0),
    ("Down", 40.0),
];
/// `RowHeadersWidth`.
const ROW_HEADER: f32 = 20.0;
/// A `DataGridView` row's default height, and its header's.
const ROW_HEIGHT: f32 = 22.0;

/// An absolutely placed box, at a Designer `Location` and `Size`.
fn at(x: f32, y: f32, width: f32, height: f32) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// A label at its `Location`.
fn label(x: f32, y: f32, text: impl Into<SharedString>, enabled: bool) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .text_xs()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(text.into())
}

/// A group box: its border, and its text on the border.
fn group_box(place: (f32, f32, f32, f32), text: &'static str) -> Div {
    let (x, y, width, height) = place;
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
                .child(text),
        )
}

/// A button at its `Location`, its `Size`'s width at least.
fn button(
    (x, y, width): (f32, f32, f32),
    id: &'static str,
    text: &'static str,
    enabled: bool,
    on_click: impl Fn(&(), &mut Window, &mut gpui::App) + 'static,
) -> Div {
    div().absolute().left(px(x)).top(px(y)).child(action_sized(
        id,
        text,
        theme::ACCENT,
        enabled,
        Some(px(width)),
        on_click,
    ))
}

/// A link label: underlined, opening its page.
fn link(x: f32, y: f32, id: &'static str, text: &'static str, url: &'static str) -> Div {
    div().absolute().left(px(x)).top(px(y)).child(
        crate::probe::measured(id, div())
            .id(id)
            .text_xs()
            .text_color(rgb(theme::ACCENT))
            .underline()
            .cursor_pointer()
            .child(text)
            .on_click(move |_event, _window, cx| cx.open_url(url)),
    )
}

/// A check box's square: filled when checked, a bar when indeterminate.
fn square(state: CheckState, enabled: bool) -> Div {
    let colour = if enabled { theme::ACCENT } else { theme::DIM };
    let mark = match state {
        CheckState::Unchecked => None,
        CheckState::Checked => Some(div().size(px(7.0)).bg(rgb(colour))),
        CheckState::Indeterminate => Some(div().w(px(7.0)).h(px(2.0)).bg(rgb(colour))),
    };
    div()
        .size(px(13.0))
        .flex_shrink_0()
        .flex()
        .items_center()
        .justify_center()
        .border_1()
        .border_color(rgb(if enabled { theme::DIM } else { theme::BORDER }))
        .bg(rgb(theme::BG))
        .children(mark)
}

/// A check box at its `Location` and `Size`, with its text: dimmed and inert while disabled.
fn check_box(
    (x, y, width, height): (f32, f32, f32, f32),
    id: SharedString,
    text: &'static str,
    (state, enabled): (CheckState, bool),
    on_click: impl Fn(&gpui::ClickEvent, &mut Window, &mut gpui::App) + 'static,
) -> AnyElement {
    let body = crate::probe::measured(id.clone(), div())
        .id(id)
        .flex()
        .items_center()
        .gap_1()
        .text_xs()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(square(state, enabled))
        .child(text);
    let body = if enabled {
        body.cursor_pointer().on_click(on_click).into_any_element()
    } else {
        body.into_any_element()
    };
    at(x, y, width, height)
        .flex()
        .items_center()
        .child(body)
        .into_any_element()
}

/// A combo box at its place: its text, and a click that drops its list down.
fn combo_box(
    combo: ComboId,
    text: String,
    enabled: bool,
    (x, y, width, height): (f32, f32, f32, f32),
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let id = combo.id();
    let base = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .size_full()
        .flex()
        .items_center()
        .justify_between()
        .px_1()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .child(div().truncate().child(text))
        .child("▾");
    let base = if enabled {
        base.bg(rgb(theme::ACTION))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.border_color(rgb(theme::ACCENT)))
            .on_click(cx.listener(move |this, _event, window, cx| {
                window.blur(cx);
                this.compass.toggle_combo(combo);
                cx.notify();
            }))
    } else {
        base.bg(rgb(theme::PANEL)).text_color(rgb(theme::DIM))
    };
    at(x, y, width, height).child(base).into_any_element()
}

/// A combo's list, dropped down beneath it, scrolling past `MaxDropDownItems`' eight.
fn dropdown(
    combo: ComboId,
    options: &[(i64, String)],
    selected: Option<i64>,
    (x, y, width): (f32, f32, f32),
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    const ROW: f32 = 16.0;
    let mut list = div()
        .id(SharedString::from(format!("{}-list", combo.id())))
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width.max(140.0)))
        .max_h(px(ROW * 8.0 + 2.0))
        .overflow_y_scroll()
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .occlude();
    for (key, text) in options {
        let key = *key;
        let chosen = selected == Some(key);
        let name = format!("{}-{key}", combo.id());
        list = list.child(
            crate::probe::measured(name.clone(), div())
                .id(SharedString::from(name))
                .flex_shrink_0()
                .h(px(ROW))
                .px_1()
                .text_xs()
                .bg(rgb(if chosen { theme::BORDER } else { theme::PANEL }))
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::ACTION)))
                .child(text.clone())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.compass.choose(combo, key);
                    cx.notify();
                })),
        );
    }
    list.into_any_element()
}

/// The priority table: its header, then a row per compass, scrolling within the Designer's
/// 672 x 209 as the grid does - across as well, since its columns come to 710.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.Designer.cs:417-534`
fn table(compass: &Compass, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let cell = |width: f32| {
        div()
            .flex_shrink_0()
            .w(px(width))
            .h(px(ROW_HEIGHT))
            .px_1()
            .flex()
            .items_center()
            .border_r_1()
            .border_b_1()
            .border_color(rgb(theme::BORDER))
            .text_xs()
            .overflow_hidden()
            .whitespace_nowrap()
    };
    let mut header = div().flex().child(cell(ROW_HEADER).bg(rgb(theme::ACTION)));
    for (name, width) in COLUMNS {
        header = header.child(
            cell(width)
                .bg(rgb(theme::ACTION))
                .text_color(rgb(theme::DIM))
                .child(name),
        );
    }
    let mut body = div().flex().flex_col().child(header);
    let count = compass.rows.len();
    for (place, row) in compass.rows.iter().enumerate() {
        let cells = row.cells(place, &compass.orientations);
        let mut line = div()
            .flex()
            .text_color(rgb(theme::TEXT))
            .child(cell(ROW_HEADER).bg(rgb(theme::ACTION)));
        for (index, (_, width)) in COLUMNS.iter().enumerate().take(9) {
            let text = cells.get(index).cloned().unwrap_or_default();
            line = line.child(match index {
                // Missing and External are check box columns, read-only.
                6 | 7 => cell(*width).justify_center().child(square(
                    if text == "true" {
                        CheckState::Checked
                    } else {
                        CheckState::Unchecked
                    },
                    false,
                )),
                _ => cell(*width).child(text),
            });
        }
        for (up, glyph, width) in [(true, "▲", 40.0), (false, "▼", 40.0)] {
            let id = format!(
                "compass-row-{}-{}",
                place + 1,
                if up { "up" } else { "down" }
            );
            let usable = if up { place != 0 } else { place + 1 < count };
            line = line.child(
                cell(width).justify_center().child(
                    crate::probe::measured(id.clone(), div())
                        .id(SharedString::from(id))
                        .px_1()
                        .text_color(rgb(if usable { theme::ACCENT } else { theme::DIM }))
                        .cursor_pointer()
                        .hover(|style| style.bg(rgb(theme::BORDER)))
                        .child(glyph)
                        .on_click(cx.listener(move |this, _event, _window, cx| {
                            this.compass.move_row(place, up);
                            cx.notify();
                        })),
                ),
            );
        }
        body = body.child(line);
    }
    let width = ROW_HEADER + COLUMNS.iter().map(|(_, width)| width).sum::<f32>();
    at(3.0, 49.0, 672.0, 209.0)
        .child(
            crate::probe::measured("compass-table", div())
                .id("compass-table")
                .size_full()
                .overflow_scroll()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::BG))
                .child(div().w(px(width)).child(body)),
        )
        .into_any_element()
}

/// Where the Onboard Mag Calibration group's controls sit on each page.
struct OnboardLayout {
    /// The group box.
    group: (f32, f32, f32, f32),
    /// Start's, Accept's and Cancel's `x`, all at `y` 20.
    buttons: [f32; 3],
    /// `lbl_obmagresult`.
    result: (f32, f32, f32, f32),
    /// The bars' `x` and width, at `y` 49, 78 and 107, 23 high.
    bars: (f32, f32),
    /// Whether the page has the lights beside the bars.
    lights: bool,
    /// The fitness combo.
    fitness: (f32, f32, f32, f32),
    /// Whether "Relax fitness if calibration fails" is red.
    relax_red: bool,
}

/// `ConfigHWCompass2`'s.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.Designer.cs:114-307`
const PRIORITY_ONBOARD: OnboardLayout = OnboardLayout {
    group: (3.0, 355.0, 600.0, 162.0),
    buttons: [44.0, 125.0, 206.0],
    result: (350.0, 20.0, 244.0, 110.0),
    bars: (57.0, 258.0),
    lights: true,
    fitness: (57.0, 135.0, 140.0, 21.0),
    relax_red: true,
};

/// `ConfigHWCompass`'s.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.resx (groupBoxonboardcalib and its controls)`
const LEGACY_ONBOARD: OnboardLayout = OnboardLayout {
    group: (17.0, 309.0, 413.0, 162.0),
    buttons: [16.0, 97.0, 178.0],
    result: (259.0, 20.0, 148.0, 110.0),
    bars: (49.0, 204.0),
    lights: false,
    fitness: (77.0, 136.0, 121.0, 21.0),
    relax_red: false,
};

/// The Onboard Mag Calibration group box.
fn onboard_group(
    compass: &Compass,
    layout: &OnboardLayout,
    enabled: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let onboard = &compass.onboard;
    let [start_x, accept_x, cancel_x] = layout.buttons;
    let mut group = group_box(layout.group, "Onboard Mag Calibration")
        .child(button(
            (start_x, 20.0, 75.0),
            "compass-cal-start",
            "Start",
            enabled,
            cx.listener(|this, _event: &(), _window, cx| {
                let open = this.telemetry.view().connected;
                this.compass.click_start(open);
                cx.notify();
            }),
        ))
        .child(button(
            (accept_x, 20.0, 75.0),
            "compass-cal-accept",
            "Accept",
            enabled && onboard.accept_enabled,
            cx.listener(|this, _event: &(), _window, cx| {
                this.compass.click_accept();
                cx.notify();
            }),
        ))
        .child(button(
            (cancel_x, 20.0, 75.0),
            "compass-cal-cancel",
            "Cancel",
            enabled && onboard.cancel_enabled,
            cx.listener(|this, _event: &(), _window, cx| {
                this.compass.click_cancel();
                cx.notify();
            }),
        ));
    let (bar_x, bar_width) = layout.bars;
    for (index, (name, y)) in [("Mag 1", 49.0), ("Mag 2", 78.0), ("Mag 3", 107.0)]
        .into_iter()
        .enumerate()
    {
        let value = onboard.bars.get(index).copied().unwrap_or(0).min(100);
        group = group.child(label(6.0, y + 10.0, name, enabled)).child(
            at(bar_x, y, bar_width, 23.0)
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::BG))
                .child(
                    div()
                        .absolute()
                        .left_0()
                        .top_0()
                        .h_full()
                        .w(px((bar_width - 2.0) * f32::from(value) / 100.0))
                        .bg(rgb(theme::ACCENT)),
                )
                .child(
                    div()
                        .absolute()
                        .size_full()
                        .flex()
                        .items_center()
                        .justify_center()
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .child(value.to_string()),
                ),
        );
        if layout.lights {
            let green = onboard.green.get(index).copied().unwrap_or(false);
            group = group.child(
                at(321.0, y, 23.0, 23.0)
                    .border_1()
                    .border_color(rgb(theme::BORDER))
                    .bg(rgb(if green { theme::OK } else { theme::PANEL })),
            );
        }
    }
    let mut text = div()
        .flex()
        .flex_col()
        .text_xs()
        .text_color(rgb(theme::TEXT));
    for line in onboard.lines() {
        text = text.child(line.to_owned());
    }
    let (x, y, width, height) = layout.result;
    let (fx, fy, fwidth, fheight) = layout.fitness;
    group = group
        .child(
            at(x, y, width, height).child(
                crate::probe::measured("compass-cal-result", div())
                    .id("compass-cal-result")
                    .size_full()
                    .p_1()
                    .overflow_y_scroll()
                    .border_1()
                    .border_color(rgb(theme::BORDER))
                    .bg(rgb(theme::BG))
                    .child(text),
            ),
        )
        .child(label(7.0, 139.0, "Fitness", enabled))
        .child(combo_box(
            ComboId::Fitness,
            compass.fitness.text().to_owned(),
            enabled && compass.fitness.enabled,
            layout.fitness,
            cx,
        ))
        .child(
            div()
                .absolute()
                .left(px(203.0))
                .top(px(139.0))
                .text_xs()
                .font_weight(FontWeight::BOLD)
                .text_color(rgb(if layout.relax_red {
                    theme::ALERT
                } else {
                    theme::TEXT
                }))
                .child("Relax fitness if calibration fails"),
        );
    if compass.open == Some(ComboId::Fitness) {
        group = group.child(dropdown(
            ComboId::Fitness,
            &compass.fitness.options,
            compass.fitness.selected,
            (fx, fy + fheight, fwidth),
            cx,
        ));
    }
    group.into_any_element()
}

/// Large Vehicle MagCal's button.
fn large_magcal(x: f32, y: f32, enabled: bool, cx: &mut Context<MissionPlanner>) -> Div {
    button(
        (x, y, 75.0),
        "compass-largemagcal",
        "Large Vehicle MagCal",
        enabled,
        cx.listener(|this, _event: &(), _window, cx| {
            this.compass.click_large_magcal();
            cx.notify();
        }),
    )
}

/// The page showing: `ConfigHWCompass2` as its Designer lays it out, or `ConfigHWCompass` as its
/// `.resx` does.
pub fn page(compass: &Compass, focus: &Focus, cx: &mut Context<MissionPlanner>) -> AnyElement {
    if !compass.active {
        return div().into_any_element();
    }
    match compass.class {
        Class::Priority => priority_page(compass, cx),
        Class::Legacy => legacy_page(compass, focus, cx),
    }
}

/// `ConfigHWCompass2`, 678 x 582.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.Designer.cs:84-571`
fn priority_page(compass: &Compass, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let use_box = |index: usize, x: f32, text: &'static str, cx: &mut Context<MissionPlanner>| {
        let check = compass.uses.get(index).copied().unwrap_or_default();
        check_box(
            (x, 277.0, 100.0, 17.0),
            SharedString::from(format!("compass-use{}", index + 1)),
            text,
            (check.state, check.enabled),
            cx.listener(move |this, _event, _window, cx| {
                this.compass.click_check(index);
                cx.notify();
            }),
        )
    };
    let body = div()
        .relative()
        .w(px(678.0))
        .h(px(582.0))
        .child(
            div()
                .absolute()
                .left(px(3.0))
                .top(px(0.0))
                .text_base()
                .text_color(rgb(theme::TEXT))
                .child("Compass Priority"),
        )
        // groupBox5: a group box ten pixels high with no text, drawn as the line it looks like.
        .child(at(-1.0, 24.0, 679.0, 1.0).bg(rgb(theme::BORDER)))
        .child(label(
            3.0,
            33.0,
            "Set the Compass Priority by reordering the compasses in the table below (Highest at the top)",
            true,
        ))
        .child(table(compass, cx))
        .child(label(
            0.0,
            261.0,
            "Do you want to disable any of the first 3 compasses?",
            true,
        ))
        .child(use_box(0, 3.0, "Use Compass 1", cx))
        .child(use_box(1, 109.0, "Use Compass 2", cx))
        .child(use_box(2, 215.0, "Use Compass 3", cx))
        .child(button(
            (321.0, 271.0, 75.0),
            "compass-remove-missing",
            "Remove Missing",
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.compass.remove_missing();
                cx.notify();
            }),
        ))
        .child(check_box(
            (414.0, 277.0, 148.0, 17.0),
            SharedString::from("compass-learn"),
            "Automatically learn offsets",
            (compass.learn.state, compass.learn.enabled),
            cx.listener(|this, _event, _window, cx| {
                this.compass.click_check(3);
                cx.notify();
            }),
        ))
        .child(label(
            0.0,
            297.0,
            "A reboot is required to adjust the ordering.",
            true,
        ))
        .child(button(
            (3.0, 313.0, 75.0),
            "compass-reboot",
            "Reboot",
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.compass.click_reboot();
                cx.notify();
            }),
        ))
        .child(label(
            0.0,
            339.0,
            "A mag calibration is required to remap the above changes.",
            true,
        ))
        .child(onboard_group(compass, &PRIORITY_ONBOARD, true, cx))
        .child(large_magcal(3.0, 523.0, true, cx));
    crate::ui::panel(TITLE, body).into_any_element()
}

/// A declination text box at its place: typing and a caret while it has the focus; dimmed and
/// inert while disabled.
fn declination_box(
    which: Declination,
    field: &TextField,
    handle: &FocusHandle,
    enabled: bool,
    (x, y, width, height): (f32, f32, f32, f32),
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let id = match which {
        Declination::Degrees => "compass-declination-deg",
        Declination::Minutes => "compass-declination-min",
    };
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
        .child(field.value().to_owned());
    let base = if enabled {
        base.track_focus(handle)
            .key_context("TextField")
            .on_key_down(cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                if this.compass.declination_key(which, event) {
                    cx.notify();
                }
            }))
            .bg(rgb(theme::ACTION))
            .border_color(rgb(theme::ACCENT))
            .text_color(rgb(theme::TEXT))
            .cursor_text()
    } else {
        base.bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::DIM))
    };
    at(x, y, width, height).child(base).into_any_element()
}

/// `ConfigHWCompass`, 650 x 474.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.resx; ConfigHWCompass.Designer.cs:31-501`
fn legacy_page(compass: &Compass, focus: &Focus, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let legacy = &compass.legacy;
    let enabled = legacy.enabled;
    let mut body = div()
        .relative()
        .w(px(650.0))
        .h(px(474.0))
        .child(
            div()
                .absolute()
                .left(px(7.0))
                .top(px(3.0))
                .text_base()
                .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
                .child("Compass"),
        )
        // groupBox2: five pixels high and no text, a line.
        .child(at(3.0, 23.0, 644.0, 1.0).bg(rgb(theme::BORDER)));

    if legacy.quick_visible {
        let quick = |which: usize,
                     place: (f32, f32, f32),
                     id: &'static str,
                     text: &'static str,
                     cx: &mut Context<MissionPlanner>| {
            button(
                place,
                id,
                text,
                enabled,
                cx.listener(move |this, _event: &(), _window, cx| {
                    let open = this.telemetry.view().connected;
                    this.compass.click_quick(which, open);
                    cx.notify();
                }),
            )
        };
        body = body
            .child(label(
                14.0,
                37.0,
                "Select device to quick-configure parameters:",
                enabled,
            ))
            .child(quick(
                0,
                (231.0, 33.0, 114.0),
                "compass-quick-pixhawk",
                "Pixhawk/PX4",
                cx,
            ))
            .child(quick(
                1,
                (349.0, 33.0, 144.0),
                "compass-quick-apm25",
                "APM2.5 (Internal Compass)",
                cx,
            ))
            .child(quick(
                2,
                (497.0, 33.0, 151.0),
                "compass-quick-apmexternal",
                "APM and External Compass",
                cx,
            ));
    }

    // groupBoxGeneralSettings.
    let declination = enabled && legacy.declination_enabled();
    let mut general = group_box((17.0, 62.0, 615.0, 85.0), "General Compass Settings")
        .child(check_box(
            (218.0, 18.0, 177.0, 22.0),
            SharedString::from("compass-autodec"),
            "Obtain declination automatically",
            (
                if legacy.autodec {
                    CheckState::Checked
                } else {
                    CheckState::Unchecked
                },
                enabled,
            ),
            cx.listener(|this, _event, window, cx| {
                window.blur(cx);
                let view = this.telemetry.view();
                this.compass.click_autodec(&view.parameters);
                cx.notify();
            }),
        ))
        .child(label(236.0, 44.0, "Degrees", enabled))
        .child(declination_box(
            Declination::Degrees,
            &legacy.degrees,
            &focus.degrees,
            declination,
            (284.0, 41.0, 53.0, 20.0),
            cx,
        ))
        .child(label(343.0, 43.0, "Minutes", enabled))
        .child(declination_box(
            Declination::Minutes,
            &legacy.minutes,
            &focus.minutes,
            declination,
            (391.0, 40.0, 53.0, 20.0),
            cx,
        ))
        .child(link(
            236.0,
            64.0,
            "compass-declination-site",
            "Declination WebSite",
            DECLINATION_SITE,
        ))
        .child(check_box(
            (465.0, 18.0, 148.0, 23.0),
            SharedString::from("compass-learn"),
            "Automatically learn offsets",
            (compass.learn.state, enabled && compass.learn.enabled),
            cx.listener(|this, _event, window, cx| {
                window.blur(cx);
                this.compass.click_check(3);
                cx.notify();
            }),
        ));
    if legacy.primary_visible {
        general = general
            .child(label(12.0, 48.0, "Primary Compass:", enabled))
            .child(combo_box(
                ComboId::Primary,
                legacy
                    .primary
                    .and_then(|index| COMPASS_NUMBERS.get(index))
                    .map_or("", |name| name)
                    .to_owned(),
                enabled && legacy.primary_enabled,
                (108.0, 44.0, 92.0, 21.0),
                cx,
            ));
        if compass.open == Some(ComboId::Primary) {
            let options: Vec<(i64, String)> = COMPASS_NUMBERS
                .iter()
                .zip(0..)
                .map(|(name, key)| (key, (*name).to_owned()))
                .collect();
            general = general.child(dropdown(
                ComboId::Primary,
                &options,
                legacy.primary.and_then(|index| i64::try_from(index).ok()),
                (108.0, 65.0, 92.0),
                cx,
            ));
        }
    }
    body = body.child(general);

    // groupBoxCompass1 to 3.
    for (index, (group, x)) in legacy
        .compasses
        .iter()
        .zip([17.0, 223.0, 432.0])
        .enumerate()
    {
        if !group.visible {
            continue;
        }
        let label_x = if index == 0 { 12.0 } else { 9.0 };
        let number = index + 1;
        let title = match index {
            0 => "Compass #1",
            1 => "Compass #2",
            _ => "Compass #3",
        };
        let mut groupbox = group_box((x, 153.0, 200.0, 150.0), title)
            .child(check_box(
                (12.0, 20.0, 148.0, 23.0),
                SharedString::from(format!("compass{number}-use")),
                "Use this compass",
                (group.use_check.state, enabled && group.use_check.enabled),
                cx.listener(move |this, _event, window, cx| {
                    window.blur(cx);
                    let view = this.telemetry.view();
                    this.compass
                        .click_legacy_check(index, false, &view.parameters);
                    cx.notify();
                }),
            ))
            .child(check_box(
                (12.0, 39.0, 148.0, 23.0),
                SharedString::from(format!("compass{number}-external")),
                "Externally mounted",
                (group.external.state, enabled && group.external.enabled),
                cx.listener(move |this, _event, window, cx| {
                    window.blur(cx);
                    let view = this.telemetry.view();
                    this.compass
                        .click_legacy_check(index, true, &view.parameters);
                    cx.notify();
                }),
            ));
        // `ShowRelevantFields`. `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:740-751`
        if group.external.checked() {
            groupbox = groupbox.child(combo_box(
                ComboId::Orient(index),
                group.orient.text().to_owned(),
                enabled && group.orient.enabled,
                (12.0, 68.0, 182.0, 21.0),
                cx,
            ));
        }
        if group.use_check.checked() {
            let (offsets, colour) = group.offsets.as_ref().map_or_else(
                || ("OFFSET".to_owned(), theme::TEXT),
                |(text, colour)| {
                    (
                        text.clone(),
                        match colour {
                            OffsetColour::Red => theme::ALERT,
                            OffsetColour::Yellow => theme::WARN,
                            OffsetColour::Green => theme::OK,
                        },
                    )
                },
            );
            groupbox = groupbox
                .child(
                    label(label_x, 103.0, offsets, enabled).text_color(rgb(if enabled {
                        colour
                    } else {
                        theme::DIM
                    })),
                )
                .child(label(
                    label_x,
                    120.0,
                    group.mot.clone().unwrap_or_else(|| "MOT".to_owned()),
                    enabled,
                ));
        }
        if group.external.checked() && compass.open == Some(ComboId::Orient(index)) {
            groupbox = groupbox.child(dropdown(
                ComboId::Orient(index),
                &group.orient.options,
                group.orient.selected,
                (12.0, 89.0, 182.0),
                cx,
            ));
        }
        body = body.child(groupbox);
    }

    if legacy.onboard_visible {
        body = body.child(onboard_group(compass, &LEGACY_ONBOARD, enabled, cx));
    }
    if legacy.or_visible {
        body = body.child(label(436.0, 309.0, "OR", enabled));
    }
    if legacy.mpcalib_visible {
        // `BUT_MagCalibrationLive` runs `MagCalib.DoGUIMagCalib`, Mission Planner's own
        // calibration from raw magnetometer samples, whose sphere window is not ported: the
        // button is drawn, and inert. The group holds nothing else - no Log Calibration: the
        // Designer makes none, and `BUT_MagCalibrationLog_Click` (`:362-373`) has no caller.
        // `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:257-261;
        // ConfigHWCompass.Designer.cs:266-279`
        body = body.child(
            group_box(
                (460.0, 309.0, 172.0, 57.0),
                "Mission Planner Mag Calibration",
            )
            .child(button(
                (6.0, 20.0, 66.0),
                "compass-live-calibration",
                "Live Calibration",
                false,
                |_event: &(), _window, _cx| {},
            ))
            .child(link(
                76.0,
                27.0,
                "compass-youtube",
                "Youtube Example",
                YOUTUBE_EXAMPLE,
            )),
        );
    }
    body = body.child(large_magcal(460.0, 372.0, enabled, cx));
    crate::ui::panel(TITLE, body).into_any_element()
}

/// The focus handles: the boxes', and the older page's two declination text boxes'.
pub struct Focus {
    /// The modal box's, and its text field's.
    pub dialog: FocusHandle,
    /// `TXT_declination_deg`'s.
    pub degrees: FocusHandle,
    /// `TXT_declination_min`'s.
    pub minutes: FocusHandle,
}

impl Focus {
    /// New handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            dialog: cx.focus_handle(),
            degrees: cx.focus_handle(),
            minutes: cx.focus_handle(),
        }
    }

    /// Which declination boxes have the focus.
    #[must_use]
    pub fn declination(&self, window: &Window) -> [bool; 2] {
        [
            self.degrees.is_focused(window),
            self.minutes.is_focused(window),
        ]
    }
}

/// The box showing, drawn over the whole window: every one is modal.
pub fn overlay(
    compass: &Compass,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let dialog = compass.dialog()?;
    let (title, text) = dialog.words();
    let (first, second) = dialog.buttons();
    let on_key = cx.listener(|this, event: &KeyDownEvent, _window, cx| {
        if this.compass.dialog_key(event, &mut this.telemetry) {
            cx.notify();
        }
    });
    let mut words = div()
        .flex()
        .flex_col()
        .text_sm()
        .text_color(rgb(theme::TEXT));
    for line in text.trim_end().split('\n') {
        words = words.child(line.trim_end_matches('\r').to_owned());
    }
    let mut body = crate::probe::measured("compass-dialog", div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(396.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(title.to_owned()),
        )
        .child(words);
    let mut on_key = Some(on_key);
    if let Dialog::MagCalYaw(field) = dialog
        && let Some(on_key) = on_key.take()
    {
        body = body.child(crate::textfield::text_field(
            "compass-dialog-value",
            field,
            &focus.dialog,
            focus.dialog.is_focused(window),
            px(372.0),
            on_key,
        ));
    }
    let mut buttons = div().flex().justify_end().gap_2().child(action_sized(
        "compass-dialog-ok",
        first,
        theme::ACCENT,
        true,
        Some(px(75.0)),
        cx.listener(|this, _event: &(), _window, cx| {
            this.compass.answer(true, &mut this.telemetry);
            cx.notify();
        }),
    ));
    if !second.is_empty() {
        buttons = buttons.child(action_sized(
            "compass-dialog-cancel",
            second,
            theme::DIM,
            true,
            Some(px(75.0)),
            cx.listener(|this, _event: &(), _window, cx| {
                this.compass.answer(false, &mut this.telemetry);
                cx.notify();
            }),
        ));
    }
    body = body.child(buttons);
    let body = match on_key {
        // No box to type in: the dialog holds the focus, so Enter and Escape reach it.
        Some(on_key) => body
            .id("compass-dialog-keys")
            .track_focus(&focus.dialog)
            .on_key_down(on_key)
            .into_any_element(),
        None => body.into_any_element(),
    };
    let size = window.viewport_size();
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(
                    div()
                        .id("compass-dialog-backdrop")
                        .w(size.width)
                        .h(size.height)
                        .flex()
                        .items_center()
                        .justify_center()
                        .occlude()
                        .child(body),
                ),
        )
        .with_priority(2)
        .into_any_element(),
    )
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use mp_calibration::{CompassProgress, CompassReport};

    use super::*;

    /// The bundled documentation, so no test depends on what another has fetched.
    fn bundled(name: &str) -> Option<&'static mp_params::ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    fn params(list: &[(&str, f64)]) -> Vec<(String, f64)> {
        list.iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    /// What the bundled SITL copter (ArduPilot 4.6+) lists of its compasses, read from it.
    fn sitl() -> Vec<(String, f64)> {
        params(&[
            ("COMPASS_AUTODEC", 1.0),
            ("COMPASS_CAL_FIT", 16.0),
            ("COMPASS_DEC", 0.223_357_215_523_719_8),
            ("COMPASS_DEV_ID", 97_539.0),
            ("COMPASS_DEV_ID2", 131_874.0),
            ("COMPASS_DEV_ID3", 263_178.0),
            ("COMPASS_DEV_ID4", 97_283.0),
            ("COMPASS_DEV_ID5", 97_795.0),
            ("COMPASS_DEV_ID6", 98_051.0),
            ("COMPASS_DEV_ID7", 0.0),
            ("COMPASS_DEV_ID8", 0.0),
            ("COMPASS_EXTERN2", 0.0),
            ("COMPASS_EXTERN3", 0.0),
            ("COMPASS_EXTERNAL", 1.0),
            ("COMPASS_LEARN", 0.0),
            ("COMPASS_ORIENT", 0.0),
            ("COMPASS_ORIENT2", 0.0),
            ("COMPASS_ORIENT3", 0.0),
            ("COMPASS_PRIO1_ID", 97_539.0),
            ("COMPASS_PRIO2_ID", 131_874.0),
            ("COMPASS_PRIO3_ID", 263_178.0),
            ("COMPASS_USE", 1.0),
            ("COMPASS_USE2", 1.0),
            ("COMPASS_USE3", 1.0),
        ])
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn info() -> VehicleInfo {
        VehicleInfo {
            open: true,
            firmware: Firmware::ArduCopter2,
            version: (4, 6, 0),
            capabilities: CAPABILITY_COMPASS_CALIBRATION,
        }
    }

    /// The page as the list shows it for the SITL copter.
    fn priority(parameters: &[(String, f64)]) -> Compass<usize> {
        let mut compass = Compass::default();
        compass.activate(Class::Priority, parameters, key(), info(), bundled);
        compass
    }

    /// The vehicle as a script: every request it is asked answered as `answers` says - by
    /// parameter name, or `start`, `accept`, `cancel`, `yaw` - and accepted otherwise.
    #[derive(Default)]
    struct Fake {
        params: Vec<(String, f64)>,
        answers: HashMap<&'static str, Progress>,
        made: Vec<(String, Progress)>,
        reboots: usize,
        cleared: usize,
        log: MagCalLog,
    }

    impl Fake {
        fn answering(answers: &[(&'static str, Progress)]) -> Self {
            Self {
                answers: answers.iter().copied().collect(),
                ..Self::default()
            }
        }

        fn request(&mut self, name: &str, key: &str) -> Option<usize> {
            let progress = self
                .answers
                .get(key)
                .copied()
                .unwrap_or(Progress::Finished(RequestOutcome::Accepted { value: None }));
            self.made.push((name.to_owned(), progress));
            Some(self.made.len() - 1)
        }

        fn names(&self) -> Vec<&str> {
            self.made.iter().map(|(name, _)| name.as_str()).collect()
        }
    }

    impl Autopilot for Fake {
        type Handle = usize;

        fn set_param(&mut self, name: &str, value: f64) -> Option<usize> {
            let key = self
                .answers
                .keys()
                .find(|key| **key == name)
                .copied()
                .unwrap_or("");
            self.request(&format!("{name}={value}"), key)
        }

        fn start(&mut self) -> Option<usize> {
            self.request("start", "start")
        }

        fn accept(&mut self) -> Option<usize> {
            self.request("accept", "accept")
        }

        fn cancel(&mut self) -> Option<usize> {
            self.request("cancel", "cancel")
        }

        fn fixed_yaw(&mut self, yaw_degrees: f32) -> Option<usize> {
            self.request(&format!("yaw {yaw_degrees}"), "yaw")
        }

        fn progress(&self, handle: usize) -> Progress {
            self.made
                .get(handle)
                .map_or(Progress::Lost, |(_, progress)| *progress)
        }

        fn reboot(&mut self) -> bool {
            self.reboots += 1;
            true
        }

        fn clear_mag_cal(&mut self) {
            self.cleared += 1;
            self.log = MagCalLog::default();
        }

        fn mag_cal(&self) -> MagCalLog {
            self.log.clone()
        }

        fn parameters(&self) -> Vec<(String, f64)> {
            self.params.clone()
        }
    }

    const REFUSED: Progress = Progress::Finished(RequestOutcome::Rejected(4));
    const TIMED_OUT: Progress = Progress::Finished(RequestOutcome::TimedOut);

    /// Runs the page's jobs to the end, one frame at `now`.
    fn run(compass: &mut Compass<usize>, fake: &mut Fake, now: Instant) {
        let view = TelemetryView::disconnected("test");
        compass.tick(fake, &view, true, [false; 2], now);
    }

    fn dialog_text(compass: &Compass<usize>) -> Option<String> {
        compass.dialog().map(|dialog| dialog.words().1.to_owned())
    }

    // --- Device ids ------------------------------------------------------------------------------

    /// The SITL's device ids read as `Device.DeviceStructure` reads them: bus type in the low
    /// three bits, then bus, address and device type; a DroneCAN compass is `SENSOR_ID#` and its
    /// number.
    /// `// C#: ExtLibs/Utilities/Device.cs:59-62, 101-127; GCSViews/ConfigurationView/DeviceInfo.cs:23-45`
    #[test]
    fn device_ids_decode_as_device_cs_decodes_them() {
        assert_eq!(bus_type_name(97_539), "UAVCAN");
        assert_eq!((bus(97_539), address(97_539)), (0, 125));
        assert_eq!(compass_dev_type(97_539), "SENSOR_ID#1");

        assert_eq!(bus_type_name(131_874), "SPI");
        assert_eq!((bus(131_874), address(131_874)), (4, 3));
        assert_eq!(compass_dev_type(131_874), "LSM303D");

        assert_eq!(bus_type_name(263_178), "SPI");
        assert_eq!((bus(263_178), address(263_178)), (1, 4));
        assert_eq!(compass_dev_type(263_178), "AK8963");

        // A three-bit bus type the enum does not name, and a device type it does not name.
        assert_eq!(bus_type_name(7), "7");
        assert_eq!(compass_dev_type(0x0003_0001), "3");
        assert_eq!(compass_dev_type(0x000F_0004), "SITL");
    }

    /// `Activate`'s table on the SITL: the three priorities, then the other nonzero device ids by
    /// name; the first compass external with its orientation named, the extras with neither.
    #[test]
    fn the_table_is_the_priorities_then_the_other_devices() {
        let compass = priority(&sitl());
        let ids: Vec<i32> = compass.rows.iter().map(DeviceRow::dev_id).collect();
        assert_eq!(ids, [97_539, 131_874, 263_178, 97_283, 97_795, 98_051]);
        let params: Vec<&str> = compass.rows.iter().map(|row| row.param.as_str()).collect();
        assert_eq!(
            params,
            [
                "COMPASS_PRIO1_ID",
                "COMPASS_PRIO2_ID",
                "COMPASS_PRIO3_ID",
                "COMPASS_DEV_ID4",
                "COMPASS_DEV_ID5",
                "COMPASS_DEV_ID6"
            ]
        );
        let orientations = &compass.orientations;
        assert_eq!(
            compass.rows[0].cells(0, orientations).join(" "),
            "1 97539 UAVCAN 0 125 SENSOR_ID#1 false true None"
        );
        assert_eq!(
            compass.rows[1].cells(1, orientations).join(" "),
            "2 131874 SPI 4 3 LSM303D false false None"
        );
        assert_eq!(
            compass.rows[3].cells(3, orientations).join(" "),
            "4 97283 UAVCAN 0 124 SENSOR_ID#1 false false "
        );
        assert!(compass.dialog().is_none());
    }

    /// A priority naming a device no `DEV_ID` holds is missing, and `Activate` says so; a device
    /// a priority names is not listed twice.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:98-115, 141-144`
    #[test]
    fn a_priority_naming_an_absent_device_is_missing() {
        let mut parameters = sitl();
        for (name, value) in &mut parameters {
            if name == "COMPASS_PRIO2_ID" {
                *value = 555.0;
            }
        }
        let (rows, any_missing) = device_list(&parameters);
        assert!(any_missing);
        let missing: Vec<bool> = rows.iter().map(|row| row.missing).collect();
        assert_eq!(missing, [false, true, false, false, false, false, false]);
        let ids: Vec<i32> = rows.iter().map(DeviceRow::dev_id).collect();
        assert_eq!(ids, [97_539, 555, 263_178, 131_874, 97_283, 97_795, 98_051]);

        let compass = priority(&parameters);
        assert_eq!(dialog_text(&compass).as_deref(), Some(MISSING_MESSAGE));
    }

    /// Without a single `COMPASS_ORIENT` the C# throws before the orientations and the message.
    #[test]
    fn without_an_orientation_activate_stops_before_the_message() {
        let mut parameters = sitl();
        parameters.retain(|(name, _)| name != "COMPASS_ORIENT");
        for (name, value) in &mut parameters {
            if name == "COMPASS_PRIO2_ID" {
                *value = 555.0;
            }
        }
        let compass = priority(&parameters);
        assert!(compass.orientations.is_empty());
        assert!(compass.dialog().is_none());
        assert_eq!(compass.rows.len(), 7);
    }

    /// `UpdateFirst3`: the first three rows' ids, a short table clearing what it lacks without a
    /// check.
    #[test]
    fn the_first_three_rows_are_the_priorities() {
        let (rows, _) = device_list(&sitl());
        let writes = first3_writes(&rows);
        assert_eq!(writes.len(), 3);
        assert_eq!(
            (writes[0].param, writes[0].value, writes[0].checked),
            ("COMPASS_PRIO1_ID", 97_539.0, true)
        );
        assert_eq!(
            (writes[2].param, writes[2].value, writes[2].checked),
            ("COMPASS_PRIO3_ID", 263_178.0, true)
        );

        let one = first3_writes(rows.get(..1).unwrap_or_default());
        assert_eq!(
            one.iter()
                .map(|write| (write.param, write.value, write.checked))
                .collect::<Vec<_>>(),
            [
                ("COMPASS_PRIO1_ID", 97_539.0, true),
                ("COMPASS_PRIO2_ID", 0.0, false),
                ("COMPASS_PRIO3_ID", 0.0, false)
            ]
        );
    }

    // --- The bound controls ------------------------------------------------------------------------

    /// A Use box checked for 1, unchecked for 0, indeterminate otherwise; a click writes 0 or 1
    /// through the retrying set; a vehicle without the parameter leaves it disabled.
    /// `// C#: Controls/MavlinkCheckBox.cs:60-143`
    #[test]
    fn a_use_box_reads_and_writes_its_parameter() {
        let mut compass = priority(&sitl());
        assert_eq!(compass.uses[0].state, CheckState::Checked);
        assert_eq!(compass.uses[0].param, Some("COMPASS_USE"));
        assert_eq!(compass.learn.state, CheckState::Unchecked);

        let mut fake = Fake::default();
        compass.click_check(0);
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(fake.names(), ["COMPASS_USE=0"]);
        assert_eq!(compass.uses[0].state, CheckState::Unchecked);
        assert!(compass.dialog().is_none());
        compass.click_check(0);
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(fake.names(), ["COMPASS_USE=0", "COMPASS_USE=1"]);

        let odd = Check::setup(&["X"], &params(&[("X", 2.0)]));
        assert_eq!(odd.state, CheckState::Indeterminate);
        assert!(odd.checked());
        let absent = Check::setup(&["COMPASS_USE3"], &[]);
        assert!(!absent.enabled);
        let mut absent_copy = absent;
        assert!(absent_copy.click().is_none());
        let old_name = Check::setup(
            &["COMPASS_USE", "COMPASS1_USE"],
            &params(&[("COMPASS1_USE", 0.0)]),
        );
        assert_eq!(old_name.param, Some("COMPASS1_USE"));
    }

    /// A refused write shows `Strings.ErrorSetValueFailed` for the box's parameter.
    #[test]
    fn a_refused_use_write_says_so() {
        let mut compass = priority(&sitl());
        let mut fake = Fake::answering(&[("COMPASS_USE2", REFUSED)]);
        compass.click_check(1);
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(
            dialog_text(&compass).as_deref(),
            Some("Set COMPASS_USE2 Failed")
        );
        assert_eq!(
            compass.last_write,
            Some(Ended {
                what: "COMPASS_USE2".to_owned(),
                answer: Answer::False
            })
        );
    }

    /// The fitness combo lists `COMPASS_CAL_FIT`'s documented values with the vehicle's chosen,
    /// and a choice writes it; failing, "Set COMPASS_CAL_FIT Failed!".
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:121-122; Controls/MavlinkComboBox.cs:73-99, 169-199`
    #[test]
    fn the_fitness_combo_is_cal_fit() {
        let mut compass = priority(&sitl());
        assert_eq!(compass.fitness.options.len(), 4);
        assert_eq!(compass.fitness.selected, Some(16));
        assert_eq!(compass.fitness.text(), "Default");

        let mut fake = Fake::answering(&[("COMPASS_CAL_FIT", TIMED_OUT)]);
        compass.toggle_combo(ComboId::Fitness);
        assert_eq!(compass.open, Some(ComboId::Fitness));
        compass.choose(ComboId::Fitness, 32);
        assert_eq!(compass.open, None);
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(fake.names(), ["COMPASS_CAL_FIT=32"]);
        assert_eq!(
            dialog_text(&compass).as_deref(),
            Some("Set COMPASS_CAL_FIT Failed!")
        );
        // The same choice again is no change, and writes nothing.
        compass.choose(ComboId::Fitness, 32);
        assert_eq!(compass.pending(), 0);

        // A vehicle without it: disabled, showing the first option as `DataSource` leaves it.
        let absent = Combo::setup("COMPASS_CAL_FIT", &[], bundled);
        assert!(!absent.enabled);
        assert_eq!(absent.text(), "Very Strict");
    }

    // --- Priorities --------------------------------------------------------------------------------

    /// Up on the second row swaps the first two and writes the first three - the third already
    /// held, so not sent by the link, but asked for - then `rebootrequired`; leaving the page asks
    /// to reboot, and a Yes reboots and says what `CheckReboot` says.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:147-262`
    #[test]
    fn reordering_writes_the_priorities_and_asks_for_a_reboot() {
        let mut compass = priority(&sitl());
        let mut fake = Fake::default();
        compass.move_row(0, true);
        compass.move_row(5, false);
        assert_eq!(
            compass.pending(),
            0,
            "the first row cannot go up, the last down"
        );

        compass.move_row(1, true);
        assert_eq!(compass.rows[0].dev_id(), 131_874);
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(
            fake.names(),
            [
                "COMPASS_PRIO1_ID=131874",
                "COMPASS_PRIO2_ID=97539",
                "COMPASS_PRIO3_ID=263178"
            ]
        );
        assert!(compass.reboot_required);

        compass.deactivate(true);
        assert_eq!(dialog_text(&compass).as_deref(), Some(REBOOT_REQUIRED));
        compass.answer(true, &mut fake);
        assert_eq!(fake.reboots, 1);
        assert!(!compass.reboot_required);
        // The divergence at REBOOT_FAILED: a reboot that went out is not reported as failed.
        assert!(compass.dialog().is_none());
        assert_eq!(
            compass.pending(),
            0,
            "nothing starts after a Deactivate's reboot"
        );
    }

    /// A refused priority says `ErrorSettingParameter` and the rest are still written; a timeout
    /// ends the handler there, before `rebootrequired`.
    #[test]
    fn a_refused_priority_says_so_and_a_timeout_stops_the_rest() {
        let mut compass = priority(&sitl());
        let mut fake = Fake::answering(&[("COMPASS_PRIO2_ID", REFUSED)]);
        compass.move_row(2, false);
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(fake.made.len(), 3);
        assert_eq!(
            dialog_text(&compass).as_deref(),
            Some(ERROR_SETTING_PARAMETER)
        );
        assert!(compass.reboot_required);

        let mut compass = priority(&sitl());
        let mut fake = Fake::answering(&[("COMPASS_PRIO1_ID", TIMED_OUT)]);
        compass.move_row(1, true);
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(fake.names(), ["COMPASS_PRIO1_ID=131874"]);
        assert!(!compass.reboot_required);
        assert!(compass.dialog().is_none());
    }

    /// Remove Missing takes every missing row out, then writes the first three.
    #[test]
    fn remove_missing_rewrites_the_priorities_without_the_missing() {
        let mut parameters = sitl();
        for (name, value) in &mut parameters {
            if name == "COMPASS_PRIO1_ID" {
                *value = 555.0;
            }
        }
        let mut compass = priority(&parameters);
        let mut fake = Fake::default();
        compass.answer(true, &mut fake);
        compass.remove_missing();
        run(&mut compass, &mut fake, Instant::now());
        assert!(compass.rows.iter().all(|row| !row.missing));
        assert_eq!(
            fake.names(),
            [
                "COMPASS_PRIO1_ID=131874",
                "COMPASS_PRIO2_ID=263178",
                "COMPASS_PRIO3_ID=97539"
            ]
        );
    }

    // --- The onboard calibration -------------------------------------------------------------------

    fn progress(compass_id: u8, percent: u8) -> CompassProgress {
        CompassProgress {
            compass_id,
            status: 2,
            attempt: 1,
            percent,
        }
    }

    fn report(compass_id: u8, autosaved: bool) -> CompassReport {
        CompassReport {
            compass_id,
            offsets: [12.25, -2.0, -0.04],
            fitness: 3.25,
            status: mp_calibration::compass::MAG_CAL_SUCCESS,
            autosaved,
        }
    }

    /// `ToString("0.0")` and `ToString("0")` as .NET Framework writes them.
    #[test]
    fn numbers_are_written_as_dotnet_writes_them() {
        assert_eq!(one_place(12.25), "12.3");
        assert_eq!(one_place(-2.0), "-2.0");
        assert_eq!(one_place(-0.04), "0.0");
        assert_eq!(one_place(3.25), "3.3");
        assert_eq!(whole_text(47.8), "48");
        assert_eq!(whole_text(-0.4), "0");
        assert_eq!(whole_text(-30.0), "-30");
    }

    /// Start, then the timer: the text box lists each compass in the order first heard with its
    /// percentage, then each report with offsets, fitness and outcome; a report fills its bar and
    /// lights its light; every compass saved ends it - the buttons off, the timer stopped, a
    /// second between ticks from then on, and "Please reboot the autopilot".
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:264-294, 358-464`
    #[test]
    fn start_then_the_timer_reports_each_compass() {
        let mut compass = priority(&sitl());
        let mut fake = Fake::default();
        let now = Instant::now();
        assert!(!compass.onboard.accept_enabled);
        compass.click_start(true);
        run(&mut compass, &mut fake, now);
        assert_eq!(fake.names(), ["start"]);
        assert_eq!(fake.cleared, 1);
        assert!(compass.onboard.timer);
        assert!(compass.onboard.accept_enabled && compass.onboard.cancel_enabled);
        assert_eq!(
            compass.last_command,
            Some(Ended {
                what: "DO_START_MAG_CAL".to_owned(),
                answer: Answer::True
            })
        );

        fake.log.progress = vec![progress(1, 40), progress(2, 0), progress(0, 10)];
        run(&mut compass, &mut fake, now + Duration::from_millis(50));
        assert!(compass.onboard.text.is_empty(), "not yet an interval");
        run(&mut compass, &mut fake, now + Duration::from_millis(100));
        assert_eq!(compass.onboard.text, "id:1 40% id:2 0% id:0 10% \r\n");
        assert_eq!(compass.onboard.bars, [10, 40, 0]);

        fake.log.reports = vec![report(0, true), report(1, true)];
        run(&mut compass, &mut fake, now + Duration::from_millis(200));
        assert_eq!(
            compass.onboard.lines(),
            [
                "id:1 40% id:2 0% id:0 10% ",
                "id:0 x:12.3 y:-2.0 z:0.0 fit:3.3 MAG_CAL_SUCCESS",
                "id:1 x:12.3 y:-2.0 z:0.0 fit:3.3 MAG_CAL_SUCCESS"
            ]
        );
        assert_eq!(compass.onboard.bars, [100, 100, 0]);
        assert_eq!(compass.onboard.green, [true, true, false]);
        assert_eq!(compass.onboard.interval, TIMER_SLOW);
        assert!(compass.onboard.timer, "compass 2 has not saved");

        fake.log.reports.push(report(2, true));
        run(&mut compass, &mut fake, now + Duration::from_millis(1300));
        assert!(!compass.onboard.timer);
        assert!(!compass.onboard.accept_enabled && !compass.onboard.cancel_enabled);
        assert_eq!(dialog_text(&compass).as_deref(), Some(PLEASE_REBOOT));
    }

    /// A Start that times out says so and leaves the buttons as they were; one the vehicle refuses
    /// carries on, as `doCommand`'s false does.
    #[test]
    fn a_start_that_times_out_says_so() {
        let mut compass = priority(&sitl());
        let mut fake = Fake::answering(&[("start", TIMED_OUT)]);
        compass.click_start(true);
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(
            dialog_text(&compass).as_deref(),
            Some(format!("{START_FAILED}{COMMAND_TIMEOUT}").as_str())
        );
        assert!(!compass.onboard.accept_enabled);
        assert!(!compass.onboard.timer);
        assert_eq!(fake.cleared, 0);

        let mut compass = priority(&sitl());
        let mut fake = Fake::answering(&[("start", REFUSED)]);
        compass.click_start(true);
        run(&mut compass, &mut fake, Instant::now());
        assert!(compass.onboard.timer);
        assert!(compass.dialog().is_none());
    }

    /// Cancel sends its command and stops the timer, and leaves Accept and Cancel enabled as the
    /// C# does; a timeout says `doCommand`'s exception. Neither is clickable before a Start.
    #[test]
    fn cancel_stops_the_timer() {
        let mut compass = priority(&sitl());
        let mut fake = Fake::answering(&[("cancel", TIMED_OUT)]);
        compass.click_cancel();
        compass.click_accept();
        assert_eq!(compass.pending(), 0);
        compass.click_start(true);
        run(&mut compass, &mut fake, Instant::now());
        compass.click_cancel();
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(fake.names(), ["start", "cancel"]);
        assert!(!compass.onboard.timer);
        assert!(compass.onboard.cancel_enabled && compass.onboard.accept_enabled);
        assert_eq!(dialog_text(&compass).as_deref(), Some(COMMAND_TIMEOUT));
        assert_eq!(
            compass.last_command,
            Some(Ended {
                what: "DO_CANCEL_MAG_CAL".to_owned(),
                answer: Answer::Threw
            })
        );

        compass.answer(true, &mut fake);
        compass.click_accept();
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(fake.names(), ["start", "cancel", "accept"]);
    }

    /// Start with a reboot required asks first: No starts nothing; Yes reboots, says
    /// `CheckReboot`'s message, and starts once it is dismissed.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:154-182, 264-269`
    #[test]
    fn start_with_a_reboot_required_asks_first() {
        let mut compass = priority(&sitl());
        let mut fake = Fake::default();
        compass.move_row(1, true);
        run(&mut compass, &mut fake, Instant::now());
        fake.made.clear();

        compass.click_start(true);
        assert_eq!(dialog_text(&compass).as_deref(), Some(REBOOT_REQUIRED));
        compass.answer(false, &mut fake);
        run(&mut compass, &mut fake, Instant::now());
        assert!(fake.made.is_empty());
        assert!(compass.reboot_required);

        compass.click_start(true);
        compass.answer(true, &mut fake);
        assert_eq!(fake.reboots, 1);
        // The divergence at REBOOT_FAILED: the reboot went out, so no "failed" message stands
        // between the answer and the start the question interrupted.
        assert!(compass.dialog().is_none());
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(fake.names(), ["start"]);

        // With the link closed `CheckReboot` returns true without asking.
        let mut compass = priority(&sitl());
        compass.move_row(1, true);
        run(&mut compass, &mut fake, Instant::now());
        compass.click_start(false);
        assert!(compass.dialog().is_none());
        assert_eq!(compass.pending(), 1);
    }

    /// Reboot asks "Reboot?"; OK reboots, Cancel - the box's close button - does not.
    #[test]
    fn reboot_asks_first() {
        let mut compass = priority(&sitl());
        let mut fake = Fake::default();
        compass.click_reboot();
        assert_eq!(dialog_text(&compass).as_deref(), Some(REBOOT_QUESTION));
        compass.answer(false, &mut fake);
        assert_eq!(fake.reboots, 0);
        compass.click_reboot();
        compass.answer(true, &mut fake);
        assert_eq!(fake.reboots, 1);
    }

    /// Large Vehicle MagCal asks for the heading, starting at 0; OK sends it and says Completed or
    /// `CommandFailed`; Cancel, or a heading that does not parse, sends nothing.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:476-498`
    #[test]
    fn large_vehicle_magcal_sends_the_typed_heading() {
        let mut compass = priority(&sitl());
        let mut fake = Fake::default();
        compass.click_large_magcal();
        let Some(Dialog::MagCalYaw(field)) = compass.dialog() else {
            panic!("the InputBox");
        };
        assert_eq!(field.value(), "0");
        assert_eq!(
            compass.dialog().map(Dialog::words).unwrap().0,
            MAGCAL_YAW_TITLE
        );
        if let Some(Dialog::MagCalYaw(field)) = compass.dialogs.front_mut() {
            field.set("272.5");
        }
        compass.answer(true, &mut fake);
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(fake.names(), ["yaw 272.5"]);
        assert_eq!(
            dialog_text(&compass).as_deref(),
            Some(crate::fly::strings::COMPLETED)
        );
        compass.answer(true, &mut fake);

        let mut fake = Fake::answering(&[("yaw", REFUSED)]);
        compass.click_large_magcal();
        compass.answer(true, &mut fake);
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(
            dialog_text(&compass).as_deref(),
            Some(crate::fly::strings::COMMAND_FAILED)
        );
        compass.answer(true, &mut fake);

        let mut fake = Fake::default();
        compass.click_large_magcal();
        compass.answer(false, &mut fake);
        compass.click_large_magcal();
        if let Some(Dialog::MagCalYaw(field)) = compass.dialogs.front_mut() {
            field.set("north");
        }
        compass.answer(true, &mut fake);
        run(&mut compass, &mut fake, Instant::now());
        assert!(fake.made.is_empty());
        assert!(compass.dialog().is_none());
    }

    /// The instance lives as long as SETUP: another page and back keeps `rebootrequired`;
    /// leaving SETUP disposes it, keeping a box it was showing.
    #[test]
    fn leaving_setup_disposes_the_page() {
        let mut compass = priority(&sitl());
        let mut fake = Fake::default();
        compass.move_row(1, true);
        run(&mut compass, &mut fake, Instant::now());
        compass.deactivate(true);
        let view = TelemetryView::disconnected("test");
        compass.tick(&mut fake, &view, true, [false; 2], Instant::now());
        assert!(compass.reboot_required, "another page of the same screen");

        compass.tick(&mut fake, &view, false, [false; 2], Instant::now());
        assert!(!compass.reboot_required);
        assert!(compass.rows.is_empty());
        assert_eq!(dialog_text(&compass).as_deref(), Some(REBOOT_REQUIRED));
    }

    // --- The older page ----------------------------------------------------------------------------

    /// An ArduPilot 4.0 copter: no priorities, three compasses, the onboard calibration.
    fn copter_40() -> Vec<(String, f64)> {
        params(&[
            ("COMPASS_AUTODEC", 0.0),
            ("COMPASS_CAL_FIT", 16.0),
            ("COMPASS_DEC", -0.218_166_16),
            ("COMPASS_EXTERN2", 0.0),
            ("COMPASS_EXTERN3", 0.0),
            ("COMPASS_EXTERNAL", 1.0),
            ("COMPASS_LEARN", 1.0),
            ("COMPASS_MOT_X", 0.0),
            ("COMPASS_MOT_Y", 0.0),
            ("COMPASS_MOT_Z", 0.0),
            ("COMPASS_MOT2_X", 0.0),
            ("COMPASS_MOT2_Y", 0.0),
            ("COMPASS_MOT2_Z", 0.0),
            ("COMPASS_OFS_X", 5.0),
            ("COMPASS_OFS_Y", 13.0),
            ("COMPASS_OFS_Z", -18.0),
            ("COMPASS_OFS2_X", 450.0),
            ("COMPASS_OFS2_Y", 0.0),
            ("COMPASS_OFS2_Z", 0.0),
            ("COMPASS_OFS3_X", 0.0),
            ("COMPASS_OFS3_Y", 0.0),
            ("COMPASS_OFS3_Z", 0.0),
            ("COMPASS_ORIENT", 8.0),
            ("COMPASS_ORIENT2", 0.0),
            ("COMPASS_ORIENT3", 0.0),
            ("COMPASS_PRIMARY", 1.0),
            ("COMPASS_USE", 1.0),
            ("COMPASS_USE2", 1.0),
            ("COMPASS_USE3", 0.0),
        ])
    }

    fn legacy_page(parameters: &[(String, f64)], info: VehicleInfo) -> Compass<usize> {
        let mut compass = Compass::default();
        compass.activate(Class::Legacy, parameters, key(), info, bundled);
        compass
    }

    fn copter_40_info() -> VehicleInfo {
        VehicleInfo {
            version: (4, 0, 7),
            ..info()
        }
    }

    /// `ConfigHWCompass.Activate` on a 4.0 copter: the quick buttons hidden (a copter past 3.2.1),
    /// the onboard group shown and Mission Planner's hidden (it can calibrate onboard), the
    /// declination in degrees and minutes, each compass's boxes, orientation, offsets coloured by
    /// their size and MOT, and the primary.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:31-243`
    #[test]
    fn the_older_page_reads_a_40_copter() {
        let compass = legacy_page(&copter_40(), copter_40_info());
        let legacy = compass.legacy;
        assert!(legacy.enabled);
        assert!(!legacy.startup);
        assert!(!legacy.quick_visible);
        assert!(legacy.onboard_visible && !legacy.or_visible && !legacy.mpcalib_visible);
        assert!(!legacy.autodec && legacy.declination_enabled());
        // -0.21816616 rad is -12.5 degrees: `(int)` -12, and -30 minutes.
        assert_eq!(legacy.degrees.value(), "-12");
        assert_eq!(legacy.minutes.value(), "-30");
        assert_eq!(compass.learn.state, CheckState::Checked);

        let [first, second, third] = &legacy.compasses;
        assert!(first.visible && second.visible && third.visible);
        assert_eq!(first.use_check.state, CheckState::Checked);
        assert_eq!(first.external.state, CheckState::Checked);
        assert_eq!(first.orient.text(), "Roll180");
        assert_eq!(
            first.offsets,
            Some((
                "OFFSETS  X: 5,   Y: 13,   Z: -18".to_owned(),
                OffsetColour::Green
            ))
        );
        assert_eq!(
            first.mot.as_deref(),
            Some("MOT          X: 0,   Y: 0,   Z: 0")
        );
        assert_eq!(
            second.offsets.as_ref().map(|(_, colour)| *colour),
            Some(OffsetColour::Yellow)
        );
        assert_eq!(
            third.offsets.as_ref().map(|(_, colour)| *colour),
            Some(OffsetColour::Red)
        );
        assert_eq!(third.mot, None, "no COMPASS_MOT3_X");
        assert_eq!(legacy.primary, Some(1));
        assert!(legacy.primary_visible && legacy.primary_enabled);
        assert_eq!(compass.fitness.selected, Some(16));
    }

    /// Without the link the page is disabled and read no further; without `COMPASS_OFS_X` it is
    /// disabled after the first compass, with `startup` left set; a vehicle without
    /// `COMPASS_EXTERN2` hides the second group. A plane from 3.7.1 shows every group, and one
    /// that cannot calibrate onboard shows only Mission Planner's.
    #[test]
    fn the_older_page_stops_where_activate_returns() {
        let closed = legacy_page(
            &copter_40(),
            VehicleInfo {
                open: false,
                ..copter_40_info()
            },
        );
        assert!(!closed.legacy.enabled);

        let mut parameters = copter_40();
        parameters.retain(|(name, _)| name != "COMPASS_OFS_X" && name != "COMPASS_EXTERN2");
        let compass = legacy_page(&parameters, copter_40_info());
        assert!(!compass.legacy.enabled);
        assert!(compass.legacy.startup);
        assert_eq!(compass.legacy.compasses[0].offsets, None);

        let mut parameters = copter_40();
        parameters.retain(|(name, _)| name != "COMPASS_EXTERN2");
        let compass = legacy_page(&parameters, copter_40_info());
        assert!(!compass.legacy.compasses[1].visible);
        assert!(compass.legacy.compasses[2].visible);

        let plane = legacy_page(
            &copter_40(),
            VehicleInfo {
                firmware: Firmware::ArduPlane,
                version: (4, 0, 9),
                ..copter_40_info()
            },
        );
        let legacy = plane.legacy;
        assert!(legacy.quick_visible);
        assert!(legacy.onboard_visible && legacy.or_visible && legacy.mpcalib_visible);

        let no_onboard = legacy_page(
            &copter_40(),
            VehicleInfo {
                capabilities: 0,
                ..copter_40_info()
            },
        );
        let legacy = no_onboard.legacy;
        assert!(!legacy.onboard_visible && !legacy.or_visible && legacy.mpcalib_visible);
    }

    /// Learn on the older page writes twice: its own handler's `setParam`, then the box's, which
    /// finds the value held.
    #[test]
    fn learn_on_the_older_page_is_written_by_both_handlers() {
        let mut compass = legacy_page(&copter_40(), copter_40_info());
        let mut fake = Fake::default();
        compass.click_check(3);
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(fake.names(), ["COMPASS_LEARN=0", "COMPASS_LEARN=0"]);
    }

    /// Leaving a declination box writes degrees and minutes as radians, the minutes taken away
    /// from a negative angle; text that does not parse says `InvalidNumberEntered`.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:283-322`
    #[test]
    fn a_declination_box_left_writes_the_declination() {
        let mut compass = legacy_page(&copter_40(), copter_40_info());
        let mut fake = Fake::default();
        let view = TelemetryView {
            parameters: copter_40().into(),
            ..TelemetryView::disconnected("test")
        };
        compass.legacy.minutes.set("15");
        compass.tick(&mut fake, &view, true, [false, true], Instant::now());
        assert!(fake.made.is_empty(), "still in the box");
        compass.tick(&mut fake, &view, true, [false, false], Instant::now());
        let [(written, _)] = fake.made.as_slice() else {
            panic!("one write: {:?}", fake.names());
        };
        let value: f64 = written
            .strip_prefix("COMPASS_DEC=")
            .and_then(|value| value.parse().ok())
            .unwrap();
        let expected = f64::from(-12.0_f32 - 15.0_f32 / 60.0) * DEG2RAD;
        assert!((value - expected).abs() < 1e-12, "{value} vs {expected}");

        compass.legacy.degrees.set("west");
        compass.tick(&mut fake, &view, true, [true, false], Instant::now());
        compass.tick(&mut fake, &view, true, [false, false], Instant::now());
        assert_eq!(dialog_text(&compass).as_deref(), Some(INVALID_NUMBER));
    }

    /// Obtain declination automatically: the boxes follow it and it is written.
    #[test]
    fn autodec_writes_and_disables_the_boxes() {
        let mut compass = legacy_page(&copter_40(), copter_40_info());
        let mut fake = Fake::default();
        compass.click_autodec(&copter_40());
        assert!(!compass.legacy.declination_enabled());
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(fake.names(), ["COMPASS_AUTODEC=1"]);
    }

    /// APM2.5: the first orientation to none - written, since it held Roll180 - then its eight
    /// writes and `Activate`; a throw part way says `ErrorSettingParameter`, skips the rest, and
    /// still runs `Activate`.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:654-679`
    #[test]
    fn a_quick_configure_button_writes_its_set() {
        let mut compass = legacy_page(&copter_40(), copter_40_info());
        let mut fake = Fake {
            params: copter_40(),
            ..Fake::default()
        };
        compass.click_quick(1, true);
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(
            fake.names(),
            [
                "COMPASS_ORIENT=0",
                "COMPASS_USE1=1",
                "COMPASS_USE2=0",
                "COMPASS_USE3=0",
                "COMPASS_EXTERNAL=0",
                "COMPASS_EXTERN2=0",
                "COMPASS_EXTERN3=0",
                "COMPASS_PRIMARY=0",
                "COMPASS_LEARN=1"
            ]
        );
        // `Activate()` read the parameters again: the orientation is the vehicle's once more.
        assert_eq!(compass.legacy.compasses[0].orient.text(), "Roll180");

        let mut compass = legacy_page(&copter_40(), copter_40_info());
        let mut fake = Fake {
            params: copter_40(),
            ..Fake::answering(&[("COMPASS_USE3", TIMED_OUT)])
        };
        compass.click_quick(2, true);
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(
            fake.names(),
            [
                "COMPASS_EXTERNAL=1",
                "COMPASS_EXTERN2=0",
                "COMPASS_EXTERN3=0",
                "COMPASS_USE1=1",
                "COMPASS_USE2=0",
                "COMPASS_USE3=0"
            ],
            "Roll180 already held, so the orientation is not written"
        );
        assert_eq!(
            dialog_text(&compass).as_deref(),
            Some(ERROR_SETTING_PARAMETER)
        );
    }

    /// Pixhawk/PX4 writes its set, asks about the firmware, and on No sets Roll180 and the first
    /// compass internal.
    #[test]
    fn pixhawk_asks_about_the_firmware() {
        let mut parameters = copter_40();
        for (name, value) in &mut parameters {
            if name == "COMPASS_ORIENT" {
                *value = 0.0;
            }
        }
        let mut compass = legacy_page(&parameters, copter_40_info());
        let mut fake = Fake {
            params: parameters,
            ..Fake::default()
        };
        compass.click_quick(0, true);
        run(&mut compass, &mut fake, Instant::now());
        assert_eq!(fake.made.len(), 8);
        assert_eq!(dialog_text(&compass).as_deref(), Some(QUICK_FIRMWARE));
        compass.answer(false, &mut fake);
        run(&mut compass, &mut fake, Instant::now());
        let names = fake.names();
        assert_eq!(
            names.get(8..),
            Some(&["COMPASS_ORIENT=8", "COMPASS_EXTERNAL=0"][..])
        );
        assert!(compass.dialog().is_none());

        compass.click_quick(0, false);
        assert_eq!(
            dialog_text(&compass).as_deref(),
            Some("You are not connected.")
        );
    }

    // --- Through the real link ---------------------------------------------------------------------

    use crate::telemetry::scripted::{Vehicle, ack, param, until};
    use mp_mavlink_dialects::all::{MagCalProgress, MavMessage, ParamValue};

    /// `MAV_PARAM_TYPE_INT8`, as ArduPilot declares `COMPASS_USE`.
    const INT8: u8 = 1;
    /// `MAV_RESULT_ACCEPTED`.
    const ACCEPTED: u8 = 0;

    /// The page on the real link, to a scripted autopilot: a Use box written and echoed, then
    /// Start with the C#'s parameters, the vehicle's progress in the text box, and Cancel with
    /// param3 = 1 - the path the SETUP screen takes, but for the drawing.
    #[test]
    fn the_page_talks_to_the_vehicle_through_the_link() {
        let (mut telemetry, mut vehicle) =
            Vehicle::connect(mp_link::ProtocolTimeouts::default().faster(20));
        vehicle.send(&param("COMPASS_USE", 1.0, INT8));
        until("COMPASS_USE to be listed", || {
            telemetry.holds_parameter("COMPASS_USE")
        });
        let mut compass: Compass = Compass::default();
        let view = telemetry.view();
        compass.activate(Class::Priority, &sitl(), Key::of(&view), info(), bundled);

        // The echo, as ArduPilot answers a PARAM_SET.
        let echo = |vehicle: &mut Vehicle| {
            for message in vehicle.read() {
                match message {
                    MavMessage::ParamSet(set) => {
                        vehicle.send(&MavMessage::ParamValue(ParamValue {
                            param_value: set.param_value,
                            param_count: 1,
                            param_index: 0,
                            param_id: set.param_id,
                            param_type: INT8,
                        }));
                    }
                    MavMessage::CommandLong(long) => {
                        vehicle.send(&ack(long.command, ACCEPTED));
                    }
                    _ => {}
                }
            }
        };

        compass.click_check(0);
        until("the write to be answered", || {
            echo(&mut vehicle);
            compass.tick(&mut telemetry, &view, true, [false; 2], Instant::now());
            compass.pending() == 0
        });
        assert_eq!(
            compass.last_write,
            Some(Ended {
                what: "COMPASS_USE".to_owned(),
                answer: Answer::True
            })
        );
        let set = vehicle.heard.iter().find_map(|message| match message {
            MavMessage::ParamSet(set) => Some(set.param_value),
            _ => None,
        });
        assert_eq!(set, Some(0.0));

        compass.click_start(true);
        until("Start to be answered", || {
            echo(&mut vehicle);
            compass.tick(&mut telemetry, &view, true, [false; 2], Instant::now());
            compass.pending() == 0
        });
        let start = vehicle.heard.iter().find_map(|message| match message {
            MavMessage::CommandLong(long)
                if long.command == mp_calibration::CMD_DO_START_MAG_CAL =>
            {
                Some([long.param1, long.param2, long.param3])
            }
            _ => None,
        });
        assert_eq!(start, Some([0.0, 1.0, 1.0]));
        assert!(compass.onboard.timer);

        vehicle.send(&MavMessage::MagCalProgress(MagCalProgress {
            direction_x: 0.0,
            direction_y: 0.0,
            direction_z: 0.0,
            compass_id: 0,
            cal_mask: 7,
            cal_status: 2,
            attempt: 1,
            completion_pct: 25,
            completion_mask: [0; 10],
        }));
        until("the progress to reach the page", || {
            compass.tick(
                &mut telemetry,
                &view,
                true,
                [false; 2],
                Instant::now() + Duration::from_secs(1),
            );
            compass.onboard.text.starts_with("id:0 25% ")
        });
        assert_eq!(compass.onboard.bars[0], 25);

        compass.click_cancel();
        until("Cancel to be answered", || {
            echo(&mut vehicle);
            compass.tick(&mut telemetry, &view, true, [false; 2], Instant::now());
            compass.pending() == 0
        });
        let cancel = vehicle.heard.iter().find_map(|message| match message {
            MavMessage::CommandLong(long)
                if long.command == mp_calibration::CMD_DO_CANCEL_MAG_CAL =>
            {
                Some([long.param1, long.param2, long.param3])
            }
            _ => None,
        });
        assert_eq!(cancel, Some([0.0, 0.0, 1.0]));
        assert!(!compass.onboard.timer);
        assert_eq!(
            compass.last_command,
            Some(Ended {
                what: "DO_CANCEL_MAG_CAL".to_owned(),
                answer: Answer::True
            })
        );
    }

    /// The SETUP screen draws this module's page for both entries, and activates and deactivates
    /// it as the list chooses them.
    #[test]
    fn the_setup_screen_draws_and_activates_the_page() {
        let source = include_str!("../setup.rs");
        assert!(source.contains("\"ConfigHWCompass2\" | \"ConfigHWCompass\" => column()"));
        assert!(source.contains("crate::config::compass::page("));
        assert!(source.contains("self.compass.activate("));
        assert!(source.contains("self.compass.deactivate(open)"));
        let main = include_str!("../main.rs");
        assert!(main.contains("config::compass::record_facts(&self.compass, &view)"));
        assert!(main.contains("config::compass::overlay("));
        assert!(main.contains("self.compass.tick("));
    }

    /// Log Calibration is dead C#: `ConfigHWCompass.cs` holds `BUT_MagCalibrationLog_Click`, but
    /// neither page's Designer makes the button or wires the handler, so it is recorded, not
    /// ported (PLAN.md §12 D16), and neither page draws it. Mission Planner's group holds Live
    /// Calibration and the link, as this page draws it. If the C# ever wires the button, this
    /// fails and the button is to be ported.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:362-373;
    /// ConfigHWCompass.Designer.cs:266-279`
    #[test]
    fn log_calibration_has_no_button_to_port() {
        use crate::config_coverage::source::csharp;
        // No line of the page's code - comments and these tests aside - names the button, its
        // text or its handler.
        let names = [
            ["Log", "Calibration"].join(" "),
            ["MagCalibration", "Log"].concat(),
            ["log", "calibration"].join("-"),
        ];
        let (page, _) = include_str!("compass.rs")
            .split_once(concat!("#[cfg(test)]\n", "mod tests {"))
            .expect("the tests");
        for line in page.lines() {
            let code = line.trim_start();
            if code.starts_with("//") {
                continue;
            }
            for name in &names {
                assert!(!code.contains(name.as_str()), "{line}");
            }
        }
        let dir = "GCSViews/ConfigurationView";
        let (Some(cs), Some(designer), Some(resx), Some(designer2), Some(cs2)) = (
            csharp(&format!("{dir}/ConfigHWCompass.cs")),
            csharp(&format!("{dir}/ConfigHWCompass.Designer.cs")),
            csharp(&format!("{dir}/ConfigHWCompass.resx")),
            csharp(&format!("{dir}/ConfigHWCompass2.Designer.cs")),
            csharp(&format!("{dir}/ConfigHWCompass2.cs")),
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        assert!(cs.contains("private async void BUT_MagCalibrationLog_Click("));
        assert!(cs.contains("await MagCalib.ProcessLog(ans)"));
        for (name, text) in [
            ("ConfigHWCompass.Designer.cs", &designer),
            ("ConfigHWCompass.resx", &resx),
            ("ConfigHWCompass2.Designer.cs", &designer2),
            ("ConfigHWCompass2.cs", &cs2),
        ] {
            assert!(
                !text.contains("MagCalibrationLog"),
                "{name} makes the button"
            );
            assert!(!text.contains("ProcessLog"), "{name} calls ProcessLog");
        }
        assert!(
            designer.contains("this.groupBoxmpcalib.Controls.Add(this.BUT_MagCalibrationLive);")
        );
        assert_eq!(
            designer
                .matches("this.groupBoxmpcalib.Controls.Add(")
                .count(),
            2
        );
    }
}
