//! The SETUP and CONFIG screens: Mission Planner's `InitialSetup` and `SoftwareConfig`.
//!
//! Each is a `BackstageView` (`ExtLibs/Controls/BackstageView/BackstageView.cs`): a list of pages
//! down the left, `WidthMenu` 172 pixels wide, and the chosen page filling the rest, one at a
//! time (`// C#: GCSViews/InitialSetup.Designer.cs:72; GCSViews/SoftwareConfig.Designer.cs:41;
//! ExtLibs/Controls/BackstageView/BackstageView.Designer.cs:35-52`). The screen's `Load` handler
//! adds the pages, each under the condition around its `AddBackstageViewPage` call - connected,
//! every parameter in, the vehicle type - so the list is built when the screen is shown, and
//! built again when `MainV2` reloads the screen: on a connect, a disconnect, a change of vehicle,
//! and when the Loading page sees the last parameter arrive.
//!
//! [`SETUP_LIST`] and [`CONFIG_LIST`] are those calls, one line each, in the C#'s order, with the
//! conditions as code. The titles and headings come from `config_coverage.rs`, whose tests hold
//! them to the `.resx`; this module's tests hold each condition to the `if`s around its call and
//! the argument it passes. A page this application has is one arm of `page_body`; a page it
//! lacks shows its class and "not ported", so the list is the C#'s whole.
//!
//! The panels this screen used to stack in one column went to the pages they stand for: the
//! accelerometer panel and Calibrate Level to `ConfigAccelerometerCalibration`, compass to
//! `ConfigHWCompass` and `ConfigHWCompass2`, radio to `ConfigRadioInput`, motors to
//! `ConfigMotorTest`, the joystick to `JoystickSetup`. Four stand for nothing in either list - the
//! vehicle's identity, the estimator, the dataflash logs and the ground-pressure calibration -
//! and fill the page area until a page is chosen, where the C# leaves it empty. Their homes in
//! the C# are on the flight screen (the DataFlash Logs tab's `BUT_DFMavlink`, the HUD's EKF and
//! vibration windows, the Actions tab's `Preflight_Calibration`), which this module does not draw.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, FontWeight, SharedString, Window, div, prelude::*, px, rgb};
use mp_vehicle::VehicleId;

use crate::MissionPlanner;
use crate::config::flight_modes::{Firmware, firmware_of};
pub use crate::config_coverage::Screen as List;
use crate::telemetry::TelemetryView;
use crate::ui::{action_sized, panel, theme};

/// `WidthMenu`: the list's width.
/// `// C#: GCSViews/InitialSetup.Designer.cs:72; GCSViews/SoftwareConfig.Designer.cs:41`
const MENU_WIDTH: f32 = 172.0;

/// `ButtonHeight`: each entry's height.
/// `// C#: ExtLibs/Controls/BackstageView/BackstageView.cs:32`
const BUTTON_HEIGHT: f32 = 30.0;

/// The width the ported pages are laid out in, as the setup column was.
const PAGE_WIDTH: f32 = 760.0;

/// `MAV_PROTOCOL_CAPABILITY_FTP`.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:7076`
const CAPABILITY_FTP: u32 = 32;

/// `MAV_TYPE_HELICOPTER`, `isHeli`'s `aptype`.
const TYPE_HELICOPTER: u8 = 4;

// ---- Display view (row 71) ----
/// A display-view switch: `MainV2.DisplayConfiguration.<flag>`, of the view chosen - Advanced
/// unless the Planner page's Layout or the saved `displayview` says otherwise
/// (`display_view.rs`). One the class does not have is off.
/// `// C#: ExtLibs/Utilities/DisplayView.cs:37-455; MainV2.cs:351-366`
pub(crate) fn display(flag: &str) -> bool {
    crate::display_view::flag(flag)
}
// ---- end Display view ----

/// What the lists' conditions read: the C#'s `isConnected`, `gotAllParams` and the rest.
#[derive(Debug, Clone, Copy)]
pub struct Vehicle<'a> {
    /// `isConnected`, `MainV2.comPort.BaseStream.IsOpen`. Mission Planner's connect waits for a
    /// heartbeat and closes the port without one, so an open port is a vehicle heard from.
    pub connected: bool,
    /// `gotAllParams`: `TotalReceived >= TotalReported`. And, while connected, a table that is not
    /// empty, as `failsafe::available` has it: Mission Planner downloads the parameters as part of
    /// connecting, so a connected vehicle with none has them still to come; here they are
    /// downloaded when asked for, and until then the list is the one the C# shows while they
    /// arrive.
    pub got_all_params: bool,
    /// `cs.firmware`, as `setAPType` sets it; `ArduCopter2`, its initial value, before a vehicle.
    pub firmware: Firmware,
    /// `MAV.aptype`, the heartbeat's `MAV_TYPE`.
    pub mav_type: u8,
    /// `cs.version`'s major and minor, from `AUTOPILOT_VERSION`. Mission Planner asks for that
    /// message on connecting and this application's link does not, so until one arrives the
    /// version the firmware's banner names stands in for it.
    pub version: (u8, u8),
    /// `cs.capabilities`, from `AUTOPILOT_VERSION`.
    pub capabilities: u32,
    /// `MAV.param`.
    pub parameters: &'a [(String, f64)],
}

impl<'a> Vehicle<'a> {
    /// The vehicle a view shows.
    pub fn of(view: &'a TelemetryView, banner: Option<&str>) -> Self {
        let connected = view.connected && view.vehicle.is_some();
        let state = view.state.as_deref();
        let reported = state
            .map(|state| state.autopilot_info.version)
            .filter(|version| version[0] != 0 || version[1] != 0)
            .map(|version| (version[0], version[1]));
        Self {
            connected,
            got_all_params: !(connected && view.parameters.is_empty())
                && view.parameters.len() >= usize::from(view.parameters_expected),
            firmware: state.map_or(Firmware::ArduCopter2, |state| {
                firmware_of(state.autopilot, state.vehicle_type, banner)
            }),
            mav_type: state.map_or(0, |state| state.vehicle_type),
            version: reported
                .or_else(|| banner.and_then(banner_version))
                .unwrap_or((0, 0)),
            capabilities: state.map_or(0, |state| state.autopilot_info.capabilities),
            parameters: &view.parameters,
        }
    }

    /// `MAV.param.ContainsKey(name)`.
    fn has(&self, name: &str) -> bool {
        self.parameters.iter().any(|(held, _)| held == name)
    }

    /// `isCopter`.
    /// `// C#: GCSViews/InitialSetup.cs:79-82`
    fn is_copter(&self) -> bool {
        self.connected && self.firmware == Firmware::ArduCopter2
    }

    /// `isCopter35plus`: the version alone, connected or not.
    /// `// C#: GCSViews/InitialSetup.cs:84-87`
    fn is_copter_35_plus(&self) -> bool {
        self.version >= (3, 5)
    }

    /// `isHeli`.
    /// `// C#: GCSViews/InitialSetup.cs:89-92`
    fn is_heli(&self) -> bool {
        self.connected && self.mav_type == TYPE_HELICOPTER
    }

    /// `isPlane`: ArduPlane or Ateryx.
    /// `// C#: GCSViews/InitialSetup.cs:104-112`
    fn is_plane(&self) -> bool {
        self.connected && matches!(self.firmware, Firmware::ArduPlane | Firmware::Ateryx)
    }

    /// `isQuadPlane`: a plane with `Q_ENABLE` 1.
    /// `// C#: GCSViews/InitialSetup.cs:94-102`
    fn is_quadplane(&self) -> bool {
        self.connected
            && self.is_plane()
            && self
                .parameters
                .iter()
                .any(|(name, value)| name == "Q_ENABLE" && (*value - 1.0).abs() < f64::EPSILON)
    }

    /// `isTracker`.
    /// `// C#: GCSViews/InitialSetup.cs:74-77`
    fn is_tracker(&self) -> bool {
        self.connected && self.firmware == Firmware::ArduTracker
    }
}

/// The version a firmware banner names: `ArduCopter V4.5.7 (...)` is 4.5.
fn banner_version(banner: &str) -> Option<(u8, u8)> {
    let (_, rest) = banner.split_once(" V")?;
    let mut numbers = rest.split(|c: char| !c.is_ascii_digit());
    let major = numbers.next()?.parse().ok()?;
    let minor = numbers.next()?.parse().ok()?;
    Some((major, minor))
}

/// A condition of an `if` around an `AddBackstageViewPage` call; an `else` is the negation of its
/// `if`. The tests hold each to the C#'s text.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Guard {
    /// `MainV2.DisplayConfiguration.<flag>`.
    Display(&'static str),
    /// `gotAllParams`.
    GotAllParams,
    /// `!gotAllParams`.
    NotGotAllParams,
    /// `MainV2.comPort.BaseStream.IsOpen`.
    Open,
    /// Its `else`.
    NotOpen,
    /// `MainV2.comPort.MAV.param.ContainsKey("COMPASS_PRIO1_ID")`.
    CompassPriorities,
    /// Its `else`.
    NoCompassPriorities,
    /// `(isCopter || isQuadPlane) && MainV2.DisplayConfiguration.displayInitialParams`.
    InitialParams,
    /// `MainV2.comPort.MAV.cs.firmware == Firmwares.<firmware>`.
    FirmwareIs(Firmware),
    /// `!Program.MONO && ConfigOSD.IsApplicable() && MainV2.DisplayConfiguration.displayOSD`.
    /// Not Mono: this is not a .NET runtime, and Windows is the platform the C# is written for.
    /// `IsApplicable` is any parameter whose name starts `OSD`.
    /// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:207-217`
    OnboardOsd,
    /// `(MainV2.comPort.MAV.cs.capabilities & (int)MAVLink.MAV_PROTOCOL_CAPABILITY.FTP) > 0`.
    Ftp,
    /// `!MainV2.comPort.BaseStream.IsOpen || gotAllParams`.
    ParamList,
    /// `start == null`: no page before it has been made the one to open first.
    NoStart,
    /// Its `else`.
    Start,
}

impl Guard {
    /// Whether it holds; `start` is whether a page before has been kept as `start`.
    fn holds(self, vehicle: &Vehicle, start: bool) -> bool {
        match self {
            Self::Display(flag) => display(flag),
            Self::GotAllParams => vehicle.got_all_params,
            Self::NotGotAllParams => !vehicle.got_all_params,
            Self::Open => vehicle.connected,
            Self::NotOpen => !vehicle.connected,
            Self::CompassPriorities => vehicle.has("COMPASS_PRIO1_ID"),
            Self::NoCompassPriorities => !vehicle.has("COMPASS_PRIO1_ID"),
            Self::InitialParams => {
                (vehicle.is_copter() || vehicle.is_quadplane()) && display("displayInitialParams")
            }
            Self::FirmwareIs(firmware) => vehicle.firmware == firmware,
            Self::OnboardOsd => {
                vehicle.parameters.iter().any(|(name, _)| {
                    name.get(..3)
                        .is_some_and(|start| start.eq_ignore_ascii_case("OSD"))
                }) && display("displayOSD")
            }
            Self::Ftp => vehicle.capabilities & CAPABILITY_FTP > 0,
            Self::ParamList => !vehicle.connected || vehicle.got_all_params,
            Self::NoStart => !start,
            Self::Start => start,
        }
    }

    /// The C#'s text for it.
    #[cfg(test)]
    fn csharp(self) -> String {
        match self {
            Self::Display(flag) => format!("MainV2.DisplayConfiguration.{flag}"),
            Self::GotAllParams => "gotAllParams".to_owned(),
            Self::NotGotAllParams => "!gotAllParams".to_owned(),
            Self::Open => "MainV2.comPort.BaseStream.IsOpen".to_owned(),
            Self::NotOpen => "!(MainV2.comPort.BaseStream.IsOpen)".to_owned(),
            Self::CompassPriorities => {
                "MainV2.comPort.MAV.param.ContainsKey(\"COMPASS_PRIO1_ID\")".to_owned()
            }
            Self::NoCompassPriorities => {
                "!(MainV2.comPort.MAV.param.ContainsKey(\"COMPASS_PRIO1_ID\"))".to_owned()
            }
            Self::InitialParams => {
                "(isCopter || isQuadPlane) && MainV2.DisplayConfiguration.displayInitialParams"
                    .to_owned()
            }
            Self::FirmwareIs(firmware) => {
                format!(
                    "MainV2.comPort.MAV.cs.firmware == Firmwares.{}",
                    firmware.label()
                )
            }
            Self::OnboardOsd => "!Program.MONO && ConfigOSD.IsApplicable() && \
                                 MainV2.DisplayConfiguration.displayOSD"
                .to_owned(),
            Self::Ftp => "(MainV2.comPort.MAV.cs.capabilities & \
                          (int)MAVLink.MAV_PROTOCOL_CAPABILITY.FTP) > 0"
                .to_owned(),
            Self::ParamList => "!MainV2.comPort.BaseStream.IsOpen || gotAllParams".to_owned(),
            Self::NoStart => "start == null".to_owned(),
            Self::Start => "!(start == null)".to_owned(),
        }
    }
}

/// `InitialSetup.AddBackstageViewPage`'s `enabled` argument: the page is added only when it
/// holds. `SoftwareConfig`'s calls have none.
/// `// C#: GCSViews/InitialSetup.cs:134-149`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Enabled {
    /// `true`, or no argument.
    Always,
    /// `isConnected`.
    Connected,
    /// `isDisConnected`.
    Disconnected,
    /// `isConnected && gotAllParams`.
    ConnectedAllParams,
    /// `isHeli && gotAllParams`.
    HeliAllParams,
    /// `isCopter && gotAllParams && !isCopter35plus`.
    CopterBefore35,
    /// `MainV2.comPort.MAV.param.ContainsKey("FRAME_CLASS") || isCopter && gotAllParams &&
    /// isCopter35plus`.
    FrameClass,
    /// `isTracker`.
    Tracker,
}

impl Enabled {
    /// Whether it holds.
    fn holds(self, vehicle: &Vehicle) -> bool {
        match self {
            Self::Always => true,
            Self::Connected => vehicle.connected,
            Self::Disconnected => !vehicle.connected,
            Self::ConnectedAllParams => vehicle.connected && vehicle.got_all_params,
            Self::HeliAllParams => vehicle.is_heli() && vehicle.got_all_params,
            Self::CopterBefore35 => {
                vehicle.is_copter() && vehicle.got_all_params && !vehicle.is_copter_35_plus()
            }
            Self::FrameClass => {
                vehicle.has("FRAME_CLASS")
                    || vehicle.is_copter() && vehicle.got_all_params && vehicle.is_copter_35_plus()
            }
            Self::Tracker => vehicle.is_tracker(),
        }
    }

    /// The C#'s text for it.
    #[cfg(test)]
    const fn csharp(self) -> &'static str {
        match self {
            Self::Always => "true",
            Self::Connected => "isConnected",
            Self::Disconnected => "isDisConnected",
            Self::ConnectedAllParams => "isConnected && gotAllParams",
            Self::HeliAllParams => "isHeli && gotAllParams",
            Self::CopterBefore35 => "isCopter && gotAllParams && !isCopter35plus",
            Self::FrameClass => {
                "MainV2.comPort.MAV.param.ContainsKey(\"FRAME_CLASS\") || isCopter && \
                 gotAllParams && isCopter35plus"
            }
            Self::Tracker => "isTracker",
        }
    }
}

/// One `AddBackstageViewPage` call. Its title and heading are the ledger's, by line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Entry {
    /// The line of the call in the screen's source.
    pub line: u32,
    /// The page's C# class.
    pub class: &'static str,
    /// The `if`s around the call, outermost first.
    pub guards: &'static [Guard],
    /// `InitialSetup`'s `enabled` argument.
    pub enabled: Enabled,
    /// `SoftwareConfig`'s `advanced` argument: hidden outside the Advanced view.
    /// `// C#: ExtLibs/Controls/BackstageView/BackstageView.cs:284-288`
    pub advanced: bool,
    /// `SoftwareConfig`'s `start = AddBackstageViewPage(...)`: the page opened when no remembered
    /// one is listed.
    pub start: bool,
}

/// An `InitialSetup` call.
const fn page(line: u32, class: &'static str, guards: &'static [Guard], enabled: Enabled) -> Entry {
    Entry {
        line,
        class,
        guards,
        enabled,
        advanced: false,
        start: false,
    }
}

/// A `SoftwareConfig` call.
const fn tuning(line: u32, class: &'static str, guards: &'static [Guard]) -> Entry {
    page(line, class, guards, Enabled::Always)
}

/// A `SoftwareConfig` call kept as `start`.
const fn start(line: u32, class: &'static str, guards: &'static [Guard]) -> Entry {
    Entry {
        start: true,
        ..tuning(line, class, guards)
    }
}

/// A `SoftwareConfig` call added as advanced.
const fn advanced(line: u32, class: &'static str, guards: &'static [Guard]) -> Entry {
    Entry {
        advanced: true,
        ..tuning(line, class, guards)
    }
}

use Enabled::{
    Always, Connected, ConnectedAllParams, CopterBefore35, Disconnected, FrameClass, HeliAllParams,
    Tracker,
};
use Guard::{
    CompassPriorities, Display, FirmwareIs, Ftp, GotAllParams, InitialParams, NoCompassPriorities,
    NoStart, NotGotAllParams, NotOpen, OnboardOsd, Open, ParamList, Start,
};

/// `InitialSetup.HardwareConfig_Load`'s calls, in order, one line each.
/// `// C#: GCSViews/InitialSetup.cs:155-353`
#[rustfmt::skip]
pub const SETUP_LIST: &[Entry] = &[
    page(162, "ConfigParamLoading", &[NotGotAllParams, Open], Always),
    page(169, "ConfigFirmwareDisabled", &[Display("displayInstallFirmware")], Connected),
    page(171, "ConfigFirmwareManifest", &[Display("displayInstallFirmware")], Disconnected),
    page(173, "ConfigFirmware", &[Display("displayInstallFirmware")], Disconnected),
    page(178, "ConfigSecureAP", &[], Disconnected),
    page(182, "ConfigMandatory", &[], ConnectedAllParams),
    page(187, "ConfigTradHeli4", &[Display("displayFrameType")], HeliAllParams),
    page(188, "ConfigFrameType", &[Display("displayFrameType")], CopterBefore35),
    page(189, "ConfigFrameClassType", &[Display("displayFrameType")], FrameClass),
    page(196, "ConfigAccelerometerCalibration", &[Display("displayAccelCalibration")], ConnectedAllParams),
    page(203, "ConfigHWCompass2", &[Display("displayCompassConfiguration"), CompassPriorities], ConnectedAllParams),
    page(206, "ConfigHWCompass", &[Display("displayCompassConfiguration"), NoCompassPriorities], ConnectedAllParams),
    page(211, "ConfigRadioInput", &[Display("displayRadioCalibration")], ConnectedAllParams),
    page(215, "ConfigRadioOutput", &[Display("displayServoOutput")], ConnectedAllParams),
    page(220, "ConfigSerial", &[Display("displaySerialPorts")], ConnectedAllParams),
    page(224, "ConfigESCCalibration", &[Display("displayEscCalibration")], ConnectedAllParams),
    page(228, "ConfigFlightModes", &[Display("displayFlightModes")], ConnectedAllParams),
    page(232, "ConfigFailSafe", &[Display("displayFailSafe")], ConnectedAllParams),
    page(237, "ConfigInitialParams", &[InitialParams], ConnectedAllParams),
    page(241, "ConfigHWIDs", &[Display("displayHWIDs")], ConnectedAllParams),
    page(243, "ConfigOptional", &[], Always),
    page(251, "ConfigSerialInjectGPS", &[Display("displayRTKInject")], Always),
    page(254, "ConfigCubeID", &[], Connected),
    page(259, "Sikradio", &[Display("displaySikRadio")], Always),
    page(263, "ConfigADSB", &[Display("displayADSB")], ConnectedAllParams),
    page(266, "ConfigGPSOrder", &[Display("displayGPSOrder")], ConnectedAllParams),
    page(270, "ConfigBatteryMonitoring", &[Display("displayBattMonitor")], ConnectedAllParams),
    page(271, "ConfigBatteryMonitoring2", &[Display("displayBattMonitor")], ConnectedAllParams),
    page(276, "ConfigDroneCAN", &[Display("displayCAN")], Always),
    page(280, "JoystickSetup", &[Display("displayJoystick")], Always),
    page(285, "ConfigCompassMot", &[Display("displayCompassMotorCalib")], ConnectedAllParams),
    page(289, "ConfigHWRangeFinder", &[Display("displayRangeFinder")], ConnectedAllParams),
    page(293, "ConfigHWAirspeed", &[Display("displayAirSpeed")], ConnectedAllParams),
    page(297, "ConfigHWPX4Flow", &[Display("displayPx4Flow")], Always),
    page(301, "ConfigHWOptFlow", &[Display("displayOpticalFlow")], ConnectedAllParams),
    page(305, "ConfigHWOSD", &[Display("displayOsd")], ConnectedAllParams),
    page(309, "ConfigMount", &[Display("displayCameraGimbal")], ConnectedAllParams),
    page(313, "ConfigAntennaTracker", &[Display("displayAntennaTracker")], Tracker),
    page(317, "ConfigMotorTest", &[Display("displayMotorTest")], ConnectedAllParams),
    page(321, "ConfigHWBT", &[Display("displayBluetooth")], Always),
    page(325, "ConfigHWParachute", &[Display("displayParachute")], ConnectedAllParams),
    page(329, "ConfigHWESP8266", &[Display("displayEsp")], ConnectedAllParams),
    page(333, "TrackerUI", &[Display("displayAntennaTracker")], Always),
    page(337, "ConfigFFT", &[Display("displayFFTSetup")], ConnectedAllParams),
    page(342, "ConfigAdvanced", &[Display("isAdvancedMode")], Always),
    page(346, "ConfigTerminal", &[Display("isAdvancedMode"), Display("displayTerminal")], Always),
    page(351, "ConfigREPL", &[Display("isAdvancedMode"), Display("displayREPL")], Connected),
];

/// `SoftwareConfig.SoftwareConfig_Load`'s calls, in order, one line each.
/// `// C#: GCSViews/SoftwareConfig.cs:142-316`
#[rustfmt::skip]
pub const CONFIG_LIST: &[Entry] = &[
    tuning(156, "ConfigAC_Fence", &[GotAllParams, Open, FirmwareIs(Firmware::ArduCopter2), Display("displayGeoFence")]),
    start(164, "ConfigSimplePids", &[GotAllParams, Open, FirmwareIs(Firmware::ArduCopter2), Display("displayBasicTuning")]),
    tuning(169, "ConfigArducopter", &[GotAllParams, Open, FirmwareIs(Firmware::ArduCopter2), Display("displayExtendedTuning")]),
    start(177, "ConfigArduplane", &[GotAllParams, Open, FirmwareIs(Firmware::ArduPlane), Display("displayBasicTuning")]),
    tuning(182, "ConfigArducopter", &[GotAllParams, Open, FirmwareIs(Firmware::ArduPlane), Display("displayExtendedTuning")]),
    start(188, "ConfigArdurover", &[GotAllParams, Open, FirmwareIs(Firmware::ArduRover)]),
    start(193, "ConfigAntennaTracker", &[GotAllParams, Open, FirmwareIs(Firmware::ArduTracker)]),
    tuning(198, "ConfigFriendlyParams", &[GotAllParams, Open, Display("displayStandardParams")]),
    advanced(203, "ConfigFriendlyParamsAdv", &[GotAllParams, Open, Display("displayAdvancedParams")]),
    tuning(208, "ConfigOSD", &[GotAllParams, Open, OnboardOsd]),
    tuning(215, "MavFTPUI", &[GotAllParams, Open, Display("displayMavFTP"), Ftp]),
    tuning(221, "ConfigUserDefined", &[GotAllParams, Open, Display("displayUserParam")]),
    tuning(229, "ConfigRawParams", &[Display("displayFullParamList"), ParamList]),
    start(235, "ConfigFlightModes", &[Open, FirmwareIs(Firmware::Ateryx)]),
    tuning(236, "ConfigAteryxSensors", &[Open, FirmwareIs(Firmware::Ateryx)]),
    tuning(237, "ConfigAteryx", &[Open, FirmwareIs(Firmware::Ateryx)]),
    start(243, "ConfigParamLoading", &[Open, NotGotAllParams, NoStart]),
    tuning(245, "ConfigParamLoading", &[Open, NotGotAllParams, Start]),
    tuning(250, "ConfigPlanner", &[Open, Display("displayPlannerSettings")]),
    start(257, "ConfigPlanner", &[NotOpen, Display("displayPlannerSettings")]),
];

/// A screen's calls.
#[must_use]
pub const fn entries(list: List) -> &'static [Entry] {
    match list {
        List::Setup => SETUP_LIST,
        List::Config => CONFIG_LIST,
    }
}

/// The start of a list's control ids and facts: `setup-page-ConfigFlightModes`, `config.page`.
const fn prefix(list: List) -> &'static str {
    match list {
        List::Setup => "setup",
        List::Config => "config",
    }
}

/// The title a call gives its page, from the ledger.
fn title(list: List, index: usize) -> &'static str {
    entries(list)
        .get(index)
        .and_then(|entry| crate::config_coverage::listing(list, entry.line))
        .map_or("", |(listed, _)| listed.title)
}

/// The heading a call passes as its parent, as an index into the same list; `None` for none.
fn heading(list: List, index: usize) -> Option<usize> {
    let entry = entries(list).get(index)?;
    let (listed, _) = crate::config_coverage::listing(list, entry.line)?;
    let parent = listed.parent?;
    entries(list)
        .iter()
        .position(|candidate| candidate.class == parent)
}

/// What a `Load` handler added: the backstage view's `Pages`, in the order they were added.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Built {
    /// Each page added, as an index into its list's calls, with its heading's, when the heading
    /// was added too: the variable a heading's call is kept in is null when it was not added, and
    /// a page passed a null parent is listed at the top.
    pub items: Vec<(usize, Option<usize>)>,
    /// `SoftwareConfig`'s `start`: the last page kept as the one to open first.
    pub start: Option<usize>,
}

/// Runs a screen's `Load` handler against a vehicle.
#[must_use]
pub fn build(list: List, vehicle: &Vehicle) -> Built {
    let mut built = Built::default();
    // Whether `start` was set when `if (start == null)` was tested: its `else` is the other branch
    // of that one test, not a second test made after the first branch has set it.
    let mut tested: Option<bool> = None;
    for (index, entry) in entries(list).iter().enumerate() {
        let mut kept = built.start.is_some();
        if entry.guards.contains(&Guard::NoStart) {
            tested = Some(kept);
        } else if entry.guards.contains(&Guard::Start) {
            kept = tested.unwrap_or(kept);
        }
        if !entry.guards.iter().all(|guard| guard.holds(vehicle, kept))
            || !entry.enabled.holds(vehicle)
        {
            continue;
        }
        let parent = heading(list, index)
            .filter(|parent| built.items.iter().any(|(added, _)| added == parent));
        built.items.push((index, parent));
        if entry.start {
            built.start = Some(index);
        }
    }
    built
}

/// What a list was built for. `MainV2` shows a SETUP or CONFIG screen again - closing it and
/// running its `Load` anew - when the link opens or closes and when the vehicle changes; and
/// `doConnect` shows it again at its end, after `getParamList` has run, which is the list a
/// connected vehicle's pages are built from. Here the download runs after the connect rather
/// than inside it, so the download's end is a change of its own: a list built while the
/// parameters were still coming (the Loading page listed) is built again once the download is
/// done. The download's end, not `gotAllParams`: a feature switched on makes the vehicle report
/// more parameters than are held (AVD_ENABLE 1 took a copter from 1,408 to 1,419), which is not
/// a reload in the C# either - the page stays until the screen is shown again.
/// `// C#: MainV2.cs:1419-1425, 1684, 1740-1748; Controls/ConnectionControl.cs:143`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Key {
    connected: bool,
    vehicle: Option<VehicleId>,
    /// The connect-time download has run to its end.
    params_fetched: bool,
}

impl Key {
    /// The key a view is at.
    #[must_use]
    pub fn of(view: &TelemetryView) -> Self {
        Self {
            connected: view.connected && view.vehicle.is_some(),
            vehicle: view.vehicle,
            params_fetched: view.parameters_fetched,
        }
    }
}

/// One entry as the list draws it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Row {
    /// The page, as an index into its list's calls.
    pub index: usize,
    /// Its class.
    pub class: &'static str,
    /// The text drawn: a heading with pages under it is `>> title`, a page under one is
    /// indented six spaces.
    /// `// C#: ExtLibs/Controls/BackstageView/BackstageView.cs:222-234`
    pub label: String,
    /// Whether it is the page showing.
    pub selected: bool,
}

/// A `BackstageView` and the screen around it: the pages, which one is showing, which headings
/// are open, and the screen's `lastpagename`.
#[derive(Debug)]
pub struct Backstage {
    list: List,
    built: Built,
    /// `_activePage`, as an index into the list's calls.
    active: Option<usize>,
    /// `expanded`: the headings whose pages are drawn beneath them.
    expanded: Vec<usize>,
    /// `lastpagename`, the title of the page showing when the screen last closed. Static in the
    /// C#, so it outlives the screen.
    last_page_name: String,
    /// What the list was built for, while the screen shows.
    loaded: Option<Key>,
}

impl Backstage {
    /// A screen that has not been shown.
    #[must_use]
    pub const fn new(list: List) -> Self {
        Self {
            list,
            built: Built {
                items: Vec::new(),
                start: None,
            },
            active: None,
            expanded: Vec::new(),
            last_page_name: String::new(),
            loaded: None,
        }
    }

    /// The page showing.
    #[must_use]
    pub fn page(&self) -> Option<&'static Entry> {
        entries(self.list).get(self.active?)
    }

    /// `Load`: adds the pages, then returns the page to open - the one titled `lastpagename`, or
    /// on CONFIG, when there is none, `start`. SETUP opens nothing the first time: its list shows
    /// with no page chosen.
    /// `// C#: GCSViews/InitialSetup.cs:155-395; GCSViews/SoftwareConfig.cs:142-316`
    pub fn load(&mut self, vehicle: &Vehicle, key: Key) -> Option<usize> {
        self.built = build(self.list, vehicle);
        self.active = None;
        self.expanded.clear();
        self.loaded = Some(key);
        let remembered = self
            .built
            .items
            .iter()
            .map(|(index, _)| *index)
            .find(|index| title(self.list, *index) == self.last_page_name);
        match self.list {
            List::Setup => remembered,
            List::Config => remembered.or(self.built.start),
        }
    }

    /// `FormClosing`: remembers the page showing as `lastpagename`, then `backstageView.Close()`.
    /// Returns the page to deactivate.
    /// `// C#: GCSViews/InitialSetup.cs:397-403; GCSViews/SoftwareConfig.cs:323-329;
    /// ExtLibs/Controls/BackstageView/BackstageView.cs:533-575`
    pub fn close(&mut self) -> Option<usize> {
        if let Some(active) = self.active {
            self.last_page_name = title(self.list, active).to_owned();
        }
        self.built = Built::default();
        self.expanded.clear();
        self.loaded = None;
        self.active.take()
    }

    /// Whether the list has been built for the screen showing.
    #[must_use]
    pub const fn loaded(&self) -> Option<Key> {
        self.loaded
    }

    /// The list, built, kept as it is under the link as it now is: a change `MainV2` shows no
    /// screen again for.
    pub fn rekey(&mut self, key: Key) {
        if self.loaded.is_some() {
            self.loaded = Some(key);
        }
    }

    /// Whether the list is to be shown again this frame: the screen left, the link opened or
    /// closed or the vehicle changed (`key` moved), or the Loading page seeing every parameter
    /// in. `held` is the link moving under a page whose change `MainV2` does not show the screen
    /// again for - Force Bootloader on either Install Firmware page, whose
    /// `MainV2.comPort.Open(false)` is not `doConnect` - and keys the list to it instead.
    /// `// C#: ExtLibs/Controls/MainSwitcher.cs:112-138; GCSViews/ConfigurationView/ConfigParamLoading.cs:44-48; GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:517; ConfigFirmware.cs:623; MainV2.cs:1419-1425, 1740-1748`
    pub fn stale(&mut self, key: Key, showing: bool, got_all_params: bool, held: bool) -> bool {
        let Some(loaded) = self.loaded else {
            return false;
        };
        if showing && held && loaded != key {
            self.rekey(key);
        }
        let loading = self
            .page()
            .is_some_and(|entry| entry.class == "ConfigParamLoading");
        !showing || self.loaded != Some(key) || (loading && got_all_params)
    }

    /// Whether a page has pages under it in this list.
    fn has_children(&self, index: usize) -> bool {
        self.built
            .items
            .iter()
            .any(|(_, parent)| *parent == Some(index))
    }

    /// The page's heading in this list.
    fn parent_of(&self, index: usize) -> Option<usize> {
        self.built
            .items
            .iter()
            .find(|(added, _)| *added == index)
            .and_then(|(_, parent)| *parent)
    }

    /// `ActivatePage`'s part in the list: `DrawMenu` opens or closes a heading chosen, and opens
    /// the heading of a page chosen under a closed one; then the page is `_activePage`. Returns
    /// the page that was showing, for its `Deactivate` - which runs even when it is the page
    /// chosen again.
    /// `// C#: ExtLibs/Controls/BackstageView/BackstageView.cs:261-343, 435-523`
    pub fn activate(&mut self, index: usize) -> Option<usize> {
        if self.has_children(index) {
            if let Some(at) = self.expanded.iter().position(|open| *open == index) {
                self.expanded.remove(at);
            } else {
                self.expanded.push(index);
            }
        } else if let Some(parent) = self.parent_of(index)
            && !self.expanded.contains(&parent)
        {
            self.expanded.push(parent);
        }
        self.active.replace(index)
    }

    /// The entries `DrawMenu` draws, top to bottom: each page at the top, and beneath a heading
    /// that is open, its pages. A page added as advanced is skipped outside the Advanced view.
    /// `// C#: ExtLibs/Controls/BackstageView/BackstageView.cs:261-343`
    #[must_use]
    pub fn rows(&self) -> Vec<Row> {
        self.tree(false)
    }

    /// Every page the list holds, in the order it draws them with every heading open.
    #[must_use]
    pub fn pages(&self) -> Vec<Row> {
        self.tree(true)
    }

    fn tree(&self, open: bool) -> Vec<Row> {
        let table = entries(self.list);
        let row = |index: usize, label: String| Row {
            index,
            class: table.get(index).map_or("", |entry| entry.class),
            label,
            selected: self.active == Some(index),
        };
        let mut rows = Vec::new();
        for (index, parent) in &self.built.items {
            let index = *index;
            if parent.is_some()
                || table
                    .get(index)
                    .is_some_and(|entry| entry.advanced && !display("isAdvancedMode"))
            {
                continue;
            }
            let children = self.has_children(index);
            let name = title(self.list, index);
            rows.push(row(
                index,
                if children {
                    format!(">> {name}")
                } else {
                    name.to_owned()
                },
            ));
            if children && (open || self.expanded.contains(&index)) {
                for (child, _) in self
                    .built
                    .items
                    .iter()
                    .filter(|(_, parent)| *parent == Some(index))
                {
                    rows.push(row(*child, format!("      {}", title(self.list, *child))));
                }
            }
        }
        rows
    }
}

/// The screen a list is on.
const fn screen_of(list: List) -> crate::Screen {
    match list {
        List::Setup => crate::Screen::Setup,
        List::Config => crate::Screen::Config,
    }
}

impl MissionPlanner {
    fn backstage(&self, list: List) -> &Backstage {
        match list {
            List::Setup => &self.setup_list,
            List::Config => &self.config_list,
        }
    }

    fn backstage_mut(&mut self, list: List) -> &mut Backstage {
        match list {
            List::Setup => &mut self.setup_list,
            List::Config => &mut self.config_list,
        }
    }

    /// Once a frame: shows a screen's list when the screen is shown, shows it again when
    /// `MainV2` would - the link opening or closing, the vehicle changing, or the Loading page's
    /// timer seeing every parameter in - and closes it when the screen is left, deactivating its
    /// page as disposing the screen does. Not while Force Bootloader - either Install Firmware
    /// page's - holds the window's link: its `Open(false)` shows no screen again
    /// ([`Backstage::stale`]), and the legacy page object, which belongs to the screen, is kept
    /// under the new key with the list.
    /// `// C#: ExtLibs/Controls/MainSwitcher.cs:112-138; GCSViews/ConfigurationView/ConfigParamLoading.cs:44-48; GCSViews/ConfigurationView/ConfigFirmware.cs:623; ConfigFirmwareManifest.cs:517`
    pub(crate) fn backstage_tick(&mut self, view: &TelemetryView) {
        let key = Key::of(view);
        let got_all_params = Vehicle::of(view, None).got_all_params;
        for list in List::ALL {
            let showing = self.screen == screen_of(list);
            let held = matches!(list, List::Setup)
                && crate::config::force_bootloader::holds_setup(
                    &self.install_firmware,
                    &self.firmware_legacy,
                );
            let stale = self
                .backstage_mut(list)
                .stale(key, showing, got_all_params, held);
            if showing && held {
                self.firmware_legacy.rekey(key);
            }
            if stale && let Some(old) = self.backstage_mut(list).close() {
                self.deactivate_page(list, old);
            }
            if stale && matches!(list, List::Setup) {
                self.install_firmware.screen_disposed();
            }
            if showing && self.backstage(list).loaded().is_none() {
                let vehicle = Vehicle::of(view, self.telemetry.firmware_banner());
                if let Some(index) = self.backstage_mut(list).load(&vehicle, key) {
                    self.choose_page(list, index);
                }
            }
        }
    }

    // ---- Display view (row 71) ----
    /// A SETUP or CONFIG tab clicked while its screen shows: `MainSwitcher.ShowScreen` disposes
    /// the screen showing - the same one included, as it is not persistent - and makes it anew,
    /// so its list is closed here, its page deactivated, and built again at the next tick with
    /// the display view as it is now.
    /// `// C#: ExtLibs/Controls/MainSwitcher.cs:112-153; MainV2.cs:1357-1362, 3179-3180`
    pub(crate) fn show_screen_again(&mut self, screen: crate::Screen) {
        for list in List::ALL {
            if screen_of(list) == screen
                && let Some(old) = self.backstage_mut(list).close()
            {
                self.deactivate_page(list, old);
                if matches!(list, List::Setup) {
                    self.install_firmware.screen_disposed();
                }
            }
        }
    }
    // ---- end Display view ----

    /// `ActivatePage`: the list's part, then the old page's `Deactivate` and the new one's
    /// `Activate`.
    /// `// C#: ExtLibs/Controls/BackstageView/BackstageView.cs:428-523`
    pub(crate) fn choose_page(&mut self, list: List, index: usize) {
        if let Some(old) = self.backstage_mut(list).activate(index) {
            self.deactivate_page(list, old);
        }
        self.activate_page(list, index);
    }

    /// `IActivate.Activate` for the pages that keep state.
    fn activate_page(&mut self, list: List, index: usize) {
        match entries(list).get(index).map(|entry| entry.class) {
            // C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs (Activate), and the
            // connected page's text: ConfigFirmwareDisabled.cs
            Some("ConfigFirmwareManifest" | "ConfigFirmwareDisabled")
                if !self.install_firmware.is_open() =>
            {
                self.install_firmware.toggle(&self.telemetry.view());
            }
            // ---- Firmware Legacy / Ateryx ----
            // C#: GCSViews/ConfigurationView/ConfigFirmware.cs:34-64
            Some("ConfigFirmware") => self.firmware_legacy_activate(),
            // Every time, as `ActivatePage` calls it.
            // C#: GCSViews/ConfigurationView/ConfigAteryx.cs:32-56
            Some("ConfigAteryx") => self.ateryx_activate(),
            // ---- end Firmware Legacy / Ateryx ----
            // C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:86-145; ConfigHWCompass.cs:31-243
            Some(class @ ("ConfigHWCompass2" | "ConfigHWCompass")) if !self.compass.is_active() => {
                let view = self.telemetry.view();
                let info = crate::config::compass::VehicleInfo::of(
                    &view,
                    self.telemetry.firmware_banner(),
                );
                if let Some(class) = crate::config::compass::Class::of(class) {
                    self.compass.activate(
                        class,
                        &view.parameters,
                        Key::of(&view),
                        info,
                        crate::metadata::lookup,
                    );
                }
            }
            // C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:36-54
            Some("ConfigFrameClassType") if !self.frame_type.is_active() => {
                self.frame_type.toggle(&self.telemetry);
            }
            // C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:38-232
            Some("ConfigFlightModes") if !self.flight_modes.is_active() => {
                self.flight_modes.toggle(&self.telemetry);
            }
            // C#: GCSViews/ConfigurationView/ConfigFailSafe.cs:24-93
            Some("ConfigFailSafe") if !self.failsafe.is_open() => {
                self.failsafe.toggle(&self.telemetry);
            }
            // C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:18-176
            Some("ConfigBatteryMonitoring") if !self.battery_monitor.is_open() => {
                self.battery_monitor
                    .toggle(&self.telemetry, &self.persisted);
            }
            // C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:42-177
            Some("ConfigRadioInput") if !self.radio_input.is_active() => {
                self.radio_input.toggle(&self.telemetry);
            }
            // Every time, as `ActivatePage` calls it: the buttons are built from the frame each
            // time the page is shown.
            // C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:43-109
            Some("ConfigMotorTest") => {
                let view = self.telemetry.view();
                self.motor_test.activate(&view, crate::metadata::lookup);
            }
            // The page object's constructor builds the rows, once per screen; `Activate` starts
            // the bars' timer.
            // C#: GCSViews/ConfigurationView/ConfigRadioOutput.cs:11-33, 71-74
            Some("ConfigRadioOutput") => {
                let view = self.telemetry.view();
                self.servo_output.activate(
                    &self.telemetry,
                    &view.parameters,
                    Key::of(&view),
                    crate::metadata::lookup,
                );
            }
            // The page object's first `Activate` reads the ports' names over MAVFTP.
            // C#: GCSViews/ConfigurationView/ConfigSerial.cs:37-380
            Some("ConfigSerial") => {
                let view = self.telemetry.view();
                self.serial_ports.activate(
                    &self.telemetry,
                    &view.parameters,
                    Key::of(&view),
                    crate::metadata::lookup,
                );
            }
            // C#: GCSViews/ConfigurationView/ConfigESCCalibration.cs:15-26
            Some("ConfigESCCalibration") => {
                let view = self.telemetry.view();
                self.esc_calibration.activate(
                    &self.telemetry,
                    &view.parameters,
                    Key::of(&view),
                    crate::metadata::lookup,
                );
            }
            // Every time, as `ActivatePage` calls it.
            // C#: GCSViews/ConfigurationView/ConfigPlanner.cs:55-256
            Some("ConfigPlanner") => self.planner_activate(),
            // ---- Mandatory Hardware pages ----
            // C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:27-31
            Some("ConfigAccelerometerCalibration") => {
                let key = Key::of(&self.telemetry.view());
                self.accel_calibration.activate(key);
            }
            // C#: GCSViews/ConfigurationView/ConfigFrameType.cs:25-34
            Some("ConfigFrameType") => {
                let view = self.telemetry.view();
                self.frame_type_legacy
                    .activate(&view.parameters, Key::of(&view));
            }
            // `ConfigSecureAP` is a plain `UserControl`: shown, on the page object of this screen.
            Some("ConfigSecureAP") => {
                let key = Key::of(&self.telemetry.view());
                self.secure.show(key);
            }
            // ---- end Mandatory Hardware pages ----
            // Optional Hardware pages (`config/optional.rs`), each `Activate` every time.
            // C#: GCSViews/ConfigurationView/ConfigADSB.cs:253-305; ConfigBatteryMonitoring2.cs:17-62;
            // ConfigHWRangeFinder.cs:17-34; ConfigHWAirspeed.cs:18-63; ConfigHWOptFlow.cs:17-75;
            // ConfigMount.cs:21-168
            Some(
                class @ ("ConfigADSB"
                | "ConfigBatteryMonitoring2"
                | "ConfigHWRangeFinder"
                | "ConfigHWAirspeed"
                | "ConfigHWOptFlow"
                | "ConfigMount"),
            ) => self.optional_activate(class),
            // end Optional Hardware pages
            // ---- Basic Tuning / Advanced ----
            // Every time, as `ActivatePage` calls it. `ConfigAdvanced.Activate` does nothing.
            // C#: GCSViews/ConfigurationView/ConfigArduplane.cs:26-126
            Some("ConfigArduplane") => self.basic_tuning_activate(),
            // ---- end Basic Tuning / Advanced ----
            // ---- Extended Tuning ----
            // Every time, as `ActivatePage` calls it.
            // C#: GCSViews/ConfigurationView/ConfigArducopter.cs:26-204
            Some("ConfigArducopter") => self.extended_tuning_activate(),
            // ---- end Extended Tuning ----
            // ---- GeoFence / rover Basic Tuning / User Params ----
            // Every time, as `ActivatePage` calls it (`config/software_pages.rs`).
            // C#: GCSViews/ConfigurationView/ConfigAC_Fence.cs:19-49; ConfigArdurover.cs:26-114;
            // ConfigUserDefined.cs:88-91
            Some(
                class @ ("ConfigAC_Fence" | "ConfigSimplePids" | "ConfigArdurover"
                | "ConfigUserDefined"),
            ) => {
                self.software_activate(class);
            }
            // ---- end GeoFence / rover Basic Tuning / User Params ----
            // ---- Standard / Advanced Params, MAVFtp, Heli Setup (row 71) ----
            // Every time, as `ActivatePage` calls it (`config/software_pages2.rs`); `MavFTPUI`
            // loads once per screen, as it is not `IActivate`.
            // C#: GCSViews/ConfigurationView/ConfigFriendlyParams.cs:253-260; Controls/MavFTPUI.cs:665-668;
            // GCSViews/ConfigurationView/ConfigTradHeli4.cs:28-172
            Some(
                class @ ("ConfigFriendlyParams"
                | "ConfigFriendlyParamsAdv"
                | "MavFTPUI"
                | "ConfigTradHeli4"),
            ) => self.software2_activate(class),
            // ---- end Standard / Advanced Params, MAVFtp, Heli Setup ----
            // ---- SETUP's small pages (row 70) ----
            // Every time, as `ActivatePage` calls it (`config/extra_setup.rs`).
            // C#: GCSViews/ConfigurationView/ConfigHWParachute.cs:17-45; ConfigHWOSD.cs:14-21;
            // ConfigGPSOrder.cs:21-51; ConfigHWIDs.cs:16-31; ConfigCompassMot.cs:25-30;
            // ConfigInitialParams.cs:56-68
            Some(
                class @ ("ConfigHWParachute"
                | "ConfigHWOSD"
                | "ConfigGPSOrder"
                | "ConfigHWIDs"
                | "ConfigCompassMot"
                | "ConfigInitialParams"
                | "ConfigFFT"
                | "Sikradio"),
            ) => self.extra_setup_activate(class),
            // C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:39-57
            Some("ConfigDroneCAN") => self.extra_setup_activate("ConfigDroneCAN"),
            // ---- end SETUP's small pages ----
            // ---- RTK/GPS Inject ----
            // C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:81-165, 1214-1222
            Some("ConfigSerialInjectGPS") => self.rtk_inject_activate(),
            // ---- end RTK/GPS Inject ----
            // `Joystick_Load`, each time the page shows (`joystick.rs`).
            // C#: Joystick/JoystickSetup.cs:28-129
            Some("JoystickSetup") => {
                self.sticks
                    .load(crate::joystick::Host::Setup, &self.persisted);
            }
            _ => {}
        }
    }

    /// `IDeactivate.Deactivate` for the pages that keep state.
    fn deactivate_page(&mut self, list: List, index: usize) {
        match entries(list).get(index).map(|entry| entry.class) {
            // C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs (Deactivate): OFFICIAL again
            Some("ConfigFirmwareManifest" | "ConfigFirmwareDisabled")
                if self.install_firmware.is_open() =>
            {
                self.install_firmware.close();
            }
            // ---- Firmware Legacy / Ateryx ----
            // C#: GCSViews/ConfigurationView/ConfigFirmware.cs:665-673
            Some("ConfigFirmware") => self.firmware_legacy.deactivate(),
            // `ConfigAteryx` is `IActivate` only: hidden, the box with the focus validated.
            Some("ConfigAteryx") => self.ateryx.hide(),
            // ---- end Firmware Legacy / Ateryx ----
            // C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:147-152; ConfigHWCompass.cs:252-255
            Some("ConfigHWCompass2" | "ConfigHWCompass") if self.compass.is_active() => {
                let open = self.telemetry.view().connected;
                self.compass.deactivate(open);
            }
            // C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs (Deactivate)
            Some("ConfigFrameClassType") if self.frame_type.is_active() => {
                self.frame_type.toggle(&self.telemetry);
            }
            // C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:234-237
            Some("ConfigFlightModes") if self.flight_modes.is_active() => {
                self.flight_modes.toggle(&self.telemetry);
            }
            // C#: GCSViews/ConfigurationView/ConfigFailSafe.cs:95-99
            Some("ConfigFailSafe") if self.failsafe.is_open() => {
                self.failsafe.toggle(&self.telemetry);
            }
            // Nothing a box is left holding is validated: `Deactivate` sets `startup` before the
            // page is hidden (`BackstageView.cs:452-466`).
            Some("ConfigBatteryMonitoring") if self.battery_monitor.is_open() => {
                self.battery_monitor.close();
            }
            // C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:179-182
            Some("ConfigRadioInput") if self.radio_input.is_active() => {
                self.radio_input.deactivate();
            }
            // `ConfigMotorTest` is `IActivate` only: hidden, nothing stopped or written.
            Some("ConfigMotorTest") if self.motor_test.is_active() => {
                self.motor_test.deactivate();
            }
            // C#: GCSViews/ConfigurationView/ConfigRadioOutput.cs:76-79
            Some("ConfigRadioOutput") => {
                self.servo_output.deactivate(std::time::Instant::now());
            }
            // C#: GCSViews/ConfigurationView/ConfigSerial.cs:493-496
            Some("ConfigSerial") => self.serial_ports.hide(),
            // `ConfigESCCalibration` is `IActivate` only: hidden, a number being typed into read.
            Some("ConfigESCCalibration") => {
                self.esc_calibration.hide(std::time::Instant::now());
            }
            // `ConfigPlanner` is `IActivate` only: hidden, its boxes put away.
            Some("ConfigPlanner") if self.planner.is_active() => self.planner.deactivate(),
            // ---- Mandatory Hardware pages ----
            // C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:33-37
            Some("ConfigAccelerometerCalibration") => self.accel_calibration.deactivate(),
            // C#: GCSViews/ConfigurationView/ConfigFrameType.cs:36-39
            Some("ConfigFrameType") => self.frame_type_legacy.deactivate(),
            Some("ConfigSecureAP") => self.secure.hide(),
            // ---- end Mandatory Hardware pages ----
            // Optional Hardware pages: `Deactivate` where the page has one, else hidden.
            // C#: GCSViews/ConfigurationView/ConfigADSB.cs:667-671; ConfigBatteryMonitoring2.cs:64-68;
            // ConfigHWRangeFinder.cs:36-39
            Some(
                class @ ("ConfigADSB"
                | "ConfigBatteryMonitoring2"
                | "ConfigHWRangeFinder"
                | "ConfigHWAirspeed"
                | "ConfigHWOptFlow"
                | "ConfigMount"),
            ) => self.optional_deactivate(class),
            // end Optional Hardware pages
            // ---- Basic Tuning / Advanced ----
            // `ConfigArduplane` is `IActivate` only: hidden, a number being typed into read.
            Some("ConfigArduplane") => self.basic_tuning.hide(std::time::Instant::now()),
            // ---- end Basic Tuning / Advanced ----
            // ---- Extended Tuning ----
            // `ConfigArducopter` is `IActivate` only: hidden, a number being typed into read.
            Some("ConfigArducopter") => self.extended_tuning_hide(),
            // ---- end Extended Tuning ----
            // ---- GeoFence / rover Basic Tuning / User Params ----
            // `ConfigAC_Fence` and `ConfigArdurover` are `IActivate` only: hidden, a number being
            // typed into read. `ConfigUserDefined.Deactivate` does nothing.
            // C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:93-96
            Some(
                class @ ("ConfigAC_Fence" | "ConfigSimplePids" | "ConfigArdurover"
                | "ConfigUserDefined"),
            ) => {
                self.software_deactivate(class);
            }
            // ---- end GeoFence / rover Basic Tuning / User Params ----
            // ---- Standard / Advanced Params, MAVFtp, Heli Setup (row 71) ----
            // `ConfigTradHeli4.Deactivate` empties its four tables; the others are hidden, a
            // number or a name being typed into read.
            // C#: GCSViews/ConfigurationView/ConfigTradHeli4.cs:187-193
            Some(
                class @ ("ConfigFriendlyParams"
                | "ConfigFriendlyParamsAdv"
                | "MavFTPUI"
                | "ConfigTradHeli4"),
            ) => self.software2_deactivate(class),
            // ---- end Standard / Advanced Params, MAVFtp, Heli Setup ----
            // ---- SETUP's small pages (row 70) ----
            // `ConfigCompassMot.Deactivate` stops a running calibration; the rest are `IActivate`
            // only: hidden, a number or text being typed into read.
            // C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:32-48
            Some(
                class @ ("ConfigHWParachute"
                | "ConfigHWOSD"
                | "ConfigGPSOrder"
                | "ConfigHWIDs"
                | "ConfigCompassMot"
                | "ConfigInitialParams"
                | "ConfigFFT"
                | "Sikradio"),
            ) => self.extra_setup_deactivate(class),
            // C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:680-684
            Some("ConfigDroneCAN") => self.extra_setup_deactivate("ConfigDroneCAN"),
            // ---- end SETUP's small pages ----
            // ---- RTK/GPS Inject ----
            // C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1224-1227
            Some("ConfigSerialInjectGPS") => self.rtk_inject_deactivate(),
            // ---- end RTK/GPS Inject ----
            // C#: Joystick/JoystickSetup.cs:527-536
            Some("JoystickSetup") => self.sticks.close(),
            _ => {}
        }
    }

    /// A screen drawn as a backstage view: the list, and the page chosen from it.
    pub(crate) fn backstage_screen(
        &self,
        list: List,
        view: &TelemetryView,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> AnyElement {
        let backstage = self.backstage(list);
        let mut menu = crate::probe::measured(format!("{}-list", prefix(list)), div())
            .id(SharedString::from(format!("{}-list", prefix(list))))
            .flex()
            .flex_col()
            .flex_shrink_0()
            .w(px(MENU_WIDTH))
            .min_h(px(0.0))
            .overflow_y_scroll()
            .bg(rgb(theme::PANEL))
            .border_r_1()
            .border_color(rgb(theme::BORDER));
        for row in backstage.rows() {
            menu = menu.child(list_button(list, row, cx));
        }

        let content = match backstage.page() {
            Some(entry) => self
                .page_body(entry.class, view, window, cx)
                .unwrap_or_else(|| {
                    let index = backstage.active.unwrap_or_default();
                    not_ported(title(list, index), entry.class)
                }),
            // Nothing chosen: the C#'s page area is empty.
            None => div().into_any_element(),
        };

        div()
            .flex()
            .flex_1()
            .min_h(px(0.0))
            .min_w(px(0.0))
            .child(menu)
            .child(
                crate::probe::measured(format!("{}-page", prefix(list)), div())
                    .id(SharedString::from(format!("{}-page", prefix(list))))
                    .flex()
                    .flex_col()
                    .flex_1()
                    .min_w(px(0.0))
                    .min_h(px(0.0))
                    .p_2()
                    .overflow_y_scroll()
                    .child(content),
            )
            .into_any_element()
    }

    /// The page area for a page this application has, or `None` for one it lacks. Adding a
    /// ported page is one arm here (and, for a page that keeps state, one in `activate_page` and
    /// `deactivate_page`).
    fn page_body(
        &self,
        class: &str,
        view: &TelemetryView,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> Option<AnyElement> {
        Some(match class {
            "ConfigParamLoading" => param_loading_page(cx),
            "ConfigMandatory" => heading_page(MANDATORY_TEXT),
            "ConfigOptional" => heading_page(OPTIONAL_TEXT),
            // ---- Mandatory Hardware pages ----
            // C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.Designer.cs:31-121;
            // ConfigAccelerometerCalibration.resx
            "ConfigAccelerometerCalibration" => div()
                .flex()
                .flex_col()
                .children(crate::config::accel_calibration::page(
                    &self.accel_calibration,
                    cx,
                ))
                .into_any_element(),
            // C#: GCSViews/ConfigurationView/ConfigFrameType.Designer.cs:31-222;
            // ConfigFrameType.resx
            "ConfigFrameType" => div()
                .flex()
                .flex_col()
                .children(crate::config::frame_type_legacy::page(
                    &self.frame_type_legacy,
                    cx,
                ))
                .into_any_element(),
            // C#: GCSViews/ConfigurationView/ConfigSecureAP.Designer.cs:29-143
            "ConfigSecureAP" => div()
                .flex()
                .flex_col()
                .child(crate::config::secure::page(&self.secure, cx))
                .into_any_element(),
            // ---- end Mandatory Hardware pages ----
            // C#: GCSViews/ConfigurationView/ConfigHWCompass2.Designer.cs:84-571;
            // GCSViews/ConfigurationView/ConfigHWCompass.resx
            "ConfigHWCompass2" | "ConfigHWCompass" => column()
                .child(crate::config::compass::page(
                    &self.compass,
                    &self.compass_focus,
                    cx,
                ))
                .into_any_element(),
            "ConfigRadioInput" => column()
                .children(crate::config::radio::page(&self.radio_input, cx))
                .into_any_element(),
            "ConfigFirmwareManifest" | "ConfigFirmwareDisabled" => column()
                .child(crate::config::firmware::page(
                    &self.install_firmware,
                    &self.firmware_page_focus,
                    cx,
                ))
                .into_any_element(),
            // ---- Firmware Legacy / Ateryx ----
            // Wider than the column: the Designer's page is 986 pixels.
            // C#: GCSViews/ConfigurationView/ConfigFirmware.Designer.cs:52-307; ConfigFirmware.resx
            "ConfigFirmware" => div()
                .flex()
                .flex_col()
                .child(crate::config::firmware_legacy::page(
                    &self.firmware_legacy,
                    cx,
                ))
                .into_any_element(),
            // C#: GCSViews/ConfigurationView/ConfigAteryx.Designer.cs:29-906; ConfigAteryx.resx
            "ConfigAteryx" => {
                crate::config::ateryx::page(&self.ateryx, &self.ateryx_focus, window, cx)
            }
            // ---- end Firmware Legacy / Ateryx ----
            "ConfigFrameClassType" => column()
                .children(crate::config::frame_type::page(&self.frame_type, cx))
                .into_any_element(),
            "ConfigFlightModes" => column()
                .children(crate::config::flight_modes::page(
                    &self.flight_modes,
                    view,
                    &self.flight_modes_focus,
                    cx,
                ))
                .into_any_element(),
            "ConfigFailSafe" => column()
                .child(crate::config::failsafe::page(
                    &self.failsafe,
                    &self.failsafe_focus,
                    view,
                    window,
                    cx,
                ))
                .into_any_element(),
            "ConfigBatteryMonitoring" => column()
                .child(crate::config::battery_monitor::page(
                    &self.battery_monitor,
                    &self.battery_focus,
                    window,
                    cx,
                ))
                .into_any_element(),
            "ConfigMotorTest" => column()
                .children(crate::config::motor_test::page(
                    &self.motor_test,
                    &self.motor_focus,
                    window,
                    cx,
                ))
                .into_any_element(),
            // C#: GCSViews/ConfigurationView/ConfigRadioOutput.Designer.cs:28-176
            "ConfigRadioOutput" => column()
                .children(crate::config::servo_output::page(
                    &self.servo_output,
                    &self.servo_focus,
                    view,
                    window,
                    cx,
                ))
                .into_any_element(),
            // Wider than the other pages: the Designer's table is 789 pixels.
            // C#: GCSViews/ConfigurationView/ConfigSerial.Designer.cs:29-121
            "ConfigSerial" => div()
                .flex()
                .flex_col()
                .children(crate::config::serial_ports::page(&self.serial_ports, cx))
                .into_any_element(),
            // C#: GCSViews/ConfigurationView/ConfigESCCalibration.resx
            "ConfigESCCalibration" => column()
                .children(crate::config::esc_calibration::page(
                    &self.esc_calibration,
                    &self.esc_focus,
                    window,
                    cx,
                ))
                .into_any_element(),
            // C#: Joystick/JoystickSetup.Designer.cs; JoystickSetup.resx
            "JoystickSetup" => div()
                .flex()
                .flex_col()
                .child(crate::joystick::page(
                    &self.sticks,
                    &self.joystick_focus,
                    window,
                    cx,
                ))
                .into_any_element(),
            "ConfigRawParams" => self.params_body(view, window, cx),
            // C#: GCSViews/ConfigurationView/ConfigPlanner.Designer.cs:29-994; ConfigPlanner.resx
            "ConfigPlanner" => self.planner_page(view, window, cx),
            // Optional Hardware pages (`config/optional.rs`), each at its `.resx` places.
            // C#: GCSViews/ConfigurationView/ConfigADSB.resx
            "ConfigADSB" => self.optional_page(class, view, window, cx),
            // C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.resx
            "ConfigBatteryMonitoring2" => self.optional_page(class, view, window, cx),
            // C#: GCSViews/ConfigurationView/ConfigHWRangeFinder.resx
            "ConfigHWRangeFinder" => self.optional_page(class, view, window, cx),
            // C#: GCSViews/ConfigurationView/ConfigHWAirspeed.resx
            "ConfigHWAirspeed" => self.optional_page(class, view, window, cx),
            // C#: GCSViews/ConfigurationView/ConfigHWOptFlow.resx
            "ConfigHWOptFlow" => self.optional_page(class, view, window, cx),
            // C#: GCSViews/ConfigurationView/ConfigMount.resx
            "ConfigMount" => self.optional_page(class, view, window, cx),
            // end Optional Hardware pages
            // ---- Basic Tuning / Advanced ----
            // C#: GCSViews/ConfigurationView/ConfigArduplane.Designer.cs:29-1078; ConfigArduplane.resx
            "ConfigArduplane" => crate::config::basic_tuning::page(
                &self.basic_tuning,
                &self.basic_tuning_focus,
                window,
                cx,
            ),
            // C#: GCSViews/ConfigurationView/ConfigAdvanced.Designer.cs:29-266; ConfigAdvanced.resx
            "ConfigAdvanced" => crate::config::advanced::page(cx),
            // ---- end Basic Tuning / Advanced ----
            // ---- Extended Tuning ----
            // C#: GCSViews/ConfigurationView/ConfigArducopter.Designer.cs; ConfigArducopter.resx
            "ConfigArducopter" => self.extended_tuning_page(window, cx),
            // ---- end Extended Tuning ----
            // ---- GeoFence / rover Basic Tuning / User Params ----
            // C#: GCSViews/ConfigurationView/ConfigAC_Fence.Designer.cs:29-220; ConfigAC_Fence.resx
            "ConfigAC_Fence" => self.software_page(class, window, cx),
            // C#: GCSViews/ConfigurationView/ConfigSimplePids.Designer.cs:29-56; ConfigSimplePids.resx;
            // ExtLibs/Controls/RangeControl.Designer.cs
            "ConfigSimplePids" => self.software_page(class, window, cx),
            // C#: GCSViews/ConfigurationView/ConfigArdurover.Designer.cs:29-723; ConfigArdurover.resx
            "ConfigArdurover" => self.software_page(class, window, cx),
            // C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:46-86
            "ConfigUserDefined" => self.software_page(class, window, cx),
            // ---- end GeoFence / rover Basic Tuning / User Params ----
            // ---- Standard / Advanced Params, MAVFtp, Heli Setup (row 71) ----
            // C#: GCSViews/ConfigurationView/ConfigFriendlyParams.Designer.cs:29-74; ConfigFriendlyParams.resx
            "ConfigFriendlyParams" => self.software2_page(class, window, cx),
            "ConfigFriendlyParamsAdv" => self.software2_page(class, window, cx),
            // C#: Controls/MavFTPUI.Designer.cs:29-235
            "MavFTPUI" => self.software2_page(class, window, cx),
            // C#: GCSViews/ConfigurationView/ConfigTradHeli4.Designer.cs:29-1068
            "ConfigTradHeli4" => self.software2_page(class, window, cx),
            // ---- end Standard / Advanced Params, MAVFtp, Heli Setup ----
            // ---- SETUP's small pages (row 70) ----
            // C#: GCSViews/ConfigurationView/ConfigHWParachute.Designer.cs; ConfigHWOSD.resx;
            // ConfigGPSOrder.Designer.cs; ConfigHWIDs.Designer.cs; ConfigCompassMot.Designer.cs;
            // ConfigInitialParams.Designer.cs
            "ConfigHWParachute" => self.extra_setup_page(class, window, cx),
            "ConfigHWOSD" => self.extra_setup_page(class, window, cx),
            "ConfigGPSOrder" => self.extra_setup_page(class, window, cx),
            "ConfigHWIDs" => self.extra_setup_page(class, window, cx),
            "ConfigCompassMot" => self.extra_setup_page(class, window, cx),
            "ConfigInitialParams" => self.extra_setup_page(class, window, cx),
            // C#: GCSViews/ConfigurationView/ConfigFFT.cs:71-157
            "ConfigFFT" => self.extra_setup_page(class, window, cx),
            // C#: GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:29-695
            "ConfigDroneCAN" => self.extra_setup_page(class, window, cx),
            // C#: Radio/Sikradio.Designer.cs and Sikradio.resx
            "Sikradio" => self.extra_setup_page(class, window, cx),
            // ---- end SETUP's small pages ----
            // ---- RTK/GPS Inject ----
            // C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.Designer.cs:29-783;
            // ConfigSerialInjectGPS.resx
            "ConfigSerialInjectGPS" => {
                crate::config::rtk_inject::page(&self.rtk_inject, &self.rtk_focus, window, cx)
            }
            // ---- end RTK/GPS Inject ----
            _ => return None,
        })
    }
}

/// A page's column: the width the ported pages are laid out in.
fn column() -> gpui::Div {
    div().flex().flex_col().gap_2().w(px(PAGE_WIDTH))
}

/// One entry of the list: `CreateLinkButton`'s `BackstageViewButton`, 30 pixels high with its
/// text at (5, 6), bold. The chosen one is filled with the highlight, with the arrow into the page
/// at its right; the rest are grey text, lined above and below while the pointer is over them.
/// `// C#: ExtLibs/Controls/BackstageView/BackstageView.cs:217-259; ExtLibs/Controls/BackstageView/BackstageViewButton.cs:62-138`
fn list_button(list: List, row: Row, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let id = format!("{}-page-{}", prefix(list), row.class);
    let index = row.index;
    let base = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .relative()
        .flex_shrink_0()
        .w_full()
        .h(px(BUTTON_HEIGHT))
        .pl(px(5.0))
        .pt(px(6.0))
        .border_t_1()
        .border_b_1()
        .overflow_hidden()
        .whitespace_nowrap()
        .text_sm()
        .font_weight(FontWeight::BOLD)
        .cursor_pointer()
        .child(row.label)
        .on_click(cx.listener(move |this, _event, _window, cx| {
            this.choose_page(list, index);
            cx.notify();
        }));
    if row.selected {
        base.bg(rgb(theme::ACCENT))
            .border_color(rgb(theme::ACCENT))
            .text_color(rgb(theme::BG))
            .child(selected_arrow())
            .into_any_element()
    } else {
        base.text_color(rgb(theme::DIM))
            .border_color(gpui::transparent_black())
            .hover(|style| {
                style
                    .bg(rgb(theme::ACTION))
                    .border_color(rgb(theme::BORDER))
            })
            .into_any_element()
    }
}

/// The arrow a chosen entry points into the page with, in the page's colour: eight pixels either
/// side of the middle of its right edge.
/// `// C#: ExtLibs/Controls/BackstageView/BackstageViewButton.cs:84-103`
fn selected_arrow() -> impl IntoElement {
    gpui::canvas(
        |_bounds, _window, _cx| (),
        |bounds, (), window, _cx| {
            let right = bounds.origin.x + bounds.size.width;
            let middle = bounds.origin.y + bounds.size.height / 2.0;
            let size = px(8.0);
            let mut arrow = gpui::PathBuilder::fill();
            arrow.move_to(gpui::point(right, middle + size));
            arrow.line_to(gpui::point(right - size, middle));
            arrow.line_to(gpui::point(right, middle - size));
            arrow.close();
            if let Ok(path) = arrow.build() {
                window.paint_path(path, gpui::Hsla::from(rgb(theme::BG)));
            }
        },
    )
    .absolute()
    .top_0()
    .right_0()
    .w(px(8.0))
    .h_full()
}

/// A page the list names and this application does not have.
fn not_ported(title: &str, class: &str) -> AnyElement {
    panel(
        title,
        div()
            .text_sm()
            .text_color(rgb(theme::DIM))
            .child(format!("{class} - not ported")),
    )
    .w(px(PAGE_WIDTH))
    .into_any_element()
}

/// `ConfigMandatory`'s `label1`.
/// `// C#: GCSViews/ConfigurationView/ConfigMandatory.resx (label1.Text)`
const MANDATORY_TEXT: &str = "The following pages are required to be configured before your \
                              autopilot will work. Please work though them all.";

/// `ConfigOptional`'s `label1`.
/// `// C#: GCSViews/ConfigurationView/ConfigOptional.resx (label1.Text)`
const OPTIONAL_TEXT: &str = "The following pages are OPTIONAL configure them if you have \
                             aditional hardware, or other requirements.";

/// A heading's page: its one sentence, `label1` at (25, 25), 246 by 78.
/// `// C#: GCSViews/ConfigurationView/ConfigMandatory.resx, ConfigOptional.resx (label1.Location, label1.Size)`
fn heading_page(text: &'static str) -> AnyElement {
    div()
        .pl(px(25.0))
        .pt(px(25.0))
        .child(
            div()
                .w(px(246.0))
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(text),
        )
        .into_any_element()
}

/// `ConfigParamLoading`'s `label1`.
/// `// C#: GCSViews/ConfigurationView/ConfigParamLoading.Designer.cs:44-45`
const LOADING_TEXT: &str =
    "Paramaters are still loading. Many screens will not work untill all Parameters are loaded. ";

/// The Loading page, listed while the parameters are still arriving: its label, and Retry Now,
/// which asks for the list again. Its timer, which shows the screen again once every parameter is
/// in, is `backstage_tick`'s.
/// `// C#: GCSViews/ConfigurationView/ConfigParamLoading.cs:34-53; ConfigParamLoading.Designer.cs:36-66`
fn param_loading_page(cx: &mut Context<MissionPlanner>) -> AnyElement {
    div()
        .relative()
        .w(px(495.0))
        .h(px(323.0))
        .child(
            div()
                .absolute()
                .left(px(140.0))
                .top(px(131.0))
                .w(px(218.0))
                .h(px(60.0))
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(LOADING_TEXT),
        )
        .child(
            div()
                .absolute()
                .left(px(209.0))
                .top(px(194.0))
                .child(action_sized(
                    "but_forceparams",
                    "Retry Now",
                    theme::ACCENT,
                    true,
                    Some(px(75.0)),
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.telemetry.download_parameters();
                        cx.notify();
                    }),
                )),
        )
        .into_any_element()
}

/// Facts a UI test asserts on, for each list: the page showing (`setup.page`, its class, and
/// `setup.title`), every page the list holds in the order it draws them (`setup.pages`, classes;
/// `setup.titles`), the entries drawn now (`setup.shown`), the headings with their pages
/// (`setup.groups`, `title=class,class;...`) and the headings open (`setup.expanded`); the same
/// under `config.` for CONFIG.
pub fn record_facts(lists: [&Backstage; 2]) {
    use crate::facts::record;
    for backstage in lists {
        let key = prefix(backstage.list);
        let classes = |rows: &[Row]| {
            rows.iter()
                .map(|row| row.class)
                .collect::<Vec<_>>()
                .join(",")
        };
        let pages = backstage.pages();
        record(
            format!("{key}.page"),
            backstage.page().map_or("none", |entry| entry.class),
        );
        record(
            format!("{key}.title"),
            backstage
                .active
                .map_or("none", |index| title(backstage.list, index)),
        );
        record(format!("{key}.pages"), classes(&pages));
        let titles: Vec<&str> = pages
            .iter()
            .map(|row| title(backstage.list, row.index))
            .collect();
        record(format!("{key}.titles"), titles.join(","));
        record(format!("{key}.shown"), classes(&backstage.rows()));
        let groups: Vec<String> = pages
            .iter()
            .filter(|row| backstage.has_children(row.index))
            .map(|heading| {
                let children: Vec<&str> = backstage
                    .built
                    .items
                    .iter()
                    .filter(|(_, parent)| *parent == Some(heading.index))
                    .filter_map(|(child, _)| entries(backstage.list).get(*child))
                    .map(|entry| entry.class)
                    .collect();
                format!(
                    "{}={}",
                    title(backstage.list, heading.index),
                    children.join(",")
                )
            })
            .collect();
        record(format!("{key}.groups"), groups.join(";"));
        let expanded: Vec<&str> = backstage
            .expanded
            .iter()
            .filter_map(|index| entries(backstage.list).get(*index))
            .map(|entry| entry.class)
            .collect();
        record(format!("{key}.expanded"), expanded.join(","));
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use mp_vehicle::VehicleState;

    use super::*;
    use crate::config_coverage::source::{calls, csharp, workspace};
    use crate::config_coverage::{OTHER_PAGES, Ours, PANELS};

    /// Whitespace collapsed, as the ledger compares arguments.
    fn collapse(text: &str) -> String {
        text.split_whitespace().collect::<Vec<_>>().join(" ")
    }

    /// The text inside the parentheses that open at `open`, and where they close.
    fn parenthesised(source: &str, open: usize) -> (String, usize) {
        let bytes = source.as_bytes();
        let mut depth = 0_i32;
        let mut quoted = false;
        let mut at = open;
        while at < bytes.len() {
            match (quoted, bytes[at]) {
                (true, b'\\') => at += 1,
                (true, b'"') | (false, b'"') => quoted = !quoted,
                (false, b'(') => depth += 1,
                (false, b')') => {
                    depth -= 1;
                    if depth == 0 {
                        return (collapse(&source[open + 1..at]), at);
                    }
                }
                _ => {}
            }
            at += 1;
        }
        panic!("the parentheses at {open} do not close");
    }

    /// Whether `text` starts with `word`, whole.
    fn keyword(text: &str, word: &str) -> bool {
        text.strip_prefix(word).is_some_and(|rest| {
            rest.starts_with(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
        })
    }

    /// The conditions of the `if`s and `else`s around `offset` in `handler`'s body, outermost
    /// first, an `else` as `!(<its if>)`; a `try`, a loop or a bare block adds nothing.
    fn enclosing(source: &str, handler: &str, offset: usize) -> Vec<String> {
        let start = source
            .find(&format!("private void {handler}("))
            .expect("the handler");
        let body = start + source[start..].find('{').expect("its body");
        let bytes = source.as_bytes();
        let mut blocks: Vec<Vec<String>> = Vec::new();
        let mut pending: Vec<String> = Vec::new();
        let mut last_if: Option<String> = None;
        let mut at = body + 1;
        while at < offset {
            let rest = &source[at..];
            let starts_word = !(bytes[at - 1].is_ascii_alphanumeric() || bytes[at - 1] == b'_');
            if rest.starts_with("//") {
                at += rest.find('\n').unwrap_or(rest.len());
            } else if let Some(string) = rest.strip_prefix('"') {
                at += string.find('"').expect("the string closes") + 2;
            } else if starts_word && keyword(rest, "if") {
                let open = at + rest.find('(').expect("a condition");
                let (condition, close) = parenthesised(source, open);
                pending.push(condition);
                at = close + 1;
            } else if starts_word && keyword(rest, "else") {
                let negated = last_if.clone().expect("an else follows an if");
                pending.push(format!("!({negated})"));
                at += "else".len();
            } else {
                match bytes[at] {
                    b'{' => blocks.push(std::mem::take(&mut pending)),
                    b'}' => last_if = blocks.pop().and_then(|headers| headers.last().cloned()),
                    b';' => {
                        if let Some(last) = pending.last() {
                            last_if = Some(last.clone());
                        }
                        pending.clear();
                    }
                    _ => {}
                }
                at += 1;
            }
        }
        blocks.into_iter().flatten().chain(pending).collect()
    }

    /// The parser reads what it is meant to: nesting, a braceless `if`, an `else`, and a comment
    /// or a string that looks like code.
    #[test]
    fn the_condition_reader_reads_nesting_else_and_comments() {
        let source = "private void Load(object sender)\n{\n  if (a)\n  {\n    // if (x) {\n    \
                      if (b) Call(\"}\");\n    else\n      Call(\";\");\n    if (c) { d(); }\n    \
                      HERE\n  }\n}";
        let here = source.find("HERE").expect("the marker");
        assert_eq!(enclosing(source, "Load", here), ["a"]);
        let first = source.find("Call(\"}").expect("the first call");
        assert_eq!(enclosing(source, "Load", first), ["a", "b"]);
        let second = source.find("Call(\";").expect("the second call");
        assert_eq!(enclosing(source, "Load", second), ["a", "!(b)"]);
    }

    /// Every call is the C#'s, in order, with the class it adds, the `if`s around it and the
    /// argument that says when it is added; on CONFIG, which calls keep `start` and which add an
    /// advanced page.
    #[test]
    fn every_call_and_its_conditions_are_the_csharps() {
        for list in List::ALL {
            let Some(source) = csharp(list.source()) else {
                eprintln!("skipped: the C# tree is not checked out here");
                return;
            };
            let calls = calls(&source, list.handler());
            let theirs: Vec<(u32, &str)> = calls
                .iter()
                .map(|call| (call.line, call.class.as_str()))
                .collect();
            let ours: Vec<(u32, &str)> = entries(list)
                .iter()
                .map(|entry| (entry.line, entry.class))
                .collect();
            assert_eq!(ours, theirs, "{}'s calls", list.menu());
            for (entry, call) in entries(list).iter().zip(&calls) {
                let at = format!("{} line {}", list.menu(), call.line);
                let guards: Vec<String> = entry.guards.iter().map(|guard| guard.csharp()).collect();
                assert_eq!(
                    guards,
                    enclosing(&source, list.handler(), call.offset),
                    "{at}: the ifs around it"
                );
                match list {
                    List::Setup => {
                        assert_eq!(
                            call.args.get(2).map_or("true", String::as_str),
                            entry.enabled.csharp(),
                            "{at}: enabled"
                        );
                        assert_eq!(call.args.get(4), None, "{at}: advanced");
                        assert!(!entry.advanced && !entry.start, "{at}");
                    }
                    List::Config => {
                        assert_eq!(entry.enabled, Enabled::Always, "{at}");
                        assert_eq!(
                            call.args.get(3).is_some_and(|advanced| advanced == "true"),
                            entry.advanced,
                            "{at}: advanced"
                        );
                        assert_eq!(call.prefix == "start =", entry.start, "{at}: start");
                    }
                }
            }
        }
    }

    /// Every call has the ledger's title and, where it has one, a heading that is a call of the
    /// same list - so the list draws the `.resx`'s words under the C#'s headings.
    #[test]
    fn every_call_has_the_ledgers_title_and_heading() {
        for list in List::ALL {
            for (index, entry) in entries(list).iter().enumerate() {
                let (listed, panel) = crate::config_coverage::listing(list, entry.line)
                    .unwrap_or_else(|| {
                        panic!("{} line {} is not in the ledger", list.menu(), entry.line)
                    });
                assert_eq!(panel.class, entry.class);
                assert!(!title(list, index).is_empty());
                assert_eq!(listed.parent.is_some(), heading(list, index).is_some());
            }
        }
        let headings: Vec<&str> = SETUP_LIST
            .iter()
            .enumerate()
            .filter_map(|(index, _)| heading(List::Setup, index))
            .map(|index| SETUP_LIST[index].class)
            .collect::<std::collections::BTreeSet<_>>()
            .into_iter()
            .collect();
        assert_eq!(
            headings,
            ["ConfigAdvanced", "ConfigMandatory", "ConfigOptional"]
        );
    }

    /// Every switch a list reads is one `DisplayView` has (`display_view.rs` holds the table to
    /// the C#), and the view the application starts in is Mission Planner's Advanced one.
    #[test]
    fn the_display_view_is_mission_planners_advanced_view() {
        let Some(main) = csharp("MainV2.cs") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        assert!(main.contains(": new DisplayView().Advanced();"));
        assert!(main.contains("BackstageView.Advanced = DisplayConfiguration.isAdvancedMode;"));
        assert_eq!(
            crate::display_view::current(),
            crate::display_view::DisplayView::advanced()
        );
        for list in List::ALL {
            for entry in entries(list) {
                for guard in entry.guards {
                    if let Guard::Display(flag) = guard {
                        assert!(
                            crate::display_view::PROPERTIES
                                .iter()
                                .any(|(name, ..)| name == flag),
                            "{flag} is read and not in the table"
                        );
                    }
                }
            }
        }
    }

    /// A Custom view that turns Standard and Advanced Params on, chosen, and the CONFIG list built
    /// again: both pages listed - Advanced Params only while `isAdvancedMode` is on.
    #[test]
    fn a_custom_view_lists_the_parameter_pages() {
        let params = parameters(&["OSD_TYPE"]);
        let vehicle = copter(&params);
        let dir = std::env::temp_dir().join(format!(
            "headless-planner-setup-view-{}",
            std::process::id()
        ));
        let _ = std::fs::create_dir_all(&dir);
        let mut settings = crate::settings::Persisted::at(Some(dir.join("config.xml")));
        let file = dir.join(crate::display_view::CUSTOM_FILE);
        let _ = std::fs::write(
            &file,
            "{\"displayStandardParams\": true, \"displayAdvancedParams\": true, \"isAdvancedMode\": true}",
        );
        let custom = crate::display_view::DisplayView::custom(Some(&file));
        crate::display_view::set(custom, &mut settings);
        let mut config = Backstage::new(List::Config);
        config.load(&vehicle, KEY);
        let shown = classes(&config.rows());
        assert!(shown.contains(&"ConfigFriendlyParams"), "{shown:?}");
        assert!(shown.contains(&"ConfigFriendlyParamsAdv"), "{shown:?}");
        // Without the Advanced mode, the advanced page is added and not drawn.
        let _ = std::fs::write(
            &file,
            "{\"displayStandardParams\": true, \"displayAdvancedParams\": true}",
        );
        let custom = crate::display_view::DisplayView::custom(Some(&file));
        crate::display_view::set(custom, &mut settings);
        config.close();
        config.load(&vehicle, KEY);
        assert!(config.built.items.iter().any(|(index, _)| {
            CONFIG_LIST
                .get(*index)
                .is_some_and(|e| e.class == "ConfigFriendlyParamsAdv")
        }));
        assert!(!classes(&config.rows()).contains(&"ConfigFriendlyParamsAdv"));
        assert!(classes(&config.rows()).contains(&"ConfigFriendlyParams"));
        // Basic: neither, and SETUP loses its Terminal.
        crate::display_view::set(crate::display_view::DisplayView::basic(), &mut settings);
        config.close();
        config.load(&vehicle, KEY);
        assert!(!classes(&config.pages()).contains(&"ConfigFriendlyParams"));
        let mut setup = Backstage::new(List::Setup);
        setup.load(&vehicle, KEY);
        assert!(!classes(&setup.pages()).contains(&"ConfigTerminal"));
        crate::display_view::set(crate::display_view::DisplayView::advanced(), &mut settings);
        let _ = std::fs::remove_dir_all(&dir);
    }

    const KEY: Key = Key {
        connected: true,
        vehicle: None,
        params_fetched: true,
    };

    fn parameters(names: &[&str]) -> Vec<(String, f64)> {
        names.iter().map(|name| ((*name).to_owned(), 1.0)).collect()
    }

    /// A copter whose parameters are all in.
    fn copter(parameters: &[(String, f64)]) -> Vehicle<'_> {
        Vehicle {
            connected: true,
            got_all_params: true,
            firmware: Firmware::ArduCopter2,
            mav_type: 2,
            version: (4, 5),
            capabilities: 0,
            parameters,
        }
    }

    /// No vehicle.
    fn disconnected() -> Vehicle<'static> {
        Vehicle {
            connected: false,
            got_all_params: true,
            firmware: Firmware::ArduCopter2,
            mav_type: 0,
            version: (0, 0),
            capabilities: 0,
            parameters: &[],
        }
    }

    fn loaded(list: List, vehicle: &Vehicle) -> Backstage {
        let mut backstage = Backstage::new(list);
        backstage.load(vehicle, KEY);
        backstage
    }

    fn classes(rows: &[Row]) -> Vec<&'static str> {
        rows.iter().map(|row| row.class).collect()
    }

    fn labels(rows: &[Row]) -> Vec<String> {
        rows.iter().map(|row| row.label.clone()).collect()
    }

    fn index_of(list: List, class: &str) -> usize {
        entries(list)
            .iter()
            .position(|entry| entry.class == class)
            .unwrap_or_else(|| panic!("{class} is not in the list"))
    }

    /// What Mission Planner lists for a copter of 3.5 or later with every parameter in: the
    /// connected firmware page, the three headings with their pages - ADSB last under Mandatory
    /// Hardware, where its late call puts it - and neither the tracker's nor a heli's pages.
    const COPTER: [&str; 39] = [
        "ConfigFirmwareDisabled",
        "ConfigMandatory",
        "ConfigFrameClassType",
        "ConfigAccelerometerCalibration",
        "ConfigHWCompass2",
        "ConfigRadioInput",
        "ConfigRadioOutput",
        "ConfigSerial",
        "ConfigESCCalibration",
        "ConfigFlightModes",
        "ConfigFailSafe",
        "ConfigInitialParams",
        "ConfigHWIDs",
        "ConfigADSB",
        "ConfigOptional",
        "ConfigSerialInjectGPS",
        "ConfigCubeID",
        "Sikradio",
        "ConfigGPSOrder",
        "ConfigBatteryMonitoring",
        "ConfigBatteryMonitoring2",
        "ConfigDroneCAN",
        "JoystickSetup",
        "ConfigCompassMot",
        "ConfigHWRangeFinder",
        "ConfigHWAirspeed",
        "ConfigHWPX4Flow",
        "ConfigHWOptFlow",
        "ConfigHWOSD",
        "ConfigMount",
        "ConfigMotorTest",
        "ConfigHWBT",
        "ConfigHWParachute",
        "ConfigHWESP8266",
        "TrackerUI",
        "ConfigFFT",
        "ConfigAdvanced",
        "ConfigTerminal",
        "ConfigREPL",
    ];

    #[test]
    fn a_copter_lists_what_mission_planner_lists() {
        let params = parameters(&["FRAME_CLASS", "COMPASS_PRIO1_ID"]);
        let setup = loaded(List::Setup, &copter(&params));
        assert_eq!(classes(&setup.pages()), COPTER);
        let titles: Vec<&str> = setup
            .pages()
            .iter()
            .map(|row| title(List::Setup, row.index))
            .collect();
        assert_eq!(
            titles.join(","),
            "Install Firmware,Mandatory Hardware,Frame Type,Accel Calibration,Compass,Radio \
             Calibration,Servo Output,Serial Ports,ESC Calibration,Flight Modes,FailSafe,Initial \
             Tune Parameter,HW ID,ADSB,Optional Hardware,RTK/GPS Inject,CubeID Update,Sik \
             Radio,CAN GPS Order,Battery Monitor,Battery Monitor 2,DroneCAN/UAVCAN,Joystick,\
             Compass/Motor Calib,Range Finder,Airspeed,PX4Flow,Optical Flow,OSD,Camera \
             Gimbal,Motor Test,Bluetooth Setup,Parachute,ESP8266 Setup,Antenna Tracker,FFT \
             Setup,Advanced,Terminal,Script REPL"
        );
        // With every heading closed, as the screen opens.
        assert_eq!(
            labels(&setup.rows()),
            [
                "Install Firmware",
                ">> Mandatory Hardware",
                ">> Optional Hardware",
                ">> Advanced"
            ]
        );
    }

    /// The script checks the same list against SITL, so the two cannot drift apart.
    #[test]
    fn the_scripts_copter_list_is_this_one() {
        let script = std::fs::read_to_string(workspace().join("tests/gui/setup-list.gui"))
            .expect("tests/gui/setup-list.gui");
        let listed = script
            .lines()
            .filter_map(|line| line.strip_prefix("expect setup.pages "))
            .find(|value| !value.starts_with('~'))
            .expect("the script checks setup.pages whole");
        assert_eq!(listed.trim(), COPTER.join(","));
    }

    #[test]
    fn the_vehicle_picks_the_frame_and_compass_pages() {
        // Before 3.5 and with no FRAME_CLASS: the old frame page.
        let none = parameters(&[]);
        let old = Vehicle {
            version: (3, 4),
            ..copter(&none)
        };
        let pages = classes(&loaded(List::Setup, &old).pages());
        assert!(pages.contains(&"ConfigFrameType"));
        assert!(!pages.contains(&"ConfigFrameClassType"));
        // No COMPASS_PRIO1_ID: the older compass page.
        assert!(pages.contains(&"ConfigHWCompass"));
        assert!(!pages.contains(&"ConfigHWCompass2"));

        // A heli has its own page too.
        let params = parameters(&["FRAME_CLASS"]);
        let heli = Vehicle {
            mav_type: TYPE_HELICOPTER,
            ..copter(&params)
        };
        let pages = classes(&loaded(List::Setup, &heli).pages());
        assert!(pages.contains(&"ConfigTradHeli4"));

        // A plane has no Initial Tune Parameter unless it is a quadplane.
        let plane = Vehicle {
            firmware: Firmware::ArduPlane,
            ..copter(&none)
        };
        assert!(!classes(&loaded(List::Setup, &plane).pages()).contains(&"ConfigInitialParams"));
        let quad = vec![("Q_ENABLE".to_owned(), 1.0)];
        let quadplane = Vehicle {
            firmware: Firmware::ArduPlane,
            ..copter(&quad)
        };
        assert!(classes(&loaded(List::Setup, &quadplane).pages()).contains(&"ConfigInitialParams"));

        // A tracker has its own page under Optional Hardware.
        let tracker = Vehicle {
            firmware: Firmware::ArduTracker,
            ..copter(&none)
        };
        assert!(classes(&loaded(List::Setup, &tracker).pages()).contains(&"ConfigAntennaTracker"));
    }

    #[test]
    fn without_a_vehicle_the_lists_are_the_disconnected_ones() {
        let setup = loaded(List::Setup, &disconnected());
        assert_eq!(
            classes(&setup.pages()),
            [
                "ConfigFirmwareManifest",
                "ConfigFirmware",
                "ConfigSecureAP",
                "ConfigOptional",
                "ConfigSerialInjectGPS",
                "Sikradio",
                "ConfigDroneCAN",
                "JoystickSetup",
                "ConfigHWPX4Flow",
                "ConfigHWBT",
                "TrackerUI",
                "ConfigAdvanced",
                "ConfigTerminal",
            ]
        );
        let mut config = Backstage::new(List::Config);
        let start = config.load(&disconnected(), KEY);
        assert_eq!(
            classes(&config.pages()),
            ["ConfigRawParams", "ConfigPlanner"]
        );
        // `start` is the disconnected Planner, the call at 257.
        assert_eq!(start.map(|index| CONFIG_LIST[index].line), Some(257));
    }

    #[test]
    fn while_parameters_arrive_the_lists_offer_the_loading_page() {
        let none = parameters(&[]);
        let arriving = Vehicle {
            got_all_params: false,
            ..copter(&none)
        };
        let setup = loaded(List::Setup, &arriving);
        let pages = classes(&setup.pages());
        assert_eq!(pages.first(), Some(&"ConfigParamLoading"));
        assert!(!pages.contains(&"ConfigMandatory"));
        assert!(!pages.contains(&"ConfigFlightModes"));
        assert!(pages.contains(&"ConfigCubeID"));

        let mut config = Backstage::new(List::Config);
        let start = config.load(&arriving, KEY);
        assert_eq!(
            classes(&config.pages()),
            ["ConfigParamLoading", "ConfigPlanner"]
        );
        assert_eq!(start.map(|index| CONFIG_LIST[index].line), Some(243));

        // An Ateryx's Flight Modes is `start` already, so the Loading page is the `else`'s call.
        let ateryx = Vehicle {
            firmware: Firmware::Ateryx,
            ..arriving
        };
        let mut config = Backstage::new(List::Config);
        let start = config.load(&ateryx, KEY);
        let lines: Vec<u32> = config
            .pages()
            .iter()
            .map(|row| CONFIG_LIST[row.index].line)
            .collect();
        assert_eq!(lines, [235, 236, 237, 245, 250]);
        assert_eq!(start.map(|index| CONFIG_LIST[index].line), Some(235));
    }

    #[test]
    fn a_copters_config_list_opens_on_basic_tuning() {
        let params = parameters(&["OSD_TYPE"]);
        let vehicle = Vehicle {
            capabilities: CAPABILITY_FTP,
            ..copter(&params)
        };
        let mut config = Backstage::new(List::Config);
        let start = config.load(&vehicle, KEY);
        assert_eq!(
            classes(&config.pages()),
            [
                "ConfigAC_Fence",
                "ConfigSimplePids",
                "ConfigArducopter",
                "ConfigOSD",
                "MavFTPUI",
                "ConfigUserDefined",
                "ConfigRawParams",
                "ConfigPlanner",
            ]
        );
        assert_eq!(
            start.map(|index| CONFIG_LIST[index].class),
            Some("ConfigSimplePids")
        );
        // No headings: every page is drawn.
        assert_eq!(config.rows(), config.pages());
    }

    #[test]
    fn a_heading_opens_and_closes_and_a_page_under_a_closed_one_opens_it() {
        let params = parameters(&["FRAME_CLASS", "COMPASS_PRIO1_ID"]);
        let mut setup = loaded(List::Setup, &copter(&params));
        let mandatory = index_of(List::Setup, "ConfigMandatory");
        let modes = index_of(List::Setup, "ConfigFlightModes");

        assert_eq!(setup.activate(mandatory), None);
        let shown = labels(&setup.rows());
        assert_eq!(
            shown.get(1).map(String::as_str),
            Some(">> Mandatory Hardware")
        );
        assert_eq!(shown.get(2).map(String::as_str), Some("      Frame Type"));
        assert!(shown.contains(&"      Flight Modes".to_owned()));
        assert_eq!(
            setup.page().map(|entry| entry.class),
            Some("ConfigMandatory")
        );

        // A page under it: the heading stays open, and the heading's page is deactivated.
        assert_eq!(setup.activate(modes), Some(mandatory));
        assert!(
            setup
                .rows()
                .iter()
                .any(|row| row.index == modes && row.selected)
        );

        // The heading again closes it, and it is the page showing.
        assert_eq!(setup.activate(mandatory), Some(modes));
        assert_eq!(setup.rows().len(), 4);

        // A page under a closed heading opens it.
        assert_eq!(setup.activate(modes), Some(mandatory));
        assert_eq!(setup.expanded, [mandatory]);

        // The page showing, chosen again, is deactivated and activated again.
        assert_eq!(setup.activate(modes), Some(modes));
    }

    #[test]
    fn the_page_showing_when_the_screen_closed_is_chosen_again() {
        let params = parameters(&["FRAME_CLASS", "COMPASS_PRIO1_ID"]);
        let modes = index_of(List::Setup, "ConfigFlightModes");
        let mut setup = Backstage::new(List::Setup);
        assert_eq!(setup.load(&copter(&params), KEY), None);
        setup.activate(modes);
        assert_eq!(setup.close(), Some(modes));
        assert_eq!(setup.loaded(), None);
        assert_eq!(setup.load(&copter(&params), KEY), Some(modes));
        setup.activate(modes);

        // Shown again without it listed, nothing is chosen, and the name is kept for when it is.
        assert_eq!(setup.close(), Some(modes));
        assert_eq!(setup.load(&disconnected(), KEY), None);
        assert_eq!(setup.close(), None);
        assert_eq!(setup.load(&copter(&params), KEY), Some(modes));

        // On CONFIG the remembered page beats `start`.
        let mut config = Backstage::new(List::Config);
        let raw = index_of(List::Config, "ConfigRawParams");
        assert_ne!(config.load(&copter(&params), KEY), Some(raw));
        config.activate(raw);
        config.close();
        assert_eq!(config.load(&copter(&params), KEY), Some(raw));
    }

    /// The link opening under Install Firmware while Force Bootloader holds it (`held`) keeps the
    /// manifest page, the list keyed to the link as it now is - `Open(false)` is not `doConnect`
    /// and shows no screen again - where without it the same change shows the screen again. The
    /// link closing once Force Bootloader is done shows it again, as the C#'s heartbeat loop does
    /// for a port gone; leaving the screen closes it whatever.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:517; MainV2.cs:1419-1425, 1740-1748, 2973-2997`
    #[test]
    fn force_bootloaders_link_shows_no_screen_again_while_it_holds_it() {
        let mut view = TelemetryView::disconnected("serial:/dev/ttyACM0:115200");
        let idle = Key::of(&view);
        view.connected = true;
        view.vehicle = Some(VehicleId::new(1, 1));
        let heard = Key::of(&view);
        let manifest = index_of(List::Setup, "ConfigFirmwareManifest");

        let mut setup = Backstage::new(List::Setup);
        setup.load(&disconnected(), idle);
        setup.activate(manifest);
        assert!(!setup.stale(idle, true, true, false), "nothing moved");
        assert!(!setup.stale(heard, true, true, true), "held: kept");
        assert_eq!(setup.loaded(), Some(heard), "keyed to the link as it is");
        assert_eq!(
            setup.page().map(|entry| entry.class),
            Some("ConfigFirmwareManifest")
        );
        assert!(
            !setup.stale(heard, true, true, false),
            "done, and still open: kept"
        );
        assert!(
            setup.stale(idle, true, true, false),
            "the link closed: shown again"
        );
        assert!(setup.stale(idle, false, true, true), "the screen left");

        // Not held: the link opening shows the screen again.
        let mut setup = Backstage::new(List::Setup);
        setup.load(&disconnected(), idle);
        setup.activate(manifest);
        assert!(setup.stale(heard, true, true, false));
        assert_eq!(setup.loaded(), Some(idle));

        // A list not built is never stale, and is not keyed.
        let mut unbuilt = Backstage::new(List::Setup);
        assert!(!unbuilt.stale(heard, true, true, true));
        unbuilt.rekey(heard);
        assert_eq!(unbuilt.loaded(), None);
    }

    #[test]
    fn a_banner_names_the_version() {
        assert_eq!(banner_version("ArduCopter V4.5.7 (2a3dc4b7)"), Some((4, 5)));
        assert_eq!(banner_version("ArduPlane V4.6.0-dev (abc)"), Some((4, 6)));
        assert_eq!(banner_version("ArduCopter"), None);
    }

    /// The key a list is built for moves when the link opens, when the vehicle changes and when
    /// the connect-time download ends (`doConnect` shows the screen again after `getParamList`),
    /// and not when a parameter arrives, changes, or the vehicle raises its count afterwards.
    /// `// C#: MainV2.cs:1684, 1740-1748`
    #[test]
    fn the_key_moves_when_the_download_ends() {
        let mut view = TelemetryView::disconnected("tcp:127.0.0.1:5760");
        let idle = Key::of(&view);
        view.connected = true;
        view.vehicle = Some(VehicleId::new(1, 1));
        let heard = Key::of(&view);
        assert_ne!(idle, heard, "the link opened");

        view.parameters = parameters(&["FRAME_CLASS"]).into();
        view.parameters_expected = 3;
        let arriving = Key::of(&view);
        assert_eq!(heard, arriving, "a parameter arriving is not a reload");
        view.parameters = parameters(&["FRAME_CLASS", "FRAME_TYPE", "ARMING_CHECK"]).into();
        assert_eq!(arriving, Key::of(&view), "nor the last of them");

        view.parameters_fetched = true;
        let fetched = Key::of(&view);
        assert_ne!(arriving, fetched, "the download ended: the list is built again");
        // AVD_ENABLE 1: the vehicle now reports more than are held. Not a reload.
        view.parameters_expected = 5;
        assert_eq!(fetched, Key::of(&view), "a raised count is not a reload");
        view.parameters = parameters(&["FRAME_CLASS", "FRAME_TYPE", "ARMING_CHECK"]).into();
        assert_eq!(fetched, Key::of(&view), "a value changing is not a reload");
    }

    /// A view's vehicle: connected once heard from, every parameter in only when some are, and
    /// the version from `AUTOPILOT_VERSION` or, without one, the banner.
    #[test]
    fn a_view_gives_the_vehicle_the_conditions_read() {
        let mut view = TelemetryView::disconnected("tcp:127.0.0.1:5760");
        assert!(!Vehicle::of(&view, None).connected);

        view.connected = true;
        view.vehicle = Some(VehicleId::new(1, 1));
        let mut state = VehicleState::default();
        state.autopilot = 3;
        state.vehicle_type = 2;
        view.state = Some(Arc::new(state));
        let vehicle = Vehicle::of(&view, Some("ArduCopter V4.5.7 (2a3dc4b7)"));
        assert!(vehicle.connected);
        assert!(!vehicle.got_all_params, "none downloaded yet");
        assert_eq!(vehicle.firmware, Firmware::ArduCopter2);
        assert_eq!(vehicle.version, (4, 5));

        view.parameters = parameters(&["FRAME_CLASS"]).into();
        view.parameters_expected = 2;
        assert!(!Vehicle::of(&view, None).got_all_params, "one of two");
        view.parameters_expected = 1;
        assert!(Vehicle::of(&view, None).got_all_params);

        state.autopilot_info.version = [4, 6, 0, 255];
        view.state = Some(Arc::new(state));
        assert_eq!(
            Vehicle::of(&view, Some("ArduCopter V4.5.7")).version,
            (4, 6)
        );
    }

    /// Each page the ledger says this application has - done or in part - is drawn when chosen,
    /// by an arm of `page_body`; and no arm names a page the lists lack.
    #[test]
    fn every_page_the_ledger_claims_is_drawn() {
        let source = include_str!("setup.rs");
        let start = source.find("fn page_body(").expect("page_body");
        let end = start
            + source[start..]
                .find("_ => return None")
                .expect("its last arm");
        let mut arms: Vec<&str> = Vec::new();
        for line in source[start..end].lines() {
            let line = line.trim();
            let Some((pattern, _)) = line.split_once(" =>") else {
                continue;
            };
            for class in pattern.split(" | ") {
                if let Some(class) = class.strip_prefix('"').and_then(|c| c.strip_suffix('"')) {
                    arms.push(class);
                }
            }
        }
        for class in &arms {
            assert!(
                List::ALL
                    .iter()
                    .any(|list| entries(*list).iter().any(|entry| entry.class == *class)),
                "{class} is drawn and no list adds it"
            );
        }
        for panel in PANELS.iter().chain(OTHER_PAGES) {
            if matches!(panel.ours, Ours::Done(_) | Ours::Partial(..)) {
                assert!(
                    arms.contains(&panel.class),
                    "{} is claimed and not drawn",
                    panel.class
                );
            }
        }
    }
}
