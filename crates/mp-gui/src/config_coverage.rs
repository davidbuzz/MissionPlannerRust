//! Which of Mission Planner's setup and configuration pages this application has (DELIVERABLES.md
//! D12).
//!
//! D12's definition of done is "every C# config panel enumerated with a checked-in coverage ledger
//! at 100 %". The panels are the `Config*.cs` user controls in `GCSViews/ConfigurationView/`, and
//! two screens list them down their left-hand side: `GCSViews/InitialSetup.cs`, which `MainV2`'s
//! SETUP button opens, and `GCSViews/SoftwareConfig.cs`, which its CONFIG button opens (C#:
//! `MainV2.cs:3179`, `MainV2.cs:3180`, `MainV2.Designer.cs:150`, `MainV2.Designer.cs:158`). Each
//! screen builds its list in its `Load` handler with one `AddBackstageViewPage(typeof(...), title,
//! ...)` per page, most of them behind a vehicle check, and the order of those calls is the order
//! of the list.
//!
//! [`PANELS`] is one row per panel, in the order the two lists first add it: where each list adds
//! it (the line of the call, which is this table's citation of the C#), the title the list shows,
//! the heading it sits under, the vehicles it is shown for, how many events its Designer wires,
//! and what stands in for it here. Panels neither list adds come last. [`OTHER_PAGES`] holds the
//! four pages the lists add that are not in `ConfigurationView/`, so the lists are complete; they
//! are not counted as panels. The report renders to `docs/coverage/configuration.md`, as
//! `coverage.rs` and `planner_coverage.rs` do for the flight and planning screens. The SETUP and
//! CONFIG screens (`setup.rs`) draw their lists with this table's titles and headings, looked up
//! by the line of each call ([`listing`]).
//!
//! The tests hold the table to the C#. When the tree is present, every `Config*.cs` in the
//! directory must be a row and every row a file there, declaring the class the row names; each
//! wiring count must be its Designer's; and the listings must be the `AddBackstageViewPage` calls
//! of the two `Load` handlers, line for line and in order, under the same headings, with the title
//! each call's argument resolves to in `InitialSetup.resx` or `Strings.resx`. Every Rust path a row
//! claims must exist and hold what it names. And the committed report must match the table.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]
// The binary reads the counts, and the titles and headings the SETUP and CONFIG lists draw; the
// vehicles and the rest are read by the report, which is built by the tests.
#![cfg_attr(not(test), allow(dead_code))]

/// The two screens that list the panels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Screen {
    /// `GCSViews/InitialSetup.cs`, behind `MainV2`'s SETUP button.
    Setup,
    /// `GCSViews/SoftwareConfig.cs`, behind `MainV2`'s CONFIG button.
    Config,
}

impl Screen {
    /// The two, in `MainV2`'s menu order.
    pub const ALL: [Self; 2] = [Self::Setup, Self::Config];

    /// The menu button's text in `MainV2.resx`.
    #[must_use]
    pub const fn menu(self) -> &'static str {
        match self {
            // C#: MainV2.resx:506
            Self::Setup => "SETUP",
            // C#: MainV2.resx:567
            Self::Config => "CONFIG",
        }
    }

    /// The file that builds the list, relative to the C# tree.
    #[must_use]
    pub const fn source(self) -> &'static str {
        match self {
            Self::Setup => "GCSViews/InitialSetup.cs",
            Self::Config => "GCSViews/SoftwareConfig.cs",
        }
    }

    /// The `Load` handler that builds it.
    #[must_use]
    pub const fn handler(self) -> &'static str {
        match self {
            // C#: GCSViews/InitialSetup.cs:155
            Self::Setup => "HardwareConfig_Load",
            // C#: GCSViews/SoftwareConfig.cs:142
            Self::Config => "SoftwareConfig_Load",
        }
    }
}

/// One `AddBackstageViewPage` call: where a list adds a page.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Listed {
    /// Which screen's list.
    pub screen: Screen,
    /// The line of the call in [`Screen::source`].
    pub line: u32,
    /// The title the list shows, as the call's argument resolves in the `.resx`.
    pub title: &'static str,
    /// The class of the heading it is listed under, if any.
    pub parent: Option<&'static str>,
    /// Which vehicles it is shown for; the words are defined in the report's key.
    pub vehicles: &'static str,
}

/// A place in this application's source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct At {
    /// The file, relative to the workspace root.
    pub file: &'static str,
    /// `fn name` for a function, or a control id that appears quoted in the file.
    pub item: &'static str,
}

/// What stands in for a C# panel here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ours {
    /// Everything the panel does, here.
    Done(At),
    /// Some of it, here; the text says what is and what is missing.
    Partial(At, &'static str),
    /// Not implemented.
    Missing,
    /// No function of its own: what it is.
    Plumbing(&'static str),
    /// Deliberately not carried over, with the reason.
    Dropped(&'static str),
}

/// One page.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Panel {
    /// The C# class.
    pub class: &'static str,
    /// Its source file, relative to the C# tree.
    pub file: &'static str,
    /// Event wirings in its Designer, or `None` where it has no Designer.
    pub wirings: Option<usize>,
    /// Where the lists add it, SETUP's first and each list's in line order; empty for none.
    pub listed: &'static [Listed],
    /// What this application has for it.
    pub ours: Ours,
}

const fn panel(
    class: &'static str,
    file: &'static str,
    wirings: Option<usize>,
    listed: &'static [Listed],
    ours: Ours,
) -> Panel {
    Panel {
        class,
        file,
        wirings,
        listed,
        ours,
    }
}

const fn setup(
    line: u32,
    title: &'static str,
    parent: Option<&'static str>,
    vehicles: &'static str,
) -> Listed {
    Listed {
        screen: Screen::Setup,
        line,
        title,
        parent,
        vehicles,
    }
}

/// `SoftwareConfig`'s list has no headings: every call passes `null` for the parent.
const fn config(line: u32, title: &'static str, vehicles: &'static str) -> Listed {
    Listed {
        screen: Screen::Config,
        line,
        title,
        parent: None,
        vehicles,
    }
}

const fn at(file: &'static str, item: &'static str) -> At {
    At { file, item }
}

/// A file in `GCSViews/ConfigurationView/`.
macro_rules! cv {
    ($name:literal) => {
        concat!("GCSViews/ConfigurationView/", $name, ".cs")
    };
}

use Ours::{Missing, Partial, Plumbing};

// The headings of `InitialSetup`'s list: the page each `var mand`, `var opt` and `var adv` holds.
const TOP: Option<&str> = None;
const MANDATORY: Option<&str> = Some("ConfigMandatory");
const OPTIONAL: Option<&str> = Some("ConfigOptional");
const ADVANCED: Option<&str> = Some("ConfigAdvanced");

// The vehicles words. Most `InitialSetup` calls pass `isConnected && gotAllParams`: a vehicle is
// connected and its parameter list is whole (C#: GCSViews/InitialSetup.cs:64, :119). `true`, or no
// argument, adds the page connected or not. `SoftwareConfig`'s calls from GeoFence to User Params
// sit inside `if (gotAllParams)` and `if (MainV2.comPort.BaseStream.IsOpen)` (C#:
// GCSViews/SoftwareConfig.cs:148, :150); the rest check the link themselves.
const ANY: &str = "any";
const ALWAYS: &str = "always";
const CONNECTED: &str = "connected";
const DISCONNECTED: &str = "disconnected";
const LOADING: &str = "connected, parameters still arriving";

const SETUP_RS: &str = "crates/mp-gui/src/setup.rs";
const PARAMS_RS: &str = "crates/mp-gui/src/params.rs";
const JOYSTICK_RS: &str = "crates/mp-gui/src/joystick.rs";
const COMPASS_RS: &str = "crates/mp-gui/src/config/compass.rs";

/// Every `Config*.cs` in `GCSViews/ConfigurationView/`, in the order `InitialSetup` and then
/// `SoftwareConfig` first list it, then the ones neither lists.
pub const PANELS: &[Panel] = &[
    // ---- SETUP: InitialSetup.HardwareConfig_Load, GCSViews/InitialSetup.cs:155 ----
    panel(
        "ConfigParamLoading",
        cv!("ConfigParamLoading"),
        Some(2),
        &[
            setup(162, "Loading", TOP, LOADING),
            config(243, "Loading", LOADING),
            config(245, "Loading", LOADING),
        ],
        // C#: GCSViews/ConfigurationView/ConfigParamLoading.cs:34-53 - the label, Retry Now,
        // and the timer that reloads the screen once the list is whole.
        Ours::Done(at(SETUP_RS, "fn param_loading_page")),
    ),
    panel(
        "ConfigFirmwareDisabled",
        cv!("ConfigFirmwareDisabled"),
        Some(1),
        &[setup(169, "Install Firmware", TOP, CONNECTED)],
        // C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.cs, on the Install Firmware page.
        Partial(
            at("crates/mp-gui/src/config/firmware.rs", "fn page"),
            "the connected page's text, with Bootloader Update disabled",
        ),
    ),
    panel(
        "ConfigFirmwareManifest",
        cv!("ConfigFirmwareManifest"),
        Some(16),
        &[setup(171, "Install Firmware", TOP, DISCONNECTED)],
        // C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs, on the Install Firmware page.
        Partial(
            at("crates/mp-gui/src/config/firmware.rs", "fn page"),
            "the catalogue fetched as APFirmware.GetList fetches it, each vehicle labelled with the newest firmware of the release, Beta, a vehicle's click running LookForPort to the chosen file; Upload disabled - nothing flashes in this build; the vehicle pictures are named boxes",
        ),
    ),
    panel(
        "ConfigFirmware",
        cv!("ConfigFirmware"),
        Some(20),
        &[setup(173, "Install Firmware Legacy", TOP, DISCONNECTED)],
        Missing,
    ),
    panel(
        "ConfigSecureAP",
        cv!("ConfigSecureAP"),
        Some(4),
        &[setup(178, "Secure", TOP, DISCONNECTED)],
        Missing,
    ),
    panel(
        "ConfigMandatory",
        cv!("ConfigMandatory"),
        Some(0),
        &[setup(182, "Mandatory Hardware", TOP, ANY)],
        // C#: GCSViews/ConfigurationView/ConfigMandatory.resx, label1.Text
        Plumbing("the Mandatory Hardware heading of the list: one sentence, no controls"),
    ),
    panel(
        "ConfigTradHeli4",
        cv!("ConfigTradHeli4"),
        Some(0),
        &[setup(187, "Heli Setup", MANDATORY, "heli")],
        Missing,
    ),
    panel(
        "ConfigFrameType",
        cv!("ConfigFrameType"),
        Some(12),
        &[setup(188, "Frame Type", MANDATORY, "copter before 3.5")],
        Missing,
    ),
    panel(
        "ConfigFrameClassType",
        cv!("ConfigFrameClassType"),
        Some(19),
        &[setup(
            189,
            "Frame Type",
            MANDATORY,
            "any with FRAME_CLASS; copter 3.5 and later",
        )],
        // C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:36-336, ported but for the
        // frame pictures (the C#'s PNG resources, drawn as named boxes), the "Other" button that
        // has no handler, and the pre-3.5 ConfigFrameType page.
        Partial(
            at("crates/mp-gui/src/config/frame_type.rs", "fn page"),
            "the eight class buttons and six type rows from Common.ValidList, each click \
             writing FRAME_CLASS then FRAME_TYPE through the retrying set; the frame pictures \
             are named boxes, not the C#'s images",
        ),
    ),
    panel(
        "ConfigAccelerometerCalibration",
        cv!("ConfigAccelerometerCalibration"),
        Some(3),
        &[setup(196, "Accel Calibration", MANDATORY, ANY)],
        Partial(
            at(SETUP_RS, "fn accelerometer_panel"),
            "has Calibrate Accel's six positions, and Calibrate Level as `cal-level` on the \
             page; missing Simple Accel Cal",
        ),
    ),
    panel(
        "ConfigHWCompass2",
        cv!("ConfigHWCompass2"),
        Some(11),
        &[setup(
            203,
            "Compass",
            MANDATORY,
            "any with COMPASS_PRIO1_ID",
        )],
        // C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:86-532 - the priority table with its
        // up and down writes of COMPASS_PRIO1_ID to PRIO3_ID, Remove Missing, the Use and learn
        // boxes, Reboot and its CheckReboot, the onboard calibration's Start, Accept and Cancel
        // with the timer's bars, lights and text, the fitness combo, and Large Vehicle MagCal.
        Ours::Done(at(COMPASS_RS, "fn page")),
    ),
    panel(
        "ConfigHWCompass",
        cv!("ConfigHWCompass"),
        Some(21),
        &[setup(
            206,
            "Compass",
            MANDATORY,
            "any without COMPASS_PRIO1_ID",
        )],
        // C#: GCSViews/ConfigurationView/ConfigHWCompass.cs:31-780. Shown for ArduPilot before
        // 4.1, which has no COMPASS_PRIO1_ID.
        Partial(
            at(COMPASS_RS, "fn page"),
            "has the declination and its automatic box, learn, the primary compass, each \
             compass's use, external, orientation, offsets and MOT, the three quick-configure \
             buttons, the onboard calibration and Large Vehicle MagCal; missing Live \
             Calibration (MagCalib.DoGUIMagCalib, drawn and inert)",
        ),
    ),
    panel(
        "ConfigRadioInput",
        cv!("ConfigRadioInput"),
        Some(8),
        &[setup(211, "Radio Calibration", MANDATORY, ANY)],
        // C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:42-508, ported whole: the sixteen
        // bars bound through RCMAP_*, Calibrate Radio's message boxes, loop, trims and forced
        // RCn_MIN/_MAX/_TRIM writes with its summary, the Reverse boxes on RCn_REV or
        // RCn_REVERSED (hidden on a copter) with SWITCH_ENABLE, the plane's elevon boxes, the
        // three START_RX_PAIR binds, and the RC_CHANNELS stream requests. The vertical bars'
        // text is a word to a line where WinForms rotates it; requestDatastream's rate check and
        // the cs.rate* fields only CurrentState's stream re-request reads are not carried over.
        Ours::Done(at("crates/mp-gui/src/config/radio.rs", "fn page")),
    ),
    panel(
        "ConfigRadioOutput",
        cv!("ConfigRadioOutput"),
        Some(1),
        &[setup(215, "Servo Output", MANDATORY, ANY)],
        // C#: GCSViews/ConfigurationView/ConfigRadioOutput.cs:11-92, ported whole: sixteen rows
        // or thirty-two with SERVO_32_ENABLE, each with its SERVO_OUTPUT_RAW bar and the
        // Mavlink* controls on SERVOn_REVERSED, _FUNCTION, _MIN, _TRIM and _MAX writing on
        // change - a number typed or stepped, 300 ms after, with MavlinkNumericUpDown's
        // out-of-range question. The C# has no RCn_* fallback, and nor does this. A number's
        // mouse wheel is not carried over.
        Ours::Done(at("crates/mp-gui/src/config/servo_output.rs", "fn page")),
    ),
    panel(
        "ConfigSerial",
        cv!("ConfigSerial"),
        Some(0),
        &[setup(220, "Serial Ports", MANDATORY, ANY)],
        // C#: GCSViews/ConfigurationView/ConfigSerial.cs:37-513, but for the port names, which
        // Activate downloads from @SYS/uarts.txt over MAVLink FTP - a client this application
        // does not have.
        Partial(
            at("crates/mp-gui/src/config/serial_ports.rs", "fn page"),
            "a row per SERIALn to the highest SERIALn_BAUD, the speed and protocol combos \
             writing on change through the page's setParam, SerialOptionRules.json's rules and \
             MAVLink warning in the note, the options label and the Set Bitmask window; missing \
             the port names from @SYS/uarts.txt, which need MAVLink FTP",
        ),
    ),
    panel(
        "ConfigESCCalibration",
        cv!("ConfigESCCalibration"),
        Some(1),
        &[setup(224, "ESC Calibration", MANDATORY, ANY)],
        // C#: GCSViews/ConfigurationView/ConfigESCCalibration.cs:10-45, ported whole: the text
        // with its props warning, Calibrate ESCs setting ESC_CALIBRATION to 3 and disabled once
        // it has, and MOT_PWM_TYPE, MOT_PWM_MIN/MAX and MOT_SPIN_ARM/MIN/MAX writing on change.
        Ours::Done(at("crates/mp-gui/src/config/esc_calibration.rs", "fn page")),
    ),
    panel(
        "ConfigFlightModes",
        cv!("ConfigFlightModes"),
        Some(8),
        &[
            setup(228, "Flight Modes", MANDATORY, ANY),
            config(235, "Flight Modes", "Ateryx"),
        ],
        // C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:38-467, ported whole but for the
        // Ctrl+S shortcut (ProcessCmdKey), standardFlightModesOnly (its default only), and the
        // message box, which is a line on the page.
        Partial(
            at("crates/mp-gui/src/config/flight_modes.rs", "fn page"),
            "the six combos from the firmware's mode list, the lit PWM band, Simple and Super \
             Simple, Save through the retrying set; not Ctrl+S, standardFlightModesOnly beyond \
             its default, nor the message box",
        ),
    ),
    panel(
        "ConfigFailSafe",
        cv!("ConfigFailSafe"),
        Some(4),
        &[setup(232, "FailSafe", MANDATORY, ANY)],
        // C#: GCSViews/ConfigurationView/ConfigFailSafe.cs:24-192, ported whole but for typed
        // numbers - gpui has no numeric up-down, so the step arrows stand in - and the
        // out-of-range prompt that only typed values raise.
        Partial(
            at("crates/mp-gui/src/config/failsafe.rs", "fn page"),
            "the channel bars, the mode/armed/GPS readouts, the throttle, battery and GCS \
             controls writing their parameters on change through the retrying set; numbers by \
             step arrows only, no typing",
        ),
    ),
    panel(
        "ConfigInitialParams",
        cv!("ConfigInitialParams"),
        Some(3),
        &[setup(
            237,
            "Initial Tune Parameter",
            MANDATORY,
            "copter, quadplane",
        )],
        Missing,
    ),
    panel(
        "ConfigHWIDs",
        cv!("ConfigHWIDs"),
        Some(0),
        &[setup(241, "HW ID", MANDATORY, ANY)],
        Missing,
    ),
    panel(
        "ConfigOptional",
        cv!("ConfigOptional"),
        Some(0),
        &[setup(243, "Optional Hardware", TOP, ALWAYS)],
        // C#: GCSViews/ConfigurationView/ConfigOptional.resx, label1.Text
        Plumbing("the Optional Hardware heading of the list: one sentence, no controls"),
    ),
    panel(
        "ConfigSerialInjectGPS",
        cv!("ConfigSerialInjectGPS"),
        Some(24),
        &[setup(251, "RTK/GPS Inject", OPTIONAL, ALWAYS)],
        Missing,
    ),
    panel(
        "ConfigCubeID",
        cv!("ConfigCubeID"),
        Some(2),
        &[setup(254, "CubeID Update", OPTIONAL, CONNECTED)],
        Missing,
    ),
    panel(
        "ConfigADSB",
        cv!("ConfigADSB"),
        Some(5),
        &[setup(263, "ADSB", MANDATORY, ANY)],
        Missing,
    ),
    panel(
        "ConfigGPSOrder",
        cv!("ConfigGPSOrder"),
        Some(1),
        &[setup(266, "CAN GPS Order", OPTIONAL, ANY)],
        Missing,
    ),
    panel(
        "ConfigBatteryMonitoring",
        cv!("ConfigBatteryMonitoring"),
        Some(13),
        &[setup(270, "Battery Monitor", OPTIONAL, ANY)],
        // C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:18-653, ported but for the
        // power-module photo, typing into the Sensor and HW Ver combos, and the speech alert,
        // which lasts a session since nothing writes config.xml.
        Partial(
            at("crates/mp-gui/src/config/battery_monitor.rs", "fn page"),
            "the Monitor, Sensor and HW Ver combos with the nine presets and the pin table, the \
             divider and amps-per-volt arithmetic in single precision, each box writing its \
             parameter on leaving through the retrying set; no photo, no typing into the combos",
        ),
    ),
    panel(
        "ConfigBatteryMonitoring2",
        cv!("ConfigBatteryMonitoring2"),
        Some(10),
        &[setup(271, "Battery Monitor 2", OPTIONAL, ANY)],
        Missing,
    ),
    panel(
        "ConfigDroneCAN",
        cv!("ConfigDroneCAN"),
        Some(15),
        &[setup(276, "DroneCAN/UAVCAN", OPTIONAL, ALWAYS)],
        Missing,
    ),
    panel(
        "ConfigCompassMot",
        cv!("ConfigCompassMot"),
        Some(2),
        &[setup(285, "Compass/Motor Calib", OPTIONAL, ANY)],
        Missing,
    ),
    panel(
        "ConfigHWRangeFinder",
        cv!("ConfigHWRangeFinder"),
        Some(2),
        &[setup(289, "Range Finder", OPTIONAL, ANY)],
        Missing,
    ),
    panel(
        "ConfigHWAirspeed",
        cv!("ConfigHWAirspeed"),
        Some(1),
        &[setup(293, "Airspeed", OPTIONAL, ANY)],
        Missing,
    ),
    panel(
        "ConfigHWPX4Flow",
        cv!("ConfigHWPX4Flow"),
        Some(1),
        &[setup(297, "PX4Flow", OPTIONAL, ALWAYS)],
        Missing,
    ),
    panel(
        "ConfigHWOptFlow",
        cv!("ConfigHWOptFlow"),
        Some(2),
        &[setup(301, "Optical Flow", OPTIONAL, ANY)],
        Missing,
    ),
    panel(
        "ConfigHWOSD",
        cv!("ConfigHWOSD"),
        Some(1),
        &[setup(305, "OSD", OPTIONAL, ANY)],
        Missing,
    ),
    panel(
        "ConfigMount",
        cv!("ConfigMount"),
        Some(5),
        &[setup(309, "Camera Gimbal", OPTIONAL, ANY)],
        Missing,
    ),
    panel(
        "ConfigAntennaTracker",
        cv!("ConfigAntennaTracker"),
        Some(3),
        &[
            setup(313, "Antenna tracker", OPTIONAL, "tracker"),
            config(193, "Extended Tuning", "tracker"),
        ],
        Missing,
    ),
    panel(
        "ConfigMotorTest",
        cv!("ConfigMotorTest"),
        Some(3),
        &[setup(317, "Motor Test", OPTIONAL, ANY)],
        // C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:43-396 - the motor count and the
        // lettered, labelled buttons from the frame and APMotorLayout.json, the throttle and
        // duration boxes, Test all motors, Stop all motors, Test all in Sequence, the
        // MOT_SPIN_ARM and MOT_SPIN_MIN setters and the motor-order link.
        Ours::Done(at("crates/mp-gui/src/config/motor_test.rs", "fn page")),
    ),
    panel(
        "ConfigHWBT",
        cv!("ConfigHWBT"),
        Some(1),
        &[setup(321, "Bluetooth Setup", OPTIONAL, ALWAYS)],
        Missing,
    ),
    panel(
        "ConfigHWParachute",
        cv!("ConfigHWParachute"),
        Some(1),
        &[setup(325, "Parachute", OPTIONAL, ANY)],
        Missing,
    ),
    panel(
        "ConfigHWESP8266",
        cv!("ConfigHWesp8266"),
        Some(3),
        &[setup(329, "ESP8266 Setup", OPTIONAL, ANY)],
        Missing,
    ),
    panel(
        "ConfigFFT",
        cv!("ConfigFFT"),
        None,
        &[setup(337, "FFT Setup", OPTIONAL, ANY)],
        Missing,
    ),
    panel(
        "ConfigAdvanced",
        cv!("ConfigAdvanced"),
        Some(13),
        &[setup(342, "Advanced", TOP, "always, Advanced view")],
        Missing,
    ),
    panel(
        "ConfigTerminal",
        cv!("ConfigTerminal"),
        Some(12),
        &[setup(346, "Terminal", ADVANCED, "always, Advanced view")],
        Missing,
    ),
    panel(
        "ConfigREPL",
        cv!("ConfigREPL"),
        Some(4),
        &[setup(
            351,
            "Script REPL",
            ADVANCED,
            "connected, Advanced view",
        )],
        Missing,
    ),
    // ---- CONFIG: SoftwareConfig.SoftwareConfig_Load, GCSViews/SoftwareConfig.cs:142 ----
    panel(
        "ConfigAC_Fence",
        cv!("ConfigAC_Fence"),
        Some(0),
        &[config(156, "GeoFence", "copter")],
        Missing,
    ),
    panel(
        "ConfigSimplePids",
        cv!("ConfigSimplePids"),
        Some(1),
        &[config(164, "Basic Tuning", "copter")],
        Missing,
    ),
    panel(
        "ConfigArducopter",
        cv!("ConfigArducopter"),
        Some(128),
        &[
            config(169, "Extended Tuning", "copter"),
            // C#: GCSViews/ConfigurationView/ConfigArducopter.cs:34 enables it for a plane only
            // with Q_ENABLE set.
            config(182, "QP Extended Tuning", "plane (enabled for a quadplane)"),
        ],
        Missing,
    ),
    panel(
        "ConfigArduplane",
        cv!("ConfigArduplane"),
        Some(47),
        &[config(177, "Basic Tuning", "plane")],
        Missing,
    ),
    panel(
        "ConfigArdurover",
        cv!("ConfigArdurover"),
        Some(3),
        &[config(188, "Basic Tuning", "rover")],
        Missing,
    ),
    panel(
        "ConfigFriendlyParams",
        cv!("ConfigFriendlyParams"),
        Some(1),
        // C#: ExtLibs/Utilities/DisplayView.cs:340, :427 - off in the Basic and Advanced views.
        &[config(
            198,
            "Standard Params",
            "any, Custom view with Standard Params",
        )],
        Missing,
    ),
    panel(
        "ConfigFriendlyParamsAdv",
        cv!("ConfigFriendlyParamsAdv"),
        None,
        // C#: ExtLibs/Utilities/DisplayView.cs:341, :428; the `true` makes it an advanced page,
        // which ExtLibs/Controls/BackstageView/BackstageView.cs:285 skips outside the Advanced
        // view (MainV2.cs:601).
        &[config(
            203,
            "Advanced Params",
            "any, Custom view with Advanced Params and Advanced mode",
        )],
        Missing,
    ),
    panel(
        "ConfigOSD",
        cv!("ConfigOSD"),
        Some(0),
        // C#: GCSViews/ConfigurationView/ConfigOSD.cs:214
        &[config(
            208,
            "Onboard OSD",
            "any with OSD parameters, not on Mono",
        )],
        Missing,
    ),
    panel(
        "ConfigUserDefined",
        cv!("ConfigUserDefined"),
        Some(0),
        &[config(221, "User Params", ANY)],
        Missing,
    ),
    panel(
        "ConfigRawParams",
        cv!("ConfigRawParams"),
        Some(22),
        &[config(229, "Full Parameter List", "any, or disconnected")],
        Partial(
            at(PARAMS_RS, "fn list_panel"),
            "the parameter screen has Refresh Params, Search, the group tree, editing a value, \
             Save to file, Compare Params, and Load from file as compare then apply; missing \
             Reset to Default, Load Presaved and its file list, Commit Params, the Modified and \
             None Default filters, Refresh Table and the tree's collapse",
        ),
    ),
    panel(
        "ConfigAteryxSensors",
        cv!("ConfigAteryxSensors"),
        Some(3),
        &[config(236, "Ateryx Zero Sensors", "Ateryx")],
        Missing,
    ),
    panel(
        "ConfigAteryx",
        cv!("ConfigAteryx"),
        Some(8),
        &[config(237, "Ateryx Pids", "Ateryx")],
        Missing,
    ),
    panel(
        "ConfigPlanner",
        cv!("ConfigPlanner"),
        Some(64),
        &[
            config(250, "Planner", CONNECTED),
            config(257, "Planner", DISCONNECTED),
        ],
        // C#: GCSViews/ConfigurationView/ConfigPlanner.cs:26-1192 - every control at its .resx
        // place, bound to the Settings key its handler writes.
        Partial(
            at("crates/mp-gui/src/config/planner.rs", "fn planner_page"),
            "every control at its place, each bound to the Settings key its handler writes; the \
             units (ChangeUnits), the telemetry rates and their stream requests, the speech boxes \
             and their InputBox templates, Load Waypoints on connect, the map access mode, \
             Joystick Setup, Browse and Open Map Cache act at once; dimmed for want of what they \
             drive: video, the HUD overlay, GDI+, language, theme, Layout, OSD colour, Vario, \
             password, the ADSB server, analytics, beta updates, MAVLink debug and the testing \
             screen; the flight screen does not yet read the units, the track length, the map's \
             rotation or the icon settings, nor the link the GCS id or the rates on connecting",
        ),
    ),
    // ---- Neither list adds these ----
    panel(
        "ConfigHWCAN",
        cv!("ConfigHWCAN"),
        Some(6),
        &[],
        Ours::Dropped(
            "Mission Planner never shows it: its entry is commented out at InitialSetup.cs:275, \
             beside ConfigDroneCAN's",
        ),
    ),
    panel(
        "ConfigPlannerAdv",
        cv!("ConfigPlannerAdv"),
        Some(0),
        &[],
        Ours::Dropped("Mission Planner never shows it: nothing lists or opens it"),
    ),
    panel(
        "ConfigSecure",
        cv!("ConfigSecure"),
        Some(6),
        &[],
        Ours::Dropped(
            "Mission Planner never shows it: nothing lists or opens it; the Secure page is \
             ConfigSecureAP",
        ),
    ),
    panel(
        "ConfigTradHeli",
        cv!("ConfigTradHeli"),
        Some(22),
        &[],
        Ours::Dropped(
            "Mission Planner never shows it: its entry is commented out at InitialSetup.cs:186; \
             Heli Setup is ConfigTradHeli4",
        ),
    ),
];

/// The pages the lists add that are not in `ConfigurationView/`. Listed so the lists are whole;
/// not counted as panels.
pub const OTHER_PAGES: &[Panel] = &[
    panel(
        "Sikradio",
        "Radio/Sikradio.cs",
        Some(17),
        &[setup(259, "Sik Radio", OPTIONAL, ALWAYS)],
        Missing,
    ),
    panel(
        "JoystickSetup",
        "Joystick/JoystickSetup.cs",
        Some(11),
        &[setup(280, "Joystick", OPTIONAL, ALWAYS)],
        Partial(
            at(JOYSTICK_RS, "fn panel_for"),
            "has the device list and Enable; missing the per-channel axis grid, the button \
             functions, Elevons, Save, Manual Control, Import and Export",
        ),
    ),
    panel(
        "TrackerUI",
        "Antenna/TrackerUI.cs",
        Some(0),
        &[setup(333, "Antenna Tracker", OPTIONAL, ALWAYS)],
        Missing,
    ),
    panel(
        "MavFTPUI",
        "Controls/MavFTPUI.cs",
        Some(16),
        &[config(215, "MAVFtp", "any reporting MAVLink FTP")],
        Missing,
    ),
];

/// How many rows of `table` are in each state: (done, partial, missing, plumbing, dropped).
#[must_use]
pub fn counts_of(table: &[Panel]) -> (usize, usize, usize, usize, usize) {
    let mut counts = (0, 0, 0, 0, 0);
    for panel in table {
        match panel.ours {
            Ours::Done(_) => counts.0 += 1,
            Ours::Partial(..) => counts.1 += 1,
            Ours::Missing => counts.2 += 1,
            Ours::Plumbing(_) => counts.3 += 1,
            Ours::Dropped(_) => counts.4 += 1,
        }
    }
    counts
}

/// How many panels are in each state: (done, partial, missing, plumbing, dropped).
#[must_use]
pub fn counts() -> (usize, usize, usize, usize, usize) {
    counts_of(PANELS)
}

/// The event wirings of every panel's Designer, added up.
#[must_use]
pub fn wirings_of(table: &[Panel]) -> usize {
    table.iter().filter_map(|panel| panel.wirings).sum()
}

/// What the application publishes about this table, as `(key, value)`.
#[must_use]
pub fn facts() -> [(&'static str, usize); 7] {
    let (done, partial, missing, plumbing, dropped) = counts();
    [
        ("coverage.configuration.total", PANELS.len()),
        ("coverage.configuration.done", done),
        ("coverage.configuration.partial", partial),
        ("coverage.configuration.missing", missing),
        ("coverage.configuration.plumbing", plumbing),
        ("coverage.configuration.dropped", dropped),
        ("coverage.configuration.wirings", wirings_of(PANELS)),
    ]
}

/// The screen whose list first adds `panel`, or `None` for one neither adds.
#[must_use]
pub fn group(panel: &Panel) -> Option<Screen> {
    panel.listed.first().map(|listed| listed.screen)
}

/// The call `screen`'s list makes at `line` of its source, with the page it adds: where the setup
/// and configuration screens (`setup.rs`) read each entry's title and heading, so the lists they
/// draw and this ledger cannot disagree.
#[must_use]
pub fn listing(screen: Screen, line: u32) -> Option<(&'static Listed, &'static Panel)> {
    PANELS.iter().chain(OTHER_PAGES).find_map(|panel| {
        panel
            .listed
            .iter()
            .find(|listed| listed.screen == screen && listed.line == line)
            .map(|listed| (listed, panel))
    })
}

/// Every listing on `screen`'s list, with its page, in the list's order.
#[cfg(test)]
fn listings(screen: Screen) -> Vec<(&'static Listed, &'static Panel)> {
    let mut out: Vec<(&'static Listed, &'static Panel)> = PANELS
        .iter()
        .chain(OTHER_PAGES)
        .flat_map(|panel| panel.listed.iter().map(move |listed| (listed, panel)))
        .filter(|(listed, _)| listed.screen == screen)
        .collect();
    out.sort_by_key(|(listed, _)| listed.line);
    out
}

/// The title of the heading `class` names, as the list shows it.
#[cfg(test)]
fn heading(class: &str) -> &'static str {
    PANELS
        .iter()
        .find(|panel| panel.class == class)
        .and_then(|panel| panel.listed.first())
        .map_or("?", |listed| listed.title)
}

/// A cell for what stands in for a page.
#[cfg(test)]
fn ours_cell(ours: Ours) -> String {
    match ours {
        Ours::Done(at) => format!("done: `{}` `{}`", at.file, at.item),
        Ours::Partial(at, what) => format!("partial: `{}` `{}` - {what}", at.file, at.item),
        Ours::Missing => "**missing**".to_owned(),
        Ours::Plumbing(what) => format!("plumbing: {what}"),
        Ours::Dropped(why) => format!("dropped: {why}"),
    }
}

/// A cell for a page's wiring count.
#[cfg(test)]
fn wirings_cell(wirings: Option<usize>) -> String {
    wirings.map_or_else(|| "no Designer".to_owned(), |count| count.to_string())
}

/// A cell naming a page: the class, and the file where its name differs or it lives elsewhere.
#[cfg(test)]
fn page_cell(panel: &Panel) -> String {
    let stem = panel
        .file
        .rsplit('/')
        .next()
        .and_then(|name| name.strip_suffix(".cs"))
        .unwrap_or(panel.file);
    if !panel.file.starts_with("GCSViews/ConfigurationView/") {
        format!("`{}` (`{}`, not a panel)", panel.class, panel.file)
    } else if stem == panel.class {
        format!("`{}`", panel.class)
    } else {
        format!("`{}` (`{stem}.cs`)", panel.class)
    }
}

/// The report, as Markdown: the counts, the groups, the largest missing panels, then each list.
#[cfg(test)]
#[must_use]
pub fn report() -> String {
    let (done, partial, missing, plumbing, dropped) = counts();
    let mut out = String::new();
    out.push_str("# Configuration panel coverage\n\n");
    out.push_str(
        "Generated from `crates/mp-gui/src/config_coverage.rs` by `cargo test -p mp-gui \
         config_coverage::tests::update_report -- --ignored`; a test fails when this file is \
         stale. One \
         row per panel in `GCSViews/ConfigurationView/` - every `Config*.cs` - in the order \
         `GCSViews/InitialSetup.cs` (the SETUP button) and then `GCSViews/SoftwareConfig.cs` (the \
         CONFIG button) first add it to their left-hand lists; panels neither list adds come \
         last. Wirings are the events the panel's Designer wires, a measure of its size that \
         undercounts a panel which builds its controls in code.\n\n",
    );
    out.push_str(&format!(
        "| panels | done | partial | missing | plumbing | dropped | wirings |\n\
         |---:|---:|---:|---:|---:|---:|---:|\n\
         | {} | {done} | {partial} | {missing} | {plumbing} | {dropped} | {} |\n\n",
        PANELS.len(),
        wirings_of(PANELS)
    ));

    out.push_str(
        "| group | panels | done | partial | missing | plumbing | dropped | wirings | wirings \
         in missing panels |\n|---|---:|---:|---:|---:|---:|---:|---:|---:|\n",
    );
    let groups = [
        (
            Some(Screen::Setup),
            "SETUP, `InitialSetup.HardwareConfig_Load`",
        ),
        (
            Some(Screen::Config),
            "CONFIG, `SoftwareConfig.SoftwareConfig_Load`",
        ),
        (None, "neither list"),
    ];
    for (screen, name) in groups {
        let members: Vec<Panel> = PANELS
            .iter()
            .filter(|panel| group(panel) == screen)
            .copied()
            .collect();
        let (done, partial, missing, plumbing, dropped) = counts_of(&members);
        let missing_wirings: usize = members
            .iter()
            .filter(|panel| panel.ours == Ours::Missing)
            .filter_map(|panel| panel.wirings)
            .sum();
        out.push_str(&format!(
            "| {name} | {} | {done} | {partial} | {missing} | {plumbing} | {dropped} | {} | \
             {missing_wirings} |\n",
            members.len(),
            wirings_of(&members)
        ));
    }
    out.push('\n');

    let (done, partial, missing, plumbing, dropped) = counts_of(OTHER_PAGES);
    let others: Vec<String> = OTHER_PAGES
        .iter()
        .map(|page| format!("`{}`", page.class))
        .collect();
    out.push_str(&format!(
        "The lists also add {} pages that are not in `ConfigurationView/` ({}): {done} done, \
         {partial} partial, {missing} missing, {plumbing} plumbing, {dropped} dropped. They are \
         in the lists below and not in the counts above.\n\n",
        OTHER_PAGES.len(),
        others.join(", ")
    ));

    out.push_str("The largest missing panels, by wirings:\n\n| panel | title | wirings |\n");
    out.push_str("|---|---|---:|\n");
    let mut largest: Vec<&Panel> = PANELS
        .iter()
        .filter(|panel| panel.ours == Ours::Missing)
        .collect();
    largest.sort_by(|a, b| {
        b.wirings
            .unwrap_or(0)
            .cmp(&a.wirings.unwrap_or(0))
            .then(a.class.cmp(b.class))
    });
    for panel in largest.iter().take(12) {
        let title = panel.listed.first().map_or("", |listed| listed.title);
        out.push_str(&format!(
            "| {} | {title} | {} |\n",
            page_cell(panel),
            wirings_cell(panel.wirings)
        ));
    }
    out.push('\n');

    out.push_str(
        "Vehicles: **any** is a connected vehicle whose parameter list is whole \
         (`isConnected && gotAllParams`); **always** is connected or not; **connected** and \
         **disconnected** are the link alone; a named vehicle, parameter or view is what the \
         call, or the `if` around it, checks. **Advanced view** is `DisplayView.isAdvancedMode`. \
         A page with a `DisplayView` switch also needs it on, which it is by default unless the \
         vehicles say otherwise. The list shows a heading as `>> title` and indents what is \
         under it (`ExtLibs/Controls/BackstageView/BackstageView.cs:227`, `:232`).\n\n",
    );

    for screen in Screen::ALL {
        out.push_str(&format!(
            "## {} - `{}` `{}`\n\n",
            screen.menu(),
            screen.source(),
            screen.handler()
        ));
        out.push_str(
            "| line | page | title | under | vehicles | wirings | ours |\n\
             |---:|---|---|---|---|---:|---|\n",
        );
        for (listed, panel) in listings(screen) {
            let under = listed.parent.map_or("", heading);
            let first = panel.listed.first().copied();
            let ours = if first == Some(*listed) {
                ours_cell(panel.ours)
            } else {
                let first = first.unwrap_or(*listed);
                format!("as at `{}:{}`", first.screen.source(), first.line)
            };
            out.push_str(&format!(
                "| {} | {} | {} | {under} | {} | {} | {ours} |\n",
                listed.line,
                page_cell(panel),
                listed.title,
                listed.vehicles,
                wirings_cell(panel.wirings),
            ));
        }
        out.push('\n');
    }

    out.push_str("## Neither list\n\n| page | wirings | ours |\n|---|---:|---|\n");
    for panel in PANELS.iter().filter(|panel| panel.listed.is_empty()) {
        out.push_str(&format!(
            "| {} | {} | {} |\n",
            page_cell(panel),
            wirings_cell(panel.wirings),
            ours_cell(panel.ours)
        ));
    }
    out
}

/// Reading the C# tree, for this ledger's tests and for the tests of the lists drawn from it
/// (`setup.rs`).
#[cfg(test)]
pub(crate) mod source {
    use std::collections::BTreeMap;
    use std::path::{Path, PathBuf};

    /// The workspace root.
    pub(crate) fn workspace() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
    }

    /// The C# tree.
    pub(crate) fn csharp_root() -> PathBuf {
        workspace().join("referneces/missionplanner")
    }

    /// A C# file, relative to the tree, or `None` when the tree is not checked out.
    pub(crate) fn csharp(path: &str) -> Option<String> {
        std::fs::read_to_string(csharp_root().join(path)).ok()
    }

    /// Every `<data name="K"><value>V</value>` in a `.resx`.
    pub(crate) fn resx(text: &str) -> BTreeMap<String, String> {
        let mut values = BTreeMap::new();
        for data in text.split("<data name=\"").skip(1) {
            let Some((name, rest)) = data.split_once('"') else {
                continue;
            };
            let Some((value, _)) = rest
                .split_once("<value>")
                .and_then(|(_, value)| value.split_once("</value>"))
            else {
                continue;
            };
            let value = value
                .replace("&lt;", "<")
                .replace("&gt;", ">")
                .replace("&amp;", "&");
            values.insert(name.to_owned(), value);
        }
        values
    }

    /// One `AddBackstageViewPage(typeof(...), ...)` call.
    #[derive(Debug)]
    pub(crate) struct Call {
        /// 1-based line of the call.
        pub(crate) line: u32,
        /// The page's class, without its namespace.
        pub(crate) class: String,
        /// The arguments, whitespace collapsed; the first is the `typeof`.
        pub(crate) args: Vec<String>,
        /// `x` in `var x = AddBackstageViewPage(...)`.
        pub(crate) assigned: Option<String>,
        /// Where the call starts in the source, in bytes.
        pub(crate) offset: usize,
        /// What precedes the call on its line, trimmed: `start =` where `SoftwareConfig` keeps
        /// the page to open first.
        pub(crate) prefix: String,
    }

    /// The arguments of the call whose `(` opens `text`, split at its top-level commas.
    pub(crate) fn arguments(text: &str) -> Vec<String> {
        let mut args = Vec::new();
        let mut current = String::new();
        let mut depth = 0_i32;
        let mut quoted = false;
        let mut escaped = false;
        for c in text.chars() {
            if quoted {
                current.push(c);
                if escaped {
                    escaped = false;
                } else if c == '\\' {
                    escaped = true;
                } else if c == '"' {
                    quoted = false;
                }
                continue;
            }
            match c {
                '"' => {
                    quoted = true;
                    current.push(c);
                }
                '(' => {
                    depth += 1;
                    if depth > 1 {
                        current.push(c);
                    }
                }
                ')' => {
                    depth -= 1;
                    if depth == 0 {
                        args.push(current);
                        break;
                    }
                    current.push(c);
                }
                ',' if depth == 1 => args.push(std::mem::take(&mut current)),
                _ => current.push(c),
            }
        }
        args.into_iter()
            .map(|arg| arg.split_whitespace().collect::<Vec<_>>().join(" "))
            .collect()
    }

    /// Every call in `handler`'s body that is not commented out, in the source's order.
    pub(crate) fn calls(source: &str, handler: &str) -> Vec<Call> {
        const OPEN: &str = "AddBackstageViewPage(typeof(";
        let start = source
            .find(&format!("private void {handler}("))
            .unwrap_or_else(|| panic!("{handler} is not in the source"));
        let mut out = Vec::new();
        for (offset, _) in source
            .match_indices(OPEN)
            .filter(|(position, _)| *position > start)
        {
            let line_start = source[..offset].rfind('\n').map_or(0, |at| at + 1);
            let before = &source[line_start..offset];
            if before.contains("//") {
                continue;
            }
            let line = u32::try_from(source[..offset].matches('\n').count() + 1).unwrap_or(0);
            let open = offset + OPEN.len() - "(typeof(".len();
            let args = arguments(&source[open..]);
            let class = args
                .first()
                .and_then(|arg| arg.strip_prefix("typeof("))
                .and_then(|arg| arg.strip_suffix(')'))
                .and_then(|arg| arg.rsplit('.').next())
                .unwrap_or_default()
                .to_owned();
            let assigned = before
                .trim()
                .strip_prefix("var ")
                .and_then(|rest| rest.strip_suffix('='))
                .map(|name| name.trim().to_owned());
            out.push(Call {
                line,
                class,
                args,
                assigned,
                offset,
                prefix: before.trim().to_owned(),
            });
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use std::path::Path;

    use super::source::{calls, csharp, csharp_root, resx, workspace};
    use super::*;

    /// Where the committed report lives, relative to this crate.
    const REPORT: &str = "../../docs/coverage/configuration.md";

    /// The Designer beside a `.cs` file: `X.Designer.cs`, or `X.designer.cs` as `ConfigMount`'s
    /// is spelled.
    fn designer(file: &str) -> Option<String> {
        let stem = file.strip_suffix(".cs")?;
        csharp(&format!("{stem}.Designer.cs")).or_else(|| csharp(&format!("{stem}.designer.cs")))
    }

    /// The event wirings in a Designer: `this.X.Y += new Z(this.H);`, as `coverage.rs` reads them.
    fn wirings(designer: &str) -> usize {
        designer
            .lines()
            .filter(|line| {
                let line = line.trim();
                line.split_once(" += new ").is_some_and(|(left, right)| {
                    left.starts_with("this.")
                        && right
                            .rsplit_once("(this.")
                            .is_some_and(|(_, handler)| handler.ends_with(");"))
                })
            })
            .count()
    }

    /// The text a title argument evaluates to: `rm.GetString("K")` from `InitialSetup.resx`,
    /// `Strings.K` from `Strings.resx`, a literal, a `+` of those, or a local the handler sets.
    fn resolve(
        expression: &str,
        setup: &BTreeMap<String, String>,
        strings: &BTreeMap<String, String>,
        source: &str,
    ) -> Option<String> {
        let mut text = String::new();
        for term in expression.split(" + ") {
            let term = term.trim();
            let value = if let Some(key) = term
                .strip_prefix("rm.GetString(\"")
                .and_then(|rest| rest.strip_suffix("\")"))
            {
                setup.get(key).cloned()
            } else if let Some(key) = term.strip_prefix("Strings.") {
                strings.get(key).cloned()
            } else if let Some(literal) = term
                .strip_prefix('"')
                .and_then(|rest| rest.strip_suffix('"'))
            {
                Some(literal.to_owned())
            } else {
                // A local: `var x = <expr>;`, and if that is null, `x = "<literal>";`, which is
                // how the RTK/GPS Inject title is made (C#: GCSViews/InitialSetup.cs:246).
                let assigned = source
                    .split(&format!("var {term} = "))
                    .nth(1)
                    .and_then(|rest| rest.split_once(';'))
                    .and_then(|(expression, _)| resolve(expression, setup, strings, source));
                assigned.or_else(|| {
                    source
                        .split(&format!("{term} = \""))
                        .nth(1)
                        .and_then(|rest| rest.split_once("\";"))
                        .map(|(literal, _)| literal.to_owned())
                })
            };
            text.push_str(&value?);
        }
        Some(text)
    }

    /// Every `Config*.cs` in the directory is a row, and every row is a file there declaring the
    /// class it names. Including the two with no Designer: `ConfigFFT` builds its controls in code
    /// and `ConfigFriendlyParamsAdv` is `ConfigFriendlyParams` in another mode, and both are
    /// listed.
    #[test]
    fn the_panels_are_the_directorys() {
        let directory = csharp_root().join("GCSViews/ConfigurationView");
        let Ok(entries) = std::fs::read_dir(&directory) else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let mut files: Vec<String> = entries
            .filter_map(Result::ok)
            .map(|entry| entry.file_name().to_string_lossy().into_owned())
            .filter(|name| {
                name.starts_with("Config")
                    && name.ends_with(".cs")
                    && !name.to_lowercase().ends_with(".designer.cs")
            })
            .map(|name| format!("GCSViews/ConfigurationView/{name}"))
            .collect();
        files.sort();
        let mut ours: Vec<String> = PANELS.iter().map(|panel| panel.file.to_owned()).collect();
        ours.sort();
        let before = ours.len();
        ours.dedup();
        assert_eq!(ours.len(), before, "a file has two rows");
        assert_eq!(ours, files, "the rows are not the directory's Config*.cs");
        assert_eq!(PANELS.len(), 61, "ConfigurationView holds 61 Config*.cs");

        for panel in PANELS.iter().chain(OTHER_PAGES) {
            let source = csharp(panel.file).unwrap_or_else(|| panic!("{} is missing", panel.file));
            assert!(
                source.contains(&format!("class {} :", panel.class)),
                "{} does not declare {}",
                panel.file,
                panel.class
            );
        }
        for page in OTHER_PAGES {
            assert!(
                !page.file.starts_with("GCSViews/ConfigurationView/"),
                "{} is a panel, not another page",
                page.class
            );
        }
    }

    /// Each wiring count is its Designer's, and `None` is a panel with no Designer.
    #[test]
    fn the_wiring_counts_are_the_designers() {
        if csharp(Screen::Setup.source()).is_none() {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        }
        for panel in PANELS.iter().chain(OTHER_PAGES) {
            let counted = designer(panel.file).map(|designer| wirings(&designer));
            assert_eq!(panel.wirings, counted, "{}'s wirings", panel.class);
        }
        assert_eq!(wirings_of(PANELS), 569);
    }

    /// The listings are the calls each `Load` handler makes, line for line, in order, with the
    /// same page and the same heading.
    #[test]
    fn the_lists_are_the_csharps_in_its_order() {
        for screen in Screen::ALL {
            let Some(source) = csharp(screen.source()) else {
                eprintln!("skipped: the C# tree is not checked out here");
                return;
            };
            let calls = calls(&source, screen.handler());
            let headings: BTreeMap<String, String> = calls
                .iter()
                .filter_map(|call| Some((call.assigned.clone()?, call.class.clone())))
                .collect();
            let theirs: Vec<(u32, String, Option<String>)> = calls
                .iter()
                .map(|call| {
                    // InitialSetup's (type, title, enabled, parent); SoftwareConfig's (type, title,
                    // parent, advanced), whose parent is always `null`.
                    let parent = match screen {
                        Screen::Setup => call.args.get(3),
                        Screen::Config => call.args.get(2).filter(|arg| *arg != "null"),
                    };
                    let parent = parent.map(|name| {
                        headings
                            .get(name)
                            .cloned()
                            .unwrap_or_else(|| panic!("{name} is not a heading"))
                    });
                    (call.line, call.class.clone(), parent)
                })
                .collect();
            let ours: Vec<(u32, String, Option<String>)> = listings(screen)
                .into_iter()
                .map(|(listed, panel)| {
                    (
                        listed.line,
                        panel.class.to_owned(),
                        listed.parent.map(str::to_owned),
                    )
                })
                .collect();
            assert_eq!(ours, theirs, "{}'s list", screen.menu());
        }
    }

    /// Every title is the text the call's argument gives, from the `.resx` it names.
    #[test]
    fn every_title_is_the_one_the_list_shows() {
        let (Some(setup_resx), Some(strings_resx)) = (
            csharp("GCSViews/InitialSetup.resx"),
            csharp("ExtLibs/Strings/Strings.resx"),
        ) else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let setup_resx = resx(&setup_resx);
        let strings_resx = resx(&strings_resx);
        let mut checked = 0;
        for screen in Screen::ALL {
            let Some(source) = csharp(screen.source()) else {
                return;
            };
            for call in calls(&source, screen.handler()) {
                let expression = call.args.get(1).map_or("", String::as_str);
                let shown = resolve(expression, &setup_resx, &strings_resx, &source)
                    .unwrap_or_else(|| {
                        panic!("{} line {}: {expression}", screen.menu(), call.line)
                    });
                let listed = listings(screen)
                    .into_iter()
                    .find(|(listed, _)| listed.line == call.line)
                    .map(|(listed, _)| listed.title);
                assert_eq!(
                    listed,
                    Some(shown.as_str()),
                    "{} line {}",
                    screen.menu(),
                    call.line
                );
                checked += 1;
            }
        }
        assert_eq!(checked, 67, "the two lists make 67 calls");
    }

    /// The menu buttons are `MainV2`'s, and open these two screens.
    #[test]
    fn the_screens_are_mainv2s() {
        let (Some(resx_text), Some(main)) = (csharp("MainV2.resx"), csharp("MainV2.cs")) else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let texts = resx(&resx_text);
        assert_eq!(
            texts.get("MenuInitConfig.Text").map(String::as_str),
            Some(Screen::Setup.menu())
        );
        assert_eq!(
            texts.get("MenuConfigTune.Text").map(String::as_str),
            Some(Screen::Config.menu())
        );
        assert!(
            main.contains("new MainSwitcher.Screen(\"HWConfig\", typeof(GCSViews.InitialSetup)")
        );
        assert!(
            main.contains("new MainSwitcher.Screen(\"SWConfig\", typeof(GCSViews.SoftwareConfig)")
        );
    }

    /// The rows are in the order the lists first add them, SETUP's before CONFIG's, and the
    /// panels neither adds come last in file order. Each row's own listings are in that order too.
    #[test]
    fn the_rows_are_in_the_order_the_lists_add_them() {
        let mut expected: Vec<&str> = Vec::new();
        for screen in Screen::ALL {
            for (_, panel) in listings(screen) {
                let is_panel = PANELS.iter().any(|row| row.class == panel.class);
                if is_panel && !expected.contains(&panel.class) {
                    expected.push(panel.class);
                }
            }
        }
        let mut unlisted: Vec<&Panel> = PANELS
            .iter()
            .filter(|panel| panel.listed.is_empty())
            .collect();
        unlisted.sort_by_key(|panel| panel.file);
        expected.extend(unlisted.iter().map(|panel| panel.class));
        let ours: Vec<&str> = PANELS.iter().map(|panel| panel.class).collect();
        assert_eq!(ours, expected);

        for panel in PANELS.iter().chain(OTHER_PAGES) {
            let keys: Vec<(u8, u32)> = panel
                .listed
                .iter()
                .map(|listed| (u8::from(listed.screen == Screen::Config), listed.line))
                .collect();
            let mut sorted = keys.clone();
            sorted.sort_unstable();
            assert_eq!(keys, sorted, "{}'s listings are out of order", panel.class);
        }
    }

    /// Every Rust path a row claims exists and holds the function or control id it names.
    #[test]
    fn every_claimed_rust_path_exists() {
        let mut checked = 0;
        for panel in PANELS.iter().chain(OTHER_PAGES) {
            let (Ours::Done(at) | Ours::Partial(at, _)) = panel.ours else {
                continue;
            };
            let source = std::fs::read_to_string(workspace().join(at.file)).unwrap_or_else(|_| {
                panic!("{} claims {}, which does not exist", panel.class, at.file)
            });
            let found = at.item.strip_prefix("fn ").map_or_else(
                || source.contains(&format!("\"{}\"", at.item)),
                |name| source.contains(&format!("fn {name}(")),
            );
            assert!(
                found,
                "{} claims `{}` in {}, which is not there",
                panel.class, at.item, at.file
            );
            checked += 1;
        }
        assert_eq!(checked, 18);
    }

    /// The committed report matches the table.
    #[test]
    fn the_committed_report_is_current() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(REPORT);
        let committed = std::fs::read_to_string(&path).unwrap_or_default();
        assert!(
            committed == report(),
            "docs/coverage/configuration.md is stale; run `cargo test -p mp-gui config_coverage::tests::update_report -- --ignored`"
        );
    }

    /// Rewrites the report. Run on purpose, not on every test.
    #[test]
    #[ignore = "writes docs/coverage/configuration.md"]
    fn update_report() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(REPORT);
        std::fs::write(&path, report()).expect("write the report");
    }

    /// The counts are the recorded ones, so a change in either direction is a deliberate edit.
    #[test]
    fn the_counts_are_the_ones_recorded() {
        let (done, partial, missing, plumbing, dropped) = counts();
        assert_eq!(done + partial + missing + plumbing + dropped, PANELS.len());
        eprintln!(
            "ConfigurationView: {done} done, {partial} partial, {missing} missing, {plumbing} plumbing, {dropped} dropped"
        );
        assert_eq!(
            (done, partial, missing, plumbing, dropped),
            (6, 11, 38, 2, 4)
        );
        let by_group: Vec<usize> = [Some(Screen::Setup), Some(Screen::Config), None]
            .iter()
            .map(|screen| {
                PANELS
                    .iter()
                    .filter(|panel| group(panel) == *screen)
                    .count()
            })
            .collect();
        assert_eq!(by_group, [44, 13, 4]);
    }

    /// The application publishes the counts, and the numbers it publishes are the table's.
    #[test]
    fn the_counts_are_published_as_facts() {
        let facts: BTreeMap<&str, usize> = facts().into_iter().collect();
        let (done, partial, missing, plumbing, dropped) = counts();
        assert_eq!(
            facts.get("coverage.configuration.total"),
            Some(&PANELS.len())
        );
        assert_eq!(facts.get("coverage.configuration.done"), Some(&done));
        assert_eq!(facts.get("coverage.configuration.partial"), Some(&partial));
        assert_eq!(facts.get("coverage.configuration.missing"), Some(&missing));
        assert_eq!(
            facts.get("coverage.configuration.plumbing"),
            Some(&plumbing)
        );
        assert_eq!(facts.get("coverage.configuration.dropped"), Some(&dropped));
        assert_eq!(
            facts.get("coverage.configuration.wirings"),
            Some(&wirings_of(PANELS))
        );
        // And the render path records them: the only caller of `facts()`.
        assert!(
            include_str!("main.rs").contains("config_coverage::facts()"),
            "main.rs does not record the configuration coverage facts"
        );
    }
}
