//! Flight Modes: the mode each position of the mode switch selects.
//!
//! Mission Planner's `ConfigFlightModes`, a page of Initial Setup under Mandatory Hardware
//! (`GCSViews/InitialSetup.cs:226-229`). Six combos, one per switch position, filled from the
//! vehicle's mode list and set from `FLTMODE1`-`6` (`MODE1`-`6` on a rover, `COM_FLTMODE1`-`6` on
//! PX4); Simple and Super Simple check boxes beside each on a copter, packed into the `SIMPLE` and
//! `SUPER_SIMPLE` bitmasks; the PWM band each position answers to; and, above them, the mode the
//! vehicle is in and the pulse width on the mode channel, with the combo that pulse selects lit.
//!
//! The layout is `ConfigFlightModes.resx`'s: "Current Mode:" and "Current PWM:" above a five-column
//! table - `Flight Mode N`, the combo, Simple Mode, Super Simple Mode, the PWM band, at 110, 110,
//! 110, 140 and 88 pixels - with Save Modes under the combos and the Simple help link under the
//! Super Simple column. WinForms has combo boxes and check boxes and gpui has neither, so a combo
//! is a box that opens its list beneath its row and a check box is a square that fills.
//!
//! What the C# does on the UI thread - six to eight `setParam` calls in a row, each blocking until
//! the vehicle echoes the value or three retries go unanswered - is a queue here, advanced once a
//! frame, one write at a time and in the same order, stopping where the C# would throw.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;

use gpui::{AnyElement, Context, FocusHandle, KeyDownEvent, MouseButton, SharedString, div, prelude::*, px, rgb};
use mp_link::requests::RequestOutcome;
use mp_vehicle::VehicleFamily;
use mp_vehicle::rc::RcChannels;

use crate::MissionPlanner;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{action, panel, theme};

/// How many switch positions there are, and so how many combos.
pub const POSITIONS: usize = 6;

/// `MAV_AUTOPILOT_GENERIC`.
const AUTOPILOT_GENERIC: u8 = 0;
/// `MAV_AUTOPILOT_ARDUPILOTMEGA`.
const AUTOPILOT_ARDUPILOTMEGA: u8 = 3;
/// `MAV_AUTOPILOT_UDB`.
const AUTOPILOT_UDB: u8 = 10;
/// `MAV_AUTOPILOT_PX4`.
const AUTOPILOT_PX4: u8 = 12;
/// `MAV_TYPE_FIXED_WING`.
const TYPE_FIXED_WING: u8 = 1;

/// The parameters each firmware's combos are read from and written to.
const FLTMODE: [&str; POSITIONS] = [
    "FLTMODE1", "FLTMODE2", "FLTMODE3", "FLTMODE4", "FLTMODE5", "FLTMODE6",
];
/// A rover's.
const MODE: [&str; POSITIONS] = ["MODE1", "MODE2", "MODE3", "MODE4", "MODE5", "MODE6"];
/// PX4's.
const COM_FLTMODE: [&str; POSITIONS] = [
    "COM_FLTMODE1",
    "COM_FLTMODE2",
    "COM_FLTMODE3",
    "COM_FLTMODE4",
    "COM_FLTMODE5",
    "COM_FLTMODE6",
];

/// Every parameter this page reads or writes, published as `params.value.<name>` facts.
const WATCHED: [&str; 22] = [
    "FLTMODE1",
    "FLTMODE2",
    "FLTMODE3",
    "FLTMODE4",
    "FLTMODE5",
    "FLTMODE6",
    "MODE1",
    "MODE2",
    "MODE3",
    "MODE4",
    "MODE5",
    "MODE6",
    "COM_FLTMODE1",
    "COM_FLTMODE2",
    "COM_FLTMODE3",
    "COM_FLTMODE4",
    "COM_FLTMODE5",
    "COM_FLTMODE6",
    "FLTMODE_CH",
    "MODE_CH",
    "SIMPLE",
    "SUPER_SIMPLE",
];

/// The PWM band each switch position answers to, as the `.resx` labels it (`label12`, `label7`
/// to `label11`), top row first.
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.resx (label12, label7-label11 .Text)`
pub const BANDS: [&str; POSITIONS] = [
    "PWM 0 - 1230",
    "PWM 1231 - 1360",
    "PWM 1361 - 1490",
    "PWM 1491 - 1620",
    "PWM 1621 - 1749",
    "PWM 1750 +",
];

/// The table's column widths, `tableLayoutPanel1.LayoutSettings`' `Columns Styles`.
const COLUMNS: [f32; 5] = [110.0, 110.0, 110.0, 140.0, 88.0];
/// The table's row height, from the same.
const ROW: f32 = 25.0;

/// Where the Simple and Super Simple link goes.
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:427`
const SIMPLE_HELP: &str = "https://ardupilot.org/copter/docs/simpleandsuper-simple-modes.html";

/// What the C# shows when a write fails: `Strings.ErrorSettingParameter`, under `Strings.ERROR`.
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:411; ExtLibs/Strings/Strings.resx:168`
const ERROR_SETTING_PARAMETER: &str = "Error setting parameter";

/// The firmwares the C#'s `Firmwares` enum distinguishes, as far as this page tells them apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Firmware {
    /// `ArduPlane`.
    ArduPlane,
    /// `Ateryx`, a fixed wing on a generic autopilot.
    Ateryx,
    /// `ArduCopter2`, which is also what an unrecognised vehicle is left as.
    ArduCopter2,
    /// `ArduRover`.
    ArduRover,
    /// `ArduSub`.
    ArduSub,
    /// `ArduTracker`.
    ArduTracker,
    /// `PX4`.
    Px4,
    /// `Other` and `Gimbal`: nothing this page has a branch for.
    #[default]
    Other,
}

impl Firmware {
    /// The C#'s name for it, for the facts.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::ArduPlane => "ArduPlane",
            Self::Ateryx => "Ateryx",
            Self::ArduCopter2 => "ArduCopter2",
            Self::ArduRover => "ArduRover",
            Self::ArduSub => "ArduSub",
            Self::ArduTracker => "ArduTracker",
            Self::Px4 => "PX4",
            Self::Other => "Other",
        }
    }
}

/// The firmware a vehicle is running, as `setAPType` decides it: from the version banner when it
/// names exactly one vehicle, else from `MAV_TYPE`, for an ArduPilot; by autopilot otherwise.
///
/// Where the C# leaves `cs.firmware` untouched it keeps its initial `ArduCopter2`, so those arms
/// are `ArduCopter2` here - a generic autopilot that is not a fixed wing reads as a copter in
/// Mission Planner, and in this port of it.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6695-6818; ExtLibs/ArduPilot/CurrentState.cs:102`
#[must_use]
pub fn firmware_of(autopilot: u8, mav_type: u8, banner: Option<&str>) -> Firmware {
    match autopilot {
        AUTOPILOT_ARDUPILOTMEGA => {
            // C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6704-6720
            const LOOKUP: [(&str, Firmware); 6] = [
                ("ArduPlane V", Firmware::ArduPlane),
                ("ArduCopter V", Firmware::ArduCopter2),
                ("Blimp V", Firmware::ArduCopter2),
                ("ArduRover V", Firmware::ArduRover),
                ("ArduSub V", Firmware::ArduSub),
                ("AntennaTracker V", Firmware::ArduTracker),
            ];
            if let Some(version) = banner.filter(|version| version.len() > 1) {
                let mut matches = LOOKUP
                    .iter()
                    .filter(|(prefix, _)| version.starts_with(prefix));
                if let (Some((_, firmware)), None) = (matches.next(), matches.next()) {
                    return *firmware;
                }
            }
            // C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6722-6783
            match mav_type {
                // FIXED_WING, FLAPPING_WING, VTOL_DUOROTOR to VTOL_RESERVED5.
                1 | 16 | 19..=25 => Firmware::ArduPlane,
                // QUADROTOR, COAXIAL, HELICOPTER, HEXAROTOR, OCTOROTOR, TRICOPTER, DODECAROTOR.
                2 | 3 | 4 | 13 | 14 | 15 | 29 => Firmware::ArduCopter2,
                // GROUND_ROVER, SURFACE_BOAT.
                10 | 11 => Firmware::ArduRover,
                12 => Firmware::ArduSub,
                5 => Firmware::ArduTracker,
                _ => Firmware::Other,
            }
        }
        // C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6787-6795
        AUTOPILOT_UDB if mav_type == TYPE_FIXED_WING => Firmware::ArduPlane,
        // C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6797-6805
        AUTOPILOT_GENERIC if mav_type == TYPE_FIXED_WING => Firmware::Ateryx,
        // C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6807-6809
        AUTOPILOT_PX4 => Firmware::Px4,
        // C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6811-6818 - a GIMBAL is `Gimbal`,
        // which no page branches on; anything else keeps the initial `ArduCopter2`.
        AUTOPILOT_UDB | AUTOPILOT_GENERIC => Firmware::ArduCopter2,
        _ if mav_type == 26 => Firmware::Other,
        _ => Firmware::ArduCopter2,
    }
}

/// The six parameters `Activate` reads into the combos, or `None` for a firmware it has no branch
/// for - whose combos stay empty.
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:40-225`
#[must_use]
pub const fn mode_parameters(firmware: Firmware) -> Option<[&'static str; POSITIONS]> {
    match firmware {
        Firmware::ArduPlane | Firmware::Ateryx | Firmware::ArduCopter2 => Some(FLTMODE),
        Firmware::ArduRover => Some(MODE),
        Firmware::Px4 => Some(COM_FLTMODE),
        Firmware::ArduSub | Firmware::ArduTracker | Firmware::Other => None,
    }
}

/// Whether the Simple and Super Simple check boxes and their link show.
///
/// Hidden on a plane, a rover and PX4; on a copter, hidden when the display view's
/// `standardFlightModesOnly` is set (`standard_only`); never touched - so shown - for a firmware
/// `Activate` does not branch on.
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:43-57, 81-95, 119-136, 184-198; ExtLibs/Utilities/DisplayView.cs:216`
#[must_use]
pub const fn simple_shown(firmware: Firmware, standard_only: bool) -> bool {
    match firmware {
        Firmware::ArduPlane | Firmware::Ateryx | Firmware::ArduRover | Firmware::Px4 => false,
        Firmware::ArduCopter2 => !standard_only,
        Firmware::ArduSub | Firmware::ArduTracker | Firmware::Other => true,
    }
}

/// `ProcessCmdKey`'s chord: Ctrl+S alone is Save Modes.
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:239-247`
#[must_use]
pub fn save_chord(event: &KeyDownEvent) -> bool {
    let keystroke = &event.keystroke;
    keystroke.modifiers.control
        && !keystroke.modifiers.alt
        && !keystroke.modifiers.shift
        && keystroke.key.eq_ignore_ascii_case("s")
}

impl MissionPlanner {
    /// A key on the page: Ctrl+S presses Save Modes, as `ProcessCmdKey` does. Whether the key
    /// was the page's.
    /// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:239-247`
    pub(crate) fn flight_modes_key(&mut self, event: &KeyDownEvent) -> bool {
        if !self.flight_modes.is_active() || !save_chord(event) {
            return false;
        }
        let view = self.telemetry.view();
        self.flight_modes.save(&view);
        true
    }
}

/// What the combos offer, as (value, name): `Common.getModesList` for the firmware, with PX4's
/// replaced by `COM_FLTMODE1`'s documented values as `Activate` replaces it.
///
/// `documented` answers a parameter's documented values for this firmware, or `None` when there
/// is no documentation for it. The C# reads `ParameterMetaDataRepository`; where that has nothing
/// for a plane, a copter or a rover, the list generated from the same metadata file is the answer
/// it would have given.
/// `// C#: ExtLibs/ArduPilot/Common.cs:88-176; GCSViews/ConfigurationView/ConfigFlightModes.cs:202-213`
pub fn modes_list(
    firmware: Firmware,
    documented: impl Fn(&str) -> Option<Vec<(i64, String)>>,
) -> Vec<(i64, String)> {
    let generated = |family: VehicleFamily| {
        family
            .modes()
            .iter()
            .map(|(value, name)| (i64::from(*value), (*name).to_owned()))
            .collect::<Vec<_>>()
    };
    match firmware {
        Firmware::Px4 => documented("COM_FLTMODE1").unwrap_or_default(),
        Firmware::ArduPlane => {
            let mut modes =
                documented("FLTMODE1").unwrap_or_else(|| generated(VehicleFamily::Plane));
            modes.push((16, "INITIALISING".to_owned()));
            modes
        }
        Firmware::Ateryx => documented("FLTMODE1").unwrap_or_default(),
        Firmware::ArduCopter2 => {
            documented("FLTMODE1").unwrap_or_else(|| generated(VehicleFamily::Copter))
        }
        Firmware::ArduRover => {
            documented("MODE1").unwrap_or_else(|| generated(VehicleFamily::Rover))
        }
        // `Activate` has no branch for these, so `updateDropDown` never runs and the combos stay
        // empty whatever `getModesList` would have said.
        Firmware::ArduSub | Firmware::ArduTracker | Firmware::Other => Vec::new(),
    }
}

/// A parameter's documented values, from the application's parameter documentation.
///
/// The bundled table is ArduCopter's, so for any other firmware only a file fetched for the
/// connected vehicle speaks for it; without one this says nothing and [`modes_list`] falls back.
fn documented_values(firmware: Firmware, name: &str) -> Option<Vec<(i64, String)>> {
    if firmware != Firmware::ArduCopter2 && crate::metadata::documented() == 0 {
        return None;
    }
    let meta = crate::metadata::lookup(name)?;
    (!meta.values.is_empty()).then(|| {
        meta.values
            .iter()
            .map(|(value, name)| (*value, (*name).to_owned()))
            .collect()
    })
}

/// Which combo a pulse width lights: `readSwitch`, "from arducopter code".
///
/// The pulse is truncated to an integer first, as `(int)inpwm` truncates.
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:341-352`
#[must_use]
pub fn read_switch(pwm: f32) -> usize {
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(int)inpwm`
    let pulsewidth = pwm as i32;
    if pulsewidth > 1230 && pulsewidth <= 1360 {
        return 1;
    }
    if pulsewidth > 1360 && pulsewidth <= 1490 {
        return 2;
    }
    if pulsewidth > 1490 && pulsewidth <= 1620 {
        return 3;
    }
    if pulsewidth > 1620 && pulsewidth <= 1749 {
        return 4; // Software Manual
    }
    if pulsewidth >= 1750 {
        return 5; // Hardware Manual
    }
    0
}

/// The mode channel's parameter and value: `FLTMODE_CH` if the vehicle has it, else `MODE_CH`.
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:263-274`
pub fn switch_parameter(value_of: impl Fn(&str) -> Option<f64>) -> Option<(&'static str, f64)> {
    value_of("FLTMODE_CH")
        .map(|value| ("FLTMODE_CH", value))
        .or_else(|| value_of("MODE_CH").map(|value| ("MODE_CH", value)))
}

/// The pulse width on the mode channel: `ch5in` to `ch16in` by the parameter's value, and zero
/// for anything else or a channel the receiver is not reporting (`chNin`'s default).
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:276-317`
#[must_use]
pub fn switch_pwm(channel: f64, rc: &RcChannels) -> f32 {
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(int)param.Value`
    let sw = channel as i64;
    match usize::try_from(sw) {
        Ok(number @ 5..=16) => rc.channel(number).map_or(0.0, f32::from),
        _ => 0.0,
    }
}

/// Six check boxes from a bitmask: bit N is position N + 1, as `simple >> N & 1` reads it.
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:154-176`
#[must_use]
pub fn unpack_switches(mask: i64) -> [bool; POSITIONS] {
    std::array::from_fn(|bit| (mask >> bit) & 1 == 1)
}

/// A bitmask from six check boxes: `SimpleMode.Simple1` (1) to `Simple6` (32), summed.
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:12-22, 389-404`
#[must_use]
pub fn pack_switches(checked: [bool; POSITIONS]) -> f64 {
    // An integer mask, as the C#'s `int` is: summing floats would give -0.0 for no switches,
    // since that is the identity Rust's `Sum for f64` starts from, and the vehicle would be
    // sent - and the facts would show - "-0".
    let mask: u32 = checked
        .iter()
        .enumerate()
        .filter(|(_, checked)| **checked)
        .map(|(bit, _)| 1u32 << bit)
        .sum();
    f64::from(mask)
}

/// Whether a copter mode takes Simple and Super Simple: `flightmode_SelectedIndexChanged`, which
/// enables a row's two check boxes when the combo's text names one of these, and disables them
/// otherwise.
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:436-467`
#[must_use]
pub fn simple_allowed(mode: &str) -> bool {
    const TAKES_SIMPLE: [&str; 12] = [
        "althold",
        "auto",
        "autotune",
        "land",
        "loiter",
        "ofloiter",
        "poshold",
        "rtl",
        "sport",
        "stabilize",
        "flowhold",
        "zigzag",
    ];
    let currentmode = mode.to_lowercase();
    TAKES_SIMPLE.iter().any(|name| currentmode.contains(name))
}

/// A parameter's value as `int.Parse(param.ToString())` reads it: whole numbers only.
fn whole(value: f64) -> Option<i64> {
    #[allow(clippy::cast_possible_truncation)] // checked whole below, and parameters are small
    let truncated = value as i64;
    (value.fract() == 0.0 && value.is_finite()).then_some(truncated)
}

/// What Save Modes writes, in the order it writes it: the six modes under whichever names the
/// vehicle has - `FLTMODE`, else `MODE`, else `COM_FLTMODE` - then, on a copter, `SIMPLE` and
/// `SUPER_SIMPLE` if it has them.
///
/// A combo with nothing selected is `None`: `SelectedValue.ToString()` throws there, which ends
/// the save with an error after whatever was written before it.
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:354-407`
pub fn save_set(
    firmware: Firmware,
    holds: impl Fn(&str) -> bool,
    selected: &[Option<i64>; POSITIONS],
    simple: [bool; POSITIONS],
    super_simple: [bool; POSITIONS],
) -> Vec<(&'static str, Option<f64>)> {
    let names = if holds("FLTMODE1") {
        Some(FLTMODE)
    } else if holds("MODE1") {
        Some(MODE)
    } else if holds("COM_FLTMODE1") {
        Some(COM_FLTMODE)
    } else {
        None
    };
    let mut writes: Vec<(&'static str, Option<f64>)> = names
        .into_iter()
        .flat_map(|names| {
            names.into_iter().zip(selected.iter()).map(|(name, value)| {
                #[allow(clippy::cast_precision_loss)] // mode numbers are small
                let value = value.map(|value| value as f64);
                (name, value)
            })
        })
        .collect();
    if firmware == Firmware::ArduCopter2 {
        if holds("SIMPLE") {
            writes.push(("SIMPLE", Some(pack_switches(simple))));
        }
        if holds("SUPER_SIMPLE") {
            writes.push(("SUPER_SIMPLE", Some(pack_switches(super_simple))));
        }
    }
    writes
}

/// How a write is getting on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Progress {
    /// Not answered yet.
    Waiting,
    /// Over.
    Finished(RequestOutcome),
    /// The link no longer knows of it.
    Lost,
}

/// Something that writes parameters and says how each write went: the link, or a test's stand-in.
pub trait ParamWriter {
    /// What a write is known by.
    type Handle: Copy;
    /// Starts one write; `None` when there is nothing to write to.
    fn write(&self, name: &str, value: f64) -> Option<Self::Handle>;
    /// How it is getting on.
    fn progress(&self, handle: Self::Handle) -> Progress;
}

impl ParamWriter for Telemetry {
    type Handle = mp_link::RequestId;

    fn write(&self, name: &str, value: f64) -> Option<Self::Handle> {
        self.set_parameter_confirmed(name, value)
    }

    fn progress(&self, handle: Self::Handle) -> Progress {
        match self.request(handle) {
            None => Progress::Lost,
            Some(request) => request
                .outcome()
                .map_or(Progress::Waiting, Progress::Finished),
        }
    }
}

/// Where Save Modes is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SaveState {
    /// Not pressed.
    Idle,
    /// Writing.
    Saving,
    /// Done: the button reads "Complete".
    Complete,
    /// Done after a write failed: "Complete", and `Strings.ErrorSettingParameter`.
    Failed,
}

impl SaveState {
    /// For the facts.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Idle => "idle",
            Self::Saving => "saving",
            Self::Complete => "complete",
            Self::Failed => "failed",
        }
    }
}

/// Save Modes' writes, one at a time and in order, as the C#'s blocking `setParam` calls go.
#[derive(Debug)]
pub struct Saver<H> {
    queue: VecDeque<(&'static str, Option<f64>)>,
    waiting: Option<H>,
    state: SaveState,
}

impl<H> Default for Saver<H> {
    fn default() -> Self {
        Self {
            queue: VecDeque::new(),
            waiting: None,
            state: SaveState::Idle,
        }
    }
}

impl<H: Copy> Saver<H> {
    /// Where it is.
    #[must_use]
    pub const fn state(&self) -> SaveState {
        self.state
    }

    /// Starts writing a [`save_set`].
    pub fn start(&mut self, writes: Vec<(&'static str, Option<f64>)>) {
        self.queue = writes.into();
        self.waiting = None;
        self.state = SaveState::Saving;
    }

    /// Moves the writes on: collects an answered one and starts the next.
    ///
    /// A write that times out ends the save, as `setParam`'s `TimeoutException` ends the C#'s;
    /// one the vehicle does not have, or already holds the value of, does not (`setParam` returns
    /// false or true for those, and the C# does not look). An empty combo ends it too.
    /// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:356-413; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1640-1651, 1765`
    pub fn advance<W: ParamWriter<Handle = H>>(&mut self, writer: &W) {
        if self.state != SaveState::Saving {
            return;
        }
        loop {
            if let Some(handle) = self.waiting {
                match writer.progress(handle) {
                    Progress::Waiting => return,
                    Progress::Finished(RequestOutcome::TimedOut) | Progress::Lost => {
                        return self.fail();
                    }
                    Progress::Finished(_) => self.waiting = None,
                }
            }
            let Some((name, value)) = self.queue.pop_front() else {
                self.state = SaveState::Complete;
                return;
            };
            let Some(value) = value else {
                return self.fail();
            };
            match writer.write(name, value) {
                Some(handle) => self.waiting = Some(handle),
                None => return self.fail(),
            }
        }
    }

    fn fail(&mut self) {
        self.queue.clear();
        self.waiting = None;
        self.state = SaveState::Failed;
    }
}

/// The page's state: what `Activate` read, what the operator has changed since, and the save.
#[derive(Debug, Default)]
pub struct FlightModes {
    /// Between `Activate` and `Deactivate`: the page is showing.
    active: bool,
    firmware: Firmware,
    /// The combos' `DataSource`.
    list: Vec<(i64, String)>,
    /// Each combo's `SelectedValue`; `None` is an empty combo.
    selected: [Option<i64>; POSITIONS],
    simple: [bool; POSITIONS],
    super_simple: [bool; POSITIONS],
    /// Which combo's list is open.
    open: Option<usize>,
    saver: Saver<mp_link::RequestId>,
}

impl FlightModes {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// Opens the page if it is closed and closes it if it is open.
    pub fn toggle(&mut self, telemetry: &Telemetry) {
        if self.active {
            self.deactivate();
        } else {
            self.activate(&telemetry.view(), telemetry.firmware_banner());
        }
    }

    /// `Activate`: fills the combos with the vehicle's modes and sets them, and the check boxes,
    /// from its parameters.
    ///
    /// Read in the C#'s order and stopped where it would throw - a mode parameter missing or not a
    /// whole number leaves that combo and every one after it empty, and the check boxes unread.
    /// A value the list does not hold leaves its combo empty, as `SelectedValue` does.
    /// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:38-232`
    pub fn activate(&mut self, view: &TelemetryView, banner: Option<&str>) {
        let firmware = view.state.as_ref().map_or(Firmware::ArduCopter2, |state| {
            firmware_of(state.autopilot, state.vehicle_type, banner)
        });
        *self = Self {
            active: true,
            firmware,
            list: modes_list(firmware, |name| documented_values(firmware, name)),
            saver: std::mem::take(&mut self.saver),
            ..Self::default()
        };
        let Some(names) = mode_parameters(firmware) else {
            return;
        };
        for (slot, name) in self.selected.iter_mut().zip(names) {
            let Some(value) = parameter(view, name).and_then(whole) else {
                return;
            };
            *slot = self
                .list
                .iter()
                .any(|(listed, _)| *listed == value)
                .then_some(value);
        }
        if firmware == Firmware::ArduCopter2 {
            for (name, boxes) in [
                ("SIMPLE", &mut self.simple),
                ("SUPER_SIMPLE", &mut self.super_simple),
            ] {
                if let Some(value) = parameter(view, name) {
                    let Some(mask) = whole(value) else {
                        return;
                    };
                    *boxes = unpack_switches(mask);
                }
            }
        }
    }

    /// `Deactivate`: the page stops updating. A save under way carries on.
    /// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:234-237`
    pub fn deactivate(&mut self) {
        self.active = false;
        self.open = None;
    }

    /// Once a frame: moves a save on.
    pub fn tick(&mut self, telemetry: &Telemetry) {
        self.saver.advance(telemetry);
    }

    /// Save Modes.
    fn save(&mut self, view: &TelemetryView) {
        let writes = save_set(
            self.firmware,
            |name| parameter(view, name).is_some(),
            &self.selected,
            self.simple,
            self.super_simple,
        );
        self.saver.start(writes);
    }

    /// The name a combo shows.
    fn name_of(&self, value: Option<i64>) -> Option<&str> {
        let value = value?;
        self.list
            .iter()
            .find(|(listed, _)| *listed == value)
            .map(|(_, name)| name.as_str())
    }

    /// Whether a row's check boxes take clicks: on a copter, only for a mode that takes Simple.
    fn simple_enabled(&self, row: usize) -> bool {
        if self.firmware != Firmware::ArduCopter2 {
            return true;
        }
        let selected = self.selected.get(row).copied().flatten();
        simple_allowed(self.name_of(selected).unwrap_or(""))
    }
}

/// A parameter's value, if the vehicle has listed it.
fn parameter(view: &TelemetryView, name: &str) -> Option<f64> {
    view.parameters
        .iter()
        .find(|(held, _)| held == name)
        .map(|(_, value)| *value)
}

/// What the timer shows: the mode channel's label text, its pulse width, and the lit row.
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.cs:250-339`
fn switch_reading(view: &TelemetryView) -> (String, f32, usize) {
    let rc = view
        .state
        .as_ref()
        .map(|state| state.rc)
        .unwrap_or_default();
    match switch_parameter(|name| parameter(view, name)) {
        Some((_, channel)) => {
            let pwm = switch_pwm(channel, &rc);
            (format!("{channel}: {pwm}"), pwm, read_switch(pwm))
        }
        // `LBL_flightmodepwm` keeps its designer text, and `pwm` its zero.
        None => ("0".to_owned(), 0.0, read_switch(0.0)),
    }
}

/// `lbl_currentmode`, bound to `cs.mode`: the vehicle's mode named from the same list.
/// `// C#: GCSViews/ConfigurationView/ConfigFlightModes.Designer.cs:129; ExtLibs/ArduPilot/CurrentState.cs:2899-2923`
fn current_mode(modes: &FlightModes, view: &TelemetryView) -> String {
    view.state
        .as_ref()
        .and_then(|state| modes.name_of(Some(i64::from(state.custom_mode))))
        .unwrap_or("Unknown")
        .to_owned()
}

/// Facts for a test: what the page read, what it shows, and how the save went; and the vehicle's
/// value of every parameter the page touches, as `params.value.<name>`.
pub fn record_facts(modes: &FlightModes, view: &TelemetryView) {
    use crate::facts::record;
    record("config.flightmodes.active", modes.active);
    record("config.flightmodes.firmware", modes.firmware.label());
    record("config.flightmodes.modes", modes.list.len());
    for (row, value) in modes.selected.iter().enumerate() {
        let position = row + 1;
        record(
            format!("config.flightmodes.{position}"),
            value.map_or_else(|| "none".to_owned(), |value| value.to_string()),
        );
        record(
            format!("config.flightmodes.{position}.name"),
            modes.name_of(*value).unwrap_or(""),
        );
    }
    record("config.flightmodes.simple", pack_switches(modes.simple));
    record(
        "config.flightmodes.supersimple",
        pack_switches(modes.super_simple),
    );
    let (label, pwm, lit) = switch_reading(view);
    record(
        "config.flightmodes.channel",
        switch_parameter(|name| parameter(view, name))
            .map_or_else(|| "none".to_owned(), |(_, channel)| channel.to_string()),
    );
    record("config.flightmodes.pwm", pwm);
    record("config.flightmodes.pwm.label", label);
    record("config.flightmodes.current", lit + 1);
    record("config.flightmodes.mode", current_mode(modes, view));
    record("config.flightmodes.save", modes.saver.state().label());
    for name in WATCHED {
        if let Some(value) = parameter(view, name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

/// The page, while it is showing.
pub fn page(
    modes: &FlightModes,
    view: &TelemetryView,
    focus: &FocusHandle,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if !modes.active {
        return None;
    }
    let (pwm_label, _, lit) = switch_reading(view);
    let show_simple = simple_shown(
        modes.firmware,
        crate::display_view::flag("standardFlightModesOnly"),
    );

    // "Current Mode:" and "Current PWM:", label13/lbl_currentmode and label14/LBL_flightmodepwm,
    // at x 94 and 174 above the table.
    let readout = |label: &'static str, value: String| {
        div()
            .flex()
            .items_center()
            .child(
                div()
                    .w(px(80.0))
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(label),
            )
            .child(div().text_xs().text_color(rgb(theme::TEXT)).child(value))
    };
    let header = div()
        .pl(px(94.0))
        .flex()
        .flex_col()
        .child(readout("Current Mode:", current_mode(modes, view)))
        .child(readout("Current PWM:", pwm_label));

    let mut table = div().flex().flex_col();
    for (row, band) in BANDS.iter().enumerate() {
        let enabled = modes.simple_enabled(row);
        let simple = modes.simple.get(row).copied().unwrap_or(false);
        let super_simple = modes.super_simple.get(row).copied().unwrap_or(false);
        table = table.child(
            div()
                .flex()
                .items_center()
                .h(px(ROW))
                .child(cell(0).child(format!("Flight Mode {}", row + 1)))
                .child(cell(1).child(combo(modes, row, row == lit, cx)))
                .child(cell(2).children(show_simple.then(|| {
                    check_box(
                        format!("fm-simple-{}", row + 1),
                        "Simple Mode",
                        simple,
                        enabled,
                        cx.listener(move |this, _event, _window, cx| {
                            if let Some(checked) = this.flight_modes.simple.get_mut(row) {
                                *checked = !*checked;
                            }
                            cx.notify();
                        }),
                    )
                })))
                .child(cell(3).children(show_simple.then(|| {
                    check_box(
                        format!("fm-ssimple-{}", row + 1),
                        "Super Simple Mode",
                        super_simple,
                        enabled,
                        cx.listener(move |this, _event, _window, cx| {
                            if let Some(checked) = this.flight_modes.super_simple.get_mut(row) {
                                *checked = !*checked;
                            }
                            cx.notify();
                        }),
                    )
                })))
                .child(cell(4).text_color(rgb(theme::DIM)).child(*band)),
        );
        if modes.open == Some(row) {
            table = table.child(options(modes, row, cx));
        }
    }

    let saving = modes.saver.state() == SaveState::Saving;
    let save_label = match modes.saver.state() {
        SaveState::Idle | SaveState::Saving => "Save Modes",
        SaveState::Complete | SaveState::Failed => "Complete",
    };
    table = table.child(
        div()
            .flex()
            .items_center()
            .pt_1()
            .child(cell(0))
            .child(cell(1).child(action(
                "fm-save",
                save_label,
                theme::ACCENT,
                !saving,
                cx.listener(|this, _event: &(), _window, cx| {
                    let view = this.telemetry.view();
                    this.flight_modes.save(&view);
                    cx.notify();
                }),
            )))
            .child(cell(2))
            .child(cell(3).children(show_simple.then(|| {
                crate::probe::measured("fm-simple-help", div())
                    .id("fm-simple-help")
                    .text_xs()
                    .text_color(rgb(theme::ACCENT))
                    .underline()
                    .cursor_pointer()
                    .child("Simple and Super Simple description")
                    .on_click(|_event, _window, cx| cx.open_url(SIMPLE_HELP))
            }))),
    );

    Some(
        panel(
            "flight modes",
            // The page takes the keyboard when it is clicked, so `ProcessCmdKey`'s Ctrl+S
            // reaches it from any of its controls.
            div()
                .id("fm-page")
                .track_focus(focus)
                .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                    if this.flight_modes_key(event) {
                        cx.stop_propagation();
                        cx.notify();
                    }
                }))
                .on_mouse_down(
                    MouseButton::Left,
                    cx.listener(|this, _event, window, cx| {
                        window.focus(&this.flight_modes_focus, cx);
                    }),
                )
                .flex()
                .flex_col()
                .gap_2()
                .child(header)
                .child(table)
                .children((modes.saver.state() == SaveState::Failed).then(|| {
                    div()
                        .text_xs()
                        .text_color(rgb(theme::ALERT))
                        .child(ERROR_SETTING_PARAMETER)
                })),
        )
        .into_any_element(),
    )
}

/// One cell of the table, at its column's width.
fn cell(column: usize) -> gpui::Div {
    div()
        .flex_shrink_0()
        .w(px(COLUMNS.get(column).copied().unwrap_or(0.0)))
        .text_xs()
        .text_color(rgb(theme::TEXT))
}

/// `CMB_fmodeN`: the chosen mode, lit when the switch is on this row; a click opens the list.
fn combo(
    modes: &FlightModes,
    row: usize,
    lit: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let id = format!("fm-mode{}", row + 1);
    let selected = modes.selected.get(row).copied().flatten();
    let name = modes.name_of(selected).unwrap_or("").to_owned();
    // `ThemeManager.CurrentPPMBackground`, green, on the combo the switch selects.
    let colour = if lit { theme::OK } else { theme::BORDER };
    crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .w(px(104.0))
        .h(px(20.0))
        .px_1()
        .flex()
        .items_center()
        .justify_between()
        .rounded_sm()
        .border_1()
        .border_color(rgb(colour))
        .bg(rgb(if lit { theme::ACTION } else { theme::BG }))
        .text_xs()
        .text_color(rgb(if lit { theme::OK } else { theme::TEXT }))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .child(name)
        .child("▾")
        .on_click(cx.listener(move |this, _event, _window, cx| {
            let modes = &mut this.flight_modes;
            modes.open = (modes.open != Some(row)).then_some(row);
            cx.notify();
        }))
        .into_any_element()
}

/// An open combo's list, beneath its row: every mode, the chosen one marked.
fn options(modes: &FlightModes, row: usize, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let selected = modes.selected.get(row).copied().flatten();
    let mut list = div()
        .flex()
        .flex_wrap()
        .gap_1()
        .py_1()
        .ml(px(COLUMNS[0]))
        .w(px(COLUMNS.iter().skip(1).sum::<f32>()));
    for (value, name) in &modes.list {
        let value = *value;
        let chosen = selected == Some(value);
        let id = format!("fm-mode{}-{value}", row + 1);
        list = list.child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .px_2()
                .py(px(1.0))
                .rounded_sm()
                .border_1()
                .border_color(rgb(if chosen { theme::ACCENT } else { theme::BORDER }))
                .bg(rgb(if chosen { theme::ACTION } else { theme::PANEL }))
                .text_xs()
                .text_color(rgb(if chosen { theme::ACCENT } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(name.clone())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    let modes = &mut this.flight_modes;
                    if let Some(slot) = modes.selected.get_mut(row) {
                        *slot = Some(value);
                    }
                    modes.open = None;
                    cx.notify();
                })),
        );
    }
    list.into_any_element()
}

/// A check box: a square that fills when checked, and its text. Dimmed and inert when disabled,
/// keeping its tick, as a disabled `CheckBox` keeps `Checked`.
fn check_box(
    id: String,
    label: &'static str,
    checked: bool,
    enabled: bool,
    on_click: impl Fn(&gpui::ClickEvent, &mut gpui::Window, &mut gpui::App) + 'static,
) -> AnyElement {
    let colour = if enabled { theme::TEXT } else { theme::DIM };
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
    let body = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .flex()
        .items_center()
        .gap_1()
        .text_xs()
        .text_color(rgb(colour))
        .child(square)
        .child(label);
    if enabled {
        body.cursor_pointer().on_click(on_click).into_any_element()
    } else {
        body.into_any_element()
    }
}

#[cfg(test)]
mod tests {

    /// No switches is a positive zero, as the C#'s `int` mask is: `Sum for f64` would give
    /// -0.0, which prints as "-0" and is not what the vehicle should be sent.
    #[test]
    fn no_switches_pack_to_a_positive_zero() {
        let packed = pack_switches([false; POSITIONS]);
        assert!(packed.is_sign_positive(), "{packed}");
        assert_eq!(format!("{packed}"), "0");
        assert_eq!(pack_switches([true, false, true, false, false, false]), 5.0);
    }
    use super::*;

    /// The six bands, at and either side of every threshold the C# names: 1230, 1360, 1490, 1620,
    /// 1749/1750.
    #[test]
    fn the_switch_bands_are_the_csharps_thresholds() {
        let cases = [
            (0.0, 0),
            (1000.0, 0),
            (1230.0, 0),
            (1231.0, 1),
            (1360.0, 1),
            (1361.0, 2),
            (1490.0, 2),
            (1491.0, 3),
            (1500.0, 3),
            (1620.0, 3),
            (1621.0, 4),
            (1749.0, 4),
            (1750.0, 5),
            (2000.0, 5),
        ];
        for (pwm, row) in cases {
            assert_eq!(read_switch(pwm), row, "{pwm}");
        }
        // `(int)inpwm` truncates, so a pulse a fraction over a threshold is still under it.
        assert_eq!(read_switch(1230.9), 0);
        assert_eq!(read_switch(1749.9), 4);
    }

    /// The bands the rows are labelled with say what `read_switch` does.
    #[test]
    fn each_label_names_the_band_its_row_is_lit_for() {
        for (row, label) in BANDS.iter().enumerate() {
            let numbers: Vec<f32> = label
                .trim_start_matches("PWM ")
                .split(" - ")
                .filter_map(|part| part.trim_end_matches(" +").parse().ok())
                .collect();
            for pwm in numbers {
                assert_eq!(read_switch(pwm), row, "{label}: {pwm}");
            }
        }
    }

    /// The pulse comes from the channel `FLTMODE_CH` names, or `MODE_CH` without it; channels
    /// below five and above sixteen, and a silent channel, read zero.
    #[test]
    fn the_mode_channel_is_fltmode_ch_else_mode_ch() {
        let both = |name: &str| match name {
            "FLTMODE_CH" => Some(5.0),
            "MODE_CH" => Some(8.0),
            _ => None,
        };
        assert_eq!(switch_parameter(both), Some(("FLTMODE_CH", 5.0)));
        let rover = |name: &str| (name == "MODE_CH").then_some(8.0);
        assert_eq!(switch_parameter(rover), Some(("MODE_CH", 8.0)));
        assert_eq!(switch_parameter(|_| None), None);

        let mut rc = RcChannels::default();
        rc.values[3] = 1100;
        rc.values[4] = 1400;
        rc.values[15] = 1900;
        assert_eq!(switch_pwm(5.0, &rc), 1400.0);
        assert_eq!(switch_pwm(16.0, &rc), 1900.0);
        assert_eq!(switch_pwm(4.0, &rc), 0.0, "the C#'s switch starts at 5");
        assert_eq!(switch_pwm(17.0, &rc), 0.0);
        assert_eq!(switch_pwm(0.0, &rc), 0.0, "FLTMODE_CH 0 is Disabled");
        assert_eq!(switch_pwm(6.0, &rc), 0.0, "not reported is ch6in's zero");
        assert_eq!(read_switch(switch_pwm(5.0, &rc)), 2);
    }

    /// Bit N is position N + 1, both ways, and every combination survives the round trip.
    #[test]
    fn the_simple_bitmask_packs_and_unpacks() {
        assert_eq!(
            unpack_switches(0b10_0101),
            [true, false, true, false, false, true]
        );
        assert_eq!(pack_switches([true, false, true, false, false, true]), 37.0);
        assert_eq!(pack_switches([false; POSITIONS]), 0.0);
        assert_eq!(pack_switches([true; POSITIONS]), 63.0);
        assert_eq!(
            pack_switches([false, true, false, false, false, false]),
            2.0
        );
        for mask in 0..64 {
            #[allow(clippy::cast_precision_loss)]
            let expected = mask as f64;
            assert_eq!(pack_switches(unpack_switches(mask)), expected);
        }
        // Bits above the sixth are not positions.
        assert_eq!(unpack_switches(64), [false; POSITIONS]);
    }

    /// The firmware, as `setAPType` reads it: the banner first, then the airframe.
    #[test]
    fn the_firmware_is_read_as_set_ap_type_reads_it() {
        assert_eq!(firmware_of(3, 2, None), Firmware::ArduCopter2);
        assert_eq!(firmware_of(3, 13, None), Firmware::ArduCopter2);
        assert_eq!(firmware_of(3, 1, None), Firmware::ArduPlane);
        assert_eq!(firmware_of(3, 20, None), Firmware::ArduPlane, "a quadplane");
        assert_eq!(firmware_of(3, 10, None), Firmware::ArduRover);
        assert_eq!(firmware_of(3, 11, None), Firmware::ArduRover, "a boat");
        assert_eq!(firmware_of(3, 12, None), Firmware::ArduSub);
        assert_eq!(firmware_of(3, 5, None), Firmware::ArduTracker);
        assert_eq!(firmware_of(3, 7, None), Firmware::Other);
        // The banner wins: a quadplane reports a VTOL type, and a blimp is flown as a copter.
        assert_eq!(
            firmware_of(3, 2, Some("ArduPlane V4.5.7 (1c0c8d9c)")),
            Firmware::ArduPlane
        );
        assert_eq!(
            firmware_of(3, 7, Some("Blimp V4.5.0")),
            Firmware::ArduCopter2
        );
        assert_eq!(
            firmware_of(3, 1, Some("ChibiOS: 6a85082c")),
            Firmware::ArduPlane,
            "a banner naming nothing falls through to the type"
        );
        assert_eq!(firmware_of(12, 2, None), Firmware::Px4);
        assert_eq!(firmware_of(0, 1, None), Firmware::Ateryx);
        assert_eq!(firmware_of(10, 1, None), Firmware::ArduPlane);
        // Left untouched, `cs.firmware` is its initial ArduCopter2.
        assert_eq!(firmware_of(0, 2, None), Firmware::ArduCopter2);
        assert_eq!(firmware_of(8, 26, None), Firmware::Other, "a gimbal");
    }

    /// Plane, copter and rover read their own parameters; PX4 its own; anything else none.
    #[test]
    fn each_firmware_reads_its_own_mode_parameters() {
        assert_eq!(mode_parameters(Firmware::ArduCopter2), Some(FLTMODE));
        assert_eq!(mode_parameters(Firmware::ArduPlane), Some(FLTMODE));
        assert_eq!(mode_parameters(Firmware::Ateryx), Some(FLTMODE));
        assert_eq!(mode_parameters(Firmware::ArduRover), Some(MODE));
        assert_eq!(mode_parameters(Firmware::Px4), Some(COM_FLTMODE));
        assert_eq!(mode_parameters(Firmware::ArduSub), None);
        assert_eq!(mode_parameters(Firmware::Other), None);

        assert!(simple_shown(Firmware::ArduCopter2, false));
        assert!(
            !simple_shown(Firmware::ArduCopter2, true),
            "standardFlightModesOnly hides them on a copter"
        );
        assert!(!simple_shown(Firmware::ArduPlane, false));
        assert!(!simple_shown(Firmware::ArduRover, false));
        assert!(!simple_shown(Firmware::Px4, false));
        assert!(simple_shown(Firmware::ArduSub, true), "Activate never hides them");
    }

    /// `ProcessCmdKey`: Ctrl+S alone, whatever the letter's case; not with Shift or Alt, and
    /// not another letter.
    #[test]
    fn ctrl_s_is_the_save_chord() {
        let press = |key: &str, control: bool, shift: bool, alt: bool| KeyDownEvent {
            keystroke: gpui::Keystroke {
                modifiers: gpui::Modifiers {
                    control,
                    shift,
                    alt,
                    ..gpui::Modifiers::default()
                },
                key: key.to_owned(),
                key_char: None,
            },
            is_held: false,
            prefer_character_input: false,
        };
        assert!(save_chord(&press("s", true, false, false)));
        assert!(save_chord(&press("S", true, false, false)));
        assert!(!save_chord(&press("s", false, false, false)));
        assert!(!save_chord(&press("s", true, true, false)));
        assert!(!save_chord(&press("s", true, false, true)));
        assert!(!save_chord(&press("d", true, false, false)));
    }

    /// The copter list is `FLTMODE1`'s documented values; a plane's gets INITIALISING on the
    /// end; a rover's is `MODE1`'s; with no documentation the generated lists stand in.
    #[test]
    fn the_mode_list_is_per_vehicle() {
        let none = |_: &str| None;
        let copter = modes_list(Firmware::ArduCopter2, none);
        assert_eq!(copter.first(), Some(&(0, "Stabilize".to_owned())));
        assert!(copter.contains(&(9, "Land".to_owned())));
        assert!(copter.contains(&(27, "Auto RTL".to_owned())));
        assert!(!copter.iter().any(|(value, _)| *value == 8), "8 is unused");

        let plane = modes_list(Firmware::ArduPlane, none);
        assert_eq!(plane.first(), Some(&(0, "Manual".to_owned())));
        assert_eq!(plane.last(), Some(&(16, "INITIALISING".to_owned())));
        assert!(plane.contains(&(5, "FBWA".to_owned())));

        let rover = modes_list(Firmware::ArduRover, none);
        assert!(rover.contains(&(4, "Hold".to_owned())));
        assert!(!rover.iter().any(|(_, name)| name == "INITIALISING"));

        // Documentation, where there is some, is the list.
        let documented = |name: &str| {
            (name == "FLTMODE1").then(|| vec![(0, "Manual".to_owned()), (10, "Auto".to_owned())])
        };
        assert_eq!(
            modes_list(Firmware::ArduPlane, documented),
            vec![
                (0, "Manual".to_owned()),
                (10, "Auto".to_owned()),
                (16, "INITIALISING".to_owned())
            ]
        );
        assert_eq!(modes_list(Firmware::ArduCopter2, documented).len(), 2);
        assert_eq!(modes_list(Firmware::Ateryx, documented).len(), 2);
        // PX4's is COM_FLTMODE1's documentation and nothing else.
        assert!(modes_list(Firmware::Px4, none).is_empty());
        assert!(modes_list(Firmware::ArduSub, documented).is_empty());
    }

    /// The bundled documentation is a copter's, and gives the copter list the generated table
    /// does - the two come from the same file, so the fallback is what the C# would have read.
    #[test]
    fn the_bundled_copter_documentation_is_the_generated_list() {
        let bundled: Vec<(i64, String)> = mp_params::param_meta::lookup("FLTMODE1")
            .expect("FLTMODE1 is documented")
            .values
            .iter()
            .map(|(value, name)| (*value, (*name).to_owned()))
            .collect();
        assert_eq!(bundled, modes_list(Firmware::ArduCopter2, |_| None));
    }

    /// A row's Simple boxes take clicks only for a mode whose name holds one of the C#'s words.
    #[test]
    fn simple_is_offered_for_the_csharps_modes() {
        for mode in [
            "Stabilize",
            "AltHold",
            "Auto",
            "AutoTune",
            "Land",
            "Loiter",
            "PosHold",
            "RTL",
            "Smart_RTL",
            "Auto RTL",
            "Sport",
            "FlowHold",
            "ZigZag",
        ] {
            assert!(simple_allowed(mode), "{mode}");
        }
        for mode in [
            "Acro", "Guided", "Circle", "Drift", "Flip", "Brake", "Throw", "",
        ] {
            assert!(!simple_allowed(mode), "{mode}");
        }
    }

    /// Save writes the six under the vehicle's names, then SIMPLE and SUPER_SIMPLE on a copter
    /// that has them, in that order.
    #[test]
    fn the_save_set_is_the_csharps() {
        let selected = [Some(7), Some(5), Some(6), Some(3), Some(5), Some(0)];
        let simple = [false, true, false, false, false, false];
        let super_simple = [true, false, false, false, false, true];
        let copter =
            |name: &str| name.starts_with("FLTMODE") || name == "SIMPLE" || name == "SUPER_SIMPLE";
        assert_eq!(
            save_set(
                Firmware::ArduCopter2,
                copter,
                &selected,
                simple,
                super_simple
            ),
            vec![
                ("FLTMODE1", Some(7.0)),
                ("FLTMODE2", Some(5.0)),
                ("FLTMODE3", Some(6.0)),
                ("FLTMODE4", Some(3.0)),
                ("FLTMODE5", Some(5.0)),
                ("FLTMODE6", Some(0.0)),
                ("SIMPLE", Some(2.0)),
                ("SUPER_SIMPLE", Some(33.0)),
            ]
        );
        // A plane has FLTMODE and no SIMPLE write, whatever the boxes say.
        let plane = save_set(Firmware::ArduPlane, copter, &selected, simple, super_simple);
        assert_eq!(plane.len(), POSITIONS);
        assert!(plane.iter().all(|(name, _)| name.starts_with("FLTMODE")));
        // A rover writes MODE1-6.
        let rover = |name: &str| name.starts_with("MODE");
        let written = save_set(Firmware::ArduRover, rover, &selected, simple, super_simple);
        assert_eq!(written.first(), Some(&("MODE1", Some(7.0))));
        assert_eq!(written.len(), POSITIONS);
        // PX4's names.
        let px4 = |name: &str| name.starts_with("COM_FLTMODE");
        let written = save_set(Firmware::Px4, px4, &selected, simple, super_simple);
        assert_eq!(written.last(), Some(&("COM_FLTMODE6", Some(0.0))));
        // Nothing the vehicle has, nothing written.
        assert!(
            save_set(
                Firmware::ArduSub,
                |_| false,
                &selected,
                simple,
                super_simple
            )
            .is_empty()
        );
        // A copter without SIMPLE writes the six only.
        let bare = |name: &str| name.starts_with("FLTMODE");
        assert_eq!(
            save_set(Firmware::ArduCopter2, bare, &selected, simple, super_simple).len(),
            POSITIONS
        );
        // An empty combo is carried, so the save can stop there.
        let gap = [Some(7), None, Some(6), Some(3), Some(5), Some(0)];
        let written = save_set(Firmware::ArduCopter2, copter, &gap, simple, super_simple);
        assert_eq!(written.get(1), Some(&("FLTMODE2", None)));
    }

    /// A stand-in link: every write answered at once with the outcome given for its name.
    struct Answering {
        outcomes: std::collections::BTreeMap<&'static str, Progress>,
        written: std::cell::RefCell<Vec<(String, f64)>>,
    }

    impl Answering {
        fn new(outcomes: &[(&'static str, Progress)]) -> Self {
            Self {
                outcomes: outcomes.iter().copied().collect(),
                written: std::cell::RefCell::new(Vec::new()),
            }
        }
        fn names(&self) -> Vec<String> {
            self.written
                .borrow()
                .iter()
                .map(|(name, _)| name.clone())
                .collect()
        }
    }

    impl ParamWriter for Answering {
        type Handle = usize;
        fn write(&self, name: &str, value: f64) -> Option<usize> {
            let mut written = self.written.borrow_mut();
            written.push((name.to_owned(), value));
            Some(written.len() - 1)
        }
        fn progress(&self, handle: usize) -> Progress {
            let written = self.written.borrow();
            let Some((name, _)) = written.get(handle) else {
                return Progress::Lost;
            };
            self.outcomes
                .get(name.as_str())
                .copied()
                .unwrap_or(Progress::Finished(RequestOutcome::Accepted { value: None }))
        }
    }

    /// Every write in order, one after another, then Complete.
    #[test]
    fn a_save_writes_in_order_and_completes() {
        let link = Answering::new(&[
            ("FLTMODE3", Progress::Finished(RequestOutcome::Unchanged)),
            (
                "FLTMODE4",
                Progress::Finished(RequestOutcome::UnknownParameter),
            ),
        ]);
        let mut saver = Saver::default();
        assert_eq!(saver.state(), SaveState::Idle);
        saver.start(save_set(
            Firmware::ArduCopter2,
            |_| true,
            &[Some(7), Some(5), Some(6), Some(3), Some(5), Some(0)],
            [false, true, false, false, false, false],
            [false; POSITIONS],
        ));
        saver.advance(&link);
        assert_eq!(saver.state(), SaveState::Complete);
        assert_eq!(
            link.names(),
            [
                "FLTMODE1",
                "FLTMODE2",
                "FLTMODE3",
                "FLTMODE4",
                "FLTMODE5",
                "FLTMODE6",
                "SIMPLE",
                "SUPER_SIMPLE"
            ]
        );
        assert_eq!(
            link.written.borrow().get(6),
            Some(&("SIMPLE".to_owned(), 2.0))
        );
    }

    /// A write that times out ends the save there, as the C#'s TimeoutException does.
    #[test]
    fn a_timed_out_write_stops_the_save() {
        let link = Answering::new(&[("FLTMODE2", Progress::Finished(RequestOutcome::TimedOut))]);
        let mut saver = Saver::default();
        saver.start(save_set(
            Firmware::ArduCopter2,
            |_| true,
            &[Some(7); POSITIONS],
            [false; POSITIONS],
            [false; POSITIONS],
        ));
        saver.advance(&link);
        assert_eq!(saver.state(), SaveState::Failed);
        assert_eq!(link.names(), ["FLTMODE1", "FLTMODE2"]);
    }

    /// An empty combo ends the save after what came before it, as the C#'s null reference does.
    #[test]
    fn an_empty_combo_stops_the_save() {
        let link = Answering::new(&[]);
        let mut saver = Saver::default();
        saver.start(save_set(
            Firmware::ArduCopter2,
            |_| true,
            &[Some(7), Some(5), None, Some(3), Some(5), Some(0)],
            [false; POSITIONS],
            [false; POSITIONS],
        ));
        saver.advance(&link);
        assert_eq!(saver.state(), SaveState::Failed);
        assert_eq!(link.names(), ["FLTMODE1", "FLTMODE2"]);
    }

    /// While a write is unanswered the save waits, and nothing more is sent.
    #[test]
    fn a_save_waits_for_each_answer() {
        let link = Answering::new(&[("FLTMODE1", Progress::Waiting)]);
        let mut saver = Saver::default();
        saver.start(save_set(
            Firmware::ArduCopter2,
            |_| true,
            &[Some(7); POSITIONS],
            [false; POSITIONS],
            [false; POSITIONS],
        ));
        saver.advance(&link);
        saver.advance(&link);
        assert_eq!(saver.state(), SaveState::Saving);
        assert_eq!(link.names(), ["FLTMODE1"]);
    }

    fn view_with(parameters: &[(&str, f64)], vehicle_type: u8) -> TelemetryView {
        let mut view = TelemetryView::disconnected("test");
        view.parameters = parameters
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect();
        let mut state = mp_vehicle::VehicleState::default();
        state.autopilot = 3;
        state.vehicle_type = vehicle_type;
        state.custom_mode = 9;
        view.state = Some(std::sync::Arc::new(state));
        view
    }

    /// `Activate` on SITL's copter: the six modes, the check boxes from their bitmasks, and the
    /// save built from what it read writes the same values back.
    #[test]
    fn activating_reads_the_vehicles_modes_and_boxes() {
        let view = view_with(
            &[
                ("FLTMODE1", 7.0),
                ("FLTMODE2", 9.0),
                ("FLTMODE3", 6.0),
                ("FLTMODE4", 3.0),
                ("FLTMODE5", 5.0),
                ("FLTMODE6", 0.0),
                ("FLTMODE_CH", 5.0),
                ("SIMPLE", 2.0),
                ("SUPER_SIMPLE", 33.0),
            ],
            2,
        );
        let mut modes = FlightModes::default();
        modes.activate(&view, None);
        assert!(modes.is_active());
        assert_eq!(modes.firmware, Firmware::ArduCopter2);
        assert_eq!(
            modes.selected,
            [Some(7), Some(9), Some(6), Some(3), Some(5), Some(0)]
        );
        assert_eq!(modes.simple, [false, true, false, false, false, false]);
        assert_eq!(modes.super_simple, [true, false, false, false, false, true]);
        assert_eq!(modes.name_of(Some(9)), Some("Land"));
        assert_eq!(current_mode(&modes, &view), "Land");
        // Circle takes no Simple; Land does.
        assert!(!modes.simple_enabled(0));
        assert!(modes.simple_enabled(1));
        let (label, pwm, lit) = switch_reading(&view);
        assert_eq!((label.as_str(), pwm, lit), ("5: 0", 0.0, 0));

        modes.deactivate();
        assert!(!modes.is_active());
    }

    /// A missing mode parameter leaves it and every combo after it empty, and the boxes unread;
    /// a value not in the list leaves its own combo empty.
    #[test]
    fn activating_stops_where_the_csharp_throws() {
        let view = view_with(
            &[
                ("FLTMODE1", 8.0),
                ("FLTMODE2", 9.0),
                ("FLTMODE4", 3.0),
                ("SIMPLE", 63.0),
            ],
            2,
        );
        let mut modes = FlightModes::default();
        modes.activate(&view, None);
        assert_eq!(modes.selected, [None, Some(9), None, None, None, None]);
        assert_eq!(modes.simple, [false; POSITIONS]);
    }

    /// A rover reads MODE1-6 and MODE_CH, and hides the Simple boxes.
    #[test]
    fn a_rover_reads_mode_and_mode_ch() {
        let view = view_with(
            &[
                ("MODE1", 0.0),
                ("MODE2", 4.0),
                ("MODE3", 10.0),
                ("MODE4", 11.0),
                ("MODE5", 15.0),
                ("MODE6", 5.0),
                ("MODE_CH", 8.0),
            ],
            10,
        );
        let mut modes = FlightModes::default();
        modes.activate(&view, None);
        assert_eq!(modes.firmware, Firmware::ArduRover);
        assert_eq!(
            modes.selected,
            [Some(0), Some(4), Some(10), Some(11), Some(15), Some(5)]
        );
        assert!(!simple_shown(modes.firmware, false));
        let written = save_set(
            modes.firmware,
            |name| parameter(&view, name).is_some(),
            &modes.selected,
            modes.simple,
            modes.super_simple,
        );
        assert_eq!(written.get(1), Some(&("MODE2", Some(4.0))));
        assert_eq!(switch_reading(&view).0, "8: 0");
    }
}
