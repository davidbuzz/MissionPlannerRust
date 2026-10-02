//! Bridges the link engine to the UI.
//!
//! The UI must never touch the link's internals: it reads an immutable snapshot per frame and
//! nothing else. That is the whole point of the snapshot bus in `mp-vehicle` - a render pass
//! cannot block on I/O, and cannot observe a half-updated vehicle.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use mp_link::FtpError;
use mp_link::mavftp::{FtpOutcome, FtpRequest, Progress};
use mp_link::messages::LogMessage;
use mp_link::mission_transfer::TransferState;
use mp_link::requests::{Request, RequestOutcome};
use mp_link::{Link, LinkConfig, RequestId, commands};
use mp_mavlink_dialects::all::MavMessage;
use mp_mission::MissionItem;
use mp_units::LatLon;
use mp_vehicle::{DateTime, FenceItem, StreamRates, VehicleFamily, VehicleId, VehicleState};

use crate::fly::{error_box, strings};

/// The parameter holding the bitmask of checks performed before arming, before ArduPilot 4.7.
///
/// A bitmask of checks to *perform*; zero is none of them, and the firmware default is 1 meaning
/// all.
pub const ARMING_CHECK: &str = "ARMING_CHECK";

/// The same setting from ArduPilot 4.7 onwards, with the sense inverted.
///
/// A bitmask of checks to *skip*; the default is 0, meaning skip nothing. ArduPilot's own
/// conversion code maps the old `ARMING_CHECK == 0` to this being -1, and its parameter
/// documentation says -1 skips "all non-mandatory current and future checks".
pub const ARMING_SKIPCHK: &str = "ARMING_SKIPCHK";

/// `ARMING_SKIPCHK` value that skips every non-mandatory check.
pub const SKIP_ALL_CHECKS: f32 = -1.0;
/// The name a flight recording is given, in local time.
///
/// Mission Planner names tlogs `DateTime.Now.ToString("yyyy-MM-dd HH-mm-ss")`, which is local, and
/// that is the right choice for a filename a pilot matches to a flight they remember by the clock
/// on the wall. Recorded UTC once by accident and the result was a file called `07-20-54` for a
/// flight at twenty past five in the afternoon - unreadable at exactly the moment it matters,
/// which is finding the right log after a crash.
///
/// The frames inside the file carry UTC, so nothing about the data depends on this.
fn flight_stamp() -> String {
    chrono::Local::now().format("%Y-%m-%d %H-%M-%S").to_string()
}

/// How many log lines the flight screen shows.
///
/// The pane scrolls, so this is how much history is reachable rather than how much fits. The link
/// keeps more than this; what is not shown is still in the telemetry log, and the pane says so.
const MESSAGE_LINES: usize = 200;

/// Everything one frame of UI needs to know.
#[derive(Debug, Clone)]
pub struct TelemetryView {
    /// The link target as typed by the user.
    pub target: String,
    /// Whether the link thread is alive.
    pub connected: bool,
    /// The vehicle being displayed, if one has been heard from.
    pub vehicle: Option<VehicleId>,
    /// The latest state snapshot for that vehicle.
    pub state: Option<Arc<VehicleState>>,
    /// Frames received over the link.
    pub frames: u64,
    /// Frames rejected by the checksum.
    pub crc_errors: u64,
    /// Vehicles seen on this link, including gimbals, companions and other ground stations.
    pub vehicle_count: usize,
    /// The mission read back from the vehicle, once a download has completed.
    pub mission: Vec<MissionItem>,
    /// Whether `mission` is a finished download rather than one still arriving. The transfer's
    /// item list grows as items come in, and a plan that adopts it on the first item keeps a
    /// mission of one. `tests/gui/fly-setwp.gui` found it: a five-item mission read back as one
    /// row once the download published its progress.
    pub mission_complete: bool,
    /// The vehicle's mission as the link's traffic has shown it - `MAV.wps`: a download's items
    /// as they arrive, an upload's as the vehicle takes them, a script's `setWP`s - which the
    /// flight screen draws when no plan is being edited, and counts for its Set WP list.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:313; GCSViews/FlightData.cs:2571-2576, 3810-3843`
    pub wps: Vec<MissionItem>,
    /// The vehicle's rally points likewise - `MAV.rallypoints` - which the flight screen draws
    /// when the plan has none of its own.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:315; GCSViews/FlightData.cs:3898-3905`
    pub rally_points: Vec<MissionItem>,
    /// Recent `STATUSTEXT` and `COMMAND_ACK` lines, newest last.
    pub messages: Vec<LogMessage>,
    /// How many messages the link had to discard to stay bounded.
    pub messages_dropped: u64,
    /// What the mission transfer is doing, if one has been started.
    pub transfer: Option<TransferStatus>,
    /// The vehicle's parameters, by name.
    ///
    /// Shared rather than owned: the same list is handed to every frame until a parameter
    /// arrives ([`Telemetry::view`]), so taking a view does not copy fourteen hundred names.
    pub parameters: Arc<[(String, f64)]>,
    /// Each parameter's `MAV_PARAM_TYPE`, as the vehicle declared it: `TypeAP`. Shared as
    /// `parameters` is, and made with it.
    pub parameter_types: Arc<std::collections::BTreeMap<String, mp_params::ParamType>>,
    /// How many the vehicle says it has, once it has said.
    pub parameters_expected: u16,
    /// Whether a parameter download has run to its end since the link opened: `doConnect`'s
    /// `getParamList` done, after which `MainV2` shows the SETUP or CONFIG screen again. Latched:
    /// neither a count the vehicle raises later (a feature switched on adds its parameters) nor
    /// a Refresh Params pressed on a page unsets it, as neither sends the C# back through
    /// `doConnect` - the page stays.
    pub parameters_fetched: bool,
    /// Where the parameter fetch is, in words - "MAVFTP 45%", "stream 400 of 1408", "1408 over
    /// MAVFTP" - or "none" before one has been started.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1813-1936`
    pub parameters_fetch: String,
    /// The defaults `@PARAM/param.pck?withdefaults=1` carried, by name; empty when the parameters
    /// came over the stream, which has none to give. Shared between frames, as `parameters` is.
    /// `// C#: ExtLibs/Mavlink/MAVLinkParam.cs:36 (default_value)`
    pub parameters_defaults: Arc<std::collections::BTreeMap<String, f64>>,
}

/// A mission transfer, as the UI needs to describe it.
///
/// A percentage on its own is not enough: an upload that reaches 100% and then fails looks
/// identical to one that succeeded, and the difference is whether the aircraft has the mission.
#[derive(Debug, Clone)]
pub struct TransferStatus {
    /// What is happening, in words.
    pub label: String,
    /// How far through, from zero to one.
    pub fraction: f32,
    /// Whether the transfer has stopped, either way.
    pub finished: bool,
    /// Whether it stopped because it failed.
    pub failed: bool,
}

impl TelemetryView {
    /// The view shown before any link exists.
    #[must_use]
    pub fn disconnected(target: impl Into<String>) -> Self {
        Self {
            target: target.into(),
            connected: false,
            vehicle: None,
            state: None,
            frames: 0,
            crc_errors: 0,
            vehicle_count: 0,
            mission: Vec::new(),
            mission_complete: false,
            wps: Vec::new(),
            rally_points: Vec::new(),
            messages: Vec::new(),
            messages_dropped: 0,
            transfer: None,
            parameters: Arc::default(),
            parameter_types: Arc::default(),
            parameters_expected: 0,
            parameters_fetched: false,
            parameters_fetch: "none".to_owned(),
            parameters_defaults: Arc::default(),
        }
    }

    /// A parameter's type as the vehicle declared it, `TypeAP`; `None` for a name the table has
    /// not got.
    #[must_use]
    pub fn parameter_type(&self, name: &str) -> Option<mp_params::ParamType> {
        self.parameter_types.get(name).copied()
    }
}

/// The parameter list the last view carried, kept for the next: see [`Telemetry::parameters_of`].
#[derive(Debug)]
struct SharedParameters {
    /// The table's [`mp_params::ParamTable::generation`] when the list was made from it.
    generation: u64,
    list: Arc<[(String, f64)]>,
    types: Arc<std::collections::BTreeMap<String, mp_params::ParamType>>,
    expected: u16,
}

/// What a screen says when a request it made ends, as the C# says it around the call it blocks on.
///
/// Mission Planner blocks the screen inside `setParam`, `doCommand` and `setWPCurrent` while they
/// send and send again, and wraps each call in its own `try`/`catch` and `if (!ok)` with a message
/// box in one or both. Here the link runs the retries and the screen carries on, so the message
/// box's text travels with the request and is said on the status line when the request ends.
/// `None` is the C# saying nothing.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Report {
    /// Every retry went unanswered: the C#'s `catch` around the `TimeoutException`.
    pub timed_out: Option<String>,
    /// The vehicle said no - a `MAV_RESULT` other than accepted and in progress - or has no
    /// parameter of that name: the C#'s `false` branch.
    pub refused: Option<String>,
    /// It worked, where the C# says so on screen.
    pub accepted: Option<String>,
    /// Sent when the vehicle says no: `setDigicamControl` falls back to `DIGICAM_CONTROL` when
    /// its command is refused.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4557-4569`
    pub fallback: Option<MavMessage>,
}

impl Report {
    /// Says `text` if every retry goes unanswered, and nothing otherwise.
    #[must_use]
    pub fn on_timeout(text: impl Into<String>) -> Self {
        Self {
            timed_out: Some(text.into()),
            ..Self::default()
        }
    }

    /// Says `text` if the vehicle refuses or never answers, and nothing otherwise.
    #[must_use]
    pub fn on_failure(text: impl Into<String>) -> Self {
        let text = text.into();
        Self {
            timed_out: Some(text.clone()),
            refused: Some(text),
            ..Self::default()
        }
    }

    /// What to say for an outcome, if anything.
    #[must_use]
    pub fn text(&self, outcome: RequestOutcome) -> Option<&str> {
        match outcome {
            RequestOutcome::TimedOut => self.timed_out.as_deref(),
            RequestOutcome::Rejected(_) | RequestOutcome::UnknownParameter => {
                self.refused.as_deref()
            }
            // `setParam` returns true for a value already held (C#: MAVLinkInterface.cs:1647-1651),
            // and `doCommand` true for the commands it does not wait on.
            RequestOutcome::Accepted { .. } | RequestOutcome::Sent | RequestOutcome::Unchanged => {
                self.accepted.as_deref()
            }
        }
    }
}

/// A request a screen handed to the link, and what to say when it ends.
#[derive(Debug, Clone)]
struct Awaited {
    id: RequestId,
    /// When it was handed over, for [`Telemetry::lookup`].
    made: Instant,
    report: Report,
}

/// How long a request may be missing from the link before it is taken as forgotten.
///
/// The link moves what it picks up from its queue into its table in two steps, with neither lock
/// held in between (mp-link's `run_link`, `picked_up`), and in that moment `Link::request` finds
/// the request in neither. The moment is microseconds unless the link thread is descheduled in
/// it; a request missing for longer than this has been let go.
pub const PICKUP_GRACE: Duration = Duration::from_millis(500);

/// What the link says of a request a screen made.
#[derive(Debug, Clone)]
pub enum Lookup {
    /// Where it is. Boxed: a request carries the message it sends, which dwarfs the other two.
    Found(Box<Request>),
    /// Neither queued nor held, but made a moment ago: the link is picking it up.
    PickingUp,
    /// Forgotten, or the link is gone: it will never be heard of again.
    Gone,
}

/// What `testMotor` says: its `false` branch's box, and its `catch`'s with the motor's number.
/// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:318-327`
fn motor_report(motor: i32) -> Report {
    Report {
        refused: Some("Command was denied by the autopilot".to_owned()),
        timed_out: Some(error_box(format!(
            "{} Motor: {motor}",
            strings::ERROR_COMMUNICATING
        ))),
        ..Report::default()
    }
}

/// Owns the link and produces views of it.
#[derive(Debug)]
pub struct Telemetry {
    link: Option<Link>,
    target: String,
    error: Option<String>,
    /// Where this session is being recorded.
    recording: Option<std::path::PathBuf>,
    /// The vehicle the operator chose, if they chose one.
    ///
    /// `None` means whichever the link considers primary, which is the autopilot on the first
    /// system heard from. A choice is remembered even while that vehicle is quiet, because a
    /// vehicle going briefly silent is not a reason to start showing a different one.
    selected: Option<VehicleId>,
    /// Vehicles already asked for their banner, so each is asked once.
    banner_requested: std::collections::BTreeSet<VehicleId>,
    /// The firmware banner, once a vehicle has said it: `ArduCopter V4.5.7 (1c0c8d9c)`.
    banner: Option<String>,
    /// Requests the screens made and have not yet heard the end of, oldest first.
    awaited: Vec<Awaited>,
    /// What [`Telemetry::parameters_of`] built last, for the next frame to reuse.
    parameters: Mutex<Option<SharedParameters>>,
    /// Whether a parameter download has run to its end since this link opened: latched, as
    /// `doConnect`'s `getParamList` is done once and a Refresh Params afterwards does not put
    /// the C# back through `doConnect` - the screen it is pressed on stays.
    parameters_fetched: std::sync::atomic::AtomicBool,
    /// The Planner page's rates as last handed to [`Telemetry::hand_over_rates`].
    rates_handed: Option<StreamRates>,
    /// `cs.messages.Clear()`: the sequence number of the first message still shown - 0 until a
    /// script clears, then one past the last message there was. Sequence numbers start at 0, so
    /// this is a bound, not a last-cleared number: the first version hid message 0 for good.
    messages_shown_from: std::sync::atomic::AtomicU64,
    /// A plain reboot's look at a serial port afterwards, and the reopen it may lead to; see
    /// [`Telemetry::reopen_after_reboot`]. Behind a lock because [`Telemetry::reboot`] is `&self`.
    reopen: Mutex<Option<Reopen>>,
}

/// `doReboot(false, true)` on a serial port: "Direct USB will disconnect after a reboot, wait and
/// see if we should re-connect".
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2573-2583`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Reopen {
    /// `Thread.Sleep(500)` running out at this instant, then `if (!BaseStream.IsOpen)`.
    Check(Instant),
    /// The port was gone: `Open(true)`, whose `OpenBg` lets a serial port settle for a second
    /// ("allow settings to settle - previous dtr") before it opens it at this instant.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:668-700, 711-723, 747`
    Open(Instant),
}

/// How long `doReboot` sleeps after a plain reboot on a serial port before it looks at the port.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2578`
pub const REBOOT_REOPEN_WAIT: Duration = Duration::from_millis(500);

/// `OpenBg`'s "SerialPort Sleep 1" before it opens a serial port.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:717-722`
pub const SERIAL_SETTLE: Duration = Duration::from_secs(1);

/// `Strings.ConnectingMavlink`: the title of `Open`'s progress box, on the status line here.
/// `// C#: ExtLibs/Strings/Strings.resx:303-305; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:676`
pub const CONNECTING_MAVLINK: &str = "Connecting Mavlink";

/// `Strings.ConnectFailed`: `OpenBg`'s error for its progress box, on the status line here (the
/// owner's ruling: a failure goes on the status line, never in a box).
/// `// C#: ExtLibs/Strings/Strings.resx:300-302; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:960-961`
pub const CONNECT_FAILED: &str = "Connect Failed";

/// What [`Telemetry::reopen_after_reboot`] did this frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Reopened {
    /// The port was gone at the look: `Open(true)` is under way, the port to be opened after
    /// `OpenBg`'s settle.
    Connecting,
    /// The port opened again, as `Open(true)`: the parameters are to be fetched afresh.
    Opened,
    /// It would not open: what goes on the status line.
    Failed(String),
}

impl Telemetry {
    /// Where a flight is recorded, and what the file is called.
    ///
    /// Named for the time the flight started, as Mission Planner names them, so the directory
    /// sorts chronologically and a file can be matched to a flight without opening it. An index is
    /// appended if that name is taken, because `TlogWriter::create` refuses to overwrite - two
    /// connections in the same second must not have one silently lose its recording.
    #[must_use]
    pub fn recording_path() -> Option<std::path::PathBuf> {
        if std::env::var_os("MP_NO_RECORD").is_some() {
            return None;
        }
        let directory = std::env::var_os("MP_LOG_DIR")
            .map_or_else(Self::log_directory, std::path::PathBuf::from);
        if std::fs::create_dir_all(&directory).is_err() {
            return None;
        }

        Some(Self::recording_path_in(&directory, &flight_stamp()))?
    }

    /// Where flights are recorded, and the HUD's AVI goes: `MP_LOG_DIR` when set, else
    /// [`Self::log_directory`]. `Settings.Instance.LogDir`, as `recordHudToAVIToolStripMenuItem`
    /// reads it. `// C#: GCSViews/FlightData.cs:4663-4665`
    #[must_use]
    pub(crate) fn recording_directory() -> std::path::PathBuf {
        std::env::var_os("MP_LOG_DIR").map_or_else(Self::log_directory, std::path::PathBuf::from)
    }

    /// Where flights are recorded when nothing says otherwise.
    ///
    /// Mission Planner's `Settings.GetDefaultLogDir`: the user data directory plus `logs` - this
    /// application's own user data directory, `MissionPlannerRust`, not the C#'s (PLAN.md
    /// section 12, D11), so neither program's recordings are in the other's way. A pilot who
    /// pointed Mission Planner's `logdirectory` at a folder of their own still shares that one,
    /// through the imported `config.xml`.
    ///
    /// The user data directory is not where a Linux user would guess, and guessing it is how this
    /// function once recorded into a directory the C# application never reads. The rule and how
    /// it was measured are in `crates/mp-settings/src/lib.rs`; this only asks it.
    ///
    /// Not the working directory. A ground station launched from a desktop icon inherits whatever
    /// directory the launcher happened to be in - often `/` or the user's home - and writing
    /// flight recordings there scatters them somewhere nobody thinks to look.
    /// `// C#: ExtLibs/Utilities/Settings.cs:146-158`
    fn log_directory() -> std::path::PathBuf {
        let chosen = mp_settings::Config::default_path()
            .and_then(|path| mp_settings::Config::load(&path).ok());
        Self::log_directory_from(chosen.as_ref())
    }

    /// The recording directory given the C# application's settings, if it has any.
    ///
    /// `Settings.LogDir` is the `logdirectory` key when the operator set one, else the default
    /// under the user data directory. Reading the key means a pilot who pointed Mission Planner
    /// at a drive of their own finds this application's recordings on the same drive.
    /// `// C#: ExtLibs/Utilities/Settings.cs:127-140`
    fn log_directory_from(config: Option<&mp_settings::Config>) -> std::path::PathBuf {
        config
            .and_then(mp_settings::Config::log_directory)
            .or_else(mp_settings::default_log_directory)
            .unwrap_or_else(|| {
                // No home directory at all is a strange environment, not a reason to lose the
                // recording; the temp directory keeps it for the length of the session.
                std::env::temp_dir().join("mission-planner-rust-logs")
            })
    }

    /// The name to record under in a given directory, avoiding one that is taken.
    ///
    /// Separate from `recording_path` so it can be tested: the directory and the stamp are the
    /// only inputs, where `recording_path` reads the environment, and setting an environment
    /// variable is `unsafe` in this edition and denied by the workspace.
    fn recording_path_in(directory: &std::path::Path, stamp: &str) -> Option<std::path::PathBuf> {
        let first = directory.join(format!("{stamp}.tlog"));
        if !first.exists() {
            return Some(first);
        }
        // Bounded, because an unbounded loop looking for a free name is a hang waiting for a full
        // disk. Ninety-nine flights in one second is not a case worth serving.
        (1..100)
            .map(|index| directory.join(format!("{stamp}-{index}.tlog")))
            .find(|candidate| !candidate.exists())
    }

    /// Opens a link. A failure here is shown in the UI rather than killing the process: a ground
    /// station that exits because a USB cable was not plugged in yet is useless in the field.
    #[must_use]
    pub fn connect(url: &str) -> Self {
        // Every flight is recorded, without being asked. An operator who wanted a recording and
        // did not press a button has lost the flight; one who did not want it has a file.
        let recording = Self::recording_path();
        let config = LinkConfig {
            record_path: recording.clone(),
            ..LinkConfig::default()
        };
        match Link::connect(url, config) {
            Ok(link) => Self {
                recording,
                ..Self::over(link, url)
            },
            Err(err) => Self {
                error: Some(err.to_string()),
                target: url.to_owned(),
                ..Self::idle()
            },
        }
    }

    /// A telemetry-less instance, for launching the UI with no link.
    #[must_use]
    pub fn idle() -> Self {
        Self {
            link: None,
            target: String::new(),
            error: None,
            selected: None,
            banner_requested: std::collections::BTreeSet::new(),
            banner: None,
            recording: None,
            awaited: Vec::new(),
            parameters: Mutex::new(None),
            parameters_fetched: std::sync::atomic::AtomicBool::new(false),
            rates_handed: None,
            reopen: Mutex::new(None),
            messages_shown_from: std::sync::atomic::AtomicU64::new(0),
        }
    }

    /// Screens over a link that is already open: [`Telemetry::connect`]'s, and a test's over an
    /// in-memory transport.
    #[must_use]
    pub fn over(link: Link, url: &str) -> Self {
        Self {
            link: Some(link),
            target: url.to_owned(),
            ..Self::idle()
        }
    }

    /// Where this session is being recorded, if it is.
    ///
    /// Shown on screen, because the link reports a failed recording to stderr and carries on -
    /// which in an application launched from a desktop icon means a failed recording and a
    /// successful one look identical.
    #[must_use]
    pub fn recording(&self) -> Option<&std::path::Path> {
        self.recording.as_deref()
    }

    /// Why the link could not be opened, if it could not.
    #[must_use]
    pub fn error(&self) -> Option<&str> {
        self.error.as_deref()
    }

    /// Takes a consistent view for this frame.
    #[must_use]
    pub fn view(&self) -> TelemetryView {
        let Some(link) = &self.link else {
            return TelemetryView::disconnected(self.target.clone());
        };
        let stats = link.stats();
        let vehicles = link.vehicles();
        // The chosen vehicle drives every screen, not just the commands. Showing one vehicle's
        // telemetry while commands went to another would be the worst of both.
        let primary = self
            .selected
            .filter(|chosen| vehicles.contains(chosen))
            .and_then(|id| link.vehicle(id).map(|handle| (id, handle)))
            .or_else(|| link.primary_vehicle());

        // Fetch the mission the link holds, if a download has finished. The UI never triggers
        // one itself: a ground station that silently pulls a mission whenever it connects makes
        // it impossible to tell whether what is on screen came from the vehicle or the operator.
        let parameters_fetch = primary
            .as_ref()
            .map_or_else(|| "none".to_owned(), |(id, _)| Self::fetch_words(link, *id));
        let parameters_defaults = primary
            .as_ref()
            .and_then(|(id, _)| link.param_fetch(*id))
            .map(|fetch| fetch.defaults)
            .unwrap_or_default();
        let (parameters, parameter_types, parameters_expected) = primary.as_ref().map_or_else(
            || (Arc::default(), Arc::default(), 0),
            |(id, _)| self.parameters_of(link, *id),
        );
        let fetch_complete = primary.as_ref().is_some_and(|(id, _)| {
            link.param_fetch(*id).is_some_and(|fetch| {
                matches!(
                    fetch.state,
                    mp_link::param_fetch::ParamFetchState::Complete { .. }
                )
            })
        });
        if fetch_complete {
            self.parameters_fetched
                .store(true, std::sync::atomic::Ordering::Relaxed);
        }
        let parameters_fetched = self
            .parameters_fetched
            .load(std::sync::atomic::Ordering::Relaxed);
        let transfer = primary
            .as_ref()
            .and_then(|(id, _)| link.mission_transfer(*id));
        let mission = transfer
            .as_ref()
            .map(|transfer| transfer.items().to_vec())
            .unwrap_or_default();
        let mission_complete = transfer
            .as_ref()
            .is_some_and(|transfer| matches!(transfer.state(), TransferState::Complete));
        let (wps, rally_points) = primary.as_ref().map_or_else(
            || (Vec::new(), Vec::new()),
            |(id, _)| (link.wps(*id), link.rally_points(*id)),
        );
        let transfer = transfer.as_ref().map(|transfer| {
            let label = match transfer.state() {
                TransferState::Idle => "idle".to_owned(),
                TransferState::AwaitingCount => "asking the vehicle for its mission".to_owned(),
                TransferState::Downloading { count, next } => {
                    format!("reading item {next} of {count}")
                }
                TransferState::Uploading { last_requested } => {
                    let done = last_requested.map_or(0, |seq| u32::from(seq) + 1);
                    format!("writing item {done} of {}", transfer.items().len())
                }
                // `saveWPsFast`'s "Setting WP a": the item the burst reached.
                TransferState::UploadingFast { next, .. } => {
                    format!("writing item {next} of {}", transfer.items().len())
                }
                TransferState::Complete => "mission transferred".to_owned(),
                TransferState::Failed(why) => format!("transfer failed: {why}"),
            };
            TransferStatus {
                label,
                fraction: transfer.progress(),
                finished: transfer.is_finished(),
                failed: matches!(transfer.state(), TransferState::Failed(_)),
            }
        });

        TelemetryView {
            target: link.description(),
            connected: link.is_running(),
            vehicle: primary.as_ref().map(|(id, _)| *id),
            state: primary.map(|(_, handle)| handle.load()),
            frames: link.frames_received(),
            crc_errors: stats.decode.crc_errors,
            vehicle_count: vehicles.len(),
            mission,
            mission_complete,
            wps,
            rally_points,
            messages: {
                let from = self
                    .messages_shown_from
                    .load(std::sync::atomic::Ordering::Relaxed);
                link.recent_messages(MESSAGE_LINES)
                    .into_iter()
                    .filter(|message| message.seq >= from)
                    .collect()
            },
            messages_dropped: link.messages_dropped(),
            transfer,
            parameters,
            parameter_types,
            parameters_expected,
            parameters_fetched,
            parameters_fetch,
            parameters_defaults,
        }
    }

    /// The fetch's state in words, for the status line and the facts.
    fn fetch_words(link: &Link, id: VehicleId) -> String {
        use mp_link::param_fetch::{FetchVia, ParamFetchState};
        let Some(fetch) = link.param_fetch(id) else {
            return "none".to_owned();
        };
        match fetch.state {
            ParamFetchState::Ftp => {
                let percent = link
                    .ftp_progress(id)
                    .map_or(-1, |(_, progress)| progress.percent);
                if percent < 0 {
                    "MAVFTP".to_owned()
                } else {
                    format!("MAVFTP {percent}%")
                }
            }
            ParamFetchState::Stream { .. } => {
                let download = link.param_download(id);
                let received = download
                    .as_ref()
                    .map_or(0, mp_link::param_download::ParamDownload::received);
                let expected = download.and_then(|d| d.expected()).unwrap_or(0);
                format!("stream {received} of {expected}")
            }
            ParamFetchState::Complete { via, count } => match via {
                FetchVia::MavFtp => format!("{count} over MAVFTP"),
                FetchVia::Stream => format!("{count} over the stream"),
            },
            ParamFetchState::Cancelled => "cancelled".to_owned(),
        }
    }

    /// A vehicle's parameters for a view, and how many it says it has.
    ///
    /// A view is taken every frame and the table holds some fourteen hundred parameters, which
    /// change only when one arrives. Copying them out each frame cost a millisecond a frame in a
    /// debug build to find them unchanged, so the link is asked only which generation its table
    /// is at, and the list is made again only when that has moved; otherwise the last one is
    /// handed out again. Generations are numbered across every table in the process, so a list
    /// made for one vehicle is never handed out for another.
    #[allow(clippy::type_complexity)] // the list, its types and the count, shared as one
    fn parameters_of(
        &self,
        link: &Link,
        id: VehicleId,
    ) -> (
        Arc<[(String, f64)]>,
        Arc<std::collections::BTreeMap<String, mp_params::ParamType>>,
        u16,
    ) {
        let Some(generation) = link.params_generation(id) else {
            return (Arc::default(), Arc::default(), 0);
        };
        let mut shared = self
            .parameters
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        if let Some(held) = shared.as_ref()
            && held.generation == generation
        {
            return (
                Arc::clone(&held.list),
                Arc::clone(&held.types),
                held.expected,
            );
        }
        let Some(table) = link.params(id) else {
            return (Arc::default(), Arc::default(), 0);
        };
        let list: Arc<[(String, f64)]> = table
            .iter()
            .map(|(name, value)| (name.clone(), value.as_f64()))
            .collect();
        let types: Arc<std::collections::BTreeMap<String, mp_params::ParamType>> = Arc::new(
            table
                .iter()
                .map(|(name, value)| (name.clone(), value.param_type()))
                .collect(),
        );
        let expected = table.expected().unwrap_or(0);
        // The table's own generation, which a parameter arriving since the question may have
        // moved past the one asked about; the list is of this table, so it is kept under it.
        *shared = Some(SharedParameters {
            generation: table.generation(),
            list: Arc::clone(&list),
            types: Arc::clone(&types),
            expected,
        });
        (list, types, expected)
    }

    /// Asks the vehicle for its mission. Explicit, never automatic.
    pub fn request_mission(&self) {
        if let Some(link) = &self.link
            && let Some((id, _)) = link.primary_vehicle()
        {
            link.download_mission(id);
        }
    }

    /// Sends the given mission to the vehicle, replacing what is on board.
    pub fn upload_mission(&self, items: Vec<MissionItem>) {
        if let Some(link) = &self.link
            && let Some((id, _)) = link.primary_vehicle()
        {
            link.upload_mission(id, items);
        }
    }

    /// Sends a geofence, replacing whatever the vehicle holds.
    /// Write Fast's upload.
    pub fn upload_mission_fast(&self, items: Vec<MissionItem>) {
        if let Some(link) = &self.link
            && let Some((id, _)) = link.primary_vehicle()
        {
            link.upload_mission_fast(id, items);
        }
    }

    pub fn upload_fence(&self, items: Vec<MissionItem>) {
        if let Some(link) = &self.link
            && let Some((id, _)) = link.primary_vehicle()
        {
            link.upload_list(id, items, mp_mission::fence::MISSION_TYPE_FENCE);
        }
    }

    /// Asks the vehicle for its geofence.
    pub fn request_fence(&self) {
        if let Some(link) = &self.link
            && let Some((id, _)) = link.primary_vehicle()
        {
            link.download_list(id, mp_mission::fence::MISSION_TYPE_FENCE);
        }
    }

    /// The fence the vehicle reported, once a download has completed.
    #[must_use]
    pub fn fence_items(&self) -> Vec<MissionItem> {
        self.completed_list(mp_mission::fence::MISSION_TYPE_FENCE)
    }

    /// Sends rally points, replacing whatever the vehicle holds.
    pub fn upload_rally(&self, items: Vec<MissionItem>) {
        if let Some(link) = &self.link
            && let Some((id, _)) = link.primary_vehicle()
        {
            link.upload_list(id, items, mp_mission::fence::MISSION_TYPE_RALLY);
        }
    }

    /// Asks the vehicle for its rally points.
    pub fn request_rally(&self) {
        if let Some(link) = &self.link
            && let Some((id, _)) = link.primary_vehicle()
        {
            link.download_list(id, mp_mission::fence::MISSION_TYPE_RALLY);
        }
    }

    /// The rally points the vehicle reported, once a download has completed.
    #[must_use]
    pub fn rally_items(&self) -> Vec<MissionItem> {
        self.completed_list(mp_mission::fence::MISSION_TYPE_RALLY)
    }

    /// `MAV.rallypoints.Clear()`, Clear Rally Points' second clearing, beside its markers'.
    /// `// C#: GCSViews/FlightPlanner.cs:2108-2109`
    pub fn clear_rally_points(&self) {
        if let Some((link, id)) = self.target() {
            link.clear_rally_points(id);
        }
    }

    /// `mav_mission.download(..., MAV_MISSION_TYPE.RALLY)` from the vehicle being flown, which
    /// [`Telemetry::rally_list`] then reads. False with no vehicle to ask.
    /// `// C#: ExtLibs/ArduPilot/mav_mission.cs:14-63`
    pub fn download_rally(&self) -> bool {
        self.target()
            .is_some_and(|(link, id)| link.download_list(id, mp_mission::fence::MISSION_TYPE_RALLY))
    }

    /// The last rally-point transfer, once it has ended and nothing newer is waiting to start:
    /// its items, or why it failed. `None` while one runs or is still queued, and with no link.
    ///
    /// What `mav_mission.download(..., MAV_MISSION_TYPE.RALLY)` returns, or throws.
    /// `// C#: ExtLibs/ArduPilot/mav_mission.cs:14-63`
    #[must_use]
    pub fn rally_list(&self) -> Option<Result<Vec<MissionItem>, String>> {
        let (link, id) = self.target()?;
        let rally = mp_mission::fence::MISSION_TYPE_RALLY;
        if link.list_transfer_queued(id, rally) {
            return None;
        }
        let transfer = link.list_transfer(id, rally)?;
        match transfer.state() {
            TransferState::Complete => Some(Ok(transfer.items().to_vec())),
            TransferState::Failed(why) => Some(Err(why.to_string())),
            _ => None,
        }
    }

    /// `setRallyPoint` on the vehicle being flown: `RALLY_POINT`, read back with
    /// `RALLY_FETCH_POINT`, the link's retries between. `None` with no vehicle; the outcome is read
    /// with [`Telemetry::request`].
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6441-6476`
    pub fn set_rally_point(&self, point: mp_link::requests::RallyPointSet) -> Option<RequestId> {
        let (link, id) = self.target()?;
        Some(link.set_rally_point(id, point))
    }

    /// `mav_mission.download(..., MAV_MISSION_TYPE.FENCE)` from the vehicle being flown, which
    /// [`Telemetry::fence_list`] then reads. False with no vehicle to ask.
    /// `// C#: ExtLibs/ArduPilot/mav_mission.cs:14-63`
    pub fn download_fence(&self) -> bool {
        self.target()
            .is_some_and(|(link, id)| link.download_list(id, mp_mission::fence::MISSION_TYPE_FENCE))
    }

    /// The last geofence transfer, once it has ended and nothing newer is waiting to start: its
    /// items, or why it failed. `None` while one runs or is still queued, and with no link.
    /// `// C#: ExtLibs/ArduPilot/mav_mission.cs:14-63`
    #[must_use]
    pub fn fence_list(&self) -> Option<Result<Vec<MissionItem>, String>> {
        let (link, id) = self.target()?;
        let fence = mp_mission::fence::MISSION_TYPE_FENCE;
        if link.list_transfer_queued(id, fence) {
            return None;
        }
        let transfer = link.list_transfer(id, fence)?;
        match transfer.state() {
            TransferState::Complete => Some(Ok(transfer.items().to_vec())),
            TransferState::Failed(why) => Some(Err(why.to_string())),
            _ => None,
        }
    }

    /// `setFencePoint` on the vehicle being flown: `FENCE_POINT`, read back with
    /// `FENCE_FETCH_POINT`, the link's retries between. `None` with no vehicle; the outcome is read
    /// with [`Telemetry::request`].
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6415-6439`
    pub fn set_fence_point(&self, point: mp_link::requests::FencePointSet) -> Option<RequestId> {
        let (link, id) = self.target()?;
        Some(link.set_fence_point(id, point))
    }

    /// `getFencePoint` on the vehicle being flown: `FENCE_FETCH_POINT` for point `idx`. `None`
    /// with no vehicle; the point is the request's [`mp_link::requests::Request::fence_point`].
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5908-5967`
    pub fn get_fence_point(&self, idx: u8) -> Option<RequestId> {
        let (link, id) = self.target()?;
        Some(link.get_fence_point(id, idx))
    }

    /// `getWP(sysid, compid, index, type)` on `target`: that one item of that list read on its
    /// own, the link's retries between; no mission transfer is started or touched. `None`
    /// without a link; the item is the request's [`mp_link::requests::Request::wp`].
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3398-3565`
    pub fn get_wp(&self, target: VehicleId, index: u16, mission_type: u8) -> Option<RequestId> {
        Some(self.link.as_ref()?.get_wp(target, index, mission_type))
    }

    /// `setWPTotal(sysid, compid, total, type)` on `target`: `MISSION_COUNT` until the vehicle
    /// asks for the first item, the link's retries between. `None` without a link; the outcome
    /// is read with [`Telemetry::request`].
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3753-3882`
    pub fn set_wp_total(
        &self,
        target: VehicleId,
        total: u16,
        mission_type: u8,
    ) -> Option<RequestId> {
        Some(
            self.link
                .as_ref()?
                .set_wp_total(target, total, mission_type),
        )
    }

    /// `setParam(sysid, compid, name, value, force)` on `target`, for a caller that reads the
    /// outcome itself with [`Telemetry::request`]. `None` without a link.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1628-1770`
    pub fn write_parameter_on(
        &self,
        target: VehicleId,
        name: &str,
        value: f64,
        force: bool,
    ) -> Option<RequestId> {
        Some(self.link.as_ref()?.set_param(target, name, value, force))
    }

    /// `BaseStream.IsOpen`: whether there is a link and its thread still runs - false once the
    /// port has gone. Cheap: no view is taken.
    #[must_use]
    pub fn is_open(&self) -> bool {
        self.link.as_ref().is_some_and(Link::is_running)
    }

    /// `MAVlist[sysid, compid].cs`: the state of the vehicle `id`, shown or not, if it has been
    /// heard on the link.
    #[must_use]
    pub fn vehicle_state(&self, id: VehicleId) -> Option<Arc<VehicleState>> {
        self.link.as_ref()?.vehicle(id).map(|handle| handle.load())
    }

    /// `MAVlist[sysid, compid].param[name].Value`: a parameter of the vehicle `id` as the link
    /// holds it, shown or not.
    #[must_use]
    pub fn parameter_of(&self, id: VehicleId, name: &str) -> Option<f64> {
        self.link
            .as_ref()?
            .params(id)?
            .get(name)
            .map(mp_params::ParamValue::as_f64)
    }

    /// A message as a builder made it - addressed within itself - onto the link, as the C#'s
    /// `generatePacket` puts one. False without a link.
    pub fn send(&self, message: &MavMessage) -> bool {
        self.link.as_ref().is_some_and(|link| link.send(message))
    }

    /// The items of a finished transfer of one list, or nothing if it has not finished.
    ///
    /// Only when complete: a partial list read mid-transfer would be adopted as if it were the
    /// whole thing, and a fence missing its last side is a fence that does not enclose anything.
    fn completed_list(&self, mission_type: u8) -> Vec<MissionItem> {
        let Some(link) = &self.link else {
            return Vec::new();
        };
        let Some((id, _)) = link.primary_vehicle() else {
            return Vec::new();
        };
        link.list_transfer(id, mission_type)
            .filter(|transfer| matches!(transfer.state(), TransferState::Complete))
            .map(|transfer| transfer.items().to_vec())
            .unwrap_or_default()
    }

    /// The vehicle currently being flown, if any.
    ///
    /// The chosen one if there is one and it is still on the link, otherwise whichever the link
    /// considers primary. A vehicle that has been chosen and then disappears falls back rather
    /// than leaving commands addressed to something that is not there.
    fn target(&self) -> Option<(&Link, VehicleId)> {
        let link = self.link.as_ref()?;
        let id = self
            .selected
            .filter(|chosen| link.vehicles().contains(chosen))
            .or_else(|| link.primary_vehicle().map(|(id, _)| id))?;
        Some((link, id))
    }

    /// Every vehicle heard from, in a stable order.
    #[must_use]
    pub fn vehicles(&self) -> Vec<VehicleId> {
        self.link.as_ref().map(Link::vehicles).unwrap_or_default()
    }

    /// Chooses which vehicle the screens show and commands address.
    pub fn select(&mut self, id: VehicleId) {
        self.selected = Some(id);
    }

    /// The vehicle currently being flown, without the link.
    fn target_id(&self) -> Option<VehicleId> {
        self.target().map(|(_, id)| id)
    }

    // --- Requests: the calls the C# blocks on, retried by the link ---------------------------------

    /// Keeps what to say when a request ends.
    fn awaiting(&mut self, id: RequestId, report: Report) -> RequestId {
        self.awaited.push(Awaited {
            id,
            made: Instant::now(),
            report,
        });
        id
    }

    /// `doCommand` with its acknowledgement waited for: `COMMAND_LONG` to `target` until a
    /// `COMMAND_ACK` for it, sent again by the link as the C# sends again - three more times, two
    /// seconds apart; ten for arming; once more after 25 for a calibration - and `report` said
    /// when it ends. `None` without a link.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2688-2836`
    pub fn command(
        &mut self,
        target: VehicleId,
        command: u16,
        params: [f32; 7],
        report: Report,
    ) -> Option<RequestId> {
        let id = self.link.as_ref()?.command(target, command, params, true);
        Some(self.awaiting(id, report))
    }

    /// A `COMMAND_LONG` a builder made, sent as [`Telemetry::command`] sends one: to the vehicle
    /// it names, with its parameters. Nothing else is sent.
    ///
    /// The builders in `mp_link::commands` and `mp_calibration` stay the one place a command's
    /// parameters are written down; this only reads them back out.
    pub fn command_message(&mut self, message: &MavMessage, report: Report) -> Option<RequestId> {
        let (target, command, params) = command_long_parts(message)?;
        self.command(target, command, params, report)
    }

    /// `setWPCurrent`: `MISSION_SET_CURRENT` until a `MISSION_CURRENT` arrives, sent again every
    /// two seconds up to five times, and `report` said when it ends.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2452-2501`
    pub fn set_current_waypoint(
        &mut self,
        target: VehicleId,
        seq: u16,
        report: Report,
    ) -> Option<RequestId> {
        let id = self.link.as_ref()?.set_current_waypoint(target, seq);
        Some(self.awaiting(id, report))
    }

    /// `doCommandInt` with its acknowledgement waited for: `COMMAND_INT` to `target` until a
    /// `COMMAND_ACK` for it, sent again three more times two seconds apart, anything but
    /// accepted a refusal, and `report` said when it ends.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2847-2951`
    #[allow(clippy::too_many_arguments)] // the C#'s seven, plus the frame and the report
    pub fn command_int(
        &mut self,
        target: VehicleId,
        command: u16,
        frame: u8,
        params: [f32; 4],
        x: i32,
        y: i32,
        z: f32,
        report: Report,
    ) -> Option<RequestId> {
        let id = self
            .link
            .as_ref()?
            .command_int(target, command, frame, params, x, y, z, true);
        Some(self.awaiting(id, report))
    }

    /// `setWP` for one item: the `MISSION_ITEM` or `MISSION_ITEM_INT` given, until the vehicle
    /// acknowledges it or asks for the next, sent again ten more times 450 ms apart, and
    /// `report` said when it ends. `None` for a message that is not an item, or without a link.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3975-4380`
    pub fn set_wp(
        &mut self,
        target: VehicleId,
        item: MavMessage,
        report: Report,
    ) -> Option<RequestId> {
        let id = self.link.as_ref()?.set_wp(target, item)?;
        Some(self.awaiting(id, report))
    }

    /// `getHomePosition`: `GET_HOME_POSITION` until a `HOME_POSITION` arrives, asked again
    /// three more times 700 ms apart, and `report` said when it ends. The position itself lands
    /// in the vehicle's state, as every `HOME_POSITION` does.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3343-3387`
    pub fn get_home_position(&mut self, target: VehicleId, report: Report) -> Option<RequestId> {
        let id = self.link.as_ref()?.get_home_position(target);
        Some(self.awaiting(id, report))
    }

    /// `setParam` on `target`: `PARAM_SET` until the vehicle echoes the parameter, sent again every
    /// 700 ms up to three times, and `report` said when it ends. Refused without sending for a
    /// name the vehicle has not listed, and not sent for a value it already holds unless `force`.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1628-1770`
    pub fn set_parameter_on(
        &mut self,
        target: VehicleId,
        name: &str,
        value: f64,
        force: bool,
        report: Report,
    ) -> Option<RequestId> {
        let id = self.link.as_ref()?.set_param(target, name, value, force);
        Some(self.awaiting(id, report))
    }

    /// `setParam` on the vehicle being flown, for a caller that reads the outcome itself with
    /// [`Telemetry::request`] - a list of writes made one after another, as the C#'s are.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1628-1770`
    pub fn write_parameter(&self, name: &str, value: f64, force: bool) -> Option<RequestId> {
        let (link, id) = self.target()?;
        Some(link.set_param(id, name, value, force))
    }

    /// `GetParam`: `PARAM_REQUEST_READ` until the parameter arrives, sent again every 700 ms up to
    /// three times. What arrives goes into the vehicle's table.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2296-2380`
    pub fn read_parameter(&self, name: &str) -> Option<RequestId> {
        let (link, id) = self.target()?;
        Some(link.read_param(id, name))
    }

    /// Whether the vehicle being flown has listed a parameter of this name - which `setParam`
    /// needs before it will send one.
    #[must_use]
    pub fn holds_parameter(&self, name: &str) -> bool {
        self.target()
            .and_then(|(link, id)| link.params(id))
            .is_some_and(|table| table.get(name).is_some())
    }

    /// Where a request is, as the link has it. `None` once the link has forgotten it, or with
    /// no link. The link once lost sight of a request for a moment while picking it up, which
    /// is what [`Telemetry::lookup`]'s grace was for; `Link::request` now reads the queue under
    /// the table's lock, so this is safe to ask from the moment a request is made.
    #[must_use]
    pub fn request(&self, id: RequestId) -> Option<Request> {
        self.link.as_ref()?.request(id)
    }

    /// Where a request made at `made` is, telling a request the link is still picking up from
    /// one it has let go (see [`PICKUP_GRACE`]).
    #[must_use]
    pub fn lookup(&self, id: RequestId, made: Instant) -> Lookup {
        let Some(link) = &self.link else {
            return Lookup::Gone;
        };
        match link.request(id) {
            Some(request) => Lookup::Found(Box::new(request)),
            None if link.is_running() && made.elapsed() < PICKUP_GRACE => Lookup::PickingUp,
            None => Lookup::Gone,
        }
    }

    /// Once a frame: what the requests that have ended say, oldest first. A refused command's
    /// fallback goes on the wire here, as `setDigicamControl` sends it after `doCommand` returns
    /// false. A request the link has let go says nothing.
    pub fn take_reports(&mut self) -> Vec<String> {
        let Some(link) = &self.link else {
            self.awaited.clear();
            return Vec::new();
        };
        let mut said = Vec::new();
        let awaited = std::mem::take(&mut self.awaited);
        for waiting in awaited {
            let request = match self.lookup(waiting.id, waiting.made) {
                Lookup::Found(request) => request,
                Lookup::PickingUp => {
                    self.awaited.push(waiting);
                    continue;
                }
                Lookup::Gone => continue,
            };
            let Some(outcome) = request.outcome() else {
                self.awaited.push(waiting);
                continue;
            };
            if matches!(outcome, RequestOutcome::Rejected(_))
                && let Some(fallback) = &waiting.report.fallback
            {
                link.send(fallback);
            }
            if let Some(text) = waiting.report.text(outcome) {
                said.push(text.to_owned());
            }
        }
        said
    }

    /// Arms or disarms, with the vehicle's pre-arm checks applied: `doARM`, which waits ten
    /// seconds a try for the acknowledgement "as may need an imu calib". A refusal is said as
    /// `BUT_ARM_Click`'s message box begins, a timeout as its `catch` says it.
    /// `// C#: GCSViews/FlightData.cs:1034-1079, ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2632-2657`
    pub fn arm(&mut self, arm: bool) {
        let Some(id) = self.target_id() else {
            return;
        };
        let action = if arm { "Arm" } else { "Disarm" };
        let report = Report {
            refused: Some(error_box(format!("{action} failed."))),
            timed_out: Some(error_box(strings::ERROR_NO_RESPONSE)),
            ..Report::default()
        };
        self.command_message(&commands::arm(id, arm, false), report);
    }

    /// Starts a six-position accelerometer calibration.
    ///
    /// Sent and not waited for: `doCommand` returns at once for a calibration with param5 = 1,
    /// "for advanced accel offsets, and blocks execution", so the C#'s `if` always takes the
    /// true branch and the vehicle's own `COMMAND_LONG` asking for the first position is the
    /// answer.
    /// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:68-80, ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2734-2739`
    pub fn start_accelerometer_calibration(&self) {
        if let Some((link, id)) = self.target() {
            link.clear_accel_calibration();
            link.send(&mp_calibration::start_accelerometer(id));
        }
    }

    /// What the accelerometer calibration is waiting for, if anything.
    #[must_use]
    pub fn accel_calibration(&self) -> mp_calibration::AccelCalibration {
        self.link.as_ref().map_or(
            mp_calibration::AccelCalibration::Idle,
            Link::accel_calibration,
        )
    }

    /// Forgets a finished calibration, so it stops being reported as running.
    pub fn clear_accel_calibration(&self) {
        if let Some(link) = &self.link {
            link.clear_accel_calibration();
        }
    }

    /// Start: `doCommand(DO_START_MAG_CAL, 0, 1, 1, 0, 0, 0, 0)` to the vehicle being flown, retried
    /// by the link as the C# retries it. The Compass page reads the answer with
    /// [`Telemetry::request`] and says what `BUT_OBmagcalstart_Click` says; nothing is said here.
    /// `None` without a vehicle, where `doCommand` returns false.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:271-280`
    pub fn start_compass_calibration(&mut self) -> Option<RequestId> {
        let id = self.target_id()?;
        self.command_message(&mp_calibration::start_compass(id), Report::default())
    }

    /// Accept: `doCommand(DO_ACCEPT_MAG_CAL, 0, 0, 1, 0, 0, 0, 0)`, answered as Start is.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:323-333`
    pub fn accept_compass_calibration(&mut self) -> Option<RequestId> {
        let id = self.target_id()?;
        self.command_message(&mp_calibration::accept_compass(id), Report::default())
    }

    /// Cancel: `doCommand(DO_CANCEL_MAG_CAL, 0, 0, 1, 0, 0, 0, 0)`, answered as Start is.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:341-350`
    pub fn cancel_compass_calibration(&mut self) -> Option<RequestId> {
        let id = self.target_id()?;
        self.command_message(&mp_calibration::cancel_compass(id), Report::default())
    }

    /// Large Vehicle MagCal: `doCommand(FIXED_MAG_CAL_YAW, heading, 0, 0, 0, 0, 0, 0)`, answered
    /// as Start is.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:476-498`
    pub fn fixed_mag_cal_yaw(&mut self, yaw_degrees: f32) -> Option<RequestId> {
        let id = self.target_id()?;
        self.command_message(
            &mp_calibration::fixed_mag_cal_yaw(id, yaw_degrees),
            Report::default(),
        )
    }

    /// Every `MAG_CAL_PROGRESS` and `MAG_CAL_REPORT` since the last clear, as the Compass page's
    /// timer reads them.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:296-321`
    #[must_use]
    pub fn compass_calibration(&self) -> mp_calibration::MagCalLog {
        self.link
            .as_ref()
            .map(Link::compass_calibration)
            .unwrap_or_default()
    }

    /// `mprog.Clear()` and `mrep.Clear()`: forgets what the vehicle has said of a calibration.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:282-283`
    pub fn clear_compass_calibration(&self) {
        if let Some(link) = &self.link {
            link.clear_compass_calibration();
        }
    }

    /// Every `COMPASSMOT_STATUS` heard since the last call, oldest first, with the vehicle that
    /// sent it: what the Compass/Motor Calib page's subscription is handed. Nothing without a link.
    /// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:29, 87-119`
    #[must_use]
    pub fn take_compassmot_status(
        &self,
    ) -> Vec<(VehicleId, mp_mavlink_dialects::all::CompassmotStatus)> {
        self.link
            .as_ref()
            .map(Link::take_compassmot_status)
            .unwrap_or_default()
    }

    /// `SendAck`: a `COMMAND_ACK` for `MAV_CMD_PREFLIGHT_CALIBRATION`, result 0, sent twice, which
    /// ends a running `compassmot` - to nobody in particular, as the C#'s packet has its target
    /// fields at their defaults. Whether both went; false without a vehicle.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2956-2966`
    pub fn send_calibration_ack(&self) -> bool {
        let Some((link, _)) = self.target() else {
            return false;
        };
        let ack = MavMessage::CommandAck(mp_mavlink_dialects::all::CommandAck {
            command: mp_calibration::CMD_PREFLIGHT_CALIBRATION,
            result: 0,
            progress: 0,
            result_param2: 0,
            target_system: 0,
            target_component: 0,
        });
        // "send twice". The C# sleeps 20 ms between the two on its UI thread; here both are
        // handed to the link's sender at once, which puts them on the wire back to back.
        let first = link.send(&ack);
        let second = link.send(&ack);
        first && second
    }

    /// One `testMotor` call: `MAV_CMD_DO_MOTOR_TEST` to the vehicle being flown, through the
    /// link's retrying `doCommand` - sent again two seconds apart up to three more times until the
    /// vehicle's `COMMAND_ACK` - with a refusal said as "Command was denied by the autopilot" and a
    /// timeout as `Strings.ErrorCommunicating` with the motor's number. The request, to read its
    /// outcome with [`Telemetry::request`]; `None` with no vehicle.
    /// `// C#: GCSViews/ConfigurationView/ConfigMotorTest.cs:305-327, ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2688-2836`
    pub fn test_motor(
        &mut self,
        command: mp_calibration::motor::MotorCommand,
    ) -> Option<RequestId> {
        let id = self.target_id()?;
        self.command_message(&command.message(id), motor_report(command.motor))
    }

    /// Asks the vehicle to list its dataflash logs.
    pub fn request_log_list(&self) {
        if let Some((link, id)) = self.target() {
            link.request_log_list(id);
        }
    }

    /// The logs the vehicle has listed.
    #[must_use]
    pub fn log_listings(&self) -> Vec<mp_ftp::logs::LogListing> {
        self.link
            .as_ref()
            .map(Link::log_listings)
            .unwrap_or_default()
    }

    /// Starts downloading one log.
    pub fn download_log(&self, id: u16, size: u32) {
        if let Some((link, target)) = self.target() {
            link.download_log(target, id, size);
        }
    }

    /// How far a log download has got: the log, bytes received, and its size.
    #[must_use]
    pub fn log_progress(&self) -> Option<(u16, u32, u32)> {
        self.link.as_ref().and_then(Link::log_download_progress)
    }

    /// Keeps a download moving, re-requesting only when it has stalled.
    pub fn nudge_log_download(&self) {
        if let Some((link, id)) = self.target() {
            link.nudge_log_download(id);
        }
    }

    /// The finished log, once every byte has arrived.
    #[must_use]
    pub fn finished_log(&self) -> Option<(u16, Vec<u8>)> {
        self.link.as_ref().and_then(Link::finished_log)
    }

    /// Forgets a download.
    pub fn clear_log_download(&self) {
        if let Some(link) = &self.link {
            link.clear_log_download();
        }
    }

    // --- MAVFTP: the link's one client per vehicle -------------------------------------------------

    /// Starts a MAVFTP request on the vehicle being flown and returns at once: the C#'s
    /// `new MAVFtp(MainV2.comPort, MainV2.comPort.MAV.sysid, MainV2.comPort.MAV.compid)` and the
    /// call made on it, run by the link rather than on a thread of the screen's. The vehicle it
    /// went to; `None` with no vehicle, or with a request already running on that vehicle's
    /// client. How it ends is read with [`Telemetry::ftp_progress`] and
    /// [`Telemetry::take_ftp_outcome`].
    /// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:83, 105`
    pub fn start_ftp(&self, request: FtpRequest) -> Option<VehicleId> {
        let (link, id) = self.target()?;
        link.ftp(id, request).then_some(id)
    }

    /// Starts a MAVFTP request on a vehicle already asked: a later call on the same `MAVFtp`.
    /// False with no link, or with a request running on it.
    pub fn ftp_on(&self, vehicle: VehicleId, request: FtpRequest) -> bool {
        self.link
            .as_ref()
            .is_some_and(|link| link.ftp(vehicle, request))
    }

    /// Whether a request is running on the vehicle's client, and its last `Progress` report;
    /// `None` when this link has no client for the vehicle.
    #[must_use]
    pub fn ftp_progress(&self, vehicle: VehicleId) -> Option<(bool, Progress)> {
        self.link.as_ref()?.ftp_progress(vehicle)
    }

    /// Takes the vehicle's finished request's outcome, once there is one: what `GetFile` and the
    /// rest return, or the exception they throw.
    pub fn take_ftp_outcome(&self, vehicle: VehicleId) -> Option<Result<FtpOutcome, FtpError>> {
        self.link.as_ref()?.take_ftp_outcome(vehicle)
    }

    /// Asks the vehicle's running request to stop: the caller's `CancellationTokenSource.Cancel()`.
    pub fn cancel_ftp(&self, vehicle: VehicleId) {
        if let Some(link) = &self.link {
            link.cancel_ftp(vehicle);
        }
    }

    /// Other aircraft currently known about.
    #[must_use]
    pub fn traffic(&self) -> Vec<mp_link::traffic::Traffic> {
        self.link.as_ref().map(Link::traffic).unwrap_or_default()
    }

    /// `getParamList`: the parameters fetched over MAVFTP first (`@PARAM/param.pck?withdefaults=1`)
    /// and over the `PARAM_REQUEST_LIST` stream when that will not do - the parameter screen's
    /// button, and the fetch on connecting.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1781-1799`
    pub fn download_parameters(&self) {
        if let Some((link, id)) = self.target() {
            link.fetch_params(id);
        }
    }

    /// Writes one parameter and waits, on the link thread, for the vehicle to echo it: the C#'s
    /// `setParam`, with its checks and its retries, not forced. `None` with no vehicle to write
    /// to; the outcome is read with [`Telemetry::request`] or [`Telemetry::lookup`]. The config
    /// pages use this; the parameter editor uses [`Telemetry::write_parameter`] directly.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1628-1766`
    pub fn set_parameter_confirmed(&self, name: &str, value: f64) -> Option<RequestId> {
        self.write_parameter(name, value, false)
    }

    /// Reboots the autopilot: `doReboot(false, true)`.
    ///
    /// The link drops when the vehicle obeys, which is what success looks like. Useful after a
    /// calibration, and the usual first thing to try when a board is behaving oddly.
    ///
    /// `doReboot` calls `doCommand` with its acknowledgement required, and `doCommand` writes
    /// the `COMMAND_LONG` once, then for `PREFLIGHT_REBOOT_SHUTDOWN` writes it again at once -
    /// no gap - and returns true without waiting, because a vehicle that obeys has no time to
    /// answer. So it goes through the link's `doCommand` ([`Link::command`], acknowledgement
    /// required), which makes the same two sends and ends the request `Sent`. As the answer is
    /// always true, `doReboot`'s fallback second `doCommand` never runs.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2553-2589, 2717, 2758-2763`
    ///
    /// Whether there was a vehicle to send it to: `doReboot`'s return.
    ///
    /// On a serial port the C# then sleeps half a second and reopens the port if the reboot
    /// took it away; that is [`Telemetry::reopen_after_reboot`], driven by the window's frames
    /// rather than a sleep on the thread that draws them.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2573-2583`
    pub fn reboot(&self) -> bool {
        let Some((link, id)) = self.target() else {
            return false;
        };
        let Some((target, command, params)) = command_long_parts(&commands::reboot(id)) else {
            return false;
        };
        link.command(target, command, params, true);
        // `if (!bootloadermode && (BaseStream is SerialPort))`.
        if self.is_serial()
            && let Ok(mut reopen) = self.reopen.lock()
        {
            *reopen = Some(Reopen::Check(Instant::now() + REBOOT_REOPEN_WAIT));
        }
        true
    }

    /// `cs.messages.Clear()`: every message so far dropped from the view; new ones show.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs (messages)`
    pub fn clear_messages(&self) {
        let next = self
            .link
            .as_ref()
            .and_then(|link| {
                link.recent_messages(1)
                    .first()
                    .map(|message| message.seq + 1)
            })
            .unwrap_or(0);
        self.messages_shown_from
            .store(next, std::sync::atomic::Ordering::Relaxed);
    }

    /// Whether the link was opened on a serial port: `BaseStream is SerialPort`.
    fn is_serial(&self) -> bool {
        matches!(
            self.target.parse::<mp_transport::LinkUrl>(),
            Ok(mp_transport::LinkUrl::Serial { .. })
        )
    }

    /// Once a frame: the rest of a plain reboot on a serial port. Half a second after it
    /// ([`REBOOT_REOPEN_WAIT`]) the port is looked at once; still open, nothing more is done.
    /// Gone (a board on direct USB drops its port as it reboots, and the link stops at its next
    /// read), it is `Open(true)`: after `OpenBg`'s settle ([`SERIAL_SETTLE`]) the same port is
    /// opened again by `open` ([`Telemetry::connect`] in the application, as the connect
    /// button opens it) and this `Telemetry` becomes the new one, its parameters to be fetched
    /// as on any connect. A port that will not open is [`CONNECT_FAILED`] for the status line.
    ///
    /// **Not the C#'s:** the new link records to a new `.tlog`, where the C#'s reopen goes on
    /// writing the file `MainV2` opened, and `OpenBg`'s second second ("SerialPort Sleep 2",
    /// after the open, before any traffic) is not slept: the link writes as soon as it opens,
    /// as it does on the connect button.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2573-2583, 668-700, 711-760, 949-963`
    pub fn reopen_after_reboot(
        &mut self,
        now: Instant,
        open: impl FnOnce(&str) -> Self,
    ) -> Option<Reopened> {
        let reopen = self
            .reopen
            .get_mut()
            .unwrap_or_else(PoisonError::into_inner);
        match *reopen {
            None => None,
            Some(Reopen::Check(at) | Reopen::Open(at)) if now < at => None,
            Some(Reopen::Check(_)) => {
                // `if (!BaseStream.IsOpen)`.
                if self.link.as_ref().is_some_and(Link::is_running) {
                    *reopen = None;
                    None
                } else {
                    *reopen = Some(Reopen::Open(now + SERIAL_SETTLE));
                    Some(Reopened::Connecting)
                }
            }
            Some(Reopen::Open(_)) => {
                *reopen = None;
                let url = self.target.clone();
                *self = open(&url);
                Some(match self.error() {
                    Some(err) => Reopened::Failed(format!("{CONNECT_FAILED}: {url}: {err}")),
                    None => Reopened::Opened,
                })
            }
        }
    }

    /// Which arming-check parameter this vehicle has, if we have learned it yet.
    ///
    /// ArduPilot 4.7 renamed `ARMING_CHECK` to `ARMING_SKIPCHK` and inverted its sense, so the
    /// right name depends on the firmware at the other end. Asking is the only way to know:
    /// writing a name the vehicle does not have is silently ignored, which is exactly how this
    /// went wrong the first time - the button appeared to work, the parameter did not exist, and
    /// an arm that succeeded for an unrelated reason looked like proof that it had.
    #[must_use]
    pub fn arming_check_param(&self) -> Option<&'static str> {
        let link = self.link.as_ref()?;
        let (id, _) = link.primary_vehicle()?;
        let table = link.params(id)?;
        if table.get(ARMING_SKIPCHK).is_some() {
            return Some(ARMING_SKIPCHK);
        }
        if table.get(ARMING_CHECK).is_some() {
            return Some(ARMING_CHECK);
        }
        None
    }

    /// Asks the vehicle for both arming-check parameter names.
    ///
    /// Cheap - two reads - and the answer says which firmware generation this is without
    /// downloading a thousand parameters to find out. Each is `GetParam`, asked again by the link
    /// until it arrives or its retries run out; the name this firmware does not have is the one
    /// that runs out.
    pub fn probe_arming_check_param(&self) {
        self.read_parameter(ARMING_SKIPCHK);
        self.read_parameter(ARMING_CHECK);
    }

    /// Turns the vehicle's arming checks off.
    ///
    /// Writes whichever parameter this vehicle actually has. Until that is known it asks for both
    /// and writes neither: `setParam` refuses a name the vehicle has not listed, and the caller
    /// retries, by which time the answer has arrived and the right one is written.
    ///
    /// These are persistent parameters, so they stay off until something sets them back - which is
    /// the behaviour asked for, and why the caller says so on screen rather than doing it quietly.
    pub fn disable_arming_checks(&mut self) {
        self.write_arming_checks(SKIP_ALL_CHECKS, 0.0);
    }

    /// Restores the vehicle's arming checks to the firmware defaults.
    ///
    /// The defaults differ with the name: nothing skipped for the new parameter, everything
    /// checked for the old one. Writing the wrong default would be worse than writing nothing.
    pub fn enable_arming_checks(&mut self) {
        self.write_arming_checks(0.0, 1.0);
    }

    /// Writes whichever arming-check parameter this vehicle has.
    ///
    /// `skipchk` is the value for the 4.7-and-later name, `legacy` the value for the older one.
    /// They are different numbers for the same intent, because the sense was inverted along with
    /// the rename.
    ///
    /// Until the vehicle has said which it has, both are asked for and nothing is written: the
    /// write is `setParam`, which will not send a name the vehicle has not listed. A timeout is
    /// said as the C#'s parameter screen says one.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:359-363`
    fn write_arming_checks(&mut self, skipchk: f32, legacy: f32) {
        let Some(id) = self.target_id() else {
            return;
        };
        let (name, value) = match self.arming_check_param() {
            Some(name) if name == ARMING_SKIPCHK => (ARMING_SKIPCHK, skipchk),
            Some(_) => (ARMING_CHECK, legacy),
            None => {
                self.probe_arming_check_param();
                return;
            }
        };
        self.set_parameter_on(
            id,
            name,
            f64::from(value),
            false,
            Report::on_timeout(format!("Set {name} Failed")),
        );
    }

    /// Arms with the checks bypassed.
    ///
    /// Two things, because one is not enough. The magic 2989 in `MAV_CMD_COMPONENT_ARM_DISARM`
    /// param2 tells the vehicle to skip its *pre-arm* checks, and a real board refused it anyway,
    /// listing an uncalibrated accelerometer and a bad GPS fix - those are arming checks, and the
    /// magic does not touch them. Turning `ARMING_CHECK` off does.
    ///
    /// The parameter write and the command are separate messages and the parameter takes effect
    /// asynchronously, so the caller re-sends the command for a short while rather than assuming
    /// one attempt lands after the write.
    ///
    /// It is a separate call from [`Telemetry::arm`] rather than a flag on it, so that no code
    /// path can force by accident: forcing is something a caller asks for by name.
    ///
    /// The arm is `doARM(..., force: true)`: a `doCommand` the link waits on and retries. Its
    /// request is returned so the caller asks again only once this one has ended, as the C#'s
    /// caller can only call again once `doARM` has returned; nothing is said when it ends,
    /// because the caller says it.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2632-2645`
    pub fn force_arm(&mut self) -> Option<RequestId> {
        self.disable_arming_checks();
        let id = self.target_id()?;
        self.command_message(&commands::arm(id, true, true), Report::default())
    }

    /// Changes flight mode.
    ///
    /// Sent and not waited for: `setMode` sends `DO_SET_MODE` with `requireack` false and then
    /// `SET_MODE`, and returns; the mode in the next heartbeat is the answer.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4631-4641`
    pub fn set_mode(&self, custom_mode: u32) {
        if let Some((link, id)) = self.target() {
            link.send(&commands::set_mode(id, custom_mode));
        }
    }

    /// Lands where the vehicle is.
    ///
    /// A `doCommand` waited on as the take-off is, with a timeout said the same way.
    /// `// C#: GCSViews/FlightData.cs:5305-5313`
    pub fn land(&mut self) {
        let Some(id) = self.target_id() else {
            return;
        };
        self.command_message(
            &commands::land(id),
            Report::on_timeout(error_box(strings::COMMAND_FAILED)),
        );
    }

    /// Flies to a position at the given height, in Guided.
    ///
    /// Sent and not waited for: `setGuidedModeWP` puts `SET_POSITION_TARGET_GLOBAL_INT` on the
    /// wire with `generatePacket` and returns.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4449-4454, 4500-4554`
    pub fn goto(&self, position: LatLon, altitude_metres: f32) {
        if let Some((link, id)) = self.target() {
            link.send(&commands::goto_position(
                id,
                position.latitude(),
                position.longitude(),
                altitude_metres,
            ));
        }
    }

    /// The flight modes this vehicle offers, as (number, name).
    ///
    /// Empty for a vehicle family we have no mode list for; the caller shows nothing rather than a
    /// copter's modes on an unknown airframe. Derived entirely from the snapshot, so it needs no
    /// link and stays correct when the vehicle disappears mid-flight.
    #[must_use]
    pub fn modes_for(view: &TelemetryView) -> &'static [(u32, &'static str)] {
        view.state
            .as_ref()
            .and_then(|state| VehicleFamily::from_mav_type(state.vehicle_type))
            .map_or(&[][..], VehicleFamily::modes)
    }
}

impl Telemetry {
    /// Once a frame: asks a newly seen vehicle for its banner, and notices the banner arriving.
    ///
    /// Mission Planner asks for `AUTOPILOT_VERSION` (`getVersion`, three ways) and then sends
    /// `DO_SEND_BANNER` at connect and for each new vehicle, and takes the firmware version from
    /// the `STATUSTEXT` that names the vehicle; the parameter documentation for that release is
    /// fetched from it. Sent and not waited for: the C#'s `doCommand` passes `requireack` false.
    /// Without the version request ArduPilot never sends `AUTOPILOT_VERSION`, its capabilities
    /// stay 0 and the MAVFtp page never lists (found by `config-mavftp.gui`, 2026-09-25).
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:928-931, 1822-1830, 1856-1857`
    pub fn tick(&mut self) {
        let Some(link) = &self.link else {
            return;
        };
        if let Some((id, _)) = link.primary_vehicle()
            && !self.banner_requested.contains(&id)
        {
            for request in commands::get_version(id) {
                link.send(&request);
            }
            link.send(&commands::send_banner(id));
            self.banner_requested.insert(id);
        }
        if self.banner.is_none() {
            let messages = link.recent_messages(MESSAGE_LINES);
            self.banner = crate::metadata::banner_in(messages.iter().map(|m| m.text.as_str()))
                .map(str::to_owned);
        }
    }

    /// The shown vehicle's geofence as the link has seen it, in sequence order:
    /// `MAVState.fencepoints`, which the quick view's `GeoFenceDist` measures from. Empty with no
    /// vehicle, which `GeoFenceDist` reads as no fence. `// C#: ExtLibs/ArduPilot/CurrentState.cs:1632`
    #[must_use]
    pub fn fence_points(&self) -> Vec<FenceItem> {
        self.target()
            .map(|(link, id)| link.fence_points(id))
            .unwrap_or_default()
    }

    /// Set Home Alt's write: the shown vehicle's `cs.altoffsethome`, which the link applies on
    /// its next pass. Nothing without a vehicle, where the C# writes the placeholder state no
    /// screen shows. `// C#: GCSViews/FlightData.cs:1236-1247`
    pub fn set_alt_offset_home(&self, offset: f32) {
        if let Some((link, id)) = self.target() {
            link.set_alt_offset_home(id, offset);
        }
    }

    /// The Planner page's telemetry rates, handed over each frame: when they have changed since
    /// the last frame, a rate combo has set them, and they go where its handler puts them - the
    /// saved defaults, `CurrentState.rate*backup`, and the shown vehicle's `cs.rateX` - so the
    /// link's stream requests ask for them from then on. The first hand-over only takes note:
    /// the page starts from the saved defaults, which every vehicle starts from.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:573-640`
    pub fn hand_over_rates(&mut self, rates: StreamRates) {
        let first = self.rates_handed.is_none();
        if self.rates_handed.replace(rates) == Some(rates) || first {
            return;
        }
        StreamRates::set_backups(rates);
        if let Some((link, id)) = self.target() {
            link.set_stream_rates(id, rates);
        }
    }

    /// What the state's clock, its counts, its stream rates and the fence say, for a script:
    /// `vehicle.datetime.age` is how many milliseconds `datetime` is behind the wall clock -
    /// small on a live link, whose packets are stamped as they arrive; `vehicle.rates` is
    /// attitude, position, status, sensors and RC; `vehicle.geofencedist` is what a quick view
    /// bound to `GeoFenceDist` reads, to the metre, from the fence last handed to it.
    #[must_use]
    pub fn facts(&self, view: &TelemetryView) -> Vec<(&'static str, String)> {
        let state = view.state.as_deref();
        let datetime = state.map_or(DateTime::MIN, |state| state.datetime);
        #[allow(clippy::cast_possible_truncation)] // milliseconds between two dates
        let age = (DateTime::now().seconds_since(datetime) * 1000.0) as i64;
        #[allow(clippy::cast_possible_truncation)] // at most 99999 metres
        let fence_distance = state
            .and_then(|state| crate::quick::value("GeoFenceDist", state))
            .map_or_else(
                || "none".to_owned(),
                |metres| (metres.round() as i64).to_string(),
            );
        vec![
            ("vehicle.datetime.age", age.to_string()),
            ("vehicle.datetime.ticks", datetime.ticks().to_string()),
            (
                "vehicle.timeinair",
                state.map_or(0.0, |state| state.time_in_air).to_string(),
            ),
            (
                "vehicle.disttraveled",
                state.map_or(0.0, |state| state.dist_traveled).to_string(),
            ),
            (
                "vehicle.rates",
                state.map_or_else(
                    || "none".to_owned(),
                    |state| {
                        let rates = state.rates;
                        format!(
                            "{},{},{},{},{}",
                            rates.attitude, rates.position, rates.status, rates.sensors, rates.rc
                        )
                    },
                ),
            ),
            (
                "vehicle.fence.points",
                self.fence_points().len().to_string(),
            ),
            ("vehicle.geofencedist", fence_distance),
        ]
    }

    /// Publishes [`Telemetry::facts`].
    pub fn record_facts(&self, view: &TelemetryView) {
        for (key, value) in self.facts(view) {
            crate::facts::record(key, value);
        }
    }

    /// The firmware banner, if the vehicle has said it.
    #[must_use]
    pub fn firmware_banner(&self) -> Option<&str> {
        self.banner.as_deref()
    }

    /// Where stick frames should go right now: a handle that sends on the link, and the vehicle.
    ///
    /// For the joystick reader's send thread, which cannot borrow the link and must not wait for
    /// the screen. `None` with no vehicle, which the reader treats as a refused send - for a
    /// release frame that is the difference between an aircraft handed back and one still being
    /// flown by a stick nobody is holding, so it is retried rather than swallowed. Asked once a
    /// frame, so the vehicle chosen on screen is the one the next frame is addressed to; the
    /// handle itself is two ids and a channel, cheap to hand over every time.
    pub fn send_handle(&self) -> Option<(mp_link::LinkSender, VehicleId)> {
        self.target().map(|(link, id)| (link.sender(), id))
    }

    /// `MainV2.comPort.OnPacketReceived += handler` and `OnPacketSent += handler`: every packet
    /// the link reads or writes, told to `handler` on the link thread until the subscription is
    /// dropped. `None` with no link: the C#'s `comPort` is there before a connection, and a
    /// subscriber that wants one asks again once there is ([`Telemetry::carries`]).
    /// `// C#: Controls/MAVLinkInspector.cs:34, 427-428`
    pub fn on_packet(
        &self,
        handler: impl FnMut(&mp_link::inspector::Packet) + Send + 'static,
    ) -> Option<mp_link::inspector::PacketSubscription> {
        self.link.as_ref().map(|link| link.on_packet(handler))
    }

    /// Whether `subscription` is to the link this telemetry has now: false once a connect or a
    /// reopen has put another in its place, where the C#'s one `comPort` keeps its subscribers.
    pub fn carries(&self, subscription: &mp_link::inspector::PacketSubscription) -> bool {
        self.link
            .as_ref()
            .is_some_and(|link| link.carries(subscription))
    }

    /// Whether there is a link to subscribe to.
    pub const fn has_link(&self) -> bool {
        self.link.is_some()
    }

    /// `MainV2.comPort.MAV.Camera`: the shown vehicle's camera, and the vehicle.
    /// `// C#: Controls/GimbalVideoControl.cs:53-63`
    pub fn camera(&self) -> Option<(VehicleId, mp_link::camera::Camera)> {
        self.target()
            .and_then(|(link, id)| link.camera(id).map(|camera| (id, camera)))
    }

    /// `MainV2.comPort.MAV.GimbalManager`: the shown vehicle's gimbal manager, and the vehicle.
    /// `// C#: Controls/GimbalVideoControl.cs:80-90`
    pub fn gimbal_manager(&self) -> Option<(VehicleId, mp_link::gimbal_manager::GimbalManager)> {
        self.target()
            .and_then(|(link, id)| link.gimbal_manager(id).map(|manager| (id, manager)))
    }

    /// `CameraProtocol.VideoStreams`.
    pub fn video_streams(
        &self,
    ) -> Vec<(
        (u8, u8, u8),
        mp_mavlink_dialects::all::VideoStreamInformation,
    )> {
        self.link
            .as_ref()
            .map(Link::video_streams)
            .unwrap_or_default()
    }

    /// `selectedCamera?.RequestCameraInformationAsync()` for the shown vehicle's camera; false
    /// with no link or no started camera.
    pub fn request_camera_information(&self) -> bool {
        self.target()
            .is_some_and(|(link, id)| link.request_camera_information(id))
    }

    /// Whether that request is still under way.
    pub fn camera_information_pending(&self) -> bool {
        self.target()
            .is_some_and(|(link, id)| link.camera_information_pending(id))
    }

    /// Sets the shown vehicle's stream rates, `MainV2.comPort.MAV.cs.rateX`, without saving them
    /// as the defaults: what Radio Calibration does around its capture.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:214-217, 388-391`
    pub fn set_stream_rates(&self, rates: mp_vehicle::StreamRates) {
        if let Some((link, id)) = self.target() {
            link.set_stream_rates(id, rates);
        }
    }

    /// `requestDatastream(MAV_DATA_STREAM.RAW_SENSORS, hz)`: the RAW Sensor window asking for
    /// the raw sensor stream at a rate now - twice, as `getDatastream` sends every request; the
    /// link asks again at the vehicle's rates on its own clock. Nothing without a vehicle.
    /// `// C#: Controls/RAW_Sensor.cs:257, 272-273; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3262-3263`
    pub fn request_raw_sensors(&self, hz: i32) {
        let Some((link, id)) = self.target() else {
            return;
        };
        let Some(request) = mp_link::current_settings::request_datastream(
            id,
            mp_link::current_settings::RAW_SENSORS,
            hz,
        ) else {
            return;
        };
        for _ in 0..2 {
            link.send(&request);
        }
    }
}

/// A `COMMAND_LONG` a builder made, read back as `doCommand`'s arguments: the vehicle it names,
/// the command and its seven parameters. `None` for any other message.
pub(crate) fn command_long_parts(message: &MavMessage) -> Option<(VehicleId, u16, [f32; 7])> {
    let MavMessage::CommandLong(long) = message else {
        return None;
    };
    Some((
        VehicleId::new(long.target_system, long.target_component),
        long.command,
        [
            long.param1,
            long.param2,
            long.param3,
            long.param4,
            long.param5,
            long.param6,
            long.param7,
        ],
    ))
}

/// A scripted vehicle on the far end of an in-memory link, for driving a screen's sets and
/// commands through the real link - its thread, its request machines, their retries - in a test.
///
/// The arrangement of `mp-link`'s `tests/retries.rs`: the link is the real one over a
/// [`mp_transport::testing::Loopback`], and the vehicle is a script that reads what the link sent
/// and decides what, if anything, to answer. A send the script does not answer is a send lost on
/// the way.
#[cfg(test)]
pub mod scripted {
    use std::time::{Duration, Instant};

    use mp_link::{Link, LinkConfig, ProtocolTimeouts};
    use mp_mavlink::{FrameDecoder, encode_v2};
    use mp_mavlink_dialects::all::{CommandAck, DIALECT, Heartbeat, MavMessage, ParamValue};
    use mp_transport::Transport;
    use mp_transport::testing::{Loopback, LoopbackEnd};
    use mp_vehicle::VehicleId;

    use super::Telemetry;

    /// The autopilot every test talks to.
    pub const VEHICLE: VehicleId = VehicleId::new(1, 1);
    /// The link's own address, which the vehicle's acknowledgements are addressed to.
    const GCS: VehicleId = VehicleId::new(255, 190);
    /// `MAV_PARAM_TYPE_INT32`, which is how ArduPilot declares `RTL_ALT`.
    pub const INT32: u8 = 6;
    /// How long anything may take before the test is declared hung.
    const HUNG: Duration = Duration::from_secs(5);

    /// The vehicle's end of the link.
    pub struct Vehicle {
        end: LoopbackEnd,
        decoder: FrameDecoder,
        seq: u8,
        /// Everything the link has sent it, in order.
        pub heard: Vec<MavMessage>,
    }

    impl Vehicle {
        /// A link with Mission Planner's retry counts and these waits, the screens' telemetry
        /// over it, and an ArduPilot copter on the other end that has announced itself.
        pub fn connect(timeouts: ProtocolTimeouts) -> (Telemetry, Self) {
            Self::connect_as(timeouts, "loopback")
        }

        /// [`Vehicle::connect`], the screens told the link was opened from `url`: a serial
        /// port's, for what is done only on a serial link. Nothing is opened from it.
        pub fn connect_as(timeouts: ProtocolTimeouts, url: &str) -> (Telemetry, Self) {
            let (link, vehicle) = Self::link(timeouts);
            (Telemetry::over(link, url), vehicle)
        }

        /// The link alone, with the copter on its far end announced and seen: for what runs
        /// over a link without the screens, the firmware page's reboot.
        pub fn link(timeouts: ProtocolTimeouts) -> (Link, Self) {
            let (link, mut vehicle) = Self::link_silent(timeouts);
            vehicle.heartbeat();
            until("the vehicle to be seen", || {
                link.vehicles().contains(&VEHICLE)
            });
            (link, vehicle)
        }

        /// [`Self::link`] with nothing heard yet: the test sends the first heartbeat, from
        /// whichever component it wants heard first.
        pub fn link_silent(timeouts: ProtocolTimeouts) -> (Link, Self) {
            let (vehicle_side, gcs_side) = Loopback::pair();
            let config = LinkConfig {
                send_heartbeat: false,
                stream_rate_hz: 0,
                timeouts,
                ..LinkConfig::default()
            };
            let link = Link::from_transport(Box::new(gcs_side), config);
            let vehicle = Self {
                end: vehicle_side,
                decoder: FrameDecoder::new(),
                seq: 0,
                heard: Vec::new(),
            };
            (link, vehicle)
        }

        /// The copter's `HEARTBEAT`.
        pub fn heartbeat(&mut self) {
            self.send(&MavMessage::Heartbeat(Heartbeat {
                custom_mode: 0,
                r#type: 2,
                autopilot: 3,
                base_mode: 81,
                system_status: 3,
                mavlink_version: 3,
            }));
        }

        /// The cable pulled, as a board rebooting drops its USB port: the link learns it at its
        /// next read.
        pub fn unplug(&self) {
            self.end.plug().pull();
        }

        /// Sends a message as the autopilot.
        pub fn send(&mut self, message: &MavMessage) {
            self.send_from(VEHICLE, message);
        }

        /// Sends a message as another vehicle on the same link.
        pub fn send_from(&mut self, from: VehicleId, message: &MavMessage) {
            let mut payload = [0u8; 255];
            let len = message.encode(&mut payload);
            let mut frame = [0u8; mp_mavlink::MAX_FRAME_LEN];
            let n = encode_v2(
                &mut frame,
                self.seq,
                from.sysid,
                from.compid,
                message.id(),
                &payload[..len],
                message.crc_extra(),
                0,
            )
            .unwrap();
            self.seq = self.seq.wrapping_add(1);
            self.end.write_all(&frame[..n]).unwrap();
        }

        /// What the link has sent since the last call; each is also kept in `heard`.
        pub fn read(&mut self) -> Vec<MavMessage> {
            let mut buf = [0u8; 4096];
            let mut fresh = Vec::new();
            loop {
                let n = self.end.read(&mut buf).unwrap_or(0);
                if n == 0 {
                    break;
                }
                self.decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                    if let Some(message) = MavMessage::decode(frame.msgid, frame.payload) {
                        fresh.push(message);
                    }
                });
            }
            self.heard.extend(fresh.iter().copied());
            fresh
        }

        /// How many of the messages heard `pick` recognises.
        pub fn count(&self, pick: impl Fn(&MavMessage) -> bool) -> usize {
            self.heard.iter().filter(|message| pick(message)).count()
        }
    }

    /// A `PARAM_VALUE` as the vehicle sends it: the only parameter of a one-parameter vehicle.
    pub fn param(name: &str, value: f32, param_type: u8) -> MavMessage {
        MavMessage::ParamValue(ParamValue {
            param_value: value,
            param_count: 1,
            param_index: 0,
            param_id: mp_params::encode_param_id(name),
            param_type,
        })
    }

    /// The vehicle's `COMMAND_ACK` for `command`.
    pub fn ack(command: u16, result: u8) -> MavMessage {
        MavMessage::CommandAck(CommandAck {
            command,
            result,
            progress: 0,
            result_param2: 0,
            target_system: GCS.sysid,
            target_component: GCS.compid,
        })
    }

    /// Polls until `check` holds, failing rather than hanging.
    pub fn until(what: &str, mut check: impl FnMut() -> bool) {
        let deadline = Instant::now() + HUNG;
        while !check() {
            assert!(Instant::now() < deadline, "timed out waiting for {what}");
            std::thread::sleep(Duration::from_millis(1));
        }
    }
}

/// A view shares its parameter list with the last one until a parameter arrives.
#[cfg(test)]
mod shared_parameters {
    use std::sync::Arc;
    use std::time::{Duration, Instant};

    use mp_link::ProtocolTimeouts;
    use mp_mavlink_dialects::all::{MavMessage, ParamValue};

    use super::scripted::{Vehicle, param, until};

    /// `MAV_PARAM_TYPE_REAL32`.
    const REAL32: u8 = 9;

    /// Equal as a float carried on the wire can be.
    fn close(left: f64, right: f64) -> bool {
        (left - right).abs() <= 1e-6 * right.abs().max(1.0)
    }

    /// SITL's 1,408 parameters, every line of the dump, sent as the vehicle sends a download.
    fn send_sitl(vehicle: &mut Vehicle) -> Vec<(String, f64)> {
        let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/params/sitl-copter.param");
        let text = std::fs::read_to_string(&fixture).expect("the SITL dump");
        let sent: Vec<(String, f64)> = text
            .lines()
            .filter_map(|line| {
                let (name, value) = line.split_once(',')?;
                Some((name.to_owned(), value.trim().parse().ok()?))
            })
            .collect();
        assert_eq!(sent.len(), 1408);
        let count = u16::try_from(sent.len()).expect("fits");
        for (index, (name, value)) in sent.iter().enumerate() {
            #[allow(clippy::cast_possible_truncation)] // SITL's values are f32 on the wire
            vehicle.send(&MavMessage::ParamValue(ParamValue {
                param_value: *value as f32,
                param_count: count,
                param_index: u16::try_from(index).expect("fits"),
                param_id: mp_params::encode_param_id(name),
                param_type: REAL32,
            }));
        }
        sent
    }

    fn per_view(telemetry: &super::Telemetry) -> Duration {
        const VIEWS: u32 = 200;
        let started = Instant::now();
        for _ in 0..VIEWS {
            let _ = std::hint::black_box(telemetry.view());
        }
        started.elapsed() / VIEWS
    }

    /// Two frames over the same table carry the same list - one allocation, not a copy each - and
    /// a parameter arriving gives the next frame a new list with the new value, which is then
    /// shared in its turn. The list is the table's, in the table's order, either way.
    #[test]
    fn a_view_shares_the_parameter_list_until_a_parameter_arrives() {
        let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default());
        let empty = per_view(&telemetry);
        let sent = send_sitl(&mut vehicle);
        until("every parameter", || {
            telemetry.view().parameters.len() == sent.len()
        });

        let first = telemetry.view();
        let second = telemetry.view();
        assert!(
            Arc::ptr_eq(&first.parameters, &second.parameters),
            "the second frame copied a table that had not changed"
        );
        assert_eq!(first.parameters_expected, 1408);
        let mut expected = sent;
        expected.sort_by(|a, b| a.0.cmp(&b.0));
        assert_eq!(first.parameters.len(), expected.len());
        for ((name, value), (sent_name, sent_value)) in first.parameters.iter().zip(&expected) {
            assert_eq!(name, sent_name);
            assert!(
                close(*value, *sent_value),
                "{name}: {value} for {sent_value}"
            );
        }
        let full = per_view(&telemetry);
        eprintln!("a view with no parameters: {empty:?}; with SITL's 1,408: {full:?}");

        vehicle.send(&param("RTL_ALT_M", 42.5, REAL32));
        until("the new value", || {
            telemetry
                .view()
                .parameters
                .iter()
                .any(|(name, value)| name == "RTL_ALT_M" && close(*value, 42.5))
        });
        let third = telemetry.view();
        assert!(
            !Arc::ptr_eq(&first.parameters, &third.parameters),
            "a parameter arrived and the old list was handed out"
        );
        assert_eq!(third.parameters.len(), 1408, "RTL_ALT_M was already held");
        assert_eq!(third.parameters_expected, 1408, "the largest count claimed");
        assert!(Arc::ptr_eq(&third.parameters, &telemetry.view().parameters));
    }
}

#[cfg(test)]
mod tests {
    use super::scripted::{INT32, VEHICLE, Vehicle, ack, param, until};
    use super::*;
    use crate::fly::{Route, action_report, route, send_routed, set_mode_messages};
    use mp_link::ProtocolTimeouts;
    use mp_link::requests::MAV_RESULT_ACCEPTED;

    /// `MAV_RESULT_DENIED`.
    const DENIED: u8 = 2;

    /// What the screens are told a loopback link was opened from, for what is done only on a
    /// serial port. Never opened: the tests hand their own link to the reopen.
    const SERIAL: &str = "serial:/nonexistent/mp-test-port:115200";

    /// The link's waits, divided so a test runs the C#'s full retry ladder in a blink; every
    /// count stays the C#'s.
    fn fast() -> ProtocolTimeouts {
        ProtocolTimeouts::default().faster(20)
    }

    /// Message sequence numbers start at 0 and that message shows; `cs.messages.Clear()` drops
    /// what is there and the next message shows. The first version of the clear hid message 0
    /// for good, which the accelerometer page's failure test caught on 2026-09-25.
    #[test]
    fn the_first_message_shows_and_a_clear_drops_only_what_came_before() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&crate::fly::statustext(6, "first"));
        until("the first message", || {
            telemetry
                .view()
                .messages
                .iter()
                .any(|message| message.text == "first")
        });
        assert!(
            telemetry
                .view()
                .messages
                .iter()
                .any(|message| message.seq == 0),
            "message 0 is shown"
        );
        telemetry.clear_messages();
        assert!(telemetry.view().messages.is_empty(), "cleared");
        vehicle.send(&crate::fly::statustext(6, "second"));
        until("the second message", || {
            telemetry
                .view()
                .messages
                .iter()
                .any(|message| message.text == "second")
        });
        assert!(
            !telemetry
                .view()
                .messages
                .iter()
                .any(|message| message.text == "first"),
            "what was cleared stays cleared"
        );
    }

    /// The `COMMAND_LONG`s the vehicle heard, as (command, confirmation).
    fn longs(vehicle: &Vehicle) -> Vec<(u16, u8)> {
        vehicle
            .heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::CommandLong(long) => Some((long.command, long.confirmation)),
                _ => None,
            })
            .collect()
    }

    /// A reboot is `doReboot(false, true)`: `doCommand` writes `PREFLIGHT_REBOOT_SHUTDOWN`
    /// with param1 = 1 and, for a reboot, writes it again at once and returns without waiting.
    /// Two frames on the wire, both at confirmation 0, the request ended `Sent`, and nothing
    /// more however long the vehicle stays quiet.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2553-2567, 2717, 2758-2763`
    #[test]
    fn a_reboot_is_sent_twice_and_not_waited_for() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        assert!(telemetry.reboot());
        let reboot = |m: &MavMessage| {
            matches!(m, MavMessage::CommandLong(l)
                if l.command == mp_link::requests::CMD_PREFLIGHT_REBOOT_SHUTDOWN
                    && l.param1 == 1.0
                    && l.target_system == VEHICLE.sysid
                    && l.target_component == VEHICLE.compid)
        };
        until("the reboot to be sent twice", || {
            vehicle.read();
            vehicle.count(reboot) == 2
        });
        // The link's retry would come after `command.timeout`; there is none.
        std::thread::sleep(fast().command.timeout * 3);
        vehicle.read();
        assert_eq!(
            longs(&vehicle),
            [
                (mp_link::requests::CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0),
                (mp_link::requests::CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0)
            ]
        );
    }

    /// A board on a serial port drops the port as it reboots: half a second on, the port is
    /// found gone and, after `OpenBg`'s second, opened again - here onto a second copter - and
    /// the screens run over the new link.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2573-2583, 717-722`
    #[test]
    fn a_serial_port_the_reboot_took_away_is_opened_again() {
        let (mut telemetry, vehicle) = Vehicle::connect_as(fast(), SERIAL);
        let before = Instant::now();
        assert!(telemetry.reboot());
        let rebooted = Instant::now();
        vehicle.unplug();
        until("the link to see the port go", || {
            !telemetry.link.as_ref().is_some_and(Link::is_running)
        });
        let never = |_: &str| -> Telemetry { panic!("opened early") };
        // Before the half second: nothing.
        let early = before + REBOOT_REOPEN_WAIT - Duration::from_millis(1);
        assert_eq!(telemetry.reopen_after_reboot(early, never), None);
        // At it: the port is gone, and `Open(true)` begins.
        let looked = rebooted + REBOOT_REOPEN_WAIT;
        assert_eq!(
            telemetry.reopen_after_reboot(looked, never),
            Some(Reopened::Connecting)
        );
        // Not before `OpenBg`'s settle.
        let settling = looked + SERIAL_SETTLE - Duration::from_millis(1);
        assert_eq!(telemetry.reopen_after_reboot(settling, never), None);
        let mut opened_on = None;
        let mut second = None;
        let reopened = telemetry.reopen_after_reboot(looked + SERIAL_SETTLE, |url| {
            opened_on = Some(url.to_owned());
            let (telemetry, vehicle) = Vehicle::connect_as(fast(), url);
            second = Some(vehicle);
            telemetry
        });
        assert_eq!(reopened, Some(Reopened::Opened));
        assert_eq!(opened_on.as_deref(), Some(SERIAL));
        assert!(telemetry.vehicles().contains(&VEHICLE));
        // Once: the reopen is over.
        assert_eq!(
            telemetry.reopen_after_reboot(looked + SERIAL_SETTLE * 10, never),
            None
        );
        // The new link carries the screens' commands.
        let mut second = second.expect("the second copter");
        assert!(telemetry.reboot());
        until("the reboot on the new link", || {
            second.read();
            second.count(|m| matches!(m, MavMessage::CommandLong(_))) == 2
        });
    }

    /// The port still open half a second on: `doReboot` does nothing more, and nothing is
    /// looked at again.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2579-2582`
    #[test]
    fn a_serial_port_still_open_after_the_reboot_is_left_alone() {
        let (mut telemetry, _vehicle) = Vehicle::connect_as(fast(), SERIAL);
        assert!(telemetry.reboot());
        let rebooted = Instant::now();
        let never = |_: &str| -> Telemetry { panic!("reopened an open port") };
        let looked = rebooted + REBOOT_REOPEN_WAIT;
        assert_eq!(telemetry.reopen_after_reboot(looked, never), None);
        assert_eq!(
            telemetry.reopen_after_reboot(looked + SERIAL_SETTLE * 10, never),
            None
        );
        assert!(telemetry.link.as_ref().is_some_and(Link::is_running));
    }

    /// A port that will not open again is `Strings.ConnectFailed` for the status line, and the
    /// screens are left disconnected with the reason, as a failed connect leaves them.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:949-963`
    #[test]
    fn a_serial_port_that_will_not_reopen_is_connect_failed_on_the_status_line() {
        let (mut telemetry, vehicle) = Vehicle::connect_as(fast(), SERIAL);
        assert!(telemetry.reboot());
        let rebooted = Instant::now();
        vehicle.unplug();
        until("the link to see the port go", || {
            !telemetry.link.as_ref().is_some_and(Link::is_running)
        });
        let looked = rebooted + REBOOT_REOPEN_WAIT;
        assert_eq!(
            telemetry.reopen_after_reboot(looked, |_| panic!("opened early")),
            Some(Reopened::Connecting)
        );
        let failed = telemetry.reopen_after_reboot(looked + SERIAL_SETTLE, |url| Telemetry {
            error: Some("No such file or directory".to_owned()),
            target: url.to_owned(),
            ..Telemetry::idle()
        });
        assert_eq!(
            failed,
            Some(Reopened::Failed(format!(
                "Connect Failed: {SERIAL}: No such file or directory"
            )))
        );
        assert!(telemetry.link.is_none());
        assert_eq!(telemetry.error(), Some("No such file or directory"));
    }

    /// Only a serial port is looked at again: a network link that stops after a reboot is not
    /// reopened.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2576`
    #[test]
    fn a_network_link_is_not_reopened_after_a_reboot() {
        let (mut telemetry, vehicle) = Vehicle::connect_as(fast(), "tcp:127.0.0.1:5760");
        assert!(telemetry.reboot());
        let rebooted = Instant::now();
        vehicle.unplug();
        until("the link to see the port go", || {
            !telemetry.link.as_ref().is_some_and(Link::is_running)
        });
        let never = |_: &str| -> Telemetry { panic!("reopened a network link") };
        for after in [REBOOT_REOPEN_WAIT, REBOOT_REOPEN_WAIT + SERIAL_SETTLE * 2] {
            assert_eq!(telemetry.reopen_after_reboot(rebooted + after, never), None);
        }
    }

    /// No vehicle: nothing to reboot, `doReboot`'s false.
    #[test]
    fn a_reboot_without_a_link_is_false() {
        assert!(!Telemetry::idle().reboot());
    }

    /// Change Speed's `DO_CHANGE_SPEED`, the way the flight screen sends it: through [`route`]
    /// to the link's retrying command. The first send is lost - the vehicle never hears it
    /// answered - so the link sends it again with its confirmation counted up, the vehicle
    /// accepts that one, and the press has nothing to say: `doCommand`'s retry, on the screen's
    /// own path.
    /// `// C#: GCSViews/FlightData.cs:4426-4438, ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2783-2834`
    #[test]
    fn a_command_whose_first_send_is_lost_is_sent_again_and_accepted() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let press = [commands::change_speed(VEHICLE, 7.5)];
        let Route::Command { command, .. } = route(&press[0]) else {
            panic!("DO_CHANGE_SPEED is doCommand's");
        };
        let report = Report::on_timeout(error_box(strings::ERROR_COMMUNICATING));
        let (sender, _) = telemetry.send_handle().unwrap();
        let (requests, queued) = send_routed(&mut telemetry, &sender, &press, &report, true);
        assert!(queued);
        let [id] = requests[..] else {
            panic!("one request: {requests:?}");
        };

        let mut said = Vec::new();
        until("the command to end", || {
            for message in vehicle.read() {
                if matches!(message, MavMessage::CommandLong(_)) && longs(&vehicle).len() == 2 {
                    vehicle.send(&ack(command, MAV_RESULT_ACCEPTED));
                }
            }
            said.extend(telemetry.take_reports());
            telemetry
                .request(id)
                .is_some_and(|request| request.is_finished())
        });

        assert_eq!(longs(&vehicle), [(command, 0), (command, 1)]);
        let request = telemetry.request(id).unwrap();
        assert_eq!(request.sends(), 2);
        assert_eq!(
            request.outcome(),
            Some(RequestOutcome::Accepted { value: None })
        );
        said.extend(telemetry.take_reports());
        assert!(said.is_empty(), "{said:?}");
    }

    /// Set WP's `MISSION_SET_CURRENT`, the way the flight screen sends it, with the first lost:
    /// sent again, and the vehicle's `MISSION_CURRENT` ends it with nothing to say. The mode
    /// request in the same kind of press goes once, as `setMode` sends it, however long the
    /// vehicle stays quiet about it.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2457-2501, 4631-4641`
    #[test]
    fn set_wp_whose_first_send_is_lost_is_sent_again_and_a_mode_request_is_sent_once() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let (sender, _) = telemetry.send_handle().unwrap();
        let report = Report::on_timeout(error_box(strings::COMMAND_FAILED));
        let (requests, _) = send_routed(
            &mut telemetry,
            &sender,
            &[commands::mission_set_current(VEHICLE, 2)],
            &report,
            true,
        );
        let [id] = requests[..] else {
            panic!("one request: {requests:?}");
        };
        let is_set_current = |m: &MavMessage| matches!(m, MavMessage::MissionSetCurrent(_));
        let mut said = Vec::new();
        until("the set-current to end", || {
            for message in vehicle.read() {
                if is_set_current(&message) && vehicle.count(is_set_current) == 2 {
                    vehicle.send(&MavMessage::MissionCurrent(
                        mp_mavlink_dialects::all::MissionCurrent {
                            seq: 2,
                            total: 0,
                            mission_state: 0,
                            mission_mode: 0,
                        },
                    ));
                }
            }
            said.extend(telemetry.take_reports());
            telemetry.request(id).is_some_and(|r| r.is_finished())
        });
        let request = telemetry.request(id).unwrap();
        assert_eq!(request.sends(), 2);
        assert_eq!(
            request.outcome(),
            Some(RequestOutcome::Accepted { value: None })
        );
        assert!(said.is_empty(), "{said:?}");

        // Guided: DO_SET_MODE without an ack waited for, then SET_MODE twice. No request.
        let press = set_mode_messages(VEHICLE, Some(VehicleFamily::Copter), "GUIDED");
        let (requests, queued) = send_routed(&mut telemetry, &sender, &press, &report, true);
        assert!(requests.is_empty() && queued);
        std::thread::sleep(fast().command.timeout * 3);
        vehicle.read();
        assert_eq!(
            vehicle.count(|m| matches!(m, MavMessage::CommandLong(l) if l.command == commands::CMD_DO_SET_MODE)),
            1
        );
        assert_eq!(vehicle.count(|m| matches!(m, MavMessage::SetMode(_))), 2);
    }

    /// Set WP to a vehicle that never answers: `MISSION_SET_CURRENT` six times, two seconds
    /// apart in the C#, then its `catch` - said on the status line, once.
    /// `// C#: GCSViews/FlightData.cs:1658-1672, ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2472-2491`
    #[test]
    fn every_retry_unanswered_is_said_as_the_csharps_catch_says_it() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let report = Report::on_timeout(error_box(strings::COMMAND_FAILED));
        telemetry.set_current_waypoint(VEHICLE, 2, report).unwrap();

        let mut said = Vec::new();
        until("the set-current to give up", || {
            vehicle.read();
            said.extend(telemetry.take_reports());
            !said.is_empty()
        });
        vehicle.read();

        assert_eq!(said, ["Error: The Command failed to execute"]);
        assert_eq!(
            vehicle.count(|m| matches!(m, MavMessage::MissionSetCurrent(s) if s.seq == 2)),
            6
        );
        assert!(telemetry.take_reports().is_empty(), "said once");
    }

    /// Trigger Camera refused: `setDigicamControl` falls back to `DIGICAM_CONTROL`, and the
    /// press says nothing more.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4557-4569`
    #[test]
    fn a_refused_camera_trigger_falls_back_to_digicam_control() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let message = crate::fly::action_messages(
            "Trigger_Camera",
            &crate::fly::ActionContext {
                target: VEHICLE,
                copter: true,
                motor_outputs_enabled: false,
                now_unix_usec: 0,
            },
        )
        .unwrap()
        .remove(0);
        telemetry
            .command_message(&message, action_report("Trigger_Camera", VEHICLE))
            .unwrap();

        let mut said = Vec::new();
        until("the fallback", || {
            for message in vehicle.read() {
                if let MavMessage::CommandLong(long) = message {
                    vehicle.send(&ack(long.command, DENIED));
                }
            }
            said.extend(telemetry.take_reports());
            vehicle.count(|m| matches!(m, MavMessage::DigicamControl(d) if d.shot == 1)) == 1
        });
        assert!(said.is_empty(), "{said:?}");
        assert_eq!(longs(&vehicle).len(), 1, "a refusal is not retried");
    }

    /// A request missing from the link a moment after it was made is being picked up - the link
    /// moves it from queue to table in two steps, and finds it in neither between them - and one
    /// missing for longer has been let go. Without this, a write polled in that moment was taken
    /// as never answered and the next write overtook it.
    #[test]
    fn a_request_missing_a_moment_after_it_was_made_is_being_picked_up() {
        let (mut elsewhere, _vehicle) = Vehicle::connect(fast());
        let id = elsewhere
            .set_current_waypoint(VEHICLE, 1, Report::default())
            .unwrap();
        // Requests are numbered per link, and this link has made none.
        let (telemetry, _other) = Vehicle::connect(fast());
        assert!(matches!(
            telemetry.lookup(id, Instant::now()),
            Lookup::PickingUp
        ));
        let long_ago = Instant::now().checked_sub(PICKUP_GRACE).unwrap();
        assert!(matches!(telemetry.lookup(id, long_ago), Lookup::Gone));
        until("the request to be held", || {
            matches!(elsewhere.lookup(id, long_ago), Lookup::Found(_))
        });
    }

    /// Arm refused: `BUT_ARM_Click`'s message box begins "Arm failed."; the status line says it.
    /// `// C#: GCSViews/FlightData.cs:1057-1066`
    #[test]
    fn a_refused_arm_says_so() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        telemetry.arm(true);
        let mut said = Vec::new();
        until("the refusal", || {
            for message in vehicle.read() {
                if let MavMessage::CommandLong(long) = message {
                    vehicle.send(&ack(long.command, DENIED));
                }
            }
            said.extend(telemetry.take_reports());
            !said.is_empty()
        });
        assert_eq!(said, ["Error: Arm failed."]);
        assert_eq!(longs(&vehicle), [(commands::CMD_COMPONENT_ARM_DISARM, 0)]);
    }

    /// The arming-check write goes only to a name the vehicle has listed; until then it is
    /// asked for, and nothing is written.
    #[test]
    fn arming_checks_are_written_only_once_the_vehicle_has_named_its_parameter() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        telemetry.disable_arming_checks();
        until("both names asked for", || {
            vehicle.read();
            vehicle.count(|m| matches!(m, MavMessage::ParamRequestRead(_))) >= 2
        });
        assert_eq!(vehicle.count(|m| matches!(m, MavMessage::ParamSet(_))), 0);

        vehicle.send(&param(ARMING_SKIPCHK, 0.0, INT32));
        until("the name to be known", || {
            telemetry.arming_check_param() == Some(ARMING_SKIPCHK)
        });
        telemetry.disable_arming_checks();
        until("the write", || {
            vehicle.read();
            vehicle.count(|m| matches!(m, MavMessage::ParamSet(_))) == 1
        });
        let sets: Vec<_> = vehicle
            .heard
            .iter()
            .filter_map(|m| match m {
                MavMessage::ParamSet(set) => {
                    Some((mp_params::decode_param_id(&set.param_id), set.param_value))
                }
                _ => None,
            })
            .collect();
        assert_eq!(sets, [(ARMING_SKIPCHK.to_owned(), SKIP_ALL_CHECKS)]);
    }

    /// The name is what a pilot reads off a wall clock, not what a machine reads off a file.
    #[test]
    fn stamp_has_the_shape_mission_planner_uses() {
        let stamp = flight_stamp();
        assert_eq!(stamp.len(), 19, "{stamp}");
        let (date, time) = stamp
            .split_once(' ')
            .expect("a space between date and time");
        assert!(
            date.split('-')
                .all(|part| part.chars().all(|c| c.is_ascii_digit())),
            "{date}"
        );
        assert_eq!(time.split('-').count(), 3, "{time}");
        // Colons are what a human would write and what Windows refuses in a filename, which is
        // why Mission Planner uses hyphens and why this asserts their absence.
        assert!(!stamp.contains(':'), "{stamp}");
    }

    /// Two connections in the same second must not have one silently lose its recording.
    #[test]
    fn a_taken_name_is_not_reused() {
        let directory = std::env::temp_dir().join(format!(
            "headless-planner-record-test-{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&directory);
        std::fs::create_dir_all(&directory).expect("a writable temp directory");

        let first = Telemetry::recording_path_in(&directory, "stamp").expect("a free name");
        assert!(first.ends_with("stamp.tlog"), "{}", first.display());
        std::fs::write(&first, b"").expect("writable");

        let second = Telemetry::recording_path_in(&directory, "stamp").expect("a free name");
        assert!(second.ends_with("stamp-1.tlog"), "{}", second.display());
        assert_ne!(first, second);

        let _ = std::fs::remove_dir_all(&directory);
    }

    /// Recordings go in this application's own data directory's `logs` (PLAN.md section 12, D11).
    ///
    /// Asserted through `mp_settings` rather than against a spelled-out path, because the right
    /// answer differs by platform and by `XDG_DATA_HOME` - what cannot differ
    /// is that it is the `logs` directory under the user data directory. A default of
    /// `~/Documents/MissionPlannerRust/logs` would end the same way; the equality is what catches
    /// it.
    #[test]
    fn flights_are_recorded_where_mission_planner_looks_for_them() {
        // With no `logdirectory` in config.xml. The real file on the machine running this can
        // name one - the C#'s Planner page writes the key when a pilot picks a folder
        // (`ConfigPlanner.cs:792`) - and that one wins, as the next test proves.
        let directory = Telemetry::log_directory_from(None);
        let Some(expected) = mp_settings::default_log_directory() else {
            // No home directory: the fallback, which must still not be the working directory.
            assert!(directory.is_absolute(), "{}", directory.display());
            return;
        };
        assert_eq!(directory, expected);
        assert!(
            directory.ends_with("MissionPlannerRust/logs"),
            "{}",
            directory.display()
        );
    }

    /// A `logdirectory` the operator chose in Mission Planner wins over the default.
    #[test]
    fn a_log_directory_chosen_in_mission_planner_is_used() {
        let mut config = mp_settings::Config::default();
        config.set("logdirectory", "/mnt/flights/logs");
        assert_eq!(
            Telemetry::log_directory_from(Some(&config)),
            std::path::PathBuf::from("/mnt/flights/logs")
        );
        // An empty key is "not chosen", as the C# treats it.
        config.set("logdirectory", "");
        assert_eq!(
            Telemetry::log_directory_from(Some(&config)),
            Telemetry::log_directory_from(None)
        );
    }

    /// Ninety-nine is the cap, and past it the answer is "no recording" rather than a hang.
    #[test]
    fn the_search_for_a_free_name_is_bounded() {
        let directory = std::env::temp_dir().join(format!(
            "headless-planner-record-full-{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&directory);
        std::fs::create_dir_all(&directory).expect("a writable temp directory");

        std::fs::write(directory.join("full.tlog"), b"").expect("writable");
        for index in 1..100 {
            std::fs::write(directory.join(format!("full-{index}.tlog")), b"").expect("writable");
        }
        assert!(Telemetry::recording_path_in(&directory, "full").is_none());

        let _ = std::fs::remove_dir_all(&directory);
    }
}

/// What the screens write into the vehicle's state, and the fence handed to the quick view, go
/// through the real link (PLAN.md §13.4 row 39).
#[cfg(test)]
mod state_wiring {
    use mp_link::ProtocolTimeouts;
    use mp_mavlink_dialects::all::{GlobalPositionInt, MavMessage, MissionCount, MissionItemInt};
    use mp_vehicle::StreamRates;

    use super::scripted::{Vehicle, until};

    /// `MAV_MISSION_TYPE_FENCE`.
    const FENCE: u8 = 1;

    /// Set Home Alt writes `cs.altoffsethome` on the vehicle's state, and every altitude shown
    /// reads it from there. `// C#: GCSViews/FlightData.cs:1236-1247`
    #[test]
    fn set_home_alt_writes_the_vehicles_own_offset() {
        let (telemetry, _vehicle) = Vehicle::connect(ProtocolTimeouts::default());
        assert_eq!(crate::fly::alt_offset_home(&telemetry.view()), 0.0);
        telemetry.set_alt_offset_home(-584.0);
        until("the state to carry it", || {
            crate::fly::alt_offset_home(&telemetry.view()) == -584.0
        });
        telemetry.set_alt_offset_home(0.0);
        until("the state to drop it", || {
            crate::fly::alt_offset_home(&telemetry.view()) == 0.0
        });
    }

    /// The Planner page's rates reach the saved defaults and the shown vehicle's `cs.rateX` when
    /// a combo changes them, and not before.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:573-640`
    #[test]
    fn the_planner_rates_reach_the_vehicle_and_the_saved_defaults() {
        let saved = StreamRates::backups();
        let (mut telemetry, _vehicle) = Vehicle::connect(ProtocolTimeouts::default());
        let rates = |telemetry: &super::Telemetry| telemetry.view().state.map(|state| state.rates);
        assert_eq!(rates(&telemetry), Some(saved));
        // The page as it starts: nothing to hand over.
        let start = StreamRates {
            attitude: 9,
            ..saved
        };
        telemetry.hand_over_rates(start);
        std::thread::sleep(std::time::Duration::from_millis(60));
        assert_eq!(rates(&telemetry), Some(saved));
        assert_eq!(StreamRates::backups(), saved);
        // A combo changed.
        let chosen = StreamRates {
            attitude: 10,
            rc: 5,
            ..saved
        };
        telemetry.hand_over_rates(chosen);
        assert_eq!(StreamRates::backups(), chosen);
        until("the vehicle's rates", || rates(&telemetry) == Some(chosen));
        StreamRates::set_backups(saved);
    }

    /// The fence a download shows the link is the one the quick view's `GeoFenceDist` measures
    /// from once it is handed over, and no fence reads 99999.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1617-1753`
    #[test]
    fn the_fence_the_link_saw_is_the_one_the_quick_view_measures_from() {
        let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default());
        vehicle.send(&MavMessage::GlobalPositionInt(GlobalPositionInt {
            time_boot_ms: 0,
            lat: -353_632_620,
            lon: 1_491_652_370,
            alt: 600_000,
            relative_alt: 0,
            vx: 0,
            vy: 0,
            vz: 0,
            hdg: 0,
        }));
        until("a position", || {
            telemetry
                .view()
                .state
                .is_some_and(|state| state.position.is_some())
        });
        let state = *telemetry.view().state.unwrap();
        crate::quick::set_fence(telemetry.fence_points());
        assert_eq!(crate::quick::value("GeoFenceDist", &state), Some(99999.0));

        vehicle.send(&MavMessage::MissionCount(MissionCount {
            count: 3,
            target_system: 255,
            target_component: 190,
            mission_type: FENCE,
        }));
        let corners = [
            (-353_600_000, 1_491_600_000),
            (-353_700_000, 1_491_600_000),
            (-353_650_000, 1_491_700_000),
        ];
        for (seq, (x, y)) in (0u16..).zip(corners) {
            vehicle.send(&MavMessage::MissionItemInt(MissionItemInt {
                param1: 3.0,
                param2: 0.0,
                param3: 0.0,
                param4: 0.0,
                x,
                y,
                z: 0.0,
                seq,
                command: 5001,
                target_system: 255,
                target_component: 190,
                frame: 3,
                current: 0,
                autocontinue: 1,
                mission_type: FENCE,
            }));
        }
        until("the fence", || telemetry.fence_points().len() == 3);
        let fence = telemetry.fence_points();
        crate::quick::set_fence(fence.clone());
        let shown = crate::quick::value("GeoFenceDist", &state).unwrap();
        assert_eq!(shown, f64::from(state.geo_fence_dist(&fence)));
        assert!(shown > 0.0 && shown < 99999.0, "{shown}");
        crate::quick::set_fence(Vec::new());
    }

    /// `tests/gui/state-wired.gui` asserts only facts [`super::Telemetry::facts`] publishes and
    /// binds quick views only to properties the chooser offers.
    #[test]
    fn the_state_script_asks_for_what_is_published() {
        let script = include_str!("../../../tests/gui/state-wired.gui");
        let published: Vec<&str> = super::Telemetry::idle()
            .facts(&super::TelemetryView::disconnected(""))
            .into_iter()
            .map(|(key, _)| key)
            .collect();
        let choices = crate::quick::choices();
        let mut facts = 0;
        let mut chosen = 0;
        for line in script.lines() {
            let words: Vec<&str> = line.split_whitespace().collect();
            match words.as_slice() {
                ["expect", key, ..]
                    if key.starts_with("vehicle.") && *key != "vehicle.connected" =>
                {
                    assert!(published.contains(key), "{key} is not published");
                    facts += 1;
                }
                ["click", id] => {
                    if let Some(name) = id.strip_prefix("fly-quick-choice-") {
                        assert!(choices.contains(&name), "{name} is not offered");
                        chosen += 1;
                    }
                }
                _ => {}
            }
        }
        assert!(facts >= 8 && chosen == 3, "{facts} facts, {chosen} choices");
    }
}
