//! Bridges the link engine to the UI.
//!
//! The UI must never touch the link's internals: it reads an immutable snapshot per frame and
//! nothing else. That is the whole point of the snapshot bus in `mp-vehicle` - a render pass
//! cannot block on I/O, and cannot observe a half-updated vehicle.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::Arc;

use mp_link::messages::LogMessage;
use mp_link::mission_transfer::TransferState;
use mp_link::{Link, LinkConfig, commands};
use mp_mission::MissionItem;
use mp_units::LatLon;
use mp_vehicle::{VehicleFamily, VehicleId, VehicleState};

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
    /// Recent `STATUSTEXT` and `COMMAND_ACK` lines, newest last.
    pub messages: Vec<LogMessage>,
    /// How many messages the link had to discard to stay bounded.
    pub messages_dropped: u64,
    /// What the mission transfer is doing, if one has been started.
    pub transfer: Option<TransferStatus>,
    /// The vehicle's parameters, by name.
    pub parameters: Vec<(String, f64)>,
    /// How many the vehicle says it has, once it has said.
    pub parameters_expected: u16,
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
            messages: Vec::new(),
            messages_dropped: 0,
            transfer: None,
            parameters: Vec::new(),
            parameters_expected: 0,
        }
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

    /// Where flights are recorded when nothing says otherwise.
    ///
    /// The directory Mission Planner itself records into and browses, `Settings.GetDefaultLogDir`:
    /// the user data directory plus `logs`. Sharing it is the point - the two programs see one set
    /// of flights, and a recording made by either is found by both. That matters more than it
    /// sounds: the reason to open a tlog is usually to answer a question about a flight, and a
    /// pilot who has both installed should not have to remember which one was connected.
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
                link: Some(link),
                target: url.to_owned(),
                error: None,
                selected: None,
                banner_requested: std::collections::BTreeSet::new(),
                banner: None,
                recording,
            },
            Err(err) => Self {
                link: None,
                target: url.to_owned(),
                error: Some(err.to_string()),
                selected: None,
                banner_requested: std::collections::BTreeSet::new(),
                banner: None,
                recording: None,
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
        let parameters = primary.as_ref().and_then(|(id, _)| link.params(*id));
        let transfer = primary
            .as_ref()
            .and_then(|(id, _)| link.mission_transfer(*id));
        let mission = transfer
            .as_ref()
            .map(|transfer| transfer.items().to_vec())
            .unwrap_or_default();
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
            messages: link.recent_messages(MESSAGE_LINES),
            messages_dropped: link.messages_dropped(),
            transfer,
            parameters: parameters
                .as_ref()
                .map(|table| {
                    table
                        .iter()
                        .map(|(name, value)| (name.clone(), value.as_f64()))
                        .collect()
                })
                .unwrap_or_default(),
            parameters_expected: parameters
                .as_ref()
                .and_then(mp_params::ParamTable::expected)
                .unwrap_or(0),
        }
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

    /// Arms or disarms, with the vehicle's pre-arm checks applied.
    pub fn arm(&self, arm: bool) {
        if let Some((link, id)) = self.target() {
            link.send(&commands::arm(id, arm, false));
        }
    }

    /// Starts a six-position accelerometer calibration.
    pub fn start_accelerometer_calibration(&self) {
        if let Some((link, id)) = self.target() {
            link.clear_accel_calibration();
            link.send(&mp_calibration::start_accelerometer(id));
        }
    }

    /// Tells the vehicle the airframe is in the position it asked for.
    pub fn confirm_accelerometer_position(&self, position: mp_calibration::AccelPosition) {
        if let Some((link, id)) = self.target() {
            link.send(&mp_calibration::accelerometer_position_reached(
                id, position,
            ));
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

    /// Tells the vehicle that however it is sitting now is level.
    pub fn calibrate_level(&self) {
        if let Some((link, id)) = self.target() {
            link.send(&mp_calibration::level(id));
        }
    }

    /// Starts an onboard compass calibration.
    pub fn calibrate_compass(&self) {
        if let Some((link, id)) = self.target() {
            link.clear_compass_calibration();
            link.send(&mp_calibration::start_compass(id));
        }
    }

    /// Compass calibration progress, one entry per compass.
    #[must_use]
    pub fn compass_calibration(&self) -> Vec<mp_calibration::CompassProgress> {
        self.link
            .as_ref()
            .map(Link::compass_calibration)
            .unwrap_or_default()
    }

    /// Stops a running compass calibration.
    pub fn cancel_compass_calibration(&self) {
        if let Some((link, id)) = self.target() {
            link.send(&mp_calibration::cancel_compass(id));
        }
    }

    /// Forgets compass calibration progress.
    pub fn clear_compass_calibration(&self) {
        if let Some(link) = &self.link {
            link.clear_compass_calibration();
        }
    }

    /// Recalibrates the barometer's ground pressure reference.
    pub fn calibrate_ground_pressure(&self) {
        if let Some((link, id)) = self.target() {
            link.send(&mp_calibration::ground_pressure(id));
        }
    }

    /// Spins one motor briefly, at a bounded throttle and with a timeout.
    pub fn test_motor(&self, motor: u8, throttle_percent: f32) {
        if let Some((link, id)) = self.target() {
            link.send(&mp_calibration::test_motor(id, motor, throttle_percent));
        }
    }

    /// Stops a running motor test.
    pub fn stop_motor(&self, motor: u8) {
        if let Some((link, id)) = self.target() {
            link.send(&mp_calibration::stop_motor(id, motor));
        }
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

    /// Other aircraft currently known about.
    #[must_use]
    pub fn traffic(&self) -> Vec<mp_link::traffic::Traffic> {
        self.link.as_ref().map(Link::traffic).unwrap_or_default()
    }

    /// Starts a parameter download.
    pub fn download_parameters(&self) {
        if let Some((link, id)) = self.target() {
            link.download_params(id);
        }
    }

    /// Writes one parameter.
    pub fn set_parameter(&self, name: &str, value: f32) {
        if let Some((link, id)) = self.target() {
            link.send(&commands::param_set(id, name, value));
        }
    }

    /// Reboots the autopilot.
    ///
    /// The link drops when the vehicle obeys, which is what success looks like. Useful after a
    /// calibration, and the usual first thing to try when a board is behaving oddly.
    pub fn reboot(&self) {
        if let Some((link, id)) = self.target() {
            link.send(&commands::reboot(id));
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
    /// Cheap - two messages - and the answer says which firmware generation this is without
    /// downloading a thousand parameters to find out.
    pub fn probe_arming_check_param(&self) {
        if let Some((link, id)) = self.target() {
            link.send(&commands::request_param_by_name(id, ARMING_SKIPCHK));
            link.send(&commands::request_param_by_name(id, ARMING_CHECK));
        }
    }

    /// Turns the vehicle's arming checks off.
    ///
    /// Writes whichever parameter this vehicle actually has. Until that is known it asks for both
    /// and writes both: the one that does not exist is ignored, and the caller retries, by which
    /// time the answer has arrived and the right one is written on its own.
    ///
    /// These are persistent parameters, so they stay off until something sets them back - which is
    /// the behaviour asked for, and why the caller says so on screen rather than doing it quietly.
    pub fn disable_arming_checks(&self) {
        self.write_arming_checks(SKIP_ALL_CHECKS, 0.0);
    }

    /// Restores the vehicle's arming checks to the firmware defaults.
    ///
    /// The defaults differ with the name: nothing skipped for the new parameter, everything
    /// checked for the old one. Writing the wrong default would be worse than writing nothing.
    pub fn enable_arming_checks(&self) {
        self.write_arming_checks(0.0, 1.0);
    }

    /// Writes whichever arming-check parameter this vehicle has.
    ///
    /// `skipchk` is the value for the 4.7-and-later name, `legacy` the value for the older one.
    /// They are different numbers for the same intent, because the sense was inverted along with
    /// the rename.
    ///
    /// Until the vehicle has said which it has, both are written and both are asked for. The one
    /// that does not exist is ignored, and the caller retries - by which time the answer has
    /// arrived and only the right one is written.
    fn write_arming_checks(&self, skipchk: f32, legacy: f32) {
        let Some((link, id)) = self.target() else {
            return;
        };
        match self.arming_check_param() {
            Some(name) if name == ARMING_SKIPCHK => {
                link.send(&commands::param_set(id, ARMING_SKIPCHK, skipchk));
            }
            Some(_) => {
                link.send(&commands::param_set(id, ARMING_CHECK, legacy));
            }
            None => {
                self.probe_arming_check_param();
                link.send(&commands::param_set(id, ARMING_SKIPCHK, skipchk));
                link.send(&commands::param_set(id, ARMING_CHECK, legacy));
            }
        }
    }

    /// Arms with the checks bypassed.
    ///
    /// Two things, because one is not enough. The magic 21196 in `MAV_CMD_COMPONENT_ARM_DISARM`
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
    pub fn force_arm(&self) {
        self.disable_arming_checks();
        if let Some((link, id)) = self.target() {
            link.send(&commands::arm(id, true, true));
        }
    }

    /// Changes flight mode.
    pub fn set_mode(&self, custom_mode: u32) {
        if let Some((link, id)) = self.target() {
            link.send(&commands::set_mode(id, custom_mode));
        }
    }

    /// Takes off to the given height above home.
    pub fn takeoff(&self, altitude_metres: f32) {
        if let Some((link, id)) = self.target() {
            link.send(&commands::takeoff(id, altitude_metres));
        }
    }

    /// Lands where the vehicle is.
    pub fn land(&self) {
        if let Some((link, id)) = self.target() {
            link.send(&commands::land(id));
        }
    }

    /// Flies to a position at the given height, in Guided.
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
    /// Mission Planner sends `DO_SEND_BANNER` at connect and for each new vehicle, and takes the
    /// firmware version from the `STATUSTEXT` that names the vehicle; the parameter
    /// documentation for that release is fetched from it.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:930, 1822-1830, 1856`
    pub fn tick(&mut self) {
        let Some(link) = &self.link else {
            return;
        };
        if let Some((id, _)) = link.primary_vehicle()
            && !self.banner_requested.contains(&id)
        {
            link.send(&commands::send_banner(id));
            self.banner_requested.insert(id);
        }
        if self.banner.is_none() {
            let messages = link.recent_messages(MESSAGE_LINES);
            self.banner = crate::metadata::banner_in(messages.iter().map(|m| m.text.as_str()))
                .map(str::to_owned);
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
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let directory =
            std::env::temp_dir().join(format!("mpr-record-test-{}", std::process::id()));
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

    /// Recordings go where Mission Planner looks for them, whichever rule this platform takes.
    ///
    /// Asserted through `mp_settings` rather than against a spelled-out path, because the right
    /// answer differs by platform and by whether an old installation exists - what cannot differ
    /// is that it is the C# application's own `logs` directory. The previous default,
    /// `~/Documents/Mission Planner/logs`, ends the same way; the equality is what catches it.
    #[test]
    fn flights_are_recorded_where_mission_planner_looks_for_them() {
        let directory = Telemetry::log_directory();
        let Some(expected) = mp_settings::default_log_directory() else {
            // No home directory: the fallback, which must still not be the working directory.
            assert!(directory.is_absolute(), "{}", directory.display());
            return;
        };
        assert_eq!(directory, expected);
        assert!(
            directory.ends_with("Mission Planner/logs"),
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
        let directory =
            std::env::temp_dir().join(format!("mpr-record-full-{}", std::process::id()));
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
