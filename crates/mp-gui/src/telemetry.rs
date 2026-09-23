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
        }
    }
}

/// Owns the link and produces views of it.
#[derive(Debug)]
pub struct Telemetry {
    link: Option<Link>,
    target: String,
    error: Option<String>,
}

impl Telemetry {
    /// Opens a link. A failure here is shown in the UI rather than killing the process: a ground
    /// station that exits because a USB cable was not plugged in yet is useless in the field.
    #[must_use]
    pub fn connect(url: &str) -> Self {
        let config = LinkConfig::default();
        match Link::connect(url, config) {
            Ok(link) => Self {
                link: Some(link),
                target: url.to_owned(),
                error: None,
            },
            Err(err) => Self {
                link: None,
                target: url.to_owned(),
                error: Some(err.to_string()),
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
        }
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
        let primary = link.primary_vehicle();

        // Fetch the mission the link holds, if a download has finished. The UI never triggers
        // one itself: a ground station that silently pulls a mission whenever it connects makes
        // it impossible to tell whether what is on screen came from the vehicle or the operator.
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
    fn target(&self) -> Option<(&Link, VehicleId)> {
        let link = self.link.as_ref()?;
        let (id, _) = link.primary_vehicle()?;
        Some((link, id))
    }

    /// Arms or disarms.
    ///
    /// Never forced. `MAV_CMD_COMPONENT_ARM_DISARM` takes a magic 21196 in param2 that bypasses
    /// every pre-arm check, and a ground station that offers that behind an ordinary button is how
    /// aircraft take off with an uncalibrated compass. Forcing belongs behind its own deliberate
    /// control, which this is not.
    pub fn arm(&self, arm: bool) {
        if let Some((link, id)) = self.target() {
            link.send(&commands::arm(id, arm, false));
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
