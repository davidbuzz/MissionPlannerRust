//! Bridges the link engine to the UI.
//!
//! The UI must never touch the link's internals: it reads an immutable snapshot per frame and
//! nothing else. That is the whole point of the snapshot bus in `mp-vehicle` - a render pass
//! cannot block on I/O, and cannot observe a half-updated vehicle.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::Arc;

use mp_link::{Link, LinkConfig};
use mp_mission::MissionItem;
use mp_vehicle::{VehicleId, VehicleState};

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
        let mission = primary
            .as_ref()
            .and_then(|(id, _)| link.mission_transfer(*id))
            .map(|transfer| transfer.items().to_vec())
            .unwrap_or_default();

        TelemetryView {
            target: link.description(),
            connected: link.is_running(),
            vehicle: primary.as_ref().map(|(id, _)| *id),
            state: primary.map(|(_, handle)| handle.load()),
            frames: link.frames_received(),
            crc_errors: stats.decode.crc_errors,
            vehicle_count: vehicles.len(),
            mission,
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
}
