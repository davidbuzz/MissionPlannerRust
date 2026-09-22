//! Tracking several vehicles on one link.
//!
//! Replaces `MAVList`. A single telemetry link commonly carries more than one MAVLink system:
//! the autopilot, a companion computer, a gimbal, an ADS-B receiver, other aircraft in a swarm.
//! Treating the link as one vehicle - which parts of Mission Planner effectively do - is why
//! telemetry from a second system can overwrite the first.

use std::collections::BTreeMap;

use mp_mavlink_dialects::all::MavMessage;

use crate::snapshot::{StateHandle, StatePublisher};
use crate::state::VehicleState;

/// Identifies one MAVLink system/component pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VehicleId {
    /// System id.
    pub sysid: u8,
    /// Component id.
    pub compid: u8,
}

impl VehicleId {
    /// Creates an id.
    #[must_use]
    pub const fn new(sysid: u8, compid: u8) -> Self {
        Self { sysid, compid }
    }
}

impl std::fmt::Display for VehicleId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}:{}", self.sysid, self.compid)
    }
}

/// Every vehicle seen on a link.
#[derive(Debug, Default)]
pub struct VehicleRegistry {
    vehicles: BTreeMap<VehicleId, StatePublisher>,
}

impl VehicleRegistry {
    /// Creates an empty registry.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Applies a frame to the right vehicle, creating it on first sight.
    ///
    /// Returns the id so the caller can notice new arrivals.
    pub fn apply(&mut self, sysid: u8, compid: u8, seq: u8, message: &MavMessage) -> VehicleId {
        let id = VehicleId::new(sysid, compid);
        let publisher = self
            .vehicles
            .entry(id)
            .or_insert_with(|| StatePublisher::new(VehicleState::new(sysid, compid)));

        publisher.working.link.record(seq);
        publisher.working.apply(message);
        id
    }

    /// Publishes snapshots for every vehicle.
    ///
    /// Called on a cadence by the link thread rather than per packet: at 200 Hz of telemetry,
    /// publishing per packet would be pure overhead, since no display can show more than one
    /// state per frame.
    pub fn publish_all(&mut self) {
        for publisher in self.vehicles.values_mut() {
            publisher.publish();
        }
    }

    /// A reader handle for one vehicle.
    #[must_use]
    pub fn handle(&self, id: VehicleId) -> Option<StateHandle> {
        self.vehicles.get(&id).map(StatePublisher::handle)
    }

    /// Every known vehicle id, in a stable order.
    #[must_use]
    pub fn ids(&self) -> Vec<VehicleId> {
        self.vehicles.keys().copied().collect()
    }

    /// How many vehicles are known.
    #[must_use]
    pub fn len(&self) -> usize {
        self.vehicles.len()
    }

    /// Whether no vehicle has been seen.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.vehicles.is_empty()
    }

    /// Read-only access to a vehicle's working state, for tests and diagnostics.
    #[must_use]
    pub fn working(&self, id: VehicleId) -> Option<&VehicleState> {
        self.vehicles.get(&id).map(|p| &p.working)
    }
}
