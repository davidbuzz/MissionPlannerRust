// Copyright (C) 2026 David "Buzz" Bussenschutt
//
// This file is part of MissionPlannerRust, a Rust implementation derived from
// Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,
// https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.
//
// MissionPlannerRust is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, version 3 of the License.
//
// MissionPlannerRust is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// MissionPlannerRust. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-only

//! Tracking several vehicles on one link.
//!
//! Replaces `MAVList`. A single telemetry link commonly carries more than one MAVLink system:
//! the autopilot, a companion computer, a gimbal, an ADS-B receiver, other aircraft in a swarm.
//! Treating the link as one vehicle - which parts of Mission Planner effectively do - is why
//! telemetry from a second system can overwrite the first.

use std::collections::BTreeMap;

use mp_mavlink_dialects::all::MavMessage;

use crate::clock::DateTime;
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
#[derive(Debug)]
pub struct VehicleRegistry {
    vehicles: BTreeMap<VehicleId, StatePublisher>,
    /// Whether a `NAMED_VALUE_FLOAT` reaches every component of its system, the C#'s
    /// `propagateNamedFloats` setting - true unless config.xml says otherwise; nothing in the
    /// C#'s screens sets it.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2283-2285`
    pub propagate_named_floats: bool,
}

impl Default for VehicleRegistry {
    fn default() -> Self {
        Self {
            vehicles: BTreeMap::new(),
            propagate_named_floats: true,
        }
    }
}

impl VehicleRegistry {
    /// Creates an empty registry.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Applies a frame to the right vehicle, creating it on first sight, without moving its
    /// clock: [`VehicleRegistry::apply_at`] for a link that knows when the frame arrived.
    ///
    /// Returns the id so the caller can notice new arrivals.
    pub fn apply(&mut self, sysid: u8, compid: u8, seq: u8, message: &MavMessage) -> VehicleId {
        self.apply_with_clock(sysid, compid, seq, message, None)
    }

    /// Applies a frame that arrived at `datetime`: the sender's
    /// [`VehicleState::datetime`] is set to it first, as `MAVLinkInterface` stamps
    /// `CurrentState.datetime` before a packet is processed - with `DateTime.Now` on a live link,
    /// which [`crate::DateTime::now`] is, and with the recorded time when a `.tlog` is played,
    /// which [`crate::DateTime::from_tlog_micros`] reads.
    ///
    /// **Divergence:** on a live link the C# stamps the selected vehicle (`MAV.cs`), not the
    /// sender, so every other vehicle's clock stays at `DateTime.MinValue` and none of its time
    /// in air, distance or vertical speed ever counts; here each vehicle's clock is its own
    /// packets', as the C# has it for a log.
    ///
    /// A vehicle seen for the first time starts counting seconds from this frame's time, as the
    /// C#'s `lastsecondcounter` starts at `DateTime.Now` when the state is made.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4710, 6613;
    /// ExtLibs/ArduPilot/CurrentState.cs:128`
    pub fn apply_at(
        &mut self,
        sysid: u8,
        compid: u8,
        seq: u8,
        message: &MavMessage,
        datetime: DateTime,
    ) -> VehicleId {
        self.apply_with_clock(sysid, compid, seq, message, Some(datetime))
    }

    fn apply_with_clock(
        &mut self,
        sysid: u8,
        compid: u8,
        seq: u8,
        message: &MavMessage,
        datetime: Option<DateTime>,
    ) -> VehicleId {
        let id = VehicleId::new(sysid, compid);
        let publisher = self.vehicles.entry(id).or_insert_with(|| {
            let mut state = VehicleState::new(sysid, compid);
            if let Some(datetime) = datetime {
                state.last_second_counter = datetime;
            }
            StatePublisher::new(state)
        });

        if let Some(datetime) = datetime {
            publisher.working.datetime = datetime;
        }
        publisher.working.link.record(seq);
        publisher.working.apply(message);

        // A SiK radio reports as its own system, and what it says is about the link every
        // vehicle on it shares, so the C# lets every vehicle's state take it. Sequence numbers
        // stay with the sender: they measure its stream, not anyone else's. A named value goes
        // to every component of its own system, unless the setting says not to.
        // C#: ExtLibs/ArduPilot/CurrentState.cs:2280-2285
        let within_system = match message {
            MavMessage::Radio(_) | MavMessage::RadioStatus(_) => None,
            MavMessage::NamedValueFloat(_) if self.propagate_named_floats => Some(sysid),
            _ => return id,
        };
        for (other, publisher) in &mut self.vehicles {
            if *other != id && within_system.is_none_or(|system| other.sysid == system) {
                publisher.working.apply(message);
            }
        }
        id
    }

    /// `UpdateCurrentSettings` on every vehicle, as the C# does after each read from the link
    /// (`MainV2.cs:3065-3076`) and after each packet of a log (`Log/MavlinkLog.cs:141-144`); see
    /// [`VehicleState::update_current_settings`] for `link_closed`.
    pub fn update_current_settings(&mut self, link_closed: bool) {
        for publisher in self.vehicles.values_mut() {
            publisher.working.update_current_settings(link_closed);
        }
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

    /// The working state of a vehicle, for what the C# writes into `CurrentState` from outside
    /// it: the parameters the low-airspeed warning reads
    /// ([`VehicleState::set_airspeed_min_params`]), and the fields the flight screen and the
    /// planner set - [`VehicleState::alt_offset_home`], [`VehicleState::base`],
    /// [`VehicleState::gimbal_point`], [`VehicleState::time_since_last_shot`] and
    /// [`VehicleState::rates`]. The next publish carries the change.
    pub fn working_mut(&mut self, id: VehicleId) -> Option<&mut VehicleState> {
        self.vehicles.get_mut(&id).map(|p| &mut p.working)
    }
}
