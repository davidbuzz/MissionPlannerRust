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

//! Decoded vehicle state and the telemetry snapshot bus.
//!
//! Replaces `ExtLibs/ArduPilot/CurrentState.cs`, `MAVState` and `MAVList`.
//!
//! # The problem with how the C# does it
//!
//! In Mission Planner the UI binds directly to a mutable `CurrentState` that the telemetry thread
//! writes to, with locks and `Invoke` marshalling scattered around it. A form can therefore read
//! a half-updated position - latitude from one packet, longitude from the next - and the render
//! path can block behind the I/O thread.
//!
//! # What this does instead
//!
//! The link thread owns a private [`VehicleState`] and mutates it per packet, allocation-free.
//! Periodically it *publishes* an immutable snapshot. Readers take a snapshot with one atomic
//! load, never block, and always observe a state that existed at a single instant. Snapshots are
//! recycled through a small pool, so steady-state publishing does not allocate either - there is
//! a test that asserts exactly that.

#![forbid(unsafe_code)]

pub mod clock;
pub mod coverage;
pub mod fence;
pub mod health;
pub mod link_quality;
/// Flight mode names, generated from Mission Planner's parameter metadata.
#[path = "generated/modes.rs"]
pub mod modes;
pub mod mode_lookup;
pub mod onboard;
pub mod proximity;
pub mod rc;
pub mod registry;
pub mod sensors;
pub mod snapshot;
pub mod state;
pub mod statics;
pub mod units;
pub mod update;

pub use clock::DateTime;
pub use fence::FenceItem;
pub use link_quality::{LinkQuality, Radio};
pub use modes::{VehicleFamily, flight_mode_name};
pub use rc::RcChannels;
pub use registry::{VehicleId, VehicleRegistry};
pub use sensors::Sensors;
pub use snapshot::{StateHandle, StatePublisher};
pub use state::{Attitude, Battery, GpsInfo, LatLngAlt, Nav, Rangefinder, Terrain, VehicleState};
pub use statics::StreamRates;
