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

//! Generated MAVLink message types, metadata and enums.
//!
//! The contents of `generated/` are produced by `cargo xtask codegen mavlink` from
//! `references/missionplanner/ExtLibs/Mavlink/message_definitions/*.xml` - the same definitions
//! the C# build uses. They are checked in so building needs no Python, no network and no
//! reference tree.
//!
//! Correctness of the generator is not assumed: `cargo xtask verify-mavlink` checks every
//! message's `CRC_EXTRA`, `min_len` and `len` against the table dumped from the shipping
//! `MAVLink.dll`, and the test suite decodes real flight logs and compares field values.

/// The `all` dialect: every message Mission Planner knows about.
#[path = "generated/all.rs"]
pub mod all;

pub use all::{DIALECT, MESSAGES, MavMessage};
