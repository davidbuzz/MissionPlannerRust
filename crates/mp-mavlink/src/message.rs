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

//! The trait generated message types implement.

/// A typed MAVLink message.
///
/// Implementations are generated from the XML definitions by `cargo xtask codegen mavlink`;
/// the metadata constants are verified against the shipping C# table.
pub trait Message: Sized {
    /// Message id.
    const ID: u32;
    /// Message name as written in the XML definition.
    const NAME: &'static str;
    /// `CRC_EXTRA` seed for this message.
    const CRC_EXTRA: u8;
    /// Payload length excluding extension fields.
    const MIN_LEN: usize;
    /// Full payload length including extension fields.
    const LEN: usize;

    /// Decodes from a payload, zero-extending a v2-truncated one.
    fn decode(payload: &[u8]) -> Self;

    /// Encodes into `out`, which must be at least [`Self::LEN`] bytes. Returns bytes written.
    fn encode(&self, out: &mut [u8]) -> usize;
}
