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

//! Firmware files and the px4 bootloader protocol.
//!
//! Ported from `ExtLibs/px4uploader/` @ efb0801 (GPL-3.0-only). This is the one path in the
//! application with no simulator: a wrong byte here does not produce a wrong reading, it produces a
//! board that will not boot. PLAN.md R9 rates it the only *fatal*-impact risk with no SITL
//! equivalent, and Deliverable 13 requires the byte protocol to be proven against a mock bootloader before
//! any real board is touched.
//!
//! So the shape of this crate is: everything that can be a pure function is one, the protocol is
//! driven over a `Read + Write` so a test can be the other end of it, and nothing here opens a
//! serial port by itself.
//!
//! Three details are transliterated rather than improved, because the bootloader on the other end
//! is not going to change to suit us. Each is marked where it appears.

pub mod detect;
pub mod firmware;
pub mod flow;
pub mod github;
pub mod legacy;
pub mod manifest;
pub mod protocol;
pub mod signed;
pub mod uploader;

pub use detect::{Boards, Detected, DeviceInfo, detect_board, match_ports};
pub use firmware::Firmware;
pub use protocol::{Code, Info};
pub use uploader::{Uploader, UploaderError};
