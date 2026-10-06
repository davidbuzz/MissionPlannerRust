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

//! MAVLink v1/v2 frame codec.
//!
//! This crate owns the wire format only: framing, checksums and signing. Message *contents*
//! live in the generated dialect crate, so that this hot path stays small, auditable and
//! free of generated code.
//!
//! Design constraints, from `DELIVERABLES.md` Deliverable 2:
//!
//! * **Zero-copy parse.** [`frame::parse`] borrows the caller's buffer; payloads are never copied.
//! * **Allocation-free.** Nothing in this crate allocates. [`decoder::FrameDecoder`] owns a
//!   fixed-size buffer, so a link cannot grow memory under load or attack.
//! * **No panics.** Every slice access is length-checked first; the fuzz targets assert this.
//!
//! The C# original this replaces is `ExtLibs/Mavlink/MavlinkParse.cs` and `MavlinkCRC.cs`.

pub mod crc;
pub mod decoder;
pub mod dialect;
pub mod field;
pub mod frame;
pub mod message;
pub mod payload;
pub mod signing;

pub use decoder::{DecodeStats, FrameDecoder};
pub use dialect::{Dialect, MessageInfo, StaticDialect};
pub use field::{FieldInfo, FieldValue};
pub use frame::{
    EncodeError, Frame, INCOMPAT_FLAG_SIGNED, INCOMPAT_FLAG_SYSID32, INCOMPAT_FLAG_TARGET32,
    MAX_FRAME_LEN, MAX_PAYLOAD_LEN, MavVersion, ParseError, SIGNATURE_LEN, STX_V1, STX_V2,
    SUPPORTED_INCOMPAT_FLAGS, V2_HEADER_LEN, V2_MAX_HEADER_LEN, encode_v1, encode_v2,
    encode_v2_targeted, parse, trim_payload, v2_header_len, v2_layout,
};
pub use message::Message;
pub use signing::{SigningKey, sign, verify};
