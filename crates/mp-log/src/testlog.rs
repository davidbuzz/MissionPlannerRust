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

//! Building small dataflash logs by hand, for tests that need to know exactly what is in one.

/// One dataflash record: the two head bytes, the type, the payload.
pub(crate) fn record(msg_type: u8, payload: &[u8]) -> Vec<u8> {
    let mut out = vec![
        crate::dataflash::HEAD_BYTE1,
        crate::dataflash::HEAD_BYTE2,
        msg_type,
    ];
    out.extend_from_slice(payload);
    out
}

/// A fixed-width text field, zero padded as the firmware writes them.
pub(crate) fn fixed(text: &str, width: usize) -> Vec<u8> {
    let mut out = text.as_bytes().to_vec();
    out.resize(width, 0);
    out
}

/// An `FMT` record declaring a message type: `BBnNZ` = Type, Length, Name, Format, Columns.
pub(crate) fn fmt(msg_type: u8, length: u8, name: &str, format: &str, columns: &str) -> Vec<u8> {
    let mut payload = vec![msg_type, length];
    payload.extend(fixed(name, 4));
    payload.extend(fixed(format, 16));
    payload.extend(fixed(columns, 64));
    record(crate::dataflash::FMT_TYPE, &payload)
}
