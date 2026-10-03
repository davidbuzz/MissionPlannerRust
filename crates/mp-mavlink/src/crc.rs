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

//! CRC-16/MCRF4XX, the checksum MAVLink calls "X.25".
//!
//! Ported from `ExtLibs/Mavlink/MavlinkCRC.cs`. The bit manipulation is reproduced exactly,
//! including the 8-bit truncation the C# `unchecked` block relies on, because any divergence
//! silently drops every packet on a real link.

/// Initial CRC accumulator value (`X25_INIT_CRC` in the C# original).
pub const INIT: u16 = 0xFFFF;

/// Folds one byte into the accumulator.
#[inline(always)]
#[must_use]
// The truncation to u8 is the algorithm, matching the C# `(byte)(crc & 0x00ff)`.
#[allow(clippy::cast_possible_truncation)]
pub const fn accumulate(byte: u8, crc: u16) -> u16 {
    let ch = byte ^ (crc as u8);
    let ch = ch ^ (ch << 4);
    let ch = ch as u16;
    (crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4)
}

/// Folds a slice into the accumulator.
#[inline]
#[must_use]
pub fn accumulate_slice(bytes: &[u8], mut crc: u16) -> u16 {
    for &b in bytes {
        crc = accumulate(b, crc);
    }
    crc
}

/// Computes a frame checksum: the framed bytes (excluding STX and the checksum itself)
/// followed by the message's `CRC_EXTRA` seed.
#[inline]
#[must_use]
pub fn checksum(bytes: &[u8], crc_extra: u8) -> u16 {
    accumulate(crc_extra, accumulate_slice(bytes, INIT))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The canonical CRC-16/MCRF4XX check value for the ASCII string "123456789".
    #[test]
    fn known_check_value() {
        assert_eq!(accumulate_slice(b"123456789", INIT), 0x6F91);
    }

    #[test]
    fn empty_input_is_init() {
        assert_eq!(accumulate_slice(b"", INIT), INIT);
    }

    /// A HEARTBEAT (msgid 0, crc_extra 50) with a known payload, taken from a real link.
    #[test]
    fn heartbeat_checksum_is_stable() {
        // len, seq, sysid, compid, msgid, payload(9)
        let framed = [9u8, 0, 1, 1, 0, 0, 0, 0, 0, 6, 8, 0, 0, 3];
        // Regression value cross-checked against an independent reimplementation of
        // ExtLibs/Mavlink/MavlinkCRC.cs, so a "harmless" refactor of accumulate() cannot pass.
        assert_eq!(checksum(&framed, 50), 0x20F1);
    }
}
