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

//! `TransferCRC.cs`: CRC-16-CCITT (polynomial 0x1021, from 0xFFFF, no reflection), which a
//! multi-frame transfer carries in its first two bytes, computed over the data type's 64-bit
//! signature, little-endian, and then the payload.
//! `// C#: ExtLibs/DroneCAN/TransferCRC.cs:1-60; ExtLibs/DroneCAN/DroneCAN.cs:1456-1461, 1955-1981`

/// A CRC being worked out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransferCrc(u16);

impl Default for TransferCrc {
    fn default() -> Self {
        Self::new()
    }
}

impl TransferCrc {
    /// `value_ = 0xFFFF`.
    #[must_use]
    pub const fn new() -> Self {
        Self(0xffff)
    }

    /// `add(byte)`.
    pub fn add(&mut self, byte: u8) {
        self.0 ^= u16::from(byte) << 8;
        for _ in 0..8 {
            self.0 = if self.0 & 0x8000 != 0 {
                (self.0 << 1) ^ 0x1021
            } else {
                self.0 << 1
            };
        }
    }

    /// `add(bytes, len)`.
    pub fn add_bytes(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.add(*byte);
        }
    }

    /// `get()`.
    #[must_use]
    pub const fn get(&self) -> u16 {
        self.0
    }
}

/// `TransferCRC.compute(bytes, len)`.
#[must_use]
pub fn compute(bytes: &[u8]) -> u16 {
    let mut crc = TransferCrc::new();
    crc.add_bytes(bytes);
    crc.get()
}

/// A multi-frame transfer's CRC: the signature's eight bytes (`BitConverter.GetBytes`, little
/// endian), then the payload.
#[must_use]
pub fn transfer_crc(signature: u64, payload: &[u8]) -> u16 {
    let mut crc = TransferCrc::new();
    crc.add_bytes(&signature.to_le_bytes());
    crc.add_bytes(payload);
    crc.get()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `check()`: "123456789" is 0x29B1.
    /// `// C#: ExtLibs/DroneCAN/TransferCRC.cs:9-14`
    #[test]
    fn the_check_value() {
        assert_eq!(compute(b"123456789"), 0x29b1);
        let mut crc = TransferCrc::default();
        crc.add_bytes(b"1234");
        crc.add_bytes(b"56789");
        assert_eq!(crc.get(), 0x29b1);
    }
}
