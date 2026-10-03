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

//! libcanard v0's bit stream: how a scalar of any width from 1 to 64 bits is laid into a
//! transfer's payload and read back out.
//!
//! `canardEncodeScalar` stores the value in its little-endian bytes, shifts the last, partial
//! byte up so its bits sit at the top, and `copyBitArray` copies the bits most significant first:
//! so each whole byte of the value goes in from bit 7 to bit 0, low byte first, and then the
//! remaining `n % 8` bits, most significant first. `canardDecodeScalar` reverses it, and a read
//! past the payload's end gives zeros for the bits that are not there (`descatterTransferPayload`
//! copies only what there is, and the field starts at zero). The generated `encode` methods hand
//! each field to `dronecan_transmit_chunk_handler`, which appends it at the stream's bit count;
//! a `void` field is a `chunk_cb(null, n)`, which moves the count on over zeros.
//! `// C#: ExtLibs/DroneCAN/Canard.cs:114-208, 222-389; ExtLibs/DroneCAN/DroneCAN.cs:25-35`

/// A payload being written: `statetracking`'s bytes and bit count.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BitWriter {
    bytes: Vec<u8>,
    bits: usize,
}

impl BitWriter {
    /// An empty payload.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            bytes: Vec::new(),
            bits: 0,
        }
    }

    /// One bit, at the stream's end.
    fn push_bit(&mut self, set: bool) {
        let byte = self.bits / 8;
        if self.bytes.len() <= byte {
            self.bytes.push(0);
        }
        if set && let Some(slot) = self.bytes.get_mut(byte) {
            *slot |= 0x80 >> (self.bits % 8);
        }
        self.bits += 1;
    }

    /// `canardEncodeScalar(buffer, 0, len, value)` and `chunk_cb(buffer, len, ctx)`: the low
    /// `len` bits of `value` (1 to 64), whole bytes first, low byte first, each from its top bit.
    pub fn put(&mut self, value: u64, len: u32) {
        let len = len.min(64);
        let whole = len / 8;
        for index in 0..whole {
            let byte = (value >> (8 * index)) & 0xff;
            for bit in (0..8).rev() {
                self.push_bit(byte & (1 << bit) != 0);
            }
        }
        let rest = len % 8;
        if rest > 0 {
            let partial = (value >> (8 * whole)) & ((1 << rest) - 1);
            for bit in (0..rest).rev() {
                self.push_bit(partial & (1 << bit) != 0);
            }
        }
    }

    /// A signed value in `len` bits: its two's complement, cut to the width.
    pub fn put_signed(&mut self, value: i64, len: u32) {
        self.put(u64::from_ne_bytes(value.to_ne_bytes()), len);
    }

    /// A `bool` in one bit.
    pub fn put_bool(&mut self, value: bool) {
        self.put(u64::from(value), 1);
    }

    /// A `float32`: its bits, as `storage.f32` holds them.
    pub fn put_f32(&mut self, value: f32) {
        self.put(u64::from(value.to_bits()), 32);
    }

    /// `chunk_cb(null, len, ctx)`: a `void` field, `len` zero bits.
    pub fn skip(&mut self, len: u32) {
        for _ in 0..len {
            self.push_bit(false);
        }
    }

    /// How many bits have been written.
    #[must_use]
    pub const fn bit_len(&self) -> usize {
        self.bits
    }

    /// `statetracking.ToBytes()`: the bytes the bits fill, the last padded with zeros.
    #[must_use]
    pub fn into_bytes(mut self) -> Vec<u8> {
        self.bytes.truncate(self.bits.div_ceil(8));
        self.bytes
    }
}

/// A received transfer's payload being read: `CanardRxTransfer` and the decoder's `bit_ofs`.
#[derive(Debug, Clone)]
pub struct BitReader<'a> {
    bytes: &'a [u8],
    bit: u32,
}

impl<'a> BitReader<'a> {
    /// Reading `bytes` from the first bit.
    #[must_use]
    pub const fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, bit: 0 }
    }

    /// One bit, zero past the end.
    fn bit_at(&self, index: u32) -> bool {
        let byte = usize::try_from(index / 8).unwrap_or(usize::MAX);
        self.bytes
            .get(byte)
            .is_some_and(|value| value & (0x80 >> (index % 8)) != 0)
    }

    /// `canardDecodeScalar(transfer, bit_ofs, len, false, ref value)` and `bit_ofs += len`.
    pub fn get(&mut self, len: u32) -> u64 {
        let len = len.min(64);
        let mut value = 0u64;
        let whole = len / 8;
        let mut at = self.bit;
        for index in 0..whole {
            let mut byte = 0u64;
            for _ in 0..8 {
                byte = (byte << 1) | u64::from(self.bit_at(at));
                at += 1;
            }
            value |= byte << (8 * index);
        }
        let rest = len % 8;
        if rest > 0 {
            let mut partial = 0u64;
            for _ in 0..rest {
                partial = (partial << 1) | u64::from(self.bit_at(at));
                at += 1;
            }
            value |= partial << (8 * whole);
        }
        self.bit = self.bit.saturating_add(len);
        value
    }

    /// `canardDecodeScalar(..., true, ...)`: the sign bit carried up through the width.
    pub fn get_signed(&mut self, len: u32) -> i64 {
        let len = len.clamp(1, 64);
        let raw = self.get(len);
        let value = i64::from_ne_bytes(raw.to_ne_bytes());
        if len == 64 {
            return value;
        }
        let shift = 64 - len;
        (value << shift) >> shift
    }

    /// A `bool`.
    pub fn get_bool(&mut self) -> bool {
        self.get(1) != 0
    }

    /// A `float32`: `storage.f32` of the 32 bits.
    pub fn get_f32(&mut self) -> f32 {
        let raw = self.get(32);
        f32::from_bits(u32::try_from(raw).unwrap_or(0))
    }

    /// `bit_ofs += len`: a `void` field passed over.
    pub fn skip(&mut self, len: u32) {
        self.bit = self.bit.saturating_add(len);
    }

    /// `bit_ofs`.
    #[must_use]
    pub const fn bit(&self) -> u32 {
        self.bit
    }

    /// A tail array's length, as the decoders work it out under tail array optimisation:
    /// `(uint8_t)(((transfer.payload_len*8)-bit_ofs)/8)` - unsigned 32-bit arithmetic, cut to the
    /// length field's C# type (a byte for a length of up to eight bits, `uint16_t` for nine), so a
    /// field that runs past the end wraps as the C#'s does.
    #[must_use]
    pub fn tail_len(&self, len_bits: u32) -> usize {
        let payload_bits = u32::try_from(self.bytes.len())
            .unwrap_or(u32::MAX)
            .wrapping_mul(8);
        let len = payload_bits.wrapping_sub(self.bit) / 8;
        if len_bits <= 8 {
            usize::from(low_byte(len))
        } else {
            usize::from(u16::from_le_bytes([low_byte(len), low_byte(len >> 8)]))
        }
    }

    /// `n` bytes, each eight bits: a `uint8[]` field's items.
    pub fn bytes(&mut self, count: usize) -> Vec<u8> {
        (0..count).map(|_| low_byte_u64(self.get(8))).collect()
    }
}

/// The low byte, as a `(byte)` cast keeps it.
#[must_use]
pub fn low_byte(value: u32) -> u8 {
    value.to_le_bytes()[0]
}

/// The low byte of a 64-bit value.
#[must_use]
pub fn low_byte_u64(value: u64) -> u8 {
    value.to_le_bytes()[0]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `testconversion`: a value written twice, back to back, and both read back - the C#'s own
    /// round trips, from `DroneCAN.test()`.
    /// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:37-58, 1626-1640`
    #[test]
    fn the_cs_round_trips_hold() {
        let unsigned = |value: u64, len: u32| {
            let mut writer = BitWriter::new();
            writer.put(value, len);
            writer.put(value, len);
            let bytes = writer.into_bytes();
            let mut reader = BitReader::new(&bytes);
            assert_eq!(reader.get(len), value, "{value} in {len}");
            assert_eq!(reader.get(len), value, "{value} in {len}, second");
        };
        let signed = |value: i64, len: u32| {
            let mut writer = BitWriter::new();
            writer.put_signed(value, len);
            writer.put_signed(value, len);
            let bytes = writer.into_bytes();
            let mut reader = BitReader::new(&bytes);
            assert_eq!(reader.get_signed(len), value, "{value} in {len}");
            assert_eq!(reader.get_signed(len), value, "{value} in {len}, second");
        };
        unsigned(125, 7);
        unsigned(3, 3);
        signed(-3, 3);
        unsigned(3, 5);
        signed(-3, 5);
        unsigned(1_234_567_890, 55);
        unsigned(1_234_567_890, 33);
        signed(-1_234_567_890, 33);
        signed(-12_345_678, 27);
        signed(1 << 25, 27);
        signed(11_573_116_430, 37);
        signed(-3_330_374_480, 37);
    }

    /// The layout itself: a 13-bit value is its low byte from the top bit, then its top five
    /// bits; a NodeStatus's 2, 3 and 3 bits share one byte, health at the top.
    #[test]
    fn bits_go_in_as_libcanard_lays_them() {
        let mut writer = BitWriter::new();
        writer.put(0x1abc, 13);
        assert_eq!(writer.bit_len(), 13);
        // 0xbc, then 0x1a's five bits (11010) at the top of the next byte.
        assert_eq!(writer.into_bytes(), [0xbc, 0b1101_0000]);

        let mut writer = BitWriter::new();
        writer.put(2, 2); // health ERROR
        writer.put(3, 3); // mode SOFTWARE_UPDATE
        writer.put(5, 3); // sub_mode
        assert_eq!(writer.into_bytes(), [0b1001_1101]);

        let mut writer = BitWriter::new();
        writer.put_bool(true);
        writer.skip(5);
        writer.put(1, 2);
        assert_eq!(writer.into_bytes(), [0b1000_0001]);
    }

    /// A read past the end is zeros, and a tail array's length is the bytes left.
    #[test]
    fn reads_past_the_end_are_zero() {
        let bytes = [0xff, 0x80];
        let mut reader = BitReader::new(&bytes);
        assert_eq!(reader.get(4), 0xf);
        assert_eq!(reader.tail_len(7), 1);
        // Bits 4 to 11 are the low byte, 0xf8; bits 12 to 19 the high one, half past the end.
        assert_eq!(reader.get(16), 0x00f8);
        assert_eq!(reader.get(8), 0);
        // 28 bits read of 16: the C#'s unsigned arithmetic wraps, and the byte is what is kept.
        let wrapped = 16u32.wrapping_sub(28) / 8;
        assert_eq!(reader.tail_len(8), usize::from(low_byte(wrapped)));
        assert_eq!(
            reader.tail_len(9),
            usize::try_from(wrapped & 0xffff).unwrap()
        );
    }

    /// Floats are their bits, and signed values carry their sign through any width.
    #[test]
    fn floats_and_signs() {
        let mut writer = BitWriter::new();
        writer.put_f32(-2.5);
        writer.put_signed(-1, 48);
        writer.put_signed(i64::MIN, 64);
        let bytes = writer.into_bytes();
        let mut reader = BitReader::new(&bytes);
        assert!((reader.get_f32() + 2.5).abs() < f32::EPSILON);
        assert_eq!(reader.get_signed(48), -1);
        assert_eq!(reader.get_signed(64), i64::MIN);
    }
}
