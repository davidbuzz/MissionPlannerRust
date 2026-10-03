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

//! Payload field accessors with MAVLink v2 zero-extension semantics.
//!
//! A v2 sender strips trailing zero bytes, so a receiver must treat any byte past the end of the
//! received payload as zero. Every accessor here does that, which is why generated message code
//! can decode a truncated payload without a length check per field.

macro_rules! getter {
    ($name:ident, $ty:ty, $size:expr) => {
        /// Reads a field at `offset`, treating bytes past the end of the payload as zero.
        #[inline]
        #[must_use]
        pub fn $name(payload: &[u8], offset: usize) -> $ty {
            let mut buf = [0u8; $size];
            if let Some(src) = payload.get(offset..offset + $size) {
                buf.copy_from_slice(src);
            } else if let Some(src) = payload.get(offset..) {
                // Partially present: copy what we have, leave the rest zero.
                let n = src.len().min($size);
                if let (Some(d), Some(s)) = (buf.get_mut(..n), src.get(..n)) {
                    d.copy_from_slice(s);
                }
            }
            <$ty>::from_le_bytes(buf)
        }
    };
}

getter!(get_u16, u16, 2);
getter!(get_i16, i16, 2);
getter!(get_u32, u32, 4);
getter!(get_i32, i32, 4);
getter!(get_u64, u64, 8);
getter!(get_i64, i64, 8);
getter!(get_f32, f32, 4);
getter!(get_f64, f64, 8);

/// Reads a `u8`, treating bytes past the end of the payload as zero.
#[inline]
#[must_use]
pub fn get_u8(payload: &[u8], offset: usize) -> u8 {
    payload.get(offset).copied().unwrap_or(0)
}

/// Reads an `i8`, treating bytes past the end of the payload as zero.
#[inline]
#[must_use]
pub fn get_i8(payload: &[u8], offset: usize) -> i8 {
    get_u8(payload, offset) as i8
}

/// Writes a slice into `out` at `offset`, ignoring any part that does not fit.
#[inline]
pub fn put_bytes(out: &mut [u8], offset: usize, bytes: &[u8]) {
    if let Some(dst) = out.get_mut(offset..offset + bytes.len()) {
        dst.copy_from_slice(bytes);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_past_the_end_are_zero() {
        let payload = [1u8, 2];
        assert_eq!(get_u8(&payload, 0), 1);
        assert_eq!(get_u8(&payload, 5), 0);
        assert_eq!(
            get_u32(&payload, 0),
            0x0000_0201,
            "partial read zero-extends"
        );
        assert_eq!(get_u64(&payload, 100), 0);
    }

    #[test]
    fn little_endian_round_trip() {
        let mut out = [0u8; 8];
        put_bytes(&mut out, 0, &0x1234_5678_u32.to_le_bytes());
        assert_eq!(get_u32(&out, 0), 0x1234_5678);
    }
}
