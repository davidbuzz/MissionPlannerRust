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

//! Typed units and geodesy primitives.
//!
//! # Why this crate exists
//!
//! Mission Planner passes latitude, longitude and altitude around as bare numbers in at least
//! four different scales: degrees, radians, 1e7 fixed-point degrees (MAVLink's wire format) and
//! centidegrees. The C# code carries this in variable names (`lat`, `latf`, `lat_int`) and gets
//! it wrong in places. A swapped or misscaled coordinate in a ground control station is not a
//! cosmetic bug - it flies a vehicle somewhere unintended.
//!
//! So: no bare `f64` for a position anywhere in this port. Conversions are explicit, named, and
//! tested against known values.

#![forbid(unsafe_code)]

pub mod geodesy;
pub mod tiles;

pub use geodesy::{Bearing, LatLon, LatLonAlt, PositionError, WebMercator};
pub use tiles::{TileId, tiles_for_view, zoom_for_span};

/// How far apart two doubles are in units in the last place: the number of representable values
/// between them, `0` for equal values (so for `0.0` and `-0.0`), `u64::MAX` when either is NaN.
#[must_use]
pub fn ulps_apart(a: f64, b: f64) -> u64 {
    if a.is_nan() || b.is_nan() {
        return u64::MAX;
    }
    if a == b {
        return 0;
    }
    // The bit pattern, reflected for negatives so that the line of patterns is in numeric order
    // and the distance across zero counts the values on both sides.
    fn ordered(x: f64) -> i128 {
        let magnitude = i128::from(x.to_bits() & 0x7fff_ffff_ffff_ffff);
        if x.is_sign_negative() {
            -magnitude
        } else {
            magnitude
        }
    }
    u64::try_from((ordered(a) - ordered(b)).unsigned_abs()).unwrap_or(u64::MAX)
}

/// How far, in ulps, a value computed here may lie from the same value in a golden file written by
/// Mission Planner's own code; [`GOLDEN_ABS`] is the same allowance as a difference, and
/// [`golden_match`] applies whichever is the looser.
///
/// The goldens under `testdata/` were made by running Mission Planner's C# headless under mono on
/// Linux, so their sines, cosines, arctangents, exponentials and powers are glibc's. On Linux the
/// port reproduces them to the bit and is held to that: both allowances are zero. Apple's libm
/// returns the last bit of those functions differently in places, and a result composed of several
/// of them drifts by a few more: on the owner's Mac (2026-10-03) a UTM coordinate differed by one
/// ulp, a Web Mercator inverse (`atan`, `exp`) by eight, and a corridor latitude of 6.5e-4 degrees
/// by 4e-19, four of its own ulps, the error being the computation's at the scale of its inputs,
/// not of its result. Elsewhere, then, the hold is sixteen ulps or 1e-14 - a nanometre of latitude,
/// a hundredth of a picometre of UTM - and the tests still report where identity holds.
pub const GOLDEN_ULPS: u64 = if cfg!(target_os = "linux") { 0 } else { 16 };

/// See [`GOLDEN_ULPS`].
pub const GOLDEN_ABS: f64 = if cfg!(target_os = "linux") { 0.0 } else { 1e-14 };

/// Whether `ours` is `theirs` within [`GOLDEN_ULPS`] or [`GOLDEN_ABS`]: equality where the goldens
/// were made, and the documented allowance elsewhere. A NaN matches nothing.
#[must_use]
// Where the goldens were made the allowance is zero, and clippy sees `<= 0`: that case is the point.
#[allow(clippy::absurd_extreme_comparisons)]
pub fn golden_match(ours: f64, theirs: f64) -> bool {
    ulps_apart(ours, theirs) <= GOLDEN_ULPS || (ours - theirs).abs() <= GOLDEN_ABS
}

/// Angle in degrees.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct Degrees(pub f64);

/// Angle in radians.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct Radians(pub f64);

impl Degrees {
    /// Converts to radians.
    #[must_use]
    pub fn to_radians(self) -> Radians {
        Radians(self.0.to_radians())
    }

    /// Normalises to `[0, 360)`.
    #[must_use]
    pub fn normalised(self) -> Self {
        let mut v = self.0 % 360.0;
        if v < 0.0 {
            v += 360.0;
        }
        Self(v)
    }

    /// Normalises to `(-180, 180]`.
    #[must_use]
    pub fn normalised_signed(self) -> Self {
        let mut v = self.normalised().0;
        if v > 180.0 {
            v -= 360.0;
        }
        Self(v)
    }
}

impl Radians {
    /// Converts to degrees.
    #[must_use]
    pub fn to_degrees(self) -> Degrees {
        Degrees(self.0.to_degrees())
    }
}

/// A length in metres.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct Metres(pub f64);

impl Metres {
    /// From millimetres, MAVLink's altitude unit.
    #[must_use]
    pub fn from_millimetres(mm: i32) -> Self {
        Self(f64::from(mm) / 1000.0)
    }

    /// From centimetres, used by several MAVLink fields.
    #[must_use]
    pub fn from_centimetres(cm: i32) -> Self {
        Self(f64::from(cm) / 100.0)
    }

    /// In feet, for the UI's imperial mode.
    #[must_use]
    pub fn as_feet(self) -> f64 {
        self.0 * 3.280_839_895_013_123
    }
}

/// A speed in metres per second.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct MetresPerSecond(pub f64);

impl MetresPerSecond {
    /// From centimetres per second, MAVLink's velocity unit.
    #[must_use]
    pub fn from_centimetres_per_second(cms: i16) -> Self {
        Self(f64::from(cms) / 100.0)
    }

    /// In kilometres per hour.
    #[must_use]
    pub fn as_kmh(self) -> f64 {
        self.0 * 3.6
    }

    /// In knots.
    #[must_use]
    pub fn as_knots(self) -> f64 {
        self.0 * 1.943_844_492_440_605
    }

    /// In miles per hour.
    #[must_use]
    pub fn as_mph(self) -> f64 {
        self.0 * 2.236_936_292_054_402
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn ulps_count_the_values_between_two_doubles() {
        assert_eq!(super::ulps_apart(1.0, 1.0), 0);
        assert_eq!(super::ulps_apart(0.0, -0.0), 0);
        assert_eq!(super::ulps_apart(1.0, 1.0 + f64::EPSILON), 1);
        assert_eq!(super::ulps_apart(1.0 + f64::EPSILON, 1.0), 1);
        assert_eq!(super::ulps_apart(-1.0, -1.0 - f64::EPSILON), 1);
        // Across zero every subnormal on both sides is counted, 2^52 of them each.
        assert_eq!(
            super::ulps_apart(f64::MIN_POSITIVE, -f64::MIN_POSITIVE),
            1 << 53
        );
        assert_eq!(super::ulps_apart(f64::NAN, 1.0), u64::MAX);
        // What the Mac returned for a golden UTM point against what glibc wrote.
        assert_eq!(
            super::ulps_apart(17.400_000_000_014_856, 17.400_000_000_014_852),
            1
        );
        assert_eq!(super::ulps_apart(1.0, 2.0), 1 << 52);
    }

    #[test]
    fn a_golden_match_is_equality_where_the_goldens_were_made() {
        assert!(super::golden_match(1.0, 1.0));
        assert!(super::golden_match(-0.0, 0.0));
        assert!(!super::golden_match(f64::NAN, f64::NAN));
        assert!(!super::golden_match(1.0, 1.1));
        if cfg!(target_os = "linux") {
            assert!(!super::golden_match(1.0, 1.0 + f64::EPSILON));
            assert!(!super::golden_match(0.0, 1e-300));
        } else {
            assert!(super::golden_match(1.0, 1.0 + f64::EPSILON));
            assert!(super::golden_match(-0.0006515212629328473, -0.0006515212629328477));
            assert!(super::golden_match(-30.000000150439007, -30.00000015043898));
            assert!(!super::golden_match(30.0, 30.0 + 1e-12));
        }
    }

    use super::*;

    #[test]
    fn angle_conversions_round_trip() {
        let d = Degrees(123.456);
        let back = d.to_radians().to_degrees();
        assert!((back.0 - d.0).abs() < 1e-12);
    }

    #[test]
    fn normalisation_handles_wrap_in_both_directions() {
        assert!((Degrees(370.0).normalised().0 - 10.0).abs() < 1e-12);
        assert!((Degrees(-10.0).normalised().0 - 350.0).abs() < 1e-12);
        assert!((Degrees(190.0).normalised_signed().0 - -170.0).abs() < 1e-12);
        assert!((Degrees(180.0).normalised_signed().0 - 180.0).abs() < 1e-12);
    }

    #[test]
    fn length_conversions_match_known_values() {
        assert!((Metres::from_millimetres(1500).0 - 1.5).abs() < 1e-12);
        assert!((Metres::from_centimetres(250).0 - 2.5).abs() < 1e-12);
        // 100 m is 328.084 ft.
        assert!((Metres(100.0).as_feet() - 328.083_989_5).abs() < 1e-6);
    }

    #[test]
    fn speed_conversions_match_known_values() {
        let v = MetresPerSecond(10.0);
        assert!((v.as_kmh() - 36.0).abs() < 1e-12);
        assert!((v.as_knots() - 19.438_444_9).abs() < 1e-6);
        assert!((v.as_mph() - 22.369_362_9).abs() < 1e-6);
        assert!((MetresPerSecond::from_centimetres_per_second(1234).0 - 12.34).abs() < 1e-12);
    }
}
