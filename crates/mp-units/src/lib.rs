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

pub use geodesy::{Bearing, LatLon, LatLonAlt, PositionError, WebMercator};

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
