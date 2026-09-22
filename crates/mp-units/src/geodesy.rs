//! Positions and the operations a ground control station performs on them.

use crate::{Degrees, Metres};

/// Mean Earth radius (IUGG), the sphere used for great-circle calculations.
pub const EARTH_MEAN_RADIUS: f64 = 6_371_008.8;

/// WGS84 semi-major axis.
pub const WGS84_A: f64 = 6_378_137.0;
/// WGS84 flattening.
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;

/// A compass bearing, degrees clockwise from true north.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct Bearing(pub Degrees);

impl Bearing {
    /// The bearing in degrees, normalised to `[0, 360)`.
    #[must_use]
    pub fn degrees(self) -> f64 {
        self.0.normalised().0
    }
}

/// A geographic position.
///
/// Constructed from explicit units only, so a caller cannot accidentally pass MAVLink's
/// 1e7 fixed-point integers where degrees are expected - the mistake that puts a vehicle in the
/// Gulf of Guinea.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LatLon {
    latitude: Degrees,
    longitude: Degrees,
}

/// Why a position is not valid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum PositionError {
    /// Latitude outside [-90, 90].
    #[error("latitude out of range")]
    Latitude,
    /// Longitude outside [-180, 180].
    #[error("longitude out of range")]
    Longitude,
}

impl LatLon {
    /// Builds a position from degrees, rejecting out-of-range values.
    pub fn new(latitude: f64, longitude: f64) -> Result<Self, PositionError> {
        if !(-90.0..=90.0).contains(&latitude) || latitude.is_nan() {
            return Err(PositionError::Latitude);
        }
        if !(-180.0..=180.0).contains(&longitude) || longitude.is_nan() {
            return Err(PositionError::Longitude);
        }
        Ok(Self {
            latitude: Degrees(latitude),
            longitude: Degrees(longitude),
        })
    }

    /// Builds a position from MAVLink's 1e7 fixed-point degrees.
    ///
    /// An unset position is transmitted as `(0, 0)`, which is a real place in the Atlantic.
    /// Callers that care about "no fix yet" must check the GPS fix type, not the coordinates.
    pub fn from_mavlink_e7(lat_e7: i32, lon_e7: i32) -> Result<Self, PositionError> {
        Self::new(f64::from(lat_e7) / 1e7, f64::from(lon_e7) / 1e7)
    }

    /// Latitude in degrees.
    #[must_use]
    pub const fn latitude(self) -> f64 {
        self.latitude.0
    }

    /// Longitude in degrees.
    #[must_use]
    pub const fn longitude(self) -> f64 {
        self.longitude.0
    }

    /// As MAVLink's 1e7 fixed-point degrees.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)] // range-checked at construction
    pub fn to_mavlink_e7(self) -> (i32, i32) {
        (
            (self.latitude.0 * 1e7).round() as i32,
            (self.longitude.0 * 1e7).round() as i32,
        )
    }

    /// Great-circle distance using the haversine formula on a sphere.
    ///
    /// Accurate to roughly 0.3% against the WGS84 ellipsoid, which is fine for range readouts and
    /// map interaction. Survey grid generation (D11) needs ellipsoidal accuracy and must use a
    /// geodesic solver instead.
    #[must_use]
    pub fn distance_to(self, other: Self) -> Metres {
        let lat1 = self.latitude.to_radians().0;
        let lat2 = other.latitude.to_radians().0;
        let dlat = (other.latitude.0 - self.latitude.0).to_radians();
        let dlon = (other.longitude.0 - self.longitude.0).to_radians();

        let a = (dlat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().asin().min(std::f64::consts::FRAC_PI_2 * 2.0);
        Metres(EARTH_MEAN_RADIUS * c)
    }

    /// Initial bearing along the great circle to `other`.
    #[must_use]
    pub fn bearing_to(self, other: Self) -> Bearing {
        let lat1 = self.latitude.to_radians().0;
        let lat2 = other.latitude.to_radians().0;
        let dlon = (other.longitude.0 - self.longitude.0).to_radians();

        let y = dlon.sin() * lat2.cos();
        let x = lat1
            .cos()
            .mul_add(lat2.sin(), -(lat1.sin() * lat2.cos() * dlon.cos()));
        Bearing(Degrees(y.atan2(x).to_degrees()).normalised())
    }

    /// The position reached by travelling `distance` along `bearing`.
    #[must_use]
    pub fn offset(self, bearing: Bearing, distance: Metres) -> Self {
        let angular = distance.0 / EARTH_MEAN_RADIUS;
        let brg = bearing.0.to_radians().0;
        let lat1 = self.latitude.to_radians().0;
        let lon1 = self.longitude.to_radians().0;

        let lat2 = (lat1.sin() * angular.cos() + lat1.cos() * angular.sin() * brg.cos()).asin();
        let lon2 = lon1
            + (brg.sin() * angular.sin() * lat1.cos())
                .atan2(angular.cos() - lat1.sin() * lat2.sin());

        Self {
            latitude: Degrees(lat2.to_degrees()),
            longitude: Degrees(lon2.to_degrees()).normalised_signed(),
        }
    }
}

/// A position with an altitude.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LatLonAlt {
    /// Horizontal position.
    pub position: LatLon,
    /// Altitude above mean sea level.
    pub altitude_msl: Metres,
    /// Altitude above the home/takeoff point, which is what pilots actually fly to.
    pub altitude_relative: Metres,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Brisbane and Sydney, a pair with a well-known separation.
    fn brisbane() -> LatLon {
        LatLon::new(-27.4698, 153.0251).expect("valid")
    }
    fn sydney() -> LatLon {
        LatLon::new(-33.8688, 151.2093).expect("valid")
    }

    #[test]
    fn rejects_impossible_coordinates() {
        assert_eq!(LatLon::new(91.0, 0.0), Err(PositionError::Latitude));
        assert_eq!(LatLon::new(-90.1, 0.0), Err(PositionError::Latitude));
        assert_eq!(LatLon::new(0.0, 180.1), Err(PositionError::Longitude));
        assert_eq!(LatLon::new(f64::NAN, 0.0), Err(PositionError::Latitude));
        assert!(
            LatLon::new(90.0, 180.0).is_ok(),
            "the poles and the date line are valid"
        );
    }

    #[test]
    fn mavlink_fixed_point_round_trips() {
        let original = LatLon::new(-27.469_812_3, 153.025_123_4).expect("valid");
        let (lat_e7, lon_e7) = original.to_mavlink_e7();
        assert_eq!(lat_e7, -274_698_123);
        assert_eq!(lon_e7, 1_530_251_234);

        let back = LatLon::from_mavlink_e7(lat_e7, lon_e7).expect("valid");
        assert!((back.latitude() - original.latitude()).abs() < 1e-7);
        assert!((back.longitude() - original.longitude()).abs() < 1e-7);
    }

    #[test]
    fn distance_matches_a_known_separation() {
        // Brisbane to Sydney is about 732 km great-circle.
        let d = brisbane().distance_to(sydney());
        assert!((d.0 - 732_000.0).abs() < 5_000.0, "got {} m", d.0);
    }

    #[test]
    fn distance_is_symmetric_and_zero_to_self() {
        let a = brisbane();
        let b = sydney();
        assert!((a.distance_to(b).0 - b.distance_to(a).0).abs() < 1e-6);
        assert!(a.distance_to(a).0 < 1e-9);
    }

    #[test]
    fn bearing_points_the_right_way() {
        // Sydney is south-southwest of Brisbane.
        let b = brisbane().bearing_to(sydney()).degrees();
        assert!((190.0..=230.0).contains(&b), "bearing was {b}");

        // Due east along the equator.
        let origin = LatLon::new(0.0, 0.0).expect("valid");
        let east = LatLon::new(0.0, 1.0).expect("valid");
        assert!((origin.bearing_to(east).degrees() - 90.0).abs() < 1e-6);
    }

    #[test]
    fn offset_then_distance_returns_the_offset() {
        let start = brisbane();
        for bearing in [0.0, 45.0, 90.0, 180.0, 270.0, 359.0] {
            for metres in [1.0, 100.0, 10_000.0] {
                let moved = start.offset(Bearing(Degrees(bearing)), Metres(metres));
                let measured = start.distance_to(moved).0;
                assert!(
                    (measured - metres).abs() < metres * 1e-6 + 1e-6,
                    "bearing {bearing}, expected {metres} m, measured {measured} m"
                );
            }
        }
    }

    #[test]
    fn offset_across_the_date_line_stays_valid() {
        let near_line = LatLon::new(0.0, 179.999).expect("valid");
        let moved = near_line.offset(Bearing(Degrees(90.0)), Metres(1000.0));
        assert!(
            (-180.0..=180.0).contains(&moved.longitude()),
            "longitude wrapped out of range: {}",
            moved.longitude()
        );
    }
}
