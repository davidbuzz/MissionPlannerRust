//! Positions and the operations a ground control station performs on them.
//!
//! The arithmetic is Mission Planner's, transliterated operation for operation: distance, bearing
//! and offset from `ExtLibs/Utilities/PointLatLngAlt.cs`, Web Mercator from GMap.NET's
//! `MercatorProjection.cs`. `crates/mp-units/tests/projection.rs` holds every one of them to what
//! the C# returns under mono for the coordinates in `testdata/projection`, most to the bit, so an
//! algebraically equal rewrite - a `mul_add`, a different radius, a reassociated product - is a
//! test failure, not a refactor.

use std::f64::consts::PI;

use crate::{Degrees, Metres};

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

    /// Great-circle distance, `PointLatLngAlt.GetDistance` (`PointLatLngAlt.cs:382-393`).
    ///
    /// The C# hand-inlines a haversine on a 6371 km sphere, not the WGS84 ellipsoid, and PLAN.md
    /// §1.3 ports it literally: a better geodesy would move every distance the application shows
    /// by up to 0.5% from what Mission Planner shows for the same two points. The expression is
    /// the C#'s in its order - `(6371 * c) * 1000`, not `6371000 * c` - because the two round
    /// differently and the test holds this to the bit.
    #[must_use]
    pub fn distance_to(self, other: Self) -> Metres {
        // C#: ExtLibs/Utilities/PointLatLngAlt.cs:384-392, the C#'s names kept.
        let d = self.latitude() * 0.017_453_292_519_943_295;
        let num2 = self.longitude() * 0.017_453_292_519_943_295;
        let num3 = other.latitude() * 0.017_453_292_519_943_295;
        let num4 = other.longitude() * 0.017_453_292_519_943_295;
        let num5 = num4 - num2;
        let num6 = num3 - d;
        // Math.Pow(x, 2.0) is libm's pow under mono and powi(2) is one multiply; they gave the
        // same bits for every pair in testdata/projection, which tests/projection.rs holds.
        let num7 =
            (num6 / 2.0).sin().powi(2) + ((d.cos() * num3.cos()) * (num5 / 2.0).sin().powi(2));
        let num8 = 2.0 * num7.sqrt().atan2((1.0 - num7).sqrt());
        Metres((6371.0 * num8) * 1000.0)
    }

    /// Initial great-circle bearing to `other`, `PointLatLngAlt.GetBearing`
    /// (`PointLatLngAlt.cs:350-360`), in `[0, 360)`.
    ///
    /// Normalised as the C# does, `(b + 360) % 360`, which rounds an eastward bearing to the
    /// precision of a number above 360; normalising only the negative ones would keep bits the C#
    /// throws away.
    #[must_use]
    pub fn bearing_to(self, other: Self) -> Bearing {
        // C#: ExtLibs/Utilities/PointLatLngAlt.cs:352-359. MathHelper.deg2rad is 1 / (180 / PI),
        // which is PI / 180 to the bit, the constant to_radians multiplies by.
        let latitude1 = self.latitude().to_radians();
        let latitude2 = other.latitude().to_radians();
        let longitude_difference = (other.longitude() - self.longitude()).to_radians();

        let y = longitude_difference.sin() * latitude2.cos();
        // Two products and a difference, never a mul_add: the C# rounds each product.
        let x = latitude1.cos() * latitude2.sin()
            - latitude1.sin() * latitude2.cos() * longitude_difference.cos();

        Bearing(Degrees((y.atan2(x).to_degrees() + 360.0) % 360.0))
    }

    /// The position reached by travelling `distance` along `bearing`, `PointLatLngAlt.newpos`
    /// (`PointLatLngAlt.cs:313-335`).
    ///
    /// The C# travels on a 6378.1 km sphere while [`LatLon::distance_to`] measures on a 6371 km
    /// one, so a point placed 1000 m away measures 998.89 m. That is Mission Planner's behaviour
    /// and it is kept: every offset the planner draws is placed this way.
    ///
    /// The one departure: the C# returns a longitude past ±180 when the path crosses the
    /// antimeridian, and a [`LatLon`] cannot hold one, so such a longitude is wrapped - the same
    /// meridian, 360 degrees round. A longitude already in range is returned to the bit.
    #[must_use]
    pub fn offset(self, bearing: Bearing, distance: Metres) -> Self {
        // C#: ExtLibs/Utilities/PointLatLngAlt.cs:319-332.
        let radius_of_earth = 6_378_100.0;

        let lat1 = self.latitude().to_radians();
        let lon1 = self.longitude().to_radians();
        let brng = bearing.0.0.to_radians();
        let dr = distance.0 / radius_of_earth;

        let lat2 = (lat1.sin() * dr.cos() + lat1.cos() * dr.sin() * brng.cos()).asin();
        let lon2 =
            lon1 + (brng.sin() * dr.sin() * lat1.cos()).atan2(dr.cos() - lat1.sin() * lat2.sin());

        let latout = lat2.to_degrees();
        let lngout = lon2.to_degrees();

        Self {
            latitude: Degrees(latout),
            longitude: if (-180.0..=180.0).contains(&lngout) {
                Degrees(lngout)
            } else {
                Degrees(lngout).normalised_signed()
            },
        }
    }
}

/// Web Mercator (EPSG:3857) projected coordinates, normalised to 0..1.
///
/// This is the projection every slippy map tile server uses, so a map that draws tiles must work
/// in it: at zoom `z` the world is `2^z` tiles across, and tile `(x, y)` covers
/// `[x/2^z, (x+1)/2^z]` of this unit square.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct WebMercator {
    /// Easting, 0 at 180°W and 1 at 180°E.
    pub x: f64,
    /// Northing, 0 at the north edge and 1 at the south edge - y grows downward, matching tile
    /// and screen coordinates rather than latitude.
    pub y: f64,
}

/// The latitude GMap clips to, `MercatorProjection.cs:14-15`, because the projection sends the
/// poles to infinity.
///
/// It is 85.05112878, which is 1.9e-10 degrees beyond the projection's true edge
/// (`atan(sinh(pi))`, 85.0511287798066), so a pole projects to a `y` of -6e-12 rather than 0: less
/// than a hundredth of a pixel at zoom 20. Kept as GMap has it, since every latitude between the
/// two projects differently otherwise.
pub const WEB_MERCATOR_MAX_LATITUDE: f64 = 85.051_128_78;

impl LatLon {
    /// Projects to Web Mercator, clamping latitude to GMap's limit.
    ///
    /// `MercatorProjection.FromLatLngToPixel` (`MercatorProjection.cs:52-71`) up to the point
    /// where it scales to a zoom level and rounds to a whole pixel: this is the continuous value
    /// it rounds, so the map can pan and zoom by fractions of a pixel where GMap cannot.
    #[must_use]
    pub fn to_web_mercator(self) -> WebMercator {
        // C#: MercatorProjection.cs:56-61. GMap's Clip is Math.Min(Math.Max(n, min), max)
        // (PureProjection.cs:424-427), which clamp is for every number that is not NaN, and a
        // LatLon holds no NaN.
        let lat = self
            .latitude()
            .clamp(-WEB_MERCATOR_MAX_LATITUDE, WEB_MERCATOR_MAX_LATITUDE);
        let lng = self.longitude().clamp(-180.0, 180.0);

        let x = (lng + 180.0) / 360.0;
        // lat * PI / 180, not lat.to_radians(): the C# multiplies by PI first, and the two round
        // differently.
        let sin_latitude = (lat * PI / 180.0).sin();
        let y = 0.5 - ((1.0 + sin_latitude) / (1.0 - sin_latitude)).ln() / (4.0 * PI);
        WebMercator { x, y }
    }

    /// Inverse of [`LatLon::to_web_mercator`], `MercatorProjection.FromPixelToLatLng`
    /// (`MercatorProjection.cs:73-88`) without the pixel: given `pixel / map size`, which is exact
    /// for GMap's power-of-two map sizes, it returns what GMap returns to the bit.
    pub fn from_web_mercator(projected: WebMercator) -> Result<Self, PositionError> {
        // C#: MercatorProjection.cs:81-85.
        let xx = projected.x - 0.5;
        let yy = 0.5 - projected.y;

        let lat = 90.0 - 360.0 * (-yy * 2.0 * PI).exp().atan() / PI;
        let lng = 360.0 * xx;
        Self::new(lat, lng)
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
    fn offset_then_distance_comes_back_short_by_the_ratio_of_the_two_radii() {
        // Mission Planner places a point on a 6378.1 km sphere (newpos) and measures on a 6371 km
        // one (GetDistance), so what it measures is 6371 / 6378.1 of what it placed. A port that
        // "fixed" this would disagree with every distance the C# shows next to a planned point.
        let start = brisbane();
        for bearing in [0.0, 45.0, 90.0, 180.0, 270.0, 359.0] {
            for metres in [1.0, 100.0, 10_000.0] {
                let moved = start.offset(Bearing(Degrees(bearing)), Metres(metres));
                let measured = start.distance_to(moved).0;
                let expected = metres * 6371.0 / 6378.1;
                assert!(
                    (measured - expected).abs() < expected * 1e-6 + 1e-6,
                    "bearing {bearing}, placed {metres} m, expected {expected} m, measured {measured} m"
                );
            }
        }
    }

    #[test]
    fn web_mercator_anchors_are_exact() {
        // Null Island is the centre of the unit square.
        let origin = LatLon::new(0.0, 0.0).expect("valid").to_web_mercator();
        assert!((origin.x - 0.5).abs() < 1e-12, "x was {}", origin.x);
        assert!((origin.y - 0.5).abs() < 1e-12, "y was {}", origin.y);

        // The corners of the projected world.
        let nw = LatLon::new(WEB_MERCATOR_MAX_LATITUDE, -180.0)
            .expect("valid")
            .to_web_mercator();
        assert!(
            nw.x.abs() < 1e-12 && nw.y.abs() < 1e-9,
            "north-west was {nw:?}"
        );
        let se = LatLon::new(-WEB_MERCATOR_MAX_LATITUDE, 180.0)
            .expect("valid")
            .to_web_mercator();
        assert!(
            (se.x - 1.0).abs() < 1e-12 && (se.y - 1.0).abs() < 1e-9,
            "south-east was {se:?}"
        );
    }

    #[test]
    fn web_mercator_y_grows_southward() {
        // Screen and tile coordinates grow downward; latitude grows upward. Getting this backwards
        // flips the map vertically, which looks plausible enough to ship.
        let north = LatLon::new(40.0, 0.0).expect("valid").to_web_mercator();
        let south = LatLon::new(-40.0, 0.0).expect("valid").to_web_mercator();
        assert!(
            north.y < south.y,
            "north {} should be above south {}",
            north.y,
            south.y
        );
    }

    #[test]
    fn web_mercator_round_trips() {
        for (lat, lon) in [
            (0.0, 0.0),
            (-35.363_262, 149.165_237),
            (51.477_9, -0.001_5),
            (-33.868_8, 151.209_3),
            (78.0, -170.0),
        ] {
            let original = LatLon::new(lat, lon).expect("valid");
            let back = LatLon::from_web_mercator(original.to_web_mercator()).expect("valid");
            assert!(
                (back.latitude() - lat).abs() < 1e-9,
                "latitude {lat} came back as {}",
                back.latitude()
            );
            assert!(
                (back.longitude() - lon).abs() < 1e-9,
                "longitude {lon} came back as {}",
                back.longitude()
            );
        }
    }

    #[test]
    fn web_mercator_clamps_the_poles_rather_than_producing_infinity() {
        // GMap's clip latitude is a hair beyond the projection's edge, so the poles land a hair
        // outside the unit square - six trillionths of the world, not infinity.
        for latitude in [90.0, -90.0] {
            let pole = LatLon::new(latitude, 0.0).expect("valid").to_web_mercator();
            assert!(pole.y.is_finite(), "{latitude} projected to {}", pole.y);
            let edge = if latitude > 0.0 { 0.0 } else { 1.0 };
            assert!(
                (pole.y - edge).abs() < 1e-11,
                "{latitude} projected to {}",
                pole.y
            );
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
