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

//! UTM as Mission Planner computes it, for the survey grid.
//!
//! Replaces the parts of `ExtLibs/Utilities/utmpos.cs`, `ExtLibs/Utilities/PointLatLngAlt.cs` and
//! `ExtLibs/ProjNet` (`ProjectedCoordinateSystem.WGS84_UTM`, `MapProjection`,
//! `TransverseMercator`) that `Grid.CreateGrid` reaches. Transliterated operation by operation,
//! in the C#'s evaluation order: the grid is gated against the C# at 1e-7 degrees (PLAN.md §7.2
//! class C), and a different transverse Mercator series - Krüger's, or a geodesy crate's - agrees
//! with ProjNet only to its own truncation error, which the grid's nearest-line tie-breaks can turn
//! into a different lane order.
//!
//! Three conventions are kept because the grid depends on them:
//! - the hemisphere is the sign of the zone (`utmpos.cs:71`, PLAN.md §1.3);
//! - a list is projected in the zone of its caller's choosing and the hemisphere of the list's
//!   *first* point (`PointLatLngAlt.cs:294-302`), so a polygon that crosses a zone edge or the
//!   equator is projected in one frame, distortion and all;
//! - distances are planar in that frame (`utmpos.cs:94`), not on the ellipsoid.

use std::f64::consts::PI;

/// Degrees to radians as ProjNet has it, `MathTransform.cs:244`.
const D2R: f64 = PI / 180.0;
/// Radians to degrees as ProjNet has it, `MathTransform.cs:239`.
const R2D: f64 = 180.0 / PI;

/// `MapProjection.cs:415`, `:430`, `:435`, `:440`.
const TWO_PI: f64 = PI * 2.0;
const MAX_VAL: f64 = 4.0;
const PRJ_MAXLONG: f64 = 2_147_483_647.0;
const DBLLONG: f64 = 4.611_686_01e18;

/// `TransverseMercator.cs:79`.
const EPSILON: f64 = 1e-6;

// The series coefficients of `TransverseMercator.cs:103-110` and `MapProjection.cs:682-693`, with
// the C#'s literals as written: both compilers round a literal to the nearest double, so copying
// the text is what keeps the constants identical.
#[allow(clippy::excessive_precision)]
const FC1: f64 = 1.000_000_000_000_000_000_000_00;
const FC2: f64 = 0.5;
#[allow(clippy::excessive_precision)]
const FC3: f64 = 0.166_666_666_666_666_666_666_66;
#[allow(clippy::excessive_precision)]
const FC4: f64 = 0.083_333_333_333_333_333_333_33;
const FC5: f64 = 0.05;
#[allow(clippy::excessive_precision)]
const FC6: f64 = 0.033_333_333_333_333_333_333_33;
#[allow(clippy::excessive_precision)]
const FC7: f64 = 0.023_809_523_809_523_809_523_80;
#[allow(clippy::excessive_precision)]
const FC8: f64 = 0.017_857_142_857_142_857_142_85;

const C00: f64 = 1.0;
const C02: f64 = 0.25;
const C04: f64 = 0.046_875;
const C06: f64 = 0.019_531_25;
const C08: f64 = 0.010_681_152_343_75;
const C22: f64 = 0.75;
const C44: f64 = 0.468_75;
#[allow(clippy::excessive_precision)]
const C46: f64 = 0.013_020_833_333_333_333_33;
#[allow(clippy::excessive_precision)]
const C48: f64 = 0.007_120_768_229_166_666_66;
#[allow(clippy::excessive_precision)]
const C66: f64 = 0.364_583_333_333_333_333_33;
#[allow(clippy::excessive_precision)]
const C68: f64 = 0.005_696_614_583_333_333_33;
const C88: f64 = 0.307_617_187_5;

/// The WGS84 ellipsoid as `Ellipsoid.WGS84` defines it, `Ellipsoid.cs:83`: the inverse flattening
/// is definitive and the semi-minor axis is derived from it.
const SEMI_MAJOR: f64 = 6_378_137.0;
const INVERSE_FLATTENING: f64 = 298.257_223_563;

/// `LinearUnit.Metre`: the projection's unit, which every coordinate is divided by on the way out.
const METERS_PER_UNIT: f64 = 1.0;

/// `Math.Pow(x, 2)`.
///
/// Mono's `Math.Pow` calls libm's `pow`, which is not specified to round as `x * x` does, and LLVM
/// rewrites `powf(x, 2.0)` into `x * x` when it can see the exponent. Hiding the exponent keeps the
/// call going to libm, like the C#'s.
pub(crate) fn pow2(x: f64) -> f64 {
    x.powf(std::hint::black_box(2.0))
}

/// `MapProjection.sign`, `MapProjection.cs:497`.
fn sign(x: f64) -> f64 {
    if x < 0.0 { -1.0 } else { 1.0 }
}

/// `MapProjection.adjust_lon`, `MapProjection.cs:509-537`: wraps a longitude into ±π.
///
/// Only the first two arms are reachable from a survey, but the loop is short and a partial port
/// would be a silent divergence for whoever calls it with a larger angle.
#[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)] // C#: (long) truncates
fn adjust_lon(mut x: f64) -> f64 {
    let mut count: i64 = 0;
    loop {
        if x.abs() <= PI {
            break;
        } else if ((x / PI).abs() as i64) < 2 {
            x -= sign(x) * TWO_PI;
        } else if (((x / TWO_PI).abs() as i64) as f64) < PRJ_MAXLONG {
            x -= ((x / TWO_PI) as i64) as f64 * TWO_PI;
        } else if (((x / (PRJ_MAXLONG * TWO_PI)).abs() as i64) as f64) < PRJ_MAXLONG {
            x -= ((x / (PRJ_MAXLONG * TWO_PI)) as i64) as f64 * (TWO_PI * PRJ_MAXLONG);
        } else if (((x / (DBLLONG * TWO_PI)).abs() as i64) as f64) < PRJ_MAXLONG {
            x -= ((x / (DBLLONG * TWO_PI)) as i64) as f64 * (TWO_PI * DBLLONG);
        } else {
            x -= sign(x) * TWO_PI;
        }
        count += 1;
        if (count as f64) > MAX_VAL {
            break;
        }
    }
    x
}

/// A UTM zone as `PointLatLngAlt.GetUTMZone` numbers it, `PointLatLngAlt.cs:191-198`: negative in
/// the southern hemisphere.
#[must_use]
#[allow(clippy::cast_possible_truncation)] // C#: (int) truncates toward zero, as `as` does
pub(crate) fn utm_zone(lat: f64, lng: f64) -> i32 {
    let mut zone = ((lng - -186.0) / 6.0) as i32;
    if lat < 0.0 {
        zone = -zone;
    }
    zone
}

/// ProjNet's transverse Mercator, specialised to `ProjectedCoordinateSystem.WGS84_UTM`.
///
/// `ProjectedCoordinateSystem.cs:68-88` supplies the parameters, `CoordinateTransformationFactory`
/// (`:149-157`, `:372-386`) adds the ellipsoid's axes and the metre, and `MapProjection`'s
/// constructor (`MapProjection.cs:98-132`) and `TransverseMercator`'s (`TransverseMercator.cs:145-160`)
/// derive the rest. Each field is named after the C# field it holds.
#[derive(Debug, Clone, Copy)]
struct TransverseMercator {
    es: f64,
    esp: f64,
    ml0: f64,
    semi_major: f64,
    scale_factor: f64,
    central_meridian: f64,
    false_easting: f64,
    false_northing: f64,
    en0: f64,
    en1: f64,
    en2: f64,
    en3: f64,
    en4: f64,
}

impl TransverseMercator {
    /// `ProjectedCoordinateSystem.WGS84_UTM(zone, zoneIsNorth)`.
    fn wgs84_utm(zone: u32, zone_is_north: bool) -> Self {
        // C#: Ellipsoid.cs:63
        let semi_minor = (1.0 - (1.0 / INVERSE_FLATTENING)) * SEMI_MAJOR;
        // C#: MapProjection.cs:811-826, EccentricySquared via FlatteningFactor
        let f = (SEMI_MAJOR - semi_minor) / SEMI_MAJOR;
        let es = 2.0 * f - f * f;

        // C#: ProjectedCoordinateSystem.cs:71-75. `zone * 6 - 183` is int arithmetic in the C#;
        // every zone a longitude produces is small enough that the double sum is the same integer.
        let central_meridian = D2R * (f64::from(zone) * 6.0 - 183.0);
        let lat_origin = D2R * 0.0;
        let false_easting = 500_000.0 * METERS_PER_UNIT;
        let false_northing = if zone_is_north { 0.0 } else { 10_000_000.0 } * METERS_PER_UNIT;

        // C#: MapProjection.cs:122-130, in its order, `t` and all.
        let en0 = C00 - es * (C02 + es * (C04 + es * (C06 + es * C08)));
        let en1 = es * (C22 - es * (C04 + es * (C06 + es * C08)));
        let mut t = es * es;
        let en2 = t * (C44 - es * (C46 + es * C48));
        t *= es;
        let en3 = t * (C66 - es * C68);
        let en4 = t * es * C88;

        let mut projection = Self {
            es,
            // C#: TransverseMercator.cs:152
            esp: es / (1.0 - es),
            ml0: 0.0,
            semi_major: SEMI_MAJOR,
            scale_factor: 0.9996,
            central_meridian,
            false_easting,
            false_northing,
            en0,
            en1,
            en2,
            en3,
            en4,
        };
        // C#: TransverseMercator.cs:153
        projection.ml0 = projection.mlfn(lat_origin, lat_origin.sin(), lat_origin.cos());
        projection
    }

    /// `MapProjection.mlfn(phi, sphi, cphi)`, `MapProjection.cs:767-772`: meridian distance.
    fn mlfn(&self, phi: f64, sphi: f64, cphi: f64) -> f64 {
        let cphi = cphi * sphi;
        let sphi = sphi * sphi;
        self.en0 * phi - cphi * (self.en1 + sphi * (self.en2 + sphi * (self.en3 + sphi * self.en4)))
    }

    /// `MapProjection.inv_mlfn`, `MapProjection.cs:780-803`. `None` where the C# throws
    /// `InvalidOperationException("No convergence")`.
    fn inv_mlfn(&self, arg: f64) -> Option<f64> {
        const MLFN_TOL: f64 = 1e-11;
        const MAXIMUM_ITERATIONS: i32 = 20;
        let k = 1.0 / (1.0 - self.es);
        let mut phi = arg;
        let mut i = MAXIMUM_ITERATIONS;
        loop {
            i -= 1;
            if i < 0 {
                return None;
            }
            let s = phi.sin();
            let mut t = 1.0 - self.es * s * s;
            t = (self.mlfn(phi, s, phi.cos()) - arg) * (t * t.sqrt()) * k;
            phi -= t;
            if t.abs() < MLFN_TOL {
                return Some(phi);
            }
        }
    }

    /// `TransverseMercator.RadiansToMeters`, `TransverseMercator.cs:167-200`.
    fn radians_to_meters(&self, lon: f64, lat: f64) -> (f64, f64) {
        let mut x = adjust_lon(lon - self.central_meridian);

        let mut y = lat;
        let sinphi = y.sin();
        let cosphi = y.cos();

        let mut t = if cosphi.abs() > EPSILON {
            sinphi / cosphi
        } else {
            0.0
        };
        t *= t;
        let mut al = cosphi * x;
        let als = al * al;
        al /= (1.0 - self.es * sinphi * sinphi).sqrt();
        let n = self.esp * cosphi * cosphi;

        y = self.mlfn(y, sinphi, cosphi) - self.ml0
            + sinphi
                * al
                * x
                * FC2
                * (1.0
                    + FC4
                        * als
                        * (5.0 - t
                            + n * (9.0 + 4.0 * n)
                            + FC6
                                * als
                                * (61.0
                                    + t * (t - 58.0)
                                    + n * (270.0 - 330.0 * t)
                                    + FC8 * als * (1385.0 + t * (t * (543.0 - t) - 3111.0)))));

        x = al
            * (FC1
                + FC3
                    * als
                    * (1.0 - t
                        + n
                        + FC5
                            * als
                            * (5.0
                                + t * (t - 18.0)
                                + n * (14.0 - 58.0 * t)
                                + FC7 * als * (61.0 + t * (t * (179.0 - t) - 479.0)))));

        // `scale_factor*_semiMajor*x`: the same product, IEEE multiplication being commutative.

        x *= self.scale_factor * self.semi_major;

        y *= self.scale_factor * self.semi_major;
        (x, y)
    }

    /// `TransverseMercator.MetersToRadians`, `TransverseMercator.cs:243-280`.
    fn meters_to_radians(&self, px: f64, py: f64) -> Option<(f64, f64)> {
        let mut x = px / self.semi_major;
        let mut y = py / self.semi_major;

        let phi = self.inv_mlfn(self.ml0 + y / self.scale_factor)?;

        if phi.abs() >= PI / 2.0 {
            y = if y < 0.0 { -(PI / 2.0) } else { PI / 2.0 };
            x = 0.0;
        } else {
            let sinphi = phi.sin();
            let cosphi = phi.cos();
            let mut t = if cosphi.abs() > EPSILON {
                sinphi / cosphi
            } else {
                0.0
            };
            let n = self.esp * cosphi * cosphi;
            let mut con = 1.0 - self.es * sinphi * sinphi;
            let d = x * con.sqrt() / self.scale_factor;
            con *= t;
            t *= t;
            let ds = d * d;

            y = phi
                - (con * ds / (1.0 - self.es))
                    * FC2
                    * (1.0
                        - ds * FC4
                            * (5.0 + t * (3.0 - 9.0 * n) + n * (1.0 - 4.0 * n)
                                - ds * FC6
                                    * (61.0 + t * (90.0 - 252.0 * n + 45.0 * t) + 46.0 * n
                                        - ds * FC8
                                            * (1385.0
                                                + t * (3633.0 + t * (4095.0 + 1574.0 * t))))));

            x = adjust_lon(
                self.central_meridian
                    + d * (FC1
                        - ds * FC3
                            * (1.0 + 2.0 * t + n
                                - ds * FC5
                                    * (5.0 + t * (28.0 + 24.0 * t + 8.0 * n) + 6.0 * n
                                        - ds * FC7
                                            * (61.0 + t * (662.0 + t * (1320.0 + 720.0 * t))))))
                        / cosphi,
            );
        }
        Some((x, y))
    }

    /// `MapProjection.DegreesToMeters`, `MapProjection.cs:307-323`: (lng, lat) to (x, y).
    fn degrees_to_meters(&self, lng: f64, lat: f64) -> (f64, f64) {
        let (x, y) = self.radians_to_meters(D2R * lng, D2R * lat);
        (
            (x + self.false_easting) / METERS_PER_UNIT,
            (y + self.false_northing) / METERS_PER_UNIT,
        )
    }

    /// `MapProjection.MetersToDegrees`, `MapProjection.cs:283-300`: (x, y) to (lng, lat).
    fn meters_to_degrees(&self, x: f64, y: f64) -> Option<(f64, f64)> {
        let (lng, lat) = self.meters_to_radians(
            x * METERS_PER_UNIT - self.false_easting,
            y * METERS_PER_UNIT - self.false_northing,
        )?;
        Some((R2D * lng, R2D * lat))
    }

    /// `PointLatLngAlt.TryGetTransform`, `PointLatLngAlt.cs:275-292`: the zone is the caller's, the
    /// hemisphere comes from `lat`.
    ///
    /// The C# caches one transform per signed zone. Every call `CreateGrid` makes passes a zone
    /// whose sign already agrees with `lat`, so the cache can only return what this computes.
    fn for_zone(utmzone: i32, lat: f64) -> Self {
        // C#: `lat < 0 ? false : true`, so a NaN latitude is north.
        let north = lat.partial_cmp(&0.0) != Some(std::cmp::Ordering::Less);
        Self::wgs84_utm(utmzone.unsigned_abs(), north)
    }
}

/// `utmpos`: a point in one UTM zone's plane, `utmpos.cs:12`.
///
/// The C# struct also carries a `Tag`; the grid keeps its tags beside the positions instead.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct UtmPos {
    /// Easting, metres.
    pub(crate) x: f64,
    /// Northing, metres, with the southern false northing when `zone` is negative.
    pub(crate) y: f64,
    /// Zone, negative in the southern hemisphere.
    pub(crate) zone: i32,
}

impl UtmPos {
    /// `utmpos.Zero`: the default struct, which the grid uses as "no intersection".
    pub(crate) const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        zone: 0,
    };

    /// `new utmpos(x, y, zone)`.
    pub(crate) const fn new(x: f64, y: f64, zone: i32) -> Self {
        Self { x, y, zone }
    }

    /// `new utmpos(PointLatLngAlt)`, `utmpos.cs:39-46`: projected in the point's own zone.
    pub(crate) fn from_lat_lng(lat: f64, lng: f64) -> Self {
        let zone = utm_zone(lat, lng);
        let (x, y) = to_utm(zone, lat, lat, lng);
        Self { x, y, zone }
    }

    /// `utmpos.ToLLA`, `utmpos.cs:69-83`: back to (lat, lng), the hemisphere from the zone's sign.
    ///
    /// `None` where ProjNet throws for want of convergence.
    pub(crate) fn to_lla(self) -> Option<(f64, f64)> {
        let (lng, lat) = TransverseMercator::wgs84_utm(self.zone.unsigned_abs(), self.zone >= 0)
            .meters_to_degrees(self.x, self.y)?;
        Some((lat, lng))
    }

    /// `utmpos.GetDistance`, `utmpos.cs:94-97`: planar, in the zone's metres.
    pub(crate) fn distance(self, b: Self) -> f64 {
        (pow2((self.x - b.x).abs()) + pow2((self.y - b.y).abs())).sqrt()
    }

    /// `utmpos.GetBearing`, `utmpos.cs:99-105`: degrees clockwise from grid north, in [0, 360).
    /// `MathHelper.rad2deg` (`Math.cs:10`) is `180 / Math.PI`, the same double as [`R2D`].
    pub(crate) fn bearing(self, b: Self) -> f64 {
        let y = b.y - self.y;
        let x = b.x - self.x;

        (R2D * x.atan2(y) + 360.0) % 360.0
    }

    /// `utmpos.operator ==`, `utmpos.cs:116-119`: position and zone.
    pub(crate) fn op_eq(self, other: Self) -> bool {
        self.x == other.x && self.y == other.y && self.zone == other.zone
    }

    /// `utmpos.Equals(object)`, `utmpos.cs:107-114`: position only, the zone is not compared.
    /// This is what `List.Remove`, `ValueType.Equals` and `Dictionary` use.
    pub(crate) fn equals(self, other: Self) -> bool {
        self.x == other.x && self.y == other.y
    }

    /// `utmpos.IsZero`, `utmpos.cs:142`.
    pub(crate) fn is_zero(self) -> bool {
        self.op_eq(Self::ZERO)
    }
}

/// `PointLatLngAlt.ToUTM(utmzone, lat, lng)`, `PointLatLngAlt.cs:305-310`, generalised so the
/// hemisphere's latitude can be a different point's: `ToUTM(int, List)` projects every point of the
/// list with the first point's hemisphere (`PointLatLngAlt.cs:296`).
pub(crate) fn to_utm(utmzone: i32, hemisphere_lat: f64, lat: f64, lng: f64) -> (f64, f64) {
    TransverseMercator::for_zone(utmzone, hemisphere_lat).degrees_to_meters(lng, lat)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mission Planner's value to the bit where its goldens were made, and within
    /// [`mp_units::GOLDEN_ULPS`] of it elsewhere (one ulp on the owner's Mac, 2026-10-03).
    fn to_the_bit(ours: f64, theirs: f64) -> bool {
        mp_units::golden_match(ours, theirs)
    }

    /// Every point of `testdata/projection/points.txt` through `new utmpos(point)` and
    /// `ToLLA()` under mono, as `tools/csharp-reference/regen-projection.sh` printed them
    /// (`testdata/projection/golden/utm.csv`, G17): the zone and both metres bit for bit, and
    /// the way back bit for bit. The oracle is the same build of `ExtLibs/Utilities` the grid
    /// goldens come from.
    #[test]
    fn every_projection_golden_point_matches_the_c_sharp_bit_for_bit() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/projection/golden/utm.csv");
        let text = std::fs::read_to_string(&path).expect("testdata/projection/golden/utm.csv");
        let mut compared = 0;
        for line in text.lines().filter(|line| line.starts_with("utm,")) {
            let fields: Vec<&str> = line.split(',').collect();
            assert_eq!(fields.len(), 8, "{line}");
            let parse = |i: usize| {
                fields[i]
                    .parse::<f64>()
                    .unwrap_or_else(|_| panic!("{line}"))
            };
            let (lat, lng) = (parse(1), parse(2));
            let zone: i32 = fields[3].parse().unwrap_or_else(|_| panic!("{line}"));
            let (x, y, back_lat, back_lng) = (parse(4), parse(5), parse(6), parse(7));

            let ours = UtmPos::from_lat_lng(lat, lng);
            assert_eq!(ours.zone, zone, "zone of ({lat}, {lng})");
            assert!(
                to_the_bit(ours.x, x),
                "x of ({lat}, {lng}): {} vs {x}",
                ours.x
            );
            assert!(
                to_the_bit(ours.y, y),
                "y of ({lat}, {lng}): {} vs {y}",
                ours.y
            );

            let (our_lat, our_lng) = ours.to_lla().unwrap_or_else(|| panic!("ToLLA of {line}"));
            assert!(
                to_the_bit(our_lat, back_lat),
                "ToLLA lat of {line}: {our_lat}"
            );
            assert!(
                to_the_bit(our_lng, back_lng),
                "ToLLA lng of {line}: {our_lng}"
            );
            compared += 1;
        }
        assert!(
            compared >= 600,
            "only {compared} golden points: is the file complete?"
        );
    }

    #[test]
    fn zones_are_signed_by_hemisphere() {
        // PLAN.md §1.3: the oracle returns zone -56 for Brisbane.
        assert_eq!(utm_zone(-27.4698, 153.0251), -56);
        assert_eq!(utm_zone(47.4, 8.5), 32);
        // (int) truncates toward zero, and zone 1 starts at -180.
        assert_eq!(utm_zone(0.0, -180.0), 1);
        assert_eq!(utm_zone(-0.0, 0.0), 31);
    }

    /// `new utmpos(PointLatLngAlt)` and `utmpos.ToLLA()` as Mission Planner computes them: printed
    /// (G17) by a probe compiled against the regen-grid.sh build of `ExtLibs/Utilities` under
    /// mono 6.12, at the MP commit PLAN.md pins. The round trip is not exact - ProjNet's series is
    /// truncated, by 5e-12 degrees three degrees off the central meridian - and must not be, since
    /// the grid inherits the same residue.
    #[test]
    #[allow(clippy::excessive_precision)] // G17, digit for digit as the oracle printed them
    fn projection_matches_mission_planner_bit_for_bit() {
        #[allow(clippy::type_complexity)]
        let oracle: [((f64, f64), (f64, f64, i32), (f64, f64)); 5] = [
            (
                (-35.363_261, 149.165_23),
                (696_719.244_924_322, 6_084_519.586_830_075_8, -55),
                (-35.363_260_999_994_822, 149.165_230_000_008_53),
            ),
            (
                (47.4, 8.5),
                (462_271.877_775_581_67, 5_249_737.396_579_823_5, 32),
                (47.399_999_999_999_991, 8.499_999_999_999_998_2),
            ),
            (
                (69.6, 18.9),
                (418_319.852_774_184_37, 7_722_670.373_663_689, 34),
                (69.599_999_999_996_427, 18.900_000_000_000_567),
            ),
            (
                (-0.001, 32.6),
                (455_489.650_806_278_16, 9_999_889.467_242_129_1, -36),
                (-0.001_000_000_000_003_310_3, 32.600_000_000_000_009),
            ),
            (
                (-27.4698, 153.0251),
                (502_479.868_951_974_09, 6_961_528.092_882_787_8, -56),
                (-27.469_800_000_000_003, 153.025_100_000_000_01),
            ),
        ];
        for ((lat, lng), (x, y, zone), (back_lat, back_lng)) in oracle {
            let utm = UtmPos::from_lat_lng(lat, lng);
            assert_eq!(utm.zone, zone, "{lat},{lng}");
            assert!(
                to_the_bit(utm.x, x) && to_the_bit(utm.y, y),
                "{lat},{lng}: {},{} vs {x},{y}",
                utm.x,
                utm.y
            );
            let (our_lat, our_lng) = utm.to_lla().unwrap_or_else(|| panic!("ToLLA of {lat},{lng}"));
            assert!(
                to_the_bit(our_lat, back_lat) && to_the_bit(our_lng, back_lng),
                "{lat},{lng} back: {our_lat},{our_lng} vs {back_lat},{back_lng}"
            );
        }
    }

    #[test]
    fn the_southern_hemisphere_carries_the_false_northing() {
        let south = UtmPos::from_lat_lng(-35.363_261, 149.165_230);
        assert_eq!(south.zone, -55);
        assert!(
            south.y > 6_000_000.0 && south.y < 7_000_000.0,
            "{}",
            south.y
        );
        let north = UtmPos::from_lat_lng(35.363_261, 149.165_230);
        assert!((north.y - (10_000_000.0 - south.y)).abs() < 1e-6);
        assert!((north.x - south.x).abs() < 1e-6);
    }

    #[test]
    fn adjust_lon_wraps_into_plus_minus_pi() {
        assert_eq!(adjust_lon(1.0), 1.0);
        assert!((adjust_lon(PI + 0.5) - (0.5 - PI)).abs() < 1e-15);
        assert!((adjust_lon(-PI - 0.5) - (PI - 0.5)).abs() < 1e-15);
        assert!(adjust_lon(7.0 * PI).abs() <= PI);
    }
}
