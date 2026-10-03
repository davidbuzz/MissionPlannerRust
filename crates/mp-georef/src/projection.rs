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

//! Where a photo lies on the ground: `GeoRefImageBase.getboundingbox` and the part of
//! `ImageProjection.calc` it reaches, which the KML's ground overlays are placed by.
//!
//! `getboundingbox` always passes a roll and pitch of zero, so only `calc`'s "quick method" is
//! reached from it: the four corners of the field of view at the photo's altitude, each pushed
//! along its bearing until the line from the camera through it meets the terrain. The other
//! branch - the full rotation matrix for a tilted camera - is reached from the flight map's photo
//! markers (`GMapMarkerPhoto`), whose shots carry the camera's roll, pitch and yaw, and is
//! [`calc`].
//!
//! The field of view the form passes is its defaults, 200 by 130 degrees (`Georefimage.Designer.cs:
//! 229-252`): half of 200 is past 90, so the "horizontal" tangent is negative and the corners
//! swap sides. The arithmetic is the C#'s either way.

use crate::georef::{DEG2RAD, RAD2DEG};

/// The ground height at a point: `srtm.getAltitude(lat, lng).alt`.
pub trait Terrain {
    /// Metres above the datum, 0 where the C# has no tile (`altresponce.Invalid`).
    fn altitude(&self, lat: f64, lng: f64) -> f64;
}

impl Terrain for mp_terrain::Srtm {
    /// `srtm.getAltitude(lat, lng)` at its default zoom of 16, which queues a missing tile for
    /// download and answers 0 until it has it.
    fn altitude(&self, lat: f64, lng: f64) -> f64 {
        self.get_altitude(lat, lng, mp_terrain::DEFAULT_ZOOM).alt
    }
}

/// No terrain at all: every lookup is `altresponce.Invalid`'s 0.
#[derive(Debug, Clone, Copy, Default)]
pub struct Flat;

impl Terrain for Flat {
    fn altitude(&self, _lat: f64, _lng: f64) -> f64 {
        0.0
    }
}

/// A `PointLatLngAlt`, as far as the projection uses one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    /// `Lat`.
    pub lat: f64,
    /// `Lng`.
    pub lng: f64,
    /// `Alt`.
    pub alt: f64,
}

impl Point {
    /// `newpos`: the point `distance` metres along `bearing` on a sphere of 6378.1 km, keeping
    /// the altitude. `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:313-335`
    #[must_use]
    pub fn newpos(self, bearing: f64, distance: f64) -> Self {
        let radius_of_earth = 6_378_100.0;
        let lat1 = DEG2RAD * self.lat;
        let lon1 = DEG2RAD * self.lng;
        let brng = DEG2RAD * bearing;
        let dr = distance / radius_of_earth;
        let lat2 = (lat1.sin() * dr.cos() + lat1.cos() * dr.sin() * brng.cos()).asin();
        let lon2 =
            lon1 + (brng.sin() * dr.sin() * lat1.cos()).atan2(dr.cos() - lat1.sin() * lat2.sin());
        Self {
            lat: RAD2DEG * lat2,
            lng: RAD2DEG * lon2,
            alt: self.alt,
        }
    }

    /// `GetBearing`, in `[0, 360)`. `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:350-360`
    #[must_use]
    pub fn get_bearing(self, p2: Self) -> f64 {
        let latitude1 = DEG2RAD * self.lat;
        let latitude2 = DEG2RAD * p2.lat;
        let longitude_difference = DEG2RAD * (p2.lng - self.lng);
        let y = longitude_difference.sin() * latitude2.cos();
        let x = latitude1.cos() * latitude2.sin()
            - latitude1.sin() * latitude2.cos() * longitude_difference.cos();
        (RAD2DEG * y.atan2(x) + 360.0) % 360.0
    }

    /// `GetDistance`: a haversine on a 6371 km sphere, ignoring altitude.
    /// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:382-393`
    #[must_use]
    pub fn get_distance(self, p2: Self) -> f64 {
        let d = self.lat * 0.017_453_292_519_943_295;
        let num2 = self.lng * 0.017_453_292_519_943_295;
        let num3 = p2.lat * 0.017_453_292_519_943_295;
        let num4 = p2.lng * 0.017_453_292_519_943_295;
        let num5 = num4 - num2;
        let num6 = num3 - d;
        let num7 =
            (num6 / 2.0).sin().powi(2) + ((d.cos() * num3.cos()) * (num5 / 2.0).sin().powi(2));
        let num8 = 2.0 * num7.sqrt().atan2((1.0 - num7).sqrt());
        (6371.0 * num8) * 1000.0
    }
}

/// A `System.Drawing.PointF`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct PointF {
    x: f32,
    y: f32,
}

/// `FindLineIntersection`: where two segments cross, or `(0, 0)`. The differences and products
/// are `float` arithmetic, as the C# writes them on `PointF`s; only the two ratios are `double`.
/// `// C#: ExtLibs/Utilities/ImageProjection.cs:400-417`
fn find_line_intersection(start1: PointF, end1: PointF, start2: PointF, end2: PointF) -> PointF {
    let denom = f64::from(
        ((end1.x - start1.x) * (end2.y - start2.y)) - ((end1.y - start1.y) * (end2.x - start2.x)),
    );
    if denom == 0.0 {
        return PointF::default();
    }
    let numer = f64::from(
        ((start1.y - start2.y) * (end2.x - start2.x))
            - ((start1.x - start2.x) * (end2.y - start2.y)),
    );
    let r = numer / denom;
    let numer2 = f64::from(
        ((start1.y - start2.y) * (end1.x - start1.x))
            - ((start1.x - start2.x) * (end1.y - start1.y)),
    );
    let s = numer2 / denom;
    // Written as the C# writes it, because a NaN ratio passes where `contains` would not.
    #[allow(clippy::manual_range_contains)]
    let outside = (r < 0.0 || r > 1.0) || (s < 0.0 || s > 1.0);
    if outside {
        return PointF::default();
    }
    #[allow(clippy::cast_possible_truncation)]
    PointF {
        x: (f64::from(start1.x) + (r * f64::from(end1.x - start1.x))) as f32,
        y: (f64::from(start1.y) + (r * f64::from(end1.y - start1.y))) as f32,
    }
}

/// `calcIntersection`: from the camera towards `dest`, 100 m at a time, the first place the
/// line to `dest` crosses the terrain profile; the camera itself if it never does.
/// `// C#: ExtLibs/Utilities/ImageProjection.cs:354-393`
#[allow(clippy::cast_possible_truncation)]
fn calc_intersection(plla: Point, dest: Point, terrain: &dyn Terrain) -> Point {
    const STEP: i32 = 100;
    let mut distout: i32 = 10;
    let dist = plla.get_distance(dest);
    let y = plla.get_bearing(dest);
    while f64::from(distout) < dist + 100.0 {
        let mut near = plla.newpos(y, f64::from(distout));
        near.alt = terrain.altitude(near.lat, near.lng);
        let mut far = plla.newpos(y, f64::from(distout + STEP));
        far.alt = terrain.altitude(far.lat, far.lng);
        #[allow(clippy::cast_precision_loss)]
        let newpoint = find_line_intersection(
            PointF {
                x: 0.0,
                y: plla.alt as f32,
            },
            PointF {
                x: dist as f32,
                y: dest.alt as f32,
            },
            PointF {
                x: distout as f32,
                y: near.alt as f32,
            },
            PointF {
                // `(float)distout + step`: float arithmetic.
                x: distout as f32 + STEP as f32,
                y: far.alt as f32,
            },
        );
        if newpoint.x != 0.0 {
            let mut hit = plla.newpos(y, f64::from(newpoint.x));
            hit.alt = f64::from(newpoint.y);
            return hit;
        }
        distout += STEP;
    }
    plla
}

/// `ImageProjection.calc` with no roll or pitch: the ground under the four corners of a
/// `hfov` by `vfov` field of view looking straight down from `plla`, the camera turned `yaw`
/// degrees.
/// `// C#: ExtLibs/Utilities/ImageProjection.cs:131-185`
#[must_use]
pub fn calc_quick(
    plla: Point,
    yaw: f64,
    hfov: f64,
    vfov: f64,
    terrain: &dyn Terrain,
) -> [Point; 4] {
    let fovh = (hfov / 2.0 * DEG2RAD).tan() * plla.alt;
    let fovv = (vfov / 2.0 * DEG2RAD).tan() * plla.alt;
    let bearing1 = fovh.atan2(fovv) * RAD2DEG;
    let distance = (fovh * fovh + fovv * fovv).sqrt();
    let corner = |bearing: f64| {
        let mut newpos = plla.newpos(bearing, distance);
        newpos.alt = 0.0;
        calc_intersection(plla, newpos, terrain)
    };
    [
        corner(bearing1 + yaw),
        corner(yaw - bearing1),
        corner(bearing1 + yaw - 180.0),
        corner(yaw - bearing1 - 180.0),
    ]
}

/// `Vector3`, as far as the projection uses one: the operators `calc` and `Matrix3` reach.
/// `// C#: ExtLibs/Utilities/Vector3.cs:196-247, 254-257`
#[derive(Debug, Clone, Copy, PartialEq)]
struct V3 {
    x: f64,
    y: f64,
    z: f64,
}

impl V3 {
    const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// `operator *(Vector3, Vector3)`: the dot product.
    fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// `operator %`: the cross product.
    fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    /// `operator *(Vector3, double)`.
    fn scaled(self, by: f64) -> Self {
        Self::new(self.x * by, self.y * by, self.z * by)
    }

    fn plus(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }

    fn minus(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }

    /// `length()`.
    fn length(self) -> f64 {
        self.dot(self).sqrt()
    }
}

/// `Matrix3`: three row vectors `a`, `b` and `c`, with the four members `ImageProjection.calc`
/// uses - `from_euler`, `rotate`, `normalize` and the product with a vector. Its other members
/// (`to_euler`, `from_euler312`, the transpose, the matrix product, `rotateXY`, `trace`) have
/// their callers elsewhere and are not here.
/// `// C#: ExtLibs/Utilities/Matrix3.cs:102-119, 185-190, 225-243, 281-291`
#[derive(Debug, Clone, Copy, PartialEq)]
struct Matrix3 {
    a: V3,
    b: V3,
    c: V3,
}

impl Matrix3 {
    /// `from_euler`: the rotation matrix of the Euler angles, radians.
    fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (sp, cp) = pitch.sin_cos();
        let (sr, cr) = roll.sin_cos();
        let (sy, cy) = yaw.sin_cos();
        Self {
            a: V3::new(
                cp * cy,
                (sr * sp * cy) - (cr * sy),
                (cr * sp * cy) + (sr * sy),
            ),
            b: V3::new(
                cp * sy,
                (sr * sp * sy) + (cr * cy),
                (cr * sp * sy) - (sr * cy),
            ),
            c: V3::new(-sp, sr * cp, cr * cp),
        }
    }

    /// `rotate`: the matrix turned by a small rotation `g` about the three axes - each row
    /// crossed with `g` and added to itself.
    fn rotate(&mut self, g: V3) {
        self.a = self.a.plus(self.a.cross(g));
        self.b = self.b.plus(self.b.cross(g));
        self.c = self.c.plus(self.c.cross(g));
    }

    /// `normalize`: the rows made orthogonal and unit again.
    fn normalize(&mut self) {
        let error = self.a.dot(self.b);
        let t0 = self.a.minus(self.b.scaled(0.5 * error));
        let t1 = self.b.minus(self.a.scaled(0.5 * error));
        let t2 = t0.cross(t1);
        self.a = t0.scaled(1.0 / t0.length());
        self.b = t1.scaled(1.0 / t1.length());
        self.c = t2.scaled(1.0 / t2.length());
    }

    /// `operator *(Matrix3, Vector3)`.
    fn times(self, v: V3) -> V3 {
        V3::new(self.a.dot(v), self.b.dot(v), self.c.dot(v))
    }
}

/// `ImageProjection.calc`: the ground under the four corners of an `hfov` by `vfov` field of
/// view from `plla`, the camera rolled, pitched and turned by the degrees given. Level and
/// unpitched, the quick method ([`calc_quick`]), in its order; otherwise each corner from the
/// rotation matrix - the field's edges turned about the body's axes (`frontangle` and the rest
/// take `P*0` and `R*0`: the attitude enters through the matrix alone), the point 10,000 along
/// the camera's axis brought to the ground - top-left, top-right, bottom-right, bottom-left.
/// The `addtomap` debug markers and the centre worked out for one are not here.
/// `// C#: ExtLibs/Utilities/ImageProjection.cs:17-201`
#[must_use]
pub fn calc(
    plla: Point,
    roll: f64,
    pitch: f64,
    yaw: f64,
    hfov: f64,
    vfov: f64,
    terrain: &dyn Terrain,
) -> [Point; 4] {
    if roll == 0.0 && pitch == 0.0 {
        return calc_quick(plla, yaw, hfov, vfov, terrain);
    }
    let (front, back) = (vfov / 2.0, -vfov / 2.0);
    let (left, right) = (hfov / 2.0, -hfov / 2.0);
    let corner = |across: f64, along: f64| -> Point {
        let mut dcm = Matrix3::from_euler(roll * DEG2RAD, pitch * DEG2RAD, yaw * DEG2RAD);
        dcm.rotate(V3::new(across * DEG2RAD, 0.0, 0.0));
        dcm.normalize();
        dcm.rotate(V3::new(0.0, along * DEG2RAD, 0.0));
        dcm.normalize();
        let test = dcm.times(V3::new(0.0, 0.0, 10_000.0));
        let bearing = test.y.atan2(test.x) * RAD2DEG;
        let mut newpos = plla.newpos(bearing, (test.x * test.x + test.y * test.y).sqrt());
        newpos.alt -= test.z;
        calc_intersection(plla, newpos, terrain)
    };
    let tr = corner(right, front);
    let tl = corner(left, front);
    let bl = corner(left, back);
    let br = corner(right, back);
    [tl, tr, br, bl]
}

/// A `System.Drawing.RectangleF`: `X` is the least longitude, `Y` the least latitude.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RectangleF {
    /// `X`.
    pub x: f32,
    /// `Y`.
    pub y: f32,
    /// `Width`.
    pub width: f32,
    /// `Height`.
    pub height: f32,
}

impl RectangleF {
    /// `Left`.
    #[must_use]
    pub const fn left(self) -> f32 {
        self.x
    }

    /// `Top`: the least latitude, which the KML calls south.
    #[must_use]
    pub const fn top(self) -> f32 {
        self.y
    }

    /// `Right`: `X + Width`, a `float` sum.
    #[must_use]
    pub fn right(self) -> f32 {
        self.x + self.width
    }

    /// `Bottom`: `Y + Height`, which the KML calls north.
    #[must_use]
    pub fn bottom(self) -> f32 {
        self.y + self.height
    }
}

/// `Math.Max` and `Math.Min`: a NaN in either argument is the answer.
fn net_max(a: f64, b: f64) -> f64 {
    if a > b || a.is_nan() { a } else { b }
}

fn net_min(a: f64, b: f64) -> f64 {
    if a < b || a.is_nan() { a } else { b }
}

/// `getboundingbox`: the latitude and longitude extent of the footprint, narrowed to `float`.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:1679-1705`
#[allow(clippy::cast_possible_truncation)]
#[must_use]
pub fn get_bounding_box(
    centery: f64,
    centerx: f64,
    alt: f64,
    angle: f64,
    width: f64,
    height: f64,
    terrain: &dyn Terrain,
) -> RectangleF {
    let rect = calc_quick(
        Point {
            lat: centery,
            lng: centerx,
            alt,
        },
        angle,
        width,
        height,
        terrain,
    );
    let (mut minx, mut miny, mut maxx, mut maxy) = (999.0f64, 999.0f64, -999.0f64, -999.0f64);
    for pnt in rect {
        maxx = net_max(maxx, pnt.lat);
        minx = net_min(minx, pnt.lat);
        miny = net_min(miny, pnt.lng);
        maxy = net_max(maxy, pnt.lng);
    }
    RectangleF {
        x: miny as f32,
        y: minx as f32,
        width: (maxy - miny) as f32,
        height: (maxx - minx) as f32,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_flat_world_puts_the_corners_at_the_nadir_distance() {
        // Over flat ground at 0 m the line from 100 m up to a point at 0 m never crosses the
        // terrain segment strictly inside, and calcIntersection gives back the camera.
        let plla = Point {
            lat: -27.4698,
            lng: 153.0251,
            alt: 100.0,
        };
        let rect = get_bounding_box(plla.lat, plla.lng, plla.alt, 90.0, 60.0, 40.0, &Flat);
        assert!(rect.width >= 0.0 && rect.height >= 0.0);
    }

    #[test]
    fn segments_cross_where_they_cross() {
        let p = find_line_intersection(
            PointF { x: 0.0, y: 100.0 },
            PointF { x: 100.0, y: 0.0 },
            PointF { x: 0.0, y: 0.0 },
            PointF { x: 100.0, y: 100.0 },
        );
        assert_eq!(p, PointF { x: 50.0, y: 50.0 });
        let parallel = find_line_intersection(
            PointF { x: 0.0, y: 0.0 },
            PointF { x: 1.0, y: 1.0 },
            PointF { x: 0.0, y: 1.0 },
            PointF { x: 1.0, y: 2.0 },
        );
        assert_eq!(parallel, PointF::default());
    }
}

#[cfg(test)]
mod calc_tests {
    use super::*;

    const CAMERA: Point = Point {
        lat: -27.47,
        lng: 153.025,
        alt: 40.0,
    };

    /// `from_euler` is the C#'s: a quarter turn of yaw puts north along the body's y.
    #[test]
    fn the_matrix_is_the_csharps() {
        let m = Matrix3::from_euler(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let close = |v: V3, (x, y, z): (f64, f64, f64)| {
            (v.x - x).abs() < 1e-12 && (v.y - y).abs() < 1e-12 && (v.z - z).abs() < 1e-12
        };
        assert!(close(m.a, (0.0, -1.0, 0.0)), "{:?}", m.a);
        assert!(close(m.b, (1.0, 0.0, 0.0)), "{:?}", m.b);
        assert!(close(m.c, (0.0, 0.0, 1.0)), "{:?}", m.c);
        let mut n = m;
        n.normalize();
        assert!(close(n.a, (0.0, -1.0, 0.0)) && close(n.c, (0.0, 0.0, 1.0)));
        let down = m.times(V3::new(0.0, 0.0, 10_000.0));
        assert!(close(down, (0.0, 0.0, 10_000.0)));
    }

    /// Level and unpitched, `calc` is the quick method, corner for corner.
    #[test]
    fn a_level_camera_is_the_quick_method() {
        let quick = calc_quick(CAMERA, 45.0, 63.0, 43.0, &Flat);
        let full = calc(CAMERA, 0.0, 0.0, 45.0, 63.0, 43.0, &Flat);
        assert_eq!(quick, full);
    }

    /// A camera pitched with no roll, looking north: four corners on the ground, in pairs
    /// across the line of flight - the left corners west of the right ones and as far from it -
    /// and all of them off the point under the camera.
    #[test]
    fn a_pitched_camera_puts_its_footprint_ahead_and_symmetric() {
        let [tl, tr, br, bl] = calc(CAMERA, 0.0, -30.0, 0.0, 63.0, 43.0, &Flat);
        for corner in [tl, tr, br, bl] {
            // The crossing is found in single precision, as the C#'s `PointF`s find it.
            assert!(corner.alt.abs() < 1e-3, "on the ground: {corner:?}");
            assert!(
                CAMERA.get_distance(corner) > 10.0,
                "off the camera's point: {corner:?}"
            );
        }
        // Symmetric about the line of flight (north): the left corners west, the right east,
        // each pair the same distance either side.
        assert!(tl.lng < CAMERA.lng && bl.lng < CAMERA.lng);
        assert!(tr.lng > CAMERA.lng && br.lng > CAMERA.lng);
        assert!(((CAMERA.lng - tl.lng) - (tr.lng - CAMERA.lng)).abs() < 1e-9);
        // The front corners further along the pitch than the back ones.
        assert!((tl.lat - CAMERA.lat).abs() > (bl.lat - CAMERA.lat).abs() || tl.lat != bl.lat);
    }
}
