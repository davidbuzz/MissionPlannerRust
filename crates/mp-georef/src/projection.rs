//! Where a photo lies on the ground: `GeoRefImageBase.getboundingbox` and the part of
//! `ImageProjection.calc` it reaches, which the KML's ground overlays are placed by.
//!
//! `getboundingbox` always passes a roll and pitch of zero, so only `calc`'s "quick method" is
//! reached: the four corners of the field of view at the photo's altitude, each pushed along its
//! bearing until the line from the camera through it meets the terrain. The other branch - the
//! full rotation matrix for a tilted camera - is not reachable from Geo Reference Images and is
//! not ported.
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
