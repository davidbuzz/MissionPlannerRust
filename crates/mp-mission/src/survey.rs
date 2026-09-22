//! Survey grid generation.
//!
//! Replaces `ExtLibs/Utilities/Grid.cs` (1,013 lines). Given a polygon and a line spacing, produce
//! the lawnmower pattern that covers it.
//!
//! # Projection
//!
//! The reference implementation works in UTM. This works in a local tangent plane centred on the
//! polygon: east-north metres, with longitude scaled by the cosine of the centre latitude. Over a
//! survey-sized area the two agree to well under a metre, and the tangent plane has no zone
//! boundaries to straddle - a survey that crosses a UTM zone edge is a real situation that the
//! reference handles by picking the first vertex's zone and living with the distortion.
//!
//! This is a deliberate divergence, recorded rather than hidden. Grid output is therefore **not**
//! byte-identical to Mission Planner's, and the tests assert geometric properties - coverage,
//! spacing, containment - rather than matching a golden file.

use mp_units::LatLon;

/// Why a grid could not be generated.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum GridError {
    /// Fewer than three vertices cannot enclose an area.
    #[error("a survey area needs at least 3 vertices, found {found}")]
    NotAPolygon {
        /// Vertices given.
        found: usize,
    },
    /// Spacing must be positive and finite.
    #[error("line spacing must be a positive number of metres")]
    BadSpacing,
    /// The polygon encloses no area.
    #[error("the survey area has zero extent")]
    DegenerateArea,
}

/// How the survey should be flown.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridOptions {
    /// Distance between adjacent lines, in metres.
    pub spacing: f64,
    /// Bearing of the lines, degrees clockwise from north.
    pub angle: f64,
    /// Extra distance flown past each end of a line, in metres.
    ///
    /// A camera survey needs this: the aircraft must be straight and level at the first photo, and
    /// it is still turning at the polygon edge.
    pub overshoot: f64,
    /// Altitude above home for every waypoint, in metres.
    pub altitude: f64,
}

impl Default for GridOptions {
    fn default() -> Self {
        Self {
            spacing: 50.0,
            angle: 0.0,
            overshoot: 0.0,
            altitude: 50.0,
        }
    }
}

/// Metres east and north of the projection origin.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Enu {
    east: f64,
    north: f64,
}

/// Metres per degree of latitude on the WGS84 ellipsoid, near enough for a local plane.
const METRES_PER_DEGREE_LAT: f64 = 111_320.0;

/// Projects to and from a local tangent plane.
struct Plane {
    origin_lat: f64,
    origin_lon: f64,
    metres_per_degree_lon: f64,
}

impl Plane {
    fn new(origin: LatLon) -> Self {
        Self {
            origin_lat: origin.latitude(),
            origin_lon: origin.longitude(),
            // Longitude lines converge toward the poles; at 60 degrees a degree of longitude is
            // half what it is at the equator.
            metres_per_degree_lon: METRES_PER_DEGREE_LAT * origin.latitude().to_radians().cos(),
        }
    }

    fn to_plane(&self, position: LatLon) -> Enu {
        Enu {
            east: (position.longitude() - self.origin_lon) * self.metres_per_degree_lon,
            north: (position.latitude() - self.origin_lat) * METRES_PER_DEGREE_LAT,
        }
    }

    fn to_geographic(&self, point: Enu) -> Option<LatLon> {
        let lat = point.north / METRES_PER_DEGREE_LAT + self.origin_lat;
        let lon = if self.metres_per_degree_lon.abs() < 1e-6 {
            self.origin_lon
        } else {
            point.east / self.metres_per_degree_lon + self.origin_lon
        };
        LatLon::new(lat, lon).ok()
    }
}

/// Generates the survey pattern covering `polygon`.
///
/// Returns the waypoints in flight order: down one line, across, back along the next.
pub fn grid(polygon: &[LatLon], options: &GridOptions) -> Result<Vec<LatLon>, GridError> {
    if polygon.len() < 3 {
        return Err(GridError::NotAPolygon {
            found: polygon.len(),
        });
    }
    if !(options.spacing.is_finite() && options.spacing > 0.0) {
        return Err(GridError::BadSpacing);
    }

    // Project into a plane centred on the polygon, so distortion is smallest where the work is.
    let centre = centroid(polygon)?;
    let plane = Plane::new(centre);
    let points: Vec<Enu> = polygon.iter().map(|p| plane.to_plane(*p)).collect();

    // Rotate so the survey lines run east-west in rotated space; generating axis-aligned lines and
    // rotating back is simpler and more numerically stable than intersecting arbitrary lines.
    let theta = options.angle.to_radians();
    let (sin, cos) = theta.sin_cos();
    let rotate = |p: Enu| Enu {
        east: p.east * cos - p.north * sin,
        north: p.east * sin + p.north * cos,
    };
    let unrotate = |p: Enu| Enu {
        east: p.east * cos + p.north * sin,
        north: -p.east * sin + p.north * cos,
    };

    let rotated: Vec<Enu> = points.iter().map(|p| rotate(*p)).collect();

    let min_north = rotated
        .iter()
        .map(|p| p.north)
        .fold(f64::INFINITY, f64::min);
    let max_north = rotated
        .iter()
        .map(|p| p.north)
        .fold(f64::NEG_INFINITY, f64::max);
    if !(min_north.is_finite() && max_north.is_finite()) || (max_north - min_north) < 1e-6 {
        return Err(GridError::DegenerateArea);
    }

    // Start half a spacing inside the edge, so the first and last lines sit within the area rather
    // than along its boundary where a camera would photograph mostly the outside.
    let mut lines: Vec<Vec<Enu>> = Vec::new();
    let mut north = min_north + options.spacing / 2.0;
    let mut flip = false;

    while north <= max_north {
        let mut crossings = horizontal_crossings(&rotated, north);
        crossings.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        // Crossings pair up into inside-segments. An odd count means the line grazed a vertex;
        // dropping the stray is better than joining two disjoint parts of a concave shape.
        for pair in crossings.chunks_exact(2) {
            let (Some(from), Some(to)) = (pair.first(), pair.last()) else {
                continue;
            };
            let (mut start, mut end) = (from - options.overshoot, to + options.overshoot);
            if flip {
                std::mem::swap(&mut start, &mut end);
            }
            lines.push(vec![Enu { east: start, north }, Enu { east: end, north }]);
        }

        flip = !flip;
        north += options.spacing;
    }

    let mut out = Vec::with_capacity(lines.len() * 2);
    for line in lines {
        for point in line {
            if let Some(position) = plane.to_geographic(unrotate(point)) {
                out.push(position);
            }
        }
    }
    Ok(out)
}

/// Where a horizontal line at `north` crosses the polygon's edges.
fn horizontal_crossings(polygon: &[Enu], north: f64) -> Vec<f64> {
    let mut out = Vec::new();
    for index in 0..polygon.len() {
        let Some(a) = polygon.get(index) else {
            continue;
        };
        let Some(b) = polygon.get((index + 1) % polygon.len()) else {
            continue;
        };

        // Half-open test: a vertex belongs to exactly one of its two edges, which is what stops a
        // line through a vertex from counting the crossing twice and inverting inside/outside.
        let crosses =
            (a.north <= north && b.north > north) || (b.north <= north && a.north > north);
        if !crosses {
            continue;
        }
        let span = b.north - a.north;
        if span.abs() < f64::EPSILON {
            continue;
        }
        let t = (north - a.north) / span;
        out.push(a.east + t * (b.east - a.east));
    }
    out
}

/// Area centroid, which is where the projection distorts least.
fn centroid(polygon: &[LatLon]) -> Result<LatLon, GridError> {
    #[allow(clippy::cast_precision_loss)] // vertex counts are small
    let count = polygon.len() as f64;
    let lat = polygon.iter().map(|p| p.latitude()).sum::<f64>() / count;
    let lon = polygon.iter().map(|p| p.longitude()).sum::<f64>() / count;
    LatLon::new(lat, lon).map_err(|_| GridError::DegenerateArea)
}

/// Whether a position lies within `tolerance` metres of being inside the polygon.
///
/// Strict containment is the wrong question for survey geometry: a grid line clipped to the area
/// ends *exactly on* the boundary, and so does every vertex. Asking "inside, or on the edge" is
/// what callers actually mean, whether they are validating a waypoint against a geofence or
/// checking a generated grid.
#[must_use]
pub fn contains_within(polygon: &[LatLon], position: LatLon, tolerance: f64) -> bool {
    contains(polygon, position) || distance_to_boundary(polygon, position) <= tolerance
}

/// Shortest distance in metres from a position to the polygon's boundary.
#[must_use]
pub fn distance_to_boundary(polygon: &[LatLon], position: LatLon) -> f64 {
    if polygon.is_empty() {
        return f64::INFINITY;
    }
    let plane = Plane::new(position);
    let point = plane.to_plane(position);

    let mut nearest = f64::INFINITY;
    for index in 0..polygon.len() {
        let (Some(a), Some(b)) = (polygon.get(index), polygon.get((index + 1) % polygon.len()))
        else {
            continue;
        };
        let (a, b) = (plane.to_plane(*a), plane.to_plane(*b));
        nearest = nearest.min(distance_to_segment(point, a, b));
    }
    nearest
}

/// Distance from a point to a line segment, in the plane's metres.
fn distance_to_segment(point: Enu, a: Enu, b: Enu) -> f64 {
    let (dx, dy) = (b.east - a.east, b.north - a.north);
    let length_squared = dx.mul_add(dx, dy * dy);
    if length_squared < f64::EPSILON {
        return (point.east - a.east).hypot(point.north - a.north);
    }
    let t = (((point.east - a.east) * dx) + ((point.north - a.north) * dy)) / length_squared;
    let t = t.clamp(0.0, 1.0);
    let closest = Enu {
        east: a.east + t * dx,
        north: a.north + t * dy,
    };
    (point.east - closest.east).hypot(point.north - closest.north)
}

/// Whether a position lies strictly inside a polygon, by ray casting.
///
/// Points exactly on the boundary are not inside; use [`contains_within`] when the edge counts.
#[must_use]
pub fn contains(polygon: &[LatLon], position: LatLon) -> bool {
    let mut inside = false;
    for index in 0..polygon.len() {
        let Some(a) = polygon.get(index) else {
            continue;
        };
        let Some(b) = polygon.get((index + 1) % polygon.len()) else {
            continue;
        };

        let (ay, by) = (a.latitude(), b.latitude());
        let (ax, bx) = (a.longitude(), b.longitude());
        let y = position.latitude();
        if (ay > y) != (by > y) {
            let span = by - ay;
            if span.abs() < f64::EPSILON {
                continue;
            }
            let x = ax + (y - ay) / span * (bx - ax);
            if position.longitude() < x {
                inside = !inside;
            }
        }
    }
    inside
}
