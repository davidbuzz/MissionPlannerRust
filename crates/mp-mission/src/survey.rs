//! A survey grid's lane ends, and whether positions lie in an area.
//!
//! [`grid`] is Mission Planner's own `Grid.CreateGrid` ([`crate::grid::create_grid`], a
//! transliteration of `ExtLibs/Utilities/Grid.cs` checked point for point against the C# by
//! `tests/grid_vectors.rs`) with a handful of its settings, returning the lane ends. The planning
//! screen's Survey (Grid) dialog does not use it: it is [`crate::gridui`], `GridUI.cs` whole. The
//! containment helpers below are this crate's own, for checking waypoints against an area; they
//! are not part of `Grid.cs`.

use mp_units::LatLon;

use crate::grid::{GridArgs, GridTag, create_grid};

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
    /// No survey line crosses the area: `CreateGrid` returned nothing (`Grid.cs:555`).
    #[error("the survey area has zero extent")]
    DegenerateArea,
    /// A grid point could not be converted back from UTM, where ProjNet throws for want of
    /// convergence (`MapProjection.cs:792`). Positions a [`LatLon`] accepts always convert.
    #[error("a survey point could not be converted from UTM")]
    Projection,
}

/// How the survey should be flown.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridOptions {
    /// Distance between adjacent lines, in metres: `CreateGrid`'s `distance`.
    pub spacing: f64,
    /// Bearing of the lines, degrees clockwise from north: 0 flies north-south lines, 90
    /// east-west, as in Mission Planner.
    pub angle: f64,
    /// Extra distance flown on past the area at the end of each line, in metres: Mission Planner's
    /// overshoot, both directions (`overshoot1` and `overshoot2`, `Grid.cs:646`, `:712`).
    ///
    /// A camera survey needs this: the aircraft must still be straight and level at the last
    /// photo of a line, and it starts turning as soon as it reaches its waypoint. The matching
    /// run-up at the start of a line is the lead-in, which this screen does not expose.
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

/// Metres per degree of latitude on the WGS84 ellipsoid, near enough for a local plane.
const METRES_PER_DEGREE_LAT: f64 = 111_320.0;

/// Metres east and north of the projection origin.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Enu {
    east: f64,
    north: f64,
}

/// Projects to a local tangent plane, for the containment helpers' distances.
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
}

/// Generates the survey pattern covering `polygon`.
///
/// This is `Grid.CreateGrid` with `options` in place of the settings they name - `spacing` is the
/// distance between lanes, `overshoot` is both overshoots - and `GridUI`'s defaults for the rest
/// ([`GridArgs::default`]): no trigger spacing, no lead-in, adjacent lanes, starting from the
/// polygon vertex nearest the planned home, which is (0, 0) until a home is planned.
///
/// Returns the waypoints in flight order, two per lane: the `S` and `E` points of each lane, which
/// are the points `GridUI` makes waypoints of (`GridUI.cs:1702`). The trigger points between them
/// are for camera commands this screen does not add.
///
/// # Errors
///
/// [`GridError::NotAPolygon`] for fewer than 3 vertices, [`GridError::BadSpacing`] for a spacing
/// that is not a positive number, [`GridError::DegenerateArea`] when no line crosses the area, and
/// [`GridError::Projection`] as for [`create_grid`].
pub fn grid(polygon: &[LatLon], options: &GridOptions) -> Result<Vec<LatLon>, GridError> {
    if polygon.len() < 3 {
        return Err(GridError::NotAPolygon {
            found: polygon.len(),
        });
    }
    // The C# clamps rather than refusing (`Grid.cs:361`), but its dialog cannot go below 0.3 m
    // (`GridUI.Designer.cs:1143`); refusing here is this API's equivalent of that floor.
    if !(options.spacing.is_finite() && options.spacing > 0.0) {
        return Err(GridError::BadSpacing);
    }

    let args = GridArgs {
        altitude: options.altitude,
        distance: options.spacing,
        angle: options.angle,
        overshoot1: options.overshoot,
        overshoot2: options.overshoot,
        ..GridArgs::default()
    };
    let ends = create_grid(polygon, &args)?
        .into_iter()
        .filter(|point| matches!(point.tag, GridTag::Start | GridTag::End))
        .map(|point| LatLon::new(point.lat, point.lng).map_err(|_| GridError::Projection))
        .collect::<Result<Vec<_>, _>>()?;
    if ends.is_empty() {
        return Err(GridError::DegenerateArea);
    }
    Ok(ends)
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
