//! Survey grids as Mission Planner lays them out: `Grid.CreateGrid`, transliterated.
//!
//! Replaces `ExtLibs/Utilities/Grid.cs:327-1010`: `CreateGrid` and every helper it calls. The
//! corridor and rotary patterns in the same file are [`crate::corridor`] and [`crate::rotary`],
//! which share the helpers here that `Grid.cs` shares between them.
//!
//! This is a line-by-line port, not a reimplementation. The C# works in the UTM zone of the
//! polygon's first vertex, lays a family of parallel lines across the polygon's bounding
//! rectangle, clips them against the polygon edges, and then chains them by repeatedly picking the
//! nearest remaining line - and that nearest-line search, with its strict comparisons and
//! insertion-ordered dictionaries, decides the order the survey is flown in. Reproducing the order
//! means reproducing the search, so the structure below follows the C# statement for statement,
//! including the parts that look like accidents (a line crossing the polygon once is kept at full
//! length, `Grid.cs:508`; the home position is projected in its own zone rather than the
//! polygon's, `utmpos.cs:39`). Each is cited where it happens.
//!
//! The result is compared point by point against `CreateGrid` run under mono, by
//! `tests/grid_vectors.rs` over `testdata/grid`.

use mp_units::LatLon;

use crate::survey::GridError;
use crate::utm::{UtmPos, pow2, to_utm, utm_zone};

/// `Grid.cs:11`.
const RAD2DEG: f64 = 180.0 / std::f64::consts::PI;
/// `Grid.cs:12`: the reciprocal of `RAD2DEG`, which is not always the same double as `PI / 180`.
const DEG2RAD: f64 = 1.0 / RAD2DEG;

/// Where the survey starts, `Grid.StartPosition`, `Grid.cs:24-32`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum StartPosition {
    /// The polygon vertex nearest the planned home.
    #[default]
    Home,
    /// The bounding rectangle's bottom-left corner.
    BottomLeft,
    /// The bounding rectangle's top-left corner.
    TopLeft,
    /// The bounding rectangle's bottom-right corner.
    BottomRight,
    /// The bounding rectangle's top-right corner.
    TopRight,
    /// The polygon vertex nearest [`GridArgs::start_point`].
    Point,
}

impl StartPosition {
    /// Every variant, in the C# enum's order, which is the order `CMB_startfrom` lists them.
    pub const ALL: [Self; 6] = [
        Self::Home,
        Self::BottomLeft,
        Self::TopLeft,
        Self::BottomRight,
        Self::TopRight,
        Self::Point,
    ];

    /// The C# enum member's name, which is what `GridUI` stores and parses (`GridUI.cs:613`).
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Home => "Home",
            Self::BottomLeft => "BottomLeft",
            Self::TopLeft => "TopLeft",
            Self::BottomRight => "BottomRight",
            Self::TopRight => "TopRight",
            Self::Point => "Point",
        }
    }

    /// The variant with this C# name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|start| start.name() == name)
    }
}

/// What a grid point is for, the C#'s string `Tag` (`Grid.cs:602-605`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GridTag {
    /// `"S"`: the start of a lane, lead-in included.
    Start,
    /// `"SM"`: where the lane enters the polygon, and triggering starts.
    StartMiddle,
    /// `"M"`: a trigger point inside the lane, only when a trigger spacing is set.
    Middle,
    /// `"ME"`: where the lane leaves the polygon, and triggering ends.
    MiddleEnd,
    /// `"E"`: the end of a lane, overshoot included.
    End,
}

impl GridTag {
    /// The tag as the C# writes it.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Start => "S",
            Self::StartMiddle => "SM",
            Self::Middle => "M",
            Self::MiddleEnd => "ME",
            Self::End => "E",
        }
    }

    /// The tag with this C# spelling.
    #[must_use]
    pub fn from_str_cs(tag: &str) -> Option<Self> {
        [
            Self::Start,
            Self::StartMiddle,
            Self::Middle,
            Self::MiddleEnd,
            Self::End,
        ]
        .into_iter()
        .find(|t| t.as_str() == tag)
    }
}

/// `CreateGrid`'s arguments (`Grid.cs:354`), named as the C# names them.
///
/// [`Default`] gives what `GridUI` passes on a fresh install (`GridUI.cs:609-615`), except the
/// angle: the dialog proposes the bearing of the polygon's longest side (`GridUI.cs:104`), which
/// is the dialog's choice rather than the grid's, so the default here is 0.
///
/// `CreateGrid` also takes `bool shutter`, which it never reads; it has no field here.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridArgs {
    /// Altitude given to every point, metres. `NUM_altitude`, default 100.
    pub altitude: f64,
    /// Distance between lanes, metres. `NUM_Distance`, default 50; below 0.1 it is 0.1.
    pub distance: f64,
    /// Distance between trigger points along a lane, metres, or 0 for none. `NUM_spacing`,
    /// default 0; a non-zero value below 0.1 is 0.1.
    pub spacing: f64,
    /// Bearing of the lanes, degrees clockwise from grid north: 0 flies north-south lanes.
    pub angle: f64,
    /// Distance flown past the polygon at the end of a lane flown in the `angle` direction.
    /// `NUM_overshoot`, default 0.
    pub overshoot1: f64,
    /// The same for a lane flown against `angle`. `NUM_overshoot2`, default 0.
    pub overshoot2: f64,
    /// Which corner or vertex the survey starts from. `CMB_startfrom`, default Home.
    pub startpos: StartPosition,
    /// Lanes to skip between passes, so a plane can turn; 0 flies adjacent lanes. `NUM_Lane_Dist`.
    pub min_lane_separation: f32,
    /// Distance before the polygon at the start of a lane flown in the `angle` direction.
    /// `NUM_leadin`, default 0.
    pub leadin1: f32,
    /// The same for a lane flown against `angle`. `NUM_leadin2`, default 0.
    pub leadin2: f32,
    /// The planned home, `MAV.cs.PlannedHomeLocation`. It is (0, 0) until a home is planned
    /// (`CurrentState.cs:41`), and `CreateGrid` uses whatever it is given.
    pub home: LatLon,
    /// `Grid.StartPointLatLngAlt` (`Grid.cs:34`), a static the C# reads for
    /// [`StartPosition::Point`].
    pub start_point: LatLon,
    /// Pick the next lane from the end of the overshoot rather than from the polygon edge.
    /// `chk_optimize_for_distance`, unchecked by default.
    pub use_extended_endpoint: bool,
}

impl Default for GridArgs {
    fn default() -> Self {
        Self {
            altitude: 100.0,
            distance: 50.0,
            spacing: 0.0,
            angle: 0.0,
            overshoot1: 0.0,
            overshoot2: 0.0,
            startpos: StartPosition::Home,
            min_lane_separation: 0.0,
            leadin1: 0.0,
            leadin2: 0.0,
            home: LatLon::default(),
            start_point: LatLon::default(),
            use_extended_endpoint: false,
        }
    }
}

/// One point of a grid, the C#'s `PointLatLngAlt` with its `Tag`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridPoint {
    /// Latitude, degrees.
    pub lat: f64,
    /// Longitude, degrees.
    pub lng: f64,
    /// Altitude, metres: always [`GridArgs::altitude`].
    pub alt: f64,
    /// What the point is for.
    pub tag: GridTag,
}

/// `Grid.linelatlng`, `Grid.cs:14-22`.
#[derive(Debug, Clone, Copy)]
struct Line {
    /// Start of the line.
    p1: UtmPos,
    /// End of the line.
    p2: UtmPos,
    /// Where the unclipped line started: trigger points are laid out from here, so they line up
    /// across lanes.
    basepnt: UtmPos,
}

impl Line {
    /// `ValueType.Equals` on `linelatlng`, which is what `List.Remove` uses: field by field, each
    /// field through `utmpos.Equals`, so positions compare and zones and tags do not.
    fn equals(&self, other: &Self) -> bool {
        self.p1.equals(other.p1) && self.p2.equals(other.p2) && self.basepnt.equals(other.basepnt)
    }
}

/// `List<linelatlng>.Remove`: the first equal element, if any.
fn remove_line(list: &mut Vec<Line>, line: &Line) {
    if let Some(index) = list.iter().position(|l| l.equals(line)) {
        list.remove(index);
    }
}

/// `List<utmpos>.Remove`: the first equal element. Whether one was removed, for the caller that
/// would otherwise loop forever.
fn remove_pos(list: &mut Vec<UtmPos>, pos: UtmPos) -> bool {
    match list.iter().position(|p| p.equals(pos)) {
        Some(index) => {
            list.remove(index);
            true
        }
        None => false,
    }
}

/// `Math.Min(double, double)` as .NET Framework and mono define it: NaN in the first argument
/// wins, and ties return the second.
fn cs_min(val1: f64, val2: f64) -> f64 {
    if val1 < val2 || val1.is_nan() {
        val1
    } else {
        val2
    }
}

/// `Math.Max(double, double)`, likewise.
fn cs_max(val1: f64, val2: f64) -> f64 {
    if val1 > val2 || val1.is_nan() {
        val1
    } else {
        val2
    }
}

/// `(int)x` as mono 6.12 compiles it on x86-64, `cvttsd2si`: toward zero, and `int.MinValue` for
/// NaN or anything out of range - where Rust's `as` saturates and sends NaN to 0. Probed under the
/// oracle's mono: `(int)1e12`, `(int)-1e12` and `(int)double.NaN` are all -2147483648.
#[allow(clippy::cast_possible_truncation)] // in range, `as` truncates toward zero as the C# does
pub(crate) fn cs_int(x: f64) -> i32 {
    if x > -2_147_483_649.0 && x < 2_147_483_648.0 {
        x as i32
    } else {
        i32::MIN
    }
}

/// `(long)x` likewise: `long.MinValue` for NaN or out of range (`(long)1e19` under the oracle's
/// mono is -9223372036854775808, as is `(long)double.NaN`).
#[allow(clippy::cast_possible_truncation)] // in range, `as` truncates toward zero as the C# does
pub(crate) fn cs_long(x: f64) -> i64 {
    // -2^63 is a double; 2^63 is the first one past the end.
    if (-9_223_372_036_854_775_808.0..9_223_372_036_854_775_808.0).contains(&x) {
        x as i64
    } else {
        i64::MIN
    }
}

/// `Rect`, `ExtLibs/Utilities/Rect.cs`.
///
/// `Right` and `Bottom` are stored as `Left + Width` and `Top + Height`, which is not always the
/// same double as the maximum they were computed from; the grid's start line depends on it.
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct Rect {
    pub(crate) top: f64,
    pub(crate) bottom: f64,
    pub(crate) left: f64,
    pub(crate) right: f64,
}

impl Rect {
    /// `Rect.cs:21-27`.
    fn new(left: f64, top: f64, width: f64, height: f64) -> Self {
        Self {
            top,
            bottom: top + height,
            left,
            right: left + width,
        }
    }

    fn width(&self) -> f64 {
        self.right - self.left
    }

    fn height(&self) -> f64 {
        self.top - self.bottom
    }

    fn mid_width(&self) -> f64 {
        ((self.right - self.left) / 2.0) + self.left
    }

    fn mid_height(&self) -> f64 {
        ((self.top - self.bottom) / 2.0) + self.bottom
    }

    /// `Rect.cs:29-33`.
    fn diag_distance(&self) -> f64 {
        (pow2(self.width()) + pow2(self.height())).sqrt()
    }
}

/// `Grid.getPolyMinMax`, `Grid.cs:750-770`.
pub(crate) fn get_poly_min_max(utmpos: &[UtmPos]) -> Rect {
    let Some(first) = utmpos.first() else {
        return Rect::default();
    };
    let (mut minx, mut maxx) = (first.x, first.x);
    let (mut miny, mut maxy) = (first.y, first.y);
    for pnt in utmpos {
        minx = cs_min(minx, pnt.x);
        maxx = cs_max(maxx, pnt.x);

        miny = cs_min(miny, pnt.y);
        maxy = cs_max(maxy, pnt.y);
    }
    Rect::new(minx, maxy, maxx - minx, miny - maxy)
}

/// `Grid.newpos(ref x, ref y, bearing, distance)`, `Grid.cs:773-780`: polar to rectangular.
pub(crate) fn newpos_xy(x: &mut f64, y: &mut f64, bearing: f64, distance: f64) {
    let mut deg_n = 90.0 - bearing;
    if deg_n < 0.0 {
        deg_n += 360.0;
    }
    *x += distance * (deg_n * DEG2RAD).cos();
    *y += distance * (deg_n * DEG2RAD).sin();
}

/// `Grid.newpos(utmpos, bearing, distance)`, `Grid.cs:783-792`.
pub(crate) fn newpos(input: UtmPos, bearing: f64, distance: f64) -> UtmPos {
    let (mut x, mut y) = (input.x, input.y);
    newpos_xy(&mut x, &mut y, bearing, distance);
    UtmPos::new(x, y, input.zone)
}

/// `Grid.FindLineIntersection`, `Grid.cs:802-820`: where two segments cross, or
/// [`UtmPos::ZERO`] if they do not.
// The C#'s `r < 0 || r > 1` lets a NaN through where `!(0.0..=1.0).contains(&r)` would not.
#[allow(clippy::manual_range_contains)]
fn find_line_intersection(start1: UtmPos, end1: UtmPos, start2: UtmPos, end2: UtmPos) -> UtmPos {
    let denom =
        ((end1.x - start1.x) * (end2.y - start2.y)) - ((end1.y - start1.y) * (end2.x - start2.x));
    // AB & CD are parallel
    if denom == 0.0 {
        return UtmPos::ZERO;
    }
    let numer = ((start1.y - start2.y) * (end2.x - start2.x))
        - ((start1.x - start2.x) * (end2.y - start2.y));
    let r = numer / denom;
    let numer2 = ((start1.y - start2.y) * (end1.x - start1.x))
        - ((start1.x - start2.x) * (end1.y - start1.y));
    let s = numer2 / denom;
    if (r < 0.0 || r > 1.0) || (s < 0.0 || s > 1.0) {
        return UtmPos::ZERO;
    }
    UtmPos::new(
        start1.x + (r * (end1.x - start1.x)),
        start1.y + (r * (end1.y - start1.y)),
        start1.zone,
    )
}

/// `Grid.FindLineIntersectionExtension`, `Grid.cs:830-852`: where the two lines through the
/// segments cross, wherever that is, or [`UtmPos::ZERO`] if they are parallel.
pub(crate) fn find_line_intersection_extension(
    start1: UtmPos,
    end1: UtmPos,
    start2: UtmPos,
    end2: UtmPos,
) -> UtmPos {
    let denom =
        ((end1.x - start1.x) * (end2.y - start2.y)) - ((end1.y - start1.y) * (end2.x - start2.x));
    // AB & CD are parallel
    if denom == 0.0 {
        return UtmPos::ZERO;
    }
    let numer = ((start1.y - start2.y) * (end2.x - start2.x))
        - ((start1.x - start2.x) * (end2.y - start2.y));
    let r = numer / denom;
    // The C# also computes `s` and tests both against [0, 1], with an empty body.
    UtmPos::new(
        start1.x + (r * (end1.x - start1.x)),
        start1.y + (r * (end1.y - start1.y)),
        start1.zone,
    )
}

/// `Grid.findClosestPoint`, `Grid.cs:854-871`: the first of the nearest, or [`UtmPos::ZERO`] for
/// an empty list.
pub(crate) fn find_closest_point(start: UtmPos, list: &[UtmPos]) -> UtmPos {
    let mut answer = UtmPos::ZERO;
    let mut currentbest = f64::MAX;
    for pnt in list {
        let dist1 = start.distance(*pnt);
        if dist1 < currentbest {
            answer = *pnt;
            currentbest = dist1;
        }
    }
    answer
}

/// `Grid.AddAngle`, `Grid.cs:874-885`.
fn add_angle(angle: f64, degrees: f64) -> f64 {
    let mut angle = angle + degrees;
    angle %= 360.0;
    while angle < 0.0 {
        angle += 360.0;
    }
    angle
}

/// `double.Equals(double)`, the key comparison of `Dictionary<double, _>`: NaN equals NaN.
fn double_key_equals(a: f64, b: f64) -> bool {
    a == b || (a.is_nan() && b.is_nan())
}

/// `double.CompareTo`, the order `List<double>.Sort` uses: NaN sorts first.
fn double_compare(a: f64, b: f64) -> std::cmp::Ordering {
    use std::cmp::Ordering;
    if a < b {
        Ordering::Less
    } else if a > b {
        Ordering::Greater
    } else if a == b {
        Ordering::Equal
    } else if a.is_nan() {
        if b.is_nan() {
            Ordering::Equal
        } else {
            Ordering::Less
        }
    } else {
        Ordering::Greater
    }
}

/// `Grid.findClosestLine`, `Grid.cs:887-972`: the next lane to fly. `None` only for an empty list,
/// where the C# throws.
///
/// With no minimum separation it is the line with the nearest endpoint, the first one on a tie.
/// With one, it draws a perpendicular through `start`, measures along it to every line, and takes
/// the nearest line at least `min_distance` away - by way of two dictionaries whose iteration order
/// is insertion order, which is what makes the tie-breaks reproducible.
fn find_closest_line(start: UtmPos, list: &[Line], min_distance: f64, angle: f64) -> Option<Line> {
    if min_distance == 0.0 {
        let mut answer = *list.first()?;
        let mut shortest = f64::MAX;

        for line in list {
            let ans1 = start.distance(line.p1);
            let ans2 = start.distance(line.p2);
            let shorterpnt = if ans1 < ans2 { line.p1 } else { line.p2 };

            if shortest > start.distance(shorterpnt) {
                answer = *line;
                shortest = start.distance(shorterpnt);
            }
        }

        return Some(answer);
    }

    // By now, just add 5.000 km to our lines so they are long enough to allow intersection
    const METERS_TO_EXTEND: f64 = 5000.0;

    let perperndicular_orientation = add_angle(angle, 90.0);

    let start_perpendicular_line = newpos(start, perperndicular_orientation, -METERS_TO_EXTEND);
    let stop_perpendicular_line = newpos(start, perperndicular_orientation, METERS_TO_EXTEND);

    // `Dictionary<utmpos, linelatlng>`: a repeated key keeps its first slot and takes the new
    // line. Keys match through `utmpos.Equals` among those with equal hash codes, and the hash
    // includes the zone (`utmpos.cs:131-140`).
    let mut intersected_points: Vec<(UtmPos, Line)> = Vec::new();
    // The keys of `Dictionary<double, utmpos>`; its values are never read.
    let mut ordered_keys: Vec<f64> = Vec::new();

    for line in list {
        let p = find_line_intersection_extension(
            line.p1,
            line.p2,
            start_perpendicular_line,
            stop_perpendicular_line,
        );

        match intersected_points
            .iter_mut()
            .find(|(key, _)| key.equals(p) && key.zone == p.zone)
        {
            Some(entry) => entry.1 = *line,
            None => intersected_points.push((p, *line)),
        }

        let distance_p = start.distance(p);
        if !ordered_keys
            .iter()
            .any(|key| double_key_equals(*key, distance_p))
        {
            ordered_keys.push(distance_p);
        }
    }

    ordered_keys.sort_by(|a, b| double_compare(*a, *b));

    // Lets select a line that is the closest to "start" point but "mindistance" away at least.
    let mut key = f64::MAX;
    let mut keys = ordered_keys.iter();
    while key == f64::MAX {
        let Some(candidate) = keys.next() else {
            break;
        };
        if *candidate >= min_distance {
            key = *candidate;
        }
    }

    // If no line is selected (because all of them are closer than minDistance, then get the farest one
    if key == f64::MAX {
        key = *ordered_keys.last()?;
    }

    let filtered: Vec<Line> = intersected_points
        .iter()
        .filter(|(p, _)| p.distance(start) >= key)
        .map(|(_, line)| *line)
        .collect();

    find_closest_line(start, &filtered, 0.0, angle)
}

/// `Grid.PointInPolygon`, `Grid.cs:974-1010`.
fn point_in_polygon(p: UtmPos, poly: &[UtmPos]) -> bool {
    let mut inside = false;

    if poly.len() < 3 {
        return inside;
    }
    let Some(mut old_point) = poly.last().copied() else {
        return inside;
    };

    for new_point in poly.iter().copied() {
        let (p1, p2) = if new_point.y > old_point.y {
            (old_point, new_point)
        } else {
            (new_point, old_point)
        };

        if (new_point.y < p.y) == (p.y <= old_point.y)
            && (p.x - p1.x) * (p2.y - p1.y) < (p2.x - p1.x) * (p.y - p1.y)
        {
            inside = !inside;
        }
        old_point = new_point;
    }
    inside
}

/// `Grid.CreateGrid`, `Grid.cs:354-748`: the survey pattern over `polygon`, in flight order.
///
/// Each lane is `S`, `SM`, any `M` trigger points, `ME`, `E`. An empty polygon, or one no lane
/// crosses, gives an empty grid, as the C# does.
///
/// # Errors
///
/// [`GridError::Projection`] if a point cannot be converted back from UTM, where the C# throws.
/// Positions that [`LatLon`] accepts always convert.
#[allow(clippy::too_many_lines)] // one C# method, kept in one piece so it reads against Grid.cs
pub fn create_grid(polygon: &[LatLon], args: &GridArgs) -> Result<Vec<GridPoint>, GridError> {
    let angle = args.angle;
    let mut spacing = args.spacing;
    let mut distance = args.distance;

    if spacing < 0.1 && spacing != 0.0 {
        spacing = 0.1;
    }

    if distance < 0.1 {
        distance = 0.1;
    }

    let Some(first) = polygon.first() else {
        return Ok(Vec::new());
    };

    // Make a non round number in case of corner cases
    let mut min_lane_separation = args.min_lane_separation;
    if min_lane_separation != 0.0 {
        min_lane_separation += 0.5;
    }
    // Lane Separation in meters
    let min_lane_separation_in_meters = f64::from(min_lane_separation) * distance;

    let mut ans: Vec<(UtmPos, GridTag)> = Vec::new();

    // utm zone distance calcs will be done in
    let utmzone = utm_zone(first.latitude(), first.longitude());

    // utm position list: every vertex in the first vertex's zone and hemisphere
    // C#: PointLatLngAlt.cs:294-302
    let mut utmpositions: Vec<UtmPos> = polygon
        .iter()
        .map(|p| {
            let (x, y) = to_utm(utmzone, first.latitude(), p.latitude(), p.longitude());
            UtmPos::new(x, y, utmzone)
        })
        .collect();

    // close the loop if its not already
    if let (Some(head), Some(tail)) = (utmpositions.first().copied(), utmpositions.last().copied())
        && !head.op_eq(tail)
    {
        utmpositions.push(head); // make a full loop
    }

    // get mins/maxs of coverage area
    let area = get_poly_min_max(&utmpositions);

    // used to determine the size of the outer grid area
    let diagdist = area.diag_distance();

    // somewhere to store out generated lines
    let mut grid: Vec<Line> = Vec::new();
    // number of lines we need
    let mut lines: i32 = 0;

    // get start point middle
    let mut x = area.mid_width();
    let mut y = area.mid_height();

    // get left extent
    let mut xb1 = x;
    let mut yb1 = y;
    // to the left
    newpos_xy(&mut xb1, &mut yb1, angle - 90.0, diagdist / 2.0 + distance);
    // backwards
    newpos_xy(&mut xb1, &mut yb1, angle + 180.0, diagdist / 2.0 + distance);

    // The C# also computes the right extent (Grid.cs:417-427), for a debug overlay that draws
    // nothing (`addtomap`, Grid.cs:36-44).

    // set start point to left hand side
    x = xb1;
    y = yb1;

    // draw the outergrid, this is a grid that cover the entire area of the rectangle plus more.
    while f64::from(lines) < ((diagdist + distance * 2.0) / distance) {
        // copy the start point to generate the end point
        let mut nx = x;
        let mut ny = y;
        newpos_xy(&mut nx, &mut ny, angle, diagdist + distance * 2.0);

        grid.push(Line {
            p1: UtmPos::new(x, y, utmzone),
            p2: UtmPos::new(nx, ny, utmzone),
            basepnt: UtmPos::new(x, y, utmzone),
        });

        newpos_xy(&mut x, &mut y, angle + 90.0, distance);
        lines += 1;
    }

    // find intersections with our polygon

    // store lines that dont have any intersections
    let mut remove: Vec<Line> = Vec::new();

    let gridno = grid.len();

    // cycle through our grid; lines split off below are appended past `gridno` and not revisited
    for a in 0..gridno {
        let Some(line_a) = grid.get(a).copied() else {
            continue;
        };

        let mut closestdistance = f64::MAX;
        let mut farestdistance = f64::MIN;

        let mut closestpoint = UtmPos::ZERO;
        let mut farestpoint = UtmPos::ZERO;

        // somewhere to store our intersections
        let mut matchs: Vec<UtmPos> = Vec::new();

        let mut crosses = 0;
        for edge in utmpositions.windows(2) {
            let [from, to] = edge else {
                continue;
            };
            let newutmpos = find_line_intersection(*from, *to, line_a.p1, line_a.p2);
            if !newutmpos.is_zero() {
                crosses += 1;
                matchs.push(newutmpos);
                if closestdistance > line_a.p1.distance(newutmpos) {
                    closestpoint = newutmpos;
                    closestdistance = line_a.p1.distance(newutmpos);
                }
                if farestdistance < line_a.p1.distance(newutmpos) {
                    farestpoint = newutmpos;
                    farestdistance = line_a.p1.distance(newutmpos);
                }
            }
        }
        if crosses == 0 {
            // outside our polygon
            if !point_in_polygon(line_a.p1, &utmpositions)
                && !point_in_polygon(line_a.p2, &utmpositions)
            {
                remove.push(line_a);
            }
        } else if crosses == 1 {
            // bad - shouldnt happen. The C# keeps the line unclipped, at full length.
        } else if crosses == 2 {
            // simple start and finish
            if let Some(line) = grid.get_mut(a) {
                line.p1 = closestpoint;
                line.p2 = farestpoint;
            }
        } else {
            // multiple intersections
            remove.push(line_a);

            while matchs.len() > 1 {
                closestpoint = find_closest_point(closestpoint, &matchs);
                let p1 = closestpoint;
                let removed_p1 = remove_pos(&mut matchs, closestpoint);

                closestpoint = find_closest_point(closestpoint, &matchs);
                let p2 = closestpoint;
                let removed_p2 = remove_pos(&mut matchs, closestpoint);

                grid.push(Line {
                    p1,
                    p2,
                    basepnt: line_a.basepnt,
                });

                // Only a NaN coordinate makes the nearest point unfindable; the C# then spins
                // here forever, and a hang is not a behaviour worth porting.
                if !(removed_p1 || removed_p2) {
                    break;
                }
            }
        }
    }

    // cleanup and keep only lines that pass though our polygon
    for line in &remove {
        remove_line(&mut grid, line);
    }

    if grid.is_empty() {
        return Ok(Vec::new());
    }

    // pick start positon based on initial point rectangle
    let startposutm = match args.startpos {
        // C#: utmpos.cs:39 - projected in the home's own zone, not the polygon's
        StartPosition::Home => UtmPos::from_lat_lng(args.home.latitude(), args.home.longitude()),
        StartPosition::BottomLeft => UtmPos::new(area.left, area.bottom, utmzone),
        StartPosition::BottomRight => UtmPos::new(area.right, area.bottom, utmzone),
        StartPosition::TopLeft => UtmPos::new(area.left, area.top, utmzone),
        StartPosition::TopRight => UtmPos::new(area.right, area.top, utmzone),
        StartPosition::Point => {
            UtmPos::from_lat_lng(args.start_point.latitude(), args.start_point.longitude())
        }
    };

    // find the closes polygon point based from our startpos selection
    let startposutm = find_closest_point(startposutm, &utmpositions);

    // find closest line point to startpos
    // Lane separation does not apply to starting point
    let Some(mut closest) = find_closest_line(startposutm, &grid, 0.0, angle) else {
        return Ok(Vec::new());
    };

    // get the closes point from the line we picked
    let mut lastpnt = if closest.p1.distance(startposutm) < closest.p2.distance(startposutm) {
        closest.p1
    } else {
        closest.p2
    };

    // S =  start
    // E = end
    // ME = middle end
    // SM = start middle

    while !grid.is_empty() {
        // for each line, check which end of the line is the next closest
        let next_from = if closest.p1.distance(lastpnt) < closest.p2.distance(lastpnt) {
            let newstart = newpos(closest.p1, angle, f64::from(-args.leadin1));
            ans.push((newstart, GridTag::Start));

            if args.leadin1 < 0.0 {
                ans.push((newstart, GridTag::StartMiddle));
            } else {
                ans.push((closest.p1, GridTag::StartMiddle));
            }

            if spacing > 0.0 {
                let mut d = spacing - (closest.basepnt.distance(closest.p1) % spacing);
                while d < closest.p1.distance(closest.p2) {
                    let mut ax = closest.p1.x;
                    let mut ay = closest.p1.y;

                    newpos_xy(&mut ax, &mut ay, angle, d);
                    ans.push((UtmPos::new(ax, ay, utmzone), GridTag::Middle));
                    d += spacing;
                }
            }

            let newend = newpos(closest.p2, angle, args.overshoot1);

            if args.overshoot1 < 0.0 {
                ans.push((newend, GridTag::MiddleEnd));
            } else {
                ans.push((closest.p2, GridTag::MiddleEnd));
            }

            ans.push((newend, GridTag::End));

            lastpnt = closest.p2;

            if args.use_extended_endpoint {
                newend
            } else {
                closest.p2
            }
        } else {
            let newstart = newpos(closest.p2, angle, f64::from(args.leadin2));
            ans.push((newstart, GridTag::Start));

            if args.leadin2 < 0.0 {
                ans.push((newstart, GridTag::StartMiddle));
            } else {
                ans.push((closest.p2, GridTag::StartMiddle));
            }

            if spacing > 0.0 {
                let mut d = closest.basepnt.distance(closest.p2) % spacing;
                while d < closest.p1.distance(closest.p2) {
                    let mut ax = closest.p2.x;
                    let mut ay = closest.p2.y;

                    newpos_xy(&mut ax, &mut ay, angle, -d);
                    ans.push((UtmPos::new(ax, ay, utmzone), GridTag::Middle));
                    d += spacing;
                }
            }

            let newend = newpos(closest.p1, angle, -args.overshoot2);

            if args.overshoot2 < 0.0 {
                ans.push((newend, GridTag::MiddleEnd));
            } else {
                ans.push((closest.p1, GridTag::MiddleEnd));
            }

            ans.push((newend, GridTag::End));

            lastpnt = closest.p1;

            if args.use_extended_endpoint {
                newend
            } else {
                closest.p1
            }
        };

        remove_line(&mut grid, &closest);
        if grid.is_empty() {
            break;
        }

        let Some(next) = find_closest_line(next_from, &grid, min_lane_separation_in_meters, angle)
        else {
            break;
        };
        closest = next;
    }

    // set the altitude on all points; the C# converts each point to lat/lng as it is added
    // (utmpos.cs:53, the implicit operator), which is the same arithmetic done later
    ans.into_iter()
        .map(|(pos, tag)| {
            let (lat, lng) = pos.to_lla().ok_or(GridError::Projection)?;
            Ok(GridPoint {
                lat,
                lng,
                alt: args.altitude,
                tag,
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_start_positions_parse_by_their_csharp_names() {
        for start in StartPosition::ALL {
            assert_eq!(StartPosition::from_name(start.name()), Some(start));
        }
        assert_eq!(StartPosition::from_name("home"), None);
    }

    #[test]
    fn tags_round_trip_through_their_csharp_spelling() {
        for tag in ["S", "SM", "M", "ME", "E"] {
            assert_eq!(GridTag::from_str_cs(tag).map(GridTag::as_str), Some(tag));
        }
        assert_eq!(GridTag::from_str_cs("X"), None);
    }

    #[test]
    fn an_empty_polygon_is_an_empty_grid() {
        assert_eq!(create_grid(&[], &GridArgs::default()), Ok(Vec::new()));
    }

    #[test]
    fn every_lane_is_start_middle_end() {
        let area = [
            LatLon::new(-35.360, 149.160).unwrap(),
            LatLon::new(-35.360, 149.171).unwrap(),
            LatLon::new(-35.369, 149.171).unwrap(),
            LatLon::new(-35.369, 149.160).unwrap(),
        ];
        let points = create_grid(&area, &GridArgs::default()).unwrap();
        assert!(!points.is_empty());
        for lane in points.chunks(4) {
            let tags: Vec<&str> = lane.iter().map(|p| p.tag.as_str()).collect();
            assert_eq!(tags, ["S", "SM", "ME", "E"]);
        }
    }

    #[test]
    fn add_angle_normalises_into_0_to_360() {
        assert_eq!(add_angle(300.0, 90.0), 30.0);
        assert_eq!(add_angle(-100.0, 90.0), 350.0);
    }

    #[test]
    fn point_in_polygon_is_the_even_odd_rule() {
        let square = [
            UtmPos::new(0.0, 0.0, 1),
            UtmPos::new(10.0, 0.0, 1),
            UtmPos::new(10.0, 10.0, 1),
            UtmPos::new(0.0, 10.0, 1),
            UtmPos::new(0.0, 0.0, 1),
        ];
        assert!(point_in_polygon(UtmPos::new(5.0, 5.0, 1), &square));
        assert!(!point_in_polygon(UtmPos::new(15.0, 5.0, 1), &square));
    }
}
