//! Survey grids against Mission Planner's own: PLAN.md §13.3 item 3, DELIVERABLES.md D11.
//!
//! Every file in `testdata/grid/golden` is what `Grid.CreateGrid` returned under mono for one case
//! of `testdata/grid/cases.txt`, with the arguments it was called with recorded above the points
//! (`tools/csharp-reference/regen-grid.sh`, and `testdata/grid/README.md` for how). Each case is
//! re-run through [`create_grid`] with those arguments and held to PLAN.md §7.2 tolerance class C:
//! the same number of points, in the same order with the same tags, each within 1e-7 degrees, at
//! the same altitude.
//!
//! A case that can only match up to a tie-break - two candidate lanes at the same distance, picked
//! differently - goes in [`TIE_BREAKS`] with its reason, and is held to the class C invariants
//! instead. Every case is in exactly one of the two, and the strict set must be the large
//! majority: a port that needs many exceptions is a wrong port.
//!
//! The invariants are the ones `Grid.cs` actually keeps, and
//! [`the_invariants_hold_for_mission_planners_own_grids`] checks them against every golden:
//! - adjacent lanes are one lane spacing apart, to 1%;
//! - every part of a lane's line that lies inside the area is flown, and the area reaches less
//!   than one spacing beyond the outermost lanes - between them, every point is within half a
//!   spacing of a lane;
//! - nothing is further outside the area than the longest lead-in or overshoot.
//!
//! PLAN.md §7.2 words the coverage invariant as "every polygon interior point within spacing/2 of
//! a lane". Mission Planner's own grids break that: the lanes are laid out from the bounding
//! rectangle, not from the edges, so the strip between an edge and the outermost lane is up to a
//! whole spacing wide (0.9 of it in `cbr_field_a137`), and the tip of a sliver narrower than one
//! spacing can fall between two lanes altogether (`cbr_sliver_a90`, three spacings from the nearest
//! lane). An invariant the C# fails cannot stand in for the C#, so the half-spacing bound is held
//! only between the outermost lanes, where it is true.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic
)]

use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use mp_mission::grid::{GridArgs, GridPoint, GridTag, StartPosition, create_grid};
use mp_units::LatLon;

/// Cases held to the invariants rather than point for point, each with the reason.
///
/// Empty: every case matches the C#. Add a case here only with the tie it breaks differently,
/// named, and never to make a failure go away.
const TIE_BREAKS: &[(&str, &str)] = &[];

/// §7.2 class C: the per-point tolerance.
const DEGREES: f64 = 1e-7;

fn grid_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/grid")
}

/// One golden file: the case's inputs as `CreateGrid` received them, and what it returned.
struct Golden {
    name: String,
    polygon: Vec<LatLon>,
    args: GridArgs,
    points: Vec<GridPoint>,
}

fn parse_f64(value: &str, file: &str) -> f64 {
    value
        .parse()
        .unwrap_or_else(|_| panic!("{file}: {value} is not a number"))
}

fn parse_lat_lng(values: &[&str], file: &str) -> LatLon {
    let [lat, lng] = values else {
        panic!("{file}: expected lat,lng, got {values:?}");
    };
    LatLon::new(parse_f64(lat, file), parse_f64(lng, file)).unwrap()
}

fn parse_bool(value: &str, file: &str) -> bool {
    match value {
        "True" => true,
        "False" => false,
        _ => panic!("{file}: {value} is not a C# bool"),
    }
}

fn read_golden(path: &Path) -> Golden {
    let file = path.display().to_string();
    let text = std::fs::read_to_string(path).unwrap();

    let mut name = None;
    let mut polygon = Vec::new();
    let mut args = GridArgs::default();
    let mut declared = None;
    let mut points = Vec::new();
    let mut seen = BTreeSet::new();

    for line in text.lines().filter(|l| !l.starts_with('#')) {
        let mut fields = line.split(',');
        let key = fields.next().unwrap();
        let values: Vec<&str> = fields.collect();
        if key != "vertex" && key != "wp" {
            assert!(seen.insert(key.to_owned()), "{file}: {key} given twice");
        }
        let one = || -> &str {
            assert_eq!(values.len(), 1, "{file}: {key} takes one value");
            values[0]
        };
        match key {
            "case" => name = Some(one().to_owned()),
            "vertex" => polygon.push(parse_lat_lng(&values, &file)),
            "altitude" => args.altitude = parse_f64(one(), &file),
            "distance" => args.distance = parse_f64(one(), &file),
            "spacing" => args.spacing = parse_f64(one(), &file),
            "angle" => args.angle = parse_f64(one(), &file),
            "overshoot1" => args.overshoot1 = parse_f64(one(), &file),
            "overshoot2" => args.overshoot2 = parse_f64(one(), &file),
            "startpos" => {
                args.startpos = StartPosition::from_name(one())
                    .unwrap_or_else(|| panic!("{file}: no start position {}", one()));
            }
            // CreateGrid never reads it (Grid.cs:354); GridArgs has no field for it.
            "shutter" => {
                parse_bool(one(), &file);
            }
            "minLaneSeparation" => args.min_lane_separation = one().parse().unwrap(),
            "leadin1" => args.leadin1 = one().parse().unwrap(),
            "leadin2" => args.leadin2 = one().parse().unwrap(),
            "HomeLocation" => args.home = parse_lat_lng(&values, &file),
            "useextendedendpoint" => args.use_extended_endpoint = parse_bool(one(), &file),
            "StartPointLatLngAlt" => args.start_point = parse_lat_lng(&values, &file),
            "points" => declared = Some(one().parse::<usize>().unwrap()),
            "wp" => {
                let [lat, lng, alt, tag] = values[..] else {
                    panic!("{file}: wp takes lat,lng,alt,tag");
                };
                points.push(GridPoint {
                    lat: parse_f64(lat, &file),
                    lng: parse_f64(lng, &file),
                    alt: parse_f64(alt, &file),
                    tag: GridTag::from_str_cs(tag)
                        .unwrap_or_else(|| panic!("{file}: no grid tag {tag}")),
                });
            }
            _ => panic!("{file}: unknown key {key}"),
        }
    }

    // Every argument must have been recorded: a missing line would silently take this crate's
    // default instead of the one the C# ran with.
    for key in [
        "case",
        "altitude",
        "distance",
        "spacing",
        "angle",
        "overshoot1",
        "overshoot2",
        "startpos",
        "shutter",
        "minLaneSeparation",
        "leadin1",
        "leadin2",
        "HomeLocation",
        "useextendedendpoint",
        "StartPointLatLngAlt",
        "points",
    ] {
        assert!(seen.contains(key), "{file}: no {key}");
    }
    assert_eq!(declared, Some(points.len()), "{file}: truncated");
    let name = name.unwrap();
    assert_eq!(
        path.file_stem().and_then(|s| s.to_str()),
        Some(name.as_str()),
        "{file}: named for another case"
    );
    Golden {
        name,
        polygon,
        args,
        points,
    }
}

fn goldens() -> Vec<Golden> {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(grid_dir().join("golden"))
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| path.extension().is_some_and(|ext| ext == "csv"))
        .collect();
    paths.sort();
    paths.iter().map(|path| read_golden(path)).collect()
}

/// The case names `cases.txt` declares.
fn declared_cases() -> BTreeSet<String> {
    std::fs::read_to_string(grid_dir().join("cases.txt"))
        .unwrap()
        .lines()
        .map(|line| line.split('#').next().unwrap_or(""))
        .filter_map(|line| {
            let mut words = line.split_whitespace();
            (words.next() == Some("case")).then(|| words.next().unwrap().to_owned())
        })
        .collect()
}

/// Longitude difference with the antimeridian taken into account.
fn lng_difference(a: f64, b: f64) -> f64 {
    let d = (a - b).abs() % 360.0;
    d.min(360.0 - d)
}

/// Why `ours` is not `theirs` to class C, or `None` if it is.
fn strict_mismatch(ours: &[GridPoint], theirs: &[GridPoint]) -> Option<String> {
    if ours.len() != theirs.len() {
        return Some(format!(
            "{} points where Mission Planner has {}",
            ours.len(),
            theirs.len()
        ));
    }
    for (index, (a, b)) in ours.iter().zip(theirs).enumerate() {
        let (dlat, dlng) = ((a.lat - b.lat).abs(), lng_difference(a.lng, b.lng));
        if a.tag != b.tag || dlat > DEGREES || dlng > DEGREES || a.alt != b.alt {
            return Some(format!(
                "point {index}: {:?} {},{} alt {} where Mission Planner has {:?} {},{} alt {}",
                a.tag, a.lat, a.lng, a.alt, b.tag, b.lat, b.lng, b.alt
            ));
        }
    }
    None
}

/// A local east-north plane about `origin`, metres: WGS84 ECEF, rotated to the origin's east, north
/// and up, with up dropped. Its scale is exact at the origin and off by (d/R)^2 / 2 at a distance d
/// - 2e-7 across the largest case - and it has no seam at the antimeridian.
struct Plane {
    origin: [f64; 3],
    sin_lat: f64,
    cos_lat: f64,
    sin_lng: f64,
    cos_lng: f64,
}

fn ecef(lat: f64, lng: f64) -> [f64; 3] {
    let a = 6_378_137.0_f64;
    let e2 = 0.006_694_379_990_14_f64;
    let (phi, lambda) = (lat.to_radians(), lng.to_radians());
    let n = a / (1.0 - e2 * phi.sin().powi(2)).sqrt();
    [
        n * phi.cos() * lambda.cos(),
        n * phi.cos() * lambda.sin(),
        n * (1.0 - e2) * phi.sin(),
    ]
}

impl Plane {
    fn new(origin: LatLon) -> Self {
        let (phi, lambda) = (
            origin.latitude().to_radians(),
            origin.longitude().to_radians(),
        );
        Self {
            origin: ecef(origin.latitude(), origin.longitude()),
            sin_lat: phi.sin(),
            cos_lat: phi.cos(),
            sin_lng: lambda.sin(),
            cos_lng: lambda.cos(),
        }
    }

    fn at(&self, lat: f64, lng: f64) -> (f64, f64) {
        let p = ecef(lat, lng);
        let d = [
            p[0] - self.origin[0],
            p[1] - self.origin[1],
            p[2] - self.origin[2],
        ];
        let east = -self.sin_lng * d[0] + self.cos_lng * d[1];
        let north = -self.sin_lat * self.cos_lng * d[0] - self.sin_lat * self.sin_lng * d[1]
            + self.cos_lat * d[2];
        (east, north)
    }
}

type Xy = (f64, f64);

fn distance_to_segment(p: Xy, a: Xy, b: Xy) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let length_squared = dx * dx + dy * dy;
    let t = if length_squared == 0.0 {
        0.0
    } else {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / length_squared).clamp(0.0, 1.0)
    };
    (p.0 - (a.0 + t * dx)).hypot(p.1 - (a.1 + t * dy))
}

fn inside(p: Xy, polygon: &[Xy]) -> bool {
    let mut result = false;
    let mut previous = polygon[polygon.len() - 1];
    for &vertex in polygon {
        if (vertex.1 > p.1) != (previous.1 > p.1)
            && p.0 < (previous.0 - vertex.0) * (p.1 - vertex.1) / (previous.1 - vertex.1) + vertex.0
        {
            result = !result;
        }
        previous = vertex;
    }
    result
}

/// How far the plane may disagree with the C#'s UTM along a survey-sized edge, and more: the
/// geometry is the C#'s own, so anything this loose is a real difference.
const TOLERANCE: f64 = 0.25;

/// Distance from `p` to the polygon's boundary, from either side.
fn distance_outside_or_in(p: Xy, polygon: &[Xy]) -> f64 {
    (0..polygon.len())
        .map(|i| distance_to_segment(p, polygon[i], polygon[(i + 1) % polygon.len()]))
        .fold(f64::INFINITY, f64::min)
}

fn distance_outside(p: Xy, polygon: &[Xy]) -> f64 {
    if inside(p, polygon) {
        return 0.0;
    }
    distance_outside_or_in(p, polygon)
}

/// PLAN.md §7.2's class C invariants for `points` as a grid over `golden`'s polygon with
/// `golden`'s arguments. The failures, one line each.
fn invariant_failures(golden: &Golden, points: &[GridPoint]) -> Vec<String> {
    let mut failures = Vec::new();
    let Some(&origin) = golden.polygon.first() else {
        return failures;
    };
    let plane = Plane::new(origin);
    let polygon: Vec<Xy> = golden
        .polygon
        .iter()
        .map(|p| plane.at(p.latitude(), p.longitude()))
        .collect();
    let args = &golden.args;
    // Grid.cs:361
    let distance = args.distance.max(0.1);

    // Each lane as flown, from its start (S) to its end (E); SM, the M points and ME lie between.
    let mut lanes: Vec<(Xy, Xy)> = Vec::new();
    let mut start = None;
    for point in points {
        match point.tag {
            GridTag::Start => start = Some(plane.at(point.lat, point.lng)),
            GridTag::End => match start.take() {
                Some(from) => lanes.push((from, plane.at(point.lat, point.lng))),
                None => failures.push("a lane ends that never started".to_owned()),
            },
            _ => {}
        }
    }
    if lanes.is_empty() {
        if !points.is_empty() {
            failures.push("points but no lanes".to_owned());
        }
        return failures;
    }

    // Nothing further outside the area than the longest lead-in or overshoot: the S and E points
    // are the only ones allowed out, and only by those.
    let allowed = [
        args.overshoot1,
        args.overshoot2,
        f64::from(args.leadin1),
        f64::from(args.leadin2),
        0.0,
    ]
    .into_iter()
    .fold(0.0, f64::max)
        + TOLERANCE;
    for (index, point) in points.iter().enumerate() {
        let outside = distance_outside(plane.at(point.lat, point.lng), &polygon);
        if outside > allowed {
            failures.push(format!(
                "point {index} ({:?}) is {outside:.2} m outside the area, allowed {allowed:.2}",
                point.tag
            ));
        }
    }

    // The lanes are parallel in UTM, which is not quite parallel to this plane's north, so their
    // normal comes from the longest lane itself. A grid of lanes shorter than their spacing (an
    // area that is all edge) has no direction worth measuring.
    let length = |l: &(Xy, Xy)| (l.1.0 - l.0.0).hypot(l.1.1 - l.0.1);
    let longest = lanes
        .iter()
        .copied()
        .max_by(|a, b| length(a).total_cmp(&length(b)))
        .unwrap();
    if length(&longest) <= distance {
        return failures;
    }
    let (dx, dy) = (longest.1.0 - longest.0.0, longest.1.1 - longest.0.1);
    let normal = (-dy / length(&longest), dx / length(&longest));
    let offset = |p: Xy| p.0 * normal.0 + p.1 * normal.1;
    let lane_offset = |l: &(Xy, Xy)| offset(((l.0.0 + l.1.0) / 2.0, (l.0.1 + l.1.1) / 2.0));

    // Adjacent lanes one lane spacing apart, to 1%. Lanes split by a concave boundary share a
    // line; count each line once.
    let mut lines: Vec<f64> = lanes.iter().map(lane_offset).collect();
    lines.sort_by(f64::total_cmp);
    lines.dedup_by(|a, b| (*a - *b).abs() < distance * 0.01);
    for pair in lines.windows(2) {
        let ratio = (pair[1] - pair[0]) / distance;
        if (ratio - 1.0).abs() > 0.01 {
            failures.push(format!(
                "adjacent lanes {:.2} m apart, spacing {distance}",
                pair[1] - pair[0]
            ));
        }
    }

    // No line missing at the edges: the area reaches less than one spacing past the outermost
    // lanes, so every point between them is within half a spacing of a lane's line.
    let (first, last) = (lines[0], lines[lines.len() - 1]);
    let (low, high) = polygon.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| {
        (lo.min(offset(*p)), hi.max(offset(*p)))
    });
    for (gap, side) in [(first - low, "one"), (high - last, "the other")] {
        if gap > distance * 1.01 + TOLERANCE {
            failures.push(format!(
                "the area reaches {gap:.2} m past the outermost lane on {side} side, spacing {distance}"
            ));
        }
    }

    // Every part of a lane's line inside the area is flown: for points sampled across the area,
    // the foot on each neighbouring line, where it is inside the area, is on a lane. A negative
    // lead-in or overshoot deliberately stops short of the edge, by up to its size.
    let short = [
        -args.overshoot1,
        -args.overshoot2,
        -f64::from(args.leadin1),
        -f64::from(args.leadin2),
        0.0,
    ]
    .into_iter()
    .fold(0.0, f64::max)
        + TOLERANCE;
    let (mut min, mut max) = ((f64::MAX, f64::MAX), (f64::MIN, f64::MIN));
    for p in &polygon {
        min = (min.0.min(p.0), min.1.min(p.1));
        max = (max.0.max(p.0), max.1.max(p.1));
    }
    let steps = 60;
    let mut unflown = 0;
    for i in 1..steps {
        for j in 1..steps {
            let p = (
                min.0 + (max.0 - min.0) * f64::from(i) / f64::from(steps),
                min.1 + (max.1 - min.1) * f64::from(j) / f64::from(steps),
            );
            if !inside(p, &polygon) {
                continue;
            }
            let o = offset(p);
            let below = lines.iter().copied().rfind(|l| *l <= o);
            let above = lines.iter().copied().find(|l| *l >= o);
            for line in [below, above].into_iter().flatten() {
                let foot = (p.0 + (line - o) * normal.0, p.1 + (line - o) * normal.1);
                if !inside(foot, &polygon) || distance_outside_or_in(foot, &polygon) < short {
                    continue;
                }
                let nearest = lanes
                    .iter()
                    .filter(|l| (lane_offset(l) - line).abs() < distance * 0.01)
                    .map(|(a, b)| distance_to_segment(foot, *a, *b))
                    .fold(f64::INFINITY, f64::min);
                if nearest > TOLERANCE {
                    unflown += 1;
                }
            }
        }
    }
    if unflown > 0 {
        failures.push(format!(
            "{unflown} sampled points on lane lines inside the area are not flown"
        ));
    }
    failures
}

#[test]
fn every_case_has_a_golden_and_every_golden_a_case() {
    let declared = declared_cases();
    let generated: BTreeSet<String> = goldens().into_iter().map(|g| g.name).collect();
    assert!(
        declared.len() >= 30,
        "D11 asks for at least 30 cases, cases.txt has {}",
        declared.len()
    );
    assert_eq!(
        declared, generated,
        "cases.txt and golden/ disagree: run tools/csharp-reference/regen-grid.sh"
    );
}

#[test]
fn every_tie_break_names_a_case_and_a_reason() {
    let declared = declared_cases();
    let mut names = BTreeSet::new();
    for (name, reason) in TIE_BREAKS {
        assert!(declared.contains(*name), "{name} is not a case");
        assert!(names.insert(*name), "{name} listed twice");
        assert!(!reason.trim().is_empty(), "{name} has no reason");
    }
    // The strict list is everything else, and must stay the large majority.
    assert!(
        TIE_BREAKS.len() * 10 <= declared.len(),
        "{} of {} cases need a tie-break exception; the port is wrong",
        TIE_BREAKS.len(),
        declared.len()
    );
}

#[test]
fn the_grid_matches_mission_planner_point_for_point() {
    let exceptions: BTreeSet<&str> = TIE_BREAKS.iter().map(|(name, _)| *name).collect();
    let mut report = String::new();
    let (mut strict, mut identical, mut worst) = (0, 0, 0.0_f64);
    for golden in goldens() {
        if exceptions.contains(golden.name.as_str()) {
            continue;
        }
        strict += 1;
        let ours = create_grid(&golden.polygon, &golden.args).unwrap();
        if let Some(why) = strict_mismatch(&ours, &golden.points) {
            writeln!(report, "{}: {why}", golden.name).unwrap();
            continue;
        }
        for (a, b) in ours.iter().zip(&golden.points) {
            worst = worst
                .max((a.lat - b.lat).abs())
                .max(lng_difference(a.lng, b.lng));
        }
        if ours == golden.points {
            identical += 1;
        }
    }
    println!(
        "{strict} cases held to class C, {identical} of them bit-identical; largest difference \
         {worst:e} degrees"
    );
    assert!(report.is_empty(), "differs from Grid.CreateGrid:\n{report}");
}

#[test]
fn tie_break_cases_hold_the_invariants() {
    let exceptions: BTreeSet<&str> = TIE_BREAKS.iter().map(|(name, _)| *name).collect();
    let mut report = String::new();
    for golden in goldens() {
        if !exceptions.contains(golden.name.as_str()) {
            continue;
        }
        let ours = create_grid(&golden.polygon, &golden.args).unwrap();
        if ours.len() != golden.points.len() {
            writeln!(
                report,
                "{}: {} points where Mission Planner has {}",
                golden.name,
                ours.len(),
                golden.points.len()
            )
            .unwrap();
        }
        for failure in invariant_failures(&golden, &ours) {
            writeln!(report, "{}: {failure}", golden.name).unwrap();
        }
    }
    assert!(report.is_empty(), "invariants broken:\n{report}");
}

/// The invariants are only a fallback if they are true of Mission Planner's own grids, so they are
/// checked against every golden - which also proves they are not vacuous.
#[test]
fn the_invariants_hold_for_mission_planners_own_grids() {
    let mut report = String::new();
    let mut checked = 0;
    for golden in goldens() {
        checked += 1;
        for failure in invariant_failures(&golden, &golden.points) {
            writeln!(report, "{}: {failure}", golden.name).unwrap();
        }
    }
    assert!(checked > 0);
    assert!(
        report.is_empty(),
        "invariants broken by Grid.CreateGrid:\n{report}"
    );
}

/// And each of them catches the grid it exists to catch, starting from a real one.
#[test]
fn the_invariants_catch_a_broken_grid() {
    let golden = goldens()
        .into_iter()
        .find(|g| g.name == "cbr_square_a0")
        .unwrap();
    assert!(invariant_failures(&golden, &golden.points).is_empty());
    // No trigger spacing, so every lane is S, SM, ME, E; flown edge to edge from the vertex
    // nearest home, so the first lane flown is an outermost one.
    let lanes: Vec<Vec<GridPoint>> = golden.points.chunks(4).map(<[_]>::to_vec).collect();
    assert!(lanes.len() > 4);
    assert!(lanes.iter().all(|lane| lane[0].tag == GridTag::Start));
    let middle = lanes.len() / 2;
    let broken = |mutate: &dyn Fn(usize, &mut Vec<GridPoint>)| -> Vec<String> {
        let mut points = Vec::new();
        for (index, lane) in lanes.iter().enumerate() {
            let mut lane = lane.clone();
            mutate(index, &mut lane);
            points.extend(lane);
        }
        invariant_failures(&golden, &points)
    };
    let caught = |failures: &[String], what: &str| {
        assert!(
            failures.iter().any(|f| f.contains(what)),
            "expected \"{what}\", got {failures:?}"
        );
    };

    // A lane missing from the middle leaves two lanes two spacings apart.
    let failures = broken(&|index, lane| {
        if index == middle {
            lane.clear();
        }
    });
    caught(&failures, "adjacent lanes");

    // A lane missing from the edge leaves a strip wider than a spacing.
    let failures = broken(&|index, lane| {
        if index == 0 {
            lane.clear();
        }
    });
    caught(&failures, "past the outermost lane");

    // A lane stopping halfway leaves the rest of its line unflown.
    let failures = broken(&|index, lane| {
        if index == middle {
            let (start, end) = (lane[0], lane[3]);
            for point in &mut lane[2..] {
                point.lat = (start.lat + end.lat) / 2.0;
                point.lng = (start.lng + end.lng) / 2.0;
            }
        }
    });
    caught(&failures, "not flown");

    // A lane running on past the edge by its own length exits by far more than the overshoot.
    let failures = broken(&|index, lane| {
        if index == middle {
            let (start, end) = (lane[0], lane[3]);
            lane[3].lat = end.lat + (end.lat - start.lat);
            lane[3].lng = end.lng + (end.lng - start.lng);
        }
    });
    caught(&failures, "outside the area");
}
