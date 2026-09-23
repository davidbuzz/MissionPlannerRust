//! Survey grid properties, through the planner's [`grid`] wrapper.
//!
//! Point-for-point agreement with Mission Planner's `Grid.CreateGrid` is `grid_vectors.rs`'s job,
//! over the goldens in `testdata/grid`. These tests say what that agreement means for someone
//! planning a survey - lines at the requested spacing and bearing, flown back and forth, clipped to
//! the area, extended by the overshoot - so a reader does not have to infer it from coordinates.
//! Where Mission Planner's behaviour is not what a reader might expect, the test says so and names
//! the golden case that pins it.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use mp_mission::survey::{
    GridError, GridOptions, contains, contains_within, distance_to_boundary, grid,
};
use mp_units::LatLon;

fn at(lat: f64, lon: f64) -> LatLon {
    LatLon::new(lat, lon).expect("valid")
}

/// A square roughly 1 km on a side near ArduPilot's default test location.
fn square() -> Vec<LatLon> {
    vec![
        at(-35.360, 149.160),
        at(-35.360, 149.171),
        at(-35.369, 149.171),
        at(-35.369, 149.160),
    ]
}

/// A U open to the north, so north-south lines through the gap cross the boundary four times.
fn u_shape() -> Vec<LatLon> {
    vec![
        at(-35.3900, 149.1800),
        at(-35.3900, 149.1830),
        at(-35.3960, 149.1830),
        at(-35.3960, 149.1880),
        at(-35.3900, 149.1880),
        at(-35.3900, 149.1910),
        at(-35.3990, 149.1910),
        at(-35.3990, 149.1800),
    ]
}

/// Metres east and north of `origin`, near enough for spacing to a percent.
fn plane(origin: LatLon, position: LatLon) -> (f64, f64) {
    let per_degree = 111_320.0;
    (
        (position.longitude() - origin.longitude())
            * per_degree
            * origin.latitude().to_radians().cos(),
        (position.latitude() - origin.latitude()) * per_degree,
    )
}

/// The lines' normal, from the direction of the first lane, and each lane's offset along it,
/// sorted: metres in [`plane`] about the area's first vertex.
fn lane_offsets(area: &[LatLon], waypoints: &[LatLon]) -> ((f64, f64), Vec<f64>) {
    let origin = area[0];
    let lanes: Vec<((f64, f64), (f64, f64))> = waypoints
        .chunks(2)
        .map(|lane| (plane(origin, lane[0]), plane(origin, lane[1])))
        .collect();
    let (a, b) = lanes[0];
    let length = (b.0 - a.0).hypot(b.1 - a.1);
    let normal = (-(b.1 - a.1) / length, (b.0 - a.0) / length);
    let mut offsets: Vec<f64> = lanes
        .iter()
        .map(|(a, b)| ((a.0 + b.0) / 2.0) * normal.0 + ((a.1 + b.1) / 2.0) * normal.1)
        .collect();
    offsets.sort_by(f64::total_cmp);
    (normal, offsets)
}

#[test]
fn a_grid_covers_the_area_it_was_given() {
    let area = square();
    let options = GridOptions {
        spacing: 100.0,
        angle: 0.0,
        overshoot: 0.0,
        altitude: 50.0,
    };
    let waypoints = grid(&area, &options).expect("generates");

    assert!(
        waypoints.len() >= 8,
        "expected several lines, got {} points",
        waypoints.len()
    );
    assert_eq!(waypoints.len() % 2, 0, "waypoints come in line endpoints");

    // Every endpoint must be inside the area or on its edge, since no overshoot was requested.
    // A clipped line legitimately ends exactly on the boundary, so strict containment is the
    // wrong test.
    for point in &waypoints {
        assert!(
            contains_within(&area, *point, 0.5),
            "waypoint {:.6},{:.6} is {:.1} m outside the survey area",
            point.latitude(),
            point.longitude(),
            distance_to_boundary(&area, *point)
        );
    }
}

#[test]
fn line_spacing_matches_the_request() {
    let area = square();
    for (spacing, angle) in [(50.0, 0.0), (100.0, 90.0), (200.0, 45.0)] {
        let options = GridOptions {
            spacing,
            angle,
            ..GridOptions::default()
        };
        let (_, offsets) = lane_offsets(&area, &grid(&area, &options).expect("generates"));

        assert!(
            offsets.len() >= 3,
            "spacing {spacing} produced only {} lines",
            offsets.len()
        );
        for pair in offsets.windows(2) {
            let metres = pair[1] - pair[0];
            assert!(
                (metres - spacing).abs() < spacing * 0.02,
                "requested {spacing} m spacing at {angle} degrees, measured {metres:.1} m"
            );
        }
    }
}

#[test]
fn between_the_outermost_lines_no_point_is_further_than_half_a_spacing_from_one() {
    // The property that makes a survey a survey. Mission Planner lays its lines out from the
    // area's bounding rectangle (Grid.cs:405-451), not from the edges, so the strip between an
    // edge and the outermost line can be up to a whole spacing wide; between the outermost lines,
    // no point is more than half a spacing from one.
    let area = square();
    let spacing = 100.0;
    let (normal, offsets) = lane_offsets(
        &area,
        &grid(
            &area,
            &GridOptions {
                spacing,
                ..GridOptions::default()
            },
        )
        .expect("generates"),
    );
    let (first, last) = (offsets[0], offsets[offsets.len() - 1]);

    // Sample the interior on a fine lattice, measuring across the lines.
    let mut worst: f64 = 0.0;
    for i in 1..40 {
        for j in 1..40 {
            let lat = -35.369 + (f64::from(i) / 40.0) * 0.009;
            let lon = 149.160 + (f64::from(j) / 40.0) * 0.011;
            let sample = at(lat, lon);
            if !contains(&area, sample) {
                continue;
            }
            let (east, north) = plane(area[0], sample);
            let across = east * normal.0 + north * normal.1;
            if across < first || across > last {
                assert!(
                    (across - first).abs().min((across - last).abs()) < spacing,
                    "a point sat a whole spacing beyond the outermost line"
                );
                continue;
            }
            let nearest = offsets
                .iter()
                .map(|line| (line - across).abs())
                .fold(f64::INFINITY, f64::min);
            worst = worst.max(nearest);
        }
    }
    assert!(
        worst <= spacing / 2.0 + 1.0,
        "a point sat {worst:.1} m from the nearest line, more than half of {spacing} m"
    );
}

#[test]
fn the_pattern_alternates_direction_like_a_lawnmower() {
    // Flying every line in the same direction means a transit back across the whole area between
    // each pass, which roughly doubles the flight time. At angle 0 the lines run north-south.
    let waypoints = grid(
        &square(),
        &GridOptions {
            spacing: 100.0,
            ..GridOptions::default()
        },
    )
    .expect("generates");

    let directions: Vec<bool> = waypoints
        .chunks(2)
        .map(|line| line[1].latitude() > line[0].latitude())
        .collect();
    assert!(directions.len() >= 4);
    for pair in directions.windows(2) {
        assert_ne!(
            pair[0], pair[1],
            "consecutive lines must run in opposite directions"
        );
    }
}

#[test]
fn a_concave_area_produces_split_lines_rather_than_flying_through_the_gap() {
    // North-south lines through a U cross the boundary four times. Joining those crossings into
    // one line would fly the aircraft straight across the part that was deliberately excluded;
    // Grid.cs:519-540 pairs them up into separate lines instead.
    let area = u_shape();
    let waypoints = grid(
        &area,
        &GridOptions {
            spacing: 60.0,
            ..GridOptions::default()
        },
    )
    .expect("generates");

    for point in &waypoints {
        assert!(
            contains_within(&area, *point, 0.5),
            "waypoint {:.6},{:.6} is {:.1} m into the excluded gap",
            point.latitude(),
            point.longitude(),
            distance_to_boundary(&area, *point)
        );
    }

    // Midpoints of each line must also be inside; that is what catches a line spanning the gap.
    let mut split = 0;
    for line in waypoints.chunks(2) {
        let mid = at(
            (line[0].latitude() + line[1].latitude()) / 2.0,
            (line[0].longitude() + line[1].longitude()) / 2.0,
        );
        assert!(contains_within(&area, mid, 0.5), "a line crosses the gap");
        // A line over one arm of the U stops at the gap's floor, well short of the full height.
        if (line[0].latitude() - line[1].latitude()).abs() < 0.008 {
            split += 1;
        }
    }
    assert!(split >= 2, "no line was split by the gap");
}

#[test]
fn overshoot_extends_lines_beyond_the_boundary() {
    // A camera survey needs the aircraft straight and level through the last photo of a line, so
    // lines must end outside the area. Mission Planner adds the overshoot at the end a line is
    // flown towards, only (Grid.cs:646, :712); the start has its own lead-in. At angle 90 the
    // lines run east-west.
    let area = square();
    let plain = grid(
        &area,
        &GridOptions {
            spacing: 100.0,
            angle: 90.0,
            ..GridOptions::default()
        },
    )
    .unwrap();
    let extended = grid(
        &area,
        &GridOptions {
            spacing: 100.0,
            angle: 90.0,
            overshoot: 50.0,
            ..GridOptions::default()
        },
    )
    .unwrap();

    assert_eq!(
        plain.len(),
        extended.len(),
        "overshoot must not change the number of lines"
    );

    let width = |points: &[LatLon]| -> f64 {
        points
            .chunks(2)
            .map(|l| (l[1].longitude() - l[0].longitude()).abs())
            .fold(0.0_f64, f64::max)
    };
    let grew = (width(&extended) - width(&plain)) * 111_320.0 * (-35.36f64).to_radians().cos();
    assert!(
        (grew - 50.0).abs() < 5.0,
        "50 m of overshoot should lengthen a line by about 50 m, measured {grew:.1}"
    );

    // And every line runs on past the area by the overshoot, at its end and not its start.
    for (line, before) in extended.chunks(2).zip(plain.chunks(2)) {
        assert_eq!(line[0], before[0], "the overshoot moved a line's start");
        let (east, north) = plane(before[1], line[1]);
        let beyond = east.hypot(north);
        assert!(
            (beyond - 50.0).abs() < 1.0,
            "a line ends {beyond:.1} m past its edge, not 50"
        );
    }
}

#[test]
fn the_angle_is_the_bearing_of_the_lines() {
    // As in Mission Planner: 0 flies north-south lines, 90 east-west (Grid.cs:773-780 treats the
    // angle as a compass bearing).
    let area = square();
    for (angle, north_south) in [(0.0, true), (90.0, false), (180.0, true), (270.0, false)] {
        let waypoints = grid(
            &area,
            &GridOptions {
                spacing: 100.0,
                angle,
                ..GridOptions::default()
            },
        )
        .unwrap();
        for line in waypoints.chunks(2) {
            let (east, north) = plane(line[0], line[1]);
            if east.hypot(north) < 1.0 {
                continue;
            }
            // Within the ~1.3 degrees UTM grid north differs from true north here.
            let along_north = north.abs() > 20.0 * east.abs();
            let along_east = east.abs() > 20.0 * north.abs();
            assert!(
                if north_south { along_north } else { along_east },
                "at {angle} degrees a line runs {east:.1} m east and {north:.1} m north"
            );
        }
    }
}

#[test]
fn degenerate_requests_are_refused() {
    assert_eq!(
        grid(
            &[at(-35.0, 149.0), at(-35.0, 149.1)],
            &GridOptions::default()
        ),
        Err(GridError::NotAPolygon { found: 2 })
    );
    assert_eq!(
        grid(
            &square(),
            &GridOptions {
                spacing: 0.0,
                ..GridOptions::default()
            }
        ),
        Err(GridError::BadSpacing)
    );
    assert_eq!(
        grid(
            &square(),
            &GridOptions {
                spacing: f64::NAN,
                ..GridOptions::default()
            }
        ),
        Err(GridError::BadSpacing)
    );

    // An area collapsed to a point has no line through it (golden `cbr_point_a0`).
    let point = vec![at(-35.3632, 149.1652); 3];
    assert_eq!(
        grid(&point, &GridOptions::default()),
        Err(GridError::DegenerateArea)
    );
}

#[test]
fn a_flat_area_still_gets_lines_across_it() {
    // Three collinear vertices enclose nothing, but Mission Planner does not refuse them: lines
    // crossing the flat polygon find two intersections on its edges and are kept, at zero length
    // (golden `cbr_flat_a0`). The wrapper passes that through rather than inventing a refusal.
    let flat = vec![at(-35.35, 149.15), at(-35.35, 149.16), at(-35.35, 149.17)];
    let waypoints = grid(
        &flat,
        &GridOptions {
            spacing: 100.0,
            ..GridOptions::default()
        },
    )
    .unwrap();
    assert!(!waypoints.is_empty());
    for line in waypoints.chunks(2) {
        let (east, north) = plane(line[0], line[1]);
        assert!(
            east.hypot(north) < 0.1,
            "a line over a flat area has length"
        );
    }
}
