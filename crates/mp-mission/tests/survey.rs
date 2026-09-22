//! Survey grid properties.
//!
//! The grid is not compared against a golden file: this implementation works in a local tangent
//! plane where the reference uses UTM, so the coordinates differ in the last metre by design. What
//! must hold is geometric, and geometric properties are stronger tests than a golden file anyway -
//! a golden file proves the output has not changed, these prove it is correct.

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

/// An L shape, to exercise lines that cross the boundary more than twice.
fn concave() -> Vec<LatLon> {
    vec![
        at(-35.360, 149.160),
        at(-35.360, 149.171),
        at(-35.365, 149.171),
        at(-35.365, 149.165),
        at(-35.369, 149.165),
        at(-35.369, 149.160),
    ]
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
    // wrong test - half a metre of tolerance is the projection's own error.
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
    for spacing in [50.0, 100.0, 200.0] {
        let options = GridOptions {
            spacing,
            ..GridOptions::default()
        };
        let waypoints = grid(&area, &options).expect("generates");

        // Lines run east-west at angle 0, so consecutive lines differ in latitude by the spacing.
        let mut latitudes: Vec<f64> = waypoints.chunks(2).map(|line| line[0].latitude()).collect();
        latitudes.sort_by(|a, b| a.partial_cmp(b).unwrap());
        latitudes.dedup_by(|a, b| (*a - *b).abs() < 1e-9);

        assert!(
            latitudes.len() >= 3,
            "spacing {spacing} produced only {} lines",
            latitudes.len()
        );
        for pair in latitudes.windows(2) {
            let metres = (pair[1] - pair[0]) * 111_320.0;
            assert!(
                (metres - spacing).abs() < spacing * 0.02,
                "requested {spacing} m spacing, measured {metres:.1} m"
            );
        }
    }
}

#[test]
fn no_point_in_the_area_is_further_than_half_a_spacing_from_a_line() {
    // The property that makes a survey a survey: complete coverage. A grid that leaves gaps
    // produces a photo mosaic with holes, which is only discovered after the flight.
    let area = square();
    let spacing = 100.0;
    let waypoints = grid(
        &area,
        &GridOptions {
            spacing,
            ..GridOptions::default()
        },
    )
    .expect("generates");
    let line_latitudes: Vec<f64> = waypoints.chunks(2).map(|l| l[0].latitude()).collect();

    // Sample the interior on a fine lattice.
    let mut worst: f64 = 0.0;
    for i in 1..40 {
        for j in 1..40 {
            let lat = -35.369 + (f64::from(i) / 40.0) * 0.009;
            let lon = 149.160 + (f64::from(j) / 40.0) * 0.011;
            let sample = at(lat, lon);
            if !contains(&area, sample) {
                continue;
            }
            let nearest = line_latitudes
                .iter()
                .map(|line_lat| (line_lat - lat).abs() * 111_320.0)
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
    // each pass, which roughly doubles the flight time.
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
        .map(|line| line[1].longitude() > line[0].longitude())
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
fn a_concave_area_produces_split_lines_rather_than_flying_through_the_notch() {
    // An L-shaped area has lines that cross the boundary four times. Joining those crossings into
    // one line flies the aircraft straight through the part that was deliberately excluded.
    let area = concave();
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
            "waypoint {:.6},{:.6} is {:.1} m into the excluded notch",
            point.latitude(),
            point.longitude(),
            distance_to_boundary(&area, *point)
        );
    }

    // Midpoints of each line must also be inside; that is what catches a line spanning the notch.
    for line in waypoints.chunks(2) {
        let mid = at(
            (line[0].latitude() + line[1].latitude()) / 2.0,
            (line[0].longitude() + line[1].longitude()) / 2.0,
        );
        assert!(
            contains_within(&area, mid, 0.5),
            "a line crosses the excluded notch"
        );
    }
}

#[test]
fn overshoot_extends_lines_beyond_the_boundary() {
    // A camera survey needs the aircraft straight and level before the first photo, so lines must
    // start outside the area.
    let area = square();
    let plain = grid(
        &area,
        &GridOptions {
            spacing: 100.0,
            ..GridOptions::default()
        },
    )
    .unwrap();
    let extended = grid(
        &area,
        &GridOptions {
            spacing: 100.0,
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
        (grew - 100.0).abs() < 10.0,
        "50 m of overshoot at each end should lengthen a line by about 100 m, measured {grew:.1}"
    );
}

#[test]
fn rotating_the_grid_rotates_the_lines() {
    let area = square();
    let east_west = grid(
        &area,
        &GridOptions {
            spacing: 100.0,
            angle: 0.0,
            ..GridOptions::default()
        },
    )
    .unwrap();
    let north_south = grid(
        &area,
        &GridOptions {
            spacing: 100.0,
            angle: 90.0,
            ..GridOptions::default()
        },
    )
    .unwrap();

    let spread = |points: &[LatLon], f: fn(&LatLon) -> f64| -> f64 {
        let values: Vec<f64> = points.iter().map(f).collect();
        values.iter().copied().fold(f64::NEG_INFINITY, f64::max)
            - values.iter().copied().fold(f64::INFINITY, f64::min)
    };

    // East-west lines vary mostly in longitude along each line; rotated 90 degrees they vary in
    // latitude instead.
    let ew_ratio = spread(&east_west, |p| p.longitude()) / spread(&east_west, |p| p.latitude());
    let ns_ratio = spread(&north_south, |p| p.longitude()) / spread(&north_south, |p| p.latitude());
    assert!(
        ew_ratio > ns_ratio,
        "rotating by 90 degrees should change the dominant axis"
    );
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

    // A polygon with no height in the rotated frame encloses nothing.
    let flat = vec![at(-35.0, 149.0), at(-35.0, 149.1), at(-35.0, 149.2)];
    assert_eq!(
        grid(&flat, &GridOptions::default()),
        Err(GridError::DegenerateArea)
    );
}
