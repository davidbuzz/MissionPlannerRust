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

//! Auto WP > Create Wp Circle and Create Spline Circle: the points the planning screen's map menu
//! puts round the click.
//!
//! Ported from `GCSViews/FlightPlanner.cs` @ efb0801 (GPL-3.0-only),
//! `createWpCircleToolStripMenuItem_Click` (2963-3048) and
//! `createSplineCircleToolStripMenuItem_Click` (2857-2961), from the parse of their answers on;
//! the prompts and the rows are the planning screen's. Both place a point with the same lines: a
//! great-circle destination on a sphere of 6371 km, the radius over that sphere divided in
//! `float`. `testdata/planner/golden/{wp,spline}_circle.csv` are those lines run under mono
//! (`tools/csharp-reference/PlannerOracle.cs`), and the tests hold these to them bit for bit.

/// `MathHelper.rad2deg`, `ExtLibs/Utilities/Math.cs:10`.
const RAD2DEG: f64 = 180.0 / std::f64::consts::PI;

/// `MathHelper.deg2rad`, `ExtLibs/Utilities/Math.cs:11`: `1.0 / rad2deg`, which is not the same
/// double as `PI / 180`.
const DEG2RAD: f64 = 1.0 / RAD2DEG;

/// How many points a circle may place.
///
/// **A divergence.** The C# loops for as long as its angle stays in range, so a very large point
/// count, or a spline circle whose altitude never reaches its top, adds rows until the process
/// runs out of memory. A mission's items are numbered by a `uint16` on the wire, so no more than
/// this many could ever be sent; the loops stop there.
pub const MAX_POINTS: usize = u16::MAX as usize;

/// The point `radius` metres from (`lat`, `lng`) on bearing `a` degrees, as both handlers write
/// it: `float d = Radius; float R = 6371000;` and `d / R` in `float`, the rest in `double`.
/// `// C#: GCSViews/FlightPlanner.cs:2932-2944, 3027-3039`
fn destination(lat: f64, lng: f64, radius: i32, a: f64) -> (f64, f64) {
    #[allow(clippy::cast_precision_loss)] // `float d = Radius;`
    let d = radius as f32;
    let r: f32 = 6_371_000.0;
    let dr = f64::from(d / r);
    let lat2 = ((lat * DEG2RAD).sin() * dr.cos()
        + (lat * DEG2RAD).cos() * dr.sin() * (a * DEG2RAD).cos())
    .asin();
    let lon2 = lng * DEG2RAD
        + ((a * DEG2RAD).sin() * dr.sin() * (lat * DEG2RAD).cos())
            .atan2(dr.cos() - (lat * DEG2RAD).sin() * lat2.sin());
    (lat2 * RAD2DEG, lon2 * RAD2DEG)
}

/// `double step = 360.0f / Points;`: a `float` division, widened.
fn step(points: i32) -> f64 {
    #[allow(clippy::cast_precision_loss)] // `(float) Points`
    let points = points as f32;
    f64::from(360.0_f32 / points)
}

/// Create Wp Circle's points, in the order the rows are added: from `start_angle` round by
/// `360 / points` while the angle is between 0 and `start_angle + 360`, both included - so a
/// whole circle ends on its first point again - or the other way round from `start_angle + 360`
/// when `direction` is -1. Each is (latitude, longitude), as `setfromMap` is handed them; nothing
/// wraps a longitude past 180, as nothing in the C# does.
///
/// The C#'s own corners are kept: no points gives one point (`360f / 0` is infinity); a negative
/// start angle gives none; a start angle past one step, turning backwards, goes round until the
/// angle passes 0.
/// `// C#: GCSViews/FlightPlanner.cs:3009-3044`
#[must_use]
pub fn wp_circle(
    lat: f64,
    lng: f64,
    radius: i32,
    points: i32,
    direction: i32,
    start_angle: i32,
) -> Vec<(f64, f64)> {
    let mut a = f64::from(start_angle);
    let mut step = step(points);
    if direction == -1 {
        a += 360.0;
        step *= -1.0;
    }
    let end = f64::from(start_angle.wrapping_add(360));
    let mut out = Vec::new();
    while a <= end && a >= 0.0 && out.len() < MAX_POINTS {
        out.push(destination(lat, lng, radius, a));
        a += step;
    }
    out
}

/// Why a spline circle was not made.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SplineCircleError {
    /// The altitude would never reach its top: see [`spline_circle`].
    AltStep,
}

/// Create Spline Circle's points after its `DO_SET_ROI`, in the order the rows are added: laps of
/// five points - north, east, south, west and north again (`Points = 4`, the angle from 0 to 360
/// inclusive; the start angle it asks for is never parsed) - each with the altitude `setfromMap` is
/// handed. The first lap is all at `min_alt`; every point after it climbs `alt_step / 4` in
/// integer division, and a lap starts while the altitude is at most `max_alt`.
///
/// **A divergence.** Where `alt_step / 4` is not positive and a lap is flown at all, the C#'s
/// altitude never passes `max_alt` and it adds laps forever, until the process dies. That is
/// refused here with [`SplineCircleError::AltStep`], which the screen says in the C#'s words for a
/// step it cannot parse, "Bad alt step".
/// `// C#: GCSViews/FlightPlanner.cs:2918-2958`
pub fn spline_circle(
    lat: f64,
    lng: f64,
    radius: i32,
    min_alt: i32,
    max_alt: i32,
    alt_step: i32,
) -> Result<Vec<(f64, f64, i32)>, SplineCircleError> {
    const POINTS: i32 = 4;
    const START_ANGLE: i32 = 0;
    let climb = alt_step / POINTS;
    if climb <= 0 && min_alt <= max_alt {
        return Err(SplineCircleError::AltStep);
    }
    let step = step(POINTS);
    let mut out = Vec::new();
    let mut startup = true;
    let mut step_alt = min_alt;
    while step_alt <= max_alt && out.len() < MAX_POINTS {
        let mut a = 0.0;
        while a <= f64::from(START_ANGLE + 360) && a >= 0.0 && out.len() < MAX_POINTS {
            let (lat2, lng2) = destination(lat, lng, radius, a);
            out.push((lat2, lng2, step_alt));
            if !startup {
                step_alt = step_alt.wrapping_add(climb);
            }
            a += step;
        }
        // reset back to the start
        if startup {
            step_alt = min_alt;
        }
        // we have finsihed the first run
        startup = false;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOLDEN_WP: &str = include_str!("../../../testdata/planner/golden/wp_circle.csv");
    const GOLDEN_SPLINE: &str = include_str!("../../../testdata/planner/golden/spline_circle.csv");

    /// Each `case` line and the lines under it, as fields.
    fn cases(golden: &str) -> Vec<(Vec<String>, Vec<Vec<String>>)> {
        let mut out: Vec<(Vec<String>, Vec<Vec<String>>)> = Vec::new();
        for line in golden.lines() {
            let fields: Vec<String> = line.split(',').map(ToOwned::to_owned).collect();
            if fields.first().map(String::as_str) == Some("case") {
                out.push((fields, Vec::new()));
            } else if let Some((_, rows)) = out.last_mut() {
                rows.push(fields);
            }
        }
        out
    }

    fn number(field: &str) -> f64 {
        field.parse().expect("a number in the golden")
    }

    fn integer(field: &str) -> i32 {
        field.parse().expect("an integer in the golden")
    }

    /// Every case of `wp_circle.csv`: the same number of points, each the same doubles - to the
    /// bit on Linux, where the golden was made, and within [`mp_units::golden_match`]'s allowance
    /// where the platform's libm rounds a last bit otherwise (Windows: one ulp, CI run
    /// 37173996196).
    #[test]
    fn a_wp_circle_is_the_c_sharps_to_the_bit() {
        let cases = cases(GOLDEN_WP);
        assert_eq!(cases.len(), 6);
        for (case, rows) in cases {
            let got = wp_circle(
                number(&case[2]),
                number(&case[3]),
                integer(&case[4]),
                integer(&case[5]),
                integer(&case[6]),
                integer(&case[7]),
            );
            assert_eq!(got.len(), rows.len(), "{}", case[1]);
            for (index, (point, row)) in got.iter().zip(&rows).enumerate() {
                assert_eq!(row[0], "wp");
                let (lat, lng) = (number(&row[1]), number(&row[2]));
                assert!(
                    mp_units::golden_match(point.0, lat),
                    "{} {index}: latitude {} where the C# has {lat}",
                    case[1],
                    point.0
                );
                assert!(
                    mp_units::golden_match(point.1, lng),
                    "{} {index}: longitude {} where the C# has {lng}",
                    case[1],
                    point.1
                );
            }
        }
    }

    /// The C#'s corners, named: twenty points close on the first; none gives one; a negative
    /// start gives none; turning backwards from 100 degrees goes on round to 10.
    #[test]
    fn a_wp_circle_keeps_the_c_sharps_corners() {
        let (lat, lng) = (-35.363_262_1, 149.165_237_4);
        let full = wp_circle(lat, lng, 50, 20, 1, 0);
        assert_eq!(full.len(), 21);
        assert_eq!(full.first(), full.last());
        assert_eq!(wp_circle(lat, lng, 50, 0, 1, 0).len(), 1);
        assert!(wp_circle(lat, lng, 50, 20, 1, -10).is_empty());
        assert_eq!(wp_circle(lat, lng, 50, 20, -1, 100).len(), 26);
    }

    /// Every case of `spline_circle.csv`: the ROI's lines aside, the same points at the same
    /// altitudes.
    #[test]
    fn a_spline_circle_is_the_c_sharps_to_the_bit() {
        let cases = cases(GOLDEN_SPLINE);
        assert_eq!(cases.len(), 3);
        for (case, rows) in cases {
            let got = spline_circle(
                number(&case[2]),
                number(&case[3]),
                integer(&case[4]),
                integer(&case[5]),
                integer(&case[6]),
                integer(&case[7]),
            )
            .expect("a step that climbs");
            let points: Vec<&Vec<String>> = rows.iter().filter(|row| row[0] == "wp").collect();
            assert_eq!(got.len(), points.len(), "{}", case[1]);
            for (index, (point, row)) in got.iter().zip(points).enumerate() {
                assert_eq!(
                    point.0.to_bits(),
                    number(&row[1]).to_bits(),
                    "{} {index}",
                    case[1]
                );
                assert_eq!(
                    point.1.to_bits(),
                    number(&row[2]).to_bits(),
                    "{} {index}",
                    case[1]
                );
                assert_eq!(point.2, integer(&row[3]), "{} {index}", case[1]);
            }
        }
    }

    /// A step under four climbs nothing a point, and the C# would lap forever: refused. Over an
    /// empty band it laps nothing and is not.
    #[test]
    fn a_spline_circle_that_would_never_end_is_refused() {
        let (lat, lng) = (-35.363_262_1, 149.165_237_4);
        assert_eq!(
            spline_circle(lat, lng, 50, 5, 20, 3),
            Err(SplineCircleError::AltStep)
        );
        assert_eq!(
            spline_circle(lat, lng, 50, 5, 20, -8),
            Err(SplineCircleError::AltStep)
        );
        assert_eq!(spline_circle(lat, lng, 50, 30, 20, 3), Ok(Vec::new()));
    }

    /// A point count the C# would grow rows from until it ran out of memory stops at what a
    /// mission can number.
    #[test]
    fn a_circle_stops_at_what_a_mission_can_number() {
        let points = wp_circle(0.0, 0.0, 10, i32::MAX, 1, 0);
        assert_eq!(points.len(), MAX_POINTS);
    }
}
