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

//! Corridor surveys as Mission Planner lays them out: `Grid.CreateCorridor`, transliterated.
//!
//! Replaces `ExtLibs/Utilities/Grid.cs:46-188`: `CreateCorridor` and `GenerateOffsetPath`. The
//! corridor is a set of lanes parallel to a centre line, `distance` apart across `width`, flown
//! alternately along and against it. Each lane is the centre line offset sideways leg by leg, with
//! consecutive offset legs joined where their lines cross (`Grid.cs:124`).
//!
//! Like [`crate::grid`] this follows the C# statement for statement, in its floating-point order,
//! including what looks accidental: a centre line of two points - one leg, no corner - has no lanes
//! at all (`Grid.cs:109`); legs whose offsets are parallel (a repeated vertex, three collinear ones)
//! meet at `utmpos.Zero`, which the C# flies to (`Grid.cs:834`); trigger points are laid at whole
//! metres (`Grid.cs:143`, an `int` loop). `CreateCorridor` also takes an angle, overshoots, a shutter
//! flag, a lane separation and a lead-in, and reads none of them; [`CorridorArgs`] has no field for
//! them.
//!
//! The result is compared bit for bit against `CreateCorridor` run under mono, by
//! `tests/corridor_vectors.rs` over `testdata/grid`.

use mp_units::LatLon;

use crate::grid::{
    GridPoint, GridTag, StartPosition, cs_int, find_line_intersection_extension, newpos, newpos_xy,
};
use crate::survey::GridError;
use crate::utm::{UtmPos, to_utm, utm_zone};

/// `CreateCorridor`'s arguments (`Grid.cs:55-57`) that reach the pattern, named as the C# names
/// them.
///
/// [`Default`] gives what `GridUI` passes on a fresh install (`GridUI.cs:592-596`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CorridorArgs {
    /// Altitude given to every point, metres. `NUM_altitude`, default 100.
    pub altitude: f64,
    /// Distance between lanes, metres. `NUM_Distance`, default 50; below 0.1 it is 0.1.
    pub distance: f64,
    /// Distance between trigger points along a lane, metres, or 0 for none. `NUM_spacing`,
    /// default 0; a non-zero value below 4 is 4, and the points fall at whole metres.
    pub spacing: f64,
    /// Where the corridor starts. Only [`StartPosition::Home`] is told apart (`Grid.cs:87`): it
    /// flies the first lane from the centre line's first point, every other choice from its last.
    /// `CMB_startfrom`, default Home.
    pub startpos: StartPosition,
    /// Width of the corridor, metres. `num_corridorwidth`, default 100, which `GridUI` passes
    /// through `float` (`GridUI.cs:596`). There are `2 * (int)(width / distance / 2) + 1` lanes.
    pub width: f64,
}

impl Default for CorridorArgs {
    fn default() -> Self {
        Self {
            altitude: 100.0,
            distance: 50.0,
            spacing: 0.0,
            startpos: StartPosition::Home,
            width: 100.0,
        }
    }
}

/// `Grid.CreateCorridor`, `Grid.cs:55-101`: the corridor along `polygon`, which is the centre line
/// (the C# calls it `polygon` too), in flight order.
///
/// Each lane is `S`, `SM`, any `M` trigger points up to the first corner, then at each corner an
/// `S` and the trigger points of the next leg, and finally `ME` and `E`. An empty centre line, or one
/// of fewer than three points, gives no points, as the C# does.
///
/// # Errors
///
/// [`GridError::Projection`] if a point cannot be converted back from UTM, where the C# throws.
pub fn create_corridor(
    polygon: &[LatLon],
    args: &CorridorArgs,
) -> Result<Vec<GridPoint>, GridError> {
    let mut spacing = args.spacing;
    let mut distance = args.distance;

    // C#: Grid.cs:59-63
    if spacing < 4.0 && spacing != 0.0 {
        spacing = 4.0;
    }

    if distance < 0.1 {
        distance = 0.1;
    }

    let Some(first) = polygon.first() else {
        return Ok(Vec::new());
    };

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

    // C#: Grid.cs:76-78. `(lanes / 2) * -1` multiplies by the int -1, widened: an exact negation.
    let lanes = args.width / distance;
    let start = cs_int(-(lanes / 2.0));
    // `start * -1` in unchecked int arithmetic: int.MinValue stays itself.
    let end = start.wrapping_neg();

    let mut lane = start;
    while lane <= end {
        // correct side of the line we are on because of list reversal
        let multi: i32 = if lane.wrapping_sub(start) % 2 == 1 {
            -1
        } else {
            1
        };

        if args.startpos != StartPosition::Home {
            utmpositions.reverse();
        }

        // C#: Grid.cs:90, `distance * multi * lane`: double times int, then times int.
        let offset = distance * f64::from(multi) * f64::from(lane);
        ans.extend(generate_offset_path(
            &utmpositions,
            offset,
            spacing,
            utmzone,
        ));

        if args.startpos == StartPosition::Home {
            utmpositions.reverse();
        }

        // The C#'s `lane++` past int.MaxValue wraps and loops for ever; no width reaches it.
        if lane == end {
            break;
        }
        lane += 1;
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

/// `Grid.GenerateOffsetPath`, `Grid.cs:103-188`: one lane, `distance` to the right of the line
/// through `utmpositions` (negative for the left).
fn generate_offset_path(
    utmpositions: &[UtmPos],
    distance: f64,
    spacing: f64,
    utmzone: i32,
) -> Vec<(UtmPos, GridTag)> {
    let mut ans: Vec<(UtmPos, GridTag)> = Vec::new();

    let mut oldpos = UtmPos::ZERO;

    // `for (a = 0; a < Count - 2; a++)`: one window per corner, none for fewer than three points.
    for (a, corner) in utmpositions.windows(3).enumerate() {
        let [prev_center, curr_center, next_center] = *corner else {
            continue;
        };

        let l1bearing = prev_center.bearing(curr_center);
        let l2bearing = curr_center.bearing(next_center);

        let l1prev = newpos(prev_center, l1bearing + 90.0, distance);
        let l1curr = newpos(curr_center, l1bearing + 90.0, distance);

        let l2curr = newpos(curr_center, l2bearing + 90.0, distance);
        let l2next = newpos(next_center, l2bearing + 90.0, distance);

        let l1l2center = find_line_intersection_extension(l1prev, l1curr, l2curr, l2next);

        //start
        if a == 0 {
            // add start
            ans.push((l1prev, GridTag::Start));

            // add start/trigger
            ans.push((l1prev, GridTag::StartMiddle));

            oldpos = l1prev;
        }

        //spacing
        if spacing > 0.0 {
            lay_triggers(&mut ans, oldpos, l1l2center, l1bearing, spacing, utmzone);
        }

        //end of leg
        ans.push((l1l2center, GridTag::Start));
        oldpos = l1l2center;

        // last leg
        if a + 3 == utmpositions.len() {
            if spacing > 0.0 {
                lay_triggers(&mut ans, l1l2center, l2next, l2bearing, spacing, utmzone);
            }

            ans.push((l2next, GridTag::MiddleEnd));

            ans.push((l2next, GridTag::End));
        }
    }

    ans
}

/// The trigger loops of `Grid.cs:143-153` and `:166-176`, which differ only in their ends: an `M`
/// point every `(int)spacing` whole metres from `from` along `bearing`, starting at
/// `(int)(length % spacing)`, while short of `to`.
fn lay_triggers(
    ans: &mut Vec<(UtmPos, GridTag)>,
    from: UtmPos,
    to: UtmPos,
    bearing: f64,
    spacing: f64,
    utmzone: i32,
) {
    let step = cs_int(spacing);
    let mut d = cs_int(from.distance(to) % spacing);
    while f64::from(d) < from.distance(to) {
        let mut ax = from.x;
        let mut ay = from.y;

        newpos_xy(&mut ax, &mut ay, bearing, f64::from(d));
        ans.push((UtmPos::new(ax, ay, utmzone), GridTag::Middle));

        // A spacing of 2^31 m or more makes the C#'s int step int.MinValue, and `d` then cycles
        // for ever; there is no pattern in a hang to port.
        if step <= 0 {
            break;
        }
        d = d.wrapping_add(step);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line() -> Vec<LatLon> {
        [
            (-35.3600, 149.1600),
            (-35.3650, 149.1680),
            (-35.3620, 149.1760),
        ]
        .into_iter()
        .map(|(lat, lng)| LatLon::new(lat, lng).unwrap())
        .collect()
    }

    #[test]
    fn an_empty_line_and_a_single_leg_have_no_lanes() {
        let args = CorridorArgs::default();
        assert_eq!(create_corridor(&[], &args), Ok(Vec::new()));
        assert_eq!(create_corridor(&line()[..2], &args), Ok(Vec::new()));
    }

    #[test]
    fn lanes_are_two_times_half_the_width_over_distance_plus_one() {
        // A three-point line without triggers: S, SM, the corner's S, ME, E per lane.
        for (width, distance, lanes) in [(100.0, 50.0, 3), (1.0, 50.0, 1), (250.0, 40.0, 7)] {
            let args = CorridorArgs {
                width,
                distance,
                ..CorridorArgs::default()
            };
            let points = create_corridor(&line(), &args).unwrap();
            assert_eq!(
                points.len(),
                lanes * 5,
                "width {width}, distance {distance}"
            );
            for lane in points.chunks(5) {
                let tags: Vec<&str> = lane.iter().map(|p| p.tag.as_str()).collect();
                assert_eq!(tags, ["S", "SM", "S", "ME", "E"]);
            }
        }
    }

    #[test]
    fn home_flies_the_first_lane_from_the_first_point_and_anything_else_from_the_last() {
        let single = CorridorArgs {
            width: 1.0,
            ..CorridorArgs::default()
        };
        let line = line();
        let from_home = create_corridor(&line, &single).unwrap();
        let from_corner = create_corridor(
            &line,
            &CorridorArgs {
                startpos: StartPosition::BottomLeft,
                ..single
            },
        )
        .unwrap();
        let near = |p: &GridPoint, q: LatLon| {
            (p.lat - q.latitude()).abs() < 1e-9 && (p.lng - q.longitude()).abs() < 1e-9
        };
        assert!(near(&from_home[0], line[0]));
        assert!(near(&from_corner[0], line[2]));
    }

    #[test]
    fn a_huge_trigger_spacing_does_not_hang() {
        let args = CorridorArgs {
            spacing: 1e10,
            ..CorridorArgs::default()
        };
        assert!(create_corridor(&line(), &args).is_ok());
    }
}
