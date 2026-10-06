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

//! `GeoFenceDist`: how far the vehicle is from the nearest edge of its geofence.
//!
//! The fence itself is not vehicle state - the C# reads it from `MAVState.fencepoints`, which the
//! fence download and the mission-protocol traffic on the link fill (`MAVLinkInterface.cs:4112,
//! 5641-5694`) - so whoever holds the fence passes it in, as [`FenceItem`]s in sequence order.
//! The distance is the C#'s arithmetic step for step: its chunking of the items into shapes, its
//! point-in-polygon test, its cross-track distance to each edge that the vehicle is beside, and
//! the plain distance to every vertex.
//! `// C#: ExtLibs/ArduPilot/CurrentState.cs:1617-1753`

use crate::state::VehicleState;

/// `MAV_CMD.FENCE_RETURN_POINT`, and the four shapes after it.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:1301-1317`
const FENCE_RETURN_POINT: u16 = 5000;
/// `MAV_CMD.FENCE_POLYGON_VERTEX_INCLUSION`.
const FENCE_POLYGON_VERTEX_INCLUSION: u16 = 5001;
/// `MAV_CMD.FENCE_POLYGON_VERTEX_EXCLUSION`.
const FENCE_POLYGON_VERTEX_EXCLUSION: u16 = 5002;
/// `MAV_CMD.FENCE_CIRCLE_INCLUSION`.
const FENCE_CIRCLE_INCLUSION: u16 = 5003;
/// `MAV_CMD.FENCE_CIRCLE_EXCLUSION`.
const FENCE_CIRCLE_EXCLUSION: u16 = 5004;

/// `MathHelper.deg2rad`, `1 / (180 / PI)`, which is `PI / 180` to the bit.
/// `// C#: ExtLibs/Utilities/Math.cs:10-11`
const DEG2RAD: f64 = std::f64::consts::PI / 180.0;

/// One item of a fence, the parts of `mavlink_mission_item_int_t` that `GeoFenceDist` reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FenceItem {
    /// `MAV_CMD`: a polygon vertex, a circle or the return point.
    pub command: u16,
    /// A polygon's vertex count, or a circle's radius in metres.
    pub param1: f32,
    /// Latitude, degrees times 1e7.
    pub x: i32,
    /// Longitude, degrees times 1e7.
    pub y: i32,
}

/// A position in degrees, `(lat, lng)`, as the C#'s `PointLatLngAlt` holds one: no range check.
type Point = (f64, f64);

/// `PointLatLngAlt.GetDistance`: haversine on a 6371 km sphere, metres.
/// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:382-393`
fn get_distance(a: Point, b: Point) -> f64 {
    let d = a.0 * 0.017_453_292_519_943_295;
    let num2 = a.1 * 0.017_453_292_519_943_295;
    let num3 = b.0 * 0.017_453_292_519_943_295;
    let num4 = b.1 * 0.017_453_292_519_943_295;
    let num5 = num4 - num2;
    let num6 = num3 - d;
    let num7 = (num6 / 2.0).sin().powi(2) + ((d.cos() * num3.cos()) * (num5 / 2.0).sin().powi(2));
    let num8 = 2.0 * num7.sqrt().atan2((1.0 - num7).sqrt());
    (6371.0 * num8) * 1000.0
}

/// `PointLatLngAlt.GetDistance2`: the same haversine written the other way, metres.
/// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:395-409`
fn get_distance2(a: Point, b: Point) -> f64 {
    let r = 6371.0;
    let d_lat = (b.0 - a.0) * DEG2RAD;
    let d_lon = (b.1 - a.1) * DEG2RAD;
    let lat1 = a.0 * DEG2RAD;
    let lat2 = b.0 * DEG2RAD;
    let h = (d_lat / 2.0).sin() * (d_lat / 2.0).sin()
        + (d_lon / 2.0).sin() * (d_lon / 2.0).sin() * lat1.cos() * lat2.cos();
    let c = 2.0 * h.sqrt().atan2((1.0 - h).sqrt());
    r * c * 1000.0
}

/// `PointLatLngAlt.GetBearing`, degrees in `[0, 360)`.
/// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:350-360`
fn get_bearing(a: Point, b: Point) -> f64 {
    let latitude1 = DEG2RAD * a.0;
    let latitude2 = DEG2RAD * b.0;
    let longitude_difference = DEG2RAD * (b.1 - a.1);
    let y = longitude_difference.sin() * latitude2.cos();
    let x = latitude1.cos() * latitude2.sin()
        - latitude1.sin() * latitude2.cos() * longitude_difference.cos();
    ((180.0 / std::f64::consts::PI) * y.atan2(x) + 360.0) % 360.0
}

/// .NET's `Math.Min(double, double)`: NaN if either is.
fn min(a: f64, b: f64) -> f64 {
    if a < b || a.is_nan() { a } else { b }
}

/// A double narrowed to a C# `float`.
#[allow(clippy::cast_possible_truncation)]
const fn cast_f32(value: f64) -> f32 {
    value as f32
}

/// An item's position, `x / 1e7, y / 1e7`.
fn position(item: &FenceItem) -> Point {
    (f64::from(item.x) / 1e7, f64::from(item.y) / 1e7)
}

/// `Extensions.ChunkByField` with `GeoFenceDist`'s rule: each shape is a run of items starting
/// at one, taking the next while it has the first one's command and fewer items have been taken
/// than its own `param1` says - except that a circle stands alone. As `(start, length)` ranges.
/// `// C#: ExtLibs/Utilities/Extensions.cs:253-276; ExtLibs/ArduPilot/CurrentState.cs:1630-1643`
fn chunks(items: &[FenceItem]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut start = 0;
    while let Some(rest) = items.get(start..).filter(|rest| !rest.is_empty()) {
        let first = rest.first().copied();
        let taken = rest
            .iter()
            .enumerate()
            .take_while(|(count, b)| {
                // The first item itself: `first.Equals(a)` on the dictionary's pairs, whose keys
                // are the sequence numbers, so only the item itself.
                let Some(a) = first.filter(|_| *count != 0) else {
                    return true;
                };
                // C#: CurrentState.cs:1633-1642
                if a.command == FENCE_CIRCLE_EXCLUSION || a.command == FENCE_CIRCLE_INCLUSION {
                    return false;
                }
                // `count >= b.Value.param1`: an int compared as a float.
                #[allow(clippy::cast_precision_loss)]
                let count = *count as f32;
                if count >= b.param1 {
                    return false;
                }
                a.command == b.command
            })
            .count();
        out.push((start, taken));
        start += taken;
    }
    out
}

/// `PolygonTools.orientation`: 0 colinear, 1 clockwise, 2 anticlockwise, of `(x, y)` points.
/// `// C#: ExtLibs/ArduPilot/PolygonTools.cs:41-52`
fn orientation(p: Point, q: Point, r: Point) -> u8 {
    let val = (q.1 - p.1) * (r.0 - q.0) - (q.0 - p.0) * (r.1 - q.1);
    if val == 0.0 {
        0
    } else if val > 0.0 {
        1
    } else {
        2
    }
}

/// `PolygonTools.onSegment`. `// C#: ExtLibs/ArduPilot/PolygonTools.cs:24-34`
fn on_segment(p: Point, q: Point, r: Point) -> bool {
    q.0 <= p.0.max(r.0) && q.0 >= p.0.min(r.0) && q.1 <= p.1.max(r.1) && q.1 >= p.1.min(r.1)
}

/// `PolygonTools.doIntersect`. `// C#: ExtLibs/ArduPilot/PolygonTools.cs:56-104`
fn do_intersect(p1: Point, q1: Point, p2: Point, q2: Point) -> bool {
    let o1 = orientation(p1, q1, p2);
    let o2 = orientation(p1, q1, q2);
    let o3 = orientation(p2, q2, p1);
    let o4 = orientation(p2, q2, q1);
    (o1 != o2 && o3 != o4)
        || (o1 == 0 && on_segment(p1, p2, q1))
        || (o2 == 0 && on_segment(p1, q2, q1))
        || (o3 == 0 && on_segment(p2, p1, q2))
        || (o4 == 0 && on_segment(p2, q1, q2))
}

/// `PolygonTools.isInside`: a ray to x = 360 counted across the edges, of `(x, y)` points.
/// `// C#: ExtLibs/ArduPilot/PolygonTools.cs:112-154`
fn is_inside(polygon: &[Point], p: Point) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }
    let extreme = (360.0, p.1);
    let mut count = 0;
    let mut i = 0;
    loop {
        let next = (i + 1) % n;
        if let (Some(&a), Some(&b)) = (polygon.get(i), polygon.get(next))
            && do_intersect(a, b, p, extreme)
        {
            if orientation(a, p, b) == 0 {
                return on_segment(a, p, b);
            }
            count += 1;
        }
        i = next;
        if i == 0 {
            break;
        }
    }
    count % 2 == 1
}

impl VehicleState {
    /// `GeoFenceDist`: metres from the vehicle to the nearest edge of `fence` - 0 when it is
    /// outside an inclusion zone or inside an exclusion one, and 99999 with no fence.
    ///
    /// `fence` is the vehicle's fence items in sequence order: `MAVState.fencepoints`, a
    /// `ConcurrentDictionary` keyed by sequence number, which enumerates small integer keys in
    /// that order. The position is the C#'s `Location`, (0, 0) without one.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1617-1753`
    #[must_use]
    pub fn geo_fence_dist(&self, fence: &[FenceItem]) -> f32 {
        let location: Point = self
            .position
            .map_or((0.0, 0.0), |p| (p.latitude(), p.longitude()));
        let mut disttotal: f32 = 99999.0;
        // C#: CurrentState.cs:1627-1628
        let items: Vec<FenceItem> = fence
            .iter()
            .filter(|item| item.command != FENCE_RETURN_POINT)
            .copied()
            .collect();
        let shapes = chunks(&items);
        let shape = |&(start, length): &(usize, usize)| {
            items.get(start..start + length).unwrap_or_default()
        };

        for sublist in shapes.iter().map(shape) {
            // C#: CurrentState.cs:1648-1673, a circle
            if let [item] = sublist {
                let dist = get_distance(position(item), location);
                let radius = f64::from(item.param1);
                if item.command == FENCE_CIRCLE_EXCLUSION {
                    if dist < radius {
                        return 0.0;
                    }
                    disttotal = cast_f32(min(dist - radius, f64::from(disttotal)));
                } else if item.command == FENCE_CIRCLE_INCLUSION {
                    if dist > radius {
                        return 0.0;
                    }
                    disttotal = cast_f32(min(radius - dist, f64::from(disttotal)));
                }
            }
            // C#: CurrentState.cs:1675-1676
            let Some(first) = sublist.first().filter(|_| sublist.len() >= 3) else {
                continue;
            };
            // C#: CurrentState.cs:1678-1695. `CloseLoop` always appends the first item again:
            // its `Equals` compares the dictionary's pairs, whose keys differ.
            let closed = || sublist.iter().chain(std::iter::once(first));
            let polygon: Vec<Point> = closed()
                .map(|item| (f64::from(item.y) / 1e7, f64::from(item.x) / 1e7))
                .collect();
            if is_inside(&polygon, (location.1, location.0)) {
                if first.command == FENCE_POLYGON_VERTEX_EXCLUSION {
                    return 0.0;
                }
            } else if first.command == FENCE_POLYGON_VERTEX_INCLUSION {
                return 0.0;
            }
            // C#: CurrentState.cs:1697-1735, the cross-track distance to each edge the vehicle
            // is beside.
            let mut line_start: Option<Point> = None;
            for item in closed() {
                let Some(start) = line_start else {
                    line_start = Some(position(item));
                    continue;
                };
                let line_end = position(item);
                let line_dist = get_distance2(start, line_end);
                let dist_to_location = get_distance2(start, location);
                let bear_to_location = get_bearing(start, location);
                let line_bear = get_bearing(start, line_end);
                let mut angle = bear_to_location - line_bear;
                if angle < 0.0 {
                    angle += 360.0;
                }
                let alongline = (angle * DEG2RAD).cos() * dist_to_location;
                line_start = Some(line_end);
                if alongline < 0.0 || alongline > line_dist {
                    continue;
                }
                // The C#'s spherical `dXt` beside it is computed and never used.
                let d_xt2 = (angle * DEG2RAD).sin() * dist_to_location;
                disttotal = cast_f32(min(f64::from(disttotal), d_xt2.abs()));
            }
        }

        // C#: CurrentState.cs:1738-1746, every point but an inclusion circle's centre.
        for item in shapes.iter().flat_map(shape) {
            if item.command == FENCE_CIRCLE_INCLUSION {
                continue;
            }
            let d_xt2 = get_distance(position(item), location);
            disttotal = cast_f32(min(f64::from(disttotal), d_xt2.abs()));
        }
        disttotal
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vertex(command: u16, param1: f32) -> FenceItem {
        FenceItem {
            command,
            param1,
            x: 0,
            y: 0,
        }
    }

    #[test]
    fn items_chunk_into_shapes_by_command_and_count() {
        let items = [
            vertex(FENCE_POLYGON_VERTEX_INCLUSION, 3.0),
            vertex(FENCE_POLYGON_VERTEX_INCLUSION, 3.0),
            vertex(FENCE_POLYGON_VERTEX_INCLUSION, 3.0),
            // The same command again: a second polygon, because the first has its three.
            vertex(FENCE_POLYGON_VERTEX_INCLUSION, 3.0),
            vertex(FENCE_POLYGON_VERTEX_INCLUSION, 3.0),
            vertex(FENCE_POLYGON_VERTEX_EXCLUSION, 3.0),
            vertex(FENCE_CIRCLE_EXCLUSION, 10.0),
            vertex(FENCE_CIRCLE_EXCLUSION, 10.0),
        ];
        assert_eq!(chunks(&items), [(0, 3), (3, 2), (5, 1), (6, 1), (7, 1)]);
        assert!(chunks(&[]).is_empty());
    }

    #[test]
    fn a_square_contains_its_centre_and_not_a_point_beside_it() {
        let square = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)];
        assert!(is_inside(&square, (0.5, 0.5)));
        assert!(!is_inside(&square, (1.5, 0.5)));
        // On an edge counts as inside.
        assert!(is_inside(&square, (1.0, 0.5)));
        assert!(
            !is_inside(&square[..2], (0.5, 0.5)),
            "fewer than three points"
        );
    }

    #[test]
    fn math_min_is_nan_if_either_side_is() {
        assert!(min(f64::NAN, 1.0).is_nan());
        assert!(min(1.0, f64::NAN).is_nan());
        assert_eq!(min(1.0, 2.0).to_bits(), 1.0_f64.to_bits());
    }
}
