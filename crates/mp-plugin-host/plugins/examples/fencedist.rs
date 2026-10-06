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

//! `plugins/example3-fencedist.cs`: "Draw Fence Dist" on the flight screen's map menu, and
//! `testCode`, the distance from a point to the nearest fence (`CurrentState.GeoFenceDist`).
//!
//! Left out: the click's picture. The C# samples `testCode` over a 400 x 400 grid of the map's
//! view area and draws it as a `GMapMarkerFill` on an overlay it adds to `FDGMapControl`; the
//! world has no map drawing surface, so the click measures from the vehicle and puts the
//! distance on the status line.

use std::sync::{Mutex, PoisonError};

use mp_plugins::host::{self, FencePoint, MapMenu};
use mp_plugins::{Guest, bearing, distance};

/// `MAV_CMD_NAV_FENCE_RETURN_POINT`.
const FENCE_RETURN_POINT: u16 = 5000;
/// `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION`.
const FENCE_POLYGON_VERTEX_INCLUSION: u16 = 5001;
/// `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION`.
const FENCE_POLYGON_VERTEX_EXCLUSION: u16 = 5002;
/// `MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION`.
const FENCE_CIRCLE_INCLUSION: u16 = 5003;
/// `MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION`.
const FENCE_CIRCLE_EXCLUSION: u16 = 5004;

/// The menu entry's id.
static ENTRY: Mutex<Option<u32>> = Mutex::new(None);

struct FenceDist;

/// `ChunkByField` with the example's rule: a circle stands alone; vertices of one command group
/// until the group holds `param1` of them. `// C#: plugins/example3-fencedist.cs:76-93;
/// ExtLibs/Utilities/Extensions.cs:253-278`
fn chunks(points: &[FencePoint]) -> Vec<Vec<FencePoint>> {
    let points: Vec<FencePoint> = points
        .iter()
        .filter(|point| point.command != FENCE_RETURN_POINT)
        .cloned()
        .collect();
    let mut out = Vec::new();
    let mut rest = &points[..];
    while let Some(first) = rest.first() {
        let mut take = 1;
        for (count, point) in rest.iter().enumerate().skip(1) {
            let matching = !(first.command == FENCE_CIRCLE_EXCLUSION
                || first.command == FENCE_CIRCLE_INCLUSION)
                && f64::from(u32::try_from(count).unwrap_or(u32::MAX)) < f64::from(point.param1)
                && first.command == point.command;
            if !matching {
                break;
            }
            take += 1;
        }
        let (chunk, tail) = rest.split_at(take);
        out.push(chunk.to_vec());
        rest = tail;
    }
    out
}

/// `PolygonTools.orientation`. `// C#: ExtLibs/ArduPilot/PolygonTools.cs:36-49`
fn orientation(p: (f64, f64), q: (f64, f64), r: (f64, f64)) -> u8 {
    let val = (q.1 - p.1) * (r.0 - q.0) - (q.0 - p.0) * (r.1 - q.1);
    if val == 0.0 {
        0
    } else if val > 0.0 {
        1
    } else {
        2
    }
}

/// `PolygonTools.onSegment`. `// C#: ExtLibs/ArduPilot/PolygonTools.cs:22-33`
fn on_segment(p: (f64, f64), q: (f64, f64), r: (f64, f64)) -> bool {
    q.0 <= p.0.max(r.0) && q.0 >= p.0.min(r.0) && q.1 <= p.1.max(r.1) && q.1 >= p.1.min(r.1)
}

/// `PolygonTools.doIntersect`. `// C#: ExtLibs/ArduPilot/PolygonTools.cs:52-104`
fn intersect(p1: (f64, f64), q1: (f64, f64), p2: (f64, f64), q2: (f64, f64)) -> bool {
    let (o1, o2) = (orientation(p1, q1, p2), orientation(p1, q1, q2));
    let (o3, o4) = (orientation(p2, q2, p1), orientation(p2, q2, q1));
    (o1 != o2 && o3 != o4)
        || (o1 == 0 && on_segment(p1, p2, q1))
        || (o2 == 0 && on_segment(p1, q2, q1))
        || (o3 == 0 && on_segment(p2, p1, q2))
        || (o4 == 0 && on_segment(p2, q1, q2))
}

/// `PolygonTools.isInside`: a ray to x = 360 crossing the sides an odd number of times.
/// `// C#: ExtLibs/ArduPilot/PolygonTools.cs:112-156`
fn inside(polygon: &[(f64, f64)], p: (f64, f64)) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }
    let extreme = (360.0, p.1);
    let mut count = 0;
    for i in 0..n {
        let (Some(&a), Some(&b)) = (polygon.get(i), polygon.get((i + 1) % n)) else {
            continue;
        };
        if intersect(a, b, p, extreme) {
            if orientation(a, p, b) == 0 {
                return on_segment(a, p, b);
            }
            count += 1;
        }
    }
    count % 2 == 1
}

/// `CloseLoop`: the first point again at the end, unless it is the last already. The C#'s items
/// are `KeyValuePair`s keyed by their index, so the first equals the last only when they are one
/// item. `// C#: ExtLibs/Utilities/Extensions.cs:834-843`
fn close_loop(points: &[FencePoint]) -> Vec<FencePoint> {
    let mut out = points.to_vec();
    if let (Some(first), true) = (points.first(), points.len() > 1) {
        out.push(*first);
    }
    out
}

/// `testCode`: the distance in metres from (`lat`, `lng`) to the nearest fence, 0 when the point
/// breaches one, 99999 when there is none. `// C#: plugins/example3-fencedist.cs:67-205`
fn fence_distance(points: &[FencePoint], lat: f64, lng: f64) -> f64 {
    let mut total = 99_999.0_f64;
    let list = chunks(points);
    for sublist in &list {
        if let [item] = sublist.as_slice() {
            let dist = distance(item.lat, item.lng, lat, lng);
            let radius = f64::from(item.param1);
            if item.command == FENCE_CIRCLE_EXCLUSION {
                if dist < radius {
                    return 0.0;
                }
                total = total.min(dist - radius);
            } else if item.command == FENCE_CIRCLE_INCLUSION {
                if dist > radius {
                    return 0.0;
                }
                total = total.min(radius - dist);
            }
        }
        if sublist.len() < 3 {
            continue;
        }
        let polygon: Vec<(f64, f64)> = close_loop(sublist)
            .iter()
            .map(|point| (point.lng, point.lat))
            .collect();
        let command = sublist.first().map_or(0, |first| first.command);
        if inside(&polygon, (lng, lat)) {
            if command == FENCE_POLYGON_VERTEX_EXCLUSION {
                return 0.0;
            }
        } else if command == FENCE_POLYGON_VERTEX_INCLUSION {
            return 0.0;
        }
        // The cross-track distance to each side the point is abreast of.
        let closed = close_loop(sublist);
        for pair in closed.windows(2) {
            let [start, end] = pair else { continue };
            let line = distance(start.lat, start.lng, end.lat, end.lng);
            let to_point = distance(start.lat, start.lng, lat, lng);
            let mut angle = bearing(start.lat, start.lng, lat, lng)
                - bearing(start.lat, start.lng, end.lat, end.lng);
            if angle < 0.0 {
                angle += 360.0;
            }
            let along = angle.to_radians().cos() * to_point;
            if along < 0.0 || along > line {
                continue;
            }
            let cross = angle.to_radians().sin() * to_point;
            total = total.min(cross.abs());
        }
    }
    // And the vertices themselves: outside a polygon the nearest may be a corner.
    for point in list.iter().flatten() {
        if point.command == FENCE_CIRCLE_INCLUSION {
            continue;
        }
        total = total.min(distance(point.lat, point.lng, lat, lng).abs());
    }
    // `(float)`: the C# keeps the distance as a float.
    #[allow(clippy::cast_possible_truncation)]
    let single = total as f32;
    f64::from(single)
}

impl Guest for FenceDist {
    fn name() -> String {
        "FenceDist".to_owned()
    }

    fn version() -> String {
        "0.10".to_owned()
    }

    fn author() -> String {
        "Michael Oborne".to_owned()
    }

    fn loop_rate_hz() -> f32 {
        0.0
    }

    fn init() -> bool {
        true
    }

    // `// C#: plugins/example3-fencedist.cs:46-53`
    fn loaded() -> bool {
        let id = host::menu_add(MapMenu::FlightData, None, "Draw Fence Dist");
        *ENTRY.lock().unwrap_or_else(PoisonError::into_inner) = Some(id);
        true
    }

    fn run_loop() -> bool {
        true
    }

    fn exit() -> bool {
        true
    }

    // `// C#: plugins/example3-fencedist.cs:213-249`, the picture left out (see the header).
    fn menu_click(id: u32, _lat: f64, _lng: f64) {
        if *ENTRY.lock().unwrap_or_else(PoisonError::into_inner) != Some(id) {
            return;
        }
        let (Some(lat), Some(lng)) = (mp_plugins::cs_number("lat"), mp_plugins::cs_number("lng"))
        else {
            return;
        };
        let dist = fence_distance(&host::fence_points(), lat, lng);
        host::status(&format!("Fence distance {dist:.1} m"));
    }

    fn form_event(_id: String, _value: String) {}
}

mp_plugins::export_plugin!(FenceDist);
