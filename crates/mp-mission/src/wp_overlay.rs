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

//! `WPOverlay`'s lines: the route the planning and flight maps draw through a mission, and the
//! polygons and circles they draw for a fence.
//!
//! Ported from `ExtLibs/Maps/WPOverlay.cs` (GPL-3.0-only): `CreateOverlay`'s collection of the
//! route's points - which commands are on it, the loiter's exit at a tangent, a DO_JUMP's
//! repeats, the route cut at a land and a DO_LAND_START - and `RegenerateWPRoute`, which turns
//! each run of points into the "wp route", sampling a run of spline waypoints with `Spline2`
//! ([`crate::spline2`]), and its "home route", the legs from the last point to home and from home
//! to the first; and the fence half of `CreateOverlay`, its polygons and circles. The markers
//! `CreateOverlay` adds with them are the map's (`crates/mp-gui/src/mapview.rs`).

use crate::MissionItem;
use crate::circle_survey::newpos;
use crate::spline2::{DEFAULT_ACCEL_CMS, DEFAULT_SPEED_CMS, SegmentEnd, Spline2};

/// `MathHelper.rad2deg`. `// C#: ExtLibs/Utilities/Math.cs:10`
const RAD2DEG: f64 = 180.0 / std::f64::consts::PI;

/// `MathHelper.deg2rad`. `// C#: ExtLibs/Utilities/Math.cs:11`
const DEG2RAD: f64 = 1.0 / RAD2DEG;

/// The most points a mission's routes may hold.
///
/// **A divergence.** A DO_JUMP's repeats are copied into the route as often as its Param2 says,
/// so a jump repeated a million times makes the C# build - and draw, every frame - a route of
/// millions of points. The route stops taking repeats past this many, which no mission flies.
pub const MAX_ROUTE_POINTS: usize = 100_000;

/// The most steps one spline segment is sampled in.
///
/// **A divergence.** The C# steps until the spline's time reaches 1, which a position that is
/// not a number never lets it do; a segment stops here, thousands of steps past any real one.
pub const MAX_SPLINE_STEPS: usize = 100_000;

/// The `MAV_CMD` values `CreateOverlay` tests for. `// C#: ExtLibs/Mavlink/Mavlink.cs (MAV_CMD)`
mod cmd {
    pub(super) const WAYPOINT: u16 = 16;
    pub(super) const LOITER_UNLIM: u16 = 17;
    pub(super) const LOITER_TURNS: u16 = 18;
    pub(super) const LOITER_TIME: u16 = 19;
    pub(super) const RETURN_TO_LAUNCH: u16 = 20;
    pub(super) const LAND: u16 = 21;
    pub(super) const CONTINUE_AND_CHANGE_ALT: u16 = 30;
    pub(super) const LOITER_TO_ALT: u16 = 31;
    pub(super) const SPLINE_WAYPOINT: u16 = 82;
    pub(super) const VTOL_LAND: u16 = 85;
    pub(super) const GUIDED_ENABLE: u16 = 92;
    pub(super) const DELAY: u16 = 93;
    pub(super) const LAST: u16 = 95;
    pub(super) const DO_JUMP: u16 = 177;
    pub(super) const DO_RETURN_PATH_START: u16 = 188;
    pub(super) const DO_LAND_START: u16 = 189;
    pub(super) const DO_SET_ROI: u16 = 201;
    pub(super) const FENCE_RETURN_POINT: u16 = 5000;
    pub(super) const FENCE_POLYGON_VERTEX_INCLUSION: u16 = 5001;
    pub(super) const FENCE_POLYGON_VERTEX_EXCLUSION: u16 = 5002;
    pub(super) const FENCE_CIRCLE_INCLUSION: u16 = 5003;
    pub(super) const FENCE_CIRCLE_EXCLUSION: u16 = 5004;
    pub(super) const RALLY_POINT: u16 = 5100;
}

/// Whether `CreateOverlay` counts a command among the "navigatable points": below `MAV_CMD.LAST`
/// except RTL, CONTINUE_AND_CHANGE_ALT, DELAY and GUIDED_ENABLE, and DO_SET_ROI, DO_LAND_START and
/// DO_RETURN_PATH_START besides.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:66-73`
#[must_use]
pub const fn navigable(command: u16) -> bool {
    use cmd::{
        CONTINUE_AND_CHANGE_ALT, DELAY, DO_LAND_START, DO_RETURN_PATH_START, DO_SET_ROI,
        GUIDED_ENABLE, LAST, RETURN_TO_LAUNCH,
    };
    (command < LAST
        && !matches!(
            command,
            RETURN_TO_LAUNCH | CONTINUE_AND_CHANGE_ALT | DELAY | GUIDED_ENABLE
        ))
        || matches!(command, DO_SET_ROI | DO_LAND_START | DO_RETURN_PATH_START)
}

/// One point of a route: a `PointLatLngAlt`, its altitude above sea level as far as `GetHomeAlt`
/// can make it so, and whether it carries `Tag2 = "spline"`.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:215-218`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoutePoint {
    /// `Lat`.
    pub lat: f64,
    /// `Lng`.
    pub lng: f64,
    /// `Alt`, metres.
    pub alt: f64,
    /// `Tag2 == "spline"`: a SPLINE_WAYPOINT's.
    pub spline: bool,
}

/// The pen a `GMapRoute` is drawn with.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:576-589`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RoutePen {
    /// The "wp route": `new Pen(Color.Yellow, 4)` with `DashStyle.Custom`, which `GMapRoute`
    /// draws solid with an arrow on each leg.
    Mission,
    /// The "home route": `new Pen(Color.Yellow, 2)`, `DashStyle.Dash` where home is under 5 km
    /// from both of the route's ends - "if we have a large distance between home and the
    /// first/last point, it hangs on the draw of a the dashed line" - and solid otherwise.
    Home {
        /// `DashStyle.Dash`.
        dashed: bool,
    },
}

/// One `GMapRoute`: its points, latitude and longitude, in order, and its pen.
#[derive(Debug, Clone, PartialEq)]
pub struct Route {
    /// `Points`.
    pub points: Vec<(f64, f64)>,
    /// `Stroke`.
    pub pen: RoutePen,
}

/// `Spline2._wp_accel_cms` and `_wp_speed_cms`, static in the C#: their defaults until
/// `processToScreen` has read a vehicle's WPNAV_ACCEL and WPNAV_SPEED.
/// `// C#: ExtLibs/Utilities/Spline2.cs:56-58; GCSViews/FlightPlanner.cs:5565-5586`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SplineRates {
    /// `_wp_accel_cms`, cm/s/s.
    pub accel_cms: f32,
    /// `_wp_speed_cms`, cm/s.
    pub speed_cms: f32,
}

impl Default for SplineRates {
    fn default() -> Self {
        Self {
            accel_cms: DEFAULT_ACCEL_CMS,
            speed_cms: DEFAULT_SPEED_CMS,
        }
    }
}

/// `PointLatLngAlt.GetDistance`: a haversine on a 6371 km sphere, metres.
/// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:382-393`
#[must_use]
pub fn get_distance(from: (f64, f64), to: (f64, f64)) -> f64 {
    let d = from.0 * 0.017_453_292_519_943_295;
    let num2 = from.1 * 0.017_453_292_519_943_295;
    let num3 = to.0 * 0.017_453_292_519_943_295;
    let num4 = to.1 * 0.017_453_292_519_943_295;
    let num5 = num4 - num2;
    let num6 = num3 - d;
    let num7 = (num6 / 2.0).sin().powi(2) + ((d.cos() * num3.cos()) * (num5 / 2.0).sin().powi(2));
    let num8 = 2.0 * num7.sqrt().atan2((1.0 - num7).sqrt());
    (6371.0 * num8) * 1000.0
}

/// `PointLatLngAlt.GetBearing`: the initial great-circle bearing, 0 to 360 degrees.
/// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:350-360`
#[must_use]
pub fn get_bearing(from: (f64, f64), to: (f64, f64)) -> f64 {
    let latitude1 = DEG2RAD * from.0;
    let latitude2 = DEG2RAD * to.0;
    let longitude_difference = DEG2RAD * (to.1 - from.1);
    let y = longitude_difference.sin() * latitude2.cos();
    let x = latitude1.cos() * latitude2.sin()
        - latitude1.sin() * latitude2.cos() * longitude_difference.cos();
    (RAD2DEG * y.atan2(x) + 360.0) % 360.0
}

/// `Math.Sign` of a `double`, as an `int`: -1, 0 or 1.
fn sign(value: f64) -> f64 {
    if value > 0.0 {
        1.0
    } else if value < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// The mission's routes as `WPOverlay.CreateOverlay` builds them and `RegenerateWPRoute` draws
/// them, in the order they join the overlay's `Routes`: each run cut at a land or a DO_LAND_START
/// as a "wp route" with no home route, then the last run's home route and wp route.
///
/// `home` is `CreateOverlay`'s home - latitude, longitude and altitude - or `None` for
/// `PointLatLngAlt.Zero`, which puts no "H" first; `loiter_radius` its `loiterradius`, which a
/// loiter exiting at a tangent turns on when its command carries no radius of its own;
/// `home_alt` its `GetHomeAlt(frame, lat, lng)`, what a row's altitude is put above. Item 0,
/// where `items` holds one, is home's (`mission_items.RemoveAt(0)` on the flight screen) and is
/// skipped.
///
/// What the route takes, row by row: a waypoint, takeoff and the other navigable commands with
/// both coordinates; a spline waypoint wherever it is, marked to be curved; a loiter with either
/// coordinate, and where Param4 is 1 its exit point, a tangent to its circle towards the next row;
/// a land or a DO_LAND_START with both, each ending the run before it; a waypoint at 0,0 the
/// point before it again; a DO_JUMP the points from its target row to itself, Param2 times
/// (`pointlist` indexed by the row, so - as in the C# - one row off without home, and further
/// off after a land at 0,0, which adds nothing to `pointlist`). DO_SET_ROI, RTL, the DO commands,
/// the fence and rally commands are not on it.
///
/// Where the C# throws - a jump to a row past the end of `pointlist`, a spline whose run has no
/// point before it - the C#'s `writeKML` fails and its map keeps the overlay it had; here the
/// jump takes the rows there are, and the run with the spline draws nothing.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:28-357, 441-591`
#[must_use]
pub fn mission_routes(
    home: Option<(f64, f64, f64)>,
    items: &[MissionItem],
    loiter_radius: f64,
    rates: SplineRates,
    home_alt: &dyn Fn(u8, f64, f64) -> f64,
) -> Vec<Route> {
    use cmd::{
        DO_JUMP, DO_LAND_START, DO_SET_ROI, FENCE_CIRCLE_EXCLUSION, FENCE_CIRCLE_INCLUSION,
        FENCE_POLYGON_VERTEX_EXCLUSION, FENCE_POLYGON_VERTEX_INCLUSION, FENCE_RETURN_POINT, LAND,
        LOITER_TIME, LOITER_TO_ALT, LOITER_TURNS, LOITER_UNLIM, RALLY_POINT, SPLINE_WAYPOINT,
        VTOL_LAND, WAYPOINT,
    };
    let mut routes = Vec::new();
    let mut pointlist: Vec<Option<RoutePoint>> = Vec::new();
    let mut route: Vec<Option<RoutePoint>> = Vec::new();
    let home_point = home.map(|(lat, lng, alt)| RoutePoint {
        lat,
        lng,
        alt,
        spline: false,
    });
    if let Some(home) = home_point {
        pointlist.push(Some(home));
        route.push(Some(home));
    }
    let origin = home.map_or((0.0, 0.0), |(lat, lng, _)| (lat, lng));
    let items: Vec<&MissionItem> = items.iter().filter(|item| item.seq != 0).collect();

    for (a, item) in items.iter().enumerate() {
        let command = item.command;
        let zero = item.x == 0.0 && item.y == 0.0;
        let located = item.x != 0.0 && item.y != 0.0;
        // `Locationwp.alt` is a float, and `item.alt + gethomealt(...)` a double.
        #[allow(clippy::cast_possible_truncation)] // the grid's float, on purpose
        let alt = f64::from(item.z as f32);
        let point = |spline: bool| RoutePoint {
            lat: item.x,
            lng: item.y,
            alt: alt + home_alt(item.frame, item.x, item.y),
            spline,
        };

        // invalid locationwp
        if command == 0 {
            pointlist.push(None);
            continue;
        }

        if navigable(command) {
            // land can be 0,0 or a lat,lng
            if matches!(command, LAND | VTOL_LAND) && zero {
                continue;
            }
            if command == DO_LAND_START && located {
                let p = point(false);
                pointlist.push(Some(p));
                route.push(Some(p));
                // draw everything before
                if !route.is_empty() {
                    regenerate_wp_route(&mut route, origin, false, rates, &mut routes);
                    route.clear();
                }
                route.push(Some(p));
            } else if matches!(command, LAND | VTOL_LAND) && located {
                let p = point(false);
                pointlist.push(Some(p));
                route.push(Some(p));
                regenerate_wp_route(&mut route, origin, false, rates, &mut routes);
                route.clear();
            } else if command == DO_SET_ROI {
                // "do set roi is not a nav command. so we dont route through it"
                pointlist.push(Some(point(false)));
            } else if matches!(
                command,
                LOITER_TIME | LOITER_TURNS | LOITER_TO_ALT | LOITER_UNLIM
            ) {
                if zero {
                    // loiter at current location.
                    pointlist.push(None);
                } else {
                    let p = point(false);
                    pointlist.push(Some(p));
                    // The loiter's radius and direction, if the command gives one.
                    let mut this_loiterradius = loiter_radius;
                    if matches!(command, LOITER_TURNS | LOITER_UNLIM) {
                        if item.param3 != 0.0 {
                            this_loiterradius = item.param3;
                        }
                    } else if command == LOITER_TO_ALT && item.param2 != 0.0 {
                        this_loiterradius = item.param2;
                    }
                    let loiterdirection = sign(this_loiterradius);
                    this_loiterradius = this_loiterradius.abs();

                    // exit at tangent
                    if item.param4 == 1.0 {
                        let from = (p.lat, p.lng);
                        let to = match items.get(a + 1) {
                            Some(next) if next.x != 0.0 && next.y != 0.0 => (next.x, next.y),
                            _ => from,
                        };
                        let bearing = get_bearing(from, to);
                        let dist = get_distance(from, to);
                        route.push(Some(p));
                        if dist > this_loiterradius {
                            let theta = (this_loiterradius / dist).acos() * RAD2DEG;
                            let (lat, lng) = newpos(
                                p.lat,
                                p.lng,
                                bearing - loiterdirection * theta,
                                this_loiterradius,
                            );
                            route.push(Some(RoutePoint {
                                lat,
                                lng,
                                alt: p.alt,
                                spline: false,
                            }));
                        }
                    } else {
                        route.push(Some(p));
                    }
                }
            } else if command == SPLINE_WAYPOINT {
                let p = point(true);
                pointlist.push(Some(p));
                route.push(Some(p));
            } else if command == WAYPOINT && zero {
                if let Some(last) = pointlist.last() {
                    route.push(*last);
                }
                pointlist.push(None);
            } else if located {
                let p = point(false);
                pointlist.push(Some(p));
                route.push(Some(p));
            } else {
                pointlist.push(None);
            }
        } else if command == DO_JUMP {
            // "fix do jumps into the future"
            pointlist.push(None);
            #[allow(clippy::cast_possible_truncation)] // `(int) Math.Max(item.p1, 0)`
            let wpno = (item.param1 as f32).max(0.0) as i32;
            #[allow(clippy::cast_possible_truncation)] // `(int) item.p2`
            let repeat = item.param2 as f32 as i32;
            let wpno = usize::try_from(wpno).unwrap_or(0);
            let mut list = Vec::new();
            // cycle through reps, then wps
            'reps: for _ in 0..repeat.max(0) {
                for no in wpno..=a {
                    if route.len() + list.len() >= MAX_ROUTE_POINTS {
                        break 'reps;
                    }
                    if let Some(Some(p)) = pointlist.get(no) {
                        list.push(Some(*p));
                    }
                }
            }
            route.extend(list);
        } else if matches!(
            command,
            FENCE_POLYGON_VERTEX_INCLUSION
                | FENCE_POLYGON_VERTEX_EXCLUSION
                | FENCE_CIRCLE_EXCLUSION
                | FENCE_CIRCLE_INCLUSION
                | FENCE_RETURN_POINT
                | RALLY_POINT
        ) {
            pointlist.push(Some(RoutePoint {
                lat: item.x,
                lng: item.y,
                alt: 0.0,
                spline: false,
            }));
        } else {
            pointlist.push(None);
        }
    }

    regenerate_wp_route(&mut route, origin, true, rates, &mut routes);
    routes
}

/// `RegenerateWPRoute(fullpointlist, HomeLocation, includehomeroute)`: a run of points as the
/// overlay's routes, onto `out`.
///
/// The run's first point is added again at its end, "this causes the spline to always have a
/// straight finish". Each run of spline points is sampled with a `Spline2` about `origin` - the
/// overlay's home, 0,0 without one - from the straight segment before it: every step of each
/// segment, then the spline point itself, the last segment ending straight towards the point after
/// the run. Of the points that come out - more than two, or nothing is drawn - the first and last
/// make the home route, last to first to second, when `includehomeroute`; the wp route is the rest
/// (all but the last copy of the first, without the home route).
/// `// C#: ExtLibs/Maps/WPOverlay.cs:441-591`
pub fn regenerate_wp_route(
    fullpointlist: &mut Vec<Option<RoutePoint>>,
    origin: (f64, f64),
    includehomeroute: bool,
    rates: SplineRates,
    out: &mut Vec<Route>,
) {
    let Some(&first) = fullpointlist.first() else {
        return;
    };
    let mut lastpnt = first;
    let mut lastpnt2 = first;
    let mut splinepnts: Vec<Option<RoutePoint>> = Vec::new();
    let mut wproute: Vec<(f64, f64)> = Vec::new();

    // "add home - this causes the spline to always have a straight finish"
    fullpointlist.push(first);

    for entry in fullpointlist.iter() {
        let Some(point) = *entry else {
            continue;
        };
        if point.spline {
            if splinepnts.is_empty() {
                splinepnts.push(lastpnt);
            }
            splinepnts.push(Some(point));
            continue;
        }
        if !splinepnts.is_empty() {
            splinepnts.push(Some(point));
            // The C# reads every one of these as a point: a run whose spline has no point before
            // it throws there.
            let (Some(from), Some(to)) = (lastpnt2, lastpnt) else {
                return;
            };
            let Some(splined) = splinepnts.iter().copied().collect::<Option<Vec<_>>>() else {
                return;
            };
            wproute.extend(sample_splines(&splined, from, to, origin, rates));
            splinepnts.clear();
        }
        wproute.push((point.lat, point.lng));
        lastpnt2 = lastpnt;
        lastpnt = Some(point);
    }

    // homeroute = last, home, first
    let count = wproute.len();
    if count <= 2 {
        return;
    }
    let mut homepoint = (0.0, 0.0);
    let mut firstpoint = (0.0, 0.0);
    let mut lastpoint = (0.0, 0.0);
    let mut home_points = Vec::new();
    let mut points = Vec::with_capacity(count);
    for (index, x) in wproute.iter().enumerate() {
        let counter = index + 1;
        if counter == 1 && includehomeroute {
            homepoint = *x;
            continue;
        }
        if counter == 2 {
            firstpoint = *x;
        }
        if counter == count - 1 {
            lastpoint = *x;
        }
        if counter == count {
            if includehomeroute {
                home_points = vec![lastpoint, homepoint, firstpoint];
            }
            continue;
        }
        points.push(*x);
    }
    let dashed =
        get_distance(homepoint, lastpoint) < 5000.0 && get_distance(homepoint, firstpoint) < 5000.0;
    if includehomeroute {
        out.push(Route {
            points: home_points,
            pen: RoutePen::Home { dashed },
        });
    }
    out.push(Route {
        points,
        pen: RoutePen::Mission,
    });
}

/// One run of spline points as `RegenerateWPRoute` samples it: `splinepnts` is the point before
/// the run, the run, and the point after it; `from` and `to` the straight segment the run follows
/// (`lastpnt2`, `lastpnt`). Each spline point is a segment's destination, stepped along with
/// `advance_spline_target_along_track(1f)` until it is reached - every step's target taken - and
/// then the point itself; the last segment ends straight, the others into the next spline.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:473-513`
#[must_use]
pub fn sample_splines(
    splinepnts: &[RoutePoint],
    from: RoutePoint,
    to: RoutePoint,
    origin: (f64, f64),
    rates: SplineRates,
) -> Vec<(f64, f64)> {
    let mut list = Vec::new();
    let mut sp = Spline2::new(origin.0, origin.1, rates.accel_cms, rates.speed_cms);
    let from = sp.pv_location_to_vector(from.lat, from.lng, from.alt);
    let to = sp.pv_location_to_vector(to.lat, to.lng, to.alt);
    sp.set_wp_origin_and_destination(from, to);
    sp.set_reached_destination(true);
    let count = splinepnts.len();
    for (no, pair) in splinepnts.windows(2).enumerate().skip(1) {
        let [destination, next] = pair else {
            continue;
        };
        let segtype = if no < count.saturating_sub(2) {
            SegmentEnd::Spline
        } else {
            SegmentEnd::Straight
        };
        let target = sp.pv_location_to_vector(destination.lat, destination.lng, destination.alt);
        let after = sp.pv_location_to_vector(next.lat, next.lng, next.alt);
        sp.set_spline_destination(target, false, segtype, after);
        let mut steps = 0;
        while !sp.reached_destination() && steps < MAX_SPLINE_STEPS {
            sp.advance_spline_target_along_track(1.0);
            let (lat, lng, _) = sp.pv_vector_to_location(sp.target_pos);
            list.push((lat, lng));
            steps += 1;
        }
        list.push((destination.lat, destination.lng));
    }
    list
}

/// One fence item as `CreateOverlay` reads it from `MAV.fencepoints`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FenceVertex {
    /// `id`, the `MAV_CMD`.
    pub command: u16,
    /// `p1`: a polygon's vertex count, a circle's radius in metres.
    pub param1: f32,
    /// `lat`.
    pub lat: f64,
    /// `lng`.
    pub lng: f64,
}

/// A fence polygon `CreateOverlay` closed: its vertices, and whether the vertex that completed it
/// was an inclusion's - `Pens.Pink` and no fill - or an exclusion's - `Pens.Red` over
/// `Color.FromArgb(30, 255, 0, 0)`.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:292-322`
#[derive(Debug, Clone, PartialEq)]
pub struct FencePolygon {
    /// The vertices, latitude and longitude.
    pub points: Vec<(f64, f64)>,
    /// Whether it was drawn as an inclusion.
    pub inclusion: bool,
}

/// A fence circle: the `GMapMarkerRect` round a circle item, its `wprad` the item's Param1 - an
/// inclusion's in `Color.Blue`, an exclusion's in `Color.Red` filled `Color.FromArgb(30, 255, 0,
/// 0)`.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:323-334`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FenceCircle {
    /// The centre, latitude and longitude.
    pub centre: (f64, f64),
    /// The radius, metres.
    pub radius: f64,
    /// Whether it is an inclusion.
    pub inclusion: bool,
}

/// What `CreateOverlay` draws of a fence: the polygons it closed, the circles, and a
/// `GMarkerGoogleType.blue_dot` pin on every vertex, circle and return point.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FenceOverlay {
    /// The closed polygons, in the order they closed.
    pub polygons: Vec<FencePolygon>,
    /// The circles, in order.
    pub circles: Vec<FenceCircle>,
    /// The `blue_dot` pins, latitude and longitude.
    pub pins: Vec<(f64, f64)>,
}

/// The fence half of `CreateOverlay`: the vertices go into one polygon at a time, whichever kind
/// each is, until the polygon holds as many as the vertex's Param1 says - only then is it drawn,
/// in the colours of the kind of vertex that completed it - and a circle item is a circle of its
/// Param1. Every fence item has its pin; the return point's rect, `Color.Orange`, has no radius
/// and draws no circle.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:292-340, 401-405`
#[must_use]
pub fn fence_overlay(items: &[FenceVertex]) -> FenceOverlay {
    use cmd::{
        FENCE_CIRCLE_EXCLUSION, FENCE_CIRCLE_INCLUSION, FENCE_POLYGON_VERTEX_EXCLUSION,
        FENCE_POLYGON_VERTEX_INCLUSION, FENCE_RETURN_POINT,
    };
    let mut overlay = FenceOverlay::default();
    let mut fencepoly: Option<Vec<(f64, f64)>> = None;
    for item in items {
        let at = (item.lat, item.lng);
        match item.command {
            FENCE_POLYGON_VERTEX_INCLUSION | FENCE_POLYGON_VERTEX_EXCLUSION => {
                let polygon = fencepoly.get_or_insert_with(Vec::new);
                polygon.push(at);
                overlay.pins.push(at);
                // `fencepoly.Points.Count == item.p1`: the count compared as a float.
                #[allow(clippy::cast_precision_loss)] // a vertex count
                let complete = polygon.len() as f32 == item.param1;
                if complete && let Some(points) = fencepoly.take() {
                    overlay.polygons.push(FencePolygon {
                        points,
                        inclusion: item.command == FENCE_POLYGON_VERTEX_INCLUSION,
                    });
                }
            }
            FENCE_CIRCLE_EXCLUSION | FENCE_CIRCLE_INCLUSION => {
                overlay.pins.push(at);
                overlay.circles.push(FenceCircle {
                    centre: at,
                    radius: f64::from(item.param1),
                    inclusion: item.command == FENCE_CIRCLE_INCLUSION,
                });
            }
            FENCE_RETURN_POINT => overlay.pins.push(at),
            _ => {}
        }
    }
    overlay
}

#[cfg(test)]
mod tests {
    use super::*;

    const HOME: (f64, f64, f64) = (-35.363_262_1, 149.165_237_4, 584.0);

    fn row(seq: u16, command: u16, lat: f64, lng: f64) -> MissionItem {
        MissionItem {
            seq,
            command,
            x: lat,
            y: lng,
            z: 100.0,
            ..MissionItem::default()
        }
    }

    /// `GetHomeAlt` for a relative frame: home's altitude.
    fn relative(_frame: u8, _lat: f64, _lng: f64) -> f64 {
        HOME.2
    }

    fn routes(home: Option<(f64, f64, f64)>, items: &[MissionItem]) -> Vec<Route> {
        mission_routes(home, items, 45.0, SplineRates::default(), &relative)
    }

    fn at(item: &MissionItem) -> (f64, f64) {
        (item.x, item.y)
    }

    fn home() -> (f64, f64) {
        (HOME.0, HOME.1)
    }

    /// Three waypoints: the home route runs from the last to home to the first, dashed with home
    /// that close; the wp route is the three in order.
    #[test]
    fn three_waypoints_are_one_route_with_dashed_legs_to_home() {
        let items = [
            row(1, 16, -35.3620, 149.1650),
            row(2, 16, -35.3610, 149.1660),
            row(3, 16, -35.3620, 149.1670),
        ];
        let drawn = routes(Some(HOME), &items);
        assert_eq!(drawn.len(), 2);
        assert_eq!(drawn[0].pen, RoutePen::Home { dashed: true });
        assert_eq!(drawn[0].points, vec![at(&items[2]), home(), at(&items[0])]);
        assert_eq!(drawn[1].pen, RoutePen::Mission);
        assert_eq!(
            drawn[1].points,
            vec![at(&items[0]), at(&items[1]), at(&items[2])]
        );
    }

    /// Home 5 km or more from either end: the legs to home are solid.
    #[test]
    fn far_from_home_the_legs_are_solid() {
        let items = [
            row(1, 16, -35.3620, 149.1650),
            row(2, 16, -35.3000, 149.1650), // about 7 km north
            row(3, 16, -35.3010, 149.1660),
        ];
        let drawn = routes(Some(HOME), &items);
        assert_eq!(drawn[0].pen, RoutePen::Home { dashed: false });
        assert!(get_distance(home(), at(&items[2])) >= 5000.0);
    }

    /// Two points or fewer after the first copy is added again draw nothing; one waypoint draws
    /// the legs to it and a route of one point.
    #[test]
    fn a_route_needs_more_than_two_points() {
        assert!(routes(None, &[row(1, 16, -35.36, 149.16)]).is_empty());
        let one = [row(1, 16, -35.3620, 149.1650)];
        let drawn = routes(Some(HOME), &one);
        assert_eq!(drawn.len(), 2);
        assert_eq!(drawn[0].points, vec![at(&one[0]), home(), at(&one[0])]);
        assert_eq!(drawn[1].points, vec![at(&one[0])]);
    }

    /// Which commands are on the route: not DO_SET_ROI, RTL, DO commands, a waypoint missing a
    /// coordinate or a land at 0,0; a takeoff with a position is.
    #[test]
    fn the_route_takes_what_create_overlay_routes_through() {
        let items = [
            row(1, 22, -35.3620, 149.1650),  // takeoff
            row(2, 201, -35.3600, 149.1600), // DO_SET_ROI: not routed
            row(3, 183, 1.0, 2.0),           // DO_SET_SERVO: not navigable
            row(4, 16, -35.3610, 149.1660),
            row(5, 16, -35.3600, 0.0),   // one coordinate: not routed
            row(6, 20, -35.3000, 149.0), // RTL: not routed
            row(7, 21, 0.0, 0.0),        // land at 0,0: not routed, and no route cut
            row(8, 16, -35.3620, 149.1670),
        ];
        let drawn = routes(Some(HOME), &items);
        assert_eq!(drawn.len(), 2);
        assert_eq!(
            drawn[1].points,
            vec![at(&items[0]), at(&items[3]), at(&items[7])]
        );
    }

    /// A land with a position ends a run: drawn as a wp route from home with no legs to home, and
    /// what follows is the last run, whose "home" is its own first point.
    #[test]
    fn a_land_cuts_the_route() {
        let items = [
            row(1, 16, -35.3620, 149.1650),
            row(2, 21, -35.3610, 149.1660),
            row(3, 16, -35.3600, 149.1670),
            row(4, 16, -35.3590, 149.1680),
        ];
        let drawn = routes(Some(HOME), &items);
        assert_eq!(drawn.len(), 3);
        assert_eq!(drawn[0].pen, RoutePen::Mission);
        assert_eq!(drawn[0].points, vec![home(), at(&items[0]), at(&items[1])]);
        // [3, 4] and 3 again: home route 4 -> 3 -> 4, the wp route 4 alone.
        assert_eq!(drawn[1].pen, RoutePen::Home { dashed: true });
        assert_eq!(
            drawn[1].points,
            vec![at(&items[3]), at(&items[2]), at(&items[3])]
        );
        assert_eq!(drawn[2].points, vec![at(&items[3])]);
    }

    /// A DO_LAND_START draws what came before it, home first, and starts the next run at itself.
    #[test]
    fn a_do_land_start_draws_what_came_before() {
        let items = [
            row(1, 16, -35.3620, 149.1650),
            row(2, 16, -35.3615, 149.1655),
            row(3, 189, -35.3610, 149.1660),
            row(4, 16, -35.3600, 149.1670),
            row(5, 21, -35.3590, 149.1680),
        ];
        let drawn = routes(Some(HOME), &items);
        assert_eq!(drawn[0].pen, RoutePen::Mission);
        assert_eq!(
            drawn[0].points,
            vec![home(), at(&items[0]), at(&items[1]), at(&items[2])]
        );
        assert_eq!(
            drawn[1].points,
            vec![at(&items[2]), at(&items[3]), at(&items[4])]
        );
        assert_eq!(
            drawn.len(),
            2,
            "the land took the last run: nothing is left"
        );
    }

    /// A DO_JUMP repeats the points from its target row to itself into the route, Param2 times;
    /// with home, `pointlist[1]` is row 1.
    #[test]
    fn a_jump_repeats_its_rows_into_the_route() {
        let mut jump = row(3, 177, 0.0, 0.0);
        jump.param1 = 1.0;
        jump.param2 = 2.0;
        let items = [
            row(1, 16, -35.3620, 149.1650),
            row(2, 16, -35.3610, 149.1660),
            jump,
            row(4, 16, -35.3600, 149.1670),
        ];
        let drawn = routes(Some(HOME), &items);
        let (a, b, d) = (at(&items[0]), at(&items[1]), at(&items[3]));
        assert_eq!(drawn[1].points, vec![a, b, a, b, a, b, d]);
        // A repeat count of 0 or less adds nothing.
        let mut never = jump;
        never.param2 = -1.0;
        let drawn = routes(Some(HOME), &[items[0], items[1], never, items[3]]);
        assert_eq!(drawn[1].points, vec![a, b, d]);
    }

    /// A waypoint at 0,0 puts the point before it on the route again.
    #[test]
    fn a_waypoint_at_zero_repeats_the_point_before() {
        let items = [
            row(1, 16, -35.3620, 149.1650),
            row(2, 16, 0.0, 0.0),
            row(3, 16, -35.3610, 149.1660),
        ];
        let drawn = routes(Some(HOME), &items);
        let (a, c) = (at(&items[0]), at(&items[2]));
        assert_eq!(drawn[1].points, vec![a, a, c]);
    }

    /// A loiter exiting at a tangent (Param4 1) adds its exit point: on its circle, at the
    /// tangent towards the next row, turning the way its radius's sign says.
    #[test]
    fn a_loiter_exiting_at_a_tangent_adds_the_exit_point() {
        let mut loiter = row(2, 18, -35.3610, 149.1660);
        loiter.param3 = 60.0;
        loiter.param4 = 1.0;
        let next = row(3, 16, -35.3570, 149.1660); // due north, about 440 m
        let items = [row(1, 16, -35.3620, 149.1650), loiter, next];
        let drawn = routes(Some(HOME), &items);
        let points = &drawn[1].points;
        assert_eq!(points.len(), 4);
        let exit = points[2];
        let centre = at(&loiter);
        assert!((get_distance(centre, exit) - 60.0).abs() < 0.5);
        // The tangent: the angle at the exit between the centre and the next row is a right one.
        let dist = get_distance(centre, at(&next));
        let theta = (60.0 / dist).acos() * RAD2DEG;
        let bearing = get_bearing(centre, exit);
        let wanted = (get_bearing(centre, at(&next)) - theta + 360.0) % 360.0;
        assert!((bearing - wanted).abs() < 0.5, "{bearing} against {wanted}");
        // Counter-clockwise, a negative radius: the other side.
        loiter.param3 = -60.0;
        let drawn = routes(Some(HOME), &[items[0], loiter, next]);
        let other = get_bearing(centre, drawn[1].points[2]);
        let wanted = (get_bearing(centre, at(&next)) + theta) % 360.0;
        assert!((other - wanted).abs() < 0.5, "{other} against {wanted}");
        // Without Param4, the loiter's centre alone.
        loiter.param4 = 0.0;
        let drawn = routes(Some(HOME), &[items[0], loiter, next]);
        assert_eq!(drawn[1].points.len(), 3);
    }

    /// Spline waypoints are curved: the wp route holds the samples between them, each segment
    /// starting from its origin and ending on the spline point itself, then the straight on.
    #[test]
    fn spline_waypoints_are_sampled_between_the_straight_points() {
        let items = [
            row(1, 16, -35.3620, 149.1650),
            row(2, 82, -35.3610, 149.1660),
            row(3, 82, -35.3600, 149.1650),
            row(4, 16, -35.3590, 149.1660),
        ];
        let drawn = routes(Some(HOME), &items);
        let points = &drawn[1].points;
        // Home and row 1 straight, then the samples of two spline segments, then row 4.
        assert!(points.len() > 6, "{} points", points.len());
        assert_eq!(points[0], at(&items[0]));
        // The first sample is row 1 again, as the spline gives it back.
        assert!((points[1].0 - items[0].x).abs() < 1e-5);
        assert!((points[1].1 - items[0].y).abs() < 2e-5);
        assert!(points.contains(&at(&items[1])));
        assert!(points.contains(&at(&items[2])));
        assert_eq!(points.last(), Some(&at(&items[3])));
        // The samples are what the spline gives for the same points.
        let rp = |item: &MissionItem| RoutePoint {
            lat: item.x,
            lng: item.y,
            alt: 100.0 + HOME.2,
            spline: item.command == 82,
        };
        let home_point = RoutePoint {
            lat: HOME.0,
            lng: HOME.1,
            alt: HOME.2,
            spline: false,
        };
        let sampled = sample_splines(
            &[rp(&items[0]), rp(&items[1]), rp(&items[2]), rp(&items[3])],
            home_point,
            rp(&items[0]),
            home(),
            SplineRates::default(),
        );
        assert_eq!(&points[1..points.len() - 1], sampled.as_slice());
        // No two steps much more than the speed limit's 6 m apart - plus the grain of the C#'s
        // float coordinates, about 1.5 m of longitude at this latitude (a 7.5 m step was seen).
        for pair in points[1..points.len() - 1].windows(2) {
            assert!(get_distance(pair[0], pair[1]) < 9.0, "{pair:?}");
        }
    }

    /// Without home the first point stands in for it: the legs run to row 1, and the curve is
    /// worked out about 0,0.
    #[test]
    fn without_home_the_first_point_is_home() {
        let items = [
            row(1, 16, -35.3620, 149.1650),
            row(2, 16, -35.3610, 149.1660),
            row(3, 16, -35.3620, 149.1670),
        ];
        let drawn = routes(None, &items);
        assert_eq!(
            drawn[0].points,
            vec![at(&items[2]), at(&items[0]), at(&items[1])]
        );
        assert_eq!(drawn[1].points, vec![at(&items[1]), at(&items[2])]);
    }

    /// The flight screen's list starts with home, item 0, which is not a row.
    #[test]
    fn item_zero_is_home_and_not_a_row() {
        let items = [
            row(0, 16, -35.0, 149.0),
            row(1, 16, -35.3620, 149.1650),
            row(2, 16, -35.3610, 149.1660),
        ];
        let drawn = routes(Some(HOME), &items);
        assert_eq!(drawn[1].points, vec![at(&items[1]), at(&items[2])]);
    }

    /// `GetBearing` and `GetDistance` as the C# has them.
    #[test]
    fn bearing_and_distance_are_the_c_sharps() {
        let a = (-35.0, 149.0);
        assert!((get_bearing(a, (-34.0, 149.0)) - 0.0).abs() < 1e-9);
        assert!((get_bearing(a, (-36.0, 149.0)) - 180.0).abs() < 1e-9);
        // East along a southern parallel the great circle sets off a little south of east.
        assert!((get_bearing(a, (-35.0, 150.0)) - 90.287).abs() < 0.01);
        // One degree of latitude on a 6371 km sphere.
        assert!((get_distance(a, (-34.0, 149.0)) - 111_194.93).abs() < 0.01);
    }

    /// The fence: vertices into one polygon until its count is reached, coloured by the vertex
    /// that completed it; circles of their radius; a pin on each item.
    #[test]
    fn the_fence_is_drawn_as_create_overlay_draws_it() {
        let vertex = |command, count: f32, lat, lng| FenceVertex {
            command,
            param1: count,
            lat,
            lng,
        };
        let items = [
            vertex(5000, 0.0, -35.36, 149.16),
            vertex(5001, 3.0, -35.361, 149.161),
            vertex(5001, 3.0, -35.362, 149.161),
            vertex(5001, 3.0, -35.362, 149.162),
            vertex(5002, 3.0, -35.3615, 149.1612),
            vertex(5002, 3.0, -35.3618, 149.1612),
            vertex(5002, 3.0, -35.3618, 149.1616),
            vertex(5003, 120.0, -35.37, 149.17),
            vertex(5004, 30.5, -35.371, 149.171),
            // An inclusion vertex left open, then an exclusion that completes it: one red polygon.
            vertex(5001, 2.0, -35.38, 149.18),
            vertex(5002, 2.0, -35.381, 149.18),
            // Never complete: never drawn.
            vertex(5001, 4.0, -35.39, 149.19),
        ];
        let overlay = fence_overlay(&items);
        assert_eq!(overlay.polygons.len(), 3);
        assert!(overlay.polygons[0].inclusion);
        assert_eq!(overlay.polygons[0].points.len(), 3);
        assert!(!overlay.polygons[1].inclusion);
        assert!(!overlay.polygons[2].inclusion);
        assert_eq!(
            overlay.polygons[2].points,
            vec![(-35.38, 149.18), (-35.381, 149.18)]
        );
        assert_eq!(
            overlay.circles,
            vec![
                FenceCircle {
                    centre: (-35.37, 149.17),
                    radius: 120.0,
                    inclusion: true,
                },
                FenceCircle {
                    centre: (-35.371, 149.171),
                    radius: 30.5,
                    inclusion: false,
                },
            ]
        );
        assert_eq!(overlay.pins.len(), items.len());
    }

    /// The commands `CreateOverlay` counts as navigable.
    #[test]
    fn navigable_is_create_overlays_test() {
        for command in [16, 17, 18, 19, 21, 22, 31, 82, 85, 189, 188, 201] {
            assert!(navigable(command), "{command}");
        }
        for command in [0_u16, 20, 30, 92, 93, 95, 177, 183, 5000, 5100] {
            assert!(!navigable(command) || command == 0, "{command}");
        }
    }
}
