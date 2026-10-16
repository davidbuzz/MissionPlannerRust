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

//! What the map draws of `WPOverlay` besides its markers' rects, and what the planner's map
//! draws over it: the routes `RegenerateWPRoute` makes (`mp_mission::wp_overlay`) in their yellow
//! pens, the custom arrows `GMapRoute` puts on the mission's legs; each item's pin - the green
//! `GMapMarkerWP` with its number and, selected, its red disc, or DO_SET_ROI's red `GMarkerGoogle`
//! with its number always shown; the vehicle's fence as the "fence" `WPOverlay` draws it, its
//! polygons, circles and blue-dot pins; the rally points' "Rally Point / Alt" tooltip; and
//! `SelectedArea`, the rubber band the planner's Ctrl box-select and GMap's Alt selection draw.
//!
//! Ported from `ExtLibs/Maps/WPOverlay.cs`, `ExtLibs/Maps/GMapMarkerWP.cs`,
//! `ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapRoute.cs` (`OnRender`,
//! `UpdateGraphicsPath`), `GMapPolygon.cs` (`IsInside`, `OnRender`) and
//! `ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs` (the selected area's
//! drawing), GPL-3.0-only.

use gpui::{
    App, Bounds, Hsla, PathBuilder, Pixels, Point, SharedString, TextAlign, TextRun, Window, point,
    px, rgb,
};
use mp_mission::MissionItem;
use mp_mission::wp_overlay::{self, FenceVertex, RoutePen, SplineRates};
use mp_units::{LatLon, WebMercator};

use super::{MapViewport, MarkerTag, PIN_AREA, PIN_GREEN, PIN_LABEL, PIN_LABEL_SIZE, PIN_RED};

/// `Color.Yellow`: both routes' pen.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:576, 587`
pub const ROUTE_YELLOW: u32 = 0xff_ff_00;
/// The "wp route"'s pen width, `new Pen(Color.Yellow, 4)`.
pub const MISSION_ROUTE_WIDTH: f32 = 4.0;
/// The "home route"'s pen width, `new Pen(Color.Yellow, 2)`.
pub const HOME_ROUTE_WIDTH: f32 = 2.0;
/// `UpdateGraphicsPath`: a route with a point further than this from the map's corner, in pixels,
/// has its pen made solid - no dashes, and no arrows.
/// `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapRoute.cs:128-135`
pub const SOLID_BEYOND: f32 = 99_000.0;
/// The custom arrows: a leg longer than this many pixels gets one.
/// `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapRoute.cs:201`
pub const ARROW_MIN_LEG: f64 = 20.0;
/// Each arm of an arrow, pixels.
pub const ARROW_LENGTH: f32 = 15.0;
/// Each arm's turn from the leg, degrees.
pub const ARROW_TURN: f64 = 210.0;
/// `GMarkerGoogleType.blue_dot`'s fill, the blue most of `blue-dot.png`'s pixels are.
/// `// C#: ExtLibs/GMap.NET.Drawing/Resources/blue-dot.png`
pub const PIN_BLUE_DOT: u32 = 0x57_81_fc;
/// The dot in the blue-dot pin's head: five pixels across, `0x040404`.
const PIN_DOT: (f32, u32) = (5.0, 0x04_04_04);
/// `Color.Pink`: the geofence polygon's pen, `geofencepolygon.Stroke = new Pen(Color.Pink, 5)`, and
/// `Pens.Pink`, an inclusion polygon's in the "fence" overlay.
/// `// C#: GCSViews/FlightPlanner.cs:272-274; ExtLibs/Maps/WPOverlay.cs:302-303`
pub const FENCE_PINK: u32 = 0xff_c0_cb;
/// `geofencepolygon`'s pen width.
pub const GEOFENCE_WIDTH: f32 = 5.0;
/// `Color.Red`, `Pens.Red`: an exclusion's pen.
pub const FENCE_RED: u32 = 0xff_00_00;
/// `Color.Blue`: an inclusion circle's pen.
pub const FENCE_BLUE: u32 = 0x00_00_ff;
/// `Color.FromArgb(30, 255, 0, 0)`: an exclusion's fill, as RGBA.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:317, 327`
pub const EXCLUSION_FILL: u32 = 0xff_00_00_1e;
/// `SelectionPen`, `new Pen(Brushes.Blue, 2)`: the selected area's outline.
/// `// C#: ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs:166`
pub const SELECTION_BLUE: u32 = 0x00_00_ff;
/// `MainMap.SelectedAreaFillColor = Color.FromArgb(33, 65, 105, 225)`, as RGBA.
/// `// C#: GCSViews/FlightPlanner.Designer.cs:892`
pub const SELECTED_AREA_FILL: u32 = 0x41_69_e1_21;
/// `Brushes.Red`, `Pens.Red`: a selected waypoint's disc.
/// `// C#: ExtLibs/Maps/GMapMarkerWP.cs:44-48`
pub const SELECTED_RED: u32 = 0xff_00_00;
/// `GMapMarkerRallyPt`'s area from its point: `marker_02.png`, 24 x 45, at `Offset = (-10, -40)`.
/// `// C#: ExtLibs/Maps/GMapMarkerRallyPt.cs:17-30`
pub const RALLY_AREA: (f32, f32, f32, f32) = (-10.0, -40.0, 24.0, 45.0);
/// `GMapPolygon.OnRender`'s "max graphics size": a polygon whose first point is further than this
/// from the map's corner is not drawn.
/// `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapPolygon.cs:186-191`
const POLYGON_MAX: f32 = 65_535.0;
/// Points per stroked path, as the flown track has it: gpui holds a path to 65,535 vertices.
const ROUTE_CHUNK: usize = 8_000;

/// `DashStyle.Dash` for a pen `width` wide: three widths on, one off.
#[must_use]
pub fn dash_pattern(width: f32) -> [f32; 2] {
    [3.0 * width, width]
}

/// One of `GMapRoute`'s custom arrows: two arms from a leg's middle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Arrow {
    /// The leg's middle, where both arms start.
    pub mid: (f32, f32),
    /// The end of the arm turned 210 degrees one way.
    pub left: (f32, f32),
    /// The end of the arm turned 210 degrees the other.
    pub right: (f32, f32),
}

/// `GMapRoute.OnRender`'s custom arrows over a route's points in the map's pixels: on each leg
/// longer than 20 pixels, two 15-pixel arms from its middle at 210 degrees either side of it.
/// A leg of 20 or less is skipped and the point before it kept, so the next leg is measured from
/// there; a point at the map's very corner, `PointF.Empty`, counts as none.
/// `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapRoute.cs:179-227`
#[must_use]
pub fn route_arrows(points: &[(f32, f32)]) -> Vec<Arrow> {
    let deg2rad = std::f64::consts::PI / 180.0;
    let mut arrows = Vec::new();
    let mut last: Option<(f32, f32)> = None;
    for &item in points {
        let Some(from) = last.filter(|last| *last != (0.0, 0.0)) else {
            last = Some(item);
            continue;
        };
        let polx = item.0 - from.0;
        let poly = item.1 - from.1;
        // distance
        let r = (f64::from(polx).powi(2) + f64::from(poly).powi(2)).sqrt();
        if r <= ARROW_MIN_LEG {
            continue;
        }
        // angle
        let angle = f64::from(poly).atan2(f64::from(polx));
        if angle.is_nan() {
            continue;
        }
        let mid = (from.0 + polx / 2.0, from.1 + poly / 2.0);
        let arm = |turn: f64| {
            #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)Math.Cos(...)`
            let (cos, sin) = (turn.cos() as f32, turn.sin() as f32);
            (mid.0 + ARROW_LENGTH * cos, mid.1 + ARROW_LENGTH * sin)
        };
        arrows.push(Arrow {
            mid,
            left: arm(angle + ARROW_TURN * deg2rad),
            right: arm(angle - ARROW_TURN * deg2rad),
        });
        last = Some(item);
    }
    arrows
}

/// Whether `UpdateGraphicsPath` makes a route's pen solid: one of its points, in the map's
/// pixels, is further than 99,000 from the corner. (The C# changes the route's own pen, so it
/// stays solid until `writeKML` makes the route again; here it is solid while it is that far.)
#[must_use]
pub fn drawn_solid(points: &[(f32, f32)]) -> bool {
    points
        .iter()
        .any(|(x, y)| x.abs() > SOLID_BEYOND || y.abs() > SOLID_BEYOND)
}

/// `GMapMarkerWP.selected`'s disc about a marker's point: `FillEllipse` in the marker's own
/// rectangle, `LocalPosition` and the 32 x 32 `Size` - so its centre, and its width.
/// `// C#: ExtLibs/Maps/GMapMarkerWP.cs:44-48`
#[must_use]
pub fn selected_disc(at: (f32, f32)) -> ((f32, f32), f32) {
    let (left, top, width, height) = PIN_AREA;
    (
        (at.0 + left + width / 2.0, at.1 + top + height / 2.0),
        width,
    )
}

/// `GMapPolygon.IsInside`: the crossings of a ray east from `p`, latitude first.
/// `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapPolygon.cs:276-302`
#[must_use]
pub fn is_inside(polygon: &[(f64, f64)], p: (f64, f64)) -> bool {
    let count = polygon.len();
    if count < 3 {
        return false;
    }
    let mut result = false;
    let mut j = count - 1;
    for (i, p1) in polygon.iter().enumerate() {
        let Some(p2) = polygon.get(j) else {
            break;
        };
        if ((p1.0 < p.0 && p2.0 >= p.0) || (p2.0 < p.0 && p1.0 >= p.0))
            && p1.1 + (p.0 - p1.0) / (p2.0 - p1.0) * (p2.1 - p1.1) < p.1
        {
            result = !result;
        }
        j = i;
    }
    result
}

/// The planner's group select on a Ctrl release: the polygon `MouseDownStart`, (start's
/// latitude, end's longitude), `MouseDownEnd`, (end's latitude, start's longitude), and the rows
/// of the `WPOverlay` markers inside it, in the overlay's order - home's "H", which
/// `int.Parse` refuses, is never one.
/// `// C#: GCSViews/FlightPlanner.cs:7700-7727, 7362-7376`
#[must_use]
pub fn box_select(start: LatLon, end: LatLon, markers: &[(u16, LatLon)]) -> Vec<u16> {
    let (start, end) = (
        (start.latitude(), start.longitude()),
        (end.latitude(), end.longitude()),
    );
    let polygon = [start, (start.0, end.1), end, (end.0, start.1)];
    let mut rows = Vec::new();
    for (row, at) in markers {
        if is_inside(&polygon, (at.latitude(), at.longitude())) && !rows.contains(row) {
            rows.push(*row);
        }
    }
    rows
}

/// A `RectLatLng` between two points, as both the planner's Ctrl box and GMap's selection make
/// it: its top-left the greater latitude and the lesser longitude, its bottom-right the others.
/// `// C#: GCSViews/FlightPlanner.cs:7518-7524;
/// ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs:2119-2130`
#[must_use]
pub fn selection_rect(a: LatLon, b: LatLon) -> Option<(LatLon, LatLon)> {
    let top_left = LatLon::new(
        a.latitude().max(b.latitude()),
        a.longitude().min(b.longitude()),
    )
    .ok()?;
    let bottom_right = LatLon::new(
        a.latitude().min(b.latitude()),
        a.longitude().max(b.longitude()),
    )
    .ok()?;
    Some((top_left, bottom_right))
}

/// One route, projected.
#[derive(Debug, Clone, PartialEq)]
struct ProjectedRoute {
    points: Vec<WebMercator>,
    pen: RoutePen,
}

/// The "fence" overlay, projected: polygons with whether each is an inclusion, circles with their
/// radius and kind, and the pins.
#[derive(Debug, Clone, Default, PartialEq)]
struct ProjectedFence {
    polygons: Vec<(Vec<WebMercator>, bool)>,
    circles: Vec<(WebMercator, f64, bool)>,
    pins: Vec<WebMercator>,
}

/// What the last paint drew, for the facts.
#[derive(Debug, Clone, Default, PartialEq)]
struct Drawn {
    arrows: usize,
    green: usize,
    red: usize,
    blue_dot: usize,
    discs: usize,
    fence_circles: usize,
    fence_polygons: usize,
}
/// Home with its altitude (latitude, longitude, altitude) and the loiter radius a build of the
/// routes was made for.
type BuiltFor = (Option<(f64, f64, f64)>, Option<f64>);

/// What the map holds of the lines and the selection this module draws.
#[derive(Debug, Clone, Default)]
pub struct WpLayer {
    /// The mission as last set, which the routes are built from.
    items: Vec<MissionItem>,
    /// Home, with its altitude, and the loiter radius the routes were built with; `None` until
    /// they are built, and after the items change.
    built_for: Option<BuiltFor>,
    /// `RegenerateWPRoute`'s routes, in the order they join the overlay.
    routes: Vec<ProjectedRoute>,
    /// `MAV.fencepoints` as last handed over.
    fence_items: Vec<FenceVertex>,
    /// What `CreateOverlay` draws of them.
    fence: ProjectedFence,
    /// `groupmarkers`: the rows whose `GMapMarkerWP` is `selected`.
    selected: Vec<u16>,
    /// `MainMap.SelectedArea`: top-left and bottom-right.
    selected_area: Option<(LatLon, LatLon)>,
    /// The rally markers under the pointer: where each is, and its tooltip.
    rally_tooltips: Vec<(LatLon, String)>,
    /// The group drag under way: the rows whose markers move, and by how much, latitude and
    /// longitude.
    group_drag: Option<(Vec<u16>, (f64, f64))>,
    drawn: Drawn,
}

/// The ground `GetHomeAlt` puts a terrain-frame row above: the process's `srtm` lookup.
#[cfg(not(test))]
fn route_terrain(lat: f64, lng: f64) -> crate::srtm::AltResponse {
    crate::srtm::altitude(lat, lng)
}

/// Under `cargo test` no terrain, as `plan::Terrain` has it, so no test reads this machine's
/// `srtm` folder.
#[cfg(test)]
const fn route_terrain(_lat: f64, _lng: f64) -> crate::srtm::AltResponse {
    crate::srtm::AltResponse::INVALID
}

fn project(lat: f64, lng: f64) -> Option<WebMercator> {
    LatLon::new(lat, lng).ok().map(LatLon::to_web_mercator)
}

impl MapViewport {
    /// The mission the routes are built from, as `set_mission` is given it.
    pub(super) fn set_route_items(&mut self, items: &[MissionItem]) {
        if self.wp.items.as_slice() != items {
            self.wp.items = items.to_vec();
            self.wp.built_for = None;
        }
        self.refresh_routes();
    }

    /// Builds the routes again when the mission, home, home's altitude or the overlay's loiter
    /// radius has changed: `writeKML`'s `CreateOverlay`, home's altitude `GetHomeAlt`'s for a
    /// relative row (0 without a home, `new PointLatLngAlt()`'s), and none on a map drawing no
    /// `WPOverlay`.
    pub(super) fn refresh_routes(&mut self) {
        let home = self.home_position.map(|home| {
            (
                home.latitude(),
                home.longitude(),
                self.home_altitude.unwrap_or(0.0),
            )
        });
        let loiter_radius = self.overlay_mode.map(|overlay| overlay.loiter_radius);
        let key = (home, loiter_radius);
        if self.wp.built_for == Some(key) {
            return;
        }
        self.wp.built_for = Some(key);
        let Some(loiter_radius) = loiter_radius else {
            self.wp.routes.clear();
            return;
        };
        let homealt = home.map_or(0.0, |(_, _, alt)| alt);
        let home_alt = |frame: u8, lat: f64, lng: f64| {
            super::get_home_alt(frame, homealt, lat, lng, &route_terrain)
        };
        self.wp.routes = wp_overlay::mission_routes(
            home,
            &self.wp.items,
            loiter_radius,
            SplineRates::default(),
            &home_alt,
        )
        .into_iter()
        .map(|route| ProjectedRoute {
            points: route
                .points
                .iter()
                .filter_map(|(lat, lng)| project(*lat, *lng))
                .collect(),
            pen: route.pen,
        })
        .collect();
    }

    /// `MAV.fencepoints`, the shown vehicle's fence as the link has seen it, which both screens
    /// draw as a "fence" `WPOverlay` (`CreateOverlay(PointLatLngAlt.Zero, fencepoints, 0, 0)`).
    /// `// C#: GCSViews/FlightPlanner.cs:1497-1517; GCSViews/FlightData.cs:4021-4035`
    pub fn set_vehicle_fence(&mut self, items: &[mp_vehicle::FenceItem]) {
        let items: Vec<FenceVertex> = items
            .iter()
            .map(|item| FenceVertex {
                command: item.command,
                param1: item.param1,
                lat: f64::from(item.x) / 1e7,
                lng: f64::from(item.y) / 1e7,
            })
            .collect();
        if items == self.wp.fence_items {
            return;
        }
        let overlay = wp_overlay::fence_overlay(&items);
        self.wp.fence = ProjectedFence {
            polygons: overlay
                .polygons
                .iter()
                .map(|polygon| {
                    (
                        polygon
                            .points
                            .iter()
                            .filter_map(|(lat, lng)| project(*lat, *lng))
                            .collect(),
                        polygon.inclusion,
                    )
                })
                .collect(),
            circles: overlay
                .circles
                .iter()
                .filter_map(|circle| {
                    let (lat, lng) = circle.centre;
                    project(lat, lng).map(|at| (at, circle.radius, circle.inclusion))
                })
                .collect(),
            pins: overlay
                .pins
                .iter()
                .filter_map(|(lat, lng)| project(*lat, *lng))
                .collect(),
        };
        self.wp.fence_items = items;
    }

    /// `groupmarkers` on the map: the rows whose markers draw their red disc. A mission set
    /// with different markers - `writeKML`'s new ones - draws none until they are selected again.
    pub fn set_selected_rows(&mut self, rows: &[u16]) {
        rows.clone_into(&mut self.wp.selected);
    }

    /// `MainMap.SelectedArea`, or `RectLatLng.Empty`.
    pub fn set_selected_area(&mut self, area: Option<(LatLon, LatLon)>) {
        self.wp.selected_area = area;
    }

    /// The planner's group drag: each marker of `rows` drawn moved by `delta` degrees of latitude
    /// and longitude - `temp.Offset(latdif, -lngdif)` on every move - until the drop writes the
    /// rows and `writeKML` makes the markers again. `None` ends it.
    /// `// C#: GCSViews/FlightPlanner.cs:7444-7473`
    pub fn set_group_drag(&mut self, drag: Option<(Vec<u16>, (f64, f64))>) {
        self.wp.group_drag = drag;
    }

    /// The rally markers the pointer is over, each with its tooltip. Returns whether that is
    /// different from before.
    pub fn set_rally_tooltips(&mut self, tooltips: Vec<(LatLon, String)>) -> bool {
        let changed = tooltips != self.wp.rally_tooltips;
        self.wp.rally_tooltips = tooltips;
        changed
    }

    /// The rows of `WPOverlay`'s markers and where each is, in the overlay's order: what the
    /// planner's group select looks inside its box for. None on a map drawing no `WPOverlay`.
    #[must_use]
    pub fn overlay_rows(&self) -> Vec<(u16, LatLon)> {
        if self.overlay_mode.is_none() {
            return Vec::new();
        }
        self.overlay
            .iter()
            .filter_map(|marker| match marker.tag {
                MarkerTag::Item(row) => Some((row, marker.position)),
                _ => None,
            })
            .collect()
    }

    /// GMap's `IsMouseOverMarker`: whether the pointer, at its last move, was over a marker's
    /// area - which a left press there does not start a selection from.
    #[must_use]
    pub fn is_over_marker(&self) -> bool {
        !self.hovered.rects.is_empty() || !self.hovered.pins.is_empty()
    }

    /// Facts for what this module draws: the routes, each `home:dashed`, `home:solid` or
    /// `mission:<points>`, the mission's legs and arrows; the pins by colour and the selected
    /// discs; the vehicle fence's polygons and circles; the selected rows and area.
    pub(super) fn wp_facts(&self) -> Vec<(&'static str, String)> {
        let none = || "none".to_owned();
        let routes: Vec<String> = self
            .wp
            .routes
            .iter()
            .map(|route| match route.pen {
                RoutePen::Home { dashed: true } => "home:dashed".to_owned(),
                RoutePen::Home { dashed: false } => "home:solid".to_owned(),
                RoutePen::Mission => format!("mission:{}", route.points.len()),
            })
            .collect();
        let segments: usize = self
            .wp
            .routes
            .iter()
            .filter(|route| route.pen == RoutePen::Mission)
            .map(|route| route.points.len().saturating_sub(1))
            .sum();
        let home = self
            .wp
            .routes
            .iter()
            .find_map(|route| match route.pen {
                RoutePen::Home { dashed: true } => Some("dashed"),
                RoutePen::Home { dashed: false } => Some("solid"),
                RoutePen::Mission => None,
            })
            .unwrap_or("none");
        let rows = |rows: &[u16]| {
            if rows.is_empty() {
                none()
            } else {
                rows.iter()
                    .map(u16::to_string)
                    .collect::<Vec<_>>()
                    .join(",")
            }
        };
        let drawn = &self.wp.drawn;
        vec![
            (
                "map.routes",
                if routes.is_empty() {
                    none()
                } else {
                    routes.join(",")
                },
            ),
            ("map.route.segments", segments.to_string()),
            ("map.route.home", home.to_owned()),
            ("map.route.arrows", drawn.arrows.to_string()),
            ("map.pins.green", drawn.green.to_string()),
            ("map.pins.red", drawn.red.to_string()),
            ("map.pins.blue_dot", drawn.blue_dot.to_string()),
            ("map.selected", rows(&self.wp.selected)),
            ("map.selected.discs", drawn.discs.to_string()),
            (
                "map.selected_area",
                self.wp.selected_area.map_or_else(none, |(a, b)| {
                    format!(
                        "{:.7},{:.7};{:.7},{:.7}",
                        a.latitude(),
                        a.longitude(),
                        b.latitude(),
                        b.longitude()
                    )
                }),
            ),
            ("map.fence.polygons", drawn.fence_polygons.to_string()),
            ("map.fence.circles", drawn.fence_circles.to_string()),
        ]
    }
}

#[cfg(test)]
impl MapViewport {
    /// The view a paint of a `width` by `height` map at the window's corner leaves, `span` of the
    /// world across about `centre`: for a test of what turns window points into places.
    pub fn paint_for_test(&mut self, centre: LatLon, span: f64, (width, height): (f32, f32)) {
        let centre = centre.to_web_mercator();
        self.last_viewport = (width, height);
        self.last_origin = (0.0, 0.0);
        let down = span * f64::from(height) / f64::from(width);
        self.last_view = Some((centre.x - span / 2.0, centre.y - down / 2.0, span, down));
        self.camera = Some(super::Camera { centre, span });
    }
}

/// A stroked polyline, in chunks a path can hold, dashed or not. Returns whether every chunk
/// tessellated.
fn stroke_route(
    window: &mut Window,
    points: &[Point<Pixels>],
    width: f32,
    colour: u32,
    dash: Option<[f32; 2]>,
) -> bool {
    let mut drawn = true;
    let mut start = 0;
    while start + 1 < points.len() {
        let end = (start + ROUTE_CHUNK).min(points.len());
        let Some(chunk) = points.get(start..end) else {
            break;
        };
        let mut builder = PathBuilder::stroke(px(width));
        if let Some([on, off]) = dash {
            builder = builder.dash_array(&[px(on), px(off)]);
        }
        let mut chunk = chunk.iter();
        if let Some(first) = chunk.next() {
            builder.move_to(*first);
        }
        for point in chunk {
            builder.line_to(*point);
        }
        match builder.build() {
            Ok(path) => window.paint_path(path, Hsla::from(rgb(colour))),
            Err(_) => drawn = false,
        }
        // One point of overlap, so the chunks join.
        start = end - 1;
    }
    drawn
}

/// The routes, each in its pen: the home route two pixels, dashed while home is near, the
/// mission's four, solid, with an arrow on each leg; a route reaching past 99,000 pixels solid
/// and without arrows. `origin` is the map's corner, which `GMapRoute`'s local points are from.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:576-589; ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapRoute.cs:120-246`
pub(super) fn paint_routes(
    map: &mut MapViewport,
    origin: Point<Pixels>,
    to_screen: &dyn Fn(WebMercator) -> Point<Pixels>,
    window: &mut Window,
) {
    map.wp.drawn.arrows = 0;
    if map.overlay_mode.is_none() {
        return;
    }
    let mut failures = 0;
    for route in &map.wp.routes {
        let screen: Vec<Point<Pixels>> = route.points.iter().map(|p| to_screen(*p)).collect();
        // `LocalPoints`, whole pixels from the map's corner.
        let local: Vec<(f32, f32)> = screen
            .iter()
            .map(|p| {
                (
                    f32::from(p.x - origin.x).round(),
                    f32::from(p.y - origin.y).round(),
                )
            })
            .collect();
        let solid = drawn_solid(&local);
        let ok = match route.pen {
            RoutePen::Home { dashed } => {
                let dash = (dashed && !solid).then(|| dash_pattern(HOME_ROUTE_WIDTH));
                stroke_route(window, &screen, HOME_ROUTE_WIDTH, ROUTE_YELLOW, dash)
            }
            RoutePen::Mission => {
                let ok = stroke_route(window, &screen, MISSION_ROUTE_WIDTH, ROUTE_YELLOW, None);
                if !solid {
                    let at = |(x, y): (f32, f32)| point(origin.x + px(x), origin.y + px(y));
                    for arrow in route_arrows(&local) {
                        for arm in [arrow.left, arrow.right] {
                            super::stroke_through(
                                window,
                                &[at(arrow.mid), at(arm)],
                                MISSION_ROUTE_WIDTH,
                                ROUTE_YELLOW,
                            );
                        }
                        map.wp.drawn.arrows += 1;
                    }
                }
                ok
            }
        };
        if !ok {
            failures += 1;
        }
    }
    map.track_path_failures += failures;
}

/// Paints `GMapMarkerWP` at `at`: the green pin, and its number where it is written - at zoom 17
/// and in, or under the pointer - from `LocalPosition + (10, 3)`, four pixels further left when
/// `MeasureString` makes it wider than 15 (which pads a sixth of the font's size each side).
/// `// C#: ExtLibs/Maps/GMapMarkerWP.cs:20-60`
fn paint_wp_pin(window: &mut Window, cx: &mut App, at: Point<Pixels>, label: Option<String>) {
    super::paint_pin(window, cx, at, PIN_GREEN, None);
    let Some(text) = label else {
        return;
    };
    let run = TextRun {
        len: text.len(),
        font: window.text_style().font(),
        color: Hsla::from(rgb(0x00_00_00)),
        background_color: None,
        underline: None,
        strikethrough: None,
    };
    let line =
        window
            .text_system()
            .shape_line(SharedString::from(text), px(PIN_LABEL_SIZE), &[run], None);
    let measured = f32::from(line.width) + PIN_LABEL_SIZE / 3.0;
    let left = if measured > 15.0 {
        PIN_LABEL.0 - 4.0
    } else {
        PIN_LABEL.0
    };
    let _ = line.paint(
        point(at.x + px(left), at.y + px(PIN_LABEL.1)),
        px(PIN_LABEL_SIZE * 1.2),
        TextAlign::Left,
        None,
        window,
        cx,
    );
}

/// A circle's outline as so many points, for a one-pixel arc.
fn circle_outline(centre: Point<Pixels>, radius: f32) -> Vec<Point<Pixels>> {
    (0..=48_u16)
        .map(|step| {
            let angle = f32::from(step) / 48.0 * std::f32::consts::TAU;
            point(
                centre.x + px(radius * angle.cos()),
                centre.y + px(radius * angle.sin()),
            )
        })
        .collect()
}

/// `WPOverlay`'s item markers: each DO_SET_ROI's red `GMarkerGoogle`, and every other item's
/// `GMapMarkerWP` - its red disc first where `groupmarkers` selected it, then the green pin and its
/// number. `zoom` is the map's.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:112-140, 385-415; ExtLibs/Maps/GMapMarkerWP.cs:42-60`
pub(super) fn paint_item_pins(
    map: &mut MapViewport,
    to_screen: &dyn Fn(WebMercator) -> Point<Pixels>,
    zoom: f64,
    window: &mut Window,
    cx: &mut App,
) {
    map.wp.drawn.green = 0;
    map.wp.drawn.red = 0;
    map.wp.drawn.discs = 0;
    if map.overlay_mode.is_none() {
        return;
    }
    for marker in &map.overlay {
        let MarkerTag::Item(row) = marker.tag else {
            continue;
        };
        // A marker the group drag is moving is drawn where it has been moved to.
        let world = map
            .wp
            .group_drag
            .as_ref()
            .filter(|(rows, _)| rows.contains(&row))
            .and_then(|(_, (lat, lng))| {
                project(
                    marker.position.latitude() + lat,
                    marker.position.longitude() + lng,
                )
            })
            .unwrap_or(marker.at);
        let at = to_screen(world);
        // DO_SET_ROI: `GMarkerGoogleType.red`, no "Alt:".
        if marker.alt.is_none() {
            super::paint_pin(window, cx, at, PIN_RED, None);
            map.wp.drawn.red += 1;
            continue;
        }
        if map.wp.selected.contains(&row) {
            let ((x, y), width) = selected_disc((f32::from(at.x), f32::from(at.y)));
            let centre = point(px(x), px(y));
            super::paint_disc(window, centre, width, (SELECTED_RED << 8) | 0xff);
            super::stroke_through(
                window,
                &circle_outline(centre, width / 2.0),
                1.0,
                SELECTED_RED,
            );
            map.wp.drawn.discs += 1;
        }
        let shown =
            super::pin_label_shown(zoom) || map.hovered.pins.contains(&MarkerTag::Item(row));
        paint_wp_pin(window, cx, at, shown.then(|| row.to_string()));
        map.wp.drawn.green += 1;
    }
}

/// Paints a `blue_dot` pin: the pin in its blue, and the dark dot in its head.
fn paint_blue_dot(window: &mut Window, cx: &mut App, at: Point<Pixels>) {
    super::paint_pin(window, cx, at, PIN_BLUE_DOT, None);
    let (width, colour) = PIN_DOT;
    let (x, y) = super::PIN_HEAD;
    super::paint_disc(
        window,
        point(at.x + px(x), at.y + px(y)),
        width,
        (colour << 8) | 0xff,
    );
}

/// The "fence" `WPOverlay` of the vehicle's fence: each closed polygon filled and outlined in its
/// kind's pen, one pixel; each vertex, circle and return point's blue-dot pin; each circle's
/// `GMapMarkerRect` at the map's scale, dashed two pixels in blue or red, an exclusion's then
/// filled. A polygon whose first point is past 65,535 pixels is not drawn, as `GMapPolygon` has
/// it.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:292-340; ExtLibs/Maps/GMapMarkerRect.cs:54-98;
/// ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapPolygon.cs:174-200`
pub(super) fn paint_fence_layer(
    map: &mut MapViewport,
    bounds: Bounds<Pixels>,
    to_screen: &dyn Fn(WebMercator) -> Point<Pixels>,
    window: &mut Window,
    cx: &mut App,
) {
    map.wp.drawn.fence_circles = 0;
    map.wp.drawn.fence_polygons = 0;
    map.wp.drawn.blue_dot = 0;
    if map.overlay_mode.is_none() {
        return;
    }
    let origin = bounds.origin;
    for (points, inclusion) in &map.wp.fence.polygons {
        let screen: Vec<Point<Pixels>> = points.iter().map(|p| to_screen(*p)).collect();
        let Some(first) = screen.first() else {
            continue;
        };
        if screen.len() < 2
            || f32::from(first.x - origin.x).abs() > POLYGON_MAX
            || f32::from(first.y - origin.y).abs() > POLYGON_MAX
        {
            continue;
        }
        let colour = if *inclusion {
            FENCE_PINK
        } else {
            super::fill_through(window, &screen, Hsla::from(gpui::rgba(EXCLUSION_FILL)));
            FENCE_RED
        };
        let mut closed = screen.clone();
        closed.push(*first);
        super::stroke_through(window, &closed, 1.0, colour);
        map.wp.drawn.fence_polygons += 1;
    }
    for pin in &map.wp.fence.pins {
        paint_blue_dot(window, cx, to_screen(*pin));
        map.wp.drawn.blue_dot += 1;
    }
    let Some((m2pixelwidth, m2pixelheight)) = map.metres_to_pixels() else {
        return;
    };
    if !(m2pixelheight > 0.001 && m2pixelheight.is_finite() && m2pixelheight < f64::from(i32::MAX))
    {
        return;
    }
    let clip = (
        f32::from(bounds.origin.x) - 2.0,
        f32::from(bounds.origin.y) - 2.0,
        f32::from(bounds.origin.x + bounds.size.width) + 2.0,
        f32::from(bounds.origin.y + bounds.size.height) + 2.0,
    );
    for (at, radius, inclusion) in &map.wp.fence.circles {
        if *radius == 0.0 {
            continue;
        }
        let centre = to_screen(*at);
        // `LocalPosition` is the point in whole pixels plus the rect's offset.
        #[allow(clippy::cast_possible_truncation)] // a screen coordinate, well within range
        let local_x = f32::from(centre.x - origin.x).floor() as i64 + (super::RECT_AREA.0 as i64);
        let diameter = super::rect_diameter(local_x, m2pixelwidth, *radius);
        if diameter == 0 {
            continue;
        }
        #[allow(clippy::cast_precision_loss)] // a pixel count
        let half = diameter as f32 / 2.0;
        let colour = if *inclusion { FENCE_BLUE } else { FENCE_RED };
        let [on, off] = dash_pattern(2.0);
        for run in super::circle_runs((f32::from(centre.x), f32::from(centre.y)), half, clip) {
            let mut builder = PathBuilder::stroke(px(2.0)).dash_array(&[px(on), px(off)]);
            let mut points = run.iter();
            let Some((x, y)) = points.next() else {
                continue;
            };
            builder.move_to(point(px(*x), px(*y)));
            for (x, y) in points {
                builder.line_to(point(px(*x), px(*y)));
            }
            if let Ok(path) = builder.build() {
                window.paint_path(path, Hsla::from(rgb(colour)));
            }
        }
        // `FillPie` over the arc, an exclusion's alone.
        if !*inclusion {
            window.with_content_mask(Some(gpui::ContentMask { bounds }), |window| {
                super::paint_disc(window, centre, half * 2.0, EXCLUSION_FILL);
            });
        }
        map.wp.drawn.fence_circles += 1;
    }
}

/// The rally markers' tooltips, "Rally Point" and its altitude, for each under the pointer.
/// `// C#: ExtLibs/Maps/GMapMarkerRallyPt.cs:32-38; GCSViews/FlightPlanner.cs:952-957, 6644-6649;
/// GCSViews/FlightData.cs:4015-4019`
pub(super) fn paint_rally_tooltips(
    map: &mut MapViewport,
    window: &mut Window,
    cx: &mut App,
    to_screen: &dyn Fn(WebMercator) -> Point<Pixels>,
) {
    for (at, text) in &map.wp.rally_tooltips {
        super::paint_tooltip(window, cx, to_screen(at.to_web_mercator()), text);
        map.tooltips_drawn.push(text.clone());
    }
}

/// `SelectedArea`, after everything else: `DrawRectangle` in the selection pen, then
/// `FillRectangle` in the fill.
/// `// C#: ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs:1639-1651`
pub(super) fn paint_selected_area(
    map: &MapViewport,
    to_screen: &dyn Fn(WebMercator) -> Point<Pixels>,
    window: &mut Window,
) {
    let Some((top_left, bottom_right)) = map.wp.selected_area else {
        return;
    };
    let a = to_screen(top_left.to_web_mercator());
    let b = to_screen(bottom_right.to_web_mercator());
    let corners = [a, point(b.x, a.y), b, point(a.x, b.y), a];
    super::stroke_through(window, &corners, 2.0, SELECTION_BLUE);
    super::fill_through(
        window,
        &corners[..4],
        Hsla::from(gpui::rgba(SELECTED_AREA_FILL)),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `DashStyle.Dash`: 6 on and 2 off for the home route's 2-pixel pen.
    #[test]
    fn the_home_route_dash_is_three_widths_on_and_one_off() {
        assert_eq!(dash_pattern(HOME_ROUTE_WIDTH), [6.0, 2.0]);
        assert_eq!(dash_pattern(4.0), [12.0, 4.0]);
    }

    /// An arrow at a leg's middle, its arms 15 long, 210 degrees either side of the leg - so they
    /// point back, a chevron along the way the route goes.
    #[test]
    fn an_arrow_sits_at_the_middle_of_each_long_leg() {
        let arrows = route_arrows(&[(100.0, 100.0), (200.0, 100.0), (200.0, 300.0)]);
        assert_eq!(arrows.len(), 2);
        let first = arrows[0];
        assert_eq!(first.mid, (150.0, 100.0));
        let half = 15.0 * (210.0_f32.to_radians()).cos();
        assert!(
            (first.left.0 - (150.0 + half)).abs() < 1e-3,
            "{:?}",
            first.left
        );
        assert!((first.left.1 - 92.5).abs() < 1e-3, "{:?}", first.left);
        assert!((first.right.0 - (150.0 + half)).abs() < 1e-3);
        assert!((first.right.1 - 107.5).abs() < 1e-3);
        // Down the second leg: the arms point back up.
        assert_eq!(arrows[1].mid, (200.0, 200.0));
        assert!(arrows[1].left.1 < 200.0 && arrows[1].right.1 < 200.0);
    }

    /// A leg of 20 pixels or less has no arrow, and the next leg is measured from the point
    /// before it; a point at the very corner counts as none.
    #[test]
    fn short_legs_are_skipped_and_the_corner_is_no_point() {
        let arrows = route_arrows(&[(100.0, 100.0), (110.0, 100.0), (200.0, 100.0)]);
        assert_eq!(arrows.len(), 1);
        assert_eq!(arrows[0].mid, (150.0, 100.0));
        assert!(route_arrows(&[(100.0, 100.0), (120.0, 100.0)]).is_empty());
        let arrows = route_arrows(&[(0.0, 0.0), (100.0, 0.0), (200.0, 0.0)]);
        assert_eq!(arrows.len(), 1);
        assert_eq!(arrows[0].mid, (150.0, 0.0));
    }

    /// Past 99,000 pixels from the corner a route is drawn solid.
    #[test]
    fn a_route_far_off_the_map_is_solid() {
        assert!(!drawn_solid(&[(0.0, 0.0), (98_999.0, -98_999.0)]));
        assert!(drawn_solid(&[(0.0, 0.0), (99_001.0, 5.0)]));
        assert!(drawn_solid(&[(5.0, -99_001.0)]));
    }

    /// The disc fills the pin's own 32-pixel square, which stands on the point: its centre a
    /// pixel right of the point and fifteen up.
    #[test]
    fn the_selected_disc_fills_the_pins_square() {
        assert_eq!(selected_disc((100.0, 200.0)), ((101.0, 185.0), 32.0));
    }

    fn at(lat: f64, lng: f64) -> LatLon {
        LatLon::new(lat, lng).expect("a position")
    }

    /// `IsInside`: a point in a box, on none of its edges, and one outside.
    #[test]
    fn is_inside_counts_the_crossings() {
        let square = [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0)];
        assert!(is_inside(&square, (0.5, 0.5)));
        assert!(!is_inside(&square, (1.5, 0.5)));
        assert!(!is_inside(&square, (0.5, -0.5)));
        assert!(
            !is_inside(&square[..2], (0.5, 0.5)),
            "fewer than three corners"
        );
    }

    /// The box selects the rows whose markers are inside it, whichever way it was drawn, in the
    /// overlay's order and each once.
    #[test]
    fn the_box_selects_the_rows_inside_it() {
        let markers = [
            (1, at(-35.3620, 149.1650)),
            (2, at(-35.3610, 149.1660)),
            (3, at(-35.3500, 149.1700)),
            (2, at(-35.3610, 149.1660)),
        ];
        let rows = box_select(at(-35.3600, 149.1640), at(-35.3630, 149.1670), &markers);
        assert_eq!(rows, vec![1, 2]);
        let rows = box_select(at(-35.3630, 149.1670), at(-35.3600, 149.1640), &markers);
        assert_eq!(rows, vec![1, 2]);
        // A click, no box: nothing.
        let here = at(-35.3620, 149.1650);
        assert!(box_select(here, here, &markers).is_empty());
    }

    fn fact(map: &MapViewport, key: &str) -> String {
        map.wp_facts()
            .into_iter()
            .find(|(name, _)| *name == key)
            .map(|(_, value)| value)
            .unwrap_or_else(|| panic!("no fact {key}"))
    }

    fn waypoint(seq: u16, lat: f64, lng: f64) -> MissionItem {
        MissionItem {
            seq,
            command: 16,
            x: lat,
            y: lng,
            z: 100.0,
            ..MissionItem::default()
        }
    }

    fn planner() -> super::super::Overlay {
        super::super::Overlay {
            wp_radius: 30.0,
            loiter_radius: 45.0,
            planner: true,
        }
    }

    /// The map builds `WPOverlay`'s routes from what the screen hands it - the mission, home and
    /// its altitude, the overlay - again when any of them changes, and none with no overlay.
    #[test]
    fn the_map_builds_the_routes_from_its_mission_and_home() {
        let mut map = MapViewport::new(0, 0);
        map.set_overlay(Some(planner()));
        map.set_home(Some(at(-35.3632, 149.1652)));
        map.set_home_altitude(Some(584.0));
        map.set_mission(&[
            waypoint(1, -35.3620, 149.1650),
            waypoint(2, -35.3610, 149.1660),
            waypoint(3, -35.3620, 149.1670),
        ]);
        assert_eq!(fact(&map, "map.routes"), "home:dashed,mission:3");
        assert_eq!(fact(&map, "map.route.segments"), "2");
        assert_eq!(fact(&map, "map.route.home"), "dashed");
        // A fourth row: built again.
        map.set_mission(&[
            waypoint(1, -35.3620, 149.1650),
            waypoint(2, -35.3610, 149.1660),
            waypoint(3, -35.3620, 149.1670),
            waypoint(4, -35.3630, 149.1680),
        ]);
        assert_eq!(fact(&map, "map.routes"), "home:dashed,mission:4");
        // Home 10 km off: the legs to it are solid.
        map.set_home(Some(at(-35.45, 149.1652)));
        assert_eq!(fact(&map, "map.route.home"), "solid");
        // No home: the first row stands in for it.
        map.set_home(None);
        assert_eq!(fact(&map, "map.routes"), "home:dashed,mission:3");
        // A map drawing no `WPOverlay` draws no route.
        map.set_overlay(None);
        assert_eq!(fact(&map, "map.routes"), "none");
        assert_eq!(fact(&map, "map.route.home"), "none");
    }

    /// The vehicle's fence becomes the "fence" overlay's polygons, circles and pins.
    #[test]
    #[allow(clippy::cast_possible_truncation)] // degrees to the protocol's 1e-7
    fn the_vehicle_fence_is_projected_as_create_overlay_draws_it() {
        let mut map = MapViewport::new(0, 0);
        let item = |command: u16, param1: f32, lat: f64, lng: f64| mp_vehicle::FenceItem {
            command,
            param1,
            x: (lat * 1e7) as i32,
            y: (lng * 1e7) as i32,
        };
        map.set_vehicle_fence(&[
            item(5001, 3.0, -35.361, 149.161),
            item(5001, 3.0, -35.362, 149.161),
            item(5001, 3.0, -35.362, 149.162),
            item(5003, 150.0, -35.37, 149.17),
            item(5004, 40.0, -35.371, 149.171),
        ]);
        assert_eq!(map.wp.fence.polygons.len(), 1);
        assert!(map.wp.fence.polygons[0].1, "an inclusion");
        assert_eq!(map.wp.fence.circles.len(), 2);
        assert!((map.wp.fence.circles[0].1 - 150.0).abs() < f64::EPSILON);
        assert!(map.wp.fence.circles[0].2);
        assert!(!map.wp.fence.circles[1].2);
        assert_eq!(map.wp.fence.pins.len(), 5);
        map.set_vehicle_fence(&[]);
        assert!(map.wp.fence.circles.is_empty());
    }

    /// A selection the screen sets is what the facts say, and a mission with different markers
    /// - `writeKML`'s new ones - takes the discs away.
    #[test]
    fn the_selection_is_published_and_new_markers_are_not_selected() {
        let mut map = MapViewport::new(0, 0);
        map.set_overlay(Some(planner()));
        let rows = [
            waypoint(1, -35.3620, 149.1650),
            waypoint(2, -35.3610, 149.1660),
        ];
        map.set_mission(&rows);
        map.set_selected_rows(&[1, 2]);
        map.set_selected_area(Some((at(-35.36, 149.16), at(-35.37, 149.17))));
        assert_eq!(fact(&map, "map.selected"), "1,2");
        assert_eq!(
            fact(&map, "map.selected_area"),
            "-35.3600000,149.1600000;-35.3700000,149.1700000"
        );
        map.set_mission(&rows);
        assert_eq!(fact(&map, "map.selected"), "1,2", "the same markers");
        map.set_mission(&rows[..1]);
        assert_eq!(fact(&map, "map.selected"), "none");
        assert_eq!(
            map.overlay_rows(),
            vec![(1, at(-35.3620, 149.1650))],
            "the rows the box looks for"
        );
    }

    /// The rectangle's top-left is the greater latitude and the lesser longitude.
    #[test]
    fn the_selection_rect_is_top_left_and_bottom_right() {
        let (top_left, bottom_right) =
            selection_rect(at(-35.37, 149.17), at(-35.36, 149.16)).expect("a rect");
        assert_eq!(top_left, at(-35.36, 149.16));
        assert_eq!(bottom_right, at(-35.37, 149.17));
    }
}
