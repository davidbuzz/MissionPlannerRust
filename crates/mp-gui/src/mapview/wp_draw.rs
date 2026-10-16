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
//! polygons, circles and blue-dot pins; the rally points' "Rally Point / Alt" tooltip;
//! `SelectedArea`, the rubber band the planner's Ctrl box-select and GMap's Alt selection draw;
//! Measure Distance's red markers and green line (`polygonsoverlay`); and the Survey (Grid)
//! dialog's map - its `routesOverlay` of segments, markers and polygon, the "Line:" balloon a
//! segment under the pointer gets, and GMap's scale - which takes the planner's map's place while
//! the dialog shows.
//!
//! Ported from `ExtLibs/Maps/WPOverlay.cs`, `ExtLibs/Maps/GMapMarkerWP.cs`,
//! `ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapRoute.cs` (`OnRender`,
//! `UpdateGraphicsPath`, `IsInside`), `GMapPolygon.cs` (`IsInside`, `OnRender`),
//! `ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs` (the selected area's
//! drawing, the routes' hover, the scale), `GCSViews/FlightPlanner.cs` (`ContextMeasure_Click`)
//! and `Grid/GridUI.cs` (`domainUpDown1_ValueChanged`, `AddDrawPolygon`, `map_OnRouteEnter`),
//! GPL-3.0-only.

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
            // Measure Distance's `polygonsoverlay`.
            (
                "map.measure.markers",
                self.measure.markers.len().to_string(),
            ),
            (
                "map.measure.line",
                self.measure.line.map_or_else(none, |(a, b)| {
                    format!(
                        "{:.7},{:.7};{:.7},{:.7}",
                        a.latitude(),
                        a.longitude(),
                        b.latitude(),
                        b.longitude()
                    )
                }),
            ),
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

// ---------------------------------------------------------------------------------------------
// Measure Distance's marks: `polygonsoverlay`, which `ContextMeasure_Click` alone draws on.
// ---------------------------------------------------------------------------------------------

/// `Color.Green`: the measured line's pen, `line.Stroke.Color = Color.Green`.
/// `// C#: GCSViews/FlightPlanner.cs:2643-2644`
pub const MEASURE_GREEN: u32 = 0x00_80_00;
/// `GMapPolygon.DefaultStroke`'s width, five pixels, which the measured line is drawn with.
/// `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapPolygon.cs:231-262`
pub const MEASURE_WIDTH: f32 = 5.0;

/// What `polygonsoverlay` holds for Measure Distance: the red `GMarkerGoogle` the first choice
/// drops at `startmeasure`; and, while the second choice's "Distance:" box shows, the green
/// "measure dist" `GMapPolygon` from there to where the menu was opened, with a red marker there
/// too - line and markers taken off together once the box is closed.
/// `// C#: GCSViews/FlightPlanner.cs:2625-2661`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MeasureOverlay {
    /// `polygonsoverlay.Markers`: the red markers, in the order they were added.
    pub markers: Vec<LatLon>,
    /// `polygonsoverlay.Polygons`' "measure dist": its two points.
    pub line: Option<(LatLon, LatLon)>,
}

impl MapViewport {
    /// Measure Distance's marks, as `polygonsoverlay` holds them now.
    pub fn set_measure(&mut self, overlay: MeasureOverlay) {
        self.measure = overlay;
    }
}

/// Measure Distance's marks on the planning map: the line first - `GMapOverlay.OnRender` draws an
/// overlay's polygons before its markers - as `GMapPolygon` draws its path, closed through its
/// two points, its fill having no area; then each red marker.
/// `// C#: GCSViews/FlightPlanner.cs:2625-2661;
/// ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapOverlay.cs:306-346;
/// ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapPolygon.cs:174-200`
pub(super) fn paint_measure(
    map: &MapViewport,
    to_screen: &dyn Fn(WebMercator) -> Point<Pixels>,
    window: &mut Window,
    cx: &mut App,
) {
    if !map.overlay_mode.is_some_and(|overlay| overlay.planner) {
        return;
    }
    if let Some((from, to)) = map.measure.line {
        let a = to_screen(from.to_web_mercator());
        let b = to_screen(to.to_web_mercator());
        super::stroke_through(window, &[a, b, a], MEASURE_WIDTH, MEASURE_GREEN);
    }
    for at in &map.measure.markers {
        super::paint_pin(window, cx, to_screen(at.to_web_mercator()), PIN_RED, None);
    }
}

// ---------------------------------------------------------------------------------------------
// The Survey (Grid) dialog's map: its `routesOverlay` as `domainUpDown1_ValueChanged` fills it -
// the grid's segments, its markers, the polygon - the "Line:" balloon `map_OnRouteEnter` puts on
// a segment the pointer comes onto, and the scale `MapScaleInfoEnabled` draws.
// ---------------------------------------------------------------------------------------------

/// The segments' pen: `seg.Stroke = new Pen(Color.Yellow, 4)` with `DashStyle.Custom`, which
/// `GMapRoute` draws solid with its arrows.
/// `// C#: Grid/GridUI.cs:763-767; ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapRoute.cs:161-170`
pub const SEGMENT_WIDTH: f32 = 4.0;
/// `AddDrawPolygon`'s pen, `new Pen(Color.Red, 2)`, the polygon's or a corridor's route.
/// `// C#: Grid/GridUI.cs:934-952`
pub const BOUNDARY_WIDTH: f32 = 2.0;
/// `map.ScalePen = new Pen(Color.Orange)`: the dialog's scale, one pixel.
/// `// C#: Grid/GridUI.cs:110-111`
pub const SCALE_ORANGE: u32 = 0xff_a5_00;
/// `GMapControl.ScaleFont`: sans-serif, 5 point, italic - in pixels at 96 dpi.
/// `// C#: ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs:473`
pub const SCALE_FONT_SIZE: f32 = 5.0 * 96.0 / 72.0;
/// How far inside the map the middle of the part of a segment a script is offered to put the
/// pointer on must be ([`MapViewport::survey_line_anchor`]), pixels.
const ANCHOR_MARGIN: f32 = 10.0;

/// Which of the dialog's markers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GridMarkerKind {
    /// A corner of the polygon, with Boundary ticked: `GMarkerGoogle(item, GMarkerGoogleType.red)`.
    Corner,
    /// A trigger point, with Internals ticked: `GMarkerGoogle(item, GMarkerGoogleType.green)`.
    Trigger,
    /// A lane's start or end, with Markers ticked: `GMapMarkerWP(item, a.ToString())`, which
    /// writes its number.
    Waypoint,
}

/// One of the dialog's markers, and its `ToolTipText`, shown while the pointer is on it
/// (`MarkerTooltipMode.OnMouseOver`).
#[derive(Debug, Clone, PartialEq)]
pub struct GridMarker {
    /// Where it is.
    pub position: LatLon,
    /// Which it is.
    pub kind: GridMarkerKind,
    /// `a.ToString()`: its number.
    pub text: String,
}

/// What `domainUpDown1_ValueChanged` leaves on the dialog's `routesOverlay`.
/// `// C#: Grid/GridUI.cs:618-778, 934-962`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SurveyGrid {
    /// `AddDrawPolygon`'s, with Boundary ticked: the polygon's corners, and whether they are a
    /// `GMapRoute` - with Corridor ticked - rather than a closed `GMapPolygon`.
    pub boundary: Option<(Vec<LatLon>, bool)>,
    /// With Grid ticked, each grid point's `GMapRoute` "segment": from the point drawn before it
    /// to it, and empty for a trigger point Internals leaves out.
    pub segments: Vec<Vec<LatLon>>,
    /// The markers, in the overlay's order: the corners, then the grid's.
    pub markers: Vec<GridMarker>,
}

/// The dialog's map as the map holds it: the overlay, and what the pointer has done to it.
#[derive(Debug, Clone, Default)]
pub struct SurveyMap {
    grid: SurveyGrid,
    /// The run of `domainUpDown1_ValueChanged` the overlay is from.
    generation: u32,
    /// `GMapRoute.IsMouseOver`, segment by segment.
    over: Vec<bool>,
    /// The markers whose bitmap held the pointer at its last move: `GMapMarker.IsMouseOver`.
    pins: Vec<usize>,
    /// `marker`, the `GMapMarkerRect` `map_OnRouteEnter` adds: where, and its "Line: " text.
    line: Option<(LatLon, String)>,
    /// The arrows the last paint drew.
    arrows: usize,
    /// The scale's bars the last paint drew.
    scale: Vec<ScaleBar>,
    /// Whether the map has painted this overlay: until it has, its last view is the planning
    /// map's, not the dialog's.
    painted: bool,
}

/// `GMapRoute.IsInside`, `GraphicsPath.IsOutlineVisible` of the route's path for its pen: a
/// point within half the pen's width of a leg and between its ends, the pen's caps being flat;
/// a leg of no length holds none. The points and the pointer are whole pixels from the map's
/// corner, as GMap's `LocalPoints` and the mouse's are.
/// `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapRoute.cs:107-115, 118-146`
#[must_use]
pub fn outline_holds(points: &[(f32, f32)], (x, y): (f32, f32), width: f32) -> bool {
    let half = f64::from(width) / 2.0;
    points.windows(2).any(|leg| {
        let [a, b] = leg else {
            return false;
        };
        let (dx, dy) = (f64::from(b.0 - a.0), f64::from(b.1 - a.1));
        let length2 = dx.mul_add(dx, dy * dy);
        if length2 <= f64::EPSILON {
            return false;
        }
        let (px, py) = (f64::from(x - a.0), f64::from(y - a.1));
        let along = px.mul_add(dx, py * dy) / length2;
        if !(0.0..=1.0).contains(&along) {
            return false;
        }
        px.mul_add(dy, -(py * dx)).abs() / length2.sqrt() <= half
    })
}

/// `map_OnRouteEnter`'s words for a segment: "Line: " and `item.Distance` - the route's legs on
/// GMap's sphere, in kilometres - written `((float)item.Distance * 1000f).ToString("0.##") + " m"`.
/// (The dialog's feet, under `distunits` "Feet", are not ported: `mp_mission::gridui` is metric.)
/// `// C#: Grid/GridUI.cs:1141-1152; ExtLibs/GMap.NET.Core/GMap.NET/MapRoute.cs:80-96`
#[must_use]
pub fn line_text(points: &[LatLon]) -> String {
    let kilometres: f64 = points
        .windows(2)
        .map(|leg| match leg {
            [a, b] => super::gmap_distance_km(*a, *b),
            _ => 0.0,
        })
        .sum();
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)`
    let metres = kilometres as f32 * 1000.0;
    format!("Line: {} m", mp_mission::dotnet::format_f32(metres, "0.##"))
}

/// The part of the leg from `a` to `b` inside the rectangle `(left, top, right, bottom)`, as the
/// fractions of the leg it starts and ends at, if any of it is inside.
fn clip_leg(
    a: (f32, f32),
    b: (f32, f32),
    (left, top, right, bottom): (f32, f32, f32, f32),
) -> Option<(f32, f32)> {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let (mut start, mut end) = (0.0_f32, 1.0_f32);
    for (p, q) in [
        (-dx, a.0 - left),
        (dx, right - a.0),
        (-dy, a.1 - top),
        (dy, bottom - a.1),
    ] {
        if p.abs() <= f32::EPSILON {
            if q < 0.0 {
                return None;
            }
            continue;
        }
        let r = q / p;
        if p < 0.0 {
            start = start.max(r);
        } else {
            end = end.min(r);
        }
        if start > end {
            return None;
        }
    }
    Some((start, end))
}

impl MapViewport {
    /// The Survey (Grid) dialog's map while it shows - `grid` being what the run `generation` of
    /// `domainUpDown1_ValueChanged` drew - or `None` once it is closed. A new run clears
    /// `routesOverlay` and fills it again, so its segments and markers are new, none is under the
    /// pointer, and the "Line:" balloon has gone with the markers; the same run changes nothing.
    /// `// C#: Grid/GridUI.cs:618-622`
    pub fn set_survey(&mut self, survey: Option<(SurveyGrid, u32)>) {
        let Some((grid, generation)) = survey else {
            self.survey = None;
            return;
        };
        if self
            .survey
            .as_ref()
            .is_some_and(|held| held.generation == generation)
        {
            return;
        }
        let over = vec![false; grid.segments.len()];
        self.survey = Some(SurveyMap {
            grid,
            generation,
            over,
            ..SurveyMap::default()
        });
    }

    /// A position as whole pixels from the map's corner, where the last paint put it.
    fn local_pixel(&self, at: LatLon) -> Option<(f32, f32)> {
        let (x, y) = self.screen_of(at)?;
        let (x, y) = self.to_viewport(x, y);
        Some((x.round(), y.round()))
    }

    /// The pointer moved over the dialog's map with no button down: GMap's `OnMouseMove` over
    /// `routesOverlay`. The markers whose bitmap holds it are under it; each segment the 4-pixel
    /// pen's outline holds it on is entered, and each it was on and is not is left, in the
    /// overlay's order. Entering one is `map_OnRouteEnter`, the balloon put at the pointer with
    /// "Line: " and that segment's length; leaving one is `map_OnRouteLeave`, the balloon taken
    /// off, whichever segment's it was. Leaving the map changes nothing, as GMap's `OnMouseLeave`
    /// leaves every `IsMouseOver` as it was. Returns whether anything is drawn differently.
    ///
    /// The C#'s balloon goes where the pointer was at the move before - `currentMousePosition`,
    /// which the form's `MouseMove` sets after GMap's tests have run - a pixel away for a hand on
    /// a mouse; here it goes where the pointer is.
    /// `// C#: Grid/GridUI.cs:1141-1180, 1253-1258;
    /// ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs:2134-2235, 2312, 2359-2363`
    pub fn survey_hover(&mut self, (x, y): (f32, f32)) -> bool {
        let Some(position) = self.position_at(x, y) else {
            return false;
        };
        let (pins, hits) = {
            let Some(survey) = self.survey.as_ref() else {
                return false;
            };
            let (left, top, width, height) = super::PIN_AREA;
            let pins: Vec<usize> = survey
                .grid
                .markers
                .iter()
                .enumerate()
                .filter(|(_, marker)| {
                    self.screen_of(marker.position).is_some_and(|(sx, sy)| {
                        let (dx, dy) = (x - sx, y - sy);
                        dx >= left && dx < left + width && dy >= top && dy < top + height
                    })
                })
                .map(|(index, _)| index)
                .collect();
            let (px, py) = self.to_viewport(x, y);
            let pointer = (px.round(), py.round());
            let hits: Vec<bool> = survey
                .grid
                .segments
                .iter()
                .map(|segment| {
                    let local: Vec<(f32, f32)> = segment
                        .iter()
                        .filter_map(|at| self.local_pixel(*at))
                        .collect();
                    outline_holds(&local, pointer, SEGMENT_WIDTH)
                })
                .collect();
            (pins, hits)
        };
        let Some(survey) = self.survey.as_mut() else {
            return false;
        };
        let before = (survey.pins.clone(), survey.line.clone());
        survey.pins = pins;
        for (index, hit) in hits.into_iter().enumerate() {
            let was = survey.over.get(index).copied().unwrap_or(false);
            if hit && !was {
                let text = survey
                    .grid
                    .segments
                    .get(index)
                    .map_or_else(String::new, |segment| line_text(segment));
                survey.line = Some((position, text));
            } else if !hit && was {
                survey.line = None;
            }
            if let Some(over) = survey.over.get_mut(index) {
                *over = hit;
            }
        }
        before != (survey.pins.clone(), survey.line.clone())
    }

    /// Where a script can put the pointer on one of the dialog's segments: the middle of the
    /// longest part of a leg that lies at least ten pixels inside the map at the last paint, in
    /// the map's own pixels, from the same whole-pixel points the pointer is tested against. None
    /// until the map has painted the overlay.
    #[must_use]
    pub fn survey_line_anchor(&self) -> Option<(f32, f32)> {
        let survey = self.survey.as_ref().filter(|survey| survey.painted)?;
        let (width, height) = self.last_viewport;
        let inside = (
            ANCHOR_MARGIN,
            ANCHOR_MARGIN,
            width - ANCHOR_MARGIN,
            height - ANCHOR_MARGIN,
        );
        let mut best: Option<(f32, (f32, f32))> = None;
        for segment in &survey.grid.segments {
            let local: Vec<(f32, f32)> = segment
                .iter()
                .filter_map(|at| self.local_pixel(*at))
                .collect();
            for leg in local.windows(2) {
                let [a, b] = leg else {
                    continue;
                };
                let Some((start, end)) = clip_leg(*a, *b, inside) else {
                    continue;
                };
                let length = (end - start) * (b.0 - a.0).hypot(b.1 - a.1);
                let middle = (start + end) / 2.0;
                let at = (
                    (b.0 - a.0).mul_add(middle, a.0),
                    (b.1 - a.1).mul_add(middle, a.1),
                );
                if length > 0.0 && best.is_none_or(|(longest, _)| length > longest) {
                    best = Some((length, at));
                }
            }
        }
        best.map(|(_, at)| at)
    }

    /// Facts for the dialog's map: its segments, markers and boundary, the arrows drawn, the
    /// "Line:" balloon, and the scale's bars as `label:width`.
    pub(super) fn survey_facts(&self) -> Vec<(&'static str, String)> {
        let none = || "none".to_owned();
        let Some(survey) = self.survey.as_ref() else {
            return vec![
                ("map.survey", none()),
                ("map.survey.segments", "0".to_owned()),
                ("map.survey.markers", "0".to_owned()),
                ("map.survey.boundary", none()),
                ("map.survey.arrows", "0".to_owned()),
                ("map.survey.line", none()),
                ("map.scale", none()),
            ];
        };
        let scale: Vec<String> = survey
            .scale
            .iter()
            .map(|bar| format!("{}:{}", bar.label, bar.width))
            .collect();
        vec![
            ("map.survey", "shown".to_owned()),
            (
                "map.survey.segments",
                survey.grid.segments.len().to_string(),
            ),
            ("map.survey.markers", survey.grid.markers.len().to_string()),
            (
                "map.survey.boundary",
                match survey.grid.boundary {
                    Some((_, true)) => "route".to_owned(),
                    Some((_, false)) => "polygon".to_owned(),
                    None => none(),
                },
            ),
            ("map.survey.arrows", survey.arrows.to_string()),
            (
                "map.survey.line",
                survey
                    .line
                    .as_ref()
                    .map_or_else(none, |(_, text)| text.clone()),
            ),
            (
                "map.scale",
                if scale.is_empty() {
                    none()
                } else {
                    scale.join(",")
                },
            ),
        ]
    }
}

/// The dialog's map in `routesOverlay`'s order - `GMapOverlay.OnRender` draws its routes, then its
/// polygons, then its markers, then their tooltips: a Corridor's boundary route and each segment,
/// the segments four pixels of yellow with an arrow on each leg longer than 20 pixels (solid and
/// without them past 99,000); the boundary polygon, two pixels of red over no fill; each marker -
/// a corner's red pin, a trigger point's green, a lane end's `GMapMarkerWP` with its number at
/// zoom 17 and in or under the pointer; the tooltips of the markers under the pointer; and the
/// "Line:" balloon, the square `GMapToolTip` its marker was given. `zoom` is the map's.
/// `// C#: Grid/GridUI.cs:618-778, 934-962, 1141-1165;
/// ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapOverlay.cs:290-358`
pub(super) fn paint_survey(
    map: &mut MapViewport,
    origin: Point<Pixels>,
    to_screen: &dyn Fn(WebMercator) -> Point<Pixels>,
    zoom: f64,
    window: &mut Window,
    cx: &mut App,
) {
    let Some(survey) = map.survey.as_ref() else {
        return;
    };
    let screen = |points: &[LatLon]| -> Vec<Point<Pixels>> {
        points
            .iter()
            .map(|at| to_screen(at.to_web_mercator()))
            .collect()
    };
    let mut arrows = 0;
    if let Some((corners, true)) = &survey.grid.boundary {
        stroke_route(
            window,
            &screen(corners.as_slice()),
            BOUNDARY_WIDTH,
            FENCE_RED,
            None,
        );
    }
    for segment in &survey.grid.segments {
        let points = screen(segment.as_slice());
        if points.len() < 2 {
            continue;
        }
        // `LocalPoints`, whole pixels from the map's corner.
        let local: Vec<(f32, f32)> = points
            .iter()
            .map(|p| {
                (
                    f32::from(p.x - origin.x).round(),
                    f32::from(p.y - origin.y).round(),
                )
            })
            .collect();
        stroke_route(window, &points, SEGMENT_WIDTH, ROUTE_YELLOW, None);
        if !drawn_solid(&local) {
            let at = |(x, y): (f32, f32)| point(origin.x + px(x), origin.y + px(y));
            for arrow in route_arrows(&local) {
                for arm in [arrow.left, arrow.right] {
                    super::stroke_through(
                        window,
                        &[at(arrow.mid), at(arm)],
                        SEGMENT_WIDTH,
                        ROUTE_YELLOW,
                    );
                }
                arrows += 1;
            }
        }
    }
    if let Some((corners, false)) = &survey.grid.boundary {
        let mut closed = screen(corners.as_slice());
        let first = closed.first().copied();
        if let Some(first) = first
            && closed.len() > 1
            && f32::from(first.x - origin.x).abs() <= POLYGON_MAX
            && f32::from(first.y - origin.y).abs() <= POLYGON_MAX
        {
            closed.push(first);
            super::stroke_through(window, &closed, BOUNDARY_WIDTH, FENCE_RED);
        }
    }
    for (index, marker) in survey.grid.markers.iter().enumerate() {
        let at = to_screen(marker.position.to_web_mercator());
        match marker.kind {
            GridMarkerKind::Corner => super::paint_pin(window, cx, at, PIN_RED, None),
            GridMarkerKind::Trigger => super::paint_pin(window, cx, at, PIN_GREEN, None),
            GridMarkerKind::Waypoint => {
                let shown = super::pin_label_shown(zoom) || survey.pins.contains(&index);
                paint_wp_pin(window, cx, at, shown.then(|| marker.text.clone()));
            }
        }
    }
    let mut tooltips = Vec::new();
    for index in &survey.pins {
        if let Some(marker) = survey.grid.markers.get(*index) {
            let at = to_screen(marker.position.to_web_mercator());
            super::paint_tooltip(window, cx, at, &marker.text);
            tooltips.push(marker.text.clone());
        }
    }
    if let Some((at, text)) = &survey.line {
        super::paint_square_tooltip(window, cx, to_screen(at.to_web_mercator()), text);
        tooltips.push(text.clone());
    }
    map.tooltips_drawn.extend(tooltips);
    if let Some(survey) = map.survey.as_mut() {
        survey.arrows = arrows;
        survey.painted = true;
    }
}

/// One bar of the scale: its width in pixels, its label, and where the label is written from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScaleBar {
    /// The rectangle's width, `(int)(pxRes * transform)`.
    pub width: i32,
    /// `"5000Km"` to `"100m"`.
    pub label: &'static str,
    /// The label's left edge: ten pixels past the bar's width, nine for "100m".
    pub label_x: i32,
}

/// `GMapControl.OnPaintOverlays`' scale while `MapScaleInfoEnabled`: the pixels of 5000 km down to
/// 100 m as `Core.UpdateGroundResolution` finds them at the map's whole zoom and centre latitude -
/// `GetGroundResolution`, the equator on the projection's 6378137 m `Axis` times the latitude's
/// cosine over the map's `256 * 2^zoom` pixels, each `(int)` - and, longest first, a bar for each
/// the map is wider than and zoomed in far enough for, stretched by `MapRenderTransform`: `2^` the
/// zoom's fraction under `ScaleModes.Fractional`, a `float`. The bars are drawn one over another
/// from the same corner. (The C# finds the pixels when a tile load completes; here at each paint.)
/// `// C#: ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs:1682-1726, 2686-2706;
/// ExtLibs/GMap.NET.Core/GMap.NET.Internals/Core.cs:842, 1196-1205;
/// ExtLibs/GMap.NET.Core/GMap.NET/PureProjection.cs:252-255; Grid/GridUI.Designer.cs:1453`
#[must_use]
#[allow(clippy::float_cmp)] // the C#'s `remainder != 0`
pub fn scale_bars(width: i32, zoom: f64, latitude: f64) -> Vec<ScaleBar> {
    let remainder = zoom % 1.0;
    let step = zoom - remainder;
    #[allow(clippy::cast_possible_truncation)] // `(float) Math.Pow(2d, remainder)`
    let transform = if remainder == 0.0 {
        1.0
    } else {
        f64::from(2.0_f64.powf(remainder) as f32)
    };
    let rez = (latitude * (std::f64::consts::PI / 180.0)).cos()
        * 2.0
        * std::f64::consts::PI
        * 6_378_137.0
        / (256.0 * 2.0_f64.powf(step));
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(int)`
    let pixels = |metres: f64| (metres / rez) as i32;
    [
        (5_000_000.0, "5000Km", true, 10),
        (1_000_000.0, "1000Km", true, 10),
        (100_000.0, "100Km", zoom > 2.0, 10),
        (10_000.0, "10Km", zoom > 5.0, 10),
        (1_000.0, "1000m", zoom >= 10.0, 10),
        (100.0, "100m", zoom > 11.0, 9),
    ]
    .into_iter()
    .filter_map(|(metres, label, zoomed, gap)| {
        let resolution = pixels(metres);
        (width > resolution && zoomed).then(|| {
            #[allow(clippy::cast_possible_truncation)] // the C#'s `(int)`
            let drawn = (f64::from(resolution) * transform) as i32;
            ScaleBar {
                width: drawn,
                label,
                label_x: drawn + gap,
            }
        })
    })
    .collect()
}

/// Paints the scale's bars at the map's top left in the scale pen: `DrawRectangle` from (10, 10),
/// the bar's width across and ten down, and its label from `(label_x, 11)` in `ScaleFont`.
/// `// C#: ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs:1684-1726`
pub(super) fn paint_scale(
    bars: &[ScaleBar],
    origin: Point<Pixels>,
    window: &mut Window,
    cx: &mut App,
) {
    let mut font = window.text_style().font();
    font.style = gpui::FontStyle::Italic;
    for bar in bars {
        // `DrawRectangle` covers its right and bottom edges' pixels too.
        #[allow(clippy::cast_precision_loss)] // a pixel count
        let width = bar.width as f32 + 1.0;
        window.paint_quad(gpui::quad(
            Bounds {
                origin: point(origin.x + px(10.0), origin.y + px(10.0)),
                size: gpui::size(px(width), px(11.0)),
            },
            gpui::Corners::default(),
            gpui::rgba(0x00_00_00_00),
            gpui::Edges::all(px(1.0)),
            rgb(SCALE_ORANGE),
            gpui::BorderStyle::default(),
        ));
        let run = TextRun {
            len: bar.label.len(),
            font: font.clone(),
            color: Hsla::from(rgb(SCALE_ORANGE)),
            background_color: None,
            underline: None,
            strikethrough: None,
        };
        let line = window.text_system().shape_line(
            SharedString::from(bar.label),
            px(SCALE_FONT_SIZE),
            &[run],
            None,
        );
        #[allow(clippy::cast_precision_loss)] // a pixel count
        let left = bar.label_x as f32;
        let _ = line.paint(
            point(origin.x + px(left), origin.y + px(11.0)),
            px(SCALE_FONT_SIZE * 1.2),
            TextAlign::Left,
            None,
            window,
            cx,
        );
    }
}

impl MapViewport {
    /// The scale's bars the last paint of the dialog's map drew, kept for the facts.
    pub(super) fn set_survey_scale(&mut self, bars: Vec<ScaleBar>) {
        if let Some(survey) = self.survey.as_mut() {
            survey.scale = bars;
        }
    }

    /// Whether the dialog's map is what this map draws: its overlay set, on the planning screen.
    pub(super) fn draws_survey(&self) -> bool {
        self.survey.is_some() && self.overlay_mode.is_some_and(|overlay| overlay.planner)
    }
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

    /// Measure Distance's `polygonsoverlay` as the facts publish it: its markers and its line.
    #[test]
    fn measure_distances_marks_are_published() {
        let mut map = MapViewport::new(0, 0);
        assert_eq!(fact(&map, "map.measure.markers"), "0");
        assert_eq!(fact(&map, "map.measure.line"), "none");
        let (from, to) = (at(-35.36, 149.16), at(-35.37, 149.17));
        map.set_measure(MeasureOverlay {
            markers: vec![from],
            line: None,
        });
        assert_eq!(fact(&map, "map.measure.markers"), "1");
        assert_eq!(fact(&map, "map.measure.line"), "none");
        map.set_measure(MeasureOverlay {
            markers: vec![from, to],
            line: Some((from, to)),
        });
        assert_eq!(fact(&map, "map.measure.markers"), "2");
        assert_eq!(
            fact(&map, "map.measure.line"),
            "-35.3600000,149.1600000;-35.3700000,149.1700000"
        );
        map.set_measure(MeasureOverlay::default());
        assert_eq!(fact(&map, "map.measure.markers"), "0");
    }

    /// `GMapRoute.IsInside` for a 4-pixel pen: within two pixels of a leg and between its ends,
    /// the caps being flat; a leg of no length holds nothing.
    #[test]
    fn a_route_holds_the_points_within_half_its_pen() {
        let leg = [(100.0, 100.0), (200.0, 100.0)];
        assert!(outline_holds(&leg, (150.0, 100.0), 4.0));
        assert!(outline_holds(&leg, (150.0, 102.0), 4.0));
        assert!(outline_holds(&leg, (100.0, 98.0), 4.0));
        assert!(!outline_holds(&leg, (150.0, 103.0), 4.0));
        assert!(
            !outline_holds(&leg, (99.0, 100.0), 4.0),
            "the caps are flat"
        );
        assert!(!outline_holds(&leg, (201.0, 100.0), 4.0));
        let corner = [(0.0, 0.0), (0.0, 50.0), (50.0, 50.0)];
        assert!(outline_holds(&corner, (25.0, 51.0), 4.0));
        assert!(outline_holds(&corner, (-1.0, 25.0), 4.0));
        assert!(!outline_holds(&corner, (25.0, 25.0), 4.0));
        assert!(!outline_holds(&[(5.0, 5.0), (5.0, 5.0)], (5.0, 5.0), 4.0));
        assert!(!outline_holds(&[(5.0, 5.0)], (5.0, 5.0), 4.0));
    }

    /// `map_OnRouteEnter`'s words: the route's length on GMap's sphere as a single in metres,
    /// `"0.##"` of its seven digits - so a degree of the equator is 111319.5.
    #[test]
    fn the_line_balloon_is_the_segments_length_in_metres() {
        assert_eq!(line_text(&[at(0.0, 0.0), at(0.0, 1.0)]), "Line: 111319.5 m");
        assert_eq!(
            line_text(&[
                at(-35.36, 149.16),
                at(-35.36, 149.161),
                at(-35.3605, 149.161)
            ]),
            "Line: 146.44 m"
        );
        assert_eq!(
            line_text(&[at(-35.36, 149.16), at(-35.3605, 149.16)]),
            "Line: 55.66 m"
        );
        assert_eq!(line_text(&[]), "Line: 0 m");
    }

    /// GMap's scale: at the equator at zoom 15 the 1000 m bar is 209 pixels and the 100 m one 20,
    /// each label ten past its bar (nine for 100 m); half a level in, both stretched by the single
    /// `2^0.5`; at zoom 11 the 10 km and 1000 m bars, the 100 m one waiting for a zoom past 11; at
    /// zoom 2 the 5000 and 1000 km ones; none the map is not wider than; and further from the
    /// equator a metre is more pixels.
    #[test]
    fn the_scale_picks_its_bars_by_width_and_zoom() {
        let bar = |label, width, label_x| ScaleBar {
            width,
            label,
            label_x,
        };
        assert_eq!(
            scale_bars(800, 15.0, 0.0),
            vec![bar("1000m", 209, 219), bar("100m", 20, 29)]
        );
        assert_eq!(
            scale_bars(800, 15.5, 0.0),
            vec![bar("1000m", 295, 305), bar("100m", 28, 37)]
        );
        assert_eq!(
            scale_bars(800, 11.0, 0.0),
            vec![bar("10Km", 130, 140), bar("1000m", 13, 23)]
        );
        assert_eq!(
            scale_bars(800, 2.0, 0.0),
            vec![bar("5000Km", 127, 137), bar("1000Km", 25, 35)]
        );
        assert_eq!(scale_bars(200, 15.0, 0.0), vec![bar("100m", 20, 29)]);
        assert_eq!(
            scale_bars(800, 16.0, -35.363_262_1),
            vec![bar("1000m", 513, 523), bar("100m", 51, 60)]
        );
    }

    /// The dialog's map: a segment the pointer comes onto gets the "Line:" balloon with its
    /// length, a move along it changes nothing and a move off it takes the balloon away; a marker
    /// under the pointer is noted; the place a script is offered lies on the longest segment; the
    /// same run of the handler changes nothing and a new one starts the overlay afresh.
    /// `// C#: Grid/GridUI.cs:618-622, 1141-1180`
    #[test]
    fn a_segment_under_the_pointer_gets_its_line_balloon() {
        let mut map = MapViewport::new(0, 0);
        map.set_overlay(Some(planner()));
        map.paint_for_test(at(-35.3610, 149.1650), 2.5e-5, (800.0, 600.0));
        let (a, b, c) = (
            at(-35.3615, 149.1640),
            at(-35.3615, 149.1660),
            at(-35.3605, 149.1660),
        );
        let grid = SurveyGrid {
            boundary: Some((vec![a, b, c], false)),
            segments: vec![vec![a, a], vec![a, b], vec![b, c]],
            markers: vec![GridMarker {
                position: c,
                kind: GridMarkerKind::Waypoint,
                text: "3".to_owned(),
            }],
        };
        map.set_survey(Some((grid.clone(), 1)));
        assert!(map.draws_survey());
        let fact = |map: &MapViewport, key: &str| {
            map.survey_facts()
                .into_iter()
                .find(|(name, _)| *name == key)
                .map(|(_, value)| value)
                .unwrap_or_else(|| panic!("no fact {key}"))
        };
        assert_eq!(fact(&map, "map.survey"), "shown");
        assert_eq!(fact(&map, "map.survey.segments"), "3");
        assert_eq!(fact(&map, "map.survey.markers"), "1");
        assert_eq!(fact(&map, "map.survey.boundary"), "polygon");
        assert_eq!(fact(&map, "map.survey.line"), "none");

        let (ax, ay) = map.screen_of(a).expect("on the map");
        let (bx, _) = map.screen_of(b).expect("on the map");
        let on_ab = (f32::midpoint(ax, bx), ay);
        assert!(map.survey_hover(on_ab));
        let words = line_text(&[a, b]);
        assert_eq!(fact(&map, "map.survey.line"), words);
        assert!(!map.survey_hover((on_ab.0 + 10.0, on_ab.1)), "along it");
        assert!(map.survey_hover((on_ab.0, on_ab.1 + 10.0)), "off it");
        assert_eq!(fact(&map, "map.survey.line"), "none");

        // On the marker's pin, off both segments.
        let (cx, cy) = map.screen_of(c).expect("on the map");
        assert!(map.survey_hover((cx + 5.0, cy - 20.0)));
        assert_eq!(
            map.survey.as_ref().map(|survey| survey.pins.clone()),
            Some(vec![0])
        );
        assert_eq!(fact(&map, "map.survey.line"), "none");

        // The longest segment's middle, on it - once the map has painted the overlay, its view
        // until then being the planning map's.
        assert_eq!(map.survey_line_anchor(), None, "not painted yet");
        if let Some(survey) = map.survey.as_mut() {
            survey.painted = true;
        }
        let anchor = map.survey_line_anchor().expect("a segment on the map");
        assert!((anchor.0 - on_ab.0).abs() <= 1.0, "{anchor:?} {on_ab:?}");
        assert!((anchor.1 - on_ab.1).abs() <= 1.0, "{anchor:?} {on_ab:?}");
        assert!(map.survey_hover(anchor));
        assert_eq!(fact(&map, "map.survey.line"), words);

        // The same run: as it was. A new run: nothing under the pointer, no balloon.
        map.set_survey(Some((grid.clone(), 1)));
        assert_eq!(fact(&map, "map.survey.line"), words);
        map.set_survey(Some((grid, 2)));
        assert_eq!(fact(&map, "map.survey.line"), "none");
        assert_eq!(map.survey.as_ref().map(|survey| survey.pins.len()), Some(0));
        // Closed: none of it.
        map.set_survey(None);
        assert!(!map.draws_survey());
        assert_eq!(fact(&map, "map.survey"), "none");
        assert_eq!(fact(&map, "map.scale"), "none");
    }

    /// The part of a leg inside a box: from where it comes in to where it goes out.
    #[test]
    fn a_leg_is_clipped_to_the_box() {
        let inside = (10.0, 10.0, 90.0, 90.0);
        assert_eq!(
            clip_leg((0.0, 50.0), (100.0, 50.0), inside),
            Some((0.1, 0.9))
        );
        assert_eq!(
            clip_leg((20.0, 20.0), (80.0, 20.0), inside),
            Some((0.0, 1.0))
        );
        assert_eq!(clip_leg((0.0, 5.0), (100.0, 5.0), inside), None);
        assert_eq!(clip_leg((95.0, 0.0), (95.0, 100.0), inside), None);
    }
}
