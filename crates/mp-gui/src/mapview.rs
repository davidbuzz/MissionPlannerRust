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

//! Map viewport spike (DELIVERABLES.md Deliverable 7/Deliverable 8).
//!
//! # The question this answers
//!
//! `gpui::Primitive` is a closed eight-variant enum and the wgpu renderer's `Surfaces` batch is an
//! empty match arm, so a consumer cannot add a custom GPU pass. The plan's judgement was that this
//! does not matter for a map, because tiles are images, tracks are stroked paths and markers are
//! quads - all of which gpui already renders on the GPU. This module tests that judgement with the
//! real thing rather than an argument: a tile grid, a 100,000-point flight track and a few thousand
//! markers, painted through `canvas()` and measured.
//!
//! If the numbers here hold, Deliverable 8's map needs no fork of gpui. If they do not, the abandon
//! conditions in the plan apply.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::HashMap;
use std::sync::Arc;
use web_time::{Duration, Instant};

use gpui::{
    App, Bounds, Corners, Hsla, Path, PathBuilder, Pixels, Point, RenderImage, SharedString,
    TextAlign, TextRun, Window, canvas, point, px, quad, rgb, size,
};
use mp_mission::MissionItem;
use mp_tiles::store::{TileAnswer, TileStore};
use mp_units::{Bearing, LatLon, TileId, WebMercator, tiles};

/// Which of Mission Planner's vehicle markers the flight map draws: `Common.getMAVMarker`'s choice
/// by the vehicle's `MAV_TYPE` - and, where the C# asks the firmware, by the types that firmware
/// flies.
/// `// C#: Common.cs:71-215`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MarkerKind {
    /// `GMapMarkerPlane`: `FIXED_WING`, or any VTOL type.
    Plane,
    /// `GMapMarkerRover`: `GROUND_ROVER`.
    Rover,
    /// `GMapMarkerBoat`: `SURFACE_BOAT`.
    Boat,
    /// `GMapMarkerSub`: `SUBMARINE`.
    Sub,
    /// `GMapMarkerHeli`: `HELICOPTER`.
    Heli,
    /// `GMapMarkerAntennaTracker`: the ArduTracker firmware, `ANTENNA_TRACKER` here.
    Tracker,
    /// `GMapMarkerSingle`: `COAXIAL`.
    Single,
    /// `GMapMarkerQuad`: `QUADROTOR`, or the ArduCopter firmware - the copter frames here.
    Quad,
    /// `GMarkerGoogle`'s green dot: a type the C# has no marker for.
    #[default]
    Dot,
}

impl MarkerKind {
    /// Every marker, for the tests.
    #[cfg(test)]
    pub const ALL: [Self; 9] = [
        Self::Plane,
        Self::Rover,
        Self::Boat,
        Self::Sub,
        Self::Heli,
        Self::Tracker,
        Self::Single,
        Self::Quad,
        Self::Dot,
    ];

    /// The bitmap a marker draws, and the size it draws it at: `Resources.rover` (70 by 70),
    /// `boat` (21 by 59), `heli` (60 by 60), `sub` scaled to 59 by 59, `redsinglecopter2` (59 by
    /// 59), and the tracker's `Antenna_Tracker_01` at 40 by 40. The quad and the plane are drawn,
    /// the dot has none.
    /// `// C#: ExtLibs/Maps/GMapMarkerRover.cs:12-14, 80; GMapMarkerBoat.cs:12-14; GMapMarkerHeli.cs:12;
    /// GMapMarkerSub.cs:12-13; GMapMarkerSingle.cs:12; GMapMarkerAntennaTracker.cs:12, 43`
    #[must_use]
    pub const fn icon(self) -> Option<(&'static str, (f32, f32))> {
        match self {
            Self::Rover => Some(("rover", (70.0, 70.0))),
            Self::Boat => Some(("boat", (21.0, 59.0))),
            Self::Heli => Some(("heli", (60.0, 60.0))),
            Self::Sub => Some(("sub", (59.0, 59.0))),
            Self::Single => Some(("redsinglecopter2", (59.0, 59.0))),
            Self::Tracker => Some(("Antenna_Tracker_01", (40.0, 40.0))),
            Self::Plane | Self::Quad | Self::Dot => None,
        }
    }

    /// The marker for a `MAV_TYPE`, in the order the C# asks: a plane or VTOL, a rover, a boat,
    /// a submarine, a helicopter, the tracker, a single copter, a copter, else the dot. The C#
    /// asks the firmware for ArduTracker and ArduCopter; the firmware's types stand for it here.
    /// `// C#: Common.cs:71-215`
    #[must_use]
    pub const fn of(mav_type: u8) -> Self {
        match mav_type {
            1 | 19..=25 => Self::Plane,
            10 => Self::Rover,
            11 => Self::Boat,
            12 => Self::Sub,
            4 => Self::Heli,
            5 => Self::Tracker,
            3 => Self::Single,
            2 | 13 | 14 | 15 | 29 => Self::Quad,
            _ => Self::Dot,
        }
    }

    /// Its name, for a fact.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Plane => "plane",
            Self::Rover => "rover",
            Self::Boat => "boat",
            Self::Sub => "sub",
            Self::Heli => "heli",
            Self::Tracker => "tracker",
            Self::Single => "single",
            Self::Quad => "quad",
            Self::Dot => "dot",
        }
    }

    /// Which of the four bearing lines this marker's `OnRender` draws, when its setting is on:
    /// every marker the heading (red), the course (black) and the target (orange); the plane,
    /// rover, boat and sub the nav bearing (green) too; the tracker its two, settings or not.
    /// `// C#: ExtLibs/Maps/GMapMarkerQuad.cs:132-149; GMapMarkerPlane.cs:82-102; GMapMarkerRover.cs:46-66;
    /// GMapMarkerBoat.cs:43-63; GMapMarkerSub.cs:43-63; GMapMarkerHeli.cs:38-53; GMapMarkerSingle.cs:38-53;
    /// GMapMarkerAntennaTracker.cs:31-40`
    #[must_use]
    pub fn lines(self, settings: &MarkerSettings) -> Vec<&'static str> {
        let mut lines = Vec::new();
        if self == Self::Tracker {
            return vec!["heading", "target"];
        }
        if self == Self::Dot {
            return lines;
        }
        if settings.heading {
            lines.push("heading");
        }
        if settings.nav_bearing
            && matches!(self, Self::Plane | Self::Rover | Self::Boat | Self::Sub)
        {
            lines.push("nav_bearing");
        }
        if settings.cog {
            lines.push("cog");
        }
        if settings.target {
            lines.push("target");
        }
        lines
    }
}

/// What the marker draws besides where it is and which way it points: `getMAVMarker`'s other
/// arguments, read from `MAV.cs` and `MAV.param` each update.
/// `// C#: Common.cs:71-215`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MarkerDetails {
    /// Which marker.
    pub kind: MarkerKind,
    /// `cs.groundcourse`, degrees: the black line.
    pub cog: f32,
    /// `cs.nav_bearing`, degrees: the green line, and the quad's `Target` (orange).
    pub nav_bearing: f32,
    /// `cs.target_bearing`, degrees: the orange line of the plane, rover, boat, sub and tracker.
    pub target: f32,
    /// `MAV.sysid`: the quad's and single's number, and the plane's colour.
    pub sysid: u8,
    /// `AVD_W_DIST_XY`, metres, for the quad's orange circle; `-1` without the parameter.
    pub warn: f32,
    /// `AVD_F_DIST_XY`, metres, for the quad's red circle; `-1` without.
    pub danger: f32,
    /// `cs.radius` in metres, the plane's turn radius for its arc.
    pub radius: f32,
}

impl Default for MarkerDetails {
    /// The markers' fields before `getMAVMarker` sets them: the bearings `-1`, which the C# draws
    /// as a line all the same.
    fn default() -> Self {
        Self {
            kind: MarkerKind::Dot,
            cog: -1.0,
            nav_bearing: -1.0,
            target: -1.0,
            sysid: 0,
            warn: -1.0,
            danger: -1.0,
            radius: -1.0,
        }
    }
}

/// `GMapMarkerBase`'s statics, which `MainV2` reads from the settings and the Planner page's
/// check boxes set: the bearing lines' length and which of them are drawn.
/// `// C#: ExtLibs/Maps/GMapMarkerBase.cs:12-17; MainV2.cs:3855-3860`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MarkerSettings {
    /// `GMapMarkerBase_length`: 500 pixels.
    pub length: f32,
    /// `GMapMarkerBase_DisplayCOG`.
    pub cog: bool,
    /// `GMapMarkerBase_DisplayHeading`.
    pub heading: bool,
    /// `GMapMarkerBase_DisplayNavBearing`.
    pub nav_bearing: bool,
    /// `GMapMarkerBase_DisplayRadius`.
    pub radius: bool,
    /// `GMapMarkerBase_DisplayTarget`.
    pub target: bool,
}

impl Default for MarkerSettings {
    fn default() -> Self {
        Self {
            length: 500.0,
            cog: true,
            heading: true,
            nav_bearing: true,
            radius: true,
            target: true,
        }
    }
}

/// A synthetic flight track and the state needed to draw it.
pub struct MapViewport {
    /// Track points in normalised 0..1 space, scaled to the viewport when painted.
    track: Vec<(f32, f32)>,
    /// Marker positions in normalised 0..1 space.
    markers: Vec<(f32, f32)>,
    /// Rolling paint cost, exponentially smoothed.
    paint_ema: Duration,
    /// Worst paint observed since the last reset.
    paint_worst: Duration,
    /// Paints measured.
    paints: u64,
    /// Stroked paths submitted for the track on the last paint.
    track_paths: usize,
    /// Paths the tessellator refused on the last paint.
    track_path_failures: usize,
    /// Tessellated track paths, keyed by the viewport size they were built for.
    ///
    /// Tessellation is the expensive part and the track does not change between frames, so doing
    /// it per frame is pure waste. This is the same insight a real map needs for level-of-detail:
    /// geometry is rebuilt when the view changes, not when the clock ticks.
    cache: Option<(f32, f32, Vec<Path<Pixels>>)>,
    /// Whether the last paint reused cached geometry.
    used_cache: bool,
    /// Per-phase cost of the last paint: tiles, track clone, track submit, markers.
    phases: [Duration; 4],
    /// Points actually submitted after decimation.
    drawn_points: usize,
    /// The vehicle's flight path in projected world coordinates.
    path: Vec<WebMercator>,
    /// The rectangle `path` covers, widened as each point arrives.
    ///
    /// The automatic fit frames the whole path on every paint while the map follows the vehicle,
    /// and finding that rectangle by scanning every point was nearly all of such a paint's CPU
    /// time on a long track (`crates/mp-units/benches/pan_zoom.rs`, `fit_scan_1m`). The path only
    /// grows, so its rectangle only widens, one point at a time.
    path_extent: Option<Extent>,
    /// Where the vehicle is now, and which way it is pointing: `cs.yaw`, the marker's `Heading`.
    vehicle: Option<(WebMercator, Bearing)>,
    /// The rest of what the vehicle's marker draws: which marker, its other bearings, the radii.
    marker: MarkerDetails,
    /// The Planner page's `GMapMarkerBase_*` settings: the lines' length and which are drawn.
    marker_settings: MarkerSettings,
    /// Home, where the screen showing the map puts it: the planner's Home Location boxes, or the
    /// vehicle's home on the flight screen. Drawn as the C#'s "H" marker.
    home: Option<WebMercator>,
    /// The same, as it was given, for the facts a test reads.
    home_position: Option<LatLon>,
    /// Whether the last paint drew the home marker's "H", which the C# shows only past zoom 16.
    home_label_drawn: bool,
    /// The geofence's return location, `geofenceoverlay.Markers[0]`: a red marker.
    fence_return: Option<WebMercator>,
    /// The planned mission, projected once when it is set rather than every frame.
    mission: Vec<(WebMercator, u16)>,
    /// The mission's marker pairs as `WPOverlay.CreateOverlay` makes them: what the radius
    /// circles, the hover tooltips and Zoom to Mission are drawn from. See [`overlay_markers`].
    overlay: Vec<OverlayMarker>,
    /// What the screen showing the map hands `CreateOverlay`, or `None` on a map that draws no
    /// `WPOverlay` (the log browser's).
    overlay_mode: Option<Overlay>,
    /// Home's altitude, which the "H" marker's "Alt:" tooltip shows.
    home_altitude: Option<f64>,
    /// The markers whose area held the pointer at its last move with no button down: GMap's
    /// `IsMouseOver`, kept until the pointer moves again, as GMap keeps it.
    hovered: Hovered,
    /// The tooltips the last paint drew, for the facts.
    tooltips_drawn: Vec<String>,
    /// The flight screen's Guided Mode marker, while it has one.
    guided: Option<GuidedMarker>,
    /// The planner's "Tracker Home" marker, while the tracker's position is set and not home's.
    tracker: Option<GuidedMarker>,
    /// The geofence's exclusion polygons.
    fence_exclusions: Vec<Vec<WebMercator>>,
    /// The planner's `chk_grid`: the UTM grid over the map at zoom 10 and closer.
    grid: bool,
    /// How many grid lines the last paint drew.
    grid_lines_drawn: usize,
    /// The survey area being drawn, if any.
    polygon: Vec<WebMercator>,
    /// Map Tool > KML Overlay's shapes and labels.
    kml: KmlLayer,
    /// The geofence, if one is being drawn or has been read back.
    fence: Vec<WebMercator>,
    /// Rally points, where the vehicle goes on a failsafe.
    rally: Vec<WebMercator>,
    /// Other aircraft, with whether their report is recent enough to be trusted.
    traffic: Vec<(WebMercator, bool)>,
    /// `photosoverlay`: the camera's shots.
    photos: Vec<ProjectedPhoto>,
    /// `kmlpolygons`' `GMapMarkerOverlapCount`, while Camera Overlap is on: each cell's count.
    coverage: Option<Vec<(WebMercator, u32)>>,
    /// Where map imagery comes from, if any has been configured.
    tiles: Option<Arc<TileStore>>,
    /// Tiles already uploaded to the GPU, keyed so the `ImageId` stays stable.
    ///
    /// Stability is the whole game: gpui keys its texture atlas on `ImageId`, so handing it a
    /// fresh `RenderImage` with identical pixels every frame re-uploads every tile every frame.
    /// The earlier spike measured that at 37 microseconds per tile against 0.44 when the id is
    /// stable - a hundredfold difference, and at forty tiles a screen the difference between a
    /// map that pans smoothly and one that does not.
    images: HashMap<TileId, Arc<RenderImage>>,
    /// Tiles drawn from imagery in the last paint.
    tiles_drawn: usize,
    /// Tiles that had to be drawn from a coarser ancestor.
    tiles_approximate: usize,
    /// Tiles with nothing to draw at all.
    tiles_missing: usize,
    /// The view the user has chosen, or `None` while the map follows the vehicle.
    ///
    /// Follow-the-vehicle is right until the moment someone wants to look at something else, and
    /// then it is infuriating: every pan is undone on the next telemetry packet. Panning therefore
    /// takes control, and keeps it until the user gives it back.
    camera: Option<Camera>,
    /// Where the map starts, from the settings' `maplast_lat`, `maplast_lng` and `maplast_zoom`
    /// (`FlightData.cs:524-548`): a place and a zoom, made a camera on the first paint, when the
    /// viewport's width is known. `None` once used or when the settings had none.
    start: Option<(WebMercator, f64)>,
    /// Where the last drag was, in screen pixels.
    drag_from: Option<(f32, f32)>,
    /// The viewport size at the last paint, needed to convert pixel drags into world units.
    last_viewport: (f32, f32),
    /// The world rectangle actually displayed at the last paint: (x, y, width, height).
    ///
    /// Recorded by the painter rather than recomputed on demand because the fit depends on the
    /// viewport size, and a click handler that guessed at the size would place waypoints slightly
    /// away from where the operator clicked - an error too small to see and too large to fly.
    last_view: Option<(f64, f64, f64, f64)>,
    /// Where the viewport sits in the window at the last paint.
    ///
    /// Mouse events arrive in window coordinates while the map thinks in viewport ones. Dragging
    /// only needs the delta so the difference does not show, but zooming to the cursor does: an
    /// uncorrected offset makes the map drift away from the pointer on every scroll.
    last_origin: (f32, f32),
}

/// The smallest rectangle holding a set of world points.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Extent {
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
}

impl Extent {
    /// `extent` widened to hold `p`, or the rectangle of `p` alone. The same minimum and maximum,
    /// point for point, as a scan of the whole set, whatever order the points came in.
    fn grow(extent: Option<Self>, p: WebMercator) -> Self {
        extent.map_or(
            Self {
                min_x: p.x,
                max_x: p.x,
                min_y: p.y,
                max_y: p.y,
            },
            |e| Self {
                min_x: e.min_x.min(p.x),
                max_x: e.max_x.max(p.x),
                min_y: e.min_y.min(p.y),
                max_y: e.max_y.max(p.y),
            },
        )
    }
}

/// A user-chosen view of the world.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Camera {
    /// Centre of the view in projected coordinates.
    pub centre: WebMercator,
    /// Width of the view in projected units; 1.0 is the whole world.
    pub span: f64,
}

/// Tiles across and down. 8x6 at 256px covers a 2048x1536 viewport.
const TILE_COLS: usize = 8;
const TILE_ROWS: usize = 6;

/// Track points per stroked path.
///
/// gpui tessellates paths into a `u16` index buffer, so a single path cannot exceed 65,535
/// vertices. A stroked polyline emits roughly four per point once joins are counted, so 8,000
/// points per chunk leaves comfortable headroom.
const TRACK_CHUNK: usize = 8_000;

/// Track points to keep per horizontal pixel of viewport.
///
/// gpui's path API is submit-every-frame: a cached `Path` must still be cloned into the scene on
/// each paint, and that copy is proportional to vertex count. A 100,000-point track therefore
/// costs ~60 ms per frame no matter how well it is cached - seven times over a 120 fps budget.
///
/// The fix is the one every map uses: never submit more geometry than the screen can show. At two
/// points per pixel a 730 px viewport needs ~1,460 points, and the result is visually identical
/// because the extra points were landing inside the same pixels.
///
/// This is stride decimation, which is enough to establish the budget. The real map (Deliverable 8) needs
/// Douglas-Peucker or a pre-built pyramid so that decimation preserves shape rather than
/// sampling blindly, plus view culling so off-screen track costs nothing at all.
const POINTS_PER_PIXEL: f32 = 2.0;

impl MapViewport {
    /// Builds a viewport with a synthetic track of `track_points` samples.
    ///
    /// The track is a lawnmower survey pattern with jitter, which is both what a real survey
    /// looks like and a worse case for stroking than a smooth curve.
    #[must_use]
    pub fn new(track_points: usize, marker_count: usize) -> Self {
        let mut track = Vec::with_capacity(track_points);
        let legs = 24.0_f32;
        for i in 0..track_points {
            let t = i as f32 / track_points.max(1) as f32;
            let leg = (t * legs).floor();
            let along = (t * legs) - leg;
            // Alternate direction each leg, which is what a lawnmower survey does. Tested on the
            // float directly so there is no conversion to justify.
            let x = if (leg % 2.0).abs() < 0.5 {
                along
            } else {
                1.0 - along
            };
            let y = leg / legs;
            // Jitter so consecutive points are never collinear, which would let a tessellator
            // collapse segments and flatter the measurement.
            let jitter = ((i as f32) * 0.37).sin() * 0.004;
            track.push((x.clamp(0.0, 1.0), (y + jitter).clamp(0.0, 1.0)));
        }

        let mut markers = Vec::with_capacity(marker_count);
        for i in 0..marker_count {
            let t = i as f32 / marker_count.max(1) as f32;
            markers.push((
                (t * 7.0).fract(),
                ((t * 13.0).sin() * 0.5 + 0.5).clamp(0.0, 1.0),
            ));
        }

        Self {
            track,
            markers,
            paint_ema: Duration::ZERO,
            paint_worst: Duration::ZERO,
            paints: 0,
            track_paths: 0,
            track_path_failures: 0,
            cache: None,
            used_cache: false,
            phases: [Duration::ZERO; 4],
            drawn_points: 0,
            path: Vec::new(),
            path_extent: None,
            vehicle: None,
            marker: MarkerDetails::default(),
            marker_settings: MarkerSettings::default(),
            home: None,
            home_position: None,
            home_label_drawn: false,
            fence_return: None,
            mission: Vec::new(),
            overlay: Vec::new(),
            overlay_mode: None,
            home_altitude: None,
            hovered: Hovered::default(),
            tooltips_drawn: Vec::new(),
            guided: None,
            tracker: None,
            fence_exclusions: Vec::new(),
            grid: false,
            grid_lines_drawn: 0,
            polygon: Vec::new(),
            kml: KmlLayer::default(),
            fence: Vec::new(),
            rally: Vec::new(),
            traffic: Vec::new(),
            photos: Vec::new(),
            coverage: None,
            tiles: None,
            images: HashMap::new(),
            tiles_drawn: 0,
            tiles_approximate: 0,
            tiles_missing: 0,
            camera: None,
            start: None,
            drag_from: None,
            last_viewport: (1.0, 1.0),
            last_view: None,
            last_origin: (0.0, 0.0),
        }
    }

    /// Points in the track.
    #[must_use]
    #[allow(dead_code)] // the synthetic scene is opt-in; these describe it when it is on
    pub fn track_len(&self) -> usize {
        self.track.len()
    }

    /// Markers drawn.
    #[must_use]
    #[allow(dead_code)] // the synthetic scene is opt-in; these describe it when it is on
    pub fn marker_len(&self) -> usize {
        self.markers.len()
    }

    /// Smoothed paint cost.
    #[must_use]
    pub const fn paint_ema(&self) -> Duration {
        self.paint_ema
    }

    /// Worst paint cost seen.
    #[must_use]
    pub const fn paint_worst(&self) -> Duration {
        self.paint_worst
    }

    /// Paints measured so far.
    #[must_use]
    pub const fn paints(&self) -> u64 {
        self.paints
    }

    /// Stroked paths the track needed on the last paint.
    #[must_use]
    pub const fn track_paths(&self) -> usize {
        self.track_paths
    }

    /// Paths the tessellator refused on the last paint. Anything above zero means part of the
    /// track was not drawn.
    #[must_use]
    pub const fn track_path_failures(&self) -> usize {
        self.track_path_failures
    }

    /// Whether the last paint reused cached tessellation. Used by the synthetic benchmark path.
    #[must_use]
    #[allow(dead_code)]
    pub const fn used_cache(&self) -> bool {
        self.used_cache
    }

    /// Per-phase cost of the last paint: tiles, path clone, path submit, markers.
    ///
    /// Kept as API rather than folded into the debug line: the phase split is what turned "the
    /// map is slow" into "the cost is geometry submission", and it should stay measurable.
    #[must_use]
    #[allow(dead_code)]
    pub const fn phases(&self) -> [Duration; 4] {
        self.phases
    }

    /// Points submitted to the renderer after decimation.
    #[must_use]
    #[allow(dead_code)]
    pub const fn drawn_points(&self) -> usize {
        self.drawn_points
    }

    /// Records where the vehicle is.
    ///
    /// Positions are appended to the flight path only when the vehicle has actually moved. A
    /// hovering aircraft reports its position several times a second, and storing every report
    /// would grow the path without drawing anything new. Nor is a position with a latitude or a
    /// longitude of 0, as `FlightData` adds a route point only `if (cs.lat != 0 && cs.lng != 0)`:
    /// a GPS reports 0, 0 until its fix, and the route would otherwise run from the Gulf of
    /// Guinea to the vehicle (the owner saw it on 2026-09-26, the SITL started at Brisbane).
    /// `// C#: GCSViews/FlightData.cs:3793-3797`
    pub fn observe(&mut self, position: LatLon, heading: Bearing) {
        // No marker at 0,0 either: `addMAVMarker` returns on one, and a marker there anchors
        // the fit to the Gulf of Guinea and zooms the map out to half the world (the owner's
        // report, 2026-10-03). `// C#: GCSViews/FlightData.cs:962-968`
        if !is_fixed(position) {
            return;
        }
        let projected = position.to_web_mercator();
        self.vehicle = Some((projected, heading));
        self.note_moved(position, projected);
    }

    /// The shown vehicle has no position worth drawing - none, or one at 0,0 - so its marker
    /// comes off the map, as `addMAVMarker` adds none for it; the flown route stays.
    /// `// C#: GCSViews/FlightData.cs:962-968`
    pub fn vehicle_unfixed(&mut self) {
        self.vehicle = None;
    }

    /// What the vehicle's marker draws besides where it is and which way it points, and the
    /// Planner page's settings for it; handed over with each snapshot, as `getMAVMarker` reads
    /// them from `MAV.cs` each update.
    pub fn set_marker(&mut self, details: MarkerDetails, settings: MarkerSettings) {
        self.marker = details;
        self.marker_settings = settings;
    }

    /// The marker as it will be drawn, for the tests.
    #[cfg(test)]
    #[must_use]
    pub const fn marker(&self) -> &MarkerDetails {
        &self.marker
    }

    /// A position observed: appended to the flight path when the vehicle has moved.
    fn note_moved(&mut self, position: LatLon, projected: WebMercator) {
        // A world-space threshold of 1e-8 is roughly a metre near the equator - small enough to
        // trace a taxi, large enough to reject GPS jitter on a stationary vehicle.
        const MOVED: f64 = 1e-8;
        let moved = self.path.last().is_none_or(|last| {
            (last.x - projected.x).abs() > MOVED || (last.y - projected.y).abs() > MOVED
        });
        if moved && is_fixed(position) {
            self.path.push(projected);
            self.path_extent = Some(Extent::grow(self.path_extent, projected));
        }
    }

    /// Puts home where the screen showing the map says it is, or takes it away.
    ///
    /// Set every frame, because the two screens that share this map draw different homes: the
    /// planner's is the Home Location boxes (`writeKML`'s `home`), the flight screen's the
    /// vehicle's `cs.HomeLocation` - see `plan::planner_map_home` and `plan::flight_map_home`.
    /// `// C#: GCSViews/FlightPlanner.cs:1400-1415; GCSViews/FlightData.cs:3808-3845`
    pub fn set_home(&mut self, home: Option<LatLon>) {
        // `writeKML` rebuilds every marker, and a new marker is not under the pointer until the
        // pointer moves: a home that moved is no longer the one hovered.
        // `WPOverlay` draws no "H" at `PointLatLngAlt.Zero`. `// C#: ExtLibs/Maps/WPOverlay.cs:44`
        let home = home.filter(|home| is_fixed(*home));
        if home != self.home_position {
            self.hovered.forget(MarkerTag::Home);
        }
        self.home = home.map(LatLon::to_web_mercator);
        self.home_position = home;
    }

    /// Home's altitude, for the "H" marker's tooltip: `addpolygonmarker("H", ..., home.Alt *
    /// altunitmultiplier, ...)`. Metres here, so the multiplier is 1.
    /// `// C#: ExtLibs/Maps/WPOverlay.cs:44-50, 394-398`
    pub fn set_home_altitude(&mut self, altitude: Option<f64>) {
        self.home_altitude = altitude;
    }

    /// Which `WPOverlay` the screen showing the map builds, or none.
    ///
    /// Set every frame, as home is, because the two screens that share this map build it with
    /// different radii: the planner's are its WP Radius and Loiter Radius boxes, the flight
    /// screen's are zero (`FlightData.cs:3830-3843`).
    pub fn set_overlay(&mut self, overlay: Option<Overlay>) {
        self.overlay_mode = overlay;
    }

    /// Where home is drawn, as it was given.
    #[must_use]
    pub const fn home(&self) -> Option<LatLon> {
        self.home_position
    }

    /// Whether the last paint drew the "H" on the home marker.
    #[must_use]
    pub const fn home_label_drawn(&self) -> bool {
        self.home_label_drawn
    }

    /// Puts the geofence's return location on the map, or takes it away: the red marker that the
    /// Geo-Fence drop-down's Set Return Location, Load from File and Download leave on
    /// `geofenceoverlay`.
    /// `// C#: GCSViews/FlightPlanner.cs:6663-6670, 4368-4381, 881-889`
    pub fn set_fence_return(&mut self, position: Option<LatLon>) {
        self.fence_return = position
            .filter(|position| is_fixed(*position))
            .map(LatLon::to_web_mercator);
    }

    /// Whether the flown route is drawn: on the flight screen, whose `route` it is, and not the
    /// planner's map, which has none (`FlightPlanner.cs` draws no track).
    /// `// C#: GCSViews/FlightData.cs:3771-3797`
    #[must_use]
    pub fn draws_flown_route(&self) -> bool {
        !self.overlay_mode.is_some_and(|overlay| overlay.planner)
    }

    /// Clear Track: the flown route goes, and the map starts recording it again from the
    /// vehicle's next position. (The C# also empties `MAV.camerapoints`, which this map does not
    /// hold.) The flight screen's Actions grid has no Clear Track button yet; this is what it
    /// will call.
    /// `// C#: GCSViews/FlightData.cs:1101-1107`
    #[allow(dead_code)] // the flight screen's Actions grid has no Clear Track button to call it yet
    pub fn clear_track(&mut self) {
        self.path.clear();
    }

    /// Replaces the planned mission shown on the map.
    ///
    /// Only navigation commands are drawn. A DO_SET_SERVO item has zeroes in its coordinate
    /// fields, and plotting those would hang a waypoint marker off the coast of Africa on every
    /// mission that changes a servo.
    pub fn set_mission(&mut self, items: &[MissionItem]) {
        self.mission = items
            .iter()
            .filter_map(|item| {
                let position = item.position().ok().flatten().filter(|p| is_fixed(*p))?;
                Some((position.to_web_mercator(), item.seq))
            })
            .collect();
        let overlay = overlay_markers(items);
        if overlay != self.overlay {
            // `writeKML` makes new markers, home's included, and none of them is under the
            // pointer until it moves.
            self.hovered = Hovered::default();
            self.overlay = overlay;
        }
    }

    /// Replaces the survey area shown on the map.
    pub fn set_polygon(&mut self, vertices: &[LatLon]) {
        self.polygon = fixed_only(vertices);
    }

    /// Replaces KML Overlay's layer: `kmlpolygonsoverlay`'s polygons, routes and labels, projected,
    /// and whether `FlightData.kmlpolygons` holds the polygons and routes too.
    pub fn set_kml(&mut self, overlay: Option<&mp_kml::read::Overlay>, on_flight: bool) {
        let shapes = |shapes: &[mp_kml::read::Shape]| -> Vec<KmlShape> {
            shapes
                .iter()
                .map(|shape| KmlShape {
                    points: shape
                        .points
                        .iter()
                                                .filter_map(|coord| LatLon::new(coord.lat, coord.lon).ok())
                        .filter(|position| is_fixed(*position))
                        .map(LatLon::to_web_mercator)
                        .collect(),
                    argb: shape.argb,
                    #[allow(clippy::cast_precision_loss)] // a pen width
                    width: shape.width as f32,
                })
                .collect()
        };
        self.kml = match overlay {
            Some(overlay) => KmlLayer {
                polygons: shapes(&overlay.polygons),
                routes: shapes(&overlay.routes),
                labels: overlay
                    .labels
                    .iter()
                    .filter_map(|label| {
                        LatLon::new(label.at.lat, label.at.lon)
                            .ok()
                            .map(|at| KmlLabel {
                                at: at.to_web_mercator(),
                                text: label.text.clone(),
                            })
                    })
                    .collect(),
                on_flight,
            },
            None => KmlLayer::default(),
        };
    }

    /// Replaces the geofence shown on the map.
    pub fn set_fence(&mut self, vertices: &[LatLon]) {
        self.fence = fixed_only(vertices);
    }

    /// How many rally pins the map draws.
    #[must_use]
    pub fn rally_count(&self) -> usize {
        self.rally.len()
    }

    /// Replaces the rally points shown on the map.
    pub fn set_rally(&mut self, positions: &[LatLon]) {
        self.rally = fixed_only(positions);
    }

    /// Replaces the other aircraft shown on the map.
    pub fn set_traffic(&mut self, traffic: &[(LatLon, bool)]) {
        self.traffic = traffic
            .iter()
            .filter(|(position, _)| is_fixed(*position))
            .map(|(position, stale)| (position.to_web_mercator(), *stale))
            .collect();
    }

    /// Replaces `photosoverlay`'s markers.
    pub fn set_photos(&mut self, photos: &[PhotoMarker]) {
        self.photos = photos
            .iter()
            .filter(|photo| is_fixed(photo.position))
            .map(|photo| ProjectedPhoto {
                at: photo.position.to_web_mercator(),
                red: photo.below_min_interval,
                footprint: photo
                    .footprint
                    .iter()
                    .map(|corner| corner.to_web_mercator())
                    .collect(),
                draw_footprint: photo.draw_footprint,
                tooltip: photo.tooltip.clone(),
            })
            .collect();
    }

    /// Replaces `kmlpolygons`' overlap count: its cells and their counts, or `None` while Camera
    /// Overlap is off or no shot has come.
    pub fn set_coverage(&mut self, cells: Option<&[(LatLon, u32)]>) {
        self.coverage = cells.map(|cells| {
            cells
                .iter()
                .map(|(at, count)| (at.to_web_mercator(), *count))
                .collect()
        });
    }

    /// Whether the map is following the vehicle rather than a view the user chose.
    #[must_use]
    pub const fn is_following(&self) -> bool {
        self.camera.is_none()
    }

    /// Returns to following the vehicle.
    pub fn follow_vehicle(&mut self) {
        self.camera = None;
    }

    /// Centres the view on a position at the current zoom: `GMapControl.Position = point`.
    ///
    /// Before anything has framed a view there is no zoom to keep, and the span is the
    /// automatic fit's smallest, about 400 m.
    pub fn centre_on(&mut self, at: LatLon) {
        let span = self.current_view().map_or(3.5e-6 * 1.25, |view| view.span);
        self.camera = Some(Camera {
            centre: at.to_web_mercator(),
            span,
        });
    }

    /// The view the user or [`MapViewport::centre_on`] chose, or `None` while it fits itself.
    #[must_use]
    pub const fn camera(&self) -> Option<Camera> {
        self.camera
    }

    /// `gMapControl1.Position = ...; Zoomlevel.Value = ...` at start-up, from the settings'
    /// `maplast_lat`, `maplast_lng` and `maplast_zoom`: the map opens where it was last left,
    /// at that zoom. It is where the map rests while nothing is framed and no view has been
    /// chosen - the vehicle, once heard, is still followed, as `CHK_autopan` (checked by
    /// default) pans to it - so it does not become a camera. A zoom that is not a number is
    /// the default's.
    /// `// C#: GCSViews/FlightData.cs:524-548, 4242-4253; FlightData.Designer.cs:2859`
    pub fn start_at(&mut self, at: LatLon, zoom: f64) {
        let zoom = if zoom.is_finite() {
            zoom.clamp(1.0, 18.0)
        } else {
            3.0
        };
        self.start = Some((at.to_web_mercator(), zoom));
    }

    /// `gMapControl1.Position` and `gMapControl1.Zoom`, for `maplast_*` when the screen is left.
    /// `// C#: GCSViews/FlightData.cs:662-664`
    #[must_use]
    pub fn position_and_zoom(&self) -> Option<(LatLon, f64)> {
        Some((self.centre()?, self.zoom_level()?))
    }

    /// The view with nothing to frame and none chosen, for a viewport `w` by `h`: the start
    /// position at its zoom, else (0, 0) at zoom 3, where Mission Planner's map is at start
    /// (`GCSViews/FlightData.cs:524-534`). The world is 1.0 wide; at zoom `z` it is
    /// 256 * 2^z pixels, so the viewport spans `w / (256 * 2^z)` of it - what `gmap_zoom`
    /// reads back as `z`.
    fn idle_view(&self, w: f32, h: f32) -> (f64, f64, f64, f64) {
        let (centre, zoom) = self.start.unwrap_or((WebMercator { x: 0.5, y: 0.5 }, 3.0));
        let span = f64::from(w.max(1.0)) / (256.0 * 2f64.powf(zoom));
        let height = span * f64::from(h) / f64::from(w).max(1.0);
        (centre.x - span / 2.0, centre.y - height / 2.0, span, height)
    }

    /// `GMapControl.Zoom`: GMap's zoom level of the view on screen, fractional, from its span and
    /// the width it was last painted at. `None` before the map has painted with a size.
    #[must_use]
    pub fn zoom_level(&self) -> Option<f64> {
        let view = self.current_view()?;
        // GMap keeps the zoom it was given; this works it back from a span, and a zoom of 17 set
        // must not read back as 16.999999999999996 - whose whole part, which `SetZoomToFitRect`
        // compares, is 16.
        Some((gmap_zoom(view.span, self.last_viewport.0) * 1e9).round() / 1e9)
    }

    /// `GMapControl.Position`: the position at the centre of the view.
    #[must_use]
    pub fn centre(&self) -> Option<LatLon> {
        LatLon::from_web_mercator(self.current_view()?.centre).ok()
    }

    /// `GMapControl.Zoom = zoom`: the view zoomed about its centre, clamped to the planning map's
    /// `MinZoom` and `MaxZoom`. `ScaleMode` is `Fractional`, so a fractional zoom scales the view
    /// by `2^remainder` - exactly `256 * 2^zoom` pixels round the world, as [`gmap_zoom`] reads
    /// it back. Taking a zoom stops the view fitting itself, as panning does.
    ///
    /// Before anything has framed a view there is no centre to zoom about, and nothing happens.
    /// `// C#: ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs:2666-2715;
    /// GCSViews/FlightPlanner.Designer.cs:883-891`
    pub fn set_zoom(&mut self, zoom: f64) {
        let Some(view) = self.current_view() else {
            return;
        };
        let zoom = zoom.clamp(GMAP_MIN_ZOOM, GMAP_MAX_ZOOM);
        let width = f64::from(self.last_viewport.0.max(1.0));
        self.camera = Some(Camera {
            centre: view.centre,
            span: width / (256.0 * zoom.exp2()),
        });
    }

    /// `GMapControl.ZoomAndCenterMarkers`, for a set of marker positions: the largest whole zoom
    /// at which the rectangle round them fits the map with ten pixels to spare, and the centre of
    /// that rectangle in latitude and longitude. False, and nothing moved, with nothing to fit or
    /// when not even zoom 0 fits.
    ///
    /// `SetZoomToFitRect` leaves the zoom alone when its whole part is already the one that fits
    /// (`(int) Zoom != maxZoom`), so a view at 16.7 asked to fit 16 stays at 16.7.
    /// `// C#: ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs:919-1053;
    /// ExtLibs/GMap.NET.Core/GMap.NET.Internals/Core.cs:549-575`
    pub fn zoom_to_fit(&mut self, points: &[LatLon]) -> bool {
        let fixed: Vec<LatLon> = points.iter().copied().filter(|p| is_fixed(*p)).collect();
        let points = fixed.as_slice();
        let Some(rect) = LatLngRect::around(points) else {
            return false;
        };
        let (width, height) = self.last_viewport;
        let fit = max_zoom_to_fit(&rect, f64::from(width).floor(), f64::from(height).floor());
        if fit <= 0 {
            return false;
        }
        let Ok(centre) = LatLon::new(
            rect.top - (rect.top - rect.bottom) / 2.0,
            rect.left + (rect.right - rect.left) / 2.0,
        ) else {
            return false;
        };
        self.centre_on(centre);
        #[allow(clippy::cast_possible_truncation)] // `(int) Zoom`, and a zoom is 0 to 24
        let whole = self.zoom_level().map(|zoom| zoom.trunc() as i32);
        if whole != Some(fit) {
            self.set_zoom(f64::from(fit));
        }
        true
    }

    /// Where the markers of `WPOverlay` are: home's, then each item's, as
    /// `GetRectOfAllMarkers("WPOverlay")` visits them. Empty on a map drawing no `WPOverlay`.
    /// `// C#: ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs:1007-1053`
    #[must_use]
    pub fn overlay_positions(&self) -> Vec<LatLon> {
        if self.overlay_mode.is_none() {
            return Vec::new();
        }
        self.home_position
            .into_iter()
            .chain(self.overlay.iter().map(|marker| marker.position))
            .collect()
    }

    /// `MainMap.ZoomAndCenterMarkers("WPOverlay")`: Zoom to Mission.
    /// `// C#: GCSViews/FlightPlanner.cs:8383-8386`
    pub fn zoom_and_centre_markers(&mut self) -> bool {
        let points = self.overlay_positions();
        self.zoom_to_fit(&points)
    }

    /// Where a position was drawn at the last paint, in window coordinates.
    ///
    /// For something painted over the map after it, in the same frame: the map records the
    /// rectangle it showed as it paints, so this is exact for that frame.
    #[must_use]
    pub fn screen_of(&self, at: LatLon) -> Option<(f32, f32)> {
        self.screen_position(at.to_web_mercator())
    }

    /// Stops the view moving on its own, keeping whatever is on screen now.
    ///
    /// Called when the operator edits the plan. The automatic fit frames everything it knows
    /// about, so adding a waypoint changes what it has to frame and the map jumps - which means
    /// the next click lands somewhere the operator did not aim at. Editing must not move the
    /// ground under the cursor.
    pub fn freeze_view(&mut self) {
        if self.camera.is_none() {
            self.camera = self.current_view();
        }
    }

    /// Starts a drag at a screen position.
    pub fn begin_drag(&mut self, x: f32, y: f32) {
        self.drag_from = Some((x, y));
        // Taking hold of the map stops it following, so the view does not snap back mid-gesture.
        if self.camera.is_none() {
            self.camera = self.current_view();
        }
    }

    /// Continues a drag, moving the world under the cursor.
    pub fn drag_to(&mut self, x: f32, y: f32) {
        let (width, height) = self.last_viewport;
        let (Some((from_x, from_y)), Some(camera)) = (self.drag_from, self.camera.as_mut()) else {
            return;
        };
        if width <= 0.0 || height <= 0.0 {
            return;
        }

        // The map moves with the cursor, so the world shifts opposite to the pointer.
        let per_pixel = camera.span / f64::from(width);
        camera.centre.x -= f64::from(x - from_x) * per_pixel;
        camera.centre.y -= f64::from(y - from_y) * per_pixel;
        self.drag_from = Some((x, y));
    }

    /// Ends a drag.
    pub fn end_drag(&mut self) {
        self.drag_from = None;
    }

    /// Converts a window position into one relative to the map viewport.
    pub(crate) fn to_viewport(&self, x: f32, y: f32) -> (f32, f32) {
        (x - self.last_origin.0, y - self.last_origin.1)
    }

    /// Zooms by a number of scroll steps, keeping the world under the cursor in place.
    ///
    /// Zooming to the window centre instead is the difference between a map that feels direct and
    /// one that has to be re-panned after every scroll.
    pub fn zoom(&mut self, window_x: f32, window_y: f32, steps: f32) {
        let (cursor_x, cursor_y) = self.to_viewport(window_x, window_y);
        let Some(view) = self.current_view() else {
            return;
        };
        let (width, height) = self.last_viewport;
        if width <= 0.0 || height <= 0.0 {
            return;
        }

        // Each step is a factor of 1.2, clamped so the view cannot invert or exceed the world.
        let factor = f64::from(1.2_f32.powf(-steps));
        let new_span = (view.span * factor).clamp(1e-9, 1.5);

        // Keep the world point under the cursor fixed: shift the centre by the difference between
        // where that point sits before and after the zoom.
        let offset_x = f64::from(cursor_x / width - 0.5);
        let offset_y = f64::from(cursor_y / height - 0.5) * f64::from(height) / f64::from(width);
        let centre = WebMercator {
            x: view.centre.x + offset_x * (view.span - new_span),
            y: view.centre.y + offset_y * (view.span - new_span),
        };
        self.camera = Some(Camera {
            centre,
            span: new_span,
        });
    }

    /// Where a world position sits on screen, in window coordinates.
    ///
    /// The forward direction of [`MapViewport::position_at`], and like it, built from the
    /// rectangle the painter recorded rather than a recomputed one.
    fn screen_position(&self, world: WebMercator) -> Option<(f32, f32)> {
        let (x, y, width, height) = self.last_view?;
        if width <= 0.0 || height <= 0.0 {
            return None;
        }
        let (view_width, view_height) = self.last_viewport;
        #[allow(clippy::cast_possible_truncation)] // screen coordinates are f32 by the renderer
        let screen = (
            ((world.x - x) / width) as f32 * view_width + self.last_origin.0,
            ((world.y - y) / height) as f32 * view_height + self.last_origin.1,
        );
        Some(screen)
    }

    /// The mission waypoint under a window coordinate, if one is close enough to have been meant.
    ///
    /// Close enough is generous compared with the drawn marker: the marker is six pixels across
    /// and nobody hits a six pixel target while looking at an aircraft. Nearest wins, so
    /// overlapping waypoints still resolve to one rather than to whichever was drawn last.
    #[must_use]
    pub fn waypoint_at(&self, window_x: f32, window_y: f32) -> Option<u16> {
        const GRAB_RADIUS_PX: f32 = 14.0;

        let mut best: Option<(f32, u16)> = None;
        for (world, seq) in &self.mission {
            let Some((x, y)) = self.screen_position(*world) else {
                continue;
            };
            let distance = (x - window_x).hypot(y - window_y);
            if distance <= GRAB_RADIUS_PX && best.is_none_or(|(closest, _)| distance < closest) {
                best = Some((distance, *seq));
            }
        }
        best.map(|(_, seq)| seq)
    }

    /// Whether the map has painted once with a size, so a window position can become a place:
    /// what [`Self::position_at`] needs. Published as `map.ready`, which a script that clicks the
    /// map first waits for (a press before the first paint records nothing).
    #[must_use]
    pub fn has_view(&self) -> bool {
        let Some((x, y, width, height)) = self.last_view else {
            return false;
        };
        if self.last_viewport.0 <= 0.0 || self.last_viewport.1 <= 0.0 {
            return false;
        }
        // And the middle of the view is a place: a first paint of the whole world at zoom 0
        // reaches past the poles, where a press converts to nothing (fly-poi.gui, 2026-09-25).
        LatLon::from_web_mercator(WebMercator {
            x: x + width / 2.0,
            y: y + height / 2.0,
        })
        .is_ok()
    }

    /// The position under a window coordinate, or `None` if the map has not painted yet.
    ///
    /// The inverse of the painter's `to_screen`. It reads the rectangle the painter recorded
    /// rather than recomputing one, so a click lands where it looks like it landed even while the
    /// automatic fit is moving.
    #[must_use]
    pub fn position_at(&self, window_x: f32, window_y: f32) -> Option<LatLon> {
        let (x, y, width, height) = self.last_view?;
        let (cursor_x, cursor_y) = self.to_viewport(window_x, window_y);
        let (view_width, view_height) = self.last_viewport;
        if view_width <= 0.0 || view_height <= 0.0 {
            return None;
        }
        let projected = WebMercator {
            x: f64::from(cursor_x / view_width).mul_add(width, x),
            y: f64::from(cursor_y / view_height).mul_add(height, y),
        };
        LatLon::from_web_mercator(projected).ok()
    }

    /// The view currently displayed, whether chosen by the user or fitted automatically.
    fn current_view(&self) -> Option<Camera> {
        if let Some(camera) = self.camera {
            return Some(camera);
        }
        // With nothing to frame, the idle view is the view - so a wheel, a drag or the zoom
        // bar can start from it, and a press freezes it - once a paint has given it a size.
        let (x, y, width, height) = self.view_box().or_else(|| {
            let (w, h) = self.last_viewport;
            self.last_view.is_some().then(|| self.idle_view(w, h))
        })?;
        Some(Camera {
            centre: WebMercator {
                x: x + width / 2.0,
                y: y + height / 2.0,
            },
            span: width,
        })
    }

    /// How many mission waypoints are drawn.
    #[must_use]
    pub fn mission_len(&self) -> usize {
        self.mission.len()
    }

    /// Points in the recorded flight path.
    #[must_use]
    pub fn path_len(&self) -> usize {
        self.path.len()
    }

    /// Whether anything real has been observed yet.
    #[must_use]
    pub const fn has_fix(&self) -> bool {
        self.vehicle.is_some()
    }

    /// The world-space rectangle to display: everything observed, with margin, never narrower
    /// than a minimum span so a stationary vehicle does not zoom to infinity.
    fn view_box(&self) -> Option<(f64, f64, f64, f64)> {
        // A mission loaded from a file can be on the other side of the planet from the vehicle -
        // ArduPilot's own test missions are in Colorado while its default simulator sits in
        // Canberra. Fitting both collapses the map to two dots and shows nothing useful, so a
        // mission that far away is excluded from the framing rather than allowed to ruin it. It
        // is still drawn; the view simply does not chase it.
        const FAR_AWAY: f64 = 0.02; // about 2% of the world, several hundred kilometres

        let near = |p: &WebMercator, a: &WebMercator| {
            (p.x - a.x).abs() < FAR_AWAY && (p.y - a.y).abs() < FAR_AWAY
        };
        // Home by the same rule, measured from the vehicle. The planner's home is whatever the
        // Home Location boxes hold, and at start-up that is the home Mission Planner saved last,
        // which can be a city away from the vehicle that has just connected.
        let home = self
            .home
            .filter(|home| self.vehicle.as_ref().is_none_or(|(v, _)| near(home, v)));
        let anchor = self.vehicle.as_ref().map(|(p, _)| *p).or(home);
        let mission_in_view = self
            .mission
            .iter()
            .map(|(p, _)| p)
            .filter(|p| anchor.is_none_or(|a| near(p, &a)));

        // The path's rectangle is kept as it grows; only the handful of points besides it are
        // visited here.
        let Extent {
            min_x,
            max_x,
            min_y,
            max_y,
        } = home
            .iter()
            .chain(mission_in_view)
            .chain(self.vehicle.as_ref().map(|(p, _)| p))
            .fold(self.path_extent, |extent, p| Some(Extent::grow(extent, *p)))?;

        // About 400 m at the equator; enough context around a parked aircraft to be useful.
        const MIN_SPAN: f64 = 3.5e-6;
        let span = (max_x - min_x).max(max_y - min_y).max(MIN_SPAN) * 1.25;
        let (cx, cy) = ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0);
        // The rect keeps the viewport's shape, so a world unit is the same number of pixels
        // across as down. A square rect into a wide viewport scaled the two axes differently,
        // and a frozen view (which is made from this one with the viewport's shape) then drew
        // everything a few pixels from where the fitted view had it - a marker placed by a
        // click landed above the click once the menu froze the view.
        let (w, h) = self.last_viewport;
        let height = if w > 0.0 && h > 0.0 {
            span * f64::from(h) / f64::from(w)
        } else {
            span
        };
        Some((cx - span / 2.0, cy - height / 2.0, span, height))
    }

    fn record(&mut self, elapsed: Duration) {
        self.paints += 1;
        // Ignore the first few paints: shader compilation and atlas warm-up are one-off costs
        // that would otherwise dominate the average and flatter nothing.
        if self.paints <= 3 {
            return;
        }
        self.paint_worst = self.paint_worst.max(elapsed);
        self.paint_ema = if self.paint_ema.is_zero() {
            elapsed
        } else {
            (self.paint_ema * 7 + elapsed) / 8
        };
    }
}

impl MapViewport {
    /// Gives the map somewhere to get imagery from.
    pub fn set_tiles(&mut self, store: Arc<TileStore>) {
        self.tiles = Some(store);
        self.images.clear();
    }

    /// Whether imagery is configured.
    #[must_use]
    pub const fn has_tiles(&self) -> bool {
        self.tiles.is_some()
    }

    /// Whether the tile store is cache-only: the Planner page's Map Access Mode, or `MP_OFFLINE`.
    #[must_use]
    pub fn tiles_offline(&self) -> bool {
        self.tiles.as_ref().is_some_and(|store| store.is_offline())
    }

    /// Whether the map has painted at all yet.
    #[must_use]
    pub const fn painted(&self) -> bool {
        self.paints > 0
    }

    /// How the last paint went: drawn, approximated from a coarser tile, and missing.
    #[must_use]
    pub const fn tile_counts(&self) -> (usize, usize, usize) {
        (self.tiles_drawn, self.tiles_approximate, self.tiles_missing)
    }

    /// Where the tiles asked for so far came from, or `None` with no imagery configured.
    ///
    /// Cumulative over the store's life, where [`Self::tile_counts`] is the last paint only: the
    /// two answer different questions, "what is on screen" and "how did it get there".
    #[must_use]
    pub fn tile_stats(&self) -> Option<mp_tiles::StoreStats> {
        self.tiles.as_ref().map(|store| store.stats())
    }

    /// What must be shown on screen about where the imagery came from.
    #[must_use]
    pub fn attribution(&self) -> Option<&'static str> {
        self.tiles
            .as_ref()
            .map(|store| store.source().attribution_text())
    }

    /// Which provider is being shown, if any.
    #[must_use]
    pub fn source_id(&self) -> Option<&'static str> {
        self.tiles.as_ref().map(|store| store.source().id)
    }

    /// The uploaded image for a tile, uploading it if this is the first sight of it.
    ///
    /// The cache is keyed on `TileId` so the `ImageId` inside stays the same across frames. It is
    /// bounded, because an hour of panning would otherwise hold every tile ever seen.
    fn image_for(&mut self, id: TileId, tile: &mp_tiles::store::DecodedTile) -> Arc<RenderImage> {
        if let Some(existing) = self.images.get(&id) {
            return Arc::clone(existing);
        }

        // gpui stores RenderImage frames in BGRA despite the RgbaImage container - see
        // decode_static_image_from_decoder, which does pixel.swap(0, 2) after into_rgba8(). A tile
        // decoded from PNG must be swizzled the same way or every tile renders colour-swapped.
        let mut bgra = tile.rgba.clone();
        for pixel in bgra.chunks_exact_mut(4) {
            pixel.swap(0, 2);
        }

        let image = image::RgbaImage::from_raw(tile.width, tile.height, bgra)
            .map(|buffer| Arc::new(RenderImage::new(vec![image::Frame::new(buffer)])));

        let Some(image) = image else {
            // A tile whose pixel count disagrees with its dimensions. Rather than panicking, treat
            // it as missing; the next fetch replaces it.
            return Arc::new(RenderImage::new(vec![image::Frame::new(
                image::RgbaImage::new(1, 1),
            )]));
        };

        if self.images.len() >= MAX_UPLOADED_TILES {
            // Cheap eviction: the map asks for what is on screen every frame, so anything dropped
            // that is still needed is re-uploaded on the next paint. Precision here would cost
            // more than it saves.
            self.images.clear();
        }
        self.images.insert(id, Arc::clone(&image));
        image
    }
}

/// How many uploaded tiles to keep. A screenful is about forty; this is several screenfuls.
const MAX_UPLOADED_TILES: usize = 192;

/// Paints map imagery for the view, falling back to the graticule where there is none.
///
/// Returns false if there is no imagery configured at all, so the caller can draw the placeholder.
fn paint_tiles(
    map: &mut MapViewport,
    origin: Point<Pixels>,
    w: f32,
    h: f32,
    view: (f64, f64, f64, f64),
    window: &mut Window,
) -> bool {
    let Some(store) = map.tiles.clone() else {
        return false;
    };
    let (vx, vy, vw, vh) = view;
    if vw <= 0.0 || vh <= 0.0 || w <= 0.0 || h <= 0.0 {
        return false;
    }

    // The zoom whose tiles are closest to one screen pixel per tile pixel. Choosing by the
    // viewport rather than by a fixed step is what keeps imagery sharp at any window size.
    let zoom = tiles::zoom_for_span(vw, w, tiles::TILE_SIZE_PX).min(store.source().max_zoom);
    let north_west = WebMercator { x: vx, y: vy };
    let south_east = WebMercator {
        x: vx + vw,
        y: vy + vh,
    };

    map.tiles_drawn = 0;
    map.tiles_approximate = 0;
    map.tiles_missing = 0;

    #[allow(clippy::cast_possible_truncation)] // screen coordinates are f32 by the renderer's API
    let screen_rect = |a: WebMercator, b: WebMercator| -> Bounds<Pixels> {
        let left = ((a.x - vx) / vw) as f32 * w;
        let top = ((a.y - vy) / vh) as f32 * h;
        let right = ((b.x - vx) / vw) as f32 * w;
        let bottom = ((b.y - vy) / vh) as f32 * h;
        Bounds {
            origin: point(origin.x + px(left), origin.y + px(top)),
            size: size(px(right - left), px(bottom - top)),
        }
    };

    for id in tiles::tiles_for_view(north_west, south_east, zoom) {
        let (tile_nw, tile_se) = id.bounds();
        let rect = screen_rect(tile_nw, tile_se);

        match store.get(id) {
            TileAnswer::Exact(tile) => {
                let image = map.image_for(id, &tile);
                // bounds clips, image_bounds says where the whole image would go. They are the
                // same for an exact tile.
                let _ = window.paint_image(rect, rect, Corners::default(), image, 0, false);
                map.tiles_drawn += 1;
            }
            TileAnswer::Ancestor { id: ancestor, tile } => {
                let image = map.image_for(ancestor, &tile);
                // Draw the ancestor at its own, larger extent and clip to this tile's rectangle,
                // which shows exactly the part of it that belongs here. This is why the map fills
                // in blurry and then sharpens rather than appearing blank and snapping.
                let (ancestor_nw, ancestor_se) = ancestor.bounds();
                let ancestor_rect = screen_rect(ancestor_nw, ancestor_se);
                let _ =
                    window.paint_image(rect, ancestor_rect, Corners::default(), image, 0, false);
                map.tiles_approximate += 1;
            }
            TileAnswer::Missing => {
                // A flat panel colour, not the checkerboard: a chequered hole among real imagery
                // reads as a rendering fault rather than as a tile that has not arrived.
                window.paint_quad(quad(
                    rect,
                    Corners::default(),
                    rgb(0x1c_25_2d),
                    gpui::Edges::default(),
                    rgb(0x00_00_00),
                    gpui::BorderStyle::default(),
                ));
                map.tiles_missing += 1;
            }
        }
    }
    true
}

/// Paints the checkerboard that stands in for raster tiles when no imagery is configured.
///
/// Drawn whether or not there is anything to plot on it. A map with no fix should look like a map
/// waiting for a position, not like a panel that failed to paint.
#[allow(clippy::cast_precision_loss)] // tile counts are single digits
fn paint_graticule(origin: Point<Pixels>, w: f32, h: f32, window: &mut Window) {
    let tile_w = w / TILE_COLS as f32;
    let tile_h = h / TILE_ROWS as f32;
    for row in 0..TILE_ROWS {
        for col in 0..TILE_COLS {
            let shade = if (row + col) % 2 == 0 {
                0x20_2a_33
            } else {
                0x1c_25_2d
            };
            window.paint_quad(quad(
                Bounds {
                    origin: point(
                        origin.x + px(col as f32 * tile_w),
                        origin.y + px(row as f32 * tile_h),
                    ),
                    size: size(px(tile_w), px(tile_h)),
                },
                gpui::Corners::default(),
                rgb(shade),
                gpui::Edges::default(),
                rgb(0x00_00_00),
                gpui::BorderStyle::default(),
            ));
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The mission overlay as `WPOverlay.CreateOverlay` builds it: a `GMapMarkerWP` and the
// `GMapMarkerRect` round it for each item, the rect's radius circle, and the pointer over them.
// ---------------------------------------------------------------------------------------------

/// The planning map's `MinZoom`: `GMapControl.Zoom` goes no lower.
/// `// C#: GCSViews/FlightPlanner.cs:187`
pub const GMAP_MIN_ZOOM: f64 = 0.0;
/// The planning map's `MaxZoom`, which the Zoom box and the zoom bar also stop at.
/// `// C#: GCSViews/FlightPlanner.cs:188, 3451-3456`
pub const GMAP_MAX_ZOOM: f64 = 24.0;

/// `MAV_CMD` values `CreateOverlay` tests for.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs (MAV_CMD)`
mod cmd {
    pub const WAYPOINT: u16 = 16;
    pub const LOITER_UNLIM: u16 = 17;
    pub const LOITER_TURNS: u16 = 18;
    pub const LOITER_TIME: u16 = 19;
    pub const RETURN_TO_LAUNCH: u16 = 20;
    pub const LAND: u16 = 21;
    pub const CONTINUE_AND_CHANGE_ALT: u16 = 30;
    pub const LOITER_TO_ALT: u16 = 31;
    pub const SPLINE_WAYPOINT: u16 = 82;
    pub const VTOL_LAND: u16 = 85;
    pub const GUIDED_ENABLE: u16 = 92;
    pub const DELAY: u16 = 93;
    pub const LAST: u16 = 95;
    pub const DO_JUMP: u16 = 177;
    pub const DO_RETURN_PATH_START: u16 = 188;
    pub const DO_LAND_START: u16 = 189;
    pub const DO_SET_ROI: u16 = 201;
    pub const FENCE_RETURN_POINT: u16 = 5000;
    pub const FENCE_POLYGON_VERTEX_INCLUSION: u16 = 5001;
    pub const FENCE_POLYGON_VERTEX_EXCLUSION: u16 = 5002;
    pub const FENCE_CIRCLE_INCLUSION: u16 = 5003;
    pub const FENCE_CIRCLE_EXCLUSION: u16 = 5004;
    pub const RALLY_POINT: u16 = 5100;
}

/// `MAV_FRAME` values `GetHomeAlt` tests for.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs (MAV_FRAME)`
mod frame {
    pub const GLOBAL: u8 = 0;
    pub const GLOBAL_INT: u8 = 5;
    pub const GLOBAL_TERRAIN_ALT: u8 = 10;
    pub const GLOBAL_TERRAIN_ALT_INT: u8 = 11;
}

/// `Color.White`: a `GMapMarkerRect`'s pen until it is given another.
/// `// C#: ExtLibs/Maps/GMapMarkerRect.cs:15, 58-63`
pub const RECT_WHITE: u32 = 0xff_ff_ff;
/// `Color.LightBlue`, a loiter's rect.
pub const RECT_LIGHT_BLUE: u32 = 0xad_d8_e6;
/// `Color.Green`, a spline waypoint's rect.
pub const RECT_GREEN: u32 = 0x00_80_00;
/// `Color.Red`, the pen `MainMap_OnMarkerEnter` gives the rect under the pointer.
/// `// C#: GCSViews/FlightPlanner.cs:8072-8074`
pub const RECT_RED: u32 = 0xff_00_00;
/// `Color.Blue`, the Guided Mode marker's rect.
/// `// C#: GCSViews/FlightData.cs:4218-4220`
pub const RECT_BLUE: u32 = 0x00_00_ff;

/// The flight screen's Guided Mode marker: `FlightPlanner.addpolygonmarker(this, "Guided Mode",
/// ...)` onto `routes` while the vehicle is in Guided and has been sent somewhere - a green
/// `GMarkerGoogle` whose tooltip always shows, and a `GMapMarkerRect` of the saved WP radius in
/// blue.
/// `// C#: GCSViews/FlightData.cs:4214-4221; GCSViews/FlightPlanner.cs:1635-1700`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidedMarker {
    /// The marker's tag: "Guided Mode", or "Tracker Home" for the planner's tracker marker,
    /// which `addpolygonmarker` makes the same way.
    pub tag: &'static str,
    /// `GuidedMode.x / 1e7`, `GuidedMode.y / 1e7`.
    pub position: LatLon,
    /// `(int) GuidedMode.z`.
    pub alt: i32,
    /// `Settings.Instance.GetFloat("TXT_WPRad") / CurrentState.multiplierdist`.
    pub wp_radius: f64,
}

impl GuidedMarker {
    /// Its tooltip, `tag + " : " + alt` - what `addpolygonmarker` writes on every pass after the
    /// one that makes the marker, which writes `tag + " - " + alt` until the next.
    /// `// C#: GCSViews/FlightPlanner.cs:1648, 1672`
    #[must_use]
    pub fn tooltip(&self) -> String {
        format!("{} : {}", self.tag, self.alt)
    }
}

/// `GMapMarkerRect`'s area from its point: `Size = (50, 50)`, `Offset = (-25, -45)`.
/// `// C#: ExtLibs/Maps/GMapMarkerRect.cs:43-52`
const RECT_AREA: (f32, f32, f32, f32) = (-25.0, -45.0, 50.0, 50.0);
/// A `GMarkerGoogle` pin's area from its point: the 32 x 32 bitmap at `Offset = (-15, -31)` (see
/// [`PIN_HEAD`]). `GMapMarkerWP` is one.
const PIN_AREA: (f32, f32, f32, f32) = (-15.0, -31.0, 32.0, 32.0);

/// Which marker: home's "H" or an item's number, as the marker's `Tag` reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MarkerTag {
    /// `addpolygonmarker("H", ...)`.
    Home,
    /// `addpolygonmarker((a + 1).ToString(), ...)`: the item's row, which is its `seq`.
    Item(u16),
    /// The flight screen's "Guided Mode" marker.
    Guided,
    /// The planner's "Tracker Home" marker.
    Tracker,
    /// The flight map's `GMapMarkerPhoto`, by its place in `photosoverlay`.
    Photo(u16),
}

impl MarkerTag {
    /// The tag as text.
    #[must_use]
    pub fn text(self) -> String {
        match self {
            Self::Home => "H".to_owned(),
            Self::Item(seq) => seq.to_string(),
            Self::Guided => "Guided Mode".to_owned(),
            Self::Tracker => "Tracker Home".to_owned(),
            Self::Photo(index) => format!("Photo {index}"),
        }
    }
}

/// A `GMapMarkerPhoto`: one `CAMERA_FEEDBACK` on the flight map - the camera icon, red
/// (`camera_icon`) when the shot came sooner after the one before than `CAM_MIN_INTERVAL`
/// allows and green (`camera_icon_G`) otherwise - with its footprint, drawn for the last four
/// shots and under the pointer, and its tooltip. `// C#: ExtLibs/Maps/GMapMarkerPhoto.cs`
#[derive(Debug, Clone, PartialEq)]
pub struct PhotoMarker {
    /// `new PointLatLng(mark.lat / 1e7, mark.lng / 1e7)`.
    pub position: LatLon,
    /// `shotBellowMinInterval`.
    pub below_min_interval: bool,
    /// `footprintpoly`.
    pub footprint: Vec<LatLon>,
    /// `drawfootprint`.
    pub draw_footprint: bool,
    /// `ToolTipText`.
    pub tooltip: String,
}

/// A photo marker as the map keeps it: projected.
#[derive(Debug, Clone, PartialEq)]
struct ProjectedPhoto {
    at: WebMercator,
    red: bool,
    footprint: Vec<WebMercator>,
    draw_footprint: bool,
    tooltip: String,
}

/// `GMapMarkerPhoto`'s icon: `Offset = (-10, -10)`, `Size = 20 by 20`.
const PHOTO_AREA: (f32, f32, f32, f32) = (-10.0, -10.0, 20.0, 20.0);
/// `camera_icon_G`'s disc, and `camera_icon`'s.
/// `// C#: Resources/camera-icon-G.png, Resources/camera-icon.png`
const CAMERA_GREEN: u32 = 0x5d_db_38;
/// See [`CAMERA_GREEN`].
const CAMERA_RED: u32 = 0xb4_19_2d;
/// `Pens.Crimson`, the footprint's outline. `// C#: ExtLibs/Maps/GMapMarkerPhoto.cs:62`
const CRIMSON: u32 = 0xdc_14_3c;
/// `GMapMarkerOverlapCount`'s colours by count, one and up: Purple, Blue, Aqua, Green, Yellow,
/// Orange, Red, DarkRed - eight and more alike. `// C#: ExtLibs/Maps/GMapMarkerOverlapCount.cs:28-38`
const OVERLAP_COLOURS: [u32; 8] = [
    0x80_00_80, 0x00_00_ff, 0x00_ff_ff, 0x00_80_00, 0xff_ff_00, 0xff_a5_00, 0xff_00_00, 0x8b_00_00,
];
/// Their alpha, `Color.FromArgb(140, ...)`.
const OVERLAP_ALPHA: u32 = 140;

/// The radius `addpolygonmarker` gives a marker's `GMapMarkerRect`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RectRadius {
    /// `wprad = 0`: the rect draws no circle - home's, and DO_SET_ROI's.
    None,
    /// `CreateOverlay`'s `wpradius`.
    Wp,
    /// A loiter's: its own radius where the command carries one that is not zero, otherwise
    /// `CreateOverlay`'s `loiterradius`, drawn at its absolute value - the sign is the direction.
    Loiter(Option<f64>),
}

/// One of KML Overlay's `GMapPolygon`s or `GMapRoute`s: its points, projected, and its pen.
#[derive(Debug, Clone, PartialEq)]
struct KmlShape {
    points: Vec<WebMercator>,
    /// The pen's colour as .NET's ARGB.
    argb: u32,
    /// The pen's width, pixels.
    width: f32,
}

/// A `GMapMarkerKMLLabel`: the placemark's name drawn at its point.
#[derive(Debug, Clone, PartialEq)]
struct KmlLabel {
    at: WebMercator,
    text: String,
}

/// `kmlpolygonsoverlay`, and whether `FlightData.kmlpolygons` shares its polygons and routes.
/// `// C#: GCSViews/FlightPlanner.cs:4141-4147, 4246-4262`
#[derive(Debug, Clone, Default, PartialEq)]
struct KmlLayer {
    polygons: Vec<KmlShape>,
    routes: Vec<KmlShape>,
    labels: Vec<KmlLabel>,
    on_flight: bool,
}

/// `GMapMarkerKMLLabel`'s font: `SystemFonts.DefaultFont`, 8.25 pt, in pixels.
const KML_LABEL_SIZE: f32 = 11.0;

/// One item's marker and the `GMapMarkerRect` round it, as `addpolygonmarker` makes them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OverlayMarker {
    /// The marker's tag.
    pub tag: MarkerTag,
    /// Where it is.
    pub position: LatLon,
    /// The same, projected.
    at: WebMercator,
    /// The "Alt: " tooltip's altitude, `item.alt * altunitmultiplier`. `None` on DO_SET_ROI's red
    /// `GMarkerGoogle`, whose tooltip is its number instead.
    pub alt: Option<f64>,
    /// The rect's radius.
    pub radius: RectRadius,
    /// The rect's pen.
    pub colour: u32,
}

/// `WPOverlay.CreateOverlay`'s markers for a mission, item by item: which items get a marker, and
/// what radius and colour its `GMapMarkerRect` has.
///
/// Navigable commands (below `MAV_CMD.LAST` except RTL, CONTINUE_AND_CHANGE_ALT, DELAY and
/// GUIDED_ENABLE, plus DO_SET_ROI, DO_LAND_START and DO_RETURN_PATH_START) are drawn where they
/// have a position; LAND at 0,0 is not, a loiter only needs one coordinate, and a spline waypoint
/// is drawn wherever it is, 0,0 included. A waypoint, a land or a spline has the WP radius; a
/// loiter has its own or the loiter radius, in light blue; a spline's rect is green; DO_SET_ROI is
/// a red marker with a rect of no radius. Item 0, which the flight screen's list starts with, is
/// home, and home has its own marker (`mission_items.RemoveAt(0)`). The fence and rally commands
/// `CreateOverlay` also knows are drawn by their own overlays here.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:52-250, 385-439; GCSViews/FlightData.cs:3826-3829`
#[must_use]
pub fn overlay_markers(items: &[MissionItem]) -> Vec<OverlayMarker> {
    use cmd::{
        CONTINUE_AND_CHANGE_ALT, DELAY, DO_LAND_START, DO_RETURN_PATH_START, DO_SET_ROI,
        GUIDED_ENABLE, LAND, LAST, LOITER_TIME, LOITER_TO_ALT, LOITER_TURNS, LOITER_UNLIM,
        RETURN_TO_LAUNCH, SPLINE_WAYPOINT, VTOL_LAND, WAYPOINT,
    };
    let mut markers = Vec::new();
    for item in items.iter().filter(|item| item.seq != 0) {
        let command = item.command;
        let navigable = (command < LAST
            && !matches!(
                command,
                RETURN_TO_LAUNCH | CONTINUE_AND_CHANGE_ALT | DELAY | GUIDED_ENABLE
            ))
            || matches!(command, DO_SET_ROI | DO_LAND_START | DO_RETURN_PATH_START);
        if command == 0 || !navigable {
            continue;
        }
        let zero = item.x == 0.0 && item.y == 0.0;
        let located = item.x != 0.0 && item.y != 0.0;
        let (radius, colour, roi) = match command {
            LAND | VTOL_LAND if zero => continue,
            DO_LAND_START | LAND | VTOL_LAND if located => (RectRadius::Wp, RECT_WHITE, false),
            DO_SET_ROI if located => (RectRadius::None, RECT_WHITE, true),
            DO_SET_ROI => continue,
            LOITER_TIME | LOITER_TURNS | LOITER_TO_ALT | LOITER_UNLIM => {
                if zero {
                    continue;
                }
                let own = match command {
                    LOITER_TURNS | LOITER_UNLIM => item.param3,
                    LOITER_TO_ALT => item.param2,
                    _ => 0.0,
                };
                (
                    RectRadius::Loiter((own != 0.0).then_some(own)),
                    RECT_LIGHT_BLUE,
                    false,
                )
            }
            SPLINE_WAYPOINT => (RectRadius::Wp, RECT_GREEN, false),
            WAYPOINT if zero => continue,
            _ if located => (RectRadius::Wp, RECT_WHITE, false),
            _ => continue,
        };
        let Ok(position) = LatLon::new(item.x, item.y) else {
            continue;
        };
        markers.push(OverlayMarker {
            tag: MarkerTag::Item(item.seq),
            position,
            at: position.to_web_mercator(),
            alt: (!roi).then_some(item.z),
            radius,
            colour,
        });
    }
    markers
}

/// One entry of `WPOverlay.pointlist`: a `PointLatLngAlt` with the altitude `CreateOverlay` gives
/// it - above sea level, as far as `GetHomeAlt` can make it so - and its tag.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:22; ExtLibs/Utilities/PointLatLngAlt.cs:22-28`
#[derive(Debug, Clone, PartialEq)]
pub struct PlanPoint {
    /// `Lat`.
    pub lat: f64,
    /// `Lng`.
    pub lng: f64,
    /// `Alt`, metres.
    pub alt: f64,
    /// `Tag`: "H" for home, the row's number for an item, "ROI" and the number for a DO_SET_ROI.
    pub tag: String,
}

/// `GetHomeAlt`: what `CreateOverlay` adds to an item's altitude to put it above sea level - 0
/// for an absolute frame, the ground there for a terrain frame (-999 when `srtm` has no height
/// for it), and home's altitude for anything else.
/// `// C#: ExtLibs/Maps/WPOverlay.cs:359-375`
fn get_home_alt(
    altmode: u8,
    homealt: f64,
    lat: f64,
    lng: f64,
    terrain: &dyn Fn(f64, f64) -> crate::srtm::AltResponse,
) -> f64 {
    if altmode == frame::GLOBAL_INT || altmode == frame::GLOBAL {
        return 0.0; // for absolute we dont need to add homealt
    }
    if altmode == frame::GLOBAL_TERRAIN_ALT_INT || altmode == frame::GLOBAL_TERRAIN_ALT {
        let sralt = terrain(lat, lng);
        if sralt.current_type == crate::srtm::TileType::Invalid {
            return -999.0;
        }
        return sralt.alt;
    }
    homealt
}

/// `WPOverlay.CreateOverlay`'s `pointlist` for the planning screen's mission: home first when the
/// Home Location boxes hold one (`writeKML`'s `home`, tagged "H", which is never
/// `PointLatLngAlt.Zero`), then one entry per row - a point where `CreateOverlay` adds one, `None`
/// where it adds `null`, and nothing at all for a LAND or VTOL_LAND at 0,0, which it skips.
///
/// A navigable row's altitude is `item.alt + GetHomeAlt(frame)`, `item.alt` being the grid's
/// `float`; a DO_SET_ROI is tagged "ROI" and its number, wherever it is; a loiter or a waypoint at
/// 0,0 and a DO_JUMP are `null`; the fence and rally commands are points at altitude 0.
/// `// C#: GCSViews/FlightPlanner.cs:1400-1434, 1473; ExtLibs/Maps/WPOverlay.cs:28-356`
#[must_use]
pub fn point_list(
    home: Option<mp_mission::rows::Home>,
    items: &[MissionItem],
    terrain: &dyn Fn(f64, f64) -> crate::srtm::AltResponse,
) -> Vec<Option<PlanPoint>> {
    use cmd::{
        CONTINUE_AND_CHANGE_ALT, DELAY, DO_JUMP, DO_LAND_START, DO_RETURN_PATH_START, DO_SET_ROI,
        FENCE_CIRCLE_EXCLUSION, FENCE_CIRCLE_INCLUSION, FENCE_POLYGON_VERTEX_EXCLUSION,
        FENCE_POLYGON_VERTEX_INCLUSION, FENCE_RETURN_POINT, GUIDED_ENABLE, LAND, LAST, LOITER_TIME,
        LOITER_TO_ALT, LOITER_TURNS, LOITER_UNLIM, RALLY_POINT, RETURN_TO_LAUNCH, SPLINE_WAYPOINT,
        VTOL_LAND, WAYPOINT,
    };
    let mut pointlist = Vec::new();
    // `new PointLatLngAlt()` when a box does not parse: its `Alt` of 0 is still what a relative
    // row adds.
    let homealt = home.map_or(0.0, |home| home.alt);
    if let Some(home) = home {
        pointlist.push(Some(PlanPoint {
            lat: home.lat,
            lng: home.lng,
            alt: home.alt,
            tag: "H".to_owned(),
        }));
    }
    for (a, item) in items.iter().enumerate() {
        let command = item.command;
        let number = (a + 1).to_string();
        // `Locationwp.alt` is a float, and `item.alt + gethomealt(...)` a double.
        #[allow(clippy::cast_possible_truncation)] // the grid's float, on purpose
        let alt = f64::from(item.z as f32);
        let point = |tag: String| {
            Some(PlanPoint {
                lat: item.x,
                lng: item.y,
                alt: alt + get_home_alt(item.frame, homealt, item.x, item.y, terrain),
                tag,
            })
        };
        let zero = item.x == 0.0 && item.y == 0.0;
        let located = item.x != 0.0 && item.y != 0.0;
        // invalid locationwp
        if command == 0 {
            pointlist.push(None);
            continue;
        }
        let navigable = (command < LAST
            && !matches!(
                command,
                RETURN_TO_LAUNCH | CONTINUE_AND_CHANGE_ALT | DELAY | GUIDED_ENABLE
            ))
            || matches!(command, DO_SET_ROI | DO_LAND_START | DO_RETURN_PATH_START);
        if navigable {
            // land can be 0,0 or a lat,lng
            if matches!(command, LAND | VTOL_LAND) && zero {
                continue;
            }
            let entry = match command {
                DO_LAND_START if located => point(number),
                LAND | VTOL_LAND if located => point(number),
                DO_SET_ROI => point(format!("ROI{number}")),
                LOITER_TIME | LOITER_TURNS | LOITER_TO_ALT | LOITER_UNLIM => {
                    if zero {
                        None
                    } else {
                        point(number)
                    }
                }
                SPLINE_WAYPOINT => point(number),
                WAYPOINT if zero => None,
                _ if located => point(number),
                _ => None,
            };
            pointlist.push(entry);
            continue;
        }
        let entry = match command {
            // "fix do jumps into the future": the jump's repeats go into the route, not here.
            DO_JUMP => None,
            FENCE_POLYGON_VERTEX_INCLUSION
            | FENCE_POLYGON_VERTEX_EXCLUSION
            | FENCE_CIRCLE_EXCLUSION
            | FENCE_CIRCLE_INCLUSION
            | FENCE_RETURN_POINT
            | RALLY_POINT => Some(PlanPoint {
                lat: item.x,
                lng: item.y,
                alt: 0.0,
                tag: number,
            }),
            _ => None,
        };
        pointlist.push(entry);
    }
    pointlist
}

/// What the screen showing the map hands `WPOverlay.CreateOverlay`, and whether its
/// `OnMarkerEnter` turns a hovered rect red and selects its row - the planning screen's does; the
/// flight screen's only notes the marker.
/// `// C#: GCSViews/FlightPlanner.cs:1431-1434, 8068-8129; GCSViews/FlightData.cs:3060-3063`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Overlay {
    /// `wpradius`, metres.
    pub wp_radius: f64,
    /// `loiterradius`, metres.
    pub loiter_radius: f64,
    /// Whether this is the planning screen's map.
    pub planner: bool,
}

impl Overlay {
    /// The flight screen's: `CreateOverlay(homeplla, mission_items, 0, 0, ...)`, so no waypoint
    /// has a circle and a loiter has one only where its command carries its own radius.
    /// `// C#: GCSViews/FlightData.cs:3830-3843`
    pub const FLIGHT: Self = Self {
        wp_radius: 0.0,
        loiter_radius: 0.0,
        planner: false,
    };
}

/// Which markers hold the pointer: GMap's `IsMouseOver`, marker by marker.
#[derive(Debug, Clone, Default, PartialEq)]
struct Hovered {
    /// `GMapMarkerRect`s, which the planning screen turns red.
    rects: Vec<MarkerTag>,
    /// `GMapMarkerWP`s, which write their label and show their tooltip.
    pins: Vec<MarkerTag>,
}

impl Hovered {
    /// A marker that is no longer the one it was.
    fn forget(&mut self, tag: MarkerTag) {
        self.rects.retain(|hovered| *hovered != tag);
        self.pins.retain(|hovered| *hovered != tag);
    }
}

/// What a move of the pointer did to the markers under it.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct HoverChange {
    /// The rects the pointer has just entered, in the order GMap raises `OnMarkerEnter` for them.
    pub entered: Vec<MarkerTag>,
    /// Whether anything is drawn differently now.
    pub changed: bool,
}

/// One radius circle, as `GMapMarkerRect.OnRender` draws it at the view last painted.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RadiusCircle {
    /// The marker it is round.
    pub tag: MarkerTag,
    /// The marker's point, in window coordinates.
    pub centre: (f32, f32),
    /// The side of the square `DrawArc` is given, in whole pixels.
    pub diameter: i64,
    /// The radius, metres.
    pub radius: f64,
    /// The pen.
    pub colour: u32,
}

/// `PureProjection.GetDistance`: the haversine distance in kilometres on the projection's
/// `Axis`, 6378137 m for Mercator.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET/PureProjection.cs:436-448`
#[must_use]
pub fn gmap_distance_km(from: LatLon, to: LatLon) -> f64 {
    let (lat1, lng1) = (from.latitude().to_radians(), from.longitude().to_radians());
    let (lat2, lng2) = (to.latitude().to_radians(), to.longitude().to_radians());
    let a = ((lat2 - lat1) / 2.0).sin().powi(2)
        + lat1.cos() * lat2.cos() * ((lng2 - lng1) / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    (6_378_137.0 / 1000.0) * c
}

/// The side of `GMapMarkerRect`'s circle: `loc.X = (int)(LocalPosition.X - m2pixelwidth * wprad *
/// 2)` and the square is `|loc.X - LocalPosition.X|` - twice the radius in pixels, whole, the
/// truncation toward zero taken from where the marker sits.
/// `// C#: ExtLibs/Maps/GMapMarkerRect.cs:78-86`
#[must_use]
pub fn rect_diameter(local_x: i64, m2pixelwidth: f64, radius: f64) -> i64 {
    #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)] // the C#'s `(int)`
    let loc = (local_x as f64 - m2pixelwidth * radius * 2.0) as i32;
    (i64::from(loc) - local_x).abs()
}

impl MapViewport {
    /// GMap's `OnMouseMove` over the markers, the pointer at `pointer` with no button down, or
    /// gone from the map (`None`): every marker whose area holds it is under it. Returns the
    /// rects it has newly entered, which is what the planning screen's `OnMarkerEnter` acts on.
    ///
    /// Home's pair first, then each item's, as `CreateOverlay` adds them.
    /// `// C#: ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs:2134-2185`
    pub fn hover(&mut self, pointer: Option<(f32, f32)>) -> HoverChange {
        let mut now = Hovered::default();
        if let Some((x, y)) = pointer
            && self.overlay_mode.is_some()
        {
            let within =
                |(sx, sy): (f32, f32), (left, top, width, height): (f32, f32, f32, f32)| {
                    let (dx, dy) = (x - sx, y - sy);
                    dx >= left && dx < left + width && dy >= top && dy < top + height
                };
            let home = self.home.map(|at| (MarkerTag::Home, at));
            let items = self.overlay.iter().map(|marker| (marker.tag, marker.at));
            for (tag, at) in home.into_iter().chain(items) {
                let Some(screen) = self.screen_position(at) else {
                    continue;
                };
                if within(screen, RECT_AREA) {
                    now.rects.push(tag);
                }
                if within(screen, PIN_AREA) {
                    now.pins.push(tag);
                }
            }
            // `GMapMarkerPhoto`: its 20 by 20 icon about the point.
            for (index, photo) in self.photos.iter().enumerate() {
                if let Some(screen) = self.screen_position(photo.at)
                    && within(screen, PHOTO_AREA)
                    && let Ok(index) = u16::try_from(index)
                {
                    now.pins.push(MarkerTag::Photo(index));
                }
            }
        }
        let entered = now
            .rects
            .iter()
            .filter(|tag| !self.hovered.rects.contains(tag))
            .copied()
            .collect();
        let changed = now != self.hovered;
        self.hovered = now;
        HoverChange { entered, changed }
    }

    /// Pixels per metre across the map and down it, as `GMapMarkerRect.OnRender` measures them:
    /// the distance along the top edge from the left to `Width`, and - as the C# has it - from the
    /// left to `Height`, also along the top edge.
    /// `// C#: ExtLibs/Maps/GMapMarkerRect.cs:69-76`
    fn metres_to_pixels(&self) -> Option<(f64, f64)> {
        let (x, y, width, _) = self.last_view?;
        let (view_width, view_height) = self.last_viewport;
        if view_width <= 0.0 || view_height <= 0.0 {
            return None;
        }
        let local = |pixels: f32| {
            LatLon::from_web_mercator(WebMercator {
                x: f64::from(pixels / view_width).mul_add(width, x),
                y,
            })
            .ok()
        };
        let origin = local(0.0)?;
        let across = gmap_distance_km(origin, local(view_width)?) * 1000.0;
        let down = gmap_distance_km(origin, local(view_height)?) * 1000.0;
        Some((
            f64::from(view_width) / across,
            f64::from(view_height) / down,
        ))
    }

    /// The radius circles at the view last painted: each `GMapMarkerRect` whose radius is not
    /// zero, scaled from metres to pixels at the top edge of the map, red where the planning
    /// screen has the pointer over it. None on a map drawing no `WPOverlay`.
    /// `// C#: ExtLibs/Maps/GMapMarkerRect.cs:54-98`
    #[must_use]
    pub fn radius_circles(&self) -> Vec<RadiusCircle> {
        let Some((m2pixelwidth, m2pixelheight)) = self.metres_to_pixels() else {
            return Vec::new();
        };
        if !(m2pixelheight > 0.001
            && m2pixelheight.is_finite()
            && m2pixelheight < f64::from(i32::MAX))
        {
            return Vec::new();
        }
        let circle = |tag: MarkerTag, at: WebMercator, radius: f64, colour: u32| {
            if radius == 0.0 {
                return None;
            }
            let centre = self.screen_position(at)?;
            // `LocalPosition` is the point, in whole pixels, plus the rect's offset.
            #[allow(clippy::cast_possible_truncation)] // a screen coordinate, well within range
            let local_x = (centre.0 - self.last_origin.0).floor() as i64 + (RECT_AREA.0 as i64);
            let diameter = rect_diameter(local_x, m2pixelwidth, radius);
            (diameter != 0).then_some(RadiusCircle {
                tag,
                centre,
                diameter,
                radius,
                colour,
            })
        };
        let mut circles = Vec::new();
        if let Some(overlay) = self.overlay_mode {
            for marker in &self.overlay {
                let radius = match marker.radius {
                    RectRadius::None => 0.0,
                    RectRadius::Wp => overlay.wp_radius,
                    RectRadius::Loiter(own) => own.unwrap_or(overlay.loiter_radius).abs(),
                };
                let colour = if overlay.planner && self.hovered.rects.contains(&marker.tag) {
                    RECT_RED
                } else {
                    marker.colour
                };
                circles.extend(circle(marker.tag, marker.at, radius, colour));
            }
        }
        // The flight screen's Guided Mode marker, on `routes` over the mission.
        if let Some(guided) = self.guided {
            circles.extend(circle(
                MarkerTag::Guided,
                guided.position.to_web_mercator(),
                guided.wp_radius,
                RECT_BLUE,
            ));
        }
        // The planner's "Tracker Home", `addpolygonmarker(..., Color.Blue, routesoverlay)`.
        if let Some(tracker) = self.tracker {
            circles.extend(circle(
                MarkerTag::Tracker,
                tracker.position.to_web_mercator(),
                tracker.wp_radius,
                RECT_BLUE,
            ));
        }
        circles
    }

    /// Puts the planner's "Tracker Home" marker on the map, or takes it away.
    pub fn set_tracker(&mut self, tracker: Option<GuidedMarker>) {
        self.tracker = tracker.filter(|marker| is_fixed(marker.position));
    }

    /// `chk_grid_CheckedChanged`: `grid = chk_grid.Checked; MainMap.Refresh()`.
    /// `// C#: GCSViews/FlightPlanner.cs:2053-2057`
    pub fn set_grid(&mut self, on: bool) {
        self.grid = on;
    }

    /// How many UTM grid lines the last paint drew, zone boundaries included.
    #[must_use]
    pub const fn grid_lines_drawn(&self) -> usize {
        self.grid_lines_drawn
    }

    /// The view's corners as the C#'s `MainMap.ViewArea` gives them: top-left and bottom-right,
    /// from the rectangle the last paint recorded. `None` before the first paint.
    #[must_use]
    pub fn view_corners(&self) -> Option<(LatLon, LatLon)> {
        let (x, y, width, height) = self.last_view?;
        let top_left = LatLon::from_web_mercator(WebMercator { x, y }).ok()?;
        let bottom_right = LatLon::from_web_mercator(WebMercator {
            x: x + width,
            y: y + height,
        })
        .ok()?;
        Some((top_left, bottom_right))
    }

    /// Replaces the geofence's exclusion polygons.
    pub fn set_fence_exclusions(&mut self, polygons: &[Vec<LatLon>]) {
        self.fence_exclusions = polygons.iter().map(|polygon| fixed_only(polygon)).collect();
    }

    /// Puts the flight screen's Guided Mode marker on the map, or takes it away.
    pub fn set_guided(&mut self, guided: Option<GuidedMarker>) {
        self.guided = guided.filter(|marker| is_fixed(marker.position));
    }

    /// The "Alt: " tooltips to draw: each `GMapMarkerWP` under the pointer, which
    /// `addpolygonmarker` gave `ToolTipMode.OnMouseOver` and its altitude written `"0"`.
    /// `// C#: ExtLibs/Maps/WPOverlay.cs:393-398; ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapOverlay.cs:348-358`
    fn hover_tooltips(&self) -> Vec<(WebMercator, String)> {
        if self.overlay_mode.is_none() {
            return Vec::new();
        }
        let home = self
            .home
            .zip(self.home_altitude)
            .map(|(at, alt)| (MarkerTag::Home, at, Some(alt)));
        let items = self
            .overlay
            .iter()
            .map(|marker| (marker.tag, marker.at, marker.alt));
        home.into_iter()
            .chain(items)
            .filter(|(tag, _, _)| self.hovered.pins.contains(tag))
            .filter_map(|(_, at, alt)| Some((at, format!("Alt: {}", format_zero(alt?)))))
            .collect()
    }

    /// Facts a test asserts on: the zoom and centre the zoom controls and the Zoom menu set, the
    /// radius circles, and what the pointer is over.
    #[must_use]
    pub fn facts(&self) -> Vec<(&'static str, String)> {
        let zoom = self.zoom_level();
        let circles = self.radius_circles();
        let none = || "none".to_owned();
        let join = |parts: Vec<String>, by: &str| {
            if parts.is_empty() {
                none()
            } else {
                parts.join(by)
            }
        };
        let mut hovered: Vec<MarkerTag> = Vec::new();
        for tag in self.hovered.rects.iter().chain(&self.hovered.pins) {
            if !hovered.contains(tag) {
                hovered.push(*tag);
            }
        }
        #[allow(clippy::cast_possible_truncation)] // `(int) Zoom`, 0 to 24 or so
        let step = zoom.map_or_else(none, |zoom| (zoom.trunc() as i64).to_string());
        vec![
            (
                "map.zoom",
                zoom.map_or_else(none, |zoom| format!("{zoom:.2}")),
            ),
            ("map.zoom.step", step),
            (
                "map.centre",
                self.centre().map_or_else(none, |centre| {
                    format!("{:.7},{:.7}", centre.latitude(), centre.longitude())
                }),
            ),
            ("map.circles", circles.len().to_string()),
            (
                "map.circles.radii",
                join(
                    circles
                        .iter()
                        .map(|circle| circle.radius.to_string())
                        .collect(),
                    ",",
                ),
            ),
            (
                "map.circles.red",
                circles
                    .iter()
                    .filter(|circle| circle.colour == RECT_RED)
                    .count()
                    .to_string(),
            ),
            (
                "map.hover",
                join(hovered.into_iter().map(MarkerTag::text).collect(), ","),
            ),
            ("map.tooltip", join(self.tooltips_drawn.clone(), "|")),
            ("map.vehicle.kind", self.marker.kind.name().to_owned()),
            (
                "map.vehicle.heading",
                self.vehicle
                    .map_or_else(none, |(_, heading)| format!("{:.0}", heading.degrees())),
            ),
            (
                "map.vehicle.lines",
                join(
                    self.marker
                        .kind
                        .lines(&self.marker_settings)
                        .into_iter()
                        .map(str::to_owned)
                        .collect(),
                    ",",
                ),
            ),
            ("map.vehicle.sysid", self.marker.sysid.to_string()),
            ("map.vehicle.drawn", self.vehicle.is_some().to_string()),
        ]
    }
}

/// Whether a position is one worth drawing: a latitude and a longitude that are not 0, which is
/// what a GPS reports before its fix and what an unset mission item or home holds. Mission
/// Planner draws nothing at such a position - no route point (`FlightData.cs:3794`), no vehicle
/// marker (`:962-968`), no waypoint, loiter or landing marker and no leg to it
/// (`WPOverlay.cs:134, 183, 231`), no home (`WPOverlay.cs:44`) - and here every layer applies the
/// same test, and the fit with it, so nothing at 0,0 can be drawn, run a line to, or zoom the map
/// out to half the world (the owner's report, 2026-10-03).
#[must_use]
#[allow(clippy::float_cmp)] // the C#'s test is exact: 0 is what a GPS without a fix sends
pub fn is_fixed(position: LatLon) -> bool {
    position.latitude() != 0.0 && position.longitude() != 0.0
}

/// The positions worth drawing, projected.
fn fixed_only(positions: &[LatLon]) -> Vec<WebMercator> {
    positions
        .iter()
        .filter(|position| is_fixed(**position))
        .map(|position| position.to_web_mercator())
        .collect()
}

/// `double.ToString("0")`: rounded half away from zero, and never "-0".
fn format_zero(value: f64) -> String {
    let rounded = value.round();
    if rounded == 0.0 {
        "0".to_owned()
    } else {
        format!("{rounded:.0}")
    }
}

/// The smallest `RectLatLng` holding a set of positions, as `GetRectOfAllMarkers` builds it.
#[derive(Debug, Clone, Copy, PartialEq)]
struct LatLngRect {
    left: f64,
    top: f64,
    right: f64,
    bottom: f64,
}

impl LatLngRect {
    fn around(points: &[LatLon]) -> Option<Self> {
        let mut points = points.iter();
        let first = points.next()?;
        let start = Self {
            left: first.longitude(),
            top: first.latitude(),
            right: first.longitude(),
            bottom: first.latitude(),
        };
        Some(points.fold(start, |rect, point| Self {
            left: rect.left.min(point.longitude()),
            top: rect.top.max(point.latitude()),
            right: rect.right.max(point.longitude()),
            bottom: rect.bottom.min(point.latitude()),
        }))
    }
}

/// `MercatorProjection.FromLatLngToPixel`: a position's pixel at a whole zoom.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Projections/MercatorProjection.cs:52-71`
fn mercator_pixel(lat: f64, lng: f64, zoom: i32) -> (i64, i64) {
    let lat = lat.clamp(-85.051_128_78, 85.051_128_78);
    let lng = lng.clamp(-180.0, 180.0);
    let x = (lng + 180.0) / 360.0;
    let sin = (lat * std::f64::consts::PI / 180.0).sin();
    let y = 0.5 - ((1.0 + sin) / (1.0 - sin)).ln() / (4.0 * std::f64::consts::PI);
    let size = 256.0 * f64::from(zoom).exp2();
    #[allow(clippy::cast_possible_truncation)] // `(long)`, of a pixel inside the map
    let pixel = |fraction: f64| (fraction * size + 0.5).clamp(0.0, size - 1.0) as i64;
    (pixel(x), pixel(y))
}

/// `Core.GetMaxZoomToFitRect`: the largest whole zoom from `MinZoom` up at which the rectangle is
/// no more than ten pixels wider or taller than the map, and half of `MaxZoom` for a rectangle of
/// no width or no height.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Core.cs:549-575`
fn max_zoom_to_fit(rect: &LatLngRect, width: f64, height: f64) -> i32 {
    #[allow(clippy::cast_possible_truncation)] // 0 and 24
    let (min_zoom, max_zoom) = (GMAP_MIN_ZOOM as i32, GMAP_MAX_ZOOM as i32);
    if rect.top - rect.bottom == 0.0 || rect.right - rect.left == 0.0 {
        return max_zoom / 2;
    }
    let mut zoom = min_zoom;
    for i in min_zoom..=max_zoom {
        let (x1, y1) = mercator_pixel(rect.top, rect.left, i);
        let (x2, y2) = mercator_pixel(rect.bottom, rect.right, i);
        #[allow(clippy::cast_precision_loss)] // pixel counts, far inside f64's integers
        let fits = ((x2 - x1) as f64) <= width + 10.0 && ((y2 - y1) as f64) <= height + 10.0;
        if !fits {
            break;
        }
        zoom = i;
    }
    zoom
}

/// The runs of a circle's outline that can be seen in `clip` (left, top, right, bottom), as
/// polylines. A circle is drawn as short chords; a chord is kept when its box meets the clip, so
/// a circle hundreds of thousands of pixels round - a loiter radius at zoom 22 - costs what its
/// visible arc costs rather than what its whole outline would.
#[must_use]
pub fn circle_runs(
    centre: (f32, f32),
    radius: f32,
    clip: (f32, f32, f32, f32),
) -> Vec<Vec<(f32, f32)>> {
    let (left, top, right, bottom) = clip;
    if radius.is_nan()
        || radius <= 0.0
        || centre.0 + radius < left
        || centre.0 - radius > right
        || centre.1 + radius < top
        || centre.1 - radius > bottom
    {
        return Vec::new();
    }
    // About three pixels a chord, within bounds.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let segments = (radius * std::f32::consts::TAU / 3.0)
        .ceil()
        .clamp(16.0, 16_384.0) as u16;
    let point = |step: u16| {
        let angle = f64::from(step) / f64::from(segments) * std::f64::consts::TAU;
        #[allow(clippy::cast_possible_truncation)] // screen coordinates are f32
        (
            f64::from(radius).mul_add(angle.cos(), f64::from(centre.0)) as f32,
            f64::from(radius).mul_add(angle.sin(), f64::from(centre.1)) as f32,
        )
    };
    let seen = |a: (f32, f32), b: (f32, f32)| {
        a.0.max(b.0) >= left
            && a.0.min(b.0) <= right
            && a.1.max(b.1) >= top
            && a.1.min(b.1) <= bottom
    };
    let kept: Vec<bool> = (0..segments)
        .map(|step| seen(point(step), point(step + 1)))
        .collect();
    let Some(start) = kept.iter().position(|kept| !kept) else {
        return vec![(0..=segments).map(point).collect()];
    };
    let mut runs = Vec::new();
    let mut run: Vec<(f32, f32)> = Vec::new();
    for offset in 1..=kept.len() {
        let index = (start + offset) % kept.len();
        let Ok(step) = u16::try_from(index) else {
            continue;
        };
        if kept.get(index).copied().unwrap_or(false) {
            if run.is_empty() {
                run.push(point(step));
            }
            run.push(point(step + 1));
        } else if !run.is_empty() {
            runs.push(std::mem::take(&mut run));
        }
    }
    if !run.is_empty() {
        runs.push(run);
    }
    runs
}

/// Paints a radius circle: `DrawArc` with the rect's pen, two pixels wide and dashed
/// (`DashStyle.Dash`, three widths on and one off).
/// `// C#: ExtLibs/Maps/GMapMarkerRect.cs:15, 46, 91`
fn paint_radius_circle(window: &mut Window, circle: &RadiusCircle, bounds: Bounds<Pixels>) -> bool {
    let clip = (
        f32::from(bounds.origin.x) - 2.0,
        f32::from(bounds.origin.y) - 2.0,
        f32::from(bounds.origin.x + bounds.size.width) + 2.0,
        f32::from(bounds.origin.y + bounds.size.height) + 2.0,
    );
    #[allow(clippy::cast_precision_loss)] // a pixel count
    let radius = circle.diameter as f32 / 2.0;
    let mut drawn = true;
    for run in circle_runs(circle.centre, radius, clip) {
        let mut builder = PathBuilder::stroke(px(2.0)).dash_array(&[px(6.0), px(2.0)]);
        let mut points = run.iter();
        let Some((x, y)) = points.next() else {
            continue;
        };
        builder.move_to(point(px(*x), px(*y)));
        for (x, y) in points {
            builder.line_to(point(px(*x), px(*y)));
        }
        match builder.build() {
            Ok(path) => window.paint_path(path, Hsla::from(rgb(circle.colour))),
            Err(_) => drawn = false,
        }
    }
    drawn
}

/// `GMapToolTip.DefaultFont`: sans-serif, 14 pixels, bold.
const TOOLTIP_FONT_SIZE: f32 = 14.0;
/// `GMapToolTip.DefaultStroke`: `Color.FromArgb(140, Color.MidnightBlue)`, two pixels.
const TOOLTIP_STROKE: u32 = 0x19_19_70_8c;
/// `GMapToolTip.DefaultFill`: `Color.FromArgb(222, Color.AliceBlue)`.
const TOOLTIP_FILL: u32 = 0xf0_f8_ff_de;
/// `GMapToolTip.DefaultForeground`: `Color.Navy`.
const TOOLTIP_TEXT: u32 = 0x00_00_80;

/// Paints a marker's tooltip as `GMapRoundedToolTip` does: a rounded box of radius 10 whose text
/// is padded 10 each side and 10 in all, placed 14 right of and 44 above the marker's point with
/// its bottom edge there, and a line from the point to the box's lower left. (The line's
/// `RoundAnchor` start cap is not drawn.)
/// `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/ToolTips/GMapRoundedToolTip.cs:16-64;
/// ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapToolTip.cs:44-111`
fn paint_tooltip(window: &mut Window, cx: &mut App, at: Point<Pixels>, text: &str) {
    const RADIUS: f32 = 10.0;
    const OFFSET: (f32, f32) = (14.0, -44.0);
    let mut font = window.text_style().font();
    font.weight = gpui::FontWeight::BOLD;
    let run = TextRun {
        len: text.len(),
        font,
        color: Hsla::from(rgb(TOOLTIP_TEXT)),
        background_color: None,
        underline: None,
        strikethrough: None,
    };
    let line = window.text_system().shape_line(
        SharedString::from(text.to_owned()),
        px(TOOLTIP_FONT_SIZE),
        &[run],
        None,
    );
    // `MeasureString(...).ToSize()`: whole pixels.
    let text_width = f32::from(line.width).ceil();
    let text_height = (TOOLTIP_FONT_SIZE * 1.2).floor();
    let width = text_width + RADIUS * 2.0;
    let height = text_height + RADIUS;
    let left = at.x + px(OFFSET.0);
    let top = at.y - px(text_height) + px(OFFSET.1);

    let mut leader = PathBuilder::stroke(px(2.0));
    leader.move_to(at);
    leader.line_to(point(
        left + px(RADIUS / 2.0),
        top + px(height - RADIUS / 2.0),
    ));
    if let Ok(path) = leader.build() {
        window.paint_path(path, Hsla::from(gpui::rgba(TOOLTIP_STROKE)));
    }
    window.paint_quad(quad(
        Bounds {
            origin: point(left, top),
            size: size(px(width), px(height)),
        },
        Corners::all(px(RADIUS)),
        gpui::rgba(TOOLTIP_FILL),
        gpui::Edges::all(px(2.0)),
        gpui::rgba(TOOLTIP_STROKE),
        gpui::BorderStyle::default(),
    ));
    // `StringAlignment.Center` both ways.
    let origin = point(
        left + px((width - text_width) / 2.0),
        top + px((height - text_height) / 2.0),
    );
    let _ = line.paint(origin, px(text_height), TextAlign::Left, None, window, cx);
}

// ---------------------------------------------------------------------------------------------
// Map Tool > Zoom To: `GMapControl.SetPositionByKeywords` through OpenStreetMap's geocoder.
// ---------------------------------------------------------------------------------------------

/// `OpenStreetMapProviderBase.GeocoderUrlFormat`, `{0}` the keywords.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/OpenStreetMap/OpenStreetMapProvider.cs:473`
pub const GEOCODER_URL: &str = "https://nominatim.openstreetmap.org/search?q={0}&format=xml";
/// OpenStreetMap's `RefererUrl`, which `GetContentUsingHttp` sends.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/OpenStreetMap/OpenStreetMapProvider.cs:17;
/// GMap.NET.MapProviders/GMapProvider.cs:443-461`
pub const GEOCODER_REFERER: &str = "https://www.openstreetmap.org/";

/// `GeoCoderStatusCode`, as far as a keyword search can end.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET/StatusCodes.cs:7-72`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeocoderStatus {
    /// `Unknow`: nothing came back, or not a page of places.
    Unknow,
    /// `G_GEO_SUCCESS`: a page of places, possibly none.
    Success,
    /// `ExceptionInCode`: the request failed, or the page could not be read.
    ExceptionInCode,
}

impl std::fmt::Display for GeocoderStatus {
    /// The member's name, as the C# message writes the enum.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Unknow => "Unknow",
            Self::Success => "G_GEO_SUCCESS",
            Self::ExceptionInCode => "ExceptionInCode",
        })
    }
}

/// `MakeGeocoderUrl`: the keywords with each space made `+`, in the format, and the characters
/// `System.Uri` escapes - anything not ASCII, controls, and `"<>\^`{|}` - percent-encoded as it
/// escapes them, UTF-8 byte by byte.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/OpenStreetMap/OpenStreetMapProvider.cs:264-267`
#[must_use]
pub fn geocoder_url(keywords: &str) -> String {
    let mut escaped = String::new();
    for byte in keywords.replace(' ', "+").bytes() {
        if byte.is_ascii_graphic() && !b"\"<>\\^`{|}".contains(&byte) {
            escaped.push(char::from(byte));
        } else {
            escaped.push_str(&format!("%{byte:02X}"));
        }
    }
    GEOCODER_URL.replace("{0}", &escaped)
}

/// `GetLatLngFromGeocoderUrl` on the page that came back: a page that starts `<?xml` and has a
/// `<place` in it is read, and every `/searchresults/place` whose `place_rank` is not below
/// `MinExpectedRank` (0) gives its `lat` and `lon`; anything else is `Unknow`, and a page that does
/// not read is `ExceptionInCode`.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/OpenStreetMap/OpenStreetMapProvider.cs:22, 286-367`
#[must_use]
pub fn parse_geocoder(page: &str) -> (GeocoderStatus, Vec<LatLon>) {
    if !(page.starts_with("<?xml") && page.contains("<place")) {
        return (GeocoderStatus::Unknow, Vec::new());
    }
    let Some(places) = xml_places(page) else {
        return (GeocoderStatus::ExceptionInCode, Vec::new());
    };
    let mut points = Vec::new();
    for attributes in places {
        let attribute = |name: &str| {
            attributes
                .iter()
                .find(|(key, _)| key == name)
                .map(|(_, value)| value.as_str())
        };
        if let Some(rank) = attribute("place_rank").and_then(|rank| rank.trim().parse::<i32>().ok())
            && rank < 0
        {
            continue;
        }
        // `double.Parse`, which throws on what is not a number: the latitude is read before the
        // longitude is looked for.
        let Some(lat) = attribute("lat") else {
            continue;
        };
        let Ok(lat) = lat.trim().parse::<f64>() else {
            return (GeocoderStatus::ExceptionInCode, Vec::new());
        };
        let Some(lon) = attribute("lon") else {
            continue;
        };
        let Ok(lon) = lon.trim().parse::<f64>() else {
            return (GeocoderStatus::ExceptionInCode, Vec::new());
        };
        // A position that is not one is left out rather than put on the map.
        if let Ok(point) = LatLon::new(lat, lon) {
            points.push(point);
        }
    }
    (GeocoderStatus::Success, points)
}

/// An element's attributes, name and value, in order.
type Attributes = Vec<(String, String)>;

/// A start tag, read.
struct StartTag<'a> {
    /// Its name.
    name: String,
    /// Its attributes, entities decoded.
    attributes: Attributes,
    /// Whether it closes itself, `/>`.
    closed: bool,
    /// What follows it.
    rest: &'a str,
}

/// The attributes of every `place` element directly under a `searchresults` root, or `None` for
/// a page that is not well-formed enough to read.
fn xml_places(page: &str) -> Option<Vec<Attributes>> {
    let mut places = Vec::new();
    let mut depth = 0_usize;
    let mut root: Option<String> = None;
    let mut rest = page;
    while let Some(open) = rest.find('<') {
        rest = rest.get(open..)?;
        if let Some(after) = rest.strip_prefix("<?") {
            rest = after.get(after.find("?>")? + 2..)?;
        } else if let Some(after) = rest.strip_prefix("<!--") {
            rest = after.get(after.find("-->")? + 3..)?;
        } else if let Some(after) = rest.strip_prefix("<!") {
            rest = after.get(after.find('>')? + 1..)?;
        } else if let Some(after) = rest.strip_prefix("</") {
            depth = depth.checked_sub(1)?;
            rest = after.get(after.find('>')? + 1..)?;
        } else {
            let tag = start_tag(rest.get(1..)?)?;
            if depth == 0 {
                if root.is_some() {
                    return None;
                }
                root = Some(tag.name.clone());
            }
            if depth == 1 && root.as_deref() == Some("searchresults") && tag.name == "place" {
                places.push(tag.attributes);
            }
            if !tag.closed {
                depth += 1;
            }
            rest = tag.rest;
        }
    }
    (depth == 0 && root.is_some()).then_some(places)
}

/// A start tag after its `<`.
fn start_tag(text: &str) -> Option<StartTag<'_>> {
    let end_of_name = text.find(|c: char| c.is_whitespace() || c == '/' || c == '>')?;
    let name = text.get(..end_of_name)?.to_owned();
    if name.is_empty() {
        return None;
    }
    let mut rest = text.get(end_of_name..)?;
    let mut attributes = Vec::new();
    loop {
        rest = rest.trim_start();
        if let Some(after) = rest.strip_prefix("/>") {
            return Some(StartTag {
                name,
                attributes,
                closed: true,
                rest: after,
            });
        }
        if let Some(after) = rest.strip_prefix('>') {
            return Some(StartTag {
                name,
                attributes,
                closed: false,
                rest: after,
            });
        }
        let equals = rest.find('=')?;
        let key = rest.get(..equals)?.trim().to_owned();
        let value_text = rest.get(equals + 1..)?.trim_start();
        let quote = value_text
            .chars()
            .next()
            .filter(|c| *c == '"' || *c == '\'')?;
        let value_text = value_text.get(1..)?;
        let close = value_text.find(quote)?;
        attributes.push((key, xml_unescape(value_text.get(..close)?)));
        rest = value_text.get(close + 1..)?;
    }
}

/// The five predefined entities and numeric references.
fn xml_unescape(value: &str) -> String {
    let mut out = String::new();
    let mut rest = value;
    while let Some(amp) = rest.find('&') {
        out.push_str(rest.get(..amp).unwrap_or(""));
        let after = rest.get(amp..).unwrap_or("");
        let Some(semi) = after.find(';') else {
            out.push_str(after);
            return out;
        };
        let entity = after.get(1..semi).unwrap_or("");
        let decoded = match entity {
            "amp" => Some('&'),
            "lt" => Some('<'),
            "gt" => Some('>'),
            "quot" => Some('"'),
            "apos" => Some('\''),
            _ => entity
                .strip_prefix("#x")
                .and_then(|hex| u32::from_str_radix(hex, 16).ok())
                .or_else(|| entity.strip_prefix('#').and_then(|dec| dec.parse().ok()))
                .and_then(char::from_u32),
        };
        match decoded {
            Some(character) => out.push(character),
            None => out.push_str(after.get(..=semi).unwrap_or("")),
        }
        rest = after.get(semi + 1..).unwrap_or("");
    }
    out.push_str(rest);
    out
}

/// `OpenStreetMapProvider.GetPoint(keywords, out status)`: the first place the geocoder finds,
/// fetched through `fetch` - which is `GetContentUsingHttp`, and whose failure is the exception
/// `GetLatLngFromGeocoderUrl` turns into `ExceptionInCode`. (GMap's geocoder cache is not kept:
/// every search goes to the geocoder.)
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/OpenStreetMap/OpenStreetMapProvider.cs:164-185, 286-367`
pub fn geocode<E>(
    keywords: &str,
    fetch: impl FnOnce(&str) -> Result<String, E>,
) -> (GeocoderStatus, Option<LatLon>) {
    let Ok(page) = fetch(&geocoder_url(keywords)) else {
        return (GeocoderStatus::ExceptionInCode, None);
    };
    if page.is_empty() {
        return (GeocoderStatus::Unknow, None);
    }
    let (status, points) = parse_geocoder(&page);
    (status, points.first().copied())
}

// ---------------------------------------------------------------------------------------------
// Markers drawn as the C# draws them: GMap's Google-style pins.
// ---------------------------------------------------------------------------------------------

/// Where a `GMarkerGoogle` pin's head is, from its point. The pin bitmaps are 32 x 32 and drawn
/// with `Offset = (-Size.Width / 2 + 1, -Size.Height + 1)`, so their pixel (15, 31) sits on the
/// position; the head is the circle about pixel (15, 9), nine and a half pixels round.
/// `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/Markers/GMarkerGoogle.cs:111-127; ExtLibs/GMap.NET.Drawing/Resources/green.png`
pub const PIN_HEAD: (f32, f32) = (0.0, -22.0);
/// The head's radius, to the outside of its dark edge.
pub const PIN_RADIUS: f32 = 9.5;
/// Where the taper from the head ends and the three-pixel stalk to the point begins.
pub const PIN_STALK: f32 = -6.0;
/// The pins' dark edge, as the bitmaps have it.
const PIN_EDGE: u32 = 0x04_00_01;
/// `GMarkerGoogleType.green`'s fill, which `GMapMarkerWP` - home and every waypoint - is drawn in.
pub const PIN_GREEN: u32 = 0x00_e1_3c;
/// `GMarkerGoogleType.red`'s fill: the geofence's return location.
pub const PIN_RED: u32 = 0xfc_63_55;
/// Where `GMapMarkerWP` writes its label, from the point: `LocalPosition + (10, 3)`, and
/// `LocalPosition` is the point less the offset above, (-15, -31).
/// `// C#: ExtLibs/Maps/GMapMarkerWP.cs:52-59`
pub const PIN_LABEL: (f32, f32) = (-5.0, -28.0);
/// `SystemFonts.DefaultFont`, 8.25 pt, in pixels at 96 dpi.
const PIN_LABEL_SIZE: f32 = 11.0;

/// A pin's outline, with its point at the origin: from the end of the taper at `tip` up round
/// the head of `radius` and back. The two straight sides are tangents to the head.
#[must_use]
pub fn pin_outline(radius: f32, tip: f32) -> Vec<(f32, f32)> {
    const SEGMENTS: u16 = 24;
    let (cx, cy) = PIN_HEAD;
    let distance = tip - cy;
    if distance <= radius {
        return Vec::new();
    }
    // Straight down is a quarter turn on a screen, where y grows downward. The tangent points are
    // either side of it, and the arc between them goes the long way round, over the top.
    let down = std::f32::consts::FRAC_PI_2;
    let half = (radius / distance).acos();
    let start = down + half;
    let sweep = std::f32::consts::TAU - 2.0 * half;
    let mut points = vec![(cx, tip)];
    for step in 0..=SEGMENTS {
        let angle = start + sweep * f32::from(step) / f32::from(SEGMENTS);
        points.push((cx + radius * angle.cos(), cy + radius * angle.sin()));
    }
    points
}

/// Where `GMapMarkerRallyPt`'s pin has its head, from its point: `marker_02.png` is 24 x 45 and
/// drawn with `Offset = (-10, -40)`, so its pixel (10, 40) sits on the position; the head is the
/// circle about pixel (12, 13), ten and a half pixels round, and the point is at pixel (11, 41).
/// `// C#: ExtLibs/Maps/GMapMarkerRallyPt.cs:17-31, 45-50; Resources/marker_02.png`
pub const RALLY_PIN_HEAD: (f32, f32) = (2.0, -27.0);
/// The rally pin's head radius, to the outside of its dark edge.
pub const RALLY_PIN_RADIUS: f32 = 10.5;
/// The rally pin's point, from the position.
pub const RALLY_PIN_TIP: (f32, f32) = (1.0, 1.0);
/// The hole in the rally pin's head: a ring about pixel (12, 13.5), three pixels round.
pub const RALLY_PIN_HOLE: (f32, f32, f32) = (2.0, -26.5, 3.0);
/// `marker_02`'s fill, the purple most of its opaque pixels are.
pub const RALLY_PIN_FILL: u32 = 0x9b_4d_95;

/// A pin's outline with its head of `radius` at `head` and its point at `tip`: from the point up
/// one tangent, round the head the long way, and down the other.
#[must_use]
pub fn pin_outline_at(head: (f32, f32), radius: f32, tip: (f32, f32)) -> Vec<(f32, f32)> {
    const SEGMENTS: u16 = 24;
    let (cx, cy) = head;
    let (dx, dy) = (tip.0 - cx, tip.1 - cy);
    let distance = dx.hypot(dy);
    if distance <= radius {
        return Vec::new();
    }
    // The tangent points are either side of the direction to the point, and the arc between them
    // goes the long way round, away from it.
    let towards = dy.atan2(dx);
    let half = (radius / distance).acos();
    let start = towards + half;
    let sweep = std::f32::consts::TAU - 2.0 * half;
    let mut points = vec![tip];
    for step in 0..=SEGMENTS {
        let angle = start + sweep * f32::from(step) / f32::from(SEGMENTS);
        points.push((cx + radius * angle.cos(), cy + radius * angle.sin()));
    }
    points
}

/// A filled disc `width` across about `centre`, in a colour with its alpha (`0xRRGGBBAA`).
fn paint_disc(window: &mut Window, centre: Point<Pixels>, width: f32, rgba: u32) {
    let side = px(width);
    window.paint_quad(quad(
        Bounds {
            origin: point(centre.x - side / 2.0, centre.y - side / 2.0),
            size: size(side, side),
        },
        Corners::all(side / 2.0),
        gpui::rgba(rgba),
        gpui::Edges::all(px(0.0)),
        gpui::rgba(rgba),
        gpui::BorderStyle::default(),
    ));
}

/// `camera_icon_G` or `camera_icon` at 20 by 20, centred on its point: a disc with a white
/// camera on it. `// C#: ExtLibs/Maps/GMapMarkerPhoto.cs:15-16, 43-44, 66-70`
fn paint_camera_icon(window: &mut Window, at: Point<Pixels>, disc: u32) {
    let (left, top) = (at.x - px(10.0), at.y - px(10.0));
    let box_at =
        |window: &mut Window, dx: f32, dy: f32, w: f32, h: f32, radius: f32, colour: u32| {
            window.paint_quad(quad(
                Bounds {
                    origin: point(left + px(dx), top + px(dy)),
                    size: size(px(w), px(h)),
                },
                Corners::all(px(radius)),
                rgb(colour),
                gpui::Edges::all(px(0.0)),
                rgb(colour),
                gpui::BorderStyle::default(),
            ));
        };
    box_at(window, 0.0, 0.0, 20.0, 20.0, 10.0, disc);
    box_at(window, 7.0, 5.0, 6.0, 2.0, 0.5, 0xff_ff_ff);
    box_at(window, 5.0, 7.0, 10.0, 8.0, 1.0, 0xff_ff_ff);
    box_at(window, 7.0, 8.0, 6.0, 6.0, 3.0, disc);
    box_at(window, 8.5, 9.5, 3.0, 3.0, 1.5, 0xff_ff_ff);
}

/// Paints `GMapMarkerRallyPt` with its point at `at`: `marker_02`, a purple pin whose head has a
/// hole. The bitmap is drawn here as its shape: the dark edge, the purple inside it, and the hole
/// as the dark ring it is ringed by - the C#'s shows the map through it.
/// `// C#: ExtLibs/Maps/GMapMarkerRallyPt.cs:42-50`
fn paint_rally_pin(window: &mut Window, at: Point<Pixels>) {
    let polygon = |window: &mut Window, points: &[(f32, f32)], colour: u32| {
        let mut builder = PathBuilder::fill();
        let mut points = points.iter();
        let Some((x, y)) = points.next() else {
            return;
        };
        builder.move_to(point(at.x + px(*x), at.y + px(*y)));
        for (x, y) in points {
            builder.line_to(point(at.x + px(*x), at.y + px(*y)));
        }
        builder.close();
        if let Ok(path) = builder.build() {
            window.paint_path(path, Hsla::from(rgb(colour)));
        }
    };
    polygon(
        window,
        &pin_outline_at(RALLY_PIN_HEAD, RALLY_PIN_RADIUS, RALLY_PIN_TIP),
        PIN_EDGE,
    );
    let (tip_x, tip_y) = RALLY_PIN_TIP;
    polygon(
        window,
        &pin_outline_at(
            RALLY_PIN_HEAD,
            RALLY_PIN_RADIUS - 1.5,
            (tip_x + 0.5, tip_y - 2.0),
        ),
        RALLY_PIN_FILL,
    );
    let (hole_x, hole_y, hole_radius) = RALLY_PIN_HOLE;
    window.paint_quad(quad(
        Bounds {
            origin: point(
                at.x + px(hole_x - hole_radius),
                at.y + px(hole_y - hole_radius),
            ),
            size: size(px(hole_radius * 2.0), px(hole_radius * 2.0)),
        },
        Corners::all(px(hole_radius)),
        rgb(PIN_EDGE),
        gpui::Edges::default(),
        rgb(PIN_EDGE),
        gpui::BorderStyle::default(),
    ));
}

/// GMap's zoom level for a view `span` world units across `width` pixels: a map is
/// `256 * 2^zoom` pixels round at `zoom`.
#[must_use]
pub fn gmap_zoom(span: f64, width: f32) -> f64 {
    if span <= 0.0 || width <= 0.0 {
        return 0.0;
    }
    (f64::from(width) / (256.0 * span)).log2()
}

/// Whether `GMapMarkerWP` writes its label at this zoom: the `Overlay.Control.Zoom > 16` half of
/// `Overlay.Control.Zoom > 16 || IsMouseOver`; the painter adds the pointer's half.
/// `// C#: ExtLibs/Maps/GMapMarkerWP.cs:58-59`
#[must_use]
pub fn pin_label_shown(zoom: f64) -> bool {
    zoom > 16.0
}

/// Paints a pin with its point at `at`, and its label if it has one.
fn paint_pin(window: &mut Window, cx: &mut App, at: Point<Pixels>, fill: u32, label: Option<&str>) {
    let polygon = |window: &mut Window, points: &[(f32, f32)], colour: u32| {
        let mut builder = PathBuilder::fill();
        let mut points = points.iter();
        let Some((x, y)) = points.next() else {
            return;
        };
        builder.move_to(point(at.x + px(*x), at.y + px(*y)));
        for (x, y) in points {
            builder.line_to(point(at.x + px(*x), at.y + px(*y)));
        }
        builder.close();
        if let Ok(path) = builder.build() {
            window.paint_path(path, Hsla::from(rgb(colour)));
        }
    };
    // The edge, the stalk, then the fill inside the edge.
    polygon(window, &pin_outline(PIN_RADIUS, PIN_STALK + 1.0), PIN_EDGE);
    window.paint_quad(quad(
        Bounds {
            origin: point(at.x - px(1.5), at.y + px(PIN_STALK)),
            size: size(px(3.0), px(-PIN_STALK)),
        },
        Corners::default(),
        rgb(PIN_EDGE),
        gpui::Edges::default(),
        rgb(PIN_EDGE),
        gpui::BorderStyle::default(),
    ));
    polygon(
        window,
        &pin_outline(PIN_RADIUS - 1.5, PIN_STALK - 0.5),
        fill,
    );

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
    let line = window.text_system().shape_line(
        SharedString::from(text.to_owned()),
        px(PIN_LABEL_SIZE),
        &[run],
        None,
    );
    let origin = point(at.x + px(PIN_LABEL.0), at.y + px(PIN_LABEL.1));
    // A label that will not paint leaves the pin without it, which is what the C# shows below
    // zoom 16 anyway.
    let _ = line.paint(
        origin,
        px(PIN_LABEL_SIZE * 1.2),
        TextAlign::Left,
        None,
        window,
        cx,
    );
}

/// Paints the live map: the vehicle's real flight path, home, and the vehicle itself.
///
/// Screen mapping goes through Web Mercator, the projection tile servers use, so the same
/// transform will place raster tiles when Deliverable 8 adds them.
fn paint_live(map: &mut MapViewport, bounds: Bounds<Pixels>, window: &mut Window, cx: &mut App) {
    let started = Instant::now();
    let origin = bounds.origin;
    let w = f32::from(bounds.size.width);
    let h = f32::from(bounds.size.height);

    map.last_viewport = (w, h);
    map.last_origin = (f32::from(origin.x), f32::from(origin.y));
    // Until the home marker is drawn below, it is not - including by a paint with nothing to frame.
    map.home_label_drawn = false;
    map.tooltips_drawn.clear();
    // A view the user chose wins over the automatic fit; that is what makes panning stick.
    // With nothing to frame and no view chosen, the map is where Mission Planner's is at start:
    // GMap's default position (0, 0), and zoom 3 - the "no zoom in" the C# picks when the saved
    // position rounds to 0 (`GCSViews/FlightData.cs:524-534`). A map without a position could
    // not turn a press into a place, and a script that pressed it recorded nothing
    // (fly-poi.gui, plan-add-below.gui, 2026-09-25).
    let fitted = map.camera.map_or_else(
        || map.view_box().or_else(|| Some(map.idle_view(w, h))),
        |camera| {
            let height = camera.span * f64::from(h) / f64::from(w).max(1.0);
            Some((
                camera.centre.x - camera.span / 2.0,
                camera.centre.y - height / 2.0,
                camera.span,
                height,
            ))
        },
    );
    map.last_view = fitted;
    let Some((vx, vy, vw, vh)) = fitted else {
        // Nothing to frame: no fix, no home, no mission. Draw the empty graticule rather than
        // returning and leaving whatever was underneath. A vehicle on a bench indoors sits in
        // this state for as long as it takes to get outside, and it should look like a map
        // waiting for a position rather than like a panel that failed to paint.
        paint_graticule(origin, w, h, window);
        map.record(started.elapsed());
        return;
    };
    // Projection maths is f64 because Web Mercator near the poles needs the range; screen
    // coordinates are f32 because that is what the renderer takes. The narrowing is deliberate
    // and happens once, here, rather than being scattered through the painter.
    #[allow(clippy::cast_possible_truncation)]
    let to_screen = |p: WebMercator| -> Point<Pixels> {
        point(
            origin.x + px(((p.x - vx) / vw) as f32 * w),
            origin.y + px(((p.y - vy) / vh) as f32 * h),
        )
    };

    // Imagery, or the graticule if none is configured.
    let phase_tiles = Instant::now();
    if !paint_tiles(map, origin, w, h, (vx, vy, vw, vh), window) {
        paint_graticule(origin, w, h, window);
    }
    map.phases[0] = phase_tiles.elapsed();

    // The flown path. Decimated to screen resolution for the reasons measured in ADR 0001, and
    // chunked because a gpui path holds at most 65,535 vertices.
    map.track_paths = 0;
    map.track_path_failures = 0;
    map.drawn_points = 0;
    if map.draws_flown_route() && map.path.len() > 1 {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let budget = (w * POINTS_PER_PIXEL).clamp(2.0, 1_000_000.0) as usize;
        let stride = map.path.len().div_ceil(budget).max(1);
        let mut screen: Vec<Point<Pixels>> = map
            .path
            .iter()
            .step_by(stride)
            .map(|p| to_screen(*p))
            .collect();
        // Keep the newest position so the track always reaches the vehicle - but only if
        // decimation actually dropped it, or the last point gets drawn twice.
        if !map.path.len().saturating_sub(1).is_multiple_of(stride)
            && let Some(last) = map.path.last()
        {
            screen.push(to_screen(*last));
        }
        map.drawn_points = screen.len();

        let submit = Instant::now();
        for chunk in screen.chunks(TRACK_CHUNK) {
            if chunk.len() < 2 {
                continue;
            }
            let mut builder = PathBuilder::stroke(px(2.0));
            let mut points = chunk.iter();
            if let Some(first) = points.next() {
                builder.move_to(*first);
            }
            for p in points {
                builder.line_to(*p);
            }
            match builder.build() {
                Ok(path) => {
                    map.track_paths += 1;
                    window.paint_path(path, Hsla::from(rgb(0x3f_b9_50)));
                }
                Err(_) => map.track_path_failures += 1,
            }
        }
        map.phases[2] = submit.elapsed();
    }

    // The geofence: a boundary, so it is drawn in the alert colour and closed explicitly. It goes
    // under everything else, because it is the thing the rest must stay inside.
    if map.fence.len() > 1 {
        let mut builder = PathBuilder::stroke(px(2.0));
        let mut vertices = map.fence.iter();
        if let Some(first) = vertices.next() {
            builder.move_to(to_screen(*first));
            for vertex in vertices {
                builder.line_to(to_screen(*vertex));
            }
            builder.line_to(to_screen(*first));
        }
        match builder.build() {
            Ok(path) => window.paint_path(path, Hsla::from(rgb(0xf8_51_49))),
            Err(_) => map.track_path_failures += 1,
        }
    }

    // The geofence's exclusion polygons, in the fence's pen.
    for polygon in &map.fence_exclusions {
        if polygon.len() < 2 {
            continue;
        }
        let mut builder = PathBuilder::stroke(px(2.0));
        let mut vertices = polygon.iter();
        if let Some(first) = vertices.next() {
            builder.move_to(to_screen(*first));
            for vertex in vertices {
                builder.line_to(to_screen(*vertex));
            }
            builder.line_to(to_screen(*first));
        }
        match builder.build() {
            Ok(path) => window.paint_path(path, Hsla::from(rgb(0xf8_51_49))),
            Err(_) => map.track_path_failures += 1,
        }
    }

    // The planner's UTM grid, `MainMap_Paint` with `grid` set: the lines `utm_grid` works out
    // for the view's corners, drawn straight between their ends in the selection pen's blue -
    // one pixel wide, the zone's two meridians two - at zoom 10 and closer. Planner only: the
    // flight screen's map has no grid box.
    // `// C#: GCSViews/FlightPlanner.cs:4809-4903; ExtLibs/GMap.NET.WindowsForms/GMapControl.cs:166`
    map.grid_lines_drawn = 0;
    let planner = map.overlay_mode.is_some_and(|overlay| overlay.planner);
    if map.grid && planner {
        let corners = map.view_corners();
        let zoom = map.zoom_level();
        if let (Some((top_left, bottom_right)), Some(zoom)) = (corners, zoom) {
            let lines = mp_mission::utm_grid::grid_lines(
                (top_left.latitude(), top_left.longitude()),
                (bottom_right.latitude(), bottom_right.longitude()),
                zoom,
            );
            for line in &lines {
                let (Ok(from), Ok(to)) = (
                    LatLon::new(line.from.0, line.from.1),
                    LatLon::new(line.to.0, line.to.1),
                ) else {
                    continue;
                };
                let width = if line.boundary { 2.0 } else { 1.0 };
                let mut builder = PathBuilder::stroke(px(width));
                builder.move_to(to_screen(from.to_web_mercator()));
                builder.line_to(to_screen(to.to_web_mercator()));
                match builder.build() {
                    Ok(path) => {
                        window.paint_path(path, Hsla::from(rgb(0x00_00_ff)));
                        map.grid_lines_drawn += 1;
                    }
                    Err(_) => map.track_path_failures += 1,
                }
            }
        }
    }

    // The survey area, drawn first so the grid generated from it sits on top. Closed explicitly
    // rather than relying on the path builder: an area whose last edge is missing looks like an
    // open shape, and an operator would reasonably assume the survey will not cover it.
    if map.polygon.len() > 1 {
        let mut builder = PathBuilder::stroke(px(1.5));
        let mut vertices = map.polygon.iter();
        if let Some(first) = vertices.next() {
            builder.move_to(to_screen(*first));
            for vertex in vertices {
                builder.line_to(to_screen(*vertex));
            }
            builder.line_to(to_screen(*first));
        }
        match builder.build() {
            Ok(path) => window.paint_path(path, Hsla::from(rgb(0xd2_99_22))),
            Err(_) => map.track_path_failures += 1,
        }
    }
    for vertex in &map.polygon {
        let at = to_screen(*vertex);
        window.paint_quad(quad(
            Bounds {
                origin: point(at.x - px(3.0), at.y - px(3.0)),
                size: size(px(6.0), px(6.0)),
            },
            gpui::Corners::all(px(1.0)),
            rgb(0xd2_99_22),
            gpui::Edges::default(),
            rgb(0x00_00_00),
            gpui::BorderStyle::default(),
        ));
    }

    // KML Overlay: `kmlpolygonsoverlay` on the planner, and on the flight screen - once Yes was
    // said - `FlightData.kmlpolygons`, which took the polygons and routes and not the labels.
    // Each shape in its own pen; a polygon closed, its fill transparent.
    // `// C#: GCSViews/FlightPlanner.cs:4246-4262, 4318-4343`
    let planner = map
        .overlay_mode
        .as_ref()
        .is_some_and(|overlay| overlay.planner);
    if planner || map.kml.on_flight {
        let shapes = map
            .kml
            .polygons
            .iter()
            .map(|shape| (shape, true))
            .chain(map.kml.routes.iter().map(|shape| (shape, false)));
        let mut failures = 0;
        for (shape, closed) in shapes {
            if shape.points.len() < 2 {
                continue;
            }
            let mut builder = PathBuilder::stroke(px(shape.width.max(1.0)));
            let mut points = shape.points.iter();
            if let Some(first) = points.next() {
                builder.move_to(to_screen(*first));
                for vertex in points {
                    builder.line_to(to_screen(*vertex));
                }
                if closed {
                    builder.line_to(to_screen(*first));
                }
            }
            // .NET's ARGB to gpui's RGBA.
            let rgba = shape.argb.rotate_left(8);
            match builder.build() {
                Ok(path) => window.paint_path(path, Hsla::from(gpui::rgba(rgba))),
                Err(_) => failures += 1,
            }
        }
        map.track_path_failures += failures;
    }
    if planner {
        // `GMapMarkerKMLLabel.OnRender`: the name in white, ten right and three down of the point,
        // four back when it is wider than fifteen pixels.
        // `// C#: ExtLibs/Maps/GMapMarkerKMLLabel.cs:42-53`
        for label in &map.kml.labels {
            let at = to_screen(label.at);
            let run = TextRun {
                len: label.text.len(),
                font: window.text_style().font(),
                color: Hsla::from(rgb(0xff_ff_ff)),
                background_color: None,
                underline: None,
                strikethrough: None,
            };
            let line = window.text_system().shape_line(
                SharedString::from(label.text.clone()),
                px(KML_LABEL_SIZE),
                &[run],
                None,
            );
            let left = if f32::from(line.width) > 15.0 {
                6.0
            } else {
                10.0
            };
            let _ = line.paint(
                point(at.x + px(left), at.y + px(3.0)),
                px(KML_LABEL_SIZE * 1.2),
                TextAlign::Left,
                None,
                window,
                cx,
            );
        }
    }

    // `GMapMarkerOverlapCount` on `kmlpolygons`: a disc five metres wide at each cell, in the
    // count's colour, those off the screen skipped; then the legend, eight discs of twenty down
    // the map's left from (20, 100), 25 apart, each numbered in white.
    // `// C#: ExtLibs/Maps/GMapMarkerOverlapCount.cs:42-97, 137-177`
    if let Some(cells) = &map.coverage {
        let m2pixel = map.metres_to_pixels().map_or(1.0, |(across, _)| across);
        #[allow(clippy::cast_possible_truncation)] // pixels of a disc
        let width = (5.0 * m2pixel) as f32;
        let screen = Bounds {
            origin,
            size: size(px(w), px(h)),
        };
        for (at, count) in cells {
            let centre = to_screen(*at);
            if !screen.contains(&centre) {
                continue;
            }
            let colour = OVERLAP_COLOURS
                .get(usize::try_from(count.saturating_sub(1)).unwrap_or(7).min(7))
                .copied()
                .unwrap_or(0x8b_00_00);
            paint_disc(window, centre, width, (colour << 8) | OVERLAP_ALPHA);
        }
        for (index, colour) in OVERLAP_COLOURS.iter().enumerate() {
            #[allow(clippy::cast_precision_loss)] // eight discs
            let centre = point(
                origin.x + px(20.0),
                origin.y + px(100.0 + index as f32 * 25.0),
            );
            paint_disc(window, centre, 20.0, (colour << 8) | OVERLAP_ALPHA);
            let text = (index + 1).to_string();
            let run = TextRun {
                len: text.len(),
                font: window.text_style().font(),
                color: Hsla::from(rgb(0xff_ff_ff)),
                background_color: None,
                underline: None,
                strikethrough: None,
            };
            let line = window.text_system().shape_line(
                SharedString::from(text),
                px(KML_LABEL_SIZE),
                &[run],
                None,
            );
            let _ = line.paint(
                point(
                    centre.x - line.width / 2.0,
                    centre.y - px(KML_LABEL_SIZE * 1.2 / 2.0),
                ),
                px(KML_LABEL_SIZE * 1.2),
                TextAlign::Left,
                None,
                window,
                cx,
            );
        }
    }

    // The planned mission: a dashed-looking track plus a marker per waypoint, drawn beneath the
    // flown path so the two are distinguishable where they overlap.
    if map.mission.len() > 1 {
        let mut builder = PathBuilder::stroke(px(1.5));
        let mut points = map.mission.iter();
        if let Some((first, _)) = points.next() {
            builder.move_to(to_screen(*first));
        }
        for (point, _) in points {
            builder.line_to(to_screen(*point));
        }
        match builder.build() {
            Ok(path) => window.paint_path(path, Hsla::from(rgb(0x58_a6_ff))),
            Err(_) => map.track_path_failures += 1,
        }
    }
    for (waypoint, _) in &map.mission {
        let at = to_screen(*waypoint);
        window.paint_quad(quad(
            Bounds {
                origin: point(at.x - px(3.0), at.y - px(3.0)),
                size: size(px(6.0), px(6.0)),
            },
            gpui::Corners::all(px(3.0)),
            rgb(0x58_a6_ff),
            gpui::Edges::default(),
            rgb(0x00_00_00),
            gpui::BorderStyle::default(),
        ));
    }

    // The radius circles: each marker's `GMapMarkerRect`, which `addpolygonmarker` adds after the
    // marker, so it is drawn over it.
    // `// C#: ExtLibs/Maps/WPOverlay.cs:418-434; ExtLibs/Maps/GMapMarkerRect.cs:54-98`
    for circle in map.radius_circles() {
        if !paint_radius_circle(window, &circle, bounds) {
            map.track_path_failures += 1;
        }
    }

    // Rally points: `GMapMarkerRallyPt`, the purple pin `marker_02`, over the mission.
    for rally in &map.rally {
        paint_rally_pin(window, to_screen(*rally));
    }

    // `photosoverlay`: each `GMapMarkerPhoto` as its camera icon - red for a shot sooner than
    // CAM_MIN_INTERVAL after the one before, green otherwise - and its crimson footprint for the
    // last four shots and the one under the pointer, with the tooltip of that one.
    // `// C#: ExtLibs/Maps/GMapMarkerPhoto.cs:65-77; GCSViews/FlightData.cs:4043-4062`
    for (index, photo) in map.photos.iter().enumerate() {
        let at = to_screen(photo.at);
        paint_camera_icon(
            window,
            at,
            if photo.red { CAMERA_RED } else { CAMERA_GREEN },
        );
        let hovered = u16::try_from(index)
            .is_ok_and(|index| map.hovered.pins.contains(&MarkerTag::Photo(index)));
        if (photo.draw_footprint || hovered) && photo.footprint.len() > 1 {
            let mut builder = PathBuilder::stroke(px(1.0));
            let mut corners = photo.footprint.iter();
            if let Some(first) = corners.next() {
                builder.move_to(to_screen(*first));
                for corner in corners {
                    builder.line_to(to_screen(*corner));
                }
                builder.line_to(to_screen(*first));
            }
            if let Ok(path) = builder.build() {
                window.paint_path(path, Hsla::from(rgb(CRIMSON)));
            }
        }
        if hovered {
            paint_tooltip(window, cx, at, &photo.tooltip);
            map.tooltips_drawn.push(photo.tooltip.clone());
        }
    }

    // Other aircraft. Drawn last, over everything else, because a symbol that says where not to
    // fly is worth more than the plan underneath it. A stale report is hollow rather than solid:
    // it was there, and we no longer know that it still is.
    for (position, stale) in &map.traffic {
        let at = to_screen(*position);
        let size = px(10.0);
        let colour = if *stale { 0x8b_94_9e } else { 0xf8_51_49 };
        window.paint_quad(quad(
            Bounds {
                origin: point(at.x - size / 2.0, at.y - size / 2.0),
                size: gpui::size(size, size),
            },
            gpui::Corners::all(size / 2.0),
            // A hollow symbol for a stale one, which is a border with no fill.
            if *stale { rgb(0x00_00_00) } else { rgb(colour) },
            gpui::Edges::all(px(2.0)),
            rgb(colour),
            gpui::BorderStyle::default(),
        ));
    }

    // The geofence's return location: a red pin, `GMarkerGoogleType.red`, whose "GeoFence Return"
    // is a tooltip the C# shows on hover.
    // `// C#: GCSViews/FlightPlanner.cs:6663-6670`
    if let Some(position) = map.fence_return {
        paint_pin(window, cx, to_screen(position), PIN_RED, None);
    }

    // Home: `WPOverlay.CreateOverlay` adds it as `GMapMarkerWP(point, "H")`, a green pin with an
    // "H" written on its head once the map is zoomed in past 16, or while the pointer is on it.
    // `// C#: ExtLibs/Maps/WPOverlay.cs:44-50, 385-398; ExtLibs/Maps/GMapMarkerWP.cs:20-59`
    if let Some(home) = map.home {
        let shown =
            pin_label_shown(gmap_zoom(vw, w)) || map.hovered.pins.contains(&MarkerTag::Home);
        paint_pin(window, cx, to_screen(home), PIN_GREEN, shown.then_some("H"));
        map.home_label_drawn = shown;
    }

    // The tooltips, over the overlay's markers: "Alt: " on each marker under the pointer.
    // `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapOverlay.cs:348-358`
    map.tooltips_drawn.clear();
    for (at, text) in map.hover_tooltips() {
        paint_tooltip(window, cx, to_screen(at), &text);
        map.tooltips_drawn.push(text);
    }

    // The flight screen's Guided Mode marker on `routes`, above the mission: a green
    // `GMarkerGoogle` with `ToolTipMode.Always`. Its rect is among the circles above.
    // `// C#: GCSViews/FlightData.cs:4214-4221; GCSViews/FlightPlanner.cs:1664-1690`
    if let Some(guided) = map.guided {
        let at = to_screen(guided.position.to_web_mercator());
        paint_pin(window, cx, at, PIN_GREEN, None);
        let text = guided.tooltip();
        paint_tooltip(window, cx, at, &text);
        map.tooltips_drawn.push(text);
    }
    // The planner's "Tracker Home", the same `addpolygonmarker` on `routesoverlay`, planner only.
    // `// C#: GCSViews/FlightPlanner.cs:6910-6916`
    if let Some(tracker) = map.tracker
        && map
            .overlay_mode
            .as_ref()
            .is_some_and(|overlay| overlay.planner)
    {
        let at = to_screen(tracker.position.to_web_mercator());
        paint_pin(window, cx, at, PIN_GREEN, None);
        let text = tracker.tooltip();
        paint_tooltip(window, cx, at, &text);
        map.tooltips_drawn.push(text);
    }

    // The vehicle, as Mission Planner's marker for its type draws it.
    let phase_vehicle = Instant::now();
    if let Some((position, heading)) = map.vehicle {
        let at = to_screen(position);
        let m2pixelwidth = map.metres_to_pixels().map(|(across, _)| across);
        #[allow(clippy::cast_possible_truncation)]
        let heading = heading.degrees() as f32;
        paint_vehicle(
            window,
            cx,
            bounds,
            at,
            heading,
            &map.marker,
            &map.marker_settings,
            m2pixelwidth,
        );
    }
    map.phases[3] = phase_vehicle.elapsed();

    map.record(started.elapsed());
}

/// `GMapMarkerQuad`'s green and blue: `ColorFromHex("8dc63f")`, `("00aeef")`.
const QUAD_GREEN: u32 = 0x8d_c6_3f;
/// The quad's blue arm.
const QUAD_BLUE: u32 = 0x00_ae_ef;
/// The sysid's `Color.Red` and the heading line's.
const MARKER_RED: u32 = 0xff_00_00;
/// `Color.Green`, the nav bearing line.
const MARKER_GREEN: u32 = 0x00_80_00;
/// `Color.Orange`, the target line and the quad's warn circle.
const MARKER_ORANGE: u32 = 0xff_a5_00;
/// `Color.HotPink`, the plane's radius arc.
const MARKER_HOT_PINK: u32 = 0xff_69_b4;
/// `GMarkerGoogleType.green_dot`'s green, for a type without a marker.
const MARKER_DOT: u32 = 0x00_c8_00;

/// `GMapMarkerPlane.plane`: the outline, in its 56-wide bitmap's pixels, nose up.
/// `// C#: ExtLibs/Maps/GMapMarkerPlane.cs:15-40`
const PLANE_OUTLINE: [(f32, f32); 22] = [
    (28.0, 0.0),
    (32.0, 13.0),
    (53.0, 27.0),
    (55.0, 32.0),
    (31.0, 28.0),
    (30.0, 35.0),
    (30.0, 43.0),
    (37.0, 48.0),
    (37.0, 50.0),
    (29.0, 50.0),
    (29.0, 53.0),
    (27.0, 53.0),
    (27.0, 50.0),
    (19.0, 50.0),
    (19.0, 48.0),
    (26.0, 43.0),
    (26.0, 35.0),
    (25.0, 28.0),
    (1.0, 32.0),
    (3.0, 27.0),
    (24.0, 13.0),
    (28.0, 0.0),
];

/// The plane's colour by `which`, `sysid - 1`: `which % 7` as the C#'s `int` remainder has it -
/// a `which` below zero matches none of the seven and leaves the plane white.
/// `// C#: ExtLibs/Maps/GMapMarkerPlane.cs:163-178`
#[must_use]
pub fn plane_colour(which: i32) -> u32 {
    match which % 7 {
        0 => MARKER_RED,
        1 => 0x00_00_00,
        2 => 0x00_00_ff,
        3 => 0x0032_cd32,
        4 => 0xff_ff_00,
        5 => MARKER_ORANGE,
        6 => 0xff_c0_cb,
        _ => 0xff_ff_ff,
    }
}

/// A point `(x, y)` in a marker's frame - the point at the origin, y downward - turned
/// `degrees` clockwise about it, as `RotateTransform(heading)` turns the frame.
fn turned(at: Point<Pixels>, (x, y): (f32, f32), degrees: f32) -> Point<Pixels> {
    let (sin, cos) = degrees.to_radians().sin_cos();
    point(
        at.x + px(y.mul_add(-sin, x * cos)),
        at.y + px(y.mul_add(cos, x * sin)),
    )
}

/// A stroked polyline through `points`.
fn stroke_through(window: &mut Window, points: &[Point<Pixels>], width: f32, colour: u32) {
    let mut iter = points.iter();
    let Some(first) = iter.next() else {
        return;
    };
    let mut builder = PathBuilder::stroke(px(width));
    builder.move_to(*first);
    for p in iter {
        builder.line_to(*p);
    }
    if let Ok(path) = builder.build() {
        window.paint_path(path, Hsla::from(rgb(colour)));
    }
}

/// A filled polygon through `points`.
fn fill_through(window: &mut Window, points: &[Point<Pixels>], colour: Hsla) {
    let mut iter = points.iter();
    let Some(first) = iter.next() else {
        return;
    };
    let mut builder = PathBuilder::fill();
    builder.move_to(*first);
    for p in iter {
        builder.line_to(*p);
    }
    builder.line_to(*first);
    if let Ok(path) = builder.build() {
        window.paint_path(path, colour);
    }
}

/// `DrawArc` of the circle in the square at `(left, top)` of side `2 * radius`, from `start`
/// degrees clockwise from the x axis through `sweep` degrees, as the frame's points.
fn arc_points(
    at: Point<Pixels>,
    frame: f32,
    (left, top): (f32, f32),
    radius: f32,
    start: f32,
    sweep: f32,
) -> Vec<Point<Pixels>> {
    let centre = (left + radius, top + radius);
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let segments = ((sweep.abs() / 6.0).ceil() as usize).clamp(1, 120);
    (0..=segments)
        .map(|i| {
            #[allow(clippy::cast_precision_loss)]
            let angle = (start + sweep * i as f32 / segments as f32).to_radians();
            turned(
                at,
                (
                    radius.mul_add(angle.cos(), centre.0),
                    radius.mul_add(angle.sin(), centre.1),
                ),
                frame,
            )
        })
        .collect()
}

/// A text at a point, as `DrawString` puts its top left there.
fn paint_text(
    window: &mut Window,
    cx: &mut App,
    at: Point<Pixels>,
    text: &str,
    size: f32,
    colour: u32,
    monospace: bool,
) {
    let mut font = window.text_style().font();
    font.weight = gpui::FontWeight::BOLD;
    if monospace {
        font.family = "monospace".into();
    }
    let run = TextRun {
        len: text.len(),
        font,
        color: Hsla::from(rgb(colour)),
        background_color: None,
        underline: None,
        strikethrough: None,
    };
    let line = window.text_system().shape_line(
        SharedString::from(text.to_owned()),
        px(size),
        &[run],
        None,
    );
    let _ = line.paint(at, px(size * 1.2), TextAlign::Left, None, window, cx);
}

/// The vehicle's marker at `at`, as its `OnRender` draws it: the bearing lines from the point,
/// each `length` long - the heading red, the nav bearing green, the course black, the target
/// orange - then the vehicle turned to its heading: the quad's four motors, arms and body drawn,
/// the plane's outline filled in its sysid's colour over a shadow, the rover, boat, heli, sub and
/// single copter as their bitmaps, the tracker's icon upright, and a green dot for a type with no
/// marker. The quad adds its sysid and the avoidance radii; the plane its turn radius arc.
///
/// Divergences: the sysid is drawn upright, where the C# turns it with the frame; the dot is a
/// drawn circle, not GMap's bitmap; the map is never rotated here (`Overlay.Control.Bearing` 0).
/// `// C#: ExtLibs/Maps/GMapMarkerQuad.cs:120-254; GMapMarkerPlane.cs:66-187; GMapMarkerRover.cs:33-93;
/// GMapMarkerBoat.cs:30-90; GMapMarkerSub.cs:30-90; GMapMarkerHeli.cs:26-79; GMapMarkerSingle.cs:26-85;
/// GMapMarkerAntennaTracker.cs:22-45`
#[allow(clippy::too_many_arguments)] // the frame, the point, the three readings and the scale
fn paint_vehicle(
    window: &mut Window,
    cx: &mut App,
    clip: Bounds<Pixels>,
    at: Point<Pixels>,
    heading: f32,
    details: &MarkerDetails,
    settings: &MarkerSettings,
    m2pixelwidth: Option<f64>,
) {
    let kind = details.kind;
    let length = if kind == MarkerKind::Tracker {
        500.0
    } else {
        settings.length
    };
    // `DrawLine(pen, 0, 0, cos((b - 90) deg) * length, sin((b - 90) deg) * length)`.
    let line = |window: &mut Window, bearing: f32, colour: u32| {
        let angle = (bearing - 90.0).to_radians();
        let end = point(
            at.x + px(angle.cos() * length),
            at.y + px(angle.sin() * length),
        );
        stroke_through(window, &[at, end], 2.0, colour);
    };
    for which in kind.lines(settings) {
        match which {
            "heading" => line(window, heading, MARKER_RED),
            "nav_bearing" => line(window, details.nav_bearing, MARKER_GREEN),
            "cog" => line(window, details.cog, 0x00_00_00),
            // The quad's `Target` is `cs.nav_bearing`; the others' `cs.target_bearing`.
            "target" => line(
                window,
                if kind == MarkerKind::Quad {
                    details.nav_bearing
                } else {
                    details.target
                },
                MARKER_ORANGE,
            ),
            _ => {}
        }
    }
    let scale = window.scale_factor();
    // A bitmap marker: the image turned about the point `(-width / 2, -width / 2)` puts on the
    // vehicle - the C#'s rectangle, which takes the width for both.
    let bitmap = |window: &mut Window, resource: &'static str, (w, h): (f32, f32)| {
        let pivot = ((w / 2.0).floor(), (w / 2.0).floor());
        let Some((render, side)) =
            crate::pictures::rotated(resource, (w, h), pivot, heading, scale)
        else {
            return;
        };
        let target = Bounds {
            origin: point(at.x - px(side / 2.0), at.y - px(side / 2.0)),
            size: size(px(side), px(side)),
        };
        let _ = window.paint_image(clip, target, Corners::default(), render, 0, false);
    };
    match kind {
        MarkerKind::Quad => {
            let frame = |p: (f32, f32)| turned(at, p, heading);
            // The motors, 20 across with a 5 across hub, at the arms' ends: `(35, 12)`,
            // `(35, 57)`, `(57, 35)` and `(12, 35)` of the 70-pixel icon the point centres.
            for motor in [(0.0, -23.0), (0.0, 22.0), (22.0, 0.0), (-23.0, 0.0)] {
                for radius in [10.0_f32, 2.5] {
                    let ring: Vec<Point<Pixels>> = (0..=24)
                        .map(|i| {
                            #[allow(clippy::cast_precision_loss)]
                            let angle = std::f32::consts::TAU * i as f32 / 24.0;
                            frame((
                                radius.mul_add(angle.cos(), motor.0),
                                radius.mul_add(angle.sin(), motor.1),
                            ))
                        })
                        .collect();
                    stroke_through(window, &ring, 3.0, QUAD_GREEN);
                }
            }
            stroke_through(
                window,
                &[frame((0.0, -23.0)), frame((0.0, 0.0))],
                3.0,
                QUAD_BLUE,
            );
            stroke_through(
                window,
                &[frame((0.0, 1.0)), frame((0.0, 22.0))],
                3.0,
                QUAD_GREEN,
            );
            stroke_through(
                window,
                &[frame((22.0, 0.0)), frame((-23.0, 0.0))],
                3.0,
                QUAD_GREEN,
            );
            fill_through(
                window,
                &[
                    frame((-3.0, -5.0)),
                    frame((2.0, -5.0)),
                    frame((2.0, 3.0)),
                    frame((-3.0, 3.0)),
                ],
                Hsla::from(rgb(QUAD_GREEN)),
            );
            paint_text(
                window,
                cx,
                point(at.x - px(8.0), at.y - px(8.0)),
                &details.sysid.to_string(),
                15.0,
                MARKER_RED,
                true,
            );
            // The avoidance radii: a circle of `m2pixelwidth * 2 * warn` across in orange,
            // `danger`'s in red; nothing for a circle the map's scale makes no pixels of, and
            // nothing after a warn circle that does.
            if let Some(m2pixelwidth) = m2pixelwidth {
                for (metres, colour) in
                    [(details.warn, MARKER_ORANGE), (details.danger, MARKER_RED)]
                {
                    #[allow(clippy::cast_possible_truncation)]
                    let dimension = (m2pixelwidth * f64::from(metres) * 2.0) as i32;
                    if dimension == 0 {
                        break;
                    }
                    if m2pixelwidth > 0.001 && metres > 0.0 {
                        #[allow(clippy::cast_precision_loss)]
                        let radius = dimension as f32 / 2.0;
                        let ring: Vec<Point<Pixels>> = (0..=72)
                            .map(|i| {
                                #[allow(clippy::cast_precision_loss)]
                                let angle = std::f32::consts::TAU * i as f32 / 72.0;
                                point(
                                    at.x + px(radius * angle.cos()),
                                    at.y + px(radius * angle.sin()),
                                )
                            })
                            .collect();
                        stroke_through(window, &ring, 1.0, colour);
                    }
                }
            }
        }
        MarkerKind::Plane => {
            // `DisplayRadius`: the turn the plane is making, HotPink, from its course.
            if settings.radius
                && let Some(m2pixelwidth) = m2pixelwidth
            {
                #[allow(clippy::cast_possible_truncation)]
                let m2pixelwidth = m2pixelwidth as f32;
                let radius = details.radius;
                let alpha = (100.0 * m2pixelwidth / radius).to_degrees();
                let scaled = radius * m2pixelwidth;
                let cog = details.cog;
                if radius < -1.0 && alpha < -1.0 {
                    let p1 = cog.to_radians().cos().mul_add(scaled, scaled);
                    let p2 = cog.to_radians().sin().mul_add(scaled, scaled);
                    let points = arc_points(at, 0.0, (p1, p2), scaled.abs(), cog, alpha);
                    stroke_through(window, &points, 2.0, MARKER_HOT_PINK);
                } else if radius > 1.0 && alpha > 1.0 {
                    let p1 = (cog - 180.0).to_radians().cos().mul_add(scaled, scaled);
                    let p2 = (cog - 180.0).to_radians().sin().mul_add(scaled, scaled);
                    let points = arc_points(at, 0.0, (-p1, -p2), scaled, cog - 180.0, alpha);
                    stroke_through(window, &points, 2.0, MARKER_HOT_PINK);
                }
            }
            let shadow: Vec<Point<Pixels>> = PLANE_OUTLINE
                .iter()
                .map(|(x, y)| turned(at, (x - 26.0, y - 26.0), heading))
                .collect();
            fill_through(window, &shadow, Hsla::from(gpui::rgba(0x0000_0032)));
            let plane: Vec<Point<Pixels>> = PLANE_OUTLINE
                .iter()
                .map(|(x, y)| turned(at, (x - 28.0, y - 28.0), heading))
                .collect();
            fill_through(
                window,
                &plane,
                Hsla::from(rgb(plane_colour(i32::from(details.sysid) - 1))),
            );
        }
        MarkerKind::Rover | MarkerKind::Boat | MarkerKind::Heli | MarkerKind::Sub => {
            if let Some((resource, size)) = kind.icon() {
                bitmap(window, resource, size);
            }
        }
        MarkerKind::Single => {
            if let Some((resource, size)) = kind.icon() {
                bitmap(window, resource, size);
            }
            paint_text(
                window,
                cx,
                point(at.x - px(8.0), at.y - px(8.0)),
                &details.sysid.to_string(),
                15.0,
                MARKER_RED,
                true,
            );
        }
        // `DrawImage(icon, -20, -20, 40, 40)`: upright, whatever the heading.
        MarkerKind::Tracker => {
            if let Some((resource, (w, h))) = kind.icon() {
                let target = Bounds {
                    origin: point(at.x - px(w / 2.0), at.y - px(h / 2.0)),
                    size: size(px(w), px(h)),
                };
                let _ = crate::pictures::paint_stretched(resource, clip, target, window);
            }
        }
        MarkerKind::Dot => {
            let dot: Vec<Point<Pixels>> = (0..=24)
                .map(|i| {
                    #[allow(clippy::cast_precision_loss)]
                    let angle = std::f32::consts::TAU * i as f32 / 24.0;
                    point(at.x + px(6.0 * angle.cos()), at.y + px(6.0 * angle.sin()))
                })
                .collect();
            fill_through(window, &dot, Hsla::from(rgb(MARKER_DOT)));
        }
    }
}

/// Paints one frame of the map into `bounds`.
///
/// Split out from the element so the ordering is explicit: tiles, then track, then markers -
/// painter's order, because gpui composites in submission order within a layer.
fn paint_map(map: &mut MapViewport, bounds: Bounds<Pixels>, window: &mut Window) {
    let started = Instant::now();

    let origin = bounds.origin;
    let w = f32::from(bounds.size.width);
    let h = f32::from(bounds.size.height);

    // 1. Tiles. A real map pays for these as textures through paint_image; here they are quads of
    // the same count and coverage, which measures the compositing cost without a tile server.
    let phase_tiles = Instant::now();
    let tile_w = w / TILE_COLS as f32;
    let tile_h = h / TILE_ROWS as f32;
    for row in 0..TILE_ROWS {
        for col in 0..TILE_COLS {
            let shade = if (row + col) % 2 == 0 {
                0x20_2a_33
            } else {
                0x1c_25_2d
            };
            window.paint_quad(quad(
                Bounds {
                    origin: point(
                        origin.x + px(col as f32 * tile_w),
                        origin.y + px(row as f32 * tile_h),
                    ),
                    size: size(px(tile_w), px(tile_h)),
                },
                gpui::Corners::default(),
                rgb(shade),
                gpui::Edges::default(),
                rgb(0x00_00_00),
                gpui::BorderStyle::default(),
            ));
        }
    }

    map.phases[0] = phase_tiles.elapsed();

    // 2. The flight track, stroked in chunks.
    //
    // A single path for the whole track does not work, and finding that out is the point of this
    // spike: gpui tessellates into `VertexBuffers<_, u16>`, so one path holds at most 65,535
    // vertices. A stroked polyline emits several vertices per point, so a path caps out around
    // 20-30k points - and `PathBuilder::build` returns an error rather than truncating, which is
    // silently skipped by the obvious `if let Ok(..)`. A 100k-point track simply vanishes.
    //
    // So the track is chunked, with one point of overlap so the segments join without a visible
    // seam. The real map (Deliverable 8) needs this anyway for level-of-detail and view culling.
    map.track_paths = 0;
    map.track_path_failures = 0;
    if map.track.len() > 1 {
        let colour = Hsla::from(rgb(0x3f_b9_50));
        let caching = std::env::var("MP_MAP_CACHE").map_or(true, |v| v != "0");

        let cache_valid = caching
            && map
                .cache
                .as_ref()
                .is_some_and(|(cw, ch, _)| (*cw - w).abs() < 0.5 && (*ch - h).abs() < 0.5);

        if !cache_valid {
            // Decimate to screen resolution before tessellating.
            // Clamped to a sane range first, so the conversion is exact for any viewport a
            // display can have.
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let budget = (w * POINTS_PER_PIXEL).clamp(2.0, 1_000_000.0) as usize;
            let stride = map.track.len().div_ceil(budget).max(1);
            let decimated: Vec<(f32, f32)> = if std::env::var("MP_MAP_DECIMATE")
                .map_or(true, |v| v != "0")
            {
                let mut out: Vec<(f32, f32)> = map.track.iter().step_by(stride).copied().collect();
                // Always keep the final point: a track that stops short of the vehicle is wrong.
                if let (Some(last), Some(end)) = (map.track.last(), out.last())
                    && last != end
                {
                    out.push(*last);
                }
                out
            } else {
                map.track.clone()
            };
            map.drawn_points = decimated.len();

            let mut built = Vec::new();
            let mut failures = 0usize;
            for chunk in decimated.chunks(TRACK_CHUNK) {
                if chunk.len() < 2 {
                    continue;
                }
                let mut builder = PathBuilder::stroke(px(1.5));
                let mut points = chunk.iter();
                if let Some((x, y)) = points.next() {
                    builder.move_to(point(origin.x + px(x * w), origin.y + px(y * h)));
                }
                for (x, y) in points {
                    builder.line_to(point(origin.x + px(x * w), origin.y + px(y * h)));
                }
                match builder.build() {
                    Ok(path) => built.push(path),
                    Err(_) => failures += 1,
                }
            }
            map.track_path_failures = failures;
            map.cache = Some((w, h, built));
        }
        map.used_cache = cache_valid;
        if std::env::var("MP_MAP_DEBUG").is_ok() && map.paints.is_multiple_of(60) {
            eprintln!(
                "paint {}: cache={} avg={:.2}ms drawn={}/{} | tiles={:.2} clone={:.2} submit={:.2} markers={:.2}",
                map.paints,
                if cache_valid { "hit" } else { "miss" },
                map.paint_ema.as_secs_f64() * 1000.0,
                map.drawn_points,
                map.track.len(),
                map.phases[0].as_secs_f64() * 1000.0,
                map.phases[1].as_secs_f64() * 1000.0,
                map.phases[2].as_secs_f64() * 1000.0,
                map.phases[3].as_secs_f64() * 1000.0
            );
        }

        if let Some((_, _, paths)) = map.cache.as_ref() {
            map.track_paths = paths.len();
            let mut clone_cost = Duration::ZERO;
            let mut submit_cost = Duration::ZERO;
            for path in paths {
                let t0 = Instant::now();
                let copy = path.clone();
                clone_cost += t0.elapsed();
                let t1 = Instant::now();
                window.paint_path(copy, colour);
                submit_cost += t1.elapsed();
            }
            map.phases[1] = clone_cost;
            map.phases[2] = submit_cost;
        }
    }

    // 3. Markers as small quads.
    let phase_markers = Instant::now();
    for (x, y) in &map.markers {
        window.paint_quad(quad(
            Bounds {
                origin: point(
                    origin.x + px(x * w) - px(2.0),
                    origin.y + px(y * h) - px(2.0),
                ),
                size: size(px(4.0), px(4.0)),
            },
            gpui::Corners::all(px(2.0)),
            rgb(0xd2_99_22),
            gpui::Edges::default(),
            rgb(0x00_00_00),
            gpui::BorderStyle::default(),
        ));
    }

    map.phases[3] = phase_markers.elapsed();
    map.record(started.elapsed());
}

/// Builds the canvas element for the map.
///
/// `size_full` is load-bearing: a canvas has no intrinsic size, so inside a flex container it
/// lays out at zero width and every painted point collapses onto the left edge - which still
/// tessellates, still measures, and still looks like a working renderer in a screenshot.
pub fn map_element(map: std::rc::Rc<std::cell::RefCell<MapViewport>>) -> impl gpui::IntoElement {
    use gpui::Styled as _;
    canvas(
        |_bounds, _window, _cx| (),
        move |bounds: Bounds<Pixels>, (), window: &mut Window, cx: &mut App| {
            let mut map = map.borrow_mut();
            // The synthetic 100k-point scene is for benchmarking the renderer and nothing else;
            // MP_MAP_DEMO=1 selects it. It used to be what you saw whenever there was no fix,
            // which is precisely the situation a real flight controller is in on a bench indoors:
            // connecting to actual hardware filled the map with a hundred thousand points of
            // meaningless green squiggle, which looks like the application is broken.
            if std::env::var("MP_MAP_DEMO").is_ok() {
                paint_map(&mut map, bounds, window);
            } else {
                paint_live(&mut map, bounds, window, cx);
            }
        },
    )
    .size_full()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// With nothing to frame, the map starts where Mission Planner's does: the equator and the
    /// meridian at zoom 3, so a press converts to a place from the first paint.
    #[test]
    fn the_start_view_is_the_origin_at_zoom_3() {
        let (x, y, width, height) = MapViewport::new(0, 0).idle_view(1124.0, 1087.0);
        let centre = LatLon::from_web_mercator(WebMercator {
            x: x + width / 2.0,
            y: y + height / 2.0,
        })
        .expect("a place");
        assert!(centre.latitude().abs() < 1e-9 && centre.longitude().abs() < 1e-9);
        assert!((width - 1124.0 / 2048.0).abs() < 1e-12);
        assert!((height / width - 1087.0 / 1124.0).abs() < 1e-9);
    }

    /// `maplast_*`: the map rests where it was left, at that zoom, once a paint gives it a
    /// size - the current view, so a wheel or a press starts from it - and reads the same place
    /// and zoom back for the next `Deactivate`. It is not a chosen view: the map still follows
    /// the vehicle, as `CHK_autopan` does, so a vehicle heard afterwards is framed.
    /// `// C#: GCSViews/FlightData.cs:524-548, 662-664, 4242-4253`
    #[test]
    fn the_start_position_is_the_idle_view_and_the_map_still_follows() {
        let mut map = MapViewport::new(0, 0);
        assert_eq!(map.current_view(), None, "not painted yet: no view");
        let canberra = LatLon::new(-35.3632621, 149.1652374).expect("Canberra");
        map.start_at(canberra, 16.0);
        map.last_viewport = (800.0, 600.0);
        map.last_origin = (0.0, 0.0);
        let (x, y, width, height) = map.idle_view(800.0, 600.0);
        map.last_view = Some((x, y, width, height));
        assert!(
            map.camera.is_none(),
            "a start position is not a chosen view: it follows"
        );
        let (at, zoom) = map.position_and_zoom().expect("a position and zoom");
        assert!((zoom - 16.0).abs() < 1e-6, "zoom {zoom}");
        assert!((at.latitude() - canberra.latitude()).abs() < 1e-6);
        assert!((at.longitude() - canberra.longitude()).abs() < 1e-6);

        // A press freezes that view: the camera is the idle view, not nothing.
        map.begin_drag(400.0, 300.0);
        let camera = map.camera.expect("the press took hold of the view");
        assert!((gmap_zoom(camera.span, 800.0) - 16.0).abs() < 1e-6);
        map.end_drag();
        map.follow_vehicle();

        // A vehicle heard: the fit frames it, ahead of the idle view.
        map.observe(canberra, Bearing::default());
        map.observe(
            LatLon::new(-35.37, 149.17).expect("near"),
            Bearing::default(),
        );
        assert!(map.camera.is_none(), "still following");
        assert!(map.view_box().is_some(), "the vehicle's track is framed");

        // Not a number: the default's zoom, not a NaN span.
        let mut other = MapViewport::new(0, 0);
        other.start_at(canberra, f64::NAN);
        let (_, _, width, _) = other.idle_view(800.0, 600.0);
        assert!((gmap_zoom(width, 800.0) - 3.0).abs() < 1e-9);
    }

    /// Puts a viewport into the state the painter would leave it in, so camera and projection
    /// behaviour can be tested without a window. The numbers are a 800x600 pane at the origin,
    /// looking at a span of the world around Canberra.
    fn painted(map: &mut MapViewport, span: f64) {
        let centre = LatLon::new(-35.363, 149.165)
            .expect("valid position")
            .to_web_mercator();
        map.last_viewport = (800.0, 600.0);
        map.last_origin = (0.0, 0.0);
        let height = span * 600.0 / 800.0;
        map.last_view = Some((centre.x - span / 2.0, centre.y - height / 2.0, span, height));
        map.camera = Some(Camera { centre, span });
    }

    /// A navigation waypoint at a position, as the plan would produce.
    fn waypoint(seq: u16, at: LatLon) -> MissionItem {
        MissionItem {
            seq,
            current: 0,
            frame: 3,
            command: 16,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: at.latitude(),
            y: at.longitude(),
            z: 50.0,
            autocontinue: 1,
        }
    }

    fn viewport() -> MapViewport {
        // No synthetic scene: these tests are about the camera, not the demo content.
        MapViewport::new(0, 0)
    }

    #[test]
    fn a_fresh_viewport_follows_the_vehicle() {
        let map = viewport();
        assert!(map.is_following());
    }

    #[test]
    fn dragging_takes_control_from_follow_mode() {
        // Otherwise the next telemetry frame snaps the view back and the drag appears to fail.
        let mut map = viewport();
        painted(&mut map, 0.001);
        map.camera = None;
        map.observe(
            LatLon::new(-35.363, 149.165).expect("valid position"),
            Bearing(mp_units::Degrees(0.0)),
        );
        map.begin_drag(400.0, 300.0);
        assert!(!map.is_following());
    }

    #[test]
    fn follow_vehicle_gives_control_back() {
        let mut map = viewport();
        painted(&mut map, 0.001);
        assert!(!map.is_following());
        map.follow_vehicle();
        assert!(map.is_following());
    }

    #[test]
    fn the_map_moves_with_the_pointer() {
        let mut map = viewport();
        painted(&mut map, 0.001);
        let before = map.camera.expect("camera").centre;

        map.begin_drag(400.0, 300.0);
        map.drag_to(500.0, 300.0);
        let after = map.camera.expect("camera").centre;

        // Dragging right moves the world right, so the centre moves left.
        assert!(after.x < before.x, "{before:?} -> {after:?}");
        let expected = 100.0 * 0.001 / 800.0;
        assert!(
            ((before.x - after.x) - expected).abs() < 1e-12,
            "moved {} expected {expected}",
            before.x - after.x
        );
    }

    #[test]
    fn a_drag_without_a_start_does_nothing() {
        let mut map = viewport();
        painted(&mut map, 0.001);
        let before = map.camera.expect("camera").centre;
        map.drag_to(500.0, 300.0);
        assert_eq!(map.camera.expect("camera").centre.x, before.x);
    }

    #[test]
    fn zooming_keeps_the_point_under_the_cursor_still() {
        // The property that separates a map that feels direct from one that must be re-panned
        // after every scroll.
        let mut map = viewport();
        painted(&mut map, 0.001);
        let cursor = (620.0_f32, 180.0_f32);
        let before = map
            .position_at(cursor.0, cursor.1)
            .expect("a position under the cursor");

        map.zoom(cursor.0, cursor.1, 3.0);
        // The painter would refresh this; do it by hand so the inverse projection matches.
        let camera = map.camera.expect("camera");
        let height = camera.span * 600.0 / 800.0;
        map.last_view = Some((
            camera.centre.x - camera.span / 2.0,
            camera.centre.y - height / 2.0,
            camera.span,
            height,
        ));

        let after = map
            .position_at(cursor.0, cursor.1)
            .expect("a position under the cursor");
        assert!(
            (before.latitude() - after.latitude()).abs() < 1e-9,
            "{before:?} -> {after:?}"
        );
        assert!(
            (before.longitude() - after.longitude()).abs() < 1e-9,
            "{before:?} -> {after:?}"
        );
    }

    #[test]
    fn zooming_in_narrows_the_span() {
        let mut map = viewport();
        painted(&mut map, 0.001);
        map.zoom(400.0, 300.0, 1.0);
        assert!(map.camera.expect("camera").span < 0.001);
    }

    #[test]
    fn zoom_cannot_invert_or_swallow_the_world() {
        let mut map = viewport();
        painted(&mut map, 0.001);
        for _ in 0..200 {
            map.zoom(400.0, 300.0, 10.0);
        }
        assert!(map.camera.expect("camera").span > 0.0);
        for _ in 0..200 {
            map.zoom(400.0, 300.0, -10.0);
        }
        assert!(map.camera.expect("camera").span <= 1.5);
    }

    #[test]
    fn the_centre_of_the_viewport_is_the_centre_of_the_view() {
        let mut map = viewport();
        painted(&mut map, 0.001);
        let centre = map.position_at(400.0, 300.0).expect("a position");
        assert!((centre.latitude() - -35.363).abs() < 1e-9, "{centre:?}");
        assert!((centre.longitude() - 149.165).abs() < 1e-9, "{centre:?}");
    }

    #[test]
    fn clicks_are_measured_from_the_viewport_not_the_window() {
        // Mouse events arrive in window coordinates. An uncorrected offset puts every waypoint
        // the same distance from where the operator clicked - too small to notice, too large to
        // fly.
        let mut map = viewport();
        painted(&mut map, 0.001);
        let at_origin = map.position_at(400.0, 300.0).expect("a position");

        map.last_origin = (120.0, 80.0);
        let offset = map.position_at(520.0, 380.0).expect("a position");

        assert!(
            (at_origin.latitude() - offset.latitude()).abs() < 1e-12,
            "{at_origin:?} vs {offset:?}"
        );
        assert!(
            (at_origin.longitude() - offset.longitude()).abs() < 1e-12,
            "{at_origin:?} vs {offset:?}"
        );
    }

    #[test]
    fn a_waypoint_under_the_cursor_is_found() {
        let mut map = viewport();
        painted(&mut map, 0.001);
        let at = LatLon::new(-35.363, 149.165).expect("valid");
        map.set_mission(&[waypoint(7, at)]);

        // The centre of the viewport, where that position was placed.
        assert_eq!(map.waypoint_at(400.0, 300.0), Some(7));
    }

    #[test]
    fn a_click_away_from_every_waypoint_finds_none() {
        // Otherwise a pan that began near a waypoint would drag it instead, which is the worst
        // kind of editing bug: it looks like the map moved.
        let mut map = viewport();
        painted(&mut map, 0.001);
        map.set_mission(&[waypoint(0, LatLon::new(-35.363, 149.165).expect("valid"))]);
        assert_eq!(map.waypoint_at(100.0, 100.0), None);
    }

    #[test]
    fn the_grab_radius_is_generous_compared_with_the_drawn_marker() {
        // The marker is six pixels across and nobody hits a six pixel target while looking at an
        // aircraft.
        let mut map = viewport();
        painted(&mut map, 0.001);
        map.set_mission(&[waypoint(3, LatLon::new(-35.363, 149.165).expect("valid"))]);
        assert_eq!(
            map.waypoint_at(408.0, 308.0),
            Some(3),
            "11px away should hit"
        );
        assert_eq!(map.waypoint_at(430.0, 330.0), None, "42px away should miss");
    }

    #[test]
    fn the_nearest_waypoint_wins_when_two_overlap() {
        // Otherwise whichever happened to be drawn last would win, which changes with the order
        // of the mission rather than with what the operator is pointing at.
        let mut map = viewport();
        painted(&mut map, 0.001);
        let centre = LatLon::new(-35.363, 149.165).expect("valid");
        let nearby = LatLon::new(-35.36305, 149.165).expect("valid");
        map.set_mission(&[waypoint(1, centre), waypoint(2, nearby)]);

        let found = map.waypoint_at(400.0, 300.0).expect("one of them");
        assert_eq!(found, 1, "the one actually under the cursor should win");
    }

    #[test]
    fn hit_testing_before_the_first_paint_finds_nothing() {
        let mut map = viewport();
        map.set_mission(&[waypoint(0, LatLon::new(-35.363, 149.165).expect("valid"))]);
        assert_eq!(map.waypoint_at(400.0, 300.0), None);
    }

    #[test]
    fn screen_position_is_the_inverse_of_position_at() {
        // The two are used together during a drag: one finds the waypoint, the other decides
        // where it lands. If they disagree, a waypoint jumps the moment it is grabbed.
        let mut map = viewport();
        painted(&mut map, 0.001);
        for (x, y) in [(400.0_f32, 300.0_f32), (120.0, 500.0), (700.0, 80.0)] {
            let world = map.position_at(x, y).expect("a position");
            let (back_x, back_y) = map
                .screen_position(world.to_web_mercator())
                .expect("a screen position");
            assert!((back_x - x).abs() < 0.01, "{x} -> {back_x}");
            assert!((back_y - y).abs() < 0.01, "{y} -> {back_y}");
        }
    }

    #[test]
    fn freezing_the_view_stops_the_automatic_fit_moving_it() {
        // Editing must not move the ground under the cursor: place a waypoint, and the next click
        // has to land where the operator aimed.
        let mut map = viewport();
        painted(&mut map, 0.001);
        map.camera = None;
        map.observe(
            LatLon::new(-35.363, 149.165).expect("valid"),
            Bearing(mp_units::Degrees(0.0)),
        );
        assert!(map.is_following());

        map.freeze_view();
        assert!(!map.is_following(), "the view should now be the operator's");
    }

    #[test]
    fn freezing_an_already_frozen_view_leaves_it_alone() {
        // Otherwise every placed waypoint would re-freeze to a slightly different view and the
        // map would creep.
        let mut map = viewport();
        painted(&mut map, 0.001);
        let before = map.camera.expect("a camera");
        map.freeze_view();
        let after = map.camera.expect("a camera");
        assert!((before.span - after.span).abs() < f64::EPSILON);
        assert!((before.centre.x - after.centre.x).abs() < f64::EPSILON);
    }

    /// Centring moves the view and keeps its zoom, as setting a GMap control's `Position` does.
    #[test]
    fn centring_on_a_position_keeps_the_zoom() {
        let mut map = viewport();
        painted(&mut map, 0.001);
        let target = LatLon::new(-35.37, 149.17).expect("valid position");
        map.centre_on(target);
        let camera = map.camera().expect("a chosen view");
        assert!((camera.span - 0.001).abs() < f64::EPSILON);
        let centre = target.to_web_mercator();
        assert!((camera.centre.x - centre.x).abs() < 1e-12);
        assert!((camera.centre.y - centre.y).abs() < 1e-12);
        assert!(!map.is_following());
    }

    /// With nothing framed yet there is still somewhere to centre.
    #[test]
    fn centring_before_anything_is_framed_uses_the_smallest_fit() {
        let mut map = viewport();
        map.centre_on(LatLon::new(-35.37, 149.17).expect("valid position"));
        assert!(map.camera().is_some_and(|camera| camera.span > 0.0));
    }

    /// Where a position was drawn is the inverse of what is under a point.
    #[test]
    fn a_position_is_drawn_where_a_click_would_find_it() {
        let mut map = viewport();
        painted(&mut map, 0.001);
        let at = LatLon::new(-35.364, 149.166).expect("valid position");
        let (x, y) = map.screen_of(at).expect("painted");
        let back = map.position_at(x, y).expect("painted");
        assert!((back.latitude() - at.latitude()).abs() < 1e-6);
        assert!((back.longitude() - at.longitude()).abs() < 1e-6);
        let centre = LatLon::new(-35.363, 149.165).expect("valid position");
        let (cx, cy) = map.screen_of(centre).expect("painted");
        assert!(
            (cx - 400.0).abs() < 0.5 && (cy - 300.0).abs() < 0.5,
            "{cx},{cy}"
        );
    }

    #[test]
    fn a_click_before_the_first_paint_yields_nothing() {
        // Better than a coordinate derived from a view that has never been shown.
        let map = viewport();
        assert!(map.position_at(400.0, 300.0).is_none());
    }

    fn canberra() -> LatLon {
        LatLon::new(-35.363, 149.165).expect("valid")
    }

    /// `getMAVMarker`'s choice for each `MAV_TYPE`, in the C#'s order of asking: the VTOL types
    /// are planes, the copter frames quads, a helicopter and a single copter their own, a type
    /// with no marker the dot.
    /// `// C#: Common.cs:71-215`
    #[test]
    fn the_marker_is_the_csharps_for_the_vehicles_type() {
        use MarkerKind::{Boat, Dot, Heli, Plane, Quad, Rover, Single, Sub, Tracker};
        assert_eq!(MarkerKind::of(1), Plane);
        for vtol in 19..=25 {
            assert_eq!(MarkerKind::of(vtol), Plane, "MAV_TYPE {vtol}");
        }
        assert_eq!(MarkerKind::of(10), Rover);
        assert_eq!(MarkerKind::of(11), Boat);
        assert_eq!(MarkerKind::of(12), Sub);
        assert_eq!(MarkerKind::of(4), Heli);
        assert_eq!(MarkerKind::of(5), Tracker);
        assert_eq!(MarkerKind::of(3), Single);
        for copter in [2, 13, 14, 15, 29] {
            assert_eq!(MarkerKind::of(copter), Quad, "MAV_TYPE {copter}");
        }
        assert_eq!(MarkerKind::of(0), Dot);
        assert_eq!(MarkerKind::of(6), Dot, "a GCS");
        assert_eq!(MarkerKind::default(), Dot);
    }

    /// Which lines each marker draws, and the Planner page's switches over them: the quad has
    /// no nav bearing line, the plane has all four, the tracker its two whatever the settings.
    #[test]
    fn the_markers_lines_follow_the_settings() {
        let all = MarkerSettings::default();
        assert_eq!(MarkerKind::Quad.lines(&all), ["heading", "cog", "target"]);
        assert_eq!(
            MarkerKind::Plane.lines(&all),
            ["heading", "nav_bearing", "cog", "target"]
        );
        assert_eq!(MarkerKind::Heli.lines(&all), ["heading", "cog", "target"]);
        assert_eq!(MarkerKind::Tracker.lines(&all), ["heading", "target"]);
        assert!(MarkerKind::Dot.lines(&all).is_empty());
        let none = MarkerSettings {
            cog: false,
            heading: false,
            nav_bearing: false,
            target: false,
            ..all
        };
        assert!(MarkerKind::Rover.lines(&none).is_empty());
        assert_eq!(MarkerKind::Tracker.lines(&none), ["heading", "target"]);
        let cog_only = MarkerSettings {
            heading: false,
            nav_bearing: false,
            target: false,
            ..all
        };
        assert_eq!(MarkerKind::Boat.lines(&cog_only), ["cog"]);
    }

    /// The plane's colour is its sysid's: sysid 1 red, 2 black, ... 8 red again; a sysid of 0
    /// matches none of the seven and is white, as the C#'s `-1 % 7` matches none.
    #[test]
    fn the_planes_colour_is_its_sysids() {
        assert_eq!(plane_colour(0), MARKER_RED);
        assert_eq!(plane_colour(1), 0x00_00_00);
        assert_eq!(plane_colour(7), MARKER_RED);
        assert_eq!(plane_colour(-1), 0xff_ff_ff);
    }

    /// A marker frame's point turned to a heading: at heading 90 the nose, which points up at 0,
    /// points east; at 180 down.
    #[test]
    fn a_frames_point_turns_with_the_heading() {
        let at = point(px(100.0), px(100.0));
        let nose = (0.0, -10.0);
        let east = turned(at, nose, 90.0);
        assert!((f32::from(east.x) - 110.0).abs() < 1e-3, "{east:?}");
        assert!((f32::from(east.y) - 100.0).abs() < 1e-3, "{east:?}");
        let south = turned(at, nose, 180.0);
        assert!((f32::from(south.x) - 100.0).abs() < 1e-3, "{south:?}");
        assert!((f32::from(south.y) - 110.0).abs() < 1e-3, "{south:?}");
        // `DrawArc` from 0 through 90 degrees of a circle of radius 10 at the origin's square:
        // from east round to south.
        let arc = arc_points(at, 0.0, (-10.0, -10.0), 10.0, 0.0, 90.0);
        let first = arc.first().expect("a start");
        let last = arc.last().expect("an end");
        assert!(
            (f32::from(first.x) - 110.0).abs() < 1e-3 && (f32::from(first.y) - 100.0).abs() < 1e-3
        );
        assert!(
            (f32::from(last.x) - 100.0).abs() < 1e-3 && (f32::from(last.y) - 110.0).abs() < 1e-3
        );
    }

    /// What the window hands the map is what the facts say: the marker's kind, its lines, its
    /// sysid and the heading it turns to.
    #[test]
    fn the_facts_name_the_marker() {
        let mut map = MapViewport::new(0, 0);
        let get = |map: &MapViewport, key: &str| {
            map.facts()
                .into_iter()
                .find(|(k, _)| *k == key)
                .map(|(_, v)| v)
                .expect(key)
        };
        assert_eq!(get(&map, "map.vehicle.kind"), "dot");
        assert_eq!(get(&map, "map.vehicle.heading"), "none");
        assert_eq!(get(&map, "map.vehicle.lines"), "none");
        map.observe(canberra(), Bearing(mp_units::Degrees(271.6)));
        map.set_marker(
            MarkerDetails {
                kind: MarkerKind::of(2),
                sysid: 1,
                ..MarkerDetails::default()
            },
            MarkerSettings {
                target: false,
                ..MarkerSettings::default()
            },
        );
        assert_eq!(map.marker().kind, MarkerKind::Quad);
        assert_eq!(get(&map, "map.vehicle.kind"), "quad");
        assert_eq!(get(&map, "map.vehicle.heading"), "272");
        assert_eq!(get(&map, "map.vehicle.lines"), "heading,cog");
        assert_eq!(get(&map, "map.vehicle.sysid"), "1");
    }

    /// Home is where the screen put it, and goes when the screen says there is none: the two
    /// screens sharing this map draw different homes, so a stale one must not linger.
    #[test]
    fn home_is_drawn_where_it_was_set_and_goes_when_taken_away() {
        let mut map = viewport();
        assert_eq!(map.home(), None);
        map.set_home(Some(canberra()));
        assert_eq!(map.home(), Some(canberra()));
        assert!(
            map.home.is_some(),
            "the projected home is what the painter draws"
        );
        map.set_home(None);
        assert_eq!(map.home(), None);
        assert!(map.home.is_none());
    }

    /// A home with nothing else on the map is enough to frame it, so the planner can be clicked
    /// before any vehicle has connected.
    #[test]
    fn a_home_alone_frames_the_map() {
        let mut map = viewport();
        assert!(map.view_box().is_none());
        map.set_home(Some(canberra()));
        let (x, y, width, _) = map.view_box().expect("a view");
        let home = canberra().to_web_mercator();
        assert!(home.x > x && home.x < x + width, "home should be in view");
        assert!(home.y > y && home.y < y + width, "home should be in view");
    }

    /// The home the planner saved last time can be a city away from the vehicle that has just
    /// connected. It is drawn, but framing both would shrink them to two dots.
    #[test]
    fn a_home_far_from_the_vehicle_is_not_framed() {
        let mut map = viewport();
        map.observe(canberra(), Bearing(mp_units::Degrees(0.0)));
        let (_, _, alone, _) = map.view_box().expect("a view");
        let brisbane = LatLon::new(-27.5097, 153.0154).expect("valid");
        map.set_home(Some(brisbane));
        let (_, _, with_home, _) = map.view_box().expect("a view");
        assert!(
            (with_home - alone).abs() < 1e-12,
            "a far home moved the fit: {alone} -> {with_home}"
        );
        // Near the vehicle, it is framed.
        map.set_home(Some(LatLon::new(-35.36, 149.17).expect("valid")));
        let (_, _, near, _) = map.view_box().expect("a view");
        assert!(near > alone, "a near home should widen the fit");
    }

    /// Clear Track empties the flown route and keeps the vehicle; the route starts again from its
    /// next position. `// C#: GCSViews/FlightData.cs:1101-1107`
    #[test]
    fn clear_track_empties_the_flown_path_and_recording_starts_again() {
        let mut map = viewport();
        let heading = Bearing(mp_units::Degrees(0.0));
        map.observe(canberra(), heading);
        map.observe(LatLon::new(-35.364, 149.166).expect("valid"), heading);
        map.observe(LatLon::new(-35.365, 149.167).expect("valid"), heading);
        assert_eq!(map.path_len(), 3);

        map.clear_track();
        assert_eq!(map.path_len(), 0);
        assert!(map.has_fix(), "the vehicle is still where it was");

        map.observe(LatLon::new(-35.366, 149.168).expect("valid"), heading);
        assert_eq!(
            map.path_len(),
            1,
            "the next position starts the route again"
        );
    }

    /// A position with a latitude or a longitude of 0 - a GPS before its fix - is no point of the
    /// flown route, as `FlightData`'s `cs.lat != 0 && cs.lng != 0` keeps it out, and no marker
    /// either, as `addMAVMarker` adds none at 0,0 (the owner's report of 2026-10-03: a marker
    /// there anchored the fit to half the world); a marker already placed stays where the fix was.
    /// `// C#: GCSViews/FlightData.cs:962-968, 3793-3797`
    #[test]
    fn a_position_at_zero_is_no_point_of_the_flown_route() {
        let mut map = viewport();
        let heading = Bearing(mp_units::Degrees(0.0));
        for (lat, lng) in [(0.0, 0.0), (0.0, 153.0), (-27.5, 0.0)] {
            map.observe(LatLon::new(lat, lng).expect("valid"), heading);
        }
        assert_eq!(map.path_len(), 0);
        assert!(!map.has_fix(), "no marker at 0,0");
        map.observe(LatLon::new(-27.5134, 153.0095).expect("valid"), heading);
        assert!(map.has_fix());
        map.observe(LatLon::new(0.0, 0.0).expect("valid"), heading);
        assert!(map.has_fix(), "the marker stays where the fix was");
        map.observe(LatLon::new(-27.5140, 153.0100).expect("valid"), heading);
        assert_eq!(
            map.path_len(),
            2,
            "the fixes, and not the 0, 0 between them"
        );
    }

    /// The flown route is the flight screen's: the planner's map draws none.
    #[test]
    fn only_the_flight_screen_draws_the_flown_route() {
        let (planner, _, _) = hover_map(true);
        assert!(!planner.draws_flown_route());
        let (flight, _, _) = hover_map(false);
        assert!(flight.draws_flown_route());
        assert!(
            viewport().draws_flown_route(),
            "no overlay set: the flight screen's default"
        );
    }

    /// The return location is a marker of its own, set and taken away.
    #[test]
    fn the_fence_return_marker_is_set_and_taken_away() {
        let mut map = viewport();
        map.set_fence_return(Some(canberra()));
        assert!(map.fence_return.is_some());
        map.set_fence_return(None);
        assert!(map.fence_return.is_none());
    }

    /// The pin is GMap's 32 x 32 bitmap: its point at the position, its head at the top of the
    /// bitmap, nineteen pixels across at the widest - pixels 6 to 24 of `green.png`.
    #[test]
    fn a_pin_has_the_shape_of_the_gmap_bitmap() {
        let outline = pin_outline(PIN_RADIUS, PIN_STALK + 1.0);
        assert!(outline.len() > 20);
        assert_eq!(
            outline.first(),
            Some(&(0.0, -5.0)),
            "the taper ends over the point"
        );
        let top = outline.iter().map(|p| p.1).fold(f32::MAX, f32::min);
        let left = outline.iter().map(|p| p.0).fold(f32::MAX, f32::min);
        let right = outline.iter().map(|p| p.0).fold(f32::MIN, f32::max);
        let bottom = outline.iter().map(|p| p.1).fold(f32::MIN, f32::max);
        // The bitmap's row 0 is 31 pixels above the point.
        assert!((top - -31.5).abs() < 0.1, "top {top}");
        assert!((right - left - 19.0).abs() < 0.1, "width {}", right - left);
        assert!(bottom <= 0.0, "nothing below the point");
        // The fill sits inside the edge.
        let fill = pin_outline(PIN_RADIUS - 1.5, PIN_STALK - 0.5);
        let fill_top = fill.iter().map(|p| p.1).fold(f32::MAX, f32::min);
        assert!(fill_top > top);
        // A head that does not fit above its tip has no outline rather than a folded one.
        assert!(pin_outline(10.0, PIN_HEAD.1 + 5.0).is_empty());
    }

    /// `GMapMarkerWP` writes its label only past zoom 16, in GMap's zoom, where a 256 pixel tile
    /// covers the world at zero.
    #[test]
    fn the_home_label_shows_past_zoom_16_only() {
        let width = 1024.0_f32;
        let at_zoom = |zoom: i32| f64::from(width) / (256.0 * 2_f64.powi(zoom));
        assert!((gmap_zoom(at_zoom(16), width) - 16.0).abs() < 1e-9);
        assert!(pin_label_shown(gmap_zoom(at_zoom(17), width)));
        assert!(
            !pin_label_shown(gmap_zoom(at_zoom(16), width)),
            "16 is not past 16"
        );
        assert!(!pin_label_shown(gmap_zoom(at_zoom(12), width)));
        assert!(!pin_label_shown(gmap_zoom(0.0, width)), "no view, no label");
    }

    /// The label is where the C# writes it: ten pixels in and three down from the bitmap's corner,
    /// which is on the head.
    #[test]
    fn the_label_is_written_on_the_head() {
        assert_eq!(PIN_LABEL, (-15.0 + 10.0, -31.0 + 3.0));
        let (x, y) = PIN_LABEL;
        assert!((x - PIN_HEAD.0).abs() < PIN_RADIUS);
        assert!((y - PIN_HEAD.1).abs() < PIN_RADIUS);
    }

    /// The automatic fit as it was before the path's rectangle was kept: a scan of every point.
    /// The reference the kept rectangle has to match, and the cost it has to beat.
    fn view_box_by_scanning(map: &MapViewport) -> Option<(f64, f64, f64, f64)> {
        const FAR_AWAY: f64 = 0.02;
        const MIN_SPAN: f64 = 3.5e-6;
        let near = |p: &WebMercator, a: &WebMercator| {
            (p.x - a.x).abs() < FAR_AWAY && (p.y - a.y).abs() < FAR_AWAY
        };
        // A far home is left out of the framing, as the fit leaves it out.
        let home = map
            .home
            .filter(|home| map.vehicle.as_ref().is_none_or(|(v, _)| near(home, v)));
        let anchor = map.vehicle.as_ref().map(|(p, _)| *p).or(home);
        let mission_in_view: Vec<&WebMercator> = map
            .mission
            .iter()
            .map(|(p, _)| p)
            .filter(|p| anchor.is_none_or(|a| near(p, &a)))
            .collect();
        let mut points = map.path.iter().chain(home.iter()).chain(mission_in_view);
        let first = points.next().or(map.vehicle.as_ref().map(|(p, _)| p))?;
        let (mut min_x, mut max_x) = (first.x, first.x);
        let (mut min_y, mut max_y) = (first.y, first.y);
        for p in points.chain(map.vehicle.as_ref().map(|(p, _)| p)) {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }
        let span = (max_x - min_x).max(max_y - min_y).max(MIN_SPAN) * 1.25;
        let (cx, cy) = ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0);
        Some((cx - span / 2.0, cy - span / 2.0, span, span))
    }

    /// A survey flown south of Canberra, `points` long: lanes of a thousand reports each, every
    /// one far enough from the last to be kept, as the link would report them.
    fn fly_survey(map: &mut MapViewport, points: usize) {
        for i in 0..points {
            let (lane, along) = (i / 1000, i % 1000);
            let along = if lane % 2 == 0 { along } else { 999 - along };
            map.observe(
                LatLon::new(
                    -35.45 + lane as f64 * 1e-4 + (i as f64 * 0.37).sin() * 1e-6,
                    149.10 + along as f64 * 1e-4,
                )
                .expect("valid position"),
                Bearing(mp_units::Degrees(90.0)),
            );
        }
    }

    #[test]
    fn the_kept_rectangle_frames_exactly_what_scanning_every_point_did() {
        // Nothing observed: nothing to frame, either way.
        let mut map = viewport();
        assert_eq!(map.view_box(), None);
        assert_eq!(view_box_by_scanning(&map), None);

        // A mission alone, with no vehicle or home to judge distance from.
        let near = LatLon::new(-35.40, 149.12).expect("valid position");
        let far = LatLon::new(39.8, -105.1).expect("valid position");
        map.set_mission(&[waypoint(1, near), waypoint(2, far)]);
        assert_eq!(map.view_box(), view_box_by_scanning(&map));

        // Then a home, then a flight: the far waypoint drops out of the framing, the near one
        // stays in, and the rectangle matches the scan to the last bit at every step.
        map.set_home(Some(LatLon::new(-35.363, 149.165).expect("valid position")));
        assert_eq!(map.view_box(), view_box_by_scanning(&map));
        for points in [1, 2, 1_000, 5_000] {
            fly_survey(&mut map, points);
            assert!(map.view_box().is_some());
            assert_eq!(map.view_box(), view_box_by_scanning(&map), "{points}");
        }
        // A vehicle reporting the same place again adds no point to the path, and the fit still
        // agrees with the scan.
        let length = map.path_len();
        let last = LatLon::from_web_mercator(*map.path.last().expect("flown")).expect("valid");
        map.observe(last, Bearing(mp_units::Degrees(0.0)));
        assert_eq!(map.path_len(), length);
        assert_eq!(map.view_box(), view_box_by_scanning(&map));
    }

    #[test]
    fn following_a_long_track_no_longer_scans_it_every_paint() {
        // PLAN.md §13.3 row 8 measured the fit scan over a million points as nearly all of a
        // following frame's CPU time. With the rectangle kept, the fit costs the same for a
        // million points as for one. The fastest of several tries on each side, so a thread
        // descheduled mid-call cannot decide the result.
        let mut map = viewport();
        fly_survey(&mut map, 1_000_000);
        assert_eq!(map.path_len(), 1_000_000);
        let fastest = |f: &dyn Fn() -> Option<(f64, f64, f64, f64)>, tries: usize| {
            (0..tries)
                .map(|_| {
                    let started = Instant::now();
                    std::hint::black_box(f());
                    started.elapsed()
                })
                .min()
                .unwrap_or_default()
        };
        let kept = fastest(&|| map.view_box(), 50);
        let scanned = fastest(&|| view_box_by_scanning(&map), 5);
        eprintln!("fit over 1,000,000 points: kept rectangle {kept:?}, scanning {scanned:?}");
        assert_eq!(map.view_box(), view_box_by_scanning(&map));
        assert!(
            kept * 100 < scanned,
            "the fit took {kept:?} against the scan's {scanned:?}: it is scanning the track"
        );
    }

    // ---- The zoom: `GMapControl.Zoom`, `Position`, `ZoomAndCenterMarkers` ----

    /// What the painter records for the camera it was given, as `paint_live` does.
    fn repaint(map: &mut MapViewport) {
        let camera = map.camera.expect("camera");
        let (width, height) = map.last_viewport;
        let span_down = camera.span * f64::from(height) / f64::from(width);
        map.last_view = Some((
            camera.centre.x - camera.span / 2.0,
            camera.centre.y - span_down / 2.0,
            camera.span,
            span_down,
        ));
    }

    fn at(lat: f64, lng: f64) -> LatLon {
        LatLon::new(lat, lng).expect("valid position")
    }

    /// An item of any command at a position, with its second and third parameters.
    fn item(seq: u16, command: u16, lat: f64, lng: f64, p2: f64, p3: f64) -> MissionItem {
        MissionItem {
            seq,
            command,
            param2: p2,
            param3: p3,
            x: lat,
            y: lng,
            z: 50.0,
            ..MissionItem::default()
        }
    }

    /// `Zoom` reads back what it was set to, about the same centre, held to 0 to 24.
    #[test]
    fn setting_the_zoom_keeps_the_centre_and_reads_back() {
        let mut map = viewport();
        painted(&mut map, 0.001);
        let centre = map.centre().expect("centre");
        map.set_zoom(17.0);
        assert!((map.zoom_level().expect("zoom") - 17.0).abs() < 1e-9);
        let after = map.centre().expect("centre");
        assert!((after.latitude() - centre.latitude()).abs() < 1e-9);
        assert!((after.longitude() - centre.longitude()).abs() < 1e-9);
        // A 256-pixel world at zoom 0: 800 pixels wide is 800 / (256 * 2^17) of the world.
        let span = map.camera.expect("camera").span;
        assert!((span - 800.0 / (256.0 * 131_072.0)).abs() < 1e-15);
        map.set_zoom(30.0);
        assert!((map.zoom_level().expect("zoom") - GMAP_MAX_ZOOM).abs() < 1e-9);
        map.set_zoom(-3.0);
        assert!((map.zoom_level().expect("zoom") - GMAP_MIN_ZOOM).abs() < 1e-9);
    }

    /// A whole zoom reads back whole, whatever the map's width: `(int) Zoom` of a zoom set to 17
    /// is 17, which Zoom to Mission's `(int) Zoom != maxZoom` relies on.
    #[test]
    fn a_whole_zoom_reads_back_whole() {
        for width in [800.0_f32, 1003.0, 1377.0, 1541.0] {
            let mut map = viewport();
            painted(&mut map, 0.001);
            map.last_viewport = (width, 600.0);
            for zoom in 0..=24 {
                map.set_zoom(f64::from(zoom));
                let read = map.zoom_level().expect("zoom");
                assert!(
                    (read.trunc() - f64::from(zoom)).abs() < f64::EPSILON,
                    "{zoom} read back as {read} at width {width}"
                );
            }
        }
    }

    /// With nothing framed there is no view to zoom.
    #[test]
    fn no_view_no_zoom() {
        let mut map = viewport();
        map.set_zoom(12.0);
        assert_eq!(map.zoom_level(), None);
        assert_eq!(map.centre(), None);
    }

    /// `GetMaxZoomToFitRect` for a square a hundredth of a degree across on the equator, in an
    /// 800 x 600 map: 466 pixels at zoom 16, 932 at 17, and 810 is the most that fits.
    #[test]
    fn the_largest_whole_zoom_that_fits_is_chosen() {
        let rect = LatLngRect::around(&[at(0.005, 0.0), at(-0.005, 0.01)]).expect("rect");
        assert_eq!(max_zoom_to_fit(&rect, 800.0, 600.0), 16);
        // Twice as wide a map takes one more.
        assert_eq!(max_zoom_to_fit(&rect, 1600.0, 1200.0), 17);
        // No height, or no width: half of MaxZoom.
        let flat = LatLngRect::around(&[at(0.0, 0.0), at(0.0, 0.01)]).expect("rect");
        assert_eq!(max_zoom_to_fit(&flat, 800.0, 600.0), 12);
    }

    /// `ZoomAndCenterMarkers`: centred on the middle of the rectangle in degrees, at the zoom
    /// that fits it - which does fit, where the next does not.
    #[test]
    fn zoom_to_fit_frames_the_rectangle_of_the_markers() {
        let mut map = viewport();
        painted(&mut map, 0.001);
        let points = [at(-35.36, 149.16), at(-35.37, 149.18), at(-35.35, 149.17)];
        assert!(map.zoom_to_fit(&points));
        let zoom = map.zoom_level().expect("zoom");
        assert!((zoom - zoom.round()).abs() < 1e-9, "a whole zoom: {zoom}");
        let centre = map.centre().expect("centre");
        assert!((centre.latitude() - -35.36).abs() < 1e-9, "{centre:?}");
        assert!((centre.longitude() - 149.17).abs() < 1e-9, "{centre:?}");
        #[allow(clippy::cast_possible_truncation)]
        let zoom = zoom.round() as i32;
        let fits = |zoom: i32| {
            let (x1, y1) = mercator_pixel(-35.35, 149.16, zoom);
            let (x2, y2) = mercator_pixel(-35.37, 149.18, zoom);
            x2 - x1 <= 810 && y2 - y1 <= 610
        };
        assert!(fits(zoom) && !fits(zoom + 1), "zoom {zoom}");
    }

    /// One marker has no rectangle to fit: zoom 12, centred on it.
    #[test]
    fn zoom_to_fit_one_marker_is_zoom_12() {
        let mut map = viewport();
        painted(&mut map, 0.001);
        assert!(map.zoom_to_fit(&[at(-35.0, 149.0)]));
        assert!((map.zoom_level().expect("zoom") - 12.0).abs() < 1e-9);
        assert!(!map.zoom_to_fit(&[]), "nothing to fit");
    }

    /// `if ((int) Zoom != maxZoom) Zoom = maxZoom`: a view already in the zoom that fits keeps
    /// its fraction.
    #[test]
    fn zoom_to_fit_keeps_a_fraction_of_the_same_whole_zoom() {
        let mut map = viewport();
        painted(&mut map, 0.001);
        map.set_zoom(12.6);
        assert!(map.zoom_to_fit(&[at(-35.0, 149.0)]));
        assert!((map.zoom_level().expect("zoom") - 12.6).abs() < 1e-9);
    }

    /// Zoom to Mission fits home and the overlay's markers, and a map drawing no overlay has none.
    #[test]
    fn zoom_to_mission_fits_home_and_the_markers() {
        let mut map = viewport();
        painted(&mut map, 0.001);
        map.set_home(Some(at(-35.36, 149.16)));
        map.set_mission(&[item(1, 16, -35.40, 149.20, 0.0, 0.0)]);
        assert!(
            !map.zoom_and_centre_markers(),
            "no overlay: the log browser's map"
        );
        map.set_overlay(Some(Overlay::FLIGHT));
        assert_eq!(
            map.overlay_positions(),
            vec![at(-35.36, 149.16), at(-35.40, 149.20)]
        );
        assert!(map.zoom_and_centre_markers());
        let centre = map.centre().expect("centre");
        assert!((centre.latitude() - -35.38).abs() < 1e-9);
        assert!((centre.longitude() - 149.18).abs() < 1e-9);
    }

    // ---- The overlay: `WPOverlay.CreateOverlay`'s markers and their rects ----

    /// Which items get a marker, and what their rects are given.
    #[test]
    fn the_overlay_has_the_markers_create_overlay_makes() {
        let items = [
            item(1, 16, -35.1, 149.1, 0.0, 0.0),   // waypoint: WP radius
            item(2, 18, -35.2, 149.2, 0.0, -60.0), // loiter turns, its own radius
            item(3, 19, -35.3, 149.3, 0.0, 70.0),  // loiter time: the default, whatever p3 says
            item(4, 31, -35.4, 149.4, 80.0, 0.0),  // loiter to alt: its own, from p2
            item(5, 17, -35.5, 0.0, 0.0, 0.0),     // loiter unlimited on one coordinate
            item(6, 82, 0.0, 0.0, 0.0, 0.0),       // spline, drawn even at 0,0
            item(7, 201, -35.7, 149.7, 0.0, 0.0),  // DO_SET_ROI: red, no radius, no Alt
            item(8, 21, 0.0, 0.0, 0.0, 0.0),       // LAND at 0,0: none
            item(9, 21, -35.9, 149.9, 0.0, 0.0),   // LAND with a position: WP radius
            item(10, 20, -35.0, 149.0, 0.0, 0.0),  // RTL: none
            item(11, 177, -35.0, 149.0, 0.0, 0.0), // DO_JUMP: none
            item(12, 16, -35.0, 0.0, 0.0, 0.0),    // a waypoint needs both coordinates
            item(13, 189, -35.3, 149.3, 0.0, 0.0), // DO_LAND_START with a position
            item(14, 22, -35.4, 149.4, 0.0, 0.0),  // takeoff with a position: WP radius
            item(15, 93, -35.4, 149.4, 0.0, 0.0),  // DELAY: none
        ];
        let markers = overlay_markers(&items);
        let summary: Vec<(u16, RectRadius, u32, bool)> = markers
            .iter()
            .map(|marker| {
                let MarkerTag::Item(seq) = marker.tag else {
                    panic!("an item's marker")
                };
                (seq, marker.radius, marker.colour, marker.alt.is_some())
            })
            .collect();
        assert_eq!(
            summary,
            vec![
                (1, RectRadius::Wp, RECT_WHITE, true),
                (2, RectRadius::Loiter(Some(-60.0)), RECT_LIGHT_BLUE, true),
                (3, RectRadius::Loiter(None), RECT_LIGHT_BLUE, true),
                (4, RectRadius::Loiter(Some(80.0)), RECT_LIGHT_BLUE, true),
                (5, RectRadius::Loiter(None), RECT_LIGHT_BLUE, true),
                (6, RectRadius::Wp, RECT_GREEN, true),
                (7, RectRadius::None, RECT_WHITE, false),
                (9, RectRadius::Wp, RECT_WHITE, true),
                (13, RectRadius::Wp, RECT_WHITE, true),
                (14, RectRadius::Wp, RECT_WHITE, true),
            ]
        );
        // Item 0 is home, and home is drawn from home.
        assert!(overlay_markers(&[item(0, 16, -35.0, 149.0, 0.0, 0.0)]).is_empty());
    }

    /// `(int)(LocalPosition.X - D)` truncates toward zero, so the side is `D` rounded up where the
    /// rect starts right of the map's edge and rounded down where it starts left of it.
    #[test]
    fn the_circle_side_truncates_as_the_c_sharp_does() {
        assert_eq!(rect_diameter(100, 1.0, 10.0), 20);
        assert_eq!(rect_diameter(100, 1.0, 10.3), 21);
        assert_eq!(rect_diameter(-50, 1.0, 10.3), 20);
        assert_eq!(rect_diameter(100, 1.0, 0.2), 1);
    }

    /// The planner's circle is its radius at the pixels-per-metre of the map's top edge, and a
    /// zoom in doubles it.
    #[test]
    fn a_wp_radius_circle_is_scaled_to_the_zoom() {
        let mut map = viewport();
        painted(&mut map, 1e-5);
        map.set_overlay(Some(Overlay {
            wp_radius: 30.0,
            loiter_radius: 45.0,
            planner: true,
        }));
        map.set_mission(&[item(1, 16, -35.363, 149.165, 0.0, 0.0)]);
        let circles = map.radius_circles();
        assert_eq!(circles.len(), 1);
        let (x, y, width, _) = map.last_view.expect("painted");
        let left = LatLon::from_web_mercator(WebMercator { x, y }).expect("left");
        let right = LatLon::from_web_mercator(WebMercator { x: x + width, y }).expect("right");
        let per_metre = 800.0 / (gmap_distance_km(left, right) * 1000.0);
        #[allow(clippy::cast_precision_loss)]
        let side = circles[0].diameter as f64;
        assert!(
            (side - 60.0 * per_metre).abs() <= 1.0,
            "{side} px for {} px",
            60.0 * per_metre
        );
        assert!(side > 50.0, "big enough to mean something: {side}");
        assert_eq!(circles[0].colour, RECT_WHITE);
        assert!((circles[0].radius - 30.0).abs() < f64::EPSILON);

        let zoom = map.zoom_level().expect("zoom");
        map.set_zoom(zoom + 1.0);
        repaint(&mut map);
        #[allow(clippy::cast_precision_loss)]
        let zoomed = map.radius_circles()[0].diameter as f64;
        assert!((zoomed - 2.0 * side).abs() <= 3.0, "{side} -> {zoomed}");
    }

    /// The flight screen's overlay has no WP radius and no default loiter radius: only a loiter
    /// with its own radius has a circle, drawn at its size whichever way it turns.
    #[test]
    fn the_flight_map_draws_only_loiters_with_their_own_radius() {
        let mut map = viewport();
        painted(&mut map, 1e-5);
        map.set_mission(&[
            item(1, 16, -35.363, 149.165, 0.0, 0.0),
            item(2, 18, -35.3631, 149.1651, 0.0, -20.0),
            item(3, 19, -35.3632, 149.1652, 0.0, 0.0),
        ]);
        assert!(map.radius_circles().is_empty(), "no overlay, no circles");
        map.set_overlay(Some(Overlay::FLIGHT));
        let circles = map.radius_circles();
        assert_eq!(circles.len(), 1);
        assert_eq!(circles[0].tag, MarkerTag::Item(2));
        assert!((circles[0].radius - 20.0).abs() < f64::EPSILON);
        // The planner's default loiter radius gives the loiter time its circle, and the
        // waypoint its WP radius; a WP radius of 0 draws none.
        map.set_overlay(Some(Overlay {
            wp_radius: 0.0,
            loiter_radius: 45.0,
            planner: true,
        }));
        let radii: Vec<f64> = map
            .radius_circles()
            .iter()
            .map(|circle| circle.radius)
            .collect();
        assert_eq!(radii, vec![20.0, 45.0]);
    }

    /// The flight screen's Guided Mode marker has a blue rect of the saved WP radius, whatever
    /// the mission's overlay draws, and none at a radius of 0.
    #[test]
    fn the_guided_mode_marker_has_a_wp_radius_circle() {
        let mut map = viewport();
        painted(&mut map, 1e-5);
        map.set_overlay(Some(Overlay::FLIGHT));
        map.set_mission(&[item(1, 16, -35.363, 149.165, 0.0, 0.0)]);
        let guided = GuidedMarker {
            tag: "Guided Mode",
            position: at(-35.3625, 149.1657),
            alt: 20,
            wp_radius: 30.0,
        };
        map.set_guided(Some(guided));
        let circles = map.radius_circles();
        assert_eq!(circles.len(), 1);
        assert_eq!(circles[0].tag, MarkerTag::Guided);
        assert_eq!(circles[0].colour, RECT_BLUE);
        assert!((circles[0].radius - 30.0).abs() < f64::EPSILON);
        assert_eq!(guided.tooltip(), "Guided Mode : 20");
        map.set_guided(Some(GuidedMarker {
            wp_radius: 0.0,
            ..guided
        }));
        assert!(map.radius_circles().is_empty());
        map.set_guided(None);
        assert!(map.radius_circles().is_empty());
    }

    // ---- The pointer over the markers: `IsMouseOver`, `OnMarkerEnter` ----

    /// A planning map with home and two waypoints far enough apart not to overlap.
    fn hover_map(planner: bool) -> (MapViewport, (f32, f32), (f32, f32)) {
        let mut map = viewport();
        painted(&mut map, 1e-5);
        map.set_overlay(Some(Overlay {
            wp_radius: 30.0,
            loiter_radius: 45.0,
            planner,
        }));
        map.set_home(Some(at(-35.3640, 149.1640)));
        map.set_home_altitude(Some(584.1));
        map.set_mission(&[
            item(1, 16, -35.363, 149.165, 0.0, 0.0),
            item(2, 16, -35.362, 149.166, 0.0, 0.0),
        ]);
        let one = map.screen_of(at(-35.363, 149.165)).expect("on screen");
        let home = map.screen_of(at(-35.3640, 149.1640)).expect("on screen");
        (map, one, home)
    }

    /// On a waypoint: its rect is entered, its circle goes red on the planning map, and its
    /// "Alt:" tooltip shows; the rect reaches 45 above the point where the pin stops at 31.
    #[test]
    fn the_pointer_on_a_waypoint_enters_its_rect_and_shows_its_altitude() {
        let (mut map, (x, y), _) = hover_map(true);
        let change = map.hover(Some((x, y)));
        assert_eq!(change.entered, vec![MarkerTag::Item(1)]);
        assert!(change.changed);
        let red: Vec<MarkerTag> = map
            .radius_circles()
            .iter()
            .filter(|circle| circle.colour == RECT_RED)
            .map(|circle| circle.tag)
            .collect();
        assert_eq!(red, vec![MarkerTag::Item(1)]);
        let tips: Vec<String> = map
            .hover_tooltips()
            .into_iter()
            .map(|(_, text)| text)
            .collect();
        assert_eq!(tips, vec!["Alt: 50".to_owned()]);
        // Still there: nothing newly entered.
        assert!(map.hover(Some((x, y))).entered.is_empty());
        // Forty above: in the rect, not in the pin - red, and no tooltip.
        let change = map.hover(Some((x, y - 40.0)));
        assert!(change.entered.is_empty());
        assert_eq!(map.hover_tooltips(), Vec::new());
        assert_eq!(
            map.radius_circles()
                .iter()
                .filter(|circle| circle.colour == RECT_RED)
                .count(),
            1
        );
        // Off the map: nothing is under it.
        assert!(map.hover(None).changed);
        assert!(
            map.radius_circles()
                .iter()
                .all(|circle| circle.colour != RECT_RED)
        );
    }

    /// The flight screen's `OnMarkerEnter` only notes the marker: the tooltip shows, the circle
    /// stays its colour.
    #[test]
    fn the_flight_map_does_not_turn_a_rect_red() {
        let (mut map, (x, y), _) = hover_map(false);
        map.hover(Some((x, y)));
        assert!(
            map.radius_circles()
                .iter()
                .all(|circle| circle.colour != RECT_RED)
        );
        assert_eq!(map.hover_tooltips().len(), 1);
    }

    /// Home's pin shows its altitude, the boxes' altitude written `"0"`.
    #[test]
    fn home_under_the_pointer_shows_its_altitude() {
        let (mut map, _, home) = hover_map(true);
        let change = map.hover(Some(home));
        assert_eq!(change.entered, vec![MarkerTag::Home]);
        let tips: Vec<String> = map
            .hover_tooltips()
            .into_iter()
            .map(|(_, text)| text)
            .collect();
        assert_eq!(tips, vec!["Alt: 584".to_owned()]);
        let facts = map.facts();
        let fact = |key: &str| {
            facts
                .iter()
                .find(|(name, _)| *name == key)
                .map(|(_, value)| value.clone())
                .expect("fact")
        };
        assert_eq!(fact("map.hover"), "H");
    }

    /// `writeKML` rebuilds the markers, and a rebuilt marker is not under the pointer until it
    /// moves.
    #[test]
    fn a_changed_mission_leaves_nothing_hovered() {
        let (mut map, (x, y), _) = hover_map(true);
        map.hover(Some((x, y)));
        map.set_mission(&[item(1, 16, -35.363, 149.165, 0.0, 0.0)]);
        assert!(map.hover_tooltips().is_empty());
        // The same mission again changes nothing, and the next move finds it.
        assert_eq!(map.hover(Some((x, y))).entered, vec![MarkerTag::Item(1)]);
        map.set_mission(&[item(1, 16, -35.363, 149.165, 0.0, 0.0)]);
        assert_eq!(map.hover_tooltips().len(), 1);
    }

    /// `double.ToString("0")`.
    #[test]
    fn altitudes_are_written_as_whole_numbers() {
        assert_eq!(format_zero(584.1), "584");
        assert_eq!(format_zero(2.5), "3");
        assert_eq!(format_zero(-2.5), "-3");
        assert_eq!(format_zero(-0.2), "0");
    }

    // ---- Drawing a circle: the runs of it the map can show ----

    #[test]
    fn a_circle_on_screen_is_one_closed_run() {
        let runs = circle_runs((400.0, 300.0), 50.0, (0.0, 0.0, 800.0, 600.0));
        assert_eq!(runs.len(), 1);
        let run = &runs[0];
        let (first, last) = (run[0], run[run.len() - 1]);
        assert!((first.0 - last.0).abs() < 1e-3 && (first.1 - last.1).abs() < 1e-3);
        assert!(run.iter().all(|(x, y)| {
            (((x - 400.0).powi(2) + (y - 300.0).powi(2)).sqrt() - 50.0).abs() < 1e-2
        }));
    }

    #[test]
    fn a_circle_off_screen_is_not_drawn() {
        assert!(circle_runs((-100.0, -100.0), 50.0, (0.0, 0.0, 800.0, 600.0)).is_empty());
        assert!(circle_runs((400.0, 300.0), 0.0, (0.0, 0.0, 800.0, 600.0)).is_empty());
    }

    /// A circle a million pixels round costs its visible arc, not its outline.
    #[test]
    fn an_enormous_circle_is_only_its_visible_arc() {
        let runs = circle_runs((400.0, 1_000_300.0), 1_000_000.0, (0.0, 0.0, 800.0, 600.0));
        let points: usize = runs.iter().map(Vec::len).sum();
        assert!(!runs.is_empty());
        assert!(points < 16, "{points} points");
        // A circle half off the left edge is one arc.
        assert_eq!(
            circle_runs((0.0, 300.0), 100.0, (0.0, 0.0, 800.0, 600.0)).len(),
            1
        );
    }

    // ---- Map Tool > Zoom To's geocoder ----

    /// `MakeGeocoderUrl`: spaces made `+`, and what `System.Uri` escapes escaped.
    #[test]
    fn the_geocoder_url_is_osm_with_the_keywords() {
        assert_eq!(
            geocoder_url("Perth Airport, Australia"),
            "https://nominatim.openstreetmap.org/search?q=Perth+Airport,+Australia&format=xml"
        );
        assert_eq!(
            geocoder_url("Zürich"),
            "https://nominatim.openstreetmap.org/search?q=Z%C3%BCrich&format=xml"
        );
    }

    /// The page from `OpenStreetMapProvider.GetPoints`'s own comment: four places, the first of
    /// which is the answer.
    const NOMINATIM: &str = r#"<?xml version="1.0" encoding="UTF-8" ?>
<searchresults timestamp="Wed, 01 Feb 12 09:46:00 -0500" attribution="Data Copyright OpenStreetMap Contributors, Some Rights Reserved. CC-BY-SA 2.0." querystring="lithuania,vilnius" polygon="false" exclude_place_ids="29446018,53849547,8831058,29614806" more_url="http://open.mapquestapi.com/nominatim/v1/search?format=xml&amp;exclude_place_ids=29446018&amp;q=lithuania%2Cvilnius">
<place place_id="29446018" osm_type="way" osm_id="24598347" place_rank="30" boundingbox="54.6868133544922,54.6879043579102,25.2885360717773,25.2898139953613" lat="54.6873633486028" lon="25.289199818878" display_name="National Museum of Lithuania, 1, Arsenalo g., Vilnius" class="tourism" type="museum"/>
<place place_id="53849547" osm_type="way" osm_id="55469274" place_rank="30" lat="54.6900227236882" lon="25.2683589759401" display_name="Ministry of Foreign Affairs" class="amenity" type="public_building"/>
<place place_id="8831058" osm_type="node" osm_id="836234960" place_rank="30" lat="54.677095" lon="25.2738876" display_name="Railway Museum of Lithuania" class="tourism" type="museum"/>
<place place_id="29614806" osm_type="way" osm_id="24845629" place_rank="30" lat="54.6913385159005" lon="25.2617684209873" display_name="Seimas" class="amenity" type="public_building"/>
</searchresults>"#;

    #[test]
    fn the_geocoder_page_gives_its_places() {
        let (status, points) = parse_geocoder(NOMINATIM);
        assert_eq!(status, GeocoderStatus::Success);
        assert_eq!(points.len(), 4);
        assert!((points[0].latitude() - 54.687_363_348_602_8).abs() < 1e-12);
        assert!((points[0].longitude() - 25.289_199_818_878).abs() < 1e-12);
    }

    /// Not a page of places is `Unknow`; one that does not read, or a latitude that is not a
    /// number, is `ExceptionInCode`; a place ranked below 0 is passed over; a `place` that is not
    /// directly under `searchresults` is not one; and a page of places with none that count is
    /// still a success.
    #[test]
    fn the_geocoder_page_fails_as_the_c_sharp_does() {
        assert_eq!(parse_geocoder("").0, GeocoderStatus::Unknow);
        assert_eq!(parse_geocoder("[]").0, GeocoderStatus::Unknow);
        assert_eq!(
            parse_geocoder("<?xml version=\"1.0\"?><searchresults></searchresults>").0,
            GeocoderStatus::Unknow
        );
        assert_eq!(
            parse_geocoder("<?xml version=\"1.0\"?><searchresults><place lat=\"1\"").0,
            GeocoderStatus::ExceptionInCode
        );
        assert_eq!(
            parse_geocoder(
                "<?xml version=\"1.0\"?><searchresults><place lat=\"north\" lon=\"2\"/></searchresults>"
            )
            .0,
            GeocoderStatus::ExceptionInCode
        );
        let (status, points) = parse_geocoder(
            "<?xml version=\"1.0\"?><searchresults><place place_rank=\"-1\" lat=\"1\" lon=\"2\"/><place place_rank=\"x\" lat=\"3\" lon=\"4\"/><more><place lat=\"5\" lon=\"6\"/></more></searchresults>",
        );
        assert_eq!(status, GeocoderStatus::Success);
        assert_eq!(points, vec![at(3.0, 4.0)]);
        let (status, points) =
            parse_geocoder("<?xml version=\"1.0\"?><error><place lat=\"1\" lon=\"2\"/></error>");
        assert_eq!((status, points.len()), (GeocoderStatus::Success, 0));
    }

    /// `GetPoint` through the fetch it is given: the URL asked for, the first place, and a
    /// failed request as `ExceptionInCode`.
    #[test]
    fn geocoding_asks_for_the_url_and_takes_the_first_place() {
        let mut asked = String::new();
        let (status, found) = geocode("lithuania vilnius", |url| {
            asked = url.to_owned();
            Ok::<_, ()>(NOMINATIM.to_owned())
        });
        assert_eq!(
            asked,
            "https://nominatim.openstreetmap.org/search?q=lithuania+vilnius&format=xml"
        );
        assert_eq!(status, GeocoderStatus::Success);
        assert!((found.expect("a place").latitude() - 54.687_363_348_602_8).abs() < 1e-12);
        assert_eq!(
            geocode("x", |_| Err::<String, _>("offline")),
            (GeocoderStatus::ExceptionInCode, None)
        );
        assert_eq!(
            geocode("x", |_| Ok::<_, ()>(String::new())),
            (GeocoderStatus::Unknow, None)
        );
        assert_eq!(GeocoderStatus::Unknow.to_string(), "Unknow");
        assert_eq!(
            GeocoderStatus::ExceptionInCode.to_string(),
            "ExceptionInCode"
        );
    }

    /// The facts a script reads: zoom and centre, circles, hover.
    #[test]
    fn the_map_publishes_its_zoom_circles_and_hover() {
        let (mut map, (x, y), _) = hover_map(true);
        map.set_zoom(18.0);
        repaint(&mut map);
        map.hover(Some((x, y)));
        let facts = map.facts();
        let fact = |key: &str| {
            facts
                .iter()
                .find(|(name, _)| *name == key)
                .map(|(_, value)| value.clone())
                .unwrap_or_else(|| panic!("no fact {key}"))
        };
        assert_eq!(fact("map.zoom"), "18.00");
        assert_eq!(fact("map.zoom.step"), "18");
        assert_eq!(fact("map.centre"), "-35.3630000,149.1650000");
        assert_eq!(fact("map.circles"), "2");
        assert_eq!(fact("map.circles.radii"), "30,30");
        assert_eq!(fact("map.hover"), "1");
        assert_eq!(fact("map.tooltip"), "none", "nothing painted yet");
        let fresh = viewport();
        let facts = fresh.facts();
        assert!(facts.contains(&("map.zoom", "none".to_owned())));
        assert!(facts.contains(&("map.circles", "0".to_owned())));
        assert!(facts.contains(&("map.hover", "none".to_owned())));
    }
}

/// `WPOverlay.pointlist` (PLAN.md §13.4 row 35): what `CreateOverlay` lists for the Elevation
/// Graph, entry by entry, and the altitude `GetHomeAlt` puts each at.
#[cfg(test)]
mod point_list_tests {
    use super::*;
    use crate::srtm::{AltResponse, TileType};
    use mp_mission::rows::Home;

    fn flat(_: f64, _: f64) -> AltResponse {
        AltResponse {
            current_type: TileType::Valid,
            alt: 584.0,
            alt_source: "SRTM",
        }
    }

    fn nothing(_: f64, _: f64) -> AltResponse {
        AltResponse::INVALID
    }

    fn row(command: u16, frame: u8, x: f64, y: f64, z: f64) -> MissionItem {
        MissionItem {
            seq: 0,
            current: 0,
            frame,
            command,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x,
            y,
            z,
            autocontinue: 1,
        }
    }

    const HOME: Home = Home {
        lat: -35.36,
        lng: 149.16,
        alt: 584.1,
    };

    /// Home first, tagged "H"; then a relative row at its altitude plus home's, an absolute one
    /// at its own, a terrain one at its own plus the ground - and -999 where there is none - each
    /// row's altitude the grid's float.
    #[test]
    fn each_frame_puts_a_row_above_sea_level_as_get_home_alt_does() {
        let rows = [
            row(16, 3, -35.361, 149.161, 100.3),
            row(16, 0, -35.362, 149.162, 700.0),
            row(16, 10, -35.363, 149.163, 50.0),
            row(16, 11, -35.364, 149.164, 50.0),
            row(16, 5, -35.365, 149.165, 650.0),
        ];
        let points = point_list(Some(HOME), &rows, &flat);
        let alts: Vec<f64> = points.iter().flatten().map(|point| point.alt).collect();
        assert_eq!(
            alts,
            [
                584.1,
                f64::from(100.3_f32) + 584.1,
                700.0,
                634.0,
                634.0,
                650.0
            ]
        );
        let tags: Vec<&str> = points
            .iter()
            .flatten()
            .map(|point| point.tag.as_str())
            .collect();
        assert_eq!(tags, ["H", "1", "2", "3", "4", "5"]);
        let unknown = point_list(Some(HOME), &rows[2..3], &nothing);
        assert_eq!(
            unknown
                .last()
                .and_then(|point| point.as_ref())
                .map(|point| point.alt),
            Some(-949.0)
        );
    }

    /// No home in the boxes: no "H", and a relative row gets the empty home's 0.
    #[test]
    fn without_a_home_the_list_starts_at_row_one() {
        let points = point_list(None, &[row(16, 3, -35.361, 149.161, 100.0)], &flat);
        assert_eq!(points.len(), 1);
        let first = points.first().and_then(Option::as_ref).expect("a point");
        assert_eq!((first.tag.as_str(), first.alt), ("1", 100.0));
    }

    /// A land at 0,0 is skipped outright; a waypoint or a loiter at 0,0, a DO_JUMP, an RTL and a
    /// command it does not know are `null`; a DO_SET_ROI is "ROI" and its number wherever it is;
    /// a spline at 0,0 is a point; a fence or rally point is a point at 0.
    #[test]
    fn create_overlay_lists_each_command_as_the_c_sharp_does() {
        let rows = [
            row(21, 3, 0.0, 0.0, 0.0),
            row(16, 3, 0.0, 0.0, 100.0),
            row(17, 3, 0.0, 0.0, 100.0),
            row(177, 3, 1.0, 1.0, 0.0),
            row(20, 3, 0.0, 0.0, 0.0),
            row(201, 3, -35.37, 149.17, 20.0),
            row(82, 3, 0.0, 0.0, 30.0),
            row(5100, 3, -35.38, 149.18, 40.0),
            row(183, 3, -35.39, 149.19, 0.0),
            row(189, 3, 0.0, 0.0, 0.0),
            row(189, 3, -35.4, 149.2, 10.0),
        ];
        let points = point_list(None, &rows, &flat);
        let listed: Vec<Option<(&str, f64)>> = points
            .iter()
            .map(|point| point.as_ref().map(|point| (point.tag.as_str(), point.alt)))
            .collect();
        assert_eq!(
            listed,
            [
                None,
                None,
                None,
                None,
                Some(("ROI6", 20.0)),
                Some(("7", 30.0)),
                Some(("8", 0.0)),
                None,
                None,
                Some(("11", 10.0)),
            ]
        );
    }
}

#[cfg(test)]
mod zero_position_tests {
    //! Nothing at latitude 0 or longitude 0 is drawn, run a line to, or framed (the owner's
    //! report of 2026-10-03; the C#'s `lat != 0 && lng != 0` guards).
    use super::*;

    fn at(lat: f64, lng: f64) -> LatLon {
        LatLon::new(lat, lng).expect("a valid position")
    }

    fn waypoint(seq: u16, position: LatLon) -> MissionItem {
        MissionItem {
            seq,
            current: 0,
            frame: 3,
            command: 16,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: position.latitude(),
            y: position.longitude(),
            z: 50.0,
            autocontinue: 1,
        }
    }

    #[test]
    fn a_position_with_a_zero_coordinate_is_not_fixed() {
        assert!(is_fixed(at(-27.4698, 153.0251)));
        assert!(!is_fixed(at(0.0, 0.0)));
        assert!(
            !is_fixed(at(0.0, 153.0251)),
            "the C#'s test is either coordinate"
        );
        assert!(!is_fixed(at(-27.4698, 0.0)));
    }

    #[test]
    fn nothing_at_zero_reaches_a_layer() {
        let brisbane = at(-27.4698, 153.0251);
        let mut map = MapViewport::new(0, 0);
        map.observe(brisbane, Bearing(mp_units::Degrees(90.0)));
        assert!(map.vehicle.is_some());
        let before = map.vehicle;
        // The vehicle's next report at 0,0 or on the equator moves neither the marker nor the path.
        map.observe(at(0.0, 0.0), Bearing(mp_units::Degrees(0.0)));
        map.observe(at(0.0, 153.0251), Bearing(mp_units::Degrees(0.0)));
        assert_eq!(map.vehicle, before, "the marker stays where the fix was");
        assert_eq!(map.path.len(), 1, "no route point at 0,0");
        map.vehicle_unfixed();
        assert!(map.vehicle.is_none());
        assert_eq!(map.path.len(), 1, "the flown route stays");

        map.set_mission(&[
            waypoint(1, brisbane),
            waypoint(2, at(0.0, 0.0)),
            waypoint(3, at(0.0, 153.0)),
        ]);
        assert_eq!(map.mission.len(), 1, "no waypoint and no leg to 0,0");
        map.set_traffic(&[(at(0.0, 0.0), false), (brisbane, false)]);
        assert_eq!(map.traffic.len(), 1);
        map.set_rally(&[at(0.0, 0.0), brisbane]);
        assert_eq!(map.rally.len(), 1);
        map.set_polygon(&[brisbane, at(0.0, 0.0), at(-27.47, 153.03)]);
        assert_eq!(map.polygon.len(), 2);
        map.set_fence(&[brisbane, at(0.0, 0.0)]);
        assert_eq!(map.fence.len(), 1);
        map.set_fence_exclusions(&[vec![brisbane, at(0.0, 0.0), at(-27.47, 153.03)]]);
        assert_eq!(map.fence_exclusions[0].len(), 2);
        map.set_fence_return(Some(at(0.0, 0.0)));
        assert!(map.fence_return.is_none());
        map.set_home(Some(at(0.0, 0.0)));
        assert!(map.home.is_none(), "no H at PointLatLngAlt.Zero");
        map.set_home(Some(brisbane));
        assert!(map.home.is_some());
        map.set_guided(Some(GuidedMarker {
            tag: "Guided Mode",
            position: at(0.0, 0.0),
            alt: 10,
            wp_radius: 30.0,
        }));
        assert!(map.guided.is_none());
    }

    /// The owner's case: a vehicle at Brisbane, then a report at 0,0 - the fit stays on
    /// Brisbane rather than framing both, which is half the world.
    #[test]
    fn a_report_at_zero_does_not_zoom_the_fit_out() {
        let mut map = MapViewport::new(0, 0);
        map.last_viewport = (800.0, 600.0);
        map.observe(at(-27.4698, 153.0251), Bearing(mp_units::Degrees(90.0)));
        let (_, _, span_before, _) = map.view_box().expect("a vehicle frames a view");
        map.observe(at(0.0, 0.0), Bearing(mp_units::Degrees(0.0)));
        map.set_mission(&[waypoint(1, at(0.0, 0.0))]);
        map.set_traffic(&[(at(0.0, 0.0), false)]);
        let (_, _, span_after, _) = map.view_box().expect("still framed");
        assert!(
            (span_after - span_before).abs() < 1e-12,
            "the fit moved: {span_before} to {span_after}"
        );
        assert!(
            span_after < 1e-4,
            "a few hundred metres, not half the world: {span_after}"
        );
        // `zoom_to_fit` over a set that includes 0,0 frames the rest.
        assert!(map.zoom_to_fit(&[at(-27.4698, 153.0251), at(0.0, 0.0), at(-27.47, 153.03)]));
        let span_fit = map.camera.expect("a view chosen").span;
        assert!(span_fit < 1e-3, "Zoom to Mission ignores 0,0: {span_fit}");
    }
}
