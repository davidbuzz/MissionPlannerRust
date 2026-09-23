//! Map viewport spike (DELIVERABLES.md D7/D8).
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
//! If the numbers here hold, D8's map needs no fork of gpui. If they do not, the abandon
//! conditions in the plan apply.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use gpui::{
    App, Bounds, Corners, Hsla, Path, PathBuilder, Pixels, Point, RenderImage, Window, canvas,
    point, px, quad, rgb, size,
};
use mp_mission::MissionItem;
use mp_tiles::store::{TileAnswer, TileStore};
use mp_units::{Bearing, LatLon, TileId, WebMercator, tiles};

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
    /// Where the vehicle is now, and which way it is pointing.
    vehicle: Option<(WebMercator, Bearing)>,
    /// The home point, once the vehicle reports one.
    home: Option<WebMercator>,
    /// The planned mission, projected once when it is set rather than every frame.
    mission: Vec<(WebMercator, u16)>,
    /// The survey area being drawn, if any.
    polygon: Vec<WebMercator>,
    /// The geofence, if one is being drawn or has been read back.
    fence: Vec<WebMercator>,
    /// Rally points, where the vehicle goes on a failsafe.
    rally: Vec<WebMercator>,
    /// Other aircraft, with whether their report is recent enough to be trusted.
    traffic: Vec<(WebMercator, bool)>,
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
/// This is stride decimation, which is enough to establish the budget. The real map (D8) needs
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
            vehicle: None,
            home: None,
            mission: Vec::new(),
            polygon: Vec::new(),
            fence: Vec::new(),
            rally: Vec::new(),
            traffic: Vec::new(),
            tiles: None,
            images: HashMap::new(),
            tiles_drawn: 0,
            tiles_approximate: 0,
            tiles_missing: 0,
            camera: None,
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
    /// would grow the path without drawing anything new.
    pub fn observe(&mut self, position: LatLon, heading: Bearing) {
        let projected = position.to_web_mercator();
        self.vehicle = Some((projected, heading));

        // A world-space threshold of 1e-8 is roughly a metre near the equator - small enough to
        // trace a taxi, large enough to reject GPS jitter on a stationary vehicle.
        const MOVED: f64 = 1e-8;
        let moved = self.path.last().is_none_or(|last| {
            (last.x - projected.x).abs() > MOVED || (last.y - projected.y).abs() > MOVED
        });
        if moved {
            self.path.push(projected);
        }
    }

    /// Records the home point.
    pub fn set_home(&mut self, home: LatLon) {
        self.home = Some(home.to_web_mercator());
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
                let position = item.position().ok().flatten()?;
                Some((position.to_web_mercator(), item.seq))
            })
            .collect();
    }

    /// Replaces the survey area shown on the map.
    pub fn set_polygon(&mut self, vertices: &[LatLon]) {
        self.polygon = vertices
            .iter()
            .map(|vertex| vertex.to_web_mercator())
            .collect();
    }

    /// Replaces the geofence shown on the map.
    pub fn set_fence(&mut self, vertices: &[LatLon]) {
        self.fence = vertices
            .iter()
            .map(|vertex| vertex.to_web_mercator())
            .collect();
    }

    /// Replaces the rally points shown on the map.
    pub fn set_rally(&mut self, positions: &[LatLon]) {
        self.rally = positions
            .iter()
            .map(|position| position.to_web_mercator())
            .collect();
    }

    /// Replaces the other aircraft shown on the map.
    pub fn set_traffic(&mut self, traffic: &[(LatLon, bool)]) {
        self.traffic = traffic
            .iter()
            .map(|(position, stale)| (position.to_web_mercator(), *stale))
            .collect();
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
    fn to_viewport(&self, x: f32, y: f32) -> (f32, f32) {
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
        let (x, y, width, height) = self.view_box()?;
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
    #[allow(dead_code)] // surfaced in the status strip when a mission is loaded
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

        let anchor = self.vehicle.as_ref().map(|(p, _)| *p).or(self.home);
        let mission_in_view: Vec<&WebMercator> = self
            .mission
            .iter()
            .map(|(p, _)| p)
            .filter(|p| {
                anchor.is_none_or(|a| (p.x - a.x).abs() < FAR_AWAY && (p.y - a.y).abs() < FAR_AWAY)
            })
            .collect();

        let mut points = self
            .path
            .iter()
            .chain(self.home.iter())
            .chain(mission_in_view);
        let first = points.next().or(self.vehicle.as_ref().map(|(p, _)| p))?;
        let (mut min_x, mut max_x) = (first.x, first.x);
        let (mut min_y, mut max_y) = (first.y, first.y);
        for p in points.chain(self.vehicle.as_ref().map(|(p, _)| p)) {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }

        // About 400 m at the equator; enough context around a parked aircraft to be useful.
        const MIN_SPAN: f64 = 3.5e-6;
        let span = (max_x - min_x).max(max_y - min_y).max(MIN_SPAN) * 1.25;
        let (cx, cy) = ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0);
        Some((cx - span / 2.0, cy - span / 2.0, span, span))
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

    /// How the last paint went: drawn, approximated from a coarser tile, and missing.
    #[must_use]
    pub const fn tile_counts(&self) -> (usize, usize, usize) {
        (self.tiles_drawn, self.tiles_approximate, self.tiles_missing)
    }

    /// What must be shown on screen about where the imagery came from.
    #[must_use]
    pub fn attribution(&self) -> Option<&'static str> {
        self.tiles.as_ref().map(|store| store.source().attribution)
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

/// Paints the live map: the vehicle's real flight path, home, and the vehicle itself.
///
/// Screen mapping goes through Web Mercator, the projection tile servers use, so the same
/// transform will place raster tiles when D8 adds them.
fn paint_live(map: &mut MapViewport, bounds: Bounds<Pixels>, window: &mut Window) {
    let started = Instant::now();
    let origin = bounds.origin;
    let w = f32::from(bounds.size.width);
    let h = f32::from(bounds.size.height);

    map.last_viewport = (w, h);
    map.last_origin = (f32::from(origin.x), f32::from(origin.y));
    // A view the user chose wins over the automatic fit; that is what makes panning stick.
    let fitted = map.camera.map_or_else(
        || map.view_box(),
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
    if map.path.len() > 1 {
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

    // Rally points: diamonds, so they read as somewhere to go rather than as a waypoint on the
    // route. Drawn over the mission, because a failsafe overrides it.
    for rally in &map.rally {
        let at = to_screen(*rally);
        for (extent, colour) in [(8.0_f32, 0x00_00_00), (6.0, 0xd2_99_22)] {
            window.paint_quad(quad(
                Bounds {
                    origin: point(at.x - px(extent / 2.0), at.y - px(extent / 2.0)),
                    size: size(px(extent), px(extent)),
                },
                gpui::Corners::all(px(extent / 2.0)),
                rgb(colour),
                gpui::Edges::default(),
                rgb(0x00_00_00),
                gpui::BorderStyle::default(),
            ));
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

    // Home.
    if let Some(home) = map.home {
        let at = to_screen(home);
        window.paint_quad(quad(
            Bounds {
                origin: point(at.x - px(5.0), at.y - px(5.0)),
                size: size(px(10.0), px(10.0)),
            },
            gpui::Corners::all(px(2.0)),
            rgb(0x58_a6_ff),
            gpui::Edges::all(px(1.0)),
            rgb(0xe6_ed_f3),
            gpui::BorderStyle::Solid,
        ));
    }

    // The vehicle, as an arrow pointing where it is heading. Bearing is clockwise from north and
    // screen y grows downward, so north is -y and east is +x.
    let phase_vehicle = Instant::now();
    if let Some((position, heading)) = map.vehicle {
        let at = to_screen(position);
        #[allow(clippy::cast_possible_truncation)]
        let theta = (heading.degrees() as f32).to_radians();
        let arm = |angle_deg: f32, radius: f32| -> Point<Pixels> {
            let a = theta + angle_deg.to_radians();
            point(at.x + px(a.sin() * radius), at.y - px(a.cos() * radius))
        };

        let mut nose = PathBuilder::fill();
        nose.move_to(arm(0.0, 12.0));
        nose.line_to(arm(140.0, 9.0));
        nose.line_to(arm(180.0, 3.0));
        nose.line_to(arm(-140.0, 9.0));
        nose.line_to(arm(0.0, 12.0));
        if let Ok(path) = nose.build() {
            window.paint_path(path, Hsla::from(rgb(0xf8_51_49)));
        }
    }
    map.phases[3] = phase_vehicle.elapsed();

    map.record(started.elapsed());
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
    // seam. The real map (D8) needs this anyway for level-of-detail and view culling.
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
        move |bounds: Bounds<Pixels>, (), window: &mut Window, _cx: &mut App| {
            let mut map = map.borrow_mut();
            // The synthetic 100k-point scene is for benchmarking the renderer and nothing else;
            // MP_MAP_DEMO=1 selects it. It used to be what you saw whenever there was no fix,
            // which is precisely the situation a real flight controller is in on a bench indoors:
            // connecting to actual hardware filled the map with a hundred thousand points of
            // meaningless green squiggle, which looks like the application is broken.
            if std::env::var("MP_MAP_DEMO").is_ok() {
                paint_map(&mut map, bounds, window);
            } else {
                paint_live(&mut map, bounds, window);
            }
        },
    )
    .size_full()
}

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn a_click_before_the_first_paint_yields_nothing() {
        // Better than a coordinate derived from a view that has never been shown.
        let map = viewport();
        assert!(map.position_at(400.0, 300.0).is_none());
    }
}
