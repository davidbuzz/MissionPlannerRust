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

use std::time::{Duration, Instant};

use gpui::{
    App, Bounds, Hsla, Path, PathBuilder, Pixels, Point, Window, canvas, point, px, quad, rgb, size,
};
use mp_units::{Bearing, LatLon, WebMercator};

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
        }
    }

    /// Points in the track.
    #[must_use]
    pub fn track_len(&self) -> usize {
        self.track.len()
    }

    /// Markers drawn.
    #[must_use]
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
        let mut points = self.path.iter().chain(self.home.iter());
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

/// Paints the live map: the vehicle's real flight path, home, and the vehicle itself.
///
/// Screen mapping goes through Web Mercator, the projection tile servers use, so the same
/// transform will place raster tiles when D8 adds them.
fn paint_live(map: &mut MapViewport, bounds: Bounds<Pixels>, window: &mut Window) {
    let started = Instant::now();
    let origin = bounds.origin;
    let w = f32::from(bounds.size.width);
    let h = f32::from(bounds.size.height);

    let Some((vx, vy, vw, vh)) = map.view_box() else {
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

    // Graticule, standing in for raster tiles until the tile pipeline exists.
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
            // The synthetic 100k-point scene stays available for benchmarking the renderer;
            // MP_MAP_DEMO=1 selects it. Everything else draws the real vehicle.
            if std::env::var("MP_MAP_DEMO").is_ok() || !map.has_fix() {
                paint_map(&mut map, bounds, window);
            } else {
                paint_live(&mut map, bounds, window);
            }
        },
    )
    .size_full()
}
