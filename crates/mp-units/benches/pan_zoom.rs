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

//! The map's per-frame CPU work at Deliverable 8's load: PLAN.md §13.3 item 8, §8.2's map frame budget.
//!
//! D8's DoD is 120 fps - 8.33 ms a frame - with a 1,000,000-point track and 10,000 markers, and
//! §8.2 measures the whole map frame at 2560x1440 on the GPU with `wgpu-profiler`. This measures
//! the part that happens on the CPU before anything is painted, for that viewport and that load:
//! the projection, the tile enumeration, and the camera-relative `f32` conversion of PLAN.md §9.1.
//!
//! **Why here and not in `crates/mp-gui/benches`.** `mp-gui` is a binary crate with no library
//! target, so a bench - an external target - cannot call anything in `mapview.rs`. The pure
//! geometry the map runs on is this crate's, so the bench lives beside it, and the few lines of
//! `mapview.rs` that turn that geometry into a frame are transliterated below with their line
//! numbers. The map's own path is not benchable from outside until `mp-gui` has a library target
//! or those functions move into one; that, and the GPU half of §8.2, are not measured here.
//!
//! The parts, each a criterion bench:
//! - `project_track_1m` - `LatLon::to_web_mercator` over the whole track: paid once as positions
//!   arrive (`MapViewport::observe`), not per frame, but it is the projection's own cost;
//! - `fit_scan_1m` - the automatic fit (`mapview.rs:546-583`), a min/max over every track point,
//!   which runs on every frame while the map follows the vehicle;
//! - `track_decimated` - what `paint_live` converts per frame: the track strided to two points
//!   per horizontal pixel, then camera-relative `f32` (`mapview.rs:840-876`);
//! - `track_full_1m` - the same conversion of every point, which is what §9.1's upload of the
//!   whole track would cost if it were redone per frame;
//! - `markers_10k` - 10,000 markers to screen, as `paint_live` places every mission, rally and
//!   traffic marker;
//! - `tiles_pan` and `tiles_zoom_step` - `zoom_for_span`, `tiles_for_view` and each tile's screen
//!   rectangle (`mapview.rs:705-728`) after a 40-pixel drag and after one scroll step in;
//! - `frame_follow` and `frame_pan` - the whole CPU frame, following (fit scan included) and
//!   panned (the user's camera, no fit).
//!
//! After criterion, the bench runs `frame_follow` 500 times and fails if its 99th percentile is
//! over 8.33 ms: the §8.2 figure, as a floor for the CPU share alone. Skipped in unoptimised builds
//! (`cargo test --benches`), where the number means nothing.
//!
//! Measured on the development machine, an i7-10875H (`cargo bench -p mp-units --bench pan_zoom`,
//! release profile, one thread, other builds running beside it), criterion's median per call:
//!
//! | part               | time     | per frame?              |
//! |--------------------|----------|-------------------------|
//! | project_track_1m   | 18.8 ms  | no - once per point     |
//! | fit_scan_1m        | 3.25 ms  | yes, while following    |
//! | track_decimated    | 25.3 µs  | yes                     |
//! | track_full_1m      | 1.74 ms  | not today (decimated)   |
//! | markers_10k        | 11.8 µs  | yes                     |
//! | tiles_pan          | 4.2 µs   | yes (169 tiles)         |
//! | tiles_zoom_step    | 4.4 µs   | yes                     |
//! | frame_follow       | 2.50 ms  | the following frame     |
//! | frame_pan          | 33.5 µs  | the panned frame        |
//!
//! Gate: `frame_follow` p50 2.63 ms, p99 5.42 ms, against 8.33 ms - inside the budget, but with
//! two thirds of it gone before the GPU draws anything, and nearly all of that is the fit scan,
//! which grows with the track. A panned frame, which skips the fit, is 34 µs. The one number over
//! the budget, `project_track_1m`, is not per-frame work. The timings move by tens of percent
//! with the machine's load; the ratios do not.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
#![allow(clippy::cast_sign_loss)]
#![allow(missing_docs)] // criterion_group! expands to an undocumented pub fn

use std::hint::black_box;
use web_time::{Duration, Instant};

use criterion::{Criterion, criterion_group};
use mp_units::tiles::{TILE_SIZE_PX, tiles_for_view, zoom_for_span};
use mp_units::{LatLon, WebMercator};

/// §8.2's viewport.
const WIDTH: f32 = 2560.0;
const HEIGHT: f32 = 1440.0;

/// Deliverable 8's load.
const TRACK_POINTS: usize = 1_000_000;
const MARKERS: usize = 10_000;

/// 120 fps.
const FRAME_BUDGET: Duration = Duration::from_micros(8_333);

/// Track points kept per horizontal pixel, `mapview.rs:147`.
const POINTS_PER_PIXEL: f32 = 2.0;

/// ArduPilot's SITL home at CMAC, where the survey is flown.
const HOME: (f64, f64) = (-35.363_261, 149.165_230);

/// A lawnmower survey over about 5 km x 5 km: 200 lanes of 5,000 points, alternating direction,
/// with a wobble so no two consecutive points are collinear. What a long survey log looks like.
fn survey_track() -> Vec<LatLon> {
    const LANES: usize = 200;
    let per_lane = TRACK_POINTS / LANES;
    let (lat_span, lng_span) = (0.045, 0.055);
    (0..TRACK_POINTS)
        .map(|i| {
            let (lane, along) = (i / per_lane, (i % per_lane) as f64 / per_lane as f64);
            let along = if lane % 2 == 0 { along } else { 1.0 - along };
            let wobble = ((i as f64) * 0.37).sin() * 2e-6;
            LatLon::new(
                HOME.0 - lat_span / 2.0 + lat_span * lane as f64 / LANES as f64 + wobble,
                HOME.1 - lng_span / 2.0 + lng_span * along,
            )
            .unwrap()
        })
        .collect()
}

/// Markers scattered over the same area.
fn markers() -> Vec<WebMercator> {
    (0..MARKERS)
        .map(|i| {
            let t = i as f64 / MARKERS as f64;
            LatLon::new(
                HOME.0 + ((t * 13.0).sin() * 0.02),
                HOME.1 + ((t * 7.0).fract() - 0.5) * 0.05,
            )
            .unwrap()
            .to_web_mercator()
        })
        .collect()
}

/// A camera: centre and span, as `mapview.rs:116-122` holds it.
#[derive(Clone, Copy)]
struct Camera {
    centre: WebMercator,
    span: f64,
}

/// The displayed rectangle for a camera, `mapview.rs:815-825`: (x, y, width, height).
fn view_of(camera: Camera) -> (f64, f64, f64, f64) {
    let height = camera.span * f64::from(HEIGHT) / f64::from(WIDTH);
    (
        camera.centre.x - camera.span / 2.0,
        camera.centre.y - height / 2.0,
        camera.span,
        height,
    )
}

/// `mapview.rs:840-845`: camera-relative, narrowed to `f32` once, here.
#[inline]
fn to_screen(p: WebMercator, (vx, vy, vw, vh): (f64, f64, f64, f64)) -> (f32, f32) {
    (
        ((p.x - vx) / vw) as f32 * WIDTH,
        ((p.y - vy) / vh) as f32 * HEIGHT,
    )
}

/// The automatic fit, `mapview.rs:546-583`, over the track alone (no vehicle, home or mission,
/// which are a handful of points against a million).
fn fit(path: &[WebMercator]) -> (f64, f64, f64, f64) {
    const MIN_SPAN: f64 = 3.5e-6;
    let first = path[0];
    let (mut min_x, mut max_x, mut min_y, mut max_y) = (first.x, first.x, first.y, first.y);
    for p in path {
        min_x = min_x.min(p.x);
        max_x = max_x.max(p.x);
        min_y = min_y.min(p.y);
        max_y = max_y.max(p.y);
    }
    let span = (max_x - min_x).max(max_y - min_y).max(MIN_SPAN) * 1.25;
    let (cx, cy) = ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0);
    (cx - span / 2.0, cy - span / 2.0, span, span)
}

/// The tiles for a view and each one's screen rectangle, `mapview.rs:705-728`.
fn tiles(view: (f64, f64, f64, f64), out: &mut Vec<(f32, f32, f32, f32)>) {
    let (vx, vy, vw, vh) = view;
    // Capped at the provider's deepest zoom, as mapview.rs:705 caps it; 19 is OpenStreetMap's.
    let zoom = zoom_for_span(vw, WIDTH, TILE_SIZE_PX).min(19);
    out.clear();
    for id in tiles_for_view(
        WebMercator { x: vx, y: vy },
        WebMercator {
            x: vx + vw,
            y: vy + vh,
        },
        zoom,
    ) {
        let (nw, se) = id.bounds();
        let (left, top) = to_screen(nw, view);
        let (right, bottom) = to_screen(se, view);
        out.push((left, top, right - left, bottom - top));
    }
}

/// The track as `paint_live` converts it, `mapview.rs:854-876`: strided to the viewport's width,
/// with the newest point kept.
fn track_decimated(path: &[WebMercator], view: (f64, f64, f64, f64), out: &mut Vec<(f32, f32)>) {
    let budget = (WIDTH * POINTS_PER_PIXEL).clamp(2.0, 1_000_000.0) as usize;
    let stride = path.len().div_ceil(budget).max(1);
    out.clear();
    out.extend(path.iter().step_by(stride).map(|p| to_screen(*p, view)));
    if !path.len().saturating_sub(1).is_multiple_of(stride)
        && let Some(last) = path.last()
    {
        out.push(to_screen(*last, view));
    }
}

fn to_screen_all(points: &[WebMercator], view: (f64, f64, f64, f64), out: &mut Vec<(f32, f32)>) {
    out.clear();
    out.extend(points.iter().map(|p| to_screen(*p, view)));
}

/// Everything the map computes for one frame before painting.
struct Frame {
    tiles: Vec<(f32, f32, f32, f32)>,
    track: Vec<(f32, f32)>,
    markers: Vec<(f32, f32)>,
}

impl Frame {
    fn new() -> Self {
        Self {
            tiles: Vec::with_capacity(512),
            track: Vec::with_capacity(TRACK_POINTS),
            markers: Vec::with_capacity(MARKERS),
        }
    }

    /// One frame: the fit if following, then tiles, track and markers.
    fn run(&mut self, path: &[WebMercator], markers: &[WebMercator], camera: Option<Camera>) {
        let view = camera.map_or_else(|| fit(path), view_of);
        tiles(view, &mut self.tiles);
        track_decimated(path, view, &mut self.track);
        to_screen_all(markers, view, &mut self.markers);
    }
}

/// The camera after `mapview.rs`'s drag of `dx` pixels (`drag_to`, :392-405).
fn panned(camera: Camera, dx: f32) -> Camera {
    let per_pixel = camera.span / f64::from(WIDTH);
    Camera {
        centre: WebMercator {
            x: camera.centre.x - f64::from(dx) * per_pixel,
            y: camera.centre.y,
        },
        span: camera.span,
    }
}

/// The camera after one scroll step in at the viewport's centre (`zoom`, :422-446).
fn zoomed(camera: Camera) -> Camera {
    let factor = f64::from(1.2_f32.powf(-1.0));
    Camera {
        centre: camera.centre,
        span: (camera.span * factor).clamp(1e-9, 1.5),
    }
}

fn pan_zoom(c: &mut Criterion) {
    let track = survey_track();
    let path: Vec<WebMercator> = track.iter().map(|p| p.to_web_mercator()).collect();
    let markers = markers();
    let fitted = fit(&path);
    let camera = Camera {
        centre: WebMercator {
            x: fitted.0 + fitted.2 / 2.0,
            y: fitted.1 + fitted.3 / 2.0,
        },
        span: fitted.2,
    };
    let view = view_of(camera);

    let mut group = c.benchmark_group("pan_zoom");
    group.bench_function("project_track_1m", |b| {
        let mut out = Vec::with_capacity(TRACK_POINTS);
        b.iter(|| {
            out.clear();
            out.extend(black_box(&track).iter().map(|p| p.to_web_mercator()));
            black_box(&out);
        });
    });
    group.bench_function("fit_scan_1m", |b| {
        b.iter(|| black_box(fit(black_box(&path))))
    });
    group.bench_function("track_decimated", |b| {
        let mut out = Vec::with_capacity(8192);
        b.iter(|| {
            track_decimated(black_box(&path), view, &mut out);
            black_box(&out);
        });
    });
    group.bench_function("track_full_1m", |b| {
        let mut out = Vec::with_capacity(TRACK_POINTS);
        b.iter(|| {
            to_screen_all(black_box(&path), view, &mut out);
            black_box(&out);
        });
    });
    group.bench_function("markers_10k", |b| {
        let mut out = Vec::with_capacity(MARKERS);
        b.iter(|| {
            to_screen_all(black_box(&markers), view, &mut out);
            black_box(&out);
        });
    });
    group.bench_function("tiles_pan", |b| {
        let mut out = Vec::with_capacity(512);
        let after = view_of(panned(camera, 40.0));
        b.iter(|| {
            tiles(black_box(after), &mut out);
            black_box(&out);
        });
    });
    group.bench_function("tiles_zoom_step", |b| {
        let mut out = Vec::with_capacity(512);
        let after = view_of(zoomed(camera));
        b.iter(|| {
            tiles(black_box(after), &mut out);
            black_box(&out);
        });
    });
    group.bench_function("frame_follow", |b| {
        let mut frame = Frame::new();
        b.iter(|| frame.run(black_box(&path), black_box(&markers), None));
    });
    group.bench_function("frame_pan", |b| {
        let mut frame = Frame::new();
        let mut at = camera;
        b.iter(|| {
            at = panned(at, 1.0);
            frame.run(black_box(&path), black_box(&markers), Some(at));
        });
    });
    group.finish();
}

/// §8.2's floor for the CPU share of a frame: the following frame's 99th percentile, which is
/// the heavier of the two, over 500 frames.
fn gate() {
    if cfg!(debug_assertions) {
        println!("pan_zoom gate: skipped in an unoptimised build");
        return;
    }
    let path: Vec<WebMercator> = survey_track().iter().map(|p| p.to_web_mercator()).collect();
    let markers = markers();
    let mut frame = Frame::new();
    for _ in 0..20 {
        frame.run(&path, &markers, None);
    }
    let mut times: Vec<Duration> = (0..500)
        .map(|_| {
            let started = Instant::now();
            frame.run(black_box(&path), black_box(&markers), None);
            started.elapsed()
        })
        .collect();
    times.sort_unstable();
    let (p50, p99) = (times[times.len() / 2], times[times.len() * 99 / 100]);
    println!(
        "pan_zoom gate: frame_follow p50 {p50:?}, p99 {p99:?}, budget {FRAME_BUDGET:?} \
         ({} tiles, {} track points, {} markers)",
        frame.tiles.len(),
        frame.track.len(),
        frame.markers.len()
    );
    assert!(
        p99 <= FRAME_BUDGET,
        "the map's CPU work alone takes {p99:?} at p99, over the {FRAME_BUDGET:?} of 120 fps"
    );
}

criterion_group!(benches, pan_zoom);

fn main() {
    benches();
    Criterion::default().configure_from_args().final_summary();
    gate();
}
