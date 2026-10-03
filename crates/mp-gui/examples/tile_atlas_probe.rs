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

//! Measures what a raster map tile actually costs through gpui's image path.
//!
//! The load-bearing question for Deliverable 8's tile pipeline is whether a `RenderImage` painted with
//! `Window::paint_image` is uploaded to the sprite atlas once or re-uploaded every frame. The
//! source says once (`AtlasState::get_or_insert_with` in gpui's `platform.rs`), keyed on
//! `RenderImageParams { image_id, frame_index }`. This probe checks that claim against a running
//! GPU by painting N 256x256 tiles per frame in two modes:
//!
//! * `stable` - the same `Arc<RenderImage>` every frame, so `ImageId` is stable and every paint
//!   after the first should be an atlas hit.
//! * `churn`  - a fresh `RenderImage` every frame, so `ImageId` changes and every paint is a
//!   miss: a full decode-buffer upload plus an atlas allocation that is never freed.
//!
//! The ratio between the two is the answer. Run:
//!   MP_MODE=stable MP_TILES=60 cargo run -p mp-gui --example tile_atlas_probe
//!   MP_MODE=churn  MP_TILES=60 cargo run -p mp-gui --example tile_atlas_probe

#![allow(
    clippy::print_stderr,
    clippy::print_stdout,
    missing_docs,
    unreachable_pub
)]
#![allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]

use std::rc::Rc;
use std::sync::Arc;
use std::time::{Duration, Instant};

use gpui::{
    App, Application, Bounds, Context, Corners, Platform, Render, RenderImage, Window,
    WindowBounds, WindowOptions, canvas, div, point, prelude::*, px, size,
};

const TILE_PX: u32 = 256;

fn current_platform() -> Rc<dyn Platform> {
    #[cfg(any(target_os = "linux", target_os = "freebsd"))]
    {
        gpui_linux::current_platform(false)
    }
    #[cfg(target_os = "windows")]
    {
        Rc::new(gpui_windows::WindowsPlatform::new(false).expect("windows platform"))
    }
    #[cfg(target_os = "macos")]
    {
        Rc::new(gpui_macos::MacPlatform::new(false))
    }
}

/// One 256x256 tile's worth of BGRA bytes.
///
/// gpui stores `RenderImage` frames in **BGRA** order despite the `image::RgbaImage` container -
/// see `decode_static_image_from_decoder`, which does `pixel.swap(0, 2)` after `into_rgba8()`.
/// A tile decoded from PNG must be swizzled the same way or every tile renders colour-swapped.
fn make_tile(seed: u32) -> Arc<RenderImage> {
    let mut buf = image::RgbaImage::new(TILE_PX, TILE_PX);
    for (x, y, px) in buf.enumerate_pixels_mut() {
        let checker = ((x / 32) + (y / 32)) % 2 == 0;
        let base = if checker { 60u8 } else { 30u8 };
        // stored B, G, R, A
        *px = image::Rgba([
            base.wrapping_add((seed * 37) as u8),
            base.wrapping_add((seed * 11) as u8),
            base.wrapping_add((seed * 71) as u8),
            255,
        ]);
    }
    Arc::new(RenderImage::new(vec![image::Frame::new(buf)]))
}

struct Probe {
    tiles: Vec<Arc<RenderImage>>,
    churn: bool,
    frames: u64,
    max_frames: u64,
    paint_ema: Duration,
    paint_worst: Duration,
    paint_total: Duration,
    measured: u64,
    errors: u64,
    /// Images painted in churn mode that must be evicted from the atlas.
    pending_evict: Vec<Arc<RenderImage>>,
    evict: bool,
    started: Option<Instant>,
}

impl Probe {
    fn new(cx: &mut Context<Self>) -> Self {
        let n: usize = std::env::var("MP_TILES")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(60);
        let churn = std::env::var("MP_MODE").as_deref() == Ok("churn");
        let evict = std::env::var("MP_EVICT").is_ok();
        // Defaults to roughly five seconds of frames. A probe should answer its question and get
        // off the screen; a long-running window is indistinguishable from a hang.
        let max_frames: u64 = std::env::var("MP_FRAMES")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(300);

        cx.spawn(async move |this, cx| {
            loop {
                cx.background_executor()
                    .timer(Duration::from_millis(1))
                    .await;
                if this.update(cx, |_, cx| cx.notify()).is_err() {
                    break;
                }
            }
        })
        .detach();

        Self {
            tiles: (0..n as u32).map(make_tile).collect(),
            churn,
            frames: 0,
            max_frames,
            paint_ema: Duration::ZERO,
            paint_worst: Duration::ZERO,
            paint_total: Duration::ZERO,
            measured: 0,
            errors: 0,
            pending_evict: Vec::new(),
            evict,
            started: None,
        }
    }
}

impl Render for Probe {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let this = cx.entity();
        div().size_full().child(
            canvas(|_, _, _| (), {
                move |bounds: Bounds<gpui::Pixels>, _, window: &mut Window, cx: &mut App| {
                    this.update(cx, |probe: &mut Probe, _| {
                        if probe.started.is_none() {
                            probe.started = Some(Instant::now());
                        }
                        if probe.churn {
                            // Every frame gets brand-new ImageIds: the pathological case.
                            let n = probe.tiles.len() as u32;
                            let genn = probe.frames as u32;
                            let fresh: Vec<Arc<RenderImage>> =
                                (0..n).map(|i| make_tile(genn.wrapping_mul(n) + i)).collect();
                            if probe.evict {
                                let stale = std::mem::replace(&mut probe.pending_evict, fresh.clone());
                                for img in stale {
                                    let _ = window.drop_image(img);
                                }
                            }
                            probe.tiles = fresh;
                        }

                        let cols = ((f32::from(bounds.size.width) / TILE_PX as f32).ceil() as usize).max(1);
                        let t0 = Instant::now();
                        for (i, img) in probe.tiles.iter().enumerate() {
                            let col = i % cols;
                            let row = i / cols;
                            let origin = point(
                                bounds.origin.x + px((col * TILE_PX as usize) as f32),
                                bounds.origin.y + px((row * TILE_PX as usize) as f32),
                            );
                            let tile_bounds = Bounds {
                                origin,
                                size: size(px(TILE_PX as f32), px(TILE_PX as f32)),
                            };
                            if window
                                .paint_image(
                                    tile_bounds,
                                    tile_bounds,
                                    Corners::default(),
                                    img.clone(),
                                    0,
                                    false,
                                )
                                .is_err()
                            {
                                probe.errors += 1;
                            }
                        }
                        let elapsed = t0.elapsed();

                        probe.frames += 1;
                        if probe.frames > 5 {
                            probe.measured += 1;
                            probe.paint_total += elapsed;
                            probe.paint_worst = probe.paint_worst.max(elapsed);
                            probe.paint_ema = if probe.paint_ema.is_zero() {
                                elapsed
                            } else {
                                (probe.paint_ema * 7 + elapsed) / 8
                            };
                        }

                        if probe.frames.is_multiple_of(100) {
                            eprintln!(
                                "frame {:>5}  paint_ema {:>8.3} ms  worst {:>8.3} ms  tiles {}  errors {}",
                                probe.frames,
                                probe.paint_ema.as_secs_f64() * 1e3,
                                probe.paint_worst.as_secs_f64() * 1e3,
                                probe.tiles.len(),
                                probe.errors,
                            );
                        }

                        if probe.frames >= probe.max_frames {
                            let wall = probe.started.map_or(Duration::ZERO, |s| s.elapsed());
                            let mean = if probe.measured == 0 {
                                Duration::ZERO
                            } else {
                                probe.paint_total / probe.measured as u32
                            };
                            println!(
                                "RESULT mode={} tiles={} evict={} frames={} mean_paint_ms={:.4} ema_ms={:.4} worst_ms={:.4} per_tile_us={:.3} wall_s={:.2} fps={:.1} errors={}",
                                if probe.churn { "churn" } else { "stable" },
                                probe.tiles.len(),
                                probe.evict,
                                probe.frames,
                                mean.as_secs_f64() * 1e3,
                                probe.paint_ema.as_secs_f64() * 1e3,
                                probe.paint_worst.as_secs_f64() * 1e3,
                                mean.as_secs_f64() * 1e6 / probe.tiles.len().max(1) as f64,
                                wall.as_secs_f64(),
                                probe.frames as f64 / wall.as_secs_f64().max(1e-9),
                                probe.errors,
                            );
                            std::process::exit(0);
                        }
                    });
                }
            })
            .size_full(),
        )
    }
}

fn main() {
    Application::with_platform(current_platform()).run(|cx: &mut App| {
        let bounds = Bounds::centered(None, size(px(1280.0), px(800.0)), cx);
        let options = WindowOptions {
            window_bounds: Some(WindowBounds::Windowed(bounds)),
            ..Default::default()
        };
        if let Err(e) = cx.open_window(options, |_, cx| cx.new(Probe::new)) {
            eprintln!("no window: {e}");
            return;
        }
        cx.activate(true);
    });
}
