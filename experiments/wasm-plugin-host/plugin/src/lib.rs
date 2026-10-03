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

//! A Mission Planner plugin as WebAssembly: the C#'s `Plugin` lifecycle - `Name`, `Version`,
//! `Author`, `Init`, `Loaded`, `Loop` at `loopratehz`, `Exit` (`Plugin/Plugin.cs:15-42`) - and
//! what two of the shipped examples do with `Host`:
//!
//! * `plugins/example3-fencedist.cs`: `Loaded` adds "Draw Fence Dist" to the flight screen's map
//!   menu (`Host.FDMenuMap`), and the click measures how far the vehicle (`Host.cs`) is from the
//!   fence (`MAV.fencepoints`);
//! * `plugins/example2-menu.cs`: the click shows a message box (`CustomMessageBox.Show`) and asks
//!   with an `InputBox`.
//!
//! The host is reached through imports in the `env` module; strings cross as a pointer and a
//! length into this module's linear memory. No WASI: the plugin has no files, clock or sockets of
//! its own, only what the host lends it - which is the point of the experiment.
//!
//! Built for `wasm32-unknown-unknown`; the `std` there has no I/O, so a `panic!` aborts, which
//! reaches the host as an `unreachable` trap.

use std::cell::RefCell;

#[cfg(target_arch = "wasm32")]
#[link(wasm_import_module = "env")]
unsafe extern "C" {
    /// `log.Info`: a line for the host's log.
    fn host_log(ptr: *const u8, len: usize);
    /// `Host.cs.<field>`: a telemetry value by name; NaN for a name the host does not have.
    fn host_cs_get(ptr: *const u8, len: usize) -> f64;
    /// `Host.FDMenuMap.Items.Add(new ToolStripMenuItem(text))`: the entry's id, which
    /// `menu_click` gets back when it is chosen.
    fn host_menu_add(ptr: *const u8, len: usize) -> i32;
    /// `CustomMessageBox.Show(text)`.
    fn host_message(ptr: *const u8, len: usize);
    /// The status line: what the C# example draws on the map as a marker's text.
    fn host_status(ptr: *const u8, len: usize);
    /// `MAV.fencepoints.Count`, less the return point.
    fn host_fence_count() -> i32;
    /// A fence vertex's latitude and longitude, in degrees.
    fn host_fence_lat(index: i32) -> f64;
    fn host_fence_lng(index: i32) -> f64;
}

/// Off WebAssembly - the workspace builds this crate natively too, to type-check it - the host
/// is nobody: the imports answer with nothing, as a plugin with no host would see.
#[cfg(not(target_arch = "wasm32"))]
mod no_host {
    pub unsafe fn host_log(_: *const u8, _: usize) {}
    pub unsafe fn host_cs_get(_: *const u8, _: usize) -> f64 {
        f64::NAN
    }
    pub unsafe fn host_menu_add(_: *const u8, _: usize) -> i32 {
        0
    }
    pub unsafe fn host_message(_: *const u8, _: usize) {}
    pub unsafe fn host_status(_: *const u8, _: usize) {}
    pub unsafe fn host_fence_count() -> i32 {
        0
    }
    pub unsafe fn host_fence_lat(_: i32) -> f64 {
        f64::NAN
    }
    pub unsafe fn host_fence_lng(_: i32) -> f64 {
        f64::NAN
    }
}
#[cfg(not(target_arch = "wasm32"))]
use no_host::*;

fn log(text: &str) {
    // SAFETY: the pointer and length name bytes of a live `&str` in this module's own memory.
    unsafe { host_log(text.as_ptr(), text.len()) }
}

fn cs(name: &str) -> f64 {
    // SAFETY: as above.
    unsafe { host_cs_get(name.as_ptr(), name.len()) }
}

fn status(text: &str) {
    // SAFETY: as above.
    unsafe { host_status(text.as_ptr(), text.len()) }
}

fn message(text: &str) {
    // SAFETY: as above.
    unsafe { host_message(text.as_ptr(), text.len()) }
}

/// The plugin's state between calls: the menu entry `Loaded` made, and the last distance.
struct State {
    menu: Option<i32>,
    loops: u64,
    last_distance: f64,
}

thread_local! {
    static STATE: RefCell<State> = const {
        RefCell::new(State {
            menu: None,
            loops: 0,
            last_distance: f64::NAN,
        })
    };
}

const NAME: &str = "FenceDist";
const VERSION: &str = "0.10";
const AUTHOR: &str = "Michael Oborne";

/// `Name`, `Version` and `Author`: each as a pointer and a length the host reads.
#[unsafe(no_mangle)]
pub extern "C" fn plugin_name_ptr() -> *const u8 {
    NAME.as_ptr()
}
#[unsafe(no_mangle)]
pub extern "C" fn plugin_name_len() -> usize {
    NAME.len()
}
#[unsafe(no_mangle)]
pub extern "C" fn plugin_version_ptr() -> *const u8 {
    VERSION.as_ptr()
}
#[unsafe(no_mangle)]
pub extern "C" fn plugin_version_len() -> usize {
    VERSION.len()
}
#[unsafe(no_mangle)]
pub extern "C" fn plugin_author_ptr() -> *const u8 {
    AUTHOR.as_ptr()
}
#[unsafe(no_mangle)]
pub extern "C" fn plugin_author_len() -> usize {
    AUTHOR.len()
}

/// `loopratehz`: how often the host calls `plugin_loop`. The examples leave it 0 (never); this
/// plugin asks for 2 Hz so the experiment measures the call.
#[unsafe(no_mangle)]
pub extern "C" fn plugin_loop_rate_hz() -> f32 {
    2.0
}

/// `Init`: before the screens exist. `true` (1) keeps the plugin, as the loader keeps one whose
/// `Init` returns true.
/// `// C#: Plugin/PluginLoader.cs:175`
#[unsafe(no_mangle)]
pub extern "C" fn plugin_init() -> i32 {
    log("FenceDist init");
    1
}

/// `Loaded`: the screens exist; the menu entry goes in.
/// `// C#: plugins/example3-fencedist.cs:46-52; Plugin/PluginLoader.cs:327`
#[unsafe(no_mangle)]
pub extern "C" fn plugin_loaded() -> i32 {
    let text = "Draw Fence Dist";
    // SAFETY: `text` is a live `&str`.
    let id = unsafe { host_menu_add(text.as_ptr(), text.len()) };
    STATE.with(|state| state.borrow_mut().menu = Some(id));
    log("FenceDist loaded");
    1
}

/// `Loop`, at `loopratehz`: the distance from the vehicle to the fence, on the status line.
/// `// C#: MainV2.cs:2524-2540`
#[unsafe(no_mangle)]
pub extern "C" fn plugin_loop() -> i32 {
    let distance = fence_distance(cs("lat"), cs("lng"));
    STATE.with(|state| {
        let mut state = state.borrow_mut();
        state.loops += 1;
        state.last_distance = distance;
    });
    if distance.is_finite() {
        status(&format!("fence {distance:.1} m"));
    }
    1
}

/// `Exit`.
#[unsafe(no_mangle)]
pub extern "C" fn plugin_exit() -> i32 {
    log("FenceDist exit");
    1
}

/// The menu entry chosen: `but_Click` - the example's message box, and the measurement.
/// `// C#: plugins/example2-menu.cs:54-60; plugins/example3-fencedist.cs:53-56`
#[unsafe(no_mangle)]
pub extern "C" fn plugin_menu_click(id: i32) -> i32 {
    let ours = STATE.with(|state| state.borrow().menu == Some(id));
    if !ours {
        return 0;
    }
    message("This is a sample plugin\nSee the source in the plugins folder");
    let distance = fence_distance(cs("lat"), cs("lng"));
    status(&format!("fence {distance:.1} m"));
    1
}

/// How many times `Loop` has run, for the host's tests.
#[unsafe(no_mangle)]
pub extern "C" fn plugin_loops() -> u64 {
    STATE.with(|state| state.borrow().loops)
}

/// The last distance `Loop` measured, for the host's tests.
#[unsafe(no_mangle)]
pub extern "C" fn plugin_last_distance() -> f64 {
    STATE.with(|state| state.borrow().last_distance)
}

/// A plugin that panics: what a bug in a plugin does to the host.
#[unsafe(no_mangle)]
pub extern "C" fn plugin_explode() -> i32 {
    panic!("a plugin bug")
}

/// A plugin that never returns: what a runaway plugin does to the host. `black_box` keeps the
/// optimiser from folding the loop into its answer.
#[unsafe(no_mangle)]
pub extern "C" fn plugin_spin() -> i32 {
    let mut n: u64 = 0;
    loop {
        n = std::hint::black_box(n).wrapping_add(1);
        if std::hint::black_box(n) == 0 {
            return 1;
        }
    }
}

/// The C# example's measurement, in its shape: the nearest point on each fence edge to the
/// vehicle, on a sphere of 6371 km (`var R = 6371e3`), the smallest kept; 99999 with no fence
/// and NaN without a position.
/// `// C#: plugins/example3-fencedist.cs:58-130`
fn fence_distance(lat: f64, lng: f64) -> f64 {
    if !lat.is_finite() || !lng.is_finite() {
        return f64::NAN;
    }
    // SAFETY: the host's count and its indexed reads have no pointers.
    let count = unsafe { host_fence_count() };
    if count < 2 {
        return 99999.0;
    }
    let vertices: Vec<(f64, f64)> = (0..count)
        // SAFETY: as above.
        .map(|i| unsafe { (host_fence_lat(i), host_fence_lng(i)) })
        .collect();
    let mut best = 99999.0_f64;
    for i in 0..vertices.len() {
        let a = vertices[i];
        let b = vertices[(i + 1) % vertices.len()];
        best = best.min(distance_to_segment((lat, lng), a, b));
    }
    best
}

/// The distance from `p` to the segment `a`-`b`, in metres, in a local flat frame at `p` on the
/// 6371 km sphere: good to a fraction of a percent over a fence's few kilometres.
fn distance_to_segment(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    const R: f64 = 6371e3;
    let scale = p.0.to_radians().cos();
    let to_xy = |q: (f64, f64)| {
        (
            (q.1 - p.1).to_radians() * scale * R,
            (q.0 - p.0).to_radians() * R,
        )
    };
    let (ax, ay) = to_xy(a);
    let (bx, by) = to_xy(b);
    let (dx, dy) = (bx - ax, by - ay);
    let length2 = dx * dx + dy * dy;
    let t = if length2 == 0.0 {
        0.0
    } else {
        (-(ax * dx + ay * dy) / length2).clamp(0.0, 1.0)
    };
    let (cx, cy) = (ax + t * dx, ay + t * dy);
    (cx * cx + cy * cy).sqrt()
}
