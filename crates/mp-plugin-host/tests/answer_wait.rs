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

//! How long a plugin waits for the window (crates/mp-plugin-host/src/channel.rs): an ordinary
//! request gives up after [`mp_plugin_host::ANSWER_WAIT`] and takes "no"; the Welcome-Demo-Sitl's
//! pointer waits [`mp_plugin_host::DEMO_ANSWER_WAIT`], since a page drawn in software can be
//! slower than that between frames, and a gesture the plugin took for refused was made again
//! (2026-10-05: a survey polygon's corner clicked twice). Here with both waits shortened, the
//! window answering between them.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::sync::mpsc::channel;
use std::sync::{Arc, RwLock};
use std::time::Duration;

use mp_plugin_host::{ChannelSurface, DEMO_ANSWER_WAIT, RequestBody, Snapshot, Surface as _};

#[test]
fn a_late_answer_is_the_demos_but_not_an_ordinary_requests() {
    assert!(DEMO_ANSWER_WAIT > mp_plugin_host::ANSWER_WAIT);
    let (requests, window) = channel();
    let mut surface = ChannelSurface::new(0, requests, Arc::new(RwLock::new(Snapshot::default())))
        .with_waits(Duration::from_millis(50), Duration::from_secs(10));
    // The window: every request answered "yes", a quarter of a second late.
    let answering = wasm_thread::spawn(move || {
        let mut answered = Vec::new();
        while let Ok(request) = window.recv() {
            wasm_thread::sleep(Duration::from_millis(250));
            match request.body {
                RequestBody::SetMode { reply, .. } => {
                    reply.send(true);
                    answered.push("set mode");
                }
                RequestBody::DemoClickMap { reply, .. } => {
                    reply.send(true);
                    answered.push("demo click");
                }
                _ => {}
            }
        }
        answered
    });
    // An ordinary request gives up first and takes "no"...
    assert!(!surface.set_mode("Auto"));
    // ...the demo's waits for the window's answer.
    assert!(surface.demo_click_map(-35.36, 149.16, 1000));
    drop(surface);
    // Both were served: the "no" was the plugin's giving up, not the window's answer.
    assert_eq!(answering.join().unwrap(), ["set mode", "demo click"]);
}
