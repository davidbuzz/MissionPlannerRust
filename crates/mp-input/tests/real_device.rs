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

//! Stick-to-wire latency over a real joystick.
//!
//! Ignored by default because it needs a joystick and a person: the device is edge-triggered, so
//! a stick nobody moves produces no samples at all. To run it, plug a joystick in and run
//!
//! ```text
//! cargo test -p mp-input --test real_device -- --ignored --nocapture
//! ```
//!
//! then stir the sticks for the ten seconds it reads. `MP_JS_DEVICE` picks a node other than
//! `/dev/input/js0`, and `MP_JS_SECONDS` changes how long it reads. Opening the node needs the
//! user to be in the `input` group. Nothing is sent anywhere: the sink accepts every frame and
//! throws it away.
//!
//! What it measures is the reader's own histogram - from the oldest `read` a frame delivers to the
//! sink returning - with `X`, `Y`, `Rx` and `Ry` - axes 0, 1, 3 and 4, the two sticks of an
//! XInput pad - on channels 1 to 4, so the sticks count and the triggers and buttons do not. A stick being moved is a stream of events, and every one that
//! lands within `MIN_INTERVAL` of the last frame is held by the rate floor, so the bound here is a
//! floor plus Deliverable 15's 5 ms, as for the stirred fake device. Deliverable 15's 5 ms itself is for an isolated
//! movement, which a person cannot produce on demand; `latency.rs` measures that. The kernel waking that read comes before it and cannot be timed
//! through the `js` API, whose event timestamps are jiffies on a clock this process cannot read;
//! the fake-device test in `latency.rs` covers that half, through a socket the kernel wakes the
//! same way.

#![cfg(target_os = "linux")]
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::path::Path;
use std::time::Duration;
use wasm_thread as thread;

use mp_input::{JoystickAxis, MIN_INTERVAL, Mapping, Poll, StickReader};

/// Deliverable 15's bar, on top of the rate floor a moving stick is held by.
const TARGET: Duration = Duration::from_millis(5);

/// Fewer changes than this and a p99 is one or two samples; it is printed but not judged.
const ENOUGH: u64 = 100;

#[test]
#[ignore = "needs a joystick on /dev/input/js0 and a person moving it"]
fn a_real_stick_reaches_the_sink_within_a_floor_and_five_milliseconds() {
    let path = std::env::var("MP_JS_DEVICE").unwrap_or_else(|_| "/dev/input/js0".to_owned());
    if !Path::new(&path).exists() {
        println!("{path} does not exist: plug a joystick in, or set MP_JS_DEVICE");
        return;
    }
    let seconds = std::env::var("MP_JS_SECONDS")
        .ok()
        .and_then(|text| text.parse().ok())
        .unwrap_or(10);

    let mut mapping = Mapping::default();
    for (channel, axis) in [
        (1, JoystickAxis::X),
        (2, JoystickAxis::Y),
        (3, JoystickAxis::Rx),
        (4, JoystickAxis::Ry),
    ] {
        mapping.config.set_axis(channel, axis);
    }
    let reader = StickReader::open(&path, mapping, |_frame| true)
        .unwrap_or_else(|err| panic!("opening {path}: {err} (is the user in the 'input' group?)"));
    assert!(reader.set_enabled(true));
    println!("reading {path} for {seconds} s - move the sticks");
    thread::sleep(Duration::from_secs(seconds));

    let histogram = reader.latency();
    let liveness = reader.liveness();
    reader.close();

    println!("\n{path}, read -> sink returned:\n{histogram}");
    assert_eq!(liveness, Poll::Alive, "the device went away during the run");
    if histogram.count() < ENOUGH {
        println!(
            "only {} changes: move the sticks for longer to get a p99 worth reading",
            histogram.count()
        );
        return;
    }
    let p99 = histogram.p99().expect("samples");
    let bound = MIN_INTERVAL + TARGET;
    assert!(p99 <= bound, "p99 {p99:?} is over {bound:?}\n{histogram}");
}
