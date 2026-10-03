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

//! A run that proves the window opened and the GPU painted it, then exits.
//!
//! D7 asks for a Windows build that opens a window and paints; the Direct3D 11 backend has never
//! been exercised because nobody here has a Windows machine. This is the part of that which can be
//! automated: the application starts normally, paints for real, and exits with a status that says
//! whether it did. CI runs it on all three platforms, so the backend that has never run becomes
//! the backend that runs on every push.
//!
//! What it checks is deliberately narrow and deliberately real. It does not assert what is on
//! screen - a screenshot does that, and needs eyes. It asserts that the window opened, that the
//! renderer produced frames, and that the process came down cleanly. That is the failure this
//! catches: a backend that will not initialise, a surface it cannot create, a shader that will not
//! compile. Those kill the application at startup on the machine that has the problem and nowhere
//! else, and they are invisible to every test that does not open a window.
//!
//! Counted in `render`, which gpui calls once per painted frame. Counting anything earlier - the
//! window handle existing, the executor starting - would pass on a machine where the GPU never
//! produced a pixel, which is the exact failure being looked for.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

/// Frames painted since the process started.
static FRAMES: AtomicU64 = AtomicU64::new(0);

/// How many frames count as "it paints".
///
/// More than one, because the first frame of a gpui application can come from a path that a broken
/// backend still survives; a second one means the swap chain is cycling. Not many more, because
/// this is a smoke test and its value is in running everywhere rather than running long.
const REQUIRED_FRAMES: u64 = 3;

/// How long to wait for them before calling it a failure.
///
/// Generous: a CI runner with a software rasteriser is slow, and a smoke test that fails when the
/// machine is busy is a smoke test that gets disabled.
const DEADLINE: Duration = Duration::from_secs(30);

/// Whether this run should exit as soon as it has proved itself.
///
/// An environment variable rather than a flag, so it can be set for a run of the real binary with
/// its real arguments - which is the point. A smoke test of a special build proves something about
/// the special build.
#[must_use]
pub fn enabled() -> bool {
    std::env::var_os("MP_SMOKE").is_some()
}

/// Records a painted frame.
pub fn painted() {
    FRAMES.fetch_add(1, Ordering::Relaxed);
}

/// How many frames have been painted.
#[must_use]
pub fn frames() -> u64 {
    FRAMES.load(Ordering::Relaxed)
}

/// Watches from another thread and ends the process once the window has proved itself.
///
/// A thread rather than a task on the executor: the thing being tested is whether the render loop
/// runs at all, and a watchdog scheduled on the same executor would never fire on the machine
/// where it does not. It has to be able to report a hang, so it must not share the machinery that
/// might be hung.
pub fn watch() {
    std::thread::spawn(|| {
        let started = std::time::Instant::now();
        loop {
            let painted = frames();
            if painted >= REQUIRED_FRAMES {
                let elapsed = started.elapsed();
                println!("smoke: {painted} frames painted in {elapsed:.1?} - the window works");
                // Exiting from here rather than asking the application to close: a clean shutdown
                // path is a different thing to test, and if it were broken this test would hang
                // instead of reporting the frames it did see.
                std::process::exit(0);
            }
            if started.elapsed() >= DEADLINE {
                eprintln!(
                    "smoke: only {painted} frames in {:.0?}; the window did not paint",
                    started.elapsed()
                );
                std::process::exit(1);
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The counter is what the pass/fail decision reads, so it has to count.
    #[test]
    fn frames_are_counted() {
        let before = frames();
        painted();
        painted();
        assert_eq!(frames(), before + 2);
    }

    /// One frame is not enough to say a swap chain is cycling.
    const _: () = assert!(REQUIRED_FRAMES > 1);
}
