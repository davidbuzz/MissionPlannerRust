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
//! Deliverable 7 asks for a Windows build that opens a window and paints; the Direct3D 11 backend has never
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
//! And that text is drawn: once a frame has painted, the window's text system lays out a line,
//! and a system that cannot tell its letters apart fails the run - a window that painted every
//! frame and not one label passed before (macOS, 2026-10-04). That was gpui_macos built without
//! its `font-kit` feature, which then takes gpui's `NoopTextSystem`: it has no font, and gives
//! every plain letter the same glyph, so a line still comes back with as many glyphs as letters
//! and only how many *different* glyphs it has tells it from a real one.
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

/// The line the window's text system is asked to lay out once a frame has painted.
pub const TEXT_PROBE: &str = "MissionPlannerRust";

/// The different glyphs the text system gave [`TEXT_PROBE`]'s letters; [`NOT_ASKED`] till it has
/// been asked.
static GLYPHS: AtomicU64 = AtomicU64::new(NOT_ASKED);
const NOT_ASKED: u64 = u64::MAX;

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

/// Whether the text system is still to be asked about [`TEXT_PROBE`].
#[must_use]
pub fn text_unasked() -> bool {
    GLYPHS.load(Ordering::Relaxed) == NOT_ASKED
}

/// How many different glyphs a laid-out line holds.
#[must_use]
pub fn different_glyphs(line: &gpui::LineLayout) -> usize {
    let mut glyphs: Vec<u32> = line
        .runs
        .iter()
        .flat_map(|run| run.glyphs.iter().map(|glyph| glyph.id.0))
        .collect();
    glyphs.sort_unstable();
    glyphs.dedup();
    glyphs.len()
}

/// Records how many different glyphs the window's text system gave [`TEXT_PROBE`].
pub fn text_shaped(glyphs: usize) {
    GLYPHS.store(u64::try_from(glyphs).unwrap_or(NOT_ASKED - 1), Ordering::Relaxed);
}

/// The run's verdict once it has one: the window painted, and its text system told the probe
/// line's letters apart - a window whose every frame paints and not one label shows passed before
/// (the owner's bug of 2026-10-04: on macOS the planner drew no text at all, gpui_macos built
/// without its `font-kit` feature, and CI's smoke run passed). `None` while it is still to tell.
fn verdict(painted: u64, glyphs: u64) -> Option<Result<String, String>> {
    if painted < REQUIRED_FRAMES || glyphs == NOT_ASKED {
        return None;
    }
    Some(if glyphs < 2 {
        Err(format!(
            "smoke: {painted} frames painted, but the text system gave the letters of \"{TEXT_PROBE}\" {glyphs} different glyph(s) - it has no font, and no text would be drawn"
        ))
    } else {
        Ok(format!(
            "smoke: {painted} frames painted, \"{TEXT_PROBE}\" shaped in {glyphs} different glyphs - the window works"
        ))
    })
}

/// Watches from another thread and ends the process once the window has proved itself.
///
/// A thread rather than a task on the executor: the thing being tested is whether the render loop
/// runs at all, and a watchdog scheduled on the same executor would never fire on the machine
/// where it does not. It has to be able to report a hang, so it must not share the machinery that
/// might be hung.
pub fn watch() {
    wasm_thread::spawn(|| {
        let started = web_time::Instant::now();
        loop {
            let painted = frames();
            // Exiting from here rather than asking the application to close: a clean shutdown
            // path is a different thing to test, and if it were broken this test would hang
            // instead of reporting the frames it did see.
            match verdict(painted, GLYPHS.load(Ordering::Relaxed)) {
                Some(Ok(line)) => {
                    println!("{line} (in {:.1?})", started.elapsed());
                    std::process::exit(0);
                }
                Some(Err(line)) => {
                    eprintln!("{line}");
                    std::process::exit(1);
                }
                None => {}
            }
            if started.elapsed() >= DEADLINE {
                eprintln!(
                    "smoke: only {painted} frames in {:.0?}; the window did not paint",
                    started.elapsed()
                );
                std::process::exit(1);
            }
            wasm_thread::sleep(Duration::from_millis(50));
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

    /// Painting is not enough: a text system that cannot tell letters apart - none shaped, or
    /// every one the same glyph, as gpui's NoopTextSystem gives them - fails the run, and the run
    /// waits for both answers before it says either.
    #[test]
    fn the_run_passes_only_with_frames_and_text() {
        assert_eq!(verdict(REQUIRED_FRAMES - 1, 13), None);
        assert_eq!(verdict(REQUIRED_FRAMES, NOT_ASKED), None);
        assert!(matches!(verdict(REQUIRED_FRAMES, 0), Some(Err(_))));
        assert!(matches!(verdict(REQUIRED_FRAMES, 1), Some(Err(_))));
        assert!(matches!(verdict(REQUIRED_FRAMES, 13), Some(Ok(_))));
    }

    /// The text system gpui_macos took without `font-kit` - the macOS build that drew no text -
    /// fails the run: it lays the probe line out letter for letter, but every letter as one glyph.
    #[test]
    fn the_text_system_with_no_font_fails_the_run() {
        use gpui::PlatformTextSystem as _;
        let line = gpui::NoopTextSystem::new().layout_line(
            TEXT_PROBE,
            gpui::px(14.0),
            &[gpui::FontRun {
                len: TEXT_PROBE.len(),
                font_id: gpui::FontId(0),
            }],
        );
        let glyphs = different_glyphs(&line);
        assert_eq!(glyphs, 1);
        let glyphs = u64::try_from(glyphs).expect("a count");
        assert!(matches!(verdict(REQUIRED_FRAMES, glyphs), Some(Err(_))));
    }

    /// One frame is not enough to say a swap chain is cycling.
    const _: () = assert!(REQUIRED_FRAMES > 1);
}
