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

//! Layout regression: nothing may be drawn outside the window, or outside the panel that holds it.
//!
//! This is the failure this UI keeps having. It is invisible in a screenshot, because a screenshot
//! shows the part that fitted - the flight screen shipped twice with its bottom panel chopped in
//! half, and both times the screenshot looked fine.
//!
//! The application reports where every panel and control ends up when `MP_PROBE` names a file
//! (see `src/probe.rs`). This runs it, reads that file and checks the geometry.
//!
//! Ignored by default because it needs a display, but ignored rather than silently skipped: the
//! test output says it exists and was not run. Run it with:
//!
//!     cargo test -p mp-gui --test layout -- --ignored --test-threads=1
//!
//! It opens a window for a few seconds and closes it again. One at a time, because several
//! windows fighting for focus is how a layout test becomes a flaky one.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::{Child, Command};
use web_time::{Duration, Instant};

/// A measured rectangle.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Rect {
    x: f32,
    y: f32,
    width: f32,
    height: f32,
    /// Whether the application found the control's paint clipped: not wholly inside the window
    /// and the boxes above it (`src/probe.rs`).
    clipped: bool,
    /// Whether the application names it as one that must never be hidden (`src/layout_guard.rs`).
    important: bool,
    /// Whether the application lets it scroll out of sight - only a scrolling list's rows may
    /// (`src/layout_guard.rs`, `MAY_SCROLL`).
    must_show: bool,
}

impl Rect {
    fn bottom(self) -> f32 {
        self.y + self.height
    }

    fn right(self) -> f32 {
        self.x + self.width
    }
}

/// Kills the application even if an assertion panics, so a failing test does not leave a window on
/// someone's desktop.
struct Running(Child);

impl Drop for Running {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Runs the GUI at a given size on a given screen and returns what it measured.
fn measure(width: u32, height: u32, screen: &str) -> BTreeMap<String, Rect> {
    // The application writes Mission Planner's config.xml on starting; a measurement must not
    // rewrite the settings of the Mission Planner installed on this machine.
    let config = mp_os::temp_dir().join("headless-planner-layout-config.xml");
    measure_with(width, height, screen, &config)
}

/// Runs the GUI at a given size on a given screen with a given `config.xml`, and returns what it
/// measured. Offline, with a data directory of its own, so no tile is fetched and no saved link
/// is opened.
fn measure_with(width: u32, height: u32, screen: &str, config: &Path) -> BTreeMap<String, Rect> {
    let probe: PathBuf = mp_os::temp_dir().join(format!(
        "headless-planner-layout-{width}x{height}-{screen}.json"
    ));
    let _ = std::fs::remove_file(&probe);
    let home = mp_os::temp_dir().join("headless-planner-layout-home");
    let _ = std::fs::create_dir_all(&home);

    let child = Command::new(env!("CARGO_BIN_EXE_planner"))
        .env("MP_PROBE", &probe)
        .env("MP_WINDOW", format!("{width}x{height}"))
        .env("MP_SCREEN", screen)
        .env("MP_OFFLINE", "1")
        .env("XDG_DATA_HOME", &home)
        .env("MP_CONFIG_XML", config)
        .env(
            "DISPLAY",
            std::env::var("DISPLAY").unwrap_or_else(|_| ":0".to_owned()),
        )
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .expect("planner should start; this test needs a display and is ignored by default");
    let _running = Running(child);

    // Wait for the layout to settle. The probe is rewritten whenever anything moves, so a stable
    // read means the layout has stopped changing rather than merely started.
    let deadline = Instant::now() + Duration::from_secs(25);
    let mut previous = BTreeMap::new();
    let mut stable_since: Option<Instant> = None;
    loop {
        let current = read(&probe);
        if !current.is_empty() && current == previous {
            let since = *stable_since.get_or_insert_with(Instant::now);
            if since.elapsed() > Duration::from_millis(600) {
                return current;
            }
        } else {
            stable_since = None;
            previous = current;
        }
        assert!(
            Instant::now() < deadline,
            "the layout never settled; measured {} controls",
            previous.len()
        );
        wasm_thread::sleep(Duration::from_millis(100));
    }
}

/// Parses the probe file. One control per line, so a line-oriented parse is enough.
fn read(path: &Path) -> BTreeMap<String, Rect> {
    let Ok(text) = std::fs::read_to_string(path) else {
        return BTreeMap::new();
    };
    let mut out = BTreeMap::new();
    for line in text.lines() {
        let Some((name, rest)) = line.trim().split_once("\": {") else {
            continue;
        };
        let name = name.trim_start_matches('"');
        let number = |key: &str| -> Option<f32> {
            let marker = format!("\"{key}\": ");
            let start = rest.find(&marker)? + marker.len();
            let tail = rest.get(start..)?;
            let end = tail.find([',', ' ', '}']).unwrap_or(tail.len());
            tail.get(..end)?.parse().ok()
        };
        let (Some(x), Some(y), Some(width), Some(height)) =
            (number("x"), number("y"), number("width"), number("height"))
        else {
            continue;
        };
        let flag = |key: &str| rest.contains(&format!("\"{key}\": true"));
        out.insert(
            name.to_owned(),
            Rect {
                x,
                y,
                width,
                height,
                clipped: flag("clipped"),
                important: flag("important"),
                must_show: flag("must_show"),
            },
        );
    }
    out
}

/// The panels on a screen, by name.
fn panels(measured: &BTreeMap<String, Rect>) -> Vec<(String, Rect)> {
    measured
        .iter()
        .filter_map(|(name, rect)| {
            name.strip_prefix("panel:")
                .map(|name| (name.to_owned(), *rect))
        })
        .collect()
}

#[test]
#[ignore = "opens a window; needs a display"]
fn nothing_on_the_flight_screen_is_drawn_outside_the_window() {
    // The bug this exists for: the left column's bottom panel was chopped in half by the window
    // edge at the default size, twice, and neither screenshot showed it.
    let measured = measure(1600, 1200, "fly");
    let root = measured.get("root").copied().expect("the root is measured");

    let mut offenders = Vec::new();
    for (name, rect) in panels(&measured) {
        if rect.bottom() > root.bottom() || rect.right() > root.right() || rect.y < 0.0 {
            offenders.push(format!(
                "{name}: {:.0}..{:.0} vertically, window is {:.0}..{:.0}",
                rect.y,
                rect.bottom(),
                root.y,
                root.bottom()
            ));
        }
    }
    assert!(
        offenders.is_empty(),
        "panels drawn outside the window:\n  {}",
        offenders.join("\n  ")
    );
}

#[test]
#[ignore = "opens a window; needs a display"]
fn the_flight_screens_left_column_fits_without_scrolling() {
    // Scrolling works, but a column that has to be scrolled to reach the link statistics is a
    // column whose bottom panel looks broken. At the size the application opens at, it all fits.
    let measured = measure(1600, 1200, "fly");
    let column = measured
        .get("fly-column")
        .copied()
        .expect("the left column is measured");

    let lowest = panels(&measured)
        .into_iter()
        // The messages panel is in the right-hand column, not this one.
        .filter(|(name, _)| name != "messages")
        .max_by(|a, b| {
            a.1.bottom()
                .partial_cmp(&b.1.bottom())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .expect("at least one panel");

    assert!(
        lowest.1.bottom() <= column.bottom(),
        "the left column's content runs {:.0}px past the bottom of the column \
         (lowest panel '{}' ends at {:.0}, column ends at {:.0})",
        lowest.1.bottom() - column.bottom(),
        lowest.0,
        lowest.1.bottom(),
        column.bottom()
    );
}

/// The owner's self-test of 2026-10-03, made mandatory everywhere on 2026-10-04: on every screen,
/// at the size the window opens at and at a laptop's, no control is cut off - not wholly inside
/// the window and every box that clips it - but a scrolling list's rows (`src/layout_guard.rs`).
/// The planner's Mission box had scrolled Read WPs, Write WPs and the file buttons below the
/// window; then its column ran past the window's bottom with the mission grid in it, under a
/// guard that watched only the Mission box's buttons; the EXPERIMENTAL tab's rows ran off the
/// bottom of the owner's Mac window, which 1600x1200 never showed. The application judges the
/// clipping itself, from gpui's content mask; this reads its verdict, and demands that each
/// screen's important controls were there to judge.
#[test]
#[ignore = "opens a window; needs a display"]
fn no_control_is_cut_off_on_any_screen() {
    let mut hidden = Vec::new();
    let screens = [
        "fly",
        "plan",
        "setup",
        "config",
        "simulation",
        "params",
        "logs",
        "experimental",
        "plugins",
        "help",
    ];
    for screen in screens {
        for (width, height) in [(1600, 1200), (1280, 800)] {
            let measured = measure(width, height, screen);
            let important = measured.values().filter(|rect| rect.important).count();
            assert!(
                important >= 2,
                "{screen}: only {important} important controls measured; the connect box alone is two",
            );
            for (name, rect) in &measured {
                if rect.clipped && rect.must_show {
                    hidden.push(format!(
                        "{screen} at {width}x{height}: {name} at {:.0},{:.0} {:.0}x{:.0} is cut off",
                        rect.x, rect.y, rect.width, rect.height
                    ));
                }
            }
        }
    }
    assert!(
        hidden.is_empty(),
        "{} controls cut off:\n  {}",
        hidden.len(),
        hidden.join("\n  ")
    );
}

/// The owner's case, as he found it: the planner docked Bottom (`FP_docking`), where the Mission
/// box sat in a strip too short for it.
#[test]
#[ignore = "opens a window; needs a display"]
fn the_planners_mission_buttons_are_on_screen_when_docked_bottom() {
    let config = mp_os::temp_dir().join("headless-planner-layout-bottom-config.xml");
    std::fs::write(
        &config,
        "<?xml version=\"1.0\" encoding=\"utf-8\"?><Config><FP_docking>Bottom</FP_docking></Config>",
    )
    .unwrap();
    let measured = measure_with(1600, 1200, "plan", &config);
    let root = measured.get("root").copied().expect("the root is measured");
    for name in [
        "plan-read",
        "plan-write",
        "plan-writefast",
        "plan-save",
        "plan-load",
    ] {
        let rect = measured
            .get(name)
            .copied()
            .unwrap_or_else(|| panic!("{name} is measured"));
        assert!(
            !rect.clipped && rect.bottom() <= root.bottom(),
            "{name} ends at {:.0}, the window at {:.0}, clipped {}",
            rect.bottom(),
            root.bottom(),
            rect.clipped
        );
    }
}

#[test]
#[ignore = "opens a window; needs a display"]
fn the_plan_screen_keeps_its_panels_inside_the_window() {
    let measured = measure(1600, 1200, "plan");
    let root = measured.get("root").copied().expect("the root is measured");

    for (name, rect) in panels(&measured) {
        assert!(
            rect.right() <= root.right(),
            "panel '{name}' runs off the right edge: ends at {:.0}, window is {:.0} wide",
            rect.right(),
            root.right()
        );
    }
}

#[test]
#[ignore = "opens a window; needs a display"]
fn the_setup_and_config_screens_keep_their_list_and_page_inside_the_window() {
    // Each is a backstage view: the list down the left and the page beside it, each scrolling on
    // its own. Neither may run past the window at the size the application opens at, and no page
    // may run off its right edge.
    for (screen, list, page) in [
        ("setup", "setup-list", "setup-page"),
        ("config", "config-list", "config-page"),
    ] {
        let measured = measure(1600, 1200, screen);
        let root = measured.get("root").copied().expect("the root is measured");
        for name in [list, page] {
            let rect = measured
                .get(name)
                .copied()
                .unwrap_or_else(|| panic!("{name} is measured on the {screen} screen"));
            assert!(
                rect.bottom() <= root.bottom() && rect.right() <= root.right() && rect.x >= 0.0,
                "{name} runs past the window: {rect:?}, window {root:?}"
            );
        }
        for (name, rect) in panels(&measured) {
            assert!(
                rect.right() <= root.right(),
                "panel '{name}' on the {screen} screen runs off the right edge: ends at {:.0}, \
                 window is {:.0} wide",
                rect.right(),
                root.right()
            );
        }
    }
}

#[test]
#[ignore = "opens a window; needs a display"]
fn a_small_window_still_lays_out_rather_than_collapsing() {
    // MP_WINDOW accepts sizes down to 640x480. Everything will not fit at that size and the
    // columns scroll, which is fine - what must not happen is a panel of zero size or one placed
    // at a negative coordinate, both of which mean the layout gave up rather than adapted.
    let measured = measure(800, 600, "fly");
    assert!(!measured.is_empty(), "nothing was measured at 800x600");

    for (name, rect) in panels(&measured) {
        assert!(
            rect.width > 0.0 && rect.height > 0.0,
            "panel '{name}' collapsed to {:.0}x{:.0}",
            rect.width,
            rect.height
        );
        assert!(
            rect.x >= 0.0,
            "panel '{name}' was placed off the left edge at x={:.0}",
            rect.x
        );
    }
}
