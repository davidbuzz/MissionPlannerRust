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

//! The controls an operator must never lose: for each screen, the ids that have to be wholly on
//! screen - inside the window and inside every box that clips them - whenever that screen shows.
//!
//! The owner's report of 2026-10-03: the planner's Mission box had scrolled Read WPs, Write WPs
//! and the file buttons off the bottom of the window, and he asked for a self-test that proves,
//! from the geometry, that parts of the screen like these are never hidden. The probe
//! ([`crate::probe`]) measures every named control as it is laid out and notes whether its paint
//! was clipped - the content mask gpui holds for it, cut to the window, did not contain it. This
//! module names the controls that matter and publishes `layout.hidden`, how many of them are
//! clipped on the screen showing, with their names in `layout.hidden.names`. Every GUI run ends
//! by reading it (`tools/gui-test.sh`: a run that leaves an important control hidden fails
//! unless the script says `allow-hidden`), and `tests/layout.rs` asserts it at the sizes the
//! window opens at, screen by screen.
//!
//! "Important" is a judgement, kept here in one place: what the operator reaches for on each
//! screen - the mission's read, write, save and load, the map, the connect box and button, the
//! flight screen's page strip and column, the setup and config screens' bodies. Each must be
//! measured on its screen, so a screen that lost one fails rather than passing on nothing.
//!
//! And no control may be cut off, important or not (the owner, 2026-10-04: the planner's column
//! ran below the window and took the mission grid with it, past a guard that only watched the
//! Mission box's buttons - "they need to be mandatory everywhere"). Every measured control counts
//! as hidden when its paint is clipped, but for the rows of a list that scrolls as Mission
//! Planner's does ([`MAY_SCROLL`]): a list whose fortieth row is below its edge is a list, a
//! panel whose bottom is below the window is a bug.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use crate::Screen;

/// The ids that must be wholly visible on each screen.
pub const IMPORTANT: &[(Screen, &[&str])] = &[
    (
        Screen::Fly,
        &["main-port", "main-connect", "fly-tabs", "fly-column", "map"],
    ),
    (
        Screen::Plan,
        &[
            "main-port",
            "main-connect",
            "map",
            "plan-read",
            "plan-write",
            "plan-writefast",
            "plan-save",
            "plan-load",
        ],
    ),
    (Screen::Setup, &["main-port", "main-connect", "setup-body"]),
    (
        Screen::Config,
        &["main-port", "main-connect", "config-body"],
    ),
    (Screen::Sitl, &["main-port", "main-connect"]),
    // The temp form's table: every row shown, as the form shows them (the owner's bug of
    // 2026-10-04, rows run off the bottom of a smaller window).
    (
        Screen::Experimental,
        &["main-port", "main-connect", "experimental-table"],
    ),
    (
        Screen::Plugins,
        &["main-port", "main-connect", "plugin-manager"],
    ),
    (Screen::Params, &["main-port", "main-connect"]),
    (Screen::Logs, &["main-port", "main-connect"]),
    (Screen::Help, &["main-port", "main-connect"]),
];

/// The rows of the lists that scroll, by the prefix of their probe names: the only controls
/// allowed to be out of sight. A list is named here when Mission Planner's own scrolls - a
/// `DataGridView`, a `ListBox` - never to excuse a panel that does not fit.
pub const MAY_SCROLL: &[&str] = &[
    // `Commands`, the mission grid (`FlightPlanner.Designer.cs`): its rows past the third.
    "plan-row-",
    // A file dialog's list of its folder, as the dialog's own list scrolls.
    "plan-file-",
];

/// Whether a control must be wholly on screen whenever it is measured: all but a scrolling
/// list's rows.
#[must_use]
pub fn must_show(name: &str) -> bool {
    !MAY_SCROLL.iter().any(|prefix| name.starts_with(prefix))
}

/// The important ids of a screen.
#[must_use]
pub fn important(screen: Screen) -> &'static [&'static str] {
    IMPORTANT
        .iter()
        .find(|(held, _)| *held == screen)
        .map_or(&[], |(_, ids)| ids)
}

/// Whether a control is important on any screen (the probe marks it in its file).
#[must_use]
pub fn is_important(name: &str) -> bool {
    IMPORTANT.iter().any(|(_, ids)| ids.contains(&name))
}

/// The controls on screen whose paint was clipped this frame - measured, and not wholly inside
/// the window and the boxes above them - but for the rows [`MAY_SCROLL`] lets go; and the
/// important controls of `screen`, the one showing, that were not laid out at all, as
/// "`name` (missing)": a screen that lost its mission grid has hidden it as surely as one that
/// scrolled it away. (The probe keeps only what was measured in the last frame, which is the
/// screen showing; another screen's controls are not counted.)
#[must_use]
pub fn hidden(screen: Screen) -> Vec<String> {
    let mut hidden: Vec<String> = crate::probe::clipped_names()
        .into_iter()
        .filter(|name| must_show(name))
        .collect();
    hidden.extend(
        important(screen)
            .iter()
            .filter(|name| crate::probe::clipped(name).is_none())
            .map(|name| format!("{name} (missing)")),
    );
    hidden
}

/// `layout.hidden` and `layout.hidden.names`; "n/a" when the probe is off, since nothing is
/// measured then.
pub fn record_facts(screen: Screen) {
    if !crate::probe::enabled() {
        crate::facts::record("layout.hidden", "n/a");
        crate::facts::record("layout.hidden.names", "n/a");
        return;
    }
    let hidden = hidden(screen);
    crate::facts::record("layout.hidden", hidden.len());
    crate::facts::record(
        "layout.hidden.names",
        if hidden.is_empty() {
            "none".to_owned()
        } else {
            describe(&hidden)
        },
    );
}

/// Each hidden control with where it was laid out and what could be seen there, so the failure
/// says which edge cut it: `map[9,77 724x839 in 0,0 1600x760]`.
fn describe(hidden: &[String]) -> String {
    hidden
        .iter()
        .map(|name| match crate::probe::placement(name) {
            Some((at, seen)) => format!(
                "{name}[{:.0},{:.0} {:.0}x{:.0} in {:.0},{:.0} {:.0}x{:.0}]",
                at.x, at.y, at.width, at.height, seen.x, seen.y, seen.width, seen.height
            ),
            None => name.clone(),
        })
        .collect::<Vec<_>>()
        .join(",")
}

/// How long a cut-off lasts before the banner names it: longer than the frame a screen just
/// switched to is measured late in, when its important controls look missing.
pub const BANNER_AFTER: std::time::Duration = std::time::Duration::from_millis(500);

/// The debug build's own guard (the owner, 2026-10-04: the cut-off guard "mandatory everywhere"):
/// what [`hidden`] has named on the screen showing for longer than [`BANNER_AFTER`], said in a red
/// strip across the window's foot - at whatever size the window is, on every screen, without a
/// test run. The strip is drawn over the window, so it moves nothing; a release build measures
/// nothing and never shows it.
#[derive(Debug, Default)]
pub struct Banner {
    /// What is cut off, described, and since when it has been so.
    showing: Option<(String, std::time::Instant)>,
}

impl Banner {
    /// This frame's verdict on `screen`.
    pub fn update(&mut self, screen: Screen) {
        let names = if crate::probe::enabled() {
            hidden(screen)
        } else {
            Vec::new()
        };
        let text = (!names.is_empty()).then(|| describe(&names));
        self.observe(text, std::time::Instant::now());
    }

    /// What is cut off at `now`, if anything: the time starts again when it changes.
    pub fn observe(&mut self, text: Option<String>, now: std::time::Instant) {
        self.showing = match (text, self.showing.take()) {
            (None, _) => None,
            (Some(text), Some((shown, since))) if shown == text => Some((shown, since)),
            (Some(text), _) => Some((text, now)),
        };
    }

    /// What the strip says at `now`, once the cut-off has lasted.
    #[must_use]
    pub fn text_at(&self, now: std::time::Instant) -> Option<&str> {
        self.showing
            .as_ref()
            .filter(|(_, since)| now.duration_since(*since) >= BANNER_AFTER)
            .map(|(text, _)| text.as_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every screen the window can show has a list, and every list names the connect box, which
    /// is on every screen.
    #[test]
    fn every_screen_names_its_controls_and_the_connect_box() {
        for screen in [
            Screen::Fly,
            Screen::Plan,
            Screen::Setup,
            Screen::Config,
            Screen::Sitl,
            Screen::Params,
            Screen::Logs,
            Screen::Experimental,
            Screen::Plugins,
            Screen::Help,
        ] {
            let ids = important(screen);
            assert!(!ids.is_empty(), "{screen:?} has no important controls");
            assert!(
                ids.contains(&"main-connect"),
                "{screen:?} lacks the connect button"
            );
        }
        assert!(is_important("plan-write"));
        assert!(
            !is_important("plan-items"),
            "a scrolling list is allowed to scroll"
        );
    }

    /// Everything must show but a scrolling list's rows: the planner's column, its panels and
    /// the grid itself may not be cut off; the grid's fortieth row may.
    #[test]
    fn only_a_lists_rows_may_scroll_out_of_sight() {
        for name in [
            "plan-sidebar",
            "panel:mission items",
            "panel:home",
            "plan-read",
            "map",
            "experimental-table",
        ] {
            assert!(must_show(name), "{name}");
        }
        assert!(!must_show("plan-row-40"));
    }

    /// The banner names a cut-off once it has lasted, not the frame a screen is switched to; a
    /// change starts the time again, and nothing cut off takes it away.
    #[test]
    fn the_banner_names_what_stays_cut_off() {
        let start = std::time::Instant::now();
        let later = |ms: u64| start + std::time::Duration::from_millis(ms);
        let mut banner = Banner::default();
        banner.observe(Some("plan-read".to_owned()), start);
        assert_eq!(banner.text_at(later(100)), None, "a frame late is not cut off");
        banner.observe(Some("plan-read".to_owned()), later(200));
        assert_eq!(banner.text_at(later(600)), Some("plan-read"));
        banner.observe(Some("plan-read,plan-write".to_owned()), later(700));
        assert_eq!(banner.text_at(later(900)), None, "the time starts again");
        assert_eq!(banner.text_at(later(1_300)), Some("plan-read,plan-write"));
        banner.observe(None, later(1_400));
        assert_eq!(banner.text_at(later(5_000)), None);
    }

    /// The owner's case: the Mission box's buttons are on the planner's list, so a planner whose
    /// buttons are clipped counts them as hidden.
    #[test]
    fn the_mission_buttons_are_the_planners_important_controls() {
        let ids = important(Screen::Plan);
        for id in [
            "plan-read",
            "plan-write",
            "plan-writefast",
            "plan-save",
            "plan-load",
        ] {
            assert!(ids.contains(&id), "{id} is not guarded");
        }
    }
}
