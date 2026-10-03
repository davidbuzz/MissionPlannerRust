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
//! flight screen's page strip and column, the setup and config screens' bodies. Add to the list
//! rather than argue about a control that was hidden.

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
    (Screen::Params, &["main-port", "main-connect"]),
];

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

/// The important controls of `screen` whose paint was clipped this frame: measured, and not
/// wholly inside the window and the boxes above them. A control not measured - its screen not
/// showing, a page not built - is not hidden, it is absent; this says nothing about those.
#[must_use]
pub fn hidden(screen: Screen) -> Vec<&'static str> {
    important(screen)
        .iter()
        .copied()
        .filter(|name| crate::probe::clipped(name) == Some(true))
        .collect()
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
    // Each hidden control with where it was laid out and what could be seen there, so the
    // failure says which edge cut it: `map[9,77 724x839 in 0,0 1600x760]`.
    crate::facts::record(
        "layout.hidden.names",
        if hidden.is_empty() {
            "none".to_owned()
        } else {
            hidden
                .iter()
                .map(|name| match crate::probe::placement(name) {
                    Some((at, seen)) => format!(
                        "{name}[{:.0},{:.0} {:.0}x{:.0} in {:.0},{:.0} {:.0}x{:.0}]",
                        at.x, at.y, at.width, at.height, seen.x, seen.y, seen.width, seen.height
                    ),
                    None => (*name).to_owned(),
                })
                .collect::<Vec<_>>()
                .join(",")
        },
    );
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
