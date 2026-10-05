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

/// The Survey (Grid) dialog's: while it shows it is FLIGHT PLAN's screen, so the mission's
/// buttons are not there to be cut off (the owner's browser, 2026-10-05: "CUT OFF ... plan-read
/// (missing)" over the open dialog). Its map and its close box are on every tab; Accept is on
/// Simple only, and like every control drawn it may not be cut off when it shows.
pub const SURVEY: &[&str] = &["main-port", "main-connect", "survey-map", "survey-close"];

/// The important ids of what shows: `screen`'s, or the Survey (Grid) dialog's while it is open
/// over FLIGHT PLAN.
#[must_use]
pub fn important_now(screen: Screen, survey_open: bool) -> &'static [&'static str] {
    if screen == Screen::Plan && survey_open {
        SURVEY
    } else {
        important(screen)
    }
}

/// The rows of the lists that scroll, by the prefix of their probe names: the only controls
/// allowed to be out of sight. A list is named here when Mission Planner's own scrolls - a
/// `DataGridView`, a `ListBox` - never to excuse a panel that does not fit.
pub const MAY_SCROLL: &[&str] = &[
    // `Commands`, the mission grid (`FlightPlanner.Designer.cs`): its rows past the third.
    "plan-row-",
    // A file dialog's list of its folder, as the dialog's own list scrolls.
    "plan-file-",
    // FLIGHT DATA's page strip, `tabControlactions`: one line, as the C#'s, its other tabs
    // reached with the strip's arrows.
    // `// C#: GCSViews/FlightData.Designer.cs (tabControlactions)`
    "fly-tab-",
    // The Full Parameter List's groups, `treeView1`'s nodes, which scroll in their list.
    // `// C#: GCSViews/ConfigurationView/ConfigRawParams.Designer.cs`
    "param-group-",
    // SETUP's and CONFIG's page lists, the BackstageView's `pnlMenu`, which scrolls.
    // `// C#: ExtLibs/Controls/BackstageView/BackstageView.Designer.cs (pnlMenu)`
    "setup-list",
    "setup-page-",
    "config-list",
    "config-page-",
    // HW IDs' grid, `myDataGridView1`, which scrolls.
    // `// C#: GCSViews/ConfigurationView/ConfigHWIDs.Designer.cs`
    "hwids-grid",
    // HELP's text, `richTextBox1`, which scrolls in its box.
    // `// C#: GCSViews/Help.Designer.cs (richTextBox1)`
    "help-text",
    // The MAVLink Inspector's and the DroneCAN Inspector's trees, each a `MyTreeView`, whose
    // nodes scroll in it (config-mavlink-inspector.gui: 56 nodes below the tree's box, 2026-10-06).
    // `// C#: Controls/MAVLinkInspector.cs:20, 179; Controls/DroneCANInspector.cs:21, 205`
    "inspector-node-",
    "dronecan-inspector-node-",
    // The OSD page's left side, `tableLeft`, which scrolls, and what scrolls in it: the screen's
    // canvas (`layoutControl`) and the item list (`panelItemList`) - 720 by 874 in a 593 by 762
    // page at 1600 by 1200 (config-onboard-osd.gui, 2026-10-06). A scrolling box is measured by
    // what it holds, so it too.
    // `// C#: ExtLibs/OSDConfigurator/GUI/ScreenControl.Designer.cs:179, 184-185 (tableLeft)`
    "osd-left",
    "osd-layout",
    "osd-items",
    // The log browser's fields, `treeView1`'s nodes and their expanders, which scroll in it
    // (log-params.gui and three others: the GPS and IMU fields below its box, 2026-10-06).
    // `// C#: Log/LogBrowse.designer.cs:63, 445 (treeView1)`
    "logfield-",
    // FLIGHT DATA's quick view chooser, `selectform`, every property a check box in a form that
    // scrolls (fly-quick.gui: 186 below it, 2026-10-06).
    // `// C#: GCSViews/FlightData.cs:4556-4566 (AutoScroll = true)`
    "fly-quick-choice-",
];

/// The rows of the lists that scroll and are named by what they hold - a parameter's name, an
/// OSD item's - in capitals, as no control's is: so told from the screen's own controls under the
/// same prefix (`param-search`, `osd-item-options`).
const NAMED_ROWS: &[&str] = &[
    // The Full Parameter List's rows and their cells, `Params`' rows, which scroll in their grid.
    // The grid fills the height the screen leaves it, so its last row is mostly part in view (the
    // owner's banner of 2026-10-05: "CUT OFF (the layout guard): param-COMPASS_DIA_X[...]").
    // `// C#: GCSViews/ConfigurationView/ConfigRawParams.Designer.cs:57, 241 (Params)`
    "param-",
    "param-value-",
    "param-desc-",
    "param-fav-",
    // Standard Params' and Advanced Params' rows, in `flowLayoutPanel1`, which scrolls
    // (config-advanced-params.gui: AHRS_GPS_MINSATS at the window's foot, 2026-10-06).
    // `// C#: GCSViews/ConfigurationView/ConfigFriendlyParams.resx:125-126 (AutoScroll True)`
    "standardparams-",
    "advancedparams-",
    // The OSD page's items and their check boxes, in `panelItemList`, which scrolls
    // (config-onboard-osd.gui: 21 below it, 2026-10-06).
    // `// C#: ExtLibs/OSDConfigurator/GUI/ScreenControl.Designer.cs:53 (AutoScroll)`
    "osd-item-",
];

/// Whether a control must be wholly on screen whenever it is measured: all but a scrolling
/// list's rows.
#[must_use]
pub fn must_show(name: &str) -> bool {
    !MAY_SCROLL.iter().any(|prefix| name.starts_with(prefix)) && !is_named_row(name)
}

/// Whether a control is a row of one of [`NAMED_ROWS`]' lists: its prefix, then a name in capitals.
fn is_named_row(name: &str) -> bool {
    NAMED_ROWS.iter().any(|prefix| {
        name.strip_prefix(prefix)
            .is_some_and(|rest| rest.starts_with(|first: char| first.is_ascii_uppercase()))
    })
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
/// `important` controls of what shows ([`important_now`]) that were not laid out at all, as
/// "`name` (missing)": a screen that lost its mission grid has hidden it as surely as one that
/// scrolled it away. (The probe keeps only what was measured in the last frame, which is the
/// screen showing; another screen's controls are not counted.)
#[must_use]
pub fn hidden(important: &[&str]) -> Vec<String> {
    let mut hidden: Vec<String> = crate::probe::clipped_names()
        .into_iter()
        .filter(|name| must_show(name))
        .collect();
    hidden.extend(
        important
            .iter()
            .filter(|name| crate::probe::clipped(name).is_none())
            .map(|name| format!("{name} (missing)")),
    );
    hidden
}

/// `layout.hidden` and `layout.hidden.names`, judged by `important` ([`important_now`]); "n/a"
/// when the probe is off, since nothing is measured then.
pub fn record_facts(important: &[&str]) {
    if !crate::probe::enabled() {
        crate::facts::record("layout.hidden", "n/a");
        crate::facts::record("layout.hidden.names", "n/a");
        return;
    }
    let hidden = hidden(important);
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

/// The file in the planner's data folder every cut-off the banner shows is added to, one line
/// each: [`record_line`].
pub const RECORD_FILE: &str = "layout-guard.log";

/// One cut-off as the record has it: when, on which screen, in what size of window, and what.
#[must_use]
pub fn record_line(time: &str, screen: &str, size: (f32, f32), text: &str) -> String {
    format!(
        "{time} CUT OFF on {screen} at {:.0}x{:.0}: {text}",
        size.0, size.1
    )
}

/// Adds `line` to the record. A record that cannot be written is not worth stopping for: the
/// strip and the stderr log still say it.
fn append_record(path: &std::path::Path, line: &str) {
    use std::io::Write as _;
    let written = mp_os::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
        .and_then(|mut file| writeln!(file, "{line}"));
    if let Err(err) = written {
        log::debug!("layout guard record {}: {err}", path.display());
    }
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
    showing: Option<(String, web_time::Instant)>,
    /// Whether the strip has shown it yet, so the log has it once.
    logged: bool,
}

impl Banner {
    /// This frame's verdict on what shows, judged by `important` ([`important_now`]), in a window
    /// `size` wide and high. Each cut-off the
    /// strip comes to show also goes to the log and to [`RECORD_FILE`], with the time, the
    /// `place` - the screen, and the page within it - and the window's size, so every run leaves a record of every one it met that outlives it
    /// (the owner, 2026-10-04: "are you capturing *all the CUT OFF events into a log ... so you
    /// dont miss them").
    pub fn update(&mut self, place: &str, important: &[&str], size: (f32, f32)) {
        let names = if crate::probe::enabled() {
            hidden(important)
        } else {
            Vec::new()
        };
        let text = (!names.is_empty()).then(|| describe(&names));
        let now = web_time::Instant::now();
        self.observe(text, now);
        // A cut-off not yet shown: drawn again when the strip is due (repaint.rs).
        if self.showing.is_some() && self.text_at(now).is_none() {
            crate::repaint::again_in(BANNER_AFTER);
        }
        if let Some(text) = self.newly_shown(now) {
            let line = record_line(
                &chrono::Local::now().format("%Y-%m-%d %H:%M:%S").to_string(),
                place,
                size,
                &text,
            );
            log::warn!("{line}");
            if let Some(folder) = mp_settings::user_data_directory() {
                append_record(&folder.join(RECORD_FILE), &line);
            }
        }
    }

    /// What is cut off at `now`, if anything: the time starts again when it changes.
    pub fn observe(&mut self, text: Option<String>, now: web_time::Instant) {
        self.showing = match (text, self.showing.take()) {
            (None, _) => None,
            (Some(text), Some((shown, since))) if shown == text => Some((shown, since)),
            (Some(text), _) => {
                self.logged = false;
                Some((text, now))
            }
        };
    }

    /// What the strip has just come to show at `now`: each cut-off once.
    pub fn newly_shown(&mut self, now: web_time::Instant) -> Option<String> {
        if self.logged {
            return None;
        }
        let text = self.text_at(now)?.to_owned();
        self.logged = true;
        Some(text)
    }

    /// What the strip says at `now`, once the cut-off has lasted.
    #[must_use]
    pub fn text_at(&self, now: web_time::Instant) -> Option<&str> {
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
        // The parameter grid's rows and their cells scroll; the screen's controls do not.
        for row in [
            "param-COMPASS_DIA_X",
            "param-value-COMPASS_DIA_X",
            "param-desc-COMPASS_DIA_X",
            "param-fav-COMPASS_DIA_X",
        ] {
            assert!(!must_show(row), "{row}");
        }
        for control in [
            "param-values",
            "param-value-edit",
            "param-col-Desc",
            "param-search",
            "param-refresh",
        ] {
            assert!(must_show(control), "{control}");
        }
        // The inspectors' tree nodes and the OSD page's items scroll, and its scrolling side with
        // them; the OSD page's options box does not.
        for row in [
            "inspector-node-1-1-0",
            "dronecan-inspector-node-125",
            "osd-item-ALTITUDE",
            "osd-item-ALTITUDE-check",
            "osd-items",
            "osd-left",
            "fly-quick-choice-battery_temp",
            "logfield-GPS.HDop",
        ] {
            assert!(!must_show(row), "{row}");
        }
        assert!(!must_show("advancedparams-AHRS_GPS_MINSATS"));
        for control in [
            "osd-item-options",
            "advancedparams-write",
            "advancedparams-find-box",
        ] {
            assert!(must_show(control), "{control}");
        }
    }

    /// The banner names a cut-off once it has lasted, not the frame a screen is switched to; a
    /// change starts the time again, and nothing cut off takes it away.
    #[test]
    fn the_banner_names_what_stays_cut_off() {
        let start = web_time::Instant::now();
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

    /// The record's line says when, where, at what size and what; lines are added, never
    /// replaced, so a restart keeps the run before.
    #[test]
    fn the_record_keeps_every_cut_off() {
        assert_eq!(
            record_line("2026-10-05 00:59:01", "fly", (1600.0, 920.0), "panel:actions"),
            "2026-10-05 00:59:01 CUT OFF on fly at 1600x920: panel:actions"
        );
        let path = mp_os::temp_dir().join(format!("mp-layout-record-{}.log", mp_os::process_id()));
        let _ = mp_os::fs::remove_file(&path);
        append_record(&path, "first");
        append_record(&path, "second");
        assert_eq!(
            mp_os::fs::read_to_string(&path).unwrap_or_default(),
            "first\nsecond\n"
        );
        let _ = mp_os::fs::remove_file(&path);
    }

    /// The log has each cut-off once, when the strip first shows it, and again only when it
    /// changes and lasts.
    #[test]
    fn the_log_has_each_cut_off_once() {
        let start = web_time::Instant::now();
        let later = |ms: u64| start + std::time::Duration::from_millis(ms);
        let mut banner = Banner::default();
        banner.observe(Some("header".to_owned()), start);
        assert_eq!(banner.newly_shown(later(100)), None);
        assert_eq!(banner.newly_shown(later(600)), Some("header".to_owned()));
        banner.observe(Some("header".to_owned()), later(700));
        assert_eq!(banner.newly_shown(later(800)), None, "logged already");
        banner.observe(Some("help-changelog".to_owned()), later(900));
        assert_eq!(
            banner.newly_shown(later(1_500)),
            Some("help-changelog".to_owned())
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

    /// The owner's browser, 2026-10-05: with the Survey (Grid) dialog open FLIGHT PLAN's
    /// buttons are not drawn, and the guard said they were missing. The dialog is judged by its
    /// own map and close box, and the connect box over it; closed, the planner by its own.
    #[test]
    fn the_survey_dialog_is_judged_by_its_own_controls() {
        let open = important_now(Screen::Plan, true);
        assert_eq!(open, SURVEY);
        assert!(open.contains(&"main-connect"));
        assert!(open.contains(&"survey-map"));
        assert!(!open.contains(&"plan-write"));
        assert_eq!(important_now(Screen::Plan, false), important(Screen::Plan));
        // The survey belongs to FLIGHT PLAN; any other screen is its own.
        assert_eq!(important_now(Screen::Fly, true), important(Screen::Fly));
    }
}
