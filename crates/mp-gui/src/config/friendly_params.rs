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

//! Standard Params and Advanced Params: `GCSViews/ConfigurationView/ConfigFriendlyParams.cs`,
//! which CONFIG's list adds as "Standard Params" (`GCSViews/SoftwareConfig.cs:196-199`) and, as
//! `ConfigFriendlyParamsAdv` - the same class with `ParameterMode` set to `Advanced`
//! (`ConfigFriendlyParamsAdv.cs:5-10`) - as "Advanced Params" (`SoftwareConfig.cs:201-204`), both
//! once every parameter is in.
//!
//! What it shows: Write Params, Refresh Params and Find along the top, and under them a
//! `FlowLayoutPanel` of one control per parameter the vehicle holds whose documentation gives it a
//! display name and marks it for the page's level: `@User: Standard` for Standard Params;
//! `@User: Advanced`, or no `@User` at all, for Advanced Params (`FilterParamList`, `:269-294`).
//! Each control is built from the documentation as ADSB's are - `ConfigADSB.cs` is this class's
//! code with another list: a range with an increment is a `RangeControl`, a bitmask a
//! `MavlinkCheckBoxBitMask`, a list of values a `ValuesControl`, anything else nothing
//! (`AddControl`, `:341-543`) - ordered by name, the `fav_params` favourites first (`:324-330`).
//!
//! Changing a control writes nothing: it records the value (`Control_ValueChanged`, `:545-548`);
//! Write Params writes every recorded value, `ENABLE` names first, each in its own `try`, and says
//! "Parameters successfully saved." when none failed (`:179-205`). Refresh Params asks "Update
//! Params" with its "Show me again?" box, fetches the list again and binds it (`:212-236`). Find
//! asks for a word and hides the controls whose name and description lack it, filtering as it is
//! typed, half a second after the last key (`:21-118`).
//!
//! The page object and its drawing are ADSB's ([`super::adsb`]) with [`STANDARD`] or
//! [`ADVANCED`] for its [`Spec`]: the same handlers, the same controls, the flow panel's layout -
//! each control three pixels in and three apart, a hidden one taking no room - at
//! `ConfigFriendlyParams.resx`'s places, 576 wide in a 626-wide panel at (12, 36).
//!
//! Where this differs from the C#, and why:
//!
//! * a failed write ("Set X Failed", `Strings.ErrorSetValueFailed`) and a failed fetch
//!   ("Error receiving list") go on the status line, not in a box: the owner's ruling of
//!   2026-09-25, as `extra_setup::link_error` has it;
//! * a `@User` value other than `Standard` or `Advanced` counts as none, so the parameter is an
//!   Advanced one: the documentation is parsed into those two levels and "not said"
//!   (`mp_params::pdef`), and ArduPilot writes no other;
//! * the flow panel's own scroll bar: the page scrolls, with the list in it.
//!
//! Ctrl+S (`ProcessCmdKey`, `:159-168`), the track bar's thumb and keys, typing into a
//! `ValuesControl`'s box and a bitmask's narrowing to the parameter's integer type are the ADSB
//! page's, as `adsb.rs` has them. Find's answer and Refresh Params' "Show me again?" are kept in
//! `Settings.Instance` as ADSB's are, under the same keys.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, FocusHandle, Window, div, prelude::*};
use mp_params::ParamMeta;
use mp_params::param_meta::UserLevel;

use super::adsb::{Access, Adsb, Ids, Spec, list_body};
use crate::MissionPlanner;
use crate::ui::panel;

/// Standard Params' class, as the lists name it.
pub const STANDARD_CLASS: &str = "ConfigFriendlyParams";

/// Advanced Params' class.
pub const ADVANCED_CLASS: &str = "ConfigFriendlyParamsAdv";

/// `Strings.StandardParams`.
/// `// C#: GCSViews/SoftwareConfig.cs:198; ExtLibs/Strings/Strings.resx:262-264`
pub const STANDARD_TITLE: &str = "Standard Params";

/// `Strings.AdvancedParams`.
/// `// C#: GCSViews/SoftwareConfig.cs:203; ExtLibs/Strings/Strings.resx:238-240`
pub const ADVANCED_TITLE: &str = "Advanced Params";

/// The `Settings` list of favourites both pages put first.
/// `// C#: GCSViews/ConfigurationView/ConfigFriendlyParams.cs:316`
pub const FAVOURITES: &str = "fav_params";

/// `flowLayoutPanel1`'s `Location` and `Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigFriendlyParams.resx (flowLayoutPanel1.Location, .Size)`
const LIST_AT: (f32, f32) = (12.0, 36.0);
/// Its `Size`.
const LIST_SIZE: (f32, f32) = (626.0, 145.0);

/// `FilterParamList` in Standard mode: a display name, and `@User: Standard`.
/// `// C#: GCSViews/ConfigurationView/ConfigFriendlyParams.cs:283-292`
#[must_use]
pub fn standard(_name: &str, meta: &ParamMeta) -> bool {
    !meta.display_name.is_empty() && meta.user_level == UserLevel::Standard
}

/// `FilterParamList` in Advanced mode: a display name, and `@User: Advanced` or no `@User`.
/// `// C#: GCSViews/ConfigurationView/ConfigFriendlyParams.cs:283-292`
#[must_use]
pub fn advanced(_name: &str, meta: &ParamMeta) -> bool {
    !meta.display_name.is_empty()
        && matches!(
            meta.user_level,
            UserLevel::Advanced | UserLevel::Unspecified
        )
}

/// The ids of a page's fixed controls, under its name.
const fn ids(
    write: &'static str,
    refresh: &'static str,
    find: &'static str,
    find_box: &'static str,
    message: &'static str,
    message_ok: &'static str,
    confirm: [&'static str; 4],
) -> Ids {
    let [confirm, confirm_showagain, confirm_ok, confirm_cancel] = confirm;
    Ids {
        write,
        refresh,
        find,
        find_box,
        message,
        message_ok,
        confirm,
        confirm_showagain,
        confirm_ok,
        confirm_cancel,
    }
}

/// Standard Params: `ConfigFriendlyParams`, `ParameterMode` Standard.
/// `// C#: GCSViews/ConfigurationView/ConfigFriendlyParams.cs:146-157`
pub static STANDARD: Spec = Spec {
    name: "standardparams",
    title: STANDARD_TITLE,
    favourites: FAVOURITES,
    select: standard,
    list_at: LIST_AT,
    list_size: LIST_SIZE,
    flow: true,
    ids: ids(
        "standardparams-write",
        "standardparams-refresh",
        "standardparams-find",
        "standardparams-find-box",
        "standardparams-message",
        "standardparams-message-ok",
        [
            "standardparams-confirm",
            "standardparams-confirm-showagain",
            "standardparams-confirm-ok",
            "standardparams-confirm-cancel",
        ],
    ),
};

/// Advanced Params: `ConfigFriendlyParamsAdv`, `ParameterMode` Advanced.
/// `// C#: GCSViews/ConfigurationView/ConfigFriendlyParamsAdv.cs:5-10`
pub static ADVANCED: Spec = Spec {
    // "advancedparams", not "advanced": the Advanced page (config/advanced.rs) publishes under
    // `config.advanced.` and this page's `active` overwrote its own (found 2026-09-25).
    name: "advancedparams",
    title: ADVANCED_TITLE,
    favourites: FAVOURITES,
    select: advanced,
    list_at: LIST_AT,
    list_size: LIST_SIZE,
    flow: true,
    ids: ids(
        "advancedparams-write",
        "advancedparams-refresh",
        "advancedparams-find",
        "advancedparams-find-box",
        "advancedparams-message",
        "advancedparams-message-ok",
        [
            "advancedparams-confirm",
            "advancedparams-confirm-showagain",
            "advancedparams-confirm-ok",
            "advancedparams-confirm-cancel",
        ],
    ),
};

/// A page object as nothing has shown it.
#[must_use]
pub fn standard_page() -> Adsb {
    Adsb::new(&STANDARD)
}

/// Advanced Params' page object as nothing has shown it.
#[must_use]
pub fn advanced_page() -> Adsb {
    Adsb::new(&ADVANCED)
}

/// The page, laid out as `ConfigFriendlyParams.resx` lays it out, in a 652-wide page.
/// `// C#: GCSViews/ConfigurationView/ConfigFriendlyParams.Designer.cs:29-74; ConfigFriendlyParams.resx`
#[allow(clippy::too_many_arguments)] // the page's four focuses, and the window they are read in
pub fn page(
    list: &Adsb,
    access: Access,
    number: &FocusHandle,
    prompt: &FocusHandle,
    page: &FocusHandle,
    track: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if !list.is_active() {
        return div().into_any_element();
    }
    let body = list_body(list, access, number, prompt, page, track, window, cx);
    panel(list.spec().title, body).into_any_element()
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use mp_link::requests::RequestOutcome;

    use super::*;

    /// A table that says no parameter's type.
    fn untyped(_: &str) -> Option<mp_params::ParamType> {
        None
    }
    use crate::config::adsb::Control;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::tests::Answering;
    use crate::config::optional::{Event, Job, SetQueue, error};
    use crate::config::servo_output::Message;
    use crate::setup::Key;
    use crate::telemetry::TelemetryView;

    fn bundled(name: &str) -> Option<&'static ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    fn table(entries: &[(&str, f64)]) -> Vec<(String, f64)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    /// Part of a copter's list, as SITL's copter holds it: a Standard range, value list and
    /// bitmask, an Advanced range, and one the documentation gives no `@User`.
    fn copter() -> Vec<(String, f64)> {
        table(&[
            ("RTL_ALT", 1500.0),
            ("FS_THR_ENABLE", 1.0),
            ("ARMING_CHECK", 1.0),
            ("ANGLE_MAX", 3000.0),
            ("BARO_FLTR_RNG", 0.0),
            ("NOT_DOCUMENTED", 1.0),
        ])
    }

    fn names(page: &Adsb) -> Vec<&str> {
        page.controls().iter().map(|c| c.name.as_str()).collect()
    }

    fn control<'a>(page: &'a Adsb, name: &str) -> Option<&'a Control> {
        page.controls().iter().find(|c| c.name == name)
    }

    fn run(jobs: Vec<Job>, link: &Answering) -> (Vec<Message>, Vec<Event>) {
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        let mut messages = VecDeque::new();
        let mut events = Vec::new();
        for _ in 0..100 {
            events.extend(queue.advance(link, &mut messages));
            if queue.pending() == 0 {
                break;
            }
        }
        (messages.into_iter().collect(), events)
    }

    /// The buttons, the panel and the page are where the `.resx` has them; `chk_advview` is in
    /// the `.resx` and the Designer never adds it, so it is not drawn.
    #[test]
    fn the_places_and_texts_are_the_resx_ones() {
        let Some(resx) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigFriendlyParams.resx",
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let get = |key: &str| values.get(key).map(String::as_str);
        assert_eq!(get("BUT_writePIDS.Text"), Some("Write Params"));
        assert_eq!(get("BUT_writePIDS.Location"), Some("12, 11"));
        assert_eq!(get("BUT_rerequestparams.Text"), Some("Refresh Params"));
        assert_eq!(get("BUT_rerequestparams.Location"), Some("121, 11"));
        assert_eq!(get("BUT_Find.Text"), Some("Find"));
        assert_eq!(get("BUT_Find.Location"), Some("230, 11"));
        assert_eq!(get("flowLayoutPanel1.Location"), Some("12, 36"));
        assert_eq!(get("flowLayoutPanel1.Size"), Some("626, 145"));
        assert_eq!(get("$this.Size"), Some("652, 190"));
        assert_eq!(STANDARD.list_at, (12.0, 36.0));
        assert_eq!(STANDARD.list_size, (626.0, 145.0));
        assert!((STANDARD.control_width() - 576.0).abs() < f32::EPSILON);
        assert_eq!(STANDARD.description_width(), 626);
        assert_eq!(STANDARD.bitmask_description_width(), 576);
        let designer = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigFriendlyParams.Designer.cs",
        )
        .unwrap_or_default();
        assert!(!designer.contains("chk_advview"), "never added");
    }

    /// Standard Params: the Standard parameters with a display name, each the control its
    /// documentation makes, by name.
    #[test]
    fn standard_lists_the_standard_parameters() {
        let mut page = standard_page();
        let jobs = page.activate(&copter(), &untyped, key(), bundled, &[]);
        assert!(jobs.is_empty());
        assert_eq!(names(&page), ["ARMING_CHECK", "FS_THR_ENABLE", "RTL_ALT"]);
        assert_eq!(
            control(&page, "ARMING_CHECK").map(Control::kind_name),
            Some("bitmask")
        );
        let fs = control(&page, "FS_THR_ENABLE").expect("a list of values");
        assert_eq!(fs.kind_name(), "values");
        assert_eq!(fs.shown(), "Enabled always RTL");
        assert_eq!(fs.label, "Throttle Failsafe Enable (FS_THR_ENABLE)");
        let rtl = control(&page, "RTL_ALT").expect("a range");
        assert_eq!(rtl.kind_name(), "range");
        assert_eq!(rtl.shown(), "1500");
        assert!(rtl.description.starts_with("Units: cm\r\nDescription: "));
    }

    /// Advanced Params: `@User: Advanced`, and the ones the documentation gives no `@User`.
    #[test]
    fn advanced_lists_the_advanced_and_unmarked_parameters() {
        let mut page = advanced_page();
        page.activate(&copter(), &untyped, key(), bundled, &[]);
        assert_eq!(names(&page), ["ANGLE_MAX", "BARO_FLTR_RNG"]);
        assert_eq!(
            bundled("BARO_FLTR_RNG").map(|m| m.user_level),
            Some(UserLevel::Unspecified)
        );
        // A parameter nothing documents is on neither.
        let mut standard = standard_page();
        standard.activate(&copter(), &untyped, key(), bundled, &[]);
        assert!(control(&standard, "NOT_DOCUMENTED").is_none());
        assert!(control(&page, "NOT_DOCUMENTED").is_none());
    }

    /// Over SITL's own list, as the C# reads it: every parameter with a display name is on one of
    /// the two pages and not both, and the controls are the ones its documentation makes.
    #[test]
    fn every_named_sitl_parameter_is_on_one_page() {
        let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/params/sitl-copter.param");
        let text = std::fs::read_to_string(&fixture).expect("the SITL dump");
        let parameters: Vec<(String, f64)> = text
            .lines()
            .filter_map(|line| {
                let (name, value) = line.split_once(',')?;
                Some((name.to_owned(), value.trim().parse().ok()?))
            })
            .collect();
        assert_eq!(parameters.len(), 1408);
        let mut standard_list = standard_page();
        standard_list.activate(&parameters, &untyped, key(), bundled, &[]);
        let mut advanced_list = advanced_page();
        advanced_list.activate(&parameters, &untyped, key(), bundled, &[]);
        for (name, _) in &parameters {
            let Some(meta) = bundled(name) else {
                continue;
            };
            if meta.display_name.is_empty() {
                assert!(!standard(name, meta) && !advanced(name, meta), "{name}");
                continue;
            }
            assert!(standard(name, meta) != advanced(name, meta), "{name}");
            // A control where the documentation makes one: on the page that takes it.
            let on_standard = control(&standard_list, name).is_some();
            let on_advanced = control(&advanced_list, name).is_some();
            assert!(!(on_standard && on_advanced), "{name} is on both");
            if on_standard {
                assert!(standard(name, meta), "{name}");
            }
            if on_advanced {
                assert!(advanced(name, meta), "{name}");
            }
        }
        assert!(control(&standard_list, "FS_THR_ENABLE").is_some());
        assert!(standard_list.controls().len() > 100);
        assert!(advanced_list.controls().len() > 100);
        // In name order, as the culture sorts them.
        let order = names(&standard_list);
        let mut sorted = order.clone();
        sorted.sort_by(|a, b| crate::config::adsb::culture_cmp(a, b));
        assert_eq!(order, sorted);
    }

    /// `fav_params` first, then by name, as `toadd.OrderBy` sorts.
    #[test]
    fn favourites_come_first() {
        let mut page = standard_page();
        page.activate(&copter(), &untyped, key(), bundled, &["RTL_ALT".to_owned()]);
        assert_eq!(names(&page), ["RTL_ALT", "ARMING_CHECK", "FS_THR_ENABLE"]);
    }

    /// A change is recorded, not written; Write Params writes it and says so.
    #[test]
    fn write_params_writes_what_changed() {
        let mut page = standard_page();
        page.activate(&copter(), &untyped, key(), bundled, &[]);
        let index = page
            .controls()
            .iter()
            .position(|c| c.name == "FS_THR_ENABLE")
            .expect("FS_THR_ENABLE");
        page.toggle_dropdown(index);
        page.choose(index, 3);
        assert_eq!(
            page.changed().get("FS_THR_ENABLE").map(String::as_str),
            Some("3")
        );
        let link = Answering::new(&[]);
        let (messages, events) = run(page.write_params(), &link);
        assert!(messages.is_empty());
        assert_eq!(link.taken(), [("FS_THR_ENABLE".to_owned(), 3.0)]);
        assert!(events.iter().any(|event| matches!(
            event,
            Event::Done {
                tag: "write",
                threw: false
            }
        )));
    }

    /// A timed-out write is the C#'s "Set X Failed" error, which the page puts on the status
    /// line (the owner's ruling), not a box.
    #[test]
    fn a_failed_write_is_a_link_error() {
        let mut page = standard_page();
        page.activate(&copter(), &untyped, key(), bundled, &[]);
        let index = page
            .controls()
            .iter()
            .position(|c| c.name == "FS_THR_ENABLE")
            .expect("FS_THR_ENABLE");
        page.choose(index, 0);
        let link = Answering::new(&[(
            "FS_THR_ENABLE",
            Progress::Finished(RequestOutcome::TimedOut),
        )]);
        let (messages, _) = run(page.write_params(), &link);
        assert_eq!(messages, [error("Set FS_THR_ENABLE Failed")]);
        assert!(messages.iter().all(crate::config::extra_setup::link_error));
    }

    /// Find hides the controls whose name and description lack the word.
    #[test]
    fn find_filters_by_name_and_description() {
        let mut page = standard_page();
        page.activate(&copter(), &untyped, key(), bundled, &[]);
        page.filter("rtl");
        let shown: Vec<&str> = page
            .controls()
            .iter()
            .filter(|c| c.visible)
            .map(|c| c.name.as_str())
            .collect();
        // RTL_ALT by its name; FS_THR_ENABLE's values say "RTL", and Find reads only the
        // label and the description.
        assert_eq!(shown, ["RTL_ALT"]);
        page.filter("");
        assert!(page.controls().iter().all(|c| c.visible));
    }

    #[test]
    fn the_gui_scripts_name_facts_and_controls_these_pages_have() {
        let source = format!(
            "{}{}",
            include_str!("adsb.rs"),
            include_str!("friendly_params.rs")
        );
        let per_control = [".kind", ".text", ".visible", ".label", ".trackbar"];
        for (script, name) in [
            (
                include_str!("../../../../tests/gui/config-standard-params.gui"),
                "standardparams",
            ),
            (
                include_str!("../../../../tests/gui/config-advanced-params.gui"),
                "advancedparams",
            ),
        ] {
            let prefix = format!("config.{name}.");
            let mut facts = 0;
            for line in script.lines() {
                let line = line.split('#').next().unwrap_or("");
                let mut words = line.split_whitespace();
                match (words.next(), words.next()) {
                    (Some("expect"), Some(key)) if key.starts_with(&prefix) => {
                        let what = key.trim_start_matches(&prefix);
                        let recorded = source.contains(&format!("fact(\"{what}\")"))
                            || what.starts_with("changed.")
                            || per_control.iter().any(|suffix| key.ends_with(suffix));
                        assert!(recorded, "{key} is not recorded");
                        facts += 1;
                    }
                    (Some("click"), Some(id)) if id.starts_with(&format!("{name}-")) => {
                        let drawn = source.contains(&format!("\"{id}\""))
                            || id
                                .chars()
                                .nth(name.len() + 1)
                                .is_some_and(char::is_uppercase);
                        assert!(drawn, "{id} is not drawn");
                    }
                    _ => {}
                }
            }
            assert!(facts >= 3, "{name}: {facts} facts");
        }
    }
}
