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

//! The C#'s example plugins, ported to the world, each driven through the host against a
//! scripted application.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

mod common;

use common::{Scripted, fence, load};
use mp_plugin_host::{ControlKind, CsValue, DialogResult, MapMenu, MessageButtons};

/// `plugins/example.cs`: nameless, and `Init` false, so it is never kept.
#[test]
fn example_is_never_kept() {
    let Some((mut plugin, _)) = load("example") else {
        return;
    };
    assert_eq!(plugin.info().name, "");
    assert_eq!(plugin.info().file, "example.wasm");
    assert!(!plugin.init().unwrap());
}

/// `example2-menu.cs`: the planning map's entry asks for a heading, then two servos at the top of
/// the mission and two at the bottom.
#[test]
fn menu_fixes_the_missions_top_and_bottom() {
    let Some((mut plugin, script)) = load("menu") else {
        return;
    };
    assert_eq!(
        (plugin.info().name.as_str(), plugin.info().version.as_str()),
        ("Small stuff", "0.10")
    );
    assert!(plugin.init().unwrap());
    assert!(plugin.loaded().unwrap());
    assert_eq!(
        script.record().menu,
        [(
            MapMenu::FlightPlanner,
            None,
            "Fix mission top/bottom".to_owned()
        )]
    );
    // Another entry's click is not this plugin's.
    plugin.menu_click(9, 0.0, 0.0).unwrap();
    assert!(script.record().messages.is_empty());
    script.with(|r| r.input_answers.push_back(Some("90".to_owned())));
    plugin.menu_click(0, -35.0, 149.0).unwrap();
    let r = script.record();
    assert_eq!(
        r.messages[0].0,
        "This is a sample plugin\nSee the source in the plugins folder"
    );
    assert_eq!(
        r.inputs[0],
        (
            "Enter Angle".to_owned(),
            "This will be the heading".to_owned(),
            "0".to_owned()
        )
    );
    let inserted: Vec<(i32, u16, f64, f64)> = r
        .inserted
        .iter()
        .map(|(at, wp)| (*at, wp.command, wp.p1, wp.p2))
        .collect();
    assert_eq!(inserted, [(0, 183, 9.0, 90.0), (1, 183, 10.0, 1000.0)]);
    let added: Vec<(u16, f64, f64)> = r
        .added
        .iter()
        .map(|wp| (wp.command, wp.p1, wp.p2))
        .collect();
    assert_eq!(added, [(183, 9.0, 1000.0), (183, 10.0, 1000.0)]);
}

/// A heading that is not a number stops the handler, as `Int32.Parse`'s exception does.
#[test]
fn menu_stops_at_a_bad_heading() {
    let Some((mut plugin, script)) = load("menu") else {
        return;
    };
    plugin.loaded().unwrap();
    script.with(|r| r.input_answers.push_back(Some("east".to_owned())));
    plugin.menu_click(0, 0.0, 0.0).unwrap();
    assert!(script.record().inserted.is_empty());
    assert!(script.record().added.is_empty());
}

fn canberra(script: &Scripted) {
    script.with(|r| {
        r.cs.insert("lat".to_owned(), CsValue::Number(-35.3632));
        r.cs.insert("lng".to_owned(), CsValue::Number(149.1652));
    });
}

/// `example3-fencedist.cs`: an inclusion polygon around the vehicle - the southern side 311 m
/// away is the nearest - then an exclusion circle it is inside, which is a breach.
#[test]
fn fencedist_measures_to_the_nearest_fence() {
    let Some((mut plugin, script)) = load("fencedist") else {
        return;
    };
    plugin.init().unwrap();
    plugin.loaded().unwrap();
    assert_eq!(
        script.record().menu,
        [(MapMenu::FlightData, None, "Draw Fence Dist".to_owned())]
    );
    canberra(&script);
    script.with(|r| {
        r.fence = vec![
            fence(5000, 0.0, -35.3632, 149.1652),
            fence(5001, 4.0, -35.360, 149.160),
            fence(5001, 4.0, -35.360, 149.170),
            fence(5001, 4.0, -35.366, 149.170),
            fence(5001, 4.0, -35.366, 149.160),
        ];
    });
    plugin.menu_click(0, 0.0, 0.0).unwrap();
    let status = script.record().status.last().cloned().unwrap();
    let metres: f64 = status
        .trim_start_matches("Fence distance ")
        .trim_end_matches(" m")
        .parse()
        .unwrap();
    assert!((metres - 311.3).abs() < 1.0, "{status}");
    // An exclusion circle of 100 m round the vehicle: inside it is a breach, 0.
    script.with(|r| r.fence = vec![fence(5004, 100.0, -35.3632, 149.1652)]);
    plugin.menu_click(0, 0.0, 0.0).unwrap();
    assert_eq!(
        script.record().status.last().map(String::as_str),
        Some("Fence distance 0.0 m")
    );
    // No fence: 99999.
    script.with(|r| r.fence.clear());
    plugin.menu_click(0, 0.0, 0.0).unwrap();
    assert_eq!(
        script.record().status.last().map(String::as_str),
        Some("Fence distance 99999.0 m")
    );
}

/// `example6-mapicondesc.cs`: the entry is added in `Init`; its click offers the template and
/// keeps the answer; Cancel keeps nothing.
#[test]
fn mapicondesc_keeps_the_description() {
    let Some((mut plugin, script)) = load("mapicondesc") else {
        return;
    };
    assert!(plugin.init().unwrap());
    assert_eq!(script.record().menu.len(), 1);
    script.with(|r| r.input_answers.push_back(None));
    plugin.menu_click(0, 0.0, 0.0).unwrap();
    assert!(
        script.record().inputs[0]
            .2
            .starts_with("{alt}{altunit} {airspeed}")
    );
    assert!(!script.record().config.contains_key("mapicondesc"));
    script.with(|r| r.input_answers.push_back(Some("{alt}m".to_owned())));
    plugin.menu_click(0, 0.0, 0.0).unwrap();
    assert_eq!(script.record().config["mapicondesc"], "{alt}m");
    // Asked again, the kept one is offered.
    plugin.menu_click(0, 0.0, 0.0).unwrap();
    assert_eq!(script.record().inputs[2].2, "{alt}m");
}

/// `example8-modechange.cs`: never kept as shipped; driven anyway, its combo box follows the
/// vehicle's mode, lists FLTMODE1's modes, and sets the one chosen.
#[test]
fn modechange_follows_and_sets_the_mode() {
    let Some((mut plugin, script)) = load("modechange") else {
        return;
    };
    assert!(!plugin.init().unwrap());
    assert_eq!(plugin.loop_rate_hz(), 0.0);
    plugin.loaded().unwrap();
    assert_eq!(script.control("mode").unwrap().kind, ControlKind::ComboBox);
    script.with(|r| {
        r.cs.insert("connected".to_owned(), CsValue::Flag(true));
        r.cs.insert(
            "firmware".to_owned(),
            CsValue::Text("ArduCopter2".to_owned()),
        );
        r.cs.insert("mode".to_owned(), CsValue::Text("Stabilize".to_owned()));
        r.options.insert(
            "FLTMODE1".to_owned(),
            vec![(0, "Stabilize".to_owned()), (6, "RTL".to_owned())],
        );
    });
    plugin.run_loop().unwrap();
    // `loopratehz = 0.3f` at the end of `Loop`.
    assert_eq!(plugin.loop_rate_hz(), 0.3);
    let mode = script.control("mode").unwrap();
    assert_eq!(mode.value, "Stabilize");
    assert_eq!(mode.options, ["Stabilize", "RTL"]);
    plugin.form_event("mode", "RTL").unwrap();
    assert_eq!(script.record().modes, ["RTL"]);
    // Refused: the C#'s box.
    script.with(|r| r.set_mode_ok = false);
    plugin.form_event("mode", "Stabilize").unwrap();
    assert_eq!(script.record().messages[0].0, "Error: no response from MAV");
}

/// `example21-persistentsimple.cs`: three buttons, each its mode.
#[test]
fn persistentsimple_buttons_set_their_modes() {
    let Some((mut plugin, script)) = load("persistentsimple") else {
        return;
    };
    assert!(!plugin.init().unwrap());
    plugin.loaded().unwrap();
    let (title, controls) = script.record().form.clone().unwrap();
    assert_eq!(title, "Persistent Simple Actions");
    let buttons: Vec<&str> = controls.iter().map(|c| c.value.as_str()).collect();
    assert_eq!(buttons, ["Auto", "Loiter", "RTL"]);
    plugin.form_event("Loiter", "").unwrap();
    assert_eq!(script.record().modes, ["Loiter"]);
    script.with(|r| r.set_mode_ok = false);
    plugin.form_event("RTL", "").unwrap();
    assert_eq!(
        script.record().messages[0],
        (
            "The Command failed to execute\n".to_owned(),
            "Error".to_owned(),
            MessageButtons::Ok
        )
    );
}

/// `example22-payloadconfig.cs`: a payload whose parameters are at their values starts checked;
/// checking another writes its parameters; an armed vehicle refuses.
#[test]
fn payloadconfig_writes_a_payloads_parameters() {
    let Some((mut plugin, script)) = load("payloadconfig") else {
        return;
    };
    script.with(|r| {
        r.params.insert("MNT1_TYPE".to_owned(), 9.0);
        r.params.insert("CAM_TRIGG_TYPE".to_owned(), 0.0);
    });
    assert!(plugin.init().unwrap());
    plugin.loaded().unwrap();
    assert_eq!(script.control("Gimbal").unwrap().value, "true");
    assert_eq!(script.control("Mapping Camera").unwrap().value, "false");
    plugin.form_event("Mapping Camera", "true").unwrap();
    assert_eq!(
        script.record().param_writes,
        [
            ("CAM_TRIGG_TYPE".to_owned(), 1.0),
            ("SERVO9_FUNCTION".to_owned(), 10.0)
        ]
    );
    assert_eq!(script.control("Mapping Camera").unwrap().value, "true");
    // Armed: the box, and nothing written.
    script.with(|r| {
        r.cs.insert("armed".to_owned(), CsValue::Flag(true));
        r.param_writes.clear();
    });
    plugin.form_event("Gimbal", "false").unwrap();
    assert_eq!(
        script.record().messages.last().unwrap().0,
        "The vehicle is armed. Payload selection is disabled."
    );
    assert!(script.record().param_writes.is_empty());
    assert_eq!(script.control("Gimbal").unwrap().value, "true");
}

/// A message box's answer reaches the plugin: `DialogResult` both ways.
#[test]
fn a_yes_no_answer_reaches_the_plugin() {
    let Some((mut plugin, script)) = load("terrainmaker") else {
        return;
    };
    plugin.loaded().unwrap();
    script.with(|r| r.message_answers.push_back(DialogResult::No));
    plugin.menu_click(0, 0.0, 0.0).unwrap();
    let r = script.record();
    assert_eq!(r.messages[0].2, MessageButtons::YesNo);
    // No: no area, nothing asked after.
    assert!(r.inputs.is_empty());
}
