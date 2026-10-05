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

//! The owner's Welcome-Demo-Sitl plugin (2026-10-05), not in the C#, driven through the host
//! against a scripted screen: its clicks in order, the mission square, what it waits for, and
//! that it stops asking for loops when done. The browser build's check
//! (experiments/web-experiment/check/demo_check.js) runs it against the real planner.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

mod common;

use common::{Scripted, load};
use mp_plugin_host::{Area, CsValue, Plugin};

/// Every control the demo clicks, and the map.
const SCREEN: &[&str] = &[
    "tab-simulation",
    "sitl-picture-quad",
    "plugin-form-Drone_ID-close",
    "tab-plan",
    "map",
    "plan-write",
    "tab-fly",
    "fly-tab-actions",
    "force-arm",
    "takeoff",
    "fly-prompt-ok",
    "Auto",
    "tab-plugins",
    "plugin-manager-enabled-welcomedemositl.wasm",
    "plugin-manager-save",
];

/// The vehicle the simulator gives: Canberra's field, satellites enough.
const LAT: f64 = -35.363_262;
const LNG: f64 = 149.165_237;

/// The screen with every control on it, and a vehicle connected with a position.
fn everything_shown(script: &Scripted) {
    script.with(|r| {
        r.shown = SCREEN.iter().map(|name| (*name).to_owned()).collect();
        r.cs.insert("connected".to_owned(), CsValue::Flag(true));
        r.cs.insert("lat".to_owned(), CsValue::Number(LAT));
        r.cs.insert("lng".to_owned(), CsValue::Number(LNG));
        r.cs.insert("satcount".to_owned(), CsValue::Number(10.0));
    });
}

/// `n` of the demo's looks.
fn ticks(plugin: &mut Plugin, n: usize) {
    for _ in 0..n {
        plugin.run_loop().expect("the demo's loop runs");
    }
}

/// The demo, a look at a time, until it asks for no more or `limit` looks.
fn run_out(plugin: &mut Plugin, limit: usize) {
    for _ in 0..limit {
        if plugin.loop_rate_hz() == 0.0 {
            return;
        }
        plugin.run_loop().expect("the demo's loop runs");
    }
    panic!("the demo was still going after {limit} looks");
}

/// The owner's sequence: SIMULATION > Multirotor, PLAN and four waypoints, Write, FLY > Actions,
/// force arm, TakeOff and its OK, Auto, then its own box unticked and saved on PLUGINS, and back
/// to FLY; then no more loops.
#[test]
fn the_demo_clicks_the_owners_sequence_then_rests() {
    let Some((mut plugin, script)) = load("welcomedemositl") else {
        return;
    };
    assert_eq!(plugin.info().name, "Welcome-Demo-Sitl");
    assert!(plugin.init().unwrap());
    assert!(plugin.loaded().unwrap());
    assert!(plugin.loop_rate_hz() > 0.0);
    // The planner not up yet: nothing clicked.
    ticks(&mut plugin, 20);
    assert!(script.record().demo_clicks.is_empty());

    everything_shown(&script);
    script.with(|r| {
        r.cs.insert("armed".to_owned(), CsValue::Flag(true));
        r.cs.insert("alt".to_owned(), CsValue::Number(5.0));
        r.cs.insert("mode".to_owned(), CsValue::Text("Auto".to_owned()));
    });
    run_out(&mut plugin, 1000);

    let clicks = script.record().demo_clicks.clone();
    let controls: Vec<&str> = clicks
        .iter()
        .map(|click| {
            if click.starts_with("map ") {
                "map"
            } else {
                click
            }
        })
        .collect();
    assert_eq!(
        controls,
        [
            "tab-simulation",
            "sitl-picture-quad",
            "plugin-form-Drone_ID-close",
            "tab-plan",
            "map",
            "map",
            "map",
            "map",
            "plan-write",
            "tab-fly",
            "fly-tab-actions",
            "force-arm",
            "takeoff",
            "fly-prompt-ok",
            "Auto",
            "tab-plugins",
            "plugin-manager-enabled-welcomedemositl.wasm",
            "plugin-manager-save",
            "tab-fly",
        ]
    );
    assert_eq!(plugin.loop_rate_hz(), 0.0);
    assert!(
        script.record().status.is_empty(),
        "{:?}",
        script.record().status
    );
}

/// The four waypoints the demo clicks on a planning map showing `view`.
fn corners_on(view: Area) -> Vec<(f64, f64)> {
    let Some((mut plugin, script)) = load("welcomedemositl") else {
        return Vec::new();
    };
    assert!(plugin.init().unwrap());
    everything_shown(&script);
    script.with(|r| r.view_area = Some(view));
    let maps = |script: &Scripted| -> Vec<(f64, f64)> {
        script
            .record()
            .demo_clicks
            .iter()
            .filter_map(|click| {
                let (lat, lng) = click.strip_prefix("map ")?.split_once(',')?;
                Some((lat.parse().ok()?, lng.parse().ok()?))
            })
            .collect()
    };
    for _ in 0..400 {
        if maps(&script).len() == 4 {
            break;
        }
        plugin.run_loop().unwrap();
    }
    maps(&script)
}

/// The square's corners for a half side of `half` metres.
fn square(half: f64) -> [(f64, f64); 4] {
    let d_lat = half / 111_320.0;
    let d_lng = half / (111_320.0 * LAT.to_radians().cos());
    [
        (LAT + d_lat, LNG + d_lng),
        (LAT - d_lat, LNG + d_lng),
        (LAT - d_lat, LNG - d_lng),
        (LAT + d_lat, LNG - d_lng),
    ]
}

/// The waypoints: a square around the vehicle, clockwise from the north-east, its half side
/// 0.6 of the room to the planning map's nearest edge - so every corner is on the map, as at
/// zoom 19.7, where the first try put one above it - and at most 150 metres.
#[test]
fn the_mission_is_a_square_around_the_vehicle_inside_the_map() {
    // 0.0005 degrees to the top and bottom edges, 55.66 m; 0.001 east and west, 90.8 m: a half
    // side of 0.6 of 55.66 m.
    let corners = corners_on(Area {
        top: LAT + 0.0005,
        bottom: LAT - 0.0005,
        left: LNG - 0.001,
        right: LNG + 0.001,
    });
    if corners.is_empty() {
        return;
    }
    for ((lat, lng), (want_lat, want_lng)) in corners.iter().zip(square(0.6 * 55.66)) {
        assert!((lat - want_lat).abs() < 2e-6, "{lat} against {want_lat}");
        assert!((lng - want_lng).abs() < 2e-6, "{lng} against {want_lng}");
    }
    // A map kilometres across: 150 m.
    let corners = corners_on(Area {
        top: LAT + 0.05,
        bottom: LAT - 0.05,
        left: LNG - 0.05,
        right: LNG + 0.05,
    });
    assert_eq!(corners.len(), 4);
    for ((lat, lng), (want_lat, want_lng)) in corners.iter().zip(square(150.0)) {
        assert!((lat - want_lat).abs() < 2e-6, "{lat} against {want_lat}");
        assert!((lng - want_lng).abs() < 2e-6, "{lng} against {want_lng}");
    }
}

/// Between clicks it waits for what they were for: no TakeOff before the vehicle is armed, and
/// when it does not arm the demo stops, saying why on the status line, and asks for no more loops.
#[test]
fn the_demo_waits_for_the_arming_and_stops_when_it_does_not_come() {
    let Some((mut plugin, script)) = load("welcomedemositl") else {
        return;
    };
    assert!(plugin.init().unwrap());
    everything_shown(&script);
    let last = |script: &Scripted| script.record().demo_clicks.last().cloned();
    for _ in 0..1000 {
        if last(&script).as_deref() == Some("force-arm") {
            break;
        }
        plugin.run_loop().unwrap();
    }
    assert_eq!(last(&script).as_deref(), Some("force-arm"));
    // Twenty seconds at five looks a second, less a few: still waiting, nothing more clicked.
    ticks(&mut plugin, 90);
    assert_eq!(last(&script).as_deref(), Some("force-arm"));
    assert!(plugin.loop_rate_hz() > 0.0);
    ticks(&mut plugin, 20);
    assert_eq!(last(&script).as_deref(), Some("force-arm"));
    assert_eq!(
        script.record().status,
        ["Welcome demo stopped: the vehicle did not arm"]
    );
    assert_eq!(plugin.loop_rate_hz(), 0.0);
}

/// A message box over the window - Drone ID's at a start with no pages saved - takes the first
/// click, as a user reads it and presses OK; and nothing moves while a gesture is under way.
#[test]
fn a_message_box_is_answered_first_and_a_gesture_is_waited_for() {
    let Some((mut plugin, script)) = load("welcomedemositl") else {
        return;
    };
    assert!(plugin.init().unwrap());
    everything_shown(&script);
    script.with(|r| {
        r.shown.insert("plugin-question-ok".to_owned());
        r.demo_busy = true;
    });
    ticks(&mut plugin, 50);
    assert!(script.record().demo_clicks.is_empty());
    script.with(|r| r.demo_busy = false);
    for _ in 0..50 {
        if !script.record().demo_clicks.is_empty() {
            break;
        }
        plugin.run_loop().unwrap();
    }
    assert_eq!(script.record().demo_clicks, ["plugin-question-ok"]);
    // Answered, the box goes, and the demo goes on with its own first click.
    script.with(|r| {
        r.shown.remove("plugin-question-ok");
    });
    for _ in 0..50 {
        if script.record().demo_clicks.len() == 2 {
            break;
        }
        plugin.run_loop().unwrap();
    }
    assert_eq!(
        script.record().demo_clicks,
        ["plugin-question-ok", "tab-simulation"]
    );
}

/// A copter that disarms on the ground before it climbs - ArduCopter's `DISARM_DELAY`, when the
/// pointer is slow to reach TakeOff - is armed again and TakeOff pressed again, its prompt not
/// asked a second time; then the demo goes on to Auto.
#[test]
fn a_copter_that_disarms_before_climbing_is_armed_again() {
    let Some((mut plugin, script)) = load("welcomedemositl") else {
        return;
    };
    assert!(plugin.init().unwrap());
    everything_shown(&script);
    let last = |script: &Scripted| script.record().demo_clicks.last().cloned();
    let until = |plugin: &mut Plugin, script: &Scripted, control: &str| {
        for _ in 0..1000 {
            if last(script).as_deref() == Some(control) {
                return;
            }
            plugin.run_loop().unwrap();
        }
        panic!("no click on {control}");
    };
    until(&mut plugin, &script, "force-arm");
    script.with(|r| {
        r.cs.insert("armed".to_owned(), CsValue::Flag(true));
    });
    until(&mut plugin, &script, "fly-prompt-ok");
    // Disarmed before the climb; the prompt is not asked again.
    script.with(|r| {
        r.cs.insert("armed".to_owned(), CsValue::Flag(false));
        r.shown.remove("fly-prompt-ok");
    });
    until(&mut plugin, &script, "force-arm");
    script.with(|r| {
        r.cs.insert("armed".to_owned(), CsValue::Flag(true));
    });
    until(&mut plugin, &script, "takeoff");
    // Three seconds for a prompt that does not come, then the climb.
    ticks(&mut plugin, 25);
    assert_eq!(last(&script).as_deref(), Some("takeoff"));
    script.with(|r| {
        r.cs.insert("alt".to_owned(), CsValue::Number(5.0));
    });
    until(&mut plugin, &script, "Auto");
    let clicks = script.record().demo_clicks.clone();
    let tail: Vec<&str> = clicks
        .iter()
        .rev()
        .take(6)
        .rev()
        .map(String::as_str)
        .collect();
    assert_eq!(
        tail,
        [
            "force-arm",
            "takeoff",
            "fly-prompt-ok",
            "force-arm",
            "takeoff",
            "Auto"
        ]
    );
    assert!(script.record().status.is_empty());
}
