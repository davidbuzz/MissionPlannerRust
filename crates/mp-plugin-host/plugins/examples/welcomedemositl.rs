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

//! Welcome-Demo-Sitl, the owner's plugin (2026-10-05), not in the C#: a first visit shows what
//! the planner does. A drawn pointer moves as a hand would and clicks, in order, SIMULATION and
//! its Multirotor (a copter starts in the simulator and the planner connects to it), PLAN and
//! four waypoints around the copter (closing Drone ID's form first, which lies over Write),
//! Write, FLY and its Actions page, force arm, TakeOff and its prompt's OK, and Auto, so the
//! copter flies the mission; then the PLUGINS tab, where it unticks
//! its own Enabled box and saves, so the next start leaves it out, and back to FLY to watch.
//!
//! A plugin rather than part of the planner so a user can turn it off as any other. The browser
//! build carries it built in and enabled; the desktop does not, and runs it only from a file put
//! in the plugins folder.
//!
//! Every click is the host's `demo-click`: a real click on the control the planner names, which
//! the planner handles as a pilot's. Between clicks it waits for what the click was for - the
//! link and a position, the arming, the climb, the mode - and stops, leaving the planner as it
//! is and saying why on the status line, when that does not come. Once finished it asks for no
//! more loops, so it takes no time from the planner.

use std::sync::{Mutex, PoisonError};

use mp_plugins::host::{self, CsValue};
use mp_plugins::{Guest, cs_text};

/// How often the demo looks, a tick each time.
const RATE_HZ: f32 = 5.0;
/// Ticks in a second.
const TICKS: u32 = 5;
/// How long the pointer takes to reach a control, as a hand moves.
const MOVE_MS: u32 = 1000;
/// Ticks the pointer rests after a click, so a watcher sees what it did.
const DWELL: u32 = 3;
/// How long a control may take to come on screen before the demo stops, in seconds.
const SHOW_WAIT: u32 = 20;
/// Another plugin's message box: Drone ID's, at a start that has not yet saved its pages.
const QUESTION_OK: &str = "plugin-question-ok";
/// Drone ID's form's close box, named by the form's title.
const DRONE_ID_CLOSE: &str = "plugin-form-Drone_ID-close";
/// This plugin's Enabled box on the PLUGINS tab, named by its file.
const OWN_BOX: &str = "plugin-manager-enabled-welcomedemositl.wasm";
/// Half the mission square's side as a part of the vehicle's distance to the planning map's
/// nearest edge, so every corner is on screen and well inside it whatever the zoom...
const SQUARE_PART: f64 = 0.6;
/// ...and at most this, in metres: near enough to watch the copter fly it.
const SQUARE_MAX: f64 = 150.0;
/// Without the map's view, or this little room around the vehicle, the half side in metres.
const SQUARE_SMALL: f64 = 5.0;
/// Metres in a degree of latitude.
const METRES_PER_DEGREE: f64 = 111_320.0;
/// How often a copter that disarmed before it climbed is armed again: ArduCopter disarms a
/// landed copter after `DISARM_DELAY`, ten seconds, and a slow machine can take that long to
/// reach TakeOff.
const REARMS: u32 = 2;
/// The step that arms: where a copter that disarmed before it climbed goes back to.
const ARM_STEP: Act = Act::Click("force-arm");

/// What the demo waits for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Until {
    /// The control the planner names on screen: the planner up.
    Shown(&'static str),
    /// The link open, and a position from three-dimensional GPS with satellites enough.
    Located,
    /// The vehicle armed.
    Armed,
    /// The vehicle two metres up.
    Climbed,
    /// The vehicle in Auto.
    Auto,
}

/// One step of the demo.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Act {
    /// The pointer to the control the planner names, and a click on it.
    Click(&'static str),
    /// The same if the control comes on screen within so many seconds, else on without it: a
    /// prompt that is asked only the first time.
    Answer(&'static str, u32),
    /// The pointer to a corner of the mission square on the planning map, and a click there,
    /// which adds a waypoint.
    Waypoint(usize),
    /// Wait for something, at most so many seconds.
    Wait(Until, u32),
    /// A pause, in ticks.
    Pause(u32),
}

/// The demo, in order.
const SCRIPT: &[Act] = &[
    Act::Wait(Until::Shown("tab-simulation"), 120),
    Act::Pause(2 * TICKS),
    Act::Click("tab-simulation"),
    Act::Click("sitl-picture-quad"),
    Act::Wait(Until::Located, 180),
    Act::Pause(3 * TICKS),
    // Drone ID's form, shown once a vehicle is connected, lies over the planning screen's Write.
    Act::Answer(DRONE_ID_CLOSE, 2),
    Act::Click("tab-plan"),
    Act::Waypoint(0),
    Act::Waypoint(1),
    Act::Waypoint(2),
    Act::Waypoint(3),
    Act::Click("plan-write"),
    Act::Pause(3 * TICKS),
    Act::Click("tab-fly"),
    Act::Click("fly-tab-actions"),
    Act::Click("force-arm"),
    Act::Wait(Until::Armed, 20),
    Act::Click("takeoff"),
    // TakeOff asks the height the first time; later it sends the height given then.
    Act::Answer("fly-prompt-ok", 3),
    Act::Wait(Until::Climbed, 40),
    Act::Click("Auto"),
    Act::Wait(Until::Auto, 15),
    Act::Pause(2 * TICKS),
    Act::Click("tab-plugins"),
    Act::Click(OWN_BOX),
    Act::Click("plugin-manager-save"),
    Act::Click("tab-fly"),
];

/// Where the demo is.
#[derive(Debug)]
struct State {
    /// The step under way, an index into [`SCRIPT`].
    step: usize,
    /// Ticks spent on it.
    ticks: u32,
    /// Ticks still to rest before the next look.
    resting: u32,
    /// The mission square's corners, latitude and longitude, fixed at the first waypoint.
    corners: Option<[(f64, f64); 4]>,
    /// Times armed again after a disarm before the climb.
    rearmed: u32,
    /// Finished, or stopped: no more loops.
    finished: bool,
}

static STATE: Mutex<State> = Mutex::new(State {
    step: 0,
    ticks: 0,
    resting: 0,
    corners: None,
    rearmed: 0,
    finished: false,
});

impl State {
    /// On to the next step after a rest of `rest` ticks.
    fn next(&mut self, rest: u32) {
        self.step += 1;
        self.ticks = 0;
        self.resting = rest;
        self.finished = self.step >= SCRIPT.len();
    }

    /// The demo stops where it is, saying why.
    fn stop(&mut self, why: &str) {
        let text = format!("Welcome demo stopped: {why}");
        host::status(&text);
        host::log(&text);
        self.finished = true;
    }
}

/// A number the planner gives for `name`, zero when it gives none.
fn number(name: &str) -> f64 {
    match host::cs(name) {
        Some(CsValue::Number(value)) => value,
        _ => 0.0,
    }
}

/// Whether what the demo waits for has come.
fn arrived(until: Until) -> bool {
    match until {
        Until::Shown(control) => host::demo_visible(control),
        Until::Located => {
            matches!(host::cs("connected"), Some(CsValue::Flag(true)))
                && number("lat") != 0.0
                && number("satcount") >= 6.0
        }
        Until::Armed => matches!(host::cs("armed"), Some(CsValue::Flag(true))),
        Until::Climbed => number("alt") >= 2.0,
        Until::Auto => cs_text("mode").is_some_and(|mode| mode.eq_ignore_ascii_case("auto")),
    }
}

/// The mission square around the vehicle, its corners clockwise from the north-east, sized to
/// the planning map as last drawn: a part of the room between the vehicle and the map's nearest
/// edge, so a click on each corner lands on the map.
fn square() -> [(f64, f64); 4] {
    let (lat, lng) = (number("lat"), number("lng"));
    let metres_per_degree_east = METRES_PER_DEGREE * lat.to_radians().cos().max(0.01);
    let room = host::fp_view_area().map_or(0.0, |area| {
        let north_south = (area.top - lat).min(lat - area.bottom) * METRES_PER_DEGREE;
        let east_west = (area.right - lng).min(lng - area.left) * metres_per_degree_east;
        north_south.min(east_west)
    });
    let half = (room * SQUARE_PART).clamp(SQUARE_SMALL, SQUARE_MAX);
    let d_lat = half / METRES_PER_DEGREE;
    let d_lng = half / metres_per_degree_east;
    [
        (lat + d_lat, lng + d_lng),
        (lat - d_lat, lng + d_lng),
        (lat - d_lat, lng - d_lng),
        (lat + d_lat, lng - d_lng),
    ]
}

/// One look: the step under way advanced, or waited on.
fn tick(state: &mut State) {
    if state.finished || host::demo_busy() {
        return;
    }
    if state.resting > 0 {
        state.resting -= 1;
        return;
    }
    let Some(&act) = SCRIPT.get(state.step) else {
        state.finished = true;
        return;
    };
    state.ticks += 1;
    // A message box over the window takes the click; a user reads it and presses OK first.
    if matches!(act, Act::Click(_) | Act::Answer(..) | Act::Waypoint(_))
        && host::demo_visible(QUESTION_OK)
    {
        if host::demo_click(QUESTION_OK, MOVE_MS) {
            state.resting = DWELL;
        }
        return;
    }
    match act {
        Act::Click(control) => {
            if host::demo_click(control, MOVE_MS) {
                state.next(DWELL);
            } else if state.ticks > SHOW_WAIT * TICKS {
                state.stop(&format!("{control} not on screen"));
            }
        }
        Act::Answer(control, seconds) => {
            if host::demo_click(control, MOVE_MS) {
                state.next(DWELL);
            } else if state.ticks > seconds * TICKS {
                state.next(0);
            }
        }
        Act::Waypoint(corner) => {
            let corners = *state.corners.get_or_insert_with(square);
            let Some(&(lat, lng)) = corners.get(corner) else {
                state.stop("no such corner of the mission");
                return;
            };
            if host::demo_click_map(lat, lng, MOVE_MS) {
                state.next(DWELL);
            } else if state.ticks > SHOW_WAIT * TICKS {
                state.stop("the mission's corner is not on the map");
            }
        }
        Act::Wait(until, seconds) => {
            if arrived(until) {
                state.next(0);
            } else if until == Until::Climbed
                && state.ticks > 2 * TICKS
                && !arrived(Until::Armed)
                && state.rearmed < REARMS
                && let Some(arm) = SCRIPT.iter().position(|act| *act == ARM_STEP)
            {
                // Disarmed on the ground before the climb: armed again, as a user would.
                state.rearmed += 1;
                state.step = arm;
                state.ticks = 0;
                state.resting = DWELL;
            } else if state.ticks > seconds * TICKS {
                state.stop(match until {
                    Until::Shown(_) => "the planner did not come up",
                    Until::Located => "no vehicle with a position",
                    Until::Armed => "the vehicle did not arm",
                    Until::Climbed => "the vehicle did not climb",
                    Until::Auto => "the vehicle did not take Auto",
                });
            }
        }
        Act::Pause(ticks) => {
            if state.ticks >= ticks {
                state.next(0);
            }
        }
    }
}

struct WelcomeDemoSitl;

impl Guest for WelcomeDemoSitl {
    fn name() -> String {
        "Welcome-Demo-Sitl".to_owned()
    }

    fn version() -> String {
        "0.1".to_owned()
    }

    fn author() -> String {
        "MissionPlannerRust".to_owned()
    }

    fn loop_rate_hz() -> f32 {
        if STATE
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .finished
        {
            0.0
        } else {
            RATE_HZ
        }
    }

    fn init() -> bool {
        true
    }

    fn loaded() -> bool {
        true
    }

    fn run_loop() -> bool {
        tick(&mut STATE.lock().unwrap_or_else(PoisonError::into_inner));
        true
    }

    fn exit() -> bool {
        true
    }

    fn menu_click(_id: u32, _lat: f64, _lng: f64) {}

    fn form_event(_id: String, _value: String) {}
}

mp_plugins::export_plugin!(WelcomeDemoSitl);
