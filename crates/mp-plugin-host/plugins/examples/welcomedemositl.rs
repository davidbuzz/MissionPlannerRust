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
//! its Multirotor (a copter starts in the simulator and the planner connects to it), PLAN, its zoom
//! icon's Zoom To Vehicle (and the Zoom box's down arrow while the map is too near for the
//! survey), Set Home Here on the map's right-click menu at the copter, four
//! waypoints around the copter, a survey - Polygon > Draw a Polygon on the map's menu, four
//! corners of a square as big to the left, Auto WP > Survey (Grid) and its Accept - Write, FLY and
//! its Actions page, force arm, TakeOff and its prompt's OK, and Auto, so the copter flies the
//! mission; then the PLUGINS tab, where it unticks
//! its own Enabled box and saves, so the next start leaves it out, and back to FLY to watch, where
//! its pointer goes.
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
/// The planner's own message box's OK: what Write says when it will not write.
const PLAN_PROMPT_OK: &str = "plan-prompt-ok";
/// The Survey (Grid) dialog's message box's OK: what its Accept says when it will not add.
const SURVEY_PROMPT_OK: &str = "survey-prompt-ok";
/// The planning map taller than this, in metres, is too far out for waypoints a click apart: Zoom
/// To Vehicle brings it in to zoom 17, a few hundred metres.
const VIEW_MOST: f64 = 5_000.0;
/// This plugin's Enabled box on the PLUGINS tab, named by its file.
const OWN_BOX: &str = "plugin-manager-enabled-welcomedemositl.wasm";
/// Half the mission square's side as a part of the vehicle's distance to the planning map's
/// nearest edge - to the west, the survey's square's far side - so every corner is on screen and
/// well inside it whatever the zoom...
const SQUARE_PART: f64 = 0.6;
/// The survey's square: as big as the mission's, its centre this many half sides west of the
/// copter, so a half side clear of the mission's square.
const SURVEY_OFFSET: f64 = 3.0;
/// ...and at most this, in metres: near enough to watch the copter fly it.
const SQUARE_MAX: f64 = 150.0;
/// Without the map's view, or this little room around the vehicle, the half side in metres.
const SQUARE_SMALL: f64 = 5.0;
/// The half side, in metres, the survey needs at least: its lines are 50 m apart by default
/// (`NUM_Distance`, Grid/GridUI.Designer.cs), and a square under about two of them across gets
/// none - "Bad Grid" (the browser, 2026-10-05: its planning map, already nearer than 17, which
/// Zoom To Vehicle leaves as it is, gave squares of a few tens of metres).
const SQUARE_LEAST: f64 = 60.0;
/// The Zoom box's down arrow, half a zoom level a press (`Zoomlevel.Increment`).
const ZOOM_OUT: &str = "plan-zoomlevel-down";
/// Presses of it at most: six zoom levels.
const ZOOM_OUTS: u32 = 12;
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

/// The two squares on the planning map.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Square {
    /// The four waypoints', around the copter.
    Mission,
    /// The survey polygon's, to the left of the mission's.
    Survey,
}

/// Where on the planning map a right click opens its menu.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Spot {
    /// The copter.
    Vehicle,
    /// The middle of the survey's square.
    Survey,
}

/// One step of the demo.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Act {
    /// The pointer to the control the planner names, and a click on it.
    Click(&'static str),
    /// The same if the control comes on screen within so many seconds, else on without it: a
    /// prompt that is asked only the first time.
    Answer(&'static str, u32),
    /// The pointer to a corner of a square on the planning map, and a click there, which adds a
    /// waypoint - or, once Draw a Polygon has been chosen, a corner of the polygon.
    Corner(Square, usize),
    /// The pointer to a spot on the map, and a right click there: the map's menu.
    RightClick(Spot),
    /// The Zoom box's down arrow, pressed while the map is too near for squares of
    /// [`SQUARE_LEAST`].
    WidenForSquares,
    /// Wait for something, at most so many seconds.
    Wait(Until, u32),
    /// A pause, in ticks.
    Pause(u32),
    /// The answer to the click before: a message box from the planner or the survey dialog
    /// within so many seconds is a refusal - Write's "Your home location is invalid", say -
    /// answered, and the demo stops, saying what did not happen.
    Refusal(u32, &'static str),
}

/// The demo, in order.
const SCRIPT: &[Act] = &[
    Act::Wait(Until::Shown("tab-simulation"), 120),
    Act::Pause(2 * TICKS),
    Act::Click("tab-simulation"),
    Act::Click("sitl-picture-quad"),
    Act::Wait(Until::Located, 180),
    Act::Pause(3 * TICKS),
    Act::Click("tab-plan"),
    // Zoom To Vehicle, on the planning map's zoom icon: the map on the copter, in to 17 - the
    // planning map starts out at zoom 3, as the C#'s does.
    // `// C#: GCSViews/FlightPlanner.cs:232, 8366-8377`
    Act::Click("plan-zoomicon"),
    Act::Click("menu-zoomToVehicle"),
    Act::Pause(TICKS),
    Act::WidenForSquares,
    // Set Home Here, on the map's right-click menu at the copter: the home Write sends, without
    // which it says "Your home location is invalid" (the owner's run, 2026-10-05).
    Act::RightClick(Spot::Vehicle),
    Act::Click("menu-setHomeHere"),
    Act::Pause(TICKS),
    Act::Corner(Square::Mission, 0),
    Act::Corner(Square::Mission, 1),
    Act::Corner(Square::Mission, 2),
    Act::Corner(Square::Mission, 3),
    // A survey (the owner's word, 2026-10-05): Polygon > Draw a Polygon on the map's menu, which
    // makes the map's clicks the polygon's corners; four of them, in a square to the left of the
    // mission's; then Auto WP > Survey (Grid) over it, and Accept, which adds the grid's rows
    // after the four waypoints and closes the dialog.
    // `// C#: GCSViews/FlightPlanner.cs:1731-1758, 6752-6757; Grid/GridUI.cs:1604`
    Act::RightClick(Spot::Survey),
    Act::Click("menu-polygon"),
    Act::Click("menu-addPolygonPoint2"),
    Act::Corner(Square::Survey, 0),
    Act::Corner(Square::Survey, 1),
    Act::Corner(Square::Survey, 2),
    Act::Corner(Square::Survey, 3),
    Act::RightClick(Spot::Survey),
    Act::Click("menu-autoWP"),
    Act::Click("menu-surveyGrid"),
    Act::Click("survey-BUT_Accept"),
    Act::Refusal(2, "the survey's Accept did not add its grid"),
    Act::Click("plan-write"),
    Act::Refusal(4, "Write did not write the mission"),
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
    /// The two squares' corners, latitude and longitude, fixed at the first waypoint.
    squares: Option<Squares>,
    /// Times armed again after a disarm before the climb.
    rearmed: u32,
    /// Presses of the Zoom box's down arrow.
    zoom_outs: u32,
    /// Finished, or stopped: no more loops.
    finished: bool,
}

static STATE: Mutex<State> = Mutex::new(State {
    step: 0,
    ticks: 0,
    resting: 0,
    squares: None,
    rearmed: 0,
    zoom_outs: 0,
    finished: false,
});

impl State {
    /// On to the next step after a rest of `rest` ticks; past the last, the demo is over and its
    /// pointer goes.
    fn next(&mut self, rest: u32) {
        self.step += 1;
        self.ticks = 0;
        self.resting = rest;
        self.finished = self.step >= SCRIPT.len();
        if self.finished {
            host::demo_end();
        }
    }

    /// The demo stops where it is, saying why, and its pointer goes.
    fn stop(&mut self, why: &str) {
        let text = format!("Welcome demo stopped: {why}");
        host::status(&text);
        host::log(&text);
        self.finished = true;
        host::demo_end();
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

/// The planning map's height as last drawn, in metres; none known, as tall as can be.
fn view_height() -> f64 {
    host::fp_view_area().map_or(f64::MAX, |area| {
        (area.top - area.bottom).abs() * METRES_PER_DEGREE
    })
}

/// The two squares, each's corners clockwise from the north-east.
#[derive(Debug, Clone, Copy)]
struct Squares {
    mission: [(f64, f64); 4],
    survey: [(f64, f64); 4],
    /// The survey square's middle.
    survey_middle: (f64, f64),
}

impl Squares {
    fn corners(&self, square: Square) -> &[(f64, f64); 4] {
        match square {
            Square::Mission => &self.mission,
            Square::Survey => &self.survey,
        }
    }
}

/// Metres east in a degree of longitude at `lat`.
fn metres_per_degree_east(lat: f64) -> f64 {
    METRES_PER_DEGREE * lat.to_radians().cos().max(0.01)
}

/// The squares' half side the planning map as last drawn has room for, in metres, before the
/// limits: a part of the room between the vehicle and the map's edges - north, south and east
/// the mission square's, west the survey square's far side - so a click on each corner lands on
/// the map. None known, none.
fn room_for_half() -> f64 {
    let (lat, lng) = (number("lat"), number("lng"));
    let east_metres = metres_per_degree_east(lat);
    host::fp_view_area().map_or(0.0, |area| {
        let north_south = (area.top - lat).min(lat - area.bottom) * METRES_PER_DEGREE;
        let east = (area.right - lng) * east_metres;
        let west = (lng - area.left) * east_metres / (SURVEY_OFFSET + 1.0);
        north_south.min(east).min(west) * SQUARE_PART
    })
}

/// The mission square around the vehicle and the survey's to its left: [`room_for_half`], at
/// most [`SQUARE_MAX`].
fn squares() -> Squares {
    let (lat, lng) = (number("lat"), number("lng"));
    let metres_per_degree_east = metres_per_degree_east(lat);
    let half = room_for_half().clamp(SQUARE_SMALL, SQUARE_MAX);
    let d_lat = half / METRES_PER_DEGREE;
    let d_lng = half / metres_per_degree_east;
    let around = |lng: f64| {
        [
            (lat + d_lat, lng + d_lng),
            (lat - d_lat, lng + d_lng),
            (lat - d_lat, lng - d_lng),
            (lat + d_lat, lng - d_lng),
        ]
    };
    let survey_lng = lng - SURVEY_OFFSET * d_lng;
    Squares {
        mission: around(lng),
        survey: around(survey_lng),
        survey_middle: (lat, survey_lng),
    }
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
    if matches!(
        act,
        Act::Click(_)
            | Act::Answer(..)
            | Act::Corner(..)
            | Act::RightClick(_)
            | Act::WidenForSquares
    ) && host::demo_visible(QUESTION_OK)
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
        Act::Corner(square, corner) => {
            if state.squares.is_none() && view_height() > VIEW_MOST {
                state.stop("the planning map is too far out to place waypoints");
                return;
            }
            let squares = *state.squares.get_or_insert_with(squares);
            let Some(&(lat, lng)) = squares.corners(square).get(corner) else {
                state.stop("no such corner");
                return;
            };
            if host::demo_click_map(lat, lng, MOVE_MS) {
                state.next(DWELL);
            } else if state.ticks > SHOW_WAIT * TICKS {
                state.stop(match square {
                    Square::Mission => "the mission's corner is not on the map",
                    Square::Survey => "the survey's corner is not on the map",
                });
            }
        }
        Act::WidenForSquares => {
            if room_for_half() >= SQUARE_LEAST {
                state.next(0);
            } else if state.zoom_outs >= ZOOM_OUTS {
                state.stop("the planning map would not zoom out far enough for the survey");
            } else if host::demo_click(ZOOM_OUT, MOVE_MS) {
                // Looked at again once the map has drawn at the new zoom.
                state.zoom_outs += 1;
                state.resting = DWELL;
            } else if state.ticks > SHOW_WAIT * TICKS {
                state.stop(&format!("{ZOOM_OUT} not on screen"));
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
        Act::RightClick(spot) => {
            let (lat, lng) = match spot {
                Spot::Vehicle => (number("lat"), number("lng")),
                Spot::Survey => state.squares.get_or_insert_with(squares).survey_middle,
            };
            if host::demo_right_click_map(lat, lng, MOVE_MS) {
                state.next(DWELL);
            } else if state.ticks > SHOW_WAIT * TICKS {
                state.stop(match spot {
                    Spot::Vehicle => "the copter is not on the map",
                    Spot::Survey => "the survey's square is not on the map",
                });
            }
        }
        Act::Refusal(seconds, what) => {
            if let Some(ok) = [PLAN_PROMPT_OK, SURVEY_PROMPT_OK]
                .into_iter()
                .find(|ok| host::demo_visible(ok))
            {
                let _ = host::demo_click(ok, MOVE_MS);
                state.stop(what);
            } else if state.ticks > seconds * TICKS {
                state.next(0);
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
