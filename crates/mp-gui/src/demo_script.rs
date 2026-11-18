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

//! The demo, as a list anyone can edit: `tools/demo/owner.gui`, a step a line, played by `planner
//! --demo` with the drawn pointer (demo_pointer.rs) - the owner's word of 2026-11-18: "i would like
//! this editable-list-of-clicks to be the entire demo, and for it to start-with whats in the demo
//! right now". Until then `--demo` ran the Welcome-Demo-Sitl plugin, its steps fixed in its source,
//! and an outside script (tools/demo/tour.py) played the file after it with the real pointer. Now
//! the file is the whole of it, its first lines the plugin's steps written out, and nothing
//! outside the planner plays anything. Not in the C#.
//!
//! The lines, each what `--record` prints or one of the few written by hand:
//!
//! * `click <control>` - the pointer to the control the planner names, and a click. It waits for
//!   the control to come on screen, twenty seconds at most.
//! * `click map@0.40x0.40` - a point in a map, across then down as parts of its box; `:right`
//!   after any `click` is the right button. These are as `--record` prints them.
//! * `click map@vehicle` and `click map@vehicle+80,-80` - the vehicle's place on the map, and a
//!   point so many metres east and north of it: where the copter is, whatever the map shows.
//! * `screen <name>` - wait for that screen to be the one showing (`--record` prints it when a
//!   screen is chosen).
//! * `key <name>` - a key, `Escape`, `Return`, `ctrl+f`; `type <text>` - text into what has the
//!   keyboard.
//! * `wait <control> [seconds]` - wait for a control to come on screen; `wait located`, `wait
//!   armed`, `wait climbed` and `wait auto` wait for the vehicle: a link and a three-dimensional
//!   fix, the arming, two metres up, the mode Auto.
//! * `answer <control> [seconds]` - click it if it comes within so long, else go on: a question
//!   asked only the first time.
//! * `pause <seconds>`.
//!
//! A `#` starts a comment. A `click` whose control does not come within five seconds, or a
//! `screen` that does not show, is passed over and the demo goes on - a list recorded in one
//! window is played in another, and the owner's word on the tour it replaces was that "'not on
//! screen' is a poor choice for a demo" (2026-10-17). A `wait` that does not come stops the demo
//! where it is, the planner left as it stands and the status line saying which line and why: a
//! copter that has not armed is not sent on.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::Lock as _;
use std::sync::Mutex;
use web_time::{Duration, Instant};

/// The demo's file, from where the planner is started.
pub const FILE: &str = "tools/demo/owner.gui";
/// The harness's door: another list than [`FILE`], for a script that plays a short one.
pub const FILE_ENV: &str = "MP_DEMO_FILE";
/// The demo as it was when the planner was built: played where [`FILE`] is not to be found.
pub const BUILT_IN: &str = include_str!("../../../tools/demo/owner.gui");
/// How long a `wait` waits when its line does not say.
pub const SHOW_WAIT: Duration = Duration::from_secs(20);
/// How long a `click` waits for its control, and a `screen` for its screen, before the line is
/// passed over.
pub const CLICK_WAIT: Duration = Duration::from_secs(5);
/// How long the pointer takes to reach a control, as a hand moves.
pub const MOVE_MS: u32 = 1000;
/// The rest after a click, so a watcher sees what it did.
pub const DWELL: Duration = Duration::from_millis(600);

/// Where in a control a click lands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Place {
    /// The middle of what shows of it.
    Centre,
    /// A point across and down its box, as parts of it.
    Part(f32, f32),
    /// On a map: metres east and north of the vehicle.
    Vehicle(f64, f64),
}

/// What a `wait` waits for.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Until {
    /// A control on screen.
    Shown(String),
    /// The link open, and a position from three-dimensional GPS with satellites enough.
    Located,
    /// The vehicle armed.
    Armed,
    /// The vehicle two metres up.
    Climbed,
    /// The vehicle in Auto.
    Auto,
}

/// One line of the demo.
#[derive(Debug, Clone, PartialEq)]
pub enum Step {
    /// `screen <name>`.
    Screen(String),
    /// `click <control>[@place][:right]`.
    Click {
        /// The control's name, as the probe has it.
        control: String,
        /// Where in it.
        place: Place,
        /// The right button.
        right: bool,
    },
    /// `key <name>`.
    Key(String),
    /// `type <text>`.
    Type(String),
    /// `wait <what> [seconds]`.
    Wait(Until, Duration),
    /// `answer <control> [seconds]`.
    Answer(String, Duration),
    /// `pause <seconds>`.
    Pause(Duration),
}

/// Seconds, as a line writes them.
fn seconds(text: &str) -> Option<Duration> {
    let value: f32 = text.parse().ok()?;
    (value.is_finite() && (0.0..=3600.0).contains(&value)).then(|| Duration::from_secs_f32(value))
}

/// `click`'s target: `name`, `name@0.40x0.40`, `name@vehicle`, `name@vehicle+80,-80`, each with
/// `:right` after it for the right button.
fn click(target: &str) -> Option<Step> {
    let (target, right) = match target.strip_suffix(":right") {
        Some(rest) => (rest, true),
        None => (target, false),
    };
    let (control, place) = match target.split_once('@') {
        None => (target, Place::Centre),
        Some((control, "vehicle")) => (control, Place::Vehicle(0.0, 0.0)),
        Some((control, at)) => match at.strip_prefix("vehicle") {
            Some(offset) => {
                let (east, north) = offset.split_once(',')?;
                let metres = |text: &str| -> Option<f64> {
                    let value: f64 = text.parse().ok()?;
                    (text.starts_with(['+', '-']) && value.is_finite()).then_some(value)
                };
                (control, Place::Vehicle(metres(east)?, metres(north)?))
            }
            None => {
                let (across, down) = at.split_once('x')?;
                let part = |text: &str| -> Option<f32> {
                    let value: f32 = text.parse().ok()?;
                    (0.0..=1.0).contains(&value).then_some(value)
                };
                (control, Place::Part(part(across)?, part(down)?))
            }
        },
    };
    (!control.is_empty() && !control.contains(char::is_whitespace)).then(|| Step::Click {
        control: control.to_owned(),
        place,
        right,
    })
}

/// The steps of a demo file, each with the number of its line.
///
/// # Errors
///
/// The first line that is no step, by its number and its text.
pub fn parse(text: &str) -> Result<Vec<(usize, Step)>, String> {
    let mut steps = Vec::new();
    for (index, raw) in text.lines().enumerate() {
        let line = raw.split('#').next().unwrap_or_default().trim();
        if line.is_empty() {
            continue;
        }
        let (verb, rest) = line.split_once(char::is_whitespace).unwrap_or((line, ""));
        let rest = rest.trim();
        let mut words = rest.split_whitespace();
        let step = match verb {
            "screen" if !rest.is_empty() && words.clone().count() == 1 => {
                Some(Step::Screen(rest.to_owned()))
            }
            "click" if words.clone().count() == 1 => click(rest),
            "key" if !rest.is_empty() && words.clone().count() == 1 => {
                Some(Step::Key(rest.to_owned()))
            }
            "type" if !rest.is_empty() => Some(Step::Type(rest.to_owned())),
            "pause" if words.clone().count() == 1 => seconds(rest).map(Step::Pause),
            "wait" | "answer" => {
                let what = words.next();
                let limit = match words.next() {
                    Some(text) => seconds(text),
                    None => Some(SHOW_WAIT),
                };
                match (what, limit, words.next()) {
                    (Some(what), Some(limit), None) if verb == "answer" => {
                        Some(Step::Answer(what.to_owned(), limit))
                    }
                    (Some(what), Some(limit), None) => Some(Step::Wait(
                        match what {
                            "located" => Until::Located,
                            "armed" => Until::Armed,
                            "climbed" => Until::Climbed,
                            "auto" => Until::Auto,
                            control => Until::Shown(control.to_owned()),
                        },
                        limit,
                    )),
                    _ => None,
                }
            }
            _ => None,
        };
        match step {
            Some(step) => steps.push((index + 1, step)),
            None => {
                return Err(format!(
                    "line {}: not a step of the demo: {}",
                    index + 1,
                    raw.trim()
                ));
            }
        }
    }
    Ok(steps)
}

/// The planner, as the demo sees it and acts on it.
pub trait World {
    /// Whether a control is on screen.
    fn visible(&self, control: &str) -> bool;
    /// The screen showing, by its tab's name.
    fn screen(&self) -> &str;
    /// Whether what a `wait` waits for of the vehicle has come.
    fn vehicle(&self, until: &Until) -> bool;
    /// Whether the pointer is still moving or typing.
    fn busy(&self) -> bool;
    /// The pointer to a place and a click there; false while the place is not on screen.
    fn click(&mut self, control: &str, place: Place, right: bool) -> bool;
    /// A key pressed.
    fn key(&mut self, name: &str);
    /// Text typed.
    fn type_text(&mut self, text: &str);
    /// The status line.
    fn status(&mut self, text: &str);
    /// The demo is over: the pointer goes.
    fn end(&mut self);
}

/// The demo being played.
#[derive(Debug)]
pub struct Player {
    steps: Vec<(usize, Step)>,
    /// The step under way.
    at: usize,
    /// When it began.
    since: Option<Instant>,
    /// Its click has been asked for.
    clicked: bool,
    /// A rest before the next step.
    resting: Option<Instant>,
    /// Finished, or stopped and why.
    ended: Option<Result<(), String>>,
    /// The lines passed over: a control that did not come, a screen that did not show.
    skipped: Vec<usize>,
}

impl Player {
    /// A demo of those steps, from its first.
    #[must_use]
    pub const fn new(steps: Vec<(usize, Step)>) -> Self {
        Self {
            steps,
            at: 0,
            since: None,
            clicked: false,
            resting: None,
            ended: None,
            skipped: Vec::new(),
        }
    }

    /// The lines passed over so far, for the facts: `none`, or their numbers.
    #[must_use]
    pub fn skipped(&self) -> String {
        if self.skipped.is_empty() {
            return "none".to_owned();
        }
        let lines: Vec<String> = self.skipped.iter().map(usize::to_string).collect();
        lines.join(" ")
    }

    /// A line whose control or screen did not come: noted, and on to the next.
    fn pass_over(&mut self, line: usize, what: &str, now: Instant) {
        log::info!("the demo passes over line {line} of {FILE}: {what}");
        self.skipped.push(line);
        self.next(Duration::ZERO, now);
    }

    /// Whether there are steps still to play.
    #[must_use]
    pub const fn running(&self) -> bool {
        self.ended.is_none()
    }

    /// Where the demo is, for the facts: the line under way, `done`, or `stopped: <why>`.
    #[must_use]
    pub fn progress(&self) -> String {
        match &self.ended {
            Some(Ok(())) => "done".to_owned(),
            Some(Err(why)) => format!("stopped: {why}"),
            None => self
                .steps
                .get(self.at)
                .map_or_else(|| "done".to_owned(), |(line, _)| format!("line {line}")),
        }
    }

    fn next(&mut self, rest: Duration, now: Instant) {
        self.at += 1;
        self.since = None;
        self.clicked = false;
        self.resting = (!rest.is_zero()).then(|| now + rest);
    }

    fn stop(&mut self, line: usize, why: &str, world: &mut dyn World) {
        let text = format!("Demo stopped at line {line} of {FILE}: {why}");
        world.status(&text);
        world.end();
        self.ended = Some(Err(text));
    }

    /// Once a frame: the step under way looked at, and done when it can be.
    pub fn tick(&mut self, now: Instant, world: &mut dyn World) {
        if self.ended.is_some() || world.busy() {
            return;
        }
        if let Some(until) = self.resting {
            if now < until {
                return;
            }
            self.resting = None;
        }
        let Some((line, step)) = self.steps.get(self.at).cloned() else {
            world.end();
            self.ended = Some(Ok(()));
            return;
        };
        let waited = now.saturating_duration_since(*self.since.get_or_insert(now));
        match step {
            Step::Screen(name) => {
                if world.screen() == name {
                    self.next(Duration::ZERO, now);
                } else if waited > CLICK_WAIT {
                    self.pass_over(line, &format!("the {name} screen did not show"), now);
                }
            }
            Step::Click {
                control,
                place,
                right,
            } => {
                if self.clicked {
                    // The pointer has been there and clicked: on, after a rest.
                    self.next(DWELL, now);
                } else if world.click(&control, place, right) {
                    self.clicked = true;
                } else if waited > CLICK_WAIT {
                    self.pass_over(line, &format!("{control} did not come on screen"), now);
                }
            }
            Step::Answer(control, limit) => {
                if self.clicked {
                    self.next(DWELL, now);
                } else if world.click(&control, Place::Centre, false) {
                    self.clicked = true;
                } else if waited > limit {
                    // Not asked this time.
                    self.next(Duration::ZERO, now);
                }
            }
            Step::Key(name) => {
                world.key(&name);
                self.next(DWELL, now);
            }
            Step::Type(text) => {
                world.type_text(&text);
                self.next(DWELL, now);
            }
            Step::Wait(until, limit) => {
                let come = match &until {
                    Until::Shown(control) => world.visible(control),
                    vehicle => world.vehicle(vehicle),
                };
                if come {
                    self.next(Duration::ZERO, now);
                } else if waited > limit {
                    let what = match until {
                        Until::Shown(control) => format!("{control} did not come on screen"),
                        Until::Located => {
                            "the vehicle did not connect and find its position".to_owned()
                        }
                        Until::Armed => "the vehicle did not arm".to_owned(),
                        Until::Climbed => "the vehicle did not climb".to_owned(),
                        Until::Auto => "the vehicle did not go into Auto".to_owned(),
                    };
                    self.stop(line, &what, world);
                }
            }
            Step::Pause(length) => {
                if waited >= length {
                    self.next(Duration::ZERO, now);
                }
            }
        }
    }
}

/// The demo `--demo` asked for, kept from the command line until the window is made.
static ASKED: Mutex<Option<Vec<(usize, Step)>>> = Mutex::new(None);

/// The demo's steps: [`FILE`] where it is, read from where the planner was started, else the
/// copy built in. The harness's door names another file, which must be there.
///
/// # Errors
///
/// A line of the file that is no step; the door's file not to be read.
pub fn load() -> Result<Vec<(usize, Step)>, String> {
    if let Ok(door) = std::env::var(FILE_ENV) {
        return parse(&mp_os::fs::read_to_string(&door).map_err(|why| format!("{door}: {why}"))?);
    }
    match mp_os::fs::read_to_string(FILE) {
        Ok(text) => parse(&text),
        Err(_) => parse(BUILT_IN),
    }
}

/// `--demo`: these steps are to be played.
pub fn ask(steps: Vec<(usize, Step)>) {
    if let Ok(mut asked) = ASKED.os_lock() {
        *asked = Some(steps);
    }
}

/// The demo asked for, once: the window's to play.
#[must_use]
pub fn take() -> Option<Player> {
    ASKED
        .os_lock()
        .ok()
        .and_then(|mut asked| asked.take())
        .map(Player::new)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A planner that records what the demo does to it.
    #[derive(Default)]
    struct Fake {
        on_screen: Vec<&'static str>,
        screen: &'static str,
        located: bool,
        armed: bool,
        busy: bool,
        did: Vec<String>,
    }

    impl World for Fake {
        fn visible(&self, control: &str) -> bool {
            self.on_screen.contains(&control)
        }
        fn screen(&self) -> &str {
            self.screen
        }
        fn vehicle(&self, until: &Until) -> bool {
            match until {
                Until::Located => self.located,
                Until::Armed => self.armed,
                _ => false,
            }
        }
        fn busy(&self) -> bool {
            self.busy
        }
        fn click(&mut self, control: &str, place: Place, right: bool) -> bool {
            // A map point is there when the map is; a control when it shows.
            let there = self.on_screen.contains(&control);
            if there {
                self.did
                    .push(format!("click {control} {place:?} right={right}"));
            }
            there
        }
        fn key(&mut self, name: &str) {
            self.did.push(format!("key {name}"));
        }
        fn type_text(&mut self, text: &str) {
            self.did.push(format!("type {text}"));
        }
        fn status(&mut self, text: &str) {
            self.did.push(format!("status {text}"));
        }
        fn end(&mut self) {
            self.did.push("end".to_owned());
        }
    }

    fn steps(text: &str) -> Vec<Step> {
        parse(text)
            .expect("a demo")
            .into_iter()
            .map(|(_, step)| step)
            .collect()
    }

    #[test]
    fn every_line_form_reads_and_what_is_no_step_is_named_by_its_line() {
        let click = |control: &str, place, right| Step::Click {
            control: control.to_owned(),
            place,
            right,
        };
        assert_eq!(
            steps(
                "# a comment\n\nscreen plan\nclick plan-zoomicon   # after it\nclick map@0.40x0.60\n\
                 click map@0.50x0.50:right\nclick map@vehicle\nclick map@vehicle+80,-80\n\
                 click map@vehicle-240,+0:right\nclick tab-fly:right\nkey Escape\ntype 100 m\n\
                 wait tab-simulation 120\nwait located 180\nwait armed\nwait climbed 40\nwait auto 15\n\
                 answer fly-prompt-ok 3\npause 2.5\n"
            ),
            [
                Step::Screen("plan".to_owned()),
                click("plan-zoomicon", Place::Centre, false),
                click("map", Place::Part(0.4, 0.6), false),
                click("map", Place::Part(0.5, 0.5), true),
                click("map", Place::Vehicle(0.0, 0.0), false),
                click("map", Place::Vehicle(80.0, -80.0), false),
                click("map", Place::Vehicle(-240.0, 0.0), true),
                click("tab-fly", Place::Centre, true),
                Step::Key("Escape".to_owned()),
                Step::Type("100 m".to_owned()),
                Step::Wait(
                    Until::Shown("tab-simulation".to_owned()),
                    Duration::from_secs(120)
                ),
                Step::Wait(Until::Located, Duration::from_secs(180)),
                Step::Wait(Until::Armed, SHOW_WAIT),
                Step::Wait(Until::Climbed, Duration::from_secs(40)),
                Step::Wait(Until::Auto, Duration::from_secs(15)),
                Step::Answer("fly-prompt-ok".to_owned(), Duration::from_secs(3)),
                Step::Pause(Duration::from_millis(2500)),
            ]
        );
        // The numbers are the file's lines, comments and blanks counted.
        assert_eq!(parse("# one\n\nclick a\n").expect("a demo")[0].0, 3);
        for (bad, line) in [
            ("click a\ndance\n", 2),
            ("click\n", 1),
            ("click a b\n", 1),
            ("click map@2.0x0.5\n", 1),
            ("click map@vehicle+80\n", 1),
            ("click map@vehicle80,80\n", 1),
            ("pause soon\n", 1),
            ("wait\n", 1),
            ("wait located soon\n", 1),
            ("screen\n", 1),
            ("type\n", 1),
        ] {
            let error = parse(bad).expect_err(bad);
            assert!(
                error.starts_with(&format!("line {line}: not a step of the demo")),
                "{bad}: {error}"
            );
        }
    }

    /// A click waits for its control, is one click however long the pointer takes, and rests
    /// before the next; a wait goes on the moment what it waits for has come; the end takes the
    /// pointer away.
    #[test]
    fn the_steps_play_in_order_each_waiting_for_what_it_needs() {
        let mut player = Player::new(
            parse("click tab-simulation\nwait located 60\nkey Escape\nclick map@vehicle+80,-80\n")
                .expect("a demo"),
        );
        let mut world = Fake::default();
        let start = Instant::now();
        let at = |ms: u64| start + Duration::from_millis(ms);
        // The control is not there yet: nothing.
        player.tick(at(0), &mut world);
        assert!(world.did.is_empty());
        assert_eq!(player.progress(), "line 1");
        // It comes: one click, and while the pointer moves nothing more.
        world.on_screen = vec!["tab-simulation", "map"];
        player.tick(at(100), &mut world);
        world.busy = true;
        player.tick(at(200), &mut world);
        player.tick(at(900), &mut world);
        assert_eq!(world.did, ["click tab-simulation Centre right=false"]);
        // The pointer done: a rest, then the wait - which waits.
        world.busy = false;
        player.tick(at(1200), &mut world);
        player.tick(at(1300), &mut world);
        assert_eq!(player.progress(), "line 2");
        player.tick(at(2000), &mut world);
        assert_eq!(world.did.len(), 1);
        // The vehicle located: the key at once, a rest, and the map's click by the vehicle.
        world.located = true;
        player.tick(at(2100), &mut world);
        player.tick(at(2100), &mut world);
        assert_eq!(world.did.last().map(String::as_str), Some("key Escape"));
        player.tick(at(2200), &mut world);
        assert_eq!(world.did.len(), 2, "resting");
        player.tick(at(2800), &mut world);
        assert_eq!(
            world.did.last().map(String::as_str),
            Some("click map Vehicle(80.0, -80.0) right=false")
        );
        // Past the last: the pointer goes, once.
        player.tick(at(2900), &mut world);
        player.tick(at(3400), &mut world);
        assert!(player.running());
        player.tick(at(3700), &mut world);
        assert!(!player.running());
        assert_eq!(player.progress(), "done");
        assert_eq!(world.did.last().map(String::as_str), Some("end"));
        let done = world.did.len();
        player.tick(at(9000), &mut world);
        assert_eq!(world.did.len(), done);
    }

    /// A click whose control does not come is passed over, as an `answer` is and a screen that
    /// does not show, and the demo goes on, saying which lines; a `wait` that does not come stops
    /// it where it is, saying which line and why.
    #[test]
    fn a_click_that_does_not_come_is_passed_over_and_a_wait_stops_the_demo() {
        let start = Instant::now();
        let at = |seconds: u64| start + Duration::from_secs(seconds);

        let mut player = Player::new(
            parse("# the demo\nanswer fly-prompt-ok 3\nclick never\nscreen plan\nclick takeoff\nwait armed 20\nclick Auto\n")
                .expect("a demo"),
        );
        let mut world = Fake {
            on_screen: vec!["takeoff", "Auto"],
            screen: "fly",
            ..Fake::default()
        };
        // The question never comes: three seconds, and on.
        player.tick(at(0), &mut world);
        player.tick(at(2), &mut world);
        assert_eq!(player.progress(), "line 2");
        player.tick(at(4), &mut world);
        // A control that never comes, and a screen that never shows: five seconds each, and on.
        player.tick(at(4), &mut world);
        player.tick(at(8), &mut world);
        assert_eq!(player.progress(), "line 3");
        player.tick(at(10), &mut world);
        player.tick(at(10), &mut world);
        player.tick(at(16), &mut world);
        assert_eq!(player.skipped(), "3 4");
        assert!(world.did.is_empty(), "nothing clicked, nothing said");
        // The click that is there, clicked.
        player.tick(at(16), &mut world);
        assert_eq!(world.did, ["click takeoff Centre right=false"]);
        player.tick(at(17), &mut world);
        // The vehicle does not arm: the demo stops there, and Auto is never clicked.
        player.tick(at(18), &mut world);
        player.tick(at(37), &mut world);
        assert!(player.running(), "twenty seconds are allowed");
        player.tick(at(39), &mut world);
        assert!(!player.running());
        assert_eq!(
            world.did[1..],
            [
                format!("status Demo stopped at line 6 of {FILE}: the vehicle did not arm"),
                "end".to_owned()
            ]
        );
        assert_eq!(
            player.progress(),
            format!("stopped: Demo stopped at line 6 of {FILE}: the vehicle did not arm")
        );
        player.tick(at(90), &mut world);
        assert_eq!(world.did.len(), 3, "Auto is never clicked");

        // Each wait's own words.
        for (text, why) in [
            (
                "wait located 5\n",
                "the vehicle did not connect and find its position",
            ),
            (
                "wait tab-simulation 5\n",
                "tab-simulation did not come on screen",
            ),
            ("wait climbed 5\n", "the vehicle did not climb"),
            ("wait auto 5\n", "the vehicle did not go into Auto"),
        ] {
            let mut player = Player::new(parse(text).expect("a demo"));
            let mut world = Fake::default();
            player.tick(at(0), &mut world);
            player.tick(at(30), &mut world);
            assert_eq!(
                world.did[0],
                format!("status Demo stopped at line 1 of {FILE}: {why}")
            );
            assert_eq!(player.skipped(), "none");
        }
    }

    /// The file the planner ships is the demo: it reads whole, and it opens with what the demo
    /// was when its steps were fixed in the Welcome-Demo-Sitl plugin - the simulator's copter
    /// started, the vehicle waited for, PLAN and Zoom To Vehicle, home set by the map's menu, the
    /// four waypoints around the copter, the survey, Write, FLY, force arm, TakeOff and Auto.
    #[test]
    fn the_shipped_demo_reads_and_begins_with_the_demo_as_it_was() {
        let demo = parse(BUILT_IN).expect("tools/demo/owner.gui");
        let clicks: Vec<String> = demo
            .iter()
            .filter_map(|(_, step)| match step {
                Step::Click {
                    control,
                    place,
                    right,
                } => Some(match place {
                    Place::Centre if !right => control.clone(),
                    Place::Centre => format!("{control}:right"),
                    Place::Vehicle(east, north) if *right => {
                        format!("{control}@{east},{north}:right")
                    }
                    Place::Vehicle(east, north) => format!("{control}@{east},{north}"),
                    Place::Part(..) => format!("{control}@part"),
                }),
                _ => None,
            })
            .collect();
        let was = [
            "tab-simulation",
            "sitl-picture-quad",
            "tab-plan",
            "plan-zoomicon",
            "menu-zoomToVehicle",
            "map@0,0:right",
            "menu-setHomeHere",
            "map@80,80",
            "map@80,-80",
            "map@-80,-80",
            "map@-80,80",
            "map@-240,0:right",
            "menu-polygon",
            "menu-addPolygonPoint2",
            "map@-160,80",
            "map@-160,-80",
            "map@-320,-80",
            "map@-320,80",
            "map@-240,0:right",
            "menu-autoWP",
            "menu-surveyGrid",
            "survey-BUT_Accept",
            "plan-write",
            "tab-fly",
            "fly-tab-actions",
            "force-arm",
            "takeoff",
            "Auto",
        ];
        assert_eq!(clicks.get(..was.len()).expect("as many clicks"), was);
        // And its waits for the vehicle, in their order.
        let waits: Vec<&Until> = demo
            .iter()
            .filter_map(|(_, step)| match step {
                Step::Wait(until, _) => Some(until),
                _ => None,
            })
            .collect();
        assert_eq!(
            waits,
            [
                &Until::Shown("tab-simulation".to_owned()),
                &Until::Located,
                &Until::Armed,
                &Until::Climbed,
                &Until::Auto
            ]
        );
    }
}
