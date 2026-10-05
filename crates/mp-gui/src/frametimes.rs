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

//! What each screen's frames cost in an ordinary run - the owner's request of 2026-10-05 to
//! measure the planner's rendering before speeding it up. storm.rs measures frames under a
//! synthetic telemetry storm; this measures them as they come, idle or connected, on whatever
//! screen shows, and splits each one:
//!
//! * **render**: the top of `render` to the root element built - the planner's own work of
//!   reading state and building the element tree;
//! * **paint**: from there to the paint of [`marker`], the root's last child - gpui's layout,
//!   prepaint and paint, with the planner's own paint code inside them (the HUD, the map);
//! * **present**: from that paint to the end of the update that drew it, when gpui has drawn the
//!   scene and handed it to the platform (storm.rs's packet-to-pixel ends there too);
//! * **gap**: from one frame's start to the next's, so 1/gap is the frame rate.
//!
//! The first frame after arriving on a screen is kept apart (`first_us`): it pays for building
//! the page, and opening a page is one of the moments the owner asked about. The other frames are
//! counted per visit: arriving on a screen starts its count anew, so a script can measure one
//! screen in turn with something changed between visits. The harness's own work in a frame - the probe's file, the
//! facts - is left out, as storm.rs leaves it out.
//!
//! On with `MP_FRAMES=1`; off, nothing is timed and the element tree is untouched. Facts, per
//! screen as `frames.<screen>.<what>` (`fly`, `plan`, ...), in microseconds unless named:
//!
//! | fact | what |
//! |---|---|
//! | `count` | frames counted |
//! | `fps` | frames a second over the time counted, rounded down |
//! | `p50_us`, `p99_us`, `max_us` | the whole frame: render, paint and present |
//! | `render.p50_us`, `render.p99_us` | the render part |
//! | `paint.p50_us`, `paint.p99_us` | the paint part |
//! | `present.p50_us`, `present.p99_us` | the present part |
//! | `gap.p50_us`, `gap.p99_us`, `gap.max_us` | the gap |
//! | `first_us` | the first frame on the screen, whole |
//! | `lap.<name>.p50_us`, `lap.<name>.p99_us` | a stretch of `render` ending at [`lap`]`(name)`, so where the render part goes |
//! | `spent.<name>.p50_us`, `spent.<name>.p99_us` | paint code's own time in the frame, as it reports it with [`spent`]: `hud`, `map` |
//! | `fresh`, `stale` | frames that showed the vehicle's state with a message it had not shown before, and frames that showed only what the frame before had |
//! | `latency.count`, `latency.p50_us`, `latency.p99_us`, `latency.max_us` | packet to pixel: from the newest message in a fresh frame's state arriving at the link to that frame presented |
//! | `wait.p50_us`, `wait.p99_us` | the part of that before the frame began: the link's publish, then the wait for a repaint |
//! | `age.p50_us`, `age.p99_us` | how old the newest message on screen is when each frame is presented, fresh or stale |
//!
//! Packet to pixel (the owner's question of 2026-10-05: is a faster repaint worth it, or is it
//! old data drawn more often?): with `MP_FRAMES` the link stamps each vehicle state with when its
//! newest message arrived (`mp_link::LinkConfig::stamp_arrivals`, `VehicleState::packet_in`), and
//! the frame says which state it drew ([`showing`]) - the vehicle's `messages_applied` tells a
//! state with something new from the one the frame before drew.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::sync::OnceLock;
use web_time::{Duration, Instant};

use gpui::{IntoElement, Styled as _};

use crate::storm::percentile;

/// Frames counted on a screen at most: ten minutes at 60 Hz.
const MAX_FRAMES: usize = 36_000;

/// The facts are refreshed every this many frames on a screen.
const PUBLISH_EVERY: usize = 10;

/// Whether frames are measured: `MP_FRAMES` set, and not to `0`.
#[must_use]
pub fn enabled() -> bool {
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var("MP_FRAMES").is_ok_and(|value| value != "0"))
}

/// One screen's frames.
#[derive(Debug, Default)]
struct Screen {
    /// Each lap of `render`, by name.
    laps: BTreeMap<&'static str, Vec<Duration>>,
    /// Each named piece of paint code's time in a frame.
    spent: BTreeMap<&'static str, Vec<Duration>>,
    whole: Vec<Duration>,
    render: Vec<Duration>,
    paint: Vec<Duration>,
    present: Vec<Duration>,
    gaps: Vec<Duration>,
    /// The first frame on the screen, whole.
    first: Option<Duration>,
    /// Frames that drew something new from the vehicle, and frames that did not.
    fresh: u128,
    stale: u128,
    /// A fresh frame's packet to pixel, and the part of it before the frame began.
    latency: Vec<Duration>,
    wait: Vec<Duration>,
    /// The newest message's age at every frame's present.
    age: Vec<Duration>,
    /// When the first counted frame began, and the last.
    since: Option<Instant>,
    last: Option<Instant>,
}

impl Screen {
    /// The facts, keyed under `frames.<screen>.`.
    fn facts(&self, screen: &str) -> Vec<(String, u128)> {
        let sorted = |samples: &[Duration]| {
            let mut sorted = samples.to_vec();
            sorted.sort_unstable();
            sorted
        };
        let (whole, render, paint, present, gaps) = (
            sorted(&self.whole),
            sorted(&self.render),
            sorted(&self.paint),
            sorted(&self.present),
            sorted(&self.gaps),
        );
        let (latency, wait, age) = (sorted(&self.latency), sorted(&self.wait), sorted(&self.age));
        let span = match (self.since, self.last) {
            (Some(since), Some(last)) => last.saturating_duration_since(since),
            _ => Duration::ZERO,
        };
        // Frames between the first counted one's start and the last's: one fewer than counted.
        let fps = (gaps.len() as u128 * 1_000_000)
            .checked_div(span.as_micros())
            .unwrap_or(0);
        let us = |d: Duration| d.as_micros();
        [
            ("count", whole.len() as u128),
            ("fps", fps),
            ("p50_us", us(percentile(&whole, 50))),
            ("p99_us", us(percentile(&whole, 99))),
            ("max_us", us(whole.last().copied().unwrap_or_default())),
            ("render.p50_us", us(percentile(&render, 50))),
            ("render.p99_us", us(percentile(&render, 99))),
            ("paint.p50_us", us(percentile(&paint, 50))),
            ("paint.p99_us", us(percentile(&paint, 99))),
            ("present.p50_us", us(percentile(&present, 50))),
            ("present.p99_us", us(percentile(&present, 99))),
            ("gap.p50_us", us(percentile(&gaps, 50))),
            ("gap.p99_us", us(percentile(&gaps, 99))),
            ("gap.max_us", us(gaps.last().copied().unwrap_or_default())),
            ("first_us", us(self.first.unwrap_or_default())),
            ("fresh", self.fresh),
            ("stale", self.stale),
            ("latency.count", latency.len() as u128),
            ("latency.p50_us", us(percentile(&latency, 50))),
            ("latency.p99_us", us(percentile(&latency, 99))),
            (
                "latency.max_us",
                us(latency.last().copied().unwrap_or_default()),
            ),
            ("wait.p50_us", us(percentile(&wait, 50))),
            ("wait.p99_us", us(percentile(&wait, 99))),
            ("age.p50_us", us(percentile(&age, 50))),
            ("age.p99_us", us(percentile(&age, 99))),
        ]
        .into_iter()
        .map(|(what, value)| (format!("frames.{screen}.{what}"), value))
        .chain(
            [("lap", &self.laps), ("spent", &self.spent)]
                .into_iter()
                .flat_map(|(kind, named)| named.iter().map(move |entry| (kind, entry)))
                .flat_map(|(kind, (name, samples))| {
                    let sorted = sorted(samples);
                    [
                        (
                            format!("frames.{screen}.{kind}.{name}.p50_us"),
                            us(percentile(&sorted, 50)),
                        ),
                        (
                            format!("frames.{screen}.{kind}.{name}.p99_us"),
                            us(percentile(&sorted, 99)),
                        ),
                    ]
                }),
        )
        .collect()
    }
}

/// The frame being drawn.
#[derive(Debug)]
struct Drawing {
    screen: &'static str,
    started: Instant,
    /// When its root was built, with the harness's time inside the frame by then.
    built: Option<(Instant, Duration)>,
    /// The end of the last lap, with the harness's time by then.
    lap_from: (Instant, Duration),
    laps: Vec<(&'static str, Duration)>,
    /// Paint code's own time, by name, summed over the frame.
    spent: BTreeMap<&'static str, Duration>,
    /// The vehicle state it draws: [`showing`].
    data: Option<Shown>,
}

/// The vehicle state a frame draws: its `messages_applied`, and when its newest message arrived
/// at the link, when the link stamped it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Shown {
    applied: u64,
    packet_in: Option<Instant>,
}

/// A frame painted and waiting to be presented.
#[derive(Debug)]
struct Painted {
    screen: &'static str,
    /// Render and paint, less the harness.
    cost: Duration,
    /// When its marker was painted.
    at: Instant,
    render: Duration,
    paint: Duration,
    /// The first frame after arriving on its screen.
    first: bool,
    /// When it began.
    started: Instant,
    laps: Vec<(&'static str, Duration)>,
    spent: BTreeMap<&'static str, Duration>,
    data: Option<Shown>,
}

/// The frame being drawn and every screen's frames so far.
#[derive(Debug, Default)]
struct Clock {
    /// The frame being drawn.
    frame: Option<Drawing>,
    /// Harness work inside the frame being drawn.
    harness: Duration,
    /// The frame painted and waiting to be presented.
    painted: Option<Painted>,
    /// The screen the frame before was on, and when it began.
    previous: Option<(&'static str, Instant)>,
    /// The `messages_applied` of the last state presented, on whatever screen.
    applied: Option<u64>,
    screens: BTreeMap<&'static str, Screen>,
}

impl Clock {
    /// A frame on `screen` begins.
    fn begin(&mut self, screen: &'static str, now: Instant) {
        self.frame = Some(Drawing {
            screen,
            started: now,
            built: None,
            lap_from: (now, Duration::ZERO),
            laps: Vec::new(),
            spent: BTreeMap::new(),
            data: None,
        });
        self.harness = Duration::ZERO;
    }

    /// A stretch of `render` ends, named `name`: its time less the harness's within it.
    fn lap(&mut self, name: &'static str, now: Instant) {
        let harness = self.harness;
        if let Some(drawing) = self.frame.as_mut() {
            let (from, harness_then) = drawing.lap_from;
            let took = now
                .saturating_duration_since(from)
                .saturating_sub(harness.saturating_sub(harness_then));
            drawing.laps.push((name, took));
            drawing.lap_from = (now, harness);
        }
    }

    /// The root element is built: the last lap, `root`, ends.
    fn rendered(&mut self, now: Instant) {
        self.lap("root", now);
        let harness = self.harness;
        if let Some(drawing) = self.frame.as_mut() {
            drawing.built = Some((now, harness));
        }
    }

    /// The marker is painted: the frame waits only for its present.
    fn painted(&mut self, now: Instant) {
        let Some(Drawing {
            screen,
            started,
            built,
            laps,
            spent,
            data,
            ..
        }) = self.frame.take()
        else {
            return;
        };
        // The harness's time is taken from the part it fell in.
        let (built, harness_then) = built.unwrap_or((now, self.harness));
        let render = built
            .saturating_duration_since(started)
            .saturating_sub(harness_then);
        let paint = now
            .saturating_duration_since(built)
            .saturating_sub(self.harness.saturating_sub(harness_then));
        let cost = render + paint;
        // The gap, and whether this is a screen's first frame.
        let arrived = self.previous.is_none_or(|(before, _)| before != screen);
        self.painted = Some(Painted {
            screen,
            cost,
            at: now,
            render,
            paint,
            first: arrived,
            started,
            laps,
            spent,
            data,
        });
        let gap = self
            .previous
            .filter(|(before, _)| *before == screen)
            .map(|(_, at)| started.saturating_duration_since(at));
        self.previous = Some((screen, started));
        let counted = self.screens.entry(screen).or_default();
        if arrived {
            // A visit's frames: coming back to a screen starts its count anew.
            *counted = Screen::default();
        }
        if let Some(gap) = gap
            && counted.gaps.len() < MAX_FRAMES
        {
            counted.gaps.push(gap);
        }
        counted.since.get_or_insert(started);
        counted.last = Some(started);
    }

    /// The frame painted last has been presented. Returns its screen's facts when they are due.
    fn presented(&mut self, now: Instant) -> Option<Vec<(String, u128)>> {
        let frame = self.painted.take()?;
        let present = now.saturating_duration_since(frame.at);
        // Whether the frame drew something new, before the visit's first frame is set apart, so
        // the next frame is judged against this one.
        let fresh = frame
            .data
            .map(|shown| self.applied.replace(shown.applied) != Some(shown.applied));
        let counted = self.screens.entry(frame.screen).or_default();
        if frame.first {
            // Opening the page: kept apart, and the visit's facts published from nothing, so a
            // reader never takes the last visit's for this one's.
            counted.first = Some(frame.cost + present);
            return Some(counted.facts(frame.screen));
        }
        if counted.whole.len() >= MAX_FRAMES {
            return None;
        }
        if let (Some(fresh), Some(shown)) = (fresh, frame.data) {
            if fresh {
                counted.fresh += 1;
            } else {
                counted.stale += 1;
            }
            if let Some(packet_in) = shown.packet_in {
                let age = now.saturating_duration_since(packet_in);
                counted.age.push(age);
                if fresh {
                    counted.latency.push(age);
                    counted
                        .wait
                        .push(frame.started.saturating_duration_since(packet_in));
                }
            }
        }
        counted.whole.push(frame.cost + present);
        counted.render.push(frame.render);
        counted.paint.push(frame.paint);
        counted.present.push(present);
        for (name, took) in frame.laps {
            counted.laps.entry(name).or_default().push(took);
        }
        for (name, took) in frame.spent {
            counted.spent.entry(name).or_default().push(took);
        }
        counted
            .whole
            .len()
            .is_multiple_of(PUBLISH_EVERY)
            .then(|| counted.facts(frame.screen))
    }
}

thread_local! {
    /// The UI thread's clock: frames are drawn on one thread, so it needs no lock.
    static CLOCK: RefCell<Clock> = RefCell::new(Clock::default());
}

/// A frame on `screen` begins: first thing in `render`.
pub fn begin(screen: &'static str) {
    if enabled() {
        let now = Instant::now();
        CLOCK.with_borrow_mut(|clock| clock.begin(screen, now));
    }
}

/// The vehicle state the frame being drawn shows: its `messages_applied` and `packet_in`, `None`
/// with no vehicle. Called once the frame has its view.
pub fn showing(state: Option<(u64, Option<Instant>)>) {
    if enabled() {
        CLOCK.with_borrow_mut(|clock| {
            if let Some(drawing) = clock.frame.as_mut() {
                drawing.data = state.map(|(applied, packet_in)| Shown { applied, packet_in });
            }
        });
    }
}

/// The root element is built: last thing in `render`.
pub fn rendered() {
    if enabled() {
        let now = Instant::now();
        CLOCK.with_borrow_mut(|clock| clock.rendered(now));
    }
}

/// A stretch of `render` ends here, named `name` (`frames.<screen>.lap.<name>`): where the render
/// part goes, lap by lap. The last lap, from the last call to the root built, is `root`.
pub fn lap(name: &'static str) {
    if enabled() {
        let now = Instant::now();
        CLOCK.with_borrow_mut(|clock| clock.lap(name, now));
    }
}

/// Paint code named `name` took `took` in the frame being drawn (`frames.<screen>.spent.<name>`),
/// summed when it runs more than once in a frame.
pub fn spent(name: &'static str, took: Duration) {
    if enabled() {
        CLOCK.with_borrow_mut(|clock| {
            if let Some(drawing) = clock.frame.as_mut() {
                *drawing.spent.entry(name).or_default() += took;
            }
        });
    }
}

/// Harness work inside the frame being drawn, which is not the frame's cost.
pub fn exclude(spent: Duration) {
    if enabled() {
        CLOCK.with_borrow_mut(|clock| clock.harness += spent);
    }
}

/// The element whose paint ends the paint part, for the root's last child; `None` without
/// `MP_FRAMES`, so a normal run's element tree is untouched. Empty and positioned absolutely,
/// so it takes no part in the layout being measured.
#[must_use]
pub fn marker() -> Option<impl IntoElement> {
    enabled().then(|| {
        gpui::canvas(
            |_bounds, _window, _cx| (),
            |_bounds, (), _window, cx| {
                let now = Instant::now();
                CLOCK.with_borrow_mut(|clock| clock.painted(now));
                // gpui draws and presents the frame in the update that painted it, and runs what
                // was deferred when that update ends.
                cx.defer(|_| {
                    let now = Instant::now();
                    if let Some(facts) = CLOCK.with_borrow_mut(|clock| clock.presented(now)) {
                        for (key, value) in facts {
                            crate::facts::record(key, value);
                        }
                    }
                });
            },
        )
        .absolute()
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    /// A frame drawn on `screen` at `start` ms: built after `render`, painted `paint` later and
    /// presented `present` after that. Returns the facts if they were due.
    fn frame(
        clock: &mut Clock,
        t0: Instant,
        screen: &'static str,
        start: u64,
        (render, paint, present): (u64, u64, u64),
    ) -> Option<Vec<(String, u128)>> {
        let at = t0 + ms(start);
        clock.begin(screen, at);
        clock.rendered(at + ms(render));
        clock.painted(at + ms(render + paint));
        clock.presented(at + ms(render + paint + present))
    }

    fn fact(facts: &[(String, u128)], key: &str) -> u128 {
        facts
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| *v)
            .unwrap_or_else(|| panic!("no fact {key}"))
    }

    /// Each part of a frame is timed apart and the whole is their sum; the gap is start to
    /// start; ten frames a hundred milliseconds apart are 10 fps.
    #[test]
    fn a_frame_is_split_into_render_paint_and_present() {
        let t0 = Instant::now();
        let mut clock = Clock::default();
        let mut facts = None;
        // The first frame is the page's opening; ten more are counted.
        for n in 0..11 {
            facts = frame(&mut clock, t0, "fly", n * 100, (2, 3, 4));
        }
        let facts = facts.expect("due at the tenth counted frame");
        assert_eq!(fact(&facts, "frames.fly.count"), 10);
        assert_eq!(fact(&facts, "frames.fly.render.p50_us"), 2_000);
        assert_eq!(fact(&facts, "frames.fly.paint.p50_us"), 3_000);
        assert_eq!(fact(&facts, "frames.fly.present.p50_us"), 4_000);
        assert_eq!(fact(&facts, "frames.fly.p50_us"), 9_000);
        assert_eq!(fact(&facts, "frames.fly.gap.p50_us"), 100_000);
        assert_eq!(fact(&facts, "frames.fly.fps"), 10);
        assert_eq!(fact(&facts, "frames.fly.first_us"), 9_000);
    }

    /// Screens are counted apart; the first frame after arriving on one is its `first_us`, and
    /// its gap is not counted, since the frame before it was another screen's.
    #[test]
    fn screens_are_counted_apart_and_a_screens_first_frame_is_kept() {
        let t0 = Instant::now();
        let mut clock = Clock::default();
        for n in 0..5 {
            frame(&mut clock, t0, "fly", n * 16, (1, 1, 1));
        }
        frame(&mut clock, t0, "plan", 100, (30, 20, 10));
        let mut facts = None;
        for n in 1..=10 {
            facts = frame(&mut clock, t0, "plan", 100 + n * 16, (1, 1, 1));
        }
        let facts = facts.expect("due at plan's tenth counted frame");
        assert_eq!(fact(&facts, "frames.plan.count"), 10);
        assert_eq!(fact(&facts, "frames.plan.first_us"), 60_000);
        assert_eq!(fact(&facts, "frames.plan.gap.max_us"), 16_000);
        assert_eq!(fact(&facts, "frames.plan.max_us"), 3_000);
        // Fly's first frame was its opening; four counted.
        assert_eq!(clock.screens["fly"].whole.len(), 4);
        // Back on fly: a new visit, counted from nothing.
        frame(&mut clock, t0, "fly", 300, (5, 5, 5));
        frame(&mut clock, t0, "fly", 316, (1, 1, 1));
        assert_eq!(clock.screens["fly"].whole, [ms(3)]);
        assert_eq!(clock.screens["fly"].first, Some(ms(15)));
    }

    /// Laps split the render part: each its stretch, less the harness's work within it, the
    /// last one `root`.
    #[test]
    fn laps_split_the_render_part() {
        let t0 = Instant::now();
        let mut clock = Clock::default();
        frame(&mut clock, t0, "fly", 0, (1, 1, 1));
        let mut facts = None;
        for n in 1..=10 {
            let at = t0 + ms(n * 100);
            clock.begin("fly", at);
            clock.lap("ticks", at + ms(2));
            // The facts written: harness work inside the next lap.
            clock.harness += ms(3);
            clock.lap("body", at + ms(8));
            clock.rendered(at + ms(9));
            // Paint code reporting its own time, twice in the frame.
            for _ in 0..2 {
                if let Some(drawing) = clock.frame.as_mut() {
                    *drawing.spent.entry("hud").or_default() += ms(1);
                }
            }
            clock.painted(at + ms(10));
            facts = clock.presented(at + ms(10));
        }
        let facts = facts.expect("due at the tenth counted frame");
        assert_eq!(fact(&facts, "frames.fly.lap.ticks.p50_us"), 2_000);
        assert_eq!(fact(&facts, "frames.fly.lap.body.p50_us"), 3_000);
        assert_eq!(fact(&facts, "frames.fly.lap.root.p50_us"), 1_000);
        assert_eq!(fact(&facts, "frames.fly.render.p50_us"), 6_000);
        assert_eq!(fact(&facts, "frames.fly.spent.hud.p50_us"), 2_000);
    }

    /// The harness's own work inside a frame is not its cost.
    #[test]
    fn the_harness_is_left_out() {
        let t0 = Instant::now();
        let mut clock = Clock::default();
        frame(&mut clock, t0, "fly", 0, (1, 1, 1));
        clock.begin("fly", t0);
        // Five in the render part, one in the paint part.
        clock.harness += ms(5);
        clock.rendered(t0 + ms(6));
        clock.harness += ms(1);
        clock.painted(t0 + ms(9));
        clock.presented(t0 + ms(9));
        assert_eq!(clock.screens["fly"].whole, [ms(3)]);
        assert_eq!(clock.screens["fly"].render, [ms(1)]);
        assert_eq!(clock.screens["fly"].paint, [ms(2)]);
    }

    /// The owner's question of 2026-10-05, whether a faster repaint shows new data or old data
    /// more often: a frame that draws a state with a message the frame before had not is fresh,
    /// and its packet to pixel runs from that message's arrival to the frame presented; a frame
    /// that draws the same state again is stale, and only the age of what it shows is kept.
    #[test]
    fn packet_to_pixel_is_timed_on_the_frames_that_show_something_new() {
        let t0 = Instant::now();
        let mut clock = Clock::default();
        // Each frame: render 2, paint 3, present 4 - presented 9 ms after it began.
        let mut show = |start: u64, applied: u64, arrived: Option<u64>| {
            let at = t0 + ms(start);
            clock.begin("fly", at);
            if let Some(drawing) = clock.frame.as_mut() {
                drawing.data = Some(Shown {
                    applied,
                    packet_in: arrived.map(|arrived| t0 + ms(arrived)),
                });
            }
            clock.rendered(at + ms(2));
            clock.painted(at + ms(5));
            clock.presented(at + ms(9));
        };
        // The visit's first frame, set apart.
        show(100, 1, Some(95));
        // A message at 190, drawn by the frame at 200: fresh, presented at 209.
        show(200, 2, Some(190));
        // Nothing new by 300: stale, its newest message 119 ms old when presented.
        show(300, 2, Some(190));
        // A message at 395: fresh again.
        show(400, 3, Some(395));
        // A vehicle whose link does not stamp arrivals: fresh, but not timed.
        show(500, 4, None);
        let screen = &clock.screens["fly"];
        assert_eq!((screen.fresh, screen.stale), (3, 1));
        assert_eq!(screen.latency, [ms(19), ms(14)]);
        assert_eq!(screen.wait, [ms(10), ms(5)]);
        assert_eq!(screen.age, [ms(19), ms(119), ms(14)]);
        let facts = screen.facts("fly");
        assert_eq!(fact(&facts, "frames.fly.latency.count"), 2);
        assert_eq!(fact(&facts, "frames.fly.latency.max_us"), 19_000);
        assert_eq!(fact(&facts, "frames.fly.fresh"), 3);
        assert_eq!(fact(&facts, "frames.fly.stale"), 1);
    }
}
