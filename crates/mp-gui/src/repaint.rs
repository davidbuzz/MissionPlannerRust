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

//! When the window is repainted - the owner's choice of 2026-10-05, after the measurements in
//! docs/perf.md: when there is something new to draw, and at least once a second.
//!
//! Mission Planner repaints its flight screen on a 10 Hz timer, `updateBindingSource`
//! (`GCSViews/FlightData.cs:5421-5424`), and so did this planner, everywhere. Measured, a timer
//! draws old data - at a vehicle's 4 Hz, 60% of its frames redrew what the frame before showed -
//! and draws ten frames a second idle; drawing on new data drew no stale frame and brought a
//! packet to its pixels in 16 ms rather than 49 at the median. **Not the C#'s**, by that choice.
//! The window draws (input aside, which draws at once as ever):
//!
//! - when a vehicle's published state has taken a message, looked for every [`POLL`];
//! - when something drawn asked to be drawn again ([`again_in`]): a job in flight every
//!   [`IN_FLIGHT`], as the timer did; an animation at its own pace;
//! - when another thread handed the window something ([`mp_os::wake`]): a plugin's request;
//! - and at least every [`FLOOR`].
//!
//! `MP_REPAINT` chooses another way: `tick:<ms>` a timer (`tick:100`, Mission Planner's 10 Hz,
//! the old way), `data:<ms>` another floor (`MP_FRAMES`, frametimes.rs, measures the difference).

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::OnceLock;
use std::sync::atomic::{AtomicU64, Ordering};

use web_time::{Duration, Instant};

/// How often [`Policy::Data`] looks for new data: under the link's 5 ms publish
/// (`mp_link::DEFAULT_PUBLISH_INTERVAL`), so a new snapshot waits a millisecond for its repaint
/// on average.
pub const POLL: Duration = Duration::from_millis(2);

/// `data`'s floor when none is given.
pub const FLOOR: Duration = Duration::from_secs(1);

/// How often a job in flight is looked at: the timer's 100 ms, Mission Planner's 10 Hz, so a
/// result arrives on screen as soon as it did.
pub const IN_FLIGHT: Duration = Duration::from_millis(100);

/// How often a video showing is drawn: thirty frames a second, as the captures hand them over.
pub const VIDEO_FRAME: Duration = Duration::from_millis(33);

/// The soonest a frame has asked to be drawn again, in milliseconds from [`epoch`]; `u64::MAX`
/// for none.
#[derive(Debug)]
pub struct Deadline(AtomicU64);

impl Deadline {
    /// None asked for.
    #[must_use]
    pub const fn new() -> Self {
        Self(AtomicU64::new(u64::MAX))
    }

    /// Due within `after` of `now`: the soonest of every ask stands.
    pub fn ask(&self, after: Duration, now: Instant) {
        self.0.fetch_min(millis(now + after), Ordering::Relaxed);
    }

    /// Whether it is due at `now`; the ask is then spent.
    pub fn take_due(&self, now: Instant) -> bool {
        let at = self.0.load(Ordering::Relaxed);
        at <= millis(now)
            && self
                .0
                .compare_exchange(at, u64::MAX, Ordering::Relaxed, Ordering::Relaxed)
                .is_ok()
    }
}

/// The window's.
static AGAIN: Deadline = Deadline::new();

/// The clock the deadlines count from.
fn epoch() -> Instant {
    static EPOCH: OnceLock<Instant> = OnceLock::new();
    *EPOCH.get_or_init(Instant::now)
}

fn millis(at: Instant) -> u64 {
    u64::try_from(at.saturating_duration_since(epoch()).as_millis()).unwrap_or(u64::MAX)
}

/// Draw again within `after`: something drawn is still changing - a job in flight, an animation.
/// Asked again each frame while it lasts.
pub fn again_in(after: Duration) {
    AGAIN.ask(after, Instant::now());
}

/// A job in flight, its result looked for each frame: drawn again within [`IN_FLIGHT`].
pub fn in_flight() {
    again_in(IN_FLIGHT);
}

/// Whether a frame asked for by [`again_in`] is due at `now`; the ask is then spent.
pub fn take_due(now: Instant) -> bool {
    AGAIN.take_due(now)
}

/// When to repaint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Policy {
    /// Every interval, whatever has changed.
    Tick(Duration),
    /// When the vehicles' states change, and at least every `floor`.
    Data {
        /// The longest the window goes without a repaint.
        floor: Duration,
    },
}

impl Policy {
    /// `MP_REPAINT`'s words: `tick:<ms>`, `data` or `data:<ms>`; anything else is `None`.
    #[must_use]
    pub fn parse(words: &str) -> Option<Self> {
        let millis = |ms: &str| {
            ms.parse::<u64>()
                .ok()
                .filter(|ms| *ms > 0)
                .map(Duration::from_millis)
        };
        match words.trim().split_once(':') {
            Some(("tick", ms)) => millis(ms).map(Policy::Tick),
            Some(("data", ms)) => millis(ms).map(|floor| Policy::Data { floor }),
            None if words.trim() == "data" => Some(Policy::Data { floor: FLOOR }),
            _ => None,
        }
    }

    /// The environment's choice, `MP_REPAINT`, if it makes one.
    #[must_use]
    pub fn from_env() -> Option<Self> {
        std::env::var("MP_REPAINT")
            .ok()
            .and_then(|words| Self::parse(&words))
    }

    /// The window's policy: `MP_REPAINT`'s; else as fast as it can draw for `MP_BENCH` (`bench`)
    /// and at a display's rate for `MP_STORM` (`storm`), which measure frames; else on new data,
    /// at least every [`FLOOR`].
    #[must_use]
    pub fn chosen(bench: Option<Duration>, storm: Option<Duration>) -> Self {
        Self::from_env()
            .or(bench.map(Policy::Tick))
            .or(storm.map(Policy::Tick))
            .unwrap_or(Policy::Data { floor: FLOOR })
    }

    /// How long the repaint loop sleeps between looks.
    #[must_use]
    pub const fn between_looks(self) -> Duration {
        match self {
            Policy::Tick(interval) => interval,
            Policy::Data { .. } => POLL,
        }
    }
}

/// The repaint loop's memory: what the vehicles' states were at the last repaint, and when.
#[derive(Debug, Clone, Copy)]
pub struct Watch {
    mark: Option<u64>,
    repainted: Instant,
}

impl Watch {
    /// A watch that has seen nothing, its last repaint `now`.
    #[must_use]
    pub const fn new(now: Instant) -> Self {
        Self {
            mark: None,
            repainted: now,
        }
    }

    /// Whether to repaint at `now`: the vehicles' states and the wakes marked `mark`
    /// ([`crate::telemetry::Telemetry::change_mark`], [`mp_os::wakes`]), and `asked` whether a
    /// frame asked for one by now ([`take_due`]).
    pub fn repaint(&mut self, policy: Policy, mark: u64, asked: bool, now: Instant) -> bool {
        let due = match policy {
            Policy::Tick(_) => true,
            Policy::Data { floor } => {
                asked
                    || self.mark != Some(mark)
                    || now.saturating_duration_since(self.repainted) >= floor
            }
        };
        if due {
            self.mark = Some(mark);
            self.repainted = now;
        }
        due
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    #[test]
    fn the_words_name_a_policy() {
        assert_eq!(Policy::parse("tick:33"), Some(Policy::Tick(ms(33))));
        assert_eq!(Policy::parse("data"), Some(Policy::Data { floor: FLOOR }));
        assert_eq!(
            Policy::parse("data:250"),
            Some(Policy::Data { floor: ms(250) })
        );
        for nonsense in ["", "tick", "tick:0", "tick:x", "data:", "data:-1", "fast"] {
            assert_eq!(Policy::parse(nonsense), None, "{nonsense:?}");
        }
        assert_eq!(Policy::Tick(ms(100)).between_looks(), ms(100));
        assert_eq!(Policy::Data { floor: FLOOR }.between_looks(), POLL);
    }

    /// The owner's "on new data, and a minimum of 1 Hz": a repaint when the states change, none
    /// while they do not, and one when a second has passed without.
    #[test]
    fn data_repaints_on_a_change_and_at_the_floor() {
        let t0 = Instant::now();
        let policy = Policy::Data { floor: FLOOR };
        let mut watch = Watch::new(t0);
        assert!(watch.repaint(policy, 7, false, t0), "the first look draws");
        assert!(!watch.repaint(policy, 7, false, t0 + ms(2)));
        assert!(watch.repaint(policy, 8, false, t0 + ms(4)), "a new message");
        assert!(!watch.repaint(policy, 8, false, t0 + ms(1003)));
        assert!(watch.repaint(policy, 8, false, t0 + ms(1004)), "the floor");
        assert!(!watch.repaint(policy, 8, false, t0 + ms(1006)));
    }

    /// The default is the owner's choice: on new data, at least once a second; the environment,
    /// a benchmark or a storm choose otherwise.
    #[test]
    fn the_default_is_on_new_data_with_a_floor() {
        if std::env::var("MP_REPAINT").is_ok() {
            return;
        }
        assert_eq!(Policy::chosen(None, None), Policy::Data { floor: FLOOR });
        assert_eq!(Policy::chosen(Some(ms(1)), None), Policy::Tick(ms(1)));
        assert_eq!(Policy::chosen(None, Some(ms(16))), Policy::Tick(ms(16)));
    }

    /// Something drawn that is still changing asks for the next frame: nothing new from the
    /// vehicles, and drawn all the same; asked, then spent.
    #[test]
    fn a_frame_asked_for_is_drawn() {
        let t0 = Instant::now();
        let policy = Policy::Data { floor: FLOOR };
        let mut watch = Watch::new(t0);
        assert!(watch.repaint(policy, 1, false, t0));
        assert!(!watch.repaint(policy, 1, false, t0 + ms(50)));
        assert!(watch.repaint(policy, 1, true, t0 + ms(100)), "asked");
        assert!(!watch.repaint(policy, 1, false, t0 + ms(102)));
    }

    /// `again_in`'s deadline: not due before it, due once at it, the soonest of several asks.
    #[test]
    fn again_in_is_due_at_its_deadline_once() {
        // A deadline of its own: the window's is asked by every test that draws.
        let deadline = Deadline::new();
        let now = Instant::now();
        deadline.ask(ms(10_000), now);
        deadline.ask(ms(50), now);
        assert!(!deadline.take_due(now));
        assert!(deadline.take_due(now + ms(60)), "the soonest ask");
        assert!(!deadline.take_due(now + ms(20_000)), "spent");
    }

    #[test]
    fn a_tick_repaints_at_every_wake() {
        let t0 = Instant::now();
        let mut watch = Watch::new(t0);
        let policy = Policy::Tick(ms(100));
        assert!(watch.repaint(policy, 1, false, t0));
        assert!(watch.repaint(policy, 1, false, t0 + ms(100)));
    }
}
