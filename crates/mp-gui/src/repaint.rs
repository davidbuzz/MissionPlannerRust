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

//! When the window is repainted - the owner's question of 2026-10-05: is a faster repaint worth
//! having, or would it draw old data more often; and should the screen repaint on new data, with
//! a floor of once a second.
//!
//! The planner repaints on a timer, every [`crate::REFRESH`] (100 ms): Mission Planner's own 10 Hz,
//! the flight screen's `updateBindingSource` timer (`GCSViews/FlightData.cs:5421-5424`). Input
//! repaints at once besides. `MP_REPAINT` chooses another way, to be measured against it
//! (`MP_FRAMES`, frametimes.rs, times each frame's packet to pixel):
//!
//! | `MP_REPAINT` | repaint |
//! |---|---|
//! | `tick:<ms>` | every `<ms>` milliseconds, as the default does every 100 |
//! | `data` | when a vehicle's state has a message the last repaint had not, looked for every [`POLL`], and at least once a second |
//! | `data:<ms>` | the same with a floor of `<ms>` |
//!
//! **Not the C#'s**, and not the product's until the owner chooses: measurement scaffolding.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use web_time::{Duration, Instant};

/// How often [`Policy::Data`] looks for new data: under the link's 5 ms publish
/// (`mp_link::DEFAULT_PUBLISH_INTERVAL`), so a new snapshot waits a millisecond for its repaint
/// on average.
pub const POLL: Duration = Duration::from_millis(2);

/// `data`'s floor when none is given.
pub const FLOOR: Duration = Duration::from_secs(1);

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

    /// How long the repaint loop sleeps between looks.
    #[must_use]
    pub const fn wake(self) -> Duration {
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

    /// Whether to repaint at `now`, the vehicles' states marked `mark`
    /// ([`crate::telemetry::Telemetry::change_mark`]).
    pub fn repaint(&mut self, policy: Policy, mark: u64, now: Instant) -> bool {
        let due = match policy {
            Policy::Tick(_) => true,
            Policy::Data { floor } => {
                self.mark != Some(mark) || now.saturating_duration_since(self.repainted) >= floor
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
        assert_eq!(Policy::Tick(ms(100)).wake(), ms(100));
        assert_eq!(Policy::Data { floor: FLOOR }.wake(), POLL);
    }

    /// The owner's "on new data, and a minimum of 1 Hz": a repaint when the states change, none
    /// while they do not, and one when a second has passed without.
    #[test]
    fn data_repaints_on_a_change_and_at_the_floor() {
        let t0 = Instant::now();
        let policy = Policy::Data { floor: FLOOR };
        let mut watch = Watch::new(t0);
        assert!(watch.repaint(policy, 7, t0), "the first look draws");
        assert!(!watch.repaint(policy, 7, t0 + ms(2)));
        assert!(watch.repaint(policy, 8, t0 + ms(4)), "a new message");
        assert!(!watch.repaint(policy, 8, t0 + ms(1003)));
        assert!(watch.repaint(policy, 8, t0 + ms(1004)), "the floor");
        assert!(!watch.repaint(policy, 8, t0 + ms(1006)));
    }

    #[test]
    fn a_tick_repaints_at_every_wake() {
        let t0 = Instant::now();
        let mut watch = Watch::new(t0);
        let policy = Policy::Tick(ms(100));
        assert!(watch.repaint(policy, 1, t0));
        assert!(watch.repaint(policy, 1, t0 + ms(100)));
    }
}
