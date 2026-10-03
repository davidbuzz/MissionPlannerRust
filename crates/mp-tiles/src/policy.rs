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

//! When to ask for a tile, and when to stop asking.
//!
//! Separated from the fetching itself so it can be tested without a network or a clock. The rules
//! here are the difference between a map that is a good citizen and one that gets an IP address
//! blocked: tile providers serve this for free, and a client that retries a failing tile every
//! frame is indistinguishable from an attack.

use std::collections::HashMap;
use std::time::{Duration, Instant};

use mp_units::TileId;

/// How many requests may be outstanding at once.
///
/// OpenStreetMap's usage policy asks for no more than two connections. A screenful at 1600x1200 is
/// about forty tiles, so they arrive over a few seconds rather than at once - which is fine,
/// because a missing tile is drawn from its parent in the meantime.
pub const MAX_IN_FLIGHT: usize = 2;

/// How long to wait before retrying a tile that failed.
///
/// Doubles per consecutive failure. A provider having a bad minute should not be asked again
/// immediately, and a provider that is down should be asked rarely.
pub const INITIAL_BACKOFF: Duration = Duration::from_secs(2);

/// The longest we will ever wait before trying a failed tile again.
pub const MAX_BACKOFF: Duration = Duration::from_secs(300);

/// How many failures before a tile is considered hopeless for this session.
///
/// Not permanent: a pan back to the area retries it after the backoff. Permanent would mean a tile
/// that failed once during a dropout stays blank until the application restarts.
pub const FAILURES_BEFORE_GIVING_UP: u32 = 5;

/// What should happen to a request for a tile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decision {
    /// Ask the provider for it.
    Fetch,
    /// Already being fetched; do nothing.
    InFlight,
    /// Failed recently; wait before asking again.
    BackingOff,
    /// Too many requests already outstanding.
    Saturated,
    /// Fetching is switched off.
    Offline,
}

/// Tracks what has been asked for and what has failed.
#[derive(Debug)]
pub struct FetchPolicy {
    in_flight: HashMap<TileId, Instant>,
    failures: HashMap<TileId, Failure>,
    offline: bool,
    max_in_flight: usize,
}

#[derive(Debug, Clone, Copy)]
struct Failure {
    count: u32,
    retry_after: Instant,
}

impl Default for FetchPolicy {
    fn default() -> Self {
        Self::new()
    }
}

impl FetchPolicy {
    /// A policy that will fetch.
    #[must_use]
    pub fn new() -> Self {
        Self {
            in_flight: HashMap::new(),
            failures: HashMap::new(),
            offline: false,
            max_in_flight: MAX_IN_FLIGHT,
        }
    }

    /// A policy that never fetches, for offline operation and for tests.
    #[must_use]
    pub fn offline() -> Self {
        Self {
            offline: true,
            ..Self::new()
        }
    }

    /// Switches fetching on or off.
    pub fn set_offline(&mut self, offline: bool) {
        self.offline = offline;
    }

    /// Whether fetching is switched off.
    #[must_use]
    pub const fn is_offline(&self) -> bool {
        self.offline
    }

    /// How many requests are outstanding.
    #[must_use]
    pub fn in_flight(&self) -> usize {
        self.in_flight.len()
    }

    /// Decides what to do about a tile, at a given moment.
    ///
    /// Takes the time rather than reading the clock, so the backoff can be tested without sleeping
    /// and without a fake clock abstraction leaking into the caller.
    #[must_use]
    pub fn decide(&self, tile: TileId, now: Instant) -> Decision {
        if self.offline {
            return Decision::Offline;
        }
        if self.in_flight.contains_key(&tile) {
            return Decision::InFlight;
        }
        if let Some(failure) = self.failures.get(&tile)
            && now < failure.retry_after
        {
            return Decision::BackingOff;
        }
        if self.in_flight.len() >= self.max_in_flight {
            return Decision::Saturated;
        }
        Decision::Fetch
    }

    /// Records that a tile is being fetched. Returns false if it should not have been.
    pub fn begin(&mut self, tile: TileId, now: Instant) -> bool {
        if self.decide(tile, now) != Decision::Fetch {
            return false;
        }
        self.in_flight.insert(tile, now);
        true
    }

    /// Records a successful fetch, clearing any history of failure.
    pub fn succeeded(&mut self, tile: TileId) {
        self.in_flight.remove(&tile);
        self.failures.remove(&tile);
    }

    /// Records a failure and schedules the next attempt.
    pub fn failed(&mut self, tile: TileId, now: Instant) {
        self.in_flight.remove(&tile);
        let entry = self.failures.entry(tile).or_insert(Failure {
            count: 0,
            retry_after: now,
        });
        entry.count = entry.count.saturating_add(1);
        entry.retry_after = now + backoff_for(entry.count);
    }

    /// How many times a tile has failed.
    #[must_use]
    pub fn failure_count(&self, tile: TileId) -> u32 {
        self.failures.get(&tile).map_or(0, |failure| failure.count)
    }

    /// Whether a tile has failed so often that it is not worth drawing a loading state for.
    #[must_use]
    pub fn has_given_up(&self, tile: TileId) -> bool {
        self.failure_count(tile) >= FAILURES_BEFORE_GIVING_UP
    }

    /// Abandons requests that have been outstanding too long.
    ///
    /// A fetch that never completes would otherwise hold one of the two slots forever, and two of
    /// them would stop the map loading anything at all for the rest of the session.
    pub fn expire(&mut self, now: Instant, timeout: Duration) -> Vec<TileId> {
        let stale: Vec<TileId> = self
            .in_flight
            .iter()
            .filter(|(_, started)| now.duration_since(**started) > timeout)
            .map(|(tile, _)| *tile)
            .collect();
        for tile in &stale {
            self.failed(*tile, now);
        }
        stale
    }

    /// Forgets every failure, so a map that went offline and came back tries again at once.
    pub fn clear_failures(&mut self) {
        self.failures.clear();
    }
}

/// The delay before the nth consecutive retry.
fn backoff_for(failures: u32) -> Duration {
    let exponent = failures.saturating_sub(1).min(16);
    INITIAL_BACKOFF
        .saturating_mul(1_u32 << exponent)
        .min(MAX_BACKOFF)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tile(x: i64) -> TileId {
        TileId::new(10, x, 5).expect("a valid tile")
    }

    #[test]
    fn a_fresh_tile_is_fetched() {
        let policy = FetchPolicy::new();
        assert_eq!(policy.decide(tile(0), Instant::now()), Decision::Fetch);
    }

    #[test]
    fn a_tile_already_being_fetched_is_not_fetched_twice() {
        // Forty tiles come into view at once and the render pass asks for each of them every
        // frame; without this the same tile is requested sixty times a second.
        let mut policy = FetchPolicy::new();
        let now = Instant::now();
        assert!(policy.begin(tile(0), now));
        assert_eq!(policy.decide(tile(0), now), Decision::InFlight);
        assert!(!policy.begin(tile(0), now));
    }

    #[test]
    fn no_more_than_the_allowed_requests_are_outstanding() {
        // The providers serve this for free and ask for a small number of connections.
        let mut policy = FetchPolicy::new();
        let now = Instant::now();
        for x in 0..MAX_IN_FLIGHT {
            assert!(policy.begin(tile(x as i64), now), "tile {x} should start");
        }
        assert_eq!(
            policy.decide(tile(99), now),
            Decision::Saturated,
            "a further request should wait"
        );
        assert_eq!(policy.in_flight(), MAX_IN_FLIGHT);
    }

    #[test]
    fn finishing_a_fetch_frees_a_slot() {
        let mut policy = FetchPolicy::new();
        let now = Instant::now();
        for x in 0..MAX_IN_FLIGHT {
            policy.begin(tile(x as i64), now);
        }
        policy.succeeded(tile(0));
        assert_eq!(policy.decide(tile(99), now), Decision::Fetch);
    }

    #[test]
    fn a_failed_tile_is_not_retried_immediately() {
        // A client that retries a failing tile every frame is indistinguishable from an attack.
        let mut policy = FetchPolicy::new();
        let now = Instant::now();
        policy.begin(tile(0), now);
        policy.failed(tile(0), now);
        assert_eq!(policy.decide(tile(0), now), Decision::BackingOff);
        assert_eq!(
            policy.decide(tile(0), now + INITIAL_BACKOFF + Duration::from_millis(1)),
            Decision::Fetch
        );
    }

    #[test]
    fn the_backoff_grows_with_each_failure_and_then_stops_growing() {
        let mut policy = FetchPolicy::new();
        let mut now = Instant::now();
        let mut previous = Duration::ZERO;
        // Far enough to reach the cap: 2s doubling passes 300s at the ninth failure.
        for attempt in 1..=12 {
            policy.begin(tile(0), now);
            policy.failed(tile(0), now);
            let wait = backoff_for(policy.failure_count(tile(0)));
            assert!(
                wait >= previous,
                "attempt {attempt}: backoff shrank from {previous:?} to {wait:?}"
            );
            assert!(
                wait <= MAX_BACKOFF,
                "attempt {attempt}: {wait:?} is unbounded"
            );
            previous = wait;
            now += wait + Duration::from_millis(1);
        }
        assert_eq!(previous, MAX_BACKOFF);
    }

    #[test]
    fn a_success_forgets_the_failures_that_came_before_it() {
        // Otherwise a tile that failed during a dropout keeps a long backoff after the network
        // comes back, and the map has a permanent hole in it.
        let mut policy = FetchPolicy::new();
        let now = Instant::now();
        policy.begin(tile(0), now);
        policy.failed(tile(0), now);
        policy.failed(tile(0), now);
        assert_eq!(policy.failure_count(tile(0)), 2);

        let later = now + MAX_BACKOFF;
        policy.begin(tile(0), later);
        policy.succeeded(tile(0));
        assert_eq!(policy.failure_count(tile(0)), 0);
        assert_eq!(policy.decide(tile(0), later), Decision::Fetch);
    }

    #[test]
    fn giving_up_is_for_this_session_not_forever() {
        let mut policy = FetchPolicy::new();
        let mut now = Instant::now();
        for _ in 0..FAILURES_BEFORE_GIVING_UP {
            policy.begin(tile(0), now);
            policy.failed(tile(0), now);
            now += MAX_BACKOFF + Duration::from_millis(1);
        }
        assert!(policy.has_given_up(tile(0)));
        // Still eligible once the backoff expires: a pan back to the area tries again.
        assert_eq!(policy.decide(tile(0), now), Decision::Fetch);
    }

    #[test]
    fn offline_means_offline() {
        let policy = FetchPolicy::offline();
        assert!(policy.is_offline());
        assert_eq!(policy.decide(tile(0), Instant::now()), Decision::Offline);
    }

    #[test]
    fn going_offline_stops_fetching_even_for_a_tile_that_would_have_been_fetched() {
        let mut policy = FetchPolicy::new();
        let now = Instant::now();
        assert_eq!(policy.decide(tile(0), now), Decision::Fetch);
        policy.set_offline(true);
        assert_eq!(policy.decide(tile(0), now), Decision::Offline);
        assert!(!policy.begin(tile(0), now));
    }

    #[test]
    fn a_fetch_that_never_finishes_does_not_hold_its_slot_forever() {
        // Two wedged requests would otherwise stop the map loading anything for the rest of the
        // session, which looks exactly like the map being broken.
        let mut policy = FetchPolicy::new();
        let now = Instant::now();
        for x in 0..MAX_IN_FLIGHT {
            policy.begin(tile(x as i64), now);
        }
        assert_eq!(policy.decide(tile(99), now), Decision::Saturated);

        let timeout = Duration::from_secs(30);
        let expired = policy.expire(now + timeout + Duration::from_secs(1), timeout);
        assert_eq!(expired.len(), MAX_IN_FLIGHT);
        assert_eq!(policy.in_flight(), 0);
    }

    #[test]
    fn expiring_counts_as_a_failure_so_a_wedged_provider_is_backed_off_too() {
        let mut policy = FetchPolicy::new();
        let now = Instant::now();
        policy.begin(tile(0), now);
        let later = now + Duration::from_secs(60);
        policy.expire(later, Duration::from_secs(30));
        assert_eq!(policy.failure_count(tile(0)), 1);
        assert_eq!(policy.decide(tile(0), later), Decision::BackingOff);
    }

    #[test]
    fn coming_back_online_can_clear_the_backlog_of_failures() {
        let mut policy = FetchPolicy::new();
        let now = Instant::now();
        for x in 0..3 {
            policy.begin(tile(x), now);
            policy.failed(tile(x), now);
        }
        policy.clear_failures();
        for x in 0..3 {
            assert_eq!(policy.failure_count(tile(x)), 0);
        }
    }
}
