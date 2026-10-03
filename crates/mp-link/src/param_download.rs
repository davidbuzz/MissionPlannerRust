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

//! The parameter download: `PARAM_REQUEST_LIST`, then whatever it takes to fill the holes.
//!
//! Replaces the loop in `getParamListAsync`
//! (C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1948-2263).
//!
//! # How the C# recovers a lossy stream, and so how this does
//!
//! The vehicle streams every parameter once and never retransmits, so a download is one request
//! and then a recovery strategy. The C#'s, step by step:
//!
//! 1. Send `PARAM_REQUEST_LIST` and take whatever arrives, in any order, keyed by `param_index`.
//! 2. Start recovering when either the stream has been quiet for four seconds, or the last index
//!    arrived with holes behind it - no point waiting four seconds for a stream that has ended
//!    (:2089-2090, :2114).
//! 3. If less than three quarters arrived, ask for the whole list again, at most twice (:2117).
//!    A vehicle that has not said how many it has yet does not use up those two (:2119-2124).
//! 4. Otherwise ask one by one: up to ten `PARAM_REQUEST_READ` by index per round, a round a
//!    second, each round starting where the last left off so a hole the vehicle never answers
//!    cannot starve the ones after it (:2135-2205).
//! 5. Finish when every index up to the reported count has arrived (:2226).
//!
//! There is no step 6. The C# never gives up on its own: a hole the vehicle never fills is asked
//! for once a second until the operator cancels the progress dialog (:2104-2112). This machine
//! does the same, and says so in its state - [`ParamDownloadState::Recovering`] is the
//! "incomplete" outcome, visible the moment recovery starts, and [`ParamDownload::cancel`] is the
//! dialog's Cancel. A download never blocks anything, so "never gives up" costs one request a
//! second rather than a hung screen.
//!
//! Two places differ from the C#, both on purpose:
//!
//! * **The count.** The C# believes the latest `param_count` (:2025); this believes the largest,
//!   as [`mp_params::ParamTable`] does and for the reason given there: ArduPilot revises the count
//!   upward mid-download, and a download that completes against the smaller figure is thirteen
//!   parameters short with nothing to say so.
//! * **The skip.** The C# skips a round when more than ten new parameters arrived since the last
//!   one (:2146-2147). It `continue`s to the top of a `do`/`while` having already zeroed the
//!   counter and without stamping the round's time, so the next pass does the round anyway: the
//!   guard delays nothing. It is not ported, because it does nothing.

use std::collections::BTreeSet;
use std::time::Instant;

use mp_vehicle::VehicleId;

use crate::timeouts::ProtocolTimeouts;

/// How many `PARAM_REQUEST_READ` go out per recovery round.
///
/// C#: MAVLinkInterface.cs:2187 (`if (queued >= 10) break;`).
pub const PARAM_RETRY_BURST: usize = 10;

/// `param_index` of a reply to a read by name: "not telling you where this sits in the list".
///
/// C#: MAVLinkInterface.cs:2077-2078 (`if (par.param_index != 65535)`).
const INDEX_NOT_IN_LIST: u16 = u16::MAX;

/// Where a download is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamDownloadState {
    /// `PARAM_REQUEST_LIST` sent and the vehicle streaming, or asked for the whole list again.
    Streaming {
        /// Whole-list requests after the first, of the C#'s two.
        full_list_retries: u8,
    },
    /// The stream ended with holes, and they are being asked for one by one.
    ///
    /// This is the "incomplete" outcome: the table is short and the caller can see by how much.
    /// It lasts until the holes fill or the caller cancels, exactly as the C#'s loop does.
    Recovering {
        /// How many rounds of up to ten reads have gone out.
        rounds: u32,
    },
    /// Every index up to the count arrived.
    Complete,
    /// The caller stopped it, with holes outstanding.
    Cancelled,
}

/// The indices one recovery round asks for, without allocating: at most [`PARAM_RETRY_BURST`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Burst {
    indices: [u16; PARAM_RETRY_BURST],
    len: usize,
}

impl Burst {
    /// The indices, in the order they go out.
    #[must_use]
    pub fn as_slice(&self) -> &[u16] {
        self.indices.get(..self.len).unwrap_or(&[])
    }

    fn push(&mut self, index: u16) -> bool {
        match self.indices.get_mut(self.len) {
            Some(slot) => {
                *slot = index;
                self.len += 1;
                self.len < PARAM_RETRY_BURST
            }
            None => false,
        }
    }
}

/// What the download wants sent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamAction {
    /// Nothing, yet.
    Nothing,
    /// `PARAM_REQUEST_LIST`.
    RequestList,
    /// One `PARAM_REQUEST_READ` by index per entry.
    RequestIndices(Burst),
}

/// One vehicle's parameter download.
#[derive(Debug, Clone)]
pub struct ParamDownload {
    /// Which vehicle.
    pub target: VehicleId,
    state: ParamDownloadState,
    /// `indexsreceived`: what arrived in *this* download. A second download starts empty, as the
    /// C#'s does, rather than finishing at once against the table the first one filled.
    received: BTreeSet<u16>,
    /// The largest `param_count` reported, or `None` before the first.
    reported: Option<u16>,
    /// `start`: the last time a new parameter arrived, while still streaming.
    last_valid: Instant,
    /// `missing_params`: the last index arrived with holes behind it.
    last_index_seen_short: bool,
    /// `retry`: whole-list requests after the first.
    full_list_retries: u8,
    /// `lastonebyone`.
    last_round: Option<Instant>,
    /// `tenbytenindex`: where the next round starts looking.
    next_index: u16,
    rounds: u32,
    timeouts: ProtocolTimeouts,
}

impl ParamDownload {
    /// Starts a download. The caller sends the first `PARAM_REQUEST_LIST`; see [`Self::begin`].
    #[must_use]
    pub fn new(target: VehicleId, timeouts: ProtocolTimeouts, now: Instant) -> Self {
        Self {
            target,
            state: ParamDownloadState::Streaming {
                full_list_retries: 0,
            },
            received: BTreeSet::new(),
            reported: None,
            last_valid: now,
            last_index_seen_short: false,
            full_list_retries: 0,
            last_round: None,
            next_index: 0,
            rounds: 0,
            timeouts,
        }
    }

    /// The first thing to send. C#: MAVLinkInterface.cs:2097.
    #[must_use]
    pub const fn begin(&self) -> ParamAction {
        ParamAction::RequestList
    }

    /// Where the download is.
    #[must_use]
    pub const fn state(&self) -> ParamDownloadState {
        self.state
    }

    /// Whether it has stopped, complete or not.
    #[must_use]
    pub const fn is_finished(&self) -> bool {
        matches!(
            self.state,
            ParamDownloadState::Complete | ParamDownloadState::Cancelled
        )
    }

    /// How many distinct indices arrived in this download.
    #[must_use]
    pub fn received(&self) -> usize {
        self.received.len()
    }

    /// How many the vehicle says it has, once it has said.
    #[must_use]
    pub const fn expected(&self) -> Option<u16> {
        self.reported
    }

    /// Indices not yet received, lowest first.
    #[must_use]
    pub fn missing(&self) -> Vec<u16> {
        (0..self.total())
            .filter(|index| !self.received.contains(index))
            .collect()
    }

    /// The operator's Cancel. C#: MAVLinkInterface.cs:2104-2112, "User Canceled".
    pub fn cancel(&mut self) {
        if !self.is_finished() {
            self.state = ParamDownloadState::Cancelled;
        }
    }

    /// `param_total`: one until the vehicle says otherwise, so the loop runs at all (:1956).
    fn total(&self) -> u16 {
        self.reported.unwrap_or(1)
    }

    /// A `PARAM_VALUE` from this vehicle arrived. C#: MAVLinkInterface.cs:2013-2093.
    pub fn on_param_value(&mut self, index: u16, count: u16, now: Instant) {
        if self.is_finished() {
            return;
        }
        // One by one, the stream is over and the round timer rules; a trickle of answers must
        // not hold recovery off (:2018-2020).
        if matches!(self.state, ParamDownloadState::Streaming { .. }) {
            self.last_valid = now;
        }
        self.reported = Some(self.reported.map_or(count, |seen| seen.max(count)));
        if count == 0 {
            // A vehicle with no parameters. The C# returns early here and then leaves its loop,
            // because nothing received is not less than nothing reported (:2028-2029, :2226).
            self.finish_if_complete();
            return;
        }
        if index != INDEX_NOT_IN_LIST && !self.received.insert(index) {
            // Already have it (:2039-2047).
            return;
        }
        let total = self.total();
        if index == total.saturating_sub(1) {
            // The last one arrived: if anything is missing, the stream is over and waiting four
            // seconds to find that out is waste (:2088-2090).
            self.last_index_seen_short |= self.received.len() < usize::from(total);
        }
        self.finish_if_complete();
    }

    fn finish_if_complete(&mut self) {
        if self.received.len() >= usize::from(self.total()) && self.reported.is_some() {
            self.state = ParamDownloadState::Complete;
        }
    }

    /// Called every pass of the link loop. C#: MAVLinkInterface.cs:2114-2208.
    pub fn on_tick(&mut self, now: Instant) -> ParamAction {
        if self.is_finished() {
            return ParamAction::Nothing;
        }
        let quiet =
            now.saturating_duration_since(self.last_valid) >= self.timeouts.param_list_quiet;
        if !(self.last_index_seen_short || quiet) {
            return ParamAction::Nothing;
        }

        let total = self.total();
        let recovering = matches!(self.state, ParamDownloadState::Recovering { .. });
        // `(param_total / 4) * 3` in integer arithmetic, as the C# has it (:2117).
        let three_quarters = usize::from(total / 4) * 3;
        if !recovering
            && self.full_list_retries < self.timeouts.param_list_full_retries
            && (total <= 1 || self.received.len() < three_quarters)
        {
            // A vehicle that has not said how many it has does not use up a retry (:2119-2124).
            if total > 1 {
                self.full_list_retries += 1;
            }
            self.state = ParamDownloadState::Streaming {
                full_list_retries: self.full_list_retries,
            };
            self.last_valid = now;
            self.last_index_seen_short = false;
            return ParamAction::RequestList;
        }

        if self.last_round.is_some_and(|last| {
            now.saturating_duration_since(last) < self.timeouts.param_list_round
        }) {
            self.state = ParamDownloadState::Recovering {
                rounds: self.rounds,
            };
            return ParamAction::Nothing;
        }

        // `startindex = tenbytenindex % (param_total - 1)` (:2140-2141).
        let start = if total > 1 {
            self.next_index % (total - 1)
        } else {
            0
        };
        let mut burst = Burst::default();
        for index in start..total {
            if self.received.contains(&index) {
                continue;
            }
            self.next_index = index.saturating_add(1);
            if !burst.push(index) {
                break;
            }
        }
        if burst.len == 0 {
            // Nothing missing after where this round started: the next round starts from the
            // top (:2201-2205). This round is spent regardless, as the C#'s is.
            self.next_index = 0;
        }
        self.last_round = Some(now);
        self.rounds += 1;
        self.state = ParamDownloadState::Recovering {
            rounds: self.rounds,
        };
        if burst.len == 0 {
            ParamAction::Nothing
        } else {
            ParamAction::RequestIndices(burst)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn download(now: Instant) -> ParamDownload {
        ParamDownload::new(VehicleId::new(1, 1), ProtocolTimeouts::default(), now)
    }

    fn indices(action: ParamAction) -> Vec<u16> {
        match action {
            ParamAction::RequestIndices(burst) => burst.as_slice().to_vec(),
            other => panic!("expected a burst, got {other:?}"),
        }
    }

    #[test]
    fn indices_arriving_in_any_order_complete_the_download() {
        let t0 = Instant::now();
        let mut d = download(t0);
        for index in [3, 0, 4, 1, 2] {
            d.on_param_value(index, 5, t0);
        }
        assert_eq!(d.state(), ParamDownloadState::Complete);
        assert_eq!(
            d.on_tick(t0 + Duration::from_secs(60)),
            ParamAction::Nothing
        );
    }

    #[test]
    fn the_last_index_arriving_short_starts_recovery_without_waiting_for_quiet() {
        let t0 = Instant::now();
        let mut d = download(t0);
        for index in (0..40).filter(|i| *i != 7) {
            d.on_param_value(index, 40, t0);
        }
        // 39 of 40 is past three quarters, so straight to one by one, at once.
        assert_eq!(indices(d.on_tick(t0)), vec![7]);
        assert_eq!(d.state(), ParamDownloadState::Recovering { rounds: 1 });
    }

    #[test]
    fn a_quiet_stream_under_three_quarters_asks_for_the_whole_list_twice_then_one_by_one() {
        let t0 = Instant::now();
        let quiet = ProtocolTimeouts::default().param_list_quiet;
        let mut d = download(t0);
        for index in 0..10 {
            d.on_param_value(index, 100, t0);
        }
        assert_eq!(
            d.on_tick(t0 + quiet - Duration::from_millis(1)),
            ParamAction::Nothing
        );
        assert_eq!(d.on_tick(t0 + quiet), ParamAction::RequestList);
        assert_eq!(d.on_tick(t0 + quiet * 2), ParamAction::RequestList);
        // Two whole-list retries spent: the third quiet spell goes one by one, ten at a time.
        assert_eq!(
            indices(d.on_tick(t0 + quiet * 3)),
            (10..20).collect::<Vec<_>>()
        );
    }

    #[test]
    fn rounds_rotate_so_an_unanswered_hole_cannot_starve_the_ones_after_it() {
        let t0 = Instant::now();
        let round = ProtocolTimeouts::default().param_list_round;
        let mut d = download(t0);
        // 100 parameters, 25 missing: 0..10 never answered, 10..25 answered on request.
        for index in 25..100 {
            d.on_param_value(index, 100, t0);
        }
        assert_eq!(indices(d.on_tick(t0)), (0..10).collect::<Vec<_>>());
        assert_eq!(
            d.on_tick(t0 + round / 2),
            ParamAction::Nothing,
            "one round a second"
        );
        assert_eq!(indices(d.on_tick(t0 + round)), (10..20).collect::<Vec<_>>());
        for index in 10..25 {
            d.on_param_value(index, 100, t0 + round);
        }
        // Nothing missing after index 20: the round is spent and the next starts from the top.
        assert_eq!(d.on_tick(t0 + round * 2), ParamAction::Nothing);
        assert_eq!(
            indices(d.on_tick(t0 + round * 3)),
            (0..10).collect::<Vec<_>>()
        );
        assert_eq!(d.state(), ParamDownloadState::Recovering { rounds: 4 });
    }

    #[test]
    fn a_vehicle_that_never_answers_is_asked_for_the_list_every_quiet_spell_until_cancelled() {
        // param_total stays at one, so the whole-list retry is never used up (:2119-2124).
        let t0 = Instant::now();
        let quiet = ProtocolTimeouts::default().param_list_quiet;
        let mut d = download(t0);
        for spell in 1..=5 {
            assert_eq!(d.on_tick(t0 + quiet * spell), ParamAction::RequestList);
        }
        d.cancel();
        assert_eq!(d.state(), ParamDownloadState::Cancelled);
        assert_eq!(d.on_tick(t0 + quiet * 10), ParamAction::Nothing);
    }

    #[test]
    fn a_vehicle_with_no_parameters_is_a_complete_download() {
        let t0 = Instant::now();
        let mut d = download(t0);
        d.on_param_value(INDEX_NOT_IN_LIST, 0, t0);
        assert_eq!(d.state(), ParamDownloadState::Complete);
    }

    #[test]
    fn an_upward_revision_reopens_the_count() {
        let t0 = Instant::now();
        let mut d = download(t0);
        d.on_param_value(0, 2, t0);
        d.on_param_value(1, 3, t0);
        assert_eq!(d.expected(), Some(3));
        assert_eq!(d.missing(), vec![2]);
        assert!(!d.is_finished());
    }
}
