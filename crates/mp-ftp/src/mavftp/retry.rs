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

//! How long each MAVFTP command waits for its answer, and how many times it is sent.
//!
//! Every command in `MAVFtp.cs` runs inside a `RetryTimeout`
//! (C#: ExtLibs/ArduPilot/RetryTimeout.cs): send, wait `TimeoutMS`, send again, `Retries` times in
//! all. The reply handler steers it through three fields, and this port keeps all three because
//! the commands lean on each one differently:
//!
//! * `Complete = true` ends the wait at once, successfully.
//! * `Retries = 0` stops further sends but does **not** end the current wait: `DoWork` only looks
//!   at `Retries` when a wait runs out (RetryTimeout.cs:63-76). So a command the vehicle refuses
//!   outright still takes its full timeout to report the refusal - two seconds for
//!   `kCmdOpenFileRO`, thirty for `kCmdCalcFileCRC32`.
//! * `RetriesCurrent = 0` gives the command its sends back; `RetriesCurrent = 999` (the caller
//!   cancelled) ends it when the current wait runs out.
//!
//! `DoWork` notices `Complete` by polling every 100 ms (RetryTimeout.cs:69-75). That poll is not
//! ported: a finished command finishes when its answer arrives, and the next command in a
//! sequence starts then too, up to 100 ms sooner than the C#'s would.

use web_time::{Duration, Instant};

/// One command's patience, as `new RetryTimeout(Retrys, TimeoutMS)` sets it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Patience {
    /// `Retries`: how many times the command goes on the wire, the first send included, before
    /// it gives up. Not "retries after the first": `DoWork`'s loop runs from zero while
    /// `RetriesCurrent < Retries` and sends once per pass (RetryTimeout.cs:63-67).
    pub retries: i32,
    /// `TimeoutMS`: how long each send waits.
    pub timeout: Duration,
}

impl Patience {
    const fn new(retries: i32, millis: u64) -> Self {
        Self {
            retries,
            timeout: Duration::from_millis(millis),
        }
    }
}

/// Every command's patience. [`Default`] is Mission Planner's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FtpTimeouts {
    /// `kCmdOpenFileRO`. C#: MAVFtp.cs:647 (`new RetryTimeout(5, 2000)`).
    pub open_file_ro: Patience,
    /// `kCmdCalcFileCRC32`. C#: MAVFtp.cs:967 (`new RetryTimeout(3, 30000)`).
    pub calc_file_crc32: Patience,
    /// `kCmdListDirectory` and `kCmdListDirectoryWithTime`. C#: MAVFtp.cs:1326
    /// (`new RetryTimeout(5)`, the wait left at RetryTimeout.cs:37's 1000 ms).
    pub list_directory: Patience,
    /// `kCmdResetSessions`. C#: MAVFtp.cs:1958 (`new RetryTimeout(5, 1000)`).
    pub reset_sessions: Patience,
    /// Every other command: `new RetryTimeout()`, which is 30 sends a second apart
    /// (RetryTimeout.cs:37). C#: MAVFtp.cs:735 (`kCmdBurstReadFile`), :1061
    /// (`kCmdCreateDirectory`), :1153 (`kCmdCreateFile`), :1549 (`kCmdReadFile`), :1677
    /// (`kCmdRemoveDirectory`), :1762 (`kCmdRemoveFile`), :1846 (`kCmdRename`), :1974
    /// (`kCmdTerminateSession`), :2215 (`kCmdWriteFile`).
    pub other: Patience,
}

impl Default for FtpTimeouts {
    fn default() -> Self {
        Self {
            open_file_ro: Patience::new(5, 2000),
            calc_file_crc32: Patience::new(3, 30_000),
            list_directory: Patience::new(5, 1000),
            reset_sessions: Patience::new(5, 1000),
            other: Patience::new(30, 1000),
        }
    }
}

impl FtpTimeouts {
    /// Every wait divided by `divisor`, every count unchanged: for tests, which need the C#'s
    /// sends on the wire without its wall-clock time.
    #[must_use]
    pub fn faster(self, divisor: u32) -> Self {
        let divisor = divisor.max(1);
        let scale = |p: Patience| Patience {
            retries: p.retries,
            timeout: p.timeout / divisor,
        };
        Self {
            open_file_ro: scale(self.open_file_ro),
            calc_file_crc32: scale(self.calc_file_crc32),
            list_directory: scale(self.list_directory),
            reset_sessions: scale(self.reset_sessions),
            other: scale(self.other),
        }
    }
}

/// What a wait running out means.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Expiry {
    /// The wait has not run out.
    Waiting,
    /// It ran out and there are sends left: send again.
    Again,
    /// It ran out and there are none: the command gives up.
    GaveUp,
}

/// A `RetryTimeout` in progress.
///
/// C#: ExtLibs/ArduPilot/RetryTimeout.cs.
#[derive(Debug, Clone)]
pub(crate) struct RetryTimeout {
    /// `Complete`.
    pub complete: bool,
    /// `Retries`.
    pub retries: i32,
    /// `RetriesCurrent`.
    pub retries_current: i32,
    /// `TimeoutMS`.
    timeout: Duration,
    /// `TimeOutDateTime`.
    deadline: Instant,
}

impl RetryTimeout {
    pub(crate) fn new(patience: Patience, now: Instant) -> Self {
        Self {
            complete: false,
            retries: patience.retries,
            retries_current: 0,
            timeout: patience.timeout,
            deadline: now,
        }
    }

    /// The top of `DoWork`: whether its loop makes a first pass at all.
    ///
    /// C#: RetryTimeout.cs:62-63 (`Complete = false; for (RetriesCurrent = 0; ...`).
    pub(crate) fn begin(&mut self) -> bool {
        self.complete = false;
        self.retries_current = 0;
        self.retries_current < self.retries
    }

    /// Starts the wait after a pass's `WorkToDo`: `TimeOutDateTime = DateTime.Now.AddMilliseconds
    /// (TimeoutMS)`, RetryTimeout.cs:68. `ResetTimeout` (:82-85) is the same thing.
    pub(crate) fn arm(&mut self, now: Instant) {
        self.deadline = now + self.timeout;
    }

    /// The bottom of `DoWork`'s wait: once `TimeOutDateTime` has passed, `RetriesCurrent++` and
    /// the loop's condition. C#: RetryTimeout.cs:63, 69.
    pub(crate) fn expire(&mut self, now: Instant) -> Expiry {
        if now < self.deadline {
            return Expiry::Waiting;
        }
        self.retries_current = self.retries_current.saturating_add(1);
        if self.retries_current < self.retries {
            Expiry::Again
        } else {
            Expiry::GaveUp
        }
    }

    /// What the handlers do when the caller has cancelled: `timeout.RetriesCurrent = 999`
    /// (e.g. MAVFtp.cs:656), which ends the command when the current wait runs out.
    pub(crate) fn cancel(&mut self) {
        self.retries_current = 999;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn retries_counts_sends_not_resends() {
        // new RetryTimeout(5, 2000): five sends in all, two seconds apart, then give up.
        let start = Instant::now();
        let mut retry = RetryTimeout::new(Patience::new(5, 2000), start);
        assert!(retry.begin());
        let mut sends = 1;
        let mut now = start;
        retry.arm(now);
        loop {
            now += Duration::from_millis(2000);
            match retry.expire(now) {
                Expiry::Waiting => panic!("the wait has run out"),
                Expiry::Again => {
                    sends += 1;
                    retry.arm(now);
                }
                Expiry::GaveUp => break,
            }
        }
        assert_eq!(sends, 5);
        assert_eq!(now - start, Duration::from_secs(10));
    }

    #[test]
    fn stopping_retries_still_waits_out_the_current_wait() {
        // timeout.Retries = 0 is how a handler says "stop"; DoWork only reads it when the wait
        // runs out, so a refusal is reported a whole timeout after it arrived.
        let start = Instant::now();
        let mut retry = RetryTimeout::new(Patience::new(5, 2000), start);
        assert!(retry.begin());
        retry.arm(start);
        retry.retries = 0;
        assert_eq!(
            retry.expire(start + Duration::from_millis(1999)),
            Expiry::Waiting
        );
        assert_eq!(
            retry.expire(start + Duration::from_millis(2000)),
            Expiry::GaveUp
        );
    }

    #[test]
    fn giving_the_sends_back_leaves_one_fewer_than_a_fresh_start() {
        // RetriesCurrent = 0 in a handler, then DoWork's ++ when the wait runs out: the command
        // has Retries - 1 more sends, not Retries.
        let start = Instant::now();
        let mut retry = RetryTimeout::new(Patience::new(3, 100), start);
        assert!(retry.begin());
        retry.arm(start);
        retry.retries_current = 0;
        let mut now = start;
        let mut again = 0;
        loop {
            now += Duration::from_millis(100);
            match retry.expire(now) {
                Expiry::Waiting => unreachable!(),
                Expiry::Again => {
                    again += 1;
                    retry.arm(now);
                }
                Expiry::GaveUp => break,
            }
        }
        assert_eq!(again, 2);
    }

    #[test]
    fn faster_divides_the_waits_and_keeps_the_counts() {
        let fast = FtpTimeouts::default().faster(100);
        assert_eq!(fast.calc_file_crc32.retries, 3);
        assert_eq!(fast.calc_file_crc32.timeout, Duration::from_millis(300));
        assert_eq!(fast.other.retries, 30);
        assert_eq!(fast.open_file_ro.timeout, Duration::from_millis(20));
    }
}
