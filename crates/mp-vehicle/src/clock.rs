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

//! The clock `CurrentState` keeps: `datetime`, a C# `System.DateTime`.
//!
//! Several properties are rates or running totals over time - the vertical speed, the time in
//! air, the distance travelled, the battery's used capacity - and the C# measures them against
//! `CurrentState.datetime`, which `MAVLinkInterface` stamps before each packet: with
//! `DateTime.Now` on a live link (`MAVLinkInterface.cs:4721`) and with the packet's recorded time
//! when a `.tlog` is played back (`MAVLinkInterface.cs:6649`). [`DateTime`] is that value, held as
//! the C# holds it - ticks of 100 ns since 0001-01-01 - so the arithmetic on it is the C#'s to the
//! bit, including against `DateTime.MinValue`, the value every one of these clocks starts at.
//!
//! The C#'s `DateTime.Now` is local time; [`DateTime::now`] is UTC. Only differences and the
//! seconds field are ever computed from it, and a time zone changes neither.

use web_time::{SystemTime, UNIX_EPOCH};

/// A C# `DateTime`: ticks of 100 ns since 0001-01-01 00:00:00.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DateTime {
    ticks: i64,
}

/// `TimeSpan.TicksPerSecond`.
const TICKS_PER_SECOND: i64 = 10_000_000;
/// `TimeSpan.TicksPerMillisecond`.
const TICKS_PER_MILLISECOND: i64 = 10_000;
/// `new DateTime(1970, 1, 1).Ticks`.
const UNIX_EPOCH_TICKS: i64 = 621_355_968_000_000_000;
/// `TimeSpan.SecondsPerTick`: `1.0 / TicksPerSecond`, a double constant the C# multiplies by.
const SECONDS_PER_TICK: f64 = 1.0 / 10_000_000.0;
/// `TimeSpan.HoursPerTick`: `1.0 / TicksPerHour`.
const HOURS_PER_TICK: f64 = 1.0 / 36_000_000_000.0;

impl DateTime {
    /// `DateTime.MinValue`, which every clock in `CurrentState` starts at.
    pub const MIN: Self = Self { ticks: 0 };

    /// A time from its ticks.
    #[must_use]
    pub const fn from_ticks(ticks: i64) -> Self {
        Self { ticks }
    }

    /// `DateTime.Ticks`.
    #[must_use]
    pub const fn ticks(self) -> i64 {
        self.ticks
    }

    /// The time a `.tlog` record's timestamp stands for, as `readlogPacketMavlink` reads it:
    /// microseconds since the Unix epoch cut to whole milliseconds, `new DateTime(1970, 1, 1)
    /// .AddMilliseconds(dateint / 1000)`. `None` for a stamp the C# ignores - 9,999,999 hours or
    /// more - which leaves its clock where it was.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6539-6558`
    #[must_use]
    pub fn from_tlog_micros(micros: u64) -> Option<Self> {
        // C#: MAVLinkInterface.cs:6553
        if micros / 1000 / 1000 / 60 / 60 >= 9_999_999 {
            return None;
        }
        // C#: MAVLinkInterface.cs:6555, `dateint / 1000` in integer arithmetic: whole
        // milliseconds, which AddMilliseconds keeps whole. Below 9,999,999 hours it fits.
        let millis = i64::try_from(micros / 1000).ok()?;
        Some(Self {
            ticks: UNIX_EPOCH_TICKS + millis * TICKS_PER_MILLISECOND,
        })
    }

    /// Now, what a live link stamps each packet with (`MAVLinkInterface.cs:4721`). UTC; see the
    /// module documentation.
    #[must_use]
    pub fn now() -> Self {
        let since_epoch = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default();
        let ticks = i64::try_from(since_epoch.as_nanos() / 100).unwrap_or(i64::MAX);
        Self {
            ticks: UNIX_EPOCH_TICKS.saturating_add(ticks),
        }
    }

    /// `DateTime.Second`: the seconds field, 0 to 59.
    #[must_use]
    pub const fn second(self) -> i64 {
        (self.ticks / TICKS_PER_SECOND) % 60
    }

    /// `(self - earlier).TotalSeconds`, negative when `earlier` is later.
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // `(double)_ticks`, as TimeSpan does it
    pub fn seconds_since(self, earlier: Self) -> f64 {
        (self.ticks - earlier.ticks) as f64 * SECONDS_PER_TICK
    }

    /// `(self - earlier).TotalHours`.
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // `(double)_ticks`, as TimeSpan does it
    pub fn hours_since(self, earlier: Self) -> f64 {
        (self.ticks - earlier.ticks) as f64 * HOURS_PER_TICK
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn min_value_is_the_start_of_year_one_with_second_zero() {
        assert_eq!(DateTime::MIN.ticks(), 0);
        assert_eq!(DateTime::MIN.second(), 0);
        assert_eq!(DateTime::default(), DateTime::MIN);
    }

    #[test]
    fn a_tlog_stamp_is_cut_to_whole_milliseconds_from_1970() {
        // 2026-01-01T00:00:07.123456Z
        let micros = 1_767_225_607_123_456;
        let time = DateTime::from_tlog_micros(micros).unwrap();
        assert_eq!(
            time.ticks(),
            UNIX_EPOCH_TICKS + 1_767_225_607_123 * TICKS_PER_MILLISECOND
        );
        assert_eq!(time.second(), 7);
        // The C# ignores a stamp of 9,999,999 hours or more (MAVLinkInterface.cs:6553).
        assert_eq!(DateTime::from_tlog_micros(9_999_999 * 3_600_000_000), None);
        assert!(DateTime::from_tlog_micros(9_999_998 * 3_600_000_000).is_some());
    }

    #[test]
    fn spans_are_the_timespans_doubles() {
        let a = DateTime::from_ticks(12_345_678_901);
        let b = DateTime::from_ticks(12_345_678_901 + 2_500_000);
        assert_eq!(b.seconds_since(a), 2_500_000_f64 * (1.0 / 10_000_000.0));
        assert_eq!(a.seconds_since(b), -0.25);
        assert_eq!(b.hours_since(a), 2_500_000_f64 * (1.0 / 36_000_000_000.0));
        // From MinValue, as the first reading of every clock is.
        assert!(b.seconds_since(DateTime::MIN) > 1000.0);
    }

    #[test]
    fn now_is_after_1970() {
        assert!(DateTime::now().ticks() > UNIX_EPOCH_TICKS);
    }
}
