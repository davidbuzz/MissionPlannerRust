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

//! `System.DateTime` as `GeoRefImageBase` uses it: ticks of 100 ns since 0001-01-01, the three
//! kinds, and the arithmetic whose rounding decides which log entry a photo is matched to.
//!
//! Everything here is .NET Framework's, which is what Mission Planner runs on and what mono's
//! `mscorlib` reproduces (the oracle's runtime): `AddSeconds` and `AddMilliseconds` round to the
//! whole millisecond half away from zero before adding, `TimeSpan.TotalMilliseconds` multiplies the
//! ticks by `0.0001` rather than dividing, and `Convert.ToInt64(double)` rounds half to even.
//!
//! **Local time is UTC here.** The C# converts through the machine's time zone in three places -
//! a dataflash item's time is built in local time and turned back with `ToUniversalTime`
//! (`DFLog.cs:702-711`, `GeoRefImageBase.cs:228`), a tlog's receive time is `ToLocalTime`d
//! (`MavlinkParse.cs:146-148`), and KML writes the offset of a local time - and on any machine
//! those round trips give back what went in, except across a daylight-saving change inside one
//! log. This port keeps the kind, so the files write what a machine on UTC writes, which is what the
//! oracle runs under.

use std::fmt::Write as _;

/// `DateTime.Kind`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Kind {
    /// `DateTimeKind.Unspecified`: a photo's EXIF time, and `DateTime.MinValue`.
    #[default]
    Unspecified,
    /// `DateTimeKind.Utc`: a time built from GPS week and milliseconds, or from Unix time.
    Utc,
    /// `DateTimeKind.Local`: a tlog's receive time.
    Local,
}

/// Ticks in a millisecond.
pub const TICKS_PER_MILLISECOND: i64 = 10_000;
/// Ticks in a second.
pub const TICKS_PER_SECOND: i64 = 10_000_000;
/// Ticks in a day.
pub const TICKS_PER_DAY: i64 = 864_000_000_000;
/// `DateTime.MaxValue.Ticks`: 9999-12-31 23:59:59.9999999.
pub const MAX_TICKS: i64 = 3_155_378_975_999_999_999;
/// 1970-01-01 in ticks.
pub const UNIX_EPOCH_TICKS: i64 = 621_355_968_000_000_000;
/// 1980-01-06, the GPS epoch, in ticks.
pub const GPS_EPOCH_TICKS: i64 = 624_515_616_000_000_000;

/// `DateTime.MaxValue.Ticks / TicksPerMillisecond`, the bound `DateTime.Add` checks.
/// `// C#: referencesource mscorlib/system/datetime.cs:112, 327-331`
const MAX_MILLIS: i64 = 315_537_897_600_000;

/// A `System.DateTime`.
///
/// Equality and order are the C#'s: by ticks alone, whatever the kind.
#[derive(Debug, Clone, Copy, Default)]
pub struct DateTime {
    /// `Ticks`.
    pub ticks: i64,
    /// `Kind`.
    pub kind: Kind,
}

impl PartialEq for DateTime {
    fn eq(&self, other: &Self) -> bool {
        self.ticks == other.ticks
    }
}

impl Eq for DateTime {}

impl PartialOrd for DateTime {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for DateTime {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.ticks.cmp(&other.ticks)
    }
}

/// Why a `DateTime` operation throws: `ArgumentOutOfRangeException`, worded as mono words it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error(
    "The added or subtracted value results in an un-representable DateTime.\nParameter name: value"
)]
pub struct OutOfRange;

impl DateTime {
    /// `DateTime.MinValue`.
    pub const MIN: Self = Self {
        ticks: 0,
        kind: Kind::Unspecified,
    };

    /// A time from its ticks.
    #[must_use]
    pub const fn from_ticks(ticks: i64, kind: Kind) -> Self {
        Self { ticks, kind }
    }

    /// `new DateTime(year, month, day, hour, minute, second, kind)`, or `None` where the
    /// constructor throws.
    #[must_use]
    pub fn from_parts(
        year: i32,
        month: i32,
        day: i32,
        hour: i32,
        minute: i32,
        second: i32,
        kind: Kind,
    ) -> Option<Self> {
        if !(1..=9999).contains(&year)
            || !(1..=12).contains(&month)
            || !(0..24).contains(&hour)
            || !(0..60).contains(&minute)
            || !(0..60).contains(&second)
        {
            return None;
        }
        if day < 1 || day > days_in_month(year, month) {
            return None;
        }
        let days = days_from_civil(i64::from(year), month, day) - days_from_civil(1, 1, 1);
        let seconds = i64::from(hour) * 3600 + i64::from(minute) * 60 + i64::from(second);
        Some(Self {
            ticks: days * TICKS_PER_DAY + seconds * TICKS_PER_SECOND,
            kind,
        })
    }

    /// Unix milliseconds as a UTC time: `FromUTCTimeMilliseconds`.
    /// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:484-488`
    ///
    /// # Errors
    ///
    /// Where `AddMilliseconds` throws.
    pub fn from_unix_millis(milliseconds: i64) -> Result<Self, OutOfRange> {
        // `epoch.AddMilliseconds(milliseconds)`: the long becomes a double first.
        #[allow(clippy::cast_precision_loss)]
        Self::from_ticks(UNIX_EPOCH_TICKS, Kind::Utc).add_milliseconds(milliseconds as f64)
    }

    /// `DateTime.Add(double value, int scale)`: `value * scale` milliseconds, rounded half away
    /// from zero to a whole millisecond, then added.
    /// `// C#: referencesource mscorlib/system/datetime.cs:325-331`
    fn add_scaled(self, value: f64, scale: f64) -> Result<Self, OutOfRange> {
        let scaled = value * scale + if value >= 0.0 { 0.5 } else { -0.5 };
        if scaled.is_nan() {
            return Err(OutOfRange);
        }
        // `(long)` of a double beyond the range is undefined in .NET; the range check that
        // follows rejects every such value, so saturating is only a way of reaching that check.
        #[allow(clippy::cast_possible_truncation)]
        let millis = scaled as i64;
        if millis <= -MAX_MILLIS || millis >= MAX_MILLIS {
            return Err(OutOfRange);
        }
        self.add_ticks(millis * TICKS_PER_MILLISECOND)
    }

    /// `AddMilliseconds`.
    ///
    /// # Errors
    ///
    /// Where the C# throws `ArgumentOutOfRangeException`.
    pub fn add_milliseconds(self, value: f64) -> Result<Self, OutOfRange> {
        self.add_scaled(value, 1.0)
    }

    /// `AddSeconds`.
    ///
    /// # Errors
    ///
    /// Where the C# throws `ArgumentOutOfRangeException`.
    pub fn add_seconds(self, value: f64) -> Result<Self, OutOfRange> {
        self.add_scaled(value, 1000.0)
    }

    /// `AddDays`.
    ///
    /// # Errors
    ///
    /// Where the C# throws `ArgumentOutOfRangeException`.
    pub fn add_days(self, value: f64) -> Result<Self, OutOfRange> {
        self.add_scaled(value, 86_400_000.0)
    }

    /// `AddTicks`.
    ///
    /// # Errors
    ///
    /// Where the C# throws `ArgumentOutOfRangeException`.
    pub fn add_ticks(self, value: i64) -> Result<Self, OutOfRange> {
        let ticks = self.ticks.checked_add(value).ok_or(OutOfRange)?;
        if !(0..=MAX_TICKS).contains(&ticks) {
            return Err(OutOfRange);
        }
        Ok(Self {
            ticks,
            kind: self.kind,
        })
    }

    /// `this - other`, as a `TimeSpan`'s ticks.
    #[must_use]
    pub const fn minus(self, other: Self) -> TimeSpan {
        TimeSpan(self.ticks - other.ticks)
    }

    /// `ToMilliseconds`: `Convert.ToInt64((date - epoch).TotalMilliseconds)` with a UTC epoch,
    /// whatever the date's kind.
    /// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:501-505`
    #[must_use]
    // A DateTime is at most 3.2e14 ms from the epoch, well inside an i64.
    #[allow(clippy::cast_possible_truncation)]
    pub fn to_milliseconds(self) -> i64 {
        self.minus(Self::from_ticks(UNIX_EPOCH_TICKS, Kind::Utc))
            .total_milliseconds()
            .round_ties_even() as i64
    }

    /// The civil date and time: year, month, day, hour, minute, second, and the ticks within the
    /// second.
    #[must_use]
    pub fn parts(self) -> (i64, i64, i64, i64, i64, i64, i64) {
        let days = self.ticks.div_euclid(TICKS_PER_DAY);
        let rest = self.ticks.rem_euclid(TICKS_PER_DAY);
        let (year, month, day) = civil_from_days(days + days_from_civil(1, 1, 1));
        let seconds = rest / TICKS_PER_SECOND;
        (
            year,
            month,
            day,
            seconds / 3600,
            seconds / 60 % 60,
            seconds % 60,
            rest % TICKS_PER_SECOND,
        )
    }

    /// `ToString("yyyy:MM:dd HH:mm:ss")` in the invariant culture.
    #[must_use]
    pub fn format_exif(self) -> String {
        let (y, mo, d, h, mi, s, _) = self.parts();
        format!("{y:04}:{mo:02}:{d:02} {h:02}:{mi:02}:{s:02}")
    }

    /// `ToString("yyyy-MM-ddTHH:mm:ssZ")`: `T` and `Z` are literals, not the time's kind.
    #[must_use]
    pub fn format_gpx(self) -> String {
        let (y, mo, d, h, mi, s, _) = self.parts();
        format!("{y:04}-{mo:02}-{d:02}T{h:02}:{mi:02}:{s:02}Z")
    }

    /// `ToString("yyyy-MM-ddTHH:mm:sszzzzzz")`, what SharpKml writes a `DateTime` as: `zzz` and
    /// longer is the offset from UTC as `+HH:mm`, which for a UTC time is `+00:00` and for any
    /// other kind is the machine's own - taken to be UTC (see the module notes).
    /// `// C#: ExtLibs/SharpKml/Base/KmlFormatter.cs:34-38`
    #[must_use]
    pub fn format_kml(self) -> String {
        let (y, mo, d, h, mi, s, _) = self.parts();
        format!("{y:04}-{mo:02}-{d:02}T{h:02}:{mi:02}:{s:02}+00:00")
    }

    /// `XmlConvert.ToString(value, XmlDateTimeSerializationMode.RoundtripKind)`, what
    /// `XmlSerializer` writes a `DateTime` as: `yyyy-MM-ddTHH:mm:ss.FFFFFFFK` - the fraction
    /// without its trailing zeros, and no point at all for a whole second; `K` is `Z` for UTC,
    /// the offset for local time (`+00:00`, see the module notes) and nothing otherwise.
    #[must_use]
    pub fn format_xml_roundtrip(self) -> String {
        let (y, mo, d, h, mi, s, fraction) = self.parts();
        let mut out = format!("{y:04}-{mo:02}-{d:02}T{h:02}:{mi:02}:{s:02}");
        if fraction != 0 {
            let digits = format!("{fraction:07}");
            let _ = write!(out, ".{}", digits.trim_end_matches('0'));
        }
        out.push_str(match self.kind {
            Kind::Utc => "Z",
            Kind::Local => "+00:00",
            Kind::Unspecified => "",
        });
        out
    }
}

/// A `System.TimeSpan`, as ticks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct TimeSpan(pub i64);

impl TimeSpan {
    /// `TotalMilliseconds`: the ticks times `MillisecondsPerTick`, clamped to the range a
    /// `TimeSpan` can express in whole milliseconds.
    /// `// C#: referencesource mscorlib/system/timespan.cs:139-151`
    #[must_use]
    pub fn total_milliseconds(self) -> f64 {
        const MILLISECONDS_PER_TICK: f64 = 1.0 / 10_000.0;
        #[allow(clippy::cast_precision_loss)]
        const MAX_MILLISECONDS: f64 = (i64::MAX / TICKS_PER_MILLISECOND) as f64;
        #[allow(clippy::cast_precision_loss)]
        const MIN_MILLISECONDS: f64 = (i64::MIN / TICKS_PER_MILLISECOND) as f64;
        #[allow(clippy::cast_precision_loss)]
        let temp = self.0 as f64 * MILLISECONDS_PER_TICK;
        // A NaN stays NaN, as the C#'s two comparisons leave it.
        temp.clamp(MIN_MILLISECONDS, MAX_MILLISECONDS)
    }

    /// `TotalSeconds`: the ticks times `SecondsPerTick`.
    /// `// C#: referencesource mscorlib/system/timespan.cs:157-159`
    #[must_use]
    pub fn total_seconds(self) -> f64 {
        const SECONDS_PER_TICK: f64 = 1.0 / 10_000_000.0;
        #[allow(clippy::cast_precision_loss)]
        let seconds = self.0 as f64 * SECONDS_PER_TICK;
        seconds
    }
}

const fn is_leap(year: i32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || year % 400 == 0
}

const fn days_in_month(year: i32, month: i32) -> i32 {
    match month {
        2 if is_leap(year) => 29,
        2 => 28,
        4 | 6 | 9 | 11 => 30,
        _ => 31,
    }
}

/// Days from 1970-01-01 to a proleptic Gregorian date.
fn days_from_civil(year: i64, month: i32, day: i32) -> i64 {
    let year = if month <= 2 { year - 1 } else { year };
    let era = year.div_euclid(400);
    let yoe = year.rem_euclid(400);
    let month = i64::from(month);
    let mp = if month > 2 { month - 3 } else { month + 9 };
    let doy = (153 * mp + 2) / 5 + i64::from(day) - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

/// The proleptic Gregorian date of a day count from 1970-01-01.
fn civil_from_days(days: i64) -> (i64, i64, i64) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    (yoe + era * 400 + i64::from(month <= 2), month, day)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_epochs_are_the_ticks_dotnet_gives() {
        let unix = DateTime::from_parts(1970, 1, 1, 0, 0, 0, Kind::Utc).unwrap();
        assert_eq!(unix.ticks, UNIX_EPOCH_TICKS);
        let gps = DateTime::from_parts(1980, 1, 6, 0, 0, 0, Kind::Utc).unwrap();
        assert_eq!(gps.ticks, GPS_EPOCH_TICKS);
        assert_eq!(unix.parts(), (1970, 1, 1, 0, 0, 0, 0));
        assert_eq!(DateTime::MIN.format_exif(), "0001:01:01 00:00:00");
    }

    #[test]
    fn add_seconds_rounds_to_the_millisecond_half_away_from_zero() {
        // What mono printed for new DateTime(2026,1,1).AddSeconds(-1.2345f): the float widens to
        // -1.2345000505447388 s, so -1234.50005 ms rounds away to -1235.
        let start = DateTime::from_parts(2026, 1, 1, 0, 0, 0, Kind::Unspecified).unwrap();
        let back = start.add_seconds(f64::from(-1.2345f32)).unwrap();
        assert_eq!(start.ticks - back.ticks, 1235 * TICKS_PER_MILLISECOND);
        assert_eq!(
            start.add_milliseconds(0.5).unwrap().ticks - start.ticks,
            TICKS_PER_MILLISECOND
        );
        assert_eq!(start.add_milliseconds(0.4).unwrap().ticks, start.ticks);
        assert!(DateTime::MIN.add_seconds(-1.0).is_err());
    }

    #[test]
    fn to_milliseconds_rounds_half_to_even() {
        let at = |ticks| DateTime::from_ticks(UNIX_EPOCH_TICKS + ticks, Kind::Utc);
        assert_eq!(at(5_000).to_milliseconds(), 0);
        assert_eq!(at(15_000).to_milliseconds(), 2);
        assert_eq!(at(25_000).to_milliseconds(), 2);
        assert_eq!(at(25_001).to_milliseconds(), 3);
        assert_eq!(at(-15_000).to_milliseconds(), -2);
    }

    #[test]
    fn the_xml_form_trims_the_fraction() {
        let t = DateTime::from_parts(2026, 9, 24, 1, 30, 5, Kind::Utc).unwrap();
        assert_eq!(t.format_xml_roundtrip(), "2026-09-24T01:30:05Z");
        let t = t.add_ticks(6_000_000).unwrap();
        assert_eq!(t.format_xml_roundtrip(), "2026-09-24T01:30:05.6Z");
        let t = DateTime::from_ticks(t.ticks + 7890, Kind::Local);
        assert_eq!(t.format_xml_roundtrip(), "2026-09-24T01:30:05.600789+00:00");
        assert_eq!(t.format_kml(), "2026-09-24T01:30:05+00:00");
        assert_eq!(t.format_gpx(), "2026-09-24T01:30:05Z");
    }
}
