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

//! A release's version as this project numbers its releases, read from a tag (`v0_0_2`), an
//! archive's name (`0_0_2`) or a manifest (`0.0.2`) alike: the owner keeps the tags' underscores
//! (2026-11-18), so the numbers are what is compared, however they were joined. Not in the C#,
//! whose `System.Version` (`version.rs`) reads four numbers joined by points alone: updates.md,
//! section 4.3.

use std::cmp::Ordering;

/// The numbers of a release, most significant first; at least one.
#[derive(Debug, Clone, Eq)]
pub struct ReleaseVersion(Vec<u64>);

impl ReleaseVersion {
    /// The numbers of `text`: an optional `v` or `V`, then one to eight whole numbers joined by
    /// points or underscores. Anything else is an error that says what was read.
    pub fn parse(text: &str) -> Result<Self, String> {
        let trimmed = text.trim();
        let numbers = trimmed.strip_prefix(['v', 'V']).unwrap_or(trimmed);
        let bad = || format!("not a release version: {text:?}");
        let mut parts = Vec::new();
        for part in numbers.split(['.', '_']) {
            if part.is_empty() || !part.bytes().all(|b| b.is_ascii_digit()) {
                return Err(bad());
            }
            parts.push(part.parse::<u64>().map_err(|_| bad())?);
        }
        if parts.is_empty() || parts.len() > 8 {
            return Err(bad());
        }
        Ok(Self(parts))
    }

    /// Version nought, below every release.
    #[must_use]
    pub fn zero() -> Self {
        Self(vec![0])
    }

    /// The numbers, a missing one read as zero: `0.1` is `0.1.0`.
    fn padded(&self, len: usize) -> impl Iterator<Item = u64> + '_ {
        self.0.iter().copied().chain(std::iter::repeat(0)).take(len)
    }
}

impl PartialEq for ReleaseVersion {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl PartialOrd for ReleaseVersion {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for ReleaseVersion {
    fn cmp(&self, other: &Self) -> Ordering {
        let len = self.0.len().max(other.0.len());
        self.padded(len).cmp(other.padded(len))
    }
}

impl std::fmt::Display for ReleaseVersion {
    /// The numbers joined by points, as many as were given.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let text: Vec<String> = self.0.iter().map(u64::to_string).collect();
        f.write_str(&text.join("."))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(text: &str) -> ReleaseVersion {
        ReleaseVersion::parse(text).unwrap()
    }

    #[test]
    fn a_tag_an_archive_name_and_a_manifest_read_alike() {
        assert_eq!(v("v0_0_2"), v("0.0.2"));
        assert_eq!(v("0_0_2"), v("V0.0.2"));
        assert_eq!(v(" 0.0.2\n"), v("0.0.2"));
        assert_eq!(v("v0_0_2").to_string(), "0.0.2");
        assert_eq!(v("v1").to_string(), "1");
    }

    #[test]
    fn versions_order_by_their_numbers_not_their_text() {
        assert!(v("v0_0_2") > v("v0_0_1"));
        assert!(v("0.0.10") > v("0.0.9"));
        assert!(v("0.1") > v("0.0.99"));
        assert!(v("1") > v("0.99.99"));
        // A missing number is zero.
        assert_eq!(v("0.1"), v("0.1.0"));
        assert!(v("0.1.1") > v("0.1"));
    }

    #[test]
    fn what_is_not_a_version_is_refused_by_name() {
        for text in ["", "v", "beta", "0..2", "0.0.2-beta", "0.x", "-1.0", "1.2.3.4.5.6.7.8.9"] {
            let error = ReleaseVersion::parse(text).unwrap_err();
            assert!(error.contains(&format!("{text:?}")), "{error}");
        }
    }
}
