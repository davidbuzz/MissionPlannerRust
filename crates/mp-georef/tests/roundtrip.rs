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

//! `ToString("R")` - what the positions' XML beside the log is written with - held to 5,000
//! numbers the oracle formatted under mono (`testdata/georef/golden/roundtrip.txt`): log-shaped
//! latitudes and altitudes, random doubles of every magnitude, and floats.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

mod common;

use mp_georef::numfmt::{double_roundtrip, single_roundtrip};

#[test]
fn round_trip_formats_match_mono() {
    let text = std::fs::read_to_string(common::data().join("golden/roundtrip.txt")).unwrap();
    let mut failures = Vec::new();
    let mut count = 0;
    for line in text.lines() {
        let parts: Vec<&str> = line.split(' ').collect();
        let (got, want) = if parts[0] == "F" {
            let bits = u32::from_str_radix(parts[1], 16).unwrap();
            (single_roundtrip(f32::from_bits(bits)), parts[2])
        } else {
            let bits = u64::from_str_radix(parts[0], 16).unwrap();
            (double_roundtrip(f64::from_bits(bits)), parts[1])
        };
        count += 1;
        if got != want {
            failures.push(format!("{line}: got {got}"));
        }
    }
    assert_eq!(count, 5000);
    assert!(
        failures.is_empty(),
        "{} of 5000 differ:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
