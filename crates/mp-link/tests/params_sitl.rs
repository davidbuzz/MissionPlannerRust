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

//! Parameter decoding against a real vehicle.
//!
//! Ignored by default; start SITL with `tools/sitl/run-sitl.sh copter` and run
//! `cargo test -p mp-link -- --ignored`.
//!
//! This exists because the bug it catches is invisible to a unit test. Both the specification
//! reading and ArduPilot's reading of `PARAM_VALUE` produce *a* number, so a round-trip test
//! passes either way. Only comparing against what the values physically mean shows that a 3,300
//! mAh battery was being read as 1,162,756,096.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use web_time::{Duration, Instant};

use mp_link::{Link, LinkConfig};

fn download_params() -> mp_params::ParamTable {
    let config = LinkConfig {
        stream_rate_hz: 0,
        ..LinkConfig::default()
    };
    let link = Link::connect("tcp:127.0.0.1:5760", config).expect("SITL on port 5760");

    let deadline = Instant::now() + Duration::from_secs(20);
    while link.primary_vehicle().is_none() && Instant::now() < deadline {
        wasm_thread::sleep(Duration::from_millis(100));
    }
    let (id, _) = link.primary_vehicle().expect("a vehicle");
    link.download_params(id);

    let deadline = Instant::now() + Duration::from_secs(120);
    loop {
        if let Some(table) = link.params(id)
            && table.is_complete()
        {
            return table;
        }
        assert!(Instant::now() < deadline, "parameter download timed out");
        wasm_thread::sleep(Duration::from_millis(100));
    }
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn parameter_values_are_physically_plausible() {
    let table = download_params();
    assert!(
        table.len() > 1_000,
        "expected a full parameter set, got {}",
        table.len()
    );

    // Each of these has a range no correct decoding can leave. A value outside it means the
    // bytes were interpreted with the wrong rule, which is what happened before this test existed.
    let checks: &[(&str, f64, f64)] = &[
        ("BATT_CAPACITY", 0.0, 100_000.0), // mAh; read wrongly this was 1.16e9
        ("BATT_MONITOR", 0.0, 100.0),      // an enumeration with a few dozen entries
        ("FS_THR_ENABLE", 0.0, 10.0),      // a small enumeration
        ("SERIAL0_BAUD", 0.0, 2_000_000.0), // ArduPilot stores this divided by 1000
        ("SYSID_THISMAV", 0.0, 255.0),     // a MAVLink system id, by definition one byte
        ("FS_GCS_ENABLE", 0.0, 10.0),
    ];

    let mut checked = 0;
    for (name, low, high) in checks {
        let Some(value) = table.get(name) else {
            continue;
        };
        let number = value.as_f64();
        assert!(
            number >= *low && number <= *high,
            "{name} = {number} is outside {low}..{high}; the parameter encoding is wrong for \
             this autopilot"
        );
        checked += 1;
    }
    assert!(
        checked >= 4,
        "only {checked} of the sanity parameters were present"
    );
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn integer_parameters_are_whole_numbers() {
    // An integer parameter carried as a float must read back as an integer. A configuration
    // screen showing 2.9999998 for a flight mode is alarming, and comparisons against it fail.
    let table = download_params();

    let mut checked = 0;
    for (name, value) in table.iter() {
        if !value.param_type().is_integer() {
            continue;
        }
        let number = value.as_f64();
        assert!(
            (number - number.round()).abs() < f64::EPSILON,
            "{name} is an integer parameter but reads as {number}"
        );
        checked += 1;
    }
    assert!(
        checked > 500,
        "expected many integer parameters, checked {checked}"
    );
}
