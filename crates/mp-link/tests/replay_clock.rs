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

//! A recorded flight played through the real link carries the recording's clock (PLAN.md §13.4
//! row 39).
//!
//! `readlogPacketMavlink` stamps the sender's `cs.datetime` with the record's timestamp, cut to
//! whole milliseconds (`MAVLinkInterface.cs:6500-6519, 6613`), and `testdata/currentstate/
//! autotest.csv` holds what `CurrentState` computed under mono when `autotest.tlog` was played
//! through `MAVLinkInterface` - vehicle 1:1's `datetime` in ticks among it, at the file position
//! after each packet that changed a field. Here the same file goes through [`ReplayTransport`] and
//! [`Link`] - the path `file:` links and the flight screen's Load Log take - cut at several of the
//! C#'s rows, and the link's vehicle 1:1 must hold the C#'s `datetime` to the tick at the end of
//! each: unpaced, as a `file:` link plays it, and paced, as the flight screen plays it.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::PathBuf;
use web_time::{Duration, Instant};

use mp_link::{Link, LinkConfig};
use mp_transport::ReplayTransport;
use mp_vehicle::VehicleId;

/// The autopilot the C#'s rows describe.
const AUTOPILOT: VehicleId = VehicleId::new(1, 1);

fn testdata(path: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(path)
}

/// The C#'s rows as `(file position after the packet, datetime ticks)`.
fn oracle() -> Vec<(usize, i64)> {
    let text = std::fs::read_to_string(testdata("currentstate/autotest.csv")).unwrap();
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().unwrap().split(',').collect();
    let position = header.iter().position(|name| *name == "position").unwrap();
    let ticks = header
        .iter()
        .position(|name| *name == "datetime_ticks")
        .unwrap();
    lines
        .filter(|line| !line.starts_with('#'))
        .map(|line| {
            let values: Vec<&str> = line.split(',').collect();
            (
                values[position].parse().unwrap(),
                values[ticks].parse().unwrap(),
            )
        })
        .collect()
}

/// Five of the C#'s rows, first to last.
fn points(rows: &[(usize, i64)]) -> Vec<(usize, i64)> {
    let last = rows.len() - 1;
    [0, last / 4, last / 2, 3 * last / 4, last]
        .iter()
        .map(|&index| rows[index])
        .collect()
}

/// A link replaying `data` that neither records nor sends a heartbeat, as the flight screen's
/// `replay` opens one.
fn config() -> LinkConfig {
    LinkConfig {
        record_path: None,
        send_heartbeat: false,
        ..LinkConfig::default()
    }
}

#[test]
fn a_file_link_stamps_each_packet_with_its_records_time_as_the_csharp_does() {
    let data = std::fs::read(testdata("mavlink/autotest.tlog")).unwrap();
    let rows = oracle();
    assert!(rows.len() > 3000, "{} rows", rows.len());
    for (position, ticks) in points(&rows) {
        let replay = ReplayTransport::from_bytes("autotest.tlog", data[..position].to_vec());
        let link = Link::from_transport(Box::new(replay), config());
        let deadline = Instant::now() + Duration::from_secs(60);
        while link.is_running() {
            assert!(
                Instant::now() < deadline,
                "the replay to {position} never ended"
            );
            wasm_thread::sleep(Duration::from_millis(5));
        }
        let state = link.vehicle(AUTOPILOT).expect("vehicle 1:1").load();
        assert_eq!(
            state.datetime.ticks(),
            ticks,
            "datetime after the packet ending at byte {position}"
        );
    }
}

#[test]
fn the_flight_screens_paced_replay_carries_the_same_clock() {
    let data = std::fs::read(testdata("mavlink/autotest.tlog")).unwrap();
    let rows = oracle();
    let (position, ticks) = rows[rows.len() / 2];
    let (replay, control) =
        ReplayTransport::from_bytes("autotest.tlog", data[..position].to_vec()).paced();
    // As fast as it will go: the clock is the recording's, whatever the speed.
    control.set_speed(1.0e9);
    let mut link = Link::from_transport(Box::new(replay), config());
    let deadline = Instant::now() + Duration::from_secs(60);
    while control.position() < position {
        assert!(Instant::now() < deadline, "the paced replay never finished");
        wasm_thread::sleep(Duration::from_millis(5));
    }
    // A publish after the last packet.
    let published = link.stats().publishes;
    while link.stats().publishes < published + 2 {
        assert!(Instant::now() < deadline, "nothing published");
        wasm_thread::sleep(Duration::from_millis(5));
    }
    let state = link.vehicle(AUTOPILOT).expect("vehicle 1:1").load();
    assert_eq!(state.datetime.ticks(), ticks, "paced, to byte {position}");
    link.close();
}
