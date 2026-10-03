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

//! Mission transfer against a real vehicle.
//!
//! Ignored by default because it needs ArduPilot SITL, but ignored rather than silently skipped:
//! the test output says it exists and was not run. Start SITL with `tools/sitl/run-sitl.sh copter`
//! and run `cargo test -p mp-link -- --ignored`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::time::{Duration, Instant};

use mp_link::mission_transfer::TransferState;
use mp_link::{Link, LinkConfig};
use mp_mission::MissionItem;
use mp_vehicle::VehicleId;

fn connect() -> (Link, VehicleId) {
    let config = LinkConfig {
        stream_rate_hz: 0,
        ..LinkConfig::default()
    };
    let link = Link::connect("tcp:127.0.0.1:5760", config).expect("SITL on port 5760");

    let deadline = Instant::now() + Duration::from_secs(20);
    while link.primary_vehicle().is_none() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(100));
    }
    let (id, _) = link.primary_vehicle().expect("a vehicle");
    (link, id)
}

fn await_transfer(link: &Link, id: VehicleId) -> Vec<MissionItem> {
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        if let Some(transfer) = link.mission_transfer(id) {
            match transfer.state() {
                TransferState::Complete => return transfer.items().to_vec(),
                TransferState::Failed(err) => panic!("transfer failed: {err}"),
                _ => {}
            }
        }
        assert!(Instant::now() < deadline, "transfer timed out");
        std::thread::sleep(Duration::from_millis(50));
    }
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn a_mission_uploaded_to_a_real_vehicle_reads_back_unchanged() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/missions/AVCMission_copter_AVC2013_mission.txt"
    );
    let text = std::fs::read_to_string(path).expect("mission corpus");
    let sent = mp_mission::read_waypoints(&text).expect("parses");
    assert!(sent.len() > 5, "want a mission with some substance");

    let (link, id) = connect();

    link.upload_mission(id, sent.clone());
    await_transfer(&link, id);

    link.download_mission(id);
    let received = await_transfer(&link, id);

    // The vehicle is the oracle here: whatever it stored is what it will fly. Reading back is the
    // only way to know it stored what was sent rather than what it felt like storing.
    assert_eq!(
        received.len(),
        sent.len(),
        "item count changed in the vehicle"
    );

    for (sent_item, got) in sent.iter().zip(&received) {
        assert_eq!(
            got.command, sent_item.command,
            "command changed at seq {}",
            sent_item.seq
        );
        assert_eq!(
            got.frame, sent_item.frame,
            "frame changed at seq {}",
            sent_item.seq
        );

        // 1e7 fixed point is about a centimetre; anything larger is a scaling bug.
        assert!(
            (got.x - sent_item.x).abs() < 1e-6,
            "seq {}: latitude {} came back as {}",
            sent_item.seq,
            sent_item.x,
            got.x
        );
        assert!(
            (got.y - sent_item.y).abs() < 1e-6,
            "seq {}: longitude {} came back as {}",
            sent_item.seq,
            sent_item.y,
            got.y
        );
        assert!(
            (got.z - sent_item.z).abs() < 0.01,
            "seq {}: altitude {} came back as {}",
            sent_item.seq,
            sent_item.z,
            got.z
        );

        // Non-navigation commands carry parameters in the coordinate fields; those must survive
        // unscaled, which is the failure mode this catches.
        if !sent_item.is_navigation() {
            assert!(
                (got.param1 - sent_item.param1).abs() < 1e-3,
                "seq {}: command {} param1 {} came back as {}",
                sent_item.seq,
                sent_item.command,
                sent_item.param1,
                got.param1
            );
        }
    }
}

#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn an_empty_mission_download_succeeds_rather_than_hanging() {
    // A vehicle with no mission answers MISSION_COUNT 0, and the protocol still expects an ack.
    // Treating zero as "not started" hangs the transfer until it times out.
    let (link, id) = connect();
    link.upload_mission(
        id,
        vec![MissionItem {
            seq: 0,
            ..MissionItem::default()
        }],
    );
    await_transfer(&link, id);

    link.download_mission(id);
    let items = await_transfer(&link, id);
    assert!(
        items.len() <= 1,
        "expected a near-empty mission, got {}",
        items.len()
    );
}

/// A terrain-frame mission survives the trip to a real vehicle and back.
///
/// The frame is the whole point of the item. A mission planned at 80 m above the ground, uploaded
/// and read back as 80 m above *home*, is a mission that flies into the first hill - and it reads
/// identically in every other field, so nothing else in a round-trip test would notice.
///
/// ArduPilot accepts `MAV_FRAME_GLOBAL_TERRAIN_ALT` in a mission whether or not it has terrain
/// data; it refuses at execution, not at upload. So this asserts storage fidelity, which is what
/// a ground station is responsible for.
#[test]
#[ignore = "needs ArduPilot SITL on tcp:127.0.0.1:5760"]
fn a_terrain_frame_mission_reads_back_in_the_same_frame() {
    /// `MAV_FRAME_GLOBAL_TERRAIN_ALT`.
    const TERRAIN: u8 = 10;
    let (link, id) = connect();

    let items = vec![
        MissionItem {
            seq: 0,
            current: 1,
            frame: 0,
            command: 16,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: -35.363_262,
            y: 149.165_237,
            z: 584.0,
            autocontinue: 1,
        },
        MissionItem {
            seq: 1,
            current: 0,
            frame: TERRAIN,
            command: 16,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: -35.362_000,
            y: 149.166_000,
            z: 80.0,
            autocontinue: 1,
        },
        MissionItem {
            seq: 2,
            current: 0,
            frame: TERRAIN,
            command: 16,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: -35.361_000,
            y: 149.167_000,
            z: 120.0,
            autocontinue: 1,
        },
    ];

    link.upload_mission(id, items.clone());
    let uploaded = await_transfer(&link, id);
    assert!(
        uploaded.is_empty() || uploaded.len() == items.len(),
        "upload reported {} items",
        uploaded.len()
    );

    link.download_mission(id);
    let read_back = await_transfer(&link, id);
    assert_eq!(read_back.len(), items.len(), "item count changed");

    for (sent, got) in items.iter().zip(read_back.iter()) {
        assert_eq!(
            got.frame, sent.frame,
            "item {} came back in frame {} having been sent in {}",
            sent.seq, got.frame, sent.frame
        );
        assert!(
            (got.z - sent.z).abs() < 0.01,
            "item {} altitude {} != {}",
            sent.seq,
            got.z,
            sent.z
        );
    }
    assert!(
        read_back
            .iter()
            .filter(|item| item.frame == TERRAIN)
            .count()
            == 2,
        "both terrain waypoints should come back in the terrain frame"
    );
}
