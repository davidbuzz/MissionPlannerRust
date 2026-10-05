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

//! The packet-in stamp behind the packet-to-pixel measurement (DELIVERABLES.md D9): a link
//! asked to stamp arrivals puts, in each vehicle's snapshot, when its newest frame reached the
//! link; a link not asked - every product path - leaves it empty.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use web_time::{Duration, Instant};

use mp_link::testing::{attitude, heartbeat};
use mp_link::{Link, LinkConfig};
use mp_transport::Transport as _;
use mp_transport::testing::Loopback;
use mp_vehicle::{StateHandle, VehicleId};

/// A link over an in-memory transport, and the vehicle's end of it.
fn link(stamp_arrivals: bool) -> (Link, mp_transport::testing::LoopbackEnd) {
    let (gcs_side, vehicle_side) = Loopback::pair();
    let config = LinkConfig {
        stream_rate_hz: 0,
        send_heartbeat: false,
        publish_interval: Duration::from_millis(5),
        stamp_arrivals,
        ..LinkConfig::default()
    };
    (
        Link::from_transport(Box::new(gcs_side), config),
        vehicle_side,
    )
}

/// Waits for the snapshot to satisfy `done`, failing rather than hanging.
fn wait_for(link: &Link, done: impl Fn(&mp_vehicle::VehicleState) -> bool) -> StateHandle {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        if let Some(handle) = link.vehicle(VehicleId::new(1, 1))
            && done(&handle.load())
        {
            return handle;
        }
        assert!(Instant::now() < deadline, "the snapshot never got there");
        wasm_thread::sleep(Duration::from_millis(2));
    }
}

#[test]
fn a_stamping_link_puts_the_newest_frames_arrival_in_the_snapshot() {
    let (link, mut vehicle) = link(true);
    let before = Instant::now();
    vehicle.write_all(&heartbeat(0)).unwrap();
    let handle = wait_for(&link, |state| state.packet_in.is_some());
    let first = handle.load().packet_in.expect("stamped");
    assert!(first >= before, "stamped before the frame was sent");
    assert!(first <= Instant::now());

    // A later frame moves the stamp on: the snapshot carries the newest arrival, not the first.
    wasm_thread::sleep(Duration::from_millis(20));
    let resent = Instant::now();
    vehicle.write_all(&attitude(1, 0.25)).unwrap();
    let handle = wait_for(&link, |state| {
        state.packet_in.is_some_and(|stamp| stamp >= resent)
    });
    let state = handle.load();
    assert!(
        (state.attitude.roll.0 - 0.25).abs() < 1e-6,
        "the frame stamped is the frame shown"
    );
    assert!(state.packet_in.expect("stamped") > first);
}

#[test]
fn a_link_not_asked_to_stamp_leaves_the_snapshot_unstamped() {
    // Every product path: the stamp is off by default.
    assert!(!LinkConfig::default().stamp_arrivals);
    let (link, mut vehicle) = link(false);
    vehicle.write_all(&heartbeat(0)).unwrap();
    vehicle.write_all(&attitude(1, 0.25)).unwrap();
    let handle = wait_for(&link, |state| state.messages_applied >= 2);
    assert_eq!(handle.load().packet_in, None);
}
