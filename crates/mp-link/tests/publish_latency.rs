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

//! How long a message waits to be published when it arrives on its own (the owner's question of
//! 2026-10-05: the time from fresh data's arrival to its pixels, as short as it can be). The link
//! publishes every vehicle's state at most every [`DEFAULT_PUBLISH_INTERVAL`] (5 ms), after a read
//! returns; a message that came within that interval of the last publish waited for the next read
//! to return - the next packet, or the transport's whole read timeout (100 ms) when none came. At
//! a vehicle's 4 Hz, with gaps of tens of milliseconds between its packets, that was the newest
//! state held back past several frames. Over a real socket, as a vehicle's link is.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::io::Write as _;
use std::net::TcpListener;

use web_time::{Duration, Instant};

use mp_link::testing::{attitude, heartbeat};
use mp_link::{DEFAULT_PUBLISH_INTERVAL, Link, LinkConfig};
use mp_vehicle::VehicleId;

const VEHICLE: VehicleId = VehicleId::new(1, 1);

/// Waits for the vehicle's published state to have taken `applied` messages; how long it took.
fn published(link: &Link, applied: u64) -> Duration {
    let started = Instant::now();
    let deadline = started + Duration::from_secs(10);
    loop {
        if link
            .vehicle(VEHICLE)
            .is_some_and(|handle| handle.load().messages_applied >= applied)
        {
            return started.elapsed();
        }
        assert!(Instant::now() < deadline, "never published");
        std::thread::sleep(Duration::from_micros(200));
    }
}

/// A message on its own, straight after a publish, is published within about the interval, not
/// when the read after it gives up.
#[test]
fn a_lone_message_is_published_within_the_interval() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let config = LinkConfig {
        stream_rate_hz: 0,
        send_heartbeat: false,
        record_path: None,
        ..LinkConfig::default()
    };
    let link = Link::connect(&format!("tcp:127.0.0.1:{port}"), config).unwrap();
    let (mut vehicle, _) = listener.accept().unwrap();
    vehicle.set_nodelay(true).unwrap();
    vehicle.write_all(&heartbeat(0)).unwrap();
    published(&link, 1);
    let mut seq = 1_u8;
    let mut waits = Vec::new();
    for _ in 0..8 {
        // One message, published - the publish has just happened - then another at once, inside
        // the interval since it, and nothing after.
        let applied = link.vehicle(VEHICLE).unwrap().load().messages_applied;
        vehicle.write_all(&attitude(seq, 0.1)).unwrap();
        published(&link, applied + 1);
        seq = seq.wrapping_add(1);
        vehicle.write_all(&attitude(seq, 0.2)).unwrap();
        waits.push(published(&link, applied + 2));
        seq = seq.wrapping_add(1);
        // Apart, so each pair starts after a quiet spell, as a vehicle's bursts do.
        std::thread::sleep(Duration::from_millis(30));
    }
    waits.sort_unstable();
    let median = waits[waits.len() / 2];
    // The interval and a little for the machine; the read timeout it waited before is 100 ms.
    assert!(
        median < DEFAULT_PUBLISH_INTERVAL * 4,
        "a lone message waited {median:?} to be published (every wait: {waits:?})"
    );
}
