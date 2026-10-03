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

//! Verifies the generated wire layout against real flight data.
//!
//! A field-offset mistake in generated code is invisible to the compiler and catastrophic in the
//! air. These tests decode every frame of a real ArduPilot flight into typed structs, re-encode
//! them, and require the bytes back exactly - which can only happen if every offset, size and
//! endianness in the generated layout is right.

// Test code deliberately uses unwrap/expect/indexing.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_possible_truncation)]

use std::collections::{BTreeMap, BTreeSet};

use mp_mavlink::{Dialect as _, Message as _, parse};
use mp_mavlink_dialects::all::{DIALECT, Heartbeat, MavMessage};

fn tlog() -> Vec<u8> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink/autotest.tlog"
    );
    std::fs::read(path).expect("tlog corpus")
}

/// Walks the tlog the way `MavlinkParse.ReadPacket(hasTimestamp: true)` does.
fn walk(log: &[u8], mut on_frame: impl FnMut(u32, &[u8])) {
    const MAX_SCAN: usize = 280;
    let mut pos = 0usize;
    while pos + 8 <= log.len() {
        pos += 8;
        let mut scan = pos;
        let limit = (pos + MAX_SCAN).min(log.len());
        while scan < limit && log[scan] != mp_mavlink::STX_V2 && log[scan] != mp_mavlink::STX_V1 {
            scan += 1;
        }
        if scan >= limit {
            if limit <= pos {
                break;
            }
            pos = limit;
            continue;
        }
        match parse(&log[scan..], &DIALECT) {
            Ok((frame, used)) => {
                on_frame(frame.msgid, frame.payload);
                pos = scan + used;
            }
            Err(_) => pos = scan + 1,
        }
    }
}

#[test]
fn every_frame_in_a_real_flight_survives_decode_and_re_encode() {
    let log = tlog();
    let mut checked = 0u64;
    let mut per_type: BTreeMap<u32, u64> = BTreeMap::new();
    let mut failures = Vec::new();

    walk(&log, |msgid, payload| {
        let Some(info) = DIALECT.info(msgid) else {
            return;
        };
        let Some(msg) = MavMessage::decode(msgid, payload) else {
            return;
        };

        let full_len = usize::from(info.len);
        let mut reencoded = vec![0u8; full_len];
        msg.encode(&mut reencoded);

        // The wire payload may be truncated; zero-extend it for comparison.
        let mut expected = vec![0u8; full_len];
        let n = payload.len().min(full_len);
        expected[..n].copy_from_slice(&payload[..n]);

        if reencoded != expected && failures.len() < 5 {
            failures.push(format!(
                "{} (id {msgid}): re-encoded {:02x?} != original {:02x?}",
                info.name, reencoded, expected
            ));
        }
        *per_type.entry(msgid).or_default() += 1;
        checked += 1;
    });

    assert!(
        failures.is_empty(),
        "layout mismatches:\n{}",
        failures.join("\n")
    );
    assert!(
        checked > 30_000,
        "expected a large corpus, checked {checked}"
    );
    assert!(
        per_type.len() > 20,
        "expected many message types, saw {}",
        per_type.len()
    );
    println!(
        "verified {checked} frames across {} message types",
        per_type.len()
    );
}

#[test]
fn heartbeat_decodes_to_sensible_values() {
    let log = tlog();
    let mut seen = BTreeSet::new();
    walk(&log, |msgid, payload| {
        if msgid == Heartbeat::ID {
            let hb = Heartbeat::decode(payload);
            seen.insert((hb.r#type, hb.autopilot, hb.mavlink_version));
        }
    });
    assert!(!seen.is_empty(), "a flight log must contain heartbeats");
    for (mav_type, autopilot, version) in &seen {
        assert_eq!(
            *version, 3,
            "ArduPilot speaks MAVLink 2 with version field 3"
        );
        assert!(*autopilot <= 20, "implausible autopilot id {autopilot}");
        assert!(*mav_type <= 45, "implausible vehicle type {mav_type}");
    }
}

#[test]
fn generated_metadata_matches_the_dialect_table() {
    // The struct constants and the lookup table are generated from the same source, but from
    // different code paths; a mismatch means the emitter is inconsistent with itself.
    assert_eq!(
        Heartbeat::CRC_EXTRA,
        DIALECT.crc_extra(Heartbeat::ID).unwrap()
    );
    assert_eq!(
        Heartbeat::LEN,
        usize::from(DIALECT.info(Heartbeat::ID).unwrap().len)
    );
    assert_eq!(
        Heartbeat::MIN_LEN,
        usize::from(DIALECT.info(Heartbeat::ID).unwrap().min_len)
    );
}

#[test]
fn message_table_is_sorted_and_unique() {
    // Binary search in StaticDialect depends on this.
    let mut last = None;
    for info in mp_mavlink_dialects::MESSAGES {
        if let Some(prev) = last {
            assert!(
                info.id > prev,
                "table must be strictly sorted by id: {prev} then {}",
                info.id
            );
        }
        last = Some(info.id);
    }
}
