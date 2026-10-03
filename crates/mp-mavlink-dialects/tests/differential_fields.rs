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

//! Field-level differential test against the shipping C# implementation.
//!
//! The byte round-trip test in `layout.rs` proves our wire layout is right. It cannot prove our
//! *names* are right: a field at the correct offset with the wrong name, or an `i16` we decoded
//! as `u16`, round-trips perfectly and is still wrong everywhere it is displayed.
//!
//! So this compares, for every frame in the corpus, every field by name against what Mission
//! Planner's own `MAVLink.dll` decoded from the same bytes - 24,000+ values produced by
//! reflecting over the C# structs in `tools/csharp-reference/MpRefDump.cs`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::collections::BTreeMap;

use mp_mavlink::{FieldValue, parse};
use mp_mavlink_dialects::all::{DIALECT, MavMessage};

fn testdata(name: &str) -> std::path::PathBuf {
    std::path::Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink"
    ))
    .join(name)
}

/// Walks the tlog exactly as `MavlinkParse.ReadPacket(hasTimestamp: true)` does.
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

/// Compares one field value against the C# rendering.
///
/// Floats need care: C# prints an `f32` round-tripped as `0.1`, while widening that same `f32`
/// to `f64` and printing it gives `0.10000000149011612`. Both are the same bits; only a numeric
/// comparison at `f32` precision can see that.
fn values_match(ours: &FieldValue, theirs: &str) -> bool {
    let our_str = ours.to_compare_string();
    if our_str == theirs {
        return true;
    }

    let our_parts: Vec<&str> = our_str.split_whitespace().collect();
    let their_parts: Vec<&str> = theirs.split_whitespace().collect();
    if our_parts.len() != their_parts.len() {
        return false;
    }

    our_parts.iter().zip(&their_parts).all(|(a, b)| {
        let (Ok(a), Ok(b)) = (a.parse::<f64>(), b.parse::<f64>()) else {
            return a == b;
        };
        if a.is_nan() && b.is_nan() {
            return true;
        }
        #[allow(clippy::cast_possible_truncation)] // deliberate: compare at f32 precision
        let (a32, b32) = (a as f32, b as f32);
        a == b || a32 == b32
    })
}

#[test]
fn every_decoded_field_matches_the_csharp_implementation() {
    let log = std::fs::read(testdata("autotest.tlog")).expect("tlog corpus");
    let csv = std::fs::read_to_string(testdata("autotest.fields.csharp.csv"))
        .expect("field corpus; run tools/csharp-reference/regen.sh");

    // index -> (msgname, field -> value)
    let mut expected: BTreeMap<u64, (String, BTreeMap<String, String>)> = BTreeMap::new();
    for line in csv.lines().skip(1) {
        // The value may itself contain commas only if it were a string; MAVLink has none.
        let parts: Vec<&str> = line.splitn(5, ',').collect();
        let [index, _msgid, msgname, field, value] = parts[..] else {
            continue;
        };
        let entry = expected
            .entry(index.parse().expect("index"))
            .or_insert_with(|| (msgname.to_owned(), BTreeMap::new()));
        entry.1.insert(field.to_owned(), value.to_owned());
    }
    assert!(
        expected.len() > 2_000,
        "field corpus looks truncated: {}",
        expected.len()
    );

    let mut index = 0u64;
    let mut compared = 0u64;
    let mut messages_compared = 0u64;
    let mut mismatches: Vec<String> = Vec::new();
    let mut names_missing: Vec<String> = Vec::new();

    walk(&log, |msgid, payload| {
        let current = index;
        index += 1;

        let Some((msgname, fields)) = expected.get(&current) else {
            return;
        };
        let Some(msg) = MavMessage::decode(msgid, payload) else {
            return;
        };
        assert_eq!(
            msg.name(),
            msgname,
            "message name differs at frame {current}"
        );
        messages_compared += 1;

        for (name, value) in msg.fields() {
            let Some(theirs) = fields.get(name) else {
                if names_missing.len() < 10 {
                    names_missing.push(format!("{msgname}.{name} absent from the C# decode"));
                }
                continue;
            };
            compared += 1;
            if !values_match(&value, theirs) && mismatches.len() < 15 {
                mismatches.push(format!(
                    "frame {current} {msgname}.{name}: ours {:?} vs C# {theirs:?}",
                    value.to_compare_string()
                ));
            }
        }
    });

    println!("compared {compared} field values across {messages_compared} messages");
    assert!(
        names_missing.is_empty(),
        "field name mismatches:\n{}",
        names_missing.join("\n")
    );
    assert!(
        mismatches.is_empty(),
        "field value mismatches:\n{}",
        mismatches.join("\n")
    );
    assert!(
        compared > 20_000,
        "expected a large comparison, did {compared}"
    );
}
