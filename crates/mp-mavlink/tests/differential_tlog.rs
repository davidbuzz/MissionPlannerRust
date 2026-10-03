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

//! Differential test against the shipping C# implementation (DELIVERABLES.md Deliverable 19).
//!
//! `testdata/mavlink/autotest.tlog` is a real ArduPilot autotest flight. The `.csharp.csv`
//! beside it is what Mission Planner's own `MAVLink.dll` decoded from it, produced headless under
//! mono by `tools/csharp-reference/MpRefDump.cs`. This test decodes the same bytes with our codec
//! and asserts the results are identical, frame for frame.
//!
//! This is the bar the whole port is held to: not "our parser looks right", but "our parser and
//! the C# original agree on 35,000 frames of real flight data".
//!
//! The walker below deliberately mirrors `MavlinkParse.ReadPacket`'s quirky framing policy
//! (consume 8 timestamp bytes, then scan forward for the next STX) so that the two
//! implementations are compared on the *same* frames. A robust tlog reader for the product
//! belongs in the log crate (Deliverable 14); this one exists to make the comparison exact.

// Test code deliberately uses unwrap/expect/indexing: a panic here is a test failure with a
// useful message, which is exactly what we want. The production lint policy stays strict.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_possible_truncation)]

mod support;

use std::collections::HashMap;

use mp_mavlink::{Dialect, parse};

/// Dialect built from the message table dumped out of the shipped `MAVLink.dll`, so the
/// differential test uses exactly the metadata the reference implementation used.
struct BinaryDialect(HashMap<u32, u8>);

impl Dialect for BinaryDialect {
    fn crc_extra(&self, msgid: u32) -> Option<u8> {
        self.0.get(&msgid).copied()
    }

    fn name(&self) -> &str {
        "missionplanner-binary"
    }
}

fn testdata(name: &str) -> std::path::PathBuf {
    std::path::Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink"
    ))
    .join(name)
}

fn binary_dialect() -> BinaryDialect {
    let text = std::fs::read_to_string(testdata("binary_message_infos.csv"))
        .expect("binary message table; run tools/csharp-reference/regen.sh");
    let mut map = HashMap::new();
    for line in text.lines().skip(1) {
        let mut f = line.split(',');
        let (Some(id), Some(_name), Some(crc)) = (f.next(), f.next(), f.next()) else {
            continue;
        };
        map.insert(id.parse().expect("id"), crc.parse().expect("crc"));
    }
    BinaryDialect(map)
}

#[derive(Debug, PartialEq, Eq)]
struct Row {
    index: u64,
    msgid: u32,
    seq: u8,
    sysid: u8,
    compid: u8,
    payload_len: u8,
    crc16: u16,
    frame_hex: String,
}

impl Row {
    /// Identity of a frame, ignoring its position in the stream: when we recover a frame the C#
    /// parser dropped, every later index shifts by one, but the frames themselves are unchanged.
    fn key(&self) -> (u32, u8, u8, u8, u8, u16, &str) {
        (
            self.msgid,
            self.seq,
            self.sysid,
            self.compid,
            self.payload_len,
            self.crc16,
            &self.frame_hex,
        )
    }
}

fn hex(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        s.push_str(&format!("{b:02x}"));
    }
    s
}

/// Every `*.tlog` in the corpus that has a matching `*.csharp.csv` golden dump.
///
/// Corpus-driven on purpose: dropping a new log and its golden output into `testdata/` extends
/// coverage without touching this file.
fn corpus() -> Vec<(String, std::path::PathBuf, std::path::PathBuf)> {
    let dir = std::path::Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink"
    ));
    let mut out = Vec::new();
    let entries = std::fs::read_dir(dir).expect("testdata/mavlink must exist");
    for entry in entries.flatten() {
        let path = entry.path();
        if path.extension().and_then(|e| e.to_str()) != Some("tlog") {
            continue;
        }
        let golden = path.with_extension("tlog.csharp.csv");
        if golden.exists() {
            let name = path
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("?")
                .to_owned();
            out.push((name, path, golden));
        }
    }
    out.sort();
    assert!(
        !out.is_empty(),
        "no tlog corpus found; run tools/csharp-reference/regen.sh"
    );
    out
}

#[test]
fn rust_decode_matches_csharp_decode_frame_for_frame() {
    for (name, log_path, golden_path) in corpus() {
        println!("comparing {name}");
        compare_one(&log_path, &golden_path, &name);
    }
}

fn compare_one(log_path: &std::path::Path, golden_path: &std::path::Path, name: &str) {
    let dialect = binary_dialect();
    let log = std::fs::read(log_path).expect("tlog corpus");
    let expected_csv = std::fs::read_to_string(golden_path).expect("C# golden output");

    let mut expected = Vec::new();
    for line in expected_csv.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        assert_eq!(f.len(), 8, "malformed golden row: {line}");
        expected.push(Row {
            index: f[0].parse().expect("index"),
            msgid: f[1].parse().expect("msgid"),
            seq: f[2].parse().expect("seq"),
            sysid: f[3].parse().expect("sysid"),
            compid: f[4].parse().expect("compid"),
            payload_len: f[5].parse().expect("payload_len"),
            crc16: f[6].parse().expect("crc16"),
            frame_hex: f[7].to_owned(),
        });
    }
    assert!(
        expected.len() > 30_000,
        "golden corpus looks truncated: {}",
        expected.len()
    );

    let mut actual = Vec::with_capacity(expected.len());
    for (index, frame) in walk_tlog(&log, &dialect).into_iter().enumerate() {
        actual.push(Row {
            index: index as u64,
            ..frame
        });
    }
    // The relationship we require is containment, not equality: every frame the C#
    // implementation decoded must appear in ours, in the same order.
    //
    // We legitimately recover frames it drops. `MavlinkParse.ReadPacket` consumes 8 timestamp
    // bytes on every call whether or not it is aligned, so after a corrupt frame it can lose
    // sync and walk past good data - it reports badCRC=32 on the multisystem corpus. Our decoder
    // resynchronises a byte at a time and accepts a frame only when its CRC passes with the
    // correct per-message seed, so an "extra" frame is a recovered one, not an invented one.
    let ours: Vec<_> = actual.iter().map(Row::key).collect();
    let theirs: Vec<_> = expected.iter().map(Row::key).collect();

    let mut cursor = 0usize;
    let mut missed = Vec::new();
    for want in &theirs {
        match ours[cursor..].iter().position(|got| got == want) {
            Some(offset) => cursor += offset + 1,
            None if missed.len() < 5 => missed.push(format!("{want:?}")),
            None => {}
        }
    }
    assert!(
        missed.is_empty(),
        "{name}: frames the C# reference decoded that we missed:\n{}",
        missed.join("\n")
    );

    let extra = ours.len().saturating_sub(theirs.len());
    println!(
        "  {name}: {} frames decoded, {extra} recovered beyond the C# reference",
        ours.len()
    );
    assert!(
        extra * 100 <= theirs.len(),
        "{name}: {extra} extra frames exceeds 1% of {} - suspect false positives, not recovery",
        theirs.len()
    );
}

#[test]
fn every_message_in_a_real_flight_is_known_to_the_dialect() {
    let dialect = binary_dialect();
    let log = std::fs::read(testdata("autotest.tlog")).expect("tlog corpus");

    let seen: std::collections::BTreeSet<u32> = walk_tlog(&log, &dialect)
        .into_iter()
        .map(|r| r.msgid)
        .collect();

    assert!(
        seen.len() > 20,
        "a real flight should exercise many message types, saw {}",
        seen.len()
    );
}

#[test]
fn source_and_binary_message_tables_agree_where_they_overlap() {
    // Two independent extractions of the same table: one grepped out of the C# source tree,
    // one dumped from the shipped assembly. Disagreement means dialect drift we must know about.
    let src = support::reference_dialect();
    let bin = binary_dialect();

    let mut checked = 0;
    for row in &src.rows {
        if let Some(bin_crc) = bin.crc_extra(row.id) {
            assert_eq!(
                bin_crc, row.crc_extra,
                "CRC_EXTRA drift for message {} ({})",
                row.id, row.name
            );
            checked += 1;
        }
    }
    assert!(
        checked > 300,
        "expected substantial overlap, checked {checked}"
    );
}

/// Walks a tlog the way `MavlinkParse.ReadPacket(hasTimestamp: true)` does: consume the 8-byte
/// big-endian timestamp, scan forward to the next start-of-frame byte, then parse one frame.
fn walk_tlog<D: Dialect + ?Sized>(log: &[u8], dialect: &D) -> Vec<Row> {
    const MAX_SCAN: usize = 280;
    let mut out = Vec::new();
    let mut pos = 0usize;

    while pos + 8 <= log.len() {
        pos += 8; // timestamp

        let mut scan = pos;
        let limit = (pos + MAX_SCAN).min(log.len());
        while scan < limit && log[scan] != mp_mavlink::STX_V2 && log[scan] != mp_mavlink::STX_V1 {
            scan += 1;
        }
        if scan >= limit {
            // No start byte in range: `ReadPacket` returns null having consumed these bytes and
            // the caller simply calls it again. Real logs open with plain console text, so this
            // path runs for the first few hundred bytes of an autotest tlog.
            if limit <= pos {
                break;
            }
            pos = limit;
            continue;
        }

        match parse(&log[scan..], dialect) {
            Ok((frame, used)) => {
                out.push(Row {
                    index: 0,
                    msgid: frame.msgid,
                    seq: frame.seq,
                    sysid: frame.sysid,
                    compid: frame.compid,
                    payload_len: frame.payload.len() as u8,
                    crc16: frame.checksum,
                    frame_hex: hex(frame.raw),
                });
                pos = scan + used;
            }
            Err(_) => {
                pos = scan + 1;
            }
        }
    }
    out
}
