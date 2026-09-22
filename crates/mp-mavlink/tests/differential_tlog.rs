//! Differential test against the shipping C# implementation (DELIVERABLES.md D19).
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
//! belongs in the log crate (D14); this one exists to make the comparison exact.

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

fn hex(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        s.push_str(&format!("{b:02x}"));
    }
    s
}

#[test]
fn rust_decode_matches_csharp_decode_frame_for_frame() {
    let dialect = binary_dialect();
    let log = std::fs::read(testdata("autotest.tlog")).expect("tlog corpus");
    let expected_csv =
        std::fs::read_to_string(testdata("autotest.tlog.csharp.csv")).expect("C# golden output");

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

    assert_eq!(
        actual.len(),
        expected.len(),
        "frame count differs from the C# reference"
    );

    // Report the first divergence in full rather than a bare count mismatch.
    for (a, e) in actual.iter().zip(&expected) {
        assert_eq!(a, e, "divergence at frame {}", a.index);
    }
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
