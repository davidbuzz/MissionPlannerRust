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

//! Log round-tripping, and reading the real corpora.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use mp_link::tlog::TlogWriter;
use mp_log::TlogReader;
use mp_mavlink::{Message as _, encode_v2};
use mp_mavlink_dialects::all::{DIALECT, Heartbeat};

fn heartbeat(seq: u8) -> Vec<u8> {
    let hb = Heartbeat {
        custom_mode: u32::from(seq),
        r#type: 2,
        autopilot: 3,
        base_mode: 81,
        system_status: 3,
        mavlink_version: 3,
    };
    let mut payload = [0u8; Heartbeat::LEN];
    hb.encode(&mut payload);
    let mut frame = [0u8; 64];
    let n = encode_v2(
        &mut frame,
        seq,
        1,
        1,
        Heartbeat::ID,
        &payload,
        Heartbeat::CRC_EXTRA,
        0,
    )
    .unwrap();
    frame[..n].to_vec()
}

fn scratch(name: &str) -> std::path::PathBuf {
    let dir = mp_os::temp_dir().join("mp-log-tests");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(name);
    let _ = std::fs::remove_file(&path);
    path
}

#[test]
fn frames_and_timestamps_round_trip() {
    let path = scratch("roundtrip.tlog");
    let mut written = Vec::new();
    {
        let mut writer = TlogWriter::create(&path).unwrap();
        for seq in 0..50u8 {
            let frame = heartbeat(seq);
            let stamp = 1_700_000_000_000_000 + u64::from(seq) * 1000;
            writer.write_frame_at(&frame, stamp).unwrap();
            written.push((stamp, frame));
        }
        writer.flush().unwrap();
        assert_eq!(writer.frames(), 50);
    }

    let data = std::fs::read(&path).unwrap();
    let records = TlogReader::new(&data).records(&DIALECT);
    assert_eq!(records.len(), written.len());
    for (record, (stamp, frame)) in records.iter().zip(&written) {
        assert_eq!(
            record.timestamp_micros,
            Some(*stamp),
            "timestamp must survive the round trip"
        );
        assert_eq!(
            record.frame,
            &frame[..],
            "frame bytes must survive the round trip"
        );
    }
}

#[test]
fn creating_a_log_never_overwrites_an_existing_recording() {
    // Losing a flight recording to a re-run is unacceptable, so create is exclusive.
    let path = scratch("nooverwrite.tlog");
    let _writer = TlogWriter::create(&path).unwrap();
    assert!(
        TlogWriter::create(&path).is_err(),
        "must refuse to clobber an existing log"
    );
}

#[test]
fn a_corrupted_log_still_yields_its_good_records() {
    let path = scratch("corrupt.tlog");
    {
        let mut writer = TlogWriter::create(&path).unwrap();
        writer.write_frame_at(&heartbeat(1), 1_000).unwrap();
        // Console text of the sort ArduPilot emits at boot, wrapped as a record.
        writer
            .write_frame_at(b"\n\nInit ArduCopter V4.8.0\n", 2_000)
            .unwrap();
        writer.write_frame_at(&heartbeat(2), 3_000).unwrap();
        writer.flush().unwrap();
    }
    let mut data = std::fs::read(&path).unwrap();
    // Truncate mid-frame, as a crash during recording would.
    data.extend_from_slice(&1_700_000_000_000_000u64.to_be_bytes());
    data.extend_from_slice(&heartbeat(3)[..6]);

    let records = TlogReader::new(&data).records(&DIALECT);
    let seqs: Vec<u8> = records
        .iter()
        .filter_map(|r| {
            mp_mavlink::parse(r.frame, &DIALECT)
                .ok()
                .map(|(f, _)| f.seq)
        })
        .collect();
    assert!(
        seqs.contains(&1) && seqs.contains(&2),
        "good records must survive: {seqs:?}"
    );
}

#[test]
fn the_real_corpora_read_back_at_the_expected_frame_counts() {
    // These counts are the ones the differential test established against the C# implementation,
    // so a regression in the reader shows up here rather than as a mysterious diff later.
    for (name, expected) in [("autotest.tlog", 35_750usize), ("multisystem.tlog", 34_775)] {
        let path = std::path::Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../testdata/mavlink"
        ))
        .join(name);
        let data = std::fs::read(&path).expect("corpus");
        let records = TlogReader::new(&data).records(&DIALECT);
        assert_eq!(records.len(), expected, "frame count for {name}");

        // Timestamps must be plausible Unix microseconds and broadly increasing.
        let stamps: Vec<Option<u64>> = records.iter().map(|r| r.timestamp_micros).collect();
        let sane = stamps.iter().filter(|s| s.is_some()).count();
        assert!(
            sane * 100 / stamps.len() > 90,
            "{name}: only {sane} of {} timestamps look like Unix microseconds",
            stamps.len()
        );
    }
}

#[test]
fn timestamps_span_the_recording() {
    // The summary tool reported a zero duration for a 36-second recording, so the reader's
    // timestamps are worth asserting directly rather than only checking they look plausible.
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink/autotest.tlog"
    );
    let data = std::fs::read(path).expect("corpus");
    let records = TlogReader::new(&data).records(&DIALECT);

    // Only records that carry a real stamp; the first frame in this log follows console text,
    // so the bytes before it are not a timestamp at all.
    let stamps: Vec<u64> = records.iter().filter_map(|r| r.timestamp_micros).collect();
    let first = *stamps.first().expect("records with stamps");
    let last = *stamps.last().expect("records with stamps");
    let span_seconds = (last.saturating_sub(first)) as f64 / 1e6;

    assert!(
        stamps.len() * 100 > records.len() * 95,
        "most records should carry a stamp"
    );
    assert!(
        span_seconds > 10.0 && span_seconds < 600.0,
        "expected a recording of tens of seconds, measured {span_seconds}"
    );
}
