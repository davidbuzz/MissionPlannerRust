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

//! "Create Matlab file" held to Mission Planner's own `.mat` files.
//!
//! The goldens are what `MatLab.ProcessLog` wrote with csmatio under mono
//! (`tools/csharp-reference/regen-log.sh`). Every array is compared byte for byte but two things,
//! each checked on its own: the header's text, which names the platform and the time the file was
//! made; and the order of the names inside `Seen`, the last array, which the C# takes from a
//! `Hashtable` - an order of the runtime's string hashing, mono's here, not .NET Framework's, and
//! first-seen in the port. `Seen` must hold the same names in the same number of bytes.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::{Path, PathBuf};

use mp_log::convert::flight_mode_name;
use mp_log::matlab::{mat_path_for, process_log, process_log_file, write_mat};

fn testdata(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(name)
}

fn u32_at(data: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(data[at..at + 4].try_into().unwrap())
}

/// The top-level elements after the 128-byte header: where each starts and ends.
fn elements(data: &[u8]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut at = 128;
    while at < data.len() {
        let size = u32_at(data, at + 4) as usize;
        out.push((at, at + 8 + size));
        at += 8 + size;
    }
    out
}

/// Every string held by the character arrays inside one element, in order.
fn strings(element: &[u8]) -> Vec<String> {
    let mut found = Vec::new();
    let mut at = 8; // inside the outer miMATRIX
    walk(element, &mut at, element.len(), &mut found);
    found
}

fn walk(data: &[u8], at: &mut usize, end: usize, found: &mut Vec<String>) {
    while *at < end {
        let tag = u32_at(data, *at);
        let (kind, size, body, next) = if tag >> 16 != 0 {
            let size = (tag >> 16) as usize;
            (tag & 0xffff, size, *at + 4, *at + 8)
        } else {
            let size = u32_at(data, *at + 4) as usize;
            (tag, size, *at + 8, *at + 8 + size.div_ceil(8) * 8)
        };
        match kind {
            14 => {
                let mut inner = body;
                walk(data, &mut inner, body + size, found);
            }
            4 => found.push(
                data[body..body + size]
                    .chunks_exact(2)
                    .map(|c| char::from(c[0]))
                    .collect(),
            ),
            _ => {}
        }
        *at = next;
    }
}

fn assert_matches_golden(data: &[u8], golden: &str, lines: usize) {
    let converted = process_log(data, &flight_mode_name).unwrap();
    assert_eq!(converted.lines, lines, "the line count names the file");
    let ours = write_mat(&converted.arrays, "Wed, 23 Sep 2026 19:31:54 GMT");
    let theirs = std::fs::read(testdata(golden)).unwrap();

    // The header: the same words but for the time; then no subsystem data, version 1, "IM".
    let prefix = b"MATLAB 5.0 MAT-file, Platform: Unix, CREATED on: ";
    if cfg!(unix) {
        assert!(ours.starts_with(prefix));
    }
    assert!(theirs.starts_with(prefix));
    assert_eq!(ours[116..128], theirs[116..128]);

    let ours_at = elements(&ours);
    let theirs_at = elements(&theirs);
    assert_eq!(ours_at.len(), theirs_at.len(), "how many arrays");
    let (last, arrays) = ours_at.split_last().unwrap();
    for (index, (&(a, b), &(c, d))) in arrays.iter().zip(&theirs_at).enumerate() {
        assert!(
            ours[a..b] == theirs[c..d],
            "array {index} ({:?}) differs",
            strings(&theirs[c..d]).first()
        );
    }
    // Seen: the same names, in the same bytes, in another order.
    let &(c, d) = theirs_at.last().unwrap();
    assert_eq!(last.1 - last.0, d - c, "Seen's size");
    assert_eq!(
        &ours[last.0 + 40..last.0 + 48],
        b"\x01\x00\x04\x00Seen",
        "Seen's name"
    );
    let mut ours_seen = strings(&ours[last.0..last.1]);
    let mut theirs_seen = strings(&theirs[c..d]);
    ours_seen.sort();
    theirs_seen.sort();
    assert_eq!(ours_seen, theirs_seen);
}

#[test]
fn a_whole_log_matches_mission_planners_mat_file() {
    let data = std::fs::read(testdata("dataflash.bin")).unwrap();
    assert_matches_golden(
        &data,
        "dataflash/golden/matlab/dataflash.bin-11439.mat",
        11439,
    );
}

/// Read as text, it holds nothing a row is made of: an empty `PARM` and an empty `Seen`.
#[test]
fn a_log_read_as_text_writes_empty_tables() {
    let data = std::fs::read(testdata("dataflash_damaged.bin")).unwrap();
    assert_matches_golden(
        &data,
        "dataflash/golden/matlab/dataflash_damaged.bin-3047.mat",
        3047,
    );
}

/// From its first header, the damaged log's duplicated lines each make a row.
#[test]
fn a_damaged_log_matches_line_for_line() {
    let data = std::fs::read(testdata("dataflash_damaged.bin")).unwrap();
    assert_matches_golden(
        &data[18..],
        "dataflash/golden/matlab/resync/resync.bin-33073.mat",
        33073,
    );
}

/// The product path writes `<log>-<lines>.mat` beside the log.
#[test]
fn the_file_is_named_for_its_lines() {
    let dir = std::env::temp_dir().join(format!("mp-log-matlab-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let log = dir.join("flight.bin");
    std::fs::copy(testdata("dataflash.bin"), &log).unwrap();
    let path = process_log_file(&log, &flight_mode_name).unwrap();
    assert_eq!(path, mat_path_for(&log, 11439));
    assert_eq!(path, dir.join("flight.bin-11439.mat"));
    let written = std::fs::read(&path).unwrap();
    let golden =
        std::fs::read(testdata("dataflash/golden/matlab/dataflash.bin-11439.mat")).unwrap();
    assert_eq!(written.len(), golden.len());
    std::fs::remove_dir_all(&dir).unwrap();
}

/// The edge-case log: `NaN`, `Infinity` and text read as numbers, every duplicate and dropped line.
#[test]
fn every_edge_matches_line_for_line() {
    let data = std::fs::read(testdata("dataflash/edge.bin")).unwrap();
    assert_matches_golden(&data, "dataflash/golden/matlab/edge/edge.bin-339.mat", 339);
}

/// A text log: instance columns from `FMTU`, `MSG` and `ISBD` cells - the array fields as cells of
/// their own, whose empty name csmatio writes `@` - and parameters overwritten.
#[test]
fn a_text_log_matches_with_its_cells() {
    let data = std::fs::read(testdata("dataflash/synthetic.log")).unwrap();
    assert_matches_golden(
        &data,
        "dataflash/golden/matlab/synthetic/synthetic.log-63.mat",
        63,
    );
}
