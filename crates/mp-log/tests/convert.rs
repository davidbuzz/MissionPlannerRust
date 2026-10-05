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

//! "Convert .Bin to .Log" held to Mission Planner's own output, byte for byte.
//!
//! The goldens are what `BinaryLog.ConvertBin` wrote for the checked-in logs, run headless under
//! mono by `tools/csharp-reference/regen-log.sh` with `onFlightMode` wired as `MainV2` wires it.
//! Mono formats floats and doubles as .NET Framework does (the general format, 7 and 15 digits);
//! every line of both files is compared, and none differs.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::{Path, PathBuf};

use mp_log::convert::{convert_bin, convert_bin_file, flight_mode_name, log_path_for};

fn testdata(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(name)
}

/// The first line that differs, for a failure message a person can read.
fn first_difference(ours: &[u8], theirs: &[u8]) -> String {
    let ours: Vec<&[u8]> = ours.split(|&b| b == b'\n').collect();
    let theirs: Vec<&[u8]> = theirs.split(|&b| b == b'\n').collect();
    for (line, (a, b)) in ours.iter().zip(&theirs).enumerate() {
        if a != b {
            return format!(
                "line {}:\n  ours:   {}\n  theirs: {}",
                line + 1,
                String::from_utf8_lossy(a),
                String::from_utf8_lossy(b)
            );
        }
    }
    format!("{} lines against {}", ours.len(), theirs.len())
}

fn assert_converts_as_mission_planner_does(log: &str, golden: &str) {
    let data = std::fs::read(testdata(log)).unwrap();
    let expected = std::fs::read(testdata(golden)).unwrap();
    let ours = convert_bin(&data, &flight_mode_name);
    assert!(
        ours == expected,
        "{log}: {}",
        first_difference(&ours, &expected)
    );
}

#[test]
fn a_whole_log_converts_byte_for_byte() {
    assert_converts_as_mission_planner_does("dataflash.bin", "dataflash/golden/dataflash.log");
}

/// The damaged log opens with garbage, carries headers of types it never declares, and messages
/// cut short: every one of `ReadMessage`'s ways of skipping is taken, thousands of times.
#[test]
fn a_damaged_log_converts_byte_for_byte() {
    assert_converts_as_mission_planner_does(
        "dataflash_damaged.bin",
        "dataflash/golden/dataflash_damaged.log",
    );
}

/// The flight mode is named from the firmware the log names, which `ArduCopter` in a `MSG` does
/// before the `MODE` record - so the golden says `Loiter`, not `5`.
#[test]
fn the_mode_is_named_for_the_firmware() {
    let golden =
        std::fs::read_to_string(testdata("dataflash/golden/dataflash_damaged.log")).unwrap();
    assert!(golden.contains("\r\nMODE, 80421985, Loiter, 5, 2\r\n"));
}

/// The button writes `<name>.log` beside the `.bin`, and that is the file the product path reads.
#[test]
fn the_file_is_written_beside_the_log() {
    let dir = mp_os::temp_dir().join(format!("mp-log-convert-{}", mp_os::process_id()));
    std::fs::create_dir_all(&dir).unwrap();
    let bin = dir.join("00000001.BIN");
    std::fs::copy(testdata("dataflash.bin"), &bin).unwrap();
    let out = log_path_for(&bin);
    assert_eq!(out, dir.join("00000001.log"));
    convert_bin_file(&bin, &out, &flight_mode_name).unwrap();
    assert_eq!(
        std::fs::read(&out).unwrap(),
        std::fs::read(testdata("dataflash/golden/dataflash.log")).unwrap()
    );
    std::fs::remove_dir_all(&dir).unwrap();
}

/// The harness's own edge-case log (`MpLog.cs` `MkEdge`): every field type at its extremes, three
/// hundred arbitrary float and double bit patterns, NULs, control and high bytes in strings, a
/// format with an unknown character, one that runs past its body and one too short to have one, the
/// header the scan misses, a message before its format, a type declared twice, a mode named for a
/// plane and then a copter, and an array cut off by the end of the file.
#[test]
fn every_edge_converts_byte_for_byte() {
    assert_converts_as_mission_planner_does("dataflash/edge.bin", "dataflash/golden/edge.log");
}
