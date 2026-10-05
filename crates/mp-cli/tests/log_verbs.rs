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

//! `headless-planner log <verb> <file> [out]`: the DataFlash Logs page's four buttons from the command line,
//! run as a user runs them - the built binary, on copies of the checked-in logs - and held to what
//! Mission Planner writes (`testdata/dataflash/golden`, from
//! `tools/csharp-reference/regen-log.sh`).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::{Path, PathBuf};
use std::process::Command;

fn testdata(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(name)
}

/// Bytes with every CRLF made LF, for comparing what was written here with a golden: the goldens
/// were written by Mission Planner's code under mono on Linux, where `Environment.NewLine` is LF,
/// and the port ends lines with the platform's, as the C# does, so on Windows they are CRLF (the
/// hosted runner, 2026-10-03). Both sides go through this.
fn lf(bytes: Vec<u8>) -> Vec<u8> {
    let mut out = Vec::with_capacity(bytes.len());
    let mut bytes = bytes.into_iter().peekable();
    while let Some(byte) = bytes.next() {
        if byte == b'\r' && bytes.peek() == Some(&b'\n') {
            continue;
        }
        out.push(byte);
    }
    out
}

/// A fresh directory for one test.
fn scratch(test: &str) -> PathBuf {
    let dir = mp_os::temp_dir().join(format!(
        "headless-planner-log-{test}-{}",
        mp_os::process_id()
    ));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn headless_planner(args: &[&Path]) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_headless-planner"))
        .arg("log")
        .args(args)
        // "Auto Analysis" downloads the analyzer first; through a proxy that refuses, it fails at
        // once and falls back to the runner already there, as Mission Planner does.
        .env("ALL_PROXY", "http://127.0.0.1:1")
        .env_remove("NO_PROXY")
        .env_remove("no_proxy")
        // `headless-planner` imports Mission Planner's files into its own data directory on the first start
        // that finds it empty (`mp_settings::migrate`); a test is not that start. A data
        // directory that does not exist, with no C# directory beside it, imports nothing.
        .env(
            "XDG_DATA_HOME",
            mp_os::temp_dir().join(format!("headless-planner-log-data-{}", mp_os::process_id())),
        )
        .output()
        .unwrap()
}

#[test]
fn bintolog_writes_mission_planners_text_beside_the_bin() {
    let dir = scratch("bintolog");
    let bin = dir.join("flight.bin");
    std::fs::copy(testdata("dataflash.bin"), &bin).unwrap();
    let output = headless_planner(&[Path::new("bintolog"), &bin]);
    assert!(output.status.success(), "{output:?}");
    assert_eq!(
        std::fs::read(dir.join("flight.log")).unwrap(),
        std::fs::read(testdata("dataflash/golden/dataflash.log")).unwrap()
    );
    let elsewhere = dir.join("elsewhere.txt");
    let output = headless_planner(&[Path::new("bintolog"), &bin, &elsewhere]);
    assert!(output.status.success(), "{output:?}");
    assert!(elsewhere.exists());
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn dflogtokml_writes_every_file_into_the_directory() {
    let dir = scratch("dflogtokml");
    let out = dir.join("out");
    std::fs::create_dir_all(&out).unwrap();
    let output = headless_planner(&[Path::new("dflogtokml"), &testdata("dataflash.bin"), &out]);
    assert!(output.status.success(), "{output:?}");
    for file in [
        "dataflash.bin.gpx",
        "dataflash.bin0wp.txt",
        "dataflash.bin.param",
    ] {
        assert_eq!(
            lf(std::fs::read(out.join(file)).unwrap()),
            lf(std::fs::read(testdata(&format!("dataflash/golden/kml/{file}"))).unwrap()),
            "{file}"
        );
    }
    assert!(out.join("dataflash.kmz").exists());
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn matlab_names_the_file_for_its_lines() {
    let dir = scratch("matlab");
    let log = dir.join("flight.bin");
    std::fs::copy(testdata("dataflash.bin"), &log).unwrap();
    let output = headless_planner(&[Path::new("matlab"), &log]);
    assert!(output.status.success(), "{output:?}");
    let written = std::fs::read(dir.join("flight.bin-11439.mat")).unwrap();
    let golden =
        std::fs::read(testdata("dataflash/golden/matlab/dataflash.bin-11439.mat")).unwrap();
    assert_eq!(written.len(), golden.len());
    assert!(written.starts_with(b"MATLAB 5.0 MAT-file, Platform: "));
    std::fs::remove_dir_all(&dir).unwrap();
}

/// `headless-planner log loganalysis`: the analyzer's checks on a checked-in log, the XML written
/// where `out` says, and Mission Planner's report of it printed.
#[test]
fn loganalysis_prints_the_analyzers_report() {
    let dir = scratch("loganalysis");
    let xml = dir.join("report.xml");
    let output = headless_planner(&[
        Path::new("loganalysis"),
        &testdata("dataflash/synthetic.log"),
        &xml,
    ]);
    assert!(output.status.success(), "{output:?}");
    // The report's own lines end with `Environment.NewLine` (CRLF on Windows) and the analyzer's
    // with CRLF everywhere, as Mission Planner prints them; both are read here on LF.
    let stdout = String::from_utf8(lf(output.stdout)).unwrap();
    assert!(stdout.contains("Vehicletype ArduCopter\n"), "{stdout}");
    assert!(stdout.contains("Firmware Version V4.5.7\n"), "{stdout}");
    assert!(
        stdout.contains("Test: Dupe Log Data = UNKNOWN - range() step argument must not be zero\n"),
        "{stdout}"
    );
    assert!(stdout.contains("Test: VCC = UNKNOWN - No CURR log data\n"), "{stdout}");
    let written = String::from_utf8(lf(std::fs::read(&xml).unwrap())).unwrap();
    assert!(written.starts_with("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<loganalysis>\n"));
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn a_verb_without_a_file_is_a_usage_error() {
    let output = headless_planner(&[Path::new("matlab")]);
    assert_eq!(output.status.code(), Some(2));
}

/// `headless-planner log fft`: the FFT window's title and its peaks, from the checked-in log's first IMU.
#[test]
fn fft_prints_the_windows_title_and_the_peaks() {
    let output = headless_planner(&[
        Path::new("fft"),
        &testdata("dataflash.bin"),
        Path::new("IMU[0].AccZ"),
        Path::new("128"),
        Path::new("--peaks"),
        Path::new("3"),
    ]);
    assert!(output.status.success(), "{output:?}");
    let text = String::from_utf8(output.stdout).unwrap();
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(lines[0], "FFT IMU - dataflash.bin - 22.2hz input");
    assert_eq!(
        lines[1],
        "IMU[0].AccZ: 128-point FFT, 3 slices of which 2 averaged, amplitude in dB from 5 Hz"
    );
    assert_eq!(lines.len(), 5, "{text}");
    assert!(lines[2].trim_start().starts_with("8 hz/480 rpm"), "{text}");

    // Too few samples for the size is a failure that says so; a size that is not a power of two
    // is a usage error.
    let short = headless_planner(&[
        Path::new("fft"),
        &testdata("dataflash.bin"),
        Path::new("IMU[0].AccZ"),
    ]);
    assert!(!short.status.success());
    assert!(String::from_utf8_lossy(&short.stderr).contains("455 samples"));
    let odd = headless_planner(&[
        Path::new("fft"),
        &testdata("dataflash.bin"),
        Path::new("IMU[0].AccZ"),
        Path::new("100"),
    ]);
    assert_eq!(odd.status.code(), Some(2));
}
