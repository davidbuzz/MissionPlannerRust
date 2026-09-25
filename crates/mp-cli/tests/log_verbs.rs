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

/// A fresh directory for one test.
fn scratch(test: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("headless-planner-log-{test}-{}", std::process::id()));
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
            std::env::temp_dir().join(format!("headless-planner-log-data-{}", std::process::id())),
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
            std::fs::read(out.join(file)).unwrap(),
            std::fs::read(testdata(&format!("dataflash/golden/kml/{file}"))).unwrap(),
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

/// The analyzer is a Windows program; here a script stands in for it, writing the analyzer's
/// example output where it is told to, and the report printed is Mission Planner's text of it.
#[cfg(unix)]
#[test]
fn loganalysis_prints_the_analyzers_report() {
    use std::os::unix::fs::PermissionsExt;

    let dir = scratch("loganalysis");
    let analyzer = dir.join("LogAnalyzer");
    std::fs::create_dir_all(&analyzer).unwrap();
    let runner = analyzer.join("runner.exe");
    std::fs::write(
        &runner,
        format!(
            "#!/bin/sh\ncp '{}' \"$2\"\n",
            testdata("dataflash/example_output.xml").display()
        ),
    )
    .unwrap();
    std::fs::set_permissions(&runner, std::fs::Permissions::from_mode(0o755)).unwrap();
    let log = dir.join("flight.log");
    std::fs::write(
        &log,
        "FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\n",
    )
    .unwrap();

    let output = headless_planner(&[Path::new("loganalysis"), &log, &analyzer]);
    assert!(output.status.success(), "{output:?}");
    let golden =
        std::fs::read_to_string(testdata("dataflash/golden/loganalysis/example_output.txt"))
            .unwrap();
    let stdout = String::from_utf8(output.stdout).unwrap();
    assert!(stdout.ends_with(&golden), "{stdout}");
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
