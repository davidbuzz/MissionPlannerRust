//! `headless-planner magcal <log>`: `MagCalib.ProcessLog` run as a user runs it - the built binary on the
//! checked-in logs - printing the C#'s console lines and its box.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::{Path, PathBuf};
use std::process::Command;

fn testdata(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(name)
}

fn headless_planner(args: &[&str]) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_headless-planner"))
        .arg("magcal")
        .args(args)
        // A data directory that does not exist imports nothing (`mp_settings::migrate`).
        .env(
            "XDG_DATA_HOME",
            std::env::temp_dir().join(format!("headless-planner-magcal-data-{}", std::process::id())),
        )
        .output()
        .unwrap()
}

#[test]
fn a_dataflash_log_gives_the_offsets_box() {
    let log = testdata("dataflash.bin");
    let output = headless_planner(&[log.to_str().unwrap()]);
    assert!(output.status.success(), "{output:?}");
    // The fit is `mp-calibration`'s golden (`tests/magcal_vectors.rs`): offsets -292.5 122.1 552.4.
    assert_eq!(
        String::from_utf8(output.stdout).unwrap(),
        "New Mag Offsets\nNew offsets for compass #1 are -293 122 552\n\n\
         Please write these down for manual entry\n"
    );
}

#[test]
fn ellipsoid_prints_the_fits_log_lines_first() {
    let log = testdata("dataflash.bin");
    let output = headless_planner(&[log.to_str().unwrap(), "--ellipsoid"]);
    assert!(output.status.success(), "{output:?}");
    let text = String::from_utf8(output.stdout).unwrap();
    let lines: Vec<&str> = text.lines().collect();
    assert!(lines[0].starts_with("magcal 1 ofs -292.54"), "{text}");
    // The offsets of the last `MAG` line, which is the second compass's.
    assert!(lines[0].ends_with(" old ofs -123,148,-16"), "{text}");
    assert!(lines[1].starts_with("magcalel 1 ofs -292.54"), "{text}");
    assert_eq!(lines[2], "New Mag Offsets");
}

#[test]
fn a_still_telemetry_log_is_not_enough_data() {
    let log = testdata("mavlink/autotest.tlog");
    let output = headless_planner(&[log.to_str().unwrap()]);
    assert!(!output.status.success());
    assert_eq!(
        String::from_utf8(output.stdout).unwrap(),
        "Extracted 6 data points\nCurrent offset: (0, 0, 0)\nLog does not contain enough data\n"
    );
}

#[test]
fn a_missing_throttle_value_is_a_usage_error() {
    let output = headless_planner(&["x.tlog", "--min-throttle"]);
    assert_eq!(output.status.code(), Some(2));
}
