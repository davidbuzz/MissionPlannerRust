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

//! `headless-planner magcal <log> [--ellipsoid] [--min-throttle N]`: `MagCalib.ProcessLog` without its file
//! dialog - the Temp screen's `BUT_magfit2`, and the older compass page's
//! `BUT_MagCalibrationLog_Click`, which its Designer wires to no button (PLAN.md §12 D16).
//!
//! A file whose name ends in `tlog` is read as `getOffsets` reads it: every `RAW_IMU` taken while
//! the throttle is at least `--min-throttle` percent (the page asks "Min Throttle", offering 30;
//! `ProcessLog`'s own default, and this verb's, is 0), with the last `SENSOR_OFFSETS` taken back
//! off. Anything else is a dataflash log read as `getOffsetsLog` reads it, through `DFLogBuffer`:
//! every `MAG`, `MAG2` and `MAG3` line. The offsets are fitted by `mp_calibration::magcalib` and
//! printed as the C# prints them - its console lines, then its box. Nothing is connected, so the
//! box is the one that says to write them down.
//!
//! `--ellipsoid` also prints the C#'s log lines for its fits (`magcal`, `magcalel`), which is where
//! the ellipsoid it fits is to be seen: `ProcessLog` hands only the offsets on.
//!
//! The reading and the fit are `mp_log::magcal_log`'s, which the planner's EXPERIMENTAL
//! "mag calb log" shares. This verb writes no `magoffset.dxf`: `doDXF`'s drawing goes into the
//! planner's data directory when the planner reads a log, and a command line asked for offsets.
//! `// C#: MagCalib.cs:93-133, 813-1117`

#![allow(clippy::print_stdout, clippy::print_stderr)]

use std::fmt::Write as _;
use std::process::ExitCode;

use mp_log::magcal_log::{Gathered, compass_one, gather};
use mp_calibration::magcalib::{self, DataflashSamples, LogFit, TITLE, fit_dataflash, fit_tlog};

/// The usage line.
pub(crate) const USAGE: &str = "headless-planner magcal <log> [--ellipsoid] [--min-throttle N]";

/// A list of values as `{0},{1},{2}` writes them.
fn commas(values: &[f64]) -> String {
    values
        .iter()
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join(",")
}

/// A value of an answer, 0 past its end.
fn at(x: &[f64], i: usize) -> f64 {
    x.get(i).copied().unwrap_or(0.0)
}

/// `magcalel`'s nine values: `ofs a,b,c di d,e,f di g h i rad r`. `// C#: MagCalib.cs:1100`
fn ellipsoid_line(label: &str, x: &[f64], rad: f64) -> String {
    format!(
        "{label} ofs {} di {} di {} {} {} rad {rad}",
        commas(&[at(x, 0), at(x, 1), at(x, 2)]),
        commas(&[at(x, 3), at(x, 4), at(x, 5)]),
        at(x, 6),
        at(x, 7),
        at(x, 8),
    )
}

/// What `headless-planner magcal` prints for what was gathered, and whether it ended in a box of offsets.
pub(crate) fn report(gathered: &Gathered, ellipsoid: bool) -> (String, bool) {
    let mut out = String::new();
    let fit = match gathered {
        Gathered::Tlog(samples) => {
            // `// C#: MagCalib.cs:1062-1063`
            let [x, y, z] = samples.offset();
            let _ = writeln!(out, "Extracted {} data points", samples.data.len());
            let _ = writeln!(out, "Current offset: ({x}, {y}, {z})");
            match fit_tlog(samples.data.clone()) {
                Ok(fit) => {
                    // `// C#: MagCalib.cs:1092`
                    let [a, b, c] = samples.old_method();
                    let _ = writeln!(out, "Old Method {a} {b} {c}");
                    if ellipsoid {
                        tlog_log_lines(&mut out, &fit);
                    }
                    fit
                }
                Err(error) => {
                    let _ = writeln!(out, "{error}");
                    return (out, false);
                }
            }
        }
        Gathered::Dataflash(samples) => match fit_dataflash(compass_one(gathered)) {
            Ok(fit) => {
                if ellipsoid {
                    dataflash_log_lines(&mut out, samples);
                }
                fit
            }
            Err(error) => {
                // `ProcessLog` catches alglib's exception and shows nothing (`:127-130`).
                let _ = writeln!(out, "{error}: Mission Planner shows nothing");
                return (out, false);
            }
        },
    };
    // `SaveOffsets` with no link: the box that says to write them down. `// C#: MagCalib.cs:1319-1324`
    let _ = writeln!(out, "{TITLE}");
    let _ = writeln!(out, "{}", magcalib::manual_message(1, &fit.offsets));
    (out, true)
}

/// `getOffsets`' log lines. `// C#: MagCalib.cs:1096, 1100, 1104`
fn tlog_log_lines(out: &mut String, fit: &LogFit) {
    let x = &fit.sphere.x;
    let _ = writeln!(
        out,
        "magcal 1 ofs {} strength {} ",
        commas(&[at(x, 0), at(x, 1), at(x, 2)]),
        at(x, 3)
    );
    let rad = fit.ellipsoid.rad;
    let _ = writeln!(
        out,
        "{}",
        ellipsoid_line("magcalel 1", &fit.ellipsoid.x, rad)
    );
    if let Some(refit) = &fit.refit {
        let _ = writeln!(out, "{}", ellipsoid_line("magcalel 2", &refit.x, rad));
    }
}

/// `getOffsetsLog`'s log lines, for each compass with samples: the sphere, then the ellipsoid,
/// each with the offsets its last line carried. `// C#: MagCalib.cs:901-921`
fn dataflash_log_lines(out: &mut String, samples: &DataflashSamples) {
    for (index, (data, old)) in samples.data.iter().zip(&samples.old_offsets).enumerate() {
        if data.is_empty() {
            continue;
        }
        let n = index + 1;
        let old = commas(old);
        for (label, ellipsoid) in [("magcal", false), ("magcalel", true)] {
            if let Ok(fit) = magcalib::least_sq(data, ellipsoid) {
                let x = &fit.x;
                let _ = writeln!(
                    out,
                    "{label} {n} ofs {} strength {} old ofs {old}",
                    commas(&[at(x, 0), at(x, 1), at(x, 2)]),
                    at(x, 3)
                );
            }
        }
    }
}

/// `headless-planner magcal`.
pub(crate) fn run(args: &[String]) -> ExitCode {
    let mut path = None;
    let mut ellipsoid = false;
    let mut throttle = 0;
    let mut rest = args.iter();
    while let Some(arg) = rest.next() {
        match arg.as_str() {
            "--ellipsoid" => ellipsoid = true,
            "--min-throttle" => match rest.next().map(|value| value.parse::<i32>()) {
                Some(Ok(value)) => throttle = value,
                _ => {
                    eprintln!("usage: {USAGE}");
                    return ExitCode::from(2);
                }
            },
            _ if path.is_none() => path = Some(arg.as_str()),
            _ => {
                eprintln!("usage: {USAGE}");
                return ExitCode::from(2);
            }
        }
    }
    let Some(path) = path else {
        eprintln!("usage: {USAGE}");
        return ExitCode::from(2);
    };
    let data = match mp_os::fs::read(path) {
        Ok(data) => data,
        Err(error) => {
            // The C#'s "Log Can not be opened. Are you still connected?" is the tlog path's
            // (`:977-983`); this verb says why instead.
            eprintln!("could not read {path}: {error}");
            return ExitCode::FAILURE;
        }
    };
    let gathered = match gather(path, &data, throttle) {
        Ok(gathered) => gathered,
        Err(error) => {
            eprintln!("{path}: {error}: Mission Planner shows nothing");
            return ExitCode::FAILURE;
        }
    };
    let (text, fitted) = report(&gathered, ellipsoid);
    print!("{text}");
    if fitted {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use mp_calibration::CalibrationError;
    use std::path::{Path, PathBuf};

    const fn not_enough(samples: usize) -> CalibrationError {
        CalibrationError::NotEnoughData { samples }
    }

    fn testdata() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata")
    }

    /// The logs under `testdata` the fixtures are drawn from, and the fixture each becomes.
    const LOGS: [(&str, &str); 8] = [
        ("dataflash.bin", "dataflash"),
        ("dataflash_damaged.bin", "dataflash_damaged"),
        ("dataflash/edge.bin", "edge"),
        ("georef/camera.bin", "camera-bin"),
        ("georef/camera.tlog", "camera-tlog"),
        ("currentstate/synthetic.tlog", "synthetic"),
        ("mavlink/autotest.tlog", "autotest"),
        ("mavlink/multisystem.tlog", "multisystem"),
    ];

    /// A fixture's text: where the samples came from, then one sample a line.
    fn fixture(log: &str, gathered: &Gathered) -> String {
        let mut text = String::new();
        let path = match gathered {
            Gathered::Tlog(_) => "tlog",
            Gathered::Dataflash(_) => "dataflash",
        };
        let _ = writeln!(
            text,
            "# MagCalib.cs samples of compass 1, as `headless-planner magcal` gathers them"
        );
        let _ = writeln!(
            text,
            "# regenerate: cargo test -p mp-cli -- --ignored regenerate_magcal_fixtures"
        );
        let _ = writeln!(text, "# log: testdata/{log}");
        let _ = writeln!(text, "# path: {path}");
        let _ = writeln!(text, "# min-throttle: 0");
        for [x, y, z] in compass_one(gathered) {
            let _ = writeln!(text, "{x} {y} {z}");
        }
        text
    }

    /// Writes `testdata/magcal/<name>.txt` for every log above with any compass 1 samples, so
    /// `mp-calibration`'s golden tests read a few kilobytes rather than parse the logs each run.
    #[test]
    #[ignore = "regenerates testdata/magcal; run by hand when the extraction changes"]
    fn regenerate_magcal_fixtures() {
        let out = testdata().join("magcal");
        mp_os::fs::create_dir_all(&out).unwrap();
        for (log, name) in LOGS {
            let data = mp_os::fs::read(testdata().join(log)).unwrap();
            let Ok(gathered) = gather(log, &data, 0) else {
                println!("{log}: unreadable");
                continue;
            };
            let samples = compass_one(&gathered).len();
            println!("{log}: {samples} samples");
            if samples > 0 {
                mp_os::fs::write(out.join(format!("{name}.txt")), fixture(log, &gathered)).unwrap();
            }
        }
    }

    /// Each fixture is still what the extraction makes of its log.
    #[test]
    fn the_fixtures_are_what_the_logs_give() {
        for (log, name) in LOGS {
            let path = testdata().join("magcal").join(format!("{name}.txt"));
            let Ok(expected) = mp_os::fs::read_to_string(&path) else {
                continue;
            };
            let data = mp_os::fs::read(testdata().join(log)).unwrap();
            let gathered = gather(log, &data, 0).unwrap();
            assert_eq!(fixture(log, &gathered), expected, "{log}");
        }
    }

    #[test]
    fn a_still_vehicle_is_not_enough_data() {
        let data = mp_os::fs::read(testdata().join("mavlink/autotest.tlog")).unwrap();
        let gathered = gather("autotest.tlog", &data, 0).unwrap();
        let (text, fitted) = report(&gathered, false);
        assert!(!fitted);
        assert_eq!(
            text,
            format!(
                "Extracted 6 data points\nCurrent offset: (0, 0, 0)\n{}\n",
                not_enough(6)
            )
        );
    }

    #[test]
    fn a_throttle_never_reached_gathers_nothing() {
        let data = mp_os::fs::read(testdata().join("mavlink/autotest.tlog")).unwrap();
        let gathered = gather("autotest.tlog", &data, 101).unwrap();
        assert!(compass_one(&gathered).is_empty());
    }

    #[test]
    fn the_dataflash_path_ends_in_the_box() {
        let data = mp_os::fs::read(testdata().join("dataflash.bin")).unwrap();
        let gathered = gather("dataflash.bin", &data, 0).unwrap();
        let (text, fitted) = report(&gathered, true);
        assert!(fitted, "{text}");
        let lines: Vec<&str> = text.lines().collect();
        assert!(lines[0].starts_with("magcal 1 ofs "), "{text}");
        assert!(lines[1].starts_with("magcalel 1 ofs "), "{text}");
        assert_eq!(lines[2], TITLE);
        assert!(
            lines[3].starts_with("New offsets for compass #1 are "),
            "{text}"
        );
        assert_eq!(lines[5], "Please write these down for manual entry");
    }
}
