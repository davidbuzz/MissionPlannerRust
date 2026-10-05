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

//! `headless-planner log <verb> <file> [out]`: the four buttons of the flight screen's DataFlash Logs page,
//! without the screen.
//!
//! Each verb is one button run on one file, writing what the button writes where the button
//! writes it unless `out` says otherwise:
//!
//! - `bintolog` - "Convert .Bin to .Log" (`but_bintolog`): `<name>.log` beside the `.bin`, or
//!   `out`.
//! - `dflogtokml` - "Create KML + gpx" (`but_dflogtokml`): `.gpx`, waypoint, rally, `.param`,
//!   RINEX and `.kmz` beside the log, or in the directory `out`.
//! - `matlab` - "Create Matlab file" (`BUT_matlab`): `<log>-<lines>.mat` beside the log, or `out`.
//! - `loganalysis` - "Auto Analysis" (`BUT_loganalysis`): runs ArduPilot's LogAnalyzer checks,
//!   ported (`mp_log::analysis`), on the log - a `.bin` converted first - writes the analyzer's
//!   XML beside it (or to `out`), and prints Mission Planner's report of it.
//!
//! Flight modes are named as the application names them, from the firmware the log names.
//! `// C#: GCSViews/FlightData.cs:1082-1098, 1135-1197, 1311-1385, 1387-1390`

#![allow(clippy::print_stdout, clippy::print_stderr)]

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use mp_log::convert::flight_mode_name;

/// The verbs `headless-planner log` takes before a file.
pub(crate) const VERBS: [&str; 4] = ["bintolog", "dflogtokml", "matlab", "loganalysis"];

/// Runs `verb` on `file`.
#[must_use]
pub(crate) fn run(verb: &str, file: &str, out: Option<&str>) -> ExitCode {
    let path = Path::new(file);
    let outcome = match verb {
        "bintolog" => bin_to_log(path, out),
        "dflogtokml" => to_kml(path, out),
        "matlab" => matlab(path, out),
        "loganalysis" => analysis(path, out),
        _ => Err(format!("unknown verb {verb}")),
    };
    match outcome {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("{message}");
            ExitCode::FAILURE
        }
    }
}

fn bin_to_log(path: &Path, out: Option<&str>) -> Result<(), String> {
    let target = out.map_or_else(|| mp_log::convert::log_path_for(path), PathBuf::from);
    mp_log::convert::convert_bin_file(path, &target, &flight_mode_name)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    println!("{}", target.display());
    Ok(())
}

fn to_kml(path: &Path, out: Option<&str>) -> Result<(), String> {
    let kml = match out {
        Some(dir) => {
            let mut name = path.file_name().unwrap_or_default().to_owned();
            name.push(".kml");
            Path::new(dir).join(name)
        }
        None => {
            let mut name = path.as_os_str().to_owned();
            name.push(".kml");
            PathBuf::from(name)
        }
    };
    let written =
        mp_kml::dflog::dflog_to_kml_at(path, &kml, &flight_mode_name, &mp_kml::dflog::local_zone);
    let (paths, failure) = match written {
        Ok(paths) => (paths, None),
        Err(mp_kml::dflog::DflogKmlError::Mission(paths)) => (
            paths,
            Some("the log's mission could not be read; no .kmz".to_owned()),
        ),
        Err(e) => return Err(format!("{}: {e}", path.display())),
    };
    for written in paths {
        println!("{}", written.display());
    }
    failure.map_or(Ok(()), Err)
}

fn matlab(path: &Path, out: Option<&str>) -> Result<(), String> {
    let data = mp_os::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let converted = mp_log::matlab::process_log(&data, &flight_mode_name)
        .map_err(|e| format!("Error converting file {e}"))?;
    let target = out.map_or_else(
        || mp_log::matlab::mat_path_for(path, converted.lines),
        PathBuf::from,
    );
    let bytes = mp_log::matlab::write_mat(&converted.arrays, &mp_log::matlab::created_now());
    mp_os::fs::write(&target, bytes).map_err(|e| format!("{}: {e}", target.display()))?;
    println!("{}", target.display());
    Ok(())
}

fn analysis(path: &Path, out: Option<&str>) -> Result<(), String> {
    let analysis = mp_log::analysis::analyse_to(path, out.map(Path::new), &flight_mode_name)
        .map_err(|e| e.to_string())?;
    print!("{}", mp_log::analysis::report(&analysis));
    Ok(())
}
