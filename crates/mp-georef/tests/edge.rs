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

//! The edge photos of `testdata/georef/edge` (made by `common::photos::edge_photos`), held to the
//! C# under mono: each one's `getPhotoTime`, and its `WriteCoordinatesToImage` copy at three
//! positions - south-east with an altitude, north-west at zero, and below zero, which ExifLibrary
//! refuses - byte for byte, with the lines the form would show.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

mod common;

use mp_georef::exif_write::write_coordinates_to_image;
use mp_georef::photos::PhotoTimes;
use mp_georef::time::{DateTime, Kind};

fn kind(t: DateTime) -> &'static str {
    match t.kind {
        Kind::Unspecified => "Unspecified",
        Kind::Utc => "Utc",
        Kind::Local => "Local",
    }
}

fn sorted_files(dir: &std::path::Path) -> Vec<std::path::PathBuf> {
    let mut files: Vec<_> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.is_file())
        .collect();
    files.sort();
    files
}

#[test]
fn photo_times_match_the_csharp() {
    let want = std::fs::read_to_string(common::data().join("golden/edge-phototimes.txt")).unwrap();
    // "Today" is whatever day the oracle ran; the port is handed the same.
    let today_ticks: i64 = want.lines().next().unwrap()["today ".len()..]
        .parse()
        .unwrap();
    let mut times = PhotoTimes::new(DateTime::from_ticks(today_ticks, Kind::Local));
    let mut got = format!("today {today_ticks}\n");
    for file in sorted_files(&common::data().join("edge")) {
        let t = times.get(file.to_str().unwrap());
        got.push_str(&format!(
            "{} {} {}\n",
            file.file_name().unwrap().to_str().unwrap(),
            t.ticks,
            kind(t)
        ));
    }
    assert_eq!(got, want);
}

fn geotag_case(name: &str, lat: f64, lon: f64, alt: f64) -> Vec<String> {
    let work = common::scratch(&format!("edge-{name}"));
    let work_str = work.to_str().unwrap().to_owned();
    let mut copies = Vec::new();
    for file in sorted_files(&common::data().join("edge")) {
        if file.extension().is_some_and(|e| e == "jpg") {
            let to = work.join(file.file_name().unwrap());
            std::fs::copy(&file, &to).unwrap();
            copies.push(to);
        }
    }
    let mut messages = String::new();
    for copy in &copies {
        write_coordinates_to_image(
            copy.to_str().unwrap(),
            lat,
            lon,
            alt,
            &work_str,
            &mut |t: &str| {
                messages.push_str(t);
            },
        );
    }
    let golden = common::data().join("golden").join(name);
    let mut failures = Vec::new();
    let want = std::fs::read_to_string(golden.join("messages.txt")).unwrap();
    // The separator after the directory is the platform's, as the C#'s Path.Combine gives it: a
    // backslash on Windows (the hosted runner, 2026-10-03), where the Linux-made golden has a slash.
    let got = messages
        .replace(&work_str, "{dir}")
        .replace("{dir}\\", "{dir}/");
    if got != want {
        failures.push(format!(
            "{name}: messages\n--- want\n{want}\n--- got\n{got}"
        ));
    }
    let want_dir = golden.join("geotagged");
    let got_dir = work.join("geotagged");
    let want_files: Vec<_> = if want_dir.exists() {
        sorted_files(&want_dir)
    } else {
        Vec::new()
    };
    let got_files: Vec<_> = if got_dir.exists() {
        sorted_files(&got_dir)
    } else {
        Vec::new()
    };
    let names = |v: &[std::path::PathBuf]| {
        v.iter()
            .map(|p| p.file_name().unwrap().to_owned())
            .collect::<Vec<_>>()
    };
    if names(&want_files) != names(&got_files) {
        failures.push(format!(
            "{name}: files {:?} against {:?}",
            names(&got_files),
            names(&want_files)
        ));
    }
    for want in &want_files {
        let got = got_dir.join(want.file_name().unwrap());
        let (w, g) = (
            std::fs::read(want).unwrap(),
            std::fs::read(&got).unwrap_or_default(),
        );
        if w != g {
            let at = w
                .iter()
                .zip(&g)
                .position(|(a, b)| a != b)
                .unwrap_or(w.len().min(g.len()));
            failures.push(format!(
                "{name}: {} differs at byte {at} ({} against {} bytes)",
                want.file_name().unwrap().to_str().unwrap(),
                g.len(),
                w.len()
            ));
        }
    }
    failures
}

#[test]
fn geotagged_copies_match_the_csharp() {
    let mut failures = Vec::new();
    failures.extend(geotag_case("edge-se", -27.469_799_8, 153.025_100_2, 65.18));
    failures.extend(geotag_case("edge-nw", 51.4778, -0.0015, 0.0));
    failures.extend(geotag_case(
        "edge-below",
        -27.469_799_8,
        153.025_100_2,
        -3.5,
    ));
    assert!(failures.is_empty(), "{}", failures.join("\n\n"));
}
