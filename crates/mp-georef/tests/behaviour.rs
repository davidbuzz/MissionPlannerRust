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

//! What the C# does that the oracle's cases do not reach, each read from the source it cites:
//! which files are photos, what a run keeps between runs, and where a run stops.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

mod common;

use mp_georef::georef::ProcessingMode;
use mp_georef::photos::list_photos;
use mp_georef::time::{DateTime, Kind};
use mp_georef::{Flat, FormSettings, GeoRefImageBase};

fn today() -> DateTime {
    DateTime::from_parts(2026, 9, 24, 0, 0, 0, Kind::Local).unwrap()
}

/// `Directory.GetFiles(dir, "*.jpg")` then `"*.tif"`: the folder itself only, the extension
/// matched without regard to case (as on Windows), `.jpeg` not matched.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:659-664, 798-799`
#[test]
fn photos_are_jpg_then_tif_in_the_folder_itself() {
    let dir = common::scratch("list-photos");
    for name in ["b.JPG", "a.jpg", "c.tif", "d.jpeg", "e.txt", "f.TIF"] {
        std::fs::write(dir.join(name), b"x").unwrap();
    }
    std::fs::create_dir_all(dir.join("geotagged")).unwrap();
    std::fs::write(dir.join("geotagged/g.jpg"), b"x").unwrap();
    let d = dir.to_str().unwrap();
    let names: Vec<String> = list_photos(d)
        .unwrap()
        .iter()
        .map(|p| mp_georef::photos::file_name(p))
        .collect();
    assert_eq!(names, ["a.jpg", "b.JPG", "c.tif", "f.TIF"]);
}

/// "Process" does nothing at all without a log file and a folder (`georefimage.cs:145-148`), and
/// "GeoTag Images" throws on a base altitude that is not a number (`double.Parse`,
/// `georefimage.cs:306-307`, outside any `try`).
#[test]
fn a_missing_log_is_nothing_and_a_bad_base_altitude_throws() {
    let dir = common::scratch("missing-log");
    common::copy_photos(&dir);
    let d = dir.to_str().unwrap();
    let mut georef = GeoRefImageBase::new(today());
    let mut lines = String::new();
    let got = georef.process(
        &format!("{d}/nothing.bin"),
        d,
        &FormSettings::default(),
        &mut |t: &str| lines.push_str(t),
        &Flat,
    );
    assert!(got.is_none());
    assert!(lines.is_empty());

    // A CAM run to have something to geotag.
    std::fs::copy(common::data().join("camera.bin"), dir.join("camera.bin")).unwrap();
    georef.use_amsl_alt = true;
    georef
        .process(
            &format!("{d}/camera.bin"),
            d,
            &FormSettings::default(),
            &mut |_: &str| {},
            &Flat,
        )
        .unwrap();
    let settings = FormSettings {
        base_alt_text: "twelve".to_owned(),
        ..FormSettings::default()
    };
    assert!(
        georef
            .geotag_images(d, &settings, &mut |_: &str| {})
            .is_err()
    );
}

/// The positions are read only when there are none (`GeoRefImageBase.cs:621-636, 752-761`): a
/// time-offset run after a CAM run with AMSL wanted reuses the GPS positions that run read, and
/// says nothing of reading the log.
#[test]
fn positions_are_kept_between_runs() {
    let dir = common::scratch("kept");
    common::copy_photos(&dir);
    std::fs::copy(common::data().join("camera.bin"), dir.join("camera.bin")).unwrap();
    let d = dir.to_str().unwrap();
    let log = format!("{d}/camera.bin");
    let mut georef = GeoRefImageBase::new(today());
    georef.use_amsl_alt = true;
    georef
        .process(&log, d, &FormSettings::default(), &mut |_: &str| {}, &Flat)
        .unwrap();
    let positions = georef.vehicle_locations.len();
    assert_eq!(positions, 617);
    let mut lines = String::new();
    let settings = FormSettings {
        mode: ProcessingMode::TimeOffset,
        offset_text: "36003.3".to_owned(),
        ..FormSettings::default()
    };
    georef
        .process(&log, d, &settings, &mut |t: &str| lines.push_str(t), &Flat)
        .unwrap();
    assert!(lines.starts_with("Log locations : 617\n"), "{lines}");
    assert_eq!(georef.vehicle_locations.len(), positions);
}

/// A photo with no date is `DateTime.MinValue`, and a positive offset takes it below the start
/// of time: `AddSeconds` throws inside the matching's `Parallel.ForEach`, which rethrows it
/// wrapped, the form prints the error, and the photos matched before are not kept
/// (`GeoRefImageBase.cs:685-726`, `georefimage.cs:197-200`). The text is mono's, as a probe of
/// `Parallel.ForEach` over `DateTime.MinValue.AddSeconds(-5)` printed it. (The C# may also print
/// the lines of photos other threads matched before the throw; the port matches in order and
/// stops at the first.)
#[test]
fn an_undated_photo_with_an_offset_stops_the_run() {
    let dir = common::scratch("undated");
    common::copy_photos(&dir);
    std::fs::copy(
        common::data().join("edge/e01_no_exif.jpg"),
        dir.join("e01_no_exif.jpg"),
    )
    .unwrap();
    std::fs::copy(common::data().join("camera.bin"), dir.join("camera.bin")).unwrap();
    let d = dir.to_str().unwrap();
    let mut georef = GeoRefImageBase::new(today());
    let mut lines = String::new();
    let settings = FormSettings {
        mode: ProcessingMode::TimeOffset,
        offset_text: "36003.3".to_owned(),
        ..FormSettings::default()
    };
    let got = georef.process(
        &format!("{d}/camera.bin"),
        d,
        &settings,
        &mut |t: &str| lines.push_str(t),
        &Flat,
    );
    assert!(got.is_none());
    assert!(
        lines.ends_with(
            "Error System.AggregateException: One or more errors occurred. (The added or \
             subtracted value results in an un-representable DateTime.\nParameter name: value)"
        ),
        "{lines}"
    );
    // picturesInfo keeps what it held: the empty dictionary it started with.
    assert_eq!(georef.pictures_info.as_ref().map(|p| p.len()), Some(0));
}

/// The form's smaller pieces of logic: the shutter lag box, the offset a folder's `location.txt`
/// offers, and what changing the log forgets (`georefimage.cs:111-137, 327-335, 360-367`).
#[test]
fn the_forms_text_boxes_and_the_folder_offset() {
    use mp_georef::run::{offset_from_location_txt, parse_shutter_lag};
    assert_eq!(parse_shutter_lag("150"), 150);
    assert_eq!(parse_shutter_lag(" 20 "), 20);
    assert_eq!(parse_shutter_lag("1.5"), 0);
    assert_eq!(parse_shutter_lag("x"), 0);

    let dir = common::scratch("folder-offset");
    let d = dir.to_str().unwrap();
    assert_eq!(offset_from_location_txt(d), None);
    std::fs::write(
        dir.join("location.txt"),
        "seconds_offset: x\nseconds_offset: 36003.3\n",
    )
    .unwrap();
    assert_eq!(offset_from_location_txt(d).as_deref(), Some("36003"));
    // What the report writes never offers one.
    std::fs::write(
        dir.join("location.txt"),
        std::fs::read(common::data().join("golden/time-bin/location.txt")).unwrap(),
    )
    .unwrap();
    assert_eq!(offset_from_location_txt(d), None);

    let mut georef = GeoRefImageBase::new(today());
    let log = common::data().join("camera.bin");
    georef.vehicle_locations =
        mp_georef::georef::read_gps_msg_in_log(log.to_str().unwrap(), "GPS").unwrap();
    georef.forget_positions(true);
    assert!(georef.vehicle_locations.is_empty());
    assert_eq!(georef.pictures_info.as_ref().map(|p| p.len()), Some(0));
}

/// `EstimateOffset` reads photos and log entries 1 to 4 and the last three by index; fewer than
/// four photos is an index past the array, which the form's button does not catch
/// (`GeoRefImageBase.cs:545-560`, `georefimage.cs:258-266`).
#[test]
fn estimating_with_three_photos_throws() {
    let dir = common::scratch("three");
    for n in 1..=3 {
        let name = common::photos::photo_name(n);
        std::fs::copy(common::data().join("photos").join(&name), dir.join(&name)).unwrap();
    }
    let mut georef = GeoRefImageBase::new(today());
    let log = common::data().join("camera.bin");
    let got = georef.estimate_offset(
        log.to_str().unwrap(),
        dir.to_str().unwrap(),
        "GPS",
        false,
        &mut |_: &str| {},
    );
    assert!(got.is_err());
    // With no photos at all the answer is -1.
    let empty = common::scratch("none");
    let got = georef.estimate_offset(
        log.to_str().unwrap(),
        empty.to_str().unwrap(),
        "GPS",
        false,
        &mut |_: &str| {},
    );
    assert_eq!(got.unwrap(), -1.0);
}
