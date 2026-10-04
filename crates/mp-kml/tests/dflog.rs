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

//! "Create KML + gpx" held to Mission Planner's own output.
//!
//! The goldens under `testdata/dataflash/golden/kml/` are what `LogOutput.writeKML` left beside
//! each log when `tools/csharp-reference/regen-log.sh` ran the button's body headless under mono,
//! in UTC: the `.gpx`, the waypoint files, the `.param`, the `.kml` taken back out of the `.kmz`,
//! and the `.kmz`'s entry list. Every file is compared byte for byte, with one line changed first:
//! the `.kml`'s root element, whose two namespace declarations mono writes in the opposite order to
//! .NET Framework (see `mp_kml::dflog`).
//!
//! Four logs: the clean one (a mission, parameters, no GPS fix), the damaged one (read as a text
//! log, because it does not open with a header), the damaged one from its first header (the only
//! fixture with a fix, attitude and `POS` that goes the binary way), and the clean one's `.log`
//! read as text.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::{Path, PathBuf};

use mp_kml::dflog::{LogOutput, NEWLINE, dflog_to_kml, process_log, utc};
use mp_log::convert::flight_mode_name;
use mp_log::zip::{self, DosTime};

fn testdata(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(name)
}

/// mono's root element, and .NET Framework's.
const MONO_ROOT: &str = "<kml xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\" \
                         xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\">";
const DOTNET_ROOT: &str = "<kml xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" \
                           xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">";

const WHEN: DosTime = DosTime {
    year: 2026,
    month: 9,
    day: 24,
    hour: 5,
    minute: 30,
    second: 0,
};

fn first_difference(ours: &[u8], theirs: &[u8]) -> String {
    let ours: Vec<&[u8]> = ours.split(|&b| b == b'\n').collect();
    let theirs: Vec<&[u8]> = theirs.split(|&b| b == b'\n').collect();
    for (line, (a, b)) in ours.iter().zip(&theirs).enumerate() {
        if a != b {
            let clip = |s: &[u8]| {
                let s = String::from_utf8_lossy(s).into_owned();
                s.chars().take(300).collect::<String>()
            };
            return format!(
                "line {}:\n  ours:   {}\n  theirs: {}",
                line + 1,
                clip(a),
                clip(b)
            );
        }
    }
    format!("{} lines against {}", ours.len(), theirs.len())
}

/// Runs the button's body on `data`, named `name`, and compares everything it leaves with the
/// goldens in `golden_dir`.
fn assert_matches_golden(name: &str, data: &[u8], golden_dir: &str) {
    let mut output = LogOutput::new();
    process_log(Path::new(name), data, &flight_mode_name, &mut output);
    let kml_name = format!("{name}.kml");
    let written = output
        .write_kml(&Path::new("out").join(&kml_name), &utc, "\n", WHEN)
        .unwrap_or_else(|_| panic!("{name}: writeKML threw"));

    let dir = testdata(golden_dir);
    let stem = kml_name.trim_end_matches(".kml");
    let kmz_name = kml_name
        .to_lowercase()
        .replace(".log.kml", ".kmz")
        .replace(".bin.kml", ".kmz");

    // The side files: exactly the ones the C# left, each byte for byte.
    let mut expected: Vec<String> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(|entry| {
            let entry = entry.unwrap();
            entry
                .file_type()
                .unwrap()
                .is_file()
                .then(|| entry.file_name().to_string_lossy().into_owned())
        })
        .filter(|file| file.starts_with(stem) && !file.ends_with(".kml"))
        .filter(|file| !file.ends_with(".entries"))
        .collect();
    expected.push(kmz_name.clone());
    expected.sort();
    let mut ours: Vec<String> = written
        .files
        .iter()
        .map(|(path, _)| {
            assert_eq!(path.parent(), Some(Path::new("out")), "{}", path.display());
            path.file_name().unwrap().to_string_lossy().into_owned()
        })
        .collect();
    ours.sort();
    assert_eq!(ours, expected, "{name}: the files written");
    for (path, contents) in &written.files {
        let file = path.file_name().unwrap().to_string_lossy().into_owned();
        if file == kmz_name {
            continue;
        }
        let golden = std::fs::read(dir.join(&file)).unwrap();
        assert!(
            *contents == golden,
            "{name}: {file}: {}",
            first_difference(contents, &golden)
        );
    }

    // The .kml, with .NET Framework's root element in place of mono's.
    let golden = std::fs::read_to_string(dir.join(&kml_name)).unwrap();
    assert_eq!(golden.matches(MONO_ROOT).count(), 1, "{name}: mono's root");
    let golden = golden.replacen(MONO_ROOT, DOTNET_ROOT, 1);
    assert!(
        written.kml == golden.as_bytes(),
        "{name}: {kml_name}: {}",
        first_difference(&written.kml, golden.as_bytes())
    );

    // The .kmz: the .kml under its own name, then the model, as the entry list says.
    let (_, kmz) = written
        .files
        .iter()
        .find(|(path, _)| path.ends_with(&kmz_name))
        .unwrap();
    let entries = zip::read(kmz).unwrap();
    let listing: String = entries
        .iter()
        .map(|entry| format!("{},{}\n", entry.name, entry.data.len()))
        .collect();
    let golden_listing = std::fs::read_to_string(dir.join(format!("{kmz_name}.entries"))).unwrap();
    assert_eq!(listing, golden_listing, "{name}: the .kmz");
    assert_eq!(entries[0].data, written.kml);
    assert_eq!(entries[1].data, mp_kml::dflog::PLANE_MODEL);
}

#[test]
fn a_clean_log_writes_its_mission_and_parameters() {
    let data = std::fs::read(testdata("dataflash.bin")).unwrap();
    assert_matches_golden("dataflash.bin", &data, "dataflash/golden/kml");
}

/// It opens with 18 bytes that are not a header, so `DFLogBuffer` takes it for text.
#[test]
fn a_log_that_does_not_open_with_a_header_is_read_as_text() {
    let data = std::fs::read(testdata("dataflash_damaged.bin")).unwrap();
    assert_matches_golden("dataflash_damaged.bin", &data, "dataflash/golden/kml");
}

/// The damaged log from its first header: flight paths per mode, the POS path, 52 aircraft, the
/// GPX track and waypoints - and every duplicate line `DFLogBuffer` makes of an undecodable header.
#[test]
fn a_flight_writes_paths_aircraft_and_a_track() {
    let data = std::fs::read(testdata("dataflash_damaged.bin")).unwrap();
    assert_eq!(
        data.windows(2).position(|w| w == [0xA3, 0x95]),
        Some(18),
        "regen-log.sh cuts the log at its first header, byte 18"
    );
    assert_matches_golden("resync.bin", &data[18..], "dataflash/golden/kml/resync");
}

#[test]
fn a_text_log_is_read_line_by_line() {
    let data = std::fs::read(testdata("dataflash/golden/dataflash.log")).unwrap();
    assert_matches_golden("dataflash.log", &data, "dataflash/golden/kml/text");
}

/// The product path: the files land beside the log, the `.kmz` in lower case.
#[test]
fn the_button_writes_beside_the_log() {
    let dir = std::env::temp_dir().join(format!("mp-kml-dflog-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let log = dir.join("Flight.BIN");
    std::fs::copy(testdata("dataflash.bin"), &log).unwrap();
    let written = dflog_to_kml(&log, &flight_mode_name, &utc).unwrap();
    let names: Vec<String> = written
        .iter()
        .map(|path| path.file_name().unwrap().to_string_lossy().into_owned())
        .collect();
    assert_eq!(
        names,
        [
            "Flight.BIN.gpx",
            "Flight.BIN0wp.txt",
            "Flight.BIN.param",
            "flight.kmz"
        ]
    );
    for path in &written {
        assert!(path.exists(), "{}", path.display());
    }
    // The button writes `Environment.NewLine` as the C# does - "\r\n" on Windows - and the golden
    // was taken under mono on Linux, so its lines end "\n".
    let golden = std::fs::read_to_string(testdata("dataflash/golden/kml/dataflash.bin0wp.txt"))
        .unwrap()
        .replace('\n', NEWLINE);
    assert_eq!(
        std::fs::read_to_string(dir.join("Flight.BIN0wp.txt")).unwrap(),
        golden
    );
    std::fs::remove_dir_all(&dir).unwrap();
}

/// The hand-written log: raw GNSS as RINEX, rally points, a mission uploaded three times, two
/// modes, aircraft that do and do not move, POS thinning, a fix too high and one off the globe.
#[test]
fn a_synthetic_log_writes_every_kind_of_file() {
    let data = std::fs::read(testdata("dataflash/synthetic.log")).unwrap();
    assert_matches_golden("synthetic.log", &data, "dataflash/golden/kml/synthetic");
}

/// The edge-case log through `DFLogBuffer`: nothing a KML is made of, and nothing breaks.
#[test]
fn the_edge_log_writes_an_empty_document() {
    let data = std::fs::read(testdata("dataflash/edge.bin")).unwrap();
    assert_matches_golden("edge.bin", &data, "dataflash/golden/kml/edge");
}
