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

//! `.param` files written by other ground stations are read, and what we write is read back.
//!
//! Named in Deliverable 12's definition of done. A parameter file is the one artefact an operator moves
//! between tools - saved in Mission Planner, posted on a forum, pulled out of MAVProxy - so
//! failing to read somebody else's file is failing at the format's whole purpose.
//!
//! On the fixtures: `mavproxy.parm` is a genuine capture, the first forty lines of a file pulled
//! off a real vehicle. `mission-planner.param` is written in the C# application's output format -
//! its `#NOTE:` header, comma separator, sorted order, and values as
//! `double.ToString(InvariantCulture)` - rather than captured from a run of it, because that
//! application does not run on this machine. It is a fixture for the format, not proof that a
//! particular build of Mission Planner produced those bytes, and it is worth saying so rather than
//! letting a future reader assume otherwise.
//!
//! The first version of this fixture was written with six decimal places throughout, because that
//! is what the Rust implementation happened to emit. `ExtLibs/Utilities/ParamFile.cs:85-108` is in
//! this repository and says otherwise: shortest representation, scientific below 1e-4. A fixture
//! built to match the implementation tests nothing, and reads as though it tests everything.

use mp_params::param_file::{Change, ParamFile};

fn fixture(name: &str) -> ParamFile {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name);
    let file = ParamFile::load(&path).unwrap_or_else(|err| panic!("reading {name}: {err}"));
    assert!(
        file.rejected().is_empty(),
        "{name} had unreadable lines: {:?}",
        file.rejected()
    );
    file
}

/// The C# application's own format, header and all.
#[test]
fn a_mission_planner_file_is_read() {
    let file = fixture("mission-planner.param");
    assert_eq!(
        file.len(),
        30,
        "thirty-one lines, one of which is the header"
    );
    assert_eq!(file.get("ATC_ANG_PIT_P"), Some(4.5));
    assert_eq!(file.get("BATT_CAPACITY"), Some(3300.0));
    // Negative values, which a naive split on '-' would lose.
    assert_eq!(file.get("COMPASS_OFS_X"), Some(-31.25));
    // A name that fills the 16-byte MAVLink field exactly, with no room for a terminator.
    assert_eq!(file.get("SERVO16_FUNCTION"), Some(0.0));
    // The `#NOTE:` header is a comment, not a parameter called `#NOTE:`.
    assert_eq!(file.get("#NOTE:"), None);
}

/// A real file off a real vehicle, whitespace separated as MAVProxy writes them.
#[test]
fn a_mavproxy_file_is_read() {
    let file = fixture("mavproxy.parm");
    assert_eq!(file.len(), 40);
    assert_eq!(file.get("ACRO_BAL_PITCH"), Some(1.0));
    // Written as a bare integer rather than with six decimal places.
    assert_eq!(file.get("ACRO_OPTIONS"), Some(0.0));
    assert_eq!(file.get("ADSB_LIST_RADIUS"), Some(2000.0));
}

/// The same parameters are the same parameters however they were written down.
#[test]
fn the_two_formats_agree_where_they_overlap() {
    let planner = fixture("mission-planner.param");
    let proxy = fixture("mavproxy.parm");
    let differences = planner.compare(&proxy);
    for difference in &differences {
        // Anything present in both must agree; the two files cover different parameter sets, so
        // names in only one of them are expected.
        assert!(
            !matches!(difference.kind, Change::Changed { .. }),
            "{difference}"
        );
    }
    // And they do overlap, so the test above is testing something.
    let shared = planner
        .iter()
        .filter(|(name, _)| proxy.get(name).is_some())
        .count();
    assert!(shared >= 8, "only {shared} parameters in common");
}

/// What we write is read back as what we wrote. This is the promise a backup makes.
#[test]
fn our_own_output_round_trips_through_the_disk() {
    let original = fixture("mission-planner.param");
    let directory = mp_os::temp_dir().join(format!(
        "headless-planner-param-compat-{}",
        mp_os::process_id()
    ));
    std::fs::create_dir_all(&directory).expect("a writable temp directory");
    let path = directory.join("written.param");

    original.save(&path).expect("writing the file");
    let reread = ParamFile::load(&path).expect("reading it back");

    assert!(reread.rejected().is_empty(), "{:?}", reread.rejected());
    assert_eq!(original.len(), reread.len());
    assert!(
        original.compare(&reread).is_empty(),
        "{:?}",
        original.compare(&reread)
    );

    let _ = std::fs::remove_dir_all(&directory);
}

/// A file we wrote is still in the shape the C# application reads: `NAME,VALUE`, sorted.
///
/// Asserted on the text rather than by re-parsing, because re-parsing only proves we can read our
/// own output, which is not the question.
#[test]
fn what_we_write_is_shaped_the_way_the_other_tools_read() {
    let text = fixture("mission-planner.param").render();
    let mut previous = String::new();
    for line in text.lines() {
        let (name, value) = line
            .split_once(',')
            .unwrap_or_else(|| panic!("a comma in {line:?}"));
        assert!(name > previous.as_str(), "{name} came after {previous}");
        assert!(!value.is_empty(), "a value in {line:?}");
        assert!(
            value.parse::<f64>().is_ok(),
            "{value:?} in {line:?} is not a number the other tools can read"
        );
        // Shortest representation: `1`, not `1.0` and not `1.000000`. A trailing zero after a
        // decimal point is the tell that a fixed-width printer has crept back in.
        if let Some((_, fraction)) = value.split_once('.')
            && !fraction.contains(['E', 'e'])
        {
            assert!(
                !fraction.ends_with('0'),
                "{value:?} has a trailing zero; .NET's ToString would have dropped it"
            );
        }
        previous = name.to_owned();
    }
}

/// Byte-for-byte: reading the fixture and writing it back produces the fixture again.
///
/// The strongest statement this can make without a run of the C# application, and the one Deliverable 12 asks
/// for. It fails on any formatting drift at all - a decimal place gained, a sort order changed, a
/// line ending altered - rather than on a semantic difference.
#[test]
fn a_mission_planner_file_rewrites_to_itself_byte_for_byte() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/mission-planner.param");
    let original = std::fs::read_to_string(&path).expect("the fixture");
    let rewritten = fixture("mission-planner.param").render();

    // The header is a comment we do not reproduce, and the three parameters the load filter drops
    // are gone by design, so the comparison is over what survives a load.
    let expected: String = original
        .lines()
        .filter(|line| !line.starts_with('#'))
        .filter(|line| {
            line.split(',')
                .next()
                .is_some_and(mp_params::param_file::is_loaded)
        })
        .map(|line| format!("{line}\n"))
        .collect();

    assert_eq!(
        rewritten, expected,
        "what we write no longer matches the format the fixture records"
    );
}
