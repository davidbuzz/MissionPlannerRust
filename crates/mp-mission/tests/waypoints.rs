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

//! The waypoint file format, tested against 135 real mission files.
//!
//! The corpus in `testdata/missions/` is ArduPilot's own autotest mission set: every mission its
//! CI flies, written by several different tools over many years. If our reader disagrees with any
//! of them, a user's file will fail to load.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use mp_mission::{MissionItem, read_waypoints, write_waypoints};

fn corpus() -> Vec<(String, String)> {
    let dir = std::path::Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/missions"
    ));
    let mut out = Vec::new();
    for entry in std::fs::read_dir(dir).expect("mission corpus").flatten() {
        let path = entry.path();
        let Ok(text) = std::fs::read_to_string(&path) else {
            continue;
        };
        // The header must be on the first line. Matching anywhere in the file would sweep in
        // source files that merely mention the format - which is exactly what happened when this
        // corpus was first assembled.
        if !text
            .lines()
            .next()
            .is_some_and(|first| first.contains("QGC WPL"))
        {
            continue;
        }
        let name = path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("?")
            .to_owned();
        out.push((name, text));
    }
    out.sort();
    assert!(
        out.len() > 100,
        "expected the full corpus, found {}",
        out.len()
    );
    out
}

#[test]
fn every_real_mission_file_parses() {
    let mut items_total = 0usize;
    let mut failures = Vec::new();

    for (name, text) in corpus() {
        match read_waypoints(&text) {
            Ok(items) => {
                assert!(!items.is_empty(), "{name} parsed to an empty mission");
                items_total += items.len();
            }
            Err(err) => failures.push(format!("{name}: {err}")),
        }
    }

    assert!(
        failures.is_empty(),
        "files that failed to parse:\n{}",
        failures.join("\n")
    );
    println!("parsed {items_total} items across the corpus");
    assert!(
        items_total > 1_000,
        "corpus looks thin: {items_total} items"
    );
}

#[test]
fn reading_and_writing_is_stable() {
    // Byte-identical round-tripping against the corpus is not achievable - those files were
    // written by other tools with different decimal precision - but our own output must be a fixed
    // point: write, read, write again, and the bytes must not move.
    for (name, text) in corpus() {
        let first = read_waypoints(&text).unwrap_or_else(|e| panic!("{name}: {e}"));
        let written = write_waypoints(&first);
        let second = read_waypoints(&written).unwrap_or_else(|e| panic!("{name} rewritten: {e}"));
        let rewritten = write_waypoints(&second);

        assert_eq!(written, rewritten, "{name}: writing is not idempotent");
        assert_eq!(
            first.len(),
            second.len(),
            "{name}: item count changed on round trip"
        );
    }
}

#[test]
fn coordinates_survive_the_round_trip_to_within_a_centimetre() {
    // Eight decimal places of latitude is about a millimetre. Anything worse than a centimetre
    // would be a formatting bug, not rounding.
    for (name, text) in corpus() {
        let original = read_waypoints(&text).unwrap();
        let round_tripped = read_waypoints(&write_waypoints(&original)).unwrap();

        for (a, b) in original.iter().zip(&round_tripped) {
            assert_eq!(a.command, b.command, "{name}: command changed");
            assert_eq!(a.frame, b.frame, "{name}: frame changed");
            assert!(
                (a.x - b.x).abs() < 1e-7 && (a.y - b.y).abs() < 1e-7,
                "{name}: position moved from {:?} to {:?}",
                (a.x, a.y),
                (b.x, b.y)
            );
            assert!(
                (a.z - b.z).abs() < 1e-5,
                "{name}: altitude moved {} to {}",
                a.z,
                b.z
            );
        }
    }
}

#[test]
fn the_output_matches_the_reference_format_byte_for_byte() {
    // Hand-built from the format FlightPlanner.cs writes, including the quirk that the home item
    // uses seven decimals and bare integer parameters while every other item uses eight.
    let mission = vec![
        MissionItem {
            seq: 0,
            current: 1,
            frame: 0,
            command: 16,
            x: -35.362_938,
            y: 149.165_085,
            z: 584.409_973,
            autocontinue: 1,
            ..MissionItem::default()
        },
        MissionItem {
            seq: 1,
            current: 0,
            frame: 3,
            command: 22,
            param1: 15.0,
            x: -35.361_164,
            y: 149.163_986,
            z: 28.110_001,
            autocontinue: 1,
            ..MissionItem::default()
        },
    ];

    let expected = "QGC WPL 110\n\
        0\t1\t0\t16\t0\t0\t0\t0\t-35.3629380\t149.1650850\t584.409973\t1\n\
        1\t0\t3\t22\t15.00000000\t0.00000000\t0.00000000\t0.00000000\t-35.36116400\t149.16398600\t28.110001\t1\n";

    assert_eq!(write_waypoints(&mission), expected);
}

#[test]
fn the_documented_quirks_are_reproduced() {
    // Separators may be tabs, spaces or commas; comments and short lines are skipped; a mission
    // that does not start at sequence 0 gets a home item inserted; command 99 loads as 0.
    let text = "QGC WPL 110\n\
        # a comment line\n\
        \n\
        1 0 3 99 0 0 0 0 -35.1 149.1 100 1\n\
        2,0,3,16,0,0,0,0,-35.2,149.2,120,1\n\
        3\t0\t3\t16\t0\t0\t0\t0\t-35.3\t149.3\t140\t1\n\
        4\t0\t3\n";

    let items = read_waypoints(text).expect("parses");
    assert_eq!(
        items.len(),
        4,
        "home inserted, three records, short line skipped"
    );
    assert_eq!(items[0].seq, 0, "a blank home was inserted");
    assert_eq!(items[1].command, 0, "command 99 loads as 0");
    assert_eq!(items[2].seq, 2, "comma separated record read");
    assert!(
        (items[3].x + 35.3).abs() < 1e-9,
        "tab separated record read"
    );
}

#[test]
fn a_file_without_the_header_is_refused() {
    assert!(read_waypoints("1 0 3 16 0 0 0 0 -35 149 100 1\n").is_err());
    assert!(read_waypoints("").is_err());
}

#[test]
fn non_navigation_commands_are_not_treated_as_positions() {
    // MAV_CMD_DO_SET_SERVO puts a servo number in param1 and leaves x and y at zero. Treating
    // that as a coordinate puts a phantom waypoint in the Gulf of Guinea.
    let servo = MissionItem {
        seq: 3,
        command: 183,
        param1: 9.0,
        ..MissionItem::default()
    };
    assert!(!servo.is_navigation());
    assert_eq!(servo.position().expect("valid"), None);

    let waypoint = MissionItem {
        seq: 4,
        command: 16,
        x: -35.1,
        y: 149.1,
        ..MissionItem::default()
    };
    assert!(waypoint.is_navigation());
    assert!(waypoint.position().expect("valid").is_some());
}

#[test]
fn validation_over_the_real_corpus_is_sane() {
    // Running the checks across 129 missions written by other people is the test that catches a
    // rule which is technically right and practically useless. A check that fires on most real
    // missions is noise, and noise is what makes pilots ignore warnings.
    use mp_mission::validate::{Severity, validate};

    let mut with_danger = 0usize;
    let mut total = 0usize;
    let mut messages: std::collections::BTreeMap<String, usize> = std::collections::BTreeMap::new();

    for (name, text) in corpus() {
        let Ok(items) = mp_mission::read_waypoints(&text) else {
            continue;
        };
        if items.is_empty() {
            continue;
        }
        total += 1;

        // Use the mission's own home as the reference, which is what a pilot flying it would do.
        let home = items
            .first()
            .and_then(|item| item.position().ok().flatten());
        let findings = validate(&items, home);

        if findings.iter().any(|f| f.severity == Severity::Danger) {
            with_danger += 1;
            for finding in findings.iter().filter(|f| f.severity == Severity::Danger) {
                let key: String = finding.message.chars().take(40).collect();
                *messages.entry(key).or_default() += 1;
                let _ = name;
            }
        }
    }

    println!("{with_danger} of {total} corpus missions raise a danger");
    for (message, count) in &messages {
        println!("  {count:>4}  {message}...");
    }

    assert!(total > 100, "expected the full corpus, checked {total}");
    // Some of ArduPilot's test missions deliberately fly odd profiles, so a few dangers are
    // expected. The rate was 27% before altitude checks became vehicle-aware - rover and boat
    // missions were being warned about altitudes that are meaningless to them - and is 6% now.
    // A ceiling of 15% catches a rule that starts firing on ordinary missions again.
    assert!(
        with_danger * 100 < total * 15,
        "{with_danger} of {total} real missions raise a danger; the rules are too aggressive"
    );
}

// ---- The planning screen's file: home at record 0, the grid after it ----

use mp_mission::rows::{Home, split_home};
use mp_mission::waypoints::{BLANK_HOME_RECORD, write_planned};

/// `savewaypoints` byte for byte: home as record 0 - `0 1 0 16`, seven decimals, bare-integer
/// parameters - from the boxes, then each grid row as `(a + 1)`, current 0, autocontinue 1,
/// whatever the row held.
#[test]
fn the_planners_file_has_home_at_record_zero() {
    let home = Home {
        lat: -35.362_938,
        lng: 149.165_085,
        alt: 584.409_973,
    };
    let rows = vec![
        MissionItem {
            seq: 7,
            current: 1,
            frame: 3,
            command: 22,
            param1: 15.0,
            x: -35.361_164,
            y: 149.163_986,
            z: 28.110_001,
            autocontinue: 0,
            ..MissionItem::default()
        },
        MissionItem {
            seq: 9,
            frame: 10,
            command: 16,
            x: -35.360_1,
            y: 149.162_2,
            z: 40.0,
            ..MissionItem::default()
        },
    ];

    let expected = "QGC WPL 110\n\
        0\t1\t0\t16\t0\t0\t0\t0\t-35.3629380\t149.1650850\t584.409973\t1\n\
        1\t0\t3\t22\t15.00000000\t0.00000000\t0.00000000\t0.00000000\t-35.36116400\t149.16398600\t28.110001\t1\n\
        2\t0\t10\t16\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t-35.36010000\t149.16220000\t40.000000\t1\n";
    assert_eq!(write_planned(Some(home), &rows), expected);
}

/// With no home in the boxes the file still starts with a home record - the blank one the C#
/// writes when `double.Parse` throws - so the first row is never read back as home.
#[test]
fn without_a_home_the_blank_record_holds_its_place() {
    let rows = vec![MissionItem {
        command: 16,
        x: -35.1,
        y: 149.1,
        z: 50.0,
        ..MissionItem::default()
    }];
    let text = write_planned(None, &rows);
    let mut lines = text.lines();
    assert_eq!(lines.next(), Some("QGC WPL 110"));
    assert_eq!(lines.next(), Some("0\t1\t0\t0\t0\t0\t0\t0\t0\t0\t0\t1"));
    assert!(lines.next().is_some_and(|line| line.starts_with("1\t")));
    assert_eq!(BLANK_HOME_RECORD, "0\t1\t0\t0\t0\t0\t0\t0\t0\t0\t0\t1");

    let read = read_waypoints(&text).expect("parses");
    let (home, grid) = split_home(&read);
    assert_eq!(home.map(|item| item.command), Some(0), "the blank home");
    assert_eq!(grid.len(), 1);
    assert_eq!(grid[0].seq, 1);
    assert!((grid[0].x - -35.1).abs() < 1e-12);
}

/// Every corpus mission, taken apart as the planning screen reads it and written as it saves,
/// reads back to the same home and the same grid. Where the corpus file is one Mission Planner
/// wrote - it starts `0 1 0 16` and every record is as our record writer makes it - the planning
/// screen's file is that file, byte for byte.
#[test]
fn the_corpus_survives_the_planning_screen() {
    let mut identical = 0usize;
    for (name, text) in corpus() {
        let read = read_waypoints(&text).unwrap_or_else(|e| panic!("{name}: {e}"));
        let (home_item, grid) = split_home(&read);
        let home = home_item.map(|item| Home {
            lat: item.x,
            lng: item.y,
            alt: item.z,
        });
        let written = write_planned(home, &grid);
        let reread = read_waypoints(&written).unwrap_or_else(|e| panic!("{name}: {e}"));
        let (home_again, grid_again) = split_home(&reread);
        assert_eq!(grid_again.len(), grid.len(), "{name}: rows");
        for (a, b) in grid.iter().zip(&grid_again) {
            assert_eq!(
                (a.seq, a.frame, a.command),
                (b.seq, b.frame, b.command),
                "{name}"
            );
            assert!(
                (a.x - b.x).abs() < 1e-7 && (a.y - b.y).abs() < 1e-7,
                "{name}"
            );
        }
        let (Some(first), Some(again)) = (home_item, home_again) else {
            panic!("{name}: no home");
        };
        assert!(
            (first.x - again.x).abs() < 1e-6 && (first.y - again.y).abs() < 1e-6,
            "{name}: home moved"
        );

        let unix = text.replace("\r\n", "\n");
        let as_the_c_sharp_writes = unix
            .lines()
            .nth(1)
            .is_some_and(|line| line.starts_with("0\t1\t0\t16\t"))
            && grid.len() + 1 == read.len()
            && write_waypoints(&read) == unix;
        if as_the_c_sharp_writes {
            assert_eq!(written, unix, "{name}: not what Mission Planner wrote");
            identical += 1;
        }
    }
    println!("{identical} corpus files are Mission Planner's own and came back byte for byte");
    // Five of the corpus files were written by Mission Planner's savewaypoints; the byte-for-byte
    // half of this test is only a test while it has them to compare against.
    assert!(
        identical >= 5,
        "only {identical} Mission Planner files compared"
    );
}
