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

//! `LogFile` reads through its index exactly what the slice functions read by walking every
//! record: the same fields, units, labels, positions, routes and samples, over the real
//! fixtures, a damaged log, the edge-case log that declares one type twice, and logs joined end
//! to end - which is what a gigabyte of repeated flights looks like, and where a type number can
//! mean one thing in one part of the file and another in the next.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(missing_docs)]

use mp_log::dataflash::{DataflashReader, FMT_TYPE, decode_record};
use mp_log::logfile::LogFile;
use mp_log::overlay::{self, Firmware, Positions};
use mp_log::{LogMessage, plot, track};

fn testdata(name: &str) -> Vec<u8> {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(name);
    std::fs::read(&path).unwrap_or_else(|err| panic!("reading {}: {err}", path.display()))
}

/// What the log browser names modes with.
fn mode_name(firmware: Firmware, mode: u64) -> Option<String> {
    mp_log::convert::flight_mode_name(firmware, u8::try_from(mode).ok()?)
}

/// The logs every comparison runs over, by name.
fn corpus() -> Vec<(&'static str, Vec<u8>)> {
    let healthy = testdata("dataflash.bin");
    let damaged = testdata("dataflash_damaged.bin");
    let edge = testdata("dataflash/edge.bin");
    let joined = |parts: &[&[u8]]| parts.concat();
    vec![
        ("healthy", healthy.clone()),
        ("damaged", damaged.clone()),
        ("edge", edge.clone()),
        ("healthy x3", joined(&[&healthy, &healthy, &healthy])),
        ("damaged, healthy", joined(&[&damaged, &healthy])),
        // edge.bin's type numbers mean other things in the healthy log: every one is declared
        // three times over, differently each time.
        (
            "healthy, edge, healthy",
            joined(&[&healthy, &edge, &healthy]),
        ),
        ("edge, damaged, edge", joined(&[&edge, &damaged, &edge])),
        ("healthy cut short", healthy[..healthy.len() / 3].to_vec()),
        (
            "healthy from its middle",
            healthy[healthy.len() / 2..].to_vec(),
        ),
        ("empty", Vec::new()),
    ]
}

/// Two lists of decoded records are the same, a `NaN` field equal to a `NaN` field: a log holds
/// them (`PARM.Default`, `CTUN.DSAlt`), and `f64`'s own equality would call a record different
/// from itself.
fn assert_same(read: &[(usize, LogMessage)], expected: &[(usize, LogMessage)], what: &str) {
    assert_eq!(read.len(), expected.len(), "{what}: how many");
    for (read, expected) in read.iter().zip(expected) {
        assert_eq!(format!("{read:?}"), format!("{expected:?}"), "{what}");
    }
}

/// The walk every slice function makes, filtered by name: every record with its line, named and
/// decoded by the formats the walk had reached.
fn walked(data: &[u8], names: &[&str]) -> Vec<(usize, LogMessage)> {
    let mut reader = DataflashReader::new(data);
    let mut out = Vec::new();
    let mut line = 0usize;
    while let Some(record) = reader.next_record() {
        let this_line = line;
        line += 1;
        let Some(message) = decode_record(reader.formats(), &data[record.offset..]) else {
            continue;
        };
        if names.contains(&message.name.as_str()) {
            out.push((this_line, message));
        }
    }
    out
}

#[test]
fn the_field_inventory_is_the_walks() {
    for (name, data) in corpus() {
        let log = LogFile::from_bytes(data.clone());
        assert_eq!(log.plottable(), plot::plottable(&data), "{name}");
    }
}

#[test]
fn the_units_are_the_walks() {
    for (name, data) in corpus() {
        let log = LogFile::from_bytes(data.clone());
        assert_eq!(log.units(), plot::units(&data), "{name}");
    }
}

#[test]
fn the_time_origin_is_the_walks() {
    for (name, data) in corpus() {
        let log = LogFile::from_bytes(data.clone());
        assert_eq!(log.time_origin(), plot::time_origin(&data), "{name}");
    }
}

#[test]
fn the_chart_labels_are_the_walks() {
    for (name, data) in corpus() {
        let log = LogFile::from_bytes(data.clone());
        assert_eq!(
            log.overlays(mode_name),
            overlay::overlays(&data, mode_name),
            "{name}"
        );
    }
}

#[test]
fn the_cursors_positions_are_the_walks() {
    for (name, data) in corpus() {
        let log = LogFile::from_bytes(data.clone());
        assert_eq!(log.positions(), Positions::read(&data), "{name}");
    }
}

#[test]
fn the_routes_are_the_walks() {
    for (name, data) in corpus() {
        let log = LogFile::from_bytes(data.clone());
        assert_eq!(log.routes(), track::routes(&data), "{name}");
    }
}

/// A field of every message and instance the inventory offers extracts to the same samples,
/// split by instance where the inventory splits it and unsplit as well; every field of `ATT`,
/// the first plot. In the bigger logs, every tenth of those.
#[test]
fn every_series_is_the_walks() {
    for (name, data) in corpus() {
        let log = LogFile::from_bytes(data.clone());
        let fields = plot::plottable(&data);
        let mut tried: Vec<&plot::PlottableField> = Vec::new();
        for field in &fields {
            let new = tried.last().is_none_or(|last| {
                (&last.message, last.instance) != (&field.message, field.instance)
            });
            if new || field.message == "ATT" {
                tried.push(field);
            }
        }
        let step = if data.len() > 1_000_000 { 10 } else { 1 };
        for field in tried.into_iter().step_by(step) {
            assert_eq!(
                log.extract_instance(&field.message, field.instance, &field.field),
                plot::extract_instance(&data, &field.message, field.instance, &field.field),
                "{name}: {field}"
            );
            if field.instance.is_some() {
                assert_eq!(
                    log.extract(&field.message, &field.field),
                    plot::extract(&data, &field.message, &field.field),
                    "{name}: {field} unsplit"
                );
            }
        }
        // What is not there is nothing, both ways.
        assert!(log.extract("ATT", "NotAField").is_empty(), "{name}");
        assert!(log.extract("NOSUCHMSG", "Roll").is_empty(), "{name}");
        assert_eq!(
            log.extract_instance("VIBE", Some(7), "VibeX"),
            plot::extract_instance(&data, "VIBE", Some(7), "VibeX"),
            "{name}"
        );
    }
}

/// The first plot the log browser draws, on the fixture: the numbers are real, not just equal.
#[test]
fn the_first_plot_is_attitude_roll() {
    let data = testdata("dataflash.bin");
    let log = LogFile::from_bytes(data);
    let roll = log.extract_instance("ATT", None, "Roll");
    assert_eq!(roll.len(), 182);
    assert_eq!(
        roll.iter()
            .map(|point| u32::try_from(point.line).unwrap())
            .collect::<Vec<_>>(),
        log.index().rows_named("ATT"),
        "every ATT record is a sample, on its own row"
    );
    assert!(roll.iter().all(|point| point.value.abs() < 180.0));
}

/// `messages` is the walk filtered by name, for one type, several, and `FMT` itself.
#[test]
fn messages_of_some_types_are_the_walks_records_of_them() {
    let sets: [&[&str]; 6] = [
        &["PARM"],
        &["MSG", "PARM"],
        &["GPS", "POS", "CMD", "CAM"],
        &["FMT"],
        &["EDGE", "EDG2", "STR"],
        &["NOPE"],
    ];
    for (name, data) in corpus() {
        let log = LogFile::from_bytes(data.clone());
        for names in sets {
            let read: Vec<(usize, LogMessage)> = log.messages(names).collect();
            assert_same(&read, &walked(&data, names), &format!("{name}: {names:?}"));
        }
    }
}

/// A row decodes as the grid decodes it: from its offset, against the whole log's formats.
#[test]
fn a_record_is_the_grids_row() {
    for (name, data) in corpus() {
        let log = LogFile::from_bytes(data.clone());
        assert_eq!(log.len(), log.index().len(), "{name}");
        for row in (0..log.len()).step_by(97) {
            let offset = usize::try_from(log.index().offset(row).unwrap()).unwrap();
            assert_eq!(
                format!("{:?}", log.record(row)),
                format!("{:?}", log.index().decode(&data[offset..])),
                "{name} row {row}"
            );
        }
        assert_eq!(log.record(log.len()), None, "{name}");
    }
}

/// Each type's lines are the rows of that type, in order, and together they are every row.
#[test]
fn each_types_lines_are_its_rows() {
    for (name, data) in corpus() {
        let log = LogFile::from_bytes(data);
        let index = log.index();
        let mut total = 0usize;
        for msg_type in 0..=u8::MAX {
            let lines = index.lines_of(msg_type);
            total += lines.len();
            assert!(lines.windows(2).all(|pair| pair[0] < pair[1]), "{name}");
            for line in lines {
                assert_eq!(index.msg_type(*line as usize), Some(msg_type), "{name}");
            }
        }
        assert_eq!(total, index.len(), "{name}");
    }
}

/// The edge log renames its type 10 part way through: the records before are `EDGE` and the
/// ones after `EDG2`, each read with its own declaration, as the walk reads them.
#[test]
fn a_type_declared_twice_is_read_with_the_declaration_in_force() {
    let data = testdata("dataflash/edge.bin");
    let log = LogFile::from_bytes(data.clone());
    let index = log.index();
    let lines = index.lines_of(10);
    let first = index.format_at(10, lines[0] as usize).unwrap();
    let last = index
        .format_at(10, *lines.last().unwrap() as usize)
        .unwrap();
    assert_eq!(first.name, "EDGE");
    assert_eq!(last.name, "EDG2");
    assert_eq!(
        index.formats()[&10].name,
        "EDG2",
        "the whole log's last word"
    );
    let edge: Vec<usize> = log.messages(&["EDGE"]).map(|(line, _)| line).collect();
    let edg2: Vec<usize> = log.messages(&["EDG2"]).map(|(line, _)| line).collect();
    assert!(!edge.is_empty() && !edg2.is_empty());
    assert!(edge.iter().all(|line| line < &edg2[0]));
    assert_eq!(edge.len() + edg2.len(), lines.len());
    // The grid's `rows_named` goes by the whole log's names, as `DFLogBuffer` does.
    assert_eq!(index.rows_named("EDG2").len(), lines.len());
    // And what that means for the products is what it means for the walk.
    assert_eq!(log.plottable(), plot::plottable(&data));
}

/// `FMT` records are never data: a type the log does not know is not in any list.
#[test]
fn format_records_are_lines_but_not_data() {
    let data = testdata("dataflash.bin");
    let log = LogFile::from_bytes(data);
    assert_eq!(log.index().lines_of(FMT_TYPE).len(), 169);
    assert!(!log.plottable().iter().any(|field| field.message == "FMT"));
    assert_eq!(log.messages(&["FMT"]).count(), 169);
}

/// Bytes that are not a log read as a log of nothing, and never panic.
#[test]
fn garbage_reads_as_the_walk_reads_it() {
    for pattern in [
        vec![0xA3u8; 4096],
        [0xA3u8, 0x95, 0x80].repeat(1024),
        [0xA3u8, 0x95, 0xFF].repeat(1024),
        (0..=255u8).cycle().take(10_000).collect(),
    ] {
        let log = LogFile::from_bytes(pattern.clone());
        assert_eq!(log.plottable(), plot::plottable(&pattern));
        assert_eq!(log.units(), plot::units(&pattern));
        assert_eq!(log.time_origin(), plot::time_origin(&pattern));
        assert_eq!(
            log.overlays(mode_name),
            overlay::overlays(&pattern, mode_name)
        );
        assert_eq!(log.positions(), Positions::read(&pattern));
        assert_eq!(log.routes(), track::routes(&pattern));
        assert_same(
            &log.messages(&["FMT"]).collect::<Vec<_>>(),
            &walked(&pattern, &["FMT"]),
            "garbage",
        );
    }
}

/// Opening from a file is opening its bytes.
#[test]
fn opening_a_file_reads_it() {
    let path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/dataflash.bin");
    let opened = LogFile::open(&path).unwrap();
    assert_eq!(opened.bytes(), testdata("dataflash.bin").as_slice());
    assert_eq!(opened.len(), 11_439);
    assert!(LogFile::open(path.with_extension("missing")).is_err());
}
