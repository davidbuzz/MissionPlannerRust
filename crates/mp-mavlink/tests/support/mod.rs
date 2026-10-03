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

//! Shared test helpers: a dialect built from the C# original's message table.
//!
//! `testdata/mavlink/csharp_message_infos.csv` is extracted from `MAVLINK_MESSAGE_INFOS` in
//! `references/missionplanner/ExtLibs/Mavlink/Mavlink.cs`. Testing against it means our codec is
//! checked against the shipping C# implementation's own metadata, not against our assumptions.

#![allow(dead_code)]
#![allow(unreachable_pub)]
// shared test helper module, re-included per test binary

// Test code deliberately uses unwrap/expect/indexing: a panic here is a test failure with a
// useful message, which is exactly what we want. The production lint policy stays strict.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_possible_truncation)]

use std::collections::HashMap;

use mp_mavlink::Dialect;

#[derive(Debug, Clone)]
pub struct Row {
    pub id: u32,
    pub name: String,
    pub crc_extra: u8,
    pub min_len: u8,
    pub len: u8,
}

#[derive(Debug, Default)]
pub struct CsvDialect {
    crc: HashMap<u32, u8>,
    pub rows: Vec<Row>,
}

impl Dialect for CsvDialect {
    fn crc_extra(&self, msgid: u32) -> Option<u8> {
        self.crc.get(&msgid).copied()
    }

    fn name(&self) -> &str {
        "csharp-reference"
    }
}

impl CsvDialect {
    pub fn row(&self, name: &str) -> &Row {
        self.rows
            .iter()
            .find(|r| r.name == name)
            .expect("message present in reference table")
    }
}

/// Loads the reference table. Panics loudly if the corpus is missing: a silently skipped
/// differential test is worse than a failing one.
pub fn reference_dialect() -> CsvDialect {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink/csharp_message_infos.csv"
    );
    let text = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("reference corpus missing at {path}: {e}"));

    let mut out = CsvDialect::default();
    for line in text.lines().skip(1) {
        let mut f = line.split(',');
        let (Some(id), Some(name), Some(crc), Some(min_len), Some(len)) =
            (f.next(), f.next(), f.next(), f.next(), f.next())
        else {
            panic!("malformed row: {line}");
        };
        let row = Row {
            id: id.parse().expect("id"),
            name: name.to_owned(),
            crc_extra: crc.parse().expect("crc_extra"),
            min_len: min_len.parse().expect("min_len"),
            len: len.parse().expect("len"),
        };
        out.crc.insert(row.id, row.crc_extra);
        out.rows.push(row);
    }
    assert!(
        out.rows.len() > 300,
        "reference table looks truncated: {} rows",
        out.rows.len()
    );
    out
}
