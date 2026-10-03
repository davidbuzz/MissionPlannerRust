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

//! The dBase III table beside a shapefile (`.dbf`), as far as Load SHP File reads it.
//!
//! `LoadSHPFile` (`GCSViews/FlightPlanner.cs:4569-4716` @ efb0801) opens the shapefile with
//! DotSpatial's `FeatureSet.Open`, calls `FillAttributes()`, and then reads three columns of the
//! `DataTable` by name - `ELEVATION`, `alt` and `wp` - one row per record. DotSpatial's source is
//! not in the reference tree (only `DotSpatial.Projections.dll` is), so this reader follows the
//! dBase III file layout itself: a 32-byte header with the record count at byte 4 and the header
//! and record lengths at 8 and 10, 32-byte field descriptors up to a `0x0D`, then fixed-width
//! records each starting with a deletion flag. Every record is kept, deleted or not, and every
//! value is its text with the padding trimmed; the caller parses what it reads, as
//! `Convert.ChangeType(value, TypeCode.Single)` did. Column names are matched without regard to
//! case, as `DataColumnCollection.Contains` matches them.

/// Why a `.dbf` could not be read.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum DbfError {
    /// Shorter than its header says, or than its records need.
    #[error("the attribute table ends early")]
    Truncated,
    /// The header's lengths do not describe a table.
    #[error("not a dBase table")]
    Malformed,
}

/// One column.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Field {
    name: String,
    length: usize,
}

/// A table: its columns, and every record's values as text.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Table {
    fields: Vec<Field>,
    records: Vec<Vec<String>>,
}

impl Table {
    /// How many records the table has: `dtOriginal.Rows.Count`.
    #[must_use]
    pub fn rows(&self) -> usize {
        self.records.len()
    }

    /// The column called `name`, case ignored: `dtOriginal.Columns.Contains(name)`.
    #[must_use]
    pub fn column(&self, name: &str) -> Option<usize> {
        self.fields
            .iter()
            .position(|field| field.name.eq_ignore_ascii_case(name))
    }

    /// The names of the columns, in order.
    #[must_use]
    pub fn columns(&self) -> Vec<&str> {
        self.fields
            .iter()
            .map(|field| field.name.as_str())
            .collect()
    }

    /// A record's value in a column, its padding trimmed.
    #[must_use]
    pub fn value(&self, row: usize, column: usize) -> Option<&str> {
        self.records
            .get(row)
            .and_then(|record| record.get(column))
            .map(String::as_str)
    }
}

fn u16_at(bytes: &[u8], at: usize) -> Result<usize, DbfError> {
    bytes
        .get(at..at + 2)
        .and_then(|b| b.try_into().ok())
        .map(|b| usize::from(u16::from_le_bytes(b)))
        .ok_or(DbfError::Truncated)
}

fn u32_at(bytes: &[u8], at: usize) -> Result<usize, DbfError> {
    bytes
        .get(at..at + 4)
        .and_then(|b| b.try_into().ok())
        .map(u32::from_le_bytes)
        .and_then(|n| usize::try_from(n).ok())
        .ok_or(DbfError::Truncated)
}

/// Reads a table.
///
/// # Errors
///
/// The bytes end before the header, the field descriptors or the records do, or the header's
/// lengths do not add up.
pub fn read(bytes: &[u8]) -> Result<Table, DbfError> {
    let count = u32_at(bytes, 4)?;
    let header_length = u16_at(bytes, 8)?;
    let record_length = u16_at(bytes, 10)?;
    if header_length < 33 || record_length == 0 {
        return Err(DbfError::Malformed);
    }
    let mut fields = Vec::new();
    let mut at = 32;
    while bytes.get(at).copied().ok_or(DbfError::Truncated)? != 0x0D {
        let descriptor = bytes.get(at..at + 32).ok_or(DbfError::Truncated)?;
        let name_bytes = descriptor.get(..11).unwrap_or_default();
        let name_end = name_bytes.iter().position(|&b| b == 0).unwrap_or(11);
        let name = String::from_utf8_lossy(name_bytes.get(..name_end).unwrap_or_default())
            .trim()
            .to_owned();
        let length = usize::from(descriptor.get(16).copied().ok_or(DbfError::Truncated)?);
        fields.push(Field { name, length });
        at += 32;
        if at > header_length {
            return Err(DbfError::Malformed);
        }
    }
    // The deletion flag, then each field at its width.
    if fields.iter().map(|field| field.length).sum::<usize>() + 1 != record_length {
        return Err(DbfError::Malformed);
    }
    // The header's count is the file's word for it, not the bytes': a table whose header claims
    // a billion records it has not got must fail on the first missing record, not on allocating
    // room for them all (the fuzzer found an 86-byte table that asked for 23 GB).
    let possible = bytes
        .len()
        .saturating_sub(header_length)
        .checked_div(record_length)
        .unwrap_or(0);
    let mut records = Vec::with_capacity(count.min(possible));
    let mut at = header_length;
    for _ in 0..count {
        let record = bytes
            .get(at..at + record_length)
            .ok_or(DbfError::Truncated)?;
        let mut values = Vec::with_capacity(fields.len());
        let mut offset = 1;
        for field in &fields {
            let raw = record
                .get(offset..offset + field.length)
                .unwrap_or_default();
            values.push(String::from_utf8_lossy(raw).trim().to_owned());
            offset += field.length;
        }
        records.push(values);
        at += record_length;
    }
    Ok(Table { fields, records })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The fuzzer's finding (fuzz/artifacts/dbf_table, 2026-10-03): a header counting 989
    /// million records in an 86-byte file. The read must fail at the records, not allocate for
    /// the count.
    #[test]
    fn a_record_count_the_bytes_cannot_hold_fails_at_the_records() {
        let mut bytes = vec![
            0x03, 0x7e, 0x09, 0x18, 0x02, 0x00, 0x00, 0x3b, 0x41, 0x00, 0x0b, 0x00,
        ];
        bytes.resize(32, 0);
        let mut descriptor = [0u8; 32];
        descriptor[..2].copy_from_slice(b"ID");
        descriptor[11] = b'N';
        descriptor[16] = 10;
        bytes.extend(descriptor);
        bytes.push(0x0D);
        bytes.extend(b"          1          2\x1a");
        assert!(matches!(read(&bytes), Err(DbfError::Truncated)));
    }

    /// A table as a shapefile writer lays one out.
    #[allow(clippy::cast_possible_truncation)] // test tables are small
    fn table(fields: &[(&str, u8, usize, u8)], rows: &[&[&str]]) -> Vec<u8> {
        let header_length = 32 + 32 * fields.len() + 1;
        let record_length = 1 + fields.iter().map(|f| f.2).sum::<usize>();
        let mut out = vec![0x03, 126, 9, 24];
        out.extend((rows.len() as u32).to_le_bytes());
        out.extend((header_length as u16).to_le_bytes());
        out.extend((record_length as u16).to_le_bytes());
        out.resize(32, 0);
        for (name, kind, length, decimals) in fields {
            let mut descriptor = [0u8; 32];
            descriptor[..name.len()].copy_from_slice(name.as_bytes());
            descriptor[11] = *kind;
            descriptor[16] = *length as u8;
            descriptor[17] = *decimals;
            out.extend(descriptor);
        }
        out.push(0x0D);
        for row in rows {
            out.push(b' ');
            for ((_, _, length, _), value) in fields.iter().zip(row.iter()) {
                out.extend(format!("{value:>length$}").bytes());
            }
        }
        out.push(0x1A);
        out
    }

    #[test]
    fn columns_are_read_by_name_without_regard_to_case_and_values_trimmed() {
        let bytes = table(
            &[
                ("ELEVATION", b'N', 10, 2),
                ("wp", b'N', 5, 0),
                ("NAME", b'C', 8, 0),
            ],
            &[&["12.50", "2", "a"], &["7", "1", "bb"], &["", "3", ""]],
        );
        let read = read(&bytes).expect("a table");
        assert_eq!(read.rows(), 3);
        assert_eq!(read.columns(), vec!["ELEVATION", "wp", "NAME"]);
        assert_eq!(read.column("elevation"), Some(0));
        assert_eq!(read.column("WP"), Some(1));
        assert_eq!(read.column("alt"), None);
        assert_eq!(read.value(0, 0), Some("12.50"));
        assert_eq!(read.value(1, 1), Some("1"));
        assert_eq!(read.value(2, 0), Some(""));
        assert_eq!(read.value(1, 2), Some("bb"));
        assert_eq!(read.value(3, 0), None);
    }

    #[test]
    fn the_committed_field_table_has_one_id_row() {
        let bytes = include_bytes!("../../../testdata/planner/field.dbf");
        let read = read(bytes).expect("a table");
        assert_eq!(read.columns(), vec!["ID"]);
        assert_eq!(read.rows(), 1);
        assert_eq!(read.value(0, 0), Some("1"));
    }

    #[test]
    fn a_short_or_senseless_table_is_refused() {
        assert_eq!(read(&[0x03; 10]), Err(DbfError::Truncated));
        let mut bytes = table(&[("A", b'C', 4, 0)], &[&["x"]]);
        bytes.truncate(bytes.len() - 4);
        assert_eq!(read(&bytes), Err(DbfError::Truncated));
        let mut bytes = table(&[("A", b'C', 4, 0)], &[&["x"]]);
        bytes[10] = 9; // a record length the fields do not add up to
        assert_eq!(read(&bytes), Err(DbfError::Malformed));
    }
}
