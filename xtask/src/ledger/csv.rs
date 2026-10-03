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

//! The ledger's CSV dialect: RFC 4180, LF line endings, a field quoted only when it must be.
//!
//! Hand-rolled rather than a dependency because the ledger is edited by people in spreadsheets
//! and by agents in patches, and both need the byte-for-byte output to be stable: a field is
//! quoted if and only if it contains a comma, a quote or a line break, so a round trip through
//! this module never changes a byte that nobody edited.

use anyhow::{Result, bail};

/// Appends one record, terminated by `\n`.
pub fn write_record<'a>(out: &mut String, fields: impl IntoIterator<Item = &'a str>) {
    for (i, field) in fields.into_iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        if field.contains([',', '"', '\n', '\r']) {
            out.push('"');
            out.push_str(&field.replace('"', "\"\""));
            out.push('"');
        } else {
            out.push_str(field);
        }
    }
    out.push('\n');
}

/// One parsed record and the line it started on, so problems can be reported where a person
/// would look for them.
#[derive(Debug)]
pub struct Record {
    /// 1-based line number of the record's first line.
    pub line: usize,
    /// The unquoted fields.
    pub fields: Vec<String>,
}

/// Parses a whole document. Blank lines are skipped; CRLF is accepted because a spreadsheet on
/// Windows will write it, and the next refresh normalises it back to LF.
pub fn parse(text: &str) -> Result<Vec<Record>> {
    // A spreadsheet export often starts with a byte order mark; it is not part of the header.
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    let mut records = Vec::new();
    let mut fields = Vec::new();
    let mut field = String::new();
    let mut quoted = false;
    let mut in_quotes = false;
    let mut line = 1usize;
    let mut record_line = 1usize;
    let mut chars = text.chars().peekable();

    while let Some(c) = chars.next() {
        if in_quotes {
            match c {
                '"' if chars.peek() == Some(&'"') => {
                    chars.next();
                    field.push('"');
                }
                '"' => in_quotes = false,
                '\n' => {
                    line += 1;
                    field.push(c);
                }
                _ => field.push(c),
            }
            continue;
        }
        match c {
            '"' if field.is_empty() && !quoted => {
                in_quotes = true;
                quoted = true;
            }
            '"' => bail!("line {line}: stray quote inside an unquoted field"),
            ',' => {
                fields.push(std::mem::take(&mut field));
                quoted = false;
            }
            '\r' if chars.peek() == Some(&'\n') => {}
            '\n' => {
                end_record(&mut records, &mut fields, &mut field, record_line);
                quoted = false;
                line += 1;
                record_line = line;
            }
            _ if quoted => bail!("line {line}: text after a closing quote"),
            _ => field.push(c),
        }
    }
    if in_quotes {
        bail!("line {record_line}: quoted field is never closed");
    }
    end_record(&mut records, &mut fields, &mut field, record_line);
    Ok(records)
}

fn end_record(
    records: &mut Vec<Record>,
    fields: &mut Vec<String>,
    field: &mut String,
    line: usize,
) {
    fields.push(std::mem::take(field));
    let fields = std::mem::take(fields);
    // A blank line parses as one empty field; it is layout, not a record.
    if fields.len() == 1 && fields.first().is_some_and(String::is_empty) {
        return;
    }
    records.push(Record { line, fields });
}
