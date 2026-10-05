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

//! The data grid's context menu: Export Visible and Export Files.
//!
//! `contextMenuStrip1` is the grid's right-click menu. Export Visible writes every row the grid
//! holds to a CSV file, each cell's text followed by a comma; Export Files writes out the files a
//! vehicle logged in `FILE` records - a script, a parameter file - into a folder, each record's
//! data at its offset. Both ask where through a dialog, which is a line in the window here.
//! `// C#: Log/LogBrowse.designer.cs:87-105, 353; Log/LogBrowse.cs:3588-3609, 3857-3911`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeMap;
use std::io::{Seek, SeekFrom, Write};
use std::path::{Component, Path, PathBuf};

use mp_log::dataflash::{LogMessage, Value};

/// What Export Visible's `SaveFileDialog` suggests.
pub const VISIBLE_NAME: &str = "output.csv";

/// What Export Files' `FolderBrowserDialog` says.
pub const FILES_DESCRIPTION: &str = "Where to save the files";

/// `Environment.NewLine`, which `StreamWriter.WriteLine` ends each line with: the platform's,
/// as Mono writes it on Linux and .NET on Windows.
pub const NEWLINE: &str = if cfg!(windows) { "\r\n" } else { "\n" };

/// One row of the export: each cell's `FormattedValue` and a comma, for every column the grid has.
///
/// A virtual grid's cell past the end of its record has no value, which formats as nothing, so
/// every line has as many commas as the grid has columns.
/// `// C#: Log/LogBrowse.cs:3594-3605`
#[must_use]
pub fn csv_line(cells: &[String], columns: usize) -> String {
    let mut line = String::new();
    for column in 0..columns.max(cells.len()) {
        if let Some(cell) = cells.get(column) {
            line.push_str(cell);
        }
        line.push(',');
    }
    line
}

/// What happened to the files a log carried.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Exported {
    /// The files written, by the name the log gave them, in the order first seen.
    pub files: Vec<String>,
    /// Names refused because they would have been written outside the folder.
    pub refused: Vec<String>,
}

/// Where a logged file goes: `Path.Combine(dir, name)` with the name's leading separators
/// trimmed, refused if the result is not inside the folder - the C#'s guard against a log that
/// names `../../something`.
/// `// C#: Log/LogBrowse.cs:3874-3889`
#[must_use]
pub fn place(directory: &Path, name: &str) -> Option<PathBuf> {
    let trimmed = name.trim_start_matches(['/', '\\']);
    let mut path = directory.to_path_buf();
    for component in Path::new(&trimmed.replace('\\', "/")).components() {
        match component {
            Component::Normal(part) => path.push(part),
            Component::CurDir => {}
            // `..`, a root or a prefix would leave the folder; `GetFullPath` resolves them and
            // the `StartsWith` check refuses the result.
            Component::ParentDir | Component::RootDir | Component::Prefix(_) => return None,
        }
    }
    (path != directory).then_some(path)
}

/// Export Files: every `FILE` record's data, written at its offset into the file it names.
///
/// A name seen for the first time is created, or emptied if it exists (`SetLength(0)`), and kept
/// open; its records are written where their `Offset` says, `Length` bytes of `Data` each.
/// `// C#: Log/LogBrowse.cs:3857-3911`
///
/// # Errors
/// The first file that cannot be created or written.
pub fn export_files<'a>(
    records: impl IntoIterator<Item = &'a LogMessage>,
    directory: &Path,
) -> std::io::Result<Exported> {
    let mut exported = Exported::default();
    let mut open: BTreeMap<String, std::fs::File> = BTreeMap::new();
    for record in records {
        if record.name != "FILE" {
            continue;
        }
        let Some(name) = record.field("FileName").and_then(Value::as_text) else {
            continue;
        };
        let offset = record
            .field("Offset")
            .and_then(Value::as_f64)
            .unwrap_or(0.0);
        let length = record
            .field("Length")
            .and_then(Value::as_f64)
            .unwrap_or(0.0);
        let data: &[u8] = match record.field("Data") {
            Some(Value::Bytes(bytes)) => bytes,
            _ => &[],
        };
        let Some(path) = place(directory, name) else {
            if !exported.refused.iter().any(|seen| seen == name) {
                exported.refused.push(name.to_owned());
            }
            continue;
        };
        if !open.contains_key(name) {
            if let Some(parent) = path.parent() {
                std::fs::create_dir_all(parent)?;
            }
            let file = std::fs::OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .open(&path)?;
            open.insert(name.to_owned(), file);
            exported.files.push(name.to_owned());
        }
        let Some(file) = open.get_mut(name) else {
            continue;
        };
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        // a u32 offset and a byte length, as logged
        let (offset, length) = (offset.max(0.0) as u64, length.max(0.0) as usize);
        // `data.MakeSize(length)`: cut or zero-padded to the length.
        let mut chunk = data.to_vec();
        chunk.resize(length, 0);
        file.seek(SeekFrom::Start(offset))?;
        file.write_all(&chunk)?;
    }
    Ok(exported)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every cell, then a comma; a short row padded to the grid's width.
    #[test]
    fn a_row_is_its_cells_each_followed_by_a_comma() {
        let cells = vec!["12".to_owned(), "t".to_owned(), "ATT".to_owned()];
        assert_eq!(csv_line(&cells, 5), "12,t,ATT,,,");
        assert_eq!(csv_line(&cells, 2), "12,t,ATT,");
    }

    /// A name that would leave the folder is refused; one with leading separators is inside it.
    #[test]
    fn names_stay_inside_the_folder() {
        let folder = Path::new("/tmp/out");
        assert_eq!(
            place(folder, "/APM/scripts/a.lua"),
            Some(PathBuf::from("/tmp/out/APM/scripts/a.lua"))
        );
        assert_eq!(
            place(folder, "\\x\\y.txt"),
            Some(PathBuf::from("/tmp/out/x/y.txt"))
        );
        assert_eq!(place(folder, "../escape.txt"), None);
        assert_eq!(place(folder, "a/../../escape.txt"), None);
        assert_eq!(place(folder, "/"), None);
    }

    fn file(name: &str, offset: u64, data: &[u8]) -> LogMessage {
        let mut padded = data.to_vec();
        padded.resize(64, 0);
        LogMessage {
            name: "FILE".to_owned(),
            fields: vec![
                ("FileName".to_owned(), Value::Text(name.to_owned())),
                ("Offset".to_owned(), Value::Uint(offset)),
                (
                    "Length".to_owned(),
                    Value::Uint(u64::try_from(data.len()).unwrap_or(0)),
                ),
                ("Data".to_owned(), Value::Bytes(padded)),
            ],
        }
    }

    /// The records are written at their offsets, the file emptied first, the escape refused.
    #[test]
    fn files_are_written_record_by_record() {
        let folder = mp_os::temp_dir().join(format!("mp-export-{}", mp_os::process_id()));
        let _ = std::fs::remove_dir_all(&folder);
        std::fs::create_dir_all(folder.join("APM")).expect("folder");
        std::fs::write(folder.join("APM/a.txt"), b"old contents that are longer").expect("old");
        let records = [
            file("/APM/a.txt", 6, b"world"),
            file("/APM/a.txt", 0, b"hello "),
            file("../evil.txt", 0, b"no"),
            file("b.txt", 0, b"b"),
        ];
        let exported = export_files(&records, &folder).expect("exported");
        assert_eq!(exported.files, vec!["/APM/a.txt", "b.txt"]);
        assert_eq!(exported.refused, vec!["../evil.txt"]);
        assert_eq!(
            std::fs::read(folder.join("APM/a.txt")).expect("a"),
            b"hello world"
        );
        assert_eq!(std::fs::read(folder.join("b.txt")).expect("b"), b"b");
        assert!(
            !folder
                .parent()
                .is_some_and(|up| up.join("evil.txt").exists())
        );
        let _ = std::fs::remove_dir_all(&folder);
    }
}
