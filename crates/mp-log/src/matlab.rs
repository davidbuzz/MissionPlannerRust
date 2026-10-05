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

//! Create Matlab file: a dataflash log as a MATLAB `.mat` (level 5, uncompressed), as Mission
//! Planner's `MatLab.ProcessLog` writes it with csmatio.
//!
//! `BUT_matlab_Click` hands each chosen file to `MatLab.ProcessLog` (`GCSViews/FlightData.cs:1387`,
//! `Log/MatLabForms.cs:43-74`), which walks the log's lines as `DFLogBuffer` gives them
//! ([`crate::dflogbuffer`]) and writes `<log>-<lines>.mat` beside it, holding:
//!
//! - `<NAME>_label` for every line that starts `FMT` - a cell of `LineNo` and the columns - except
//!   `PARM`'s. An `FMTU` line starts `FMT` too, so each of those adds a `<units>_label` of its own;
//! - `<NAME>` (or `<NAME>_<instance>` where the type's `FMTU` units mark an instance column) for
//!   every other message whose field count matches its format: one row per line, the line number
//!   then every field read as a number, text as 0;
//! - `MSG1` (and `ISBD1`): every `MSG` line again as a cell of its fields, text kept;
//! - `PARM`: the parameters, name and value, sorted as a `SortedDictionary<string, double>` sorts
//!   under the invariant culture ([`crate::netfmt::culture_compare`]);
//! - `Seen`: every message name that made a row.
//!
//! `tests/matlab.rs` holds the result to Mission Planner's own file byte for byte, with two
//! exceptions it checks separately: the header's creation time, and the order of the names in
//! `Seen`, which the C# takes from a `Hashtable` and so from the runtime's string hashing - mono's
//! order is not .NET Framework's, and neither is anything a reader can rely on. Here it is the
//! order the names were first seen.
//! `// C#: ExtLibs/Utilities/MatLab.cs:16-266`

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use crate::convert::ModeName;
use crate::dflogbuffer::DfLogBuffer;
use crate::netfmt;

/// Why a log could not be converted.
#[derive(Debug, thiserror::Error)]
pub enum MatlabError {
    /// The log could not be read or the file written.
    #[error("{0}")]
    Io(#[from] std::io::Error),
    /// A line the C# indexes past its end: an `FMT` line of fewer than five fields, or an instance
    /// column the line does not have. `ProcessLog` throws there and writes nothing.
    #[error("line {line} of the log is too short for its format")]
    ShortLine {
        /// The line, counted from one as `ProcessLog` counts them.
        line: usize,
    },
    /// A message whose rows are not all the same width, which csmatio refuses.
    #[error("the rows of {name} differ in length")]
    Ragged {
        /// The variable.
        name: String,
    },
}

/// A MATLAB array, as csmatio's `MLArray`s are built here.
#[derive(Debug, Clone, PartialEq)]
pub enum MlArray {
    /// `MLDouble`: a matrix of doubles, rows of equal length.
    Double {
        /// Variable name; `None` is written `@`, as for a cell's element.
        name: Option<String>,
        /// The rows.
        rows: Vec<Vec<f64>>,
    },
    /// `MLChar`: one row of text.
    Char {
        /// Variable name.
        name: Option<String>,
        /// The text.
        text: String,
    },
    /// `MLCell`: a matrix of arrays, in column-major order.
    Cell {
        /// Variable name.
        name: Option<String>,
        /// Rows, columns.
        dims: [usize; 2],
        /// The elements, column-major; missing ones are never left by this writer.
        cells: Vec<MlArray>,
    },
}

/// `CreateCellArray`: a column of trimmed strings. `// C#: ExtLibs/Utilities/MatLab.cs:20-26`
fn cell_array(name: &str, names: &[String]) -> MlArray {
    MlArray::Cell {
        name: Some(name.to_owned()),
        dims: [names.len(), 1],
        cells: names
            .iter()
            .map(|text| MlArray::Char {
                name: None,
                text: netfmt::trim(text).to_owned(),
            })
            .collect(),
    }
}

/// `CreateCellArrayCustom`: a column of the line's fields, a number where `double.TryParse` reads
/// one, a cell of the parts of a `[...]` array, and trimmed text otherwise.
/// `// C#: ExtLibs/Utilities/MatLab.cs:28-55`
fn cell_array_custom(name: &str, items: &[&str]) -> MlArray {
    let cells = items
        .iter()
        .map(|item| {
            if let Some(value) = netfmt::parse_double(item) {
                MlArray::Double {
                    name: None,
                    rows: vec![vec![value]],
                }
            } else if item.trim_start().starts_with('[') && item.trim_end().ends_with(']') {
                let parts: Vec<&str> = item
                    .split([' ', '[', ']'])
                    .filter(|s| !s.is_empty())
                    .collect();
                cell_array_custom("", &parts)
            } else {
                MlArray::Char {
                    name: None,
                    text: netfmt::trim(item).to_owned(),
                }
            }
        })
        .collect::<Vec<_>>();
    MlArray::Cell {
        name: Some(name.to_owned()),
        dims: [cells.len(), 1],
        cells,
    }
}

/// An insertion-ordered map: a `Dictionary<string, _>` enumerates in the order keys were added.
#[derive(Debug, Default)]
struct Ordered<T> {
    keys: Vec<String>,
    values: HashMap<String, T>,
}

impl<T: Default> Ordered<T> {
    fn entry(&mut self, key: &str) -> &mut T {
        if !self.values.contains_key(key) {
            self.keys.push(key.to_owned());
        }
        self.values.entry(key.to_owned()).or_default()
    }

    fn take(mut self) -> Vec<(String, T)> {
        self.keys
            .drain(..)
            .filter_map(|key| self.values.remove(&key).map(|value| (key, value)))
            .collect()
    }
}

/// What `ProcessLog` collects from a log, before `DoWrite`.
#[derive(Debug)]
pub struct MatlabLog {
    /// `a`: how many lines were read, which names the file.
    pub lines: usize,
    /// The arrays, in the order they are written.
    pub arrays: Vec<MlArray>,
}

/// Create Matlab file for one log in memory: every array `MatLab.ProcessLog` would write, in its
/// order. The DataFlash Logs page's "Create Matlab file" button (`BUT_matlab`) calls this through
/// [`process_log_file`].
///
/// `mode_name` names flight modes as for [`crate::convert::convert_bin`]; a named mode reads as
/// the number 0, as the C# parses it.
///
/// # Errors
///
/// Where `ProcessLog` throws: see [`MatlabError`].
/// `// C#: ExtLibs/Utilities/MatLab.cs:57-266`
pub fn process_log(data: &[u8], mode_name: ModeName<'_>) -> Result<MatlabLog, MatlabError> {
    let mut buffer = DfLogBuffer::new(data, mode_name);
    let mut arrays: Vec<MlArray> = Vec::new();
    let mut rows: Ordered<Vec<Vec<f64>>> = Ordered::default();
    let mut cells: Ordered<Vec<MlArray>> = Ordered::default();
    let mut widths: HashMap<String, usize> = HashMap::new();
    let mut seen: Vec<String> = Vec::new();
    let mut params: HashMap<String, f64> = HashMap::new();
    let mut a = 0usize;

    for index in 0..buffer.count() {
        let line = buffer.line(index);
        a += 1;
        let text = line.replace(", ", ",").replace(": ", ":");
        let items: Vec<&str> = text.split([',', ':']).collect();

        if line.starts_with("FMT") {
            // `new string[items.Length - 5 + 1]`, then `names[0] = "LineNo"`.
            if items.len() < 5 {
                return Err(MatlabError::ShortLine { line: a });
            }
            let names: Vec<String> = std::iter::once("LineNo".to_owned())
                .chain(items.iter().skip(5).map(|s| (*s).to_owned()))
                .collect();
            let name = items.get(3).copied().unwrap_or_default();
            let format = cell_array(&format!("{}_label", netfmt::trim(name)), &names);
            if name != "PARM" {
                arrays.push(format);
            }
            widths.insert(name.to_owned(), names.len());
        } else if line.starts_with("PARM") {
            let name = buffer.dflog.find_message_offset("PARM", "Name");
            let value = buffer.dflog.find_message_offset("PARM", "Value");
            if let (Some(name), Some(value)) = (
                name.and_then(|i| items.get(i)),
                value
                    .and_then(|i| items.get(i))
                    .and_then(|s| netfmt::parse_double(s)),
            ) {
                params.insert((*name).to_owned(), value);
            }
        } else {
            if items.len() < 2 {
                continue;
            }
            let Some(&linetype) = items.first() else {
                continue;
            };
            if widths.get(linetype) != Some(&items.len()) {
                continue;
            }
            if !seen.iter().any(|s| s == linetype) {
                seen.push(linetype.to_owned());
            }
            if linetype.to_lowercase() == "msg" || linetype.to_uppercase() == "ISBD" {
                cells
                    .entry(linetype)
                    .push(cell_array_custom(linetype, &items));
            }
            let instance = buffer.instance_index(linetype);
            let key = if instance > 0 {
                let column = usize::try_from(instance).unwrap_or(usize::MAX);
                let Some(value) = items.get(column) else {
                    return Err(MatlabError::ShortLine { line: a });
                };
                format!("{linetype}_{value}")
            } else {
                linetype.to_owned()
            };
            #[allow(clippy::cast_precision_loss)]
            let row = std::iter::once(a as f64)
                .chain(
                    items
                        .iter()
                        .skip(1)
                        .map(|item| netfmt::parse_double_any(item).unwrap_or(0.0)),
                )
                .collect();
            rows.entry(&key).push(row);
        }
    }

    // DoWrite
    for (name, rows) in rows.take() {
        let width = rows.first().map_or(0, Vec::len);
        if rows.iter().any(|row| row.len() != width) {
            return Err(MatlabError::Ragged { name });
        }
        arrays.push(MlArray::Double {
            name: Some(name),
            rows,
        });
    }
    for (name, lines) in cells.take() {
        arrays.push(MlArray::Cell {
            name: Some(format!("{name}1")),
            dims: [1, lines.len()],
            cells: lines,
        });
    }
    let mut names: Vec<(String, f64)> = params.into_iter().collect();
    names.sort_by(|a, b| netfmt::culture_compare(&a.0, &b.0));
    // Column-major: every name, then every value.
    let mut parm = Vec::with_capacity(names.len() * 2);
    parm.extend(names.iter().map(|(name, _)| MlArray::Char {
        name: None,
        text: name.clone(),
    }));
    parm.extend(names.iter().map(|(_, value)| MlArray::Double {
        name: None,
        rows: vec![vec![*value]],
    }));
    arrays.push(MlArray::Cell {
        name: Some("PARM".to_owned()),
        dims: [names.len(), 2],
        cells: parm,
    });
    arrays.push(cell_array("Seen", &seen));
    Ok(MatlabLog { lines: a, arrays })
}

/// Where "Create Matlab file" writes a log's arrays: `<log>-<lines>.mat`, beside it.
/// `// C#: ExtLibs/Utilities/MatLab.cs:204, 259`
#[must_use]
pub fn mat_path_for(log: &Path, lines: usize) -> PathBuf {
    let mut name = log.as_os_str().to_owned();
    name.push(format!("-{lines}.mat"));
    PathBuf::from(name)
}

/// "Create Matlab file" (`BUT_matlab`) for one file: converts `log` and writes the `.mat` beside
/// it, returning where.
///
/// # Errors
///
/// The log cannot be read, the `.mat` cannot be written, or [`process_log`] fails.
/// `// C#: Log/MatLabForms.cs:43-74; ExtLibs/Utilities/MatLab.cs:57-266`
pub fn process_log_file(log: &Path, mode_name: ModeName<'_>) -> Result<PathBuf, MatlabError> {
    let data = std::fs::read(log)?;
    let converted = process_log(&data, mode_name)?;
    let path = mat_path_for(log, converted.lines);
    std::fs::write(&path, write_mat(&converted.arrays, &created_now()))?;
    Ok(path)
}

/// csmatio's header date, `ddd, dd MMM yyyy HH:mm:ss GMT`, for now.
#[must_use]
pub fn created_now() -> String {
    let seconds = web_time::SystemTime::now()
        .duration_since(web_time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let seconds = i64::try_from(seconds).unwrap_or(0);
    let days = seconds.div_euclid(86_400);
    let (year, month, day) = crate::dflogbuffer::civil_from_days(days);
    let of_day = seconds.rem_euclid(86_400);
    const WEEKDAYS: [&str; 7] = ["Thu", "Fri", "Sat", "Sun", "Mon", "Tue", "Wed"];
    const MONTHS: [&str; 12] = [
        "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
    ];
    let weekday = WEEKDAYS
        .get(usize::try_from(days.rem_euclid(7)).unwrap_or(0))
        .copied()
        .unwrap_or("Thu");
    let month = MONTHS
        .get(usize::try_from(month).unwrap_or(1).saturating_sub(1))
        .copied()
        .unwrap_or("Jan");
    format!(
        "{weekday}, {day:02} {month} {year} {:02}:{:02}:{:02} GMT",
        of_day / 3600,
        of_day / 60 % 60,
        of_day % 60
    )
}

/// `Environment.OSVersion.Platform` as the header names it.
const PLATFORM: &str = if cfg!(windows) { "Win32NT" } else { "Unix" };

/// MAT-file data types. `miINT8`, `miUINT16`, `miINT32`, `miUINT32`, `miDOUBLE`, `miMATRIX`.
const MI_INT8: u32 = 1;
const MI_UINT16: u32 = 4;
const MI_INT32: u32 = 5;
const MI_UINT32: u32 = 6;
const MI_DOUBLE: u32 = 9;
const MI_MATRIX: u32 = 14;
/// Array classes: `mxCELL_CLASS`, `mxCHAR_CLASS`, `mxDOUBLE_CLASS`.
const MX_CELL: u32 = 1;
const MX_CHAR: u32 = 4;
const MX_DOUBLE: u32 = 6;

/// A level 5 MAT-file of `arrays`, uncompressed, laid out byte for byte as csmatio's
/// `MatFileWriter(file, list, false)` lays it out: the 116-byte text header naming the platform
/// and `created`, zero-padded; no subsystem data; version 0x0100 and `IM`, little-endian. Then each
/// array as a `miMATRIX` element holding its flags (`miUINT32`), dimensions (`miINT32`), name
/// (`miINT8`, `@` when it has none) and data: doubles as `miDOUBLE` column by column, text as
/// `miUINT16`, a cell as one `miMATRIX` per element, column by column. A data element of one to
/// four bytes goes in the short form, tag and data in eight bytes; anything else is padded to
/// eight. csmatio's own output for arrays of each kind is in `tests/matlab.rs`.
#[must_use]
pub fn write_mat(arrays: &[MlArray], created: &str) -> Vec<u8> {
    let mut out =
        format!("MATLAB 5.0 MAT-file, Platform: {PLATFORM}, CREATED on: {created}").into_bytes();
    out.resize(116, 0);
    out.extend([0u8; 8]);
    out.extend(0x0100u16.to_le_bytes());
    out.extend(*b"IM");
    for array in arrays {
        out.extend(matrix(array));
    }
    out
}

/// One data element: tag and data, padded; the short form for one to four bytes.
fn element(data_type: u32, data: &[u8]) -> Vec<u8> {
    let size = u32::try_from(data.len()).unwrap_or(u32::MAX);
    let mut out = Vec::with_capacity(data.len() + 16);
    if (1..=4).contains(&data.len()) {
        out.extend((data_type | (size << 16)).to_le_bytes());
        out.extend(data);
        out.resize(8, 0);
    } else {
        out.extend(data_type.to_le_bytes());
        out.extend(size.to_le_bytes());
        out.extend(data);
        out.resize(8 + data.len().div_ceil(8) * 8, 0);
    }
    out
}

fn dims_element(dims: [usize; 2]) -> Vec<u8> {
    let mut data = Vec::with_capacity(8);
    for dim in dims {
        data.extend(i32::try_from(dim).unwrap_or(i32::MAX).to_le_bytes());
    }
    element(MI_INT32, &data)
}

fn matrix(array: &MlArray) -> Vec<u8> {
    let (class, name, dims) = match array {
        MlArray::Double { name, rows } => (
            MX_DOUBLE,
            name,
            [rows.len(), rows.first().map_or(0, Vec::len)],
        ),
        MlArray::Char { name, text } => {
            let length = text.encode_utf16().count();
            (
                MX_CHAR,
                name,
                if length == 0 { [0, 0] } else { [1, length] },
            )
        }
        MlArray::Cell { name, dims, .. } => (MX_CELL, name, *dims),
    };
    let mut body = element(MI_UINT32, &[class.to_le_bytes(), [0; 4]].concat());
    body.extend(dims_element(dims));
    let name = match name.as_deref() {
        Some(name) if !name.is_empty() => name,
        _ => "@",
    };
    body.extend(element(MI_INT8, name.as_bytes()));
    match array {
        MlArray::Double { rows, .. } => {
            let width = rows.first().map_or(0, Vec::len);
            let mut data = Vec::with_capacity(rows.len() * width * 8);
            for column in 0..width {
                for row in rows {
                    data.extend(row.get(column).copied().unwrap_or(0.0).to_le_bytes());
                }
            }
            body.extend(element(MI_DOUBLE, &data));
        }
        MlArray::Char { text, .. } => {
            let data: Vec<u8> = text.encode_utf16().flat_map(u16::to_le_bytes).collect();
            body.extend(element(MI_UINT16, &data));
        }
        MlArray::Cell { cells, .. } => {
            for cell in cells {
                body.extend(matrix(cell));
            }
        }
    }
    let mut out = Vec::with_capacity(body.len() + 8);
    out.extend(MI_MATRIX.to_le_bytes());
    out.extend(u32::try_from(body.len()).unwrap_or(u32::MAX).to_le_bytes());
    out.extend(body);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// csmatio's bytes for a cell holding an empty string, a two-letter string, a nested unnamed
    /// cell and a NaN; an empty cell; a 2x3 double; and an empty cell with a long name - written
    /// by `MatFileWriter` under mono (`csmatio` 1.0.20) and copied here from the 128th byte on.
    #[test]
    fn arrays_are_laid_out_as_csmatio_lays_them_out() {
        let arrays = vec![
            MlArray::Cell {
                name: Some("C".to_owned()),
                dims: [4, 1],
                cells: vec![
                    MlArray::Char {
                        name: None,
                        text: String::new(),
                    },
                    MlArray::Char {
                        name: None,
                        text: "ab".to_owned(),
                    },
                    MlArray::Cell {
                        name: Some(String::new()),
                        dims: [2, 1],
                        cells: vec![
                            MlArray::Double {
                                name: None,
                                rows: vec![vec![1.5]],
                            },
                            MlArray::Char {
                                name: None,
                                text: "x".to_owned(),
                            },
                        ],
                    },
                    MlArray::Double {
                        name: None,
                        rows: vec![vec![netfmt::DOTNET_NAN]],
                    },
                ],
            },
            MlArray::Cell {
                name: Some("E".to_owned()),
                dims: [0, 1],
                cells: vec![],
            },
            MlArray::Double {
                name: Some("D".to_owned()),
                rows: vec![vec![1.0, 2.0, 3.0], vec![4.0, 5.0, 6.0]],
            },
            MlArray::Cell {
                name: Some("ABCDEFGHI".to_owned()),
                dims: [0, 2],
                cells: vec![],
            },
        ];
        let expected = "\
            0e000000800100000600000008000000010000000000000005000000080000000400000001000000\
            01000100430000000e00000030000000060000000800000004000000000000000500000008000000\
            0000000000000000010001004000000004000000000000000e000000300000000600000008000000\
            04000000000000000500000008000000010000000200000001000100400000000400040061006200\
            0e000000a00000000600000008000000010000000000000005000000080000000200000001000000\
            01000100400000000e00000038000000060000000800000006000000000000000500000008000000\
            010000000100000001000100400000000900000008000000000000000000f83f0e00000030000000\
            06000000080000000400000000000000050000000800000001000000010000000100010040000000\
            04000200780000000e00000038000000060000000800000006000000000000000500000008000000\
            010000000100000001000100400000000900000008000000000000000000f8ff0e00000028000000\
            06000000080000000100000000000000050000000800000000000000010000000100010045000000\
            0e000000600000000600000008000000060000000000000005000000080000000200000003000000\
            01000100440000000900000030000000000000000000f03f00000000000010400000000000000040\
            0000000000001440000000000000084000000000000018400e000000380000000600000008000000\
            01000000000000000500000008000000000000000200000001000000090000004142434445464748\
            4900000000000000";
        let expected: Vec<u8> = (0..expected.len())
            .step_by(2)
            .map(|at| u8::from_str_radix(&expected[at..at + 2], 16).unwrap())
            .collect();
        let written = write_mat(&arrays, "Wed, 23 Sep 2026 19:31:54 GMT");
        assert_eq!(&written[128..], &expected[..]);
        assert!(written.starts_with(b"MATLAB 5.0 MAT-file, Platform: "));
        assert_eq!(&written[124..128], b"\x00\x01IM");
    }
}
