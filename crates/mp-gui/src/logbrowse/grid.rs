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

//! The data grid under the chart: `dataGridView1`, one row per record of the log.
//!
//! Ported from `Log/LogBrowse.cs` @ efb0801 (GPL-3.0-only). The C# grid is in virtual mode:
//! it is told how many rows there are and asks for a cell's text as it paints it, which
//! `CellValueNeeded` answers by decoding that one record. On Mono, where virtual mode misbehaves,
//! it keeps a thousand real rows and repaints them from an offset a scroll bar moves. Either way
//! only what is on screen is ever decoded, and so here: the log is indexed once when it is
//! opened (`mp_log::logfile`), and a screenful of rows is decoded when the window moves.
//!
//! The columns are the C#'s: the line number, the time, the message type - `typecoloum`, which
//! is 2 - and then the record's fields in the order its format declares them. The Graph Left and
//! Graph Right buttons graph whichever cell is current, and refuse what `graphit_clickprocess`
//! refuses; [`Grid::resolve`] is that function.
//! `// C#: Log/LogBrowse.cs:56, 766-800, 1068-1128, 2820-2896, 3204-3272, 3639-3710`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_log::dataflash::{LogMessage, Value};
use mp_log::index::{RecordIndex, boot_ms};

/// The column that holds the message type: `typecoloum`.
///
/// Columns 0 and 1 are the line number and the time, so a record's first field is column 3 and
/// field `n` of its format is column `n + TYPE_COLUMN + 1`.
pub const TYPE_COLUMN: usize = 2;

/// Rows on screen at once.
pub const VISIBLE_ROWS: usize = 16;

/// Height of one row, in pixels. The wheel is converted to rows with it.
pub const ROW_HEIGHT: f32 = 18.0;

/// What `CellValueNeeded` writes a time as: `yyyy-MM-dd HH:mm:ss.fff`.
const TIME_FORMAT: &str = "%Y-%m-%d %H:%M:%S%.3f";

/// One row as the grid shows it.
#[derive(Debug, Clone, PartialEq)]
pub struct GridRow {
    /// Its position in the grid, counting from the top of the (possibly filtered) list.
    pub row: usize,
    /// The record it shows: `DFItem.lineno`, a count of records from the start of the log.
    pub line: usize,
    /// Its message type.
    pub name: String,
    /// The text of each column, from the line number on.
    pub cells: Vec<String>,
    /// The record's instance number, for a message type whose `FMTU` marks one.
    pub instance: Option<i64>,
    /// The record's `TimeUS`, if it has one.
    pub time_us: Option<f64>,
}

/// The current cell: `dataGridView1.CurrentCell`, and the row it is in.
#[derive(Debug, Clone, PartialEq)]
struct Current {
    /// Grid row.
    row: usize,
    /// Grid column.
    column: usize,
    /// The row's contents, kept so the current cell resolves without reading the file again.
    record: GridRow,
}

/// The field a cell holds, once `graphit_clickprocess` has worked it out.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CellField {
    /// Message type.
    pub message: String,
    /// Instance, for a type that has them.
    pub instance: Option<i64>,
    /// Field label.
    pub field: String,
}

impl std::fmt::Display for CellField {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.instance {
            Some(instance) => write!(f, "{}[{instance}].{}", self.message, self.field),
            None => write!(f, "{}.{}", self.message, self.field),
        }
    }
}

/// Why a cell cannot be graphed, in the words the C# message box uses.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Refusal {
    /// The grid has no rows: `Strings.PleaseLoadValidFile`.
    NoLog,
    /// No cell is current: `Strings.PleaseSelectCell`.
    NoCell,
    /// The line-number column.
    FirstColumn,
    /// The row's type has no format: `Strings.NoFMTMessage` and the type.
    NoFormat(String),
    /// The time or type column: `Strings.CannotGraphField`.
    NotAField,
    /// A column past the row's last field: `Strings.InvalidField`.
    InvalidField,
}

impl std::fmt::Display for Refusal {
    /// `// C#: Log/LogBrowse.cs:1070-1109; ExtLibs/Strings/Strings.resx`
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoLog => f.write_str("Please load a valid file"),
            Self::NoCell => f.write_str("Please select a cell first"),
            Self::FirstColumn => {
                f.write_str("Please pick another column, Highlight the cell you wish to graph")
            }
            Self::NoFormat(name) => write!(f, "No FMT message for {name}"),
            Self::NotAField => f.write_str("Cannot graph this field"),
            Self::InvalidField => f.write_str("Invalid Field"),
        }
    }
}

/// How a row's time is written: `DFItem.time`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Clock {
    /// The first GPS fix, in Unix milliseconds, and the boot clock at that fix.
    Gps { start_ms: i64, boot_ms: i64 },
    /// No fix in the log: `gpsstarttime` stays `DateTime.MinValue`, the year 1, and a row's time
    /// is that plus its boot time.
    FromYearOne,
}

impl Clock {
    /// A row's time, in the local zone as `gpsTimeToTime` converts it.
    fn text(self, boot_ms: Option<f64>) -> String {
        self.text_in(boot_ms, &chrono::Local)
    }

    /// A row's time in a given zone.
    ///
    /// `gpsstarttime.AddTicks((timems - msoffset) * 10000)`: the start plus how long after it the
    /// row was logged, where a row with no time of its own counts as zero. The C# works in
    /// 100-nanosecond ticks and shows milliseconds; microseconds lose nothing that shows. A time
    /// the calendar cannot hold is an exception the C#'s cell handler swallows, leaving the cell
    /// empty, and is left empty here.
    fn text_in<Tz: chrono::TimeZone>(self, boot_ms: Option<f64>, zone: &Tz) -> String
    where
        Tz::Offset: std::fmt::Display,
    {
        self.formatted_in(boot_ms, zone, TIME_FORMAT)
    }

    /// A row's time in a given zone and format.
    fn formatted_in<Tz: chrono::TimeZone>(
        self,
        boot_ms: Option<f64>,
        zone: &Tz,
        format: &str,
    ) -> String
    where
        Tz::Offset: std::fmt::Display,
    {
        let boot_ms = boot_ms.unwrap_or(0.0);
        match self {
            Self::Gps {
                start_ms,
                boot_ms: at_fix,
            } => {
                #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
                // a boot clock in milliseconds is far inside both types' ranges
                let since_fix_us = ((boot_ms - at_fix as f64) * 1000.0) as i64;
                chrono::DateTime::from_timestamp_millis(start_ms)
                    .and_then(|start| {
                        start.checked_add_signed(chrono::TimeDelta::microseconds(since_fix_us))
                    })
                    .map(|time| time.with_timezone(zone).format(format).to_string())
                    .unwrap_or_default()
            }
            Self::FromYearOne => {
                #[allow(clippy::cast_possible_truncation)] // as above
                let since_boot_us = (boot_ms * 1000.0) as i64;
                chrono::NaiveDate::from_ymd_opt(1, 1, 1)
                    .and_then(|day| day.and_hms_opt(0, 0, 0))
                    .and_then(|start| {
                        start.checked_add_signed(chrono::TimeDelta::microseconds(since_boot_us))
                    })
                    .map(|time| time.format(format).to_string())
                    .unwrap_or_default()
            }
        }
    }
}

/// A filter to one message type: `logdatafilter`.
#[derive(Debug, Clone)]
struct Filter {
    /// The type chosen.
    name: String,
    /// The rows of that type, in log order.
    rows: Vec<u32>,
}

/// Where the grid's rows come from: the log the browser opened, held in memory with its index,
/// as `DFLogBuffer` holds its stream and `linestartoffset`.
struct Records(std::rc::Rc<mp_log::logfile::LogFile>);

impl Records {
    /// Every record's place in the log.
    fn index(&self) -> &RecordIndex {
        self.0.index()
    }
}

/// The grid: which records it shows, which are on screen, and which cell is current.
pub struct Grid {
    /// Where rows are read back from, and every record's place in the log.
    records: Records,
    /// How the time column is written.
    clock: Clock,
    /// The type the grid is filtered to, if it is.
    filter: Option<Filter>,
    /// The first row on screen.
    first: usize,
    /// The rows on screen, decoded.
    window: Vec<GridRow>,
    /// The current cell.
    current: Option<Current>,
    /// Whether the list of types to filter by is showing.
    choosing: bool,
    /// Wheel movement not yet worth a whole row.
    carry: f32,
}

impl std::fmt::Debug for Grid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Grid")
            .field("records", &self.records.index().len())
            .field("filter", &self.filter.as_ref().map(|filter| &filter.name))
            .field("first", &self.first)
            .field("current", &self.current())
            .finish_non_exhaustive()
    }
}

impl Grid {
    /// A grid over an indexed log, showing its first rows with the first cell current; its rows
    /// are decoded from the bytes the log holds, as `DFLogBuffer`'s indexer decodes a line.
    ///
    /// A `DataGridView` makes its first cell current when it is filled, so a Graph Left pressed
    /// straight after opening a log is refused for the line-number column, as the C# refuses it.
    /// `leap_seconds` is what GPS time is ahead of UTC; the C# asks for today's.
    pub fn over(log: std::rc::Rc<mp_log::logfile::LogFile>, leap_seconds: i64) -> Self {
        let records = Records(log);
        let clock = records
            .index()
            .gps_start()
            .map_or(Clock::FromYearOne, |start| Clock::Gps {
                start_ms: start.unix_ms(leap_seconds),
                boot_ms: start.boot_ms,
            });
        let mut grid = Self {
            records,
            clock,
            filter: None,
            first: 0,
            window: Vec::new(),
            current: None,
            choosing: false,
            carry: 0.0,
        };
        grid.load();
        grid.select(0, 0);
        grid
    }

    /// Rows the grid holds: every record, or those of the filtered type.
    #[must_use]
    pub fn rows(&self) -> usize {
        self.filter
            .as_ref()
            .map_or(self.records.index().len(), |filter| filter.rows.len())
    }

    /// Records in the log.
    #[must_use]
    pub fn records(&self) -> usize {
        self.records.index().len()
    }

    /// Columns every row has: the widest message's fields and the three in front of them.
    ///
    /// The C# sizes its grid from the longest `FMT` column string, in characters rather than
    /// fields, and so ends up far wider than any row; the widest row is the width that shows
    /// anything.
    #[must_use]
    pub fn columns(&self) -> usize {
        self.records.index().widest() + TYPE_COLUMN + 1
    }

    /// The first row on screen.
    #[must_use]
    pub const fn first(&self) -> usize {
        self.first
    }

    /// The rows on screen.
    #[must_use]
    pub fn window(&self) -> &[GridRow] {
        &self.window
    }

    /// The current cell, as row and column.
    #[must_use]
    pub fn current(&self) -> Option<(usize, usize)> {
        self.current
            .as_ref()
            .map(|current| (current.row, current.column))
    }

    /// The type the grid is filtered to.
    #[must_use]
    pub fn filter(&self) -> Option<&str> {
        self.filter.as_ref().map(|filter| filter.name.as_str())
    }

    /// Whether the list of types to filter by is showing.
    #[must_use]
    pub const fn is_choosing(&self) -> bool {
        self.choosing
    }

    /// The types there are to filter by: `SeenMessageTypes`, sorted.
    #[must_use]
    pub fn types(&self) -> Vec<String> {
        self.records.index().seen()
    }

    /// The column headers.
    ///
    /// `dataGridView1_RowEnter` numbers every column from -2, so the line, time and type columns
    /// read -2, -1 and 0, and then names the columns after the current row's fields, blanking
    /// the rest. Entering a row of another type renames them all.
    #[must_use]
    pub fn headers(&self) -> Vec<String> {
        let format = self
            .current
            .as_ref()
            .and_then(|current| self.records.index().format_named(&current.record.name));
        (0..self.columns())
            .map(|column| {
                let field = column.checked_sub(TYPE_COLUMN + 1);
                match (format, field) {
                    (Some(format), Some(field)) => {
                        format.labels.get(field).cloned().unwrap_or_default()
                    }
                    _ => i64::try_from(column)
                        .map_or_else(|_| String::new(), |column| (column - 2).to_string()),
                }
            })
            .collect()
    }

    /// Makes a cell current.
    ///
    /// Rows are clicked on screen, so the row is almost always in the window; one that is not
    /// is read from the file.
    pub fn select(&mut self, row: usize, column: usize) {
        if row >= self.rows() || column >= self.columns() {
            return;
        }
        let record = match self.window.iter().find(|shown| shown.row == row) {
            Some(shown) => shown.clone(),
            None => {
                let Some(line) = self.line_of(row) else {
                    return;
                };
                let message = self.read(line);
                self.build_row(row, line, message.as_ref())
            }
        };
        self.current = Some(Current {
            row,
            column,
            record,
        });
    }

    /// The field the current cell holds, or why it cannot be graphed.
    ///
    /// `graphit_clickprocess`, check for check and in its order: an empty grid, no current cell,
    /// the line-number column, a type with no format, the time or type column, a column past the
    /// row's last field. The instance comes from the row's own `#` field, as the C# reads it out
    /// of the column `FMTU` points at.
    /// `// C#: Log/LogBrowse.cs:1068-1128`
    pub fn resolve(&self) -> Result<CellField, Refusal> {
        if self.rows() == 0 {
            return Err(Refusal::NoLog);
        }
        let Some(current) = &self.current else {
            return Err(Refusal::NoCell);
        };
        if current.column == 0 {
            return Err(Refusal::FirstColumn);
        }
        let name = &current.record.name;
        let Some(format) = self.records.index().format_named(name) else {
            return Err(Refusal::NoFormat(name.clone()));
        };
        let Some(field) = current.column.checked_sub(TYPE_COLUMN + 1) else {
            return Err(Refusal::NotAField);
        };
        let Some(label) = format.labels.get(field) else {
            return Err(Refusal::InvalidField);
        };
        Ok(CellField {
            message: name.clone(),
            instance: current.record.instance,
            field: label.clone(),
        })
    }

    /// Moves the window by whole rows, stopping at either end.
    pub fn scroll_by(&mut self, rows: isize) {
        let last = self.rows().saturating_sub(VISIBLE_ROWS);
        let first = self.first.saturating_add_signed(rows).min(last);
        if first != self.first {
            self.first = first;
            self.load();
        }
    }

    /// Moves the window by a wheel's movement in pixels, positive towards the start.
    ///
    /// A trackpad reports a few pixels at a time, so what does not make a whole row is carried
    /// to the next event rather than dropped - dropping it makes a slow two-finger scroll do
    /// nothing at all.
    pub fn scroll_pixels(&mut self, pixels: f32) {
        self.carry -= pixels / ROW_HEIGHT;
        let whole = self.carry.trunc();
        if whole != 0.0 {
            self.carry -= whole;
            #[allow(clippy::cast_possible_truncation)] // a wheel event is a handful of rows
            self.scroll_by(whole as isize);
        }
    }

    /// Brings a row to the middle of the screen and makes one of its cells current.
    ///
    /// `GoToSample`'s `scrollGrid` and `CurrentCell`. `scrollGrid` scrolls unless the row is
    /// already in the middle band of the screen - which, with an even number of rows on screen,
    /// is a band of no rows, so it always scrolls - to put the row half a screen from the top. A
    /// row past the end is `Rows[SampleID]` throwing, which the C# swallows: nothing happens.
    /// The row is a row of the grid as it is, filtered or not, as the C# indexes it.
    /// `// C#: Log/LogBrowse.cs:3299-3327, 3507-3522`
    pub fn go_to(&mut self, row: usize, column: usize) {
        if row >= self.rows() || column >= self.columns() {
            return;
        }
        let half = VISIBLE_ROWS / 2;
        if self.first + half > row || self.first + VISIBLE_ROWS - half <= row {
            let last = self.rows().saturating_sub(VISIBLE_ROWS);
            let first = row.saturating_sub(half).min(last);
            if first != self.first {
                self.first = first;
                self.carry = 0.0;
                self.load();
            }
        }
        self.select(row, column);
    }

    /// Shows or hides the list of types to filter by: a click on a column header.
    pub fn toggle_chooser(&mut self) {
        self.choosing = !self.choosing;
    }

    /// Filters the grid to one message type, or with `None` shows every record again.
    ///
    /// `dataGridView1_ColumnHeaderMouseClick`: the C# offers the log's types in a small dialog,
    /// and its Cancel clears the filter. Either way the grid starts again at its first row.
    /// `// C#: Log/LogBrowse.cs:2820-2896`
    pub fn set_filter(&mut self, name: Option<&str>) {
        self.filter = name.map(|name| Filter {
            name: name.to_owned(),
            rows: self.records.index().rows_named(name),
        });
        self.choosing = false;
        self.first = 0;
        self.carry = 0.0;
        self.current = None;
        self.load();
        self.select(0, 0);
    }

    /// Makes a cell current and scrolls only as far as it takes to show it, as setting
    /// `CurrentCell` does: `ProcessCmdKey`'s Ctrl+G. A row past the end is refused, and the C#
    /// says "Line Doesn't Exist".
    /// `// C#: Log/LogBrowse.cs:183-205`
    pub fn show(&mut self, row: usize, column: usize) -> bool {
        if row >= self.rows() || column >= self.columns() {
            return false;
        }
        let first = if row < self.first {
            row
        } else if row >= self.first + VISIBLE_ROWS {
            row + 1 - VISIBLE_ROWS
        } else {
            self.first
        };
        if first != self.first {
            self.first = first;
            self.carry = 0.0;
            self.load();
        }
        self.select(row, column);
        true
    }

    /// The time of day a boot-clock time is, as ZedGraph's `PointDateFormat` - `HH:mm:ss.fff` -
    /// writes it for a point on the time axis.
    /// `// C#: Log/LogBrowse.cs:3547`
    #[must_use]
    pub fn time_of_day(&self, boot_ms: f64) -> String {
        self.clock
            .formatted_in(Some(boot_ms), &chrono::Local, "%H:%M:%S%.3f")
    }

    /// The columns `LoadLog` gives the C#'s grid, which Export Visible writes a cell of for
    /// every row: the longest format's column list, in characters, and three.
    ///
    /// `colsplit` is meant to count a format's columns but splits the list's first character, so
    /// it is always 1, and the list's length in characters stands in for its column count.
    /// `// C#: Log/LogBrowse.cs:380-387`
    #[must_use]
    pub fn csv_columns(&self) -> usize {
        self.records
            .index()
            .formats()
            .values()
            .map(|format| format.labels.join(",").len() + TYPE_COLUMN + 1)
            .max()
            .unwrap_or(0)
    }

    /// Every row the grid holds, in order, each read from the log as it is reached: Export
    /// Visible's `foreach (DataGridViewRow row in dataGridView1.Rows)`.
    pub fn for_each_row(&mut self, mut each: impl FnMut(&GridRow)) {
        for row in 0..self.rows() {
            let Some(line) = self.line_of(row) else {
                break;
            };
            let message = self.read(line);
            each(&self.build_row(row, line, message.as_ref()));
        }
    }

    /// The record a grid row shows: its line in the log.
    #[must_use]
    pub fn line_of_row(&self, row: usize) -> Option<usize> {
        self.line_of(row)
    }

    /// The `TimeUS` of the record a grid row shows, if it has one: read from the window when
    /// the row is on screen, which a double-clicked row is.
    #[must_use]
    pub fn time_of_row(&self, row: usize) -> Option<f64> {
        self.window
            .iter()
            .find(|shown| shown.row == row)
            .and_then(|shown| shown.time_us)
    }

    /// The record a grid row shows.
    fn line_of(&self, row: usize) -> Option<usize> {
        match &self.filter {
            Some(filter) => filter
                .rows
                .get(row)
                .and_then(|line| usize::try_from(*line).ok()),
            None => (row < self.records.index().len()).then_some(row),
        }
    }

    /// Decodes the rows on screen.
    fn load(&mut self) {
        let end = (self.first + VISIBLE_ROWS).min(self.rows());
        let mut window = Vec::with_capacity(end.saturating_sub(self.first));
        for row in self.first..end {
            let Some(line) = self.line_of(row) else {
                break;
            };
            let message = self.read(line);
            window.push(self.build_row(row, line, message.as_ref()));
        }
        self.window = window;
    }

    /// Reads one record back from the log.
    ///
    /// A record is at most 255 bytes, so one short read covers it whatever its type. A read that
    /// fails leaves the row blank rather than the grid: the file was there a moment ago, and a
    /// row that cannot be shown is not a reason to show none.
    fn read(&mut self, line: usize) -> Option<LogMessage> {
        self.records.0.record(line)
    }

    /// Writes a record out as a row: `CellValueNeeded` for every column at once.
    /// `// C#: Log/LogBrowse.cs:3204-3272`
    fn build_row(&self, row: usize, line: usize, message: Option<&LogMessage>) -> GridRow {
        let Some(message) = message else {
            return GridRow {
                row,
                line,
                name: String::new(),
                cells: vec![line.to_string()],
                instance: None,
                time_us: None,
            };
        };
        let codes = self
            .records
            .index()
            .format_named(&message.name)
            .map(|format| format.format.as_bytes());
        let mut cells = Vec::with_capacity(message.fields.len() + TYPE_COLUMN + 1);
        cells.push(line.to_string());
        cells.push(self.clock.text(boot_ms(message)));
        cells.push(message.name.clone());
        for (position, (_, value)) in message.fields.iter().enumerate() {
            let code = codes.and_then(|codes| codes.get(position)).copied();
            cells.push(value_text(value, code));
        }
        #[allow(clippy::cast_possible_truncation)] // an instance number is a small integer
        let instance = self
            .records
            .index()
            .msg_type(line)
            .and_then(|msg_type| self.records.index().instance_field(msg_type))
            .and_then(|field| message.fields.get(field))
            .and_then(|(_, value)| value.as_f64())
            .map(|value| value as i64);
        GridRow {
            row,
            line,
            name: message.name.clone(),
            cells,
            instance,
            time_us: message.field("TimeUS").and_then(Value::as_f64),
        }
    }
}

/// A field's value as the grid writes it: `DFItem.items`.
///
/// Numbers in the invariant culture, a `float` field as the single-precision number it was
/// logged as rather than the double it was widened to - `0.1`, not `0.10000000149011612` - text
/// as text, a byte field as its characters with the NULs trimmed, and an array as `[a b c]`, which
/// is `UnionArray.ToString`. Tiny and huge values use an exponent, as .NET does.
/// `// C#: ExtLibs/Utilities/DFLog.cs:79-96; ExtLibs/Utilities/BinaryLog.cs:41-79`
fn value_text(value: &Value, code: Option<u8>) -> String {
    match value {
        Value::Int(value) => value.to_string(),
        Value::Uint(value) => value.to_string(),
        Value::Float(value) => {
            if matches!(code, Some(b'f' | b'g')) {
                #[allow(clippy::cast_possible_truncation)] // it was an f32 on disk
                let single = *value as f32;
                float_text(
                    f64::from(single),
                    &single.to_string(),
                    &format!("{single:E}"),
                )
            } else {
                float_text(*value, &value.to_string(), &format!("{value:E}"))
            }
        }
        Value::Text(text) => text.clone(),
        Value::Bytes(bytes) => String::from_utf8_lossy(bytes).trim_matches('\0').to_owned(),
        Value::Samples(samples) => format!(
            "[{}]",
            samples
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(" ")
        ),
    }
}

/// Picks the plain or the exponent form of a number, as .NET's shortest form does.
fn float_text(value: f64, plain: &str, exponent: &str) -> String {
    let magnitude = value.abs();
    if magnitude != 0.0 && !(1e-5..1e15).contains(&magnitude) {
        exponent.to_owned()
    } else {
        plain.to_owned()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> Vec<u8> {
        let path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/dataflash.bin");
        std::fs::read(&path).unwrap_or_else(|err| panic!("reading {}: {err}", path.display()))
    }

    fn grid_over(data: Vec<u8>) -> Grid {
        Grid::over(
            std::rc::Rc::new(mp_log::logfile::LogFile::from_bytes(data)),
            18,
        )
    }

    fn grid() -> Grid {
        grid_over(fixture())
    }

    /// The grid holds every record, shows one screenful, and starts with the first cell current.
    #[test]
    fn the_grid_opens_on_the_first_record_with_the_first_cell_current() {
        let grid = grid();
        assert_eq!(grid.rows(), 11_439);
        assert_eq!(grid.records(), 11_439);
        assert_eq!(grid.window().len(), VISIBLE_ROWS);
        assert_eq!(grid.current(), Some((0, 0)));
        assert_eq!(
            grid.columns(),
            19,
            "sixteen fields at most, and three in front"
        );

        // The first record is the log declaring FMT itself.
        assert_eq!(
            grid.window()[0].cells,
            vec![
                "0",
                "0001-01-01 00:00:00.000",
                "FMT",
                "128",
                "89",
                "FMT",
                "BBnNZ",
                "Type,Length,Name,Format,Columns",
            ]
        );
    }

    /// The window is one screen wherever it is, and stops at both ends of the log.
    #[test]
    fn the_window_holds_one_screen_and_stops_at_the_ends() {
        let mut grid = grid();
        grid.scroll_by(isize::MAX);
        assert_eq!(grid.first(), grid.rows() - VISIBLE_ROWS);
        assert_eq!(grid.window().len(), VISIBLE_ROWS);
        assert_eq!(
            grid.window().last().map(|row| row.row),
            Some(grid.rows() - 1)
        );

        grid.scroll_by(-5);
        assert_eq!(grid.first(), grid.rows() - VISIBLE_ROWS - 5);
        let lines: Vec<usize> = grid.window().iter().map(|row| row.line).collect();
        let expected: Vec<usize> = (grid.first()..grid.first() + VISIBLE_ROWS).collect();
        assert_eq!(lines, expected, "unfiltered, a row is its own line");

        grid.scroll_by(isize::MIN);
        assert_eq!(grid.first(), 0);

        // Walking the whole log a screen at a time never holds more than a screen.
        let mut first = 0;
        loop {
            assert!(grid.window().len() <= VISIBLE_ROWS);
            grid.scroll_by(VISIBLE_ROWS as isize);
            if grid.first() == first {
                break;
            }
            first = grid.first();
        }
    }

    /// Going to a row puts it half a screen down and makes its time cell current; near either end
    /// the screen stops at the end; past the end nothing happens.
    #[test]
    fn going_to_a_row_centres_it_and_makes_its_time_cell_current() {
        let mut grid = grid();
        grid.go_to(1374, 1);
        assert_eq!(grid.first(), 1374 - VISIBLE_ROWS / 2);
        assert_eq!(grid.current(), Some((1374, 1)));
        assert!(grid.window().iter().any(|row| row.row == 1374));
        // The headers are the row's, as RowEnter names them: 1374 is the MODE record.
        assert_eq!(grid.headers().get(3).map(String::as_str), Some("TimeUS"));
        assert_eq!(grid.headers().get(4).map(String::as_str), Some("Mode"));

        grid.go_to(3, 2);
        assert_eq!(grid.first(), 0, "the top stops at the first row");
        assert_eq!(grid.current(), Some((3, 2)));

        grid.go_to(11_438, 1);
        assert_eq!(
            grid.first(),
            grid.rows() - VISIBLE_ROWS,
            "the end stops too"
        );

        grid.go_to(11_439, 1);
        assert_eq!(grid.current(), Some((11_438, 1)), "past the end, nothing");
    }

    /// A wheel moves whole rows, and a movement too small for one is kept, not lost.
    #[test]
    fn a_wheel_moves_whole_rows_and_carries_the_rest() {
        let mut grid = grid();
        grid.scroll_pixels(-ROW_HEIGHT * 2.5);
        assert_eq!(grid.first(), 2);
        grid.scroll_pixels(-ROW_HEIGHT * 0.5);
        assert_eq!(grid.first(), 3, "the half row left over counted");
        grid.scroll_pixels(ROW_HEIGHT * 3.0);
        assert_eq!(grid.first(), 0, "up is back towards the start");
    }

    /// Filtering keeps one type in log order; clearing the filter brings back every record.
    #[test]
    fn a_filter_shows_one_type_and_clearing_it_shows_all() {
        let mut grid = grid();
        grid.toggle_chooser();
        assert!(grid.is_choosing());
        assert!(grid.types().iter().any(|name| name == "ATT"));

        grid.set_filter(Some("ATT"));
        assert!(!grid.is_choosing(), "choosing a type closes the list");
        assert_eq!(grid.filter(), Some("ATT"));
        assert_eq!(grid.rows(), 182);
        assert!(grid.window().iter().all(|row| row.name == "ATT"));
        assert_eq!(grid.window()[0].line, 1_387, "the first ATT is line 1387");
        assert_eq!(
            grid.window()[0].cells[0],
            "1387",
            "column 0 is the line, not the row"
        );
        assert_eq!(
            grid.current(),
            Some((0, 0)),
            "the grid starts again at its first cell"
        );

        grid.set_filter(None);
        assert_eq!(grid.rows(), 11_439);
        assert_eq!(grid.window()[0].line, 0);
    }

    /// Filtering to a type with no records leaves an empty grid, which refuses first of all.
    #[test]
    fn an_empty_grid_refuses_before_anything_else() {
        let mut grid = grid();
        grid.set_filter(Some("UBX1"));
        assert_eq!(grid.rows(), 0);
        assert!(grid.window().is_empty());
        assert_eq!(grid.resolve(), Err(Refusal::NoLog));
        assert_eq!(Refusal::NoLog.to_string(), "Please load a valid file");
    }

    /// With no cell current there is nothing to graph.
    #[test]
    fn no_current_cell_is_refused() {
        let mut grid = grid();
        grid.current = None;
        assert_eq!(grid.resolve(), Err(Refusal::NoCell));
        assert_eq!(
            grid.resolve().unwrap_err().to_string(),
            "Please select a cell first"
        );
    }

    /// Column 0 is the line number, and the C# says so before it looks at anything else.
    #[test]
    fn column_zero_is_refused() {
        let mut grid = grid();
        grid.set_filter(Some("ATT"));
        grid.select(0, 0);
        assert_eq!(grid.resolve(), Err(Refusal::FirstColumn));
        assert_eq!(
            grid.resolve().unwrap_err().to_string(),
            "Please pick another column, Highlight the cell you wish to graph"
        );
    }

    /// The time and type columns are not fields.
    #[test]
    fn the_time_and_type_columns_are_not_fields() {
        let mut grid = grid();
        grid.set_filter(Some("ATT"));
        for column in [1, TYPE_COLUMN] {
            grid.select(0, column);
            assert_eq!(grid.resolve(), Err(Refusal::NotAField), "column {column}");
        }
        assert_eq!(Refusal::NotAField.to_string(), "Cannot graph this field");
    }

    /// A column past the last of a row's fields is an invalid field, not the next row's.
    #[test]
    fn a_column_past_the_rows_fields_is_invalid() {
        let mut grid = grid();
        grid.set_filter(Some("ATT"));
        // ATT has ten fields, columns 3 to 12.
        grid.select(0, 12);
        assert!(grid.resolve().is_ok());
        grid.select(0, 13);
        assert_eq!(grid.resolve(), Err(Refusal::InvalidField));
        assert_eq!(Refusal::InvalidField.to_string(), "Invalid Field");
    }

    /// A cell resolves to its row's type and its column's field, counted after the type.
    #[test]
    fn a_cell_resolves_to_its_field() {
        let mut grid = grid();
        grid.set_filter(Some("ATT"));
        grid.select(0, 5);
        let field = grid.resolve().unwrap();
        assert_eq!(field.to_string(), "ATT.Roll");
        assert_eq!(field.instance, None);
        grid.select(0, 7);
        assert_eq!(grid.resolve().unwrap().to_string(), "ATT.Pitch");
    }

    /// A type with instances resolves to the instance its own row names.
    #[test]
    fn a_cell_of_a_type_with_instances_carries_the_rows_instance() {
        let mut grid = grid();
        grid.set_filter(Some("VIBE"));
        let mut seen = Vec::new();
        for row in 0..3 {
            grid.select(row, 5);
            let field = grid.resolve().unwrap();
            assert_eq!(field.field, "VibeX");
            seen.push(field.instance);
        }
        seen.sort();
        assert_eq!(seen, vec![Some(0), Some(1), Some(2)], "one row per IMU");
        grid.select(0, 5);
        assert_eq!(grid.resolve().unwrap().to_string(), "VIBE[0].VibeX");
    }

    /// The headers name the current row's fields, and the three in front are numbered.
    #[test]
    fn the_headers_follow_the_current_row() {
        let mut grid = grid();
        let headers = grid.headers();
        assert_eq!(&headers[..4], ["-2", "-1", "0", "Type"]);
        assert_eq!(headers[7], "Columns");
        assert_eq!(headers[8], "", "past FMT's five fields");

        grid.set_filter(Some("ATT"));
        let headers = grid.headers();
        assert_eq!(&headers[3..6], ["TimeUS", "DesRoll", "Roll"]);
        assert_eq!(headers.len(), grid.columns());
    }

    /// With no fix the time counts from the year 1, as `DateTime.MinValue` plus the boot clock.
    #[test]
    fn with_no_fix_the_time_counts_from_the_year_one() {
        let mut grid = grid();
        grid.set_filter(Some("GPS"));
        // TimeUS 52,351,828 is 52.351828 s after boot; the C# shows the milliseconds, truncated.
        assert_eq!(grid.window()[0].cells[1], "0001-01-01 00:00:52.351");
    }

    /// After a fix, a row's time is the fix's GPS time plus how long after it the row was logged.
    #[test]
    fn after_a_fix_the_time_is_gps_time() {
        let clock = Clock::Gps {
            // 2024-02-07 23:59:42 UTC: GPS week 2300 plus four days, less 18 leap seconds.
            start_ms: 1_707_350_400_000 - 18_000,
            boot_ms: 2_500,
        };
        assert_eq!(
            clock.text_in(Some(3_000.9), &chrono::Utc),
            "2024-02-07 23:59:42.500"
        );
        assert_eq!(
            clock.text_in(Some(1_000.0), &chrono::Utc),
            "2024-02-07 23:59:40.500",
            "before the fix is before the start"
        );
        assert_eq!(
            clock.text_in(None, &chrono::Utc),
            "2024-02-07 23:59:39.500",
            "a record with no time counts as boot"
        );
    }

    /// A log with a fix writes its rows in GPS time from that fix on.
    ///
    /// The damaged fixture's first fix is GPS week 2439, 45,075.8 seconds in - 2026-10-04
    /// 12:31:15.8 GPS, 12:30:57.8 UTC - logged 9,999,332 microseconds after boot (SITL's, read
    /// by a header walk in Python).
    #[test]
    fn a_log_with_a_fix_writes_gps_time() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/dataflash_damaged.bin");
        let data = std::fs::read(&path).unwrap();
        let index = RecordIndex::build(&data);
        assert_eq!(
            index.gps_start(),
            Some(mp_log::index::GpsStart {
                week: 2_439,
                week_ms: 45_075_800,
                boot_ms: 9_999,
            })
        );
        let grid = grid_over(data);
        assert_eq!(
            grid.clock.text_in(Some(9_999.332), &chrono::Utc),
            "2026-10-04 12:30:57.800"
        );
    }

    /// Values are written as .NET writes them.
    #[test]
    fn values_are_written_as_the_c_sharp_writes_them() {
        assert_eq!(
            value_text(&Value::Float(f64::from(0.1_f32)), Some(b'f')),
            "0.1"
        );
        assert_eq!(value_text(&Value::Float(0.1), Some(b'd')), "0.1");
        assert_eq!(
            value_text(&Value::Float(-35.363_261_2), Some(b'L')),
            "-35.3632612"
        );
        assert_eq!(value_text(&Value::Float(100.0), Some(b'f')), "100");
        assert_eq!(value_text(&Value::Float(1.5e-7), Some(b'f')), "1.5E-7");
        assert_eq!(value_text(&Value::Int(-3), Some(b'b')), "-3");
        assert_eq!(
            value_text(&Value::Samples(vec![1, -2, 3]), Some(b'a')),
            "[1 -2 3]"
        );
        assert_eq!(
            value_text(&Value::Bytes(b"Roll,Pitch\0\0\0".to_vec()), Some(b'Z')),
            "Roll,Pitch"
        );
    }

    /// A log that never declares `FMT` itself has no format for its `FMT` rows.
    ///
    /// Every ArduPilot log does declare it, first thing; one that does not is the case the C#
    /// check exists for, and it names the type it could not find.
    #[test]
    fn a_row_whose_type_has_no_format_is_refused_by_name() {
        let mut log = Vec::new();
        // TEST is `QH`: eight bytes and two, after the three of the header.
        let mut payload = vec![150, 13];
        for (text, width) in [("TEST", 4), ("QH", 16), ("TimeUS,A", 64)] {
            let mut field = text.as_bytes().to_vec();
            field.resize(width, 0);
            payload.extend(field);
        }
        log.extend([0xA3, 0x95, 0x80]);
        log.extend(payload);
        log.extend([0xA3, 0x95, 150]);
        log.extend(7u64.to_le_bytes());
        log.extend(9u16.to_le_bytes());

        let mut grid = grid_over(log);
        assert_eq!(grid.rows(), 2);
        assert_eq!(grid.window()[0].name, "FMT", "read by its fixed layout");
        assert_eq!(grid.window()[0].cells[7], "TimeUS,A");
        grid.select(0, 3);
        assert_eq!(grid.resolve(), Err(Refusal::NoFormat("FMT".to_owned())));
        assert_eq!(
            grid.resolve().unwrap_err().to_string(),
            "No FMT message for FMT"
        );
        grid.select(1, 4);
        assert_eq!(grid.resolve().unwrap().to_string(), "TEST.A");
    }
}
