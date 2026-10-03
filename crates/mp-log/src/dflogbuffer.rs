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

//! A log as Mission Planner's `DFLogBuffer` and `DFLog` present it to "Create KML + gpx" and
//! "Create Matlab file": a numbered list of text lines, and the format table read back out of them.
//!
//! `DFLogBuffer` does not read a binary log front to back the way "Convert .Bin to .Log" does. It
//! first indexes it - every header [`BinaryLog::read_message_type_offset`] stops at is a line, a
//! message of a type with no format yet among them - and then produces line `n` by reading one
//! message from that line's offset with [`BinaryLog::read_message`], a reader that by then has seen
//! every format in the file. What follows from that, reproduced here:
//!
//! - a message logged before its own `FMT` still decodes, where the straight conversion drops it;
//! - a line whose message will not decode (a header with no format, an all-blank `MSG`) is read on
//!   past it, so it comes out as a second copy of the next message that does;
//! - a file that does not open with `A3 95` is taken to be a text log, and its lines are its bytes
//!   between newlines, as ASCII, the newline kept, and the last unterminated line never read.
//!
//! Before the lines are handed out, the buffer reads its `FMT`, `FMTU`, `MSG` and `PARM` records
//! the way `setlinecount` does. The only thing that leaves behind that changes a line is the
//! firmware the text names, which decides what a flight mode is called.
//! `// C#: ExtLibs/Utilities/DFLogBuffer.cs; ExtLibs/Utilities/DFLog.cs`

use std::collections::HashMap;

use crate::convert::{BinaryLog, HEAD_BYTE1, HEAD_BYTE2, ModeName};
use crate::netfmt;

/// The format message's type. `// C#: ExtLibs/Utilities/DFLogBuffer.cs:128`
const FMT_TYPE: usize = 128;

/// How many `MSG` and `PARM` lines are converted to text to find the firmware, one past the limit
/// because the count is tested after the line is read. `// C#: DFLogBuffer.cs:295-308`
const FIRMWARE_LINES: usize = 100_000;

/// One format, as `DFLog.FMTLine` reads it out of the text of an `FMT` line.
/// `// C#: ExtLibs/Utilities/DFLog.cs:21-29`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Label {
    /// `Id`: the message type.
    pub id: i32,
    /// `Format`: the format characters.
    pub format: String,
    /// `FieldNames`: the columns, as the line has them after `", "` became `","`.
    pub field_names: Vec<String>,
    /// `Length`: the declared record length.
    pub length: i32,
    /// `Name`.
    pub name: String,
}

/// `DFLog`: the formats a converter has read from `FMT` lines, and the field lookups made of them.
/// `// C#: ExtLibs/Utilities/DFLog.cs:16-755`
#[derive(Debug, Clone, Default)]
pub struct DfLog {
    logformat: HashMap<String, Label>,
    /// `msgoffsetcache`: successful lookups only, never forgotten. The C# keys it by the XOR of the
    /// two strings' hash codes, so two different pairs can in principle collide; this keys it by
    /// the pair.
    offsets: HashMap<(String, String), usize>,
}

impl DfLog {
    /// A format table with nothing in it.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// `logformat.ContainsKey(name)`.
    #[must_use]
    pub fn contains(&self, name: &str) -> bool {
        self.logformat.contains_key(name)
    }

    /// `logformat[name]`.
    #[must_use]
    pub fn label(&self, name: &str) -> Option<&Label> {
        self.logformat.get(name)
    }

    /// `logformat.Count`.
    #[must_use]
    pub fn len(&self) -> usize {
        self.logformat.len()
    }

    /// Whether no format has been read.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.logformat.is_empty()
    }

    /// `FMTLine`: reads a format out of a line that starts `FMT` - an `FMTU` line too, whose unit
    /// string then names a format - and anything that does not parse is ignored.
    /// `// C#: ExtLibs/Utilities/DFLog.cs:598-642`
    pub fn fmt_line(&mut self, line: &str) {
        if !line.starts_with("FMT") {
            return;
        }
        let line = line.replace(", ", ",").replace(": ", ":");
        let items: Vec<&str> = netfmt::trim(&line).split([',', ':']).collect();
        if items.len() < 5 {
            return;
        }
        let (Some(name), Some(id), Some(format), Some(length)) = (
            items.get(3),
            items.get(1).and_then(|s| netfmt::parse_i32(s)),
            items.get(4),
            items.get(2).and_then(|s| netfmt::parse_i32(s)),
        ) else {
            return;
        };
        let label = Label {
            id,
            format: (*format).to_owned(),
            field_names: items.iter().skip(5).map(|s| (*s).to_owned()).collect(),
            length,
            name: (*name).to_owned(),
        };
        self.logformat.insert(label.name.clone(), label);
        // "mod for custom logformat that hides the FMT key"
        if !self.logformat.contains_key("FMT") {
            self.logformat.insert(
                "FMT".to_owned(),
                Label {
                    id: 0x80,
                    format: "BBnNZ".to_owned(),
                    field_names: ["Type", "Length", "Name", "Format", "Columns"]
                        .map(str::to_owned)
                        .to_vec(),
                    length: 59,
                    name: "FMT".to_owned(),
                },
            );
        }
    }

    /// `FindMessageOffset`: where column `find` of message `linetype` sits in a line split on `,`
    /// and `:` - one past its index among the columns, the name being first - or `None` (`-1`).
    /// `// C#: ExtLibs/Utilities/DFLog.cs:715-734`
    pub fn find_message_offset(&mut self, linetype: &str, find: &str) -> Option<usize> {
        let key = (linetype.to_owned(), find.to_owned());
        if let Some(&cached) = self.offsets.get(&key) {
            return Some(cached);
        }
        if !self.logformat.contains_key(&linetype.to_uppercase()) {
            return None;
        }
        // The C# then indexes by the name as given, which throws when only its upper case is
        // known; every caller passes an upper case name.
        let index = self
            .logformat
            .get(linetype)?
            .field_names
            .iter()
            .position(|name| name == find)?;
        self.offsets.insert(key, index + 1);
        Some(index + 1)
    }

    /// `GetTimeGPS`: the UTC time a `GPS` line with a 3D fix or better gives, in milliseconds since
    /// the Unix epoch, or `None` for `DateTime.MinValue`. Mission Planner then converts it to local
    /// time, which is the caller's to do.
    ///
    /// The leap seconds are those of today's date, as the C# takes them from `DateTime.Now`.
    /// `// C#: ExtLibs/Utilities/DFLog.cs:646-711`
    pub fn gps_time(&mut self, line: &str) -> Option<i64> {
        if !line.starts_with("GPS") || self.logformat.is_empty() {
            return None;
        }
        let text = line.replace(", ", ",").replace(": ", ":");
        let items: Vec<&str> = text.split([',', ':']).collect();
        if let Some(status) = self.find_message_offset("GPS", "Status") {
            // An index past the line throws out of GetTimeGPS; every caller catches it as no time.
            let status = netfmt::trim(items.get(status)?);
            if status == "0" || status == "1" || status == "2" {
                return None;
            }
        }
        let time = self
            .find_message_offset("GPS", "TimeMS")
            .or_else(|| self.find_message_offset("GPS", "GMS"))?;
        let week = self
            .find_message_offset("GPS", "Week")
            .or_else(|| self.find_message_offset("GPS", "GWk"))?;
        let week = netfmt::parse_i32(items.get(week)?)?;
        #[allow(clippy::cast_precision_loss)]
        let seconds = netfmt::parse_i64(items.get(time)?)? as f64 / 1000.0;
        if !(0..=5000).contains(&week) || !(0.0..=604_800.0).contains(&seconds) {
            return None;
        }
        Some(gps_time_to_utc_millis(week, seconds))
    }

    /// `GetDFItemFromLine`: a line of text as an item, its fields split on `,` and `:` with the
    /// empty ones dropped. An `FMT` line is read into the table on the way.
    ///
    /// The C# also appends the names of an `ERR`'s subsystem and an `EV`'s event to their fields;
    /// nothing that reads these items looks past the fields the format declares, so they are not.
    /// `// C#: ExtLibs/Utilities/DFLog.cs:534-596`
    pub fn item_from_line(&mut self, line: &str) -> DfItem {
        let items = netfmt::trim(line)
            .split([',', ':'])
            .filter(|s| !s.is_empty())
            .map(|s| Some(s.to_owned()))
            .collect();
        if line.starts_with("FMT") {
            self.fmt_line(line);
        }
        DfItem { items }
    }
}

/// `LeapSecondsGPS(DateTime.Now.Year, DateTime.Now.Month)`: TAI-UTC less 19.
/// `// C#: ExtLibs/Utilities/rtcm3.cs:715-735`
fn leap_seconds_gps_now() -> i32 {
    let days = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs() / 86_400);
    let (year, month, _) = civil_from_days(i64::try_from(days).unwrap_or(0));
    let yyyymm = year * 100 + i64::from(month);
    let tai = match yyyymm {
        201_701.. => 37,
        201_507.. => 36,
        201_207.. => 35,
        200_901.. => 34,
        200_601.. => 33,
        199_901.. => 32,
        199_707.. => 31,
        199_601.. => 30,
        _ => 0,
    };
    tai - 19
}

/// `gpsTimeToTime` before its `ToLocalTime`: 1980-01-06 UTC, plus the weeks, plus the seconds less
/// the leap seconds - which `DateTime.AddSeconds` rounds to the millisecond, half away from zero.
/// `// C#: ExtLibs/Utilities/DFLog.cs:702-711`
#[must_use]
pub fn gps_time_to_utc_millis(week: i32, seconds: f64) -> i64 {
    // 1980-01-06T00:00:00Z in Unix milliseconds.
    const GPS_EPOCH: i64 = 315_964_800_000;
    let offset = seconds - f64::from(leap_seconds_gps_now());
    let rounded = offset * 1000.0 + if offset >= 0.0 { 0.5 } else { -0.5 };
    #[allow(clippy::cast_possible_truncation)]
    let millis = rounded as i64;
    GPS_EPOCH + i64::from(week) * 7 * 86_400_000 + millis
}

/// Year, month, day of a day count from 1970-01-01 (proleptic Gregorian).
#[must_use]
pub fn civil_from_days(days: i64) -> (i64, u32, u32) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    (
        year,
        u32::try_from(month).unwrap_or(1),
        u32::try_from(day).unwrap_or(1),
    )
}

/// `DFLog.DFItem` as the converters use it: the fields as text, `None` where the C# holds `null`.
/// `// C#: ExtLibs/Utilities/DFLog.cs:31-255`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DfItem {
    /// `items`: the message name, then its fields.
    pub items: Vec<Option<String>>,
}

impl DfItem {
    /// `msgtype`: the first field, or empty.
    #[must_use]
    pub fn msgtype(&self) -> &str {
        self.items.first().and_then(|s| s.as_deref()).unwrap_or("")
    }

    /// `this[name]`: the field of that column, or `None`.
    /// `// C#: ExtLibs/Utilities/DFLog.cs:212-221`
    pub fn get(&self, dflog: &mut DfLog, name: &str) -> Option<&str> {
        let index = dflog.find_message_offset(self.msgtype(), name)?;
        self.items.get(index)?.as_deref()
    }
}

/// `DFLogBuffer`: a log as numbered lines of text.
/// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:18-846`
pub struct DfLogBuffer<'a> {
    data: &'a [u8],
    binary: bool,
    binlog: BinaryLog,
    /// `linestartoffset`.
    line_starts: Vec<usize>,
    count: usize,
    /// `messageindexline`: the lines of each type.
    index_lines: Vec<Vec<usize>>,
    /// `dflog`: the formats, as the `FMT` lines read.
    pub dflog: DfLog,
    /// `FMTU`: each type's unit and multiplier strings.
    fmtu: HashMap<i32, (String, String)>,
    mode_name: ModeName<'a>,
}

impl std::fmt::Debug for DfLogBuffer<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DfLogBuffer")
            .field("binary", &self.binary)
            .field("count", &self.count)
            .finish_non_exhaustive()
    }
}

impl<'a> DfLogBuffer<'a> {
    /// Indexes a log and reads its format records, as the constructor and `setlinecount` do.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:52-332`
    #[must_use]
    pub fn new(data: &'a [u8], mode_name: ModeName<'a>) -> Self {
        let binary = data.first() == Some(&HEAD_BYTE1) && data.get(1) == Some(&HEAD_BYTE2);
        let mut buffer = Self {
            data,
            binary,
            binlog: BinaryLog::new(),
            line_starts: Vec::new(),
            count: 0,
            index_lines: vec![Vec::new(); 256],
            dflog: DfLog::new(),
            fmtu: HashMap::new(),
            mode_name,
        };
        if binary {
            buffer.index_binary();
        } else {
            buffer.index_text();
        }
        buffer.read_tables();
        buffer
    }

    /// `setlinecount`'s binary half. `// C#: ExtLibs/Utilities/DFLogBuffer.cs:103-143`
    fn index_binary(&mut self) {
        let length = self.data.len();
        let mut pos = 0usize;
        while pos < length {
            let (msg_type, offset) = self
                .binlog
                .read_message_type_offset(self.data, &mut pos, length);
            if msg_type == 0 && offset == 0 {
                continue;
            }
            if let Some(lines) = self.index_lines.get_mut(usize::from(msg_type)) {
                lines.push(self.count);
            }
            self.line_starts.push(offset);
            self.count += 1;
        }
        // "build fmt line database to pre seed the FMT message"
        for line in self.index_lines.get(FMT_TYPE).cloned().unwrap_or_default() {
            let text = self.line(line);
            self.dflog.fmt_line(&text);
        }
    }

    /// `setlinecount`'s text half. `// C#: ExtLibs/Utilities/DFLogBuffer.cs:144-201`
    fn index_text(&mut self) {
        self.line_starts.push(0);
        for (at, &byte) in self.data.iter().enumerate() {
            if byte == b'\n' {
                self.line_starts.push(at + 1);
                self.count += 1;
            }
        }
        for line in 0..self.count {
            let text = self.line(line);
            let Some(comma) = text.find(',').filter(|&at| at > 0) else {
                continue;
            };
            let msgtype = text.get(..comma).unwrap_or_default().to_owned();
            if msgtype == "FMT" {
                self.dflog.fmt_line(&text);
            }
            if let Some(label) = self.dflog.label(&msgtype) {
                // `(byte)Id`
                let msg_type = usize::from(label.id.to_le_bytes()[0]);
                if let Some(lines) = self.index_lines.get_mut(msg_type) {
                    lines.push(line);
                }
            }
        }
    }

    /// The rest of `setlinecount`: the `FMT` and `FMTU` tables, and the `MSG` and `PARM` lines read
    /// as text for the firmware. `// C#: ExtLibs/Utilities/DFLogBuffer.cs:207-331`
    fn read_tables(&mut self) {
        for item in self.items_of(&["FMT"]) {
            let (line, item) = item;
            if item.items.is_empty() {
                continue;
            }
            // `FMT[int.Parse(item["Type"])] = (...)` throws before the line is read again when a
            // field it takes is missing.
            let complete = item
                .get(&mut self.dflog, "Type")
                .and_then(netfmt::parse_i32)
                .is_some()
                && item
                    .get(&mut self.dflog, "Length")
                    .and_then(|s| netfmt::parse_i32(netfmt::trim(s)))
                    .is_some()
                && item.get(&mut self.dflog, "Name").is_some()
                && item.get(&mut self.dflog, "Format").is_some()
                && self
                    .dflog
                    .find_message_offset("FMT", "Columns")
                    .is_some_and(|skip| item.items.len() > skip);
            if complete {
                let text = self.line(line);
                self.dflog.fmt_line(&text);
            }
        }
        for (_, item) in self.items_of(&["FMTU"]) {
            if item.items.is_empty() {
                continue;
            }
            let (Some(msg_type), Some(units), Some(multipliers)) = (
                item.get(&mut self.dflog, "FmtType")
                    .and_then(netfmt::parse_i32),
                item.get(&mut self.dflog, "UnitIds")
                    .map(|s| netfmt::trim(s).to_owned()),
                item.get(&mut self.dflog, "MultIds")
                    .map(|s| netfmt::trim(s).to_owned()),
            ) else {
                continue;
            };
            self.fmtu.insert(msg_type, (units, multipliers));
        }
        // "used to set the firmware type": the string form of each `MSG` and `PARM` line.
        let mut read = 0usize;
        for (line, _) in self.items_of(&["MSG", "PARM"]) {
            self.line(line);
            read += 1;
            if read > FIRMWARE_LINES {
                break;
            }
        }
    }

    /// `GetEnumeratorType`: every line of the named types, in line order, as items - read now, in
    /// the order the C#'s lazy enumeration reads them. Each comes with its line number.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:701-774`
    pub fn items_of(&mut self, types: &[&str]) -> Vec<(usize, DfItem)> {
        let mut lines = Vec::new();
        for name in types {
            if let Some(label) = self.dflog.label(name) {
                let msg_type = usize::from(label.id.to_le_bytes()[0]);
                lines.extend(
                    self.index_lines
                        .get(msg_type)
                        .into_iter()
                        .flatten()
                        .copied(),
                );
            }
        }
        if types.len() > 1 {
            lines.sort_unstable();
        }
        let mut out = Vec::new();
        for line in lines {
            let item = self.item(line);
            if types.contains(&item.msgtype()) {
                out.push((line, item));
            }
        }
        out
    }

    /// How many lines there are: `Count`.
    #[must_use]
    pub const fn count(&self) -> usize {
        self.count
    }

    /// Whether the log was read as binary: it opens with `A3 95`.
    #[must_use]
    pub const fn is_binary(&self) -> bool {
        self.binary
    }

    fn span(&self, index: usize) -> (usize, usize) {
        let start = self
            .line_starts
            .get(index)
            .copied()
            .unwrap_or(self.data.len());
        let end = self
            .line_starts
            .get(index + 1)
            .copied()
            .unwrap_or(self.data.len());
        (start, end)
    }

    /// `this[int]`: line `index` as text. A binary line is the first message that decodes from the
    /// line's offset on, `"\r\n"` ended; a text line keeps its newline.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:613-662`
    pub fn line(&mut self, index: usize) -> String {
        let (start, end) = self.span(index);
        if self.binary {
            let mut pos = start;
            let length = self.data.len();
            self.binlog
                .read_message(self.data, &mut pos, length, self.mode_name)
        } else {
            netfmt::ascii(self.data.get(start..end).unwrap_or_default())
        }
    }

    /// `this[long]`: line `index` as an item. A binary line is read no further than the next.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:562-611`
    fn item(&mut self, index: usize) -> DfItem {
        let (start, end) = self.span(index);
        if self.binary {
            let mut pos = start;
            let fields = self
                .binlog
                .read_message_objects(self.data, &mut pos, end, self.mode_name);
            DfItem {
                items: fields
                    .unwrap_or_default()
                    .iter()
                    .map(crate::convert::Field::item_text)
                    .collect(),
            }
        } else {
            let text = netfmt::ascii(self.data.get(start..end).unwrap_or_default());
            self.dflog.item_from_line(&text)
        }
    }

    /// `getInstanceIndex`: which field of a line of this type is its instance number - one past
    /// the `#` in its `FMTU` units, so `0` when there is none - or `-1` when the log gives no
    /// units for the type. `// C#: ExtLibs/Utilities/DFLogBuffer.cs:831-845`
    #[must_use]
    pub fn instance_index(&self, type_name: &str) -> i32 {
        let Some(label) = self.dflog.label(type_name) else {
            return -1;
        };
        let Some((units, _)) = self.fmtu.get(&label.id) else {
            return -1;
        };
        // `IndexOf("#") + 1`; the units are ASCII, so a byte index is a character index.
        units
            .find('#')
            .map_or(0, |at| i32::try_from(at + 1).unwrap_or(0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::convert::no_mode_names;

    #[test]
    fn fmt_lines_read_as_the_csharp_splits_them() {
        let mut dflog = DfLog::new();
        dflog.fmt_line("FMT, 130, 45, GPS, BIHBcLLeeEefI, Status,TimeMS,Week,NSats\r\n");
        let label = dflog.label("GPS").unwrap();
        assert_eq!(label.id, 130);
        assert_eq!(label.field_names, ["Status", "TimeMS", "Week", "NSats"]);
        assert_eq!(dflog.find_message_offset("GPS", "TimeMS"), Some(2));
        assert_eq!(dflog.find_message_offset("GPS", "Nope"), None);
        // The FMT format is supplied when a log does not declare it.
        assert!(dflog.contains("FMT"));
        // An FMTU line starts "FMT" too, and names a "format" after its unit string.
        dflog.fmt_line("FMTU, 1000, 65, s#-, F--\r\n");
        assert!(dflog.contains("s#-"));
    }

    #[test]
    fn gps_time_needs_a_fix() {
        let mut dflog = DfLog::new();
        dflog.fmt_line("FMT, 1, 2, GPS, QBIH, TimeUS,Status,GMS,GWk");
        assert_eq!(dflog.gps_time("GPS, 1, 1, 100000, 2000"), None);
        let time = dflog.gps_time("GPS, 1, 3, 100000, 2000").unwrap();
        let expected = 315_964_800_000 + 2000 * 7 * 86_400_000 + 100_000 - 18_000;
        assert_eq!(time, expected);
    }

    #[test]
    fn a_text_log_is_its_lines_with_the_last_unterminated_one_unread() {
        let data = b"FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\nX, 1\nY";
        let mut buffer = DfLogBuffer::new(data, &no_mode_names);
        assert!(!buffer.is_binary());
        assert_eq!(buffer.count(), 2);
        assert_eq!(buffer.line(1), "X, 1\n");
    }

    #[test]
    fn civil_dates() {
        assert_eq!(civil_from_days(0), (1970, 1, 1));
        assert_eq!(civil_from_days(20_681), (2026, 8, 16));
        assert_eq!(civil_from_days(-719_162), (1, 1, 1));
    }
}
