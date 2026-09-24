//! A dataflash log as `GeoRefImageBase` reads it: through `DFLogBuffer.GetEnumeratorType`, one
//! `DFLog.DFItem` at a time, each with the text of its fields and a time.
//!
//! `mp_log::dflogbuffer::DfLogBuffer` ports the buffer for the log converters, which never ask an
//! item its time. `GeoRefImageBase` does, and an item's time is not in the message: it is the
//! log's GPS start time plus the item's boot time less the boot time of the GPS message that set
//! the start, and the start is set as a side effect of *constructing* the first `GPS…` item whose
//! line carries a valid GPS week and time (`DFLog.cs:151-206`). Which item that is depends on
//! which items the buffer has constructed by then - and the buffer's own constructor constructs
//! a good many (`DFLogBuffer.cs:207-331`). So this module indexes the log the way the buffer does,
//! with `mp_log`'s `BinaryLog` for the binary framing and its `DfLog` for the format table and the
//! GPS time, and constructs items in the buffer's order so the start time lands where the C#'s
//! does.
//! `// C#: ExtLibs/Utilities/DFLogBuffer.cs; ExtLibs/Utilities/DFLog.cs`

use mp_log::convert::{BinaryLog, Field, HEAD_BYTE1, HEAD_BYTE2, no_mode_names};
use mp_log::dflogbuffer::DfLog;
use mp_log::netfmt;

use crate::time::{DateTime, Kind, OutOfRange, TICKS_PER_MILLISECOND, UNIX_EPOCH_TICKS};

/// The format message's type. `// C#: ExtLibs/Utilities/DFLogBuffer.cs:128`
const FMT_TYPE: usize = 128;

/// One `DFLog.DFItem`: its fields as the text `items` gives, `None` where the C# holds `null`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Item {
    /// `items`: the message name, then the fields.
    pub items: Vec<Option<String>>,
}

impl Item {
    /// `msgtype`: the first field, or empty.
    #[must_use]
    pub fn msgtype(&self) -> &str {
        self.items.first().and_then(|s| s.as_deref()).unwrap_or("")
    }

    /// `items[index]`, or `None` where the C# reads a `null` or throws `IndexOutOfRange`.
    #[must_use]
    pub fn field(&self, index: Option<usize>) -> Option<&str> {
        self.items.get(index?)?.as_deref()
    }
}

/// The `DFLogBuffer` constructor threw: an item too short for its instance column.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("Index was outside the bounds of the array.")]
pub struct ShortInstanceItem;

/// `DFLogBuffer`, with the part of `DFLog` that times an item.
pub struct DfBuffer<'a> {
    data: &'a [u8],
    binary: bool,
    binlog: BinaryLog,
    /// `linestartoffset`.
    line_starts: Vec<usize>,
    /// `messageindexline`: the lines of each type.
    index_lines: Vec<Vec<usize>>,
    /// `dflog`.
    pub dflog: DfLog,
    /// `gpsstarttime` in Unix milliseconds; `None` is `DateTime.MinValue`.
    gps_start: Option<i64>,
    /// `msoffset`: the boot time, in milliseconds, of the item that set the start.
    msoffset: i64,
}

impl std::fmt::Debug for DfBuffer<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DfBuffer")
            .field("binary", &self.binary)
            .field("lines", &self.line_starts.len())
            .field("gps_start", &self.gps_start)
            .finish_non_exhaustive()
    }
}

impl<'a> DfBuffer<'a> {
    /// `new DFLogBuffer(stream)`: the log indexed, its formats read, and every item the
    /// constructor builds built in its order.
    ///
    /// # Errors
    ///
    /// Where the constructor throws: an item of an instanced type too short to hold the instance
    /// column its units name (`DFLogBuffer.cs:258`, outside any `try`).
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:52-332`
    pub fn new(data: &'a [u8]) -> Result<Self, ShortInstanceItem> {
        let binary = data.first() == Some(&HEAD_BYTE1) && data.get(1) == Some(&HEAD_BYTE2);
        let mut buffer = Self {
            data,
            binary,
            binlog: BinaryLog::new(),
            line_starts: Vec::new(),
            index_lines: vec![Vec::new(); 256],
            dflog: DfLog::new(),
            gps_start: None,
            msoffset: 0,
        };
        if binary {
            buffer.index_binary();
        } else {
            buffer.index_text();
        }
        buffer.construct_as_setlinecount_does()?;
        Ok(buffer)
    }

    /// `setlinecount`'s binary half. `// C#: ExtLibs/Utilities/DFLogBuffer.cs:103-143`
    fn index_binary(&mut self) {
        let length = self.data.len();
        let mut pos = 0usize;
        let mut count = 0usize;
        while pos < length {
            let (msg_type, offset) = self
                .binlog
                .read_message_type_offset(self.data, &mut pos, length);
            if msg_type == 0 && offset == 0 {
                continue;
            }
            if let Some(lines) = self.index_lines.get_mut(usize::from(msg_type)) {
                lines.push(count);
            }
            self.line_starts.push(offset);
            count += 1;
        }
        // "build fmt line database to pre seed the FMT message": each FMT line as text.
        for line in self.index_lines.get(FMT_TYPE).cloned().unwrap_or_default() {
            let (start, _) = self.span(line);
            let mut at = start;
            let text = self
                .binlog
                .read_message(self.data, &mut at, length, &no_mode_names);
            self.dflog.fmt_line(&text);
        }
    }

    /// `setlinecount`'s text half: lines end at `\n`, the last unterminated one is never read,
    /// and a line is indexed under the type its name has once a format names it.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:144-201`
    fn index_text(&mut self) {
        self.line_starts.push(0);
        let mut count = 0usize;
        for (at, &byte) in self.data.iter().enumerate() {
            if byte == b'\n' {
                self.line_starts.push(at + 1);
                count += 1;
            }
        }
        for line in 0..count {
            let text = self.text_line(line);
            let Some(comma) = text.find(',').filter(|&at| at > 0) else {
                continue;
            };
            let msgtype = text.get(..comma).unwrap_or_default().to_owned();
            if msgtype == "FMT" {
                self.dflog.fmt_line(&text);
            }
            if let Some(label) = self.dflog.label(&msgtype) {
                let msg_type = usize::from(label.id.to_le_bytes()[0]);
                if let Some(lines) = self.index_lines.get_mut(msg_type) {
                    lines.push(line);
                }
            }
        }
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

    fn text_line(&self, index: usize) -> String {
        let (start, end) = self.span(index);
        netfmt::ascii(self.data.get(start..end).unwrap_or_default())
    }

    fn find(&mut self, linetype: &str, name: &str) -> Option<usize> {
        self.dflog.find_message_offset(linetype, name)
    }

    /// The items `setlinecount` constructs after indexing, in its order - the `FMT`s, the
    /// `FMTU`s, up to 2002 items of each type whose units name an instance column, and the
    /// `GPS`, `GPS2` and `GPSB` items up to the first with a 3D fix - for the one thing
    /// constructing an item can change here: the GPS start time.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:207-331`
    fn construct_as_setlinecount_does(&mut self) -> Result<(), ShortInstanceItem> {
        // `FMT[type] = (...)` for every complete FMT item, then `FMTLine` of it again
        // (DFLogBuffer.cs:208-228). Reading a format line again reads the same format, so only
        // each type's name is kept.
        let mut fmt_names: Vec<(i32, String)> = Vec::new();
        for line in self.lines_of(&["FMT"]) {
            let item = self.item(line);
            if !yielded(&["FMT"], item.msgtype()) {
                continue;
            }
            let ty = self.find("FMT", "Type");
            let ty = item.field(ty).and_then(netfmt::parse_i32);
            let length = self.find("FMT", "Length");
            let length_ok = item
                .field(length)
                .and_then(|s| netfmt::parse_i32(netfmt::trim(s)))
                .is_some();
            let name = self.find("FMT", "Name");
            let name = item.field(name).map(|s| netfmt::trim(s).to_owned());
            let format = self.find("FMT", "Format");
            let format_ok = item.field(format).is_some();
            let columns_ok = self
                .find("FMT", "Columns")
                .is_some_and(|skip| item.items.len() > skip);
            if let (Some(ty), true, Some(name), true, true) =
                (ty, length_ok, name, format_ok, columns_ok)
            {
                match fmt_names.iter_mut().find(|(t, _)| *t == ty) {
                    Some(slot) => slot.1 = name,
                    None => fmt_names.push((ty, name)),
                }
            }
        }
        // `InstanceType[FmtType] = (UnitIds.IndexOf("#"), ...)` for every FMTU whose unit ids
        // hold a `#`, in the order the dictionary keeps them (DFLogBuffer.cs:230-251).
        let mut instance_types: Vec<(i32, usize)> = Vec::new();
        for line in self.lines_of(&["FMTU"]) {
            let item = self.item(line);
            if !yielded(&["FMTU"], item.msgtype()) {
                continue;
            }
            let ty = self.find("FMTU", "FmtType");
            let ty = item.field(ty).and_then(netfmt::parse_i32);
            let units = self.find("FMTU", "UnitIds");
            let units = item.field(units).map(|s| netfmt::trim(s).to_owned());
            let mults = self.find("FMTU", "MultIds");
            let mults_ok = item.field(mults).is_some();
            if let (Some(ty), Some(units), true) = (ty, units, mults_ok)
                && let Some(hash) = units.find('#')
            {
                match instance_types.iter_mut().find(|(t, _)| *t == ty) {
                    Some(slot) => slot.1 = hash,
                    None => instance_types.push((ty, hash)),
                }
            }
        }
        // Up to 2002 items of each such type, each read at its instance column, which throws if
        // the item is too short (DFLogBuffer.cs:253-268).
        for (ty, hash) in instance_types {
            let Some(name) = fmt_names
                .iter()
                .find(|(t, _)| *t == ty)
                .map(|(_, n)| n.clone())
            else {
                continue;
            };
            let types = [name.as_str()];
            let mut a = 0;
            for line in self.lines_of(&types) {
                let item = self.item(line);
                if !yielded(&types, item.msgtype()) {
                    continue;
                }
                if item.items.len() <= hash + 1 {
                    return Err(ShortInstanceItem);
                }
                if a > 2000 {
                    break;
                }
                a += 1;
            }
        }
        // UNIT and MULT are read only when `Unit` and `Mult` already hold something, which at
        // this point they never do (DFLogBuffer.cs:269-293); MSG and PARM are read as text for the
        // firmware's name, which nothing here uses (DFLogBuffer.cs:295-308).
        //
        // "try get gps time" (DFLogBuffer.cs:312-328).
        let gps_types = ["GPS", "GPS2", "GPSB"];
        let mut gpsa = 0;
        for line in self.lines_of(&gps_types) {
            let item = self.item(line);
            if !yielded(&gps_types, item.msgtype()) {
                continue;
            }
            gpsa += 1;
            let status = self.dflog.find_message_offset(item.msgtype(), "Status");
            if item
                .field(status)
                .and_then(netfmt::parse_i32)
                .is_some_and(|s| s >= 3)
            {
                break;
            }
            if gpsa > 2000 {
                break;
            }
        }
        Ok(())
    }

    /// The lines `GetEnumeratorType(types)` visits: every indexed line of each type its instance
    /// map names that the format table knows, sorted when more than one type is asked for. Which
    /// of their items it then yields is [`yielded`]'s question.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:701-774`
    #[must_use]
    pub fn lines_of(&self, types: &[&str]) -> Vec<usize> {
        let mut lines = Vec::new();
        for key in instance_keys(types) {
            if let Some(label) = self.dflog.label(&key) {
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
        lines
    }

    /// `this[(long)line]`: the line's item, constructed - which, for the first `GPS…` item with a
    /// valid GPS time, sets the start time.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:562-611, DFLog.cs:151-206`
    pub fn item(&mut self, line: usize) -> Item {
        let (start, end) = self.span(line);
        let item = if self.binary {
            let mut pos = start;
            let fields = self
                .binlog
                .read_message_objects(self.data, &mut pos, end, &no_mode_names);
            Item {
                items: fields
                    .unwrap_or_default()
                    .iter()
                    .map(Field::item_text)
                    .collect(),
            }
        } else {
            let text = self.text_line(line);
            Item {
                items: self.dflog.item_from_line(&text).items,
            }
        };
        self.on_construct(&item);
        item
    }

    /// The `DFItem` constructor's GPS start time. `// C#: ExtLibs/Utilities/DFLog.cs:163-206`
    fn on_construct(&mut self, item: &Item) {
        let msgtype = item.msgtype().to_owned();
        if self.gps_start.is_some() || !msgtype.starts_with("GPS") || !self.dflog.contains(&msgtype)
        {
            return;
        }
        let joined = item
            .items
            .iter()
            .map(|s| s.as_deref().unwrap_or(""))
            .collect::<Vec<_>>()
            .join(",");
        let Some(time) = self.dflog.gps_time(&joined) else {
            return;
        };
        self.gps_start = Some(time);
        if let Some(t) = self.dflog.find_message_offset(&msgtype, "T") {
            match item.field(Some(t)).and_then(netfmt::parse_i32) {
                Some(ms) => self.msoffset = i64::from(ms),
                None => self.gps_start = None,
            }
        }
        if let Some(us) = self.dflog.find_message_offset(&msgtype, "TimeUS") {
            match item.field(Some(us)).and_then(netfmt::parse_i64) {
                Some(us) => self.msoffset = us / 1000,
                None => self.gps_start = None,
            }
        }
    }

    /// `timems`: the item's boot time in milliseconds - `TimeMS`, else `TimeUS / 1000.0`, else
    /// `T` - or `None` where `long.Parse` throws. An item of an unknown type, or with none of the
    /// three, is 0.
    /// `// C#: ExtLibs/Utilities/DFLog.cs:97-133`
    #[allow(clippy::cast_precision_loss)]
    fn timems(&mut self, item: &Item) -> Option<f64> {
        let msgtype = item.msgtype().to_owned();
        if !self.dflog.contains(&msgtype) {
            return Some(0.0);
        }
        if let Some(index) = self.dflog.find_message_offset(&msgtype, "TimeMS") {
            return Some(netfmt::parse_i64(item.field(Some(index))?)? as f64);
        }
        if let Some(index) = self.dflog.find_message_offset(&msgtype, "TimeUS") {
            return Some(netfmt::parse_i64(item.field(Some(index))?)? as f64 / 1000.0);
        }
        if let Some(index) = self.dflog.find_message_offset(&msgtype, "T") {
            return Some(netfmt::parse_i64(item.field(Some(index))?)? as f64);
        }
        Some(0.0)
    }

    /// `item.time.ToUniversalTime()`: the start time plus the item's boot time since the start,
    /// in whole ticks, and UTC whatever it was built as.
    ///
    /// # Errors
    ///
    /// Where the C# throws: a boot time `long.Parse` rejects, or a time outside `DateTime`.
    /// `// C#: ExtLibs/Utilities/DFLog.cs:42-51`
    pub fn time(&mut self, item: &Item) -> Result<DateTime, OutOfRange> {
        let timems = self.timems(item).ok_or(OutOfRange)?;
        #[allow(clippy::cast_precision_loss)]
        let offset = (timems - self.msoffset as f64) * 10000.0;
        if !offset.is_finite() {
            return Err(OutOfRange);
        }
        #[allow(clippy::cast_possible_truncation)]
        let ticks = offset as i64;
        let start = match self.gps_start {
            // gpsTimeToTime's ToLocalTime, undone by ToUniversalTime: UTC either way.
            Some(ms) => {
                DateTime::from_ticks(UNIX_EPOCH_TICKS + ms * TICKS_PER_MILLISECOND, Kind::Utc)
            }
            None => DateTime::MIN,
        };
        Ok(DateTime::from_ticks(
            start.add_ticks(ticks)?.ticks,
            Kind::Utc,
        ))
    }
}

/// The instance map `GetEnumeratorType` builds from the names asked for, as its keys in the order
/// it adds them: for each name, the word `(\w+)(\[([0-9]+)\])?` finds first, then for a name
/// ending in a digit the word before that last digit (`(\w+)([0-9]+)$`, so asking for `GPS2` also
/// asks for `GPS`). A name asked for without `[n]` asks for every instance, which is every name
/// `GeoRefImageBase` asks for, so the instance values themselves are not kept.
/// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:703-735`
fn instance_keys(types: &[&str]) -> Vec<String> {
    let is_word = |c: char| c.is_alphanumeric() || c == '_';
    let mut keys: Vec<String> = Vec::new();
    for name in types {
        let word: String = name
            .chars()
            .skip_while(|c| !is_word(*c))
            .take_while(|c| is_word(*c))
            .collect();
        let mut candidates = vec![word];
        if name.ends_with(|c: char| c.is_ascii_digit()) {
            let mut run: Vec<char> = name.chars().rev().take_while(|c| is_word(*c)).collect();
            run.reverse();
            if run.len() >= 2 {
                candidates.push(run.iter().take(run.len() - 1).collect());
            }
        }
        for key in candidates {
            if !key.is_empty() && !keys.contains(&key) {
                keys.push(key);
            }
        }
    }
    keys
}

/// Whether `GetEnumeratorType(types)` yields an item of `msgtype`: its type is one of the instance
/// map's keys. `// C#: ExtLibs/Utilities/DFLogBuffer.cs:764-772`
#[must_use]
pub fn yielded(types: &[&str], msgtype: &str) -> bool {
    instance_keys(types).iter().any(|k| k == msgtype)
}

/// `float.Parse(s, CultureInfo.InvariantCulture)`: the double `double.Parse` reads, narrowed, and
/// an overflow (a finite double past `float.MaxValue`) a failure.
#[must_use]
pub fn parse_single(text: &str) -> Option<f32> {
    let value = netfmt::parse_double(text)?;
    #[allow(clippy::cast_possible_truncation)]
    let single = value as f32;
    (single.is_finite() || !value.is_finite()).then_some(single)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn text_log() -> Vec<u8> {
        [
            "FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns",
            "FMT, 130, 45, GPS, QBIHLL, TimeUS,Status,GMS,GWk,Lat,Lng",
            "FMT, 131, 20, ATT, Qff, TimeUS,Roll,Pitch",
            "ATT, 1000000, 1.5, 2.5",
            "GPS, 1500000, 1, 0, 0, 0, 0",
            "GPS, 2000000, 3, 100000, 2000, -35.1, 149.2",
            "ATT, 2100000, 3.5, 4.5",
            "GPS, 2200789, 3, 100200, 2000, -35.2, 149.3",
            "",
        ]
        .join("\n")
        .into_bytes()
    }

    #[test]
    fn the_first_gps_fix_sets_the_start_time_while_the_buffer_loads() {
        let data = text_log();
        let mut buffer = DfBuffer::new(&data).unwrap();
        let lines = buffer.lines_of(&["GPS", "ATT"]);
        assert_eq!(lines, [3, 4, 5, 6, 7]);
        // The start was set by the constructor's GPS pass, from the first 3D fix.
        let expected_start = mp_log::dflogbuffer::gps_time_to_utc_millis(2000, 100.0);
        assert_eq!(buffer.gps_start, Some(expected_start));
        assert_eq!(buffer.msoffset, 2000);
        let item = buffer.item(7);
        let time = buffer.time(&item).unwrap();
        // 200.789 ms after the start, in ticks.
        assert_eq!(
            time.ticks,
            UNIX_EPOCH_TICKS + expected_start * TICKS_PER_MILLISECOND + 2_007_890
        );
        // A text line keeps the space after each comma.
        assert_eq!(item.field(Some(5)), Some(" -35.2"));
    }

    #[test]
    fn the_instance_map_is_the_two_regexes() {
        assert_eq!(
            instance_keys(&["GPS", "GPS2", "ATT"]),
            ["GPS", "GPS2", "ATT"]
        );
        assert_eq!(instance_keys(&["GPS2"]), ["GPS2", "GPS"]);
        assert_eq!(
            instance_keys(&["ACC12", "GPS[1]"]),
            ["ACC12", "ACC1", "GPS"]
        );
        assert!(yielded(&["GPS2"], "GPS"));
        assert!(!yielded(&["CAM", "RFND"], "TRIG"));
    }

    #[test]
    fn float_parse_overflows_where_the_single_cannot_hold_it() {
        assert_eq!(parse_single("1.5"), Some(1.5));
        assert_eq!(parse_single("1e39"), None);
        assert_eq!(parse_single("x"), None);
    }
}
