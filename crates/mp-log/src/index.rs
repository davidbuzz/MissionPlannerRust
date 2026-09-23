//! Random access to a dataflash log's records, for a grid that shows them a screenful at a time.
//!
//! Mission Planner's log browser puts every record of a log in `dataGridView1`, and a real log
//! holds hundreds of thousands. It does not hold them: `DFLogBuffer` walks the file once for the
//! byte offset each record starts at, and the grid's `CellValueNeeded` decodes a row only when
//! the grid asks to paint it. This is that walk, and the few things the grid needs from the whole
//! log before it can show any one row - the formats, which field of a message is its instance, and
//! where GPS time puts the start of the log's clock.
//!
//! **What it costs.** Nine bytes a record: an eight-byte offset and a one-byte type. The C# keeps
//! three eight-byte lists per record (`linestartoffset`, `messageindex`, `messageindexline`), so
//! this is a third of what the original spends, and nothing here grows with the size of a record.
//! `// C#: ExtLibs/Utilities/DFLogBuffer.cs:28-40, 98-126`

use std::collections::BTreeMap;

use crate::dataflash::{DataflashReader, LogMessage, MessageFormat, Value, decode_record};

/// Every record's place in a log, and what a row needs from the rest of it.
#[derive(Debug, Clone, Default)]
pub struct RecordIndex {
    /// Where each record starts, in log order. A row number is an index into this.
    offsets: Vec<u64>,
    /// Each record's message type, parallel to `offsets`, so a filter by type needs no reads.
    types: Vec<u8>,
    /// The formats, as the whole log declares them.
    formats: BTreeMap<u8, MessageFormat>,
    /// Which field of a message type carries its instance number, from the `#` in its `FMTU`.
    instance_fields: BTreeMap<u8, usize>,
    /// How many records of each type the log holds: `SeenMessageTypes`, with counts.
    counts: BTreeMap<u8, usize>,
    /// Where GPS time puts the start of the log's clock, if a GPS ever had a fix.
    gps_start: Option<GpsStart>,
}

impl RecordIndex {
    /// Walks a log once and indexes every record in it, format declarations included.
    ///
    /// The `FMTU` records and the first GPS fix are decoded on the way past, because the grid
    /// needs both before it can draw its first row; nothing else is decoded.
    #[must_use]
    pub fn build(data: &[u8]) -> Self {
        let mut index = Self::default();
        let mut reader = DataflashReader::new(data);
        while let Some(record) = reader.next_record() {
            // usize to u64 is lossless on every target this builds for; the offset is stored
            // wide because a log is a file, and files outgrow a 32-bit address space.
            index.offsets.push(record.offset as u64);
            index.types.push(record.msg_type);
            *index.counts.entry(record.msg_type).or_default() += 1;

            let name = reader
                .formats()
                .get(&record.msg_type)
                .map(|format| format.name.as_str());
            let wanted = match name {
                Some("FMTU") => true,
                // `DFItem` sets `gpsstarttime` from the first message whose type starts with
                // GPS and that has a fix; after that it never looks again.
                // `// C#: ExtLibs/Utilities/DFLog.cs:163-208`
                Some(name) => index.gps_start.is_none() && name.starts_with("GPS"),
                None => false,
            };
            if !wanted {
                continue;
            }
            let Some(message) = data
                .get(record.offset..)
                .and_then(|bytes| decode_record(reader.formats(), bytes))
            else {
                continue;
            };
            if message.name == "FMTU" {
                if let Some((format_type, position)) = instance_position(&message) {
                    index.instance_fields.insert(format_type, position);
                }
            } else {
                index.gps_start = GpsStart::from_message(&message);
            }
        }
        index.formats = reader.formats().clone();
        // Pushing doubles a vector's capacity as it goes, which would leave up to half of each
        // table unused for as long as the log is open.
        index.offsets.shrink_to_fit();
        index.types.shrink_to_fit();
        index
    }

    /// Records in the log.
    #[must_use]
    pub fn len(&self) -> usize {
        self.offsets.len()
    }

    /// Whether the log holds no records at all.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.offsets.is_empty()
    }

    /// Where a record starts.
    #[must_use]
    pub fn offset(&self, row: usize) -> Option<u64> {
        self.offsets.get(row).copied()
    }

    /// A record's message type.
    #[must_use]
    pub fn msg_type(&self, row: usize) -> Option<u8> {
        self.types.get(row).copied()
    }

    /// The formats the log declares.
    #[must_use]
    pub const fn formats(&self) -> &BTreeMap<u8, MessageFormat> {
        &self.formats
    }

    /// A format by message name: `dflog.logformat[name]`.
    #[must_use]
    pub fn format_named(&self, name: &str) -> Option<&MessageFormat> {
        self.formats.values().find(|format| format.name == name)
    }

    /// Which field of a message type is its instance number, if it has one.
    #[must_use]
    pub fn instance_field(&self, msg_type: u8) -> Option<usize> {
        self.instance_fields.get(&msg_type).copied()
    }

    /// The most fields any declared message has.
    #[must_use]
    pub fn widest(&self) -> usize {
        self.formats
            .values()
            .map(|format| format.labels.len().max(format.format.len()))
            .max()
            .unwrap_or(0)
    }

    /// The message types the log holds records of, by name, sorted: `SeenMessageTypes`.
    #[must_use]
    pub fn seen(&self) -> Vec<String> {
        let mut names: Vec<String> = self
            .counts
            .keys()
            .filter_map(|msg_type| self.formats.get(msg_type))
            .map(|format| format.name.clone())
            .collect();
        names.sort();
        names.dedup();
        names
    }

    /// The rows holding a message of one name, in log order.
    ///
    /// Row numbers are held as `u32`: four bytes a matching row rather than eight, and a log
    /// with four billion records of one type would be a file of a terabyte. One that somehow had
    /// more is cut short rather than wrapped round.
    #[must_use]
    pub fn rows_named(&self, name: &str) -> Vec<u32> {
        let wanted: Vec<u8> = self
            .formats
            .iter()
            .filter(|(_, format)| format.name == name)
            .map(|(msg_type, _)| *msg_type)
            .collect();
        self.types
            .iter()
            .enumerate()
            .filter(|(_, msg_type)| wanted.contains(msg_type))
            .map_while(|(row, _)| u32::try_from(row).ok())
            .collect()
    }

    /// Decodes a record from bytes that start at its header.
    #[must_use]
    pub fn decode(&self, bytes: &[u8]) -> Option<LogMessage> {
        decode_record(&self.formats, bytes)
    }

    /// Where GPS time puts the start of the log's clock.
    #[must_use]
    pub const fn gps_start(&self) -> Option<GpsStart> {
        self.gps_start
    }

    /// Bytes the index itself holds on the heap for its per-record tables.
    ///
    /// The number that scales with the log. Formats and counts are bounded by the 256 message
    /// types a log can declare, and are left out.
    #[must_use]
    pub fn record_bytes(&self) -> usize {
        self.offsets.capacity() * std::mem::size_of::<u64>() + self.types.capacity()
    }
}

/// Which field an `FMTU` marks as its format's instance: `InstanceType[FmtType]`.
///
/// The position of the `#` in the unit string is the position of the instance field in the
/// message.
/// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:230-245`
fn instance_position(fmtu: &LogMessage) -> Option<(u8, usize)> {
    let format_type = fmtu.field("FmtType").and_then(Value::as_f64)?;
    let position = fmtu
        .field("UnitIds")
        .and_then(Value::as_text)?
        .trim()
        .find('#')?;
    if !(0.0..=f64::from(u8::MAX)).contains(&format_type) {
        return None;
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // range-checked above
    Some((format_type as u8, position))
}

/// The instant GPS time gives the log's clock: `DFLog.gpsstarttime`, with `msoffset`.
///
/// Mission Planner writes each row's time as the GPS time of the first fix plus how long after
/// that fix the row was logged. Before any fix the log has no wall-clock time at all, and the C#
/// counts from the year 1 - see [`boot_ms`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpsStart {
    /// The GPS week of the fix.
    pub week: u32,
    /// Milliseconds into that week.
    pub week_ms: u64,
    /// The vehicle's clock at the fix, in whole milliseconds since boot.
    pub boot_ms: i64,
}

/// Unix time of the GPS epoch, 1980-01-06, in milliseconds.
const GPS_EPOCH_UNIX_MS: i64 = 315_964_800_000;

/// Milliseconds in a GPS week.
const WEEK_MS: i64 = 7 * 24 * 60 * 60 * 1000;

impl GpsStart {
    /// Reads the GPS time out of a `GPS` message, if it has a fix and a plausible time.
    ///
    /// `GetTimeGPS`: a status of 0, 1 or 2 is no 3D fix and no time; the time of week is
    /// `TimeMS` in old logs and `GMS` in new ones, the week `Week` or `GWk`; a week past 5000 or a
    /// time past the end of the week is garbage. The boot-clock offset is `TimeUS` in
    /// milliseconds, or the old `T` field.
    /// `// C#: ExtLibs/Utilities/DFLog.cs:163-208, 646-700`
    #[must_use]
    pub fn from_message(message: &LogMessage) -> Option<Self> {
        if let Some(status) = message.field("Status").and_then(Value::as_f64)
            && status < 3.0
        {
            return None;
        }
        let week_ms = message
            .field("TimeMS")
            .or_else(|| message.field("GMS"))
            .and_then(Value::as_f64)?;
        let week = message
            .field("Week")
            .or_else(|| message.field("GWk"))
            .and_then(Value::as_f64)?;
        if !(0.0..=5000.0).contains(&week) || !(0.0..=WEEK_MS as f64).contains(&week_ms) {
            return None;
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        // both range-checked just above
        let (week, week_ms) = (week as u32, week_ms as u64);
        // `msoffset = long.Parse(TimeUS) / 1000`: integer milliseconds, truncated.
        #[allow(clippy::cast_possible_truncation)] // microseconds since boot fit an i64
        let boot_ms = message
            .field("TimeUS")
            .and_then(Value::as_f64)
            .map(|time_us| (time_us as i64) / 1000)
            .or_else(|| message.field("T").and_then(Value::as_f64).map(|t| t as i64))
            .unwrap_or(0);
        Some(Self {
            week,
            week_ms,
            boot_ms,
        })
    }

    /// The fix's time as Unix milliseconds, UTC.
    ///
    /// `gpsTimeToTime`: the GPS epoch, plus the weeks, plus the time of week, less the leap
    /// seconds GPS time has gained on UTC. The C# asks for the leap seconds of *today's* date
    /// rather than the log's, so a log from 2016 read now is a second out; the caller decides
    /// which date to ask about, and [`leap_seconds_gps`] answers.
    /// `// C#: ExtLibs/Utilities/DFLog.cs:702-711`
    #[must_use]
    pub fn unix_ms(&self, leap_seconds: i64) -> i64 {
        GPS_EPOCH_UNIX_MS
            + i64::from(self.week) * WEEK_MS
            + i64::try_from(self.week_ms).unwrap_or(0)
            - leap_seconds * 1000
    }
}

/// Seconds GPS time is ahead of UTC in a given month: `LeapSecondsGPS`.
///
/// The TAI-UTC table less the nineteen seconds GPS time was already behind TAI at its epoch. The
/// C#'s table stops at 2017, as the leap seconds have so far.
/// `// C#: ExtLibs/Utilities/rtcm3.cs:715-735`
#[must_use]
pub fn leap_seconds_gps(year: i32, month: u32) -> i64 {
    let yyyymm = i64::from(year) * 100 + i64::from(month);
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

/// A record's time on the vehicle's clock, in milliseconds since boot: `DFItem.timems`.
///
/// `TimeMS` if the message has one, else `TimeUS` in milliseconds, else the old `T`. A message
/// with none of them - `FMT`, say - has no time, and the C# then uses zero.
/// `// C#: ExtLibs/Utilities/DFLog.cs:98-131`
#[must_use]
pub fn boot_ms(message: &LogMessage) -> Option<f64> {
    if let Some(ms) = message.field("TimeMS").and_then(Value::as_f64) {
        return Some(ms);
    }
    if let Some(us) = message.field("TimeUS").and_then(Value::as_f64) {
        return Some(us / 1000.0);
    }
    message.field("T").and_then(Value::as_f64)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{fmt, record};

    fn fixture() -> Vec<u8> {
        let path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/dataflash.bin");
        std::fs::read(&path).unwrap_or_else(|err| panic!("reading {}: {err}", path.display()))
    }

    /// Every record is indexed, the format declarations among them, and in log order.
    ///
    /// The number is what a byte-level walk of the fixture finds: 169 `FMT` records and 11,270
    /// data records. `next_message` sees the second figure; the grid shows both, as the C# grid
    /// does, because an `FMT` is a line of the log like any other.
    #[test]
    fn the_real_log_indexes_every_record_format_declarations_included() {
        let data = fixture();
        let index = RecordIndex::build(&data);
        assert_eq!(index.len(), 11_439);

        let mut messages = 0usize;
        let mut reader = DataflashReader::new(&data);
        while reader.next_message().is_some() {
            messages += 1;
        }
        assert_eq!(
            index.len() - messages,
            169,
            "the difference is exactly the FMTs"
        );
        assert!(
            index.offsets.windows(2).all(|pair| pair[0] < pair[1]),
            "offsets run forward"
        );
        assert_eq!(index.offset(0), Some(0), "a healthy log starts at byte 0");
    }

    /// A row is decoded from its own bytes and nothing else, against the whole log's formats.
    #[test]
    fn a_row_decodes_from_its_offset_alone() {
        let data = fixture();
        let index = RecordIndex::build(&data);

        // Row 0 is the log declaring FMT itself, decoded with that declaration.
        let first = index
            .decode(&data[usize::try_from(index.offset(0).unwrap()).unwrap()..])
            .unwrap();
        assert_eq!(first.name, "FMT");
        assert_eq!(first.field("Name"), Some(&Value::Text("FMT".to_owned())));

        // Every ATT the reader finds in order, the index finds by offset.
        let mut reader = DataflashReader::new(&data);
        let walked: Vec<LogMessage> = std::iter::from_fn(|| reader.next_message())
            .filter(|message| message.name == "ATT")
            .collect();
        let rows = index.rows_named("ATT");
        assert_eq!(rows.len(), walked.len());
        assert_eq!(rows.len(), 182);
        for (row, expected) in rows.iter().zip(&walked) {
            let offset = usize::try_from(index.offset(*row as usize).unwrap()).unwrap();
            assert_eq!(index.decode(&data[offset..]).as_ref(), Some(expected));
        }
    }

    /// The instance field comes from the `#` in `FMTU`, as a position in the message.
    #[test]
    fn the_instance_field_is_the_hash_in_fmtu() {
        let index = RecordIndex::build(&fixture());
        let vibe = index.format_named("VIBE").unwrap();
        let field = index.instance_field(vibe.msg_type).unwrap();
        assert_eq!(vibe.labels[field], "IMU");
        let att = index.format_named("ATT").unwrap();
        assert_eq!(
            index.instance_field(att.msg_type),
            None,
            "ATT has one instance"
        );
    }

    /// The seen types are the ones with records, by name, sorted - not every format declared.
    #[test]
    fn the_seen_types_are_the_ones_with_records() {
        let index = RecordIndex::build(&fixture());
        let seen = index.seen();
        assert!(seen.windows(2).all(|pair| pair[0] < pair[1]), "{seen:?}");
        for name in ["ATT", "FMT", "GPS", "VIBE", "CMD"] {
            assert!(seen.iter().any(|seen| seen == name), "{name} in {seen:?}");
        }
        // GPA is declared and has records; UBX1 is declared and has none.
        assert!(!seen.iter().any(|seen| seen == "UBX1"), "{seen:?}");
        assert_eq!(index.widest(), 16);
    }

    /// Nine bytes a record, and nothing that grows with a record's size.
    #[test]
    fn the_index_costs_nine_bytes_a_record() {
        let index = RecordIndex::build(&fixture());
        // `shrink_to_fit` may leave a little over, by the allocator's leave; not a doubling.
        assert!(index.record_bytes() >= 9 * index.len());
        assert!(
            index.record_bytes() <= 9 * index.len() + 64,
            "{} bytes for {} records",
            index.record_bytes(),
            index.len()
        );
    }

    /// The fixture's GPS never gets a fix, so there is no GPS time to anchor the clock.
    #[test]
    fn a_log_with_no_fix_has_no_gps_start() {
        assert_eq!(RecordIndex::build(&fixture()).gps_start(), None);
    }

    /// Arbitrary bytes index without panicking, and the index never outnumbers the bytes.
    #[test]
    fn garbage_indexes_to_nothing_and_does_not_panic() {
        for pattern in [
            vec![0xA3u8; 4096],
            [0xA3u8, 0x95, 0x80].repeat(1024),
            [0xA3u8, 0x95, 0xFF].repeat(1024),
            (0..=255u8).cycle().take(10_000).collect(),
        ] {
            let index = RecordIndex::build(&pattern);
            assert!(index.len() <= pattern.len());
            for row in 0..index.len() {
                let offset = usize::try_from(index.offset(row).unwrap()).unwrap();
                let _ = index.decode(&pattern[offset..]);
            }
        }
    }

    const GPS: u8 = 140;

    /// A GPS record: `QBBIHBcLLeffffB`, the layout of the fixture's GPS.
    fn gps(time_us: u64, status: u8, week_ms: u32, week: u16, lat: f64, lng: f64) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.push(0); // I
        payload.push(status);
        payload.extend(week_ms.to_le_bytes());
        payload.extend(week.to_le_bytes());
        payload.push(10); // NSats
        payload.extend(90i16.to_le_bytes()); // HDop, hundredths
        #[allow(clippy::cast_possible_truncation)]
        payload.extend(((lat * 1e7) as i32).to_le_bytes());
        #[allow(clippy::cast_possible_truncation)]
        payload.extend(((lng * 1e7) as i32).to_le_bytes());
        payload.extend(58_400i32.to_le_bytes()); // Alt, hundredths
        for _ in 0..4 {
            payload.extend(0f32.to_le_bytes());
        }
        payload.push(1); // U
        record(GPS, &payload)
    }

    fn gps_log(records: &[Vec<u8>]) -> Vec<u8> {
        let mut log = fmt(
            GPS,
            51,
            "GPS",
            "QBBIHBcLLeffffB",
            "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,Yaw,U",
        );
        for bytes in records {
            log.extend_from_slice(bytes);
        }
        log
    }

    /// The first 3D fix sets the clock; a GPS with no fix before it does not.
    #[test]
    fn the_first_fix_anchors_the_clock() {
        let log = gps_log(&[
            gps(1_000_000, 1, 0, 0, 0.0, 0.0),
            gps(2_500_700, 3, 345_600_000, 2_300, -35.363, 149.165),
            gps(3_000_000, 3, 345_600_500, 2_300, -35.363, 149.165),
        ]);
        let start = RecordIndex::build(&log).gps_start().unwrap();
        assert_eq!(
            start,
            GpsStart {
                week: 2_300,
                week_ms: 345_600_000,
                boot_ms: 2_500,
            },
            "TimeUS 2,500,700 is 2,500 whole milliseconds"
        );
        // Week 2300 began 2024-02-04; four days in is 2024-02-08, less 18 leap seconds.
        assert_eq!(start.unix_ms(18), 1_707_350_400_000 - 18_000);
    }

    /// A week or a time of week outside what GPS can say is not a time.
    #[test]
    fn an_impossible_gps_time_is_refused() {
        let message = |week: u16, week_ms: u32| {
            let log = gps_log(&[gps(0, 3, week_ms, week, 0.0, 0.0)]);
            RecordIndex::build(&log).gps_start()
        };
        assert!(message(2_300, 1_000).is_some());
        assert_eq!(message(6_000, 1_000), None, "week past 5000");
        assert_eq!(
            message(2_300, 700_000_000),
            None,
            "past the end of the week"
        );
    }

    /// The leap second table, as the C# has it.
    #[test]
    fn leap_seconds_follow_the_table() {
        assert_eq!(leap_seconds_gps(2026, 9), 18);
        assert_eq!(leap_seconds_gps(2017, 1), 18);
        assert_eq!(leap_seconds_gps(2016, 12), 17);
        assert_eq!(leap_seconds_gps(2012, 7), 16);
        assert_eq!(
            leap_seconds_gps(1990, 1),
            -19,
            "the C# has no entry before 1996"
        );
    }

    /// A message's boot time prefers `TimeMS`, then `TimeUS`, then `T`.
    #[test]
    fn boot_time_reads_whichever_field_the_message_has() {
        let message = |fields: &[(&str, Value)]| LogMessage {
            name: "X".to_owned(),
            fields: fields
                .iter()
                .map(|(label, value)| ((*label).to_owned(), value.clone()))
                .collect(),
        };
        assert_eq!(
            boot_ms(&message(&[("TimeUS", Value::Uint(52_351_828))])),
            Some(52_351.828)
        );
        assert_eq!(
            boot_ms(&message(&[("TimeMS", Value::Uint(1_500))])),
            Some(1_500.0)
        );
        assert_eq!(boot_ms(&message(&[("T", Value::Uint(9))])), Some(9.0));
        assert_eq!(boot_ms(&message(&[("Type", Value::Uint(9))])), None);
    }
}
