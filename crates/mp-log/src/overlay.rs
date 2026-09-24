//! What Mission Planner's log browser writes over its chart, and where its cursor finds a place.
//!
//! `LogBrowse` labels its chart with the log's flight mode changes (`DrawModes`), its errors
//! (`DrawErrors`), its text messages (`DrawMSG`), its events (`DrawEV`) and, when the x axis is
//! line numbers, a label for each minute of GPS time (`DrawTime`). A double click on the chart puts
//! a cursor on a record and a marker on the map where the nearest GPS or `POS` record says the
//! vehicle was (`GoToSample`, `GetGPSFromRow`), finding the record for a clicked time with
//! `GetLineNoFromTime`. This is the reading half of all of that - which records, what each label
//! says, which record a click lands on and which place it names - with no chart and no map in it.
//!
//! Line numbers are `DFItem.lineno`: records counted from the start of the log, format
//! declarations included, which is the grid's row number and what `RecordIndex` counts.
//! `// C#: Log/LogBrowse.cs:1731-2183, 3278-3420, 3464-3523; ExtLibs/Utilities/DFLog.cs:736-753`

use std::collections::BTreeMap;

use crate::dataflash::{DataflashReader, LogMessage, MessageFormat, Value, decode_record};

/// The field that places a record on the time axis.
const TIME_FIELD: &str = "TimeUS";

/// One label over the chart.
#[derive(Debug, Clone, PartialEq)]
pub struct Mark {
    /// The record it labels: `DFItem.lineno`.
    pub line: usize,
    /// The record's `TimeUS`, which is where it sits when the x axis is time.
    pub time_us: Option<f64>,
    /// What the label says.
    pub text: String,
}

/// A change of flight mode: one `MODE` record.
#[derive(Debug, Clone, PartialEq)]
pub struct ModeChange {
    /// Where it is and what it says: the mode's name, or its number when the name is unknown.
    pub mark: Mark,
    /// `ModeNum`, which picks the colour of the band `DrawModes` draws up to the next change.
    pub number: i64,
}

/// Which vehicle a log came from, as `BinaryLog` guesses it from the log's own text.
///
/// The guess decides which table names a flight mode: mode 4 is Guided on a copter and Acro on a
/// plane. `BinaryLog` makes it from every `MSG` and `PARM` line `DFLogBuffer` converts to text
/// when a log is opened, and the last line that says anything wins.
/// `// C#: ExtLibs/Utilities/BinaryLog.cs:178-203; ExtLibs/Utilities/DFLogBuffer.cs:295-309`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Firmware {
    /// `ArduCopter2`.
    Copter,
    /// `ArduPlane`.
    Plane,
    /// `ArduRover`.
    Rover,
    /// `ArduTracker`.
    Tracker,
}

/// Everything `LogBrowse` can draw over its chart, whether or not its box is ticked.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Overlays {
    /// The vehicle the log's text names, if it names one.
    pub firmware: Option<Firmware>,
    /// `DrawModes`: a label per mode change, and the band behind the curves up to the next.
    pub modes: Vec<ModeChange>,
    /// `DrawErrors`: `Err: SUBSYSTEM-code` per `ERR` record.
    pub errors: Vec<Mark>,
    /// `DrawMSG`: the text of each `MSG` record.
    pub messages: Vec<Mark>,
    /// `DrawEV`: `EV: NAME` per `EV` record.
    pub events: Vec<Mark>,
    /// `DrawTime`: `N min` on the first `GPS` record of each new minute, drawn only when the x
    /// axis is line numbers.
    pub minutes: Vec<Mark>,
}

/// How many `MSG` and `PARM` lines the firmware guess reads before it stops.
///
/// `limitcount` is checked after the line is converted, so one more than this is read.
/// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:295-309`
pub(crate) const FIRMWARE_LINES: usize = 100_000;

/// The firmware one `MSG` or `PARM` record's line names, if it names one.
pub(crate) fn firmware_named(message: &LogMessage) -> Option<Firmware> {
    guess_firmware(&line_text(message))
}

/// Reads everything `LogBrowse` draws over its chart out of a log.
///
/// `mode_name` is `BinaryLog.onFlightMode`: given the firmware and a mode number, the name, or
/// `None` when it has none - and then the number is the label, as the C# falls back to it. It is
/// asked only for a `Mode` field whose format character is `M`, which is the only one `BinaryLog`
/// names; a mode logged as text is its own label.
///
/// A record is skipped, not guessed at, wherever the C# skips it: a type the log declares no
/// format for, a format without the field the label is made of, a record too short to hold it.
/// `// C#: Log/LogBrowse.cs:1731-1817, 1819-1897, 1900-2018, 2020-2089, 2091-2183`
#[must_use]
pub fn overlays(data: &[u8], mode_name: impl Fn(Firmware, u64) -> Option<String>) -> Overlays {
    let mut walk = OverlayWalk::default();
    let mut reader = DataflashReader::new(data);
    let mut line = 0usize;
    while let Some(record) = reader.next_record() {
        let this_line = line;
        line += 1;
        let Some(format) = reader.formats().get(&record.msg_type) else {
            continue;
        };
        if !OVERLAY_TYPES.contains(&format.name.as_str()) {
            continue;
        }
        let Some(message) = data
            .get(record.offset..)
            .and_then(|bytes| decode_record(reader.formats(), bytes))
        else {
            continue;
        };
        walk.record(this_line, format, &message);
        if matches!(message.name.as_str(), "MSG" | "PARM") {
            walk.firmware_line(&message);
        }
    }
    walk.finish(mode_name)
}

/// The message types [`overlays`] reads.
pub(crate) const OVERLAY_TYPES: [&str; 6] = ["MODE", "ERR", "EV", "MSG", "PARM", "GPS"];

/// What [`overlays`] gathers as it goes, one record at a time in log order.
///
/// Fed by the walk over every record, or by [`crate::logfile::LogFile`] with only the records of
/// [`OVERLAY_TYPES`], read through the index; either way the same records in the same order, and
/// so the same labels. The firmware guess is fed the `MSG` and `PARM` lines by
/// [`Self::firmware_line`], or settled some other way and given to [`Self::firmware`].
#[derive(Debug, Default)]
pub(crate) struct OverlayWalk {
    /// What is finished as it is read.
    overlays: Overlays,
    /// Mode changes, named only once the firmware guess is final.
    raw_modes: Vec<(Mark, ModeValue, i64)>,
    /// `MSG` and `PARM` lines the firmware guess has read.
    firmware_lines: usize,
    /// `DrawTime`'s state.
    clock: MinuteClock,
}

impl OverlayWalk {
    /// One record of a type in [`OVERLAY_TYPES`], decoded with the format it was logged under.
    pub(crate) fn record(&mut self, line: usize, format: &MessageFormat, message: &LogMessage) {
        let time_us = message.field(TIME_FIELD).and_then(Value::as_f64);
        let at = |text: String| Mark {
            line,
            time_us,
            text,
        };
        match message.name.as_str() {
            "MODE" => {
                if let Some((value, number)) = mode_fields(format, message) {
                    self.raw_modes.push((at(String::new()), value, number));
                }
            }
            "ERR" => {
                if let Some(text) = error_text(message) {
                    self.overlays.errors.push(at(text));
                }
            }
            "EV" => {
                if let Some(id) = message.field("Id").and_then(whole) {
                    self.overlays
                        .events
                        .push(at(format!("EV: {}", event_name(id))));
                }
            }
            "MSG" => {
                if let Some(value) = message.field("Message") {
                    self.overlays
                        .messages
                        .push(at(text_of(value).trim().to_owned()));
                }
            }
            "GPS" => {
                let minute = minute_field(format).and_then(|(index, micro)| {
                    Some((micro, message.fields.get(index)?.1.as_f64()?))
                });
                self.gps(line, time_us, minute);
            }
            _ => {}
        }
    }

    /// One `MSG` or `PARM` line, for the firmware guess: the last of the first
    /// [`FIRMWARE_LINES`] and one that names a vehicle names it.
    pub(crate) fn firmware_line(&mut self, message: &LogMessage) {
        if self.reading_firmware() {
            self.firmware_lines += 1;
            if let Some(found) = firmware_named(message) {
                self.overlays.firmware = Some(found);
            }
        }
    }

    /// The firmware guess, settled without [`Self::firmware_line`].
    pub(crate) const fn firmware(&mut self, firmware: Option<Firmware>) {
        self.overlays.firmware = firmware;
    }

    /// One `GPS` record, by the two things `DrawTime` reads of it: its `TimeUS`, and the time
    /// [`minute_field`] names, with whether that is microseconds - `None` where the record has
    /// no such number.
    pub(crate) fn gps(&mut self, line: usize, time_us: Option<f64>, minute: Option<(bool, f64)>) {
        if let Some((micro, value)) = minute
            && let Some(text) = self.clock.tick(micro, value)
        {
            self.overlays.minutes.push(Mark {
                line,
                time_us,
                text,
            });
        }
    }

    /// Whether the firmware guess still reads `MSG` and `PARM` lines: once it has read its
    /// limit, a `PARM` record changes nothing and need not be read at all.
    pub(crate) const fn reading_firmware(&self) -> bool {
        self.firmware_lines <= FIRMWARE_LINES
    }

    /// The labels, with every mode named.
    pub(crate) fn finish(self, mode_name: impl Fn(Firmware, u64) -> Option<String>) -> Overlays {
        let mut overlays = self.overlays;
        // `BinaryLog` names a mode as the record is converted, which `DrawModes` does after the
        // whole log's `MSG` and `PARM` lines have been read for the guess - so every mode is
        // named against the final guess, not the one in force where the mode was logged.
        overlays.modes = self
            .raw_modes
            .into_iter()
            .map(|(mut mark, value, number)| {
                mark.text = match value {
                    ModeValue::Numbered(mode) => overlays
                        .firmware
                        .and_then(|firmware| mode_name(firmware, mode))
                        .unwrap_or_else(|| mode.to_string()),
                    ModeValue::Written(text) => text,
                }
                .trim()
                .to_owned();
                ModeChange { mark, number }
            })
            .collect();
        overlays
    }
}

/// A `MODE` record's `Mode` field, as `BinaryLog` turns it into a label.
#[derive(Debug, Clone, PartialEq)]
enum ModeValue {
    /// Logged as a number with the `M` format character: named by `onFlightMode`.
    Numbered(u64),
    /// Anything else, which is written as it is.
    Written(String),
}

/// The mode and its number, if the record has both fields `DrawModes` needs.
///
/// A record too short to hold the mode is skipped, as the C# skips it. `int.Parse(ModeNum)` of
/// something that is not a whole number would throw and end the C#'s walk; here that one record
/// is skipped.
/// `// C#: Log/LogBrowse.cs:1923-1951`
fn mode_fields(format: &MessageFormat, message: &LogMessage) -> Option<(ModeValue, i64)> {
    let position = format.labels.iter().position(|label| label == "Mode")?;
    let (_, value) = message.fields.get(position)?;
    let number = message.field("ModeNum").and_then(whole)?;
    let code = format.format.as_bytes().get(position).copied();
    let value = match (code, value) {
        (Some(b'M'), Value::Uint(mode)) => ModeValue::Numbered(*mode),
        (_, value) => ModeValue::Written(text_of(value)),
    };
    Some((value, number))
}

/// `Err: SUBSYSTEM-code`, if the record has the two fields.
/// `// C#: Log/LogBrowse.cs:1764-1790`
fn error_text(message: &LogMessage) -> Option<String> {
    let subsystem = message.field("Subsys").and_then(whole)?;
    let code = message.field("ECode")?;
    Some(format!(
        "Err: {}-{}",
        subsystem_name(subsystem),
        text_of(code).trim()
    ))
}

/// A field that holds a whole number, as `int.Parse` of its text reads it.
fn whole(value: &Value) -> Option<i64> {
    match value {
        Value::Int(value) => Some(*value),
        Value::Uint(value) => i64::try_from(*value).ok(),
        Value::Float(value) if value.fract() == 0.0 && value.abs() < 9.0e15 => {
            #[allow(clippy::cast_possible_truncation)] // whole and range-checked just above
            Some(*value as i64)
        }
        _ => None,
    }
}

/// A field as `DFItem.items` writes it: numbers in the invariant culture, bytes as ASCII with the
/// NULs trimmed. `// C#: ExtLibs/Utilities/DFLog.cs:79-96`
fn text_of(value: &Value) -> String {
    match value {
        Value::Int(value) => value.to_string(),
        Value::Uint(value) => value.to_string(),
        Value::Float(value) => value.to_string(),
        Value::Text(text) => text.clone(),
        Value::Bytes(bytes) => String::from_utf8_lossy(bytes).trim_matches('\0').to_owned(),
        Value::Samples(samples) => format!("{samples:?}"),
    }
}

/// A record as the text line `BinaryLog.ReadMessage` makes of it: the name and every field,
/// joined by `", "`. `// C#: ExtLibs/Utilities/BinaryLog.cs:156-176`
fn line_text(message: &LogMessage) -> String {
    let mut line = message.name.clone();
    for (_, value) in &message.fields {
        line.push_str(", ");
        line.push_str(&text_of(value));
    }
    line
}

/// The firmware one line of text names, by `BinaryLog`'s rules and in its order.
/// `// C#: ExtLibs/Utilities/BinaryLog.cs:178-203`
fn guess_firmware(line: &str) -> Option<Firmware> {
    // The C#'s second test, `PARM, H_SWASH_PLATE` or `ArduCopter`, names a copter too, and is
    // folded into the first.
    if line.contains("PARM, RATE_RLL_P")
        || line.contains("ArduCopter")
        || line.contains("Copter")
        || line.contains("PARM, H_SWASH_PLATE")
    {
        Some(Firmware::Copter)
    } else if line.contains("PARM, PTCH2SRV_P")
        || line.contains("ArduPlane")
        || line.contains("Plane")
    {
        Some(Firmware::Plane)
    } else if line.contains("PARM, SKID_STEER_OUT")
        || line.contains("ArduRover")
        || line.contains("Rover")
    {
        Some(Firmware::Rover)
    } else if line.contains("AntennaTracker") || line.contains("Tracker") {
        Some(Firmware::Tracker)
    } else {
        None
    }
}

/// `DrawTime`'s walk over the `GPS` records: a label on the first record of each new minute.
///
/// The minute is the `Minute` of a `DateTime` counted from `DateTime.MinValue` - so the first
/// record, at minute 0, gets no label, and the sixtieth minute is labelled again because the
/// component has wrapped to 0 - and the label is the whole elapsed time in minutes, rounded.
/// `TimeMS` is read if the format has it, else `TimeUS`; either is divided by a thousand whenever
/// the format has a `TimeUS`, as the C# decides by that and not by which it read. The subtraction
/// is unsigned: a time that goes backwards wraps it to something `AddMilliseconds` refuses, and
/// that exception ends `DrawTime` there.
/// `// C#: Log/LogBrowse.cs:2091-2183`
#[derive(Debug, Default)]
struct MinuteClock {
    /// `startdelta`: the first time read that was not zero.
    start: u64,
    /// `lastdrawn.Minute`.
    last_minute: i64,
    /// Whether an exception has ended the walk.
    stopped: bool,
}

/// `DateTime.MaxValue` in milliseconds from `DateTime.MinValue`: what `AddMilliseconds` allows.
const MAX_MILLIS: i64 = 315_537_897_600_000;

/// Which field of a `GPS` format `DrawTime` reads its time from - `TimeMS` if the format has it,
/// else `TimeUS` - and whether the format has a `TimeUS`, which decides the unit.
/// `// C#: Log/LogBrowse.cs:2110-2130`
pub(crate) fn minute_field(format: &MessageFormat) -> Option<(usize, bool)> {
    let position = |name: &str| format.labels.iter().position(|label| label == name);
    let micro = position("TimeUS");
    Some((position("TimeMS").or(micro)?, micro.is_some()))
}

impl MinuteClock {
    /// One `GPS` record's time, read from its [`minute_field`]; the label to draw on it, if it
    /// starts a new minute.
    fn tick(&mut self, micro: bool, value: f64) -> Option<String> {
        if self.stopped {
            return None;
        }
        // `UInt64.TryParse` of the number written out: a whole, non-negative one only.
        if !(value >= 0.0 && value.fract() == 0.0 && value < 1.8e19) {
            return None;
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // checked just above
        let time = value as u64;
        if self.start == 0 {
            self.start = time;
        }
        #[allow(clippy::cast_precision_loss)] // as the C#'s double arithmetic loses it
        let elapsed = if micro {
            time.wrapping_sub(self.start) as f64 / 1000.0
        } else {
            time.wrapping_sub(self.start) as f64
        };
        // `DateTime.AddMilliseconds` rounds to the nearest millisecond and throws past the end of
        // the calendar.
        let millis = elapsed.round();
        if millis >= MAX_MILLIS as f64 {
            self.stopped = true;
            return None;
        }
        #[allow(clippy::cast_possible_truncation)] // bounded by MAX_MILLIS just above
        let millis = millis as i64;
        let minute = (millis / 60_000) % 60;
        if minute == self.last_minute {
            return None;
        }
        self.last_minute = minute;
        #[allow(clippy::cast_precision_loss)] // bounded by MAX_MILLIS
        let minutes = (millis as f64 / 60_000.0).round();
        Some(format!("{minutes} min"))
    }
}

/// A record the cursor can find a place in: one whose name starts `GPS` or `POS`.
#[derive(Debug, Clone, PartialEq)]
pub struct PositionRecord {
    /// Its line.
    pub line: usize,
    /// Its message name.
    pub name: String,
    /// Its `TimeUS`.
    pub time_us: Option<f64>,
    /// Latitude and longitude, where `GetGPSFromRow` would accept this record: a `GPS` one needs
    /// a `Status` of 3 or better. `None` where it would not.
    pub place: Option<(f64, f64)>,
}

/// Where `GetLineNoFromTime` puts a time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineAtTime {
    /// The first `GPS`, `GPS2` or `POS` record at or after the time.
    Line(usize),
    /// Every such record is before it: `long.MaxValue`, which the C#'s `(int)` cast makes -1.
    AfterAll,
    /// The log has no such record: 0.
    NoPositions,
}

/// The records a double click on the chart can land on and find a place in.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Positions {
    /// Every record whose name starts `GPS` or `POS`, in log order.
    records: Vec<PositionRecord>,
    /// Records in the whole log: `logdata.Count`.
    lines: usize,
    /// Whether the log declares a `GPS` or a `POS` format at all.
    declared: bool,
}

/// How far `GetGPSFromRow` looks either way for a position record.
const SEARCH: usize = 1000;

impl Positions {
    /// Reads a log's position records.
    ///
    /// A `GPS`-prefixed record is read with the `GPS` format's field positions and a
    /// `POS`-prefixed one with the `POS` format's, whatever its own format says - `GetGPSFromRow`
    /// asks `FindMessageOffset("GPS", ...)` for `GPS2` and `GPSB` lines too.
    /// `// C#: Log/LogBrowse.cs:3374-3412`
    #[must_use]
    pub fn read(data: &[u8]) -> Self {
        let mut records = Vec::new();
        let mut reader = DataflashReader::new(data);
        let mut line = 0usize;
        while let Some(record) = reader.next_record() {
            let this_line = line;
            line += 1;
            let Some(name) = reader
                .formats()
                .get(&record.msg_type)
                .map(|format| format.name.clone())
            else {
                continue;
            };
            if !(name.starts_with("GPS") || name.starts_with("POS")) {
                continue;
            }
            let Some(message) = data
                .get(record.offset..)
                .and_then(|bytes| decode_record(reader.formats(), bytes))
            else {
                continue;
            };
            let place = place_of(reader.formats(), &message);
            records.push(PositionRecord {
                line: this_line,
                name,
                time_us: message.field(TIME_FIELD).and_then(Value::as_f64),
                place,
            });
        }
        let declared = reader
            .formats()
            .values()
            .any(|format| format.name == "GPS" || format.name == "POS");
        Self {
            records,
            lines: line,
            declared,
        }
    }

    /// The records a log's position records make, read some other way than by [`Self::read`]:
    /// every one whose name starts `GPS` or `POS`, in log order; the number of records in the
    /// log; and whether it declares a `GPS` or `POS` format.
    pub(crate) const fn from_parts(
        records: Vec<PositionRecord>,
        lines: usize,
        declared: bool,
    ) -> Self {
        Self {
            records,
            lines,
            declared,
        }
    }

    /// The records read.
    #[must_use]
    pub fn records(&self) -> &[PositionRecord] {
        &self.records
    }

    /// `GetGPSFromRow`: the place the position record nearest a line names, if it names one.
    ///
    /// The nearest `GPS` or `POS` record within a thousand lines either way; forwards only if it
    /// is strictly nearer. Reproduced as it behaves, including where it reads oddly: when nothing
    /// is found looking back, `roffset` is still its starting -1000, so the C# goes a thousand
    /// lines *forward* and reads whatever record is there - even when it found one a few lines
    /// ahead. The owner can rule otherwise; this is the site.
    /// `// C#: Log/LogBrowse.cs:3329-3420`
    #[must_use]
    pub fn from_row(&self, line: usize) -> Option<(f64, f64)> {
        if line >= self.lines || !self.declared {
            return None;
        }
        // The first record at or after the line, and the last at or before it.
        let after = self.records.partition_point(|record| record.line < line);
        let forward = self
            .records
            .get(after)
            .map(|record| record.line - line)
            .filter(|distance| *distance < SEARCH);
        let before = self.records.partition_point(|record| record.line <= line);
        let backward = before
            .checked_sub(1)
            .and_then(|index| self.records.get(index))
            .map(|record| line - record.line)
            .filter(|distance| *distance < SEARCH);

        let offset = forward.map_or(SEARCH as i64, |distance| distance as i64);
        let roffset = backward.map_or(-(SEARCH as i64), |distance| distance as i64);
        let target = if offset < roffset {
            line as i64 + offset
        } else {
            line as i64 - roffset
        };
        let target = usize::try_from(target)
            .ok()
            .filter(|target| *target < self.lines)?;
        self.records
            .binary_search_by_key(&target, |record| record.line)
            .ok()
            .and_then(|index| self.records.get(index))
            .and_then(|record| record.place)
    }

    /// `GetLineNoFromTime`: the first `GPS`, `GPS2` or `POS` record at or after a time.
    ///
    /// "Always forwards": the first in log order, so a log whose clock restarts part way through
    /// answers from before the restart where it can.
    /// `// C#: ExtLibs/Utilities/DFLog.cs:736-753`
    #[must_use]
    pub fn line_at_time(&self, time_us: f64) -> LineAtTime {
        let mut any = false;
        for record in &self.records {
            if !matches!(record.name.as_str(), "GPS" | "GPS2" | "POS") {
                continue;
            }
            any = true;
            if record.time_us.is_some_and(|time| time >= time_us) {
                return LineAtTime::Line(record.line);
            }
        }
        if any {
            LineAtTime::AfterAll
        } else {
            LineAtTime::NoPositions
        }
    }

    /// The time of a position record's line, if the line is one.
    #[must_use]
    pub fn time_of(&self, line: usize) -> Option<f64> {
        self.records
            .binary_search_by_key(&line, |record| record.line)
            .ok()
            .and_then(|index| self.records.get(index))
            .and_then(|record| record.time_us)
    }
}

/// Where a position record says the vehicle was, if `GetGPSFromRow` would accept it.
///
/// Fields are found by their position in the `GPS` or `POS` format, as `FindMessageOffset` finds
/// them. A `GPS` line needs the format to have a `Status`, and a status of 3 or more.
/// `// C#: Log/LogBrowse.cs:3374-3412`
fn place_of(formats: &BTreeMap<u8, MessageFormat>, message: &LogMessage) -> Option<(f64, f64)> {
    let family = family_of(&message.name);
    let format = formats.values().find(|format| format.name == family)?;
    place_in(family, |name| {
        let position = format.labels.iter().position(|label| label == name)?;
        message.fields.get(position)?.1.as_f64()
    })
}

/// Which format a position record's fields are found in: `GPS` for any name that starts so, and
/// `POS` for the rest.
pub(crate) fn family_of(name: &str) -> &'static str {
    if name.starts_with("GPS") {
        "GPS"
    } else {
        "POS"
    }
}

/// The fields [`place_in`] reads, by name.
pub(crate) const PLACE_FIELDS: [&str; 3] = ["Status", "Lat", "Lng"];

/// [`place_of`] once the family's format is found, with `field` reading the record's field at
/// the place the family's format gives a name - one of [`PLACE_FIELDS`].
pub(crate) fn place_in(family: &str, field: impl Fn(&str) -> Option<f64>) -> Option<(f64, f64)> {
    if family == "GPS" {
        let status = field("Status")?;
        if status < 3.0 {
            return None;
        }
    }
    Some((field("Lat")?, field("Lng")?))
}

/// `DFLog.LogErrorSubsystem` by number, or the number when it is not one the C# names.
/// `// C#: ExtLibs/Utilities/DFLog.cs:339-374`
fn subsystem_name(number: i64) -> String {
    let name = match number {
        1 => "MAIN",
        2 => "RADIO",
        3 => "COMPASS",
        4 => "OPTFLOW",
        5 => "FAILSAFE_RADIO",
        6 => "FAILSAFE_BATT",
        7 => "FAILSAFE_GPS",
        8 => "FAILSAFE_GCS",
        9 => "FAILSAFE_FENCE",
        10 => "FLIGHT_MODE",
        11 => "GPS",
        12 => "CRASH_CHECK",
        13 => "FLIP",
        14 => "AUTOTUNE",
        15 => "PARACHUTES",
        16 => "EKFCHECK",
        17 => "FAILSAFE_EKFINAV",
        18 => "BARO",
        19 => "CPU",
        20 => "FAILSAFE_ADSB",
        21 => "TERRAIN",
        22 => "NAVIGATION",
        23 => "FAILSAFE_TERRAIN",
        24 => "EKF_PRIMARY",
        25 => "THRUST_LOSS_CHECK",
        26 => "FAILSAFE_SENSORS",
        27 => "FAILSAFE_LEAK",
        28 => "PILOT_INPUT",
        29 => "FAILSAFE_VIBE",
        30 => "INTERNAL_ERROR",
        31 => "FAILSAFE_DEADRECKON",
        _ => return number.to_string(),
    };
    name.to_owned()
}

/// `DFLog.Log_Event` by number, or the number when it is not one the C# names.
/// `// C#: ExtLibs/Utilities/DFLog.cs:258-337`
fn event_name(number: i64) -> String {
    let name = match number {
        7 => "AP_STATE",
        9 => "INIT_SIMPLE_BEARING",
        10 => "ARMED",
        11 => "DISARMED",
        15 => "AUTO_ARMED",
        17 => "LAND_COMPLETE_MAYBE",
        18 => "LAND_COMPLETE",
        28 => "NOT_LANDED",
        19 => "LOST_GPS",
        21 => "FLIP_START",
        22 => "FLIP_END",
        25 => "SET_HOME",
        26 => "SET_SIMPLE_ON",
        27 => "SET_SIMPLE_OFF",
        29 => "SET_SUPERSIMPLE_ON",
        30 => "AUTOTUNE_INITIALISED",
        31 => "AUTOTUNE_OFF",
        32 => "AUTOTUNE_RESTART",
        33 => "AUTOTUNE_SUCCESS",
        34 => "AUTOTUNE_FAILED",
        35 => "AUTOTUNE_REACHED_LIMIT",
        36 => "AUTOTUNE_PILOT_TESTING",
        37 => "AUTOTUNE_SAVEDGAINS",
        38 => "SAVE_TRIM",
        39 => "SAVEWP_ADD_WP",
        41 => "FENCE_ENABLE",
        42 => "FENCE_DISABLE",
        43 => "ACRO_TRAINER_OFF",
        44 => "ACRO_TRAINER_LEVELING",
        45 => "ACRO_TRAINER_LIMITED",
        46 => "GRIPPER_GRAB",
        47 => "GRIPPER_RELEASE",
        49 => "PARACHUTE_DISABLED",
        50 => "PARACHUTE_ENABLED",
        51 => "PARACHUTE_RELEASED",
        52 => "LANDING_GEAR_DEPLOYED",
        53 => "LANDING_GEAR_RETRACTED",
        54 => "MOTORS_EMERGENCY_STOPPED",
        55 => "MOTORS_EMERGENCY_STOP_CLEARED",
        56 => "MOTORS_INTERLOCK_DISABLED",
        57 => "MOTORS_INTERLOCK_ENABLED",
        58 => "ROTOR_RUNUP_COMPLETE",
        59 => "ROTOR_SPEED_BELOW_CRITICAL",
        60 => "EKF_ALT_RESET",
        61 => "LAND_CANCELLED_BY_PILOT",
        62 => "EKF_YAW_RESET",
        63 => "AVOIDANCE_ADSB_ENABLE",
        64 => "AVOIDANCE_ADSB_DISABLE",
        65 => "AVOIDANCE_PROXIMITY_ENABLE",
        66 => "AVOIDANCE_PROXIMITY_DISABLE",
        67 => "GPS_PRIMARY_CHANGED",
        68 => "WINCH_RELAXED",
        69 => "WINCH_LENGTH_CONTROL",
        70 => "WINCH_RATE_CONTROL",
        71 => "ZIGZAG_STORE_A",
        72 => "ZIGZAG_STORE_B",
        73 => "LAND_REPO_ACTIVE",
        74 => "STANDBY_ENABLE",
        75 => "STANDBY_DISABLE",
        80 => "FENCE_FLOOR_ENABLE",
        81 => "FENCE_FLOOR_DISABLE",
        85 => "EK3_SOURCES_SET_TO_PRIMARY",
        86 => "EK3_SOURCES_SET_TO_SECONDARY",
        87 => "EK3_SOURCES_SET_TO_TERTIARY",
        90 => "AIRSPEED_PRIMARY_CHANGED",
        163 => "SURFACED",
        164 => "NOT_SURFACED",
        165 => "BOTTOMED",
        166 => "NOT_BOTTOMED",
        _ => return number.to_string(),
    };
    name.to_owned()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{fixed, fmt, record};

    fn testdata(name: &str) -> Vec<u8> {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name);
        std::fs::read(&path).unwrap_or_else(|err| panic!("reading {}: {err}", path.display()))
    }

    /// The names a copter gives its modes, enough for the fixtures: what `onFlightMode` answers.
    fn copter_modes(firmware: Firmware, mode: u64) -> Option<String> {
        let name = match (firmware, mode) {
            (Firmware::Copter, 3) => "Auto",
            (Firmware::Copter, 5) => "Loiter",
            (Firmware::Plane, 4) => "ACRO",
            _ => return None,
        };
        Some(name.to_owned())
    }

    fn type_of(data: &[u8], name: &str) -> u8 {
        crate::index::RecordIndex::build(data)
            .format_named(name)
            .map(|format| format.msg_type)
            .unwrap_or_else(|| panic!("{name} is declared"))
    }

    /// The healthy fixture: a copter, one mode change named Auto, fourteen messages, and no
    /// errors or events - and every label on the record it came from.
    #[test]
    fn the_healthy_fixture_has_a_mode_and_its_messages() {
        let data = testdata("dataflash.bin");
        let overlays = overlays(&data, copter_modes);
        assert_eq!(overlays.firmware, Some(Firmware::Copter));
        assert_eq!(overlays.modes.len(), 1);
        assert_eq!(overlays.modes[0].mark.text, "Auto");
        assert_eq!(overlays.modes[0].number, 3);
        assert_eq!(overlays.messages.len(), 14);
        assert_eq!(overlays.messages[0].text, "ArduCopter V4.2.2 (4fcfa4b2)");
        assert!(overlays.errors.is_empty(), "the fixture logs no ERR");
        assert!(overlays.events.is_empty(), "nor any EV");
        assert!(
            overlays.minutes.is_empty(),
            "its GPS spans eighteen seconds, which is no minute"
        );

        let index = crate::index::RecordIndex::build(&data);
        let mode = type_of(&data, "MODE");
        let message = type_of(&data, "MSG");
        assert_eq!(index.msg_type(overlays.modes[0].mark.line), Some(mode));
        for mark in &overlays.messages {
            assert_eq!(index.msg_type(mark.line), Some(message), "{mark:?}");
            assert!(mark.time_us.is_some());
        }
    }

    /// The damaged fixture came off a vehicle in Loiter, whose EKF reset its yaw five times.
    #[test]
    fn the_damaged_fixture_has_its_events() {
        let data = testdata("dataflash_damaged.bin");
        let overlays = overlays(&data, copter_modes);
        assert_eq!(overlays.firmware, Some(Firmware::Copter));
        assert_eq!(overlays.modes.len(), 1);
        assert_eq!(overlays.modes[0].mark.text, "Loiter");
        assert_eq!(overlays.events.len(), 5);
        for event in &overlays.events {
            assert_eq!(event.text, "EV: EKF_YAW_RESET");
        }
        assert_eq!(overlays.messages.len(), 144);
        assert!(overlays.errors.is_empty());
    }

    /// With no name for a mode, its number is the label, as `onFlightMode` returning null leaves.
    #[test]
    fn a_mode_with_no_name_is_its_number() {
        let data = testdata("dataflash.bin");
        let overlays = overlays(&data, |_, _| None);
        assert_eq!(overlays.modes[0].mark.text, "3");
    }

    const GPS: u8 = 140;
    const POS: u8 = 141;
    const FILL: u8 = 142;
    const MODE: u8 = 143;
    const ERR: u8 = 144;
    const EV: u8 = 145;
    const MSG: u8 = 146;
    const PARM: u8 = 147;

    /// Declares every type the hand-built logs use: eight lines.
    fn declarations() -> Vec<u8> {
        let mut log = fmt(GPS, 20, "GPS", "QBLL", "TimeUS,Status,Lat,Lng");
        log.extend(fmt(POS, 19, "POS", "QLL", "TimeUS,Lat,Lng"));
        log.extend(fmt(FILL, 12, "FILL", "QB", "TimeUS,V"));
        log.extend(fmt(MODE, 14, "MODE", "QMBB", "TimeUS,Mode,ModeNum,Rsn"));
        log.extend(fmt(ERR, 13, "ERR", "QBB", "TimeUS,Subsys,ECode"));
        log.extend(fmt(EV, 12, "EV", "QB", "TimeUS,Id"));
        log.extend(fmt(MSG, 75, "MSG", "QZ", "TimeUS,Message"));
        log.extend(fmt(PARM, 31, "PARM", "QNf", "TimeUS,Name,Value"));
        log
    }

    const DECLARATIONS: usize = 8;

    fn degrees(value: f64) -> [u8; 4] {
        #[allow(clippy::cast_possible_truncation)]
        ((value * 1e7).round() as i32).to_le_bytes()
    }

    fn gps(time_us: u64, status: u8, lat: f64, lng: f64) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.push(status);
        payload.extend(degrees(lat));
        payload.extend(degrees(lng));
        record(GPS, &payload)
    }

    fn pos(time_us: u64, lat: f64, lng: f64) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.extend(degrees(lat));
        payload.extend(degrees(lng));
        record(POS, &payload)
    }

    fn fill(time_us: u64) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.push(0);
        record(FILL, &payload)
    }

    fn mode(time_us: u64, mode: u8) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.extend([mode, mode, 0]);
        record(MODE, &payload)
    }

    fn err(time_us: u64, subsystem: u8, code: u8) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.extend([subsystem, code]);
        record(ERR, &payload)
    }

    fn ev(time_us: u64, id: u8) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.push(id);
        record(EV, &payload)
    }

    fn msg(time_us: u64, text: &str) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.extend(fixed(text, 64));
        record(MSG, &payload)
    }

    fn parm(time_us: u64, name: &str, value: f32) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.extend(fixed(name, 16));
        payload.extend(value.to_le_bytes());
        record(PARM, &payload)
    }

    /// Errors read as `Err: SUBSYSTEM-code`, events as `EV: NAME`, and a number the C#'s enum
    /// does not name is written as the number, as an enum's `ToString` writes it.
    #[test]
    fn errors_and_events_are_labelled_as_the_c_sharp_labels_them() {
        let mut log = declarations();
        log.extend(err(1_000_000, 5, 1));
        log.extend(err(2_000_000, 99, 2));
        log.extend(ev(3_000_000, 10));
        log.extend(ev(4_000_000, 200));
        let overlays = overlays(&log, |_, _| None);
        let errors: Vec<&str> = overlays.errors.iter().map(|m| m.text.as_str()).collect();
        assert_eq!(errors, vec!["Err: FAILSAFE_RADIO-1", "Err: 99-2"]);
        let events: Vec<&str> = overlays.events.iter().map(|m| m.text.as_str()).collect();
        assert_eq!(events, vec!["EV: ARMED", "EV: 200"]);
        assert_eq!(overlays.errors[0].line, DECLARATIONS);
        assert_eq!(overlays.errors[0].time_us, Some(1_000_000.0));
        assert_eq!(overlays.events[1].line, DECLARATIONS + 3);
    }

    /// The last line that names a vehicle wins, so a plane's modes are named from the plane's
    /// table even after a line that said copter.
    #[test]
    fn the_last_line_naming_a_vehicle_decides_the_modes_names() {
        let mut log = declarations();
        log.extend(msg(1, "ArduCopter V4.2.2"));
        log.extend(mode(2, 4));
        log.extend(msg(3, "ArduPlane V4.5.0"));
        let overlays = overlays(&log, copter_modes);
        assert_eq!(overlays.firmware, Some(Firmware::Plane));
        assert_eq!(
            overlays.modes[0].mark.text, "ACRO",
            "named against the final guess, as DrawModes converts after the guess is made"
        );
    }

    /// `PARM, RATE_RLL_P` only matches a line with no time in between, which is an old log's.
    #[test]
    fn a_parameter_names_the_vehicle_only_in_a_log_without_timestamps() {
        let mut log = declarations();
        log.extend(parm(1, "RATE_RLL_P", 0.1));
        assert_eq!(overlays(&log, |_, _| None).firmware, None);
        assert_eq!(
            guess_firmware("PARM, RATE_RLL_P, 0.1"),
            Some(Firmware::Copter)
        );
        assert_eq!(
            guess_firmware("PARM, SKID_STEER_OUT, 0"),
            Some(Firmware::Rover)
        );
        assert_eq!(
            guess_firmware("MSG, 1, AntennaTracker V1"),
            Some(Firmware::Tracker)
        );
        assert_eq!(guess_firmware("MSG, 1, QuadPlane"), Some(Firmware::Plane));
        assert_eq!(guess_firmware("MSG, 1, hello"), None);
    }

    /// A mode logged as text is its own label.
    #[test]
    fn a_mode_written_as_text_is_its_own_label() {
        let mut log = fmt(MODE, 16, "MODE", "QnB", "TimeUS,Mode,ModeNum");
        let mut payload = 5u64.to_le_bytes().to_vec();
        payload.extend(fixed("AUTO", 4));
        payload.push(3);
        log.extend(record(MODE, &payload));
        let overlays = overlays(&log, copter_modes);
        assert_eq!(overlays.modes.len(), 1);
        assert_eq!(overlays.modes[0].mark.text, "AUTO");
        assert_eq!(overlays.modes[0].number, 3);
    }

    /// A MODE without a `ModeNum` is not drawn: `DrawModes` needs both.
    #[test]
    fn a_mode_without_its_number_is_not_drawn() {
        let mut log = fmt(MODE, 12, "MODE", "QM", "TimeUS,Mode");
        let mut payload = 5u64.to_le_bytes().to_vec();
        payload.push(3);
        log.extend(record(MODE, &payload));
        assert!(overlays(&log, copter_modes).modes.is_empty());
    }

    /// `DrawTime`: nothing on the first GPS record, then a label on the first record of each new
    /// minute, saying how many whole minutes have gone, rounded.
    #[test]
    fn a_minute_label_goes_on_the_first_gps_record_of_each_minute() {
        let mut log = declarations();
        // Every twenty-five seconds for three and a half minutes, from a boot clock of 10 s.
        for step in 0..9u64 {
            log.extend(gps(10_000_000 + step * 25_000_000, 1, 0.0, 0.0));
        }
        let overlays = overlays(&log, |_, _| None);
        let labels: Vec<(&str, usize)> = overlays
            .minutes
            .iter()
            .map(|mark| (mark.text.as_str(), mark.line - DECLARATIONS))
            .collect();
        // 75 s is minute 1 (label "1"), 125 s minute 2 ("2"), 200 s minute 3 ("3").
        assert_eq!(labels, vec![("1 min", 3), ("2 min", 5), ("3 min", 8)]);
    }

    /// A GPS clock that goes backwards wraps the C#'s unsigned subtraction, `AddMilliseconds`
    /// throws, and `DrawTime` draws nothing more.
    #[test]
    fn a_gps_clock_that_goes_backwards_ends_the_minute_labels() {
        let mut log = declarations();
        log.extend(gps(10_000_000, 1, 0.0, 0.0));
        log.extend(gps(80_000_000, 1, 0.0, 0.0)); // "1 min"
        log.extend(gps(5_000_000, 1, 0.0, 0.0)); // backwards: the end
        log.extend(gps(200_000_000, 1, 0.0, 0.0)); // would be "3 min"
        let overlays = overlays(&log, |_, _| None);
        let labels: Vec<&str> = overlays.minutes.iter().map(|m| m.text.as_str()).collect();
        assert_eq!(labels, vec!["1 min"]);
    }

    /// The healthy fixture's GPS never had a fix, so no record gives a place, wherever the
    /// cursor lands.
    #[test]
    fn a_gps_without_a_fix_gives_the_cursor_no_place() {
        let positions = Positions::read(&testdata("dataflash.bin"));
        assert_eq!(
            positions.records().len(),
            91,
            "the GPS records; GPA does not start with GPS and POWR not with POS"
        );
        for line in [0, 1376, 1400, 5000, 11_000, 11_438] {
            assert_eq!(positions.from_row(line), None, "line {line}");
        }
        assert_eq!(positions.from_row(11_439), None, "past the end");
    }

    /// The damaged fixture's positions are where the vehicle was.
    #[test]
    fn a_log_with_a_fix_gives_the_cursor_the_nearest_place() {
        let data = testdata("dataflash_damaged.bin");
        let positions = Positions::read(&data);
        let first = positions
            .records()
            .iter()
            .find(|record| record.place.is_some())
            .expect("a record with a place");
        let (lat, lng) = positions
            .from_row(first.line)
            .expect("a place on its own line");
        assert!(
            (lat + 27.5134).abs() < 0.01 && (lng - 153.0094).abs() < 0.01,
            "{lat},{lng}"
        );
    }

    /// `GetLineNoFromTime`: the first GPS, GPS2 or POS record at or after the time; past them
    /// all, the C#'s `long.MaxValue`; with none, 0.
    #[test]
    fn a_time_is_the_first_position_record_at_or_after_it() {
        let mut log = declarations();
        log.extend(fill(1_000_000));
        log.extend(gps(2_000_000, 3, -35.0, 149.0));
        log.extend(pos(3_000_000, -35.1, 149.1));
        let positions = Positions::read(&log);
        let gps_line = DECLARATIONS + 1;
        assert_eq!(positions.line_at_time(0.0), LineAtTime::Line(gps_line));
        assert_eq!(
            positions.line_at_time(2_000_000.0),
            LineAtTime::Line(gps_line)
        );
        assert_eq!(
            positions.line_at_time(2_000_001.0),
            LineAtTime::Line(gps_line + 1)
        );
        assert_eq!(positions.line_at_time(3_000_001.0), LineAtTime::AfterAll);
        assert_eq!(positions.time_of(gps_line), Some(2_000_000.0));
        assert_eq!(
            positions.time_of(DECLARATIONS),
            None,
            "not a position record"
        );

        let mut none = declarations();
        none.extend(fill(1));
        assert_eq!(
            Positions::read(&none).line_at_time(0.0),
            LineAtTime::NoPositions
        );
    }

    /// A log of `FILL` with position records at chosen lines.
    fn with_positions_at(lines: &[usize], total: usize) -> Vec<u8> {
        let mut log = declarations();
        for line in DECLARATIONS..total {
            if lines.contains(&line) {
                log.extend(pos(line as u64, latitude_of(line), 149.0));
            } else {
                log.extend(fill(line as u64));
            }
        }
        log
    }

    /// A latitude that says which line it came from, to seven places as a log stores it.
    fn latitude_of(line: usize) -> f64 {
        #[allow(clippy::cast_precision_loss)]
        let at = line as f64;
        ((-35.0 - at / 1e5) * 1e7).round() / 1e7
    }

    /// Nearest either way within a thousand lines; forward only when strictly nearer, so a tie
    /// goes back.
    #[test]
    fn the_cursor_takes_the_nearest_position_record() {
        let positions = Positions::read(&with_positions_at(&[100, 110], 400));
        let at = |line: usize| positions.from_row(line).map(|(lat, _)| lat);
        assert_eq!(at(100), Some(latitude_of(100)), "its own line");
        assert_eq!(at(103), Some(latitude_of(100)), "three back, seven forward");
        assert_eq!(at(108), Some(latitude_of(110)), "eight back, two forward");
        assert_eq!(at(105), Some(latitude_of(100)), "a tie goes back");
        assert_eq!(at(300), Some(latitude_of(110)), "only behind");
    }

    /// With nothing behind it, the C# jumps a thousand lines forward - past a record a few lines
    /// ahead - and finds a place only if one is exactly there.
    #[test]
    fn with_nothing_behind_the_cursor_jumps_a_thousand_lines_forward() {
        let positions = Positions::read(&with_positions_at(&[20, 1012], 1300));
        let at = |line: usize| positions.from_row(line).map(|(lat, _)| lat);
        assert_eq!(
            at(15),
            None,
            "line 20 is five ahead, but the C# reads line 1015, which is not a position"
        );
        assert_eq!(
            at(12),
            Some(latitude_of(1012)),
            "line 1012 is exactly a thousand ahead"
        );
        assert_eq!(
            at(400),
            Some(latitude_of(20)),
            "within a thousand behind, found"
        );
        assert_eq!(
            at(1011),
            Some(latitude_of(1012)),
            "one ahead is nearer than 991 behind"
        );
    }

    /// A GPS record needs a fix of 3 or more; a POS record needs nothing.
    #[test]
    fn a_gps_record_without_a_fix_is_no_place_but_a_pos_record_is() {
        let mut log = declarations();
        log.extend(gps(1, 2, -35.0, 149.0));
        log.extend(gps(2, 3, -35.2, 149.2));
        log.extend(pos(3, -35.3, 149.3));
        let positions = Positions::read(&log);
        assert_eq!(positions.from_row(DECLARATIONS), None, "status 2");
        assert_eq!(positions.from_row(DECLARATIONS + 1), Some((-35.2, 149.2)));
        assert_eq!(positions.from_row(DECLARATIONS + 2), Some((-35.3, 149.3)));
    }
}
