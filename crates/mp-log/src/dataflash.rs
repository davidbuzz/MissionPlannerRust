//! Dataflash (`.BIN`) log parsing.
//!
//! Replaces `ExtLibs/Utilities/BinaryLog.cs` and `DFLog.cs`.
//!
//! # Format
//!
//! A dataflash log is a stream of messages, each `0xA3 0x95 <type>` followed by a payload whose
//! length and layout are described by an `FMT` message that appeared **earlier in the same log**.
//! The log is therefore self-describing and cannot be parsed without reading it in order: a
//! message type means nothing until its format has been seen.
//!
//! `FMT` is itself message type 0x80, with a fixed layout, which is the bootstrap.
//!
//! # Why the field types matter
//!
//! Several types are scaled integers rather than the numbers they represent: `L` is degrees in 1e7
//! fixed point, `c` and `C` are hundredths, `e` and `E` are hundredths of a larger range. Reading
//! an `L` as a plain `i32` yields 350,123,456 instead of 35.0123456 - a number that looks like
//! data and is off by seven orders of magnitude.

use std::collections::BTreeMap;

/// First byte of every message header.
pub const HEAD_BYTE1: u8 = 0xA3;
/// Second byte of every message header.
pub const HEAD_BYTE2: u8 = 0x95;
/// Message type of the format-definition message.
pub const FMT_TYPE: u8 = 0x80;
/// Payload length of an `FMT` message: type, length, 4-byte name, 16-byte format, 64-byte labels.
pub const FMT_PAYLOAD_LEN: usize = 1 + 1 + 4 + 16 + 64;

/// A decoded field value.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    /// Signed integer field.
    Int(i64),
    /// Unsigned integer field.
    Uint(u64),
    /// Floating point, including the scaled integer types once scaled.
    Float(f64),
    /// Text field.
    Text(String),
    /// Opaque bytes.
    Bytes(Vec<u8>),
    /// Array of 16-bit samples.
    Samples(Vec<i16>),
}

impl Value {
    /// The value as a number, where that is meaningful.
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // charting a 2^53 counter is not a real scenario
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Self::Int(v) => Some(*v as f64),
            Self::Uint(v) => Some(*v as f64),
            Self::Float(v) => Some(*v),
            Self::Text(_) | Self::Bytes(_) | Self::Samples(_) => None,
        }
    }

    /// The value as text, where it is text.
    ///
    /// Needed by `FMTU`, whose `UnitIds` is a string whose *positions* carry meaning - a `#` marks
    /// which field of a message is its instance number.
    #[must_use]
    pub fn as_text(&self) -> Option<&str> {
        match self {
            Self::Text(text) => Some(text),
            _ => None,
        }
    }
}

/// A dataflash field type, as a single character in an `FMT` format string.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FieldType(pub u8);

impl FieldType {
    /// Bytes this field occupies on disk.
    #[must_use]
    pub const fn size(self) -> Option<usize> {
        Some(match self.0 {
            b'b' | b'B' | b'M' => 1,
            b'h' | b'H' | b'c' | b'C' | b'g' => 2,
            b'i' | b'I' | b'e' | b'E' | b'L' | b'f' | b'n' => 4,
            b'q' | b'Q' | b'd' => 8,
            b'N' => 16,
            b'Z' | b'a' => 64,
            _ => return None,
        })
    }

    /// Decodes one field from `bytes`, which must be at least [`FieldType::size`] long.
    ///
    /// Every read goes through a checked array conversion rather than indexing, so a truncated
    /// buffer returns `None` instead of panicking. That matters here: this parser is pointed at
    /// logs recovered from failing SD cards.
    #[must_use]
    pub fn decode(self, bytes: &[u8]) -> Option<Value> {
        let size = self.size()?;
        let raw = bytes.get(..size)?;

        let two = || -> Option<[u8; 2]> { raw.get(..2)?.try_into().ok() };
        let four = || -> Option<[u8; 4]> { raw.get(..4)?.try_into().ok() };
        let eight = || -> Option<[u8; 8]> { raw.get(..8)?.try_into().ok() };

        Some(match self.0 {
            b'b' => Value::Int(i64::from(*raw.first()? as i8)),
            b'B' | b'M' => Value::Uint(u64::from(*raw.first()?)),
            b'h' => Value::Int(i64::from(i16::from_le_bytes(two()?))),
            b'H' => Value::Uint(u64::from(u16::from_le_bytes(two()?))),
            b'i' => Value::Int(i64::from(i32::from_le_bytes(four()?))),
            b'I' => Value::Uint(u64::from(u32::from_le_bytes(four()?))),
            b'q' => Value::Int(i64::from_le_bytes(eight()?)),
            b'Q' => Value::Uint(u64::from_le_bytes(eight()?)),
            b'f' => Value::Float(f64::from(f32::from_le_bytes(four()?))),
            b'd' => Value::Float(f64::from_le_bytes(eight()?)),
            b'g' => Value::Float(f64::from(half_to_f32(u16::from_le_bytes(two()?)))),
            // Scaled integers. Getting these wrong produces numbers that look like data: an `L`
            // read unscaled is 350123456 instead of 35.0123456.
            b'c' => Value::Float(f64::from(i16::from_le_bytes(two()?)) / 100.0),
            b'C' => Value::Float(f64::from(u16::from_le_bytes(two()?)) / 100.0),
            b'e' => Value::Float(f64::from(i32::from_le_bytes(four()?)) / 100.0),
            b'E' => Value::Float(f64::from(u32::from_le_bytes(four()?)) / 100.0),
            b'L' => Value::Float(f64::from(i32::from_le_bytes(four()?)) / 1e7),
            b'n' | b'N' => Value::Text(trim_text(raw)),
            b'Z' => Value::Bytes(raw.to_vec()),
            b'a' => Value::Samples(
                raw.chunks_exact(2)
                    .filter_map(|c| c.try_into().ok().map(i16::from_le_bytes))
                    .collect(),
            ),
            _ => return None,
        })
    }
}

/// IEEE 754 half precision to `f32`. ArduPilot logs some fields this way to halve the storage.
fn half_to_f32(bits: u16) -> f32 {
    let sign = u32::from(bits & 0x8000) << 16;
    let exponent = (bits >> 10) & 0x1F;
    let mantissa = u32::from(bits & 0x03FF);

    let out = match exponent {
        0 if mantissa == 0 => sign,
        // Subnormal: normalise by hand rather than losing the value.
        0 => {
            let mut e: i32 = -1;
            let mut m = mantissa;
            while m & 0x0400 == 0 {
                m <<= 1;
                e -= 1;
            }
            sign | (((127 - 15 + e + 1) as u32) << 23) | ((m & 0x03FF) << 13)
        }
        31 => sign | 0x7F80_0000 | (mantissa << 13),
        _ => sign | ((u32::from(exponent) + 127 - 15) << 23) | (mantissa << 13),
    };
    f32::from_bits(out)
}

fn trim_text(raw: &[u8]) -> String {
    let end = raw.iter().position(|b| *b == 0).unwrap_or(raw.len());
    raw.get(..end).map_or_else(String::new, |b| {
        String::from_utf8_lossy(b).trim().to_owned()
    })
}

/// The layout of one message type, as declared by an `FMT` message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MessageFormat {
    /// Message type byte.
    pub msg_type: u8,
    /// Total message length including the three header bytes.
    pub length: u8,
    /// Message name, e.g. `GPS`.
    pub name: String,
    /// Format string, one character per field.
    pub format: String,
    /// Field labels, comma separated in the log.
    pub labels: Vec<String>,
}

impl MessageFormat {
    /// Payload length: total length minus the three header bytes.
    #[must_use]
    pub const fn payload_len(&self) -> usize {
        (self.length as usize).saturating_sub(3)
    }

    /// Whether the declared length matches the sum of the field sizes.
    ///
    /// A mismatch means the log was written by firmware whose format string and length disagree,
    /// which happens, and silently misreading every subsequent field of that type is worse than
    /// skipping it.
    #[must_use]
    pub fn is_self_consistent(&self) -> bool {
        let declared: Option<usize> = self
            .format
            .bytes()
            .map(|c| FieldType(c).size())
            .sum::<Option<usize>>();
        declared == Some(self.payload_len())
    }
}

/// One decoded message.
#[derive(Debug, Clone, PartialEq)]
pub struct LogMessage {
    /// Message name, e.g. `ATT`.
    pub name: String,
    /// Fields in declaration order, paired with their labels.
    pub fields: Vec<(String, Value)>,
}

impl LogMessage {
    /// Looks a field up by label.
    #[must_use]
    pub fn field(&self, label: &str) -> Option<&Value> {
        self.fields
            .iter()
            .find(|(name, _)| name == label)
            .map(|(_, value)| value)
    }
}

/// Counters describing what a parse encountered.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DataflashStats {
    /// Messages decoded.
    pub messages: u64,
    /// Format definitions read.
    pub formats: u64,
    /// Messages whose type had no format definition, so they could not be decoded.
    pub unknown_types: u64,
    /// Formats whose declared length disagreed with their format string.
    pub inconsistent_formats: u64,
    /// Bytes skipped looking for a header.
    pub resync_bytes: u64,
}

/// Reads messages from a dataflash log held in memory.
#[derive(Debug)]
pub struct DataflashReader<'a> {
    data: &'a [u8],
    walk: Walk,
}

/// The walk over a log's records, apart from the bytes it walks.
///
/// Kept apart so that a walk can stop where the bytes read so far run out and go on when more
/// arrive: [`crate::index::RecordIndex`] indexes a file a few megabytes at a time as it reads
/// it, while those bytes are still in the cache, rather than reading a gigabyte and then walking
/// it back out of memory. Given the whole log it is [`DataflashReader::next_record`].
#[derive(Debug, Clone)]
pub(crate) struct Walk {
    pos: usize,
    formats: BTreeMap<u8, MessageFormat>,
    stats: DataflashStats,
    /// Each declared type's payload length and whether its format adds up, by type byte.
    ///
    /// What [`Self::step`] asks of every record, kept beside `formats` so that a walk of a log of
    /// tens of millions of records does neither a map lookup nor a sum over a format string for
    /// each one. Written only where `formats` is.
    shapes: [Option<Shape>; 256],
}

/// What the walk needs of a format to step over one of its records.
#[derive(Debug, Clone, Copy)]
struct Shape {
    /// [`MessageFormat::payload_len`].
    payload_len: usize,
    /// [`MessageFormat::is_self_consistent`].
    consistent: bool,
}

/// Where a step of the walk ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Step {
    /// At a whole record.
    Record(RecordAt),
    /// At the end of the bytes so far, part way into something they do not yet hold all of.
    More,
    /// At the limit it was given: the next record would start there or after.
    Limit,
    /// At the end of the log.
    End,
}

impl Default for Walk {
    fn default() -> Self {
        Self {
            pos: 0,
            formats: BTreeMap::new(),
            stats: DataflashStats::default(),
            shapes: [None; 256],
        }
    }
}

impl Walk {
    /// A walk that goes on from `pos` knowing the formats `known` knows: where a walk of one
    /// piece of a log starts, on the guess that the log declared nothing new before it.
    pub(crate) fn resumed_at(pos: usize, known: &Self) -> Self {
        Self {
            pos,
            formats: known.formats.clone(),
            stats: DataflashStats::default(),
            shapes: known.shapes,
        }
    }

    /// Format definitions seen so far.
    pub(crate) const fn formats(&self) -> &BTreeMap<u8, MessageFormat> {
        &self.formats
    }

    /// Moves the walk's position by `by`: a walk of a piece of the bytes, carried on over the
    /// whole of them, where the piece starts `by` in.
    pub(crate) const fn rebase(&mut self, by: usize) {
        self.pos += by;
    }

    /// Steps to the next whole record in `data`, `FMT` included.
    ///
    /// `complete` says whether `data` is the whole log. When it is, running out of bytes ends
    /// the walk, as it always has; when it is not, the walk stops where it is and says so, and
    /// the next step, given the same bytes and more after them, goes on from there - nothing it
    /// has decided depends on a byte it has not seen.
    pub(crate) fn step(&mut self, data: &[u8], complete: bool) -> Step {
        self.step_until(data, complete, usize::MAX)
    }

    /// [`Self::step`], stopping short of a record that would start at `limit` or after: the
    /// walk of one piece of a log, which leaves the next piece's records to the next piece.
    pub(crate) fn step_until(&mut self, data: &[u8], complete: bool, limit: usize) -> Step {
        let out = || if complete { Step::End } else { Step::More };
        loop {
            // Find a header. Logs from a failing SD card contain runs of garbage, so the
            // scan is bounds-checked at every step rather than trusting the length arithmetic.
            loop {
                if self.pos >= limit {
                    return Step::Limit;
                }
                match data.get(self.pos..self.pos + 3) {
                    Some([HEAD_BYTE1, HEAD_BYTE2, _]) => break,
                    Some(_) => {
                        self.pos += 1;
                        self.stats.resync_bytes += 1;
                    }
                    None => return out(),
                }
            }

            let offset = self.pos;
            let Some(&msg_type) = data.get(self.pos + 2) else {
                return out();
            };
            let body_start = self.pos + 3;

            if msg_type == FMT_TYPE {
                let Some(body) = data.get(body_start..body_start + FMT_PAYLOAD_LEN) else {
                    return out();
                };
                let format = parse_fmt(body);
                self.pos = body_start + FMT_PAYLOAD_LEN;
                if let Some(format) = format {
                    self.stats.formats += 1;
                    let consistent = format.is_self_consistent();
                    if !consistent {
                        self.stats.inconsistent_formats += 1;
                    }
                    if let Some(shape) = self.shapes.get_mut(usize::from(format.msg_type)) {
                        *shape = Some(Shape {
                            payload_len: format.payload_len(),
                            consistent,
                        });
                    }
                    self.formats.insert(format.msg_type, format);
                    return Step::Record(RecordAt { offset, msg_type });
                }
                continue;
            }

            let Some(Shape {
                payload_len,
                consistent,
            }) = self.shapes.get(usize::from(msg_type)).copied().flatten()
            else {
                // A message type with no definition cannot be skipped reliably, because its
                // length is unknown. Resynchronise rather than guess.
                self.stats.unknown_types += 1;
                self.pos += 1;
                self.stats.resync_bytes += 1;
                continue;
            };

            // A body cut short by the end of the log ends the walk: there is nothing after it.
            if data.get(body_start..body_start + payload_len).is_none() {
                return out();
            }
            self.pos = body_start + payload_len;

            if !consistent {
                continue;
            }
            return Step::Record(RecordAt { offset, msg_type });
        }
    }
}

impl<'a> DataflashReader<'a> {
    /// Wraps a log.
    #[must_use]
    pub fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            walk: Walk::default(),
        }
    }

    /// Format definitions seen so far.
    #[must_use]
    pub const fn formats(&self) -> &BTreeMap<u8, MessageFormat> {
        &self.walk.formats
    }

    /// Parse counters.
    #[must_use]
    pub const fn stats(&self) -> &DataflashStats {
        &self.walk.stats
    }

    /// Reads the next decodable message, or `None` at the end of the log.
    ///
    /// `FMT` records are consumed into [`Self::formats`] rather than returned: every reader of
    /// messages wants the data, not the declarations. [`Self::next_record`] is the walk that
    /// includes them.
    pub fn next_message(&mut self) -> Option<LogMessage> {
        loop {
            let record = self.next_record()?;
            if record.msg_type == FMT_TYPE {
                continue;
            }
            // `next_record` only stops on a type whose format it holds and whose body is whole,
            // so neither lookup below can miss; they are checked anyway, and a miss ends the walk
            // rather than panicking.
            let format = self.walk.formats.get(&record.msg_type)?;
            let body_start = record.offset + 3;
            let body = self
                .data
                .get(body_start..body_start + format.payload_len())?;
            let fields = decode_fields(format, body);
            self.walk.stats.messages += 1;
            return Some(LogMessage {
                name: format.name.clone(),
                fields,
            });
        }
    }

    /// Steps to the next whole record, `FMT` included, and says where it starts.
    ///
    /// Mission Planner's log browser shows every record the log holds as a row of its grid,
    /// format declarations among them, and reaches a row by its byte offset rather than by
    /// holding the row: `DFLogBuffer` keeps a `linestartoffset` per record and decodes on demand.
    /// This is the walk that finds those offsets. The rules are the ones [`Self::next_message`]
    /// has always applied - resynchronise on garbage, skip a type with no format, skip a type
    /// whose format contradicts itself - because both are the same walk.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:98-126`
    pub fn next_record(&mut self) -> Option<RecordAt> {
        match self.walk.step(self.data, true) {
            Step::Record(record) => Some(record),
            Step::More | Step::Limit | Step::End => None,
        }
    }
}

/// Where one record sits in a log, as [`DataflashReader::next_record`] finds it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecordAt {
    /// Byte offset of the record's first header byte.
    pub offset: usize,
    /// Its message type.
    pub msg_type: u8,
}

/// The longest record the format allows: the length byte is a `u8` and counts the header.
pub const MAX_RECORD_LEN: usize = u8::MAX as usize;

/// Decodes the one record at the start of `bytes`, with a format table already collected.
///
/// The random-access half of [`DataflashReader::next_record`]: a grid showing rows ten thousand
/// records into a log decodes those rows and nothing else, against the formats the whole log
/// declared. An `FMT` record decodes against the log's own declaration of `FMT` when it has one,
/// as every ArduPilot log does, and otherwise by its fixed layout, so a format declaration is a
/// row like any other.
#[must_use]
pub fn decode_record(formats: &BTreeMap<u8, MessageFormat>, bytes: &[u8]) -> Option<LogMessage> {
    let Some(&[HEAD_BYTE1, HEAD_BYTE2, msg_type]) = bytes.get(..3) else {
        return None;
    };
    decode_as(formats.get(&msg_type), bytes)
}

/// Decodes the one record at the start of `bytes` with the format given for its type.
///
/// [`decode_record`] with the format chosen by the caller rather than looked up: a log that
/// declares a type twice, differently, has records of both layouts, and the walk decodes each
/// with the declaration in force where it was logged. `None` for an `FMT` record means its
/// fixed layout, as it does there.
#[must_use]
pub fn decode_as(format: Option<&MessageFormat>, bytes: &[u8]) -> Option<LogMessage> {
    let Some(&[HEAD_BYTE1, HEAD_BYTE2, msg_type]) = bytes.get(..3) else {
        return None;
    };
    let body = bytes.get(3..)?;
    match format {
        Some(format) if format.is_self_consistent() => {
            let body = body.get(..format.payload_len())?;
            Some(LogMessage {
                name: format.name.clone(),
                fields: decode_fields(format, body),
            })
        }
        _ if msg_type == FMT_TYPE => {
            let format = parse_fmt(body.get(..FMT_PAYLOAD_LEN)?)?;
            Some(LogMessage {
                name: "FMT".to_owned(),
                fields: vec![
                    ("Type".to_owned(), Value::Uint(u64::from(format.msg_type))),
                    ("Length".to_owned(), Value::Uint(u64::from(format.length))),
                    ("Name".to_owned(), Value::Text(format.name)),
                    ("Format".to_owned(), Value::Text(format.format)),
                    ("Columns".to_owned(), Value::Text(format.labels.join(","))),
                ],
            })
        }
        _ => None,
    }
}

/// One field of a format, found once and read from any number of records: a columnar read.
///
/// `DFLogBuffer` decodes a whole record to read one field of it. Reading ten million records'
/// `Roll` that way decodes every field of every one; this finds where `Roll` sits in the layout
/// once, and then reads those bytes of each record and nothing else. A field is found exactly as
/// [`LogMessage::field`] finds it in a decoded record - the first with the label, among the fields
/// the record decodes to - so a read here and a lookup there agree.
/// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:696-760`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Column {
    /// Where the field starts, counted from the record's first header byte.
    at: usize,
    /// Its type.
    field: FieldType,
}

impl Column {
    /// The field at a position in a format, if the format's record decodes that far.
    pub(crate) fn nth(format: &MessageFormat, position: usize) -> Option<Self> {
        let mut at = 3usize;
        for (index, code) in format.format.bytes().enumerate() {
            let field = FieldType(code);
            let size = field.size()?;
            if index == position {
                return Some(Self { at, field });
            }
            at += size;
        }
        None
    }

    /// The first field with a label, as [`LogMessage::field`] finds it: a field the format gives
    /// no label is `field<n>`, as [`decode_fields`] names it.
    pub(crate) fn named(format: &MessageFormat, label: &str) -> Option<Self> {
        let position = (0..format.format.len()).find(|index| match format.labels.get(*index) {
            Some(named) => named == label,
            None => format!("field{index}") == label,
        })?;
        Self::nth(format, position)
    }

    /// Whether the field is a number: whether [`Value::as_f64`] of it is `Some`.
    pub(crate) const fn numeric(self) -> bool {
        !matches!(self.field.0, b'n' | b'N' | b'Z' | b'a')
    }

    /// Whether the field is text: whether [`Value::as_text`] of it is `Some`.
    pub(crate) const fn text(self) -> bool {
        matches!(self.field.0, b'n' | b'N')
    }

    /// The field's bytes in the record that starts at `record`, undecoded.
    pub(crate) fn raw(self, data: &[u8], record: usize) -> Option<&[u8]> {
        let size = self.field.size()?;
        data.get(record + self.at..record + self.at + size)
    }

    /// The field's value in the record that starts at `record`.
    pub(crate) fn read(self, data: &[u8], record: usize) -> Option<Value> {
        self.field.decode(data.get(record + self.at..)?)
    }

    /// The field as a number, where it is one: `read(..)?.as_f64()`, without making a
    /// [`Value`] of it - the read a sweep over millions of records does of each.
    #[allow(clippy::cast_precision_loss)] // as `Value::as_f64` loses it
    pub(crate) fn read_f64(self, data: &[u8], record: usize) -> Option<f64> {
        let start = record + self.at;
        let two = || -> Option<[u8; 2]> { data.get(start..start + 2)?.try_into().ok() };
        let four = || -> Option<[u8; 4]> { data.get(start..start + 4)?.try_into().ok() };
        let eight = || -> Option<[u8; 8]> { data.get(start..start + 8)?.try_into().ok() };
        Some(match self.field.0 {
            b'b' => f64::from(i8::from_le_bytes([*data.get(start)?])),
            b'B' | b'M' => f64::from(*data.get(start)?),
            b'h' => f64::from(i16::from_le_bytes(two()?)),
            b'H' => f64::from(u16::from_le_bytes(two()?)),
            b'i' => f64::from(i32::from_le_bytes(four()?)),
            b'I' => f64::from(u32::from_le_bytes(four()?)),
            b'q' => i64::from_le_bytes(eight()?) as f64,
            b'Q' => u64::from_le_bytes(eight()?) as f64,
            b'f' => f64::from(f32::from_le_bytes(four()?)),
            b'd' => f64::from_le_bytes(eight()?),
            b'g' => f64::from(half_to_f32(u16::from_le_bytes(two()?))),
            b'c' => f64::from(i16::from_le_bytes(two()?)) / 100.0,
            b'C' => f64::from(u16::from_le_bytes(two()?)) / 100.0,
            b'e' => f64::from(i32::from_le_bytes(four()?)) / 100.0,
            b'E' => f64::from(u32::from_le_bytes(four()?)) / 100.0,
            b'L' => f64::from(i32::from_le_bytes(four()?)) / 1e7,
            _ => return None,
        })
    }
}

/// What a reader found in the last format it looked at, kept until the format changes.
///
/// Records read in log order through an index nearly all share one format per type, so the
/// columns a reader wants are looked up once per stretch rather than once per record.
#[derive(Debug)]
pub(crate) struct PerFormat<'a, T> {
    /// The format last looked at, and what was found in it.
    last: Option<(&'a MessageFormat, T)>,
}

impl<T> Default for PerFormat<'_, T> {
    fn default() -> Self {
        Self { last: None }
    }
}

impl<'a, T: Copy> PerFormat<'a, T> {
    /// What `find` finds in `format`, found again only when the format is not the last one.
    pub(crate) fn get(
        &mut self,
        format: &'a MessageFormat,
        find: impl FnOnce(&'a MessageFormat) -> T,
    ) -> T {
        match self.last {
            Some((seen, found)) if std::ptr::eq(seen, format) => found,
            _ => {
                let found = find(format);
                self.last = Some((format, found));
                found
            }
        }
    }
}

/// The label [`decode_fields`] gives a format's field: its own, or `field<n>` when the format
/// declares fewer labels than fields.
pub(crate) fn label_of(format: &MessageFormat, index: usize) -> String {
    format
        .labels
        .get(index)
        .cloned()
        .unwrap_or_else(|| format!("field{index}"))
}

/// Decodes a record's body field by field, stopping at the first that does not fit.
fn decode_fields(format: &MessageFormat, body: &[u8]) -> Vec<(String, Value)> {
    let mut fields = Vec::with_capacity(format.format.len());
    let mut offset = 0usize;
    for (index, code) in format.format.bytes().enumerate() {
        let field_type = FieldType(code);
        let Some(size) = field_type.size() else { break };
        let Some(slice) = body.get(offset..offset + size) else {
            break;
        };
        let Some(value) = field_type.decode(slice) else {
            break;
        };
        fields.push((label_of(format, index), value));
        offset += size;
    }
    fields
}

/// Parses an `FMT` payload.
pub(crate) fn parse_fmt(body: &[u8]) -> Option<MessageFormat> {
    let msg_type = *body.first()?;
    let length = *body.get(1)?;
    let name = trim_text(body.get(2..6)?);
    let format = trim_text(body.get(6..22)?);
    let labels_raw = trim_text(body.get(22..86)?);

    let labels = if labels_raw.is_empty() {
        Vec::new()
    } else {
        labels_raw.split(',').map(|l| l.trim().to_owned()).collect()
    };

    Some(MessageFormat {
        msg_type,
        length,
        name,
        format,
        labels,
    })
}
