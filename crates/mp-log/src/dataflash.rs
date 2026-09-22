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
    pos: usize,
    formats: BTreeMap<u8, MessageFormat>,
    stats: DataflashStats,
}

impl<'a> DataflashReader<'a> {
    /// Wraps a log.
    #[must_use]
    pub fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            pos: 0,
            formats: BTreeMap::new(),
            stats: DataflashStats::default(),
        }
    }

    /// Format definitions seen so far.
    #[must_use]
    pub const fn formats(&self) -> &BTreeMap<u8, MessageFormat> {
        &self.formats
    }

    /// Parse counters.
    #[must_use]
    pub const fn stats(&self) -> &DataflashStats {
        &self.stats
    }

    /// Reads the next decodable message, or `None` at the end of the log.
    pub fn next_message(&mut self) -> Option<LogMessage> {
        loop {
            // Find a header. Logs from a failing SD card contain runs of garbage, so the
            // scan is bounds-checked at every step rather than trusting the length arithmetic.
            loop {
                match self.data.get(self.pos..self.pos + 3) {
                    Some([HEAD_BYTE1, HEAD_BYTE2, _]) => break,
                    Some(_) => {
                        self.pos += 1;
                        self.stats.resync_bytes += 1;
                    }
                    None => return None,
                }
            }

            let msg_type = *self.data.get(self.pos + 2)?;
            let body_start = self.pos + 3;

            if msg_type == FMT_TYPE {
                let body = self.data.get(body_start..body_start + FMT_PAYLOAD_LEN)?;
                let format = parse_fmt(body);
                self.pos = body_start + FMT_PAYLOAD_LEN;
                if let Some(format) = format {
                    self.stats.formats += 1;
                    if !format.is_self_consistent() {
                        self.stats.inconsistent_formats += 1;
                    }
                    self.formats.insert(format.msg_type, format);
                }
                continue;
            }

            let Some(format) = self.formats.get(&msg_type).cloned() else {
                // A message type with no definition cannot be skipped reliably, because its
                // length is unknown. Resynchronise rather than guess.
                self.stats.unknown_types += 1;
                self.pos += 1;
                self.stats.resync_bytes += 1;
                continue;
            };

            let payload_len = format.payload_len();
            let body = self.data.get(body_start..body_start + payload_len)?;
            self.pos = body_start + payload_len;

            if !format.is_self_consistent() {
                continue;
            }

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
                let label = format
                    .labels
                    .get(index)
                    .cloned()
                    .unwrap_or_else(|| format!("field{index}"));
                fields.push((label, value));
                offset += size;
            }

            self.stats.messages += 1;
            return Some(LogMessage {
                name: format.name.clone(),
                fields,
            });
        }
    }
}

/// Parses an `FMT` payload.
fn parse_fmt(body: &[u8]) -> Option<MessageFormat> {
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
