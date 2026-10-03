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

//! Convert .Bin to .Log: Mission Planner's `BinaryLog`, the dataflash reader behind every
//! conversion on the flight screen's DataFlash Logs page.
//!
//! `but_bintolog_Click` runs `BinaryLog.ConvertBin` on each file it is given and writes the text
//! beside it, `<name>.log` (`GCSViews/FlightData.cs:1082-1098`). "Create KML + gpx", "Create
//! Matlab file" and the log browser read the same messages through `DFLogBuffer`, which is this
//! reader driven by an index (see [`crate::dflogbuffer`]). So this is a port of `BinaryLog.cs` as
//! it is, quirks included, and not the tidier [`crate::dataflash`] reader:
//!
//! - the header is found by a three-state scan that misses `A3 A3 95` (the second `A3` resets the
//!   scan instead of restarting it);
//! - a message type is looked up in a cache filled lazily from the formats seen so far, so a type
//!   declared again later keeps its first layout;
//! - a message whose format runs past its body, whose type has no format, or whose declared length
//!   is below three is dropped with its bytes consumed, and the scan carries on after it;
//! - a `Z` field that is all NULs throws inside the line's formatting (`Aggregate` of an empty
//!   sequence), so that message is dropped too - every all-blank `MSG` vanishes from the text;
//! - a body cut short by the end of the file is read as far as it goes and zero-filled.
//!
//! The text is one line per message, fields joined by `", "`, each ended by `"\r\n"` on every
//! platform, numbers formatted as .NET Framework formats them ([`crate::netfmt`]). A flight mode
//! (`M`) is named through `BinaryLog.onFlightMode`, which the application wires to the firmware's
//! mode table (`MainV2.cs:3394-3418`); it is the `mode_name` argument here, asked only once the log
//! has named its firmware, and a mode it does not name is written as its number.
//!
//! `tests/convert.rs` holds [`convert_bin`] to Mission Planner's own output, byte for byte, on both
//! checked-in logs.
//! `// C#: ExtLibs/Utilities/BinaryLog.cs`

use std::path::{Path, PathBuf};

use crate::netfmt;
use crate::overlay::Firmware;

/// First byte of every message header. `// C#: ExtLibs/Utilities/BinaryLog.cs:19`
pub const HEAD_BYTE1: u8 = 0xA3;
/// Second byte of every message header. `// C#: ExtLibs/Utilities/BinaryLog.cs:20`
pub const HEAD_BYTE2: u8 = 0x95;
/// The format message's type.
const FMT_TYPE: u8 = 0x80;
/// `Marshal.SizeOf(log_Format)`: type, length, 4-byte name, 16-byte format, 64-byte labels.
/// `// C#: ExtLibs/Utilities/BinaryLog.cs:22-30`
const FMT_BODY: usize = 1 + 1 + 4 + 16 + 64;

/// `BinaryLog.onFlightMode`: the name of mode `number` on `firmware`, or `None` for the number.
pub type ModeName<'a> = &'a dyn Fn(Firmware, u8) -> Option<String>;

/// A flight mode is never named: every `M` field is written as its number, as the C# writes it
/// when nothing is subscribed to `onFlightMode`.
#[must_use]
pub fn no_mode_names(_: Firmware, _: u8) -> Option<String> {
    None
}

/// `BinaryLog.onFlightMode` as the application wires it: the first entry for `mode` in
/// `getModesList` of the firmware. Copter, plane and rover modes are the parameter metadata's
/// (`FLTMODE1`, `FLTMODE1`, `MODE1` in `ParameterMetaDataBackup.xml`, which `mp-vehicle`'s table
/// is generated from), a plane has `16: INITIALISING` after them, and a tracker has a fixed list.
/// This is what the DataFlash Logs page passes as `mode_name`.
/// `// C#: MainV2.cs:3394-3418; ExtLibs/ArduPilot/Common.cs:88-183`
#[must_use]
pub fn flight_mode_name(firmware: Firmware, mode: u8) -> Option<String> {
    use mp_vehicle::VehicleFamily;
    let number = u32::from(mode);
    let name = match firmware {
        Firmware::Copter => VehicleFamily::Copter.mode_name(number),
        Firmware::Plane => VehicleFamily::Plane
            .mode_name(number)
            .or((mode == 16).then_some("INITIALISING")),
        Firmware::Rover => VehicleFamily::Rover.mode_name(number),
        Firmware::Tracker => match mode {
            0 => Some("MANUAL"),
            1 => Some("STOP"),
            2 => Some("SCAN"),
            3 => Some("SERVO_TEST"),
            10 => Some("AUTO"),
            16 => Some("INITIALISING"),
            _ => None,
        },
    };
    name.map(str::to_owned)
}

/// One `FMT` record as `log_Format` holds it. `// C#: ExtLibs/Utilities/BinaryLog.cs:22-30`
#[derive(Debug, Clone)]
struct LogFormat {
    msg_type: u8,
    length: u8,
    name: String,
    format: String,
}

/// `log_format_cache`. `// C#: ExtLibs/Utilities/BinaryLog.cs:32-38`
#[derive(Debug, Clone, Default)]
struct CacheEntry {
    length: u8,
    name: String,
    format: String,
}

/// One decoded field, as `GetObjectFromMessage` boxes it.
/// `// C#: ExtLibs/Utilities/BinaryLog.cs:500-573`
#[derive(Debug, Clone, PartialEq)]
pub enum Field {
    /// A `string`: the message name, `n`, `N`, a flight mode, the text of an `FMT`.
    Text(String),
    /// `b`, `h`, `i`, `q`.
    Int(i64),
    /// `B`, `H`, `I`, `Q`, and the type and length of an `FMT`.
    UInt(u64),
    /// `f` and `g`: a `float`.
    Single(f32),
    /// `d`, and the scaled `c`, `C`, `e`, `E`, `L`: a `double`.
    Double(f64),
    /// `Z`: 64 raw bytes.
    Bytes(Vec<u8>),
    /// `a`: the 64 bytes of 32 `int16`, as the `UnionArray` holds them.
    Array(Vec<u8>),
    /// A format character `GetObjectFromMessage` does not know: `null`, and no bytes.
    Null,
}

impl Field {
    /// The field as `ReadMessage` writes it into a line, or `None` where that throws.
    /// `// C#: ExtLibs/Utilities/BinaryLog.cs:157-176`
    fn line_text(&self) -> Option<String> {
        Some(match self {
            Self::Bytes(bytes) => {
                let text = netfmt::ascii(bytes);
                let text = netfmt::trim_nul(&text)
                    .replace('\\', "\\\\")
                    .replace('\n', "\\n")
                    .replace('\r', "\\r")
                    .replace('\t', "\\t");
                // `Select(...).Aggregate(...)` over no characters throws.
                if text.is_empty() {
                    return None;
                }
                let mut out = String::with_capacity(text.len());
                for c in text.chars() {
                    if (c as u32) < 32 || (c as u32) > 127 {
                        out.push_str(&format!("\\x{:02X}", c as u32));
                    } else {
                        out.push(c);
                    }
                }
                out
            }
            // `null` joins as nothing.
            other => other.item_text().unwrap_or_default(),
        })
    }

    /// The field as `DFItem.items` writes it: a number in the invariant culture, bytes as ASCII
    /// with the NULs trimmed, anything else by its `ToString`, and `null` for an unknown field.
    /// `// C#: ExtLibs/Utilities/DFLog.cs:86-93, BinaryLog.cs:75-78`
    #[must_use]
    pub fn item_text(&self) -> Option<String> {
        Some(match self {
            Self::Text(text) => text.clone(),
            Self::Int(value) => value.to_string(),
            Self::UInt(value) => value.to_string(),
            Self::Single(value) => netfmt::single(*value),
            Self::Double(value) => netfmt::double(*value),
            Self::Bytes(bytes) => netfmt::trim_nul(&netfmt::ascii(bytes)).to_owned(),
            // `UnionArray.ToString`: `_shorts.Take(32)` over a `byte[]` seen through a `short[]`
            // field (`BinaryLog.cs:40-78`). The runtime enumerates the array it really is, so what
            // comes out is its first 32 bytes, each as a number - `[0 128 ...]` for -32768 - and
            // that is what Mission Planner writes under mono, the oracle here. (.NET Framework
            // dispatches the enumeration on the real type too; that it fails there rather than
            // printing the same is likely and not verified.)
            Self::Array(bytes) => {
                let joined: Vec<String> = bytes.iter().take(32).map(ToString::to_string).collect();
                format!("[{}]", joined.join(" "))
            }
            Self::Null => return None,
        })
    }
}

/// The firmware a line of text names, by `ReadMessage`'s tests in its order, or `None` to leave
/// the guess as it was. `// C#: ExtLibs/Utilities/BinaryLog.cs:178-201`
fn firmware_named(line: &str) -> Option<Firmware> {
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

/// `HalfHelper.HalfushortToSingle`: IEEE half to single through van der Zijp's tables.
/// `// C#: ExtLibs/Utilities/Half/HalfHelper.cs:24-88, 177-181`
fn half_to_single(value: u16) -> f32 {
    let exponent = u32::from(value >> 10);
    let offset: u32 = if exponent == 0 || exponent == 32 {
        0
    } else {
        1024
    };
    let index = offset + u32::from(value & 0x3ff);
    let mantissa = match index {
        0 => 0,
        1..=1023 => {
            let mut m = index << 13;
            let mut e: u32 = 0;
            while m & 0x0080_0000 == 0 {
                e = e.wrapping_sub(0x0080_0000);
                m <<= 1;
            }
            m &= !0x0080_0000;
            e = e.wrapping_add(0x3880_0000);
            m | e
        }
        _ => 0x3800_0000 + ((index - 1024) << 13),
    };
    let exponent = match exponent {
        0 => 0,
        1..=30 => exponent << 23,
        31 => 0x4780_0000,
        32 => 0x8000_0000,
        63 => 0xC780_0000,
        _ => 0x8000_0000 + ((exponent - 32) << 23),
    };
    f32::from_bits(mantissa.wrapping_add(exponent))
}

/// Reads `N` bytes little-endian at `offset`, or `None` where `BitConverter` throws.
fn bytes_at<const N: usize>(message: &[u8], offset: usize) -> Option<[u8; N]> {
    message.get(offset..offset.checked_add(N)?)?.try_into().ok()
}

/// Mission Planner's dataflash reader, with the state it keeps between messages: the formats seen,
/// the per-type cache and the firmware the text has named.
/// `// C#: ExtLibs/Utilities/BinaryLog.cs:17-602`
#[derive(Debug, Clone)]
pub struct BinaryLog {
    /// `logformat`: by name, in the order the names were first seen.
    logformat: Vec<LogFormat>,
    /// `packettypecache`, by type.
    cache: Vec<CacheEntry>,
    /// `_firmware`; `None` is the empty string.
    firmware: Option<Firmware>,
}

impl Default for BinaryLog {
    fn default() -> Self {
        Self::new()
    }
}

impl BinaryLog {
    /// A reader that has seen nothing.
    #[must_use]
    pub fn new() -> Self {
        Self {
            logformat: Vec::new(),
            cache: vec![CacheEntry::default(); 256],
            firmware: None,
        }
    }

    /// The firmware the text so far has named.
    #[must_use]
    pub const fn firmware(&self) -> Option<Firmware> {
        self.firmware
    }

    /// `logformat[name] = fmt`: replaces in place, or adds at the end.
    fn remember_format(&mut self, format: LogFormat) {
        match self.logformat.iter_mut().find(|f| f.name == format.name) {
            Some(slot) => *slot = format,
            None => self.logformat.push(format),
        }
    }

    fn cache_entry(&self, msg_type: u8) -> Option<&CacheEntry> {
        self.cache.get(usize::from(msg_type))
    }

    fn set_cache(&mut self, format: &LogFormat) {
        if let Some(slot) = self.cache.get_mut(usize::from(format.msg_type)) {
            *slot = CacheEntry {
                length: format.length,
                name: format.name.clone(),
                format: format.format.clone(),
            };
        }
    }

    /// Reads an `FMT` body - short at the end of the file, and zero-filled - and remembers it.
    fn read_fmt(&mut self, data: &[u8], pos: &mut usize) -> (LogFormat, String) {
        let body = read_body(data, pos, FMT_BODY);
        let text = |range: std::ops::Range<usize>| {
            netfmt::trim_nul(&netfmt::ascii(body.get(range).unwrap_or_default())).to_owned()
        };
        let format = LogFormat {
            msg_type: body.first().copied().unwrap_or(0),
            length: body.get(1).copied().unwrap_or(0),
            name: text(2..6),
            format: text(6..22),
        };
        let labels = text(22..86);
        self.remember_format(format.clone());
        (format, labels)
    }

    /// `ReadMessage`: the next message from `*pos` as a line of text, `"\r\n"` included, or the
    /// empty string if none starts before `length`.
    /// `// C#: ExtLibs/Utilities/BinaryLog.cs:123-215`
    pub fn read_message(
        &mut self,
        data: &[u8],
        pos: &mut usize,
        length: usize,
        mode_name: ModeName<'_>,
    ) -> String {
        let mut step = 0;
        while *pos < length {
            let Some(&byte) = data.get(*pos) else { break };
            *pos += 1;
            match step {
                0 => {
                    if byte == HEAD_BYTE1 {
                        step = 1;
                    }
                }
                1 => step = if byte == HEAD_BYTE2 { 2 } else { 0 },
                _ => {
                    step = 0;
                    let Some(fields) = self.log_entry_objects(byte, data, pos, mode_name) else {
                        continue;
                    };
                    let Some(texts) = fields
                        .iter()
                        .map(Field::line_text)
                        .collect::<Option<Vec<String>>>()
                    else {
                        continue;
                    };
                    let line = texts.join(", ") + "\r\n";
                    if let Some(firmware) = firmware_named(&line) {
                        self.firmware = Some(firmware);
                    }
                    return line;
                }
            }
        }
        String::new()
    }

    /// `ReadMessageObjects`: the next message's fields, or `None` if none starts before `length`.
    /// `// C#: ExtLibs/Utilities/BinaryLog.cs:343-391`
    pub fn read_message_objects(
        &mut self,
        data: &[u8],
        pos: &mut usize,
        length: usize,
        mode_name: ModeName<'_>,
    ) -> Option<Vec<Field>> {
        let mut step = 0;
        while *pos < length {
            let byte = *data.get(*pos)?;
            *pos += 1;
            match step {
                0 => {
                    if byte == HEAD_BYTE1 {
                        step = 1;
                    }
                }
                1 => step = if byte == HEAD_BYTE2 { 2 } else { 0 },
                _ => {
                    step = 0;
                    if let Some(fields) = self.log_entry_objects(byte, data, pos, mode_name) {
                        return Some(fields);
                    }
                }
            }
        }
        None
    }

    /// `ReadMessageTypeOffset`: the next message's type and where its header starts, having read an
    /// `FMT` into the cache or stepped over a known type's body; `(0, 0)` if none starts before
    /// `length`. A type with no format is returned too, with only its header consumed.
    /// `// C#: ExtLibs/Utilities/BinaryLog.cs:218-341`
    pub fn read_message_type_offset(
        &mut self,
        data: &[u8],
        pos: &mut usize,
        length: usize,
    ) -> (u8, usize) {
        let mut step = 0;
        while *pos < length {
            let Some(&byte) = data.get(*pos) else { break };
            *pos += 1;
            match step {
                0 => {
                    if byte == HEAD_BYTE1 {
                        step = 1;
                    }
                }
                1 => step = if byte == HEAD_BYTE2 { 2 } else { 0 },
                _ => {
                    step = 0;
                    let start = *pos - 3;
                    // logEntryFMT
                    if byte == FMT_TYPE {
                        let (format, _) = self.read_fmt(data, pos);
                        self.set_cache(&format);
                        return (byte, start);
                    }
                    let size = self.cache_entry(byte).map_or(0, |c| usize::from(c.length));
                    if size == 0 {
                        return (byte, start);
                    }
                    // `new byte[size - 3]` of a negative size throws, and the catch goes on
                    // scanning.
                    if size < 3 {
                        continue;
                    }
                    read_body(data, pos, size - 3);
                    return (byte, start);
                }
            }
        }
        (0, 0)
    }

    /// `logEntryObjects`: the fields of a message of type `msg_type` whose body starts at `*pos`,
    /// or `None` where the C# returns `null` or throws.
    /// `// C#: ExtLibs/Utilities/BinaryLog.cs:393-478`
    fn log_entry_objects(
        &mut self,
        msg_type: u8,
        data: &[u8],
        pos: &mut usize,
        mode_name: ModeName<'_>,
    ) -> Option<Vec<Field>> {
        if msg_type == FMT_TYPE {
            let (format, labels) = self.read_fmt(data, pos);
            return Some(vec![
                Field::Text("FMT".to_owned()),
                Field::UInt(u64::from(format.msg_type)),
                Field::UInt(u64::from(format.length)),
                Field::Text(format.name),
                Field::Text(format.format),
                Field::Text(labels),
            ]);
        }
        let (name, format, size) = match self.cache_entry(msg_type) {
            Some(entry) if entry.length != 0 => (
                entry.name.clone(),
                entry.format.clone(),
                usize::from(entry.length),
            ),
            _ => {
                // Rebuild the whole cache from the formats seen so far; the last of them with
                // this type wins.
                let mut found = (String::new(), String::new(), 0usize);
                for format in self.logformat.clone() {
                    self.set_cache(&format);
                    if format.msg_type == msg_type {
                        found = (
                            format.name.clone(),
                            format.format.clone(),
                            usize::from(format.length),
                        );
                    }
                }
                found
            }
        };
        if size < 3 {
            // No format (`return null`), or `new byte[size - 3]` of a negative size.
            return None;
        }
        let body = read_body(data, pos, size - 3);
        self.process_message_objects(&body, name, &format, mode_name)
    }

    /// `ProcessMessageObjects`: the message name, then each field of `format` in turn.
    /// `// C#: ExtLibs/Utilities/BinaryLog.cs:480-498`
    fn process_message_objects(
        &self,
        message: &[u8],
        name: String,
        format: &str,
        mode_name: ModeName<'_>,
    ) -> Option<Vec<Field>> {
        let mut fields = Vec::with_capacity(format.len() + 1);
        fields.push(Field::Text(name));
        let mut offset = 0usize;
        for code in format.chars() {
            let (field, size) = self.object_from_message(code, message, offset, mode_name)?;
            fields.push(field);
            offset += size;
        }
        Some(fields)
    }

    /// `GetObjectFromMessage`: one field and its size, or `None` where the read runs past the body
    /// and `BitConverter` or the span throws.
    /// `// C#: ExtLibs/Utilities/BinaryLog.cs:500-573`
    fn object_from_message(
        &self,
        code: char,
        message: &[u8],
        offset: usize,
        mode_name: ModeName<'_>,
    ) -> Option<(Field, usize)> {
        Some(match code {
            'b' => (
                Field::Int(i64::from(i8::from_le_bytes(bytes_at(message, offset)?))),
                1,
            ),
            'B' => (Field::UInt(u64::from(*message.get(offset)?)), 1),
            'h' => (
                Field::Int(i64::from(i16::from_le_bytes(bytes_at(message, offset)?))),
                2,
            ),
            'H' => (
                Field::UInt(u64::from(u16::from_le_bytes(bytes_at(message, offset)?))),
                2,
            ),
            'i' => (
                Field::Int(i64::from(i32::from_le_bytes(bytes_at(message, offset)?))),
                4,
            ),
            'I' => (
                Field::UInt(u64::from(u32::from_le_bytes(bytes_at(message, offset)?))),
                4,
            ),
            'q' => (
                Field::Int(i64::from_le_bytes(bytes_at(message, offset)?)),
                8,
            ),
            'Q' => (
                Field::UInt(u64::from_le_bytes(bytes_at(message, offset)?)),
                8,
            ),
            'g' => (
                Field::Single(half_to_single(u16::from_le_bytes(bytes_at(
                    message, offset,
                )?))),
                2,
            ),
            'f' => (
                Field::Single(f32::from_le_bytes(bytes_at(message, offset)?)),
                4,
            ),
            'd' => (
                Field::Double(f64::from_le_bytes(bytes_at(message, offset)?)),
                8,
            ),
            'c' => (
                Field::Double(f64::from(i16::from_le_bytes(bytes_at(message, offset)?)) / 100.0),
                2,
            ),
            'C' => (
                Field::Double(f64::from(u16::from_le_bytes(bytes_at(message, offset)?)) / 100.0),
                2,
            ),
            'e' => (
                Field::Double(f64::from(i32::from_le_bytes(bytes_at(message, offset)?)) / 100.0),
                4,
            ),
            'E' => (
                Field::Double(f64::from(u32::from_le_bytes(bytes_at(message, offset)?)) / 100.0),
                4,
            ),
            'L' => (
                Field::Double(
                    f64::from(i32::from_le_bytes(bytes_at(message, offset)?)) / 10_000_000.0,
                ),
                4,
            ),
            'n' => (Field::Text(text_at::<4>(message, offset)?), 4),
            'N' => (Field::Text(text_at::<16>(message, offset)?), 16),
            'M' => {
                let number = *message.get(offset)?;
                let name = self
                    .firmware
                    .and_then(|firmware| mode_name(firmware, number))
                    .unwrap_or_else(|| number.to_string());
                (Field::Text(name), 1)
            }
            'Z' => (Field::Bytes(bytes_at::<64>(message, offset)?.to_vec()), 64),
            'a' => (Field::Array(bytes_at::<64>(message, offset)?.to_vec()), 64),
            _ => (Field::Null, 0),
        })
    }
}

/// `Encoding.ASCII.GetString(message, offset, N).Trim('\0')`.
fn text_at<const N: usize>(message: &[u8], offset: usize) -> Option<String> {
    let raw = bytes_at::<N>(message, offset)?;
    Some(netfmt::trim_nul(&netfmt::ascii(&raw)).to_owned())
}

/// `Stream.Read` of `count` bytes into a fresh array: what the file still holds, the rest zeros.
fn read_body(data: &[u8], pos: &mut usize, count: usize) -> Vec<u8> {
    let mut body = vec![0u8; count];
    let available = data.len().saturating_sub(*pos).min(count);
    if let (Some(target), Some(source)) =
        (body.get_mut(..available), data.get(*pos..*pos + available))
    {
        target.copy_from_slice(source);
    }
    *pos += available;
    body
}

/// Convert .Bin to .Log: the whole of a dataflash log as Mission Planner's text, what
/// `but_bintolog_Click` writes for each file. The DataFlash Logs page's "Convert .Bin to .Log"
/// button (`but_bintolog`) calls this through [`convert_bin_file`].
///
/// `mode_name` names a flight mode for the firmware the log has named so far - the application's
/// `BinaryLog.onFlightMode` - and [`no_mode_names`] leaves every mode a number.
/// `// C#: ExtLibs/Utilities/BinaryLog.cs:89-119; GCSViews/FlightData.cs:1082-1098`
#[must_use]
pub fn convert_bin(data: &[u8], mode_name: ModeName<'_>) -> Vec<u8> {
    let mut log = BinaryLog::new();
    let mut pos = 0usize;
    let length = data.len();
    let mut out = Vec::with_capacity(data.len() * 2);
    while pos < length {
        let line = log.read_message(data, &mut pos, length, mode_name);
        // `ASCIIEncoding.ASCII.GetBytes`: the text is ASCII already, every byte having been decoded
        // as ASCII on the way in.
        out.extend(line.bytes().map(|b| if b < 0x80 { b } else { b'?' }));
    }
    out
}

/// Where "Convert .Bin to .Log" writes a log's text: beside it, its extension replaced by `.log`.
/// `// C#: GCSViews/FlightData.cs:1093-1094`
#[must_use]
pub fn log_path_for(bin: &Path) -> PathBuf {
    bin.with_extension("log")
}

/// "Convert .Bin to .Log" (`but_bintolog`) for one file: reads `input` and writes its text to
/// `output`, which the button makes with [`log_path_for`].
///
/// # Errors
///
/// The file cannot be read or the text cannot be written.
/// `// C#: GCSViews/FlightData.cs:1082-1098`
pub fn convert_bin_file(
    input: &Path,
    output: &Path,
    mode_name: ModeName<'_>,
) -> std::io::Result<()> {
    let data = std::fs::read(input)?;
    std::fs::write(output, convert_bin(&data, mode_name))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fmt(msg_type: u8, length: u8, name: &str, format: &str, labels: &str) -> Vec<u8> {
        let mut out = vec![HEAD_BYTE1, HEAD_BYTE2, FMT_TYPE, msg_type, length];
        let mut field = |text: &str, width: usize| {
            let mut bytes = text.as_bytes().to_vec();
            bytes.resize(width, 0);
            out.extend(bytes);
        };
        field(name, 4);
        field(format, 16);
        field(labels, 64);
        out
    }

    fn text(data: &[u8]) -> String {
        String::from_utf8(convert_bin(data, &no_mode_names)).unwrap()
    }

    #[test]
    fn an_fmt_line_is_its_own_declaration() {
        let data = fmt(0x80, 89, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns");
        assert_eq!(
            text(&data),
            "FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\r\n"
        );
    }

    #[test]
    fn fields_are_formatted_as_dotnet_formats_them() {
        let mut data = fmt(10, 3 + 2 + 4 + 4 + 2 + 8, "TST", "cLfgq", "A,B,C,D,E");
        data.extend([HEAD_BYTE1, HEAD_BYTE2, 10]);
        data.extend((-1234i16).to_le_bytes());
        data.extend((-353_632_621i32).to_le_bytes());
        data.extend(0.999_999_97f32.to_le_bytes());
        data.extend(0x3C00u16.to_le_bytes());
        data.extend((-5i64).to_le_bytes());
        let text = text(&data);
        assert!(
            text.ends_with("TST, -12.34, -35.3632621, 0.9999999, 1, -5\r\n"),
            "{text}"
        );
    }

    #[test]
    fn an_all_nul_string_drops_the_message() {
        let mut data = fmt(11, 3 + 64, "MSG", "Z", "Message");
        data.extend([HEAD_BYTE1, HEAD_BYTE2, 11]);
        data.extend([0u8; 64]);
        data.extend([HEAD_BYTE1, HEAD_BYTE2, 11]);
        let mut message = b"a\tb\\c\x01".to_vec();
        message.resize(64, 0);
        data.extend(message);
        let text = text(&data);
        assert!(
            text.ends_with("Message\r\nMSG, a\\tb\\\\c\\x01\r\n"),
            "{text}"
        );
    }

    #[test]
    fn a_header_after_a_lone_first_byte_is_missed() {
        let mut data = fmt(12, 4, "ONE", "B", "V");
        data.extend([HEAD_BYTE1, HEAD_BYTE1, HEAD_BYTE2, 12, 7]);
        data.extend([HEAD_BYTE1, HEAD_BYTE2, 12, 8]);
        let text = text(&data);
        assert!(text.ends_with("V\r\nONE, 8\r\n"), "{text}");
    }

    #[test]
    fn a_mode_is_named_once_the_log_names_its_firmware() {
        let mut data = fmt(13, 4, "MODE", "M", "Mode");
        data.extend(fmt(14, 3 + 64, "MSG", "Z", "Message"));
        data.extend([HEAD_BYTE1, HEAD_BYTE2, 13, 5]);
        data.extend([HEAD_BYTE1, HEAD_BYTE2, 14]);
        let mut message = b"ArduCopter V4.5.0".to_vec();
        message.resize(64, 0);
        data.extend(message);
        data.extend([HEAD_BYTE1, HEAD_BYTE2, 13, 5]);
        let names = |firmware: Firmware, mode: u8| {
            (firmware == Firmware::Copter && mode == 5).then(|| "Loiter".to_owned())
        };
        let text = String::from_utf8(convert_bin(&data, &names)).unwrap();
        assert!(
            text.ends_with("MODE, 5\r\nMSG, ArduCopter V4.5.0\r\nMODE, Loiter\r\n"),
            "{text}"
        );
    }

    #[test]
    fn half_floats_convert_exactly() {
        assert_eq!(half_to_single(0x3C00), 1.0);
        assert_eq!(half_to_single(0xC000), -2.0);
        assert_eq!(half_to_single(0x0001), 5.960_464_5e-8);
        assert_eq!(half_to_single(0x7C00), f32::INFINITY);
        assert!(half_to_single(0x7E00).is_nan());
        assert_eq!(half_to_single(0x8000).to_bits(), 0x8000_0000);
    }
}
