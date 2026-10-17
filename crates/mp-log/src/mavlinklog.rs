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

//! What the flight screen's "Tlog > Kml or Graph" form (`Log/MavlinkLog.cs`) reads out of a
//! telemetry log: every packet as `DebugPacket` writes it (Convert to Text, Convert to CSV), every
//! number of every message against time (Graph Log), and the vehicle's state at each position
//! (Create KML + GPX, written by `mp_kml::tlog`).
//!
//! The form reads a log through a `MAVLinkInterface` in log-read mode, one `readPacketAsync` at a
//! time while the file has bytes left (`Log/MavlinkLog.cs:118, 241, 438, 1070`). [`LogReader`] is
//! that reading, which is not [`crate::TlogReader`]'s (`MavlinkParse.ReadPacket`'s):
//!
//! - a record is `readlogPacketMavlink`'s: eight bytes of timestamp - read again as the frame when
//!   the first of them is a start byte, believed under 9,999,999 hours - then the frame from its
//!   start byte, scanned forward to when it is not the next byte however far it is, cut to the
//!   length its header gives and read short, zeros after, at the end of the file;
//! - the reader's clock, `lastlogread`, is the last believed stamp, `DateTime.MinValue` before any;
//! - `readPacketAsync` hands on a frame whose checksum passes, and returns `MAVLinkMessage.Invalid`
//!   for a MAVLink 1 frame of the wrong length, an unsupported incompatibility flag or a checksum
//!   that fails - except that a failing frame with a MAVLink 2 start byte in its last twenty bytes
//!   sends it back for the next record within the same read, as its `goto readloop` does in log
//!   mode (the rewound buffer it builds is not read: log mode reads the file again).
//!
//! `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4655-5009, 6472-6614`

use std::collections::{BTreeMap, HashMap, HashSet};
use std::fmt::Write as _;

use base64::Engine as _;
use mp_mavlink::{FieldInfo, FieldValue, parse};
use mp_mavlink_dialects::all::{DIALECT, MavMessage};
/// The reader's clock: `CurrentState.datetime`'s type, for the crates that write what is read here.
pub use mp_vehicle::DateTime;
use mp_vehicle::{VehicleId, VehicleRegistry};

use crate::dflogbuffer::civil_from_days;
use crate::netfmt;

/// A local time zone: the offset from UTC, in seconds, at a UTC instant in Unix milliseconds -
/// `ToLocalTime`'s. The shape of `mp_kml::dflog::Zone`, so the application's `local_zone` serves
/// both.
pub type Zone<'a> = &'a dyn Fn(i64) -> i32;

/// UTC as the zone: what the tests use.
#[must_use]
pub fn utc(_: i64) -> i32 {
    0
}

/// MAVLink 1's start byte, `MAVLINK_STX_MAVLINK1`.
const STX_V1: u8 = 0xFE;
/// MAVLink 2's start byte, `MAVLINK_STX`.
const STX_V2: u8 = 0xFD;
/// `MAVLINK_IFLAG_SIGNED`, the one incompatibility flag supported (`MAVLINK_SUPPORTED_IFLAGS`).
const IFLAG_SIGNED: u8 = 0x01;
/// `MAVLINK_SIGNATURE_BLOCK_LEN`.
const SIGNATURE_LEN: usize = 13;
/// How many bytes from its end a failing frame is searched for a MAVLink 2 start byte.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4995`
const REWIND_SEARCH: usize = 20;
/// `if (readcount > 300) break;`: the records one read may go through.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4690-4695`
const READS_PER_CALL: usize = 301;
/// `getHeartBeat`'s reads: it gives up after its two hundred and first.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1199`
const HEARTBEAT_READS: usize = 200;
/// `MAV_TYPE.GCS`.
const MAV_TYPE_GCS: u8 = 6;
/// `gcssysid`, the ground station's system id, and 253, which log playback treats as one too.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5105`
const GCS_SYSIDS: [u32; 2] = [255, 253];
/// `MAV_MODE_FLAG.CUSTOM_MODE_ENABLED`.
const CUSTOM_MODE_ENABLED: u8 = 1;
/// `MAV_MISSION_TYPE.MISSION`.
const MISSION: u8 = 0;

/// `new DateTime(1970, 1, 1).Ticks`.
const UNIX_EPOCH_TICKS: i64 = 621_355_968_000_000_000;
/// `TimeSpan.TicksPerMillisecond`.
const TICKS_PER_MS: i64 = 10_000;
/// `TimeSpan.TicksPerSecond`.
const TICKS_PER_SECOND: i64 = 10_000_000;
/// `TimeSpan.TicksPerDay`.
const TICKS_PER_DAY: i64 = 864_000_000_000;
/// `new DateTime(1899, 12, 30).Ticks`: day 0 of an `XDate`.
const XDATE_EPOCH_TICKS: i64 = 599_264_352_000_000_000;
/// Milliseconds in a day.
const MS_PER_DAY: f64 = 86_400_000.0;

// ---- The reader's clock ------------------------------------------------------------------------

/// `lastlogread` as the C# holds it, in ticks: `date1.ToLocalTime()` of the record's UTC time, so
/// local; or `DateTime.MinValue` as it is, before any record has given a time.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:485, 6516-6518`
#[must_use]
pub fn local_ticks(time: DateTime, zone: Zone<'_>) -> i64 {
    let ticks = time.ticks();
    if ticks == 0 {
        return 0;
    }
    let utc_ms = (ticks - UNIX_EPOCH_TICKS).div_euclid(TICKS_PER_MS);
    ticks + i64::from(zone(utc_ms)) * TICKS_PER_SECOND
}

/// A time in ticks as its calendar parts: year, month, day, hour, minute, second, millisecond.
fn parts(ticks: i64) -> (i64, u32, u32, i64, i64, i64, i64) {
    let days = ticks.div_euclid(TICKS_PER_DAY);
    let ms = ticks.rem_euclid(TICKS_PER_DAY) / TICKS_PER_MS;
    let (year, month, day) = civil_from_days(days - UNIX_EPOCH_TICKS / TICKS_PER_DAY);
    (
        year,
        month,
        day,
        ms / 3_600_000,
        ms / 60_000 % 60,
        ms / 1000 % 60,
        ms % 1000,
    )
}

/// `DateTime.ToString()` in English, the general format `M/d/yyyy h:mm:ss tt`, of a time in ticks.
#[must_use]
pub fn general_date(ticks: i64) -> String {
    let (year, month, day, hour, minute, second, _) = parts(ticks);
    let (twelve, half) = match hour {
        0 => (12, "AM"),
        1..=11 => (hour, "AM"),
        12 => (12, "PM"),
        _ => (hour - 12, "PM"),
    };
    format!("{month}/{day}/{year:04} {twelve}:{minute:02}:{second:02} {half}")
}

/// `DateTime.ToString("yyyy-MM-ddTHH:mm:ss.fff")` of a time in ticks.
#[must_use]
pub fn iso_millis(ticks: i64) -> String {
    let (year, month, day, hour, minute, second, ms) = parts(ticks);
    format!("{year:04}-{month:02}-{day:02}T{hour:02}:{minute:02}:{second:02}.{ms:03}")
}

/// `new XDate(time)`: a local time in ticks as days since 1899-12-30, to the millisecond, as
/// `CalendarDateToXLDate` takes a `DateTime`'s parts.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/XDate.cs (XDate(DateTime))`
#[must_use]
#[allow(clippy::cast_precision_loss)] // milliseconds since 1899: well inside f64's 53 bits
pub fn xdate(ticks: i64) -> f64 {
    let ms = (ticks - XDATE_EPOCH_TICKS).div_euclid(TICKS_PER_MS);
    ms as f64 / MS_PER_DAY
}

// ---- Reading ------------------------------------------------------------------------------------

/// A packet `readPacketAsync` hands on, as its `MAVLinkMessage` holds it.
#[derive(Debug, Clone, PartialEq)]
pub struct Read {
    /// `header`: the start byte, `0xFD` or `0xFE`.
    pub header: u8,
    /// `payloadlength`, as the frame carries it (a MAVLink 2 payload trimmed of its zeros).
    pub payload_length: u8,
    /// `incompat_flags`, 0 on MAVLink 1.
    pub incompat_flags: u8,
    /// `compat_flags`, 0 on MAVLink 1.
    pub compat_flags: u8,
    /// `seq`.
    pub seq: u8,
    /// `sysid`.
    pub sysid: u32,
    /// `compid`.
    pub compid: u8,
    /// `msgid`.
    pub msgid: u32,
    /// `Length`: the frame's bytes, start byte through signature.
    pub length: usize,
    /// `crc16`, as the frame carries it.
    pub crc16: u16,
    /// `sig`: the signature block of a signed frame.
    pub signature: Option<Vec<u8>>,
    /// `data`: the message decoded, `None` for one this build does not know, whose `data` is null.
    pub message: Option<MavMessage>,
    /// `buffer`: the frame's bytes.
    pub raw: Vec<u8>,
}

/// One `readPacketAsync` of the log: the reader's clock after it, `lastlogread`, as UTC (see
/// [`local_ticks`]), and the packet - `None` for `MAVLinkMessage.Invalid`.
#[derive(Debug, Clone, PartialEq)]
pub struct Packet {
    /// `lastlogread`, before `ToLocalTime`; [`DateTime::MIN`] before any record gave a time.
    pub time: DateTime,
    /// The packet handed on.
    pub read: Option<Read>,
}

/// One record as `readlogPacketMavlink` reads it: the stamp it believed, the frame's bytes - as
/// long as its header says, zeros past the end of the file - and where the next record starts.
struct Record {
    stamp: Option<u64>,
    frame: Option<Vec<u8>>,
    end: usize,
}

/// The record at `start`. A start byte among the stamp's first is the frame's, as `datearray[7]`
/// after the reverse is the first byte read; the lost-sync scan reads on to the next start byte;
/// an unsupported incompatibility flag is the C#'s `InvalidDataException`, after the third byte,
/// which `readPacketAsync` catches and returns `Invalid` for; the end of the file before the
/// frame's third byte is `Invalid` too.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6472-6614`
fn record_at(data: &[u8], start: usize) -> Record {
    let mut pos = start;
    let mut stamp_bytes = [0u8; 8];
    let got = data.len().saturating_sub(pos).min(8);
    if let (Some(into), Some(from)) = (stamp_bytes.get_mut(..got), data.get(pos..pos + got)) {
        into.copy_from_slice(from);
    }
    pos += got;
    let mut stamp = None;
    if matches!(stamp_bytes[0], STX_V1 | STX_V2) {
        // `Seek(-8, SeekOrigin.Current)`: the eight bytes read again as the frame.
        pos = start;
    } else {
        let micros = u64::from_be_bytes(stamp_bytes);
        if micros / 1000 / 1000 / 60 / 60 < 9_999_999 {
            stamp = Some(micros);
        }
    }
    let invalid = |stamp, end| Record {
        stamp,
        frame: None,
        end,
    };
    // Byte 0, and the lost-sync scan to the next start byte.
    let Some(mut stx) = data.get(pos).copied() else {
        return invalid(stamp, data.len());
    };
    pos += 1;
    while stx != STX_V1 && stx != STX_V2 {
        let Some(byte) = data.get(pos).copied() else {
            return invalid(stamp, data.len());
        };
        stx = byte;
        pos += 1;
    }
    // Byte 1, the payload's length: the frame's, with its header and checksum.
    let Some(payload) = data.get(pos).copied() else {
        return invalid(stamp, data.len());
    };
    pos += 1;
    let header = if stx == STX_V2 { 9 } else { 5 };
    let mut length = usize::from(payload) + header + 1 + 2;
    // Byte 2: on MAVLink 2 the incompatibility flags, and the signature they may add.
    let Some(flags) = data.get(pos).copied() else {
        return invalid(stamp, data.len());
    };
    pos += 1;
    if stx == STX_V2 {
        if flags & !IFLAG_SIGNED != 0 {
            return invalid(stamp, pos);
        }
        if flags & IFLAG_SIGNED != 0 {
            length += SIGNATURE_LEN;
        }
    }
    let mut frame = vec![0u8; length];
    if let Some(head) = frame.get_mut(..3) {
        head.copy_from_slice(&[stx, payload, flags]);
    }
    let available = data.len().saturating_sub(pos).min(length - 3);
    if let (Some(into), Some(from)) = (
        frame.get_mut(3..3 + available),
        data.get(pos..pos + available),
    ) {
        into.copy_from_slice(from);
    }
    pos += available;
    Record {
        stamp,
        frame: Some(frame),
        end: pos,
    }
}

/// What `readPacketAsync` makes of a record's frame.
enum Outcome {
    /// A packet, handed on.
    Valid(Box<Read>),
    /// `MAVLinkMessage.Invalid`.
    Invalid,
    /// A failing frame with a MAVLink 2 start byte near its end: on to the next record.
    Again,
}

/// The checks `readPacketAsync` makes, in its order: a MAVLink 1 frame whose length is not its
/// message's (a message the table does not know passes), then the checksum - its failure looked
/// at for a MAVLink 2 start byte in the last twenty bytes, from the second byte at least.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4943-5009`
fn outcome(frame: &[u8]) -> Outcome {
    use mp_mavlink::Dialect as _;
    if frame.first() == Some(&STX_V1)
        && let (Some(&length), Some(&msgid)) = (frame.get(1), frame.get(5))
        && let Some(info) = DIALECT.info(u32::from(msgid))
        && info.len != 0
        && length != info.min_len
    {
        return Outcome::Invalid;
    }
    match parse(frame, &DIALECT) {
        Ok((parsed, _)) => Outcome::Valid(Box::new(Read {
            header: parsed.raw.first().copied().unwrap_or(0),
            payload_length: parsed.raw.get(1).copied().unwrap_or(0),
            incompat_flags: parsed.incompat_flags,
            compat_flags: parsed.compat_flags,
            seq: parsed.seq,
            sysid: parsed.sysid,
            compid: parsed.compid,
            msgid: parsed.msgid,
            length: parsed.raw.len(),
            crc16: parsed.checksum,
            signature: parsed.signature.map(<[u8]>::to_vec),
            message: MavMessage::decode(parsed.msgid, parsed.payload),
            raw: parsed.raw.to_vec(),
        })),
        Err(mp_mavlink::ParseError::UnsupportedIncompatFlags { .. }) => Outcome::Invalid,
        Err(_) => {
            let from = frame.len().saturating_sub(REWIND_SEARCH).max(1);
            if frame.get(from..).is_some_and(|tail| tail.contains(&STX_V2)) {
                Outcome::Again
            } else {
                Outcome::Invalid
            }
        }
    }
}

/// A telemetry log read as the form reads it: one [`Packet`] a `readPacketAsync`, while the file
/// has bytes left.
#[derive(Debug)]
pub struct LogReader<'a> {
    data: &'a [u8],
    pos: usize,
    time: DateTime,
}

impl<'a> LogReader<'a> {
    /// A log already in memory, read from its start.
    #[must_use]
    pub const fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            pos: 0,
            time: DateTime::MIN,
        }
    }

    /// `(int)((float)Position / (float)Length * 100.0f)`: how far through the file the reader
    /// is, in percent, as the form's progress bar shows it before each read.
    #[must_use]
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss
    )] // a percentage of a file's length
    pub fn percent(&self) -> u8 {
        if self.data.is_empty() {
            return 0;
        }
        (self.pos as f32 / self.data.len() as f32 * 100.0) as u8
    }

    /// The next `readPacketAsync`, or `None` once the file has no bytes left.
    pub fn next_packet(&mut self) -> Option<Packet> {
        if self.pos >= self.data.len() {
            return None;
        }
        for _ in 0..READS_PER_CALL {
            let record = record_at(self.data, self.pos);
            self.pos = record.end.max(self.pos + 1);
            if let Some(time) = record.stamp.and_then(DateTime::from_tlog_micros) {
                self.time = time;
            }
            let found = match record.frame {
                Some(frame) => outcome(&frame),
                None => Outcome::Invalid,
            };
            match found {
                Outcome::Valid(read) => {
                    return Some(Packet {
                        time: self.time,
                        read: Some(*read),
                    });
                }
                Outcome::Invalid => break,
                Outcome::Again if self.pos >= self.data.len() => {
                    // The next record at the end of the file: eight bytes read as none, a stamp
                    // of 0 - 1970 - and no frame.
                    if let Some(epoch) = DateTime::from_tlog_micros(0) {
                        self.time = epoch;
                    }
                    break;
                }
                Outcome::Again => {}
            }
        }
        Some(Packet {
            time: self.time,
            read: None,
        })
    }

    /// The next packet, `progress` told the percentage first when it has changed - the form's
    /// `if (progressBar1.Value != percent)` before each read.
    pub fn next_with_progress(&mut self, progress: &mut dyn FnMut(u8)) -> Option<Packet> {
        progress(self.percent());
        self.next_packet()
    }
}

/// Every packet of a log, in order: [`LogReader`] read to the end.
#[must_use]
pub fn packets(data: &[u8]) -> Vec<Packet> {
    let mut reader = LogReader::new(data);
    std::iter::from_fn(|| reader.next_packet()).collect()
}

// ---- DebugPacket --------------------------------------------------------------------------------

/// The C# struct's `Type.Name`: `mavlink_heartbeat_t` for `HEARTBEAT`.
#[must_use]
pub fn struct_name(message: &str) -> String {
    format!("mavlink_{}_t", message.to_lowercase())
}

/// A scalar field's `fieldValue.ToString()`: an integer in full, a `float` to seven significant
/// digits and a `double` to fifteen, as the .NET Framework writes them.
fn scalar_text(info: &FieldInfo, value: &FieldValue) -> String {
    match value {
        FieldValue::Unsigned(value) => value.to_string(),
        FieldValue::Signed(value) => value.to_string(),
        FieldValue::Float(value) if info.base_type == "double" => netfmt::double(*value),
        #[allow(clippy::cast_possible_truncation)] // a float field's value, widened exactly
        FieldValue::Float(value) => netfmt::single(*value as f32),
        _ => String::new(),
    }
}

/// An array field's text as `DebugPacket` writes it: a `byte[]` - the C#'s `char` and `uint8_t`
/// arrays (`mavgen_cs.py`) - as its characters up to the first NUL, each byte the character of
/// that code; a `short[]` as each element up to the first 0 in hexadecimal and a `|`,
/// `Convert.ToString(fiel, 16)`, a negative one in sixteen bits; any other array as nothing.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3612-3684`
fn array_text(info: &FieldInfo, value: &FieldValue) -> String {
    match (info.base_type, value) {
        ("char" | "uint8_t", FieldValue::UnsignedArray(bytes)) => bytes
            .iter()
            .take_while(|byte| **byte != 0)
            .map(|byte| u8::try_from(*byte).map_or('?', char::from))
            .collect(),
        ("int16_t", FieldValue::SignedArray(values)) => values
            .iter()
            .take_while(|value| **value != 0)
            .map(|value| {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                // an int16 field's value, as its sixteen bits
                let bits = *value as i16 as u16;
                format!("{bits:x}|")
            })
            .collect(),
        _ => String::new(),
    }
}

/// `DebugPacket(packet, ref text, PrintToConsole, delimeter)`'s text: the header's numbers in
/// hexadecimal, `{0,2:X}` each and `{7,6:X}` the message id; the struct's name; each field's name
/// and value; then `sig` and the signature in base 64, `Len` and the frame's length, `crc16` and
/// its checksum, and `\r\n` - every part followed by the delimiter. `None` where the C#'s text is
/// left empty: a frame of five bytes or fewer, or a message this build cannot decode, whose null
/// `data` throws and is caught.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3586-3713`
#[must_use]
pub fn debug_text(read: &Read, delimiter: &str) -> Option<String> {
    if read.length <= 5 {
        return None;
    }
    let message = read.message.as_ref()?;
    let d = delimiter;
    let mut out = format!(
        "{:>2X}{d}{:>2X}{d}{:>2X}{d}{:>2X}{d}{:>2X}{d}{:>2X}{d}{:>2X}{d}{:>6X}{d}",
        read.header,
        read.payload_length,
        read.incompat_flags,
        read.compat_flags,
        read.seq,
        read.sysid,
        read.compid,
        read.msgid,
    );
    out.push_str(&struct_name(message.name()));
    out.push_str(d);
    for (info, (name, value)) in message.field_info().iter().zip(message.fields()) {
        out.push_str(name);
        out.push_str(d);
        if info.array_len > 0 {
            out.push_str(&array_text(info, &value));
        } else {
            out.push_str(&scalar_text(info, &value));
        }
        out.push_str(d);
    }
    let sig = read
        .signature
        .as_deref()
        .map(|sig| base64::engine::general_purpose::STANDARD.encode(sig))
        .unwrap_or_default();
    let _ = write!(
        out,
        "{d}sig {sig}{d}Len{d}{}{d}crc16{d}{}\r\n",
        read.length, read.crc16
    );
    Some(out)
}

/// `Environment.NewLine`: `\r\n` on Windows, `\n` under Mono and here elsewhere.
pub const NEWLINE: &str = if cfg!(windows) { "\r\n" } else { "\n" };

/// Convert to Text's file: for each read, `lastlogread` as `DateTime.ToString()` writes it, a
/// space, and [`debug_text`] with a space between - nothing after the space where the read was
/// `Invalid` or its text empty, so the next read's time follows on the same line, as the C#
/// writes it. `progress` is told the percentage before each read.
/// `// C#: Log/MavlinkLog.cs:200-271`
#[must_use]
pub fn text_file(data: &[u8], zone: Zone<'_>, progress: &mut dyn FnMut(u8)) -> String {
    let mut reader = LogReader::new(data);
    let mut out = String::new();
    while let Some(packet) = reader.next_with_progress(progress) {
        let text = packet
            .read
            .as_ref()
            .and_then(|read| debug_text(read, " "))
            .unwrap_or_default();
        out.push_str(&general_date(local_ticks(packet.time, zone)));
        out.push(' ');
        out.push_str(&text);
    }
    out
}

/// `MAVFtp.FTPPayloadHeader.ToString()`, `JsonConvert.SerializeObject(this)`: the header's fields
/// laid out `Pack = 1` over the payload - its opcodes the numbers Json.NET writes an enum as - and
/// the 239 bytes of data in base 64.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVFtp.cs:2402-2457`
#[must_use]
pub fn ftp_header_json(payload: &[u8]) -> String {
    let byte = |at: usize| payload.get(at).copied().unwrap_or(0);
    let seq_number = u16::from_le_bytes([byte(0), byte(1)]);
    let offset = u32::from_le_bytes([byte(8), byte(9), byte(10), byte(11)]);
    let mut data = [0u8; 239];
    for (index, into) in data.iter_mut().enumerate() {
        *into = byte(12 + index);
    }
    format!(
        "{{\"seq_number\":{seq_number},\"session\":{},\"opcode\":{},\"size\":{},\"req_opcode\":{},\
         \"burst_complete\":{},\"padding\":{},\"offset\":{offset},\"data\":\"{}\"}}",
        byte(2),
        byte(3),
        byte(4),
        byte(5),
        byte(6),
        byte(7),
        base64::engine::general_purpose::STANDARD.encode(data),
    )
}

/// Convert to CSV's file: each read with text, [`debug_text`] with commas, after `lastlogread` as
/// `yyyy-MM-ddTHH:mm:ss.fff` and a comma; a `FILE_TRANSFER_PROTOCOL` with its payload's header
/// after the text trimmed, `", "` and `Environment.NewLine` ([`NEWLINE`]). A read with no text
/// writes nothing.
/// `// C#: Log/MavlinkLog.cs:1029-1108`
#[must_use]
pub fn csv_file(data: &[u8], zone: Zone<'_>, progress: &mut dyn FnMut(u8)) -> String {
    let mut reader = LogReader::new(data);
    let mut out = String::new();
    while let Some(packet) = reader.next_with_progress(progress) {
        let Some(read) = packet.read.as_ref() else {
            continue;
        };
        let Some(mut text) = debug_text(read, ",") else {
            continue;
        };
        if let Some(MavMessage::FileTransferProtocol(ftp)) = &read.message {
            text = format!(
                "{}, {}{NEWLINE}",
                text.trim_end(),
                ftp_header_json(&ftp.payload)
            );
        }
        out.push_str(&iso_millis(local_ticks(packet.time, zone)));
        out.push(',');
        out.push_str(&text);
    }
    out
}

// ---- Graph Log ----------------------------------------------------------------------------------

/// What Graph Log reads out of a log: `options`, every field seen as `mavlink_attitude_t.roll` - an
/// array's elements one each, `mavlink_statustext_t.text[0]` - in the order first seen, and the
/// form's own four after them; `datappl`, each one's points keyed `roll mavlink_attitude_t`; and
/// each field's units, as `MAVLink.GetUnit` gives them.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GraphLog {
    /// `options`, before the form's sort.
    pub options: Vec<String>,
    /// `datappl`: `(XDate, value)` points.
    pub series: HashMap<String, Vec<(f64, f64)>>,
    /// `MAVLink.GetUnit(field, name: MESSAGE)`: `[units]` from the field's `Units` attribute, or
    /// `""`, keyed `MESSAGE.field` (an array's field without its index).
    pub units: HashMap<String, String>,
}

impl GraphLog {
    /// `MAVLink.GetUnit(field, name: message)`: `""` for a field or message the log did not hold,
    /// as the C#'s lookup of a name it does not have throws and is caught.
    /// `// C#: ExtLibs/Mavlink/MavlinkParse.cs:8-36`
    #[must_use]
    pub fn unit(&self, message: &str, field: &str) -> &str {
        self.units
            .get(&format!("{message}.{field}"))
            .map_or("", String::as_str)
    }
}

/// `getHeartBeat`'s reads at the start of a log: up to and including the first heartbeat (or
/// `HIGH_LATENCY2`) from anything but a ground station, two hundred and one at most.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1155-1206`
fn get_heart_beat(reader: &mut LogReader<'_>, progress: &mut dyn FnMut(u8)) {
    let mut reads = 0usize;
    while let Some(packet) = reader.next_with_progress(progress) {
        reads += 1;
        if let Some(read) = packet.read.as_ref()
            && read.length > 5
        {
            let vehicle = match &read.message {
                Some(MavMessage::Heartbeat(heartbeat)) => heartbeat.r#type != MAV_TYPE_GCS,
                Some(MavMessage::HighLatency2(latency)) => latency.r#type != MAV_TYPE_GCS,
                _ => false,
            };
            if vehicle {
                return;
            }
        }
        if reads > HEARTBEAT_READS {
            return;
        }
    }
}

/// A field's value as a point's `y`: every numeric type graphs.
/// `// C#: Log/MavlinkLog.cs:337-394`
#[allow(clippy::cast_precision_loss)] // `(double)(ulong)value`, as the C# converts
fn values_of(value: &FieldValue) -> Vec<f64> {
    match value {
        FieldValue::Unsigned(value) => vec![*value as f64],
        FieldValue::Signed(value) => vec![*value as f64],
        FieldValue::Float(value) => vec![*value],
        FieldValue::UnsignedArray(values) => values.iter().map(|value| *value as f64).collect(),
        FieldValue::SignedArray(values) => values.iter().map(|value| *value as f64).collect(),
        FieldValue::FloatArray(values) => values.clone(),
    }
}

/// `PointLatLngAlt.GetDistance`: a haversine on a 6371 km sphere, the C#'s expression in its
/// order, in metres.
/// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:382-393`
#[must_use]
pub fn get_distance(lat1: f64, lng1: f64, lat2: f64, lng2: f64) -> f64 {
    let d = lat1 * 0.017_453_292_519_943_295;
    let num2 = lng1 * 0.017_453_292_519_943_295;
    let num3 = lat2 * 0.017_453_292_519_943_295;
    let num4 = lng2 * 0.017_453_292_519_943_295;
    let num5 = num4 - num2;
    let num6 = num3 - d;
    let num7 = (num6 / 2.0).sin().powi(2) + ((d.cos() * num3.cos()) * (num5 / 2.0).sin().powi(2));
    let num8 = 2.0 * num7.sqrt().atan2((1.0 - num7).sqrt());
    (6371.0 * num8) * 1000.0
}

/// `GetLogFileValidFields`: the log read after `getHeartBeat`'s reads - every packet this build
/// decodes but a ground station's heartbeat, each field's value at `lastlogread` - and then the
/// form's four of its own, in one `try` each as the C# has them:
///
/// - `custom`, `dospecial("GPS_RAW_INT")`'s compiled `getAltAboveHome`: each `GPS_RAW_INT`'s
///   `alt / 1000.0f` less `customforusenumber`, a `double` field that starts at 0 and is set only
///   when it is -1, so never - the GPS altitude in metres; one point a time, the last at it, in
///   the order the times came;
/// - `mag_field`, the magnitude of `RAW_IMU`'s magnetometer at each of its points; then
///   `dist_home`, each `GPS_RAW_INT` position's distance from the first with a 3D fix, or from
///   (0, 0) with none; then `sitltime`, the seconds between `RAW_IMU`'s `time_usec`s - the three in
///   one `try`, so the first whose data the log lacks ends the ones after it, its option added.
///
/// `progress` is told the percentage before each read.
/// `// C#: Log/MavlinkLog.cs:396-601, 608-829`
#[must_use]
pub fn graph_log(data: &[u8], zone: Zone<'_>, progress: &mut dyn FnMut(u8)) -> GraphLog {
    let mut log = GraphLog::default();
    let mut seen: HashSet<String> = HashSet::new();
    // `packetdata["GPS_RAW_INT"]`: a `Dictionary<DateTime, object>`, a later packet at a time
    // already held replacing the earlier where it stands.
    let mut gps: Vec<(i64, f32)> = Vec::new();
    let mut gps_at: HashMap<i64, usize> = HashMap::new();
    let mut reader = LogReader::new(data);
    get_heart_beat(&mut reader, progress);
    while let Some(packet) = reader.next_with_progress(progress) {
        let Some(read) = packet.read.as_ref() else {
            continue;
        };
        // `DebugPacket(packet, false)` returning null: "No info on packet".
        let Some(message) = read.message.as_ref().filter(|_| read.length > 5) else {
            continue;
        };
        if let MavMessage::Heartbeat(heartbeat) = message
            && heartbeat.r#type == MAV_TYPE_GCS
        {
            continue;
        }
        let local = local_ticks(packet.time, zone);
        let x = xdate(local);
        let name = struct_name(message.name());
        if let MavMessage::GpsRawInt(raw) = message {
            #[allow(clippy::cast_precision_loss)] // `gps.alt / 1000.0f`: an int over a float
            let alt = raw.alt as f32 / 1000.0;
            match gps_at.get(&local) {
                Some(&at) => {
                    if let Some(slot) = gps.get_mut(at) {
                        slot.1 = alt;
                    }
                }
                None => {
                    gps_at.insert(local, gps.len());
                    gps.push((local, alt));
                }
            }
        }
        for (info, (field, value)) in message.field_info().iter().zip(message.fields()) {
            log.units
                .entry(format!("{}.{field}", message.name()))
                .or_insert_with(|| {
                    if info.units.is_empty() {
                        String::new()
                    } else {
                        format!("[{}]", info.units)
                    }
                });
            let values = values_of(&value);
            if info.array_len > 0 {
                for (index, value) in values.into_iter().enumerate() {
                    let indexed = format!("{field}[{index}]");
                    let option = format!("{name}.{indexed}");
                    if seen.insert(option.clone()) {
                        log.options.push(option);
                    }
                    log.series
                        .entry(format!("{indexed} {name}"))
                        .or_default()
                        .push((x, value));
                }
            } else {
                let option = format!("{name}.{field}");
                if seen.insert(option.clone()) {
                    log.options.push(option);
                }
                if let Some(value) = values.first() {
                    log.series
                        .entry(format!("{field} {name}"))
                        .or_default()
                        .push((x, *value));
                }
            }
        }
    }
    // dospecial("GPS_RAW_INT")
    log.options.push("mavlink_custom_t.custom".to_owned());
    log.series.insert(
        "custom mavlink_custom_t".to_owned(),
        gps.iter()
            .map(|(time, alt)| (xdate(*time), f64::from(*alt) - 0.0))
            .collect(),
    );
    add_custom(&mut log);
    progress(100);
    log
}

/// `addMagField`, `addDistHome` and `addIMUTime`, in their one `try`.
/// `// C#: Log/MavlinkLog.cs:566-575, 736-829`
fn add_custom(log: &mut GraphLog) {
    let get = |log: &GraphLog, key: &str| log.series.get(key).cloned();
    // addMagField
    log.options.push("mavlink_custom_t.mag_field".to_owned());
    log.series
        .insert("mag_field mavlink_custom_t".to_owned(), Vec::new());
    let (Some(xs), ys, zs) = (
        get(log, "xmag mavlink_raw_imu_t"),
        get(log, "ymag mavlink_raw_imu_t"),
        get(log, "zmag mavlink_raw_imu_t"),
    ) else {
        return;
    };
    let mut magnitudes = Vec::new();
    let mut thrown = false;
    for (a, (x, mx)) in xs.iter().enumerate() {
        let (Some((_, my)), Some((_, mz))) = (
            ys.as_ref().and_then(|ys| ys.get(a)),
            zs.as_ref().and_then(|zs| zs.get(a)),
        ) else {
            thrown = true;
            break;
        };
        magnitudes.push((*x, (mx.powi(2) + my.powi(2) + mz.powi(2)).sqrt()));
    }
    log.series
        .insert("mag_field mavlink_custom_t".to_owned(), magnitudes);
    if thrown {
        return;
    }
    // addDistHome
    log.options.push("mavlink_custom_t.dist_home".to_owned());
    log.series
        .insert("dist_home mavlink_custom_t".to_owned(), Vec::new());
    let Some(fixes) = get(log, "fix_type mavlink_gps_raw_int_t") else {
        return;
    };
    let lats = get(log, "lat mavlink_gps_raw_int_t");
    let lons = get(log, "lon mavlink_gps_raw_int_t");
    let mut home = (0.0, 0.0);
    for (a, (_, fix)) in fixes.iter().enumerate() {
        if *fix == 3.0 {
            let (Some((_, lat)), Some((_, lon))) = (
                lats.as_ref().and_then(|lats| lats.get(a)),
                lons.as_ref().and_then(|lons| lons.get(a)),
            ) else {
                return;
            };
            home = (lat / 10_000_000.0, lon / 10_000_000.0);
            break;
        }
    }
    let Some(lats) = lats else {
        return;
    };
    let mut distances = Vec::new();
    let mut thrown = false;
    for (a, (x, lat)) in lats.iter().enumerate() {
        let Some((_, lon)) = lons.as_ref().and_then(|lons| lons.get(a)) else {
            thrown = true;
            break;
        };
        distances.push((
            *x,
            get_distance(home.0, home.1, lat / 10_000_000.0, lon / 10_000_000.0),
        ));
    }
    log.series
        .insert("dist_home mavlink_custom_t".to_owned(), distances);
    if thrown {
        return;
    }
    // addIMUTime
    log.options.push("mavlink_custom_t.sitltime".to_owned());
    log.series
        .insert("sitltime mavlink_custom_t".to_owned(), Vec::new());
    let Some(times) = get(log, "time_usec mavlink_raw_imu_t") else {
        return;
    };
    let Some((_, first)) = times.first() else {
        return;
    };
    let mut last = first * 1.0e-6;
    let mut deltas = Vec::new();
    for (x, usec) in &times {
        deltas.push((*x, (usec * 1.0e-6) - last));
        last = usec * 1.0e-6;
    }
    log.series
        .insert("sitltime mavlink_custom_t".to_owned(), deltas);
}

// ---- Create KML + GPX ---------------------------------------------------------------------------

/// What `writeGPX` and `writeKML` read of a `CurrentState` clone, taken at each
/// `GLOBAL_POSITION_INT`.
#[derive(Debug, Clone, PartialEq)]
pub struct Sample {
    /// `cs.datetime`: the time of the packet that took it, UTC; [`DateTime::MIN`] with no stamp.
    pub datetime: DateTime,
    /// `cs.lat`.
    pub lat: f64,
    /// `cs.lng`.
    pub lng: f64,
    /// `cs.altasl`, metres.
    pub altasl: f32,
    /// `cs.alt`, metres above home.
    pub alt: f32,
    /// `cs.roll`, degrees.
    pub roll: f32,
    /// `cs.pitch`, degrees.
    pub pitch: f32,
    /// `cs.yaw`, degrees.
    pub yaw: f32,
    /// `cs.mode`: "Unknown" until a heartbeat names one.
    pub mode: String,
}

/// What Create KML + GPX has once the log is read: `flightdataDictionary`, each system's samples
/// in the order its first position came, and `mine.MAV.wps[0].z`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FlightData {
    /// Each system id and its samples - every component of the system in the one list, as the
    /// dictionary is keyed by system id alone.
    pub vehicles: Vec<(u32, Vec<Sample>)>,
    /// The altitude of mission item 0 held for `MAV` - the first vehicle a heartbeat came from,
    /// whose state `MAVlist.Count == 1` makes current - or `None` where it holds none.
    pub home_z: Option<f32>,
}

/// `processInfoFromStream`'s missions: `MISSION_COUNT` of a mission clearing it, `MISSION_ITEM`
/// and `MISSION_INT` of one (not a guided item, `current` 2) setting its item - each for the
/// target system and component, or the sender's when the target is a ground station or 0.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5605-5672`
fn missions(wps: &mut HashMap<(u32, u8), BTreeMap<u16, f32>>, read: &Read, message: &MavMessage) {
    let target = |system: u8, component: u8| {
        let system = u32::from(system);
        if system == GCS_SYSIDS[0] || system == 0 {
            (read.sysid, read.compid)
        } else {
            (system, component)
        }
    };
    match message {
        MavMessage::MissionCount(count) if count.mission_type == MISSION => {
            wps.entry(target(count.target_system, count.target_component))
                .or_default()
                .clear();
        }
        MavMessage::MissionItem(item) if item.current != 2 && item.mission_type == MISSION => {
            wps.entry(target(item.target_system, item.target_component))
                .or_default()
                .insert(item.seq, item.z);
        }
        MavMessage::MissionItemInt(item) if item.current != 2 && item.mission_type == MISSION => {
            wps.entry(target(item.target_system, item.target_component))
                .or_default()
                .insert(item.seq, item.z);
        }
        _ => {}
    }
}

/// Create KML + GPX's replay: each packet after the ground station's (which log playback hands on
/// before `CurrentState` hears them) applied to its vehicle's state at `lastlogread`, the state
/// cloned at each `GLOBAL_POSITION_INT` for the sender's system; the mode named as `CurrentState`
/// names it - a heartbeat with `CUSTOM_MODE_ENABLED` and a mode number not the last, looked up in
/// the vehicle's mode list, the last name kept where it has none; and the missions as
/// `processInfoFromStream` keeps them, ground station's packets included. `progress` is told the
/// percentage before each read.
/// `// C#: Log/MavlinkLog.cs:80-196; ExtLibs/ArduPilot/CurrentState.cs:2897-2923, 3245-3283,
/// 4388-4393; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5097-5108, 5290-5345`
#[must_use]
pub fn flight_data(data: &[u8], progress: &mut dyn FnMut(u8)) -> FlightData {
    let mut reader = LogReader::new(data);
    let mut registry = VehicleRegistry::new();
    // `mode` and `_mode`, per state.
    let mut modes: HashMap<VehicleId, (String, u32)> = HashMap::new();
    let mut current: Option<VehicleId> = None;
    let mut wps: HashMap<(u32, u8), BTreeMap<u16, f32>> = HashMap::new();
    let mut vehicles: Vec<(u32, Vec<Sample>)> = Vec::new();
    let rad2deg = 180.0 / std::f64::consts::PI;
    while let Some(packet) = reader.next_with_progress(progress) {
        let Some(read) = packet.read.as_ref() else {
            continue;
        };
        let Some(message) = read.message.as_ref() else {
            continue;
        };
        missions(&mut wps, read, message);
        if GCS_SYSIDS.contains(&read.sysid) {
            continue;
        }
        let id = registry.apply_at(read.sysid, read.compid, read.seq, message, packet.time);
        match message {
            MavMessage::Heartbeat(heartbeat) => {
                current.get_or_insert(id);
                let vehicle_type = registry.working(id).map_or(0, |state| state.vehicle_type);
                let (mode, number) = modes
                    .entry(id)
                    .or_insert_with(|| ("Unknown".to_owned(), 99_999));
                if heartbeat.base_mode & CUSTOM_MODE_ENABLED != 0
                    && *number != heartbeat.custom_mode
                {
                    if let Some(name) =
                        mp_vehicle::flight_mode_name(vehicle_type, heartbeat.custom_mode)
                    {
                        name.clone_into(mode);
                    }
                    *number = heartbeat.custom_mode;
                }
            }
            MavMessage::HighLatency2(_) => {
                current.get_or_insert(id);
            }
            MavMessage::GlobalPositionInt(_) => {
                let Some(state) = registry.working(id) else {
                    continue;
                };
                let (lat, lng) = state
                    .position
                    .map_or((0.0, 0.0), |at| (at.latitude(), at.longitude()));
                // `(float)(att.roll * MathHelper.rad2deg)` and the other float properties.
                #[allow(clippy::cast_possible_truncation)]
                let sample = Sample {
                    datetime: state.datetime,
                    lat,
                    lng,
                    altasl: state.altitude_msl.0 as f32,
                    alt: state.alt(),
                    roll: (state.attitude.roll.0 * rad2deg) as f32,
                    pitch: (state.attitude.pitch.0 * rad2deg) as f32,
                    yaw: (state.attitude.yaw.0 * rad2deg) as f32,
                    mode: modes
                        .get(&id)
                        .map_or_else(|| "Unknown".to_owned(), |(mode, _)| mode.clone()),
                };
                match vehicles.iter_mut().find(|(sysid, _)| *sysid == read.sysid) {
                    Some((_, samples)) => samples.push(sample),
                    None => vehicles.push((read.sysid, vec![sample])),
                }
            }
            _ => {}
        }
    }
    let home_z = current
        .and_then(|id| wps.get(&(id.sysid, id.compid)))
        .and_then(|items| items.get(&0))
        .copied();
    FlightData { vehicles, home_z }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::indexing_slicing,
        clippy::cast_possible_truncation
    )]

    use super::*;
    use mp_mavlink::{Message as _, encode_v1, encode_v2};
    use mp_mavlink_dialects::all::{
        Attitude, FileTransferProtocol, GlobalPositionInt, GpsRawInt, Heartbeat, MissionItemInt,
        RawImu, Statustext,
    };

    fn testdata(name: &str) -> std::path::PathBuf {
        std::path::Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../testdata/mavlink"
        ))
        .join(name)
    }

    /// A message as a MAVLink 2 frame from `sysid`:1.
    fn frame<M: mp_mavlink::Message>(message: &M, seq: u8, sysid: u32) -> Vec<u8> {
        let mut payload = vec![0u8; M::LEN];
        message.encode(&mut payload);
        let mut out = [0u8; 300];
        let n = encode_v2(&mut out, seq, sysid, 1, M::ID, &payload, M::CRC_EXTRA, 0).unwrap();
        out[..n].to_vec()
    }

    /// 2026-10-16 00:00:00 UTC, in Unix microseconds.
    const DAY: u64 = 1_792_108_800_000_000;

    /// A tlog of `(seconds after DAY, frame)` records.
    fn tlog(records: &[(f64, Vec<u8>)]) -> Vec<u8> {
        let mut out = Vec::new();
        for (seconds, frame) in records {
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let micros = DAY + (seconds * 1_000_000.0) as u64;
            out.extend(micros.to_be_bytes());
            out.extend(frame);
        }
        out
    }

    fn heartbeat(custom_mode: u32, r#type: u8) -> Heartbeat {
        Heartbeat {
            custom_mode,
            r#type,
            autopilot: 3,
            base_mode: 89,
            system_status: 4,
            mavlink_version: 3,
        }
    }

    /// The times as the C# writes them: `DateTime.ToString()` in English, the CSV's form, and
    /// `MinValue` as it is.
    #[test]
    fn times_are_written_as_dotnet_writes_them() {
        let time = DateTime::from_tlog_micros(DAY + 13 * 3_600_000_000 + 5_123_456).unwrap();
        assert_eq!(
            general_date(local_ticks(time, &utc)),
            "10/16/2026 1:00:05 PM"
        );
        assert_eq!(
            iso_millis(local_ticks(time, &utc)),
            "2026-10-16T13:00:05.123"
        );
        // Ten hours east: `ToLocalTime`.
        assert_eq!(
            general_date(local_ticks(time, &|_| 36_000)),
            "10/16/2026 11:00:05 PM"
        );
        assert_eq!(
            general_date(local_ticks(DateTime::MIN, &|_| 36_000)),
            "1/1/0001 12:00:00 AM"
        );
        assert_eq!(iso_millis(0), "0001-01-01T00:00:00.000");
        let midnight = DateTime::from_tlog_micros(DAY).unwrap();
        assert_eq!(
            general_date(local_ticks(midnight, &utc)),
            "10/16/2026 12:00:00 AM"
        );
        // XDate: days since 1899-12-30; 2026-10-16 is day 46311.
        assert_eq!(xdate(local_ticks(midnight, &utc)), 46_311.0);
        assert_eq!(xdate(0), -693_593.0);
    }

    /// The first frame of the autotest flight as `DebugPacket` writes it, every number from Mission
    /// Planner's own decode of it (`autotest.tlog.csharp.csv`: msgid 0, seq 0, sysid 1, compid 1,
    /// 9 bytes of payload, crc16 49794, 21 bytes; `autotest.fields.csharp.csv`: the six fields in
    /// the struct's order and their values).
    #[test]
    fn the_autotest_flights_first_heartbeat_is_written_as_the_csharp_writes_it() {
        let data = std::fs::read(testdata("autotest.tlog")).unwrap();
        let first = packets(&data)
            .into_iter()
            .find_map(|packet| packet.read)
            .unwrap();
        assert_eq!(
            debug_text(&first, " ").unwrap(),
            "FD  9  0  0  0  1  1      0 mavlink_heartbeat_t custom_mode 0 type 2 autopilot 3 \
             base_mode 89 system_status 1 mavlink_version 3  sig  Len 21 crc16 49794\r\n"
        );
        assert_eq!(
            debug_text(&first, ",").unwrap(),
            "FD, 9, 0, 0, 0, 1, 1,     0,mavlink_heartbeat_t,custom_mode,0,type,2,autopilot,3,\
             base_mode,89,system_status,1,mavlink_version,3,,sig ,Len,21,crc16,49794\r\n"
        );
    }

    /// Every packet the reader hands on in the autotest flight is one Mission Planner's own
    /// parser decoded, in its order, its fields in the C# struct's order with the values
    /// `DebugPacket` writes: integers in full, floats as `ToString()` writes the same single, text
    /// arrays to their NUL, other arrays nothing. The C#'s corpora: `autotest.tlog.csharp.csv`
    /// (every frame) and `autotest.fields.csharp.csv` (every field of the first 3,000).
    #[test]
    fn every_packet_and_field_is_the_csharps() {
        let data = std::fs::read(testdata("autotest.tlog")).unwrap();
        let frames = std::fs::read_to_string(testdata("autotest.tlog.csharp.csv")).unwrap();
        let theirs: Vec<(String, String)> = frames
            .lines()
            .skip(1)
            .map(|line| {
                let parts: Vec<&str> = line.split(',').collect();
                (parts[0].to_owned(), parts[7].to_owned())
            })
            .collect();
        let fields = std::fs::read_to_string(testdata("autotest.fields.csharp.csv")).unwrap();
        let mut by_index: BTreeMap<String, Vec<(String, String)>> = BTreeMap::new();
        for line in fields.lines().skip(1) {
            let parts: Vec<&str> = line.splitn(5, ',').collect();
            by_index
                .entry(parts[0].to_owned())
                .or_default()
                .push((parts[3].to_owned(), parts[4].to_owned()));
        }
        let hex = |bytes: &[u8]| bytes.iter().map(|b| format!("{b:02x}")).collect::<String>();
        let mut at = 0usize;
        let mut matched = 0usize;
        let mut compared = 0usize;
        for packet in packets(&data) {
            let Some(read) = packet.read else { continue };
            let Some(message) = read.message.as_ref() else {
                continue;
            };
            let ours = hex(&read.raw);
            let found = theirs[at..]
                .iter()
                .position(|(_, frame)| *frame == ours)
                .unwrap_or_else(|| panic!("frame {ours} not in the C#'s decode after {at}"));
            at += found + 1;
            matched += 1;
            let index = &theirs[at - 1].0;
            let Some(expected) = by_index.get(index) else {
                continue;
            };
            // The fields in the C# struct's order: ours among theirs, in the same order.
            let names: Vec<&str> = message.fields().iter().map(|(name, _)| *name).collect();
            let their_names: Vec<&str> = expected
                .iter()
                .map(|(name, _)| name.as_str())
                .filter(|name| names.contains(name))
                .collect();
            assert_eq!(names, their_names, "field order of frame {index}");
            for (info, (name, value)) in message.field_info().iter().zip(message.fields()) {
                let theirs = &expected.iter().find(|(field, _)| field == name).unwrap().1;
                let text = if info.array_len > 0 {
                    array_text(info, &value)
                } else {
                    scalar_text(info, &value)
                };
                let wanted = if info.array_len > 0 {
                    match info.base_type {
                        "char" | "uint8_t" => theirs
                            .split(' ')
                            .map(|byte| byte.parse::<u8>().unwrap())
                            .take_while(|byte| *byte != 0)
                            .map(char::from)
                            .collect(),
                        "int16_t" => theirs
                            .split(' ')
                            .map(|v| v.parse::<i16>().unwrap())
                            .take_while(|v| *v != 0)
                            .map(|v| format!("{:x}|", v as u16))
                            .collect(),
                        _ => String::new(),
                    }
                } else if info.base_type == "float" {
                    netfmt::single(theirs.parse::<f32>().unwrap())
                } else if info.base_type == "double" {
                    netfmt::double(theirs.parse::<f64>().unwrap())
                } else {
                    theirs.clone()
                };
                assert_eq!(text, wanted, "{name} of frame {index}");
                compared += 1;
            }
        }
        assert!(
            matched + 5 >= theirs.len(),
            "handed on {matched} of the C#'s {}",
            theirs.len()
        );
        assert!(compared > 20_000, "compared {compared}");
    }

    /// Convert to Text: each read after its time and a space; an `Invalid` one - a frame whose
    /// checksum fails with no MAVLink 2 start byte in its last twenty bytes - the time and the space
    /// alone, the next read's time following on the same line, as the C# writes it.
    #[test]
    fn convert_to_text_writes_each_read_after_its_time() {
        // A heartbeat whose checksum is spoiled, with no MAVLink 2 start byte after its own: the
        // first mode number that gives one.
        let bad = (0..256)
            .map(|mode| {
                let mut bad = frame(&heartbeat(mode, 2), 1, 1);
                let last = bad.len() - 1;
                bad[last] ^= 0xFF;
                bad
            })
            .find(|bad| !bad[1..].contains(&STX_V2))
            .unwrap();
        let data = tlog(&[
            (0.0, frame(&heartbeat(0, 2), 0, 1)),
            (1.0, bad),
            (
                2.5,
                frame(
                    &Statustext {
                        severity: 6,
                        text: {
                            let mut text = [0u8; 50];
                            text[..5].copy_from_slice(b"Hello");
                            text
                        },
                        id: 0,
                        chunk_seq: 0,
                    },
                    2,
                    1,
                ),
            ),
        ]);
        let mut seen = Vec::new();
        let text = text_file(&data, &utc, &mut |percent| seen.push(percent));
        let lines: Vec<&str> = text.split("\r\n").collect();
        assert_eq!(lines.len(), 3, "{text:?}");
        assert!(lines[0].starts_with("10/16/2026 12:00:00 AM FD  9  0  0  0  1  1      0 "));
        // The bad heartbeat's line is its time alone; the status text follows it on that line.
        assert!(
            lines[1].starts_with(
                "10/16/2026 12:00:01 AM 10/16/2026 12:00:02 AM FD  6  0  0  2  1  1     FD \
                 mavlink_statustext_t severity 6 text Hello id 0 chunk_seq 0  sig  Len 18 crc16 "
            ),
            "{:?}",
            lines[1]
        );
        assert_eq!(lines[2], "");
        assert_eq!(seen.first(), Some(&0));
    }

    /// A failing frame with a MAVLink 2 start byte in its last twenty bytes is passed over within
    /// the read, its time kept and the next record's taken: no line of its own.
    #[test]
    fn a_failing_frame_with_a_start_byte_near_its_end_is_read_past() {
        let mut bad = frame(
            &Attitude {
                time_boot_ms: 1,
                roll: 1.0,
                pitch: 1.0,
                yaw: 1.0,
                rollspeed: 1.0,
                pitchspeed: 1.0,
                yawspeed: 1.0,
            },
            1,
            1,
        );
        // Not zeros, so nothing is trimmed: 28 bytes of payload, 40 of frame. A 0xFD put among
        // its last twenty spoils the checksum.
        let n = bad.len();
        assert_eq!(n, 40);
        bad[n - 6] = STX_V2;
        let data = tlog(&[(0.0, bad), (3.0, frame(&heartbeat(0, 2), 2, 1))]);
        let read = packets(&data);
        assert_eq!(read.len(), 1, "{read:?}");
        assert_eq!(read[0].read.as_ref().unwrap().msgid, 0);
        assert_eq!(
            general_date(local_ticks(read[0].time, &utc)),
            "10/16/2026 12:00:03 AM"
        );
    }

    /// A record whose eight bytes start a frame has no stamp: the clock stays where it was.
    #[test]
    fn a_frame_with_no_stamp_keeps_the_last_time() {
        let mut data = tlog(&[(4.0, frame(&heartbeat(0, 2), 0, 1))]);
        data.extend(frame(&heartbeat(0, 2), 1, 1));
        let read = packets(&data);
        assert_eq!(read.len(), 2);
        assert_eq!(read[1].read.as_ref().unwrap().seq, 1);
        assert_eq!(read[0].time, read[1].time);
    }

    /// A MAVLink 1 frame whose length is not its message's is `Invalid` before its checksum is
    /// looked at.
    #[test]
    fn a_mavlink1_frame_of_the_wrong_length_is_invalid() {
        let mut payload = [0u8; Heartbeat::LEN];
        heartbeat(0, 2).encode(&mut payload);
        let mut out = [0u8; 64];
        // Eight of its nine bytes, checksummed: the checksum passes, the length does not.
        let n = encode_v1(
            &mut out,
            0,
            1,
            1,
            Heartbeat::ID,
            &payload[..8],
            Heartbeat::CRC_EXTRA,
        )
        .unwrap();
        let data = tlog(&[(0.0, out[..n].to_vec())]);
        let read = packets(&data);
        assert_eq!(read.len(), 1);
        assert!(read[0].read.is_none());
    }

    /// Convert to CSV: the time to the millisecond, commas, nothing for a read with no text, and a
    /// file transfer's header as Json.NET writes the struct.
    #[test]
    fn convert_to_csv_writes_commas_and_the_ftp_header() {
        let mut payload = [0u8; 251];
        payload[0] = 7; // seq_number
        payload[3] = 4; // opcode
        payload[4] = 3; // size
        payload[8] = 0x10; // offset
        payload[12..15].copy_from_slice(b"abc");
        let ftp = FileTransferProtocol {
            target_network: 0,
            target_system: 1,
            target_component: 1,
            payload,
        };
        let data = tlog(&[(0.25, frame(&ftp, 0, 255))]);
        let csv = csv_file(&data, &utc, &mut |_| {});
        assert!(csv.starts_with("2026-10-16T00:00:00.250,FD,"), "{csv:?}");
        let mut data_bytes = [0u8; 239];
        data_bytes[..3].copy_from_slice(b"abc");
        let header = format!(
            "{{\"seq_number\":7,\"session\":0,\"opcode\":4,\"size\":3,\"req_opcode\":0,\
             \"burst_complete\":0,\"padding\":0,\"offset\":16,\"data\":\"{}\"}}",
            base64::engine::general_purpose::STANDARD.encode(data_bytes)
        );
        // `text.TrimEnd() + ", " + header + Environment.NewLine`: the text's `\r\n` gone.
        let (text, rest) = csv.split_once(", {").unwrap();
        assert_eq!(format!("{{{rest}"), format!("{header}{NEWLINE}"));
        let crc = text.rsplit_once(",crc16,").unwrap().1;
        assert!(crc.parse::<u16>().is_ok(), "{text:?}");
        assert_eq!(csv.matches(NEWLINE).count(), 1);
    }

    /// Graph Log: `getHeartBeat`'s reads left out, every field of the rest an option in the order
    /// first seen - an array's elements each - with its points at `lastlogread`, a ground
    /// station's heartbeat left out, and the four the form adds: `custom` the GPS altitude,
    /// `mag_field`, `dist_home` from the first 3D fix, `sitltime` the IMU clock's steps.
    #[test]
    fn graph_log_reads_every_field_and_adds_the_forms_own() {
        let gps = |lat: i32, fix: u8, alt: i32| GpsRawInt {
            time_usec: 0,
            lat,
            lon: 1_530_000_000,
            alt,
            eph: 0,
            epv: 0,
            vel: 0,
            cog: 0,
            fix_type: fix,
            satellites_visible: 0,
            alt_ellipsoid: 0,
            h_acc: 0,
            v_acc: 0,
            vel_acc: 0,
            hdg_acc: 0,
            yaw: 0,
        };
        let imu = |usec: u64, mag: i16| RawImu {
            time_usec: usec,
            xacc: 0,
            yacc: 0,
            zacc: 0,
            xgyro: 0,
            ygyro: 0,
            zgyro: 0,
            xmag: mag,
            ymag: 0,
            zmag: 4,
            id: 0,
            temperature: 0,
        };
        let data = tlog(&[
            // A ground station's heartbeat does not end getHeartBeat; the vehicle's does.
            (0.0, frame(&heartbeat(0, MAV_TYPE_GCS), 0, 255)),
            (0.0, frame(&gps(-270_000_000, 1, 99_000), 1, 1)),
            (0.0, frame(&heartbeat(0, 2), 2, 1)),
            (1.0, frame(&heartbeat(0, MAV_TYPE_GCS), 1, 255)),
            (1.0, frame(&gps(-270_000_000, 3, 25_000), 3, 1)),
            (2.0, frame(&gps(-270_010_000, 3, 26_500), 4, 1)),
            (2.0, frame(&imu(1_000_000, 3), 5, 1)),
            (3.0, frame(&imu(1_250_000, 0), 6, 1)),
        ]);
        let log = graph_log(&data, &utc, &mut |_| {});
        // The GPS before the vehicle's heartbeat is getHeartBeat's, and not graphed.
        let fix = &log.series["fix_type mavlink_gps_raw_int_t"];
        assert_eq!(fix.iter().map(|(_, y)| *y).collect::<Vec<_>>(), [3.0, 3.0]);
        let one = xdate(local_ticks(
            DateTime::from_tlog_micros(DAY + 1_000_000).unwrap(),
            &utc,
        ));
        assert_eq!(fix[0].0, one);
        assert!((one - (46_311.0 + 1.0 / 86_400.0)).abs() < 1e-9);
        assert!(
            !log.options
                .iter()
                .any(|o| o.starts_with("mavlink_heartbeat_t"))
        );
        assert_eq!(log.options[0], "mavlink_gps_raw_int_t.time_usec");
        assert!(log.options.contains(&"mavlink_raw_imu_t.xmag".to_owned()));
        assert_eq!(
            &log.options[log.options.len() - 4..],
            [
                "mavlink_custom_t.custom",
                "mavlink_custom_t.mag_field",
                "mavlink_custom_t.dist_home",
                "mavlink_custom_t.sitltime"
            ]
        );
        // custom: one point a time, the last GPS at it.
        let custom: Vec<f64> = log.series["custom mavlink_custom_t"]
            .iter()
            .map(|(_, y)| *y)
            .collect();
        assert_eq!(
            custom,
            [f64::from(25_000f32 / 1000.0), f64::from(26_500f32 / 1000.0)]
        );
        // mag_field: |(3, 0, 4)| = 5, then |(0, 0, 4)| = 4.
        let mag: Vec<f64> = log.series["mag_field mavlink_custom_t"]
            .iter()
            .map(|(_, y)| *y)
            .collect();
        assert_eq!(mag, [5.0, 4.0]);
        // dist_home: from the first 3D fix.
        let dist = &log.series["dist_home mavlink_custom_t"];
        assert_eq!(dist[0].1, 0.0);
        assert_eq!(dist[1].1, get_distance(-27.0, 153.0, -27.001, 153.0));
        assert!((dist[1].1 - 111.2).abs() < 0.5, "{}", dist[1].1);
        // sitltime: 0, then a quarter second.
        let steps: Vec<f64> = log.series["sitltime mavlink_custom_t"]
            .iter()
            .map(|(_, y)| *y)
            .collect();
        assert_eq!(steps[0], 0.0);
        assert!((steps[1] - 0.25).abs() < 1e-9);
        assert_eq!(log.unit("RAW_IMU", "xmag"), "[mgauss]");
        assert_eq!(log.unit("GPS_RAW_INT", "fix_type"), "");
        assert_eq!(log.unit("CUSTOM", "custom"), "");
    }

    /// With no `RAW_IMU`, `addMagField` throws on its first line after adding its option, and
    /// `addDistHome` and `addIMUTime` never run.
    #[test]
    fn no_raw_imu_ends_the_forms_own_after_mag_field() {
        let data = tlog(&[
            (0.0, frame(&heartbeat(0, 2), 0, 1)),
            (
                1.0,
                frame(
                    &Attitude {
                        time_boot_ms: 1,
                        roll: 0.5,
                        pitch: 0.0,
                        yaw: 0.0,
                        rollspeed: 0.0,
                        pitchspeed: 0.0,
                        yawspeed: 0.0,
                    },
                    1,
                    1,
                ),
            ),
        ]);
        let log = graph_log(&data, &utc, &mut |_| {});
        assert_eq!(
            &log.options[log.options.len() - 2..],
            ["mavlink_custom_t.custom", "mavlink_custom_t.mag_field"]
        );
        assert!(log.series["custom mavlink_custom_t"].is_empty());
        let one = xdate(local_ticks(
            DateTime::from_tlog_micros(DAY + 1_000_000).unwrap(),
            &utc,
        ));
        assert_eq!(log.series["roll mavlink_attitude_t"], [(one, 0.5)]);
        assert_eq!(log.unit("ATTITUDE", "roll"), "[rad]");
    }

    /// Create KML's replay: a sample of the sender's state at each position, the mode named from
    /// the heartbeat, "Unknown" before one; a ground station's packets not heard by any state;
    /// mission item 0 of the first vehicle a heartbeat came from.
    #[test]
    fn flight_data_samples_each_position_with_the_state_at_it() {
        let position = |lat: i32, relative: i32| GlobalPositionInt {
            time_boot_ms: 0,
            lat,
            lon: 1_530_000_000,
            alt: 584_000,
            relative_alt: relative,
            vx: 0,
            vy: 0,
            vz: 0,
            hdg: 0,
        };
        let item = MissionItemInt {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 0,
            y: 0,
            z: 584.25,
            seq: 0,
            command: 16,
            target_system: 255,
            target_component: 190,
            frame: 0,
            current: 0,
            autocontinue: 1,
            mission_type: 0,
        };
        let data = tlog(&[
            (0.0, frame(&position(-270_000_000, 0), 0, 1)),
            (1.0, frame(&heartbeat(5, 2), 1, 1)),
            (
                1.5,
                frame(
                    &Attitude {
                        time_boot_ms: 0,
                        roll: 0.1,
                        pitch: -0.2,
                        yaw: 1.0,
                        rollspeed: 0.0,
                        pitchspeed: 0.0,
                        yawspeed: 0.0,
                    },
                    2,
                    1,
                ),
            ),
            (2.0, frame(&position(-270_001_000, 10_500), 3, 1)),
            (2.0, frame(&position(-280_000_000, 0), 0, 255)),
            (3.0, frame(&item, 4, 1)),
        ]);
        let flight = flight_data(&data, &mut |_| {});
        assert_eq!(flight.vehicles.len(), 1);
        let (sysid, samples) = &flight.vehicles[0];
        assert_eq!(*sysid, 1);
        assert_eq!(samples.len(), 2);
        assert_eq!(samples[0].mode, "Unknown");
        assert_eq!(samples[1].mode, "Loiter");
        assert_eq!(samples[1].lat, -270_001_000.0 / 10_000_000.0);
        assert_eq!(samples[1].lng, 153.0);
        assert_eq!(samples[1].altasl, 584.0);
        assert_eq!(samples[1].alt, 10.5);
        assert_eq!(
            samples[1].roll,
            (f64::from(0.1f32) * (180.0 / std::f64::consts::PI)) as f32
        );
        assert_eq!(
            samples[1].pitch,
            (f64::from(-0.2f32) * (180.0 / std::f64::consts::PI)) as f32
        );
        assert_eq!(
            samples[1].datetime,
            DateTime::from_tlog_micros(DAY + 2_000_000).unwrap()
        );
        assert_eq!(flight.home_z, Some(584.25));
    }
}
