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

//! A tlog as `GeoRefImageBase` reads it: `CommsFile.GetMessageOfType(ids, hasTimestamp: true)`,
//! which is `MavlinkParse.ReadPacket` over the file until nothing is left to read.
//!
//! `mp_log::TlogReader` is a reader of its own design - it treats a timestamp as a hint and
//! resynchronises a byte at a time - so it is not used: which packets the C# sees in a damaged log,
//! and what time each carries, is `ReadPacket`'s business, ported here.
//! `// C#: ExtLibs/Utilities/Extensions.cs:746-760, ExtLibs/Mavlink/MavlinkParse.cs:132-246`

use mp_mavlink::crc;
use mp_mavlink::{Dialect, STX_V1, STX_V2};
use mp_mavlink_dialects::DIALECT;

use crate::time::{DateTime, Kind, UNIX_EPOCH_TICKS};

/// `MAVLINK_MAX_PACKET_LEN`: payload, header and checksum, and a signature.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:18`
const MAX_PACKET_LEN: usize = 255 + 12 + 13;
/// `MAVLINK_CORE_HEADER_LEN` and the MAVLink 1 one. `// C#: ExtLibs/Mavlink/Mavlink.cs:12-13`
const CORE_HEADER_LEN: usize = 9;
const CORE_HEADER_V1_LEN: usize = 5;
/// `MAVLINK_SIGNATURE_BLOCK_LEN`. `// C#: ExtLibs/Mavlink/Mavlink.cs:20`
const SIGNATURE_LEN: usize = 13;

/// One packet `ReadPacket` returned.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet {
    /// `rxtime`: the record's timestamp as local time, or `DateTime.MinValue`.
    pub rxtime: DateTime,
    /// `msgid`.
    pub msgid: u32,
    /// The payload as sent, which a v2 sender may have cut short.
    pub payload: Vec<u8>,
}

/// Why reading stopped early: `ReadWithTimeout` found the file ending inside the start-byte scan,
/// which `ReadPacket` does not catch (`MavlinkParse.cs:157-166`), so the exception leaves
/// `GetMessageOfType` and whatever was reading it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("End of data")]
pub struct EndOfData;

/// `GetMessageOfType(ids, true)`: every packet whose id is in `ids`, in file order, or the
/// exception that ended the read.
///
/// # Errors
///
/// [`EndOfData`] where the C# throws out of the enumeration; the packets before it are lost with
/// it, as they are when the exception unwinds the C#'s `foreach`.
/// `// C#: ExtLibs/Utilities/Extensions.cs:746-760`
pub fn messages_of_type(data: &[u8], ids: &[u32]) -> Result<Vec<Packet>, EndOfData> {
    let mut pos = 0usize;
    let mut out = Vec::new();
    // `while (commsFile.BytesToRead > 0)`
    while pos < data.len() {
        if let Some(packet) = read_packet(data, &mut pos)?
            && ids.contains(&packet.msgid)
        {
            out.push(packet);
        }
    }
    Ok(out)
}

/// `ReadPacket` with `hasTimestamp`: eight bytes of big-endian microseconds, then a scan of at
/// most 281 bytes for a start byte, the header, the rest, and the checksum; `None` for a packet
/// dropped (no start byte in reach, cut short by the end of the file, or a bad checksum).
/// `// C#: ExtLibs/Mavlink/MavlinkParse.cs:132-246`
fn read_packet(data: &[u8], pos: &mut usize) -> Result<Option<Packet>, EndOfData> {
    // `BaseStream.Read(datearray, 0, 8)`: as many as are left, the rest zero, then reversed.
    let mut stamp = [0u8; 8];
    for slot in &mut stamp {
        match data.get(*pos) {
            Some(&b) => {
                *slot = b;
                *pos += 1;
            }
            None => break,
        }
    }
    let micros = u64::from_be_bytes(stamp);
    let mut rxtime = DateTime::MIN;
    if micros / 1000 / 1000 / 60 / 60 < 9_999_999 {
        // `date1.AddMilliseconds(dateint / 1000)`, then `ToLocalTime()`: local is UTC here.
        #[allow(clippy::cast_precision_loss)]
        let millis = (micros / 1000) as f64;
        if let Ok(time) = DateTime::from_ticks(UNIX_EPOCH_TICKS, Kind::Utc).add_milliseconds(millis)
        {
            rxtime = DateTime::from_ticks(time.ticks, Kind::Local);
        }
    }

    // The start byte: `ReadWithTimeout` of one byte throws at the end of a seekable stream.
    let mut readcount = 0usize;
    let mut stx = 0u8;
    while readcount <= MAX_PACKET_LEN {
        let &byte = data.get(*pos).ok_or(EndOfData)?;
        *pos += 1;
        stx = byte;
        if byte == STX_V2 || byte == STX_V1 {
            break;
        }
        readcount += 1;
    }
    if readcount >= MAX_PACKET_LEN {
        return Ok(None);
    }

    let header_len = if stx == STX_V2 {
        CORE_HEADER_LEN
    } else {
        CORE_HEADER_V1_LEN
    };
    let header_len_stx = header_len + 1;
    // The header: an EndOfStreamException here is caught, and nothing is consumed.
    let start = *pos - 1;
    if *pos + header_len > data.len() {
        return Ok(None);
    }
    let header = data.get(start..start + header_len_stx).ok_or(EndOfData)?;
    let payload_len = usize::from(header.get(1).copied().unwrap_or(0));
    let mut length_to_read = payload_len + header_len_stx;
    if stx == STX_V2 && header.get(2).copied().unwrap_or(0) & 0x01 != 0 {
        length_to_read += SIGNATURE_LEN;
    }
    // The rest: `lengthtoread - (headerlengthstx - 2)` more bytes, after the header.
    let rest = length_to_read + 2 - header_len_stx;
    let body_from = *pos + header_len;
    if body_from + rest > data.len() {
        *pos += header_len;
        return Ok(None);
    }
    *pos = body_from + rest;
    let buffer = data.get(start..*pos).ok_or(EndOfData)?;

    let msgid = if stx == STX_V2 {
        u32::from_le_bytes([
            buffer.get(7).copied().unwrap_or(0),
            buffer.get(8).copied().unwrap_or(0),
            buffer.get(9).copied().unwrap_or(0),
            0,
        ])
    } else {
        u32::from(buffer.get(5).copied().unwrap_or(0))
    };
    // The checksum as sent sits after the payload, signature or no signature
    // (MAVLinkMessage.cs:179-182, 204-207)...
    let crc_at = header_len + payload_len + 1;
    let sent = u16::from_le_bytes([
        buffer.get(crc_at).copied().unwrap_or(0),
        buffer.get(crc_at + 1).copied().unwrap_or(0),
    ]);
    // ...while the one computed runs from after the start byte to two bytes short of the end,
    // which for a signed frame takes in the checksum and most of the signature, so no signed
    // frame passes (MavlinkParse.cs:225-235, MavlinkCRC.cs:18-37).
    let covered = buffer.get(1..buffer.len().saturating_sub(2)).unwrap_or(&[]);
    let computed = crc::accumulate(
        DIALECT.crc_extra(msgid).unwrap_or(0),
        crc::accumulate_slice(covered, crc::INIT),
    );
    if computed != sent {
        return Ok(None);
    }
    let payload = buffer
        .get(header_len_stx..header_len_stx + payload_len)
        .unwrap_or(&[])
        .to_vec();
    Ok(Some(Packet {
        rxtime,
        msgid,
        payload,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(micros: u64, frame: &[u8]) -> Vec<u8> {
        let mut out = micros.to_be_bytes().to_vec();
        out.extend_from_slice(frame);
        out
    }

    fn heartbeat_v2(seq: u8) -> Vec<u8> {
        let payload = [0u8, 0, 0, 0, 2, 3, 81, 4, 3];
        let mut frame = vec![STX_V2, 9, 0, 0, seq, 1, 1, 0, 0, 0];
        frame.extend_from_slice(&payload);
        let crc = crc::checksum(frame.get(1..).unwrap(), 50);
        frame.extend_from_slice(&crc.to_le_bytes());
        frame
    }

    #[test]
    fn packets_carry_their_record_time() {
        let mut data = record(1_758_677_405_600_123, &heartbeat_v2(0));
        data.extend(record(1_758_677_405_700_000, &heartbeat_v2(1)));
        let packets = messages_of_type(&data, &[0]).unwrap();
        assert_eq!(packets.len(), 2);
        let first = packets.first().unwrap();
        // Microseconds to whole milliseconds, then local (UTC) time.
        assert_eq!(first.rxtime.to_milliseconds(), 1_758_677_405_600);
        assert_eq!(first.rxtime.kind, Kind::Local);
        assert!(messages_of_type(&data, &[1]).unwrap().is_empty());
    }

    #[test]
    fn a_bad_checksum_drops_the_packet_and_a_cut_scan_throws() {
        let mut frame = heartbeat_v2(0);
        if let Some(last) = frame.last_mut() {
            *last ^= 0xFF;
        }
        let mut data = record(1_758_677_405_600_000, &frame);
        data.extend(record(1_758_677_405_700_000, &heartbeat_v2(1)));
        assert_eq!(messages_of_type(&data, &[0]).unwrap().len(), 1);
        // Garbage after the last record: its eight "timestamp" bytes are read, the scan for a
        // start byte runs off the end, and the whole read is lost.
        data.extend_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
        assert_eq!(messages_of_type(&data, &[0]), Err(EndOfData));
    }
}
