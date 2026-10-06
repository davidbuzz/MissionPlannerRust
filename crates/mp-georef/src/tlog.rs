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
//! `// C#: ExtLibs/Utilities/Extensions.cs:746-760, ExtLibs/Mavlink/MavlinkParse.cs:132-240`

use mp_mavlink::crc;
use mp_mavlink::{Dialect, STX_V1, STX_V2, SUPPORTED_INCOMPAT_FLAGS, v2_header_len, v2_layout};
use mp_mavlink_dialects::DIALECT;

use crate::time::{DateTime, Kind, UNIX_EPOCH_TICKS};

/// `MAVLINK_MAX_PACKET_LEN`: payload, the widest header (`SYSID32` and `TARGET32`), checksum,
/// and a signature. `// C#: ExtLibs/Mavlink/Mavlink.cs:18-19`
const MAX_PACKET_LEN: usize = 255 + 17 + 2 + 13;
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
/// which `ReadPacket` does not catch (`MavlinkParse.cs:158-169`), so the exception leaves
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
/// `// C#: ExtLibs/Mavlink/MavlinkParse.cs:132-240`
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
    let flags = if stx == STX_V2 {
        header.get(2).copied().unwrap_or(0)
    } else {
        0
    };
    // A v2 header as long as its flags make it (`GetHeaderLength`, e6454ccdd).
    let full_header = if stx == STX_V2 {
        v2_header_len(flags)
    } else {
        header_len_stx
    };
    let mut length_to_read = payload_len + full_header;
    if flags & 0x01 != 0 {
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
    // A flag this reader does not know: the packet read, and dropped. C#: MavlinkParse.cs:217-218
    if flags & !SUPPORTED_INCOMPAT_FLAGS != 0 {
        return Ok(None);
    }

    let msgid = if stx == STX_V2 {
        v2_layout(buffer).map_or(0, |(_, msgid)| msgid)
    } else {
        u32::from(buffer.get(5).copied().unwrap_or(0))
    };
    // The checksum as sent sits after the payload, and the one computed runs over the header and
    // the payload - a signed frame's too, since e6454ccdd measured them from the message's own
    // header length; before it the sum ran to two bytes short of the end and no signed frame
    // passed. C#: MavlinkParse.cs:219-237; MAVLinkMessage.cs:238-239
    let crc_at = full_header + payload_len;
    let sent = u16::from_le_bytes([
        buffer.get(crc_at).copied().unwrap_or(0),
        buffer.get(crc_at + 1).copied().unwrap_or(0),
    ]);
    let covered = buffer.get(1..crc_at).unwrap_or(&[]);
    let computed = crc::accumulate(
        DIALECT.crc_extra(msgid).unwrap_or(0),
        crc::accumulate_slice(covered, crc::INIT),
    );
    if computed != sent {
        return Ok(None);
    }
    let payload = buffer
        .get(full_header..full_header + payload_len)
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

    /// A heartbeat whose header carries flags: its checksum worked out over that header.
    fn heartbeat_flagged(flags: u8, sysid: [u8; 4], target: Option<[u8; 4]>) -> Vec<u8> {
        let payload = [0u8, 0, 0, 0, 2, 3, 81, 4, 3];
        let mut frame = vec![STX_V2, 9, flags, 0, 0];
        if flags & 0x02 != 0 {
            frame.extend_from_slice(&sysid);
        } else {
            frame.push(sysid[0]);
        }
        frame.extend_from_slice(&[1, 0, 0, 0]);
        if let Some(target) = target {
            frame.extend_from_slice(&target);
        }
        frame.extend_from_slice(&payload);
        let crc = crc::checksum(frame.get(1..).unwrap(), 50);
        frame.extend_from_slice(&crc.to_le_bytes());
        if flags & 0x01 != 0 {
            frame.extend_from_slice(&[7; 13]);
        }
        frame
    }

    /// `ReadPacket` as of e6454ccdd: a header as long as its flags make it - a 32-bit sender and
    /// target - and a signed frame both read, the checksum over the header and payload alone; a
    /// flag it does not know, read and dropped.
    /// `// C#: ExtLibs/Mavlink/MavlinkParse.cs:189-237`
    #[test]
    fn wide_and_signed_frames_are_read_and_an_unknown_flag_is_dropped() {
        let wide = heartbeat_flagged(0x06, 70_000_u32.to_le_bytes(), Some([9, 0, 1, 0]));
        let signed = heartbeat_flagged(0x01, [1, 0, 0, 0], None);
        let unknown = heartbeat_flagged(0x08, [1, 0, 0, 0], None);
        let mut data = record(1_758_677_405_600_000, &wide);
        data.extend(record(1_758_677_405_700_000, &unknown));
        data.extend(record(1_758_677_405_800_000, &signed));
        let packets = messages_of_type(&data, &[0]).unwrap();
        assert_eq!(packets.len(), 2);
        for packet in &packets {
            assert_eq!(packet.payload, [0, 0, 0, 0, 2, 3, 81, 4, 3]);
        }
        assert_eq!(
            packets
                .iter()
                .map(|p| p.rxtime.to_milliseconds())
                .collect::<Vec<_>>(),
            [1_758_677_405_600, 1_758_677_405_800]
        );
    }
}
