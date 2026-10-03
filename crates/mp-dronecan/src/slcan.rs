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

//! SLCAN, the Lawicel serial-line CAN protocol: one frame a line, `T` + eight hex digits of
//! identifier + one of length + the data in hex, ended by `\r`.
//!
//! The C# routes every frame through it: the SLCAN adapter's lines directly, and the MAVLink and
//! multicast buses by making a line of each `CAN_FRAME` or datagram and writing it into a
//! `CommsInjection` the node reads like a port (`ConfigDroneCAN.cs:155-196, 1564-1626`). The
//! lines here are made and read as the C# makes and reads them, so whatever a bus delivers comes
//! out as it would there.
//! `// C#: ExtLibs/DroneCAN/DroneCAN.cs:1396-1421, 1769-1863;
//! GCSViews/ConfigurationView/ConfigDroneCAN.cs:155-196`

use crate::frame::{Frame, Payload, data_length_to_dlc, dlc_to_data_length};

/// What a line read was.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Line {
    /// A data frame: its identifier as read, the identifier's value (the reassembly key), and its
    /// data.
    Frame(Frame, u32, Payload),
    /// `N`: the adapter's serial number answer, which the C# writes to its console.
    Serial(String),
    /// `Z`: a transmit acknowledgement, `cmdack`.
    Ack,
    /// Anything else, or a line the C# would throw on: nothing.
    Ignored,
}

/// `hextoint`: a hex digit's value, anything else 0.
fn hex_digit(digit: char) -> u8 {
    digit
        .to_digit(16)
        .and_then(|value| u8::try_from(value).ok())
        .unwrap_or(0)
}

/// `ReadMessageSLCAN(line)`: a line, as the reader hands it on (its `\r` or `\a` included or not).
#[must_use]
pub fn read_line(line: &str) -> Line {
    // `line_len` is taken before the bell is stripped.
    if line.chars().count() <= 4 {
        return Line::Ignored;
    }
    let line = if line.starts_with('\u{7}') {
        line.replace('\u{7}', "")
    } else {
        line.to_owned()
    };
    let chars: Vec<char> = line.chars().collect();
    let Some(kind) = chars.first().copied() else {
        return Line::Ignored;
    };
    let (id_len, fd) = match kind {
        'T' | 'B' => (8, false),
        't' | 'b' => (3, false),
        'D' => (8, true),
        'd' => (3, true),
        'N' => return Line::Serial(line),
        'Z' => return Line::Ack,
        _ => return Line::Ignored,
    };
    if chars.len() < 1 + id_len + 1 {
        return Line::Ignored;
    }
    let id_text: String = chars.iter().skip(1).take(id_len).collect();
    if id_text.contains('T') {
        // "Bad SLCAN": a line run into the next; read from the next `T`.
        let next = chars
            .iter()
            .skip(1)
            .position(|c| *c == 'T')
            .map(|at| chars.iter().skip(1 + at).collect::<String>());
        return next.map_or(Line::Ignored, |rest| read_line(&rest));
    }
    // `Convert.ToUInt32(msgdata, 16)` and `Convert.ToByte(..., 16)` throw on what is not hex.
    let Ok(packet_id) = u32::from_str_radix(&id_text, 16) else {
        return Line::Ignored;
    };
    let Some(dlc) = chars
        .get(1 + id_len)
        .and_then(|c| c.to_digit(16))
        .and_then(|value| u8::try_from(value).ok())
    else {
        return Line::Ignored;
    };
    let packet_len = usize::from(dlc_to_data_length(dlc));
    if packet_len == 0 {
        return Line::Ignored;
    }
    let digits: Vec<char> = chars
        .iter()
        .skip(1 + 1 + id_len)
        .take(packet_len * 2)
        .copied()
        .collect();
    // `NowNextBy2`: pairs; an odd last digit is dropped.
    let data: Vec<u8> = digits
        .chunks_exact(2)
        .map(|pair| match pair {
            [high, low] => (hex_digit(*high) << 4) + hex_digit(*low),
            _ => 0,
        })
        .collect();
    if data.is_empty() {
        return Line::Ignored;
    }
    Line::Frame(
        Frame::from_id(packet_id, true, fd),
        packet_id,
        Payload::new(data),
    )
}

/// One frame of `PackageMessageSLCAN`'s text: `T` (or `B` on CAN FD), the identifier, the data
/// length code, the data padded to the code's length, `\r`.
#[must_use]
pub fn format_frame(frame: &Frame, payload: &Payload, canfd: bool) -> String {
    let dlc = data_length_to_dlc(payload.bytes.len());
    format!(
        "{}{}{:X}{}\r",
        if canfd { 'B' } else { 'T' },
        frame.to_hex(),
        dlc,
        payload.to_hex(usize::from(dlc_to_data_length(dlc)))
    )
}

/// The line the page's subscription makes of a `CAN_FRAME`: its identifier as it came (with
/// ArduPilot's extended flag in bit 31), its length, and all eight data bytes - the length
/// decides how many the reader takes.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:169-183`
#[must_use]
pub fn from_can_frame(id: u32, len: u8, data: &[u8]) -> String {
    let frame = Frame::from_id(id, true, false);
    let payload = Payload::new(data.to_vec());
    format!(
        "T{}{:X}{}\r",
        frame.to_hex(),
        len,
        payload.to_hex(usize::from(dlc_to_data_length(len)))
    )
}

/// `WriteToStreamSLCAN`'s split: the lines of a string of frames, each without its `\r`.
pub fn lines(text: &str) -> impl Iterator<Item = &str> {
    text.split('\r').filter(|line| !line.is_empty())
}

/// The commands `StartSLCAN` writes to an adapter, in order, each followed by a read of its
/// answer: close, the speed - `(byte)Baud.baud1mbit`, the byte 8 rather than the digit, as the
/// C# writes it - the serial number, the version, open, and clear the status.
/// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:187-232`
pub const OPEN_COMMANDS: [&[u8]; 6] = [b"C\r", b"S\x08\r", b"N\r", b"V\r", b"O\r", b"F\r"];

/// `"\r\r\r"`, written first to clear whatever the adapter had half-received.
pub const CLEANUP: &[u8] = b"\r\r\r";

/// `Stop(closestream)`'s close command.
pub const CLOSE: &[u8] = b"C\r";

#[cfg(test)]
mod tests {
    use super::*;

    /// A recorded frame - `08042479`, a GNSS fix from node 121 - read as the C# reads it.
    /// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:1561-1572`
    #[test]
    fn a_recorded_line_reads() {
        let Line::Frame(frame, packet_id, payload) = read_line("T080424798EB3C000000000086\r")
        else {
            panic!("a frame");
        };
        assert_eq!(packet_id, 0x0804_2479);
        assert_eq!(frame.source_node(), 121);
        assert_eq!(frame.msg_type_id(), 0x0424);
        assert_eq!(frame.priority(), 8);
        assert_eq!(payload.bytes, [0xEB, 0x3C, 0, 0, 0, 0, 0, 0x86]);
        assert!(payload.sot() && !payload.eot());
    }

    /// What is made reads back; the length and the junk follow the C#'s rules.
    #[test]
    fn made_lines_read_back() {
        let mut frame = Frame::from_id(0, true, false);
        frame.set_source_node(127);
        frame.set_priority(30);
        frame.set_msg_type_id(341);
        let payload = Payload::new(vec![1, 2, 3, 0xC0]);
        let text = format_frame(&frame, &payload, false);
        assert_eq!(text, "T1E01557F4010203C0\r");
        assert_eq!(
            read_line(&text),
            Line::Frame(frame, frame.id(), payload.clone())
        );
        assert_eq!(read_line("T1E0"), Line::Ignored, "too short");
        assert_eq!(read_line("T1E01557F0\r"), Line::Ignored, "no data");
        assert_eq!(read_line("X1E01557F10\r"), Line::Ignored);
        assert_eq!(read_line("Z\r\r\r\r"), Line::Ack);
        assert_eq!(read_line("NA123\r"), Line::Serial("NA123\r".to_owned()));
        // The bell before a line goes; a line run into another reads from the next `T`.
        assert_eq!(
            read_line(&format!("\u{7}{text}")),
            Line::Frame(frame, frame.id(), payload.clone())
        );
        assert_eq!(
            read_line(&format!("T12{text}")),
            Line::Frame(frame, frame.id(), payload)
        );
    }

    /// A `CAN_FRAME` from ArduPilot - bit 31 set, eight bytes of data of which three count -
    /// becomes the line the C# makes, which reads back with the flag kept and the three bytes.
    #[test]
    fn a_can_frame_becomes_a_line() {
        let id = 0x8000_0000 | 0x1E01_550A;
        let text = from_can_frame(id, 3, &[1, 2, 0xC5, 9, 9, 9, 9, 9]);
        assert_eq!(text, format!("T9E01550A30102C5{}\r", "09".repeat(5)));
        let Line::Frame(frame, packet_id, payload) = read_line(&text) else {
            panic!("a frame");
        };
        assert_eq!(packet_id, id);
        assert_eq!(frame.source_node(), 10);
        assert_eq!(frame.priority(), 30);
        assert_eq!(payload.bytes, [1, 2, 0xC5]);
        assert_eq!(lines("a\rb\r\r").collect::<Vec<_>>(), ["a", "b"]);
    }
}
