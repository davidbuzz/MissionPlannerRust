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

//! Transfers: a message's payload cut into frames (`PackageMessage`) and frames put back into a
//! payload (`ProcessFrame`).
//!
//! A payload of up to seven bytes (63 on CAN FD) goes in one frame with no CRC. A longer one
//! starts with its CRC - over the type's signature and the payload ([`crate::crc`]) - in two
//! bytes, the first frame carrying five bytes of payload after it and each later one seven, every
//! frame closed by its tail byte: start, end, toggle (from false, flipping each frame) and the
//! transfer id.
//!
//! Reassembly is the C#'s, quirks kept: transfers are told apart by the whole frame identifier
//! and the transfer id; a start of transfer begins one afresh; a frame for a transfer that has not
//! started is dropped; a toggle out of turn drops the transfer unless that frame ends it, when it
//! is let through; and a single-frame transfer's payload is not checked, having no CRC.
//! `// C#: ExtLibs/DroneCAN/DroneCAN.cs:1423-1519, 1865-2010`

use std::collections::HashMap;

use crate::crc::transfer_crc;
use crate::dsdl::Message;
use crate::frame::{Frame, Payload};
use crate::names::lookup;

/// How a message is addressed: `PackageMessage` decides from the type's name.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Addressing {
    /// A broadcast: the identifier carries the type id.
    Message,
    /// A service: the identifier carries the destination, request or response, and the
    /// service's id.
    Service {
        /// `destNode`.
        destination: u8,
        /// A `_req` type.
        request: bool,
    },
}

/// `PackageMessage(destNode, priority, transferID, msg, canfd)`: the frames of one transfer.
#[must_use]
pub fn package(
    source_node: u8,
    destination: u8,
    priority: u8,
    transfer_id: u8,
    message: &Message,
    canfd: bool,
) -> Vec<(Frame, Payload)> {
    let info = message.info();
    let payload = message.encode(canfd);
    let mut frame = Frame::from_id(0, true, false);
    frame.set_source_node(source_node);
    frame.set_priority(priority);
    if info.is_message() {
        frame.set_msg_type_id(info.id);
    } else {
        frame.set_service(true);
        frame.set_svc_destination_node(destination);
        frame.set_svc_is_request(info.is_request());
        frame.set_svc_type_id(info.id.to_le_bytes()[0]);
    }
    let frame_size = if canfd { 63 } else { 7 };
    if payload.len() <= frame_size {
        let mut bytes = payload;
        bytes.push(0);
        let mut tail = Payload::new(bytes);
        tail.set_sot(true);
        tail.set_eot(true);
        tail.set_transfer_id(transfer_id);
        return vec![(frame, tail)];
    }
    let crc = transfer_crc(info.signature, &payload).to_le_bytes();
    let mut frames = Vec::new();
    let mut toggle = false;
    let mut at = 0;
    while at < payload.len() {
        let mut bytes;
        let size;
        if at == 0 {
            // The first frame: the CRC, then `framesize - 2` bytes of payload.
            size = frame_size - 2;
            bytes = crc.to_vec();
            bytes.extend(payload.iter().take(size));
        } else {
            size = (payload.len() - at).min(frame_size);
            bytes = payload.iter().skip(at).take(size).copied().collect();
        }
        bytes.push(0);
        let mut tail = Payload::new(bytes);
        tail.set_sot(at == 0);
        tail.set_eot(at + size >= payload.len());
        tail.set_transfer_id(transfer_id);
        tail.set_toggle(toggle);
        toggle = !toggle;
        frames.push((frame, tail));
        at += size;
    }
    frames
}

/// What a frame came to.
#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    /// Nothing yet: the transfer goes on, or the frame belonged to nothing.
    Pending,
    /// `FrameError`: a toggle out of turn, a CRC that does not match, a type that would not
    /// decode.
    Error,
    /// A transfer of a type the table does not know, or one its search cannot fit: dropped.
    Unknown,
    /// A whole transfer, decoded.
    Complete(Box<(Frame, Message, u8)>),
}

/// `transfer` and `transferToggle`: the transfers being put back together.
#[derive(Debug, Default)]
pub struct Reassembler {
    transfers: HashMap<(u32, u8), (Vec<u8>, bool)>,
}

impl Reassembler {
    /// An empty one.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// `ProcessFrame(frame, packet_id, payload)`.
    pub fn push(&mut self, frame: Frame, packet_id: u32, payload: &Payload) -> Outcome {
        let key = (packet_id, payload.transfer_id());
        if payload.sot() {
            self.transfers.insert(key, (Vec::new(), false));
        }
        let Some((bytes, toggle)) = self.transfers.get_mut(&key) else {
            return Outcome::Pending;
        };
        bytes.extend_from_slice(payload.data());
        if *toggle != payload.toggle() && !payload.eot() {
            self.transfers.remove(&key);
            return Outcome::Error;
        }
        *toggle = !*toggle;
        if !payload.eot() {
            return Outcome::Pending;
        }
        let Some((result, _)) = self.transfers.remove(&key) else {
            return Outcome::Pending;
        };
        let Some(info) = lookup(&frame) else {
            return Outcome::Unknown;
        };
        let mut start = 0;
        if !payload.sot() {
            // The end of a multi-frame transfer: its CRC first.
            start = 2;
            let (Some(low), Some(high)) = (result.first(), result.get(1)) else {
                return Outcome::Error;
            };
            let crc = u16::from_le_bytes([*low, *high]);
            let body = result.get(start..).unwrap_or(&[]);
            if transfer_crc(info.signature, body) != crc {
                return Outcome::Error;
            }
        }
        let body = result.get(start..).unwrap_or(&[]);
        let message = Message::decode(info, body, frame.fd);
        let mut frame = frame;
        frame.size_of_entire_msg = body.len();
        Outcome::Complete(Box::new((frame, message, payload.transfer_id())))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsdl::{GetSetRes, NodeStatus, OPCODE_SAVE, Value};
    use crate::names::by_name;

    /// `SaveConfig`'s recorded request, made again: node 126 asking 121, transfer 1.
    /// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:432-433`
    #[test]
    fn the_recorded_execute_opcode_request() {
        let frames = package(
            126,
            121,
            30,
            1,
            &Message::ExecuteOpcodeReq {
                opcode: OPCODE_SAVE,
                argument: 0,
            },
            false,
        );
        assert_eq!(frames.len(), 1);
        let (frame, payload) = &frames[0];
        assert_eq!(frame.id(), 0x1E0A_F9FE);
        assert_eq!(payload.bytes, [0, 0, 0, 0, 0, 0, 0, 0xC1]);

        // And the answer the comment records, read back.
        let mut reassembler = Reassembler::new();
        let answer = Payload::new(vec![0, 0, 0, 0, 0, 0, 0x80, 0xC1]);
        let Outcome::Complete(done) = reassembler.push(
            Frame::from_id(0x1E0A_7EF9, true, false),
            0x1E0A_7EF9,
            &answer,
        ) else {
            panic!("a whole transfer");
        };
        let (frame, message, transfer_id) = *done;
        assert_eq!(frame.source_node(), 121);
        assert_eq!(transfer_id, 1);
        assert_eq!(
            message,
            Message::ExecuteOpcodeRes {
                argument: 0,
                ok: true
            }
        );
    }

    /// A multi-frame transfer: CRC first, five bytes then sevens, toggles flipping, the end
    /// marked; and back again through the reassembler, CRC checked.
    #[test]
    fn multi_frame_round_trip() {
        let message = Message::GetSetRes(GetSetRes {
            value: Value::Integer(42),
            default_value: Value::Real(1.5),
            name: b"SOME_PARAMETER_NAME".to_vec(),
            ..GetSetRes::default()
        });
        let payload_len = message.encode(false).len();
        let frames = package(10, 127, 30, 7, &message, false);
        assert_eq!(frames.len(), 1 + (payload_len - 5).div_ceil(7));
        for (index, (_, payload)) in frames.iter().enumerate() {
            assert_eq!(payload.sot(), index == 0);
            assert_eq!(payload.eot(), index == frames.len() - 1);
            assert_eq!(payload.toggle(), index % 2 == 1);
            assert_eq!(payload.transfer_id(), 7);
            assert!(payload.bytes.len() <= 8);
        }
        let signature = by_name("uavcan_protocol_param_GetSet_res")
            .map(|row| row.signature)
            .unwrap_or_default();
        let crc = transfer_crc(signature, &message.encode(false)).to_le_bytes();
        assert_eq!(frames[0].1.bytes[..2], crc);

        let mut reassembler = Reassembler::new();
        let mut outcome = Outcome::Pending;
        for (frame, payload) in &frames {
            outcome = reassembler.push(*frame, frame.id(), payload);
        }
        let Outcome::Complete(done) = outcome else {
            panic!("a whole transfer: {outcome:?}");
        };
        assert_eq!(done.1, message);
        assert_eq!(done.0.size_of_entire_msg, payload_len);
        assert_eq!(done.0.svc_destination_node(), 127);
        assert!(!done.0.svc_is_request());
    }

    /// A corrupted byte fails the CRC; a frame out of turn drops the transfer; a frame of a
    /// transfer never started is ignored.
    #[test]
    fn errors() {
        let message = Message::GetSetRes(GetSetRes {
            name: b"A_MUCH_LONGER_PARAMETER_NAME".to_vec(),
            ..GetSetRes::default()
        });
        let frames = package(10, 127, 30, 3, &message, false);
        assert!(frames.len() > 3);

        let mut reassembler = Reassembler::new();
        let mut outcome = Outcome::Pending;
        for (index, (frame, payload)) in frames.iter().enumerate() {
            let mut payload = payload.clone();
            if index == 1 {
                payload.bytes[0] ^= 0xff;
            }
            outcome = reassembler.push(*frame, frame.id(), &payload);
        }
        assert_eq!(outcome, Outcome::Error, "bad CRC");

        let mut reassembler = Reassembler::new();
        let (first, second) = (&frames[0], &frames[2]);
        assert_eq!(
            reassembler.push(first.0, first.0.id(), &first.1),
            Outcome::Pending
        );
        // The third frame straight after the first: its toggle is the first's, not the flip.
        assert!(!second.1.eot());
        assert_eq!(
            reassembler.push(second.0, second.0.id(), &second.1),
            Outcome::Error,
            "toggle out of turn"
        );

        let mut reassembler = Reassembler::new();
        let (frame, middle) = &frames[1];
        assert_eq!(
            reassembler.push(*frame, frame.id(), middle),
            Outcome::Pending,
            "never started"
        );
    }

    /// A single frame needs no CRC and decodes at once.
    #[test]
    fn single_frame() {
        let status = NodeStatus {
            uptime_sec: 99,
            ..NodeStatus::default()
        };
        let frames = package(127, 0, 30, 31, &Message::NodeStatus(status), false);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].0.msg_type_id(), 341);
        let mut reassembler = Reassembler::new();
        let Outcome::Complete(done) = reassembler.push(frames[0].0, frames[0].0.id(), &frames[0].1)
        else {
            panic!("decoded");
        };
        assert_eq!(done.1, Message::NodeStatus(status));
        assert_eq!(done.2, 31);
    }
}
