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

//! What a script on the Scripts tab needs of a link that nothing else in the application does.
//!
//! `MAV.BaseStream.Write(buffer, offset, count)` - the shipped `example5 inject data.py` - puts
//! bytes on the port as they are, beside the frames `MAVLinkInterface` writes: `BaseStream` is
//! the `ICommsSerial` itself (`ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:32-66`). Here the
//! link thread owns the port, so the bytes go through its outbound queue with everything else.

use crate::LinkSender;

/// The most bytes one queued write carries. The link thread stamps its sequence number into the
/// fifth byte of everything it takes off the queue (a v2 frame's `seq`, see `run_link`'s
/// "Outbound queue"), and recomputes a checksum when the bytes parse as a frame; pieces of four
/// have no fifth byte and no frame, so they go out untouched and in order.
const RAW_PIECE: usize = 4;

impl LinkSender {
    /// `BaseStream.Write(buffer, 0, count)`: `bytes` on the port as they are, not framed.
    /// Whether all of them were queued; false once the link has stopped, where the C#'s closed
    /// port throws.
    ///
    /// **Not quite the C#'s:** the pieces are queued one after another, so a frame another
    /// thread queues at that moment can land between two of them - as a frame written by the
    /// C#'s own reader thread can land inside a script's `Write`, which takes no lock. A
    /// recording (`.tlog`) holds each piece as a record of its own.
    pub fn write_raw(&self, bytes: &[u8]) -> bool {
        bytes
            .chunks(RAW_PIECE)
            .all(|piece| self.outbound.send(piece.to_vec()).is_ok())
    }
}

#[cfg(test)]
mod tests {
    use web_time::{Duration, Instant};

    use mp_transport::Transport as _;
    use mp_transport::testing::Loopback;

    use crate::{Link, LinkConfig};

    /// The six bytes example5 writes reach the other end exactly as written - a fifth byte the
    /// link would re-stamp as a frame's sequence number included - and nothing else is sent with
    /// the heartbeat and the stream requests off.
    #[test]
    fn raw_bytes_reach_the_port_as_they_are() {
        let (gcs_side, mut vehicle_side) = Loopback::pair();
        let config = LinkConfig {
            stream_rate_hz: 0,
            send_heartbeat: false,
            ..LinkConfig::default()
        };
        let link = Link::from_transport(Box::new(gcs_side), config);
        let key = [0x13, 0x00, 0x00, 0x00, 0x08, 0x00, 0xfd, 0x09];
        assert!(link.sender().write_raw(&key));
        let mut got = Vec::new();
        let deadline = Instant::now() + Duration::from_secs(10);
        while got.len() < key.len() && Instant::now() < deadline {
            let mut buf = [0u8; 64];
            let n = vehicle_side.read(&mut buf).unwrap_or(0);
            got.extend_from_slice(buf.get(..n).unwrap_or(&[]));
            if n == 0 {
                wasm_thread::sleep(Duration::from_millis(1));
            }
        }
        assert_eq!(got, key);
    }
}
