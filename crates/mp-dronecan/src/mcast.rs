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

//! `StartmcastCAN`: a CAN bus carried as UDP multicast, one frame a datagram, to the group
//! `239.65.82.<bus>` on port 57732 - pydronecan's `mcast` driver, which ArduPilot's SITL speaks
//! for its simulated CAN buses.
//!
//! A datagram is `magic` (0x2934), a CRC-16 of what follows ([`crate::crc::compute`]), `flags`
//! (bit 0: CAN FD), the frame's `message_id` (bit 31 set for an extended frame) and the data, the
//! numbers little-endian. One heard becomes an SLCAN line for the node - `B` for CAN FD, though
//! `ReadMessageSLCAN` reads a `B` line as classic - and a line the node writes becomes a datagram.
//! `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:1487-1633`

use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::time::Duration;

use crate::crc::compute;
use crate::frame::{Frame, Payload, data_length_to_dlc, dlc_to_data_length};
use crate::slcan::{self, Line};

/// `MCAST_MAGIC`.
pub const MAGIC: u16 = 0x2934;
/// `MCAST_FLAG_CANFD`.
pub const FLAG_CANFD: u16 = 0x0001;
/// The port, `"57732"`.
pub const PORT: u16 = 57732;

/// `239.65.82.B`: the group of bus `bus` (0 or 1).
#[must_use]
pub const fn group(bus: u8) -> Ipv4Addr {
    Ipv4Addr::new(239, 65, 82, bus)
}

/// A line the node wrote, as `FrameReceived` sends it: the datagram, or `None` for what
/// `ReadMessageSLCAN` drops.
#[must_use]
pub fn datagram_of(line: &str) -> Option<Vec<u8>> {
    let Line::Frame(frame, _, payload) = slcan::read_line(line) else {
        return None;
    };
    let flags = if frame.fd { FLAG_CANFD } else { 0 };
    let message_id = frame
        .id()
        .wrapping_add(if frame.extended { 0x8000_0000 } else { 0 });
    let mut body = Vec::with_capacity(6 + payload.bytes.len());
    body.extend_from_slice(&flags.to_le_bytes());
    body.extend_from_slice(&message_id.to_le_bytes());
    body.extend_from_slice(&payload.bytes);
    let mut packet = Vec::with_capacity(4 + body.len());
    packet.extend_from_slice(&MAGIC.to_le_bytes());
    packet.extend_from_slice(&compute(&body).to_le_bytes());
    packet.extend_from_slice(&body);
    Some(packet)
}

/// A datagram heard, as the receive loop makes a line of it: `None` for a wrong magic or CRC, or
/// one too short to have a header.
#[must_use]
pub fn line_of(datagram: &[u8]) -> Option<String> {
    let word = |at: usize| {
        datagram
            .get(at..at + 2)
            .and_then(|bytes| <[u8; 2]>::try_from(bytes).ok())
            .map(u16::from_le_bytes)
    };
    let magic = word(0)?;
    let crc = word(2)?;
    let flags = word(4)?;
    let id = datagram
        .get(6..10)
        .and_then(|bytes| <[u8; 4]>::try_from(bytes).ok())
        .map(u32::from_le_bytes)?;
    if magic != MAGIC || crc != compute(datagram.get(4..)?) {
        return None;
    }
    let canfd = flags & FLAG_CANFD != 0;
    let frame = Frame::from_id(id & 0x1FFF_FFFF, true, canfd);
    let payload = Payload::new(datagram.get(10..)?.to_vec());
    let length = data_length_to_dlc(payload.bytes.len());
    Some(format!(
        "{}{}{:X}{}\r",
        if canfd { 'B' } else { 'T' },
        frame.to_hex(),
        length,
        payload.to_hex(usize::from(dlc_to_data_length(length)))
    ))
}

/// An interface the multicast list offers: `NetworkInterface`'s `Description`, and its address.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Interface {
    /// What the combo box shows.
    pub description: String,
    /// The IPv4 address the group is joined on; unspecified when it has none.
    pub address: Ipv4Addr,
}

/// `GetAllNetworkInterfaces().Where(SupportsMulticast && Up && GetIPv4Properties() != null)`:
/// on Unix from getifaddrs, each interface once, in the order first listed, with its first IPv4
/// address.
///
/// **Not the C#'s on Windows:** there is no safe interface listing here without a native call,
/// so the list is empty and Connect says "No network interfaces found", as the C# does with none.
#[must_use]
pub fn interfaces() -> Vec<Interface> {
    #[cfg(unix)]
    {
        use nix::net::if_::InterfaceFlags;
        let Ok(addresses) = nix::ifaddrs::getifaddrs() else {
            return Vec::new();
        };
        let mut list: Vec<Interface> = Vec::new();
        for address in addresses {
            let flags = address.flags;
            if !flags.contains(InterfaceFlags::IFF_UP)
                || !flags.contains(InterfaceFlags::IFF_MULTICAST)
            {
                continue;
            }
            let ipv4 = address
                .address
                .as_ref()
                .and_then(|storage| storage.as_sockaddr_in())
                .map(|sin| sin.ip());
            match list
                .iter_mut()
                .find(|known| known.description == address.interface_name)
            {
                Some(known) => {
                    if known.address.is_unspecified()
                        && let Some(ip) = ipv4
                    {
                        known.address = ip;
                    }
                }
                None => list.push(Interface {
                    description: address.interface_name.clone(),
                    address: ipv4.unwrap_or(Ipv4Addr::UNSPECIFIED),
                }),
            }
        }
        list
    }
    #[cfg(not(unix))]
    {
        Vec::new()
    }
}

/// The C#'s `UdpClient`: address reuse, a one-second receive timeout, bound to any address on
/// [`PORT`], the group joined and sent to on the interface.
///
/// # Errors
/// What the socket calls fail with; the C#'s throw out of the click handler.
pub fn open(bus: u8, interface: &Interface) -> std::io::Result<UdpSocket> {
    use socket2::{Domain, Protocol, SockAddr, Socket, Type};
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    socket.set_multicast_if_v4(&interface.address)?;
    socket.set_reuse_address(true)?;
    socket.set_read_timeout(Some(Duration::from_secs(1)))?;
    socket.bind(&SockAddr::from(SocketAddrV4::new(
        Ipv4Addr::UNSPECIFIED,
        PORT,
    )))?;
    socket.join_multicast_v4(&group(bus), &interface.address)?;
    Ok(socket.into())
}

/// Sends a datagram to the bus's group.
///
/// # Errors
/// The send's.
pub fn send(socket: &UdpSocket, bus: u8, datagram: &[u8]) -> std::io::Result<usize> {
    socket.send_to(datagram, SocketAddrV4::new(group(bus), PORT))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node::{Identity, Node};
    use std::time::Instant;

    /// Our node's status as a datagram, and back to the line it was: the header as pydronecan
    /// packs it, the CRC over what follows it.
    #[test]
    fn datagrams_both_ways() {
        let now = Instant::now();
        let mut node = Node::new(Identity::default(), now);
        node.start(now);
        node.tick(now, 1);
        let lines = node.take_outgoing();
        let datagram = datagram_of(&lines[0]).expect("a frame");
        assert_eq!(&datagram[..2], &MAGIC.to_le_bytes());
        assert_eq!(&datagram[4..6], &[0, 0], "classic CAN");
        assert_eq!(
            u32::from_le_bytes([datagram[6], datagram[7], datagram[8], datagram[9]]),
            0x8000_0000 | 0x1E01_557F
        );
        assert_eq!(datagram.len(), 10 + 8);
        let line = line_of(&datagram).expect("a line");
        assert_eq!(line, lines[0]);

        let mut broken = datagram.clone();
        broken[12] ^= 1;
        assert_eq!(line_of(&broken), None, "the CRC");
        broken = datagram;
        broken[0] = 0;
        assert_eq!(line_of(&broken), None, "the magic");
        assert_eq!(line_of(&[0x34, 0x29]), None, "too short");
        assert_eq!(group(1), Ipv4Addr::new(239, 65, 82, 1));
    }

    /// The listing names interfaces once each; on a machine with a loopback, it is there.
    #[cfg(unix)]
    #[test]
    fn interfaces_are_listed_once() {
        let list = interfaces();
        let mut names: Vec<&str> = list.iter().map(|i| i.description.as_str()).collect();
        let count = names.len();
        names.dedup();
        assert_eq!(names.len(), count);
    }
}
