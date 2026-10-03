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

//! The UDP client: Mission Planner's "UDPCl", which sends to a host rather than waiting to hear
//! from one.
//!
//! Ported from `ExtLibs/Comms/CommsUDPSerialConnect.cs` (`UdpSerialConnect`). Where
//! [`crate::UdpTransport`] binds a port and learns its peer from the first datagram (the C#'s
//! `UdpSerial`), this one is told the host and port, sends there from a port of its own, and reads
//! whatever comes back to that port, from anyone.
//!
//! What the C# has and this keeps: no reconnect (its `VerifyConnected` is empty), `IsOpen` true
//! from `Open` until `Close` whatever the socket does, writes that never fail, a datagram longer
//! than the read kept for the next read, and a multicast host joined on its own port.

use std::fmt::Write as _;
use std::io;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr, ToSocketAddrs, UdpSocket};
use std::time::Duration;

use crate::socket::is_timeout;
use crate::{DEFAULT_READ_TIMEOUT, OpenError, Transport};

/// The largest UDP payload, so one receive always takes a whole datagram.
const LARGEST_DATAGRAM: usize = 65_536;

/// Mission Planner's UDP client link.
#[derive(Debug)]
pub struct UdpClientTransport {
    socket: UdpSocket,
    /// `hostEndPoint`: where every write goes.
    host: SocketAddr,
    /// `RemoteIpEndPoint`: who the last datagram came from.
    remote: Option<SocketAddr>,
    /// `rbuffer`: what has been received and not yet read, from `cursor` on.
    received: Vec<u8>,
    cursor: usize,
    /// Where one datagram is received before it joins `received`.
    scratch: Vec<u8>,
    /// Whether the host is a multicast group this socket joined.
    multicast: bool,
    /// `IsOpen`: set by `Open`, cleared by `Close`, and nothing else.
    open: bool,
    description: String,
}

impl UdpClientTransport {
    /// `Open(host, port)`: resolves the host, and binds a socket to send to it from.
    ///
    /// The C#'s parameterless `Open` asks for the host and port in two input boxes, defaulting to
    /// `UDP_host` and `UDP_port` from config.xml, and saves the answers back; here the link URL
    /// carries them (`udpcl:host:port`) and saving them is the settings layer's business.
    /// `// C#: ExtLibs/Comms/CommsUDPSerialConnect.cs:74-150`
    pub fn open(host: &str, port: u16) -> Result<Self, OpenError> {
        // C#: CommsUDPSerialConnect.cs:83-90 - an address as written, else the first address DNS
        // gives for the name.
        let host_endpoint = match host.parse::<IpAddr>() {
            Ok(address) => SocketAddr::new(address, port),
            Err(_) => (host, port)
                .to_socket_addrs()
                .map_err(|e| OpenError::io(format!("resolving {host}"), e))?
                .next()
                .ok_or_else(|| {
                    OpenError::io(
                        format!("resolving {host}"),
                        io::Error::new(io::ErrorKind::NotFound, "no addresses"),
                    )
                })?,
        };
        let any = match host_endpoint {
            SocketAddr::V4(_) => IpAddr::V4(Ipv4Addr::UNSPECIFIED),
            SocketAddr::V6(_) => IpAddr::V6(Ipv6Addr::UNSPECIFIED),
        };

        let multicast = is_in_range(host_endpoint.ip());
        let socket = if multicast {
            // C#: CommsUDPSerialConnect.cs:92-111 - a multicast host is heard on its own port, so
            // the socket is bound to that port and joins the group. The C# joins again every 30 s
            // while open, but a second join of a group the socket is already in fails
            // (AddressAlreadyInUse, golden/udp.txt) and its `catch { return; }` ends the loop: what
            // it does is one join, so that is what this does. It joins by parsing the host as
            // written, so a name that resolves to a group is bound but never joined - kept.
            let socket = UdpSocket::bind(SocketAddr::new(any, port))
                .map_err(|e| OpenError::io(format!("binding port {port}"), e))?;
            match host.parse::<IpAddr>() {
                Ok(IpAddr::V4(group)) => {
                    let _ = socket.join_multicast_v4(&group, &Ipv4Addr::UNSPECIFIED);
                }
                Ok(IpAddr::V6(group)) => {
                    let _ = socket.join_multicast_v6(&group, 0);
                }
                Err(_) => {}
            }
            socket
        } else {
            // C#: CommsUDPSerialConnect.cs:115 - `new UdpClient(family)` binds on its first send, to
            // a port the system picks; binding it now picks the same kind of port, and nothing can
            // arrive at it before the far end has been sent something anyway.
            UdpSocket::bind(SocketAddr::new(any, 0))
                .map_err(|e| OpenError::io("binding a local port", e))?
        };
        socket
            .set_read_timeout(Some(DEFAULT_READ_TIMEOUT))
            .map_err(|e| OpenError::io("setting read timeout", e))?;

        let mut description = String::new();
        let _ = write!(description, "udpcl:{host_endpoint}");
        Ok(Self {
            socket,
            host: host_endpoint,
            remote: None,
            received: Vec::new(),
            cursor: 0,
            scratch: vec![0; LARGEST_DATAGRAM],
            multicast,
            open: true,
            description,
        })
    }

    /// `hostEndPoint`: where writes go.
    #[must_use]
    pub const fn host(&self) -> SocketAddr {
        self.host
    }

    /// `RemoteIpEndPoint`: who sent the last datagram read, which need not be the host - the C#
    /// reads from anyone ("assumes the udp packets are mavlink aligned, if we are receiving from
    /// more than one source").
    #[must_use]
    pub const fn remote(&self) -> Option<SocketAddr> {
        self.remote
    }

    /// The local address the socket is bound to.
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    /// `BytesToRead`: bytes received and not yet read, plus the socket's `Available`.
    ///
    /// `Available` is the operating system's FIONREAD, which differs for UDP: Linux counts only
    /// the next datagram (100 with a 100- and a 50-byte datagram queued, golden/udp.txt), Windows
    /// and the BSDs every byte queued. Both are kept, per platform.
    /// `// C#: ExtLibs/Comms/CommsUDPSerialConnect.cs:66`
    pub fn bytes_to_read(&mut self) -> io::Result<usize> {
        let buffered = self.received.len() - self.cursor;
        Ok(buffered + self.available()?)
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn available(&mut self) -> io::Result<usize> {
        self.socket.set_nonblocking(true)?;
        let peeked = self.socket.peek_from(&mut self.scratch);
        self.socket.set_nonblocking(false)?;
        match peeked {
            Ok((size, _)) => Ok(size),
            Err(e) if is_timeout(&e) => Ok(0),
            Err(e) => Err(e),
        }
    }

    #[cfg(not(any(target_os = "linux", target_os = "android")))]
    fn available(&mut self) -> io::Result<usize> {
        // Every queued byte: take the datagrams into the read buffer, where they are read from
        // next anyway, and count what is there.
        let before = self.received.len();
        self.socket.set_nonblocking(true)?;
        let drained = loop {
            match self.socket.recv_from(&mut self.scratch) {
                Ok((size, from)) => {
                    self.received
                        .extend_from_slice(self.scratch.get(..size).unwrap_or_default());
                    self.remote = Some(from);
                }
                Err(e) if is_timeout(&e) => break Ok(()),
                Err(e) => break Err(e),
            }
        };
        self.socket.set_nonblocking(false)?;
        drained?;
        Ok(self.received.len() - before)
    }

    /// Hands out what has been received, from the cursor on.
    fn take(&mut self, buf: &mut [u8]) -> usize {
        let waiting = self.received.get(self.cursor..).unwrap_or_default();
        let n = waiting.len().min(buf.len());
        if let (Some(to), Some(from)) = (buf.get_mut(..n), waiting.get(..n)) {
            to.copy_from_slice(from);
        }
        self.cursor += n;
        n
    }
}

impl Transport for UdpClientTransport {
    /// `Read`: from what is buffered, else one datagram, waiting up to the read timeout for it.
    ///
    /// The C#'s `Read(buf, 0, length)` waits until it holds `length` bytes or `ReadTimeout` (500
    /// ms) passes, which suits `MAVLinkInterface` asking for exactly the frame it expects. This
    /// crate's contract is to return what has arrived, so a read here returns as soon as there is
    /// a datagram, and the rest of a datagram longer than `buf` stays for the next read, as it
    /// stays in the C#'s `rbuffer`.
    /// `// C#: ExtLibs/Comms/CommsUDPSerialConnect.cs:163-201`
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        // C#: CommsUDPSerialConnect.cs:166
        if buf.is_empty() {
            return Ok(0);
        }
        if self.cursor == self.received.len() && self.open {
            // C#: CommsUDPSerialConnect.cs:172-173 - all read, so the buffer starts again.
            self.received.clear();
            self.cursor = 0;
            match self.socket.recv_from(&mut self.scratch) {
                Ok((size, from)) => {
                    self.received
                        .extend_from_slice(self.scratch.get(..size).unwrap_or_default());
                    // C#: CommsUDPSerialConnect.cs:189
                    self.remote = Some(from);
                }
                Err(e) if is_timeout(&e) => return Ok(0),
                Err(e) => return Err(e),
            }
        }
        Ok(self.take(buf))
    }

    /// `Write`: one datagram to the host; a failure is swallowed, as the C# swallows it.
    ///
    /// The C#'s `Write(buf, offset, length)` sends `buf[0..length]` whatever the offset; a slice
    /// has no offset to get wrong.
    /// `// C#: ExtLibs/Comms/CommsUDPSerialConnect.cs:251-261`
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        let _ = self.socket.send_to(buf, self.host);
        Ok(())
    }

    fn description(&self) -> &str {
        &self.description
    }

    /// `IsOpen`: from `Open` to `Close`, whatever the socket does in between.
    /// `// C#: ExtLibs/Comms/CommsUDPSerialConnect.cs:70, 97, 118, 309`
    fn is_open(&self) -> bool {
        self.open
    }

    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        self.socket.set_read_timeout(Some(timeout))
    }

    /// `Close`: leaves the multicast group, if it is one, and is no longer open.
    /// `// C#: ExtLibs/Comms/CommsUDPSerialConnect.cs:307-341`
    fn close(&mut self) {
        self.open = false;
        if self.multicast {
            let _ = match self.host.ip() {
                IpAddr::V4(group) => self
                    .socket
                    .leave_multicast_v4(&group, &Ipv4Addr::UNSPECIFIED),
                IpAddr::V6(group) => self.socket.leave_multicast_v6(&group, 0),
            };
        }
    }
}

/// `IsInRange("224.0.0.0", "239.255.255.255", address)`: whether the C# treats a host as a
/// multicast group.
///
/// Ported as written, which compares the address's *last* four bytes, reversed and read as a
/// signed little-endian `int`: so for IPv4 it is the multicast block, and for IPv6 it is whether
/// the address ends in 224.0.0.0-239.255.255.255 - `fe80::e000:1` is "in range" and `ff02::1`,
/// a real IPv6 group, is not (golden/udp.txt).
/// `// C#: ExtLibs/Comms/CommsUDPSerialConnect.cs:152-161`
#[must_use]
pub fn is_in_range(address: IpAddr) -> bool {
    // C#: BitConverter.ToInt32(GetAddressBytes().Reverse().ToArray(), 0)
    let as_int = |bytes: &[u8]| -> i64 {
        let mut reversed: Vec<u8> = bytes.iter().rev().copied().collect();
        reversed.resize(4, 0);
        let first: [u8; 4] = [
            reversed.first().copied().unwrap_or(0),
            reversed.get(1).copied().unwrap_or(0),
            reversed.get(2).copied().unwrap_or(0),
            reversed.get(3).copied().unwrap_or(0),
        ];
        i64::from(i32::from_le_bytes(first))
    };
    let start = as_int(&[224, 0, 0, 0]);
    let end = as_int(&[239, 255, 255, 255]);
    let ip = match address {
        IpAddr::V4(v4) => as_int(&v4.octets()),
        IpAddr::V6(v6) => as_int(&v6.octets()),
    };
    ip >= start && ip <= end
}
