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

//! TCP and UDP transports.

use std::fmt::Write as _;
use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream, ToSocketAddrs, UdpSocket};
use std::time::Duration;

use crate::{DEFAULT_READ_TIMEOUT, OpenError, Transport};

/// A TCP link, either outbound or accepted from a listener.
#[derive(Debug)]
pub struct TcpTransport {
    stream: TcpStream,
    /// `tcp:<peer>`, kept ready for [`Transport::description`]. A TCP peer cannot change.
    description: String,
    open: bool,
}

/// How long a TCP connect may take before it is a failure.
pub const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);

impl TcpTransport {
    /// Connects to a remote endpoint.
    pub fn connect(host: &str, port: u16) -> Result<Self, OpenError> {
        let addr = (host, port)
            .to_socket_addrs()
            .map_err(|e| OpenError::io(format!("resolving {host}:{port}"), e))?
            .next()
            .ok_or_else(|| {
                OpenError::io(
                    format!("resolving {host}:{port}"),
                    io::Error::new(io::ErrorKind::NotFound, "no addresses"),
                )
            })?;
        Self::from_addr(addr)
    }

    fn from_addr(addr: SocketAddr) -> Result<Self, OpenError> {
        // Bounded where the C#'s `new TcpClient(host, port)` waits the operating system's time
        // (a minute or more for a host that drops the SYN): a link opening this again every
        // second while its peer is away (`mp_link`'s reconnection) must stay stoppable, and a
        // connection that is not up in five seconds is not coming up.
        let stream = TcpStream::connect_timeout(&addr, CONNECT_TIMEOUT)
            .map_err(|e| OpenError::io(format!("connecting to {addr}"), e))?;
        Self::wrap(stream)
    }

    /// Listens on a port and blocks until one peer connects.
    pub fn listen(port: u16) -> Result<Self, OpenError> {
        let listener = TcpListener::bind(("0.0.0.0", port))
            .map_err(|e| OpenError::io(format!("listening on port {port}"), e))?;
        let (stream, _) = listener
            .accept()
            .map_err(|e| OpenError::io(format!("accepting on port {port}"), e))?;
        Self::wrap(stream)
    }

    fn wrap(stream: TcpStream) -> Result<Self, OpenError> {
        // Telemetry is many small packets; Nagle would add tens of milliseconds of latency to
        // every command we send.
        stream
            .set_nodelay(true)
            .map_err(|e| OpenError::io("setting TCP_NODELAY", e))?;
        stream
            .set_read_timeout(Some(DEFAULT_READ_TIMEOUT))
            .map_err(|e| OpenError::io("setting read timeout", e))?;
        let description = stream
            .peer_addr()
            .map_or_else(|_| "tcp:unknown".to_owned(), |a| format!("tcp:{a}"));
        Ok(Self {
            stream,
            description,
            open: true,
        })
    }
}

impl Transport for TcpTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.stream.read(buf) {
            Ok(0) => {
                self.open = false; // orderly shutdown by the peer
                Ok(0)
            }
            Ok(n) => Ok(n),
            Err(e) if is_timeout(&e) => Ok(0),
            Err(e) => {
                self.open = false;
                Err(e)
            }
        }
    }

    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        self.stream.write_all(buf).inspect_err(|_| {
            self.open = false;
        })
    }

    fn description(&self) -> &str {
        &self.description
    }

    fn is_open(&self) -> bool {
        self.open
    }

    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        self.stream.set_read_timeout(Some(timeout))
    }

    fn close(&mut self) {
        let _ = self.stream.shutdown(std::net::Shutdown::Both);
        self.open = false;
    }
}

/// A UDP link that listens on a port and answers every endpoint it has heard from: Mission
/// Planner's `UdpSerial`.
///
/// This mirrors how ArduPilot and SITL are normally reached: the GCS binds a port and the vehicle
/// sends to it. The sender of each datagram read joins the endpoints, in the order first heard and
/// each once, and none is ever dropped; every write goes to each of them, so several sources on
/// one port - two SITLs, a vehicle and a GCS - all get the GCS's requests.
/// `// C#: ExtLibs/Comms/CommsUdpSerial.cs:20, 194-202, 264-276`
///
/// Binding returns at once and the first datagram is read like any other, which is what the C#'s
/// `UdpSerial(UdpClient)` does - AutoConnect's, open from the start, so its `Open` returns at once
/// (`CommsUdpSerial.cs:48-53, 102-106`; `ExtLibs/Utilities/AutoConnect.cs:397-403`). The C#'s own
/// `Open` instead waits for the first datagram, takes its sender as the first endpoint and drops
/// its bytes (`CommsUdpSerial.cs:132-165`); here the port comes in the link URL and the waiting is
/// the link's reads.
#[derive(Debug)]
pub struct UdpTransport {
    socket: UdpSocket,
    /// `EndPointList`: every endpoint heard from or added, in the order first heard, each once.
    /// Every write goes to each.
    endpoints: Vec<SocketAddr>,
    /// The endpoint heard from or added last, which the description names.
    peer: Option<SocketAddr>,
    local: String,
    /// `udp:<local> <-> <peer>`, or `udp:<local> (no peer yet)`, kept ready for
    /// [`Transport::description`] and rewritten in place when the peer changes.
    description: String,
    open: bool,
}

/// The most text a peer's address can print as: a scoped IPv6 address and a port. The
/// description's buffer is sized for it at bind, so hearing from a new peer - in `read`, on the
/// ingest path - never allocates, even with two vehicles taking turns on one port.
const LONGEST_PEER: usize =
    "[ffff:ffff:ffff:ffff:ffff:ffff:255.255.255.255%4294967295]:65535".len();

/// The endpoints room is made for at bind, so hearing a new one - in `read`, on the ingest path -
/// does not allocate until there are more than this: two SITLs, a vehicle and a GCS are four. The
/// C#'s list grows without limit, and so does this one past them.
const ENDPOINTS_RESERVED: usize = 8;

impl UdpTransport {
    /// Binds a local port.
    pub fn bind(bind: &str, port: u16) -> Result<Self, OpenError> {
        let socket = UdpSocket::bind((bind, port))
            .map_err(|e| OpenError::io(format!("binding {bind}:{port}"), e))?;
        socket
            .set_read_timeout(Some(DEFAULT_READ_TIMEOUT))
            .map_err(|e| OpenError::io("setting read timeout", e))?;
        let local = socket
            .local_addr()
            .map_or_else(|_| "unknown".to_owned(), |a| a.to_string());
        let description = String::with_capacity("udp: <-> ".len() + local.len() + LONGEST_PEER);
        let mut transport = Self {
            socket,
            endpoints: Vec::with_capacity(ENDPOINTS_RESERVED),
            peer: None,
            local,
            description,
            open: true,
        };
        transport.describe();
        Ok(transport)
    }

    /// Adds an endpoint to send to, for the case where the GCS must speak first: the C#'s
    /// `EndPointList` is public for it ("add to EndPointList if need when injecting"). One
    /// already there is not added again.
    /// `// C#: ExtLibs/Comms/CommsUdpSerial.cs:20, 25-28`
    pub fn add_endpoint(&mut self, endpoint: SocketAddr) {
        self.hear(endpoint);
    }

    /// A datagram came from `from`, or it was added: it joins the endpoints unless it is one of
    /// them already, and the description names it.
    /// `// C#: ExtLibs/Comms/CommsUdpSerial.cs:194-202`
    fn hear(&mut self, from: SocketAddr) {
        let known = self.endpoints.iter().any(|e| same_endpoint(e, &from));
        if !known {
            self.endpoints.push(from);
        }
        if self.peer != Some(from) {
            self.peer = Some(from);
            self.describe();
        }
    }

    /// Rewrites the description in the buffer sized at bind.
    fn describe(&mut self) {
        self.description.clear();
        // Writing to a `String` cannot fail.
        let _ = match self.peer {
            Some(peer) => write!(self.description, "udp:{} <-> {peer}", self.local),
            None => write!(self.description, "udp:{} (no peer yet)", self.local),
        };
    }

    /// `EndPointList`: every endpoint heard from or added, in the order first heard - the order
    /// each write goes to them in.
    #[must_use]
    pub fn endpoints(&self) -> &[SocketAddr] {
        &self.endpoints
    }

    /// The endpoint heard from or added last, if any: the one the description names.
    #[must_use]
    pub const fn peer(&self) -> Option<SocketAddr> {
        self.peer
    }
}

/// Whether two endpoints are one, as `List.Contains` asks `IPEndPoint.Equals`: the same address
/// family, address and port, and for IPv6 the same scope id. An `IPEndPoint` has no flow label,
/// so a `SocketAddrV6`'s is not compared, where its own `==` would compare it.
/// `// C#: ExtLibs/Comms/CommsUdpSerial.cs:201`
fn same_endpoint(a: &SocketAddr, b: &SocketAddr) -> bool {
    match (a, b) {
        (SocketAddr::V4(a), SocketAddr::V4(b)) => a == b,
        (SocketAddr::V6(a), SocketAddr::V6(b)) => {
            a.ip() == b.ip() && a.port() == b.port() && a.scope_id() == b.scope_id()
        }
        _ => false,
    }
}

impl Transport for UdpTransport {
    /// One datagram, whose sender is heard (`Read`'s `EndPointList.Add`).
    /// `// C#: ExtLibs/Comms/CommsUdpSerial.cs:192-203`
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.socket.recv_from(buf) {
            Ok((n, from)) => {
                self.hear(from);
                Ok(n)
            }
            Err(e) if is_timeout(&e) => Ok(0),
            Err(e) => Err(e),
        }
    }

    /// `Write`: one datagram to each endpoint, in the order they were first heard; a failed send
    /// is swallowed and the next endpoint still sent to, as the C# swallows it. Before anyone has
    /// been heard there is nowhere to send, and nothing is sent or kept for later.
    /// `// C#: ExtLibs/Comms/CommsUdpSerial.cs:264-276`
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        for endpoint in &self.endpoints {
            let _ = self.socket.send_to(buf, *endpoint);
        }
        Ok(())
    }

    fn description(&self) -> &str {
        &self.description
    }

    fn is_open(&self) -> bool {
        self.open
    }

    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        self.socket.set_read_timeout(Some(timeout))
    }

    fn close(&mut self) {
        self.open = false;
    }
}

/// Whether an error is a read timeout rather than a real failure.
///
/// Platforms disagree here: Unix reports `WouldBlock`, Windows reports `TimedOut`. Treating one
/// as fatal is a classic "works on my machine" transport bug.
pub(crate) fn is_timeout(e: &io::Error) -> bool {
    matches!(
        e.kind(),
        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
    )
}

#[cfg(test)]
mod tests {
    use std::net::{Ipv4Addr, Ipv6Addr, SocketAddrV6};

    use super::*;

    /// How long a test waits for a datagram before it fails rather than hangs.
    const GUARD: Duration = Duration::from_secs(10);

    /// The GCS: a UDP link listening on 127.0.0.1, and the address it listens on.
    fn listener() -> (UdpTransport, SocketAddr) {
        let mut gcs = UdpTransport::bind("127.0.0.1", 0).unwrap();
        gcs.set_read_timeout(GUARD).unwrap();
        let address = gcs.socket.local_addr().unwrap();
        (gcs, address)
    }

    /// A source sending to it: a vehicle, a SITL, another GCS.
    fn source() -> UdpSocket {
        let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
        socket.set_read_timeout(Some(GUARD)).unwrap();
        socket
    }

    /// `from` sends `datagram` to the GCS at `address`, and the GCS reads it.
    fn heard(gcs: &mut UdpTransport, address: SocketAddr, from: &UdpSocket, datagram: &[u8]) {
        from.send_to(datagram, address).unwrap();
        let mut buf = [0u8; 64];
        let n = gcs.read(&mut buf).unwrap();
        assert_eq!(&buf[..n], datagram, "the GCS did not hear {datagram:?}");
    }

    /// The next datagram a source receives.
    fn next(at: &UdpSocket) -> Vec<u8> {
        let mut buf = [0u8; 64];
        let (n, _) = at.recv_from(&mut buf).unwrap();
        buf[..n].to_vec()
    }

    /// Two sources on one port - two SITLs, a vehicle and a GCS - both get every write, and a
    /// write before anyone was heard went nowhere and is not sent later.
    #[test]
    fn a_write_reaches_every_endpoint_heard() {
        let (mut gcs, address) = listener();
        gcs.write_all(b"to nobody").unwrap();
        let (first, second) = (source(), source());
        heard(&mut gcs, address, &first, b"heartbeat one");
        heard(&mut gcs, address, &second, b"heartbeat two");

        gcs.write_all(b"request").unwrap();
        assert_eq!(next(&first), b"request");
        assert_eq!(next(&second), b"request");
    }

    /// `EndPointList` is a list: endpoints in the order first heard, one heard again not added
    /// again - so it gets each write once - and none dropped for being quiet.
    #[test]
    fn endpoints_are_kept_in_the_order_first_heard_and_each_once() {
        let (mut gcs, address) = listener();
        let (first, second) = (source(), source());
        heard(&mut gcs, address, &first, b"a");
        heard(&mut gcs, address, &second, b"b");
        heard(&mut gcs, address, &first, b"c");
        heard(&mut gcs, address, &first, b"d");

        let first_at = first.local_addr().unwrap();
        let second_at = second.local_addr().unwrap();
        assert_eq!(gcs.endpoints(), [first_at, second_at]);
        // The description names the one heard last.
        assert_eq!(gcs.peer(), Some(first_at));

        // A source listed twice would get "one" twice before "two".
        gcs.write_all(b"one").unwrap();
        gcs.write_all(b"two").unwrap();
        for each in [&first, &second] {
            assert_eq!(next(each), b"one");
            assert_eq!(next(each), b"two");
        }
    }

    /// One endpoint whose send fails does not stop the write reaching the rest: the C# catches
    /// each send's failure on its own. An IPv6 address cannot be sent to from an IPv4 socket.
    #[test]
    fn an_endpoint_that_cannot_be_sent_to_does_not_stop_the_others() {
        let (mut gcs, address) = listener();
        let unreachable = SocketAddr::from((Ipv6Addr::LOCALHOST, 9));
        gcs.add_endpoint(unreachable);
        gcs.add_endpoint(unreachable);
        let vehicle = source();
        heard(&mut gcs, address, &vehicle, b"heartbeat");
        let vehicle_at = vehicle.local_addr().unwrap();
        assert_eq!(gcs.endpoints(), [unreachable, vehicle_at]);

        gcs.write_all(b"request").unwrap();
        assert_eq!(next(&vehicle), b"request");
    }

    /// Endpoints compared as `IPEndPoint.Equals` compares them: family, address and port, and an
    /// IPv6 address's scope, never its flow label.
    #[test]
    fn an_endpoint_is_compared_as_the_csharp_compares_it() {
        let v4 = SocketAddr::from(([127, 0, 0, 1], 14550));
        let other_port = SocketAddr::from(([127, 0, 0, 1], 14551));
        let other_address = SocketAddr::from(([127, 0, 0, 2], 14550));
        let mapped = SocketAddr::from((Ipv4Addr::LOCALHOST.to_ipv6_mapped(), 14550));
        assert!(same_endpoint(&v4, &v4));
        assert!(!same_endpoint(&v4, &other_port));
        assert!(!same_endpoint(&v4, &other_address));
        assert!(!same_endpoint(&v4, &mapped), "another address family");

        let plain = SocketAddr::V6(SocketAddrV6::new(Ipv6Addr::LOCALHOST, 14550, 0, 0));
        let labelled = SocketAddr::V6(SocketAddrV6::new(Ipv6Addr::LOCALHOST, 14550, 7, 0));
        let scoped = SocketAddr::V6(SocketAddrV6::new(Ipv6Addr::LOCALHOST, 14550, 0, 3));
        assert!(same_endpoint(&plain, &labelled));
        assert!(!same_endpoint(&plain, &scoped));

        // So a list holding `plain` takes `scoped` and not `labelled`.
        let (mut gcs, _) = listener();
        gcs.add_endpoint(plain);
        gcs.add_endpoint(labelled);
        gcs.add_endpoint(scoped);
        assert_eq!(gcs.endpoints(), [plain, scoped]);
    }
}
