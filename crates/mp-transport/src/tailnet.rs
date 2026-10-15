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

//! The links' way onto the tailnet through the planner's built-in Tailscale node (mp-tailnet) -
//! the owner's feature of 2026-10-14, not in the C#.
//!
//! While the planner has a node in use ([`use_node`]: the PLUGINS tab's "Tailscale node
//! (experimental)" row ticked), a `tcp:` or `udpcl:` link to a host on the tailnet is dialled
//! through the node rather than through the operating system: an address Tailscale assigns
//! (100.64.0.0/10, fd7a:115c:a1e0::/48), a name in `ts.net`, or - once the node runs - a name its
//! MagicDNS table holds (`white`, `buzz55`). Every other host goes through the operating system as
//! before, and every host does while no node is in use or the one in use has failed or stopped
//! ([`route`]), so a failed node never stands between a link and the computer's own Tailscale.
//!
//! Only those two kinds of link, opened by their URL ([`crate::open_url`], and so the link's
//! reconnections too): `udp:` and `tcpin:` listen rather than dial, and a screen that opens a
//! transport itself (`TcpTransport::connect`, `UdpClientTransport::open`) still goes through the
//! operating system.

use std::io::{self, Read as _, Write as _};
use std::net::IpAddr;
use std::sync::{Mutex, PoisonError};
use std::time::Duration;

use mp_os::Lock as _;
use mp_tailnet::{Node, State};

use crate::socket::{CONNECT_TIMEOUT, is_timeout};
use crate::{DEFAULT_READ_TIMEOUT, OpenError, Transport};

/// How long a link waits for the node's name table to say whether it holds a name: the node
/// answers from memory once it runs, so this bounds only a node that has just started.
const LOOKUP: Duration = Duration::from_secs(1);

/// The node links dial through, while the planner has one in use.
static NODE: Mutex<Option<Node>> = Mutex::new(None);

/// Links dial through `node` from now on - the tailnet's hosts, as [`route`] tells them - or, with
/// `None`, every host through the operating system again. Links already open stay as they are.
pub fn use_node(node: Option<Node>) {
    *NODE.os_lock().unwrap_or_else(PoisonError::into_inner) = node;
}

/// The node in use, if any.
fn in_use() -> Option<Node> {
    NODE.os_lock().unwrap_or_else(PoisonError::into_inner).clone()
}

/// Which way a link to a host goes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Route {
    /// Through the operating system, as every link went before the node.
    Os,
    /// Through the planner's built-in node.
    Tailnet,
}

/// The way a link to `host` goes while the node in use stands as `state`.
///
/// Through the operating system unless the node is in use ([`State::in_use`]: starting, waiting
/// for its sign-in, or running). Then an IP address goes through the node if Tailscale assigns it
/// ([`mp_tailnet::is_tailscale_ip`]), a name if it is in `ts.net` ([`mp_tailnet::is_ts_net_name`])
/// or if the running node's MagicDNS table holds it - `known` asks the table, and only of a
/// running node. A node not yet running takes the tailnet's addresses and `ts.net` names, and
/// the link waits for it as long as a connection may take; other names wait for a node that runs.
#[must_use]
pub fn route(host: &str, state: &State, known: impl FnOnce(&str) -> bool) -> Route {
    if !state.in_use() {
        return Route::Os;
    }
    if let Ok(ip) = unbracketed(host).parse::<IpAddr>() {
        return if mp_tailnet::is_tailscale_ip(ip) {
            Route::Tailnet
        } else {
            Route::Os
        };
    }
    if mp_tailnet::is_ts_net_name(host) || (state.is_running() && known(host)) {
        Route::Tailnet
    } else {
        Route::Os
    }
}

/// A host without the brackets an IPv6 address is written in beside a port: `[fd7a::1]`.
fn unbracketed(host: &str) -> &str {
    host.strip_prefix('[')
        .and_then(|inner| inner.strip_suffix(']'))
        .unwrap_or(host)
}

/// A `tcp:` link through the node in use when [`route`] says its host is the tailnet's; `None`
/// for the operating system to dial.
pub(crate) fn open_tcp(host: &str, port: u16) -> Option<Result<TailnetTcpTransport, OpenError>> {
    let node = in_use()?;
    let host = unbracketed(host);
    match route(host, &node.state(), |name| node.resolve(name, LOOKUP).is_ok()) {
        Route::Os => None,
        Route::Tailnet => Some(TailnetTcpTransport::connect(&node, host, port)),
    }
}

/// A `udpcl:` link through the node in use when [`route`] says its host is the tailnet's; `None`
/// for the operating system to open.
pub(crate) fn open_udp_client(
    host: &str,
    port: u16,
) -> Option<Result<TailnetUdpClientTransport, OpenError>> {
    let node = in_use()?;
    let host = unbracketed(host);
    match route(host, &node.state(), |name| node.resolve(name, LOOKUP).is_ok()) {
        Route::Os => None,
        Route::Tailnet => Some(TailnetUdpClientTransport::open(&node, host, port)),
    }
}

/// A `tcp:` link over the tailnet: [`crate::TcpTransport`]'s behaviour on the node's connection.
#[derive(Debug)]
pub struct TailnetTcpTransport {
    stream: mp_tailnet::TcpStream,
    /// `tcp:<peer> (tailnet)`, kept ready for [`Transport::description`].
    description: String,
    open: bool,
}

impl TailnetTcpTransport {
    /// Connects to `host`'s `port` through `node`, taking no longer than a link's connection
    /// through the operating system may ([`CONNECT_TIMEOUT`]).
    fn connect(node: &Node, host: &str, port: u16) -> Result<Self, OpenError> {
        let mut stream = node.connect_tcp(host, port, CONNECT_TIMEOUT).map_err(|err| {
            OpenError::io(format!("connecting to {host}:{port} over the tailnet"), err.into())
        })?;
        stream.set_read_timeout(Some(DEFAULT_READ_TIMEOUT));
        let description = format!("tcp:{} (tailnet)", stream.peer_addr());
        Ok(Self {
            stream,
            description,
            open: true,
        })
    }
}

impl Transport for TailnetTcpTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.stream.read(buf) {
            Ok(0) => {
                self.open = false; // the far end closed its side
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
        self.stream.set_read_timeout(Some(timeout));
        Ok(())
    }

    fn close(&mut self) {
        self.stream.shutdown();
        self.open = false;
    }
}

/// A `udpcl:` link over the tailnet: [`crate::UdpClientTransport`]'s behaviour on a socket of the
/// node's - every write to the host and never failing, reads from anyone, a datagram longer than a
/// read kept for the next - save that it closes when the node stops under it, so that the link
/// opens it again (through a node, or the operating system).
#[derive(Debug)]
pub struct TailnetUdpClientTransport {
    socket: mp_tailnet::UdpSocket,
    /// What has been received and not yet read, from `cursor` on.
    received: Vec<u8>,
    cursor: usize,
    open: bool,
    /// `udpcl:<host> (tailnet)`, kept ready for [`Transport::description`].
    description: String,
}

impl TailnetUdpClientTransport {
    /// A socket of `node`'s to send to `host`'s `port` from, taking no longer than a link's
    /// connection may ([`CONNECT_TIMEOUT`]).
    fn open(node: &Node, host: &str, port: u16) -> Result<Self, OpenError> {
        let mut socket = node.bind_udp(host, port, CONNECT_TIMEOUT).map_err(|err| {
            OpenError::io(format!("opening a tailnet socket to {host}:{port}"), err.into())
        })?;
        socket.set_read_timeout(Some(DEFAULT_READ_TIMEOUT));
        let description = format!("udpcl:{} (tailnet)", socket.remote());
        Ok(Self {
            socket,
            received: Vec::new(),
            cursor: 0,
            open: true,
            description,
        })
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

impl Transport for TailnetUdpClientTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        if self.cursor == self.received.len() && self.open {
            // All read: the buffer starts again with the next datagram.
            self.received.clear();
            self.cursor = 0;
            match self.socket.recv_into(&mut self.received) {
                Ok(_from) => {}
                Err(e) if is_timeout(&e) => return Ok(0),
                Err(e) => {
                    if self.socket.is_ended() {
                        self.open = false;
                    }
                    return Err(e);
                }
            }
        }
        Ok(self.take(buf))
    }

    /// One datagram to the host; a failure swallowed, as the operating system's UDP client
    /// swallows it.
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        let _ = self.socket.send(buf);
        Ok(())
    }

    fn description(&self) -> &str {
        &self.description
    }

    fn is_open(&self) -> bool {
        self.open
    }

    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        self.socket.set_read_timeout(Some(timeout));
        Ok(())
    }

    fn close(&mut self) {
        self.open = false;
    }
}

#[cfg(test)]
mod tests {
    use std::net::{Ipv4Addr, Ipv6Addr};

    use super::*;

    fn running() -> State {
        State::Running {
            name: "mpr-host.tail1234.ts.net".to_owned(),
            ipv4: Ipv4Addr::new(100, 101, 102, 103),
            ipv6: Ipv6Addr::new(0xfd7a, 0x115c, 0xa1e0, 0, 0, 0, 0, 1),
        }
    }

    /// The node's table, for the tests: `white` and `buzz55`, by short and full name.
    fn table(name: &str) -> bool {
        matches!(
            name,
            "white" | "buzz55" | "white.tail1234.ts.net" | "buzz55.tail1234.ts.net"
        )
    }

    /// A running node takes the tailnet's addresses - IPv4, IPv6, bracketed, mapped - its
    /// `ts.net` names in any case, and the names its table holds; everything else, the
    /// operating system's.
    #[test]
    fn a_running_node_takes_the_tailnets_hosts() {
        let state = running();
        for tailnet in [
            "100.64.0.1",
            "100.101.102.103",
            "fd7a:115c:a1e0::1",
            "[fd7a:115c:a1e0::1]",
            "::ffff:100.64.0.1",
            "white.tail1234.ts.net",
            "WHITE.Tail1234.TS.NET.",
            "other.tail9999.ts.net",
            "white",
            "buzz55",
        ] {
            assert_eq!(route(tailnet, &state, table), Route::Tailnet, "{tailnet}");
        }
        for elsewhere in [
            "127.0.0.1",
            "192.168.4.1",
            "10.0.2.2",
            "100.115.92.7",
            "100.128.0.1",
            "::1",
            "[::1]",
            "localhost",
            "sitl.example.com",
            "grey",
        ] {
            assert_eq!(route(elsewhere, &state, table), Route::Os, "{elsewhere}");
        }
    }

    /// An address the node's table holds is decided by the address, not the table: the table is
    /// asked of names only.
    #[test]
    fn an_address_is_never_looked_up() {
        let mut asked = Vec::new();
        let way = route("192.168.4.1", &running(), |name| {
            asked.push(name.to_owned());
            true
        });
        assert_eq!(way, Route::Os);
        assert!(asked.is_empty(), "{asked:?}");
    }

    /// A node starting or waiting for its sign-in takes the tailnet's addresses and `ts.net`
    /// names - the link waits for it - but its table is not asked, so other names go through
    /// the operating system.
    #[test]
    fn a_node_not_yet_running_takes_addresses_and_ts_net_names_only() {
        for state in [
            State::Starting,
            State::NeedsSignIn {
                url: "https://login.tailscale.com/a/abc".to_owned(),
            },
        ] {
            let mut asked = false;
            let way = route("white", &state, |_| {
                asked = true;
                true
            });
            assert_eq!(way, Route::Os);
            assert!(!asked, "the table of a node not running is not asked");
            assert_eq!(route("100.64.0.1", &state, table), Route::Tailnet);
            assert_eq!(route("white.tail1234.ts.net", &state, table), Route::Tailnet);
            assert_eq!(route("192.168.4.1", &state, table), Route::Os);
        }
    }

    /// A stopped or failed node takes nothing: every link goes through the operating system, where
    /// the computer's own Tailscale may carry it.
    #[test]
    fn a_stopped_or_failed_node_takes_nothing() {
        for state in [
            State::Stopped,
            State::Failed {
                why: mp_tailnet::NO_NODE.to_owned(),
            },
        ] {
            for host in ["100.64.0.1", "fd7a:115c:a1e0::1", "white.tail1234.ts.net", "white"] {
                assert_eq!(route(host, &state, table), Route::Os, "{host} with {state}");
            }
        }
    }

    /// With no node in use - the planner's state until its row is ticked - every link is the
    /// operating system's.
    #[test]
    fn with_no_node_in_use_the_operating_system_dials() {
        assert!(open_tcp("100.64.0.1", 5760).is_none());
        assert!(open_tcp("white", 5760).is_none());
        assert!(open_udp_client("100.64.0.1", 14550).is_none());
        assert_eq!(unbracketed("[fd7a::1]"), "fd7a::1");
        assert_eq!(unbracketed("white"), "white");
        assert_eq!(unbracketed("[white"), "[white");
    }
}
