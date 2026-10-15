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

//! The built-in Tailscale node in a build without it - the `node` feature off (the owner's
//! feature of 2026-10-14, not in the C#): the same calls as the node build's, each saying that
//! this build has none ([`NO_NODE`]), so a planner built without tailscale-rs says why its node
//! does not start rather than nothing. Its sockets cannot be made.

use std::convert::Infallible;
use std::fmt;
use std::io;
use std::net::{IpAddr, SocketAddr};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use crate::{Error, NO_NODE, Settings, State};

/// The built-in Tailscale node of a build that has none: failed from the start with [`NO_NODE`],
/// stopped once [`Node::stop`] says so. A handle, cloned freely, as the node build's is.
#[derive(Debug, Clone, Default)]
pub struct Node {
    stopped: Arc<AtomicBool>,
}

impl Node {
    /// A node that failed as it started: this build has none.
    #[must_use]
    pub fn start(_settings: Settings) -> Self {
        Self::default()
    }

    /// Failed with [`NO_NODE`]; stopped once stopped.
    #[must_use]
    pub fn state(&self) -> State {
        if self.stopped.load(Ordering::Relaxed) {
            State::Stopped
        } else {
            State::Failed {
                why: NO_NODE.to_owned(),
            }
        }
    }

    /// Never.
    #[must_use]
    pub fn is_running(&self) -> bool {
        false
    }

    /// Stopped from now on.
    pub fn stop(&self) {
        self.stopped.store(true, Ordering::Relaxed);
    }

    /// An IP address is itself, as in the node build; a name is not looked up: [`Error::NoNode`].
    ///
    /// # Errors
    /// [`Error::NoNode`] for anything but an IP address.
    pub fn resolve(&self, host: &str, _timeout: Duration) -> Result<IpAddr, Error> {
        host.parse().map_err(|_| Error::NoNode)
    }

    /// No connection: [`Error::NoNode`].
    ///
    /// # Errors
    /// Always [`Error::NoNode`].
    pub fn connect_tcp(
        &self,
        _host: &str,
        _port: u16,
        _timeout: Duration,
    ) -> Result<TcpStream, Error> {
        Err(Error::NoNode)
    }

    /// No socket: [`Error::NoNode`].
    ///
    /// # Errors
    /// Always [`Error::NoNode`].
    pub fn bind_udp(
        &self,
        _host: &str,
        _port: u16,
        _timeout: Duration,
    ) -> Result<UdpSocket, Error> {
        Err(Error::NoNode)
    }
}

/// A TCP connection over the tailnet, which a build without the node never makes.
pub struct TcpStream {
    never: Infallible,
}

impl TcpStream {
    /// The local end.
    #[must_use]
    pub fn local_addr(&self) -> SocketAddr {
        match self.never {}
    }

    /// The far end.
    #[must_use]
    pub fn peer_addr(&self) -> SocketAddr {
        match self.never {}
    }

    /// How long a read waits.
    pub fn set_read_timeout(&mut self, _timeout: Option<Duration>) {
        match self.never {}
    }

    /// Closes it.
    pub fn shutdown(&mut self) {
        match self.never {}
    }
}

impl io::Read for TcpStream {
    fn read(&mut self, _buf: &mut [u8]) -> io::Result<usize> {
        match self.never {}
    }
}

impl io::Write for TcpStream {
    fn write(&mut self, _buf: &[u8]) -> io::Result<usize> {
        match self.never {}
    }

    fn flush(&mut self) -> io::Result<()> {
        match self.never {}
    }
}

impl fmt::Debug for TcpStream {
    fn fmt(&self, _f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.never {}
    }
}

/// A UDP socket on the tailnet, which a build without the node never makes.
pub struct UdpSocket {
    never: Infallible,
}

impl UdpSocket {
    /// The local end.
    #[must_use]
    pub fn local_addr(&self) -> SocketAddr {
        match self.never {}
    }

    /// Where [`UdpSocket::send`] sends.
    #[must_use]
    pub fn remote(&self) -> SocketAddr {
        match self.never {}
    }

    /// How long a read waits.
    pub fn set_read_timeout(&mut self, _timeout: Option<Duration>) {
        match self.never {}
    }

    /// One datagram to the remote end.
    ///
    /// # Errors
    /// None is ever sent.
    pub fn send(&self, _data: &[u8]) -> io::Result<()> {
        match self.never {}
    }

    /// One datagram from anyone, into `into`, and who sent it.
    ///
    /// # Errors
    /// None is ever received.
    pub fn recv_into(&mut self, _into: &mut Vec<u8>) -> io::Result<SocketAddr> {
        match self.never {}
    }

    /// Whether nothing more can arrive.
    #[must_use]
    pub fn is_ended(&self) -> bool {
        match self.never {}
    }
}

impl fmt::Debug for UdpSocket {
    fn fmt(&self, _f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.never {}
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;

    /// Without the node, it fails as it starts, saying so; stops when told; an address resolves
    /// to itself, and anything that needs the tailnet says this build has none.
    #[test]
    fn a_build_without_the_node_says_so() {
        let node = Node::start(Settings::new(PathBuf::from("keys")));
        assert_eq!(node.state(), State::Failed { why: NO_NODE.to_owned() });
        assert!(!node.is_running());
        assert!(!node.state().in_use());
        let timeout = Duration::from_millis(1);
        assert_eq!(
            node.resolve("100.101.102.103", timeout),
            Ok(IpAddr::from([100, 101, 102, 103]))
        );
        assert_eq!(node.resolve("white", timeout), Err(Error::NoNode));
        assert!(matches!(node.connect_tcp("white", 5760, timeout), Err(Error::NoNode)));
        assert!(matches!(node.bind_udp("100.64.0.1", 14550, timeout), Err(Error::NoNode)));
        let handle = node.clone();
        node.stop();
        assert_eq!(handle.state(), State::Stopped);
    }
}
