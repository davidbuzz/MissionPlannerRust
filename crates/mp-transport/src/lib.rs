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

//! Link transports for MAVLink connections.
//!
//! Replaces `ExtLibs/Comms` (`ICommsSerial`, `CommsSerialPort`, `CommsTCP`, `CommsUDP`,
//! `CommsUDPSerialConnect`, `CommsWebSocket`, `CommsNTRIP`, ...).
//!
//! # Why blocking, not async
//!
//! Every transport here is blocking with an explicit timeout, driven by a dedicated I/O thread
//! owned by the link engine. For a ground control station this beats an async runtime on the
//! metric that matters: a blocking `read` with a timeout on a dedicated thread has lower and far
//! more predictable latency than a task competing for an executor, and telemetry is a small
//! number of long-lived links rather than thousands of short connections. It also keeps this
//! crate free of any runtime dependency, so the UI layer can pick its own without a rewrite here.
//!
//! # The replay transport is not a toy
//!
//! [`ReplayTransport`] makes a recorded flight a first-class input. Tests, benchmarks and the
//! differential harness all drive the real code path with real data instead of synthetic frames.

mod codec;
mod dotnet;
pub mod enumerate;
pub mod ntrip;
pub mod replay;
#[cfg(feature = "serial")]
pub mod serial;
#[cfg(all(windows, feature = "serial"))]
mod win32;
pub mod socket;
pub mod testing;
pub mod udp_client;
pub mod url;
pub mod websocket;

use std::io;
use std::time::Duration;

pub use enumerate::PortInfo;
pub use ntrip::{NtripOptions, NtripTransport};
pub use replay::ReplayTransport;
#[cfg(feature = "serial")]
pub use serial::{SerialTransport, list_ports};
pub use socket::{TcpTransport, UdpTransport};
pub use udp_client::UdpClientTransport;
pub use url::{LinkUrl, UrlError};
pub use websocket::WebSocketTransport;

/// A bidirectional byte link.
///
/// Implementations must not block indefinitely: [`Transport::read`] returns `Ok(0)` when the
/// timeout expires with no data, which is a normal idle tick rather than end-of-stream. End of
/// stream is reported by [`Transport::is_open`] going false.
pub trait Transport: Send {
    /// Reads available bytes, returning `Ok(0)` if the timeout elapsed with nothing to read.
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize>;

    /// Writes the whole buffer.
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()>;

    /// Human-readable description, shown in the UI and in logs.
    ///
    /// Borrowed, so asking costs nothing: the link asks on every snapshot publish, to notice a UDP
    /// link learning its peer, and that path must not allocate (DELIVERABLES.md Deliverable 5). A transport
    /// keeps its text ready and rewrites it only when what it describes changes.
    fn description(&self) -> &str;

    /// Whether the link is still usable.
    fn is_open(&self) -> bool;

    /// Sets the read timeout. Implementations should apply it to subsequent reads.
    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()>;

    /// Closes the link. Idempotent.
    fn close(&mut self) {}

    /// When the bytes the last [`Transport::read`] returned were sent: [`ReadTime::Live`] - just
    /// now - for a link to a vehicle, and a recording's own clock for a replay. The C# stamps each
    /// packet's `CurrentState.datetime` with one or the other (`MAVLinkInterface.cs:4721, 6649`),
    /// and a link that holds only a `Box<dyn Transport>` learns which from here.
    fn read_time(&self) -> ReadTime {
        ReadTime::Live
    }
}

/// When a transport's bytes were sent, as [`Transport::read_time`] reports it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadTime {
    /// Just now: a live link, whose packets are stamped with the time they are read.
    Live,
    /// A recording's clock: the newest usable timestamp read from it so far, microseconds since
    /// the Unix epoch - `MAVLinkInterface.lastlogread` - or `None` before there is one, where the
    /// C#'s `lastlogread` is still `DateTime.MinValue`. A record whose timestamp is not usable
    /// leaves it where it was, as `readlogPacketMavlink` does.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:482, 6539-6558, 6649`
    Recorded(Option<u64>),
}

/// Default read timeout: short enough that a link teardown is responsive, long enough that an
/// idle link does not spin the I/O thread.
pub const DEFAULT_READ_TIMEOUT: Duration = Duration::from_millis(100);

/// Errors from opening a link.
#[derive(Debug, thiserror::Error)]
pub enum OpenError {
    /// The link URL could not be parsed.
    #[error("invalid link url: {0}")]
    Url(#[from] UrlError),
    /// The underlying I/O operation failed.
    #[error("{context}: {source}")]
    Io {
        /// What was being attempted.
        context: String,
        /// The underlying error.
        #[source]
        source: io::Error,
    },
    /// The build lacks support for this transport.
    #[error("{0} support is not compiled into this build")]
    Unsupported(&'static str),
}

impl OpenError {
    /// Attaches context to an I/O error.
    pub fn io(context: impl Into<String>, source: io::Error) -> Self {
        Self::Io {
            context: context.into(),
            source,
        }
    }
}

/// Opens a transport from a link URL such as `serial:/dev/ttyACM0:115200`, `tcp:host:5760`,
/// `udp:0.0.0.0:14550`, `udpcl:192.168.4.1:14550`, `ws://host:8080/path`,
/// `ntrip://user:pass@caster:2101/MOUNT` or `file:flight.tlog`.
pub fn open(url: &str) -> Result<Box<dyn Transport>, OpenError> {
    let parsed: LinkUrl = url.parse()?;
    open_url(&parsed)
}

/// Opens a transport from an already-parsed link URL.
pub fn open_url(url: &LinkUrl) -> Result<Box<dyn Transport>, OpenError> {
    match url {
        LinkUrl::Serial { path, baud } => {
            #[cfg(feature = "serial")]
            {
                Ok(Box::new(SerialTransport::open(path, *baud)?))
            }
            #[cfg(not(feature = "serial"))]
            {
                let _ = (path, baud);
                Err(OpenError::Unsupported("serial"))
            }
        }
        LinkUrl::Tcp { host, port } => Ok(Box::new(TcpTransport::connect(host, *port)?)),
        LinkUrl::TcpListen { port } => Ok(Box::new(TcpTransport::listen(*port)?)),
        LinkUrl::Udp { bind, port } => Ok(Box::new(UdpTransport::bind(bind, *port)?)),
        LinkUrl::File { path } => Ok(Box::new(ReplayTransport::open(path)?)),
        LinkUrl::UdpClient { host, port } => Ok(Box::new(UdpClientTransport::open(host, *port)?)),
        LinkUrl::WebSocket { url } => Ok(Box::new(WebSocketTransport::open(url)?)),
        // No position yet, so no GGA until one is set: what the RTK page does with "send GGA"
        // unticked.
        LinkUrl::Ntrip { url } => Ok(Box::new(NtripTransport::open(
            url,
            NtripOptions::default(),
        )?)),
    }
}
