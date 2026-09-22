//! TCP and UDP transports.

use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream, ToSocketAddrs, UdpSocket};
use std::time::Duration;

use crate::{DEFAULT_READ_TIMEOUT, OpenError, Transport};

/// A TCP link, either outbound or accepted from a listener.
#[derive(Debug)]
pub struct TcpTransport {
    stream: TcpStream,
    peer: String,
    open: bool,
}

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
        let stream = TcpStream::connect(addr)
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
        let peer = stream
            .peer_addr()
            .map_or_else(|_| "unknown".to_owned(), |a| a.to_string());
        Ok(Self {
            stream,
            peer,
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

    fn description(&self) -> String {
        format!("tcp:{}", self.peer)
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

/// A UDP link that learns its peer from the first datagram received.
///
/// This mirrors how ArduPilot and SITL are normally reached: the GCS binds a port and the vehicle
/// sends to it, after which replies go back to whoever was heard from last.
#[derive(Debug)]
pub struct UdpTransport {
    socket: UdpSocket,
    peer: Option<SocketAddr>,
    local: String,
    open: bool,
}

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
        Ok(Self {
            socket,
            peer: None,
            local,
            open: true,
        })
    }

    /// Sets the peer explicitly, for the case where we must speak first.
    pub fn set_peer(&mut self, peer: SocketAddr) {
        self.peer = Some(peer);
    }

    /// The peer learned so far, if any.
    #[must_use]
    pub const fn peer(&self) -> Option<SocketAddr> {
        self.peer
    }
}

impl Transport for UdpTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.socket.recv_from(buf) {
            Ok((n, from)) => {
                self.peer = Some(from);
                Ok(n)
            }
            Err(e) if is_timeout(&e) => Ok(0),
            Err(e) => Err(e),
        }
    }

    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        let Some(peer) = self.peer else {
            // Nothing heard yet and no explicit peer: dropping is correct, erroring would make
            // every startup log noisy.
            return Ok(());
        };
        self.socket.send_to(buf, peer).map(|_| ())
    }

    fn description(&self) -> String {
        match self.peer {
            Some(peer) => format!("udp:{} <-> {peer}", self.local),
            None => format!("udp:{} (no peer yet)", self.local),
        }
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
fn is_timeout(e: &io::Error) -> bool {
    matches!(
        e.kind(),
        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
    )
}
