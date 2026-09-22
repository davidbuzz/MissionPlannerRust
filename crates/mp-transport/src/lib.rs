//! Link transports for MAVLink connections.
//!
//! Replaces `ExtLibs/Comms` (`ICommsSerial`, `CommsSerialPort`, `CommsTCP`, `CommsUDP`, ...).
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

pub mod replay;
#[cfg(feature = "serial")]
pub mod serial;
pub mod socket;
pub mod testing;
pub mod url;

use std::io;
use std::time::Duration;

pub use replay::ReplayTransport;
#[cfg(feature = "serial")]
pub use serial::{SerialTransport, list_ports};
pub use socket::{TcpTransport, UdpTransport};
pub use url::{LinkUrl, UrlError};

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
    fn description(&self) -> String;

    /// Whether the link is still usable.
    fn is_open(&self) -> bool;

    /// Sets the read timeout. Implementations should apply it to subsequent reads.
    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()>;

    /// Closes the link. Idempotent.
    fn close(&mut self) {}
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
/// `udp:0.0.0.0:14550` or `file:flight.tlog`.
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
    }
}
