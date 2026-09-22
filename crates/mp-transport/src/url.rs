//! Link URL parsing.
//!
//! Mission Planner's connection UI is a combo box of COM ports plus ad-hoc TCP/UDP dialogs. A
//! single textual form is easier to script, log, test and put in a config file, and it is what
//! MAVProxy users already expect.

use std::fmt;
use std::str::FromStr;

/// A parsed link target.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LinkUrl {
    /// `serial:/dev/ttyACM0:115200` or `serial:COM3` (default baud).
    Serial {
        /// Device path or Windows port name.
        path: String,
        /// Baud rate.
        baud: u32,
    },
    /// `tcp:host:5760` - outbound connection.
    Tcp {
        /// Remote host.
        host: String,
        /// Remote port.
        port: u16,
    },
    /// `tcpin:5760` - listen for an inbound connection.
    TcpListen {
        /// Local port.
        port: u16,
    },
    /// `udp:0.0.0.0:14550` - bind and learn the peer from the first packet.
    Udp {
        /// Local bind address.
        bind: String,
        /// Local port.
        port: u16,
    },
    /// `file:flight.tlog` - replay a recorded log.
    File {
        /// Path to the log.
        path: String,
    },
}

/// Default serial baud rate: what ArduPilot uses on USB and what Mission Planner defaults to.
pub const DEFAULT_BAUD: u32 = 115_200;
/// Default MAVLink UDP port.
pub const DEFAULT_UDP_PORT: u16 = 14550;
/// Default MAVLink TCP port (ArduPilot SITL).
pub const DEFAULT_TCP_PORT: u16 = 5760;

/// Why a link URL is invalid.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum UrlError {
    /// No `scheme:` prefix.
    #[error("missing scheme in {0:?}; expected serial:, tcp:, tcpin:, udp: or file:")]
    MissingScheme(String),
    /// Scheme is not one we support.
    #[error("unknown scheme {0:?}")]
    UnknownScheme(String),
    /// A required part is absent.
    #[error("{0}")]
    Malformed(String),
    /// A numeric part did not parse.
    #[error("invalid number in {0:?}")]
    BadNumber(String),
}

impl FromStr for LinkUrl {
    type Err = UrlError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.trim();
        let (scheme, rest) = s
            .split_once(':')
            .ok_or_else(|| UrlError::MissingScheme(s.to_owned()))?;

        match scheme.to_ascii_lowercase().as_str() {
            "serial" | "com" => {
                if rest.is_empty() {
                    return Err(UrlError::Malformed(
                        "serial: needs a device path".to_owned(),
                    ));
                }
                // Windows paths contain no colon; a trailing `:baud` is optional everywhere.
                match rest.rsplit_once(':') {
                    Some((path, baud))
                        if !baud.is_empty() && baud.chars().all(|c| c.is_ascii_digit()) =>
                    {
                        Ok(Self::Serial {
                            path: path.to_owned(),
                            baud: baud
                                .parse()
                                .map_err(|_| UrlError::BadNumber(baud.to_owned()))?,
                        })
                    }
                    _ => Ok(Self::Serial {
                        path: rest.to_owned(),
                        baud: DEFAULT_BAUD,
                    }),
                }
            }
            "tcp" => {
                let (host, port) = split_host_port(rest, DEFAULT_TCP_PORT)?;
                if host.is_empty() {
                    return Err(UrlError::Malformed("tcp: needs a host".to_owned()));
                }
                Ok(Self::Tcp { host, port })
            }
            "tcpin" => {
                let port = if rest.is_empty() {
                    DEFAULT_TCP_PORT
                } else {
                    rest.rsplit(':')
                        .next()
                        .unwrap_or(rest)
                        .parse()
                        .map_err(|_| UrlError::BadNumber(rest.to_owned()))?
                };
                Ok(Self::TcpListen { port })
            }
            "udp" => {
                // A bare number is a port, not a host: `udp:14550` is the form users type most.
                if !rest.contains(':')
                    && !rest.is_empty()
                    && rest.chars().all(|c| c.is_ascii_digit())
                {
                    let port = rest
                        .parse()
                        .map_err(|_| UrlError::BadNumber(rest.to_owned()))?;
                    return Ok(Self::Udp {
                        bind: "0.0.0.0".to_owned(),
                        port,
                    });
                }
                let (bind, port) = split_host_port(rest, DEFAULT_UDP_PORT)?;
                let bind = if bind.is_empty() {
                    "0.0.0.0".to_owned()
                } else {
                    bind
                };
                Ok(Self::Udp { bind, port })
            }
            "file" | "replay" => {
                if rest.is_empty() {
                    return Err(UrlError::Malformed("file: needs a path".to_owned()));
                }
                Ok(Self::File {
                    path: rest.to_owned(),
                })
            }
            other => Err(UrlError::UnknownScheme(other.to_owned())),
        }
    }
}

fn split_host_port(rest: &str, default_port: u16) -> Result<(String, u16), UrlError> {
    match rest.rsplit_once(':') {
        Some((host, port)) => {
            let port = port
                .parse()
                .map_err(|_| UrlError::BadNumber(port.to_owned()))?;
            Ok((host.to_owned(), port))
        }
        None => Ok((rest.to_owned(), default_port)),
    }
}

impl fmt::Display for LinkUrl {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Serial { path, baud } => write!(f, "serial:{path}:{baud}"),
            Self::Tcp { host, port } => write!(f, "tcp:{host}:{port}"),
            Self::TcpListen { port } => write!(f, "tcpin:{port}"),
            Self::Udp { bind, port } => write!(f, "udp:{bind}:{port}"),
            Self::File { path } => write!(f, "file:{path}"),
        }
    }
}
