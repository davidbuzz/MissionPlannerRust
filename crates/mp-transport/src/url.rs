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
    /// `udpcl:192.168.4.1:14550` - send to a host from a port of our own, and read what comes
    /// back: Mission Planner's "UDPCl" (`UdpSerialConnect`). `udpcl:14550` is the C#'s default
    /// host, 127.0.0.1.
    UdpClient {
        /// Remote host.
        host: String,
        /// Remote port.
        port: u16,
    },
    /// `ws://host:port/path` - a websocket: Mission Planner's "WS", whose URL is `WS_url`. Kept as
    /// written, since that is what the C# hands `System.Uri`; `wss://` parses but needs TLS to
    /// open.
    WebSocket {
        /// The whole URL, `ws://` or `wss://` included.
        url: String,
    },
    /// `ntrip://user:pass@host:port/mount` - RTK corrections from an NTRIP caster. Kept as written,
    /// since `CommsNTRIP.Open` escapes the text itself.
    Ntrip {
        /// The whole URL, `ntrip://` included.
        url: String,
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
    #[error(
        "missing scheme in {0:?}; expected serial:, tcp:, tcpin:, udp:, udpcl:, ws://, ntrip:// or file:"
    )]
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

        let Some((scheme, rest)) = s.split_once(':') else {
            // No scheme at all. This is what a user actually types, because it is what the shell
            // completes: `/dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176_...-if-mavlink` comes
            // straight off tab completion, and demanding `serial:` in front of it is a rule that
            // exists only for the parser's convenience.
            return bare_path(s).ok_or_else(|| UrlError::MissingScheme(s.to_owned()));
        };

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
            "udpcl" => {
                // C#: ExtLibs/Comms/CommsUDPSerialConnect.cs:31-35, 133-134 - host 127.0.0.1 and
                // port 14550 unless told otherwise.
                if !rest.is_empty() && rest.chars().all(|c| c.is_ascii_digit()) {
                    let port = rest
                        .parse()
                        .map_err(|_| UrlError::BadNumber(rest.to_owned()))?;
                    return Ok(Self::UdpClient {
                        host: "127.0.0.1".to_owned(),
                        port,
                    });
                }
                let (host, port) = split_host_port(rest, DEFAULT_UDP_PORT)?;
                if host.is_empty() {
                    return Err(UrlError::Malformed("udpcl: needs a host".to_owned()));
                }
                Ok(Self::UdpClient { host, port })
            }
            "ws" | "wss" => {
                // C#: ExtLibs/Comms/CommsWebSocket.cs:103-111, 196 - the URL goes to `new Uri` and
                // `ClientWebSocket.ConnectAsync` as typed.
                crate::dotnet::Uri::parse(s)
                    .map_err(|e| UrlError::Malformed(format!("{s}: {e}")))?;
                Ok(Self::WebSocket { url: s.to_owned() })
            }
            "ntrip" => {
                // C#: ExtLibs/Comms/CommsNTRIP.cs:137-155 - checked as `Open` will read it.
                if !rest.starts_with("//") {
                    return Err(UrlError::Malformed(format!(
                        "{s}: an NTRIP URL is ntrip://user:pass@host:port/mount"
                    )));
                }
                crate::ntrip::check_url(s).map_err(|e| UrlError::Malformed(format!("{s}: {e}")))?;
                Ok(Self::Ntrip { url: s.to_owned() })
            }
            "file" | "replay" => {
                if rest.is_empty() {
                    return Err(UrlError::Malformed("file: needs a path".to_owned()));
                }
                Ok(Self::File {
                    path: rest.to_owned(),
                })
            }
            // Not a scheme we know - but a Windows drive path's colon is a drive letter, not a
            // scheme, so `C:\logs\flight.tlog` arrives here and is a path. Tried only after
            // every real scheme has been ruled out, so an explicit `file:flight.tlog` is never
            // mistaken for a bare path that happens to end in a log extension.
            other => bare_path(s).ok_or_else(|| UrlError::UnknownScheme(other.to_owned())),
        }
    }
}

/// Recognises a path typed without a scheme, by shape alone.
///
/// Shape rather than a filesystem check, so that parsing stays pure and testable: a device that is
/// unplugged should fail when it is opened, with a message about the device, rather than being
/// parsed as something else entirely.
fn bare_path(s: &str) -> Option<LinkUrl> {
    if s.is_empty() {
        return None;
    }

    // Windows serial ports: COM3, and \\.\COM10 for numbers above nine.
    let windows_serial = {
        let stripped = s.strip_prefix(r#"\\.\"#).unwrap_or(s);
        let upper = stripped.to_ascii_uppercase();
        upper
            .strip_prefix("COM")
            .is_some_and(|n| !n.is_empty() && n.chars().all(|c| c.is_ascii_digit()))
    };

    // Unix device nodes. /dev/serial/by-id/... is the stable name, and the one worth typing:
    // /dev/ttyACM0 changes when something else is plugged in first.
    if s.starts_with("/dev/") || windows_serial {
        return Some(LinkUrl::Serial {
            path: s.to_owned(),
            baud: DEFAULT_BAUD,
        });
    }

    // A log to replay, by extension. `.bin` and `.log` are dataflash, `.tlog` is telemetry.
    let lower = s.to_ascii_lowercase();
    let is_log = [".tlog", ".bin", ".log", ".rlog"]
        .iter()
        .any(|extension| lower.ends_with(extension));
    if is_log {
        return Some(LinkUrl::File { path: s.to_owned() });
    }

    // A Windows drive path such as C:\logs\flight.tlog. The colon is a drive letter, not a
    // scheme, and without this it parses as the unknown scheme "c".
    let drive_path = {
        let mut chars = s.chars();
        let letter = chars.next().is_some_and(|c| c.is_ascii_alphabetic());
        let colon = chars.next() == Some(':');
        let separator = matches!(chars.next(), Some('\\' | '/'));
        letter && colon && separator
    };
    if drive_path {
        return Some(LinkUrl::File { path: s.to_owned() });
    }

    None
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
            Self::UdpClient { host, port } => write!(f, "udpcl:{host}:{port}"),
            Self::WebSocket { url } | Self::Ntrip { url } => f.write_str(url),
        }
    }
}

#[cfg(test)]
mod bare_path_tests {
    use super::*;

    fn parse(s: &str) -> LinkUrl {
        s.parse()
            .unwrap_or_else(|e| panic!("{s:?} should parse: {e}"))
    }

    #[test]
    fn a_device_path_from_tab_completion_is_a_serial_port() {
        // The exact string a shell completes for an ArduPilot flight controller. Demanding
        // "serial:" in front of it is a rule that exists only for the parser's convenience.
        let url =
            parse("/dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176_3B1C280E8295B591-if-mavlink");
        match url {
            LinkUrl::Serial { path, baud } => {
                assert!(path.ends_with("-if-mavlink"), "{path}");
                assert_eq!(baud, DEFAULT_BAUD);
            }
            other => panic!("expected a serial port, got {other:?}"),
        }
    }

    #[test]
    fn the_short_device_names_work_too() {
        assert!(matches!(parse("/dev/ttyACM0"), LinkUrl::Serial { .. }));
        assert!(matches!(parse("/dev/ttyUSB0"), LinkUrl::Serial { .. }));
    }

    #[test]
    fn windows_serial_ports_are_recognised_by_name() {
        assert!(matches!(parse("COM3"), LinkUrl::Serial { .. }));
        assert!(matches!(parse("com3"), LinkUrl::Serial { .. }));
        // Above nine, Windows needs the device-namespace form.
        assert!(matches!(parse(r#"\\.\COM10"#), LinkUrl::Serial { .. }));
    }

    #[test]
    fn a_log_file_is_recognised_by_extension() {
        assert!(matches!(parse("flight.tlog"), LinkUrl::File { .. }));
        assert!(matches!(parse("00000042.BIN"), LinkUrl::File { .. }));
        assert!(matches!(
            parse("logs/2026-09-23.tlog"),
            LinkUrl::File { .. }
        ));
    }

    #[test]
    fn a_windows_drive_path_is_a_path_not_a_scheme() {
        // Its colon is a drive letter. Without this it parsed as the unknown scheme "c".
        match parse(r"C:\logs\flight.tlog") {
            LinkUrl::File { path } => assert!(path.starts_with("C:"), "{path}"),
            other => panic!("expected a file, got {other:?}"),
        }
        assert!(matches!(parse("D:/logs/flight.tlog"), LinkUrl::File { .. }));
    }

    #[test]
    fn explicit_schemes_still_win_over_the_bare_forms() {
        // A bare path must not shadow anything that was already unambiguous, and must not swallow
        // the scheme when it does apply. Checking the parsed *values*, not just the variant: an
        // earlier version of this matched `LinkUrl::File { .. }` and so happily passed while
        // parsing "file:flight.tlog" into a path of "file:flight.tlog".
        assert_eq!(
            parse("serial:/dev/ttyACM0:115200"),
            LinkUrl::Serial {
                path: "/dev/ttyACM0".to_owned(),
                baud: 115_200
            }
        );
        assert_eq!(
            parse("file:flight.tlog"),
            LinkUrl::File {
                path: "flight.tlog".to_owned()
            }
        );
        assert_eq!(
            parse("file:/dev/ttyACM0"),
            LinkUrl::File {
                path: "/dev/ttyACM0".to_owned()
            }
        );
        assert!(matches!(parse("tcp:127.0.0.1:5760"), LinkUrl::Tcp { .. }));
        assert!(matches!(parse("udp:14550"), LinkUrl::Udp { .. }));
    }

    #[test]
    fn something_that_is_neither_still_reports_the_missing_scheme() {
        // Guessing at a bare word would turn a typo into a confusing connection attempt.
        let error = "localhost"
            .parse::<LinkUrl>()
            .expect_err("should not parse");
        assert!(
            matches!(error, UrlError::MissingScheme(_)),
            "expected a missing-scheme error, got {error}"
        );
        assert!("".parse::<LinkUrl>().is_err());
    }

    #[test]
    fn a_bare_path_is_parsed_by_shape_not_by_looking_at_the_disk() {
        // Parsing stays pure: a device that is unplugged fails when it is opened, with a message
        // about the device, rather than being parsed as something else entirely.
        let url = parse("/dev/serial/by-id/usb-nothing-here-if-mavlink");
        assert!(matches!(url, LinkUrl::Serial { .. }));
    }

    #[test]
    fn a_parsed_bare_path_round_trips_through_display() {
        let url = parse("/dev/ttyACM0");
        let shown = url.to_string();
        assert_eq!(shown, format!("serial:/dev/ttyACM0:{DEFAULT_BAUD}"));
        assert_eq!(shown.parse::<LinkUrl>().expect("round trip"), url);
    }
}
