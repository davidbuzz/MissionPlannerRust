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

//! The built-in Tailscale node: the owner's feature of 2026-10-14, not in the C# - "we will do
//! tailscale-rs as an optional built-in on the desktop, behind a setting marked experimental. we
//! WILL take the existing rust code, and add MagicDNS to it enough so that 'white' or 'buzz55' as
//! a hostname works instead of a [100].x ip address."
//!
//! A [`Node`] joins a tailnet as a node of its own, with no Tailscale client installed:
//! tailscale-rs - vendored in `third_party/tailscale-rs` with MagicDNS added (its README) - on a
//! tokio runtime of the node's own thread, behind calls that block with a timeout, as the
//! planner's link threads do. [`Node::start`] starts it and [`Node::state`] says how it stands
//! (starting, waiting for a sign-in at an address, running under a name and two addresses,
//! failed, stopped); [`Node::stop`] ends it. [`Node::resolve`] resolves a host as Go's tsnet does
//! when it dials one, and [`Node::connect_tcp`] and [`Node::bind_udp`] give the planner's TCP and
//! UDP-client links their sockets through it - mp-transport's `tailnet` module decides which
//! links go this way, by [`is_tailscale_ip`], [`is_ts_net_name`] and the node's name table.
//!
//! The node is the `node` feature, off by default; without it [`Node::start`] gives a node that
//! has failed with [`NO_NODE`], so a build without tailscale-rs says why rather than nothing.
//! Nothing at all in a web page, whose tailnet is the page's own (`web/www`). Written with the
//! planner's portable calls (mp-os's locks and waits, web-time's clock, wasm_thread's threads) as
//! every crate is, though no web page builds it.
#![cfg(not(target_family = "wasm"))]

use std::fmt;
use std::io;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr};
use std::path::PathBuf;

#[cfg(feature = "node")]
mod node;
#[cfg(not(feature = "node"))]
mod stub;

#[cfg(feature = "node")]
pub use node::{Node, TcpStream, UdpSocket};
#[cfg(not(feature = "node"))]
pub use stub::{Node, TcpStream, UdpSocket};

// The MagicDNS rules as tailscale-rs carries them, compiled into these tests so that the
// workspace runs that file's own: the vendored crates are outside the workspace
// (third_party/tailscale-rs/README.md).
#[cfg(test)]
#[allow(dead_code, unreachable_pub)]
#[path = "../../../third_party/tailscale-rs/ts_runtime/src/peer_tracker/magic_dns.rs"]
mod vendored_magic_dns;

/// What a build without the `node` feature says when its node is asked for.
pub const NO_NODE: &str = "this build has no built-in Tailscale node";

/// What a node says when it is given no folder to keep its keys in.
pub const NO_KEY_FOLDER: &str = "no folder to keep its keys in";

/// The file in [`Settings::state_dir`] the node keeps its keys in: its identity on the tailnet.
pub const KEY_FILE: &str = "tailscale-node.json";

/// What the node's name starts with, so that a tailnet tells the planner from the computer it runs
/// on, which may be a node of its own through the Tailscale client: `mpr-` and the machine's name.
pub const HOSTNAME_PREFIX: &str = "mpr-";

/// The longest DNS label, and so the longest name the node asks for.
const MAX_LABEL: usize = 63;

/// The name of Tailscale's own MagicDNS domain, as the end of a name: `white.tail1234.ts.net`.
const TS_NET: &str = ".ts.net";

/// How the node is started.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Settings {
    /// Where it keeps its keys ([`KEY_FILE`]): the planner's own data directory. With none
    /// (empty), the node does not start: it fails with [`NO_KEY_FOLDER`].
    pub state_dir: PathBuf,
    /// The name it asks the tailnet for: [`default_hostname`] unless set.
    pub hostname: String,
    /// A control server other than Tailscale's - a Headscale - or `None` for Tailscale's own.
    pub control_url: Option<String>,
}

impl Settings {
    /// Its keys in `state_dir`, [`default_hostname`], and Tailscale's control server.
    #[must_use]
    pub fn new(state_dir: PathBuf) -> Self {
        Self {
            state_dir,
            hostname: default_hostname(),
            control_url: None,
        }
    }
}

/// The node's name unless another is set: [`hostname_for`] this machine's name.
#[must_use]
pub fn default_hostname() -> String {
    hostname_for(&gethostname::gethostname().to_string_lossy())
}

/// The node's name on a machine named `machine`: [`HOSTNAME_PREFIX`] and the machine's first label
/// as a DNS label - lowercase; anything but a letter, a digit or a hyphen made a hyphen; no hyphen
/// at either end; no longer than a label may be - or `mpr-planner` for a machine with no name to
/// use. The control server would make a label of whatever was asked; asking for one keeps the name
/// the same on every control server.
#[must_use]
pub fn hostname_for(machine: &str) -> String {
    let first = machine.split('.').next().unwrap_or_default();
    let label: String = first
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() {
                c.to_ascii_lowercase()
            } else {
                '-'
            }
        })
        .collect();
    let label = label.trim_matches('-');
    let label = if label.is_empty() { "planner" } else { label };
    let mut name = format!("{HOSTNAME_PREFIX}{label}");
    // Every character is ASCII, so any length is a character boundary.
    name.truncate(MAX_LABEL);
    name.trim_end_matches('-').to_owned()
}

/// `tsaddr.IsTailscaleIP`: whether an address is one Tailscale assigns - in 100.64.0.0/10 less
/// ChromeOS's 100.115.92.0/23, or in fd7a:115c:a1e0::/48 - an IPv4-mapped IPv6 address taken as
/// the IPv4 address it maps.
/// Go: net/tsaddr/tsaddr.go:72-92 (the ranges at 23-37 and 89-92)
#[must_use]
pub fn is_tailscale_ip(ip: IpAddr) -> bool {
    match ip {
        IpAddr::V4(v4) => is_tailscale_ipv4(v4),
        IpAddr::V6(v6) => match v6.to_ipv4_mapped() {
            Some(v4) => is_tailscale_ipv4(v4),
            None => matches!(v6.segments(), [0xfd7a, 0x115c, 0xa1e0, ..]),
        },
    }
}

/// `tsaddr.IsTailscaleIPv4`: in the CGNAT range 100.64.0.0/10 and not in ChromeOS's, from which
/// Tailscale assigns nothing.
/// Go: net/tsaddr/tsaddr.go:80-85
fn is_tailscale_ipv4(ip: Ipv4Addr) -> bool {
    let [first, second, third, _] = ip.octets();
    // 100.64.0.0/10: 100, then 64 to 127.
    let cgnat = first == 100 && (64..=127).contains(&second);
    // 100.115.92.0/23: 100.115.92.x and 100.115.93.x.
    let chrome_os = first == 100 && second == 115 && (92..=93).contains(&third);
    cgnat && !chrome_os
}

/// Whether a host is a name in `ts.net`, Tailscale's own MagicDNS domain -
/// `white.tail1234.ts.net` - in any case, with or without a trailing dot.
#[must_use]
pub fn is_ts_net_name(host: &str) -> bool {
    let name = host.strip_suffix('.').unwrap_or(host).to_ascii_lowercase();
    name.len() > TS_NET.len() && name.ends_with(TS_NET)
}

/// How the node stands.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum State {
    /// Not started, or stopped.
    #[default]
    Stopped,
    /// Started: reading its keys, reaching the control server, or waiting for its first network
    /// map.
    Starting,
    /// The control server wants the node signed in to a tailnet first.
    NeedsSignIn {
        /// The page to sign in at, in a browser.
        url: String,
    },
    /// On the tailnet.
    Running {
        /// Its full MagicDNS name (`mpr-host.tail1234.ts.net`).
        name: String,
        /// Its IPv4 address on the tailnet, in 100.64.0.0/10.
        ipv4: Ipv4Addr,
        /// Its IPv6 address on the tailnet, in fd7a:115c:a1e0::/48.
        ipv6: Ipv6Addr,
    },
    /// It could not start, or the tailnet failed it since.
    Failed {
        /// Why, said.
        why: String,
    },
}

impl State {
    /// One word for it, as the facts carry it: `stopped`, `starting`, `needs-sign-in`, `running`
    /// or `failed`.
    #[must_use]
    pub const fn word(&self) -> &'static str {
        match self {
            Self::Stopped => "stopped",
            Self::Starting => "starting",
            Self::NeedsSignIn { .. } => "needs-sign-in",
            Self::Running { .. } => "running",
            Self::Failed { .. } => "failed",
        }
    }

    /// Whether it is on the tailnet.
    #[must_use]
    pub const fn is_running(&self) -> bool {
        matches!(self, Self::Running { .. })
    }

    /// Whether it is running or on its way - starting, or waiting for its sign-in - rather than
    /// stopped or failed: links to the tailnet go through a node in use and no other.
    #[must_use]
    pub const fn in_use(&self) -> bool {
        matches!(
            self,
            Self::Starting | Self::NeedsSignIn { .. } | Self::Running { .. }
        )
    }
}

impl fmt::Display for State {
    /// A line for the screen: `running as mpr-host.tail1234.ts.net (100.101.102.103, fd7a:...)`,
    /// `needs a sign-in at https://...`, `failed: why`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Stopped => f.write_str("stopped"),
            Self::Starting => f.write_str("starting"),
            Self::NeedsSignIn { url } => write!(f, "needs a sign-in at {url}"),
            Self::Running { name, ipv4, ipv6 } => write!(f, "running as {name} ({ipv4}, {ipv6})"),
            Self::Failed { why } => write!(f, "failed: {why}"),
        }
    }
}

/// How the node stands from what its device said: `sign_in`, the page the control server gave
/// while the node is not signed in (`None` once it is); `node`, its name and addresses as its
/// network map gives them, once there is one. Signed in with no map yet is still starting.
#[cfg_attr(not(feature = "node"), allow(dead_code))]
pub(crate) fn state_after(
    sign_in: Option<&str>,
    node: Option<(&str, Ipv4Addr, Ipv6Addr)>,
) -> State {
    match (sign_in, node) {
        (Some(url), _) => State::NeedsSignIn {
            url: url.to_owned(),
        },
        (None, Some((name, ipv4, ipv6))) => State::Running {
            name: name.to_owned(),
            ipv4,
            ipv6,
        },
        (None, None) => State::Starting,
    }
}

/// What a call waiting for the node to run does when it stands as `state`, with the call's time
/// up or not: go on (`Some(Ok)`), give up and say why (`Some(Err)`), or wait for the next change
/// (`None`). A node starting or waiting for its sign-in may yet run; a stopped or failed one will
/// not.
#[cfg_attr(not(feature = "node"), allow(dead_code))]
pub(crate) fn wait_verdict(state: &State, time_up: bool) -> Option<Result<(), Error>> {
    match state {
        State::Running { .. } => Some(Ok(())),
        State::Stopped | State::Failed { .. } => Some(Err(Error::NotRunning(state.to_string()))),
        State::NeedsSignIn { url } => time_up.then(|| Err(Error::NeedsSignIn(url.clone()))),
        State::Starting => time_up.then(|| Err(Error::NotRunning(state.to_string()))),
    }
}

/// Why the node could not do what it was asked.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    /// This build has no node: [`NO_NODE`].
    #[error("{}", NO_NODE)]
    NoNode,
    /// The node is not on the tailnet: how it stands instead.
    #[error("the built-in Tailscale node is not running ({0})")]
    NotRunning(String),
    /// The node waits for a sign-in, at this page.
    #[error("the built-in Tailscale node needs a sign-in at {0}")]
    NeedsSignIn(String),
    /// A host that is neither an address nor a MagicDNS name on the tailnet (Go's
    /// `errUnresolved`).
    #[error("{0} is not a name on the tailnet")]
    Unresolved(String),
    /// The tailnet did not answer in time.
    #[error("the tailnet did not answer in time")]
    Timeout,
    /// tailscale-rs's own error, said.
    #[error("{0}")]
    Tailscale(String),
}

impl From<Error> for io::Error {
    fn from(err: Error) -> Self {
        let kind = match &err {
            Error::NoNode => io::ErrorKind::Unsupported,
            Error::NotRunning(_) | Error::NeedsSignIn(_) => io::ErrorKind::NotConnected,
            Error::Unresolved(_) => io::ErrorKind::NotFound,
            Error::Timeout => io::ErrorKind::TimedOut,
            Error::Tailscale(_) => io::ErrorKind::Other,
        };
        Self::new(kind, err)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 100.64.0.0/10 and fd7a:115c:a1e0::/48 are the tailnet's, ChromeOS's 100.115.92.0/23 is
    /// not, a mapped address is the address it maps, and the neighbours of the ranges are not.
    #[test]
    fn tailscale_addresses_are_tsaddrs() {
        let ip = |text: &str| text.parse::<IpAddr>().expect("an address");
        for tailnet in [
            "100.64.0.0",
            "100.101.102.103",
            "100.127.255.255",
            "100.115.91.255",
            "100.115.94.0",
            "fd7a:115c:a1e0::1",
            "fd7a:115c:a1e0:ab12:4843:cd96:6258:b240",
            "::ffff:100.64.0.1",
        ] {
            assert!(is_tailscale_ip(ip(tailnet)), "{tailnet} is the tailnet's");
        }
        for elsewhere in [
            "100.63.255.255",
            "100.128.0.0",
            "100.115.92.1",
            "100.115.93.255",
            "127.0.0.1",
            "192.168.4.1",
            "10.0.2.2",
            "fd7a:115c:a1e1::1",
            "fd7b:115c:a1e0::1",
            "::1",
            "::ffff:192.168.1.1",
        ] {
            assert!(!is_tailscale_ip(ip(elsewhere)), "{elsewhere} is not the tailnet's");
        }
    }

    /// A name in `ts.net`, in any case, a trailing dot or none; `ts.net` itself is no node's.
    #[test]
    fn ts_net_names_end_in_ts_net() {
        assert!(is_ts_net_name("white.tail1234.ts.net"));
        assert!(is_ts_net_name("White.Tail1234.TS.NET."));
        assert!(is_ts_net_name("a.ts.net"));
        assert!(!is_ts_net_name("ts.net"));
        assert!(!is_ts_net_name(".ts.net"));
        assert!(!is_ts_net_name("white"));
        assert!(!is_ts_net_name("white.example.com"));
        assert!(!is_ts_net_name("white.ts.network"));
        assert!(!is_ts_net_name("100.64.0.1"));
    }

    /// The node's name: the prefix and the machine's first label made a lowercase DNS label, a
    /// fallback for a machine with none, never longer than a label.
    #[test]
    fn the_nodes_name_is_a_label_after_the_prefix() {
        assert_eq!(hostname_for("Buzz-Laptop.local"), "mpr-buzz-laptop");
        assert_eq!(hostname_for("WHITE"), "mpr-white");
        assert_eq!(hostname_for("My PC"), "mpr-my-pc");
        assert_eq!(hostname_for("_build_box_"), "mpr-build-box");
        assert_eq!(hostname_for("ünïcode"), "mpr-n-code");
        assert_eq!(hostname_for(""), "mpr-planner");
        assert_eq!(hostname_for("---"), "mpr-planner");
        let long = hostname_for(&"a".repeat(100));
        assert_eq!(long.len(), MAX_LABEL);
        assert!(long.starts_with(HOSTNAME_PREFIX));
        let cut = hostname_for(&format!("{}-b", "a".repeat(58)));
        assert!(!cut.ends_with('-'), "{cut}");
        assert!(default_hostname().starts_with(HOSTNAME_PREFIX));
        let settings = Settings::new(PathBuf::from("keys"));
        assert_eq!(settings.state_dir, PathBuf::from("keys"));
        assert!(settings.hostname.starts_with(HOSTNAME_PREFIX));
        assert_eq!(settings.control_url, None);
    }

    /// What the device said, as a state: a sign-in page first, whatever else; signed in and
    /// named, running; signed in with no map yet, starting.
    #[test]
    fn the_devices_answers_map_to_states() {
        let ipv4 = Ipv4Addr::new(100, 101, 102, 103);
        let ipv6 = Ipv6Addr::new(0xfd7a, 0x115c, 0xa1e0, 0, 0, 0, 0, 1);
        let url = "https://login.tailscale.com/a/abc123";
        assert_eq!(
            state_after(Some(url), None),
            State::NeedsSignIn { url: url.to_owned() }
        );
        assert_eq!(
            state_after(Some(url), Some(("mpr-host.tail1234.ts.net", ipv4, ipv6))),
            State::NeedsSignIn { url: url.to_owned() }
        );
        let running = state_after(None, Some(("mpr-host.tail1234.ts.net", ipv4, ipv6)));
        assert_eq!(
            running,
            State::Running {
                name: "mpr-host.tail1234.ts.net".to_owned(),
                ipv4,
                ipv6
            }
        );
        assert_eq!(state_after(None, None), State::Starting);

        assert_eq!(running.word(), "running");
        assert!(running.is_running() && running.in_use());
        assert_eq!(
            running.to_string(),
            "running as mpr-host.tail1234.ts.net (100.101.102.103, fd7a:115c:a1e0::1)"
        );
        let signing_in = State::NeedsSignIn { url: url.to_owned() };
        assert_eq!(signing_in.word(), "needs-sign-in");
        assert!(!signing_in.is_running() && signing_in.in_use());
        assert_eq!(signing_in.to_string(), format!("needs a sign-in at {url}"));
        assert!(State::Starting.in_use());
        assert_eq!(State::Starting.word(), "starting");
        let failed = State::Failed { why: NO_NODE.to_owned() };
        assert_eq!(failed.word(), "failed");
        assert!(!failed.in_use());
        assert_eq!(failed.to_string(), format!("failed: {NO_NODE}"));
        assert_eq!(State::default(), State::Stopped);
        assert!(!State::Stopped.in_use());
        assert_eq!(State::Stopped.word(), "stopped");
    }

    /// A call waiting for the node: on at once when it runs, never on a stopped or failed one,
    /// and on a starting or signing-in one only until its time is up, then saying why.
    #[test]
    fn a_waiting_call_goes_on_waits_or_gives_up() {
        let running = State::Running {
            name: "mpr-host.tail1234.ts.net".to_owned(),
            ipv4: Ipv4Addr::new(100, 64, 0, 1),
            ipv6: Ipv6Addr::LOCALHOST,
        };
        assert_eq!(wait_verdict(&running, false), Some(Ok(())));
        assert_eq!(wait_verdict(&running, true), Some(Ok(())));
        assert_eq!(wait_verdict(&State::Starting, false), None);
        assert_eq!(
            wait_verdict(&State::Starting, true),
            Some(Err(Error::NotRunning("starting".to_owned())))
        );
        let signing_in = State::NeedsSignIn { url: "https://login".to_owned() };
        assert_eq!(wait_verdict(&signing_in, false), None);
        assert_eq!(
            wait_verdict(&signing_in, true),
            Some(Err(Error::NeedsSignIn("https://login".to_owned())))
        );
        assert_eq!(
            wait_verdict(&State::Stopped, false),
            Some(Err(Error::NotRunning("stopped".to_owned())))
        );
        let failed = State::Failed { why: "no keys".to_owned() };
        assert_eq!(
            wait_verdict(&failed, false),
            Some(Err(Error::NotRunning("failed: no keys".to_owned())))
        );
    }

    /// The errors as a link reports them, and as the transports' I/O sees them: a timeout is a
    /// timeout, a name not found is not found, a node not running is not connected.
    #[test]
    fn errors_say_why_and_map_to_io_kinds() {
        let kind = |err: Error| io::Error::from(err).kind();
        assert_eq!(kind(Error::NoNode), io::ErrorKind::Unsupported);
        assert_eq!(kind(Error::NotRunning("stopped".to_owned())), io::ErrorKind::NotConnected);
        assert_eq!(kind(Error::NeedsSignIn("https://x".to_owned())), io::ErrorKind::NotConnected);
        assert_eq!(kind(Error::Unresolved("white".to_owned())), io::ErrorKind::NotFound);
        assert_eq!(kind(Error::Timeout), io::ErrorKind::TimedOut);
        assert_eq!(kind(Error::Tailscale("reset".to_owned())), io::ErrorKind::Other);
        assert_eq!(Error::NoNode.to_string(), NO_NODE);
        assert_eq!(
            Error::NeedsSignIn("https://login.tailscale.com/a/x".to_owned()).to_string(),
            "the built-in Tailscale node needs a sign-in at https://login.tailscale.com/a/x"
        );
        assert_eq!(
            Error::Unresolved("white".to_owned()).to_string(),
            "white is not a name on the tailnet"
        );
        let said = io::Error::from(Error::NotRunning("starting".to_owned())).to_string();
        assert_eq!(said, "the built-in Tailscale node is not running (starting)");
    }
}
