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

//! Tailscale as this computer has it: whether the official client is installed, signed in and on
//! a tailnet - for the indicator in the connection area, the status line's reason when a link to
//! a tailnet address fails, and the first start's offer of Tailscale's download page. The owner's
//! feature of 2026-10-14 ("A Tailscale indicator in the connection area. On the desktop: not
//! installed, signed out, or "on tailnet as name, 100.x" ... offer it as a link, not run an
//! installer"). **Not in the C#**: Mission Planner knows nothing of Tailscale. using_tailscale.md
//! tells a user how the two are used together.
//!
//! The desktop asks Tailscale's own CLI, `tailscale status --json --peers=false`, on a thread of
//! its own every [`POLL_INTERVAL`] and when asked ([`Poller::check_now`]); the window takes the
//! newest answer once a frame ([`Poller::tick`]) and never waits for one. Nothing here changes
//! anything - no `up`, no `login`, no `set`: signing in is the user's, in Tailscale's app or a
//! terminal, and the planner sees it at its next look. A web page runs no programs: there the
//! page's own Tailscale node has the state (web/www/tailscale.js), and nothing here is started.
//!
//! `MP_TAILSCALE_CLI` is the harness's door ([`CLI_ENV`]): the CLI to run in place of the one
//! found - a GUI script's tests/gui/fake-tailscale.sh, which answers as the real one does in a
//! state of the script's choosing - or `none` for a computer without Tailscale.
//!
//! The JSON's fields and the CLI's exits are Tailscale 1.104's, read from its source:
//! `ipn/ipnstate/ipnstate.go` (`Status`, `PeerStatus`, `TailnetStatus`), the CLI's
//! `cmd/tailscale/cli/status.go` and `diag.go`, and `ipn/ipnlocal/local.go`'s `nextStateLocked`
//! for what each backend state means.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]
// In a web page the CLI's half is not compiled, and what only it uses goes unused there: the
// page's node gives the browser build its state.
#![cfg_attr(target_family = "wasm", allow(dead_code))]

use std::net::{IpAddr, Ipv4Addr, Ipv6Addr};
use std::time::Duration;

use serde_json::Value;

/// The harness's door: the CLI to run in place of the one found, or [`NO_CLI`] for none.
pub const CLI_ENV: &str = "MP_TAILSCALE_CLI";

/// [`CLI_ENV`]'s word for a computer without Tailscale.
pub const NO_CLI: &str = "none";

/// What the CLI is asked: the status as JSON, only reading. `--peers=false` leaves out the other
/// devices on the tailnet (status.go:81-83, the local API's `?peers=false`): this device's state
/// is all the indicator shows, and a tailnet of hundreds of devices would otherwise be hundreds
/// of them every few seconds.
pub const STATUS_ARGS: [&str; 3] = ["status", "--json", "--peers=false"];

/// How often the poller asks: a sign-in shows within seconds of it - the owner's "then the
/// planner detects the sign-in" - and one short program every five seconds costs nothing worth
/// measuring.
pub const POLL_INTERVAL: Duration = Duration::from_secs(5);

/// How long one ask may take before the CLI is killed and the state is unknown: a daemon answers
/// in milliseconds, and the macOS app's executable, started as the CLI, takes longer to load.
/// Under [`POLL_INTERVAL`], so one ask is over before the next is due.
pub const CLI_TIMEOUT: Duration = Duration::from_secs(4);

/// Tailscale's download page, which leads to each system's.
pub const DOWNLOAD_PAGE: &str = "https://tailscale.com/download";

/// Its page for Linux.
pub const DOWNLOAD_PAGE_LINUX: &str = "https://tailscale.com/download/linux";

/// Its page for macOS.
pub const DOWNLOAD_PAGE_MAC: &str = "https://tailscale.com/download/mac";

/// Its page for Windows.
pub const DOWNLOAD_PAGE_WINDOWS: &str = "https://tailscale.com/download/windows";

/// Linux's install and sign-in, as using_tailscale.md has them (its step 2): Tailscale's install
/// script, then `tailscale up`, which prints the sign-in link.
pub const LINUX_INSTALL: &[&str] = &[
    "curl -fsSL https://tailscale.com/install.sh | sh",
    "sudo tailscale up",
];

/// Where the planner's own words on Tailscale are: using_tailscale.md, on the HELP screen.
pub const HELP_PAGE: &str = "HELP > Using Tailscale";

/// What the indicator shows: this computer's Tailscale, as its CLI last told it.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum TailnetState {
    /// Not asked yet: the poller's first answer has not come.
    #[default]
    Unchecked,
    /// No Tailscale client on this computer: no CLI on `PATH` or where the installers put it.
    NotInstalled,
    /// In a web page, the page's own node not started yet: its 8 MB are fetched only when a link
    /// or the indicator asks for it (web/www/tailscale.js, `startTailscale`).
    #[cfg_attr(not(target_family = "wasm"), allow(dead_code))] // the browser build's
    NotStarted,
    /// The client is installed and its daemon - `tailscaled`, the macOS app's network
    /// extension, the Windows service - does not answer.
    NotRunning {
        /// The CLI's own words, such as "failed to connect to local tailscaled; it doesn't
        /// appear to be running (sudo systemctl start tailscaled ?)" (diag.go:50-71).
        reason: String,
    },
    /// Signed out (`NeedsLogin`): never signed in, signed out, or its key expired.
    SignedOut {
        /// Where to sign in (`AuthURL`), once a sign-in has been started - by `tailscale up` or
        /// the app; none before.
        login_url: Option<String>,
    },
    /// Signed in, and waiting for the tailnet's admin to approve this device
    /// (`NeedsMachineAuth`).
    NeedsApproval,
    /// Turned off by its user (`Stopped`: `tailscale down`, or the app's switch).
    Stopped,
    /// Signed in and connecting (`Starting`), or the daemon's first moments, before it knows
    /// whether it is signed in (`NoState`, which "UIs should print "Loading..." in": local.go,
    /// `nextStateLocked`).
    Starting,
    /// On a tailnet (`Running`).
    OnTailnet(Device),
    /// The CLI answered something this module cannot read, or nothing in [`CLI_TIMEOUT`].
    Unknown {
        /// What went wrong.
        reason: String,
    },
}

impl TailnetState {
    /// The indicator's words: "Tailscale: not installed", "Tailscale: signed out", "Tailscale: on
    /// tailnet as buzz-laptop, 100.101.102.103" and so on.
    #[must_use]
    pub fn text(&self) -> String {
        match self {
            Self::Unchecked => "Tailscale: checking".to_owned(),
            Self::NotInstalled => "Tailscale: not installed".to_owned(),
            Self::NotStarted => "Tailscale: not started".to_owned(),
            Self::NotRunning { .. } => "Tailscale: not running".to_owned(),
            Self::SignedOut { .. } => "Tailscale: signed out".to_owned(),
            Self::NeedsApproval => "Tailscale: waiting for approval".to_owned(),
            Self::Stopped => "Tailscale: stopped".to_owned(),
            Self::Starting => "Tailscale: starting".to_owned(),
            Self::OnTailnet(device) => {
                let mut text = "Tailscale: on tailnet".to_owned();
                if !device.name.is_empty() {
                    text.push_str(" as ");
                    text.push_str(&device.name);
                }
                if let Some(address) = device.address() {
                    text.push_str(&format!(", {address}"));
                }
                text
            }
            Self::Unknown { .. } => "Tailscale: state unknown".to_owned(),
        }
    }

    /// The longer account, for the indicator's hover: the CLI's own words when the daemon does
    /// not answer, where to sign in, this device's tailnet, name and addresses.
    #[must_use]
    pub fn details(&self) -> String {
        match self {
            Self::Unchecked => "Tailscale has not been asked yet".to_owned(),
            Self::NotInstalled => format!(
                "Tailscale is not installed on this computer: {}",
                download_page(Platform::current())
            ),
            Self::NotStarted => "This page's Tailscale node starts on the first TCP or UDPCl \
                 connect, or with a click here, and asks you to sign in"
                .to_owned(),
            Self::NotRunning { reason } => {
                format!("Tailscale is installed and not running: {reason}")
            }
            Self::SignedOut {
                login_url: Some(url),
            } => format!("This computer is signed out of Tailscale: sign in at {url}"),
            Self::SignedOut { login_url: None } => "This computer is signed out of Tailscale: \
                 sign in with the Tailscale app, or on Linux with `sudo tailscale up`"
                .to_owned(),
            Self::NeedsApproval => "Signed in to Tailscale: this device waits for the tailnet's \
                 admin to approve it"
                .to_owned(),
            Self::Stopped => "Tailscale is turned off: turn it on in the Tailscale app, or on \
                 Linux with `sudo tailscale up`"
                .to_owned(),
            Self::Starting => "Tailscale is starting".to_owned(),
            Self::OnTailnet(device) => device.details(),
            Self::Unknown { reason } => format!("Tailscale's state is unknown: {reason}"),
        }
    }

    /// Whether this computer is on a tailnet.
    #[must_use]
    pub const fn is_on_tailnet(&self) -> bool {
        matches!(self, Self::OnTailnet(_))
    }

    /// This device on its tailnet, when it is on one.
    #[must_use]
    pub const fn device(&self) -> Option<&Device> {
        match self {
            Self::OnTailnet(device) => Some(device),
            _ => None,
        }
    }

    /// The fact's word for the state, stable for scripts: `unchecked`, `not-installed`,
    /// `not-running`, `signed-out`, `needs-approval`, `stopped`, `starting`, `on-tailnet`,
    /// `unknown`.
    #[must_use]
    pub const fn word(&self) -> &'static str {
        match self {
            Self::Unchecked => "unchecked",
            Self::NotInstalled => "not-installed",
            Self::NotStarted => "not-started",
            Self::NotRunning { .. } => "not-running",
            Self::SignedOut { .. } => "signed-out",
            Self::NeedsApproval => "needs-approval",
            Self::Stopped => "stopped",
            Self::Starting => "starting",
            Self::OnTailnet(_) => "on-tailnet",
            Self::Unknown { .. } => "unknown",
        }
    }

    /// Whether the state says this computer is off every tailnet: seen to be so, unlike
    /// [`Self::Unchecked`] and [`Self::Unknown`], which say nothing either way.
    const fn is_off_tailnet(&self) -> bool {
        matches!(
            self,
            Self::NotInstalled
                | Self::NotStarted
                | Self::NotRunning { .. }
                | Self::SignedOut { .. }
                | Self::NeedsApproval
                | Self::Stopped
                | Self::Starting
        )
    }
}

/// This computer as its tailnet knows it, from `status --json`'s `Self` and `CurrentTailnet`.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Device {
    /// Its machine name: the first label of its MagicDNS name (`DNSName`, `buzz-laptop` of
    /// `buzz-laptop.tail1234.ts.net.`) - the name the admin console shows, `tailscale status`
    /// prints and the other devices reach it by - else the computer's host name (`HostName`).
    pub name: String,
    /// Its tailnet IPv4 address, a 100.x.
    pub ipv4: Option<Ipv4Addr>,
    /// Its tailnet IPv6 address, an fd7a:115c:a1e0:...
    pub ipv6: Option<Ipv6Addr>,
    /// The tailnet's name (`CurrentTailnet.Name`): an organisation's domain, or for a personal
    /// tailnet its owner's login.
    pub tailnet: Option<String>,
    /// The tailnet's MagicDNS suffix (`CurrentTailnet.MagicDNSSuffix`, `tail1234.ts.net`).
    pub magic_dns_suffix: Option<String>,
}

impl Device {
    /// The address the indicator shows: the IPv4, else the IPv6.
    #[must_use]
    pub fn address(&self) -> Option<IpAddr> {
        self.ipv4
            .map(IpAddr::V4)
            .or_else(|| self.ipv6.map(IpAddr::V6))
    }

    /// The hover's account: the tailnet - by its MagicDNS suffix, which names no person, else by
    /// its name - this device's name and both its addresses.
    fn details(&self) -> String {
        let mut text = "On the tailnet".to_owned();
        if let Some(tailnet) = self.magic_dns_suffix.as_ref().or(self.tailnet.as_ref()) {
            text.push(' ');
            text.push_str(tailnet);
        }
        if !self.name.is_empty() {
            text.push_str(" as ");
            text.push_str(&self.name);
        }
        let addresses: Vec<String> = [self.ipv4.map(IpAddr::V4), self.ipv6.map(IpAddr::V6)]
            .into_iter()
            .flatten()
            .map(|address| address.to_string())
            .collect();
        if !addresses.is_empty() {
            text.push_str(": ");
            text.push_str(&addresses.join(", "));
        }
        text
    }
}

/// `tailscale status --json`'s answer read: its `BackendState` as the indicator's state, with the
/// sign-in link (`AuthURL`) when signed out and this device when on a tailnet. The fields are
/// `ipnstate.Status`'s, `PeerStatus`'s and `TailnetStatus`'s, as Go's encoding/json names them -
/// untagged, so by their Go names (ipnstate.go:32-91, 172-191, 238-346).
#[must_use]
pub fn parse_status(json: &str) -> TailnetState {
    let status: Value = match serde_json::from_str(json) {
        Ok(status) => status,
        Err(err) => {
            return TailnetState::Unknown {
                reason: format!("tailscale status gave no JSON: {err}"),
            };
        }
    };
    let Some(backend_state) = status.get("BackendState").and_then(Value::as_str) else {
        return TailnetState::Unknown {
            reason: "tailscale status gave no BackendState".to_owned(),
        };
    };
    from_backend_state(backend_state, non_empty(status.get("AuthURL")), || {
        device(&status)
    })
}

/// A backend state - `ipn.State`'s words (ipn/backend.go:41-49), which `status --json` gives as
/// `BackendState` and the browser's node notifies - as the indicator's state; `device` is asked
/// for only when it is on the tailnet.
#[must_use]
pub fn from_backend_state(
    backend_state: &str,
    login_url: Option<String>,
    device: impl FnOnce() -> Device,
) -> TailnetState {
    match backend_state {
        "Running" => TailnetState::OnTailnet(device()),
        "Starting" | "NoState" => TailnetState::Starting,
        "NeedsLogin" => TailnetState::SignedOut { login_url },
        "NeedsMachineAuth" => TailnetState::NeedsApproval,
        "Stopped" => TailnetState::Stopped,
        // `InUseOtherUser`, which `status` never gives - another user's ask is refused before it
        // is answered (ipnserver/server.go:188-196) - or a state newer than this module: in the
        // CLI's own words for one it does not know (status.go:264-265).
        other => TailnetState::Unknown {
            reason: format!("unexpected state: {other}"),
        },
    }
}

/// The page's own node (crates/mp-transport/src/page.rs's `PageTailnet`, web/www/tailscale.js's
/// `tailscaleState`) as the indicator's state, by the page's word for where it is: `NotStarted`,
/// `Loading`, `NeedsLogin`, `NeedsApproval`, `Running` or `Stopped`; its name and addresses when
/// running, its sign-in link while it waits for one, and why it stopped.
#[must_use]
#[cfg_attr(not(target_family = "wasm"), allow(dead_code))] // the browser build's, and its tests'
pub fn from_page(
    state: &str,
    name: Option<&str>,
    addresses: &[String],
    login_url: Option<String>,
    reason: Option<String>,
) -> TailnetState {
    let device = || Device {
        name: name.unwrap_or_default().to_owned(),
        ipv4: addresses.iter().find_map(|address| address.parse().ok()),
        ipv6: addresses.iter().find_map(|address| address.parse().ok()),
        tailnet: None,
        magic_dns_suffix: None,
    };
    match state {
        "NotStarted" => TailnetState::NotStarted,
        "Loading" => TailnetState::Starting,
        "NeedsLogin" => TailnetState::SignedOut { login_url },
        "NeedsApproval" => TailnetState::NeedsApproval,
        "Running" => TailnetState::OnTailnet(device()),
        "Stopped" => TailnetState::NotRunning {
            reason: reason.unwrap_or_else(|| "the page's node stopped".to_owned()),
        },
        other => TailnetState::Unknown {
            reason: format!("unexpected page state: {other}"),
        },
    }
}

/// The host a link's target names - `tcp:my-pi:5760`'s `my-pi`, `udpcl:[fd7a::1]:14550`'s
/// `fd7a::1` - for the kinds that dial one (TCP and UDPCl); none for the others (a serial port,
/// UDP's listening port, a log played).
#[must_use]
pub fn link_host(target: &str) -> Option<&str> {
    let (kind, rest) = target.split_once(':')?;
    if !(kind.eq_ignore_ascii_case("tcp") || kind.eq_ignore_ascii_case("udpcl")) {
        return None;
    }
    let host = match rest.strip_prefix('[') {
        Some(bracketed) => bracketed.split_once(']')?.0,
        None => rest.rsplit_once(':').map_or(rest, |(host, _port)| host),
    };
    (!host.is_empty()).then_some(host)
}

/// This device, from the status's `Self` (`DNSName`, `HostName`, `TailscaleIPs`) and
/// `CurrentTailnet` (`Name`, `MagicDNSSuffix`; the top-level `MagicDNSSuffix`, its deprecated
/// twin, for a daemon too old to have it).
fn device(status: &Value) -> Device {
    let me = status.get("Self");
    let name = me
        .and_then(|me| non_empty(me.get("DNSName")))
        .and_then(|dns_name| first_label(&dns_name))
        .or_else(|| me.and_then(|me| non_empty(me.get("HostName"))))
        .unwrap_or_default();
    let addresses: Vec<IpAddr> = me
        .and_then(|me| me.get("TailscaleIPs"))
        .and_then(Value::as_array)
        .filter(|addresses| !addresses.is_empty())
        .or_else(|| status.get("TailscaleIPs").and_then(Value::as_array))
        .map(|addresses| {
            addresses
                .iter()
                .filter_map(Value::as_str)
                .filter_map(|address| address.parse::<IpAddr>().ok())
                .collect::<Vec<IpAddr>>()
        })
        .unwrap_or_default();
    let tailnet = status.get("CurrentTailnet");
    Device {
        name,
        ipv4: addresses.iter().find_map(|address| match address {
            IpAddr::V4(ipv4) => Some(*ipv4),
            IpAddr::V6(_) => None,
        }),
        ipv6: addresses.iter().find_map(|address| match address {
            IpAddr::V6(ipv6) => Some(*ipv6),
            IpAddr::V4(_) => None,
        }),
        tailnet: tailnet.and_then(|tailnet| non_empty(tailnet.get("Name"))),
        magic_dns_suffix: tailnet
            .and_then(|tailnet| non_empty(tailnet.get("MagicDNSSuffix")))
            .or_else(|| non_empty(status.get("MagicDNSSuffix"))),
    }
}

/// A JSON string that says something: Go writes an unset string as "".
fn non_empty(value: Option<&Value>) -> Option<String> {
    value
        .and_then(Value::as_str)
        .filter(|text| !text.is_empty())
        .map(str::to_owned)
}

/// A DNS name's first label: a MagicDNS name's machine name.
fn first_label(dns_name: &str) -> Option<String> {
    dns_name
        .split('.')
        .next()
        .filter(|label| !label.is_empty())
        .map(str::to_owned)
}

/// Whether `host` - as typed at "Enter host name/ip", without a port - is on a tailnet: an IPv4
/// address in 100.64.0.0/10 or an IPv6 address in fd7a:115c:a1e0::/48, Tailscale's ranges, or a
/// MagicDNS name, ending `.ts.net` with or without its final dot. A machine name on its own,
/// `my-pi`, is not judged one: any network's names look like that.
#[must_use]
pub fn is_tailnet_host(host: &str) -> bool {
    tailnet_host_kind(host).is_some()
}

/// What `host` is on a tailnet, in the status line's words: an address or a name; none when it
/// is neither. An IPv6 address may come in its URL brackets.
fn tailnet_host_kind(host: &str) -> Option<&'static str> {
    let host = host.trim();
    let bare = host
        .strip_prefix('[')
        .and_then(|inner| inner.strip_suffix(']'))
        .unwrap_or(host);
    if let Ok(address) = bare.parse::<IpAddr>() {
        return is_tailnet_address(address).then_some("address");
    }
    let name = host.strip_suffix('.').unwrap_or(host).to_ascii_lowercase();
    let machine = name.strip_suffix(".ts.net")?;
    (!machine.is_empty()).then_some("name")
}

/// Tailscale's address ranges: 100.64.0.0/10, the shared address space its IPv4 addresses come
/// from, and fd7a:115c:a1e0::/48.
fn is_tailnet_address(address: IpAddr) -> bool {
    match address {
        IpAddr::V4(ipv4) => {
            let [first, second, ..] = ipv4.octets();
            first == 100 && (64..=127).contains(&second)
        }
        IpAddr::V6(ipv6) => {
            let [first, second, third, ..] = ipv6.segments();
            first == 0xfd7a && second == 0x115c && third == 0xa1e0
        }
    }
}

/// The status line's words when a link to `host` failed and this computer is not on a tailnet:
/// "100.101.102.103 is a Tailscale address, and this computer is not on a tailnet (Tailscale:
/// not installed) - see HELP > Using Tailscale". None when `host` is not a tailnet's, when this
/// computer is on one, and while its state is not known: nothing is said that was not seen.
#[must_use]
pub fn failure_reason(host: &str, state: &TailnetState) -> Option<String> {
    let kind = tailnet_host_kind(host)?;
    state.is_off_tailnet().then(|| {
        format!(
            "{} is a Tailscale {kind}, and this computer is not on a tailnet ({}) - see \
             {HELP_PAGE}",
            host.trim(),
            state.text()
        )
    })
}

/// The systems told apart here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Platform {
    /// Linux.
    Linux,
    /// macOS.
    MacOs,
    /// Windows.
    Windows,
    /// Anything else: another Unix, or a web page.
    Other,
}

impl Platform {
    /// This build's system.
    #[must_use]
    pub const fn current() -> Self {
        if cfg!(target_os = "linux") {
            Self::Linux
        } else if cfg!(target_os = "macos") {
            Self::MacOs
        } else if cfg!(windows) {
            Self::Windows
        } else {
            Self::Other
        }
    }
}

/// Tailscale's download page for `platform`: the system's own on Tailscale's site
/// (tailscale.com/download links one each for macOS, Windows and Linux), the general one for
/// any other.
#[must_use]
pub const fn download_page(platform: Platform) -> &'static str {
    match platform {
        Platform::Linux => DOWNLOAD_PAGE_LINUX,
        Platform::MacOs => DOWNLOAD_PAGE_MAC,
        Platform::Windows => DOWNLOAD_PAGE_WINDOWS,
        Platform::Other => DOWNLOAD_PAGE,
    }
}

/// The first start's offer of Tailscale, for the welcome step to show - the owner's "offer it as
/// a link, not run an installer: open Tailscale's download page, or show the one-line Linux
/// command. Then the planner detects the sign-in." Nothing is installed or run from it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Offer {
    /// The step's words, before the link.
    pub text: &'static str,
    /// The page the link opens: Tailscale's download page for this system.
    pub page: &'static str,
    /// What to type in a terminal instead, a line each to copy: on Linux Tailscale's install
    /// script and `tailscale up` ([`LINUX_INSTALL`]); none on macOS, whose app signs in.
    pub commands: &'static [&'static str],
    /// The step's words after them.
    pub after: &'static str,
}

/// [`Offer::text`] on Linux.
const LINUX_OFFER_TEXT: &str = "Tailscale builds a private network of your own devices, called \
     your tailnet, that works across the internet: the planner reaches a vehicle on it wherever \
     each of them is. It is not installed on this computer. To install it, open Tailscale's \
     download page, or run these two lines in a terminal and sign in at the link the second one \
     prints:";

/// [`Offer::text`] on macOS.
const MAC_OFFER_TEXT: &str = "Tailscale builds a private network of your own devices, called \
     your tailnet, that works across the internet: the planner reaches a vehicle on it wherever \
     each of them is. It is not installed on this computer. To install it, open Tailscale's \
     download page, install the app, open it and sign in.";

/// [`Offer::after`].
const OFFER_AFTER: &str =
    "The planner sees the sign-in by itself. HELP > Using Tailscale says more.";

/// The first start's offer on `platform` in `state`: only when Tailscale is not installed, and
/// only on Linux and macOS - the owner's "offer Tailscale at first start, if its not already
/// installed (linux & osx)". None while the state is not yet known ([`TailnetState::Unchecked`]):
/// the welcome step asks again once the poller has answered.
#[must_use]
pub fn offer(platform: Platform, state: &TailnetState) -> Option<Offer> {
    if !matches!(state, TailnetState::NotInstalled) {
        return None;
    }
    match platform {
        Platform::Linux => Some(Offer {
            text: LINUX_OFFER_TEXT,
            page: download_page(platform),
            commands: LINUX_INSTALL,
            after: OFFER_AFTER,
        }),
        Platform::MacOs => Some(Offer {
            text: MAC_OFFER_TEXT,
            page: download_page(platform),
            commands: &[],
            after: OFFER_AFTER,
        }),
        Platform::Windows | Platform::Other => None,
    }
}

/// [`offer`] on this computer.
#[must_use]
pub fn first_start_offer(state: &TailnetState) -> Option<Offer> {
    offer(Platform::current(), state)
}

/// What a GUI script can see: `tailscale.state` ([`TailnetState::word`]), `tailscale.name` and
/// `tailscale.address` - this device's machine name and the indicator's address, `none` off a
/// tailnet.
pub fn record_facts(state: &TailnetState) {
    if !crate::facts::enabled() {
        return;
    }
    for (key, value) in facts(state) {
        crate::facts::record(key, value);
    }
}

/// The facts [`record_facts`] records.
fn facts(state: &TailnetState) -> [(&'static str, String); 3] {
    let device = state.device();
    [
        ("tailscale.state", state.word().to_owned()),
        (
            "tailscale.name",
            device.map_or_else(|| "none".to_owned(), |device| device.name.clone()),
        ),
        (
            "tailscale.address",
            device
                .and_then(Device::address)
                .map_or_else(|| "none".to_owned(), |address| address.to_string()),
        ),
    ]
}

/// The indicator's source: on the desktop a thread asking the CLI, at once and then every
/// [`POLL_INTERVAL`] and at each [`Poller::check_now`], its newest answer taken once a frame by
/// [`Poller::tick`]; in a web page nothing at all, its state staying [`TailnetState::Unchecked`].
#[derive(Debug)]
pub struct Poller {
    /// The newest answer taken.
    state: TailnetState,
    /// The thread's ends; none when it could not be started.
    #[cfg(not(target_family = "wasm"))]
    worker: Option<cli::Worker>,
}

impl Poller {
    /// Starts asking. In a web page nothing is started.
    #[must_use]
    pub fn start() -> Self {
        #[cfg(not(target_family = "wasm"))]
        {
            Self::start_with(cli::check, POLL_INTERVAL)
        }
        #[cfg(target_family = "wasm")]
        {
            Self {
                state: TailnetState::Unchecked,
            }
        }
    }

    /// Starts the thread, `check` its look and `interval` its pace: [`cli::check`] every
    /// [`POLL_INTERVAL`] for the planner, another for a test.
    #[cfg(not(target_family = "wasm"))]
    fn start_with(check: impl Fn() -> TailnetState + Send + 'static, interval: Duration) -> Self {
        let (ask, asked) = std::sync::mpsc::channel::<()>();
        let (answer, answers) = std::sync::mpsc::channel::<TailnetState>();
        let spawned = wasm_thread::Builder::new()
            .name("mp-tailscale".to_owned())
            .spawn(move || cli::poll(&check, interval, &asked, &answer));
        match spawned {
            Ok(_) => Self {
                state: TailnetState::Unchecked,
                worker: Some(cli::Worker { ask, answers }),
            },
            Err(err) => Self {
                state: TailnetState::Unknown {
                    reason: format!("no thread to ask Tailscale on: {err}"),
                },
                worker: None,
            },
        }
    }

    /// Asks for a look now - after a link to a tailnet address failed, say - its answer taken by
    /// a later [`Self::tick`]. Nothing in a web page.
    pub fn check_now(&self) {
        #[cfg(not(target_family = "wasm"))]
        {
            if let Some(worker) = &self.worker {
                let _ = worker.ask.send(());
            }
        }
    }

    /// Takes the thread's newest answer, without waiting: whether the state changed.
    pub fn tick(&mut self) -> bool {
        #[cfg(not(target_family = "wasm"))]
        {
            let Some(worker) = &self.worker else {
                return false;
            };
            let mut changed = false;
            while let Ok(state) = worker.answers.try_recv() {
                if state != self.state {
                    self.state = state;
                    changed = true;
                }
            }
            changed
        }
        #[cfg(target_family = "wasm")]
        {
            false
        }
    }

    /// The newest answer taken.
    #[must_use]
    pub const fn state(&self) -> &TailnetState {
        &self.state
    }
}

/// The desktop's half: the CLI found, run and read, on the poller's thread.
#[cfg(not(target_family = "wasm"))]
mod cli {
    use std::ffi::OsStr;
    use std::io::Read as _;
    use std::path::{Path, PathBuf};
    use std::process::{Command, Stdio};
    use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender, TryRecvError, channel};
    use std::time::Duration;

    use mp_os::RecvTimeout as _;
    use mp_os::fs::FsExt as _;
    use web_time::Instant;

    use super::{CLI_ENV, CLI_TIMEOUT, NO_CLI, Platform, STATUS_ARGS, TailnetState, parse_status};

    /// Where Linux's packages put the CLI, after `PATH`: Tailscale's own packages in
    /// `/usr/bin`, a build from source in `/usr/local/bin`.
    const LINUX_CLIS: [&str; 2] = ["/usr/bin/tailscale", "/usr/local/bin/tailscale"];

    /// The macOS app's own executable, which is the CLI when given arguments: "Both macsys and
    /// app store versions can run CLI executable with suffix /Contents/MacOS/Tailscale"
    /// (version/prop.go:143-144; safesocket/safesocket_darwin.go:65-68, "when the CLI is invoked
    /// from the Tailscale.app GUI"), and Tailscale's CLI page has the alias to it.
    const MAC_APP_CLI: &str = "/Applications/Tailscale.app/Contents/MacOS/Tailscale";

    /// Where macOS has the CLI after `PATH`, which a planner started from the Finder has little
    /// of: the app's own; the launcher the standalone app's "CLI integration" installs in
    /// `/usr/local/bin`, where Homebrew's on Intel is too; Homebrew's on Apple silicon.
    const MAC_CLIS: [&str; 3] = [
        MAC_APP_CLI,
        "/usr/local/bin/tailscale",
        "/opt/homebrew/bin/tailscale",
    ];

    /// Where the Windows installer puts the CLI, when `ProgramFiles` does not say.
    const WINDOWS_CLI: &str = r"C:\Program Files\Tailscale\tailscale.exe";

    /// The words of every error the CLI gives when the daemon does not answer: diag.go:32-71's,
    /// and local.go:206's when the CLI is built without diag.go.
    const NOT_ANSWERING: &str = "failed to connect to local";

    /// The most of one run's output that is read: `status --json` without the peers is a few
    /// kilobytes, and a program that writes on and on is cut off here and killed at its time.
    const MOST_OUTPUT: u64 = 1 << 20;

    /// How often a run is looked at while it lasts.
    const WAIT_STEP: Duration = Duration::from_millis(10);

    /// How long a run's output is waited for once it has ended: its pipes close as it does,
    /// unless something it started holds them still.
    const OUTPUT_GRACE: Duration = Duration::from_millis(500);

    /// Where the CLI is looked for, in turn: each folder on `path` - an absolute one: a relative
    /// one would be the planner's working folder's - then where each system's installers put it;
    /// on Windows `program_files`' `Tailscale` folder, then the usual one.
    pub(super) fn candidates(
        platform: Platform,
        path: Option<&OsStr>,
        program_files: Option<&OsStr>,
    ) -> Vec<PathBuf> {
        let program = if platform == Platform::Windows {
            "tailscale.exe"
        } else {
            "tailscale"
        };
        let mut candidates: Vec<PathBuf> = path
            .map(|path| {
                mp_os::split_paths(path)
                    .filter(|folder| folder.is_absolute())
                    .map(|folder| folder.join(program))
                    .collect::<Vec<PathBuf>>()
            })
            .unwrap_or_default();
        match platform {
            Platform::Linux => candidates.extend(LINUX_CLIS.map(PathBuf::from)),
            Platform::MacOs => candidates.extend(MAC_CLIS.map(PathBuf::from)),
            Platform::Windows => {
                if let Some(folder) = program_files.filter(|folder| !folder.is_empty()) {
                    candidates.push(Path::new(folder).join("Tailscale").join("tailscale.exe"));
                }
                candidates.push(PathBuf::from(WINDOWS_CLI));
            }
            Platform::Other => {}
        }
        candidates
    }

    /// The CLI to ask: `overridden`'s ([`CLI_ENV`]) when it is set - none for [`NO_CLI`] - else
    /// the first of [`candidates`] that `is_file` finds.
    pub(super) fn find_cli_from(
        overridden: Option<&OsStr>,
        platform: Platform,
        path: Option<&OsStr>,
        program_files: Option<&OsStr>,
        is_file: impl Fn(&Path) -> bool,
    ) -> Option<PathBuf> {
        match overridden.filter(|value| !value.is_empty()) {
            Some(value) if value.to_str() == Some(NO_CLI) => None,
            Some(value) => Some(PathBuf::from(value)),
            None => candidates(platform, path, program_files)
                .into_iter()
                .find(|candidate| is_file(candidate.as_path())),
        }
    }

    /// [`find_cli_from`] on this computer, its environment read afresh at each look, so a
    /// Tailscale installed while the planner runs is found at the next.
    pub(super) fn find_cli() -> Option<PathBuf> {
        find_cli_from(
            std::env::var_os(CLI_ENV).as_deref(),
            Platform::current(),
            std::env::var_os("PATH").as_deref(),
            std::env::var_os("ProgramFiles").as_deref(),
            |candidate| candidate.os_is_file(),
        )
    }

    /// One run of the CLI: its exit code - none when a signal ended it - and what it wrote.
    #[derive(Debug, Clone, PartialEq, Eq, Default)]
    pub(super) struct Ran {
        /// Its exit code.
        pub(super) code: Option<i32>,
        /// What it wrote to its standard output.
        pub(super) stdout: String,
        /// What it wrote to its standard error.
        pub(super) stderr: String,
    }

    /// Why a run has no answer.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub(super) enum RunError {
        /// The program is not there.
        NotFound,
        /// It had not ended when its time was up, and was killed.
        TimedOut,
        /// It could not be started or waited for: the system's words.
        Failed(String),
    }

    /// `tailscale status --json --peers=false`, its input closed and its output piped; on
    /// Windows with no console window, the CLI being a console program.
    pub(super) fn status_command(cli: &Path) -> Command {
        let mut command = Command::new(cli);
        command
            .args(STATUS_ARGS)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        #[cfg(windows)]
        {
            use std::os::windows::process::CommandExt as _;
            /// `CREATE_NO_WINDOW`.
            const CREATE_NO_WINDOW: u32 = 0x0800_0000;
            command.creation_flags(CREATE_NO_WINDOW);
        }
        command
    }

    /// Runs `command` to its end, or kills it at `timeout`: its output read as it comes, on
    /// threads of their own, so a full pipe never holds it up.
    ///
    /// # Errors
    /// The program not there, not ended in time, or not started: a [`RunError`].
    pub(super) fn run(mut command: Command, timeout: Duration) -> Result<Ran, RunError> {
        let mut child = command.spawn().map_err(|err| {
            if err.kind() == std::io::ErrorKind::NotFound {
                RunError::NotFound
            } else {
                RunError::Failed(err.to_string())
            }
        })?;
        let stdout = child.stdout.take().map(read_all);
        let stderr = child.stderr.take().map(read_all);
        let deadline = Instant::now() + timeout;
        let ended = loop {
            match child.try_wait() {
                Ok(Some(_)) => break Ok(()),
                Ok(None) if Instant::now() < deadline => wasm_thread::sleep(WAIT_STEP),
                Ok(None) => break Err(RunError::TimedOut),
                Err(err) => break Err(RunError::Failed(err.to_string())),
            }
        };
        if ended.is_err() {
            let _ = child.kill();
        }
        // Reaped on every path: after an end `try_wait` saw, `wait` gives its status at once.
        let status = child.wait();
        ended?;
        let status = status.map_err(|err| RunError::Failed(err.to_string()))?;
        Ok(Ran {
            code: status.code(),
            stdout: text(stdout),
            stderr: text(stderr),
        })
    }

    /// Reads `pipe` to its end - at most [`MOST_OUTPUT`] - on a thread of its own, the bytes
    /// handed over when it closes.
    fn read_all(pipe: impl std::io::Read + Send + 'static) -> Receiver<Vec<u8>> {
        let (sender, receiver) = channel();
        let _ = wasm_thread::Builder::new()
            .name("mp-tailscale-pipe".to_owned())
            .spawn(move || {
                let mut bytes = Vec::new();
                let _ = pipe.take(MOST_OUTPUT).read_to_end(&mut bytes);
                let _ = sender.send(bytes);
            });
        receiver
    }

    /// A pipe's bytes as text, once its reader has them all.
    fn text(reader: Option<Receiver<Vec<u8>>>) -> String {
        reader
            .and_then(|reader| reader.os_recv_timeout(OUTPUT_GRACE).ok())
            .map(|bytes| String::from_utf8_lossy(&bytes).into_owned())
            .unwrap_or_default()
    }

    /// What a run says: the status, whatever its exit code - `status --json` prints it and ends
    /// well in every backend state (status.go:88-101); else, from the CLI's error on stderr and
    /// its exit 1 (cmd/tailscale/tailscale.go:22-25), a daemon that does not answer
    /// ([`NOT_ANSWERING`]) or something this module cannot read - access refused, a flag the CLI
    /// does not know - in its own words.
    pub(super) fn interpret(ran: &Ran) -> TailnetState {
        if let Some(json) = ran.stdout.find('{').and_then(|at| ran.stdout.get(at..)) {
            return parse_status(json);
        }
        let said: Vec<&str> = ran
            .stderr
            .lines()
            .map(str::trim)
            .filter(|line| !line.is_empty())
            .collect();
        if let Some(line) = said
            .iter()
            .copied()
            .find(|line| line.to_ascii_lowercase().contains(NOT_ANSWERING))
        {
            return TailnetState::NotRunning {
                reason: line.to_owned(),
            };
        }
        let reason = match (said.first().copied(), ran.code) {
            (Some(line), _) => line.to_owned(),
            (None, Some(code)) => format!("tailscale status ended {code}, saying nothing"),
            (None, None) => "tailscale status was ended by a signal".to_owned(),
        };
        TailnetState::Unknown { reason }
    }

    /// A run's outcome as a state: a CLI that is not there is Tailscale not installed.
    pub(super) fn answer(ran: Result<Ran, RunError>) -> TailnetState {
        match ran {
            Ok(ran) => interpret(&ran),
            Err(RunError::NotFound) => TailnetState::NotInstalled,
            Err(RunError::TimedOut) => TailnetState::Unknown {
                reason: "tailscale status gave no answer in time".to_owned(),
            },
            Err(RunError::Failed(why)) => TailnetState::Unknown {
                reason: format!("tailscale status could not be run: {why}"),
            },
        }
    }

    /// One look: the CLI found and asked, its answer read; no CLI is no Tailscale.
    pub(super) fn check() -> TailnetState {
        find_cli().map_or(TailnetState::NotInstalled, |cli| {
            answer(run(status_command(&cli), CLI_TIMEOUT))
        })
    }

    /// The poller's ends of its thread.
    #[derive(Debug)]
    pub(super) struct Worker {
        /// A look asked for now; dropped, the thread's end.
        pub(super) ask: Sender<()>,
        /// The thread's answers, each a state unlike the one before it.
        pub(super) answers: Receiver<TailnetState>,
    }

    /// The poller's thread: `check` at once, then each `interval` and at each ask, an answer
    /// handed over only when it differs from the last - with a wake, so the window draws it at
    /// its next look rather than at its floor - until the poller is gone.
    pub(super) fn poll(
        check: &dyn Fn() -> TailnetState,
        interval: Duration,
        asked: &Receiver<()>,
        answer: &Sender<TailnetState>,
    ) {
        let mut last = None;
        loop {
            // Every ask so far is answered by the look that follows.
            loop {
                match asked.try_recv() {
                    Ok(()) => {}
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => return,
                }
            }
            let state = check();
            if last.as_ref() != Some(&state) {
                if answer.send(state.clone()).is_err() {
                    return;
                }
                mp_os::wake();
                last = Some(state);
            }
            match asked.os_recv_timeout(interval) {
                Ok(()) | Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return,
            }
        }
    }
}

// --- the planner's side: the indicator, the first start's offer, the status line's reason ---

/// The setting that says the first start's offer has been made - or was not needed, Tailscale
/// being installed - so it is not made again at a start; the indicator still opens it.
pub(crate) const OFFERED_KEY: &str = "tailscale_offered";

/// The offer box's id and its buttons'.
const OFFER_BOX: &str = "tailscale-offer";
const OFFER_OPEN: &str = "tailscale-offer-open";
const OFFER_CLOSE: &str = "tailscale-offer-close";

impl crate::MissionPlanner {
    /// Whether Tailscale is enabled in the PLUGINS tab: the owner's native row
    /// (`plugin_manager::TAILSCALE`), on until it is unticked.
    pub(crate) fn tailnet_enabled(&self) -> bool {
        crate::plugins_ui::native_enabled(&self.persisted, &crate::plugin_manager::TAILSCALE)
    }

    /// This computer's Tailscale as the indicator shows it: the official client's state on the
    /// desktop, the page's own node in a web page.
    pub(crate) fn tailnet_state(&self) -> TailnetState {
        #[cfg(target_family = "wasm")]
        {
            use mp_transport::page::PageTailnetState;
            let Some(page) = mp_transport::page::tailnet() else {
                return TailnetState::Unchecked;
            };
            let word = match page.state {
                PageTailnetState::NotStarted => "NotStarted",
                PageTailnetState::Loading => "Loading",
                PageTailnetState::NeedsLogin => "NeedsLogin",
                PageTailnetState::NeedsApproval => "NeedsApproval",
                PageTailnetState::Running => "Running",
                PageTailnetState::Stopped => "Stopped",
            };
            from_page(
                word,
                page.name.as_deref(),
                &page.addresses,
                page.login_url,
                page.reason,
            )
        }
        #[cfg(not(target_family = "wasm"))]
        {
            self.tailnet
                .as_ref()
                .map_or(TailnetState::Unchecked, |poller| poller.state().clone())
        }
    }

    /// Once a frame: the desktop's poller started while Tailscale is enabled and let go when it
    /// is not; the first start's offer, once the state is known; and a link to a tailnet host
    /// that failed explained on the status line, once for each target, while this computer is
    /// off its tailnet (the owner's plan of 2026-10-14).
    pub(crate) fn tailnet_tick(&mut self, view: &crate::telemetry::TelemetryView) {
        if !self.tailnet_enabled() {
            self.tailnet = None;
            self.tailnet_offer = None;
            return;
        }
        #[cfg(not(target_family = "wasm"))]
        {
            self.tailnet.get_or_insert_with(Poller::start).tick();
        }
        let state = self.tailnet_state();
        if !matches!(state, TailnetState::Unchecked) && self.persisted.get(OFFERED_KEY).is_none() {
            self.tailnet_offer = first_start_offer(&state);
            self.persisted.set(OFFERED_KEY, "True");
        }
        let failed = self.telemetry.error().is_some()
            || (view.reconnecting && view.reconnect_error.is_some());
        if !failed {
            self.tailnet_explained = None;
            return;
        }
        if self.tailnet_explained.as_deref() == Some(view.target.as_str()) {
            return;
        }
        let Some(host) = link_host(&view.target).filter(|host| is_tailnet_host(host)) else {
            return;
        };
        // Said once a known state has come: unchecked or unknown says nothing either way.
        if matches!(state, TailnetState::Unchecked | TailnetState::Unknown { .. }) {
            return;
        }
        self.tailnet_explained = Some(view.target.clone());
        // Looked at again at once: the tailnet may be what just went.
        if let Some(poller) = &self.tailnet {
            poller.check_now();
        }
        if let Some(reason) = failure_reason(host, &state) {
            self.file_status = Some(reason);
        }
    }

    /// The indicator, in the connection area after the link's state: a dot - the OK colour on
    /// the tailnet - and the state's words, the longer account on hover. A click does what the
    /// state asks for: the offer when Tailscale is not installed, the sign-in link when there is
    /// one, the page's node started in a web page; else the account on the status line.
    pub(crate) fn tailnet_indicator(
        &self,
        cx: &mut gpui::Context<crate::MissionPlanner>,
    ) -> Option<gpui::AnyElement> {
        use gpui::{
            AppContext as _, InteractiveElement as _, IntoElement as _, ParentElement as _,
            StatefulInteractiveElement as _, Styled as _, div, rgb,
        };
        if !self.tailnet_enabled() {
            return None;
        }
        let state = self.tailnet_state();
        let colour = if state.is_on_tailnet() {
            crate::theme::OK
        } else if matches!(
            state,
            TailnetState::Unchecked | TailnetState::Starting | TailnetState::NotStarted
        ) {
            crate::theme::DIM
        } else {
            crate::theme::WARN
        };
        let details = gpui::SharedString::from(state.details());
        Some(
            crate::probe::measured("tailscale-indicator", div())
                .id("tailscale-indicator")
                .flex()
                .items_center()
                .gap_1()
                .cursor_pointer()
                .child(div().size_2().rounded_full().bg(rgb(colour)))
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(crate::theme::DIM))
                        .child(state.text()),
                )
                .tooltip(move |_window, cx| -> gpui::AnyView {
                    cx.new(|_| crate::config::rover_tuning::Tip(details.clone()))
                        .into()
                })
                .on_click(cx.listener(|this, _event, _window, cx| {
                    this.tailnet_clicked();
                    cx.notify();
                }))
                .into_any_element(),
        )
    }

    /// The indicator clicked.
    fn tailnet_clicked(&mut self) {
        let state = self.tailnet_state();
        match &state {
            TailnetState::NotInstalled => match offer(Platform::current(), &state) {
                Some(offer) => self.tailnet_offer = Some(offer),
                None => self.tailnet_open(download_page(Platform::current())),
            },
            #[cfg(not(target_family = "wasm"))]
            TailnetState::SignedOut {
                login_url: Some(url),
            } => {
                let url = url.clone();
                self.tailnet_open(&url);
            }
            #[cfg(target_family = "wasm")]
            TailnetState::NotStarted => mp_transport::page::start_tailnet(),
            #[cfg(target_family = "wasm")]
            TailnetState::SignedOut { .. } => {
                self.file_status = Some(
                    "sign in to Tailscale with the link at the page's bottom right".to_owned(),
                );
            }
            _ => self.file_status = Some(state.details()),
        }
    }

    /// A page handed to the desktop's browser, kept for the facts; a failure on the status line.
    fn tailnet_open(&mut self, url: &str) {
        match crate::scripts_tab::open_with_shell(std::path::Path::new(url)) {
            Ok(()) => self.tailnet_opened = Some(url.to_owned()),
            Err(why) => self.file_status = Some(format!("could not open {url}: {why}")),
        }
    }

    /// The offer, over the window while it shows: its words, the lines to type on Linux, Open
    /// the download page and Close. Never an installer run (the owner's "offer it as a link, not
    /// run an installer").
    pub(crate) fn tailnet_offer_box(
        &self,
        window: &gpui::Window,
        cx: &mut gpui::Context<crate::MissionPlanner>,
    ) -> Option<gpui::AnyElement> {
        let offer = self.tailnet_offer?;
        let mut text = offer.text.to_owned();
        for command in offer.commands {
            text.push_str("\n\n    ");
            text.push_str(command);
        }
        text.push_str("\n\n");
        text.push_str(offer.after);
        let page = offer.page;
        let buttons = vec![
            crate::ui::action(
                OFFER_OPEN,
                "Open the download page",
                crate::theme::ACCENT,
                true,
                cx.listener(move |this, _event: &(), _window, cx| {
                    this.tailnet_open(page);
                    cx.notify();
                }),
            ),
            crate::ui::action(
                OFFER_CLOSE,
                "Close",
                crate::theme::DIM,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.tailnet_offer = None;
                    cx.notify();
                }),
            ),
        ];
        Some(crate::config::servo_output::modal(
            OFFER_BOX, "Tailscale", &text, false, buttons, window,
        ))
    }

    /// The facts: the state's (`record_facts`), whether the indicator shows (`tailscale.shown`),
    /// its words (`tailscale.text`), the offer (`tailscale.offer`) and the last page handed to the
    /// browser (`tailscale.opened`).
    pub(crate) fn tailnet_record_facts(&self) {
        let enabled = self.tailnet_enabled();
        crate::facts::record("tailscale.shown", enabled);
        if enabled {
            let state = self.tailnet_state();
            record_facts(&state);
            crate::facts::record("tailscale.text", state.text());
        } else {
            crate::facts::record("tailscale.state", "off");
        }
        crate::facts::record(
            "tailscale.offer",
            if self.tailnet_offer.is_some() { "shown" } else { "none" },
        );
        crate::facts::record(
            "tailscale.opened",
            self.tailnet_opened.as_deref().unwrap_or("none"),
        );
    }
}

// The tests run on the desktop, where the CLI is - inside a plain `#[cfg(test)]`, which is what
// clippy reads as test code.
#[cfg(test)]
#[cfg(not(target_family = "wasm"))]
mod tests {
    use super::cli::{RunError, answer, candidates, find_cli_from, interpret, run, status_command};
    use super::*;
    use std::ffi::{OsStr, OsString};
    use std::path::{Path, PathBuf};
    use std::process::Command;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicUsize, Ordering};

    /// `tailscale status --json --peers=false` on a computer on a tailnet, as Tailscale 1.104
    /// prints it: `ipnstate.Status` through `json.MarshalIndent(st, "", "  ")`, field by field in
    /// the Go structs' order, `Peer` null for `--peers=false`.
    const RUNNING: &str = r#"{
  "Version": "1.104.0-t0123456789-g0123456789",
  "TUN": true,
  "BackendState": "Running",
  "HaveNodeKey": true,
  "AuthURL": "",
  "TailscaleIPs": [
    "100.101.102.103",
    "fd7a:115c:a1e0::1234:5678"
  ],
  "Self": {
    "ID": "nTkGsU6fz711CNTRL",
    "NodeID": 1234567890123456,
    "PublicKey": "nodekey:5f2b0c7e1d9a4b3c8e6f0a1b2c3d4e5f60718293a4b5c6d7e8f90a1b2c3d4e5f",
    "HostName": "buzz-laptop",
    "DNSName": "buzz-laptop.tail1234.ts.net.",
    "OS": "linux",
    "UserID": 9876543210,
    "TailscaleIPs": [
      "100.101.102.103",
      "fd7a:115c:a1e0::1234:5678"
    ],
    "AllowedIPs": [
      "100.101.102.103/32",
      "fd7a:115c:a1e0::1234:5678/128"
    ],
    "Addrs": [
      "203.0.113.7:41641",
      "192.168.1.20:41641"
    ],
    "CurAddr": "",
    "Relay": "syd",
    "PeerRelay": "",
    "RxBytes": 0,
    "TxBytes": 0,
    "Created": "2026-09-01T02:03:04.123456789Z",
    "LastWrite": "0001-01-01T00:00:00Z",
    "LastSeen": "0001-01-01T00:00:00Z",
    "LastHandshake": "0001-01-01T00:00:00Z",
    "Online": true,
    "ExitNode": false,
    "ExitNodeOption": false,
    "Active": false,
    "PeerAPIURL": [
      "http://100.101.102.103:45678",
      "http://[fd7a:115c:a1e0::1234:5678]:45678"
    ],
    "TaildropTarget": 0,
    "NoFileSharingReason": "",
    "Capabilities": [
      "HTTPS://TAILSCALE.COM/s/DEPRECATED-NODE-CAPS#see-https://github.com/tailscale/tailscale/issues/11508",
      "https://tailscale.com/cap/file-sharing"
    ],
    "CapMap": {
      "https://tailscale.com/cap/file-sharing": null
    },
    "InNetworkMap": true,
    "InMagicSock": false,
    "InEngine": false,
    "KeyExpiry": "2027-03-01T02:03:04Z"
  },
  "Health": null,
  "MagicDNSSuffix": "tail1234.ts.net",
  "CurrentTailnet": {
    "Name": "pilot@example.com",
    "StableID": "TfAkE1CNTRL",
    "MagicDNSSuffix": "tail1234.ts.net",
    "MagicDNSEnabled": true
  },
  "CertDomains": [
    "buzz-laptop.tail1234.ts.net"
  ],
  "ExtraRecords": null,
  "Peer": null,
  "User": {
    "9876543210": {
      "ID": 9876543210,
      "LoginName": "pilot@example.com",
      "DisplayName": "A Pilot"
    }
  },
  "ClientVersion": null
}"#;

    /// The same, signed out: no network map, so no addresses, no tailnet, and a `Self` with
    /// only the computer's host name and system (local.go, `updateStatusLocked`).
    const SIGNED_OUT: &str = r#"{
  "Version": "1.104.0-t0123456789-g0123456789",
  "TUN": true,
  "BackendState": "NeedsLogin",
  "AuthURL": "",
  "TailscaleIPs": null,
  "Self": {
    "ID": "",
    "NodeID": 0,
    "PublicKey": "nodekey:0000000000000000000000000000000000000000000000000000000000000000",
    "HostName": "buzz-laptop",
    "DNSName": "",
    "OS": "linux",
    "UserID": 0,
    "TailscaleIPs": null,
    "Addrs": null,
    "CurAddr": "",
    "Relay": "",
    "PeerRelay": "",
    "RxBytes": 0,
    "TxBytes": 0,
    "Created": "0001-01-01T00:00:00Z",
    "LastWrite": "0001-01-01T00:00:00Z",
    "LastSeen": "0001-01-01T00:00:00Z",
    "LastHandshake": "0001-01-01T00:00:00Z",
    "Online": false,
    "ExitNode": false,
    "ExitNodeOption": false,
    "Active": false,
    "PeerAPIURL": null,
    "TaildropTarget": 0,
    "NoFileSharingReason": "",
    "InNetworkMap": false,
    "InMagicSock": false,
    "InEngine": false
  },
  "Health": [
    "You are logged out."
  ],
  "MagicDNSSuffix": "",
  "CurrentTailnet": null,
  "CertDomains": null,
  "ExtraRecords": null,
  "Peer": null,
  "User": null,
  "ClientVersion": null
}"#;

    /// A sign-in's link, as Tailscale's coordination server gives one.
    const LOGIN_URL: &str = "https://login.tailscale.com/a/1a2b3c4d5e6f7";

    /// `json` with its backend state `state`.
    fn in_state(json: &str, state: &str) -> String {
        let key = "\"BackendState\": \"";
        let at = json.find(key).expect("a backend state") + key.len();
        let end = at + json[at..].find('"').expect("its closing quote");
        format!("{}{state}{}", &json[..at], &json[end..])
    }

    /// The device RUNNING and the fake CLI's `running` describe.
    fn buzz_laptop() -> Device {
        Device {
            name: "buzz-laptop".to_owned(),
            ipv4: Some(Ipv4Addr::new(100, 101, 102, 103)),
            ipv6: Some("fd7a:115c:a1e0::1234:5678".parse().expect("an IPv6 address")),
            tailnet: Some("pilot@example.com".to_owned()),
            magic_dns_suffix: Some("tail1234.ts.net".to_owned()),
        }
    }

    #[test]
    fn running_is_on_the_tailnet_as_its_machine_name_and_addresses() {
        let state = parse_status(RUNNING);
        assert_eq!(state, TailnetState::OnTailnet(buzz_laptop()));
        assert!(state.is_on_tailnet());
        assert_eq!(state.device(), Some(&buzz_laptop()));
        assert_eq!(
            state.text(),
            "Tailscale: on tailnet as buzz-laptop, 100.101.102.103"
        );
        assert_eq!(
            state.details(),
            "On the tailnet tail1234.ts.net as buzz-laptop: 100.101.102.103, \
             fd7a:115c:a1e0::1234:5678"
        );
    }

    #[test]
    fn each_backend_state_is_the_indicators() {
        // Signed in, with a network map: its addresses are there, and only Running is on it.
        assert_eq!(
            parse_status(&in_state(RUNNING, "Starting")),
            TailnetState::Starting
        );
        assert_eq!(
            parse_status(&in_state(RUNNING, "NeedsMachineAuth")),
            TailnetState::NeedsApproval
        );
        // `tailscale down` keeps the last network map (local.go, `enterStateLocked`): the
        // addresses are still in the status, and it is off.
        assert_eq!(
            parse_status(&in_state(RUNNING, "Stopped")),
            TailnetState::Stopped
        );
        // Signed out, with none.
        assert_eq!(
            parse_status(SIGNED_OUT),
            TailnetState::SignedOut { login_url: None }
        );
        assert_eq!(
            parse_status(&SIGNED_OUT.replace(
                "\"AuthURL\": \"\"",
                &format!("\"AuthURL\": \"{LOGIN_URL}\"")
            )),
            TailnetState::SignedOut {
                login_url: Some(LOGIN_URL.to_owned())
            }
        );
        assert_eq!(
            parse_status(&in_state(SIGNED_OUT, "NoState")),
            TailnetState::Starting
        );
        // One `status` never gives, and one from a newer Tailscale: the CLI's words for it.
        assert_eq!(
            parse_status(&in_state(SIGNED_OUT, "InUseOtherUser")),
            TailnetState::Unknown {
                reason: "unexpected state: InUseOtherUser".to_owned()
            }
        );
    }

    #[test]
    fn what_is_not_a_status_is_unknown() {
        for json in ["", "Logged out.", "{\"Version\": \"1.104.0\"}", "[]", "{"] {
            assert_eq!(parse_status(json).word(), "unknown", "{json}");
        }
    }

    #[test]
    fn the_device_falls_back_to_its_host_name_and_the_older_fields() {
        // No MagicDNS name, no addresses on `Self` but the status's own, no `CurrentTailnet` but
        // the deprecated top-level suffix; the IPv6 listed first.
        let status = serde_json::json!({
            "BackendState": "Running",
            "TailscaleIPs": ["fd7a:115c:a1e0::9", "100.64.0.9"],
            "Self": { "HostName": "my-pi", "DNSName": "", "TailscaleIPs": null },
            "MagicDNSSuffix": "tail9.ts.net",
            "CurrentTailnet": null
        })
        .to_string();
        let device = Device {
            name: "my-pi".to_owned(),
            ipv4: Some(Ipv4Addr::new(100, 64, 0, 9)),
            ipv6: Some("fd7a:115c:a1e0::9".parse().expect("the test's own value")),
            tailnet: None,
            magic_dns_suffix: Some("tail9.ts.net".to_owned()),
        };
        assert_eq!(parse_status(&status), TailnetState::OnTailnet(device));
        // Only an IPv6: the indicator shows it.
        let only_ipv6 = TailnetState::OnTailnet(Device {
            name: "my-pi".to_owned(),
            ipv6: Some("fd7a:115c:a1e0::9".parse().expect("the test's own value")),
            ..Device::default()
        });
        assert_eq!(
            only_ipv6.text(),
            "Tailscale: on tailnet as my-pi, fd7a:115c:a1e0::9"
        );
        // Nothing known of it at all.
        assert_eq!(
            TailnetState::OnTailnet(Device::default()).text(),
            "Tailscale: on tailnet"
        );
    }

    #[test]
    fn a_daemon_that_does_not_answer_is_not_running_in_the_clis_words() {
        // diag.go:67 on Linux with systemd, :61 on macOS, :59 on Windows, and local.go:206's when
        // the CLI has no diag.go: each on stderr, with exit 1 and nothing on stdout.
        for said in [
            "failed to connect to local tailscaled; it doesn't appear to be running (sudo \
             systemctl start tailscaled ?)",
            "failed to connect to local Tailscale service; is Tailscale running?",
            "failed to connect to local tailscaled process; is the Tailscale service running?",
            "Failed to connect to local Tailscale daemon for /localapi/v0/status; not running? \
             Error: dial unix /var/run/tailscale/tailscaled.sock: connect: no such file or \
             directory",
        ] {
            let ran = cli::Ran {
                code: Some(1),
                stdout: String::new(),
                stderr: format!("{said}\n"),
            };
            assert_eq!(
                interpret(&ran),
                TailnetState::NotRunning {
                    reason: said.to_owned()
                }
            );
        }
    }

    #[test]
    fn the_status_is_read_whatever_else_the_cli_says() {
        let ran = cli::Ran {
            code: Some(0),
            stdout: RUNNING.to_owned(),
            stderr: "Warning: client version \"1.104.0\" != tailscaled server version \"1.102.0\"\n"
                .to_owned(),
        };
        assert_eq!(interpret(&ran), TailnetState::OnTailnet(buzz_laptop()));
    }

    #[test]
    fn any_other_failure_is_unknown_in_its_own_words() {
        // Access refused (cli.go:168-170), a flag the CLI does not know, silence, a signal.
        let unknown = |code, stderr: &str| {
            interpret(&cli::Ran {
                code,
                stdout: String::new(),
                stderr: stderr.to_owned(),
            })
        };
        assert_eq!(
            unknown(
                Some(1),
                "Access denied: status access denied\n\nUse 'sudo tailscale status --json'.\n"
            ),
            TailnetState::Unknown {
                reason: "Access denied: status access denied".to_owned()
            }
        );
        assert_eq!(
            unknown(Some(2), "flag provided but not defined: -peers\n"),
            TailnetState::Unknown {
                reason: "flag provided but not defined: -peers".to_owned()
            }
        );
        assert_eq!(
            unknown(Some(1), ""),
            TailnetState::Unknown {
                reason: "tailscale status ended 1, saying nothing".to_owned()
            }
        );
        assert_eq!(unknown(None, "").word(), "unknown");
    }

    #[test]
    fn a_cli_that_is_not_there_is_tailscale_not_installed() {
        let missing = mp_os::temp_dir()
            .join("mp-no-such-tailscale")
            .join("tailscale");
        let ran = run(Command::new(missing), CLI_TIMEOUT);
        assert_eq!(ran, Err(RunError::NotFound));
        assert_eq!(answer(ran), TailnetState::NotInstalled);
        assert_eq!(answer(Err(RunError::TimedOut)).word(), "unknown");
        assert_eq!(
            answer(Err(RunError::Failed("Permission denied".to_owned()))).word(),
            "unknown"
        );
    }

    #[test]
    fn the_cli_is_only_asked_for_its_status() {
        let command = status_command(Path::new("tailscale"));
        assert_eq!(command.get_program(), OsStr::new("tailscale"));
        assert_eq!(
            command.get_args().collect::<Vec<_>>(),
            STATUS_ARGS.map(OsStr::new)
        );
        assert_eq!(STATUS_ARGS[0], "status");
    }

    /// A process that outlives its time is killed, and the run ends at its time, not the
    /// process's.
    #[cfg(unix)]
    #[test]
    fn a_cli_that_does_not_answer_is_killed_at_its_time() {
        let mut command = Command::new("sh");
        command
            .args(["-c", "sleep 3"])
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped());
        let started = web_time::Instant::now();
        assert_eq!(
            run(command, Duration::from_millis(200)),
            Err(RunError::TimedOut)
        );
        assert!(started.elapsed() < Duration::from_secs(2));
    }

    /// tests/gui/fake-tailscale.sh, which the GUI scripts point `MP_TAILSCALE_CLI` at, through
    /// the run the planner makes: each of its states read as the planner's.
    #[cfg(unix)]
    #[test]
    fn the_fake_cli_answers_each_state_as_the_real_one() {
        let fake = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/gui/fake-tailscale.sh");
        let ask = |state: &str, args: &[&str]| {
            let mut command = Command::new("sh");
            command
                .arg(fake.as_os_str())
                .args(args)
                .env("FAKE_TAILSCALE_STATE", state)
                .stdin(std::process::Stdio::null())
                .stdout(std::process::Stdio::piped())
                .stderr(std::process::Stdio::piped());
            answer(run(command, CLI_TIMEOUT))
        };
        let expected = [
            ("running", TailnetState::OnTailnet(buzz_laptop())),
            ("starting", TailnetState::Starting),
            ("nostate", TailnetState::Starting),
            (
                "needslogin",
                TailnetState::SignedOut {
                    login_url: Some(LOGIN_URL.to_owned()),
                },
            ),
            ("loggedout", TailnetState::SignedOut { login_url: None }),
            ("needsapproval", TailnetState::NeedsApproval),
            ("stopped", TailnetState::Stopped),
        ];
        for (state, wanted) in expected {
            assert_eq!(ask(state, &STATUS_ARGS), wanted, "{state}");
        }
        assert_eq!(ask("down", &STATUS_ARGS).word(), "not-running");
        // Anything but its status is refused, as nothing else may be asked of the real one.
        assert_eq!(ask("running", &["up"]).word(), "unknown");
        assert_eq!(ask("running", &["status"]).word(), "unknown");
    }

    #[test]
    fn the_cli_is_looked_for_on_the_path_first_then_where_linux_puts_it() {
        let root = mp_os::temp_dir();
        let (first, second) = (root.join("first"), root.join("second"));
        let path: OsString = std::env::join_paths([&first, &second]).expect("the test's own value");
        let found = |present: &[PathBuf]| {
            find_cli_from(
                None,
                Platform::Linux,
                Some(path.as_os_str()),
                None,
                |candidate| present.iter().any(|there| there == candidate),
            )
        };
        let usr_bin = PathBuf::from("/usr/bin/tailscale");
        let usr_local_bin = PathBuf::from("/usr/local/bin/tailscale");
        assert_eq!(found(&[]), None);
        assert_eq!(
            found(std::slice::from_ref(&usr_local_bin)),
            Some(usr_local_bin.clone())
        );
        assert_eq!(
            found(&[usr_local_bin, usr_bin.clone()]),
            Some(usr_bin.clone())
        );
        assert_eq!(
            found(&[usr_bin, second.join("tailscale")]),
            Some(second.join("tailscale"))
        );
        assert_eq!(
            found(&[second.join("tailscale"), first.join("tailscale")]),
            Some(first.join("tailscale"))
        );
    }

    #[test]
    fn a_relative_or_empty_folder_on_the_path_is_not_looked_in() {
        let root = mp_os::temp_dir();
        let relative = Path::new("relative").join("bin");
        let path = std::env::join_paths([PathBuf::new(), relative, root.join("bin")]).expect("the test's own value");
        let candidates = candidates(Platform::Linux, Some(path.as_os_str()), None);
        assert_eq!(
            candidates,
            [
                root.join("bin").join("tailscale"),
                PathBuf::from("/usr/bin/tailscale"),
                PathBuf::from("/usr/local/bin/tailscale"),
            ]
        );
    }

    #[test]
    fn on_macos_the_apps_own_executable_is_the_cli() {
        let found = |present: &[&str]| {
            find_cli_from(None, Platform::MacOs, None, None, |candidate| {
                present.iter().any(|there| Path::new(there) == candidate)
            })
        };
        let app = "/Applications/Tailscale.app/Contents/MacOS/Tailscale";
        assert_eq!(
            found(&["/usr/local/bin/tailscale", app]),
            Some(PathBuf::from(app))
        );
        assert_eq!(
            found(&["/opt/homebrew/bin/tailscale", "/usr/local/bin/tailscale"]),
            Some(PathBuf::from("/usr/local/bin/tailscale"))
        );
        assert_eq!(
            found(&["/opt/homebrew/bin/tailscale"]),
            Some(PathBuf::from("/opt/homebrew/bin/tailscale"))
        );
    }

    #[test]
    fn on_windows_the_cli_is_tailscale_exe_in_program_files() {
        let root = mp_os::temp_dir();
        let path = std::env::join_paths([root.join("bin")]).expect("the test's own value");
        let program_files = Path::new(r"D:\Programs");
        let installed = program_files.join("Tailscale").join("tailscale.exe");
        let usual = PathBuf::from(r"C:\Program Files\Tailscale\tailscale.exe");
        let found = |present: &[PathBuf]| {
            find_cli_from(
                None,
                Platform::Windows,
                Some(path.as_os_str()),
                Some(program_files.as_os_str()),
                |candidate| present.iter().any(|there| there == candidate),
            )
        };
        assert_eq!(found(std::slice::from_ref(&usual)), Some(usual.clone()));
        assert_eq!(
            found(&[usual.clone(), installed.clone()]),
            Some(installed.clone())
        );
        assert_eq!(
            found(&[installed, root.join("bin").join("tailscale.exe")]),
            Some(root.join("bin").join("tailscale.exe"))
        );
        // `ProgramFiles` unset: the usual folder alone.
        let only_usual = find_cli_from(None, Platform::Windows, None, None, |candidate| {
            candidate == usual.as_path()
        });
        assert_eq!(only_usual, Some(usual));
    }

    #[test]
    fn the_harness_names_the_cli_or_none() {
        let path = std::env::join_paths([mp_os::temp_dir()]).expect("the test's own value");
        let everywhere = |_: &Path| true;
        // `none`: no Tailscale, whatever is installed.
        assert_eq!(
            find_cli_from(
                Some(OsStr::new(NO_CLI)),
                Platform::Linux,
                Some(path.as_os_str()),
                None,
                everywhere
            ),
            None
        );
        // A path: that one, whether or not it is there - a run of one not there says so.
        assert_eq!(
            find_cli_from(
                Some(OsStr::new("/work/tailscale")),
                Platform::Linux,
                Some(path.as_os_str()),
                None,
                |_| false
            ),
            Some(PathBuf::from("/work/tailscale"))
        );
        // Empty is unset: looked for as ever.
        assert_eq!(
            find_cli_from(Some(OsStr::new("")), Platform::Linux, None, None, |candidate| {
                candidate == Path::new("/usr/bin/tailscale")
            }),
            Some(PathBuf::from("/usr/bin/tailscale"))
        );
    }

    #[test]
    fn tailnet_hosts_are_tailscales_ranges_and_magicdns_names() {
        for host in [
            "100.64.0.0",
            "100.127.255.255",
            "100.101.102.103",
            " 100.64.0.1 ",
            "fd7a:115c:a1e0::1",
            "[fd7a:115c:a1e0::1]",
            "fd7a:115c:a1e0:ab12:4843:cd96:6265:6667",
            "my-pi.tail1234.ts.net.",
            "my-pi.tail1234.ts.net",
            "MY-PI.TAIL1234.TS.NET",
        ] {
            assert!(is_tailnet_host(host), "{host} is a tailnet host");
        }
        for host in [
            "100.63.255.255",
            "100.128.0.0",
            "10.0.0.1",
            "127.0.0.1",
            "192.168.1.20",
            "fd7a:115c:a1e1::1",
            "fd7a:115c::1",
            "::1",
            "ts.net.example.com",
            "ts.net",
            ".ts.net",
            "white",
            "my-pi",
            "localhost",
            "",
        ] {
            assert!(!is_tailnet_host(host), "{host} is not a tailnet host");
        }
    }

    #[test]
    fn a_failed_tailnet_link_says_why_when_this_computer_is_off_the_tailnet() {
        assert_eq!(
            failure_reason("100.101.102.103", &TailnetState::NotInstalled).as_deref(),
            Some(
                "100.101.102.103 is a Tailscale address, and this computer is not on a tailnet \
                 (Tailscale: not installed) - see HELP > Using Tailscale"
            )
        );
        assert_eq!(
            failure_reason(
                "my-pi.tail1234.ts.net",
                &TailnetState::SignedOut { login_url: None }
            )
            .as_deref(),
            Some(
                "my-pi.tail1234.ts.net is a Tailscale name, and this computer is not on a \
                 tailnet (Tailscale: signed out) - see HELP > Using Tailscale"
            )
        );
        for off in [
            TailnetState::NotRunning {
                reason: "failed to connect to local tailscaled".to_owned(),
            },
            TailnetState::NeedsApproval,
            TailnetState::Stopped,
            TailnetState::Starting,
        ] {
            let reason = failure_reason("fd7a:115c:a1e0::1", &off).expect("the test's own value");
            assert!(reason.contains(&format!("({})", off.text())), "{reason}");
        }
        // On a tailnet, the failure is not this computer's; and nothing is said of a host off
        // the tailnet, or before the state is known.
        let on = TailnetState::OnTailnet(buzz_laptop());
        assert_eq!(failure_reason("100.101.102.103", &on), None);
        assert_eq!(failure_reason("192.168.1.20", &TailnetState::NotInstalled), None);
        assert_eq!(failure_reason("white", &TailnetState::NotInstalled), None);
        assert_eq!(failure_reason("100.101.102.103", &TailnetState::Unchecked), None);
        let unknown = TailnetState::Unknown {
            reason: "tailscale status gave no answer in time".to_owned(),
        };
        assert_eq!(failure_reason("100.101.102.103", &unknown), None);
    }

    #[test]
    fn the_indicator_says_each_state_in_a_word_or_two() {
        let states = [
            (TailnetState::Unchecked, "Tailscale: checking", "unchecked"),
            (
                TailnetState::NotInstalled,
                "Tailscale: not installed",
                "not-installed",
            ),
            (
                TailnetState::NotRunning {
                    reason: String::new(),
                },
                "Tailscale: not running",
                "not-running",
            ),
            (
                TailnetState::SignedOut { login_url: None },
                "Tailscale: signed out",
                "signed-out",
            ),
            (
                TailnetState::NeedsApproval,
                "Tailscale: waiting for approval",
                "needs-approval",
            ),
            (TailnetState::Stopped, "Tailscale: stopped", "stopped"),
            (TailnetState::Starting, "Tailscale: starting", "starting"),
            (
                TailnetState::OnTailnet(buzz_laptop()),
                "Tailscale: on tailnet as buzz-laptop, 100.101.102.103",
                "on-tailnet",
            ),
            (
                TailnetState::Unknown {
                    reason: String::new(),
                },
                "Tailscale: state unknown",
                "unknown",
            ),
        ];
        for (state, text, word) in states {
            assert_eq!(state.text(), text);
            assert_eq!(state.word(), word);
            assert_eq!(state.is_on_tailnet(), word == "on-tailnet");
        }
        assert_eq!(TailnetState::default(), TailnetState::Unchecked);
    }

    #[test]
    fn the_hover_gives_the_clis_words_and_the_sign_in_link() {
        let not_running = TailnetState::NotRunning {
            reason: "failed to connect to local tailscaled".to_owned(),
        };
        assert!(
            not_running
                .details()
                .ends_with("failed to connect to local tailscaled")
        );
        let signed_out = TailnetState::SignedOut {
            login_url: Some(LOGIN_URL.to_owned()),
        };
        assert!(signed_out.details().ends_with(LOGIN_URL));
        assert!(
            TailnetState::NotInstalled
                .details()
                .ends_with(download_page(Platform::current()))
        );
    }

    #[test]
    fn the_facts_are_the_state_name_and_address() {
        assert_eq!(
            facts(&TailnetState::OnTailnet(buzz_laptop())),
            [
                ("tailscale.state", "on-tailnet".to_owned()),
                ("tailscale.name", "buzz-laptop".to_owned()),
                ("tailscale.address", "100.101.102.103".to_owned()),
            ]
        );
        assert_eq!(
            facts(&TailnetState::SignedOut { login_url: None }),
            [
                ("tailscale.state", "signed-out".to_owned()),
                ("tailscale.name", "none".to_owned()),
                ("tailscale.address", "none".to_owned()),
            ]
        );
        // Off unless MP_FACTS names a file, as under `cargo test`: nothing recorded, no panic.
        record_facts(&TailnetState::NotInstalled);
    }

    #[test]
    fn tailscale_is_offered_at_first_start_on_linux_and_macos_when_not_installed() {
        let linux = offer(Platform::Linux, &TailnetState::NotInstalled).expect("the test's own value");
        assert_eq!(linux.page, "https://tailscale.com/download/linux");
        assert_eq!(
            linux.commands,
            [
                "curl -fsSL https://tailscale.com/install.sh | sh",
                "sudo tailscale up"
            ]
        );
        let mac = offer(Platform::MacOs, &TailnetState::NotInstalled).expect("the test's own value");
        assert_eq!(mac.page, "https://tailscale.com/download/mac");
        assert!(mac.commands.is_empty());
        for offered in [linux, mac] {
            assert!(offered.text.contains("not installed"));
            assert!(offered.after.contains(HELP_PAGE));
        }
        // Not on Windows (the owner's "linux & osx"), not where Tailscale is installed in any
        // state, and not before the state is known.
        assert_eq!(offer(Platform::Windows, &TailnetState::NotInstalled), None);
        assert_eq!(offer(Platform::Other, &TailnetState::NotInstalled), None);
        for installed in [
            TailnetState::Unchecked,
            TailnetState::SignedOut { login_url: None },
            TailnetState::Stopped,
            TailnetState::OnTailnet(buzz_laptop()),
            TailnetState::NotRunning {
                reason: String::new(),
            },
        ] {
            assert_eq!(offer(Platform::Linux, &installed), None, "{installed:?}");
        }
        assert_eq!(download_page(Platform::Other), DOWNLOAD_PAGE);
        assert_eq!(
            download_page(Platform::Windows),
            "https://tailscale.com/download/windows"
        );
    }

    /// Ticks `poller` until its state is `wanted`, for at most five seconds.
    fn tick_until(poller: &mut Poller, wanted: &TailnetState) {
        let deadline = web_time::Instant::now() + Duration::from_secs(5);
        while poller.state() != wanted {
            assert!(
                web_time::Instant::now() < deadline,
                "still {:?}, wanted {wanted:?}",
                poller.state()
            );
            poller.tick();
            wasm_thread::sleep(Duration::from_millis(5));
        }
    }

    #[test]
    fn the_poller_hands_over_each_new_state_and_looks_again_when_asked() {
        let looks = Arc::new(AtomicUsize::new(0));
        let counted = Arc::clone(&looks);
        // An hour between looks: within the test only an ask brings another.
        let mut poller = Poller::start_with(
            move || {
                if counted.fetch_add(1, Ordering::SeqCst) == 0 {
                    TailnetState::NotInstalled
                } else {
                    TailnetState::Stopped
                }
            },
            Duration::from_secs(3600),
        );
        assert_eq!(*poller.state(), TailnetState::Unchecked);
        tick_until(&mut poller, &TailnetState::NotInstalled);
        poller.check_now();
        tick_until(&mut poller, &TailnetState::Stopped);
        // A look that finds nothing new hands nothing over.
        poller.check_now();
        let deadline = web_time::Instant::now() + Duration::from_secs(5);
        while looks.load(Ordering::SeqCst) < 3 {
            assert!(web_time::Instant::now() < deadline, "no third look");
            wasm_thread::sleep(Duration::from_millis(5));
        }
        wasm_thread::sleep(Duration::from_millis(50));
        assert!(!poller.tick());
        assert_eq!(*poller.state(), TailnetState::Stopped);
    }

    /// The page's node, by the page's words: not started, loading, signed out with its link,
    /// waiting for approval, on the tailnet with its name and both addresses, stopped and why.
    #[test]
    fn the_page_node_reads_as_the_indicator_shows_it() {
        let none: [String; 0] = [];
        assert_eq!(from_page("NotStarted", None, &none, None, None), TailnetState::NotStarted);
        assert_eq!(from_page("Loading", None, &none, None, None), TailnetState::Starting);
        assert_eq!(
            from_page("NeedsLogin", None, &none, Some("https://login.example/a".to_owned()), None),
            TailnetState::SignedOut {
                login_url: Some("https://login.example/a".to_owned())
            }
        );
        assert_eq!(from_page("NeedsApproval", None, &none, None, None), TailnetState::NeedsApproval);
        let running = from_page(
            "Running",
            Some("mpr-jerboa-stonecat"),
            &["100.101.102.103".to_owned(), "fd7a:115c:a1e0::1".to_owned()],
            None,
            None,
        );
        assert_eq!(running.text(), "Tailscale: on tailnet as mpr-jerboa-stonecat, 100.101.102.103");
        assert_eq!(
            running.device().and_then(|device| device.ipv6).map(|ip| ip.to_string()),
            Some("fd7a:115c:a1e0::1".to_owned())
        );
        assert_eq!(
            from_page("Stopped", None, &none, None, Some("no network".to_owned())).word(),
            "not-running"
        );
        assert_eq!(TailnetState::NotStarted.word(), "not-started");
        assert_eq!(TailnetState::NotStarted.text(), "Tailscale: not started");
        assert!(failure_reason("100.101.102.103", &TailnetState::NotStarted).is_some());
    }

    /// The host of a link that dials one, IPv6 in brackets; none for the kinds that do not.
    #[test]
    fn a_links_host_is_what_it_dials() {
        assert_eq!(link_host("tcp:my-pi:5760"), Some("my-pi"));
        assert_eq!(link_host("TCP:100.101.102.103:5760"), Some("100.101.102.103"));
        assert_eq!(link_host("udpcl:my-pi.tail1234.ts.net:14550"), Some("my-pi.tail1234.ts.net"));
        assert_eq!(link_host("udpcl:[fd7a:115c:a1e0::1]:14550"), Some("fd7a:115c:a1e0::1"));
        assert_eq!(link_host("udp:14550"), None);
        assert_eq!(link_host("serial:/dev/ttyACM0:115200"), None);
        assert_eq!(link_host("file:testdata/mavlink/autotest.tlog"), None);
        assert_eq!(link_host("tcp:"), None);
    }
}
