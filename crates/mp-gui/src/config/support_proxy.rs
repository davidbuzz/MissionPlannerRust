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

//! The Support Proxy: `Controls/SerialSupportProxy.cs`, the window the Advanced page's Support
//! Proxy opens (`ConfigAdvanced.BUT_supportproxy_Click`, `GCSViews/ConfigurationView/
//! ConfigAdvanced.cs:124-127`: `new SerialSupportProxy().Show()`), which shares the vehicle's link
//! with a support engineer's ground station through a server.
//!
//! What it shows, as the Designer lays out its 317 x 83 form "Support Proxy Connection" - a
//! table's cells, whose places the Designer writes down (`SerialSupportProxy.Designer.cs:29-186`;
//! the `.resx` holds nothing): Server and its text box, Support ID and its number (1 to 65535),
//! Protocol with UDP and TCP, and Connect beside the first two rows.
//!
//! What it does:
//!
//! * the constructor (`SerialSupportProxy.cs:17-47`): Connect reads Stop, and the server and
//!   the number are disabled, while `MainV2.comPort.MirrorStream` is open - a proxy started from
//!   an earlier form is still running, as nothing stops it when its form closes; UDP is ticked
//!   unless `SerialSupportProxy_UDP` says false; the server and the number are the ones last
//!   connected to over that protocol (`UDP_host_SerialSupportProxy` and `UDP_port_...`, or `TCP_`),
//!   else support.ardupilot.org and 1; the number has the keyboard, its text selected, and Enter
//!   anywhere is Connect (`AcceptButton`);
//! * Connect (`BUT_connect_Click`, `:49-106`), with the mirror open: closes it, and Connect and the
//!   boxes are back. Otherwise a stream to the server - `UdpSerialConnect` or a `TcpSerial` that
//!   reconnects, both with `ConfigRef` `_SerialSupportProxy`, so that opening it keeps its host
//!   and port under those names (`ExtLibs/Comms/CommsUDPSerialConnect.cs:74-79`,
//!   `CommsTCPSerial.cs:101-146`) - made `MirrorStream`, written to from the vehicle and writing to
//!   it (`MirrorStreamWrite`), as [`mp_link::mirror`] relays; once open, Stop, the boxes disabled,
//!   and `SerialSupportProxy_UDP` kept.
//!
//! Where this is not the C#, each written at its site:
//!
//! * the form is drawn over SETUP, modal, where the C#'s is a free form (`Show`); a second click
//!   opens a fresh one. Closing it leaves the mirror as the C# leaves it, running;
//! * the stream is opened off the UI thread - a TCP connect can take seconds - Connect dimmed
//!   meanwhile, where the C# holds the form;
//! * "Error Connecting", the C#'s box when the stream will not open, is the status line, as the
//!   owner ruled (2026-09-25): a connection failure never gets a message box. So is what the
//!   constructor throws for a kept number that is not one (`int.Parse`) or is outside 1 to 65535,
//!   and the form does not open, as `new` threw;
//! * the mirror is subscribed to the link again when the application's link is replaced by
//!   another - a connect, a reopen - where the C#'s one `comPort` keeps its `Mirrors` across them;
//! * the colours are this application's.
//!
//! `Tracking.AddPage` (`:29`) reports the form to Google Analytics, which nothing in this
//! application does.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::mpsc::{Receiver, TryRecvError};

use gpui::{
    AnyElement, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px, rgb,
};
use mp_link::inspector::PacketSubscription;
use mp_link::mirror::{Mirror, Relayed, Reopen};
use mp_mission::dotnet::{bool_text, parse_bool};
use mp_transport::{TcpTransport, Transport, UdpClientTransport};

use super::motor_test::NumericUpDown;
use super::optional::{at, button, label, text_box};
use crate::MissionPlanner;
use crate::settings::Persisted;
use crate::telemetry::Telemetry;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::theme;

/// The form's caption. `// C#: Controls/SerialSupportProxy.Designer.cs:180`
pub const FORM_TEXT: &str = "Support Proxy Connection";
/// `ClientSize`. `// C#: Controls/SerialSupportProxy.Designer.cs:175`
pub const CLIENT: (f32, f32) = (317.0, 83.0);

/// `label1`. `// C#: Controls/SerialSupportProxy.Designer.cs:46-52`
pub const SERVER: &str = "Server";
/// Its place.
const SERVER_AT: (f32, f32) = (27.0, 10.0);
/// `label2`, the number's: the support engineer's id is the server's port.
/// `// C#: Controls/SerialSupportProxy.Designer.cs:56-62`
pub const SUPPORT_ID: &str = "Support ID";
/// Its place.
const SUPPORT_ID_AT: (f32, f32) = (7.0, 36.0);
/// `label3`. `// C#: Controls/SerialSupportProxy.Designer.cs:141-147`
pub const PROTOCOL: &str = "Protocol";
/// Its place.
const PROTOCOL_AT: (f32, f32) = (19.0, 61.0);

/// `TXT_host`. `// C#: Controls/SerialSupportProxy.Designer.cs:66-72`
pub const HOST_AT: (f32, f32, f32, f32) = (71.0, 7.0, 158.0, 20.0);
/// Its `Text`, and the constructor's default.
/// `// C#: Controls/SerialSupportProxy.Designer.cs:72; SerialSupportProxy.cs:37`
pub const HOST_DEFAULT: &str = "support.ardupilot.org";

/// `NUM_port`: 1 to 65535, `InterceptArrowKeys` false. `// C#: Controls/SerialSupportProxy.Designer.cs:76-97`
pub const PORT_AT: (f32, f32, f32, f32) = (71.0, 33.0, 62.0, 20.0);
/// `Minimum` and `Maximum`.
pub const PORT_BOUNDS: (f64, f64) = (1.0, 65535.0);
/// The constructor's default, `?? "1"`. `// C#: Controls/SerialSupportProxy.cs:38`
pub const PORT_DEFAULT: &str = "1";

/// `BUT_connect`. `// C#: Controls/SerialSupportProxy.Designer.cs:128-137`
pub const CONNECT_AT: (f32, f32, f32, f32) = (235.0, 18.0, 75.0, 23.0);
/// `Strings.Connect`. `// C#: ExtLibs/Strings/Strings.resx:453-455`
pub const CONNECT: &str = "Connect";
/// `Strings.Stop`. `// C#: ExtLibs/Strings/Strings.resx:450-452`
pub const STOP: &str = "Stop";

/// `rad_udp`, checked. `// C#: Controls/SerialSupportProxy.Designer.cs:151-159`
pub const UDP_AT: (f32, f32, f32, f32) = (71.0, 59.0, 48.0, 17.0);
/// `rad_tcp`. `// C#: Controls/SerialSupportProxy.Designer.cs:163-169`
pub const TCP_AT: (f32, f32, f32, f32) = (153.0, 59.0, 46.0, 17.0);

/// Whether the last proxy was UDP. `// C#: Controls/SerialSupportProxy.cs:32, 104`
pub const UDP_SETTING: &str = "SerialSupportProxy_UDP";
/// The streams' `ConfigRef`. `// C#: Controls/SerialSupportProxy.cs:66, 76`
pub const CONFIG_REF: &str = "_SerialSupportProxy";

/// The box the C# shows when the stream will not open; the status line's words here.
/// `// C#: Controls/SerialSupportProxy.cs:94`
pub const ERROR_CONNECTING: &str = "Error Connecting";

/// .NET's `OverflowException` from `int.Parse`.
pub const OVERFLOW: &str = "Value was either too large or too small for an Int32.";

/// The two protocols.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Protocol {
    /// `UdpSerialConnect`.
    Udp,
    /// `TcpSerial`.
    Tcp,
}

impl Protocol {
    /// `settings_prefix`, and `OnSettings`' prefix: "UDP" or "TCP".
    #[must_use]
    pub const fn prefix(self) -> &'static str {
        match self {
            Self::Udp => "UDP",
            Self::Tcp => "TCP",
        }
    }

    /// Where the host it was last opened to is kept: `UDP_host_SerialSupportProxy`.
    #[must_use]
    pub fn host_key(self) -> String {
        format!("{}_host{CONFIG_REF}", self.prefix())
    }

    /// Where the port is kept.
    #[must_use]
    pub fn port_key(self) -> String {
        format!("{}_port{CONFIG_REF}", self.prefix())
    }
}

/// Which box has the keyboard.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Field {
    /// `TXT_host`.
    Host,
    /// `NUM_port`.
    Port,
}

/// The `SerialSupportProxy` form.
#[derive(Debug)]
pub struct ProxyForm {
    /// `TXT_host`.
    pub host: TextField,
    /// `NUM_port`.
    pub port: NumericUpDown,
    /// `rad_udp.Checked`; `rad_tcp` is the other.
    pub udp: bool,
    /// `BUT_connect.Text == Strings.Stop`: the server and the number disabled.
    pub stopping: bool,
    /// The box with the keyboard.
    pub editing: Option<Field>,
    /// `NUM_port.Select(0, 5)`: the number's text selected - all of it, a number of 1 to 65535
    /// being five characters at most - until a key collapses the selection, the first typed
    /// replacing it.
    pub port_selected: bool,
}

impl ProxyForm {
    /// `new SerialSupportProxy()`: the settings `get` reads, and whether the mirror is open. A
    /// kept number that is not a whole number, or not one `NUM_port` holds, is what the C#
    /// throws.
    /// `// C#: Controls/SerialSupportProxy.cs:17-47`
    pub fn new(get: impl Fn(&str) -> Option<String>, mirror_open: bool) -> Result<Self, String> {
        // `GetBoolean(key, true)`: `bool.TryParse`, else true.
        let udp = get(UDP_SETTING)
            .as_deref()
            .and_then(parse_bool)
            .unwrap_or(true);
        let protocol = if udp { Protocol::Udp } else { Protocol::Tcp };
        let mut host = TextField::new("");
        host.set(get(&protocol.host_key()).unwrap_or_else(|| HOST_DEFAULT.to_owned()));
        let text = get(&protocol.port_key()).unwrap_or_else(|| PORT_DEFAULT.to_owned());
        // `int.Parse`: a whole number in an `int`, else a `FormatException` - or an
        // `OverflowException` for digits too many for one.
        let trimmed = text.trim();
        let digits = trimmed.strip_prefix(['-', '+']).unwrap_or(trimmed);
        let value = match mp_log::netfmt::parse_i64(&text) {
            Some(value) if i32::try_from(value).is_ok() => value,
            Some(_) => return Err(OVERFLOW.to_owned()),
            None if !digits.is_empty() && digits.chars().all(|c| c.is_ascii_digit()) => {
                return Err(OVERFLOW.to_owned());
            }
            None => return Err(format!("{text:?}: {}", super::fftui::NOT_A_NUMBER)),
        };
        let (low, high) = PORT_BOUNDS;
        #[allow(clippy::cast_precision_loss)] // an int
        let value = value as f64;
        if !(low..=high).contains(&value) {
            return Err(format!(
                "Value of '{value}' is not valid for 'Value'. 'Value' should be between \
                 'Minimum' and 'Maximum'."
            ));
        }
        // `NUM_port.Select(); NUM_port.Select(0, 5)`: the number has the keyboard, its text
        // selected, so what is typed replaces it.
        Ok(Self {
            host,
            port: NumericUpDown::new((value, low, high)),
            udp,
            stopping: mirror_open,
            editing: Some(Field::Port),
            port_selected: true,
        })
    }

    /// A key in the number: typed over the selection while there is one - a character, Backspace
    /// or Delete replacing it - and as `NumericUpDown` takes it otherwise; any other key collapses
    /// the selection first. The arrow keys do nothing (`InterceptArrowKeys = false`).
    /// `// C#: Controls/SerialSupportProxy.Designer.cs:78`
    pub fn port_key(&mut self, event: &KeyDownEvent) -> bool {
        let keystroke = &event.keystroke;
        if matches!(keystroke.key.as_str(), "up" | "down") {
            return false;
        }
        if std::mem::take(&mut self.port_selected) {
            let chord = keystroke.modifiers.control || keystroke.modifiers.platform;
            let typing = !chord && keystroke.key_char.as_deref().is_some_and(|c| !c.is_empty());
            if typing || matches!(keystroke.key.as_str(), "backspace" | "delete") {
                self.port.field.clear();
                if !typing {
                    return true;
                }
            }
        }
        self.port.key(event)
    }

    /// The protocol ticked.
    #[must_use]
    pub const fn protocol(&self) -> Protocol {
        if self.udp {
            Protocol::Udp
        } else {
            Protocol::Tcp
        }
    }

    /// `NUM_port.Value.ToString("0")`: validated, as reading `Value` validates.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // 1..=65535
    pub fn port_number(&mut self) -> u16 {
        self.port.commit().round() as u16
    }

    /// The keyboard left a box: the number's text validated.
    pub fn leave(&mut self) {
        if self.editing.take() == Some(Field::Port) {
            self.port.commit();
        }
    }

    /// A box clicked into: the click puts the caret in it, collapsing the selection.
    pub fn begin(&mut self, field: Field) {
        if self.editing != Some(field) {
            self.leave();
        }
        self.editing = Some(field);
        self.port_selected = false;
    }
}

/// A stream being opened: its protocol, and the thread's answer.
type Opening = (Protocol, Receiver<Result<Box<dyn Transport>, String>>);

/// The window, and the mirror it starts, held with the Advanced page: `MainV2.comPort`'s
/// `MirrorStream`, which outlives the form.
#[derive(Debug, Default)]
pub struct SupportProxy {
    /// The form, while it is open.
    pub window: Option<ProxyForm>,
    /// How many times it has been opened.
    pub opened: usize,
    /// `MirrorStream`: the last one made, open or closed.
    mirror: Option<Mirror>,
    /// Which protocol it is.
    pub protocol: Option<Protocol>,
    /// Its subscription to the link's packets.
    subscription: Option<PacketSubscription>,
    /// Whether it writes to the link it is subscribed to.
    attached: bool,
    /// A stream being opened.
    opening: Option<Opening>,
    /// What the C# would have shown or thrown, for the status line.
    status: Option<String>,
    /// The settings this window reads and writes, as the last tick found them, for the facts.
    pub settings: Vec<(String, String)>,
}

impl SupportProxy {
    /// `MainV2.comPort.MirrorStream != null && MainV2.comPort.MirrorStream.IsOpen`.
    #[must_use]
    pub fn mirror_open(&self) -> bool {
        self.mirror.as_ref().is_some_and(Mirror::is_open)
    }

    /// What the mirror has passed.
    #[must_use]
    pub fn relayed(&self) -> Relayed {
        self.mirror
            .as_ref()
            .map(Mirror::relayed)
            .unwrap_or_default()
    }

    /// Whether a stream is being opened.
    #[must_use]
    pub fn opening(&self) -> Option<Protocol> {
        self.opening.as_ref().map(|(protocol, _)| *protocol)
    }

    /// `new SerialSupportProxy().Show()`: a fresh form, or what its constructor threw.
    /// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:124-127`
    pub fn show(&mut self, persisted: &Persisted) {
        match ProxyForm::new(
            |key| persisted.get(key).map(str::to_owned),
            self.mirror_open(),
        ) {
            Ok(form) => {
                self.opened += 1;
                self.window = Some(form);
            }
            Err(error) => self.status = Some(error),
        }
    }

    /// The form's close box: the mirror goes on.
    pub fn close(&mut self) {
        self.window = None;
    }

    /// `rad_udp` or `rad_tcp` clicked: it is ticked, the other not.
    pub fn choose(&mut self, protocol: Protocol) {
        if self.opening.is_some() {
            return;
        }
        if let Some(form) = self.window.as_mut() {
            form.udp = protocol == Protocol::Udp;
        }
    }

    /// `BUT_connect_Click`: Stop closes the open mirror; Connect opens a stream to the server
    /// the form names - its host and port kept as the stream keeps them - to be made the mirror.
    /// `// C#: Controls/SerialSupportProxy.cs:49-106`
    pub fn press_connect(&mut self, persisted: &mut Persisted) {
        if self.opening.is_some() {
            return;
        }
        if self.mirror_open() {
            if let Some(mirror) = self.mirror.as_mut() {
                mirror.close();
            }
            self.subscription = None;
            self.attached = false;
            if let Some(form) = self.window.as_mut() {
                form.stopping = false;
            }
            return;
        }
        let Some(form) = self.window.as_mut() else {
            return;
        };
        let protocol = form.protocol();
        let host = form.host.value().to_owned();
        let port = form.port_number();
        form.leave();
        // `MirrorStream = serial`: the one before, open no longer, let go.
        self.mirror = None;
        self.subscription = None;
        self.attached = false;
        self.protocol = Some(protocol);
        // `OnSettings("UDP_port" + ConfigRef, Port, true)` and the host, or the TCP ones, before
        // the stream tries to open.
        persisted.set(&protocol.port_key(), port.to_string());
        persisted.set(&protocol.host_key(), host.clone());
        let (send, receive) = std::sync::mpsc::channel();
        let spawned = std::thread::Builder::new()
            .name("mp-support-proxy".to_owned())
            .spawn(move || {
                let _ = send.send(open(protocol, &host, port));
            });
        match spawned {
            Ok(_) => self.opening = Some((protocol, receive)),
            Err(error) => self.status = Some(format!("{ERROR_CONNECTING}: {error}")),
        }
    }

    /// Once a frame: a stream opened made the mirror - Stop, the boxes disabled,
    /// `SerialSupportProxy_UDP` kept - or the failure on the status line; the mirror subscribed
    /// to the link there is and writing to it; the number read when the keyboard left it.
    /// What goes on the status line is returned.
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        persisted: &mut Persisted,
        number_focused: bool,
        text_focused: bool,
    ) -> Option<String> {
        if let Some(form) = self.window.as_mut() {
            match form.editing {
                Some(Field::Port) if !number_focused => form.leave(),
                Some(Field::Host) if !text_focused => form.editing = None,
                _ => {}
            }
        }
        if let Some((protocol, receiver)) = self.opening.as_ref() {
            let protocol = *protocol;
            let opened = match receiver.try_recv() {
                Ok(opened) => Some(opened),
                Err(TryRecvError::Empty) => None,
                Err(TryRecvError::Disconnected) => Some(Err("no answer".to_owned())),
            };
            if let Some(opened) = opened {
                self.opening = None;
                self.opened_stream(protocol, opened, persisted);
            }
        }
        if let Some(mirror) = self.mirror.as_ref().filter(|mirror| mirror.is_open()) {
            if !self
                .subscription
                .as_ref()
                .is_some_and(|subscription| telemetry.carries(subscription))
            {
                self.subscription = telemetry.on_packet(mirror.handler());
                self.attached = false;
            }
            if !self.attached
                && self.subscription.is_some()
                && let Some((sender, _)) = telemetry.send_handle()
            {
                mirror.attach(Some(sender));
                self.attached = true;
            }
        }
        self.settings = [
            UDP_SETTING.to_owned(),
            Protocol::Udp.host_key(),
            Protocol::Udp.port_key(),
            Protocol::Tcp.host_key(),
            Protocol::Tcp.port_key(),
        ]
        .into_iter()
        .map(|key| {
            let value = persisted.get(&key).unwrap_or("none").to_owned();
            (key, value)
        })
        .collect();
        self.status.take()
    }

    /// What opening the stream came to: the mirror made, or "Error Connecting".
    /// `// C#: Controls/SerialSupportProxy.cs:79-105`
    fn opened_stream(
        &mut self,
        protocol: Protocol,
        opened: Result<Box<dyn Transport>, String>,
        persisted: &mut Persisted,
    ) {
        let stream = match opened {
            Ok(stream) => stream,
            Err(error) => {
                // `CustomMessageBox.Show("Error Connecting")`: the status line (the owner's
                // ruling of 2026-09-25).
                self.status = Some(format!("{ERROR_CONNECTING}: {error}"));
                return;
            }
        };
        let description = stream.description().to_owned();
        let reopen = (protocol == Protocol::Tcp).then(|| reopen_tcp(&description));
        match Mirror::start(stream, true, reopen.flatten()) {
            Ok(mirror) => {
                self.mirror = Some(mirror);
                if let Some(form) = self.window.as_mut() {
                    form.stopping = true;
                    form.editing = None;
                }
                persisted.set(UDP_SETTING, bool_text(protocol == Protocol::Udp));
            }
            Err(error) => self.status = Some(format!("{ERROR_CONNECTING}: {error}")),
        }
    }

    /// Waits for the stream being opened, as a test does.
    #[cfg(test)]
    fn finish_opening(
        &mut self,
        telemetry: &Telemetry,
        persisted: &mut Persisted,
    ) -> Option<String> {
        let mut status = None;
        for _ in 0..2000 {
            if let Some(words) = self.tick(telemetry, persisted, true, true) {
                status = Some(words);
            }
            if self.opening.is_none() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        status
    }
}

/// `UdpSerialConnect.Open(host, port)`, or `TcpSerial.Open()` with its host and port set.
/// `// C#: ExtLibs/Comms/CommsUDPSerialConnect.cs:74-121; ExtLibs/Comms/CommsTCPSerial.cs:101-169`
fn open(protocol: Protocol, host: &str, port: u16) -> Result<Box<dyn Transport>, String> {
    match protocol {
        Protocol::Udp => UdpClientTransport::open(host, port)
            .map(|stream| Box::new(stream) as Box<dyn Transport>)
            .map_err(|error| error.to_string()),
        Protocol::Tcp => TcpTransport::connect(host, port)
            .map(|stream| Box::new(stream) as Box<dyn Transport>)
            .map_err(|error| error.to_string()),
    }
}

/// `TcpSerial.autoReconnect`: the address the stream was opened to, connected to again.
/// `// C#: ExtLibs/Comms/CommsTCPSerial.cs:330-361`
fn reopen_tcp(description: &str) -> Option<Reopen> {
    let address: std::net::SocketAddr = description.strip_prefix("tcp:")?.parse().ok()?;
    Some(Box::new(move || {
        TcpTransport::connect(&address.ip().to_string(), address.port())
            .ok()
            .map(|stream| Box::new(stream) as Box<dyn Transport>)
    }))
}

/// The keyboard focus of the form's boxes.
pub struct FocusHandles {
    /// `NUM_port`.
    pub number: FocusHandle,
    /// `TXT_host`.
    pub text: FocusHandle,
}

impl FocusHandles {
    /// New handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            number: cx.focus_handle(),
            text: cx.focus_handle(),
        }
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(holder: &SupportProxy) {
    use crate::facts::record;
    record("config.supportproxy.window", holder.window.is_some());
    record("config.supportproxy.opened", holder.opened);
    record(
        "config.supportproxy.mirror",
        match holder.mirror.as_ref() {
            None => "none",
            Some(mirror) if mirror.is_open() => "open",
            Some(_) => "closed",
        },
    );
    record(
        "config.supportproxy.mirror.protocol",
        holder.protocol.map_or("none", Protocol::prefix),
    );
    record(
        "config.supportproxy.opening",
        holder.opening().map_or("none", Protocol::prefix),
    );
    let relayed = holder.relayed();
    record("config.supportproxy.relayed.up", relayed.up);
    record("config.supportproxy.relayed.down", relayed.down);
    record("config.supportproxy.relayed.dropped", relayed.dropped);
    for (key, value) in &holder.settings {
        record(format!("config.supportproxy.setting.{key}"), value);
    }
    let Some(form) = holder.window.as_ref() else {
        return;
    };
    record(
        "config.supportproxy.button",
        if form.stopping { STOP } else { CONNECT },
    );
    record("config.supportproxy.host", form.host.value());
    record("config.supportproxy.port", form.port.field.value());
    record("config.supportproxy.host.enabled", !form.stopping);
    record("config.supportproxy.port.enabled", !form.stopping);
    record("config.supportproxy.udp", form.udp);
    record("config.supportproxy.tcp", !form.udp);
    record(
        "config.supportproxy.editing",
        match form.editing {
            None => "none",
            Some(Field::Host) => "host",
            Some(Field::Port) => "port",
        },
    );
    record("config.supportproxy.port.selected", form.port_selected);
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// How the drawing reaches the holder.
fn access(this: &mut MissionPlanner) -> &mut SupportProxy {
    &mut this.extra.support_proxy
}

/// Connect, from the button or from Enter: `AcceptButton`.
fn connect(this: &mut MissionPlanner) {
    this.extra.support_proxy.press_connect(&mut this.persisted);
}

/// A radio button and its text at its place, ticked or not.
fn radio(
    id: &'static str,
    text: &'static str,
    (x, y, width, height): (f32, f32, f32, f32),
    checked: bool,
    enabled: bool,
    protocol: Protocol,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let ring = if enabled {
        theme::ACCENT
    } else {
        theme::BORDER
    };
    let base = crate::probe::measured(id, div())
        .id(SharedString::from(id))
        .size_full()
        .flex()
        .items_center()
        .gap_1()
        .child(
            div()
                .size(px(13.0))
                .flex()
                .items_center()
                .justify_center()
                .rounded_full()
                .border_1()
                .border_color(rgb(ring))
                .bg(rgb(theme::BG))
                .children(checked.then(|| {
                    div().size(px(5.0)).rounded_full().bg(rgb(if enabled {
                        theme::ACCENT
                    } else {
                        theme::DIM
                    }))
                })),
        )
        .child(
            div()
                .text_xs()
                .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
                .child(text),
        );
    let base = if enabled {
        base.cursor_pointer()
            .on_click(cx.listener(move |this, _event, _window, cx| {
                access(this).choose(protocol);
                cx.notify();
            }))
    } else {
        base
    };
    at(x, y, width, height).child(base).into_any_element()
}

/// The form over the window, its caption with a close box.
/// `// C#: Controls/SerialSupportProxy.Designer.cs:29-186`
pub fn overlay(
    holder: &SupportProxy,
    focus: &FocusHandles,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = holder.window.as_ref()?;
    let size = window.viewport_size();
    let idle = holder.opening.is_none();
    let boxes = idle && !form.stopping;
    let text = focus.text.clone();
    let client = div()
        .relative()
        .w(px(CLIENT.0))
        .h(px(CLIENT.1))
        .child(label(SERVER_AT.0, SERVER_AT.1, SERVER, true))
        .child(label(SUPPORT_ID_AT.0, SUPPORT_ID_AT.1, SUPPORT_ID, true))
        .child(label(PROTOCOL_AT.0, PROTOCOL_AT.1, PROTOCOL, true))
        .child(text_box(
            "proxy-TXT_host",
            form.host.value(),
            Some(&text),
            form.editing == Some(Field::Host),
            boxes,
            HOST_AT,
            |this| {
                if let Some(form) = access(this).window.as_mut() {
                    form.begin(Field::Host);
                }
            },
            |this, event| {
                let outcome = access(this)
                    .window
                    .as_mut()
                    .map(|form| form.host.key(event));
                match outcome {
                    Some(KeyOutcome::Submitted) => {
                        connect(this);
                        true
                    }
                    Some(KeyOutcome::Changed) => true,
                    _ => false,
                }
            },
            cx,
        ))
        .child(super::spectrogram::updown(
            "proxy-NUM_port",
            &form.port,
            PORT_AT,
            form.editing == Some(Field::Port),
            &focus.number,
            boxes,
            |this| {
                if let Some(form) = access(this).window.as_mut() {
                    form.begin(Field::Port);
                }
            },
            |this, event| {
                match event.keystroke.key.as_str() {
                    // `InterceptArrowKeys = false`.
                    "up" | "down" => false,
                    "enter" => {
                        if let Some(form) = access(this).window.as_mut() {
                            form.port.commit();
                        }
                        connect(this);
                        true
                    }
                    _ => access(this)
                        .window
                        .as_mut()
                        .is_some_and(|form| form.port_key(event)),
                }
            },
            |this, delta| {
                if let Some(form) = access(this).window.as_mut() {
                    form.port.step(delta);
                }
            },
            window,
            cx,
        ))
        .child(button(
            "proxy-BUT_connect",
            if form.stopping { STOP } else { CONNECT },
            CONNECT_AT,
            idle,
            |this, window, cx| {
                window.blur(cx);
                connect(this);
            },
            cx,
        ))
        .child(radio(
            "proxy-rad_udp",
            "UDP",
            UDP_AT,
            form.udp,
            idle,
            Protocol::Udp,
            cx,
        ))
        .child(radio(
            "proxy-rad_tcp",
            "TCP",
            TCP_AT,
            !form.udp,
            idle,
            Protocol::Tcp,
            cx,
        ));
    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(div().text_xs().text_color(rgb(theme::DIM)).child(FORM_TEXT))
        .child(crate::ui::action(
            "proxy-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                access(this).close();
                cx.notify();
            }),
        ));
    let body = crate::probe::measured("supportproxy", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    let over = div()
        .id("supportproxy-backdrop")
        .w(size.width)
        .h(size.height)
        .flex()
        .items_center()
        .justify_center()
        .occlude()
        .child(body);
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(over),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use std::net::UdpSocket;
    use std::time::Duration;

    use super::*;
    use crate::telemetry::scripted::{Vehicle, until};
    use mp_link::ProtocolTimeouts;
    use mp_mavlink::FrameDecoder;
    use mp_mavlink_dialects::all::{DIALECT, MavMessage, ParamRequestRead};

    /// Settings as `get` reads them.
    fn settings(pairs: &[(&str, &str)]) -> impl Fn(&str) -> Option<String> {
        let held: BTreeMap<String, String> = pairs
            .iter()
            .map(|(key, value)| ((*key).to_owned(), (*value).to_owned()))
            .collect();
        move |key| held.get(key).cloned()
    }

    /// The Designer's words and places, read from the tree when it is here.
    #[test]
    fn the_text_and_places_are_the_designers() {
        let Some(designer) =
            crate::config_coverage::source::csharp("Controls/SerialSupportProxy.Designer.cs")
        else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        for (text, (x, y)) in [
            (SERVER, SERVER_AT),
            (SUPPORT_ID, SUPPORT_ID_AT),
            (PROTOCOL, PROTOCOL_AT),
        ] {
            assert!(designer.contains(&format!("Text = \"{text}\";")), "{text}");
            assert!(designer.contains(&format!("Point({x}, {y})")), "{text}");
        }
        for (x, y, width, height) in [HOST_AT, PORT_AT, CONNECT_AT, UDP_AT, TCP_AT] {
            assert!(designer.contains(&format!("Point({x}, {y})")));
            assert!(designer.contains(&format!("Size({width}, {height})")));
        }
        for text in [HOST_DEFAULT, CONNECT, "UDP", "TCP", FORM_TEXT] {
            assert!(designer.contains(&format!("Text = \"{text}\";")), "{text}");
        }
        assert!(designer.contains("ClientSize = new System.Drawing.Size(317, 83)"));
        assert!(designer.contains("this.NUM_port.InterceptArrowKeys = false;"));
        // The Designer's lines end in CRLF: the decimal's parts, one a line, up to the comma.
        assert!(designer.contains("            65535,"));
        assert!(designer.contains("this.rad_udp.Checked = true;"));
        // One wiring: Connect's click.
        assert_eq!(designer.matches(" += new System.EventHandler(").count(), 1);
        assert!(designer.contains(
            "this.BUT_connect.Click += new System.EventHandler(this.BUT_connect_Click);"
        ));
        let cs = crate::config_coverage::source::csharp("Controls/SerialSupportProxy.cs")
            .unwrap_or_default();
        assert!(cs.contains(&format!("GetBoolean(\"{UDP_SETTING}\", true)")));
        assert!(cs.contains(&format!("ConfigRef = \"{CONFIG_REF}\"")));
        assert!(cs.contains(&format!("?? \"{HOST_DEFAULT}\"")));
        assert!(cs.contains(&format!("?? \"{PORT_DEFAULT}\"")));
        assert!(cs.contains(&format!("CustomMessageBox.Show(\"{ERROR_CONNECTING}\")")));
        let strings = crate::config_coverage::source::csharp("ExtLibs/Strings/Strings.resx")
            .unwrap_or_default();
        let values = crate::config_coverage::source::resx(&strings);
        assert_eq!(values.get("Stop").map(String::as_str), Some(STOP));
        assert_eq!(values.get("Connect").map(String::as_str), Some(CONNECT));
    }

    /// The constructor: UDP, support.ardupilot.org and 1 with nothing kept, the number with the
    /// keyboard and its text selected; TCP's kept server for a kept false; Stop while the mirror
    /// is open; what `int.Parse` and `Value` throw for a kept number.
    #[test]
    fn the_constructor_reads_what_was_kept() {
        let form = ProxyForm::new(settings(&[]), false).expect("a form");
        assert!(form.udp && !form.stopping);
        assert_eq!(form.host.value(), HOST_DEFAULT);
        assert_eq!(form.port.field.value(), "1");
        assert_eq!(form.editing, Some(Field::Port));
        assert!(form.port_selected);
        let kept = settings(&[
            ("SerialSupportProxy_UDP", " False "),
            ("TCP_host_SerialSupportProxy", "10.0.0.9"),
            ("TCP_port_SerialSupportProxy", "4321"),
            ("UDP_host_SerialSupportProxy", "elsewhere"),
        ]);
        let form = ProxyForm::new(kept, true).expect("a form");
        assert!(!form.udp && form.stopping);
        assert_eq!(form.protocol(), Protocol::Tcp);
        assert_eq!(
            (form.host.value(), form.port.field.value()),
            ("10.0.0.9", "4321")
        );
        assert!(form.port_selected);
        // A word that is no bool is `GetBoolean`'s default, true.
        let form = ProxyForm::new(settings(&[("SerialSupportProxy_UDP", "yes")]), false);
        assert!(form.is_ok_and(|form| form.udp));
        for (text, error) in [
            ("abc", "\"abc\": Input string was not in a correct format."),
            ("99999999999", OVERFLOW),
            (
                "0",
                "Value of '0' is not valid for 'Value'. 'Value' should be between 'Minimum' and 'Maximum'.",
            ),
            (
                "65536",
                "Value of '65536' is not valid for 'Value'. 'Value' should be between 'Minimum' and 'Maximum'.",
            ),
        ] {
            let form = ProxyForm::new(settings(&[("UDP_port_SerialSupportProxy", text)]), false);
            assert_eq!(form.err().as_deref(), Some(error), "{text}");
        }
    }

    /// Connect over UDP to a server on this machine: the host and port kept before the stream
    /// opens, Stop once it has, the boxes disabled and UDP kept; the vehicle's heartbeat reaches
    /// the server, and what the server sends reaches the vehicle; Stop closes it. The form closed
    /// and opened again shows Stop while the mirror runs.
    #[test]
    fn connect_relays_the_vehicle_both_ways_and_stop_ends_it() {
        let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default());
        let mut persisted = Persisted::at(None);
        let server = UdpSocket::bind("127.0.0.1:0").expect("a socket");
        server
            .set_read_timeout(Some(Duration::from_millis(20)))
            .expect("a timeout");
        let port = server.local_addr().expect("an address").port();

        let mut proxy = SupportProxy::default();
        proxy.show(&persisted);
        let form = proxy.window.as_mut().expect("open");
        form.host.set("127.0.0.1");
        // From 1 to the server's port, by the arrows.
        form.port.step(f64::from(port) - 1.0);
        assert_eq!(form.port.field.value(), port.to_string());
        proxy.press_connect(&mut persisted);
        assert_eq!(proxy.opening(), Some(Protocol::Udp));
        assert_eq!(
            persisted.get("UDP_host_SerialSupportProxy"),
            Some("127.0.0.1")
        );
        assert_eq!(proxy.finish_opening(&telemetry, &mut persisted), None);
        assert!(proxy.mirror_open());
        assert_eq!(persisted.get(UDP_SETTING), Some("True"));
        let form = proxy.window.as_ref().expect("open");
        assert!(form.stopping && form.editing.is_none());

        // The vehicle's heartbeats reach the server, framed from the vehicle.
        let mut decoder = FrameDecoder::new();
        let mut heard = Vec::new();
        let mut from = None;
        until("a heartbeat at the server", || {
            proxy.tick(&telemetry, &mut persisted, false, false);
            vehicle.heartbeat();
            let mut buffer = [0u8; 2048];
            if let Ok((n, peer)) = server.recv_from(&mut buffer) {
                from = Some(peer);
                decoder.push_and_drain(&buffer[..n], &DIALECT, |frame| {
                    if let Some(message) = MavMessage::decode(frame.msgid, frame.payload) {
                        heard.push((frame.sysid, frame.compid, message));
                    }
                });
            }
            !heard.is_empty()
        });
        assert!(matches!(
            heard.first(),
            Some((1, 1, MavMessage::Heartbeat(_)))
        ));
        // The server asks the vehicle for a parameter, from 253/190.
        let request = MavMessage::ParamRequestRead(ParamRequestRead {
            param_index: -1,
            target_system: 1,
            target_component: 1,
            param_id: *b"SYSID_THISMAV\0\0\0",
        });
        let mut payload = [0u8; 255];
        let len = request.encode(&mut payload);
        let mut frame = [0u8; mp_mavlink::MAX_FRAME_LEN];
        let n = mp_mavlink::encode_v2(
            &mut frame,
            0,
            253,
            190,
            request.id(),
            &payload[..len],
            request.crc_extra(),
            0,
        )
        .expect("a frame");
        server
            .send_to(&frame[..n], from.expect("the mirror's address"))
            .expect("sent");
        until("the request at the vehicle", || {
            vehicle.heartbeat();
            std::thread::sleep(Duration::from_millis(5));
            vehicle.read().contains(&request)
        });
        assert!(proxy.relayed().up >= 1 && proxy.relayed().down >= 1);

        // Closed and opened again: Stop, while the mirror runs.
        proxy.close();
        proxy.show(&persisted);
        let form = proxy.window.as_ref().expect("open");
        assert!(form.stopping);
        assert_eq!(form.port.field.value(), port.to_string());

        // Stop.
        proxy.press_connect(&mut persisted);
        assert!(!proxy.mirror_open());
        assert!(proxy.window.as_ref().is_some_and(|form| !form.stopping));
        let after = proxy.relayed().up;
        vehicle.heartbeat();
        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(proxy.relayed().up, after, "closed: nothing more");
    }

    /// TCP to a port nobody listens on: its host and port kept, as `TcpSerial.Open` keeps them
    /// before it connects; "Error Connecting" on the status line, no box, Connect still; the
    /// protocol not kept.
    #[test]
    fn a_stream_that_will_not_open_is_the_status_line() {
        let (telemetry, _vehicle) = Vehicle::connect(ProtocolTimeouts::default());
        let mut persisted = Persisted::at(None);
        let closed = std::net::TcpListener::bind("127.0.0.1:0").expect("a port");
        let port = closed.local_addr().expect("an address").port();
        drop(closed);
        let mut proxy = SupportProxy::default();
        proxy.show(&persisted);
        proxy.choose(Protocol::Tcp);
        let form = proxy.window.as_mut().expect("open");
        assert!(!form.udp);
        form.host.set("127.0.0.1");
        form.port.step(f64::from(port) - 1.0);
        proxy.press_connect(&mut persisted);
        let status = proxy.finish_opening(&telemetry, &mut persisted);
        assert!(status.is_some_and(|words| words.starts_with(ERROR_CONNECTING)));
        assert!(!proxy.mirror_open());
        assert!(proxy.window.as_ref().is_some_and(|form| !form.stopping));
        assert_eq!(
            persisted.get("TCP_host_SerialSupportProxy"),
            Some("127.0.0.1")
        );
        assert_eq!(
            persisted.get("TCP_port_SerialSupportProxy"),
            Some(port.to_string().as_str())
        );
        assert_eq!(persisted.get(UDP_SETTING), None);
    }

    /// A key, as gpui hands one over: `key`, and the character it types.
    fn press(key: &str, typed: Option<&str>) -> KeyDownEvent {
        KeyDownEvent {
            keystroke: gpui::Keystroke {
                modifiers: gpui::Modifiers::default(),
                key: key.to_owned(),
                key_char: typed.map(str::to_owned),
            },
            is_held: false,
            prefer_character_input: false,
        }
    }

    /// The number's text selected: the first character typed replaces it, the rest follow; the
    /// arrow keys do nothing; a click collapses the selection, and typing then adds to the text.
    /// `// C#: Controls/SerialSupportProxy.cs:41-42; SerialSupportProxy.Designer.cs:78`
    #[test]
    fn the_number_is_typed_over_its_selection() {
        let mut form = ProxyForm::new(settings(&[]), false).expect("a form");
        for digit in ["5", "7", "9", "9"] {
            assert!(form.port_key(&press(digit, Some(digit))));
        }
        assert_eq!(form.port.field.value(), "5799");
        assert!(!form.port_key(&press("up", None)));
        assert!(!form.port_key(&press("down", None)));
        assert_eq!(form.port_number(), 5799);
        let mut form = ProxyForm::new(settings(&[("UDP_port_SerialSupportProxy", "42")]), false)
            .expect("a form");
        form.begin(Field::Port);
        assert!(!form.port_selected);
        form.port_key(&press("1", Some("1")));
        assert_eq!(form.port.field.value(), "421");
        // Backspace over the selection empties the box, which `Value` puts back on leaving.
        let mut form = ProxyForm::new(settings(&[]), false).expect("a form");
        assert!(form.port_key(&press("backspace", None)));
        assert_eq!(form.port.field.value(), "");
        form.leave();
        assert_eq!(form.port.field.value(), "1");
    }

    /// A kept number that is no number: the form does not open, and the status line says why.
    #[test]
    fn a_constructor_that_throws_opens_nothing() {
        let (telemetry, _vehicle) = Vehicle::connect(ProtocolTimeouts::default());
        let mut persisted = Persisted::at(None);
        persisted.set("UDP_port_SerialSupportProxy", "port");
        let mut proxy = SupportProxy::default();
        proxy.show(&persisted);
        assert!(proxy.window.is_none());
        assert_eq!(proxy.opened, 0);
        let status = proxy.tick(&telemetry, &mut persisted, false, false);
        assert!(status.is_some_and(|words| words.contains("Input string")));
    }

    /// Every fact the GUI script asserts on is recorded here, and every control it clicks is
    /// drawn here.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_window_has() {
        let script = include_str!("../../../../tests/gui/config-supportproxy.gui");
        let source = include_str!("support_proxy.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.supportproxy.") => {
                    let recorded = source.contains(&format!("\"{key}\""))
                        || key.starts_with("config.supportproxy.setting.");
                    assert!(recorded, "{key} is not recorded");
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("proxy-") => {
                    let arrow = id.strip_suffix("-up").or_else(|| id.strip_suffix("-down"));
                    let drawn = source.contains(&format!("\"{id}\""))
                        || arrow.is_some_and(|base| source.contains(&format!("\"{base}\"")));
                    assert!(drawn, "{id} is not drawn");
                }
                _ => {}
            }
        }
        assert!(facts > 10, "{facts} facts");
    }
}
