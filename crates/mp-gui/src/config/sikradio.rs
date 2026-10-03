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

//! SETUP > Optional Hardware > Sik Radio: `Radio/Sikradio.cs`, laid out as `Sikradio.resx` lays
//! it out - Load Settings, Save Settings, Reset to Defaults and Upload Firmware (custom) along the
//! top, the Local and Remote group boxes with each radio's version, band, board, country, RSSI and
//! settings, Copy required to remote and the status under them, the progress bar at the bottom.
//! The radio protocol, the settings and the firmware upload are `mp_sikradio`'s; this module is
//! the page: its controls as WinForms has them (`Ctl`, `Group`), the handlers' halves that touch
//! them, and the port the radio is on.
//!
//! The radio's port is the C#'s: `ComPort.GetComPortForSiKRadio` closes the window's link
//! (`MainV2.comPort.Close()`) and opens its own at the connection box's port and baud rate, or,
//! when the link is TCP, a new `TcpSerial`, which asks "remote host" and "remote Port" with the
//! last answers filled in (`Radio/ComPort.cs:10-20`, `Radio/Sikradio.cs:234-271`,
//! `ExtLibs/Comms/CommsTCPSerial.cs:101-157`). The port stays the page's until the page goes with
//! the SETUP screen, when the radio is put back in transparent mode and the port closed
//! (`Disconnect`). Closing the window's link shows no screen again, as `comPort.Close()` shows
//! none: SETUP's list is keyed to the link closed (as Force Bootloader's is).
//!
//! Where this differs from the C#, and why:
//!
//! * The C# runs each handler on the UI thread, blocking it through the radio's guard times and
//!   pumping `Application.DoEvents`; here a handler's radio half runs on a thread of its own
//!   ([`Work`]), telling the page what `lbl_status`, the progress bar and the boxes say as it goes,
//!   and the page applies what it read to the controls when it is done, in the C#'s order. The
//!   controls the C# disables for a handler are disabled for it here; a click while one runs is
//!   ignored, as the C#'s UI thread is busy.
//! * A box reporting a radio or port that failed - "Failed to enter command mode", "Set Command
//!   error", "Failed to save parameters", "Couldn't communicate with modem", "Error during read",
//!   the unhandled exception of a handler with no `catch` - is the window's status line, the
//!   owner's ruling of 2026-09-25. Warnings, validations, reports and questions stay boxes.
//! * `OpenFileDialog` and `SaveFileDialog` are a typed path, as on the other pages; `dlgSave` adds
//!   no extension, having no `DefaultExt`.
//! * Upload Firmware (standard) (`BUT_upload`, `BUT_upload_Click`, `getFirmware`'s downloads) is
//!   not drawn: `Sikradio.resx` makes it invisible and nothing shows it, so it cannot be clicked.
//!   The Progressbar click still toggles `beta`, which only that button reads, as the C#'s does.
//! * The C#'s `ToolTip` font shrink for a spare editor's label is not done: the label is drawn at
//!   its size.
//! * Changing the connection box's port or baud rate after the page has its port makes a new
//!   session on the same port, as `GetSession` does; the port changes when the page goes.
//! * Not ported, having no callers: `doConnect`, `getTelemPortWithRadio` (reached only with the
//!   window's link open, which `GetComPortForSiKRadio` has just closed), `GetParamNumber`,
//!   `GetValueFromControl`, `uploader_LogEvent`, `iHex_LogEvent`, `iHex_ProgressEvent`,
//!   `uploader_ProgressEvent`, `btnCommsLog_Click`, and the three handlers the Designer does not
//!   wire: `txt_aeskey_TextChanged`, `linkLabel_mavlink_LinkClicked`,
//!   `linkLabel_lowlatency_LinkClicked` - their links are drawn, inert, as the C# draws them.
//!
//! The Designer wires seventeen events (`Radio/Sikradio.Designer.cs:226-1807`); [`Click`] has
//! each, and [`WIRINGS`] lists them in the Designer's order with where each is handled.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::fmt;
use std::io;
use std::path::PathBuf;
use std::sync::mpsc;
use std::time::{Duration, Instant};

use gpui::{
    AnyElement, AnyView, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*,
    px, rgb,
};
use mp_sikradio::page::{self as wire, Change, KeyBox, Loaded, Programmed, Report, SavePlan};
use mp_sikradio::session::Session;
use mp_sikradio::settings::{Setting, Settings, range};
use mp_sikradio::uploader::{Board, Frequency};
use mp_sikradio::{Port, Wire, try_parse_int};
use mp_transport::Transport;

use crate::MissionPlanner;
use crate::config::basic_tuning::Tip;
use crate::config::firmware::{BoxIds, PathBox, path_box};
use crate::config::optional::{InputBox, at, group, message_box};
use crate::config::servo_output::{Combo, Message, dropdown, modal};
use crate::setup::Key;
use crate::telemetry::Telemetry;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action, theme};

// ---------------------------------------------------------------------------------------------
// The controls.
// ---------------------------------------------------------------------------------------------

/// A control's kind, from its Designer type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// `Label`.
    Label,
    /// `ComboBox`, every one a `DropDownList`.
    Combo,
    /// `CheckBox`.
    Check,
    /// `TextBox`.
    Text,
    /// `MyButton`, `Button`.
    Button,
    /// `LinkLabel`.
    Link,
    /// `ProgressBar`.
    Bar,
    /// `GroupBox`.
    Group,
}

/// A `.resx` `Location` and `Size`.
type Place = (f32, f32, f32, f32);

/// A control as the Designer and the `.resx` make it.
#[derive(Debug, Clone, Copy)]
struct Spec {
    name: &'static str,
    kind: Kind,
    place: Place,
    text: &'static str,
    visible: bool,
    enabled: bool,
    read_only: bool,
    multiline: bool,
    tip: &'static str,
    items: &'static [&'static str],
}

impl Spec {
    const fn new(name: &'static str, kind: Kind, place: Place) -> Self {
        Self {
            name,
            kind,
            place,
            text: "",
            visible: true,
            enabled: true,
            read_only: false,
            multiline: false,
            tip: "",
            items: &[],
        }
    }
    const fn text(mut self, text: &'static str) -> Self {
        self.text = text;
        self
    }
    const fn hidden(mut self) -> Self {
        self.visible = false;
        self
    }
    const fn disabled(mut self) -> Self {
        self.enabled = false;
        self
    }
    const fn read_only(mut self) -> Self {
        self.read_only = true;
        self
    }
    const fn multiline(mut self) -> Self {
        self.multiline = true;
        self
    }
    const fn tip(mut self, tip: &'static str) -> Self {
        self.tip = tip;
        self
    }
    const fn items(mut self, items: &'static [&'static str]) -> Self {
        self.items = items;
        self
    }
}

/// Which of the page's three containers a control is in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Side {
    /// The page itself, `this`.
    Top,
    /// `groupBoxLocal`.
    Local,
    /// `groupBoxRemote`.
    Remote,
}

impl Side {
    /// What the remote radio's names start with.
    const fn prefix(self) -> &'static str {
        match self {
            Self::Remote => "R",
            _ => "",
        }
    }

    const fn specs(self) -> &'static [Spec] {
        match self {
            Self::Top => TOP,
            Self::Local => LOCAL,
            Self::Remote => REMOTE,
        }
    }
}

/// An item of a combo box: its text, and the key of a `KeyValuePair` item (`MAVLINK`'s).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Item {
    /// What it shows.
    pub text: String,
    /// `ValueMember`'s `Key`.
    pub value: Option<i32>,
}

impl Item {
    fn text(text: impl Into<String>) -> Self {
        Self {
            text: text.into(),
            value: None,
        }
    }
}

/// A control as WinForms holds it.
#[derive(Debug, Clone, PartialEq)]
pub struct Ctl {
    /// The Designer's field: what the code refers to it by.
    pub designer: &'static str,
    /// `Name`: what `Controls.Find` finds it by, which the page renames.
    pub name: String,
    /// The kind.
    pub kind: Kind,
    /// Where it is in its container.
    pub place: Place,
    /// `Text`: a label's, button's or text box's.
    pub text: String,
    /// `Checked`.
    pub checked: bool,
    /// A combo box's items (`DataSource` or `Items`).
    pub items: Vec<Item>,
    /// `SelectedIndex`.
    pub selected: Option<usize>,
    /// `Tag`: the setting whose options a combo box shows.
    pub tag: Option<Setting>,
    /// `Visible`.
    pub visible: bool,
    /// `Enabled`, its own: [`SikRadio::enabled`] is it under its group's.
    pub enabled: bool,
    /// `ReadOnly`.
    pub read_only: bool,
    /// `Multiline`.
    pub multiline: bool,
    /// `toolTip1`'s text for it.
    pub tip: String,
}

impl Ctl {
    fn new(spec: &Spec) -> Self {
        Self {
            designer: spec.name,
            name: spec.name.to_owned(),
            kind: spec.kind,
            place: spec.place,
            text: if spec.kind == Kind::Text {
                String::new()
            } else {
                spec.text.to_owned()
            },
            checked: false,
            items: spec.items.iter().map(|text| Item::text(*text)).collect(),
            selected: None,
            tag: None,
            visible: spec.visible,
            enabled: spec.enabled,
            read_only: spec.read_only,
            multiline: spec.multiline,
            tip: spec.tip.to_owned(),
        }
    }

    /// `Text`: a drop-down list's is its selected item's, "" with none.
    #[must_use]
    pub fn shown(&self) -> String {
        match self.kind {
            Kind::Combo => self
                .selected
                .and_then(|i| self.items.get(i))
                .map(|item| item.text.clone())
                .unwrap_or_default(),
            _ => self.text.clone(),
        }
    }

    /// `DataSource = items`: bound, the first item selected.
    fn set_items(&mut self, items: Vec<Item>) {
        self.selected = if items.is_empty() { None } else { Some(0) };
        self.items = items;
    }

    fn set_ints(&mut self, values: &[i32]) {
        self.set_items(values.iter().map(|v| Item::text(v.to_string())).collect());
    }

    fn set_strings(&mut self, values: &[String]) {
        self.set_items(values.iter().map(|v| Item::text(v.clone())).collect());
    }

    /// `Text = value`. A drop-down list selects the item with that text - in another case when
    /// none has it exactly - and is left as it is when none has it; anything else takes the text.
    /// `// C#: ComboBox.Text's setter (System.Windows.Forms)`
    fn set_text(&mut self, value: &str) {
        if self.kind != Kind::Combo {
            value.clone_into(&mut self.text);
            return;
        }
        if self.shown() == value && self.selected.is_some() {
            return;
        }
        if let Some(index) = self
            .items
            .iter()
            .position(|item| item.text == value)
            .or_else(|| {
                self.items
                    .iter()
                    .position(|item| item.text.eq_ignore_ascii_case(value))
            })
        {
            self.selected = Some(index);
        }
    }

    /// `SelectedValue`: a `KeyValuePair` item's key, else the item.
    fn selected_value(&self) -> Option<String> {
        let item = self.items.get(self.selected?)?;
        Some(
            item.value
                .map_or_else(|| item.text.clone(), |v| v.to_string()),
        )
    }
}

/// A container's controls, in the Designer's `Controls.Add` order, and its `Enabled`.
#[derive(Debug, Clone, PartialEq)]
pub struct Group {
    /// `Enabled`.
    pub enabled: bool,
    /// The controls.
    pub ctls: Vec<Ctl>,
}

impl Group {
    fn new(side: Side) -> Self {
        Self {
            enabled: true,
            ctls: side.specs().iter().map(Ctl::new).collect(),
        }
    }

    /// The control the Designer's field names.
    fn index(&self, designer: &str) -> Option<usize> {
        self.ctls.iter().position(|c| c.designer == designer)
    }

    /// `Controls.Find(name, true)`'s first: the control of that `Name`, in any case.
    fn find(&self, name: &str) -> Option<usize> {
        self.ctls
            .iter()
            .position(|c| c.name.eq_ignore_ascii_case(name))
    }
}

/// `_KnownNameDescriptions`: the label a spare editor gets for these settings.
/// `// C#: Radio/Sikradio.cs:42-47`
const KNOWN_NAMES: [(&str, &str); 3] = [
    ("RSSI_IN_DBM", "RSSI in dBm"),
    ("AUXSER_SPEED", "Aux Baud"),
    ("AIR_FRAMELEN", "Air Frame Length"),
];

/// A `TDynamicLabelEditorPair`: the container, the label, the editor, and the two settings it
/// takes with the label text for each.
type Dynamic = (
    Side,
    &'static str,
    &'static str,
    [(&'static str, &'static str); 2],
);

/// `_DynamicLabelEditorPairRegister`: the SBUS check box and combo box of each radio, each
/// taking either of two settings, its label then that setting's.
/// `// C#: Radio/Sikradio.cs:131-158`
const DYNAMIC: [Dynamic; 4] = [
    (
        Side::Local,
        "lblSBUSIN",
        "GPO1_3SBUSIN",
        [
            ("GPO1_3SBUSIN", "GPO1_3SBUSIN"),
            ("GPO1_1SBUSIN", "GPO1_1SBUSIN"),
        ],
    ),
    (
        Side::Remote,
        "lblRSBUSIN",
        "RGPO1_3SBUSIN",
        [
            ("RGPO1_3SBUSIN", "GPO1_3SBUSIN"),
            ("RGPO1_1SBUSIN", "GPO1_1SBUSIN"),
        ],
    ),
    (
        Side::Local,
        "lblSBUSOUT",
        "GPO1_3SBUSOUT",
        [
            ("GPO1_3SBUSOUT", "GPO1_3SBUSOUT"),
            ("GPO1_1SBUSOUT", "GPO1_1SBUSOUT"),
        ],
    ),
    (
        Side::Remote,
        "lblRSBUSOUT",
        "RGPO1_3SBUSOUT",
        [
            ("RGPO1_3SBUSOUT", "GPO1_3SBUSOUT"),
            ("RGPO1_1SBUSOUT", "GPO1_1SBUSOUT"),
        ],
    ),
];

/// `_LocalLabelEditorPairs`: the label and editor pairs a setting the page has no control for may
/// borrow, in the order they were added.
/// `// C#: Radio/Sikradio.cs:160-188`
const LOCAL_PAIRS: [(&str, &str); 29] = [
    ("lblNODEID", "NODEID"),
    ("lblDESTID", "DESTID"),
    ("lblTX_ENCAP_METHOD", "TX_ENCAP_METHOD"),
    ("lblRX_ENCAP_METHOD", "RX_ENCAP_METHOD"),
    ("lblMAX_DATA", "MAX_DATA"),
    ("lblMAX_RETRIES", "MAX_RETRIES"),
    ("lblGLOBAL_RETRIES", "GLOBAL_RETRIES"),
    ("lblSER_BRK_DETMS", "SER_BRK_DETMS"),
    ("label54", "FSFRAMELOSS"),
    ("lblNETID", "NETID"),
    ("lblTXPOWER", "TXPOWER"),
    ("lblECC", "ECC"),
    ("lblMAVLINK", "MAVLINK"),
    ("lblOPPRESEND", "OPPRESEND"),
    ("lblGPI1_1R_CIN", "GPI1_1R_CIN"),
    ("lblGPO1_1R_COUT", "GPO1_1R_COUT"),
    ("lblGPO1_3STATLED", "GPO1_3STATLED"),
    ("lblGPI1_2AUXIN", "GPI1_2AUXIN"),
    ("lblGPO1_3AUXOUT", "GPO1_3AUXOUT"),
    ("lblMIN_FREQ", "MIN_FREQ"),
    ("lblMAX_FREQ", "MAX_FREQ"),
    ("lblNUM_CHANNELS", "NUM_CHANNELS"),
    ("lblDUTY_CYCLE", "DUTY_CYCLE"),
    ("lblLBT_RSSI", "LBT_RSSI"),
    ("lblRTSCTS", "RTSCTS"),
    ("lblMAX_WINDOW", "MAX_WINDOW"),
    ("lblENCRYPTION_LEVEL", "ENCRYPTION_LEVEL"),
    ("lblGPO1_0TXEN485", "GPO1_0TXEN485"),
    ("lblGPIO1_1FUNC", "GPIO1_1FUNC"),
];

/// `_RemoteLabelEditorPairs`.
/// `// C#: Radio/Sikradio.cs:190-218`
const REMOTE_PAIRS: [(&str, &str); 29] = [
    ("lblRNODEID", "RNODEID"),
    ("lblRDESTID", "RDESTID"),
    ("lblRTX_ENCAP_METHOD", "RTX_ENCAP_METHOD"),
    ("lblRRX_ENCAP_METHOD", "RRX_ENCAP_METHOD"),
    ("lblRMAX_DATA", "RMAX_DATA"),
    ("lblRMAX_RETRIES", "RMAX_RETRIES"),
    ("lblRGLOBAL_RETRIES", "RGLOBAL_RETRIES"),
    ("lblRSER_BRK_DETMS", "RSER_BRK_DETMS"),
    ("lblRFSFRAMELOSS", "RFSFRAMELOSS"),
    ("lblRNETID", "RNETID"),
    ("lblRTXPOWER", "RTXPOWER"),
    ("lblRECC", "RECC"),
    ("lblRMAVLINK", "RMAVLINK"),
    ("lblROPPRESEND", "ROPPRESEND"),
    ("lblRGPI1_1R_CIN", "RGPI1_1R_CIN"),
    ("lblRGPO1_1R_COUT", "RGPO1_1R_COUT"),
    ("lblRGPO1_3STATLED", "RGPO1_3STATLED"),
    ("lblRGPI1_2AUXIN", "RGPI1_2AUXIN"),
    ("lblRGPO1_3AUXOUT", "RGPO1_3AUXOUT"),
    ("lblRMIN_FREQ", "RMIN_FREQ"),
    ("lblRMAX_FREQ", "RMAX_FREQ"),
    ("lblRNUM_CHANNELS", "RNUM_CHANNELS"),
    ("lblRDUTY_CYCLE", "RDUTY_CYCLE"),
    ("lblRLBT_RSSI", "RLBT_RSSI"),
    ("lblRRTSCTS", "RRTSCTS"),
    ("lblRMAX_WINDOW", "RMAX_WINDOW"),
    ("lblRENCRYPTION_LEVEL", "RENCRYPTION_LEVEL"),
    ("lblRGPO1_0TXEN485", "RGPO1_0TXEN485"),
    ("lblRGPIO1_1FUNC", "RGPIO1_1FUNC"),
];

/// `ExtraParamControlsSet`'s six label and combo box pairs, by the local names.
/// `// C#: Radio/Sikradio.cs:79-89; Radio/Models.cs:31-57, 174-212`
const EXTRA: [(&str, &str); 6] = [
    ("lblNODEID", "NODEID"),
    ("lblDESTID", "DESTID"),
    ("lblTX_ENCAP_METHOD", "TX_ENCAP_METHOD"),
    ("lblRX_ENCAP_METHOD", "RX_ENCAP_METHOD"),
    ("lblMAX_DATA", "MAX_DATA"),
    ("lblMAX_RETRIES", "MAX_RETRIES"),
];

/// `ExtraParamControlsSet`'s `Others`.
const EXTRA_OTHERS: [&str; 4] = [
    "lblGLOBAL_RETRIES",
    "GLOBAL_RETRIES",
    "lblSER_BRK_DETMS",
    "SER_BRK_DETMS",
];

/// The combo boxes whose lists `SaveDefaultCBObjects` keeps, for `RestoreAllDefaultCBObjects`.
/// `// C#: Radio/Sikradio.cs:116-129`
const DEFAULT_LISTS: [&str; 5] = [
    "SERIAL_SPEED",
    "AIR_SPEED",
    "NETID",
    "NUM_CHANNELS",
    "MAX_WINDOW",
];

/// A local name as the remote radio's control is named: `lblNODEID` is `lblRNODEID`.
fn named(side: Side, local: &str) -> String {
    match (side, local.strip_prefix("lbl")) {
        (Side::Remote, Some(rest)) => format!("lblR{rest}"),
        (Side::Remote, None) => format!("R{local}"),
        _ => local.to_owned(),
    }
}

/// The radio firmware's model: `Radio/Models.cs`'s `Model`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Model {
    /// Point to point.
    P2p,
    /// Multipoint on the a, + and u.
    Multipoint,
    /// The x's asynchronous firmware.
    Async,
    /// The x's multipoint firmware.
    MultipointX,
}

/// `mavlink_option` and `mavlink_option_simple`: the MAVLINK combo box's items.
/// `// C#: Radio/Sikradio.cs:2356-2367`
const MAVLINK_OPTIONS: [(i32, &str); 3] = [(0, "RawData"), (1, "Mavlink"), (2, "LowLatency")];

/// `Enum.Parse(typeof(mavlink_option), value).ToString()`: a number's name when it has one, a
/// name as it is; anything else is the C#'s `ArgumentException`.
fn mavlink_name(value: &str) -> Result<String, String> {
    let trimmed = value.trim();
    if let Some(number) = try_parse_int(trimmed) {
        return Ok(MAVLINK_OPTIONS
            .iter()
            .find(|(v, _)| *v == number)
            .map_or_else(|| number.to_string(), |(_, n)| (*n).to_owned()));
    }
    MAVLINK_OPTIONS
        .iter()
        .find(|(_, n)| *n == trimmed)
        .map(|(_, n)| (*n).to_owned())
        .ok_or_else(|| format!("Requested value '{value}' was not found."))
}

// ---------------------------------------------------------------------------------------------
// The radio's port.
// ---------------------------------------------------------------------------------------------

/// The port as the page asks for it: `MainV2.comPort.BaseStream` - its port name and baud rate,
/// or TCP.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PortSpec {
    /// A serial port.
    Serial {
        /// `PortName`.
        path: String,
        /// `BaudRate`.
        baud: u32,
    },
    /// `PortName` "TCP...": a `TcpSerial`, whose host and port it asks for.
    Tcp {
        /// `BaudRate`, which a socket keeps and does nothing with.
        baud: u32,
    },
}

impl PortSpec {
    const fn baud(&self) -> u32 {
        match self {
            Self::Serial { baud, .. } | Self::Tcp { baud } => *baud,
        }
    }
}

/// The transport under the port.
enum Transported {
    Serial(mp_transport::SerialTransport),
    Tcp(mp_transport::TcpTransport),
    /// A port that did not open, as the C#'s unopened one after `Connect` fails.
    Closed,
}

/// The radio's port: a serial port or a TCP socket, with the baud rate it was set to and the
/// wall clock.
pub struct LinkPort {
    inner: Transported,
    baud: u32,
    epoch: Instant,
    name: String,
}

impl fmt::Debug for LinkPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LinkPort")
            .field("name", &self.name)
            .field("baud", &self.baud)
            .field("open", &self.is_open())
            .finish()
    }
}

impl LinkPort {
    /// `new SerialPort { PortName, BaudRate }.Open()`; a port that does not open is kept
    /// closed, as the C# keeps its.
    /// `// C#: Radio/Sikradio.cs:247-266`
    fn serial(path: &str, baud: u32) -> Self {
        let inner = mp_transport::SerialTransport::open(path, baud)
            .map_or(Transported::Closed, Transported::Serial);
        Self {
            inner,
            baud,
            epoch: Instant::now(),
            name: format!("serial:{path}:{baud}"),
        }
    }

    /// `new TcpSerial().Open()` with the host and port answered.
    /// `// C#: Radio/Sikradio.cs:238-244; ExtLibs/Comms/CommsTCPSerial.cs:101-157`
    fn tcp(host: &str, port: &str, baud: u32) -> Self {
        let inner = port
            .trim()
            .parse::<u16>()
            .ok()
            .and_then(|port| mp_transport::TcpTransport::connect(host.trim(), port).ok())
            .map_or(Transported::Closed, Transported::Tcp);
        Self {
            inner,
            baud,
            epoch: Instant::now(),
            name: format!("tcp:{}:{}", host.trim(), port.trim()),
        }
    }

    /// The port `Connect` failed to open.
    fn closed(name: impl Into<String>, baud: u32) -> Self {
        Self {
            inner: Transported::Closed,
            baud,
            epoch: Instant::now(),
            name: name.into(),
        }
    }

    fn transport(&mut self) -> Option<&mut dyn Transport> {
        match &mut self.inner {
            Transported::Serial(serial) => Some(serial),
            Transported::Tcp(tcp) => Some(tcp),
            Transported::Closed => None,
        }
    }
}

impl Port for LinkPort {
    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        self.transport()
            .ok_or_else(mp_sikradio::closed)?
            .write_all(data)
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
        let transport = self.transport().ok_or_else(mp_sikradio::closed)?;
        // A socket's zero timeout is no timeout at all.
        transport.set_read_timeout(timeout.max(Duration::from_millis(1)))?;
        transport.read(buf)
    }

    fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        self.baud = baud;
        if let Transported::Serial(serial) = &mut self.inner {
            serial.set_baud(baud)?;
        }
        Ok(())
    }

    fn baud(&self) -> u32 {
        self.baud
    }

    fn sleep(&mut self, duration: Duration) {
        std::thread::sleep(duration);
    }

    fn now(&self) -> Duration {
        self.epoch.elapsed()
    }

    fn is_open(&self) -> bool {
        match &self.inner {
            Transported::Serial(serial) => serial.is_open(),
            Transported::Tcp(tcp) => tcp.is_open(),
            Transported::Closed => false,
        }
    }

    fn close(&mut self) {
        if let Some(transport) = self.transport() {
            transport.close();
        }
        self.inner = Transported::Closed;
    }
}

/// `ComPort._Port` and `_Session`: the port, and the session on it while there is one
/// (`EndSession` ends the session and keeps the port).
#[derive(Debug)]
enum Linked {
    Wire(Wire<LinkPort>),
    Session(Session<LinkPort>),
}

/// The page's port: what it was opened as, and the session on it.
#[derive(Debug)]
struct Held {
    spec: PortSpec,
    link: Linked,
}

impl Held {
    /// `GetSession`: the session, made anew on the same port when the baud rate or the serial
    /// port's name is not the link's.
    /// `// C#: Radio/Sikradio.cs:2434-2462`
    fn session(self, wanted: &PortSpec) -> (PortSpec, Session<LinkPort>) {
        let spec = self.spec;
        let session = match self.link {
            Linked::Session(mut session) => {
                let renamed = match (wanted, &spec) {
                    (PortSpec::Serial { path, .. }, PortSpec::Serial { path: held, .. }) => {
                        path != held
                    }
                    (PortSpec::Serial { .. }, PortSpec::Tcp { .. }) => true,
                    _ => false,
                };
                if session.wire().baud() != wanted.baud() || renamed {
                    Session::new(session.into_wire(), wanted.baud())
                } else {
                    session
                }
            }
            Linked::Wire(wire) => Session::new(wire, wanted.baud()),
        };
        (spec, session)
    }
}

// ---------------------------------------------------------------------------------------------
// The handlers' radio halves, on their thread.
// ---------------------------------------------------------------------------------------------

/// A handler's radio half.
#[derive(Debug, Clone, PartialEq)]
pub enum Work {
    /// Load Settings.
    Load,
    /// Save Settings, with what to write.
    Save(SavePlan),
    /// Reset to Defaults, with whether there is a remote radio.
    Reset(bool),
    /// Set PPM Fail Safe, local or remote.
    Ppm(bool),
    /// An encryption level chosen, written at once; `then_key` is Copy required to remote's
    /// key, which it sets after the level's handler has returned.
    Encryption {
        /// The remote radio's.
        remote: bool,
        /// `GetEncryptionLevelValue`.
        level: i32,
        /// `RAESKEY.Text = AESKEY.Text` after it.
        then_key: Option<String>,
    },
    /// Upload Firmware (custom).
    Program,
    /// `Disconnect`, the page gone.
    Disconnect,
}

impl Work {
    const fn name(&self) -> &'static str {
        match self {
            Self::Load => "load",
            Self::Save(_) => "save",
            Self::Reset(_) => "reset",
            Self::Ppm(_) => "ppm",
            Self::Encryption { .. } => "encryption",
            Self::Program => "program",
            Self::Disconnect => "disconnect",
        }
    }
}

/// How a handler ended.
#[derive(Debug)]
enum Outcome {
    Loaded(Box<Loaded>),
    Saved(io::Result<bool>),
    Reset(io::Result<()>),
    Ppm(io::Result<()>),
    Encryption {
        remote: bool,
        then_key: Option<String>,
        result: io::Result<Option<String>>,
    },
    Programmed,
    Disconnected,
}

/// What the thread tells the page.
#[derive(Debug)]
enum Event {
    Status(String),
    Progress(f64),
    Message(String),
    Failed(String),
    TryAgain(mpsc::Sender<bool>),
    Firmware(String, mpsc::Sender<Option<PathBuf>>),
    Done(Box<(Held, Outcome)>),
}

/// The thread's [`Report`]: each thing said sent to the page, a question answered from it.
struct Sender(mpsc::Sender<Event>);

impl Report for Sender {
    fn status(&mut self, text: &str) {
        let _ = self.0.send(Event::Status(text.to_owned()));
    }
    fn progress(&mut self, completed: f64) {
        let _ = self.0.send(Event::Progress(completed));
    }
    fn message(&mut self, text: &str) {
        let _ = self.0.send(Event::Message(text.to_owned()));
    }
    fn failed(&mut self, text: &str) {
        let _ = self.0.send(Event::Failed(text.to_owned()));
    }
    fn try_again(&mut self) -> bool {
        let (reply, answer) = mpsc::channel();
        let _ = self.0.send(Event::TryAgain(reply));
        answer.recv().unwrap_or(false)
    }
    fn choose_firmware(&mut self, filter: &str) -> Option<PathBuf> {
        let (reply, answer) = mpsc::channel();
        let _ = self.0.send(Event::Firmware(filter.to_owned(), reply));
        answer.recv().ok().flatten()
    }
}

/// The handler's radio half, over the session: what happened, and the port with the session as
/// the handler leaves it.
fn run(work: Work, mut session: Session<LinkPort>, report: &mut dyn Report) -> (Outcome, Linked) {
    let outcome = match work {
        Work::Load => Outcome::Loaded(Box::new(wire::load(&mut session, report))),
        Work::Save(plan) => Outcome::Saved(wire::save(&mut session, &plan, report)),
        Work::Reset(remote) => Outcome::Reset(wire::reset(&mut session, remote, report)),
        Work::Ppm(remote) => Outcome::Ppm(wire::set_ppm_fail_safe(&mut session, remote, report)),
        Work::Encryption {
            remote,
            level,
            then_key,
        } => Outcome::Encryption {
            remote,
            then_key,
            result: wire::set_encryption_level(&mut session, remote, level, report),
        },
        Work::Program => {
            // `EndSession` after a programming, but not after a cancelled file dialog.
            if wire::program_firmware(&mut session, report) == Programmed::Ended {
                return (Outcome::Programmed, Linked::Wire(session.into_wire()));
            }
            Outcome::Programmed
        }
        Work::Disconnect => {
            wire::disconnect(&mut session);
            return (Outcome::Disconnected, Linked::Wire(session.into_wire()));
        }
    };
    (outcome, Linked::Session(session))
}

/// A handler under way.
#[derive(Debug)]
struct Job {
    work: &'static str,
    events: mpsc::Receiver<Event>,
}

/// A dialog or question over the page.
#[derive(Debug)]
enum Prompt {
    /// `TcpSerial.Open`'s "remote host", then "remote Port"; the host once answered.
    Tcp {
        input: InputBox,
        host: Option<String>,
        work: Work,
        wanted: PortSpec,
    },
    /// `dlgSave`, with the settings to write.
    Save { path: PathBox, text: String },
    /// `dlgOpen`.
    Open { path: PathBox, remote: bool },
    /// `getFirmwareLocal`'s `OpenFileDialog`.
    Firmware {
        path: PathBox,
        reply: mpsc::Sender<Option<PathBuf>>,
    },
    /// "Programming firmware failed.  Try again?".
    TryAgain { reply: mpsc::Sender<bool> },
}

// ---------------------------------------------------------------------------------------------
// The page.
// ---------------------------------------------------------------------------------------------

/// `linkLabel1_LinkClicked`'s box: the status LEDs.
/// `// C#: Radio/Sikradio.cs:2092-2096`
pub const STATUS_LEDS: &str = "The Sik Radios have 2 status LEDs, one red and one green.\ngreen LED blinking - searching for another radio \ngreen LED solid - link is established with another radio \nred LED flashing - transmitting data \nred LED solid - in firmware update mode";

/// `dlgSave.Filter` and `dlgOpen.Filter`.
/// `// C#: Radio/Sikradio.resx:6857-6865`
const INI_FILTER: &str = "INI Files|*.ini";

/// The dialog boxes' ids.
const IDS: BoxIds = BoxIds {
    question: "sikradio-again",
    yes: "sikradio-again-yes",
    no: "sikradio-again-no",
    message: "sikradio-message",
    ok: "sikradio-message-ok",
    path: "sikradio-path",
    path_value: "sikradio-path-value",
    path_ok: "sikradio-path-ok",
    path_cancel: "sikradio-path-cancel",
};

/// The Designer's seventeen wirings in its order, with where each is handled here.
/// `// C#: Radio/Sikradio.Designer.cs:226, 241, 796, 805, 1074, 1728, 1735, 1742, 1749, 1757, 1765, 1772, 1779, 1786, 1793, 1800, 1807`
#[cfg_attr(not(test), allow(dead_code))]
pub const WIRINGS: [(&str, &str); 17] = [
    ("Progressbar.Click", "Click::Progressbar"),
    ("linkLabel1.LinkClicked", "Click::StatusLeds"),
    (
        "ENCRYPTION_LEVEL.SelectedValueChanged",
        "SikRadio::encryption_changed",
    ),
    (
        "RENCRYPTION_LEVEL.SelectedValueChanged",
        "SikRadio::encryption_changed",
    ),
    ("btnRandom.Click", "Click::Random"),
    ("BUT_loadcustom.Click", "Click::LoadCustom"),
    ("BUT_resettodefault.Click", "Click::ResetToDefault"),
    ("btnRemoteLoadFromFile.Click", "Click::LoadFromFile(true)"),
    ("btnRemoteSaveToFile.Click", "Click::SaveToFile(true)"),
    ("BUT_SetPPMFailSafeRemote.Click", "Click::PpmFailSafe(true)"),
    ("BUT_SetPPMFailSafe.Click", "Click::PpmFailSafe(false)"),
    ("BUT_savesettings.Click", "Click::SaveSettings"),
    ("BUT_getcurrent.Click", "Click::GetCurrent"),
    (
        "BUT_upload.Click",
        "not drawn: Sikradio.resx makes BUT_upload invisible",
    ),
    ("BUT_Syncoptions.Click", "Click::SyncOptions"),
    ("btnSaveToFile.Click", "Click::SaveToFile(false)"),
    ("btnLoadFromFile.Click", "Click::LoadFromFile(false)"),
];

/// A click on the page: one of the Designer's wired events.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Click {
    /// `Progressbar_Click`: `beta` toggled.
    Progressbar,
    /// `linkLabel1_LinkClicked`: the status LEDs.
    StatusLeds,
    /// `btnRandom_Click`.
    Random,
    /// `BUT_loadcustom_Click`: Upload Firmware (custom).
    LoadCustom,
    /// `BUT_resettodefault_Click`.
    ResetToDefault,
    /// `btnLoadFromFile_Click`, `btnRemoteLoadFromFile_Click`.
    LoadFromFile(bool),
    /// `btnSaveToFile_Click`, `btnRemoteSaveToFile_Click`.
    SaveToFile(bool),
    /// `BUT_SetPPMFailSafe_Click`, `BUT_SetPPMFailSafeRemote_Click`.
    PpmFailSafe(bool),
    /// `BUT_savesettings_Click`.
    SaveSettings,
    /// `BUT_getcurrent_Click`: Load Settings.
    GetCurrent,
    /// `BUT_Syncoptions_Click`: Copy required to remote.
    SyncOptions,
}

/// `Sikradio`: the controls, what the radios were read as, and the port with what is under way
/// on it.
#[derive(Debug)]
pub struct SikRadio {
    active: bool,
    /// Made, as the backstage view makes a page the first time it shows it.
    created: bool,
    /// The page's own controls.
    pub top: Group,
    /// `groupBoxLocal`.
    pub local: Group,
    /// `groupBoxRemote`.
    pub remote: Group,
    /// `Progressbar.Value`.
    progress: i32,
    /// `beta`.
    beta: bool,
    /// `_LocalSettings`.
    local_settings: Option<Settings>,
    /// `_RemoteSettings`.
    remote_settings: Option<Settings>,
    /// `_AlreadyInEncCheckChangedEvtHdlr`.
    in_encryption: bool,
    /// Each register's `_Spare`, by original editor name, in the order they went in.
    local_spare: Vec<&'static str>,
    remote_spare: Vec<&'static str>,
    /// `_DefaultCBObjects`: the lists `RestoreAllDefaultCBObjects` puts back, and whether each
    /// was `Items` (nothing selected after) rather than a `DataSource`.
    default_lists: Vec<(Side, &'static str, Vec<Item>, bool)>,
    held: Option<Held>,
    job: Option<Job>,
    /// Work waiting for the one under way.
    queued: VecDeque<Work>,
    prompt: Option<Prompt>,
    messages: VecDeque<Message>,
    /// What the window's status line is to say.
    status_line: Option<String>,
    /// The combo box whose list is down, and its first row.
    open_list: Option<(Side, usize, usize)>,
    /// The key box being typed into.
    editing: Option<(Side, usize)>,
    field: TextField,
}

impl Default for SikRadio {
    fn default() -> Self {
        Self::new()
    }
}

impl SikRadio {
    /// The constructor: the controls as the Designer leaves them, then `NETID`'s 500 ids, the
    /// `MAVLINK` options, `MAX_WINDOW`'s 33 to 131, and the lists kept to put back.
    /// `// C#: Radio/Sikradio.cs:73-221`
    #[must_use]
    pub fn new() -> Self {
        let mut page = Self {
            active: false,
            created: false,
            top: Group::new(Side::Top),
            local: Group::new(Side::Local),
            remote: Group::new(Side::Remote),
            progress: 0,
            beta: false,
            local_settings: None,
            remote_settings: None,
            in_encryption: false,
            local_spare: Vec::new(),
            remote_spare: Vec::new(),
            default_lists: Vec::new(),
            held: None,
            job: None,
            queued: VecDeque::new(),
            prompt: None,
            messages: VecDeque::new(),
            status_line: None,
            open_list: None,
            editing: None,
            field: TextField::new(""),
        };
        for side in [Side::Local, Side::Remote] {
            page.with(side, &named(side, "NETID"), |c| {
                c.set_ints(&range(0, 1, 499))
            });
            page.mavlink(side, false);
            page.with(side, &named(side, "MAX_WINDOW"), |c| {
                c.set_ints(&range(33, 1, 131));
            });
        }
        for local in DEFAULT_LISTS {
            for side in [Side::Local, Side::Remote] {
                let name = named(side, local);
                if let Some(ctl) = page.ctl(side, &name) {
                    let items_based = ctl.selected.is_none();
                    let designer = ctl.designer;
                    let items = ctl.items.clone();
                    page.default_lists
                        .push((side, designer, items, items_based));
                }
            }
        }
        page
    }

    /// Shown, as the backstage view shows it.
    pub fn activate(&mut self) {
        self.active = true;
        self.created = true;
    }

    /// Another page shown: the page is kept, as the backstage view keeps it.
    pub fn hide(&mut self) {
        self.active = false;
        self.open_list = None;
        self.editing = None;
    }

    /// Whether it is the page showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// Whether a handler is under way.
    #[must_use]
    pub const fn busy(&self) -> bool {
        self.job.is_some()
    }

    /// `lbl_status.Text`.
    #[must_use]
    pub fn status(&self) -> String {
        self.ctl(Side::Top, "lbl_status")
            .map(|c| c.text.clone())
            .unwrap_or_default()
    }

    fn set_status(&mut self, text: &str) {
        self.with(Side::Top, "lbl_status", |c| text.clone_into(&mut c.text));
    }

    /// The box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// Its OK.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    fn show(&mut self, text: impl Into<String>) {
        self.messages.push_back(Message {
            title: "",
            text: text.into(),
        });
    }

    /// What the window's status line is to say, once.
    pub fn take_status(&mut self) -> Option<String> {
        self.status_line.take()
    }

    fn group(&self, side: Side) -> &Group {
        match side {
            Side::Top => &self.top,
            Side::Local => &self.local,
            Side::Remote => &self.remote,
        }
    }

    fn group_mut(&mut self, side: Side) -> &mut Group {
        match side {
            Side::Top => &mut self.top,
            Side::Local => &mut self.local,
            Side::Remote => &mut self.remote,
        }
    }

    /// The control the Designer's field names.
    #[must_use]
    pub fn ctl(&self, side: Side, designer: &str) -> Option<&Ctl> {
        let group = self.group(side);
        group.index(designer).and_then(|i| group.ctls.get(i))
    }

    fn with(&mut self, side: Side, designer: &str, f: impl FnOnce(&mut Ctl)) {
        let group = self.group_mut(side);
        if let Some(ctl) = group.index(designer).and_then(|i| group.ctls.get_mut(i)) {
            f(ctl);
        }
    }

    fn at(&self, (side, index): (Side, usize)) -> Option<&Ctl> {
        self.group(side).ctls.get(index)
    }

    fn at_mut(&mut self, (side, index): (Side, usize)) -> Option<&mut Ctl> {
        self.group_mut(side).ctls.get_mut(index)
    }

    /// `Enabled` as WinForms answers it: the control's own under its group's.
    #[must_use]
    pub fn enabled(&self, side: Side, designer: &str) -> bool {
        self.group(side).enabled && self.ctl(side, designer).is_some_and(|c| c.enabled)
    }

    fn text_of(&self, side: Side, designer: &str) -> String {
        self.ctl(side, designer).map(Ctl::shown).unwrap_or_default()
    }

    /// `SetupComboForMavlink`: `mavlink_option`'s three, or `mavlink_option_simple`'s two.
    /// `// C#: Radio/Sikradio.cs:1157-1173`
    fn mavlink(&mut self, side: Side, simple: bool) {
        let take = if simple { 2 } else { 3 };
        let items: Vec<Item> = MAVLINK_OPTIONS
            .iter()
            .take(take)
            .map(|(value, name)| Item {
                text: (*name).to_owned(),
                value: Some(*value),
            })
            .collect();
        self.with(side, &named(side, "MAVLINK"), |c| c.set_items(items));
    }

    /// `GetCBValue`: the option's value when the box shows a setting's options, `MAVLINK`'s
    /// selected key, else the text.
    /// `// C#: Radio/Sikradio.cs:1129-1155`
    fn cb_value(ctl: &Ctl) -> String {
        let text = ctl.shown();
        if let Some(tag) = &ctl.tag
            && let Some(option) = tag
                .options()
                .and_then(|o| o.iter().find(|o| o.name == text))
        {
            return option.value.to_string();
        }
        if ctl.name.contains("MAVLINK")
            && let Some(value) = ctl.selected_value()
        {
            return value;
        }
        text
    }

    /// `GetEncryptionMaxKeyLength`: "128b" is 32 hex numerals; "0" none; anything else 32.
    /// `// C#: Radio/Sikradio.cs:788-816`
    fn max_key_length(ctl: &Ctl) -> i32 {
        let text = ctl.shown();
        if text.contains('b') {
            return text
                .split('b')
                .next()
                .and_then(try_parse_int)
                .map_or(0, |bits| bits / 4);
        }
        if text == "0" { 0 } else { 32 }
    }

    /// `GetEncryptionLevelValue`.
    /// `// C#: Radio/Sikradio.cs:823-847`
    fn level_value(ctl: &Ctl) -> i32 {
        let text = ctl.shown();
        if let Some(option) = ctl
            .tag
            .as_ref()
            .and_then(Setting::options)
            .and_then(|o| o.iter().find(|o| o.name == text))
        {
            return option.value;
        }
        try_parse_int(&text).unwrap_or(0)
    }

    fn encryption_enabled(&self, side: Side) -> bool {
        self.ctl(side, &named(side, "ENCRYPTION_LEVEL"))
            .is_some_and(|c| Self::level_value(c) != 0)
    }

    // ---- the registers ----

    fn pairs(side: Side) -> &'static [(&'static str, &'static str); 29] {
        if side == Side::Remote {
            &REMOTE_PAIRS
        } else {
            &LOCAL_PAIRS
        }
    }

    fn spare_mut(&mut self, side: Side) -> &mut Vec<&'static str> {
        if side == Side::Remote {
            &mut self.remote_spare
        } else {
            &mut self.local_spare
        }
    }

    fn spec(side: Side, designer: &str) -> Option<&'static Spec> {
        side.specs().iter().find(|s| s.name == designer)
    }

    /// `TLabelEditorPairRegister.Reset`: each pair's label, name, visibility and tip as the
    /// Designer made them, and no spares.
    /// `// C#: SikRadio/RFDLib/GUI/Settings.cs:51-58, 91-99`
    fn reset_pairs(&mut self, side: Side) {
        for (label, editor) in Self::pairs(side) {
            let (Some(label_spec), Some(editor_spec)) =
                (Self::spec(side, label), Self::spec(side, editor))
            else {
                continue;
            };
            self.with(side, label, |c| {
                label_spec.text.clone_into(&mut c.text);
                c.visible = editor_spec.visible;
            });
            self.with(side, editor, |c| {
                editor_spec.name.clone_into(&mut c.name);
                c.visible = editor_spec.visible;
                editor_spec.tip.clone_into(&mut c.tip);
            });
        }
        self.spare_mut(side).clear();
    }

    /// `TLabelEditorPairRegister.SetUp`: each pair whose editor's name is no setting's is spare.
    /// `// C#: SikRadio/RFDLib/GUI/Settings.cs:106-115`
    fn set_up_pairs(&mut self, side: Side, names: &[String]) {
        for (_, editor) in Self::pairs(side) {
            let unused = self
                .ctl(side, editor)
                .is_some_and(|c| !names.contains(&c.name));
            let spare = self.spare_mut(side);
            if unused && !spare.contains(editor) {
                spare.push(editor);
            }
        }
    }

    /// `GetSpareEditor`: a spare check box for a flag, else a spare combo box, renamed for the
    /// setting and its label given the setting's description.
    /// `// C#: Radio/Sikradio.cs:1267-1291; SikRadio/RFDLib/GUI/Settings.cs:124-180`
    fn spare_editor(&mut self, side: Side, name: &str, setting: &Setting) -> Option<(Side, usize)> {
        let bare = if side == Side::Remote {
            name.get(1..).unwrap_or_default()
        } else {
            name
        };
        let description = KNOWN_NAMES
            .iter()
            .find(|(n, _)| *n == bare)
            .map_or(bare, |(_, d)| *d)
            .to_owned();
        let mut found = None;
        if setting.is_flag() {
            found = self.take_spare(side, name, &description, Kind::Check);
        }
        if found.is_none() {
            found = self.take_spare(side, name, &description, Kind::Combo);
        }
        found
    }

    fn take_spare(
        &mut self,
        side: Side,
        name: &str,
        description: &str,
        kind: Kind,
    ) -> Option<(Side, usize)> {
        let position = {
            let group = self.group(side);
            let spare = if side == Side::Remote {
                &self.remote_spare
            } else {
                &self.local_spare
            };
            spare.iter().position(|editor| {
                group
                    .index(editor)
                    .and_then(|i| group.ctls.get(i))
                    .is_some_and(|c| c.kind == kind)
            })?
        };
        let editor = self.spare_mut(side).remove(position);
        let label = Self::pairs(side)
            .iter()
            .find(|(_, e)| *e == editor)
            .map(|(l, _)| *l)?;
        self.with(side, editor, |c| {
            name.clone_into(&mut c.name);
            c.visible = true;
            description.clone_into(&mut c.tip);
        });
        self.with(side, label, |c| {
            description.clone_into(&mut c.text);
            c.visible = true;
        });
        self.group(side).index(editor).map(|i| (side, i))
    }

    /// `FindControlInGroupBox`: an SBUS editor by either of its settings - its label then that
    /// setting's - else the control of that name in the group.
    /// `// C#: Radio/Sikradio.cs:1201-1222; SikRadio/RFDLib/GUI/Settings.cs:250-261`
    fn find_control(&mut self, side: Side, name: &str) -> Option<(Side, usize)> {
        for (pair_side, label, editor, alternates) in DYNAMIC {
            if let Some((_, text)) = alternates.iter().find(|(setting, _)| *setting == name) {
                self.with(pair_side, label, |c| (*text).clone_into(&mut c.text));
                return self.group(pair_side).index(editor).map(|i| (pair_side, i));
            }
        }
        self.group(side).find(name).map(|i| (side, i))
    }

    /// `ExtraParamControlsSet.SetModel`: the six pairs as the Designer named them, then renamed,
    /// relisted and shown for the firmware.
    /// `// C#: Radio/Models.cs:67-176`
    #[allow(clippy::too_many_lines)]
    fn set_model(&mut self, side: Side, model: Model, settings: &Settings) {
        let p = side.prefix();
        let pair = |i: usize| -> (String, String) {
            EXTRA
                .get(i)
                .map(|(l, c)| (named(side, l), named(side, c)))
                .unwrap_or_default()
        };
        for i in 0..EXTRA.len() {
            let (label, combo) = pair(i);
            if let Some(spec) = Self::spec(side, &label) {
                self.with(side, &label, |c| spec.text.clone_into(&mut c.text));
            }
            let designer = self.ctl(side, &combo).map(|c| c.designer);
            if let Some(designer) = designer {
                self.with(side, &combo, |c| designer.clone_into(&mut c.name));
            }
        }
        let rename = |page: &mut Self, i: usize, name: String, text: Option<&str>| {
            let (label, combo) = pair(i);
            page.with(side, &combo, |c| c.name = name);
            if let Some(text) = text {
                page.with(side, &label, |c| text.clone_into(&mut c.text));
            }
        };
        let list = |page: &mut Self, i: usize, values: &[i32]| {
            let (_, combo) = pair(i);
            page.with(side, &combo, |c| c.set_ints(values));
        };
        let node_dest = || {
            let mut values = range(0, 1, 29);
            values.push(65535);
            values
        };
        match model {
            Model::P2p => self.set_extra_visible(side, false),
            Model::Multipoint => {
                rename(self, 1, format!("{p}NODEDESTINATION"), None);
                rename(self, 2, format!("{p}SYNCANY"), Some("Sync Any"));
                rename(self, 3, format!("{p}NODECOUNT"), Some("Node Count"));
                list(self, 0, &range(0, 1, 29));
                list(self, 1, &node_dest());
                list(self, 2, &range(0, 1, 1));
                list(self, 3, &range(2, 1, 30));
                self.set_extra_visible(side, false);
                for i in 0..4 {
                    self.set_pair_visible(side, i, true);
                }
            }
            Model::Async => {
                rename(self, 1, format!("{p}DESTID"), None);
                rename(self, 2, format!("{p}TX_ENCAP_METHOD"), Some("TXENCAP"));
                rename(self, 3, format!("{p}RX_ENCAP_METHOD"), Some("RXENCAP"));
                rename(self, 4, format!("{p}MAX_DATA"), Some("Max Data"));
                list(self, 0, &range(1, 1, 32767));
                list(self, 1, &range(1, 1, 65535));
                list(self, 2, &range(0, 1, 2));
                list(self, 3, &range(0, 1, 2));
                self.set_extra_visible(side, true);
            }
            Model::MultipointX => {
                rename(self, 1, format!("{p}NODEDESTINATION"), None);
                if settings.contains("NETCOUNT") {
                    rename(self, 2, format!("{p}NETCOUNT"), Some("Net Count"));
                } else {
                    rename(self, 2, format!("{p}NODECOUNT"), Some("Node Count"));
                }
                rename(self, 3, format!("{p}SERBREAKMS10"), Some("Ser. brk. x10ms"));
                if settings.contains("MASTERBACKUP") {
                    rename(self, 4, format!("{p}MASTERBACKUP"), Some("Master Bckp"));
                }
                rename(self, 5, format!("{p}RXFRAME"), Some("Rx Frame"));
                list(self, 0, &range(0, 1, 29));
                list(self, 1, &node_dest());
                list(self, 2, &range(0, 1, 1));
                list(self, 3, &range(2, 1, 30));
                list(self, 4, &range(0, 1, 1));
                self.set_extra_visible(side, false);
                for i in 0..4 {
                    self.set_pair_visible(side, i, true);
                }
                if settings.contains("MASTERBACKUP") {
                    self.set_pair_visible(side, 4, true);
                }
                if settings.contains("RXFRAME") {
                    self.set_pair_visible(side, 5, true);
                }
            }
        }
    }

    fn set_pair_visible(&mut self, side: Side, i: usize, visible: bool) {
        if let Some((label, combo)) = EXTRA.get(i) {
            self.with(side, &named(side, label), |c| c.visible = visible);
            self.with(side, &named(side, combo), |c| c.visible = visible);
        }
    }

    /// `SetAllVisible`.
    fn set_extra_visible(&mut self, side: Side, visible: bool) {
        for i in 0..EXTRA.len() {
            self.set_pair_visible(side, i, visible);
        }
        for other in EXTRA_OTHERS {
            self.with(side, &named(side, other), |c| c.visible = visible);
        }
    }

    // ---- filling the controls ----

    /// `GetDoesCheckboxHaveOnlyOneOption`.
    /// `// C#: Radio/Sikradio.cs:1230-1258`
    fn only_one_option(settings: &Settings, name: &str) -> bool {
        settings.get(name).filter(|s| s.is_full()).is_some_and(|s| {
            s.options().is_some_and(|o| o.len() == 1)
                || s.range().is_some_and(|r| r.options().len() == 1)
        })
    }

    /// `SetupCBWithSetting`: the setting's options - the value added when it is not one of
    /// them - or its range; else just the text. Whether the setting had either.
    /// `// C#: Radio/Sikradio.cs:1092-1127`
    fn setup_cb_with_setting(ctl: &mut Ctl, settings: &Settings, value: &str, name: &str) -> bool {
        if let Some(setting) = settings.get(name).filter(|s| s.is_full()) {
            if let Some(mut names) = setting.option_names() {
                let shown = match setting.option_name_for_value(value) {
                    Some(option) => option.to_owned(),
                    None => {
                        names.push(value.to_owned());
                        value.to_owned()
                    }
                };
                ctl.set_strings(&names);
                ctl.set_text(&shown);
                ctl.tag = Some(setting.clone());
                return true;
            }
            if let Some(range) = setting.range() {
                ctl.set_ints(&range.options_including_value(setting.value().unwrap_or(0)));
                ctl.set_text(value);
                ctl.tag = None;
                return true;
            }
        }
        ctl.tag = None;
        ctl.set_text(value);
        false
    }

    /// `UpdateControlWithValue`: the control enabled under its group, given the value. Whether
    /// the value was not one the control could show; the C#'s exception as an error.
    /// `// C#: Radio/Sikradio.cs:1300-1355`
    fn update_control_with_value(
        &mut self,
        found: (Side, usize),
        settings: &Settings,
        name: &str,
        value: &str,
    ) -> Result<bool, String> {
        let mut invalid = false;
        self.group_mut(found.0).enabled = true;
        let only_one = Self::only_one_option(settings, name);
        let mavlink_options = settings
            .get("MAVLINK")
            .is_some_and(|s| s.is_full() && s.options().is_some());
        let full = settings.get(name).filter(|s| s.is_full()).is_some();
        let Some(ctl) = self.at_mut(found) else {
            return Ok(false);
        };
        ctl.enabled = true;
        match ctl.kind {
            Kind::Check => {
                ctl.checked = value == "1";
                // A setting that can only be one value cannot be changed.
                if only_one {
                    ctl.enabled = false;
                }
            }
            Kind::Text => value.clone_into(&mut ctl.text),
            _ if full => {
                if ctl.name.contains("MAVLINK") && !mavlink_options {
                    let name = mavlink_name(value)?;
                    ctl.set_text(&name);
                } else if ctl.kind == Kind::Combo
                    && !Self::setup_cb_with_setting(ctl, settings, value, name)
                {
                    ctl.set_text(value);
                    if ctl.shown() != value {
                        invalid = true;
                    }
                }
            }
            _ => {}
        }
        Ok(invalid)
    }

    /// `UpdateControlsWithValues`: each setting given to the control of its name. The name is
    /// the editor's, `R` and all, which the remote radio's combo boxes find no setting of - as
    /// in the C#.
    /// `// C#: Radio/Sikradio.cs:1357-1372`
    fn update_controls_with_values(
        &mut self,
        side: Side,
        settings: &Settings,
    ) -> Result<(), String> {
        for (key, setting) in settings.iter() {
            let name = format!("{}{key}", side.prefix());
            if let Some(found) = self.find_control(side, &name) {
                self.update_control_with_value(found, settings, &name, &setting.value_as_string())?;
            }
        }
        Ok(())
    }

    /// `SetUpControlsWithValues`: each `S1:NAME=value` line given to the control of its name,
    /// or a spare one for a setting with a range or options; whether a value was not one its
    /// control could show.
    /// `// C#: Radio/Sikradio.cs:1380-1443`
    fn set_up_controls_with_values(
        &mut self,
        side: Side,
        items: &[String],
        settings: &Settings,
    ) -> Result<bool, String> {
        let names: Vec<String> = settings.names().map(str::to_owned).collect();
        self.set_up_pairs(Side::Local, &names);
        let remote_names: Vec<String> = names.iter().map(|n| format!("R{n}")).collect();
        self.set_up_pairs(Side::Remote, &remote_names);
        let mut invalid = false;
        for item in items {
            let values: Vec<&str> = item.split([':', '=']).filter(|v| !v.is_empty()).collect();
            let [_, name, value] = values.as_slice() else {
                continue;
            };
            let setting_name = name.trim();
            let editor = name.replace('/', "_").trim().to_owned();
            let full = format!("{}{editor}", side.prefix());
            let mut found = self.find_control(side, &full);
            if found.is_none()
                && let Some(setting) = settings.get(setting_name).filter(|s| s.is_full())
                && (setting.range().is_some() || setting.options().is_some())
            {
                found = self.spare_editor(side, &full, setting);
            }
            if let Some(found) = found {
                self.group_mut(side).enabled = true;
                invalid |=
                    self.update_control_with_value(found, settings, setting_name, value.trim())?;
            }
        }
        Ok(invalid)
    }

    /// `ResetAllControls`: every control disabled, text boxes emptied, combo boxes at their
    /// first item, check boxes cleared.
    /// `// C#: Radio/Sikradio.cs:2196-2228`
    fn reset_all_controls(&mut self, side: Side) {
        for ctl in &mut self.group_mut(side).ctls {
            ctl.enabled = false;
            match ctl.kind {
                Kind::Text => ctl.text.clear(),
                Kind::Combo if !ctl.items.is_empty() => ctl.selected = Some(0),
                Kind::Check => ctl.checked = false,
                _ => {}
            }
        }
    }

    /// `SetupCBWithDefaultEncryptionOptions`: 0 and 1.
    /// `// C#: Radio/Sikradio.cs:1075-1080`
    fn default_encryption_options(&mut self, side: Side) {
        self.with(side, &named(side, "ENCRYPTION_LEVEL"), |c| {
            c.tag = None;
            c.set_ints(&range(0, 1, 1));
            c.set_text("0");
        });
    }

    /// `EnableConfigControls`.
    /// `// C#: Radio/Sikradio.cs:2186-2194`
    fn enable_config_controls(&mut self, enable: bool, save: bool) {
        self.local.enabled = save;
        self.remote.enabled = save;
        self.with(Side::Top, "BUT_Syncoptions", |c| c.enabled = enable);
        self.with(Side::Local, "BUT_SetPPMFailSafe", |c| c.enabled = enable);
        self.with(Side::Top, "BUT_getcurrent", |c| c.enabled = enable);
        self.with(Side::Top, "BUT_savesettings", |c| c.enabled = save);
        self.with(Side::Top, "BUT_resettodefault", |c| c.enabled = enable);
    }

    /// `EnableProgrammingControls`.
    /// `// C#: Radio/Sikradio.cs:2230-2233`
    fn enable_programming_controls(&mut self, enable: bool) {
        self.with(Side::Top, "BUT_loadcustom", |c| c.enabled = enable);
    }

    /// `UpdateSetPPMFailSafeButtons`: each enabled when its R/C output is, and ticked.
    /// `// C#: Radio/Sikradio.cs:1905-1909`
    fn update_ppm_buttons(&mut self) {
        let local = self.enabled(Side::Local, "GPO1_1R_COUT")
            && self
                .ctl(Side::Local, "GPO1_1R_COUT")
                .is_some_and(|c| c.checked);
        let remote = self.enabled(Side::Remote, "RGPO1_1R_COUT")
            && self
                .ctl(Side::Remote, "RGPO1_1R_COUT")
                .is_some_and(|c| c.checked);
        self.with(Side::Local, "BUT_SetPPMFailSafe", |c| c.enabled = local);
        self.with(Side::Remote, "BUT_SetPPMFailSafeRemote", |c| {
            c.enabled = remote
        });
    }

    /// `RestoreAllDefaultCBObjects`.
    /// `// C#: Radio/Sikradio.cs:311-330`
    fn restore_default_lists(&mut self) {
        let lists = self.default_lists.clone();
        for (side, designer, items, items_based) in lists {
            self.with(side, designer, |c| {
                if items_based {
                    c.items = items;
                    c.selected = None;
                } else {
                    c.set_items(items);
                }
            });
        }
    }

    fn both(&mut self, local: &str, values: &[i32]) {
        for side in [Side::Local, Side::Remote] {
            self.with(side, &named(side, local), |c| c.set_ints(values));
        }
    }

    /// The controls before Load Settings goes to the radio: `BUT_getcurrent_Click`'s start.
    /// `// C#: Radio/Sikradio.cs:1478-1490`
    fn before_load(&mut self) {
        self.in_encryption = true;
        self.reset_all_controls(Side::Local);
        self.reset_all_controls(Side::Remote);
        self.default_encryption_options(Side::Local);
        self.default_encryption_options(Side::Remote);
        self.enable_config_controls(false, false);
        self.enable_programming_controls(false);
        self.set_status("Connecting");
    }

    /// What Load Settings read, given to the controls in the C#'s order; to where it got, and
    /// "Error" where the C# caught an exception.
    /// `// C#: Radio/Sikradio.cs:1491-1903`
    fn apply_loaded(&mut self, loaded: &Loaded) {
        let result = self.apply_loaded_inner(loaded);
        let failed = loaded.error.is_some() || result.is_err();
        if let Err(error) = result {
            // The exception thrown filling the controls, caught as the radio's are.
            self.set_status("Error");
            self.status_line = Some(format!("Error during read {error}"));
        }
        if !failed {
            if loaded.entered {
                self.enable_config_controls(true, true);
            } else {
                self.enable_config_controls(true, false);
            }
            self.enable_programming_controls(true);
        }
        self.in_encryption = false;
        self.update_ppm_buttons();
    }

    #[allow(clippy::too_many_lines)]
    fn apply_loaded_inner(&mut self, loaded: &Loaded) -> Result<(), String> {
        if !loaded.entered {
            return Ok(());
        }
        let Some(ati) = &loaded.ati else {
            return Ok(());
        };
        self.with(Side::Local, "ATI", |c| ati.clone_into(&mut c.text));
        let Some(freq) = loaded.freq else {
            return Ok(());
        };
        self.with(Side::Local, "ATI3", |c| c.text = freq.to_string());
        let (Some(board), Some(country)) = (loaded.board, &loaded.country) else {
            return Ok(());
        };
        self.with(Side::Local, "txtCountry", |c| {
            country.clone_into(&mut c.text)
        });
        self.with(Side::Local, "ATI2", |c| c.text = board.to_string());
        if board == Board::RFD900X {
            // The RFD900x has its own ranges.
            self.both("SERIAL_SPEED", &[1, 2, 4, 9, 19, 38, 57, 115, 230, 460]);
            self.both("AIR_SPEED", &[4, 64, 125, 250, 500]);
            if ati.contains("ASYNC") {
                self.both("NETID", &range(0, 1, 255));
            } else {
                self.both("NETID", &range(0, 1, 65535));
            }
            self.both("MIN_FREQ", &range(902_000, 1000, 927_000));
            self.both("MAX_FREQ", &range(903_000, 1000, 928_000));
            self.both("NUM_CHANNELS", &range(1, 1, 50));
            self.both("MAX_WINDOW", &range(20, 1, 400));
        } else {
            self.restore_default_lists();
        }
        if freq == Frequency::F915 {
            self.both("MIN_FREQ", &range(902_000, 1000, 927_000));
            self.both("MAX_FREQ", &range(903_000, 1000, 928_000));
        } else if freq == Frequency::F433 {
            self.both("MIN_FREQ", &range(414_000, 50, 460_000));
            self.both("MAX_FREQ", &range(414_000, 50, 460_000));
        } else if freq == Frequency::F868 {
            self.both("MIN_FREQ", &range(849_000, 1000, 889_000));
            self.both("MAX_FREQ", &range(849_000, 1000, 889_000));
        }
        if [
            Board::RFD900,
            Board::RFD900A,
            Board::RFD900P,
            Board::RFD900X,
        ]
        .contains(&board)
        {
            self.both("TXPOWER", &range(0, 1, 30));
        } else {
            self.both("TXPOWER", &range(0, 1, 20));
        }
        if board == Board::RFD900X {
            self.both("LBT_RSSI", &range(0, 25, 220));
        } else {
            self.both("LBT_RSSI", &range(0, 1, 1));
        }
        let Some(key) = &loaded.key else {
            return Ok(());
        };
        self.set_key(Side::Local, key);
        self.mavlink(Side::Local, loaded.simple_mavlink);
        let Some(rssi) = &loaded.rssi else {
            return Ok(());
        };
        self.with(Side::Local, "RSSI", |c| rssi.clone_into(&mut c.text));
        let (Some(answer), Some(settings)) = (&loaded.ati5, &loaded.settings) else {
            return Ok(());
        };
        self.local_settings = Some(settings.clone());
        // `DisableRFD900xControls`.
        for (side, name) in [
            (Side::Local, "GPI1_1R_CIN"),
            (Side::Remote, "RGPI1_1R_CIN"),
            (Side::Local, "GPO1_1R_COUT"),
            (Side::Remote, "RGPO1_1R_COUT"),
        ] {
            self.with(side, name, |c| c.enabled = false);
        }
        let items: Vec<String> = answer.split('\n').map(str::to_owned).collect();
        // `_DefaultLocalEnabled`: each control of both groups as the Designer enabled it.
        for side in [Side::Local, Side::Remote] {
            for spec in side.specs() {
                self.with(side, spec.name, |c| c.enabled = spec.enabled);
            }
        }
        self.with(Side::Local, "btnSaveToFile", |c| c.enabled = true);
        self.with(Side::Local, "btnLoadFromFile", |c| c.enabled = true);
        self.reset_pairs(Side::Local);
        let model = Self::model_of(ati, board, &items);
        self.set_model(Side::Local, model, settings);
        let fixed = modify_for_multipoint(&items, loaded.multipoint_fix);
        let mut invalid = self.set_up_controls_with_values(Side::Local, &fixed, settings)?;
        let random = self.encryption_enabled(Side::Local);
        self.with(Side::Local, "btnRandom", |c| c.enabled = random);
        // The remote radio's controls emptied, but its labels, buttons and the RSSI.
        for ctl in &mut self.remote.ctls {
            if ctl.name != "RSSI" && !matches!(ctl.kind, Kind::Label | Kind::Button) {
                match ctl.kind {
                    Kind::Combo => ctl.set_text(""),
                    Kind::Check => {}
                    _ => ctl.text.clear(),
                }
            }
        }
        let Some(rti) = &loaded.rti else {
            return Ok(());
        };
        self.with(Side::Remote, "RTI", |c| rti.clone_into(&mut c.text));
        if let Some(country) = &loaded.remote_country {
            self.with(Side::Remote, "txtRCountry", |c| {
                country.clone_into(&mut c.text)
            });
        }
        if let Some(remote) = &loaded.remote {
            if let Some(rti2) = &remote.rti2 {
                self.with(Side::Remote, "RTI2", |c| rti2.clone_into(&mut c.text));
            }
            self.set_key(Side::Remote, &remote.key);
            self.mavlink(Side::Remote, remote.simple_mavlink);
            self.remote_settings = Some(remote.settings.clone());
            let items: Vec<String> = remote.rti5.split('\n').map(str::to_owned).collect();
            self.reset_pairs(Side::Remote);
            self.with(Side::Remote, "btnRemoteSaveToFile", |c| c.enabled = true);
            self.with(Side::Remote, "btnRemoteLoadFromFile", |c| c.enabled = true);
            let model = Self::model_of(rti, board, &items);
            self.set_model(Side::Remote, model, &remote.settings);
            invalid |= self.set_up_controls_with_values(Side::Remote, &items, &remote.settings)?;
        }
        if loaded.error.is_none() {
            self.set_status(if invalid {
                "Done.  Some settings in modem were invalid."
            } else {
                "Done"
            });
        }
        Ok(())
    }

    /// The firmware's model from the version and the `ATI5` lines.
    /// `// C#: Radio/Sikradio.cs:1717-1735, 1832-1851`
    fn model_of(ati: &str, board: Board, items: &[String]) -> Model {
        if ati.contains("ASYNC") {
            Model::Async
        } else if ati.contains("MP on") && board == Board::RFD900X {
            Model::MultipointX
        } else if items.first().is_some_and(|i| i.starts_with('[')) {
            Model::Multipoint
        } else {
            Model::P2p
        }
    }

    fn set_key(&mut self, side: Side, key: &KeyBox) {
        let name = named(side, "AESKEY");
        self.with(side, &name, |c| match key {
            KeyBox::Shown(text) => {
                text.clone_into(&mut c.text);
                c.enabled = true;
            }
            KeyBox::Disabled => {
                c.text.clear();
                c.enabled = false;
            }
        });
    }

    // ---- reading the controls ----

    /// `GetSettingFromControl`: a check box's 0 or 1, a text box's text, a combo box's number.
    /// `// C#: Radio/Sikradio.cs:565-587`
    fn setting_from_control(&self, found: (Side, usize)) -> Option<GuiValue> {
        let ctl = self.at(found)?;
        match ctl.kind {
            Kind::Check => Some(GuiValue::Int(i32::from(ctl.checked))),
            Kind::Text => Some(GuiValue::Text(ctl.text.clone())),
            Kind::Combo => try_parse_int(&Self::cb_value(ctl)).map(GuiValue::Int),
            _ => None,
        }
    }

    /// `GetUpdatedSettingsFromGroupBox`: each of the radio's settings its control now shows
    /// differently, with the new value.
    /// `// C#: Radio/Sikradio.cs:640-666`
    fn updated_settings(&mut self, side: Side, orig: &Settings) -> Settings {
        let mut result = Settings::new();
        for (key, setting) in orig.iter() {
            let name = format!("{}{key}", side.prefix());
            let Some(found) = self.find_control(side, &name) else {
                continue;
            };
            let Some(value) = self.setting_from_control(found) else {
                continue;
            };
            // `GetIsDifferent`, `UpdateSetting`.
            match value {
                GuiValue::Int(v) if setting.is_full() && setting.value() != Some(v) => {
                    let mut updated = setting.clone();
                    updated.set_value(v);
                    result.insert(key, updated);
                }
                GuiValue::Text(t) if setting.text_value().is_some_and(|old| old != t) => {
                    let mut updated = setting.clone();
                    updated.set_value_from_string(&t);
                    result.insert(key, updated);
                }
                _ => {}
            }
        }
        result
    }

    /// `CheckSettingsValid`: the radio's settings with the controls' changes, checked; the
    /// box when they are not valid. The C#'s `Remote ? "Remote" : "Local" + " settings invalid,
    /// ..."` binds the `+` to "Local" only, so the remote radio's box says just "Remote".
    /// `// C#: Radio/Sikradio.cs:674-702`
    fn check_settings_valid(&mut self, side: Side, orig: &Settings) -> bool {
        let changed = self.updated_settings(side, orig);
        let mut settings = orig.clone();
        for (name, setting) in changed.iter() {
            settings.insert(name, setting.clone());
        }
        let errors = settings.check_valid();
        if errors.is_empty() {
            return true;
        }
        let mut text = if side == Side::Remote {
            "Remote".to_owned()
        } else {
            "Local settings invalid, operation aborted:".to_owned()
        };
        for error in errors {
            text.push_str("\n\t");
            text.push_str(&error);
        }
        self.show(text);
        false
    }

    /// The changes a radio is to be written, as `SaveSettingsFromGroupBox` reads them.
    fn changes(&mut self, side: Side, orig: &Settings) -> Vec<Change> {
        self.updated_settings(side, orig)
            .iter()
            .map(|(name, setting)| Change {
                name: name.to_owned(),
                designator: setting.designator.clone(),
                value: setting.value_as_string(),
                is_one: setting.is_full() && setting.value() == Some(1),
            })
            .collect()
    }

    /// The key a radio is to be written when its level encrypts: trimmed in its box, then
    /// padded to the level's length - or the length it was not valid for.
    /// `// C#: Radio/Sikradio.cs:934-969`
    fn key_plan(&mut self, side: Side) -> wire::KeyPlan {
        if !self.encryption_enabled(side) {
            return None;
        }
        let level = named(side, "ENCRYPTION_LEVEL");
        let max = self.ctl(side, &level).map_or(0, Self::max_key_length);
        let name = named(side, "AESKEY");
        self.with(side, &name, |c| c.text = c.text.trim().to_owned());
        let key = self.text_of(side, &name);
        let hex = !key.is_empty() && key.chars().all(|c| c.is_ascii_hexdigit());
        let length = i32::try_from(key.len()).unwrap_or(i32::MAX);
        if hex && length <= max {
            let width = usize::try_from(max).unwrap_or(0);
            Some(Ok(format!("{key:0<width$}")))
        } else {
            Some(Err(max))
        }
    }

    /// Save Settings' checks and what it is to write: `None` when a check failed (its box
    /// showing).
    /// `// C#: Radio/Sikradio.cs:849-873, 934-969`
    fn save_plan(&mut self) -> Option<SavePlan> {
        let remote_there = !self.text_of(Side::Remote, "RTI").is_empty();
        let local = self.local_settings.clone()?;
        let remote_ok = !remote_there
            || self
                .remote_settings
                .clone()
                .is_some_and(|s| self.check_settings_valid(Side::Remote, &s));
        if !remote_ok || !self.check_settings_valid(Side::Local, &local) {
            return None;
        }
        let remote = if remote_there {
            self.remote_settings
                .clone()
                .map(|s| self.changes(Side::Remote, &s))
        } else {
            None
        };
        let local_changes = self.changes(Side::Local, &local);
        let remote_key = self.key_plan(Side::Remote);
        let local_key = self.key_plan(Side::Local);
        Some(SavePlan {
            remote,
            local: local_changes,
            remote_key,
            local_key,
        })
    }

    // ---- the handlers that touch only the page ----

    /// `BUT_Syncoptions_Click`: the settings both radios must share copied to the remote one's
    /// controls. A remote encryption level that changes is written at once, its key box then
    /// given the local key, as the C#'s handler runs before the key is copied.
    /// `// C#: Radio/Sikradio.cs:2076-2088`
    fn sync_options(&mut self) -> Option<Work> {
        let copy = |page: &mut Self, name: &str| {
            let text = page.text_of(Side::Local, name);
            page.with(Side::Remote, &named(Side::Remote, name), |c| {
                c.set_text(&text)
            });
        };
        copy(self, "AIR_SPEED");
        copy(self, "NETID");
        let ecc = self.ctl(Side::Local, "ECC").is_some_and(|c| c.checked);
        self.with(Side::Remote, "RECC", |c| c.checked = ecc);
        copy(self, "MAVLINK");
        copy(self, "MIN_FREQ");
        copy(self, "MAX_FREQ");
        copy(self, "NUM_CHANNELS");
        copy(self, "MAX_WINDOW");
        let before = self.encryption_state(Side::Remote);
        let index = self
            .ctl(Side::Local, "ENCRYPTION_LEVEL")
            .and_then(|c| c.selected);
        let count = self
            .ctl(Side::Remote, "RENCRYPTION_LEVEL")
            .map_or(0, |c| c.items.len());
        if let Some(i) = index
            && i >= count
        {
            // `SelectedIndex = n` past the list: the C#'s unhandled exception.
            self.status_line = Some(format!(
                "InvalidArgument=Value of '{i}' is not valid for 'SelectedIndex'."
            ));
            return None;
        }
        self.with(Side::Remote, "RENCRYPTION_LEVEL", |c| c.selected = index);
        let key = self.text_of(Side::Local, "AESKEY");
        match self.encryption_changed(Side::Remote, before) {
            Some(Work::Encryption { remote, level, .. }) => Some(Work::Encryption {
                remote,
                level,
                then_key: Some(key),
            }),
            _ => {
                self.with(Side::Remote, "RAESKEY", |c| c.text = key);
                None
            }
        }
    }

    /// An encryption level combo box's items and selection, to tell when they change.
    fn encryption_state(&self, side: Side) -> (Vec<Item>, Option<usize>) {
        self.ctl(side, &named(side, "ENCRYPTION_LEVEL"))
            .map(|c| (c.items.clone(), c.selected))
            .unwrap_or_default()
    }

    /// `ENCRYPTION_LEVEL_CheckedChanged` and `RENCRYPTION_LEVEL_CheckedChanged`, when the box's
    /// selection has changed since `before`: the level written at once while the box is
    /// enabled and no load is filling it; the local one's Random enabled by it.
    /// `// C#: Radio/Sikradio.cs:2497-2517, 2543-2549`
    fn encryption_changed(
        &mut self,
        side: Side,
        before: (Vec<Item>, Option<usize>),
    ) -> Option<Work> {
        if self.encryption_state(side) == before {
            return None;
        }
        let name = named(side, "ENCRYPTION_LEVEL");
        let work = (self.enabled(side, &name) && !self.in_encryption).then(|| Work::Encryption {
            remote: side == Side::Remote,
            level: self.ctl(side, &name).map_or(0, Self::level_value),
            then_key: None,
        });
        if side == Side::Local {
            let random = self.encryption_enabled(Side::Local);
            self.with(Side::Local, "btnRandom", |c| c.enabled = random);
        }
        work
    }

    /// `btnRandom_Click`: a key of the local level's length, in both key boxes.
    /// `// C#: Radio/Sikradio.cs:2596-2614`
    fn random(&mut self) {
        let length = self
            .ctl(Side::Local, "ENCRYPTION_LEVEL")
            .map_or(0, Self::max_key_length);
        let key = random_key(length);
        self.with(Side::Local, "AESKEY", |c| c.text.clone_from(&key));
        self.with(Side::Remote, "RAESKEY", |c| c.text = key);
    }

    /// An item chosen from a combo box's list; the encryption level's handler for its boxes.
    pub fn choose(&mut self, side: Side, index: usize, item: usize) -> Option<Work> {
        self.open_list = None;
        let before = self.encryption_state(side);
        if let Some(ctl) = self.at_mut((side, index))
            && item < ctl.items.len()
        {
            ctl.selected = Some(item);
        }
        let is_level = self
            .at((side, index))
            .is_some_and(|c| c.designer == named(side, "ENCRYPTION_LEVEL"));
        if is_level {
            return self.encryption_changed(side, before);
        }
        None
    }

    /// A check box clicked.
    pub fn toggle(&mut self, side: Side, index: usize) {
        if let Some(ctl) = self.at_mut((side, index)) {
            ctl.checked = !ctl.checked;
        }
    }

    /// A combo box's list dropped down, or put away.
    pub fn toggle_list(&mut self, side: Side, index: usize) {
        if self
            .open_list
            .is_some_and(|(s, i, _)| (s, i) == (side, index))
        {
            self.open_list = None;
            return;
        }
        let Some(ctl) = self.at((side, index)) else {
            return;
        };
        let mut combo = combo_of(ctl);
        combo.open_list();
        self.open_list = Some((side, index, combo.top_index));
    }

    /// The wheel over a dropped-down list.
    pub fn scroll_list(&mut self, lines: i32) {
        let Some((side, index, top)) = self.open_list else {
            return;
        };
        let Some(ctl) = self.at((side, index)) else {
            return;
        };
        let mut combo = combo_of(ctl);
        combo.top_index = top;
        combo.scroll_list(lines);
        self.open_list = Some((side, index, combo.top_index));
    }

    /// `SaveToFile`'s dialog: the radio's settings with the controls' changes, to be written.
    /// `// C#: Radio/Sikradio.cs:2632-2663`
    fn save_to_file(&mut self, remote: bool) {
        let side = if remote { Side::Remote } else { Side::Local };
        let Some(orig) = (if remote {
            self.remote_settings.clone()
        } else {
            self.local_settings.clone()
        }) else {
            return;
        };
        let mut updated = self.updated_settings(side, &orig);
        for (name, setting) in orig.iter() {
            if !updated.contains(name) {
                updated.insert(name, setting.clone());
            }
        }
        let mut path = PathBox::new("", INI_FILTER);
        path.caption = "Save As";
        self.prompt = Some(Prompt::Save {
            path,
            text: updated.to_file_text(),
        });
    }

    /// `LoadFromFile`'s dialog.
    /// `// C#: Radio/Sikradio.cs:2676-2705`
    fn load_from_file(&mut self, remote: bool) {
        let settings = if remote {
            &self.remote_settings
        } else {
            &self.local_settings
        };
        if settings.is_none() {
            return;
        }
        self.prompt = Some(Prompt::Open {
            path: PathBox::new("", INI_FILTER),
            remote,
        });
    }

    /// `LoadFromFile` once the file is chosen: the radio's settings with the file's, given to
    /// the controls, and the box saying what was loaded. A level the file changes is written at
    /// once, as the C#'s handler runs.
    /// `// C#: Radio/Sikradio.cs:2676-2705; SikRadio/RFD900.cs:1389-1421`
    fn load_file(&mut self, remote: bool, file: &str) -> Option<Work> {
        let side = if remote { Side::Remote } else { Side::Local };
        let mut settings = if remote {
            self.remote_settings.clone()
        } else {
            self.local_settings.clone()
        }?;
        let Ok(text) = std::fs::read_to_string(file) else {
            self.show(format!("Failed to load settings from {file}"));
            return None;
        };
        let loaded = settings.load_from_text(&text);
        let before = self.encryption_state(side);
        if let Err(error) = self.update_controls_with_values(side, &settings) {
            self.status_line = Some(error);
            return None;
        }
        let work = self.encryption_changed(side, before);
        let mut report = "Loaded\n".to_owned();
        for (name, value) in loaded {
            report.push_str(&format!("{name} = {value}\n"));
        }
        report.push_str(&format!("from {file} OK"));
        self.show(report);
        work
    }

    /// A key pressed in the dialog showing.
    pub fn prompt_key(&mut self, event: &KeyDownEvent) -> (bool, Option<bool>) {
        let field = match &mut self.prompt {
            Some(Prompt::Tcp { input, .. }) => &mut input.field,
            Some(
                Prompt::Save { path, .. }
                | Prompt::Open { path, .. }
                | Prompt::Firmware { path, .. },
            ) => &mut path.field,
            Some(Prompt::TryAgain { .. }) | None => return (false, None),
        };
        match field.key(event) {
            KeyOutcome::Changed => (true, None),
            KeyOutcome::Submitted => (true, Some(true)),
            KeyOutcome::Cancelled => (true, Some(false)),
            KeyOutcome::Ignored => (false, None),
        }
    }

    /// A file dialog's OK or Cancel: the file written or read, or the firmware answered. A path
    /// to no file leaves an Open dialog showing, as `OpenFileDialog` refuses it.
    fn path_done(&mut self, ok: bool) -> Option<Work> {
        match self.prompt.take() {
            Some(Prompt::Save { path, text }) => {
                let file = path.field.value().trim().to_owned();
                if ok && !file.is_empty() {
                    if std::fs::write(&file, text).is_ok() {
                        self.show(format!("Saved settings to {file} OK"));
                    } else {
                        self.show(format!("Failed to save settings to {file}"));
                    }
                }
                None
            }
            Some(Prompt::Open { path, remote }) => {
                if !ok {
                    return None;
                }
                match path.chosen() {
                    Some(file) => self.load_file(remote, &file.display().to_string()),
                    None => {
                        self.prompt = Some(Prompt::Open { path, remote });
                        None
                    }
                }
            }
            Some(Prompt::Firmware { path, reply }) => {
                if !ok {
                    let _ = reply.send(None);
                    return None;
                }
                match path.chosen() {
                    Some(file) => {
                        let _ = reply.send(Some(file));
                    }
                    None => self.prompt = Some(Prompt::Firmware { path, reply }),
                }
                None
            }
            other => {
                self.prompt = other;
                None
            }
        }
    }

    /// "Try again?" answered.
    fn answer_again(&mut self, yes: bool) {
        if let Some(Prompt::TryAgain { reply }) = self.prompt.take() {
            let _ = reply.send(yes);
        }
    }

    // ---- the handlers' radio halves ----

    /// The controls a handler disables before it goes to the radio.
    fn before(&mut self, work: &Work) {
        match work {
            Work::Load => self.before_load(),
            // `BUT_savesettings_Click`: `Connecting` after the checks.
            Work::Save(_) => {
                self.enable_config_controls(false, false);
                self.enable_programming_controls(false);
                self.set_status("Connecting");
            }
            // `ProgramFirmware`'s start.
            Work::Program => {
                self.enable_programming_controls(false);
                self.enable_config_controls(false, false);
            }
            Work::Reset(_) | Work::Ppm(_) => self.set_status("Connecting"),
            Work::Encryption { .. } | Work::Disconnect => {}
        }
    }

    /// The handler's radio half started on its thread, on the session `GetSession` gives.
    fn start(&mut self, work: Work, wanted: &PortSpec) {
        let Some(held) = self.held.take() else {
            return;
        };
        self.before(&work);
        let (spec, session) = held.session(wanted);
        let (events, receiver) = mpsc::channel();
        let name = work.name();
        let sender = events.clone();
        let spawned = std::thread::Builder::new()
            .name("sik radio".to_owned())
            .spawn(move || {
                let mut report = Sender(sender);
                let (outcome, link) = run(work, session, &mut report);
                let _ = events.send(Event::Done(Box::new((Held { spec, link }, outcome))));
            });
        match spawned {
            Ok(_) => {
                self.job = Some(Job {
                    work: name,
                    events: receiver,
                });
            }
            Err(error) => self.status_line = Some(error.to_string()),
        }
    }

    /// What the thread said, taken in.
    fn event(&mut self, event: Event) {
        match event {
            Event::Status(text) => self.set_status(&text),
            Event::Progress(completed) => {
                // `ProgressEvtHdlr`: `Math.Min((int)(Completed * 100F), 100)`.
                #[allow(clippy::cast_possible_truncation)]
                let value = (completed * 100.0) as i32;
                self.progress = value.min(100);
            }
            Event::Message(text) => self.show(text),
            Event::Failed(text) => self.status_line = Some(text.replace('\n', " ")),
            Event::TryAgain(reply) => self.prompt = Some(Prompt::TryAgain { reply }),
            Event::Firmware(filter, reply) => {
                self.prompt = Some(Prompt::Firmware {
                    path: PathBox::new("", firmware_filter(&filter)),
                    reply,
                });
            }
            Event::Done(done) => {
                let (held, outcome) = *done;
                self.job = None;
                self.done(held, outcome);
            }
        }
    }

    /// A handler's end on the page.
    fn done(&mut self, held: Held, outcome: Outcome) {
        self.held = Some(held);
        match outcome {
            Outcome::Loaded(loaded) => self.apply_loaded(&loaded),
            Outcome::Saved(result) => match result {
                Ok(entered) => {
                    self.enable_config_controls(true, entered);
                    self.enable_programming_controls(true);
                    self.update_ppm_buttons();
                }
                Err(error) => self.status_line = Some(error.to_string()),
            },
            Outcome::Reset(result) | Outcome::Ppm(result) => {
                if let Err(error) = result {
                    self.status_line = Some(error.to_string());
                }
            }
            Outcome::Encryption {
                remote,
                then_key,
                result,
            } => {
                let side = if remote { Side::Remote } else { Side::Local };
                match result {
                    Ok(Some(key)) => self.with(side, &named(side, "AESKEY"), |c| c.text = key),
                    Ok(None) => {}
                    Err(error) => self.status_line = Some(error.to_string()),
                }
                if let Some(key) = then_key {
                    self.with(Side::Remote, "RAESKEY", |c| c.text = key);
                }
            }
            Outcome::Programmed => {
                self.enable_programming_controls(true);
                self.enable_config_controls(true, false);
            }
            Outcome::Disconnected => {
                // `FinishedWithComPortForSiKRadio`: the port closed.
                self.held = None;
            }
        }
    }

    /// Once a frame: what the thread said; and the page gone with the SETUP screen - the
    /// radio put back in transparent mode, the port closed, and a new page made next time.
    pub fn tick(&mut self, on_setup: bool) {
        while let Some(job) = &self.job {
            match job.events.try_recv() {
                Ok(event) => self.event(event),
                Err(mpsc::TryRecvError::Empty) => break,
                Err(mpsc::TryRecvError::Disconnected) => {
                    // The thread went without its end: the port with it.
                    self.job = None;
                }
            }
        }
        if !on_setup && self.created && self.job.is_none() && self.prompt.is_none() {
            // `DisposedEvtHdlr`: `Disconnect`.
            let held = self.held.take();
            *self = Self::new();
            if let Some(held) = held {
                let wanted = held.spec.clone();
                self.held = Some(held);
                self.start(Work::Disconnect, &wanted);
            }
        }
    }

    /// The facts a GUI script reads.
    fn facts(&self) -> Vec<(String, String)> {
        let mut facts = vec![
            ("active".to_owned(), self.active.to_string()),
            (
                "busy".to_owned(),
                self.job.as_ref().map_or("none", |j| j.work).to_owned(),
            ),
            ("status".to_owned(), self.status().replace('\n', " ")),
            ("progress".to_owned(), self.progress.to_string()),
            ("beta".to_owned(), self.beta.to_string()),
            (
                "message".to_owned(),
                self.message()
                    .map_or_else(|| "none".to_owned(), |m| m.text.replace(['\n', '\t'], " ")),
            ),
            (
                "prompt".to_owned(),
                match &self.prompt {
                    None => "none".to_owned(),
                    Some(Prompt::Tcp { input, .. }) => input.title.to_owned(),
                    Some(
                        Prompt::Save { path, .. }
                        | Prompt::Open { path, .. }
                        | Prompt::Firmware { path, .. },
                    ) => format!("{} {}", path.caption, path.filter),
                    Some(Prompt::TryAgain { .. }) => "Try again".to_owned(),
                },
            ),
            (
                "port".to_owned(),
                self.held.as_ref().map_or_else(
                    || "none".to_owned(),
                    |h| match &h.link {
                        Linked::Wire(w) => w.port().name.clone(),
                        Linked::Session(s) => s.wire_ref().port().name.clone(),
                    },
                ),
            ),
            (
                "mode".to_owned(),
                self.held.as_ref().map_or_else(
                    || "none".to_owned(),
                    |h| match &h.link {
                        Linked::Wire(_) => "ended".to_owned(),
                        Linked::Session(s) => s.mode().to_string(),
                    },
                ),
            ),
            ("local.enabled".to_owned(), self.local.enabled.to_string()),
            ("remote.enabled".to_owned(), self.remote.enabled.to_string()),
        ];
        for side in [Side::Top, Side::Local, Side::Remote] {
            for ctl in &self.group(side).ctls {
                let d = ctl.designer;
                facts.push((format!("{d}.visible"), ctl.visible.to_string()));
                facts.push((
                    format!("{d}.enabled"),
                    (self.group(side).enabled && ctl.enabled).to_string(),
                ));
                facts.push((format!("{d}.name"), ctl.name.clone()));
                match ctl.kind {
                    Kind::Check => facts.push((format!("{d}.checked"), ctl.checked.to_string())),
                    Kind::Combo => {
                        facts.push((format!("{d}.text"), ctl.shown()));
                        facts.push((format!("{d}.items"), ctl.items.len().to_string()));
                    }
                    _ => facts.push((
                        format!("{d}.text"),
                        ctl.text.replace(['\r', '\n'], " ").trim().to_owned(),
                    )),
                }
            }
        }
        facts
    }
}

/// What the C#'s `TBaseSetting` subclasses for the GUI's values hold.
#[derive(Debug, Clone, PartialEq, Eq)]
enum GuiValue {
    /// `TSetting<int>`.
    Int(i32),
    /// `TSetting<string>`.
    Text(String),
}

/// `ModifyReturnedStringsForMultipoint`: a multipoint radio's `[n]` taken off each line.
/// `// C#: Radio/Sikradio.cs:1185-1199`
fn modify_for_multipoint(items: &[String], fix: i64) -> Vec<String> {
    let Ok(fix) = usize::try_from(fix) else {
        return items.to_vec();
    };
    items
        .iter()
        .map(|item| {
            if fix > 0 && item.chars().count() > fix {
                item.chars().skip(fix).collect::<String>().trim().to_owned()
            } else {
                item.clone()
            }
        })
        .collect()
}

/// `GetRandomKey`: hex numerals, upper case, from a generator seeded by the clock.
/// `// C#: Radio/Sikradio.cs:2596-2608`
fn random_key(digits: i32) -> String {
    let mut state = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0x9E37_79B9_7F4A_7C15, |d| {
            u64::try_from(d.as_nanos() & u128::from(u64::MAX)).unwrap_or(1)
        })
        | 1;
    (0..digits.max(0))
        .map(|_| {
            // xorshift64*.
            state ^= state >> 12;
            state ^= state << 25;
            state ^= state >> 27;
            let nibble = (state.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 60) & 0xF;
            char::from_digit(u32::try_from(nibble).unwrap_or(0), 16)
                .unwrap_or('0')
                .to_ascii_uppercase()
        })
        .collect()
}

/// The firmware dialog's filter, as `getFirmwareLocal` builds it for each model's extensions.
fn firmware_filter(filter: &str) -> &'static str {
    match filter {
        "Firmware|*.hex;*.ihx" => "Firmware|*.hex;*.ihx",
        "Firmware|*.bin" => "Firmware|*.bin",
        "Firmware|*.gbl" => "Firmware|*.gbl",
        _ => "Firmware|*.*",
    }
}

/// A control as the shared drop-down list takes it: each item's index its key.
fn combo_of(ctl: &Ctl) -> Combo {
    Combo {
        param: ctl.name.clone(),
        options: ctl
            .items
            .iter()
            .enumerate()
            .map(|(i, item)| (i64::try_from(i).unwrap_or(0), item.text.clone()))
            .collect(),
        selected: ctl.selected.and_then(|i| i64::try_from(i).ok()),
        enabled: true,
        top_index: 0,
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(page: &SikRadio) {
    use crate::facts::record;
    for (key, value) in page.facts() {
        record(format!("config.sikradio.{key}"), value);
    }
}

// ---------------------------------------------------------------------------------------------
// The application's part: the port, and the clicks that need it.
// ---------------------------------------------------------------------------------------------

impl MissionPlanner {
    /// `MainV2.comPort.BaseStream` as `Connect` reads it: the window's link while it is open,
    /// else the connection box - TCP, or the serial port and baud rate.
    fn sikradio_wanted(&self) -> PortSpec {
        let baud = self
            .connect_box
            .baud
            .trim()
            .parse::<u32>()
            .unwrap_or(57_600);
        if self.telemetry.is_open() {
            match self
                .telemetry
                .view()
                .target
                .parse::<mp_transport::LinkUrl>()
            {
                Ok(mp_transport::LinkUrl::Tcp { .. }) => return PortSpec::Tcp { baud },
                Ok(mp_transport::LinkUrl::Serial { path, baud }) => {
                    return PortSpec::Serial { path, baud };
                }
                _ => {}
            }
        }
        if self.connect_box.port.contains("TCP") {
            PortSpec::Tcp { baud }
        } else {
            PortSpec::Serial {
                path: self.connect_box.port.clone(),
                baud,
            }
        }
    }

    /// A handler that goes to the radio: `GetSession`, opening the port the first time -
    /// `GetComPortForSiKRadio` closes the window's link first, and a TCP port asks its host and
    /// port - then the handler's radio half. Whether it asked, so the caller can give the
    /// question the focus.
    /// `// C#: Radio/ComPort.cs:10-20; Radio/Sikradio.cs:2434-2462`
    fn sikradio_go(&mut self, work: Work) -> bool {
        if self.extra.sikradio.busy() || self.extra.sikradio.prompt.is_some() {
            self.extra.sikradio.queued.push_back(work);
            return false;
        }
        let wanted = self.sikradio_wanted();
        if self.extra.sikradio.held.is_none() {
            if self.telemetry.is_open() {
                // `MainV2.comPort.Close()`, which shows no screen again.
                self.telemetry = Telemetry::idle();
                self.params_requested = false;
                self.mission_requested = false;
                let key = Key::of(&self.telemetry.view());
                self.setup_list.rekey(key);
            }
            match &wanted {
                PortSpec::Serial { path, baud } => {
                    self.extra.sikradio.held = Some(Held {
                        spec: wanted.clone(),
                        link: Linked::Wire(Wire::new(LinkPort::serial(path, *baud))),
                    });
                }
                PortSpec::Tcp { .. } => {
                    // `TcpSerial.Open`: the host, then the port, the last answers filled in.
                    let host = self.persisted.get("TCP_host").unwrap_or("127.0.0.1");
                    self.extra.sikradio.prompt = Some(Prompt::Tcp {
                        input: InputBox::new(
                            "remote host",
                            "Enter host name/ip (ensure remote end is already started)",
                            host,
                        ),
                        host: None,
                        work,
                        wanted,
                    });
                    return true;
                }
            }
        }
        self.extra.sikradio.start(work, &wanted);
        false
    }

    /// [`MissionPlanner::sikradio_go`] from a click, the question it asks given the focus.
    fn sikradio_go_focused(&mut self, work: Work, window: &mut Window, cx: &mut Context<Self>) {
        if self.sikradio_go(work) {
            self.extra_focus.sikradio.focus(window, cx);
        }
    }

    /// An `InputBox` of the TCP port answered: the port next, or the socket opened - and the
    /// answers kept - then the handler that asked. Cancel keeps the port unopened, as the C#'s
    /// `Connect` leaves it, and the handler goes on to fail on it.
    /// `// C#: ExtLibs/Comms/CommsTCPSerial.cs:117-157; ExtLibs/Controls/InputBox.cs:178-184`
    fn sikradio_input_done(&mut self, ok: bool, window: &mut Window, cx: &mut Context<Self>) {
        let Some(Prompt::Tcp {
            input,
            host,
            work,
            wanted,
        }) = self.extra.sikradio.prompt.take()
        else {
            return;
        };
        let baud = wanted.baud();
        if !ok {
            self.extra.sikradio.held = Some(Held {
                spec: wanted.clone(),
                link: Linked::Wire(Wire::new(LinkPort::closed("tcp", baud))),
            });
            self.extra.sikradio.start(work, &wanted);
            return;
        }
        input.remember(&mut self.persisted);
        let answer = input.field.value().to_owned();
        let Some(host) = host else {
            let port = self.persisted.get("TCP_port").unwrap_or("5760").to_owned();
            self.extra.sikradio.prompt = Some(Prompt::Tcp {
                input: InputBox::new("remote Port", "Enter remote port", &port),
                host: Some(answer),
                work,
                wanted,
            });
            self.extra_focus.sikradio.focus(window, cx);
            return;
        };
        self.persisted.set("TCP_port", answer.clone());
        self.persisted.set("TCP_host", host.clone());
        self.extra.sikradio.held = Some(Held {
            spec: wanted.clone(),
            link: Linked::Wire(Wire::new(LinkPort::tcp(&host, &answer, baud))),
        });
        self.extra.sikradio.start(work, &wanted);
    }

    /// A click on the page.
    pub(crate) fn sikradio_click(
        &mut self,
        click: Click,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        let page = &mut self.extra.sikradio;
        if page.busy() {
            return;
        }
        let work = match click {
            // `Progressbar_Click`.
            Click::Progressbar => {
                page.beta = !page.beta;
                let beta = if page.beta { "True" } else { "False" };
                page.show(format!("Beta set to {beta}"));
                None
            }
            Click::StatusLeds => {
                page.show(STATUS_LEDS);
                None
            }
            Click::Random => {
                page.random();
                None
            }
            Click::SyncOptions => page.sync_options(),
            Click::SaveToFile(remote) => {
                page.save_to_file(remote);
                self.extra_focus.sikradio.focus(window, cx);
                None
            }
            Click::LoadFromFile(remote) => {
                page.load_from_file(remote);
                self.extra_focus.sikradio.focus(window, cx);
                None
            }
            Click::GetCurrent => Some(Work::Load),
            Click::SaveSettings => page.save_plan().map(Work::Save),
            Click::ResetToDefault => {
                let remote = !page.text_of(Side::Remote, "RTI").is_empty();
                Some(Work::Reset(remote))
            }
            Click::PpmFailSafe(remote) => Some(Work::Ppm(remote)),
            Click::LoadCustom => Some(Work::Program),
        };
        if let Some(work) = work {
            self.sikradio_go_focused(work, window, cx);
        }
    }

    /// Once a frame: the page's tick, what it has for the status line, and work that waited -
    /// a question it asks then waiting for a click to take the focus, as the frame has no
    /// window to give it.
    pub(crate) fn sikradio_tick(&mut self, on_setup: bool) {
        self.extra.sikradio.tick(on_setup);
        if let Some(words) = self.extra.sikradio.take_status() {
            self.file_status = Some(words);
        }
        let page = &mut self.extra.sikradio;
        if !page.busy()
            && page.prompt.is_none()
            && let Some(work) = page.queued.pop_front()
        {
            self.sikradio_go(work);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// The page's size, `Sikradio.resx`'s `$this.Size`.
const PAGE: (f32, f32) = (1096.0, 608.0);

/// A tooltip over an element while it is enabled, as `toolTip1` shows none over a disabled one.
fn tip(element: gpui::Stateful<gpui::Div>, text: &str, enabled: bool) -> gpui::Stateful<gpui::Div> {
    if text.is_empty() || !enabled {
        return element;
    }
    let text = SharedString::from(text.to_owned());
    element.tooltip(move |_window, cx| -> AnyView { cx.new(|_| Tip(text.clone())).into() })
}

/// A button at its place.
fn button(ctl: &Ctl, enabled: bool, click: Click, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let id = format!("sikradio-{}", ctl.designer);
    let base = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .size_full()
        .flex()
        .items_center()
        .justify_center()
        .text_center()
        .rounded_sm()
        .border_1()
        .text_xs()
        .child(ctl.text.clone());
    let base = tip(base, &ctl.tip, enabled);
    let base = if enabled {
        base.bg(rgb(theme::ACTION))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.border_color(rgb(theme::ACCENT)))
            .on_click(cx.listener(move |this, _event, window, cx| {
                this.sikradio_click(click, window, cx);
                cx.notify();
            }))
    } else {
        base.bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::DIM))
    };
    let (x, y, w, h) = ctl.place;
    at(x, y, w, h).child(base).into_any_element()
}

/// The click a button or link is wired to.
fn click_of(designer: &str) -> Option<Click> {
    Some(match designer {
        "BUT_getcurrent" => Click::GetCurrent,
        "BUT_savesettings" => Click::SaveSettings,
        "BUT_resettodefault" => Click::ResetToDefault,
        "BUT_loadcustom" => Click::LoadCustom,
        "BUT_Syncoptions" => Click::SyncOptions,
        "btnRandom" => Click::Random,
        "BUT_SetPPMFailSafe" => Click::PpmFailSafe(false),
        "BUT_SetPPMFailSafeRemote" => Click::PpmFailSafe(true),
        "btnSaveToFile" => Click::SaveToFile(false),
        "btnRemoteSaveToFile" => Click::SaveToFile(true),
        "btnLoadFromFile" => Click::LoadFromFile(false),
        "btnRemoteLoadFromFile" => Click::LoadFromFile(true),
        "linkLabel1" => Click::StatusLeds,
        "Progressbar" => Click::Progressbar,
        _ => return None,
    })
}

/// A combo box: its text and arrow, a click dropping its list.
fn combo(
    side: Side,
    index: usize,
    ctl: &Ctl,
    enabled: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let id = format!("sikradio-{}", ctl.designer);
    let base = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .size_full()
        .flex()
        .items_center()
        .px_1()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .child(
            div()
                .flex_1()
                .overflow_hidden()
                .whitespace_nowrap()
                .child(ctl.shown()),
        )
        .child(div().text_size(px(7.0)).child("▼"));
    let base = tip(base, &ctl.tip, enabled);
    let base = if enabled {
        base.bg(rgb(theme::ACTION))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.border_color(rgb(theme::ACCENT)))
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.extra.sikradio.toggle_list(side, index);
                cx.notify();
            }))
    } else {
        base.bg(rgb(theme::PANEL)).text_color(rgb(theme::DIM))
    };
    let (x, y, w, h) = ctl.place;
    at(x, y, w, h).child(base).into_any_element()
}

/// A check box, its text drawn by its label beside it.
fn check(
    side: Side,
    index: usize,
    ctl: &Ctl,
    enabled: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let id = format!("sikradio-{}", ctl.designer);
    let colour = if enabled { theme::ACCENT } else { theme::DIM };
    let mark = ctl.checked.then(|| div().size(px(6.0)).bg(rgb(colour)));
    let base = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .size(px(13.0))
        .mt(px(3.0))
        .flex()
        .items_center()
        .justify_center()
        .border_1()
        .border_color(rgb(theme::DIM))
        .bg(rgb(if enabled { theme::BG } else { theme::PANEL }))
        .children(mark);
    let base = tip(base, &ctl.tip, enabled);
    let base = if enabled {
        base.cursor_pointer()
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.extra.sikradio.toggle(side, index);
                cx.notify();
            }))
    } else {
        base
    };
    let (x, y, w, h) = ctl.place;
    at(x, y, w, h).child(base).into_any_element()
}

/// A text box: the key boxes typed into, the rest read only.
#[allow(clippy::too_many_arguments)]
fn text_box(
    side: Side,
    index: usize,
    ctl: &Ctl,
    enabled: bool,
    page: &SikRadio,
    focus: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let id = format!("sikradio-{}", ctl.designer);
    // A dialog over the page has the focus handle the key boxes share.
    let editing =
        page.editing == Some((side, index)) && focus.is_focused(window) && page.prompt.is_none();
    let text = if editing {
        page.field.value().to_owned()
    } else {
        ctl.text.clone()
    };
    let base = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .size_full()
        .flex()
        .px_1()
        .rounded_sm()
        .border_1()
        .text_xs()
        .overflow_hidden();
    let base = if ctl.multiline {
        base.flex_col().child(div().child(text))
    } else {
        base.items_center().whitespace_nowrap().child(text)
    };
    let base = tip(base, &ctl.tip, enabled);
    let typed = !ctl.read_only && enabled;
    let base = if typed {
        let handle = focus.clone();
        let base = if editing {
            base.track_focus(focus)
                .key_context("TextField")
                .on_key_down(cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                    if this.extra.sikradio.edit_key(event) {
                        cx.notify();
                    }
                }))
                .child(div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT)))
        } else {
            base.on_click(cx.listener(move |this, _event, window, cx| {
                this.extra.sikradio.begin_edit(side, index);
                handle.focus(window, cx);
                cx.notify();
            }))
        };
        base.bg(rgb(theme::ACTION))
            .border_color(rgb(if editing {
                theme::ACCENT
            } else {
                theme::BORDER
            }))
            .text_color(rgb(theme::TEXT))
            .cursor_text()
    } else {
        base.bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
    };
    let (x, y, w, h) = ctl.place;
    at(x, y, w, h).child(base).into_any_element()
}

impl SikRadio {
    /// A key box clicked: typing into it.
    pub fn begin_edit(&mut self, side: Side, index: usize) {
        let text = self
            .at((side, index))
            .map(|c| c.text.clone())
            .unwrap_or_default();
        self.field.set(text);
        self.editing = Some((side, index));
    }

    /// A key typed into the key box: its text as it is typed, as a `TextBox`'s is.
    pub fn edit_key(&mut self, event: &KeyDownEvent) -> bool {
        let Some(found) = self.editing else {
            return false;
        };
        match self.field.key(event) {
            KeyOutcome::Changed => {
                let value = self.field.value().to_owned();
                if let Some(ctl) = self.at_mut(found) {
                    ctl.text = value;
                }
                true
            }
            KeyOutcome::Submitted | KeyOutcome::Cancelled => {
                self.editing = None;
                true
            }
            KeyOutcome::Ignored => false,
        }
    }
}

/// A label, link or the status: its text at its place.
fn label(ctl: &Ctl, enabled: bool, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let (x, y, w, h) = ctl.place;
    let colour = match (ctl.kind, enabled) {
        (_, false) => theme::DIM,
        (Kind::Link, true) => theme::ACCENT,
        _ => theme::TEXT,
    };
    let id = format!("sikradio-{}", ctl.designer);
    let mut text = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .text_xs()
        .text_color(rgb(colour))
        .flex()
        .flex_col()
        .children(
            ctl.text
                .lines()
                .map(|line| div().whitespace_nowrap().child(line.to_owned())),
        );
    if ctl.kind == Kind::Link {
        text = text.underline();
        text = tip(text, &ctl.tip, enabled);
        if let Some(click) = click_of(ctl.designer)
            && enabled
        {
            text = text
                .cursor_pointer()
                .on_click(cx.listener(move |this, _event, window, cx| {
                    this.sikradio_click(click, window, cx);
                    cx.notify();
                }));
        }
    }
    at(x, y, w, h).child(text).into_any_element()
}

/// One control, as its kind draws.
fn control(
    page: &SikRadio,
    side: Side,
    index: usize,
    focus: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let ctl = page.group(side).ctls.get(index)?;
    if !ctl.visible {
        return None;
    }
    let enabled = page.group(side).enabled && ctl.enabled && !page.busy();
    Some(match ctl.kind {
        Kind::Label | Kind::Link => label(ctl, enabled, cx),
        Kind::Button => button(ctl, enabled, click_of(ctl.designer)?, cx),
        Kind::Combo => combo(side, index, ctl, enabled, cx),
        Kind::Check => check(side, index, ctl, enabled, cx),
        Kind::Text => text_box(
            side,
            index,
            ctl,
            page.group(side).enabled && ctl.enabled,
            page,
            focus,
            window,
            cx,
        ),
        Kind::Bar => progress_bar(page, ctl, cx),
        Kind::Group => return None,
    })
}

/// `Progressbar`: its value, a click toggling `beta`.
fn progress_bar(page: &SikRadio, ctl: &Ctl, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let (x, y, w, h) = ctl.place;
    #[allow(clippy::cast_precision_loss)]
    let fill = w * (page.progress.clamp(0, 100) as f32) / 100.0;
    let id = format!("sikradio-{}", ctl.designer);
    let base = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .size_full()
        .relative()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .cursor_pointer()
        .child(
            div()
                .absolute()
                .left_0()
                .top_0()
                .h_full()
                .w(px(fill))
                .bg(rgb(theme::OK)),
        )
        .on_click(cx.listener(|this, _event, window, cx| {
            this.sikradio_click(Click::Progressbar, window, cx);
            cx.notify();
        }));
    at(x, y, w, h).child(base).into_any_element()
}

/// The page, laid out as `Sikradio.resx` lays it out.
pub fn page(
    radio: &SikRadio,
    focus: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if !radio.is_active() {
        return div().into_any_element();
    }
    let mut body = div().relative().w(px(PAGE.0)).h(px(PAGE.1)).flex_shrink_0();
    for (index, ctl) in radio.top.ctls.iter().enumerate() {
        let side = match ctl.designer {
            "groupBoxLocal" => Some(Side::Local),
            "groupBoxRemote" => Some(Side::Remote),
            _ => None,
        };
        if let Some(side) = side {
            let enabled = radio.group(side).enabled && !radio.busy();
            let caption = if side == Side::Local {
                "Local"
            } else {
                "Remote"
            };
            let mut boxed = group(ctl.place, caption, enabled);
            for inner in 0..radio.group(side).ctls.len() {
                boxed = boxed.children(control(radio, side, inner, focus, window, cx));
            }
            if let Some((open_side, open, top)) = radio.open_list
                && open_side == side
                && let Some(ctl) = radio.group(side).ctls.get(open)
            {
                let mut combo = combo_of(ctl);
                combo.top_index = top;
                let (x, y, w, h) = ctl.place;
                boxed = boxed.child(dropdown(
                    &format!("sikradio-{}", ctl.designer),
                    &combo,
                    (x, y + h, w.max(80.0)),
                    move |this, key| {
                        let item = usize::try_from(key).unwrap_or(0);
                        this.sikradio_choose_key(side, open, item);
                    },
                    |this, lines| this.extra.sikradio.scroll_list(lines),
                    cx,
                ));
            }
            body = body.child(boxed);
            continue;
        }
        body = body.children(control(radio, Side::Top, index, focus, window, cx));
    }
    div().flex().flex_col().child(body).into_any_element()
}

impl MissionPlanner {
    /// A row of a dropped-down list clicked. The drop-down's handler has no window, so an
    /// encryption level that must ask for a TCP port's host asks without the focus.
    fn sikradio_choose_key(&mut self, side: Side, index: usize, item: usize) {
        if let Some(work) = self.extra.sikradio.choose(side, index, item) {
            self.extra.sikradio.queued.push_back(work);
        }
    }
}

/// The input box for TCP's host and port.
fn input_dialog(
    input: &InputBox,
    focus: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let focused = focus.is_focused(window);
    let buttons = vec![
        action(
            "sikradio-input-ok",
            "OK",
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), window, cx| {
                this.sikradio_input_done(true, window, cx);
                cx.notify();
            }),
        ),
        action(
            "sikradio-input-cancel",
            "Cancel",
            theme::DIM,
            true,
            cx.listener(|this, _event: &(), window, cx| {
                this.sikradio_input_done(false, window, cx);
                cx.notify();
            }),
        ),
    ];
    let dialog = crate::probe::measured("sikradio-input", div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(340.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(input.title),
        )
        .child(
            div()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(input.prompt),
        )
        .child(crate::textfield::text_field(
            "sikradio-input-value",
            &input.field,
            focus,
            focused,
            px(310.0),
            cx.listener(|this, event: &KeyDownEvent, window, cx| {
                let (changed, done) = this.extra.sikradio.prompt_key(event);
                if let Some(ok) = done {
                    this.sikradio_input_done(ok, window, cx);
                }
                if changed {
                    cx.notify();
                }
            }),
        ))
        .child(div().flex().justify_end().gap_2().children(buttons));
    let size = window.viewport_size();
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .id("sikradio-input-backdrop")
                    .w(size.width)
                    .h(size.height)
                    .flex()
                    .items_center()
                    .justify_center()
                    .occlude()
                    .child(dialog),
            ),
    )
    .with_priority(2)
    .into_any_element()
}

/// The box, question or dialog the page is showing, over the whole window.
pub fn overlay(
    radio: &SikRadio,
    focus: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if !radio.is_active() {
        return None;
    }
    if let Some(message) = radio.message() {
        return Some(message_box(
            IDS.message,
            IDS.ok,
            message,
            window,
            |this| this.extra.sikradio.dismiss_message(),
            cx,
        ));
    }
    match radio.prompt.as_ref()? {
        Prompt::Tcp { input, .. } => Some(input_dialog(input, focus, window, cx)),
        Prompt::Save { path, .. } | Prompt::Open { path, .. } | Prompt::Firmware { path, .. } => {
            Some(path_box(
                IDS,
                path,
                focus,
                window,
                |this, event| {
                    let (changed, done) = this.extra.sikradio.prompt_key(event);
                    if let Some(ok) = done {
                        // Enter and Escape in the box: the dialog's OK and Cancel.
                        if let Some(work) = this.extra.sikradio.path_done(ok) {
                            this.extra.sikradio.queued.push_back(work);
                        }
                    }
                    changed
                },
                |this, ok| this.sikradio_path_done_queued(ok),
                cx,
            ))
        }
        Prompt::TryAgain { .. } => {
            let answer = |id: &'static str, text: &'static str, yes: bool| {
                action(
                    id,
                    text,
                    theme::ACCENT,
                    true,
                    cx.listener(move |this, _event: &(), _window, cx| {
                        this.extra.sikradio.answer_again(yes);
                        cx.notify();
                    }),
                )
            };
            let buttons = vec![
                answer("sikradio-again-yes", "Yes", true),
                answer("sikradio-again-no", "No", false),
                answer("sikradio-again-cancel", "Cancel", false),
            ];
            Some(modal(
                IDS.question,
                "Programming firmware failed.  Try again?",
                "Programming firmware failed.  Try again?",
                false,
                buttons,
                window,
            ))
        }
    }
}

impl MissionPlanner {
    /// A file dialog's OK or Cancel button: what follows queued for the next frame, as the
    /// dialog's handler has no window to focus.
    fn sikradio_path_done_queued(&mut self, ok: bool) {
        if let Some(work) = self.extra.sikradio.path_done(ok) {
            self.extra.sikradio.queued.push_back(work);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The tables, from `Sikradio.resx` and `Sikradio.Designer.cs`.
// ---------------------------------------------------------------------------------------------

/// A `toolTip1` text from the `.resx`.
const TIP_1: &str = "The number of output frames for SBUS or PPM missing before enters failsafe.";

/// A `toolTip1` text from the `.resx`.
const TIP_2: &str = "AES Encryption Level";

/// A `toolTip1` text from the `.resx`.
const TIP_3: &str = "Record default PPM stream for PPM output (vehicle side)";

/// A `toolTip1` text from the `.resx`.
const TIP_4: &str = "GPI1_1R/COUT Sets GPIO 1.1 as R/C(PPM) output";

/// A `toolTip1` text from the `.resx`.
const TIP_5: &str = "GPI1_1R/CIN Sets GPIO 1.1 as R/C(PPM) input";

/// A `toolTip1` text from the `.resx`.
const TIP_6: &str = "Mavlink";

/// A `toolTip1` text from the `.resx`.
const TIP_7: &str = "Serial baud rate in rounded kbps. So 57 means 57600. \n";

/// A `toolTip1` text from the `.resx`.
const TIP_8: &str = "AIR_SPEED is the inter-radio data rate in rounded kbps. So 128 means 128kbps. Max is 192, min is 2. I would not recommend values below 16 as the frequency hopping and tdm sync times get too long. ";

/// A `toolTip1` text from the `.resx`.
const TIP_9: &str = "NETID is a 16 bit 'network ID'. This is used to seed the frequency hopping sequence and to identify packets as coming from the right radio. Make sure you use a different NETID from anyone else running the same sort of radio in the area. ";

/// A `toolTip1` text from the `.resx`.
const TIP_10: &str = "TXPOWER is the transmit power in dBm. 20dBm is 100mW. It is useful to set this to lower levels for short range testing.\n";

/// A `toolTip1` text from the `.resx`.
const TIP_11: &str = "ECC is to enable/disable the golay error correcting code. It defaults to off. If you enable it then you packets take twice as many bytes to send, so you lose half your air bandwidth, but it can correct up to 3 bit errors per 12 bits of data. Use this for long range, usually in combination with a lower air data rate. The golay decode takes 20 microsecond per transmitted byte (40 microseconds per user data byte) which means you will also be a bit CPU constrained at the highest air data rates. So you usually use golay at 128kbps or less. \n";

/// A `toolTip1` text from the `.resx`.
const TIP_12: &str = "OPPRESEND enables/disables \"opportunistic resend\". When enabled the radio will send a packet twice if the serial input buffer has less than 256 bytes in it. The 2nd send is marked as a resend and discarded by the receiving radio if it got the first packet OK. This makes a big difference to the link quality, especially for uplink commands. \n";

/// A `toolTip1` text from the `.resx`.
const TIP_13: &str = "Enable or disable Hardware flow control";

/// A `toolTip1` text from the `.resx`.
const TIP_14: &str = "maximum frequency in kHz ";

/// A `toolTip1` text from the `.resx`.
const TIP_15: &str = "number of frequency hopping channels ";

/// A `toolTip1` text from the `.resx`.
const TIP_16: &str = "Listen Before Talk threshold";

/// A `toolTip1` text from the `.resx`.
const TIP_17: &str = "minimum frequency in kHz ";

/// A `toolTip1` text from the `.resx`.
const TIP_18: &str = "the percentage of time to allow transmit";

/// A `toolTip1` text from the `.resx`.
const TIP_19: &str = "see the spec for a RSSI to dBm graph. The numbers at the end are: \ntxe: number of transmit errors (eg. transmit timeouts) \nrxe: number of receive errors (crc error, framing error etc) \nstx: number of serial transmit overflows \nrrx: number of serial receive overflows \necc: number of 12 bit words successfully corrected by the golay code\nwhich result in a valid packet CRC \n";

/// A `toolTip1` text from the `.resx`.
const TIP_20: &str = "The 3DR Radios have 2 status LEDs, one red and one green.\ngreen LED blinking - searching for another radio \ngreen LED solid - link is established with another radio \nred LED flashing - transmitting data \nred LED solid - in firmware update mode";

/// A combo box's `Items` from the `.resx`.
const ITEMS_0_TO_30: &[&str] = &[
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16",
    "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30",
];

/// A combo box's `Items` from the `.resx`.
const ITEMS_SERIAL_SPEED: &[&str] = &["115", "111", "57", "38", "19", "9", "4", "2", "1"];

/// A combo box's `Items` from the `.resx`.
const ITEMS_AIR_SPEED: &[&str] = &[
    "250", "192", "128", "96", "64", "48", "32", "24", "19", "16", "8", "4", "2",
];

/// A combo box's `Items` from the `.resx`.
const ITEMS_1_TO_30: &[&str] = &[
    "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "17",
    "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30",
];

/// A combo box's `Items` from the `.resx`.
const ITEMS_TXPOWER: &[&str] = &["1", "2", "5", "8", "11", "14", "17", "20"];

/// A combo box's `Items` from the `.resx`.
const ITEMS_MAX_FREQ: &[&str] = &[
    "902000", "907500", "915000", "921000", "928000", "433050", "434040", "434790", "435000",
];

/// A combo box's `Items` from the `.resx`.
const ITEMS_NUM_CHANNELS: &[&str] = &[
    "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20",
    "30", "40", "50",
];

/// A combo box's `Items` from the `.resx`.
const ITEMS_LBT_RSSI: &[&str] = &["0", "25", "50", "100", "150", "200", "250"];

/// A combo box's `Items` from the `.resx`.
const ITEMS_MIN_FREQ: &[&str] = &[
    "902000", "907500", "915000", "921000", "928000", "433050", "434040", "434790", "435000", "",
    "", "",
];

/// A combo box's `Items` from the `.resx`.
const ITEMS_DUTY_CYCLE: &[&str] = &["10", "20", "30", "40", "50", "60", "70", "80", "90", "100"];

/// `groupBoxLocal`'s controls, in the order the Designer adds them.
const LOCAL: &[Spec] = &[
    Spec::new("btnLoadFromFile", Kind::Button, (209.0, 428.0, 102.0, 29.0))
        .text("Load from File...")
        .disabled(),
    Spec::new("btnSaveToFile", Kind::Button, (317.0, 428.0, 102.0, 29.0))
        .text("Save to File...")
        .disabled(),
    Spec::new("label54", Kind::Label, (427.0, 387.0, 100.0, 13.0)).text("Failsafe Frame Loss"),
    Spec::new("FSFRAMELOSS", Kind::Combo, (447.0, 403.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_1),
    Spec::new("GPO1_3AUXOUT", Kind::Check, (154.0, 432.0, 23.0, 20.0)).disabled(),
    Spec::new("lblGPO1_3AUXOUT", Kind::Label, (18.0, 436.0, 93.0, 13.0)).text("GPO1_3AUXOUT"),
    Spec::new("GPI1_2AUXIN", Kind::Check, (154.0, 407.0, 23.0, 20.0)).disabled(),
    Spec::new("lblGPI1_2AUXIN", Kind::Label, (18.0, 411.0, 76.0, 13.0)).text("GPI1_2AUXIN"),
    Spec::new("lblGPIO1_1FUNC", Kind::Label, (182.0, 411.0, 80.0, 13.0)).text("GPIO1_1FUNC"),
    Spec::new("GPIO1_1FUNC", Kind::Combo, (274.0, 406.0, 80.0, 21.0)).disabled(),
    Spec::new("ENCRYPTION_LEVEL", Kind::Combo, (274.0, 257.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_2)
        .items(ITEMS_0_TO_30),
    Spec::new("GPO1_0TXEN485", Kind::Check, (283.0, 382.0, 23.0, 20.0)).disabled(),
    Spec::new("lblGPO1_0TXEN485", Kind::Label, (182.0, 386.0, 95.0, 13.0)).text("GPO1_0TXEN485"),
    Spec::new("GPO1_3STATLED", Kind::Check, (154.0, 382.0, 23.0, 20.0)).disabled(),
    Spec::new("lblGPO1_3STATLED", Kind::Label, (18.0, 386.0, 97.0, 13.0)).text("GPO1_3STATLED"),
    Spec::new("GPO1_3SBUSOUT", Kind::Combo, (425.0, 332.0, 102.0, 21.0)).disabled(),
    Spec::new("label49", Kind::Label, (377.0, 18.0, 46.0, 13.0)).text("Country:"),
    Spec::new(
        "BUT_SetPPMFailSafe",
        Kind::Button,
        (425.0, 428.0, 102.0, 29.0),
    )
    .text("Set PPM Fail Safe")
    .disabled()
    .tip(TIP_3),
    Spec::new("txtCountry", Kind::Text, (380.0, 38.0, 61.0, 39.0))
        .read_only()
        .multiline(),
    Spec::new("label45", Kind::Label, (18.0, 361.0, 81.0, 13.0)).text("Rate/FreqBand"),
    Spec::new("RATE_FREQBAND", Kind::Combo, (128.0, 357.0, 399.0, 21.0)).disabled(),
    Spec::new("lblSBUSOUT", Kind::Label, (360.0, 311.0, 103.0, 13.0)).text("GPO1_3SBUSOUT:"),
    Spec::new("lblSBUSIN", Kind::Label, (18.0, 336.0, 88.0, 13.0)).text("GPO1_3SBUSIN"),
    Spec::new("GPO1_3SBUSIN", Kind::Check, (154.0, 332.0, 23.0, 20.0)).disabled(),
    Spec::new("btnRandom", Kind::Button, (242.0, 282.0, 113.0, 24.0))
        .text("Random")
        .disabled(),
    Spec::new(
        "lblRX_ENCAP_METHOD",
        Kind::Label,
        (360.0, 186.0, 58.0, 13.0),
    )
    .text("RXENCAP")
    .hidden(),
    Spec::new("RX_ENCAP_METHOD", Kind::Combo, (447.0, 182.0, 80.0, 21.0))
        .hidden()
        .disabled(),
    Spec::new(
        "lblTX_ENCAP_METHOD",
        Kind::Label,
        (360.0, 161.0, 57.0, 13.0),
    )
    .text("TXENCAP")
    .hidden(),
    Spec::new("TX_ENCAP_METHOD", Kind::Combo, (447.0, 157.0, 80.0, 21.0))
        .hidden()
        .disabled(),
    Spec::new("lblDESTID", Kind::Label, (360.0, 136.0, 43.0, 13.0))
        .text("Dest ID")
        .hidden(),
    Spec::new("lblNODEID", Kind::Label, (360.0, 111.0, 47.0, 13.0))
        .text("Node ID")
        .hidden(),
    Spec::new("DESTID", Kind::Combo, (447.0, 132.0, 80.0, 21.0))
        .hidden()
        .disabled(),
    Spec::new("NODEID", Kind::Combo, (447.0, 107.0, 80.0, 21.0))
        .hidden()
        .disabled(),
    Spec::new("lblGPO1_1R_COUT", Kind::Label, (18.0, 311.0, 91.0, 13.0)).text("GPO1_1R/COUT"),
    Spec::new("GPO1_1R_COUT", Kind::Check, (154.0, 307.0, 23.0, 20.0))
        .disabled()
        .tip(TIP_4),
    Spec::new("lblGPI1_1R_CIN", Kind::Label, (18.0, 286.0, 74.0, 13.0)).text("GPI1_1R/CIN"),
    Spec::new("GPI1_1R_CIN", Kind::Check, (154.0, 282.0, 23.0, 20.0))
        .disabled()
        .tip(TIP_5),
    Spec::new("MAVLINK", Kind::Combo, (96.0, 232.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_6),
    Spec::new("label2", Kind::Label, (18.0, 86.0, 39.0, 13.0)).text("Format"),
    Spec::new("SERIAL_SPEED", Kind::Combo, (96.0, 107.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_7)
        .items(ITEMS_SERIAL_SPEED),
    Spec::new("label1", Kind::Label, (18.0, 111.0, 32.0, 13.0)).text("Baud"),
    Spec::new("FORMAT", Kind::Text, (96.0, 82.0, 80.0, 20.0)).read_only(),
    Spec::new("AIR_SPEED", Kind::Combo, (96.0, 132.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_8)
        .items(ITEMS_AIR_SPEED),
    Spec::new("label3", Kind::Label, (18.0, 136.0, 53.0, 13.0)).text("Air Speed"),
    Spec::new("NETID", Kind::Combo, (96.0, 157.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_9)
        .items(ITEMS_1_TO_30),
    Spec::new("lblNETID", Kind::Label, (18.0, 161.0, 38.0, 13.0)).text("Net ID"),
    Spec::new("TXPOWER", Kind::Combo, (96.0, 182.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_10)
        .items(ITEMS_TXPOWER),
    Spec::new("lblTXPOWER", Kind::Label, (18.0, 186.0, 52.0, 13.0)).text("Tx Power"),
    Spec::new("ECC", Kind::Check, (96.0, 207.0, 80.0, 20.0))
        .disabled()
        .tip(TIP_11),
    Spec::new("lblECC", Kind::Label, (18.0, 211.0, 28.0, 13.0)).text("ECC"),
    Spec::new("lblOPPRESEND", Kind::Label, (18.0, 261.0, 61.0, 13.0)).text("Op Resend"),
    Spec::new("OPPRESEND", Kind::Check, (154.0, 257.0, 23.0, 20.0))
        .disabled()
        .tip(TIP_12),
    Spec::new("lblMAVLINK", Kind::Label, (18.0, 236.0, 44.0, 13.0)).text("Mavlink"),
    Spec::new("lblANT_MODE", Kind::Label, (360.0, 86.0, 77.0, 13.0)).text("Antenna Mode"),
    Spec::new("ANT_MODE", Kind::Combo, (447.0, 82.0, 80.0, 21.0)).disabled(),
    Spec::new("lblSER_BRK_DETMS", Kind::Label, (360.0, 286.0, 84.0, 13.0))
        .text("Break Detection")
        .hidden(),
    Spec::new("lblGLOBAL_RETRIES", Kind::Label, (360.0, 261.0, 73.0, 13.0))
        .text("Global Retries")
        .hidden(),
    Spec::new("lblMAX_RETRIES", Kind::Label, (360.0, 236.0, 63.0, 13.0))
        .text("Max Retries")
        .hidden(),
    Spec::new("SER_BRK_DETMS", Kind::Combo, (447.0, 282.0, 80.0, 21.0))
        .hidden()
        .disabled(),
    Spec::new("GLOBAL_RETRIES", Kind::Combo, (447.0, 257.0, 80.0, 21.0))
        .hidden()
        .disabled(),
    Spec::new("MAX_RETRIES", Kind::Combo, (447.0, 232.0, 80.0, 21.0))
        .hidden()
        .disabled(),
    Spec::new("MAX_DATA", Kind::Combo, (447.0, 207.0, 80.0, 21.0))
        .hidden()
        .disabled(),
    Spec::new("lblMAX_DATA", Kind::Label, (360.0, 211.0, 53.0, 13.0))
        .text("Max Data")
        .hidden(),
    Spec::new(
        "lblENCRYPTION_LEVEL",
        Kind::Label,
        (181.0, 261.0, 81.0, 13.0),
    )
    .text("AES Encryption"),
    Spec::new("label35", Kind::Label, (182.0, 286.0, 49.0, 13.0)).text("AES Key"),
    Spec::new("AESKEY", Kind::Text, (180.0, 307.0, 174.0, 20.0)),
    Spec::new("RTSCTS", Kind::Check, (274.0, 207.0, 80.0, 20.0))
        .disabled()
        .tip(TIP_13),
    Spec::new("lblRTSCTS", Kind::Label, (181.0, 211.0, 53.0, 13.0)).text("RTS CTS"),
    Spec::new("linkLabel_mavlink", Kind::Link, (183.0, 327.0, 146.0, 13.0))
        .text("Settings for Standard Mavlink"),
    Spec::new(
        "linkLabel_lowlatency",
        Kind::Link,
        (182.0, 340.0, 124.0, 13.0),
    )
    .text("Settings for Low Latency"),
    Spec::new("lblMAX_WINDOW", Kind::Label, (182.0, 236.0, 91.0, 13.0)).text("Max Window (ms)"),
    Spec::new("MAX_WINDOW", Kind::Combo, (274.0, 232.0, 80.0, 21.0))
        .disabled()
        .items(ITEMS_0_TO_30),
    Spec::new("lblMIN_FREQ", Kind::Label, (182.0, 86.0, 48.0, 13.0)).text("Min Freq"),
    Spec::new("MAX_FREQ", Kind::Combo, (274.0, 107.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_14)
        .items(ITEMS_MAX_FREQ),
    Spec::new("NUM_CHANNELS", Kind::Combo, (274.0, 132.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_15)
        .items(ITEMS_NUM_CHANNELS),
    Spec::new("lblLBT_RSSI", Kind::Label, (182.0, 186.0, 50.0, 13.0)).text("LBT Rssi"),
    Spec::new("LBT_RSSI", Kind::Combo, (274.0, 182.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_16)
        .items(ITEMS_LBT_RSSI),
    Spec::new("lblDUTY_CYCLE", Kind::Label, (182.0, 161.0, 58.0, 13.0)).text("Duty Cycle"),
    Spec::new("MIN_FREQ", Kind::Combo, (274.0, 82.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_17)
        .items(ITEMS_MIN_FREQ),
    Spec::new("lblNUM_CHANNELS", Kind::Label, (182.0, 136.0, 73.0, 13.0)).text("# of Channels"),
    Spec::new("lblMAX_FREQ", Kind::Label, (182.0, 111.0, 51.0, 13.0)).text("Max Freq"),
    Spec::new("DUTY_CYCLE", Kind::Combo, (274.0, 157.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_18)
        .items(ITEMS_DUTY_CYCLE),
    Spec::new("ATI2", Kind::Text, (315.0, 12.0, 61.0, 65.0))
        .read_only()
        .multiline(),
    Spec::new("ATI3", Kind::Text, (207.0, 12.0, 102.0, 20.0)).read_only(),
    Spec::new("label11", Kind::Label, (14.0, 15.0, 42.0, 13.0)).text("Version"),
    Spec::new("ATI", Kind::Text, (59.0, 12.0, 147.0, 20.0)).read_only(),
    Spec::new("RSSI", Kind::Text, (59.0, 38.0, 250.0, 39.0))
        .read_only()
        .multiline()
        .tip(TIP_19),
    Spec::new("label12", Kind::Label, (18.0, 50.0, 32.0, 13.0)).text("RSSI"),
];

/// `groupBoxRemote`'s controls, in the order the Designer adds them.
const REMOTE: &[Spec] = &[
    Spec::new(
        "btnRemoteLoadFromFile",
        Kind::Button,
        (209.0, 428.0, 102.0, 29.0),
    )
    .text("Load from File...")
    .disabled(),
    Spec::new(
        "btnRemoteSaveToFile",
        Kind::Button,
        (317.0, 428.0, 102.0, 29.0),
    )
    .text("Save to File...")
    .disabled(),
    Spec::new("lblRFSFRAMELOSS", Kind::Label, (427.0, 387.0, 100.0, 13.0))
        .text("Failsafe Frame Loss"),
    Spec::new("RFSFRAMELOSS", Kind::Combo, (447.0, 403.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_1),
    Spec::new("RGPO1_3AUXOUT", Kind::Check, (148.0, 432.0, 23.0, 20.0)).disabled(),
    Spec::new("lblRGPO1_3AUXOUT", Kind::Label, (10.0, 436.0, 93.0, 13.0)).text("GPO1_3AUXOUT"),
    Spec::new("RGPI1_2AUXIN", Kind::Check, (148.0, 407.0, 23.0, 20.0)).disabled(),
    Spec::new("lblRGPI1_2AUXIN", Kind::Label, (10.0, 411.0, 76.0, 13.0)).text("GPI1_2AUXIN"),
    Spec::new("lblRGPIO1_1FUNC", Kind::Label, (174.0, 408.0, 80.0, 13.0)).text("GPIO1_1FUNC"),
    Spec::new("RGPIO1_1FUNC", Kind::Combo, (266.0, 405.0, 80.0, 21.0)).disabled(),
    Spec::new("RENCRYPTION_LEVEL", Kind::Combo, (266.0, 257.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_2)
        .items(ITEMS_0_TO_30),
    Spec::new("RGPO1_0TXEN485", Kind::Check, (275.0, 382.0, 23.0, 20.0)).disabled(),
    Spec::new("lblRGPO1_0TXEN485", Kind::Label, (174.0, 386.0, 95.0, 13.0)).text("GPO1_0TXEN485"),
    Spec::new("RGPO1_3STATLED", Kind::Check, (148.0, 382.0, 23.0, 20.0)).disabled(),
    Spec::new("lblRGPO1_3STATLED", Kind::Label, (10.0, 386.0, 97.0, 13.0)).text("GPO1_3STATLED"),
    Spec::new("RGPO1_3SBUSOUT", Kind::Combo, (425.0, 332.0, 102.0, 21.0)).disabled(),
    Spec::new(
        "BUT_SetPPMFailSafeRemote",
        Kind::Button,
        (425.0, 428.0, 102.0, 29.0),
    )
    .text("Set PPM Fail Safe")
    .disabled()
    .tip(TIP_3),
    Spec::new("label50", Kind::Label, (364.0, 18.0, 46.0, 13.0)).text("Country:"),
    Spec::new("txtRCountry", Kind::Text, (367.0, 38.0, 61.0, 39.0))
        .read_only()
        .multiline(),
    Spec::new("lblRSBUSOUT", Kind::Label, (357.0, 311.0, 103.0, 13.0)).text("GPO1_3SBUSOUT:"),
    Spec::new("lblRSBUSIN", Kind::Label, (10.0, 336.0, 88.0, 13.0)).text("GPO1_3SBUSIN"),
    Spec::new("RGPO1_3SBUSIN", Kind::Check, (148.0, 332.0, 36.0, 20.0)).disabled(),
    Spec::new("label46", Kind::Label, (10.0, 361.0, 81.0, 13.0)).text("Rate/FreqBand"),
    Spec::new("RRATE_FREQBAND", Kind::Combo, (121.0, 357.0, 406.0, 21.0)).disabled(),
    Spec::new(
        "lblRRX_ENCAP_METHOD",
        Kind::Label,
        (357.0, 186.0, 58.0, 13.0),
    )
    .text("RXENCAP")
    .hidden(),
    Spec::new("RRX_ENCAP_METHOD", Kind::Combo, (447.0, 182.0, 80.0, 21.0)).hidden(),
    Spec::new(
        "lblRTX_ENCAP_METHOD",
        Kind::Label,
        (357.0, 161.0, 57.0, 13.0),
    )
    .text("TXENCAP")
    .hidden(),
    Spec::new("RTX_ENCAP_METHOD", Kind::Combo, (447.0, 157.0, 80.0, 21.0)).hidden(),
    Spec::new("lblRDESTID", Kind::Label, (357.0, 136.0, 43.0, 13.0))
        .text("Dest ID")
        .hidden(),
    Spec::new("lblRNODEID", Kind::Label, (357.0, 111.0, 47.0, 13.0))
        .text("Node ID")
        .hidden(),
    Spec::new("RDESTID", Kind::Combo, (447.0, 132.0, 80.0, 21.0)).hidden(),
    Spec::new("RNODEID", Kind::Combo, (447.0, 107.0, 80.0, 21.0)).hidden(),
    Spec::new("lblRGPO1_1R_COUT", Kind::Label, (10.0, 311.0, 91.0, 13.0)).text("GPO1_1R/COUT"),
    Spec::new("RMAVLINK", Kind::Combo, (88.0, 232.0, 80.0, 21.0)).disabled(),
    Spec::new("RGPO1_1R_COUT", Kind::Check, (148.0, 307.0, 20.0, 20.0))
        .disabled()
        .tip(TIP_4),
    Spec::new("RFORMAT", Kind::Text, (88.0, 82.0, 80.0, 20.0)).read_only(),
    Spec::new("lblRGPI1_1R_CIN", Kind::Label, (10.0, 286.0, 74.0, 13.0)).text("GPI1_1R/CIN"),
    Spec::new("RGPI1_1R_CIN", Kind::Check, (148.0, 282.0, 20.0, 20.0))
        .disabled()
        .tip(TIP_5),
    Spec::new("RSERIAL_SPEED", Kind::Combo, (88.0, 107.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_7)
        .items(ITEMS_SERIAL_SPEED),
    Spec::new("RAIR_SPEED", Kind::Combo, (88.0, 132.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_8)
        .items(ITEMS_AIR_SPEED),
    Spec::new("RNETID", Kind::Combo, (88.0, 157.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_9)
        .items(ITEMS_1_TO_30),
    Spec::new("lblROPPRESEND", Kind::Label, (11.0, 261.0, 61.0, 13.0)).text("Op Resend"),
    Spec::new("RTXPOWER", Kind::Combo, (88.0, 182.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_10)
        .items(ITEMS_TXPOWER),
    Spec::new("lblRMAVLINK", Kind::Label, (11.0, 236.0, 44.0, 13.0)).text("Mavlink"),
    Spec::new("RECC", Kind::Check, (88.0, 207.0, 80.0, 20.0))
        .disabled()
        .tip(TIP_11),
    Spec::new("lblRECC", Kind::Label, (11.0, 211.0, 28.0, 13.0)).text("ECC"),
    Spec::new("lblRTXPOWER", Kind::Label, (11.0, 186.0, 52.0, 13.0)).text("Tx Power"),
    Spec::new("ROPPRESEND", Kind::Check, (148.0, 257.0, 20.0, 20.0))
        .disabled()
        .tip(TIP_12),
    Spec::new("lblRNETID", Kind::Label, (11.0, 161.0, 38.0, 13.0)).text("Net ID"),
    Spec::new("label32", Kind::Label, (11.0, 111.0, 32.0, 13.0)).text("Baud"),
    Spec::new("label30", Kind::Label, (11.0, 136.0, 53.0, 13.0)).text("Air Speed"),
    Spec::new("label31", Kind::Label, (11.0, 86.0, 39.0, 13.0)).text("Format"),
    Spec::new("lblRANT_MODE", Kind::Label, (357.0, 86.0, 77.0, 13.0)).text("Antenna Mode"),
    Spec::new("RANT_MODE", Kind::Combo, (447.0, 82.0, 80.0, 21.0)).disabled(),
    Spec::new("lblRSER_BRK_DETMS", Kind::Label, (357.0, 286.0, 84.0, 13.0))
        .text("Break Detection")
        .hidden(),
    Spec::new(
        "lblRGLOBAL_RETRIES",
        Kind::Label,
        (357.0, 261.0, 73.0, 13.0),
    )
    .text("Global Retries")
    .hidden(),
    Spec::new("lblRMAX_RETRIES", Kind::Label, (357.0, 236.0, 63.0, 13.0))
        .text("Max Retries")
        .hidden(),
    Spec::new("RSER_BRK_DETMS", Kind::Combo, (447.0, 282.0, 80.0, 21.0)).hidden(),
    Spec::new("RGLOBAL_RETRIES", Kind::Combo, (447.0, 257.0, 80.0, 21.0)).hidden(),
    Spec::new("RMAX_RETRIES", Kind::Combo, (447.0, 232.0, 80.0, 21.0)).hidden(),
    Spec::new("RMAX_DATA", Kind::Combo, (447.0, 207.0, 80.0, 21.0)).hidden(),
    Spec::new("lblRMAX_DATA", Kind::Label, (357.0, 211.0, 53.0, 13.0))
        .text("Max Data")
        .hidden(),
    Spec::new("label38", Kind::Label, (174.0, 286.0, 49.0, 13.0)).text("AES Key"),
    Spec::new("RAESKEY", Kind::Text, (168.0, 307.0, 180.0, 20.0)),
    Spec::new(
        "lblRENCRYPTION_LEVEL",
        Kind::Label,
        (173.0, 261.0, 81.0, 13.0),
    )
    .text("AES Encryption"),
    Spec::new("RRTSCTS", Kind::Check, (266.0, 207.0, 80.0, 20.0))
        .disabled()
        .tip(TIP_13),
    Spec::new("lblRRTSCTS", Kind::Label, (173.0, 211.0, 53.0, 13.0)).text("RTS CTS"),
    Spec::new("lblRMAX_WINDOW", Kind::Label, (174.0, 236.0, 91.0, 13.0)).text("Max Window (ms)"),
    Spec::new("RMAX_WINDOW", Kind::Combo, (266.0, 232.0, 80.0, 21.0))
        .disabled()
        .items(ITEMS_0_TO_30),
    Spec::new("lblRMIN_FREQ", Kind::Label, (174.0, 86.0, 48.0, 13.0)).text("Min Freq"),
    Spec::new("RMAX_FREQ", Kind::Combo, (266.0, 107.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_14)
        .items(ITEMS_MAX_FREQ),
    Spec::new("RNUM_CHANNELS", Kind::Combo, (266.0, 132.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_15)
        .items(ITEMS_NUM_CHANNELS),
    Spec::new("RDUTY_CYCLE", Kind::Combo, (266.0, 157.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_18)
        .items(ITEMS_DUTY_CYCLE),
    Spec::new("RLBT_RSSI", Kind::Combo, (266.0, 182.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_16)
        .items(ITEMS_LBT_RSSI),
    Spec::new("RMIN_FREQ", Kind::Combo, (266.0, 82.0, 80.0, 21.0))
        .disabled()
        .tip(TIP_17)
        .items(ITEMS_MAX_FREQ),
    Spec::new("lblRMAX_FREQ", Kind::Label, (174.0, 111.0, 51.0, 13.0)).text("Max Freq"),
    Spec::new("lblRNUM_CHANNELS", Kind::Label, (174.0, 136.0, 73.0, 13.0)).text("# of Channels"),
    Spec::new("lblRDUTY_CYCLE", Kind::Label, (174.0, 161.0, 58.0, 13.0)).text("Duty Cycle"),
    Spec::new("lblRLBT_RSSI", Kind::Label, (174.0, 186.0, 50.0, 13.0)).text("LBT Rssi"),
    Spec::new("RTI2", Kind::Text, (300.0, 12.0, 61.0, 65.0))
        .read_only()
        .multiline(),
    Spec::new("label9", Kind::Label, (12.0, 15.0, 42.0, 13.0)).text("Version"),
    Spec::new("RTI", Kind::Text, (60.0, 12.0, 147.0, 20.0)).read_only(),
];

/// `this`'s controls, in the order the Designer adds them.
const TOP: &[Spec] = &[
    Spec::new("Progressbar", Kind::Bar, (12.0, 561.0, 1072.0, 36.0)),
    Spec::new("BUT_loadcustom", Kind::Button, (674.0, 3.0, 96.0, 39.0)).text("Upload Firmware (custom)"),
    Spec::new("BUT_resettodefault", Kind::Button, (586.0, 3.0, 82.0, 39.0)).text("Reset to Defaults"),
    Spec::new("linkLabel1", Kind::Link, (836.0, 11.0, 63.0, 13.0)).text("Status Leds").tip(TIP_20),
    Spec::new("label10", Kind::Label, (9.0, 11.0, 0.0, 13.0)),
    Spec::new("groupBoxRemote", Kind::Group, (551.0, 48.0, 533.0, 463.0)).text("Remote"),
    Spec::new("groupBoxLocal", Kind::Group, (12.0, 48.0, 533.0, 463.0)).text("Local"),
    Spec::new("BUT_Syncoptions", Kind::Button, (457.0, 518.0, 102.0, 29.0)).text("Copy required to remote").disabled(),
    Spec::new("BUT_savesettings", Kind::Button, (384.0, 3.0, 69.0, 39.0)).text("Save Settings").disabled(),
    Spec::new("BUT_getcurrent", Kind::Button, (309.0, 3.0, 69.0, 39.0)).text("Load Settings"),
    Spec::new("lbl_status", Kind::Label, (9.0, 518.0, 442.0, 28.0)).text("NOTE: Always click \"Copy required to remote\" when modifying\nthis ensures you wont lose radio link"),
    Spec::new("BUT_upload", Kind::Button, (459.0, 3.0, 121.0, 39.0)).text("Upload Firmware (standard)").hidden(),
];

#[cfg(test)]
mod tests {
    use super::*;
    use mp_sikradio::page::RemoteLoaded;
    use mp_sikradio::settings::get_settings;

    /// An RFD900+'s `ATI5?`, as `tests/gui/sik-radio.py` gives it.
    const QUERY: &str = "S0:FORMAT(R)[0..255]=25\r\nS1:SERIAL_SPEED(N)[1..115]=57{1200,2400,4800,9600,19200,38400,57600,115200,}\r\nS2:AIR_SPEED(N)[2..250]=64{2,4,8,16,19,24,32,48,64,96,128,192,250,}\r\nS3:NETID(N)[0..499]=25\r\nS4:TXPOWER(N)[0..30]=30\r\nS5:ECC(N)[0..1]=0\r\nS6:MAVLINK(N)[0..2]=1{RawData,Mavlink,LowLatency,}\r\nS7:OPPRESEND(N)[0..1]=1\r\nS8:MIN_FREQ(N)[902000..927000]=915000\r\nS9:MAX_FREQ(N)[903000..928000]=928000\r\nS10:NUM_CHANNELS(N)[1..50]=20\r\nS11:DUTY_CYCLE(N)[10..100]=100\r\nS12:LBT_RSSI(N)[0..220]=0\r\nS13:MANCHESTER(N)[0..1]=0\r\nS14:RTSCTS(N)[0..1]=0\r\nS15:MAX_WINDOW(N)[20..400]=131\r\nS16:ENCRYPTION_LEVEL(N)[0..1]=0{Off,128b,}\r\n";

    /// Its `ATI5`.
    const PLAIN: &str = "S0:FORMAT=25\r\nS1:SERIAL_SPEED=57\r\nS2:AIR_SPEED=64\r\nS3:NETID=25\r\nS4:TXPOWER=30\r\nS5:ECC=0\r\nS6:MAVLINK=1\r\nS7:OPPRESEND=1\r\nS8:MIN_FREQ=915000\r\nS9:MAX_FREQ=928000\r\nS10:NUM_CHANNELS=20\r\nS11:DUTY_CYCLE=100\r\nS12:LBT_RSSI=0\r\nS13:MANCHESTER=0\r\nS14:RTSCTS=0\r\nS15:MAX_WINDOW=131\r\nS16:ENCRYPTION_LEVEL=0\r\n";

    const KEY: &str = "00000000000000000000000000000000";

    fn settings(query: &str, plain: &str) -> Settings {
        let (mut settings, _) = get_settings(query, false, plain, None);
        settings.insert("AESKEY", Setting::text("&E", "AESKEY", KEY));
        settings
    }

    /// What Load Settings reads of an RFD900+, with a remote one or without.
    fn loaded(query: &str, plain: &str, remote: bool) -> Loaded {
        let settings = settings(query, plain);
        Loaded {
            entered: true,
            ati: Some("RFD SiK 2.65 on RFD900P".to_owned()),
            multipoint_fix: -1,
            freq: Some(Frequency::F915),
            board: Some(Board::RFD900P),
            country: Some("--".to_owned()),
            key: Some(KeyBox::Shown(KEY.to_owned())),
            simple_mavlink: false,
            rssi: Some("L/R RSSI: 210/198".to_owned()),
            ati5: Some(plain.to_owned()),
            settings: Some(settings.clone()),
            rti: Some(if remote {
                "RFD SiK 2.65 on RFD900P\r\n".to_owned()
            } else {
                String::new()
            }),
            remote_country: Some("--".to_owned()),
            remote: remote.then(|| RemoteLoaded {
                rti2: Some("DEVICE_ID_RFD900P".to_owned()),
                key: KeyBox::Shown(KEY.to_owned()),
                simple_mavlink: false,
                rti5: plain.to_owned(),
                settings,
            }),
            error: None,
        }
    }

    /// The page after Load Settings read `loaded`.
    fn page_with(loaded: &Loaded) -> SikRadio {
        let mut page = SikRadio::new();
        page.activate();
        page.before_load();
        page.apply_loaded(loaded);
        page
    }

    fn page() -> SikRadio {
        page_with(&loaded(QUERY, PLAIN, true))
    }

    fn text(page: &SikRadio, side: Side, name: &str) -> String {
        page.text_of(side, name)
    }

    fn index(page: &SikRadio, side: Side, name: &str) -> usize {
        page.group(side).index(name).unwrap()
    }

    /// The constructor's lists: `NETID`'s 500, `MAVLINK`'s three, `MAX_WINDOW`'s 33 to 131.
    #[test]
    fn the_constructor_fills_its_lists() {
        let page = SikRadio::new();
        let netid = page.ctl(Side::Local, "NETID").unwrap();
        assert_eq!(netid.items.len(), 500);
        assert_eq!(netid.shown(), "0");
        let mavlink = page.ctl(Side::Remote, "RMAVLINK").unwrap();
        let names: Vec<&str> = mavlink.items.iter().map(|i| i.text.as_str()).collect();
        assert_eq!(names, ["RawData", "Mavlink", "LowLatency"]);
        let window = page.ctl(Side::Local, "MAX_WINDOW").unwrap();
        assert_eq!(window.items.first().map(|i| i.text.as_str()), Some("33"));
        assert_eq!(window.items.len(), 99);
        let serial = page.ctl(Side::Local, "SERIAL_SPEED").unwrap();
        assert_eq!(
            serial.selected, None,
            "Items, not a DataSource: nothing selected"
        );
        assert_eq!(page.default_lists.len(), 10);
        assert!(!page.ctl(Side::Top, "BUT_upload").unwrap().visible);
        assert!(!page.ctl(Side::Top, "BUT_savesettings").unwrap().enabled);
    }

    /// Load Settings fills both groups from the radios' settings, in their options and ranges.
    #[test]
    fn load_fills_the_controls() {
        let page = page();
        assert_eq!(page.status(), "Done");
        assert_eq!(text(&page, Side::Local, "ATI"), "RFD SiK 2.65 on RFD900P");
        assert_eq!(text(&page, Side::Local, "ATI2"), "DEVICE_ID_RFD900P");
        assert_eq!(text(&page, Side::Local, "ATI3"), "FREQ_915");
        assert_eq!(text(&page, Side::Local, "FORMAT"), "25");
        assert_eq!(text(&page, Side::Local, "SERIAL_SPEED"), "57600");
        assert_eq!(text(&page, Side::Local, "AIR_SPEED"), "64");
        assert_eq!(text(&page, Side::Local, "NETID"), "25");
        assert_eq!(text(&page, Side::Local, "TXPOWER"), "30");
        assert_eq!(text(&page, Side::Local, "MAVLINK"), "Mavlink");
        assert_eq!(text(&page, Side::Local, "MIN_FREQ"), "915000");
        assert_eq!(text(&page, Side::Local, "MAX_WINDOW"), "131");
        assert_eq!(text(&page, Side::Local, "ENCRYPTION_LEVEL"), "Off");
        assert_eq!(text(&page, Side::Local, "AESKEY"), KEY);
        assert!(page.ctl(Side::Local, "OPPRESEND").unwrap().checked);
        assert!(!page.ctl(Side::Local, "ECC").unwrap().checked);
        assert_eq!(text(&page, Side::Remote, "RNETID"), "25");
        assert_eq!(text(&page, Side::Remote, "RTI2"), "DEVICE_ID_RFD900P");
        assert!(page.local.enabled && page.remote.enabled);
        assert!(page.enabled(Side::Top, "BUT_savesettings"));
        assert!(page.enabled(Side::Top, "BUT_loadcustom"));
        assert!(page.enabled(Side::Local, "btnSaveToFile"));
        assert!(
            !page.enabled(Side::Local, "BUT_SetPPMFailSafe"),
            "no R/C output"
        );
        assert!(!page.in_encryption);
        // MANCHESTER has no control: it takes the first spare check box, GPI1_1R/CIN's.
        let spare = page.ctl(Side::Local, "GPI1_1R_CIN").unwrap();
        assert_eq!(spare.name, "MANCHESTER");
        assert_eq!(spare.tip, "MANCHESTER");
        assert_eq!(text(&page, Side::Local, "lblGPI1_1R_CIN"), "MANCHESTER");
        // The point-to-point model hides the multipoint pairs.
        assert!(!page.ctl(Side::Local, "NODEID").unwrap().visible);
    }

    /// A value its control cannot show makes the status say so.
    #[test]
    fn a_value_the_box_cannot_show_is_invalid() {
        let query = QUERY.replace("S3:NETID(N)[0..499]=25", "S3:NETID=600");
        let plain = PLAIN.replace("S3:NETID=25", "S3:NETID=600");
        let page = page_with(&loaded(&query, &plain, false));
        assert_eq!(page.status(), "Done.  Some settings in modem were invalid.");
        assert_eq!(text(&page, Side::Local, "NETID"), "0", "left as it was");
    }

    /// A MAVLINK value that is no `mavlink_option` is the C#'s exception: "Error".
    #[test]
    fn a_bad_mavlink_value_is_an_error() {
        let query = QUERY.replace("=1{RawData,Mavlink,LowLatency,}", "=1");
        let plain = PLAIN.replace("S6:MAVLINK=1", "S6:MAVLINK=x");
        let mut page = page_with(&loaded(&query, &plain, false));
        assert_eq!(page.status(), "Error");
        assert_eq!(
            page.take_status().as_deref(),
            Some("Error during read Requested value 'x' was not found.")
        );
        assert!(
            !page.enabled(Side::Top, "BUT_loadcustom"),
            "never enabled again"
        );
        assert_eq!(mavlink_name("2").unwrap(), "LowLatency");
        assert_eq!(mavlink_name("7").unwrap(), "7");
    }

    /// Without a remote radio its group is emptied and its key box keeps nothing.
    #[test]
    fn load_without_a_remote_radio() {
        let page = page_with(&loaded(QUERY, PLAIN, false));
        assert_eq!(text(&page, Side::Remote, "RTI"), "");
        assert!(page.remote_settings.is_none());
        assert!(!page.enabled(Side::Remote, "btnRemoteSaveToFile"));
    }

    /// Save Settings writes what changed; a minimum frequency above the maximum is refused with
    /// the C#'s box - the remote radio's saying only "Remote".
    #[test]
    fn save_plans_the_changes_and_checks_them() {
        let mut page = page();
        let netid = index(&page, Side::Local, "NETID");
        assert_eq!(page.choose(Side::Local, netid, 28), None);
        let plan = page.save_plan().unwrap();
        assert_eq!(
            plan.local,
            [Change {
                name: "NETID".to_owned(),
                designator: "S3".to_owned(),
                value: "28".to_owned(),
                is_one: false,
            }]
        );
        assert_eq!(plan.remote, Some(Vec::new()));
        assert_eq!(plan.local_key, None, "Off");

        let min = index(&page, Side::Local, "MIN_FREQ");
        let last = page.ctl(Side::Local, "MIN_FREQ").unwrap().items.len() - 1;
        page.choose(Side::Local, min, last);
        let max = index(&page, Side::Local, "MAX_FREQ");
        page.choose(Side::Local, max, 0);
        assert_eq!(page.save_plan(), None);
        assert_eq!(
            page.message().unwrap().text,
            "Local settings invalid, operation aborted:\n\tMIN_FREQ can't be more than MAX_FREQ"
        );
        page.dismiss_message();
        let mut page = self::page();
        let rmin = index(&page, Side::Remote, "RMIN_FREQ");
        let last = page.ctl(Side::Remote, "RMIN_FREQ").unwrap().items.len() - 1;
        page.choose(Side::Remote, rmin, last);
        let rmax = index(&page, Side::Remote, "RMAX_FREQ");
        page.choose(Side::Remote, rmax, 0);
        assert_eq!(page.save_plan(), None);
        assert_eq!(
            page.message().unwrap().text,
            "Remote\n\tMIN_FREQ can't be more than MAX_FREQ"
        );
    }

    /// An encryption level chosen is written at once, and the key planned at its length; Copy
    /// required to remote copies the level - written too - and then the key.
    #[test]
    fn the_encryption_level_and_the_copy_to_remote() {
        let mut page = page();
        let level = index(&page, Side::Local, "ENCRYPTION_LEVEL");
        let work = page.choose(Side::Local, level, 1);
        assert_eq!(
            work,
            Some(Work::Encryption {
                remote: false,
                level: 1,
                then_key: None
            })
        );
        assert!(page.enabled(Side::Local, "btnRandom"));
        page.with(Side::Local, "AESKEY", |c| c.text = " 0123abcd ".to_owned());
        assert_eq!(
            page.key_plan(Side::Local),
            Some(Ok("0123abcd000000000000000000000000".to_owned()))
        );
        assert_eq!(text(&page, Side::Local, "AESKEY"), "0123abcd", "trimmed");
        page.with(Side::Local, "AESKEY", |c| c.text = "xyz".to_owned());
        assert_eq!(page.key_plan(Side::Local), Some(Err(32)));
        page.with(Side::Local, "AESKEY", |c| c.text = "0123".to_owned());

        let netid = index(&page, Side::Local, "NETID");
        page.choose(Side::Local, netid, 28);
        let work = page.sync_options();
        assert_eq!(
            work,
            Some(Work::Encryption {
                remote: true,
                level: 1,
                then_key: Some("0123".to_owned())
            })
        );
        assert_eq!(text(&page, Side::Remote, "RNETID"), "28");
        assert_eq!(text(&page, Side::Remote, "RENCRYPTION_LEVEL"), "128b");
        // The level written, the key read back - then the copy's key, as the C#'s order has it.
        page.done(
            Held {
                spec: PortSpec::Tcp { baud: 57_600 },
                link: Linked::Wire(Wire::new(LinkPort::closed("test", 57_600))),
            },
            Outcome::Encryption {
                remote: true,
                then_key: Some("0123".to_owned()),
                result: Ok(Some("FFFF".to_owned())),
            },
        );
        assert_eq!(text(&page, Side::Remote, "RAESKEY"), "0123");
        // Unchanged, nothing is written; and a load is not interrupted.
        assert_eq!(page.sync_options(), None);
        page.in_encryption = true;
        assert_eq!(page.choose(Side::Local, level, 0), None);
    }

    /// Random: a key of the level's length in both key boxes.
    #[test]
    fn random_fills_both_key_boxes() {
        let mut page = page();
        let level = index(&page, Side::Local, "ENCRYPTION_LEVEL");
        page.choose(Side::Local, level, 1);
        page.random();
        let key = text(&page, Side::Local, "AESKEY");
        assert_eq!(key.len(), 32);
        assert!(
            key.chars()
                .all(|c| c.is_ascii_hexdigit() && !c.is_ascii_lowercase())
        );
        assert_eq!(text(&page, Side::Remote, "RAESKEY"), key);
        assert_eq!(random_key(0), "");
    }

    /// Save to File writes the settings with the controls' changes; Load from File reads a
    /// file's into the controls and says what it loaded.
    #[test]
    fn the_settings_file_round_trip() {
        let mut page = page();
        let netid = index(&page, Side::Local, "NETID");
        page.choose(Side::Local, netid, 28);
        page.save_to_file(false);
        let Some(Prompt::Save { path, text: saved }) = &page.prompt else {
            panic!("the Save As dialog");
        };
        assert_eq!(path.caption, "Save As");
        assert_eq!(path.filter, INI_FILTER);
        assert!(saved.contains("NETID = 28"), "{saved}");
        assert!(saved.contains(&format!("AESKEY = {KEY}")));
        let dir = std::env::temp_dir().join(format!("mp-gui-sikradio-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("netid.ini");
        std::fs::write(&file, "NETID = 33\n; comment\nNOT_THERE = 1\n").unwrap();
        let file = file.display().to_string();
        page.prompt = None;
        assert_eq!(page.load_file(false, &file), None);
        assert_eq!(text(&page, Side::Local, "NETID"), "33");
        assert_eq!(
            page.message().unwrap().text,
            format!("Loaded\nNETID = 33\nfrom {file} OK")
        );
        page.dismiss_message();
        assert_eq!(page.load_file(false, "/no/such/file.ini"), None);
        assert_eq!(
            page.message().unwrap().text,
            "Failed to load settings from /no/such/file.ini"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    /// The remote radio's file loads only its check boxes and text boxes: `UpdateControlsWithValues`
    /// asks for the setting by the editor's name, `R` and all.
    #[test]
    fn a_remote_file_reaches_only_its_check_and_text_boxes() {
        let mut page = page();
        let dir = std::env::temp_dir().join(format!("mp-gui-sikradio-r-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("remote.ini");
        std::fs::write(&file, "NETID = 33\nECC = 1\nAESKEY = ABCD\n").unwrap();
        page.load_file(true, &file.display().to_string());
        assert_eq!(
            text(&page, Side::Remote, "RNETID"),
            "25",
            "the C# finds no RNETID setting"
        );
        assert!(page.ctl(Side::Remote, "RECC").unwrap().checked);
        assert_eq!(text(&page, Side::Remote, "RAESKEY"), "ABCD");
        std::fs::remove_dir_all(&dir).ok();
    }

    /// The firmware models: multipoint renames and relists the extra pairs; point to point puts
    /// them back and hides them.
    #[test]
    fn the_model_renames_the_extra_pairs() {
        let mut page = SikRadio::new();
        page.set_model(Side::Remote, Model::Multipoint, &Settings::new());
        assert_eq!(
            page.ctl(Side::Remote, "RDESTID").unwrap().name,
            "RNODEDESTINATION"
        );
        assert_eq!(text(&page, Side::Remote, "lblRTX_ENCAP_METHOD"), "Sync Any");
        let dest = page.ctl(Side::Remote, "RDESTID").unwrap();
        assert_eq!(dest.items.len(), 31);
        assert_eq!(dest.items.last().map(|i| i.text.as_str()), Some("65535"));
        assert!(page.ctl(Side::Remote, "RNODEID").unwrap().visible);
        assert!(!page.ctl(Side::Remote, "RMAX_DATA").unwrap().visible);
        page.set_model(Side::Remote, Model::P2p, &Settings::new());
        assert_eq!(page.ctl(Side::Remote, "RDESTID").unwrap().name, "RDESTID");
        assert_eq!(text(&page, Side::Remote, "lblRTX_ENCAP_METHOD"), "TXENCAP");
        assert!(!page.ctl(Side::Remote, "RNODEID").unwrap().visible);
        page.set_model(Side::Local, Model::Async, &Settings::new());
        assert!(page.ctl(Side::Local, "GLOBAL_RETRIES").unwrap().visible);
        assert_eq!(text(&page, Side::Local, "lblMAX_DATA"), "Max Data");
        assert_eq!(
            SikRadio::model_of("RFD SiK 3.07 MP on RFD900X", Board::RFD900X, &[]),
            Model::MultipointX
        );
        assert_eq!(
            SikRadio::model_of("SiK", Board::RFD900P, &["[1] S0:FORMAT=25".to_owned()]),
            Model::Multipoint
        );
    }

    /// An SBUS editor answers to either of its settings, its label then naming the one asked.
    #[test]
    fn the_sbus_editors_take_either_setting() {
        let mut page = SikRadio::new();
        let found = page.find_control(Side::Local, "GPO1_1SBUSIN").unwrap();
        assert_eq!(page.at(found).unwrap().designer, "GPO1_3SBUSIN");
        assert_eq!(text(&page, Side::Local, "lblSBUSIN"), "GPO1_1SBUSIN");
        let found = page.find_control(Side::Local, "RGPO1_3SBUSOUT").unwrap();
        assert_eq!(found.0, Side::Remote);
        assert_eq!(text(&page, Side::Remote, "lblRSBUSOUT"), "GPO1_3SBUSOUT");
        assert_eq!(
            page.find_control(Side::Local, "netid").map(|f| f.0),
            Some(Side::Local)
        );
    }

    /// The multipoint `[n]` is taken off each line past it.
    #[test]
    fn multipoint_lines_lose_their_node() {
        let items = ["[1] S0:FORMAT=25".to_owned(), "S".to_owned()];
        assert_eq!(modify_for_multipoint(&items, 3), ["S0:FORMAT=25", "S"]);
        assert_eq!(modify_for_multipoint(&items, -1), items);
    }

    /// `GetSession`: a new session on the same port when the baud rate is not the link's.
    #[test]
    fn a_new_baud_rate_is_a_new_session() {
        let wire = Wire::new(LinkPort::closed("serial:/dev/null:57600", 57_600));
        let mut session = Session::new(wire, 57_600);
        session.board = Board::RFD900P;
        let spec = PortSpec::Serial {
            path: "/dev/null".to_owned(),
            baud: 57_600,
        };
        let held = Held {
            spec: spec.clone(),
            link: Linked::Session(session),
        };
        let (_, same) = held.session(&spec);
        assert_eq!(same.board, Board::RFD900P, "the same session");
        let held = Held {
            spec,
            link: Linked::Session(same),
        };
        let faster = PortSpec::Serial {
            path: "/dev/null".to_owned(),
            baud: 115_200,
        };
        let (_, new) = held.session(&faster);
        assert_eq!(new.board, Board::FAILED, "a new one");
        assert_eq!(new.main_firmware_baud(), 115_200);
    }

    /// The controls, places, texts, flags, tooltips and items are the `.resx`'s, in the
    /// Designer's `Controls.Add` order.
    #[test]
    fn the_controls_are_the_resx() {
        let (Some(resx), Some(designer)) = (
            crate::config_coverage::source::csharp("Radio/Sikradio.resx"),
            crate::config_coverage::source::csharp("Radio/Sikradio.Designer.cs"),
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let key = |name: &str| match name {
            "AESKEY" => "txt_aeskey".to_owned(),
            "RAESKEY" => "txt_Raeskey".to_owned(),
            other => other.to_owned(),
        };
        let get = |name: &str, property: &str| {
            values
                .get(&format!("{}.{property}", key(name)))
                .map(|v| v.replace("\r\n", "\n"))
        };
        for (parent, specs) in [("groupBoxLocal", LOCAL), ("groupBoxRemote", REMOTE)] {
            let added: Vec<&str> = designer
                .split(&format!("this.{parent}.Controls.Add(this."))
                .skip(1)
                .filter_map(|rest| rest.split(')').next())
                .collect();
            let ours: Vec<&str> = specs.iter().map(|s| s.name).collect();
            assert_eq!(ours, added, "{parent}'s order");
        }
        for spec in LOCAL.iter().chain(REMOTE).chain(TOP) {
            let (x, y, w, h) = spec.place;
            assert_eq!(
                get(spec.name, "Location").as_deref(),
                Some(format!("{x}, {y}").as_str()),
                "{}",
                spec.name
            );
            assert_eq!(
                get(spec.name, "Size").as_deref(),
                Some(format!("{w}, {h}").as_str()),
                "{}",
                spec.name
            );
            assert_eq!(
                get(spec.name, "Text").unwrap_or_default(),
                spec.text,
                "{}",
                spec.name
            );
            assert_eq!(
                get(spec.name, "Visible").as_deref() == Some("False"),
                !spec.visible,
                "{}",
                spec.name
            );
            assert_eq!(
                get(spec.name, "Enabled").as_deref() == Some("False"),
                !spec.enabled,
                "{}",
                spec.name
            );
            assert_eq!(
                get(spec.name, "ToolTip").unwrap_or_default(),
                spec.tip,
                "{}",
                spec.name
            );
            for (i, item) in spec.items.iter().enumerate() {
                let property = if i == 0 {
                    "Items".to_owned()
                } else {
                    format!("Items{i}")
                };
                assert_eq!(
                    get(spec.name, &property).unwrap_or_default(),
                    *item,
                    "{}",
                    spec.name
                );
            }
            if spec.read_only {
                assert!(
                    designer.contains(&format!("this.{}.ReadOnly = true;", spec.name)),
                    "{}",
                    spec.name
                );
            }
        }
    }

    /// The Designer wires seventeen events, in [`WIRINGS`]' order.
    #[test]
    fn the_designer_wires_seventeen_events() {
        let Some(designer) = crate::config_coverage::source::csharp("Radio/Sikradio.Designer.cs")
        else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let wired: Vec<String> = designer
            .lines()
            .filter(|line| line.contains(" += new "))
            .filter_map(|line| {
                let event = line.trim().strip_prefix("this.")?.split(" += ").next()?;
                Some(event.to_owned())
            })
            .collect();
        let ours: Vec<&str> = WIRINGS.iter().map(|(event, _)| *event).collect();
        assert_eq!(wired, ours);
        assert_eq!(
            crate::config_coverage::OTHER_PAGES
                .iter()
                .find(|p| p.class == "Sikradio")
                .and_then(|p| p.wirings),
            Some(17)
        );
    }

    /// The GUI script asks for facts this page records and clicks controls it draws.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-sikradio.gui");
        let source = include_str!("sikradio.rs");
        let page = SikRadio::new();
        let facts: Vec<String> = page.facts().into_iter().map(|(key, _)| key).collect();
        let designers: Vec<&str> = LOCAL
            .iter()
            .chain(REMOTE)
            .chain(TOP)
            .map(|s| s.name)
            .collect();
        let mut expects = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.sikradio.") => {
                    let what = key.trim_start_matches("config.sikradio.");
                    assert!(facts.iter().any(|f| f == what), "{key} is not recorded");
                    expects += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("sikradio-") => {
                    let rest = id.trim_start_matches("sikradio-");
                    let drawn = source.contains(&format!("\"{id}\""))
                        || designers.contains(&rest)
                        || designers.iter().any(|d| {
                            rest.strip_prefix(*d)
                                .is_some_and(|row| row.starts_with('-'))
                        });
                    assert!(drawn, "{id} is not drawn");
                }
                _ => {}
            }
        }
        assert!(expects > 40, "{expects} expectations");
    }
}

#[cfg(test)]
mod thread_tests {
    use super::*;

    /// A page with a port that did not open, as `Connect` leaves one.
    fn page_on_a_closed_port() -> (SikRadio, PortSpec) {
        let mut page = SikRadio::new();
        page.activate();
        let spec = PortSpec::Serial {
            path: "/no/such/port".to_owned(),
            baud: 57_600,
        };
        page.held = Some(Held {
            spec: spec.clone(),
            link: Linked::Wire(Wire::new(LinkPort::serial("/no/such/port", 57_600))),
        });
        (page, spec)
    }

    fn settle(page: &mut SikRadio, on_setup: bool) {
        let deadline = Instant::now() + Duration::from_secs(20);
        while page.busy() && Instant::now() < deadline {
            page.tick(on_setup);
            std::thread::sleep(Duration::from_millis(5));
        }
        assert!(!page.busy(), "the thread did not finish");
    }

    /// Load Settings runs on its thread; what it says reaches the page, and its end puts the port
    /// back. A port that is closed fails it as the C#'s does: "Error", and "Error during read" on
    /// the status line, the controls left disabled.
    #[test]
    fn a_handler_runs_on_its_thread_and_reports_back() {
        let (mut page, spec) = page_on_a_closed_port();
        page.start(Work::Load, &spec);
        assert!(page.busy());
        assert_eq!(page.status(), "Connecting");
        assert!(!page.local.enabled, "disabled while it runs");
        settle(&mut page, true);
        assert_eq!(page.status(), "Error");
        assert_eq!(
            page.take_status().as_deref(),
            Some("Error during read The port is closed.")
        );
        assert!(page.held.is_some(), "the port kept");
        assert!(
            !page.enabled(Side::Top, "BUT_getcurrent"),
            "as the C# leaves it"
        );
    }

    /// Upload Firmware on a port that does not answer: "Programming failed.  (Try again?)", the
    /// session ended and the port kept; the page's controls back.
    #[test]
    fn a_failed_upload_ends_the_session() {
        let (mut page, spec) = page_on_a_closed_port();
        page.start(Work::Program, &spec);
        settle(&mut page, true);
        assert_eq!(page.status(), "Programming failed.  (Try again?)");
        assert!(matches!(
            page.held.as_ref().map(|h| &h.link),
            Some(Linked::Wire(_))
        ));
        assert!(page.enabled(Side::Top, "BUT_loadcustom"));
        assert!(page.enabled(Side::Top, "BUT_getcurrent"));
        assert!(!page.enabled(Side::Top, "BUT_savesettings"));
    }

    /// The SETUP screen gone: the page made anew, and the port put back in transparent mode and
    /// closed on the way.
    #[test]
    fn the_page_goes_with_the_screen() {
        let (mut page, _) = page_on_a_closed_port();
        page.show("a box");
        page.tick(false);
        assert!(!page.is_active());
        assert!(page.message().is_none(), "a new page");
        settle(&mut page, false);
        assert!(page.held.is_none(), "the port closed");
        page.tick(false);
        assert!(!page.busy(), "made once");
    }
}
