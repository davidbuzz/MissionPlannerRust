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

//! DroneCAN/UAVCAN: `GCSViews/ConfigurationView/ConfigDroneCAN.cs`, an Optional Hardware page of
//! Initial Setup (`GCSViews/InitialSetup.cs:273-277`), listed whether or not a vehicle is
//! connected. The protocol is `crates/mp-dronecan`, the port of `ExtLibs/DroneCAN`.
//!
//! What it shows, at the Designer's places in a 798 x 612 page: the interface list (SLCAN,
//! MAVLinkCAN1, MAVLinkCAN2, MCastCan1, MCastCan2), under it the network interface list the two
//! multicast kinds show, Connect, Filter, Stats, Inspector, "Check for Updates", the warning about
//! SLCAN, "Exit SLCAN on leave?" and "Log"; the node grid (ID, Name, Mode, Health, Uptime, HW and
//! SW Version, SW CRC, Menu); the current node's details under it; and the nodes' debug messages
//! at the bottom, newest first, a hundred kept.
//!
//! * **Connect** (`but_connect_Click`, `:1635-1681`): a second click is `Disconnect`. Otherwise
//!   the node list is emptied, the lists disabled, the button says "Disconnect", and the chosen
//!   bus starts:
//!   - MAVLinkCAN1/2 (`StartMavlinkCAN`, `:88-202`): a second later and every second after,
//!     `MAV_CMD_CAN_FORWARD` for the bus, not waited for; each `CAN_FRAME` the shown vehicle
//!     sends goes to our node as an SLCAN line, and each line our node writes goes back as a
//!     `CAN_FRAME` on bus `bus - 1`. Filter is enabled.
//!   - SLCAN (`startslcan`, `:204-265`): with the link open, `CAN_SLCAN_CPORT` set to 1 - if it
//!     was 0, "Reboot required after setting CPORT. Please reboot!" and nothing more - then
//!     `CAN_SLCAN_TIMOUT` 2, `CAN_P1_DRIVER` 1, and `CAN_SLCAN_SERNUM` sent twice without
//!     waiting; then the link's port is taken for SLCAN. With no link, it asks whether the port in
//!     the connection box is an SLCAN port and opens that. "Trying to connect" shows while the
//!     adapter is opened (`StartSLCAN`'s commands, `DroneCAN.cs:187-232`).
//!   - MCastCan1/2 (`StartmcastCAN`, `:1509-1633`): UDP multicast on 239.65.82.0 or .1, the
//!     chosen network interface.
//!
//!   Our node (127) then sends its status every second and answers as `org.missionplanner`,
//!   serves files, and gives node ids (`SetupSLCanPort`, `:267-477`). A node's first status adds
//!   its row, with the status's numbers; each later one sets its health, mode and uptime in words
//!   and, while it is still named "?", asks for its info, whose answer fills the name, versions,
//!   CRC and unique id. Our own node is one of the rows: every message the node sends it also
//!   hears (`InvokeMessageReceived`).
//! * **The grid** (`myDataGridView1_CellClick`, `_RowEnter`, `_RowsAdded`,
//!   `:486-502, 708-711, 725-738`): the Menu column says "Menu" and a click on it opens the
//!   node's menu, as does a right click on the grid (its `ContextMenu`); a row clicked is the
//!   current one, whose details show below. `RowEnter` looks the row's node up and posts
//!   nothing - here, nothing.
//! * **The node's menu**: Parameters (`GetParameters`, `:665-678`, "Getting Params" and then the
//!   parameter window, `Controls/DroneCANParams.cs`), Restart (`RestartNode`), Update and Update
//!   Beta (`FirmwareUpdate`, `:504-663`: "Do you want to search the internet for an update?" -
//!   Yes looks on CubePilot's server for its kind of node, else in the firmware manifest,
//!   downloads it and updates; No asks for a `.bin` or `.apj`), and the two passthroughs for a
//!   Here3 and a Here3+/Here4 (`menu_passthrough_Click`, `menu_passthrough4_Click`,
//!   `:740-865, 990-1208`): a TCP port whose client gets the GPS's RTCM (or its tunnel) and
//!   whose bytes go to it.
//! * **Filter** (`but_filter_Click`, `:897-988`): a window of every message type, those the page
//!   needs ticked; each box changed sends `CAN_FILTER_MODIFY` while the MAVLink bus runs.
//! * **Stats** (`but_stats_Click`, `:1392-1471`): a window of the nodes' `CanStats` and `Stats`.
//! * **Check for Updates** (`CHK_checkupdate_CheckedChanged`, `:1700-1751`): each node's info
//!   compared with the manifest's peripheral firmware, "New firmware" said for a newer one; and
//!   for each node's first info while ticked.
//! * **Log**: the bus's SLCAN lines, both ways, to `<LogDir>/<yyyy-MM-dd HH-mm-ss>.can`.
//! * `Deactivate` is `Disconnect`: the bus stopped, the node stopped - with the adapter's close
//!   command when "Exit SLCAN on leave?" is ticked - and the lists enabled again (`:680-706`).
//!
//! Where it differs from the C#, and why (each also at its site):
//!
//! * **Inspector** opens `config/dronecan_inspector.rs` - `Controls/DroneCANInspector.cs`, a
//!   window of every message heard, field by field, with Graph It and Subscribe - over the
//!   page's node, drawn over the page; the 27 types decoded here show their fields and any
//!   other its payload bytes (that module's notes). With no node the click is "Please connect
//!   first" on the status line, the C#'s box (the owner's ruling).
//! * The C#'s message boxes for what the window can show - "Check port settings or Port in
//!   use?", "No network interfaces found", "Forwarder problem", a parameter window's "Failed to
//!   save", an update's error, the C#'s exceptions out of a click - go on the status line (the
//!   owner's ruling of 2026-09-25). Questions and reports keep their boxes.
//! * SLCAN with the link open disconnects it, as the connect button does (`doDisconnect`),
//!   and opens its URL for SLCAN: the C# swaps a closed port into `MainV2.comPort` and leaves
//!   it looking connected.
//! * With no link, the SLCAN port is the connection box's: a network kind's questions are not
//!   asked again - the answers saved for them, or their defaults, are used.
//! * The progress window shows while an SLCAN adapter is opened; on the MAVLink and multicast
//!   buses the C#'s `StartSLCAN` writes its commands into a converter that drops them (lines of
//!   four characters or fewer), so there is nothing to wait for, and nothing is shown.
//! * "Exit SLCAN on leave?" unticked leaves the adapter in SLCAN mode, but the port is closed
//!   either way: nothing here can hold a port open once its node is gone.
//! * The grid is drawn every frame: the C#'s one-second `ResetBindings` timer only batches the
//!   redraws of a grid whose rows the reader thread changes.
//! * The details' text boxes are drawn read-only; typing into them in the C# changes only the
//!   grid's copy.
//! * Updating from the internet loads the firmware manifest here when the page has none: the
//!   C# reads `APFirmware.Manifest`, a static that `MainV2`'s daily check or the Install
//!   Firmware page fills, and throws without it; this application has no such static. "Check for
//!   Updates" uses the manifest such a search loaded, and returns without one, as the C# does
//!   with the static unset.
//! * Windows (parameters, filter, stats) are drawn over the window, one at a time, with a close
//!   box; the C#'s are forms of their own.
//!
//! The parameter window (`Controls/DroneCANParams.cs`) is [`ParamsWindow`]: its notes are there.
//!
//! Not ported, having no callers - no Designer wires them, nothing calls them (D16):
//! `but_slcandirect_Click`, `but_slcanmavlink_Click`, `but_slcanmode2_2_Click`, `but_mcast1_Click`
//! and `but_mcast2_Click` (`:77-86, 891-895, 1473-1483`), the buttons the interface list replaced.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::Lock as _;
use std::collections::{BTreeMap, VecDeque};
use std::io::{Read as _, Write as _};
use std::net::{TcpListener, TcpStream, UdpSocket};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::sync::{Arc, Mutex};
use web_time::{Duration, Instant};

use gpui::{
    AnyElement, ClickEvent, Context, Div, FocusHandle, KeyDownEvent, MouseButton, MouseDownEvent,
    SharedString, Window, div, prelude::*, px, rgb,
};
use mp_dronecan::dsdl::{
    self, GetNodeInfoRes, GetSetRes, HEALTH_CRITICAL, HEALTH_ERROR, HEALTH_OK, HEALTH_WARNING,
    MODE_INITIALIZATION, MODE_MAINTENANCE, MODE_OFFLINE, MODE_OPERATIONAL, MODE_SOFTWARE_UPDATE,
    TUNNEL_OPTION_LOCK_PORT, TUNNEL_PROTOCOL_GPS_GENERIC, TunnelTargetted, ascii,
};
use mp_dronecan::jobs::{GetParameters, Job, Poll, Request, SetValue, Update, UpdateEnd};
use mp_dronecan::node::PRIORITY;
use mp_dronecan::{Event, Identity, Message as CanMessage, Node, Received, mavlink, mcast, names};
use mp_link::inspector::PacketSubscription;
use mp_link::requests::RequestOutcome;
use mp_mavlink_dialects::all::{CanFrame, MavMessage, ParamSet};
use mp_vehicle::VehicleId;

use super::optional::{at, button, heading, label, rule};
use crate::MissionPlanner;
use crate::config::failsafe::CheckState;
use crate::config::servo_output::{Check, Combo, ERROR_TITLE, Message, check_box, combo_box};
use crate::setup::Key;
use crate::telemetry::{Lookup, Telemetry, TelemetryView};
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action, panel, theme};

// ---------------------------------------------------------------------------------------------
// The Designer's words and places.
// ---------------------------------------------------------------------------------------------

/// The page's title in Initial Setup's list, and `label6.Text`.
/// `// C#: GCSViews/InitialSetup.cs:276; GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:101`
pub const TITLE: &str = "DroneCAN/UAVCAN";

/// `ConnectionTypes`, the interface list, in its order.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:68-75`
pub const INTERFACES: [&str; 5] = [
    "SLCAN",
    "MAVLinkCAN1",
    "MAVLinkCAN2",
    "MCastCan1",
    "MCastCan2",
];

/// `but_connect.Text`, and `Disconnect`'s.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:487; ConfigDroneCAN.cs:704, 1658`
pub const CONNECT: &str = "Connect";
/// The button while a bus runs.
pub const DISCONNECT: &str = "Disconnect";
/// `but_filter.Text`. `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:447`
pub const FILTER: &str = "Filter";
/// `but_stats.Text`. `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:458`
pub const STATS: &str = "Stats";
/// `but_uavcaninspector.Text`. `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:176`
pub const INSPECTOR: &str = "Inspector";
/// `CHK_checkupdate.Text`. `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:605`
pub const CHECK_UPDATES: &str = "Check for Updates";
/// `chk_canonclose.Text`. `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:438`
pub const EXIT_SLCAN: &str = "Exit SLCAN on leave?";
/// `chk_log.Text`. `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:381`
pub const LOG: &str = "Log";
/// `label1.Text`. `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:167-168`
pub const NOTE: &str = "After enabling SLCAN, you will no longer be able to connect via \
     MAVLINK.\r\nYou must leave this screen and wait 2 seconds before connecting again\r\n";

/// The node grid's columns: header and width.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:521-597`
pub const COLUMNS: [(&str, f32); 9] = [
    ("ID", 40.0),
    ("Name", 110.0),
    ("Mode", 90.0),
    ("Health", 43.0),
    ("Uptime", 60.0),
    ("HW Version", 50.0),
    ("SW Version", 80.0),
    ("SW CRC", 110.0),
    ("Menu", 50.0),
];
/// The Menu column's text, `RowsAdded`'s value and the column's `NullValue`.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:710; ConfigDroneCAN.Designer.cs:592`
pub const MENU_CELL: &str = "Menu";

/// The debug grid's columns; Text fills the rest.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:400-428`
pub const DEBUG_COLUMNS: [(&str, f32); 3] = [("Node", 40.0), ("Level", 40.0), ("Source", 50.0)];
/// The fourth's header.
pub const DEBUG_TEXT: &str = "Text";
/// How many debug rows are kept.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:469-472`
pub const DEBUG_ROWS: usize = 100;

/// The details' labels, a row each.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:323-363`
pub const DETAIL_LABELS: [&str; 5] = [
    "Node ID / Name",
    "Mode / Health / Uptime",
    "Vendor-specific code",
    "Software version/CRC64",
    "Hardware version/UID",
];

/// The node menu's items, in order.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:113-158`
pub const MENU: [(&str, &str); 6] = [
    ("dronecan-menu-parameters", "Parameters"),
    ("dronecan-menu-restart", "Restart"),
    ("dronecan-menu-update", "Update"),
    ("dronecan-menu-updatebeta", "Update Beta"),
    ("dronecan-menu-passthrough", "CANPassThrough Here3"),
    ("dronecan-menu-passthrough4", "CANPassThough Here3+/4"),
];

/// `Strings.PleaseConnect`. `// C#: ExtLibs/Strings/Strings.resx:205-207`
pub const PLEASE_CONNECT: &str = "Please connect first";
/// `startslcan`'s question with no link, and its caption.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:211-214`
pub const NOT_CONNECTED_SLCAN: &str = "You are not currently connected via mavlink. Please \
     make sure the device is already in slcan mode or this is the slcan serialport.";
/// Its caption.
pub const SLCAN_CAPTION: &str = "SLCAN";
/// After setting `CAN_SLCAN_CPORT` from 0.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:225-226`
pub const REBOOT_REQUIRED: &str = "Reboot required after setting CPORT. Please reboot!";
/// `Strings.CheckPortSettingsOr`. `// C#: ExtLibs/Strings/Strings.resx:335-337`
pub const CHECK_PORT: &str = "Check port settings or Port in use?";
/// The progress window while an adapter opens.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:348`
pub const TRYING_TO_CONNECT: &str = "Trying to connect";
/// `StartmcastCAN` with no interface. `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:1513`
pub const NO_INTERFACES: &str = "No network interfaces found";
/// `FirmwareUpdate`'s question, and its caption.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:518-519`
pub const SEARCH_INTERNET: &str = "Do you want to search the internet for an update?";
/// Its caption.
pub const UPDATE_CAPTION: &str = "Update";
/// The download's progress. `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:542`
pub const DOWNLOAD_FW: &str = "Download FW";
/// The end of a file's send. `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:556, 611`
pub const FILE_SEND_COMPLETE: &str = "File send complete";
/// `Strings.UpdateNotFound`, text and caption. `// C#: ExtLibs/Strings/Strings.resx:603-605`
pub const UPDATE_NOT_FOUND: &str = "No update available.";
/// `Strings.GettingParams`. `// C#: ExtLibs/Strings/Strings.resx:309-311`
pub const GETTING_PARAMS: &str = "Getting Params";
/// The passthroughs' first `InputBox`.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:752, 1007`
pub const ENTER_TCP_PORT: &str = "Enter TCP Port";
/// The Here3+/4 passthrough's second. `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:1012`
pub const ENTER_BAUD: &str = "Enter Baudrate";
/// A passthrough clicked again: the box's text and caption.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:746, 996`
pub const STOP: &str = "Stop";
/// Its caption.
pub const DISABLED_FORWARDING: &str = "Disabled forwarding";
/// `CheckSingleIDForUpdate`'s box's title.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:1749`
pub const NEW_FIRMWARE: &str = "New firmware";
/// `Strings.ShowMeAgain`.
pub const SHOW_ME_AGAIN: &str = crate::config::adsb::SHOW_ME_AGAIN;
/// The setting `SetCheckboxFromConfig` reads "Check for Updates" from; nothing writes it.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:55`
pub const UPDATE_CHECK_KEY: &str = "dronecan_updatecheck";

/// `$this.Size`. `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs:629`
const PAGE_SIZE: (f32, f32) = (798.0, 612.0);
/// `label6.Location`.
const HEADING_AT: (f32, f32) = (3.0, 0.0);
/// `groupBox5`: a rule.
const RULE_AT: (f32, f32, f32) = (0.0, 26.0, 796.0);
/// `cmb_interfacetype`.
const INTERFACE_AT: (f32, f32, f32, f32) = (7.0, 37.0, 179.0, 21.0);
/// `cmb_networkinterface`.
const NETWORK_AT: (f32, f32, f32, f32) = (7.0, 64.0, 179.0, 21.0);
/// `but_connect`.
const CONNECT_AT: (f32, f32, f32, f32) = (192.0, 48.0, 75.0, 23.0);
/// `but_filter`.
const FILTER_AT: (f32, f32, f32, f32) = (290.0, 35.0, 42.0, 23.0);
/// `but_stats`.
const STATS_AT: (f32, f32, f32, f32) = (290.0, 60.0, 42.0, 23.0);
/// `but_uavcaninspector`.
const INSPECTOR_AT: (f32, f32, f32, f32) = (338.0, 35.0, 57.0, 23.0);
/// `CHK_checkupdate.Location`.
const CHECK_UPDATES_AT: (f32, f32) = (338.0, 61.0);
/// `label1.Location`.
const NOTE_AT: (f32, f32) = (401.0, 35.0);
/// `chk_canonclose.Location`.
const EXIT_SLCAN_AT: (f32, f32) = (608.0, 3.0);
/// `chk_log.Location`.
const LOG_AT: (f32, f32) = (753.0, 3.0);
/// `myDataGridView1`.
const GRID_AT: (f32, f32, f32, f32) = (7.0, 89.0, 788.0, 229.0);
/// `tableLayoutPanel1`.
const DETAILS_AT: (f32, f32, f32, f32) = (7.0, 324.0, 788.0, 136.0);
/// Its columns, in percent of its width.
const DETAIL_COLUMNS: [f32; 4] = [17.707_01, 25.605_09, 28.245_11, 28.245_11];
/// `DGDebug`.
const DEBUG_AT: (f32, f32, f32, f32) = (7.0, 465.0, 788.0, 144.0);
/// A grid's row headers, as a WinForms grid draws them by default.
const ROW_HEADER: f32 = 41.0;
/// A grid's header row.
const HEADER_HEIGHT: f32 = 23.0;
/// A grid's row.
const ROW_HEIGHT: f32 = 22.0;
/// A context menu's item.
const MENU_ROW: f32 = 22.0;

// ---------------------------------------------------------------------------------------------
// The model: `DroneCANModel`, the debug rows, the interface kinds.
// ---------------------------------------------------------------------------------------------

/// `ConnectionTypes`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionType {
    /// An SLCAN adapter, or the autopilot's SLCAN port.
    Slcan,
    /// The autopilot's CAN bus 1 or 2, forwarded over MAVLink.
    MavlinkCan(u8),
    /// Multicast bus 0 or 1.
    McastCan(u8),
}

impl ConnectionType {
    /// The kind at a place in the list.
    #[must_use]
    pub const fn from_index(index: usize) -> Option<Self> {
        Some(match index {
            0 => Self::Slcan,
            1 => Self::MavlinkCan(1),
            2 => Self::MavlinkCan(2),
            3 => Self::McastCan(0),
            4 => Self::McastCan(1),
            _ => return None,
        })
    }

    /// Its name, as the list shows it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Slcan => "SLCAN",
            Self::MavlinkCan(1) => "MAVLinkCAN1",
            Self::MavlinkCan(_) => "MAVLinkCAN2",
            Self::McastCan(0) => "MCastCan1",
            Self::McastCan(_) => "MCastCan2",
        }
    }

    /// `cmb_interfacetype_SelectedIndexChanged`: whether the network interface list shows.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:1683-1698`
    #[must_use]
    pub const fn shows_networks(self) -> bool {
        matches!(self, Self::McastCan(_))
    }
}

/// `DroneCANModel`: a row of the node grid.
/// `// C#: GCSViews/ConfigurationView/DroneCANModel.cs:6-18`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeRow {
    /// `ID`.
    pub id: u8,
    /// `Name`: "?" until the node's info comes.
    pub name: String,
    /// `Mode`.
    pub mode: String,
    /// `Health`.
    pub health: String,
    /// `Uptime`, in whole seconds.
    pub uptime: u32,
    /// `HardwareVersion`; null until the info comes.
    pub hardware_version: Option<String>,
    /// `SoftwareVersion`.
    pub software_version: Option<String>,
    /// `SoftwareCRC`.
    pub software_crc: u64,
    /// `HardwareUID`.
    pub hardware_uid: Option<String>,
    /// `VSC`.
    pub vsc: u16,
}

/// `TimeSpan.ToString()` of whole seconds: `hh:mm:ss`, or `d.hh:mm:ss` from a day.
#[must_use]
pub fn uptime_text(seconds: u32) -> String {
    let days = seconds / 86_400;
    let rest = seconds % 86_400;
    let clock = format!(
        "{:02}:{:02}:{:02}",
        rest / 3600,
        (rest % 3600) / 60,
        rest % 60
    );
    if days > 0 {
        format!("{days}.{clock}")
    } else {
        clock
    }
}

/// The health's word, for a status after the first.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:386-404`
#[must_use]
pub const fn health_text(health: u8) -> Option<&'static str> {
    Some(match health {
        HEALTH_OK => "OK",
        HEALTH_WARNING => "WARNING",
        HEALTH_ERROR => "ERROR",
        HEALTH_CRITICAL => "CRITICAL",
        _ => return None,
    })
}

/// The mode's word.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:406-428`
#[must_use]
pub const fn mode_text(mode: u8) -> Option<&'static str> {
    Some(match mode {
        MODE_OPERATIONAL => "OPERATIONAL",
        MODE_INITIALIZATION => "INITIALIZATION",
        MODE_MAINTENANCE => "MAINTENANCE",
        MODE_SOFTWARE_UPDATE => "SOFTWARE_UPDATE",
        MODE_OFFLINE => "OFFLINE",
        _ => return None,
    })
}

/// A row of `DGDebug`: node, level, source, text.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DebugRow {
    /// `frame.SourceNode`.
    pub node: u8,
    /// `debug.level.value`.
    pub level: u8,
    /// `debug.source`.
    pub source: String,
    /// `debug.text`.
    pub text: String,
}

// ---------------------------------------------------------------------------------------------
// The buses.
// ---------------------------------------------------------------------------------------------

/// `StartMavlinkCAN`'s state: the forwarding loop and the `CAN_FRAME` subscription.
#[derive(Debug)]
struct MavlinkBus {
    /// `bus`, 1 or 2.
    bus: u8,
    /// The loop's next `CAN_FORWARD`: a second after the start (its "allows old instance to
    /// exit" sleep), then every second.
    next_forward: Instant,
    /// `SubscribeToPacketType(CAN_FRAME, ..., exclusive)`.
    subscription: Option<PacketSubscription>,
    /// The frames the subscription has heard, for the next tick.
    heard: Arc<Mutex<VecDeque<CanFrame>>>,
    /// `CAN_FORWARD`s sent.
    forwards: usize,
    /// `CAN_FRAME`s (and `CANFD_FRAME`s) sent.
    sent: usize,
    /// `CAN_FRAME`s heard.
    received: usize,
}

/// What the SLCAN thread is told.
#[derive(Debug)]
enum SlcanCommand {
    /// `WriteToStreamSLCAN`: a line, with its `\r`.
    Write(String),
    /// `Stop(closestream)`: the close command first when true, then the port closed.
    Stop(bool),
}

/// What the SLCAN thread says.
#[derive(Debug)]
enum SlcanEvent {
    /// A line read while the adapter was opened: logged, not handed to the node.
    Handshake(String),
    /// `StartSLCAN` has opened the adapter.
    Opened,
    /// A line read.
    Line(String),
    /// The port would not open, or failed.
    Failed(String),
}

/// An SLCAN adapter: the port on a thread of its own.
#[derive(Debug)]
struct SlcanBus {
    commands: Sender<SlcanCommand>,
    events: Receiver<SlcanEvent>,
    /// Whether `StartSLCAN` has finished opening the adapter.
    opened: bool,
}

/// `StartmcastCAN`'s socket and its receive loop.
#[derive(Debug)]
struct McastBus {
    /// The bus, 0 or 1.
    bus: u8,
    socket: Arc<UdpSocket>,
    lines: Receiver<String>,
    /// `mavlinkCANRun`, for the receive loop.
    run: Arc<AtomicBool>,
}

/// The running bus.
#[derive(Debug)]
enum Bus {
    Mavlink(MavlinkBus),
    Slcan(SlcanBus),
    Mcast(McastBus),
}

impl Bus {
    /// The kind, for the facts.
    const fn kind(&self) -> &'static str {
        match self {
            Self::Mavlink(bus) if bus.bus == 1 => "MAVLinkCAN1",
            Self::Mavlink(_) => "MAVLinkCAN2",
            Self::Slcan(_) => "SLCAN",
            Self::Mcast(bus) if bus.bus == 0 => "MCastCan1",
            Self::Mcast(_) => "MCastCan2",
        }
    }
}

/// `StartSLCAN`'s reads: a line ends at `\r` or `\a`, kept in it, or at the 1100 ms timeout.
/// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:83-141`
fn read_line(transport: &mut dyn mp_transport::Transport, pending: &mut Vec<u8>) -> String {
    let deadline = Instant::now() + Duration::from_millis(1100);
    let mut buf = [0u8; 1];
    loop {
        if let Some(end) = pending.iter().position(|b| *b == b'\r' || *b == 0x07) {
            let line: Vec<u8> = pending.drain(..=end).collect();
            return String::from_utf8_lossy(&line).into_owned();
        }
        if Instant::now() > deadline {
            let line: Vec<u8> = std::mem::take(pending);
            return String::from_utf8_lossy(&line).into_owned();
        }
        match transport.read(&mut buf) {
            Ok(1) => pending.push(buf[0]),
            // A transport whose read does not wait: a breath before the next.
            Ok(_) => wasm_thread::sleep(Duration::from_millis(1)),
            Err(_) => {
                let line: Vec<u8> = std::mem::take(pending);
                return String::from_utf8_lossy(&line).into_owned();
            }
        }
    }
}

/// The SLCAN thread: `StartSLCAN`'s commands, each answered, then its reader; and the writes.
/// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:187-297, 407-428, 1347-1394`
fn run_slcan(
    mut transport: Box<dyn mp_transport::Transport>,
    commands: &Receiver<SlcanCommand>,
    events: &Sender<SlcanEvent>,
) {
    let _ = transport.set_read_timeout(Duration::from_millis(20));
    let mut pending = Vec::new();
    // "cleanup", 50 ms, and a read of whatever was waiting, for up to a second.
    if transport.write_all(mp_dronecan::slcan::CLEANUP).is_err() {
        let _ = events.send(SlcanEvent::Failed(CHECK_PORT.to_owned()));
        return;
    }
    wasm_thread::sleep(Duration::from_millis(50));
    let until = Instant::now() + Duration::from_secs(1);
    let mut buf = [0u8; 1024];
    while Instant::now() < until {
        match transport.read(&mut buf) {
            Ok(0) | Err(_) => break,
            Ok(_) => {}
        }
    }
    for command in mp_dronecan::slcan::OPEN_COMMANDS {
        if let Ok(SlcanCommand::Stop(_)) | Err(TryRecvError::Disconnected) = commands.try_recv() {
            transport.close();
            return;
        }
        if transport.write_all(command).is_err() {
            let _ = events.send(SlcanEvent::Failed(CHECK_PORT.to_owned()));
            return;
        }
        let answer = read_line(transport.as_mut(), &mut pending);
        let _ = events.send(SlcanEvent::Handshake(answer));
    }
    let _ = events.send(SlcanEvent::Opened);
    loop {
        loop {
            match commands.try_recv() {
                Ok(SlcanCommand::Write(line)) => {
                    let _ = transport.write_all(line.as_bytes());
                }
                Ok(SlcanCommand::Stop(close)) => {
                    if close {
                        let _ = transport.write_all(mp_dronecan::slcan::CLOSE);
                    }
                    transport.close();
                    return;
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    transport.close();
                    return;
                }
            }
        }
        match transport.read(&mut buf) {
            Ok(n) if n > 0 => {
                pending.extend_from_slice(buf.get(..n).unwrap_or(&[]));
                while let Some(end) = pending.iter().position(|b| *b == b'\r' || *b == 0x07) {
                    let line: Vec<u8> = pending.drain(..=end).collect();
                    let line = String::from_utf8_lossy(&line).into_owned();
                    if events.send(SlcanEvent::Line(line)).is_err() {
                        transport.close();
                        return;
                    }
                }
            }
            Ok(_) => wasm_thread::sleep(Duration::from_millis(1)),
            Err(error) => {
                let _ = events.send(SlcanEvent::Failed(error.to_string()));
                return;
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The windows and boxes.
// ---------------------------------------------------------------------------------------------

/// A box's buttons, and what the answer does.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Asked {
    /// `startslcan` with no link: OK opens the connection box's port.
    SlcanWithoutLink,
    /// `FirmwareUpdate`'s question: Yes searches, No asks for a file.
    SearchInternet { node: u8, beta: bool },
    /// `MessageShowAgain(NEW_FIRMWARE, ...)`: OK, and "Show me again?".
    NewFirmware { again: bool },
    /// The parameter window's Write Params: `MessageShowAgain("Write Raw Params", ...)`.
    WriteParams { again: bool },
    /// The parameter window's Refresh Params: `WarningUpdateParamList`, OK/Cancel.
    RefreshParams,
}

/// A question showing.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Question {
    caption: &'static str,
    text: String,
    asked: Asked,
}

/// What a typed path is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PathFor {
    /// `FirmwareUpdate`'s `OpenFileDialog`, "*.bin;*.apj".
    Firmware { node: u8 },
    /// The parameter window's Load from file.
    LoadParams,
    /// The parameter window's Save to file.
    SaveParams,
}

/// An `InputBox`: what it asks, and for what.
#[derive(Debug)]
struct Input {
    title: &'static str,
    field: TextField,
    for_: InputFor,
}

/// What an `InputBox` is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InputFor {
    /// The Here3 passthrough's port.
    Here3Port,
    /// The Here3+/4 passthrough's port; the node and whether the port is locked.
    Here4Port { node: u8, lock: bool },
    /// Its baud rate, after the port.
    Here4Baud { node: u8, lock: bool, port: u16 },
}

/// The `ProgressReporterDialogue` showing.
#[derive(Debug)]
struct Progress {
    text: String,
    bar: crate::config::serial_ports::Bar,
    started: Instant,
    /// What its Cancel stops.
    for_: ProgressFor,
}

/// What a progress window waits for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ProgressFor {
    /// An SLCAN adapter opening.
    Slcan,
    /// `GetParameters`.
    Parameters,
    /// A firmware update.
    Update,
}

/// What the update's search and download end with: the file, and the manifest when it was
/// loaded for the search; or what went wrong, for the status line.
type Fetched = Result<(PathBuf, Option<Arc<mp_firmware::manifest::Manifest>>), String>;

/// The firmware update in flight: the download, then the update itself.
#[derive(Debug)]
enum UpdateFlow {
    /// The search and the download on their own thread.
    Fetching {
        node: u8,
        results: Receiver<Fetched>,
        cancelled: bool,
    },
    /// `can.Update`.
    Running(Update),
}

/// A passthrough's thread, and what it says.
#[derive(Debug)]
struct Passthrough {
    /// Here3+/4's target node and baud rate; `None` for the Here3's RTCM.
    tunnel: Option<(u8, u32)>,
    to_client: Sender<Vec<u8>>,
    from_client: Receiver<PassthroughEvent>,
    stop: Arc<AtomicBool>,
    /// Whether a client is connected.
    client: bool,
    /// Here3+/4: when the tunnel last carried bytes, for the half-second keep-alive.
    last_send: Instant,
    /// Here3+/4: the six baud-setting tunnels still to send, 100 ms apart.
    bauds: VecDeque<u32>,
    next_baud: Instant,
    /// Here3+/4's `portlock`.
    lock: bool,
}

/// What a passthrough's thread says.
#[derive(Debug)]
enum PassthroughEvent {
    /// A client connected.
    Connected,
    /// Its bytes, at most 128 (120 for a tunnel) a read.
    Bytes(Vec<u8>),
    /// It went.
    Gone,
    /// The listener failed: "Forwarder problem".
    Failed(String),
}

/// The listener's thread: one client at a time, its bytes to the page, the page's to it.
fn run_passthrough(
    listener: &TcpListener,
    chunk: usize,
    stop: &AtomicBool,
    to_client: &Receiver<Vec<u8>>,
    events: &Sender<PassthroughEvent>,
) {
    let _ = listener.set_nonblocking(true);
    let mut client: Option<TcpStream> = None;
    let mut buf = vec![0u8; chunk];
    while !stop.load(Ordering::Acquire) {
        if client.is_none() {
            match listener.accept() {
                Ok((stream, _)) => {
                    let _ = stream.set_nodelay(true);
                    let _ = stream.set_nonblocking(true);
                    client = Some(stream);
                    let _ = events.send(PassthroughEvent::Connected);
                }
                Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                    wasm_thread::sleep(Duration::from_millis(5));
                }
                Err(error) => {
                    let _ = events.send(PassthroughEvent::Failed(error.to_string()));
                    return;
                }
            }
            // What came for a client before one connected is dropped.
            while to_client.try_recv().is_ok() {}
            continue;
        }
        let mut gone = false;
        if let Some(stream) = client.as_mut() {
            while let Ok(bytes) = to_client.try_recv() {
                if stream
                    .write_all(&bytes)
                    .and_then(|()| stream.flush())
                    .is_err()
                {
                    gone = true;
                }
            }
            match stream.read(&mut buf) {
                Ok(0) => gone = true,
                Ok(n) => {
                    let _ = events.send(PassthroughEvent::Bytes(
                        buf.get(..n).unwrap_or(&[]).to_vec(),
                    ));
                }
                Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                    wasm_thread::sleep(Duration::from_millis(1));
                }
                Err(_) => gone = true,
            }
        }
        if gone {
            client = None;
            let _ = events.send(PassthroughEvent::Gone);
        }
    }
}

/// The UBX `CFG-PRT` the Here3+/4 passthrough sends through the tunnel at each of six bauds,
/// setting the GPS's port to `baudrate`, two `0x55` in front.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:1031-1046`
#[must_use]
pub fn here4_baud_packet(baudrate: u32) -> Vec<u8> {
    let [b0, b1, b2, _] = baudrate.to_le_bytes();
    let payload = [
        0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, b0, b1, b2, 0x00, 0x23, 0x00, 0x23, 0x00,
        0x00, 0x00, 0x00, 0x00,
    ];
    let mut packet = vec![0x55, 0x55];
    packet.extend(crate::config::rtk_inject::ubx::generate(
        0x06, 0x00, &payload,
    ));
    packet
}

/// The bauds the GPS might be at, which the packet is sent at in turn.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:1033`
pub const HERE4_BAUDS: [u32; 6] = [9600, 38400, 57600, 115_200, 230_400, 460_800];

// ---------------------------------------------------------------------------------------------
// The parameter window: `Controls/DroneCANParams.cs`.
// ---------------------------------------------------------------------------------------------

/// A control's `Location` and `Size`: x, y, width, height.
pub type Place = (f32, f32, f32, f32);

/// The parameter window's caption: "UAVCAN Params - " and the node.
/// `// C#: Controls/DroneCANParams.cs:47`
pub const PARAMS_CAPTION: &str = "UAVCAN Params - ";
/// Its buttons, top to bottom, with their places.
/// `// C#: Controls/DroneCANParams.resx`
pub const PARAMS_BUTTONS: [(&str, &str, Place); 7] = [
    (
        "dronecan-params-load",
        "Load from file",
        (555.0, 7.0, 104.0, 19.0),
    ),
    (
        "dronecan-params-save",
        "Save to file",
        (555.0, 35.0, 104.0, 19.0),
    ),
    (
        "dronecan-params-write",
        "Write Params",
        (555.0, 69.0, 103.0, 19.0),
    ),
    (
        "dronecan-params-refresh",
        "Refresh Params",
        (555.0, 94.0, 103.0, 19.0),
    ),
    (
        "dronecan-params-compare",
        "Compare Params",
        (555.0, 119.0, 103.0, 19.0),
    ),
    (
        "dronecan-params-commit",
        "Commit Params",
        (554.0, 144.0, 104.0, 22.0),
    ),
    (
        "dronecan-params-reset",
        "Reset to Default",
        (555.0, 261.0, 103.0, 21.0),
    ),
];
/// `label1.Text`, under the buttons. `// C#: Controls/DroneCANParams.resx:303-305`
pub const PARAMS_RAW: &str = "All Units are in raw \nformat with no scaling";
/// `label2.Text`. `// C#: Controls/DroneCANParams.resx:364`
pub const PARAMS_SEARCH: &str = "Search";
/// `chk_modified.Text`. `// C#: Controls/DroneCANParams.resx:428`
pub const PARAMS_MODIFIED: &str = "Modified";
/// The grid's columns and widths; Default fills.
/// `// C#: Controls/DroneCANParams.resx:481-527`
pub const PARAMS_COLUMNS: [(&str, f32); 6] = [
    ("Command", 126.0),
    ("Value", 69.0),
    ("Min", 56.0),
    ("Max", 175.0),
    ("Default", 79.0),
    ("Fav", 30.0),
];
/// `$this.Size`. `// C#: Controls/DroneCANParams.resx:556-558`
const PARAMS_SIZE: (f32, f32) = (658.0, 357.0);
/// `Params`.
const PARAMS_GRID_AT: (f32, f32, f32, f32) = (14.0, 3.0, 535.0, 351.0);
/// `label1.Location`, `label2.Location`, `txt_search`, `chk_modified.Location`.
const PARAMS_RAW_AT: (f32, f32) = (555.0, 169.0);
const PARAMS_SEARCH_LABEL_AT: (f32, f32) = (555.0, 285.0);
const PARAMS_SEARCH_AT: (f32, f32, f32, f32) = (554.0, 301.0, 104.0, 20.0);
const PARAMS_MODIFIED_AT: (f32, f32) = (554.0, 327.0);
/// Write Params' `MessageShowAgain`. `// C#: Controls/DroneCANParams.cs:408`
pub const WRITE_TITLE: &str = "Write Raw Params";
/// Its text.
pub const WRITE_SURE: &str = "Are you Sure?";
/// The setting its "Show me again?" is kept under.
pub const WRITE_SHOW_AGAIN_KEY: &str = "SHOWAGAIN_Write_Raw_Params";
/// The setting "New firmware"'s is kept under.
pub const NEW_FIRMWARE_SHOW_AGAIN_KEY: &str = "SHOWAGAIN_New_firmware";
/// Write Params' end: text and caption. `// C#: Controls/DroneCANParams.cs:469-471`
pub const PARAMS_SAVED: &str = "Parameters successfully saved.";
/// Their caption.
pub const SAVED_CAPTION: &str = "Saved";
/// Commit Params: the node's no. `// C#: Controls/DroneCANParams.cs:262`
pub const FAILED_TO_SAVE: &str = "Failed to save";
/// Commit Params' report. `// C#: Controls/DroneCANParams.cs:270`
pub const COMMITTED: &str = "Parameters committed to non-volatile memory";
/// A value outside min and max. `// C#: Controls/DroneCANParams.cs:247`
pub const INVALID_VALUE: &str = "Invalid value";
/// Compare Params and Reset to Default, not ported: the status line's words.
pub const NOT_PORTED_COMPARE: &str = "Compare Params is not ported: the C#'s compares the file \
     with the autopilot's parameters (MainV2.comPort.MAV.param), not the node's";
/// The status line's words for Reset to Default.
pub const NOT_PORTED_RESET: &str = "Reset to Default is not ported: the C#'s resets the \
     autopilot's parameters and reboots it (MainV2.comPort), not the node's";

/// A change waiting for Write Params: `_changes`' value, a number or the text typed.
#[derive(Debug, Clone, PartialEq)]
pub enum Change {
    /// `double.TryParse` read it.
    Number(f64),
    /// It is not a number.
    Text(String),
}

/// A cell's colour after an edit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Shade {
    /// The theme's.
    Normal,
    /// `Color.Green`: changed. (The C#'s red is its `catch`, for a cell whose value is not a
    /// string; every cell here holds text.)
    Changed,
}

/// A row of the parameter grid.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamRow {
    /// `Command`: the name.
    pub name: String,
    /// `Value`.
    pub value: String,
    /// `Min`.
    pub min: String,
    /// `Max`.
    pub max: String,
    /// `Default`.
    pub default: String,
    /// `Fav`.
    pub fav: bool,
    /// Whether the filter shows it.
    pub visible: bool,
    /// The Value cell's colour.
    pub shade: Shade,
}

/// What the window is waiting for. One of these at most, in the one window: the difference in
/// the variants' sizes costs nothing worth a box.
#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
enum ParamsBusy {
    /// Write Params: each change set in turn, then the save.
    Writing {
        queue: VecDeque<String>,
        current: Option<(String, Request)>,
        saving: Option<Request>,
    },
    /// Refresh Params.
    Refreshing(GetParameters),
    /// Commit Params.
    Committing(Request),
}

/// The parameter window.
///
/// * The rows: every parameter of the list, the value, min, max and default as `GetValue()`
///   gives them (a real as `float.ToString()`), Fav ticked from `fav_params`; sorted by name,
///   favourites first (`processToScreen`, `OnParamsOnSortCompare`, `:94-150, 584-609`).
/// * A Value edit is checked against min and max when both are numbers - "Invalid value" and
///   the old text back otherwise - and then `RCn_REV`/`HSn_REV` 0 becomes -1, the cell turns
///   green and the change is kept for Write Params (`:206-255, 639-673`).
/// * Write Params (`:406-472`): "Write Raw Params - Are you Sure?" with its "Show me again?",
///   each change set with `SetParameter` - `ENABLE`s first - its cell back to normal and the
///   change forgotten whatever the node answered (the C# counts only exceptions, which
///   `SetParameter` does not throw), then `SaveConfig`, then "Parameters successfully saved.".
/// * Refresh Params (`:318-346`): `WarningUpdateParamList`, OK/Cancel; OK reads the list again.
/// * Commit Params (`:257-272`): `SaveConfig`; "Failed to save" on a no (the status line here),
///   then "Parameters committed to non-volatile memory" either way.
/// * Load from file (`:299-316, 532-582`): the file's values into the cells that differ, each an
///   edit; Save to file (`:373-404`): each row's value as a number to a `.param` file, "Invalid
///   number entered" for each that is not one.
/// * Search (`:479-530, 675-682`): half a second after the last key, a pattern of two or more
///   characters (`*` as any) shows the rows with a cell it matches; "Modified" shows only the
///   changed rows.
/// * Fav (`:611-637`): the name added to or taken from `fav_params`, and the rows sorted again.
///
/// Not ported, and why: **Compare Params** compares the file with `MainV2.comPort.MAV.param` - the
/// autopilot's parameters, not the node's - and **Reset to Default** sets the autopilot's
/// `FORMAT_VERSION` and `SYSID_SW_MREV` to 0 and reboots it (`:274-297, 348-371`): both act on the
/// vehicle from a node's window, a defect, and the second destroys its parameters. Each says so
/// on the status line ([`NOT_PORTED_COMPARE`], [`NOT_PORTED_RESET`]); a question for the owner.
/// The columns' widths (`rawparamuavcan_<column>_width`) are not kept: the columns are not
/// resizable here. Ctrl+S writes from the search box and the cell being edited, where the keys go.
#[derive(Debug)]
pub struct ParamsWindow {
    /// The node.
    pub node: u8,
    /// `_paramlist`.
    list: Vec<GetSetRes>,
    /// The rows, sorted.
    rows: Vec<ParamRow>,
    /// `_changes`.
    changes: BTreeMap<String, Change>,
    /// `txt_search`.
    pub search: TextField,
    /// The filter timer's due time.
    search_due: Option<Instant>,
    /// `chk_modified.Checked`.
    pub modified: bool,
    /// The Value cell being typed into: its row's name, and the text.
    editing: Option<(String, TextField)>,
    /// What it is waiting for.
    busy: Option<ParamsBusy>,
    /// The boxes it has shown, oldest first.
    messages: VecDeque<Message>,
}

impl ParamsWindow {
    /// `new DroneCANParams(can, node, paramlist)` and its `Activate`.
    #[must_use]
    pub fn new(node: u8, list: Vec<GetSetRes>, favourites: &[String]) -> Self {
        let mut window = Self {
            node,
            list,
            rows: Vec::new(),
            changes: BTreeMap::new(),
            search: TextField::new(""),
            search_due: None,
            modified: false,
            editing: None,
            busy: None,
            messages: VecDeque::new(),
        };
        window.process_to_screen(favourites);
        window
    }

    /// The caption.
    #[must_use]
    pub fn caption(&self) -> String {
        format!("{PARAMS_CAPTION}{}", self.node)
    }

    /// The rows, sorted.
    #[must_use]
    pub fn rows(&self) -> &[ParamRow] {
        &self.rows
    }

    /// `_changes`.
    #[must_use]
    pub const fn changes(&self) -> &BTreeMap<String, Change> {
        &self.changes
    }

    /// Whether a write, refresh or commit is running.
    #[must_use]
    pub const fn busy(&self) -> bool {
        self.busy.is_some()
    }

    /// `processToScreen`: the rows made from the list, sorted.
    /// `// C#: Controls/DroneCANParams.cs:94-150`
    fn process_to_screen(&mut self, favourites: &[String]) {
        let single = mp_log::netfmt::single;
        let mut rows: Vec<ParamRow> = Vec::new();
        for res in &self.list {
            let name = ascii(&res.name);
            if name.is_empty() || rows.iter().any(|row| row.name == name) {
                continue;
            }
            rows.push(ParamRow {
                fav: favourites.contains(&name),
                name,
                value: dsdl::value_text(&res.value, single),
                min: dsdl::numeric_text(&res.min_value, single),
                max: dsdl::numeric_text(&res.max_value, single),
                default: dsdl::value_text(&res.default_value, single),
                visible: true,
                shade: Shade::Normal,
            });
        }
        self.rows = rows;
        self.sort();
    }

    /// `Params.Sort(Command, Ascending)` through `OnParamsOnSortCompare`: favourites first, then
    /// by name as `string.CompareTo` orders them.
    fn sort(&mut self) {
        self.rows.sort_by(|a, b| {
            b.fav
                .cmp(&a.fav)
                .then_with(|| crate::config::adsb::culture_cmp(&a.name, &b.name))
        });
    }

    /// The Value cell begun.
    pub fn begin_edit(&mut self, name: &str) {
        let Some(row) = self.rows.iter().find(|row| row.name == name) else {
            return;
        };
        let mut field = TextField::new("");
        field.set(row.value.clone());
        self.editing = Some((name.to_owned(), field));
    }

    /// The cell being edited, and its text.
    #[must_use]
    pub fn editing(&self) -> Option<(&str, &str)> {
        self.editing
            .as_ref()
            .map(|(name, field)| (name.as_str(), field.value()))
    }

    /// A key in the cell; Enter commits, Escape cancels. Whether to redraw.
    pub fn edit_key(&mut self, event: &KeyDownEvent) -> bool {
        let Some((_, field)) = self.editing.as_mut() else {
            return false;
        };
        match field.key(event) {
            KeyOutcome::Submitted => {
                self.commit_edit();
                true
            }
            KeyOutcome::Cancelled => {
                self.editing = None;
                true
            }
            KeyOutcome::Changed => true,
            KeyOutcome::Ignored => false,
        }
    }

    /// The edit ended: `CellValidating`, then `CellValueChanged` if the text changed.
    /// `// C#: Controls/DroneCANParams.cs:240-255, 639-673`
    pub fn commit_edit(&mut self) {
        let Some((name, field)) = self.editing.take() else {
            return;
        };
        let text = field.value().to_owned();
        let Some(index) = self.rows.iter().position(|row| row.name == name) else {
            return;
        };
        let Some(row) = self.rows.get(index) else {
            return;
        };
        if !in_range(&row.min, &row.max, &text) {
            self.messages.push_back(Message {
                title: "",
                text: format!("{INVALID_VALUE} \"{text}\""),
            });
            return;
        }
        if text == row.value {
            return;
        }
        self.set_value(index, text);
    }

    /// `Params_CellValueChanged`: the cell's new text, the `_REV` rule, the colour, `_changes`.
    fn set_value(&mut self, index: usize, mut text: String) {
        let Some(row) = self.rows.get_mut(index) else {
            return;
        };
        if row.name.ends_with("_REV")
            && (row.name.starts_with("RC") || row.name.starts_with("HS"))
            && text == "0"
        {
            text = "-1".to_owned();
        }
        row.shade = Shade::Changed;
        let change = mp_log::netfmt::parse_double(&text)
            .map_or_else(|| Change::Text(text.clone()), Change::Number);
        row.value = text;
        self.changes.insert(row.name.clone(), change);
    }

    /// The Fav box clicked: the setting's new value, and the rows sorted again.
    /// `// C#: Controls/DroneCANParams.cs:611-637`
    pub fn click_fav(&mut self, name: &str, setting: Option<&str>) -> Option<String> {
        let row = self.rows.iter_mut().find(|row| row.name == name)?;
        row.fav = !row.fav;
        let ticked = row.fav;
        let saved = crate::raw_params_grid::fav_clicked(setting, name, ticked);
        self.sort();
        saved
    }

    /// A key in the search box: the filter half a second after the last change.
    pub fn search_key(&mut self, event: &KeyDownEvent, now: Instant) -> bool {
        match self.search.key(event) {
            KeyOutcome::Changed => {
                self.search_due = Some(now + Duration::from_millis(500));
                true
            }
            KeyOutcome::Ignored => false,
            _ => true,
        }
    }

    /// "Modified" clicked: the filter at once.
    pub fn toggle_modified(&mut self) {
        self.modified = !self.modified;
        self.filter();
    }

    /// `filterList(txt_search.Text)`.
    /// `// C#: Controls/DroneCANParams.cs:479-521`
    pub fn filter(&mut self) {
        let search = self.search.value().to_owned();
        let count = search.chars().count();
        if count >= 2 || count == 0 {
            let pattern = search.replace('*', ".*").replace("..*", ".*");
            if let Ok(filter) = regex::RegexBuilder::new(&pattern)
                .case_insensitive(true)
                .build()
            {
                for row in &mut self.rows {
                    let cells = [
                        row.name.as_str(),
                        row.value.as_str(),
                        row.min.as_str(),
                        row.max.as_str(),
                        row.default.as_str(),
                        if row.fav { "True" } else { "False" },
                    ];
                    row.visible = cells.iter().any(|cell| filter.is_match(cell));
                }
            }
        }
        if self.modified {
            for row in &mut self.rows {
                row.visible = self.changes.contains_key(&row.name);
            }
        }
    }

    /// Load from file: each of the file's values into the cell of that name where it differs.
    /// `// C#: Controls/DroneCANParams.cs:532-582`
    pub fn load(&mut self, file: &mp_params::param_file::ParamFile) {
        for (name, value) in file.iter() {
            if name == "FORMAT_VERSION" {
                continue;
            }
            let text = mp_log::netfmt::double(value);
            if let Some(index) = self.rows.iter().position(|row| row.name == name)
                && self.rows.get(index).is_some_and(|row| row.value != text)
            {
                self.set_value(index, text);
            }
        }
    }

    /// Save to file: each row's value as a number; "Invalid number entered" and the name for each
    /// that is not.
    /// `// C#: Controls/DroneCANParams.cs:373-404`
    pub fn save(&mut self, path: &Path) -> std::io::Result<()> {
        let mut values = Vec::new();
        for row in &self.rows {
            match mp_log::netfmt::parse_double(&row.value) {
                Some(value) => values.push((row.name.clone(), value)),
                None => self.messages.push_back(Message {
                    title: "",
                    text: format!("Invalid number entered\n {}", row.name),
                }),
            }
        }
        mp_params::param_file::ParamFile::from_values(values).save(path)
    }

    /// The box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }
}

/// `GetIsInRange`: within min and max when both read as numbers, and the text too; anything when
/// they are not numbers.
/// `// C#: Controls/DroneCANParams.cs:206-234`
#[must_use]
pub fn in_range(min: &str, max: &str, text: &str) -> bool {
    let number = |text: &str| text.trim().parse::<f32>().ok();
    match (number(min), number(max)) {
        (Some(min), Some(max)) => number(text).is_some_and(|value| value >= min && value <= max),
        _ => true,
    }
}

/// The Filter window: `defaultfilter` and the boxes.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:897-988`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FilterWindow {
    /// `defaultfilter`, in the order ids were added.
    pub ids: Vec<u16>,
    /// The boxes: name and id, by name ignoring case.
    pub boxes: Vec<(&'static str, u16)>,
    /// "ALL"'s box.
    pub all: bool,
    /// How many `CAN_FILTER_MODIFY`s it has sent.
    pub sent: usize,
}

/// The ids the filter starts with: 0 and what the page and its windows need.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:899-912`
pub const DEFAULT_FILTER: [u16; 11] = [0, 341, 1, 5, 11, 10, 40, 48, 45, 1, 16383];

impl FilterWindow {
    /// The window: every type in `MSG_INFO`, by name ignoring case.
    #[must_use]
    pub fn new() -> Self {
        let mut boxes: Vec<(&'static str, u16)> = names::MSG_INFO
            .iter()
            .map(|row| (row.name, row.id))
            .collect();
        boxes.sort_by_key(|(name, _)| name.to_lowercase());
        Self {
            ids: DEFAULT_FILTER.to_vec(),
            boxes,
            all: false,
            sent: 0,
        }
    }

    /// Whether a type's box is ticked: its id is in the list.
    #[must_use]
    pub fn ticked(&self, id: u16) -> bool {
        self.ids.contains(&id)
    }
}

impl Default for FilterWindow {
    fn default() -> Self {
        Self::new()
    }
}

/// The Stats window's rows: `CanStats` by node and interface, `Stats` by node.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:1392-1471`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct StatsWindow {
    /// The first grid's rows.
    pub can_stats: Vec<(u8, dsdl::CanStats)>,
    /// The second's.
    pub stats: Vec<(u8, dsdl::Stats)>,
}

/// The first grid's columns, `CanStats`' properties in their order.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:1210-1296`
pub const CAN_STATS_COLUMNS: [&str; 12] = [
    "NodeID",
    "interface",
    "tx_requests",
    "tx_rejected",
    "tx_overflow",
    "tx_success",
    "tx_timedout",
    "tx_abort",
    "rx_received",
    "rx_overflow",
    "rx_errors",
    "busoff_errors",
];
/// The second's, `Stats`'.
/// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:1298-1390`
pub const STATS_COLUMNS: [&str; 13] = [
    "NodeID",
    "tx_frames",
    "tx_errors",
    "rx_frames",
    "rx_error_oom",
    "rx_error_internal",
    "rx_error_missed_start",
    "rx_error_wrong_toggle",
    "rx_error_short_frame",
    "rx_error_bad_crc",
    "rx_ignored_wrong_address",
    "rx_ignored_not_wanted",
    "rx_ignored_unexpected_tid",
];

impl StatsWindow {
    /// A `CanStats` or `Stats` heard: the node's row replaced, or added.
    pub fn heard(&mut self, received: &Received) {
        let node = received.frame.source_node();
        match &received.message {
            CanMessage::CanStats(stats) => {
                if let Some(row) = self
                    .can_stats
                    .iter_mut()
                    .find(|(id, held)| *id == node && held.interface == stats.interface)
                {
                    row.1 = *stats;
                } else {
                    self.can_stats.push((node, *stats));
                }
            }
            CanMessage::Stats(stats) => {
                if let Some(row) = self.stats.iter_mut().find(|(id, _)| *id == node) {
                    row.1 = *stats;
                } else {
                    self.stats.push((node, *stats));
                }
            }
            _ => {}
        }
    }

    /// The first grid's cells.
    #[must_use]
    pub fn can_stats_cells(&self) -> Vec<Vec<String>> {
        self.can_stats
            .iter()
            .map(|(node, s)| {
                [
                    u32::from(*node),
                    u32::from(s.interface),
                    s.tx_requests,
                    u32::from(s.tx_rejected),
                    u32::from(s.tx_overflow),
                    u32::from(s.tx_success),
                    u32::from(s.tx_timedout),
                    u32::from(s.tx_abort),
                    s.rx_received,
                    u32::from(s.rx_overflow),
                    u32::from(s.rx_errors),
                    u32::from(s.busoff_errors),
                ]
                .iter()
                .map(ToString::to_string)
                .collect()
            })
            .collect()
    }

    /// The second's.
    #[must_use]
    pub fn stats_cells(&self) -> Vec<Vec<String>> {
        self.stats
            .iter()
            .map(|(node, s)| {
                [
                    u32::from(*node),
                    s.tx_frames,
                    u32::from(s.tx_errors),
                    s.rx_frames,
                    u32::from(s.rx_error_oom),
                    u32::from(s.rx_error_internal),
                    u32::from(s.rx_error_missed_start),
                    u32::from(s.rx_error_wrong_toggle),
                    u32::from(s.rx_error_short_frame),
                    u32::from(s.rx_error_bad_crc),
                    u32::from(s.rx_ignored_wrong_address),
                    u32::from(s.rx_ignored_not_wanted),
                    u32::from(s.rx_ignored_unexpected_tid),
                ]
                .iter()
                .map(ToString::to_string)
                .collect()
            })
            .collect()
    }
}

/// Which window is open.
#[derive(Debug)]
enum OpenWindow {
    Params(Box<ParamsWindow>),
    Filter(FilterWindow),
    Stats(StatsWindow),
}

// ---------------------------------------------------------------------------------------------
// The page object.
// ---------------------------------------------------------------------------------------------

/// The parameter fetch behind "Getting Params".
#[derive(Debug)]
struct Fetch {
    node: u8,
    job: GetParameters,
}

/// The SLCAN start's writes to the autopilot before its port is taken.
#[derive(Debug)]
enum SlcanStep {
    /// `setParam(CAN_SLCAN_CPORT, 1, force)`; the value it had.
    Cport {
        request: mp_link::RequestId,
        made: Instant,
        old: f64,
    },
    /// `CAN_SLCAN_TIMOUT` 2, forced.
    Timeout {
        request: mp_link::RequestId,
        made: Instant,
    },
    /// `CAN_P1_DRIVER` 1.
    Driver {
        request: mp_link::RequestId,
        made: Instant,
    },
}

/// The page object.
#[derive(Debug)]
pub struct DroneCan {
    made_for: Option<Key>,
    active: bool,
    /// `cmb_interfacetype`.
    interface: Combo,
    interface_open: bool,
    /// `cmb_networkinterface`: its interfaces, and the combo.
    networks: Vec<mcast::Interface>,
    network: Combo,
    network_open: bool,
    /// `cmb_networkinterface.Visible`.
    network_visible: bool,
    /// `cmb_*.Enabled`.
    lists_enabled: bool,
    /// `isConnected`.
    connected: bool,
    /// `but_connect.Text`.
    connect_text: &'static str,
    /// `but_filter.Enabled`.
    filter_enabled: bool,
    /// `CHK_checkupdate.Checked`.
    check_update: bool,
    /// `chk_log.Checked`.
    log: bool,
    /// `chk_canonclose.Checked`.
    exit_slcan: bool,
    /// `allnodes`.
    nodes: Vec<NodeRow>,
    /// The grid's current row.
    current: Option<usize>,
    /// `DGDebug`'s rows, newest first.
    debug: VecDeque<DebugRow>,
    /// The node menu: its row, and where it was opened, in the window.
    menu: Option<(usize, (f32, f32))>,
    /// `can`: the node, made with the page and again after a `Disconnect`.
    can: Option<Node>,
    /// `_paramlistcache`: every parameter list read, which `SetParameter` takes types from.
    param_cache: Vec<GetSetRes>,
    /// The bus.
    bus: Option<Bus>,
    /// `BusInUse`.
    bus_in_use: u8,
    /// `mavlinkCANRun`.
    can_run: bool,
    /// The log file, `can.LogFile`.
    log_file: Option<mp_os::fs::File>,
    /// The manifest an update's search loaded.
    manifest: Option<Arc<mp_firmware::manifest::Manifest>>,
    /// The SLCAN start's writes, in turn.
    slcan_step: Option<SlcanStep>,
    /// "New firmware"'s "Show me again?" turned off: `SHOWAGAIN_New_firmware` "False".
    new_firmware_suppressed: bool,
    /// The URL whose port the holder is to take for SLCAN.
    take_port: Option<String>,
    /// The parameter fetch.
    fetch: Option<Fetch>,
    /// `RestartNode`s waiting.
    restarts: Vec<Request>,
    /// The firmware update.
    update: Option<UpdateFlow>,
    /// The passthrough's listener: `listener`.
    passthrough: Option<Passthrough>,
    /// The window open.
    window: Option<OpenWindow>,
    /// The Inspector, while it is open, and how many times it has been opened
    /// (`config/dronecan_inspector.rs`).
    pub inspector: Option<super::dronecan_inspector::Inspector>,
    pub inspector_opened: usize,
    /// Boxes, oldest first.
    messages: VecDeque<Message>,
    /// The question showing.
    question: Option<Question>,
    /// The typed path showing.
    path: Option<(PathFor, crate::config::firmware::PathBox)>,
    /// The `InputBox` showing.
    input: Option<Input>,
    /// The progress window.
    progress: Option<Progress>,
    /// The keyboard focus of the boxes typed into, made on first use.
    focus: Option<FocusHandle>,
    /// What goes on the status line next.
    status: Option<String>,
    /// The app's version, for our node's `GetNodeInfo` answer.
    identity: Identity,
}

impl Default for DroneCan {
    /// `InitializeComponent`: and `can`, a field initialiser.
    fn default() -> Self {
        let mut interface = Combo {
            options: INTERFACES
                .iter()
                .enumerate()
                .map(|(index, name)| (i64::try_from(index).unwrap_or(0), (*name).to_owned()))
                .collect(),
            ..Combo::default()
        };
        interface.enabled = true;
        let identity = Identity::from_version(env!("CARGO_PKG_VERSION"));
        Self {
            made_for: None,
            active: false,
            interface,
            interface_open: false,
            networks: Vec::new(),
            network: Combo::default(),
            network_open: false,
            network_visible: false,
            lists_enabled: true,
            connected: false,
            connect_text: CONNECT,
            filter_enabled: true,
            check_update: false,
            log: false,
            exit_slcan: true,
            nodes: Vec::new(),
            current: None,
            debug: VecDeque::new(),
            menu: None,
            can: Some(Node::new(identity, Instant::now())),
            param_cache: Vec::new(),
            bus: None,
            bus_in_use: 0,
            can_run: false,
            log_file: None,
            manifest: None,
            slcan_step: None,
            new_firmware_suppressed: false,
            take_port: None,
            fetch: None,
            restarts: Vec::new(),
            update: None,
            passthrough: None,
            window: None,
            inspector: None,
            inspector_opened: 0,
            messages: VecDeque::new(),
            question: None,
            path: None,
            input: None,
            progress: None,
            focus: None,
            status: None,
            identity,
        }
    }
}

impl DroneCan {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// The interface chosen.
    #[must_use]
    pub fn interface(&self) -> ConnectionType {
        self.interface
            .selected
            .and_then(|index| usize::try_from(index).ok())
            .and_then(ConnectionType::from_index)
            .unwrap_or(ConnectionType::Slcan)
    }

    /// `allnodes`.
    #[must_use]
    pub fn nodes(&self) -> &[NodeRow] {
        &self.nodes
    }

    /// The current row's node.
    #[must_use]
    pub fn current(&self) -> Option<&NodeRow> {
        self.current.and_then(|index| self.nodes.get(index))
    }

    /// `DGDebug`'s rows.
    pub fn debug(&self) -> impl Iterator<Item = &DebugRow> {
        self.debug.iter()
    }

    /// `but_connect.Text`.
    #[must_use]
    pub const fn connect_text(&self) -> &'static str {
        self.connect_text
    }

    /// The box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// Dismisses it.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// What goes on the status line, once.
    pub fn take_status(&mut self) -> Option<String> {
        self.status.take()
    }

    /// The URL whose port is to be taken for SLCAN, once.
    pub fn take_port_request(&mut self) -> Option<String> {
        self.take_port.take()
    }

    /// The parameter window, when open.
    #[must_use]
    pub fn params_window(&self) -> Option<&ParamsWindow> {
        match &self.window {
            Some(OpenWindow::Params(window)) => Some(window),
            _ => None,
        }
    }

    /// The parameter window, to change.
    pub fn params_window_mut(&mut self) -> Option<&mut ParamsWindow> {
        match &mut self.window {
            Some(OpenWindow::Params(window)) => Some(window),
            _ => None,
        }
    }

    /// The node, while there is one.
    #[must_use]
    pub const fn node(&self) -> Option<&Node> {
        self.can.as_ref()
    }

    /// `Activate`: the timer, the interface list bound again (so SLCAN), the network
    /// interfaces listed, and "Check for Updates" from the settings.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:39-57`
    pub fn activate(
        &mut self,
        key: Key,
        update_check: Option<&str>,
        new_firmware_again: Option<&str>,
    ) {
        if self.made_for != Some(key) {
            *self = Self {
                made_for: Some(key),
                ..Self::default()
            };
        }
        self.active = true;
        // `MessageShowAgain`'s early return: the key there and not `GetBoolean`'s true.
        self.new_firmware_suppressed =
            new_firmware_again.is_some_and(|value| !crate::raw_params::get_boolean(Some(value)));
        // `DataSource = Enum.GetValues(...)`: the first row, and its `SelectedIndexChanged`.
        self.interface.selected = Some(0);
        self.network_visible = ConnectionType::Slcan.shows_networks();
        self.networks = mcast::interfaces();
        self.network = Combo {
            param: String::new(),
            options: self
                .networks
                .iter()
                .enumerate()
                .map(|(index, interface)| {
                    (
                        i64::try_from(index).unwrap_or(0),
                        interface.description.clone(),
                    )
                })
                .collect(),
            selected: (!self.networks.is_empty()).then_some(0),
            enabled: self.lists_enabled,
            top_index: 0,
        };
        // `SetCheckboxFromConfig`: only when the key is there.
        if let Some(value) = update_check {
            let checked = crate::raw_params::get_boolean(Some(value));
            if checked != self.check_update {
                self.check_update = checked;
                self.check_for_updates();
            }
        }
    }

    /// `Deactivate`: `Disconnect`, and the timer stopped.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:680-684`
    pub fn deactivate(&mut self) {
        self.disconnect();
        self.active = false;
        self.menu = None;
        self.interface_open = false;
        self.network_open = false;
    }

    /// `Disconnect`: the loops stopped, the passthrough's listener, the node - its adapter told to
    /// close when "Exit SLCAN on leave?" is ticked - and the lists enabled again.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:686-706`
    pub fn disconnect(&mut self) {
        self.can_run = false;
        self.stop_passthrough();
        if let Some(mut node) = self.can.take() {
            let close = node.stop(self.exit_slcan).is_some();
            self.stop_bus(close);
        } else {
            self.stop_bus(self.exit_slcan);
        }
        self.slcan_step = None;
        self.log_file = None;
        // The waits on the node that has gone: the C#'s threads go on against a stopped object
        // and end in an exception or a timeout; here they end now, with their windows.
        self.fetch = None;
        self.restarts.clear();
        self.update = None;
        if self
            .progress
            .as_ref()
            .is_some_and(|progress| progress.for_ != ProgressFor::Slcan)
        {
            self.progress = None;
        }
        self.lists_enabled = true;
        self.interface.enabled = true;
        self.network.enabled = true;
        self.connect_text = CONNECT;
        self.connected = false;
    }

    /// The bus stopped: the subscription dropped, the adapter's thread told to stop, the
    /// multicast loop ended.
    fn stop_bus(&mut self, close: bool) {
        match self.bus.take() {
            Some(Bus::Slcan(bus)) => {
                let _ = bus.commands.send(SlcanCommand::Stop(close));
            }
            Some(Bus::Mcast(bus)) => bus.run.store(false, Ordering::Release),
            Some(Bus::Mavlink(_)) | None => {}
        }
    }

    /// The passthrough's listener stopped.
    fn stop_passthrough(&mut self) {
        if let Some(passthrough) = self.passthrough.take() {
            passthrough.stop.store(true, Ordering::Release);
        }
    }

    /// An interface chosen from the list: `SelectedIndexChanged`.
    pub fn choose_interface(&mut self, index: i64) {
        self.interface_open = false;
        if self.interface.select(index) {
            self.network_visible = self.interface().shows_networks();
        }
    }

    /// A network interface chosen.
    pub fn choose_network(&mut self, index: i64) {
        self.network_open = false;
        self.network.select(index);
    }

    /// The interface list opened or closed.
    pub fn toggle_interface_list(&mut self) {
        if self.lists_enabled {
            self.interface_open = !self.interface_open;
            self.network_open = false;
            if self.interface_open {
                self.interface.open_list();
            }
        }
    }

    /// The network list opened or closed.
    pub fn toggle_network_list(&mut self) {
        if self.lists_enabled {
            self.network_open = !self.network_open;
            self.interface_open = false;
            if self.network_open {
                self.network.open_list();
            }
        }
    }

    /// The node, made again if a `Disconnect` let it go: `if (can == null) can = new
    /// DroneCAN()`, `SourceNode = 127`.
    fn node_for_start(&mut self, now: Instant) -> &mut Node {
        if self.can.is_none() {
            self.param_cache.clear();
        }
        self.can
            .get_or_insert_with(|| Node::new(self.identity, now))
    }

    /// `but_connect_Click`. `connected_link` is whether the link is open; `url` its URL, for
    /// SLCAN to take.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:1635-1681`
    pub fn click_connect(&mut self, view: &TelemetryView, telemetry: &Telemetry, now: Instant) {
        self.menu = None;
        if self.connected {
            self.disconnect();
            return;
        }
        self.can_run = false;
        // "Reset the table of nodes": `Rows.Clear()` on the bound grid clears `allnodes`.
        self.nodes.clear();
        self.current = None;
        self.lists_enabled = false;
        self.interface.enabled = false;
        self.network.enabled = false;
        self.interface_open = false;
        self.network_open = false;
        self.connect_text = DISCONNECT;
        self.connected = true;
        match self.interface() {
            ConnectionType::Slcan => self.start_slcan(view, telemetry, now),
            ConnectionType::MavlinkCan(bus) => self.start_mavlink(bus, now),
            ConnectionType::McastCan(bus) => {
                let interface = self
                    .network
                    .selected
                    .and_then(|index| usize::try_from(index).ok())
                    .and_then(|index| self.networks.get(index))
                    .cloned();
                self.start_mcast(bus, interface, now);
            }
        }
    }

    /// `StartMavlinkCAN(bus)`: Filter enabled, the forwarding loop a second from now, the node
    /// started.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:88-202`
    fn start_mavlink(&mut self, bus: u8, now: Instant) {
        self.bus_in_use = bus;
        self.filter_enabled = true;
        self.bus = Some(Bus::Mavlink(MavlinkBus {
            bus,
            next_forward: now + Duration::from_secs(1),
            subscription: None,
            heard: Arc::new(Mutex::new(VecDeque::new())),
            forwards: 0,
            sent: 0,
            received: 0,
        }));
        self.setup_node(now, true);
    }

    /// `SetupSLCanPort` past the port's opening: the log file, `StartSLCAN`'s start of the node,
    /// the file server and the allocator.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:267-362`
    fn setup_node(&mut self, now: Instant, start: bool) {
        let opening = !start || !matches!(self.bus, Some(Bus::Slcan(_)));
        if self.log && opening {
            self.log_file = log_directory().and_then(|dir| {
                let _ = mp_os::fs::create_dir_all(&dir);
                let name = format!("{}.can", chrono::Local::now().format("%Y-%m-%d %H-%M-%S"));
                mp_os::fs::File::create(dir.join(name)).ok()
            });
        }
        let node = self.node_for_start(now);
        node.set_source_node(mp_dronecan::node::SOURCE_NODE);
        if start {
            node.start(now);
            node.setup_file_server();
            node.setup_dynamic_node_allocator();
        }
    }

    /// `startslcan(1)`: with the link open, the writes to the autopilot first; with none, the
    /// question.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:204-265`
    fn start_slcan(&mut self, view: &TelemetryView, telemetry: &Telemetry, now: Instant) {
        self.filter_enabled = false;
        let link_open = crate::fly::port_open(view);
        if !link_open {
            self.question = Some(Question {
                caption: SLCAN_CAPTION,
                text: NOT_CONNECTED_SLCAN.to_owned(),
                asked: Asked::SlcanWithoutLink,
            });
            return;
        }
        // `MAV.param["CAN_SLCAN_CPORT"]`: a vehicle without it throws, and the `catch` goes
        // straight to taking the port.
        let Some(old) = super::servo_output::value_of(&view.parameters, "CAN_SLCAN_CPORT") else {
            self.take_port = Some(view.target.clone());
            return;
        };
        match telemetry.write_parameter("CAN_SLCAN_CPORT", 1.0, true) {
            Some(request) => {
                self.slcan_step = Some(SlcanStep::Cport {
                    request,
                    made: now,
                    old,
                });
            }
            None => self.take_port = Some(view.target.clone()),
        }
    }

    /// The SLCAN start's writes, one after another, as the C#'s blocking `setParam`s run.
    fn tick_slcan_start(&mut self, telemetry: &Telemetry, view: &TelemetryView, now: Instant) {
        let Some(step) = self.slcan_step.take() else {
            return;
        };
        let (request, made) = match &step {
            SlcanStep::Cport { request, made, .. }
            | SlcanStep::Timeout { request, made }
            | SlcanStep::Driver { request, made } => (*request, *made),
        };
        let outcome = match telemetry.lookup(request, made) {
            Lookup::Found(found) => match found.outcome() {
                Some(outcome) => outcome,
                None => {
                    self.slcan_step = Some(step);
                    return;
                }
            },
            Lookup::PickingUp => {
                self.slcan_step = Some(step);
                return;
            }
            Lookup::Gone => RequestOutcome::TimedOut,
        };
        // A timeout throws out of `setParam`; the `catch` goes on to take the port.
        if outcome == RequestOutcome::TimedOut {
            self.take_port = Some(view.target.clone());
            return;
        }
        match step {
            SlcanStep::Cport { old, .. } => {
                if old == 0.0 {
                    self.messages.push_back(Message {
                        title: ERROR_TITLE,
                        text: REBOOT_REQUIRED.to_owned(),
                    });
                    return;
                }
                match telemetry.write_parameter("CAN_SLCAN_TIMOUT", 2.0, true) {
                    Some(request) => {
                        self.slcan_step = Some(SlcanStep::Timeout { request, made: now });
                    }
                    None => self.take_port = Some(view.target.clone()),
                }
            }
            SlcanStep::Timeout { .. } => {
                match telemetry.write_parameter("CAN_P1_DRIVER", 1.0, false) {
                    Some(request) => {
                        self.slcan_step = Some(SlcanStep::Driver { request, made: now })
                    }
                    None => self.take_port = Some(view.target.clone()),
                }
            }
            SlcanStep::Driver { .. } => {
                // The blind `CAN_SLCAN_SERNUM` 0, twice: a vehicle without it throws on its type
                // and sends nothing.
                if super::servo_output::value_of(&view.parameters, "CAN_SLCAN_SERNUM").is_some()
                    && let Some(vehicle) = view.vehicle
                {
                    let set = MavMessage::ParamSet(ParamSet {
                        param_value: 0.0,
                        target_system: vehicle.sysid,
                        target_component: vehicle.compid,
                        param_id: mp_params::encode_param_id("CAN_SLCAN_SERNUM"),
                        // `param_types[name]`: the view carries no types; ArduPilot declares it
                        // AP_Int8, and reads its own type, not this field.
                        param_type: 2,
                    });
                    telemetry.send(&set);
                    telemetry.send(&set);
                }
                self.take_port = Some(view.target.clone());
            }
        }
    }

    /// The port taken, or the connection box's opened: the adapter's thread, "Trying to
    /// connect" while it opens.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:252-358`
    pub fn open_slcan(&mut self, url: &str, now: Instant) {
        let transport = match mp_transport::open(url) {
            Ok(transport) => transport,
            Err(_) => {
                // `port.Open()` threw: `Strings.CheckPortSettingsOr`, a status line here.
                self.status = Some(CHECK_PORT.to_owned());
                return;
            }
        };
        self.setup_node(now, false);
        let (commands, command_rx) = mpsc::channel();
        let (event_tx, events) = mpsc::channel();
        let spawned = wasm_thread::Builder::new()
            .name("dronecan-slcan".to_owned())
            .spawn(move || run_slcan(transport, &command_rx, &event_tx));
        if spawned.is_err() {
            self.status = Some(CHECK_PORT.to_owned());
            return;
        }
        self.bus = Some(Bus::Slcan(SlcanBus {
            commands,
            events,
            opened: false,
        }));
        self.progress = Some(Progress {
            text: TRYING_TO_CONNECT.to_owned(),
            bar: crate::config::serial_ports::Bar::default(),
            started: now,
            for_: ProgressFor::Slcan,
        });
    }

    /// `StartmcastCAN(bus, interface)`.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:1509-1633`
    fn start_mcast(&mut self, bus: u8, interface: Option<mcast::Interface>, now: Instant) {
        let Some(interface) = interface else {
            self.status = Some(NO_INTERFACES.to_owned());
            return;
        };
        self.bus_in_use = bus;
        self.filter_enabled = false;
        let socket = match mcast::open(bus, &interface) {
            Ok(socket) => Arc::new(socket),
            Err(error) => {
                // The socket calls throw out of the click handler.
                self.status = Some(error.to_string());
                return;
            }
        };
        let run = Arc::new(AtomicBool::new(true));
        let (lines_tx, lines) = mpsc::channel();
        let reader = Arc::clone(&socket);
        let running = Arc::clone(&run);
        let spawned = wasm_thread::Builder::new()
            .name("dronecan-mcast".to_owned())
            .spawn(move || {
                let mut buf = [0u8; 1500];
                while running.load(Ordering::Acquire) {
                    // The one-second timeout, or any other error: go round.
                    if let Ok((n, _)) = reader.recv_from(&mut buf)
                        && let Some(line) = mcast::line_of(buf.get(..n).unwrap_or(&[]))
                        && lines_tx.send(line).is_err()
                    {
                        return;
                    }
                }
            });
        if spawned.is_err() {
            return;
        }
        self.can_run = true;
        self.bus = Some(Bus::Mcast(McastBus {
            bus,
            socket,
            lines,
            run,
        }));
        self.setup_node(now, true);
    }

    /// The SLCAN question answered: OK opens the connection box's port; Cancel leaves the page
    /// connected to nothing, as the C#'s `return` does.
    fn slcan_without_link(&mut self, url: Option<String>, now: Instant) {
        match url {
            Some(url) => self.open_slcan(&url, now),
            None => self.status = Some(CHECK_PORT.to_owned()),
        }
    }

    // --- The grid ---------------------------------------------------------------------------

    /// A row clicked: the current row.
    pub fn select_row(&mut self, index: usize) {
        if index < self.nodes.len() {
            self.current = Some(index);
        }
        self.menu = None;
    }

    /// `myDataGridView1_CellClick` on the Menu column: the row current, its menu open where
    /// clicked.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:486-502`
    pub fn click_menu_cell(&mut self, index: usize, at: (f32, f32)) {
        if index >= self.nodes.len() {
            return;
        }
        self.current = Some(index);
        self.menu = if self.menu.is_some() {
            None
        } else {
            Some((index, at))
        };
    }

    /// A right click on the grid: its `ContextMenu`, for the current row.
    pub fn right_click(&mut self, at: (f32, f32)) {
        self.menu = self.current.map(|index| (index, at));
    }

    /// The current row's node: `byte.Parse(CurrentRow.Cells[ID].Value)`.
    fn current_node(&self) -> Option<u8> {
        self.current().map(|row| row.id)
    }

    /// `menu_parameters_Click`: `GetParameters` behind "Getting Params".
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:665-678, 873-877`
    pub fn menu_parameters(&mut self, now: Instant) {
        self.menu = None;
        let Some(node) = self.current_node() else {
            return;
        };
        if self.can.is_none() {
            return;
        }
        self.fetch = Some(Fetch {
            node,
            job: GetParameters::new(node, now),
        });
        self.progress = Some(Progress {
            text: GETTING_PARAMS.to_owned(),
            bar: crate::config::serial_ports::Bar::default(),
            started: now,
            for_: ProgressFor::Parameters,
        });
    }

    /// `menu_restart_Click`: `RestartNode`, its answer not looked at.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:879-883`
    pub fn menu_restart(&mut self) {
        self.menu = None;
        if let Some(node) = self.current_node()
            && self.can.is_some()
        {
            self.restarts.push(Request::restart_node(node));
        }
    }

    /// `menu_update_Click` and `menu_updatebeta_Click`: `FirmwareUpdate`'s question - once the
    /// node has given its info, which `hwversion` is read from (the C# throws without it).
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:504-519, 867-871, 885-889`
    pub fn menu_update(&mut self, beta: bool) {
        self.menu = None;
        let Some(node) = self.current_node() else {
            return;
        };
        let has_info = self
            .can
            .as_ref()
            .is_some_and(|can| can.node_info().contains_key(&node));
        if !has_info {
            // `can.NodeInfo[nodeID]` throws: the C#'s unhandled exception, a status line here.
            self.status = Some("The given key was not present in the dictionary.".to_owned());
            return;
        }
        self.question = Some(Question {
            caption: UPDATE_CAPTION,
            text: SEARCH_INTERNET.to_owned(),
            asked: Asked::SearchInternet { node, beta },
        });
    }

    /// `FirmwareUpdate`'s Yes: the search and download on a thread - `LookForUpdate`, else the
    /// manifest's peripheral firmware for the node's platform - then the update.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:518-598`
    fn search_internet(&mut self, node: u8, beta: bool, now: Instant) {
        let Some(can) = self.can.as_ref() else {
            return;
        };
        let Some(info) = can.node_info().get(&node) else {
            return;
        };
        let device = can.node_name(node);
        let hwversion = hw_version_text(info);
        let manifest = self.manifest.clone();
        let (results_tx, results) = mpsc::channel();
        let spawned = wasm_thread::Builder::new()
            .name("dronecan-update".to_owned())
            .spawn(move || {
                let _ = results_tx.send(fetch_firmware(&device, &hwversion, beta, manifest));
            });
        if spawned.is_err() {
            return;
        }
        self.update = Some(UpdateFlow::Fetching {
            node,
            results,
            cancelled: false,
        });
        self.progress = Some(Progress {
            text: DOWNLOAD_FW.to_owned(),
            bar: crate::config::serial_ports::Bar {
                marquee: false,
                value: 5,
            },
            started: now,
            for_: ProgressFor::Update,
        });
    }

    /// A firmware file chosen: an `.apj`'s image to a temporary file first, then the update.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:600-660`
    fn update_from_file(&mut self, node: u8, file: &Path, now: Instant) {
        let image = if file
            .extension()
            .is_some_and(|ext| ext.eq_ignore_ascii_case("apj"))
        {
            match mp_firmware::firmware::Firmware::load(file) {
                Ok(firmware) => {
                    let temporary = mp_os::temp_dir().join(format!(
                        "dronecan-{}-{}.bin",
                        mp_os::process_id(),
                        node
                    ));
                    if mp_os::fs::write(&temporary, &firmware.image).is_err() {
                        return;
                    }
                    temporary
                }
                Err(error) => {
                    self.status = Some(error.to_string());
                    return;
                }
            }
        } else {
            file.to_path_buf()
        };
        self.start_update(node, &image, now);
    }

    /// `can.Update(...)` behind the progress window.
    fn start_update(&mut self, node: u8, firmware: &Path, now: Instant) {
        let Some(can) = self.can.as_mut() else {
            self.progress = None;
            return;
        };
        match Update::new(node, firmware, can, now) {
            Ok(update) => {
                self.update = Some(UpdateFlow::Running(update));
                self.progress = Some(Progress {
                    text: String::new(),
                    bar: crate::config::serial_ports::Bar::default(),
                    started: now,
                    for_: ProgressFor::Update,
                });
            }
            Err(error) => {
                self.progress = None;
                self.status = Some(error.to_string());
            }
        }
    }

    /// `menu_passthrough_Click`: a second click stops the listener; else the port asked for.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:740-754`
    pub fn menu_passthrough(&mut self) {
        self.menu = None;
        if self.passthrough.is_some() {
            self.stop_passthrough();
            self.messages.push_back(Message {
                title: DISABLED_FORWARDING,
                text: STOP.to_owned(),
            });
            return;
        }
        self.ask(ENTER_TCP_PORT, "500", InputFor::Here3Port);
    }

    /// `menu_passthrough4_Click`: a second click stops the listener; else the port and the baud
    /// rate asked for. `control` held is `portlock` false.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:990-1017`
    pub fn menu_passthrough4(&mut self, control: bool) {
        self.menu = None;
        if self.passthrough.is_some() {
            self.stop_passthrough();
            self.messages.push_back(Message {
                title: DISABLED_FORWARDING,
                text: STOP.to_owned(),
            });
            return;
        }
        let Some(node) = self.current_node() else {
            return;
        };
        self.ask(
            ENTER_TCP_PORT,
            "500",
            InputFor::Here4Port {
                node,
                lock: !control,
            },
        );
    }

    /// An `InputBox` up.
    fn ask(&mut self, title: &'static str, value: &str, for_: InputFor) {
        let mut field = TextField::new("");
        field.set(value);
        self.input = Some(Input { title, field, for_ });
    }

    /// An `InputBox` answered: its number, and the next step.
    pub fn answer_input(&mut self, ok: bool, now: Instant) {
        let Some(input) = self.input.take() else {
            return;
        };
        if !ok {
            return;
        }
        // `InputBox.Show(..., ref int)`: a value that is not a number leaves it as it was.
        let value = |default: u32| input.field.value().trim().parse::<u32>().unwrap_or(default);
        match input.for_ {
            InputFor::Here3Port => {
                let port = u16::try_from(value(500)).unwrap_or(500);
                self.start_passthrough(port, None, false, now);
            }
            InputFor::Here4Port { node, lock } => {
                let port = u16::try_from(value(500)).unwrap_or(500);
                self.ask(
                    ENTER_BAUD,
                    "230400",
                    InputFor::Here4Baud { node, lock, port },
                );
            }
            InputFor::Here4Baud { node, lock, port } => {
                let baud = value(230_400);
                self.start_passthrough(port, Some((node, baud)), lock, now);
            }
        }
    }

    /// The listener started, on a thread: `menu_passthrough*.Checked`.
    fn start_passthrough(
        &mut self,
        port: u16,
        tunnel: Option<(u8, u32)>,
        lock: bool,
        now: Instant,
    ) {
        let listener = match TcpListener::bind(("0.0.0.0", port)) {
            Ok(listener) => listener,
            Err(error) => {
                // "Forwarder problem": a status line here.
                self.status = Some(format!("Forwarder problem {error}"));
                return;
            }
        };
        let stop = Arc::new(AtomicBool::new(false));
        let (to_client, client_rx) = mpsc::channel();
        let (events_tx, from_client) = mpsc::channel();
        let chunk = if tunnel.is_some() { 120 } else { 128 };
        let stopping = Arc::clone(&stop);
        let spawned = wasm_thread::Builder::new()
            .name("dronecan-passthrough".to_owned())
            .spawn(move || run_passthrough(&listener, chunk, &stopping, &client_rx, &events_tx));
        if spawned.is_err() {
            return;
        }
        self.passthrough = Some(Passthrough {
            tunnel,
            to_client,
            from_client,
            stop,
            client: false,
            last_send: now,
            bauds: if tunnel.is_some() {
                HERE4_BAUDS.into_iter().collect()
            } else {
                VecDeque::new()
            },
            next_baud: now,
            lock,
        });
    }

    // --- The buttons ------------------------------------------------------------------------

    /// `but_filter_Click`: the window.
    pub fn click_filter(&mut self) {
        self.menu = None;
        if self.filter_enabled {
            self.window = Some(OpenWindow::Filter(FilterWindow::new()));
        }
    }

    /// A box of the Filter window changed - `None` is "ALL" - and the `CAN_FILTER_MODIFY` it
    /// sends while the MAVLink bus runs.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:924-983`
    pub fn click_filter_box(
        &mut self,
        id: Option<u16>,
        telemetry: &Telemetry,
        view: &TelemetryView,
    ) {
        let Some(OpenWindow::Filter(filter)) = self.window.as_mut() else {
            return;
        };
        let num_ids = match id {
            None => {
                filter.all = !filter.all;
                0
            }
            Some(id) => {
                if filter.ids.contains(&id) {
                    filter.ids.retain(|held| *held != id);
                } else {
                    filter.ids.push(id);
                }
                u8::try_from(filter.ids.len()).unwrap_or(u8::MAX)
            }
        };
        if self.can_run
            && let Some(vehicle) = view.vehicle
        {
            let message = mavlink::filter_modify(
                &filter.ids,
                vehicle.sysid,
                vehicle.compid,
                self.bus_in_use,
                num_ids,
            );
            if telemetry.send(&message) {
                filter.sent += 1;
            }
        }
    }

    /// `but_stats_Click`: the window, subscribed while open. With no node the C# throws.
    pub fn click_stats(&mut self) {
        self.menu = None;
        if self.can.is_some() {
            self.window = Some(OpenWindow::Stats(StatsWindow::default()));
        }
    }

    /// `But_uavcaninspector_Click`: "Please connect first" with no node, else `new
    /// DroneCANInspector(can).Show()`, the window fed what the node hears from here on.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:714-723`
    pub fn click_inspector(&mut self) {
        self.menu = None;
        // `can == null`: the C# makes its node on Connect; here the node is made with the page
        // and the bus attached on Connect, so "connected" is the test.
        if !self.connected {
            self.status = Some(PLEASE_CONNECT.to_owned());
            return;
        }
        self.inspector = Some(super::dronecan_inspector::Inspector::new(Instant::now()));
        self.inspector_opened += 1;
    }

    /// The Inspector closed: `FormClosing`, its handler and timer ended.
    /// `// C#: Controls/DroneCANInspector.cs:289-294`
    pub fn close_inspector(&mut self) {
        self.inspector = None;
    }

    /// The window closed.
    pub fn close_window(&mut self) {
        self.window = None;
    }

    /// `CHK_checkupdate_CheckedChanged`.
    pub fn toggle_check_update(&mut self) {
        self.check_update = !self.check_update;
        self.check_for_updates();
    }

    /// "Check for Updates" ticked: each node with info checked, when there is a manifest.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:1700-1722`
    fn check_for_updates(&mut self) {
        if !self.check_update {
            return;
        }
        let Some(can) = self.can.as_ref() else {
            return;
        };
        let infos: Vec<(u8, GetNodeInfoRes)> = can
            .node_info()
            .iter()
            .map(|(id, info)| (*id, info.clone()))
            .collect();
        for (id, info) in infos {
            self.check_single(id, &info);
        }
    }

    /// `CheckSingleIDForUpdate`: the manifest's official peripheral firmware for the node's
    /// platform at its version or newer, with another git hash.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:1724-1751`
    fn check_single(&mut self, id: u8, info: &GetNodeInfoRes) {
        let Some(manifest) = self.manifest.as_ref() else {
            return;
        };
        let Some(can) = self.can.as_ref() else {
            return;
        };
        let device = can.node_name(id);
        let githash = format!("{:X}", info.software_version.vcs_commit);
        let version = &info.software_version;
        let option = manifest.firmware.iter().find(|a| {
            a.mav_firmware_version_type.as_deref() == Some("OFFICIAL")
                && a.vehicle_type.as_deref() == Some("AP_Periph")
                && matches!(a.format.as_deref(), Some("bin" | "zip"))
                && a.mav_type.as_deref() == Some("CAN_PERIPHERAL")
                && a.mav_firmware_version_major >= i64::from(version.major)
                && a.mav_firmware_version_minor >= i64::from(version.minor)
                && version.major != 0
                && version.minor != 0
                && a.platform
                    .as_deref()
                    .is_some_and(|platform| device.ends_with(platform))
                && !a
                    .git_sha
                    .as_deref()
                    .is_some_and(|sha| sha.to_lowercase().starts_with(&githash.to_lowercase()))
        });
        if let Some(option) = option
            && !self.new_firmware_suppressed
        {
            let text = format!(
                "New firmware for {device} {} {}\nUpdate bellow",
                option
                    .mav_firmware_version
                    .as_ref()
                    .map(ToString::to_string)
                    .unwrap_or_default(),
                option.git_sha.as_deref().unwrap_or("")
            );
            self.question = Some(Question {
                caption: NEW_FIRMWARE,
                text,
                asked: Asked::NewFirmware { again: true },
            });
        }
    }

    /// "Log" clicked.
    pub fn toggle_log(&mut self) {
        self.log = !self.log;
    }

    /// "Exit SLCAN on leave?" clicked.
    pub fn toggle_exit_slcan(&mut self) {
        self.exit_slcan = !self.exit_slcan;
    }

    // --- The node's messages ----------------------------------------------------------------

    /// The page's `MessageReceived` handler.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:364-476`
    fn page_handler(&mut self, received: &Received, node: &mut Node) {
        let source = received.frame.source_node();
        match &received.message {
            CanMessage::NodeStatus(status) => {
                let unnamed = self
                    .nodes
                    .iter()
                    .find(|row| row.id == source)
                    .is_some_and(|row| row.name == "?");
                if unnamed {
                    let id = node.next_transfer_id();
                    node.send(source, PRIORITY, id, CanMessage::GetNodeInfoReq);
                }
                for row in self.nodes.iter_mut().filter(|row| row.id == source) {
                    if let Some(health) = health_text(status.health) {
                        health.clone_into(&mut row.health);
                    }
                    if let Some(mode) = mode_text(status.mode) {
                        mode.clone_into(&mut row.mode);
                    }
                    row.uptime = status.uptime_sec;
                }
            }
            CanMessage::GetNodeInfoRes(info) => {
                for row in self.nodes.iter_mut().filter(|row| row.id == source) {
                    row.name = ascii(&info.name);
                    row.hardware_version = Some(hw_version_text(info));
                    let software = &info.software_version;
                    row.software_version = Some(format!(
                        "{}.{}.{:X}",
                        software.major, software.minor, software.vcs_commit
                    ));
                    row.software_crc = software.image_crc;
                    row.hardware_uid = Some(
                        info.hardware_version
                            .unique_id
                            .iter()
                            .map(|byte| format!("{byte:02X}"))
                            .collect::<Vec<_>>()
                            .join(" "),
                    );
                    row.vsc = info.status.vendor_specific_status_code;
                }
            }
            CanMessage::LogMessage(log) => {
                self.debug.push_front(DebugRow {
                    node: source,
                    level: log.level,
                    source: ascii(&log.source),
                    text: ascii(&log.text),
                });
                while self.debug.len() > DEBUG_ROWS {
                    self.debug.pop_back();
                }
            }
            _ => {}
        }
    }

    /// The node's other events, which the C# posts to the window's thread.
    fn node_event(&mut self, event: &Event) {
        match event {
            // `NodeAdded`: the row with the status's numbers.
            // `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:301-317`
            Event::NodeAdded(id, status) => self.nodes.push(NodeRow {
                id: *id,
                name: "?".to_owned(),
                health: status.health.to_string(),
                mode: status.mode.to_string(),
                uptime: status.uptime_sec,
                hardware_version: None,
                software_version: None,
                software_crc: 0,
                hardware_uid: None,
                vsc: status.vendor_specific_status_code,
            }),
            // `NodeInfoAdded`: checked for an update while ticked.
            // `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:319-328`
            Event::NodeInfoAdded(id, info) => {
                if self.check_update {
                    self.check_single(*id, info);
                }
            }
            Event::FileSendProgress {
                node,
                file,
                percent,
            } => {
                if let Some(progress) = self.progress.as_mut()
                    && progress.for_ == ProgressFor::Update
                {
                    // `UpdateProgressAndStatus((int)percent, id + " " + file)`, then the job's
                    // own `(p, f)`.
                    #[allow(clippy::cast_possible_truncation)] // a percentage
                    let value = *percent as i32;
                    progress.bar = crate::config::serial_ports::Bar {
                        marquee: false,
                        value,
                    };
                    // The job's own `(n, f, p) => UpdateProgressAndStatus((int)p, f)`, subscribed
                    // after the window's `id + " " + file`, is the one left showing.
                    let _ = node;
                    progress.text.clone_from(file);
                }
            }
            Event::FileSendComplete { .. } => {
                if let Some(progress) = self.progress.as_mut()
                    && progress.for_ == ProgressFor::Update
                {
                    progress.bar = crate::config::serial_ports::Bar {
                        marquee: false,
                        value: 100,
                    };
                    FILE_SEND_COMPLETE.clone_into(&mut progress.text);
                }
            }
        }
    }

    // --- Once a frame -----------------------------------------------------------------------

    /// Once a frame: the page object let go with its screen; the bus read; the node's second;
    /// what it received handed to each handler; the waits; what it wrote put on the bus.
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        on_setup: bool,
        now: Instant,
        wall_second: u32,
        favourites: Option<&str>,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            self.disconnect();
            *self = Self::default();
        }
        self.tick_slcan_start(telemetry, view, now);
        self.read_bus(telemetry, view, now);
        if let Some(node) = self.can.as_mut() {
            node.tick(now, wall_second);
        }
        self.dispatch(now);
        self.poll_jobs(now, favourites);
        self.tick_update(now);
        self.tick_passthrough(now);
        self.dispatch(now);
        self.write_bus(telemetry, view);
        if let Some(OpenWindow::Params(window)) = self.window.as_mut()
            && window.search_due.is_some_and(|due| now >= due)
        {
            window.search_due = None;
            window.filter();
        }
    }

    /// A line logged, as `ReadLine` and `WriteToStreamSLCAN` write them.
    fn log_line(&mut self, line: &str) {
        if let Some(file) = self.log_file.as_mut() {
            let _ = file.write_all(line.as_bytes());
        }
    }

    /// The bus's input to the node.
    fn read_bus(&mut self, telemetry: &Telemetry, view: &TelemetryView, now: Instant) {
        let mut lines: Vec<String> = Vec::new();
        let mut opened = false;
        let mut failed = None;
        match self.bus.as_mut() {
            Some(Bus::Mavlink(bus)) => {
                // `SubscribeToPacketType(CAN_FRAME, ..., sysidcurrent, compidcurrent, true)`:
                // made once the link is there, again for a new link.
                let stale = bus
                    .subscription
                    .as_ref()
                    .is_none_or(|subscription| !telemetry.carries(subscription));
                if telemetry.has_link() && stale {
                    let target = view.vehicle;
                    let heard = Arc::clone(&bus.heard);
                    bus.subscription = telemetry.on_packet(move |packet| {
                        if packet.sent || packet.msgid != mavlink::CAN_FRAME_ID {
                            return;
                        }
                        let from = VehicleId::new(packet.sysid, packet.compid);
                        if target.is_some_and(|target| target != from) {
                            return;
                        }
                        if let MavMessage::CanFrame(frame) = packet.message
                            && let Ok(mut heard) = heard.os_lock()
                        {
                            heard.push_back(frame);
                        }
                    });
                }
                if let Ok(mut heard) = bus.heard.os_lock() {
                    for frame in heard.drain(..) {
                        bus.received += 1;
                        lines.push(mavlink::line_of(&frame));
                    }
                }
                // The forwarding loop: `mavlinkCANRun = true` after its first sleep, then
                // `CAN_FORWARD` every second, not waited for.
                if now >= bus.next_forward {
                    self.can_run = true;
                    bus.next_forward = now + Duration::from_secs(1);
                    if let Some(vehicle) = view.vehicle
                        && telemetry.send(&mavlink::can_forward(
                            vehicle.sysid,
                            vehicle.compid,
                            bus.bus,
                        ))
                    {
                        bus.forwards += 1;
                    }
                }
            }
            Some(Bus::Slcan(bus)) => loop {
                match bus.events.try_recv() {
                    Ok(SlcanEvent::Handshake(line)) => {
                        if let Some(file) = self.log_file.as_mut() {
                            let _ = file.write_all(line.as_bytes());
                        }
                    }
                    Ok(SlcanEvent::Opened) => {
                        bus.opened = true;
                        opened = true;
                    }
                    Ok(SlcanEvent::Line(line)) => lines.push(line),
                    Ok(SlcanEvent::Failed(error)) => {
                        failed = Some(error);
                        break;
                    }
                    Err(_) => break,
                }
            },
            Some(Bus::Mcast(bus)) => {
                while let Ok(line) = bus.lines.try_recv() {
                    lines.push(line);
                }
            }
            None => {}
        }
        if opened {
            // `StartSLCAN` done: the node runs, with the file server and the allocator.
            if self
                .progress
                .as_ref()
                .is_some_and(|progress| progress.for_ == ProgressFor::Slcan)
            {
                self.progress = None;
            }
            self.setup_node(now, true);
        }
        if let Some(error) = failed {
            if self
                .progress
                .as_ref()
                .is_some_and(|progress| progress.for_ == ProgressFor::Slcan)
            {
                self.progress = None;
            }
            self.bus = None;
            self.status = Some(error);
        }
        for line in &lines {
            self.log_line(line);
            if let Some(node) = self.can.as_mut() {
                node.receive_line(line);
            }
        }
    }

    /// What the node wrote, onto the bus.
    fn write_bus(&mut self, telemetry: &Telemetry, view: &TelemetryView) {
        let Some(node) = self.can.as_mut() else {
            return;
        };
        let lines = node.take_outgoing();
        for line in &lines {
            self.log_line(line);
        }
        match self.bus.as_mut() {
            Some(Bus::Mavlink(bus)) => {
                let Some(vehicle) = view.vehicle else {
                    return;
                };
                for line in &lines {
                    if let Some(message) =
                        mavlink::message_of(line, vehicle.sysid, vehicle.compid, bus.bus)
                        && telemetry.send(&message)
                    {
                        bus.sent += 1;
                    }
                }
            }
            Some(Bus::Slcan(bus)) => {
                for line in lines {
                    let _ = bus.commands.send(SlcanCommand::Write(line));
                }
            }
            Some(Bus::Mcast(bus)) => {
                for line in &lines {
                    if let Some(datagram) = mcast::datagram_of(line) {
                        let _ = mcast::send(&bus.socket, bus.bus, &datagram);
                    }
                }
            }
            None => {}
        }
    }

    /// Every message received handed to each handler subscribed, and the events after them.
    fn dispatch(&mut self, now: Instant) {
        let Some(mut node) = self.can.take() else {
            return;
        };
        while let Some(received) = node.next_received() {
            self.page_handler(&received, &mut node);
            if let Some(OpenWindow::Stats(stats)) = self.window.as_mut() {
                stats.heard(&received);
            }
            if let Some(inspector) = self.inspector.as_mut() {
                inspector.heard(&received, now);
            }
            self.passthrough_handler(&received, &node);
            if let Some(fetch) = self.fetch.as_mut() {
                fetch.job.handle(&received, &mut node, now);
            }
            for restart in &mut self.restarts {
                restart.handle(&received, &mut node, now);
            }
            if let Some(UpdateFlow::Running(update)) = self.update.as_mut() {
                update.handle(&received, &mut node, now);
            }
            if let Some(OpenWindow::Params(window)) = self.window.as_mut() {
                match window.busy.as_mut() {
                    Some(ParamsBusy::Writing {
                        current, saving, ..
                    }) => {
                        if let Some((_, request)) = current.as_mut() {
                            request.handle(&received, &mut node, now);
                        }
                        if let Some(request) = saving.as_mut() {
                            request.handle(&received, &mut node, now);
                        }
                    }
                    Some(ParamsBusy::Refreshing(job)) => job.handle(&received, &mut node, now),
                    Some(ParamsBusy::Committing(request)) => {
                        request.handle(&received, &mut node, now);
                    }
                    None => {}
                }
            }
        }
        for event in node.take_events() {
            if let Some(UpdateFlow::Running(update)) = self.update.as_mut() {
                update.event(&event);
            }
            self.node_event(&event);
        }
        if let Some(inspector) = self.inspector.as_mut() {
            inspector.tick(now, &|id| node.node_name(id));
        }
        self.can = Some(node);
    }

    /// The waits polled.
    fn poll_jobs(&mut self, now: Instant, favourites: Option<&str>) {
        let Some(mut node) = self.can.take() else {
            return;
        };
        if let Some(fetch) = self.fetch.as_mut()
            && let Poll::Done(list) = fetch.job.poll(&mut node, now)
        {
            let node_id = fetch.node;
            self.fetch = None;
            if self
                .progress
                .as_ref()
                .is_some_and(|progress| progress.for_ == ProgressFor::Parameters)
            {
                self.progress = None;
                self.param_cache.extend(list.iter().cloned());
                // `new DroneCANParams(can, nodeID, paramlist).ShowUserControl()`.
                let favourites = crate::raw_params_grid::get_list(favourites);
                self.window = Some(OpenWindow::Params(Box::new(ParamsWindow::new(
                    node_id,
                    list,
                    &favourites,
                ))));
            }
        }
        self.restarts
            .retain_mut(|restart| matches!(restart.poll(&mut node, now), Poll::Pending));
        let cache = &mut self.param_cache;
        let mut status = None;
        if let Some(OpenWindow::Params(window)) = self.window.as_mut() {
            status = poll_params(window, &mut node, cache, now);
        }
        if status.is_some() {
            self.status = status;
        }
        self.can = Some(node);
    }

    /// The firmware update: the download's end, then the update's.
    fn tick_update(&mut self, now: Instant) {
        let Some(flow) = self.update.take() else {
            return;
        };
        match flow {
            UpdateFlow::Fetching {
                node,
                results,
                cancelled,
            } => match results.try_recv() {
                Ok(Ok((file, manifest))) => {
                    if manifest.is_some() {
                        self.manifest = manifest;
                    }
                    if cancelled {
                        self.progress = None;
                    } else {
                        self.start_update(node, &file, now);
                    }
                }
                Ok(Err(error)) => {
                    self.progress = None;
                    self.status = Some(error);
                }
                Err(TryRecvError::Empty) => {
                    self.update = Some(UpdateFlow::Fetching {
                        node,
                        results,
                        cancelled,
                    });
                }
                Err(TryRecvError::Disconnected) => self.progress = None,
            },
            UpdateFlow::Running(mut update) => {
                let Some(mut node) = self.can.take() else {
                    self.progress = None;
                    return;
                };
                match update.poll(&mut node, now) {
                    Poll::Pending => self.update = Some(UpdateFlow::Running(update)),
                    Poll::Done(end) => {
                        self.progress = None;
                        if let UpdateEnd::Error(error) = end {
                            self.status = Some(error);
                        }
                    }
                }
                self.can = Some(node);
            }
        }
    }

    /// The progress window's Cancel.
    pub fn cancel_progress(&mut self) {
        let Some(progress) = self.progress.take() else {
            return;
        };
        match progress.for_ {
            // `CancelAcknowledged`, `port.Close()`: the page stays "connected" to nothing.
            ProgressFor::Slcan => self.stop_bus(false),
            // `ForceExit`: the dialog goes and the window does not open.
            ProgressFor::Parameters => self.fetch = None,
            ProgressFor::Update => match self.update.as_mut() {
                Some(UpdateFlow::Running(update)) => update.cancel(),
                Some(UpdateFlow::Fetching { cancelled, .. }) => *cancelled = true,
                None => {}
            },
        }
    }

    /// A passthrough's `MessageReceived` handler: the GPS's data to the client.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:776-810, 1076-1106`
    fn passthrough_handler(&self, received: &Received, node: &Node) {
        let Some(passthrough) = self.passthrough.as_ref() else {
            return;
        };
        let data = match (&passthrough.tunnel, &received.message) {
            (
                None,
                CanMessage::RtcmStream { data, .. } | CanMessage::MovingBaselineData { data },
            ) => data.clone(),
            (Some((target, _)), CanMessage::TunnelTargetted(tunnel))
                if received.frame.source_node() == *target
                    && tunnel.target_node == node.source_node() =>
            {
                tunnel.buffer.clone()
            }
            _ => return,
        };
        if passthrough.client {
            let _ = passthrough.to_client.send(data);
        }
    }

    /// The passthrough's thread heard: its client's bytes onto the bus, the Here3+/4's baud
    /// tunnels and keep-alives.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:812-847, 1031-1065, 1108-1185`
    fn tick_passthrough(&mut self, now: Instant) {
        let Some(mut passthrough) = self.passthrough.take() else {
            return;
        };
        let Some(node) = self.can.as_mut() else {
            self.passthrough = Some(passthrough);
            return;
        };
        let mut failed = None;
        loop {
            match passthrough.from_client.try_recv() {
                Ok(PassthroughEvent::Connected) => passthrough.client = true,
                Ok(PassthroughEvent::Gone) => passthrough.client = false,
                Ok(PassthroughEvent::Bytes(bytes)) => {
                    let message = match passthrough.tunnel {
                        None => CanMessage::RtcmStream {
                            protocol_id: 3,
                            data: bytes,
                        },
                        Some((target, baud)) => CanMessage::TunnelTargetted(TunnelTargetted {
                            protocol: TUNNEL_PROTOCOL_GPS_GENERIC,
                            target_node: target,
                            serial_id: -1,
                            options: TUNNEL_OPTION_LOCK_PORT,
                            baudrate: baud,
                            buffer: bytes,
                        }),
                    };
                    let id = node.next_transfer_id();
                    node.send(0, PRIORITY, id, message);
                    passthrough.last_send = now;
                }
                Ok(PassthroughEvent::Failed(error)) => {
                    failed = Some(error);
                    break;
                }
                Err(_) => break,
            }
        }
        if let Some((target, baud)) = passthrough.tunnel {
            if now >= passthrough.next_baud
                && let Some(at) = passthrough.bauds.pop_front()
            {
                let packet = here4_baud_packet(baud);
                let id = node.next_transfer_id();
                node.send(
                    0,
                    PRIORITY,
                    id,
                    CanMessage::TunnelTargetted(TunnelTargetted {
                        protocol: TUNNEL_PROTOCOL_GPS_GENERIC,
                        target_node: target,
                        serial_id: -1,
                        options: if passthrough.lock {
                            TUNNEL_OPTION_LOCK_PORT
                        } else {
                            0
                        },
                        baudrate: at,
                        buffer: packet,
                    }),
                );
                passthrough.next_baud = now + Duration::from_millis(100);
            }
            // The half-second keep-alive, while a client is there.
            if passthrough.client
                && passthrough.bauds.is_empty()
                && now.saturating_duration_since(passthrough.last_send)
                    >= Duration::from_millis(500)
            {
                let id = node.next_transfer_id();
                node.send(
                    0,
                    PRIORITY,
                    id,
                    CanMessage::TunnelTargetted(TunnelTargetted {
                        protocol: TUNNEL_PROTOCOL_GPS_GENERIC,
                        target_node: target,
                        serial_id: -1,
                        options: TUNNEL_OPTION_LOCK_PORT,
                        baudrate: baud,
                        buffer: Vec::new(),
                    }),
                );
                passthrough.last_send = now;
            }
        }
        match failed {
            Some(error) => {
                // `CustomMessageBox.Show(Strings.ERROR, "Forwarder problem " + ex)`.
                self.status = Some(format!("Forwarder problem {error}"));
            }
            None => self.passthrough = Some(passthrough),
        }
    }

    // --- Questions, paths and boxes ---------------------------------------------------------

    /// The question showing.
    #[must_use]
    pub fn question(&self) -> Option<(&'static str, &str)> {
        self.question
            .as_ref()
            .map(|question| (question.caption, question.text.as_str()))
    }

    /// A question answered: Yes/OK or No/Cancel. `slcan_url` is the connection box's URL, for
    /// the SLCAN question; `settings` takes "Show me again?" answers.
    pub fn answer(
        &mut self,
        yes: bool,
        slcan_url: Option<String>,
        settings: &mut crate::settings::Persisted,
        now: Instant,
    ) {
        let Some(question) = self.question.take() else {
            return;
        };
        match question.asked {
            Asked::SlcanWithoutLink => {
                if yes {
                    self.slcan_without_link(slcan_url, now);
                }
            }
            Asked::SearchInternet { node, beta } => {
                if yes {
                    self.search_internet(node, beta, now);
                } else {
                    let directory = std::env::current_dir()
                        .map(|dir| dir.display().to_string())
                        .unwrap_or_default();
                    self.path = Some((
                        PathFor::Firmware { node },
                        crate::config::firmware::PathBox::new(&directory, "*.bin;*.apj"),
                    ));
                }
            }
            Asked::NewFirmware { again } => {
                if !again {
                    settings.set(NEW_FIRMWARE_SHOW_AGAIN_KEY, "False");
                    self.new_firmware_suppressed = true;
                }
            }
            Asked::WriteParams { again } => {
                if !again {
                    settings.set(WRITE_SHOW_AGAIN_KEY, "False");
                }
                if yes {
                    self.write_params();
                }
            }
            Asked::RefreshParams => {
                if yes && let Some(OpenWindow::Params(window)) = self.window.as_mut() {
                    window.busy =
                        Some(ParamsBusy::Refreshing(GetParameters::new(window.node, now)));
                }
            }
        }
    }

    /// "Show me again?" clicked on a show-again box.
    pub fn toggle_show_again(&mut self) {
        if let Some(Question {
            asked: Asked::NewFirmware { again } | Asked::WriteParams { again },
            ..
        }) = self.question.as_mut()
        {
            *again = !*again;
        }
    }

    /// Whether the question has a "Show me again?" box, and its state.
    #[must_use]
    pub fn show_again(&self) -> Option<bool> {
        match self.question.as_ref()?.asked {
            Asked::NewFirmware { again } | Asked::WriteParams { again } => Some(again),
            _ => None,
        }
    }

    /// The typed path's dialog closed: OK with a path, or Cancel.
    pub fn answer_path(&mut self, ok: bool, now: Instant) {
        let Some((for_, path)) = self.path.take() else {
            return;
        };
        if !ok {
            return;
        }
        match for_ {
            PathFor::Firmware { node } => {
                if let Some(file) = path.chosen() {
                    self.update_from_file(node, &file, now);
                }
            }
            PathFor::LoadParams => {
                if let Some(file) = path.chosen()
                    && let Ok(params) = mp_params::param_file::ParamFile::load(&file)
                    && let Some(window) = self.params_window_mut()
                {
                    window.load(&params);
                }
            }
            PathFor::SaveParams => {
                let file = PathBuf::from(path.field.value());
                if let Some(window) = self.params_window_mut()
                    && let Err(error) = window.save(&file)
                {
                    self.status = Some(error.to_string());
                }
            }
        }
    }

    /// A key in the typed path's box.
    pub fn path_key(&mut self, event: &KeyDownEvent, now: Instant) -> bool {
        let Some((_, path)) = self.path.as_mut() else {
            return false;
        };
        match path.field.key(event) {
            KeyOutcome::Submitted => {
                self.answer_path(true, now);
                true
            }
            KeyOutcome::Cancelled => {
                self.answer_path(false, now);
                true
            }
            KeyOutcome::Changed => true,
            KeyOutcome::Ignored => false,
        }
    }

    /// A key in the `InputBox`.
    pub fn input_key(&mut self, event: &KeyDownEvent, now: Instant) -> bool {
        let Some(input) = self.input.as_mut() else {
            return false;
        };
        match input.field.key(event) {
            KeyOutcome::Submitted => {
                self.answer_input(true, now);
                true
            }
            KeyOutcome::Cancelled => {
                self.answer_input(false, now);
                true
            }
            KeyOutcome::Changed => true,
            KeyOutcome::Ignored => false,
        }
    }

    // --- The parameter window's buttons -----------------------------------------------------

    /// Write Params: the question unless its "Show me again?" was turned off.
    /// `// C#: Controls/DroneCANParams.cs:406-410; Common.cs:260-270`
    pub fn press_write(&mut self, shown_again: Option<&str>) {
        let Some(window) = self.params_window() else {
            return;
        };
        if window.busy() {
            return;
        }
        let suppressed =
            shown_again.is_some_and(|value| !crate::raw_params::get_boolean(Some(value)));
        if suppressed {
            self.write_params();
        } else {
            self.question = Some(Question {
                caption: WRITE_TITLE,
                text: WRITE_SURE.to_owned(),
                asked: Asked::WriteParams { again: true },
            });
        }
    }

    /// The writes: each change, `ENABLE`s first.
    /// `// C#: Controls/DroneCANParams.cs:412-472`
    fn write_params(&mut self) {
        let Some(window) = self.params_window_mut() else {
            return;
        };
        let mut names: Vec<String> = window.changes.keys().cloned().collect();
        crate::config::adsb::sort_enable(&mut names);
        window.busy = Some(ParamsBusy::Writing {
            queue: names.into_iter().collect(),
            current: None,
            saving: None,
        });
    }

    /// Refresh Params: `WarningUpdateParamList`, OK/Cancel.
    /// `// C#: Controls/DroneCANParams.cs:318-346`
    pub fn press_refresh(&mut self) {
        if self.params_window().is_some_and(|window| !window.busy()) {
            self.question = Some(Question {
                caption: ERROR_TITLE,
                text: crate::config::adsb::REFRESH_WARNING.to_owned(),
                asked: Asked::RefreshParams,
            });
        }
    }

    /// Commit Params: `SaveConfig`.
    /// `// C#: Controls/DroneCANParams.cs:257-272`
    pub fn press_commit(&mut self) {
        if let Some(window) = self.params_window_mut()
            && window.busy.is_none()
        {
            window.busy = Some(ParamsBusy::Committing(Request::save_config(window.node)));
        }
    }

    /// Load from file or Save to file: the typed path.
    pub fn press_file(&mut self, save: bool) {
        if self.params_window().is_none() {
            return;
        }
        let directory = std::env::current_dir()
            .map(|dir| dir.display().to_string())
            .unwrap_or_default();
        let mut path = crate::config::firmware::PathBox::new(
            &directory,
            if save {
                "Param List|*.param;*.parm"
            } else {
                "Parameter File|*.param;*.parm|All Files|*.*"
            },
        );
        if save {
            path.caption = "Save As";
        }
        self.path = Some((
            if save {
                PathFor::SaveParams
            } else {
                PathFor::LoadParams
            },
            path,
        ));
    }

    /// The parameter window's box dismissed.
    pub fn dismiss_params_message(&mut self) {
        if let Some(window) = self.params_window_mut() {
            window.messages.pop_front();
        }
    }
}

/// The parameter window's waits: the writes in turn, the refresh, the commit. What goes on the
/// status line.
fn poll_params(
    window: &mut ParamsWindow,
    node: &mut Node,
    cache: &mut Vec<GetSetRes>,
    now: Instant,
) -> Option<String> {
    let busy = window.busy.take()?;
    match busy {
        ParamsBusy::Writing {
            mut queue,
            mut current,
            mut saving,
        } => {
            if let Some(save) = saving.as_mut() {
                if let Poll::Done(_) = save.poll(node, now) {
                    // `_can.SaveConfig(_node)`'s answer is not looked at.
                    window.messages.push_back(Message {
                        title: SAVED_CAPTION,
                        text: PARAMS_SAVED.to_owned(),
                    });
                    return None;
                }
                window.busy = Some(ParamsBusy::Writing {
                    queue,
                    current,
                    saving,
                });
                return None;
            }
            if let Some((name, request)) = current.as_mut()
                && let Poll::Done(_) = request.poll(node, now)
            {
                // Whatever the node said: the cell back to normal and the change forgotten.
                if let Some(row) = window.rows.iter_mut().find(|row| row.name == *name) {
                    row.shade = Shade::Normal;
                }
                window.changes.remove(name.as_str());
                current = None;
            }
            while current.is_none() {
                let Some(name) = queue.pop_front() else {
                    break;
                };
                // `SetParameter(node, name, value)`: the type from the cache; not known, false
                // without a send.
                let Some(kind) = cache
                    .iter()
                    .find(|res| ascii(&res.name) == name)
                    .map(|res| res.value.clone())
                else {
                    if let Some(row) = window.rows.iter_mut().find(|row| row.name == name) {
                        row.shade = Shade::Normal;
                    }
                    window.changes.remove(&name);
                    continue;
                };
                let value = match window.changes.get(&name) {
                    Some(Change::Number(number)) => SetValue::Number(*number),
                    Some(Change::Text(text)) => SetValue::Text(text.clone()),
                    None => SetValue::Number(0.0),
                };
                let request = Request::set_parameter(window.node, &name, &value, &kind);
                current = Some((name, request));
            }
            if current.is_none() && queue.is_empty() {
                saving = Some(Request::save_config(window.node));
            }
            window.busy = Some(ParamsBusy::Writing {
                queue,
                current,
                saving,
            });
            None
        }
        ParamsBusy::Refreshing(mut job) => match job.poll(node, now) {
            Poll::Pending => {
                window.busy = Some(ParamsBusy::Refreshing(job));
                None
            }
            Poll::Done(list) => {
                cache.extend(list.iter().cloned());
                window.list = list;
                let favourites: Vec<String> = window
                    .rows
                    .iter()
                    .filter(|row| row.fav)
                    .map(|row| row.name.clone())
                    .collect();
                window.process_to_screen(&favourites);
                None
            }
        },
        ParamsBusy::Committing(mut request) => match request.poll(node, now) {
            Poll::Pending => {
                window.busy = Some(ParamsBusy::Committing(request));
                None
            }
            Poll::Done(ok) => {
                window.messages.push_back(Message {
                    title: "",
                    text: COMMITTED.to_owned(),
                });
                (!ok).then(|| FAILED_TO_SAVE.to_owned())
            }
        },
    }
}

/// `hardware_version.major + "." + minor`.
fn hw_version_text(info: &GetNodeInfoRes) -> String {
    format!(
        "{}.{}",
        info.hardware_version.major, info.hardware_version.minor
    )
}

/// `hwversion.ToString("0.0##")` of `double.Parse(major + "." + minor)`: at least one place, at
/// most three.
#[must_use]
pub fn hw_version_url_text(text: &str) -> String {
    let value: f64 = text.parse().unwrap_or(0.0);
    let mut text = format!("{value:.3}");
    while text.ends_with('0') && !text.ends_with(".0") {
        text.pop();
    }
    text
}

/// `LookForUpdate`'s servers: a name containing the key is looked for under the URL.
/// `// C#: ExtLibs/DroneCAN/DroneCAN.cs:1052-1096`
#[must_use]
pub fn update_servers(beta: bool) -> Vec<(&'static str, &'static str)> {
    if beta {
        vec![
            ("com.hex.", "https://firmware.cubepilot.org/UAVCAN/beta/"),
            (
                "com.cubepilot.",
                "https://firmware.cubepilot.org/UAVCAN/beta/",
            ),
        ]
    } else {
        vec![
            ("com.hex.", "https://firmware.cubepilot.org/UAVCAN/"),
            ("com.cubepilot.", "https://firmware.cubepilot.org/UAVCAN/"),
            ("search.id", "http://localhost/url"),
        ]
    }
}

/// `FirmwareUpdate`'s search and download, on the update's thread: `LookForUpdate`'s HEADs,
/// else the manifest's first official (or beta) peripheral `bin` or `zip` whose platform ends
/// the node's name; downloaded, and a zip's firmware taken out. The file, and the manifest when
/// it was loaded here.
fn fetch_firmware(
    device: &str,
    hwversion: &str,
    beta: bool,
    manifest: Option<Arc<mp_firmware::manifest::Manifest>>,
) -> Fetched {
    use mp_firmware::manifest::{Fetch as _, Http};
    let http = Http;
    let mut url = String::new();
    for (key, server) in update_servers(beta) {
        if !device.contains(key) {
            continue;
        }
        let candidate = format!(
            "{server}{device}/{}/firmware.bin",
            hw_version_url_text(hwversion)
        );
        if http.exists(&candidate)? {
            url = candidate;
            break;
        }
    }
    let mut loaded = None;
    let mut firmware_name = None;
    if url.is_empty() {
        let manifest = match manifest {
            Some(manifest) => manifest,
            None => {
                let mut fresh = None;
                let _ = mp_firmware::manifest::load(&mut fresh, &http);
                let fresh = Arc::new(fresh.ok_or_else(|| {
                    "Object reference not set to an instance of an object.".to_owned()
                })?);
                loaded = Some(Arc::clone(&fresh));
                fresh
            }
        };
        let release = if beta { "BETA" } else { "OFFICIAL" };
        let option = manifest
            .firmware
            .iter()
            .find(|a| {
                a.mav_firmware_version_type.as_deref() == Some(release)
                    && a.vehicle_type.as_deref() == Some("AP_Periph")
                    && matches!(a.format.as_deref(), Some("bin" | "zip"))
                    && a.mav_type.as_deref() == Some("CAN_PERIPHERAL")
                    && a.platform
                        .as_deref()
                        .is_some_and(|platform| device.ends_with(platform))
            })
            .ok_or_else(|| "Sequence contains no elements".to_owned())?;
        url = option.url.clone().unwrap_or_default();
        firmware_name = option.firmware_name.clone();
    }
    if url.is_empty() {
        return Err(UPDATE_NOT_FOUND.to_owned());
    }
    let bytes = http.get(&url)?;
    let directory = mp_os::temp_dir().join(format!("dronecan-fw-{}", mp_os::process_id()));
    mp_os::fs::create_dir_all(&directory).map_err(|error| error.to_string())?;
    let downloaded = directory.join("download.tmp");
    mp_os::fs::write(&downloaded, &bytes).map_err(|error| error.to_string())?;
    if url.to_lowercase().ends_with(".zip") {
        let unpacked = directory.join("unzipped");
        mp_log::zip::extract(&bytes, &unpacked).map_err(|error| error.to_string())?;
        let name = firmware_name
            .ok_or_else(|| "Object reference not set to an instance of an object.".to_owned())?;
        return Ok((unpacked.join(name), loaded));
    }
    Ok((downloaded, loaded))
}

/// `Settings.Instance.LogDir`.
fn log_directory() -> Option<PathBuf> {
    std::env::var_os("MP_LOG_DIR")
        .map(PathBuf::from)
        .or_else(|| {
            mp_settings::Config::default_path()
                .and_then(|path| mp_settings::Config::load(&path).ok())
                .and_then(|config| config.log_directory())
                .or_else(mp_settings::default_log_directory)
        })
}

// ---------------------------------------------------------------------------------------------
// Facts.
// ---------------------------------------------------------------------------------------------

/// Facts a UI test asserts on.
pub fn record_facts(page: &DroneCan) {
    use crate::facts::record;
    record("config.dronecan.active", page.is_active());
    record("config.dronecan.interface", page.interface().name());
    record("config.dronecan.network.visible", page.network_visible);
    record("config.dronecan.networks", page.networks.len());
    record("config.dronecan.connected", page.connected);
    record("config.dronecan.button", page.connect_text());
    record("config.dronecan.lists.enabled", page.lists_enabled);
    record("config.dronecan.filter.enabled", page.filter_enabled);
    record("config.dronecan.checkupdate", page.check_update);
    record("config.dronecan.log", page.log);
    record("config.dronecan.exitslcan", page.exit_slcan);
    record(
        "config.dronecan.bus",
        page.bus.as_ref().map_or("none", Bus::kind),
    );
    record("config.dronecan.run", page.can_run);
    record(
        "config.dronecan.slcan.opened",
        matches!(page.bus.as_ref(), Some(Bus::Slcan(bus)) if bus.opened),
    );
    let (forwards, sent, received) = match page.bus.as_ref() {
        Some(Bus::Mavlink(bus)) => (bus.forwards, bus.sent, bus.received),
        _ => (0, 0, 0),
    };
    record("config.dronecan.forwards", forwards);
    record("config.dronecan.frames.sent", sent);
    record("config.dronecan.frames.received", received);
    record(
        "config.dronecan.node.running",
        page.node().is_some_and(Node::is_running),
    );
    record("config.dronecan.nodes", page.nodes().len());
    for row in page.nodes() {
        let key = format!("config.dronecan.node.{}", row.id);
        record(format!("{key}.name"), &row.name);
        record(format!("{key}.mode"), &row.mode);
        record(format!("{key}.health"), &row.health);
        record(
            format!("{key}.swversion"),
            row.software_version.as_deref().unwrap_or(""),
        );
    }
    record(
        "config.dronecan.current",
        page.current()
            .map_or_else(|| "none".to_owned(), |row| row.id.to_string()),
    );
    record("config.dronecan.menu", page.menu.is_some());
    record("config.dronecan.debug", page.debug.len());
    record(
        "config.dronecan.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    record(
        "config.dronecan.question",
        page.question().map_or("none", |(caption, _)| caption),
    );
    record(
        "config.dronecan.progress",
        page.progress
            .as_ref()
            .map_or("none", |progress| progress.text.as_str()),
    );
    record(
        "config.dronecan.window",
        match &page.window {
            None => "none",
            Some(OpenWindow::Params(_)) => "params",
            Some(OpenWindow::Filter(_)) => "filter",
            Some(OpenWindow::Stats(_)) => "stats",
        },
    );
    if let Some(window) = page.params_window() {
        record("config.dronecan.params.node", window.node);
        record("config.dronecan.params.rows", window.rows().len());
        record("config.dronecan.params.changes", window.changes().len());
        record(
            "config.dronecan.params.editing",
            window.editing().map_or("none", |(name, _)| name),
        );
        record("config.dronecan.params.busy", window.busy());
    }
    if let Some(OpenWindow::Filter(filter)) = &page.window {
        record("config.dronecan.filter.boxes", filter.boxes.len());
        record("config.dronecan.filter.ids", filter.ids.len());
        record("config.dronecan.filter.sent", filter.sent);
    }
    record("config.dronecan.restarts", page.restarts.len());
    record("config.dronecan.passthrough", page.passthrough.is_some());
    record(
        "config.dronecan.input",
        page.input.as_ref().map_or("none", |input| input.title),
    );
}

// ---------------------------------------------------------------------------------------------
// Its part in the application: `config/extra_setup.rs` calls these.
// ---------------------------------------------------------------------------------------------

impl MissionPlanner {
    /// `Activate`.
    pub(crate) fn dronecan_activate(&mut self) {
        let view = self.telemetry.view();
        let key = Key::of(&view);
        let update_check = self.persisted.get(UPDATE_CHECK_KEY).map(str::to_owned);
        let again = self
            .persisted
            .get(NEW_FIRMWARE_SHOW_AGAIN_KEY)
            .map(str::to_owned);
        self.extra
            .dronecan
            .activate(key, update_check.as_deref(), again.as_deref());
    }

    /// Once a frame: the page's tick, the port SLCAN takes from the link, and the status line.
    pub(crate) fn dronecan_tick(&mut self, view: &TelemetryView) {
        let on_setup = self.screen == crate::Screen::Setup;
        let now = Instant::now();
        let wall_second = {
            use chrono::Timelike as _;
            chrono::Local::now().second()
        };
        self.extra.dronecan.tick(
            &self.telemetry,
            view,
            on_setup,
            now,
            wall_second,
            self.persisted.get("fav_params"),
        );
        if let Some(url) = self.extra.dronecan.take_port_request() {
            // "grab the connected port, place an invalid port in its place": the link goes, and
            // its port is opened for SLCAN.
            self.do_disconnect();
            self.extra.dronecan.open_slcan(&url, now);
        }
        if let Some(status) = self.extra.dronecan.take_status() {
            self.file_status = Some(status);
        }
    }

    /// The connection box's URL, as `SetupSLCanPort` builds its port from `CMB_serialport` and
    /// `CMB_baudrate`: a network kind with the answers saved for its questions, or their
    /// defaults.
    /// `// C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:267-294`
    fn dronecan_box_url(&self) -> Option<String> {
        let port = self.connect_box.port.clone();
        let kind = crate::connect::kind(&port);
        let answers: Vec<String> = crate::connect::questions(kind)
            .iter()
            .map(|question| {
                self.persisted
                    .get(question.key)
                    .unwrap_or(question.default)
                    .to_owned()
            })
            .collect();
        crate::connect::url(kind, &port, &self.connect_box.baud, &answers)
    }

    /// The focus handle the page's boxes type into.
    fn dronecan_focus(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let handle = self
            .extra
            .dronecan
            .focus
            .get_or_insert_with(|| cx.focus_handle())
            .clone();
        handle.focus(window, cx);
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// A grid cell: its text, left-aligned and vertically centred, in a bordered box.
fn cell(width: f32, height: f32, text: impl Into<SharedString>, header: bool) -> Div {
    div()
        .flex_shrink_0()
        .w(px(width))
        .h(px(height))
        .flex()
        .items_center()
        .px_1()
        .border_r_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(if header { theme::ACTION } else { theme::BG }))
        .text_xs()
        .overflow_hidden()
        .whitespace_nowrap()
        .text_color(rgb(theme::TEXT))
        .child(text.into())
}

/// A check box of the page, its state its own.
fn page_check(
    id: &str,
    text: &'static str,
    ticked: bool,
    place: (f32, f32),
    on_click: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let mut check = Check::default();
    check.enabled = true;
    check.state = if ticked {
        CheckState::Checked
    } else {
        CheckState::Unchecked
    };
    check_box(id.to_owned(), &check, text, place, on_click, cx)
}

/// The node grid: headers, rows, the Menu cells.
fn node_grid(page: &DroneCan, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let (gx, gy, gw, gh) = GRID_AT;
    let mut header = div()
        .flex()
        .flex_shrink_0()
        .child(cell(ROW_HEADER, HEADER_HEIGHT, "", true));
    for (name, width) in COLUMNS {
        header = header.child(cell(width, HEADER_HEIGHT, name, true));
    }
    let mut rows = div()
        .id("dronecan-grid-rows")
        .flex()
        .flex_col()
        .flex_1()
        .overflow_y_scroll();
    for (index, row) in page.nodes.iter().enumerate() {
        let current = page.current == Some(index);
        let crc = format!("{:X}", row.software_crc);
        let texts: [String; 8] = [
            row.id.to_string(),
            row.name.clone(),
            row.mode.clone(),
            row.health.clone(),
            uptime_text(row.uptime),
            row.hardware_version.clone().unwrap_or_default(),
            row.software_version.clone().unwrap_or_default(),
            crc,
        ];
        let id = format!("dronecan-row-{}", row.id);
        let mut line = crate::probe::measured(id.clone(), div())
            .id(SharedString::from(id))
            .flex()
            .flex_shrink_0()
            .cursor_pointer()
            .child(cell(
                ROW_HEADER,
                ROW_HEIGHT,
                if current { "▶" } else { "" },
                true,
            ));
        for (text, (_, width)) in texts.into_iter().zip(COLUMNS) {
            let mut text_cell = cell(width, ROW_HEIGHT, text, false);
            if current {
                text_cell = text_cell.bg(rgb(theme::SELECTION));
            }
            line = line.child(text_cell);
        }
        let menu_id = format!("dronecan-row-{}-menu", row.id);
        let menu =
            crate::probe::measured(menu_id.clone(), cell(50.0, ROW_HEIGHT, MENU_CELL, false))
                .id(SharedString::from(menu_id))
                .justify_center()
                .on_click(cx.listener(move |this, event: &ClickEvent, _window, cx| {
                    let at = event.position();
                    this.extra
                        .dronecan
                        .click_menu_cell(index, (f32::from(at.x), f32::from(at.y)));
                    cx.stop_propagation();
                    cx.notify();
                }));
        line = line.child(menu).on_click(cx.listener(
            move |this, _event: &ClickEvent, _window, cx| {
                this.extra.dronecan.select_row(index);
                cx.notify();
            },
        ));
        rows = rows.child(line);
    }
    crate::probe::measured("dronecan-grid", at(gx, gy, gw, gh))
        .flex()
        .flex_col()
        .overflow_hidden()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .on_mouse_down(
            MouseButton::Right,
            cx.listener(|this, event: &MouseDownEvent, _window, cx| {
                this.extra
                    .dronecan
                    .right_click((f32::from(event.position.x), f32::from(event.position.y)));
                cx.notify();
            }),
        )
        .child(header)
        .child(rows)
        .into_any_element()
}

/// The details of the current row: `tableLayoutPanel1`'s labels and bound boxes.
fn details(page: &DroneCan) -> AnyElement {
    let (dx, dy, dw, dh) = DETAILS_AT;
    let columns: Vec<f32> = DETAIL_COLUMNS
        .iter()
        .map(|percent| dw * percent / 100.0)
        .collect();
    let row_height = dh / 5.0;
    let current = page.current();
    let text = |value: &dyn Fn(&NodeRow) -> String| current.map(value).unwrap_or_default();
    // Each row: the label, then (column, span, text).
    let rows: [Vec<(usize, usize, String)>; 5] = [
        vec![
            (1, 1, text(&|row| row.id.to_string())),
            (2, 2, text(&|row| row.name.clone())),
        ],
        vec![
            (1, 1, text(&|row| row.mode.clone())),
            (2, 1, text(&|row| row.health.clone())),
            (3, 1, text(&|row| uptime_text(row.uptime))),
        ],
        vec![
            (1, 2, text(&|row| row.vsc.to_string())),
            (3, 1, String::new()),
        ],
        vec![
            (
                1,
                1,
                text(&|row| row.software_version.clone().unwrap_or_default()),
            ),
            (2, 2, text(&|row| format!("{:X}", row.software_crc))),
        ],
        vec![
            (
                1,
                1,
                text(&|row| row.hardware_version.clone().unwrap_or_default()),
            ),
            (
                2,
                2,
                text(&|row| row.hardware_uid.clone().unwrap_or_default()),
            ),
        ],
    ];
    let left = |column: usize| columns.iter().take(column).sum::<f32>();
    let mut table = crate::probe::measured("dronecan-details", at(dx, dy, dw, dh));
    for (index, (label_text, boxes)) in DETAIL_LABELS.iter().zip(rows).enumerate() {
        #[allow(clippy::cast_precision_loss)] // five rows
        let y = row_height * index as f32;
        let label_width = columns.first().copied().unwrap_or(0.0) - 6.0;
        table = table.child(
            at(3.0, y, label_width, row_height)
                .flex()
                .items_center()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(*label_text),
        );
        for (column, span, value) in boxes {
            let width: f32 = columns.iter().skip(column).take(span).sum::<f32>() - 6.0;
            let id = format!("dronecan-detail-{index}-{column}");
            table = table.child(
                crate::probe::measured(id, at(left(column) + 3.0, y + 3.0, width, 20.0))
                    .flex()
                    .items_center()
                    .px_1()
                    .border_1()
                    .border_color(rgb(theme::BORDER))
                    .bg(rgb(theme::BG))
                    .text_xs()
                    // Fitting the 20-high cell inside its border (the layout guard on the
                    // owner's Mac, 2026-10-05: 19 high in 18).
                    .line_height(px(16.0))
                    .overflow_hidden()
                    .whitespace_nowrap()
                    .text_color(rgb(theme::TEXT))
                    .child(value),
            );
        }
    }
    table.into_any_element()
}

/// `DGDebug`.
fn debug_grid(page: &DroneCan) -> AnyElement {
    let (x, y, w, h) = DEBUG_AT;
    let fixed: f32 = DEBUG_COLUMNS.iter().map(|(_, width)| width).sum();
    let text_width = (w - ROW_HEADER - fixed - 2.0).max(40.0);
    let mut header = div()
        .flex()
        .flex_shrink_0()
        .child(cell(ROW_HEADER, HEADER_HEIGHT, "", true));
    for (name, width) in DEBUG_COLUMNS {
        header = header.child(cell(width, HEADER_HEIGHT, name, true));
    }
    header = header.child(cell(text_width, HEADER_HEIGHT, DEBUG_TEXT, true));
    let mut rows = div()
        .id("dronecan-debug-rows")
        .flex()
        .flex_col()
        .flex_1()
        .overflow_y_scroll();
    for row in page.debug() {
        rows = rows.child(
            div()
                .flex()
                .flex_shrink_0()
                .child(cell(ROW_HEADER, ROW_HEIGHT, "", true))
                .child(cell(40.0, ROW_HEIGHT, row.node.to_string(), false))
                .child(cell(40.0, ROW_HEIGHT, row.level.to_string(), false))
                .child(cell(50.0, ROW_HEIGHT, row.source.clone(), false))
                .child(cell(text_width, ROW_HEIGHT, row.text.clone(), false)),
        );
    }
    crate::probe::measured("dronecan-debug", at(x, y, w, h))
        .flex()
        .flex_col()
        .overflow_hidden()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .child(header)
        .child(rows)
        .into_any_element()
}

/// The page, laid out as the Designer lays it out.
pub fn page(page: &DroneCan, cx: &mut Context<MissionPlanner>) -> AnyElement {
    if !page.is_active() {
        return div().into_any_element();
    }
    let (ix, iy, iw, ih) = INTERFACE_AT;
    let (nx, ny, nw, nh) = NETWORK_AT;
    let mut body = div()
        .relative()
        .w(px(PAGE_SIZE.0))
        .h(px(PAGE_SIZE.1))
        .child(heading(HEADING_AT.0, HEADING_AT.1, TITLE, true))
        .child(rule(RULE_AT.0, RULE_AT.1, RULE_AT.2))
        .child(combo_box(
            "dronecan-interface".to_owned(),
            &page.interface,
            INTERFACE_AT,
            |this| this.extra.dronecan.toggle_interface_list(),
            cx,
        ));
    if page.network_visible {
        body = body.child(combo_box(
            "dronecan-network".to_owned(),
            &page.network,
            NETWORK_AT,
            |this| this.extra.dronecan.toggle_network_list(),
            cx,
        ));
    }
    body = body
        .child(button(
            "dronecan-connect",
            page.connect_text(),
            CONNECT_AT,
            true,
            |this, _window, _cx| {
                let view = this.telemetry.view();
                this.extra
                    .dronecan
                    .click_connect(&view, &this.telemetry, Instant::now());
            },
            cx,
        ))
        .child(button(
            "dronecan-filter",
            FILTER,
            FILTER_AT,
            page.filter_enabled,
            |this, _window, _cx| this.extra.dronecan.click_filter(),
            cx,
        ))
        .child(button(
            "dronecan-stats",
            STATS,
            STATS_AT,
            true,
            |this, _window, _cx| this.extra.dronecan.click_stats(),
            cx,
        ))
        .child(button(
            "dronecan-inspector",
            INSPECTOR,
            INSPECTOR_AT,
            true,
            |this, _window, _cx| this.extra.dronecan.click_inspector(),
            cx,
        ))
        .child(page_check(
            "dronecan-checkupdate",
            CHECK_UPDATES,
            page.check_update,
            CHECK_UPDATES_AT,
            |this| this.extra.dronecan.toggle_check_update(),
            cx,
        ))
        .child(
            div()
                .absolute()
                .left(px(NOTE_AT.0))
                .top(px(NOTE_AT.1))
                .flex()
                .flex_col()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .children(
                    NOTE.lines()
                        .map(|line| div().child(line.trim_end().to_owned())),
                ),
        )
        .child(page_check(
            "dronecan-exitslcan",
            EXIT_SLCAN,
            page.exit_slcan,
            EXIT_SLCAN_AT,
            |this| this.extra.dronecan.toggle_exit_slcan(),
            cx,
        ))
        .child(page_check(
            "dronecan-log",
            LOG,
            page.log,
            LOG_AT,
            |this| this.extra.dronecan.toggle_log(),
            cx,
        ))
        .child(node_grid(page, cx))
        .child(details(page))
        .child(debug_grid(page));
    if page.interface_open {
        body = body.child(crate::config::servo_output::dropdown(
            "dronecan-interface-list",
            &page.interface,
            (ix, iy + ih, iw),
            |this, key| this.extra.dronecan.choose_interface(key),
            |this, lines| this.extra.dronecan.interface.scroll_list(lines),
            cx,
        ));
    }
    if page.network_open && page.network_visible {
        body = body.child(crate::config::servo_output::dropdown(
            "dronecan-network-list",
            &page.network,
            (nx, ny + nh, nw),
            |this, key| this.extra.dronecan.choose_network(key),
            |this, lines| this.extra.dronecan.network.scroll_list(lines),
            cx,
        ));
    }
    panel(TITLE, body).into_any_element()
}

/// The node menu, where it was opened.
fn node_menu(page: &DroneCan, cx: &mut Context<MissionPlanner>) -> Option<AnyElement> {
    let (_, (x, y)) = page.menu?;
    let checked = page.passthrough.as_ref();
    let mut list = crate::probe::measured("dronecan-menu", div())
        .id("dronecan-menu")
        .flex()
        .flex_col()
        .min_w(px(180.0))
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .occlude();
    for (index, (id, text)) in MENU.iter().enumerate() {
        let mark = match index {
            4 => checked.is_some_and(|p| p.tunnel.is_none()),
            5 => checked.is_some_and(|p| p.tunnel.is_some()),
            _ => false,
        };
        let item = crate::probe::measured(*id, div())
            .id(*id)
            .flex()
            .items_center()
            .gap_1()
            .h(px(MENU_ROW))
            .px_2()
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::ACTION)))
            .child(div().w(px(10.0)).child(if mark { "✓" } else { "" }))
            .child(*text)
            .on_click(cx.listener(move |this, event: &ClickEvent, _window, cx| {
                let page = &mut this.extra.dronecan;
                let now = Instant::now();
                match index {
                    0 => page.menu_parameters(now),
                    1 => page.menu_restart(),
                    2 => page.menu_update(false),
                    3 => page.menu_update(true),
                    4 => page.menu_passthrough(),
                    _ => page.menu_passthrough4(event.modifiers().control),
                }
                cx.notify();
            }));
        list = list.child(item);
    }
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(x), px(y)))
                .snap_to_window()
                .child(list),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

/// A form over the window: its caption with a close box, and its client area.
fn form(
    id: &'static str,
    close_id: &'static str,
    caption: String,
    client: Div,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let size = window.viewport_size();
    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(div().text_xs().text_color(rgb(theme::DIM)).child(caption))
        .child(action(
            close_id,
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.extra.dronecan.close_window();
                cx.notify();
            }),
        ));
    let frame = crate::probe::measured(id, div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .id(SharedString::from(format!("{id}-backdrop")))
                    .w(size.width)
                    .h(size.height)
                    .flex()
                    .items_center()
                    .justify_center()
                    .occlude()
                    .child(frame),
            ),
    )
    .with_priority(1)
    .into_any_element()
}

/// The parameter window.
fn params_form(
    params: &ParamsWindow,
    focus: Option<&FocusHandle>,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (gx, gy, gw, gh) = PARAMS_GRID_AT;
    let mut header = div().flex().flex_shrink_0();
    for (name, width) in PARAMS_COLUMNS {
        header = header.child(cell(width, HEADER_HEIGHT, name, true));
    }
    let mut rows = div()
        .id("dronecan-params-rows")
        .flex()
        .flex_col()
        .flex_1()
        .overflow_y_scroll();
    for (index, row) in params
        .rows()
        .iter()
        .enumerate()
        .filter(|(_, row)| row.visible)
    {
        let name = row.name.clone();
        let value_id = format!("dronecan-params-value-{index}");
        let shade = match row.shade {
            Shade::Normal => theme::BG,
            Shade::Changed => 0x00_80_00,
        };
        let value_cell = match (params.editing.as_ref(), focus) {
            (Some((edited, field)), Some(handle)) if *edited == row.name => {
                let focused = handle.is_focused(window);
                div()
                    .w(px(PARAMS_COLUMNS[1].1))
                    .h(px(ROW_HEIGHT))
                    .child(crate::textfield::text_field(
                        "dronecan-params-edit",
                        field,
                        handle,
                        focused,
                        px(PARAMS_COLUMNS[1].1),
                        cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                            if event.keystroke.modifiers.control && event.keystroke.key == "s" {
                                let again =
                                    this.persisted.get(WRITE_SHOW_AGAIN_KEY).map(str::to_owned);
                                this.extra.dronecan.press_write(again.as_deref());
                                cx.notify();
                                return;
                            }
                            if this
                                .extra
                                .dronecan
                                .params_window_mut()
                                .is_some_and(|window| window.edit_key(event))
                            {
                                cx.notify();
                            }
                        }),
                    ))
                    .into_any_element()
            }
            (Some((edited, field)), None) if *edited == row.name => cell(
                PARAMS_COLUMNS[1].1,
                ROW_HEIGHT,
                field.value().to_owned(),
                false,
            )
            .into_any_element(),
            _ => crate::probe::measured(
                value_id.clone(),
                cell(PARAMS_COLUMNS[1].1, ROW_HEIGHT, row.value.clone(), false).bg(rgb(shade)),
            )
            .id(SharedString::from(value_id))
            .cursor_text()
            .on_click(cx.listener(move |this, _event: &ClickEvent, window, cx| {
                let page = &mut this.extra.dronecan;
                if let Some(params) = page.params_window_mut() {
                    params.commit_edit();
                    params.begin_edit(&name);
                }
                this.dronecan_focus(window, cx);
                cx.notify();
            }))
            .into_any_element(),
        };
        let fav_name = row.name.clone();
        let fav_id = format!("dronecan-params-fav-{index}");
        let fav = crate::probe::measured(
            fav_id.clone(),
            cell(
                PARAMS_COLUMNS[5].1,
                ROW_HEIGHT,
                if row.fav { "☑" } else { "☐" },
                false,
            ),
        )
        .id(SharedString::from(fav_id))
        .justify_center()
        .cursor_pointer()
        .on_click(cx.listener(move |this, _event: &ClickEvent, _window, cx| {
            let setting = this.persisted.get("fav_params").map(str::to_owned);
            let saved = this
                .extra
                .dronecan
                .params_window_mut()
                .and_then(|params| params.click_fav(&fav_name, setting.as_deref()));
            if let Some(saved) = saved {
                this.persisted.set("fav_params", saved);
            }
            cx.notify();
        }));
        rows = rows.child(
            crate::probe::measured(format!("dronecan-params-row-{index}"), div())
                .flex()
                .flex_shrink_0()
                .child(cell(
                    PARAMS_COLUMNS[0].1,
                    ROW_HEIGHT,
                    row.name.clone(),
                    false,
                ))
                .child(value_cell)
                .child(cell(
                    PARAMS_COLUMNS[2].1,
                    ROW_HEIGHT,
                    row.min.clone(),
                    false,
                ))
                .child(cell(
                    PARAMS_COLUMNS[3].1,
                    ROW_HEIGHT,
                    row.max.clone(),
                    false,
                ))
                .child(cell(
                    PARAMS_COLUMNS[4].1,
                    ROW_HEIGHT,
                    row.default.clone(),
                    false,
                ))
                .child(fav),
        );
    }
    let grid = crate::probe::measured("dronecan-params-grid", at(gx, gy, gw, gh))
        .flex()
        .flex_col()
        .overflow_hidden()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .child(header)
        .child(rows);
    let enabled = !params.busy();
    let mut client = div()
        .relative()
        .w(px(PARAMS_SIZE.0))
        .h(px(PARAMS_SIZE.1))
        .child(grid);
    for (id, text, place) in PARAMS_BUTTONS {
        client = client.child(button(
            id,
            text,
            place,
            enabled,
            move |this, _window, _cx| {
                let page = &mut this.extra.dronecan;
                match id {
                    "dronecan-params-load" => page.press_file(false),
                    "dronecan-params-save" => page.press_file(true),
                    "dronecan-params-write" => {
                        let again = this.persisted.get(WRITE_SHOW_AGAIN_KEY).map(str::to_owned);
                        this.extra.dronecan.press_write(again.as_deref());
                    }
                    "dronecan-params-refresh" => page.press_refresh(),
                    "dronecan-params-compare" => page.status = Some(NOT_PORTED_COMPARE.to_owned()),
                    "dronecan-params-commit" => page.press_commit(),
                    _ => page.status = Some(NOT_PORTED_RESET.to_owned()),
                }
            },
            cx,
        ));
    }
    let search_focused =
        focus.is_some_and(|handle| handle.is_focused(window)) && params.editing.is_none();
    client = client
        .child(
            div()
                .absolute()
                .left(px(PARAMS_RAW_AT.0))
                .top(px(PARAMS_RAW_AT.1))
                .flex()
                .flex_col()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .children(PARAMS_RAW.lines().map(|line| div().child(line.to_owned()))),
        )
        .child(label(
            PARAMS_SEARCH_LABEL_AT.0,
            PARAMS_SEARCH_LABEL_AT.1,
            PARAMS_SEARCH,
            true,
        ));
    let (sx, sy, sw, sh) = PARAMS_SEARCH_AT;
    let search: AnyElement = match focus {
        Some(handle) if search_focused => div()
            .child(crate::textfield::text_field(
                "dronecan-params-search",
                &params.search,
                handle,
                true,
                px(sw),
                cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                    if event.keystroke.modifiers.control && event.keystroke.key == "s" {
                        let again = this.persisted.get(WRITE_SHOW_AGAIN_KEY).map(str::to_owned);
                        this.extra.dronecan.press_write(again.as_deref());
                        cx.notify();
                        return;
                    }
                    let now = Instant::now();
                    if this
                        .extra
                        .dronecan
                        .params_window_mut()
                        .is_some_and(|params| params.search_key(event, now))
                    {
                        cx.notify();
                    }
                }),
            ))
            .into_any_element(),
        _ => crate::probe::measured("dronecan-params-search", div())
            .id("dronecan-params-search")
            .size_full()
            .flex()
            .items_center()
            .px_1()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .bg(rgb(theme::ACTION))
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .cursor_text()
            .child(params.search.value().to_owned())
            .on_click(cx.listener(|this, _event: &ClickEvent, window, cx| {
                if let Some(params) = this.extra.dronecan.params_window_mut() {
                    params.commit_edit();
                }
                this.dronecan_focus(window, cx);
                cx.notify();
            }))
            .into_any_element(),
    };
    client = client
        .child(at(sx, sy, sw, sh).child(search))
        .child(page_check(
            "dronecan-params-modified",
            PARAMS_MODIFIED,
            params.modified,
            PARAMS_MODIFIED_AT,
            |this| {
                if let Some(params) = this.extra.dronecan.params_window_mut() {
                    params.toggle_modified();
                }
            },
            cx,
        ));
    form(
        "dronecan-params",
        "dronecan-params-close",
        params.caption(),
        client,
        window,
        cx,
    )
}

/// The Filter window: "ALL" and a box a type, two columns of 320.
fn filter_form(
    filter: &FilterWindow,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let mut all = Check::default();
    all.enabled = true;
    all.state = if filter.all {
        CheckState::Checked
    } else {
        CheckState::Unchecked
    };
    let mut flow = div()
        .id("dronecan-filter-list")
        .flex()
        .flex_wrap()
        .w(px(736.0))
        .h(px(600.0))
        .overflow_y_scroll()
        .child(div().w(px(320.0)).h(px(20.0)).relative().child(check_box(
            "dronecan-filter-all".to_owned(),
            &all,
            "ALL",
            (0.0, 0.0),
            |this| {
                let view = this.telemetry.view();
                this.extra
                    .dronecan
                    .click_filter_box(None, &this.telemetry, &view);
            },
            cx,
        )));
    for (name, id) in &filter.boxes {
        let id = *id;
        let mut check = Check::default();
        check.enabled = true;
        check.state = if filter.ticked(id) {
            CheckState::Checked
        } else {
            CheckState::Unchecked
        };
        flow = flow.child(div().w(px(320.0)).h(px(20.0)).relative().child(check_box(
            format!("dronecan-filter-{name}"),
            &check,
            name,
            (0.0, 0.0),
            move |this| {
                let view = this.telemetry.view();
                this.extra
                    .dronecan
                    .click_filter_box(Some(id), &this.telemetry, &view);
            },
            cx,
        )));
    }
    form(
        "dronecan-filter-form",
        "dronecan-filter-close",
        "DroneCAN Messages".to_owned(),
        div().p_2().child(flow),
        window,
        cx,
    )
}

/// A plain grid of text, its columns at their headers' widths.
fn text_grid(id: &'static str, columns: &[&str], rows: &[Vec<String>], height: f32) -> AnyElement {
    let width = |name: &str| (f32::from(u16::try_from(name.len()).unwrap_or(10)) * 7.0).max(60.0);
    let mut header = div().flex().flex_shrink_0();
    for name in columns {
        header = header.child(cell(width(name), HEADER_HEIGHT, (*name).to_owned(), true));
    }
    let mut body = div().flex().flex_col();
    for row in rows {
        let mut line = div().flex().flex_shrink_0();
        for (name, text) in columns.iter().zip(row) {
            line = line.child(cell(width(name), ROW_HEIGHT, text.clone(), false));
        }
        body = body.child(line);
    }
    crate::probe::measured(id, div())
        .id(id)
        .w_full()
        .h(px(height))
        .overflow_scroll()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .child(header)
        .child(body)
        .into_any_element()
}

/// The Stats window: the two grids.
fn stats_form(
    stats: &StatsWindow,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let size = window.viewport_size();
    let width = (f32::from(size.width) / 1.2).max(400.0);
    let height = (f32::from(size.height) / 1.2).max(300.0);
    let client = div()
        .flex()
        .flex_col()
        .w(px(width))
        .h(px(height))
        .child(text_grid(
            "dronecan-stats-canstats",
            &CAN_STATS_COLUMNS,
            &stats.can_stats_cells(),
            height / 2.0,
        ))
        .child(text_grid(
            "dronecan-stats-stats",
            &STATS_COLUMNS,
            &stats.stats_cells(),
            height / 2.0,
        ));
    form(
        "dronecan-stats-form",
        "dronecan-stats-close",
        STATS.to_owned(),
        client,
        window,
        cx,
    )
}

/// The boxes, questions, dialogs and windows the page has up, over the whole window.
pub fn overlay(
    page: &DroneCan,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let mut layers: Vec<AnyElement> = Vec::new();
    let focus = page.focus.as_ref();
    if page.active
        && let Some(menu) = node_menu(page, cx)
    {
        layers.push(menu);
    }
    match &page.window {
        Some(OpenWindow::Params(params)) => layers.push(params_form(params, focus, window, cx)),
        Some(OpenWindow::Filter(filter)) => layers.push(filter_form(filter, window, cx)),
        Some(OpenWindow::Stats(stats)) => layers.push(stats_form(stats, window, cx)),
        None => {}
    }
    if let Some((_, path)) = page.path.as_ref()
        && let Some(handle) = focus
    {
        let ids = crate::config::firmware::BoxIds {
            question: "dronecan-path-question",
            yes: "dronecan-path-yes",
            no: "dronecan-path-no",
            message: "dronecan-path-message",
            ok: "dronecan-path-message-ok",
            path: "dronecan-path",
            path_value: "dronecan-path-value",
            path_ok: "dronecan-path-ok",
            path_cancel: "dronecan-path-cancel",
        };
        layers.push(crate::config::firmware::path_box(
            ids,
            path,
            handle,
            window,
            |this, event| this.extra.dronecan.path_key(event, Instant::now()),
            |this, ok| this.extra.dronecan.answer_path(ok, Instant::now()),
            cx,
        ));
    }
    if let Some(input) = page.input.as_ref()
        && let Some(handle) = focus
    {
        layers.push(input_dialog(input, handle, window, cx));
    }
    if let Some(progress) = page.progress.as_ref() {
        let ids = crate::config::serial_ports::ProgressIds {
            frame: "dronecan-progress",
            bar: "dronecan-progress-bar",
            cancel: "dronecan-progress-cancel",
            backdrop: "dronecan-progress-backdrop",
        };
        layers.push(crate::config::serial_ports::progress_dialog(
            ids,
            &progress.text,
            progress.bar,
            progress.started,
            Some(|this: &mut MissionPlanner| this.extra.dronecan.cancel_progress()),
            window,
            cx,
        ));
    }
    if let Some(question) = page.question.as_ref() {
        layers.push(question_box(page, question, window, cx));
    } else if let Some(message) = page.params_window().and_then(ParamsWindow::message) {
        layers.push(crate::config::optional::message_box(
            "dronecan-params-message",
            "dronecan-params-message-ok",
            message,
            window,
            |this| this.extra.dronecan.dismiss_params_message(),
            cx,
        ));
    } else if let Some(message) = page.message() {
        layers.push(crate::config::optional::message_box(
            "dronecan-message",
            "dronecan-message-ok",
            message,
            window,
            |this| this.extra.dronecan.dismiss_message(),
            cx,
        ));
    }
    if layers.is_empty() {
        return None;
    }
    Some(div().children(layers).into_any_element())
}

/// A question: Yes/No, OK/Cancel, or a show-again box's OK.
fn question_box(
    page: &DroneCan,
    question: &Question,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let answer = |yes: bool| {
        move |this: &mut MissionPlanner,
              _event: &(),
              _window: &mut Window,
              cx: &mut Context<MissionPlanner>| {
            let url = this.dronecan_box_url();
            this.extra
                .dronecan
                .answer(yes, url, &mut this.persisted, Instant::now());
            cx.notify();
        }
    };
    let (yes_text, no_text) = match question.asked {
        Asked::SearchInternet { .. } => ("Yes", Some("No")),
        Asked::NewFirmware { .. } | Asked::WriteParams { .. } => ("OK", None),
        Asked::SlcanWithoutLink | Asked::RefreshParams => ("OK", Some("Cancel")),
    };
    let mut buttons = vec![action(
        "dronecan-question-yes",
        yes_text,
        theme::ACCENT,
        true,
        cx.listener(answer(true)),
    )];
    if let Some(no_text) = no_text {
        buttons.push(action(
            "dronecan-question-no",
            no_text,
            theme::DIM,
            true,
            cx.listener(answer(false)),
        ));
    }
    if let Some(again) = page.show_again() {
        let mut check = Check::default();
        check.enabled = true;
        check.state = if again {
            CheckState::Checked
        } else {
            CheckState::Unchecked
        };
        buttons.insert(
            0,
            div()
                .relative()
                .w(px(130.0))
                .h(px(20.0))
                .child(check_box(
                    "dronecan-question-again".to_owned(),
                    &check,
                    SHOW_ME_AGAIN,
                    (0.0, 0.0),
                    |this| this.extra.dronecan.toggle_show_again(),
                    cx,
                ))
                .into_any_element(),
        );
    }
    crate::config::servo_output::modal(
        "dronecan-question",
        question.caption,
        &question.text.replace("\r\n", "\n"),
        false,
        buttons,
        window,
    )
}

/// The `InputBox`: its title, the number, OK and Cancel.
fn input_dialog(
    input: &Input,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let focused = handle.is_focused(window);
    let buttons = vec![
        action(
            "dronecan-input-ok",
            "OK",
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.extra.dronecan.answer_input(true, Instant::now());
                cx.notify();
            }),
        ),
        action(
            "dronecan-input-cancel",
            "Cancel",
            theme::DIM,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.extra.dronecan.answer_input(false, Instant::now());
                cx.notify();
            }),
        ),
    ];
    let dialog = crate::probe::measured("dronecan-input", div())
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
                .child(input.title),
        )
        .child(crate::textfield::text_field(
            "dronecan-input-value",
            &input.field,
            handle,
            focused,
            px(310.0),
            cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                if this.extra.dronecan.input_key(event, Instant::now()) {
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
                    .id("dronecan-input-backdrop")
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::scripted::{VEHICLE, Vehicle, until};
    use mp_dronecan::dsdl::{
        HardwareVersion, LogMessage, NodeStatus, NumericValue, SoftwareVersion, Value,
    };
    use mp_dronecan::slcan;
    use mp_dronecan::transfer::{Outcome, Reassembler, package};
    use mp_link::ProtocolTimeouts;
    use mp_transport::Transport as _;
    use mp_transport::testing::Loopback;

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn param(name: &str, value: Value, min: NumericValue, max: NumericValue) -> GetSetRes {
        GetSetRes {
            default_value: value.clone(),
            value,
            max_value: max,
            min_value: min,
            name: name.as_bytes().to_vec(),
        }
    }

    /// A DroneCAN node on the vehicle's CAN bus: what the link forwards to the vehicle as
    /// `CAN_FRAME`s it reads, and its answers and broadcasts go back as the vehicle's `CAN_FRAME`s.
    struct Peer {
        id: u8,
        reassembler: Reassembler,
        params: Vec<GetSetRes>,
        transfer_id: u8,
        /// The requests it has had, in order.
        heard: Vec<CanMessage>,
        /// `CAN_FORWARD`s and filters the vehicle has had.
        forwards: usize,
        filters: Vec<mp_mavlink_dialects::all::CanFilterModify>,
    }

    impl Peer {
        fn new(id: u8) -> Self {
            Self {
                id,
                reassembler: Reassembler::new(),
                params: vec![
                    param(
                        "GPS_TYPE",
                        Value::Integer(1),
                        NumericValue::Integer(0),
                        NumericValue::Integer(5),
                    ),
                    param(
                        "LED_BRIGHTNESS",
                        Value::Real(0.5),
                        NumericValue::Real(0.0),
                        NumericValue::Real(1.0),
                    ),
                ],
                transfer_id: 0,
                heard: Vec::new(),
                forwards: 0,
                filters: Vec::new(),
            }
        }

        /// Sends a message from this node, as the vehicle forwards it.
        fn send(&mut self, vehicle: &mut Vehicle, to: u8, transfer_id: u8, message: &CanMessage) {
            for (frame, payload) in package(self.id, to, PRIORITY, transfer_id, message, false) {
                let line = slcan::format_frame(&frame, &payload, false);
                if let Some(message) = mavlink::message_of(&line, 255, 190, 1) {
                    vehicle.send(&message);
                }
            }
        }

        /// A broadcast.
        fn broadcast(&mut self, vehicle: &mut Vehicle, message: &CanMessage) {
            let id = self.transfer_id;
            self.transfer_id = self.transfer_id.wrapping_add(1);
            self.send(vehicle, 0, id, message);
        }

        /// What the link sent read, and each request to this node answered.
        fn serve(&mut self, vehicle: &mut Vehicle) {
            for message in vehicle.read() {
                let frame = match message {
                    MavMessage::CanFrame(frame) => frame,
                    MavMessage::CommandLong(command)
                        if command.command == mavlink::MAV_CMD_CAN_FORWARD =>
                    {
                        self.forwards += 1;
                        continue;
                    }
                    MavMessage::CanFilterModify(filter) => {
                        self.filters.push(filter);
                        continue;
                    }
                    _ => continue,
                };
                let line = mavlink::line_of(&frame);
                let slcan::Line::Frame(header, id, payload) = slcan::read_line(&line) else {
                    continue;
                };
                let Outcome::Complete(done) = self.reassembler.push(header, id, &payload) else {
                    continue;
                };
                let (header, message, transfer_id) = *done;
                if !header.is_service()
                    || !header.svc_is_request()
                    || header.svc_destination_node() != self.id
                {
                    continue;
                }
                self.heard.push(message.clone());
                let answer = match message {
                    CanMessage::GetNodeInfoReq => {
                        let mut unique_id = [0u8; 16];
                        unique_id[0] = 0xAB;
                        unique_id[15] = 0x01;
                        Some(CanMessage::GetNodeInfoRes(Box::new(GetNodeInfoRes {
                            status: NodeStatus::default(),
                            software_version: SoftwareVersion {
                                major: 1,
                                minor: 2,
                                optional_field_flags: 3,
                                vcs_commit: 0xABC,
                                image_crc: 0xDEAD_BEEF,
                            },
                            hardware_version: HardwareVersion {
                                major: 3,
                                minor: 1,
                                unique_id,
                                certificate_of_authenticity: Vec::new(),
                            },
                            name: b"com.cubepilot.here3".to_vec(),
                        })))
                    }
                    CanMessage::GetSetReq(req) if req.name.is_empty() => {
                        Some(CanMessage::GetSetRes(
                            self.params
                                .get(usize::from(req.index))
                                .cloned()
                                .unwrap_or_default(),
                        ))
                    }
                    CanMessage::GetSetReq(req) => self
                        .params
                        .iter_mut()
                        .find(|p| p.name == req.name)
                        .map(|held| {
                            if req.value != Value::Empty {
                                held.value = req.value.clone();
                            }
                            CanMessage::GetSetRes(held.clone())
                        }),
                    CanMessage::ExecuteOpcodeReq { .. } => Some(CanMessage::ExecuteOpcodeRes {
                        argument: 0,
                        ok: true,
                    }),
                    CanMessage::RestartNodeReq { .. } => {
                        Some(CanMessage::RestartNodeRes { ok: true })
                    }
                    _ => None,
                };
                if let Some(answer) = answer {
                    self.send(vehicle, header.source_node(), transfer_id, &answer);
                }
            }
        }
    }

    /// Runs the page against the vehicle and the peer, 100 ms of the page's time a step, until
    /// `done`; the peer's status every second.
    fn run(
        page: &mut DroneCan,
        telemetry: &Telemetry,
        vehicle: &mut Vehicle,
        peer: &mut Peer,
        now: &mut Instant,
        what: &str,
        mut done: impl FnMut(&DroneCan, &Peer) -> bool,
    ) {
        for step in 0..600 {
            let view = telemetry.view();
            page.tick(telemetry, &view, true, *now, 1, None);
            if step % 10 == 0 {
                peer.broadcast(
                    vehicle,
                    &CanMessage::NodeStatus(NodeStatus {
                        uptime_sec: 60,
                        health: HEALTH_WARNING,
                        mode: MODE_OPERATIONAL,
                        sub_mode: 0,
                        vendor_specific_status_code: 7,
                    }),
                );
            }
            wasm_thread::sleep(Duration::from_millis(3));
            peer.serve(vehicle);
            if done(page, peer) {
                return;
            }
            *now += Duration::from_millis(100);
        }
        panic!("timed out waiting for {what}");
    }

    /// The Designer's words and places, read from the tree when it is here.
    #[test]
    fn the_text_is_the_designers() {
        let Some(designer) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigDroneCAN.Designer.cs",
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        for (control, text) in [
            ("label6", TITLE),
            ("but_connect", CONNECT),
            ("but_filter", FILTER),
            ("but_stats", STATS),
            ("but_uavcaninspector", INSPECTOR),
            ("CHK_checkupdate", CHECK_UPDATES),
            ("chk_canonclose", EXIT_SLCAN),
            ("chk_log", LOG),
            ("label2", DETAIL_LABELS[0]),
            ("label3", DETAIL_LABELS[1]),
            ("label4", DETAIL_LABELS[2]),
            ("label5", DETAIL_LABELS[3]),
            ("label7", DETAIL_LABELS[4]),
        ] {
            assert!(
                designer.contains(&format!("this.{control}.Text = \"{text}\";")),
                "{control}"
            );
        }
        for (_, text) in MENU {
            assert!(designer.contains(&format!(".Text = \"{text}\";")), "{text}");
        }
        for (name, width) in COLUMNS {
            assert!(
                designer.contains(&format!("HeaderText = \"{name}\";")),
                "{name}"
            );
            #[allow(clippy::cast_possible_truncation)]
            let width = width as i32;
            assert!(
                designer.contains(&format!(".Width = {width};")),
                "{name} {width}"
            );
        }
        assert!(
            designer.contains("this.cmb_interfacetype.Location = new System.Drawing.Point(7, 37);")
        );
        assert!(
            designer.contains("this.myDataGridView1.Location = new System.Drawing.Point(7, 89);")
        );
        assert!(designer.contains("this.DGDebug.Location = new System.Drawing.Point(7, 465);"));
        assert!(designer.contains("this.Size = new System.Drawing.Size(798, 612);"));
        assert!(designer.contains(
            "\"After enabling SLCAN, you will no longer be able to connect via MAVLINK.\\r\\nYou \
             mus\" +"
        ));
        let Some(cs) =
            crate::config_coverage::source::csharp("GCSViews/ConfigurationView/ConfigDroneCAN.cs")
        else {
            return;
        };
        for text in [
            NOT_CONNECTED_SLCAN,
            SLCAN_CAPTION,
            TRYING_TO_CONNECT,
            NO_INTERFACES,
            SEARCH_INTERNET,
            DOWNLOAD_FW,
            FILE_SEND_COMPLETE,
            ENTER_TCP_PORT,
            ENTER_BAUD,
            STOP,
            DISABLED_FORWARDING,
            NEW_FIRMWARE,
            UPDATE_CHECK_KEY,
        ] {
            assert!(cs.contains(&format!("\"{text}\"")), "{text}");
        }
        assert!(cs.contains("\"Reboot required\" + \" after setting CPORT. Please reboot!\""));
        for name in INTERFACES {
            assert!(cs.contains(&format!("            {name},")), "{name}");
        }
    }

    /// The words, and the uptime as `TimeSpan` writes it.
    #[test]
    fn words_and_uptimes() {
        assert_eq!(uptime_text(0), "00:00:00");
        assert_eq!(uptime_text(3_725), "01:02:05");
        assert_eq!(uptime_text(90_061), "1.01:01:01");
        assert_eq!(health_text(HEALTH_CRITICAL), Some("CRITICAL"));
        assert_eq!(health_text(9), None);
        assert_eq!(mode_text(MODE_OFFLINE), Some("OFFLINE"));
        assert_eq!(mode_text(5), None);
        assert_eq!(hw_version_url_text("1.0"), "1.0");
        assert_eq!(hw_version_url_text("2.10"), "2.1");
        assert_eq!(hw_version_url_text("1.25"), "1.25");
        assert_eq!(
            ConnectionType::from_index(3),
            Some(ConnectionType::McastCan(0))
        );
        assert!(ConnectionType::McastCan(1).shows_networks());
        assert!(!ConnectionType::MavlinkCan(2).shows_networks());
        assert_eq!(ConnectionType::MavlinkCan(2).name(), "MAVLinkCAN2");
    }

    /// The whole MAVLink path through the real link: `CAN_FORWARD` every second from a second
    /// in; our node heard by itself and named; a node on the vehicle's bus listed - its numbers,
    /// then its words, then its info - and its debug message; its parameters read into the window,
    /// one changed and written, then saved; and a restart.
    #[test]
    fn the_mavlink_bus_lists_nodes_and_reads_parameters() {
        let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default());
        let mut page = DroneCan::default();
        let view = telemetry.view();
        page.activate(Key::of(&view), None, None);
        assert_eq!(page.interface(), ConnectionType::Slcan);
        page.choose_interface(1);
        assert_eq!(page.interface(), ConnectionType::MavlinkCan(1));
        let mut now = Instant::now();
        page.click_connect(&view, &telemetry, now);
        assert_eq!(page.connect_text(), DISCONNECT);
        assert!(!page.lists_enabled && page.filter_enabled);
        let mut peer = Peer::new(10);

        run(
            &mut page,
            &telemetry,
            &mut vehicle,
            &mut peer,
            &mut now,
            "CAN_FORWARD",
            |_, peer| peer.forwards >= 2,
        );
        assert!(page.can_run, "the forwarding loop runs");

        run(
            &mut page,
            &telemetry,
            &mut vehicle,
            &mut peer,
            &mut now,
            "the nodes named",
            |page, _| {
                let named = |id: u8, name: &str| {
                    page.nodes()
                        .iter()
                        .any(|row| row.id == id && row.name == name)
                };
                named(127, "org.missionplanner") && named(10, "com.cubepilot.here3")
            },
        );
        let here3 = page
            .nodes()
            .iter()
            .find(|row| row.id == 10)
            .cloned()
            .expect("node 10");
        assert_eq!(here3.health, "WARNING");
        assert_eq!(here3.mode, "OPERATIONAL");
        assert_eq!(here3.uptime, 60);
        assert_eq!(here3.hardware_version.as_deref(), Some("3.1"));
        assert_eq!(here3.software_version.as_deref(), Some("1.2.ABC"));
        assert_eq!(here3.software_crc, 0xDEAD_BEEF);
        assert_eq!(
            here3.hardware_uid.as_deref(),
            Some("AB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01")
        );
        assert!(
            peer.heard
                .iter()
                .any(|m| matches!(m, CanMessage::GetNodeInfoReq))
        );

        // The node's debug message, at the top of the grid.
        peer.broadcast(
            &mut vehicle,
            &CanMessage::LogMessage(LogMessage {
                level: 2,
                source: b"GPS".to_vec(),
                text: b"no fix yet".to_vec(),
            }),
        );
        run(
            &mut page,
            &telemetry,
            &mut vehicle,
            &mut peer,
            &mut now,
            "the debug row",
            |page, _| page.debug().next().is_some(),
        );
        assert_eq!(
            page.debug().next(),
            Some(&DebugRow {
                node: 10,
                level: 2,
                source: "GPS".to_owned(),
                text: "no fix yet".to_owned(),
            })
        );

        // Parameters: "Getting Params", then the window.
        let row = page
            .nodes()
            .iter()
            .position(|row| row.id == 10)
            .expect("row");
        page.click_menu_cell(row, (10.0, 10.0));
        assert!(page.menu.is_some());
        page.menu_parameters(now);
        assert!(page.menu.is_none());
        assert_eq!(
            page.progress.as_ref().map(|p| p.text.as_str()),
            Some(GETTING_PARAMS)
        );
        run(
            &mut page,
            &telemetry,
            &mut vehicle,
            &mut peer,
            &mut now,
            "the window",
            |page, _| page.params_window().is_some(),
        );
        assert!(page.progress.is_none());
        let window = page.params_window().expect("open");
        assert_eq!(window.caption(), "UAVCAN Params - 10");
        let names: Vec<&str> = window.rows().iter().map(|row| row.name.as_str()).collect();
        assert_eq!(names, ["GPS_TYPE", "LED_BRIGHTNESS"]);
        assert_eq!(window.rows()[1].value, "0.5");
        assert_eq!(window.rows()[1].max, "1");

        // GPS_TYPE to 3, written - asked without its question - and saved.
        let window = page.params_window_mut().expect("open");
        window.begin_edit("GPS_TYPE");
        if let Some((_, field)) = window.editing.as_mut() {
            field.set("3");
        }
        window.commit_edit();
        assert_eq!(window.changes().len(), 1);
        assert_eq!(window.rows()[0].shade, Shade::Changed);
        page.press_write(Some("False"));
        run(
            &mut page,
            &telemetry,
            &mut vehicle,
            &mut peer,
            &mut now,
            "the writes",
            |page, _| {
                page.params_window()
                    .is_some_and(|window| window.message().is_some())
            },
        );
        let window = page.params_window().expect("open");
        assert_eq!(
            window.message().map(|m| m.text.as_str()),
            Some(PARAMS_SAVED)
        );
        assert!(window.changes().is_empty());
        assert_eq!(window.rows()[0].shade, Shade::Normal);
        assert_eq!(peer.params[0].value, Value::Integer(3));
        assert!(
            peer.heard
                .iter()
                .any(|m| matches!(m, CanMessage::ExecuteOpcodeReq { opcode: 0, .. }))
        );
        page.dismiss_params_message();
        page.close_window();

        // Restart.
        page.click_menu_cell(row, (10.0, 10.0));
        page.menu_restart();
        run(
            &mut page,
            &telemetry,
            &mut vehicle,
            &mut peer,
            &mut now,
            "the restart",
            |page, peer| {
                page.restarts.is_empty()
                    && peer
                        .heard
                        .iter()
                        .any(|m| matches!(m, CanMessage::RestartNodeReq { .. }))
            },
        );

        // Filter: the ALL box replaces the vehicle's filter with none; a type's box sends the
        // list without it.
        page.click_filter();
        let view = telemetry.view();
        page.click_filter_box(None, &telemetry, &view);
        page.click_filter_box(Some(341), &telemetry, &view);
        run(
            &mut page,
            &telemetry,
            &mut vehicle,
            &mut peer,
            &mut now,
            "the filters",
            |_, peer| peer.filters.len() == 2,
        );
        assert_eq!(peer.filters[0].num_ids, 0);
        assert_eq!(peer.filters[0].bus, 1, "BusInUse, as it is");
        assert_eq!(peer.filters[1].num_ids, 10);
        assert!(!peer.filters[1].ids.contains(&341));

        // Disconnect: the bus stopped, the rows kept.
        let view = telemetry.view();
        page.click_connect(&view, &telemetry, now);
        assert_eq!(page.connect_text(), CONNECT);
        assert!(page.bus.is_none());
        assert!(page.node().is_none());
        assert_eq!(page.nodes().len(), 2);
        assert_eq!(VEHICLE, VehicleId::new(1, 1));
    }

    /// SLCAN with no link asks first; OK with nothing to open is "Check port settings or Port in
    /// use?" on the status line, and the page stays connected to nothing, as the C#'s does.
    #[test]
    fn slcan_without_a_link_asks_and_then_says_why() {
        let telemetry = Telemetry::idle();
        let view = telemetry.view();
        let mut page = DroneCan::default();
        page.activate(key(), None, None);
        page.click_connect(&view, &telemetry, Instant::now());
        assert_eq!(page.question(), Some((SLCAN_CAPTION, NOT_CONNECTED_SLCAN)));
        assert!(!page.filter_enabled);
        let mut settings = crate::settings::Persisted::at(None);
        page.answer(true, None, &mut settings, Instant::now());
        assert_eq!(page.take_status().as_deref(), Some(CHECK_PORT));
        assert_eq!(page.connect_text(), DISCONNECT);
        // A port that will not open says the same.
        page.open_slcan("serial:/dev/nonexistent-dronecan:115200", Instant::now());
        assert_eq!(page.take_status().as_deref(), Some(CHECK_PORT));
    }

    /// The adapter's thread: the cleanup, the six commands each answered, then frames both ways,
    /// and the close command at the end.
    #[test]
    fn the_slcan_thread_opens_the_adapter() {
        let (ours, mut adapter) = Loopback::pair();
        let (commands, command_rx) = mpsc::channel();
        let (event_tx, events) = mpsc::channel();
        let thread = wasm_thread::spawn(move || run_slcan(Box::new(ours), &command_rx, &event_tx));
        let mut heard = Vec::new();
        let mut buf = [0u8; 256];
        // Each command answered with a carriage return as it comes.
        let mut answered = 0;
        until("the six commands", || {
            let n = adapter.read(&mut buf).unwrap_or(0);
            for byte in &buf[..n] {
                heard.push(*byte);
                if *byte == b'\r' && heard.len() > 3 {
                    answered += 1;
                    let _ = adapter.write_all(b"\r");
                }
            }
            answered >= 6
        });
        assert_eq!(&heard[..3], b"\r\r\r");
        assert_eq!(&heard[3..], b"C\rS\x08\rN\rV\rO\rF\r");
        let mut opened = false;
        until("opened", || {
            while let Ok(event) = events.try_recv() {
                opened |= matches!(event, SlcanEvent::Opened);
            }
            opened
        });
        adapter.write_all(b"T1E01550A80000000000000C0\r").unwrap();
        let mut line = None;
        until("the line", || {
            if let Ok(SlcanEvent::Line(text)) = events.try_recv() {
                line = Some(text);
            }
            line.is_some()
        });
        assert_eq!(line.as_deref(), Some("T1E01550A80000000000000C0\r"));
        commands
            .send(SlcanCommand::Write("T00000000100\r".to_owned()))
            .unwrap();
        commands.send(SlcanCommand::Stop(true)).unwrap();
        thread.join().unwrap();
        let mut rest = Vec::new();
        loop {
            let n = adapter.read(&mut buf).unwrap_or(0);
            if n == 0 {
                break;
            }
            rest.extend_from_slice(&buf[..n]);
        }
        assert_eq!(rest, b"T00000000100\rC\r");
    }

    /// The parameter window: favourites first then by name; an edit outside min and max refused
    /// with "Invalid value"; `RCn_REV` 0 made -1; the filter; Load from file.
    #[test]
    fn the_parameter_window() {
        let list = vec![
            param(
                "ZED",
                Value::Integer(1),
                NumericValue::Integer(0),
                NumericValue::Integer(10),
            ),
            param(
                "RC1_REV",
                Value::Integer(1),
                NumericValue::Empty,
                NumericValue::Empty,
            ),
            param(
                "ALPHA",
                Value::String(b"text".to_vec()),
                NumericValue::Empty,
                NumericValue::Empty,
            ),
            param(
                "BETA",
                Value::Empty,
                NumericValue::Empty,
                NumericValue::Empty,
            ),
        ];
        let mut window = ParamsWindow::new(10, list, &["ZED".to_owned()]);
        let names: Vec<&str> = window.rows().iter().map(|row| row.name.as_str()).collect();
        assert_eq!(names, ["ZED", "ALPHA", "BETA", "RC1_REV"]);
        assert_eq!(window.rows()[2].value, "Empty");
        assert_eq!(window.rows()[2].min, "");

        window.begin_edit("ZED");
        window.editing.as_mut().unwrap().1.set("11");
        window.commit_edit();
        assert_eq!(
            window.message().map(|m| m.text.as_str()),
            Some("Invalid value \"11\"")
        );
        assert!(window.changes().is_empty());
        assert!(in_range("0", "10", "10"));
        assert!(!in_range("0", "10", "x"));
        assert!(in_range("", "", "anything"));

        window.begin_edit("RC1_REV");
        window.editing.as_mut().unwrap().1.set("0");
        window.commit_edit();
        assert_eq!(window.rows()[3].value, "-1");
        assert_eq!(window.changes().get("RC1_REV"), Some(&Change::Number(-1.0)));

        window.begin_edit("ALPHA");
        window.editing.as_mut().unwrap().1.set("words");
        window.commit_edit();
        assert_eq!(
            window.changes().get("ALPHA"),
            Some(&Change::Text("words".to_owned()))
        );

        // Any cell: "al" is in ALPHA, and in the Fav cells' "False" - the C#'s matches them too.
        window.search.set("al*");
        window.filter();
        let shown: Vec<&str> = window
            .rows()
            .iter()
            .filter(|row| row.visible)
            .map(|row| row.name.as_str())
            .collect();
        assert_eq!(shown, ["ALPHA", "BETA", "RC1_REV"]);
        window.search.set("lph");
        window.filter();
        let shown: Vec<&str> = window
            .rows()
            .iter()
            .filter(|row| row.visible)
            .map(|row| row.name.as_str())
            .collect();
        assert_eq!(shown, ["ALPHA"]);
        window.search.set("");
        window.toggle_modified();
        let shown: Vec<&str> = window
            .rows()
            .iter()
            .filter(|row| row.visible)
            .map(|row| row.name.as_str())
            .collect();
        assert_eq!(shown, ["ALPHA", "RC1_REV"]);

        let file = mp_params::param_file::ParamFile::from_values([
            ("ZED".to_owned(), 4.0),
            ("FORMAT_VERSION".to_owned(), 1.0),
            ("MISSING".to_owned(), 1.0),
        ]);
        window.load(&file);
        assert_eq!(window.rows()[0].value, "4");
        assert_eq!(window.changes().get("ZED"), Some(&Change::Number(4.0)));

        // Fav: the setting appended to, the rows sorted again.
        let saved = window.click_fav("RC1_REV", Some("ZED"));
        assert_eq!(saved.as_deref(), Some("ZED;RC1_REV"));
        let names: Vec<&str> = window.rows().iter().map(|row| row.name.as_str()).collect();
        assert_eq!(
            names,
            ["RC1_REV", "ZED", "ALPHA", "BETA"],
            "both favourites first, by name"
        );
    }

    /// Inspector and Stats: the status line, and the window fed what the node hears.
    #[test]
    fn inspector_and_stats() {
        let mut page = DroneCan::default();
        page.activate(key(), None, None);
        page.click_inspector();
        assert_eq!(page.take_status().as_deref(), Some(PLEASE_CONNECT));
        assert!(page.inspector.is_none());
        // Connected: `new DroneCANInspector(can).Show()`.
        page.connected = true;
        page.click_inspector();
        assert!(page.inspector.is_some());
        assert_eq!(page.inspector_opened, 1);
        page.close_inspector();
        assert!(page.inspector.is_none());
        page.connected = false;
        page.click_stats();
        assert!(matches!(page.window, Some(OpenWindow::Stats(_))));
        let mut node = Node::new(Identity::default(), Instant::now());
        node.start(Instant::now());
        let stats = dsdl::CanStats {
            interface: 1,
            rx_received: 99,
            ..dsdl::CanStats::default()
        };
        for (frame, payload) in package(12, 0, PRIORITY, 0, &CanMessage::CanStats(stats), false) {
            node.receive_line(&slcan::format_frame(&frame, &payload, false));
        }
        let received = node.next_received().expect("heard");
        if let Some(OpenWindow::Stats(window)) = page.window.as_mut() {
            window.heard(&received);
            window.heard(&received);
            let cells = window.can_stats_cells();
            assert_eq!(cells.len(), 1, "the node's row replaced");
            assert_eq!(cells[0][0], "12");
            assert_eq!(cells[0][8], "99");
        }
        page.close_window();
        page.disconnect();
        page.click_inspector();
        assert_eq!(page.take_status().as_deref(), Some(PLEASE_CONNECT));
        page.click_stats();
        assert!(page.window.is_none(), "no node: the C# throws");
    }

    /// Update: no info yet is the C#'s exception on the status line; with it, the question, and
    /// No asks for a file.
    #[test]
    fn update_asks_then_wants_a_file() {
        let mut page = DroneCan::default();
        page.activate(key(), None, None);
        page.nodes.push(NodeRow {
            id: 10,
            name: "?".to_owned(),
            mode: String::new(),
            health: String::new(),
            uptime: 0,
            hardware_version: None,
            software_version: None,
            software_crc: 0,
            hardware_uid: None,
            vsc: 0,
        });
        page.select_row(0);
        page.menu_update(false);
        assert!(page.take_status().is_some());
        assert!(page.question.is_none());
        let now = Instant::now();
        let node = page.can.as_mut().expect("node");
        node.start(now);
        for (frame, payload) in package(
            10,
            127,
            PRIORITY,
            0,
            &CanMessage::GetNodeInfoRes(Box::default()),
            false,
        ) {
            node.receive_line(&slcan::format_frame(&frame, &payload, false));
        }
        page.menu_update(true);
        assert_eq!(page.question(), Some((UPDATE_CAPTION, SEARCH_INTERNET)));
        let mut settings = crate::settings::Persisted::at(None);
        page.answer(false, None, &mut settings, now);
        assert!(matches!(
            page.path,
            Some((PathFor::Firmware { node: 10 }, _))
        ));
        page.answer_path(false, now);
        assert!(page.path.is_none());
    }

    /// The Here3+/4's baud packet: two 0x55, then UBX CFG-PRT with the baud in it and its
    /// Fletcher checksum.
    #[test]
    fn the_here4_baud_packet() {
        let packet = here4_baud_packet(230_400);
        assert_eq!(&packet[..8], &[0x55, 0x55, 0xB5, 0x62, 0x06, 0x00, 20, 0]);
        assert_eq!(&packet[8 + 8..8 + 11], &230_400u32.to_le_bytes()[..3]);
        let (mut a, mut b) = (0u8, 0u8);
        for byte in &packet[4..packet.len() - 2] {
            a = a.wrapping_add(*byte);
            b = b.wrapping_add(a);
        }
        assert_eq!(&packet[packet.len() - 2..], &[a, b]);
        assert_eq!(packet.len(), 2 + 8 + 20);
    }

    /// The Filter window: every type, by name ignoring case, the page's ticked.
    #[test]
    fn the_filter_window() {
        let filter = FilterWindow::new();
        assert_eq!(filter.boxes.len(), 155);
        let lower: Vec<String> = filter
            .boxes
            .iter()
            .map(|(name, _)| name.to_lowercase())
            .collect();
        let mut sorted = lower.clone();
        sorted.sort();
        assert_eq!(lower, sorted);
        assert!(filter.ticked(341) && filter.ticked(16383) && filter.ticked(0));
        assert!(!filter.ticked(1062));
    }

    /// Leaving the page is `Disconnect`; leaving the screen lets the page object go.
    #[test]
    fn leaving_disconnects() {
        let (telemetry, _vehicle) = Vehicle::connect(ProtocolTimeouts::default());
        let mut page = DroneCan::default();
        let view = telemetry.view();
        page.activate(Key::of(&view), None, None);
        page.choose_interface(2);
        page.click_connect(&view, &telemetry, Instant::now());
        assert_eq!(page.bus.as_ref().map(Bus::kind), Some("MAVLinkCAN2"));
        page.deactivate();
        assert!(page.bus.is_none());
        assert_eq!(page.connect_text(), CONNECT);
        page.tick(&telemetry, &view, false, Instant::now(), 1, None);
        assert!(page.made_for.is_none(), "let go with its screen");
    }

    /// SLCAN with the link open: `CAN_SLCAN_CPORT` set to 1 first. From 0, "Reboot required"
    /// and nothing more; from 1, `CAN_SLCAN_TIMOUT` 2, `CAN_P1_DRIVER` 1, `CAN_SLCAN_SERNUM` 0 sent
    /// twice without waiting, and the link's port is the one to take.
    #[test]
    fn slcan_with_a_link_sets_the_port_then_takes_it() {
        const NAMES: [&str; 4] = [
            "CAN_SLCAN_CPORT",
            "CAN_SLCAN_TIMOUT",
            "CAN_P1_DRIVER",
            "CAN_SLCAN_SERNUM",
        ];
        // The vehicle's PARAM_VALUE for one of its four.
        let value = |name: &str, value: f32| {
            let index = NAMES.iter().position(|held| *held == name).unwrap_or(0);
            MavMessage::ParamValue(mp_mavlink_dialects::all::ParamValue {
                param_value: value,
                param_count: 4,
                param_index: u16::try_from(index).unwrap(),
                param_id: mp_params::encode_param_id(name),
                param_type: crate::telemetry::scripted::INT32,
            })
        };
        for old in [0.0_f32, 1.0] {
            let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default());
            for name in NAMES {
                let start = if name == "CAN_SLCAN_CPORT" { old } else { 0.0 };
                vehicle.send(&value(name, start));
            }
            until("the parameters", || telemetry.view().parameters.len() == 4);
            let mut page = DroneCan::default();
            let view = telemetry.view();
            page.activate(Key::of(&view), None, None);
            page.click_connect(&view, &telemetry, Instant::now());
            assert!(page.question().is_none(), "no question with the link open");
            let mut sets: Vec<(String, f32)> = Vec::new();
            let mut blind = 0;
            until("the writes", || {
                let view = telemetry.view();
                page.tick(&telemetry, &view, true, Instant::now(), 1, None);
                for message in vehicle.read() {
                    if let MavMessage::ParamSet(set) = message {
                        let name = mp_params::decode_param_id(&set.param_id);
                        if name == "CAN_SLCAN_SERNUM" {
                            blind += 1;
                            continue;
                        }
                        sets.push((name.clone(), set.param_value));
                        vehicle.send(&value(&name, set.param_value));
                    }
                }
                page.take_port.is_some() || page.message().is_some()
            });
            if old == 0.0 {
                assert_eq!(
                    page.message().map(|m| m.text.as_str()),
                    Some(REBOOT_REQUIRED)
                );
                assert!(page.take_port.is_none());
                assert_eq!(sets, [("CAN_SLCAN_CPORT".to_owned(), 1.0)]);
            } else {
                assert_eq!(page.take_port_request(), Some(view.target.clone()));
                let names: Vec<&str> = sets.iter().map(|(name, _)| name.as_str()).collect();
                assert_eq!(
                    names,
                    ["CAN_SLCAN_CPORT", "CAN_SLCAN_TIMOUT", "CAN_P1_DRIVER"]
                );
                assert_eq!(sets[1].1, 2.0);
                until("the blind sends", || {
                    for message in vehicle.read() {
                        if matches!(message, MavMessage::ParamSet(_)) {
                            blind += 1;
                        }
                    }
                    blind >= 2
                });
                assert_eq!(blind, 2);
            }
        }
    }

    /// Every fact the GUI script asserts on is one this page records, and every control it
    /// clicks is one this page draws.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-dronecan.gui");
        let source = concat!(
            include_str!("dronecan.rs"),
            include_str!("dronecan_inspector.rs")
        );
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.dronecan.") => {
                    // `node.<id>.<field>` is made with its id.
                    let generic = key
                        .strip_prefix("config.dronecan.node.")
                        .and_then(|rest| rest.split_once('.'))
                        .filter(|(id, _)| id.parse::<u8>().is_ok())
                        .map(|(_, field)| format!("{{key}}.{field}"));
                    let recorded = match generic {
                        Some(field) => source.contains(&format!("format!(\"{field}\")")),
                        None => source.contains(&format!("\"{key}\"")),
                    };
                    assert!(recorded, "{key} is not recorded");
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("dronecan-") => {
                    let drawn = if let Some(rest) = id.strip_prefix("dronecan-row-") {
                        let (_, menu) = rest.split_once('-').unwrap_or((rest, ""));
                        if menu.is_empty() {
                            source.contains("format!(\"dronecan-row-{}\", row.id)")
                        } else {
                            source.contains("format!(\"dronecan-row-{}-menu\", row.id)")
                        }
                    } else if let Some(index) = id.strip_prefix("dronecan-interface-list-") {
                        index
                            .parse::<usize>()
                            .is_ok_and(|index| index < INTERFACES.len())
                            && source.contains("\"dronecan-interface-list\"")
                    } else {
                        source.contains(&format!("\"{id}\""))
                    };
                    assert!(drawn, "{id} is not drawn");
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 30 && clicks > 10, "{facts} facts, {clicks} clicks");
    }
}
