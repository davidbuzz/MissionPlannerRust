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

//! The Cursor-on-Target output: `Controls/SerialOutputCoT.cs`, EXPERIMENTAL's CoT button (`new
//! SerialOutputCoT().Show()`, `temp.cs:1367-1370`). Every vehicle's position as a Cursor-on-Target
//! event - the XML a TAK client reads - written to a port every few seconds and shown in the box.
//!
//! What it does:
//!
//! * the constructor lists "TAK Multicast", the TCP and UDP hosts on 14551, the TCP and UDP
//!   clients, then the serial ports (`:27-38`); `Load` (`:367-396`) fills the UID grid from the
//!   `CoTUID` setting, a JSON array of rows, each the cells' values (`ExtensionsMP.cs:167-189`),
//!   and the interval, the port's and baud's indexes, Additional Details and Nice Formatting
//!   from theirs, enables the baud only for a port whose name has "com" in it, and, with the
//!   thread already running, stops and starts it again; `FormClosing` (`:398-405`) saves those
//!   five, and a row validated saves the grid (`:407-410`);
//! * Connect (`BUT_connect_Click`, `:40-140`): running, the thread is stopped and the stream
//!   closed; else the stream the port names is made - TAK Multicast a UDP client to 239.2.3.1:6969
//!   under `ConfigRef` `TAK_Multicast`, opened at once (its address saved as the client's
//!   settings); a TCP host listening on 14551; a TCP client under `SerialOutputCoTTCP`, connecting
//!   again whenever it drops; a UDP host or client; or the serial port - "TAK IP Port in use. Is
//!   another app on this system already using it?" when TAK's address is taken, else
//!   `Strings.InvalidPortName`; the baud `int.Parse`d (`Strings.InvalidBaudRate`); the stream
//!   opened off the form's thread, and the thread started at once;
//! * the thread (`mainloop`, `:156-193`), until stopped: for each vehicle on the link, its event
//!   ([`event_xml`]) - its UID from the grid's row for its sysid, "NOsysid<n>" when none
//!   (`FindUIDviaSysid`, `:248-256`), the Type box's text, `how` "m-g", `cs.lat`, `cs.lng`,
//!   `cs.altasl`, `cs.groundcourse` and `cs.groundspeed` as the C# reads them, in the user's units
//!   (`:213-234`), and with Additional Details the UID's row's takv, contact callsign and endpoint
//!   and VMF (`:303-331`) - written to the stream, its carriage returns taken out, a line each;
//!   all of them shown in the box; then the interval slept; a round that throws sleeps a second;
//! * Additional Details shows the grid's last four columns, or hides them (`:412-422`); Nice
//!   Formatting is the XML's indenting, an attribute a line (`:432-435`); Clear Window empties the
//!   box until the next round (`:362-365`).
//!
//! Where this is not the C# (each at its site):
//!
//! * a stream that will not open, and TAK's address taken, go on the status line (the owner's
//!   ruling of 2026-09-25) - the C# says nothing of the first; a TCP host takes one client (the
//!   notes in `serial_output.rs`);
//! * the form is drawn over the screen it was opened from, modal; the thread reads a copy of the
//!   form's boxes and the vehicles' state taken each frame, where the C#'s reads the form and
//!   `cs` live, and the closed form's last; the vehicles in the link's order, by their ids, where
//!   the C#'s `MAVlist` has them in the order heard;
//! * a stop ends the thread at once, where the C#'s wakes from its sleep and then sees it (and a
//!   Connect inside that sleep had two threads running).

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::Lock as _;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use web_time::{Duration, Instant};

use gpui::{
    AnyElement, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px, rgb,
};
use mp_mission::dotnet::{format_f64, parse_bool};
use mp_transport::{TcpTransport, Transport};
use mp_vehicle::{VehicleId, VehicleState};

use super::optional::{button, input_box};
use super::serial_output::{
    BAUDS, CONNECT, ERROR_CONNECTING, INVALID_PORT_NAME, Kind, Opening, Questions, STOP, baud_of,
    combo, open, poll,
};
use crate::MissionPlanner;
use crate::settings::Persisted;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::theme;
use mp_vehicle::units::DisplayUnits;

/// `this.Text`. `// C#: Controls/SerialOutputCoT.Designer.cs:301`
pub const FORM_TEXT: &str = "Output Cursor on Target";
/// `ClientSize`.
pub const CLIENT: (f32, f32) = (583.0, 575.0);
/// `GB_connection` and what is in it - `CMB_serialport`, `CMB_baudrate`, `BUT_connect` - placed
/// on the form. `// C#: Controls/SerialOutputCoT.Designer.cs:82-140`
const GROUP_AT: (f32, f32, f32, f32) = (12.0, 12.0, 217.0, 81.0);
const PORT_AT: (f32, f32, f32, f32) = (20.0, 31.0, 121.0, 21.0);
const BAUD_AT: (f32, f32, f32, f32) = (20.0, 58.0, 121.0, 21.0);
const CONNECT_AT: (f32, f32, f32, f32) = (147.0, 31.0, 75.0, 23.0);
/// `CB_advancedMode`, `chk_indent`. `// C#: Controls/SerialOutputCoT.Designer.cs:162-188`
const ADVANCED_AT: (f32, f32, f32, f32) = (247.0, 20.0, 107.0, 17.0);
const INDENT_AT: (f32, f32, f32, f32) = (358.0, 20.0, 100.0, 17.0);
/// `label_type` and `TB_xml_type`. `// C#: Controls/SerialOutputCoT.Designer.cs:145-160`
const TYPE_LABEL_AT: (f32, f32) = (233.0, 49.0);
const TYPE_AT: (f32, f32, f32, f32) = (265.0, 47.0, 88.0, 20.0);
/// `label2` and `updateRate_numericUpDown`. `// C#: Controls/SerialOutputCoT.Designer.cs:190-220`
const RATE_LABEL_AT: (f32, f32) = (235.0, 75.0);
const RATE_AT: (f32, f32, f32, f32) = (307.0, 73.0, 46.0, 20.0);
/// `BTN_clear_TB`, `TB_output`, `myDataGridView1`.
/// `// C#: Controls/SerialOutputCoT.Designer.cs:98-118, 222-238`
const CLEAR_AT: (f32, f32, f32, f32) = (370.0, 70.0, 88.0, 23.0);
const OUTPUT_AT: (f32, f32, f32, f32) = (12.0, 102.0, 559.0, 334.0);
const GRID_AT: (f32, f32, f32, f32) = (12.0, 443.0, 559.0, 121.0);
/// The grid's columns, `HeaderText` and `Width` (100 where the Designer gives none), and the
/// row headers' width. `// C#: Controls/SerialOutputCoT.Designer.cs:240-283`
pub const COLUMNS: [(&str, f32); 6] = [
    ("sysid", 55.0),
    ("UID", 100.0),
    ("takv", 34.0),
    ("ContactCallsign", 100.0),
    ("ContactEndPointIP", 100.0),
    ("VMF", 100.0),
];
const ROW_HEADERS: f32 = 62.0;
const ROW_HEIGHT: f32 = 22.0;
/// The takv column, a check box.
const TAKV: usize = 2;
/// `TB_xml_type.Text`. `// C#: Controls/SerialOutputCoT.Designer.cs:160`
pub const XML_TYPE: &str = "a-f-A-M-F-Q";
/// `updateRate_numericUpDown`'s `Minimum`, `Maximum` and `Value`, seconds.
/// `// C#: Controls/SerialOutputCoT.Designer.cs:193-210`
pub const RATE_MIN: f64 = 1.0;
pub const RATE_MAX: f64 = 9999.0;
pub const RATE: f64 = 10.0;
/// The TCP and UDP hosts' port in the names. `// C#: Controls/SerialOutputCoT.cs:32, 34`
pub const HOST_PORT: u16 = 14551;
/// `ConfigRef` of the TCP client and of TAK Multicast. `// C#: Controls/SerialOutputCoT.cs:74, 86`
pub const TCP_CONFIG_REF: &str = "SerialOutputCoTTCP";
pub const TAK_CONFIG_REF: &str = "TAK_Multicast";
/// TAK's multicast group. `// C#: Controls/SerialOutputCoT.cs:87`
pub const TAK_HOST: &str = "239.2.3.1";
pub const TAK_PORT: u16 = 6969;
/// The box when TAK's address is taken. `// C#: Controls/SerialOutputCoT.cs:100-101`
pub const TAK_IN_USE: &str = "TAK IP Port in use. Is another app on this system already using it?";
/// `how`: machine-generated, from GPS. `// C#: Controls/SerialOutputCoT.cs:221-229`
pub const HOW: &str = "m-g";
/// The settings `Load` reads and `FormClosing` writes, and the grid's.
/// `// C#: Controls/SerialOutputCoT.cs:372-380, 400-404, 409`
pub const UID_SETTING: &str = "CoTUID";
const RATE_SETTING: &str = "CoT_updateRate";
const PORT_SETTING: &str = "CoT_CMB_serialport";
const BAUD_SETTING: &str = "CoT_CMB_baudrate";
const ADVANCED_SETTING: &str = "CoT_CB_advancedMode";
const INDENT_SETTING: &str = "CoT_chk_indent";
/// A round that throws: a second's sleep. `// C#: Controls/SerialOutputCoT.cs:188-191`
const AFTER_THROW: Duration = Duration::from_secs(1);

// ---- The event -----------------------------------------------------------------------------

/// One vehicle's event, as `getXmlString` makes it. `// C#: Controls/SerialOutputCoT.cs:263-301`
#[derive(Debug, Clone, PartialEq)]
pub struct Event {
    pub uid: String,
    pub kind: String,
    pub how: String,
    pub lat: f64,
    pub lng: f64,
    pub alt: f64,
    pub course: f64,
    pub speed: f64,
    /// `DateTime.UtcNow`.
    pub time: chrono::DateTime<chrono::Utc>,
    /// Additional Details, from the UID's row: a `<takv />`, the contact's callsign and endpoint,
    /// the VMF uid - each only when given.
    pub takv: bool,
    pub callsign: Option<String>,
    pub endpoint: Option<String>,
    pub vmf: Option<String>,
}

/// `yyyy-MM-ddTHH:mm:ss.ffK` of a UTC time: hundredths cut, not rounded, and `Z`.
fn cot_time(time: chrono::DateTime<chrono::Utc>) -> String {
    let hundredths = time.timestamp_subsec_millis() / 10;
    format!("{}.{hundredths:02}Z", time.format("%Y-%m-%dT%H:%M:%S"))
}

/// `ToString("N<n>")` with no group separator, as `getXmlString`'s culture has it: .NET's fifteen
/// digits rounded half away from zero, a zero shown without its sign.
fn number(value: f64, decimals: usize) -> String {
    format_f64(value, &format!("0.{}", "0".repeat(decimals)))
}

/// An attribute's value as `XmlWriter` writes it: `&`, `<`, `>` and `"` as entities, tab, line
/// feed and carriage return as character references; a character XML may not hold is what the
/// C#'s writer throws for.
fn escaped(value: &str) -> Result<String, String> {
    let mut out = String::with_capacity(value.len());
    for c in value.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\t' => out.push_str("&#x9;"),
            '\n' => out.push_str("&#xA;"),
            '\r' => out.push_str("&#xD;"),
            c if (c as u32) < 0x20 || c == '\u{FFFE}' || c == '\u{FFFF}' => {
                return Err(format!(
                    "'{}', hexadecimal value 0x{:02X}, is an invalid character.",
                    c.escape_unicode(),
                    c as u32
                ));
            }
            c => out.push(c),
        }
    }
    Ok(out)
}

/// `XmlWriter` with `Indent` and `NewLineOnAttributes` as Nice Formatting sets them: an element a
/// line, an attribute a line a level in, two spaces a level, `\r\n` between; or all on one line.
struct XmlOut {
    text: String,
    indent: bool,
    level: usize,
}

impl XmlOut {
    fn line(&mut self) {
        self.text.push_str("\r\n");
        for _ in 0..self.level {
            self.text.push_str("  ");
        }
    }

    /// `<name`, on its own line but the root.
    fn open(&mut self, name: &str) {
        if self.indent && !self.text.is_empty() {
            self.line();
        }
        self.text.push('<');
        self.text.push_str(name);
        self.level += 1;
    }

    fn attribute(&mut self, name: &str, value: &str) -> Result<(), String> {
        if self.indent {
            self.line();
        } else {
            self.text.push(' ');
        }
        self.text.push_str(name);
        self.text.push_str("=\"");
        self.text.push_str(&escaped(value)?);
        self.text.push('"');
        Ok(())
    }

    /// The start tag ended, for children to follow.
    fn children(&mut self) {
        self.text.push('>');
    }

    /// An element with no children: ` />`.
    fn close_empty(&mut self) {
        self.level -= 1;
        self.text.push_str(" />");
    }

    /// `</name>` after its children, on its own line.
    fn close(&mut self, name: &str) {
        self.level -= 1;
        if self.indent {
            self.line();
        }
        self.text.push_str("</");
        self.text.push_str(name);
        self.text.push('>');
    }
}

/// `getXmlString`'s text: the declaration, then the event as `XmlSerializer` writes the
/// `MissionPlanner.Utilities.CoT` classes - attributes first, in their order, then the child
/// elements, a null one left out. `Err` is what the C#'s writer throws.
/// `// C#: Controls/SerialOutputCoT.cs:263-360; ExtLibs/Utilities/CoT/*.cs`
pub fn event_xml(event: &Event, indent: bool) -> Result<String, String> {
    let mut x = XmlOut {
        text: String::new(),
        indent,
        level: 0,
    };
    let time = cot_time(event.time);
    let start = cot_time(event.time - chrono::Duration::seconds(5));
    let stale = cot_time(event.time + chrono::Duration::seconds(120));
    x.open("event");
    x.attribute("version", "2.0")?;
    x.attribute("uid", &event.uid)?;
    x.attribute("type", &event.kind)?;
    x.attribute("time", &time)?;
    x.attribute("start", &start)?;
    x.attribute("stale", &stale)?;
    x.attribute("how", &event.how)?;
    x.children();
    x.open("detail");
    x.children();
    if event.takv {
        x.open("takv");
        x.close_empty();
    }
    if event.callsign.is_some() || event.endpoint.is_some() {
        x.open("contact");
        if let Some(callsign) = &event.callsign {
            x.attribute("callsign", callsign)?;
        }
        if let Some(endpoint) = &event.endpoint {
            x.attribute("endpoint", endpoint)?;
        }
        x.close_empty();
    }
    if let Some(vmf) = &event.vmf {
        x.open("uid");
        x.attribute("vmf", vmf)?;
        x.close_empty();
    }
    x.open("track");
    x.attribute("course", &number(event.course, 2))?;
    x.attribute("speed", &number(event.speed, 2))?;
    x.close_empty();
    x.close("detail");
    x.open("point");
    x.attribute("lat", &number(event.lat, 7))?;
    x.attribute("lon", &number(event.lng, 7))?;
    x.attribute("hae", &format!("{:>5}", number(event.alt, 2)))?;
    x.attribute("ce", "1.0")?;
    x.attribute("le", "1.0")?;
    x.close_empty();
    x.close("event");
    Ok(format!(
        "<?xml version=\"1.0\" encoding=\"utf-8\" standalone=\"yes\"?>\r\n{}",
        x.text
    ))
}

// ---- The grid ------------------------------------------------------------------------------

/// A row of the UID grid: each cell's value as the C#'s grid holds it - null, a string typed in,
/// takv's check, or whatever the setting's JSON held.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Row {
    pub cells: [Option<serde_json::Value>; 6],
}

/// A cell's `Value?.ToString()`: a string as it is, a bool as .NET writes one, a number as its
/// text, null as nothing.
#[must_use]
pub fn cell_text(value: Option<&serde_json::Value>) -> Option<String> {
    match value? {
        serde_json::Value::Null => None,
        serde_json::Value::String(text) => Some(text.clone()),
        serde_json::Value::Bool(true) => Some("True".to_owned()),
        serde_json::Value::Bool(false) => Some("False".to_owned()),
        other => Some(other.to_string()),
    }
}

impl Row {
    /// `Value?.ToString()` of a column.
    #[must_use]
    pub fn text(&self, column: usize) -> Option<String> {
        cell_text(self.cells.get(column).and_then(Option::as_ref))
    }

    /// `isValidStr`: a value whose text is not empty. `// C#: Controls/SerialOutputCoT.cs:258-261`
    #[must_use]
    pub fn valid(&self, column: usize) -> Option<String> {
        self.text(column).filter(|text| !text.is_empty())
    }
}

/// `myDataGridView1.Serialize()`: every row's cells, an `object[,]` as Newtonsoft writes it.
/// `// C#: Utilities/ExtensionsMP.cs:167-176`
#[must_use]
pub fn serialize(rows: &[Row]) -> String {
    let array: Vec<serde_json::Value> = rows
        .iter()
        .map(|row| {
            serde_json::Value::Array(
                row.cells
                    .iter()
                    .map(|cell| cell.clone().unwrap_or(serde_json::Value::Null))
                    .collect(),
            )
        })
        .collect();
    serde_json::Value::Array(array).to_string()
}

/// `Deserialize`: a row for each of the JSON's, its cells in column order; `None` where the C#
/// throws, which `Load` catches, leaving the grid empty.
/// `// C#: Utilities/ExtensionsMP.cs:178-189; Controls/SerialOutputCoT.cs:369-373`
#[must_use]
pub fn deserialize(input: &str) -> Option<Vec<Row>> {
    let serde_json::Value::Array(rows) = serde_json::from_str(input).ok()? else {
        return None;
    };
    rows.into_iter()
        .map(|row| {
            let serde_json::Value::Array(values) = row else {
                return None;
            };
            // `object[,]`: a rectangle, no wider than the grid.
            if values.len() > COLUMNS.len() {
                return None;
            }
            let mut cells: [Option<serde_json::Value>; 6] = Default::default();
            for (cell, value) in cells.iter_mut().zip(values) {
                *cell = (!value.is_null()).then_some(value);
            }
            Some(Row { cells })
        })
        .collect()
}

/// `FindUIDviaSysid`: the UID of the first row whose sysid reads as `sysid`, else
/// `"NOsysid<sysid>"`; a row there with no UID gives none. `// C#: Controls/SerialOutputCoT.cs:248-256`
#[must_use]
pub fn uid_for(rows: &[Row], sysid: u32) -> Option<String> {
    let wanted = sysid.to_string();
    match rows
        .iter()
        .find(|row| row.text(0).as_deref() == Some(&wanted))
    {
        Some(row) => row.text(1),
        None => Some(format!("NOsysid{sysid}")),
    }
}

/// `FindRowviaUID`: the first row whose UID is `uid`, none for an empty one.
/// `// C#: Controls/SerialOutputCoT.cs:235-246`
#[must_use]
pub fn row_for<'a>(rows: &'a [Row], uid: &str) -> Option<&'a Row> {
    if uid.is_empty() {
        return None;
    }
    rows.iter().find(|row| row.text(1).as_deref() == Some(uid))
}

/// What the thread reads of a vehicle: `cs.lat`, `cs.lng`, `cs.altasl`, `cs.groundspeed` and
/// `cs.groundcourse`, as the C#'s getters give them - the altitude and speed in the user's units.
/// `// C#: Controls/SerialOutputCoT.cs:215-219`
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Position {
    pub lat: f64,
    pub lng: f64,
    pub alt: f64,
    pub speed: f64,
    pub course: f64,
}

impl Position {
    #[must_use]
    pub fn of(state: &VehicleState, units: &DisplayUnits) -> Self {
        let read = |name: &str| crate::quick::display_value(name, state, units).unwrap_or(0.0);
        Self {
            lat: read("lat"),
            lng: read("lng"),
            alt: read("altasl"),
            speed: read("groundspeed"),
            course: read("groundcourse"),
        }
    }
}

/// What a round reads of the form: the Type box, the grid, Additional Details, Nice Formatting.
#[derive(Debug, Clone, Default)]
pub struct Inputs {
    pub vehicles: Vec<(VehicleId, Position)>,
    pub xml_type: String,
    pub rows: Vec<Row>,
    pub advanced: bool,
    pub indent: bool,
}

/// `getXmlString(sysid, compid)` for one vehicle at `time`: its UID, the type, `how`, its
/// position, and with Additional Details its UID's row's takv, contact and VMF; `Err` for what
/// throws there - `Convert.ToBoolean` of a takv that is no bool, or the writer's.
/// `// C#: Controls/SerialOutputCoT.cs:213-234, 303-331`
pub fn vehicle_xml(
    inputs: &Inputs,
    sysid: u32,
    at: &Position,
    time: chrono::DateTime<chrono::Utc>,
) -> Result<String, String> {
    let uid = uid_for(&inputs.rows, sysid).unwrap_or_default();
    let mut event = Event {
        uid: uid.clone(),
        kind: inputs.xml_type.clone(),
        how: HOW.to_owned(),
        lat: at.lat,
        lng: at.lng,
        alt: at.alt,
        course: at.course,
        speed: at.speed,
        time,
        takv: false,
        callsign: None,
        endpoint: None,
        vmf: None,
    };
    if inputs.advanced
        && let Some(row) = row_for(&inputs.rows, &uid)
    {
        if let Some(value) = row.cells[TAKV].as_ref().filter(|v| !v.is_null()) {
            event.takv = match value {
                serde_json::Value::Bool(b) => *b,
                other => cell_text(Some(other))
                    .as_deref()
                    .and_then(parse_bool)
                    .ok_or("String was not recognized as a valid Boolean.")?,
            };
        }
        event.callsign = row.valid(3);
        event.endpoint = row.valid(4);
        event.vmf = row.valid(5);
    }
    event_xml(&event, inputs.indent)
}

// ---- The thread ----------------------------------------------------------------------------

/// What the thread and the form share.
struct Shared {
    running: AtomicBool,
    /// `updaterate_ms`.
    rate_ms: Mutex<u64>,
    inputs: Mutex<Inputs>,
    /// `CoTStream`, once open, and TCP Client's address to connect to again.
    stream: Mutex<Option<Box<dyn Transport>>>,
    reconnect: Mutex<Option<(String, u16)>>,
    /// `TB_output.Text`, and the events written.
    view: Mutex<String>,
    written: AtomicUsize,
}

/// `mainloop` on its thread, until stopped. `// C#: Controls/SerialOutputCoT.cs:156-193`
fn main_loop(shared: &Shared) {
    while shared.running.load(Ordering::Acquire) {
        let inputs = shared
            .inputs
            .os_lock()
            .map(|i| i.clone())
            .unwrap_or_default();
        let time = chrono::Utc::now();
        let mut view = String::new();
        let mut threw = false;
        for (id, at) in &inputs.vehicles {
            let xml = match vehicle_xml(&inputs, id.sysid, at, time) {
                Ok(xml) => xml,
                Err(_) => {
                    threw = true;
                    break;
                }
            };
            view.push_str(&xml);
            if !write_line(shared, &xml.replace('\r', "")) {
                threw = true;
                break;
            }
        }
        if threw {
            sleep_while_running(shared, AFTER_THROW);
            continue;
        }
        if let Ok(mut held) = shared.view.os_lock() {
            *held = view;
        }
        let rate = shared.rate_ms.os_lock().map(|r| *r).unwrap_or(10_000);
        sleep_while_running(shared, Duration::from_millis(rate));
    }
    if let Ok(mut stream) = shared.stream.os_lock()
        && let Some(mut stream) = stream.take()
    {
        stream.close();
    }
}

/// `if (CoTStream != null && CoTStream.IsOpen) CoTStream.WriteLine(...)`: false when the write
/// throws. A TCP client that dropped connects again first, as `TcpSerial`'s `autoReconnect`
/// does. `// C#: Controls/SerialOutputCoT.cs:172-175, 74; ExtLibs/Comms/CommsTCPSerial.cs`
fn write_line(shared: &Shared, line: &str) -> bool {
    let Ok(mut held) = shared.stream.os_lock() else {
        return true;
    };
    if held.as_ref().is_none_or(|stream| !stream.is_open())
        && let Some((host, port)) = shared.reconnect.os_lock().ok().and_then(|r| r.clone())
    {
        *held = TcpTransport::connect(&host, port)
            .ok()
            .map(|s| Box::new(s) as Box<dyn Transport>);
    }
    let Some(stream) = held.as_mut().filter(|stream| stream.is_open()) else {
        return true;
    };
    if stream.write_all(format!("{line}\n").as_bytes()).is_err() {
        return false;
    }
    shared.written.fetch_add(1, Ordering::Relaxed);
    true
}

/// A sleep a stop ends at once.
fn sleep_while_running(shared: &Shared, total: Duration) {
    let until = Instant::now() + total;
    while shared.running.load(Ordering::Acquire) && Instant::now() < until {
        wasm_thread::sleep(Duration::from_millis(20).min(until - Instant::now()));
    }
}

// ---- The form ------------------------------------------------------------------------------

/// What is being typed into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Editing {
    /// A grid cell: its row - the rows' count for the new row - and column.
    Cell(usize, usize),
    /// `TB_xml_type`.
    Type,
    /// `updateRate_numericUpDown`.
    Rate,
}

/// The form's controls.
#[derive(Debug)]
pub struct Form {
    /// `CMB_serialport`: its items, its text, whether its list is down.
    pub ports: Vec<String>,
    pub port: String,
    pub port_open: bool,
    /// `CMB_baudrate`, and whether it is enabled.
    pub baud: String,
    pub baud_open: bool,
    /// `TB_xml_type.Text`.
    pub xml_type: String,
    /// `updateRate_numericUpDown.Value`.
    pub rate: f64,
    /// `CB_advancedMode`, `chk_indent`.
    pub advanced: bool,
    pub indent: bool,
    /// The grid.
    pub rows: Vec<Row>,
    /// What is being typed into, and its text.
    pub editing: Option<Editing>,
    pub field: TextField,
    /// The questions a Connect is asking, and the kind and baud they are for.
    pub questions: Questions,
    pending: Option<(Kind, u32)>,
}

impl Form {
    /// `CMB_baudrate.Enabled`: only for a port whose name has "com" in it, any case.
    /// `// C#: Controls/SerialOutputCoT.cs:424-430`
    #[must_use]
    pub fn baud_enabled(&self) -> bool {
        self.port.to_lowercase().contains("com")
    }
}

/// The output, which outlives its form: the thread, its stream, the interval.
#[derive(Default)]
pub struct CotOutput {
    /// The form, while it is open.
    pub window: Option<Form>,
    /// How many times it has been opened.
    pub opened: usize,
    shared: Option<Arc<Shared>>,
    thread: Option<wasm_thread::JoinHandle<()>>,
    /// The stream being opened.
    opening: Option<Opening>,
    /// What the C# would have boxed, for the status line.
    status: Option<String>,
}

impl std::fmt::Debug for CotOutput {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CotOutput")
            .field("window", &self.window)
            .field("running", &self.running())
            .finish_non_exhaustive()
    }
}

/// `CMB_serialport.Items`: TAK Multicast, the four network kinds, then the serial ports.
/// `// C#: Controls/SerialOutputCoT.cs:31-36`
#[must_use]
pub fn port_items() -> Vec<String> {
    let mut items = vec![
        "TAK Multicast".to_owned(),
        format!("TCP Host - {HOST_PORT}"),
        "TCP Client".to_owned(),
        format!("UDP Host - {HOST_PORT}"),
        "UDP Client".to_owned(),
    ];
    items.extend(mp_transport::list_ports().into_iter().map(|port| port.name));
    items
}

impl CotOutput {
    /// `threadrun`.
    #[must_use]
    pub fn running(&self) -> bool {
        self.shared
            .as_ref()
            .is_some_and(|shared| shared.running.load(Ordering::Acquire))
    }

    /// `BUT_connect.Text`.
    #[must_use]
    pub fn button(&self) -> &'static str {
        if self.running() { STOP } else { CONNECT }
    }

    /// Events written since the thread started.
    #[must_use]
    pub fn written(&self) -> usize {
        self.shared
            .as_ref()
            .map_or(0, |shared| shared.written.load(Ordering::Relaxed))
    }

    /// `TB_output.Text`.
    #[must_use]
    pub fn output(&self) -> String {
        self.shared
            .as_ref()
            .and_then(|shared| shared.view.os_lock().ok().map(|v| v.clone()))
            .unwrap_or_default()
    }

    /// Whether the stream is open.
    #[must_use]
    pub fn stream_open(&self) -> bool {
        self.shared.as_ref().is_some_and(|shared| {
            shared
                .stream
                .os_lock()
                .is_ok_and(|s| s.as_ref().is_some_and(|s| s.is_open()))
        })
    }

    /// `new SerialOutputCoT().Show()` and `Load`: the lists, the grid and the boxes from the
    /// settings, and a thread already running stopped and started again under this form.
    /// `// C#: Controls/SerialOutputCoT.cs:27-38, 367-396`
    pub fn show(&mut self, persisted: &mut Persisted) {
        self.opened += 1;
        let ports = port_items();
        // `CMB_*.SelectedIndex = GetInt32(...)`: an index past the list throws in `Load`; here
        // nothing is chosen.
        let index = |key: &str| {
            persisted
                .get(key)
                .and_then(|text| text.trim().parse::<usize>().ok())
                .unwrap_or(0)
        };
        let port = ports.get(index(PORT_SETTING)).cloned().unwrap_or_default();
        let baud = BAUDS
            .get(index(BAUD_SETTING))
            .map_or(String::new(), |b| (*b).to_owned());
        let rate = persisted
            .get(RATE_SETTING)
            .and_then(mp_mission::dotnet::parse_f64)
            .unwrap_or(RATE);
        let flag = |key: &str| persisted.get(key).and_then(parse_bool).unwrap_or(true);
        let rows = persisted
            .get(UID_SETTING)
            .and_then(deserialize)
            .unwrap_or_default();
        self.window = Some(Form {
            ports,
            port,
            port_open: false,
            baud,
            baud_open: false,
            xml_type: XML_TYPE.to_owned(),
            rate: rate.clamp(RATE_MIN, RATE_MAX),
            advanced: flag(ADVANCED_SETTING),
            indent: flag(INDENT_SETTING),
            rows,
            editing: None,
            field: TextField::new(""),
            questions: Questions::default(),
            pending: None,
        });
        self.push_inputs(None);
        if self.running() {
            // Stop, and start again.
            self.press_connect(persisted);
            self.press_connect(persisted);
        }
    }

    /// The close box: `FormClosing` saves the five settings; the thread goes on.
    /// `// C#: Controls/SerialOutputCoT.cs:398-405`
    pub fn close(&mut self, persisted: &mut Persisted) {
        self.leave(persisted);
        if let Some(form) = self.window.take() {
            persisted.set(RATE_SETTING, mp_mission::dotnet::general_f64(form.rate));
            let index = |items: &[String], text: &str| {
                items
                    .iter()
                    .position(|item| item == text)
                    .map_or("-1".to_owned(), |i| i.to_string())
            };
            persisted.set(PORT_SETTING, index(&form.ports, &form.port));
            let bauds: Vec<String> = BAUDS.iter().map(|b| (*b).to_owned()).collect();
            persisted.set(BAUD_SETTING, index(&bauds, &form.baud));
            persisted.set(
                ADVANCED_SETTING,
                mp_mission::dotnet::bool_text(form.advanced),
            );
            persisted.set(INDENT_SETTING, mp_mission::dotnet::bool_text(form.indent));
        }
    }

    /// The thread's copy of the form - and, with `vehicles`, of the vehicles.
    fn push_inputs(&self, vehicles: Option<Vec<(VehicleId, Position)>>) {
        let (Some(shared), Some(form)) = (self.shared.as_ref(), self.window.as_ref()) else {
            if let (Some(shared), Some(vehicles)) = (self.shared.as_ref(), vehicles)
                && let Ok(mut inputs) = shared.inputs.os_lock()
            {
                inputs.vehicles = vehicles;
            }
            return;
        };
        if let Ok(mut inputs) = shared.inputs.os_lock() {
            if let Some(vehicles) = vehicles {
                inputs.vehicles = vehicles;
            }
            inputs.xml_type.clone_from(&form.xml_type);
            inputs.rows.clone_from(&form.rows);
            inputs.advanced = form.advanced;
            inputs.indent = form.indent;
        }
        if let Ok(mut rate) = shared.rate_ms.os_lock() {
            *rate = rate_ms(form.rate);
        }
    }

    /// A combo's list opened or closed.
    pub fn toggle_list(&mut self, baud: bool) {
        if let Some(form) = self.window.as_mut() {
            if baud {
                form.port_open = false;
                form.baud_open = !form.baud_open;
            } else {
                form.baud_open = false;
                form.port_open = !form.port_open;
            }
        }
    }

    /// An item chosen: the port's `SelectedIndexChanged` enables the baud or not.
    pub fn choose(&mut self, baud: bool, index: usize) {
        let Some(form) = self.window.as_mut() else {
            return;
        };
        if baud {
            if let Some(item) = BAUDS.get(index) {
                form.baud = (*item).to_owned();
            }
            form.baud_open = false;
        } else {
            if let Some(item) = form.ports.get(index) {
                form.port = item.clone();
            }
            form.port_open = false;
        }
    }

    /// Additional Details or Nice Formatting ticked: `CB_advancedMode_CheckedChanged` and
    /// `chk_indent_CheckedChanged`. `// C#: Controls/SerialOutputCoT.cs:412-422, 432-435`
    pub fn toggle_check(&mut self, indent: bool) {
        if let Some(form) = self.window.as_mut() {
            if indent {
                form.indent = !form.indent;
            } else {
                form.advanced = !form.advanced;
            }
        }
        self.push_inputs(None);
    }

    /// Whether a column shows: sysid and UID always, the rest with Additional Details.
    #[must_use]
    pub fn column_shown(&self, column: usize) -> bool {
        column < 2 || self.window.as_ref().is_some_and(|form| form.advanced)
    }

    /// Clear Window. `// C#: Controls/SerialOutputCoT.cs:362-365`
    pub fn clear(&mut self) {
        if let Some(shared) = self.shared.as_ref()
            && let Ok(mut view) = shared.view.os_lock()
        {
            view.clear();
        }
    }

    /// The interval's arrows: a second up or down, within its range.
    pub fn step_rate(&mut self, up: bool) {
        if let Some(form) = self.window.as_mut() {
            let next = if up { form.rate + 1.0 } else { form.rate - 1.0 };
            form.rate = next.clamp(RATE_MIN, RATE_MAX);
        }
        self.push_inputs(None);
    }

    /// A click on a grid cell, the Type box or the interval: takv ticked (`CellMouseUp`'s
    /// `EndEdit`), else typed into; a click on the new row's cell adds the row.
    pub fn begin(&mut self, target: Editing, persisted: &mut Persisted) {
        self.leave(persisted);
        let Some(form) = self.window.as_mut() else {
            return;
        };
        let text = match target {
            Editing::Cell(row, column) => {
                if row == form.rows.len() {
                    form.rows.push(Row::default());
                }
                let Some(cells) = form.rows.get_mut(row) else {
                    return;
                };
                if column == TAKV {
                    let ticked = cells.cells[TAKV]
                        .as_ref()
                        .and_then(|v| cell_text(Some(v)))
                        .as_deref()
                        .and_then(parse_bool)
                        .unwrap_or(false);
                    cells.cells[TAKV] = Some(serde_json::Value::Bool(!ticked));
                    persisted.set(UID_SETTING, serialize(&form.rows));
                    self.push_inputs(None);
                    return;
                }
                cells.text(column).unwrap_or_default()
            }
            Editing::Type => form.xml_type.clone(),
            Editing::Rate => mp_mission::dotnet::general_f64(form.rate),
        };
        form.field = TextField::new("");
        form.field.set(text);
        form.editing = Some(target);
    }

    /// A key while typing; Enter or Escape ends it.
    pub fn key(&mut self, event: &KeyDownEvent, persisted: &mut Persisted) -> bool {
        let Some(form) = self.window.as_mut() else {
            return false;
        };
        if form.editing.is_none() {
            return false;
        }
        match form.field.key(event) {
            KeyOutcome::Changed => {
                // The Type box is read as it stands, each round.
                if form.editing == Some(Editing::Type) {
                    form.xml_type = form.field.value().to_owned();
                    self.push_inputs(None);
                }
                true
            }
            KeyOutcome::Submitted | KeyOutcome::Cancelled => {
                self.leave(persisted);
                true
            }
            KeyOutcome::Ignored => false,
        }
    }

    /// The typing left: a cell's text its value - none when empty - and the grid saved
    /// (`RowValidated`); the Type box's text; the interval's number within its range.
    /// `// C#: Controls/SerialOutputCoT.cs:407-410, 447-455`
    pub fn leave(&mut self, persisted: &mut Persisted) {
        let Some(form) = self.window.as_mut() else {
            return;
        };
        let Some(target) = form.editing.take() else {
            return;
        };
        let text = form.field.value().to_owned();
        match target {
            Editing::Cell(row, column) => {
                if let Some(cells) = form.rows.get_mut(row)
                    && let Some(cell) = cells.cells.get_mut(column)
                {
                    *cell = (!text.is_empty()).then(|| serde_json::Value::String(text));
                }
                persisted.set(UID_SETTING, serialize(&form.rows));
            }
            Editing::Type => form.xml_type = text,
            Editing::Rate => {
                if let Some(rate) = mp_mission::dotnet::parse_f64(&text) {
                    form.rate = rate.clamp(RATE_MIN, RATE_MAX);
                }
            }
        }
        self.push_inputs(None);
    }

    /// `BUT_connect_Click`: Stop ends the thread and closes the stream; Connect makes the stream
    /// the port names - asking the kind's questions first - and starts the thread.
    /// `// C#: Controls/SerialOutputCoT.cs:40-140`
    pub fn press_connect(&mut self, persisted: &mut Persisted) {
        if self.running() {
            if let Some(shared) = self.shared.take() {
                shared.running.store(false, Ordering::Release);
            }
            self.opening = None;
            if let Some(thread) = self.thread.take() {
                let deadline = Instant::now() + Duration::from_millis(500);
                while !thread.is_finished() && Instant::now() < deadline {
                    wasm_thread::sleep(Duration::from_millis(1));
                }
            }
            return;
        }
        let Some(form) = self.window.as_mut() else {
            return;
        };
        let port = form.port.trim().to_owned();
        if port.is_empty() {
            self.status = Some(INVALID_PORT_NAME.to_owned());
            return;
        }
        let kind = Kind::of(&port);
        // A network kind's `CMB_baudrate.SelectedIndex = 0`.
        if !matches!(kind, Kind::Serial(_)) {
            BAUDS[0].clone_into(&mut form.baud);
        }
        let baud = match baud_of(&form.baud) {
            Ok(baud) => baud,
            Err(words) => {
                self.status = Some(words);
                return;
            }
        };
        if kind == Kind::TakMulticast {
            // `Open(host, port)` keeps the address as the client's settings.
            // C#: ExtLibs/Comms/CommsUDPSerialConnect.cs:74-79
            persisted.set(&format!("UDP_port{TAK_CONFIG_REF}"), TAK_PORT.to_string());
            persisted.set(&format!("UDP_host{TAK_CONFIG_REF}"), TAK_HOST);
        }
        let config_ref = if kind == Kind::TcpClient {
            TCP_CONFIG_REF
        } else {
            ""
        };
        form.questions = Questions {
            pending: kind.questions(config_ref),
            ..Questions::default()
        };
        form.pending = Some((kind, baud));
        if !form.questions.ask_next(persisted) {
            self.start(persisted);
        }
    }

    /// A question answered: the next asked, or the stream opened with the answers.
    pub fn answered(&mut self, ok: bool, persisted: &mut Persisted) {
        let Some(form) = self.window.as_mut() else {
            return;
        };
        if !form.questions.answered(ok, persisted) {
            form.pending = None;
            return;
        }
        if !form.questions.ask_next(persisted) {
            self.start(persisted);
        }
    }

    /// The stream opened off the form's thread, and the thread started at once.
    fn start(&mut self, _persisted: &mut Persisted) {
        let Some(form) = self.window.as_mut() else {
            return;
        };
        let Some((kind, baud)) = form.pending.take() else {
            return;
        };
        let answers = std::mem::take(&mut form.questions.answers);
        let reconnect = (kind == Kind::TcpClient)
            .then(|| {
                let port = answers.get(1)?.trim().parse::<u16>().ok()?;
                Some((answers.first()?.clone(), port))
            })
            .flatten();
        self.opening = Some(open(kind, HOST_PORT, baud, answers));
        let shared = Arc::new(Shared {
            running: AtomicBool::new(true),
            rate_ms: Mutex::new(rate_ms(form.rate)),
            inputs: Mutex::new(Inputs::default()),
            stream: Mutex::new(None),
            reconnect: Mutex::new(None),
            view: Mutex::new(String::new()),
            written: AtomicUsize::new(0),
        });
        if let Ok(mut held) = shared.reconnect.os_lock() {
            *held = reconnect;
        }
        let for_thread = Arc::clone(&shared);
        match wasm_thread::Builder::new()
            .name("CoT output".to_owned())
            .spawn(move || main_loop(&for_thread))
        {
            Ok(handle) => {
                self.thread = Some(handle);
                self.shared = Some(shared);
                self.push_inputs(None);
            }
            Err(error) => {
                self.opening = None;
                self.status = Some(format!("{ERROR_CONNECTING}: {error}"));
            }
        }
    }

    /// Once a frame: the vehicles copied for the thread, and a stream opened handed to it - or
    /// what the C# would say of one that would not open. What goes on the status line is
    /// returned.
    pub fn tick(&mut self, vehicles: Vec<(VehicleId, Position)>) -> Option<String> {
        if self.shared.is_some() {
            self.push_inputs(Some(vehicles));
        }
        if let Some(opening) = self.opening.as_ref()
            && let Some(opened) = poll(opening)
        {
            self.opening = None;
            match opened {
                Ok(stream) => {
                    if let Some(shared) = self.shared.as_ref()
                        && let Ok(mut held) = shared.stream.os_lock()
                    {
                        *held = Some(stream);
                    }
                }
                Err(error) if error.contains("in use") => self.status = Some(TAK_IN_USE.to_owned()),
                Err(error) => self.status = Some(format!("{ERROR_CONNECTING}: {error}")),
            }
        }
        self.status.take()
    }

    /// Waits for the stream being opened, as a test does.
    #[cfg(test)]
    fn finish_opening(&mut self) -> Option<String> {
        let mut status = None;
        for _ in 0..2000 {
            if let Some(words) = self.tick(Vec::new()) {
                status = Some(words);
            }
            if self.opening.is_none() {
                break;
            }
            wasm_thread::sleep(Duration::from_millis(5));
        }
        status
    }
}

/// `(int)(updateRate_numericUpDown.Value * 1000)`. `// C#: Controls/SerialOutputCoT.cs:452-455`
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // 1 to 9999 seconds
fn rate_ms(seconds: f64) -> u64 {
    (seconds * 1000.0) as u64
}

/// The facts, under `config.cot.`.
pub fn record_facts(holder: &CotOutput) {
    use crate::facts::record;
    record("config.cot.window", holder.window.is_some());
    record("config.cot.opened", holder.opened);
    record("config.cot.running", holder.running());
    record("config.cot.button", holder.button());
    record("config.cot.stream", holder.stream_open());
    record("config.cot.written", holder.written());
    // The box's text on one line.
    record(
        "config.cot.output",
        holder.output().replace("\r\n", " ").replace('\n', " "),
    );
    let Some(form) = holder.window.as_ref() else {
        return;
    };
    record("config.cot.title", FORM_TEXT);
    record("config.cot.ports", form.ports.join(","));
    record("config.cot.port", form.port.clone());
    record("config.cot.baud", form.baud.clone());
    record("config.cot.baud.enabled", form.baud_enabled());
    record("config.cot.type", form.xml_type.clone());
    record(
        "config.cot.rate",
        mp_mission::dotnet::general_f64(form.rate),
    );
    record("config.cot.advanced", form.advanced);
    record("config.cot.indent", form.indent);
    record("config.cot.rows", form.rows.len());
    for (index, row) in form.rows.iter().enumerate() {
        let texts: Vec<String> = (0..COLUMNS.len())
            .map(|column| row.text(column).unwrap_or_default())
            .collect();
        record(format!("config.cot.row.{index}"), texts.join("|"));
    }
    let shown: Vec<&str> = COLUMNS
        .iter()
        .enumerate()
        .filter(|(column, _)| holder.column_shown(*column))
        .map(|(_, (name, _))| *name)
        .collect();
    record("config.cot.columns", shown.join(","));
    record(
        "config.cot.prompt",
        form.questions
            .asking
            .as_ref()
            .map_or("none", |asking| asking.question.title),
    );
}

fn access(this: &mut MissionPlanner) -> &mut CotOutput {
    &mut this.extra.cot_output
}

fn with_settings(this: &mut MissionPlanner, act: impl FnOnce(&mut CotOutput, &mut Persisted)) {
    let (cot, persisted) = (&mut this.extra.cot_output, &mut this.persisted);
    act(cot, persisted);
}

/// A box the form types into: its text, or the field being typed while it is.
#[allow(clippy::too_many_arguments)] // the box's parts
fn text_box(
    id: String,
    text: String,
    at: (f32, f32, f32, f32),
    editing: bool,
    field: &TextField,
    typing: &FocusHandle,
    target: Editing,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (x, y, w, h) = at;
    let base = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(w))
        .h(px(h))
        .flex()
        .items_center()
        .px_1()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        .text_xs()
        .whitespace_nowrap()
        .overflow_hidden()
        .text_color(rgb(theme::TEXT));
    typed(base, text, editing, field, typing, target, cx)
}

/// What is common to a box and a cell: typed into while editing, a click to begin.
fn typed(
    base: gpui::Stateful<gpui::Div>,
    text: String,
    editing: bool,
    field: &TextField,
    typing: &FocusHandle,
    target: Editing,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let handle = typing.clone();
    if editing {
        base.bg(rgb(theme::ACTION))
            .track_focus(&handle)
            .key_context("TextField")
            .child(field.value().to_owned())
            .child(div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT)))
            .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                let mut handled = false;
                with_settings(this, |cot, persisted| handled = cot.key(event, persisted));
                if handled {
                    cx.notify();
                }
            }))
            .into_any_element()
    } else {
        base.cursor_pointer()
            .child(text)
            .on_click(cx.listener(move |this, _event, window, cx| {
                with_settings(this, |cot, persisted| cot.begin(target, persisted));
                if !matches!(target, Editing::Cell(_, TAKV)) {
                    handle.focus(window, cx);
                }
                cx.notify();
            }))
            .into_any_element()
    }
}

/// A check box and its words.
fn check(
    id: &'static str,
    words: &'static str,
    ticked: bool,
    at: (f32, f32, f32, f32),
    indent: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (x, y, w, h) = at;
    crate::probe::measured(id, div())
        .id(id)
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(w))
        .h(px(h))
        .flex()
        .items_center()
        .gap_1()
        .cursor_pointer()
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .child(if ticked { "\u{2611}" } else { "\u{2610}" })
        .child(words)
        .on_click(cx.listener(move |this, _event, _window, cx| {
            access(this).toggle_check(indent);
            cx.notify();
        }))
        .into_any_element()
}

/// Words at a place.
fn label(words: &'static str, at: (f32, f32)) -> AnyElement {
    div()
        .absolute()
        .left(px(at.0))
        .top(px(at.1))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .child(words)
        .into_any_element()
}

/// The grid: its header, its rows and the new row, the columns Additional Details hides left out.
fn grid(
    holder: &CotOutput,
    form: &Form,
    typing: &FocusHandle,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (gx, gy, gw, gh) = GRID_AT;
    let shown: Vec<usize> = (0..COLUMNS.len())
        .filter(|column| holder.column_shown(*column))
        .collect();
    let header_cell = |name: &'static str, width: f32| {
        div()
            .flex_shrink_0()
            .w(px(width))
            .h(px(ROW_HEIGHT))
            .flex()
            .items_center()
            .px_1()
            .border_r_1()
            .border_b_1()
            .border_color(rgb(theme::BORDER))
            .child(name)
    };
    let mut header = div()
        .flex()
        .h(px(ROW_HEIGHT))
        .bg(rgb(theme::PANEL))
        .text_xs()
        .text_color(rgb(theme::DIM))
        .child(header_cell("", ROW_HEADERS));
    for (name, width) in shown.iter().filter_map(|&column| COLUMNS.get(column)) {
        header = header.child(header_cell(name, *width));
    }
    let mut body = crate::probe::measured("cot-grid", div())
        .id("cot-grid")
        .absolute()
        .left(px(gx))
        .top(px(gy))
        .w(px(gw))
        .h(px(gh))
        .flex()
        .flex_col()
        .overflow_scroll()
        .bg(rgb(theme::BG))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .child(header);
    let blank = Row::default();
    let rows = form
        .rows
        .iter()
        .enumerate()
        .chain(std::iter::once((form.rows.len(), &blank)));
    for (index, row) in rows {
        let mut line = div().flex().h(px(ROW_HEIGHT)).child(
            div()
                .flex_shrink_0()
                .w(px(ROW_HEADERS))
                .h(px(ROW_HEIGHT))
                .border_r_1()
                .border_b_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::PANEL)),
        );
        for &column in &shown {
            let width = COLUMNS.get(column).map_or(100.0, |c| c.1);
            let text = if column == TAKV {
                let ticked = row
                    .text(TAKV)
                    .as_deref()
                    .and_then(parse_bool)
                    .unwrap_or(false);
                if ticked { "\u{2611}" } else { "\u{2610}" }.to_owned()
            } else {
                row.text(column).unwrap_or_default()
            };
            let id = format!("cot-cell-{index}-{column}");
            let base = crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .flex_shrink_0()
                .w(px(width))
                .h(px(ROW_HEIGHT))
                .flex()
                .items_center()
                .px_1()
                .border_r_1()
                .border_b_1()
                .border_color(rgb(theme::BORDER))
                .text_xs()
                .whitespace_nowrap()
                .overflow_hidden()
                .text_ellipsis()
                .text_color(rgb(theme::TEXT));
            let target = Editing::Cell(index, column);
            let editing = form.editing == Some(target);
            line = line.child(typed(base, text, editing, &form.field, typing, target, cx));
        }
        body = body.child(line);
    }
    body.into_any_element()
}

/// The form over the screen: the connection group, the check boxes, Type and the interval, Clear
/// Window, the output and the UID grid, and a question over it.
/// `// C#: Controls/SerialOutputCoT.Designer.cs`
pub fn overlay(
    holder: &CotOutput,
    typing: &FocusHandle,
    prompt: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = holder.window.as_ref()?;
    let size = window.viewport_size();
    let editing = |target: Editing| form.editing == Some(target);
    let (gx, gy, gw, gh) = GROUP_AT;
    let group = div()
        .absolute()
        .left(px(gx))
        .top(px(gy))
        .w(px(gw))
        .h(px(gh))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_sm()
        .child(
            div()
                .absolute()
                .left(px(6.0))
                .top(px(-8.0))
                .px_1()
                .bg(rgb(theme::PANEL))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child("Connection"),
        );
    let (ox, oy, ow, oh) = OUTPUT_AT;
    let output_text = holder.output();
    let output = crate::probe::measured("cot-TB_output", div())
        .id("cot-TB_output")
        .absolute()
        .left(px(ox))
        .top(px(oy))
        .w(px(ow))
        .h(px(oh))
        .flex()
        .flex_col()
        .overflow_scroll()
        .p_1()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .children(
            output_text
                .split("\r\n")
                .map(|line| div().whitespace_nowrap().child(line.to_owned()))
                .collect::<Vec<_>>(),
        );
    let (rx, ry, rw, rh) = RATE_AT;
    let arrows = div()
        .absolute()
        .left(px(rx + rw))
        .top(px(ry))
        .h(px(rh))
        .flex()
        .flex_col()
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .child(
            crate::probe::measured("cot-rate-up", div())
                .id("cot-rate-up")
                .cursor_pointer()
                .h(px(rh / 2.0))
                .child("\u{25B4}")
                .on_click(cx.listener(|this, _event, _window, cx| {
                    access(this).step_rate(true);
                    cx.notify();
                })),
        )
        .child(
            crate::probe::measured("cot-rate-down", div())
                .id("cot-rate-down")
                .cursor_pointer()
                .h(px(rh / 2.0))
                .child("\u{25BE}")
                .on_click(cx.listener(|this, _event, _window, cx| {
                    access(this).step_rate(false);
                    cx.notify();
                })),
        );
    let bauds: Vec<String> = BAUDS.iter().map(|b| (*b).to_owned()).collect();
    let client = div()
        .relative()
        .w(px(CLIENT.0))
        .h(px(CLIENT.1))
        .child(group)
        .child(check(
            "cot-CB_advancedMode",
            "Additional Details",
            form.advanced,
            ADVANCED_AT,
            false,
            cx,
        ))
        .child(check(
            "cot-chk_indent",
            "Nice Formatting",
            form.indent,
            INDENT_AT,
            true,
            cx,
        ))
        .child(label("Type", TYPE_LABEL_AT))
        .child(text_box(
            "cot-TB_xml_type".to_owned(),
            form.xml_type.clone(),
            TYPE_AT,
            editing(Editing::Type),
            &form.field,
            typing,
            Editing::Type,
            cx,
        ))
        .child(label("Interval (sec)", RATE_LABEL_AT))
        .child(text_box(
            "cot-updateRate".to_owned(),
            mp_mission::dotnet::general_f64(form.rate),
            RATE_AT,
            editing(Editing::Rate),
            &form.field,
            typing,
            Editing::Rate,
            cx,
        ))
        .child(arrows)
        .child(button(
            "cot-BTN_clear_TB",
            "Clear Window",
            CLEAR_AT,
            true,
            |this, _window, _cx| access(this).clear(),
            cx,
        ))
        .child(output)
        .child(grid(holder, form, typing, cx))
        .child(button(
            "cot-BUT_connect",
            holder.button(),
            CONNECT_AT,
            holder.opening.is_none(),
            |this, window, cx| {
                window.blur(cx);
                with_settings(this, |cot, persisted| cot.press_connect(persisted));
            },
            cx,
        ))
        // The two combos last, so a list that drops down lies over what is below it.
        .child(combo(
            "cot-CMB_baudrate",
            &form.baud,
            &bauds,
            form.baud_open,
            form.baud_enabled(),
            BAUD_AT,
            |this| access(this).toggle_list(true),
            |this, index| access(this).choose(true, index),
            cx,
        ))
        .child(combo(
            "cot-CMB_serialport",
            &form.port,
            &form.ports,
            form.port_open,
            true,
            PORT_AT,
            |this| access(this).toggle_list(false),
            |this, index| access(this).choose(false, index),
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
            "cot-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                with_settings(this, |cot, persisted| cot.close(persisted));
                cx.notify();
            }),
        ));
    let body = crate::probe::measured("cot", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    let over = div()
        .id("cot-backdrop")
        .w(size.width)
        .h(size.height)
        .flex()
        .items_center()
        .justify_center()
        .occlude()
        .child(body);
    let mut layers = div().child(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(over),
        )
        .with_priority(1),
    );
    if let Some(asking) = form.questions.asking.as_ref() {
        layers = layers.child(input_box(
            "cot-prompt-box",
            &asking.input,
            prompt,
            window,
            |this, event| {
                let outcome = access(this)
                    .window
                    .as_mut()
                    .and_then(|form| form.questions.asking.as_mut())
                    .map(|asking| asking.input.field.key(event));
                match outcome {
                    Some(KeyOutcome::Submitted) => {
                        finish_question(this, true);
                        true
                    }
                    Some(KeyOutcome::Cancelled) => {
                        finish_question(this, false);
                        true
                    }
                    Some(KeyOutcome::Changed) => true,
                    _ => false,
                }
            },
            |this| finish_question(this, true),
            |this| finish_question(this, false),
            cx,
        ));
    }
    Some(layers.into_any_element())
}

fn finish_question(this: &mut MissionPlanner, ok: bool) {
    with_settings(this, |cot, persisted| cot.answered(ok, persisted));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// testdata/cot: the cases and the C#'s text for each (tools/csharp-reference/regen-cot.sh).
    fn testdata(name: &str) -> String {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/cot")
            .join(name);
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))
    }

    /// Every case's event is the text `getXmlString` makes, byte for byte: Nice Formatting's
    /// lines and its one line, Additional Details whole and in part, the characters escaped,
    /// .NET's rounding and its zero without a sign, the hundredths cut.
    /// `// C#: Controls/SerialOutputCoT.cs:263-360`
    #[test]
    fn every_event_is_the_csharps_text() {
        let cases = testdata("cases.txt");
        let mut checked = 0;
        for line in cases
            .lines()
            .filter(|l| !l.is_empty() && !l.starts_with('#'))
        {
            let f: Vec<&str> = line.split('|').collect();
            let number = |i: usize| f[i].parse::<f64>().expect("a number");
            let time = chrono::NaiveDateTime::parse_from_str(f[9], "%Y-%m-%dT%H:%M:%S%.f")
                .expect("a time")
                .and_utc();
            let given = |i: usize| (!f[i].is_empty()).then(|| f[i].to_owned());
            let event = Event {
                uid: f[1].to_owned(),
                kind: f[2].to_owned(),
                how: f[3].to_owned(),
                lat: number(4),
                lng: number(5),
                alt: number(6),
                course: number(7),
                speed: number(8),
                time,
                takv: f[11] == "1",
                callsign: given(12),
                endpoint: given(13),
                vmf: given(14),
            };
            let golden = testdata(&format!("golden/{}.xml", f[0]));
            assert_eq!(
                event_xml(&event, f[10] == "1").as_deref(),
                Ok(golden.as_str()),
                "{}",
                f[0]
            );
            checked += 1;
        }
        assert_eq!(checked, 8);
    }

    /// A character XML may not hold is what the C#'s writer throws for.
    #[test]
    fn a_character_xml_cannot_hold_throws() {
        let event = Event {
            uid: "bell\u{7}".to_owned(),
            kind: XML_TYPE.to_owned(),
            how: HOW.to_owned(),
            lat: 0.0,
            lng: 0.0,
            alt: 0.0,
            course: 0.0,
            speed: 0.0,
            time: chrono::Utc::now(),
            takv: false,
            callsign: None,
            endpoint: None,
            vmf: None,
        };
        assert!(event_xml(&event, true).is_err());
    }

    fn row(cells: [Option<serde_json::Value>; 6]) -> Row {
        Row { cells }
    }

    fn text(value: &str) -> Option<serde_json::Value> {
        Some(serde_json::Value::String(value.to_owned()))
    }

    /// The grid as `CoTUID` holds it - Newtonsoft's `object[,]`, nulls and the check's bool - and
    /// read back; the UID by sysid and the row by UID as the C# finds them.
    /// `// C#: Utilities/ExtensionsMP.cs:167-189; Controls/SerialOutputCoT.cs:235-256`
    #[test]
    fn the_grid_is_saved_and_looked_up_as_the_csharp_does() {
        let rows = vec![
            row([
                text("1"),
                text("UAV-1"),
                Some(serde_json::Value::Bool(true)),
                text("Hawk"),
                None,
                None,
            ]),
            row([text("2"), None, None, None, None, None]),
            row([text("1"), text("second"), None, None, None, None]),
        ];
        let json = serialize(&rows);
        assert_eq!(
            json,
            r#"[["1","UAV-1",true,"Hawk",null,null],["2",null,null,null,null,null],["1","second",null,null,null,null]]"#
        );
        assert_eq!(deserialize(&json), Some(rows.clone()));
        // A number in the JSON reads as its text, as `Value?.ToString()` reads an Int64.
        let numbers = deserialize(r#"[[3,"X",null,null,null,null]]"#).expect("rows");
        assert_eq!(uid_for(&numbers, 3).as_deref(), Some("X"));
        assert_eq!(deserialize("not json"), None);
        assert_eq!(
            uid_for(&rows, 1).as_deref(),
            Some("UAV-1"),
            "the first row for the sysid"
        );
        assert_eq!(uid_for(&rows, 2), None, "a row with no UID gives none");
        assert_eq!(uid_for(&rows, 7).as_deref(), Some("NOsysid7"));
        assert_eq!(row_for(&rows, "UAV-1"), rows.first());
        assert_eq!(row_for(&rows, ""), None);
    }

    /// A vehicle's event: its row's UID; with Additional Details its takv - a string "True" read
    /// as `Convert.ToBoolean` reads it, anything else what it throws - callsign and VMF.
    /// `// C#: Controls/SerialOutputCoT.cs:213-234, 303-331`
    #[test]
    fn a_vehicles_event_takes_its_rows_details_with_additional_details() {
        let mut inputs = Inputs {
            vehicles: Vec::new(),
            xml_type: XML_TYPE.to_owned(),
            rows: vec![row([
                text("1"),
                text("UAV-1"),
                text("True"),
                text("Hawk"),
                None,
                text("77"),
            ])],
            advanced: true,
            indent: false,
        };
        let at = Position {
            lat: -35.0,
            lng: 149.0,
            alt: 100.0,
            speed: 5.0,
            course: 90.0,
        };
        let time = chrono::Utc::now();
        let xml = vehicle_xml(&inputs, 1, &at, time).expect("an event");
        assert!(xml.contains(r#"uid="UAV-1""#), "{xml}");
        assert!(xml.contains("<takv />"), "{xml}");
        assert!(xml.contains(r#"<contact callsign="Hawk" />"#), "{xml}");
        assert!(xml.contains(r#"<uid vmf="77" />"#), "{xml}");
        inputs.advanced = false;
        let plain = vehicle_xml(&inputs, 1, &at, time).expect("an event");
        assert!(
            !plain.contains("takv") && !plain.contains("contact"),
            "{plain}"
        );
        assert!(
            vehicle_xml(&inputs, 2, &at, time)
                .expect("an event")
                .contains(r#"uid="NOsysid2""#)
        );
        inputs.advanced = true;
        inputs.rows[0].cells[TAKV] = text("yes");
        assert!(vehicle_xml(&inputs, 1, &at, time).is_err());
    }

    /// `Load` from the settings - the grid, the interval, the indexes, the two boxes - the baud
    /// enabled only for a "com" port, and `FormClosing` writing them back.
    /// `// C#: Controls/SerialOutputCoT.cs:367-405, 424-430`
    #[test]
    fn the_form_loads_and_saves_its_settings() {
        let mut persisted = Persisted::at(None);
        let mut cot = CotOutput::default();
        cot.show(&mut persisted);
        let form = cot.window.as_ref().expect("the form");
        assert_eq!(form.port, "TAK Multicast");
        assert_eq!(
            &form.ports[..5],
            [
                "TAK Multicast",
                "TCP Host - 14551",
                "TCP Client",
                "UDP Host - 14551",
                "UDP Client"
            ]
        );
        assert_eq!(form.baud, "4800");
        assert!(!form.baud_enabled(), "TAK Multicast has no baud");
        assert_eq!(form.xml_type, XML_TYPE);
        assert_eq!(form.rate, RATE);
        assert!(form.advanced && form.indent);
        assert_eq!(cot.button(), CONNECT);
        cot.choose(false, 2);
        cot.choose(true, 7);
        cot.toggle_check(false);
        cot.step_rate(false);
        cot.begin(Editing::Cell(0, 0), &mut persisted);
        cot.window.as_mut().expect("the form").field.set("1");
        cot.begin(Editing::Cell(0, 1), &mut persisted);
        cot.window.as_mut().expect("the form").field.set("UAV-1");
        cot.begin(Editing::Cell(0, TAKV), &mut persisted);
        assert_eq!(
            persisted.get(UID_SETTING),
            Some(r#"[["1","UAV-1",true,null,null,null]]"#)
        );
        cot.close(&mut persisted);
        assert_eq!(persisted.get(PORT_SETTING), Some("2"));
        assert_eq!(persisted.get(BAUD_SETTING), Some("7"));
        assert_eq!(persisted.get(ADVANCED_SETTING), Some("False"));
        assert_eq!(persisted.get(INDENT_SETTING), Some("True"));
        assert_eq!(persisted.get(RATE_SETTING), Some("9"));
        cot.show(&mut persisted);
        let form = cot.window.as_ref().expect("the form");
        assert_eq!(
            (form.port.as_str(), form.baud.as_str()),
            ("TCP Client", "115200")
        );
        assert!(!form.advanced);
        assert_eq!(form.rate, 9.0);
        assert_eq!(form.rows.len(), 1);
        assert!(
            !cot.column_shown(2),
            "Additional Details off hides the details"
        );
    }

    /// Connect on UDP Client, its two questions answered: the thread writes each vehicle's event
    /// a line, carriage returns out, at the interval; the box holds them; Stop ends it at once.
    /// `// C#: Controls/SerialOutputCoT.cs:40-193`
    #[test]
    fn connect_writes_each_vehicles_event_a_line_and_stop_ends_it() {
        let listener = std::net::UdpSocket::bind("127.0.0.1:0").expect("a socket");
        listener
            .set_read_timeout(Some(std::time::Duration::from_secs(5)))
            .expect("a timeout");
        let port = listener.local_addr().expect("an address").port();
        let mut persisted = Persisted::at(None);
        let mut cot = CotOutput::default();
        cot.show(&mut persisted);
        cot.choose(false, 4);
        // A second's interval: the first round goes before the vehicle is handed over.
        cot.window.as_mut().expect("the form").rate = RATE_MIN;
        cot.press_connect(&mut persisted);
        let answer = |cot: &mut CotOutput, persisted: &mut Persisted, text: &str| {
            let form = cot.window.as_mut().expect("the form");
            form.questions
                .asking
                .as_mut()
                .expect("a question")
                .input
                .field
                .set(text);
            cot.answered(true, persisted);
        };
        answer(&mut cot, &mut persisted, "127.0.0.1");
        answer(&mut cot, &mut persisted, &port.to_string());
        assert!(cot.running());
        assert_eq!(cot.button(), STOP);
        assert_eq!(cot.finish_opening(), None);
        let vehicle = (
            VehicleId::new(1, 1),
            Position {
                lat: -35.3632621,
                lng: 149.1652374,
                alt: 584.0,
                speed: 0.0,
                course: 0.0,
            },
        );
        cot.tick(vec![vehicle]);
        let mut buf = [0u8; 2048];
        let n = listener.recv(&mut buf).expect("an event");
        let line = std::str::from_utf8(&buf[..n]).expect("text");
        assert!(line.ends_with("</event>\n"), "{line}");
        assert!(!line.contains('\r'), "{line}");
        assert!(
            line.contains(r#"uid="NOsysid1""#) && line.contains(r#"lat="-35.3632621""#),
            "{line}"
        );
        assert!(cot.written() >= 1);
        let started = Instant::now();
        while !cot.output().contains("NOsysid1") && started.elapsed() < Duration::from_secs(5) {
            wasm_thread::sleep(Duration::from_millis(10));
        }
        assert!(
            cot.output().contains("\r\n"),
            "the box keeps the C#'s line ends"
        );
        cot.press_connect(&mut persisted);
        assert!(!cot.running());
        assert_eq!(cot.button(), CONNECT);
    }

    /// TAK Multicast: a UDP client to 239.2.3.1:6969, asking nothing, its address kept as the
    /// client's settings under `TAK_Multicast`. `// C#: Controls/SerialOutputCoT.cs:85-89`
    #[test]
    fn tak_multicast_asks_nothing_and_keeps_its_address() {
        assert_eq!(Kind::of("TAK Multicast"), Kind::TakMulticast);
        assert!(Kind::TakMulticast.questions("").is_empty());
        let mut persisted = Persisted::at(None);
        let mut cot = CotOutput::default();
        cot.show(&mut persisted);
        cot.press_connect(&mut persisted);
        assert!(cot.running());
        assert_eq!(persisted.get("UDP_hostTAK_Multicast"), Some(TAK_HOST));
        assert_eq!(persisted.get("UDP_portTAK_Multicast"), Some("6969"));
        cot.press_connect(&mut persisted);
        assert!(!cot.running());
    }

    /// The GUI script names only facts and controls this window has.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_window_has() {
        let script = include_str!("../../../../tests/gui/experimental-cot.gui");
        let source = include_str!("cot_output.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.cot.") => {
                    let head: String = key.split('.').take(3).collect::<Vec<_>>().join(".");
                    assert!(
                        source.contains(&format!("\"{head}\""))
                            || source.contains(&format!("\"{head}.")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("cot-") => {
                    let fixed = id.split('@').next().unwrap_or(id);
                    let drawn = source.contains(&format!("\"{fixed}\""))
                        || fixed.starts_with("cot-cell-")
                        || fixed.starts_with("cot-CMB_");
                    assert!(drawn, "{id} is not drawn");
                }
                _ => {}
            }
        }
        assert!(facts > 10);
    }
}
