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

//! The MAVLink Inspector: `Controls/MAVLinkInspector.cs`, the window SETUP's Advanced page's
//! "MAVLink Inspector" opens (`ConfigAdvanced.but_mavinspector_Click`,
//! `GCSViews/ConfigurationView/ConfigAdvanced.cs:27-30`: `new MAVLinkInspector(MainV2.comPort)
//! .Show()`).
//!
//! What it shows, as `InitializeComponent` builds it in a 526 x 311 form (`MAVLinkInspector.cs:
//! 176-272`; its `.resx` holds only the timer's tray place): Graph It and "Show GCS Traffic" along
//! the top, and under them, in a group box filling the rest, a tree of every message heard since
//! the window opened -
//!
//! * a node per system, "Vehicle 1";
//! * under it a node per component, "Comp 1 MAV_COMP_ID_AUTOPILOT1": the id and its
//!   `MAV_COMPONENT` name, or the id again for one the enum lacks (`:85`);
//! * under that a node per message, "ATTITUDE (10.0 Hz, #30) 390Bps": its name, its rate and its
//!   bytes a second over the last three seconds (`:108-114`), from `PacketInspector`
//!   ([`mp_link::inspector::PacketInspector`]);
//! * under that a node per field of the newest one, `String.Format("{0,-32} {1,20} {2,-20}",
//!   name, value, type)` (`:120-166`): `time_unix_usec` as the date it is, the six text arrays as
//!   ASCII, any other array its elements joined by commas, each number as .NET writes it, and the
//!   type as .NET names it - `System.Single`, `System.Byte[]`.
//!
//! What it does:
//!
//! * every packet the link reads is added as it comes, and every one it writes while "Show GCS
//!   Traffic" is ticked (`:34, 52-55, 448-457`);
//! * every 333 ms the tree is brought up to date (`timer1`, `:45-47, 57-174, 233`): nodes added
//!   for what is new, every message's and field's text rewritten, and each level sorted by its
//!   text when anything was added (`TreeView.Sort`, which compares as the culture does -
//!   [`culture_cmp`]);
//! * selecting a node below a vehicle's enables Graph It and keeps the node's path
//!   (`treeView1_AfterSelect`, `:322-337`);
//! * Graph It asks "Points of history?" (50 at first, kept between clicks), and for a field's node
//!   opens a graph of it (`but_graphit_Click`, `:341-446`): a 640 x 480 form, its curve fed each
//!   packet of that message from that system and component, read or written, at the packet's
//!   time - a curve an element for an array - the Y axis titled with the field's units, the X
//!   axis a date axis written `HH:mm:ss.fff`, drawn again every 100 ms. Graph It is then disabled
//!   until another node is selected. For a component's or a message's node it asks, then does
//!   nothing (`if(path.Length < 4) return;`);
//! * closing the form ends its subscriptions and its timer (`:304-310`), and closes the graphs it
//!   owns (`form.Show(this)`), which end theirs (`:441`).
//!
//! Where this is not the C#, each written at its site:
//!
//! * the form is drawn over SETUP, modal, where the C#'s is a free form (`Show`, not
//!   `ShowDialog`); a second click replaces it with a fresh one, as the FFT window is replaced
//!   (`config/fft.rs`). Its graphs are drawn over it, one below another's corner, each with a
//!   close box;
//! * the form does not resize, so the group box's anchors never act;
//! * the tree's text is drawn in the application's font, each column where Courier New's fixed
//!   advance puts it; the tree draws its plus and minus boxes but not its dotted lines, and the
//!   keyboard does not move its selection;
//! * a field's value with a NUL in it is shown up to the NUL, as the Win32 tree draws a string;
//! * the graph is the part of ZedGraph the handler sets up - its curves, legend, axis titles and
//!   date and linear scales ([`DateScale`], [`crate::plan::elevation::Scale`]) - not ZedGraph's
//!   zoom, pan and context menu, nor `IsPreventLabelOverlap`'s widening of a step to fit
//!   measured labels; the Y axis's title is written level, over the axis;
//! * the subscriptions are made again when the application's link is replaced by another - a
//!   connect, a reopen - where the C#'s one `comPort` keeps its subscribers across them;
//! * what the C# throws - an answer to "Points of history?" that is not a whole number, a
//!   negative one - goes on the status line (the owner's ruling of 2026-09-25); the C# shows its
//!   unhandled-exception box. A history of 0, which the C#'s `RollingPointPairList` takes and
//!   then throws on at its first point inside the link's event, gives a graph that stays empty.
//!
//! Not ported: `treeView1_DrawNode` is wired but never raised - the tree's `DrawMode` is left
//! `Normal`, and only an owner-drawn tree raises it (dead code, PLAN §12 D16); `comboBox1` and
//! `comboBox2`, which `NewSysidCompid` fills, are invisible (`:220, 229`).

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cmp::Ordering;
use std::collections::{BTreeSet, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering as Atomic};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use chrono::{Datelike as _, NaiveDate, NaiveDateTime, Timelike as _};
use gpui::{
    AnyElement, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px, rgb,
};
use mp_link::inspector::{Packet, PacketInspector, PacketSubscription};
use mp_mavlink::{FieldInfo, FieldValue};
use mp_mavlink_dialects::all::{MavComponent, MavMessage};
use mp_mission::dotnet::{format_f64, general_f32, general_f64};
use mp_vehicle::DateTime;

use super::optional::{InputBox, at, button, input_box};
use crate::MissionPlanner;
use crate::plan::elevation::Scale;
use crate::telemetry::Telemetry;
use crate::textfield::KeyOutcome;
use crate::ui::theme;

// ---------------------------------------------------------------------------------------------
// The words, places and numbers of `InitializeComponent` and the handlers.
// ---------------------------------------------------------------------------------------------

/// The form's caption, `this.Text`. `// C#: Controls/MAVLinkInspector.cs:266`
pub const FORM_TEXT: &str = "Mavlink Inspector";

/// `ClientSize`. `// C#: Controls/MAVLinkInspector.cs:259`
pub const CLIENT: (f32, f32) = (526.0, 311.0);

/// `but_graphit.Text`. `// C#: Controls/MAVLinkInspector.cs:242`
pub const GRAPH_IT: &str = "Graph It";
/// `but_graphit`'s `Location` and `Size`. `// C#: Controls/MAVLinkInspector.cs:238-240`
const GRAPH_IT_AT: (f32, f32, f32, f32) = (12.0, 3.0, 75.0, 23.0);

/// `chk_gcstraffic.Text`. `// C#: Controls/MAVLinkInspector.cs:253`
pub const GCS_TRAFFIC: &str = "Show GCS Traffic";
/// `chk_gcstraffic.Location`. `// C#: Controls/MAVLinkInspector.cs:250`
const GCS_TRAFFIC_AT: (f32, f32) = (93.0, 5.0);

/// `groupBox1`'s `Location` and `Size`: no `Text`, so an empty frame round the tree.
/// `// C#: Controls/MAVLinkInspector.cs:207-209`
const GROUP_AT: (f32, f32, f32, f32) = (0.0, 30.0, 527.0, 278.0);
/// `treeView1`'s `Location` and `Size` in the group box (`Dock = Fill`).
/// `// C#: Controls/MAVLinkInspector.cs:191-196`
const TREE_AT: (f32, f32, f32, f32) = (3.0, 16.0, 521.0, 259.0);

/// `timer1.Interval`: how often the tree is brought up to date.
/// `// C#: Controls/MAVLinkInspector.cs:233`
pub const UPDATE_EVERY: Duration = Duration::from_millis(333);

/// `int history = 50`: the points a graph keeps, until Graph It's question is answered.
/// `// C#: Controls/MAVLinkInspector.cs:339`
pub const HISTORY: i32 = 50;

/// `InputBox.Show("Points", "Points of history?", ref history)`: the caption.
/// `// C#: Controls/MAVLinkInspector.cs:343`
pub const POINTS_TITLE: &str = "Points";
/// The question.
pub const POINTS_PROMPT: &str = "Points of history?";

/// The graph's form: `new Form() { Size = new Size(640, 480) }`, caption and borders included.
/// `// C#: Controls/MAVLinkInspector.cs:344`
pub const GRAPH_SIZE: (f32, f32) = (640.0, 480.0);

/// `color`: a curve's colour by its element - `Red`, `Green`, `Blue`, `Black`, `Violet`,
/// `Orange` - the first curve `Color.Red` whatever it holds.
/// `// C#: Controls/MAVLinkInspector.cs:358, 381-382, 405-406`
pub const COLOURS: [u32; 6] = [
    0xff_00_00, 0x00_80_00, 0x00_00_ff, 0x00_00_00, 0xee_82_ee, 0xff_a5_00,
];

/// The graph's `timer.Interval`: how often it is scaled and drawn again.
/// `// C#: Controls/MAVLinkInspector.cs:383, 430-439`
pub const GRAPH_EVERY: Duration = Duration::from_millis(100);

/// `zg1.GraphPane.XAxis.Scale.Format`, `HH:mm:ss.fff`, as chrono writes it.
/// `// C#: Controls/MAVLinkInspector.cs:377`
pub const X_FORMAT: &str = "%H:%M:%S%.3f";

/// The X axis's title: ZedGraphControl's default, which the handler leaves.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.cs (new GraphPane(rect, "Title", "X Axis", "Y Axis"))`
pub const X_TITLE: &str = super::fftui::BLANK_X;
/// The Y axis's title where the field's `Units` could not be read.
pub const Y_TITLE: &str = super::fftui::BLANK_Y;

/// The byte arrays shown as ASCII: `param_id`, `text`, `model_name`, `vendor_name`, `uri` and
/// `cam_definition_uri`.
/// `// C#: Controls/MAVLinkInspector.cs:148-153`
pub const TEXT_FIELDS: [&str; 6] = [
    "param_id",
    "text",
    "model_name",
    "vendor_name",
    "uri",
    "cam_definition_uri",
];

/// `FormatException.Message`, for an answer that is not a whole number.
pub const NOT_A_NUMBER: &str = super::fftui::NOT_A_NUMBER;
/// `OverflowException.Message`, for a whole number past an `Int32`.
pub const TOO_BIG: &str = "Value was either too large or too small for an Int32.";
/// `OverflowException.Message` again, for `new PointPair[capacity]` of a negative capacity.
pub const NEGATIVE_HISTORY: &str = "Arithmetic operation resulted in an overflow.";
/// `IndexOutOfRangeException.Message`, for a path's word that is not there.
pub const OUT_OF_RANGE: &str = super::fftui::OUT_OF_RANGE;

/// Courier New 8.25 pt's advance at 96 dpi, the tree's `Font`: where its columns fall.
/// `// C#: Controls/MAVLinkInspector.cs:192`
const CHAR_WIDTH: f32 = 7.0;
/// A tree row: the font's height and the tree's padding, `ItemHeight`'s default for it.
pub(crate) const ROW_HEIGHT: f32 = 16.0;
/// `TreeView.Indent`'s default: how far each level is set in.
pub(crate) const INDENT: f32 = 19.0;

// ---------------------------------------------------------------------------------------------
// The tree.
// ---------------------------------------------------------------------------------------------

/// A `TreeNode`: its `Name`, which `Nodes.Find` looks it up by - a system id, a component id, a
/// message id or a field's name - its `Text`, its `Tag` and its children.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Node {
    /// `Name`.
    pub name: String,
    /// `Text`.
    pub text: String,
    /// A field's `Tag`, less its name: the value and the type the text is made of.
    pub tag: Option<(String, String)>,
    /// `Nodes`.
    pub nodes: Vec<Node>,
}

/// `Nodes.Find(name, false)`, or a new node of `text` added when there is none - which sets
/// `added`. WinForms finds a key whatever its case.
/// `// C#: Controls/MAVLinkInspector.cs:69-106, 122-126`
pub(crate) fn child<'a>(
    nodes: &'a mut Vec<Node>,
    name: &str,
    text: impl FnOnce() -> String,
    added: &mut bool,
) -> Option<&'a mut Node> {
    if let Some(index) = nodes
        .iter()
        .position(|node| node.name.eq_ignore_ascii_case(name))
    {
        return nodes.get_mut(index);
    }
    nodes.push(Node {
        name: name.to_owned(),
        text: text(),
        ..Node::default()
    });
    *added = true;
    nodes.last_mut()
}

/// A component's node: `"Comp " + compid + " " + (MAVLink.MAV_COMPONENT) compid`, the enum
/// writing the number itself for a value it does not name.
/// `// C#: Controls/MAVLinkInspector.cs:85`
#[must_use]
pub fn component_text(compid: u8) -> String {
    let id = u32::from(compid);
    let name = MavComponent(id)
        .name()
        .map_or_else(|| id.to_string(), str::to_owned);
    format!("Comp {compid} {name}")
}

/// A message's node: `msgtypename + " (" + rate.ToString("0.0 Hz") + ", #" + msgid + ") " +
/// bps.ToString("0Bps")`.
/// `// C#: Controls/MAVLinkInspector.cs:108-111`
#[must_use]
pub fn header_text(name: &str, msgid: u32, rate: f64, bps: f64) -> String {
    format!(
        "{name} ({}, #{msgid}) {}",
        format_f64(rate, "0.0 Hz"),
        format_f64(bps, "0Bps")
    )
}

/// `field.FieldType.ToString()`: the .NET type the C#'s struct declares the field as - `char`
/// and `uint8_t` both `byte` (`ExtLibs/Mavlink/pymavlink/generator/mavgen_cs.py:16-29`).
#[must_use]
pub fn type_name(info: &FieldInfo) -> String {
    let base = match info.base_type {
        "float" => "System.Single",
        "double" => "System.Double",
        "int8_t" => "System.SByte",
        "int16_t" => "System.Int16",
        "uint16_t" => "System.UInt16",
        "int32_t" => "System.Int32",
        "uint32_t" => "System.UInt32",
        "int64_t" => "System.Int64",
        "uint64_t" => "System.UInt64",
        // `char`, `uint8_t` and `uint8_t_mavlink_version`.
        _ => "System.Byte",
    };
    if info.array_len > 0 {
        format!("{base}[]")
    } else {
        base.to_owned()
    }
}

/// A number as `ToString()` writes it: an integer in full, a `float` to seven significant
/// digits and a `double` to fifteen, as the .NET Framework does.
#[allow(clippy::cast_possible_truncation)] // a float field's value, widened from an f32 exactly
fn number_text(value: f64, single: bool) -> String {
    if single {
        general_f32(value as f32)
    } else {
        general_f64(value)
    }
}

/// `date1.AddMilliseconds((ulong)value / 1000)` written as `DateTime.ToString()` writes it in
/// English, `M/d/yyyy h:mm:ss tt`; `None` past `DateTime.MaxValue`, where `AddMilliseconds`
/// throws and the `catch` keeps the number.
/// `// C#: Controls/MAVLinkInspector.cs:130-140`
fn unix_date_text(usec: u64) -> Option<String> {
    /// `DateTime.MaxValue` in milliseconds after 1970.
    const MAX_MS: u64 = 253_402_300_799_999;
    let ms = usec / 1000;
    if ms > MAX_MS {
        return None;
    }
    let time = chrono::DateTime::from_timestamp_millis(i64::try_from(ms).ok()?)?;
    Some(time.format("%-m/%-d/%Y %-I:%M:%S %p").to_string())
}

/// `Encoding.ASCII.GetString`: a byte past 127 is `?`, a NUL is kept.
fn ascii(bytes: &[u64]) -> String {
    bytes
        .iter()
        .map(|byte| {
            u8::try_from(*byte)
                .ok()
                .filter(u8::is_ascii)
                .map_or('?', char::from)
        })
        .collect()
}

/// A field's value as the tree writes it.
/// `// C#: Controls/MAVLinkInspector.cs:128-158`
#[must_use]
pub fn value_text(info: &FieldInfo, value: &FieldValue) -> String {
    let single = info.base_type == "float";
    let join = |items: Vec<String>| items.join(",");
    match value {
        // `(ulong)value`: only a `ulong` is a date; any other type's cast throws and is kept.
        FieldValue::Unsigned(usec)
            if info.name == "time_unix_usec" && info.base_type == "uint64_t" =>
        {
            unix_date_text(*usec).unwrap_or_else(|| usec.to_string())
        }
        FieldValue::Unsigned(value) => value.to_string(),
        FieldValue::Signed(value) => value.to_string(),
        FieldValue::Float(value) => number_text(*value, single),
        FieldValue::UnsignedArray(values) if TEXT_FIELDS.contains(&info.name) => ascii(values),
        // `value2.Cast<object>().Aggregate((a, b) => a + "," + b)`.
        FieldValue::UnsignedArray(values) => join(values.iter().map(ToString::to_string).collect()),
        FieldValue::SignedArray(values) => join(values.iter().map(ToString::to_string).collect()),
        FieldValue::FloatArray(values) => join(
            values
                .iter()
                .map(|value| number_text(*value, single))
                .collect(),
        ),
    }
}

/// A field's node: `String.Format("{0,-32} {1,20} {2,-20}", field.Name, value, type)`.
/// `// C#: Controls/MAVLinkInspector.cs:165-166`
#[must_use]
pub fn field_text(name: &str, value: &str, type_name: &str) -> String {
    format!("{name:<32} {value:>20} {type_name:<20}")
}

/// What the tree draws of a text: up to its first NUL, as Win32 draws a string.
#[must_use]
pub fn shown(text: &str) -> &str {
    text.split('\0').next().unwrap_or(text)
}

/// Where a character sorts in `CompareInfo.Compare` with no options: its primary weight and
/// whether it is upper case, or `None` for one the comparison ignores (a control character,
/// NUL among them). White space first, then punctuation in the culture's order - `_` before
/// the rest - then the digits, then the letters with their case set aside.
fn weight(c: char) -> Option<(u32, bool)> {
    /// The ASCII punctuation in the order the invariant culture sorts it.
    const PUNCTUATION: &str = "_-,;:!?.'\"()[]{}@*/\\&#%`^+<=>|~$";
    if c.is_control() {
        return None;
    }
    if c.is_whitespace() {
        return Some((0, false));
    }
    let code = u32::from(c);
    if let Some(index) = PUNCTUATION.find(c) {
        return Some((1 + u32::try_from(index).unwrap_or(0), false));
    }
    if c.is_ascii_digit() {
        return Some((100 + code - u32::from('0'), false));
    }
    if c.is_ascii_alphabetic() {
        let lower = u32::from(c.to_ascii_lowercase());
        return Some((200 + lower - u32::from('a'), c.is_ascii_uppercase()));
    }
    Some((1000 + code, false))
}

/// `TreeView.Sort()`'s comparison of two nodes' texts: `CompareInfo.Compare` in the culture,
/// which is not ordinal - `GPS_RAW_INT` sorts before `GPS2_RAW`, `chan1_raw` before
/// `chan10_raw`, a name before a longer one it begins. The letters' case decides only where
/// nothing else does, lower case first; then the characters themselves.
#[must_use]
pub fn culture_cmp(a: &str, b: &str) -> Ordering {
    let weights = |text: &str| text.chars().filter_map(weight).collect::<Vec<_>>();
    let (left, right) = (weights(a), weights(b));
    left.iter()
        .map(|w| w.0)
        .cmp(right.iter().map(|w| w.0))
        .then_with(|| left.iter().map(|w| w.1).cmp(right.iter().map(|w| w.1)))
        .then_with(|| a.cmp(b))
}

/// Each level sorted by its text.
pub(crate) fn sort(nodes: &mut [Node]) {
    nodes.sort_by(|a, b| culture_cmp(&a.text, &b.text));
    for node in nodes {
        sort(&mut node.nodes);
    }
}

/// `Update()`: the tree brought up to date with what `mavi` holds at `now` - a node added for
/// each system, component, message and field not seen before, each message's and field's text
/// written again, and, when anything was added, every level sorted. True when something was.
/// `// C#: Controls/MAVLinkInspector.cs:57-174`
pub fn update(tree: &mut Vec<Node>, mavi: &PacketInspector<Packet>, now: Instant) -> bool {
    let mut added = false;
    for packet in mavi.packet_messages() {
        let (sysid, compid, msgid) = (packet.sysid, packet.compid, packet.msgid);
        let Some(vehicle) = child(
            tree,
            &sysid.to_string(),
            || format!("Vehicle {sysid}"),
            &mut added,
        ) else {
            continue;
        };
        let Some(component) = child(
            &mut vehicle.nodes,
            &compid.to_string(),
            || component_text(compid),
            &mut added,
        ) else {
            continue;
        };
        let name = packet.message.name();
        let Some(message) = child(
            &mut component.nodes,
            &msgid.to_string(),
            || name.to_owned(),
            &mut added,
        ) else {
            continue;
        };
        message.text = header_text(
            name,
            msgid,
            mavi.seen_rate(sysid, compid, msgid, now),
            mavi.seen_bps(sysid, compid, msgid, now),
        );
        for (info, (field, value)) in packet
            .message
            .field_info()
            .iter()
            .zip(packet.message.fields())
        {
            let Some(node) = child(&mut message.nodes, field, String::new, &mut added) else {
                continue;
            };
            let value = value_text(info, &value);
            let type_name = type_name(info);
            node.text = field_text(field, &value, &type_name);
            node.tag = Some((value, type_name));
        }
    }
    if added {
        sort(tree);
    }
    added
}

/// The node a path of `Name`s leads to.
#[must_use]
pub fn node_at<'a>(tree: &'a [Node], key: &[String]) -> Option<&'a Node> {
    let (first, rest) = key.split_first()?;
    let node = tree.iter().find(|node| &node.name == first)?;
    if rest.is_empty() {
        Some(node)
    } else {
        node_at(&node.nodes, rest)
    }
}

/// `TreeNode.FullPath`: the texts from the root to the node, joined by `PathSeparator`, `\`.
#[must_use]
pub fn full_path(tree: &[Node], key: &[String]) -> Option<String> {
    let mut texts = Vec::new();
    let mut level = tree;
    for name in key {
        let node = level.iter().find(|node| &node.name == name)?;
        texts.push(node.text.clone());
        level = &node.nodes;
    }
    Some(texts.join("\\"))
}

/// One row the tree shows: where it is, how deep, its text, and its box.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Row {
    /// The `Name`s from the root to it.
    pub key: Vec<String>,
    /// Its level, 0 for a vehicle.
    pub depth: usize,
    /// Its `Text`.
    pub text: String,
    /// A field's value and type.
    pub tag: Option<(String, String)>,
    /// Whether it has children, and so a plus or minus box.
    pub parent: bool,
    /// Whether they show.
    pub expanded: bool,
}

/// The rows showing: each node, and the children of each expanded one under it.
pub(crate) fn rows_of(
    nodes: &[Node],
    prefix: &[String],
    expanded: &BTreeSet<Vec<String>>,
    out: &mut Vec<Row>,
) {
    for node in nodes {
        let mut key = prefix.to_vec();
        key.push(node.name.clone());
        let open = expanded.contains(&key);
        out.push(Row {
            key: key.clone(),
            depth: prefix.len(),
            text: node.text.clone(),
            tag: node.tag.clone(),
            parent: !node.nodes.is_empty(),
            expanded: open,
        });
        if open {
            rows_of(&node.nodes, &key, expanded, out);
        }
    }
}

/// A probe id for a node: its `Name`s joined by `-`.
#[must_use]
pub fn row_id(prefix: &str, key: &[String]) -> String {
    format!("inspector-{prefix}-{}", key.join("-"))
}

// ---------------------------------------------------------------------------------------------
// Graph It.
// ---------------------------------------------------------------------------------------------

/// `int.Parse`: optional white space and sign round the digits.
fn parse_int32(text: &str) -> Result<i32, &'static str> {
    let trimmed = text.trim();
    let digits = trimmed.strip_prefix(['+', '-']).unwrap_or(trimmed);
    if digits.is_empty() || !digits.chars().all(|c| c.is_ascii_digit()) {
        return Err(NOT_A_NUMBER);
    }
    trimmed
        .parse::<i64>()
        .ok()
        .and_then(|value| i32::try_from(value).ok())
        .ok_or(TOO_BIG)
}

/// What Graph It reads from the selected node's path.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Target {
    /// `int.Parse(path[0].Split(' ')[1])`.
    pub sysid: i32,
    /// `int.Parse(path[1].Split(' ')[1])`.
    pub compid: i32,
    /// `int.Parse(msgt.Split('#', ')')[1])`.
    pub msgid: i32,
    /// `msgt.Split(' ')[0]`: the message's name.
    pub message: String,
    /// `field.Split(' ')[0]`: the field's.
    pub field: String,
}

/// `selectedmsgid.Split('\\')` read as `but_graphit_Click` reads it: `None` for a path of fewer
/// than four nodes - a component's or a message's - and the C#'s exception for a word that is
/// not there or not a number.
/// `// C#: Controls/MAVLinkInspector.cs:346-356`
pub fn target(path: &str) -> Result<Option<Target>, &'static str> {
    let path: Vec<&str> = path.split('\\').collect();
    let [vehicle, component, message, field, ..] = path.as_slice() else {
        return Ok(None);
    };
    let word = |text: &str, separators: &[char], index: usize| {
        text.split(separators)
            .nth(index)
            .map(str::to_owned)
            .ok_or(OUT_OF_RANGE)
    };
    Ok(Some(Target {
        sysid: parse_int32(&word(vehicle, &[' '], 1)?)?,
        compid: parse_int32(&word(component, &[' '], 1)?)?,
        msgid: parse_int32(&word(message, &['#', ')'], 1)?)?,
        message: word(message, &[' '], 0)?,
        field: word(field, &[' '], 0)?,
    }))
}

/// One curve: its label, its colour and its points - `RollingPointPairList(history)`, the
/// oldest dropped past its capacity.
#[derive(Debug, Clone, PartialEq)]
pub struct Curve {
    /// The label the legend shows.
    pub label: String,
    /// Its colour.
    pub colour: u32,
    /// `(x, y)`: the packet's time as an `XDate`, and the value.
    pub points: VecDeque<(f64, f64)>,
}

impl Curve {
    fn new(label: String, colour: u32) -> Self {
        Self {
            label,
            colour,
            points: VecDeque::new(),
        }
    }
}

/// A graph's curves, which the link thread's handler adds to.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Curves {
    /// `zg1.GraphPane.CurveList`.
    pub list: Vec<Curve>,
    /// Each curve's capacity, `history`.
    pub capacity: usize,
}

/// `new DateTime(1899, 12, 30).Ticks`: day 0 of an `XDate`.
const XDATE_EPOCH_TICKS: i64 = 599_264_352_000_000_000;
/// `TimeSpan.TicksPerMillisecond`.
const TICKS_PER_MS: i64 = 10_000;
/// Milliseconds in a day.
const MS_PER_DAY: f64 = 86_400_000.0;

/// `new XDate(msg.rxtime)`: the time as days since 1899-12-30, to the millisecond, as
/// `CalendarDateToXLDate` takes a `DateTime`'s parts.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/XDate.cs (XDate(DateTime))`
#[must_use]
#[allow(clippy::cast_precision_loss)] // milliseconds since 1899: well inside f64's 53 bits
pub fn xdate(time: DateTime) -> f64 {
    let ms = (time.ticks() - XDATE_EPOCH_TICKS).div_euclid(TICKS_PER_MS);
    ms as f64 / MS_PER_DAY
}

impl Curves {
    /// A graph's one curve, `new LineItem(label, new RollingPointPairList(history), Color.Red,
    /// SymbolType.None)`.
    #[must_use]
    pub fn new(label: String, capacity: usize) -> Self {
        Self {
            list: vec![Curve::new(label, COLOURS[0])],
            capacity,
        }
    }

    /// A point on curve `index`, made as the C# makes a curve an element needs.
    pub(crate) fn push(&mut self, index: usize, field: &str, point: (f64, f64)) {
        while self.list.len() < index + 1 {
            let a = self.list.len();
            self.list.push(Curve::new(
                format!("{field}[{a}]"),
                COLOURS
                    .get(a % COLOURS.len())
                    .copied()
                    .unwrap_or(COLOURS[0]),
            ));
        }
        // A capacity of 0 throws on its first point in the C#, inside the link's event: it
        // never holds one.
        if self.capacity == 0 {
            return;
        }
        if let Some(curve) = self.list.get_mut(index) {
            curve.points.push_back(point);
            while curve.points.len() > self.capacity {
                curve.points.pop_front();
            }
        }
    }

    /// A curve `index` made for `label` if there is none yet, as the C# makes a curve for a
    /// nested type's field before its point.
    pub(crate) fn ensure(&mut self, index: usize, label: &str) {
        while self.list.len() < index + 1 {
            let a = self.list.len();
            let name = if a == index {
                label.to_owned()
            } else {
                format!("{label}[{a}]")
            };
            self.list.push(Curve::new(
                name,
                COLOURS
                    .get(a % COLOURS.len())
                    .copied()
                    .unwrap_or(COLOURS[0]),
            ));
        }
    }

    /// `opr`: a packet of the graphed message from the graphed system and component, read or
    /// written, added at its time - an array a point on each element's curve, anything else a
    /// point on the first.
    /// `// C#: Controls/MAVLinkInspector.cs:386-425`
    #[allow(clippy::cast_precision_loss)] // `IConvertible.ToDouble`, as the C# converts
    pub fn add(&mut self, target: &Target, packet: &Packet) {
        if i64::from(packet.msgid) != i64::from(target.msgid)
            || i32::from(packet.sysid) != target.sysid
            || i32::from(packet.compid) != target.compid
        {
            return;
        }
        let Some((_, value)) = packet
            .message
            .fields()
            .into_iter()
            .find(|(name, _)| *name == target.field)
        else {
            return;
        };
        let x = xdate(packet.rxtime);
        let field = target.field.as_str();
        match value {
            FieldValue::Unsigned(value) => self.push(0, field, (x, value as f64)),
            FieldValue::Signed(value) => self.push(0, field, (x, value as f64)),
            FieldValue::Float(value) => self.push(0, field, (x, value)),
            FieldValue::UnsignedArray(values) => {
                for (a, value) in values.into_iter().enumerate() {
                    self.push(a, field, (x, value as f64));
                }
            }
            FieldValue::SignedArray(values) => {
                for (a, value) in values.into_iter().enumerate() {
                    self.push(a, field, (x, value as f64));
                }
            }
            FieldValue::FloatArray(values) => {
                for (a, value) in values.into_iter().enumerate() {
                    self.push(a, field, (x, value));
                }
            }
        }
    }
}

/// A graph Graph It opened.
#[derive(Debug)]
pub struct Graph {
    /// What it plots.
    pub target: Target,
    /// The Y axis's title: the field's `Units`, `"[rad]"` or `""`.
    pub y_title: String,
    /// The curves, which its handler adds to on the link thread.
    data: Arc<Mutex<Curves>>,
    /// The curves as the timer last drew them.
    pub shown: Curves,
    /// Its `OnPacketReceived` and `OnPacketSent` subscription.
    subscription: Option<PacketSubscription>,
    /// When its timer next ticks.
    next_draw: Instant,
}

/// The Y axis's title for a message's field: its `[Units("...")]`, which mavgen writes as the
/// XML's `units` in brackets, or `""` for a field without; `Y Axis` where the C#'s lookup would
/// fail.
/// `// C#: Controls/MAVLinkInspector.cs:360-372; ExtLibs/Mavlink/pymavlink/generator/mavparse.py:256-258`
#[must_use]
pub fn units_title(message: &MavMessage, field: &str) -> String {
    message
        .field_info()
        .iter()
        .find(|info| info.name == field)
        .map_or_else(
            || Y_TITLE.to_owned(),
            |info| {
                if info.units.is_empty() {
                    String::new()
                } else {
                    format!("[{}]", info.units)
                }
            },
        )
}

impl Graph {
    /// A graph of `target`, holding `history` points a curve.
    fn new(target: Target, y_title: String, history: usize, now: Instant) -> Self {
        let label = format!("{}.{}", target.message, target.field);
        let curves = Curves::new(label, history);
        Self {
            target,
            y_title,
            data: Arc::new(Mutex::new(curves.clone())),
            shown: curves,
            subscription: None,
            next_draw: now + GRAPH_EVERY,
        }
    }

    /// Once a frame: subscribed to the application's link, and every 100 ms the curves taken
    /// as the timer's `AxisChange` and `Invalidate` draw them.
    fn tick(&mut self, telemetry: &Telemetry, now: Instant) {
        if telemetry.has_link()
            && !self
                .subscription
                .as_ref()
                .is_some_and(|subscription| telemetry.carries(subscription))
        {
            let data = Arc::clone(&self.data);
            let target = self.target.clone();
            self.subscription = telemetry.on_packet(move |packet| {
                if let Ok(mut curves) = data.lock() {
                    curves.add(&target, packet);
                }
            });
        }
        if now >= self.next_draw {
            self.next_draw = now + GRAPH_EVERY;
            if let Ok(curves) = self.data.lock() {
                self.shown = curves.clone();
            }
        }
    }

    /// Every point's `x` and `y` ranges over the curves shown; `0` to `1` for none, as
    /// `Scale.SetRange` sets an axis with no data.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/Scale.cs:2684-2710`
    #[must_use]
    pub fn ranges(&self) -> ((f64, f64), (f64, f64)) {
        ranges_of(&self.shown)
    }
}

/// Every point's `x` and `y` ranges over `curves`; `0` to `1` for none.
#[must_use]
pub(crate) fn ranges_of(curves: &Curves) -> ((f64, f64), (f64, f64)) {
    let mut x = (f64::MAX, f64::MIN);
    let mut y = (f64::MAX, f64::MIN);
    for (px_, py_) in curves.list.iter().flat_map(|curve| curve.points.iter()) {
        x = (x.0.min(*px_), x.1.max(*px_));
        y = (y.0.min(*py_), y.1.max(*py_));
    }
    let fix = |(low, high): (f64, f64)| {
        if low >= f64::MAX || high <= f64::MIN {
            (0.0, 1.0)
        } else {
            (low, high)
        }
    };
    (fix(x), fix(y))
}

// ---------------------------------------------------------------------------------------------
// ZedGraph's date axis.
// ---------------------------------------------------------------------------------------------

/// `DateUnit`: what a date axis steps by.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DateUnit {
    /// Years.
    Year,
    /// Months.
    Month,
    /// Days.
    Day,
    /// Hours.
    Hour,
    /// Minutes.
    Minute,
    /// Seconds.
    Second,
    /// Milliseconds.
    Millisecond,
}

impl DateUnit {
    /// `GetUnitMultiple`: the unit in days, a month thirty and a year 365.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/DateScale.cs:888-907`
    #[must_use]
    pub const fn days(self) -> f64 {
        match self {
            Self::Year => 365.0,
            Self::Month => 30.0,
            Self::Day => 1.0,
            Self::Hour => 1.0 / 24.0,
            Self::Minute => 1.0 / 1_440.0,
            Self::Second => 1.0 / 86_400.0,
            Self::Millisecond => 1.0 / 86_400_000.0,
        }
    }
}

/// `Scale.Default.TargetXSteps`.
const TARGET_X_STEPS: f64 = 7.0;
/// `Scale.Default.MinGrace` and `MaxGrace`.
const GRACE: f64 = 0.1;

/// The calendar parts of an `XDate`, to the millisecond: `XLDateToCalendarDate`.
fn calendar(x: f64) -> Option<NaiveDateTime> {
    let ms = (x * MS_PER_DAY).round();
    if !ms.is_finite() || ms.abs() > 1.0e15 {
        return None;
    }
    #[allow(clippy::cast_possible_truncation)] // bounded above
    let ms = ms as i64;
    NaiveDate::from_ymd_opt(1899, 12, 30)?
        .and_hms_opt(0, 0, 0)?
        .checked_add_signed(chrono::TimeDelta::try_milliseconds(ms)?)
}

/// `CalendarDateToXLDate` of parts that may be out of range, `NormalizeCalendarDate` carrying
/// each into the next: a month of 13 is January of the next year, a day of 32 the next month.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/XDate.cs:465-520, 676-712`
#[allow(clippy::cast_precision_loss)] // milliseconds since 1899
fn from_calendar(
    year: i32,
    month: i32,
    day: i64,
    hour: i64,
    minute: i64,
    second: i64,
    ms: i64,
) -> f64 {
    let months = i64::from(year) * 12 + i64::from(month) - 1;
    let (Ok(year), Ok(month)) = (
        i32::try_from(months.div_euclid(12)),
        u32::try_from(months.rem_euclid(12) + 1),
    ) else {
        return 0.0;
    };
    let Some(first) = NaiveDate::from_ymd_opt(year, month, 1).and_then(|d| d.and_hms_opt(0, 0, 0))
    else {
        return 0.0;
    };
    let Some(epoch) = NaiveDate::from_ymd_opt(1899, 12, 30).and_then(|d| d.and_hms_opt(0, 0, 0))
    else {
        return 0.0;
    };
    let offset = (((day - 1) * 24 + hour) * 60 + minute) * 60_000 + second * 1000 + ms;
    let since = (first - epoch).num_milliseconds() + offset;
    since as f64 / MS_PER_DAY
}

/// A date's parts as `i64`s: year, month, day, hour, minute, second, millisecond.
fn parts(date: NaiveDateTime) -> (i32, i32, i64, i64, i64, i64, i64) {
    (
        date.year(),
        i32::try_from(date.month()).unwrap_or(1),
        i64::from(date.day()),
        i64::from(date.hour()),
        i64::from(date.minute()),
        i64::from(date.second()),
        i64::from(date.nanosecond() / 1_000_000),
    )
}

/// `CalcDateStepSize`: the major step and its unit for a range of days, the unit decided by the
/// range and the step rounded to one the unit takes. The C#'s YearYear and YearMonth branches
/// step alike in whole years, and its DayDay and DayHour alike in whole days - they differ in
/// their minor steps and default formats, which a graph formatted `HH:mm:ss.fff` without minor
/// labels does not use - so each pair is one branch here.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/DateScale.cs:508-758; Scale.cs:310-400`
#[must_use]
pub fn date_step(range: f64, target_steps: f64) -> (f64, DateUnit) {
    let temp = range / target_steps;
    let pick = |step: f64, choices: [f64; 3], top: f64| {
        if step > choices[2] {
            top
        } else if step > choices[1] {
            choices[2]
        } else if step > choices[0] {
            choices[1]
        } else {
            choices[0]
        }
    };
    // `RangeYearYear` (1825) and `RangeYearMonth`.
    if range > 730.0 {
        ((temp / 365.0).ceil(), DateUnit::Year)
    } else if range > 300.0 {
        ((temp / 30.0).ceil(), DateUnit::Month)
    } else if range > 3.0 {
        // `RangeDayDay` (10) and `RangeDayHour`.
        (temp.ceil(), DateUnit::Day)
    } else if range > 0.4167 {
        let hours = (temp * 24.0).ceil();
        let hours = if hours > 12.0 {
            24.0
        } else if hours > 6.0 {
            12.0
        } else if hours > 2.0 {
            6.0
        } else if hours > 1.0 {
            2.0
        } else {
            1.0
        };
        (hours, DateUnit::Hour)
    } else if range > 0.125 {
        ((temp * 24.0).ceil(), DateUnit::Hour)
    } else if range > 6.94e-3 {
        (
            pick((temp * 1_440.0).ceil(), [1.0, 5.0, 15.0], 30.0),
            DateUnit::Minute,
        )
    } else if range > 2.083e-3 {
        ((temp * 1_440.0).ceil(), DateUnit::Minute)
    } else if range > 3.472e-5 {
        (
            pick((temp * 86_400.0).ceil(), [1.0, 5.0, 15.0], 30.0),
            DateUnit::Second,
        )
    } else {
        (
            calc_step_size(range * MS_PER_DAY, TARGET_X_STEPS),
            DateUnit::Millisecond,
        )
    }
}

/// `Scale.CalcStepSize`: the range over the target steps, rounded to 1, 2 or 5 times a power of
/// ten.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/Scale.cs:2577-2598`
fn calc_step_size(range: f64, target_steps: f64) -> f64 {
    let temp = range / target_steps;
    let mag = temp.log10().floor();
    let power = 10f64.powf(mag);
    let msd = (temp / power + 0.5).trunc();
    let msd = if msd > 5.0 {
        10.0
    } else if msd > 2.0 {
        5.0
    } else if msd > 1.0 {
        2.0
    } else {
        msd
    };
    msd * power
}

/// A date axis after `AxisChange`: its ends, its major step and the step's unit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DateScale {
    /// `_min`, as an `XDate`.
    pub min: f64,
    /// `_max`.
    pub max: f64,
    /// `_majorStep`, in `unit`s.
    pub major_step: f64,
    /// `_majorUnit`.
    pub unit: DateUnit,
}

impl DateScale {
    /// `Scale.PickScale` then `DateScale.PickScale` for data from `range_min` to `range_max`:
    /// the grace either side, a range of nothing widened, the step, and the ends taken out to
    /// whole units.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/Scale.cs:2394-2449; DateScale.cs:431-475`
    #[must_use]
    pub fn pick(range_min: f64, range_max: f64) -> Self {
        let legitimate = |value: f64| {
            if value.is_infinite() || value.is_nan() || value == f64::MAX {
                0.0
            } else {
                value
            }
        };
        let (min_val, max_val) = (legitimate(range_min), legitimate(range_max));
        let range = max_val - min_val;
        let mut min = min_val;
        if min < 0.0 || min_val - GRACE * range >= 0.0 {
            min = min_val - GRACE * range;
        }
        let mut max = max_val;
        if max > 0.0 || max_val + GRACE * range <= 0.0 {
            max = max_val + GRACE * range;
        }
        if max == min {
            if max.abs() > 1e-100 {
                max *= if min < 0.0 { 0.95 } else { 1.05 };
                min *= if min < 0.0 { 1.05 } else { 0.95 };
            } else {
                max = 1.0;
                min = -1.0;
            }
        }
        if max <= min {
            max = min + 1.0;
        }
        // DateScale.PickScale
        if max - min < 1.0e-20 {
            max += 0.2 * if max == 0.0 { 1.0 } else { max.abs() };
            min -= 0.2 * if min == 0.0 { 1.0 } else { min.abs() };
        }
        let (major_step, unit) = date_step(max - min, TARGET_X_STEPS);
        Self {
            min: even_step(min, false, unit),
            max: even_step(max, true, unit),
            major_step,
            unit,
        }
    }

    /// `CalcBaseTic`: the first whole unit at or after the minimum.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/DateScale.cs:262-330`
    fn base_tic(&self) -> f64 {
        let Some(date) = calendar(self.min) else {
            return self.min;
        };
        let (year, month, day, hour, minute, second, ms) = parts(date);
        let whole = match self.unit {
            DateUnit::Year => (year, 1, 1, 0, 0, 0, 0),
            DateUnit::Month => (year, month, 1, 0, 0, 0, 0),
            DateUnit::Day => (year, month, day, 0, 0, 0, 0),
            DateUnit::Hour => (year, month, day, hour, 0, 0, 0),
            DateUnit::Minute => (year, month, day, hour, minute, 0, 0),
            DateUnit::Second => (year, month, day, hour, minute, second, 0),
            DateUnit::Millisecond => (year, month, day, hour, minute, second, ms),
        };
        let (y, mo, d, h, mi, s, m) = whole;
        let tic = from_calendar(y, mo, d, h, mi, s, m);
        if tic >= self.min {
            return tic;
        }
        match self.unit {
            DateUnit::Year => from_calendar(y + 1, mo, d, h, mi, s, m),
            DateUnit::Month => from_calendar(y, mo + 1, d, h, mi, s, m),
            DateUnit::Day => from_calendar(y, mo, d + 1, h, mi, s, m),
            DateUnit::Hour => from_calendar(y, mo, d, h + 1, mi, s, m),
            DateUnit::Minute => from_calendar(y, mo, d, h, mi + 1, s, m),
            DateUnit::Second => from_calendar(y, mo, d, h, mi, s + 1, m),
            DateUnit::Millisecond => from_calendar(y, mo, d, h, mi, s, m + 1),
        }
    }

    /// `CalcMajorTicValue`: `tic` major steps after `base`, a year or a month on the calendar.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/DateScale.cs:139-170`
    #[allow(clippy::cast_possible_truncation)] // whole steps of whole months
    fn major(&self, base: f64, tic: f64) -> f64 {
        let steps = tic * self.major_step;
        match self.unit {
            DateUnit::Year | DateUnit::Month => {
                let Some(date) = calendar(base) else {
                    return base;
                };
                let (year, month, day, hour, minute, second, ms) = parts(date);
                let months = if self.unit == DateUnit::Year {
                    steps * 12.0
                } else {
                    steps
                };
                from_calendar(
                    year,
                    month + months.round() as i32,
                    day,
                    hour,
                    minute,
                    second,
                    ms,
                )
            }
            unit => base + steps * unit.days(),
        }
    }

    /// `CalcNumTics`: the major steps between the ends, from 1 to 1000.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/DateScale.cs:333-390`
    #[allow(clippy::cast_possible_truncation)] // clamped below
    fn tic_count(&self) -> i64 {
        let years = |date: Option<NaiveDateTime>| date.map_or(0, |d| i64::from(d.year()));
        let months = |date: Option<NaiveDateTime>| date.map_or(0, |d| i64::from(d.month()));
        let (low, high) = (calendar(self.min), calendar(self.max));
        #[allow(clippy::cast_precision_loss)]
        let count = match self.unit {
            DateUnit::Year => ((years(high) - years(low)) as f64 / self.major_step + 1.001) as i64,
            DateUnit::Month => {
                let span =
                    (months(high) - months(low)) as f64 + 12.0 * (years(high) - years(low)) as f64;
                (span / self.major_step + 1.001) as i64
            }
            unit => ((self.max - self.min) / (self.major_step * unit.days()) + 1.001) as i64,
        };
        count.clamp(1, 1000)
    }

    /// The major tics `DrawLabels` draws: from the base tic to the maximum, allowing a
    /// thousandth of the range past it.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/Scale.cs:1941-1957, 2006-2040`
    #[must_use]
    pub fn tics(&self) -> Vec<f64> {
        if self.major_step.is_nan() || self.major_step <= 0.0 || self.min >= self.max {
            return Vec::new();
        }
        let base = self.base_tic();
        let tolerance = (self.max - self.min) * 0.001;
        let mut tics = Vec::new();
        for i in 0..self.tic_count() {
            #[allow(clippy::cast_precision_loss)] // at most 1000
            let value = self.major(base, i as f64);
            if value < self.min {
                continue;
            }
            if value > self.max + tolerance {
                break;
            }
            tics.push(value);
        }
        tics
    }

    /// Where `x` falls between the ends, 0 at the left.
    #[must_use]
    pub fn fraction(&self, x: f64) -> f64 {
        (x - self.min) / (self.max - self.min)
    }

    /// A tic's label: `XDate.ToString(dVal, "HH:mm:ss.fff")`.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/DateScale.cs:847-853`
    #[must_use]
    pub fn label(x: f64) -> String {
        calendar(x).map_or_else(
            || "Date Error".to_owned(),
            |date| date.format(X_FORMAT).to_string(),
        )
    }
}

/// `CalcEvenStepDate`: back to the start of the unit the date is in, or on to the start of the
/// next one unless it is on one already.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/DateScale.cs:765-845`
fn even_step(date: f64, up: bool, unit: DateUnit) -> f64 {
    let Some(parts_of) = calendar(date) else {
        return date;
    };
    let (year, month, day, hour, minute, second, ms) = parts(parts_of);
    let step = i64::from(up);
    let step32 = i32::from(up);
    match unit {
        DateUnit::Year => {
            if up && month == 1 && day == 1 && hour == 0 && minute == 0 && second == 0 {
                date
            } else {
                from_calendar(year + step32, 1, 1, 0, 0, 0, 0)
            }
        }
        DateUnit::Month => {
            if up && day == 1 && hour == 0 && minute == 0 && second == 0 {
                date
            } else {
                from_calendar(year, month + step32, 1, 0, 0, 0, 0)
            }
        }
        DateUnit::Day => {
            if up && hour == 0 && minute == 0 && second == 0 {
                date
            } else {
                from_calendar(year, month, day + step, 0, 0, 0, 0)
            }
        }
        DateUnit::Hour => {
            if up && minute == 0 && second == 0 {
                date
            } else {
                from_calendar(year, month, day, hour + step, 0, 0, 0)
            }
        }
        DateUnit::Minute => {
            if up && second == 0 {
                date
            } else {
                from_calendar(year, month, day, hour, minute + step, 0, 0)
            }
        }
        DateUnit::Second => from_calendar(year, month, day, hour, minute, second + step, 0),
        DateUnit::Millisecond => from_calendar(year, month, day, hour, minute, second, ms + step),
    }
}

// ---------------------------------------------------------------------------------------------
// The window.
// ---------------------------------------------------------------------------------------------

/// The form, from its constructor to its close.
#[derive(Debug)]
pub struct Inspector {
    /// `mavi`, which the link thread's handler adds to.
    mavi: Arc<Mutex<PacketInspector<Packet>>>,
    /// `chk_gcstraffic.Checked`, which the handler reads: whether sent packets are added.
    gcs_traffic: Arc<AtomicBool>,
    /// The handler's subscription to the link.
    subscription: Option<PacketSubscription>,
    /// `treeView1.Nodes`.
    pub tree: Vec<Node>,
    /// The nodes expanded, by their `Name`s.
    expanded: BTreeSet<Vec<String>>,
    /// `SelectedNode`, by its `Name`s.
    selected: Option<Vec<String>>,
    /// `selectedmsgid`: the selected node's `FullPath` when it was selected.
    pub selected_path: Option<String>,
    /// `but_graphit.Enabled`.
    pub graph_it: bool,
    /// `history`.
    pub history: i32,
    /// When `timer1` next ticks.
    next_update: Instant,
    /// How many times it has.
    pub updates: usize,
    /// Graph It's question, while it is up.
    pub asking: Option<InputBox>,
    /// The question closed with OK, for the application to keep its answer.
    answered: Option<InputBox>,
    /// The graphs it opened.
    pub graphs: Vec<Graph>,
}

impl Inspector {
    /// `new MAVLinkInspector(comPort)`: an empty tree, Graph It disabled, "Show GCS Traffic"
    /// unticked, and the timer started.
    /// `// C#: Controls/MAVLinkInspector.cs:28-50`
    #[must_use]
    pub fn new(now: Instant) -> Self {
        Self {
            mavi: Arc::new(Mutex::new(PacketInspector::default())),
            gcs_traffic: Arc::new(AtomicBool::new(false)),
            subscription: None,
            tree: Vec::new(),
            expanded: BTreeSet::new(),
            selected: None,
            selected_path: None,
            graph_it: false,
            history: HISTORY,
            next_update: now + UPDATE_EVERY,
            updates: 0,
            asking: None,
            answered: None,
            graphs: Vec::new(),
        }
    }

    /// `MavOnOnPacketReceived`, subscribed to the link's packets: each read one added, and
    /// each written one while "Show GCS Traffic" is ticked.
    /// `// C#: Controls/MAVLinkInspector.cs:34, 52-55, 448-457`
    fn handler(&self) -> impl FnMut(&Packet) + Send + 'static {
        let mavi = Arc::clone(&self.mavi);
        let gcs_traffic = Arc::clone(&self.gcs_traffic);
        move |packet: &Packet| {
            if packet.sent && !gcs_traffic.load(Atomic::Relaxed) {
                return;
            }
            if let Ok(mut mavi) = mavi.lock() {
                mavi.add(
                    packet.sysid,
                    packet.compid,
                    packet.msgid,
                    *packet,
                    packet.length,
                    packet.at,
                );
            }
        }
    }

    /// A packet added as the handler adds it, for a test.
    #[cfg(test)]
    fn hear(&self, packet: &Packet) {
        let mut handler = self.handler();
        handler(packet);
    }

    /// Once a frame: subscribed to the application's link, the graphs' timers, and every 333 ms
    /// the tree brought up to date.
    pub fn tick(&mut self, telemetry: &Telemetry, now: Instant) {
        if telemetry.has_link()
            && !self
                .subscription
                .as_ref()
                .is_some_and(|subscription| telemetry.carries(subscription))
        {
            self.subscription = telemetry.on_packet(self.handler());
        }
        for graph in &mut self.graphs {
            graph.tick(telemetry, now);
        }
        self.timer(now);
    }

    /// `timer1.Tick`: `Update()` when the interval has run.
    fn timer(&mut self, now: Instant) {
        if now < self.next_update {
            return;
        }
        self.next_update = now + UPDATE_EVERY;
        self.updates += 1;
        if let Ok(mavi) = self.mavi.lock() {
            update(&mut self.tree, &mavi, now);
        }
    }

    /// "Show GCS Traffic" clicked: `chk_gcstraffic_CheckedChanged`, the handler subscribed to
    /// the packets written too, or no longer.
    /// `// C#: Controls/MAVLinkInspector.cs:448-457`
    pub fn toggle_gcs_traffic(&mut self) {
        self.gcs_traffic.fetch_xor(true, Atomic::Relaxed);
    }

    /// Whether "Show GCS Traffic" is ticked.
    #[must_use]
    pub fn gcs_traffic(&self) -> bool {
        self.gcs_traffic.load(Atomic::Relaxed)
    }

    /// A node's plus or minus box: its children shown or hidden.
    pub fn toggle(&mut self, key: &[String]) {
        if !self.expanded.remove(key) && node_at(&self.tree, key).is_some() {
            self.expanded.insert(key.to_vec());
        }
    }

    /// Whether a node's children show.
    #[must_use]
    pub fn is_expanded(&self, key: &[String]) -> bool {
        self.expanded.contains(key)
    }

    /// The rows the tree shows.
    #[must_use]
    pub fn rows(&self) -> Vec<Row> {
        let mut out = Vec::new();
        rows_of(&self.tree, &[], &self.expanded, &mut out);
        out
    }

    /// A node clicked: `SelectedNode`, then `treeView1_AfterSelect` - a vehicle's node changes
    /// nothing more; any other enables Graph It, its parent's `Name` being a number, and keeps
    /// its `FullPath`.
    /// `// C#: Controls/MAVLinkInspector.cs:322-337`
    pub fn select(&mut self, key: &[String]) {
        if node_at(&self.tree, key).is_none() {
            return;
        }
        self.selected = Some(key.to_vec());
        let Some((_, parent)) = key.split_last() else {
            return;
        };
        if parent.is_empty() {
            return;
        }
        let parent_name = parent.last().map_or("", String::as_str);
        if parse_int32(parent_name).is_ok() {
            self.selected_path = full_path(&self.tree, key);
            self.graph_it = true;
        } else {
            self.graph_it = false;
        }
    }

    /// The selected node, by its `Name`s.
    #[must_use]
    pub fn selected(&self) -> Option<&[String]> {
        self.selected.as_deref()
    }

    /// Graph It clicked: "Points of history?", holding the history.
    /// `// C#: Controls/MAVLinkInspector.cs:341-343`
    pub fn press_graph_it(&mut self) {
        if self.graph_it && self.asking.is_none() {
            self.asking = Some(InputBox::new(
                POINTS_TITLE,
                POINTS_PROMPT,
                &self.history.to_string(),
            ));
        }
    }

    /// A key in the question's box: typed, or Enter as OK, or Escape as Cancel.
    pub fn prompt_key(&mut self, event: &KeyDownEvent, now: Instant) -> (bool, Result<(), String>) {
        let Some(input) = self.asking.as_mut() else {
            return (false, Ok(()));
        };
        match input.field.key(event) {
            KeyOutcome::Changed => (true, Ok(())),
            KeyOutcome::Submitted => (true, self.prompt_done(true, now)),
            KeyOutcome::Cancelled => (true, self.prompt_done(false, now)),
            KeyOutcome::Ignored => (false, Ok(())),
        }
    }

    /// The question closed, and the rest of `but_graphit_Click`: OK's answer read as
    /// `int.Parse` reads it - Cancel keeps the history, `InputBox.Show`'s result is not looked
    /// at - then the path read, and for a field's node a graph opened and Graph It disabled.
    /// What the C# throws is the error.
    /// `// C#: Controls/MAVLinkInspector.cs:341-446; ExtLibs/Controls/InputBox.cs:21-27`
    pub fn prompt_done(&mut self, ok: bool, now: Instant) -> Result<(), String> {
        let Some(input) = self.asking.take() else {
            return Ok(());
        };
        if ok {
            let answer = input.field.value().to_owned();
            // `InputBox` keeps the text before the `ref int` overload parses it.
            self.answered = Some(input);
            self.history = parse_int32(&answer).map_err(str::to_owned)?;
        }
        let Some(path) = self.selected_path.clone() else {
            return Ok(());
        };
        let Some(target) = target(&path).map_err(str::to_owned)? else {
            return Ok(());
        };
        let history = usize::try_from(self.history).map_err(|_| NEGATIVE_HISTORY.to_owned())?;
        // `MAVLINK_MESSAGE_INFOS.First(a => a.msgid == msgid).type`: the message's type, read
        // here from an empty one of it; an id the dialect lacks throws, caught, "Y Axis" kept.
        let y_title = u32::try_from(target.msgid)
            .ok()
            .and_then(|msgid| MavMessage::decode(msgid, &[]))
            .map_or_else(
                || Y_TITLE.to_owned(),
                |message| units_title(&message, &target.field),
            );
        self.graphs.push(Graph::new(target, y_title, history, now));
        self.graph_it = false;
        Ok(())
    }

    /// The question's box closed with OK, for its answer to be kept.
    pub fn take_answered(&mut self) -> Option<InputBox> {
        self.answered.take()
    }

    /// A graph's close box: `form.Closing`, its subscription ended.
    /// `// C#: Controls/MAVLinkInspector.cs:441`
    pub fn close_graph(&mut self, index: usize) {
        if index < self.graphs.len() {
            self.graphs.remove(index);
        }
    }

    /// Types into the question's box, as a test does.
    #[cfg(test)]
    fn type_answer(&mut self, text: &str) {
        if let Some(input) = self.asking.as_mut() {
            input.field.set(text);
        }
    }
}

/// The window and how often it has been opened, held with the Advanced page.
#[derive(Debug, Default)]
pub struct InspectorWindow {
    /// The form, while it is open.
    pub window: Option<Inspector>,
    /// How many times it has been opened.
    pub opened: usize,
}

impl InspectorWindow {
    /// `new MAVLinkInspector(MainV2.comPort).Show()`: a fresh form.
    /// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:27-30`
    pub fn show(&mut self, now: Instant) {
        self.opened += 1;
        self.window = Some(Inspector::new(now));
    }

    /// The form's close box: `MAVLinkInspector_FormClosing`, the subscriptions and the timer
    /// ended with it, and its graphs with it.
    /// `// C#: Controls/MAVLinkInspector.cs:304-310`
    pub fn close(&mut self) {
        self.window = None;
    }

    /// Once a frame.
    pub fn tick(&mut self, telemetry: &Telemetry, now: Instant) {
        if let Some(window) = self.window.as_mut() {
            window.tick(telemetry, now);
        }
    }
}

/// How many rows' texts the facts carry.
const ROW_FACTS: usize = 60;

/// Facts a UI test asserts on.
pub fn record_facts(holder: &InspectorWindow) {
    use crate::facts::record;
    record("config.inspector.window", holder.window.is_some());
    record("config.inspector.opened", holder.opened);
    let Some(window) = holder.window.as_ref() else {
        return;
    };
    record("config.inspector.updates", window.updates);
    record("config.inspector.gcstraffic", window.gcs_traffic());
    record("config.inspector.graphit", window.graph_it);
    record("config.inspector.history", window.history);
    record(
        "config.inspector.prompt",
        window.asking.as_ref().map_or("none", |input| input.prompt),
    );
    record(
        "config.inspector.selected",
        window.selected_path.as_deref().map_or("none", shown),
    );
    record(
        "config.inspector.vehicles",
        window
            .tree
            .iter()
            .map(|node| node.text.as_str())
            .collect::<Vec<_>>()
            .join(","),
    );
    for vehicle in &window.tree {
        record(
            format!("config.inspector.components.{}", vehicle.name),
            vehicle
                .nodes
                .iter()
                .map(|node| node.text.as_str())
                .collect::<Vec<_>>()
                .join(","),
        );
        for component in &vehicle.nodes {
            record(
                format!(
                    "config.inspector.messages.{}.{}",
                    vehicle.name, component.name
                ),
                component.nodes.len(),
            );
            for message in &component.nodes {
                let key = [
                    vehicle.name.clone(),
                    component.name.clone(),
                    message.name.clone(),
                ];
                record(
                    format!("config.inspector.message.{}", key.join(".")),
                    &message.text,
                );
                if !window.is_expanded(&key) {
                    continue;
                }
                for field in &message.nodes {
                    let Some((value, type_name)) = field.tag.as_ref() else {
                        continue;
                    };
                    let at = format!("{}.{}", key.join("."), field.name);
                    record(format!("config.inspector.value.{at}"), shown(value));
                    record(format!("config.inspector.type.{at}"), type_name);
                }
            }
        }
    }
    let rows = window.rows();
    record("config.inspector.rows", rows.len());
    for (index, row) in rows.iter().take(ROW_FACTS).enumerate() {
        record(
            format!("config.inspector.row.{index}"),
            shown(&row.text).trim_end(),
        );
    }
    record("config.inspector.graphs", window.graphs.len());
    for (index, graph) in window.graphs.iter().enumerate() {
        record(
            format!("config.inspector.graph.{index}.curves"),
            graph
                .shown
                .list
                .iter()
                .map(|curve| curve.label.as_str())
                .collect::<Vec<_>>()
                .join(","),
        );
        record(
            format!("config.inspector.graph.{index}.points"),
            graph
                .shown
                .list
                .first()
                .map_or(0, |curve| curve.points.len()),
        );
        record(
            format!("config.inspector.graph.{index}.yaxis"),
            &graph.y_title,
        );
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// How the drawing reaches the form.
fn access(this: &mut MissionPlanner) -> Option<&mut Inspector> {
    this.extra.inspector.window.as_mut()
}

/// A colour as it shows on the application's dark ground: `Color.Black` is drawn in the text
/// colour, as the FFT window draws it.
const fn on_dark(colour: u32) -> u32 {
    if colour == 0 { theme::TEXT } else { colour }
}

/// A field's row: the name, the value and the type each where Courier New's columns put them -
/// the value right-aligned in its twenty, or where its left padding ends when a NUL cuts it,
/// and the type after it unless the NUL has cut it off.
pub(crate) fn field_cells(name: &str, value: &str, type_name: &str) -> gpui::Div {
    #[allow(clippy::cast_precision_loss)] // a row's characters
    let at_chars = |chars: usize| chars as f32 * CHAR_WIDTH;
    let length = value.chars().count();
    let width = length.max(20);
    let cell = |x: f32| div().absolute().top_0().left(px(x)).whitespace_nowrap();
    let mut row = div()
        .relative()
        .h(px(ROW_HEIGHT))
        .w(px(at_chars(33 + width + 1 + 20)))
        .child(cell(0.0).child(name.to_owned()));
    if value.contains('\0') {
        row = row.child(cell(at_chars(33 + width - length)).child(shown(value).to_owned()));
    } else {
        row = row
            .child(
                cell(at_chars(33))
                    .w(px(at_chars(width)))
                    .flex()
                    .justify_end()
                    .child(value.to_owned()),
            )
            .child(cell(at_chars(33 + width + 1)).child(type_name.to_owned()));
    }
    row
}

/// One row of the tree: set in by its depth, its plus or minus box, its text, highlighted when
/// selected.
fn row_element(row: &Row, selected: bool, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let toggle_id = row_id("toggle", &row.key);
    let node_id = row_id("node", &row.key);
    let toggle_key = row.key.clone();
    let select_key = row.key.clone();
    tree_row(
        row,
        selected,
        toggle_id,
        node_id,
        cx.listener(move |this, _event, _window, cx| {
            if let Some(window) = access(this) {
                window.toggle(&toggle_key);
                cx.notify();
            }
        }),
        cx.listener(move |this, event: &gpui::ClickEvent, _window, cx| {
            if let Some(window) = access(this) {
                window.select(&select_key);
                // A double click opens or closes the node, as the tree does.
                if event.click_count() == 2 {
                    window.toggle(&select_key);
                }
                cx.notify();
            }
        }),
    )
}

/// A tree row as both inspectors draw one: the plus or minus box (`on_toggle`), the text -
/// a field's in its cells - and the node selected on a click (`on_select`).
pub(crate) fn tree_row(
    row: &Row,
    selected: bool,
    toggle_id: String,
    node_id: String,
    on_toggle: impl Fn(&gpui::ClickEvent, &mut Window, &mut gpui::App) + 'static,
    on_select: impl Fn(&gpui::ClickEvent, &mut Window, &mut gpui::App) + 'static,
) -> AnyElement {
    #[allow(clippy::cast_precision_loss)] // four levels
    let indent = INDENT * row.depth as f32;
    let toggle = if row.parent {
        crate::probe::measured(toggle_id.clone(), div())
            .id(SharedString::from(toggle_id))
            .size(px(9.0))
            .flex()
            .items_center()
            .justify_center()
            .border_1()
            .border_color(rgb(theme::DIM))
            .text_size(px(8.0))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .child(if row.expanded { "-" } else { "+" })
            .on_click(on_toggle)
            .into_any_element()
    } else {
        div().size(px(9.0)).into_any_element()
    };
    let text: AnyElement = match row.tag.as_ref() {
        Some((value, type_name)) => {
            let name = row.key.last().map_or("", String::as_str);
            field_cells(name, value, type_name).into_any_element()
        }
        None => div()
            .whitespace_nowrap()
            .child(shown(&row.text).to_owned())
            .into_any_element(),
    };
    div()
        .flex()
        .items_center()
        .h(px(ROW_HEIGHT))
        .pl(px(indent + 5.0))
        .gap(px(5.0))
        .child(toggle)
        .child(
            crate::probe::measured(node_id.clone(), div())
                .id(SharedString::from(node_id))
                .px_1()
                .when(selected, |this| this.bg(rgb(theme::SELECTION)))
                .cursor_pointer()
                .child(text)
                .on_click(on_select),
        )
        .into_any_element()
}

/// The form over the window: its caption with a close box, Graph It, "Show GCS Traffic" and the
/// tree in its group box at their places.
/// `// C#: Controls/MAVLinkInspector.cs:176-272`
fn form(
    inspector: &Inspector,
    prompt: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let size = window.viewport_size();
    let (gx, gy, gw, gh) = GROUP_AT;
    let (tx, ty, tw, th) = TREE_AT;
    let selected = inspector.selected();
    let mut tree = crate::probe::measured("inspector-tree", div())
        .id("inspector-tree")
        .absolute()
        .left(px(tx))
        .top(px(ty))
        .w(px(tw))
        .h(px(th))
        .overflow_scroll()
        .bg(rgb(theme::BG))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .text_color(rgb(theme::TEXT));
    for row in inspector.rows() {
        let is_selected = selected.is_some_and(|key| key == row.key.as_slice());
        tree = tree.child(row_element(&row, is_selected, cx));
    }
    let mut gcs = crate::config::servo_output::Check::default();
    gcs.enabled = true;
    gcs.state = if inspector.gcs_traffic() {
        crate::config::failsafe::CheckState::Checked
    } else {
        crate::config::failsafe::CheckState::Unchecked
    };
    let prompt = prompt.clone();
    let client = div()
        .relative()
        .w(px(CLIENT.0))
        .h(px(CLIENT.1))
        .child(button(
            "inspector-graphit",
            GRAPH_IT,
            GRAPH_IT_AT,
            inspector.graph_it,
            move |this, window, cx| {
                if let Some(inspector) = access(this) {
                    inspector.press_graph_it();
                    prompt.focus(window, cx);
                }
            },
            cx,
        ))
        .child(crate::config::servo_output::check_box(
            "inspector-gcstraffic".to_owned(),
            &gcs,
            GCS_TRAFFIC,
            GCS_TRAFFIC_AT,
            |this| {
                if let Some(inspector) = access(this) {
                    inspector.toggle_gcs_traffic();
                }
            },
            cx,
        ))
        .child(
            at(gx, gy, gw, gh)
                .border_1()
                .border_color(rgb(theme::BORDER))
                .rounded_sm()
                .child(tree),
        );
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
            "inspector-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.extra.inspector.close();
                cx.notify();
            }),
        ));
    let form = crate::probe::measured("inspector", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    let over = div()
        .id("inspector-backdrop")
        .w(size.width)
        .h(size.height)
        .flex()
        .items_center()
        .justify_center()
        .occlude()
        .child(form);
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(over),
    )
    .with_priority(1)
    .into_any_element()
}

/// The pane's margins round the plot: the Y axis's labels on the left, the legend over it,
/// the X axis's labels and title under it.
const PLOT_MARGINS: (f32, f32, f32, f32) = (64.0, 48.0, 20.0, 46.0);

/// A graph's form: a caption with a close box, and the pane - the legend, the axes' titles
/// and labels, and the curves.
/// `// C#: Controls/MAVLinkInspector.cs:344-445`
fn graph_form(
    graph: &Graph,
    index: usize,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    graph_pane(
        "inspector-graph",
        index,
        &graph.y_title,
        &graph.shown,
        graph.ranges(),
        move |this| {
            if let Some(inspector) = access(this) {
                inspector.close_graph(index);
            }
        },
        window,
        cx,
    )
}

/// A graph's form as both inspectors draw one, ids under `prefix`: a caption with a close box
/// (`on_close`), and the pane - the legend, the axes' titles and labels, and the curves.
#[allow(clippy::too_many_arguments)] // the pane's parts
pub(crate) fn graph_pane(
    prefix: &str,
    index: usize,
    y_title: &str,
    shown: &Curves,
    ranges: ((f64, f64), (f64, f64)),
    on_close: impl Fn(&mut MissionPlanner) + 'static,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (width, height) = GRAPH_SIZE;
    let caption_height = 26.0;
    let pane_height = height - caption_height;
    let (left, top, right, bottom) = PLOT_MARGINS;
    let plot_width = width - left - right;
    let plot_height = pane_height - top - bottom;
    let ((x_low, x_high), (y_low, y_high)) = ranges;
    let x_scale = DateScale::pick(x_low, x_high);
    let y_scale = Scale::pick(y_low, y_high, None);
    #[allow(clippy::cast_possible_truncation)]
    let across = |x: f64| x_scale.fraction(x) as f32;
    #[allow(clippy::cast_possible_truncation)]
    let up = |y: f64| y_scale.fraction(y) as f32;
    let text = |content: String| {
        div()
            .absolute()
            .text_xs()
            .whitespace_nowrap()
            .text_color(rgb(theme::TEXT))
            .child(content)
    };
    let close_id = format!("{prefix}-{index}-close");
    let caption = div()
        .flex()
        .items_center()
        .justify_end()
        .h(px(caption_height))
        .px_2()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(
            crate::probe::measured(close_id.clone(), div())
                .id(SharedString::from(close_id))
                .px_2()
                .rounded_sm()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child("X")
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    on_close(this);
                    cx.notify();
                })),
        );
    let legend = div()
        .absolute()
        .top(px(8.0))
        .left_0()
        .right_0()
        .flex()
        .flex_wrap()
        .justify_center()
        .gap_4()
        .children(shown.list.iter().map(|curve| {
            div()
                .flex()
                .items_center()
                .gap_1()
                .child(div().w(px(24.0)).h(px(2.0)).bg(rgb(on_dark(curve.colour))))
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .child(curve.label.clone()),
                )
        }));
    let mut pane = crate::probe::measured(format!("{prefix}-{index}"), div())
        .relative()
        .w(px(width))
        .h(px(pane_height))
        .bg(rgb(theme::BG))
        .child(legend)
        .child(
            text(y_scale.title(y_title))
                .left(px(4.0))
                .top(px(top - 16.0)),
        )
        .child(
            div()
                .absolute()
                .left(px(left))
                .w(px(plot_width))
                .top(px(top + plot_height + 20.0))
                .flex()
                .justify_center()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(X_TITLE),
        );
    for tic in x_scale.tics() {
        pane = pane.child(
            text(DateScale::label(tic))
                .left(px(left + across(tic) * plot_width - 40.0))
                .w(px(80.0))
                .flex()
                .justify_center()
                .top(px(top + plot_height + 3.0)),
        );
    }
    for tic in y_scale.tics() {
        pane = pane.child(
            text(y_scale.label(tic))
                .left_0()
                .w(px(left - 4.0))
                .flex()
                .justify_end()
                .top(px(top + (1.0 - up(tic)) * plot_height - 7.0)),
        );
    }
    let lines: Vec<crate::plotline::Line> = shown
        .list
        .iter()
        .map(|curve| crate::plotline::Line {
            points: curve
                .points
                .iter()
                .map(|(x, y)| (across(*x), 1.0 - up(*y)))
                .collect(),
            colour: rgb(on_dark(curve.colour)).into(),
            diamonds: false,
        })
        .collect();
    pane = pane.child(
        div()
            .absolute()
            .left(px(left))
            .top(px(top))
            .w(px(plot_width))
            .h(px(plot_height))
            .border_1()
            .border_color(rgb(theme::BORDER))
            .child(crate::plotline::element(lines)),
    );
    let form = div()
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(pane);
    // Each graph set down and to the right of the one before, from the window's corner.
    let size = window.viewport_size();
    #[allow(clippy::cast_precision_loss)] // a handful of graphs
    let offset = 40.0 + 30.0 * (index % 8) as f32;
    let x = (f32::from(size.width) - width).max(0.0).min(offset * 2.0);
    let y = (f32::from(size.height) - height).max(0.0).min(offset);
    gpui::deferred(
        gpui::anchored().position(gpui::point(px(x), px(y))).child(
            div()
                .id(SharedString::from(format!("{prefix}-{index}-form")))
                .occlude()
                .child(form),
        ),
    )
    .with_priority(1)
    .into_any_element()
}

/// The form, its graphs over it and Graph It's question over them, while the form is open.
pub fn overlay(
    holder: &InspectorWindow,
    prompt: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let inspector = holder.window.as_ref()?;
    let mut layers = div().child(form(inspector, prompt, window, cx));
    for (index, graph) in inspector.graphs.iter().enumerate() {
        layers = layers.child(graph_form(graph, index, window, cx));
    }
    if let Some(input) = inspector.asking.as_ref() {
        layers = layers.child(input_box(
            "inspector-points-box",
            input,
            prompt,
            window,
            |this, event| {
                let now = Instant::now();
                let (handled, result) = access(this).map_or((false, Ok(())), |inspector| {
                    inspector.prompt_key(event, now)
                });
                after_prompt(this, result);
                handled
            },
            |this| finish_prompt(this, true),
            |this| finish_prompt(this, false),
            cx,
        ));
    }
    Some(layers.into_any_element())
}

/// The question closed by a button.
fn finish_prompt(this: &mut MissionPlanner, ok: bool) {
    let now = Instant::now();
    let Some(inspector) = access(this) else {
        return;
    };
    let result = inspector.prompt_done(ok, now);
    after_prompt(this, result);
}

/// After the question: its OK's answer kept as `InputBox` keeps it, and what the C# throws on
/// the status line.
/// `// C#: ExtLibs/Controls/InputBox.cs:178-184`
fn after_prompt(this: &mut MissionPlanner, result: Result<(), String>) {
    if let Some(input) = access(this).and_then(Inspector::take_answered) {
        input.remember(&mut this.persisted);
    }
    if let Err(error) = result {
        this.file_status = Some(error);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config_coverage::source::csharp;
    use crate::telemetry::scripted::{self, Vehicle, until};
    use mp_link::ProtocolTimeouts;

    /// `new DateTime(1970, 1, 1).Ticks`.
    const UNIX_EPOCH_TICKS: i64 = 621_355_968_000_000_000;

    /// A message of `id` as the dialect decodes an empty payload, then set by `edit`.
    fn message(id: u32, edit: impl FnOnce(&mut MavMessage)) -> MavMessage {
        let mut message = MavMessage::decode(id, &[]).expect("a message the dialect has");
        edit(&mut message);
        message
    }

    /// A packet of `message` from `sysid` and `compid`, its length the frame's, at `at`.
    fn packet(sysid: u8, compid: u8, message: MavMessage, at: Instant) -> Packet {
        Packet {
            sysid,
            compid,
            msgid: message.id(),
            message,
            length: mp_link::testing::frame(0, &message).len(),
            rxtime: DateTime::from_ticks(UNIX_EPOCH_TICKS),
            at,
            sent: false,
        }
    }

    fn heartbeat() -> MavMessage {
        message(0, |m| {
            if let MavMessage::Heartbeat(h) = m {
                h.r#type = 2;
                h.autopilot = 3;
                h.base_mode = 81;
                h.system_status = 3;
                h.mavlink_version = 3;
            }
        })
    }

    fn attitude(roll: f32) -> MavMessage {
        message(30, |m| {
            if let MavMessage::Attitude(a) = m {
                a.time_boot_ms = 123_456;
                a.roll = roll;
                a.pitch = -0.25;
                a.yaw = 1.234_567_8;
            }
        })
    }

    fn key(names: &[&str]) -> Vec<String> {
        names.iter().map(|name| (*name).to_owned()).collect()
    }

    fn info(message: &MavMessage, field: &str) -> FieldInfo {
        *message
            .field_info()
            .iter()
            .find(|info| info.name == field)
            .expect("the field")
    }

    fn value_of(message: &MavMessage, field: &str) -> String {
        let value = message
            .fields()
            .into_iter()
            .find(|(name, _)| *name == field)
            .expect("the field")
            .1;
        value_text(&info(message, field), &value)
    }

    /// A component is its id and its `MAV_COMPONENT` name, or its id twice where the enum has
    /// none - every id from 0 to 255 as the C#'s enum writes it, when the tree is there.
    #[test]
    fn components_are_named_as_the_enum_names_them() {
        assert_eq!(component_text(1), "Comp 1 MAV_COMP_ID_AUTOPILOT1");
        assert_eq!(component_text(190), "Comp 190 MAV_COMP_ID_MISSIONPLANNER");
        assert_eq!(component_text(0), "Comp 0 MAV_COMP_ID_ALL");
        let unnamed = (0..=255u8)
            .find(|id| MavComponent(u32::from(*id)).name().is_none())
            .expect("an id the enum lacks");
        assert_eq!(component_text(unnamed), format!("Comp {unnamed} {unnamed}"));
        let Some(source) = csharp("ExtLibs/Mavlink/Mavlink.cs") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let start = source.find("public enum MAV_COMPONENT").expect("the enum");
        let body = &source[start..];
        let body = &body[..body.find("\n    };").expect("its end")];
        let mut named = std::collections::BTreeMap::new();
        for entry in body.split_whitespace() {
            if let Some((name, value)) = entry.trim_end_matches(',').split_once('=')
                && let Ok(value) = value.parse::<u8>()
            {
                named.insert(value, name.to_owned());
            }
        }
        assert!(named.len() > 100, "{}", named.len());
        for id in 0..=255u8 {
            let expected = named.get(&id).cloned().unwrap_or_else(|| id.to_string());
            assert_eq!(component_text(id), format!("Comp {id} {expected}"));
        }
    }

    /// Every message's fields are its C# struct's, in the struct's order - which is the order
    /// `GetFields` gives - and each type is the one the struct declares, as .NET names it.
    #[test]
    fn every_message_s_fields_are_its_csharp_struct_s() {
        let Some(source) = csharp("ExtLibs/Mavlink/Mavlink.cs") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let dotnet = |ty: &str| {
            let (base, array) = ty.strip_suffix("[]").map_or((ty, ""), |base| (base, "[]"));
            let name = match base {
                "byte" => "System.Byte",
                "sbyte" => "System.SByte",
                "short" => "System.Int16",
                "ushort" => "System.UInt16",
                "int" => "System.Int32",
                "uint" => "System.UInt32",
                "long" => "System.Int64",
                "ulong" => "System.UInt64",
                "float" => "System.Single",
                "double" => "System.Double",
                other => panic!("a type {other}"),
            };
            format!("{name}{array}")
        };
        let mut checked = 0;
        for described in mp_mavlink_dialects::all::DIALECT.messages() {
            let message = MavMessage::decode(described.id, &[]).expect("decodes");
            let head = format!(
                "public struct mavlink_{}_t\n",
                described.name.to_lowercase()
            );
            let Some(start) = source.find(&head) else {
                continue;
            };
            let body = &source[start..];
            let body = &body[..body.find("\n    };").expect("its end")];
            // A field is `public <type> <name>;` - `public  byte type;` for most, `public byte[]
            // passkey;` after a `MarshalAs` - where the constructors have parentheses.
            let declared: Vec<(String, String)> = body
                .lines()
                .filter_map(|line| line.trim().strip_prefix("public "))
                .filter(|line| line.ends_with(';') && !line.contains('(') && !line.contains('='))
                .map(|line| {
                    // `/*MAV_TYPE*/byte type;`: the enum's comment dropped.
                    let line = line.rsplit("*/").next().unwrap_or(line);
                    let mut words = line.split_whitespace();
                    let ty = words.next().expect("a type").to_owned();
                    let name = words.next().expect("a name");
                    (
                        name.trim_end_matches(';')
                            .trim_start_matches('@')
                            .to_owned(),
                        ty,
                    )
                })
                .collect();
            let ours: Vec<(String, String)> = message
                .field_info()
                .iter()
                .map(|info| (info.name.to_owned(), type_name(info)))
                .collect();
            let theirs: Vec<(String, String)> = declared
                .iter()
                .map(|(name, ty)| (name.clone(), dotnet(ty)))
                .collect();
            assert_eq!(ours, theirs, "{}", described.name);
            let names: Vec<&str> = message.fields().iter().map(|(name, _)| *name).collect();
            let infos: Vec<&str> = message.field_info().iter().map(|info| info.name).collect();
            assert_eq!(names, infos, "{}", described.name);
            checked += 1;
        }
        assert!(checked > 200, "{checked} messages checked");
    }

    /// Each value as `ToString()` writes it: a `float` to seven significant digits, a byte as a
    /// number, the text arrays as ASCII with their NULs, any other array joined by commas, and
    /// `time_unix_usec` as its date - or its number, past `DateTime.MaxValue`.
    #[test]
    fn values_are_written_as_the_csharp_writes_them() {
        let att = attitude(0.1);
        assert_eq!(value_of(&att, "roll"), "0.1");
        assert_eq!(value_of(&att, "pitch"), "-0.25");
        assert_eq!(value_of(&att, "yaw"), "1.234568");
        assert_eq!(value_of(&att, "time_boot_ms"), "123456");
        assert_eq!(value_of(&att, "rollspeed"), "0");
        let param = scripted::param("SYSID_THISMAV", 1.0, 6);
        assert_eq!(value_of(&param, "param_id"), "SYSID_THISMAV\0\0\0");
        assert_eq!(value_of(&param, "param_value"), "1");
        assert_eq!(value_of(&param, "param_type"), "6");
        let text = message(253, |m| {
            if let MavMessage::Statustext(s) = m {
                s.severity = 6;
                s.text[..5].copy_from_slice(b"Ready");
                s.text[5] = 0xe9;
            }
        });
        let shown_text = value_of(&text, "text");
        assert!(shown_text.starts_with("Ready?\0"), "{shown_text:?}");
        assert_eq!(shown_text.len(), 50);
        let battery = message(147, |m| {
            if let MavMessage::BatteryStatus(b) = m {
                b.voltages = [4200, 4150, u16::MAX, 0, 0, 0, 0, 0, 0, 0];
            }
        });
        assert_eq!(
            value_of(&battery, "voltages"),
            "4200,4150,65535,0,0,0,0,0,0,0"
        );
        let target = message(83, |m| {
            if let MavMessage::AttitudeTarget(a) = m {
                a.q = [1.0, 0.5, -0.333_333_34, 0.0];
            }
        });
        assert_eq!(value_of(&target, "q"), "1,0.5,-0.3333333,0");
        let time = |usec: u64| {
            message(2, |m| {
                if let MavMessage::SystemTime(t) = m {
                    t.time_unix_usec = usec;
                }
            })
        };
        assert_eq!(
            value_of(&time(1_758_980_710_123_456), "time_unix_usec"),
            "9/27/2025 1:45:10 PM"
        );
        assert_eq!(value_of(&time(0), "time_unix_usec"), "1/1/1970 12:00:00 AM");
        assert_eq!(
            value_of(&time(u64::MAX), "time_unix_usec"),
            u64::MAX.to_string()
        );
    }

    /// Each type as `FieldType.ToString()` names it: `char` and `uint8_t` both `byte`, an
    /// array with `[]`.
    #[test]
    fn types_are_named_as_dotnet_names_them() {
        let param = scripted::param("X", 0.0, 9);
        assert_eq!(type_name(&info(&param, "param_id")), "System.Byte[]");
        assert_eq!(type_name(&info(&param, "param_value")), "System.Single");
        assert_eq!(type_name(&info(&param, "param_count")), "System.UInt16");
        let hb = heartbeat();
        assert_eq!(type_name(&info(&hb, "custom_mode")), "System.UInt32");
        assert_eq!(type_name(&info(&hb, "mavlink_version")), "System.Byte");
        let time = MavMessage::decode(2, &[]).expect("SYSTEM_TIME");
        assert_eq!(type_name(&info(&time, "time_unix_usec")), "System.UInt64");
        let scaled = MavMessage::decode(26, &[]).expect("SCALED_IMU");
        assert_eq!(type_name(&info(&scaled, "xacc")), "System.Int16");
    }

    /// A field's text is `{0,-32} {1,20} {2,-20}`: the name padded to 32, the value right in 20,
    /// the type padded to 20; a longer value pushes the type on. What shows stops at a NUL.
    #[test]
    fn a_field_s_text_is_string_format_s() {
        let text = field_text("roll", "0.1", "System.Single");
        assert_eq!(
            text,
            format!(
                "roll{} {}0.1 System.Single{}",
                " ".repeat(28),
                " ".repeat(17),
                " ".repeat(7)
            )
        );
        assert_eq!(text.len(), 32 + 1 + 20 + 1 + 20);
        let long = field_text(
            "voltages",
            "4200,4150,65535,0,0,0,0,0,0,0",
            "System.UInt16[]",
        );
        assert!(long.contains(" 4200,4150,65535,0,0,0,0,0,0,0 System.UInt16[]"));
        let cut = field_text("param_id", "SYSID_THISMAV\0\0\0", "System.Byte[]");
        assert_eq!(
            shown(&cut),
            format!("param_id{}     SYSID_THISMAV", " ".repeat(24))
        );
    }

    /// A message's text: its name, the rate to a tenth, the id, and the bytes a second whole,
    /// each rounded as .NET rounds, half away from zero.
    #[test]
    fn a_message_s_text_is_its_rate_id_and_bytes() {
        assert_eq!(
            header_text("ATTITUDE", 30, 10.0, 390.0),
            "ATTITUDE (10.0 Hz, #30) 390Bps"
        );
        assert_eq!(
            header_text("HEARTBEAT", 0, 0.25, 20.5),
            "HEARTBEAT (0.3 Hz, #0) 21Bps"
        );
        assert_eq!(
            header_text("SYS_STATUS", 1, 0.0, 0.0),
            "SYS_STATUS (0.0 Hz, #1) 0Bps"
        );
    }

    /// The culture's order, not the characters': `_` before a digit and a digit before a letter,
    /// white space first, case only where nothing else differs.
    #[test]
    fn the_tree_sorts_as_the_culture_compares() {
        let mut names = vec![
            "SYSTEM_TIME (",
            "GPS2_RAW (",
            "SYS_STATUS (",
            "GPS_RAW_INT (",
            "AHRS2 (",
            "AHRS (",
            "ATTITUDE",
            "attitude",
        ];
        names.sort_by(|a, b| culture_cmp(a, b));
        assert_eq!(
            names,
            [
                "AHRS (",
                "AHRS2 (",
                "attitude",
                "ATTITUDE",
                "GPS_RAW_INT (",
                "GPS2_RAW (",
                "SYS_STATUS (",
                "SYSTEM_TIME (",
            ]
        );
        let padded = |name: &str| field_text(name, "0", "System.UInt16");
        let mut fields = [
            padded("chan10_raw"),
            padded("chan2_raw"),
            padded("chan1_raw"),
        ];
        fields.sort_by(|a, b| culture_cmp(a, b));
        assert!(fields[0].starts_with("chan1_raw "));
        assert!(fields[1].starts_with("chan10_raw "));
        assert!(fields[2].starts_with("chan2_raw "));
        assert_eq!(culture_cmp("Vehicle 1", "Vehicle 255"), Ordering::Less);
        assert_eq!(culture_cmp("Vehicle 10", "Vehicle 2"), Ordering::Less);
    }

    /// `Update`: a node per system, component, message and field of what was heard, each level
    /// sorted by its text; the message's text its rate and bytes a second at the time; each
    /// field's its value and type. Nothing new is added the second time, and the texts follow.
    #[test]
    fn update_builds_the_tree_from_what_was_heard() {
        let start = Instant::now();
        let mut mavi = PacketInspector::default();
        for second in 0..3 {
            let at = start + Duration::from_secs(second);
            mavi.add(1, 1, 0, packet(1, 1, heartbeat(), at), 21, at);
        }
        let status = packet(1, 1, MavMessage::decode(1, &[]).expect("SYS_STATUS"), start);
        let time = packet(
            1,
            1,
            MavMessage::decode(2, &[]).expect("SYSTEM_TIME"),
            start,
        );
        mavi.add(1, 1, 2, time, time.length, start);
        mavi.add(1, 1, 1, status, status.length, start);
        let att = packet(1, 1, attitude(0.1), start);
        mavi.add(1, 1, 30, att, att.length, start);
        let gcs = packet(255, 190, heartbeat(), start);
        mavi.add(255, 190, 0, gcs, gcs.length, start);
        let mut tree = Vec::new();
        let now = start + Duration::from_millis(2_500);
        assert!(update(&mut tree, &mavi, now));
        let texts = |nodes: &[Node]| nodes.iter().map(|n| n.text.clone()).collect::<Vec<_>>();
        assert_eq!(texts(&tree), ["Vehicle 1", "Vehicle 255"]);
        assert_eq!(texts(&tree[0].nodes), ["Comp 1 MAV_COMP_ID_AUTOPILOT1"]);
        assert_eq!(
            texts(&tree[1].nodes),
            ["Comp 190 MAV_COMP_ID_MISSIONPLANNER"]
        );
        let messages = &tree[0].nodes[0].nodes;
        let names: Vec<&str> = messages.iter().map(|n| n.name.as_str()).collect();
        // ATTITUDE, HEARTBEAT, SYS_STATUS, SYSTEM_TIME: `_` before `T`.
        assert_eq!(names, ["30", "0", "1", "2"]);
        // Three heartbeats over the 2.5 s since the first, 21 bytes each.
        assert_eq!(messages[1].text, "HEARTBEAT (1.2 Hz, #0) 25Bps");
        let fields: Vec<&str> = messages[1].nodes.iter().map(|n| n.name.as_str()).collect();
        assert_eq!(
            fields,
            [
                "autopilot",
                "base_mode",
                "custom_mode",
                "mavlink_version",
                "system_status",
                "type"
            ]
        );
        let base_mode = &messages[1].nodes[1];
        assert_eq!(base_mode.text, field_text("base_mode", "81", "System.Byte"));
        assert_eq!(
            base_mode.tag,
            Some(("81".to_owned(), "System.Byte".to_owned()))
        );
        let roll = node_at(&tree, &key(&["1", "1", "30", "roll"])).expect("roll");
        assert_eq!(roll.text, field_text("roll", "0.1", "System.Single"));
        // Nothing new: nothing added, the rate followed on - the heartbeats at 1 s and 2 s over
        // the three seconds before 3.9 s.
        let later = start + Duration::from_millis(3_900);
        assert!(!update(&mut tree, &mavi, later));
        let heartbeats = node_at(&tree, &key(&["1", "1", "0"])).expect("heartbeat");
        assert_eq!(heartbeats.text, "HEARTBEAT (0.7 Hz, #0) 14Bps");
        // A newer attitude: its value is the newest one's.
        let newer = packet(1, 1, attitude(-1.5), later);
        mavi.add(1, 1, 30, newer, newer.length, later);
        update(&mut tree, &mavi, later);
        let roll = node_at(&tree, &key(&["1", "1", "30", "roll"])).expect("roll");
        assert_eq!(roll.tag.as_ref().map(|t| t.0.as_str()), Some("-1.5"));
    }

    /// What the link reads is added; what it writes only while "Show GCS Traffic" is ticked.
    #[test]
    fn written_packets_are_added_only_while_gcs_traffic_is_ticked() {
        let now = Instant::now();
        let mut inspector = Inspector::new(now);
        let mut sent = packet(255, 190, heartbeat(), now);
        sent.sent = true;
        inspector.hear(&packet(1, 1, heartbeat(), now));
        inspector.hear(&sent);
        let held = |inspector: &Inspector| {
            inspector
                .mavi
                .lock()
                .expect("not poisoned")
                .packet_messages()
                .iter()
                .map(|p| (p.sysid, p.compid))
                .collect::<Vec<_>>()
        };
        assert_eq!(held(&inspector), [(1, 1)]);
        inspector.toggle_gcs_traffic();
        assert!(inspector.gcs_traffic());
        inspector.hear(&sent);
        assert_eq!(held(&inspector), [(1, 1), (255, 190)]);
        inspector.toggle_gcs_traffic();
        assert!(!inspector.gcs_traffic());
    }

    /// `timer1`: the tree is brought up to date 333 ms after the form opens and every 333 ms
    /// after, and not between.
    #[test]
    fn the_tree_is_updated_every_333_ms() {
        let start = Instant::now();
        let mut inspector = Inspector::new(start);
        inspector.hear(&packet(1, 1, heartbeat(), start));
        inspector.timer(start + Duration::from_millis(332));
        assert!(inspector.tree.is_empty());
        assert_eq!(inspector.updates, 0);
        let first = start + UPDATE_EVERY;
        inspector.timer(first);
        assert_eq!(inspector.updates, 1);
        assert_eq!(inspector.tree.len(), 1);
        inspector.timer(first + Duration::from_millis(300));
        assert_eq!(inspector.updates, 1);
        inspector.timer(first + UPDATE_EVERY);
        assert_eq!(inspector.updates, 2);
    }

    /// An inspector with a vehicle's heartbeat and attitude in its tree.
    fn filled() -> Inspector {
        let start = Instant::now();
        let mut inspector = Inspector::new(start);
        inspector.hear(&packet(1, 1, heartbeat(), start));
        inspector.hear(&packet(1, 1, attitude(0.1), start));
        inspector.timer(start + UPDATE_EVERY);
        inspector
    }

    /// The tree shows its vehicles; a plus box shows a node's children under it, a minus box
    /// hides them again, a field has no box.
    #[test]
    fn the_rows_are_what_is_expanded() {
        let mut inspector = filled();
        let rows = inspector.rows();
        assert_eq!(rows.len(), 1);
        assert!(rows[0].parent && !rows[0].expanded);
        inspector.toggle(&key(&["1"]));
        inspector.toggle(&key(&["1", "1"]));
        inspector.toggle(&key(&["1", "1", "30"]));
        let rows = inspector.rows();
        let texts: Vec<String> = rows.iter().map(|r| r.text.clone()).collect();
        assert_eq!(texts[0], "Vehicle 1");
        assert_eq!(texts[1], "Comp 1 MAV_COMP_ID_AUTOPILOT1");
        assert!(texts[2].starts_with("ATTITUDE ("));
        assert_eq!(rows[3].key, key(&["1", "1", "30", "pitch"]));
        assert_eq!(rows[3].depth, 3);
        assert!(!rows[3].parent);
        assert!(texts[10].starts_with("HEARTBEAT ("), "{texts:?}");
        assert_eq!(rows.len(), 11);
        inspector.toggle(&key(&["1", "1"]));
        assert_eq!(inspector.rows().len(), 2);
        // A node that is not there is not opened.
        inspector.toggle(&key(&["9"]));
        assert!(!inspector.is_expanded(&key(&["9"])));
    }

    /// `AfterSelect`: a vehicle's node changes nothing; a component's, a message's or a
    /// field's enables Graph It and keeps its path, the texts joined by `\`.
    #[test]
    fn selecting_below_a_vehicle_enables_graph_it() {
        let mut inspector = filled();
        inspector.select(&key(&["1"]));
        assert!(!inspector.graph_it);
        assert_eq!(inspector.selected_path, None);
        assert_eq!(inspector.selected(), Some(key(&["1"]).as_slice()));
        inspector.select(&key(&["1", "1"]));
        assert!(inspector.graph_it);
        assert_eq!(
            inspector.selected_path.as_deref(),
            Some("Vehicle 1\\Comp 1 MAV_COMP_ID_AUTOPILOT1")
        );
        inspector.select(&key(&["1", "1", "30", "roll"]));
        let path = inspector.selected_path.clone().expect("a path");
        let parts: Vec<&str> = path.split('\\').collect();
        assert_eq!(parts.len(), 4);
        assert!(parts[2].starts_with("ATTITUDE ("));
        assert_eq!(parts[3], field_text("roll", "0.1", "System.Single"));
        // A vehicle's node after it: still enabled, the field's path kept.
        inspector.select(&key(&["1"]));
        assert!(inspector.graph_it);
        assert_eq!(inspector.selected_path.as_deref(), Some(path.as_str()));
    }

    /// Graph It's path read as the handler reads it.
    #[test]
    fn the_path_is_read_as_the_handler_reads_it() {
        let field = field_text("roll", "0.1", "System.Single");
        let path = format!(
            "Vehicle 1\\Comp 1 MAV_COMP_ID_AUTOPILOT1\\ATTITUDE (10.0 Hz, #30) 390Bps\\{field}"
        );
        assert_eq!(
            target(&path),
            Ok(Some(Target {
                sysid: 1,
                compid: 1,
                msgid: 30,
                message: "ATTITUDE".to_owned(),
                field: "roll".to_owned(),
            }))
        );
        assert_eq!(target("Vehicle 1\\Comp 1 MAV_COMP_ID_AUTOPILOT1"), Ok(None));
        assert_eq!(
            target("Vehicle\\Comp 1\\X (1 Hz, #2)\\f"),
            Err(OUT_OF_RANGE)
        );
        assert_eq!(
            target("Vehicle x\\Comp 1\\X (1 Hz, #2)\\f"),
            Err(NOT_A_NUMBER)
        );
    }

    /// Graph It: "Points of history?" holding 50; OK with a number keeps it and, for a field,
    /// opens its graph - titled `MESSAGE.field`, the Y axis its units - and disables Graph It;
    /// Cancel keeps the history and opens it all the same; a component's path asks and does
    /// nothing; what `int.Parse` and a negative capacity throw is the error.
    #[test]
    fn graph_it_asks_then_graphs_a_field() {
        let now = Instant::now();
        let mut inspector = filled();
        inspector.press_graph_it();
        assert!(
            inspector.asking.is_none(),
            "disabled until a node is selected"
        );
        inspector.select(&key(&["1", "1"]));
        inspector.press_graph_it();
        let asking = inspector.asking.as_ref().expect("the question");
        assert_eq!((asking.title, asking.prompt), (POINTS_TITLE, POINTS_PROMPT));
        assert_eq!(asking.field.value(), "50");
        inspector.type_answer("20");
        assert_eq!(inspector.prompt_done(true, now), Ok(()));
        assert_eq!(inspector.history, 20);
        assert!(inspector.take_answered().is_some(), "OK's answer is kept");
        assert!(
            inspector.graphs.is_empty(),
            "a component's path graphs nothing"
        );
        assert!(inspector.graph_it);

        inspector.select(&key(&["1", "1", "30", "roll"]));
        inspector.press_graph_it();
        inspector.type_answer("lots");
        assert_eq!(
            inspector.prompt_done(true, now),
            Err(NOT_A_NUMBER.to_owned())
        );
        assert!(inspector.take_answered().is_some());
        assert!(inspector.graphs.is_empty());
        assert_eq!(inspector.history, 20);
        inspector.press_graph_it();
        inspector.type_answer("-3");
        assert_eq!(
            inspector.prompt_done(true, now),
            Err(NEGATIVE_HISTORY.to_owned())
        );
        assert_eq!(inspector.history, -3);
        inspector.press_graph_it();
        inspector.type_answer("99999999999");
        assert_eq!(inspector.prompt_done(true, now), Err(TOO_BIG.to_owned()));
        assert!(inspector.take_answered().is_some());

        inspector.history = 20;
        inspector.press_graph_it();
        inspector.type_answer("7");
        assert_eq!(inspector.prompt_done(false, now), Ok(()), "Cancel");
        assert_eq!(inspector.history, 20);
        assert!(inspector.take_answered().is_none());
        assert_eq!(inspector.graphs.len(), 1);
        assert!(!inspector.graph_it);
        let graph = &inspector.graphs[0];
        assert_eq!(graph.y_title, "[rad]");
        assert_eq!(graph.shown.list.len(), 1);
        assert_eq!(graph.shown.list[0].label, "ATTITUDE.roll");
        assert_eq!(graph.shown.list[0].colour, COLOURS[0]);
        assert_eq!(graph.shown.capacity, 20);
        inspector.press_graph_it();
        assert!(inspector.asking.is_none(), "disabled after a graph");
        inspector.close_graph(0);
        assert!(inspector.graphs.is_empty());
    }

    /// Units: the field's `[Units]`, brackets and all, `""` for a field without, "Y Axis" for a
    /// field the message has not.
    #[test]
    fn the_y_axis_is_the_field_s_units() {
        let att = attitude(0.0);
        assert_eq!(units_title(&att, "roll"), "[rad]");
        assert_eq!(units_title(&att, "time_boot_ms"), "[ms]");
        assert_eq!(units_title(&heartbeat(), "type"), "");
        assert_eq!(units_title(&att, "nothing"), "Y Axis");
    }

    /// A graph takes the packets of its message from its system and component, each at its
    /// time: a value a point; an array a point on a curve per element, the curves made as the
    /// elements need them and coloured in turn; the oldest dropped past the history; none held
    /// with a history of 0.
    #[test]
    fn a_graph_takes_its_message_s_points() {
        let now = Instant::now();
        let roll = Target {
            sysid: 1,
            compid: 1,
            msgid: 30,
            message: "ATTITUDE".to_owned(),
            field: "roll".to_owned(),
        };
        let mut curves = Curves::new("ATTITUDE.roll".to_owned(), 2);
        for (seconds, value) in [(0_i64, 0.5_f32), (1, 1.5), (2, 2.5)] {
            let mut p = packet(1, 1, attitude(value), now);
            p.rxtime = DateTime::from_ticks(UNIX_EPOCH_TICKS + 10_000_000 * seconds);
            curves.add(&roll, &p);
        }
        curves.add(&roll, &packet(2, 1, attitude(9.0), now));
        curves.add(&roll, &packet(1, 1, heartbeat(), now));
        assert_eq!(curves.list.len(), 1);
        let points: Vec<(f64, f64)> = curves.list[0].points.iter().copied().collect();
        let second = 1.0 / 86_400.0;
        assert_eq!(points.len(), 2);
        assert!((points[0].0 - (25_569.0 + second)).abs() < 1e-9);
        assert_eq!((points[0].1, points[1].1), (1.5, 2.5));

        let voltages = Target {
            sysid: 1,
            compid: 1,
            msgid: 147,
            message: "BATTERY_STATUS".to_owned(),
            field: "voltages".to_owned(),
        };
        let battery = message(147, |m| {
            if let MavMessage::BatteryStatus(b) = m {
                b.voltages[0] = 4200;
                b.voltages[7] = 3900;
            }
        });
        let mut curves = Curves::new("BATTERY_STATUS.voltages".to_owned(), 50);
        curves.add(&voltages, &packet(1, 1, battery, now));
        assert_eq!(curves.list.len(), 10);
        assert_eq!(curves.list[0].label, "BATTERY_STATUS.voltages");
        assert_eq!(curves.list[1].label, "voltages[1]");
        assert_eq!(curves.list[7].label, "voltages[7]");
        let colours: Vec<u32> = curves.list.iter().map(|c| c.colour).collect();
        assert_eq!(
            &colours[..7],
            &[
                COLOURS[0], COLOURS[1], COLOURS[2], COLOURS[3], COLOURS[4], COLOURS[5], COLOURS[0]
            ]
        );
        assert_eq!(curves.list[7].points.front().map(|p| p.1), Some(3900.0));

        let mut empty = Curves::new("ATTITUDE.roll".to_owned(), 0);
        empty.add(&roll, &packet(1, 1, attitude(1.0), now));
        assert!(empty.list[0].points.is_empty());
    }

    /// `XDate`: days since 1899-12-30, to the millisecond.
    #[test]
    fn an_xdate_counts_days_from_1899() {
        assert_eq!(xdate(DateTime::from_ticks(UNIX_EPOCH_TICKS)), 25_569.0);
        assert_eq!(xdate(DateTime::from_ticks(XDATE_EPOCH_TICKS)), 0.0);
        let half_day = DateTime::from_ticks(UNIX_EPOCH_TICKS + 432_000_000_000 + 9_999);
        assert_eq!(xdate(half_day), 25_569.5);
    }

    /// A date axis over five seconds steps in seconds: the grace either side, the ends out to
    /// whole seconds, a tic and a `HH:mm:ss.fff` label each second. Longer and shorter spans
    /// step as `CalcDateStepSize` steps them.
    #[test]
    fn the_date_axis_steps_as_zedgraph_s() {
        let second = 1.0 / 86_400.0;
        let start = 25_569.0 + 10.0 * second;
        let scale = DateScale::pick(start, start + 5.0 * second);
        assert_eq!(scale.unit, DateUnit::Second);
        assert_eq!(scale.major_step, 1.0);
        assert!((scale.min - (25_569.0 + 9.0 * second)).abs() < 1e-9);
        assert!((scale.max - (25_569.0 + 16.0 * second)).abs() < 1e-9);
        let labels: Vec<String> = scale.tics().into_iter().map(DateScale::label).collect();
        assert_eq!(labels.first().map(String::as_str), Some("00:00:09.000"));
        assert_eq!(labels.last().map(String::as_str), Some("00:00:16.000"));
        assert_eq!(labels.len(), 8);
        // A minute of data, 72 s with its grace: seconds, in fifteens.
        let scale = DateScale::pick(start, start + 60.0 * second);
        assert_eq!((scale.major_step, scale.unit), (15.0, DateUnit::Second));
        // Half a second: milliseconds, 1, 2 or 5 times a power of ten.
        let scale = DateScale::pick(start, start + 0.5 * second);
        assert_eq!(
            (scale.major_step, scale.unit),
            (100.0, DateUnit::Millisecond)
        );
        // One point: ZedGraph widens a range of nothing by a twentieth of the value - years.
        assert_eq!(DateScale::pick(start, start).unit, DateUnit::Year);
        assert_eq!(date_step(20.0 / 1_440.0, 7.0), (5.0, DateUnit::Minute));
        assert_eq!(date_step(0.5, 7.0), (2.0, DateUnit::Hour));
        assert_eq!(date_step(5.0, 7.0), (1.0, DateUnit::Day));
    }

    /// The product's path: the Advanced page's button opens the window over the application's
    /// link; what the vehicle sends is in the tree at the timer's next tick; and with "Show GCS
    /// Traffic" ticked, what the application writes is too, from 255/190.
    #[test]
    fn the_advanced_page_s_button_opens_a_window_on_the_link() {
        use super::super::extra_setup::ExtraSetup;
        let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default());
        let mut pages = ExtraSetup::default();
        let start = Instant::now();
        assert!(super::super::advanced::click(
            "but_mavinspector",
            &mut pages.fft,
            &mut pages.inspector,
            start,
        ));
        pages.inspector.tick(&telemetry, start);
        vehicle.heartbeat();
        vehicle.send(&attitude(0.1));
        let held = |pages: &ExtraSetup| {
            pages.inspector.window.as_ref().map_or(0, |window| {
                window
                    .mavi
                    .lock()
                    .map_or(0, |mavi| mavi.packet_messages().len())
            })
        };
        until("the vehicle's packets", || held(&pages) == 2);
        pages.inspector.tick(&telemetry, start + UPDATE_EVERY);
        let window = pages.inspector.window.as_ref().expect("open");
        assert_eq!(window.updates, 1);
        assert_eq!(window.tree.len(), 1);
        assert_eq!(window.tree[0].text, "Vehicle 1");
        assert!(node_at(&window.tree, &key(&["1", "1", "30", "roll"])).is_some());
        // Written while unticked: not added.
        assert!(telemetry.send(&heartbeat()));
        until("the vehicle to hear it", || {
            vehicle.read();
            vehicle.count(|m| matches!(m, MavMessage::Heartbeat(_))) > 0
        });
        assert_eq!(held(&pages), 2);
        if let Some(window) = pages.inspector.window.as_mut() {
            window.toggle_gcs_traffic();
        }
        assert!(telemetry.send(&heartbeat()));
        until("the written heartbeat", || held(&pages) >= 3);
        pages
            .inspector
            .tick(&telemetry, start + UPDATE_EVERY + UPDATE_EVERY);
        let window = pages.inspector.window.as_ref().expect("open");
        let vehicles: Vec<&str> = window.tree.iter().map(|n| n.text.as_str()).collect();
        assert_eq!(vehicles, ["Vehicle 1", "Vehicle 255"]);
        assert_eq!(
            window.tree[1].nodes[0].text,
            "Comp 190 MAV_COMP_ID_MISSIONPLANNER"
        );
        // The close box: the subscription ends with the window.
        pages.inspector.close();
        assert!(pages.inspector.window.is_none());
        assert_eq!(pages.inspector.opened, 1);
    }

    /// The Designer's words and places are these.
    #[test]
    fn the_designer_s_words_and_places_are_these() {
        let Some(source) = csharp("Controls/MAVLinkInspector.cs") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let has = |line: &str| assert!(source.contains(line), "{line}");
        has(&format!("this.Text = \"{FORM_TEXT}\";"));
        has("this.ClientSize = new System.Drawing.Size(526, 311);");
        has(&format!("this.but_graphit.Text = \"{GRAPH_IT}\";"));
        has("this.but_graphit.Location = new System.Drawing.Point(12, 3);");
        has("this.but_graphit.Size = new System.Drawing.Size(75, 23);");
        has("this.but_graphit.Enabled = false;");
        has(&format!("this.chk_gcstraffic.Text = \"{GCS_TRAFFIC}\";"));
        has("this.chk_gcstraffic.Location = new System.Drawing.Point(93, 5);");
        has("this.groupBox1.Location = new System.Drawing.Point(0, 30);");
        has("this.groupBox1.Size = new System.Drawing.Size(527, 278);");
        has("this.treeView1.Location = new System.Drawing.Point(3, 16);");
        has("this.treeView1.Size = new System.Drawing.Size(521, 259);");
        has("this.timer1.Interval = 333;");
        has("int history = 50;");
        has(&format!(
            "InputBox.Show(\"{POINTS_TITLE}\", \"{POINTS_PROMPT}\", ref history);"
        ));
        has("var form = new Form() { Size = new Size(640, 480) };");
        has("var timer = new Timer() { Interval = 100 };");
        has("zg1.GraphPane.XAxis.Scale.Format = \"HH:mm:ss.fff\";");
        assert_eq!(
            DateScale::label(25_569.0 + 3_723.004 / 86_400.0),
            "01:02:03.004"
        );
        assert_eq!(X_FORMAT, "%H:%M:%S%.3f");
        has("String.Format(\"{0,-32} {1,20} {2,-20}\"");
        has(".ToString(\"0.0 Hz\")");
        has(".ToString(\"0Bps\")");
        for field in TEXT_FIELDS {
            has(&format!("field.Name == \"{field}\""));
        }
        // The invisible combo boxes, and no owner-drawn tree to raise `DrawNode`.
        has("this.comboBox1.Visible = false;");
        has("this.comboBox2.Visible = false;");
        assert!(!source.contains("DrawMode"), "the tree is not owner-drawn");
    }

    /// Every fact the GUI script asserts on is one this window records, and every control it
    /// clicks is one it draws: the tree's, for the nodes a vehicle's first packets make.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_window_has() {
        let script = include_str!("../../../../tests/gui/config-mavlink-inspector.gui");
        let source = include_str!("mavlink_inspector.rs");
        let optional = include_str!("optional.rs");
        let inspector = filled();
        let node = |path: &str| {
            let names: Vec<&str> = path.split('-').collect();
            node_at(&inspector.tree, &key(&names)).cloned()
        };
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(fact)) if fact.starts_with("config.inspector.") => {
                    let family: Vec<&str> = fact.splitn(4, '.').take(3).collect();
                    let family = family.join(".");
                    assert!(source.contains(&format!("\"{family}")), "{fact}");
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("inspector-") => {
                    clicks += 1;
                    if let Some(path) = id.strip_prefix("inspector-toggle-") {
                        assert!(node(path).is_some_and(|n| !n.nodes.is_empty()), "{id}");
                    } else if let Some(path) = id.strip_prefix("inspector-node-") {
                        assert!(node(path).is_some(), "{id}");
                    } else if id.starts_with("inspector-graph-") {
                        assert!(
                            source.contains("\"inspector-graph\"")
                                && source.contains("format!(\"{prefix}-{index}-close\")")
                        );
                    } else if id.starts_with("inspector-points-") {
                        assert!(optional.contains(&format!("\"{id}\"")), "{id}");
                    } else {
                        assert!(source.contains(&format!("\"{id}\"")), "{id}");
                    }
                }
                _ => {}
            }
        }
        assert!(facts >= 20, "{facts} facts");
        assert!(clicks >= 8, "{clicks} clicks");
        assert_eq!(
            row_id("toggle", &key(&["1", "1", "30"])),
            "inspector-toggle-1-1-30"
        );
    }
}
