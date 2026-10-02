//! The DroneCAN Inspector and its Subscribe window: `Controls/DroneCANInspector.cs` and
//! `Controls/DroneCANSubscriber.cs`, which the DroneCAN page's Inspector button opens over the
//! page's node (`new DroneCANInspector(can).Show()`, `ConfigDroneCAN.cs:714-723`).
//!
//! What the window does:
//!
//! * every message the node hears - its own, sent and handed back, among them - goes into a
//!   `PacketInspector` keyed by the source node and the data type id with the transfer's bytes
//!   (`Can_MessageReceived`, `:51-54`); every 333 ms (`timer1`) the tree is brought up to date
//!   (`Update`, `:56-117`): a node per source, `"ID n"` when first seen and then `"ID n - name
//!   ~bytesBps"` with `GetNodeName`'s name; under it a node per data type id whose text is
//!   `TypeName (r.r Hz, #id) ~bytesBps`; under that the message's fields as `PopulateMSG` lays
//!   them out (`:119-200`): a scalar as `{name,-32} {value,20} {type,-20}` with the .NET type's
//!   name, an array of numbers joined by commas (an empty one with no value), a byte array named
//!   `param_id`, `text`, `string_value` or `name` as its ASCII text, a nested type as a node of
//!   its fields; the tree sorted when a node was added (`treeView1.Sort()`, a culture sort of
//!   the texts, the fields' rows included, so a message's fields read alphabetically);
//! * a node selected makes `selectedmsgid` the names from the root joined by `/` and enables
//!   Graph It and Subscribe (`treeView1_AfterSelect`, `:326-346`);
//! * Graph It asks "Points of history?" (`InputBox`, 50 to start) and opens a graph of the
//!   selected path: a number as one curve, an array as a curve an element, a nested type as a
//!   curve a field; a point at the wall clock as each message comes, 100 ms redraws, Graph It
//!   disabled while the graph is open (`but_graphit_Click`, `AddToGraph`, `:347-477`);
//! * Subscribe opens the Subscriber (`but_subscribe_Click`, `:479-484`): a combo of the type
//!   names heard (filled once a second), a line count (100, up to 10,000) and a box that gets
//!   each message of the chosen type as its JSON, the box kept to the last count of lines as the
//!   C#'s arithmetic keeps it (`DroneCANSubscriber.cs:97-131`);
//! * closing the window ends its timer and subscription (`:289-294`); closing a graph ends
//!   its handler (`:441`).
//!
//! Where this is not the C#, each written at its site:
//!
//! * the C# reflects over the generated class of every one of the 155 data types; here the 27
//!   types `mp_dronecan::dsdl` decodes show their fields, and any other type shows one row,
//!   `payload`, of the transfer's bytes as `Byte[]` ([`fields`]); none of the 27 has an array
//!   of nested types, so `PopulateMSG`'s `name[i]` nodes (`:148-170`) have nothing to show;
//! * `time_unix_usec` is shown as a date in the C#; no decoded type here has the field;
//! * the forms are drawn over the page, one Inspector and one Subscriber at a time, where the
//!   C# shows them as forms of their own;
//! * the Subscriber's line count is stepped with two buttons, not typed;
//! * the tree is drawn plain: `MyTreeView`'s double buffering and `DrawNode` are the Windows
//!   control's flicker fixes.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeSet;
use std::time::{Duration, Instant};

use gpui::{
    AnyElement, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px, rgb,
};
use mp_dronecan::dsdl::{Message, NumericValue, Value};
use mp_dronecan::node::Received;
use mp_link::inspector::PacketInspector;
use mp_mission::dotnet::{format_f64, general_f32};

use super::mavlink_inspector::{self as mavi, Curves, Node, Row, Y_TITLE};
use super::optional::{InputBox, at, button, input_box};
use crate::MissionPlanner;
use crate::textfield::KeyOutcome;
use crate::ui::theme;

/// `this.Text`. `// C#: Controls/DroneCANInspector.cs:267`
pub const FORM_TEXT: &str = "UAVCAN Inspector";
/// `ClientSize`. `// C#: Controls/DroneCANInspector.cs:262`
pub const CLIENT: (f32, f32) = (698.0, 311.0);
/// `but_graphit`: `Location`, `Size`, `Text`. `// C#: Controls/DroneCANInspector.cs:242-246`
const GRAPH_IT_AT: (f32, f32, f32, f32) = (12.0, 3.0, 75.0, 23.0);
pub const GRAPH_IT: &str = "Graph It";
/// `but_subscribe`. `// C#: Controls/DroneCANInspector.cs:252-256`
const SUBSCRIBE_AT: (f32, f32, f32, f32) = (93.0, 3.0, 75.0, 23.0);
pub const SUBSCRIBE: &str = "Subscribe";
/// `groupBox1`, anchored all round. `// C#: Controls/DroneCANInspector.cs:229-231`
const GROUP_AT: (f32, f32, f32, f32) = (0.0, 30.0, 699.0, 278.0);
/// `treeView1`, docked in the group. `// C#: Controls/DroneCANInspector.cs:216-218`
const TREE_AT: (f32, f32, f32, f32) = (3.0, 16.0, 693.0, 259.0);
/// `timer1.Interval`. `// C#: Controls/DroneCANInspector.cs:238`
pub const UPDATE_EVERY: Duration = Duration::from_millis(333);
/// The graph's timer. `// C#: Controls/DroneCANInspector.cs:427`
pub const GRAPH_EVERY: Duration = Duration::from_millis(100);
/// Graph It's question. `// C#: Controls/DroneCANInspector.cs:349`
pub const POINTS_TITLE: &str = "Points";
pub const POINTS_PROMPT: &str = "Points of history?";
/// `history`'s start. `// C#: Controls/DroneCANInspector.cs:347`
pub const HISTORY: i32 = 50;
/// The field named `payload` for a type nothing here decodes.
pub const PAYLOAD: &str = "payload";

/// The Subscriber: `this.Size`. `// C#: Controls/DroneCANSubscriber.cs:81`
pub const SUBSCRIBER_SIZE: (f32, f32) = (419.0, 337.0);
/// `cmb_msg`. `// C#: Controls/DroneCANSubscriber.cs:40-43`
const SUBSCRIBER_COMBO_AT: (f32, f32, f32, f32) = (124.0, 3.0, 221.0, 21.0);
/// `num_lines`: its place, its `Maximum` and its `Value`. `// C#: Controls/DroneCANSubscriber.cs:48-60`
const SUBSCRIBER_LINES_AT: (f32, f32, f32, f32) = (351.0, 4.0, 61.0, 20.0);
pub const LINES_MAX: i32 = 10_000;
pub const LINES: i32 = 100;
/// `txt_packet`. `// C#: Controls/DroneCANSubscriber.cs:66-70`
const SUBSCRIBER_TEXT_AT: (f32, f32, f32, f32) = (3.0, 30.0, 409.0, 299.0);
/// The Subscriber's combo timer. `// C#: Controls/DroneCANSubscriber.cs:93`
pub const TYPES_EVERY: Duration = Duration::from_secs(1);

// ---------------------------------------------------------------------------------------------
// A message's fields, as reflection lays them out.
// ---------------------------------------------------------------------------------------------

/// A field's value, as `PopulateMSG` tells them apart: by the .NET type of the field.
/// `// C#: Controls/DroneCANInspector.cs:119-200`
#[derive(Debug, Clone, PartialEq)]
pub enum FieldValue {
    /// A number, a `bool` or an enum: its text, as `String.Format("{1,20}")` writes it, and its
    /// type's name; the number, for a graph.
    Scalar {
        text: String,
        type_name: &'static str,
        number: Option<f64>,
    },
    /// A byte array the C# shows as ASCII: `param_id`, `text`, `string_value` or `name`.
    Text(String),
    /// An array of numbers: joined by commas; an empty one shows no value.
    Numbers {
        values: Vec<f64>,
        type_name: &'static str,
    },
    /// A nested type: a node of its fields.
    Struct(Vec<Field>),
}

/// A field: its name in the generated class, and its value.
#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    /// `field.Name`.
    pub name: String,
    /// `field.GetValue(message)`.
    pub value: FieldValue,
}

/// The byte arrays the C# shows as text.
/// `// C#: Controls/DroneCANInspector.cs:139-140`
const TEXT_FIELDS: [&str; 4] = ["param_id", "text", "string_value", "name"];

fn field(name: &str, value: FieldValue) -> Field {
    Field {
        name: name.to_owned(),
        value,
    }
}

/// A number's field: `Byte`, `UInt16`... as the `using` aliases name the .NET types.
/// `// C#: ExtLibs/DroneCAN/out/include/uavcan.protocol.NodeStatus.cs:2-12`
fn scalar(name: &str, text: String, type_name: &'static str, number: f64) -> Field {
    field(
        name,
        FieldValue::Scalar {
            text,
            type_name,
            number: Some(number),
        },
    )
}

fn u8_field(name: &str, value: u8) -> Field {
    scalar(name, value.to_string(), "Byte", f64::from(value))
}
fn i8_field(name: &str, value: i8) -> Field {
    scalar(name, value.to_string(), "SByte", f64::from(value))
}
fn u16_field(name: &str, value: u16) -> Field {
    scalar(name, value.to_string(), "UInt16", f64::from(value))
}
fn i16_field(name: &str, value: i16) -> Field {
    scalar(name, value.to_string(), "Int16", f64::from(value))
}
fn u32_field(name: &str, value: u32) -> Field {
    scalar(name, value.to_string(), "UInt32", f64::from(value))
}
#[allow(clippy::cast_precision_loss)] // `IConvertible.ToDouble`, as the C# converts
fn u64_field(name: &str, value: u64) -> Field {
    scalar(name, value.to_string(), "UInt64", value as f64)
}
#[allow(clippy::cast_precision_loss)] // as above
fn i64_field(name: &str, value: i64) -> Field {
    scalar(name, value.to_string(), "Int64", value as f64)
}
fn f32_field(name: &str, value: f32) -> Field {
    scalar(name, general_f32(value), "Single", f64::from(value))
}
/// `bool.ToString()`: `True` or `False`.
fn bool_field(name: &str, value: bool) -> Field {
    scalar(
        name,
        if value { "True" } else { "False" }.to_owned(),
        "Boolean",
        if value { 1.0 } else { 0.0 },
    )
}
/// An enum's field: the member's name, the enum's type name; its number for a graph.
fn enum_field(name: &str, member: &str, type_name: &'static str, number: u8) -> Field {
    scalar(name, member.to_owned(), type_name, f64::from(number))
}

/// A dynamic byte array: its `_len` field, then the array - as the generated class declares
/// them - the four text fields as ASCII.
/// `// C#: ExtLibs/DroneCAN/out/include/uavcan.protocol.GetNodeInfo_res.cs:33`
#[allow(clippy::cast_possible_truncation)] // the C#'s uint8_t length
fn bytes_fields(name: &str, bytes: &[u8]) -> Vec<Field> {
    vec![
        u8_field(&format!("{name}_len"), bytes.len() as u8),
        bytes_field(name, bytes),
    ]
}

/// A byte array without a length field: a fixed one, or the payload row.
fn bytes_field(name: &str, bytes: &[u8]) -> Field {
    if TEXT_FIELDS.contains(&name) {
        field(name, FieldValue::Text(mp_dronecan::dsdl::ascii(bytes)))
    } else {
        field(
            name,
            FieldValue::Numbers {
                values: bytes.iter().map(|b| f64::from(*b)).collect(),
                type_name: "Byte[]",
            },
        )
    }
}

/// `uavcan_protocol_param_Value`: the union's tag, then the `unions` class with every member.
/// `// C#: ExtLibs/DroneCAN/out/include/uavcan.protocol.param.Value.cs:27-49`
fn value_fields(name: &str, value: &Value) -> Field {
    let (member, number) = match value {
        Value::Empty => ("UAVCAN_PROTOCOL_PARAM_VALUE_TYPE_EMPTY", 0),
        Value::Integer(_) => ("UAVCAN_PROTOCOL_PARAM_VALUE_TYPE_INTEGER_VALUE", 1),
        Value::Real(_) => ("UAVCAN_PROTOCOL_PARAM_VALUE_TYPE_REAL_VALUE", 2),
        Value::Boolean(_) => ("UAVCAN_PROTOCOL_PARAM_VALUE_TYPE_BOOLEAN_VALUE", 3),
        Value::String(_) => ("UAVCAN_PROTOCOL_PARAM_VALUE_TYPE_STRING_VALUE", 4),
    };
    let integer = if let Value::Integer(v) = value { *v } else { 0 };
    let real = if let Value::Real(v) = value { *v } else { 0.0 };
    let boolean = if let Value::Boolean(v) = value { *v } else { 0 };
    let string: &[u8] = if let Value::String(v) = value { v } else { &[] };
    let mut union = vec![
        field("empty", FieldValue::Struct(Vec::new())),
        i64_field("integer_value", integer),
        f32_field("real_value", real),
        u8_field("boolean_value", boolean),
    ];
    union.extend(bytes_fields("string_value", string));
    field(
        name,
        FieldValue::Struct(vec![
            enum_field(
                "uavcan_protocol_param_Value_type",
                member,
                "uavcan_protocol_param_Value_type_t",
                number,
            ),
            field("union", FieldValue::Struct(union)),
        ]),
    )
}

/// `uavcan_protocol_param_NumericValue`, likewise.
/// `// C#: ExtLibs/DroneCAN/out/include/uavcan.protocol.param.NumericValue.cs:27-43`
fn numeric_fields(name: &str, value: &NumericValue) -> Field {
    let (member, number) = match value {
        NumericValue::Empty => ("UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_TYPE_EMPTY", 0),
        NumericValue::Integer(_) => ("UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_TYPE_INTEGER_VALUE", 1),
        NumericValue::Real(_) => ("UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_TYPE_REAL_VALUE", 2),
    };
    let integer = if let NumericValue::Integer(v) = value {
        *v
    } else {
        0
    };
    let real = if let NumericValue::Real(v) = value {
        *v
    } else {
        0.0
    };
    field(
        name,
        FieldValue::Struct(vec![
            enum_field(
                "uavcan_protocol_param_NumericValue_type",
                member,
                "uavcan_protocol_param_NumericValue_type_t",
                number,
            ),
            field(
                "union",
                FieldValue::Struct(vec![
                    field("empty", FieldValue::Struct(Vec::new())),
                    i64_field("integer_value", integer),
                    f32_field("real_value", real),
                ]),
            ),
        ]),
    )
}

/// `uavcan_protocol_NodeStatus`'s fields.
fn node_status_fields(status: &mp_dronecan::dsdl::NodeStatus) -> Vec<Field> {
    vec![
        u32_field("uptime_sec", status.uptime_sec),
        u8_field("health", status.health),
        u8_field("mode", status.mode),
        u8_field("sub_mode", status.sub_mode),
        u16_field(
            "vendor_specific_status_code",
            status.vendor_specific_status_code,
        ),
    ]
}

/// A message's fields, in the generated class's order: the 27 types decoded here, and one
/// `payload` row of the bytes for any other (a divergence: the C# reflects over every type).
/// `// C#: Controls/DroneCANInspector.cs:112-116`
#[must_use]
pub fn fields(message: &Message) -> Vec<Field> {
    match message {
        Message::NodeStatus(status) => node_status_fields(status),
        Message::GetNodeInfoReq => Vec::new(),
        Message::GetNodeInfoRes(info) => {
            let mut hardware = vec![
                u8_field("major", info.hardware_version.major),
                u8_field("minor", info.hardware_version.minor),
                bytes_field("unique_id", &info.hardware_version.unique_id),
            ];
            hardware.extend(bytes_fields(
                "certificate_of_authenticity",
                &info.hardware_version.certificate_of_authenticity,
            ));
            let mut out = vec![
                field(
                    "status",
                    FieldValue::Struct(node_status_fields(&info.status)),
                ),
                field(
                    "software_version",
                    FieldValue::Struct(vec![
                        u8_field("major", info.software_version.major),
                        u8_field("minor", info.software_version.minor),
                        u8_field(
                            "optional_field_flags",
                            info.software_version.optional_field_flags,
                        ),
                        u32_field("vcs_commit", info.software_version.vcs_commit),
                        u64_field("image_crc", info.software_version.image_crc),
                    ]),
                ),
                field("hardware_version", FieldValue::Struct(hardware)),
            ];
            out.extend(bytes_fields("name", &info.name));
            out
        }
        Message::RestartNodeReq { magic_number } => vec![u64_field("magic_number", *magic_number)],
        Message::RestartNodeRes { ok } => vec![bool_field("ok", *ok)],
        Message::GetSetReq(req) => {
            let mut out = vec![
                u16_field("index", req.index),
                value_fields("value", &req.value),
            ];
            out.extend(bytes_fields("name", &req.name));
            out
        }
        Message::GetSetRes(res) => {
            let mut out = vec![
                value_fields("value", &res.value),
                value_fields("default_value", &res.default_value),
                numeric_fields("max_value", &res.max_value),
                numeric_fields("min_value", &res.min_value),
            ];
            out.extend(bytes_fields("name", &res.name));
            out
        }
        Message::ExecuteOpcodeReq { opcode, argument } => {
            vec![
                u8_field("opcode", *opcode),
                i64_field("argument", *argument),
            ]
        }
        Message::ExecuteOpcodeRes { argument, ok } => {
            vec![i64_field("argument", *argument), bool_field("ok", *ok)]
        }
        Message::FileReadReq { offset, path } => {
            let mut inner = Vec::new();
            inner.extend(bytes_fields("path", path));
            vec![
                u64_field("offset", *offset),
                field("path", FieldValue::Struct(inner)),
            ]
        }
        Message::FileReadRes { error, data } => {
            let mut out = vec![field(
                "error",
                FieldValue::Struct(vec![i16_field("value", *error)]),
            )];
            out.extend(bytes_fields("data", data));
            out
        }
        Message::BeginFirmwareUpdateReq {
            source_node_id,
            path,
        } => {
            let mut inner = Vec::new();
            inner.extend(bytes_fields("path", path));
            vec![
                u8_field("source_node_id", *source_node_id),
                field("image_file_remote_path", FieldValue::Struct(inner)),
            ]
        }
        Message::BeginFirmwareUpdateRes {
            error,
            optional_error_message,
        } => {
            let mut out = vec![u8_field("error", *error)];
            out.extend(bytes_fields(
                "optional_error_message",
                optional_error_message,
            ));
            out
        }
        Message::Allocation(allocation) => {
            let mut out = vec![
                u8_field("node_id", allocation.node_id),
                bool_field(
                    "first_part_of_unique_id",
                    allocation.first_part_of_unique_id,
                ),
            ];
            out.extend(bytes_fields("unique_id", &allocation.unique_id));
            out
        }
        Message::LogMessage(log) => {
            let mut out = vec![field(
                "level",
                FieldValue::Struct(vec![u8_field("value", log.level)]),
            )];
            out.extend(bytes_fields("source", &log.source));
            out.extend(bytes_fields("text", &log.text));
            out
        }
        Message::Stats(s) => vec![
            u32_field("tx_frames", s.tx_frames),
            u16_field("tx_errors", s.tx_errors),
            u32_field("rx_frames", s.rx_frames),
            u16_field("rx_error_oom", s.rx_error_oom),
            u16_field("rx_error_internal", s.rx_error_internal),
            u16_field("rx_error_missed_start", s.rx_error_missed_start),
            u16_field("rx_error_wrong_toggle", s.rx_error_wrong_toggle),
            u16_field("rx_error_short_frame", s.rx_error_short_frame),
            u16_field("rx_error_bad_crc", s.rx_error_bad_crc),
            u16_field("rx_ignored_wrong_address", s.rx_ignored_wrong_address),
            u16_field("rx_ignored_not_wanted", s.rx_ignored_not_wanted),
            u16_field("rx_ignored_unexpected_tid", s.rx_ignored_unexpected_tid),
        ],
        Message::CanStats(s) => vec![
            u8_field("interface", s.interface),
            u32_field("tx_requests", s.tx_requests),
            u16_field("tx_rejected", s.tx_rejected),
            u16_field("tx_overflow", s.tx_overflow),
            u16_field("tx_success", s.tx_success),
            u16_field("tx_timedout", s.tx_timedout),
            u16_field("tx_abort", s.tx_abort),
            u32_field("rx_received", s.rx_received),
            u16_field("rx_overflow", s.rx_overflow),
            u16_field("rx_errors", s.rx_errors),
            u16_field("busoff_errors", s.busoff_errors),
        ],
        Message::RtcmStream { protocol_id, data } => {
            let mut out = vec![u8_field("protocol_id", *protocol_id)];
            out.extend(bytes_fields("data", data));
            out
        }
        Message::MovingBaselineData { data } => bytes_fields("data", data),
        Message::TunnelTargetted(t) => {
            let mut out = vec![
                u8_field("protocol", t.protocol),
                u8_field("target_node", t.target_node),
                i8_field("serial_id", t.serial_id),
                u8_field("options", t.options),
                u32_field("baudrate", t.baudrate),
            ];
            out.extend(bytes_fields("buffer", &t.buffer));
            out
        }
        Message::Other { payload, .. } => vec![bytes_field(PAYLOAD, payload)],
    }
}

/// `SizeofEntireMsg`: the transfer's payload bytes - the bytes a type nothing here decodes
/// came with, or the message encoded again as it went on the bus.
#[must_use]
pub fn bytes_of(message: &Message) -> usize {
    match message {
        Message::Other { payload, .. } => payload.len(),
        _ => message.encode(false).len(),
    }
}

/// `String.Format("{0,-32} {1,20} {2,-20}", name, value, type)`: a field row's text.
/// `// C#: Controls/DroneCANInspector.cs:197-198`
#[must_use]
pub fn field_text(name: &str, value: &str, type_name: &str) -> String {
    mavi::field_text(name, value, type_name)
}

/// A field's `value` and type as the row shows them: an array of numbers joined by commas
/// (`Aggregate((a, b) => a + "," + b)`), an empty one `null` - which `String.Format` writes as
/// nothing - a text field its text.
/// `// C#: Controls/DroneCANInspector.cs:134-176`
#[must_use]
pub fn tag_of(value: &FieldValue) -> Option<(String, String)> {
    match value {
        FieldValue::Scalar {
            text, type_name, ..
        } => Some((text.clone(), (*type_name).to_owned())),
        FieldValue::Text(text) => Some((text.clone(), "Byte[]".to_owned())),
        FieldValue::Numbers { values, type_name } => Some((
            values
                .iter()
                .map(|v| format_f64(*v, "0.##########"))
                .collect::<Vec<_>>()
                .join(","),
            (*type_name).to_owned(),
        )),
        FieldValue::Struct(_) => None,
    }
}

/// `PopulateMSG`: each field's node under `parent` - made when missing - and its text.
/// `// C#: Controls/DroneCANInspector.cs:119-200`
fn populate(fields: &[Field], parent: &mut Node, added: &mut bool) {
    for item in fields {
        let Some(node) = mavi::child(&mut parent.nodes, &item.name, || item.name.clone(), added)
        else {
            continue;
        };
        match &item.value {
            FieldValue::Struct(inner) => {
                node.text = item.name.clone();
                node.tag = None;
                populate(inner, node, added);
            }
            other => {
                if let Some((value, type_name)) = tag_of(other) {
                    node.text = field_text(&item.name, &value, &type_name);
                    node.tag = Some((value, type_name));
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The Inspector.
// ---------------------------------------------------------------------------------------------

/// The message header: `TypeName (r.r Hz, #id) ~bytesBps`.
/// `// C#: Controls/DroneCANInspector.cs:100-102`
#[must_use]
pub fn header_text(name: &str, msgid: u16, rate: f64, bps: f64) -> String {
    format!(
        "{name} ({}, #{msgid}) ~{}",
        format_f64(rate, "0.0 Hz"),
        format_f64(bps, "0Bps")
    )
}

/// The node header once seen: `ID n - name ~bytesBps`.
/// `// C#: Controls/DroneCANInspector.cs:78-80`
#[must_use]
pub fn node_text(node: u8, name: &str, bps: f64) -> String {
    format!("ID {node} - {name} ~{}", format_f64(bps, "0Bps"))
}

/// What Graph It reads from `selectedmsgid`: the node, the data type id and the path of field
/// names under the message, with `name[i]` for an array's element.
/// `// C#: Controls/DroneCANInspector.cs:351-355`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Target {
    pub node: u8,
    pub msgid: u16,
    pub path: Vec<String>,
}

/// `selectedmsgid.Split('/')`: `int.Parse` of the first two, the rest the path. A path of
/// fewer than three parts - a node's or a message's - is the C#'s `IndexOutOfRange` on
/// `msgpath[1]` or an empty field path, so nothing is graphed.
pub fn target(path: &str) -> Result<Option<Target>, String> {
    let parts: Vec<&str> = path.split('/').collect();
    let [node, msgid, rest @ ..] = parts.as_slice() else {
        return Ok(None);
    };
    if rest.is_empty() {
        return Ok(None);
    }
    let node = node
        .trim()
        .parse::<u8>()
        .map_err(|_| "Input string was not in a correct format.".to_owned())?;
    let msgid = msgid
        .trim()
        .parse::<u16>()
        .map_err(|_| "Input string was not in a correct format.".to_owned())?;
    Ok(Some(Target {
        node,
        msgid,
        path: rest.iter().map(|part| (*part).to_owned()).collect(),
    }))
}

/// `new XDate(DateTime.Now)`: days since 1899-12-30, to the millisecond.
#[must_use]
#[allow(clippy::cast_precision_loss)] // milliseconds since 1970: well inside f64
pub fn xdate_now() -> f64 {
    let ms = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_millis());
    // 1970-01-01 is day 25569 of the XDate calendar.
    ms as f64 / 86_400_000.0 + 25569.0
}

/// A graph of a path: its curves, fed as each message of its node and type comes.
#[derive(Debug, Clone, PartialEq)]
pub struct Graph {
    pub target: Target,
    /// `msgidfield`: the path's last part, the first curve's label.
    pub field: String,
    /// The curves as they fill.
    data: Curves,
    /// The curves as the timer last drew them.
    pub shown: Curves,
    next_draw: Instant,
}

impl Graph {
    fn new(target: Target, history: usize, now: Instant) -> Self {
        let field = target.path.last().cloned().unwrap_or_default();
        let curves = Curves::new(field.clone(), history);
        Self {
            target,
            field,
            data: curves.clone(),
            shown: curves,
            next_draw: now + GRAPH_EVERY,
        }
    }

    /// The path walked over the message's fields: `data = field.GetValue(data)`, or the
    /// element at `[index]`.
    /// `// C#: Controls/DroneCANInspector.cs:370-387`
    fn walk<'a>(fields: &'a [Field], path: &[String]) -> Option<&'a FieldValue> {
        let mut current: Option<&FieldValue> = None;
        let mut level: &[Field] = fields;
        for part in path {
            // `name[i]`: an array element's node, which no decoded type has (the module's
            // notes), so the C#'s `IList` index finds nothing here.
            if part.ends_with(']') {
                return None;
            }
            let found = level.iter().find(|f| f.name == *part)?;
            current = Some(&found.value);
            if let FieldValue::Struct(inner) = &found.value {
                level = inner;
            }
        }
        current
    }

    /// A message of the graphed node and type: `AddToGraph` at the wall clock - an array a point
    /// on each element's curve, a number a point on the first, a nested type a curve a field.
    /// `// C#: Controls/DroneCANInspector.cs:365-422, 448-477`
    fn add(&mut self, received: &Received) {
        if received.frame.source_node() != self.target.node
            || received.message.info().id != self.target.msgid
        {
            return;
        }
        let fields = fields(&received.message);
        let Some(value) = Self::walk(&fields, &self.target.path) else {
            return;
        };
        let x = xdate_now();
        match value {
            FieldValue::Scalar { number, .. } => {
                if let Some(y) = number {
                    self.data.push(0, &self.field, (x, *y));
                }
            }
            FieldValue::Numbers { values, .. } => {
                for (a, y) in values.iter().enumerate() {
                    self.data.push(a, &self.field, (x, *y));
                }
            }
            FieldValue::Struct(inner) => {
                // The dictionary of the type's fields: a curve each, labelled `field.name`.
                let field = self.field.clone();
                for (a, item) in inner.iter().enumerate() {
                    let label = format!("{field}.{}", item.name);
                    self.data.ensure(a, &label);
                    match &item.value {
                        FieldValue::Scalar {
                            number: Some(y), ..
                        } => self.data.push(a, &label, (x, *y)),
                        FieldValue::Numbers { values, .. } => {
                            for (b, y) in values.iter().enumerate() {
                                self.data.push(a + b, &label, (x, *y));
                            }
                        }
                        _ => {}
                    }
                }
            }
            FieldValue::Text(_) => {}
        }
    }

    /// Every 100 ms the curves taken as the timer's `AxisChange` and `Invalidate` draw them.
    fn tick(&mut self, now: Instant) {
        if now >= self.next_draw {
            self.next_draw = now + GRAPH_EVERY;
            self.shown = self.data.clone();
        }
    }

    /// Every point's ranges over the curves shown; `0` to `1` for none.
    #[must_use]
    pub fn ranges(&self) -> ((f64, f64), (f64, f64)) {
        mavi::ranges_of(&self.shown)
    }
}

/// The Subscriber: `DroneCANSubscriber(can, selectedmsgid)`.
/// `// C#: Controls/DroneCANSubscriber.cs:11-146`
#[derive(Debug, Clone, PartialEq)]
pub struct Subscriber {
    /// `targettype`: `cmb_msg.Text`.
    pub target_type: String,
    /// `msgtypes`: every type name heard, in the order heard.
    types: Vec<String>,
    /// `cmb_msg.Items`, filled from `msgtypes` once a second.
    pub items: Vec<String>,
    /// Whether the combo's list is down.
    pub open: bool,
    /// `num_lines.Value`.
    pub lines: i32,
    /// `txt_packet.Lines`.
    pub text: Vec<String>,
    next_fill: Instant,
}

impl Subscriber {
    fn new(now: Instant) -> Self {
        Self {
            target_type: String::new(),
            types: Vec::new(),
            items: Vec::new(),
            open: false,
            lines: LINES,
            text: Vec::new(),
            next_fill: now + TYPES_EVERY,
        }
    }

    /// `Can_MessageReceived`: a message of the chosen type's JSON appended, the box kept to the
    /// last `num_lines` as the C# keeps it - `Skip(Lines.Length - num_lines)` of the new lines,
    /// which is more than `num_lines` when several came; the type noted for the combo.
    /// `// C#: Controls/DroneCANSubscriber.cs:97-131`
    fn heard(&mut self, received: &Received) {
        let name = received.message.info().name;
        if name == self.target_type {
            let old = self.text.len();
            let mut lines = self.text.clone();
            lines.extend(
                json(&fields(&received.message), 0)
                    .split(['\n', '\r'])
                    .filter(|line| !line.is_empty())
                    .map(str::to_owned),
            );
            let keep = usize::try_from(self.lines).unwrap_or(0);
            self.text = if lines.len() > keep {
                lines.into_iter().skip(old.saturating_sub(keep)).collect()
            } else {
                lines
            };
        }
        if !self.types.iter().any(|t| t == name) {
            self.types.push(name.to_owned());
        }
    }

    /// The combo's timer: `msgtypes` into `cmb_msg.Items`.
    fn tick(&mut self, now: Instant) {
        if now >= self.next_fill {
            self.next_fill = now + TYPES_EVERY;
            for name in &self.types {
                if !self.items.contains(name) {
                    self.items.push(name.clone());
                }
            }
        }
    }

    /// `cmb_msg_SelectedIndexChanged`: `targettype = cmb_msg.Text`.
    pub fn choose(&mut self, index: usize) {
        if let Some(name) = self.items.get(index) {
            self.target_type = name.clone();
        }
        self.open = false;
    }

    /// `num_lines` stepped by its arrows: 0 to 10,000.
    pub fn step_lines(&mut self, up: bool) {
        self.lines = (self.lines + if up { 1 } else { -1 }).clamp(0, LINES_MAX);
    }
}

/// `msg.ToJSON(Formatting.Indented)`: the generated class's public fields as Json.NET writes
/// them - a byte array as its Base64 text, other arrays as arrays, an enum as its number, a
/// nested type as an object.
#[must_use]
pub fn json(fields: &[Field], depth: usize) -> String {
    let pad = "  ".repeat(depth + 1);
    let close = "  ".repeat(depth);
    if fields.is_empty() {
        return "{}".to_owned();
    }
    let mut out = String::from("{\n");
    let last = fields.len() - 1;
    for (i, item) in fields.iter().enumerate() {
        let value = match &item.value {
            FieldValue::Scalar {
                text,
                type_name,
                number,
            } => match *type_name {
                "Boolean" => text.to_ascii_lowercase(),
                name if name.ends_with("_t") => {
                    number.map_or_else(|| "0".to_owned(), |n| format_f64(n, "0"))
                }
                _ => text.clone(),
            },
            FieldValue::Text(text) => format!("\"{}\"", base64(text.as_bytes())),
            FieldValue::Numbers { values, type_name } => {
                if *type_name == "Byte[]" {
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    let bytes: Vec<u8> = values.iter().map(|v| *v as u8).collect();
                    format!("\"{}\"", base64(&bytes))
                } else {
                    format!(
                        "[\n{}\n{pad}]",
                        values
                            .iter()
                            .map(|v| format!("{pad}  {}", format_f64(*v, "0.##########")))
                            .collect::<Vec<_>>()
                            .join(",\n")
                    )
                }
            }
            FieldValue::Struct(inner) => json(inner, depth + 1),
        };
        out.push_str(&format!(
            "{pad}\"{}\": {value}{}\n",
            item.name,
            if i == last { "" } else { "," }
        ));
    }
    out.push_str(&close);
    out.push('}');
    out
}

/// Base64, as `Convert.ToBase64String` writes a byte array.
#[must_use]
pub fn base64(bytes: &[u8]) -> String {
    use base64::Engine as _;
    base64::engine::general_purpose::STANDARD.encode(bytes)
}

/// The Inspector's form.
#[derive(Debug)]
pub struct Inspector {
    /// `pktinspect`, keyed by the source node and the data type id.
    pktinspect: PacketInspector<Received>,
    /// `treeView1.Nodes`.
    pub tree: Vec<Node>,
    /// The nodes expanded, by their `Name`s.
    expanded: BTreeSet<Vec<String>>,
    /// `SelectedNode`, by its `Name`s.
    selected: Option<Vec<String>>,
    /// `selectedmsgid`.
    pub selected_path: Option<String>,
    /// `but_graphit.Enabled` and `but_subscribe.Enabled`.
    pub graph_it: bool,
    pub subscribe: bool,
    /// `history`.
    pub history: i32,
    /// When `timer1` next ticks, and how many times it has.
    next_update: Instant,
    pub updates: usize,
    /// Graph It's question, while it is up; the answer kept.
    pub asking: Option<InputBox>,
    answered: Option<InputBox>,
    /// The graphs open.
    pub graphs: Vec<Graph>,
    /// The Subscriber, while it is open.
    pub subscriber: Option<Subscriber>,
    /// How many messages the form has heard.
    pub heard: usize,
}

impl Inspector {
    /// `new DroneCANInspector(can)`: the handler subscribed and the timer started.
    /// `// C#: Controls/DroneCANInspector.cs:31-49`
    #[must_use]
    pub fn new(now: Instant) -> Self {
        Self {
            pktinspect: PacketInspector::default(),
            tree: Vec::new(),
            expanded: BTreeSet::new(),
            selected: None,
            selected_path: None,
            graph_it: false,
            subscribe: false,
            history: HISTORY,
            next_update: now + UPDATE_EVERY,
            updates: 0,
            asking: None,
            answered: None,
            graphs: Vec::new(),
            subscriber: None,
            heard: 0,
        }
    }

    /// `Can_MessageReceived`: into `pktinspect`, and to each graph and the Subscriber.
    /// `// C#: Controls/DroneCANInspector.cs:51-54`
    pub fn heard(&mut self, received: &Received, now: Instant) {
        self.heard += 1;
        self.pktinspect.add(
            received.frame.source_node(),
            0,
            u32::from(received.message.info().id),
            received.clone(),
            bytes_of(&received.message),
            now,
        );
        for graph in &mut self.graphs {
            graph.add(received);
        }
        if let Some(subscriber) = self.subscriber.as_mut() {
            subscriber.heard(received);
        }
    }

    /// `timer1`'s `Update`, the graphs' timers and the Subscriber's.
    pub fn tick(&mut self, now: Instant, node_name: &dyn Fn(u8) -> String) {
        if now >= self.next_update {
            self.next_update = now + UPDATE_EVERY;
            self.update(now, node_name);
        }
        for graph in &mut self.graphs {
            graph.tick(now);
        }
        if let Some(subscriber) = self.subscriber.as_mut() {
            subscriber.tick(now);
        }
    }

    /// `Update`: the tree brought up to the newest message of each kind.
    /// `// C#: Controls/DroneCANInspector.cs:56-117`
    pub fn update(&mut self, now: Instant, node_name: &dyn Fn(u8) -> String) {
        self.updates += 1;
        let mut added = false;
        for received in self.pktinspect.packet_messages() {
            let node = received.frame.source_node();
            let info = received.message.info();
            let msgid = info.id;
            let bps_node = self.pktinspect.seen_bps(node, 0, 0, now);
            let Some(id_node) = mavi::child(
                &mut self.tree,
                &node.to_string(),
                || format!("ID {node}"),
                &mut added,
            ) else {
                continue;
            };
            if !added || id_node.text != format!("ID {node}") {
                id_node.text = node_text(node, &node_name(node), bps_node);
            }
            let Some(msg_node) = mavi::child(
                &mut id_node.nodes,
                &msgid.to_string(),
                || msgid.to_string(),
                &mut added,
            ) else {
                continue;
            };
            let rate = self.pktinspect.seen_rate(node, 0, u32::from(msgid), now);
            let bps = self.pktinspect.seen_bps(node, 0, u32::from(msgid), now);
            let header = header_text(info.name, msgid, rate, bps);
            if msg_node.text != header {
                msg_node.text = header;
            }
            populate(&fields(&received.message), msg_node, &mut added);
        }
        if added {
            mavi::sort(&mut self.tree);
        }
    }

    /// A node's plus or minus box.
    pub fn toggle(&mut self, key: &[String]) {
        if !self.expanded.remove(key) {
            self.expanded.insert(key.to_vec());
        }
    }

    /// The rows showing.
    #[must_use]
    pub fn rows(&self) -> Vec<Row> {
        let mut out = Vec::new();
        mavi::rows_of(&self.tree, &[], &self.expanded, &mut out);
        out
    }

    /// `treeView1_AfterSelect`: a node below the root selected makes `selectedmsgid` its names
    /// from the root joined by `/`, and enables Graph It and Subscribe; a root node does not.
    /// `// C#: Controls/DroneCANInspector.cs:326-346`
    pub fn select(&mut self, key: &[String]) {
        self.selected = Some(key.to_vec());
        if key.len() < 2 {
            return;
        }
        self.selected_path = Some(key.join("/"));
        self.graph_it = true;
        self.subscribe = true;
    }

    #[must_use]
    pub fn selected(&self) -> Option<&[String]> {
        self.selected.as_deref()
    }

    /// `but_graphit_Click`'s question.
    pub fn press_graph_it(&mut self) {
        if self.graph_it && self.asking.is_none() {
            self.asking = Some(InputBox::new(
                POINTS_TITLE,
                POINTS_PROMPT,
                &self.history.to_string(),
            ));
        }
    }

    /// A key in the question's box.
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

    /// The question closed, and the rest of `but_graphit_Click`: OK's answer as `int.Parse`
    /// reads it, the path read, a graph opened and Graph It disabled.
    /// `// C#: Controls/DroneCANInspector.cs:347-446`
    pub fn prompt_done(&mut self, ok: bool, now: Instant) -> Result<(), String> {
        let Some(input) = self.asking.take() else {
            return Ok(());
        };
        if ok {
            let answer = input.field.value().trim().to_owned();
            self.answered = Some(input);
            self.history = answer
                .parse::<i32>()
                .map_err(|_| "Input string was not in a correct format.".to_owned())?;
        }
        let Some(path) = self.selected_path.clone() else {
            return Ok(());
        };
        let Some(target) = target(&path)? else {
            return Ok(());
        };
        let history = usize::try_from(self.history)
            .map_err(|_| "Non-negative number required.".to_owned())?;
        self.graphs.push(Graph::new(target, history, now));
        self.graph_it = false;
        Ok(())
    }

    /// The question's box closed with OK, for its answer to be kept.
    pub fn take_answered(&mut self) -> Option<InputBox> {
        self.answered.take()
    }

    /// A graph's close box: `form.Closing`.
    pub fn close_graph(&mut self, index: usize) {
        if index < self.graphs.len() {
            self.graphs.remove(index);
        }
    }

    /// `but_subscribe_Click`: the Subscriber shown for the selection; nothing with none.
    /// `// C#: Controls/DroneCANInspector.cs:479-484`
    pub fn press_subscribe(&mut self, now: Instant) {
        if self.selected_path.is_none() {
            return;
        }
        self.subscriber = Some(Subscriber::new(now));
    }

    /// The Subscriber closed.
    pub fn close_subscriber(&mut self) {
        self.subscriber = None;
    }

    /// Types into the question's box, as a test does.
    #[cfg(test)]
    fn type_answer(&mut self, text: &str) {
        if let Some(input) = self.asking.as_mut() {
            input.field.set(text);
        }
    }
}

/// The facts, under `config.dronecan.inspector.`.
pub fn record_facts(inspector: Option<&Inspector>, opened: usize) {
    use crate::facts::record;
    record("config.dronecan.inspector.window", inspector.is_some());
    record("config.dronecan.inspector.opened", opened);
    let Some(inspector) = inspector else {
        return;
    };
    record("config.dronecan.inspector.updates", inspector.updates);
    record("config.dronecan.inspector.heard", inspector.heard);
    record("config.dronecan.inspector.graphit", inspector.graph_it);
    record("config.dronecan.inspector.subscribe", inspector.subscribe);
    record("config.dronecan.inspector.history", inspector.history);
    record(
        "config.dronecan.inspector.prompt",
        inspector
            .asking
            .as_ref()
            .map_or("none", |input| input.prompt),
    );
    record(
        "config.dronecan.inspector.selected",
        inspector.selected_path.clone().unwrap_or_default(),
    );
    record("config.dronecan.inspector.graphs", inspector.graphs.len());
    let rows = inspector.rows();
    record("config.dronecan.inspector.rows", rows.len());
    for (index, row) in rows.iter().enumerate() {
        record(
            format!("config.dronecan.inspector.row.{index}"),
            row.text.clone(),
        );
        if let Some((value, type_name)) = row.tag.as_ref() {
            let key = row.key.join(".");
            record(
                format!("config.dronecan.inspector.value.{key}"),
                value.clone(),
            );
            record(
                format!("config.dronecan.inspector.type.{key}"),
                type_name.clone(),
            );
        }
    }
    for (index, graph) in inspector.graphs.iter().enumerate() {
        record(
            format!("config.dronecan.inspector.graph.{index}.curves"),
            graph
                .shown
                .list
                .iter()
                .map(|curve| curve.label.clone())
                .collect::<Vec<_>>()
                .join(","),
        );
        record(
            format!("config.dronecan.inspector.graph.{index}.points"),
            graph
                .shown
                .list
                .iter()
                .map(|curve| curve.points.len())
                .sum::<usize>(),
        );
    }
    let subscriber = inspector.subscriber.as_ref();
    record("config.dronecan.subscriber.window", subscriber.is_some());
    if let Some(subscriber) = subscriber {
        record(
            "config.dronecan.subscriber.type",
            subscriber.target_type.clone(),
        );
        record(
            "config.dronecan.subscriber.items",
            subscriber.items.join(","),
        );
        record("config.dronecan.subscriber.lines", subscriber.lines);
        record(
            "config.dronecan.subscriber.text.lines",
            subscriber.text.len(),
        );
        record(
            "config.dronecan.subscriber.text",
            subscriber.text.join("\n"),
        );
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

fn access(this: &mut MissionPlanner) -> Option<&mut Inspector> {
    this.extra.dronecan.inspector.as_mut()
}

/// A probe id for a node: its `Name`s joined by `-`.
#[must_use]
pub fn row_id(prefix: &str, key: &[String]) -> String {
    format!("dronecan-inspector-{prefix}-{}", key.join("-"))
}

fn row_element(row: &Row, selected: bool, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let toggle_id = row_id("toggle", &row.key);
    let node_id = row_id("node", &row.key);
    let key = row.key.clone();
    let toggle_key = key.clone();
    let select_key = key;
    mavi::tree_row(
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
        cx.listener(move |this, _event, _window, cx| {
            if let Some(window) = access(this) {
                window.select(&select_key);
                cx.notify();
            }
        }),
    )
}

/// The form: Graph It, Subscribe, and the tree in its group.
/// `// C#: Controls/DroneCANInspector.cs:202-274`
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
    let mut tree = crate::probe::measured("dronecan-inspector-tree", div())
        .id("dronecan-inspector-tree")
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
    let prompt = prompt.clone();
    let client = div()
        .relative()
        .w(px(CLIENT.0))
        .h(px(CLIENT.1))
        .child(button(
            "dronecan-inspector-graphit",
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
        .child(button(
            "dronecan-inspector-subscribe",
            SUBSCRIBE,
            SUBSCRIBE_AT,
            inspector.subscribe,
            move |this, _window, _cx| {
                if let Some(inspector) = access(this) {
                    inspector.press_subscribe(Instant::now());
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
            "dronecan-inspector-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.extra.dronecan.close_inspector();
                cx.notify();
            }),
        ));
    let form = crate::probe::measured("dronecan-inspector", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    let over = div()
        .id("dronecan-inspector-backdrop")
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

/// The Subscriber's form: the combo, the line count and the box.
/// `// C#: Controls/DroneCANSubscriber.cs:30-85`
fn subscriber_form(
    subscriber: &Subscriber,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let size = window.viewport_size();
    let (cx_, cy, cw, ch) = SUBSCRIBER_COMBO_AT;
    let (lx, ly, lw, lh) = SUBSCRIBER_LINES_AT;
    let (tx, ty, tw, th) = SUBSCRIBER_TEXT_AT;
    let combo = crate::probe::measured("dronecan-subscriber-type", div())
        .id("dronecan-subscriber-type")
        .absolute()
        .left(px(cx_))
        .top(px(cy))
        .w(px(cw))
        .h(px(ch))
        .px_1()
        .flex()
        .items_center()
        .justify_between()
        .bg(rgb(theme::BG))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .child(subscriber.target_type.clone())
        .child("\u{25bc}")
        .on_click(cx.listener(|this, _event, _window, cx| {
            if let Some(subscriber) = access(this).and_then(|i| i.subscriber.as_mut()) {
                subscriber.open = !subscriber.open;
                cx.notify();
            }
        }));
    let mut list = div()
        .absolute()
        .left(px(cx_))
        .top(px(cy + ch))
        .w(px(cw))
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .text_color(rgb(theme::TEXT));
    if subscriber.open {
        for (index, name) in subscriber.items.iter().enumerate() {
            let id = format!("dronecan-subscriber-type-{index}");
            list = list.child(
                crate::probe::measured(id.clone(), div())
                    .id(SharedString::from(id))
                    .px_1()
                    .cursor_pointer()
                    .hover(|style| style.bg(rgb(theme::BORDER)))
                    .child(name.clone())
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        if let Some(subscriber) = access(this).and_then(|i| i.subscriber.as_mut()) {
                            subscriber.choose(index);
                            cx.notify();
                        }
                    })),
            );
        }
    }
    let lines = crate::probe::measured("dronecan-subscriber-lines", div())
        .absolute()
        .left(px(lx))
        .top(px(ly))
        .w(px(lw))
        .h(px(lh))
        .flex()
        .items_center()
        .bg(rgb(theme::BG))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .child(div().flex_1().px_1().child(subscriber.lines.to_string()))
        .child(div().flex().flex_col().children(
            [(true, "\u{25b2}", "up"), (false, "\u{25bc}", "down")].map(|(up, glyph, word)| {
                let id = format!("dronecan-subscriber-lines-{word}");
                crate::probe::measured(id.clone(), div())
                    .id(SharedString::from(id))
                    .px(px(2.0))
                    .text_size(px(7.0))
                    .cursor_pointer()
                    .child(glyph)
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        if let Some(subscriber) = access(this).and_then(|i| i.subscriber.as_mut()) {
                            subscriber.step_lines(up);
                            cx.notify();
                        }
                    }))
            }),
        ));
    let text = crate::probe::measured("dronecan-subscriber-text", div())
        .id("dronecan-subscriber-text")
        .absolute()
        .left(px(tx))
        .top(px(ty))
        .w(px(tw))
        .h(px(th))
        .overflow_scroll()
        .p_1()
        .bg(rgb(theme::BG))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .children(
            subscriber
                .text
                .iter()
                .map(|line| div().whitespace_nowrap().child(line.clone())),
        );
    let client = div()
        .relative()
        .w(px(SUBSCRIBER_SIZE.0))
        .h(px(SUBSCRIBER_SIZE.1))
        .child(text)
        .child(combo)
        .child(lines)
        .child(list);
    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child("UAVCANSubscriber"),
        )
        .child(crate::ui::action(
            "dronecan-subscriber-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                if let Some(inspector) = access(this) {
                    inspector.close_subscriber();
                }
                cx.notify();
            }),
        ));
    let form = crate::probe::measured("dronecan-subscriber", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    // Beside the Inspector, from the window's corner.
    let x = (f32::from(size.width) - SUBSCRIBER_SIZE.0 - 20.0).max(0.0);
    let y = 60.0_f32.min(f32::from(size.height));
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(x), px(y)))
            .child(div().id("dronecan-subscriber-form").occlude().child(form)),
    )
    .with_priority(2)
    .into_any_element()
}

/// The form, its graphs, its Subscriber and Graph It's question, while the form is open.
pub fn overlay(
    inspector: Option<&Inspector>,
    prompt: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let inspector = inspector?;
    let mut layers = div().child(form(inspector, prompt, window, cx));
    for (index, graph) in inspector.graphs.iter().enumerate() {
        layers = layers.child(mavi::graph_pane(
            "dronecan-inspector-graph",
            index,
            Y_TITLE,
            &graph.shown,
            graph.ranges(),
            move |this| {
                if let Some(inspector) = access(this) {
                    inspector.close_graph(index);
                }
            },
            window,
            cx,
        ));
    }
    if let Some(subscriber) = inspector.subscriber.as_ref() {
        layers = layers.child(subscriber_form(subscriber, window, cx));
    }
    if let Some(input) = inspector.asking.as_ref() {
        layers = layers.child(input_box(
            "dronecan-inspector-points-box",
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

fn finish_prompt(this: &mut MissionPlanner, ok: bool) {
    let now = Instant::now();
    let Some(inspector) = access(this) else {
        return;
    };
    let result = inspector.prompt_done(ok, now);
    after_prompt(this, result);
}

/// After the question: OK's answer kept as `InputBox` keeps it, and what the C# throws on the
/// status line.
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
    use mp_dronecan::dsdl::NodeStatus;
    use mp_dronecan::frame::Frame;

    fn status(node: u8, uptime: u32) -> Received {
        let mut frame = Frame::from_id(0, true, false);
        frame.set_source_node(node);
        frame.set_msg_type_id(341);
        Received {
            frame,
            message: Message::NodeStatus(NodeStatus {
                uptime_sec: uptime,
                health: 0,
                mode: 0,
                sub_mode: 0,
                vendor_specific_status_code: 7,
            }),
            transfer_id: 0,
        }
    }

    fn key(names: &[&str]) -> Vec<String> {
        names.iter().map(|name| (*name).to_owned()).collect()
    }

    /// `Update`: `ID n` when first seen, then the name and bytes; the message's header with
    /// its rate, id and bytes; the fields as `PopulateMSG` writes them, with the .NET type.
    /// `// C#: Controls/DroneCANInspector.cs:56-117, 197-198`
    #[test]
    fn the_tree_is_updated_as_the_csharp_updates_it() {
        let now = Instant::now();
        let mut inspector = Inspector::new(now);
        inspector.heard(&status(127, 5), now);
        inspector.update(now, &|_| "org.missionplanner".to_owned());
        assert_eq!(inspector.tree.len(), 1);
        assert_eq!(inspector.tree[0].name, "127");
        assert_eq!(inspector.tree[0].text, "ID 127");
        let message = &inspector.tree[0].nodes[0];
        assert_eq!(message.name, "341");
        assert!(
            message.text.starts_with("uavcan_protocol_NodeStatus ("),
            "{}",
            message.text
        );
        assert!(message.text.contains(", #341) ~"), "{}", message.text);
        // `treeView1.Sort()` reaches the fields: alphabetical, as the C# shows them.
        let names: Vec<&str> = message.nodes.iter().map(|n| n.name.as_str()).collect();
        assert_eq!(
            names,
            [
                "health",
                "mode",
                "sub_mode",
                "uptime_sec",
                "vendor_specific_status_code"
            ]
        );
        assert_eq!(
            message.nodes[3].tag,
            Some(("5".to_owned(), "UInt32".to_owned()))
        );
        assert_eq!(
            message.nodes[3].text,
            field_text("uptime_sec", "5", "UInt32")
        );
        // Seen again: the node's text carries the name and the bytes.
        inspector.heard(&status(127, 6), now + Duration::from_millis(100));
        inspector.update(now + Duration::from_millis(400), &|_| {
            "org.missionplanner".to_owned()
        });
        assert!(
            inspector.tree[0]
                .text
                .starts_with("ID 127 - org.missionplanner ~"),
            "{}",
            inspector.tree[0].text
        );
        assert_eq!(
            inspector.tree[0].nodes[0].nodes[3].tag,
            Some(("6".to_owned(), "UInt32".to_owned()))
        );
    }

    /// A nested type is a node of its fields; an array of bytes named `name` is its text;
    /// a type nothing decodes shows its payload.
    /// `// C#: Controls/DroneCANInspector.cs:134-196`
    #[test]
    fn nested_types_text_fields_and_unknown_types_are_laid_out() {
        let info = Message::GetNodeInfoRes(Box::new(mp_dronecan::dsdl::GetNodeInfoRes {
            status: NodeStatus::default(),
            software_version: mp_dronecan::dsdl::SoftwareVersion::default(),
            hardware_version: mp_dronecan::dsdl::HardwareVersion::default(),
            name: b"org.missionplanner".to_vec(),
        }));
        let fields = fields(&info);
        assert_eq!(fields[0].name, "status");
        assert!(matches!(fields[0].value, FieldValue::Struct(ref inner) if inner.len() == 5));
        assert_eq!(fields[3].name, "name_len");
        assert_eq!(fields[4].name, "name");
        assert_eq!(
            fields[4].value,
            FieldValue::Text("org.missionplanner".to_owned())
        );
        let other = Message::Other {
            info: mp_dronecan::names::by_name("uavcan_equipment_ahrs_MagneticFieldStrength")
                .expect("a row"),
            payload: vec![1, 2, 3],
        };
        let rows = super::fields(&other);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].name, PAYLOAD);
        assert_eq!(
            tag_of(&rows[0].value),
            Some(("1,2,3".to_owned(), "Byte[]".to_owned()))
        );
        // An empty array shows no value.
        assert_eq!(
            tag_of(&FieldValue::Numbers {
                values: Vec::new(),
                type_name: "UInt16[]"
            }),
            Some((String::new(), "UInt16[]".to_owned()))
        );
        assert_eq!(bytes_of(&other), 3);
    }

    /// Selecting a field below the root enables Graph It and Subscribe with `selectedmsgid`
    /// the path; Graph It asks for the history, then a graph of the field fills as messages
    /// come, and Graph It is disabled.
    /// `// C#: Controls/DroneCANInspector.cs:326-446`
    #[test]
    fn graph_it_asks_then_graphs_the_selected_field() {
        let now = Instant::now();
        let mut inspector = Inspector::new(now);
        inspector.heard(&status(127, 5), now);
        inspector.update(now, &|_| "?".to_owned());
        inspector.select(&key(&["127"]));
        assert!(!inspector.graph_it);
        inspector.select(&key(&["127", "341", "uptime_sec"]));
        assert!(inspector.graph_it && inspector.subscribe);
        assert_eq!(
            inspector.selected_path.as_deref(),
            Some("127/341/uptime_sec")
        );
        inspector.press_graph_it();
        assert_eq!(
            inspector.asking.as_ref().map(|i| i.prompt),
            Some(POINTS_PROMPT)
        );
        inspector.type_answer("3");
        assert_eq!(inspector.prompt_done(true, now), Ok(()));
        assert_eq!(inspector.history, 3);
        assert_eq!(inspector.graphs.len(), 1);
        assert!(!inspector.graph_it);
        for uptime in 6..11 {
            inspector.heard(&status(127, uptime), now);
        }
        inspector.tick(now + GRAPH_EVERY, &|_| "?".to_owned());
        let graph = &inspector.graphs[0];
        assert_eq!(graph.shown.list.len(), 1);
        assert_eq!(graph.shown.list[0].label, "uptime_sec");
        // Three points kept of five: the history.
        assert_eq!(graph.shown.list[0].points.len(), 3);
        assert_eq!(graph.shown.list[0].points.back().map(|p| p.1), Some(10.0));
        // A message of another node is not the graph's.
        inspector.heard(&status(5, 99), now);
        inspector.tick(now + 2 * GRAPH_EVERY, &|_| "?".to_owned());
        assert_eq!(
            inspector.graphs[0].shown.list[0].points.back().map(|p| p.1),
            Some(10.0)
        );
        inspector.close_graph(0);
        assert!(inspector.graphs.is_empty());
        // A bad answer is the C#'s FormatException.
        inspector.graph_it = true;
        inspector.press_graph_it();
        inspector.type_answer("x");
        assert!(inspector.prompt_done(true, now).is_err());
    }

    /// `target`: the node, the id and the path; a root or message node graphs nothing.
    #[test]
    fn the_path_is_read_as_the_csharp_splits_it() {
        assert_eq!(
            target("127/341/status/uptime_sec").expect("a path"),
            Some(Target {
                node: 127,
                msgid: 341,
                path: key(&["status", "uptime_sec"]),
            })
        );
        assert_eq!(target("127").expect("a path"), None);
        assert_eq!(target("127/341").expect("a path"), None);
        assert!(target("x/341/f").is_err());
    }

    /// The Subscriber: the type names heard fill its combo once a second; the chosen type's
    /// messages append their JSON, the box kept to the last count of lines as the C#'s
    /// arithmetic keeps it.
    /// `// C#: Controls/DroneCANSubscriber.cs:88-131`
    #[test]
    fn the_subscriber_lists_types_and_keeps_the_last_lines() {
        let now = Instant::now();
        let mut inspector = Inspector::new(now);
        inspector.select(&key(&["127", "341"]));
        inspector.press_subscribe(now);
        let subscriber = inspector.subscriber.as_mut().expect("the Subscriber");
        subscriber.heard(&status(127, 1));
        assert!(subscriber.items.is_empty());
        subscriber.tick(now + TYPES_EVERY);
        assert_eq!(subscriber.items, ["uavcan_protocol_NodeStatus"]);
        subscriber.choose(0);
        assert_eq!(subscriber.target_type, "uavcan_protocol_NodeStatus");
        subscriber.lines = 8;
        subscriber.heard(&status(127, 2));
        let json_lines = 7;
        assert_eq!(subscriber.text.len(), json_lines);
        assert_eq!(subscriber.text[0], "{");
        assert_eq!(subscriber.text[1], "  \"uptime_sec\": 2,");
        subscriber.heard(&status(127, 3));
        // 14 lines, more than 8: the C# skips (7 - 8 < 0 → 0) of them, keeping all 14.
        assert_eq!(subscriber.text.len(), 14);
        subscriber.heard(&status(127, 4));
        // 21 lines: skips 14 - 8 = 6, keeping 15.
        assert_eq!(subscriber.text.len(), 15);
        subscriber.step_lines(true);
        assert_eq!(subscriber.lines, 9);
        for _ in 0..20_000 {
            subscriber.step_lines(true);
        }
        assert_eq!(subscriber.lines, LINES_MAX);
    }

    /// `ToJSON(Formatting.Indented)`: a byte array as Base64, a nested type as an object, a
    /// bool lower-case, an enum its number.
    #[test]
    fn json_is_written_as_json_net_writes_it() {
        let text = json(
            &[
                u8_field("a", 1),
                bool_field("ok", true),
                field("name", FieldValue::Text("hi".to_owned())),
                field(
                    "inner",
                    FieldValue::Struct(vec![enum_field("t", "X", "x_t", 2)]),
                ),
            ],
            0,
        );
        assert_eq!(
            text,
            "{\n  \"a\": 1,\n  \"ok\": true,\n  \"name\": \"aGk=\",\n  \"inner\": {\n    \"t\": 2\n  }\n}"
        );
    }

    /// The fields the generated classes declare, held to the C# tree when it is there: each
    /// name, in order, for the types with plain fields.
    /// `// C#: ExtLibs/DroneCAN/out/include/*.cs`
    #[test]
    fn the_field_names_are_the_generated_classes() {
        for (file, message) in [
            (
                "uavcan.protocol.NodeStatus",
                Message::NodeStatus(NodeStatus::default()),
            ),
            (
                "dronecan.protocol.Stats",
                Message::Stats(mp_dronecan::dsdl::Stats::default()),
            ),
            (
                "dronecan.protocol.CanStats",
                Message::CanStats(mp_dronecan::dsdl::CanStats::default()),
            ),
        ] {
            let Some(source) = crate::config_coverage::source::csharp(&format!(
                "ExtLibs/DroneCAN/out/include/{file}.cs"
            )) else {
                return;
            };
            let declared: Vec<String> = source
                .lines()
                .filter_map(|line| {
                    let line = line.trim();
                    let rest = line.strip_prefix("public ")?;
                    if rest.starts_with("static")
                        || rest.starts_with("const")
                        || rest.starts_with("partial")
                        || rest.starts_with("void")
                    {
                        return None;
                    }
                    let name = rest.split_whitespace().nth(1)?;
                    // `@interface`: the keyword escaped, not part of `FieldInfo.Name`.
                    Some(
                        name.trim_end_matches(';')
                            .trim_start_matches('@')
                            .to_owned(),
                    )
                })
                .collect();
            let ours: Vec<String> = fields(&message).into_iter().map(|f| f.name).collect();
            assert_eq!(ours, declared, "{file}");
        }
    }

    /// The union members' names, held to the generated enums.
    #[test]
    fn the_union_tags_are_the_generated_enums() {
        let Some(source) = crate::config_coverage::source::csharp(
            "ExtLibs/DroneCAN/out/include/uavcan.protocol.param.Value.cs",
        ) else {
            return;
        };
        for value in [
            Value::Empty,
            Value::Integer(1),
            Value::Real(1.0),
            Value::Boolean(1),
            Value::String(Vec::new()),
        ] {
            let Field { value, .. } = value_fields("value", &value);
            let FieldValue::Struct(inner) = value else {
                panic!("a struct");
            };
            let FieldValue::Scalar { text, .. } = &inner[0].value else {
                panic!("the tag");
            };
            assert!(source.contains(text.as_str()), "{text}");
        }
        assert!(source.contains(
            "public uavcan_protocol_param_Value_type_t uavcan_protocol_param_Value_type;"
        ));
    }

    /// The GUI script names only facts and controls this window has.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_window_has() {
        let script = include_str!("../../../../tests/gui/config-dronecan-inspector.gui");
        let source = include_str!("dronecan_inspector.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key))
                    if key.starts_with("config.dronecan.inspector.")
                        || key.starts_with("config.dronecan.subscriber.") =>
                {
                    let head: String = key.split('.').take(4).collect::<Vec<_>>().join(".");
                    assert!(
                        source.contains(&format!("\"{head}\""))
                            || source.contains(&format!("\"{head}.")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id))
                    if id.starts_with("dronecan-inspector")
                        || id.starts_with("dronecan-subscriber") =>
                {
                    let fixed = id.split('@').next().unwrap_or(id);
                    let drawn = source.contains(&format!("\"{fixed}\""))
                        || fixed.starts_with("dronecan-inspector-toggle-")
                        || fixed.starts_with("dronecan-inspector-node-")
                        || fixed.starts_with("dronecan-inspector-graph-")
                        || fixed.starts_with("dronecan-inspector-points-")
                        || fixed.starts_with("dronecan-subscriber-type-")
                        || fixed.starts_with("dronecan-subscriber-lines-");
                    assert!(drawn, "{id} is not drawn");
                }
                _ => {}
            }
        }
        assert!(facts >= 8, "{facts} facts");
    }
}
