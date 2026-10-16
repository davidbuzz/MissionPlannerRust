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

//! `param.json`: each vehicle's parameters kept on disk as `MAVState` keeps them, and read back
//! in place of the fetch when `UseCachedParams` is set.
//! `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:27-46, 63-95; MainV2.cs:450, 1659-1667`
//!
//! # Where
//!
//! `ParamCachePath`: `Settings.GetDataDirectory()`, then `paramcache`, the vehicle's `MAV_TYPE`
//! by its name in the C#'s enum (`aptype.ToString()`: `QUADROTOR`, or the number for a type the
//! enum does not name), the board's `uid2` from `AUTOPILOT_VERSION` as lowercase hex
//! (`ToHexString`, 36 digits), the system id, the component id, and `param.json` - one file per
//! board and vehicle id ([`path`]). The data directory is the link's
//! [`crate::LinkConfig::data_directory`]. Until `AUTOPILOT_VERSION` is heard `cs.uid2` is null,
//! `Path.Combine` throws and the property returns ""; here there is no path then ([`path_for`]).
//! `aptype` is the type of the first heartbeat there and of the latest here, which differ only for
//! a vehicle that changes its `MAV_TYPE`.
//!
//! # When it is written
//!
//! `MAVState` handles the list's `PropertyChanged`: the first change sets `queuewrite` and starts
//! a timer, and two seconds later the whole list as it is then is written; changes before the
//! timer fires are not queued again, being in what it writes (`MAVState.cs:65-95`). The list
//! raises the event when its indexer replaces a name it holds - not for a name it adds, and not
//! on `Clear` - and on every `AddRange` (`ExtLibs/Mavlink/MAVLinkParamList.cs:42-64, 148-172`).
//! On the link that is:
//!
//! * a `PARAM_VALUE` for a name the table holds - `processInfoFromStream`'s `param[st] = ...`
//!   (`ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5729-5761`) - and one answering a set or a
//!   read, which `setParam` and `GetParamAsync` then put in the list again themselves, a name
//!   the frame has just added among them (`:1703-1713`, `:2372-2386`; the stream's handler runs
//!   first, `:5101`, `:5347`);
//! * a fetch finished: the MAVFTP file's `AddRange` (`:1899-1902`), and the stream's, on the
//!   frame that completes it, an empty list included (`:2240-2242`); a cancelled stream returns
//!   before its `AddRange` and queues nothing;
//! * the cached read itself, for each name it brings that the table held (`MainV2.cs:1664-1665`,
//!   through `Add`).
//!
//! Not a mission upload's `WP_TOTAL`, `CMD_TOTAL` and `MIS_TOTAL`, which the C# changes by setting
//! `.Value` on the held object (`:3806-3811`); they are in the next file written, as there.
//!
//! The timer writes nothing while `cs.uid2` is null or empty or the system id is 0, and that
//! `return` comes before `queuewrite = false` (`MAVState.cs:79-80, 92`): a change heard before the
//! board's `AUTOPILOT_VERSION` stops the cache for that vehicle for the rest of the session. That
//! is ported as it is. A failed write is logged there and swallowed here, and either way the next
//! change queues a write again. A write still queued when the link stops is written as it stops,
//! where the C#'s timer would fire after the port had closed.
//!
//! # What is written
//!
//! `param.ToJSON()` (`ExtLibs/Utilities/Extensions.cs:413-430`): Json.NET's
//! `SerializeObject(param, Formatting.Indented)` with no settings of the program's own
//! ([`to_json`]). `MAVLinkParamList` is a `List<MAVLinkParam>`, so an array, without its
//! `TotalReported`; two spaces a level, `"name": value`, each line ended with
//! `Environment.NewLine` - `\r\n` on Windows, `\n` under Mono and here elsewhere - and
//! `File.WriteAllText`'s UTF-8 without a byte-order mark. Each `MAVLinkParam` is its public
//! members, fields before properties as Json.NET's reflection lists them
//! (`ReflectionUtils.GetFieldsAndProperties`, then a stable sort on an `Order` none of them has):
//! the readonly field `default_value`, then `Name`, `Type`, `Value` and `TypeAP`; `float_value`
//! and `data` carry `[IgnoreDataMember]`, which Json.NET honours, and the rest are private
//! (`ExtLibs/Mavlink/MAVLinkParam.cs:11-73`). The enums are their numbers. A number is
//! `JsonConvert.ToString`'s ([`json_number`]): `double.ToString("R")` as .NET Framework
//! (`net472`) writes it - 15 significant digits, or 17 when 15 do not read back as the same
//! double, the exponent form (`1E-05`) below 10^-4 and from 10^15 (10^17 on the 17-digit form) -
//! with `.0` added to a number with neither a point nor an exponent, and `"NaN"`, `"Infinity"` and
//! `"-Infinity"` in quotes.
//!
//! Each field is what `MAVLinkParam` holds: `Type` the type the four bytes are read as - `REAL32`
//! for anything ArduPilot sent in a `PARAM_VALUE`, the declared type for any other autopilot -
//! `TypeAP` the type the vehicle keeps it as, and `Value` `GetValue()`, a float through its seven
//! significant digits (`:100-145`). The table holds those as its wire and storage types. Two
//! things it does not hold are kept beside it, per vehicle: a parameter the MAVFTP file brought
//! is `Type` the file's type there (`ExtLibs/ArduPilot/parampck.cs:113-121`) where the table reads
//! it as ArduPilot's float, and has the file's default as `default_value`; both last until a
//! `PARAM_VALUE` puts a new `MAVLinkParam` in its place, which has no default unless it is the
//! echo of an ArduPilot set, whose `setParam` passes the old one on
//! (`MAVLinkInterface.cs:1705-1707`). A float default is the widened float, `Convert.ToDouble`'s,
//! so `0.1` is `0.10000000149011612`.
//!
//! The order is the list's: the order its names were added, the MAVFTP file's order after a
//! MAVFTP fetch. One difference: after a stream fetch the C#'s list is rebuilt in the order that
//! fetch heard the names, where here a name heard before the fetch keeps its place.
//!
//! The values can differ from the C#'s file in their last digits, never in what reads back: the
//! C#'s `RoundToSignificantDigits` can leave a float a bit off its nearest double
//! (13.661200000000001 there, 13.6612 here; see [`mp_params::round_to_significant_digits`]), and
//! both read back as the same float.
//!
//! # How it is read
//!
//! `doConnect`, connecting with `getparams` to anything but a `MAV_COMP_ID_PERIPHERAL`: with
//! `UseCachedParams` set and the file there, last written less than an hour ago (or after now),
//! each `MAVLinkParam` of `FromJSON<MAVLinkParamList>` is `Add`ed to the list - a name it holds
//! replaced, a new one added - and `TotalReported` set to the count, *in place of* the fetch:
//! nothing is asked of the vehicle; otherwise the list is fetched (`MainV2.cs:1659-1689`). That
//! is [`Link::read_cached_params`], gated by [`crate::LinkConfig::use_cached_params`]. Nothing in
//! the C# sets `UseCachedParams` - a public property of `MainV2`, false by default, which no
//! setting, argument or plugin in the tree touches - so a connection there always fetches; it is
//! ported as the C# has it, off.
//!
//! Json.NET builds each `MAVLinkParam` with its private constructor and sets its members in the
//! file's order, matching a name exactly or else ignoring case: `Name`; `Type` and `TypeAP` from a
//! number or a name; `Value` through `SetValue` under the `Type` set so far - so a `Value` before
//! its `Type` leaves the bytes zero - with the C#'s unchecked casts; the readonly `default_value`,
//! the ignored members and anything else skipped ([`read`]). What does not read - not an array, an
//! entry that is not an object, a value of the wrong kind, text after the end - throws there,
//! and `doConnect`'s catch closes the connection with "Can not establish a connection"; here it
//! is the error [`Link::read_cached_params`] returns. An entry the table cannot hold - no name, or
//! a type outside `MAV_PARAM_TYPE`'s 32-bit ones, which the C# keeps and whose `GetValue` throws
//! later - is left out, as the link leaves out such a `PARAM_VALUE`. Json.NET's leniencies beyond
//! JSON itself (comments, single quotes, names without quotes) are not read.

use mp_os::Lock as _;
use mp_os::fs::FsExt as _;
use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use web_time::{Duration, Instant};

use mp_mavlink_dialects::all::MavType;
use mp_params::parampck::PackedParam;
use mp_params::{ParamTable, ParamType, ParamValue, round_to_significant_digits};
use mp_vehicle::onboard::AutopilotInfo;
use mp_vehicle::{VehicleId, VehicleRegistry, VehicleState};

use crate::requests::RequestKind;
use crate::{Link, Shared};

/// The directory under the data directory that holds every vehicle's cache.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:33`
pub const DIRECTORY: &str = "paramcache";

/// The file's name.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:38`
pub const FILE_NAME: &str = "param.json";

/// How long a file is read for after it was written: `LastWriteTime > DateTime.Now.AddHours(-1)`.
/// `// C#: MainV2.cs:1661-1662`
pub const FRESH_FOR: Duration = Duration::from_secs(60 * 60);

/// `Environment.NewLine`, which Json.NET ends each line with: `\r\n` on Windows, `\n` under Mono.
pub const NEWLINE: &str = if cfg!(windows) { "\r\n" } else { "\n" };

/// `MAV_COMPONENT.MAV_COMP_ID_PERIPHERAL`, which `doConnect` neither fetches nor reads the cache
/// for: "devices that do not implement the parameter microservice".
/// `// C#: MainV2.cs:1658-1659`
pub const MAV_COMP_ID_PERIPHERAL: u8 = 158;

/// How deep the reader follows nested arrays and objects: Json.NET's `MaxDepth`, 64.
const MAX_DEPTH: usize = 64;

/// The C#'s `MAV_PARAM_TYPE`, by name, for a `Type` written as one.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:3923-3956`
const PARAM_TYPE_NAMES: [(&str, u8); 10] = [
    ("UINT8", 1),
    ("INT8", 2),
    ("UINT16", 3),
    ("INT16", 4),
    ("UINT32", 5),
    ("INT32", 6),
    ("UINT64", 7),
    ("INT64", 8),
    ("REAL32", 9),
    ("REAL64", 10),
];

/// `aptype.ToString()`: the C#'s `MAV_TYPE` name, which is the dialect's without its `MAV_TYPE_`
/// prefix (`GENERIC` to `GRIPPER`, 0 to 48, as both have them), or the number for a value the
/// enum does not name, as `Enum.ToString` gives it.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:7190-7340`
#[must_use]
pub fn aptype_name(vehicle_type: u8) -> String {
    MavType(u32::from(vehicle_type))
        .name()
        .and_then(|name| name.strip_prefix("MAV_TYPE_"))
        .map_or_else(|| vehicle_type.to_string(), ToOwned::to_owned)
}

/// `cs.uid2`: `version.uid2.ToHexString()`, two lowercase hex digits a byte.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2377; ExtLibs/Utilities/Extensions.cs:397-403`
#[must_use]
pub fn uid2_hex(uid2: &[u8; 18]) -> String {
    let mut out = String::with_capacity(uid2.len() * 2);
    for byte in uid2 {
        let _ = write!(out, "{byte:02x}");
    }
    out
}

/// `ParamCachePath`: `<data directory>/paramcache/<aptype>/<uid2>/<sysid>/<compid>/param.json`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:27-46`
#[must_use]
pub fn path(data_directory: &Path, vehicle_type: u8, uid2: &[u8; 18], id: VehicleId) -> PathBuf {
    data_directory
        .join(DIRECTORY)
        .join(aptype_name(vehicle_type))
        .join(uid2_hex(uid2))
        .join(id.sysid.to_string())
        .join(id.compid.to_string())
        .join(FILE_NAME)
}

/// A vehicle's `ParamCachePath`, once it has said what board it is: none before its
/// `AUTOPILOT_VERSION`, while `cs.uid2` is null and the C#'s property returns "". An
/// `AUTOPILOT_VERSION` of nothing but zeros is taken for none, the state having no other mark of
/// one heard.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:27-46`
#[must_use]
pub fn path_for(data_directory: &Path, state: &VehicleState, id: VehicleId) -> Option<PathBuf> {
    (state.autopilot_info != AutopilotInfo::default())
        .then(|| path(data_directory, state.vehicle_type, &state.autopilot_info.uid2, id))
}

/// `File.Exists(path) && new FileInfo(path).LastWriteTime > DateTime.Now.AddHours(-1)`: a file
/// written less than [`FRESH_FOR`] ago, or after now, as the C#'s comparison has it. The C#
/// compares local times, so there an hour spanning a change to or from summer time is an hour
/// longer or shorter; here it is an hour of the system clock.
/// `// C#: MainV2.cs:1661-1662`
#[must_use]
pub fn is_fresh(path: &Path) -> bool {
    let Ok(meta) = mp_os::fs::metadata(path) else {
        return false;
    };
    meta.os_is_file()
        && meta.modified().is_ok_and(|written| match written.elapsed() {
            Ok(age) => age < FRESH_FOR,
            // Written after now: `LastWriteTime > DateTime.Now.AddHours(-1)` holds.
            Err(_) => true,
        })
}

/// One parameter as `MAVLinkParam` holds it, which is what the file holds, its members in the
/// order Json.NET writes them.
/// `// C#: ExtLibs/Mavlink/MAVLinkParam.cs:11-51`
#[derive(Debug, Clone, PartialEq)]
pub struct CachedParam {
    /// `default_value`: the MAVFTP file's default, `null` for anything else.
    pub default_value: Option<f64>,
    /// `Name`.
    pub name: String,
    /// `Type`, the `MAV_PARAM_TYPE` the four bytes are read as.
    pub param_type: u8,
    /// `Value`: `GetValue()`.
    pub value: f64,
    /// `TypeAP`, the `MAV_PARAM_TYPE` the vehicle keeps it as.
    pub type_ap: u8,
}

impl CachedParam {
    /// What `MAVLinkParam` holds for a table's entry: `Type` the wire type, or `file_type` for an
    /// entry the MAVFTP file brought; `TypeAP` the storage type; `Value` the bytes read as `Type`,
    /// a float through its seven significant digits.
    /// `// C#: ExtLibs/Mavlink/MAVLinkParam.cs:100-145; ExtLibs/ArduPilot/parampck.cs:113-121`
    #[must_use]
    pub fn of(
        name: &str,
        value: ParamValue,
        file_type: Option<ParamType>,
        default_value: Option<f64>,
    ) -> Self {
        let read_as = file_type.unwrap_or(value.wire_type());
        let number = if read_as == ParamType::Real32 && value.wire_type() == ParamType::Real32 {
            round_to_significant_digits(f64::from(value.to_param_value_field()), 7)
        } else {
            // An integer, read exactly: the table's own reading of its bytes.
            value.as_f64()
        };
        Self {
            default_value,
            name: name.to_owned(),
            param_type: read_as.to_wire(),
            value: number,
            type_ap: value.param_type().to_wire(),
        }
    }
}

/// `param.ToJSON()`: the list as Json.NET's indented JSON (see the module's "What is written").
/// `// C#: ExtLibs/Utilities/Extensions.cs:413-430`
#[must_use]
pub fn to_json(params: &[CachedParam]) -> String {
    if params.is_empty() {
        return "[]".to_owned();
    }
    let mut out = String::with_capacity(params.len() * 128);
    out.push('[');
    for (n, param) in params.iter().enumerate() {
        if n > 0 {
            out.push(',');
        }
        out.push_str(NEWLINE);
        out.push_str("  {");
        let members = [
            (
                "default_value",
                param
                    .default_value
                    .map_or_else(|| "null".to_owned(), json_number),
            ),
            ("Name", quoted(&param.name)),
            ("Type", param.param_type.to_string()),
            ("Value", json_number(param.value)),
            ("TypeAP", param.type_ap.to_string()),
        ];
        for (m, (key, value)) in members.iter().enumerate() {
            if m > 0 {
                out.push(',');
            }
            out.push_str(NEWLINE);
            out.push_str("    \"");
            out.push_str(key);
            out.push_str("\": ");
            out.push_str(value);
        }
        out.push_str(NEWLINE);
        out.push_str("  }");
    }
    out.push_str(NEWLINE);
    out.push(']');
    out
}

/// `JsonConvert.ToString(double)` as the writer calls it: `EnsureDecimalPlace` of
/// `ToString("R", CultureInfo.InvariantCulture)`, and NaN and the infinities quoted
/// (`FloatFormatHandling.String`, the default).
#[must_use]
pub fn json_number(value: f64) -> String {
    if value.is_nan() {
        return "\"NaN\"".to_owned();
    }
    if value.is_infinite() {
        return if value > 0.0 {
            "\"Infinity\""
        } else {
            "\"-Infinity\""
        }
        .to_owned();
    }
    let text = round_trip(value);
    if text.contains(['.', 'E', 'e']) {
        text
    } else {
        text + ".0"
    }
}

/// `double.ToString("R", CultureInfo.InvariantCulture)` on .NET Framework: the general format
/// with 15 significant digits when that reads back as the same double, else with 17. Negative
/// zero is `0` there (.NET Core 3.0 began writing `-0`).
fn round_trip(value: f64) -> String {
    if value == 0.0 {
        return "0".to_owned();
    }
    let short = general(value, 15);
    // The C#'s own test: the 15 digits parsed, compared with the value.
    #[allow(clippy::float_cmp)]
    let reads_back = short.parse::<f64>().is_ok_and(|back| back == value);
    if reads_back {
        short
    } else {
        general(value, 17)
    }
}

/// .NET's general format of a finite, non-zero double to `precision` significant digits: the
/// digits rounded from the exact binary value with trailing zeros dropped, plain while the first
/// digit's place is below 10^`precision` and at or above 10^-4, else `d.dddE+XX` with the
/// exponent's sign and at least two digits (`FormatGeneral`, `FormatExponent(..., 2, true)`).
fn general(value: f64, precision: usize) -> String {
    let scientific = format!("{:.*e}", precision.saturating_sub(1), value.abs());
    let (mantissa, exponent) = scientific
        .split_once('e')
        .unwrap_or((scientific.as_str(), "0"));
    let exponent: i32 = exponent.parse().unwrap_or(0);
    let all: String = mantissa.chars().filter(char::is_ascii_digit).collect();
    let digits = match all.trim_end_matches('0') {
        "" => "0",
        kept => kept,
    };
    // `number.scale`: the digits are 0.d1d2... times ten to this.
    let scale = exponent + 1;
    let limit = i32::try_from(precision).unwrap_or(i32::MAX);
    let mut out = String::new();
    if value.is_sign_negative() {
        out.push('-');
    }
    if scale > limit || scale < -3 {
        let (first, rest) = digits.split_at(1);
        out.push_str(first);
        if !rest.is_empty() {
            out.push('.');
            out.push_str(rest);
        }
        let shown = scale - 1;
        out.push('E');
        out.push(if shown < 0 { '-' } else { '+' });
        let _ = write!(out, "{:02}", shown.unsigned_abs());
    } else if scale > 0 {
        let whole = usize::try_from(scale).unwrap_or(0);
        if digits.len() > whole {
            let (int, fraction) = digits.split_at(whole);
            out.push_str(int);
            out.push('.');
            out.push_str(fraction);
        } else {
            out.push_str(digits);
            out.push_str(&"0".repeat(whole - digits.len()));
        }
    } else {
        out.push_str("0.");
        out.push_str(&"0".repeat(usize::try_from(-scale).unwrap_or(0)));
        out.push_str(digits);
    }
    out
}

/// A string as Json.NET writes one in double quotes: `"` and `\` escaped, `\n`, `\r`, `\t`, `\b`
/// and `\f` by name, any other control character and U+0085, U+2028 and U+2029 as `\u` with four
/// lowercase hex digits, everything else as it is (`StringEscapeHandling.Default`).
fn quoted(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    out.push('"');
    for c in text.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\u{8}' => out.push_str("\\b"),
            '\u{c}' => out.push_str("\\f"),
            c if u32::from(c) < 0x20 || matches!(c, '\u{85}' | '\u{2028}' | '\u{2029}') => {
                let _ = write!(out, "\\u{:04x}", u32::from(c));
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

/// `FromJSON<MAVLinkParamList>()`: the file's parameters, in its order, each as the table holds
/// it - `Type` the wire type, `TypeAP` the storage type, the bytes `SetValue` made of `Value`.
/// An error where Json.NET throws (see the module's "How it is read").
/// `// C#: MainV2.cs:1664; ExtLibs/Utilities/Extensions.cs:459-462; ExtLibs/Mavlink/MAVLinkParam.cs:147-177`
///
/// # Errors
/// The text is not JSON, or not an array of objects, or a member holds the wrong kind of value.
pub fn read(text: &str) -> Result<Vec<(String, ParamValue)>, String> {
    // `File.ReadAllText` drops a byte-order mark.
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    let mut parser = Parser { text, at: 0 };
    let root = parser.value(0)?;
    parser.space();
    if parser.at < text.len() {
        return Err(parser.error(
            "Additional text found in JSON string after finishing deserializing object",
        ));
    }
    let items = match root {
        Json::Array(items) => items,
        other => {
            return Err(format!(
                "Cannot deserialize {} into a MAVLinkParamList, which needs a JSON array",
                other.kind()
            ));
        }
    };
    let mut out = Vec::with_capacity(items.len());
    for (n, item) in items.into_iter().enumerate() {
        let members = match item {
            Json::Object(members) => members,
            other => {
                return Err(format!(
                    "entry {n} is {}, where a MAVLinkParam is an object",
                    other.kind()
                ));
            }
        };
        let mut param = Reading::default();
        for (key, value) in members {
            param
                .apply(&key, value)
                .map_err(|why| format!("entry {n}: {why}"))?;
        }
        if let Some(entry) = param.into_entry() {
            out.push(entry);
        }
    }
    Ok(out)
}

/// A `MAVLinkParam` as Json.NET fills it, member by member in the file's order.
#[derive(Debug, Default)]
struct Reading {
    /// `Name`; null until set.
    name: Option<String>,
    /// `Type`; 0 until set.
    param_type: u8,
    /// `_typeap`; 0 until set, when the `TypeAP` getter answers `Type`.
    type_ap: u8,
    /// `data`: four zero bytes until `Value` is set under a type `SetValue` knows.
    data: [u8; 4],
}

/// The members Json.NET matches a file's names against (`MAVLinkParam.cs:11-73`), exactly and
/// then ignoring case; `default_value` is readonly and the last two ignored, so four are set.
const MEMBERS: [&str; 7] = [
    "default_value",
    "Name",
    "Type",
    "Value",
    "TypeAP",
    "float_value",
    "data",
];

impl Reading {
    /// One member of the file's object.
    fn apply(&mut self, key: &str, value: Json) -> Result<(), String> {
        let member = MEMBERS
            .iter()
            .find(|member| **member == key)
            .or_else(|| MEMBERS.iter().find(|member| member.eq_ignore_ascii_case(key)));
        match member.copied() {
            Some("Name") => {
                self.name = match value {
                    Json::Text(text) | Json::Number(text) => Some(text),
                    Json::Null => None,
                    other => return Err(format!("Name is {}", other.kind())),
                };
            }
            Some("Type") => self.param_type = enum_value("Type", &value)?,
            Some("TypeAP") => self.type_ap = enum_value("TypeAP", &value)?,
            Some("Value") => {
                // `SetValue(value)` under the `Type` set so far; a type it has no case for leaves
                // the bytes as they are.
                if let Some(bytes) = set_value(self.param_type, double(&value)?) {
                    self.data = bytes;
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// The entry for the table: none without a name or with a type outside the 32-bit ones. A
    /// `REAL32` entry is ArduPilot's float with `TypeAP` as its storage type; an integer one is
    /// its bytes read as `Type`, which the C# never writes with a different `TypeAP`.
    fn into_entry(self) -> Option<(String, ParamValue)> {
        let name = self.name?;
        let wire = ParamType::from_wire(self.param_type)?;
        // `TypeAP`'s getter: `_typeap` once set, `Type` before.
        let kept = if self.type_ap == 0 {
            self.param_type
        } else {
            self.type_ap
        };
        let storage = ParamType::from_wire(kept)?;
        let value = if wire == ParamType::Real32 {
            ParamValue::from_ardupilot(f32::from_le_bytes(self.data), storage)
        } else {
            ParamValue::from_wire_bytes(self.data, wire)
        };
        Some((name, value))
    }
}

/// `SetValue(input)`: the number in `param_type`'s bytes through the C#'s unchecked casts -
/// toward zero, then the low bytes, which agree with the C#'s for any value the type can hold -
/// or a float's four; none for a type it has no case for.
/// `// C#: ExtLibs/Mavlink/MAVLinkParam.cs:147-177`
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn set_value(param_type: u8, input: f64) -> Option<[u8; 4]> {
    let whole = input as i64;
    Some(match param_type {
        1 => [whole as u8, 0, 0, 0],
        2 => [whole as i8 as u8, 0, 0, 0],
        3 => {
            let [a, b] = (whole as u16).to_le_bytes();
            [a, b, 0, 0]
        }
        4 => {
            let [a, b] = (whole as i16).to_le_bytes();
            [a, b, 0, 0]
        }
        5 => (whole as u32).to_le_bytes(),
        6 => (whole as i32).to_le_bytes(),
        9 => (input as f32).to_le_bytes(),
        _ => return None,
    })
}

/// A `MAV_PARAM_TYPE` member: a number, or a name or number in a string, as Json.NET reads an
/// enum; anything else, or a number past the byte the enum is, throws there.
fn enum_value(member: &str, value: &Json) -> Result<u8, String> {
    match value {
        Json::Number(text) => text
            .parse::<u8>()
            .map_err(|_| format!("{member} is {text}, not a MAV_PARAM_TYPE")),
        Json::Text(text) => {
            let text = text.trim();
            text.parse::<u8>()
                .ok()
                .or_else(|| {
                    PARAM_TYPE_NAMES
                        .iter()
                        .find(|(name, _)| name.eq_ignore_ascii_case(text))
                        .map(|(_, number)| *number)
                })
                .ok_or_else(|| format!("{member} is \"{text}\", not a MAV_PARAM_TYPE"))
        }
        other => Err(format!("{member} is {}", other.kind())),
    }
}

/// A `double` member: a number, or a number in a string, `NaN` and the infinities among them;
/// null or anything else throws there.
fn double(value: &Json) -> Result<f64, String> {
    let text = match value {
        Json::Number(text) | Json::Text(text) => text.trim(),
        other => return Err(format!("Value is {}", other.kind())),
    };
    match text {
        "NaN" => Ok(f64::NAN),
        "Infinity" => Ok(f64::INFINITY),
        "-Infinity" => Ok(f64::NEG_INFINITY),
        _ => text
            .parse::<f64>()
            .map_err(|_| format!("Value is \"{text}\", not a number")),
    }
}

/// A JSON value, as far as the reader needs one.
#[derive(Debug, Clone, PartialEq)]
enum Json {
    Null,
    Bool,
    /// The number as written, read when its member says what it is.
    Number(String),
    Text(String),
    Array(Vec<Json>),
    Object(Vec<(String, Json)>),
}

impl Json {
    /// What it is, for an error.
    const fn kind(&self) -> &'static str {
        match self {
            Self::Null => "null",
            Self::Bool => "a boolean",
            Self::Number(_) => "a number",
            Self::Text(_) => "a string",
            Self::Array(_) => "an array",
            Self::Object(_) => "an object",
        }
    }
}

/// A reader of JSON, with Json.NET's bare `NaN`, `Infinity` and `-Infinity` among the numbers.
struct Parser<'a> {
    text: &'a str,
    at: usize,
}

impl Parser<'_> {
    fn peek(&self) -> Option<u8> {
        self.text.as_bytes().get(self.at).copied()
    }

    fn space(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\t' | b'\n' | b'\r')) {
            self.at += 1;
        }
    }

    fn eat(&mut self, byte: u8) -> bool {
        let found = self.peek() == Some(byte);
        if found {
            self.at += 1;
        }
        found
    }

    fn word(&mut self, word: &str) -> bool {
        let found = self
            .text
            .get(self.at..)
            .is_some_and(|rest| rest.starts_with(word));
        if found {
            self.at += word.len();
        }
        found
    }

    fn digits(&mut self) -> bool {
        let start = self.at;
        while matches!(self.peek(), Some(b'0'..=b'9')) {
            self.at += 1;
        }
        self.at > start
    }

    fn next_char(&self) -> Option<char> {
        self.text.get(self.at..).and_then(|rest| rest.chars().next())
    }

    fn error(&self, what: &str) -> String {
        format!("{what}, at character {}", self.at)
    }

    fn value(&mut self, depth: usize) -> Result<Json, String> {
        if depth > MAX_DEPTH {
            return Err(self.error("the JSON nests deeper than 64"));
        }
        self.space();
        match self.peek() {
            Some(b'[') => {
                self.at += 1;
                let mut items = Vec::new();
                self.space();
                if self.eat(b']') {
                    return Ok(Json::Array(items));
                }
                loop {
                    items.push(self.value(depth + 1)?);
                    self.space();
                    if self.eat(b',') {
                        continue;
                    }
                    if self.eat(b']') {
                        return Ok(Json::Array(items));
                    }
                    return Err(self.error("expected , or ]"));
                }
            }
            Some(b'{') => {
                self.at += 1;
                let mut members = Vec::new();
                self.space();
                if self.eat(b'}') {
                    return Ok(Json::Object(members));
                }
                loop {
                    self.space();
                    if self.peek() != Some(b'"') {
                        return Err(self.error("expected a property name"));
                    }
                    let key = self.string()?;
                    self.space();
                    if !self.eat(b':') {
                        return Err(self.error("expected :"));
                    }
                    let value = self.value(depth + 1)?;
                    members.push((key, value));
                    self.space();
                    if self.eat(b',') {
                        continue;
                    }
                    if self.eat(b'}') {
                        return Ok(Json::Object(members));
                    }
                    return Err(self.error("expected , or }"));
                }
            }
            Some(b'"') => self.string().map(Json::Text),
            Some(b't') if self.word("true") => Ok(Json::Bool),
            Some(b'f') if self.word("false") => Ok(Json::Bool),
            Some(b'n') if self.word("null") => Ok(Json::Null),
            Some(b'N') if self.word("NaN") => Ok(Json::Number("NaN".to_owned())),
            Some(b'I') if self.word("Infinity") => Ok(Json::Number("Infinity".to_owned())),
            Some(b'-' | b'0'..=b'9') => self.number(),
            None => Err(self.error("the text ended where a value was expected")),
            Some(_) => Err(self.error("unexpected character")),
        }
    }

    fn number(&mut self) -> Result<Json, String> {
        if self.word("-Infinity") {
            return Ok(Json::Number("-Infinity".to_owned()));
        }
        let start = self.at;
        self.eat(b'-');
        if !self.digits() {
            return Err(self.error("a number without digits"));
        }
        if self.eat(b'.') && !self.digits() {
            return Err(self.error("a number without digits after its point"));
        }
        if matches!(self.peek(), Some(b'e' | b'E')) {
            self.at += 1;
            if !self.eat(b'+') {
                self.eat(b'-');
            }
            if !self.digits() {
                return Err(self.error("a number without digits in its exponent"));
            }
        }
        Ok(Json::Number(
            self.text.get(start..self.at).unwrap_or_default().to_owned(),
        ))
    }

    /// A string, from its opening quote.
    fn string(&mut self) -> Result<String, String> {
        self.at += 1;
        let mut out = String::new();
        loop {
            let Some(c) = self.next_char() else {
                return Err(self.error("the text ended inside a string"));
            };
            self.at += c.len_utf8();
            match c {
                '"' => return Ok(out),
                '\\' => {
                    let Some(escaped) = self.next_char() else {
                        return Err(self.error("the text ended inside a string"));
                    };
                    self.at += escaped.len_utf8();
                    match escaped {
                        '"' => out.push('"'),
                        '\\' => out.push('\\'),
                        '/' => out.push('/'),
                        'b' => out.push('\u{8}'),
                        'f' => out.push('\u{c}'),
                        'n' => out.push('\n'),
                        'r' => out.push('\r'),
                        't' => out.push('\t'),
                        'u' => {
                            let unit = self.hex4()?;
                            let decoded = if (0xD800..0xDC00).contains(&unit) && self.word("\\u") {
                                let low = self.hex4()?;
                                if (0xDC00..0xE000).contains(&low) {
                                    char::from_u32(
                                        0x10000 + ((unit - 0xD800) << 10) + (low - 0xDC00),
                                    )
                                } else {
                                    // A lone high surrogate, which a Rust string cannot hold.
                                    out.push('\u{fffd}');
                                    char::from_u32(low)
                                }
                            } else {
                                char::from_u32(unit)
                            };
                            out.push(decoded.unwrap_or('\u{fffd}'));
                        }
                        _ => return Err(self.error("an unknown escape")),
                    }
                }
                c => out.push(c),
            }
        }
    }

    fn hex4(&mut self) -> Result<u32, String> {
        let text = self.text;
        let digits = text
            .get(self.at..self.at + 4)
            .filter(|digits| digits.bytes().all(|b| b.is_ascii_hexdigit()))
            .ok_or_else(|| self.error("a \\u escape without four hex digits"))?;
        let unit = u32::from_str_radix(digits, 16).map_err(|_| self.error("a bad \\u escape"))?;
        self.at += 4;
        Ok(unit)
    }
}

/// Which waiting requests a `PARAM_VALUE` answered: `setParam`'s and `GetParamAsync`'s, each of
/// which puts the parameter in the list again itself.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct Answered {
    /// A set of this name.
    set: bool,
    /// A read of this name or index.
    read: bool,
}

impl Answered {
    /// `kind` took the frame.
    pub(crate) fn took(&mut self, kind: &RequestKind) {
        match kind {
            RequestKind::SetParam { .. } => self.set = true,
            RequestKind::ReadParam(_) => self.read = true,
            _ => {}
        }
    }
}

/// Where a vehicle's write is: `queuewrite` and its timer.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
enum Write {
    /// Nothing queued.
    #[default]
    Idle,
    /// A change since this moment, written [`crate::ProtocolTimeouts::param_cache_write`] after
    /// it.
    Queued(Instant),
    /// The timer came while the board's `uid2` was not known, or the system id was 0, and its
    /// `return` left `queuewrite` set: nothing more is written for this vehicle.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:79-80, 92`
    Stopped,
}

/// One vehicle's cache: what its `MAVLinkParam`s hold that the table does not, and its write.
#[derive(Debug, Default)]
pub(crate) struct VehicleCache {
    /// Where each name sits in the C#'s list: the order the names were added.
    places: BTreeMap<String, u64>,
    /// The place the next new name takes.
    next_place: u64,
    /// The MAVFTP file's type for each parameter it brought and nothing has replaced since: its
    /// `Type` there, where the table reads it as ArduPilot's float.
    file_types: BTreeMap<String, ParamType>,
    /// `default_value`, for each parameter that has one.
    defaults: BTreeMap<String, f64>,
    /// `queuewrite`.
    write: Write,
}

impl VehicleCache {
    /// A name added to the list, at its end; a name it holds keeps its place.
    fn place(&mut self, name: &str) {
        if !self.places.contains_key(name) {
            self.places.insert(name.to_owned(), self.next_place);
            self.next_place += 1;
        }
    }

    /// `PropertyChanged`: a write queued, unless one is (or the cache has stopped).
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:65-76`
    fn changed(&mut self, now: Instant) {
        if self.write == Write::Idle {
            self.write = Write::Queued(now);
        }
    }

    /// The list as the C# holds it: the table's parameters in the list's order - a name with no
    /// place, which nothing should leave, after the rest by name.
    fn entries(&self, table: Option<&ParamTable>) -> Vec<CachedParam> {
        let Some(table) = table else {
            return Vec::new();
        };
        let mut listed: Vec<(u64, &String, ParamValue)> = table
            .iter()
            .map(|(name, value)| {
                let place = self.places.get(name).copied().unwrap_or(u64::MAX);
                (place, name, *value)
            })
            .collect();
        // Stable, and the table iterates by name, so unplaced names stay in name order.
        listed.sort_by_key(|(place, ..)| *place);
        listed
            .into_iter()
            .map(|(_, name, value)| {
                CachedParam::of(
                    name,
                    value,
                    self.file_types.get(name).copied(),
                    self.defaults.get(name).copied(),
                )
            })
            .collect()
    }
}

/// Every vehicle's cache on a link, and where they are kept.
#[derive(Debug, Default)]
pub(crate) struct ParamCaches {
    /// `Settings.GetDataDirectory()`; `None` keeps no cache, and nothing here is tracked.
    pub(crate) directory: Option<PathBuf>,
    vehicles: BTreeMap<VehicleId, VehicleCache>,
}

impl ParamCaches {
    /// The vehicle's cache, if a cache is kept.
    fn vehicle(&mut self, id: VehicleId) -> Option<&mut VehicleCache> {
        self.directory.as_ref()?;
        Some(self.vehicles.entry(id).or_default())
    }
}

/// A `PARAM_VALUE` the table is about to take for `name`: the list's `param[st] = new
/// MAVLinkParam(...)` - the entry's file type gone, and its default unless `setParam`'s own
/// replacement puts it back, ArduPilot's passing `param.default_value` and the other autopilots'
/// none; a new name added at the list's end; and a write queued when the list held the name or a
/// set or read takes the frame. Called with the parameter table locked (lock order: parameters,
/// then the cache).
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1703-1713, 2372-2386, 5729-5761`
pub(crate) fn on_param_value(
    shared: &Shared,
    id: VehicleId,
    name: &str,
    held: bool,
    answered: Answered,
    ardupilot: bool,
    now: Instant,
) {
    let Ok(mut caches) = shared.param_cache.os_lock() else {
        return;
    };
    let Some(cache) = caches.vehicle(id) else {
        return;
    };
    cache.file_types.remove(name);
    if !(answered.set && ardupilot) {
        cache.defaults.remove(name);
    }
    cache.place(name);
    if held || answered.set || answered.read {
        cache.changed(now);
    }
}

/// A stream download finished: `param.Clear()` and `AddRange(newparamlist)`, every entry now the
/// stream's - none with the file's type or a default - and a write queued.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2240-2242`
pub(crate) fn list_fetched(shared: &Shared, id: VehicleId, now: Instant) {
    let Ok(mut caches) = shared.param_cache.os_lock() else {
        return;
    };
    let Some(cache) = caches.vehicle(id) else {
        return;
    };
    cache.file_types.clear();
    cache.defaults.clear();
    cache.changed(now);
}

/// A MAVFTP fetch filled the table: `param.Clear()` and `AddRange(mavlist)` - the list in the
/// file's order, each entry the file's type with the file's default - and a write queued.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1899-1902; ExtLibs/ArduPilot/parampck.cs:100-121`
pub(crate) fn file_filled(shared: &Shared, id: VehicleId, list: &[PackedParam], now: Instant) {
    let Ok(mut caches) = shared.param_cache.os_lock() else {
        return;
    };
    let Some(cache) = caches.vehicle(id) else {
        return;
    };
    cache.places.clear();
    cache.next_place = 0;
    cache.file_types.clear();
    cache.defaults.clear();
    for entry in list {
        cache.place(&entry.name);
        cache
            .file_types
            .insert(entry.name.clone(), entry.value.param_type());
        if let Some(default) = entry.default {
            cache.defaults.insert(entry.name.clone(), default);
        }
    }
    cache.changed(now);
}

/// One pass of the link loop: each vehicle whose write has waited `delay` written, or stopped
/// for good while its board is unknown or its system id is 0. Nothing allocates while nothing is
/// due. `delay` zero writes everything queued, as the link stops.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:76-94`
pub(crate) fn tick(shared: &Shared, registry: &VehicleRegistry, now: Instant, delay: Duration) {
    let mut due: Vec<VehicleId> = Vec::new();
    let directory = {
        let Ok(caches) = shared.param_cache.os_lock() else {
            return;
        };
        if caches.directory.is_none() {
            return;
        }
        for (id, cache) in &caches.vehicles {
            if let Write::Queued(since) = cache.write
                && now.saturating_duration_since(since) >= delay
            {
                due.push(*id);
            }
        }
        if due.is_empty() {
            return;
        }
        caches.directory.clone()
    };
    let Some(directory) = directory else {
        return;
    };
    for id in due {
        // `if (cs.uid2 == null || cs.uid2 == "" || sysid == 0) return;`
        let file = registry
            .working(id)
            .filter(|_| id.sysid != 0)
            .and_then(|state| path_for(&directory, state, id));
        let text = {
            // Lock order: parameters, then the cache.
            let Ok(tables) = shared.params.os_lock() else {
                continue;
            };
            let Ok(mut caches) = shared.param_cache.os_lock() else {
                continue;
            };
            let Some(cache) = caches.vehicles.get_mut(&id) else {
                continue;
            };
            if file.is_none() {
                cache.write = Write::Stopped;
                continue;
            }
            // `queuewrite = false`, after the write there and before it here: the link thread
            // writes, and nothing it hears can come between.
            cache.write = Write::Idle;
            // `lock (this.param) File.WriteAllText(ParamCachePath, param.ToJSON())`.
            to_json(&cache.entries(tables.get(&id)))
        };
        if let Some(file) = file {
            // `Directory.CreateDirectory`, then `File.WriteAllText`; a failure is `log.Error(e)`
            // there and nothing here: the next change writes again.
            if let Some(folder) = file.parent() {
                let _ = mp_os::fs::create_dir_all(folder);
            }
            let _ = mp_os::fs::write(&file, text);
        }
    }
}

impl Link {
    /// `doConnect`'s cached branch: with [`crate::LinkConfig::use_cached_params`] set and the
    /// vehicle's `param.json` there, written less than an hour ago, its parameters added to the
    /// table - a name held replaced, a new one added at its place in the file - and the table
    /// whole at the file's count (`TotalReported = TotalReceived`), in place of the fetch: nothing
    /// is asked of the vehicle. `Ok(Some(count))` then; `Ok(None)` - fetch, as the C#'s `else`
    /// does - with the setting off (the C#'s default, and nothing there sets it), no data
    /// directory, the board not yet known, or no fresh file.
    ///
    /// The rest of the C#'s condition is the caller's: only on connecting with `getparams`, and
    /// never for a [`MAV_COMP_ID_PERIPHERAL`], which is neither read nor fetched. The fetch's
    /// state ([`Link::param_fetch`]) is not touched: the C# reads the file in place of
    /// `getParamList`, not through it.
    /// `// C#: MainV2.cs:1659-1689`
    ///
    /// # Errors
    /// The file would not read, or not as a `MAVLinkParamList`: where the C# throws out of
    /// `doConnect`, whose catch closes the connection with "Can not establish a connection".
    pub fn read_cached_params(&self, target: VehicleId) -> Result<Option<usize>, String> {
        if !self.config.use_cached_params {
            return Ok(None);
        }
        let Some(directory) = self.config.data_directory.as_deref() else {
            return Ok(None);
        };
        let Some(state) = self.vehicle(target).map(|handle| handle.load()) else {
            return Ok(None);
        };
        let Some(file) = path_for(directory, &state, target) else {
            return Ok(None);
        };
        if !is_fresh(&file) {
            return Ok(None);
        }
        let text = mp_os::fs::read_to_string(&file)
            .map_err(|why| format!("{}: {why}", file.display()))?;
        let entries = read(&text)?;
        let count = u16::try_from(entries.len()).unwrap_or(u16::MAX);
        let now = Instant::now();
        let Ok(mut tables) = self.shared.params.os_lock() else {
            return Err("the parameter table is poisoned".to_owned());
        };
        let table = tables.entry(target).or_default();
        // Lock order: parameters, then the cache.
        let mut caches = self.shared.param_cache.os_lock();
        for (index, (name, value)) in entries.into_iter().enumerate() {
            if let Ok(caches) = caches.as_mut()
                && let Some(cache) = caches.vehicle(target)
            {
                // A `MAVLinkParam` from the file: its `Type` its own, and no default.
                cache.file_types.remove(&name);
                cache.defaults.remove(&name);
                cache.place(&name);
                // `Add` of a held name is the indexer's replacement, which raises the event.
                if table.get(&name).is_some() {
                    cache.changed(now);
                }
            }
            table.insert(name, value, u16::try_from(index).unwrap_or(u16::MAX), count);
        }
        Ok(Some(usize::from(count)))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::indexing_slicing)]

    use super::*;

    /// The newline the C# would write on this platform, in a fixture written with `\n`.
    fn native(fixture: &str) -> String {
        fixture.replace('\n', NEWLINE)
    }

    /// Three parameters as `ToJSON` writes them: ArduPilot's float-carried integer and float from
    /// the stream (`Type` 9, no default), and one the MAVFTP file brought, `Type` the file's
    /// `INT16`, with the file's default; members in Json.NET's order, two spaces a level.
    #[test]
    fn the_list_is_written_as_json_net_writes_it() {
        let params = [
            CachedParam::of(
                "ARMING_CHECK",
                ParamValue::from_ardupilot(1.0, ParamType::Int32),
                None,
                None,
            ),
            CachedParam::of(
                "MOT_THST_EXPO",
                ParamValue::from_ardupilot(0.65, ParamType::Real32),
                None,
                None,
            ),
            CachedParam::of(
                "SERVO1_FUNCTION",
                ParamValue::from_ardupilot(33.0, ParamType::Int16),
                Some(ParamType::Int16),
                Some(0.0),
            ),
        ];
        let expected = r#"[
  {
    "default_value": null,
    "Name": "ARMING_CHECK",
    "Type": 9,
    "Value": 1.0,
    "TypeAP": 6
  },
  {
    "default_value": null,
    "Name": "MOT_THST_EXPO",
    "Type": 9,
    "Value": 0.65,
    "TypeAP": 9
  },
  {
    "default_value": 0.0,
    "Name": "SERVO1_FUNCTION",
    "Type": 4,
    "Value": 33.0,
    "TypeAP": 4
  }
]"#;
        assert_eq!(to_json(&params), native(expected));
        // `[]` for an empty list, as Json.NET closes an array with nothing in it.
        assert_eq!(to_json(&[]), "[]");
    }

    /// `JsonConvert.ToString(double)`: .NET Framework's "R" - 15 digits, 17 when 15 do not read
    /// back - with `.0` on a whole number, the exponent form below 10^-4 and from 10^15, and the
    /// three symbols quoted.
    #[test]
    fn numbers_are_written_as_net_framework_writes_them_round_trip() {
        let cases: [(f64, &str); 18] = [
            (0.0, "0.0"),
            (-0.0, "0.0"),
            (1.0, "1.0"),
            (-1.0, "-1.0"),
            (0.1, "0.1"),
            (-0.2, "-0.2"),
            (0.65, "0.65"),
            (3300.0, "3300.0"),
            (4_294_967_295.0, "4294967295.0"),
            (0.0001, "0.0001"),
            (0.00001, "1E-05"),
            (123_456_789_012_345.0, "123456789012345.0"),
            (1e15, "1E+15"),
            (1.5e300, "1.5E+300"),
            // `Convert.ToDouble(0.1f)`, a MAVFTP default: 15 digits do not read back.
            (f64::from(0.1f32), "0.10000000149011612"),
            (1.0 / 3.0, "0.33333333333333331"),
            (f64::NAN, "\"NaN\""),
            (f64::NEG_INFINITY, "\"-Infinity\""),
        ];
        for (value, text) in cases {
            assert_eq!(json_number(value), text, "{value:e}");
        }
        assert_eq!(json_number(f64::INFINITY), "\"Infinity\"");
    }

    /// A float through `GetValue`'s seven significant digits, as the C#'s `Value` is; another
    /// autopilot's integer, its bytes read as their declared type.
    #[test]
    fn a_value_is_what_get_value_reads() {
        let small = CachedParam::of(
            "INS_ACCOFFS_X",
            ParamValue::from_ardupilot(1e-5, ParamType::Real32),
            None,
            None,
        );
        assert_eq!(json_number(small.value), "1E-05");
        let speed = CachedParam::of(
            "WPNAV_SPEED",
            ParamValue::from_ardupilot(0.8, ParamType::Real32),
            None,
            None,
        );
        assert_eq!(json_number(speed.value), "0.8");
        let px4 = CachedParam::of(
            "SYS_AUTOSTART",
            ParamValue::from_wire_bytes(4001i32.to_le_bytes(), ParamType::Int32),
            None,
            None,
        );
        assert_eq!((px4.param_type, px4.type_ap), (6, 6));
        assert_eq!(json_number(px4.value), "4001.0");
    }

    /// Names are quoted as Json.NET quotes them.
    #[test]
    fn a_name_is_escaped_as_json_net_escapes_it() {
        assert_eq!(
            quoted("A\"B\\C\n\u{1f}\u{2028}é"),
            "\"A\\\"B\\\\C\\n\\u001f\\u2028é\""
        );
    }

    /// What the writer writes reads back as the same table entries, with either newline.
    #[test]
    fn the_file_reads_back_as_written() {
        let params = [
            CachedParam::of(
                "BATT_CAPACITY",
                ParamValue::from_ardupilot(3300.0, ParamType::Int32),
                None,
                None,
            ),
            CachedParam::of(
                "MOT_THST_EXPO",
                ParamValue::from_ardupilot(0.65, ParamType::Real32),
                None,
                Some(f64::from(0.5f32)),
            ),
            CachedParam::of(
                "ACRO_OPTIONS",
                ParamValue::from_ardupilot(3.0, ParamType::Int8),
                Some(ParamType::Int8),
                Some(0.0),
            ),
        ];
        for text in [to_json(&params), to_json(&params).replace(NEWLINE, "\r\n")] {
            let entries = read(&text).unwrap();
            assert_eq!(entries.len(), 3);
            assert_eq!(entries[0].0, "BATT_CAPACITY");
            assert_eq!(
                entries[0].1,
                ParamValue::from_ardupilot(3300.0, ParamType::Int32)
            );
            assert_eq!(
                entries[1].1,
                ParamValue::from_ardupilot(0.65, ParamType::Real32)
            );
            // `Type` INT8: the bytes `SetValue((sbyte)3)` makes, read as an INT8.
            assert_eq!(
                entries[2].1,
                ParamValue::from_wire_bytes([3, 0, 0, 0], ParamType::Int8)
            );
            assert_eq!(entries[2].1.as_f64(), 3.0);
        }
    }

    /// Json.NET's way with each object: members in the file's order, names matched exactly or
    /// else ignoring case, `Value` set under the `Type` set before it - after it, zero - the
    /// readonly default and unknown members skipped, a byte-order mark dropped, an enum by name.
    #[test]
    fn each_member_is_set_as_json_net_sets_it() {
        let text = "\u{feff}[{\"name\":\"A\",\"type\":9,\"value\":2.5,\"typeap\":\"REAL32\",\
                    \"default_value\":7,\"Extra\":[1,{\"x\":null}]},\
                    {\"Name\":\"B\",\"Value\":5,\"Type\":6,\"TypeAP\":6},\
                    {\"Name\":\"C\",\"Type\":\"INT16\",\"Value\":\"-3\"},\
                    {\"Name\":\"D\",\"Type\":9,\"Value\":\"NaN\",\"TypeAP\":4},\
                    {\"Name\":null,\"Type\":9,\"Value\":1},\
                    {\"Name\":\"E\",\"Type\":10,\"Value\":1},\
                    {\"Name\":\"F\",\"Type\":9}]";
        let entries = read(text).unwrap();
        let names: Vec<&str> = entries.iter().map(|(name, _)| name.as_str()).collect();
        assert_eq!(
            names,
            ["A", "B", "C", "D", "F"],
            "no name, and REAL64, are not held"
        );
        assert_eq!(
            entries[0].1,
            ParamValue::from_ardupilot(2.5, ParamType::Real32)
        );
        // `Value` before `Type`: `SetValue` under type 0 has no case, the bytes stay zero.
        assert_eq!(
            entries[1].1,
            ParamValue::from_wire_bytes([0; 4], ParamType::Int32)
        );
        // An enum by name, a number in a string, and `TypeAP` unset answering `Type`.
        assert_eq!(
            entries[2].1,
            ParamValue::from_wire_bytes([0xfd, 0xff, 0, 0], ParamType::Int16)
        );
        assert_eq!(entries[2].1.as_f64(), -3.0);
        assert!(entries[3].1.to_param_value_field().is_nan());
        assert_eq!(entries[3].1.param_type(), ParamType::Int16);
        // No `Value`: four zero bytes.
        assert_eq!(entries[4].1.as_f64(), 0.0);
    }

    /// Where Json.NET throws, and so `doConnect` fails, the read is an error.
    #[test]
    fn what_json_net_will_not_read_is_an_error() {
        for text in [
            "",
            "null",
            "{}",
            "[null]",
            "[1]",
            "[{\"Name\":\"A\",\"Type\":9,\"Value\":null}]",
            "[{\"Name\":\"A\",\"Type\":300}]",
            "[{\"Name\":\"A\",\"Type\":\"WIDE\"}]",
            "[{\"Name\":[],\"Type\":9}]",
            "[] []",
            "[{\"Name\":\"A\" \"Type\":9}]",
            "[{\"Name\":\"A\\q\"}]",
        ] {
            assert!(read(text).is_err(), "{text:?} read");
        }
        assert_eq!(read("[]").unwrap(), Vec::new());
        assert_eq!(read(" [ ] \r\n").unwrap(), Vec::new());
    }

    /// `ParamCachePath`, piece by piece: the type's name in the C#'s enum, or its number; the
    /// board's id as lowercase hex.
    #[test]
    fn the_path_is_the_csharps() {
        let mut uid2 = [0u8; 18];
        for (n, byte) in uid2.iter_mut().enumerate() {
            *byte = u8::try_from(n).unwrap() * 15;
        }
        let file = path(Path::new("data"), 2, &uid2, VehicleId::new(1, 1));
        assert_eq!(
            file,
            Path::new("data")
                .join("paramcache")
                .join("QUADROTOR")
                .join("000f1e2d3c4b5a69788796a5b4c3d2e1f0ff")
                .join("1")
                .join("1")
                .join("param.json")
        );
        assert_eq!(aptype_name(0), "GENERIC");
        assert_eq!(aptype_name(1), "FIXED_WING");
        assert_eq!(aptype_name(10), "GROUND_ROVER");
        assert_eq!(aptype_name(48), "GRIPPER");
        assert_eq!(aptype_name(49), "49");
    }

    /// The list's order: names in the order they were added; a parameter the MAVFTP file brought
    /// has the file's type and default.
    #[test]
    fn the_list_keeps_its_order_and_the_files_types() {
        let mut cache = VehicleCache::default();
        let mut table = ParamTable::new();
        for (index, name) in ["ZETA", "ALPHA", "MID"].into_iter().enumerate() {
            cache.place(name);
            table.insert(
                name.to_owned(),
                ParamValue::from_ardupilot(1.0, ParamType::Int8),
                u16::try_from(index).unwrap(),
                3,
            );
        }
        let names: Vec<String> = cache
            .entries(Some(&table))
            .into_iter()
            .map(|param| param.name)
            .collect();
        assert_eq!(names, ["ZETA", "ALPHA", "MID"]);

        cache.file_types.insert("ALPHA".to_owned(), ParamType::Int8);
        cache.defaults.insert("ALPHA".to_owned(), 4.0);
        let alpha = &cache.entries(Some(&table))[1];
        assert_eq!((alpha.param_type, alpha.type_ap), (2, 2));
        assert_eq!((alpha.value, alpha.default_value), (1.0, Some(4.0)));
        assert_eq!(cache.entries(None), Vec::new());
    }
}
