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

//! The JSON mission: QGroundControl's `.plan`, which Load File reads when a file's first line
//! starts with `{`, and the `.mission` Save File writes under its "Mission JSON" filter.
//!
//! Ported from `ExtLibs/Utilities/MissionFile.cs` @ efb0801 (GPL-3.0-only): `ReadFile` and
//! `ConvertToLocationwps` are [`read`], `ConvertFromLocationwps` and `WriteFile` are [`write`].
//! The C#'s model is Newtonsoft's, classes from json2csharp; here it is read from and written to
//! a `serde_json::Value` in the classes' declaration order.
//!
//! ```text
//! {
//!   "fileType": null,
//!   "geoFence": null,
//!   "groundStation": "MissionPlanner",
//!   "mission": {
//!     "cruiseSpeed": 0,  "firmwareType": 0,  "hoverSpeed": 0,
//!     "items": [ { "autoContinue": false, "command": 16, "doJumpId": 0, "frame": 3,
//!                  "params": [p1, p2, p3, p4, lat, lng, alt], "type": null, ... } ],
//!     "plannedHomePosition": [lat, lng, alt],
//!     "vehicleType": 0,  "version": 0
//!   },
//!   "rallyPoints": null,
//!   "version": 1
//! }
//! ```
//!
//! Mission Planner does not read back what it writes: its items carry no `"type"`, and
//! `ConvertToLocationwps` takes only a `"SimpleItem"` or a `"ComplexItem"`, so a `.mission`
//! loads as its home alone. That is the C#'s behaviour and is kept.

use serde_json::{Map, Value, json};

use crate::item::MissionItem;
use crate::rows::Home;

/// Why a JSON mission could not be read: what the C#'s `ReadFile` or `ConvertToLocationwps`
/// throws, by the exception's name.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum MissionFileError {
    /// Not JSON, or JSON that does not fit the model: `JsonConvert.DeserializeObject` throws.
    #[error("Newtonsoft.Json.JsonSerializationException: {0}")]
    Json(String),
    /// A part `ConvertToLocationwps` reaches is not there.
    #[error("System.NullReferenceException: {0} is null")]
    Missing(&'static str),
    /// An item's `params`, or the planned home, has fewer entries than are read from it.
    #[error("System.ArgumentOutOfRangeException: {0}")]
    Short(String),
}

/// `ReadFile` then `ConvertToLocationwps`: home at item 0, from `plannedHomePosition`, then each
/// `"SimpleItem"`, and each item of a `"ComplexItem"`'s `TransectStyleComplexItem`; any other
/// type is passed over.
/// `// C#: ExtLibs/Utilities/MissionFile.cs:88-139`
pub fn read(text: &str) -> Result<Vec<MissionItem>, MissionFileError> {
    let root: Value =
        serde_json::from_str(text).map_err(|why| MissionFileError::Json(why.to_string()))?;
    if !root.is_object() {
        return Err(MissionFileError::Json(format!(
            "a mission is an object, not {}",
            kind(&root)
        )));
    }
    let mission = present(&root, "mission", "format.mission")?;
    let home = list(
        mission,
        "plannedHomePosition",
        "mission.plannedHomePosition",
    )?;
    let mut cmds = vec![from_home(home)?];
    for item in list(mission, "items", "mission.items")? {
        match field(item, "type").and_then(Value::as_str) {
            Some("SimpleItem") => cmds.push(from_item(item)?),
            Some("ComplexItem") => {
                let transect = present(
                    item,
                    "TransectStyleComplexItem",
                    "missionItem.TransectStyleComplexItem",
                )?;
                for inner in list(transect, "Items", "TransectStyleComplexItem.Items")? {
                    cmds.push(from_item(inner)?);
                }
            }
            _ => {}
        }
    }
    Ok(cmds)
}

/// `ConvertFromLocationwps(list, frame)` then `WriteFile`, for Save File's `.mission`: `list` is
/// home from the Home Location boxes inserted before the grid's rows (`GetCommandList`).
///
/// Home gives `plannedHomePosition`, its latitude, longitude and altitude; each row an item of
/// its command and its seven parameters, with `frame` - the screen's altitude frame - in place of
/// its own. `doJumpId` is `Locationwp._seq`, which `GetCommandList` leaves at 0, so every item has
/// 0. A row's four parameters and altitude are `float`s in a `Locationwp` and are written as the
/// `double`s those widen to.
///
/// Newtonsoft's indented text: two spaces, every null written. Two differences, neither in what a
/// reader takes from it: lines end `\n` everywhere (Newtonsoft ends them with the platform's
/// newline), and a number too small or large for plain digits is written `1e-5` where .NET's
/// round-trip format writes `1E-05`.
/// `// C#: ExtLibs/Utilities/MissionFile.cs:99-104, 241-285; ExtLibs/Utilities/locationwp.cs:139-151;
/// GCSViews/FlightPlanner.cs:3900-3912, 6084-6104`
#[must_use]
pub fn write(home: Home, rows: &[MissionItem], frame: u8) -> String {
    let items: Vec<Value> = rows
        .iter()
        .map(|row| {
            json!({
                "autoContinue": false,
                "command": row.command,
                "doJumpId": 0,
                "frame": frame,
                "params": [
                    widened(row.param1),
                    widened(row.param2),
                    widened(row.param3),
                    widened(row.param4),
                    row.x,
                    row.y,
                    widened(row.z),
                ],
                "type": null,
                "TransectStyleComplexItem": null,
                "angle": null,
                "complexItemType": null,
                "entryLocation": null,
                "flyAlternateTransects": null,
                "polygon": null,
                "splitConcavePolygons": null,
                "version": null,
            })
        })
        .collect();
    let root = json!({
        "fileType": null,
        "geoFence": null,
        "groundStation": "MissionPlanner",
        "mission": {
            "cruiseSpeed": 0,
            "firmwareType": 0,
            "hoverSpeed": 0,
            "items": items,
            "plannedHomePosition": [home.lat, home.lng, widened(home.alt)],
            "vehicleType": 0,
            "version": 0,
        },
        "rallyPoints": null,
        "version": 1,
    });
    serde_json::to_string_pretty(&root).unwrap_or_default()
}

/// A `float` field of `Locationwp` as `(double?)` gives it to the JSON.
#[allow(clippy::cast_possible_truncation)]
fn widened(value: f64) -> f64 {
    f64::from(value as f32)
}

/// `ConvertFromMissionItem(List<double>)`: home, `{alt = (float)[2], lat = [0], lng = [1]}`.
/// `// C#: ExtLibs/Utilities/MissionFile.cs:131-134`
fn from_home(position: &[Value]) -> Result<MissionItem, MissionFileError> {
    let at = |index: usize| -> Result<f64, MissionFileError> {
        let value = position.get(index).ok_or_else(|| {
            MissionFileError::Short(format!(
                "plannedHomePosition has {} entries, and entry {index} is read",
                position.len()
            ))
        })?;
        double(value, "plannedHomePosition")
    };
    let (lat, lng, alt) = (at(0)?, at(1)?, at(2)?);
    Ok(MissionItem {
        x: lat,
        y: lng,
        z: widened(alt),
        ..blank()
    })
}

/// `implicit operator Locationwp(MissionFile.Item)`: the command, the four parameters (`float`s)
/// and latitude, longitude and altitude from `params`, a null among them 0, `_seq` from
/// `doJumpId`, and the frame. `(ushort)` and `(byte)` are C#'s unchecked casts, which wrap.
/// `// C#: ExtLibs/Utilities/locationwp.cs:121-137`
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn from_item(item: &Value) -> Result<MissionItem, MissionFileError> {
    let params = list(item, "params", "missionItem.params")?;
    let param = |index: usize| -> Result<f64, MissionFileError> {
        let value = params.get(index).ok_or_else(|| {
            MissionFileError::Short(format!(
                "params has {} entries, and entry {index} is read",
                params.len()
            ))
        })?;
        if value.is_null() {
            Ok(0.0)
        } else {
            double(value, "params")
        }
    };
    Ok(MissionItem {
        command: int(item, "command")? as u16,
        param1: widened(param(0)?),
        param2: widened(param(1)?),
        param3: widened(param(2)?),
        param4: widened(param(3)?),
        x: param(4)?,
        y: param(5)?,
        z: widened(param(6)?),
        seq: int(item, "doJumpId")? as u16,
        frame: int(item, "frame")? as u8,
        ..blank()
    })
}

/// `new Locationwp()`, as a mission item: everything 0, and continuing to the next item as the
/// grid's rows do.
fn blank() -> MissionItem {
    MissionItem {
        seq: 0,
        current: 0,
        frame: 0,
        command: 0,
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
        autocontinue: 1,
    }
}

/// A property, as Newtonsoft matches one to a class's: by its exact name, else ignoring case.
fn field<'a>(object: &'a Value, name: &str) -> Option<&'a Value> {
    let map: &Map<String, Value> = object.as_object()?;
    map.get(name).or_else(|| {
        map.iter()
            .find(|(key, _)| key.eq_ignore_ascii_case(name))
            .map(|(_, value)| value)
    })
}

/// A property that is an object, or the null reference the C# would follow.
fn present<'a>(
    object: &'a Value,
    name: &str,
    what: &'static str,
) -> Result<&'a Value, MissionFileError> {
    match field(object, name) {
        None | Some(Value::Null) => Err(MissionFileError::Missing(what)),
        Some(value) if value.is_object() => Ok(value),
        Some(value) => Err(MissionFileError::Json(format!(
            "{what} is an object, not {}",
            kind(value)
        ))),
    }
}

/// A property that is a list, or the null reference the C# would follow.
fn list<'a>(
    object: &'a Value,
    name: &str,
    what: &'static str,
) -> Result<&'a [Value], MissionFileError> {
    match field(object, name) {
        None | Some(Value::Null) => Err(MissionFileError::Missing(what)),
        Some(Value::Array(values)) => Ok(values),
        Some(value) => Err(MissionFileError::Json(format!(
            "{what} is a list, not {}",
            kind(value)
        ))),
    }
}

/// A `double`.
fn double(value: &Value, what: &str) -> Result<f64, MissionFileError> {
    value.as_f64().ok_or_else(|| {
        MissionFileError::Json(format!("{what} holds {}, not a number", kind(value)))
    })
}

/// An `int` property: 0 when the file leaves it out, as a class's field stays at its default;
/// refused when it is null, fractional, or outside `Int32`, as Newtonsoft refuses it.
fn int(item: &Value, name: &str) -> Result<i32, MissionFileError> {
    let Some(value) = field(item, name) else {
        return Ok(0);
    };
    let whole = value
        .as_i64()
        .or_else(|| {
            value
                .as_f64()
                .filter(|number| number.fract() == 0.0)
                .and_then(|number| format!("{number:.0}").parse().ok())
        })
        .ok_or_else(|| {
            MissionFileError::Json(format!(
                "Error converting value {value} to type 'System.Int32' ({name})"
            ))
        })?;
    i32::try_from(whole).map_err(|_| {
        MissionFileError::Json(format!(
            "Value {whole} was either too large or too small for an Int32 ({name})"
        ))
    })
}

/// What a JSON value is, for an error.
const fn kind(value: &Value) -> &'static str {
    match value {
        Value::Null => "null",
        Value::Bool(_) => "a boolean",
        Value::Number(_) => "a number",
        Value::String(_) => "a string",
        Value::Array(_) => "a list",
        Value::Object(_) => "an object",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A plan as QGroundControl saves one: home, a takeoff, a waypoint with a null parameter, a
    /// survey whose transect holds two waypoints, and a fence and rally points that are not read.
    /// (`testdata/missions/qgc_survey.plan`, which tests/gui/plan-file-json.gui loads too.)
    const QGC_PLAN: &str = include_str!("../../../testdata/missions/qgc_survey.plan");

    #[test]
    fn a_qgc_plan_reads_as_home_then_its_simple_and_transect_items() {
        let cmds = read(QGC_PLAN).unwrap();
        assert_eq!(cmds.len(), 5, "home, two simple items, the survey's two");
        // Home: plannedHomePosition, the altitude through a float.
        assert_eq!(cmds[0].command, 0);
        assert_eq!(cmds[0].frame, 0);
        assert_eq!((cmds[0].x, cmds[0].y), (-35.363_262_1, 149.165_237_4));
        assert_eq!(cmds[0].z, f64::from(584.19_f32));
        // The takeoff: its command, pitch, a null fourth parameter as 0, and its frame.
        assert_eq!(cmds[1].command, 22);
        assert_eq!(cmds[1].param1, 15.0);
        assert_eq!(cmds[1].param4, 0.0);
        assert_eq!(cmds[1].frame, 3);
        assert_eq!(cmds[1].seq, 1, "doJumpId is _seq");
        assert_eq!(cmds[2].z, 60.5);
        // The survey's transect, in its order; the unknown type is passed over.
        assert_eq!((cmds[3].x, cmds[3].seq), (-35.3620, 3));
        assert_eq!((cmds[4].x, cmds[4].seq), (-35.3615, 4));
    }

    #[test]
    fn what_is_not_there_is_the_exception_the_c_sharp_throws() {
        assert!(matches!(read("{ not json"), Err(MissionFileError::Json(_))));
        assert_eq!(
            read(r#"{"version": 1}"#),
            Err(MissionFileError::Missing("format.mission"))
        );
        assert_eq!(
            read(r#"{"mission": {"items": []}}"#),
            Err(MissionFileError::Missing("mission.plannedHomePosition"))
        );
        assert!(matches!(
            read(r#"{"mission": {"items": [], "plannedHomePosition": [1, 2]}}"#),
            Err(MissionFileError::Short(_))
        ));
        assert!(matches!(
            read(
                r#"{"mission": {"plannedHomePosition": [1, 2, 3],
                    "items": [{"type": "SimpleItem", "command": 16, "params": [0, 0, 0, 0, 1, 2]}]}}"#
            ),
            Err(MissionFileError::Short(_))
        ));
        assert!(matches!(
            read(
                r#"{"mission": {"plannedHomePosition": [1, 2, 3],
                    "items": [{"type": "SimpleItem", "command": 16.5, "params": [0, 0, 0, 0, 1, 2, 3]}]}}"#
            ),
            Err(MissionFileError::Json(_))
        ));
        // A home and no items is a mission of home alone; names match ignoring case.
        let cmds = read(r#"{"Mission": {"Items": [], "PlannedHomePosition": [1, 2, 3]}}"#).unwrap();
        assert_eq!(cmds.len(), 1);
    }

    fn row(command: u16, param1: f64, x: f64, y: f64, z: f64, frame: u8) -> MissionItem {
        MissionItem {
            command,
            param1,
            x,
            y,
            z,
            frame,
            seq: 7,
            ..blank()
        }
    }

    #[test]
    fn a_mission_is_written_as_newtonsoft_indents_convert_from_locationwps() {
        let home = Home {
            lat: -35.363_262_1,
            lng: 149.165_237_4,
            alt: 584.0,
        };
        let rows = [
            row(22, 15.0, 0.0, 0.0, 50.0, 3),
            row(16, 0.1, -35.3625, 149.166, 100.0, 10),
        ];
        let text = write(home, &rows, 0);
        let expected = r#"{
  "fileType": null,
  "geoFence": null,
  "groundStation": "MissionPlanner",
  "mission": {
    "cruiseSpeed": 0,
    "firmwareType": 0,
    "hoverSpeed": 0,
    "items": [
      {
        "autoContinue": false,
        "command": 22,
        "doJumpId": 0,
        "frame": 0,
        "params": [
          15.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          50.0
        ],
        "type": null,
        "TransectStyleComplexItem": null,
        "angle": null,
        "complexItemType": null,
        "entryLocation": null,
        "flyAlternateTransects": null,
        "polygon": null,
        "splitConcavePolygons": null,
        "version": null
      },
      {
        "autoContinue": false,
        "command": 16,
        "doJumpId": 0,
        "frame": 0,
        "params": [
          0.10000000149011612,
          0.0,
          0.0,
          0.0,
          -35.3625,
          149.166,
          100.0
        ],
        "type": null,
        "TransectStyleComplexItem": null,
        "angle": null,
        "complexItemType": null,
        "entryLocation": null,
        "flyAlternateTransects": null,
        "polygon": null,
        "splitConcavePolygons": null,
        "version": null
      }
    ],
    "plannedHomePosition": [
      -35.3632621,
      149.1652374,
      584.0
    ],
    "vehicleType": 0,
    "version": 0
  },
  "rallyPoints": null,
  "version": 1
}"#;
        assert_eq!(text, expected);
    }

    #[test]
    fn mission_planner_reads_its_own_mission_back_as_home_alone() {
        let home = Home {
            lat: -35.0,
            lng: 149.0,
            alt: 10.0,
        };
        let text = write(home, &[row(16, 0.0, -35.1, 149.1, 20.0, 3)], 3);
        let cmds = read(&text).unwrap();
        assert_eq!(cmds.len(), 1, "its items have no type, so none is read");
        assert_eq!((cmds[0].x, cmds[0].y, cmds[0].z), (-35.0, 149.0, 10.0));
        // An empty grid writes an empty list.
        assert!(write(home, &[], 3).contains("\"items\": [],"));
    }
}
