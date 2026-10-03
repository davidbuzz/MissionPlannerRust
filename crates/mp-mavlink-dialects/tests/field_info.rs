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

//! Each message's `FIELD_INFO`, the fields' XML types and units, beside the `fields()` it
//! describes: the MAVLink Inspector shows a field's type from it and titles Graph It's axis with
//! its units (`crates/mp-gui/src/config/mavlink_inspector.rs`), so a table that fell out of step
//! with the values would put one field's type beside another's value.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use mp_mavlink::{FieldValue, Message as _};
use mp_mavlink_dialects::all::{Attitude, DIALECT, Heartbeat, MavMessage};

/// Every message: the same names in the same order, an array declared where the value is one,
/// of its declared length, and a float declared where the value is one.
#[test]
fn every_message_s_field_info_describes_its_fields() {
    let mut fields = 0;
    for described in DIALECT.messages() {
        let message = MavMessage::decode(described.id, &[]).expect("decodes");
        let values = message.fields();
        let infos = message.field_info();
        assert_eq!(values.len(), infos.len(), "{}", described.name);
        for ((name, value), info) in values.iter().zip(infos) {
            assert_eq!(*name, info.name, "{}", described.name);
            let (array, float) = match value {
                FieldValue::Unsigned(_) | FieldValue::Signed(_) => (None, false),
                FieldValue::Float(_) => (None, true),
                FieldValue::UnsignedArray(v) => (Some(v.len()), false),
                FieldValue::SignedArray(v) => (Some(v.len()), false),
                FieldValue::FloatArray(v) => (Some(v.len()), true),
            };
            assert_eq!(
                array.unwrap_or(0),
                info.array_len,
                "{}.{}",
                described.name,
                info.name
            );
            assert_eq!(
                float,
                matches!(info.base_type, "float" | "double"),
                "{}.{}",
                described.name,
                info.name
            );
            fields += 1;
        }
    }
    assert!(fields > 2000, "{fields} fields");
}

/// The XML's words: `ATTITUDE`'s angles in `rad`, its time in `ms`, `HEARTBEAT`'s version the
/// pseudo-type mavgen reads as `uint8_t`, `PARAM_VALUE`'s name sixteen `char`s.
#[test]
fn the_xml_s_types_and_units() {
    let roll = Attitude::FIELD_INFO
        .iter()
        .find(|info| info.name == "roll")
        .expect("roll");
    assert_eq!(
        (roll.base_type, roll.array_len, roll.units),
        ("float", 0, "rad")
    );
    let time = Attitude::FIELD_INFO[0];
    assert_eq!(
        (time.name, time.base_type, time.units),
        ("time_boot_ms", "uint32_t", "ms")
    );
    let version = Heartbeat::FIELD_INFO.last().expect("a field");
    assert_eq!(
        (version.name, version.base_type, version.units),
        ("mavlink_version", "uint8_t_mavlink_version", "")
    );
    let param = MavMessage::decode(22, &[]).expect("PARAM_VALUE");
    let id = param
        .field_info()
        .iter()
        .find(|info| info.name == "param_id")
        .expect("param_id");
    assert_eq!((id.base_type, id.array_len), ("char", 16));
    assert_eq!(Attitude::ID, 30);
}
