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

//! `Plugins/OpenDroneID2/`, "Open Drone ID", one of the C#'s four real plugins: the Drone ID tab
//! of the flight screen - the aircraft's UAS ID and type, the self ID, the operator ID - and the
//! backend that sends them to the vehicle's Remote ID transmitter as `OPEN_DRONE_ID_*` messages
//! (`OpenDroneID_Plugin.cs`, `OpenDroneID_UI.cs`, `OpenDroneID_Backend.cs`).
//!
//! The tab is the plugin's form, drawn by the host. The backend's messages are built here to the
//! byte and sent through `send-packet`, `comPort.sendPacket`'s place.
//!
//! What cannot run in this world, recorded: the backend starts when the first
//! `OPEN_DRONE_ID_ARM_STATUS` arrives from the vehicle's transmitter, which the C# hears through
//! `comPort.SubscribeToPacketType` - the world has no packet subscription, so `hasODID` stays
//! false, and so do the C#'s checks behind it; the operator's position comes from an NMEA GPS on
//! a serial port of the plugin's own (`NMEA_GPS_Connection`) - unimplementable, so the GCS GPS
//! stays invalid; the status indicator it adds to the flight map's controls; the developer
//! double-click that stops sending.

use std::sync::{Mutex, PoisonError};

use mp_plugins::host::{self, ControlKind, MessageButtons};
use mp_plugins::{Guest, control};

/// `MAV_ODID_ID_TYPE`. `// C#: ExtLibs/Mavlink/Mavlink.cs:5181-5200`
const ID_TYPES: &[&str] = &[
    "NONE",
    "SERIAL_NUMBER",
    "CAA_REGISTRATION_ID",
    "UTM_ASSIGNED_UUID",
    "SPECIFIC_SESSION_ID",
];

/// `MAV_ODID_UA_TYPE`. `// C#: ExtLibs/Mavlink/Mavlink.cs:5202-5236`
const UA_TYPES: &[&str] = &[
    "NONE",
    "AEROPLANE",
    "HELICOPTER_OR_MULTIROTOR",
    "GYROPLANE",
    "HYBRID_LIFT",
    "ORNITHOPTER",
    "GLIDER",
    "KITE",
    "FREE_BALLOON",
    "CAPTIVE_BALLOON",
    "AIRSHIP",
    "FREE_FALL_PARACHUTE",
    "ROCKET",
    "TETHERED_POWERED_AIRCRAFT",
    "GROUND_OBSTACLE",
    "OTHER",
];

/// `MAV_ODID_DESC_TYPE`. `// C#: ExtLibs/Mavlink/Mavlink.cs:5460-5468`
const DESC_TYPES: &[&str] = &["TEXT", "EMERGENCY", "EXTENDED_STATUS"];

/// `MAV_ODID_OPERATOR_ID_TYPE`. `// C#: ExtLibs/Mavlink/Mavlink.cs:5550-5554`
const OPERATOR_ID_TYPES: &[&str] = &["CAA"];

const OPEN_DRONE_ID_BASIC_ID: u32 = 12900;
const OPEN_DRONE_ID_SELF_ID: u32 = 12903;
const OPEN_DRONE_ID_SYSTEM: u32 = 12904;
const OPEN_DRONE_ID_OPERATOR_ID: u32 = 12905;
const OPEN_DRONE_ID_SYSTEM_UPDATE: u32 = 12919;

/// `MAV_COMP_ID_ALL`: the backend always addresses every component.
const MAV_COMP_ID_ALL: u8 = 0;

/// `OpenDroneID_Backend`'s fields. `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:17-56`
#[derive(Debug, Clone, PartialEq)]
struct Backend {
    target_system: u8,
    uas_id_type: u8,
    uas_id: String,
    ua_type: u8,
    description_type: u8,
    description: String,
    area_count: u16,
    area_radius: u16,
    area_ceiling: f32,
    area_floor: f32,
    category_eu: u8,
    class_eu: u8,
    classification_type: u8,
    operator_location_type: u8,
    operator_id_type: u8,
    operator_id: String,
    operator_latitude: f64,
    operator_longitude: f64,
    operator_altitude_geo: f32,
    /// `since_last_msg_ms`: how old the operator's position is; no GPS, so forever.
    since_last_msg_ms: u64,
    /// `count`: which of the four extended messages goes next.
    count: u32,
}

/// `_gps_timeout_ms`.
const GPS_TIMEOUT_MS: u64 = 5000;

/// `MakeBytesSize`: the string's bytes, cut or padded with zeros to `size`.
fn bytes_sized(text: &str, size: usize) -> Vec<u8> {
    let mut out: Vec<u8> = text.bytes().take(size).collect();
    out.resize(size, 0);
    out
}

/// `id_or_mac`: twenty spaces. `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:208-211`
fn id_or_mac() -> Vec<u8> {
    vec![b' '; 20]
}

/// `(int)(degrees * 1.0e7)`.
#[allow(clippy::cast_possible_truncation)]
fn e7(degrees: f64) -> i32 {
    (degrees * 1.0e7) as i32
}

impl Backend {
    const fn new() -> Self {
        Self {
            target_system: 0,
            uas_id_type: 0,
            uas_id: String::new(),
            ua_type: 0,
            description_type: 0,
            description: String::new(),
            area_count: 1,
            area_radius: 0,
            area_ceiling: -1000.0,
            area_floor: -1000.0,
            category_eu: 0,
            class_eu: 0,
            classification_type: 0,
            operator_location_type: 0,
            operator_id_type: 0,
            operator_id: String::new(),
            operator_latitude: 0.0,
            operator_longitude: 0.0,
            operator_altitude_geo: 0.0,
            since_last_msg_ms: u64::MAX,
            count: 0,
        }
    }

    fn gps_fresh(&self) -> bool {
        self.since_last_msg_ms < GPS_TIMEOUT_MS
    }

    /// `send_basic_id`. `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:136-147`
    fn basic_id(&self) -> Vec<u8> {
        let mut out = vec![self.target_system, MAV_COMP_ID_ALL];
        out.extend(id_or_mac());
        out.push(self.uas_id_type);
        out.push(self.ua_type);
        out.extend(bytes_sized(&self.uas_id, 20));
        out
    }

    /// `send_id_system`: zeros for the operator's position when the GPS has timed out.
    /// `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:149-169`
    fn system(&self, timestamp: u32) -> Vec<u8> {
        let fresh = self.gps_fresh();
        let mut out = Vec::new();
        out.extend(if fresh { e7(self.operator_latitude) } else { 0 }.to_le_bytes());
        out.extend(
            if fresh {
                e7(self.operator_longitude)
            } else {
                0
            }
            .to_le_bytes(),
        );
        out.extend(self.area_ceiling.to_le_bytes());
        out.extend(self.area_floor.to_le_bytes());
        out.extend(
            if fresh {
                self.operator_altitude_geo
            } else {
                0.0
            }
            .to_le_bytes(),
        );
        out.extend(timestamp.to_le_bytes());
        out.extend(self.area_count.to_le_bytes());
        out.extend(self.area_radius.to_le_bytes());
        out.push(self.target_system);
        out.push(MAV_COMP_ID_ALL);
        out.extend(id_or_mac());
        out.push(self.operator_location_type);
        out.push(self.classification_type);
        out.push(self.category_eu);
        out.push(self.class_eu);
        out
    }

    /// `send_system_update`. `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:171-181`
    fn system_update(&self, timestamp: u32) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend(e7(self.operator_latitude).to_le_bytes());
        out.extend(e7(self.operator_longitude).to_le_bytes());
        out.extend(self.operator_altitude_geo.to_le_bytes());
        out.extend(timestamp.to_le_bytes());
        out.push(self.target_system);
        out.push(MAV_COMP_ID_ALL);
        out
    }

    /// `send_self_id`. `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:183-191`
    fn self_id(&self) -> Vec<u8> {
        let mut out = vec![self.target_system, MAV_COMP_ID_ALL];
        out.extend(id_or_mac());
        out.push(self.description_type);
        out.extend(bytes_sized(&self.description, 23));
        out
    }

    /// `send_operator_id`. `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:198-206`
    fn operator_id(&self) -> Vec<u8> {
        let mut out = vec![self.target_system, MAV_COMP_ID_ALL];
        out.extend(id_or_mac());
        out.push(self.operator_id_type);
        out.extend(bytes_sized(&self.operator_id, 20));
        out
    }

    /// `Send`: the operator's position when it is fresh, else the next of the four extended
    /// messages in turn. Here at `Loop`'s rate, where the C# has a 100 ms timer and its own
    /// clock checks. `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:75-134`
    fn next(&mut self, timestamp: u32) -> (u32, Vec<u8>) {
        if self.gps_fresh() {
            return (OPEN_DRONE_ID_SYSTEM_UPDATE, self.system_update(timestamp));
        }
        let turn = self.count % 4;
        self.count = self.count.wrapping_add(1);
        match turn {
            0 => (OPEN_DRONE_ID_BASIC_ID, self.basic_id()),
            1 => (OPEN_DRONE_ID_SYSTEM, self.system(timestamp)),
            2 => (OPEN_DRONE_ID_SELF_ID, self.self_id()),
            _ => (OPEN_DRONE_ID_OPERATOR_ID, self.operator_id()),
        }
    }
}

/// The UI's fields. `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:19-48`
struct Ui {
    uas_id: String,
    uas_id_type: usize,
    ua_type: usize,
    self_id_type: usize,
    self_id: String,
    operator_id_type: usize,
    operator_id: String,
    /// `hasODID`: set by the first ARM_STATUS, which cannot arrive here (see the header).
    has_odid: bool,
    status: String,
    backend: Backend,
}

static UI: Mutex<Ui> = Mutex::new(Ui {
    uas_id: String::new(),
    uas_id_type: 0,
    ua_type: 0,
    self_id_type: 0,
    self_id: String::new(),
    operator_id_type: 0,
    operator_id: String::new(),
    has_odid: false,
    status: String::new(),
    backend: Backend::new(),
});

/// The index of `value` in `options`, or where it was.
fn pick(options: &[&str], value: &str, was: usize) -> usize {
    options
        .iter()
        .position(|option| *option == value)
        .unwrap_or(was)
}

/// The Drone ID tab, as the form shows it: the UAS ID and Operations tabs' fields and the status
/// line. `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.Designer.cs:92-422`
fn show(ui: &Ui) {
    let at = |options: &'static [&'static str], index: usize| -> &'static str {
        options.get(index).copied().unwrap_or("")
    };
    host::form_show(
        "Drone ID",
        &[
            control("version", ControlKind::Label, "", "Ver: 0.05", &[]),
            control("uas_id", ControlKind::TextBox, "UAS ID", &ui.uas_id, &[]),
            control(
                "uas_id_type",
                ControlKind::ComboBox,
                "UAS ID Type",
                at(ID_TYPES, ui.uas_id_type),
                ID_TYPES,
            ),
            control(
                "ua_type",
                ControlKind::ComboBox,
                "UA Type",
                at(UA_TYPES, ui.ua_type),
                UA_TYPES,
            ),
            control(
                "self_id_type",
                ControlKind::ComboBox,
                "Self ID Type",
                at(DESC_TYPES, ui.self_id_type),
                DESC_TYPES,
            ),
            control(
                "self_id",
                ControlKind::TextBox,
                "Self ID Desc",
                &ui.self_id,
                &[],
            ),
            control(
                "operator_id_type",
                ControlKind::ComboBox,
                "Oper. ID Type",
                at(OPERATOR_ID_TYPES, ui.operator_id_type),
                OPERATOR_ID_TYPES,
            ),
            control(
                "operator_id",
                ControlKind::TextBox,
                "Operator ID",
                &ui.operator_id,
                &[],
            ),
            control(
                "status",
                ControlKind::Label,
                "RID Armed Status",
                &ui.status,
                &[],
            ),
        ],
    );
}

/// `checkUID`: the form's fields into the backend, as the C# takes them.
/// `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:264-297`
#[allow(clippy::cast_possible_truncation)]
fn check_uid(ui: &mut Ui) {
    let uas_id = !ui.uas_id.is_empty() && ui.uas_id_type > 0 && ui.ua_type > 0;
    if uas_id {
        ui.backend.uas_id.clone_from(&ui.uas_id);
        ui.backend.ua_type = ui.ua_type as u8;
        ui.backend.uas_id_type = ui.uas_id_type as u8;
    }
    if !ui.self_id.is_empty() {
        ui.backend.description.clone_from(&ui.self_id);
        ui.backend.description_type = ui.self_id_type as u8;
    }
    if !ui.operator_id.is_empty() {
        ui.backend.operator_id.clone_from(&ui.operator_id);
        ui.backend.operator_id_type = ui.operator_id_type as u8;
    }
}

struct OpenDroneId;

impl Guest for OpenDroneId {
    fn name() -> String {
        "Open Drone ID".to_owned()
    }

    fn version() -> String {
        "0.05".to_owned()
    }

    fn author() -> String {
        "Steven Borenstein, updated by BlueMark Innovations BV".to_owned()
    }

    /// The UI thread's fast rate, `_update_rate_hz_1`. `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:44`
    fn loop_rate_hz() -> f32 {
        10.0
    }

    fn init() -> bool {
        true
    }

    // `Loaded` and `forceSettings`: without `tabcontrolactions` the C# asks for a restart once,
    // then saves the flight screen's tabs as the setting, so the next start has it and does not
    // ask (the owner's bug, 2026-10-04: the message came at every start).
    // `// C#: Plugins/OpenDroneID2/OpenDroneID_Plugin.cs:40-82`
    fn loaded() -> bool {
        if host::config_get("tabcontrolactions").is_none() {
            let _ = host::message_box(
                "Restart Mission Planner to enable Drone ID Tab. Disable Plugin if Not Required CTRL-P",
                "",
                MessageButtons::Ok,
            );
            host::save_tab_control_actions();
        }
        show(&UI.lock().unwrap_or_else(PoisonError::into_inner));
        true
    }

    // `mainloop`'s fast half: the checks run once the transmitter has been heard, and the
    // backend sends. `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:370-417`
    fn run_loop() -> bool {
        let mut ui = UI.lock().unwrap_or_else(PoisonError::into_inner);
        if ui.has_odid {
            check_uid(&mut ui);
            let (id, payload) = ui.backend.next(0);
            let _ = host::send_packet(id, &payload);
        }
        true
    }

    fn exit() -> bool {
        true
    }

    fn menu_click(_id: u32, _lat: f64, _lng: f64) {}

    // The form's controls: each field kept; the UAS ID also saved, as `textBox1_TextChanged`
    // saves it. `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:365-368`
    fn form_event(id: String, value: String) {
        let mut ui = UI.lock().unwrap_or_else(PoisonError::into_inner);
        match id.as_str() {
            "uas_id" => {
                host::config_set("ODID_UAS_ID", &value);
                ui.uas_id = value;
            }
            "uas_id_type" => ui.uas_id_type = pick(ID_TYPES, &value, ui.uas_id_type),
            "ua_type" => ui.ua_type = pick(UA_TYPES, &value, ui.ua_type),
            "self_id_type" => ui.self_id_type = pick(DESC_TYPES, &value, ui.self_id_type),
            "self_id" => ui.self_id = value,
            "operator_id_type" => {
                ui.operator_id_type = pick(OPERATOR_ID_TYPES, &value, ui.operator_id_type);
            }
            "operator_id" => ui.operator_id = value,
            _ => return,
        }
        check_uid(&mut ui);
    }
}

mp_plugins::export_plugin!(OpenDroneId);

#[cfg(test)]
mod tests {
    use super::*;
    use mp_mavlink_dialects::MavMessage;

    fn backend() -> Backend {
        let mut backend = Backend::new();
        backend.target_system = 1;
        backend.uas_id = "ABC123".to_owned();
        backend.uas_id_type = 1;
        backend.ua_type = 2;
        backend.description = "survey".to_owned();
        backend.operator_id = "OP-42".to_owned();
        backend
    }

    /// Each message the backend builds is the one the dialect decodes, field for field.
    #[test]
    fn the_backends_messages_decode() {
        let backend = backend();
        let Some(MavMessage::OpenDroneIdBasicId(basic)) =
            MavMessage::decode(OPEN_DRONE_ID_BASIC_ID, &backend.basic_id())
        else {
            panic!("basic id");
        };
        assert_eq!(
            (basic.target_system, basic.id_type, basic.ua_type),
            (1, 1, 2)
        );
        assert_eq!(&basic.uas_id[..6], b"ABC123");
        assert_eq!(basic.id_or_mac, [b' '; 20]);
        let Some(MavMessage::OpenDroneIdSystem(system)) =
            MavMessage::decode(OPEN_DRONE_ID_SYSTEM, &backend.system(7))
        else {
            panic!("system");
        };
        assert_eq!(system.area_ceiling, -1000.0);
        assert_eq!(system.area_count, 1);
        assert_eq!(system.timestamp, 7);
        assert_eq!(system.operator_latitude, 0, "no GPS: zeros");
        let Some(MavMessage::OpenDroneIdSelfId(self_id)) =
            MavMessage::decode(OPEN_DRONE_ID_SELF_ID, &backend.self_id())
        else {
            panic!("self id");
        };
        assert_eq!(&self_id.description[..6], b"survey");
        let Some(MavMessage::OpenDroneIdOperatorId(operator)) =
            MavMessage::decode(OPEN_DRONE_ID_OPERATOR_ID, &backend.operator_id())
        else {
            panic!("operator id");
        };
        assert_eq!(&operator.operator_id[..5], b"OP-42");
        let mut fresh = backend;
        fresh.since_last_msg_ms = 10;
        fresh.operator_latitude = -35.5;
        let Some(MavMessage::OpenDroneIdSystemUpdate(update)) =
            MavMessage::decode(OPEN_DRONE_ID_SYSTEM_UPDATE, &fresh.system_update(9))
        else {
            panic!("system update");
        };
        assert_eq!(update.operator_latitude, -355_000_000);
    }

    /// Without a fresh position the four extended messages take turns.
    #[test]
    fn the_extended_messages_take_turns() {
        let mut backend = backend();
        let ids: Vec<u32> = (0..5).map(|_| backend.next(0).0).collect();
        assert_eq!(
            ids,
            [
                OPEN_DRONE_ID_BASIC_ID,
                OPEN_DRONE_ID_SYSTEM,
                OPEN_DRONE_ID_SELF_ID,
                OPEN_DRONE_ID_OPERATOR_ID,
                OPEN_DRONE_ID_BASIC_ID
            ]
        );
    }
}
