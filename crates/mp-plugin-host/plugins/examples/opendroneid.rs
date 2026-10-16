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
//! of the flight screen - the aircraft's UAS ID and type, the self ID, the operator ID, and the
//! vehicle's Remote ID transmitter as it reports itself - and the backend that sends the IDs to
//! that transmitter as `OPEN_DRONE_ID_*` messages (`OpenDroneID_Plugin.cs`, `OpenDroneID_UI.cs`,
//! `OpenDroneID_Backend.cs`).
//!
//! The tab is the plugin's form, drawn by the host. Once a second, with a vehicle heard, the
//! plugin subscribes to `OPEN_DRONE_ID_ARM_STATUS` from that vehicle's three transmitter
//! components and its autopilot (`start_sub`, through `subscribe-packet`); the first one heard
//! starts the backend, and every one after it is the tab's arm status (`handleODIDArmMSg2`,
//! `packet-received`). The backend's messages are built here to the byte and sent through
//! `send-packet`, `comPort.sendPacket`'s place.
//!
//! A plugin has no clock. The C#'s two loops - the UI's thread, which checks ten times a second
//! and subscribes once a second, and the backend's 100 ms timer - are `Loop` at its 10 Hz, and
//! their times are counted in Loops: each is 100 ms or a little more (`MainV2.cs:2531-2547`'s
//! rule, as the host keeps it), so ten are a little over the second the C# waits for, twenty-five
//! over its 2.5 s, fifty over its 5 s.
//!
//! What cannot run in this world, recorded:
//!
//! * the operator's position comes from an NMEA GPS on a serial port of the plugin's own
//!   (`NMEA_GPS_Connection`) - unimplementable. Without one connected the C# reads it as it is
//!   here: the position 0, 0, 0 and `last_gps_msg`, a stopwatch never started, at 0 ms - so the
//!   backend takes the position for fresh and sends `OPEN_DRONE_ID_SYSTEM_UPDATE` every second,
//!   and the GCS GPS light is orange. `comPort.MAV.cs.Base`, which the C# sets to the position,
//!   is not in the world;
//! * the time of day: the status lines go without the C#'s `HH:mm:ss - `, and the messages'
//!   `timestamp_2019`, the seconds since 2019, is 0;
//! * the status indicator it adds to the flight map's controls (`host_map_status`; its twin on
//!   the tab is the form's), the double-click on either that declares an emergency, and the
//!   developer double-click on the version that stops the sending.

use std::borrow::Cow;
use std::sync::{Mutex, PoisonError};

use mp_plugins::host::{self, Control, ControlKind, CsValue, MessageButtons};
use mp_plugins::{Guest, control};

/// `MAV_ODID_ID_TYPE`. `// C#: ExtLibs/Mavlink/Mavlink.cs:5590-5609`
const ID_TYPES: &[&str] = &[
    "NONE",
    "SERIAL_NUMBER",
    "CAA_REGISTRATION_ID",
    "UTM_ASSIGNED_UUID",
    "SPECIFIC_SESSION_ID",
];

/// `MAV_ODID_UA_TYPE`. `// C#: ExtLibs/Mavlink/Mavlink.cs:5611-5645`
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

/// `MAV_ODID_DESC_TYPE`. `// C#: ExtLibs/Mavlink/Mavlink.cs:5869-5877`
const DESC_TYPES: &[&str] = &["TEXT", "EMERGENCY", "EXTENDED_STATUS"];

/// `MAV_ODID_DESC_TYPE.EMERGENCY`'s index.
const DESC_EMERGENCY: usize = 1;

/// `MAV_ODID_OPERATOR_ID_TYPE`. `// C#: ExtLibs/Mavlink/Mavlink.cs:5959-5963`
const OPERATOR_ID_TYPES: &[&str] = &["CAA"];

/// `MAV_ODID_OPERATOR_LOCATION_TYPE.LIVE_GNSS`. `// C#: ExtLibs/Mavlink/Mavlink.cs:5884-5895`
const LIVE_GNSS: u8 = 1;

const OPEN_DRONE_ID_BASIC_ID: u32 = 12900;
const OPEN_DRONE_ID_SELF_ID: u32 = 12903;
const OPEN_DRONE_ID_SYSTEM: u32 = 12904;
const OPEN_DRONE_ID_OPERATOR_ID: u32 = 12905;
const OPEN_DRONE_ID_ARM_STATUS: u32 = 12918;
const OPEN_DRONE_ID_SYSTEM_UPDATE: u32 = 12919;

/// `MAV_COMP_ID_ALL`: the backend always addresses every component.
const MAV_COMP_ID_ALL: u8 = 0;

/// The components `start_sub` subscribes to: `MAV_COMP_ID_ODID_TXRX_1` to `_3` and
/// `MAV_COMP_ID_AUTOPILOT1`, in its order. `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:111-114;
/// ExtLibs/Mavlink/Mavlink.cs:7443, 7825-7831`
const ARM_STATUS_SENDERS: [u8; 4] = [236, 237, 238, 1];

/// Loops in the C#'s second: the UI thread's slow half, `_update_rate_hz_2`, and the backend's
/// `1.0 / system_updaterate_hz`. `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:46;
/// OpenDroneID_Backend.cs:22`
const SECOND: u32 = 10;

/// Loops in `1.0 / (rate_hz * 4)`, 2.5 s, between the backend's extended messages.
/// `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:21, 88`
const EXTENDED_EVERY: u32 = 25;

/// Loops in `ODID_ARM_MESSAGE_TIMEOUT`, the 5 s the arm status may be silent.
/// `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:31, 196`
const ARM_STATUS_TIMEOUT: u32 = 50;

/// `_gps_timeout_ms`.
const GPS_TIMEOUT_MS: u64 = 5000;

/// The colours the lights are given, by `System.Drawing.Color`'s names.
const WHITE: &str = "White";
const RED: &str = "Red";
const GREEN: &str = "Green";
const ORANGE: &str = "Orange";
const YELLOW: &str = "Yellow";

/// `OpenDroneID_Backend`'s fields. `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:17-56`
#[derive(Debug, Clone, PartialEq)]
struct Backend {
    /// `target_system`: the system the first arm status came from, sent as `(byte)`.
    target_system: u32,
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
    /// `since_last_msg_ms`: how old the operator's position is, as the GCS GPS last said.
    since_last_msg_ms: u64,
    /// `count`: which of the four extended messages goes next.
    count: u32,
    /// `running`: the timer started by `Start`.
    running: bool,
    /// Loops since `last_update_send_s` and `last_ext_send_s`; none while they are
    /// `DateTime.MinValue`, so the first of each is due at once.
    since_update: Option<u32>,
    since_extended: Option<u32>,
}

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
            since_last_msg_ms: 0,
            count: 0,
            running: false,
            since_update: None,
            since_extended: None,
        }
    }

    /// `(byte)target_system`.
    #[allow(clippy::cast_possible_truncation)]
    const fn target(&self) -> u8 {
        self.target_system as u8
    }

    const fn gps_fresh(&self) -> bool {
        self.since_last_msg_ms < GPS_TIMEOUT_MS
    }

    /// `Start`: the messages addressed to `sysid`, every component, and the timer running. A
    /// second `Start` makes the C# a second timer and drops its hold on the first, which the
    /// collector may stop at any time; here there is one. The send times are the first's.
    /// `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:54-62`
    const fn start(&mut self, sysid: u32) {
        self.target_system = sysid;
        self.running = true;
    }

    /// `send_basic_id`. `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:136-147`
    fn basic_id(&self) -> Vec<u8> {
        let mut out = vec![self.target(), MAV_COMP_ID_ALL];
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
        out.push(self.target());
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
        out.push(self.target());
        out.push(MAV_COMP_ID_ALL);
        out
    }

    /// `send_self_id`. `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:183-191`
    fn self_id(&self) -> Vec<u8> {
        let mut out = vec![self.target(), MAV_COMP_ID_ALL];
        out.extend(id_or_mac());
        out.push(self.description_type);
        out.extend(bytes_sized(&self.description, 23));
        out
    }

    /// `send_operator_id`. `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:198-206`
    fn operator_id(&self) -> Vec<u8> {
        let mut out = vec![self.target(), MAV_COMP_ID_ALL];
        out.extend(id_or_mac());
        out.push(self.operator_id_type);
        out.extend(bytes_sized(&self.operator_id, 20));
        out
    }

    /// `Send_Extended`'s switch: the next of the four extended messages, in turn.
    /// `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:92-117`
    fn extended(&mut self, timestamp: u32) -> (u32, Vec<u8>) {
        let turn = self.count % 4;
        self.count = self.count.wrapping_add(1);
        match turn {
            0 => (OPEN_DRONE_ID_BASIC_ID, self.basic_id()),
            1 => (OPEN_DRONE_ID_SYSTEM, self.system(timestamp)),
            2 => (OPEN_DRONE_ID_SELF_ID, self.self_id()),
            _ => (OPEN_DRONE_ID_OPERATOR_ID, self.operator_id()),
        }
    }

    /// `Send`, the timer's tick, a Loop: the operator's position when it is fresh and a second
    /// has passed since the last (`Send_Update`), else, when 2.5 s have passed since the last,
    /// the next extended message (`Send_Extended`); else nothing. Nothing before `Start`.
    /// `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:75-134`
    fn send(&mut self, timestamp: u32) -> Option<(u32, Vec<u8>)> {
        if !self.running {
            return None;
        }
        self.since_update = self.since_update.map(|loops| loops.saturating_add(1));
        self.since_extended = self.since_extended.map(|loops| loops.saturating_add(1));
        if self.gps_fresh() && self.since_update.is_none_or(|loops| loops >= SECOND) {
            self.since_update = Some(0);
            return Some((OPEN_DRONE_ID_SYSTEM_UPDATE, self.system_update(timestamp)));
        }
        if self
            .since_extended
            .is_none_or(|loops| loops >= EXTENDED_EVERY)
        {
            self.since_extended = Some(0);
            return Some(self.extended(timestamp));
        }
        None
    }
}

/// `MAVLink.mavlink_open_drone_id_arm_status_t`: `status`, and `error`'s fifty bytes.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:34588-34622`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ArmStatus {
    status: u8,
    error: [u8; 50],
}

impl ArmStatus {
    const NONE: Self = Self {
        status: 0,
        error: [0; 50],
    };

    /// `ToStructure`: the payload's bytes laid over the struct, zeros past its end.
    fn read(payload: &[u8]) -> Self {
        let byte = |at: usize| payload.get(at).copied().unwrap_or(0);
        Self {
            status: byte(0),
            error: std::array::from_fn(|at| byte(at + 1)),
        }
    }

    /// The error's bytes before its first zero: what a text box shows of
    /// `Encoding.UTF8.GetString(error)`, a Win32 edit control's text ending at its first NUL.
    fn error_text(&self) -> String {
        let end = self
            .error
            .iter()
            .position(|&byte| byte == 0)
            .unwrap_or(self.error.len());
        String::from_utf8_lossy(self.error.get(..end).unwrap_or_default()).into_owned()
    }
}

/// `System.Diagnostics.Stopwatch`, counting Loops: `last_odid_msg`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Stopwatch {
    loops: u32,
    running: bool,
}

impl Stopwatch {
    const NEW: Self = Self {
        loops: 0,
        running: false,
    };

    /// `Start`: on from where it stopped.
    const fn start(&mut self) {
        self.running = true;
    }

    /// `Restart`: from nothing, running.
    const fn restart(&mut self) {
        self.loops = 0;
        self.running = true;
    }

    /// `Stop`: held where it is.
    const fn stop(&mut self) {
        self.running = false;
    }

    /// A Loop passed.
    const fn tick(&mut self) {
        if self.running {
            self.loops = self.loops.saturating_add(1);
        }
    }
}

/// What `NMEA_GPS_Connection` gives the UI: `getPointNMEA()`'s position and
/// `last_gps_msg.ElapsedMilliseconds`.
/// `// C#: Plugins/OpenDroneID2/NMEA_GPS_Connection.cs:22-40, 95-98`
#[derive(Debug, Clone, Copy, PartialEq)]
struct GcsGps {
    lat: f64,
    lng: f64,
    alt_wgs84: f64,
    elapsed_ms: u64,
}

/// No GPS connected, as it always is here: a `new PointNMEA()`, and a stopwatch never started.
const NO_GPS: GcsGps = GcsGps {
    lat: 0.0,
    lng: 0.0,
    alt_wgs84: 0.0,
    elapsed_ms: 0,
};

/// `OpenDroneID_Map_Status`, the indicator on the tab (`myODID_Status`): its light,
/// `LBL_ODID_OK` and `LBL_ODID_reason`, as its Designer starts them.
/// `// C#: Plugins/OpenDroneID2/OpenDroneID_Map_Status.Designer.cs:39-66`
#[derive(Debug, Clone, PartialEq, Eq)]
struct Indicator {
    light: &'static str,
    ok: &'static str,
    reason: Cow<'static, str>,
}

impl Indicator {
    const NEW: Self = Self {
        light: WHITE,
        ok: "Remote ID",
        reason: Cow::Borrowed("{Loading....}"),
    };

    /// `setStatusOK`. `// C#: Plugins/OpenDroneID2/OpenDroneID_Map_Status.cs:27-32`
    fn ok(&mut self) {
        self.light = GREEN;
        self.ok = "Remote ID OK";
        self.reason = Cow::Borrowed("DBL CLK -> Emergency");
    }

    /// `setStatusAlert`. `// C#: Plugins/OpenDroneID2/OpenDroneID_Map_Status.cs:34-39`
    fn alert(&mut self, reason: &'static str) {
        self.light = RED;
        self.ok = "Remote ID Fail";
        self.reason = Cow::Borrowed(reason);
    }

    /// `setStatusEmergency`: the self ID's text.
    /// `// C#: Plugins/OpenDroneID2/OpenDroneID_Map_Status.cs:41-47`
    fn emergency(&mut self, reason: &str) {
        self.light = RED;
        self.ok = "RID Emergency";
        self.reason = Cow::Owned(reason.to_owned());
    }
}

/// The UI's fields. `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:19-48`
#[derive(Debug)]
struct Ui {
    uas_id: String,
    uas_id_type: usize,
    ua_type: usize,
    self_id_type: usize,
    self_id: String,
    operator_id_type: usize,
    operator_id: String,
    /// `hasODID`: a transmitter heard since the last subscription.
    has_odid: bool,
    /// `_mySYS`: the system last subscribed to.
    my_sys: u32,
    /// `odid_arm_status`: the last arm status heard.
    arm: ArmStatus,
    /// `last_odid_msg`.
    last_odid_msg: Stopwatch,
    /// `_last_odid_error_agg`: the last error's bytes summed, 0 for a ready status.
    last_error_agg: u32,
    /// `_odid_arm_msg`, `_odid_arm_status`, `_gcs_gps`.
    odid_arm_msg: bool,
    odid_arm_status: bool,
    gcs_gps: bool,
    /// `LED_RemoteID_Messages`, `LED_ArmedError`, `LED_gps_valid`: "RID Comms", "ARM Status",
    /// "GCS GPS".
    comms_light: &'static str,
    arm_light: &'static str,
    gps_light: &'static str,
    /// `myODID_Status`.
    indicator: Indicator,
    /// `TXT_RID_Status_Msg`.
    status: String,
    /// `TXT_ODID_Status`: the status lines, newest first.
    log: String,
    /// Loops since the plugin was loaded: the UI thread's slow half runs on every tenth.
    loops: u32,
    backend: Backend,
    /// The form's values as last shown: shown again only when one changes.
    shown: Vec<String>,
}

static UI: Mutex<Ui> = Mutex::new(Ui::new());

/// The index of `value` in `options`, or where it was.
fn pick(options: &[&str], value: &str, was: usize) -> usize {
    options
        .iter()
        .position(|option| *option == value)
        .unwrap_or(was)
}

/// What one Loop asks of the host, once the UI's state has moved on.
#[derive(Debug, Default, PartialEq)]
struct Asks {
    /// `start_sub`'s four subscriptions, for this system.
    subscribe: Option<u32>,
    /// The backend's message.
    send: Option<(u32, Vec<u8>)>,
}

impl Ui {
    /// As the constructor leaves the fields and the Designer the controls.
    const fn new() -> Self {
        Self {
            uas_id: String::new(),
            uas_id_type: 0,
            ua_type: 0,
            self_id_type: 0,
            self_id: String::new(),
            operator_id_type: 0,
            operator_id: String::new(),
            has_odid: false,
            my_sys: 0,
            arm: ArmStatus::NONE,
            last_odid_msg: Stopwatch::NEW,
            last_error_agg: 0,
            odid_arm_msg: false,
            odid_arm_status: false,
            gcs_gps: false,
            comms_light: WHITE,
            arm_light: WHITE,
            gps_light: WHITE,
            indicator: Indicator::NEW,
            status: String::new(),
            log: String::new(),
            loops: 0,
            backend: Backend::new(),
            shown: Vec::new(),
        }
    }

    /// `addStatusMessage`: a line on top of the status box, without its time (see the header).
    /// `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:358-363`
    fn add_status(&mut self, message: &str) {
        self.log = format!("{message}\n{}", self.log);
    }

    /// `mainloop`'s round, a Loop: the transmitter's checks ten times a second once it has been
    /// heard, the GCS GPS and the subscription once a second, and the backend's timer.
    /// `connected` is `BaseStream.IsOpen`, `current` `sysidcurrent`.
    /// `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:370-415; OpenDroneID_Backend.cs:61, 75-83`
    fn run_loop(&mut self, connected: bool, current: u32) -> Asks {
        self.loops = self.loops.wrapping_add(1);
        self.last_odid_msg.tick();
        if self.has_odid {
            self.check_odid_msgs();
            self.check_uid();
            self.check_odid_ok();
        }
        let mut asks = Asks::default();
        if self.loops.is_multiple_of(SECOND) {
            self.check_gcs_gps(&NO_GPS);
            asks.subscribe = self.start_sub(connected, current);
        }
        asks.send = self.backend.send(0);
        asks
    }

    /// `start_sub`: with the link open and a vehicle heard that is not the one subscribed to,
    /// its arm status subscribed to, and the transmitter not yet heard from it. The earlier
    /// system's subscriptions stay, as the C# never takes them back. Returns the system to
    /// subscribe to. `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:99-121`
    fn start_sub(&mut self, connected: bool, current: u32) -> Option<u32> {
        if !connected || current == self.my_sys || current == 0 {
            return None;
        }
        self.add_status(&format!("Sub. to ODID ARM_STATUS for SysId: {current}"));
        self.my_sys = current;
        self.has_odid = false;
        self.last_odid_msg.stop();
        Some(current)
    }

    /// `handleODIDArmMSg2`: an arm status from system `sysid` (from whichever of the four
    /// components; the C# passes the component to `Start`, which addresses every one instead).
    /// The first since the subscription starts the backend for that system; each after it
    /// restarts the timeout and, when the error changes, says so. `current` is
    /// `comPort.MAV.sysid`. Returns the console's line, for the log, when it starts the backend.
    /// `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:123-185`
    fn heard(&mut self, sysid: u32, payload: &[u8], current: u32) -> Option<String> {
        self.arm = ArmStatus::read(payload);
        let mut started = None;
        if self.has_odid {
            self.last_odid_msg.restart();
            let agg = if self.arm.status == 0 {
                0
            } else {
                self.arm.error.iter().map(|&byte| u32::from(byte)).sum()
            };
            if agg != self.last_error_agg {
                if self.arm.status == 0 {
                    self.add_status("Arm Stats: Ready");
                } else {
                    // `s.Substring(0, s.IndexOf((char)0))` of `Encoding.ASCII.GetString`: an
                    // error with no NUL in its fifty bytes throws, `PacketReceived` catches it,
                    // and the rest of the handler does not run - the change is not kept.
                    let end = self.arm.error.iter().position(|&byte| byte == 0)?;
                    let text: String = self
                        .arm
                        .error
                        .iter()
                        .take(end)
                        .map(|&byte| if byte.is_ascii() { char::from(byte) } else { '?' })
                        .collect();
                    self.add_status(&format!("Arm Error: {text}"));
                }
                self.last_error_agg = agg;
            }
        } else {
            started = Some(format!(
                "[DRONE_ID] Detected and Starting on System ID: {current}"
            ));
            self.add_status(&format!("Detected and Starting on System ID: {current}"));
            self.last_odid_msg.start();
            // `if (dev_mode_rm == false)`: the developer's double-click is not here (the header).
            self.backend.start(sysid);
            self.add_status(
                "Double Click Primary status indicator to declare Emergency over ODID.",
            );
        }
        self.has_odid = true;
        started
    }

    /// `checkODIDMsgs`: the lights and the status box from the last arm status and how long ago
    /// it came. `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:191-214`
    fn check_odid_msgs(&mut self) {
        if !self.has_odid {
            return;
        }
        self.odid_arm_msg = self.last_odid_msg.loops < ARM_STATUS_TIMEOUT;
        if !self.odid_arm_msg {
            self.odid_arm_status = false;
        }
        self.comms_light = if self.odid_arm_msg { GREEN } else { RED };
        // Set again from the status alone, timed out or not, as the C# does.
        self.odid_arm_status = self.arm.status == 0;
        self.arm_light = if self.odid_arm_status { GREEN } else { RED };
        self.status = if !self.odid_arm_msg {
            "Timeout.".to_owned()
        } else if self.arm.status == 0 {
            "Ready".to_owned()
        } else {
            self.arm.error_text()
        };
    }

    /// `checkODID_OK`: the indicator - the emergency the self ID declares, else the first of
    /// the GCS GPS, the arm status's silence and its error, else OK.
    /// `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:224-262`
    fn check_odid_ok(&mut self) {
        if self.self_id_type == DESC_EMERGENCY {
            self.indicator.emergency(&self.self_id);
            return;
        }
        let reason = if !self.gcs_gps {
            "GCS GPS Invalid"
        } else if !self.odid_arm_msg {
            "Remote ID Msg Timeout"
        } else if !self.odid_arm_status {
            "Remote ID ARM Error"
        } else {
            self.indicator.ok();
            return;
        };
        self.indicator.alert(reason);
    }

    /// `checkUID`: the form's fields into the backend, as the C# takes them.
    /// `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:264-293`
    #[allow(clippy::cast_possible_truncation)]
    fn check_uid(&mut self) {
        let uas_id = !self.uas_id.is_empty() && self.uas_id_type > 0 && self.ua_type > 0;
        if uas_id {
            self.backend.uas_id.clone_from(&self.uas_id);
            self.backend.ua_type = self.ua_type as u8;
            self.backend.uas_id_type = self.uas_id_type as u8;
        }
        if !self.self_id.is_empty() {
            self.backend.description.clone_from(&self.self_id);
            self.backend.description_type = self.self_id_type as u8;
        }
        if !self.operator_id.is_empty() {
            self.backend.operator_id.clone_from(&self.operator_id);
            self.backend.operator_id_type = self.operator_id_type as u8;
        }
    }

    /// `checkGCSGPS`: the GCS GPS into the backend and its light. `gpsHasSBAS` is never set in
    /// the C#, so a fix is yellow, never green.
    /// `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:24, 295-341`
    #[allow(clippy::cast_possible_truncation)]
    fn check_gcs_gps(&mut self, gps: &GcsGps) {
        self.backend.operator_latitude = gps.lat;
        self.backend.operator_longitude = gps.lng;
        self.backend.operator_altitude_geo = gps.alt_wgs84 as f32;
        self.backend.operator_location_type = LIVE_GNSS;
        self.backend.since_last_msg_ms = gps.elapsed_ms;
        if gps.elapsed_ms > GPS_TIMEOUT_MS {
            self.gcs_gps = false;
        } else if gps.lat == 0.0 || gps.lng == 0.0 {
            self.gps_light = ORANGE;
            self.gcs_gps = false;
        } else {
            self.gps_light = YELLOW;
            self.gcs_gps = true;
        }
    }

    /// The Drone ID tab, as the form shows it, in the C#'s order: the indicator and the "Remote
    /// ID Status" lights above the tabs, then the Status tab (the arm status, the status lines,
    /// the version), the UAS ID tab and the Operations tab.
    /// `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.Designer.cs:92-445`
    fn controls(&self) -> Vec<Control> {
        let at = |options: &'static [&'static str], index: usize| -> &'static str {
            options.get(index).copied().unwrap_or("")
        };
        vec![
            control("odid_light", ControlKind::Label, "", self.indicator.light, &[]),
            control("odid_ok", ControlKind::Label, "", self.indicator.ok, &[]),
            control(
                "odid_reason",
                ControlKind::Label,
                "",
                &self.indicator.reason,
                &[],
            ),
            control(
                "rid_comms",
                ControlKind::Label,
                "RID Comms",
                self.comms_light,
                &[],
            ),
            control(
                "arm_status",
                ControlKind::Label,
                "ARM Status",
                self.arm_light,
                &[],
            ),
            control("gcs_gps", ControlKind::Label, "GCS GPS", self.gps_light, &[]),
            control(
                "status",
                ControlKind::Label,
                "RID Armed Status",
                &self.status,
                &[],
            ),
            control("log", ControlKind::Label, "", &self.log, &[]),
            control("version", ControlKind::Label, "", "Ver: 0.05", &[]),
            control("uas_id", ControlKind::TextBox, "UAS ID", &self.uas_id, &[]),
            control(
                "uas_id_type",
                ControlKind::ComboBox,
                "UAS ID Type",
                at(ID_TYPES, self.uas_id_type),
                ID_TYPES,
            ),
            control(
                "ua_type",
                ControlKind::ComboBox,
                "UA Type",
                at(UA_TYPES, self.ua_type),
                UA_TYPES,
            ),
            control(
                "self_id_type",
                ControlKind::ComboBox,
                "Self ID Type",
                at(DESC_TYPES, self.self_id_type),
                DESC_TYPES,
            ),
            control(
                "self_id",
                ControlKind::TextBox,
                "Self ID Desc",
                &self.self_id,
                &[],
            ),
            control(
                "operator_id_type",
                ControlKind::ComboBox,
                "Oper. ID Type",
                at(OPERATOR_ID_TYPES, self.operator_id_type),
                OPERATOR_ID_TYPES,
            ),
            control(
                "operator_id",
                ControlKind::TextBox,
                "Operator ID",
                &self.operator_id,
                &[],
            ),
        ]
    }

    /// The form's controls when a value has changed since they were last shown: the C#'s
    /// controls change as the UI's thread sets them; here the form is shown again.
    fn changed(&mut self) -> Option<Vec<Control>> {
        let controls = self.controls();
        let values: Vec<String> = controls.iter().map(|c| c.value.clone()).collect();
        if values == self.shown {
            return None;
        }
        self.shown = values;
        Some(controls)
    }
}

/// The tab shown again if anything on it changed.
fn show(ui: &mut Ui) {
    if let Some(controls) = ui.changed() {
        host::form_show("Drone ID", &controls);
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
    // ask (the owner's bug, 2026-10-04: the message came at every start). Then its page,
    // `tabDroneID`, "Drone ID", the sixth of the flight screen's tabs, which its form fills - the
    // page the saved tabs do not name until it is ticked in the tabs' Customize.
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
        host::flight_data_tab_add("tabDroneID", "Drone ID", 5);
        show(&mut UI.lock().unwrap_or_else(PoisonError::into_inner));
        true
    }

    // `mainloop` and the backend's timer, once a Loop: the subscription it asks for made, the
    // message it sends sent. `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:99-121, 370-415`
    fn run_loop() -> bool {
        let connected = matches!(host::cs("connected"), Some(CsValue::Flag(true)));
        let current = host::sysid_current();
        let mut ui = UI.lock().unwrap_or_else(PoisonError::into_inner);
        let asks = ui.run_loop(connected, current);
        if let Some(sysid) = asks.subscribe {
            for compid in ARM_STATUS_SENDERS {
                host::subscribe_packet(OPEN_DRONE_ID_ARM_STATUS, sysid, compid);
            }
        }
        if let Some((id, payload)) = asks.send {
            let _ = host::send_packet(id, &payload);
        }
        show(&mut ui);
        true
    }

    fn exit() -> bool {
        true
    }

    fn menu_click(_id: u32, _lat: f64, _lng: f64) {}

    // The form's controls: each field kept, for the next `checkUID`; the UAS ID also saved, as
    // `textBox1_TextChanged` saves it. `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:365-368`
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
            _ => {}
        }
    }

    // `handleODIDArmMSg2`, the function the four subscriptions share; the console's line to the
    // application's log. `// C#: Plugins/OpenDroneID2/OpenDroneID_UI.cs:123-185`
    fn packet_received(message_id: u32, sysid: u32, _compid: u8, payload: Vec<u8>) {
        if message_id != OPEN_DRONE_ID_ARM_STATUS {
            return;
        }
        let current = host::sysid_current();
        let mut ui = UI.lock().unwrap_or_else(PoisonError::into_inner);
        if let Some(line) = ui.heard(sysid, &payload, current) {
            host::log(&line);
        }
        show(&mut ui);
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

    /// A fresh UI, as the static starts it.
    fn ui() -> Ui {
        Ui::new()
    }

    /// An arm status's payload: its status and error text.
    fn arm(status: u8, error: &str) -> Vec<u8> {
        let mut payload = vec![status];
        payload.extend(bytes_sized(error, 50));
        payload
    }

    /// The status lines, newest first.
    fn lines(ui: &Ui) -> Vec<&str> {
        ui.log.lines().collect()
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
        let mut stale = backend.clone();
        stale.since_last_msg_ms = GPS_TIMEOUT_MS;
        stale.operator_latitude = -35.5;
        let Some(MavMessage::OpenDroneIdSystem(system)) =
            MavMessage::decode(OPEN_DRONE_ID_SYSTEM, &stale.system(7))
        else {
            panic!("system");
        };
        assert_eq!(system.area_ceiling, -1000.0);
        assert_eq!(system.area_count, 1);
        assert_eq!(system.timestamp, 7);
        assert_eq!(system.operator_latitude, 0, "the GPS timed out: zeros");
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
        // `(byte)target_system`.
        fresh.target_system = 257;
        assert_eq!(fresh.system_update(0).get(16), Some(&1));
    }

    /// The four extended messages take turns.
    #[test]
    fn the_extended_messages_take_turns() {
        let mut backend = backend();
        let ids: Vec<u32> = (0..5).map(|_| backend.extended(0).0).collect();
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

    /// The timer, a Loop each: nothing before `Start`; then, with the position fresh as it is
    /// without a GPS, the update at once and every second - ten Loops - and an extended message
    /// on the next Loop and every 2.5 s - twenty-five - after it, waiting a Loop when the
    /// update takes its turn; with the position stale, the extended messages alone.
    /// `// C#: Plugins/OpenDroneID2/OpenDroneID_Backend.cs:75-134`
    #[test]
    fn the_backend_sends_at_the_csharps_rates() {
        let mut backend = backend();
        assert_eq!(backend.send(0), None, "not started");
        backend.start(1);
        let sent: Vec<(usize, u32)> = (1..=60)
            .filter_map(|tick| backend.send(0).map(|(id, _)| (tick, id)))
            .collect();
        assert_eq!(
            sent,
            [
                (1, OPEN_DRONE_ID_SYSTEM_UPDATE),
                (2, OPEN_DRONE_ID_BASIC_ID),
                (11, OPEN_DRONE_ID_SYSTEM_UPDATE),
                (21, OPEN_DRONE_ID_SYSTEM_UPDATE),
                (27, OPEN_DRONE_ID_SYSTEM),
                (31, OPEN_DRONE_ID_SYSTEM_UPDATE),
                (41, OPEN_DRONE_ID_SYSTEM_UPDATE),
                (51, OPEN_DRONE_ID_SYSTEM_UPDATE),
                (52, OPEN_DRONE_ID_SELF_ID),
            ]
        );
        let mut stale = self::backend();
        stale.since_last_msg_ms = GPS_TIMEOUT_MS;
        stale.start(1);
        let sent: Vec<usize> = (1..=60)
            .filter(|_| stale.send(0).is_some())
            .collect();
        assert_eq!(sent, [1, 26, 51]);
    }

    /// The subscription: none until the link is open and a vehicle heard; then once for that
    /// system, the transmitter not yet heard and the line said; not again for the same system;
    /// again for another.
    #[test]
    fn start_sub_subscribes_once_a_system() {
        let mut ui = ui();
        assert_eq!(ui.start_sub(false, 1), None);
        assert_eq!(ui.start_sub(true, 0), None);
        assert_eq!(ui.start_sub(true, 1), Some(1));
        assert_eq!(lines(&ui), ["Sub. to ODID ARM_STATUS for SysId: 1"]);
        assert_eq!(ui.start_sub(true, 1), None);
        ui.has_odid = true;
        assert_eq!(ui.start_sub(true, 2), Some(2));
        assert!(!ui.has_odid);
        assert_eq!(lines(&ui)[0], "Sub. to ODID ARM_STATUS for SysId: 2");
    }

    /// The first arm status starts the backend for its system and says so; the next with an
    /// error says the error up to its NUL, the same error again nothing, a ready one "Ready",
    /// and a ready one after that nothing.
    #[test]
    fn the_arm_status_is_said_when_it_changes() {
        let mut ui = ui();
        let started = ui.heard(3, &arm(1, "SYSTEM not received"), 1);
        assert_eq!(
            started.as_deref(),
            Some("[DRONE_ID] Detected and Starting on System ID: 1")
        );
        assert!(ui.has_odid && ui.backend.running);
        assert_eq!(ui.backend.target_system, 3);
        assert_eq!(
            lines(&ui),
            [
                "Double Click Primary status indicator to declare Emergency over ODID.",
                "Detected and Starting on System ID: 1",
            ]
        );
        assert_eq!(ui.heard(3, &arm(1, "SYSTEM not received"), 1), None);
        assert_eq!(lines(&ui)[0], "Arm Error: SYSTEM not received");
        ui.heard(3, &arm(1, "SYSTEM not received"), 1);
        assert_eq!(lines(&ui).len(), 3, "the same error again");
        ui.heard(3, &arm(0, ""), 1);
        assert_eq!(lines(&ui)[0], "Arm Stats: Ready");
        ui.heard(3, &arm(0, ""), 1);
        assert_eq!(lines(&ui).len(), 4);
        // A first status that is ready, then the same: the C# says nothing for either, as the
        // sum it compares starts at 0.
        let mut quiet = self::ui();
        quiet.heard(1, &arm(0, ""), 1);
        quiet.heard(1, &arm(0, ""), 1);
        assert_eq!(lines(&quiet).len(), 2);
    }

    /// An error of fifty bytes and no NUL: the C#'s `Substring` throws, and the change is not
    /// kept, so the next one is compared with the old sum; bytes past ASCII are `?`.
    #[test]
    fn an_error_without_its_nul_is_the_csharps_exception() {
        let mut ui = ui();
        ui.heard(1, &arm(1, "x"), 1);
        let full = vec![1; 51];
        ui.heard(1, &full, 1);
        assert_eq!(lines(&ui).len(), 2, "nothing said");
        assert_eq!(ui.last_error_agg, 0);
        ui.heard(1, &[1, 0xE9, b'!', 0], 1);
        assert_eq!(lines(&ui)[0], "Arm Error: ?!");
    }

    /// The lights and the status box: green and the error while the arm status comes, ready
    /// green; silent for fifty Loops, red and "Timeout."; the indicator the GCS GPS's first
    /// without a GPS, the transmitter's with one, OK with both; an emergency's self ID over all.
    #[test]
    fn the_checks_light_the_tab() {
        let mut ui = ui();
        ui.check_gcs_gps(&NO_GPS);
        assert_eq!((ui.gps_light, ui.gcs_gps), (ORANGE, false));
        assert_eq!(ui.backend.operator_location_type, LIVE_GNSS);
        assert_eq!(ui.backend.since_last_msg_ms, 0, "a stopwatch never started");
        assert_eq!(ui.start_sub(true, 1), Some(1));
        ui.heard(1, &arm(1, "SYSTEM not received"), 1);
        ui.run_loop(true, 1);
        assert_eq!((ui.comms_light, ui.arm_light), (GREEN, RED));
        assert_eq!(ui.status, "SYSTEM not received");
        assert_eq!(
            (ui.indicator.light, ui.indicator.ok, &*ui.indicator.reason),
            (RED, "Remote ID Fail", "GCS GPS Invalid")
        );
        ui.heard(1, &arm(0, ""), 1);
        ui.run_loop(true, 1);
        assert_eq!((ui.arm_light, ui.status.as_str()), (GREEN, "Ready"));
        for _ in 0..ARM_STATUS_TIMEOUT {
            ui.run_loop(true, 1);
        }
        assert_eq!((ui.comms_light, ui.status.as_str()), (RED, "Timeout."));
        // A GPS with a fix: yellow, and the transmitter's silence is the reason.
        let fix = GcsGps {
            lat: -35.36,
            lng: 149.16,
            alt_wgs84: 600.0,
            elapsed_ms: 200,
        };
        ui.check_gcs_gps(&fix);
        assert_eq!((ui.gps_light, ui.gcs_gps), (YELLOW, true));
        ui.check_odid_ok();
        assert_eq!(&*ui.indicator.reason, "Remote ID Msg Timeout");
        ui.heard(1, &arm(0, ""), 1);
        ui.check_odid_msgs();
        ui.check_odid_ok();
        assert_eq!(
            (ui.indicator.light, ui.indicator.ok, &*ui.indicator.reason),
            (GREEN, "Remote ID OK", "DBL CLK -> Emergency")
        );
        ui.self_id_type = DESC_EMERGENCY;
        "lost link".clone_into(&mut ui.self_id);
        ui.check_odid_ok();
        assert_eq!(
            (ui.indicator.ok, &*ui.indicator.reason),
            ("RID Emergency", "lost link")
        );
        // An emergency with no text says none, not the Designer's "{Loading....}".
        ui.self_id.clear();
        ui.check_odid_ok();
        assert_eq!(&*ui.indicator.reason, "");
    }

    /// A Loop's asks: the subscription on every tenth Loop with a vehicle; the backend's
    /// messages once it is started.
    #[test]
    fn a_loop_asks_for_the_subscription_and_the_messages() {
        let mut ui = ui();
        let asks: Vec<Asks> = (0..10).map(|_| ui.run_loop(true, 1)).collect();
        assert!(asks.iter().take(9).all(|ask| *ask == Asks::default()));
        assert_eq!(asks.last().and_then(|ask| ask.subscribe), Some(1));
        assert_eq!(ui.gps_light, ORANGE);
        ui.heard(1, &arm(1, "SYSTEM not received"), 1);
        let first = ui.run_loop(true, 1);
        assert_eq!(first.send.map(|(id, _)| id), Some(OPEN_DRONE_ID_SYSTEM_UPDATE));
        let second = ui.run_loop(true, 1);
        assert_eq!(second.send.map(|(id, _)| id), Some(OPEN_DRONE_ID_BASIC_ID));
    }

    /// The form is shown again only when a value on it has changed.
    #[test]
    fn the_form_is_shown_again_when_it_changes() {
        let mut ui = ui();
        assert!(ui.changed().is_some());
        assert!(ui.changed().is_none());
        ui.run_loop(true, 1);
        assert!(ui.changed().is_none(), "nothing on the tab moved");
        ui.add_status("x");
        let controls = ui.changed().expect("shown again");
        let log = controls.iter().find(|c| c.id == "log").map(|c| c.value.clone());
        assert_eq!(log.as_deref(), Some("x\n"));
        let reason = controls.iter().find(|c| c.id == "odid_reason");
        assert_eq!(reason.map(|c| c.value.as_str()), Some("{Loading....}"));
    }
}
