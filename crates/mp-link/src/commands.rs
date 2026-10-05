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

//! Vehicle commands.
//!
//! Thin, named constructors over the raw messages. The value is not abstraction, it is that
//! `arm(id)` cannot be built with the arm/disarm parameter in the wrong slot, and that every
//! magic number has exactly one definition with a comment saying what it is.

use mp_mavlink_dialects::all::{
    CommandInt, CommandLong, MavMessage, MissionAck, MissionCount, MissionItem as FloatItem,
    MissionItemInt, MissionRequestInt, MissionRequestList, MissionSetCurrent, ParamRequestList,
    AutopilotVersionRequest, ParamRequestRead, ParamSet, RcChannelsOverride, SetGpsGlobalOrigin,
    SetMode, SetPositionTargetGlobalInt, SystemTime,
};
use mp_mission::MissionItem;
use mp_vehicle::VehicleId;

/// `MAV_CMD_NAV_TAKEOFF`.
pub const CMD_NAV_TAKEOFF: u16 = 22;
/// `MAV_CMD_NAV_LAND`.
pub const CMD_NAV_LAND: u16 = 21;
/// `MAV_CMD_COMPONENT_ARM_DISARM`.
pub const CMD_COMPONENT_ARM_DISARM: u16 = 400;
/// `MAV_CMD_DO_SET_MODE`.
pub const CMD_DO_SET_MODE: u16 = 176;
/// `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`: tells the autopilot the custom mode field is meaningful.
pub const MODE_FLAG_CUSTOM_MODE_ENABLED: u8 = 1;
/// `MAV_PARAM_TYPE_REAL32`.
pub const PARAM_TYPE_REAL32: u8 = 9;

/// ArduCopter flight mode numbers, which are vehicle-specific custom modes rather than MAVLink
/// standard modes. Defined here because the numbers are meaningless without the names.
pub mod copter_mode {
    /// Stabilize.
    pub const STABILIZE: u32 = 0;
    /// Guided: accepts position and velocity targets from a GCS.
    pub const GUIDED: u32 = 4;
    /// Loiter.
    pub const LOITER: u32 = 5;
    /// Return to launch.
    pub const RTL: u32 = 6;
    /// Land.
    pub const LAND: u32 = 9;
}

pub(crate) fn command(target: VehicleId, command: u16, params: [f32; 7]) -> MavMessage {
    MavMessage::CommandLong(CommandLong {
        param1: params[0],
        param2: params[1],
        param3: params[2],
        param4: params[3],
        param5: params[4],
        param6: params[5],
        param7: params[6],
        command,
        target_system: target.sysid,
        target_component: target.compid,
        confirmation: 0,
    })
}

/// `magic_force_arm_value`, `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2634`.
pub const MAGIC_FORCE_ARM: f32 = 2989.0;
/// `magic_force_disarm_value`, `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2635`.
pub const MAGIC_FORCE_DISARM: f32 = 21196.0;

/// Arms or disarms the vehicle.
///
/// `force` sends the magic value that bypasses the autopilot's checks - and it is a different
/// value each way: 2989 to force an arm, 21196 to force a disarm. Mission Planner's `doARMAsync`
/// has both as named constants, and ArduPilot looks for each one only on its own side; the
/// wrong one is an ordinary command the vehicle checks and refuses. It exists because the
/// protocol has it and ground crews occasionally need it; it is never the default.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2634-2635, 2640-2644`
#[must_use]
pub fn arm(target: VehicleId, arm: bool, force: bool) -> MavMessage {
    let force_magic = match (force, arm) {
        (false, _) => 0.0,
        (true, true) => MAGIC_FORCE_ARM,
        (true, false) => MAGIC_FORCE_DISARM,
    };
    command(
        target,
        CMD_COMPONENT_ARM_DISARM,
        [
            if arm { 1.0 } else { 0.0 },
            force_magic,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
    )
}

/// Takes off to an altitude above the home point, in metres.
#[must_use]
pub fn takeoff(target: VehicleId, altitude_metres: f32) -> MavMessage {
    command(
        target,
        CMD_NAV_TAKEOFF,
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, altitude_metres],
    )
}

/// Lands where the vehicle currently is.
#[must_use]
pub fn land(target: VehicleId) -> MavMessage {
    command(target, CMD_NAV_LAND, [0.0; 7])
}

/// `MAV_CMD_DO_SEND_BANNER`.
pub const CMD_DO_SEND_BANNER: u16 = 42_428;

/// `MAV_CMD_REQUEST_MESSAGE`.
pub const CMD_REQUEST_MESSAGE: u16 = 512;
/// `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES`, deprecated but still sent.
pub const CMD_REQUEST_AUTOPILOT_CAPABILITIES: u16 = 520;
/// `AUTOPILOT_VERSION`'s message id, `MAV_CMD_REQUEST_MESSAGE`'s param1.
pub const MSG_ID_AUTOPILOT_VERSION: f32 = 148.0;

/// `getVersion`: asks the vehicle for its `AUTOPILOT_VERSION` - its capabilities (FTP among
/// them, which decides the MAVFtp page and the parameter download's path), its flight software
/// version and its ids - "using all three methods", in the C#'s words: `MAV_CMD_REQUEST_MESSAGE`
/// for message 148, the deprecated `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES`, and the deprecated
/// `AUTOPILOT_VERSION_REQUEST` message. Mission Planner sends them at connect, before the banner
/// request, and waits there for the answer (`responcerequired`, true by default); here the
/// answer arrives with the stream and is read from the vehicle's state when it has.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:928, 5847-5861`
#[must_use]
pub fn get_version(target: VehicleId) -> [MavMessage; 3] {
    [
        command(
            target,
            CMD_REQUEST_MESSAGE,
            [MSG_ID_AUTOPILOT_VERSION, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ),
        command(target, CMD_REQUEST_AUTOPILOT_CAPABILITIES, [0.0; 7]),
        MavMessage::AutopilotVersionRequest(AutopilotVersionRequest {
            target_system: target.sysid,
            target_component: target.compid,
        }),
    ]
}

/// Asks the vehicle to say what it is.
///
/// The answer is the `STATUSTEXT` banner - `ArduCopter V4.5.7 (1c0c8d9c)` - which is where
/// Mission Planner reads the firmware version it fetches parameter documentation for. It sends
/// this at connect and again whenever a new vehicle appears.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:930, 1856`
#[must_use]
pub fn send_banner(target: VehicleId) -> MavMessage {
    command(target, CMD_DO_SEND_BANNER, [0.0; 7])
}

/// Sets a vehicle-specific flight mode.
#[must_use]
pub fn set_mode(target: VehicleId, custom_mode: u32) -> MavMessage {
    MavMessage::SetMode(SetMode {
        custom_mode,
        target_system: target.sysid,
        base_mode: MODE_FLAG_CUSTOM_MODE_ENABLED,
    })
}

/// Sets a parameter by name.
///
/// Parameter ids are a fixed 16-byte field that is *not* null-terminated when the name uses all
/// 16 bytes - a detail that silently truncates names like `SERVO16_FUNCTION` if handled naively.
#[must_use]
pub fn param_set(target: VehicleId, name: &str, value: f32) -> MavMessage {
    let mut param_id = [0u8; 16];
    let bytes = name.as_bytes();
    let n = bytes.len().min(16);
    if let (Some(dst), Some(src)) = (param_id.get_mut(..n), bytes.get(..n)) {
        dst.copy_from_slice(src);
    }

    MavMessage::ParamSet(ParamSet {
        param_value: value,
        target_system: target.sysid,
        target_component: target.compid,
        param_id,
        param_type: PARAM_TYPE_REAL32,
    })
}

/// Commands a guided-mode vehicle to a global position.
///
/// The type mask selects which fields the autopilot should honour; the bits are inverted in the
/// usual MAVLink way - a set bit means *ignore* that field.
#[must_use]
pub fn goto_position(
    target: VehicleId,
    latitude: f64,
    longitude: f64,
    altitude_relative_metres: f32,
) -> MavMessage {
    /// Ignore every velocity, acceleration, yaw and yaw-rate field; use position only.
    const IGNORE_ALL_BUT_POSITION: u16 = 0b0000_1111_1111_1000;
    /// `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`.
    const FRAME_GLOBAL_RELATIVE_ALT_INT: u8 = 6;

    #[allow(clippy::cast_possible_truncation)] // coordinates are range-checked by the caller
    let (lat_e7, lon_e7) = ((latitude * 1e7) as i32, (longitude * 1e7) as i32);

    MavMessage::SetPositionTargetGlobalInt(SetPositionTargetGlobalInt {
        time_boot_ms: 0,
        lat_int: lat_e7,
        lon_int: lon_e7,
        alt: altitude_relative_metres,
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
        afx: 0.0,
        afy: 0.0,
        afz: 0.0,
        yaw: 0.0,
        yaw_rate: 0.0,
        type_mask: IGNORE_ALL_BUT_POSITION,
        target_system: target.sysid,
        target_component: target.compid,
        coordinate_frame: FRAME_GLOBAL_RELATIVE_ALT_INT,
    })
}

/// Asks the vehicle to stream its entire parameter set.
///
/// The vehicle streams once and never retransmits, so a dropped packet leaves a permanent hole.
/// Recovering it is [`request_param_by_index`]'s job.
#[must_use]
pub fn request_param_list(target: VehicleId) -> MavMessage {
    MavMessage::ParamRequestList(ParamRequestList {
        target_system: target.sysid,
        target_component: target.compid,
    })
}

/// Asks for one parameter by its index in the list.
///
/// `param_id` must be empty when requesting by index; a vehicle that sees a name will answer the
/// name and ignore the index.
#[must_use]
pub fn request_param_by_index(target: VehicleId, index: u16) -> MavMessage {
    MavMessage::ParamRequestRead(ParamRequestRead {
        param_index: i16::try_from(index).unwrap_or(i16::MAX),
        target_system: target.sysid,
        target_component: target.compid,
        param_id: [0u8; 16],
    })
}

/// Asks for one parameter by name.
///
/// `param_index` must be -1 to mean "use the name", which is the opposite convention to
/// [`request_param_by_index`] and easy to get backwards.
#[must_use]
pub fn request_param_by_name(target: VehicleId, name: &str) -> MavMessage {
    MavMessage::ParamRequestRead(ParamRequestRead {
        param_index: -1,
        target_system: target.sysid,
        target_component: target.compid,
        param_id: mp_params::encode_param_id(name),
    })
}

/// `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`.
const CMD_PREFLIGHT_REBOOT_SHUTDOWN: u16 = 246;

/// Reboots the autopilot.
///
/// param1 = 1 means reboot; 2 and 3 shut down and reboot to bootloader, which are not offered
/// here. A shutdown from a ground station is a vehicle that has to be reached physically to come
/// back, and the bootloader is for firmware tools.
///
/// The link drops when this is obeyed, which is the expected outcome rather than a failure.
#[must_use]
pub fn reboot(target: VehicleId) -> MavMessage {
    command(
        target,
        CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
}

/// `PREFLIGHT_REBOOT_SHUTDOWN` with param1 = 3, which reboots the autopilot into its bootloader
/// for a firmware upload: the first command of `doReboot(true, false)`.
///
/// That call - the firmware page's, with `currentvehicle` false - waits for a heartbeat and then
/// calls `doCommand` twice, with param1 = 3 and then param1 = 1, each unconditionally; and
/// `doCommand` writes a `PREFLIGHT_REBOOT_SHUTDOWN` twice and waits for no acknowledgement. So
/// the C# puts four frames on the wire - 3, 3, 1, 1 - with no gap between them. The firmware
/// page (`mp-gui`'s `config/firmware.rs`, `reboot_to_bootloader`) sends this and then
/// [`reboot`] through [`Link::command`](crate::Link::command), which makes the two writes per
/// command; a caller sending this once with `Link::send` sends a quarter of them.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2553-2559, 2591-2614, 2758-2763`
#[must_use]
pub fn reboot_to_bootloader(target: VehicleId) -> MavMessage {
    command(
        target,
        CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
}

/// Asks the vehicle to list the dataflash logs it holds.
///
/// The range is inclusive and 0 to `u16::MAX` means all of them, which is what a ground station
/// wants: the vehicle knows how many there are and we do not.
#[must_use]
pub fn request_log_list(target: VehicleId) -> MavMessage {
    MavMessage::LogRequestList(mp_mavlink_dialects::all::LogRequestList {
        start: 0,
        end: u16::MAX,
        target_system: target.sysid,
        target_component: target.compid,
    })
}

/// Asks for part of one log.
///
/// A window rather than the whole log. Asking for all of it works and leaves no way to notice a
/// stall; asking in windows means a gap can be re-requested without starting again, which matters
/// because a log download happens over a telemetry radio and takes minutes.
#[must_use]
pub fn request_log_data(target: VehicleId, id: u16, offset: u32, count: u32) -> MavMessage {
    MavMessage::LogRequestData(mp_mavlink_dialects::all::LogRequestData {
        ofs: offset,
        count,
        id,
        target_system: target.sysid,
        target_component: target.compid,
    })
}

/// Tells the vehicle the download is finished, so it stops sending.
#[must_use]
pub fn log_request_end(target: VehicleId) -> MavMessage {
    MavMessage::LogRequestEnd(mp_mavlink_dialects::all::LogRequestEnd {
        target_system: target.sysid,
        target_component: target.compid,
    })
}

/// Asks the vehicle how many items it holds, starting a download.
///
/// `mission_type` selects which list: the mission, the geofence or the rally points. They share
/// one protocol and one set of messages, distinguished only by this field, which is why it has to
/// be carried on every message of a transfer rather than assumed.
#[must_use]
pub fn request_mission_list(target: VehicleId, mission_type: u8) -> MavMessage {
    MavMessage::MissionRequestList(MissionRequestList {
        target_system: target.sysid,
        target_component: target.compid,
        mission_type,
    })
}

/// Asks for one mission item by sequence number.
///
/// Always the `_INT` form. The float variant carries degrees in an `f32`, which has about a metre
/// of resolution at the equator - enough to move a waypoint off a runway.
#[must_use]
pub fn request_mission_item(target: VehicleId, seq: u16, mission_type: u8) -> MavMessage {
    MavMessage::MissionRequestInt(MissionRequestInt {
        seq,
        target_system: target.sysid,
        target_component: target.compid,
        mission_type,
    })
}

/// Announces how many items are about to be uploaded, starting an upload.
#[must_use]
pub fn send_mission_count(target: VehicleId, count: u16, mission_type: u8) -> MavMessage {
    MavMessage::MissionCount(MissionCount {
        count,
        target_system: target.sysid,
        target_component: target.compid,
        mission_type,
    })
}

/// Sends one mission item during an upload.
#[must_use]
pub fn send_mission_item(target: VehicleId, item: &MissionItem, mission_type: u8) -> MavMessage {
    let wire = item.to_wire();
    MavMessage::MissionItemInt(MissionItemInt {
        param1: wire.param1,
        param2: wire.param2,
        param3: wire.param3,
        param4: wire.param4,
        x: wire.x,
        y: wire.y,
        z: wire.z,
        seq: wire.seq,
        command: wire.command,
        target_system: target.sysid,
        target_component: target.compid,
        frame: wire.frame,
        current: wire.current,
        autocontinue: wire.autocontinue,
        mission_type,
    })
}

/// Acknowledges a completed transfer. `MAV_MISSION_ACCEPTED` is 0.
#[must_use]
pub fn send_mission_ack(target: VehicleId, result: u8, mission_type: u8) -> MavMessage {
    MavMessage::MissionAck(MissionAck {
        target_system: target.sysid,
        target_component: target.compid,
        r#type: result,
        mission_type,
    })
}

/// Overrides the vehicle's RC channels with stick positions from a joystick.
///
/// Eighteen values, in channel order. The caller decides what each one means - a position, an
/// "ignore this channel", or a "release this channel back to the transmitter" - because the two
/// halves of this message use opposite conventions for those and the decision belongs with the
/// code that knows which convention applies. See `mp_input::mapping`.
///
/// Sent repeatedly while a stick is in use and never held: a vehicle that keeps flying the last
/// position it received is the failure this whole path is shaped around. See `mp_input::Failsafe`.
#[must_use]
pub fn rc_override(target: VehicleId, channels: [u16; 18]) -> MavMessage {
    MavMessage::RcChannelsOverride(RcChannelsOverride {
        chan1_raw: channels[0],
        chan2_raw: channels[1],
        chan3_raw: channels[2],
        chan4_raw: channels[3],
        chan5_raw: channels[4],
        chan6_raw: channels[5],
        chan7_raw: channels[6],
        chan8_raw: channels[7],
        target_system: target.sysid,
        target_component: target.compid,
        chan9_raw: channels[8],
        chan10_raw: channels[9],
        chan11_raw: channels[10],
        chan12_raw: channels[11],
        chan13_raw: channels[12],
        chan14_raw: channels[13],
        chan15_raw: channels[14],
        chan16_raw: channels[15],
        chan17_raw: channels[16],
        chan18_raw: channels[17],
    })
}

/// `MANUAL_CONTROL` as the joystick sends it with Manual Control ticked: the four axes, -1000 to
/// 1000, no buttons, and `target` as the caller gives it - Mission Planner fills it with the
/// vehicle's component id (`rc.target = comPort.MAV.compid`), and so does its port.
/// `// C#: MainV2.cs:2409-2435`
#[must_use]
pub fn manual_control(target: u8, x: i16, y: i16, z: i16, r: i16) -> MavMessage {
    MavMessage::ManualControl(mp_mavlink_dialects::all::ManualControl {
        x,
        y,
        z,
        r,
        buttons: 0,
        target,
        buttons2: 0,
        enabled_extensions: 0,
        s: 0,
        t: 0,
        aux1: 0,
        aux2: 0,
        aux3: 0,
        aux4: 0,
        aux5: 0,
        aux6: 0,
    })
}

// The flight screen's Actions tab and its map menu. Each builder below is one message a handler
// in `GCSViews/FlightData.cs` sends, with the values the C# puts in it; the handler is cited on
// each. The C# names the commands without the `MAV_CMD_` (and `NAV_`) prefix, so its
// `MAV_CMD.DO_CHANGE_SPEED` is `MAV_CMD_DO_CHANGE_SPEED` here.

/// `MAV_CMD_NAV_WAYPOINT`.
pub const CMD_NAV_WAYPOINT: u16 = 16;
/// `MAV_CMD_NAV_LOITER_UNLIM`; the C#'s `MAV_CMD.LOITER_UNLIM`.
pub const CMD_NAV_LOITER_UNLIM: u16 = 17;
/// `MAV_CMD_NAV_RETURN_TO_LAUNCH`; the C#'s `MAV_CMD.RETURN_TO_LAUNCH`.
pub const CMD_NAV_RETURN_TO_LAUNCH: u16 = 20;
/// `MAV_CMD_DO_CHANGE_SPEED`.
pub const CMD_DO_CHANGE_SPEED: u16 = 178;
/// `MAV_CMD_DO_SET_HOME`.
pub const CMD_DO_SET_HOME: u16 = 179;
/// `MAV_CMD_DO_FLIGHTTERMINATION`.
pub const CMD_DO_FLIGHTTERMINATION: u16 = 185;
/// `MAV_CMD_DO_GO_AROUND`.
pub const CMD_DO_GO_AROUND: u16 = 191;
/// `MAV_CMD_DO_DIGICAM_CONTROL`.
pub const CMD_DO_DIGICAM_CONTROL: u16 = 203;
/// `MAV_CMD_DO_PARACHUTE`.
pub const CMD_DO_PARACHUTE: u16 = 208;
/// `MAV_CMD_DO_ENGINE_CONTROL`.
pub const CMD_DO_ENGINE_CONTROL: u16 = 223;
/// `MAV_CMD_PREFLIGHT_CALIBRATION`.
pub const CMD_PREFLIGHT_CALIBRATION: u16 = 241;
/// `MAV_CMD_MISSION_START`.
pub const CMD_MISSION_START: u16 = 300;
/// `MAV_CMD_STORAGE_FORMAT`.
pub const CMD_STORAGE_FORMAT: u16 = 526;
/// `MAV_CMD_CONTROL_HIGH_LATENCY`.
pub const CMD_CONTROL_HIGH_LATENCY: u16 = 2600;
/// `MAV_CMD_BATTERY_RESET`.
pub const CMD_BATTERY_RESET: u16 = 42_651;
/// `MAV_CMD_SCRIPTING`.
pub const CMD_SCRIPTING: u16 = 42_701;
/// `MAV_FRAME_GLOBAL`: altitude above mean sea level. The frame `doCommandInt` defaults to.
pub const FRAME_GLOBAL: u8 = 0;
/// `MAV_FRAME_GLOBAL_RELATIVE_ALT`: altitude above home.
pub const FRAME_GLOBAL_RELATIVE_ALT: u8 = 3;
/// `MAV_FRAME_GLOBAL_TERRAIN_ALT`: altitude above the terrain.
pub const FRAME_GLOBAL_TERRAIN_ALT: u8 = 10;
/// `MAV_MODE_FLAG_SAFETY_ARMED`, which the C# ORs into `SET_MODE` to toggle the safety switch.
pub const MODE_FLAG_SAFETY_ARMED: u8 = 128;
/// `MISSION_ITEM.current` = 2: "this is a guided-mode target", not a mission item.
pub const CURRENT_GUIDED: u8 = 2;
/// `MISSION_ITEM.current` = 3: "change the altitude of the guided target".
pub const CURRENT_CHANGE_ALT: u8 = 3;

/// A `COMMAND_LONG` for any command, with its seven parameters as given.
///
/// The public form of the builder every named command here uses, for the flight screen's
/// `CMB_action` list, which sends whichever `MAV_CMD` its entry names.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2695-2710` (`doCommandAsync`)
#[must_use]
pub fn command_long(target: VehicleId, command_id: u16, params: [f32; 7]) -> MavMessage {
    command(target, command_id, params)
}

/// A `COMMAND_INT`, as `doCommandIntAsync` fills it: `current` and `autocontinue` zero, the frame
/// as given (the C# defaults it to `GLOBAL`).
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2847-2881`
#[must_use]
pub fn command_int(
    target: VehicleId,
    command_id: u16,
    frame: u8,
    params: [f32; 4],
    x: i32,
    y: i32,
    z: f32,
) -> MavMessage {
    MavMessage::CommandInt(CommandInt {
        param1: params[0],
        param2: params[1],
        param3: params[2],
        param4: params[3],
        x,
        y,
        z,
        command: command_id,
        target_system: target.sysid,
        target_component: target.compid,
        frame,
        current: 0,
        autocontinue: 0,
    })
}

/// Makes a mission item the one the vehicle flies to next: `setWPCurrent`, behind Set WP and
/// Restart Mission.
///
/// The C# re-sends this every 2000 ms, five times, until a `MISSION_CURRENT` arrives from the
/// vehicle; this builds the message, and whoever sends it owns the retries.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2452-2501`
#[must_use]
pub fn mission_set_current(target: VehicleId, seq: u16) -> MavMessage {
    MavMessage::MissionSetCurrent(MissionSetCurrent {
        seq,
        target_system: target.sysid,
        target_component: target.compid,
    })
}

/// A float `MISSION_ITEM` as `setWPAsync` builds one when `use_int` is false: `x` is the
/// latitude and `y` the longitude, each cast to `f32`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3996-4043`
fn float_mission_item(
    target: VehicleId,
    frame: u8,
    current: u8,
    (latitude, longitude, altitude): (f64, f64, f32),
) -> MavMessage {
    #[allow(clippy::cast_possible_truncation)] // the C# casts to float; so does the wire field
    let (x, y) = (latitude as f32, longitude as f32);
    MavMessage::MissionItem(FloatItem {
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x,
        y,
        z: altitude,
        seq: 0,
        command: CMD_NAV_WAYPOINT,
        target_system: target.sysid,
        target_component: target.compid,
        frame,
        current,
        autocontinue: 1,
        mission_type: 0,
    })
}

/// Changes the height of the guided target: Change Alt.
///
/// `setNewWPAlt` sends a `MISSION_ITEM` with `current` = 3, sequence 0, frame
/// `GLOBAL_RELATIVE_ALT`, and a location whose only non-zero field is the altitude. The C# waits
/// 450 ms for a `MISSION_ACK` and re-sends up to ten times.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4470-4481, 3975-4043`
#[must_use]
pub fn change_alt(target: VehicleId, altitude_metres: f32) -> MavMessage {
    float_mission_item(
        target,
        FRAME_GLOBAL_RELATIVE_ALT,
        CURRENT_CHANGE_ALT,
        (0.0, 0.0, altitude_metres),
    )
}

/// A guided-mode target as `setGuidedModeWP` sends it to ArduPlane: a `MISSION_ITEM` with
/// `current` = 2 in the frame the operator chose.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4441-4448`
#[must_use]
pub fn guided_mission_item(
    target: VehicleId,
    frame: u8,
    latitude: f64,
    longitude: f64,
    altitude: f32,
) -> MavMessage {
    float_mission_item(
        target,
        frame,
        CURRENT_GUIDED,
        (latitude, longitude, altitude),
    )
}

/// A guided-mode target as `setGuidedModeWP` sends it to everything but ArduPlane:
/// `setPositionTargetGlobalInt` with only the position enabled.
///
/// The type mask is built the C#'s way: start from every bit set, clear `FORCE`, then clear the
/// position bits - or only the altitude bit when there is no latitude and longitude. With a
/// position that is `0xFDF8`. The frame is whichever the operator chose, not the `_INT` variant;
/// ArduPilot treats the two alike for this message.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4500-4551, 4450-4454`
#[must_use]
pub fn guided_position_target(
    target: VehicleId,
    frame: u8,
    latitude: f64,
    longitude: f64,
    altitude: f64,
) -> MavMessage {
    const POS_IGNORE: u16 = 0b111;
    const ALT_IGNORE: u16 = 0b100;
    const FORCE: u16 = 1 << 9;
    let mut type_mask = u16::MAX - FORCE;
    if latitude != 0.0 && longitude != 0.0 {
        type_mask -= POS_IGNORE;
    }
    if latitude == 0.0 && longitude == 0.0 {
        type_mask -= ALT_IGNORE;
    }
    // `(int)(lat * 1e7)`: truncation toward zero, which `as` also does.
    #[allow(clippy::cast_possible_truncation)]
    let (lat_int, lon_int, alt) = (
        (latitude * 1e7) as i32,
        (longitude * 1e7) as i32,
        altitude as f32,
    );
    MavMessage::SetPositionTargetGlobalInt(SetPositionTargetGlobalInt {
        time_boot_ms: 0,
        lat_int,
        lon_int,
        alt,
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
        afx: 0.0,
        afy: 0.0,
        afz: 0.0,
        yaw: 0.0,
        yaw_rate: 0.0,
        type_mask,
        target_system: target.sysid,
        target_component: target.compid,
        coordinate_frame: frame,
    })
}

/// Changes the speed: Change Speed.
///
/// `MAV_CMD_DO_CHANGE_SPEED` with param1 0 (speed type) and the number in the box as param2.
/// The C# does not divide it by `CurrentState.multiplierspeed` - it sends whatever the box holds.
/// `// C#: GCSViews/FlightData.cs:4426-4438`
#[must_use]
pub fn change_speed(target: VehicleId, speed: f32) -> MavMessage {
    command(
        target,
        CMD_DO_CHANGE_SPEED,
        [0.0, speed, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
}

/// Abandons a landing: Abort Landing, which is `doAbortLand`, `MAV_CMD_DO_GO_AROUND` with every
/// parameter zero.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2660-2663`
#[must_use]
pub fn go_around(target: VehicleId) -> MavMessage {
    command(target, CMD_DO_GO_AROUND, [0.0; 7])
}

/// Moves the home position: Set Home Here, `MAV_CMD_DO_SET_HOME` as a `COMMAND_INT` with the
/// point in `x`/`y` at 1e7 and the terrain altitude in `z`.
/// `// C#: GCSViews/FlightData.cs:4871-4874`
#[must_use]
pub fn set_home(target: VehicleId, latitude: f64, longitude: f64, altitude: f64) -> MavMessage {
    #[allow(clippy::cast_possible_truncation)] // `(int)(lat * 1e7)` and `(float)alt` in the C#
    let (x, y, z) = (
        (latitude * 1e7) as i32,
        (longitude * 1e7) as i32,
        altitude as f32,
    );
    command_int(target, CMD_DO_SET_HOME, FRAME_GLOBAL, [0.0; 4], x, y, z)
}

/// Moves the estimator's origin: Set EKF Origin Here, `SET_GPS_GLOBAL_ORIGIN`.
///
/// The altitude is `(int) alt.alt * 1000`: the cast binds first, so the height is truncated to a
/// whole metre before it is made millimetres. `time_usec` is left at zero, as the C# leaves it.
/// `// C#: GCSViews/FlightData.cs:4802-4810`
#[must_use]
pub fn set_gps_global_origin(
    target_system: u8,
    latitude: f64,
    longitude: f64,
    altitude: f64,
) -> MavMessage {
    #[allow(clippy::cast_possible_truncation)]
    let (lat, lon, alt) = (
        (latitude * 1e7) as i32,
        (longitude * 1e7) as i32,
        (altitude as i32).saturating_mul(1000),
    );
    MavMessage::SetGpsGlobalOrigin(SetGpsGlobalOrigin {
        latitude: lat,
        longitude: lon,
        altitude: alt,
        target_system,
        time_usec: 0,
    })
}

/// `MAV_CMD_DO_SET_MODE`, the "new" half of the C#'s `setMode`: param1 the base mode, param2 the
/// custom mode, sent without waiting for an answer.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4631-4641`
#[must_use]
pub fn do_set_mode(target: VehicleId, base_mode: u8, custom_mode: u32) -> MavMessage {
    #[allow(clippy::cast_precision_loss)] // mode numbers are small; the C# passes them as float
    let custom = custom_mode as f32;
    command(
        target,
        CMD_DO_SET_MODE,
        [f32::from(base_mode), custom, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
}

/// `SET_MODE` with a base mode of the caller's choosing - the "old" half of `setMode`, which the
/// C# sends twice. [`set_mode`] is this with `CUSTOM_MODE_ENABLED`; Toggle Safety Switch sends it
/// with `SAFETY_ARMED` instead.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4631-4641, GCSViews/FlightData.cs:1827`
#[must_use]
pub fn set_mode_with_base(target_system: u8, base_mode: u8, custom_mode: u32) -> MavMessage {
    MavMessage::SetMode(SetMode {
        custom_mode,
        target_system,
        base_mode,
    })
}

/// The ground station's clock: Do Action's `System_Time`, a `SYSTEM_TIME` with `time_boot_ms`
/// zero.
/// `// C#: GCSViews/FlightData.cs:1755-1772`
#[must_use]
pub fn system_time(time_unix_usec: u64) -> MavMessage {
    MavMessage::SystemTime(SystemTime {
        time_unix_usec,
        time_boot_ms: 0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mp_mavlink_dialects::all::MavMessage;

    fn target() -> VehicleId {
        VehicleId::new(1, 1)
    }

    /// The values ArduPilot looks for to mean "ignore your own checks", one each way.
    ///
    /// Pinned here against Mission Planner's own constants - `magic_force_arm_value = 2989.0f`
    /// and `magic_force_disarm_value = 21196.0f` in `MAVLinkInterface.cs:2634-2635` - because a
    /// wrong number here does not fail loudly. Until 2026-09-24 the arm side sent the disarm
    /// value, so the Force Arm button was an ordinary arm the vehicle checked and refused: the
    /// operator was left pressing a button that did nothing at the moment they most needed it.
    const FORCE_ARM: f32 = 2989.0;
    const FORCE_DISARM: f32 = 21196.0;

    fn params(message: &MavMessage) -> [f32; 7] {
        match message {
            MavMessage::CommandLong(command) => [
                command.param1,
                command.param2,
                command.param3,
                command.param4,
                command.param5,
                command.param6,
                command.param7,
            ],
            other => panic!("expected a COMMAND_LONG, got {}", other.name()),
        }
    }

    #[test]
    fn an_ordinary_arm_does_not_force() {
        // param2 must be zero, or every arm would bypass the checks and the distinction would be
        // silently meaningless.
        let message = arm(target(), true, false);
        let p = params(&message);
        assert!(
            (p[0] - 1.0).abs() < f32::EPSILON,
            "param1 should ask to arm"
        );
        assert!(
            p[1].abs() < f32::EPSILON,
            "param2 should be zero, got {}",
            p[1]
        );
    }

    #[test]
    fn forcing_an_arm_sends_the_arm_magic_the_firmware_looks_for() {
        let message = arm(target(), true, true);
        let p = params(&message);
        assert!((p[0] - 1.0).abs() < f32::EPSILON);
        assert!(
            (p[1] - FORCE_ARM).abs() < f32::EPSILON,
            "param2 should be {FORCE_ARM}, got {}",
            p[1]
        );
    }

    #[test]
    fn forcing_a_disarm_sends_the_disarm_magic_which_is_a_different_number() {
        let message = arm(target(), false, true);
        let p = params(&message);
        assert!(p[0].abs() < f32::EPSILON, "param1 should ask to disarm");
        assert!(
            (p[1] - FORCE_DISARM).abs() < f32::EPSILON,
            "param2 should be {FORCE_DISARM}, got {}",
            p[1]
        );
        assert_ne!(FORCE_ARM, FORCE_DISARM);
    }

    /// The two constants, read out of the C# file itself when the tree is present.
    #[test]
    fn the_magic_values_are_mission_planners_own() {
                // `MP_SRC` names a clone of https://github.com/ArduPilot/MissionPlanner.
        let Some(tree) = std::env::var_os("MP_SRC") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let path = std::path::PathBuf::from(tree)
            .join("ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs");
        let Ok(source) = mp_os::fs::read_to_string(&path) else {
            eprintln!("skipped: no C# tree at {}", path.display());
            return;
        };
        assert!(source.contains("const float magic_force_arm_value = 2989.0f;"));
        assert!(source.contains("const float magic_force_disarm_value = 21196.0f;"));
        assert!((MAGIC_FORCE_ARM - 2989.0).abs() < f32::EPSILON);
        assert!((MAGIC_FORCE_DISARM - 21196.0).abs() < f32::EPSILON);
    }

    #[test]
    fn disarming_asks_to_disarm() {
        let message = arm(target(), false, false);
        let p = params(&message);
        assert!(p[0].abs() < f32::EPSILON, "param1 should ask to disarm");
        assert!(p[1].abs() < f32::EPSILON);
    }

    #[test]
    fn a_reboot_asks_for_a_reboot_and_not_a_shutdown() {
        // param1 = 2 shuts the vehicle down, which from a ground station means it has to be
        // reached physically to come back. Getting this wrong would be discovered in a field.
        let p = params(&reboot(target()));
        assert!(
            (p[0] - 1.0).abs() < f32::EPSILON,
            "param1 should be 1, got {}",
            p[0]
        );
        assert!(
            p[1..].iter().all(|value| value.abs() < f32::EPSILON),
            "{p:?}"
        );
    }

    #[test]
    fn a_command_is_addressed_to_the_vehicle_it_names() {
        match arm(VehicleId::new(7, 42), true, false) {
            MavMessage::CommandLong(command) => {
                assert_eq!(command.target_system, 7);
                assert_eq!(command.target_component, 42);
            }
            other => panic!("expected a COMMAND_LONG, got {}", other.name()),
        }
    }
    /// Every channel lands in the field the wire expects, including across the gap where
    /// `target_system` and `target_component` sit between channel 8 and channel 9.
    ///
    /// Untested, a transposition here is invisible: the message encodes, the vehicle accepts it,
    /// and the aircraft rolls when the pilot asked it to climb. Each channel gets a distinct value
    /// so a swap cannot hide behind two equal numbers.
    #[test]
    fn every_rc_channel_lands_in_its_own_field() {
        let target = VehicleId {
            sysid: 7,
            compid: 42,
        };
        // 1001, 1002, ... 1018 - inside the safe band and unique per channel.
        let mut channels = [0u16; 18];
        for (index, slot) in channels.iter_mut().enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            {
                *slot = 1001 + index as u16;
            }
        }

        let MavMessage::RcChannelsOverride(message) = rc_override(target, channels) else {
            panic!("rc_override must build an RC_CHANNELS_OVERRIDE");
        };

        assert_eq!(message.target_system, 7);
        assert_eq!(message.target_component, 42);
        let placed = [
            message.chan1_raw,
            message.chan2_raw,
            message.chan3_raw,
            message.chan4_raw,
            message.chan5_raw,
            message.chan6_raw,
            message.chan7_raw,
            message.chan8_raw,
            message.chan9_raw,
            message.chan10_raw,
            message.chan11_raw,
            message.chan12_raw,
            message.chan13_raw,
            message.chan14_raw,
            message.chan15_raw,
            message.chan16_raw,
            message.chan17_raw,
            message.chan18_raw,
        ];
        assert_eq!(placed, channels);
    }

    /// A release built by `mp_input` must survive the trip into the message unchanged - both
    /// halves, with their opposite conventions.
    #[test]
    fn a_release_survives_being_put_on_the_wire() {
        let target = VehicleId {
            sysid: 1,
            compid: 1,
        };
        // The same values mp_input::mapping::Channels::release() produces, written out here so
        // this test fails if either side changes without the other.
        let mut release = [u16::MAX - 1; 18];
        for slot in release.iter_mut().take(8) {
            *slot = 0;
        }

        let MavMessage::RcChannelsOverride(message) = rc_override(target, release) else {
            panic!("rc_override must build an RC_CHANNELS_OVERRIDE");
        };
        assert_eq!(message.chan1_raw, 0, "channels 1-8 release with 0");
        assert_eq!(message.chan8_raw, 0);
        assert_eq!(
            message.chan9_raw,
            u16::MAX - 1,
            "channels 9-18 release with UINT16_MAX-1, not 0"
        );
        assert_eq!(message.chan18_raw, u16::MAX - 1);
    }

    /// `getVersion` sends all three requests for `AUTOPILOT_VERSION`, each addressed to the
    /// vehicle: without them ArduPilot never sends the message, so the capabilities stay 0 and
    /// the MAVFtp page never lists. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5847-5861`
    #[test]
    fn the_version_request_asks_all_three_ways() {
        let target = VehicleId::new(7, 42);
        let [request_message, request_capabilities, version_request] = get_version(target);
        let (command, params) = long(&request_message);
        assert_eq!(command, 512, "MAV_CMD_REQUEST_MESSAGE");
        assert_eq!(params[0], 148.0, "AUTOPILOT_VERSION is message 148");
        assert_eq!(&params[1..], &[0.0; 6]);
        let (command, params) = long(&request_capabilities);
        assert_eq!(command, 520, "MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES");
        assert_eq!(params, [0.0; 7]);
        let MavMessage::AutopilotVersionRequest(message) = version_request else {
            panic!("the third is AUTOPILOT_VERSION_REQUEST");
        };
        assert_eq!((message.target_system, message.target_component), (7, 42));
        for message in get_version(target) {
            if let MavMessage::CommandLong(command) = message {
                assert_eq!((command.target_system, command.target_component), (7, 42));
            }
        }
    }

    #[test]
    fn the_banner_request_is_do_send_banner_with_no_arguments() {
        let MavMessage::CommandLong(message) = send_banner(VehicleId {
            sysid: 1,
            compid: 1,
        }) else {
            panic!("send_banner must build a COMMAND_LONG");
        };
        assert_eq!(message.command, CMD_DO_SEND_BANNER);
        assert_eq!(message.target_system, 1);
        assert_eq!(message.param1, 0.0);
    }

    fn long(message: &MavMessage) -> (u16, [f32; 7]) {
        match message {
            MavMessage::CommandLong(command) => (command.command, params(message)),
            other => panic!("expected a COMMAND_LONG, got {}", other.name()),
        }
    }

    /// Set WP and Restart Mission: `mavlink_mission_set_current_t` with the target and the
    /// index, nothing else. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2461-2468`
    #[test]
    fn set_current_carries_the_index_and_the_vehicle() {
        let MavMessage::MissionSetCurrent(message) = mission_set_current(VehicleId::new(7, 42), 3)
        else {
            panic!("mission_set_current must build a MISSION_SET_CURRENT");
        };
        assert_eq!(message.seq, 3);
        assert_eq!(message.target_system, 7);
        assert_eq!(message.target_component, 42);
    }

    /// Change Alt: a float `MISSION_ITEM`, sequence 0, `current` 3, frame 3, a waypoint whose
    /// only non-zero field is the altitude. `// C#: MAVLinkInterface.cs:4476, 4027-4043`
    #[test]
    fn change_alt_is_a_current_3_mission_item_in_the_relative_frame() {
        let MavMessage::MissionItem(item) = change_alt(target(), 25.0) else {
            panic!("change_alt must build a MISSION_ITEM, not the _INT form the C# does not use");
        };
        assert_eq!(item.seq, 0);
        assert_eq!(item.command, CMD_NAV_WAYPOINT);
        assert_eq!(item.frame, FRAME_GLOBAL_RELATIVE_ALT);
        assert_eq!(item.current, 3);
        assert_eq!(item.autocontinue, 1);
        assert_eq!(item.mission_type, 0);
        assert_eq!((item.x, item.y, item.z), (0.0, 0.0, 25.0));
        assert_eq!(
            (item.param1, item.param2, item.param3, item.param4),
            (0.0, 0.0, 0.0, 0.0)
        );
    }

    /// ArduPlane's guided target: the same item with `current` 2, in the chosen frame, with the
    /// position cast to float as `setWPAsync` casts it.
    #[test]
    fn a_plane_guided_target_is_a_current_2_mission_item() {
        let MavMessage::MissionItem(item) = guided_mission_item(
            target(),
            FRAME_GLOBAL_TERRAIN_ALT,
            -35.363_261,
            149.165_23,
            40.0,
        ) else {
            panic!("guided_mission_item must build a MISSION_ITEM");
        };
        assert_eq!(item.current, 2);
        assert_eq!(item.frame, 10);
        assert_eq!(item.x, -35.363_261_f32);
        assert_eq!(item.y, 149.165_23_f32);
        assert_eq!(item.z, 40.0);
    }

    /// Everything else's guided target: `setPositionTargetGlobalInt(pos: true)`. The mask is the
    /// C#'s arithmetic - `ushort.MaxValue - FORCE - POS_IGNORE` - which is 0xFDF8, not the
    /// 0x0FF8 `goto_position` uses; the frame is the operator's, not the `_INT` variant.
    #[test]
    fn a_guided_position_target_has_the_csharp_mask_and_frame() {
        let MavMessage::SetPositionTargetGlobalInt(message) = guided_position_target(
            target(),
            FRAME_GLOBAL_RELATIVE_ALT,
            -35.363_261_7,
            149.165_23,
            20.0,
        ) else {
            panic!("guided_position_target must build a SET_POSITION_TARGET_GLOBAL_INT");
        };
        assert_eq!(message.type_mask, 0xFDF8);
        assert_eq!(message.type_mask, 65_535 - 512 - 7);
        assert_eq!(message.coordinate_frame, 3);
        // `(int)(lat * 1e7)` truncates toward zero.
        assert_eq!(message.lat_int, -353_632_617);
        assert_eq!(message.lon_int, 1_491_652_300);
        assert_eq!(message.alt, 20.0);
        assert_eq!((message.vx, message.yaw, message.yaw_rate), (0.0, 0.0, 0.0));

        // With no position the C# clears only the altitude-ignore bit.
        let MavMessage::SetPositionTargetGlobalInt(message) =
            guided_position_target(target(), FRAME_GLOBAL_RELATIVE_ALT, 0.0, 0.0, 20.0)
        else {
            panic!("guided_position_target must build a SET_POSITION_TARGET_GLOBAL_INT");
        };
        assert_eq!(message.type_mask, 0xFDFB);
    }

    /// Change Speed: `DO_CHANGE_SPEED`, param1 0, the box's number in param2, the rest zero.
    #[test]
    fn change_speed_puts_the_speed_in_param2() {
        let (command_id, p) = long(&change_speed(target(), 7.5));
        assert_eq!(command_id, 178);
        assert_eq!(p, [0.0, 7.5, 0.0, 0.0, 0.0, 0.0, 0.0]);
    }

    /// Abort Landing: `DO_GO_AROUND`, every parameter zero.
    #[test]
    fn abort_landing_is_a_go_around_with_no_arguments() {
        let (command_id, p) = long(&go_around(target()));
        assert_eq!(command_id, 191);
        assert_eq!(p, [0.0; 7]);
    }

    /// Set Home Here: `DO_SET_HOME` as a `COMMAND_INT` - frame `GLOBAL`, p1-p4 zero, the point at
    /// 1e7 in x and y, the altitude in z, `current` and `autocontinue` zero.
    #[test]
    fn set_home_is_a_command_int_with_the_point_in_x_and_y() {
        let MavMessage::CommandInt(message) = set_home(target(), -35.363_261_7, 149.165_23, 584.25)
        else {
            panic!("set_home must build a COMMAND_INT");
        };
        assert_eq!(message.command, 179);
        assert_eq!(message.frame, 0);
        assert_eq!((message.x, message.y), (-353_632_617, 1_491_652_300));
        assert_eq!(message.z, 584.25);
        assert_eq!(
            (
                message.param1,
                message.param2,
                message.param3,
                message.param4
            ),
            (0.0, 0.0, 0.0, 0.0)
        );
        assert_eq!((message.current, message.autocontinue), (0, 0));
    }

    /// Set EKF Origin Here: millimetres of a height truncated to whole metres first, because
    /// `(int) alt.alt * 1000` casts before it multiplies.
    #[test]
    fn the_ekf_origin_height_is_whole_metres_in_millimetres() {
        let MavMessage::SetGpsGlobalOrigin(message) =
            set_gps_global_origin(1, -35.363_261_7, 149.165_23, 584.9)
        else {
            panic!("set_gps_global_origin must build a SET_GPS_GLOBAL_ORIGIN");
        };
        assert_eq!(message.altitude, 584_000);
        assert_eq!(
            (message.latitude, message.longitude),
            (-353_632_617, 1_491_652_300)
        );
        assert_eq!(message.target_system, 1);
        assert_eq!(message.time_usec, 0);
    }

    /// `setMode`'s new half: `DO_SET_MODE` with the base mode in param1 and the custom mode in
    /// param2; its old half: `SET_MODE` with whichever base mode the caller asked for.
    #[test]
    fn set_mode_halves_carry_the_base_and_custom_modes() {
        let (command_id, p) = long(&do_set_mode(target(), 1, 4));
        assert_eq!(command_id, 176);
        assert_eq!(p, [1.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

        let MavMessage::SetMode(message) = set_mode_with_base(3, MODE_FLAG_SAFETY_ARMED, 1) else {
            panic!("set_mode_with_base must build a SET_MODE");
        };
        assert_eq!(
            (
                message.target_system,
                message.base_mode,
                message.custom_mode
            ),
            (3, 128, 1)
        );
    }

    /// `System_Time`: the clock in microseconds and `time_boot_ms` zero.
    #[test]
    fn system_time_sends_the_clock_and_no_boot_time() {
        let MavMessage::SystemTime(message) = system_time(1_700_000_000_000_000) else {
            panic!("system_time must build a SYSTEM_TIME");
        };
        assert_eq!(message.time_unix_usec, 1_700_000_000_000_000);
        assert_eq!(message.time_boot_ms, 0);
    }

    /// The generic builders put every argument where the wire expects it.
    #[test]
    fn the_generic_builders_place_every_argument() {
        let (command_id, p) = long(&command_long(
            target(),
            42_651,
            [255.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ));
        assert_eq!(command_id, CMD_BATTERY_RESET);
        assert_eq!(p, [255.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

        let MavMessage::CommandInt(message) = command_int(
            target(),
            CMD_SCRIPTING,
            FRAME_GLOBAL,
            [3.0, 0.0, 0.0, 0.0],
            0,
            0,
            0.0,
        ) else {
            panic!("command_int must build a COMMAND_INT");
        };
        assert_eq!(message.command, 42_701);
        assert_eq!(message.param1, 3.0);
    }
}
