//! Vehicle commands.
//!
//! Thin, named constructors over the raw messages. The value is not abstraction, it is that
//! `arm(id)` cannot be built with the arm/disarm parameter in the wrong slot, and that every
//! magic number has exactly one definition with a comment saying what it is.

use mp_mavlink_dialects::all::{
    CommandLong, MavMessage, ParamSet, SetMode, SetPositionTargetGlobalInt,
};
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

fn command(target: VehicleId, command: u16, params: [f32; 7]) -> MavMessage {
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

/// Arms or disarms the vehicle.
///
/// `force` sends the magic 21196 that bypasses the autopilot's safety checks. It exists because
/// the protocol has it and ground crews occasionally need it; it is never the default.
#[must_use]
pub fn arm(target: VehicleId, arm: bool, force: bool) -> MavMessage {
    let force_magic = if force { 21196.0 } else { 0.0 };
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
