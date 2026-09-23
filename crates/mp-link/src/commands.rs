//! Vehicle commands.
//!
//! Thin, named constructors over the raw messages. The value is not abstraction, it is that
//! `arm(id)` cannot be built with the arm/disarm parameter in the wrong slot, and that every
//! magic number has exactly one definition with a comment saying what it is.

use mp_mavlink_dialects::all::{
    CommandLong, MavMessage, MissionAck, MissionCount, MissionItemInt, MissionRequestInt,
    MissionRequestList, ParamRequestList, ParamRequestRead, ParamSet, RcChannelsOverride, SetMode,
    SetPositionTargetGlobalInt,
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
        param_id: crate::params::encode_param_id(name),
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

#[cfg(test)]
mod tests {
    use super::*;
    use mp_mavlink_dialects::all::MavMessage;

    fn target() -> VehicleId {
        VehicleId::new(1, 1)
    }

    /// The value ArduPilot and the MAVLink definitions both use to mean "ignore your own checks".
    ///
    /// Pinned here against Mission Planner's own constant - `magic_force_disarm_value = 21196.0f`
    /// in `ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs` - because a wrong number here would not
    /// fail loudly. The vehicle would simply refuse to arm and the operator would be left pressing
    /// a button that does nothing at the moment they most need it.
    const FORCE_MAGIC: f32 = 21196.0;

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
    fn forcing_sends_the_magic_the_firmware_looks_for() {
        let message = arm(target(), true, true);
        let p = params(&message);
        assert!((p[0] - 1.0).abs() < f32::EPSILON);
        assert!(
            (p[1] - FORCE_MAGIC).abs() < f32::EPSILON,
            "param2 should be {FORCE_MAGIC}, got {}",
            p[1]
        );
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
}
