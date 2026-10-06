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

//! `GimbalManagerProtocol`: what a vehicle's gimbal manager says about its gimbals, and the
//! `DO_GIMBAL_MANAGER_PITCHYAW` and `DO_SET_ROI_LOCATION` commands the gimbal video control sends
//! it.
//!
//! The C# makes one for the autopilot and for each component from `MAV_COMP_ID_MISSIONPLANNER`
//! to `MAV_COMP_ID_ONBOARD_COMPUTER4` when that component's first heartbeat arrives, and calls
//! `Discover` on it two seconds later and each time `UpdateCurrentSettings` asks for the telemetry
//! streams. The first `Discover` subscribes it to every packet the link receives - from any
//! vehicle, as the C#'s handler does not look at who sent it - and each asks for
//! `GIMBAL_MANAGER_INFORMATION` from everyone (system 0, component 0) without waiting for an
//! answer. It keeps the last `GIMBAL_MANAGER_INFORMATION`, `GIMBAL_MANAGER_STATUS` and
//! `GIMBAL_DEVICE_ATTITUDE_STATUS` per gimbal device, and under index 0 the one of the lowest
//! device id heard. The link thread does the making and the discovering (`lib.rs`); this is the
//! state and the commands.
//! `// C#: ExtLibs/ArduPilot/Mavlink/GimbalManagerProtocol.cs:1-361,
//! ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:565-588, ExtLibs/ArduPilot/CurrentState.cs:4658`
//!
//! Each command answers `None` where the C#'s returns `Task.FromResult(false)` without sending:
//! when the manager has not said it can. The capability checks read index 0 whatever gimbal the
//! command is for, as the C#'s `HasCapability(flag)` calls leave the device id at its default.
//!
//! Not ported, because nothing calls them: `HasAllCapability`, `SetRCYawLockAsync`,
//! `SetAnglesStream`, `SetRatesStream`, `SetROINoneAsync` and `SetROISysIDAsync`.

use std::collections::BTreeMap;

use mp_mavlink_dialects::all::{
    GimbalDeviceAttitudeStatus, GimbalDeviceFlags, GimbalManagerCapFlags, GimbalManagerFlags,
    GimbalManagerInformation, GimbalManagerStatus, MavCmd, MavMessage,
};
use mp_vehicle::VehicleId;

use crate::commands;

/// `MAVLINK_MSG_ID.GIMBAL_MANAGER_INFORMATION`.
const GIMBAL_MANAGER_INFORMATION: f32 = 280.0;

/// A `MAV_CMD` as the `u16` a `COMMAND_LONG` carries.
fn cmd(command: MavCmd) -> u16 {
    u16::try_from(command.0).unwrap_or(u16::MAX)
}

/// `MissionPlanner.Utilities.Quaternion`: `q1` to `q4`, w, x, y, z, as far as the gimbal video
/// control and these protocols use it. `// C#: ExtLibs/Utilities/Quaternion.cs:9-418`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    /// `q1`, w.
    pub q1: f64,
    /// `q2`, x.
    pub q2: f64,
    /// `q3`, y.
    pub q3: f64,
    /// `q4`, z.
    pub q4: f64,
}

impl Default for Quaternion {
    /// `new Quaternion()`: no rotation. `// C#: ExtLibs/Utilities/Quaternion.cs:16-20`
    fn default() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0)
    }
}

impl Quaternion {
    /// `new Quaternion(q1, q2, q3, q4)`, not normalised.
    #[must_use]
    pub const fn new(q1: f64, q2: f64, q3: f64, q4: f64) -> Self {
        Self { q1, q2, q3, q4 }
    }

    /// `from_euler(roll, pitch, yaw)`, in radians. `// C#: ExtLibs/Utilities/Quaternion.cs:104-119`
    #[must_use]
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (sr2, cr2) = (roll * 0.5).sin_cos();
        let (sp2, cp2) = (pitch * 0.5).sin_cos();
        let (sy2, cy2) = (yaw * 0.5).sin_cos();
        Self::new(
            cr2 * cp2 * cy2 + sr2 * sp2 * sy2,
            sr2 * cp2 * cy2 - cr2 * sp2 * sy2,
            cr2 * sp2 * cy2 + sr2 * cp2 * sy2,
            cr2 * cp2 * sy2 - sr2 * sp2 * cy2,
        )
    }

    /// `from_axis_angle(axis, theta)`: `axis` must be a unit vector.
    /// `// C#: ExtLibs/Utilities/Quaternion.cs:142-157`
    #[must_use]
    #[allow(clippy::float_cmp)] // the C#'s `theta == 0`
    pub fn from_axis_angle(axis: Vector3, theta: f64) -> Self {
        if theta == 0.0 {
            return Self::default();
        }
        let st2 = (theta / 2.0).sin();
        Self::new(
            (theta / 2.0).cos(),
            axis.x * st2,
            axis.y * st2,
            axis.z * st2,
        )
    }

    /// `get_euler_roll()`. `// C#: ExtLibs/Utilities/Quaternion.cs:331-334`
    #[must_use]
    pub fn euler_roll(self) -> f64 {
        (2.0 * (self.q1 * self.q2 + self.q3 * self.q4))
            .atan2(1.0 - 2.0 * (self.q2 * self.q2 + self.q3 * self.q3))
    }

    /// `get_euler_pitch()`. `// C#: ExtLibs/Utilities/Quaternion.cs:340-343`
    #[must_use]
    pub fn euler_pitch(self) -> f64 {
        (2.0 * (self.q1 * self.q3 - self.q4 * self.q2)).asin()
    }

    /// `get_euler_yaw()`. `// C#: ExtLibs/Utilities/Quaternion.cs:349-352`
    #[must_use]
    pub fn euler_yaw(self) -> f64 {
        (2.0 * (self.q1 * self.q4 + self.q2 * self.q3))
            .atan2(1.0 - 2.0 * (self.q3 * self.q3 + self.q4 * self.q4))
    }

    /// `rotation_matrix() * v`: `body_to_earth`. `// C#: ExtLibs/Utilities/Quaternion.cs:188-212, 253-256`
    #[must_use]
    pub fn body_to_earth(self, v: Vector3) -> Vector3 {
        let Self { q1, q2, q3, q4 } = self;
        let (q3q3, q3q4, q2q2, q2q3, q2q4) = (q3 * q3, q3 * q4, q2 * q2, q2 * q3, q2 * q4);
        let (q1q2, q1q3, q1q4, q4q4) = (q1 * q2, q1 * q3, q1 * q4, q4 * q4);
        let a = Vector3::new(
            1.0 - 2.0 * (q3q3 + q4q4),
            2.0 * (q2q3 - q1q4),
            2.0 * (q2q4 + q1q3),
        );
        let b = Vector3::new(
            2.0 * (q2q3 + q1q4),
            1.0 - 2.0 * (q2q2 + q4q4),
            2.0 * (q3q4 - q1q2),
        );
        let c = Vector3::new(
            2.0 * (q2q4 - q1q3),
            2.0 * (q3q4 + q1q2),
            1.0 - 2.0 * (q2q2 + q3q3),
        );
        Vector3::new(a.dot(v), b.dot(v), c.dot(v))
    }
}

impl std::ops::Mul for Quaternion {
    type Output = Self;

    /// `w * v`: applying `v`, then `w`. `// C#: ExtLibs/Utilities/Quaternion.cs:409-417`
    fn mul(self, v: Self) -> Self {
        let w = self;
        Self::new(
            w.q1 * v.q1 - w.q2 * v.q2 - w.q3 * v.q3 - w.q4 * v.q4,
            w.q1 * v.q2 + w.q2 * v.q1 + w.q3 * v.q4 - w.q4 * v.q3,
            w.q1 * v.q3 - w.q2 * v.q4 + w.q3 * v.q1 + w.q4 * v.q2,
            w.q1 * v.q4 + w.q2 * v.q3 - w.q3 * v.q2 + w.q4 * v.q1,
        )
    }
}

/// `MissionPlanner.Utilities.Vector3`, as far as the camera's geometry uses it.
/// `// C#: ExtLibs/Utilities/Vector3.cs:196-290`
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vector3 {
    /// `x`.
    pub x: f64,
    /// `y`.
    pub y: f64,
    /// `z`.
    pub z: f64,
}

impl Vector3 {
    /// `new Vector3(x, y, z)`.
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// `self * v`: the dot product.
    #[must_use]
    pub fn dot(self, v: Self) -> f64 {
        self.x * v.x + self.y * v.y + self.z * v.z
    }

    /// `self % v`: the cross product.
    #[must_use]
    pub fn cross(self, v: Self) -> Self {
        Self::new(
            self.y * v.z - self.z * v.y,
            self.z * v.x - self.x * v.z,
            self.x * v.y - self.y * v.x,
        )
    }

    /// `length()`.
    #[must_use]
    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }

    /// `normalize()`: divided by its length, which leaves NaNs for a zero vector as the C#'s
    /// division does.
    #[must_use]
    pub fn normalized(self) -> Self {
        let length = self.length();
        Self::new(self.x / length, self.y / length, self.z / length)
    }
}

impl std::ops::Neg for Vector3 {
    type Output = Self;

    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

/// `wrap_180`: into (-180, 180] by whole turns. `// C#: GimbalManagerProtocol.cs:220-231`
#[must_use]
pub fn wrap_180(mut angle: f64) -> f64 {
    while angle > 180.0 {
        angle -= 360.0;
    }
    while angle < -180.0 {
        angle += 360.0;
    }
    angle
}

/// One `GimbalManagerProtocol`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GimbalManager {
    /// Whether `Discover` has run, subscribing `MessagesHandler` (`first_discover` false).
    listening: bool,
    /// `ManagerInfo`: the last `GIMBAL_MANAGER_INFORMATION` per device, 0 the lowest heard.
    pub manager_info: BTreeMap<u8, GimbalManagerInformation>,
    /// `ManagerStatus`: the last `GIMBAL_MANAGER_STATUS` per device, 0 the lowest heard.
    pub manager_status: BTreeMap<u8, GimbalManagerStatus>,
    /// `GimbalStatus`: the last `GIMBAL_DEVICE_ATTITUDE_STATUS` per device, 0 the lowest heard.
    pub gimbal_status: BTreeMap<u8, GimbalDeviceAttitudeStatus>,
}

/// `dictionary[id] = value`, and under 0 too when there is none there yet or `id` is no higher
/// than the one there. `// C#: GimbalManagerProtocol.cs:58-62, 68-72, 78-82`
fn keep<T: Copy>(map: &mut BTreeMap<u8, T>, id: u8, value: T, id_of: impl Fn(&T) -> u8) {
    map.insert(id, value);
    let lowest = map.get(&0).is_none_or(|zero| id <= id_of(zero));
    if lowest {
        map.insert(0, value);
    }
}

impl GimbalManager {
    /// Whether `Discover` has subscribed it.
    #[must_use]
    pub const fn is_listening(&self) -> bool {
        self.listening
    }

    /// `Discover`: subscribed to the link's packets the first time, and the request for
    /// `GIMBAL_MANAGER_INFORMATION` to system 0, component 0, which the caller sends once and does
    /// not wait on (`requireack` false). `// C#: GimbalManagerProtocol.cs:39-50`
    pub fn discover(&mut self) -> MavMessage {
        self.listening = true;
        commands::command_long(
            VehicleId::new(0, 0),
            cmd(MavCmd::MAV_CMD_REQUEST_MESSAGE),
            [GIMBAL_MANAGER_INFORMATION, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
    }

    /// `MessagesHandler`, once `Discover` has subscribed it: any sender's.
    /// `// C#: GimbalManagerProtocol.cs:52-84`
    pub fn observe(&mut self, message: &MavMessage) {
        if !self.listening {
            return;
        }
        match message {
            MavMessage::GimbalManagerInformation(gmi) => {
                keep(&mut self.manager_info, gmi.gimbal_device_id, *gmi, |m| {
                    m.gimbal_device_id
                });
            }
            MavMessage::GimbalManagerStatus(gms) => {
                keep(&mut self.manager_status, gms.gimbal_device_id, *gms, |m| {
                    m.gimbal_device_id
                });
            }
            MavMessage::GimbalDeviceAttitudeStatus(gds) => {
                keep(&mut self.gimbal_status, gds.gimbal_device_id, *gds, |m| {
                    m.gimbal_device_id
                });
            }
            _ => {}
        }
    }

    /// `HasCapability(flags, id)`: any of the flags. `// C#: GimbalManagerProtocol.cs:86-89`
    #[must_use]
    pub fn has_capability(&self, flags: GimbalManagerCapFlags, gimbal_device_id: u8) -> bool {
        self.manager_info
            .get(&gimbal_device_id)
            .is_some_and(|info| info.cap_flags & flags.0 != 0)
    }

    /// `HasStatusFlag(flags, id)`, read from `GIMBAL_MANAGER_STATUS` with `GIMBAL_DEVICE_FLAGS`'
    /// values, as the C# reads it. `// C#: GimbalManagerProtocol.cs:96-99`
    #[must_use]
    pub fn has_status_flag(&self, flags: GimbalDeviceFlags, gimbal_device_id: u8) -> bool {
        self.manager_status
            .get(&gimbal_device_id)
            .is_some_and(|status| status.flags & flags.0 != 0)
    }

    /// `YawInVehicleFrame(id)`: the frame flags, or with neither set, not yaw-locked.
    /// `// C#: GimbalManagerProtocol.cs:101-115`
    #[must_use]
    pub fn yaw_in_vehicle_frame(&self, gimbal_device_id: u8) -> bool {
        let earth = self.has_status_flag(
            GimbalDeviceFlags::GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME,
            gimbal_device_id,
        );
        let vehicle = self.has_status_flag(
            GimbalDeviceFlags::GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
            gimbal_device_id,
        );
        if !earth && !vehicle {
            return !self.has_status_flag(
                GimbalDeviceFlags::GIMBAL_DEVICE_FLAGS_YAW_LOCK,
                gimbal_device_id,
            );
        }
        vehicle
    }

    /// `GetAttitude(id)`: the device's quaternion, turned by the vehicle's yaw (`cs.yaw`, degrees)
    /// where its yaw is in the vehicle's frame; `None` before its first attitude.
    /// `// C#: GimbalManagerProtocol.cs:122-137`
    #[must_use]
    pub fn attitude(&self, gimbal_device_id: u8, vehicle_yaw_degrees: f64) -> Option<Quaternion> {
        let status = self.gimbal_status.get(&gimbal_device_id)?;
        let q = Quaternion::new(
            f64::from(status.q[0]),
            f64::from(status.q[1]),
            f64::from(status.q[2]),
            f64::from(status.q[3]),
        );
        if self.yaw_in_vehicle_frame(gimbal_device_id) {
            return Some(Quaternion::from_euler(0.0, 0.0, vehicle_yaw_degrees.to_radians()) * q);
        }
        Some(q)
    }

    /// `DO_GIMBAL_MANAGER_PITCHYAW` as each of these sends it: the angles, the rates, the flags,
    /// an unused parameter and the device.
    fn pitchyaw(
        target: VehicleId,
        values: [f32; 4],
        flags: f32,
        gimbal_device_id: u8,
    ) -> MavMessage {
        commands::command_long(
            target,
            cmd(MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW),
            [
                values[0],
                values[1],
                values[2],
                values[3],
                flags,
                0.0,
                f32::from(gimbal_device_id),
            ],
        )
    }

    /// `RetractAsync(id)`: the `RETRACT` flag, if it has `HAS_RETRACT`.
    /// `// C#: GimbalManagerProtocol.cs:139-156`
    #[must_use]
    pub fn retract(&self, target: VehicleId, gimbal_device_id: u8) -> Option<MavMessage> {
        self.has_capability(
            GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT,
            0,
        )
        .then(|| {
            Self::pitchyaw(
                target,
                [f32::NAN; 4],
                flag(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_RETRACT),
                gimbal_device_id,
            )
        })
    }

    /// `NeutralAsync(id)`: the `NEUTRAL` flag, if it has `HAS_NEUTRAL`.
    /// `// C#: GimbalManagerProtocol.cs:158-175`
    #[must_use]
    pub fn neutral(&self, target: VehicleId, gimbal_device_id: u8) -> Option<MavMessage> {
        self.has_capability(
            GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL,
            0,
        )
        .then(|| {
            Self::pitchyaw(
                target,
                [f32::NAN; 4],
                flag(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_NEUTRAL),
                gimbal_device_id,
            )
        })
    }

    /// `SetAttitudeAsync(q, yaw_lock, id)`: the quaternion's pitch and yaw in degrees, the yaw
    /// made the vehicle's (`- cs.yaw`) when not locked, then [`GimbalManager::set_angles`].
    /// `// C#: GimbalManagerProtocol.cs:205-218`
    #[must_use]
    pub fn set_attitude(
        &self,
        target: VehicleId,
        q: Quaternion,
        yaw_lock: bool,
        gimbal_device_id: u8,
        vehicle_yaw_degrees: f64,
    ) -> Option<MavMessage> {
        let pitch = q.euler_pitch().to_degrees();
        let mut yaw = q.euler_yaw().to_degrees();
        if !yaw_lock {
            yaw -= vehicle_yaw_degrees;
        }
        self.set_angles(target, pitch, yaw, yaw_lock, gimbal_device_id)
    }

    /// `SetAnglesCommandAsync(pitch, yaw, yaw_lock, id)`: the angles wrapped to ±180, if it can
    /// point at a local location and has each axis (and the yaw mode) a non-zero angle needs.
    /// `// C#: GimbalManagerProtocol.cs:233-255`
    #[must_use]
    #[allow(clippy::float_cmp, clippy::cast_possible_truncation)] // the C#'s `!= 0`, `(float)`
    pub fn set_angles(
        &self,
        target: VehicleId,
        pitch: f64,
        yaw: f64,
        yaw_lock: bool,
        gimbal_device_id: u8,
    ) -> Option<MavMessage> {
        let has = |flags| self.has_capability(flags, 0);
        if !has(GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL)
            || (pitch != 0.0
                && !has(GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS))
            || (yaw != 0.0 && !has(GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS))
            || (yaw != 0.0
                && yaw_lock
                && !has(GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK))
            || (yaw != 0.0
                && !yaw_lock
                && !has(GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW))
        {
            return None;
        }
        Some(Self::pitchyaw(
            target,
            [
                wrap_180(pitch) as f32,
                wrap_180(yaw) as f32,
                f32::NAN,
                f32::NAN,
            ],
            lock_flag(yaw_lock),
            gimbal_device_id,
        ))
    }

    /// `SetRatesCommandAsync(pitch_rate, yaw_rate, yaw_in_earth_frame, id)`, in degrees a
    /// second, if it has each axis (and the yaw mode) a non-zero rate needs.
    /// `// C#: GimbalManagerProtocol.cs:273-294`
    #[must_use]
    #[allow(clippy::float_cmp)] // the C#'s `!= 0`
    pub fn set_rates(
        &self,
        target: VehicleId,
        pitch_rate: f32,
        yaw_rate: f32,
        yaw_in_earth_frame: bool,
        gimbal_device_id: u8,
    ) -> Option<MavMessage> {
        let has = |flags| self.has_capability(flags, 0);
        if (pitch_rate != 0.0
            && !has(GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS))
            || (yaw_rate != 0.0
                && !has(GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS))
            || (yaw_rate != 0.0
                && yaw_in_earth_frame
                && !has(GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK))
            || (yaw_rate != 0.0
                && !yaw_in_earth_frame
                && !has(GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW))
        {
            return None;
        }
        Some(Self::pitchyaw(
            target,
            [f32::NAN, f32::NAN, pitch_rate, yaw_rate],
            lock_flag(yaw_in_earth_frame),
            gimbal_device_id,
        ))
    }

    /// `SetROILocationAsync(lat, lon, alt, id, frame)`: `DO_SET_ROI_LOCATION` as a `COMMAND_INT`,
    /// the device in the first parameter, if it can point at a global location.
    /// `// C#: GimbalManagerProtocol.cs:312-329`
    #[must_use]
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(int)` and `(float)`
    pub fn set_roi_location(
        &self,
        target: VehicleId,
        lat: f64,
        lon: f64,
        alt: f64,
        gimbal_device_id: u8,
        frame: u8,
    ) -> Option<MavMessage> {
        if !self.has_capability(
            GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL,
            0,
        ) {
            return None;
        }
        Some(commands::command_int(
            target,
            cmd(MavCmd::MAV_CMD_DO_SET_ROI_LOCATION),
            frame,
            [f32::from(gimbal_device_id), 0.0, 0.0, 0.0],
            (lat * 1e7) as i32,
            (lon * 1e7) as i32,
            alt as f32,
        ))
    }
}

/// A flag as the `float` parameter it is sent in.
#[allow(clippy::cast_precision_loss)] // small flags
const fn flag(flags: GimbalManagerFlags) -> f32 {
    flags.0 as f32
}

/// `yaw_lock ? (float)GIMBAL_MANAGER_FLAGS.YAW_LOCK : 0`.
const fn lock_flag(yaw_lock: bool) -> f32 {
    if yaw_lock {
        flag(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_LOCK)
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn info(device: u8, cap_flags: u32) -> MavMessage {
        MavMessage::GimbalManagerInformation(GimbalManagerInformation {
            time_boot_ms: 0,
            cap_flags,
            roll_min: 0.0,
            roll_max: 0.0,
            pitch_min: -1.5,
            pitch_max: 0.5,
            yaw_min: -3.0,
            yaw_max: 3.0,
            gimbal_device_id: device,
        })
    }

    fn status(device: u8, flags: u32) -> MavMessage {
        MavMessage::GimbalManagerStatus(GimbalManagerStatus {
            time_boot_ms: 0,
            flags,
            gimbal_device_id: device,
            primary_control_sysid: 0,
            primary_control_compid: 0,
            secondary_control_sysid: 0,
            secondary_control_compid: 0,
        })
    }

    fn long(message: &MavMessage) -> (u16, [f32; 7], u8, u8) {
        let MavMessage::CommandLong(c) = message else {
            panic!("a COMMAND_LONG");
        };
        (
            c.command,
            [
                c.param1, c.param2, c.param3, c.param4, c.param5, c.param6, c.param7,
            ],
            c.target_system,
            c.target_component,
        )
    }

    const ALL: u32 = 0x3_FFFF;

    #[test]
    fn nothing_is_kept_until_discover_and_then_everyones_is() {
        let mut manager = GimbalManager::default();
        manager.observe(&info(1, ALL));
        assert!(manager.manager_info.is_empty());
        let (command, params, system, component) = long(&manager.discover());
        assert_eq!(command, 512);
        assert!((params[0] - 280.0).abs() < f32::EPSILON);
        assert_eq!((system, component), (0, 0));
        manager.observe(&info(3, 1));
        manager.observe(&info(2, 2));
        manager.observe(&info(5, 4));
        // Index 0 is the lowest device heard: 3, then 2, and 5 does not displace it.
        assert_eq!(manager.manager_info[&0].gimbal_device_id, 2);
        assert_eq!(manager.manager_info.len(), 4);
    }

    #[test]
    fn a_command_needs_the_capability_and_reads_index_zero() {
        let target = VehicleId::new(1, 1);
        let mut manager = GimbalManager::default();
        let _ = manager.discover();
        // No information yet: nothing is sent, as the C#'s `Task.FromResult(false)`.
        assert!(manager.set_rates(target, 5.0, 0.0, false, 0).is_none());
        assert!(manager.neutral(target, 0).is_none());
        // A zero rate needs no axis.
        assert!(manager.set_rates(target, 0.0, 0.0, false, 0).is_some());
        manager.observe(&info(
            1,
            GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS.0,
        ));
        let sent = manager.set_rates(target, 5.0, 0.0, true, 0).expect("sent");
        let (command, params, system, component) = long(&sent);
        assert_eq!((command, system, component), (1000, 1, 1));
        assert!(params[0].is_nan() && params[1].is_nan());
        assert!((params[2] - 5.0).abs() < f32::EPSILON);
        assert!((params[4] - 16.0).abs() < f32::EPSILON, "YAW_LOCK");
        // A yaw rate needs the yaw axis and the follow mode.
        assert!(manager.set_rates(target, 0.0, 5.0, false, 0).is_none());
        manager.observe(&info(
            1,
            GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS.0
                | GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW.0,
        ));
        let (_, params, _, _) = long(
            &manager
                .set_rates(target, 0.0, -5.0, false, 0)
                .expect("sent"),
        );
        assert!((params[3] + 5.0).abs() < f32::EPSILON);
        assert!(params[4].abs() < f32::EPSILON);
    }

    #[test]
    fn angles_are_wrapped_and_point_down_needs_local_pointing() {
        let target = VehicleId::new(1, 1);
        let mut manager = GimbalManager::default();
        let _ = manager.discover();
        manager.observe(&info(
            1,
            GimbalManagerCapFlags::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS.0,
        ));
        assert!(manager.set_angles(target, -90.0, 0.0, false, 0).is_none());
        manager.observe(&info(1, ALL));
        let (_, params, _, _) = long(
            &manager
                .set_angles(target, -90.0, 270.0, false, 0)
                .expect("sent"),
        );
        assert!((params[0] + 90.0).abs() < f32::EPSILON);
        assert!((params[1] + 90.0).abs() < f32::EPSILON, "270 wraps to -90");
        assert!(params[2].is_nan() && params[3].is_nan());
        let (_, params, _, _) = long(&manager.retract(target, 0).expect("sent"));
        assert!((params[4] - 1.0).abs() < f32::EPSILON);
        let (_, params, _, _) = long(&manager.neutral(target, 0).expect("sent"));
        assert!((params[4] - 2.0).abs() < f32::EPSILON);
        let Some(MavMessage::CommandInt(roi)) =
            manager.set_roi_location(target, -35.363_261, 149.165_230, 584.0, 0, 0)
        else {
            panic!("a COMMAND_INT");
        };
        assert_eq!(roi.command, 195);
        assert_eq!((roi.x, roi.y, roi.frame), (-353_632_610, 1_491_652_300, 0));
    }

    #[test]
    fn the_attitude_is_turned_by_the_vehicles_yaw_when_in_its_frame() {
        let mut manager = GimbalManager::default();
        let _ = manager.discover();
        assert!(manager.attitude(0, 90.0).is_none());
        let level = Quaternion::from_euler(0.0, -0.5, 0.0);
        #[allow(clippy::cast_possible_truncation)]
        let q = [
            level.q1 as f32,
            level.q2 as f32,
            level.q3 as f32,
            level.q4 as f32,
        ];
        manager.observe(&MavMessage::GimbalDeviceAttitudeStatus(
            GimbalDeviceAttitudeStatus {
                time_boot_ms: 0,
                q,
                angular_velocity_x: 0.0,
                angular_velocity_y: 0.0,
                angular_velocity_z: 0.0,
                failure_flags: 0,
                flags: 0,
                target_system: 0,
                target_component: 0,
                delta_yaw: 0.0,
                delta_yaw_velocity: 0.0,
                gimbal_device_id: 1,
            },
        ));
        // No frame flags and not locked: the vehicle's frame, so its yaw is added.
        let turned = manager.attitude(0, 90.0).expect("an attitude");
        assert!((turned.euler_yaw().to_degrees() - 90.0).abs() < 1e-3);
        assert!((turned.euler_pitch() + 0.5).abs() < 1e-3);
        // In the earth frame it is as reported.
        manager.observe(&status(
            1,
            GimbalDeviceFlags::GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME.0,
        ));
        let reported = manager.attitude(0, 90.0).expect("an attitude");
        assert!(reported.euler_yaw().abs() < 1e-3);
        // `SetAttitudeAsync` unlocked takes the vehicle's yaw off again.
        let target = VehicleId::new(1, 1);
        manager.observe(&info(1, ALL));
        let (_, params, _, _) = long(
            &manager
                .set_attitude(target, turned, false, 0, 90.0)
                .expect("sent"),
        );
        assert!(params[1].abs() < 1e-3);
        assert!((params[0] - (-0.5f32).to_degrees()).abs() < 1e-2);
    }

    #[test]
    fn the_quaternion_is_the_csharps() {
        let q = Quaternion::from_euler(0.1, 0.2, 0.3);
        assert!((q.euler_roll() - 0.1).abs() < 1e-12);
        assert!((q.euler_pitch() - 0.2).abs() < 1e-12);
        assert!((q.euler_yaw() - 0.3).abs() < 1e-12);
        let v = Quaternion::from_euler(0.0, 0.0, std::f64::consts::FRAC_PI_2)
            .body_to_earth(Vector3::new(1.0, 0.0, 0.0));
        assert!(v.x.abs() < 1e-12 && (v.y - 1.0).abs() < 1e-12);
        assert_eq!(
            Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), 0.0),
            Quaternion::default()
        );
        assert!((wrap_180(540.0) - 180.0).abs() < f64::EPSILON);
        assert!((wrap_180(-181.0) - 179.0).abs() < f64::EPSILON);
    }
}
