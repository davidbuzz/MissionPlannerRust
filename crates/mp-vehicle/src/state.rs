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

//! The decoded telemetry model.
//!
//! [`VehicleState::apply`] is the port of `CurrentState.Parent_OnPacketReceived`
//! (`ExtLibs/ArduPilot/CurrentState.cs:2278-4305`), the switch where Mission Planner copies each
//! message into its properties. What became of every public `CurrentState` property - held here,
//! derivable from what is held, not yet ported, or deliberately not carried over - is the table in
//! [`crate::coverage`].
//!
//! # Units
//!
//! Every field holds the value the C# keeps in its backing field: metres, metres per second,
//! volts, and degrees for the angles the C# stores in degrees ([`Attitude`] is the exception, in
//! radians). The C# converts distances, altitudes and speeds to the user's display units *in the
//! property getter* - `alt` returns `(_alt - altoffsethome) * multiplieralt`, `airspeed` returns
//! `_airspeed * multiplierspeed` (`CurrentState.cs:325-349, 494-498`) - so what it stores is SI
//! and the multiplier belongs to the display. The one place the C# multiplies on the way *in*,
//! `WIND`, is noted where it is ported.

use mp_mavlink_dialects::all::{
    BatteryStatus, DistanceSensor, GlobalPositionInt, Gps2Raw, GpsRawInt, HighLatency,
    HighLatency2, MavMessage, ServoOutputRaw, SysStatus, VfrHud,
};
use mp_units::{Bearing, Degrees, LatLon, Metres, MetresPerSecond, Radians};

use crate::clock::DateTime;
use crate::link_quality::{LinkQuality, Radio};
use crate::statics::StreamRates;

/// Attitude in the body frame.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Attitude {
    /// Roll, right wing down positive.
    pub roll: Radians,
    /// Pitch, nose up positive.
    pub pitch: Radians,
    /// Yaw, clockwise from north.
    pub yaw: Radians,
    /// Roll rate.
    pub roll_rate: f32,
    /// Pitch rate.
    pub pitch_rate: f32,
    /// Yaw rate.
    pub yaw_rate: f32,
}

/// What the navigation controller is trying to do, from `NAV_CONTROLLER_OUTPUT`.
///
/// The HUD's target bugs come from here - the green marks on the heading tape and the two
/// scrollers - and so does the cross-track bar. Stored as the wire sends them, in the units the
/// C# keeps: bearings in degrees, distance in metres, errors in metres and metres per second.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3442-3456`
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Nav {
    /// Desired roll, degrees.
    pub roll: f32,
    /// Desired pitch, degrees.
    pub pitch: f32,
    /// Desired heading, degrees.
    pub bearing: f32,
    /// Bearing to the current waypoint, degrees.
    pub target_bearing: f32,
    /// Distance to the current waypoint, metres.
    pub wp_distance: f32,
    /// Altitude error: target minus current, metres.
    pub alt_error: f32,
    /// Airspeed error, metres per second.
    ///
    /// The wire carries this in m/s and the C# divides it by 100 anyway
    /// (`aspd_error = nav.aspd_error / 100.0f`), which makes its airspeed target wrong by that
    /// factor; kept as the wire sends it here, with the divergence noted where the target speed
    /// is derived.
    pub airspeed_error: f32,
    /// Cross-track error, metres. Positive is right of track.
    pub xtrack_error: f32,
}

impl VehicleState {
    /// Rate of turn, degrees per second, as the C# derives it for the HUD.
    ///
    /// Not the gyro's yaw rate: a coordinated-turn estimate from bank angle and ground speed,
    /// `roll * g / groundspeed`, and zero below walking pace where the division would say
    /// something absurd about a stationary aircraft holding a bank.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1203-1210`
    #[must_use]
    #[allow(clippy::cast_possible_truncation)] // display precision
    pub fn turn_rate(&self) -> f32 {
        let ground_speed = self.ground_speed.0;
        if ground_speed <= 1.0 {
            return 0.0;
        }
        (self.attitude.roll.0.to_degrees() * 9.806_65 / ground_speed) as f32
    }

    /// The altitude the controller is flying to, metres, as the HUD's green mark shows it.
    ///
    /// The C# low-pass filters `alt + alt_error` into `targetalt` on every message; the filter
    /// is a display nicety on a value the wire already provides, so this is the unfiltered sum.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1107`
    #[must_use]
    pub fn target_altitude(&self) -> f64 {
        self.altitude_relative.0 + f64::from(self.nav.alt_error)
    }

    /// The airspeed the controller is flying to, metres per second.
    ///
    /// `airspeed + aspd_error`, the same shape as [`Self::target_altitude`]. **Divergence:** the
    /// C# divides the wire's `aspd_error` by 100 before this sum, which turns a 5 m/s error into
    /// 0.05 and pins its target bug to the current speed; the wire's field is in m/s
    /// (`common.xml`), so it is used as sent.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1130, 3454`
    #[must_use]
    pub fn target_airspeed(&self) -> f64 {
        self.air_speed.0 + f64::from(self.nav.airspeed_error)
    }

    /// `ekfstatus`: the single number behind the HUD's EKF indicator.
    ///
    /// The largest of the five variances, forced to 1 when the filter has no attitude, when a GPS
    /// is present and the filter has no horizontal velocity, or when it reports itself
    /// uninitialised. Zero until the first `EKF_STATUS_REPORT`, as the C#'s unset property is.
    /// The C# computes this when the report arrives, with the GPS status of that moment; this
    /// computes it when read, with the GPS status of this one.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2665-2740`
    #[must_use]
    pub fn ekf_status(&self) -> f32 {
        let ekf = &self.ekf;
        if !ekf.seen {
            return 0.0;
        }
        // C#: CurrentState.cs:2665-2667, with Math.Max, which is NaN if either side is.
        let max = |a: f32, b: f32| {
            if a.is_nan() || b.is_nan() {
                f32::NAN
            } else {
                a.max(b)
            }
        };
        let mut status = max(
            ekf.velocity_variance,
            max(
                ekf.compass_variance,
                max(
                    ekf.position_horizontal_variance,
                    max(
                        ekf.position_vertical_variance,
                        ekf.terrain_altitude_variance,
                    ),
                ),
            ),
        );
        // C#: CurrentState.cs:2694-2740, every flag from EKF_ATTITUDE (1) to EKF_UNINITIALIZED
        // (1024).
        for bit in (0..=10).map(|shift| 1_u16 << shift) {
            if ekf.flags & bit == 0 {
                match bit {
                    // EKF_ATTITUDE
                    1 => status = 1.0,
                    // EKF_VELOCITY_HORIZ, with a GPS present
                    2 if self.gps.fix_type > 0 => status = 1.0,
                    _ => {}
                }
            } else if bit == 1024 {
                // EKF_UNINITIALIZED
                status = 1.0;
            }
        }
        status
    }
}

/// GPS receiver status: `GPS_RAW_INT` for [`VehicleState::gps`], `GPS2_RAW` for
/// [`VehicleState::gps2`].
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct GpsInfo {
    /// MAVLink `GPS_FIX_TYPE`: 0 no GPS, 1 no fix, 2 2D, 3 3D, 4 DGPS, 5 RTK float, 6 RTK fixed.
    pub fix_type: u8,
    /// Satellites visible.
    ///
    /// The first receiver keeps its last count when the wire says 255 (unknown), as the C# does;
    /// the second takes whatever arrives, as the C# does for it.
    pub satellites_visible: u8,
    /// Horizontal dilution of precision, `eph / 100`.
    ///
    /// The first receiver keeps its last value when the wire says 65535 (unknown); the second
    /// takes whatever arrives, so an unknown reads 655.35 there as it does in the C#.
    pub hdop: f32,
    /// Vertical dilution of precision, `epv / 100`; NaN when unknown. The first receiver only.
    ///
    /// Not a `CurrentState` property - the C# reads `eph` and ignores `epv`.
    pub vdop: f32,
    /// Course over ground, degrees (`cog / 100`).
    ///
    /// The first receiver updates this only while the ground speed is over 0.5 m/s and the wire
    /// knows the course, so a hovering copter keeps the last real course rather than noise.
    pub course: f32,
    /// Horizontal position accuracy, metres (`h_acc / 1000`).
    pub h_acc: f32,
    /// Vertical position accuracy, metres (`v_acc / 1000`).
    pub v_acc: f32,
    /// Speed accuracy, metres per second (`vel_acc / 1000`).
    pub vel_acc: f32,
    /// Heading accuracy, degrees (`hdg_acc / 1e5`).
    pub hdg_acc: f32,
    /// Yaw from a moving-baseline receiver, degrees (`yaw / 100`).
    pub yaw: f32,
    /// The receiver's own position. The second receiver only: the first one's position goes
    /// into [`VehicleState::position`] when there is no better one, as the C# does.
    pub position: Option<LatLon>,
    /// The receiver's own altitude above mean sea level, metres. The second receiver only, for
    /// the same reason.
    pub altitude_msl: f32,
    /// The receiver's own ground speed, metres per second. The second receiver only: the first
    /// one's goes into [`VehicleState::ground_speed`].
    pub ground_speed: f32,
}

impl GpsInfo {
    /// Whether the fix is good enough to fly on.
    #[must_use]
    pub const fn has_3d_fix(&self) -> bool {
        self.fix_type >= 3
    }

    /// Whether the fix is RTK, which the map should show differently.
    #[must_use]
    pub const fn is_rtk(&self) -> bool {
        self.fix_type >= 5
    }
}

/// One battery's state, from `BATTERY_STATUS` (and `SYS_STATUS` and `BATTERY2` for the first
/// two).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Battery {
    /// Pack voltage in volts: the sum of the reported cell voltages.
    ///
    /// Not `BATTERY_STATUS.voltage`, which tops out at 65.5 V, and not `SYS_STATUS`, which the C#
    /// ignores for this: the cells summed, then low-pass filtered for the first two batteries.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3082-3085, 1295-1303`
    pub voltage: f32,
    /// Current draw in amps. Negative is charging, as on a plane regenerating on descent.
    pub current: f32,
    /// Remaining charge as a percentage. The first battery reads 0 for anything outside 0 to
    /// 100, including the wire's -1 for unknown, as the C#'s setter makes it.
    pub remaining_percent: i8,
    /// Energy consumed in milliamp-hours.
    pub consumed_mah: i32,
    /// Temperature in degrees Celsius. Kept when the wire says it does not know.
    pub temperature: f64,
    /// Estimated time remaining, minutes.
    pub remaining_minutes: f64,
    /// Cell voltages in volts, 0 where the wire has no cell. The first battery only.
    pub cells: [f64; 14],
    /// The voltage filter's state, in the C#'s double precision.
    filtered_voltage: f64,
}

impl Battery {
    /// Sets the pack voltage through the C#'s low-pass filter: the first reading as it is, then
    /// four tenths of each new one. `0.4f` and `0.6f` are single-precision literals in a double
    /// expression, and are kept as such.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1300-1301, 1562-1563`
    fn filter_voltage(&mut self, value: f64) {
        if self.filtered_voltage == 0.0 {
            self.filtered_voltage = value;
        }
        self.filtered_voltage =
            value * f64::from(0.4_f32) + self.filtered_voltage * f64::from(0.6_f32);
        self.voltage = cast_f32(self.filtered_voltage);
    }

    /// The first battery's remaining charge, through the C#'s setter: 0 for anything outside
    /// 0 to 100.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1348-1349`
    const fn set_remaining(&mut self, value: i8) {
        self.remaining_percent = if value < 0 || value > 100 { 0 } else { value };
    }
}

/// What the vehicle knows about the ground beneath it, from `TERRAIN_REPORT`.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3233-3243`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Terrain {
    /// Height above the terrain, metres (`ter_curalt`).
    pub current_height: f32,
    /// Terrain height above mean sea level, metres (`ter_alt`).
    pub terrain_height: f32,
    /// Terrain tiles loaded (`ter_load`).
    pub loaded: u16,
    /// Terrain tiles still pending (`ter_pend`).
    pub pending: u16,
    /// Terrain grid spacing, metres (`ter_space`).
    pub spacing: u16,
}

/// Rangefinders, from `RANGEFINDER` and `DISTANCE_SENSOR`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rangefinder {
    /// The range used as altitude, metres (`sonarrange`).
    ///
    /// From `RANGEFINDER` when the vehicle sends it; otherwise from the first downward-facing
    /// `DISTANCE_SENSOR`, in *whole* metres, because the C# divides the centimetres by 100 in
    /// integer arithmetic. Kept, since that is the number Mission Planner shows.
    pub range: f32,
    /// The rangefinder's voltage, from `RANGEFINDER` (`sonarvoltage`).
    pub voltage: f32,
    /// Each `DISTANCE_SENSOR`'s last distance, by sensor id 0 to 9, centimetres
    /// (`rangefinder1` to `rangefinder10`).
    pub distances: [u16; 10],
    /// Which `DISTANCE_SENSOR` id feeds [`Self::range`]: -1 once a `RANGEFINDER` has been seen,
    /// which then wins for good, and `i32::MAX` for none. The C#'s `_rangefinderalt_index`.
    altitude_sensor: i32,
}

impl Default for Rangefinder {
    fn default() -> Self {
        Self {
            range: 0.0,
            voltage: 0.0,
            distances: [0; 10],
            // C#: ExtLibs/ArduPilot/CurrentState.cs:89
            altitude_sensor: i32::MAX,
        }
    }
}

/// A point as the C#'s `PointLatLngAlt` carries one: latitude and longitude in degrees, altitude
/// in metres, with no range check, and (0, 0, 0) for a point that has not been set.
/// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:19-45`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LatLngAlt {
    /// Latitude, degrees.
    pub lat: f64,
    /// Longitude, degrees.
    pub lng: f64,
    /// Altitude, metres.
    pub alt: f64,
}

impl LatLngAlt {
    /// (0, 0, 0): `new PointLatLngAlt()`, and `PointLatLngAlt.Zero`.
    pub const ZERO: Self = Self {
        lat: 0.0,
        lng: 0.0,
        alt: 0.0,
    };
}

/// Everything decoded about one vehicle.
///
/// `Copy` on purpose: publishing a snapshot is a memcpy into a recycled allocation, not a deep
/// clone with heap traffic.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct VehicleState {
    /// MAVLink system id.
    pub sysid: u8,
    /// MAVLink component id.
    pub compid: u8,
    /// `MAV_TYPE` of the vehicle.
    pub vehicle_type: u8,
    /// `MAV_AUTOPILOT` identifier.
    pub autopilot: u8,
    /// `MAV_MODE_FLAG` bitfield.
    pub base_mode: u8,
    /// Autopilot-specific flight mode number.
    pub custom_mode: u32,
    /// `MAV_STATE` of the vehicle.
    pub system_status: u8,
    /// Whether the safety-armed flag is set.
    pub armed: bool,

    /// Attitude.
    pub attitude: Attitude,
    /// Global position, absent until one is known.
    ///
    /// From `GLOBAL_POSITION_INT` while that is valid, otherwise from `GPS_RAW_INT`, as the C#
    /// does. An invalid `GLOBAL_POSITION_INT` - a zero or `INT32_MAX` coordinate - leaves the last
    /// position in place rather than clearing it. `(0, 0)` is how the C# represents "no
    /// position", and is `None` here.
    pub position: Option<LatLon>,
    /// Altitude above mean sea level.
    pub altitude_msl: Metres,
    /// Altitude above the home point.
    pub altitude_relative: Metres,
    /// Heading from `GLOBAL_POSITION_INT.hdg`.
    ///
    /// Not a `CurrentState` property: the C# ignores `hdg` and shows `yaw` (from `ATTITUDE`) and
    /// `groundcourse` (from `GPS_RAW_INT`) instead - see [`Attitude::yaw`] and
    /// [`GpsInfo::course`].
    pub heading: Bearing,
    /// Ground speed: `VFR_HUD`, or `GPS_RAW_INT`, whichever arrived last.
    pub ground_speed: MetresPerSecond,
    /// Indicated airspeed: `VFR_HUD`, or `AIRSPEED` from the first sensor.
    pub air_speed: MetresPerSecond,
    /// Climb rate, positive up.
    pub climb_rate: MetresPerSecond,
    /// Throttle percentage as reported in `VFR_HUD`, negated while the vehicle says its motors
    /// are reversed (`ch3percent`).
    pub throttle_percent: i16,
    /// Velocity north, metres per second, from `GLOBAL_POSITION_INT` (`vx`).
    pub velocity_north: f64,
    /// Velocity east, metres per second (`vy`).
    pub velocity_east: f64,
    /// Velocity down, metres per second (`vz`). Positive is descending.
    pub velocity_down: f64,
    /// Whether the last `GLOBAL_POSITION_INT` was valid, so `GPS_RAW_INT` should not overwrite
    /// its position. The C#'s private `useLocation`.
    use_location: bool,

    /// The first GPS receiver, from `GPS_RAW_INT`.
    pub gps: GpsInfo,
    /// The second GPS receiver, from `GPS2_RAW`.
    pub gps2: GpsInfo,
    /// The first battery.
    pub battery: Battery,
    /// The second to ninth batteries: `BATTERY_STATUS` ids 1 to 8, so `batteries[0]` is the one
    /// the C# calls `battery_voltage2`.
    pub batteries: [Battery; 8],
    /// Home position, set once the vehicle reports it.
    pub home: Option<LatLon>,
    /// Home altitude above mean sea level (`HomeAlt`).
    pub home_altitude: Metres,
    /// Link health.
    pub link: LinkQuality,
    /// What a SiK radio on the link says about it.
    pub radio: Radio,
    /// What the navigation controller is aiming for.
    pub nav: Nav,
    /// The mission item the vehicle is flying to, from `MISSION_CURRENT`.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3413-3420`
    pub mission_current: u16,
    /// The last non-zero mission item the vehicle was flying to in Auto, which Resume Mission
    /// restarts from. `None` is the C#'s -1.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3422`
    pub last_auto_wp: Option<u16>,
    /// Sensor health, as reported in `SYS_STATUS`.
    pub sensors: crate::sensors::Sensors,
    /// Autopilot main loop load, percent (`SYS_STATUS.load / 10`).
    pub load: f32,
    /// `SYS_STATUS.drop_rate_comm`, as sent (`packetdropremote`).
    pub packet_drop_remote: u16,
    /// The autopilot's four internal error counters, `SYS_STATUS.errors_count1` to `4`.
    pub errors_count: [u16; 4],
    /// Radio control channel values, as last reported.
    pub rc: crate::rc::RcChannels,
    /// Servo output pulse widths, microseconds: channels 1 to 16 from `SERVO_OUTPUT_RAW` port 0,
    /// 17 to 32 from port 1.
    pub servo_outputs: [u16; 32],
    /// The estimator's account of itself.
    pub ekf: crate::health::EkfStatus,
    /// How hard the frame is shaking.
    pub vibration: crate::health::Vibration,
    /// Wind direction, degrees, the direction it blows from, from `WIND`.
    pub wind_direction: f32,
    /// Wind speed, metres per second, from `WIND`.
    pub wind_speed: f32,
    /// The terrain beneath the vehicle.
    pub terrain: Terrain,
    /// Rangefinders.
    pub rangefinder: Rangefinder,
    /// `MAVState.Proximity`: the proximity sensors' readings, from `DISTANCE_SENSOR` and
    /// `OBSTACLE_DISTANCE`, as `Controls/ProximityControl.cs` draws them (see
    /// [`crate::proximity`]). `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:105-106, 326`
    pub proximity: crate::proximity::Proximity,
    /// Absolute pressure from the first barometer, hectopascals (`press_abs`).
    pub press_abs: f32,
    /// The first barometer's temperature, centidegrees Celsius, as the wire sends it
    /// (`press_temp`). `HIGH_LATENCY` writes whole degrees here, as it does in the C#.
    pub press_temp: i32,
    /// Absolute pressure from the second barometer, hectopascals (`press_abs2`).
    pub press_abs2: f32,
    /// The second barometer's temperature, centidegrees Celsius (`press_temp2`).
    pub press_temp2: i32,
    /// The first airspeed sensor's temperature (`airspeed1_temp`).
    ///
    /// In whatever unit the last message used, because the C# does not convert: centidegrees
    /// from `SCALED_PRESSURE`, degrees from `AIRSPEED` and the high-latency messages.
    pub airspeed1_temp: f32,
    /// The second airspeed sensor's temperature, centidegrees Celsius, from `SCALED_PRESSURE2`
    /// (`airspeed2_temp`).
    pub airspeed2_temp: f32,
    /// The airspeed sensor's calibration ratio, from `AIRSPEED_AUTOCAL` (`asratio`).
    pub airspeed_ratio: f32,
    /// Angle of attack, degrees, from `AOA_SSA` (`AOA`).
    pub aoa: f32,
    /// Sideslip angle, degrees, from `AOA_SSA` (`SSA`).
    pub ssa: f32,

    /// The three IMUs (`ax` to `imu3_temp`).
    pub imu: [crate::onboard::Imu; 3],
    /// The two RPM sensors, from `RPM` (`rpm1`, `rpm2`).
    pub rpm: [f32; 2],
    /// The two hygrometers.
    pub hygrometers: [crate::onboard::Hygrometer; 2],
    /// Sixteen ESCs.
    pub escs: [crate::onboard::Esc; 16],
    /// The last PID tuning report.
    pub pid: crate::onboard::PidTuning,
    /// The generator.
    pub generator: crate::onboard::Generator,
    /// The fuel-injected engine.
    pub efi: crate::onboard::Efi,
    /// The ADS-B transponder's own status.
    pub transponder: crate::onboard::Transponder,
    /// The second AHRS.
    pub ahrs2: crate::onboard::Ahrs2,
    /// The flight controller's processor.
    pub mcu: crate::onboard::Mcu,
    /// The board's power, memory and bus errors.
    pub board: crate::onboard::Board,
    /// The geofence's breach status.
    pub fence_breach: crate::onboard::FenceBreach,
    /// Where the camera mount points.
    pub mount: crate::onboard::Mount,
    /// The optical flow sensor.
    pub optical_flow: crate::onboard::OpticalFlow,
    /// What the autopilot says it is.
    pub autopilot_info: crate::onboard::AutopilotInfo,
    /// `EXTENDED_SYS_STATE.vtol_state`, `MAV_VTOL_STATE` (`vtol_state`).
    pub vtol_state: u8,
    /// `EXTENDED_SYS_STATE.landed_state`, `MAV_LANDED_STATE` (`landed_state`).
    pub landed_state: u8,
    /// The vehicle's clock from `SYSTEM_TIME`, milliseconds since 1970 (`gpstime`, which the C#
    /// holds as a `DateTime`). 0 until reported.
    pub gps_time_unix_ms: u64,
    /// The local position from `LOCAL_POSITION_NED`, north, east and down, metres (`posn`,
    /// `pose`, `posd`).
    pub local_position: [f32; 3],
    /// Where the position controller is going, from `POSITION_TARGET_GLOBAL_INT`.
    pub target_position: Option<LatLon>,
    /// The target's altitude above sea level.
    pub target_altitude_msl: Metres,

    /// `datetime`: this vehicle's clock, which the rates and totals below are measured against.
    /// [`crate::VehicleRegistry::apply_at`] sets it to each packet's time before applying it;
    /// [`DateTime::MIN`] until then, which leaves them all still. See [`crate::clock`].
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2009-2010`
    pub datetime: DateTime,
    /// `altoffsethome`, metres: what [`VehicleState::alt`] subtracts. The flight screen's "Home
    /// Alt" button toggles it between 0 and minus the home altitude, which makes the displayed
    /// altitude above sea level (`FlightData.cs:1236-1247`); the vertical speed is worked out
    /// from the offset altitude, as in the C#.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:378-383`
    pub alt_offset_home: f32,
    /// `_alt`, the altitude above home as the `alt` setter was last given it, single precision.
    pub(crate) alt_backing: f32,
    /// `oldalt`: the offset altitude the vertical speed was last worked out from.
    pub(crate) old_alt: f32,
    /// `lastalt`: when.
    pub(crate) last_alt: DateTime,
    /// `gotVFR`: a `VFR_HUD` has given the climb rate, so the `alt` setter no longer does.
    pub(crate) got_vfr: bool,
    /// `_verticalspeed`, behind [`VehicleState::vertical_speed`].
    pub(crate) vertical_speed_backing: f32,
    /// `distTraveled`, metres: the distance flown while armed on a 3D fix, a straight line a
    /// second. See [`VehicleState::update_current_settings`].
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1186-1187, 4606-4619`
    pub dist_traveled: f32,
    /// `timeInAir`, seconds: the seconds armed with the throttle over 12% or the ground speed
    /// over 3 m/s.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1191-1192, 4621-4626`
    pub time_in_air: f32,
    /// `timeSinceArmInAir`, seconds: the same count, restarted at 0 each time the vehicle arms.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1189, 2878-2882, 4621-4626`
    pub time_since_arm_in_air: f32,
    /// `lastpos`: the position at the last second counted; `None` for the C#'s (0, 0).
    pub(crate) last_pos: Option<LatLon>,
    /// `lastsecondcounter`: the clock at the last second counted.
    pub(crate) last_second_counter: DateTime,
    /// `Base`: the moving base's position, which [`VehicleState::dist_from_moving_base`] measures
    /// from; (0, 0, 0) until set. The RTK injection page sets it from the base receiver's reports
    /// (`ConfigSerialInjectGPS.cs:910, 1077, 1098`) and the moving-base control from its own GPS
    /// (`Controls/MovingBase.cs:227`); whoever ports those writes it here.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:79, 1591-1604`
    pub base: LatLngAlt,
    /// The stream rates Mission Planner asks this vehicle for, starting from
    /// [`StreamRates::backups`] when it is first seen.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2002-2007, 4393-4397`
    pub rates: StreamRates,
    /// `GimbalPoint`: where the camera is pointed, projected onto the terrain; `None` until the
    /// flight screen has projected it. The C# projects it on each map update when the mount
    /// parameters say it is stabilised, with `GimbalPoint.ProjectPoint` - terrain heights, the
    /// mount's parameters and angles - and sets it when that finds a point
    /// (`FlightData.cs:3964-3995`); that projection is the flight screen's to port, and writes
    /// here.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2022`
    pub gimbal_point: Option<LatLngAlt>,
    /// `timesincelastshot`, seconds between the last two camera shots; 0 until the flight screen
    /// sets it from [`VehicleState::shot_interval`].
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1993-1994`
    pub time_since_last_shot: f64,
    /// `speedup`: how fast the vehicle's clock runs against ours - a simulator's speed-up - from
    /// `RAW_IMU`, filtered 95% old to 5% new. Only readings that move the IMU clock forward by
    /// less than ten seconds count, and the first is measured from an IMU clock of 0, so it
    /// counts only if the vehicle is first heard within ten seconds of boot, and never again once
    /// the vehicle reboots. **Divergence:** measured against [`VehicleState::datetime`] where the
    /// C# uses `DateTime.Now` - the same on a live link, and the recorded time in a replay, where
    /// the C#'s figure is how fast the file is being read.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2156, 3699-3713`
    pub speedup: f32,
    /// `imutime`: the IMU clock at the last reading counted, seconds.
    pub(crate) imu_time: f64,
    /// `lastimutime`: our clock then.
    pub(crate) last_imu_time: DateTime,
    /// `hilch1` to `hilch8`: `RC_CHANNELS_SCALED`'s eight channels, or `HIL_CONTROLS`' first four
    /// as ten-thousandths - what a hardware-in-the-loop simulator is being told.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:105-113, 2307-2324, 2551-2564`
    pub hil_channels: [i32; 8],
    /// `customfield0` to `customfield19`: `NAMED_VALUE_FLOAT` values, each in the field its name
    /// was given; [`VehicleState::custom_field_name`] says which name that is.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:239-258, 3913-4012`
    pub custom_fields: [f32; crate::statics::CUSTOM_FIELDS],
    /// `lowairspeed`, as each `VFR_HUD` sets it: armed, in the air since arming, with the airspeed
    /// sensor enabled and healthy, and below the minimum airspeed parameter, which the owner of
    /// the parameters gives [`VehicleState::set_airspeed_min_params`].
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:505, 3858-3888`
    pub low_airspeed: bool,
    /// The minimum airspeed parameter as last given by [`VehicleState::set_airspeed_min_params`].
    pub(crate) airspeed_min_param: Option<f32>,
    /// `_cachedAirspeedMin`: the minimum the warning last took from it.
    pub(crate) cached_airspeed_min: f32,
    /// `_lastAirspeedMinCheck`.
    pub(crate) last_airspeed_min_check: DateTime,
    /// `battery_usedmah`, milliamp-hours: `BATTERY_STATUS`'s `current_consumed` for the first
    /// battery, and between reports the `SYS_STATUS` current integrated over the clock. The
    /// C#'s `battery_mahperkm` and `battery_kmleft` are worked out from it.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1396-1415, 1483-1485, 3120`
    pub battery_used_mah: f64,
    /// `_lastcurrent`: the clock at the last current reading.
    pub(crate) last_current: DateTime,

    /// Number of MAVLink messages applied to this state.
    pub messages_applied: u64,

    /// Heartbeats applied to this state - `HEARTBEAT`s and `HIGH_LATENCY2`s, the two
    /// `getHeartBeat` returns on - so a caller can wait for the next one as `getHeartBeat` does:
    /// the firmware page's `doReboot(true, false)` waits for a heartbeat before its reboot.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1153-1203, 2591-2603`
    pub heartbeats: u64,

    /// When the newest frame applied to this state arrived at the link: the start of a
    /// packet-to-pixel measurement (DELIVERABLES.md Deliverable 9). Measurement scaffolding, not vehicle
    /// state and not the C#'s: a link stamps it only when its configuration asks
    /// (`mp_link::LinkConfig::stamp_arrivals`, which only `MP_STORM` sets), and it is `None`
    /// otherwise.
    pub packet_in: Option<std::time::Instant>,
}

/// `MAV_MODE_FLAG_SAFETY_ARMED`. `// C#: ExtLibs/Mavlink/Mavlink.cs:6557`
const MODE_FLAG_SAFETY_ARMED: u8 = 128;
/// `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`. `// C#: ExtLibs/Mavlink/Mavlink.cs:6536`
const MODE_FLAG_CUSTOM_MODE_ENABLED: u8 = 1;
/// `MAV_SENSOR_ROTATION_PITCH_270`: facing down. `// C#: ExtLibs/Mavlink/Mavlink.cs:3985`
const SENSOR_ROTATION_PITCH_270: u8 = 25;

/// `(0, 0)` is how the C# holds "no position"; anything else that is a real place is one.
fn position_or_none(latitude: f64, longitude: f64) -> Option<LatLon> {
    if latitude == 0.0 && longitude == 0.0 {
        return None;
    }
    LatLon::new(latitude, longitude).ok()
}

/// Degrees, as the C# holds an angle, into the radians [`Attitude`] holds.
fn radians(degrees: f32) -> Radians {
    Degrees(f64::from(degrees)).to_radians()
}

impl VehicleState {
    /// Creates empty state for a vehicle, with the saved stream rates as the C#'s constructor
    /// takes them through `ResetInternals`.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:223-227, 4393-4397`
    #[must_use]
    pub fn new(sysid: u8, compid: u8) -> Self {
        Self {
            sysid,
            compid,
            rates: StreamRates::backups(),
            ..Self::default()
        }
    }

    /// Applies a decoded message.
    ///
    /// The messages `CurrentState.Parent_OnPacketReceived` reads, and `HOME_POSITION`, which
    /// `MAVLinkInterface` reads for it; the vehicle's own subsystems are in
    /// [`crate::onboard`]. The rest are the business of the subsystems that care (parameters,
    /// missions, logs). Returns whether the message changed anything, which lets the publisher
    /// skip needless snapshots.
    #[allow(clippy::too_many_lines)] // a flat dispatch over message types is clearer than nesting
    pub fn apply(&mut self, message: &MavMessage) -> bool {
        self.messages_applied += 1;
        match message {
            MavMessage::Heartbeat(m) => {
                self.heartbeats += 1;
                self.vehicle_type = m.r#type;
                self.autopilot = m.autopilot;
                self.base_mode = m.base_mode;
                self.custom_mode = m.custom_mode;
                self.system_status = m.system_status;
                let armed = m.base_mode & MODE_FLAG_SAFETY_ARMED != 0;
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2878-2882, arming restarts the count.
                if !self.armed && armed {
                    self.time_since_arm_in_air = 0.0;
                }
                self.armed = armed;
                true
            }
            MavMessage::Attitude(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3251-3253, which converts to degrees.
                self.attitude = Attitude {
                    roll: Radians(f64::from(m.roll)),
                    pitch: Radians(f64::from(m.pitch)),
                    yaw: Radians(f64::from(m.yaw)),
                    roll_rate: m.rollspeed,
                    pitch_rate: m.pitchspeed,
                    yaw_rate: m.yawspeed,
                };
                true
            }
            MavMessage::GlobalPositionInt(m) => {
                self.apply_global_position_int(m);
                true
            }
            MavMessage::GpsRawInt(m) => {
                self.apply_gps_raw_int(m);
                true
            }
            MavMessage::Gps2Raw(m) => {
                self.apply_gps2_raw(m);
                true
            }
            MavMessage::GpsStatus(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3381
                self.gps.satellites_visible = m.satellites_visible;
                true
            }
            MavMessage::EkfStatusReport(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2657-2663
                self.ekf = crate::health::EkfStatus {
                    velocity_variance: m.velocity_variance,
                    position_horizontal_variance: m.pos_horiz_variance,
                    position_vertical_variance: m.pos_vert_variance,
                    compass_variance: m.compass_variance,
                    terrain_altitude_variance: m.terrain_alt_variance,
                    flags: m.flags,
                    seen: true,
                };
                true
            }
            MavMessage::Vibration(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2598-2603
                self.vibration = crate::health::Vibration {
                    x: m.vibration_x,
                    y: m.vibration_y,
                    z: m.vibration_z,
                    clipping: [m.clipping_0, m.clipping_1, m.clipping_2],
                    seen: true,
                };
                true
            }
            MavMessage::VfrHud(m) => {
                self.apply_vfr_hud(m);
                true
            }
            MavMessage::NavControllerOutput(m) => {
                self.nav = Nav {
                    roll: m.nav_roll,
                    pitch: m.nav_pitch,
                    bearing: f32::from(m.nav_bearing),
                    target_bearing: f32::from(m.target_bearing),
                    wp_distance: f32::from(m.wp_dist),
                    alt_error: m.alt_error,
                    airspeed_error: m.aspd_error,
                    xtrack_error: m.xtrack_error,
                };
                true
            }
            MavMessage::MissionCurrent(m) => {
                self.mission_current = m.seq;
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3422, `mode.ToLower() == "auto"`.
                let auto = crate::modes::flight_mode_name(self.vehicle_type, self.custom_mode)
                    .is_some_and(|mode| mode.eq_ignore_ascii_case("auto"));
                if auto && m.seq != 0 {
                    self.last_auto_wp = Some(m.seq);
                }
                true
            }
            // All sixteen channels ArduPilot maps to functions. RC_CHANNELS carries eighteen;
            // the last two are beyond what the firmware reads, and offering limits on channels
            // nothing looks at would be a calibration screen inviting a pointless setting.
            // C#: ExtLibs/ArduPilot/CurrentState.cs:3500-3516
            MavMessage::RcChannels(m) => {
                self.rc.values = [
                    m.chan1_raw,
                    m.chan2_raw,
                    m.chan3_raw,
                    m.chan4_raw,
                    m.chan5_raw,
                    m.chan6_raw,
                    m.chan7_raw,
                    m.chan8_raw,
                    m.chan9_raw,
                    m.chan10_raw,
                    m.chan11_raw,
                    m.chan12_raw,
                    m.chan13_raw,
                    m.chan14_raw,
                    m.chan15_raw,
                    m.chan16_raw,
                ];
                self.rc.count = m.chancount;
                self.rc.rssi = m.rssi;
                self.rc.reported = true;
                true
            }
            // The older eight-channel message, still sent by some links. Taken only when the
            // newer one has not been seen, so a receiver reporting both does not have its
            // upper channels blanked by the shorter message arriving second.
            MavMessage::RcChannelsRaw(m) if !self.rc.reported => {
                self.rc.values = [crate::rc::UNAVAILABLE; crate::rc::CHANNELS];
                let raw = [
                    m.chan1_raw,
                    m.chan2_raw,
                    m.chan3_raw,
                    m.chan4_raw,
                    m.chan5_raw,
                    m.chan6_raw,
                    m.chan7_raw,
                    m.chan8_raw,
                ];
                for (slot, value) in self.rc.values.iter_mut().zip(raw) {
                    *slot = value;
                }
                self.rc.count = 8;
                self.rc.rssi = m.rssi;
                true
            }
            MavMessage::ServoOutputRaw(m) => self.apply_servo_output_raw(m),
            MavMessage::SysStatus(m) => {
                self.apply_sys_status(m);
                true
            }
            MavMessage::BatteryStatus(m) => self.apply_battery_status(m),
            MavMessage::Battery2(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3073, through the setter at 1418-1430,
                // which ignores a negative current.
                match self.batteries.first_mut() {
                    Some(battery) if m.current_battery >= 0 => {
                        battery.current = f32::from(m.current_battery) / 100.0;
                        true
                    }
                    _ => false,
                }
            }
            MavMessage::HomePosition(m) => {
                // Handled by MAVLinkInterface rather than CurrentState in the C#.
                // C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5701-5707
                self.home = position_or_none(
                    f64::from(m.latitude) / 1.0e7,
                    f64::from(m.longitude) / 1.0e7,
                );
                self.home_altitude = Metres(f64::from(m.altitude) / 1000.0);
                true
            }
            MavMessage::Radio(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3389-3395
                self.radio = Radio {
                    rssi: m.rssi,
                    remrssi: m.remrssi,
                    txbuf: m.txbuf,
                    rxerrors: m.rxerrors,
                    noise: m.noise,
                    remnoise: m.remnoise,
                    fixed: m.fixed,
                };
                true
            }
            MavMessage::RadioStatus(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3403-3409
                self.radio = Radio {
                    rssi: m.rssi,
                    remrssi: m.remrssi,
                    txbuf: m.txbuf,
                    rxerrors: m.rxerrors,
                    noise: m.noise,
                    remnoise: m.remnoise,
                    fixed: m.fixed,
                };
                true
            }
            MavMessage::Wind(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2857-2858. The C# multiplies the speed
                // by `multiplierspeed` here, on the way in - the one speed it stores in display
                // units. Stored in m/s here like every other speed.
                self.wind_direction = (m.direction + 360.0) % 360.0;
                self.wind_speed = m.speed;
                true
            }
            MavMessage::TerrainReport(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3237-3241
                self.terrain = Terrain {
                    current_height: m.current_height,
                    terrain_height: m.terrain_height,
                    loaded: m.loaded,
                    pending: m.pending,
                    spacing: m.spacing,
                };
                true
            }
            MavMessage::Rangefinder(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2749-2754. Once seen, it is preferred
                // to DISTANCE_SENSOR for good.
                self.rangefinder.range = m.distance;
                self.rangefinder.voltage = m.voltage;
                self.rangefinder.altitude_sensor = -1;
                true
            }
            MavMessage::DistanceSensor(m) => {
                self.apply_distance_sensor(m);
                // `MAVState.Proximity`'s subscription to the same message.
                // C#: ExtLibs/ArduPilot/Proximity.cs:33-34, 54-67
                self.proximity.distance_sensor(m, self.datetime);
                true
            }
            MavMessage::ObstacleDistance(m) => {
                // C#: ExtLibs/ArduPilot/Proximity.cs:36-37, 68-103, with `cs.yaw` - degrees, 0
                // to 360 (CurrentState.cs:274-284) - for a north-aligned one.
                #[allow(clippy::cast_possible_truncation)] // `(float)`, as the C# stores it
                let mut yaw = self.attitude.yaw.to_degrees().0 as f32;
                if yaw < 0.0 {
                    yaw += 360.0;
                }
                self.proximity.obstacle_distance(m, yaw, self.datetime);
                true
            }
            MavMessage::Airspeed(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:4209-4213, the first sensor only.
                if m.id != 0 {
                    return false;
                }
                self.air_speed = MetresPerSecond(f64::from(m.airspeed));
                self.airspeed1_temp = cast_f32(f64::from(m.temperature) / 100.0);
                true
            }
            MavMessage::ScaledPressure(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3215-3218
                self.press_abs = m.press_abs;
                self.press_temp = i32::from(m.temperature);
                self.airspeed1_temp = f32::from(m.temperature_press_diff);
                true
            }
            MavMessage::ScaledPressure2(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3226-3229
                self.press_abs2 = m.press_abs;
                self.press_temp2 = i32::from(m.temperature);
                self.airspeed2_temp = f32::from(m.temperature_press_diff);
                true
            }
            MavMessage::HighLatency(m) => {
                self.apply_high_latency(m);
                true
            }
            MavMessage::HighLatency2(m) => {
                self.heartbeats += 1;
                self.apply_high_latency2(m);
                true
            }
            other => self.apply_onboard(other).unwrap_or(false),
        }
    }

    /// `SYS_STATUS`. `// C#: ExtLibs/ArduPilot/CurrentState.cs:2944-3057`
    ///
    /// The C# does not take the battery voltage from here - `battery_voltage` comes only from
    /// `BATTERY_STATUS` - and neither does this. The half that raises `messageHigh` for an
    /// unhealthy sensor belongs to the display, which derives it from [`Self::sensors`].
    fn apply_sys_status(&mut self, m: &SysStatus) {
        // C#: CurrentState.cs:2949
        self.load = f32::from(m.load) / 10.0;
        // C#: CurrentState.cs:2951, through the setter at 1343-1352
        self.battery.set_remaining(m.battery_remaining);
        // C#: CurrentState.cs:2952, `current_battery / 100.0f` through the setter at 1396-1415:
        // the wire's -1 ("no sensor") is -0.01 A after the division, which the setter turns into
        // 0; anything else is integrated into `battery_usedmah` over the clock.
        self.set_current(f64::from(f32::from(m.current_battery) / 100.0));
        // C#: CurrentState.cs:2954
        self.packet_drop_remote = m.drop_rate_comm;
        // C#: CurrentState.cs:2957-2960
        self.errors_count = [
            m.errors_count1,
            m.errors_count2,
            m.errors_count3,
            m.errors_count4,
        ];
        // The sensor masks are why an aircraft refuses to arm more often than anything else.
        // C#: CurrentState.cs:2962-2964
        self.sensors = crate::sensors::Sensors {
            present: m.onboard_control_sensors_present,
            enabled: m.onboard_control_sensors_enabled,
            health: m.onboard_control_sensors_health,
            reported: true,
        };
    }

    /// `GLOBAL_POSITION_INT`. `// C#: ExtLibs/ArduPilot/CurrentState.cs:3261-3288`
    fn apply_global_position_int(&mut self, m: &GlobalPositionInt) {
        // C#: CurrentState.cs:3268, whatever the position says: `relative_alt / 1000.0f`, in
        // single precision through the `alt` setter, which works out the vertical speed.
        self.altitude_relative = Metres::from_millimetres(m.relative_alt);
        self.set_alt(i32_f32(m.relative_alt) / 1000.0);
        // C#: CurrentState.cs:3270-3274. "The new AHRS dead reckoning may send 0 alt and 0
        // long": a zero or unset coordinate leaves the last position where it was, and lets
        // GPS_RAW_INT supply one instead.
        self.use_location = !(m.lat == 0 || m.lon == 0 || m.lat == i32::MAX || m.lon == i32::MAX);
        if self.use_location {
            // C#: CurrentState.cs:3277-3284
            self.position = LatLon::from_mavlink_e7(m.lat, m.lon).ok();
            self.altitude_msl = Metres::from_millimetres(m.alt);
            self.velocity_north = f64::from(m.vx) * 0.01;
            self.velocity_east = f64::from(m.vy) * 0.01;
            self.velocity_down = f64::from(m.vz) * 0.01;
        }
        // hdg is centidegrees, 65535 when unknown. Not read by the C#; see `heading`.
        if m.hdg != u16::MAX {
            self.heading = Bearing(Degrees(f64::from(m.hdg) / 100.0));
        }
    }

    /// `GPS_RAW_INT`. `// C#: ExtLibs/ArduPilot/CurrentState.cs:3289-3340`
    ///
    /// The four accuracies and the yaw are MAVLink 2 extensions, which the C# sets to -1 for a
    /// MAVLink 1 frame. The decoded message does not say which version carried it, and a
    /// MAVLink 1 frame decodes them as 0, so that is what they read here.
    fn apply_gps_raw_int(&mut self, m: &GpsRawInt) {
        // C#: CurrentState.cs:3294-3302, only when GLOBAL_POSITION_INT has no position. Each
        // coordinate is taken unless it is INT32_MAX, and `* 1.0e-7` is the C#'s arithmetic here
        // where GLOBAL_POSITION_INT divides by 1e7.
        if !self.use_location {
            let (latitude, longitude) = self
                .position
                .map_or((0.0, 0.0), |p| (p.latitude(), p.longitude()));
            let latitude = if m.lat == i32::MAX {
                latitude
            } else {
                f64::from(m.lat) * 1.0e-7
            };
            let longitude = if m.lon == i32::MAX {
                longitude
            } else {
                f64::from(m.lon) * 1.0e-7
            };
            self.position = position_or_none(latitude, longitude);
            self.altitude_msl = Metres::from_millimetres(m.alt);
        }
        // C#: CurrentState.cs:3305
        self.gps.fix_type = m.fix_type;
        // C#: CurrentState.cs:3308-3309. `Math.Round(eph / 100.0, 2)` is `eph / 100.0` for an
        // integer `eph`: scaled back by 100 it is already whole, so the rounding changes nothing.
        if m.eph != u16::MAX {
            self.gps.hdop = cast_f32(f64::from(m.eph) / 100.0);
        }
        self.gps.vdop = if m.epv == u16::MAX {
            f32::NAN
        } else {
            f32::from(m.epv) / 100.0
        };
        // C#: CurrentState.cs:3311-3312
        if m.satellites_visible != u8::MAX {
            self.gps.satellites_visible = m.satellites_visible;
        }
        // C#: CurrentState.cs:3314-3315
        if m.vel != u16::MAX {
            self.ground_speed = MetresPerSecond(f64::from(f32::from(m.vel) * 1.0e-2));
        }
        // C#: CurrentState.cs:3317-3318. The C# compares its display-unit ground speed with 0.5;
        // this compares metres per second, which is the same thing in the default units.
        if self.ground_speed.0 > 0.5 && m.cog != u16::MAX {
            self.gps.course = f32::from(m.cog) * 1.0e-2;
        }
        // C#: CurrentState.cs:3322-3326
        self.gps.h_acc = u32_f32(m.h_acc) / 1000.0;
        self.gps.v_acc = u32_f32(m.v_acc) / 1000.0;
        self.gps.vel_acc = u32_f32(m.vel_acc) / 1000.0;
        self.gps.hdg_acc = u32_f32(m.hdg_acc) / 1e5;
        self.gps.yaw = f32::from(m.yaw) / 100.0;
    }

    /// `GPS2_RAW`, every field taken as it arrives.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3341-3376`
    fn apply_gps2_raw(&mut self, m: &Gps2Raw) {
        // C#: CurrentState.cs:3346-3356
        self.gps2.position = position_or_none(f64::from(m.lat) * 1.0e-7, f64::from(m.lon) * 1.0e-7);
        self.gps2.altitude_msl = i32_f32(m.alt) / 1000.0;
        self.gps2.fix_type = m.fix_type;
        self.gps2.hdop = cast_f32(f64::from(m.eph) / 100.0);
        self.gps2.satellites_visible = m.satellites_visible;
        self.gps2.ground_speed = f32::from(m.vel) * 1.0e-2;
        self.gps2.course = f32::from(m.cog) * 1.0e-2;
        // C#: CurrentState.cs:3360-3364, MAVLink 2 extensions; see `apply_gps_raw_int`.
        self.gps2.h_acc = u32_f32(m.h_acc) / 1000.0;
        self.gps2.v_acc = u32_f32(m.v_acc) / 1000.0;
        self.gps2.vel_acc = u32_f32(m.vel_acc) / 1000.0;
        self.gps2.hdg_acc = u32_f32(m.hdg_acc) / 1e5;
        self.gps2.yaw = f32::from(m.yaw) / 100.0;
    }

    /// `VFR_HUD`. `// C#: ExtLibs/ArduPilot/CurrentState.cs:3841-3889`
    fn apply_vfr_hud(&mut self, m: &VfrHud) {
        // C#: CurrentState.cs:3846-3848
        self.ground_speed = MetresPerSecond(f64::from(m.groundspeed));
        self.air_speed = MetresPerSecond(f64::from(m.airspeed));
        #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)] // a percentage
        let throttle = m.throttle as i16;
        self.throttle_percent = throttle;
        // C#: CurrentState.cs:3850-3852, a positive throttle is negated while the motors are
        // reversed.
        if self.sensors.reverse_motor() && self.throttle_percent > 0 {
            self.throttle_percent = -self.throttle_percent;
        }
        // C#: CurrentState.cs:3855-3856, and from now on the `alt` setter leaves it alone.
        self.climb_rate = MetresPerSecond(f64::from(m.climb));
        self.got_vfr = true;
        // C#: CurrentState.cs:3858-3888
        self.check_low_airspeed(m.airspeed);
    }

    /// `BATTERY_STATUS`. `// C#: ExtLibs/ArduPilot/CurrentState.cs:3077-3208`
    fn apply_battery_status(&mut self, m: &BatteryStatus) -> bool {
        // C#: CurrentState.cs:3085, the cells summed, skipping UINT16_MAX ("no cell").
        let volts = |raw: &u16| {
            if *raw == u16::MAX {
                0.0
            } else {
                f64::from(*raw) / 1000.0
            }
        };
        let total = m.voltages.iter().map(volts).sum::<f64>()
            + m.voltages_ext.iter().map(volts).sum::<f64>();
        let current = f32::from(m.current_battery) / 100.0;
        let temperature = (m.temperature != i16::MAX).then(|| f64::from(m.temperature) / 100.0);
        // `time_remaining / 60.0f` is single precision, stored in a double.
        let remaining_minutes = f64::from(i32_f32(m.time_remaining) / 60.0);

        if m.id == 0 {
            // C#: CurrentState.cs:3089-3118. The cells only when the first one is reported.
            let [v1, v2, v3, v4, v5, v6, v7, v8, v9, v10] = m.voltages;
            let [v11, v12, v13, v14] = m.voltages_ext;
            if v1 != u16::MAX {
                self.battery.cells = [
                    f64::from(v1) / 1000.0,
                    volts(&v2),
                    volts(&v3),
                    volts(&v4),
                    volts(&v5),
                    volts(&v6),
                    volts(&v7),
                    volts(&v8),
                    volts(&v9),
                    volts(&v10),
                    volts(&v11),
                    volts(&v12),
                    volts(&v13),
                    volts(&v14),
                ];
            }
            // C#: CurrentState.cs:3120-3126. The current is written past its setter, so neither
            // it nor its clock is touched by the used capacity, which is simply replaced.
            self.battery_used_mah = f64::from(m.current_consumed);
            let battery = &mut self.battery;
            battery.consumed_mah = m.current_consumed;
            battery.set_remaining(m.battery_remaining);
            battery.filter_voltage(total);
            battery.current = current;
            if let Some(temperature) = temperature {
                battery.temperature = temperature;
            }
            battery.remaining_minutes = remaining_minutes;
            return true;
        }
        // C#: CurrentState.cs:3128-3207, ids 1 to 8. Only the second battery's voltage has a
        // filtering setter; the rest, and every remaining percentage past the first, are plain.
        let Some(battery) = self.batteries.get_mut(usize::from(m.id) - 1) else {
            return false;
        };
        battery.consumed_mah = m.current_consumed;
        battery.remaining_percent = m.battery_remaining;
        if m.id == 1 {
            battery.filter_voltage(total);
        } else {
            battery.voltage = cast_f32(total);
        }
        battery.current = current;
        if let Some(temperature) = temperature {
            battery.temperature = temperature;
        }
        battery.remaining_minutes = remaining_minutes;
        true
    }

    /// `DISTANCE_SENSOR`. `// C#: ExtLibs/ArduPilot/CurrentState.cs:2758-2792`
    fn apply_distance_sensor(&mut self, m: &DistanceSensor) {
        // C#: CurrentState.cs:2762-2771, ids 0 to 9.
        if let Some(slot) = self.rangefinder.distances.get_mut(usize::from(m.id)) {
            *slot = m.current_distance;
        }
        // C#: CurrentState.cs:2774-2790. The first downward-facing sensor is the altitude one,
        // until it turns away; a RANGEFINDER message (-1) outranks every id.
        let downward = m.orientation == SENSOR_ROTATION_PITCH_270;
        let id = i32::from(m.id);
        if downward && id < self.rangefinder.altitude_sensor {
            self.rangefinder.altitude_sensor = id;
        }
        if id == self.rangefinder.altitude_sensor {
            if downward {
                // `current_distance / 100` is integer division in the C#: whole metres.
                self.rangefinder.range = f32::from(m.current_distance / 100);
            } else {
                self.rangefinder.altitude_sensor = i32::MAX;
            }
        }
    }

    /// `SERVO_OUTPUT_RAW`. `// C#: ExtLibs/ArduPilot/CurrentState.cs:3634-3678`
    fn apply_servo_output_raw(&mut self, m: &ServoOutputRaw) -> bool {
        let first = match m.port {
            0 => 0,
            1 => 16,
            _ => return false,
        };
        let values = [
            m.servo1_raw,
            m.servo2_raw,
            m.servo3_raw,
            m.servo4_raw,
            m.servo5_raw,
            m.servo6_raw,
            m.servo7_raw,
            m.servo8_raw,
            m.servo9_raw,
            m.servo10_raw,
            m.servo11_raw,
            m.servo12_raw,
            m.servo13_raw,
            m.servo14_raw,
            m.servo15_raw,
            m.servo16_raw,
        ];
        for (slot, value) in self.servo_outputs.iter_mut().skip(first).zip(values) {
            *slot = value;
        }
        true
    }

    /// `HIGH_LATENCY`, a satellite link's summary of everything.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2421-2472`
    ///
    /// Not held: `landed` and `failsafe` (derived from `HEARTBEAT` here), and `targetairspeed`,
    /// which the C# sets directly and which is derived from the airspeed error here.
    fn apply_high_latency(&mut self, m: &HighLatency) {
        // C#: CurrentState.cs:2427-2448
        if m.base_mode & MODE_FLAG_CUSTOM_MODE_ENABLED != 0 {
            self.custom_mode = m.custom_mode;
        }
        // C#: CurrentState.cs:2450-2452, centidegrees
        self.attitude.roll = radians(f32::from(m.roll) / 100.0);
        self.attitude.pitch = radians(f32::from(m.pitch) / 100.0);
        self.attitude.yaw = radians(f32::from(m.heading) / 100.0);
        // C#: CurrentState.cs:2453
        self.throttle_percent = i16::from(m.throttle);
        // C#: CurrentState.cs:2454-2455
        self.position = position_or_none(f64::from(m.latitude) / 1e7, f64::from(m.longitude) / 1e7);
        self.set_high_latency_altitudes(f32::from(m.altitude_amsl), f32::from(m.altitude_sp));
        // C#: CurrentState.cs:2459-2465
        self.air_speed = MetresPerSecond(f64::from(m.airspeed));
        self.ground_speed = MetresPerSecond(f64::from(m.groundspeed));
        self.climb_rate = MetresPerSecond(f64::from(m.climb_rate));
        self.gps.satellites_visible = m.gps_nsat;
        self.gps.fix_type = m.gps_fix_type;
        #[allow(clippy::cast_possible_wrap)] // 0 to 100, and the setter zeroes anything else
        self.battery.set_remaining(m.battery_remaining as i8);
        // C#: CurrentState.cs:2466-2467, whole degrees into fields that otherwise hold other
        // units; see their documentation.
        self.press_temp = i32::from(m.temperature);
        self.airspeed1_temp = f32::from(m.temperature_air);
        // C#: CurrentState.cs:2469-2470
        self.mission_current = u16::from(m.wp_num);
        self.nav.wp_distance = f32::from(m.wp_distance);
    }

    /// `HIGH_LATENCY2`. `// C#: ExtLibs/ArduPilot/CurrentState.cs:2474-2549`
    ///
    /// Not held: `targetairspeed` and `targetalt`, which the C# sets directly and which are
    /// derived from the errors here, and the failure-flag text, which goes to `messageHigh`.
    fn apply_high_latency2(&mut self, m: &HighLatency2) {
        // C#: CurrentState.cs:2478-2498, with no custom-mode flag to check in this message.
        self.custom_mode = u32::from(m.custom_mode);
        // C#: CurrentState.cs:2500-2501
        self.position = position_or_none(f64::from(m.latitude) / 1e7, f64::from(m.longitude) / 1e7);
        self.set_high_latency_altitudes(f32::from(m.altitude), f32::from(m.target_altitude));
        // C#: CurrentState.cs:2507-2508, the distance in decametres.
        self.nav.wp_distance = f32::from(m.target_distance) * 10.0;
        self.mission_current = m.wp_num;
        // C#: CurrentState.cs:2534-2546, headings in two-degree steps and speeds in fifths of a
        // metre per second.
        self.attitude.yaw = radians(f32::from(m.heading) * 2.0);
        self.nav.target_bearing = f32::from(m.target_heading) * 2.0;
        self.throttle_percent = i16::from(m.throttle);
        self.air_speed = MetresPerSecond(f64::from(f32::from(m.airspeed) / 5.0));
        self.ground_speed = MetresPerSecond(f64::from(f32::from(m.groundspeed) / 5.0));
        self.wind_speed = f32::from(m.windspeed) / 5.0;
        self.wind_direction = f32::from(m.wind_heading) * 2.0;
        self.gps.hdop = f32::from(m.eph);
        self.airspeed1_temp = f32::from(m.temperature_air);
        self.climb_rate = MetresPerSecond(f64::from(i16::from(m.climb_rate) * 10));
        self.battery.set_remaining(m.battery);
    }

    /// The altitudes both high-latency messages set: above sea level, above home (sea level
    /// minus the home altitude, in single precision), and the error to the target, which is the
    /// setpoint minus the altitude above home. `HIGH_LATENCY`'s setpoint is relative to home, so
    /// that is a true error; `HIGH_LATENCY2`'s is above sea level, so the C#'s error is off by
    /// the home altitude there. Kept, since it is the number Mission Planner shows.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2456-2458, 2503-2505`
    fn set_high_latency_altitudes(&mut self, amsl: f32, target: f32) {
        self.altitude_msl = Metres(f64::from(amsl));
        let relative = amsl - cast_f32(self.home_altitude.0);
        self.altitude_relative = Metres(f64::from(relative));
        // Through the `alt` setter, as `alt = altasl - (float)HomeAlt` is.
        self.set_alt(relative);
        self.nav.alt_error = target - relative;
    }
}

/// A double narrowed to the single precision a C# `float` property holds.
#[allow(clippy::cast_possible_truncation)]
const fn cast_f32(value: f64) -> f32 {
    value as f32
}

/// A `uint` in a C# `float` expression: the nearest single-precision value.
#[allow(clippy::cast_precision_loss)]
const fn u32_f32(value: u32) -> f32 {
    value as f32
}

/// An `int` in a C# `float` expression: the nearest single-precision value.
#[allow(clippy::cast_precision_loss)]
const fn i32_f32(value: i32) -> f32 {
    value as f32
}
