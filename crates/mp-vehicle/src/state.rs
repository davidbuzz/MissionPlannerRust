//! The decoded telemetry model.

use mp_mavlink_dialects::all::MavMessage;
use mp_units::{Bearing, Degrees, LatLon, Metres, MetresPerSecond, Radians};

use crate::link_quality::LinkQuality;

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
}

/// GPS receiver status.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct GpsInfo {
    /// MAVLink `GPS_FIX_TYPE`: 0 no GPS, 1 no fix, 2 2D, 3 3D, 4 DGPS, 5 RTK float, 6 RTK fixed.
    pub fix_type: u8,
    /// Satellites visible, 255 when unknown.
    pub satellites_visible: u8,
    /// Horizontal dilution of precision.
    pub hdop: f32,
    /// Vertical dilution of precision.
    pub vdop: f32,
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

/// Power state.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Battery {
    /// Pack voltage in volts.
    pub voltage: f32,
    /// Current draw in amps; negative means unknown.
    pub current: f32,
    /// Remaining charge as a percentage; negative means unknown.
    pub remaining_percent: i8,
    /// Energy consumed in milliamp-hours.
    pub consumed_mah: i32,
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
    /// Global position, absent until a fix arrives.
    pub position: Option<LatLon>,
    /// Altitude above mean sea level.
    pub altitude_msl: Metres,
    /// Altitude above the home point.
    pub altitude_relative: Metres,
    /// Ground track.
    pub heading: Bearing,
    /// Ground speed.
    pub ground_speed: MetresPerSecond,
    /// Indicated airspeed.
    pub air_speed: MetresPerSecond,
    /// Climb rate, positive up.
    pub climb_rate: MetresPerSecond,
    /// Throttle percentage as reported in `VFR_HUD`.
    pub throttle_percent: i16,

    /// GPS status.
    pub gps: GpsInfo,
    /// Power state.
    pub battery: Battery,
    /// Home position, set once the vehicle reports it.
    pub home: Option<LatLon>,
    /// Link health.
    pub link: LinkQuality,
    /// What the navigation controller is aiming for.
    pub nav: Nav,
    /// The mission item the vehicle is flying to, from `MISSION_CURRENT`.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3413-3420`
    pub mission_current: u16,
    /// Sensor health, as reported in `SYS_STATUS`.
    pub sensors: crate::sensors::Sensors,
    /// Radio control channel values, as last reported.
    pub rc: crate::rc::RcChannels,
    /// The estimator's account of itself.
    pub ekf: crate::health::EkfStatus,
    /// How hard the frame is shaking.
    pub vibration: crate::health::Vibration,

    /// Number of MAVLink messages applied to this state.
    pub messages_applied: u64,
}

/// `MAV_MODE_FLAG_SAFETY_ARMED`.
const MODE_FLAG_SAFETY_ARMED: u8 = 128;

impl VehicleState {
    /// Creates empty state for a vehicle.
    #[must_use]
    pub fn new(sysid: u8, compid: u8) -> Self {
        Self {
            sysid,
            compid,
            ..Self::default()
        }
    }

    /// Applies a decoded message.
    ///
    /// Only the messages that drive the flight display are handled here; the rest are the
    /// business of the subsystems that care (parameters, missions, logs). Returns whether the
    /// message changed anything, which lets the publisher skip needless snapshots.
    #[allow(clippy::too_many_lines)] // a flat dispatch over message types is clearer than nesting
    pub fn apply(&mut self, message: &MavMessage) -> bool {
        self.messages_applied += 1;
        match message {
            MavMessage::Heartbeat(m) => {
                self.vehicle_type = m.r#type;
                self.autopilot = m.autopilot;
                self.base_mode = m.base_mode;
                self.custom_mode = m.custom_mode;
                self.system_status = m.system_status;
                self.armed = m.base_mode & MODE_FLAG_SAFETY_ARMED != 0;
                true
            }
            MavMessage::Attitude(m) => {
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
                // (0, 0) is how an unset position is transmitted; treat it as "no fix" rather
                // than flying the map to the Gulf of Guinea.
                self.position = if m.lat == 0 && m.lon == 0 {
                    None
                } else {
                    LatLon::from_mavlink_e7(m.lat, m.lon).ok()
                };
                self.altitude_msl = Metres::from_millimetres(m.alt);
                self.altitude_relative = Metres::from_millimetres(m.relative_alt);
                // hdg is centidegrees, 65535 when unknown.
                if m.hdg != u16::MAX {
                    self.heading = Bearing(Degrees(f64::from(m.hdg) / 100.0));
                }
                true
            }
            MavMessage::EkfStatusReport(m) => {
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
                self.vibration = crate::health::Vibration {
                    x: m.vibration_x,
                    y: m.vibration_y,
                    z: m.vibration_z,
                    clipping: [m.clipping_0, m.clipping_1, m.clipping_2],
                    seen: true,
                };
                true
            }
            MavMessage::GpsRawInt(m) => {
                self.gps = GpsInfo {
                    fix_type: m.fix_type,
                    satellites_visible: m.satellites_visible,
                    hdop: if m.eph == u16::MAX {
                        f32::NAN
                    } else {
                        f32::from(m.eph) / 100.0
                    },
                    vdop: if m.epv == u16::MAX {
                        f32::NAN
                    } else {
                        f32::from(m.epv) / 100.0
                    },
                };
                true
            }
            MavMessage::VfrHud(m) => {
                self.air_speed = MetresPerSecond(f64::from(m.airspeed));
                self.ground_speed = MetresPerSecond(f64::from(m.groundspeed));
                self.climb_rate = MetresPerSecond(f64::from(m.climb));
                self.throttle_percent = m.throttle as i16;
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
                true
            }
            // All sixteen channels ArduPilot maps to functions. RC_CHANNELS carries eighteen;
            // the last two are beyond what the firmware reads, and offering limits on channels
            // nothing looks at would be a calibration screen inviting a pointless setting.
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
            MavMessage::SysStatus(m) => {
                // The sensor masks are why an aircraft refuses to arm more often than anything
                // else, and they were being thrown away with the rest of this message.
                self.sensors = crate::sensors::Sensors {
                    present: m.onboard_control_sensors_present,
                    enabled: m.onboard_control_sensors_enabled,
                    health: m.onboard_control_sensors_health,
                    reported: true,
                };
                self.battery.voltage = f32::from(m.voltage_battery) / 1000.0;
                self.battery.current = if m.current_battery < 0 {
                    -1.0
                } else {
                    f32::from(m.current_battery) / 100.0
                };
                self.battery.remaining_percent = m.battery_remaining;
                true
            }
            MavMessage::BatteryStatus(m) => {
                self.battery.consumed_mah = m.current_consumed;
                if m.battery_remaining >= 0 {
                    self.battery.remaining_percent = m.battery_remaining;
                }
                true
            }
            MavMessage::HomePosition(m) => {
                self.home = LatLon::from_mavlink_e7(m.latitude, m.longitude).ok();
                true
            }
            _ => false,
        }
    }
}
