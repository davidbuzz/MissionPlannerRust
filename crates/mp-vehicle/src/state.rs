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
    /// Sensor health, as reported in `SYS_STATUS`.
    pub sensors: crate::sensors::Sensors,

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
