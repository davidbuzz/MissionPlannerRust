//! What the vehicle reports about its own hardware and subsystems.
//!
//! The rest of `CurrentState.Parent_OnPacketReceived`: IMUs, ESCs, the engine, the generator,
//! the transponder, the mount, the second AHRS, the board's power and memory, the firmware
//! version. Most of it is copied as sent, and the C# rarely converts: where it holds a raw sensor
//! unit, or different units from different messages in one property, so does this, and the
//! field says which.

use mp_mavlink_dialects::all::{
    AutopilotVersion, EfiStatus, GimbalDeviceAttitudeStatus, HighresImu, MavMessage,
    PositionTargetGlobalInt, UavionixAdsbOutStatus,
};
use mp_units::{LatLon, Metres};

use crate::state::VehicleState;

/// One inertial measurement unit: accelerometer, gyro and compass on each axis, and its
/// temperature (`ax` to `mz` and `imu1_temp`, with `2` and `3` for the others).
///
/// In the units of the message that last filled it, as the C# keeps them: `RAW_IMU` is the
/// sensor's raw counts, `SCALED_IMU` milli-g, millirad/s and milligauss, `HIGHRES_IMU` m/s²,
/// rad/s and gauss.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Imu {
    /// Acceleration, x, y and z.
    pub accel: [f32; 3],
    /// Angular rate, x, y and z.
    pub gyro: [f32; 3],
    /// Magnetic field, x, y and z.
    pub mag: [f32; 3],
    /// Temperature, degrees Celsius (`temperature / 100`). `HIGHRES_IMU` does not set it.
    pub temperature: f32,
}

/// One ESC's telemetry, from the four `ESC_TELEMETRY_*` messages (`esc1_volt` and on).
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3528-3632`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Esc {
    /// Volts (`voltage / 100`).
    pub voltage: f32,
    /// Amps (`current / 100`).
    pub current: f32,
    /// Revolutions per minute.
    pub rpm: f32,
    /// Degrees Celsius.
    pub temperature: f32,
}

/// The last `PID_TUNING` report (`pidff` to `pidSRateLanding`).
///
/// One axis at a time, except the slew rates, which are kept per axis as the C# keeps them.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3782-3821`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PidTuning {
    /// Which axis the rest is about: `PID_TUNING_AXIS`.
    pub axis: u8,
    /// Feed-forward component.
    pub ff: f32,
    /// Proportional component.
    pub p: f32,
    /// Integral component.
    pub i: f32,
    /// Derivative component.
    pub d: f32,
    /// Desired rate.
    pub desired: f32,
    /// Achieved rate.
    pub achieved: f32,
    /// Slew rate.
    pub srate: f32,
    /// P and D gain modifier.
    pub pdmod: f32,
    /// The last slew rate reported for roll.
    pub srate_roll: f32,
    /// For pitch.
    pub srate_pitch: f32,
    /// For yaw.
    pub srate_yaw: f32,
    /// For vertical acceleration.
    pub srate_accz: f32,
    /// For steering.
    pub srate_steer: f32,
    /// For landing.
    pub srate_landing: f32,
}

/// One hygrometer, from `HYGROMETER_SENSOR` (`hygrotemp1`, `hygrohumi1`, and `2`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Hygrometer {
    /// Temperature, centidegrees Celsius, as sent.
    pub temperature: i16,
    /// Relative humidity, centi-percent, as sent.
    pub humidity: u16,
}

/// The generator, from `GENERATOR_STATUS` (`gen_status` to `gen_maint_time`).
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2409-2419`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Generator {
    /// `MAV_GENERATOR_STATUS_FLAG` bits, as the single-precision number the C# holds.
    pub status: f32,
    /// Revolutions per minute.
    pub speed: f32,
    /// Load current, amps.
    pub current: f32,
    /// Bus voltage, volts.
    pub voltage: f32,
    /// Seconds run.
    pub runtime: u32,
    /// Seconds until maintenance, negative when overdue.
    pub maintenance_in: i32,
}

/// An electronic fuel-injected engine, from `EFI_STATUS` for the first ECU (`efi_*`).
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2794-2820`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Efi {
    /// Barometric pressure, kPa.
    pub baro: f32,
    /// Cylinder head temperature, degrees Celsius.
    pub head_temperature: f32,
    /// Engine load, percent.
    pub load: f32,
    /// Health: 1 good, 0 not.
    pub health: u8,
    /// Exhaust gas temperature, degrees Celsius.
    pub exhaust_temperature: f32,
    /// Intake manifold temperature, degrees Celsius.
    pub intake_temperature: f32,
    /// Revolutions per minute.
    pub rpm: f32,
    /// Fuel flow, cm³/min.
    pub fuel_flow: f32,
    /// Fuel consumed, cm³.
    pub fuel_consumed: f32,
    /// Fuel pressure, kPa; -1 when the engine does not measure it (the wire's exact 0).
    pub fuel_pressure: f32,
}

/// A uAvionix ADS-B transponder's own status, from `UAVIONIX_ADSB_OUT_STATUS` (`xpdr_*`).
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4027-4055`
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)] // the C#'s flags, one field each
pub struct Transponder {
    /// 1090ES transmission enabled.
    pub es1090_tx_enabled: bool,
    /// Mode S replies enabled.
    pub mode_s_enabled: bool,
    /// Mode C replies enabled.
    pub mode_c_enabled: bool,
    /// Mode A replies enabled.
    pub mode_a_enabled: bool,
    /// Ident active.
    pub ident_active: bool,
    /// X-bit status.
    pub x_bit_status: bool,
    /// Interrogated since the last status.
    pub interrogated_since_last: bool,
    /// Airborne.
    pub airborne: bool,
    /// Mode A squawk code.
    pub squawk: u16,
    /// Navigation integrity category.
    pub nic: u8,
    /// Navigation accuracy category, position.
    pub nacp: u8,
    /// Board temperature, degrees Celsius.
    pub board_temperature: u8,
    /// Maintenance required.
    pub maintenance_required: bool,
    /// ADS-B transmit system failure.
    pub adsb_tx_failure: bool,
    /// GPS unavailable.
    pub gps_unavailable: bool,
    /// GPS has no fix.
    pub gps_no_fix: bool,
    /// No status message received from the transponder.
    pub status_unavailable: bool,
    /// A status has arrived. The C# clears this once its display has shown it
    /// (`GCSViews/FlightData.cs:6481`); a snapshot reader keeps that note itself.
    pub status_pending: bool,
    /// How many statuses have arrived, wrapping: `xpdr_status_pending` as a snapshot reader can
    /// clear it. The C# sets the flag on every status and the Transponder page clears it each
    /// time it looks (`GCSViews/FlightData.cs:6373, 6481`), so a transponder still reporting is
    /// told from one that has stopped even when its status repeats unchanged; a reader here, which
    /// cannot write the flag back, keeps the count it last looked at, and a status is pending
    /// while this differs from it.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2267, 4053`
    pub status_count: u32,
    /// Callsign or flight id, as sent.
    pub flight_id: [u8; 8],
}

/// The second attitude and heading estimate, from `AHRS2` (`ahrs2_*`).
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4193-4203`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Ahrs2 {
    /// Roll, degrees.
    pub roll: f32,
    /// Pitch, degrees.
    pub pitch: f32,
    /// Yaw, degrees.
    pub yaw: f32,
    /// Altitude, metres above sea level.
    pub altitude: f32,
    /// Latitude, degrees.
    pub lat: f64,
    /// Longitude, degrees.
    pub lng: f64,
}

/// The flight controller's processor, from `MCU_STATUS` for the first one (`mcu*`).
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4175-4185`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Mcu {
    /// Temperature, degrees Celsius.
    pub temperature: f32,
    /// Supply voltage, volts.
    pub voltage: f32,
    /// Lowest supply voltage seen, volts.
    pub voltage_min: f32,
    /// Highest supply voltage seen, volts.
    pub voltage_max: f32,
}

/// The geofence's breach status, from `FENCE_STATUS` (`fenceb_*`).
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2392-2407`
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FenceBreach {
    /// Breaches so far.
    pub count: u16,
    /// Non-zero while breached.
    pub status: u8,
    /// `FENCE_BREACH`: which limit.
    pub breach_type: u8,
}

/// Where the camera mount points, degrees (`campointa`, `campointb`, `campointc`).
///
/// From `MOUNT_STATUS`, pitch, roll and yaw in its usual mode; or from
/// `GIMBAL_DEVICE_ATTITUDE_STATUS`, converted from its quaternion.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Mount {
    /// Pitch (`campointa`).
    pub pointing_a: f32,
    /// Roll (`campointb`).
    pub pointing_b: f32,
    /// Yaw (`campointc`); 0 to 360 from the gimbal-device message.
    pub pointing_c: f32,
}

/// The optical flow sensor, from `OPTICAL_FLOW` (`opt_*`).
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2566-2578`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct OpticalFlow {
    /// Compensated flow, x, m/s.
    pub comp_m_x: f32,
    /// Compensated flow, y, m/s.
    pub comp_m_y: f32,
    /// Raw flow, x, dpix.
    pub x: i16,
    /// Raw flow, y, dpix.
    pub y: i16,
    /// Quality, 0 to 255.
    pub quality: u8,
}

/// What the autopilot says it is, from `AUTOPILOT_VERSION`.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2360-2390`
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AutopilotInfo {
    /// Firmware version: major, minor, patch and `FIRMWARE_VERSION_TYPE`, the four bytes of
    /// `flight_sw_version` from the top (`version`).
    pub version: [u8; 4],
    /// The board's unique id (`uid`).
    pub uid: u64,
    /// The board's longer unique id (`uid2`), as bytes; the C# holds it as hex.
    pub uid2: [u8; 18],
    /// `MAV_PROTOCOL_CAPABILITY` bits, the low 32 as the C# keeps them (`capabilities`).
    pub capabilities: u32,
}

/// The flight controller board: its power rails, memory and bus errors, from `POWER_STATUS`,
/// `HWSTATUS` and `MEMINFO`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Board {
    /// The board's 5 V rail, *millivolts*: the C# shows `POWER_STATUS.Vcc` unconverted
    /// (`boardvoltage`).
    pub board_voltage: f32,
    /// The servo rail, millivolts, likewise (`servovoltage`).
    pub servo_voltage: f32,
    /// `MAV_POWER_STATUS` bits (`voltageflag`). `USB_CONNECTED` until the first report, as the
    /// C#'s `ResetInternals` sets it.
    pub voltage_flags: u32,
    /// The supply voltage from `HWSTATUS`, volts (`hwvoltage`).
    pub hw_voltage: f32,
    /// I2C errors from `HWSTATUS` (`i2cerrors`).
    pub i2c_errors: u16,
    /// Free memory, bytes (`freemem`).
    pub free_memory: f32,
    /// The heap top (`brklevel`).
    pub brk_level: f32,
}

/// `MAV_POWER_STATUS_USB_CONNECTED`. `// C#: ExtLibs/Mavlink/Mavlink.cs:3798`
const POWER_USB_CONNECTED: u32 = 4;

impl Default for Board {
    fn default() -> Self {
        Self {
            board_voltage: 0.0,
            servo_voltage: 0.0,
            // C#: ExtLibs/ArduPilot/CurrentState.cs:4404
            voltage_flags: POWER_USB_CONNECTED,
            hw_voltage: 0.0,
            i2c_errors: 0,
            free_memory: 0.0,
            brk_level: 0.0,
        }
    }
}

/// `HIGHRES_IMU.fields_updated` bits the C# reads. `// C#: ExtLibs/ArduPilot/CurrentState.cs:4058-4071`
const HIGHRES_XACC: u16 = 0x01;
const HIGHRES_XGYRO: u16 = 0x08;
const HIGHRES_XMAG: u16 = 0x40;
const HIGHRES_ABS_PRESSURE: u16 = 0x200;
const HIGHRES_PRESSURE_ALT: u16 = 0x800;
const HIGHRES_TEMPERATURE: u16 = 0x1000;

/// `MAV_FRAME_GLOBAL_RELATIVE_ALT` and `MAV_FRAME_GLOBAL_INT`, the two frames the C# reads a
/// position target in. `// C#: ExtLibs/Mavlink/Mavlink.cs:2963, 2970`
const FRAME_GLOBAL_RELATIVE_ALT: u8 = 3;
const FRAME_GLOBAL_INT: u8 = 5;

/// The latest date `DateTime.AddMilliseconds` reaches from 1970 before it throws, which the C#
/// catches and ignores: 9999-12-31T23:59:59.999, in milliseconds since the Unix epoch.
const DATETIME_MAX_UNIX_MS: u64 = 253_402_300_799_999;

impl VehicleState {
    /// The subsystem messages. `None` when the message is not one of them.
    #[allow(clippy::too_many_lines)] // a flat dispatch over message types is clearer than nesting
    pub(crate) fn apply_onboard(&mut self, message: &MavMessage) -> Option<bool> {
        match message {
            MavMessage::RawImu(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3685-3697
                self.imu[0] = Imu {
                    accel: [m.xacc, m.yacc, m.zacc].map(f32::from),
                    gyro: [m.xgyro, m.ygyro, m.zgyro].map(f32::from),
                    mag: [m.xmag, m.ymag, m.zmag].map(f32::from),
                    temperature: f32::from(m.temperature) / 100.0,
                };
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3699-3713
                self.update_speedup(m.time_usec);
            }
            MavMessage::RcChannelsScaled(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2307-2324, "hil mavlink 0.9": every
                // port's, as the C# does not look at it.
                self.hil_channels = [
                    m.chan1_scaled,
                    m.chan2_scaled,
                    m.chan3_scaled,
                    m.chan4_scaled,
                    m.chan5_scaled,
                    m.chan6_scaled,
                    m.chan7_scaled,
                    m.chan8_scaled,
                ]
                .map(i32::from);
            }
            MavMessage::HilControls(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2551-2564, the first four, the other four
                // kept.
                let [ch1, ch2, ch3, ch4, ..] = &mut self.hil_channels;
                *ch1 = hil_control(m.roll_ailerons);
                *ch2 = hil_control(m.pitch_elevator);
                *ch3 = hil_control(m.throttle);
                *ch4 = hil_control(m.yaw_rudder);
            }
            MavMessage::NamedValueFloat(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3913-4012. With every field named, the
                // value goes nowhere.
                let Some(field) = Self::custom_field_for(&m.name)
                    .and_then(|index| self.custom_fields.get_mut(index))
                else {
                    return Some(false);
                };
                *field = m.value;
            }
            MavMessage::ScaledImu(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3723-3735
                self.imu[0] = scaled_imu(
                    [m.xacc, m.yacc, m.zacc],
                    [m.xgyro, m.ygyro, m.zgyro],
                    [m.xmag, m.ymag, m.zmag],
                    m.temperature,
                );
            }
            MavMessage::ScaledImu2(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3739-3759
                self.imu[1] = scaled_imu(
                    [m.xacc, m.yacc, m.zacc],
                    [m.xgyro, m.ygyro, m.zgyro],
                    [m.xmag, m.ymag, m.zmag],
                    m.temperature,
                );
            }
            MavMessage::ScaledImu3(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3761-3780
                self.imu[2] = scaled_imu(
                    [m.xacc, m.yacc, m.zacc],
                    [m.xgyro, m.ygyro, m.zgyro],
                    [m.xmag, m.ymag, m.zmag],
                    m.temperature,
                );
            }
            MavMessage::HighresImu(m) => return Some(self.apply_highres_imu(m)),
            MavMessage::EscTelemetry1To4(m) => {
                self.apply_esc(0, m.voltage, m.current, m.rpm, m.temperature);
            }
            MavMessage::EscTelemetry5To8(m) => {
                self.apply_esc(4, m.voltage, m.current, m.rpm, m.temperature);
            }
            MavMessage::EscTelemetry9To12(m) => {
                self.apply_esc(8, m.voltage, m.current, m.rpm, m.temperature);
            }
            MavMessage::EscTelemetry13To16(m) => {
                self.apply_esc(12, m.voltage, m.current, m.rpm, m.temperature);
            }
            MavMessage::PidTuning(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3788-3817
                let pid = &mut self.pid;
                pid.ff = m.ff;
                pid.p = m.p;
                pid.i = m.i;
                pid.d = m.d;
                pid.axis = m.axis;
                pid.desired = m.desired;
                pid.achieved = m.achieved;
                pid.srate = m.srate;
                pid.pdmod = m.pdmod;
                match m.axis {
                    1 => pid.srate_roll = m.srate,
                    2 => pid.srate_pitch = m.srate,
                    3 => pid.srate_yaw = m.srate,
                    4 => pid.srate_accz = m.srate,
                    5 => pid.srate_steer = m.srate,
                    6 => pid.srate_landing = m.srate,
                    _ => {}
                }
            }
            MavMessage::HygrometerSensor(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3828-3837, the first two sensors.
                let Some(hygrometer) = self.hygrometers.get_mut(usize::from(m.id)) else {
                    return Some(false);
                };
                *hygrometer = Hygrometer {
                    temperature: m.temperature,
                    humidity: m.humidity,
                };
            }
            MavMessage::GeneratorStatus(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2413-2418
                self.generator = Generator {
                    status: u64_f32(m.status),
                    speed: f32::from(m.generator_speed),
                    current: m.load_current,
                    voltage: m.bus_voltage,
                    runtime: m.runtime,
                    maintenance_in: m.time_until_maintenance,
                };
            }
            MavMessage::EfiStatus(m) => return Some(self.apply_efi(m)),
            MavMessage::UavionixAdsbOutStatus(m) => self.apply_transponder(m),
            MavMessage::Ahrs2(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:4197-4202, `(float)MathHelper.rad2deg`.
                let rad2deg = cast_f32(180.0 / std::f64::consts::PI);
                self.ahrs2 = Ahrs2 {
                    roll: m.roll * rad2deg,
                    pitch: m.pitch * rad2deg,
                    yaw: m.yaw * rad2deg,
                    altitude: m.altitude,
                    lat: f64::from(m.lat) / 1e7,
                    lng: f64::from(m.lng) / 1e7,
                };
            }
            MavMessage::McuStatus(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:4178-4184, the first processor.
                if m.id != 0 {
                    return Some(false);
                }
                self.mcu = Mcu {
                    temperature: f32::from(m.mcu_temperature) / 100.0,
                    voltage: f32::from(m.mcu_voltage) / 1000.0,
                    voltage_min: f32::from(m.mcu_voltage_min) / 1000.0,
                    voltage_max: f32::from(m.mcu_voltage_max) / 1000.0,
                };
            }
            MavMessage::FenceStatus(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2396-2398. The breach message it raises
                // belongs to the display.
                self.fence_breach = FenceBreach {
                    count: m.breach_count,
                    status: m.breach_status,
                    breach_type: m.breach_type,
                };
            }
            MavMessage::MountStatus(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2586-2588
                self.mount = Mount {
                    pointing_a: i32_f32(m.pointing_a) / 100.0,
                    pointing_b: i32_f32(m.pointing_b) / 100.0,
                    pointing_c: i32_f32(m.pointing_c) / 100.0,
                };
            }
            MavMessage::GimbalDeviceAttitudeStatus(m) => self.apply_gimbal_attitude(m),
            MavMessage::OpticalFlow(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2572-2576
                self.optical_flow = OpticalFlow {
                    comp_m_x: m.flow_comp_m_x,
                    comp_m_y: m.flow_comp_m_y,
                    x: m.flow_x,
                    y: m.flow_y,
                    quality: m.quality,
                };
            }
            MavMessage::AutopilotVersion(m) => self.apply_autopilot_version(m),
            MavMessage::PowerStatus(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2827-2832: millivolts, unconverted. The
                // over-current messages it raises belong to the display.
                self.board.board_voltage = f32::from(m.vcc);
                self.board.servo_voltage = f32::from(m.vservo);
                self.board.voltage_flags = u32::from(m.flags);
            }
            MavMessage::Hwstatus(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2642-2643
                self.board.hw_voltage = f32::from(m.vcc) / 1000.0;
                self.board.i2c_errors = u16::from(m.i2cerr);
            }
            MavMessage::Meminfo(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3895-3899
                self.board.free_memory = if m.freemem32 > 0 {
                    u32_f32(m.freemem32)
                } else {
                    f32::from(m.freemem)
                };
                self.board.brk_level = f32::from(m.brkval);
            }
            MavMessage::ExtendedSysState(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3064-3065
                self.vtol_state = m.vtol_state;
                self.landed_state = m.landed_state;
            }
            MavMessage::AirspeedAutocal(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2613
                self.airspeed_ratio = m.ratio;
            }
            MavMessage::AoaSsa(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3908-3909
                self.aoa = m.aoa;
                self.ssa = m.ssa;
            }
            MavMessage::Rpm(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:3467-3468
                self.rpm = [m.rpm1, m.rpm2];
            }
            MavMessage::SystemTime(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2623-2632. `AddMilliseconds` past year
                // 9999 throws, and the C# keeps the last time.
                let milliseconds = m.time_unix_usec / 1000;
                if milliseconds > DATETIME_MAX_UNIX_MS {
                    return Some(false);
                }
                self.gps_time_unix_ms = milliseconds;
            }
            MavMessage::LocalPositionNed(m) => {
                // C#: ExtLibs/ArduPilot/CurrentState.cs:2336-2338
                self.local_position = [m.x, m.y, m.z];
            }
            MavMessage::PositionTargetGlobalInt(m) => {
                return Some(self.apply_position_target(m));
            }
            _ => return None,
        }
        Some(true)
    }

    /// `HIGHRES_IMU`, by sensor id, only the fields its mask says were updated.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4056-4173`
    fn apply_highres_imu(&mut self, m: &HighresImu) -> bool {
        let Some(imu) = self.imu.get_mut(usize::from(m.id)) else {
            return false;
        };
        if m.fields_updated & HIGHRES_XACC != 0 {
            imu.accel = [m.xacc, m.yacc, m.zacc];
        }
        if m.fields_updated & HIGHRES_XGYRO != 0 {
            imu.gyro = [m.xgyro, m.ygyro, m.zgyro];
        }
        if m.fields_updated & HIGHRES_XMAG != 0 {
            imu.mag = [m.xmag, m.ymag, m.zmag];
        }
        // Every id writes the one barometer's fields, and the altitude above sea level.
        if m.fields_updated & HIGHRES_ABS_PRESSURE != 0 {
            self.press_abs = m.abs_pressure;
        }
        if m.fields_updated & HIGHRES_TEMPERATURE != 0 {
            // `(int)imu.temperature`: truncated toward zero, whole degrees.
            self.press_temp = truncate_i32(m.temperature);
        }
        if m.fields_updated & HIGHRES_PRESSURE_ALT != 0 {
            self.altitude_msl = Metres(f64::from(m.pressure_alt));
        }
        true
    }

    /// Four ESCs from one `ESC_TELEMETRY_*` message, starting at `first`.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3532-3550` and the three cases after it.
    fn apply_esc(
        &mut self,
        first: usize,
        voltage: [u16; 4],
        current: [u16; 4],
        rpm: [u16; 4],
        temperature: [u8; 4],
    ) {
        let reported = voltage.into_iter().zip(current).zip(rpm).zip(temperature);
        for (esc, (((voltage, current), rpm), temperature)) in
            self.escs.iter_mut().skip(first).zip(reported)
        {
            *esc = Esc {
                voltage: f32::from(voltage) / 100.0,
                current: f32::from(current) / 100.0,
                rpm: f32::from(rpm),
                temperature: f32::from(temperature),
            };
        }
    }

    /// `EFI_STATUS`, the first ECU only. `// C#: ExtLibs/ArduPilot/CurrentState.cs:2799-2819`
    fn apply_efi(&mut self, m: &EfiStatus) -> bool {
        if m.ecu_index != 0.0 {
            return false;
        }
        self.efi = Efi {
            baro: m.barometric_pressure,
            head_temperature: m.cylinder_head_temperature,
            load: m.engine_load,
            health: m.health,
            exhaust_temperature: m.exhaust_gas_temperature,
            intake_temperature: m.intake_manifold_temperature,
            rpm: m.rpm,
            fuel_flow: m.fuel_flow,
            fuel_consumed: m.fuel_consumed,
            // "A value of exactly zero indicates that fuel pressure is not supported"
            fuel_pressure: if m.fuel_pressure == 0.0 {
                -1.0
            } else {
                m.fuel_pressure
            },
        };
        true
    }

    /// `UAVIONIX_ADSB_OUT_STATUS`. `// C#: ExtLibs/ArduPilot/CurrentState.cs:4031-4053`
    fn apply_transponder(&mut self, m: &UavionixAdsbOutStatus) {
        let state = |bit: u8| m.state & bit != 0;
        let fault = |bit: u8| m.fault & bit != 0;
        self.transponder = Transponder {
            es1090_tx_enabled: state(128),
            mode_s_enabled: state(64),
            mode_c_enabled: state(32),
            mode_a_enabled: state(16),
            ident_active: state(8),
            x_bit_status: state(4),
            interrogated_since_last: state(2),
            airborne: state(1),
            squawk: m.squawk,
            nic: m.nic_nacp & 0x0F,
            nacp: (m.nic_nacp >> 4) & 0x0F,
            board_temperature: m.boardtemp,
            maintenance_required: fault(128),
            adsb_tx_failure: fault(64),
            gps_unavailable: fault(32),
            gps_no_fix: fault(16),
            status_unavailable: fault(8),
            status_pending: true,
            // C#: ExtLibs/ArduPilot/CurrentState.cs:4053, `xpdr_status_pending = true;` on
            // every status: one more.
            status_count: self.transponder.status_count.wrapping_add(1),
            flight_id: m.flight_id,
        };
    }

    /// `GIMBAL_DEVICE_ATTITUDE_STATUS`: the quaternion as pitch, roll and yaw in degrees, yaw
    /// from 0 to 360, with the C#'s `Quaternion.get_euler_*`.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4020-4024; ExtLibs/Utilities/Quaternion.cs:331-352`
    fn apply_gimbal_attitude(&mut self, m: &GimbalDeviceAttitudeStatus) {
        let [q1, q2, q3, q4] = m.q.map(f64::from);
        let roll = (2.0 * (q1 * q2 + q3 * q4)).atan2(1.0 - 2.0 * (q2 * q2 + q3 * q3));
        let pitch = (2.0 * (q1 * q3 - q4 * q2)).asin();
        let yaw = (2.0 * (q1 * q4 + q2 * q3)).atan2(1.0 - 2.0 * (q3 * q3 + q4 * q4));
        let degrees = |radians: f64| cast_f32(radians * (180.0 / std::f64::consts::PI));
        let mut pointing_c = degrees(yaw);
        if pointing_c < 0.0 {
            pointing_c += 360.0;
        }
        self.mount = Mount {
            pointing_a: degrees(pitch),
            pointing_b: degrees(roll),
            pointing_c,
        };
    }

    /// `AUTOPILOT_VERSION`. `// C#: ExtLibs/ArduPilot/CurrentState.cs:2368-2381`
    fn apply_autopilot_version(&mut self, m: &AutopilotVersion) {
        #[allow(clippy::cast_possible_truncation)] // the low 32 bits, as the C#'s `(uint)` cast
        let capabilities = m.capabilities as u32;
        self.autopilot_info = AutopilotInfo {
            version: m.flight_sw_version.to_be_bytes(),
            uid: m.uid,
            uid2: m.uid2,
            capabilities,
        };
    }

    /// `POSITION_TARGET_GLOBAL_INT`, in the two frames the C# reads: above sea level, or above
    /// home with the home altitude added. `// C#: ExtLibs/ArduPilot/CurrentState.cs:2348-2356`
    ///
    /// The C# tags the point with the type mask, for display; that tag is not kept.
    fn apply_position_target(&mut self, m: &PositionTargetGlobalInt) -> bool {
        let altitude = match m.coordinate_frame {
            FRAME_GLOBAL_INT => f64::from(m.alt),
            FRAME_GLOBAL_RELATIVE_ALT => f64::from(m.alt) + self.home_altitude.0,
            _ => return false,
        };
        self.target_position =
            LatLon::new(f64::from(m.lat_int) / 1e7, f64::from(m.lon_int) / 1e7).ok();
        self.target_altitude_msl = Metres(altitude);
        true
    }
}

/// `SCALED_IMU`, `SCALED_IMU2` and `SCALED_IMU3` share a shape.
fn scaled_imu(accel: [i16; 3], gyro: [i16; 3], mag: [i16; 3], temperature: i16) -> Imu {
    Imu {
        accel: accel.map(f32::from),
        gyro: gyro.map(f32::from),
        mag: mag.map(f32::from),
        temperature: f32::from(temperature) / 100.0,
    }
}

/// A double narrowed to the single precision a C# `float` holds.
#[allow(clippy::cast_possible_truncation)]
const fn cast_f32(value: f64) -> f32 {
    value as f32
}

/// A `uint` in a C# `float`: the nearest single-precision value.
#[allow(clippy::cast_precision_loss)]
const fn u32_f32(value: u32) -> f32 {
    value as f32
}

/// An `int` in a C# `float`: the nearest single-precision value.
#[allow(clippy::cast_precision_loss)]
const fn i32_f32(value: i32) -> f32 {
    value as f32
}

/// A `ulong` in a C# `float`: the nearest single-precision value.
#[allow(clippy::cast_precision_loss)]
const fn u64_f32(value: u64) -> f32 {
    value as f32
}

/// `HIL_CONTROLS`' `(int)(control * 10000)`: the product in single precision, then toward zero.
///
/// NaN or out of `int`'s range gives `int.MinValue`, which is what the C#'s conversion does on
/// the x86 and x64 processors it runs on (`cvttss2si`'s "integer indefinite"); a simulator
/// sending a NaN control would show as -2147483648 in the C#, and does here.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2557-2560`
fn hil_control(control: f32) -> i32 {
    let scaled = control * 10000.0;
    // 2^31 is exact in single precision; -2^31 converts, anything at or past 2^31 does not, and
    // NaN is in no range.
    if !(-2_147_483_648.0..2_147_483_648.0).contains(&scaled) {
        return i32::MIN;
    }
    truncate_i32(scaled)
}

/// C#'s `(int)` of a `float`: toward zero. Out of range saturates here where the C# is
/// unspecified; no barometer reports two billion degrees.
#[allow(clippy::cast_possible_truncation)]
const fn truncate_i32(value: f32) -> i32 {
    value as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One `UAVIONIX_ADSB_OUT_STATUS`, the same each time.
    fn status() -> MavMessage {
        MavMessage::UavionixAdsbOutStatus(UavionixAdsbOutStatus {
            squawk: 1200,
            state: 16 | 64 | 128,
            nic_nacp: 0x9A,
            boardtemp: 30,
            fault: 0,
            flight_id: *b"QFA1\0\0\0\0",
        })
    }

    /// Every status counts one, repeated or not, as each sets `xpdr_status_pending`; the count
    /// is all that tells the second of two identical statuses from the first.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4027-4053`
    #[test]
    fn every_transponder_status_counts_one_even_when_it_repeats() {
        let mut state = VehicleState::default();
        assert_eq!(state.transponder.status_count, 0);
        assert!(!state.transponder.status_pending);
        state.apply(&status());
        let first = state.transponder;
        assert_eq!(first.status_count, 1);
        assert!(first.status_pending);
        state.apply(&status());
        let second = state.transponder;
        assert_eq!(second.status_count, 2);
        assert_eq!(
            Transponder {
                status_count: 1,
                ..second
            },
            first,
            "the same status but for the count"
        );
        // Wrapping: a reader compares the count for a difference only.
        state.transponder.status_count = u32::MAX;
        state.apply(&status());
        assert_eq!(state.transponder.status_count, 0);
    }
}
