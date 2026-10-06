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

//! The `CurrentState` fields about the vehicle's own hardware and subsystems, ported from
//! `Parent_OnPacketReceived` in `crates/mp-vehicle/src/onboard.rs`.
//!
//! Constructed messages through [`VehicleState::apply`], checked against the C#'s arithmetic
//! with the lines cited - including where it keeps a raw unit, which id it listens to, and what
//! it does with a value that means "not supported".

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_possible_truncation
)]

use mp_mavlink::Message;
use mp_mavlink_dialects::all::{
    Ahrs2, AirspeedAutocal, AoaSsa, AutopilotVersion, EfiStatus, EscTelemetry1To4,
    EscTelemetry13To16, ExtendedSysState, FenceStatus, GeneratorStatus, GimbalDeviceAttitudeStatus,
    HighresImu, HomePosition, Hwstatus, HygrometerSensor, LocalPositionNed, MavMessage, McuStatus,
    Meminfo, MountStatus, OpticalFlow, PidTuning, PositionTargetGlobalInt, PowerStatus, RawImu,
    Rpm, ScaledImu2, ScaledImu3, SystemTime, UavionixAdsbOutStatus,
};
use mp_vehicle::VehicleState;

/// A message of the given type with every field zero, edited by `$body`.
macro_rules! message {
    ($ty:ident, |$m:ident| $body:expr) => {{
        let Some(MavMessage::$ty(mut $m)) = MavMessage::decode(<$ty as Message>::ID, &[]) else {
            panic!(concat!("no ", stringify!($ty)))
        };
        $body;
        MavMessage::$ty($m)
    }};
}

#[test]
fn the_imus_are_held_in_the_units_their_messages_use() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3688-3700, 3742-3783
    let mut state = VehicleState::default();
    state.apply(&message!(RawImu, |m| {
        m.xacc = 12;
        m.yacc = -7;
        m.zacc = -1001;
        m.xgyro = 3;
        m.xmag = 250;
        m.zmag = -400;
        m.temperature = 4512;
    }));
    let imu = state.imu[0];
    assert_eq!(imu.accel, [12.0, -7.0, -1001.0], "raw counts, unconverted");
    assert_eq!(imu.gyro, [3.0, 0.0, 0.0]);
    assert_eq!(imu.mag, [250.0, 0.0, -400.0]);
    assert_eq!(imu.temperature, 45.12, "temperature / 100.0f");

    state.apply(&message!(ScaledImu2, |m| {
        m.zacc = -998;
        m.temperature = -150;
    }));
    assert_eq!(state.imu[1].accel[2], -998.0);
    assert_eq!(state.imu[1].temperature, -1.5);
    state.apply(&message!(ScaledImu3, |m| m.ygyro = 17));
    assert_eq!(state.imu[2].gyro[1], 17.0);
    assert_eq!(state.imu[0], imu, "each message fills its own IMU");
}

#[test]
fn highres_imu_takes_only_what_its_mask_says_was_updated() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:4059-4176
    let mut state = VehicleState::default();
    state.apply(&message!(ScaledImu2, |m| m.xgyro = 5));
    state.apply(&message!(HighresImu, |m| {
        m.id = 1;
        m.fields_updated = 0x01 | 0x200; // XACC, ABS_PRESSURE
        m.xacc = 0.25;
        m.yacc = -0.5;
        m.zacc = -9.81;
        m.xgyro = 99.0;
        m.abs_pressure = 945.5;
        m.temperature = 25.9;
        m.pressure_alt = 600.0;
    }));
    assert_eq!(state.imu[1].accel, [0.25, -0.5, -9.81]);
    assert_eq!(state.imu[1].gyro[0], 5.0, "gyro not flagged, so kept");
    assert_eq!(state.press_abs, 945.5);
    assert_eq!(state.press_temp, 0, "temperature not flagged");
    assert_eq!(state.altitude_msl.0, 0.0, "pressure altitude not flagged");

    state.apply(&message!(HighresImu, |m| {
        m.fields_updated = 0x08 | 0x40 | 0x800 | 0x1000; // XGYRO, XMAG, PRESSURE_ALT, TEMPERATURE
        m.xgyro = 0.1;
        m.xmag = 0.2;
        m.pressure_alt = 600.5;
        m.temperature = -3.7;
    }));
    assert_eq!(state.imu[0].gyro[0], 0.1);
    assert_eq!(state.imu[0].mag[0], 0.2);
    assert_eq!(state.altitude_msl.0, 600.5);
    assert_eq!(state.press_temp, -3, "(int) truncates toward zero");
    assert!(
        !state.apply(&message!(HighresImu, |m| m.id = 3)),
        "three IMUs, ids 0 to 2"
    );
}

#[test]
fn esc_telemetry_lands_four_escs_at_a_time() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3531-3635
    let mut state = VehicleState::default();
    state.apply(&message!(EscTelemetry1To4, |m| {
        m.voltage = [1610, 1605, 1600, 1595];
        m.current = [1234, 0, 0, 1];
        m.rpm = [9000, 9100, 9200, 9300];
        m.temperature = [41, 42, 43, 44];
    }));
    assert_eq!(state.escs[0].voltage, 16.1, "voltage / 100.0f");
    assert_eq!(state.escs[0].current, 12.34);
    assert_eq!(state.escs[3].current, 0.01);
    assert_eq!(state.escs[2].rpm, 9200.0);
    assert_eq!(state.escs[3].temperature, 44.0);
    state.apply(&message!(EscTelemetry13To16, |m| m.rpm = [1, 2, 3, 4]));
    assert_eq!(state.escs[12].rpm, 1.0);
    assert_eq!(state.escs[15].rpm, 4.0);
    assert_eq!(state.escs[4].rpm, 0.0, "5 to 12 untouched");
}

#[test]
fn pid_tuning_keeps_the_slew_rate_per_axis() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3791-3820
    let mut state = VehicleState::default();
    state.apply(&message!(PidTuning, |m| {
        m.axis = 2;
        m.desired = 10.0;
        m.achieved = 9.5;
        m.ff = 0.1;
        m.p = 0.2;
        m.i = 0.3;
        m.d = 0.4;
        m.srate = 1.5;
        m.pdmod = 0.9;
    }));
    let pid = state.pid;
    assert_eq!(
        (
            pid.axis,
            pid.desired,
            pid.achieved,
            pid.ff,
            pid.p,
            pid.i,
            pid.d,
            pid.pdmod
        ),
        (2, 10.0, 9.5, 0.1, 0.2, 0.3, 0.4, 0.9)
    );
    assert_eq!(pid.srate_pitch, 1.5);
    assert_eq!(pid.srate_roll, 0.0);
    state.apply(&message!(PidTuning, |m| {
        m.axis = 6;
        m.srate = 2.5;
    }));
    assert_eq!(state.pid.srate_landing, 2.5);
    assert_eq!(state.pid.srate_pitch, 1.5, "each axis keeps its own");
    state.apply(&message!(PidTuning, |m| {
        m.axis = 9;
        m.srate = 7.0;
    }));
    assert_eq!(state.pid.srate, 7.0);
    assert_eq!(state.pid.srate_landing, 2.5, "an unknown axis has no slot");
}

#[test]
fn hygrometers_are_the_first_two_ids() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3831-3840
    let mut state = VehicleState::default();
    state.apply(&message!(HygrometerSensor, |m| {
        m.id = 1;
        m.temperature = 2250;
        m.humidity = 4510;
    }));
    assert_eq!(
        state.hygrometers[1].temperature, 2250,
        "centidegrees, as sent"
    );
    assert_eq!(state.hygrometers[1].humidity, 4510);
    assert!(!state.apply(&message!(HygrometerSensor, |m| m.id = 2)));
}

#[test]
fn the_generator_and_engine_are_copied_with_the_engines_unsupported_marker() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2413-2418, 2799-2819
    let mut state = VehicleState::default();
    state.apply(&message!(GeneratorStatus, |m| {
        m.status = 0x40;
        m.generator_speed = 6000;
        m.load_current = 12.5;
        m.bus_voltage = 48.25;
        m.runtime = 3600;
        m.time_until_maintenance = -60;
    }));
    let generator = state.generator;
    assert_eq!(
        (
            generator.status,
            generator.speed,
            generator.current,
            generator.voltage,
            generator.runtime,
            generator.maintenance_in
        ),
        (64.0, 6000.0, 12.5, 48.25, 3600, -60)
    );

    state.apply(&message!(EfiStatus, |m| {
        m.rpm = 5500.0;
        m.health = 1;
        m.cylinder_head_temperature = 180.0;
        m.fuel_pressure = 0.0;
    }));
    assert_eq!(state.efi.rpm, 5500.0);
    assert_eq!(state.efi.health, 1);
    assert_eq!(state.efi.head_temperature, 180.0);
    assert_eq!(
        state.efi.fuel_pressure, -1.0,
        "exactly 0 means not supported"
    );
    state.apply(&message!(EfiStatus, |m| m.fuel_pressure = 250.0));
    assert_eq!(state.efi.fuel_pressure, 250.0);
    assert!(
        !state.apply(&message!(EfiStatus, |m| {
            m.ecu_index = 1.0;
            m.rpm = 1.0;
        })),
        "the first ECU only"
    );
    assert_eq!(
        state.efi.rpm, 0.0,
        "the second message zeroed it; the ignored one did not"
    );
}

#[test]
fn the_transponder_status_is_unpacked_bit_by_bit() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:4034-4056
    let mut state = VehicleState::default();
    state.apply(&message!(UavionixAdsbOutStatus, |m| {
        m.state = 128 | 32 | 1;
        m.fault = 64 | 8;
        m.squawk = 7000;
        m.nic_nacp = 0xA7;
        m.boardtemp = 38;
        m.flight_id = *b"VH-ABC  ";
    }));
    let t = state.transponder;
    assert!(t.es1090_tx_enabled && t.mode_c_enabled && t.airborne);
    assert!(!t.mode_s_enabled && !t.mode_a_enabled && !t.ident_active && !t.x_bit_status);
    assert!(!t.interrogated_since_last);
    assert!(t.adsb_tx_failure && t.status_unavailable);
    assert!(!t.maintenance_required && !t.gps_unavailable && !t.gps_no_fix);
    assert_eq!(
        (t.squawk, t.nic, t.nacp, t.board_temperature),
        (7000, 7, 10, 38)
    );
    assert_eq!(&t.flight_id, b"VH-ABC  ");
    assert!(t.status_pending);
}

#[test]
fn ahrs2_is_converted_to_degrees_in_single_precision() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:4200-4205, `* (float)MathHelper.rad2deg`
    let mut state = VehicleState::default();
    state.apply(&message!(Ahrs2, |m| {
        m.roll = 0.5;
        m.yaw = -1.0;
        m.altitude = 584.5;
        m.lat = -353_632_620;
        m.lng = 1_491_652_370;
    }));
    let rad2deg = (180.0 / std::f64::consts::PI) as f32;
    assert_eq!(state.ahrs2.roll, 0.5 * rad2deg);
    assert_eq!(state.ahrs2.yaw, -rad2deg);
    assert_eq!(state.ahrs2.altitude, 584.5);
    assert_eq!(
        (state.ahrs2.lat, state.ahrs2.lng),
        (-35.363_262, 149.165_237)
    );
}

#[test]
fn the_board_reports_power_in_millivolts_memory_and_bus_errors() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2827-2832, 2642-2643, 3898-3902, and the
    // USB_CONNECTED default from ResetInternals at 4404.
    let mut state = VehicleState::default();
    assert_eq!(
        state.board.voltage_flags, 4,
        "USB_CONNECTED until told otherwise"
    );
    state.apply(&message!(PowerStatus, |m| {
        m.vcc = 5012;
        m.vservo = 5230;
        m.flags = 3;
    }));
    assert_eq!(
        state.board.board_voltage, 5012.0,
        "millivolts, as the C# shows them"
    );
    assert_eq!(state.board.servo_voltage, 5230.0);
    assert_eq!(state.board.voltage_flags, 3);
    state.apply(&message!(Hwstatus, |m| {
        m.vcc = 4987;
        m.i2cerr = 2;
    }));
    assert_eq!(
        state.board.hw_voltage, 4.987,
        "HWSTATUS is converted to volts"
    );
    assert_eq!(state.board.i2c_errors, 2);
    state.apply(&message!(Meminfo, |m| {
        m.freemem = 1234;
        m.brkval = 99;
    }));
    assert_eq!(state.board.free_memory, 1234.0);
    assert_eq!(state.board.brk_level, 99.0);
    state.apply(&message!(Meminfo, |m| {
        m.freemem = 1234;
        m.freemem32 = 200_000;
    }));
    assert_eq!(
        state.board.free_memory, 200_000.0,
        "the 32-bit figure when there is one"
    );
}

#[test]
fn the_mcu_and_fence_and_small_reports_are_copied() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:4181-4187, 2396-2398, 3064-3065, 2613, 3911-3912,
    // 3467-3468, 2336-2338, 2572-2576
    let mut state = VehicleState::default();
    state.apply(&message!(McuStatus, |m| {
        m.mcu_temperature = 5234;
        m.mcu_voltage = 3312;
        m.mcu_voltage_min = 3290;
        m.mcu_voltage_max = 3330;
    }));
    assert_eq!(
        (
            state.mcu.temperature,
            state.mcu.voltage,
            state.mcu.voltage_min,
            state.mcu.voltage_max
        ),
        (52.34, 3.312, 3.29, 3.33)
    );
    assert!(!state.apply(&message!(McuStatus, |m| m.id = 1)));

    state.apply(&message!(FenceStatus, |m| {
        m.breach_count = 3;
        m.breach_status = 1;
        m.breach_type = 2;
    }));
    assert_eq!(
        (
            state.fence_breach.count,
            state.fence_breach.status,
            state.fence_breach.breach_type
        ),
        (3, 1, 2)
    );

    state.apply(&message!(ExtendedSysState, |m| {
        m.vtol_state = 3;
        m.landed_state = 2;
    }));
    assert_eq!((state.vtol_state, state.landed_state), (3, 2));
    state.apply(&message!(AirspeedAutocal, |m| m.ratio = 1.98));
    assert_eq!(state.airspeed_ratio, 1.98);
    state.apply(&message!(AoaSsa, |m| {
        m.aoa = 4.5;
        m.ssa = -1.25;
    }));
    assert_eq!((state.aoa, state.ssa), (4.5, -1.25));
    state.apply(&message!(Rpm, |m| {
        m.rpm1 = 5100.0;
        m.rpm2 = 5200.0;
    }));
    assert_eq!(state.rpm, [5100.0, 5200.0]);
    state.apply(&message!(LocalPositionNed, |m| {
        m.x = 10.5;
        m.y = -3.0;
        m.z = -20.0;
    }));
    assert_eq!(state.local_position, [10.5, -3.0, -20.0]);
    state.apply(&message!(OpticalFlow, |m| {
        m.flow_comp_m_x = 0.25;
        m.flow_comp_m_y = -0.5;
        m.flow_x = 12;
        m.flow_y = -4;
        m.quality = 200;
    }));
    let flow = state.optical_flow;
    assert_eq!(
        (flow.comp_m_x, flow.comp_m_y, flow.x, flow.y, flow.quality),
        (0.25, -0.5, 12, -4, 200)
    );
}

#[test]
fn the_mount_points_where_mount_status_or_the_gimbal_quaternion_says() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2586-2588, 4023-4027; Quaternion.cs:331-352
    let mut state = VehicleState::default();
    state.apply(&message!(MountStatus, |m| {
        m.pointing_a = -4500;
        m.pointing_b = 150;
        m.pointing_c = 18_000;
    }));
    assert_eq!(
        (
            state.mount.pointing_a,
            state.mount.pointing_b,
            state.mount.pointing_c
        ),
        (-45.0, 1.5, 180.0)
    );

    let half = std::f32::consts::FRAC_PI_4;
    // A quarter turn left about the vertical: yaw -90, which the C# makes 270.
    state.apply(&message!(GimbalDeviceAttitudeStatus, |m| {
        m.q = [half.cos(), 0.0, 0.0, -half.sin()];
    }));
    assert!(
        (state.mount.pointing_c - 270.0).abs() < 1e-4,
        "{:?}",
        state.mount
    );
    assert!(state.mount.pointing_a.abs() < 1e-4);
    assert!(state.mount.pointing_b.abs() < 1e-4);
    // Thirty degrees nose down.
    let half = (-30.0_f32).to_radians() / 2.0;
    state.apply(&message!(GimbalDeviceAttitudeStatus, |m| {
        m.q = [half.cos(), 0.0, half.sin(), 0.0];
    }));
    assert!(
        (state.mount.pointing_a + 30.0).abs() < 1e-4,
        "{:?}",
        state.mount
    );
    assert!(state.mount.pointing_c.abs() < 1e-4);
}

#[test]
fn the_autopilot_version_is_split_into_its_bytes() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2368-2381
    let mut state = VehicleState::default();
    state.apply(&message!(AutopilotVersion, |m| {
        m.flight_sw_version = 0x0405_0103;
        m.uid = 0x0102_0304_0506_0708;
        m.uid2 = [7; 18];
        m.capabilities = 0x1_0000_FFEF;
    }));
    let info = state.autopilot_info;
    assert_eq!(info.version, [4, 5, 1, 3], "4.5.1, FIRMWARE_VERSION_TYPE 3");
    assert_eq!(info.uid, 0x0102_0304_0506_0708);
    assert_eq!(info.uid2, [7; 18]);
    assert_eq!(
        info.capabilities, 0xFFEF,
        "the C#'s (uint) keeps the low 32 bits"
    );
}

#[test]
fn system_time_is_kept_unless_a_datetime_cannot_hold_it() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2623-2632
    let mut state = VehicleState::default();
    state.apply(&message!(SystemTime, |m| m.time_unix_usec = 1_700_000_000_123_456));
    assert_eq!(state.gps_time_unix_ms, 1_700_000_000_123);
    assert!(!state.apply(&message!(SystemTime, |m| m.time_unix_usec = u64::MAX)));
    assert_eq!(state.gps_time_unix_ms, 1_700_000_000_123);
}

#[test]
fn the_position_target_is_read_in_two_frames() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2348-2356
    let mut state = VehicleState::default();
    state.apply(&message!(HomePosition, |m| {
        m.latitude = -353_632_620;
        m.longitude = 1_491_652_370;
        m.altitude = 584_000;
    }));
    state.apply(&message!(PositionTargetGlobalInt, |m| {
        m.coordinate_frame = 5; // GLOBAL_INT
        m.lat_int = -353_600_000;
        m.lon_int = 1_491_600_000;
        m.alt = 620.5;
    }));
    let target = state.target_position.expect("a target");
    assert_eq!((target.latitude(), target.longitude()), (-35.36, 149.16));
    assert_eq!(state.target_altitude_msl.0, 620.5);
    state.apply(&message!(PositionTargetGlobalInt, |m| {
        m.coordinate_frame = 3; // GLOBAL_RELATIVE_ALT
        m.lat_int = -353_600_000;
        m.lon_int = 1_491_600_000;
        m.alt = 50.0;
    }));
    assert_eq!(state.target_altitude_msl.0, 634.0, "above home, plus home");
    assert!(!state.apply(&message!(PositionTargetGlobalInt, |m| {
        m.coordinate_frame = 6;
        m.alt = 1.0;
    })));
    assert_eq!(
        state.target_altitude_msl.0, 634.0,
        "other frames are ignored"
    );
}
