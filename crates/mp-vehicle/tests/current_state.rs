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

//! The `CurrentState` fields ported from `Parent_OnPacketReceived`, one message at a time.
//!
//! Each test feeds constructed messages through [`VehicleState::apply`] and checks the result
//! against the C#'s own arithmetic, cited line by line - including the quirks: which fields keep
//! their last value on an "unknown" marker, which are filtered, which are clamped, and the few
//! places the C# stores a unit other than the one its neighbours use. `crates/mp-vehicle/src/
//! coverage.rs` says which C# property each of these fields stands for.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_possible_truncation
)]

use mp_mavlink::Message;
use mp_mavlink_dialects::all::{
    Airspeed, Battery2, BatteryStatus, DistanceSensor, EkfStatusReport, GlobalPositionInt, Gps2Raw,
    GpsRawInt, GpsStatus, Heartbeat, HighLatency, HighLatency2, HomePosition, MavMessage,
    MavSysStatusSensor, MissionCurrent, Radio, RadioStatus, Rangefinder, ScaledPressure,
    ScaledPressure2, ServoOutputRaw, SysStatus, TerrainReport, VfrHud, Wind,
};
use mp_vehicle::{VehicleId, VehicleRegistry, VehicleState};

/// A message of the given type with every field zero, edited by `$body`: an empty payload
/// decodes as all zeros, which is MAVLink 2's truncation rule.
macro_rules! message {
    ($ty:ident, |$m:ident| $body:expr) => {{
        let Some(MavMessage::$ty(mut $m)) = MavMessage::decode(<$ty as Message>::ID, &[]) else {
            panic!(concat!("no ", stringify!($ty)))
        };
        $body;
        MavMessage::$ty($m)
    }};
}

fn degrees(radians: mp_units::Radians) -> f64 {
    radians.0.to_degrees()
}

// --- SYS_STATUS ---------------------------------------------------------------------------------

#[test]
fn sys_status_fills_load_errors_and_the_battery_through_the_csharp_setters() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2949-2964, setters at 1343-1352 and 1396-1415
    let terrain = MavSysStatusSensor::MAV_SYS_STATUS_TERRAIN.0;
    let mut state = VehicleState::default();
    assert!(state.apply(&message!(SysStatus, |m| {
        m.load = 523;
        m.voltage_battery = 12_600;
        m.current_battery = -1;
        m.battery_remaining = -1;
        m.drop_rate_comm = 7;
        m.errors_count1 = 1;
        m.errors_count2 = 2;
        m.errors_count3 = 3;
        m.errors_count4 = 4;
        m.onboard_control_sensors_present = terrain;
        m.onboard_control_sensors_enabled = terrain;
        m.onboard_control_sensors_health = terrain;
    })));
    assert_eq!(state.load, 52.3, "load / 10.0f");
    assert_eq!(state.packet_drop_remote, 7);
    assert_eq!(state.errors_count, [1, 2, 3, 4]);
    assert_eq!(
        state.battery.current, 0.0,
        "-1 is 'no sensor', which the setter zeroes"
    );
    assert_eq!(
        state.battery.remaining_percent, 0,
        "-1 is outside 0-100, so 0"
    );
    assert_eq!(
        state.battery.voltage, 0.0,
        "the C# never takes the voltage from SYS_STATUS"
    );
    assert!(state.sensors.terrain_active());
    assert!(
        state.sensors.safety_active(),
        "motor outputs are not enabled"
    );

    state.apply(&message!(SysStatus, |m| {
        m.current_battery = -250;
        m.battery_remaining = 55;
    }));
    assert_eq!(
        state.battery.current, -2.5,
        "a regenerating current is kept"
    );
    assert_eq!(state.battery.remaining_percent, 55);
    state.apply(&message!(SysStatus, |m| m.battery_remaining = 101));
    assert_eq!(state.battery.remaining_percent, 0, "over 100 is 0 too");
}

// --- GLOBAL_POSITION_INT and GPS_RAW_INT ------------------------------------------------------

fn global_position(lat: i32, lon: i32, alt: i32, relative_alt: i32, v: [i16; 3]) -> MavMessage {
    message!(GlobalPositionInt, |m| {
        m.lat = lat;
        m.lon = lon;
        m.alt = alt;
        m.relative_alt = relative_alt;
        m.vx = v[0];
        m.vy = v[1];
        m.vz = v[2];
        m.hdg = u16::MAX;
    })
}

fn gps_raw(lat: i32, lon: i32, alt: i32) -> MavMessage {
    message!(GpsRawInt, |m| {
        m.lat = lat;
        m.lon = lon;
        m.alt = alt;
        m.fix_type = 3;
        m.eph = u16::MAX;
        m.epv = u16::MAX;
        m.vel = u16::MAX;
        m.cog = u16::MAX;
        m.satellites_visible = u8::MAX;
    })
}

fn at(state: &VehicleState) -> (f64, f64) {
    let p = state.position.expect("a position");
    (p.latitude(), p.longitude())
}

#[test]
fn the_position_comes_from_global_position_int_while_it_is_valid_and_gps_otherwise() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3268-3284, 3294-3302
    let mut state = VehicleState::default();
    state.apply(&global_position(
        -353_632_620,
        1_491_652_370,
        584_000,
        10_000,
        [150, -250, -50],
    ));
    assert_eq!(at(&state), (-35.363_262, 149.165_237));
    assert_eq!(state.altitude_msl.0, 584.0);
    assert_eq!(state.altitude_relative.0, 10.0);
    assert_eq!(
        (
            state.velocity_north,
            state.velocity_east,
            state.velocity_down
        ),
        (150.0 * 0.01, -250.0 * 0.01, -50.0 * 0.01),
        "cm/s * 0.01"
    );

    // With a good global position, GPS_RAW_INT does not move it.
    state.apply(&gps_raw(-353_600_000, 1_491_600_000, 600_000));
    assert_eq!(at(&state), (-35.363_262, 149.165_237));
    assert_eq!(state.altitude_msl.0, 584.0);

    // A zero coordinate is "no position": the last one stays, and nothing but the relative
    // altitude is taken from it.
    state.apply(&global_position(
        0,
        1_491_652_370,
        1_000,
        20_000,
        [999, 999, 999],
    ));
    assert_eq!(at(&state), (-35.363_262, 149.165_237));
    assert_eq!(state.altitude_relative.0, 20.0);
    assert_eq!(state.altitude_msl.0, 584.0);
    assert_eq!(state.velocity_north, 1.5);

    // Now GPS_RAW_INT supplies it, with the C#'s `* 1.0e-7` rather than `/ 1e7`.
    state.apply(&gps_raw(-353_600_000, 1_491_600_000, 600_000));
    assert_eq!(
        at(&state),
        (
            f64::from(-353_600_000) * 1.0e-7,
            f64::from(1_491_600_000) * 1.0e-7
        )
    );
    assert_eq!(state.altitude_msl.0, 600.0);

    // An INT32_MAX coordinate keeps that coordinate and takes the other.
    state.apply(&gps_raw(i32::MAX, 1_491_700_000, 600_000));
    assert_eq!(
        at(&state),
        (
            f64::from(-353_600_000) * 1.0e-7,
            f64::from(1_491_700_000) * 1.0e-7
        )
    );

    // INT32_MAX in GLOBAL_POSITION_INT is as invalid as zero.
    state.apply(&global_position(i32::MAX, 1_491_652_370, 0, 0, [0; 3]));
    assert_eq!(
        at(&state).1,
        f64::from(1_491_700_000) * 1.0e-7,
        "still the GPS position"
    );
}

/// Without a 2D fix GPS_RAW_INT's speed and course are not taken: ArduPilot sends `vel` 0 then
/// while VFR_HUD carries the EKF's speed, and the ground speed flickered to 0 between the two
/// (upstream's 694628ac8). With a fix both are taken again.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3314-3321`
#[test]
fn gps_raw_int_without_a_fix_leaves_the_ground_speed_alone() {
    let mut state = VehicleState::default();
    state.apply(&message!(VfrHud, |m| {
        m.groundspeed = 12.0;
    }));
    let raw = |fix_type: u8| {
        message!(GpsRawInt, |m| {
            m.fix_type = fix_type;
            m.eph = u16::MAX;
            m.epv = u16::MAX;
            m.vel = 0;
            m.cog = 4500;
            m.satellites_visible = 0;
        })
    };
    for no_fix in [0, 1] {
        state.apply(&raw(no_fix));
        assert_eq!(state.ground_speed.0, 12.0, "fix_type {no_fix}");
    }
    state.apply(&raw(2));
    assert_eq!(state.ground_speed.0, 0.0, "a 2D fix's vel is taken");
}

#[test]
fn gps_raw_int_keeps_its_last_value_where_the_wire_says_unknown() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3305-3329
    let mut state = VehicleState::default();
    state.apply(&message!(GpsRawInt, |m| {
        m.lat = -353_632_620;
        m.lon = 1_491_652_370;
        m.fix_type = 6;
        m.eph = 121;
        m.epv = u16::MAX;
        m.satellites_visible = 17;
        m.vel = 1234;
        m.cog = 9000;
        m.h_acc = 1500;
        m.v_acc = 2500;
        m.vel_acc = 300;
        m.hdg_acc = 250_000;
        m.yaw = 4500;
    }));
    assert_eq!(state.gps.fix_type, 6);
    assert_eq!(state.gps.hdop, 1.21, "Math.Round(eph / 100.0, 2)");
    assert_eq!(state.gps.satellites_visible, 17);
    assert_eq!(
        state.ground_speed.0,
        f64::from(1234.0_f32 * 1.0e-2),
        "vel * 1.0e-2f"
    );
    assert_eq!(
        state.gps.course, 90.0,
        "cog / 100, moving faster than 0.5 m/s"
    );
    assert_eq!(state.gps.h_acc, 1.5);
    assert_eq!(state.gps.v_acc, 2.5);
    assert_eq!(state.gps.vel_acc, 0.3);
    assert_eq!(state.gps.hdg_acc, 2.5, "hdg_acc / 1e5");
    assert_eq!(state.gps.yaw, 45.0);
    assert!(
        state.position.is_some(),
        "no global position yet, so GPS supplies one"
    );

    // Unknown hdop, satellites and speed keep the last values; below 0.5 m/s the course holds.
    state.apply(&message!(GpsRawInt, |m| {
        m.fix_type = 3;
        m.eph = u16::MAX;
        m.satellites_visible = u8::MAX;
        m.vel = 40;
        m.cog = 18_000;
    }));
    assert_eq!(state.gps.hdop, 1.21);
    assert_eq!(state.gps.satellites_visible, 17);
    assert_eq!(state.ground_speed.0, f64::from(40.0_f32 * 1.0e-2));
    assert_eq!(
        state.gps.course, 90.0,
        "too slow for the course to mean anything"
    );
    state.apply(&message!(GpsRawInt, |m| {
        m.vel = u16::MAX;
        m.cog = 18_000;
    }));
    assert_eq!(state.ground_speed.0, f64::from(40.0_f32 * 1.0e-2));

    // GPS_STATUS writes the satellite count unconditionally. C#: CurrentState.cs:3384
    state.apply(&message!(GpsStatus, |m| m.satellites_visible = 9));
    assert_eq!(state.gps.satellites_visible, 9);
}

#[test]
fn gps2_raw_takes_every_field_as_it_arrives() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3349-3367
    let mut state = VehicleState::default();
    state.apply(&message!(Gps2Raw, |m| {
        m.lat = -353_000_000;
        m.lon = 1_490_000_000;
        m.alt = 12_345;
        m.fix_type = 5;
        m.eph = u16::MAX;
        m.satellites_visible = u8::MAX;
        m.vel = 250;
        m.cog = 27_000;
        m.h_acc = 20;
        m.yaw = 100;
    }));
    let position = state.gps2.position.expect("a second position");
    assert_eq!(position.latitude(), f64::from(-353_000_000) * 1.0e-7);
    assert_eq!(state.gps2.altitude_msl, 12.345, "alt / 1000.0f");
    assert_eq!(state.gps2.fix_type, 5);
    assert_eq!(
        state.gps2.hdop, 655.35,
        "no unknown check on the second receiver"
    );
    assert_eq!(state.gps2.satellites_visible, 255);
    assert_eq!(state.gps2.ground_speed, 2.5);
    assert_eq!(state.gps2.course, 270.0);
    assert_eq!(state.gps2.h_acc, 0.02);
    assert_eq!(state.gps2.yaw, 1.0);
    assert!(
        state.position.is_none(),
        "the second receiver is not the vehicle's position"
    );

    state.apply(&message!(Gps2Raw, |m| m.fix_type = 1));
    assert!(state.gps2.position.is_none(), "0, 0 is no position");
}

// --- VFR_HUD ------------------------------------------------------------------------------------

#[test]
fn vfr_hud_negates_the_throttle_while_the_motors_are_reversed() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3849-3858
    let hud = message!(VfrHud, |m| {
        m.airspeed = 18.5;
        m.groundspeed = 17.25;
        m.climb = -1.5;
        m.throttle = 40;
    });
    let mut state = VehicleState::default();
    state.apply(&hud);
    assert_eq!(state.air_speed.0, 18.5);
    assert_eq!(state.ground_speed.0, 17.25);
    assert_eq!(state.climb_rate.0, -1.5);
    assert_eq!(state.throttle_percent, 40);

    let reversed = MavSysStatusSensor::MAV_SYS_STATUS_REVERSE_MOTOR.0;
    state.apply(&message!(SysStatus, |m| {
        m.onboard_control_sensors_present = reversed;
        m.onboard_control_sensors_enabled = reversed;
        m.onboard_control_sensors_health = reversed;
    }));
    state.apply(&hud);
    assert_eq!(state.throttle_percent, -40);

    state.apply(&message!(SysStatus, |m| {
        m.onboard_control_sensors_present = reversed;
        m.onboard_control_sensors_enabled = reversed;
    }));
    state.apply(&hud);
    assert_eq!(
        state.throttle_percent, 40,
        "only when the bit is healthy too"
    );
}

// --- BATTERY_STATUS and BATTERY2 ----------------------------------------------------------------

fn battery(id: u8, cells: &[u16], edit: impl FnOnce(&mut BatteryStatus)) -> MavMessage {
    message!(BatteryStatus, |m| {
        m.id = id;
        m.voltages = [u16::MAX; 10];
        for (slot, cell) in m.voltages.iter_mut().zip(cells) {
            *slot = *cell;
        }
        m.temperature = i16::MAX;
        edit(&mut m);
    })
}

/// The C#'s `value * 0.4f + previous * 0.6f`, with the single-precision literals widened.
fn filtered(value: f64, previous: f64) -> f64 {
    let previous = if previous == 0.0 { value } else { previous };
    value * f64::from(0.4_f32) + previous * f64::from(0.6_f32)
}

#[test]
fn the_first_battery_sums_its_cells_and_filters_the_voltage() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3085-3126, the voltage setter at 1300-1301
    let mut state = VehicleState::default();
    state.apply(&battery(0, &[3700, 3710, 3720], |m| {
        m.current_battery = 1234;
        m.current_consumed = 1500;
        m.battery_remaining = 80;
        m.temperature = 2512;
        m.time_remaining = 600;
    }));
    // The cells summed in order, UINT16_MAX counting as nothing.
    let first = filtered(3.7 + 3.71 + 3.72, 0.0);
    assert_eq!(state.battery.voltage, first as f32);
    assert_eq!(state.battery.cells[..4], [3.7, 3.71, 3.72, 0.0]);
    assert_eq!(state.battery.current, 12.34);
    assert_eq!(state.battery.consumed_mah, 1500);
    assert_eq!(state.battery.remaining_percent, 80);
    assert_eq!(state.battery.temperature, 25.12);
    assert_eq!(
        state.battery.remaining_minutes, 10.0,
        "time_remaining / 60.0f"
    );

    // The second reading moves four tenths of the way. An unknown temperature keeps the last,
    // and a missing first cell leaves the cells alone.
    state.apply(&battery(0, &[u16::MAX], |m| {
        m.voltages_ext = [4000, 0, 0, 0];
        m.current_battery = -1;
        m.battery_remaining = -1;
    }));
    let second = filtered(4.0, first);
    assert_eq!(state.battery.voltage, second as f32);
    assert_eq!(state.battery.cells[..3], [3.7, 3.71, 3.72]);
    assert_eq!(state.battery.temperature, 25.12);
    assert_eq!(
        state.battery.current, -0.01,
        "written past the setter, so -1 is -0.01 A here"
    );
    assert_eq!(state.battery.remaining_percent, 0, "the setter clamps");
}

#[test]
fn the_other_batteries_follow_their_ids_with_their_own_rules() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3128-3207, the second battery's setter at 1562-1563
    let mut state = VehicleState::default();
    for _ in 0..2 {
        state.apply(&battery(1, &[4200, 4100], |m| m.battery_remaining = -1));
    }
    let total = 4.2 + 4.1;
    let second = &state.batteries[0];
    assert_eq!(
        second.voltage,
        filtered(total, filtered(total, 0.0)) as f32,
        "the second battery is filtered"
    );
    assert_eq!(second.remaining_percent, -1, "and not clamped");
    assert_eq!(
        second.cells, [0.0; 14],
        "cells are the first battery's only"
    );

    state.apply(&battery(2, &[4200, 4100], |m| m.current_battery = 250));
    assert_eq!(
        state.batteries[1].voltage, total as f32,
        "the third is plain"
    );
    assert_eq!(state.batteries[1].current, 2.5);
    state.apply(&battery(8, &[1000], |_| {}));
    assert_eq!(state.batteries[7].voltage, 1.0, "id 8 is the ninth battery");

    let before = state;
    assert!(
        !state.apply(&battery(9, &[1000], |_| {})),
        "there is no tenth"
    );
    assert_eq!(state.batteries, before.batteries);
    assert_eq!(state.battery, before.battery);

    // BATTERY2 sets the second battery's current, ignoring a negative one.
    // C#: CurrentState.cs:3073, setter at 1418-1430
    assert!(state.apply(&message!(Battery2, |m| m.current_battery = 500)));
    assert_eq!(state.batteries[0].current, 5.0);
    assert!(!state.apply(&message!(Battery2, |m| m.current_battery = -1)));
    assert_eq!(state.batteries[0].current, 5.0);
}

// --- RADIO / RADIO_STATUS -----------------------------------------------------------------------

#[test]
fn a_sik_radio_report_reaches_every_vehicle_on_the_link() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2280-2282, 3392-3412
    let mut registry = VehicleRegistry::new();
    let autopilot = VehicleId::new(1, 1);
    registry.apply(1, 1, 0, &message!(Heartbeat, |m| m.r#type = 2));
    registry.apply(
        51,
        68,
        7,
        &message!(RadioStatus, |m| {
            m.rssi = 180;
            m.remrssi = 170;
            m.txbuf = 95;
            m.noise = 40;
            m.remnoise = 45;
            m.rxerrors = 3;
            m.fixed = 2;
        }),
    );
    let seen = registry.working(autopilot).expect("the autopilot").radio;
    assert_eq!(
        (
            seen.rssi,
            seen.remrssi,
            seen.txbuf,
            seen.noise,
            seen.remnoise,
            seen.rxerrors,
            seen.fixed
        ),
        (180, 170, 95, 40, 45, 3, 2)
    );
    assert_eq!(
        registry
            .working(autopilot)
            .expect("the autopilot")
            .link
            .received,
        1,
        "the radio's sequence numbers are its own"
    );
    let radio = VehicleId::new(51, 68);
    assert_eq!(registry.working(radio).expect("the radio").radio.rssi, 180);

    // RADIO, the older message, does the same.
    registry.apply(51, 68, 8, &message!(Radio, |m| m.rssi = 99));
    assert_eq!(
        registry
            .working(autopilot)
            .expect("the autopilot")
            .radio
            .rssi,
        99
    );

    // Anything else from the radio stays with the radio.
    registry.apply(51, 68, 9, &message!(Heartbeat, |m| m.r#type = 6));
    assert_eq!(
        registry
            .working(autopilot)
            .expect("the autopilot")
            .vehicle_type,
        2
    );
}

// --- WIND, TERRAIN_REPORT -----------------------------------------------------------------------

#[test]
fn wind_is_normalised_to_a_compass_bearing_and_kept_in_metres_per_second() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2857-2858. The C# multiplies the speed by the
    // display unit here; the state keeps m/s like every other speed.
    let mut state = VehicleState::default();
    state.apply(&message!(Wind, |m| {
        m.direction = -90.0;
        m.speed = 7.5;
    }));
    assert_eq!(state.wind_direction, 270.0);
    assert_eq!(state.wind_speed, 7.5);
    state.apply(&message!(Wind, |m| m.direction = 450.0));
    assert_eq!(state.wind_direction, 90.0);
}

#[test]
fn the_terrain_report_is_held_as_sent() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3237-3241
    let mut state = VehicleState::default();
    state.apply(&message!(TerrainReport, |m| {
        m.current_height = 42.5;
        m.terrain_height = 583.25;
        m.loaded = 504;
        m.pending = 12;
        m.spacing = 100;
    }));
    let terrain = state.terrain;
    assert_eq!(
        (
            terrain.current_height,
            terrain.terrain_height,
            terrain.loaded,
            terrain.pending,
            terrain.spacing
        ),
        (42.5, 583.25, 504, 12, 100)
    );
}

// --- RANGEFINDER and DISTANCE_SENSOR ------------------------------------------------------------

fn distance(id: u8, facing_down: bool, centimetres: u16) -> MavMessage {
    message!(DistanceSensor, |m| {
        m.id = id;
        m.orientation = if facing_down { 25 } else { 0 };
        m.current_distance = centimetres;
    })
}

#[test]
fn the_first_downward_distance_sensor_is_the_altitude_in_whole_metres() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2762-2790
    let mut state = VehicleState::default();
    state.apply(&distance(1, true, 250));
    assert_eq!(state.rangefinder.distances[1], 250);
    assert_eq!(
        state.rangefinder.range, 2.0,
        "250 / 100 in integer arithmetic"
    );

    // A forward sensor is recorded but is not the altitude.
    state.apply(&distance(0, false, 1000));
    assert_eq!(state.rangefinder.distances[0], 1000);
    assert_eq!(state.rangefinder.range, 2.0);

    // A lower id facing down takes over, and the higher one is then only recorded.
    state.apply(&distance(0, true, 399));
    assert_eq!(state.rangefinder.range, 3.0);
    state.apply(&distance(1, true, 900));
    assert_eq!(state.rangefinder.distances[1], 900);
    assert_eq!(state.rangefinder.range, 3.0);

    // The altitude sensor turning away gives the job back to whoever faces down next.
    state.apply(&distance(0, false, 100));
    assert_eq!(state.rangefinder.range, 3.0);
    state.apply(&distance(1, true, 700));
    assert_eq!(state.rangefinder.range, 7.0);

    // Ids past ten are not recorded, and do not panic.
    state.apply(&distance(12, false, 5));
    assert_eq!(state.rangefinder.distances.len(), 10);
}

#[test]
fn a_rangefinder_message_outranks_every_distance_sensor_for_good() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2749-2754
    let mut state = VehicleState::default();
    state.apply(&message!(Rangefinder, |m| {
        m.distance = 1.234;
        m.voltage = 0.5;
    }));
    assert_eq!(state.rangefinder.range, 1.234);
    assert_eq!(state.rangefinder.voltage, 0.5);
    state.apply(&distance(0, true, 500));
    assert_eq!(state.rangefinder.distances[0], 500);
    assert_eq!(state.rangefinder.range, 1.234);
}

// --- SERVO_OUTPUT_RAW, MISSION_CURRENT, HOME_POSITION -------------------------------------------

#[test]
fn servo_outputs_fill_by_port() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3642-3679
    let mut state = VehicleState::default();
    assert!(state.apply(&message!(ServoOutputRaw, |m| {
        m.servo1_raw = 1100;
        m.servo8_raw = 1800;
        m.servo16_raw = 1600;
    })));
    assert_eq!(state.servo_outputs[0], 1100);
    assert_eq!(state.servo_outputs[7], 1800);
    assert_eq!(state.servo_outputs[15], 1600);
    assert!(state.apply(&message!(ServoOutputRaw, |m| {
        m.port = 1;
        m.servo1_raw = 2001;
        m.servo16_raw = 2016;
    })));
    assert_eq!(state.servo_outputs[16], 2001);
    assert_eq!(state.servo_outputs[31], 2016);
    assert_eq!(state.servo_outputs[0], 1100, "port 1 leaves port 0 alone");
    assert!(!state.apply(&message!(ServoOutputRaw, |m| {
        m.port = 2;
        m.servo1_raw = 1;
    })));
    assert_eq!(state.servo_outputs[0], 1100);
}

#[test]
fn the_last_auto_waypoint_is_remembered_for_resume_mission() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3423-3425
    let mut state = VehicleState::default();
    let current = |seq| message!(MissionCurrent, |m| m.seq = seq);
    state.apply(&message!(Heartbeat, |m| {
        m.r#type = 2;
        m.custom_mode = 3; // Auto, on a copter
    }));
    state.apply(&current(0));
    assert_eq!(
        state.last_auto_wp, None,
        "0 is not a waypoint to resume from"
    );
    state.apply(&current(5));
    assert_eq!(state.last_auto_wp, Some(5));
    state.apply(&message!(Heartbeat, |m| {
        m.r#type = 2;
        m.custom_mode = 6; // RTL
    }));
    state.apply(&current(7));
    assert_eq!(state.mission_current, 7);
    assert_eq!(state.last_auto_wp, Some(5), "only waypoints flown in Auto");
}

#[test]
fn home_carries_its_altitude() {
    // C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5676-5680
    let mut state = VehicleState::default();
    state.apply(&message!(HomePosition, |m| {
        m.latitude = -353_632_620;
        m.longitude = 1_491_652_370;
        m.altitude = 584_123;
    }));
    let home = state.home.expect("a home");
    assert_eq!(
        (home.latitude(), home.longitude()),
        (-35.363_262, 149.165_237)
    );
    assert_eq!(state.home_altitude.0, 584.123);
    state.apply(&message!(HomePosition, |m| m.altitude = 1));
    assert!(state.home.is_none(), "0, 0 is no home");
}

// --- EKF ----------------------------------------------------------------------------------------

#[test]
fn the_ekf_status_is_the_worst_variance_unless_a_flag_forces_red() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2665-2740
    const HEALTHY: u16 = 1 | 2 | 4 | 8 | 16 | 32;
    let report = |flags: u16, compass: f32| {
        message!(EkfStatusReport, |m| {
            m.velocity_variance = 0.1;
            m.pos_horiz_variance = 0.2;
            m.pos_vert_variance = 0.3;
            m.compass_variance = compass;
            m.terrain_alt_variance = 0.45;
            m.flags = flags;
        })
    };
    let mut state = VehicleState::default();
    assert_eq!(state.ekf_status(), 0.0, "nothing reported yet");

    state.apply(&report(HEALTHY, 0.4));
    assert_eq!(
        state.ekf_status(),
        0.45,
        "terrain counts here, unlike the verdict"
    );
    state.apply(&report(HEALTHY, 0.7));
    assert_eq!(state.ekf_status(), 0.7);

    state.apply(&report(HEALTHY & !1, 0.1));
    assert_eq!(state.ekf_status(), 1.0, "no attitude is red");

    state.apply(&report(HEALTHY & !2, 0.1));
    assert_eq!(
        state.ekf_status(),
        0.45,
        "no horizontal velocity without a GPS is fine"
    );
    state.apply(&message!(GpsRawInt, |m| m.fix_type = 1));
    assert_eq!(state.ekf_status(), 1.0, "and red with one");

    state.apply(&report(HEALTHY | 1024, 0.1));
    assert_eq!(state.ekf_status(), 1.0, "uninitialised is red");

    state.apply(&report(HEALTHY, f32::NAN));
    assert!(state.ekf_status().is_nan(), "Math.Max propagates NaN");
}

// --- AIRSPEED, SCALED_PRESSURE ------------------------------------------------------------------

#[test]
fn the_first_airspeed_sensor_sets_the_airspeed_and_its_temperature_in_degrees() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:4212-4216
    let mut state = VehicleState::default();
    assert!(state.apply(&message!(Airspeed, |m| {
        m.airspeed = 21.5;
        m.temperature = 2345;
    })));
    assert_eq!(state.air_speed.0, 21.5);
    assert_eq!(state.airspeed1_temp, 23.45);
    assert!(!state.apply(&message!(Airspeed, |m| {
        m.id = 1;
        m.airspeed = 99.0;
    })));
    assert_eq!(state.air_speed.0, 21.5, "only the first sensor");
}

#[test]
fn scaled_pressure_is_held_in_the_wires_centidegrees() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3215-3218, 3226-3229
    let mut state = VehicleState::default();
    state.apply(&message!(ScaledPressure, |m| {
        m.press_abs = 1013.25;
        m.temperature = 2512;
        m.temperature_press_diff = 2600;
    }));
    assert_eq!(state.press_abs, 1013.25);
    assert_eq!(state.press_temp, 2512);
    assert_eq!(
        state.airspeed1_temp, 2600.0,
        "unconverted, unlike AIRSPEED's"
    );
    state.apply(&message!(ScaledPressure2, |m| {
        m.press_abs = 990.5;
        m.temperature = -150;
        m.temperature_press_diff = 1800;
    }));
    assert_eq!(state.press_abs2, 990.5);
    assert_eq!(state.press_temp2, -150);
    assert_eq!(state.airspeed2_temp, 1800.0);
}

// --- HIGH_LATENCY and HIGH_LATENCY2 -------------------------------------------------------------

fn with_home_at(altitude_mm: i32) -> VehicleState {
    let mut state = VehicleState::default();
    state.apply(&message!(HomePosition, |m| {
        m.latitude = -353_632_620;
        m.longitude = 1_491_652_370;
        m.altitude = altitude_mm;
    }));
    state
}

#[test]
fn high_latency_fills_the_summary_fields_in_their_units() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2425-2470
    let mut state = with_home_at(500_000);
    state.apply(&message!(HighLatency, |m| {
        m.base_mode = 1;
        m.custom_mode = 10;
        m.roll = 1250;
        m.pitch = -300;
        m.heading = 27_000;
        m.throttle = 35;
        m.latitude = -353_000_000;
        m.longitude = 1_490_000_000;
        m.altitude_amsl = 620;
        // Relative to home, per common.xml.
        m.altitude_sp = 150;
        m.airspeed = 22;
        m.groundspeed = 20;
        m.climb_rate = -3;
        m.gps_nsat = 12;
        m.gps_fix_type = 3;
        m.battery_remaining = 250;
        m.temperature = 21;
        m.temperature_air = 19;
        m.wp_num = 4;
        m.wp_distance = 1500;
    }));
    assert_eq!(state.custom_mode, 10);
    assert!((degrees(state.attitude.roll) - 12.5).abs() < 1e-9);
    assert!((degrees(state.attitude.pitch) + 3.0).abs() < 1e-9);
    assert!((degrees(state.attitude.yaw) - 270.0).abs() < 1e-9);
    assert_eq!(state.throttle_percent, 35);
    assert_eq!(state.position.expect("a position").latitude(), -35.3);
    assert_eq!(state.altitude_msl.0, 620.0);
    assert_eq!(state.altitude_relative.0, 120.0, "above the home altitude");
    assert_eq!(state.nav.alt_error, 30.0);
    assert_eq!(state.air_speed.0, 22.0);
    assert_eq!(state.ground_speed.0, 20.0);
    assert_eq!(state.climb_rate.0, -3.0);
    assert_eq!(state.gps.satellites_visible, 12);
    assert_eq!(state.gps.fix_type, 3);
    assert_eq!(state.battery.remaining_percent, 0, "250 is outside 0-100");
    assert_eq!(state.press_temp, 21, "whole degrees, as the C# writes them");
    assert_eq!(state.airspeed1_temp, 19.0);
    assert_eq!(state.mission_current, 4);
    assert_eq!(state.nav.wp_distance, 1500.0);

    // Without the custom-mode flag the mode is left alone.
    state.apply(&message!(HighLatency, |m| m.custom_mode = 3));
    assert_eq!(state.custom_mode, 10);
}

#[test]
fn high_latency2_scales_its_compressed_fields() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2478-2546
    let mut state = with_home_at(500_000);
    state.apply(&message!(HighLatency2, |m| {
        m.custom_mode = 6;
        m.latitude = -353_000_000;
        m.longitude = 1_490_000_000;
        m.altitude = 700;
        m.target_altitude = 720;
        m.target_distance = 15;
        m.wp_num = 9;
        m.heading = 45;
        m.target_heading = 90;
        m.throttle = 55;
        m.airspeed = 110;
        m.groundspeed = 100;
        m.windspeed = 25;
        m.wind_heading = 135;
        m.eph = 12;
        m.temperature_air = -5;
        m.climb_rate = -3;
        m.battery = 80;
    }));
    assert_eq!(state.custom_mode, 6);
    assert_eq!(state.altitude_msl.0, 700.0);
    assert_eq!(state.altitude_relative.0, 200.0);
    // The C# subtracts the altitude above home from this message's sea-level setpoint: 720 - 200.
    // Kept, since it is the number Mission Planner shows.
    assert_eq!(state.nav.alt_error, 520.0);
    assert_eq!(state.nav.wp_distance, 150.0, "decametres");
    assert_eq!(state.mission_current, 9);
    assert!(
        (degrees(state.attitude.yaw) - 90.0).abs() < 1e-9,
        "two-degree steps"
    );
    assert_eq!(state.nav.target_bearing, 180.0);
    assert_eq!(state.throttle_percent, 55);
    assert_eq!(state.air_speed.0, 22.0, "fifths of a metre per second");
    assert_eq!(state.ground_speed.0, 20.0);
    assert_eq!(state.wind_speed, 5.0);
    assert_eq!(state.wind_direction, 270.0);
    assert_eq!(state.gps.hdop, 12.0);
    assert_eq!(state.airspeed1_temp, -5.0);
    assert_eq!(state.climb_rate.0, -30.0, "climb_rate * 10");
    assert_eq!(state.battery.remaining_percent, 80);
}

/// Every `HEARTBEAT` and `HIGH_LATENCY2` is counted, and nothing else is: the two messages
/// `getHeartBeat` returns on, so the firmware page's `doReboot(true, false)` can wait for the
/// next heartbeat as it does.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1168-1205, 2588-2600`
#[test]
fn heartbeats_and_high_latency_reports_are_counted_as_getheartbeat_takes_them() {
    let mut state = VehicleState::default();
    assert_eq!(state.heartbeats, 0);
    state.apply(&message!(Heartbeat, |m| m.r#type = 2));
    assert_eq!(state.heartbeats, 1);
    state.apply(&message!(SysStatus, |m| m.load = 1));
    state.apply(&message!(HighLatency, |m| m.heading = 1));
    assert_eq!(state.heartbeats, 1);
    state.apply(&message!(HighLatency2, |m| m.r#type = 2));
    assert_eq!(state.heartbeats, 2);
    state.apply(&message!(Heartbeat, |m| m.r#type = 2));
    assert_eq!(state.heartbeats, 3);
}
