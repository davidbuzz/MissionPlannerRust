//! What the navigation controller reports, and the HUD figures derived from it.
//!
//! `NAV_CONTROLLER_OUTPUT` is where the HUD's target bugs and cross-track bar come from, and
//! `MISSION_CURRENT` is the waypoint number beside the distance. Neither was applied to the
//! state before the HUD needed them; these pin what the fields mean and the two derivations the
//! C# makes from them.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_possible_truncation
)]

use mp_mavlink_dialects::all::{MavMessage, MissionCurrent, NavControllerOutput, VfrHud};
use mp_units::Radians;
use mp_vehicle::VehicleState;

fn nav(bearing: i16, wp_dist: u16, xtrack: f32, alt_error: f32, aspd_error: f32) -> MavMessage {
    MavMessage::NavControllerOutput(NavControllerOutput {
        nav_roll: 12.5,
        nav_pitch: -3.0,
        alt_error,
        aspd_error,
        xtrack_error: xtrack,
        nav_bearing: bearing,
        target_bearing: bearing + 5,
        wp_dist,
    })
}

#[test]
fn the_controller_output_lands_in_the_state_in_the_wires_units() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3448-3455
    let mut state = VehicleState::default();
    assert!(state.apply(&nav(273, 1_250, -4.5, 12.0, 2.0)));
    assert_eq!(state.nav.bearing, 273.0);
    assert_eq!(state.nav.target_bearing, 278.0);
    assert_eq!(state.nav.wp_distance, 1_250.0);
    assert_eq!(state.nav.xtrack_error, -4.5);
    assert_eq!(state.nav.alt_error, 12.0);
    assert_eq!(state.nav.airspeed_error, 2.0);
    assert_eq!(state.nav.roll, 12.5);
    assert_eq!(state.nav.pitch, -3.0);
}

#[test]
fn the_current_mission_item_is_the_waypoint_number() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3420
    let mut state = VehicleState::default();
    assert!(state.apply(&MavMessage::MissionCurrent(MissionCurrent {
        seq: 7,
        total: 12,
        mission_state: 0,
        mission_mode: 0,
    })));
    assert_eq!(state.mission_current, 7);
}

#[test]
fn turn_rate_is_the_coordinated_turn_estimate_not_the_gyro() {
    // roll * g / groundspeed, in degrees per second. C#: CurrentState.cs:1203-1210
    let mut state = VehicleState::default();
    state.attitude.roll = Radians(30.0_f64.to_radians());
    state.attitude.yaw_rate = 99.0; // the gyro says something else entirely
    state.ground_speed = mp_units::MetresPerSecond(20.0);
    let expected = 30.0 * 9.806_65 / 20.0;
    assert!(
        (state.turn_rate() - expected as f32).abs() < 1e-4,
        "{}",
        state.turn_rate()
    );
}

#[test]
fn turn_rate_is_zero_at_walking_pace() {
    // Below 1 m/s the division would report a stationary aircraft turning at hundreds of degrees
    // a second because it is sitting on a slope.
    let mut state = VehicleState::default();
    state.attitude.roll = Radians(30.0_f64.to_radians());
    state.ground_speed = mp_units::MetresPerSecond(0.9);
    assert_eq!(state.turn_rate(), 0.0);
}

#[test]
fn the_targets_are_current_plus_error() {
    let mut state = VehicleState::default();
    assert!(state.apply(&MavMessage::VfrHud(VfrHud {
        airspeed: 18.0,
        groundspeed: 17.0,
        alt: 120.0,
        climb: 0.5,
        heading: 90,
        throttle: 40,
    })));
    state.altitude_relative = mp_units::Metres(100.0);
    assert!(state.apply(&nav(90, 300, 0.0, 25.0, 4.0)));
    assert!((state.target_altitude() - 125.0).abs() < 1e-9);
    // The C# would say 18.04 here, having divided the wire's 4 m/s by 100; see the divergence
    // note on `target_airspeed`.
    assert!((state.target_airspeed() - 22.0).abs() < 1e-9);
}
