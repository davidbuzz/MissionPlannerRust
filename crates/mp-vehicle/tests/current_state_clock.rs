//! The `CurrentState` fields that run on its clock, one message at a time.
//!
//! Each test feeds constructed messages through [`VehicleRegistry::apply_at`], stamped as a link
//! or a log stamps them, and [`VehicleRegistry::update_current_settings`], and checks the result
//! against the C#'s formula worked by hand, cited line by line - with the edges the C# has: the
//! first reading measured from `DateTime.MinValue`, the 0.2 s gate, a clock that goes backwards,
//! seconds counted by the clock's seconds field, the "no sensor" current, and the unguarded
//! divisions. `current_state_oracle.rs` holds the same fields to the C# run itself.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]

use mp_mavlink::Message;
use mp_mavlink_dialects::all::{
    BatteryStatus, GlobalPositionInt, GpsRawInt, Heartbeat, HighLatency2, HilControls, MavMessage,
    RawImu, RcChannelsScaled, SysStatus, VfrHud,
};
use mp_units::LatLon;
use mp_vehicle::{DateTime, FenceItem, LatLngAlt, VehicleId, VehicleRegistry, VehicleState};

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

const AUTOPILOT: VehicleId = VehicleId::new(1, 1);

/// 2026-01-01T00:00:00Z: a whole minute, so its seconds field is 0.
fn t0() -> DateTime {
    DateTime::from_tlog_micros(1_767_225_600_000_000).unwrap()
}

/// `t0` plus `seconds`, to the tick.
fn at(seconds: f64) -> DateTime {
    DateTime::from_ticks(t0().ticks() + (seconds * 1e7).round() as i64)
}

/// `TimeSpan.TotalSeconds` as the C# computes it, `ticks * (1.0 / 1e7)`.
fn total_seconds(ticks: i64) -> f64 {
    ticks as f64 * (1.0 / 10_000_000.0)
}

fn apply(registry: &mut VehicleRegistry, time: DateTime, message: &MavMessage) {
    registry.apply_at(1, 1, 0, message, time);
}

fn state(registry: &VehicleRegistry) -> &VehicleState {
    registry.working(AUTOPILOT).unwrap()
}

fn position(lat: f64, lng: f64, relative_alt_mm: i32) -> MavMessage {
    message!(GlobalPositionInt, |m| {
        m.lat = (lat * 1e7).round() as i32;
        m.lon = (lng * 1e7).round() as i32;
        m.relative_alt = relative_alt_mm;
    })
}

fn heartbeat(armed: bool) -> MavMessage {
    message!(Heartbeat, |m| {
        m.r#type = 1;
        m.autopilot = 3;
        m.base_mode = if armed { 128 | 1 } else { 1 };
    })
}

// --- datetime -----------------------------------------------------------------------------------

#[test]
fn apply_at_stamps_the_senders_clock_and_apply_leaves_it() {
    // C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4721, 6649
    let mut registry = VehicleRegistry::new();
    registry.apply_at(1, 1, 0, &heartbeat(false), at(1.5));
    registry.apply_at(2, 1, 0, &heartbeat(false), at(2.5));
    assert_eq!(state(&registry).datetime, at(1.5));
    assert_eq!(
        registry.working(VehicleId::new(2, 1)).unwrap().datetime,
        at(2.5),
        "each vehicle's clock is its own packets'"
    );
    registry.apply(1, 1, 1, &heartbeat(false));
    assert_eq!(state(&registry).datetime, at(1.5), "apply does not move it");
    // A vehicle only ever given `apply` keeps DateTime.MinValue, as the C#'s unstamped ones do.
    let mut plain = VehicleRegistry::new();
    plain.apply(1, 1, 0, &heartbeat(false));
    assert_eq!(state(&plain).datetime, DateTime::MIN);
}

// --- alt, verticalspeed, climbrate ------------------------------------------------------------

#[test]
fn the_vertical_speed_is_the_filtered_rate_of_the_alt_setter() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:328-346, 1043-1051
    let mut registry = VehicleRegistry::new();

    // The first altitude is measured from DateTime.MinValue: two thousand years.
    apply(&mut registry, t0(), &position(-35.0, 149.0, 1000));
    let first = 1.0_f32 / total_seconds(t0().ticks()) as f32;
    let mut expected = 0.0_f32 * 0.4 + first * 0.6;
    assert_eq!(state(&registry).alt(), 1.0);
    assert_eq!(state(&registry).vertical_speed(), expected);
    assert!(
        expected > 0.0 && expected < 1e-10,
        "next to nothing: {expected}"
    );
    // Before any VFR_HUD the climb rate is the same unfiltered rate.
    assert_eq!(state(&registry).climb_rate.0, f64::from(first));

    // Under 0.2 s later: the altitude moves, the rate does not.
    apply(&mut registry, at(0.1), &position(-35.0, 149.0, 2000));
    assert_eq!(state(&registry).alt(), 2.0);
    assert_eq!(state(&registry).vertical_speed(), expected);

    // 0.3 s after the last rate: (2.5 - 1.0) / 0.3.
    apply(&mut registry, at(0.3), &position(-35.0, 149.0, 2500));
    let rate = (2.5_f32 - 1.0) / total_seconds(3_000_000) as f32;
    expected = expected * 0.4 + rate * 0.6;
    assert_eq!(state(&registry).vertical_speed(), expected);
    assert_eq!(state(&registry).climb_rate.0, f64::from(rate));

    // A second later at the same altitude: `oldalt != alt` is false, nothing moves.
    apply(&mut registry, at(1.3), &position(-35.0, 149.0, 2500));
    assert_eq!(state(&registry).vertical_speed(), expected);

    // The clock goes back: `lastalt > datetime` lets it through whatever the altitude, and the
    // time between is negative.
    apply(&mut registry, at(-0.7), &position(-35.0, 149.0, 1500));
    let backwards = (1.5_f32 - 2.5) / total_seconds(-10_000_000) as f32;
    assert_eq!(backwards, 1.0, "down one metre over minus one second");
    expected = expected * 0.4 + backwards * 0.6;
    assert_eq!(state(&registry).vertical_speed(), expected);

    // Once VFR_HUD has given the climb rate, the setter leaves it.
    apply(
        &mut registry,
        at(0.0),
        &message!(VfrHud, |m| m.climb = -3.25),
    );
    apply(&mut registry, at(0.5), &position(-35.0, 149.0, 9000));
    assert_eq!(state(&registry).climb_rate.0, -3.25);
    assert_ne!(
        state(&registry).vertical_speed(),
        expected,
        "the rate still runs"
    );
}

#[test]
fn the_home_alt_offset_is_subtracted_before_the_rate() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:327, 378-383; GCSViews/FlightData.cs:1236-1247
    let mut registry = VehicleRegistry::new();
    apply(&mut registry, t0(), &position(-35.0, 149.0, 10_000));
    registry.working_mut(AUTOPILOT).unwrap().alt_offset_home = -584.0;
    let state_now = state(&registry);
    assert_eq!(
        state_now.alt(),
        594.0,
        "above sea level, as the button makes it"
    );
    assert_eq!(
        state_now.altitude_relative.0, 10.0,
        "the relative altitude is kept"
    );
    // The offset jumps the altitude the next rate is taken from: (594 - 10) / 1 s.
    let before = state_now.vertical_speed();
    apply(&mut registry, at(1.0), &position(-35.0, 149.0, 10_000));
    assert_eq!(
        state(&registry).vertical_speed(),
        before * 0.4 + 584.0 * 0.6
    );
}

#[test]
fn the_high_latency_altitude_goes_through_the_alt_setter() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2503-2505, `alt = altasl - (float)HomeAlt`
    let mut registry = VehicleRegistry::new();
    apply(
        &mut registry,
        t0(),
        &message!(HighLatency2, |m| m.altitude = 600),
    );
    apply(
        &mut registry,
        at(1.0),
        &message!(HighLatency2, |m| m.altitude = 603),
    );
    let s = state(&registry);
    assert_eq!(s.alt(), 603.0, "no home yet: above sea level");
    let first = 600.0_f32 / total_seconds(t0().ticks()) as f32;
    assert_eq!(s.vertical_speed(), (first * 0.6) * 0.4 + 3.0 * 0.6);
}

// --- UpdateCurrentSettings: time in air, distance -----------------------------------------------

/// Everything the once-a-second count reads, at `time`.
fn flying(
    registry: &mut VehicleRegistry,
    time: DateTime,
    armed: bool,
    fix: u8,
    lat: f64,
    lng: f64,
    throttle: u16,
) {
    apply(registry, time, &heartbeat(armed));
    apply(
        registry,
        time,
        &message!(GpsRawInt, |m| {
            m.fix_type = fix;
            m.lat = (lat * 1e7).round() as i32;
            m.lon = (lng * 1e7).round() as i32;
            m.eph = u16::MAX;
            m.satellites_visible = u8::MAX;
            m.vel = u16::MAX;
        }),
    );
    apply(registry, time, &position(lat, lng, 5000));
    apply(registry, time, &message!(VfrHud, |m| m.throttle = throttle));
    registry.update_current_settings(false);
}

fn leg(from: (f64, f64), to: (f64, f64)) -> f32 {
    let p = |(lat, lng): (f64, f64)| LatLon::new(lat, lng).unwrap();
    p(from).distance_to(p(to)).0 as f32
}

#[test]
fn time_in_air_and_distance_count_once_a_second_while_armed() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:4602-4626, 2878-2882
    let mut registry = VehicleRegistry::new();
    let (a, b, c, d) = (
        (-35.36, 149.16),
        (-35.36, 149.161),
        (-35.3605, 149.161),
        (-35.3605, 149.162),
    );
    // Second 0: the vehicle is first seen, and its count starts at this second - no tick.
    flying(&mut registry, at(0.5), true, 3, a.0, a.1, 20);
    assert_eq!(state(&registry).time_in_air, 0.0);
    // Second 1: a tick. No distance - there was no last position - but a second in the air.
    flying(&mut registry, at(1.2), true, 3, b.0, b.1, 20);
    assert_eq!(state(&registry).dist_traveled, 0.0);
    assert_eq!(state(&registry).time_in_air, 1.0);
    assert_eq!(state(&registry).time_since_arm_in_air, 1.0);
    // Still second 1: nothing.
    flying(&mut registry, at(1.9), true, 3, c.0, c.1, 20);
    assert_eq!(state(&registry).dist_traveled, 0.0);
    assert_eq!(state(&registry).time_in_air, 1.0);
    // Second 2: from where the last second's tick was to here.
    flying(&mut registry, at(2.0), true, 3, c.0, c.1, 20);
    let mut dist = leg(b, c);
    assert_eq!(state(&registry).dist_traveled, dist);
    assert!((dist - 55.6).abs() < 0.1, "0.0005 degrees north: {dist}");
    assert_eq!(state(&registry).time_in_air, 2.0);

    // Second 3 on a 2D fix: no distance, but the position is taken for the next.
    flying(&mut registry, at(3.0), true, 2, d.0, d.1, 20);
    assert_eq!(state(&registry).dist_traveled, dist);
    // Second 4 back on 3D, not moved: 0 m added.
    flying(&mut registry, at(4.0), true, 3, d.0, d.1, 20);
    assert_eq!(state(&registry).dist_traveled, dist);
    assert_eq!(state(&registry).time_in_air, 4.0);

    // Armed on the ground, throttle 12% and slower than 3 m/s: not in the air.
    flying(&mut registry, at(5.0), true, 3, d.0, d.1, 12);
    assert_eq!(state(&registry).time_in_air, 4.0);
    // Disarmed at full throttle: not in the air either, and no distance.
    flying(&mut registry, at(6.0), false, 3, a.0, a.1, 100);
    assert_eq!(state(&registry).time_in_air, 4.0);
    assert_eq!(state(&registry).dist_traveled, dist);
    // Armed again: the count since arming restarts, the total does not.
    flying(&mut registry, at(7.0), true, 3, a.0, a.1, 13);
    assert_eq!(state(&registry).time_in_air, 5.0);
    assert_eq!(state(&registry).time_since_arm_in_air, 1.0);

    // A minute and a second later is one tick: seconds are counted by the seconds field.
    flying(&mut registry, at(68.0), true, 3, b.0, b.1, 13);
    dist += leg(a, b);
    assert_eq!(state(&registry).time_in_air, 6.0);
    assert_eq!(state(&registry).dist_traveled, dist);
    // And a whole minute later is none.
    flying(&mut registry, at(128.3), true, 3, c.0, c.1, 13);
    assert_eq!(state(&registry).time_in_air, 6.0);

    // A shut link restarts the distance from 0 at the next second that adds to it.
    apply(&mut registry, at(129.0), &position(d.0, d.1, 5000));
    registry.update_current_settings(true);
    assert_eq!(state(&registry).dist_traveled, leg(b, d));
}

#[test]
fn time_in_air_min_sec_is_minutes_and_hundredths() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:1195-1197
    let mut s = VehicleState::default();
    for (seconds, expected) in [
        (0.0_f32, 0.0_f32),
        (61.0, 1.0 + 1.0 / 100.0),
        (3661.25, 61.0 + 1.25 / 100.0),
        // `(int)` truncates toward zero and `%` keeps the sign.
        (-61.0, -1.0 + -1.0 / 100.0),
    ] {
        s.time_in_air = seconds;
        assert_eq!(s.time_in_air_min_sec(), expected, "{seconds}");
    }
}

// --- battery_usedmah, battery_mahperkm, battery_kmleft ------------------------------------------

#[test]
fn the_used_capacity_integrates_the_current_between_battery_reports() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:1396-1415, 2952, 3120
    let hours = |ticks: i64| ticks as f64 * (1.0 / 36_000_000_000.0);
    let sys_status = |current: i16| message!(SysStatus, |m| m.current_battery = current);
    let mut registry = VehicleRegistry::new();
    // "No sensor": the clock starts, the current is 0, nothing is used.
    apply(&mut registry, t0(), &sys_status(-1));
    assert_eq!(state(&registry).battery.current, 0.0);
    assert_eq!(state(&registry).battery_used_mah, 0.0);
    // 10 A for the 36 s since the clock started.
    apply(&mut registry, at(36.0), &sys_status(1000));
    let mut used = 10.0 * 1000.0 * hours(360_000_000);
    assert_eq!(state(&registry).battery_used_mah, used);
    assert!((used - 100.0).abs() < 1e-9, "{used}");
    assert_eq!(state(&registry).battery.current, 10.0);
    // "No sensor" again: 0 A, and the clock stays where the last reading left it...
    apply(&mut registry, at(72.0), &sys_status(-1));
    assert_eq!(state(&registry).battery.current, 0.0);
    // ...so 5 A now counts from 36 s, not 72.
    apply(&mut registry, at(108.0), &sys_status(500));
    used += 5.0 * 1000.0 * hours(720_000_000);
    assert_eq!(state(&registry).battery_used_mah, used);
    // BATTERY_STATUS replaces it with the vehicle's own count.
    apply(
        &mut registry,
        at(109.0),
        &message!(BatteryStatus, |m| {
            m.current_consumed = 150;
            m.battery_remaining = 50;
            m.voltages = [u16::MAX; 10];
        }),
    );
    assert_eq!(state(&registry).battery_used_mah, 150.0);
    assert_eq!(state(&registry).battery.consumed_mah, 150);
}

#[test]
fn the_battery_estimates_divide_as_the_csharp_does() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:1471-1480
    let mut s = VehicleState::default();
    s.battery_used_mah = 150.0;
    s.battery.remaining_percent = 50;
    // No distance yet: infinite per kilometre, and so 0 left.
    assert_eq!(s.battery_mah_per_km(), f64::INFINITY);
    assert_eq!(s.battery_km_left(), 0.0);
    s.dist_traveled = 2500.0;
    assert_eq!(
        s.battery_mah_per_km(),
        150.0 / f64::from(2500.0_f32 / 1000.0)
    );
    assert_eq!(s.battery_mah_per_km(), 60.0);
    // Half used for 2.5 km: 2.5 km left.
    assert_eq!(s.battery_km_left(), (2.0 * 150.0 - 150.0) / 60.0);
    // At 100% remaining: 100 / 0.
    s.battery.remaining_percent = 100;
    assert_eq!(s.battery_km_left(), f64::INFINITY);
    // Nothing used and nowhere gone: 0 / 0.
    let empty = VehicleState::default();
    assert!(empty.battery_mah_per_km().is_nan());
    assert!(empty.battery_km_left().is_nan());
}

// --- DistFromMovingBase, gimbal, shots, fence -------------------------------------------------

#[test]
fn the_distance_from_the_moving_base_is_the_flat_projection() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:1786-1805
    let mut s = VehicleState::default();
    assert_eq!(s.dist_from_moving_base(), 0.0, "no position");
    s.position = Some(LatLon::new(-35.0, 149.0).unwrap());
    // No base set: the C#'s base is (0, 0), never null, so this is the distance from 0° 0°.
    let from_null_island =
        ((35.0_f64 * 111_319.5).powi(2) + (149.0_f64 * 111_319.5).powi(2)).sqrt() as f32;
    assert_eq!(s.dist_from_moving_base(), from_null_island);
    s.base = LatLngAlt {
        lat: -35.001,
        lng: 149.002,
        alt: 10.0,
    };
    let scale = (35.001_f64 * 0.017_453_292_5).cos();
    let dlat = (-35.001_f64 - -35.0).abs() * 111_319.5;
    let dlng = (149.002_f64 - 149.0).abs() * 111_319.5 * scale;
    assert_eq!(
        s.dist_from_moving_base(),
        (dlat * dlat + dlng * dlng).sqrt() as f32
    );
}

#[test]
fn the_gimbal_point_reads_as_floats_and_zero_unset() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2022-2042
    let mut s = VehicleState::default();
    assert_eq!((s.gimbal_lat(), s.gimbal_lng()), (0.0, 0.0));
    s.gimbal_point = Some(LatLngAlt {
        lat: -35.123_456_789,
        lng: 149.987_654_321,
        alt: 0.0,
    });
    assert_eq!(s.gimbal_lat(), -35.123_456_789_f64 as f32);
    assert_eq!(s.gimbal_lng(), 149.987_654_321_f64 as f32);
}

#[test]
fn the_shot_interval_is_the_last_two_shots_apart() {
    // C#: GCSViews/FlightData.cs:4021-4038
    assert_eq!(VehicleState::shot_interval([]), None);
    // One shot: its time less double.MinValue, which rounds to double.MaxValue.
    assert_eq!(VehicleState::shot_interval([5_000_000]), Some(f64::MAX));
    let seconds = |usec: u64| (usec as f64 / 1000.0) / 1000.0;
    assert_eq!(
        VehicleState::shot_interval([1_000_000, 3_500_000, 4_250_001]),
        Some(seconds(4_250_001) - seconds(3_500_000))
    );
}

#[test]
fn the_geofence_distance_on_simple_fences() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:1617-1753
    let mut s = VehicleState::default();
    s.position = Some(LatLon::new(-35.0, 149.0).unwrap());
    assert_eq!(s.geo_fence_dist(&[]), 99999.0, "no fence");
    let circle = |command, radius| FenceItem {
        command,
        param1: radius,
        x: -350_000_000,
        y: 1_490_000_000,
    };
    // At the centre of a 100 m inclusion circle: 100 m from its edge, and its centre does not
    // count as a point.
    assert_eq!(s.geo_fence_dist(&[circle(5003, 100.0)]), 100.0);
    // At the centre of an exclusion circle: breached.
    assert_eq!(s.geo_fence_dist(&[circle(5004, 100.0)]), 0.0);
    // The return point is not part of the fence.
    assert_eq!(s.geo_fence_dist(&[circle(5000, 0.0)]), 99999.0);
    // Outside an inclusion square: breached.
    let square = [
        (-35.1, 148.9),
        (-35.1, 148.95),
        (-35.05, 148.95),
        (-35.05, 148.9),
    ]
    .map(|(lat, lng): (f64, f64)| FenceItem {
        command: 5001,
        param1: 4.0,
        x: (lat * 1e7) as i32,
        y: (lng * 1e7) as i32,
    });
    assert_eq!(s.geo_fence_dist(&square), 0.0);
}

// --- speedup ------------------------------------------------------------------------------------

#[test]
fn the_speedup_compares_the_imu_clock_with_ours() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3699-3713
    let imu = |usec: u64| message!(RawImu, |m| m.time_usec = usec);
    let mut registry = VehicleRegistry::new();
    // The first reading within ten seconds of boot is measured from MinValue: next to nothing.
    apply(&mut registry, t0(), &imu(2_000_000));
    let first = (0.0_f64 * 0.95 + 2.0 / total_seconds(t0().ticks()) * 0.05) as f32;
    assert_eq!(state(&registry).speedup, first);
    // 0.5 s of IMU in 0.25 s of ours: twice as fast.
    apply(&mut registry, at(0.25), &imu(2_500_000));
    let mut speedup =
        (f64::from(first) * 0.95 + (2.5 - 2.0) / total_seconds(2_500_000) * 0.05) as f32;
    assert_eq!(state(&registry).speedup, speedup);
    // The IMU clock going back - a reboot - is not counted, now or after.
    apply(&mut registry, at(0.5), &imu(1_000_000));
    assert_eq!(state(&registry).speedup, speedup);
    apply(&mut registry, at(0.75), &imu(1_250_000));
    assert_eq!(state(&registry).speedup, speedup);
    // Ten seconds or more of IMU at once is not counted either.
    apply(&mut registry, at(1.0), &imu(12_500_000));
    assert_eq!(state(&registry).speedup, speedup);
    apply(&mut registry, at(1.5), &imu(2_600_000));
    speedup = (f64::from(speedup) * 0.95 + 0.1 / total_seconds(12_500_000) * 0.05) as f32;
    assert_eq!(state(&registry).speedup, speedup);

    // A vehicle first heard 20 s after boot never starts: its first reading is 20 s from 0.
    let mut late = VehicleRegistry::new();
    for (i, usec) in [20_000_000_u64, 20_100_000, 20_200_000]
        .into_iter()
        .enumerate()
    {
        late.apply_at(1, 1, 0, &imu(usec), at(i as f64 * 0.1));
    }
    assert_eq!(state(&late).speedup, 0.0);
}

// --- hilch1 to hilch8 -------------------------------------------------------------------------

#[test]
fn the_hil_channels_come_from_rc_channels_scaled_and_hil_controls() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2307-2324, 2551-2564
    let mut registry = VehicleRegistry::new();
    apply(
        &mut registry,
        t0(),
        &message!(RcChannelsScaled, |m| {
            m.chan1_scaled = 1;
            m.chan2_scaled = -2;
            m.chan3_scaled = 3;
            m.chan4_scaled = -4;
            m.chan5_scaled = 5;
            m.chan6_scaled = -6;
            m.chan7_scaled = 10_000;
            m.chan8_scaled = -10_000;
        }),
    );
    assert_eq!(
        state(&registry).hil_channels,
        [1, -2, 3, -4, 5, -6, 10_000, -10_000]
    );
    apply(
        &mut registry,
        t0(),
        &message!(HilControls, |m| {
            m.roll_ailerons = 0.123_45;
            m.pitch_elevator = -0.999_99;
            m.throttle = f32::NAN;
            // 2147483593.75 in single precision is 2^31: past int's range.
            m.yaw_rudder = 214_748.36;
        }),
    );
    assert_eq!(
        state(&registry).hil_channels,
        [
            (0.123_45_f32 * 10000.0) as i32,
            -9999,
            // NaN and overflow are int.MinValue, as the C#'s conversion makes them.
            i32::MIN,
            i32::MIN,
            5,
            -6,
            10_000,
            -10_000
        ],
        "the first four; the rest kept"
    );
    assert_eq!((0.123_45_f32 * 10000.0) as i32, 1234);
}

// --- lowairspeed --------------------------------------------------------------------------------

#[test]
fn the_low_airspeed_warning_needs_the_parameter_the_sensor_and_the_air() {
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3858-3888
    const DIFF_PRESSURE: u32 = 16;
    let vfr = |airspeed: f32| {
        message!(VfrHud, |m| {
            m.airspeed = airspeed;
            m.groundspeed = 12.0;
            m.throttle = 50;
        })
    };
    let sensors = |health: u32| {
        message!(SysStatus, |m| {
            m.onboard_control_sensors_present = DIFF_PRESSURE;
            m.onboard_control_sensors_enabled = DIFF_PRESSURE;
            m.onboard_control_sensors_health = health;
            m.current_battery = -1;
        })
    };
    let mut registry = VehicleRegistry::new();
    apply(&mut registry, t0(), &heartbeat(true));
    apply(&mut registry, t0(), &sensors(DIFF_PRESSURE));
    registry
        .working_mut(AUTOPILOT)
        .unwrap()
        .set_airspeed_min_params(None, Some(12.0));
    // Not in the air yet: the parameter is not even read.
    apply(&mut registry, t0(), &vfr(9.0));
    assert!(!state(&registry).low_airspeed);
    // A second in the air (12 m/s over the ground): below 12 m/s is low.
    apply(&mut registry, at(1.0), &vfr(9.0));
    registry.update_current_settings(false);
    assert_eq!(state(&registry).time_since_arm_in_air, 1.0);
    apply(&mut registry, at(1.1), &vfr(9.0));
    assert!(state(&registry).low_airspeed);
    apply(&mut registry, at(1.2), &vfr(12.0));
    assert!(!state(&registry).low_airspeed, "not below");
    // The sensor unhealthy: no warning.
    apply(&mut registry, at(1.3), &sensors(0));
    apply(&mut registry, at(1.3), &vfr(9.0));
    assert!(!state(&registry).low_airspeed);
    apply(&mut registry, at(1.4), &sensors(DIFF_PRESSURE));
    // AIRSPEED_MIN wins over ARSPD_FBW_MIN, but is only read five seconds after the last read.
    registry
        .working_mut(AUTOPILOT)
        .unwrap()
        .set_airspeed_min_params(Some(8.0), Some(12.0));
    apply(&mut registry, at(5.0), &vfr(9.0));
    assert!(state(&registry).low_airspeed, "still 12 until 6.1 s");
    apply(&mut registry, at(6.2), &vfr(9.0));
    assert!(!state(&registry).low_airspeed, "8 now");
    // Without either parameter the last minimum read stays.
    registry
        .working_mut(AUTOPILOT)
        .unwrap()
        .set_airspeed_min_params(None, None);
    apply(&mut registry, at(12.0), &vfr(7.0));
    assert!(state(&registry).low_airspeed);
    // Disarmed: no warning.
    apply(&mut registry, at(12.5), &heartbeat(false));
    apply(&mut registry, at(12.5), &vfr(7.0));
    assert!(!state(&registry).low_airspeed);

    // A vehicle that never gets the parameter never warns: its minimum is 0.
    let mut bare = VehicleRegistry::new();
    bare.apply_at(1, 1, 0, &heartbeat(true), t0());
    bare.apply_at(1, 1, 0, &sensors(DIFF_PRESSURE), t0());
    bare.apply_at(1, 1, 0, &vfr(1.0), at(1.0));
    bare.update_current_settings(false);
    bare.apply_at(1, 1, 0, &vfr(1.0), at(1.1));
    assert!(!state(&bare).low_airspeed);
}

// --- the registry -------------------------------------------------------------------------------

#[test]
fn the_registry_counts_the_seconds_of_every_vehicle() {
    // C#: MainV2.cs:3058-3069, every MAV on the port.
    let mut registry = VehicleRegistry::new();
    for sysid in [1, 2] {
        registry.apply_at(sysid, 1, 0, &heartbeat(true), at(0.0));
        registry.apply_at(sysid, 1, 0, &message!(VfrHud, |m| m.throttle = 50), at(1.0));
    }
    registry.update_current_settings(false);
    for sysid in [1, 2] {
        assert_eq!(
            registry
                .working(VehicleId::new(sysid, 1))
                .unwrap()
                .time_in_air,
            1.0
        );
    }
}
