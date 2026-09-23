//! The ported `CurrentState` fields, filled from recorded flights.
//!
//! `current_state.rs` pins the C#'s arithmetic on constructed messages; this replays the two
//! recorded SITL flights in `testdata/mavlink` through the registry, as the link thread does, and
//! checks the fields land in the units a real vehicle's numbers imply. A scaling mistake - cm for
//! m, centidegrees for degrees - shows here as a value no aircraft reports.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::indexing_slicing
)]

use mp_mavlink::FrameDecoder;
use mp_mavlink_dialects::all::{DIALECT, MavMessage};
use mp_transport::{ReplayTransport, Transport};
use mp_vehicle::{VehicleId, VehicleRegistry, VehicleState};

/// Replays a fixture, calling `after` with the autopilot's state after each of its messages.
fn replay(name: &str, mut after: impl FnMut(&MavMessage, &VehicleState)) -> VehicleRegistry {
    let path = format!(
        "{}/../../testdata/mavlink/{name}",
        env!("CARGO_MANIFEST_DIR")
    );
    let mut transport = ReplayTransport::open(&path).unwrap().with_chunk_size(512);
    let mut decoder = FrameDecoder::new();
    let mut registry = VehicleRegistry::new();
    let mut buf = [0u8; 512];
    let autopilot = VehicleId::new(1, 1);
    loop {
        let n = transport.read(&mut buf).unwrap();
        if n == 0 {
            break;
        }
        decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
            let Some(message) = MavMessage::decode(frame.msgid, frame.payload) else {
                return;
            };
            let id = registry.apply(frame.sysid, frame.compid, frame.seq, &message);
            if id == autopilot {
                after(&message, registry.working(autopilot).unwrap());
            }
        });
    }
    registry
}

/// The largest value seen, starting below anything real.
#[derive(Debug, Clone, Copy)]
struct Max(f64);

impl Default for Max {
    fn default() -> Self {
        Self(f64::MIN)
    }
}

impl Max {
    fn see(&mut self, value: impl Into<f64>) {
        self.0 = self.0.max(value.into());
    }
}

/// What a recorded flight put into the ported fields, over its whole length.
#[derive(Debug, Default)]
struct Seen {
    messages: std::collections::BTreeMap<&'static str, u64>,
    terrain_spacing: Max,
    terrain_height: Max,
    terrain_loaded: Max,
    battery_voltage: Max,
    battery_remaining: Max,
    servo1: Max,
    rc3: Max,
    home_altitude: Max,
    hdop: Max,
    h_acc: Max,
    satellites: Max,
    press_abs: Max,
    press_temp: Max,
    load: Max,
    ekf_status: Max,
    wind_speed: Max,
    wind_direction: Max,
    speed_from_velocities: Max,
    range: Max,
    distance_orientations: std::collections::BTreeSet<(u8, u8)>,
    board_millivolts: Max,
    free_memory: Max,
    /// The negated vertical acceleration of the second IMU: gravity, in milli-g.
    imu2_down: Max,
    imu_temperature: Max,
    gps_time_unix_ms: Max,
    esc_rpm: Max,
    ahrs2_latitude: Max,
    firmware_major: Max,
    aoa: Max,
}

/// Replays a fixture and checks each ported message against the state it produced.
#[allow(clippy::too_many_lines)] // one check per message type, flat
fn observe(name: &str) -> Seen {
    let mut seen = Seen::default();
    replay(name, |message, state| {
        *seen.messages.entry(message.name()).or_default() += 1;
        match message {
            MavMessage::TerrainReport(m) => {
                assert_eq!(state.terrain.spacing, m.spacing);
                assert_eq!(state.terrain.terrain_height, m.terrain_height);
                seen.terrain_spacing.see(state.terrain.spacing);
                seen.terrain_height.see(state.terrain.terrain_height);
                seen.terrain_loaded.see(state.terrain.loaded);
            }
            MavMessage::BatteryStatus(m) if m.id == 0 => {
                // The first cell, where reported, is the volts the C# shows for it.
                if m.voltages[0] != u16::MAX {
                    assert_eq!(state.battery.cells[0], f64::from(m.voltages[0]) / 1000.0);
                }
                assert!((0..=100).contains(&state.battery.remaining_percent));
                seen.battery_voltage.see(state.battery.voltage);
                seen.battery_remaining.see(state.battery.remaining_percent);
            }
            MavMessage::ServoOutputRaw(m) if m.port == 0 => {
                assert_eq!(state.servo_outputs[0], m.servo1_raw);
                assert_eq!(state.servo_outputs[15], m.servo16_raw);
                seen.servo1.see(state.servo_outputs[0]);
            }
            MavMessage::RcChannels(m) => {
                assert_eq!(state.rc.values[2], m.chan3_raw);
                seen.rc3.see(state.rc.values[2]);
            }
            MavMessage::HomePosition(_) => seen.home_altitude.see(state.home_altitude.0),
            MavMessage::GpsRawInt(m) => {
                assert_eq!(state.gps.fix_type, m.fix_type);
                seen.hdop.see(state.gps.hdop);
                seen.h_acc.see(state.gps.h_acc);
                seen.satellites.see(state.gps.satellites_visible);
            }
            MavMessage::ScaledPressure(_) => {
                seen.press_abs.see(state.press_abs);
                seen.press_temp.see(state.press_temp);
            }
            MavMessage::SysStatus(_) => seen.load.see(state.load),
            MavMessage::EkfStatusReport(_) => seen.ekf_status.see(state.ekf_status()),
            MavMessage::Wind(_) => {
                assert!((0.0..360.0).contains(&state.wind_direction));
                seen.wind_speed.see(state.wind_speed);
                seen.wind_direction.see(state.wind_direction);
            }
            MavMessage::Airspeed(m) if m.id == 0 => {
                assert_eq!(state.air_speed.0, f64::from(m.airspeed));
            }
            MavMessage::GlobalPositionInt(_) => seen
                .speed_from_velocities
                .see((state.velocity_north.powi(2) + state.velocity_east.powi(2)).sqrt()),
            MavMessage::DistanceSensor(m) => {
                assert_eq!(
                    state.rangefinder.distances.get(usize::from(m.id)).copied(),
                    (m.id < 10).then_some(m.current_distance)
                );
                seen.distance_orientations.insert((m.id, m.orientation));
                // The lowest downward-facing sensor (orientation 25) is the altitude, in the
                // whole metres the C#'s integer division leaves.
                if (m.id, m.orientation) == (0, 25) {
                    assert_eq!(state.rangefinder.range, f32::from(m.current_distance / 100));
                }
                seen.range.see(state.rangefinder.range);
            }
            MavMessage::PowerStatus(m) => {
                assert_eq!(state.board.board_voltage, f32::from(m.vcc));
                seen.board_millivolts.see(state.board.board_voltage);
            }
            MavMessage::Meminfo(_) => seen.free_memory.see(state.board.free_memory),
            MavMessage::RawImu(m) => {
                assert_eq!(state.imu[0].accel[2], f32::from(m.zacc));
                seen.imu_temperature.see(state.imu[0].temperature);
            }
            MavMessage::ScaledImu2(m) => {
                assert_eq!(state.imu[1].accel[2], f32::from(m.zacc));
                seen.imu2_down.see(-state.imu[1].accel[2]);
            }
            MavMessage::SystemTime(m) => {
                assert_eq!(state.gps_time_unix_ms, m.time_unix_usec / 1000);
                #[allow(clippy::cast_precision_loss)] // a plausibility range
                seen.gps_time_unix_ms.see(state.gps_time_unix_ms as f64);
            }
            MavMessage::EscTelemetry1To4(m) => {
                assert_eq!(state.escs[0].rpm, f32::from(m.rpm[0]));
                seen.esc_rpm.see(state.escs[0].rpm);
            }
            MavMessage::Ahrs2(m) => {
                assert_eq!(state.ahrs2.lat, f64::from(m.lat) / 1e7);
                if m.lat != 0 {
                    seen.ahrs2_latitude.see(state.ahrs2.lat);
                }
            }
            MavMessage::AutopilotVersion(m) => {
                assert_eq!(
                    state.autopilot_info.version,
                    m.flight_sw_version.to_be_bytes()
                );
                seen.firmware_major.see(state.autopilot_info.version[0]);
            }
            MavMessage::AoaSsa(m) => {
                assert_eq!((state.aoa, state.ssa), (m.aoa, m.ssa));
                seen.aoa.see(state.aoa.abs());
            }
            _ => {}
        }
    });
    seen
}

#[test]
fn the_autotest_flight_fills_the_ported_fields_in_real_units() {
    let seen = observe("autotest.tlog");
    eprintln!("autotest.tlog: {seen:#?}");

    // TERRAIN_SPACING's default, and CMAC's ground height.
    assert_eq!(seen.terrain_spacing.0, 100.0);
    assert!((550.0..650.0).contains(&seen.terrain_height.0), "{seen:?}");
    assert!(seen.terrain_loaded.0 > 0.0);
    // A 3S simulated pack, in volts, summed from the cells and filtered.
    assert!((11.0..13.0).contains(&seen.battery_voltage.0), "{seen:?}");
    assert_eq!(seen.battery_remaining.0, 100.0);
    // Microsecond pulse widths, not a percentage and not zero.
    assert!((1000.0..=2000.0).contains(&seen.servo1.0), "{seen:?}");
    assert!((1000.0..=2000.0).contains(&seen.rc3.0), "{seen:?}");
    // Home at CMAC, in metres above sea level rather than millimetres.
    assert!((550.0..650.0).contains(&seen.home_altitude.0), "{seen:?}");
    // SITL's GPS: an HDOP of 1.21 and a MAVLink 2 accuracy in metres.
    assert_eq!(seen.hdop.0, f64::from(1.21_f32));
    assert!((0.1..5.0).contains(&seen.h_acc.0), "{seen:?}");
    assert!(seen.satellites.0 >= 6.0);
    // Hectopascals at 584 m, and a temperature in centidegrees as the wire sends it.
    assert!((900.0..1000.0).contains(&seen.press_abs.0), "{seen:?}");
    assert!((1000.0..6000.0).contains(&seen.press_temp.0), "{seen:?}");
    assert!(
        (0.0..=100.0).contains(&seen.load.0),
        "a percentage: {seen:?}"
    );
    assert!((0.0..=1.0).contains(&seen.ekf_status.0), "{seen:?}");
    // Metres per second from centimetres per second.
    assert!(
        (0.0..50.0).contains(&seen.speed_from_velocities.0),
        "{seen:?}"
    );
    // One downward rangefinder, reading whole metres off the ground.
    assert_eq!(
        seen.distance_orientations
            .iter()
            .copied()
            .collect::<Vec<_>>(),
        [(0, 25)]
    );
    assert!((1.0..50.0).contains(&seen.range.0), "{seen:?}");

    // The board's 5 V rail in millivolts, as the C# shows it.
    assert!(
        (4500.0..5500.0).contains(&seen.board_millivolts.0),
        "{seen:?}"
    );
    assert!(seen.free_memory.0 > 0.0, "{seen:?}");
    // Gravity and the take-off on the second IMU, in SCALED_IMU2's milli-g: about a thousand,
    // not the 9.8 of m/s².
    assert!((900.0..3000.0).contains(&seen.imu2_down.0), "{seen:?}");
    assert!(
        (0.0..100.0).contains(&seen.imu_temperature.0),
        "degrees: {seen:?}"
    );
    // The vehicle's clock, in milliseconds since 1970, somewhere after 2017.
    assert!(
        (1.5e12..4e12).contains(&seen.gps_time_unix_ms.0),
        "{seen:?}"
    );
    // SITL's ESC telemetry reports no rpm; the per-message check above is what holds here.
    assert!(
        seen.messages
            .get("ESC_TELEMETRY_1_TO_4")
            .copied()
            .unwrap_or(0)
            > 1000
    );
    assert!((-36.0..-35.0).contains(&seen.ahrs2_latitude.0), "{seen:?}");
    assert!(seen.firmware_major.0 >= 4.0, "{seen:?}");
}

#[test]
fn the_multisystem_flight_fills_wind_and_airspeed() {
    let seen = observe("multisystem.tlog");
    eprintln!("multisystem.tlog: {seen:#?}");
    assert!(seen.messages.get("WIND").copied().unwrap_or(0) > 1000);
    assert!(seen.messages.get("AIRSPEED").copied().unwrap_or(0) > 1000);
    assert!((0.0..30.0).contains(&seen.wind_speed.0), "m/s: {seen:?}");
    assert!((550.0..650.0).contains(&seen.home_altitude.0), "{seen:?}");
    assert!((900.0..1000.0).contains(&seen.press_abs.0), "{seen:?}");
    // A plane's angle of attack, in degrees.
    assert!(seen.messages.get("AOA_SSA").copied().unwrap_or(0) > 1000);
    assert!((0.0..30.0).contains(&seen.aoa.0), "{seen:?}");
}
