//! Drives the state model with a real flight and checks what it converged to.
//!
//! A telemetry model can decode every field correctly and still be wrong as a *model*: applying
//! the wrong message to the wrong slot, missing a unit conversion, or treating an unset value as
//! real. These assertions are about the flight, not the bytes.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use mp_mavlink::FrameDecoder;
use mp_mavlink_dialects::all::{DIALECT, MavMessage};
use mp_transport::{ReplayTransport, Transport};
use mp_vehicle::{VehicleId, VehicleRegistry};

/// What the flight looked like over its whole duration, not just at the final instant.
#[derive(Debug, Default)]
struct FlightObservations {
    positions: u64,
    unset_positions: u64,
    lat_range: (f64, f64),
    lon_range: (f64, f64),
    max_satellites: u8,
    best_fix: u8,
    max_ground_speed: f64,
    max_battery_volts: f32,
    max_abs_roll: f64,
    max_abs_pitch: f64,
}

fn replay_flight() -> VehicleRegistry {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink/autotest.tlog"
    );
    let mut transport = ReplayTransport::open(path).unwrap().with_chunk_size(128);
    let mut decoder = FrameDecoder::new();
    let mut registry = VehicleRegistry::new();
    let mut buf = [0u8; 512];

    loop {
        let n = transport.read(&mut buf).unwrap();
        if n == 0 {
            break;
        }
        decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
            if let Some(msg) = MavMessage::decode(frame.msgid, frame.payload) {
                registry.apply(frame.sysid, frame.compid, frame.seq, &msg);
            }
        });
    }
    decoder.flush(&DIALECT, |_| {});
    registry
}

/// Replays while recording the extremes of every quantity we model.
fn observe_flight() -> (VehicleRegistry, FlightObservations) {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink/autotest.tlog"
    );
    let mut transport = ReplayTransport::open(path).unwrap().with_chunk_size(128);
    let mut decoder = FrameDecoder::new();
    let mut registry = VehicleRegistry::new();
    let mut obs = FlightObservations {
        lat_range: (f64::MAX, f64::MIN),
        lon_range: (f64::MAX, f64::MIN),
        ..FlightObservations::default()
    };
    let mut buf = [0u8; 512];

    loop {
        let n = transport.read(&mut buf).unwrap();
        if n == 0 {
            break;
        }
        decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
            let Some(msg) = MavMessage::decode(frame.msgid, frame.payload) else {
                return;
            };
            registry.apply(frame.sysid, frame.compid, frame.seq, &msg);

            // Only the autopilot's telemetry describes the flight.
            if frame.sysid != 1 || frame.compid != 1 {
                return;
            }
            match msg {
                MavMessage::GlobalPositionInt(_) => {
                    obs.positions += 1;
                    match registry
                        .working(VehicleId::new(1, 1))
                        .and_then(|s| s.position)
                    {
                        Some(p) => {
                            obs.lat_range.0 = obs.lat_range.0.min(p.latitude());
                            obs.lat_range.1 = obs.lat_range.1.max(p.latitude());
                            obs.lon_range.0 = obs.lon_range.0.min(p.longitude());
                            obs.lon_range.1 = obs.lon_range.1.max(p.longitude());
                        }
                        None => obs.unset_positions += 1,
                    }
                }
                MavMessage::GpsRawInt(m) => {
                    obs.max_satellites = obs.max_satellites.max(m.satellites_visible);
                    obs.best_fix = obs.best_fix.max(m.fix_type);
                }
                MavMessage::VfrHud(_) | MavMessage::SysStatus(_) | MavMessage::Attitude(_) => {
                    if let Some(s) = registry.working(VehicleId::new(1, 1)) {
                        obs.max_ground_speed = obs.max_ground_speed.max(s.ground_speed.0);
                        obs.max_battery_volts = obs.max_battery_volts.max(s.battery.voltage);
                        obs.max_abs_roll = obs.max_abs_roll.max(s.attitude.roll.0.abs());
                        obs.max_abs_pitch = obs.max_abs_pitch.max(s.attitude.pitch.0.abs());
                    }
                }
                _ => {}
            }
        });
    }
    decoder.flush(&DIALECT, |_| {});
    (registry, obs)
}

#[test]
fn a_real_flight_produces_a_plausible_vehicle_state() {
    let (registry, obs) = observe_flight();
    assert!(
        !registry.is_empty(),
        "the log must contain at least one vehicle"
    );

    // The autopilot is system 1, component 1.
    let autopilot = VehicleId::new(1, 1);
    let state = registry.working(autopilot).expect("autopilot state");

    assert!(
        state.messages_applied > 10_000,
        "applied {}",
        state.messages_applied
    );
    assert_eq!(state.autopilot, 3, "ArduPilotMega");
    assert_eq!(state.vehicle_type, 2, "the autotest flies a quadrotor");

    // Attitude must stay within physical bounds - a units mistake shows up here immediately.
    assert!(
        obs.max_abs_roll <= std::f64::consts::PI,
        "max roll {} rad",
        obs.max_abs_roll
    );
    assert!(
        obs.max_abs_pitch <= std::f64::consts::FRAC_PI_2 + 0.01,
        "max pitch {} rad",
        obs.max_abs_pitch
    );
    // Note what this corpus is: an ArduPilot autotest run, which sits on the ground and
    // exercises protocol handling rather than flying. Max |roll| is about 0.002 rad, confirmed
    // against the C# decode. Asserting the vehicle manoeuvred would be asserting something the
    // data does not contain. A dynamic-flight corpus is still needed - see D19's SITL work.
    assert!(
        obs.max_abs_pitch > 0.0,
        "attitude must actually be decoded, not left at zero"
    );

    // This autotest flies at ArduPilot's default SITL location (CMAC, Canberra). The log both
    // begins and ends with the simulator reset, so 0/0 positions are expected - they are
    // correctly modelled as "no position", not as a spot in the Gulf of Guinea.
    assert!(
        obs.positions > 1_000,
        "position messages: {}",
        obs.positions
    );
    assert!(
        obs.unset_positions > 0,
        "this corpus contains unset positions"
    );
    assert!(
        obs.unset_positions < obs.positions / 2,
        "most positions should be real: {} of {}",
        obs.unset_positions,
        obs.positions
    );
    assert!(
        (-36.0..=-35.0).contains(&obs.lat_range.0) && (-36.0..=-35.0).contains(&obs.lat_range.1),
        "latitudes {:?} are not the SITL default location",
        obs.lat_range
    );
    assert!(
        (149.0..=150.0).contains(&obs.lon_range.0) && (149.0..=150.0).contains(&obs.lon_range.1),
        "longitudes {:?} are not the SITL default location",
        obs.lon_range
    );

    assert!(
        obs.best_fix >= 3,
        "the flight should reach a 3D fix, best was {}",
        obs.best_fix
    );
    assert!(obs.max_satellites >= 5, "max sats {}", obs.max_satellites);

    // A simulated battery sits at a sane pack voltage rather than 0 or a scaling error.
    assert!(
        (5.0..=60.0).contains(&obs.max_battery_volts),
        "battery voltage {} V looks like a scaling error",
        obs.max_battery_volts
    );

    // Speeds in m/s, not cm/s: a 10x or 100x error is the classic unit bug.
    assert!(
        (0.0..100.0).contains(&obs.max_ground_speed),
        "max ground speed {} m/s is implausible - a 10x or 100x unit error would show here",
        obs.max_ground_speed
    );

    assert!(state.link.received > 10_000);
    assert!(
        state.link.loss_percent() < 5.0,
        "loss {}%",
        state.link.loss_percent()
    );
}

#[test]
fn distinct_systems_do_not_overwrite_each_other() {
    // The failure this guards: treating a link as one vehicle, so a gimbal or a second aircraft
    // clobbers the autopilot's telemetry.
    let registry = replay_flight();
    let ids = registry.ids();
    assert!(!ids.is_empty());

    for id in &ids {
        let state = registry.working(*id).expect("state exists");
        assert_eq!(state.sysid, id.sysid);
        assert_eq!(state.compid, id.compid);
    }

    if ids.len() > 1 {
        // Each system keeps its own message count.
        let counts: Vec<u64> = ids
            .iter()
            .map(|id| registry.working(*id).unwrap().messages_applied)
            .collect();
        assert!(counts.iter().all(|c| *c > 0));
    }
}

#[test]
fn snapshots_reflect_the_replayed_state() {
    let mut registry = replay_flight();
    let autopilot = VehicleId::new(1, 1);
    registry.publish_all();

    let handle = registry.handle(autopilot).expect("handle");
    let snapshot = handle.load();
    let working = registry.working(autopilot).expect("working");

    assert_eq!(snapshot.messages_applied, working.messages_applied);
    assert_eq!(snapshot.position, working.position);
    assert_eq!(snapshot.attitude, working.attitude);
}
