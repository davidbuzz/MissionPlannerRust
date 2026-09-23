//! What a render pass costs while telemetry is pouring in (DELIVERABLES.md D10).
//!
//! The deliverable asks for no UI stall over 8 ms during a 200 Hz telemetry storm. The part of
//! that which can be measured without a window - and the part the design rests on - is the
//! snapshot bus: a render pass takes one immutable snapshot per vehicle per frame and does nothing
//! else that can block. If taking a snapshot stays cheap while the link thread is writing at full
//! rate, the UI cannot stall on telemetry; if it does not, no amount of rendering work will save
//! it.
//!
//! This is a test rather than a benchmark so it runs in CI. The thresholds are generous compared
//! with the measured figures, because a shared machine under load is the environment it has to
//! pass in, and a timing test that fails on a busy runner teaches people to ignore it.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::time::{Duration, Instant};

use mp_link::{Link, LinkConfig};
use mp_mavlink::{Message as _, encode_v2};
use mp_mavlink_dialects::all::{Attitude, GlobalPositionInt, Heartbeat};
use mp_transport::Transport;
use mp_transport::testing::Loopback;

/// A frame budget. 8 ms is the deliverable's figure, which is half of a 60 Hz frame.
const FRAME_BUDGET: Duration = Duration::from_millis(8);

/// How many snapshots to take. At 60 Hz this is about eight seconds of rendering.
const SNAPSHOTS: usize = 500;

fn heartbeat(seq: u8) -> Vec<u8> {
    let message = Heartbeat {
        custom_mode: 0,
        r#type: 2,
        autopilot: 3,
        base_mode: 81,
        system_status: 3,
        mavlink_version: 3,
    };
    frame(seq, Heartbeat::ID, Heartbeat::CRC_EXTRA, |buffer| {
        message.encode(buffer)
    })
}

fn attitude(seq: u8, roll: f32) -> Vec<u8> {
    let message = Attitude {
        time_boot_ms: u32::from(seq),
        roll,
        pitch: 0.0,
        yaw: 0.0,
        rollspeed: 0.0,
        pitchspeed: 0.0,
        yawspeed: 0.0,
    };
    frame(seq, Attitude::ID, Attitude::CRC_EXTRA, |buffer| {
        message.encode(buffer)
    })
}

fn position(seq: u8) -> Vec<u8> {
    let message = GlobalPositionInt {
        time_boot_ms: u32::from(seq),
        lat: -353_632_620,
        lon: 1_491_652_370,
        alt: 600_000,
        relative_alt: 10_000,
        vx: 0,
        vy: 0,
        vz: 0,
        hdg: 18_000,
    };
    frame(
        seq,
        GlobalPositionInt::ID,
        GlobalPositionInt::CRC_EXTRA,
        |buffer| message.encode(buffer),
    )
}

/// Encodes one frame as a vehicle would send it.
fn frame(seq: u8, id: u32, crc_extra: u8, encode: impl FnOnce(&mut [u8]) -> usize) -> Vec<u8> {
    let mut payload = [0u8; 255];
    let len = encode(&mut payload);
    let mut out = [0u8; mp_mavlink::MAX_FRAME_LEN];
    let n = encode_v2(&mut out, seq, 1, 1, id, &payload[..len], crc_extra, 0).unwrap();
    out[..n].to_vec()
}

#[test]
fn a_render_pass_stays_cheap_while_telemetry_pours_in() {
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    // Wait for the vehicle to exist before measuring, so discovery is not counted as a stall.
    vehicle_side.write_all(&heartbeat(0)).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while link.primary_vehicle().is_none() {
        assert!(Instant::now() < deadline, "no vehicle appeared");
        std::thread::sleep(Duration::from_millis(10));
    }
    let (_, handle) = link.primary_vehicle().expect("a vehicle");

    // A writer running as fast as it can, which is harder than 200 Hz. If the snapshot path holds
    // up against an unthrottled writer it holds up against a real vehicle.
    let writer = std::thread::spawn(move || {
        let mut seq = 0u8;
        let until = Instant::now() + Duration::from_secs(3);
        while Instant::now() < until {
            let roll = f32::from(seq) / 100.0;
            if vehicle_side.write_all(&attitude(seq, roll)).is_err()
                || vehicle_side.write_all(&position(seq)).is_err()
                || vehicle_side.write_all(&heartbeat(seq)).is_err()
            {
                break;
            }
            seq = seq.wrapping_add(1);
        }
    });

    // What a render pass does: take one snapshot and read from it.
    let mut worst = Duration::ZERO;
    let mut total = Duration::ZERO;
    for _ in 0..SNAPSHOTS {
        let started = Instant::now();
        let state = handle.load();
        // Touch the fields a frame actually reads, so the compiler cannot elide the load.
        let sum = state.attitude.roll.0 + state.altitude_relative.0;
        std::hint::black_box(sum);
        let elapsed = started.elapsed();
        worst = worst.max(elapsed);
        total += elapsed;

        // Roughly 60 Hz, so this measures a renderer's pace rather than a tight loop.
        std::thread::sleep(Duration::from_millis(16));
    }

    writer.join().expect("the writer thread should finish");

    let average = total / u32::try_from(SNAPSHOTS).unwrap_or(1);
    assert!(
        worst < FRAME_BUDGET,
        "worst snapshot took {worst:?}, over the {FRAME_BUDGET:?} frame budget \
         (average {average:?})"
    );
    // The design claim is that this is nanoseconds, not milliseconds. A microsecond average would
    // still pass the budget while meaning something had gone badly wrong.
    assert!(
        average < Duration::from_micros(100),
        "average snapshot took {average:?}, which is far above the expected nanoseconds"
    );
}

#[test]
fn snapshots_keep_arriving_while_the_link_is_saturated() {
    // A cheap snapshot that never changes would pass the timing test and be useless. The point of
    // the bus is fresh data, so this asserts the reader actually sees the writer's work.
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    vehicle_side.write_all(&heartbeat(0)).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while link.primary_vehicle().is_none() {
        assert!(Instant::now() < deadline, "no vehicle appeared");
        std::thread::sleep(Duration::from_millis(10));
    }
    let (_, handle) = link.primary_vehicle().expect("a vehicle");

    let mut seen = std::collections::BTreeSet::new();
    for seq in 0..60u8 {
        let roll = f32::from(seq) / 10.0;
        vehicle_side.write_all(&attitude(seq, roll)).unwrap();
        std::thread::sleep(Duration::from_millis(5));
        let state = handle.load();
        // Rounded, because the wire carries f32 and the state keeps radians as f64.
        #[allow(clippy::cast_possible_truncation)] // roll in milliradians fits an i64 easily
        let milliradians = (state.attitude.roll.0 * 1000.0).round() as i64;
        seen.insert(milliradians);
    }

    assert!(
        seen.len() > 10,
        "the snapshot barely changed across 60 updates: {} distinct values",
        seen.len()
    );
}
