//! Link behaviour against an in-memory vehicle, plus an opt-in test against real SITL.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::time::{Duration, Instant};

use mp_link::messages::Severity;
use mp_link::{Link, LinkConfig};
use mp_mavlink::{FrameDecoder, Message as _, encode_v2};
use mp_mavlink_dialects::all::{CommandAck, DIALECT, Heartbeat, MavMessage, Statustext};
use mp_transport::Transport;
use mp_transport::testing::Loopback;
use mp_vehicle::VehicleId;

/// `MAV_MODE_FLAG_SAFETY_ARMED`. The other bits (custom mode, stabilise, manual input) say
/// nothing about whether the motors can spin, which is easy to get wrong: base_mode 81 is
/// `MANUAL_INPUT | STABILIZE | CUSTOM_MODE`, and is *disarmed*.
const ARMED_FLAG: u8 = 128;
/// A typical ArduPilot disarmed base_mode.
const BASE_MODE_DISARMED: u8 = 81;

/// Builds a heartbeat frame as a vehicle would send it.
fn vehicle_heartbeat(seq: u8) -> Vec<u8> {
    vehicle_heartbeat_with_mode(seq, BASE_MODE_DISARMED)
}

/// Builds a heartbeat with a specific base_mode.
fn vehicle_heartbeat_with_mode(seq: u8, base_mode: u8) -> Vec<u8> {
    let hb = Heartbeat {
        custom_mode: 0,
        r#type: 2,    // quadrotor
        autopilot: 3, // ArduPilotMega
        base_mode,
        system_status: 3,
        mavlink_version: 3,
    };
    let mut payload = [0u8; Heartbeat::LEN];
    hb.encode(&mut payload);
    let mut frame = [0u8; 64];
    let n = encode_v2(
        &mut frame,
        seq,
        1,
        1,
        Heartbeat::ID,
        &payload,
        Heartbeat::CRC_EXTRA,
        0,
    )
    .unwrap();
    frame[..n].to_vec()
}

/// Waits for a condition, failing with a message rather than hanging forever.
///
/// Twenty seconds, not five. A port that accepts a connection is not the same as a vehicle that
/// has started streaming: SITL binds 5760 immediately and can take several seconds more to emit
/// its first heartbeat. A five-second deadline passed when run by hand and failed intermittently
/// in a full suite run, which is the worst kind of test - one that fails for reasons unrelated to
/// what it is testing.
fn wait_for(what: &str, mut check: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(20);
    while Instant::now() < deadline {
        if check() {
            return;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    panic!("timed out waiting for {what}");
}

/// Builds a STATUSTEXT frame as ArduPilot sends one.
fn vehicle_statustext(seq: u8, severity: u8, text: &str) -> Vec<u8> {
    let mut raw = [0u8; 50];
    let bytes = text.as_bytes();
    let len = bytes.len().min(raw.len());
    raw[..len].copy_from_slice(&bytes[..len]);
    let message = Statustext {
        severity,
        text: raw,
        id: 0,
        chunk_seq: 0,
    };
    let mut payload = [0u8; Statustext::LEN];
    message.encode(&mut payload);
    let mut frame = [0u8; 128];
    let n = encode_v2(
        &mut frame,
        seq,
        1,
        1,
        Statustext::ID,
        &payload,
        Statustext::CRC_EXTRA,
        0,
    )
    .unwrap();
    frame[..n].to_vec()
}

/// Builds a COMMAND_ACK frame.
fn vehicle_command_ack(seq: u8, command: u16, result: u8) -> Vec<u8> {
    let message = CommandAck {
        command,
        result,
        progress: 0,
        result_param2: 0,
        target_system: 255,
        target_component: 190,
    };
    let mut payload = [0u8; CommandAck::LEN];
    message.encode(&mut payload);
    let mut frame = [0u8; 64];
    let n = encode_v2(
        &mut frame,
        seq,
        1,
        1,
        CommandAck::ID,
        &payload,
        CommandAck::CRC_EXTRA,
        0,
    )
    .unwrap();
    frame[..n].to_vec()
}

#[test]
fn what_the_vehicle_says_reaches_the_message_log() {
    // STATUSTEXT is how ArduPilot explains a refusal. Losing it means an operator who cannot see
    // why the aircraft will not arm.
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    vehicle_side.write_all(&vehicle_heartbeat(0)).unwrap();
    vehicle_side
        .write_all(&vehicle_statustext(1, 4, "PreArm: Compass not calibrated"))
        .unwrap();

    wait_for("the status text", || !link.recent_messages(10).is_empty());

    let messages = link.recent_messages(10);
    let last = messages.last().expect("a message");
    assert_eq!(last.text, "PreArm: Compass not calibrated");
    assert_eq!(last.severity, Severity::Warning);
    assert!(last.severity.is_urgent());
    assert_eq!(
        last.from,
        VehicleId {
            sysid: 1,
            compid: 1
        }
    );
}

#[test]
fn a_rejected_command_is_logged_by_name_not_by_number() {
    // "command 400: denied" tells an operator nothing they can act on.
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    vehicle_side.write_all(&vehicle_heartbeat(0)).unwrap();
    // 400 is MAV_CMD_COMPONENT_ARM_DISARM; 4 is MAV_RESULT_FAILED.
    vehicle_side
        .write_all(&vehicle_command_ack(1, 400, 4))
        .unwrap();

    wait_for("the command ack", || {
        link.recent_messages(10)
            .iter()
            .any(|m| m.text.contains("ARM_DISARM"))
    });

    let messages = link.recent_messages(10);
    let ack = messages
        .iter()
        .find(|m| m.text.contains("ARM_DISARM"))
        .expect("the ack");
    assert_eq!(ack.text, "MAV_CMD_COMPONENT_ARM_DISARM: failed");
    assert_eq!(ack.severity, Severity::Error);
}

#[test]
fn an_accepted_command_is_not_reported_as_an_error() {
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    vehicle_side.write_all(&vehicle_heartbeat(0)).unwrap();
    vehicle_side
        .write_all(&vehicle_command_ack(1, 400, 0))
        .unwrap();

    wait_for("the command ack", || {
        link.recent_messages(10)
            .iter()
            .any(|m| m.text.contains("ARM_DISARM"))
    });

    let messages = link.recent_messages(10);
    let ack = messages
        .iter()
        .find(|m| m.text.contains("ARM_DISARM"))
        .expect("the ack");
    assert_eq!(ack.text, "MAV_CMD_COMPONENT_ARM_DISARM: accepted");
    assert_eq!(ack.severity, Severity::Info);
    assert!(!ack.severity.is_urgent());
}

#[test]
fn a_vehicle_is_discovered_and_its_state_published() {
    let (gcs_side, mut vehicle_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    for seq in 0..5 {
        vehicle_side.write_all(&vehicle_heartbeat(seq)).unwrap();
    }

    wait_for("vehicle discovery", || !link.vehicles().is_empty());
    let (id, handle) = link.primary_vehicle().expect("a vehicle");
    assert_eq!(id, VehicleId::new(1, 1));

    wait_for("state publication", || handle.load().messages_applied > 0);
    let state = handle.load();
    assert_eq!(state.vehicle_type, 2);
    assert_eq!(state.autopilot, 3);
    assert!(
        !state.armed,
        "base_mode {BASE_MODE_DISARMED} does not set the armed flag"
    );

    // Now arm, and confirm the flag is tracked rather than assumed.
    vehicle_side
        .write_all(&vehicle_heartbeat_with_mode(
            6,
            BASE_MODE_DISARMED | ARMED_FLAG,
        ))
        .unwrap();
    wait_for("armed state", || handle.load().armed);
    assert!(handle.load().armed);
}

#[test]
fn the_link_announces_itself_with_heartbeats() {
    // A vehicle that hears no GCS heartbeat eventually declares failsafe, so this is not
    // cosmetic.
    let (gcs_side, mut vehicle_side) = Loopback::pair();
    let config = LinkConfig {
        heartbeat_interval: Duration::from_millis(20),
        stream_rate_hz: 0,
        ..LinkConfig::default()
    };
    let link = Link::from_transport(Box::new(gcs_side), config);

    let mut decoder = FrameDecoder::new();
    let mut seen_gcs_heartbeat = false;
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut buf = [0u8; 512];

    while Instant::now() < deadline && !seen_gcs_heartbeat {
        let n = vehicle_side.read(&mut buf).unwrap();
        if n > 0 {
            decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                if let Some(MavMessage::Heartbeat(hb)) =
                    MavMessage::decode(frame.msgid, frame.payload)
                {
                    assert_eq!(frame.sysid, 255, "default GCS system id");
                    assert_eq!(hb.r#type, 6, "MAV_TYPE_GCS");
                    seen_gcs_heartbeat = true;
                }
            });
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(seen_gcs_heartbeat, "the link must send heartbeats");
    drop(link);
}

#[test]
fn telemetry_streams_are_requested_when_a_vehicle_appears() {
    // ArduPilot sends almost nothing until asked. Forgetting this makes a working link look
    // like a dead one, so it is worth a test of its own.
    let (gcs_side, mut vehicle_side) = Loopback::pair();
    let config = LinkConfig {
        stream_rate_hz: 4,
        ..LinkConfig::default()
    };
    let link = Link::from_transport(Box::new(gcs_side), config);

    vehicle_side.write_all(&vehicle_heartbeat(0)).unwrap();

    let mut decoder = FrameDecoder::new();
    let mut requested_streams = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut buf = [0u8; 1024];

    while Instant::now() < deadline && requested_streams.len() < 7 {
        let n = vehicle_side.read(&mut buf).unwrap();
        if n > 0 {
            decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                if let Some(MavMessage::RequestDataStream(req)) =
                    MavMessage::decode(frame.msgid, frame.payload)
                {
                    assert_eq!(req.target_system, 1);
                    assert_eq!(req.req_message_rate, 4);
                    assert_eq!(req.start_stop, 1);
                    requested_streams.push(req.req_stream_id);
                }
            });
        }
        std::thread::sleep(Duration::from_millis(5));
    }

    assert_eq!(
        requested_streams.len(),
        7,
        "all standard streams must be requested"
    );
    drop(link);
}

#[test]
fn closing_the_link_stops_its_thread() {
    let (gcs_side, _vehicle_side) = Loopback::pair();
    let mut link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());
    assert!(link.is_running());
    link.close();
    assert!(!link.is_running());
}

#[test]
fn an_idle_link_does_not_burn_cpu() {
    // The I/O loop polls a transport that returns immediately when idle. Without a yield it
    // spins a core flat, which would blow the idle-CPU budget the project sets.
    let (gcs_side, _vehicle_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    let before = cpu_time();
    std::thread::sleep(Duration::from_millis(500));
    let used = cpu_time() - before;

    drop(link);
    assert!(
        used < Duration::from_millis(100),
        "idle link used {used:?} of CPU over 500ms of wall clock"
    );
}

/// Process CPU time, used to assert the idle loop is not spinning.
fn cpu_time() -> Duration {
    // /proc/self/stat fields 14 and 15 are utime and stime in clock ticks.
    let Ok(stat) = std::fs::read_to_string("/proc/self/stat") else {
        return Duration::ZERO;
    };
    let Some(after_comm) = stat.rsplit(')').next() else {
        return Duration::ZERO;
    };
    let fields: Vec<&str> = after_comm.split_whitespace().collect();
    let ticks: u64 = fields
        .get(11)
        .and_then(|v| v.parse::<u64>().ok())
        .unwrap_or(0)
        + fields
            .get(12)
            .and_then(|v| v.parse::<u64>().ok())
            .unwrap_or(0);
    Duration::from_secs_f64(ticks as f64 / 100.0)
}

/// Real-vehicle test, opt in with `cargo test -p mp-link -- --ignored`.
///
/// Not run by default because it needs an ArduPilot SITL binary, but deliberately `#[ignore]`d
/// rather than silently skipped: the test output says it exists and was not run.
#[test]
#[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5760"]
fn talks_to_a_real_ardupilot_sitl() {
    let link = Link::connect("tcp:127.0.0.1:5760", LinkConfig::default())
        .expect("SITL must be listening on 5760");

    wait_for("vehicle discovery", || !link.vehicles().is_empty());
    let (_, handle) = link.primary_vehicle().expect("a vehicle");

    // Telemetry only flows if our stream request worked.
    wait_for("GPS fix", || handle.load().gps.fix_type >= 3);
    wait_for("battery telemetry", || handle.load().battery.voltage > 1.0);
    wait_for("position", || handle.load().position.is_some());

    let state = handle.load();
    let position = state.position.expect("position");
    assert!(
        (-36.0..=-35.0).contains(&position.latitude()),
        "SITL defaults to CMAC; got {}",
        position.latitude()
    );
    assert_eq!(
        link.stats().decode.crc_errors,
        0,
        "a local link must be clean"
    );

    // Sustained rate, not a total: reaching a fix proves one message arrived, whereas a GCS
    // needs the stream to keep coming. Sample a window rather than asserting a magic count,
    // which would only measure how fast the earlier waits happened to complete.
    let before = link.frames_received();
    std::thread::sleep(Duration::from_secs(2));
    let rate = (link.frames_received() - before) / 2;
    assert!(
        rate >= 10,
        "expected a sustained stream, got {rate} frames/s"
    );
    assert!(
        handle.load().link.loss_percent() < 1.0,
        "loopback-quality link should not lose packets: {}%",
        handle.load().link.loss_percent()
    );
}
