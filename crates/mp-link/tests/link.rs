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

//! Link behaviour against an in-memory vehicle, plus an opt-in test against real SITL.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use web_time::{Duration, Instant};

use mp_link::messages::Severity;
use mp_link::{Link, LinkConfig};
use mp_mavlink::{FrameDecoder, Message as _, encode_v2, encode_v2_targeted};
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
        wasm_thread::sleep(Duration::from_millis(5));
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

/// A heartbeat from a chosen system and component.
fn heartbeat_from(seq: u8, sysid: u32, compid: u8) -> Vec<u8> {
    let hb = Heartbeat {
        custom_mode: 0,
        r#type: 2,
        autopilot: 3,
        base_mode: BASE_MODE_DISARMED,
        system_status: 3,
        mavlink_version: 3,
    };
    let mut payload = [0u8; Heartbeat::LEN];
    hb.encode(&mut payload);
    let mut frame = [0u8; 64];
    let n = encode_v2(
        &mut frame,
        seq,
        sysid,
        compid,
        Heartbeat::ID,
        &payload,
        Heartbeat::CRC_EXTRA,
        0,
    )
    .unwrap();
    frame[..n].to_vec()
}

#[test]
fn two_vehicles_on_one_link_are_tracked_separately() {
    // A ground station on a shared radio hears every aircraft on it. Merging them would show one
    // vehicle's attitude under another's name, and send commands to whichever was heard from
    // first.
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    vehicle_side.write_all(&heartbeat_from(0, 1, 1)).unwrap();
    vehicle_side.write_all(&heartbeat_from(1, 2, 1)).unwrap();

    wait_for("both vehicles", || link.vehicles().len() >= 2);

    let vehicles = link.vehicles();
    assert!(
        vehicles.contains(&VehicleId {
            sysid: 1,
            compid: 1
        }),
        "{vehicles:?}"
    );
    assert!(
        vehicles.contains(&VehicleId {
            sysid: 2,
            compid: 1
        }),
        "{vehicles:?}"
    );

    // Each is individually addressable, which is what a vehicle picker relies on.
    for id in vehicles {
        let handle = link.vehicle(id).expect("a handle per vehicle");
        let state = handle.load();
        assert_eq!(
            state.sysid, id.sysid,
            "a handle returned another vehicle's state"
        );
    }
}

/// `VehicleAndInspectorKeysDoNotAlias` and the first half of
/// `LiveLinkReadsFragmentedFramesAndTargetsWithoutAliasing`: vehicles 1, 257, 0x80000001,
/// 0xffffff01 and 0xffffffff - all but the first with 32-bit ids - heard a byte at a time, each a
/// vehicle of its own under its whole id, with its own state, none taken for another whose low
/// byte it shares. `// C#: MissionPlannerTests/Mavlink/Sysid32Tests.cs:156-199`
#[test]
fn vehicles_with_32_bit_ids_are_each_their_own() {
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());
    let ids = [1, 257, 0x8000_0001, 0xffff_ff01, u32::MAX];
    for (seq, id) in ids.into_iter().enumerate() {
        for byte in heartbeat_from(u8::try_from(seq).unwrap(), id, 1) {
            vehicle_side.write_all(&[byte]).unwrap();
        }
    }
    wait_for("every vehicle", || link.vehicles().len() >= ids.len());
    let vehicles = link.vehicles();
    for id in ids {
        let vehicle = VehicleId::new(id, 1);
        assert!(vehicles.contains(&vehicle), "{id:#x}: {vehicles:?}");
        let state = link.vehicle(vehicle).expect("a handle per vehicle").load();
        assert_eq!(state.sysid, id, "another vehicle's state under {id:#x}");
    }
    assert_eq!(vehicles.len(), ids.len(), "{vehicles:?}");
}

/// What the link writes, read off the vehicle's end until `enough` says so or five seconds pass:
/// each frame's sender, header target, message id and payload.
type Sent = (u32, Option<u32>, u32, Vec<u8>);
fn read_sent(
    vehicle_side: &mut dyn Transport,
    mut enough: impl FnMut(&[Sent]) -> bool,
) -> Vec<Sent> {
    let mut decoder = FrameDecoder::new();
    let mut sent = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut buf = [0u8; 512];
    while Instant::now() < deadline && !enough(&sent) {
        let n = vehicle_side.read(&mut buf).unwrap();
        decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
            sent.push((
                frame.sysid,
                frame.target_system,
                frame.msgid,
                frame.payload.to_vec(),
            ));
        });
        wasm_thread::sleep(Duration::from_millis(5));
    }
    sent
}

/// `IsTargetedTo`, as `WideHeaderTargetKeepsPayloadComponent` holds it: a system and component of
/// its own, 0, or none at all is this ground station; anything else is not.
/// `// C#: MissionPlannerTests/Mavlink/Sysid32Tests.cs:112-129`
#[test]
fn what_is_addressed_to_this_ground_station() {
    let us = VehicleId::new(255, 190);
    assert!(mp_link::is_targeted_to(Some(255), Some(190), us));
    assert!(mp_link::is_targeted_to(Some(0), Some(0), us));
    assert!(mp_link::is_targeted_to(None, None, us));
    assert!(!mp_link::is_targeted_to(Some(254), Some(190), us));
    assert!(!mp_link::is_targeted_to(Some(255), Some(12), us));
    let wide = VehicleId::new(0x8000_0000, 12);
    assert!(mp_link::is_targeted_to(Some(0x8000_0000), Some(12), wide));
    assert!(!mp_link::is_targeted_to(Some(255), Some(12), wide));
}

/// The rest of `LiveLinkReadsFragmentedFramesAndTargetsWithoutAliasing`: a command whose header
/// names another receiver, 0xfffffffe, is not this ground station's and goes no further - not
/// to the inspector, not into the vehicle's state - while the next frame from the same vehicle
/// does. `// C#: MissionPlannerTests/Mavlink/Sysid32Tests.cs:200-204;
/// ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5072-5076`
#[test]
fn a_frame_whose_header_names_another_receiver_goes_no_further() {
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());
    let heard = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let sink = std::sync::Arc::clone(&heard);
    let _subscription = link.on_packet(move |packet| {
        if !packet.sent {
            sink.lock().unwrap().push((packet.sysid, packet.msgid));
        }
    });
    let source = 0xffff_ff01;
    let command = mp_link::commands::command_long(VehicleId::new(0xffff_fffe, 190), 300, [0.0; 7]);
    let mut payload = [0u8; 255];
    let len = command.encode(&mut payload);
    let mut frame = [0u8; mp_mavlink::MAX_FRAME_LEN];
    let n = encode_v2_targeted(
        &mut frame,
        0,
        source,
        1,
        command.id(),
        &payload[..len],
        command.crc_extra(),
        0,
        Some(0xffff_fffe),
    )
    .unwrap();
    vehicle_side.write_all(&frame[..n]).unwrap();
    vehicle_side
        .write_all(&heartbeat_from(1, source, 1))
        .unwrap();
    wait_for("the heartbeat after it", || {
        heard.lock().unwrap().contains(&(source, 0))
    });
    assert!(
        !heard.lock().unwrap().contains(&(source, 76)),
        "the command for another receiver reached the inspector"
    );
}

/// The send half: a command to vehicle 0xffffffff goes from this ground station (255) with the
/// whole id in its header, `TARGET32`, and 255 in its payload's byte - `SetPayloadTarget` - as the
/// C#'s `sendPacket(..., uint.MaxValue, 1)` writes it.
/// `// C#: MissionPlannerTests/Mavlink/Sysid32Tests.cs:206-209`
#[test]
fn a_command_to_a_32_bit_vehicle_carries_its_id_in_the_header() {
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let config = LinkConfig {
        stream_rate_hz: 0,
        send_heartbeat: false,
        ..LinkConfig::default()
    };
    let link = Link::from_transport(Box::new(gcs_side), config);
    let wide = VehicleId::new(u32::MAX, 1);
    vehicle_side
        .write_all(&heartbeat_from(0, wide.sysid, 1))
        .unwrap();
    wait_for("the vehicle", || link.vehicles().contains(&wide));
    assert!(link.send(&mp_link::commands::command_long(wide, 300, [0.0; 7])));
    let sent = read_sent(&mut vehicle_side, |sent| {
        sent.iter().any(|frame| frame.2 == 76)
    });
    let (sysid, target, _, payload) = sent
        .iter()
        .find(|frame| frame.2 == 76)
        .expect("the command went");
    assert_eq!(*sysid, 255);
    assert_eq!(*target, Some(u32::MAX));
    // COMMAND_LONG's target_system, after its seven floats and the command.
    assert_eq!(payload[30], 255);
}

/// A ground station whose own id is over 255 writes it four bytes wide, with `SYSID32` - its
/// heartbeats, written by the link thread, and a command queued from outside it, whose checksum
/// is worked out again after its sequence number is put in.
#[test]
fn a_ground_station_with_a_32_bit_id_writes_it_four_bytes_wide() {
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let config = LinkConfig {
        sysid: 0x0001_0000,
        heartbeat_interval: Duration::from_millis(20),
        stream_rate_hz: 0,
        ..LinkConfig::default()
    };
    let link = Link::from_transport(Box::new(gcs_side), config);
    assert!(link.send(&mp_link::commands::command_long(
        VehicleId::new(1, 1),
        300,
        [0.0; 7]
    )));
    let sent = read_sent(&mut vehicle_side, |sent| {
        sent.iter().any(|frame| frame.2 == 0) && sent.iter().any(|frame| frame.2 == 76)
    });
    for message in [0, 76] {
        let (sysid, ..) = sent
            .iter()
            .find(|frame| frame.2 == message)
            .unwrap_or_else(|| panic!("message {message} went, checksum and all"));
        assert_eq!(*sysid, 0x0001_0000, "message {message}");
    }
}

#[test]
fn the_primary_vehicle_is_the_autopilot_not_whatever_spoke_first() {
    // A gimbal or a companion computer announces itself on the same link. Treating the first
    // thing heard as the aircraft would point the whole application at a camera mount.
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    // Component 154 is a gimbal, and it speaks first.
    vehicle_side.write_all(&heartbeat_from(0, 1, 154)).unwrap();
    vehicle_side.write_all(&heartbeat_from(1, 1, 1)).unwrap();

    wait_for("both components", || link.vehicles().len() >= 2);

    let (primary, _) = link.primary_vehicle().expect("a primary vehicle");
    assert_eq!(
        primary.compid, 1,
        "the autopilot should be primary, got {primary:?}"
    );
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
        wasm_thread::sleep(Duration::from_millis(5));
    }
    assert!(seen_gcs_heartbeat, "the link must send heartbeats");
    drop(link);
}

#[test]
fn telemetry_streams_are_requested_when_a_vehicle_appears() {
    // ArduPilot sends almost nothing until asked. Forgetting this makes a working link look
    // like a dead one, so it is worth a test of its own. The rates are the vehicle's own, which
    // start from the saved defaults; each request goes twice, in `UpdateCurrentSettings`' order
    // (ExtLibs/ArduPilot/CurrentState.cs:4635-4666; MAVLinkInterface.cs:3258-3259).
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

    while Instant::now() < deadline && requested_streams.len() < 14 {
        let n = vehicle_side.read(&mut buf).unwrap();
        if n > 0 {
            decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                if let Some(MavMessage::RequestDataStream(req)) =
                    MavMessage::decode(frame.msgid, frame.payload)
                {
                    assert_eq!(req.target_system, 1);
                    assert_eq!(req.start_stop, 1);
                    requested_streams.push((req.req_stream_id, req.req_message_rate));
                }
            });
        }
        wasm_thread::sleep(Duration::from_millis(5));
    }

    let rates = mp_vehicle::StreamRates::backups();
    let rate = |hz: i32| u16::try_from(hz).unwrap();
    let expected: Vec<(u8, u16)> = [
        (2, rate(rates.status)),
        (6, rate(rates.position)),
        (10, rate(rates.attitude)),
        (11, rate(rates.attitude)),
        (12, rate(rates.sensors)),
        (1, rate(rates.sensors)),
        (3, rate(rates.rc)),
    ]
    .into_iter()
    .flat_map(|request| [request, request])
    .collect();
    assert_eq!(
        requested_streams, expected,
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
    wasm_thread::sleep(Duration::from_millis(500));
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
    wasm_thread::sleep(Duration::from_secs(2));
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

#[test]
fn a_sender_handle_puts_frames_on_the_wire_from_another_thread() {
    // The joystick reader has to send from a thread of its own - Deliverable 15's 5 ms from stick to wire
    // is not reachable through a UI timer - and it cannot borrow the link across threads. The
    // handle carries copies of what `send` needs and nothing else.
    let (gcs_side, mut vehicle_side) = Loopback::pair();
    let config = LinkConfig {
        send_heartbeat: false,
        stream_rate_hz: 0,
        ..LinkConfig::default()
    };
    let link = Link::from_transport(Box::new(gcs_side), config);
    let sender = link.sender();
    let target = VehicleId {
        sysid: 1,
        compid: 1,
    };
    let mut channels = [0u16; 18];
    channels[0] = 1500;
    channels[2] = 1100;

    let worker =
        wasm_thread::spawn(move || sender.send(&mp_link::commands::rc_override(target, channels)));
    assert!(
        worker.join().expect("the sending thread finished"),
        "the link is running, so the frame is queued"
    );

    let mut decoder = FrameDecoder::new();
    let mut seen = None;
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut buf = [0u8; 512];
    while Instant::now() < deadline && seen.is_none() {
        let n = vehicle_side.read(&mut buf).unwrap();
        if n > 0 {
            decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                if let Some(MavMessage::RcChannelsOverride(rc)) =
                    MavMessage::decode(frame.msgid, frame.payload)
                {
                    assert_eq!(frame.sysid, 255, "framed with the link's own system id");
                    assert_eq!(frame.compid, LinkConfig::default().compid);
                    seen = Some(rc);
                }
            });
        }
        wasm_thread::sleep(Duration::from_millis(5));
    }
    let rc = seen.expect("the override should reach the wire");
    assert_eq!(rc.chan1_raw, 1500);
    assert_eq!(rc.chan3_raw, 1100);
    assert_eq!(rc.target_system, 1);

    // Once the link is gone the handle says so rather than pretending. For a release frame
    // that is the difference between an aircraft handed back and one still being flown by a
    // stick nobody is holding, and the caller acts on it.
    let after = link.sender();
    drop(link);
    let deadline = Instant::now() + Duration::from_secs(2);
    let mut refused = false;
    while Instant::now() < deadline && !refused {
        refused = !after.send(&mp_link::commands::rc_override(target, channels));
        wasm_thread::sleep(Duration::from_millis(5));
    }
    assert!(
        refused,
        "a handle to a stopped link must report the send as not delivered"
    );
}
