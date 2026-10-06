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

//! The camera and gimbal manager a component's first heartbeat makes, over a real link: two
//! seconds on, `Discover` asks everyone for `GIMBAL_MANAGER_INFORMATION` and `StartID` asks the
//! camera for its information - the deprecated request after a refusal, then the video streams -
//! and what the vehicle then says is kept where the gimbal video control reads it.
//! `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:500-586; CameraProtocol.cs:212-298;
//! GimbalManagerProtocol.cs:39-84`

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use web_time::{Duration, Instant};

use mp_link::{Link, LinkConfig};
use mp_mavlink::{FrameDecoder, encode_v2};
use mp_mavlink_dialects::all::{
    CommandAck, CommandLong, DIALECT, GimbalManagerInformation, Heartbeat, MavMessage,
    VideoStreamInformation,
};
use mp_transport::Transport;
use mp_transport::testing::{Loopback, LoopbackEnd};
use mp_vehicle::VehicleId;

const VEHICLE: VehicleId = VehicleId::new(1, 1);

fn frame(seq: u8, message: &MavMessage) -> Vec<u8> {
    let mut payload = [0u8; 255];
    let len = message.encode(&mut payload);
    let mut out = [0u8; mp_mavlink::MAX_FRAME_LEN];
    let n = encode_v2(
        &mut out,
        seq,
        VEHICLE.sysid,
        VEHICLE.compid,
        message.id(),
        &payload[..len],
        message.crc_extra(),
        0,
    )
    .unwrap();
    out[..n].to_vec()
}

fn heartbeat() -> MavMessage {
    MavMessage::Heartbeat(Heartbeat {
        custom_mode: 0,
        r#type: 2,
        autopilot: 3,
        base_mode: 81,
        system_status: 3,
        mavlink_version: 3,
    })
}

/// The commands the link sent, waiting up to `within` for `until` to be among them.
fn commands_until(
    end: &mut LoopbackEnd,
    decoder: &mut FrameDecoder,
    seen: &mut Vec<CommandLong>,
    within: Duration,
    until: impl Fn(&CommandLong) -> bool,
) -> bool {
    let deadline = Instant::now() + within;
    let mut buf = [0u8; 4096];
    while Instant::now() < deadline {
        let n = end.read(&mut buf).unwrap_or(0);
        decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
            if let Some(MavMessage::CommandLong(long)) =
                MavMessage::decode(frame.msgid, frame.payload)
            {
                seen.push(long);
            }
        });
        if seen.iter().any(&until) {
            return true;
        }
        wasm_thread::sleep(Duration::from_millis(5));
    }
    false
}

fn requested(message: f32) -> impl Fn(&CommandLong) -> bool {
    move |c| c.command == 512 && (c.param1 - message).abs() < f32::EPSILON
}

#[test]
fn a_heartbeat_makes_a_camera_and_a_gimbal_manager_that_discover_two_seconds_on() {
    let (mut vehicle, gcs) = Loopback::pair();
    let config = LinkConfig {
        send_heartbeat: false,
        stream_rate_hz: 0,
        ..LinkConfig::default()
    };
    let link = Link::from_transport(Box::new(gcs), config);
    let mut decoder = FrameDecoder::new();
    let mut seen = Vec::new();
    let mut seq = 0u8;
    let mut send = |end: &mut LoopbackEnd, message: MavMessage| {
        end.write_all(&frame(seq, &message)).unwrap();
        seq = seq.wrapping_add(1);
    };

    let heard = Instant::now();
    send(&mut vehicle, heartbeat());
    // Made at once, not yet started: nothing asked, nothing kept.
    let deadline = Instant::now() + Duration::from_secs(5);
    while link.camera(VEHICLE).is_none() && Instant::now() < deadline {
        wasm_thread::sleep(Duration::from_millis(5));
    }
    let camera = link.camera(VEHICLE).expect("a camera for the autopilot");
    assert!(!camera.is_started());
    assert!(
        !link
            .gimbal_manager(VEHICLE)
            .expect("a manager")
            .is_listening()
    );
    assert!(!link.request_camera_information(VEHICLE), "not started");

    // Discover: GIMBAL_MANAGER_INFORMATION from everyone, two seconds after the heartbeat.
    assert!(commands_until(
        &mut vehicle,
        &mut decoder,
        &mut seen,
        Duration::from_secs(5),
        requested(280.0)
    ));
    assert!(
        heard.elapsed() >= Duration::from_millis(1900),
        "{:?}",
        heard.elapsed()
    );
    let discover = seen.iter().find(|c| requested(280.0)(c)).unwrap();
    assert_eq!((discover.target_system, discover.target_component), (0, 0));

    // StartID: CAMERA_INFORMATION from the camera's component, waited on.
    assert!(commands_until(
        &mut vehicle,
        &mut decoder,
        &mut seen,
        Duration::from_secs(2),
        requested(259.0)
    ));
    assert!(link.camera_information_pending(VEHICLE));
    // Refused: REQUEST_CAMERA_INFORMATION, then the video streams.
    send(
        &mut vehicle,
        MavMessage::CommandAck(CommandAck {
            command: 512,
            result: 3,
            progress: 0,
            result_param2: 0,
            target_system: 255,
            target_component: 190,
        }),
    );
    assert!(commands_until(
        &mut vehicle,
        &mut decoder,
        &mut seen,
        Duration::from_secs(2),
        requested(269.0)
    ));
    assert!(
        seen.iter().any(|c| c.command == 521),
        "the deprecated request"
    );
    assert!(!link.camera_information_pending(VEHICLE));

    // What the vehicle then says is kept.
    send(
        &mut vehicle,
        MavMessage::GimbalManagerInformation(GimbalManagerInformation {
            time_boot_ms: 0,
            cap_flags: 32,
            roll_min: 0.0,
            roll_max: 0.0,
            pitch_min: -1.5,
            pitch_max: 0.5,
            yaw_min: -3.0,
            yaw_max: 3.0,
            gimbal_device_id: 1,
        }),
    );
    let mut uri = [0u8; 160];
    uri[..4].copy_from_slice(b"5600");
    send(
        &mut vehicle,
        MavMessage::VideoStreamInformation(VideoStreamInformation {
            framerate: 30.0,
            bitrate: 0,
            flags: 0,
            resolution_h: 1280,
            resolution_v: 720,
            rotation: 0,
            hfov: 60,
            stream_id: 1,
            count: 1,
            r#type: 1,
            name: [0; 32],
            uri,
            encoding: 1,
            camera_device_id: 0,
        }),
    );
    let deadline = Instant::now() + Duration::from_secs(5);
    while link.video_streams().is_empty() && Instant::now() < deadline {
        wasm_thread::sleep(Duration::from_millis(5));
    }
    let streams = link.video_streams();
    assert_eq!(streams.len(), 1);
    assert_eq!(streams[0].0, (1, 1, 1));
    let manager = link.gimbal_manager(VEHICLE).expect("a manager");
    assert!(manager.is_listening());
    assert_eq!(manager.manager_info[&0].cap_flags, 32);
    assert!(link.camera(VEHICLE).unwrap().is_started());
}
