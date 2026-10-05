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

//! Fifty vehicles through one link (DELIVERABLES.md Deliverable 4: "multi-vehicle `sysid/compid` routing
//! with N ≥ 50 simultaneous vehicles").
//!
//! One shared radio, or one UDP port fed by a swarm's router, carries every aircraft on it, and
//! each aircraft carries more than one component: the autopilot, a gimbal, a companion computer.
//! Mission Planner keys everything on the pair (`MAVlist[sysid, compid]`,
//! ExtLibs/ArduPilot/Mavlink/MAVList.cs). Here that is checked end to end through the real link
//! thread: every component is tracked on its own, the primary vehicle is an autopilot, anything
//! sent is addressed to the one vehicle it is for, and nothing one vehicle says - telemetry,
//! parameters, acknowledgements, mission items - lands on another.
//!
//! It also measures what one frame costs the link thread with 56 components live, and prints it,
//! so a change that makes routing scale with the number of vehicles shows up as a number.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::collections::{BTreeMap, VecDeque};
use web_time::{Duration, Instant};

use mp_link::mission_transfer::TransferState;
use mp_link::param_download::ParamDownloadState;
use mp_link::requests::RequestOutcome;
use mp_link::{Link, LinkConfig, ProtocolTimeouts, commands};
use mp_mavlink::{FrameDecoder, encode_v2};
use mp_mavlink_dialects::all::{
    Attitude, CommandAck, DIALECT, GlobalPositionInt, Heartbeat, MavMessage, ParamValue,
};
use mp_mission::{MISSION_TYPE_MISSION, MissionItem};
use mp_transport::Transport;
use mp_transport::testing::{Loopback, LoopbackEnd};
use mp_vehicle::VehicleId;

/// Autopilots, one per system id.
const AUTOPILOTS: u8 = 50;
/// `MAV_COMP_ID_GIMBAL`.
const GIMBAL: u8 = 154;
/// `MAV_COMP_ID_ONBOARD_COMPUTER`.
const COMPANION: u8 = 191;
/// Systems that also carry a gimbal and a companion computer.
const WITH_PAYLOAD: [u8; 3] = [10, 20, 30];
/// Telemetry rounds: each autopilot sends a heartbeat, an attitude and a position per round.
const ROUNDS: u16 = 40;
/// `MAV_CMD_NAV_TAKEOFF`.
const TAKEOFF: u16 = 22;
/// The link's own address.
const GCS: VehicleId = VehicleId::new(255, 190);
/// How long anything may take before the test calls it hung.
const HUNG: Duration = Duration::from_secs(5);

/// Every component on the link.
fn components() -> Vec<VehicleId> {
    let mut all: Vec<VehicleId> = (1..=AUTOPILOTS)
        .map(|sysid| VehicleId::new(sysid, 1))
        .collect();
    for sysid in WITH_PAYLOAD {
        all.push(VehicleId::new(sysid, GIMBAL));
        all.push(VehicleId::new(sysid, COMPANION));
    }
    all
}

/// The far end of the link: every vehicle at once, each numbering its own frames.
struct Swarm {
    end: LoopbackEnd,
    decoder: FrameDecoder,
    /// Each component's own sequence counter, so each one's link quality is measurable.
    seq: BTreeMap<VehicleId, u8>,
    inbox: VecDeque<(u8, u8, MavMessage)>,
}

impl Swarm {
    fn frame(&mut self, from: VehicleId, message: &MavMessage, out: &mut Vec<u8>) {
        let seq = self.seq.entry(from).or_insert(0);
        let mut payload = [0u8; 255];
        let len = message.encode(&mut payload);
        let mut frame = [0u8; mp_mavlink::MAX_FRAME_LEN];
        let n = encode_v2(
            &mut frame,
            *seq,
            from.sysid,
            from.compid,
            message.id(),
            &payload[..len],
            message.crc_extra(),
            0,
        )
        .unwrap();
        *seq = seq.wrapping_add(1);
        out.extend_from_slice(&frame[..n]);
    }

    fn send(&mut self, from: VehicleId, message: &MavMessage) {
        let mut bytes = Vec::new();
        self.frame(from, message, &mut bytes);
        self.end.write_all(&bytes).unwrap();
    }

    /// Everything the link has sent so far, with the sender's ids from the frame header.
    fn pump(&mut self) {
        let mut buf = [0u8; 4096];
        loop {
            let n = self.end.read(&mut buf).unwrap_or(0);
            if n == 0 {
                return;
            }
            let Self { decoder, inbox, .. } = self;
            decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                if let Some(message) = MavMessage::decode(frame.msgid, frame.payload) {
                    inbox.push_back((frame.sysid, frame.compid, message));
                }
            });
        }
    }

    /// Everything the link sends in the next `window`.
    fn collect(&mut self, window: Duration) -> Vec<(u8, u8, MavMessage)> {
        let deadline = Instant::now() + window;
        while Instant::now() < deadline {
            self.pump();
            wasm_thread::sleep(Duration::from_millis(1));
        }
        self.pump();
        self.inbox.drain(..).collect()
    }
}

fn heartbeat(component: VehicleId) -> MavMessage {
    MavMessage::Heartbeat(Heartbeat {
        custom_mode: 0,
        // Quadrotor for an autopilot; gimbal and onboard controller for the rest.
        r#type: match component.compid {
            1 => 2,
            GIMBAL => 26,
            _ => 18,
        },
        autopilot: if component.compid == 1 { 3 } else { 8 },
        base_mode: 81,
        system_status: 4,
        mavlink_version: 3,
    })
}

/// A roll each vehicle alone reports, so a snapshot showing another's is caught.
fn roll_of(sysid: u8, round: u16) -> f32 {
    f32::from(sysid) / 100.0 + f32::from(round) / 10_000.0
}

fn attitude(sysid: u8, round: u16) -> MavMessage {
    MavMessage::Attitude(Attitude {
        time_boot_ms: u32::from(round),
        roll: roll_of(sysid, round),
        pitch: 0.0,
        yaw: 0.0,
        rollspeed: 0.0,
        pitchspeed: 0.0,
        yawspeed: 0.0,
    })
}

/// Each vehicle a hundred metres further north than the one before.
fn latitude_e7(sysid: u8) -> i32 {
    -353_632_620 + i32::from(sysid) * 9_000
}

fn position(sysid: u8, round: u16) -> MavMessage {
    MavMessage::GlobalPositionInt(GlobalPositionInt {
        time_boot_ms: u32::from(round),
        lat: latitude_e7(sysid),
        lon: 1_491_652_370,
        alt: 600_000,
        relative_alt: 10_000,
        vx: 0,
        vy: 0,
        vz: 0,
        hdg: 18_000,
    })
}

fn param(name: &str, value: f32, index: u16, count: u16) -> MavMessage {
    MavMessage::ParamValue(ParamValue {
        param_value: value,
        param_count: count,
        param_index: index,
        param_id: mp_params::encode_param_id(name),
        param_type: 9,
    })
}

fn wait_for(what: &str, mut check: impl FnMut() -> bool) {
    let deadline = Instant::now() + HUNG;
    while !check() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        wasm_thread::sleep(Duration::from_millis(1));
    }
}

/// A link, and the whole swarm on its far end. Stream requests are left on, as Mission Planner
/// sends them, because who they are addressed to is one of the things checked.
fn swarm() -> (Link, Swarm) {
    let (vehicle_side, gcs_side) = Loopback::pair();
    let config = LinkConfig {
        send_heartbeat: false,
        timeouts: ProtocolTimeouts::default().faster(10),
        ..LinkConfig::default()
    };
    let link = Link::from_transport(Box::new(gcs_side), config);
    let swarm = Swarm {
        end: vehicle_side,
        decoder: FrameDecoder::new(),
        seq: BTreeMap::new(),
        inbox: VecDeque::new(),
    };
    (link, swarm)
}

#[test]
fn fifty_vehicles_through_one_link_are_routed_apart() {
    let (link, mut swarm) = swarm();
    let all = components();
    assert_eq!(all.len(), 56);

    // A gimbal speaks first: the primary vehicle must still be an autopilot.
    let gimbal = VehicleId::new(WITH_PAYLOAD[0], GIMBAL);
    swarm.send(gimbal, &heartbeat(gimbal));

    // --- telemetry, as fast as the far end can write it ----------------------------------------
    // Built up front so the measurement is the link's work, not the encoder's.
    let mut rounds: Vec<Vec<u8>> = Vec::new();
    let mut frames = 1u64;
    for round in 0..ROUNDS {
        let mut bytes = Vec::new();
        for component in &all {
            swarm.frame(*component, &heartbeat(*component), &mut bytes);
            frames += 1;
            if component.compid == 1 {
                swarm.frame(*component, &attitude(component.sysid, round), &mut bytes);
                swarm.frame(*component, &position(component.sysid, round), &mut bytes);
                frames += 2;
            }
        }
        rounds.push(bytes);
    }
    let started = Instant::now();
    for bytes in &rounds {
        swarm.end.write_all(bytes).unwrap();
    }
    wait_for("every frame", || link.frames_received() >= frames);
    let took = started.elapsed();
    let per_frame = took / u32::try_from(frames).unwrap();
    eprintln!(
        "routing: {frames} frames from {} components in {took:?}: {per_frame:?} per frame",
        all.len()
    );
    // Generous, because this runs on shared machines in debug builds; what it catches is a
    // change that makes a frame's cost grow with the vehicle count, which is tens of times this.
    assert!(
        per_frame < Duration::from_micros(500),
        "{per_frame:?} per frame with {} components on the link",
        all.len()
    );

    // --- every component tracked on its own ----------------------------------------------------
    wait_for("every component published", || {
        link.vehicles().len() == all.len()
    });
    let mut seen = link.vehicles();
    seen.sort();
    let mut expected = all.clone();
    expected.sort();
    assert_eq!(seen, expected);

    let last = ROUNDS - 1;
    for component in &all {
        let handle = link.vehicle(*component).unwrap();
        wait_for("the last round published", || {
            let state = handle.load();
            let per_round = if component.compid == 1 { 3 } else { 1 };
            let first = u64::from(*component == gimbal);
            state.messages_applied == first + u64::from(ROUNDS) * per_round
        });
        let state = handle.load();
        assert_eq!(
            (state.sysid, state.compid),
            (component.sysid, component.compid),
            "a handle returned another component's state"
        );
        assert!(
            state.link.loss_percent() < f64::EPSILON,
            "{component}: each component's sequence is its own; {}% loss",
            state.link.loss_percent()
        );
        if component.compid == 1 {
            assert_eq!(state.autopilot, 3);
            let roll = state.attitude.roll.0;
            assert!(
                (roll - f64::from(roll_of(component.sysid, last))).abs() < 1e-6,
                "{component}: roll {roll} belongs to another vehicle"
            );
            let latitude = state.position.unwrap().latitude();
            assert!(
                (latitude - f64::from(latitude_e7(component.sysid)) / 1e7).abs() < 1e-9,
                "{component}: latitude {latitude} belongs to another vehicle"
            );
        } else {
            assert_eq!(state.autopilot, 8, "{component}: MAV_AUTOPILOT_INVALID");
            assert!(state.position.is_none(), "{component} was given a position");
        }
    }

    // --- the primary vehicle is an autopilot, not whatever spoke first --------------------------
    let (primary, _) = link.primary_vehicle().unwrap();
    assert_eq!(primary, VehicleId::new(1, 1));

    // --- streams were asked of every component listed, each addressed to it alone ------------
    // `UpdateCurrentSettings` runs for every vehicle in `MAVlist` - every system and component
    // that has sent a heartbeat, gimbals and companions included - and asks each for its seven
    // streams, each request twice as `getDatastream` sends it (ExtLibs/ArduPilot/CurrentState.cs:
    // 4632-4663; MainV2.cs:3058-3069; MAVLinkInterface.cs:3262-3263).
    let mut asked: BTreeMap<(u8, u8), usize> = BTreeMap::new();
    wait_for("every component asked for its streams", || {
        for (sysid, compid, message) in swarm.collect(Duration::from_millis(5)) {
            assert_eq!(
                (sysid, compid),
                (GCS.sysid, GCS.compid),
                "framed as the GCS"
            );
            if let MavMessage::RequestDataStream(request) = message {
                *asked
                    .entry((request.target_system, request.target_component))
                    .or_default() += 1;
            }
        }
        asked.len() == all.len() && asked.values().all(|count| *count >= 14)
    });
    for component in &all {
        assert_eq!(
            asked.get(&(component.sysid, component.compid)),
            Some(&14),
            "streams asked of {component}"
        );
    }

    // --- a command goes to the vehicle it names, and only its ack answers it --------------------
    let target = VehicleId::new(37, 1);
    let id = link.command(target, TAKEOFF, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0], true);
    let mut commands_sent = Vec::new();
    wait_for("the command on the wire", || {
        swarm.pump();
        commands_sent.extend(swarm.inbox.drain(..).filter_map(|(_, _, m)| match m {
            MavMessage::CommandLong(long) => Some(long),
            _ => None,
        }));
        !commands_sent.is_empty()
    });
    assert_eq!(commands_sent.len(), 1);
    assert_eq!(
        (
            commands_sent[0].target_system,
            commands_sent[0].target_component
        ),
        (37, 1)
    );
    let ack = |result| {
        MavMessage::CommandAck(CommandAck {
            command: TAKEOFF,
            result,
            progress: 0,
            result_param2: 0,
            target_system: GCS.sysid,
            target_component: GCS.compid,
        })
    };
    // The same command acknowledged by its neighbour, and by its own companion computer.
    swarm.send(VehicleId::new(36, 1), &ack(4));
    swarm.send(VehicleId::new(30, COMPANION), &ack(4));
    wasm_thread::sleep(Duration::from_millis(20));
    assert_eq!(
        link.request(id).unwrap().outcome(),
        None,
        "another vehicle's ack answered this one's command"
    );
    swarm.send(target, &ack(0));
    wait_for("the ack", || link.request(id).unwrap().outcome().is_some());
    assert_eq!(
        link.request(id).unwrap().outcome(),
        Some(RequestOutcome::Accepted { value: None })
    );

    // --- parameters: one table per component, whatever the names ------------------------------
    for sysid in 1..=AUTOPILOTS {
        swarm.send(
            VehicleId::new(sysid, 1),
            &param("RTL_ALT", f32::from(sysid) * 100.0, 0, 1),
        );
    }
    // A gimbal with a parameter of the same name.
    swarm.send(gimbal, &param("RTL_ALT", 1.0, 0, 1));
    wait_for("every table", || {
        (1..=AUTOPILOTS).all(|sysid| link.params(VehicleId::new(sysid, 1)).is_some())
            && link.params(gimbal).is_some()
    });
    for sysid in 1..=AUTOPILOTS {
        let value = link
            .params(VehicleId::new(sysid, 1))
            .unwrap()
            .get("RTL_ALT")
            .unwrap()
            .as_f64();
        assert!(
            (value - f64::from(sysid) * 100.0).abs() < f64::EPSILON,
            "{sysid}:1 holds {value}"
        );
    }
    let gimbal_value = link
        .params(gimbal)
        .unwrap()
        .get("RTL_ALT")
        .unwrap()
        .as_f64();
    assert!((gimbal_value - 1.0).abs() < f64::EPSILON);

    // A set on one vehicle is answered by its own echo, not by its neighbour's of the same name.
    let set_on = VehicleId::new(25, 1);
    let set = link.set_param(set_on, "RTL_ALT", 3000.0, false);
    wait_for("the set on the wire", || {
        swarm.pump();
        swarm.inbox.drain(..).any(|(_, _, m)| {
            matches!(m, MavMessage::ParamSet(p) if p.target_system == 25 && p.target_component == 1)
        })
    });
    swarm.send(VehicleId::new(26, 1), &param("RTL_ALT", 3000.0, 0, 1));
    wasm_thread::sleep(Duration::from_millis(20));
    assert_eq!(link.request(set).unwrap().outcome(), None);
    swarm.send(set_on, &param("RTL_ALT", 3000.0, 0, 1));
    wait_for("the echo", || {
        link.request(set).unwrap().outcome().is_some()
    });
    assert!(matches!(
        link.request(set).unwrap().outcome(),
        Some(RequestOutcome::Accepted { .. })
    ));

    // --- a parameter download counts only its own vehicle's parameters -------------------------
    // Vehicle 5 is being downloaded; vehicle 6 is streaming its list to some other ground station
    // on the same radio at the same time.
    let downloading = VehicleId::new(5, 1);
    let bystander = VehicleId::new(6, 1);
    link.download_params(downloading);
    wait_for("the list request", || {
        swarm.pump();
        swarm
            .inbox
            .drain(..)
            .any(|(_, _, m)| matches!(m, MavMessage::ParamRequestList(r) if r.target_system == 5))
    });
    let mut bytes = Vec::new();
    for index in 0..20u16 {
        let name = format!("P{index:02}");
        swarm.frame(bystander, &param(&name, 6.0, index, 20), &mut bytes);
        if index != 7 {
            swarm.frame(downloading, &param(&name, 5.0, index, 20), &mut bytes);
        }
    }
    swarm.end.write_all(&bytes).unwrap();
    // Vehicle 5's list ended short, so its hole is asked for - of vehicle 5, and nothing of 6.
    let mut reads: Vec<(u8, i16)> = Vec::new();
    wait_for("the hole to be asked for", || {
        swarm.pump();
        reads.extend(swarm.inbox.drain(..).filter_map(|(_, _, m)| match m {
            MavMessage::ParamRequestRead(r) => Some((r.target_system, r.param_index)),
            _ => None,
        }));
        !reads.is_empty()
    });
    assert_eq!(reads, vec![(5, 7)]);
    swarm.send(downloading, &param("P07", 5.0, 7, 20));
    wait_for("the download", || {
        link.param_download(downloading).map(|d| d.state()) == Some(ParamDownloadState::Complete)
    });
    assert!(link.param_download(bystander).is_none());
    let after = swarm.collect(Duration::from_millis(100));
    assert!(
        after
            .iter()
            .all(|(_, _, m)| !matches!(m, MavMessage::ParamRequestRead(_))),
        "{after:?}"
    );

    // --- a mission download hears only its own vehicle ----------------------------------------
    let holding = VehicleId::new(12, 1);
    let neighbour = VehicleId::new(13, 1);
    let items: Vec<MissionItem> = (0..3)
        .map(|seq| MissionItem {
            seq,
            command: 16,
            x: -35.0,
            y: 149.0 + f64::from(seq) * 0.001,
            z: 30.0,
            ..MissionItem::default()
        })
        .collect();
    link.download_mission(holding);
    let started = Instant::now();
    while !link
        .mission_transfer(holding)
        .is_some_and(|transfer| transfer.is_finished())
    {
        assert!(started.elapsed() < HUNG, "the mission download hung");
        swarm.pump();
        let requests: Vec<MavMessage> = swarm.inbox.drain(..).map(|(_, _, m)| m).collect();
        for message in requests {
            match message {
                MavMessage::MissionRequestList(request) => {
                    assert_eq!(request.target_system, 12);
                    // The neighbour answers first, with a mission of its own.
                    swarm.send(
                        neighbour,
                        &commands::send_mission_count(GCS, 9, MISSION_TYPE_MISSION),
                    );
                    swarm.send(
                        holding,
                        &commands::send_mission_count(GCS, 3, MISSION_TYPE_MISSION),
                    );
                }
                MavMessage::MissionRequestInt(request) => {
                    assert_eq!(request.target_system, 12);
                    let wrong = MissionItem {
                        z: 999.0,
                        ..items[usize::from(request.seq)]
                    };
                    swarm.send(
                        neighbour,
                        &commands::send_mission_item(GCS, &wrong, MISSION_TYPE_MISSION),
                    );
                    swarm.send(
                        holding,
                        &commands::send_mission_item(
                            GCS,
                            &items[usize::from(request.seq)],
                            MISSION_TYPE_MISSION,
                        ),
                    );
                }
                _ => {}
            }
        }
        wasm_thread::sleep(Duration::from_millis(1));
    }
    let transfer = link.mission_transfer(holding).unwrap();
    assert_eq!(transfer.state(), &TransferState::Complete);
    assert_eq!(transfer.items(), items.as_slice());
    assert!(link.mission_transfer(neighbour).is_none());
}
