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

//! The state's clock, its once-a-second counts, the stream requests, the airspeed minimums and
//! the fence, as the real link thread drives them (PLAN.md §13.4 row 39).
//!
//! A scripted vehicle on the far end of a [`Loopback`], as `tests/retries.rs` has it: the link is
//! the real one - its thread, its registry, its `UpdateCurrentSettings` pass - and each test
//! reads what the link published or put on the wire. C# lines are in
//! `ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs` unless named.
//!
//! The counts are made on the wall clock, as a live link's are, so the tests that count seconds
//! take a few seconds each.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::io;
use web_time::{Duration, Instant};

use mp_link::{Link, LinkConfig, ProtocolTimeouts, mission_transfer::TransferState};
use mp_mavlink::{FrameDecoder, encode_v2};
use mp_mavlink_dialects::all::{
    DIALECT, GlobalPositionInt, GpsRawInt, Heartbeat, MavMessage, MissionAck, MissionCount,
    MissionItemInt, MissionRequest, MissionRequestInt, ParamValue, SysStatus, VfrHud,
};
use mp_mission::MissionItem;
use mp_transport::testing::{Loopback, LoopbackEnd};
use mp_transport::{ReadTime, Transport};
use mp_vehicle::{DateTime, FenceItem, StreamRates, VehicleId, VehicleState};

/// The vehicle most tests talk to.
const VEHICLE: VehicleId = VehicleId::new(1, 1);
/// The link's own address.
const GCS: VehicleId = VehicleId::new(255, 190);
/// `MAV_MODE_FLAG_SAFETY_ARMED`.
const ARMED: u8 = 128;
/// `MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE`: the airspeed sensor.
const AIRSPEED_SENSOR: u32 = 16;
/// `MAV_PARAM_TYPE_REAL32`.
const REAL32: u8 = 9;
/// `MAV_MISSION_TYPE_FENCE`.
const FENCE: u8 = 1;
/// How long anything may take before the test is declared hung.
const HUNG: Duration = Duration::from_secs(10);

/// The vehicle end of the link.
struct Peer {
    end: LoopbackEnd,
    decoder: FrameDecoder,
    seq: u8,
    /// Everything the link sent, with when it arrived.
    heard: Vec<(Instant, MavMessage)>,
}

impl Peer {
    fn new(end: LoopbackEnd) -> Self {
        Self {
            end,
            decoder: FrameDecoder::new(),
            seq: 0,
            heard: Vec::new(),
        }
    }

    /// Sends a message as `from`.
    fn send_from(&mut self, from: VehicleId, message: &MavMessage) {
        let mut payload = [0u8; 255];
        let len = message.encode(&mut payload);
        let mut frame = [0u8; mp_mavlink::MAX_FRAME_LEN];
        let n = encode_v2(
            &mut frame,
            self.seq,
            from.sysid,
            from.compid,
            message.id(),
            &payload[..len],
            message.crc_extra(),
            0,
        )
        .unwrap();
        self.seq = self.seq.wrapping_add(1);
        self.end.write_all(&frame[..n]).unwrap();
    }

    fn send(&mut self, message: &MavMessage) {
        self.send_from(VEHICLE, message);
    }

    /// Reads what the link has sent, keeping it in `heard`; returns what was new.
    fn pump(&mut self) -> Vec<MavMessage> {
        let mut buf = [0u8; 4096];
        let mut fresh = Vec::new();
        loop {
            let n = self.end.read(&mut buf).unwrap_or(0);
            if n == 0 {
                break;
            }
            self.decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                if let Some(message) = MavMessage::decode(frame.msgid, frame.payload) {
                    fresh.push(message);
                }
            });
        }
        let now = Instant::now();
        self.heard
            .extend(fresh.iter().map(|message| (now, *message)));
        fresh
    }

    /// The `REQUEST_DATA_STREAM`s heard, as `(stream, rate, target)`.
    fn stream_requests(&self) -> Vec<(u8, u16, VehicleId)> {
        self.heard
            .iter()
            .filter_map(|(_, message)| match message {
                MavMessage::RequestDataStream(request) => Some((
                    request.req_stream_id,
                    request.req_message_rate,
                    VehicleId::new(request.target_system, request.target_component),
                )),
                _ => None,
            })
            .collect()
    }
}

/// A link with streams asked for or not, and a vehicle end.
fn link(streams: bool, timeouts: ProtocolTimeouts) -> (Link, Peer) {
    let (vehicle_side, gcs_side) = Loopback::pair();
    let config = LinkConfig {
        send_heartbeat: false,
        stream_rate_hz: u16::from(streams),
        timeouts,
        ..LinkConfig::default()
    };
    (
        Link::from_transport(Box::new(gcs_side), config),
        Peer::new(vehicle_side),
    )
}

/// Polls until `check` holds, failing rather than hanging.
fn wait_for(what: &str, mut check: impl FnMut() -> bool) {
    let deadline = Instant::now() + HUNG;
    while !check() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        wasm_thread::sleep(Duration::from_millis(2));
    }
}

/// A copter's heartbeat, armed or not.
fn heartbeat(armed: bool) -> MavMessage {
    MavMessage::Heartbeat(Heartbeat {
        custom_mode: 0,
        r#type: 2,
        autopilot: 3,
        base_mode: 81 | if armed { ARMED } else { 0 },
        system_status: 4,
        mavlink_version: 3,
    })
}

/// Where the vehicle is, `north` 1e-7 degrees north of CMAC.
fn position(north: i32) -> [MavMessage; 2] {
    let lat = -353_632_620 + north;
    let lon = 1_491_652_370;
    [
        MavMessage::GlobalPositionInt(GlobalPositionInt {
            time_boot_ms: 0,
            lat,
            lon,
            alt: 600_000,
            relative_alt: 16_000,
            vx: 0,
            vy: 0,
            vz: 0,
            hdg: 0,
        }),
        MavMessage::GpsRawInt(GpsRawInt {
            time_usec: 0,
            lat,
            lon,
            alt: 600_000,
            eph: 100,
            epv: 100,
            vel: 0,
            cog: 0,
            fix_type: 3,
            satellites_visible: 10,
            alt_ellipsoid: 0,
            h_acc: 0,
            v_acc: 0,
            vel_acc: 0,
            hdg_acc: 0,
            yaw: 0,
        }),
    ]
}

/// `VFR_HUD` with this airspeed and throttle.
fn vfr_hud(airspeed: f32, throttle: u16) -> MavMessage {
    MavMessage::VfrHud(VfrHud {
        airspeed,
        groundspeed: 0.0,
        alt: 16.0,
        climb: 0.0,
        heading: 0,
        throttle,
    })
}

/// A `PARAM_VALUE` of a REAL32 parameter, the `index`th of two.
fn param(name: &str, index: u16, value: f32) -> MavMessage {
    MavMessage::ParamValue(ParamValue {
        param_value: value,
        param_count: 2,
        param_index: index,
        param_id: mp_params::encode_param_id(name),
        param_type: REAL32,
    })
}

/// A vehicle's state as last published.
fn state(link: &Link, id: VehicleId) -> Option<VehicleState> {
    link.vehicle(id).map(|handle| *handle.load())
}

// --- the clock --------------------------------------------------------------------------------

/// A live link stamps each packet with the time it is read, before applying it, as
/// `readPacketAsync` sets `MAV.cs.datetime = DateTime.Now` (:4721); the vehicle's `datetime` is
/// `DateTime.MinValue` until then and moves with each packet.
#[test]
fn each_packet_is_stamped_with_the_time_it_is_read() {
    let (link, mut peer) = link(false, ProtocolTimeouts::default());
    let before = DateTime::now();
    peer.send(&heartbeat(false));
    wait_for("the vehicle", || state(&link, VEHICLE).is_some());
    let first = state(&link, VEHICLE).unwrap().datetime;
    let after = DateTime::now();
    assert!(before <= first && first <= after, "{first:?}");

    wasm_thread::sleep(Duration::from_millis(30));
    peer.send(&heartbeat(false));
    wait_for("the clock to move", || {
        state(&link, VEHICLE).unwrap().datetime > first
    });
    let second = state(&link, VEHICLE).unwrap().datetime;
    assert!(second.seconds_since(first) >= 0.03, "{first:?} {second:?}");
    assert!(second <= DateTime::now());
}

/// A transport playing a recording reports the recording's clock, and every packet is stamped
/// with it - as `readlogPacketMavlink` sets `cs.datetime = lastlogread` (:6649) - and nothing is
/// asked of the vehicle, which the C# plays with its port closed (`CurrentState.cs:4636-4637`).
#[test]
fn a_recording_stamps_its_own_time_and_asks_for_no_streams() {
    /// A loopback that says it is a recording made at a fixed time.
    struct Recorded(LoopbackEnd);
    impl Transport for Recorded {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            self.0.read(buf)
        }
        fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
            self.0.write_all(buf)
        }
        fn description(&self) -> &str {
            "recorded"
        }
        fn is_open(&self) -> bool {
            self.0.is_open()
        }
        fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
            self.0.set_read_timeout(timeout)
        }
        fn read_time(&self) -> ReadTime {
            ReadTime::Recorded(Some(STAMP))
        }
    }
    const STAMP: u64 = 1_767_225_607_123_456;

    let (vehicle_side, gcs_side) = Loopback::pair();
    let config = LinkConfig {
        send_heartbeat: false,
        ..LinkConfig::default()
    };
    let link = Link::from_transport(Box::new(Recorded(gcs_side)), config);
    let mut peer = Peer::new(vehicle_side);
    for _ in 0..10 {
        peer.send(&heartbeat(false));
        wasm_thread::sleep(Duration::from_millis(20));
    }
    wait_for("the vehicle", || state(&link, VEHICLE).is_some());
    assert_eq!(
        state(&link, VEHICLE).unwrap().datetime,
        DateTime::from_tlog_micros(STAMP).unwrap()
    );
    wasm_thread::sleep(Duration::from_millis(100));
    peer.pump();
    assert!(
        peer.stream_requests().is_empty(),
        "{:?}",
        peer.stream_requests()
    );
}

// --- the once-a-second counts -----------------------------------------------------------------

/// `UpdateCurrentSettings` runs after each read (`MainV2.cs:3065-3076`) and counts a second in
/// the air each time the clock's seconds field changes while armed with the throttle over 12%,
/// and the straight line flown since the last counted second while armed on a 3D fix
/// (`CurrentState.cs:4605-4629`). Disarmed, neither counts.
#[test]
fn time_in_air_and_distance_count_each_second_while_armed() {
    let (link, mut peer) = link(false, ProtocolTimeouts::default());
    // Disarmed and moving, with the throttle up: nothing counts.
    let mut north = 0;
    let tick = |peer: &mut Peer, armed: bool, north: i32| {
        peer.send(&heartbeat(armed));
        for message in position(north) {
            peer.send(&message);
        }
        peer.send(&vfr_hud(0.0, 50));
        wasm_thread::sleep(Duration::from_millis(100));
    };
    for _ in 0..15 {
        tick(&mut peer, false, north);
        north += 100;
    }
    let held = state(&link, VEHICLE).unwrap();
    assert_eq!(held.time_in_air, 0.0);
    assert_eq!(held.dist_traveled, 0.0);

    // Armed: 1e-5 degrees north - 1.11 m - a tick, 11 m a second, for 3.5 seconds of ticks - or
    // longer where the machine stretches the sleeps (the hosted macOS runner, 2026-10-03, took
    // about six seconds over them and counted six), so the count is held to the time that passed.
    let armed_at = Instant::now();
    for _ in 0..35 {
        tick(&mut peer, true, north);
        north += 100;
    }
    wasm_thread::sleep(Duration::from_millis(100));
    let held = state(&link, VEHICLE).unwrap();
    let armed_for = armed_at.elapsed().as_secs_f64();
    // One second mark per second that passed, give or take the one at either end; the first
    // mark only takes the position.
    assert!(
        (armed_for.floor() - 1.0..=armed_for.ceil()).contains(&f64::from(held.time_in_air)),
        "time in air {} after {armed_for:.2} s armed",
        held.time_in_air
    );
    assert_eq!(held.time_since_arm_in_air, held.time_in_air);
    // Each tick moves 1e-5 degrees of latitude, 1.113 m; the ticks a second holds is what this
    // run managed (ten on a quiet machine, six on the stretched runner), and the distance is the
    // speed times the seconds counted, less the two second marks at the ends that take positions
    // rather than count, plus a mark's worth either way (43 m in 6 s on the runner, 2026-10-03).
    let per_tick = 1.113;
    let ticks_per_second = 35.0 / armed_for;
    let flown = f64::from(held.dist_traveled);
    let seconds = f64::from(held.time_in_air);
    assert!(
        flown > (seconds - 2.0) * ticks_per_second * per_tick
            && flown < (seconds + 1.0) * ticks_per_second * per_tick * 1.2,
        "distance {flown} m in {seconds} s at {ticks_per_second:.1} ticks a second"
    );
}

// --- the stream requests ----------------------------------------------------------------------

/// The telemetry streams are asked for at the vehicle's own rates - the saved defaults it started
/// from, then whatever the Planner page set - in `UpdateCurrentSettings`' order, each twice as
/// `getDatastream` sends it, and a rate of -1 not at all; once when the vehicle is first listed
/// and again 38 seconds later (`CurrentState.cs:4635-4666`, here with the waits divided by 100).
#[test]
fn the_streams_are_asked_for_at_the_vehicles_own_rates() {
    let timeouts = ProtocolTimeouts::default().faster(100);
    assert_eq!(timeouts.stream_rerequest, Duration::from_millis(380));
    let (link, mut peer) = link(true, timeouts);
    peer.send(&heartbeat(false));
    wait_for("the first requests", || {
        peer.pump();
        peer.stream_requests().len() >= 14
    });
    let first = state(&link, VEHICLE).unwrap().rates;
    let expect = |rates: StreamRates| -> Vec<(u8, u16, VehicleId)> {
        [
            (2, rates.status),
            (6, rates.position),
            (10, rates.attitude),
            (11, rates.attitude),
            (12, rates.sensors),
            (1, rates.sensors),
            (3, rates.rc),
        ]
        .into_iter()
        .filter(|(_, hz)| *hz != -1)
        .flat_map(|(stream, hz)| {
            let hz = u16::try_from(hz).unwrap();
            [(stream, hz, VEHICLE), (stream, hz, VEHICLE)]
        })
        .collect()
    };
    assert_eq!(peer.stream_requests(), expect(first));
    let asked_at = peer.heard.last().unwrap().0;

    // The Planner page's rates for this vehicle, RC turned off with -1.
    let chosen = StreamRates {
        attitude: 10,
        position: 5,
        status: 3,
        sensors: 1,
        rc: -1,
    };
    link.set_stream_rates(VEHICLE, chosen);
    wait_for("the state to carry them", || {
        state(&link, VEHICLE).unwrap().rates == chosen
    });
    // Nothing more until the re-request is due.
    peer.send(&heartbeat(false));
    wait_for("the second round", || {
        peer.send(&heartbeat(false));
        wasm_thread::sleep(Duration::from_millis(10));
        peer.pump();
        peer.stream_requests().len() >= 14 + 12
    });
    let requests = peer.stream_requests();
    assert_eq!(requests[14..], expect(chosen)[..]);
    let again_at = peer
        .heard
        .iter()
        .filter(|(_, message)| matches!(message, MavMessage::RequestDataStream(_)))
        .nth(14)
        .unwrap()
        .0;
    assert!(
        again_at.duration_since(asked_at) >= Duration::from_millis(300),
        "asked again after {:?}",
        again_at.duration_since(asked_at)
    );
}

/// Every vehicle listed on the link is asked, each for itself, as the C# asks every `MAVState`
/// in `MAVlist` (`MainV2.cs:3065-3076`); one that has sent no `HEARTBEAT` is not listed
/// (`MAVList.cs:25-30, 101-117`) and is not asked.
#[test]
fn every_listed_vehicle_is_asked_and_an_unlisted_one_is_not() {
    let (_link, mut peer) = link(true, ProtocolTimeouts::default());
    let gimbal = VehicleId::new(1, 154);
    let radio = VehicleId::new(51, 68);
    peer.send(&heartbeat(false));
    peer.send_from(gimbal, &heartbeat(false));
    // Anything but a heartbeat: a hidden entry, never updated.
    peer.send_from(radio, &vfr_hud(0.0, 0));
    wait_for("both to be asked", || {
        peer.pump();
        peer.stream_requests().len() >= 28
    });
    wasm_thread::sleep(Duration::from_millis(100));
    peer.pump();
    let requests = peer.stream_requests();
    assert_eq!(requests.len(), 28);
    for id in [VEHICLE, gimbal] {
        assert_eq!(requests.iter().filter(|(_, _, to)| *to == id).count(), 14);
    }
}

// --- the airspeed minimums --------------------------------------------------------------------

/// The low-airspeed warning reads `AIRSPEED_MIN`, or failing it `ARSPD_FBW_MIN`, from the
/// vehicle's parameters, which every `PARAM_VALUE` updates as it passes (:5766-5796;
/// `CurrentState.cs:3861-3891`). Three aircraft on one link, armed, in the air and slow, with a
/// healthy airspeed sensor: the one with both parameters is held to `AIRSPEED_MIN`, the one with
/// only `ARSPD_FBW_MIN` to that, and the one with neither never warns.
#[test]
fn the_airspeed_minimums_come_from_the_parameters() {
    let (link, mut peer) = link(false, ProtocolTimeouts::default());
    let both = VEHICLE;
    let fbw = VehicleId::new(2, 1);
    let neither = VehicleId::new(3, 1);
    for id in [both, fbw, neither] {
        peer.send_from(id, &heartbeat(false));
    }
    // AIRSPEED_MIN 10 over ARSPD_FBW_MIN 4: at 5 m/s, slow only by the first.
    peer.send_from(both, &param("AIRSPEED_MIN", 0, 10.0));
    peer.send_from(both, &param("ARSPD_FBW_MIN", 1, 4.0));
    peer.send_from(fbw, &param("ARSPD_FBW_MIN", 1, 8.0));
    let sensors = MavMessage::SysStatus(SysStatus {
        onboard_control_sensors_present: AIRSPEED_SENSOR,
        onboard_control_sensors_enabled: AIRSPEED_SENSOR,
        onboard_control_sensors_health: AIRSPEED_SENSOR,
        load: 0,
        voltage_battery: 12_000,
        current_battery: -1,
        drop_rate_comm: 0,
        errors_comm: 0,
        errors_count1: 0,
        errors_count2: 0,
        errors_count3: 0,
        errors_count4: 0,
        battery_remaining: -1,
        onboard_control_sensors_present_extended: 0,
        onboard_control_sensors_enabled_extended: 0,
        onboard_control_sensors_health_extended: 0,
    });
    // Armed with the throttle up until a second has been counted in the air, then slow.
    let deadline = Instant::now() + Duration::from_millis(2500);
    while Instant::now() < deadline {
        for id in [both, fbw, neither] {
            peer.send_from(id, &heartbeat(true));
            peer.send_from(id, &sensors);
            peer.send_from(id, &vfr_hud(5.0, 50));
        }
        wasm_thread::sleep(Duration::from_millis(100));
    }
    wasm_thread::sleep(Duration::from_millis(50));
    for id in [both, fbw, neither] {
        assert!(
            state(&link, id).unwrap().time_since_arm_in_air > 0.0,
            "{id} never counted a second in the air"
        );
    }
    assert!(state(&link, both).unwrap().low_airspeed, "AIRSPEED_MIN");
    assert!(state(&link, fbw).unwrap().low_airspeed, "ARSPD_FBW_MIN");
    assert!(!state(&link, neither).unwrap().low_airspeed, "no parameter");
}

// --- the fence --------------------------------------------------------------------------------

/// A fence polygon round CMAC, three vertices.
fn fence_items() -> Vec<MissionItem> {
    [(-35.36, 149.16), (-35.37, 149.16), (-35.365, 149.17)]
        .iter()
        .enumerate()
        .map(|(seq, (lat, lng))| MissionItem {
            seq: u16::try_from(seq).unwrap(),
            frame: 3,
            command: 5001,
            param1: 3.0,
            x: *lat,
            y: *lng,
            ..MissionItem::default()
        })
        .collect()
}

/// What the link files for an uploaded item: `(Locationwp) req` (:4297, :4335).
fn filed(item: &MissionItem) -> FenceItem {
    mp_link::fence_points::uploaded(item)
}

/// A fence that passes on the link is the vehicle's `fencepoints`, whoever asked for it
/// (:5627-5698): a `MISSION_COUNT` starts it again and each item is filed under its sequence
/// number; `GeoFenceDist` measures from it.
#[test]
fn a_fence_read_from_the_vehicle_is_its_fence() {
    let (link, mut peer) = link(false, ProtocolTimeouts::default());
    peer.send(&heartbeat(false));
    for message in position(0) {
        peer.send(&message);
    }
    peer.send(&MavMessage::MissionCount(MissionCount {
        count: 3,
        target_system: GCS.sysid,
        target_component: GCS.compid,
        mission_type: FENCE,
    }));
    for item in fence_items() {
        let wire = item.to_wire();
        peer.send(&MavMessage::MissionItemInt(MissionItemInt {
            param1: wire.param1,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: wire.x,
            y: wire.y,
            z: 0.0,
            seq: wire.seq,
            command: wire.command,
            target_system: GCS.sysid,
            target_component: GCS.compid,
            frame: 3,
            current: 0,
            autocontinue: 1,
            mission_type: FENCE,
        }));
    }
    wait_for("the fence", || link.fence_points(VEHICLE).len() == 3);
    let fence = link.fence_points(VEHICLE);
    let wires: Vec<(i32, i32)> = fence_items()
        .iter()
        .map(|item| (item.to_wire().x, item.to_wire().y))
        .collect();
    assert_eq!(
        fence
            .iter()
            .map(|item| (item.x, item.y))
            .collect::<Vec<_>>(),
        wires
    );
    wait_for("the position", || {
        state(&link, VEHICLE).is_some_and(|state| state.position.is_some())
    });
    let distance = state(&link, VEHICLE).unwrap().geo_fence_dist(&fence);
    assert!(distance > 0.0 && distance < 2000.0, "{distance}");
    assert_eq!(
        state(&link, VEHICLE).unwrap().geo_fence_dist(&[]),
        99999.0,
        "no fence"
    );

    // Another count starts it again.
    peer.send(&MavMessage::MissionCount(MissionCount {
        count: 0,
        target_system: GCS.sysid,
        target_component: GCS.compid,
        mission_type: FENCE,
    }));
    wait_for("the fence cleared", || {
        link.fence_points(VEHICLE).is_empty()
    });
}

/// This link's own fence upload files each item as `setWPAsync` does: cleared when the vehicle
/// asks for the first item, each item filed when a `MISSION_REQUEST` asks for the next, the last
/// when the `MISSION_ACK` comes (:3801-3830, :4273-4346). A vehicle that asks with
/// `MISSION_REQUEST_INT` gets only the last one filed, as the C# has no branch that files from it.
#[test]
fn this_links_fence_upload_files_what_the_csharp_files() {
    for with_int in [false, true] {
        let (link, mut peer) = link(false, ProtocolTimeouts::default().faster(10));
        peer.send(&heartbeat(false));
        wait_for("the vehicle", || state(&link, VEHICLE).is_some());
        // Something filed before: the upload's first request clears it.
        peer.send(&MavMessage::MissionItemInt(MissionItemInt {
            param1: 1.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 1,
            y: 1,
            z: 0.0,
            seq: 7,
            command: 5003,
            target_system: GCS.sysid,
            target_component: GCS.compid,
            frame: 3,
            current: 0,
            autocontinue: 1,
            mission_type: FENCE,
        }));
        wait_for("the stray item", || link.fence_points(VEHICLE).len() == 1);

        let items = fence_items();
        link.upload_list(VEHICLE, items.clone(), FENCE);
        let request = |seq: u16| {
            if with_int {
                MavMessage::MissionRequestInt(MissionRequestInt {
                    seq,
                    target_system: GCS.sysid,
                    target_component: GCS.compid,
                    mission_type: FENCE,
                })
            } else {
                MavMessage::MissionRequest(MissionRequest {
                    seq,
                    target_system: GCS.sysid,
                    target_component: GCS.compid,
                    mission_type: FENCE,
                })
            }
        };
        let deadline = Instant::now() + HUNG;
        loop {
            assert!(Instant::now() < deadline, "the upload hung");
            let finished = link
                .list_transfer(VEHICLE, FENCE)
                .is_some_and(|transfer| matches!(transfer.state(), TransferState::Complete));
            if finished {
                break;
            }
            for message in peer.pump() {
                match message {
                    MavMessage::MissionCount(count) if count.mission_type == FENCE => {
                        peer.send(&request(0));
                    }
                    MavMessage::MissionItemInt(item) if item.mission_type == FENCE => {
                        if item.seq == 0 {
                            // The first request cleared the stray item.
                            assert!(link.fence_points(VEHICLE).is_empty(), "not cleared");
                        }
                        if usize::from(item.seq) + 1 < items.len() {
                            peer.send(&request(item.seq + 1));
                        } else {
                            peer.send(&MavMessage::MissionAck(MissionAck {
                                target_system: GCS.sysid,
                                target_component: GCS.compid,
                                r#type: 0,
                                mission_type: FENCE,
                            }));
                        }
                    }
                    _ => {}
                }
            }
            wasm_thread::sleep(Duration::from_millis(1));
        }
        wait_for("the last item filed", || {
            link.fence_points(VEHICLE)
                .last()
                .is_some_and(|last| *last == filed(&items[2]))
        });
        let expected: Vec<FenceItem> = if with_int {
            vec![filed(&items[2])]
        } else {
            items.iter().map(filed).collect()
        };
        assert_eq!(
            link.fence_points(VEHICLE),
            expected,
            "MISSION_REQUEST_INT: {with_int}"
        );
    }
}
