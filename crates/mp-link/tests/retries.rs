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

//! Protocol state machines under fault (DELIVERABLES.md Deliverable 4, PLAN.md §7.3, §13.3 item 5).
//!
//! Each test puts a scripted vehicle on the far end of a [`Loopback`] and has it misbehave in one
//! way on purpose: drop a reply, answer late, answer twice, answer the wrong question, skip,
//! refuse. The link under test is the real one - the real I/O thread, the real machines - with
//! Mission Planner's retry *counts* and its waits divided down by
//! [`ProtocolTimeouts::faster`], so a test that runs the C#'s full retry ladder takes a fraction
//! of a second instead of the C#'s minute.
//!
//! Every test asserts three things: that the machine stopped (complete, or a clean failure - never
//! a hang), that it stopped within the time its waits allow, and that the number of times it put
//! the step on the wire is the C#'s. The C# constants, with their lines in
//! `ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs`:
//!
//! | loop | retries | wait | lines |
//! |---|---|---|---|
//! | `setParamAsync` | 3 | 700 ms | 1748, 1754 |
//! | `GetParamAsync` | 3 | 700 ms | 2329, 2333 |
//! | `doCommandAsync` | 3 | 2000 ms; arm 10 s; calibration and bootloader 1 retry, 25 s | 2729, 2731, 2748-2768 |
//! | `setWPCurrentAsync` | 5 | 2000 ms | 2472, 2476 |
//! | `getWPCountAsync` | 6 | 700 ms | 3297, 3301 |
//! | `getWPAsync` | 5 | 2500 ms | 3459, 3463 |
//! | `setWPTotalAsync` | 3 | 700 ms | 3779, 3783 |
//! | `setWPAsync` | 10 | 450 ms | 4250, 4254 |
//! | `doCommandIntAsync` | 3 | 2000 ms | 2884, 2885 |
//! | `getHomePositionAsync` | 3 | 700 ms | 3362, 3366 |
//! | `getParamListAsync` | 2 whole-list | 4000 ms quiet, 1000 ms rounds of 10 | 2114, 2117, 2135, 2187 |
//!
//! Where this port differs from the C# on purpose, the test that shows it says so, with the C#
//! lines and the reason (PLAN.md §12 D8: bug-for-bug or corrected is decided case by case).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use mp_link::mission_transfer::{
    MISSION_ACCEPTED, MISSION_ERROR, MISSION_INVALID_SEQUENCE, MISSION_NO_SPACE, TransferFailure,
    TransferState, TransferStep,
};
use mp_link::param_download::ParamDownloadState;
use mp_link::requests::{
    CMD_GET_HOME_POSITION, CMD_PREFLIGHT_CALIBRATION, CMD_PREFLIGHT_REBOOT_SHUTDOWN,
    MAV_RESULT_ACCEPTED, MAV_RESULT_IN_PROGRESS, MAV_RESULT_TEMPORARILY_REJECTED, RequestOutcome,
};
use mp_link::{Link, LinkConfig, ProtocolTimeouts, RequestId, commands};
use mp_mavlink::{FrameDecoder, encode_v2};
use mp_mavlink_dialects::all::{
    AutopilotVersion, CommandAck, DIALECT, FencePoint, Heartbeat, HomePosition, MavMessage,
    MissionAck, MissionCurrent, MissionItem as MissionItemFloat, MissionItemInt, MissionRequest,
    MissionRequestInt, ParamValue,
};
use mp_mission::{MISSION_TYPE_MISSION, MissionItem};
use mp_params::ParamValue as ParamValueHeld;
use mp_transport::Transport;
use mp_transport::testing::{Loopback, LoopbackEnd};
use mp_vehicle::{FenceItem, VehicleId};

/// The vehicle every test talks to.
const VEHICLE: VehicleId = VehicleId::new(1, 1);
/// The link's own address, which the vehicle's replies are addressed to.
const GCS: VehicleId = VehicleId::new(255, 190);
/// `MAV_PARAM_TYPE_INT32`, which is what ArduPilot declares `RTL_ALT` as.
const PARAM_TYPE_INT32: u8 = 6;
/// `MAV_PARAM_TYPE_REAL32`.
const PARAM_TYPE_REAL32: u8 = 9;
/// `MAV_CMD_NAV_TAKEOFF`: an ordinary command, with the ordinary wait.
const TAKEOFF: u16 = 22;

/// How long any one test may take before it is declared hung. Generous against the waits it
/// runs, because the point is to catch a machine that never stops, not a busy machine.
const HUNG: Duration = Duration::from_secs(5);

/// The vehicle end of the link, scripted by each test.
struct Peer {
    end: LoopbackEnd,
    decoder: FrameDecoder,
    seq: u8,
    /// What the link sent and the script has not looked at yet.
    inbox: VecDeque<MavMessage>,
    /// Everything the link sent, with when it arrived, for counting afterwards.
    log: Vec<(Instant, MavMessage)>,
}

impl Peer {
    /// Reads whatever the link has sent.
    fn pump(&mut self) {
        let mut buf = [0u8; 4096];
        loop {
            let n = self.end.read(&mut buf).unwrap_or(0);
            if n == 0 {
                return;
            }
            let now = Instant::now();
            let Self {
                decoder,
                inbox,
                log,
                ..
            } = self;
            decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                if let Some(message) = MavMessage::decode(frame.msgid, frame.payload) {
                    inbox.push_back(message);
                    log.push((now, message));
                }
            });
        }
    }

    /// The next thing the link sent, waiting up to `within` for it.
    fn next(&mut self, within: Duration) -> Option<MavMessage> {
        let deadline = Instant::now() + within;
        loop {
            self.pump();
            if let Some(message) = self.inbox.pop_front() {
                return Some(message);
            }
            if Instant::now() >= deadline {
                return None;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    /// Everything the link sends in the next `window`.
    fn collect(&mut self, window: Duration) -> Vec<MavMessage> {
        let deadline = Instant::now() + window;
        let mut seen = Vec::new();
        while let Some(message) = self.next(deadline.saturating_duration_since(Instant::now())) {
            seen.push(message);
        }
        seen
    }

    /// Sends a message as the autopilot.
    fn send(&mut self, message: &MavMessage) {
        self.send_from(VEHICLE, message);
    }

    /// Sends a message as some other system or component on the same link.
    fn send_from(&mut self, from: VehicleId, message: &MavMessage) {
        let bytes = self.frame(from, message);
        self.end.write_all(&bytes).unwrap();
    }

    /// Several messages in one write, so the link reads them in one go and no tick of its loop
    /// falls between them.
    fn send_all(&mut self, messages: &[MavMessage]) {
        let mut bytes = Vec::new();
        for message in messages {
            bytes.extend(self.frame(VEHICLE, message));
        }
        self.end.write_all(&bytes).unwrap();
    }

    fn frame(&mut self, from: VehicleId, message: &MavMessage) -> Vec<u8> {
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
        frame[..n].to_vec()
    }

    /// How many messages the link sent that `pick` recognises.
    fn count(&self, pick: impl Fn(&MavMessage) -> bool) -> usize {
        self.log.iter().filter(|(_, message)| pick(message)).count()
    }

    /// When each message `pick` recognises arrived.
    fn times(&self, pick: impl Fn(&MavMessage) -> bool) -> Vec<Instant> {
        self.log
            .iter()
            .filter(|(_, message)| pick(message))
            .map(|(at, _)| *at)
            .collect()
    }

    /// The values `pick` extracts from what the link sent, in order.
    fn sent<T>(&self, pick: impl Fn(&MavMessage) -> Option<T>) -> Vec<T> {
        self.log
            .iter()
            .filter_map(|(_, message)| pick(message))
            .collect()
    }
}

/// A link with Mission Planner's retry counts and these waits, and a vehicle on the other end
/// that has announced itself as ArduPilot.
fn link(timeouts: ProtocolTimeouts) -> (Link, Peer) {
    let (vehicle_side, gcs_side) = Loopback::pair();
    let config = LinkConfig {
        send_heartbeat: false,
        stream_rate_hz: 0,
        timeouts,
        ..LinkConfig::default()
    };
    let link = Link::from_transport(Box::new(gcs_side), config);
    let mut peer = Peer {
        end: vehicle_side,
        decoder: FrameDecoder::new(),
        seq: 0,
        inbox: VecDeque::new(),
        log: Vec::new(),
    };
    // ArduPilot, so parameter values go in the float field as numbers (see mp-params).
    peer.send(&MavMessage::Heartbeat(Heartbeat {
        custom_mode: 0,
        r#type: 2,
        autopilot: 3,
        base_mode: 81,
        system_status: 3,
        mavlink_version: 3,
    }));
    wait_for("the vehicle to be seen", || link.vehicle(VEHICLE).is_some());
    (link, peer)
}

/// Polls until `check` holds, failing rather than hanging.
fn wait_for(what: &str, mut check: impl FnMut() -> bool) {
    let deadline = Instant::now() + HUNG;
    while !check() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        std::thread::sleep(Duration::from_millis(1));
    }
}

/// Plays the vehicle: hands each message the link sends to `script`, until `done`. Returns how
/// long that took, and fails the test if it took longer than [`HUNG`] - a machine that never
/// stops.
fn drive(
    peer: &mut Peer,
    mut script: impl FnMut(&mut Peer, MavMessage),
    mut done: impl FnMut() -> bool,
) -> Duration {
    let started = Instant::now();
    loop {
        if done() {
            return started.elapsed();
        }
        assert!(
            started.elapsed() < HUNG,
            "the machine did not stop within {HUNG:?}"
        );
        if let Some(message) = peer.next(Duration::from_millis(1)) {
            script(peer, message);
        }
    }
}

/// A vehicle that says nothing at all.
fn silent(_: &mut Peer, _: MavMessage) {}

// --- messages the vehicle sends -----------------------------------------------------------------

fn param_value(name: &str, value: f32, param_type: u8, index: u16, count: u16) -> MavMessage {
    MavMessage::ParamValue(ParamValue {
        param_value: value,
        param_count: count,
        param_index: index,
        param_id: mp_params::encode_param_id(name),
        param_type,
    })
}

/// Parameter `index` of a synthetic `count`-parameter vehicle.
fn numbered(index: u16, count: u16) -> MavMessage {
    param_value(
        &format!("P{index:04}"),
        f32::from(index),
        PARAM_TYPE_REAL32,
        index,
        count,
    )
}

fn command_ack(command: u16, result: u8) -> MavMessage {
    MavMessage::CommandAck(CommandAck {
        command,
        result,
        progress: 0,
        result_param2: 0,
        target_system: GCS.sysid,
        target_component: GCS.compid,
    })
}

fn mission_request(seq: u16) -> MavMessage {
    commands::request_mission_item(GCS, seq, MISSION_TYPE_MISSION)
}

/// The older, float, form of the request, which ArduPilot still sends to a GCS it has not seen
/// ask with `_INT` (see `route_transfer` in mp-link's lib.rs).
fn mission_request_float(seq: u16) -> MavMessage {
    MavMessage::MissionRequest(MissionRequest {
        seq,
        target_system: GCS.sysid,
        target_component: GCS.compid,
        mission_type: MISSION_TYPE_MISSION,
    })
}

fn mission_ack(result: u8) -> MavMessage {
    commands::send_mission_ack(GCS, result, MISSION_TYPE_MISSION)
}

fn mission(count: u16) -> Vec<MissionItem> {
    (0..count)
        .map(|seq| MissionItem {
            seq,
            command: 16,
            x: -35.0 + f64::from(seq) * 0.001,
            y: 149.0,
            z: 50.0,
            ..MissionItem::default()
        })
        .collect()
}

// --- what the link sends ------------------------------------------------------------------------

fn is_param_set(message: &MavMessage) -> bool {
    matches!(message, MavMessage::ParamSet(_))
}

fn is_param_list(message: &MavMessage) -> bool {
    matches!(message, MavMessage::ParamRequestList(_))
}

fn param_read_index(message: &MavMessage) -> Option<u16> {
    match message {
        MavMessage::ParamRequestRead(read) => u16::try_from(read.param_index).ok(),
        _ => None,
    }
}

fn is_param_read(message: &MavMessage) -> bool {
    matches!(message, MavMessage::ParamRequestRead(_))
}

fn command_long(message: &MavMessage) -> Option<(u16, u8)> {
    match message {
        MavMessage::CommandLong(long) => Some((long.command, long.confirmation)),
        _ => None,
    }
}

fn item_sent(message: &MavMessage) -> Option<u16> {
    match message {
        MavMessage::MissionItemInt(item) => Some(item.seq),
        _ => None,
    }
}

fn item_requested(message: &MavMessage) -> Option<u16> {
    match message {
        MavMessage::MissionRequestInt(request) => Some(request.seq),
        _ => None,
    }
}

fn ack_sent(message: &MavMessage) -> Option<u8> {
    match message {
        MavMessage::MissionAck(ack) => Some(ack.r#type),
        _ => None,
    }
}

fn partial_list(message: &MavMessage) -> Option<(i16, i16)> {
    match message {
        MavMessage::MissionWritePartialList(partial) => {
            Some((partial.start_index, partial.end_index))
        }
        _ => None,
    }
}

fn outcome(link: &Link, id: RequestId) -> Option<RequestOutcome> {
    link.request(id).and_then(|request| request.outcome())
}

fn download_state(link: &Link) -> Option<ParamDownloadState> {
    link.param_download(VEHICLE)
        .map(|download| download.state())
}

fn transfer_state(link: &Link) -> Option<TransferState> {
    link.mission_transfer(VEHICLE)
        .map(|transfer| transfer.state().clone())
}

fn transfer_finished(link: &Link) -> bool {
    link.mission_transfer(VEHICLE)
        .is_some_and(|transfer| transfer.is_finished())
}

/// A vehicle whose parameter table the link has seen: `RTL_ALT`, an `INT32` of 1500.
fn with_rtl_alt(timeouts: ProtocolTimeouts) -> (Link, Peer) {
    let (link, mut peer) = link(timeouts);
    peer.send(&param_value("RTL_ALT", 1500.0, PARAM_TYPE_INT32, 0, 1));
    wait_for("RTL_ALT in the table", || {
        link.params(VEHICLE)
            .is_some_and(|table| table.get("RTL_ALT").is_some())
    });
    (link, peer)
}

fn echo(value: f32) -> MavMessage {
    param_value("RTL_ALT", value, PARAM_TYPE_INT32, 0, 1)
}

// ================================================================================================
// Parameter download: getParamListAsync (C#: MAVLinkInterface.cs:1948-2263)
// ================================================================================================

/// `PARAM_VALUE` in any order completes the download; nothing is asked for twice.
///
/// Received indices are a set (`indexsreceived`, :2039-2047), so order is irrelevant. The whole
/// shuffled stream goes in one write so no tick of the link's loop falls inside it - the case
/// where one does is the next test.
#[test]
fn parameters_arriving_in_any_order_complete_without_asking_again() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = link(t);
    const COUNT: u16 = 50;
    // 7 is coprime with 50, so this visits every index once, and the last index is not last.
    let shuffled: Vec<MavMessage> = (0..COUNT)
        .map(|i| numbered((i * 7 + 3) % COUNT, COUNT))
        .collect();
    link.download_params(VEHICLE);
    let took = drive(
        &mut peer,
        |peer, message| {
            if is_param_list(&message) {
                peer.send_all(&shuffled);
            }
        },
        || download_state(&link) == Some(ParamDownloadState::Complete),
    );

    // A quiet spell longer than the quiet timeout: a finished download asks for nothing more.
    let after = peer.collect(t.param_list_quiet * 2);
    assert!(after.is_empty(), "sent after completing: {after:?}");
    assert_eq!(peer.count(is_param_list), 1);
    assert_eq!(peer.count(is_param_read), 0);
    assert_eq!(link.params(VEHICLE).unwrap().len(), usize::from(COUNT));
    assert!(took < t.param_list_quiet, "took {took:?}");
}

/// The last index arriving with a hole behind it starts recovery at once, without waiting for
/// the stream to go quiet: "we hit the last param, jump striaght to retry if params missing"
/// (:2088-2090, :2114). Past three quarters, recovery is one by one (:2117), so the hole is read
/// by index; the stream's own late copy of it, arriving after, is "Already got" (:2039-2047).
#[test]
fn the_last_index_arriving_short_asks_for_the_hole_at_once() {
    let t = ProtocolTimeouts::default().faster(4);
    let (link, mut peer) = link(t);
    const COUNT: u16 = 50;
    let hole = 48;

    link.download_params(VEHICLE);
    let mut read_seen_after = None;
    let mut streamed_at = None;
    drive(
        &mut peer,
        |peer, message| {
            if is_param_list(&message) {
                let stream: Vec<MavMessage> = (0..COUNT)
                    .filter(|i| *i != hole)
                    .map(|i| numbered(i, COUNT))
                    .collect();
                peer.send_all(&stream);
                streamed_at = Some(Instant::now());
            } else if param_read_index(&message) == Some(hole) {
                read_seen_after = streamed_at.map(|at| at.elapsed());
                // The answer, and then the reordered stream's own copy of it.
                peer.send(&numbered(hole, COUNT));
                peer.send(&numbered(hole, COUNT));
            }
        },
        || download_state(&link) == Some(ParamDownloadState::Complete),
    );

    let waited = read_seen_after.expect("the hole was asked for");
    assert!(
        waited < t.param_list_quiet / 2,
        "asked after {waited:?}; the quiet timeout is {:?}",
        t.param_list_quiet
    );
    assert_eq!(peer.sent(param_read_index), vec![hole]);
    assert_eq!(peer.count(is_param_list), 1);
}

/// Holes filled only on request, ten reads a round, a round a second
/// (:2135, :2187). Twelve holes with the last index present: the first round reads the ten
/// lowest, the second starts where the first stopped (`tenbytenindex`, :2140-2141, :2173) and
/// reads the remaining two a round later.
#[test]
fn holes_are_read_ten_at_a_time_a_round_apart() {
    // A round long enough for every read to be answered inside it on a loaded machine.
    let t = ProtocolTimeouts {
        param_list_round: Duration::from_millis(200),
        ..ProtocolTimeouts::default().faster(10)
    };
    let (link, mut peer) = link(t);
    const COUNT: u16 = 60;
    let holes: Vec<u16> = (0..12).map(|i| 3 + i * 4).collect();

    link.download_params(VEHICLE);
    let took = drive(
        &mut peer,
        |peer, message| {
            if is_param_list(&message) {
                let stream: Vec<MavMessage> = (0..COUNT)
                    .filter(|i| !holes.contains(i))
                    .map(|i| numbered(i, COUNT))
                    .collect();
                peer.send_all(&stream);
            } else if let Some(index) = param_read_index(&message) {
                peer.send(&numbered(index, COUNT));
            }
        },
        || download_state(&link) == Some(ParamDownloadState::Complete),
    );

    assert_eq!(
        peer.sent(param_read_index),
        holes,
        "each hole read once, in order"
    );
    // Measured where the vehicle sees them, so allow for its own scheduling; back to back
    // would be well under a millisecond.
    let reads = peer.times(is_param_read);
    let gap = reads[10].duration_since(reads[0]);
    assert!(
        gap >= t.param_list_round / 2,
        "the second round came {gap:?} after the first; a round is {:?}",
        t.param_list_round
    );
    assert_eq!(
        peer.count(is_param_list),
        1,
        "80% arrived: no whole-list retry"
    );
    assert_eq!(link.params(VEHICLE).unwrap().len(), usize::from(COUNT));
    assert!(took < t.param_list_round * 3, "took {took:?}");
}

/// Under three quarters when the stream goes quiet: the whole list again, at most twice
/// (`retry < 2`, :2117), each after a quiet spell, and then one by one.
#[test]
fn a_stream_under_three_quarters_is_asked_for_whole_twice_then_one_by_one() {
    // Rounds long enough that every read is answered inside its own round, even on a loaded
    // machine; otherwise the next round rightly asks again and the exact list below is wrong.
    let t = ProtocolTimeouts {
        param_list_quiet: Duration::from_millis(150),
        param_list_round: Duration::from_millis(200),
        ..ProtocolTimeouts::default().faster(20)
    };
    let (link, mut peer) = link(t);
    const COUNT: u16 = 40;

    link.download_params(VEHICLE);
    let mut lists = 0;
    let took = drive(
        &mut peer,
        |peer, message| {
            if is_param_list(&message) {
                lists += 1;
                // Only the first request is answered, and only in part - and not the last index,
                // so only the quiet timeout can start recovery.
                if lists == 1 {
                    let stream: Vec<MavMessage> = (0..10).map(|i| numbered(i, COUNT)).collect();
                    peer.send_all(&stream);
                }
            } else if let Some(index) = param_read_index(&message) {
                peer.send(&numbered(index, COUNT));
            }
        },
        || download_state(&link) == Some(ParamDownloadState::Complete),
    );

    assert_eq!(
        peer.count(is_param_list),
        1 + usize::from(t.param_list_full_retries),
        "the first request and the C#'s two retries"
    );
    let times = peer.times(is_param_list);
    for pair in times.windows(2) {
        let gap = pair[1].duration_since(pair[0]);
        assert!(
            gap >= t.param_list_quiet * 4 / 5,
            "whole-list retries {gap:?} apart; the quiet timeout is {:?}",
            t.param_list_quiet
        );
    }
    assert_eq!(peer.sent(param_read_index), (10..COUNT).collect::<Vec<_>>());
    assert!(
        took < t.param_list_quiet * 3 + t.param_list_round * 3 + Duration::from_millis(500),
        "took {took:?}"
    );
}

/// A hole the vehicle never fills: the "incomplete" outcome is visible at once and is not a hang.
///
/// The C# never gives up by itself - its loop runs `while (indexsreceived.Count < param_total)`
/// (:2226) until the operator presses Cancel (:2104-2112) - and neither does this: the state says
/// `Recovering` from the moment the stream ended short, the table says which index is missing,
/// and the hole is asked for again every other round until [`Link::cancel_param_download`].
/// Every *other* round because of the C#'s wrap: a round that starts past the last hole finds
/// nothing and resets to the top (:2201-2205), and that round is spent. After Cancel, nothing.
#[test]
fn a_hole_never_filled_is_asked_for_until_cancelled() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = link(t);
    const COUNT: u16 = 30;
    let hole = 17;

    link.download_params(VEHICLE);
    let took = drive(
        &mut peer,
        |peer, message| {
            if is_param_list(&message) {
                let stream: Vec<MavMessage> = (0..COUNT)
                    .filter(|i| *i != hole)
                    .map(|i| numbered(i, COUNT))
                    .collect();
                peer.send_all(&stream);
            }
        },
        || {
            matches!(
                download_state(&link),
                Some(ParamDownloadState::Recovering { .. })
            )
        },
    );
    assert!(took < t.param_list_quiet, "recovery started after {took:?}");

    let window = t.param_list_round * 10;
    let asked = peer.collect(window);
    let reads: Vec<u16> = asked.iter().filter_map(param_read_index).collect();
    assert!(reads.iter().all(|index| *index == hole), "{reads:?}");
    // Ten rounds, one read every other round: five at most, give or take a round at the ends,
    // and fewer only if a loaded machine delays rounds. Still asking, and not flooding.
    assert!(
        (2..=6).contains(&reads.len()),
        "{} reads of the hole in {window:?}",
        reads.len()
    );

    let download = link.param_download(VEHICLE).unwrap();
    assert_eq!(download.missing(), vec![hole]);
    assert!(!link.params(VEHICLE).unwrap().is_complete());
    assert!(link.is_running());

    link.cancel_param_download(VEHICLE);
    assert_eq!(download_state(&link), Some(ParamDownloadState::Cancelled));
    let after = peer.collect(t.param_list_round * 4);
    assert!(
        after.iter().all(|message| !is_param_read(message)),
        "asked after Cancel: {after:?}"
    );
}

/// A lone `PARAM_VALUE` outside any download - the echo of a set, a read's answer - carries the
/// vehicle's full count. Before this machine, the link took that for a download with 899 holes
/// and asked for them, ten every 1.5 s, for ever. The C# chases holes only inside
/// `getParamListAsync`, and so does this.
#[test]
fn a_parameter_outside_a_download_starts_no_recovery() {
    let t = ProtocolTimeouts::default().faster(40);
    let (link, mut peer) = link(t);
    peer.send(&param_value("RTL_ALT", 1500.0, PARAM_TYPE_INT32, 5, 900));
    wait_for("the parameter", || link.params(VEHICLE).is_some());

    let after = peer.collect(t.param_list_quiet * 3);
    assert!(after.is_empty(), "the link asked for: {after:?}");
    assert_eq!(link.params(VEHICLE).unwrap().expected(), Some(900));
    assert!(link.param_download(VEHICLE).is_none());
}

// ================================================================================================
// Parameter set and read: setParamAsync (:1638-1776), GetParamAsync (:2296-2400)
// ================================================================================================

/// No echo: sent, then sent again three times 700 ms apart (`retrys = 3`, :1748, :1754), then
/// "Timeout on read - setParam" (:1765). Every send is the same bytes.
#[test]
fn a_set_whose_echo_never_comes_is_sent_four_times_then_times_out() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = with_rtl_alt(t);

    let started = Instant::now();
    let id = link.set_param(VEHICLE, "RTL_ALT", 2000.0, false);
    drive(&mut peer, silent, || outcome(&link, id).is_some());
    let took = started.elapsed();

    assert_eq!(outcome(&link, id), Some(RequestOutcome::TimedOut));
    assert_eq!(peer.count(is_param_set), 4);
    assert_eq!(
        u16::try_from(peer.count(is_param_set)).unwrap(),
        t.param_set.sends()
    );
    assert_eq!(link.request(id).unwrap().sends(), 4);
    let sets: Vec<MavMessage> = peer.sent(|m| is_param_set(m).then_some(*m));
    assert!(sets.windows(2).all(|pair| pair[0] == pair[1]), "{sets:?}");
    let MavMessage::ParamSet(set) = sets[0] else {
        unreachable!()
    };
    assert_eq!(
        set.param_type, PARAM_TYPE_INT32,
        "the declared type (:1661)"
    );
    assert!(
        (set.param_value - 2000.0).abs() < f32::EPSILON,
        "a float for ArduPilot (:1668)"
    );
    let budget = t.param_set.timeout * 4;
    assert!(took >= budget, "gave up after {took:?}, before {budget:?}");
    assert!(took < budget + Duration::from_millis(500), "took {took:?}");
}

/// A request is never out of sight between being queued and being picked up.
///
/// The link thread drains the queue, begins each request, and only then puts it in the table;
/// `Link::request` reads the table and then the queue. For the length of that pick-up a request
/// was in neither, and a caller that took `None` for "forgotten" gave up on a write the vehicle
/// then accepted - the GUI's parameter editor saw it. The pick-up now happens under the table's
/// lock. This queues many requests and looks each one up as fast as it can from the moment it
/// is queued until it has been sent; a single `None` fails it.
#[test]
fn a_request_is_never_missing_between_the_queue_and_the_table() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = with_rtl_alt(t);

    let mut ids = Vec::new();
    for round in 0..40u32 {
        let value = 2000.0 + f64::from(round);
        let id = link.set_param(VEHICLE, "RTL_ALT", value, false);
        let queued = Instant::now();
        loop {
            let Some(request) = link.request(id) else {
                panic!(
                    "request {round} was neither queued nor held, {:?} after queueing",
                    queued.elapsed()
                );
            };
            if request.sends() > 0 || request.outcome().is_some() {
                break;
            }
            assert!(
                queued.elapsed() < HUNG,
                "request {round} was never picked up"
            );
            std::hint::spin_loop();
        }
        ids.push(id);
    }
    drive(&mut peer, silent, || {
        ids.iter().all(|id| outcome(&link, *id).is_some())
    });
    assert!(
        ids.iter()
            .all(|id| outcome(&link, *id) == Some(RequestOutcome::TimedOut))
    );
}

/// The echo comes, late: after the first timeout, answering the first retry. One retry spent,
/// success, and nothing sent after.
#[test]
fn a_late_echo_ends_the_set_after_one_retry() {
    let t = ProtocolTimeouts::default().faster(4);
    let (link, mut peer) = with_rtl_alt(t);

    let id = link.set_param(VEHICLE, "RTL_ALT", 2000.0, false);
    let mut sets = 0;
    drive(
        &mut peer,
        |peer, message| {
            if is_param_set(&message) {
                sets += 1;
                if sets == 2 {
                    peer.send(&echo(2000.0));
                }
            }
        },
        || outcome(&link, id).is_some(),
    );
    let after = peer.collect(t.param_set.timeout * 2);

    let Some(RequestOutcome::Accepted { value: Some(value) }) = outcome(&link, id) else {
        panic!("{:?}", outcome(&link, id));
    };
    assert!((value.as_f64() - 2000.0).abs() < f64::EPSILON);
    assert_eq!(peer.count(is_param_set), 2);
    assert!(after.is_empty(), "{after:?}");
}

/// The echo carries a different value - the vehicle clamped it, or refused it. The C# takes the
/// echo as success, stores what came back and returns true (:1693-1731); so does this, with the
/// vehicle's value in the outcome and in the table, so a caller can see the difference.
#[test]
fn an_echo_of_a_different_value_is_the_answer_and_the_table_takes_it() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = with_rtl_alt(t);

    let id = link.set_param(VEHICLE, "RTL_ALT", 2000.0, false);
    drive(
        &mut peer,
        |peer, message| {
            if is_param_set(&message) {
                peer.send(&echo(1800.0));
            }
        },
        || outcome(&link, id).is_some(),
    );

    let Some(RequestOutcome::Accepted { value: Some(value) }) = outcome(&link, id) else {
        panic!("{:?}", outcome(&link, id));
    };
    assert!((value.as_f64() - 1800.0).abs() < f64::EPSILON);
    let table = link.params(VEHICLE).unwrap();
    assert!((table.get("RTL_ALT").unwrap().as_f64() - 1800.0).abs() < f64::EPSILON);
    assert_eq!(peer.count(is_param_set), 1);
}

/// An echo of some other parameter is not the answer: "MAVLINK bad param response" and keep
/// waiting (:1699-1703). All four sends go out and the set times out.
#[test]
fn an_echo_of_another_parameter_is_not_the_answer() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = with_rtl_alt(t);

    let id = link.set_param(VEHICLE, "RTL_ALT", 2000.0, false);
    drive(
        &mut peer,
        |peer, message| {
            if is_param_set(&message) {
                peer.send(&param_value(
                    "RTL_ALT_FINAL",
                    2000.0,
                    PARAM_TYPE_INT32,
                    1,
                    2,
                ));
            }
        },
        || outcome(&link, id).is_some(),
    );

    assert_eq!(outcome(&link, id), Some(RequestOutcome::TimedOut));
    assert_eq!(peer.count(is_param_set), 4);
}

/// The checks before anything is sent (:1640-1651): a name the vehicle never listed is refused,
/// and a value the table already holds is not sent - unless forced.
#[test]
fn a_set_that_cannot_or_need_not_be_sent_is_not_sent() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = with_rtl_alt(t);

    let unknown = link.set_param(VEHICLE, "NO_SUCH_PARAM", 1.0, false);
    let same = link.set_param(VEHICLE, "RTL_ALT", 1500.0, false);
    wait_for("both answered", || {
        outcome(&link, unknown).is_some() && outcome(&link, same).is_some()
    });
    assert_eq!(
        outcome(&link, unknown),
        Some(RequestOutcome::UnknownParameter)
    );
    assert_eq!(outcome(&link, same), Some(RequestOutcome::Unchanged));
    assert!(peer.collect(t.param_set.timeout).is_empty());

    let forced = link.set_param(VEHICLE, "RTL_ALT", 1500.0, true);
    drive(
        &mut peer,
        |peer, message| {
            if is_param_set(&message) {
                peer.send(&echo(1500.0));
            }
        },
        || outcome(&link, forced).is_some(),
    );
    assert!(matches!(
        outcome(&link, forced),
        Some(RequestOutcome::Accepted { .. })
    ));
    assert_eq!(peer.count(is_param_set), 1);
}

/// `GetParam` by name: four `PARAM_REQUEST_READ` (`retrys = 3`, :2329) and "Timeout on read -
/// GetParam" (:2345). An answer naming another parameter is the "Wrong Answer" (:2365-2371) and
/// does not end the wait.
#[test]
fn a_read_answered_only_wrongly_is_sent_four_times_then_times_out() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = link(t);

    let started = Instant::now();
    let id = link.read_param(VEHICLE, "RTL_ALT");
    drive(
        &mut peer,
        |peer, message| {
            if let MavMessage::ParamRequestRead(read) = message {
                assert_eq!(read.param_index, -1, "by name");
                assert_eq!(mp_params::decode_param_id(&read.param_id), "RTL_ALT");
                peer.send(&param_value("WPNAV_SPEED", 500.0, PARAM_TYPE_REAL32, 3, 9));
            }
        },
        || outcome(&link, id).is_some(),
    );
    let took = started.elapsed();

    assert_eq!(outcome(&link, id), Some(RequestOutcome::TimedOut));
    assert_eq!(peer.count(is_param_read), usize::from(t.param_read.sends()));
    assert!(took >= t.param_read.timeout * 4, "took {took:?}");
}

// ================================================================================================
// Commands: doCommandAsync (C#: MAVLinkInterface.cs:2688-2837)
// ================================================================================================

/// No ack: four `COMMAND_LONG` 2 s apart (`retrys = 3`, `timeout = 2000`, :2729-2731), each
/// retry with its confirmation counted up (:2789), then "Timeout on read - doCommand" (:2797).
#[test]
fn a_command_never_acknowledged_is_sent_four_times_with_rising_confirmation() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = link(t);

    let started = Instant::now();
    let id = link.command(VEHICLE, TAKEOFF, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0], true);
    drive(&mut peer, silent, || outcome(&link, id).is_some());
    let took = started.elapsed();

    assert_eq!(outcome(&link, id), Some(RequestOutcome::TimedOut));
    assert_eq!(
        peer.sent(command_long),
        vec![(TAKEOFF, 0), (TAKEOFF, 1), (TAKEOFF, 2), (TAKEOFF, 3)]
    );
    let budget = t.command.timeout * 4;
    assert!(
        took >= budget && took < budget + Duration::from_millis(500),
        "took {took:?}"
    );
}

/// `IN_PROGRESS`, then `ACCEPTED`. Each `IN_PROGRESS` restarts the wait and spends every retry
/// (`start = DateTime.Now; retrys = 0;`, :2818-2823): the vehicle has the command, and sending it
/// again would start it again. Here the whole exchange takes longer than one timeout, and still
/// only one command goes out.
#[test]
fn in_progress_then_accepted_is_accepted_without_a_resend() {
    // Halved, not divided by ten: each `IN_PROGRESS` comes three-fifths of a timeout after the
    // last, and at a tenth that left 80 ms before the link gave up - less than the hosted macOS
    // runner stalled a sleeping thread (CI run 37125541254, 2026-10-03, timed out). Halved, 400 ms.
    let t = ProtocolTimeouts::default().faster(2);
    let (link, mut peer) = link(t);
    let step = t.command.timeout * 3 / 5;

    let started = Instant::now();
    let id = link.command(VEHICLE, TAKEOFF, [0.0; 7], true);
    drive(
        &mut peer,
        |peer, message| {
            if command_long(&message).is_some() {
                peer.send(&command_ack(TAKEOFF, MAV_RESULT_IN_PROGRESS));
                std::thread::sleep(step);
                peer.send(&command_ack(TAKEOFF, MAV_RESULT_IN_PROGRESS));
                std::thread::sleep(step);
                peer.send(&command_ack(TAKEOFF, MAV_RESULT_ACCEPTED));
            }
        },
        || outcome(&link, id).is_some(),
    );
    let took = started.elapsed();

    assert_eq!(
        outcome(&link, id),
        Some(RequestOutcome::Accepted { value: None })
    );
    assert!(
        took > t.command.timeout,
        "the exchange outlasted one timeout: {took:?}"
    );
    assert_eq!(peer.count(|m| command_long(m).is_some()), 1);
}

/// `IN_PROGRESS`, then nothing: one more full wait, no resend, then the timeout (:2818-2823 with
/// `retrys` zero at :2786-2797).
#[test]
fn in_progress_then_silence_times_out_without_a_resend() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = link(t);

    let id = link.command(VEHICLE, TAKEOFF, [0.0; 7], true);
    let mut acked_at = None;
    drive(
        &mut peer,
        |peer, message| {
            if command_long(&message).is_some() {
                peer.send(&command_ack(TAKEOFF, MAV_RESULT_IN_PROGRESS));
                acked_at = Some(Instant::now());
            }
        },
        || outcome(&link, id).is_some(),
    );

    assert_eq!(outcome(&link, id), Some(RequestOutcome::TimedOut));
    assert_eq!(peer.count(|m| command_long(m).is_some()), 1);
    let waited = acked_at.unwrap().elapsed();
    assert!(
        waited >= t.command.timeout - Duration::from_millis(2),
        "timed out {waited:?} after IN_PROGRESS; the wait is {:?}",
        t.command.timeout
    );
}

/// Every final `MAV_RESULT` but `ACCEPTED` ends the command at once, sent once, with that result:
/// the C# returns false on anything that is neither `ACCEPTED` nor `IN_PROGRESS` (:2829-2833).
///
/// That includes `TEMPORARILY_REJECTED`, which MAVLink documents as "retrying later should work".
/// Mission Planner does not retry it, and neither does this: a retry would be a new behaviour,
/// and for an arm refused because the EKF is still settling, an automatic re-arm a few seconds
/// later is not something to add unasked. Also covered: 6 (`CANCELLED` in newer dialects,
/// unknown to this one) and an out-of-range 200.
#[test]
fn every_refusal_ends_the_command_at_once_without_a_retry() {
    let t = ProtocolTimeouts::default().faster(4);
    let (link, mut peer) = link(t);

    for result in [
        MAV_RESULT_TEMPORARILY_REJECTED,
        2,
        3,
        4,
        6,
        7,
        8,
        200,
        MAV_RESULT_ACCEPTED,
    ] {
        let before = peer.count(|m| command_long(m).is_some());
        let id = link.command(VEHICLE, TAKEOFF, [0.0; 7], true);
        let took = drive(
            &mut peer,
            |peer, message| {
                if command_long(&message).is_some() {
                    peer.send(&command_ack(TAKEOFF, result));
                }
            },
            || outcome(&link, id).is_some(),
        );
        let expected = if result == MAV_RESULT_ACCEPTED {
            RequestOutcome::Accepted { value: None }
        } else {
            RequestOutcome::Rejected(result)
        };
        assert_eq!(outcome(&link, id), Some(expected), "result {result}");
        assert_eq!(
            peer.count(|m| command_long(m).is_some()) - before,
            1,
            "result {result} was retried"
        );
        assert!(took < t.command.timeout, "result {result} took {took:?}");
    }
}

/// An ack for another command, or for this command from another system or component, is not the
/// answer (:2800-2813: the source must be the target, and "Commands dont match").
#[test]
fn an_ack_for_another_command_or_from_another_vehicle_is_not_the_answer() {
    let t = ProtocolTimeouts::default().faster(4);
    let (link, mut peer) = link(t);

    let id = link.command(VEHICLE, TAKEOFF, [0.0; 7], true);
    let mut early = Vec::new();
    drive(
        &mut peer,
        |peer, message| {
            if command_long(&message).is_some() {
                peer.send(&command_ack(21, MAV_RESULT_ACCEPTED));
                peer.send_from(VehicleId::new(2, 1), &command_ack(TAKEOFF, 4));
                peer.send_from(VehicleId::new(1, 154), &command_ack(TAKEOFF, 4));
                std::thread::sleep(Duration::from_millis(20));
                early.push(outcome(&link, id));
                peer.send(&command_ack(TAKEOFF, MAV_RESULT_ACCEPTED));
            }
        },
        || outcome(&link, id).is_some(),
    );

    assert_eq!(early, vec![None], "a stray ack ended the command");
    assert_eq!(
        outcome(&link, id),
        Some(RequestOutcome::Accepted { value: None })
    );
    assert_eq!(peer.count(|m| command_long(m).is_some()), 1);
}

/// Two commands in flight at once, acknowledged in the other order: each ack answers its own
/// command, matched by command id as the C# matches (:2808-2813), and a second ack for a command
/// already answered answers nothing.
///
/// The C# never has two in flight - `giveComport` serialises every request - but this link lets a
/// caller issue a command without waiting for the last, so the order acks arrive in is the
/// vehicle's, not the caller's.
#[test]
fn acks_arriving_in_the_other_order_answer_their_own_commands() {
    let t = ProtocolTimeouts::default().faster(4);
    let (link, mut peer) = link(t);
    let arm = commands::CMD_COMPONENT_ARM_DISARM;

    let takeoff = link.command(VEHICLE, TAKEOFF, [0.0; 7], true);
    let arming = link.command(VEHICLE, arm, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], true);
    let mut seen = Vec::new();
    drive(
        &mut peer,
        |peer, message| {
            if let Some((command, _)) = command_long(&message) {
                seen.push(command);
                if seen.len() == 2 {
                    peer.send(&command_ack(arm, MAV_RESULT_ACCEPTED));
                    peer.send(&command_ack(TAKEOFF, 2));
                    peer.send(&command_ack(arm, 4));
                }
            }
        },
        || outcome(&link, takeoff).is_some() && outcome(&link, arming).is_some(),
    );

    assert_eq!(seen, vec![TAKEOFF, arm], "sent in the order asked");
    assert_eq!(
        outcome(&link, arming),
        Some(RequestOutcome::Accepted { value: None })
    );
    assert_eq!(outcome(&link, takeoff), Some(RequestOutcome::Rejected(2)));
}

/// Arming waits ten seconds, not two, "as may need an imu calib" (:2764-2768).
#[test]
fn arming_waits_the_longer_arm_timeout_before_retrying() {
    let t = ProtocolTimeouts::default().faster(40);
    let (link, mut peer) = link(t);
    assert!(t.command_arm.timeout > t.command.timeout * 4);

    let id = link.command(
        VEHICLE,
        commands::CMD_COMPONENT_ARM_DISARM,
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        true,
    );
    let mut sends = 0;
    drive(
        &mut peer,
        |peer, message| {
            if command_long(&message).is_some() {
                sends += 1;
                if sends == 2 {
                    peer.send(&command_ack(
                        commands::CMD_COMPONENT_ARM_DISARM,
                        MAV_RESULT_ACCEPTED,
                    ));
                }
            }
        },
        || outcome(&link, id).is_some(),
    );

    let times = peer.times(|m| command_long(m).is_some());
    assert_eq!(times.len(), 2);
    let gap = times[1].duration_since(times[0]);
    assert!(
        gap >= t.command_arm.timeout * 4 / 5,
        "the arm retry came {gap:?} after; the arm wait is {:?}, the ordinary one {:?}",
        t.command_arm.timeout,
        t.command.timeout
    );
}

/// A calibration waits 25 s and is sent twice at most (`retrys = 1; timeout = 25000;`,
/// :2748-2752), as is a bootloader flash (:2753-2757).
#[test]
fn a_calibration_is_sent_twice_at_most() {
    let t = ProtocolTimeouts::default().faster(100);
    let (link, mut peer) = link(t);

    let started = Instant::now();
    let id = link.command(
        VEHICLE,
        CMD_PREFLIGHT_CALIBRATION,
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        true,
    );
    drive(&mut peer, silent, || outcome(&link, id).is_some());
    let took = started.elapsed();

    assert_eq!(outcome(&link, id), Some(RequestOutcome::TimedOut));
    assert_eq!(peer.count(|m| command_long(m).is_some()), 2);
    assert!(took >= t.command_slow.timeout * 2, "took {took:?}");
}

/// The commands the C# sends and does not wait for, each returning true at once: `requireack`
/// false (:2720-2724); a reboot, sent twice "just incase" (:2758-2763); `GET_HOME_POSITION`
/// (:2769-2773); the accelerometer calibration that blocks the autopilot (p5 = 1, :2734-2739);
/// compassmot, sent twice (p6 = 1, :2740-2747). None is ever resent.
#[test]
fn the_commands_not_waited_for_are_sent_and_forgotten() {
    let t = ProtocolTimeouts::default().faster(40);
    let (link, mut peer) = link(t);

    let cases: [(u16, [f32; 7], bool, usize); 5] = [
        (TAKEOFF, [0.0; 7], false, 1),
        (
            CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            true,
            2,
        ),
        (CMD_GET_HOME_POSITION, [0.0; 7], true, 1),
        (
            CMD_PREFLIGHT_CALIBRATION,
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            true,
            1,
        ),
        (
            CMD_PREFLIGHT_CALIBRATION,
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            true,
            2,
        ),
    ];
    for (command, params, require_ack, sends) in cases {
        let before = peer.count(|m| command_long(m).is_some());
        let id = link.command(VEHICLE, command, params, require_ack);
        wait_for("the command", || outcome(&link, id).is_some());
        assert_eq!(
            outcome(&link, id),
            Some(RequestOutcome::Sent),
            "command {command}"
        );
        let _ = peer.collect(t.command.timeout * 3 / 2);
        assert_eq!(
            peer.count(|m| command_long(m).is_some()) - before,
            sends,
            "command {command} {params:?}"
        );
    }
}

/// `setWPCurrent`: six `MISSION_SET_CURRENT` 2 s apart (`retrys = 5`, :2472-2476), then "Timeout
/// on read - setWPCurrent" (:2488). And any `MISSION_CURRENT` from the vehicle ends it
/// (:2482-2487) - the C# does not check the sequence number, and nor does this.
#[test]
fn set_current_is_sent_six_times_then_times_out_and_any_mission_current_ends_it() {
    let t = ProtocolTimeouts::default().faster(40);
    let (link, mut peer) = link(t);
    let is_set_current = |m: &MavMessage| matches!(m, MavMessage::MissionSetCurrent(_));

    let started = Instant::now();
    let id = link.set_current_waypoint(VEHICLE, 3);
    drive(&mut peer, silent, || outcome(&link, id).is_some());
    let took = started.elapsed();
    assert_eq!(outcome(&link, id), Some(RequestOutcome::TimedOut));
    assert_eq!(
        peer.count(is_set_current),
        usize::from(t.set_current.sends())
    );
    assert_eq!(peer.count(is_set_current), 6);
    assert!(took >= t.set_current.timeout * 6, "took {took:?}");

    let answered = link.set_current_waypoint(VEHICLE, 3);
    let before = peer.count(is_set_current);
    let mut seen = 0;
    drive(
        &mut peer,
        |peer, message| {
            if is_set_current(&message) {
                seen += 1;
                if seen == 3 {
                    peer.send(&MavMessage::MissionCurrent(MissionCurrent {
                        seq: 1,
                        total: 5,
                        mission_state: 0,
                        mission_mode: 0,
                    }));
                }
            }
        },
        || outcome(&link, answered).is_some(),
    );
    assert_eq!(
        outcome(&link, answered),
        Some(RequestOutcome::Accepted { value: None })
    );
    assert_eq!(peer.count(is_set_current) - before, 3);
}

// ================================================================================================
// Mission upload: setWPTotalAsync, setWPAsync, mav_mission.upload
// ================================================================================================

/// Starts an upload of `items` and plays the vehicle with `script` until the transfer stops.
fn upload(
    link: &Link,
    peer: &mut Peer,
    items: Vec<MissionItem>,
    script: impl FnMut(&mut Peer, MavMessage),
) -> Duration {
    // Timed from before the upload is queued: the link may send its first message before this
    // thread sees the transfer start, and a lower bound on the time taken must include that.
    let started = Instant::now();
    link.upload_mission(VEHICLE, items);
    // The previous transfer, if any, is replaced when the link picks the new one up.
    wait_for("the upload to start", || {
        matches!(transfer_state(link), Some(TransferState::Uploading { .. }))
    });
    drive(peer, script, || transfer_finished(link));
    let took = started.elapsed();
    // What the machine sends as it finishes - the closing ack - goes out just after the state
    // changes; wait for it so the test sees the whole exchange.
    let _ = peer.collect(Duration::from_millis(10));
    took
}

/// Answers the count and each item by asking for the next, in order, with `ack` after the last.
fn in_order(count: u16, ack: u8) -> impl FnMut(&mut Peer, MavMessage) {
    let mut done = vec![false; usize::from(count)];
    move |peer, message| match message {
        MavMessage::MissionCount(_) => peer.send(&mission_request(0)),
        MavMessage::MissionItemInt(item) if !done[usize::from(item.seq)] => {
            done[usize::from(item.seq)] = true;
            if item.seq + 1 < count {
                peer.send(&mission_request(item.seq + 1));
            } else {
                peer.send(&mission_ack(ack));
            }
        }
        _ => {}
    }
}

/// The vehicle asks for every item twice - a duplicated request on a radio that retransmits - and
/// asks in both forms, `MISSION_REQUEST` and `MISSION_REQUEST_INT`. Each item goes out twice (the
/// second costing one of its ten retries), the answer is always `MISSION_ITEM_INT`, and the upload
/// completes with the C#'s closing `setWPACK` (mav_mission.cs:151).
#[test]
fn an_upload_asked_for_each_item_twice_in_either_form_completes() {
    let t = ProtocolTimeouts::default().faster(2);
    let (link, mut peer) = link(t);
    const COUNT: u16 = 5;
    let ask_twice = |peer: &mut Peer, seq: u16| {
        let request = if seq.is_multiple_of(2) {
            mission_request(seq)
        } else {
            mission_request_float(seq)
        };
        peer.send(&request);
        peer.send(&request);
    };

    let mut seen = [false; COUNT as usize];
    let took = upload(
        &link,
        &mut peer,
        mission(COUNT),
        |peer, message| match message {
            MavMessage::MissionCount(count) => {
                assert_eq!(count.count, COUNT);
                ask_twice(peer, 0);
            }
            MavMessage::MissionItemInt(item) if !seen[usize::from(item.seq)] => {
                seen[usize::from(item.seq)] = true;
                if item.seq + 1 < COUNT {
                    ask_twice(peer, item.seq + 1);
                } else {
                    peer.send(&mission_ack(MISSION_ACCEPTED));
                }
            }
            _ => {}
        },
    );
    let after = peer.collect(Duration::from_millis(20));

    assert_eq!(transfer_state(&link), Some(TransferState::Complete));
    let mut sent = peer.sent(item_sent);
    sent.sort_unstable();
    assert_eq!(sent, vec![0, 0, 1, 1, 2, 2, 3, 3, 4, 4]);
    assert_eq!(
        peer.count(|m| matches!(m, MavMessage::MissionItem(_))),
        0,
        "always _INT"
    );
    assert_eq!(peer.count(|m| matches!(m, MavMessage::MissionCount(_))), 1);
    assert_eq!(
        peer.sent(ack_sent),
        vec![MISSION_ACCEPTED],
        "setWPACK, once"
    );
    assert!(after.iter().all(|m| item_sent(m).is_none()), "{after:?}");
    assert!(took < t.mission_item_send.timeout * 3, "took {took:?}");
}

/// The vehicle asks for items out of order. Each is answered with the item asked for.
///
/// DIVERGENCE: the C#'s `setWP` waits for a request for exactly the next item and answers any
/// other request by sending *its* item again (C#: MAVLinkInterface.cs:4318-4356), so a vehicle
/// asking for item 2 while the C# is on item 0 is sent item 0. The MAVLink protocol has the
/// vehicle drive; this port answers what was asked. Against ArduPilot, which asks in order, the
/// wire is the same.
#[test]
fn an_upload_asked_out_of_order_sends_what_was_asked() {
    let t = ProtocolTimeouts::default().faster(2);
    let (link, mut peer) = link(t);
    let order = [0u16, 2, 1, 4, 3];
    let mut next = 0;

    upload(&link, &mut peer, mission(5), |peer, message| {
        let asked = matches!(message, MavMessage::MissionCount(_))
            || matches!(message, MavMessage::MissionItemInt(_));
        if asked {
            if let Some(seq) = order.get(next) {
                peer.send(&mission_request(*seq));
                next += 1;
            } else {
                peer.send(&mission_ack(MISSION_ACCEPTED));
            }
        }
    });

    assert_eq!(transfer_state(&link), Some(TransferState::Complete));
    assert_eq!(peer.sent(item_sent), order.to_vec());
}

/// The vehicle skips an item and then says `ACCEPTED`. A clean failure naming the item.
///
/// DIVERGENCE: the C# is waiting for a request for item 2 when the request for 3 arrives, and
/// sends item 1 again for it (C#: MAVLinkInterface.cs:4318-4356), until its ten retries run out
/// and it throws "Timeout on read - setWP" (:4267) - a failure too, but five seconds later and
/// blaming the link. A vehicle that accepts a mission it was never sent all of is not holding
/// the mission on screen; it fails here at once, with the item.
#[test]
fn an_upload_accepted_without_every_item_asked_for_fails_naming_the_item() {
    let t = ProtocolTimeouts::default().faster(2);
    let (link, mut peer) = link(t);
    let order = [0u16, 1, 3, 4];
    let mut next = 0;

    let took = upload(&link, &mut peer, mission(5), |peer, message| {
        let asked = matches!(message, MavMessage::MissionCount(_))
            || matches!(message, MavMessage::MissionItemInt(_));
        if asked {
            if let Some(seq) = order.get(next) {
                peer.send(&mission_request(*seq));
                next += 1;
            } else {
                peer.send(&mission_ack(MISSION_ACCEPTED));
            }
        }
    });

    let Some(TransferState::Failed(failure)) = transfer_state(&link) else {
        panic!("{:?}", transfer_state(&link));
    };
    assert_eq!(failure, TransferFailure::NeverRequested(2));
    assert_eq!(
        failure.to_string(),
        "the vehicle accepted the mission without asking for item 2"
    );
    assert_eq!(peer.sent(item_sent), order.to_vec());
    assert!(
        peer.sent(ack_sent).is_empty(),
        "no setWPACK for a failed upload"
    );
    assert!(took < t.mission_item_send.timeout, "took {took:?}");
}

/// Every `MAV_MISSION_RESULT` as the vehicle's final answer, and what `mav_mission.upload` does
/// with each (C#: ExtLibs/ArduPilot/mav_mission.cs:101-148):
///
/// * `ACCEPTED` (0): complete, then `setWPACK` (:151).
/// * `ERROR` (1): `MISSION_WRITE_PARTIAL_LIST` from the refused item to the mission's length and
///   the item once more (:104-111). Answered `ACCEPTED`, the upload completes.
/// * `NO_SPACE` (4) and `INVALID` (5): their own messages (:113-122).
/// * `INVALID_SEQUENCE` (13): resume from the vehicle's next request (:123-143). The vehicle asks
///   for the last item again and accepts it: complete.
/// * Everything else, including a value no dialect defines: "Upload MISSION failed WAYPOINT
///   <result>" (:144-148).
#[test]
fn every_mission_result_on_the_final_ack_ends_the_upload_as_the_csharp_does() {
    let t = ProtocolTimeouts::default().faster(2);
    let (link, mut peer) = link(t);
    const COUNT: u16 = 3;
    let last = COUNT - 1;

    for result in (0u8..=15).chain([99]) {
        let _ = peer.collect(Duration::from_millis(10));
        let mark = peer.log.len();
        let mut acked = false;
        let mut in_order = in_order(COUNT, result);
        upload(&link, &mut peer, mission(COUNT), |peer, message| {
            let retry = matches!(result, MISSION_ERROR | MISSION_INVALID_SEQUENCE);
            match message {
                // The one retry either result earns: ask for the item again, then accept it.
                MavMessage::MissionWritePartialList(partial) if result == MISSION_ERROR => {
                    assert_eq!((partial.start_index, partial.end_index), (2, 3));
                    peer.send(&mission_request(last));
                }
                MavMessage::MissionItemInt(item) if retry && acked && item.seq == last => {
                    peer.send(&mission_ack(MISSION_ACCEPTED));
                }
                MavMessage::MissionItemInt(item) if item.seq == last && !acked => {
                    acked = true;
                    in_order(peer, message);
                    if result == MISSION_INVALID_SEQUENCE {
                        peer.send(&mission_request(last));
                    }
                }
                _ => in_order(peer, message),
            }
        });

        let state = transfer_state(&link).unwrap();
        let since: Vec<MavMessage> = peer.log[mark..].iter().map(|(_, m)| *m).collect();
        let partials = since.iter().filter_map(partial_list).count();
        let last_sends = since.iter().filter(|m| item_sent(m) == Some(last)).count();
        let expected = match result {
            0 => Ok(()),
            MISSION_ERROR => {
                assert_eq!(partials, 1, "one partial re-upload for ERROR");
                assert_eq!(last_sends, 2);
                Ok(())
            }
            MISSION_INVALID_SEQUENCE => {
                assert_eq!(partials, 0, "the vehicle said where to resume in time");
                assert_eq!(last_sends, 2);
                Ok(())
            }
            MISSION_NO_SPACE => Err("Upload failed, please reduce the number of wp's".to_owned()),
            5 => Err(
                "Upload failed, mission was rejected by the Mav,\n item had a bad option wp# 2 MAV_MISSION_INVALID"
                    .to_owned(),
            ),
            99 => Err("Upload MISSION failed WAYPOINT 99".to_owned()),
            other => Err(format!(
                "Upload MISSION failed WAYPOINT {}",
                mp_mavlink_dialects::all::MavMissionResult(u32::from(other))
                    .name()
                    .unwrap()
            )),
        };
        match expected {
            Ok(()) => {
                assert_eq!(state, TransferState::Complete, "result {result}");
                assert_eq!(
                    since.iter().filter_map(ack_sent).collect::<Vec<_>>(),
                    vec![MISSION_ACCEPTED],
                    "result {result}: setWPACK"
                );
            }
            Err(message) => {
                let TransferState::Failed(failure) = state else {
                    panic!("result {result}: {state:?}");
                };
                assert_eq!(failure.to_string(), message, "result {result}");
                assert_eq!(partials, 0, "result {result} was retried");
                assert_eq!(last_sends, 1, "result {result} was retried");
            }
        }
    }
}

/// `MAV_MISSION_ERROR` twice for the same item: the one partial re-upload is spent, and the
/// second is the generic failure (mav_mission.cs:104-111 then :144-148).
#[test]
fn a_second_error_for_the_same_item_fails_the_upload() {
    let t = ProtocolTimeouts::default().faster(2);
    let (link, mut peer) = link(t);
    let mut in_order = in_order(3, MISSION_ERROR);

    upload(
        &link,
        &mut peer,
        mission(3),
        |peer, message| match message {
            MavMessage::MissionWritePartialList(_) => peer.send(&mission_request(2)),
            MavMessage::MissionItemInt(item)
                if item.seq == 2 && peer.count(|m| item_sent(m) == Some(2)) == 2 =>
            {
                peer.send(&mission_ack(MISSION_ERROR));
            }
            _ => in_order(peer, message),
        },
    );

    let Some(TransferState::Failed(failure)) = transfer_state(&link) else {
        panic!("{:?}", transfer_state(&link));
    };
    assert_eq!(
        failure.to_string(),
        "Upload MISSION failed WAYPOINT MAV_MISSION_ERROR"
    );
    assert_eq!(peer.sent(partial_list), vec![(2, 3)]);
}

/// The count is never answered: four `MISSION_COUNT` 700 ms apart (`retrys = 3`, :3779-3783),
/// then "Timeout on read - setWPTotal" (:3795).
#[test]
fn an_upload_whose_count_is_never_answered_is_sent_four_times() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = link(t);

    let took = upload(&link, &mut peer, mission(3), silent);

    let Some(TransferState::Failed(failure)) = transfer_state(&link) else {
        panic!("{:?}", transfer_state(&link));
    };
    assert_eq!(failure, TransferFailure::TimedOut(TransferStep::SendCount));
    assert_eq!(failure.to_string(), "Timeout on read - setWPTotal");
    assert_eq!(
        peer.count(|m| matches!(m, MavMessage::MissionCount(_))),
        usize::from(t.mission_count.sends())
    );
    assert!(took >= t.mission_count.timeout * 4, "took {took:?}");
}

/// An item sent and never followed by a request or an ack: sent again every 450 ms, ten times
/// (`retrys = 10`, :4250-4254), then "Timeout on read - setWP" (:4267).
#[test]
fn an_item_never_followed_up_is_sent_eleven_times() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = link(t);

    let took = upload(&link, &mut peer, mission(3), |peer, message| {
        if matches!(message, MavMessage::MissionCount(_)) {
            peer.send(&mission_request(0));
        }
    });

    assert_eq!(
        transfer_state(&link),
        Some(TransferState::Failed(TransferFailure::TimedOut(
            TransferStep::SendItem
        )))
    );
    assert_eq!(
        peer.sent(item_sent),
        vec![0; usize::from(t.mission_item_send.sends())]
    );
    assert_eq!(t.mission_item_send.sends(), 11);
    assert!(took >= t.mission_item_send.timeout * 11, "took {took:?}");
}

/// A vehicle stuck asking for the same item: each request is answered, and each answer after the
/// first is one of the item's ten retries (the C#'s `start = DateTime.MinValue`, :4354), so the
/// transfer ends after eleven sends rather than running as long as the vehicle does.
#[test]
fn a_vehicle_stuck_on_one_item_uses_up_that_items_retries() {
    let t = ProtocolTimeouts::default().faster(4);
    let (link, mut peer) = link(t);

    upload(&link, &mut peer, mission(3), |peer, message| {
        if matches!(
            message,
            MavMessage::MissionCount(_) | MavMessage::MissionItemInt(_)
        ) {
            peer.send(&mission_request(0));
        }
    });

    assert_eq!(
        transfer_state(&link),
        Some(TransferState::Failed(TransferFailure::TimedOut(
            TransferStep::SendItem
        )))
    );
    assert_eq!(peer.sent(item_sent), vec![0; 11]);
}

/// The vehicle refuses the count itself - ArduPilot's answer to a mission bigger than it can
/// store. The refusal ends the upload at once, with nothing sent.
///
/// DIVERGENCE: the C#'s `setWPTotal` returns on any `MISSION_ACK` without reading it
/// (C#: MAVLinkInterface.cs:3863-3876), so `mav_mission.upload` goes on to send item 0 to a
/// vehicle that is no longer receiving. What it reports then depends on how the vehicle answers
/// an item it did not ask for - "Timeout on read - setWP" (:4267) after ten retries if it says
/// nothing, as this scripted vehicle would - and not that the mission was too big.
#[test]
fn a_refused_count_ends_the_upload_with_the_refusal() {
    let t = ProtocolTimeouts::default().faster(2);
    let (link, mut peer) = link(t);

    let took = upload(&link, &mut peer, mission(3), |peer, message| {
        if matches!(message, MavMessage::MissionCount(_)) {
            peer.send(&mission_ack(MISSION_NO_SPACE));
        }
    });

    assert_eq!(
        transfer_state(&link),
        Some(TransferState::Failed(TransferFailure::NoSpace))
    );
    assert!(peer.sent(item_sent).is_empty());
    assert!(took < t.mission_count.timeout, "took {took:?}");
}

/// `INVALID_SEQUENCE` on the last item, and then nothing. After the 1.5 s `getRequestedWPNo`
/// wait (:4380) the partial list goes out (mav_mission.cs:135-139) and the item's own retries run
/// out: a timeout.
///
/// DIVERGENCE: the C# `continue`s after sending the partial list (mav_mission.cs:142), which
/// moves its loop past the item it meant to resend; on the last item that ends the loop, sends
/// `setWPACK` and returns - the upload reported a success the vehicle refused. Kept as a failure.
#[test]
fn invalid_sequence_then_silence_is_a_timeout_not_a_success() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = link(t);
    // Asks in order and acks the last item INVALID_SEQUENCE, then ignores everything: repeats of
    // an item and the partial list alike.
    let took = upload(
        &link,
        &mut peer,
        mission(3),
        in_order(3, MISSION_INVALID_SEQUENCE),
    );

    assert_eq!(
        transfer_state(&link),
        Some(TransferState::Failed(TransferFailure::TimedOut(
            TransferStep::SendItem
        )))
    );
    assert_eq!(peer.sent(partial_list), vec![(2, 3)]);
    assert_eq!(peer.count(|m| item_sent(m) == Some(2)), 11);
    assert!(
        peer.sent(ack_sent).is_empty(),
        "no setWPACK: this upload failed"
    );
    let budget = t.mission_resync + t.mission_item_send.timeout * 11;
    assert!(took >= budget, "took {took:?}, less than {budget:?}");
}

// ================================================================================================
// Mission download: getWPCountAsync, getWPAsync, mav_mission.download
// ================================================================================================

/// Starts a download and plays the vehicle with `script` until the transfer stops.
fn download(link: &Link, peer: &mut Peer, script: impl FnMut(&mut Peer, MavMessage)) -> Duration {
    let started = Instant::now();
    link.download_mission(VEHICLE);
    wait_for("the download to start", || {
        matches!(
            transfer_state(link),
            Some(TransferState::AwaitingCount | TransferState::Downloading { .. })
        )
    });
    drive(peer, script, || transfer_finished(link));
    let took = started.elapsed();
    let _ = peer.collect(Duration::from_millis(10));
    took
}

/// What a vehicle holding `items` answers to a download request, if anything.
fn answer(items: &[MissionItem], message: &MavMessage) -> Option<MavMessage> {
    match message {
        MavMessage::MissionRequestList(_) => Some(commands::send_mission_count(
            GCS,
            u16::try_from(items.len()).unwrap(),
            MISSION_TYPE_MISSION,
        )),
        MavMessage::MissionRequestInt(request) => items
            .get(usize::from(request.seq))
            .map(|item| commands::send_mission_item(GCS, item, MISSION_TYPE_MISSION)),
        _ => None,
    }
}

/// A vehicle that answers every request for its mission, unless `drop` says not to.
fn vehicle_holding(
    items: Vec<MissionItem>,
    mut drop: impl FnMut(u16) -> bool,
) -> impl FnMut(&mut Peer, MavMessage) {
    move |peer, message| {
        if let MavMessage::MissionRequestInt(request) = message
            && drop(request.seq)
        {
            return;
        }
        if let Some(reply) = answer(&items, &message) {
            peer.send(&reply);
        }
    }
}

/// One item's reply is lost: after 2.5 s it is asked for again (:3459-3478) and the download
/// completes, with `setWPACK` (mav_mission.cs:50). Every other item is asked for once.
#[test]
fn a_download_with_a_lost_item_asks_again_and_completes() {
    let t = ProtocolTimeouts::default().faster(10);
    let (link, mut peer) = link(t);
    let items = mission(5);
    let mut dropped = false;

    let took = download(
        &link,
        &mut peer,
        vehicle_holding(items.clone(), |seq| {
            let drop = seq == 2 && !dropped;
            dropped |= drop;
            drop
        }),
    );

    assert_eq!(transfer_state(&link), Some(TransferState::Complete));
    assert_eq!(
        link.mission_transfer(VEHICLE).unwrap().items(),
        items.as_slice()
    );
    assert_eq!(peer.sent(item_requested), vec![0, 1, 2, 2, 3, 4]);
    assert_eq!(peer.sent(ack_sent), vec![MISSION_ACCEPTED]);
    assert!(took >= t.mission_item_request.timeout, "took {took:?}");
}

/// One item never arrives: asked for six times (`retrys = 5`, :3459), then "Timeout on read -
/// getWP" (:3478). What arrived before it is kept; no ack is sent for a partial mission.
#[test]
fn a_download_whose_item_never_arrives_asks_six_times_then_times_out() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = link(t);

    let took = download(
        &link,
        &mut peer,
        vehicle_holding(mission(5), |seq| seq == 3),
    );

    let Some(TransferState::Failed(failure)) = transfer_state(&link) else {
        panic!("{:?}", transfer_state(&link));
    };
    assert_eq!(failure.to_string(), "Timeout on read - getWP");
    assert_eq!(peer.count(|m| item_requested(m) == Some(3)), 6);
    assert_eq!(usize::from(t.mission_item_request.sends()), 6);
    assert_eq!(link.mission_transfer(VEHICLE).unwrap().items().len(), 3);
    assert!(peer.sent(ack_sent).is_empty());
    let budget = t.mission_item_request.timeout * 6;
    assert!(
        took >= budget && took < budget + Duration::from_millis(500),
        "took {took:?}"
    );
}

/// The count never arrives: `MISSION_REQUEST_LIST` seven times (`retrys = 6`, :3297), then
/// "Timeout on read - getWPCount" (:3314).
#[test]
fn a_download_whose_count_never_arrives_asks_seven_times_then_times_out() {
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = link(t);

    let took = download(&link, &mut peer, silent);

    assert_eq!(
        transfer_state(&link),
        Some(TransferState::Failed(TransferFailure::TimedOut(
            TransferStep::RequestList
        )))
    );
    assert_eq!(
        peer.count(|m| matches!(m, MavMessage::MissionRequestList(_))),
        usize::from(t.mission_list.sends())
    );
    assert_eq!(t.mission_list.sends(), 7);
    assert!(took >= t.mission_list.timeout * 7, "took {took:?}");
}

/// Every item arrives twice. Each is asked for once, and the mission is right.
///
/// DIVERGENCE: the C#'s `getWP` answers any item it did not ask for with another request, at
/// once (C#: MAVLinkInterface.cs:3530-3534). For a duplicate of an item already held that is an
/// echo chamber: the extra request draws two more replies, one of them another duplicate, and the
/// requests double per item - 2^n for an n-item mission over a link that repeats frames. A stale
/// item is dropped here; only an item nobody asked for gets the C#'s immediate request (next test).
#[test]
fn a_download_whose_items_all_arrive_twice_asks_for_each_once() {
    let t = ProtocolTimeouts::default().faster(2);
    let (link, mut peer) = link(t);
    let items = mission(6);
    let mut holding = vehicle_holding(items.clone(), |_| false);

    download(&link, &mut peer, |peer, message| {
        let twice = matches!(message, MavMessage::MissionRequestInt(_));
        holding(peer, message);
        if twice {
            holding(peer, message);
        }
    });

    assert_eq!(transfer_state(&link), Some(TransferState::Complete));
    assert_eq!(
        link.mission_transfer(VEHICLE).unwrap().items(),
        items.as_slice()
    );
    assert_eq!(peer.sent(item_requested), vec![0, 1, 2, 3, 4, 5]);
}

/// An item nobody asked for - item 3 in answer to a request for 1 - is not kept, and the item
/// still needed is asked for again at once, as the C# does (:3530-3534).
#[test]
fn a_download_sent_an_item_from_the_future_asks_again_for_the_right_one() {
    let t = ProtocolTimeouts::default().faster(2);
    let (link, mut peer) = link(t);
    let items = mission(5);
    let mut holding = vehicle_holding(items.clone(), |_| false);
    let mut jumped = false;

    let took = download(&link, &mut peer, |peer, message| {
        if item_requested(&message) == Some(1) && !jumped {
            jumped = true;
            peer.send(&commands::send_mission_item(
                GCS,
                &items[3],
                MISSION_TYPE_MISSION,
            ));
        }
        holding(peer, message);
    });

    assert_eq!(transfer_state(&link), Some(TransferState::Complete));
    assert_eq!(
        link.mission_transfer(VEHICLE).unwrap().items(),
        items.as_slice()
    );
    assert_eq!(peer.count(|m| item_requested(m) == Some(1)), 2);
    assert!(
        took < t.mission_item_request.timeout,
        "no timeout was needed: {took:?}"
    );
}

// ================================================================================================
// Adversarial links: seeded drop, duplicate and delay-past-timeout (PLAN.md §7.3)
// ================================================================================================

/// A deterministic coin, so a failure names a seed that reproduces it.
struct Coin(u64);

impl Coin {
    /// True with probability `percent` in a hundred.
    fn flip(&mut self, percent: u64) -> bool {
        // Knuth's MMIX LCG; the high bits are the good ones.
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 33) % 100 < percent
    }
}

/// A vehicle over a bad link: each reply is dropped, duplicated, or held back past the link's
/// wait and delivered late, at random but from a seed.
struct BadLink {
    coin: Coin,
    late: Vec<(Instant, MavMessage)>,
    hold: Duration,
    /// Replies dropped, held back, and sent twice: printed, so a run shows the faults happened.
    dropped: usize,
    delayed: usize,
    duplicated: usize,
}

impl BadLink {
    fn new(seed: u64, hold: Duration) -> Self {
        Self {
            coin: Coin(seed),
            late: Vec::new(),
            hold,
            dropped: 0,
            delayed: 0,
            duplicated: 0,
        }
    }

    fn deliver(&mut self, peer: &mut Peer, reply: MavMessage) {
        if self.coin.flip(15) {
            self.dropped += 1;
            return;
        }
        if self.coin.flip(15) {
            self.delayed += 1;
            self.late.push((Instant::now() + self.hold, reply));
            return;
        }
        peer.send(&reply);
        if self.coin.flip(15) {
            self.duplicated += 1;
            peer.send(&reply);
        }
    }

    /// Prints what this run did, and adds it to `totals`: dropped, delayed, duplicated.
    fn report(
        &self,
        what: &str,
        seed: u64,
        took: Duration,
        state: &dyn std::fmt::Debug,
        totals: &mut [usize; 3],
    ) {
        eprintln!(
            "{what} seed {seed}: {state:?} in {took:?}; {} dropped, {} delayed past the wait, \
             {} duplicated",
            self.dropped, self.delayed, self.duplicated
        );
        totals[0] += self.dropped;
        totals[1] += self.delayed;
        totals[2] += self.duplicated;
    }

    fn release(&mut self, peer: &mut Peer) {
        let now = Instant::now();
        let (due, waiting): (Vec<_>, Vec<_>) = self.late.drain(..).partition(|(at, _)| *at <= now);
        self.late = waiting;
        for (_, reply) in due {
            peer.send(&reply);
        }
    }
}

/// Every kind of fault happened somewhere across the seeds, or the test proved less than it
/// claims to.
fn assert_every_fault_injected(what: &str, totals: [usize; 3]) {
    assert!(
        totals.iter().all(|count| *count > 0),
        "{what}: dropped, delayed and duplicated {totals:?}; every kind must happen"
    );
}

/// A mission downloaded over a link that drops, duplicates and delays replies past the wait
/// converges to the vehicle's mission, or fails cleanly - never hangs - and no item is asked for
/// more often than the C#'s six times (`getWP`, :3459).
#[test]
fn a_download_over_a_bad_link_converges_or_fails_cleanly() {
    let t = ProtocolTimeouts::default().faster(40);
    let items = mission(8);
    let mut totals = [0; 3];
    for seed in 1..=4u64 {
        let (link, mut peer) = link(t);
        let mut bad = BadLink::new(seed, t.mission_item_request.timeout * 3 / 2);

        link.download_mission(VEHICLE);
        let started = Instant::now();
        while !transfer_finished(&link) {
            assert!(started.elapsed() < HUNG, "seed {seed}: the download hung");
            bad.release(&mut peer);
            if let Some(message) = peer.next(Duration::from_millis(1))
                && let Some(reply) = answer(&items, &message)
            {
                bad.deliver(&mut peer, reply);
            }
        }

        bad.report(
            "download",
            seed,
            started.elapsed(),
            &transfer_state(&link),
            &mut totals,
        );
        match transfer_state(&link).unwrap() {
            TransferState::Complete => assert_eq!(
                link.mission_transfer(VEHICLE).unwrap().items(),
                items.as_slice(),
                "seed {seed}"
            ),
            TransferState::Failed(TransferFailure::TimedOut(_)) => {}
            other => panic!("seed {seed}: {other:?}"),
        }
        for seq in 0..8 {
            let asked = peer.count(|m| item_requested(m) == Some(seq));
            assert!(
                asked <= 6,
                "seed {seed}: item {seq} asked for {asked} times"
            );
        }
    }
    assert_every_fault_injected("download", totals);
}

/// A mission uploaded over a link that drops, duplicates and delays the vehicle's requests. The
/// vehicle is ArduPilot-shaped: it asks for the item it needs next, and asks again when an item
/// it did not expect arrives. The upload ends complete with the vehicle holding every item in
/// order, or in a clean timeout.
#[test]
fn an_upload_over_a_bad_link_converges_or_fails_cleanly() {
    let t = ProtocolTimeouts::default().faster(20);
    const COUNT: u16 = 8;
    let mut totals = [0; 3];
    for seed in 1..=4u64 {
        let (link, mut peer) = link(t);
        let mut bad = BadLink::new(seed, t.mission_item_send.timeout * 3 / 2);
        let mut held: Vec<u16> = Vec::new();

        link.upload_mission(VEHICLE, mission(COUNT));
        let started = Instant::now();
        while !transfer_finished(&link) {
            assert!(started.elapsed() < HUNG, "seed {seed}: the upload hung");
            bad.release(&mut peer);
            let Some(message) = peer.next(Duration::from_millis(1)) else {
                continue;
            };
            let next = u16::try_from(held.len()).unwrap();
            let reply = match message {
                MavMessage::MissionCount(_) if held.is_empty() => mission_request(0),
                MavMessage::MissionItemInt(item) if item.seq == next => {
                    held.push(item.seq);
                    if next + 1 < COUNT {
                        mission_request(next + 1)
                    } else {
                        mission_ack(MISSION_ACCEPTED)
                    }
                }
                MavMessage::MissionItemInt(_) if next < COUNT => mission_request(next),
                MavMessage::MissionItemInt(_) => mission_ack(MISSION_ACCEPTED),
                _ => continue,
            };
            bad.deliver(&mut peer, reply);
        }

        bad.report(
            "upload",
            seed,
            started.elapsed(),
            &transfer_state(&link),
            &mut totals,
        );
        match transfer_state(&link).unwrap() {
            TransferState::Complete => {
                assert_eq!(held, (0..COUNT).collect::<Vec<_>>(), "seed {seed}");
            }
            TransferState::Failed(TransferFailure::TimedOut(_)) => {}
            other => panic!("seed {seed}: {other:?}"),
        }
    }
    assert_every_fault_injected("upload", totals);
}

/// A parameter download over a link that drops, duplicates and delays - the stream and the
/// answers to reads alike, so indices arrive out of order and some never on the first pass.
/// It completes with every value right: the C#'s recovery never gives up (:2226), so over a link
/// that delivers anything at all, it finishes.
#[test]
fn a_parameter_download_over_a_bad_link_completes() {
    let t = ProtocolTimeouts::default().faster(40);
    const COUNT: u16 = 40;
    let mut totals = [0; 3];
    for seed in 1..=3u64 {
        let (link, mut peer) = link(t);
        let mut bad = BadLink::new(seed, t.param_list_round * 3);

        link.download_params(VEHICLE);
        let started = Instant::now();
        while download_state(&link) != Some(ParamDownloadState::Complete) {
            assert!(started.elapsed() < HUNG, "seed {seed}: the download hung");
            bad.release(&mut peer);
            let Some(message) = peer.next(Duration::from_millis(1)) else {
                continue;
            };
            if is_param_list(&message) {
                for index in 0..COUNT {
                    bad.deliver(&mut peer, numbered(index, COUNT));
                }
            } else if let Some(index) = param_read_index(&message) {
                bad.deliver(&mut peer, numbered(index, COUNT));
            }
        }

        bad.report(
            "parameters",
            seed,
            started.elapsed(),
            &download_state(&link),
            &mut totals,
        );
        let table = link.params(VEHICLE).unwrap();
        assert_eq!(table.len(), usize::from(COUNT), "seed {seed}");
        for index in 0..COUNT {
            let value = table.get(&format!("P{index:04}")).unwrap().as_f64();
            assert!(
                (value - f64::from(index)).abs() < f64::EPSILON,
                "seed {seed}: {index}"
            );
        }
    }
    assert_every_fault_injected("parameters", totals);
}

/// `doCommandIntAsync`: four `COMMAND_INT` two seconds apart (`retrys = 3`, :2884-2885), then
/// "Timeout on read - doCommand"; and its ack ends it, anything but ACCEPTED as a refusal.
#[test]
fn command_int_is_sent_four_times_then_times_out_and_its_ack_ends_it() {
    let t = ProtocolTimeouts::default().faster(40);
    let (link, mut peer) = link(t);
    let is_int = |m: &MavMessage| matches!(m, MavMessage::CommandInt(_));
    let set_home = |link: &Link| {
        link.command_int(
            VEHICLE,
            commands::CMD_DO_SET_HOME,
            commands::FRAME_GLOBAL,
            [0.0; 4],
            -353_632_620,
            1_491_652_370,
            584.0,
            true,
        )
    };

    let started = Instant::now();
    let id = set_home(&link);
    drive(&mut peer, silent, || outcome(&link, id).is_some());
    let took = started.elapsed();
    assert_eq!(outcome(&link, id), Some(RequestOutcome::TimedOut));
    assert_eq!(peer.count(is_int), usize::from(t.command.sends()));
    assert_eq!(peer.count(is_int), 4);
    assert!(took >= t.command.timeout * 4, "took {took:?}");

    let before = peer.count(is_int);
    let answered = set_home(&link);
    let mut seen = 0;
    drive(
        &mut peer,
        |peer, message| {
            if is_int(&message) {
                seen += 1;
                if seen == 3 {
                    peer.send(&MavMessage::CommandAck(CommandAck {
                        command: commands::CMD_DO_SET_HOME,
                        result: MAV_RESULT_ACCEPTED,
                        progress: 0,
                        result_param2: 0,
                        target_system: GCS.sysid,
                        target_component: GCS.compid,
                    }));
                }
            }
        },
        || outcome(&link, answered).is_some(),
    );
    assert_eq!(
        outcome(&link, answered),
        Some(RequestOutcome::Accepted { value: None })
    );
    assert_eq!(peer.count(is_int) - before, 3);

    let refused = set_home(&link);
    drive(
        &mut peer,
        |peer, message| {
            if is_int(&message) {
                peer.send(&MavMessage::CommandAck(CommandAck {
                    command: commands::CMD_DO_SET_HOME,
                    result: MAV_RESULT_IN_PROGRESS,
                    progress: 0,
                    result_param2: 0,
                    target_system: GCS.sysid,
                    target_component: GCS.compid,
                }));
            }
        },
        || outcome(&link, refused).is_some(),
    );
    assert_eq!(
        outcome(&link, refused),
        Some(RequestOutcome::Rejected(MAV_RESULT_IN_PROGRESS))
    );
}

/// `setWPAsync` for one item: eleven `MISSION_ITEM` 450 ms apart (`retrys = 10`, :4250-4254),
/// then "Timeout on read - setWP"; a `MISSION_ACK` to this ground station ends it with its
/// result, one to another is ignored (:4084-4087), and the vehicle asking for the next item is
/// an acceptance (:4115-4143).
#[test]
fn set_wp_is_sent_eleven_times_then_times_out_and_an_ack_or_the_next_request_ends_it() {
    let t = ProtocolTimeouts::default().faster(40);
    let (link, mut peer) = link(t);
    let is_item = |m: &MavMessage| matches!(m, MavMessage::MissionItem(_));
    let change_alt = |link: &Link| {
        link.set_wp(VEHICLE, commands::change_alt(VEHICLE, 25.0))
            .expect("an item")
    };

    let started = Instant::now();
    let id = change_alt(&link);
    drive(&mut peer, silent, || outcome(&link, id).is_some());
    let took = started.elapsed();
    assert_eq!(outcome(&link, id), Some(RequestOutcome::TimedOut));
    assert_eq!(
        peer.count(is_item),
        usize::from(t.mission_item_send.sends())
    );
    assert_eq!(peer.count(is_item), 11);
    assert!(took >= t.mission_item_send.timeout * 11, "took {took:?}");

    // An ack to somebody else changes nothing; ours ends it.
    let before = peer.count(is_item);
    let acked = change_alt(&link);
    let mut seen = 0;
    drive(
        &mut peer,
        |peer, message| {
            if is_item(&message) {
                seen += 1;
                let (target_system, target_component) = if seen == 1 {
                    (7, 7)
                } else {
                    (GCS.sysid, GCS.compid)
                };
                peer.send(&MavMessage::MissionAck(MissionAck {
                    target_system,
                    target_component,
                    r#type: MISSION_ACCEPTED,
                    mission_type: MISSION_TYPE_MISSION,
                }));
            }
        },
        || outcome(&link, acked).is_some(),
    );
    assert_eq!(
        outcome(&link, acked),
        Some(RequestOutcome::Accepted { value: None })
    );
    assert_eq!(peer.count(is_item) - before, 2);

    let before = peer.count(is_item);
    let requested = change_alt(&link);
    drive(
        &mut peer,
        |peer, message| {
            if is_item(&message) {
                peer.send(&MavMessage::MissionRequest(MissionRequest {
                    seq: 1,
                    target_system: GCS.sysid,
                    target_component: GCS.compid,
                    mission_type: MISSION_TYPE_MISSION,
                }));
            }
        },
        || outcome(&link, requested).is_some(),
    );
    assert_eq!(
        outcome(&link, requested),
        Some(RequestOutcome::Accepted { value: None })
    );
    assert_eq!(peer.count(is_item) - before, 1);

    let refused = change_alt(&link);
    drive(
        &mut peer,
        |peer, message| {
            if is_item(&message) {
                peer.send(&MavMessage::MissionAck(MissionAck {
                    target_system: GCS.sysid,
                    target_component: GCS.compid,
                    r#type: MISSION_NO_SPACE,
                    mission_type: MISSION_TYPE_MISSION,
                }));
            }
        },
        || outcome(&link, refused).is_some(),
    );
    assert_eq!(
        outcome(&link, refused),
        Some(RequestOutcome::Rejected(MISSION_NO_SPACE))
    );
}

/// `getHomePositionAsync`: `GET_HOME_POSITION` four times 700 ms apart (`retrys = 3`,
/// :3362-3366), each a fresh `doCommand` at confirmation zero, then "Timeout on read -
/// getHomePosition"; any `HOME_POSITION` from the vehicle ends it.
#[test]
fn get_home_position_asks_four_times_then_times_out_and_a_home_position_ends_it() {
    let t = ProtocolTimeouts::default().faster(40);
    let (link, mut peer) = link(t);
    let is_ask = |m: &MavMessage| matches!(m, MavMessage::CommandLong(long) if long.command == CMD_GET_HOME_POSITION);

    let started = Instant::now();
    let id = link.get_home_position(VEHICLE);
    drive(&mut peer, silent, || outcome(&link, id).is_some());
    let took = started.elapsed();
    assert_eq!(outcome(&link, id), Some(RequestOutcome::TimedOut));
    assert_eq!(peer.count(is_ask), usize::from(t.home_position.sends()));
    assert_eq!(peer.count(is_ask), 4);
    assert!(took >= t.home_position.timeout * 4, "took {took:?}");
    assert_eq!(
        peer.sent(|m| match m {
            MavMessage::CommandLong(long) if long.command == CMD_GET_HOME_POSITION =>
                Some(long.confirmation),
            _ => None,
        }),
        vec![0, 0, 0, 0]
    );

    let before = peer.count(is_ask);
    let answered = link.get_home_position(VEHICLE);
    let mut seen = 0;
    drive(
        &mut peer,
        |peer, message| {
            if is_ask(&message) {
                seen += 1;
                if seen == 2 {
                    peer.send(&MavMessage::HomePosition(HomePosition {
                        latitude: -353_632_620,
                        longitude: 1_491_652_370,
                        altitude: 584_090,
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        q: [1.0, 0.0, 0.0, 0.0],
                        approach_x: 0.0,
                        approach_y: 0.0,
                        approach_z: 0.0,
                        time_usec: 0,
                    }));
                }
            }
        },
        || outcome(&link, answered).is_some(),
    );
    assert_eq!(
        outcome(&link, answered),
        Some(RequestOutcome::Accepted { value: None })
    );
    assert_eq!(peer.count(is_ask) - before, 2);
}

// ================================================================================================
// A script's single-item reads and writes: getWPAsync, setWPTotalAsync
// ================================================================================================

/// An item from the vehicle, addressed to `to`: `MISSION_ITEM`, the float form.
fn item_float(seq: u16, command: u16, x: f32, y: f32, to: VehicleId) -> MavMessage {
    MavMessage::MissionItem(MissionItemFloat {
        param1: 1.0,
        param2: 2.0,
        param3: 3.0,
        param4: 4.0,
        x,
        y,
        z: 584.0,
        seq,
        command,
        target_system: to.sysid,
        target_component: to.compid,
        frame: 3,
        current: 0,
        autocontinue: 1,
        mission_type: MISSION_TYPE_MISSION,
    })
}

/// An item from the vehicle, addressed to the link: `MISSION_ITEM_INT`.
fn item_int(seq: u16, command: u16, x: i32, y: i32, mission_type: u8) -> MavMessage {
    MavMessage::MissionItemInt(MissionItemInt {
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x,
        y,
        z: 20.0,
        seq,
        command,
        target_system: GCS.sysid,
        target_component: GCS.compid,
        frame: 0,
        current: 0,
        autocontinue: 1,
        mission_type,
    })
}

/// A float `MISSION_REQUEST` the link sent: its sequence number and list.
fn request_float_sent(message: &MavMessage) -> Option<(u16, u8)> {
    match message {
        MavMessage::MissionRequest(request) => Some((request.seq, request.mission_type)),
        _ => None,
    }
}

/// `getWPAsync`: `MISSION_REQUEST` for the item - the vehicle has not said it speaks
/// `MISSION_INT` - six times 2.5 s apart (`retrys = 5`, :3459-3463), then "Timeout on read -
/// getWP". Answered: an item of another sequence number asks again at once, without spending a
/// retry (:3494-3498); one addressed to another ground station is read past, as a `MISSION_ACK`
/// is (:3487-3491); the one asked for ends it with its fields, the float position as it came
/// (:3500-3511). No mission transfer is started for it, nor a `MISSION_REQUEST_LIST` sent.
#[test]
fn get_wp_asks_six_times_then_times_out_and_the_item_asked_for_ends_it() {
    let t = ProtocolTimeouts::default().faster(50);
    let (link, mut peer) = link(t);
    let is_request = |m: &MavMessage| matches!(m, MavMessage::MissionRequest(_));

    let started = Instant::now();
    let id = link.get_wp(VEHICLE, 0, MISSION_TYPE_MISSION);
    drive(&mut peer, silent, || outcome(&link, id).is_some());
    let took = started.elapsed();
    assert_eq!(outcome(&link, id), Some(RequestOutcome::TimedOut));
    assert_eq!(
        peer.count(is_request),
        usize::from(t.mission_item_request.sends())
    );
    assert_eq!(peer.count(is_request), 6);
    assert!(took >= t.mission_item_request.timeout * 6, "took {took:?}");
    assert_eq!(
        peer.sent(request_float_sent),
        vec![(0, MISSION_TYPE_MISSION); 6]
    );

    let before = peer.count(is_request);
    let answered = link.get_wp(VEHICLE, 2, MISSION_TYPE_MISSION);
    let mut asked = 0;
    drive(
        &mut peer,
        |peer, message| {
            if is_request(&message) {
                asked += 1;
                if asked == 1 {
                    // Another item: asked for again at once.
                    peer.send(&item_float(5, 16, -35.0, 149.0, GCS));
                } else {
                    peer.send_all(&[
                        item_float(2, 16, -35.5, 149.5, VehicleId::new(7, 7)),
                        mission_ack(MISSION_ACCEPTED),
                        item_float(2, 16, -35.363_262, 149.165_24, GCS),
                    ]);
                }
            }
        },
        || outcome(&link, answered).is_some(),
    );
    assert_eq!(
        outcome(&link, answered),
        Some(RequestOutcome::Accepted { value: None })
    );
    assert_eq!(peer.count(is_request) - before, 2);
    let wp = link
        .request(answered)
        .and_then(|request| request.wp())
        .expect("the item");
    assert_eq!(
        (wp.id, wp.params, wp.alt, wp.frame),
        (16, [1.0, 2.0, 3.0, 4.0], 584.0, 3)
    );
    assert_eq!(
        (wp.lat, wp.lng),
        (f64::from(-35.363_262_f32), f64::from(149.165_24_f32))
    );
    assert!(link.mission_transfer(VEHICLE).is_none());
    assert_eq!(
        peer.count(|m| matches!(m, MavMessage::MissionRequestList(_))),
        0
    );
}

/// To a vehicle whose capabilities have `MISSION_INT`, `MISSION_REQUEST_INT`, of the list asked
/// for (:3419-3436); the `MISSION_ITEM_INT`'s position over 1e7 for a command
/// `Locationwp.isLocationCommand` names - `DO_SET_ROI`, outside the navigation block - and as it
/// came for one it does not - `DO_SET_SERVO` (:3539-3547).
#[test]
fn get_wp_asks_a_mission_int_vehicle_with_request_int_and_scales_only_location_commands() {
    const FENCE: u8 = 1;
    const DO_SET_ROI: u16 = 201;
    const DO_SET_SERVO: u16 = 183;
    let t = ProtocolTimeouts::default().faster(50);
    let (link, mut peer) = link(t);
    peer.send(&MavMessage::AutopilotVersion(AutopilotVersion {
        capabilities: u64::from(mp_link::requests::CAPABILITY_MISSION_INT),
        uid: 0,
        flight_sw_version: 0x0405_0700,
        middleware_sw_version: 0,
        os_sw_version: 0,
        board_version: 0,
        vendor_id: 0,
        product_id: 0,
        flight_custom_version: [0; 8],
        middleware_custom_version: [0; 8],
        os_custom_version: [0; 8],
        uid2: [0; 18],
    }));
    wait_for("the capabilities", || {
        link.vehicle(VEHICLE).is_some_and(|handle| {
            handle.load().autopilot_info.capabilities & mp_link::requests::CAPABILITY_MISSION_INT
                != 0
        })
    });
    for (command, x, want) in [
        (DO_SET_ROI, -353_632_620, -35.363_262),
        (DO_SET_SERVO, 5, 5.0),
    ] {
        let id = link.get_wp(VEHICLE, 1, FENCE);
        drive(
            &mut peer,
            |peer, message| {
                if let MavMessage::MissionRequestInt(request) = message {
                    assert_eq!((request.seq, request.mission_type), (1, FENCE));
                    assert_eq!(
                        (request.target_system, request.target_component),
                        (VEHICLE.sysid, VEHICLE.compid)
                    );
                    peer.send(&item_int(1, command, x, 1_491_652_370, FENCE));
                }
            },
            || outcome(&link, id).is_some(),
        );
        assert_eq!(
            outcome(&link, id),
            Some(RequestOutcome::Accepted { value: None })
        );
        let wp = link
            .request(id)
            .and_then(|request| request.wp())
            .expect("the item");
        assert_eq!(wp.id, command);
        assert!((wp.lat - want).abs() < 1e-9, "{command}: {}", wp.lat);
    }
    assert_eq!(
        peer.count(|m| matches!(m, MavMessage::MissionRequest(_))),
        0
    );
}

/// `setWPTotalAsync`: `MISSION_COUNT` four times 700 ms apart (`retrys = 3`, :3779-3795), then
/// "Timeout on read - setWPTotal". Answered: a request for item 2, or one addressed to another
/// ground station, is read past (:3808-3812); a request for item 0 ends it, and `WP_TOTAL`,
/// `CMD_TOTAL` and `MIS_TOTAL` are held as the count less one (:3814-3828); an ack ends it too,
/// and changes nothing (:3862-3876).
#[test]
fn set_wp_total_counts_four_times_then_times_out_and_the_first_request_ends_it() {
    const TOTALS: [&str; 3] = ["WP_TOTAL", "CMD_TOTAL", "MIS_TOTAL"];
    // Halved, not divided by forty: at a fortieth the count was resent every 17.5 ms, and on the
    // hosted macOS runner (CI run 37125541254, 2026-10-03) a resend came before the request that
    // ends the exchange had been read, so the answering vehicle ran twice. Halved, 350 ms apart.
    let t = ProtocolTimeouts::default().faster(2);
    let (link, mut peer) = link(t);
    peer.send_all(&[
        param_value("WP_TOTAL", 3.0, PARAM_TYPE_INT32, 0, 3),
        param_value("CMD_TOTAL", 3.0, PARAM_TYPE_INT32, 1, 3),
        param_value("MIS_TOTAL", 3.0, PARAM_TYPE_INT32, 2, 3),
    ]);
    wait_for("the totals in the table", || {
        link.params(VEHICLE)
            .is_some_and(|table| TOTALS.iter().all(|name| table.get(name).is_some()))
    });
    // The three totals, as the table holds them: each the same, or `None`.
    let wp_total = |link: &Link| {
        let held: Vec<Option<f64>> = TOTALS
            .iter()
            .map(|name| {
                link.params(VEHICLE)
                    .and_then(|table| table.get(name))
                    .map(ParamValueHeld::as_f64)
            })
            .collect();
        held.iter()
            .all(|value| *value == held[0])
            .then_some(held[0])
            .flatten()
    };
    let counted = |m: &MavMessage| match m {
        MavMessage::MissionCount(count) => Some((count.count, count.mission_type)),
        _ => None,
    };

    let started = Instant::now();
    let id = link.set_wp_total(VEHICLE, 5, MISSION_TYPE_MISSION);
    drive(&mut peer, silent, || outcome(&link, id).is_some());
    let took = started.elapsed();
    assert_eq!(outcome(&link, id), Some(RequestOutcome::TimedOut));
    assert_eq!(peer.sent(counted), vec![(5, MISSION_TYPE_MISSION); 4]);
    assert_eq!(usize::from(t.mission_count.sends()), 4);
    assert!(took >= t.mission_count.timeout * 4, "took {took:?}");
    assert_eq!(wp_total(&link), Some(3.0));

    let before = peer.sent(counted).len();
    let answered = link.set_wp_total(VEHICLE, 5, MISSION_TYPE_MISSION);
    // The vehicle answers the first count; a count the link sends after it has been answered is
    // the claim below's failure, counted there, not a second run of the vehicle's answer.
    let mut answering = true;
    drive(
        &mut peer,
        |peer, message| {
            if counted(&message).is_some() && std::mem::take(&mut answering) {
                peer.send_all(&[
                    mission_request_float(2),
                    MavMessage::MissionRequest(MissionRequest {
                        seq: 0,
                        target_system: 7,
                        target_component: 7,
                        mission_type: MISSION_TYPE_MISSION,
                    }),
                ]);
                std::thread::sleep(Duration::from_millis(5));
                assert_eq!(outcome(&link, answered), None);
                peer.send(&mission_request_float(0));
            }
        },
        || outcome(&link, answered).is_some(),
    );
    assert_eq!(
        outcome(&link, answered),
        Some(RequestOutcome::Accepted { value: None })
    );
    assert_eq!(peer.sent(counted).len() - before, 1);
    wait_for("the totals held as the count less one", || {
        wp_total(&link) == Some(4.0)
    });

    let acked = link.set_wp_total(VEHICLE, 9, MISSION_TYPE_MISSION);
    drive(
        &mut peer,
        |peer, message| {
            if counted(&message).is_some() {
                peer.send(&mission_ack(MISSION_NO_SPACE));
            }
        },
        || outcome(&link, acked).is_some(),
    );
    assert_eq!(
        outcome(&link, acked),
        Some(RequestOutcome::Accepted { value: None })
    );
    assert_eq!(wp_total(&link), Some(4.0));
}

/// The vehicle's first request is `setWPTotal`'s alone, as the C#'s loop reads it off the port:
/// the `setWP(0)` that follows sends item 0 once, and the vehicle's request for item 1 accepts
/// it. There is no second item 0, which ArduPilot answers with `MAV_MISSION_INVALID_SEQUENCE`.
/// For the geofence, the fence points the link holds are emptied when that request comes
/// (:3830-3831).
#[test]
fn set_wp_total_takes_the_first_request_so_set_wp_sends_item_zero_once() {
    const FENCE: u8 = 1;
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = link(t);
    peer.send(&MavMessage::FencePoint(FencePoint {
        lat: -35.0,
        lng: 149.0,
        target_system: GCS.sysid,
        target_component: GCS.compid,
        idx: 0,
        count: 4,
    }));
    wait_for("a fence point held", || {
        !link.fence_points(VEHICLE).is_empty()
    });
    let answer_count = |peer: &mut Peer, message: MavMessage| {
        if let MavMessage::MissionCount(count) = message {
            peer.send(&MavMessage::MissionRequest(MissionRequest {
                seq: 0,
                target_system: GCS.sysid,
                target_component: GCS.compid,
                mission_type: count.mission_type,
            }));
        }
    };

    // The mission's count leaves the fence alone (:3814-3828).
    let mission_total = link.set_wp_total(VEHICLE, 2, MISSION_TYPE_MISSION);
    drive(&mut peer, answer_count, || {
        outcome(&link, mission_total).is_some()
    });
    assert_eq!(
        outcome(&link, mission_total),
        Some(RequestOutcome::Accepted { value: None })
    );
    std::thread::sleep(Duration::from_millis(20));
    assert_eq!(link.fence_points(VEHICLE).len(), 1);

    let total = link.set_wp_total(VEHICLE, 2, FENCE);
    drive(&mut peer, answer_count, || outcome(&link, total).is_some());
    assert_eq!(
        outcome(&link, total),
        Some(RequestOutcome::Accepted { value: None })
    );
    wait_for("the fence points emptied", || {
        link.fence_points(VEHICLE).is_empty()
    });

    let is_item = |m: &MavMessage| matches!(m, MavMessage::MissionItem(_));
    let MavMessage::MissionItem(change_alt) = commands::change_alt(VEHICLE, 25.0) else {
        unreachable!("change_alt is a MISSION_ITEM")
    };
    // A return point and an inclusion vertex, as a script's `setWP(loc, i, frame, 0, 1, false,
    // FENCE)` sends them.
    let fence_item = |seq: u16, command: u16| {
        MavMessage::MissionItem(MissionItemFloat {
            seq,
            command,
            current: 0,
            param1: 4.0,
            x: -35.36,
            y: 149.16,
            mission_type: FENCE,
            ..change_alt
        })
    };
    let set = link.set_wp(VEHICLE, fence_item(0, 5000)).expect("an item");
    drive(
        &mut peer,
        |peer, message| {
            if is_item(&message) {
                peer.send(&MavMessage::MissionRequest(MissionRequest {
                    seq: 1,
                    target_system: GCS.sysid,
                    target_component: GCS.compid,
                    mission_type: FENCE,
                }));
            }
        },
        || outcome(&link, set).is_some(),
    );
    assert_eq!(
        outcome(&link, set),
        Some(RequestOutcome::Accepted { value: None })
    );
    assert_eq!(peer.count(is_item), 1);

    // Item 1, answered with an ack - a refusal, which files it all the same (:4108-4127).
    let acked = link.set_wp(VEHICLE, fence_item(1, 5001)).expect("an item");
    drive(
        &mut peer,
        |peer, message| {
            if is_item(&message) {
                peer.send(&MavMessage::MissionAck(MissionAck {
                    target_system: GCS.sysid,
                    target_component: GCS.compid,
                    r#type: MISSION_ERROR,
                    mission_type: FENCE,
                }));
            }
        },
        || outcome(&link, acked).is_some(),
    );
    assert_eq!(
        outcome(&link, acked),
        Some(RequestOutcome::Rejected(MISSION_ERROR))
    );
    // `(Locationwp) req` of the float item, back out as `(int)(lat * 1e7)`.
    #[allow(clippy::cast_possible_truncation)]
    let (x, y) = (
        (f64::from(-35.36_f32) * 1e7) as i32,
        (f64::from(149.16_f32) * 1e7) as i32,
    );
    wait_for("the fence refilled", || {
        link.fence_points(VEHICLE).len() == 2
    });
    assert_eq!(
        link.fence_points(VEHICLE),
        [
            FenceItem {
                command: 5000,
                param1: 4.0,
                x,
                y
            },
            FenceItem {
                command: 5001,
                param1: 4.0,
                x,
                y
            },
        ]
    );
}

/// `MAVState.wps` and `rallypoints`, as a script's `setWPTotal` and `setWP`s fill them: the first
/// request empties the list named, a `MISSION_REQUEST` for the item after or an ack files the
/// item sent under its own list, and the float path's `MISSION_REQUEST_INT` branch files `wps`
/// whatever the list (:3811-3828, 4113-4127, 4146-4160, 4196-4206). A `MISSION_ITEM_INT` passing
/// on the stream is held too (:5671-5698), and Clear Rally Points empties the rally list
/// (FlightPlanner.cs:2109).
#[test]
fn set_wp_fills_the_mission_and_rally_lists_as_the_csharp_does() {
    const RALLY: u8 = 2;
    const RALLY_POINT: u16 = 5100;
    const WAYPOINT: u16 = 16;
    let t = ProtocolTimeouts::default().faster(20);
    let (link, mut peer) = link(t);
    // A mission item passing on the stream - another ground station's download - is held.
    peer.send(&MavMessage::MissionItemInt(MissionItemInt {
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x: -353_632_621,
        y: 1_491_652_374,
        z: 10.0,
        seq: 3,
        command: WAYPOINT,
        target_system: GCS.sysid,
        target_component: GCS.compid,
        frame: 3,
        current: 0,
        autocontinue: 1,
        mission_type: MISSION_TYPE_MISSION,
    }));
    wait_for("a waypoint held", || link.wps(VEHICLE).len() == 1);
    assert_eq!(link.wps(VEHICLE)[0].x, -35.363_262_1);

    let answer_count = |peer: &mut Peer, message: MavMessage| {
        if let MavMessage::MissionCount(count) = message {
            peer.send(&MavMessage::MissionRequest(MissionRequest {
                seq: 0,
                target_system: GCS.sysid,
                target_component: GCS.compid,
                mission_type: count.mission_type,
            }));
        }
    };
    // setWPTotal for the mission empties wps (:3827).
    let total = link.set_wp_total(VEHICLE, 3, MISSION_TYPE_MISSION);
    drive(&mut peer, answer_count, || outcome(&link, total).is_some());
    wait_for("wps emptied", || link.wps(VEHICLE).is_empty());

    let MavMessage::MissionItem(change_alt) = commands::change_alt(VEHICLE, 25.0) else {
        unreachable!("change_alt is a MISSION_ITEM")
    };
    let float_item = |seq: u16, command: u16, current: u8, mission_type: u8| {
        MavMessage::MissionItem(MissionItemFloat {
            seq,
            command,
            current,
            param1: 0.0,
            x: -35.36,
            y: 149.16,
            z: 50.0,
            mission_type,
            ..change_alt
        })
    };
    let request = |seq: u16, mission_type: u8| {
        MavMessage::MissionRequest(MissionRequest {
            seq,
            target_system: GCS.sysid,
            target_component: GCS.compid,
            mission_type,
        })
    };
    let request_int = |seq: u16, mission_type: u8| {
        MavMessage::MissionRequestInt(MissionRequestInt {
            seq,
            target_system: GCS.sysid,
            target_component: GCS.compid,
            mission_type,
        })
    };
    let ack = |mission_type: u8| {
        MavMessage::MissionAck(MissionAck {
            target_system: GCS.sysid,
            target_component: GCS.compid,
            r#type: MISSION_ACCEPTED,
            mission_type,
        })
    };
    /// One `setWP`, the vehicle answering its item with `answer`.
    fn set(link: &Link, peer: &mut Peer, item: MavMessage, answer: MavMessage) {
        let id = link.set_wp(VEHICLE, item).expect("an item");
        drive(
            peer,
            |peer, message| {
                if matches!(message, MavMessage::MissionItem(_)) {
                    peer.send(&answer);
                }
            },
            || outcome(link, id).is_some(),
        );
        assert_eq!(
            outcome(link, id),
            Some(RequestOutcome::Accepted { value: None })
        );
    }

    // Item 0, answered by a request for item 1: filed in wps (:4146-4160).
    set(
        &link,
        &mut peer,
        float_item(0, WAYPOINT, 0, MISSION_TYPE_MISSION),
        request(1, MISSION_TYPE_MISSION),
    );
    wait_for("item 0 filed", || link.wps(VEHICLE).len() == 1);
    // Item 1, answered by a MISSION_REQUEST_INT: the float path files wps from it too (:4206).
    set(
        &link,
        &mut peer,
        float_item(1, WAYPOINT, 0, MISSION_TYPE_MISSION),
        request_int(2, MISSION_TYPE_MISSION),
    );
    wait_for("item 1 filed", || link.wps(VEHICLE).len() == 2);
    // A guided target (current 2), acknowledged: GuidedMode's, filed in no list (:4113-4116).
    set(
        &link,
        &mut peer,
        float_item(2, WAYPOINT, 2, MISSION_TYPE_MISSION),
        ack(MISSION_TYPE_MISSION),
    );
    std::thread::sleep(Duration::from_millis(20));
    assert_eq!(link.wps(VEHICLE).len(), 2);
    // A rally point, acknowledged: filed in rallypoints (:4124-4126).
    set(
        &link,
        &mut peer,
        float_item(0, RALLY_POINT, 0, RALLY),
        ack(RALLY),
    );
    wait_for("the rally point filed", || {
        link.rally_points(VEHICLE).len() == 1
    });
    assert_eq!(link.wps(VEHICLE).len(), 2);
    // A rally point answered by a MISSION_REQUEST_INT lands in wps: that branch names no list.
    set(
        &link,
        &mut peer,
        float_item(1, RALLY_POINT, 0, RALLY),
        request_int(2, RALLY),
    );
    wait_for("the rally item in wps", || {
        link.wps(VEHICLE)[1].command == RALLY_POINT
    });
    assert_eq!(link.rally_points(VEHICLE).len(), 1);
    // `(Locationwp) req` of the float item, back out over 1e7.
    #[allow(clippy::cast_possible_truncation)]
    let x = f64::from((f64::from(-35.36_f32) * 1e7) as i32) / 1e7;
    let filed = link.wps(VEHICLE)[0];
    assert_eq!(
        (filed.seq, filed.command, filed.x, filed.z),
        (0, WAYPOINT, x, 50.0)
    );
    assert_eq!(link.rally_points(VEHICLE)[0].x, x);

    // setWPTotal for the rally points empties rallypoints and leaves wps (:3829).
    let total = link.set_wp_total(VEHICLE, 0, RALLY);
    drive(&mut peer, answer_count, || outcome(&link, total).is_some());
    wait_for("rallypoints emptied", || {
        link.rally_points(VEHICLE).is_empty()
    });
    assert_eq!(link.wps(VEHICLE).len(), 2);
    // Clear Rally Points' `MAV.rallypoints.Clear()`.
    set(
        &link,
        &mut peer,
        float_item(0, RALLY_POINT, 0, RALLY),
        ack(RALLY),
    );
    wait_for("the rally point filed again", || {
        link.rally_points(VEHICLE).len() == 1
    });
    link.clear_rally_points(VEHICLE);
    assert!(link.rally_points(VEHICLE).is_empty());
    assert_eq!(link.wps(VEHICLE).len(), 2);
}
