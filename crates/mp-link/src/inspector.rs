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

//! What the MAVLink Inspector reads: every packet the link passes, and the record it keeps of
//! them.
//!
//! * [`Packet`] and [`Link::on_packet`](crate::Link::on_packet): `MAVLinkInterface`'s
//!   `OnPacketReceived`, raised for each packet read once the C# has handled it
//!   (`ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5347-5349`), and `OnPacketSent`, raised for
//!   each packet written (`:1453-1458`, `:1503-1508`). One handler hears both, [`Packet::sent`]
//!   saying which; it runs on the link thread, as the C#'s runs on its reader's, and ends when
//!   its [`PacketSubscription`] is dropped - the C#'s `-=`.
//! * [`PacketInspector`]: `ExtLibs/ArduPilot/PacketInspector.cs`, the newest packet of each
//!   message from each system and component, with its rate and its bytes a second over the last
//!   three seconds.
//!
//! Where it is not the C#, each for a reason:
//!
//! * `DateTime.Now` is an [`Instant`] handed in, so a test says when each packet came;
//! * the dictionaries are ordered maps where the C#'s enumerate in insertion order: their one
//!   reader, the inspector's tree, sorts what it is given (`Controls/MAVLinkInspector.cs:170-171`);
//! * `NewSysidCompid`, `SeenSysid` and `SeenCompid` fill the inspector's two combo boxes, which
//!   its Designer makes invisible (`Controls/MAVLinkInspector.cs:36-43, 215-229`) - nothing a
//!   user sees, so not ported; `Clear()` and the `[sysid, compid]` indexer have no callers, and
//!   `SeenBps(sysid, compid)` only the DroneCAN inspector's (`Controls/DroneCANInspector.cs:81`),
//!   another window.

use mp_os::Lock as _;
use std::collections::{BTreeMap, VecDeque};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, Weak};
use web_time::{Duration, Instant};

use mp_mavlink::Frame;
use mp_mavlink_dialects::all::{DIALECT, MavMessage};
use mp_vehicle::DateTime;

use crate::Shared;

/// One packet, as `MAVLinkMessage` carries it to an `OnPacketReceived` or `OnPacketSent`
/// handler.
/// `// C#: ExtLibs/Mavlink/MAVLinkMessage.cs:25-172`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Packet {
    /// `sysid`, 32-bit (e6454ccdd).
    pub sysid: u32,
    /// `compid`.
    pub compid: u8,
    /// `msgid`.
    pub msgid: u32,
    /// `data`: the payload decoded.
    pub message: MavMessage,
    /// `Length`: the whole frame's bytes, header, checksum and signature included.
    pub length: usize,
    /// `rxtime`: `DateTime.UtcNow` when it was read or written, or its recorded time when a
    /// `.tlog` is played (`MAVLinkInterface.cs:4948, 6611`; `MAVLinkMessage.cs:179-181`).
    pub rxtime: DateTime,
    /// When it passed, on this machine's clock: what `PacketInspector.Add`'s `DateTime.Now`
    /// measures a rate against.
    pub at: Instant,
    /// Whether the link wrote it (`OnPacketSent`) rather than read it (`OnPacketReceived`).
    pub sent: bool,
}

impl Packet {
    /// A packet the link read.
    #[must_use]
    pub fn received(frame: &Frame<'_>, message: MavMessage, rxtime: DateTime, at: Instant) -> Self {
        Self {
            sysid: frame.sysid,
            compid: frame.compid,
            msgid: frame.msgid,
            message,
            length: frame.raw.len(),
            rxtime,
            at,
            sent: false,
        }
    }

    /// A frame the link wrote, read back as `new MAVLinkMessage(packet)` reads it, stamped
    /// `DateTime.UtcNow`; `None` for bytes that are not one whole frame of a known message.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1457; ExtLibs/Mavlink/MAVLinkMessage.cs:179-181`
    #[must_use]
    pub fn written(bytes: &[u8], rxtime: DateTime, at: Instant) -> Option<Self> {
        let (frame, _) = mp_mavlink::parse(bytes, &DIALECT).ok()?;
        let message = MavMessage::decode(frame.msgid, frame.payload)?;
        Some(Self {
            sent: true,
            ..Self::received(&frame, message, rxtime, at)
        })
    }
}

/// What a subscriber is handed each packet with.
pub type PacketHandler = Box<dyn FnMut(&Packet) + Send>;

/// The link's `OnPacketReceived` and `OnPacketSent` subscribers.
#[derive(Default)]
pub(crate) struct Subscribers {
    /// Each handler, with the token its subscription holds: gone, the handler is dropped.
    handlers: Mutex<Vec<(Weak<()>, PacketHandler)>>,
    /// How many are held, so a link nobody listens to takes no lock per packet.
    count: AtomicUsize,
}

impl std::fmt::Debug for Subscribers {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Subscribers")
            .field("count", &self.count.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Subscribers {
    /// Whether anybody is listening, or was until a subscription was dropped.
    pub(crate) fn any(&self) -> bool {
        self.count.load(Ordering::Relaxed) > 0
    }

    /// A handler added, and the token that keeps it.
    fn subscribe(&self, handler: PacketHandler) -> Arc<()> {
        let alive = Arc::new(());
        if let Ok(mut handlers) = self.handlers.os_lock() {
            handlers.push((Arc::downgrade(&alive), handler));
            self.count.store(handlers.len(), Ordering::Relaxed);
        }
        alive
    }

    /// Each live handler told of `packet`; the dropped ones forgotten.
    pub(crate) fn notify(&self, packet: &Packet) {
        if let Ok(mut handlers) = self.handlers.os_lock() {
            handlers.retain(|(alive, _)| alive.strong_count() > 0);
            for (_, handler) in handlers.iter_mut() {
                handler(packet);
            }
            self.count.store(handlers.len(), Ordering::Relaxed);
        }
    }

    /// A frame the link wrote, told as a sent packet.
    pub(crate) fn sent(&self, bytes: &[u8]) {
        if let Some(packet) = Packet::written(bytes, DateTime::now(), Instant::now()) {
            self.notify(&packet);
        }
    }
}

/// A handler's hold on a link's packets: dropping it unsubscribes.
#[derive(Debug)]
pub struct PacketSubscription {
    /// The token the link holds weakly.
    _alive: Arc<()>,
    /// The link it is on.
    link: Weak<Shared>,
}

impl PacketSubscription {
    /// `handler` subscribed to `shared`'s packets.
    pub(crate) fn new(shared: &Arc<Shared>, handler: PacketHandler) -> Self {
        Self {
            _alive: shared.packets.subscribe(handler),
            link: Arc::downgrade(shared),
        }
    }

    /// Whether it is on the link `shared` belongs to.
    pub(crate) fn is_on(&self, shared: &Arc<Shared>) -> bool {
        std::ptr::eq(self.link.as_ptr(), Arc::as_ptr(shared))
    }
}

/// `RateHistory`: how many arrivals of each message its rate is worked from.
/// `// C#: ExtLibs/ArduPilot/PacketInspector.cs:19`
pub const RATE_HISTORY: usize = 200;

/// `end.AddSeconds(-3)`: how far back a rate looks.
/// `// C#: ExtLibs/ArduPilot/PacketInspector.cs:63, 85`
pub const RATE_WINDOW: Duration = Duration::from_secs(3);

/// `irate`: when something arrived, and how much of it.
/// `// C#: ExtLibs/ArduPilot/PacketInspector.cs:25-35`
#[derive(Debug, Clone, Copy)]
struct Arrival {
    at: Instant,
    value: usize,
}

/// Arrivals by `GetID(sysid, compid)`, then by message id.
type Arrivals = BTreeMap<u64, BTreeMap<u32, VecDeque<Arrival>>>;

/// `PacketInspector<T>`: the newest packet of each message from each system and component, and
/// the arrivals its rates are worked from.
/// `// C#: ExtLibs/ArduPilot/PacketInspector.cs:11-221`
#[derive(Debug, Clone)]
pub struct PacketInspector<T> {
    /// `_history`.
    history: BTreeMap<u64, BTreeMap<u32, T>>,
    /// `_rate`: an arrival of 1 a packet.
    rate: Arrivals,
    /// `_bps`: an arrival of its size a packet.
    bps: Arrivals,
    /// `RateHistory`.
    pub rate_history: usize,
}

impl<T> Default for PacketInspector<T> {
    fn default() -> Self {
        Self {
            history: BTreeMap::new(),
            rate: BTreeMap::new(),
            bps: BTreeMap::new(),
            rate_history: RATE_HISTORY,
        }
    }
}

/// `GetID`: a system and component as one number.
/// `// C#: ExtLibs/ArduPilot/PacketInspector.cs:212-215`
fn id_of(sysid: u32, compid: u8) -> u64 {
    (u64::from(sysid) << 8) | u64::from(compid)
}

/// `SeenRate`'s and `SeenBps`'s sum: each arrival in the three seconds before `now`, over the
/// time since the first arrival held or since three seconds ago, whichever is later. Nothing
/// held is 0, as `First()`'s exception is caught.
/// `// C#: ExtLibs/ArduPilot/PacketInspector.cs:59-101`
fn per_second(arrivals: Option<&VecDeque<Arrival>>, now: Instant) -> f64 {
    let Some(first) = arrivals.and_then(VecDeque::front) else {
        return 0.0;
    };
    let start = now.checked_sub(RATE_WINDOW);
    let start_time = match start {
        Some(start) if first.at < start => start,
        _ => first.at,
    };
    let seconds = now.saturating_duration_since(start_time).as_secs_f64();
    arrivals
        .into_iter()
        .flatten()
        .filter(|arrival| start.is_none_or(|start| arrival.at > start) && arrival.at < now)
        // `a.value / (end - starttime).TotalSeconds`, each divided, then summed.
        .map(|arrival| arrival.value as f64 / seconds)
        .sum()
}

impl<T: Clone> PacketInspector<T> {
    /// `Add`: `message` kept as the newest of its kind from `sysid` and `compid`, and an arrival
    /// of it and of its `size` bytes noted at `now`, the oldest beyond [`RATE_HISTORY`] dropped.
    /// A system and component not seen before is `Clear`ed in first, which makes its tables.
    /// `// C#: ExtLibs/ArduPilot/PacketInspector.cs:125-153, 186-197`
    pub fn add(
        &mut self,
        sysid: u32,
        compid: u8,
        msgid: u32,
        message: T,
        size: usize,
        now: Instant,
    ) {
        let id = id_of(sysid, compid);
        self.history.entry(id).or_default().insert(msgid, message);
        for (table, value) in [(&mut self.bps, size), (&mut self.rate, 1)] {
            let list = table.entry(id).or_default().entry(msgid).or_default();
            list.push_back(Arrival { at: now, value });
            while list.len() > self.rate_history {
                list.pop_front();
            }
        }
    }

    /// `GetPacketMessages`: the newest packet of every message from every system and component.
    /// `// C#: ExtLibs/ArduPilot/PacketInspector.cs:163-172`
    #[must_use]
    pub fn packet_messages(&self) -> Vec<T> {
        self.history
            .values()
            .flat_map(|messages| messages.values().cloned())
            .collect()
    }

    /// `SeenRate`: packets a second of one message from one system and component.
    /// `// C#: ExtLibs/ArduPilot/PacketInspector.cs:59-79`
    #[must_use]
    pub fn seen_rate(&self, sysid: u32, compid: u8, msgid: u32, now: Instant) -> f64 {
        let id = id_of(sysid, compid);
        per_second(self.rate.get(&id).and_then(|m| m.get(&msgid)), now)
    }

    /// `SeenBps(sysid, compid, msgid)`: its bytes a second.
    /// `// C#: ExtLibs/ArduPilot/PacketInspector.cs:81-101`
    #[must_use]
    pub fn seen_bps(&self, sysid: u32, compid: u8, msgid: u32, now: Instant) -> f64 {
        let id = id_of(sysid, compid);
        per_second(self.bps.get(&id).and_then(|m| m.get(&msgid)), now)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mp_mavlink::encode_v2;
    use mp_mavlink_dialects::all::{Attitude, Heartbeat};

    fn at(start: Instant, millis: u64) -> Instant {
        start + Duration::from_millis(millis)
    }

    /// Ten packets a second for five seconds: the rate is ten a second - the last three seconds'
    /// thirty over three - and the bytes a second ten times each packet's size.
    /// `VehicleAndInspectorKeysDoNotAlias`: one packet from each of five systems, four of them
    /// 32-bit, kept apart - `GetID` is `(ulong)sysid << 8 | compid`, which none shares.
    /// `// C#: MissionPlannerTests/Mavlink/Sysid32Tests.cs:156-177`
    #[test]
    fn systems_with_32_bit_ids_are_kept_apart() {
        let start = Instant::now();
        let mut mavi = PacketInspector::default();
        let ids = [1, 257, 0x8000_0001, 0xffff_ff01, u32::MAX];
        for id in ids {
            mavi.add(id, 1, 0, id, 10, start);
        }
        let mut kept = mavi.packet_messages();
        kept.sort_unstable();
        assert_eq!(kept, ids);
        for id in ids {
            assert!(mavi.seen_bps(id, 1, 0, at(start, 1_000)) > 0.0, "{id:#x}");
        }
        assert!(mavi.seen_bps(2, 1, 0, at(start, 1_000)).abs() < f64::EPSILON);
    }

    #[test]
    fn a_steady_stream_is_its_rate() {
        let start = Instant::now();
        let mut mavi = PacketInspector::default();
        for tenth in 0..50 {
            mavi.add(1, 1, 30, tenth, 39, at(start, tenth * 100));
        }
        let now = at(start, 4_950);
        let rate = mavi.seen_rate(1, 1, 30, now);
        // 30 arrivals strictly inside (1.95 s, 4.95 s) - 2.0, 2.1 .. 4.9 - over three seconds.
        assert!((rate - 10.0).abs() < 1e-9, "{rate}");
        let bps = mavi.seen_bps(1, 1, 30, now);
        assert!((bps - 390.0).abs() < 1e-9, "{bps}");
        assert_eq!(mavi.packet_messages(), vec![49]);
    }

    /// A message heard for less than three seconds is counted over the time since its first
    /// arrival, not over three seconds.
    #[test]
    fn a_new_stream_is_counted_from_its_first_packet() {
        let start = Instant::now() + RATE_WINDOW;
        let mut mavi = PacketInspector::default();
        for step in 0..4 {
            mavi.add(1, 1, 0, (), 21, at(start, step * 250));
        }
        // 0, 250, 500 and 750 ms before 1000: four over one second.
        let rate = mavi.seen_rate(1, 1, 0, at(start, 1_000));
        assert!((rate - 4.0).abs() < 1e-9, "{rate}");
    }

    /// Only `RateHistory` arrivals are kept: a stream faster than that over three seconds is
    /// worked from its last 200, over the time they span.
    #[test]
    fn the_rate_history_is_two_hundred() {
        let start = Instant::now();
        let mut mavi = PacketInspector::default();
        for ms in 0..1_000 {
            mavi.add(1, 1, 27, (), 10, at(start, ms));
        }
        // 800 .. 999 held; 200 over the 0.2 s from 800 to 1000.
        let rate = mavi.seen_rate(1, 1, 27, at(start, 1_000));
        assert!((rate - 1_000.0).abs() < 1e-6, "{rate}");
    }

    /// A message that stopped three seconds ago is 0 a second but still listed; a message never
    /// heard is 0.
    #[test]
    fn a_stopped_stream_is_zero_and_kept() {
        let start = Instant::now();
        let mut mavi = PacketInspector::default();
        mavi.add(1, 1, 0, 'h', 9, start);
        assert_eq!(mavi.seen_rate(1, 1, 0, at(start, 3_500)), 0.0);
        assert_eq!(mavi.seen_rate(1, 1, 99, at(start, 10)), 0.0);
        assert_eq!(mavi.seen_bps(2, 1, 0, at(start, 10)), 0.0);
        assert_eq!(mavi.packet_messages(), vec!['h']);
    }

    /// Each system and component keeps its own newest packet of each message; a newer one of a
    /// message replaces the older.
    #[test]
    fn the_newest_of_each_message_from_each_component() {
        let start = Instant::now();
        let mut mavi = PacketInspector::default();
        mavi.add(1, 1, 0, "hb 1/1 old", 21, start);
        mavi.add(1, 1, 0, "hb 1/1", 21, start);
        mavi.add(1, 1, 30, "att 1/1", 39, start);
        mavi.add(255, 190, 0, "hb gcs", 21, start);
        mavi.add(1, 100, 0, "hb camera", 21, start);
        assert_eq!(
            mavi.packet_messages(),
            vec!["hb 1/1", "att 1/1", "hb camera", "hb gcs"]
        );
    }

    fn frame(sysid: u32, compid: u8, message: &MavMessage) -> Vec<u8> {
        let mut payload = [0u8; 255];
        let len = message.encode(&mut payload);
        let mut out = [0u8; mp_mavlink::MAX_FRAME_LEN];
        let n = encode_v2(
            &mut out,
            7,
            sysid,
            compid,
            message.id(),
            &payload[..len],
            message.crc_extra(),
            0,
        )
        .expect("encodes");
        out[..n].to_vec()
    }

    /// A written frame read back: its ids, its message and its whole length, marked sent; bytes
    /// that are not a frame are nothing.
    #[test]
    fn a_written_frame_is_a_sent_packet() {
        let heartbeat = MavMessage::Heartbeat(Heartbeat {
            custom_mode: 0,
            r#type: 6,
            autopilot: 8,
            base_mode: 0,
            system_status: 4,
            mavlink_version: 3,
        });
        let bytes = frame(255, 190, &heartbeat);
        let now = Instant::now();
        let packet = Packet::written(&bytes, DateTime::MIN, now).expect("a frame");
        assert_eq!((packet.sysid, packet.compid, packet.msgid), (255, 190, 0));
        assert_eq!(packet.message, heartbeat);
        assert_eq!(packet.length, bytes.len());
        assert!(packet.sent);
        assert_eq!(packet.at, now);
        assert!(Packet::written(&bytes[..5], DateTime::MIN, now).is_none());
    }

    /// A running link tells its subscriber of what the vehicle sends - with the frame's length
    /// and the arrival stamped - and of what it writes itself, its heartbeat from 255/190; a
    /// subscription is to the link it was made on, and a dropped one hears nothing more.
    #[test]
    fn a_link_tells_its_subscriber_both_ways() {
        use mp_transport::Transport as _;
        use mp_transport::testing::Loopback;

        let (mut vehicle, gcs) = Loopback::pair();
        let config = crate::LinkConfig {
            heartbeat_interval: Duration::from_millis(20),
            stream_rate_hz: 0,
            ..crate::LinkConfig::default()
        };
        let link = crate::Link::from_transport(Box::new(gcs), config);
        let heard: Arc<Mutex<Vec<Packet>>> = Arc::new(Mutex::new(Vec::new()));
        let into = Arc::clone(&heard);
        let subscription = link.on_packet(move |packet| {
            if let Ok(mut heard) = into.os_lock() {
                heard.push(*packet);
            }
        });
        assert!(link.carries(&subscription));
        let attitude = crate::testing::attitude(3, 0.25);
        vehicle
            .write_all(&crate::testing::heartbeat(2))
            .expect("heartbeat");
        vehicle.write_all(&attitude).expect("attitude");
        let deadline = Instant::now() + Duration::from_secs(5);
        let has = |sent: bool, msgid: u32| {
            heard
                .os_lock()
                .expect("not poisoned")
                .iter()
                .any(|packet| packet.sent == sent && packet.msgid == msgid)
        };
        while !(has(false, 30) && has(true, 0)) {
            assert!(Instant::now() < deadline, "the link told nothing");
            wasm_thread::sleep(Duration::from_millis(2));
        }
        drop(subscription);
        let seen = heard.os_lock().expect("not poisoned").clone();
        let received = seen
            .iter()
            .find(|packet| !packet.sent && packet.msgid == 30)
            .expect("the attitude");
        assert_eq!((received.sysid, received.compid), (1, 1));
        assert_eq!(received.length, attitude.len());
        assert!(matches!(received.message, MavMessage::Attitude(m) if m.roll == 0.25));
        let written = seen
            .iter()
            .find(|packet| packet.sent && packet.msgid == 0)
            .expect("our heartbeat");
        assert_eq!((written.sysid, written.compid), (255, 190));
        // Nothing after the drop, once a packet in flight at it has landed: the heartbeat goes
        // on being written every 20 ms, and nobody hears it.
        wasm_thread::sleep(Duration::from_millis(50));
        let count = heard.os_lock().expect("not poisoned").len();
        wasm_thread::sleep(Duration::from_millis(100));
        assert_eq!(heard.os_lock().expect("not poisoned").len(), count);
    }

    /// Subscribers hear every packet until their subscription is dropped, and a link nobody
    /// listens to says so.
    #[test]
    fn a_subscription_hears_until_dropped() {
        let shared = Arc::new(Shared::default());
        assert!(!shared.packets.any());
        let heard = Arc::new(Mutex::new(Vec::new()));
        let into = Arc::clone(&heard);
        let subscription = PacketSubscription::new(
            &shared,
            Box::new(move |packet: &Packet| {
                if let Ok(mut heard) = into.os_lock() {
                    heard.push((packet.msgid, packet.sent));
                }
            }),
        );
        assert!(shared.packets.any());
        assert!(subscription.is_on(&shared));
        assert!(!subscription.is_on(&Arc::new(Shared::default())));
        let attitude = MavMessage::Attitude(Attitude {
            time_boot_ms: 1,
            roll: 0.1,
            pitch: 0.2,
            yaw: 0.3,
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
        });
        let bytes = frame(1, 1, &attitude);
        shared.packets.sent(&bytes);
        let (parsed, _) = mp_mavlink::parse(&bytes, &DIALECT).expect("parses");
        shared.packets.notify(&Packet::received(
            &parsed,
            attitude,
            DateTime::now(),
            Instant::now(),
        ));
        drop(subscription);
        shared.packets.sent(&bytes);
        assert!(!shared.packets.any());
        assert_eq!(
            *heard.os_lock().expect("not poisoned"),
            vec![(30, true), (30, false)]
        );
    }
}
