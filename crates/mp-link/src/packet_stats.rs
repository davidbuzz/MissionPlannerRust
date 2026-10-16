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

//! What the connection control's Link Stats window counts from the link
//! (`Controls/ConnectionStats.cs`): `MAVLinkInterface`'s `WhenPacketReceived`, `WhenPacketLost`
//! and the time between received packets, kept in [`LinkStats`] beside the bytes read and written,
//! which are its `BytesReceived` and `BytesSent`.
//!
//! * Packets received, [`LinkStats::packets_received`]: one for every packet that has passed its
//!   checksum, its signature and the target check - a packet whose header names its target
//!   (`TARGET32`), and not this ground station or everyone, goes no further -
//!   `WhenPacketReceived.OnNext(1)`; in a log played back, not the ground station's own.
//!   `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5074-5076, 5104-5108, 5169`
//! * Packets lost, [`LinkStats::packets_lost`]: `WhenPacketLost`'s sum, from each vehicle's
//!   sequence numbers, [`lost_between`]. A vehicle's first packet - any that passes its checksum,
//!   before the signature and target checks - sets the number expected next, so a connection
//!   starts with no loss (`recvpacketcount = packetSeqNo - 1`, "prevent packetloss counter on
//!   connect"); a packet addressed elsewhere moves nothing, so the one after it counts it lost.
//!   `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5016-5023, 5133-5167`
//! * The longest interval between two received packets, [`LinkStats::max_packet_interval`]: the
//!   window's `WhenPacketReceived.TimeInterval()` folded with `Math.Max`, whose first interval runs
//!   from when it subscribed - its `Load`, again on Reset - and here from the link's start or the
//!   last [`crate::LinkSender::restart_stats`], [`LinkStats::interval_since`].
//!   `// C#: Controls/ConnectionStats.cs:35-37, 123-132, 186-190`
//!
//! The window's other numbers - the totals since it opened, the means over the last seconds - are
//! its own, from these counters (`crates/mp-gui/src/link_stats.rs`): a count it started is the
//! counter less what it was then, which costs the link thread nothing. The longest interval cannot
//! be had that way, so the link thread starts it again when asked, once a pass.

use std::collections::BTreeMap;
use web_time::{Duration, Instant};

use mp_vehicle::VehicleId;

use crate::LinkStats;

/// The sequence numbers' modulus, `0x100`.
const SEQUENCES: i32 = 0x100;

/// How many packets were lost before one numbered `seq`, its vehicle's last having been numbered
/// `last` - `recvpacketcount`, an `int`, -1 when its first packet was numbered 0: none when `seq`
/// is the one expected, `(last + 1) % 0x100`, or repeats `last` ("a 3dr radio bug sending dup
/// seqno's"); else the numbers between, counted round the wrap.
///
/// **Divergence:** after a packet numbered 255 the C# expects 0 but tests `packetSeqNo <
/// recvpacketcount + 1`, which is 256 then, and takes the wrap's branch, `0x100 - expected +
/// packetSeqNo`: 255 then 10 is 266 lost there, where 10 were. Here it is the numbers between,
/// the wrap counted once, as the branch's own comment means it to be.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5133-5151`
#[must_use]
pub fn lost_between(last: i32, seq: u8) -> u32 {
    let seq = i32::from(seq);
    let expected = (last + 1) % SEQUENCES;
    if seq == expected || seq == last {
        return 0;
    }
    u32::try_from((seq - expected).rem_euclid(SEQUENCES)).unwrap_or(0)
}

/// The link thread's counting: each vehicle's last sequence number, and when the last packet
/// came.
#[derive(Debug)]
pub(crate) struct Counter {
    /// `MAVState.recvpacketcount`, per vehicle.
    last: BTreeMap<VehicleId, i32>,
    /// When the last packet was received, or the longest interval started being counted.
    previous: Instant,
}

impl Counter {
    /// The count from `now`, the link's start: the longest interval counted from then.
    pub(crate) fn new(now: Instant, stats: &mut LinkStats) -> Self {
        stats.interval_since = Some(now);
        Self {
            last: BTreeMap::new(),
            previous: now,
        }
    }

    /// A packet from `id` numbered `seq` that has passed its checksum: true when it is the
    /// vehicle's first, whose number less one is then its last, so it is no loss.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5016-5023`
    pub(crate) fn first_heard(&mut self, id: VehicleId, seq: u8) -> bool {
        if self.last.contains_key(&id) {
            return false;
        }
        self.last.insert(id, i32::from(seq) - 1);
        true
    }

    /// A packet from `id` numbered `seq`, received at `at`, past the target check: counted, the
    /// numbers it skipped counted lost, its number the vehicle's last, and the interval since the
    /// packet before it - from any vehicle - the longest if it is.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5133-5169; Controls/ConnectionStats.cs:125-128`
    pub(crate) fn received(&mut self, id: VehicleId, seq: u8, at: Instant, stats: &mut LinkStats) {
        let last = self.last.entry(id).or_insert_with(|| i32::from(seq) - 1);
        stats.packets_lost += u64::from(lost_between(*last, seq));
        *last = i32::from(seq);
        stats.packets_received += 1;
        let interval = at.saturating_duration_since(self.previous);
        stats.max_packet_interval = stats.max_packet_interval.max(interval);
        self.previous = at;
    }

    /// The longest interval counted again from `at`: the window's `TimeInterval()` subscribed
    /// afresh. `// C#: Controls/ConnectionStats.cs:35-37, 186-190`
    pub(crate) fn restart(&mut self, at: Instant, stats: &mut LinkStats) {
        self.previous = at;
        stats.max_packet_interval = Duration::ZERO;
        stats.interval_since = Some(at);
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

    use super::*;
    use mp_mavlink::{FrameDecoder, MavVersion, Message as _, SigningKey, STX_V1, STX_V2};
    use mp_mavlink_dialects::all::{CommandLong, MavMessage, DIALECT};
    use mp_transport::testing::{Loopback, LoopbackEnd};
    use mp_transport::Transport as _;

    /// A packet in order, one repeated and the first a vehicle sends are no loss; a gap is the
    /// numbers in it; one round the wrap counts the wrap once; one older than the last is all but
    /// the numbers between, as the C#'s arithmetic has it.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5133-5151`
    #[test]
    fn the_loss_is_the_numbers_skipped() {
        assert_eq!(lost_between(7, 8), 0, "the one expected");
        assert_eq!(lost_between(7, 7), 0, "a radio's duplicate");
        assert_eq!(lost_between(-1, 0), 0, "a first packet numbered 0");
        assert_eq!(lost_between(10, 15), 4, "11 to 14");
        assert_eq!(lost_between(250, 5), 10, "251 to 255 and 0 to 4");
        assert_eq!(lost_between(255, 0), 0, "the wrap itself");
        // The C# counts 266 here (the divergence above): 0 to 9 were lost.
        assert_eq!(lost_between(255, 10), 10, "after 255, the wrap once");
        assert_eq!(lost_between(7, 6), 254, "one older: 0x100 - 8 + 6");
        assert_eq!(lost_between(254, 255), 0);
        assert_eq!(lost_between(254, 1), 2, "255 and 0");
    }

    /// Each vehicle's first packet sets its expectation, whatever its number; the counts are per
    /// vehicle, so another's numbers are no loss; a duplicate is counted received and not lost.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5016-5023, 5133-5169`
    #[test]
    fn a_vehicle_s_first_packet_sets_what_is_expected() {
        let start = Instant::now();
        let mut stats = LinkStats::default();
        let mut counter = Counter::new(start, &mut stats);
        let autopilot = VehicleId::new(1, 1);
        let radio = VehicleId::new(51, 68);
        let packet = |counter: &mut Counter, id: VehicleId, seq: u8, stats: &mut LinkStats| {
            counter.first_heard(id, seq);
            counter.received(id, seq, start, stats);
        };
        assert!(counter.first_heard(autopilot, 100));
        assert!(!counter.first_heard(autopilot, 101), "heard once");
        counter.received(autopilot, 100, start, &mut stats);
        packet(&mut counter, autopilot, 101, &mut stats);
        packet(&mut counter, radio, 200, &mut stats);
        packet(&mut counter, autopilot, 102, &mut stats);
        assert_eq!((stats.packets_received, stats.packets_lost), (4, 0));
        packet(&mut counter, autopilot, 106, &mut stats);
        assert_eq!((stats.packets_received, stats.packets_lost), (5, 3));
        packet(&mut counter, autopilot, 106, &mut stats);
        assert_eq!((stats.packets_received, stats.packets_lost), (6, 3));
        packet(&mut counter, radio, 202, &mut stats);
        assert_eq!((stats.packets_received, stats.packets_lost), (7, 4));
    }

    /// The longest interval between two packets, from any vehicle, the first measured from the
    /// count's start; started again, it is nothing until the next packet, which is measured from
    /// the restart. `// C#: Controls/ConnectionStats.cs:123-132`
    #[test]
    fn the_longest_interval_is_kept_and_started_again() {
        let ms = Duration::from_millis;
        let start = Instant::now();
        let mut stats = LinkStats::default();
        let mut counter = Counter::new(start, &mut stats);
        assert_eq!(stats.interval_since, Some(start));
        let a = VehicleId::new(1, 1);
        let b = VehicleId::new(2, 1);
        counter.first_heard(a, 0);
        counter.first_heard(b, 0);
        counter.received(a, 0, start + ms(30), &mut stats);
        assert_eq!(stats.max_packet_interval, ms(30), "from the start");
        counter.received(b, 0, start + ms(40), &mut stats);
        counter.received(a, 1, start + ms(120), &mut stats);
        counter.received(a, 2, start + ms(125), &mut stats);
        assert_eq!(stats.max_packet_interval, ms(80));
        counter.restart(start + ms(200), &mut stats);
        assert_eq!(stats.max_packet_interval, Duration::ZERO);
        assert_eq!(stats.interval_since, Some(start + ms(200)));
        counter.received(b, 1, start + ms(215), &mut stats);
        assert_eq!(stats.max_packet_interval, ms(15), "from the restart");
        assert_eq!(stats.packets_received, 5, "the counts go on");
    }

    // ---- over a link ----

    /// Polls until `check` holds, failing rather than hanging.
    fn until(what: &str, check: impl Fn() -> bool) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while !check() {
            assert!(Instant::now() < deadline, "timed out waiting for {what}");
            wasm_thread::sleep(Duration::from_millis(1));
        }
    }

    /// A COMMAND_LONG for `command` to (1, 1).
    fn command(command: u16) -> MavMessage {
        MavMessage::CommandLong(CommandLong {
            param1: 1.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command,
            target_system: 1,
            target_component: 1,
            confirmation: 0,
        })
    }

    /// The next frame the vehicle's end is sent that is a COMMAND_LONG for `wanted`, whole, with
    /// its framing and whether it is signed - skipping anything the link sends of its own accord.
    fn next_command(vehicle: &mut LoopbackEnd, wanted: u16) -> (Vec<u8>, MavVersion, bool) {
        let deadline = Instant::now() + Duration::from_secs(5);
        let mut bytes = Vec::new();
        let mut buf = [0u8; 4096];
        loop {
            let n = vehicle.read(&mut buf).unwrap_or(0);
            bytes.extend_from_slice(&buf[..n]);
            let mut found = None;
            let mut decoder = FrameDecoder::new();
            decoder.push_and_drain(&bytes, &DIALECT, |frame| {
                if found.is_none()
                    && matches!(
                        MavMessage::decode(frame.msgid, frame.payload),
                        Some(MavMessage::CommandLong(m)) if m.command == wanted
                    )
                {
                    found = Some((frame.raw.to_vec(), frame.version, frame.is_signed()));
                }
            });
            if let Some(found) = found {
                return found;
            }
            assert!(Instant::now() < deadline, "timed out waiting for {wanted}");
            wasm_thread::sleep(Duration::from_millis(1));
        }
    }

    /// The whole path, over a real link: the vehicle's packets counted as they pass, the numbers it
    /// skipped lost, its duplicate no loss; a packet it addresses to another system neither counted
    /// nor moving the expectation, so the next counts it lost, as the C#'s does; the longest
    /// interval started again when asked and measured from then.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5016-5023, 5074-5076, 5133-5169`
    #[test]
    fn a_link_counts_what_passes_and_what_was_lost() {
        let (mut vehicle, gcs) = Loopback::pair();
        let link = crate::Link::from_transport(
            Box::new(gcs),
            crate::LinkConfig {
                send_heartbeat: false,
                stream_rate_hz: 0,
                ..crate::LinkConfig::default()
            },
        );
        let sender = link.sender();
        for seq in [10, 11, 12, 15] {
            vehicle.write_all(&crate::testing::heartbeat(seq)).unwrap();
        }
        until("four packets", || sender.stats().packets_received >= 4);
        let stats = sender.stats();
        assert_eq!(
            (stats.packets_received, stats.packets_lost),
            (4, 2),
            "13, 14"
        );
        vehicle.write_all(&crate::testing::heartbeat(15)).unwrap();
        until("the duplicate", || sender.stats().packets_received >= 5);
        assert_eq!(sender.stats().packets_lost, 2, "a duplicate is no loss");

        // Its header naming system 70,000, not this ground station: dropped, not counted, and 16
        // is lost to the count when 17 comes.
        let mut elsewhere = command(31_000);
        if let MavMessage::CommandLong(m) = &mut elsewhere {
            m.target_system = 255;
        }
        let mut payload = [0u8; 255];
        let len = elsewhere.encode(&mut payload);
        let mut wide = [0u8; mp_mavlink::MAX_FRAME_LEN];
        let n = mp_mavlink::encode_v2_targeted(
            &mut wide,
            16,
            1,
            1,
            elsewhere.id(),
            &payload[..len],
            elsewhere.crc_extra(),
            0,
            Some(70_000),
        )
        .unwrap();
        vehicle.write_all(&wide[..n]).unwrap();
        vehicle.write_all(&crate::testing::heartbeat(17)).unwrap();
        until("17", || sender.stats().packets_received >= 6);
        let stats = sender.stats();
        assert_eq!((stats.packets_received, stats.packets_lost), (6, 3));
        assert!(stats.max_packet_interval > Duration::ZERO);

        // Started again: nothing until the next packet, which is measured from the restart.
        let at = Instant::now();
        sender.restart_stats(at);
        until("the restart", || sender.stats().interval_since == Some(at));
        assert_eq!(sender.stats().max_packet_interval, Duration::ZERO);
        wasm_thread::sleep(Duration::from_millis(30));
        vehicle.write_all(&crate::testing::heartbeat(18)).unwrap();
        until("18", || sender.stats().packets_received >= 7);
        assert!(sender.stats().max_packet_interval >= Duration::from_millis(30));
        assert_eq!(sender.stats().packets_lost, 3);
    }

    /// The window's two ticks, as the bytes the vehicle is sent show them: "Mavlink 2" off frames
    /// what goes to it as MAVLink 1 - the base payload, unsigned - until a MAVLink 2 packet from it
    /// turns it back on; "Signing" on signs what goes to it, with zeros while it has no key, and off
    /// sends it unsigned; with both, MAVLink 1 and unsigned, as `generatePacket`'s first branch.
    /// `// C#: Controls/ConnectionStats.cs:176-184; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1302-1344, 1381-1437, 5025-5027`
    #[test]
    fn the_ticks_change_what_is_sent() {
        let (mut vehicle, gcs) = Loopback::pair();
        let link = crate::Link::from_transport(
            Box::new(gcs),
            crate::LinkConfig {
                send_heartbeat: false,
                stream_rate_hz: 0,
                ..crate::LinkConfig::default()
            },
        );
        let sender = link.sender();
        let id = VehicleId::new(1, 1);
        vehicle.write_all(&crate::testing::heartbeat(0)).unwrap();
        until("the vehicle", || sender.mavlink2(id) == Some(true));

        assert!(sender.send(&command(31_001)));
        let (raw, version, signed) = next_command(&mut vehicle, 31_001);
        assert_eq!((raw[0], version, signed), (STX_V2, MavVersion::V2, false));

        sender.set_mavlink2(id, false);
        assert_eq!(sender.mavlink2(id), Some(false));
        assert!(sender.send(&command(31_002)));
        let (raw, version, signed) = next_command(&mut vehicle, 31_002);
        assert_eq!((raw[0], version, signed), (STX_V1, MavVersion::V1, false));
        assert_eq!(
            usize::from(raw[1]),
            CommandLong::MIN_LEN,
            "the base payload"
        );
        assert_eq!(raw.len(), 6 + CommandLong::MIN_LEN + 2);

        // Its MAVLink 2 packet: "once set it cannot be reverted".
        vehicle.write_all(&crate::testing::heartbeat(1)).unwrap();
        until("MAVLink 2 again", || sender.mavlink2(id) == Some(true));
        assert!(sender.send(&command(31_003)));
        assert_eq!(next_command(&mut vehicle, 31_003).1, MavVersion::V2);

        sender.set_signing(id, true);
        assert!(sender.signing(id).is_some_and(|state| state.signing));
        assert!(sender.send(&command(31_004)));
        let (raw, version, signed) = next_command(&mut vehicle, 31_004);
        assert_eq!((version, signed), (MavVersion::V2, true));
        let mut checked = false;
        FrameDecoder::new().push_and_drain(&raw, &DIALECT, |frame| {
            checked = mp_mavlink::verify(&SigningKey::new([0; 32]), frame);
        });
        assert!(checked, "signed with zeros: no key yet");

        sender.set_mavlink2(id, false);
        assert!(sender.send(&command(31_005)));
        let (_, version, signed) = next_command(&mut vehicle, 31_005);
        assert_eq!(
            (version, signed),
            (MavVersion::V1, false),
            "MAVLink 1 is never signed"
        );

        sender.set_mavlink2(id, true);
        sender.set_signing(id, false);
        assert!(sender.send(&command(31_006)));
        let (_, version, signed) = next_command(&mut vehicle, 31_006);
        assert_eq!((version, signed), (MavVersion::V2, false));
    }
}
