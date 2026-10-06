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

//! A mirror of the link: `MAVLinkInterface.Mirrors` and `ProcessMirrorStream`, the stream the
//! Support Proxy (`Controls/SerialSupportProxy.cs`) sets as `MainV2.comPort.MirrorStream`, with
//! `MirrorStreamWrite` true, so that a support engineer's ground station far away shares the
//! vehicle's link.
//!
//! What the C# does with it (`ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5443-5474`), for
//! every packet it reads from the vehicle - each one it decodes (`:5429`), and each of a message
//! it does not know (`:4993`): while the stream is open, write the packet's bytes to it; then,
//! while the stream has bytes to read, read them and - when `MirrorStreamWrite` is set - write
//! them to the vehicle's port. Whatever throws is swallowed. Nothing is read from the stream but
//! after a packet from the vehicle has been written to it. `Mirrors` is a list, of which
//! `MirrorStream` and `MirrorStreamWrite` are the first (`:429-462`); everything that sets one
//! sets the first, so it is one mirror here.
//!
//! A [`Mirror`] is that stream with a thread of its own. [`Mirror::handler`] is what to give
//! [`crate::Link::on_packet`]: it hands each packet the link reads to the mirror's thread, which
//! writes it to the stream and then reads what the stream has, and [`Mirror::attach`] says which
//! link that goes to.
//!
//! Where this is not the C#, and why:
//!
//! * the writing and reading are the mirror's thread's, not the link thread's: the C# does them
//!   on the thread that reads the vehicle, so a stream that stalls stalls the vehicle's link,
//!   which this link never does (see the crate's threading model). The packets wait in a queue
//!   of [`QUEUE`] for the thread; one that finds it full is dropped and counted;
//! * the bytes. A packet subscriber is handed the packet decoded, not its bytes, so each is
//!   framed again as MAVLink 2 from its own system and component, with a sequence number the
//!   mirror keeps for that system and component: what reaches the stream is the same message
//!   from the same sender, but a MAVLink 1 packet arrives as 2, a signature is not carried, and a
//!   message the dialect does not know is not mirrored. What the stream sends back is read as
//!   frames, and each whole frame of a message the dialect knows is queued on the link as
//!   MAVLink 2 from its own system and component - the link numbers it in its own sequence, and
//!   records it and tells its packet subscribers of it, as it does everything it writes; the C#
//!   writes the bytes to the port as they came. Bytes that are not a frame are dropped. A tap of
//!   the raw bytes would need a hook in the link thread's read loop and its writer;
//! * `TcpSerial`'s `autoReconnect` (`ExtLibs/Comms/CommsTCPSerial.cs:86-87, 330-361`): a stream
//!   that has closed is opened again by [`Reopen`], at most every [`RECONNECT_EVERY`] -
//!   `doAutoReconnect`'s pace - and not also on every packet, as `VerifyConnected`'s retries do
//!   (`:363-386`), each one a connect the vehicle's reader waits for.

use mp_os::Lock as _;
use mp_os::RecvTimeout as _;
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, SyncSender, TrySendError, sync_channel};
use std::sync::{Arc, Mutex};
use wasm_thread::JoinHandle;
use web_time::{Duration, Instant};

use mp_mavlink::{Dialect as _, FrameDecoder, encode_v2};
use mp_mavlink_dialects::all::{DIALECT, MavMessage};
use mp_transport::Transport;

use crate::LinkSender;
use crate::inspector::Packet;

/// How many packets wait for the mirror's thread; one more is dropped.
pub const QUEUE: usize = 1024;

/// How long one read of the stream waits for bytes: `BytesToRead` asks without waiting, and a
/// read here returns what has come within this.
const READ_WAIT: Duration = Duration::from_millis(1);

/// How long the thread waits for a packet before it looks again at whether it should stop, or
/// open a closed stream again.
const IDLE: Duration = Duration::from_millis(50);

/// How long [`Mirror::close`] waits for the thread: longer than a pass of it takes.
pub const CLOSE_WAIT: Duration = Duration::from_millis(500);

/// `doAutoReconnect`'s pace: `lastReconnectTime = DateTime.Now.AddSeconds(5)`.
/// `// C#: ExtLibs/Comms/CommsTCPSerial.cs:355`
pub const RECONNECT_EVERY: Duration = Duration::from_secs(5);

/// How a stream that has closed is opened again: `TcpSerial.autoReconnect`, to the host and
/// port it was opened to. `None` from it is a connect that failed, tried again later.
pub type Reopen = Box<dyn FnMut() -> Option<Box<dyn Transport>> + Send>;

/// What the mirror has passed, each way.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Relayed {
    /// Packets from the vehicle written to the stream.
    pub up: u64,
    /// Frames from the stream queued on the link.
    pub down: u64,
    /// Packets from the vehicle dropped because the queue was full.
    pub dropped: u64,
}

/// What the mirror's owner and its thread share.
#[derive(Debug)]
struct Shared {
    /// Until [`Mirror::close`].
    running: AtomicBool,
    /// `MirrorStream.IsOpen`, as the thread last saw it.
    open: AtomicBool,
    /// `MirrorStreamWrite`.
    write: AtomicBool,
    /// Where what the stream sends goes: the link's sender, once attached.
    link: Mutex<Option<LinkSender>>,
    /// [`Relayed::up`].
    up: AtomicU64,
    /// [`Relayed::down`].
    down: AtomicU64,
    /// [`Relayed::dropped`].
    dropped: AtomicU64,
}

/// `MainV2.comPort.MirrorStream`, open, and its thread.
pub struct Mirror {
    /// What the thread shares.
    shared: Arc<Shared>,
    /// The packets' way to the thread; held here so it is never closed while the mirror is.
    queue: SyncSender<Vec<u8>>,
    /// The thread, until [`Mirror::close`] has waited for it.
    thread: Option<JoinHandle<()>>,
}

impl std::fmt::Debug for Mirror {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Mirror")
            .field("open", &self.is_open())
            .field("relayed", &self.relayed())
            .finish_non_exhaustive()
    }
}

impl Mirror {
    /// The stream, already opened, made the mirror: `MirrorStream = serial; MirrorStreamWrite =
    /// write`, and its thread started.
    ///
    /// # Errors
    ///
    /// The thread could not be started.
    /// `// C#: Controls/SerialSupportProxy.cs:79-80`
    pub fn start(
        stream: Box<dyn Transport>,
        write: bool,
        reopen: Option<Reopen>,
    ) -> std::io::Result<Self> {
        let shared = Arc::new(Shared {
            running: AtomicBool::new(true),
            open: AtomicBool::new(stream.is_open()),
            write: AtomicBool::new(write),
            link: Mutex::new(None),
            up: AtomicU64::new(0),
            down: AtomicU64::new(0),
            dropped: AtomicU64::new(0),
        });
        let (queue, packets) = sync_channel(QUEUE);
        let thread_shared = Arc::clone(&shared);
        let thread = wasm_thread::Builder::new()
            .name("mp-mirror".to_owned())
            .spawn(move || run(stream, reopen, &packets, &thread_shared))?;
        Ok(Self {
            shared,
            queue,
            thread: Some(thread),
        })
    }

    /// What to subscribe to the link's packets with: each packet the link reads from the
    /// vehicle - not one it writes - framed and handed to the mirror's thread, never waited for.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4993, 5429`
    pub fn handler(&self) -> impl FnMut(&Packet) + Send + 'static {
        let queue = self.queue.clone();
        let shared = Arc::clone(&self.shared);
        let mut sequence: BTreeMap<(u32, u8), u8> = BTreeMap::new();
        move |packet: &Packet| {
            if packet.sent || !shared.running.load(Ordering::Acquire) {
                return;
            }
            let seq = sequence.entry((packet.sysid, packet.compid)).or_insert(0);
            let Some(frame) = message_frame(*seq, packet.sysid, packet.compid, &packet.message)
            else {
                return;
            };
            *seq = seq.wrapping_add(1);
            if let Err(TrySendError::Full(_)) = queue.try_send(frame) {
                shared.dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// The link what the stream sends is written to - `BaseStream`, the vehicle's port - or
    /// none, when there is no link to write to.
    pub fn attach(&self, link: Option<LinkSender>) {
        if let Ok(mut held) = self.shared.link.os_lock() {
            *held = link;
        }
    }

    /// `MirrorStream.IsOpen`, as the mirror's thread last saw it.
    #[must_use]
    pub fn is_open(&self) -> bool {
        self.shared.running.load(Ordering::Acquire) && self.shared.open.load(Ordering::Acquire)
    }

    /// `MirrorStreamWrite`.
    #[must_use]
    pub fn writes(&self) -> bool {
        self.shared.write.load(Ordering::Acquire)
    }

    /// `MirrorStreamWrite = write`.
    pub fn set_writes(&self, write: bool) {
        self.shared.write.store(write, Ordering::Release);
    }

    /// What it has passed so far.
    #[must_use]
    pub fn relayed(&self) -> Relayed {
        Relayed {
            up: self.shared.up.load(Ordering::Relaxed),
            down: self.shared.down.load(Ordering::Relaxed),
            dropped: self.shared.dropped.load(Ordering::Relaxed),
        }
    }

    /// `MirrorStream.Close()`: the thread stopped and the stream closed by it - waited for up
    /// to [`CLOSE_WAIT`], which a pass takes; a thread still in a reconnect's connect is left to
    /// close the stream when that returns, rather than holding whoever closes it.
    pub fn close(&mut self) {
        self.shared.running.store(false, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            // On a web page's main thread nothing may wait (`mp_os::may_block`): told to stop,
            // the thread ends on its own.
            let deadline = Instant::now()
                + if mp_os::may_block() {
                    CLOSE_WAIT
                } else {
                    Duration::ZERO
                };
            while !thread.is_finished() && Instant::now() < deadline {
                wasm_thread::sleep(Duration::from_millis(1));
            }
            if thread.is_finished() {
                let _ = thread.join();
            }
        }
        self.shared.open.store(false, Ordering::Release);
        self.attach(None);
    }
}

impl Drop for Mirror {
    fn drop(&mut self) {
        self.close();
    }
}

/// A message framed as MAVLink 2, frame `seq` from `sysid` and `compid`.
fn message_frame(seq: u8, sysid: u32, compid: u8, message: &MavMessage) -> Option<Vec<u8>> {
    let mut payload = [0u8; 255];
    let len = message.encode(&mut payload);
    frame(
        seq,
        sysid,
        compid,
        message.id(),
        payload.get(..len)?,
        message.crc_extra(),
    )
}

/// A MAVLink 2 frame of a payload.
fn frame(
    seq: u8,
    sysid: u32,
    compid: u8,
    msgid: u32,
    payload: &[u8],
    crc_extra: u8,
) -> Option<Vec<u8>> {
    let mut out = [0u8; mp_mavlink::MAX_FRAME_LEN];
    let n = encode_v2(&mut out, seq, sysid, compid, msgid, payload, crc_extra, 0).ok()?;
    out.get(..n).map(<[u8]>::to_vec)
}

/// The mirror's thread: `ProcessMirrorStream` for each packet the link hands it.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5443-5474`
fn run(
    mut stream: Box<dyn Transport>,
    mut reopen: Option<Reopen>,
    packets: &Receiver<Vec<u8>>,
    shared: &Shared,
) {
    let _ = stream.set_read_timeout(READ_WAIT);
    let mut decoder = FrameDecoder::new();
    let mut buffer = [0u8; 4096];
    // `lastReconnectTime` starts at `DateTime.MinValue`: the first try is at once.
    let mut next_reconnect = Instant::now();
    while shared.running.load(Ordering::Acquire) {
        let packet = match packets.os_recv_timeout(IDLE) {
            Ok(packet) => Some(packet),
            Err(RecvTimeoutError::Timeout) => None,
            Err(RecvTimeoutError::Disconnected) => break,
        };
        // `MirrorStream.IsOpen`, which for a TCP stream that reconnects tries again when it is
        // not (CommsTCPSerial.cs:86-87).
        if !stream.is_open()
            && let Some(open) = reopen.as_mut()
            && Instant::now() >= next_reconnect
        {
            next_reconnect = Instant::now() + RECONNECT_EVERY;
            if let Some(mut fresh) = open() {
                let _ = fresh.set_read_timeout(READ_WAIT);
                stream = fresh;
                decoder = FrameDecoder::new();
            }
        }
        let open = stream.is_open();
        shared.open.store(open, Ordering::Release);
        let Some(packet) = packet else {
            continue;
        };
        if !open {
            continue;
        }
        // `MirrorStream.Write(buffer, 0, buffer.Length)`; a failure swallowed.
        if stream.write_all(&packet).is_ok() {
            shared.up.fetch_add(1, Ordering::Relaxed);
        }
        // `while (MirrorStream.BytesToRead > 0)`: read, and write it to the vehicle when
        // `MirrorStreamWrite`.
        loop {
            let n = match stream.read(&mut buffer) {
                Ok(0) | Err(_) => break,
                Ok(n) => n,
            };
            let write = shared.write.load(Ordering::Acquire);
            let Some(read) = buffer.get(..n) else {
                break;
            };
            decoder.push_and_drain(read, &DIALECT, |frame| {
                if !write {
                    return;
                }
                let Some(crc_extra) = DIALECT.crc_extra(frame.msgid) else {
                    return;
                };
                let Some(bytes) = self::frame(
                    0,
                    frame.sysid,
                    frame.compid,
                    frame.msgid,
                    frame.payload,
                    crc_extra,
                ) else {
                    return;
                };
                let sent = shared
                    .link
                    .os_lock()
                    .ok()
                    .and_then(|link| link.as_ref().map(|link| link.outbound.send(bytes).is_ok()))
                    .unwrap_or(false);
                if sent {
                    shared.down.fetch_add(1, Ordering::Relaxed);
                }
            });
        }
        // A write or read that failed closes a stream; the next pass may open it again.
        shared.open.store(stream.is_open(), Ordering::Release);
    }
    stream.close();
    shared.open.store(false, Ordering::Release);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Link, LinkConfig};
    use mp_mavlink::encode_v1;
    use mp_mavlink_dialects::all::{CommandLong, Heartbeat, ParamRequestRead};
    use mp_transport::testing::{Loopback, LoopbackEnd};

    /// Waits for `check`, failing rather than hanging.
    fn until(what: &str, mut check: impl FnMut() -> bool) {
        let deadline = Instant::now() + Duration::from_secs(10);
        while !check() {
            assert!(Instant::now() < deadline, "timed out waiting for {what}");
            wasm_thread::sleep(Duration::from_millis(2));
        }
    }

    /// The vehicle's heartbeat.
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

    /// What the support engineer's ground station asks: a parameter.
    fn request() -> MavMessage {
        MavMessage::ParamRequestRead(ParamRequestRead {
            param_index: -1,
            target_system: 1,
            target_component: 1,
            param_id: *b"SYSID_THISMAV\0\0\0",
        })
    }

    /// Everything one end of a loopback has, decoded: `(seq, sysid, compid, message)`.
    fn heard(end: &mut LoopbackEnd, decoder: &mut FrameDecoder) -> Vec<(u8, u32, u8, MavMessage)> {
        let mut out = Vec::new();
        let mut buffer = [0u8; 4096];
        loop {
            let n = end.read(&mut buffer).unwrap_or(0);
            if n == 0 {
                break;
            }
            decoder.push_and_drain(&buffer[..n], &DIALECT, |frame| {
                if let Some(message) = MavMessage::decode(frame.msgid, frame.payload) {
                    out.push((frame.seq, frame.sysid, frame.compid, message));
                }
            });
        }
        out
    }

    /// A link with a vehicle on one end of a loopback, the link sending nothing of its own.
    fn link() -> (Link, LoopbackEnd) {
        let (vehicle, gcs) = Loopback::pair();
        let config = LinkConfig {
            send_heartbeat: false,
            stream_rate_hz: 0,
            ..LinkConfig::default()
        };
        (Link::from_transport(Box::new(gcs), config), vehicle)
    }

    /// The vehicle's packets reach the support server, framed again from the vehicle; what the
    /// server sends - MAVLink 2 or 1 - reaches the vehicle from the server's system, after the
    /// next packet from the vehicle, as `ProcessMirrorStream` reads only then.
    #[test]
    fn the_mirror_relays_both_ways() {
        let (link, mut vehicle) = link();
        let (server_side, mut server) = Loopback::pair();
        let mirror = Mirror::start(Box::new(server_side), true, None).expect("a thread");
        let _subscription = link.on_packet(mirror.handler());
        mirror.attach(Some(link.sender()));
        assert!(mirror.is_open() && mirror.writes());

        let mut to_server = FrameDecoder::new();
        vehicle
            .write_all(&crate::testing::frame(7, &heartbeat()))
            .expect("written");
        let mut got = Vec::new();
        until("the heartbeat at the server", || {
            got.extend(heard(&mut server, &mut to_server));
            !got.is_empty()
        });
        assert_eq!(got, [(0, 1, 1, heartbeat())], "seq is the mirror's own");
        assert_eq!(mirror.relayed().up, 1);

        // The server asks, as MAVLink 2 from 253/190 and as MAVLink 1 from 252/190.
        let mut payload = [0u8; 255];
        let len = request().encode(&mut payload);
        server
            .write_all(
                &frame(
                    9,
                    253,
                    190,
                    request().id(),
                    &payload[..len],
                    request().crc_extra(),
                )
                .expect("a frame"),
            )
            .expect("written");
        let mut v1 = [0u8; mp_mavlink::MAX_FRAME_LEN];
        let n = encode_v1(
            &mut v1,
            4,
            252,
            190,
            request().id(),
            &payload[..len],
            request().crc_extra(),
        )
        .expect("a v1 frame");
        server.write_all(&v1[..n]).expect("written");
        // Nothing is read from the server until the vehicle's next packet.
        wasm_thread::sleep(Duration::from_millis(100));
        let mut to_vehicle = FrameDecoder::new();
        assert!(heard(&mut vehicle, &mut to_vehicle).is_empty());
        vehicle
            .write_all(&crate::testing::frame(8, &heartbeat()))
            .expect("written");
        let mut down = Vec::new();
        until("the requests at the vehicle", || {
            down.extend(heard(&mut vehicle, &mut to_vehicle));
            down.len() >= 2
        });
        let senders: Vec<(u32, u8, MavMessage)> = down
            .iter()
            .map(|(_, sysid, compid, message)| (*sysid, *compid, *message))
            .collect();
        assert_eq!(senders, [(253, 190, request()), (252, 190, request())]);
        assert_eq!(mirror.relayed().down, 2);
        assert_eq!(mirror.relayed().up, 2);
        let more = heard(&mut server, &mut to_server);
        assert_eq!(
            more.first().map(|(seq, ..)| *seq),
            Some(1),
            "the mirror's next"
        );
    }

    /// `MirrorStreamWrite` false: what the server sends is read and not written; what the link
    /// itself sends is not mirrored; and once closed nothing passes and the stream is closed.
    #[test]
    fn a_mirror_that_does_not_write_reads_and_drops() {
        let (link, mut vehicle) = link();
        let (server_side, mut server) = Loopback::pair();
        let mut mirror = Mirror::start(Box::new(server_side), false, None).expect("a thread");
        let _subscription = link.on_packet(mirror.handler());
        mirror.attach(Some(link.sender()));
        let command = MavMessage::CommandLong(CommandLong {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: 520,
            target_system: 1,
            target_component: 1,
            confirmation: 0,
        });
        // The link's own message is sent, not read: not mirrored.
        assert!(link.send(&command));
        let mut to_vehicle = FrameDecoder::new();
        until("the link's command at the vehicle", || {
            !heard(&mut vehicle, &mut to_vehicle).is_empty()
        });
        server
            .write_all(&crate::testing::frame(3, &request()))
            .expect("written");
        vehicle
            .write_all(&crate::testing::frame(1, &heartbeat()))
            .expect("written");
        until("the heartbeat mirrored", || mirror.relayed().up == 1);
        // The server's bytes are read in the same pass as the heartbeat is written.
        wasm_thread::sleep(Duration::from_millis(50));
        assert!(heard(&mut vehicle, &mut to_vehicle).is_empty());
        assert_eq!(mirror.relayed().down, 0);

        mirror.close();
        assert!(!mirror.is_open());
        vehicle
            .write_all(&crate::testing::frame(2, &heartbeat()))
            .expect("written");
        wasm_thread::sleep(Duration::from_millis(50));
        assert_eq!(mirror.relayed().up, 1);
        assert!(heard(&mut server, &mut FrameDecoder::new()).len() <= 1);
    }

    /// A stream that closes is opened again by `reopen`, and what the vehicle sends after goes
    /// to the new one.
    #[test]
    fn a_closed_stream_is_opened_again() {
        let (link, mut vehicle) = link();
        let (first_side, mut first) = Loopback::pair();
        let (second_side, mut second) = Loopback::pair();
        let mut spare = Some(second_side);
        let reopen: Reopen =
            Box::new(move || spare.take().map(|end| Box::new(end) as Box<dyn Transport>));
        let mirror = Mirror::start(Box::new(first_side), true, Some(reopen)).expect("a thread");
        let _subscription = link.on_packet(mirror.handler());
        first.disconnect();
        // The cable is learned of at the next write, which is lost, as `ProcessMirrorStream`'s
        // swallowed exception loses it; the next pass opens the stream again, and a heartbeat
        // after that reaches the new one.
        let mut decoder = FrameDecoder::new();
        let mut got = Vec::new();
        let mut seq = 0_u8;
        until("a heartbeat at the second stream", || {
            seq = seq.wrapping_add(1);
            let _ = vehicle.write_all(&crate::testing::frame(seq, &heartbeat()));
            wasm_thread::sleep(Duration::from_millis(20));
            got.extend(heard(&mut second, &mut decoder));
            !got.is_empty()
        });
        assert!(mirror.is_open());
        assert!(first.read(&mut [0u8; 4]).is_err(), "the first is gone");
        assert!(mirror.relayed().up >= 1);
    }

    /// A stream whose write waits until released.
    struct Gate {
        /// Each write waits for one of these, or for the sender to go.
        release: Receiver<()>,
    }

    impl Transport for Gate {
        fn read(&mut self, _buf: &mut [u8]) -> std::io::Result<usize> {
            Ok(0)
        }

        fn write_all(&mut self, _buf: &[u8]) -> std::io::Result<()> {
            let _ = self.release.recv();
            Ok(())
        }

        fn description(&self) -> &str {
            "gate"
        }

        fn is_open(&self) -> bool {
            true
        }

        fn set_read_timeout(&mut self, _timeout: Duration) -> std::io::Result<()> {
            Ok(())
        }
    }

    /// A full queue drops and counts; the link is never made to wait.
    #[test]
    fn a_full_queue_drops() {
        let (release, gate) = std::sync::mpsc::channel();
        let mut mirror =
            Mirror::start(Box::new(Gate { release: gate }), true, None).expect("a thread");
        let mut handler = mirror.handler();
        let packet = Packet {
            sysid: 1,
            compid: 1,
            msgid: 0,
            message: heartbeat(),
            length: 21,
            rxtime: mp_vehicle::DateTime::MIN,
            at: Instant::now(),
            sent: false,
        };
        // The first is taken and its write waits; the queue then fills, and five more are
        // dropped, the handler never waiting.
        handler(&packet);
        wasm_thread::sleep(Duration::from_millis(100));
        let started = Instant::now();
        for _ in 0..QUEUE + 5 {
            handler(&packet);
        }
        assert!(started.elapsed() < Duration::from_secs(1));
        assert_eq!(mirror.relayed().dropped, 5);
        drop(release);
        until("the queue written", || {
            mirror.relayed().up == 1 + QUEUE as u64
        });
        mirror.close();
    }
}
