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

//! Fault injection over the mock transport, carrying real MAVLink frames.
//!
//! Each fault a link meets in the field - dropped bytes, duplicated bytes, repeated and reordered
//! datagrams, a frame written in two pieces, a cable pulled mid-frame - is injected into a stretch
//! of a recorded ArduPilot flight, and the far end decodes it the way the link engine does. What
//! must hold:
//!
//! - nothing panics;
//! - the frames delivered are exactly the ones the fault left intact, in wire order, and each one
//!   passes its checksum when checked independently of the decoder;
//! - a disconnect is an `Err` from `read` and `write_all`, which a caller can tell from the `Ok(0)`
//!   of a timeout.
//!
//! The last four tests put the faults that can happen to them to the network transports - the UDP
//! client, the websocket and NTRIP - over real sockets on 127.0.0.1.
//!
//! What the transport does *not* owe: it does not de-duplicate or reorder. A repeated frame is
//! delivered twice and a late one late; MAVLink's sequence number is how the link notices, and
//! doing it there is the link's job (D4), not the byte pipe's.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

mod common;

use std::io;
use std::ops::Range;

use mp_mavlink::{DecodeStats, Dialect, FrameDecoder, STX_V2, crc, encode_v2};
use mp_mavlink_dialects::all::DIALECT;
use mp_transport::Transport;
use mp_transport::testing::{Fault, Loopback, LoopbackEnd};

/// Frames skipped at the start of the recording, so the stretch used is mid-flight traffic with
/// many message types rather than the connection handshake.
const SKIP: usize = 2_000;

/// `count` consecutive real frames from the autotest flight, without the tlog's timestamps.
fn real_frames(count: usize) -> Vec<Vec<u8>> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink/autotest.tlog"
    );
    let data = std::fs::read(path).unwrap();
    let mut decoder = FrameDecoder::new();
    let mut frames = Vec::new();
    decoder.push_and_drain(&data, &DIALECT, |frame| {
        if frames.len() < SKIP + count {
            frames.push(frame.raw.to_vec());
        }
    });
    assert_eq!(frames.len(), SKIP + count, "the recording is too short");
    frames.split_off(SKIP)
}

/// Where each frame sits in the frames laid end to end.
fn spans(frames: &[Vec<u8>]) -> Vec<Range<usize>> {
    let mut start = 0;
    frames
        .iter()
        .map(|frame| {
            let span = start..start + frame.len();
            start = span.end;
            span
        })
        .collect()
}

/// Checks a frame's checksum from first principles, not through the decoder under test.
fn checksum_ok(raw: &[u8]) -> bool {
    let header = if raw[0] == STX_V2 { 10 } else { 6 };
    let len = usize::from(raw[1]);
    let msgid = if raw[0] == STX_V2 {
        u32::from_le_bytes([raw[7], raw[8], raw[9], 0])
    } else {
        u32::from(raw[5])
    };
    let Some(extra) = DIALECT.crc_extra(msgid) else {
        return false;
    };
    let end = header + len;
    let sent = u16::from_le_bytes([raw[end], raw[end + 1]]);
    crc::checksum(&raw[1..end], extra) == sent
}

/// What one end received.
struct Delivered {
    frames: Vec<Vec<u8>>,
    stats: DecodeStats,
    error: Option<io::Error>,
}

/// Reads until the link is idle or fails, decoding as the link engine does, then flushes as the
/// link does when a link goes quiet, so nothing is left waiting on a partial frame.
fn receive(end: &mut LoopbackEnd, read_size: usize) -> Delivered {
    let mut decoder = FrameDecoder::new();
    let mut frames = Vec::new();
    let mut buf = vec![0u8; read_size];
    let mut error = None;
    loop {
        match end.read(&mut buf) {
            // The mock never blocks, so zero means the wire is empty.
            Ok(0) => break,
            Ok(n) => {
                decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                    frames.push(frame.raw.to_vec());
                });
            }
            Err(e) => {
                error = Some(e);
                break;
            }
        }
    }
    decoder.flush(&DIALECT, |frame| frames.push(frame.raw.to_vec()));
    Delivered {
        frames,
        stats: *decoder.stats(),
        error,
    }
}

/// Sends every frame as its own write, the way the link engine sends.
fn send_all(end: &mut LoopbackEnd, frames: &[Vec<u8>]) {
    for frame in frames {
        end.write_all(frame).unwrap();
    }
}

fn assert_all_pass_checksum(frames: &[Vec<u8>]) {
    for (i, frame) in frames.iter().enumerate() {
        assert!(checksum_ok(frame), "delivered frame {i} fails its checksum");
    }
}

/// The stream positions an every-Nth byte fault lands on.
fn every_nth(total: usize, n: usize) -> Vec<usize> {
    (0..total).filter(|i| (i + 1) % n == 0).collect()
}

#[test]
fn a_clean_link_delivers_every_frame_intact_through_short_reads() {
    let frames = real_frames(300);
    for max_read in [1, 7, 64, 4096] {
        let (mut vehicle, gcs) = Loopback::pair();
        let mut gcs = gcs.with_fault(Fault {
            max_read,
            ..Fault::default()
        });
        send_all(&mut vehicle, &frames);
        let got = receive(&mut gcs, 4096);
        assert!(got.error.is_none());
        assert_eq!(got.frames, frames, "max_read {max_read}");
        assert_eq!(got.stats.crc_errors, 0);
        assert_eq!(got.stats.resync_bytes, 0);
    }
}

#[test]
fn dropped_bytes_cost_exactly_the_frames_they_land_in() {
    let frames = real_frames(300);
    let spans = spans(&frames);
    let total = spans.last().unwrap().end;

    for drop_every in [97, 509, 1_021] {
        let dropped = every_nth(total, drop_every);
        // A frame with a byte missing is gone; every other frame must survive untouched.
        let expected: Vec<Vec<u8>> = frames
            .iter()
            .zip(&spans)
            .filter(|(_, span)| !dropped.iter().any(|i| span.contains(i)))
            .map(|(frame, _)| frame.clone())
            .collect();
        assert!(
            expected.len() < frames.len(),
            "the fault must land somewhere"
        );

        let (mut vehicle, gcs) = Loopback::pair();
        let mut gcs = gcs.with_fault(Fault {
            drop_every,
            max_read: 61,
            ..Fault::default()
        });
        send_all(&mut vehicle, &frames);
        let got = receive(&mut gcs, 4096);

        assert!(got.error.is_none());
        assert_eq!(got.frames, expected, "drop every {drop_every}");
        assert_all_pass_checksum(&got.frames);
        assert!(got.stats.crc_errors > 0, "the damage must have been seen");
    }
}

#[test]
fn duplicated_bytes_cost_exactly_the_frames_they_land_inside() {
    let frames = real_frames(300);
    let spans = spans(&frames);
    let total = spans.last().unwrap().end;

    for duplicate_every in [89, 499, 1_009] {
        let doubled = every_nth(total, duplicate_every);
        // The copy lands right after its original. Inside a frame that breaks it; on a frame's
        // first byte the copy is the start of an intact frame, and on its last byte the copy is
        // stray junk after one. The decoder resynchronises past the junk either way.
        let expected: Vec<Vec<u8>> = frames
            .iter()
            .zip(&spans)
            .filter(|(_, span)| !doubled.iter().any(|&i| span.start < i && i < span.end - 1))
            .map(|(frame, _)| frame.clone())
            .collect();
        assert!(
            expected.len() < frames.len(),
            "the fault must land somewhere"
        );

        let (mut vehicle, gcs) = Loopback::pair();
        let mut gcs = gcs.with_fault(Fault {
            duplicate_every,
            max_read: 53,
            ..Fault::default()
        });
        send_all(&mut vehicle, &frames);
        let got = receive(&mut gcs, 4096);

        assert!(got.error.is_none());
        assert_eq!(got.frames, expected, "duplicate every {duplicate_every}");
        assert_all_pass_checksum(&got.frames);
    }
}

#[test]
fn a_repeated_frame_is_delivered_twice_because_dedup_is_the_links_job() {
    let frames = real_frames(300);
    let repeat_write_every = 5;
    let expected: Vec<Vec<u8>> = frames
        .iter()
        .enumerate()
        .flat_map(|(i, frame)| {
            let copies = if (i + 1) % repeat_write_every == 0 {
                2
            } else {
                1
            };
            std::iter::repeat_n(frame.clone(), copies)
        })
        .collect();

    let (mut vehicle, gcs) = Loopback::pair();
    let mut gcs = gcs.with_fault(Fault {
        repeat_write_every,
        ..Fault::default()
    });
    send_all(&mut vehicle, &frames);
    let got = receive(&mut gcs, 4096);

    assert!(got.error.is_none());
    assert_eq!(
        got.frames.len(),
        frames.len() + frames.len() / repeat_write_every
    );
    assert_eq!(got.frames, expected);
    assert_all_pass_checksum(&got.frames);
    assert_eq!(
        got.stats.crc_errors, 0,
        "a repeat is two good frames, not damage"
    );
}

#[test]
fn reordered_frames_arrive_intact_in_the_order_the_wire_delivered_them() {
    let frames = real_frames(300);
    let reorder_write_every = 7;
    assert_ne!(
        frames.len() % reorder_write_every,
        0,
        "the last write must not be held"
    );
    let mut expected = frames.clone();
    // Write n is held and arrives after write n + 1.
    for i in (reorder_write_every - 1..frames.len() - 1).step_by(reorder_write_every) {
        expected.swap(i, i + 1);
    }

    let (mut vehicle, gcs) = Loopback::pair();
    let mut gcs = gcs.with_fault(Fault {
        reorder_write_every,
        max_read: 29,
        ..Fault::default()
    });
    send_all(&mut vehicle, &frames);
    let got = receive(&mut gcs, 4096);

    assert!(got.error.is_none());
    assert_ne!(got.frames, frames, "the fault must reorder something");
    assert_eq!(got.frames, expected);
    assert_all_pass_checksum(&got.frames);
    assert_eq!(got.stats.crc_errors, 0);
}

#[test]
fn a_frame_written_in_two_pieces_is_delivered_once_whatever_the_split() {
    // One real frame and one built here, split at every possible point.
    let mut heartbeat = [0u8; 64];
    let info = DIALECT.info(0).unwrap();
    let n = encode_v2(
        &mut heartbeat,
        7,
        1,
        1,
        0,
        &[0, 0, 0, 0, 2, 3, 81, 4, 3],
        info.crc_extra,
        0,
    )
    .unwrap();
    let frames = [real_frames(1).remove(0), heartbeat[..n].to_vec()];

    for frame in &frames {
        for split in 1..frame.len() {
            for max_read in [0, 1] {
                let (mut vehicle, gcs) = Loopback::pair();
                let mut gcs = gcs.with_fault(Fault {
                    max_read,
                    ..Fault::default()
                });
                let mut decoder = FrameDecoder::new();
                let mut got: Vec<Vec<u8>> = Vec::new();
                let mut buf = [0u8; 512];

                vehicle.write_all(&frame[..split]).unwrap();
                while let n @ 1.. = gcs.read(&mut buf).unwrap() {
                    decoder.push_and_drain(&buf[..n], &DIALECT, |f| got.push(f.raw.to_vec()));
                }
                assert!(got.is_empty(), "half a frame delivered at split {split}");

                vehicle.write_all(&frame[split..]).unwrap();
                while let n @ 1.. = gcs.read(&mut buf).unwrap() {
                    decoder.push_and_drain(&buf[..n], &DIALECT, |f| got.push(f.raw.to_vec()));
                }
                decoder.flush(&DIALECT, |f| got.push(f.raw.to_vec()));
                assert_eq!(
                    got,
                    std::slice::from_ref(frame),
                    "split {split}, max_read {max_read}"
                );
            }
        }
    }
}

#[test]
fn a_mid_frame_disconnect_is_an_error_a_caller_can_tell_from_a_timeout() {
    let frames = real_frames(20);
    let spans = spans(&frames);
    let cut = spans[10].start + 5; // five bytes into the eleventh frame

    let (mut vehicle, gcs) = Loopback::pair();
    let mut gcs = gcs.with_fault(Fault {
        unplug_after: cut,
        ..Fault::default()
    });

    // An idle link: Ok(0), and still open. This is what a timeout looks like.
    let mut buf = [0u8; 64];
    assert_eq!(gcs.read(&mut buf).unwrap(), 0);
    assert!(gcs.is_open());

    send_all(&mut vehicle, &frames);
    let got = receive(&mut gcs, 64);

    // Everything before the cut arrived whole; the frame it cut through was never delivered,
    // not even by the flush.
    assert_eq!(got.frames, frames[..10]);
    assert_all_pass_checksum(&got.frames);

    // The cut itself is an error, not a zero-byte read.
    let error = got.error.expect("the disconnect must surface as an error");
    assert_eq!(error.kind(), io::ErrorKind::BrokenPipe);
    assert!(!matches!(
        error.kind(),
        io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
    ));
    assert!(!gcs.is_open());

    // And it stays one: a caller that retries does not get zero bytes forever.
    for _ in 0..3 {
        let again = gcs.read(&mut buf).expect_err("still unplugged");
        assert_eq!(again.kind(), io::ErrorKind::BrokenPipe);
    }

    // The sender learns of it on its next write, mid-stream.
    let write = vehicle.write_all(&frames[0]).expect_err("the peer is gone");
    assert_eq!(write.kind(), io::ErrorKind::BrokenPipe);
    assert!(!vehicle.is_open());
}

#[test]
fn a_storm_of_every_fault_at_once_never_panics_and_delivers_only_real_frames() {
    let frames = real_frames(500);
    let faults = [
        Fault {
            drop_every: 211,
            duplicate_every: 157,
            max_read: 13,
            repeat_write_every: 11,
            reorder_write_every: 17,
            unplug_after: 0,
        },
        Fault {
            drop_every: 3,
            duplicate_every: 2,
            max_read: 1,
            repeat_write_every: 2,
            reorder_write_every: 3,
            unplug_after: 0,
        },
        Fault {
            drop_every: 1_000,
            duplicate_every: 0,
            max_read: 300,
            repeat_write_every: 0,
            reorder_write_every: 2,
            unplug_after: 9_000,
        },
    ];
    for fault in faults {
        let (mut vehicle, gcs) = Loopback::pair();
        let mut gcs = gcs.with_fault(fault);
        for frame in &frames {
            if vehicle.write_all(frame).is_err() {
                break;
            }
        }
        let got = receive(&mut gcs, 4096);
        assert_all_pass_checksum(&got.frames);
        for frame in &got.frames {
            assert!(
                frames.contains(frame),
                "{fault:?} delivered a frame nobody sent"
            );
        }
        assert_eq!(got.stats.frames, got.frames.len() as u64);
        if fault.unplug_after > 0 {
            assert_eq!(got.error.map(|e| e.kind()), Some(io::ErrorKind::BrokenPipe));
        }
    }
}

// ---------------------------------------------------------------- the network transports
//
// The same faults, where they can happen, to the UDP client, the websocket and NTRIP over real
// sockets on 127.0.0.1: a datagram repeated or overtaken is delivered as the wire delivered it, a
// websocket frame split anywhere is delivered once, and a connection cut mid-frame delivers what
// came before the cut and then does what the C# does about it.

/// Reads from a network transport until `want` bytes have come or it closes.
fn read_network(link: &mut dyn Transport, want: usize) -> Vec<u8> {
    let mut got = Vec::new();
    let mut buf = [0u8; 4096];
    while got.len() < want && link.is_open() {
        let n = link.read(&mut buf).unwrap();
        got.extend_from_slice(&buf[..n]);
    }
    got
}

fn decoded(bytes: &[u8]) -> Vec<Vec<u8>> {
    let mut decoder = FrameDecoder::new();
    let mut frames = Vec::new();
    decoder.push_and_drain(bytes, &DIALECT, |frame| frames.push(frame.raw.to_vec()));
    frames
}

#[test]
fn udp_client_datagrams_repeated_and_overtaken_arrive_as_the_wire_delivered_them() {
    use mp_transport::UdpClientTransport;
    use std::net::UdpSocket;

    let frames = real_frames(200);
    let vehicle = UdpSocket::bind("127.0.0.1:0").unwrap();
    vehicle.set_read_timeout(Some(common::GUARD)).unwrap();
    let port = vehicle.local_addr().unwrap().port();
    let mut client = UdpClientTransport::open("127.0.0.1", port).unwrap();
    client.set_read_timeout(common::GUARD).unwrap();
    client.write_all(b"hello").unwrap();
    let (_, client_address) = vehicle.recv_from(&mut [0u8; 16]).unwrap();

    // Every 11th datagram sent twice, and every 17th swapped with the one after it.
    let mut wire: Vec<&Vec<u8>> = Vec::new();
    let mut i = 0;
    while i < frames.len() {
        if (i + 1) % 17 == 0 && i + 1 < frames.len() {
            wire.push(&frames[i + 1]);
            wire.push(&frames[i]);
            i += 2;
            continue;
        }
        wire.push(&frames[i]);
        if (i + 1) % 11 == 0 {
            wire.push(&frames[i]);
        }
        i += 1;
    }
    // Read as they come: a socket's buffer holds only so many, and UDP drops the rest.
    let mut got = Vec::new();
    let mut buf = [0u8; 4096];
    for datagram in &wire {
        vehicle.send_to(datagram, client_address).unwrap();
        let n = client.read(&mut buf).unwrap();
        got.extend_from_slice(&buf[..n]);
    }
    let delivered = decoded(&got);
    let expected: Vec<Vec<u8>> = wire.iter().map(|d| (*d).clone()).collect();
    assert_eq!(delivered, expected, "no de-duplicating, no re-ordering");
    assert_all_pass_checksum(&delivered);
}

#[test]
fn a_websocket_frame_written_in_two_pieces_is_delivered_once_whatever_the_split() {
    use mp_transport::WebSocketTransport;
    use std::io::Write;

    let frames = real_frames(3);
    let (listener, port) = common::listen();
    let server = common::ws_accept(&listener);
    let mut ws = WebSocketTransport::open(&format!("ws://127.0.0.1:{port}/")).unwrap();
    ws.set_read_timeout(common::GUARD).unwrap();
    let (_, mut peer) = server.join().unwrap();
    let _probe = common::read_client_frame(&mut peer).unwrap();

    for frame in &frames {
        let message = common::server_frame(true, 2, frame);
        for split in 1..message.len() {
            peer.write_all(&message[..split]).unwrap();
            // Whatever has come of the payload so far, which may be nothing.
            let mut buf = [0u8; 512];
            let early = ws.read(&mut buf).unwrap();
            let mut got = buf[..early].to_vec();
            peer.write_all(&message[split..]).unwrap();
            got.extend(read_network(&mut ws, frame.len() - early));
            assert_eq!(&got, frame, "split at {split}");
            assert_eq!(
                decoded(&got),
                std::slice::from_ref(frame),
                "split at {split}"
            );
        }
    }
}

#[test]
fn a_websocket_cut_mid_frame_delivers_what_came_before_and_opens_again() {
    use mp_transport::WebSocketTransport;
    use std::io::Write;

    let frames = real_frames(20);
    let (listener, port) = common::listen();
    let server = common::ws_accept(&listener);
    let mut ws = WebSocketTransport::open(&format!("ws://127.0.0.1:{port}/")).unwrap();
    ws.set_read_timeout(common::GUARD).unwrap();
    let (_, mut peer) = server.join().unwrap();
    let _probe = common::read_client_frame(&mut peer).unwrap();

    // Ten whole messages, then half of the eleventh, then the connection goes.
    let mut wire = Vec::new();
    for frame in &frames[..10] {
        wire.extend(common::server_frame(true, 2, frame));
    }
    let cut = common::server_frame(true, 2, &frames[10]);
    wire.extend_from_slice(&cut[..cut.len() / 2]);
    peer.write_all(&wire).unwrap();
    let before = frames[..10].concat();
    let mut got = read_network(&mut ws, before.len());
    assert!(got.starts_with(&before));

    // The reader reopens (C#: CommsWebSocket.cs:180-184), and the new connection carries on. A
    // read gives what was left of the cut frame's payload, then meets the end of the stream,
    // reopens, and gives nothing.
    let reopen = common::ws_accept(&listener);
    drop(peer);
    let mut buf = [0u8; 512];
    loop {
        let n = ws.read(&mut buf).unwrap();
        if n == 0 {
            break;
        }
        got.extend_from_slice(&buf[..n]);
    }
    let (_, mut again) = reopen.join().unwrap();
    assert!(ws.is_open());
    let _probe = common::read_client_frame(&mut again).unwrap();
    let mut rest = Vec::new();
    for frame in &frames[11..] {
        rest.extend(common::server_frame(true, 2, frame));
    }
    again.write_all(&rest).unwrap();
    got.extend(read_network(&mut ws, frames[11..].concat().len()));

    // The cut frame is never delivered whole; everything else is, in order.
    let delivered = decoded(&got);
    let expected: Vec<Vec<u8>> = frames[..10].iter().chain(&frames[11..]).cloned().collect();
    assert_eq!(delivered, expected);
    assert_all_pass_checksum(&delivered);
}

#[test]
fn an_ntrip_caster_cut_mid_stream_delivers_what_came_before_and_is_tried_on_every_read() {
    use mp_transport::ntrip::RECONNECTS;
    use mp_transport::{NtripOptions, NtripTransport};
    use std::io::Write;

    let (listener, port) = common::listen();
    let caster = common::caster(&listener, b"ICY 200 OK\r\n", false);
    let mut ntrip = NtripTransport::open(
        &format!("ntrip://127.0.0.1:{port}/MOUNT"),
        NtripOptions::default(),
    )
    .unwrap();
    ntrip.set_read_timeout(common::GUARD).unwrap();
    let mut connection = caster.join().unwrap();

    // RTCM, cut in the middle of a message.
    let rtcm: Vec<u8> = [0xD3, 0x00, 0x40].into_iter().chain(0u8..0x40).collect();
    connection.stream.write_all(&rtcm[..30]).unwrap();
    connection
        .stream
        .shutdown(std::net::Shutdown::Both)
        .unwrap();
    let got = read_network(&mut ntrip, 30);
    assert_eq!(got, rtcm[..30]);

    // The end of the stream closes the link, rather than reading nothing for ever.
    let mut buf = [0u8; 64];
    assert_eq!(ntrip.read(&mut buf).unwrap(), 0);
    assert!(!ntrip.is_open());

    // The caster is gone: each read tries it again, and a failed try uses up no retry.
    drop(listener);
    for _ in 0..3 {
        let error = ntrip.read(&mut buf).expect_err("nobody to reconnect to");
        assert_eq!(error.kind(), io::ErrorKind::ConnectionRefused);
        assert_eq!(ntrip.reconnects_left(), RECONNECTS);
    }
}
