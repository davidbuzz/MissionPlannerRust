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

//! The websocket client against a minimal server on 127.0.0.1 that checks the handshake, answers
//! with the accept value its own SHA-1 works out, and unmasks what the client sends - carrying
//! real MAVLink frames from a recorded flight both ways.
//!
//! The line-for-line comparison with what the C# did is `csharp_goldens.rs`.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::cast_possible_truncation
)]

mod common;

use std::collections::BTreeSet;
use std::io::{self, Write};

use common::{
    decode, listen, read_client_frame, read_to_end, real_frames, server_frame, ws_accept,
    ws_accept_answering,
};
use mp_transport::{OpenError, Transport, WebSocketTransport};

/// Reads until `want` bytes have come, or the link closes.
fn read_bytes(ws: &mut WebSocketTransport, want: usize) -> Vec<u8> {
    let mut got = Vec::new();
    let mut buf = [0u8; 4096];
    while got.len() < want && ws.is_open() {
        let n = ws.read(&mut buf).unwrap();
        got.extend_from_slice(&buf[..n]);
    }
    got
}

fn connected() -> (
    std::net::TcpListener,
    WebSocketTransport,
    std::net::TcpStream,
) {
    let (listener, port) = listen();
    let server = ws_accept(&listener);
    let mut ws = WebSocketTransport::open(&format!("ws://127.0.0.1:{port}/mavlink")).unwrap();
    ws.set_read_timeout(common::GUARD).unwrap();
    let (request, mut peer) = server.join().unwrap();
    assert!(
        request.starts_with("GET /mavlink HTTP/1.1\r\n"),
        "{request}"
    );
    // The engine.io probe the C# sends on every open, whatever the server.
    let probe = read_client_frame(&mut peer).unwrap();
    assert_eq!(
        (probe.opcode, probe.payload.as_slice()),
        (1, &b"2probe"[..])
    );
    (listener, ws, peer)
}

#[test]
fn real_frames_arrive_whole_however_the_server_frames_them() {
    let frames = real_frames(400);
    let (_listener, mut ws, mut peer) = connected();

    // One frame per message, as a MAVLink bridge sends them.
    for frame in &frames[..100] {
        peer.write_all(&server_frame(true, 2, frame)).unwrap();
    }
    // Several messages in one TCP write.
    let mut batch = Vec::new();
    for frame in &frames[100..150] {
        batch.extend(server_frame(true, 2, frame));
    }
    peer.write_all(&batch).unwrap();
    // One message in fragments, split inside frames.
    let joined: Vec<u8> = frames[150..200].concat();
    let pieces: Vec<&[u8]> = joined.chunks(37).collect();
    for (i, piece) in pieces.iter().enumerate() {
        let opcode = if i == 0 { 2 } else { 0 };
        peer.write_all(&server_frame(i + 1 == pieces.len(), opcode, piece))
            .unwrap();
    }
    // A message long enough for a 16-bit length, and one for a 64-bit length.
    peer.write_all(&server_frame(true, 2, &frames[200..220].concat()))
        .unwrap();
    let mut long = Vec::new();
    while long.len() <= 0xFFFF {
        long.extend(frames[220..400].concat());
    }
    peer.write_all(&server_frame(true, 2, &long)).unwrap();

    let expected: Vec<u8> = [frames[..220].concat(), long].concat();
    let got = read_bytes(&mut ws, expected.len());
    assert_eq!(got.len(), expected.len());
    assert_eq!(decode(&got), decode(&expected));
    assert_eq!(got, expected);
}

#[test]
fn what_the_client_writes_goes_as_masked_binary_messages() {
    let frames = real_frames(300);
    let (_listener, mut ws, mut peer) = connected();
    for frame in &frames {
        ws.write_all(frame).unwrap();
    }
    let mut masks = BTreeSet::new();
    for frame in &frames {
        let sent = read_client_frame(&mut peer).unwrap();
        assert!(sent.fin);
        assert_eq!(sent.rsv, 0);
        assert_eq!(sent.opcode, 2, "binary");
        assert!(sent.masked, "a client must mask (RFC 6455 section 5.3)");
        assert_eq!(&sent.payload, frame);
        assert_ne!(&sent.wire, frame, "masked on the wire");
        masks.insert(sent.mask);
    }
    assert_eq!(masks.len(), frames.len(), "a fresh mask for every frame");

    // Long enough for the 16- and 64-bit lengths.
    for size in [126, 0xFFFF, 0x1_0000, 70_000] {
        let payload: Vec<u8> = (0..size).map(|i| (i % 251) as u8).collect();
        ws.write_all(&payload).unwrap();
        let sent = read_client_frame(&mut peer).unwrap();
        assert_eq!(sent.payload, payload, "{size}");
    }
}

#[test]
fn a_ping_is_answered_and_a_pong_is_not() {
    let (_listener, mut ws, mut peer) = connected();
    // In one write, so one read of the client takes both.
    let both = [
        server_frame(true, 10, b"unasked"),
        server_frame(true, 9, b"are you there"),
    ]
    .concat();
    peer.write_all(&both).unwrap();
    let mut buf = [0u8; 16];
    assert_eq!(ws.read(&mut buf).unwrap(), 0);
    let pong = read_client_frame(&mut peer).unwrap();
    assert_eq!(
        (pong.opcode, pong.payload.as_slice()),
        (10, &b"are you there"[..])
    );
    // A ping between the fragments of a message does not interrupt it.
    peer.write_all(&server_frame(false, 2, &[1, 2])).unwrap();
    peer.write_all(&server_frame(true, 9, b"mid")).unwrap();
    peer.write_all(&server_frame(true, 0, &[3, 4])).unwrap();
    assert_eq!(read_bytes(&mut ws, 4), [1, 2, 3, 4]);
    let pong = read_client_frame(&mut peer).unwrap();
    assert_eq!(pong.payload, b"mid");
}

#[test]
fn a_text_message_longer_than_the_csharps_buffer_is_looked_at_in_its_pieces() {
    // RunReader receives into 8 KiB, so the second piece of a longer text message is read as a
    // message of its own; one that starts with `3` is the probe's answer.
    let (_listener, mut ws, mut peer) = connected();
    let mut text = vec![b'x'; 8 * 1024];
    text.extend_from_slice(b"3probe");
    peer.write_all(&server_frame(true, 1, &text)).unwrap();
    // A byte of stream after it: once that has been read, the text has all been looked at.
    peer.write_all(&server_frame(true, 2, &[0xAA])).unwrap();
    assert_eq!(read_bytes(&mut ws, 1), [0xAA]);
    let first = read_client_frame(&mut peer).unwrap();
    let second = read_client_frame(&mut peer).unwrap();
    assert_eq!(first.payload, b"40/MAVControl,");
    assert_eq!(second.payload, b"5");
}

#[test]
fn a_handshake_with_the_wrong_answer_is_refused() {
    let (listener, port) = listen();
    let server = ws_accept_answering(&listener, Some("dGhpcyBpcyBub3QgaXQ="));
    let error =
        WebSocketTransport::open(&format!("ws://127.0.0.1:{port}/")).expect_err("bad accept");
    drop(server.join().unwrap());
    let OpenError::Io { source, .. } = error else {
        panic!("{error}")
    };
    assert_eq!(source.kind(), io::ErrorKind::ConnectionRefused);

    // A server that is not a websocket server at all.
    let listener2 = listener.try_clone().unwrap();
    let plain = std::thread::spawn(move || {
        let (mut stream, _) = listener2.accept().unwrap();
        common::read_until(&mut stream, b"\r\n\r\n");
        stream
            .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n")
            .unwrap();
        stream
    });
    let error = WebSocketTransport::open(&format!("ws://127.0.0.1:{port}/")).expect_err("200");
    drop(plain.join().unwrap());
    assert!(error.to_string().contains("not 101"), "{error}");
}

#[test]
fn wss_needs_tls_which_is_not_ported() {
    for url in ["wss://127.0.0.1:1/x", "WSS://example.invalid/"] {
        let error = WebSocketTransport::open(url).expect_err(url);
        assert!(
            matches!(error, OpenError::Unsupported(what) if what.contains("wss")),
            "{url}: {error}"
        );
    }
    assert!(matches!(
        mp_transport::open("wss://127.0.0.1:1/x"),
        Err(OpenError::Unsupported(_))
    ));
}

#[test]
fn a_masked_frame_from_the_server_breaks_the_protocol_and_the_link_reopens() {
    let (listener, mut ws, mut peer) = connected();
    let reopen = ws_accept(&listener);
    // Masked, which a server must never do.
    peer.write_all(&[0x82, 0x81, 1, 2, 3, 4, 5]).unwrap();
    let mut buf = [0u8; 16];
    assert_eq!(ws.read(&mut buf).unwrap(), 0);
    let (request, mut again) = reopen.join().unwrap();
    assert!(request.starts_with("GET /mavlink "));
    assert!(ws.is_open());
    let probe = read_client_frame(&mut again).unwrap();
    assert_eq!(probe.payload, b"2probe");
    // And the new connection carries the stream.
    again.write_all(&server_frame(true, 2, &[0xFD, 1])).unwrap();
    assert_eq!(read_bytes(&mut ws, 2), [0xFD, 1]);
}

#[test]
fn a_server_that_is_gone_for_good_closes_the_link() {
    let (listener, mut ws, peer) = connected();
    drop(listener);
    drop(peer);
    let mut buf = [0u8; 16];
    assert_eq!(ws.read(&mut buf).unwrap(), 0);
    assert!(!ws.is_open(), "the reopen was refused");
    assert_eq!(ws.read(&mut buf).unwrap(), 0);
    // Writes to a closed link are not errors, as the C# logs and drops them.
    ws.write_all(&[0xFD, 0, 0]).unwrap();
}

#[test]
fn close_stops_reconnecting_and_sends_no_close_frame() {
    let (_listener, mut ws, mut peer) = connected();
    ws.close();
    assert!(!ws.is_open());
    // ClientWebSocket.Dispose aborts: the server sees the connection end, and nothing before it.
    assert!(read_to_end(&mut peer).is_empty());
    let mut buf = [0u8; 16];
    assert_eq!(ws.read(&mut buf).unwrap(), 0);
    assert!(!ws.is_open());
}

#[test]
fn a_link_url_opens_the_websocket() {
    let (listener, port) = listen();
    let server = ws_accept(&listener);
    let mut link = mp_transport::open(&format!("ws://127.0.0.1:{port}/a/b?c=d")).unwrap();
    let (request, mut peer) = server.join().unwrap();
    assert!(
        request.starts_with("GET /a/b?c=d HTTP/1.1\r\n"),
        "{request}"
    );
    assert_eq!(link.description(), format!("ws://127.0.0.1:{port}/a/b?c=d"));
    let _ = read_client_frame(&mut peer).unwrap();
    link.write_all(&[0xFD, 0x00]).unwrap();
    assert_eq!(read_client_frame(&mut peer).unwrap().payload, [0xFD, 0x00]);
}
