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

//! Peers on 127.0.0.1 for the network transports' tests: a websocket server that checks the
//! handshake and unmasks what it is sent, an NTRIP caster, and the goldens the C# left in
//! `testdata/comms/golden/`.
//!
//! Every wait here is a blocking socket read or accept, with a generous timeout that only a broken
//! transport would reach; nothing sleeps.

#![allow(
    dead_code,
    unreachable_pub,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::cast_possible_truncation
)]

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::thread::JoinHandle;
use std::time::Duration;

use mp_mavlink::FrameDecoder;
use mp_mavlink_dialects::all::DIALECT;

/// How long a peer waits for the transport under test before the test fails instead of hanging.
pub const GUARD: Duration = Duration::from_secs(10);

/// A golden the C# wrote, from `testdata/comms/golden/`.
pub fn golden(name: &str) -> String {
    let path = format!(
        "{}/../../testdata/comms/golden/{name}",
        env!("CARGO_MANIFEST_DIR")
    );
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

/// A golden's lines as `(case, result)`, split at the first tab.
pub fn golden_lines(name: &str) -> Vec<(String, String)> {
    golden(name)
        .lines()
        .map(|line| {
            let (case, result) = line.split_once('\t').unwrap_or((line, ""));
            (case.to_owned(), result.to_owned())
        })
        .collect()
}

/// Bytes written as MpComms.cs writes them: printable ASCII as itself, `\\`, `\r`, `\n`, and
/// `\xHH` for the rest.
pub fn escape(bytes: &[u8]) -> String {
    let mut out = String::new();
    for &byte in bytes {
        match byte {
            b'\\' => out.push_str("\\\\"),
            b'\r' => out.push_str("\\r"),
            b'\n' => out.push_str("\\n"),
            0x20..=0x7E => out.push(char::from(byte)),
            _ => out.push_str(&format!("\\x{byte:02X}")),
        }
    }
    out
}

/// `count` consecutive real frames from the autotest flight, mid-flight.
pub fn real_frames(count: usize) -> Vec<Vec<u8>> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink/autotest.tlog"
    );
    let data = std::fs::read(path).unwrap();
    let mut decoder = FrameDecoder::new();
    let mut frames = Vec::new();
    decoder.push_and_drain(&data, &DIALECT, |frame| {
        if frames.len() < 2_000 + count {
            frames.push(frame.raw.to_vec());
        }
    });
    frames.split_off(2_000)
}

/// Decodes a byte stream into its MAVLink frames.
pub fn decode(bytes: &[u8]) -> Vec<Vec<u8>> {
    let mut decoder = FrameDecoder::new();
    let mut frames = Vec::new();
    decoder.push_and_drain(bytes, &DIALECT, |frame| frames.push(frame.raw.to_vec()));
    frames
}

pub fn listen() -> (TcpListener, u16) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    (listener, port)
}

/// Reads until `end` has been read, one byte at a time so nothing after it is taken.
pub fn read_until(stream: &mut TcpStream, end: &[u8]) -> Vec<u8> {
    stream.set_read_timeout(Some(GUARD)).unwrap();
    let mut got = Vec::new();
    let mut byte = [0u8; 1];
    while !got.ends_with(end) {
        match stream.read(&mut byte) {
            Ok(1) => got.push(byte[0]),
            _ => break,
        }
    }
    got
}

pub fn read_to_end(stream: &mut TcpStream) -> Vec<u8> {
    stream.set_read_timeout(Some(GUARD)).unwrap();
    let mut got = Vec::new();
    let _ = stream.read_to_end(&mut got);
    got
}

// ---------------------------------------------------------------- NTRIP caster

/// One connection to a mock caster: the request it read, and the socket, to go on with.
pub struct CasterConnection {
    pub request: Vec<u8>,
    pub stream: TcpStream,
}

/// Accepts one connection on `listener`, reads the request, and answers `answer`; with `hang_up`,
/// the caster then closes its sending side, as a caster does after a source table.
pub fn caster(
    listener: &TcpListener,
    answer: &'static [u8],
    hang_up: bool,
) -> JoinHandle<CasterConnection> {
    let listener = listener.try_clone().unwrap();
    std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        stream.set_nodelay(true).unwrap();
        let request = read_until(&mut stream, b"\r\n\r\n");
        stream.write_all(answer).unwrap();
        if hang_up {
            stream.shutdown(std::net::Shutdown::Write).unwrap();
        }
        CasterConnection { request, stream }
    })
}

/// Reads the GGA lines a caster was sent until the client hangs up.
pub fn sentences(stream: &mut TcpStream) -> Vec<String> {
    let text = read_to_end(stream);
    String::from_utf8(text)
        .unwrap()
        .split_inclusive("\r\n")
        .map(ToOwned::to_owned)
        .collect()
}

// ---------------------------------------------------------------- websocket server

pub const ACCEPT_GUID: &str = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

/// Accepts one connection, checks it is a websocket upgrade, and agrees to it. Gives back the
/// request (the key written `{key}`) and the socket.
pub fn ws_accept(listener: &TcpListener) -> JoinHandle<(String, TcpStream)> {
    ws_accept_answering(listener, None)
}

/// As [`ws_accept`], answering with `bad_accept` instead of the right `Sec-WebSocket-Accept`.
pub fn ws_accept_answering(
    listener: &TcpListener,
    bad_accept: Option<&'static str>,
) -> JoinHandle<(String, TcpStream)> {
    let listener = listener.try_clone().unwrap();
    std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        // Each write of the server's goes out at once, so a test that writes and then has the
        // client read finds it there.
        stream.set_nodelay(true).unwrap();
        let request = String::from_utf8(read_until(&mut stream, b"\r\n\r\n")).unwrap();
        let header = |name: &str| {
            request
                .split("\r\n")
                .find_map(|line| {
                    let (field, value) = line.split_once(':')?;
                    field
                        .eq_ignore_ascii_case(name)
                        .then(|| value.trim().to_owned())
                })
                .unwrap_or_else(|| panic!("no {name} header in {request:?}"))
        };
        assert!(request.starts_with("GET "), "{request:?}");
        assert!(request.contains(" HTTP/1.1\r\n"), "{request:?}");
        assert_eq!(header("Upgrade"), "websocket");
        assert_eq!(header("Connection"), "Upgrade");
        assert_eq!(header("Sec-WebSocket-Version"), "13");
        let key = header("Sec-WebSocket-Key");
        // RFC 6455 section 4.1: sixteen random bytes, base64.
        assert_eq!(key.len(), 24, "{key:?}");
        assert!(key.ends_with("=="), "{key:?}");
        let accept = bad_accept.map_or_else(|| accept_for(&key), ToOwned::to_owned);
        let response = format!(
            "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n\
             Sec-WebSocket-Accept: {accept}\r\n\r\n"
        );
        stream.write_all(response.as_bytes()).unwrap();
        (request.replace(&key, "{key}"), stream)
    })
}

/// `base64(SHA-1(key + GUID))`, worked out with the test's own SHA-1 so the transport's is not
/// checking itself.
pub fn accept_for(key: &str) -> String {
    let digest = sha1_reference(format!("{key}{ACCEPT_GUID}").as_bytes());
    base64_reference(&digest)
}

/// A frame as a server sends it: never masked.
pub fn server_frame(fin: bool, opcode: u8, payload: &[u8]) -> Vec<u8> {
    let mut frame = vec![if fin { 0x80 } else { 0 } | opcode];
    match payload.len() {
        len @ 0..=125 => frame.push(len as u8),
        len @ 126..=0xFFFF => {
            frame.push(126);
            frame.extend_from_slice(&(len as u16).to_be_bytes());
        }
        len => {
            frame.push(127);
            frame.extend_from_slice(&(len as u64).to_be_bytes());
        }
    }
    frame.extend_from_slice(payload);
    frame
}

/// A frame the client sent, unmasked.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClientFrame {
    pub fin: bool,
    pub rsv: u8,
    pub opcode: u8,
    pub masked: bool,
    pub mask: [u8; 4],
    pub payload: Vec<u8>,
    /// The payload as it was on the wire, masked.
    pub wire: Vec<u8>,
}

impl ClientFrame {
    /// As MpComms.cs describes a frame.
    pub fn describe(&self) -> String {
        format!(
            "fin {} rsv {} opcode {} masked {} payload {}",
            u8::from(self.fin),
            self.rsv,
            self.opcode,
            u8::from(self.masked),
            escape(&self.payload)
        )
    }
}

/// Reads one frame the client sent; `None` at the end of the stream.
pub fn read_client_frame(stream: &mut TcpStream) -> Option<ClientFrame> {
    stream.set_read_timeout(Some(GUARD)).unwrap();
    let mut head = [0u8; 2];
    stream.read_exact(&mut head).ok()?;
    let mut length = u64::from(head[1] & 0x7F);
    if length == 126 {
        let mut ext = [0u8; 2];
        stream.read_exact(&mut ext).unwrap();
        length = u64::from(u16::from_be_bytes(ext));
    } else if length == 127 {
        let mut ext = [0u8; 8];
        stream.read_exact(&mut ext).unwrap();
        length = u64::from_be_bytes(ext);
    }
    let masked = head[1] & 0x80 != 0;
    let mut mask = [0u8; 4];
    if masked {
        stream.read_exact(&mut mask).unwrap();
    }
    let mut wire = vec![0u8; usize::try_from(length).unwrap()];
    stream.read_exact(&mut wire).unwrap();
    let payload = wire
        .iter()
        .enumerate()
        .map(|(i, byte)| if masked { byte ^ mask[i % 4] } else { *byte })
        .collect();
    Some(ClientFrame {
        fin: head[0] & 0x80 != 0,
        rsv: (head[0] >> 4) & 7,
        opcode: head[0] & 0x0F,
        masked,
        mask,
        payload,
        wire,
    })
}

// ---------------------------------------------------------------- reference codecs

/// SHA-1 written out again from FIPS 180-4, independently of the crate's.
fn sha1_reference(message: &[u8]) -> [u8; 20] {
    let mut h = [
        0x6745_2301u32,
        0xEFCD_AB89,
        0x98BA_DCFE,
        0x1032_5476,
        0xC3D2_E1F0,
    ];
    let mut data = message.to_vec();
    let bit_length = (message.len() as u64) * 8;
    data.push(0x80);
    while data.len() % 64 != 56 {
        data.push(0);
    }
    data.extend_from_slice(&bit_length.to_be_bytes());
    for block in data.chunks(64) {
        let mut w = [0u32; 80];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([
                block[4 * i],
                block[4 * i + 1],
                block[4 * i + 2],
                block[4 * i + 3],
            ]);
        }
        for i in 16..80 {
            w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
        }
        let (mut a, mut b, mut c, mut d, mut e) = (h[0], h[1], h[2], h[3], h[4]);
        for (i, word) in w.iter().enumerate() {
            let (f, k) = if i < 20 {
                ((b & c) | ((!b) & d), 0x5A82_7999)
            } else if i < 40 {
                (b ^ c ^ d, 0x6ED9_EBA1)
            } else if i < 60 {
                ((b & c) | (b & d) | (c & d), 0x8F1B_BCDC)
            } else {
                (b ^ c ^ d, 0xCA62_C1D6)
            };
            let temp = a
                .rotate_left(5)
                .wrapping_add(f)
                .wrapping_add(e)
                .wrapping_add(k)
                .wrapping_add(*word);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = temp;
        }
        h[0] = h[0].wrapping_add(a);
        h[1] = h[1].wrapping_add(b);
        h[2] = h[2].wrapping_add(c);
        h[3] = h[3].wrapping_add(d);
        h[4] = h[4].wrapping_add(e);
    }
    let mut out = [0u8; 20];
    for (i, word) in h.iter().enumerate() {
        out[4 * i..4 * i + 4].copy_from_slice(&word.to_be_bytes());
    }
    out
}

pub fn base64_reference(data: &[u8]) -> String {
    let table = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::new();
    for chunk in data.chunks(3) {
        let b = [
            chunk[0],
            *chunk.get(1).unwrap_or(&0),
            *chunk.get(2).unwrap_or(&0),
        ];
        out.push(table[usize::from(b[0] >> 2)] as char);
        out.push(table[usize::from(((b[0] & 3) << 4) | (b[1] >> 4))] as char);
        out.push(if chunk.len() > 1 {
            table[usize::from(((b[1] & 15) << 2) | (b[2] >> 6))] as char
        } else {
            '='
        });
        out.push(if chunk.len() > 2 {
            table[usize::from(b[2] & 63)] as char
        } else {
            '='
        });
    }
    out
}
