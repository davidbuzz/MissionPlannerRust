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

//! The three network transports held to what Mission Planner's own C# did.
//!
//! `tools/csharp-reference/regen-comms.sh` builds `CommsNTRIP.cs`, `CommsWebSocket.cs` and
//! `CommsUDPSerialConnect.cs` straight from the reference tree, runs them under mono against peers
//! on 127.0.0.1, and writes down what crossed the wire in `testdata/comms/golden/`. Each test here
//! plays the same peer to the Rust and checks it sees the same thing. Where the Rust does
//! something else on purpose, the test says so and checks the Rust's behaviour instead.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

mod common;

use mp_os::Lock as _;
use std::io::{self, Write};
use std::net::{IpAddr, UdpSocket};
use std::sync::{Arc, Mutex};
use web_time::{Duration, SystemTime, UNIX_EPOCH};

use common::{
    caster, escape, golden_lines, listen, read_client_frame, read_to_end, sentences, server_frame,
    ws_accept,
};
use mp_transport::ntrip::{Clock, gga_sentence};
use mp_transport::udp_client::is_in_range;
use mp_transport::{
    NtripOptions, NtripTransport, OpenError, Transport, UdpClientTransport, WebSocketTransport,
};

/// What an open failed with, as the C#'s exception message would read.
fn message(error: &OpenError) -> String {
    match error {
        OpenError::Io { source, .. } => source.to_string(),
        other => other.to_string(),
    }
}

fn leak(text: String) -> &'static [u8] {
    Box::leak(text.into_bytes().into_boxed_slice())
}

/// A port nothing listens on: a transport that got past the URL and tried it would be refused,
/// and fail with something other than the URL's error.
fn closed_port() -> u16 {
    let (listener, port) = listen();
    drop(listener);
    port
}

// ---------------------------------------------------------------- NTRIP

#[test]
fn every_ntrip_request_is_the_bytes_the_csharp_sent() {
    let mut checked = 0;
    for (case, expected) in golden_lines("ntrip-requests.txt") {
        let Some(rest) = case.strip_prefix("request ") else {
            continue;
        };
        let (version, template) = rest.split_once(' ').unwrap();
        let options = NtripOptions {
            ntrip_v1: version == "v1",
            ..NtripOptions::default()
        };

        if let Some(thrown) = expected.strip_prefix("error ") {
            // `UriFormatException Invalid URI: ...`: the type, then the message.
            let (_, thrown) = thrown.split_once(' ').unwrap();
            let url = template.replace("{port}", &closed_port().to_string());
            let error = NtripTransport::open(&url, options).expect_err(&case);
            assert_eq!(escape(message(&error).as_bytes()), thrown, "{case}");
        } else {
            let (listener, port) = listen();
            let peer = caster(&listener, b"ICY 200 OK\r\n", false);
            let url = template.replace("{port}", &port.to_string());
            let transport = NtripTransport::open(&url, options)
                .unwrap_or_else(|e| panic!("{case}: {}", message(&e)));
            let request = peer.join().unwrap().request;
            let request = escape(&request).replace(&format!(":{port}"), ":{port}");
            assert_eq!(request, expected, "{case}");
            drop(transport);
        }
        checked += 1;
    }
    assert_eq!(checked, 20, "every request case in the golden");
}

/// A source table after the one mount point the golden's caster has.
const TABLE: &str = "STR;MOUNT;Canberra;RTCM 3.2;1005(10),1077(1);2;GPS+GLO;SNIP;AUS;-35.36;149.17;1;0;\
                     sNTRIP;none;B;N;9600;\r\nENDSOURCETABLE\r\n";

#[test]
fn what_open_does_with_the_casters_answer_is_what_the_csharp_did() {
    let mut checked = 0;
    for (case, expected) in golden_lines("ntrip-requests.txt") {
        let Some(status) = case.strip_prefix("response ") else {
            continue;
        };
        let table = status.contains("SOURCETABLE");
        let (listener, port) = listen();
        let answer = format!("{status}\r\n{}", if table { TABLE } else { "" });
        let peer = caster(&listener, leak(answer), table);
        let result = NtripTransport::open(
            &format!("ntrip://127.0.0.1:{port}/MOUNT"),
            NtripOptions::default(),
        );
        match expected.strip_prefix("error Exception ") {
            None => {
                assert_eq!(expected, "ok", "{case}");
                assert!(result.is_ok(), "{case}: {:?}", result.err());
            }
            Some(thrown) => {
                let error = result.expect_err(&case);
                assert_eq!(escape(message(&error).as_bytes()), thrown, "{case}");
            }
        }
        drop(peer.join().unwrap());
        checked += 1;
    }
    assert_eq!(checked, 6, "every response case in the golden");
}

/// The moment a golden sentence was stamped with, on the first day of the epoch: only the time of
/// day is in the sentence.
fn stamped(sentence: &str) -> SystemTime {
    let time = sentence.split(',').nth(1).unwrap();
    let field = |range: std::ops::Range<usize>| time[range].parse::<u64>().unwrap();
    let seconds = field(0..2) * 3600 + field(2..4) * 60 + field(4..6);
    UNIX_EPOCH + Duration::from_secs(seconds) + Duration::from_millis(field(7..9) * 10)
}

fn position(case: &str) -> (f64, f64, f64) {
    let values: Vec<f64> = case
        .split_whitespace()
        .filter_map(|word| word.parse().ok())
        .collect();
    (values[0], values[1], values[2])
}

#[test]
fn every_gga_sentence_is_the_csharps_to_the_checksum() {
    let mut checked = 0;
    for (case, expected) in golden_lines("ntrip-gga.txt") {
        if case.starts_with("cadence") || case.starts_with("open") {
            continue;
        }
        let (lat, lng, alt) = position(&case);
        let expected = expected.strip_suffix("\\r\\n").unwrap();
        assert_eq!(
            gga_sentence(lat, lng, alt, stamped(expected)),
            expected,
            "{case}"
        );
        checked += 1;
    }
    assert_eq!(checked, 13, "every position in the golden");
}

/// A clock a test moves by hand.
fn hand_clock(start: SystemTime) -> (Arc<Mutex<SystemTime>>, Clock) {
    let now = Arc::new(Mutex::new(start));
    let read = Arc::clone(&now);
    (now, Box::new(move || *read.os_lock().unwrap()))
}

#[test]
fn open_sends_the_first_gga_straight_after_the_casters_answer() {
    let golden = golden_lines("ntrip-gga.txt");
    let (case, expected) = golden
        .iter()
        .find(|(case, _)| case.starts_with("open "))
        .unwrap();
    let (lat, lng, alt) = position(case);
    let (_, clock) = hand_clock(stamped(expected));

    let (listener, port) = listen();
    let peer = caster(&listener, b"ICY 200 OK\r\n", false);
    let transport = NtripTransport::open_with_clock(
        &format!("ntrip://127.0.0.1:{port}/MOUNT"),
        NtripOptions {
            lat,
            lng,
            alt,
            ntrip_v1: false,
        },
        clock,
    )
    .unwrap();
    drop(transport);
    let mut connection = peer.join().unwrap();
    let sent = sentences(&mut connection.stream);
    assert_eq!(sent.len(), 1, "{sent:?}");
    assert_eq!(escape(sent[0].as_bytes()), *expected);
}

#[test]
fn the_gga_cadence_is_the_csharps() {
    // The golden's steps, in order: the gate open, at once again, 29 s on, 31 s on, and a
    // position of 0,0 with the gate open.
    let golden: Vec<(String, String)> = golden_lines("ntrip-gga.txt")
        .into_iter()
        .filter(|(case, _)| case.starts_with("cadence"))
        .collect();
    let steps: Vec<&str> = golden.iter().map(|(case, _)| case.as_str()).collect();
    assert_eq!(
        steps,
        [
            "cadence first",
            "cadence again",
            "cadence 29s",
            "cadence 31s",
            "cadence zero"
        ]
    );

    let start = UNIX_EPOCH + Duration::from_secs(12 * 3600);
    // Each step at its own moment, so the sentence's time says which step sent it.
    let at = [
        start,
        start + Duration::from_millis(10),
        start + Duration::from_secs(29),
        start + Duration::from_secs(31),
        start + Duration::from_secs(100),
    ];
    let (now, clock) = hand_clock(at[0]);
    let (listener, port) = listen();
    let peer = caster(&listener, b"ICY 200 OK\r\n", false);
    // Open sends the first, as the C#'s first step did.
    let mut transport = NtripTransport::open_with_clock(
        &format!("ntrip://127.0.0.1:{port}/MOUNT"),
        NtripOptions {
            lat: 1.0,
            lng: 1.0,
            alt: 1.0,
            ntrip_v1: false,
        },
        clock,
    )
    .unwrap();
    // A read is what sends it after that, as BytesToRead and Read do in the C#. The caster sends
    // nothing, so each read ends at the transport's own timeout.
    transport
        .set_read_timeout(Duration::from_millis(1))
        .unwrap();
    let mut buf = [0u8; 64];
    for (step, moment) in at.iter().enumerate().skip(1) {
        *now.os_lock().unwrap() = *moment;
        if step == 4 {
            transport.set_position(0.0, 0.0, 1.0);
        }
        assert_eq!(transport.read(&mut buf).unwrap(), 0);
    }
    drop(transport);
    let mut connection = peer.join().unwrap();
    let sent: Vec<SystemTime> = sentences(&mut connection.stream)
        .iter()
        .map(|sentence| stamped(sentence))
        .collect();
    let observed: Vec<&str> = at
        .iter()
        .map(|moment| {
            if sent.contains(moment) {
                "sent"
            } else {
                "none"
            }
        })
        .collect();
    let expected: Vec<&str> = golden.iter().map(|(_, result)| result.as_str()).collect();
    assert_eq!(observed, expected);
}

#[test]
fn a_caster_that_hangs_up_is_reconnected_three_times_and_then_not() {
    // The golden's rounds: after each hang-up the read that reconnects throws "The ntrip is
    // closed" anyway, the next read reads, and the fourth hang-up is not reconnected.
    //
    // The C# only notices the hang-up once a write fails (golden: `read after hangup` is 0 with
    // IsOpen still True, and it takes the second write to make it False). Here the end of the
    // stream is noticed by the read that meets it, so those lines are not the Rust's; the rounds'
    // outcomes are.
    let golden = golden_lines("ntrip-reconnect.txt");
    let result = |case: &str| {
        golden
            .iter()
            .find(|(c, _)| c == case)
            .map(|(_, r)| r.clone())
            .unwrap()
    };
    assert_eq!(result("open"), "accepted 1 isopen True");

    let (listener, port) = listen();
    let peer = caster(&listener, b"ICY 200 OK\r\n", false);
    let mut transport = NtripTransport::open(
        &format!("ntrip://127.0.0.1:{port}/MOUNT"),
        NtripOptions::default(),
    )
    .unwrap();
    assert!(transport.is_open());
    let mut current = peer.join().unwrap().stream;
    let mut buf = [0u8; 16];
    let mut listener = Some(listener);

    for round in 1..=4 {
        current.shutdown(std::net::Shutdown::Both).unwrap();
        // The hang-up, met by a read.
        assert_eq!(transport.read(&mut buf).unwrap(), 0, "round {round}");
        assert!(!transport.is_open(), "round {round}");

        let reconnects = result(&format!("round {round} reconnected"));
        let peer = if reconnects.starts_with('1') {
            Some(caster(listener.as_ref().unwrap(), b"ICY 200 OK\r\n", false))
        } else {
            // No reconnect is due: nothing listens, so trying one would fail differently.
            listener = None;
            None
        };
        let error = transport
            .read(&mut buf)
            .expect_err("the read that reconnects throws");
        assert_eq!(
            format!("error {error}"),
            result(&format!("round {round} read")),
            "round {round}"
        );
        match peer {
            Some(peer) => {
                assert_eq!(reconnects, "1 isopen True");
                assert!(transport.is_open(), "round {round}");
                current = peer.join().unwrap().stream;
                current.write_all(&[0xD3, 0x00, 0x13]).unwrap();
                let n = transport.read(&mut buf).unwrap();
                assert_eq!(
                    escape(&buf[..n]),
                    result(&format!("round {round} read again"))
                );
            }
            None => {
                assert_eq!(reconnects, "0 isopen False");
                assert!(!transport.is_open());
                assert_eq!(transport.reconnects_left(), 0);
            }
        }
    }
    drop(listener);
}

// ---------------------------------------------------------------- websocket

/// The websocket conversation of ws.txt, played to the Rust by the same server, written down the
/// same way.
#[test]
fn the_websocket_conversation_is_the_csharps_line_for_line() {
    let expected: Vec<String> = golden_lines("ws.txt")
        .into_iter()
        .map(|(case, result)| format!("{case}\t{result}"))
        .collect();
    let mut lines = Vec::new();
    let (listener, port) = listen();
    let url = format!("ws://LocalHost:{port}/mav/./link?x=1");
    let port_free = |text: String| text.replace(&format!(":{port}"), ":{port}");

    let server = ws_accept(&listener);
    let mut ws = WebSocketTransport::open(&url).unwrap();
    // Reads end as soon as there is something; this only bounds a broken transport.
    ws.set_read_timeout(common::GUARD).unwrap();
    let (request, mut peer) = server.join().unwrap();
    lines.push(format!(
        "handshake\t{}",
        port_free(escape(request.as_bytes()))
    ));
    lines.push(format!("open\tisopen {}", title(ws.is_open())));
    let probe = read_client_frame(&mut peer).unwrap();
    lines.push(format!("client sends\t{}", probe.describe()));

    let mut buf = [0u8; 64];
    peer.write_all(&server_frame(true, 2, &[0xFD, 0x01, 0x02, 0x03]))
        .unwrap();
    let n = read_at_least(&mut ws, &mut buf, 4);
    lines.push(format!(
        "server binary FD010203\tread {}",
        escape(&buf[..n])
    ));

    peer.write_all(&server_frame(false, 2, &[0xFD, 0x05]))
        .unwrap();
    peer.write_all(&server_frame(true, 0, &[0x06, 0x07]))
        .unwrap();
    let n = read_at_least(&mut ws, &mut buf, 4);
    lines.push(format!(
        "server binary FD05 then continuation 0607\tread {}",
        escape(&buf[..n])
    ));

    ws.write_all(&[0xFD, 0x09, 0x08]).unwrap();
    let frame = read_client_frame(&mut peer).unwrap();
    lines.push(format!("client write FD0908\t{}", frame.describe()));

    peer.write_all(&server_frame(true, 1, br#"0{"sid":"abc","upgrades":[]}"#))
        .unwrap();
    assert_eq!(ws.read(&mut buf).unwrap(), 0);
    lines.push(format!(
        "server text 0{{sid}}\tsocketio {}",
        title(ws.is_socketio())
    ));
    ws.write_all(&[0x01, 0x02, 0x03]).unwrap();
    let frame = read_client_frame(&mut peer).unwrap();
    lines.push(format!("client write 010203\t{}", frame.describe()));

    peer.write_all(&server_frame(true, 1, b"3probe")).unwrap();
    assert_eq!(ws.read(&mut buf).unwrap(), 0);
    for _ in 0..2 {
        let frame = read_client_frame(&mut peer).unwrap();
        lines.push(format!("server text 3probe\t{}", frame.describe()));
    }

    peer.write_all(&server_frame(true, 9, b"hi")).unwrap();
    assert_eq!(ws.read(&mut buf).unwrap(), 0);
    let frame = read_client_frame(&mut peer).unwrap();
    lines.push(format!("server ping hi\t{}", frame.describe()));

    // Other text draws nothing: the next frame the server sees is the answer to a later ping.
    peer.write_all(&server_frame(true, 1, br#"42["x"]"#))
        .unwrap();
    let n = ws.read(&mut buf).unwrap();
    peer.write_all(&server_frame(true, 9, b"after")).unwrap();
    assert_eq!(ws.read(&mut buf).unwrap(), 0);
    let next = read_client_frame(&mut peer).unwrap();
    let none = next.opcode == 10 && next.payload == b"after";
    lines.push(format!(
        "server text 42\t{} bytestoread {n}",
        if none { "none" } else { "something" }
    ));

    // The server goes without a close frame: the reader opens again, with the session id.
    let server = ws_accept(&listener);
    drop(peer);
    assert_eq!(ws.read(&mut buf).unwrap(), 0);
    let (request, mut again) = server.join().unwrap();
    lines.push("drop\treconnected True".to_owned());
    lines.push(format!(
        "handshake again\t{}",
        port_free(escape(request.as_bytes()))
    ));
    let probe = read_client_frame(&mut again).unwrap();
    lines.push(format!("client sends again\t{}", probe.describe()));
    lines.push(format!("isopen again\t{}", title(ws.is_open())));

    // A close frame: not answered, and the link is not open after it, and not reopened.
    again
        .write_all(&server_frame(true, 8, &[0x03, 0xE8]))
        .unwrap();
    assert_eq!(ws.read(&mut buf).unwrap(), 0);
    let open_after = ws.is_open();
    assert_eq!(ws.read(&mut buf).unwrap(), 0, "a closed link reads nothing");
    listener.set_nonblocking(true).unwrap();
    let reconnected = match listener.accept() {
        Ok(_) => true,
        Err(e) if e.kind() == io::ErrorKind::WouldBlock => false,
        Err(e) => panic!("{e}"),
    };
    ws.close();
    let answered = read_to_end(&mut again);
    lines.push(format!(
        "server close 1000\t{}",
        if answered.is_empty() {
            "none".to_owned()
        } else {
            escape(&answered)
        }
    ));
    lines.push(format!("after close\tisopen {}", title(open_after)));
    lines.push(format!("after close\treconnected {}", title(reconnected)));

    assert_eq!(lines, expected);
}

/// `True`/`False`, as C# prints a bool.
fn title(value: bool) -> &'static str {
    if value { "True" } else { "False" }
}

fn read_at_least(ws: &mut WebSocketTransport, buf: &mut [u8], want: usize) -> usize {
    let mut n = 0;
    while n < want {
        let got = ws.read(&mut buf[n..]).unwrap();
        assert!(got > 0 || ws.is_open(), "the link closed");
        n += got;
    }
    n
}

// ---------------------------------------------------------------- UDP client

#[test]
fn is_in_range_answers_as_the_csharps_does() {
    let mut checked = 0;
    for (case, expected) in golden_lines("udp.txt") {
        let Some(address) = case.strip_prefix("isinrange ") else {
            continue;
        };
        let address: IpAddr = address.parse().unwrap();
        assert_eq!(title(is_in_range(address)), expected, "{case}");
        checked += 1;
    }
    assert_eq!(checked, 13);
}

/// Asks until the answer is `Some`, for a datagram to finish arriving; bounded, and never asleep.
fn until<T>(mut ask: impl FnMut() -> Option<T>) -> T {
    let started = web_time::Instant::now();
    loop {
        if let Some(answer) = ask() {
            return answer;
        }
        assert!(started.elapsed() < common::GUARD, "gave up waiting");
        std::hint::spin_loop();
    }
}

#[test]
fn the_udp_client_reads_writes_and_counts_as_the_csharps_does() {
    let golden = golden_lines("udp.txt");
    let result = |case: &str| {
        golden
            .iter()
            .find(|(c, _)| c == case)
            .map(|(_, r)| r.clone())
            .unwrap_or_else(|| panic!("no {case:?} in udp.txt"))
    };

    let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
    peer.set_read_timeout(Some(common::GUARD)).unwrap();
    let port = peer.local_addr().unwrap().port();
    let mut client = UdpClientTransport::open("127.0.0.1", port).unwrap();
    let open = result("open");
    assert!(
        open.starts_with(&format!("isopen {} ", title(client.is_open()))),
        "{open}"
    );
    assert!(
        open.ends_with(&format!(" bytestoread {}", client.bytes_to_read().unwrap())),
        "{open}"
    );

    client.write_all(&[1, 2, 3, 4, 5]).unwrap();
    let mut got = [0u8; 64];
    let (n, from) = peer.recv_from(&mut got).unwrap();
    assert_eq!(
        result("write 0102030405 offset 0 length 5"),
        format!(
            "{}{}",
            escape(&got[..n]),
            if from.port() == port {
                " from the same port"
            } else {
                " from another port"
            }
        )
    );
    // `write 0102030405 offset 2 length 3` is the C# ignoring its offset; a slice has none.

    let big: Vec<u8> = (0..100).collect();
    let small: Vec<u8> = (200..250).collect();
    peer.send_to(&big, from).unwrap();
    peer.send_to(&small, from).unwrap();
    let everything = 150;
    // Linux's FIONREAD counts the next datagram, which is what the golden (made on Linux) holds;
    // Windows's counts every byte queued.
    let queued = until(|| {
        let n = client.bytes_to_read().unwrap();
        let settled = if cfg!(any(target_os = "linux", target_os = "android")) {
            n > 0
        } else {
            n == everything
        };
        settled.then_some(n)
    });
    if cfg!(any(target_os = "linux", target_os = "android")) {
        assert_eq!(
            result("two datagrams 100 then 50"),
            format!("bytestoread {queued}")
        );
    }

    let mut buf = [0u8; 256];
    let n = client.read(&mut buf[..10]).unwrap();
    // What is left of the first datagram, and the second once it is in.
    let left = until(|| {
        let left = client.bytes_to_read().unwrap();
        (left > 90).then_some(left)
    });
    assert_eq!(
        result("read 10"),
        format!(
            "got {n} first {} last {} bytestoread {left}",
            buf[0],
            buf[n - 1]
        )
    );

    // The C# waits for 200 bytes and gets 140 when its timeout ends; this crate's read gives
    // what has arrived, so the same 140 come in the same order, in two reads on Linux - the rest
    // of the first datagram, then the second - and in one elsewhere, where `bytes_to_read` has
    // already taken both datagrams into the buffer (its FIONREAD counts every queued byte) and the
    // read after finds nothing before its timeout (the owner's Mac, 2026-10-03).
    let first = client.read(&mut buf).unwrap();
    let second = client.read(&mut buf[first..]).unwrap();
    let n = first + second;
    let split = if cfg!(any(target_os = "linux", target_os = "android")) {
        (90, 50)
    } else {
        (140, 0)
    };
    assert_eq!((first, second), split);
    assert_eq!(
        result("read 200"),
        format!(
            "got {n} first {} last {} bytestoread {}",
            buf[0],
            buf[n - 1],
            client.bytes_to_read().unwrap()
        )
    );

    client.set_read_timeout(Duration::from_millis(1)).unwrap();
    let n = client.read(&mut buf[..10]).unwrap();
    assert_eq!(
        result("read 10 of nothing"),
        format!("got {n} after the timeout")
    );

    client.close();
    assert_eq!(
        result("close"),
        format!("isopen {}", title(client.is_open()))
    );
}

#[test]
fn joining_a_group_twice_fails_here_as_it_did_there() {
    // Why the C#'s every-30-seconds join is one join: the second throws, and its loop returns.
    assert_eq!(
        golden_lines("udp.txt")
            .into_iter()
            .find(|(case, _)| case == "join twice")
            .unwrap()
            .1,
        "error SocketException AddressAlreadyInUse"
    );
    let socket = UdpSocket::bind("0.0.0.0:0").unwrap();
    let group = "239.255.10.10".parse().unwrap();
    let any = "0.0.0.0".parse().unwrap();
    if let Err(e) = socket.join_multicast_v4(&group, &any) {
        eprintln!("skipped: this machine cannot join a multicast group ({e})");
        return;
    }
    let second = socket
        .join_multicast_v4(&group, &any)
        .expect_err("a second join");
    // The golden is mono's on Linux, where the kernel answers EADDRINUSE. Windows refuses the
    // same second IP_ADD_MEMBERSHIP with WSAEINVAL (CI run 37173996196), as it does under .NET
    // there (SocketError.InvalidArgument): the join still fails, and the loop still returns.
    let expected = if cfg!(windows) {
        io::ErrorKind::InvalidInput
    } else {
        io::ErrorKind::AddrInUse
    };
    assert_eq!(second.kind(), expected);
}
