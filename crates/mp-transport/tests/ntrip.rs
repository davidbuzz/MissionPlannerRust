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

//! The NTRIP client against a mock caster on 127.0.0.1 that checks the request it is sent,
//! serves a source table and a mount point, records the GGA sentences it receives, and streams
//! RTCM.
//!
//! The byte-for-byte comparison with what the C# sent is `csharp_goldens.rs`; this is the caster's
//! side of the conversation, with a sentence built by hand from `CommsNTRIP.cs`'s format.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

mod common;

use std::io::{self, Write};
use std::net::TcpListener;
use wasm_thread::JoinHandle;
use web_time::{Duration, SystemTime, UNIX_EPOCH};

use common::{base64_reference, caster, listen, read_to_end, read_until, sentences};
use mp_transport::ntrip::{GGA_INTERVAL, nmea_checksum};
use mp_transport::{NtripOptions, NtripTransport, OpenError, Transport};

/// Canberra, as ArduPilot's SITL has it.
const LAT: f64 = -35.363_261;
const LNG: f64 = 149.165_230;
const ALT: f64 = 584.0;

/// 12:34:56.78 UTC on some day.
fn noon_ish() -> SystemTime {
    UNIX_EPOCH
        + Duration::from_secs(20_000 * 86_400 + 12 * 3600 + 34 * 60 + 56)
        + Duration::from_millis(789)
}

/// The sentence `CommsNTRIP.SendNMEA` makes of Canberra at 12:34:56.78, built by hand from its
/// format string (CommsNTRIP.cs:416-422): `(int)lat + (lat - (int)lat) * .6f` is -35.21795660866,
/// so 3521.80 minutes south; 149.09913800394 is 14909.91 east; hundredths of a second truncated;
/// and the checksum worked out by hand.
const HAND_BUILT: &str = "$GPGGA,123456.78,3521.80,S,14909.91,E,1,10,1,584.00,M,0,M,0.0,0*6E\r\n";

/// Some RTCM 3: a preamble, a length and a body - it only has to come out as it went in.
fn rtcm(length: usize) -> Vec<u8> {
    let mut frame = vec![0xD3, 0x00, u8::try_from(length).unwrap()];
    frame.extend((0..length).map(|i| u8::try_from(i * 7 % 256).unwrap()));
    frame.extend_from_slice(&[0x12, 0x34, 0x56]);
    frame
}

/// A caster with one mount point, MOUNT, and a source table for everything else: checks what
/// it is asked, and gives back the request and the connection.
fn mock_caster(listener: &TcpListener) -> JoinHandle<(String, std::net::TcpStream)> {
    let listener = listener.try_clone().unwrap();
    wasm_thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        let request = String::from_utf8(read_until(&mut stream, b"\r\n\r\n")).unwrap();
        let first = request.lines().next().unwrap().to_owned();
        if first == "GET /MOUNT HTTP/1.1" || first == "GET /MOUNT HTTP/1.0" {
            stream.write_all(b"ICY 200 OK\r\n").unwrap();
        } else {
            stream
                .write_all(
                    b"SOURCETABLE 200 OK\r\nServer: mock\r\nContent-Type: text/plain\r\n\r\n\
                      STR;MOUNT;Canberra;RTCM 3.2;1005(10);2;GPS;SNIP;AUS;-35.36;149.17;1;0;sNTRIP;none;B;N;9600;\r\n\
                      ENDSOURCETABLE\r\n",
                )
                .unwrap();
            stream.shutdown(std::net::Shutdown::Write).unwrap();
        }
        (request, stream)
    })
}

fn header<'a>(request: &'a str, name: &str) -> Option<&'a str> {
    request.split("\r\n").find_map(|line| {
        let (field, value) = line.split_once(": ")?;
        (field == name).then_some(value)
    })
}

#[test]
fn the_caster_is_asked_properly_sent_the_position_and_streams_rtcm_back() {
    let (listener, port) = listen();
    let caster = mock_caster(&listener);
    let mut ntrip = NtripTransport::open_with_clock(
        &format!("ntrip://pilot:s3cret@127.0.0.1:{port}/MOUNT"),
        NtripOptions {
            lat: LAT,
            lng: LNG,
            alt: ALT,
            ntrip_v1: false,
        },
        Box::new(noon_ish),
    )
    .unwrap();
    let (request, mut stream) = caster.join().unwrap();

    // The request line and headers, NTRIP 2.
    assert!(
        request.starts_with("GET /MOUNT HTTP/1.1\r\n"),
        "{request:?}"
    );
    assert_eq!(
        header(&request, "Host"),
        Some(format!("127.0.0.1:{port}").as_str())
    );
    assert_eq!(header(&request, "Ntrip-Version"), Some("Ntrip/2.0"));
    assert_eq!(
        header(&request, "User-Agent"),
        Some("NTRIP MissionPlanner/1.0")
    );
    let basic = format!("Basic {}", base64_reference(b"pilot:s3cret"));
    assert_eq!(header(&request, "Authorization"), Some(basic.as_str()));
    assert_eq!(header(&request, "Connection"), Some("close"));

    // The GGA sentence arrives straight after the caster's answer, as the hand-built one.
    let gga = read_until(&mut stream, b"\r\n");
    assert_eq!(String::from_utf8(gga).unwrap(), HAND_BUILT);
    let body = &HAND_BUILT[1..HAND_BUILT.len() - 5];
    assert_eq!(nmea_checksum(&format!("${body}")), "6E");

    // RTCM out of the caster is the byte stream.
    let mut sent = Vec::new();
    for length in [19, 120, 7, 200] {
        sent.extend(rtcm(length));
    }
    stream.write_all(&sent).unwrap();
    let mut got = Vec::new();
    let mut buf = [0u8; 110];
    ntrip.set_read_timeout(common::GUARD).unwrap();
    while got.len() < sent.len() {
        let n = ntrip.read(&mut buf).unwrap();
        assert!(n > 0, "the caster's bytes stopped");
        got.extend_from_slice(&buf[..n]);
    }
    assert_eq!(got, sent);
    assert!(ntrip.is_open());
}

#[test]
fn a_mount_point_the_caster_does_not_have_is_a_source_table_and_an_error() {
    let (listener, port) = listen();
    let caster = mock_caster(&listener);
    let error = NtripTransport::open(
        &format!("ntrip://127.0.0.1:{port}/NOWHERE"),
        NtripOptions::default(),
    )
    .expect_err("a source table is not a stream");
    let (request, _) = caster.join().unwrap();
    assert!(request.starts_with("GET /NOWHERE HTTP/1.1\r\n"));
    let OpenError::Io { source, .. } = error else {
        panic!("{error}")
    };
    assert_eq!(source.kind(), io::ErrorKind::NotFound);
    assert_eq!(
        source.to_string(),
        "Got SOURCETABLE - Bad ntrip mount point\n\nSOURCETABLE 200 OK"
    );

    // Asking for no mount point at all is how a caster is asked for its table.
    let caster = mock_caster(&listener);
    let error = NtripTransport::open(
        &format!("ntrip://127.0.0.1:{port}"),
        NtripOptions::default(),
    )
    .expect_err("a source table is not a stream");
    let (request, _) = caster.join().unwrap();
    assert!(request.starts_with("GET / HTTP/1.1\r\n"), "{request:?}");
    assert!(error.to_string().contains("Got SOURCETABLE"), "{error}");
}

#[test]
fn a_refusal_is_a_bad_response_carrying_the_casters_line() {
    let (listener, port) = listen();
    let peer = caster(&listener, b"HTTP/1.1 401 Unauthorized\r\n\r\n", false);
    let error = NtripTransport::open(
        &format!("ntrip://wrong:guess@127.0.0.1:{port}/MOUNT"),
        NtripOptions::default(),
    )
    .expect_err("401");
    drop(peer.join().unwrap());
    let OpenError::Io { source, .. } = error else {
        panic!("{error}")
    };
    assert_eq!(source.kind(), io::ErrorKind::InvalidData);
    assert_eq!(
        source.to_string(),
        "Bad ntrip Response\n\nHTTP/1.1 401 Unauthorized"
    );
}

#[test]
fn ntrip_1_asks_without_host_or_version() {
    let (listener, port) = listen();
    let caster = mock_caster(&listener);
    let ntrip = NtripTransport::open(
        &format!("ntrip://127.0.0.1:{port}/MOUNT"),
        NtripOptions {
            ntrip_v1: true,
            ..NtripOptions::default()
        },
    )
    .unwrap();
    let (request, _) = caster.join().unwrap();
    assert_eq!(
        request,
        "GET /MOUNT HTTP/1.0\r\nUser-Agent: NTRIP MissionPlanner/1.0\r\nConnection: close\r\n\r\n"
    );
    drop(ntrip);
}

#[test]
fn without_a_position_the_caster_is_sent_nothing_but_the_request() {
    let (listener, port) = listen();
    let caster = mock_caster(&listener);
    let mut ntrip = NtripTransport::open(
        &format!("ntrip://127.0.0.1:{port}/MOUNT"),
        NtripOptions::default(),
    )
    .unwrap();
    ntrip.set_read_timeout(Duration::from_millis(1)).unwrap();
    let mut buf = [0u8; 16];
    assert_eq!(ntrip.read(&mut buf).unwrap(), 0);
    drop(ntrip);
    let (_, mut stream) = caster.join().unwrap();
    assert!(read_to_end(&mut stream).is_empty());
}

#[test]
fn a_position_set_after_open_goes_on_the_next_read_and_then_every_thirty_seconds() {
    // MainV2's `rtk` command line sets the position after Open (MainV2.cs:3757-3761).
    let (listener, port) = listen();
    let caster = mock_caster(&listener);
    let now = std::sync::Arc::new(std::sync::Mutex::new(noon_ish()));
    let clock = std::sync::Arc::clone(&now);
    let mut ntrip = NtripTransport::open_with_clock(
        &format!("ntrip://127.0.0.1:{port}/MOUNT"),
        NtripOptions::default(),
        Box::new(move || *clock.lock().unwrap()),
    )
    .unwrap();
    ntrip.set_read_timeout(Duration::from_millis(1)).unwrap();
    ntrip.set_position(LAT, LNG, ALT);
    let mut buf = [0u8; 16];
    // A hundred seconds of reads, a second apart.
    for second in 0..100 {
        *now.lock().unwrap() = noon_ish() + Duration::from_secs(second);
        assert_eq!(ntrip.read(&mut buf).unwrap(), 0);
    }
    drop(ntrip);
    let (_, mut stream) = caster.join().unwrap();
    let sent = sentences(&mut stream);
    assert_eq!(sent[0], HAND_BUILT);
    // Strictly more than 30 s after the last: at 0, 31, 62 and 93 s.
    let expected = 100 / (GGA_INTERVAL.as_secs() + 1) + 1;
    assert_eq!(expected, 4);
    assert_eq!(sent.len() as u64, expected, "{sent:#?}");
    assert!(sent[1].starts_with("$GPGGA,123527.78,"), "{}", sent[1]);
}

#[test]
fn tls_is_refused_as_not_ported_before_anything_is_sent() {
    for url in [
        "ntrip://user:pass@127.0.0.1:443/MOUNT",
        "https://user:pass@127.0.0.1:2101/MOUNT",
    ] {
        let error = NtripTransport::open(url, NtripOptions::default()).expect_err(url);
        assert!(
            matches!(error, OpenError::Unsupported(what) if what.contains("TLS")),
            "{url}: {error}"
        );
    }
}

#[test]
fn a_link_url_opens_the_caster_too() {
    let (listener, port) = listen();
    let caster = mock_caster(&listener);
    let mut link = mp_transport::open(&format!("ntrip://127.0.0.1:{port}/MOUNT")).unwrap();
    let (_, mut stream) = caster.join().unwrap();
    assert_eq!(
        link.description(),
        format!("ntrip:127.0.0.1:{port}/MOUNT"),
        "no credentials in the description"
    );
    stream.write_all(&rtcm(19)).unwrap();
    link.set_read_timeout(common::GUARD).unwrap();
    let mut buf = [0u8; 64];
    assert_eq!(link.read(&mut buf).unwrap(), rtcm(19).len());
}
