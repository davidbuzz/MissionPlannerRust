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

//! MAVFTP held to Mission Planner's own `MAVFtp.cs`, run under mono.
//!
//! `testdata/ftp/csharp-tables.txt` is what `testdata/ftp/MpFtp.exe tables` prints from the
//! pinned C# build (see `tools/csharp-reference/MpFtp.cs` for how it is built and run): every errno,
//! `FTPErrorCode` and `FTPOpcode` byte's `ToString` - the words the C#'s exceptions are made of -
//! `crc_crc32` over fixed inputs, and the payload bytes `FTPPayloadHeader`'s conversion makes of
//! the requests a fresh `MAVFtp` sends in four sequences. The requests here are the ones the
//! client actually puts on the wire in those sequences, not headers built for the test.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::collections::BTreeMap;
use std::time::Instant;

use mp_ftp::mavftp::testing::FakeVehicle;
use mp_ftp::mavftp::wire::{Errno, ErrorCode, Header, Opcode};
use mp_ftp::mavftp::{FtpRequest, FtpTimeouts, MavFtp, crc_crc32};
use mp_vehicle::VehicleId;

/// The golden file, as `(kind, key) -> value`.
fn golden() -> BTreeMap<(String, String), String> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/ftp/csharp-tables.txt"
    );
    let text = std::fs::read_to_string(path).expect("testdata/ftp/csharp-tables.txt");
    text.lines()
        .map(|line| {
            let mut words = line.splitn(3, ' ');
            let kind = words.next().unwrap().to_owned();
            let key = words.next().unwrap().to_owned();
            let value = words.next().unwrap_or_default().to_owned();
            ((kind, key), value)
        })
        .collect()
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

#[test]
fn every_name_is_the_one_the_csharp_prints() {
    let golden = golden();
    for i in 0..=255u8 {
        let key = |kind: &str| golden[&(kind.to_owned(), i.to_string())].clone();
        assert_eq!(Errno(i).to_string(), key("errno"), "errno {i}");
        assert_eq!(ErrorCode(i).to_string(), key("error"), "FTPErrorCode {i}");
        assert_eq!(Opcode(i).to_string(), key("opcode"), "FTPOpcode {i}");
    }
}

#[test]
fn the_crc_is_the_csharps() {
    let golden = golden();
    let crc = |name: &str| golden[&("crc".to_owned(), name.to_owned())].clone();
    let numbered: Vec<u8> = (0..5000).map(|i| u8::try_from(i % 251).unwrap()).collect();
    let every: Vec<u8> = (0..=255).collect();
    assert_eq!(format!("{:08x}", crc_crc32(0, b"")), crc("empty"));
    assert_eq!(format!("{:08x}", crc_crc32(0, b"123456789")), crc("check"));
    assert_eq!(
        format!("{:08x}", crc_crc32(u32::MAX, b"123456789")),
        crc("check-from-ones")
    );
    assert_eq!(format!("{:08x}", crc_crc32(0, &every)), crc("every-byte"));
    assert_eq!(
        format!("{:08x}", crc_crc32(0, &numbered)),
        crc("numbered-5000")
    );
}

/// Starts `request` on a fresh client against `vehicle`, and returns every payload it sent before
/// it had to wait for anything.
fn sent(vehicle: FakeVehicle, request: FtpRequest) -> Vec<Header> {
    let mut vehicle = vehicle;
    let mut ftp = MavFtp::new(VehicleId::new(1, 1), FtpTimeouts::default());
    let now = Instant::now();
    let mut all = Vec::new();
    let mut out = Vec::new();
    assert!(ftp.start(request, now, &mut out));
    while let Some(payload) = (!out.is_empty()).then(|| out.remove(0)) {
        all.push(payload.clone());
        for reply in vehicle.answer(&payload) {
            ftp.on_message(&reply.encode(), now, &mut out);
        }
    }
    all
}

fn assert_payload(golden: &BTreeMap<(String, String), String>, name: &str, header: &Header) {
    assert_eq!(
        hex(&header.encode()),
        golden[&("payload".to_owned(), name.to_owned())],
        "payload {name}"
    );
}

#[test]
fn the_requests_on_the_wire_are_the_csharps_bytes() {
    let golden = golden();
    let numbered: Vec<u8> = (0..5000).map(|i| u8::try_from(i % 251).unwrap()).collect();

    // GetFile("@SYS/uarts.txt", cancel, true, 110): reset, open, burst.
    let get = sent(
        FakeVehicle::new().with_file("@SYS/uarts.txt", b"UARTV1\n"),
        FtpRequest::Get {
            path: "@SYS/uarts.txt".to_owned(),
            burst: true,
            readsize: 110,
        },
    );
    assert_payload(&golden, "get-reset", &get[0]);
    assert_payload(&golden, "get-open", &get[1]);
    assert_payload(&golden, "get-burst", &get[2]);

    // kCmdListDirectory("/APM/") of seven entries, on a vehicle that refuses the timed listing.
    let mut vehicle = FakeVehicle::new();
    for i in 0..7 {
        vehicle = vehicle.with_file(&format!("/APM/{i}"), b"x");
    }
    let list = sent(
        vehicle,
        FtpRequest::List {
            path: "/APM/".to_owned(),
        },
    );
    assert_payload(&golden, "list-timed", &list[0]);
    assert_payload(&golden, "list-plain", &list[1]);
    assert_payload(&golden, "list-next", &list[2]);

    // UploadFile("/APM/up.bin", ...) of 1000 bytes: reset, create, the first write.
    let put = sent(
        FakeVehicle::new(),
        FtpRequest::Put {
            path: "/APM/up.bin".to_owned(),
            data: numbered[..1000].to_vec(),
        },
    );
    assert_payload(&golden, "put-create", &put[1]);
    assert_payload(&golden, "put-write", &put[2]);

    // One command each, on a fresh client.
    let one = |request: FtpRequest, vehicle: FakeVehicle| sent(vehicle, request)[0].clone();
    assert_payload(
        &golden,
        "rename",
        &one(
            FtpRequest::Rename {
                from: "/APM/a.txt".to_owned(),
                to: "/APM/b.txt".to_owned(),
            },
            FakeVehicle::new().with_file("/APM/a.txt", b"a"),
        ),
    );
    assert_payload(
        &golden,
        "crc",
        &one(
            FtpRequest::Crc32 {
                path: "@PARAM/param.pck".to_owned(),
            },
            FakeVehicle::new().with_file("@PARAM/param.pck", b"p"),
        ),
    );
    let long = "a".repeat(300);
    assert_payload(
        &golden,
        "remove-long-path",
        &one(
            FtpRequest::RemoveFile { path: long.clone() },
            FakeVehicle::new().with_file(&long, b"x"),
        ),
    );
    assert_payload(
        &golden,
        "terminate",
        &one(FtpRequest::TerminateSession, FakeVehicle::new()),
    );
}

/// A read at the last sequence number: laid out as the C# lays it out. (That its answer can never
/// match, 65535 + 1 being 65536 in the C#'s `int` arithmetic, is `steps.rs`'s test.)
#[test]
fn a_read_at_the_last_sequence_number_is_laid_out_as_the_csharps() {
    let golden = golden();
    let header = Header {
        opcode: Opcode::READ_FILE,
        seq_number: u16::MAX,
        offset: 240,
        session: 0,
        size: 80,
        ..Header::default()
    };
    assert_payload(&golden, "read-65535", &header);
    // And the payloads read back as what they were.
    for ((kind, name), value) in &golden {
        if kind != "payload" {
            continue;
        }
        let bytes: Vec<u8> = (0..value.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&value[i..i + 2], 16).unwrap())
            .collect();
        assert_eq!(bytes.len(), 251, "{name}");
        assert_eq!(Header::decode(&bytes).encode().to_vec(), bytes, "{name}");
    }
}
