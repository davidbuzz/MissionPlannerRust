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

//! MAVLink v2 signing round-trips and rejection cases.

// Test code deliberately uses unwrap/expect/indexing: a panic here is a test failure with a
// useful message, which is exactly what we want. The production lint policy stays strict.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_possible_truncation)]

mod support;

use mp_mavlink::{
    INCOMPAT_FLAG_SIGNED, SIGNATURE_LEN, SigningKey, encode_v2, parse, sign, signing, verify,
};
use support::reference_dialect;

/// Builds a signed frame by encoding, then setting the signed flag and appending the block.
fn signed_frame(key: &SigningKey, link_id: u8, timestamp: u64, seq: u8) -> Vec<u8> {
    let dialect = reference_dialect();
    let info = dialect.row("HEARTBEAT");
    let payload = [0u8, 0, 0, 0, 6, 8, 0, 0, 3];

    let mut buf = [0u8; 128];
    let n = encode_v2(&mut buf, seq, 1, 1, info.id, &payload, info.crc_extra, 0).expect("encode");

    // Re-encode with the signed flag set so the checksum covers it.
    buf[2] = INCOMPAT_FLAG_SIGNED;
    let payload_end = n - 2;
    let ck = mp_mavlink::crc::checksum(&buf[1..payload_end], info.crc_extra);
    buf[payload_end..payload_end + 2].copy_from_slice(&ck.to_le_bytes());

    let sig = sign(key, link_id, timestamp, &buf[..n]);
    let mut out = buf[..n].to_vec();
    out.extend_from_slice(&sig);
    out
}

#[test]
fn signed_frame_verifies_with_the_right_key() {
    let dialect = reference_dialect();
    let key = SigningKey::from_passphrase("correct horse battery staple");
    let bytes = signed_frame(&key, 3, 1_234_567, 42);

    let (frame, used) = parse(&bytes, &dialect).expect("parse");
    assert_eq!(used, bytes.len());
    assert!(frame.is_signed());
    assert_eq!(frame.signature.map(<[u8]>::len), Some(SIGNATURE_LEN));
    assert!(verify(&key, &frame), "signature must verify");

    let (link_id, ts) = signing::signature_meta(frame.signature.expect("sig")).expect("meta");
    assert_eq!(link_id, 3);
    assert_eq!(ts, 1_234_567);
}

#[test]
fn signed_frame_is_rejected_with_the_wrong_key() {
    let dialect = reference_dialect();
    let key = SigningKey::from_passphrase("correct horse battery staple");
    let attacker = SigningKey::from_passphrase("correct horse battery stapl3");
    let bytes = signed_frame(&key, 1, 99, 7);

    let (frame, _) = parse(&bytes, &dialect).expect("parse");
    assert!(!verify(&attacker, &frame), "forged key must not verify");
}

#[test]
fn tampering_with_the_payload_breaks_the_signature() {
    let dialect = reference_dialect();
    let key = SigningKey::from_passphrase("hunter2");
    let mut bytes = signed_frame(&key, 1, 99, 7);

    // Flip a payload byte and repair the checksum, so only the signature can catch it.
    let info = dialect.row("HEARTBEAT");
    bytes[12] ^= 0x01;
    let payload_end = bytes.len() - SIGNATURE_LEN - 2;
    let ck = mp_mavlink::crc::checksum(&bytes[1..payload_end], info.crc_extra);
    bytes[payload_end..payload_end + 2].copy_from_slice(&ck.to_le_bytes());

    let (frame, _) = parse(&bytes, &dialect).expect("parse");
    assert!(
        !verify(&key, &frame),
        "tampered frame must fail signature check"
    );
}

#[test]
fn timestamp_conversion_matches_the_2015_epoch() {
    // 2015-01-01T00:00:00Z is timestamp 0; one second later is 100_000 ten-microsecond ticks.
    assert_eq!(
        signing::timestamp_from_unix_micros(1_420_070_400_000_000),
        0
    );
    assert_eq!(
        signing::timestamp_from_unix_micros(1_420_070_401_000_000),
        100_000
    );
    assert_eq!(
        signing::timestamp_from_unix_micros(0),
        0,
        "pre-epoch clamps to zero"
    );
}
