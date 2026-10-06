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

//! 32-bit system ids and payload targets on the wire (tridge's, Mission Planner e6454ccdd): the
//! frame-level cases of the C#'s own tests, ported. Every frame is built here independently of
//! the production code - its own header layout, X.25 checksum and SHA-256 signature - as
//! `Sysid32Tests.Frame` builds them, so parser and writer are held to the wire format, not to
//! each other.
//! `// C#: MissionPlannerTests/Mavlink/Sysid32Tests.cs:14-155`

// Test code deliberately uses unwrap/expect/indexing: a panic here is a test failure with a
// useful message, which is exactly what we want. The production lint policy stays strict.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_possible_truncation)]

mod support;

use mp_mavlink::{
    FrameDecoder, MAX_FRAME_LEN, ParseError, SigningKey, encode_v2_targeted, parse, verify,
};
use sha2::{Digest, Sha256};
use support::reference_dialect;

/// HEARTBEAT, 0; its CRC extra.
const HEARTBEAT: (u32, u8) = (0, 50);
/// COMMAND_LONG, 76; its CRC extra.
const COMMAND_LONG: (u32, u8) = (76, 152);
/// The test's signing key: 42, 32 times.
const KEY: [u8; 32] = [42; 32];

/// `Sysid32Tests.Frame`: a v2 frame for `flags` - sequence 17, the source four bytes wide with
/// `SYSID32`, the target four bytes after the message id with `TARGET32`, the checksum, and with
/// `SIGNED` the block for link 3 at time 1, signed with [`KEY`].
fn frame(flags: u8, source: u32, target: u32, payload: &[u8], message: (u32, u8)) -> Vec<u8> {
    let (id, crc_extra) = message;
    let mut bytes = vec![253, payload.len() as u8, flags, 0, 17];
    if flags & 2 != 0 {
        bytes.extend_from_slice(&source.to_le_bytes());
    } else {
        bytes.push(source as u8);
    }
    bytes.push(1); // component
    bytes.extend_from_slice(&[id as u8, (id >> 8) as u8, (id >> 16) as u8]);
    if flags & 4 != 0 {
        bytes.extend_from_slice(&target.to_le_bytes());
    }
    bytes.extend_from_slice(payload);
    let mut crc: u16 = 0xffff;
    for &value in bytes[1..].iter().chain(std::iter::once(&crc_extra)) {
        crc ^= u16::from(value);
        for _ in 0..8 {
            crc = (crc >> 1) ^ if crc & 1 != 0 { 0x8408 } else { 0 };
        }
    }
    bytes.extend_from_slice(&crc.to_le_bytes());
    if flags & 1 != 0 {
        bytes.extend_from_slice(&[3, 1, 0, 0, 0, 0, 0]);
        let digest = Sha256::digest([&KEY[..], &bytes[..]].concat());
        bytes.extend_from_slice(&digest[..6]);
    }
    bytes
}

/// A heartbeat's payload, the C# helper's default.
const HEARTBEAT_PAYLOAD: [u8; 9] = [0, 0, 0, 0, 2, 3, 81, 4, 3];

/// `CommandPayload`: COMMAND_LONG for command 300, to `target` - 255 in the payload for one wider
/// than a byte - and component 190, its last byte 1 so trimming keeps all 33.
fn command_payload(target: u32) -> Vec<u8> {
    let mut payload = vec![0u8; 33];
    payload[28] = 44;
    payload[29] = 1;
    payload[30] = target.min(255) as u8;
    payload[31] = 190;
    payload[32] = 1;
    payload
}

/// Every frame the decoder finds in `bytes`, fed to it one byte at a time as a slow link
/// delivers them (the C#'s `FragmentedStream`): the source, the header's target, the sequence,
/// the message id, the payload, whether it is signed and whether the signature verifies.
type Decoded = (u32, Option<u32>, u8, u32, Vec<u8>, bool, bool);
fn decode(bytes: &[u8]) -> (Vec<Decoded>, mp_mavlink::DecodeStats) {
    let dialect = reference_dialect();
    let key = SigningKey::new(KEY);
    let mut decoder = FrameDecoder::new();
    let mut found = Vec::new();
    for byte in bytes {
        decoder.push_and_drain(std::slice::from_ref(byte), &dialect, |frame| {
            found.push((
                frame.sysid,
                frame.target_system,
                frame.seq,
                frame.msgid,
                frame.payload.to_vec(),
                frame.is_signed(),
                frame.is_signed() && verify(&key, frame),
            ));
        });
    }
    (found, *decoder.stats())
}

/// `IndependentFramesDecodeAllFlagCombinations`: every flag combination, ids from 1 to
/// 0xffffffff, read byte by byte - the source as wide as its flag says, the wide target from
/// the header, the payload's own target byte 255 under it, and a signature over the wide header.
#[test]
fn independent_frames_decode_all_flag_combinations() {
    for flags in 0..8u8 {
        for id in [1, 255, 256, 0x7fff_ffff, 0x8000_0000, 0xffff_ffff] {
            if flags & 2 == 0 && id > 255 {
                continue;
            }
            let target = if flags & 4 != 0 { u32::MAX } else { 7 };
            let packet = frame(flags, id, target, &command_payload(target), COMMAND_LONG);
            let (found, _) = decode(&packet);
            assert_eq!(found.len(), 1, "flags {flags} id {id}");
            let (sysid, header_target, seq, msgid, payload, signed, verified) = &found[0];
            assert_eq!(*sysid, id);
            assert_eq!(*seq, 17);
            assert_eq!(*msgid, 76);
            assert_eq!(u16::from_le_bytes([payload[28], payload[29]]), 300);
            // `GetTargetSystem`: the header's when there is one, else the payload's byte.
            let effective = header_target.unwrap_or(u32::from(payload[30]));
            assert_eq!(effective, target, "flags {flags} id {id}");
            assert_eq!(header_target.is_some(), flags & 4 != 0);
            assert_eq!(payload[31], 190);
            assert_eq!(*signed, flags & 1 != 0);
            assert_eq!(
                *verified,
                flags & 1 != 0,
                "the signature covers the wide header"
            );
        }
    }
}

/// `EncoderMatchesIndependentFramesAndSignatures`, unsigned (signing is the link's, and its tests
/// sign wide frames): a source over 255 four bytes wide, a target over 255 in the header and 255
/// in the payload, a smaller one only in the payload - byte for byte the independent frame, and
/// read back with the same target.
#[test]
fn the_encoder_matches_the_independent_frames() {
    let dialect = reference_dialect();
    for source in [1, 255, 256, 0xffff_ffff] {
        for target in [None, Some(0), Some(7), Some(0x8000_0000), Some(0xffff_ffff)] {
            let effective = target.unwrap_or(7);
            let payload = command_payload(effective);
            let mut out = [0u8; MAX_FRAME_LEN];
            let n = encode_v2_targeted(
                &mut out,
                17,
                source,
                1,
                COMMAND_LONG.0,
                &payload,
                COMMAND_LONG.1,
                0,
                target,
            )
            .unwrap();
            let flags = if source > 255 { 2 } else { 0 } | if effective > 255 { 4 } else { 0 };
            let expected = frame(flags, source, effective, &payload, COMMAND_LONG);
            assert_eq!(out[..n], expected[..], "source {source} target {target:?}");
            let (decoded, _) = parse(&out[..n], &dialect).unwrap();
            assert_eq!(decoded.sysid, source);
            assert_eq!(
                decoded
                    .target_system
                    .unwrap_or(u32::from(decoded.payload[30])),
                effective
            );
            assert_eq!(decoded.compat_flags, 0);
        }
    }
}

/// `RejectUnknownFlagsAndTruncatedOrCorruptFrames`: a frame with a flag no one implements is
/// dropped whole - the valid frame hidden in its payload is not read - and the next is; a frame
/// cut short anywhere is waited on, a corrupt one refused; the longest frame is
/// [`MAX_FRAME_LEN`], 287 bytes.
#[test]
fn unknown_flags_truncation_and_corruption_are_refused() {
    let dialect = reference_dialect();
    for flags in [128u8, 129, 134, 135] {
        let inner = frame(0, 42, 255, &HEARTBEAT_PAYLOAD, HEARTBEAT);
        let mut stream = frame(flags, 1, 255, &inner, HEARTBEAT);
        stream.extend(frame(2, u32::MAX, 255, &HEARTBEAT_PAYLOAD, HEARTBEAT));
        let (found, stats) = decode(&stream);
        assert_eq!(found.len(), 1, "flags {flags}: {found:?}");
        assert_eq!(found[0].0, u32::MAX);
        assert_eq!(stats.unsupported_flags, 1);
    }
    let mut valid = frame(7, u32::MAX, 255, &HEARTBEAT_PAYLOAD, HEARTBEAT);
    for n in 0..valid.len() {
        assert!(
            matches!(
                parse(&valid[..n], &dialect),
                Err(ParseError::Incomplete { .. })
            ),
            "cut at {n}"
        );
    }
    let at = valid.len() - 14;
    valid[at] ^= 1;
    assert!(matches!(
        parse(&valid, &dialect),
        Err(ParseError::Crc { .. })
    ));
    for length in [0, 255] {
        let packet = frame(7, u32::MAX, 255, &vec![0; length], HEARTBEAT);
        assert!(packet.len() <= MAX_FRAME_LEN);
        if length == 255 {
            assert_eq!(packet.len(), MAX_FRAME_LEN);
        }
        assert!(parse(&packet, &dialect).is_ok(), "payload of {length}");
    }
}
