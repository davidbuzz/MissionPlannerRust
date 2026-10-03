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

//! Property tests: anything we encode, we must parse back identically.

// Test code deliberately uses unwrap/expect/indexing: a panic here is a test failure with a
// useful message, which is exactly what we want. The production lint policy stays strict.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_possible_truncation)]

mod support;

use mp_mavlink::{MavVersion, encode_v1, encode_v2, parse, trim_payload};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use support::reference_dialect;

proptest! {
    // Counterexamples are persisted next to the test so a shrunk failure becomes a permanent
    // regression case rather than a one-off CI flake.
    #![proptest_config(ProptestConfig {
        cases: 2000,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "tests/proptest-regressions/roundtrip.txt",
        ))),
        ..ProptestConfig::default()
    })]

    /// v2 encode -> parse identity, including the trailing-zero truncation rule.
    #[test]
    fn v2_roundtrip(
        seq in any::<u8>(),
        sysid in any::<u8>(),
        compid in any::<u8>(),
        payload in proptest::collection::vec(any::<u8>(), 0..=255usize),
    ) {
        let dialect = reference_dialect();
        let info = dialect.row("HEARTBEAT");
        let mut buf = [0u8; 512];
        let n = encode_v2(&mut buf, seq, sysid, compid, info.id, &payload, info.crc_extra, 0)
            .expect("encode");
        let (frame, used) = parse(&buf[..n], &dialect).expect("parse");

        prop_assert_eq!(used, n);
        prop_assert_eq!(frame.version, MavVersion::V2);
        prop_assert_eq!(frame.seq, seq);
        prop_assert_eq!(frame.sysid, sysid);
        prop_assert_eq!(frame.compid, compid);
        prop_assert_eq!(frame.msgid, info.id);
        prop_assert_eq!(frame.payload, trim_payload(&payload));

        // Zero-extension must restore the original payload exactly.
        let mut restored = vec![0u8; payload.len()];
        frame.payload_into(&mut restored);
        prop_assert_eq!(restored, payload);
    }

    /// v1 encode -> parse identity. v1 does not truncate, so payloads survive byte for byte.
    #[test]
    fn v1_roundtrip(
        seq in any::<u8>(),
        sysid in any::<u8>(),
        compid in any::<u8>(),
        payload in proptest::collection::vec(any::<u8>(), 0..=255usize),
    ) {
        let dialect = reference_dialect();
        let info = dialect.row("ATTITUDE");
        let mut buf = [0u8; 512];
        let n = encode_v1(&mut buf, seq, sysid, compid, info.id, &payload, info.crc_extra)
            .expect("encode");
        let (frame, used) = parse(&buf[..n], &dialect).expect("parse");

        prop_assert_eq!(used, n);
        prop_assert_eq!(frame.version, MavVersion::V1);
        prop_assert_eq!(frame.payload, &payload[..]);
    }

    /// Flipping any single bit of a frame must never be absorbed silently: the frame is either
    /// rejected, or it decodes to observably different content. A codec that quietly accepts
    /// corrupted telemetry is worse than one that drops it.
    #[test]
    fn single_bit_flip_never_silently_corrupts(
        payload in proptest::collection::vec(any::<u8>(), 1..=64usize),
        bit in 0usize..8,
        pick in 0usize..4096,
    ) {
        let dialect = reference_dialect();
        let info = dialect.row("GLOBAL_POSITION_INT");
        let mut buf = [0u8; 512];
        let n = encode_v2(&mut buf, 7, 1, 1, info.id, &payload, info.crc_extra, 0).expect("encode");

        let (clean, _) = parse(&buf[..n], &dialect).expect("clean frame parses");
        let clean_id = (clean.msgid, clean.seq, clean.sysid, clean.compid);
        let mut clean_payload = [0u8; 255];
        clean.payload_into(&mut clean_payload);

        let byte_index = pick % n;
        buf[byte_index] ^= 1 << bit;

        if let Ok((frame, _)) = parse(&buf[..n], &dialect) {
            let dirty_id = (frame.msgid, frame.seq, frame.sysid, frame.compid);
            let mut dirty_payload = [0u8; 255];
            frame.payload_into(&mut dirty_payload);
            prop_assert!(
                dirty_id != clean_id || dirty_payload != clean_payload,
                "bit {bit} of byte {byte_index} was absorbed with no observable difference",
            );
        }
    }
}
