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

//! Deterministic robustness testing for the parsers.
//!
//! The `fuzz/` directory holds libfuzzer targets for deep runs, but those need a nightly toolchain
//! and cargo-fuzz. These tests check the same invariants with a seeded generator so they run on
//! every `cargo test`, on stable, on every platform - which means the properties are actually
//! enforced rather than merely declared.
//!
//! Seeded, not random: a failure here must be reproducible from the test name alone.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
// The generator narrows u64 to smaller types deliberately; it is producing test input, not
// interpreting vehicle data, so truncation is the intent rather than a hazard.
#![allow(clippy::cast_possible_truncation)]

mod support;

use mp_mavlink::{FrameDecoder, MAX_FRAME_LEN, encode_v2, parse};
use support::reference_dialect;

/// xorshift64*, so the corpus is identical on every machine and every run.
struct Rng(u64);

impl Rng {
    const fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn byte(&mut self) -> u8 {
        (self.next_u64() >> 24) as u8
    }

    fn below(&mut self, limit: usize) -> usize {
        if limit == 0 {
            0
        } else {
            (self.next_u64() % limit as u64) as usize
        }
    }
}

#[test]
fn parsing_arbitrary_bytes_never_panics_and_never_overruns() {
    let dialect = reference_dialect();
    let mut rng = Rng::new(0x5EED_1234_ABCD_0001);
    let mut buffer = vec![0u8; 512];
    let mut parsed = 0u64;

    for _ in 0..200_000 {
        let len = rng.below(400) + 1;
        buffer.truncate(0);
        for _ in 0..len {
            buffer.push(rng.byte());
        }

        if let Ok((frame, used)) = parse(&buffer, &dialect) {
            assert!(used <= buffer.len(), "consumed {used} of {}", buffer.len());
            assert_eq!(frame.raw.len(), used);
            assert!(frame.payload.len() <= 255);

            let mut out = [0u8; 255];
            frame.payload_into(&mut out);
            let _ = frame.payload_byte(254);
            let _ = frame.signable_bytes();
            parsed += 1;
        }
    }
    // Random noise should essentially never pass a CRC check with the right seed.
    assert!(
        parsed < 100,
        "random noise produced {parsed} valid frames, which is implausible"
    );
}

#[test]
fn mutating_a_valid_frame_never_panics() {
    // Corruption in the wild is a flipped bit in an otherwise well-formed frame, not uniform
    // noise. This is the case that actually reaches the field.
    let dialect = reference_dialect();
    let info = dialect.row("GLOBAL_POSITION_INT");
    let mut rng = Rng::new(0x5EED_1234_ABCD_0002);

    let payload = [7u8; 28];
    let mut original = [0u8; 64];
    let n = encode_v2(&mut original, 1, 1, 1, info.id, &payload, info.crc_extra, 0).unwrap();

    for _ in 0..200_000 {
        let mut frame = original;
        // One to three mutations, which is where real radio corruption lives.
        for _ in 0..=rng.below(3) {
            let at = rng.below(n);
            frame[at] ^= 1 << rng.below(8);
        }
        if let Ok((parsed, used)) = parse(&frame[..n], &dialect) {
            assert!(used <= n);
            let mut out = [0u8; 255];
            parsed.payload_into(&mut out);
        }
    }
}

#[test]
fn the_decoder_never_wedges_on_arbitrary_input() {
    // Liveness, not just safety: a decoder that fills its buffer and stops looks exactly like a
    // dead link, and no single-frame test can see it.
    let dialect = reference_dialect();
    let mut rng = Rng::new(0x5EED_1234_ABCD_0003);

    for _ in 0..400 {
        let mut decoder = FrameDecoder::new();
        let total = rng.below(4000) + 1;
        let mut stream = Vec::with_capacity(total);
        for _ in 0..total {
            stream.push(rng.byte());
        }

        let mut offset = 0usize;
        let mut step = 1usize;
        while offset < stream.len() {
            let end = (offset + step).min(stream.len());
            let consumed = decoder.push_and_drain(&stream[offset..end], &dialect, |_| {});
            assert_eq!(
                consumed,
                end - offset,
                "push_and_drain must consume everything"
            );
            offset = end;
            step = (step * 3 % 97) + 1;
        }
        decoder.flush(&dialect, |_| {});

        assert!(
            decoder.buffered() < MAX_FRAME_LEN,
            "decoder wedged holding {} bytes",
            decoder.buffered()
        );
    }
}

#[test]
fn valid_frames_buried_in_noise_are_still_found() {
    // The counterpart to "never panics": the decoder must not become so defensive that it drops
    // good frames. Resynchronisation has to actually resynchronise.
    let dialect = reference_dialect();
    let info = dialect.row("HEARTBEAT");
    let mut rng = Rng::new(0x5EED_1234_ABCD_0004);

    let payload = [0u8, 0, 0, 0, 6, 8, 0, 0, 3];
    let mut frame = [0u8; 64];
    let n = encode_v2(&mut frame, 42, 1, 1, info.id, &payload, info.crc_extra, 0).unwrap();

    let mut recovered = 0u32;
    const TRIALS: u32 = 500;

    for _ in 0..TRIALS {
        let mut stream = Vec::new();
        for _ in 0..rng.below(200) {
            stream.push(rng.byte());
        }
        stream.extend_from_slice(&frame[..n]);
        for _ in 0..rng.below(200) {
            stream.push(rng.byte());
        }

        let mut decoder = FrameDecoder::new();
        let mut found = false;
        decoder.push_and_drain(&stream, &dialect, |f| {
            if f.seq == 42 && f.msgid == info.id {
                found = true;
            }
        });
        decoder.flush(&dialect, |f| {
            if f.seq == 42 && f.msgid == info.id {
                found = true;
            }
        });
        if found {
            recovered += 1;
        }
    }

    // Not 100%: random noise can legitimately contain a byte sequence that consumes the real
    // frame as part of a longer well-formed-looking candidate. It must be close to it.
    assert!(
        recovered * 100 >= TRIALS * 95,
        "only recovered {recovered} of {TRIALS} frames buried in noise"
    );
}
