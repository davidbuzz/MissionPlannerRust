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

//! Stream decoding under realistic and hostile conditions.
//!
//! Real links deliver frames split across reads, interleaved with noise from autobaud, bootloader
//! chatter and half-open ports. These tests encode those failure modes as assertions.

// Test code deliberately uses unwrap/expect/indexing: a panic here is a test failure with a
// useful message, which is exactly what we want. The production lint policy stays strict.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_possible_truncation)]

mod support;

use mp_mavlink::{FrameDecoder, encode_v2};
use support::reference_dialect;

fn heartbeat(seq: u8, dialect: &support::CsvDialect) -> Vec<u8> {
    let info = dialect.row("HEARTBEAT");
    let payload = [0u8, 0, 0, 0, 6, 8, 0, 0, 3];
    let mut buf = [0u8; 64];
    let n = encode_v2(&mut buf, seq, 1, 1, info.id, &payload, info.crc_extra, 0).expect("encode");
    buf[..n].to_vec()
}

#[test]
fn decodes_frames_split_across_arbitrary_read_boundaries() {
    let dialect = reference_dialect();
    let mut stream = Vec::new();
    for seq in 0..20 {
        stream.extend_from_slice(&heartbeat(seq, &dialect));
    }

    // Every chunk size from 1 byte (worst case serial dribble) up to the whole stream.
    for chunk in 1..=stream.len() {
        let mut decoder = FrameDecoder::new();
        let mut seqs = Vec::new();
        for part in stream.chunks(chunk) {
            decoder.push_and_drain(part, &dialect, |f| seqs.push(f.seq));
        }
        assert_eq!(seqs, (0..20).collect::<Vec<_>>(), "chunk size {chunk}");
        assert_eq!(decoder.stats().frames, 20);
        assert_eq!(decoder.stats().crc_errors, 0);
    }
}

#[test]
fn resynchronises_after_leading_noise_and_recovers_every_frame() {
    let dialect = reference_dialect();
    let mut stream = vec![0xFD, 0xFD, 0x00, 0xFE, 0x42, 0xFD, 0xFF]; // header-shaped garbage
    stream.extend_from_slice(&heartbeat(1, &dialect));
    stream.extend_from_slice(&[0x00; 9]);
    stream.extend_from_slice(&heartbeat(2, &dialect));

    let mut decoder = FrameDecoder::new();
    let mut seqs = Vec::new();
    decoder.push(&stream);
    // The leading garbage claims a 253-byte payload, so an in-order drain correctly waits.
    decoder.drain(&dialect, |f| seqs.push(f.seq));
    assert!(
        seqs.is_empty(),
        "drain must not reorder around an unresolved candidate"
    );

    // At end-of-stream we know the candidate is noise; flush resynchronises past it.
    decoder.flush(&dialect, |f| seqs.push(f.seq));
    assert_eq!(seqs, vec![1, 2]);
    assert!(
        decoder.stats().resync_bytes > 0,
        "should have skipped the noise"
    );
}

#[test]
fn a_bogus_long_header_resolves_itself_once_enough_bytes_arrive() {
    let dialect = reference_dialect();
    // Garbage claiming a maximal payload, then real frames, then enough filler that the
    // bogus candidate can be evaluated and rejected without any explicit flush.
    let mut stream = vec![0xFD, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
    stream.extend_from_slice(&heartbeat(11, &dialect));
    stream.extend_from_slice(&vec![0x55; mp_mavlink::MAX_FRAME_LEN]);
    stream.extend_from_slice(&heartbeat(12, &dialect));

    let mut decoder = FrameDecoder::new();
    let mut seqs = Vec::new();
    for chunk in stream.chunks(32) {
        decoder.push_and_drain(chunk, &dialect, |f| seqs.push(f.seq));
    }

    assert!(
        seqs.contains(&12),
        "a live link must recover without an explicit flush: {seqs:?}"
    );
}

#[test]
fn corrupted_frame_is_rejected_and_the_next_one_still_arrives() {
    let dialect = reference_dialect();
    let mut bad = heartbeat(5, &dialect);
    let last = bad.len() - 1;
    bad[last] ^= 0xFF; // smash the checksum

    let mut stream = bad;
    stream.extend_from_slice(&heartbeat(6, &dialect));

    let mut decoder = FrameDecoder::new();
    let mut seqs = Vec::new();
    decoder.push_and_drain(&stream, &dialect, |f| seqs.push(f.seq));

    assert_eq!(seqs, vec![6], "corrupt frame must not be delivered");
    assert_eq!(decoder.stats().crc_errors, 1);
}

#[test]
fn unknown_message_ids_are_counted_not_delivered() {
    let dialect = reference_dialect();
    let mut buf = [0u8; 64];
    // 0x00FFFF is not a real message id, so the dialect has no CRC seed for it.
    let n = encode_v2(&mut buf, 1, 1, 1, 0x00_FFFF, &[1, 2, 3], 0, 0).expect("encode");

    let mut decoder = FrameDecoder::new();
    let mut count = 0;
    decoder.push_and_drain(&buf[..n], &dialect, |_| count += 1);

    assert_eq!(count, 0);
    assert_eq!(decoder.stats().unknown_msgids, 1);
}

#[test]
fn a_stream_of_pure_garbage_never_wedges_and_never_yields_frames() {
    let dialect = reference_dialect();
    let mut decoder = FrameDecoder::new();
    let mut count = 0;

    // A pathological stream: every byte looks like a v2 start byte with a maximal length.
    let garbage = vec![0xFDu8; 4096];
    for chunk in garbage.chunks(97) {
        decoder.push_and_drain(chunk, &dialect, |_| count += 1);
    }

    assert_eq!(count, 0);
    assert!(
        decoder.buffered() < mp_mavlink::MAX_FRAME_LEN,
        "decoder must not wedge full"
    );
    assert_eq!(
        decoder.stats().overflow_bytes,
        0,
        "buffer must keep draining"
    );
}

#[test]
fn buffer_overflow_is_reported_rather_than_silently_dropping() {
    let dialect = reference_dialect();
    let mut decoder = FrameDecoder::new();
    // Push far more than capacity without draining.
    let pushed = decoder.push(&vec![0u8; mp_mavlink::decoder::CAPACITY * 2]);
    assert_eq!(pushed, mp_mavlink::decoder::CAPACITY);
    assert_eq!(
        decoder.stats().overflow_bytes as usize,
        mp_mavlink::decoder::CAPACITY
    );
    let _ = dialect;
}

#[test]
fn push_and_drain_consumes_buffers_larger_than_its_own_capacity() {
    // Regression: `push` alone accepts only CAPACITY bytes and counts the rest as overflow, so a
    // caller handing over a 4 KiB socket read used to lose most of it. Decoding must not depend
    // on how the OS happened to chunk the stream.
    let dialect = reference_dialect();
    let mut stream = Vec::new();
    for seq in 0..200u8 {
        stream.extend_from_slice(&heartbeat(seq, &dialect));
    }
    assert!(
        stream.len() > mp_mavlink::decoder::CAPACITY * 4,
        "test needs a large buffer"
    );

    let mut decoder = FrameDecoder::new();
    let mut seqs = Vec::new();
    let consumed = decoder.push_and_drain(&stream, &dialect, |f| seqs.push(f.seq));

    assert_eq!(consumed, stream.len(), "the whole buffer must be consumed");
    assert_eq!(seqs.len(), 200, "every frame must be delivered");
    assert_eq!(decoder.stats().overflow_bytes, 0, "no silent data loss");
}
