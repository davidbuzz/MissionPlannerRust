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

//! Throughput benchmarks. Deliverable 2's gate is >1M messages/s/core on the decode path.

// Test code deliberately uses unwrap/expect/indexing: a panic here is a test failure with a
// useful message, which is exactly what we want. The production lint policy stays strict.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_possible_truncation)]
#![allow(missing_docs)] // criterion_group! expands to an undocumented pub fn

use std::collections::HashMap;

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use mp_mavlink::{Dialect, FrameDecoder, encode_v2};

/// Minimal dialect for the benchmark mix.
struct BenchDialect(HashMap<u32, u8>);

impl Dialect for BenchDialect {
    fn crc_extra(&self, msgid: u32) -> Option<u8> {
        self.0.get(&msgid).copied()
    }
}

/// A mix resembling a real ArduPilot link: mostly small, frequent messages.
/// `(msgid, crc_extra, payload_len)`, values taken from the reference message table.
const MIX: &[(u32, u8, usize)] = &[
    (0, 50, 9),    // HEARTBEAT
    (30, 39, 28),  // ATTITUDE
    (33, 104, 28), // GLOBAL_POSITION_INT
    (74, 20, 20),  // VFR_HUD
    (1, 124, 31),  // SYS_STATUS
];

fn build_stream(frames: usize) -> (Vec<u8>, BenchDialect) {
    let dialect = BenchDialect(MIX.iter().map(|&(id, crc, _)| (id, crc)).collect());
    let mut stream = Vec::new();
    let mut buf = [0u8; 300];
    for i in 0..frames {
        let (id, crc, len) = MIX[i % MIX.len()];
        let payload = vec![(i % 251) as u8 + 1; len];
        let n = encode_v2(&mut buf, i as u8, 1, 1, id, &payload, crc, 0).expect("encode");
        stream.extend_from_slice(&buf[..n]);
    }
    (stream, dialect)
}

fn decode_throughput(c: &mut Criterion) {
    const FRAMES: usize = 10_000;
    let (stream, dialect) = build_stream(FRAMES);

    let mut group = c.benchmark_group("decode");
    group.throughput(Throughput::Elements(FRAMES as u64));

    // Whole-buffer decode: the log-replay case.
    group.bench_function("bulk", |b| {
        b.iter(|| {
            let mut decoder = FrameDecoder::new();
            let mut acc = 0u64;
            for chunk in stream.chunks(mp_mavlink::MAX_FRAME_LEN) {
                decoder.push_and_drain(chunk, &dialect, |f| acc += u64::from(f.msgid));
            }
            acc
        });
    });

    // Small reads: the live serial case, where framing overhead dominates.
    for read_size in [64usize, 256, 1024] {
        group.bench_with_input(
            BenchmarkId::new("chunked", read_size),
            &read_size,
            |b, &sz| {
                b.iter(|| {
                    let mut decoder = FrameDecoder::new();
                    let mut acc = 0u64;
                    for chunk in stream.chunks(sz) {
                        decoder.push_and_drain(chunk, &dialect, |f| acc += u64::from(f.msgid));
                    }
                    acc
                });
            },
        );
    }
    group.finish();
}

fn encode_throughput(c: &mut Criterion) {
    let dialect = BenchDialect(MIX.iter().map(|&(id, crc, _)| (id, crc)).collect());
    let payload = [9u8; 28];
    let mut buf = [0u8; 300];
    let _ = dialect;

    let mut group = c.benchmark_group("encode");
    group.throughput(Throughput::Elements(1));
    group.bench_function("v2_attitude", |b| {
        b.iter(|| encode_v2(&mut buf, 1, 1, 1, 30, &payload, 39, 0).expect("encode"));
    });
    group.finish();
}

criterion_group!(benches, decode_throughput, encode_throughput);
criterion_main!(benches);
