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

//! The snapshot bus's latency: Deliverable 5's `benches/snapshot.rs`.
//!
//! D5's DoD is a single-writer, multi-reader snapshot with zero locks on the render path, and
//! PLAN.md §8.2 puts a number on the writer's side - packet to snapshot published, p99 ≤ 200 µs -
//! while §3 records what `arc-swap` measured alone: a load in 15.4 ns and a store in 118 ns on a
//! 704-byte state. This measures the bus as `mp_vehicle` uses it: [`StatePublisher::publish`],
//! which hands the working state to a pooled `Arc` and swaps it in, and [`StateHandle::load`],
//! one atomic load.
//!
//! Two criterion benches, `publish` and `load`, at steady state with one reader; and a gate that
//! is the Deliverable 5 `tests/concurrency.rs` shape without the `loom` model: a writer publishing at 1 kHz
//! for a second while eight readers load without pause. The writer's publish p99 must be under
//! 200 µs, a reader's load p99 under 20 µs - a hundred times §3's figure, room for a descheduled
//! thread on a loaded machine - and the pool must have kept the steady state allocation-free bar
//! the slots the readers were holding.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(missing_docs)] // criterion_group! expands to an undocumented pub fn

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use criterion::{Criterion, black_box, criterion_group};
use mp_vehicle::{StatePublisher, VehicleState};

/// §8.2: packet → snapshot published, p99.
const PUBLISH_BUDGET: Duration = Duration::from_micros(200);
/// A reader's load, p99: a hundred times what `arc-swap` measures alone.
const LOAD_BUDGET: Duration = Duration::from_micros(20);
const READERS: usize = 8;
const PUBLISHES: u32 = 1000;

fn publisher() -> StatePublisher {
    let mut publisher = StatePublisher::new(VehicleState::new(1, 1));
    // Warm the pool: the first few publishes allocate, the rest reuse.
    for i in 0..16 {
        publisher.working.custom_mode = i;
        publisher.publish();
    }
    publisher
}

fn snapshot(c: &mut Criterion) {
    let mut group = c.benchmark_group("snapshot");
    group.bench_function("publish", |b| {
        let mut publisher = publisher();
        let _reader = publisher.handle();
        let mut i = 0u32;
        b.iter(|| {
            i = i.wrapping_add(1);
            publisher.working.custom_mode = i;
            publisher.publish();
        });
    });
    group.bench_function("load", |b| {
        let publisher = publisher();
        let handle = publisher.handle();
        b.iter(|| black_box(handle.load().custom_mode));
    });
    group.finish();
}

fn p99(times: &mut [Duration]) -> Duration {
    times.sort_unstable();
    times[times.len() * 99 / 100]
}

/// A writer at 1 kHz for a second, eight readers loading without pause.
fn gate() {
    if cfg!(debug_assertions) {
        println!("snapshot gate: skipped in an unoptimised build");
        return;
    }
    let mut publisher = publisher();
    let stop = Arc::new(AtomicBool::new(false));
    let readers: Vec<_> = (0..READERS)
        .map(|_| {
            let handle = publisher.handle();
            let stop = Arc::clone(&stop);
            std::thread::spawn(move || {
                let mut times = Vec::with_capacity(1 << 16);
                let mut seen = 0u64;
                while !stop.load(Ordering::Relaxed) {
                    let started = Instant::now();
                    let state = handle.load();
                    let took = started.elapsed();
                    seen += u64::from(state.custom_mode);
                    if times.len() < times.capacity() {
                        times.push(took);
                    }
                }
                (times, seen)
            })
        })
        .collect();

    let allocations_before = publisher.allocations();
    let mut publishes = Vec::with_capacity(PUBLISHES as usize);
    let mut next = Instant::now();
    for i in 0..PUBLISHES {
        publisher.working.custom_mode = i;
        publisher.working.altitude_relative = mp_units::Metres(f64::from(i));
        let started = Instant::now();
        publisher.publish();
        publishes.push(started.elapsed());
        next += Duration::from_millis(1);
        if let Some(wait) = next.checked_duration_since(Instant::now()) {
            std::thread::sleep(wait);
        }
    }
    let allocated = publisher.allocations() - allocations_before;
    stop.store(true, Ordering::Relaxed);

    let mut loads = Vec::new();
    for reader in readers {
        let (times, seen) = reader.join().unwrap();
        black_box(seen);
        loads.extend(times);
    }
    let publish_p99 = p99(&mut publishes);
    let load_p99 = p99(&mut loads);
    println!(
        "snapshot gate: publish p99 {publish_p99:?} (budget {PUBLISH_BUDGET:?}), load p99 \
         {load_p99:?} over {} loads by {READERS} readers (budget {LOAD_BUDGET:?}), {allocated} \
         allocations in {PUBLISHES} publishes",
        loads.len()
    );
    assert!(
        publish_p99 <= PUBLISH_BUDGET,
        "publishing a snapshot takes {publish_p99:?} at p99, over §8.2's {PUBLISH_BUDGET:?}"
    );
    assert!(
        load_p99 <= LOAD_BUDGET,
        "loading a snapshot takes {load_p99:?} at p99 under {READERS} readers, over {LOAD_BUDGET:?}"
    );
    // Eight readers can hold eight pooled slots at one instant, so at most that many publishes
    // in a thousand may have had to allocate; a steady state of publish-and-read allocates none.
    assert!(
        allocated <= READERS as u64,
        "{allocated} allocations in {PUBLISHES} publishes: the pool is not being reused"
    );
}

criterion_group!(benches, snapshot);

fn main() {
    benches();
    Criterion::default().configure_from_args().final_summary();
    gate();
}
