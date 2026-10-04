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

//! Proves the ingest→state path allocates nothing per packet once warm (DELIVERABLES.md Deliverable 5).
//!
//! The path is the one the link thread runs (`mp-link/src/lib.rs`, `run_link`), stage for stage:
//! a transport read, [`FrameDecoder::push_and_drain`], [`MavMessage::decode`], then
//! [`VehicleRegistry::apply`] with the frame's system, component and sequence; on the publish
//! cadence [`VehicleRegistry::publish_all`], then [`Transport::description`] compared with the
//! text the link last showed, as the link does to notice a UDP link learning its peer; and on the
//! reading side [`StateHandle::load`], which is what a UI frame calls. The transport is the
//! production `file:` one, [`ReplayTransport`], over every recorded flight in `testdata/mavlink`.
//! What a reader sees at the end is checked against the working state, so the test cannot pass by
//! publishing nothing.
//!
//! # Steady state
//!
//! Each recording is replayed three times through the same decoder, registry and readers, and only
//! the third replay is measured. The first discovers the vehicles, so the UI-side handles can be
//! taken; the second runs with those readers attached. Between them they are allowed to allocate,
//! and do, for exactly these:
//!
//! * one `StatePublisher` per sender in the registry's map, created the first time a system/
//!   component pair is heard from;
//! * the snapshot `Arc`s each publisher recycles - two or three per vehicle in practice, capped
//!   at its pool size;
//! * arc-swap's per-thread bookkeeping for the thread that publishes and reads.
//!
//! Nothing else on this path owns heap memory: `VehicleState` is `Copy` and holds no `String` or
//! `Vec`, and there are no parameter tables or per-message caches here (those live in `mp-link`,
//! and `mp-link/tests/no_alloc_ingest.rs` measures them). So after those two replays nothing is
//! left to grow, and the third must be exactly zero. No message type is excluded.
//!
//! The second warm-up is not decoration. Without readers in it, the measured replay allocated one
//! `Arc` (mp-vehicle/src/snapshot.rs:62, `Arc::new` in `StatePublisher::publish`) the first time a
//! publish found both pooled snapshots busy - one in the cell, one still held by a reader. That is
//! the pool growing to fit how it is read, once, not a cost per packet.

// A global allocator is an `unsafe impl` by definition: `GlobalAlloc`'s contract cannot be
// stated in safe Rust. This is a test binary; `mp-vehicle`'s own source forbids `unsafe`.
#![allow(unsafe_code)]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_possible_truncation)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::hint::black_box;
use std::path::Path;
use std::sync::Arc;

use mp_mavlink::FrameDecoder;
use mp_mavlink_dialects::all::{DIALECT, MESSAGES, MavMessage};
use mp_transport::{ReplayTransport, Transport};
use mp_vehicle::{StateHandle, VehicleRegistry, VehicleState};

thread_local! {
    /// Whether this thread's allocations are being counted.
    static WATCHING: Cell<bool> = const { Cell::new(false) };
    /// `alloc`, `alloc_zeroed` and `realloc` calls made on this thread while watching.
    static ALLOCATIONS: Cell<u64> = const { Cell::new(0) };
    /// `dealloc` calls made on this thread while watching.
    static FREES: Cell<u64> = const { Cell::new(0) };
}

/// Bumps a counter if this thread is watching.
///
/// `try_with` rather than `with`, because an allocator can run during thread teardown and must
/// never panic. The cells are const-initialised and have no destructor, so touching them needs no
/// lazy setup and cannot recurse into the allocator.
fn note(counter: &'static wasm_thread::LocalKey<Cell<u64>>) {
    let watching = WATCHING.try_with(Cell::get).unwrap_or(false);
    if watching {
        let _ = counter.try_with(|n| n.set(n.get() + 1));
    }
}

struct CountingAllocator;

// SAFETY: every method forwards to `System` with its arguments unchanged, so this allocator makes
// exactly the guarantees `System` makes. The only addition is a thread-local counter increment,
// which neither allocates nor unwinds.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        note(&ALLOCATIONS);
        // SAFETY: the caller upholds `GlobalAlloc::alloc`'s contract, which is all `System` needs.
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        note(&ALLOCATIONS);
        // SAFETY: as for `alloc`; the contract is the same.
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        note(&FREES);
        // SAFETY: `ptr` was returned by this allocator, which means by `System`, for `layout`.
        unsafe { System.dealloc(ptr, layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        note(&ALLOCATIONS);
        // SAFETY: `ptr` and `layout` came from `System` via this allocator, and the caller upholds
        // `realloc`'s contract for `new_size`.
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

/// Heap traffic observed on this thread while watching.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Heap {
    allocations: u64,
    frees: u64,
}

fn allocations_so_far() -> u64 {
    ALLOCATIONS.with(Cell::get)
}

/// Runs `work` with counting switched on for this thread and reports what it did to the heap.
fn watch<R>(work: impl FnOnce() -> R) -> (R, Heap) {
    let allocations = ALLOCATIONS.with(Cell::get);
    let frees = FREES.with(Cell::get);
    WATCHING.with(|w| w.set(true));
    let result = work();
    WATCHING.with(|w| w.set(false));
    let heap = Heap {
        allocations: ALLOCATIONS.with(Cell::get) - allocations,
        frees: FREES.with(Cell::get) - frees,
    };
    (result, heap)
}

/// A counter that never counts would pass the test below, so first prove it sees the heap.
#[test]
fn the_counter_sees_allocations() {
    let (boxed, heap) = watch(|| black_box(Box::new(7u64)));
    assert_eq!(heap.allocations, 1, "one Box is one allocation");
    let ((), heap) = watch(|| drop(black_box(boxed)));
    assert_eq!(heap.frees, 1, "dropping it is one free");
}

/// Every `.tlog` in `testdata/mavlink`, by name, in a stable order.
fn fixture_tlogs() -> Vec<(String, Vec<u8>)> {
    let dir = Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink"
    ));
    let mut out: Vec<(String, Vec<u8>)> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("reading {}: {e}", dir.display()))
        .map(|entry| entry.expect("directory entry").path())
        .filter(|path| path.extension().is_some_and(|ext| ext == "tlog"))
        .map(|path| {
            let name = path.file_name().unwrap().to_string_lossy().into_owned();
            let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("reading {name}: {e}"));
            (name, bytes)
        })
        .collect();
    out.sort();
    assert!(
        !out.is_empty(),
        "no .tlog fixtures in {}: an empty corpus would prove nothing",
        dir.display()
    );
    out
}

/// Where a replay's allocations happened. Plain counters, so keeping them cannot allocate.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Pass {
    frames: u64,
    publishes: u64,
    /// Times the simulated UI drew a frame, reading every vehicle.
    ui_frames: u64,
    snapshot_loads: u64,
    in_decode: u64,
    in_apply: u64,
    in_publish: u64,
    /// Asking the transport for its description on each publish, and keeping a changed one.
    in_describe: u64,
    in_load: u64,
}

/// Everything the link thread and a reader do with one recording, one read at a time.
///
/// `readers` are handles to every vehicle, as the UI holds them. The simulated UI draws on every
/// other publish - a display slower than the link, which is the usual case - and keeps the first
/// vehicle's snapshot until its next frame, the way a render pass holds the state it started with.
/// That is the pattern that asks the most of the publisher's pool: one snapshot in the cell, one
/// held by the reader from two publishes ago, and a third to write into.
///
/// `apply_allocations_by_msgid` is indexed by message id and sized before the measured window, so
/// attributing an allocation to a message type does not itself allocate. `shown` is the
/// description the link last published for the UI.
fn ingest(
    transport: &mut ReplayTransport,
    decoder: &mut FrameDecoder,
    registry: &mut VehicleRegistry,
    shown: &mut String,
    readers: &[StateHandle],
    held: &mut Option<Arc<VehicleState>>,
    apply_allocations_by_msgid: &mut [u64],
) -> Pass {
    let mut pass = Pass::default();
    // The link thread's own read buffer size.
    let mut buf = [0u8; 4096];
    loop {
        let n = transport.read(&mut buf).expect("replay read");
        if n == 0 {
            break;
        }
        decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
            pass.frames += 1;
            let before = allocations_so_far();
            let message = MavMessage::decode(frame.msgid, frame.payload);
            let decoded = allocations_so_far();
            if let Some(message) = message {
                registry.apply(frame.sysid, frame.compid, frame.seq, &message);
            }
            let applied = allocations_so_far();
            pass.in_decode += decoded - before;
            pass.in_apply += applied - decoded;
            apply_allocations_by_msgid[frame.msgid as usize] += applied - decoded;
        });

        // The link publishes on a 20 ms cadence; publishing after every read is several times
        // more often than that on a real link, so this is the harder case, not a gentler one.
        let before = allocations_so_far();
        registry.publish_all();
        pass.in_publish += allocations_so_far() - before;
        pass.publishes += 1;

        // With each publish the link asks the transport how it describes itself, and keeps the
        // text only if it changed (`run_link`, mp-link/src/lib.rs). The text is lent, so asking
        // must cost nothing.
        let before = allocations_so_far();
        let current = transport.description();
        if shown.as_str() != current {
            shown.clear();
            shown.push_str(current);
        }
        pass.in_describe += allocations_so_far() - before;

        if pass.publishes % 2 == 1 {
            continue;
        }
        pass.ui_frames += 1;
        let before = allocations_so_far();
        for (i, reader) in readers.iter().enumerate() {
            let snapshot = reader.load();
            black_box(&*snapshot);
            pass.snapshot_loads += 1;
            if i == 0 {
                // Replacing the held snapshot drops the previous one: a reference count, not a
                // free, because the publisher's pool still owns the allocation.
                *held = Some(snapshot);
            }
        }
        pass.in_load += allocations_so_far() - before;
    }
    // End of the recording, handled as the link thread handles its own end: give up on a
    // partial frame, then publish what is left.
    decoder.flush(&DIALECT, |frame| {
        pass.frames += 1;
        if let Some(message) = MavMessage::decode(frame.msgid, frame.payload) {
            registry.apply(frame.sysid, frame.compid, frame.seq, &message);
        }
    });
    registry.publish_all();
    pass.publishes += 1;
    pass
}

#[test]
fn replaying_every_fixture_tlog_into_vehicle_state_allocates_nothing_per_packet() {
    let highest = MESSAGES
        .iter()
        .map(|m| m.id)
        .max()
        .expect("dialect has messages");
    let mut frames_checked = 0u64;

    for (name, bytes) in fixture_tlogs() {
        // Every transport is built before anything is measured: loading a file allocates, and is
        // not part of handling a packet.
        let mut warm_up = ReplayTransport::from_bytes(name.clone(), bytes.clone());
        let mut warm_readers = ReplayTransport::from_bytes(name.clone(), bytes.clone());
        let mut measured = ReplayTransport::from_bytes(name.clone(), bytes);
        let mut decoder = FrameDecoder::new();
        let mut registry = VehicleRegistry::new();
        let mut held = None;
        let mut by_msgid = vec![0u64; highest as usize + 1];
        // What the link shows for the UI; the first publish fills it, as the link's connect does.
        let mut shown = String::new();

        let first = ingest(
            &mut warm_up,
            &mut decoder,
            &mut registry,
            &mut shown,
            &[],
            &mut held,
            &mut by_msgid,
        );

        // A UI holds a handle per vehicle it shows; collecting them is setup, not ingest.
        let ids = registry.ids();
        let readers: Vec<StateHandle> = ids
            .iter()
            .map(|id| registry.handle(*id).expect("handle for a known vehicle"))
            .collect();
        // The readers get a replay of their own before measuring starts: a held snapshot can make
        // a publisher reach for one more pooled `Arc` than it needed with nobody reading, and that
        // is growth, not per-packet cost.
        let _ = ingest(
            &mut warm_readers,
            &mut decoder,
            &mut registry,
            &mut shown,
            &readers,
            &mut held,
            &mut by_msgid,
        );

        by_msgid.fill(0);
        let (pass, heap) = watch(|| {
            ingest(
                &mut measured,
                &mut decoder,
                &mut registry,
                &mut shown,
                &readers,
                &mut held,
                &mut by_msgid,
            )
        });

        let offenders: Vec<(usize, u64)> = by_msgid
            .iter()
            .enumerate()
            .filter(|(_, n)| **n > 0)
            .map(|(id, n)| (id, *n))
            .collect();
        assert_eq!(
            heap.allocations, 0,
            "{name}: {} allocation(s) over {} frames and {} publishes: {pass:?}; \
             (msgid, allocations) inside VehicleRegistry::apply: {offenders:?}. The rest were in \
             the transport read or the framer.",
            heap.allocations, pass.frames, pass.publishes,
        );
        assert_eq!(
            heap.frees, 0,
            "{name}: the ingest path freed memory in steady state: {pass:?}"
        );
        assert_eq!(
            shown,
            measured.description(),
            "{name}: what the UI is shown is not the transport's description"
        );

        // The replay is identical each time, so the two must agree on what they saw. Anything else
        // means the measured pass did different work from the one that warmed it up.
        assert_eq!(
            pass.frames, first.frames,
            "{name}: frames differ between passes"
        );
        assert!(
            pass.frames >= 10_000,
            "{name}: only {} frames; is the fixture truncated?",
            pass.frames
        );
        assert!(
            pass.publishes >= 1_000,
            "{name}: only {} publishes",
            pass.publishes
        );
        assert!(!readers.is_empty(), "{name}: no vehicle was heard from");
        assert!(
            pass.ui_frames >= 500,
            "{name}: only {} UI frames",
            pass.ui_frames
        );
        assert_eq!(
            pass.snapshot_loads,
            pass.ui_frames * readers.len() as u64,
            "{name}: every UI frame must read every vehicle"
        );

        // What a reader sees at the end is what the link thread built. Compared on the message
        // counter rather than the whole state, because the state legitimately holds NaN (an
        // unreported VDOP) and NaN is unequal to itself.
        for (id, reader) in ids.iter().zip(&readers) {
            let seen = reader.load();
            let working = registry.working(*id).expect("working state");
            assert!(working.messages_applied > 0, "{name}: {id} applied nothing");
            assert_eq!(
                seen.messages_applied, working.messages_applied,
                "{name}: the snapshot {id}'s reader sees is not the one the link built"
            );
        }

        eprintln!(
            "{name}: {} frames, {} publishes, {} UI frames reading {} snapshot(s), {} vehicle(s), \
             0 allocations",
            pass.frames,
            pass.publishes,
            pass.ui_frames,
            pass.snapshot_loads,
            readers.len()
        );
        frames_checked += pass.frames;
    }

    assert!(
        frames_checked >= 20_000,
        "checked only {frames_checked} frames"
    );
}
