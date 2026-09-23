//! Proves the decode path allocates nothing.
//!
//! D2 claims "zero heap allocations per packet". That claim is worthless unless a test fails when
//! it stops being true, so this binary installs a counting allocator and asserts an exact zero,
//! first over a synthetic stream and then over every frame of every real flight in
//! `testdata/mavlink`.
//!
//! "The decode path" means exactly what the link thread runs per packet
//! (`mp-link/src/lib.rs`, `run_link`): bytes from a read go into
//! [`FrameDecoder::push_and_drain`], which frames and checksums them with [`mp_mavlink::parse`]
//! against the generated dialect table, and each frame it hands out is turned into a typed message
//! by [`MavMessage::decode`]. Nothing is stubbed: the dialect is the generated one, not a test
//! table, and the bytes are the recorded ones, timestamps and all.
//!
//! Counting is per thread, switched on only around the measured work. The test harness runs tests
//! on several threads at once, and a process-wide counter would charge one test with another's
//! setup.

// A global allocator is an `unsafe impl` by definition: `GlobalAlloc`'s contract cannot be
// stated in safe Rust. This is a test binary; every `src/` in the workspace stays free of `unsafe`.
#![allow(unsafe_code)]
// Test code deliberately uses unwrap/expect/indexing: a panic here is a test failure with a
// useful message, which is exactly what we want. The production lint policy stays strict.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_possible_truncation)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::hint::black_box;
use std::path::Path;

mod support;

use mp_mavlink::{FrameDecoder, encode_v2};
use mp_mavlink_dialects::all::{DIALECT, MESSAGES, MavMessage};
use support::reference_dialect;

thread_local! {
    /// Whether this thread's allocations are being counted.
    static WATCHING: Cell<bool> = const { Cell::new(false) };
    /// `alloc`, `alloc_zeroed` and `realloc` calls made on this thread while watching. A `realloc`
    /// counts because growing a buffer is heap traffic on the hot path just as a fresh one is.
    static ALLOCATIONS: Cell<u64> = const { Cell::new(0) };
    /// `dealloc` calls made on this thread while watching. Zero allocations with non-zero frees
    /// would mean the steady state is releasing something the warm-up built, which is worth
    /// knowing even though it is not an allocation.
    static FREES: Cell<u64> = const { Cell::new(0) };
}

/// Bumps a counter if this thread is watching.
///
/// `try_with` rather than `with`, because an allocator can run during thread teardown and must
/// never panic. The cells are const-initialised and have no destructor, so touching them needs no
/// lazy setup and cannot recurse into the allocator.
fn note(counter: &'static std::thread::LocalKey<Cell<u64>>) {
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

/// A counter that never counts would pass every test below, so first prove it sees the heap.
#[test]
fn the_counter_sees_allocations_and_frees() {
    let (boxed, heap) = watch(|| black_box(Box::new(7u64)));
    assert_eq!(heap.allocations, 1, "one Box is one allocation");
    let ((), heap) = watch(|| drop(black_box(boxed)));
    assert_eq!(heap.frees, 1, "dropping it is one free");
    let (grown, heap) = watch(|| {
        let mut v: Vec<u8> = Vec::with_capacity(1);
        v.extend_from_slice(black_box(&[0u8; 64]));
        v
    });
    assert_eq!(heap.allocations, 2, "growing a Vec is a second allocation");
    drop(grown);
}

#[test]
fn decoding_ten_thousand_frames_allocates_nothing() {
    // Setup is allowed to allocate; only the measured window must not.
    let dialect = reference_dialect();
    let info = dialect.row("GLOBAL_POSITION_INT");
    let payload = [7u8; 28];
    let mut frame = [0u8; 64];
    let n = encode_v2(&mut frame, 1, 1, 1, info.id, &payload, info.crc_extra, 0).expect("encode");

    let mut stream = Vec::with_capacity(n * 64);
    for seq in 0..64u8 {
        let mut one = frame;
        one[4] = seq;
        let end = n - 2;
        let ck = mp_mavlink::crc::checksum(&one[1..end], info.crc_extra);
        one[end..end + 2].copy_from_slice(&ck.to_le_bytes());
        stream.extend_from_slice(&one[..n]);
    }

    let mut decoder = FrameDecoder::new();
    let mut checksum_acc = 0u64;

    let ((), heap) = watch(|| {
        for _ in 0..157 {
            for chunk in stream.chunks(73) {
                decoder.push_and_drain(chunk, &dialect, |f| {
                    checksum_acc = checksum_acc.wrapping_add(u64::from(f.payload_byte(0)));
                });
            }
        }
    });

    assert!(
        decoder.stats().frames >= 10_000,
        "expected a large run, got {}",
        decoder.stats().frames
    );
    assert_eq!(heap.allocations, 0, "decode path must not allocate");
    assert!(checksum_acc > 0, "work must not be optimised away");
}

/// Every `.tlog` in `testdata/mavlink`, by name, in a stable order.
///
/// Discovered rather than listed, so a flight added to the corpus is covered without anyone
/// remembering to add it here.
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

/// What one replay saw. Plain counters, so keeping them cannot allocate.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Replay {
    frames: u64,
    decoded: u64,
    /// Allocations made inside `MavMessage::decode`, as opposed to in the framer.
    decode_allocations: u64,
}

/// Feeds a recording through the decoder exactly as a link does, `chunk` bytes per read.
///
/// `decode_allocations_by_msgid` and `frames_by_msgid` are indexed by message id and sized by the
/// caller before the measured window, so attributing an allocation to a message type does not
/// itself allocate.
fn replay(
    bytes: &[u8],
    chunk: usize,
    frames_by_msgid: &mut [u64],
    decode_allocations_by_msgid: &mut [u64],
) -> Replay {
    let mut decoder = FrameDecoder::new();
    let mut seen = Replay::default();
    let mut on_frame = |frame: &mp_mavlink::Frame<'_>| {
        seen.frames += 1;
        let slot = frame.msgid as usize;
        frames_by_msgid[slot] += 1;
        // Everything the link thread reads from a frame, so none of it can be optimised out.
        black_box((frame.sysid, frame.compid, frame.seq, frame.raw));

        let before = allocations_so_far();
        let message = MavMessage::decode(frame.msgid, frame.payload);
        let cost = allocations_so_far() - before;
        seen.decode_allocations += cost;
        decode_allocations_by_msgid[slot] += cost;

        if black_box(message).is_some() {
            seen.decoded += 1;
        }
    };
    for piece in bytes.chunks(chunk) {
        decoder.push_and_drain(piece, &DIALECT, &mut on_frame);
    }
    // End of file: give up on any partial frame, as the link does when a replay runs out.
    decoder.flush(&DIALECT, &mut on_frame);
    assert_eq!(
        decoder.stats().overflow_bytes,
        0,
        "push_and_drain must never drop input"
    );
    seen
}

/// A table indexed by every message id the dialect can frame.
fn per_msgid_table() -> Vec<u64> {
    let highest = MESSAGES
        .iter()
        .map(|m| m.id)
        .max()
        .expect("dialect has messages");
    vec![0; highest as usize + 1]
}

#[test]
fn replaying_every_fixture_tlog_allocates_nothing_per_frame() {
    // Read sizes that matter: a byte at a time is a slow serial port and drives the partial-frame
    // path hardest; 512 is `ReplayTransport::DEFAULT_CHUNK`, what a `file:` link reads; 4096 is
    // the link thread's own read buffer (`run_link`'s `buf`), what a fast socket fills.
    const CHUNKS: [usize; 3] = [1, 512, 4096];

    let tlogs = fixture_tlogs();
    let mut frames_checked = 0u64;
    let mut frames_by_msgid = per_msgid_table();
    let mut decode_allocations_by_msgid = per_msgid_table();

    for (name, bytes) in &tlogs {
        let mut frames_at_each_chunk_size = Vec::new();
        for chunk in CHUNKS {
            // Warm-up. The decode path keeps no state that could grow - the decoder's buffer is
            // inline and the dialect is a static table - so this should change nothing; it is here
            // so that a future cache on this path is given its chance to fill before the claim is
            // judged, rather than failing the first time it is touched.
            let warm = replay(bytes, chunk, &mut per_msgid_table(), &mut per_msgid_table());

            frames_by_msgid.fill(0);
            decode_allocations_by_msgid.fill(0);
            let (measured, heap) = watch(|| {
                replay(
                    bytes,
                    chunk,
                    &mut frames_by_msgid,
                    &mut decode_allocations_by_msgid,
                )
            });

            let decode_sites: Vec<(u32, u64, u64)> = decode_allocations_by_msgid
                .iter()
                .enumerate()
                .filter(|(_, n)| **n > 0)
                .map(|(id, n)| (id as u32, *n, frames_by_msgid[id]))
                .collect();
            assert_eq!(
                heap.allocations,
                0,
                "{name}, {chunk}-byte reads: {} allocation(s) over {} frames - {} in the framer \
                 (FrameDecoder::push_and_drain / parse), {} in MavMessage::decode; \
                 (msgid, allocations, frames) for decode: {decode_sites:?}",
                heap.allocations,
                measured.frames,
                heap.allocations - measured.decode_allocations,
                measured.decode_allocations,
            );
            assert_eq!(
                heap.frees, 0,
                "{name}, {chunk}-byte reads: the decode path freed memory while measured"
            );
            assert_eq!(
                measured, warm,
                "{name}: a second replay must see exactly what the first did"
            );
            assert_eq!(
                measured.decoded, measured.frames,
                "{name}: every frame the generated dialect can checksum, it can also decode"
            );
            let kinds = frames_by_msgid.iter().filter(|n| **n > 0).count();
            assert!(
                kinds >= 30,
                "{name}: only {kinds} message types decoded; the fixture is not a real flight"
            );

            frames_at_each_chunk_size.push(measured.frames);
            frames_checked += measured.frames;
        }

        // How a read happens to split the stream must not change which frames come out of it.
        assert!(
            frames_at_each_chunk_size.windows(2).all(|w| w[0] == w[1]),
            "{name}: frames per read size {CHUNKS:?} = {frames_at_each_chunk_size:?}"
        );
        eprintln!(
            "{name}: {} frames at each of {CHUNKS:?}-byte reads, 0 allocations",
            frames_at_each_chunk_size[0]
        );
        // Each fixture is a real flight of tens of thousands of frames. A floor this high means a
        // truncated or emptied fixture fails here instead of passing vacuously.
        assert!(
            frames_at_each_chunk_size[0] >= 10_000,
            "{name}: only {} frames; is the fixture truncated?",
            frames_at_each_chunk_size[0]
        );
    }

    assert!(
        frames_checked >= 10_000 * (tlogs.len() * CHUNKS.len()) as u64,
        "checked only {frames_checked} frames"
    );
    eprintln!(
        "decode path: {frames_checked} frames from {} tlog(s) at {} read sizes, 0 allocations",
        tlogs.len(),
        CHUNKS.len()
    );
}
