//! Proves the decode path allocates nothing.
//!
//! D2 claims "zero allocations per packet". That claim is worthless unless a test fails when it
//! stops being true, so this binary installs a counting allocator and asserts an exact zero over
//! a large decode run. Kept in its own test binary so no other test's allocations pollute the
//! counter.

// Justification: a counting global allocator cannot be written in safe Rust.
#![allow(unsafe_code)]
// Test code deliberately uses unwrap/expect/indexing: a panic here is a test failure with a
// useful message, which is exactly what we want. The production lint policy stays strict.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_possible_truncation)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

mod support;

use mp_mavlink::{FrameDecoder, encode_v2};
use support::reference_dialect;

static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);

struct CountingAllocator;

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

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

    let before = ALLOCATIONS.load(Ordering::Relaxed);
    for _ in 0..157 {
        for chunk in stream.chunks(73) {
            decoder.push_and_drain(chunk, &dialect, |f| {
                checksum_acc = checksum_acc.wrapping_add(u64::from(f.payload_byte(0)));
            });
        }
    }
    let after = ALLOCATIONS.load(Ordering::Relaxed);

    assert!(
        decoder.stats().frames >= 10_000,
        "expected a large run, got {}",
        decoder.stats().frames
    );
    assert_eq!(after - before, 0, "decode path must not allocate");
    assert!(checksum_acc > 0, "work must not be optimised away");
}
