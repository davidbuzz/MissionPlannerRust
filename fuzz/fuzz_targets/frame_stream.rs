//! Fuzzes the streaming decoder.
//!
//! Beyond not panicking, the decoder must not wedge: a stream of arbitrary bytes has to keep
//! making progress rather than filling its buffer and stalling. That is a liveness property a
//! single-frame fuzzer cannot see, and a wedged decoder looks exactly like a dead link.

#![no_main]

use libfuzzer_sys::fuzz_target;
use mp_mavlink::{FrameDecoder, MAX_FRAME_LEN};
use mp_mavlink_dialects::all::DIALECT;

fuzz_target!(|data: &[u8]| {
    let mut decoder = FrameDecoder::new();
    let mut frames = 0u64;

    // Feed in irregular chunks, which is what a real transport does.
    let mut offset = 0usize;
    let mut step = 1usize;
    while offset < data.len() {
        let end = (offset + step).min(data.len());
        let consumed = decoder.push_and_drain(&data[offset..end], &DIALECT, |_| frames += 1);
        assert_eq!(consumed, end - offset, "push_and_drain must consume everything it is given");
        offset = end;
        step = (step * 3 % 97) + 1;
    }
    decoder.flush(&DIALECT, |_| frames += 1);

    // After a flush the decoder must not be holding a frame's worth of undecidable bytes.
    assert!(decoder.buffered() < MAX_FRAME_LEN, "decoder wedged with {} bytes", decoder.buffered());
});
