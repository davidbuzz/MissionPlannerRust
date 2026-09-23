//! Fuzzes decoding a frame into a message.
//!
//! The gap `frame_parse` leaves: that target stops at the framing layer and never calls a
//! per-message decoder, which is where a payload is read at fixed offsets and where a truncated
//! MAVLink v2 frame has to be treated as zero-padded rather than indexed into.
//!
//! The property itself lives in `mp_fuzz_checks`, so that `cargo test --workspace` compiles and
//! exercises it on stable. This file is only the libfuzzer entry point.

#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| mp_fuzz_checks::message_decode(data));
