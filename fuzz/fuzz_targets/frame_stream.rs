//! Fuzzes the streaming decoder.
//!
//! Beyond not panicking, the decoder must not wedge: a stream of arbitrary bytes has to keep
//! making progress rather than filling its buffer and stalling. That is a liveness property a
//! single-frame fuzzer cannot see, and a wedged decoder looks exactly like a dead link.
//!
//! The property itself lives in `mp_fuzz_checks`, so that `cargo test --workspace` compiles and
//! exercises it on stable. This file is only the libfuzzer entry point.

#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| mp_fuzz_checks::frame_stream(data));
