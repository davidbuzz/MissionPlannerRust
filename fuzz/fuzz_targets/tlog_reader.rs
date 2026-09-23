//! Fuzzes the telemetry log reader.
//!
//! Logs get truncated by crashes, concatenated by accident and copied off failing SD cards. The
//! reader must terminate on any input and never hand back a record that points outside the buffer.
//!
//! The property itself lives in `mp_fuzz_checks`, so that `cargo test --workspace` compiles and
//! exercises it on stable. This file is only the libfuzzer entry point.

#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| mp_fuzz_checks::tlog_reader(data));
