//! Fuzzes the waypoint file reader.
//!
//! Mission files come from other tools, other people and other decades. The reader must reject or
//! accept, never panic - and anything it accepts must survive a write-read round trip, or the file
//! a user saves will not match the one they loaded.
//!
//! The property itself lives in `mp_fuzz_checks`, so that `cargo test --workspace` compiles and
//! exercises it on stable. This file is only the libfuzzer entry point.

#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| mp_fuzz_checks::waypoint_file(data));
