//! Fuzzes single-frame parsing.
//!
//! The property: `parse` must never panic, never loop, and never report consuming more bytes than
//! it was given. Every byte on a telemetry link is attacker-influenced in the sense that matters -
//! radio noise, a misconfigured peer, a corrupted log - and a parser that panics takes the ground
//! station down mid-flight.
//!
//! The property itself lives in `mp_fuzz_checks`, so that `cargo test --workspace` compiles and
//! exercises it on stable. This file is only the libfuzzer entry point.

#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| mp_fuzz_checks::frame_parse(data));
