//! Fuzzes the log analyzer over a `.log`'s text, whose Python skips what it cannot read and whose XML the C#'s reader must then read.
//!
//! The property itself lives in `mp_fuzz_checks`, so that `cargo test --workspace` compiles and
//! exercises it on stable. This file is only the libfuzzer entry point.

#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| mp_fuzz_checks::loganalyzer_text(data));
