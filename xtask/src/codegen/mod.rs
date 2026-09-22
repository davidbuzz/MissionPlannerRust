//! Code generation from the upstream definition files (DELIVERABLES.md D18).
//!
//! Generated code is checked in so that a plain `cargo build` needs no Python, no network and no
//! reference tree. `cargo xtask codegen --check` regenerates into a temporary location and fails
//! if the result differs, which is what CI runs.

pub mod emit;
pub mod mavlink;
pub mod modes;
