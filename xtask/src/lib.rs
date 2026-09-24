//! Repository automation as a library, so its tests can call the generators and the ledger
//! directly; `cargo xtask <command>` is the binary over it (`main.rs`).

// The modules' items are `pub` for readability inside their trees, not as an exported API: only
// `codegen` and `ledger` themselves are reached from the tests.
#![allow(unreachable_pub)]

pub mod codegen;
pub mod ledger;
