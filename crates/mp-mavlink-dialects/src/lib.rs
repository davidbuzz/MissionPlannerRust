//! Generated MAVLink message types, metadata and enums.
//!
//! The contents of `generated/` are produced by `cargo xtask codegen mavlink` from
//! `references/missionplanner/ExtLibs/Mavlink/message_definitions/*.xml` - the same definitions
//! the C# build uses. They are checked in so building needs no Python, no network and no
//! reference tree.
//!
//! Correctness of the generator is not assumed: `cargo xtask verify-mavlink` checks every
//! message's `CRC_EXTRA`, `min_len` and `len` against the table dumped from the shipping
//! `MAVLink.dll`, and the test suite decodes real flight logs and compares field values.

/// The `all` dialect: every message Mission Planner knows about.
#[path = "generated/all.rs"]
pub mod all;

pub use all::{DIALECT, MESSAGES, MavMessage};
