// Copyright (C) 2026 David "Buzz" Bussenschutt
//
// This file is part of MissionPlannerRust, a Rust implementation derived from
// Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,
// https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.
//
// MissionPlannerRust is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, version 3 of the License.
//
// MissionPlannerRust is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// MissionPlannerRust. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-only

//! The checked-in generated code is what the generators produce today (DELIVERABLES.md Deliverable 18).
//!
//! `cargo xtask codegen-modes`, `codegen-param-meta` and `codegen all` write their output into
//! the crates and the output is committed, so a plain build needs no reference tree. Nothing but
//! CI's `codegen all --check` proved the committed files current, and nothing proved the mode
//! table or the parameter metadata current at all: a generator changed without its output being
//! regenerated would ship stale code. Each test here regenerates in memory, formats the result as
//! the xtask does, and compares it byte for byte with the committed file. The reference tree is
//! git-excluded, so without it the tests say so and pass, as `resx.rs` does.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::path::{Path, PathBuf};
use std::process::Command;

use xtask::codegen;

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("..")
}

fn tree() -> Option<PathBuf> {
    let tree = xtask::upstream::tree();
    if tree.is_none() {
        println!("{}; the regeneration is skipped", xtask::upstream::absent());
    }
    tree
}

/// The generated source as the xtask leaves it on disk: written and run through
/// `rustfmt --edition 2024`, which the generators call on their output.
fn formatted(name: &str, source: &str) -> String {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("codegen");
    std::fs::create_dir_all(&dir).unwrap();
    let file = dir.join(name);
    std::fs::write(&file, source).unwrap();
    let status = Command::new("rustfmt")
        .args(["--edition", "2024"])
        .arg(&file)
        .status()
        .expect("rustfmt runs (it is a component of the pinned toolchain)");
    assert!(status.success(), "rustfmt failed on the generated {name}");
    std::fs::read_to_string(&file).unwrap()
}

/// Fails naming the first differing line, so a stale file is read off the failure rather than
/// a two-megabyte diff.
fn assert_current(committed_path: &Path, fresh: &str, command: &str) {
    let committed = std::fs::read_to_string(committed_path)
        .unwrap_or_else(|e| panic!("reading {}: {e}", committed_path.display()));
    if committed == fresh {
        return;
    }
    let first_difference = committed
        .lines()
        .zip(fresh.lines())
        .position(|(a, b)| a != b)
        .unwrap_or(committed.lines().count().min(fresh.lines().count()))
        + 1;
    panic!(
        "{} is out of date: it first differs from the regenerated source at line \
         {first_difference} (committed {} lines, regenerated {}); run `cargo xtask {command}`",
        committed_path.display(),
        committed.lines().count(),
        fresh.lines().count()
    );
}

/// `crates/mp-vehicle/src/generated/modes.rs` is `codegen::modes::generate` over the parameter
/// metadata backup, formatted.
#[test]
fn flight_mode_table_is_current() {
    let Some(tree) = tree() else { return };
    let source = codegen::modes::generate(&tree.join("ParameterMetaDataBackup.xml")).unwrap();
    assert_current(
        &repo().join("crates/mp-vehicle/src/generated/modes.rs"),
        &formatted("modes.rs", &source),
        "codegen-modes",
    );
}

/// `crates/mp-params/src/generated/param_meta_copter.rs` is `codegen::param_meta::generate` for
/// the `ArduCopter2` section, formatted.
#[test]
fn copter_parameter_metadata_is_current() {
    let Some(tree) = tree() else { return };
    let source =
        codegen::param_meta::generate(&tree.join("ParameterMetaDataBackup.xml"), "ArduCopter2")
            .unwrap();
    assert_current(
        &repo().join("crates/mp-params/src/generated/param_meta_copter.rs"),
        &formatted("param_meta_copter.rs", &source),
        "codegen-param-meta",
    );
}

/// `crates/mp-mavlink-dialects/src/generated/all.rs` is the `all` dialect parsed and emitted,
/// formatted - what CI's `codegen all --check` proves, here under `cargo test` too.
#[test]
fn all_dialect_is_current() {
    let Some(tree) = tree() else { return };
    let xml = tree.join("ExtLibs/Mavlink/message_definitions/all.xml");
    let parsed = codegen::mavlink::parse_dialect(&xml).unwrap();
    let source = codegen::emit::emit_dialect(&parsed);
    assert_current(
        &repo().join("crates/mp-mavlink-dialects/src/generated/all.rs"),
        &formatted("all.rs", &source),
        "codegen all",
    );
}

/// The comparison reports a stale file: the committed source with one line changed is named
/// at that line.
#[test]
fn a_changed_line_is_reported() {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("codegen");
    std::fs::create_dir_all(&dir).unwrap();
    let stale = dir.join("stale.rs");
    std::fs::write(&stale, "// one\n// two\n// three\n").unwrap();
    let result = std::panic::catch_unwind(|| {
        assert_current(&stale, "// one\n// 2\n// three\n", "codegen-modes");
    });
    let message = result
        .expect_err("a differing line fails the comparison")
        .downcast::<String>()
        .unwrap();
    assert!(
        message.contains("at line 2") && message.contains("cargo xtask codegen-modes"),
        "{message}"
    );
    assert_current(&stale, "// one\n// two\n// three\n", "codegen-modes");
}
