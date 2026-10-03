//! The licence record is in order (DELIVERABLES.md D1): every tracked Rust file opens with the
//! owner's header, the manifests declare GPL-3.0-only and nothing in the Rust claims a later
//! version, every crate the workspace is built from is under a licence deny.toml admits, the
//! crate table in `THIRD_PARTY_LICENSES` is what `cargo metadata` says today, and NOTICE and
//! `THIRD_PARTY_LICENSES` name what the work derives from and carries.

use std::path::{Path, PathBuf};

use xtask::licence::{self, CLARIFIED, HEADER, LICENCE};

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("..")
}

fn read(relative: &str) -> String {
    let path = repo().join(relative);
    std::fs::read_to_string(&path).unwrap_or_else(|error| panic!("{}: {error}", path.display()))
}

/// Every `.rs` file git tracks opens with the header.
#[test]
fn every_rust_file_opens_with_the_header() {
    let root = repo();
    let missing = licence::without_header(&root).unwrap();
    let names: Vec<String> = missing
        .iter()
        .map(|file| file.strip_prefix(&root).unwrap_or(file).display().to_string())
        .collect();
    assert!(
        names.is_empty(),
        "{} Rust file(s) without the licence header (prepend xtask::licence::HEADER and a blank line):\n{}",
        names.len(),
        names.join("\n")
    );
    assert!(
        licence::rust_files(&root).unwrap().len() > 500,
        "git ls-files found too few Rust files to be the repository"
    );
}

/// The header is the owner's form: his copyright, the product's name, the derivation with
/// Mission Planner's copyright, the GNU notice for version 3 alone, and the SPDX identifier.
#[test]
fn the_header_is_the_owners_form() {
    for line in [
        "Copyright (C) 2026 David \"Buzz\" Bussenschutt",
        "This file is part of MissionPlannerRust, a Rust implementation derived from",
        "Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,",
        "GNU General Public License as published by the",
        "Free Software Foundation, version 3 of the License.",
        "WITHOUT ANY WARRANTY",
        "<https://www.gnu.org/licenses/>",
        &format!("SPDX-License-Identifier: {LICENCE}"),
    ] {
        assert!(HEADER.contains(line), "the header lacks `{line}`");
    }
    assert!(
        !HEADER.contains("later version"),
        "the header must not offer a later version of the licence"
    );
    assert!(HEADER.lines().all(|line| line.starts_with("//")));
    assert!(HEADER.ends_with('\n') && !HEADER.ends_with("\n\n"));
}

/// A header is the file's only when it opens the file and a blank line follows it.
#[test]
fn a_header_is_recognised_only_at_the_top() {
    assert!(licence::has_header(&format!("{HEADER}\n//! A module.\n")));
    assert!(licence::has_header(&format!("{HEADER}\n")));
    assert!(!licence::has_header(&format!("//! A module.\n{HEADER}\n")));
    assert!(
        !licence::has_header(&format!("{HEADER}//! A module.\n")),
        "no blank line after the header"
    );
    assert!(!licence::has_header(""));
    assert!(!licence::has_header("// Copyright (C) 2026 someone else\n\n"));
}

/// The manifests declare GPL-3.0-only, LICENSE is the GNU GPL version 3, and no Rust file calls
/// the C# or this work GPL-3.0-only: Mission Planner's COPYING.txt grants version 3 alone.
#[test]
fn the_work_is_gpl_3_only() {
    for manifest in ["Cargo.toml", "fuzz/Cargo.toml"] {
        assert!(
            read(manifest).contains(&format!("license = \"{LICENCE}\"")),
            "{manifest} does not declare license = \"{LICENCE}\""
        );
    }
    let licence_text = read("LICENSE");
    assert!(licence_text.contains("GNU GENERAL PUBLIC LICENSE"));
    assert!(licence_text.contains("Version 3, 29 June 2007"));
    let later = format!("GPL-3.0-{}", "or-later");
    let root = repo();
    let claiming: Vec<String> = licence::rust_files(&root)
        .unwrap()
        .into_iter()
        .filter(|file| std::fs::read_to_string(file).is_ok_and(|text| text.contains(&later)))
        .map(|file| file.strip_prefix(&root).unwrap_or(&file).display().to_string())
        .collect();
    assert!(
        claiming.is_empty(),
        "Rust files naming {later}, which neither Mission Planner nor this work is:\n{}",
        claiming.join("\n")
    );
}

/// Every crate the workspace is built from is under a licence deny.toml admits, read as
/// cargo-deny reads the expression; the few whose manifest names only a file are clarified.
#[test]
fn every_dependency_is_allowed_by_deny_toml() {
    let root = repo();
    let allow = licence::deny_allow_list(&root).unwrap();
    assert!(allow.iter().any(|licence| licence == LICENCE), "deny.toml must admit {LICENCE}");
    let dependencies = licence::dependencies(&root).unwrap();
    assert!(dependencies.len() > 500, "{} dependencies", dependencies.len());
    let refused: Vec<String> = dependencies
        .iter()
        .filter(|dependency| !licence::allowed(&dependency.licence, &allow))
        .map(|dependency| {
            format!(
                "{} {}: {}",
                dependency.name, dependency.version, dependency.licence
            )
        })
        .collect();
    assert!(
        refused.is_empty(),
        "crates whose licence deny.toml does not admit:\n{}",
        refused.join("\n")
    );
    for (name, _) in CLARIFIED {
        assert!(
            dependencies.iter().any(|dependency| dependency.name == name),
            "{name} is clarified but no longer a dependency"
        );
    }
    assert!(
        dependencies
            .iter()
            .all(|dependency| !dependency.licence.starts_with("file:")),
        "a crate names a licence file without a clarification in CLARIFIED"
    );
}

/// SPDX expressions as cargo-deny reads them: `OR` a choice, `AND` both, parentheses, `WITH` an
/// exception bound to its licence, the old `/` an `OR`; malformed text admits nothing.
#[test]
fn spdx_expressions_are_read_as_cargo_deny_reads_them() {
    let allow: Vec<String> = ["MIT", "Apache-2.0 WITH LLVM-exception", "Unicode-DFS-2016"]
        .into_iter()
        .map(str::to_owned)
        .collect();
    assert!(licence::allowed("MIT", &allow));
    assert!(!licence::allowed("GPL-2.0-only", &allow));
    assert!(licence::allowed("MIT OR GPL-3.0-only", &allow));
    assert!(licence::allowed("GPL-3.0-only OR MIT", &allow));
    assert!(licence::allowed("MIT/Apache-2.0", &allow));
    assert!(!licence::allowed("MIT AND BSD-3-Clause", &allow));
    assert!(licence::allowed("(MIT OR Apache-2.0) AND Unicode-DFS-2016", &allow));
    assert!(!licence::allowed("(MIT OR Apache-2.0) AND NCSA", &allow));
    assert!(licence::allowed("Apache-2.0 WITH LLVM-exception", &allow));
    assert!(
        !licence::allowed("Apache-2.0", &allow),
        "the exception is part of what is allowed, not the bare licence"
    );
    assert!(licence::allowed("Apache-2.0 WITH LLVM-exception OR Apache-2.0 OR MIT", &allow));
    assert!(!licence::allowed("", &allow));
    assert!(!licence::allowed("(MIT", &allow));
    assert!(!licence::allowed("MIT OR", &allow));
    assert!(!licence::allowed("MIT WITH", &allow));
    assert!(!licence::allowed("MIT )", &allow));
}

/// `THIRD_PARTY_LICENSES`'s crate table is what `cargo metadata` says today.
#[test]
fn the_crate_table_is_current() {
    assert!(
        licence::update_third_party(&repo(), true).unwrap(),
        "{}'s crate table is stale; run `cargo xtask licences`",
        licence::THIRD_PARTY
    );
}

/// NOTICE says what this is and what it came from; `THIRD_PARTY_LICENSES` names every third party
/// whose code, data or behaviour the tree carries, and what was left out for its licence.
#[test]
fn the_notices_name_what_is_carried() {
    let notice = read("NOTICE");
    for phrase in [
        "MissionPlannerRust",
        "David \"Buzz\" Bussenschutt",
        "Mission Planner",
        "Michael Oborne",
        "GNU General Public License",
        "THIRD_PARTY_LICENSES",
    ] {
        assert!(notice.contains(phrase), "NOTICE lacks `{phrase}`");
    }
    let third_party = read(licence::THIRD_PARTY);
    for phrase in [
        "Mission Planner",
        "Michael Oborne",
        "Clipper",
        "Angus Johnson",
        "Boost Software License",
        "ProjNet",
        "Morten Nielsen",
        "GeoUtility",
        "Steffen Habermehl",
        "GeoidHeightsDotNet",
        "EGM96",
        "mono",
        "px4uploader",
        "PX4 Development Team",
        "libcanard",
        "Pavel Kirienko",
        "MAVLink",
        "ArduPilot",
        "LogAnalyzer",
        "RFDLib",
        "GMap.NET",
        "ExifLibrary",
        "Clarity",
        "AGauge",
        "Code Project Open License",
        "gpui",
        "Zed Industries",
        licence::TABLE_BEGIN,
        licence::TABLE_END,
    ] {
        assert!(
            third_party.contains(phrase),
            "{} lacks `{phrase}`",
            licence::THIRD_PARTY
        );
    }
}
