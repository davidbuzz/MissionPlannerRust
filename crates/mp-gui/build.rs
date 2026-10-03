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

//! Works around a missing development symlink for `libxkbcommon-x11`.
//!
//! gpui's X11 backend links `-lxkbcommon-x11`. Distributions ship the runtime library as
//! `libxkbcommon-x11.so.0` and the bare `.so` symlink only in the `-dev` package, so a machine
//! that can *run* X11 software may still fail to *link* it.
//!
//! The correct fix is `sudo apt install libxkbcommon-x11-dev` (or the distribution equivalent).
//! When that package is absent we create the missing symlink inside `OUT_DIR` and point the
//! linker at it, so the build works without root. If the real development files are present this
//! does nothing at all.

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    #[cfg(target_os = "linux")]
    link_stub_if_needed("xkbcommon-x11");
}

#[cfg(target_os = "linux")]
fn link_stub_if_needed(lib: &str) {
    use std::path::{Path, PathBuf};

    const SEARCH_DIRS: &[&str] = &[
        "/usr/lib/x86_64-linux-gnu",
        "/usr/lib64",
        "/usr/lib",
        "/lib/x86_64-linux-gnu",
    ];

    let bare = format!("lib{lib}.so");

    // Nothing to do when the development symlink already exists.
    if SEARCH_DIRS
        .iter()
        .any(|dir| Path::new(dir).join(&bare).exists())
    {
        return;
    }

    // Find the newest versioned runtime library.
    let versioned = SEARCH_DIRS.iter().find_map(|dir| {
        let entries = std::fs::read_dir(dir).ok()?;
        let mut found: Vec<PathBuf> = entries
            .flatten()
            .map(|e| e.path())
            .filter(|p| {
                p.file_name()
                    .and_then(|n| n.to_str())
                    .is_some_and(|n| n.starts_with(&format!("{bare}.")))
            })
            .collect();
        found.sort();
        found.pop()
    });

    let Some(versioned) = versioned else {
        // Let the linker produce its own, clearer error.
        println!("cargo:warning=lib{lib} not found; install the development package");
        return;
    };

    let Ok(out_dir) = std::env::var("OUT_DIR") else {
        return;
    };
    let stub_dir = PathBuf::from(out_dir).join("link-stubs");
    if std::fs::create_dir_all(&stub_dir).is_err() {
        return;
    }
    let stub = stub_dir.join(&bare);
    if !stub.exists()
        && let Err(err) = std::os::unix::fs::symlink(&versioned, &stub)
    {
        println!("cargo:warning=could not create link stub for {lib}: {err}");
        return;
    }
    println!(
        "cargo:warning=using a local link stub for lib{lib}; install lib{lib}-dev to avoid this"
    );
    println!("cargo:rustc-link-search=native={}", stub_dir.display());
}
