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

//! Builds the plugins Mission Planner ships into the planner, and works around a missing
//! development symlink for `libxkbcommon-x11`.
//!
//! # The shipped plugins
//!
//! Mission Planner's build puts its plugins in the `plugins` folder beside the executable -
//! every `plugins\\*.cs` copied there (`MissionPlanner.csproj`, `CopyToOutputDirectory`), and the
//! Dowding, OpenDroneID and TerrainMaker projects built into it - so every install starts with
//! them loaded (`Plugin/PluginLoader.cs:203-311`). Their ports are WebAssembly components, a
//! different target from the planner, and Cargo builds nothing for another target unless a step
//! asks it to: until 2026-10-03 none did, and a plain start loaded no plugin at all (the owner's
//! bug). This script builds them for `wasm32-unknown-unknown`, in a target folder of their own so
//! the build does not wait on the lock of the one running it, and writes `builtin_plugins.rs`,
//! which `plugins_ui.rs` includes: the planner carries them, on every platform, however it is
//! started or packaged.
//!
//! Shipped: the ten ports of plugins Mission Planner's build puts in its plugins folder. Three
//! of them return false from `Init` as the C# ships them (example, modechange,
//! persistentsimple), and are listed as not loaded, as Mission Planner's plugin list shows them.
//! Not shipped: `payloadconfig`, which Mission Planner ships with an empty payload table and so
//! never loads, where the port fills the table with the two payloads the file comments out (a
//! user who wants it builds it and puts it in `plugins/`); and `misbehave`, the host's test of
//! a plugin that panics and spins.
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
    build_shipped_plugins();
    #[cfg(target_os = "linux")]
    link_stub_if_needed("xkbcommon-x11");
}

/// The plugins the planner carries: the `mp-plugins` examples, by name.
const SHIPPED: &[&str] = &[
    "anonymizebinlog",
    "dowding",
    "example",
    "fencedist",
    "mapicondesc",
    "menu",
    "modechange",
    "opendroneid",
    "persistentsimple",
    "terrainmaker",
];

/// Builds [`SHIPPED`] for WebAssembly and writes `builtin_plugins.rs` into `OUT_DIR`.
///
/// A failure fails the planner's build: a planner without its plugins is the bug this exists to
/// end, not a degraded build to carry on with.
fn build_shipped_plugins() {
    use std::path::PathBuf;
    use std::process::Command;

    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default());
    let root = manifest.join("../..");
    let plugins = root.join("crates/mp-plugin-host/plugins");
    for watched in [
        plugins.join("Cargo.toml"),
        plugins.join("src"),
        plugins.join("examples"),
        root.join("crates/mp-plugin-host/wit"),
        root.join("Cargo.lock"),
    ] {
        println!("cargo:rerun-if-changed={}", watched.display());
    }
    let out = PathBuf::from(std::env::var("OUT_DIR").unwrap_or_default());
    // A target folder beside the planner's own when there is one to find - its root carries
    // Cargo's CACHEDIR.TAG - so debug and release builds share one plugin build; else in OUT_DIR.
    let target = out
        .ancestors()
        .find(|dir| dir.join("CACHEDIR.TAG").is_file())
        .map_or_else(|| out.join("plugin-target"), |dir| dir.join("plugin-wasm"));
    let cargo = std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_owned());
    let mut command = Command::new(cargo);
    command
        .args([
            "build",
            "--release",
            "--locked",
            "-p",
            "mp-plugins",
            "--target",
            "wasm32-unknown-unknown",
        ])
        .args(SHIPPED.iter().flat_map(|name| ["--example", name]))
        .current_dir(&root)
        .env("CARGO_TARGET_DIR", &target)
        // The planner's own build settings are not the plugins': its encoded flags carry the
        // host's, and under `cargo clippy` its wrapper would lint the plugins too.
        .env_remove("CARGO_ENCODED_RUSTFLAGS")
        .env_remove("RUSTC_WORKSPACE_WRAPPER")
        .env_remove("CARGO_BUILD_TARGET");
    let status = command
        .status()
        .unwrap_or_else(|err| panic!("building the shipped plugins: cargo did not start: {err}"));
    assert!(
        status.success(),
        "building the shipped plugins for wasm32-unknown-unknown failed ({status}); the target is \
         installed with the toolchain rust-toolchain.toml names"
    );
    let built = target.join("wasm32-unknown-unknown/release/examples");
    let mut table = String::from(
        "/// The plugins Mission Planner ships, built for WebAssembly by `build.rs`: file name and\n\
         /// component bytes, in name order.\n\
         pub const BUILTIN: &[(&str, &[u8])] = &[\n",
    );
    for name in SHIPPED {
        let from = built.join(format!("{name}.wasm"));
        let to = out.join(format!("{name}.wasm"));
        std::fs::copy(&from, &to).unwrap_or_else(|err| {
            panic!("the shipped plugin {} is not there: {err}", from.display())
        });
        table.push_str(&format!(
            "    (\"{name}.wasm\", include_bytes!({:?})),\n",
            to.display().to_string()
        ));
    }
    table.push_str("];\n");
    std::fs::write(out.join("builtin_plugins.rs"), table)
        .unwrap_or_else(|err| panic!("writing builtin_plugins.rs: {err}"));
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
