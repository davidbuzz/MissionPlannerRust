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

//! Compiles plugins to the Pulley bytecode the browser build runs: `in out` pairs, each the
//! plugin's file (a component, or a core module carrying the world) and where its bytecode goes.
//! mp-gui's build script runs it over the built-in plugins for a wasm32 build.
//!
//!   precompile-web-plugins <plugin.wasm> <plugin.cwasm> [<plugin.wasm> <plugin.cwasm> ...]

#[cfg(not(target_family = "wasm"))]
fn main() -> std::process::ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() || !args.len().is_multiple_of(2) {
        eprintln!("usage: precompile-web-plugins <plugin.wasm> <plugin.cwasm> [...]");
        return std::process::ExitCode::from(2);
    }
    for pair in args.chunks(2) {
        let [from, to] = pair else {
            continue;
        };
        let compiled = std::fs::read(from)
            .map_err(|err| err.to_string())
            .and_then(|bytes| {
                mp_plugin_host::precompile_for_web(&bytes).map_err(|err| err.to_string())
            })
            .and_then(|compiled| std::fs::write(to, compiled).map_err(|err| err.to_string()));
        if let Err(why) = compiled {
            eprintln!("precompile-web-plugins: {from}: {why}");
            return std::process::ExitCode::FAILURE;
        }
    }
    std::process::ExitCode::SUCCESS
}

/// Nothing to do in a web page: the desktop compiles.
#[cfg(target_family = "wasm")]
fn main() {}
