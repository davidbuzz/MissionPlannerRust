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

//! The std calls that panic in a web page rather than fail, each std's own on the desktop.
//!
//! On `wasm32-unknown-unknown` most of std's operating system calls return an error - a file
//! does not open, a socket does not connect - and the planner already handles that. Three panic
//! instead: `std::env::temp_dir` ("no filesystem on this platform"), `std::process::id` ("no
//! pids on this platform") and `std::env::split_paths`. Here a web page gets an answer that leads
//! to the ordinary error further on: a temporary folder no file can be made in, an id of 0, and
//! no folders on the path. (experiments/web-experiment/tools/port_os.py puts these in place.)

use std::ffi::OsStr;
use std::path::PathBuf;

/// `std::env::temp_dir`; in a web page `/tmp`, where nothing can be written.
#[must_use]
pub fn temp_dir() -> PathBuf {
    #[cfg(not(target_family = "wasm"))]
    {
        std::env::temp_dir()
    }
    #[cfg(target_family = "wasm")]
    {
        PathBuf::from("/tmp")
    }
}

/// `std::process::id`; 0 in a web page, which has no processes.
#[must_use]
pub fn process_id() -> u32 {
    #[cfg(not(target_family = "wasm"))]
    {
        std::process::id()
    }
    #[cfg(target_family = "wasm")]
    {
        0
    }
}

/// `std::env::split_paths`, collected; nothing in a web page, which has no programs to find.
#[must_use]
pub fn split_paths(paths: &(impl AsRef<OsStr> + ?Sized)) -> std::vec::IntoIter<PathBuf> {
    #[cfg(not(target_family = "wasm"))]
    {
        std::env::split_paths(paths).collect::<Vec<_>>().into_iter()
    }
    #[cfg(target_family = "wasm")]
    {
        let _ = paths;
        Vec::new().into_iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn on_the_desktop_each_is_std_s_own() {
        assert_eq!(temp_dir(), std::env::temp_dir());
        assert_eq!(process_id(), std::process::id());
        let joined = std::env::join_paths(["/a", "/b/c"]).expect("joinable");
        assert_eq!(
            split_paths(&joined).collect::<Vec<_>>(),
            std::env::split_paths(&joined).collect::<Vec<_>>()
        );
    }
}
