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

//! The C# original's source tree, which the generators, the ledger and the tests read: a clone of
//! <https://github.com/ArduPilot/MissionPlanner> at commit `5dbb2b0`, named by the environment
//! variable `MP_SRC`. The clone is no part of this repository, and nothing in it says where the
//! clone is: a machine without `MP_SRC` skips what needs the tree, or is told what to set.

use std::path::PathBuf;

/// The repository the tree is a clone of.
pub const URL: &str = "https://github.com/ArduPilot/MissionPlanner";

/// The commit the ledger and the ports cite.
pub const COMMIT: &str = "5dbb2b0";

/// The environment variable that names the clone.
pub const ENV: &str = "MP_SRC";

/// The tree `MP_SRC` names, when it is set and is a directory.
#[must_use]
pub fn tree() -> Option<PathBuf> {
    std::env::var_os(ENV)
        .map(PathBuf::from)
        .filter(|tree| tree.is_dir())
}

/// What to say when the tree is not here.
#[must_use]
pub fn absent() -> String {
    format!("{ENV} is not set to a clone of {URL} (commit {COMMIT})")
}
