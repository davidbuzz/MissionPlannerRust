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

//! Settings that point into Mission Planner's own folders, pointed at this application's.
//!
//! This application keeps its files in a directory of its own (the owner's ruling of 2026-09-25,
//! PLAN.md section 12, D11) and never writes into the C#'s. The one-shot import ([`crate::migrate`])
//! copies Mission Planner's `config.xml` as it is, so a setting whose value is a path inside the
//! C#'s directory - `logdirectory` above all - came across still pointing there, and this
//! application recorded its flights into `~/.local/share/Mission Planner/logs` (the owner's bug
//! of 2026-10-14). On every load such a value has the C#'s directory replaced by this
//! application's: the user data directory's by the user data directory, the shared data
//! directory's by the shared one. Anything else is left as it is. The C#'s files are not moved
//! or copied by this; what was written there stays there.

use std::path::{Path, PathBuf};

use crate::{Config, Folders};

/// One setting pointed home: its key, what it said, and what it says now.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Rehomed {
    /// The setting's key, `logdirectory`.
    pub key: String,
    /// Its value as loaded, in Mission Planner's folder.
    pub was: String,
    /// Its value now, the same place in this application's folder.
    pub now: String,
}

/// `value` with `from` replaced by `to` when it is `from` or a path inside it - whole components
/// only, so `Mission Planner2` is not `Mission Planner`.
fn rehomed_value(value: &str, from: &Path, to: &Path) -> Option<String> {
    let path = PathBuf::from(value);
    let rest = path.strip_prefix(from).ok()?;
    let moved = if rest.as_os_str().is_empty() {
        to.to_path_buf()
    } else {
        to.join(rest)
    };
    Some(moved.to_string_lossy().into_owned())
}

/// Every setting under the C#'s user data or shared data directory, pointed at this
/// application's; what changed, in the keys' order.
#[must_use]
pub fn rehome(config: &mut Config, folders: &Folders) -> Vec<Rehomed> {
    let pairs = [
        (
            folders.csharp_user_data_directory(),
            folders.user_data_directory(),
        ),
        (folders.csharp_data_directory(), folders.data_directory()),
    ];
    let mut changed = Vec::new();
    let keys: Vec<String> = config.keys().into_iter().map(str::to_owned).collect();
    for key in keys {
        let Some(value) = config.get(&key).map(str::to_owned) else {
            continue;
        };
        let moved = pairs
            .iter()
            .find_map(|(from, to)| rehomed_value(&value, from, to));
        if let Some(now) = moved {
            config.set(key.clone(), now.clone());
            changed.push(Rehomed {
                key,
                was: value,
                now,
            });
        }
    }
    changed
}

#[cfg(test)]
mod tests {
    use super::*;

    fn folders(home: &Path) -> Folders {
        Folders {
            my_documents: home.to_path_buf(),
            local_application_data: home.join(".local/share"),
            common_application_data: PathBuf::from("/usr/share"),
            unix: true,
        }
    }

    /// The owner's own config: `logdirectory` inside Mission Planner's directory is pointed at
    /// this application's; the waypoint folder, already this application's, is left; a folder
    /// whose name only begins with the C#'s is not the C#'s.
    #[test]
    fn a_path_in_mission_planners_folder_is_pointed_at_this_ones() {
        let home = Path::new("/home/pilot");
        let mut config = Config::default();
        config.set(
            "logdirectory",
            "/home/pilot/.local/share/Mission Planner/logs",
        );
        config.set(
            "WPFileDirectory",
            "/home/pilot/.local/share/MissionPlannerRust",
        );
        config.set(
            "custom_fw_dir",
            "/home/pilot/.local/share/Mission Planner2/fw",
        );
        config.set("other", "/home/pilot/.local/share/Mission Planner");
        config.set("baud", "57600");
        let changed = rehome(&mut config, &folders(home));
        assert_eq!(
            config.get("logdirectory"),
            Some("/home/pilot/.local/share/MissionPlannerRust/logs")
        );
        assert_eq!(
            config.get("other"),
            Some("/home/pilot/.local/share/MissionPlannerRust")
        );
        assert_eq!(
            config.get("WPFileDirectory"),
            Some("/home/pilot/.local/share/MissionPlannerRust")
        );
        assert_eq!(
            config.get("custom_fw_dir"),
            Some("/home/pilot/.local/share/Mission Planner2/fw")
        );
        assert_eq!(config.get("baud"), Some("57600"));
        assert_eq!(
            changed.iter().map(|r| r.key.as_str()).collect::<Vec<_>>(),
            ["logdirectory", "other"]
        );
        assert_eq!(
            changed[0].was,
            "/home/pilot/.local/share/Mission Planner/logs"
        );
        // Again: nothing left to change.
        assert!(rehome(&mut config, &folders(home)).is_empty());
    }
}
