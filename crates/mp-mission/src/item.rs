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

//! A single mission item.

use mp_units::{LatLon, PositionError};

/// `MAV_CMD_NAV_WAYPOINT`, the default command.
pub const MAV_CMD_NAV_WAYPOINT: u16 = 16;

/// `MAV_FRAME_GLOBAL_RELATIVE_ALT`, the frame almost every mission uses: altitude above home.
pub const MAV_FRAME_GLOBAL_RELATIVE_ALT: u8 = 3;

/// `MAV_FRAME_GLOBAL`, altitude above mean sea level. The home item uses this.
pub const MAV_FRAME_GLOBAL: u8 = 0;

/// One command in a mission.
///
/// Deliberately close to the wire format rather than to a tidier model: a mission editor has to
/// round-trip commands it does not understand, and a struct that only holds the fields we happen
/// to support would quietly discard the rest of someone's flight plan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MissionItem {
    /// Position in the mission, zero-based. Item 0 is conventionally home.
    pub seq: u16,
    /// Whether this is the active item. Only ever 1 on the home item in a file.
    pub current: u8,
    /// `MAV_FRAME` the coordinates are expressed in.
    pub frame: u8,
    /// `MAV_CMD` to execute.
    pub command: u16,
    /// Command-specific parameter 1.
    pub param1: f64,
    /// Command-specific parameter 2.
    pub param2: f64,
    /// Command-specific parameter 3.
    pub param3: f64,
    /// Command-specific parameter 4.
    pub param4: f64,
    /// Latitude in degrees, or a command-specific value for non-navigation commands.
    pub x: f64,
    /// Longitude in degrees, or a command-specific value.
    pub y: f64,
    /// Altitude in metres, in the item's frame.
    pub z: f64,
    /// Whether the vehicle continues to the next item automatically.
    pub autocontinue: u8,
}

/// Why a mission item is not usable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum MissionItemError {
    /// The coordinates are not a valid position.
    #[error("item {seq} has an invalid position")]
    Position {
        /// Which item.
        seq: u16,
    },
}

impl Default for MissionItem {
    fn default() -> Self {
        Self {
            seq: 0,
            current: 0,
            frame: MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command: MAV_CMD_NAV_WAYPOINT,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            autocontinue: 1,
        }
    }
}

impl MissionItem {
    /// Whether this command carries coordinates in `x` and `y`.
    ///
    /// The distinction decides whether those fields are scaled by 1e7 on the wire, so getting it
    /// wrong is not cosmetic in either direction:
    ///
    /// * `MAV_CMD_DO_SET_SERVO` puts a servo number in `param1` and leaves x and y at zero.
    ///   Treating that as a position puts a phantom waypoint in the Gulf of Guinea.
    /// * Fence vertices and rally points sit at 5000-5004 and 5100, well outside the classic
    ///   16-95 navigation block, but they carry real latitude and longitude. Excluding them sends
    ///   a fence at latitude 0.0000035 instead of 35 - a fence the vehicle cannot breach because
    ///   it is off the coast of Africa.
    /// * `DO_SET_ROI` (201) is outside that block too and carries a position; `NAV_DELAY` (93) is
    ///   inside it and carries none.
    ///
    /// So the set is not a range. It is Mission Planner's: `Locationwp.isLocationCommand` asks
    /// whether the `MAV_CMD` member carries `[hasLocation()]` in the generated `Mavlink.cs`, and
    /// the link scales by 1e7 exactly when it does (`MAVLinkInterface.cs:4014-4023, 3545-3552`).
    /// These are those 45 members; a test holds the list to `Mavlink.cs` when the C# tree is
    /// present.
    /// `// C#: ExtLibs/Utilities/locationwp.cs:39-56`
    #[must_use]
    pub const fn is_navigation(&self) -> bool {
        matches!(
            self.command,
            16..=19
                | 21..=25
                | 31
                | 36
                | 80..=82
                | 84
                | 85
                | 94
                | 179
                | 188
                | 189
                | 192
                | 195
                | 201
                | 252
                | 611
                | 4001
                | 5000..=5004
                | 5100
                | 30001
                | 31000..=31009
                | 42006
                | 43003
        )
    }

    /// The position, if this item has one.
    pub fn position(&self) -> Result<Option<LatLon>, PositionError> {
        if !self.is_navigation() || (self.x == 0.0 && self.y == 0.0) {
            return Ok(None);
        }
        LatLon::new(self.x, self.y).map(Some)
    }

    /// Whether this is the home item: sequence zero.
    #[must_use]
    pub const fn is_home(&self) -> bool {
        self.seq == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The `MAV_CMD` members `Mavlink.cs` marks `[hasLocation()]`, read from the C# tree.
    fn has_location_in_the_c_sharp() -> Option<Vec<u16>> {
                // `MP_SRC` names a clone of https://github.com/ArduPilot/MissionPlanner.
        let tree = std::env::var_os("MP_SRC")?;
        let path = std::path::PathBuf::from(tree).join("ExtLibs/Mavlink/Mavlink.cs");
        let source = std::fs::read_to_string(path).ok()?;
        let start = source.find("public enum MAV_CMD: ushort")?;
        let body = &source[start..];
        let end = body.find("\n    }")?;
        let mut marked = false;
        let mut commands = Vec::new();
        for line in body[..end].lines() {
            let line = line.trim();
            if line.starts_with("[hasLocation") {
                marked = true;
                continue;
            }
            if line.starts_with('[') || line.starts_with("///") || line.is_empty() {
                continue;
            }
            if let Some((_, value)) = line.split_once('=')
                && let Ok(value) = value.trim().trim_end_matches(',').parse::<u16>()
            {
                if marked {
                    commands.push(value);
                }
                marked = false;
            }
        }
        Some(commands)
    }

    /// Every command the C# scales by 1e7 is navigation here, and no other `u16` is.
    #[test]
    fn the_location_commands_are_the_ones_mavlink_cs_marks() {
        let Some(marked) = has_location_in_the_c_sharp() else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        assert_eq!(
            marked.len(),
            45,
            "Mavlink.cs marks 45 MAV_CMD members hasLocation"
        );
        for command in 0..=u16::MAX {
            let item = MissionItem {
                command,
                ..MissionItem::default()
            };
            assert_eq!(
                item.is_navigation(),
                marked.contains(&command),
                "command {command}"
            );
        }
    }

    /// The ones that decided it: an ROI carries a position, RTL and a delay do not.
    #[test]
    fn a_region_of_interest_has_a_position_and_a_return_to_launch_does_not() {
        let roi = MissionItem {
            command: 201,
            x: -35.36,
            y: 149.16,
            ..MissionItem::default()
        };
        assert!(roi.is_navigation());
        assert!(roi.position().expect("valid").is_some());
        for command in [20, 93, 177, 183] {
            let item = MissionItem {
                command,
                ..MissionItem::default()
            };
            assert!(!item.is_navigation(), "command {command}");
        }
    }
}
