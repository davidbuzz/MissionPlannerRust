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

//! Generates flight mode tables from Mission Planner's parameter metadata.
//!
//! Mission Planner does not hard-code ArduPilot's flight modes. It reads them from the `Values`
//! attribute of the `FLTMODE1` (or `MODE1`) parameter in `ParameterMetaDataBackup.xml`, which is
//! generated from ArduPilot's own source. Copying that approach means the mode list updates with
//! the firmware rather than drifting from it - and drift here is visible to every pilot, because
//! the flight mode is the field they look at most.

use std::collections::BTreeMap;
use std::path::Path;

use anyhow::{Context, Result, bail};

/// A vehicle family and the metadata section it lives in.
pub struct VehicleFamily {
    /// Rust enum variant name.
    pub variant: &'static str,
    /// Section name in the metadata XML.
    pub section: &'static str,
    /// Parameters that might hold the mode list, in priority order.
    ///
    /// Vehicles disagree: copters and planes use `FLTMODE1`, rovers use `INITIAL_MODE`, and some
    /// firmware ships metadata with no mode list at all. Trying candidates and skipping a family
    /// that has none is better than failing the whole generation over one vehicle.
    pub parameters: &'static [&'static str],
    /// `MAV_TYPE` values that map to this family.
    pub mav_types: &'static [u8],
    /// Human-readable name.
    pub label: &'static str,
}

/// The families Mission Planner supports.
pub const FAMILIES: &[VehicleFamily] = &[
    VehicleFamily {
        variant: "Copter",
        section: "ArduCopter2",
        parameters: &["FLTMODE1"],
        // Quad, coaxial, helicopter, hexa, octo, tri, and the less common rotorcraft. A vehicle
        // type not listed here falls back to no mode names rather than to the wrong ones.
        mav_types: &[2, 3, 4, 13, 14, 15, 29],
        label: "Copter",
    },
    VehicleFamily {
        variant: "Plane",
        section: "ArduPlane",
        parameters: &["FLTMODE1"],
        mav_types: &[1, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25],
        label: "Plane",
    },
    VehicleFamily {
        variant: "Rover",
        section: "ArduRover",
        parameters: &["MODE1", "INITIAL_MODE"],
        mav_types: &[10, 11],
        label: "Rover",
    },
    VehicleFamily {
        variant: "Sub",
        section: "ArduSub",
        parameters: &["MODE1", "INITIAL_MODE"],
        mav_types: &[12],
        label: "Sub",
    },
    VehicleFamily {
        variant: "Tracker",
        section: "ArduTracker",
        parameters: &["MODE1", "INITIAL_MODE"],
        mav_types: &[5],
        label: "Antenna tracker",
    },
];

/// Extracts `value:Name` pairs for one family, or `None` if this metadata has no mode list for it.
fn modes_for(xml: &str, family: &VehicleFamily) -> Result<Option<BTreeMap<u32, String>>> {
    let Some(section) = extract(xml, family.section) else {
        return Ok(None);
    };

    let values = family
        .parameters
        .iter()
        .find_map(|parameter| extract(section, parameter).and_then(|p| extract(p, "Values")));
    let Some(values) = values else {
        return Ok(None);
    };

    let mut out = BTreeMap::new();
    for entry in values.split(',') {
        let Some((number, name)) = entry.split_once(':') else {
            continue;
        };
        let Ok(number) = number.trim().parse::<u32>() else {
            continue;
        };
        let name = name.trim();
        if name.is_empty() {
            continue;
        }
        out.insert(number, name.to_owned());
    }
    if out.is_empty() {
        return Ok(None);
    }
    Ok(Some(out))
}

/// Returns the contents of the first `<tag>...</tag>`.
fn extract<'a>(xml: &'a str, tag: &str) -> Option<&'a str> {
    let open = format!("<{tag}>");
    let close = format!("</{tag}>");
    let start = xml.find(&open)? + open.len();
    let end = xml[start..].find(&close)? + start;
    xml.get(start..end)
}

/// Generates the mode table module.
pub fn generate(metadata_path: &Path) -> Result<String> {
    let xml = std::fs::read_to_string(metadata_path)
        .with_context(|| format!("reading {}", metadata_path.display()))?;

    // Resolve every family up front, so a family without a mode list is omitted from the generated
    // enum entirely rather than appearing with no modes.
    let mut resolved: Vec<(&VehicleFamily, BTreeMap<u32, String>)> = Vec::new();
    for family in FAMILIES {
        match modes_for(&xml, family)? {
            Some(modes) => resolved.push((family, modes)),
            None => eprintln!(
                "note: no mode list for {} in this metadata; omitting it",
                family.label
            ),
        }
    }
    if resolved.is_empty() {
        bail!("the metadata contained no flight mode lists at all");
    }

        let mut out = String::new();
    out.push_str(crate::licence::HEADER);
    out.push('\n');
    out.push_str(
        "//! Flight mode names, generated from Mission Planner's parameter metadata.\n\
         //!\n\
         //! DO NOT EDIT. Regenerate with `cargo xtask codegen modes`.\n\
         //!\n\
         //! Source: `references/missionplanner/ParameterMetaDataBackup.xml`, the same file the C#\n\
         //! application reads at runtime. Mode numbers are vehicle-specific: mode 4 is Guided on a\n\
         //! copter and ACRO on a plane, so a lookup without the vehicle type is not just imprecise,\n\
         //! it is wrong.\n\n",
    );

    out.push_str("/// A vehicle family, which determines how a custom mode number is read.\n");
    out.push_str("#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]\n");
    out.push_str("pub enum VehicleFamily {\n");
    for (family, _) in &resolved {
        out.push_str(&format!(
            "    /// {}.\n    {},\n",
            family.label, family.variant
        ));
    }
    out.push_str("}\n\n");

    // MAV_TYPE mapping.
    out.push_str("impl VehicleFamily {\n");
    out.push_str(
        "    /// The family a `MAV_TYPE` belongs to, or `None` if it is not one we know.\n",
    );
    out.push_str("    ///\n");
    out.push_str(
        "    /// Returning `None` rather than guessing matters: showing a copter's mode\n",
    );
    out.push_str(
        "    /// names for an unrecognised airframe is worse than showing the raw number.\n",
    );
    out.push_str("    #[must_use]\n");
    out.push_str("    pub const fn from_mav_type(mav_type: u8) -> Option<Self> {\n");
    out.push_str("        Some(match mav_type {\n");
    for (family, _) in &resolved {
        let types: Vec<String> = family.mav_types.iter().map(ToString::to_string).collect();
        out.push_str(&format!(
            "            {} => Self::{},\n",
            types.join(" | "),
            family.variant
        ));
    }
    out.push_str("            _ => return None,\n        })\n    }\n\n");

    // Mode name lookup.
    out.push_str("    /// The name of a custom mode for this family.\n");
    out.push_str("    #[must_use]\n");
    out.push_str("    pub const fn mode_name(self, custom_mode: u32) -> Option<&'static str> {\n");
    out.push_str("        Some(match self {\n");
    for (family, modes) in &resolved {
        out.push_str(&format!(
            "            Self::{} => match custom_mode {{\n",
            family.variant
        ));
        for (number, name) in modes {
            out.push_str(&format!("                {number} => \"{name}\",\n"));
        }
        out.push_str("                _ => return None,\n            },\n");
    }
    out.push_str("        })\n    }\n\n");

    // Every mode, for a dropdown.
    out.push_str("    /// Every mode this family offers, as (number, name), ordered by number.\n");
    out.push_str("    #[must_use]\n");
    out.push_str("    pub const fn modes(self) -> &'static [(u32, &'static str)] {\n");
    out.push_str("        match self {\n");
    for (family, modes) in &resolved {
        out.push_str(&format!("            Self::{} => &[\n", family.variant));
        for (number, name) in modes {
            out.push_str(&format!("                ({number}, \"{name}\"),\n"));
        }
        out.push_str("            ],\n");
    }
    out.push_str("        }\n    }\n}\n\n");

    // Convenience free function.
    out.push_str(
        "/// The mode name for a vehicle type and custom mode number.\n\
         #[must_use]\n\
         pub fn flight_mode_name(mav_type: u8, custom_mode: u32) -> Option<&'static str> {\n\
         \x20   VehicleFamily::from_mav_type(mav_type)?.mode_name(custom_mode)\n\
         }\n",
    );

    Ok(out)
}
