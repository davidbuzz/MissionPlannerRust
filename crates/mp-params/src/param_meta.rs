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

//! Parameter metadata: what each parameter means, and what values it will accept.
//!
//! A parameter download gives names and numbers. This is everything that turns those into a
//! configuration screen a pilot can use without a wiki tab open - and, more importantly, into one
//! that refuses a value the firmware will reject.

/// How exposed a parameter should be.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UserLevel {
    /// Safe for anyone to change.
    Standard,
    /// Hidden unless advanced settings are enabled.
    Advanced,
    /// The metadata does not say; treated as advanced.
    Unspecified,
}

impl UserLevel {
    /// Whether to show this parameter in the ordinary configuration view.
    ///
    /// Unspecified counts as advanced: a parameter nobody documented well enough to classify is
    /// not one to put in front of a new user by default.
    #[must_use]
    pub const fn is_basic(self) -> bool {
        matches!(self, Self::Standard)
    }
}

/// Everything known about one parameter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamMeta {
    /// Parameter name, e.g. `WPNAV_SPEED`.
    pub name: &'static str,
    /// Short label for a form field.
    pub display_name: &'static str,
    /// Full description.
    pub description: &'static str,
    /// Units as the firmware documents them, e.g. `cm/s`.
    pub units: &'static str,
    /// Inclusive minimum and maximum.
    pub range: Option<(f64, f64)>,
    /// The `Range` field's text as the file writes it - `0.0 1.0` where `range` holds the two
    /// numbers - which is what the Options cell shows; empty without one.
    pub range_text: &'static str,
    /// Suggested step for a spinner.
    pub increment: Option<f64>,
    /// Named values, when the parameter is an enumeration.
    pub values: &'static [(i64, &'static str)],
    /// Named bits, when it is a bitmask.
    pub bitmask: &'static [(u32, &'static str)],
    /// How exposed it should be.
    pub user_level: UserLevel,
    /// Whether a reboot is needed for a change to take effect.
    pub reboot_required: bool,
}

impl ParamMeta {
    /// Whether a value is inside the documented range.
    ///
    /// Out-of-range values are not merely untidy: ArduPilot will accept a parameter write it
    /// considers invalid and then behave unexpectedly, so catching it in the editor is the last
    /// chance to catch it at all.
    #[must_use]
    pub fn accepts(&self, value: f64) -> bool {
        match self.range {
            Some((low, high)) => value >= low && value <= high,
            None => true,
        }
    }

    /// The name of an enumerated value.
    #[must_use]
    pub fn value_name(&self, value: i64) -> Option<&'static str> {
        self.values
            .iter()
            .find(|(number, _)| *number == value)
            .map(|(_, name)| *name)
    }

    /// The bits set in a bitmask value, by name.
    #[must_use]
    pub fn bit_names(&self, value: u32) -> Vec<&'static str> {
        self.bitmask
            .iter()
            .filter(|(bit, _)| *bit < 32 && value & (1u32 << *bit) != 0)
            .map(|(_, name)| *name)
            .collect()
    }

    /// Whether this parameter is an enumeration rather than a free number.
    #[must_use]
    pub const fn is_enumeration(&self) -> bool {
        !self.values.is_empty()
    }

    /// Whether this parameter is a bitmask.
    #[must_use]
    pub const fn is_bitmask(&self) -> bool {
        !self.bitmask.is_empty()
    }
}

/// One vehicle's section of the bundled `ParameterMetaDataBackup.xml`: the `<ArduCopter2>`,
/// `<ArduPlane>`, ... element under `<Params>`, named as the C#'s `Firmwares` names the vehicle.
#[derive(Debug)]
pub struct Section {
    /// The element's name.
    name: &'static str,
    /// Its parameters, as places in the generated `ENTRIES`, in name order.
    entries: &'static [u16],
    /// The names it marks `<ReadOnly>True</ReadOnly>`, sorted.
    read_only: &'static [&'static str],
}

/// The table `cargo xtask codegen-param-meta` writes from the XML: every vehicle section, and
/// each distinct entry once.
#[path = "generated/param_meta.rs"]
mod generated;

/// The entry at a place in the generated table.
fn entry(place: u16) -> Option<&'static ParamMeta> {
    generated::ENTRIES.get(usize::from(place))
}

impl Section {
    /// The element's name: `ArduCopter2`, `ArduPlane`, `ArduRover`, `ArduSub`, `ArduTracker`.
    #[must_use]
    pub const fn name(&self) -> &'static str {
        self.name
    }

    /// How many parameters it documents.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether it documents none.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Its parameters, in name order.
    pub fn parameters(&self) -> impl Iterator<Item = &'static ParamMeta> {
        self.entries.iter().filter_map(|place| entry(*place))
    }

    /// The parameter of exactly this name, if the section documents it: `element.Element(nodeKey)`.
    /// A bisection, the places being in name order.
    /// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPM.cs:86`
    #[must_use]
    pub fn get(&self, name: &str) -> Option<&'static ParamMeta> {
        let at = self
            .entries
            .binary_search_by(|place| entry(*place).map(|meta| meta.name).cmp(&Some(name)))
            .ok()?;
        entry(*self.entries.get(at)?)
    }

    /// Whether the section marks the parameter `<ReadOnly>True</ReadOnly>`.
    #[must_use]
    pub fn is_read_only(&self, name: &str) -> bool {
        self.read_only.binary_search(&name).is_ok()
    }
}

/// Every vehicle section the bundled file has, in its order.
#[must_use]
pub fn sections() -> &'static [Section] {
    generated::SECTIONS
}

/// The section the C# reads for a vehicle: `_parameterMetaDataXML.Element("Params")
/// .Elements(vechileType)`, the element of exactly that name.
///
/// `vechileType` is `MainV2.comPort.MAV.cs.firmware.ToString()`, a `Firmwares` name, which
/// `ParameterMetaDataRepository` hands this fallback as it stands - the renaming to `ArduCopter`,
/// `Rover` and `AntennaTracker` is the fetched files', not this one's. The file has sections for
/// `ArduCopter2`, `ArduPlane`, `ArduRover`, `ArduSub` and `ArduTracker`; for `Ateryx`, `Gimbal`,
/// `Other` and `AP_Periph` it has none, so the C# finds no element and answers the empty string,
/// as `None` here (`PX4` never reaches it: it is sent to the PX4 repository instead).
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPM.cs:70-104; ExtLibs/Utilities/ParameterMetaDataRepository.cs:36-49; ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:198-204; ExtLibs/ArduPilot/Firmwares.cs:6-18`
#[must_use]
pub fn section(vehicle: &str) -> Option<&'static Section> {
    sections().iter().find(|section| section.name == vehicle)
}

/// Looks a parameter up by its exact name in the vehicle's section ([`section`]).
///
/// Mission Planner's rule, and the only one: its fallback, `ParameterMetaDataRepositoryAPM`, asks
/// the vehicle's section of the XML this table is generated from for the element named exactly as
/// the parameter, and answers the empty string when that section has none - it does not look in
/// another vehicle's section, so a copter's `RTL_ALT` asked for a plane is undocumented, and a
/// plane's `FLTMODE1` is the plane's list of modes, not the copter's
/// (`// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPM.cs:70-104`). The `apm.pdef.xml` it
/// reads first is asked for exactly `Vehicle:NAME` or `NAME`
/// (`// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:219-231`). A numbered name
/// the table does not list - `SERVO33_FUNCTION` against a table that stops at 32 - has no
/// documentation, as it has none in Mission Planner; it does not borrow a sibling's.
///
/// A bisection of the section, which is sorted by name.
#[must_use]
pub fn lookup(vehicle: &str, name: &str) -> Option<&'static ParamMeta> {
    section(vehicle)?.get(name)
}

/// Whether the bundled XML marks the parameter `<ReadOnly>True</ReadOnly>` in the vehicle's
/// section ([`section`]): the C#'s `ParameterMetaDataRepositoryAPM` answer for `ReadOnly`, asked
/// when the fetched `apm.pdef.xml` has nothing for the name.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepository.cs:42-49; ParameterMetaDataRepositoryAPM.cs:70-104`
#[must_use]
pub fn read_only_backup(vehicle: &str, name: &str) -> bool {
    section(vehicle).is_some_and(|section| section.is_read_only(name))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `Firmwares.ArduCopter2`'s name, the copter section's.
    const COPTER: &str = "ArduCopter2";

    /// Every section of the file at 5dbb2b0, in its order, with the parameters the XML gives it:
    /// each in name order, each found by its own name as itself, its ReadOnly names its own.
    #[test]
    fn every_section_of_the_file_is_bundled_by_its_name() {
        let found: Vec<(&str, usize)> = sections()
            .iter()
            .map(|section| (section.name(), section.len()))
            .collect();
        assert_eq!(
            found,
            [
                ("ArduCopter2", 3136),
                ("ArduPlane", 2925),
                ("ArduRover", 2641),
                ("ArduSub", 1717),
                ("ArduTracker", 1055),
            ]
        );
        for section in sections() {
            let names: Vec<&str> = section.parameters().map(|meta| meta.name).collect();
            assert_eq!(names.len(), section.len(), "{}", section.name());
            assert!(
                names.windows(2).all(|pair| pair[0] < pair[1]),
                "{} is not in name order",
                section.name()
            );
            for meta in section.parameters() {
                assert!(
                    lookup(section.name(), meta.name).is_some_and(|found| std::ptr::eq(found, meta)),
                    "{} {}",
                    section.name(),
                    meta.name
                );
            }
            assert!(section.read_only.windows(2).all(|pair| pair[0] < pair[1]));
            assert!(
                section.read_only.iter().all(|name| section.get(name).is_some()),
                "{}",
                section.name()
            );
            assert!(!section.read_only.is_empty(), "{}", section.name());
        }
    }

    /// A parameter documented alike in several sections is one entry, one documented
    /// differently is one per way: the table holds fewer entries than the sections list.
    #[test]
    fn an_entry_documented_alike_is_held_once() {
        assert!(
            generated::ENTRIES
                .windows(2)
                .all(|pair| pair[0].name < pair[1].name
                    || (pair[0].name == pair[1].name && pair[0] != pair[1])),
            "the entries are in name order and each is distinct"
        );
        let listed: usize = sections().iter().map(Section::len).sum();
        assert_eq!(listed, 11_474);
        assert!(generated::ENTRIES.len() < listed / 2);
        eprintln!(
            "{} distinct entries for {listed} parameters in {} sections",
            generated::ENTRIES.len(),
            sections().len()
        );

        let monitor: Vec<&ParamMeta> = sections()
            .iter()
            .filter_map(|section| section.get("BATT_MONITOR"))
            .collect();
        assert_eq!(monitor.len(), 5, "every section documents BATT_MONITOR");
        assert!(monitor.iter().all(|meta| std::ptr::eq(*meta, monitor[0])));
    }

    /// `GetParameterMetaData(name, key, vechileType)` reads the vehicle's section and no other:
    /// a copter's parameter asked for a plane is the C#'s empty string, `None`; a parameter both
    /// document differently is each one's own; a vehicle the file has no section for has nothing.
    /// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPM.cs:80-103`
    #[test]
    fn a_copter_parameter_asked_for_a_plane_gives_what_the_csharp_gives() {
        let rtl = lookup(COPTER, "RTL_ALT").expect("a copter's RTL altitude");
        assert_eq!(rtl.display_name, "RTL Altitude");
        assert_eq!(lookup("ArduPlane", "RTL_ALT"), None);
        assert_eq!(lookup("ArduRover", "RTL_ALT"), None);
        assert_eq!(lookup("ArduPlane", "ACRO_RP_EXPO"), None);

        let quadplane = lookup("ArduPlane", "Q_ENABLE").expect("a plane's QuadPlane switch");
        assert_eq!(quadplane.display_name, "Enable QuadPlane");
        assert_eq!(lookup(COPTER, "Q_ENABLE"), None);
        assert!(lookup("ArduRover", "MODE1").is_some());
        assert_eq!(lookup(COPTER, "MODE1"), None);
        assert!(lookup("ArduSub", "WPNAV_SPEED").is_some());
        assert_eq!(lookup("ArduPlane", "WPNAV_SPEED"), None);

        let copter_modes = lookup(COPTER, "FLTMODE1").expect("a copter's modes");
        let plane_modes = lookup("ArduPlane", "FLTMODE1").expect("a plane's modes");
        assert_eq!(copter_modes.value_name(0), Some("Stabilize"));
        assert_eq!(plane_modes.value_name(0), Some("Manual"));
        assert_eq!(copter_modes.display_name, "Flight Mode 1");
        assert_eq!(plane_modes.display_name, "FlightMode1");

        // `Ateryx`, `Gimbal`, `Other` and `AP_Periph` have no section, and neither has the
        // fetched files' `ArduCopter`; an empty name is the C#'s ArgumentException, caught.
        for vehicle in ["Ateryx", "Gimbal", "Other", "AP_Periph", "PX4", "ArduCopter", ""] {
            assert!(section(vehicle).is_none(), "{vehicle:?}");
            assert_eq!(lookup(vehicle, "BATT_MONITOR"), None, "{vehicle:?}");
            assert!(!read_only_backup(vehicle, "BARO1_DEVID"), "{vehicle:?}");
        }
        assert_eq!(lookup(COPTER, ""), None);
    }

    /// The bundled file's ReadOnly marks, the vehicle's: a device id is read-only, a tuning gain
    /// is not, and a section without the parameter has no mark for it.
    #[test]
    fn the_bundled_read_only_marks_are_the_vehicles() {
        for section in sections() {
            assert!(read_only_backup(section.name(), "BARO1_DEVID"), "{}", section.name());
            assert!(!read_only_backup(section.name(), "NOT_A_PARAM"));
        }
        assert!(!read_only_backup(COPTER, "ATC_RAT_RLL_P"));
        assert!(read_only_backup(COPTER, "STAT_RUNTIME"));
        assert!(read_only_backup("ArduPlane", "STAT_RUNTIME"));
        assert!(
            !read_only_backup("ArduSub", "STAT_RUNTIME"),
            "the sub's section has no STAT_RUNTIME"
        );
    }

    /// SITL's 1,408 names, as `testdata/params/sitl-copter.param` lists them.
    fn sitl_names() -> Vec<String> {
        let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/params/sitl-copter.param");
        let text = mp_os::fs::read_to_string(&fixture).expect("the SITL parameter dump");
        let names: Vec<String> = text
            .lines()
            .filter(|line| !line.starts_with('#'))
            .filter_map(|line| line.split(',').next())
            .map(str::to_owned)
            .collect();
        assert_eq!(names.len(), 1408, "the dump PLAN.md 10.5 counted");
        names
    }

    /// The lookup is exact, as `ParameterMetaDataRepositoryAPM.GetParameterMetaData` asks the
    /// XML for the element of exactly that name (`ParameterMetaDataRepositoryAPM.cs:70-104`) and
    /// the pdef lookup for exactly `Vehicle:NAME` or `NAME`
    /// (`ParameterMetaDataRepositoryAPMpdef.cs:219-231`): `RC5_MIN` is documented only if the
    /// table has `RC5_MIN`, and a renumbered name the table lacks is not documented at all - not
    /// by a sibling with other digits, which is what a digit-folding retry here used to do.
    #[test]
    fn the_bundled_lookup_is_exact() {
        let copter = section(COPTER).expect("the copter section");
        let listed = |name: &str| copter.parameters().any(|meta| meta.name == name);

        for name in ["RC5_MIN", "SERVO9_FUNCTION", "BATT2_MONITOR"] {
            assert_eq!(
                lookup(COPTER, name).map(|meta| meta.name),
                listed(name).then_some(name),
                "{name}"
            );
        }
        assert!(listed("RC5_MIN"), "the table documents RC5_MIN by name");

        // Renumbered names the table lacks, each with a sibling it does list.
        for (absent, sibling) in [
            ("RC99_MIN", "RC5_MIN"),
            ("SERVO99_FUNCTION", "SERVO9_FUNCTION"),
            ("BATT99_MONITOR", "BATT2_MONITOR"),
            ("SERVO_FUNCTION", "SERVO1_FUNCTION"),
        ] {
            assert!(listed(sibling), "{sibling} is in the table");
            assert!(!listed(absent), "{absent} is not in the table");
            assert_eq!(
                lookup(COPTER, absent),
                None,
                "{absent} borrowed a sibling's documentation"
            );
        }

        // Every SITL name finds an entry of exactly its own name or nothing.
        let names = sitl_names();
        let mut documented = 0;
        for name in &names {
            match lookup(COPTER, name) {
                Some(meta) => {
                    assert_eq!(meta.name, name.as_str());
                    documented += 1;
                }
                None => assert!(!listed(name), "{name} is in the table"),
            }
        }
        eprintln!(
            "bundled table documents {documented} of {} SITL names, by exact name",
            names.len()
        );
    }
}
