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

//! What the shipped scripts actually need from the host.
//!
//! D16 requires that all nineteen `testdata/scripts/*.py` run unmodified, and PLAN.md §12 D6 says
//! any script that will not is recorded per script rather than hand-waved. Both need the same
//! thing first: a measurement of what surface the corpus touches.
//!
//! That surface is much larger than `Script.*`. The scripts reach into `MAV` (the
//! `MAVLinkInterface`), `cs` (`CurrentState`), `MainV2`, `FlightPlanner` and `Joystick` - so
//! "implement `Script.cs`" is not the job, and finding that out by running an interpreter and
//! watching it raise would be the slow way to learn it.
//!
//! The scan is textual and deliberately crude: it looks for `Object.member`, which over-reports
//! (a name in a comment counts) and under-reports (`getattr` would not). It is a scope estimate,
//! not a parser, and it is honest about which by never being used to decide that a script passes -
//! only to decide what to build next.

/// Which object a script reached into.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Surface {
    /// `Script` - the object this crate implements. `Script.cs`.
    Script,
    /// `cs` - the vehicle state. `CurrentState.cs`, our `mp_vehicle::VehicleState`.
    CurrentState,
    /// `MAV` - the link itself. `MAVLinkInterface.cs`, our `mp_link::Link`.
    Link,
    /// `MainV2` - the application root.
    Application,
    /// `FlightPlanner` or `FlightData` - a screen.
    Screen,
    /// `Ports` - the list of links.
    Ports,
    /// `Joystick`.
    Joystick,
    /// `mavutil` - bound to the `Script` object, and its two methods return null in the C#.
    MavUtil,
}

impl Surface {
    /// The name a script uses.
    #[must_use]
    pub const fn binding(self) -> &'static str {
        match self {
            Self::Script => "Script",
            Self::CurrentState => "cs",
            Self::Link => "MAV",
            Self::Application => "MainV2",
            Self::Screen => "FlightPlanner",
            Self::Ports => "Ports",
            Self::Joystick => "Joystick",
            Self::MavUtil => "mavutil",
        }
    }

    /// Every binding the C# puts in a script's scope. `// C#: Script.cs:45-53`
    #[must_use]
    pub const fn all() -> &'static [Self] {
        &[
            Self::Script,
            Self::CurrentState,
            Self::Link,
            Self::Application,
            Self::Screen,
            Self::Ports,
            Self::Joystick,
            Self::MavUtil,
        ]
    }
}

/// One `Object.member` a script referred to.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct Requirement {
    /// Which scope object.
    pub surface: Surface,
    /// The member named on it.
    pub member: String,
}

/// What one script needs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScriptRequirements {
    /// The file, without its directory.
    pub name: String,
    /// Everything it referred to, sorted and deduplicated.
    pub requirements: Vec<Requirement>,
    /// How many distinct .NET type references it makes. See [`Self::needs_dotnet`].
    dotnet_references: usize,
    /// The .NET names it reaches, whole. See [`Self::dotnet_names`].
    dotnet_names: Vec<String>,
}

/// The names under which IronPython hands a script .NET: its own bridge and the three
/// namespaces the shipped scripts import from.
pub const DOTNET_ROOTS: [&str; 4] = ["clr", "System", "MissionPlanner", "MAVLink"];

impl ScriptRequirements {
    /// Scans one script's source.
    #[must_use]
    pub fn scan(name: &str, source: &str) -> Self {
        let mut found: Vec<Requirement> = Vec::new();
        // `FlightData` is the same surface as `FlightPlanner`, so both map to Screen and the scan
        // has to look for each name rather than for the enum's own label.
        const BINDINGS: &[(&str, Surface)] = &[
            ("Script", Surface::Script),
            ("cs", Surface::CurrentState),
            ("MAV", Surface::Link),
            ("MainV2", Surface::Application),
            ("FlightPlanner", Surface::Screen),
            ("FlightData", Surface::Screen),
            ("Ports", Surface::Ports),
            ("Joystick", Surface::Joystick),
            ("mavutil", Surface::MavUtil),
        ];

        for line in source.lines() {
            // Comments are skipped, which removes the commonest source of over-reporting: these
            // scripts are mostly worked examples and carry a lot of commented-out alternatives.
            let code = line.split('#').next().unwrap_or("");
            for (binding, surface) in BINDINGS {
                let mut rest = code;
                while let Some(position) = rest.find(binding) {
                    let after = rest.get(position + binding.len()..).unwrap_or("");
                    let before_ok = rest
                        .get(..position)
                        .and_then(|s| s.chars().next_back())
                        .is_none_or(|c| !c.is_alphanumeric() && c != '_' && c != '.');
                    if before_ok && let Some(tail) = after.strip_prefix('.') {
                        let member: String = tail
                            .chars()
                            .take_while(|c| c.is_alphanumeric() || *c == '_')
                            .collect();
                        if !member.is_empty() {
                            let requirement = Requirement {
                                surface: *surface,
                                member,
                            };
                            if !found.contains(&requirement) {
                                found.push(requirement);
                            }
                        }
                    }
                    rest = after;
                }
            }
        }
        found.sort();
        Self {
            name: name.to_owned(),
            requirements: found,
            dotnet_references: count_dotnet_references(source),
            dotnet_names: scan_dotnet_names(source),
        }
    }

    /// The .NET names this script reaches, whole, sorted and deduplicated: every dotted path
    /// under [`DOTNET_ROOTS`] - `MissionPlanner.MainV2.speechEngine.SpeakAsync`,
    /// `MAVLink.MAV_CMD.WAYPOINT`, `clr.AddReference` - and every name a `from ... import`
    /// takes from one - `System.Byte`. Comments and string literals are skipped, so
    /// `clr.AddReference("MissionPlanner.Utilities")` names `clr.AddReference` only.
    ///
    /// The measure of what a shim for the CLR has to answer to; like the rest of the scan it is
    /// textual, and a name reached through `getattr` is not seen.
    #[must_use]
    pub fn dotnet_names(&self) -> &[String] {
        &self.dotnet_names
    }

    /// Whether this script reaches into .NET directly, rather than through the scope bindings.
    ///
    /// **The hard compatibility limit, and it is not about the engine.** IronPython loads every
    /// loaded assembly into the script's namespace (`engine.Runtime.LoadAssembly(ass)` for each,
    /// `Script.cs:40-44`), so a script can write `MissionPlanner.MainV2.instance` or
    /// `System.Windows.Forms.MessageBox.Show(...)` and reach a .NET type by name. No Rust Python
    /// engine can offer that, because there is no CLR behind it - the types do not exist to be
    /// reached.
    ///
    /// A script doing this cannot run unmodified under *any* engine we could choose, which makes
    /// this a different question from "is rustpython compatible enough". PLAN.md §10.4 assumes the
    /// nineteen run unmodified given a Python engine; this is the measurement that says which of
    /// them can.
    #[must_use]
    pub fn needs_dotnet(&self) -> bool {
        self.dotnet_references > 0
    }

    /// How many distinct .NET references were found.
    #[must_use]
    pub const fn dotnet_references(&self) -> usize {
        self.dotnet_references
    }

    /// Whether every member this script needs is on a surface the host implements.
    #[must_use]
    pub fn only_needs(&self, surfaces: &[Surface]) -> bool {
        self.requirements
            .iter()
            .all(|requirement| surfaces.contains(&requirement.surface))
    }

    /// The surfaces this script touches, in order.
    #[must_use]
    pub fn surfaces(&self) -> Vec<Surface> {
        let mut out: Vec<Surface> = self.requirements.iter().map(|r| r.surface).collect();
        out.sort();
        out.dedup();
        out
    }
}

/// A line of code without its comment and with every string literal emptied: `'...'` and
/// `"..."`, a backslash escaping the next character, as the corpus writes them.
fn code_only(line: &str) -> String {
    let mut out = String::with_capacity(line.len());
    let mut quote: Option<char> = None;
    let mut escaped = false;
    for c in line.chars() {
        match quote {
            Some(open) => {
                if escaped {
                    escaped = false;
                } else if c == '\\' {
                    escaped = true;
                } else if c == open {
                    quote = None;
                    out.push(c);
                }
            }
            None => {
                if c == '#' {
                    break;
                }
                if c == '\'' || c == '"' {
                    quote = Some(c);
                }
                out.push(c);
            }
        }
    }
    out
}

/// See [`ScriptRequirements::dotnet_names`].
fn scan_dotnet_names(source: &str) -> Vec<String> {
    let is_root = |name: &str| {
        DOTNET_ROOTS
            .iter()
            .any(|root| name == *root || name.starts_with(&format!("{root}.")))
    };
    let mut found: Vec<String> = Vec::new();
    for line in source.lines() {
        let code = code_only(line);
        let trimmed = code.trim();
        // `from System import Byte, Func`: the names it takes.
        if let Some(rest) = trimmed.strip_prefix("from ")
            && let Some((module, names)) = rest.split_once(" import ")
            && is_root(module.trim())
        {
            for name in names.split(',') {
                let name = name.split(" as ").next().unwrap_or("").trim();
                if !name.is_empty() && name != "*" {
                    found.push(format!("{}.{name}", module.trim()));
                }
            }
            continue;
        }
        // Dotted paths: a run of identifier characters and dots that starts at a root.
        let mut token = String::new();
        for c in code.chars().chain(std::iter::once(' ')) {
            if c.is_alphanumeric() || c == '_' || c == '.' {
                token.push(c);
                continue;
            }
            let name = token.trim_end_matches('.');
            if name.contains('.') && is_root(name) {
                found.push(name.to_owned());
            }
            token.clear();
        }
    }
    found.sort();
    found.dedup();
    found
}

/// Counts the distinct ways a script reaches into .NET.
///
/// Textual, and it over-reports if a comment mentions one - which is why comments are stripped
/// first. The three forms that matter: `clr.` (IronPython's own bridge), a `MissionPlanner.`
/// type path, and a `System.` type path. `import clr` counts as the bridge.
fn count_dotnet_references(source: &str) -> usize {
    let mut found: Vec<String> = Vec::new();
    for line in source.lines() {
        let code = line.split('#').next().unwrap_or("").trim();
        if code.is_empty() {
            continue;
        }
        for marker in ["clr.", "import clr", "MissionPlanner.", "System."] {
            if code.contains(marker) && !found.iter().any(|f| f == marker) {
                found.push(marker.to_owned());
            }
        }
    }
    found.len()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The .NET reach is what no Rust engine can provide, so it is detected on its own.
    #[test]
    fn a_dotnet_type_path_is_recognised_as_such() {
        for source in [
            "MissionPlanner.MainV2.instance.FlightPlanner.BUT_read_Click(x, null)\n",
            "import clr\n",
            "clr.AddReference('MissionPlanner')\n",
            "System.Threading.Thread.Sleep(100)\n",
        ] {
            assert!(
                ScriptRequirements::scan("x.py", source).needs_dotnet(),
                "{source:?} reaches into .NET"
            );
        }
    }

    /// And a script that stays inside the scope bindings does not.
    #[test]
    fn a_script_using_only_the_scope_bindings_does_not_need_dotnet() {
        let scanned = ScriptRequirements::scan(
            "x.py",
            "Script.ChangeMode('GUIDED')\nprint(cs.alt)\nScript.Sleep(100)\n",
        );
        assert!(!scanned.needs_dotnet());
    }

    /// `MissionPlanner.MainV2.instance` is a .NET path, not a use of the `MainV2` binding. Getting
    /// this backwards is what made the first reading of the corpus wrong.
    #[test]
    fn a_qualified_dotnet_path_is_not_a_scope_binding() {
        let scanned =
            ScriptRequirements::scan("x.py", "MissionPlanner.MainV2.speechEnable = True\n");
        assert!(scanned.needs_dotnet());
        assert!(
            !scanned.surfaces().contains(&Surface::Application),
            "the scope binding MainV2 is not what this script is using"
        );
    }

    /// A mention in a comment is not a dependency.
    #[test]
    fn a_commented_dotnet_reference_is_not_counted() {
        assert!(!ScriptRequirements::scan("x.py", "# import clr\n").needs_dotnet());
    }

    #[test]
    fn a_member_reference_is_found() {
        let scanned = ScriptRequirements::scan("x.py", "Script.Sleep(100)\ncs.roll\n");
        assert_eq!(
            scanned.requirements,
            vec![
                Requirement {
                    surface: Surface::Script,
                    member: "Sleep".to_owned()
                },
                Requirement {
                    surface: Surface::CurrentState,
                    member: "roll".to_owned()
                },
            ]
        );
    }

    /// Commented-out code is not a requirement. These scripts are worked examples and carry a lot
    /// of it, so counting it would inflate the surface with calls nobody makes.
    #[test]
    fn a_commented_call_is_not_a_requirement() {
        let scanned = ScriptRequirements::scan("x.py", "# Script.ChangeMode('GUIDED')\ncs.alt\n");
        assert_eq!(scanned.surfaces(), vec![Surface::CurrentState]);
    }

    /// A name that merely contains a binding is not a use of it.
    #[test]
    fn a_longer_identifier_is_not_a_binding() {
        let scanned = ScriptRequirements::scan("x.py", "mycs.roll\nMAVLink.thing\nx = cs.alt\n");
        assert_eq!(
            scanned.requirements,
            vec![Requirement {
                surface: Surface::CurrentState,
                member: "alt".to_owned()
            }]
        );
    }

    /// Both screen bindings are the same surface.
    #[test]
    fn the_two_screen_bindings_are_one_surface() {
        let scanned = ScriptRequirements::scan(
            "x.py",
            "FlightPlanner.BUT_read_Click()\nFlightData.instance\n",
        );
        assert_eq!(scanned.surfaces(), vec![Surface::Screen]);
        assert_eq!(scanned.requirements.len(), 2);
    }

    /// The .NET names are whole dotted paths and the names a `from` takes; a path inside a
    /// string, one in a comment and one that merely ends in a root are not.
    #[test]
    fn the_dotnet_names_a_script_reaches_are_listed_whole() {
        let scanned = ScriptRequirements::scan(
            "x.py",
            concat!(
                "import clr\n",
                "clr.AddReference(\"MissionPlanner.Utilities\") # includes MAVLink.Nope\n",
                "from System import Byte, Func as F\n",
                "from MissionPlanner.Utilities import Locationwp\n",
                "import MissionPlanner.Comms\n",
                "id = int(MAVLink.MAV_CMD.WAYPOINT)\n",
                "MissionPlanner.MainV2.speechEngine.SpeakAsync('test ' + cs.roll.ToString())\n",
                "x = myMAVLink.MAV_CMD\n",
                "from math import radians\n",
            ),
        );
        assert_eq!(
            scanned.dotnet_names(),
            [
                "MAVLink.MAV_CMD.WAYPOINT",
                "MissionPlanner.Comms",
                "MissionPlanner.MainV2.speechEngine.SpeakAsync",
                "MissionPlanner.Utilities.Locationwp",
                "System.Byte",
                "System.Func",
                "clr.AddReference",
            ]
        );
    }

    /// Every binding the C# creates is one the scan knows about, or the measurement has a hole.
    #[test]
    fn the_scan_covers_every_binding_the_c_sharp_creates() {
        for surface in Surface::all() {
            let source = format!("{}.member\n", surface.binding());
            let scanned = ScriptRequirements::scan("x.py", &source);
            assert_eq!(
                scanned.surfaces(),
                vec![*surface],
                "{} is not recognised",
                surface.binding()
            );
        }
    }
}
