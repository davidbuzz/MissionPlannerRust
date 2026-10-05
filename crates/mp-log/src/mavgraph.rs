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

//! The preselected graphs of the log browser: `mavgraph`, the sets `CMB_preselect` offers.
//!
//! Mission Planner ships a `graphs` directory beside its executable - MAVProxy's `mavgraphs.xml`,
//! `mavgraphs2.xml`, `ekfGraphs.xml` and `ekf3Graphs.xml`, and its own `mavgraphsMP.xml`
//! (`MissionPlanner.csproj:774-788`) - and `readmavgraphsxml` reads every `*.xml` there into the
//! list the drop-down shows, after a handful built into the class. Each graph is a name and some
//! expressions; each expression is split at white space, and each piece is either a field -
//! `ATT.Roll`, `GPS[0].Spd`, with `:2` for the right axis - or an expression to evaluate.
//!
//! The five files are shipped here, as `camerasBuiltin.xml` is in `mp-mission`: embedded, and
//! held to the reference copies by a test.
//! `// C#: ExtLibs/Utilities/mavgraph.cs:1-325; Log/LogBrowse.cs:469-477, 3137-3167`

use std::cmp::Ordering;

/// One thing a preselected graph plots: `mavgraph.displayitem`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayItem {
    /// `type`: the message, for a field; the whole piece, for an expression.
    pub kind: String,
    /// `field`: the field, for a field; empty for an expression.
    pub field: String,
    /// `expression`: the piece as written, when it is not a plain field.
    pub expression: Option<String>,
    /// `left`: false when the piece ends in `:2`.
    pub left: bool,
}

impl DisplayItem {
    /// What `CMB_preselect_SelectedIndexChanged` hands `GraphItem` as its type: the expression
    /// as written, or `type + "." + field`.
    /// `// C#: Log/LogBrowse.cs:3150-3157`
    #[must_use]
    pub fn graphed(&self) -> String {
        self.expression
            .clone()
            .unwrap_or_else(|| format!("{}.{}", self.kind, self.field))
    }

    /// The curve's label: `GraphItem_AddCurve` names a curve `type + "." + header`, the header of
    /// an expression is empty and its unit is forced empty, and a right-axis curve gets ` R`.
    /// `// C#: Log/LogBrowse.cs:1604-1628, 2786-2795`
    #[must_use]
    pub fn label(&self) -> String {
        let mut label = format!("{}.", self.graphed());
        if !self.left {
            label.push_str(" R");
        }
        label
    }
}

/// One entry of the drop-down: `mavgraph.displaylist`, which shows its `Name`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayList {
    /// `Name`.
    pub name: String,
    /// `items`: `None` for "a/None", which the handler returns on.
    pub items: Option<Vec<DisplayItem>>,
}

/// The shipped files, in the order `Directory.GetFiles` lists them on Windows: by name.
pub const SHIPPED: [(&str, &str); 5] = [
    (
        "ekf3Graphs.xml",
        include_str!("../assets/graphs/ekf3Graphs.xml"),
    ),
    (
        "ekfGraphs.xml",
        include_str!("../assets/graphs/ekfGraphs.xml"),
    ),
    (
        "mavgraphs.xml",
        include_str!("../assets/graphs/mavgraphs.xml"),
    ),
    (
        "mavgraphs2.xml",
        include_str!("../assets/graphs/mavgraphs2.xml"),
    ),
    (
        "mavgraphsMP.xml",
        include_str!("../assets/graphs/mavgraphsMP.xml"),
    ),
];

/// A field item, for the built-in table.
fn field(kind: &str, field: &str, left: bool) -> DisplayItem {
    DisplayItem {
        kind: kind.to_owned(),
        field: field.to_owned(),
        expression: None,
        left,
    }
}

/// The graphs built into `mavgraph.graphs`, in the class's order.
/// `// C#: ExtLibs/Utilities/mavgraph.cs:50-206`
#[must_use]
pub fn builtin() -> Vec<DisplayList> {
    let list = |name: &str, items: Vec<DisplayItem>| DisplayList {
        name: name.to_owned(),
        items: Some(items),
    };
    let (l, r) = (true, false);
    vec![
        DisplayList {
            name: "a/None".to_owned(),
            items: None,
        },
        list(
            "Builtin/Mechanical Failure",
            vec![
                field("ATT", "Roll", l),
                field("ATT", "DesRoll", l),
                field("ATT", "Pitch", l),
                field("ATT", "DesPitch", l),
                field("CTUN", "Alt", r),
                field("CTUN", "DAlt", r),
            ],
        ),
        list(
            "Builtin/Mechanical Failure - Stab",
            vec![field("ATT", "Roll", l), field("ATT", "DesRoll", l)],
        ),
        list(
            "Builtin/Mechanical Failure - Auto",
            vec![field("ATT", "Roll", l), field("NTUN", "DRoll", l)],
        ),
        list(
            "Builtin/Vibrations",
            vec![
                field("IMU", "AccX", l),
                field("IMU", "AccY", l),
                field("IMU", "AccZ", l),
            ],
        ),
        list(
            "Builtin/Vibrations 3.3",
            vec![
                field("VIBE", "VibeX", l),
                field("VIBE", "VibeY", l),
                field("VIBE", "VibeZ", l),
                field("VIBE", "Clip0", r),
                field("VIBE", "Clip1", r),
                field("VIBE", "Clip2", r),
            ],
        ),
        list(
            "Builtin/GPS Glitch",
            vec![field("GPS", "HDop", l), field("GPS", "NSats", r)],
        ),
        list(
            "Builtin/Power Issues",
            vec![field("CURR", "Vcc", l), field("POWR", "Vcc", l)],
        ),
        list("Builtin/Errors", vec![field("ERR", "ECode", l)]),
        list(
            "Builtin/Battery Issues",
            vec![
                field("CTUN", "ThrIn", l),
                field("CURR", "ThrOut", l),
                field("CURR", "Volt", r),
            ],
        ),
        list(
            "Builtin/imu consistency xyz",
            vec![
                field("IMU", "AccX", l),
                field("IMU2", "AccX", l),
                field("IMU", "AccY", l),
                field("IMU2", "AccY", l),
                field("IMU", "AccZ", r),
                field("IMU2", "AccZ", r),
            ],
        ),
        list(
            "Builtin/mag consistency xyz",
            vec![
                field("MAG", "MagX", l),
                field("MAG2", "MagX", l),
                field("MAG", "MagY", r),
                field("MAG2", "MagY", r),
                field("MAG", "MagZ", l),
                field("MAG2", "MagZ", l),
            ],
        ),
        list(
            "Builtin/copter loiter",
            vec![
                field("NTUN", "DVelX", l),
                field("NTUN", "VelX", l),
                field("NTUN", "DVelY", l),
                field("NTUN", "VelY", l),
            ],
        ),
        list(
            "Builtin/copter althold",
            vec![
                field("CTUN", "BarAlt", l),
                field("CTUN", "DAlt", l),
                field("CTUN", "Alt", l),
                field("GPS", "Alt", l),
            ],
        ),
        list(
            "Builtin/ekf VEL tune",
            vec![
                field("NKF3", "IVN", l),
                field("NKF3", "IPN", l),
                field("NKF3", "IVE", l),
                field("NKF3", "IPE", l),
                field("NKF3", "IVD", l),
                field("NKF3", "IPD", l),
            ],
        ),
    ]
}

/// Whether a character is in .NET's `[A-z0-9_]`: `A` to `z` takes in `[ \ ] ^ _` and the
/// backtick as well as the letters, so `GPS[0]` is a type.
const fn in_class(c: char) -> bool {
    matches!(c, 'A'..='z' | '0'..='9')
}

/// `^([A-z0-9_]+)\.([A-z0-9_]+)[:2]*$`: the type and field of a piece that is a plain field.
/// `// C#: ExtLibs/Utilities/mavgraph.cs:287`
#[must_use]
pub fn plain_field(piece: &str) -> Option<(&str, &str)> {
    let (kind, rest) = piece.split_once('.')?;
    let field_end = rest.find(|c: char| !in_class(c)).unwrap_or(rest.len());
    let (field, tail) = rest.split_at(field_end);
    let ok = !kind.is_empty()
        && kind.chars().all(in_class)
        && !field.is_empty()
        && tail.chars().all(|c| c == ':' || c == '2');
    ok.then_some((kind, field))
}

/// `processGraphItem`: a graph's expressions, each split at spaces, tabs and newlines, each
/// piece a field or an expression, `:2` marking the right axis.
/// `// C#: ExtLibs/Utilities/mavgraph.cs:277-323`
#[must_use]
pub fn process_graph_item(expressions: &[String]) -> Vec<DisplayItem> {
    let mut list = Vec::new();
    for expression in expressions {
        for piece in expression
            .split([' ', '\t', '\n'])
            .filter(|piece| !piece.is_empty())
        {
            let piece = piece.trim();
            let left = !piece.ends_with(":2");
            match plain_field(piece) {
                Some((kind, field)) => list.push(DisplayItem {
                    kind: kind.to_owned(),
                    field: field.to_owned(),
                    expression: None,
                    left,
                }),
                None => list.push(DisplayItem {
                    kind: piece.to_owned(),
                    field: String::new(),
                    expression: Some(piece.to_owned()),
                    left,
                }),
            }
        }
    }
    list
}

/// One graphs file: `readmavgraphsxml`'s loop over its `graph` elements.
///
/// Each graph is named by its `name` attribute (any case) with the file's name, less its
/// extension, after a space; its `expression` and `description` children are read as trimmed
/// text. A file that is not XML gives what was read before it broke, which is nothing here: the
/// C# logs the exception and moves to the next file.
/// `// C#: ExtLibs/Utilities/mavgraph.cs:223-273`
#[must_use]
pub fn read_file(file_name: &str, xml: &str) -> Vec<DisplayList> {
    let stem = std::path::Path::new(file_name)
        .file_stem()
        .map_or_else(String::new, |stem| stem.to_string_lossy().into_owned());
    let Ok(document) = roxmltree::Document::parse(xml) else {
        return Vec::new();
    };
    document
        .descendants()
        .filter(|node| node.has_tag_name("graph"))
        .map(|graph| {
            let name = graph
                .attributes()
                .find(|attribute| attribute.name().eq_ignore_ascii_case("name"))
                .map(|attribute| format!("{} {stem}", attribute.value()));
            let expressions: Vec<String> = graph
                .descendants()
                .filter(|node| {
                    node.is_element() && node.tag_name().name().eq_ignore_ascii_case("expression")
                })
                .map(|node| read_string(node).trim().to_owned())
                .collect();
            DisplayList {
                name: name.unwrap_or_default(),
                items: Some(process_graph_item(&expressions)),
            }
        })
        .collect()
}

/// `XmlReader.ReadString` on an element: its text up to the first child element.
fn read_string(node: roxmltree::Node<'_, '_>) -> String {
    node.children()
        .take_while(|child| !child.is_element())
        .filter_map(|child| child.text())
        .collect()
}

/// The drop-down's list: the built-in graphs and every shipped file's, sorted by name as
/// `LoadLog2` sorts them before giving them to `CMB_preselect`.
/// `// C#: Log/LogBrowse.cs:469-475; ExtLibs/Utilities/mavgraph.cs:17-22, 209-275`
#[must_use]
pub fn graphs() -> Vec<DisplayList> {
    let mut graphs = builtin();
    for (file, xml) in SHIPPED {
        graphs.extend(read_file(file, xml));
    }
    sort(&mut graphs);
    graphs
}

/// `graphs.Sort((a, b) => a.Name.CompareTo(b.Name))`: the invariant culture's comparison.
pub fn sort(graphs: &mut [DisplayList]) {
    graphs.sort_by(|a, b| compare(&a.name, &b.name));
}

/// `string.CompareTo`, as `netfmt::culture_compare` places the characters graph names use.
fn compare(a: &str, b: &str) -> Ordering {
    crate::netfmt::culture_compare(a, b)
}

#[cfg(test)]
mod tests {
    use mp_os::fs::FsExt as _;
    use super::*;

    /// The shipped copies are the reference tree's, byte for byte.
    #[test]
    fn the_shipped_files_are_mission_planners() {
                // `MP_SRC` names a clone of https://github.com/ArduPilot/MissionPlanner.
        let Some(tree) = std::env::var_os("MP_SRC") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let reference = std::path::PathBuf::from(tree).join("graphs");
        if !reference.os_is_dir() {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        }
        for (name, shipped) in SHIPPED {
            let original = mp_os::fs::read_to_string(reference.join(name)).expect("reference file");
            assert!(original == shipped, "{name} differs from the reference");
        }
        let mut xml: Vec<String> = mp_os::fs::read_dir(&reference)
            .expect("graphs directory")
            .filter_map(Result::ok)
            .map(|entry| entry.file_name().to_string_lossy().into_owned())
            .filter(|name| name.ends_with(".xml"))
            .collect();
        xml.sort();
        let shipped: Vec<&str> = SHIPPED.iter().map(|(name, _)| *name).collect();
        assert_eq!(xml, shipped, "every *.xml the C# reads is shipped");
    }

    /// Every graph of every file is read, after the fifteen built in, and "a/None" - which does
    /// nothing - is the first, which is the one the drop-down selects when it is filled.
    #[test]
    fn the_list_is_the_builtins_and_every_files_graphs_sorted() {
        let graphs = graphs();
        assert_eq!(builtin().len(), 15);
        assert_eq!(graphs.len(), 15 + 47 + 41 + 108 + 137 + 30);
        assert_eq!(graphs[0].name, "a/None");
        assert_eq!(graphs[0].items, None);
        for pair in graphs.windows(2) {
            assert_ne!(
                compare(&pair[0].name, &pair[1].name),
                Ordering::Greater,
                "{} before {}",
                pair[0].name,
                pair[1].name
            );
        }
        let roll = graphs
            .iter()
            .find(|graph| graph.name == "Attitude/Roll and Pitch mavgraphs")
            .expect("MAVProxy's roll and pitch");
        let graphed: Vec<String> = roll
            .items
            .iter()
            .flatten()
            .map(DisplayItem::graphed)
            .collect();
        // Both expressions' pieces, the first's MAVLink ones included.
        assert_eq!(
            graphed,
            vec![
                "degrees(ATTITUDE.roll)",
                "degrees(ATTITUDE.pitch)",
                "ATT.Roll",
                "ATT.Pitch"
            ]
        );
    }

    /// The drop-down's first page, which `tests/gui/log-preselect.gui` clicks by position.
    #[test]
    fn the_first_page_is_the_a_names() {
        let graphs = graphs();
        let first: Vec<&str> = graphs
            .iter()
            .take(4)
            .map(|graph| graph.name.as_str())
            .collect();
        assert_eq!(
            first,
            vec![
                "a/None",
                "ADAP/K1H mavgraphs2",
                "Aerobatics/Position Error mavgraphs2",
                "Aliasing/AccX mavgraphs"
            ]
        );
        let roll = graphs
            .iter()
            .position(|graph| graph.name == "Attitude/Roll and Pitch mavgraphs");
        assert_eq!(roll, Some(ROLL_AND_PITCH));
    }

    /// Where "Attitude/Roll and Pitch mavgraphs" sorts: `log-preselect-item-9`.
    const ROLL_AND_PITCH: usize = 9;

    /// A piece is a field when it is `TYPE.Field`, a `[n]` and a `:2` allowed; anything else is
    /// an expression. `:2` puts either on the right.
    #[test]
    fn pieces_are_fields_or_expressions() {
        assert_eq!(plain_field("ATT.Roll"), Some(("ATT", "Roll")));
        assert_eq!(plain_field("GPS[0].Spd"), Some(("GPS[0]", "Spd")));
        assert_eq!(plain_field("CTUN.Alt:2"), Some(("CTUN", "Alt")));
        assert_eq!(plain_field("CTUN.Alt2"), Some(("CTUN", "Alt2")));
        assert_eq!(plain_field("degrees(ATT.Roll)"), None);
        assert_eq!(plain_field("CTUN.As*CTUN.E2T"), None);
        assert_eq!(plain_field("ATT.Roll:3"), None);
        let items = process_graph_item(&[
            "GPS.Spd CTUN.As:2\n  degrees(ATT.Roll):2".to_owned(),
            "IMU[0].AccX".to_owned(),
        ]);
        let shown: Vec<(String, bool, String)> = items
            .iter()
            .map(|item| (item.graphed(), item.left, item.label()))
            .collect();
        assert_eq!(
            shown,
            vec![
                ("GPS.Spd".to_owned(), true, "GPS.Spd.".to_owned()),
                ("CTUN.As".to_owned(), false, "CTUN.As. R".to_owned()),
                (
                    "degrees(ATT.Roll):2".to_owned(),
                    false,
                    "degrees(ATT.Roll):2. R".to_owned()
                ),
                ("IMU[0].AccX".to_owned(), true, "IMU[0].AccX.".to_owned()),
            ]
        );
    }

    /// A graph is named for its file, and its text is read trimmed.
    #[test]
    fn a_file_names_its_graphs_after_itself() {
        let graphs = read_file(
            "mine.xml",
            "<graphs><graph NAME='Mine/One'><description>x</description>\
             <expression>  ATT.Roll\tATT.Pitch:2 </expression></graph>\
             <graph name=\"Mine/Two\"><expression>sqrt(ATT.Roll)</expression></graph></graphs>",
        );
        assert_eq!(graphs.len(), 2);
        assert_eq!(graphs[0].name, "Mine/One mine");
        assert_eq!(graphs[0].items.as_ref().map(Vec::len), Some(2));
        assert_eq!(graphs[1].name, "Mine/Two mine");
        assert!(read_file("bad.xml", "<graphs><graph").is_empty());
    }

    /// The built-in graphs as the class declares them.
    #[test]
    fn the_builtin_graphs_are_the_classs() {
        let builtin = builtin();
        let failure = &builtin[1];
        assert_eq!(failure.name, "Builtin/Mechanical Failure");
        let labels: Vec<String> = failure
            .items
            .iter()
            .flatten()
            .map(DisplayItem::label)
            .collect();
        assert_eq!(
            labels,
            vec![
                "ATT.Roll.",
                "ATT.DesRoll.",
                "ATT.Pitch.",
                "ATT.DesPitch.",
                "CTUN.Alt. R",
                "CTUN.DAlt. R"
            ]
        );
    }
}
