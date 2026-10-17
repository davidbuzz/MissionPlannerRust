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

//! The flight screen's "Tlog > Kml or Graph" (`BUT_log2kml`, the Telemetry Logs page's third row,
//! and so LOGS' copy of the page): `new MavlinkLog()` shown, `Log/MavlinkLog.cs` - a form of eight
//! buttons along its top, a progress bar under them, and under that a graph beside a tree of a
//! log's fields. Each click of the page's button makes a new form, as the C#'s does, so nothing
//! is kept between uses; within one form, Graph Log's tree and curves stay until the next Graph
//! Log, and its Y axes until the form closes, as ZedGraph's are never cleared.
//!
//! The buttons, each asking for a telemetry log with the planner's own chooser (`Telemetry
//! Log|*.tlog;*.tlog.*`; Create Matlab file's `*.tlog|*.tlog`), in the log directory where the C#
//! sets `InitialDirectory` and empty where it sets none (Create KML + GPX, Graph Log):
//!
//! - Create KML + GPX: the log replayed ([`mp_log::mavlinklog::flight_data`]); with Shift held,
//!   "Alt offset" asked first; with no mission item 0 in the log, "Relative Alt" asked after;
//!   `<name>.gpx` and `<log>.kmz` written beside it ([`mp_kml::tlog`]);
//! - Convert to Text, Convert to CSV: every packet as `DebugPacket` writes it, to `<name>.txt` and
//!   `<name>.csv` ([`mp_log::mavlinklog::text_file`], [`mp_log::mavlinklog::csv_file`]);
//! - Graph Log: the form's caption "Log - <file>", every field of the log in the tree, a root per
//!   message, and the curves cleared ([`mp_log::mavlinklog::graph_log`]); a field's box ticked
//!   graphs it against time on the left axis of its units, a right click on it on the right
//!   ([`Graph::click`]);
//! - Create Matlab file: `<log>.mat` ([`mp_log::matlab::process_tlog_file`]);
//! - Extract Params, Extract WPs and Extract CS are drawn dimmed, and say so when clicked: they are
//!   not ported (see [`Button::not_ported`]).
//!
//! Each job runs on a thread of its own, as many at once as are started - the C#'s handlers are
//! `async void`, each running while the form is used - the progress bar showing whichever spoke
//! last. Where this is not the C#, each written at its site: the dialogs choose one file where the
//! C#'s are `Multiselect`; Shift is read when the button is clicked, where the C# reads it as the
//! dialog closes; what the C# shows in a box - a log that cannot be opened, a bad offset, a
//! conversion that throws - goes on the status line (the owner's rule of 2026-09-25); the graph is
//! the part of ZedGraph the form sets up - its curves, legend, axes and titles, a date axis written
//! `HH:mm:ss` - not its zoom, pan or menu, each Y axis's title written level over its labels; the
//! splitter does not move.
//! `// C#: GCSViews/FlightData.cs:1305-1310; Log/MavlinkLog.cs:1-1519; Log/MavlinkLog.Designer.cs:29-180`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::path::{Path, PathBuf};
use std::sync::mpsc::{Receiver, Sender, TryRecvError};

use gpui::{
    AnyElement, ClickEvent, Context, MouseButton, MouseDownEvent, SharedString, Window, div,
    prelude::*, px, rgb,
};
use mp_log::mavlinklog::{self, FlightData, GraphLog};
use mp_log::netfmt;

use crate::MissionPlanner;
use crate::config::firmware::PathBox;
use crate::config::mavlink_inspector::{DateScale, culture_cmp};
use crate::config::optional::InputBox;
use crate::i18n::fl;
use crate::plan::elevation::Scale;
use crate::ui::{Live, theme};

/// `ClientSize`. `// C#: Log/MavlinkLog.resx ($this.ClientSize)`
const FORM_WIDTH: f32 = 884.0;
const FORM_HEIGHT: f32 = 379.0;
/// The buttons' `Location.Y` and `Size`. `// C#: Log/MavlinkLog.resx (BUT_*.Location, .Size)`
const BUTTON_TOP: f32 = 6.0;
const BUTTON_WIDTH: f32 = 68.0;
const BUTTON_HEIGHT: f32 = 30.0;
/// `progressBar1`: its place and size. `// C#: Log/MavlinkLog.resx (progressBar1)`
const PROGRESS: (f32, f32, f32, f32) = (10.0, 42.0, 862.0, 26.0);
/// `splitContainer1` at 12, 74, 860 by 293, `SplitterDistance` 703: `zg1` in the 703 on the left,
/// `treeView1` in the 153 after the splitter's 4. `// C#: Log/MavlinkLog.resx (splitContainer1, zg1, treeView1)`
const SPLIT_LEFT: f32 = 12.0;
const SPLIT_TOP: f32 = 74.0;
const CHART_WIDTH: f32 = 703.0;
const SPLIT_HEIGHT: f32 = 293.0;
const TREE_LEFT: f32 = SPLIT_LEFT + CHART_WIDTH + 4.0;
const TREE_WIDTH: f32 = 153.0;
/// A tree node's height.
const NODE_HEIGHT: f32 = 16.0;
/// Each Y axis's width beside the chart.
const AXIS_WIDTH: f32 = 50.0;

/// The form's `Text`. `// C#: Log/MavlinkLog.resx ($this.Text)`
pub const TITLE: &str = "Log";
/// `tlogfilemask`. `// C#: Log/MavlinkLog.cs:32`
pub const TLOG_FILE_MASK: &str = "Telemetry Log|*.tlog;*.tlog.*";
/// Create Matlab file's filter. `// C#: Log/MatLabForms.cs:14`
pub const MATLAB_FILE_MASK: &str = "*.tlog|*.tlog";
/// What every button's handler shows when the log will not open.
/// `// C#: Log/MavlinkLog.cs:93, 231, 424, 1060, 1143, 1225, 1472`
pub const CANNOT_OPEN: &str = "Log Can not be opened. Are you still connected?";
/// Create KML + GPX's offset that is not a number. `// C#: Log/MavlinkLog.cs:73-77`
pub const BAD_OFFSET: &str = "Bad Offset";
/// The pane's title and the X axis's. `// C#: Log/MavlinkLog.cs:40-42`
const PANE_TITLE: &str = "Mavlink Log Graph";
const X_TITLE: &str = "Time (sec)";
/// `XAxis.Scale.Format`, `HH:mm:ss`. `// C#: Log/MavlinkLog.cs:45`
const X_FORMAT: &str = "%H:%M:%S";
/// The default pane's Y axis title, ZedGraph's `y_title_def`, which `IndexOf` compares against.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.cs:527-533`
const Y_AXIS_DEFAULT: &str = "Y Axis";

/// `MavlinkLogBase.ColourValues`: the curves' colours in turn.
/// `// C#: Log/MavlinkLogBase.cs:19-29`
const COLOUR_VALUES: [u32; 16] = [
    0xFF_0000, 0x00_FF00, 0x00_00FF, 0xFF_FF00, 0xFF_00FF, 0x00_FFFF, 0x80_0000, 0x00_8000,
    0x00_0080, 0x80_8000, 0xC0_0000, 0x00_C000, 0x00_00C0, 0xC0_C000, 0xC0_00C0, 0x00_C0C0,
];

/// The form's buttons, left to right as the `.resx` places them.
/// `// C#: Log/MavlinkLog.resx (BUT_*.Location, .Text); Log/MavlinkLog.Designer.cs:33-142`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Button {
    /// `BUT_redokml`, at 129.
    Kml,
    /// `BUT_humanreadable`, at 203.
    Text,
    /// `BUT_convertcsv`, at 277.
    Csv,
    /// `BUT_graphmavlog`, at 351.
    Graph,
    /// `BUT_paramsfromlog`, at 425.
    Params,
    /// `BUT_getwpsfromlog`, at 499.
    Wps,
    /// `but_cs`, at 573.
    Cs,
    /// `BUT_matlab`, at 647.
    Matlab,
}

impl Button {
    /// In the order they stand.
    pub const ALL: [Self; 8] = [
        Self::Kml,
        Self::Text,
        Self::Csv,
        Self::Graph,
        Self::Params,
        Self::Wps,
        Self::Cs,
        Self::Matlab,
    ];

    /// Its probe and element id.
    pub const fn id(self) -> &'static str {
        match self {
            Self::Kml => "mavlinklog-kml",
            Self::Text => "mavlinklog-text",
            Self::Csv => "mavlinklog-csv",
            Self::Graph => "mavlinklog-graph",
            Self::Params => "mavlinklog-params",
            Self::Wps => "mavlinklog-wps",
            Self::Cs => "mavlinklog-cs",
            Self::Matlab => "mavlinklog-matlab",
        }
    }

    /// Its `Text`.
    pub const fn text(self) -> &'static str {
        match self {
            Self::Kml => "Create KML + GPX",
            Self::Text => "Convert to Text",
            Self::Csv => "Convert to CSV",
            Self::Graph => "Graph Log",
            Self::Params => "Extract Params",
            Self::Wps => "Extract WPs",
            Self::Cs => "Extract CS",
            Self::Matlab => "Create Matlab file",
        }
    }

    /// Its `Location.X`.
    const fn x(self) -> f32 {
        match self {
            Self::Kml => 129.0,
            Self::Text => 203.0,
            Self::Csv => 277.0,
            Self::Graph => 351.0,
            Self::Params => 425.0,
            Self::Wps => 499.0,
            Self::Cs => 573.0,
            Self::Matlab => 647.0,
        }
    }

    /// Its dialog's `Filter`.
    pub const fn filter(self) -> &'static str {
        match self {
            Self::Matlab => MATLAB_FILE_MASK,
            _ => TLOG_FILE_MASK,
        }
    }

    /// Whether its dialog opens in the log directory, `InitialDirectory = Settings.Instance.LogDir`:
    /// every one but Create KML + GPX's and Graph Log's, which set none.
    /// `// C#: Log/MavlinkLog.cs:55-61, 208-214, 282-296, 1037-1043; Log/MatLabForms.cs:18-24`
    pub const fn in_log_directory(self) -> bool {
        !matches!(self, Self::Kml | Self::Graph)
    }

    /// Why it does nothing here, which its click says. Extract Params and Extract WPs ask the
    /// replay for the parameter list and the mission as a link is asked (`getParamList`,
    /// `getWPCount` and `getWP` in log-read mode, the mission's with wall-clock timeouts), and
    /// Extract CS writes `MAVState.ToJSON()` - Json.NET's dump of the whole vehicle state - at each
    /// heartbeat: none of the three is ported.
    /// `// C#: Log/MavlinkLog.cs:1110-1334, 1441-1517`
    pub const fn not_ported(self) -> Option<&'static str> {
        match self {
            Self::Params => Some("Extract Params is not ported"),
            Self::Wps => Some("Extract WPs is not ported"),
            Self::Cs => Some("Extract CS is not ported"),
            _ => None,
        }
    }
}

// ---- Graph Log ----------------------------------------------------------------------------------

/// A field's node: its text and its check box.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Node {
    /// `Text`: the field, `roll`, `text[0]`.
    pub text: String,
    /// `Checked`.
    pub checked: bool,
}

/// A message's node: `ATTITUDE`, and its fields under it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Root {
    /// `Text`.
    pub text: String,
    /// `Checked`, which the handler takes back off.
    pub checked: bool,
    /// Whether its fields show: a `TreeView`'s nodes start collapsed.
    pub expanded: bool,
    /// Its fields.
    pub children: Vec<Node>,
}

/// One of the pane's Y axes: its title, `Title.Text`, which `IndexOf` finds it by, and whether it
/// is drawn.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Axis {
    /// `Title.Text`: the units, `[rad]`, or `""`.
    pub title: String,
    /// `IsVisible`.
    pub visible: bool,
}

/// The `x` range, then each left axis's `y` range and each right axis's ([`Graph::ranges`]).
pub type Ranges = ((f64, f64), Vec<(f64, f64)>, Vec<(f64, f64)>);

/// A converter of a log's bytes in a zone, reporting its progress a percent at a time
/// ([`mavlinklog`]'s MATLAB and text writers).
type Convert = fn(&[u8], mavlinklog::Zone<'_>, &mut dyn FnMut(u8)) -> String;

/// A curve: `LineItem`'s label, colour and points, `Tag` the name it was graphed by, and the axis
/// it is drawn against - `YAxisIndex` in the left list, or the right with `IsY2Axis`.
#[derive(Debug, Clone)]
pub struct Curve {
    /// `Tag`: `roll mavlink_attitude_t`.
    pub tag: String,
    /// `Label.Text`: `roll attitude_t`, `-R` after it on the right.
    pub label: String,
    /// Its colour.
    pub colour: u32,
    /// `YAxisIndex`.
    pub y_axis_index: usize,
    /// `IsY2Axis`.
    pub is_y2: bool,
    /// The points, `(XDate, value)`.
    pub series: mp_chart::Series,
    /// The points' `x` and `y` ranges, none for no points.
    ranges: Option<((f64, f64), (f64, f64))>,
}

/// `zg1` and `treeView1`: the tree Graph Log fills, the curves its nodes add, the pane's two lists
/// of Y axes, and what the form keeps between clicks - `colorStep`, `rightclick` and `selection`.
#[derive(Debug, Clone)]
pub struct Graph {
    /// `treeView1.Nodes`.
    pub roots: Vec<Root>,
    /// `datappl`, `options` and the units, from the last log read.
    log: GraphLog,
    /// `zg1.GraphPane.CurveList`.
    pub curves: Vec<Curve>,
    /// `YAxisList`: the default `Y Axis`, drawn with its title hidden, then one a unit.
    pub y_axes: Vec<Axis>,
    /// `Y2AxisList`: the default, untitled and hidden, then one a unit, each hidden.
    pub y2_axes: Vec<Axis>,
    /// `colorStep`.
    colour_step: usize,
    /// `rightclick`.
    right_click: bool,
    /// `selection`: every name graphed.
    pub selection: Vec<String>,
}

impl Default for Graph {
    fn default() -> Self {
        Self::new()
    }
}

/// `ReplaceLastOccurrence(name, "_t", "")`, `Replace("mavlink_", "")` and `ToUpper()`: a struct's
/// name as its root's text, `mavlink_attitude_t` as `ATTITUDE`.
/// `// C#: Log/MavlinkLog.cs:326-335, 842-843`
#[must_use]
pub fn root_text(struct_name: &str) -> String {
    let without = match struct_name.rfind("_t") {
        Some(at) => format!(
            "{}{}",
            struct_name.get(..at).unwrap_or(""),
            struct_name.get(at + 2..).unwrap_or("")
        ),
        None => struct_name.to_owned(),
    };
    without.replace("mavlink_", "").to_uppercase()
}

/// `ResetTreeView`: a root for each struct's name as it first comes, each option's field added to
/// the root made last - so the options, sorted, put each message's fields under it.
/// `// C#: Log/MavlinkLog.cs:831-855`
#[must_use]
pub fn tree_of(options: &[String]) -> Vec<Root> {
    let mut roots: Vec<Root> = Vec::new();
    let mut added = std::collections::HashSet::new();
    for option in options {
        let mut items = option.split('.');
        let (Some(message), Some(field)) = (items.next(), items.next()) else {
            continue;
        };
        let text = root_text(message);
        if added.insert(text.clone()) {
            roots.push(Root {
                text,
                checked: false,
                expanded: false,
                children: Vec::new(),
            });
        }
        if let Some(last) = roots.last_mut() {
            last.children.push(Node {
                text: field.to_owned(),
                checked: false,
            });
        }
    }
    roots
}

impl Graph {
    /// The form's pane before any log: the default Y axis, whose title the constructor hides, and
    /// the default Y2 axis, untitled and hidden.
    /// `// C#: Log/MavlinkLog.cs:38-48; ExtLibs/ZedGraph/ZedGraph/Y2Axis.cs:52`
    #[must_use]
    pub fn new() -> Self {
        Self {
            roots: Vec::new(),
            log: GraphLog::default(),
            curves: Vec::new(),
            y_axes: vec![Axis {
                title: Y_AXIS_DEFAULT.to_owned(),
                visible: true,
            }],
            y2_axes: vec![Axis {
                title: String::new(),
                visible: false,
            }],
            colour_step: 0,
            right_click: false,
            selection: Vec::new(),
        }
    }

    /// The end of Graph Log: `options` sorted as `String.Compare` sorts them, the tree made of
    /// them, and the curves cleared; `selection` and `colorStep` begun again. The Y axes stay.
    /// `// C#: Log/MavlinkLog.cs:302-304, 405-412, 577-593`
    pub fn load(&mut self, log: GraphLog) {
        let mut options = log.options.clone();
        options.sort_by(|a, b| culture_cmp(a, b));
        self.roots = tree_of(&options);
        self.log = log;
        self.curves.clear();
        self.selection.clear();
        self.colour_step = 0;
    }

    /// A Graph Log whose log would not open: `GetLogFileValidFields` returns after clearing
    /// `selection`, `datappl` and `colorStep`, and the curves are cleared; the tree stays.
    /// `// C#: Log/MavlinkLog.cs:405-426, 304`
    pub fn not_read(&mut self) {
        self.selection.clear();
        self.log.series.clear();
        self.colour_step = 0;
        self.curves.clear();
    }

    /// `GetYAxis`: the axis a curve is drawn against - `(true, index)` on the right - its index
    /// taken as 0 past the end of its list.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/CurveItem.cs (GetYAxis)`
    #[must_use]
    pub fn axis_of(&self, curve: &Curve) -> (bool, usize) {
        let len = if curve.is_y2 {
            self.y2_axes.len()
        } else {
            self.y_axes.len()
        };
        let index = if curve.y_axis_index < len {
            curve.y_axis_index
        } else {
            0
        };
        (curve.is_y2, index)
    }

    /// `treeView1_NodeMouseClick` on a field's node (`child`) or a message's: `on_box` a left
    /// click on its check box, which the tree toggles itself before the handler runs; `right` a
    /// right click, which the handler toggles. A field ticked is graphed - on the left axis, or the
    /// right after a right click - unless a curve whose label starts with its text and holds its
    /// message's in lower case is there already (so `roll` is not graphed beside `rollspeed`);
    /// unticked, every such curve goes (so `roll` takes `rollspeed` with it), and every axis no
    /// curve left is drawn against is hidden. A message's node ticked is unticked again.
    /// `// C#: Log/MavlinkLog.cs:1341-1431`
    pub fn click(&mut self, root: usize, child: Option<usize>, right: bool, on_box: bool) {
        let Some(parent) = self.roots.get_mut(root) else {
            return;
        };
        let Some(child) = child else {
            if on_box && !right {
                parent.checked = !parent.checked;
            }
            parent.checked = false;
            return;
        };
        let parent_text = parent.text.clone();
        let Some(node) = parent.children.get_mut(child) else {
            return;
        };
        if on_box && !right {
            node.checked = !node.checked;
        }
        if right {
            node.checked = !node.checked;
        }
        let (checked, text) = (node.checked, node.text.clone());
        let lower = parent_text.to_lowercase();
        let matches =
            |curve: &Curve| curve.label.starts_with(&text) && curve.label.contains(&lower);
        if checked {
            if self.curves.iter().any(matches) {
                return;
            }
            self.graph_item(&parent_text, &text, !right);
        } else {
            let (removed, kept): (Vec<Curve>, Vec<Curve>) = std::mem::take(&mut self.curves)
                .into_iter()
                .partition(matches);
            drop(removed);
            self.curves = kept;
            let visible: Vec<(bool, usize)> = self
                .curves
                .iter()
                .map(|curve| self.axis_of(curve))
                .collect();
            for (index, axis) in self.y_axes.iter_mut().enumerate() {
                if !visible.contains(&(false, index)) {
                    axis.visible = false;
                }
            }
            for (index, axis) in self.y2_axes.iter_mut().enumerate() {
                if !visible.contains(&(true, index)) {
                    axis.visible = false;
                }
            }
        }
    }

    /// `GraphItem`: `rightclick` set, and the field graphed by its name, `roll
    /// mavlink_attitude_t`. `// C#: Log/MavlinkLog.cs:1433-1439`
    fn graph_item(&mut self, parent: &str, text: &str, left_axis: bool) {
        self.right_click = !left_axis;
        let name = format!("{text} mavlink_{}_t", parent.to_lowercase());
        self.checked(&name);
    }

    /// `chk_box_CheckedChanged` with its box ticked: the curve added in the next colour, labelled
    /// with the name less `mavlink_`, its points `datappl`'s - `ATTITUDE`'s roll, pitch and yaw in
    /// degrees - and its axis the one of its units, `MAVLink.GetUnit`'s: on the left the axis
    /// titled with them, made if there is none; on the right likewise, a new right axis hidden as
    /// ZedGraph makes them, and the default right axis shown, the label `-R` after it. Finding an
    /// axis shows the left one at its index, as `GetYAxis` reads the left list until `IsY2Axis` is
    /// set after it.
    /// `// C#: Log/MavlinkLog.cs:887-971; ExtLibs/ZedGraph/ZedGraph/YAxisList.cs:129-140`
    fn checked(&mut self, name: &str) {
        self.selection.push(name.to_owned());
        let colour = COLOUR_VALUES
            .get(self.colour_step % COLOUR_VALUES.len())
            .copied()
            .unwrap_or(COLOUR_VALUES[0]);
        self.colour_step += 1;
        let mut points = self.log.series.get(name).cloned().unwrap_or_default();
        let right = self.right_click;
        let mut y_axis_index = 0;
        let split: Vec<&str> = name.split(' ').collect();
        if let [field, message] = split.as_slice() {
            let field = field.split('[').next().unwrap_or("");
            let message = message.replace("mavlink_", "");
            let message = message
                .strip_suffix("_t")
                .unwrap_or(&message)
                .to_uppercase();
            let unit = self.log.unit(&message, field).to_owned();
            let find = |axes: &[Axis]| {
                axes.iter()
                    .position(|axis| axis.title.to_lowercase() == unit.to_lowercase())
            };
            let shown = |index: usize, axes: &mut Vec<Axis>| {
                let at = if index < axes.len() { index } else { 0 };
                if let Some(axis) = axes.get_mut(at) {
                    axis.visible = true;
                }
            };
            match (find(&self.y_axes), find(&self.y2_axes)) {
                (Some(index), _) if !right => {
                    y_axis_index = index;
                    shown(index, &mut self.y_axes);
                }
                (_, Some(index)) if right => {
                    y_axis_index = index;
                    shown(index, &mut self.y_axes);
                }
                _ if right => {
                    self.y2_axes.push(Axis {
                        title: unit,
                        visible: false,
                    });
                    y_axis_index = self.y2_axes.len() - 1;
                }
                _ => {
                    self.y_axes.push(Axis {
                        title: unit,
                        visible: true,
                    });
                    y_axis_index = self.y_axes.len() - 1;
                }
            }
        }
        if matches!(
            name,
            "roll mavlink_attitude_t" | "pitch mavlink_attitude_t" | "yaw mavlink_attitude_t"
        ) {
            for point in &mut points {
                point.1 *= 180.0 / std::f64::consts::PI;
            }
        }
        let mut label = name.replace("mavlink_", "");
        if right {
            if let Some(axis) = self.y2_axes.first_mut() {
                axis.visible = true;
            }
            label.push_str("-R");
        }
        let mut series = mp_chart::Series::new(label.clone(), points.len().max(1));
        let mut ranges: Option<((f64, f64), (f64, f64))> = None;
        for (x, y) in &points {
            series.push(*x, *y);
            if x.is_finite() && y.is_finite() {
                ranges = Some(match ranges {
                    None => ((*x, *x), (*y, *y)),
                    Some(((x0, x1), (y0, y1))) => {
                        ((x0.min(*x), x1.max(*x)), (y0.min(*y), y1.max(*y)))
                    }
                });
            }
        }
        self.curves.push(Curve {
            tag: name.to_owned(),
            label,
            colour,
            y_axis_index,
            is_y2: right,
            series,
            ranges,
        });
    }

    /// The `x` range over every curve, and each axis's `y` range over the curves drawn against
    /// it: `0` to `1` for none, as `Scale.SetRange` sets an axis with no data.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/Scale.cs:2684-2710`
    #[must_use]
    pub fn ranges(&self) -> Ranges {
        let mut x: Option<(f64, f64)> = None;
        let mut left: Vec<Option<(f64, f64)>> = vec![None; self.y_axes.len()];
        let mut right: Vec<Option<(f64, f64)>> = vec![None; self.y2_axes.len()];
        let widen = |range: &mut Option<(f64, f64)>, (low, high): (f64, f64)| {
            *range = Some(range.map_or((low, high), |(a, b)| (a.min(low), b.max(high))));
        };
        for curve in &self.curves {
            let Some((xs, ys)) = curve.ranges else {
                continue;
            };
            widen(&mut x, xs);
            let (is_y2, index) = self.axis_of(curve);
            let list = if is_y2 { &mut right } else { &mut left };
            if let Some(range) = list.get_mut(index) {
                widen(range, ys);
            }
        }
        let fix = |range: Option<(f64, f64)>| range.unwrap_or((0.0, 1.0));
        (
            fix(x),
            left.into_iter().map(fix).collect(),
            right.into_iter().map(fix).collect(),
        )
    }
}

// ---- The form -----------------------------------------------------------------------------------

/// What the form is asking.
enum Asking {
    /// A button's `OpenFileDialog`, the path typed or chosen in the planner's chooser, and for
    /// Create KML + GPX whether Shift was held.
    Open {
        button: Button,
        path: PathBox,
        shift: bool,
    },
    /// Create KML + GPX with Shift held: `InputBox.Show("Alt offset", ...)`, for the log chosen.
    Offset { file: PathBuf, input: InputBox },
    /// Create KML + GPX once the log is read, with no mission item 0 in it:
    /// `InputBox.Show("Relative Alt", ...)`.
    HomeAlt {
        file: PathBuf,
        flight: Box<FlightData>,
        input: InputBox,
    },
}

/// What a job's thread says.
enum Progress {
    /// `progressBar1.Value`.
    Percent(u8),
    /// Create KML + GPX's replay, done: what `writeKML` is to be given, with the offset typed.
    Flight {
        file: PathBuf,
        flight: Box<FlightData>,
        offset: String,
    },
    /// Graph Log's reading, done.
    Graph(Box<GraphLog>),
    /// The files written.
    Written(Vec<PathBuf>),
    /// What went wrong, for the status line.
    Failed(String),
}

/// A button's work on its thread.
struct Job {
    button: Button,
    receiver: Receiver<Progress>,
}

/// The form: its caption, the progress bar, the graph and its tree, what it is asking and the work
/// running.
pub struct MavlinkLogForm {
    /// `Text`: "Log", then "Log - <file>" once Graph Log has chosen one.
    pub title: String,
    /// `progressBar1.Value`, 0 to 100.
    pub progress: u8,
    /// The graph and its tree.
    pub graph: Graph,
    /// What the last job to finish wrote, for the facts.
    pub written: Vec<PathBuf>,
    asking: Option<Asking>,
    jobs: Vec<Job>,
    /// The boxes' keyboard, made the first time one shows.
    focus: Option<gpui::FocusHandle>,
    tree_scroll: gpui::ScrollHandle,
}

impl Default for MavlinkLogForm {
    fn default() -> Self {
        Self::new()
    }
}

impl MavlinkLogForm {
    /// `new MavlinkLog()`. `// C#: Log/MavlinkLog.cs:34-51`
    #[must_use]
    pub fn new() -> Self {
        Self {
            title: TITLE.to_owned(),
            progress: 0,
            graph: Graph::new(),
            written: Vec::new(),
            asking: None,
            jobs: Vec::new(),
            focus: None,
            tree_scroll: gpui::ScrollHandle::new(),
        }
    }

    /// The buttons whose work is running.
    fn busy(&self) -> Vec<&'static str> {
        self.jobs.iter().map(|job| job.button.text()).collect()
    }
}

/// `BUT_log2kml_Click`: a new form shown, as `new MavlinkLog()` makes one each click.
/// `// C#: GCSViews/FlightData.cs:1305-1310`
pub fn open(this: &mut MissionPlanner) {
    this.mavlink_log = Some(MavlinkLogForm::new());
}

/// The Telemetry Logs page's "Tlog > Kml or Graph" button, for its third row's first cell
/// (`BUT_log2kml` at row 2, column 0 of `tableLayoutPaneltlogs`): 80 by 46, its words wrapped in
/// it, in the 91-pixel column.
/// `// C#: GCSViews/FlightData.Designer.cs:2208, 2342-2351; GCSViews/FlightData.resx (BUT_log2kml)`
pub fn page_button(cx: &mut Context<MissionPlanner>) -> AnyElement {
    crate::probe::measured("fly-log2kml", div())
        .id("fly-log2kml")
        .w(px(80.0))
        .h(px(46.0))
        .flex()
        .items_center()
        .justify_center()
        .rounded_md()
        .border_1()
        .text_xs()
        .text_center()
        .bg(rgb(theme::ACTION))
        .border_color(rgb(theme::ACCENT))
        .text_color(rgb(theme::ACCENT))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        // The words in a box of the button's width, so they wrap onto two lines as WinForms wraps
        // them: as a bare child they ran 106 px wide through the 80 px button and past the
        // window's left edge (the layout guard, 2026-10-17).
        .child(
            div()
                .w_full()
                .px_1()
                .text_center()
                .child(fl!("flightdata-BUT_log2kml-Text")),
        )
        .on_click(cx.listener(|this, _event: &ClickEvent, _window, cx| {
            open(this);
            cx.notify();
        }))
        .into_any_element()
}

/// `Path.GetFileNameWithoutExtension`: a file name up to its last `.`, or all of it.
#[must_use]
pub fn without_extension(name: &str) -> &str {
    name.rfind('.')
        .map_or(name, |at| name.get(..at).unwrap_or(name))
}

/// `Path.GetDirectoryName(logfile) + Path.DirectorySeparatorChar +
/// Path.GetFileNameWithoutExtension(logfile) + extension`: a file beside the log.
#[must_use]
pub fn beside(log: &Path, extension: &str) -> PathBuf {
    let name = log
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    let dir = log.parent().unwrap_or_else(|| Path::new(""));
    dir.join(format!("{}{extension}", without_extension(&name)))
}

/// A button clicked: its dialog, in the log directory where the C# opens it there. A button not
/// ported says why instead (its dimmed click, [`Button::not_ported`]).
/// `// C#: Log/MavlinkLog.cs:53-62, 200-216, 273-298, 1029-1045; Log/MatLabForms.cs:10-26`
fn press(
    this: &mut MissionPlanner,
    button: Button,
    shift: bool,
    window: &mut Window,
    cx: &mut Context<MissionPlanner>,
) {
    let directory = if button.in_log_directory() {
        crate::fly::log_directory()
            .map(|dir| dir.display().to_string())
            .unwrap_or_default()
    } else {
        String::new()
    };
    let Some(form) = this.mavlink_log.as_mut() else {
        return;
    };
    form.asking = Some(Asking::Open {
        button,
        path: PathBox::new(&directory, button.filter()),
        shift,
    });
    let focus = form.focus.get_or_insert_with(|| cx.focus_handle()).clone();
    focus.focus(window, cx);
}

/// A sender's progress reporter: the percentage sent when it changes.
fn reporter(sender: &Sender<Progress>) -> impl FnMut(u8) + '_ {
    let mut last = None;
    move |percent| {
        if last != Some(percent) {
            last = Some(percent);
            let _ = sender.send(Progress::Percent(percent));
        }
    }
}

/// A button's work started on a thread of its own, the progress bar from 0.
fn spawn(
    this: &mut MissionPlanner,
    button: Button,
    work: impl FnOnce(&Sender<Progress>) + Send + 'static,
) {
    let Some(form) = this.mavlink_log.as_mut() else {
        return;
    };
    let (sender, receiver) = std::sync::mpsc::channel();
    form.progress = 0;
    let spawned = wasm_thread::Builder::new()
        .name("mp-mavlink-log".to_owned())
        .spawn(move || work(&sender));
    match spawned {
        Ok(_) => form.jobs.push(Job { button, receiver }),
        Err(error) => log::debug!("Tlog > Kml or Graph: {error}"),
    }
}

/// A conversion that writes one file beside the log: the log read, `convert`ed, written.
fn convert(
    this: &mut MissionPlanner,
    button: Button,
    file: PathBuf,
    extension: &'static str,
    convert: Convert,
) {
    spawn(this, button, move |sender| {
        let Ok(data) = mp_os::fs::read(&file) else {
            let _ = sender.send(Progress::Failed(CANNOT_OPEN.to_owned()));
            return;
        };
        let text = convert(&data, &mp_kml::dflog::local_zone, &mut reporter(sender));
        let out = beside(&file, extension);
        let _ = sender.send(match mp_os::fs::write(&out, text) {
            Ok(()) => Progress::Written(vec![out]),
            Err(error) => Progress::Failed(format!("{}: {error}", out.display())),
        });
    });
}

/// Create KML + GPX's replay of the log, `offset` the offset typed or "0".
/// `// C#: Log/MavlinkLog.cs:80-160`
fn start_kml(this: &mut MissionPlanner, file: PathBuf, offset: String) {
    spawn(this, Button::Kml, move |sender| {
        let Ok(data) = mp_os::fs::read(&file) else {
            let _ = sender.send(Progress::Failed(CANNOT_OPEN.to_owned()));
            return;
        };
        let flight = mavlinklog::flight_data(&data, &mut reporter(sender));
        let _ = sender.send(Progress::Flight {
            file,
            flight: Box::new(flight),
            offset,
        });
    });
}

/// `basealtstring` from the mission's item 0: `(wps[0].z + float.Parse(offsetalt)).ToString()`,
/// read back by `double.Parse` - so the sum to a float's seven digits.
/// `// C#: Log/MavlinkLog.cs:164-167, 189`
#[must_use]
pub fn base_altitude(home_z: f32, offset: &str) -> f64 {
    #[allow(clippy::cast_possible_truncation)] // `float.Parse`
    let offset = netfmt::parse_double(offset).map_or(0.0, |value| value as f32);
    netfmt::parse_double(&netfmt::single(home_z + offset)).unwrap_or(0.0)
}

/// The replay read: with a mission item 0, its altitude and the offset are the home altitude; with
/// none, "Relative Alt" is asked. `// C#: Log/MavlinkLog.cs:162-173`
fn kml_read(this: &mut MissionPlanner, file: PathBuf, flight: FlightData, offset: &str) {
    match flight.home_z {
        Some(z) => {
            let base = base_altitude(z, offset);
            write_kml(this, file, flight, base);
        }
        None => {
            if let Some(form) = this.mavlink_log.as_mut() {
                form.asking = Some(Asking::HomeAlt {
                    file,
                    flight: Box::new(flight),
                    input: InputBox::new(
                        "Relative Alt",
                        "Please enter your home altitude, or press cancel to use absolute alt",
                        "0",
                    ),
                });
            }
        }
    }
}

/// `writeGPX` for each system and `writeKML`, `base` the home altitude over
/// `CurrentState.multiplierdist`. `// C#: Log/MavlinkLog.cs:179-191`
fn write_kml(this: &mut MissionPlanner, file: PathBuf, flight: FlightData, base: f64) {
    let base = base / f64::from(crate::fly::MULTIPLIER_DIST);
    spawn(this, Button::Kml, move |sender| {
        let _ = sender.send(
            match mp_kml::tlog::create_kml(
                &file,
                &flight,
                base,
                crate::PRODUCT_NAME,
                &mp_kml::dflog::local_zone,
            ) {
                Ok(files) => Progress::Written(files),
                Err(error) => Progress::Failed(format!("{}: {error}", file.display())),
            },
        );
    });
}

/// The file dialog answered: OK on a file starts its button's work - Graph Log's caption set first;
/// OK on no file keeps the dialog, as the platform dialog stays; Cancel does nothing.
/// `// C#: Log/MavlinkLog.cs:62-80, 216-218, 298-304, 1045-1047; Log/MatLabForms.cs:26-38`
fn path_answered(this: &mut MissionPlanner, ok: bool) {
    let Some(form) = this.mavlink_log.as_mut() else {
        return;
    };
    let Some(Asking::Open {
        button,
        path,
        shift,
    }) = form.asking.take()
    else {
        return;
    };
    if !ok {
        return;
    }
    let Some(file) = path.chosen() else {
        form.asking = Some(Asking::Open {
            button,
            path,
            shift,
        });
        return;
    };
    match button {
        Button::Kml if shift => {
            form.asking = Some(Asking::Offset {
                file,
                input: InputBox::new("Alt offset", "Please enter your offset altitude", "0"),
            });
        }
        Button::Kml => start_kml(this, file, "0".to_owned()),
        Button::Text => convert(this, button, file, ".txt", mavlinklog::text_file),
        Button::Csv => convert(this, button, file, ".csv", mavlinklog::csv_file),
        Button::Graph => {
            let name = file
                .file_name()
                .map(|name| name.to_string_lossy().into_owned())
                .unwrap_or_default();
            form.title = format!("{TITLE} - {name}");
            spawn(this, button, move |sender| {
                let Ok(data) = mp_os::fs::read(&file) else {
                    let _ = sender.send(Progress::Failed(CANNOT_OPEN.to_owned()));
                    return;
                };
                let log =
                    mavlinklog::graph_log(&data, &mp_kml::dflog::local_zone, &mut reporter(sender));
                let _ = sender.send(Progress::Graph(Box::new(log)));
            });
        }
        Button::Matlab => spawn(this, button, move |sender| {
            let _ = sender.send(
                match mp_log::matlab::process_tlog_file(&file, &mp_kml::dflog::local_zone) {
                    Ok(path) => Progress::Written(vec![path]),
                    // `"Error converting file " + ex.ToString()`.
                    Err(error) => Progress::Failed(format!("Error converting file {error}")),
                },
            );
        }),
        Button::Params | Button::Wps | Button::Cs => {}
    }
}

/// An input box answered. "Alt offset": OK takes the answer, Cancel the "0" it offered, as
/// `InputBox.Show` leaves its value on Cancel - one that is not a number "Bad Offset" and the end
/// of it. "Relative Alt": the same; then `double.Parse`, which throws on one that is not a number
/// where here the status line says so.
/// `// C#: Log/MavlinkLog.cs:64-78, 168-173, 189; ExtLibs/Controls/InputBox.cs:37-59, 143-152`
fn input_answered(this: &mut MissionPlanner, ok: bool) {
    let Some(form) = this.mavlink_log.as_mut() else {
        return;
    };
    let Some(asking) = form.asking.take() else {
        return;
    };
    match asking {
        Asking::Offset { file, input } => {
            let offset = if ok {
                input.remember(&mut this.persisted);
                input.field.value().to_owned()
            } else {
                "0".to_owned()
            };
            if netfmt::parse_double(&offset).is_none() {
                this.file_status = Some(BAD_OFFSET.to_owned());
                return;
            }
            start_kml(this, file, offset);
        }
        Asking::HomeAlt {
            file,
            flight,
            input,
        } => {
            let text = if ok {
                input.remember(&mut this.persisted);
                input.field.value().to_owned()
            } else {
                "0".to_owned()
            };
            match netfmt::parse_double(&text) {
                Some(base) => write_kml(this, file, *flight, base),
                None => {
                    this.file_status = Some(format!("Relative Alt: \"{text}\" is not a number"));
                }
            }
        }
        open @ Asking::Open { .. } => form.asking = Some(open),
    }
}

/// Once a frame: what each job's thread has said - the progress bar, and each job done: Create
/// KML + GPX's replay to its next step, Graph Log's fields into the tree, the files written for
/// the facts, a failure on the status line.
pub fn tick(this: &mut MissionPlanner) {
    let Some(form) = this.mavlink_log.as_mut() else {
        return;
    };
    if form.jobs.is_empty() {
        return;
    }
    crate::repaint::in_flight();
    let mut finished = Vec::new();
    let mut percent = None;
    form.jobs.retain(|job| {
        loop {
            match job.receiver.try_recv() {
                Ok(Progress::Percent(value)) => percent = Some(value),
                Ok(done) => {
                    finished.push((job.button, done));
                    break false;
                }
                Err(TryRecvError::Empty) => break true,
                Err(TryRecvError::Disconnected) => break false,
            }
        }
    });
    if let Some(value) = percent {
        form.progress = value;
    }
    for (button, done) in finished {
        match done {
            Progress::Flight {
                file,
                flight,
                offset,
            } => kml_read(this, file, *flight, &offset),
            Progress::Graph(log) => {
                if let Some(form) = this.mavlink_log.as_mut() {
                    form.graph.load(*log);
                    form.progress = 100;
                }
            }
            Progress::Written(files) => {
                if let Some(form) = this.mavlink_log.as_mut() {
                    form.progress = 100;
                    form.written = files;
                }
            }
            Progress::Failed(text) => {
                if button == Button::Graph
                    && let Some(form) = this.mavlink_log.as_mut()
                {
                    form.graph.not_read();
                }
                this.file_status = Some(text);
            }
            Progress::Percent(_) => {}
        }
    }
}

// ---- Drawing ------------------------------------------------------------------------------------

/// The ids of the form's boxes.
const IDS: crate::config::firmware::BoxIds = crate::config::firmware::BoxIds {
    question: "mavlinklog-question",
    yes: "mavlinklog-question-yes",
    no: "mavlinklog-question-no",
    message: "mavlinklog-message",
    ok: "mavlinklog-message-ok",
    path: "mavlinklog-path",
    path_value: "mavlinklog-path-value",
    path_ok: "mavlinklog-path-ok",
    path_cancel: "mavlinklog-path-cancel",
};

/// A button at its `.resx` place and size, its text wrapped in it as `MyButton` wraps it; dimmed,
/// saying why, where it is not ported.
fn form_button(button: Button, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let live = button
        .not_ported()
        .map_or(Live::Yes, |why| Live::Dimmed(why.into()));
    let base = crate::probe::measured(button.id(), div())
        .id(button.id())
        .absolute()
        .left(px(button.x()))
        .top(px(BUTTON_TOP))
        .w(px(BUTTON_WIDTH))
        .h(px(BUTTON_HEIGHT))
        .flex()
        .items_center()
        .justify_center()
        .rounded_md()
        .border_1()
        .text_size(px(10.0))
        .line_height(px(10.0))
        .text_center()
        .child(button.text());
    if live.is_live() {
        base.bg(rgb(theme::ACTION))
            .border_color(rgb(theme::ACCENT))
            .text_color(rgb(theme::ACCENT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(cx.listener(move |this, _event: &ClickEvent, window, cx| {
                let shift = window.modifiers().shift;
                press(this, button, shift, window, cx);
                cx.notify();
            }))
            .into_any_element()
    } else {
        crate::ui::dimmed_click(
            base.bg(rgb(theme::PANEL))
                .border_color(rgb(theme::BORDER))
                .text_color(rgb(theme::DIM)),
            &live,
        )
        .into_any_element()
    }
}

/// A node's check box: ticked or not; its left press the tree's own toggle, then the handler.
fn check_box(
    id: String,
    checked: bool,
    root: usize,
    child: Option<usize>,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .flex_shrink_0()
        .w(px(11.0))
        .h(px(11.0))
        .border_1()
        .border_color(rgb(theme::TEXT))
        .bg(rgb(if checked { theme::ACCENT } else { theme::BG }))
        .cursor_pointer()
        // A child, so the probe measures the box: it records an element by its children's
        // prepaint, and an empty square has none (logs-mavlinklog.gui found the checks unnamed,
        // 2026-10-17).
        .child(div().size_full())
        .on_mouse_down(
            MouseButton::Left,
            cx.listener(move |this, _event: &MouseDownEvent, _window, cx| {
                if let Some(form) = this.mavlink_log.as_mut() {
                    form.graph.click(root, child, false, true);
                }
                cx.notify();
            }),
        )
        .into_any_element()
}

/// `treeView1`: each message's node, its fields under it while it is expanded - a node's id its
/// message's text and its own, `mavlinklog-node-ATTITUDE-roll`. A message's node expands by its
/// plus or a double click; a right click on a field's node toggles it.
fn tree(form: &MavlinkLogForm, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let mut list = div().flex().flex_col();
    for (r, root) in form.graph.roots.iter().enumerate() {
        let id = format!("mavlinklog-node-{}", root.text);
        let expander = crate::probe::measured(format!("{id}-expand"), div())
            .id(SharedString::from(format!("{id}-expand")))
            .w(px(12.0))
            .flex_shrink_0()
            .text_xs()
            .text_color(rgb(theme::DIM))
            .cursor_pointer()
            .child(if root.expanded { "-" } else { "+" })
            .on_mouse_down(
                MouseButton::Left,
                cx.listener(move |this, _event: &MouseDownEvent, _window, cx| {
                    if let Some(root) = this
                        .mavlink_log
                        .as_mut()
                        .and_then(|form| form.graph.roots.get_mut(r))
                    {
                        root.expanded = !root.expanded;
                    }
                    cx.notify();
                }),
            );
        let label = div()
            .id(SharedString::from(format!("{id}-label")))
            .flex_1()
            .min_w(px(0.0))
            .truncate()
            .child(root.text.clone())
            .on_mouse_down(
                MouseButton::Left,
                cx.listener(move |this, event: &MouseDownEvent, _window, cx| {
                    if event.click_count == 2
                        && let Some(root) = this
                            .mavlink_log
                            .as_mut()
                            .and_then(|form| form.graph.roots.get_mut(r))
                    {
                        root.expanded = !root.expanded;
                    }
                    cx.notify();
                }),
            );
        list = list.child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id.clone()))
                .flex()
                .items_center()
                .gap_1()
                .h(px(NODE_HEIGHT))
                .flex_shrink_0()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(expander)
                .child(check_box(format!("{id}-check"), root.checked, r, None, cx))
                .child(label),
        );
        if !root.expanded {
            continue;
        }
        for (c, node) in root.children.iter().enumerate() {
            let node_id = format!("{id}-{}", node.text);
            list = list.child(
                crate::probe::measured(node_id.clone(), div())
                    .id(SharedString::from(node_id.clone()))
                    .flex()
                    .items_center()
                    .gap_1()
                    .h(px(NODE_HEIGHT))
                    .flex_shrink_0()
                    .pl(px(16.0))
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .on_mouse_down(
                        MouseButton::Right,
                        cx.listener(move |this, _event: &MouseDownEvent, _window, cx| {
                            if let Some(form) = this.mavlink_log.as_mut() {
                                form.graph.click(r, Some(c), true, false);
                            }
                            cx.notify();
                        }),
                    )
                    .child(check_box(
                        format!("{node_id}-check"),
                        node.checked,
                        r,
                        Some(c),
                        cx,
                    ))
                    .child(
                        div()
                            .flex_1()
                            .min_w(px(0.0))
                            .truncate()
                            .child(node.text.clone()),
                    ),
            );
        }
    }
    crate::probe::measured("mavlinklog-tree", div())
        .id("mavlinklog-tree")
        .absolute()
        .left(px(TREE_LEFT))
        .top(px(SPLIT_TOP))
        .w(px(TREE_WIDTH))
        .h(px(SPLIT_HEIGHT))
        .p_1()
        .bg(rgb(theme::BG))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .overflow_y_scroll()
        .track_scroll(&form.tree_scroll)
        .child(list)
        .into_any_element()
}

/// Text placed absolutely in the pane.
fn label(content: String) -> gpui::Div {
    div()
        .absolute()
        .text_xs()
        .whitespace_nowrap()
        .text_color(rgb(theme::TEXT))
        .child(content)
}

/// `zg1`: the title, the legend - each curve's colour and label - the date axis along the bottom,
/// each visible Y axis beside the chart, the first nearest it, the right ones on the right, and
/// each curve against its own axis's scale, whether that axis is drawn or not.
/// `// C#: Log/MavlinkLog.cs:38-48, 887-1000`
fn chart(graph: &Graph) -> AnyElement {
    let ((x_low, x_high), left_ranges, right_ranges) = graph.ranges();
    let x_scale = DateScale::pick(x_low, x_high);
    let left_scales: Vec<Scale> = left_ranges
        .iter()
        .map(|(low, high)| Scale::pick(*low, *high, None))
        .collect();
    let right_scales: Vec<Scale> = right_ranges
        .iter()
        .map(|(low, high)| Scale::pick(*low, *high, None))
        .collect();
    let shown_left: Vec<usize> = (0..graph.y_axes.len())
        .filter(|index| graph.y_axes.get(*index).is_some_and(|axis| axis.visible))
        .collect();
    let shown_right: Vec<usize> = (0..graph.y2_axes.len())
        .filter(|index| graph.y2_axes.get(*index).is_some_and(|axis| axis.visible))
        .collect();
    #[allow(clippy::cast_precision_loss)] // a handful of axes
    let (left, right) = (
        8.0 + AXIS_WIDTH * shown_left.len() as f32,
        8.0 + AXIS_WIDTH * shown_right.len() as f32,
    );
    // The bottom inset holds the X title's label (at plot bottom + 17, 16 high) inside the pane's
    // 291 px of client height: at 34 it ended 1 px below (the layout guard, 2026-10-17).
    let (top, bottom) = (56.0, 36.0);
    let plot_width = (CHART_WIDTH - left - right).max(1.0);
    let plot_height = (SPLIT_HEIGHT - top - bottom).max(1.0);
    #[allow(clippy::cast_possible_truncation)]
    let across = |x: f64| x_scale.fraction(x) as f32;
    let mut pane = crate::probe::measured("mavlinklog-chart", div())
        .id("mavlinklog-chart")
        .absolute()
        .left(px(SPLIT_LEFT))
        .top(px(SPLIT_TOP))
        .w(px(CHART_WIDTH))
        .h(px(SPLIT_HEIGHT))
        .bg(rgb(theme::BG))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .overflow_hidden()
        .child(
            div()
                .absolute()
                .top(px(2.0))
                .left_0()
                .right_0()
                .flex()
                .justify_center()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(PANE_TITLE),
        )
        .child(
            div()
                .absolute()
                .top(px(22.0))
                .left(px(8.0))
                .right(px(8.0))
                .h(px(30.0))
                .overflow_hidden()
                .flex()
                .flex_wrap()
                .justify_center()
                .gap_2()
                .children(graph.curves.iter().map(|curve| {
                    div()
                        .flex()
                        .items_center()
                        .gap_1()
                        .child(div().w(px(20.0)).h(px(2.0)).bg(rgb(curve.colour)))
                        .child(
                            div()
                                .text_xs()
                                .text_color(rgb(theme::TEXT))
                                .child(curve.label.clone()),
                        )
                })),
        )
        .child(
            label(X_TITLE.to_owned())
                .left(px(left))
                .w(px(plot_width))
                .flex()
                .justify_center()
                .top(px(top + plot_height + 17.0)),
        );
    for tic in x_scale.tics() {
        // A tic's label is centred on it and held inside the pane: ZedGraph's chart rect is
        // inset so the edge tics' labels fit the control, where a label 30 px past the plot's
        // edge ran 22 px outside this pane and the layout guard counted the chart cut off
        // (logs-mavlinklog.gui, 2026-10-17).
        let at = (left + across(tic) * plot_width - 30.0).clamp(0.0, CHART_WIDTH - 62.0);
        pane = pane.child(
            label(DateScale::label_in(tic, X_FORMAT))
                .left(px(at))
                .w(px(60.0))
                .flex()
                .justify_center()
                .top(px(top + plot_height + 2.0)),
        );
    }
    // Each Y axis drawn: its labels at its tics and its title over them; the default left axis's
    // title hidden (`YAxis.Title.IsVisible = false`).
    let axis_labels = |pane: gpui::Stateful<gpui::Div>,
                       scale: &Scale,
                       title: Option<String>,
                       at: f32,
                       on_right: bool| {
        #[allow(clippy::cast_possible_truncation)]
        let up = |y: f64| scale.fraction(y) as f32;
        let mut pane = pane;
        for tic in scale.tics() {
            let text = label(scale.label(tic))
                .left(px(at))
                .w(px(AXIS_WIDTH - 4.0))
                .flex()
                .top(px(top + (1.0 - up(tic)) * plot_height - 7.0));
            pane = pane.child(if on_right {
                text.justify_start()
            } else {
                text.justify_end()
            });
        }
        if let Some(title) = title {
            pane = pane.child(
                label(title)
                    .left(px(at))
                    .w(px(AXIS_WIDTH - 4.0))
                    .flex()
                    .justify_center()
                    .top(px(top - 14.0)),
            );
        }
        pane
    };
    for (k, index) in shown_left.iter().enumerate() {
        let (Some(axis), Some(scale)) = (graph.y_axes.get(*index), left_scales.get(*index)) else {
            continue;
        };
        #[allow(clippy::cast_precision_loss)]
        let at = left - AXIS_WIDTH * (k as f32 + 1.0);
        let title = (*index != 0).then(|| scale.title(&axis.title));
        pane = axis_labels(pane, scale, title, at, false);
    }
    for (k, index) in shown_right.iter().enumerate() {
        let (Some(axis), Some(scale)) = (graph.y2_axes.get(*index), right_scales.get(*index))
        else {
            continue;
        };
        #[allow(clippy::cast_precision_loss)]
        let at = left + plot_width + 4.0 + AXIS_WIDTH * k as f32;
        pane = axis_labels(pane, scale, Some(scale.title(&axis.title)), at, true);
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // the plot's pixels
    let columns = plot_width.round().max(1.0) as usize;
    let lines: Vec<crate::plotline::Line> = graph
        .curves
        .iter()
        .filter_map(|curve| {
            let (is_y2, index) = graph.axis_of(curve);
            let scale = if is_y2 {
                right_scales.get(index)
            } else {
                left_scales.get(index)
            }?;
            #[allow(clippy::cast_possible_truncation)]
            let points: Vec<(f32, f32)> =
                mp_chart::trace(&curve.series, x_scale.min, x_scale.max, columns)
                    .into_iter()
                    .map(|(x, y)| (x as f32, (1.0 - scale.fraction(y)) as f32))
                    .collect();
            (!points.is_empty()).then(|| crate::plotline::Line {
                points,
                colour: rgb(curve.colour).into(),
                diamonds: false,
            })
        })
        .collect();
    pane.child(
        div()
            .absolute()
            .left(px(left))
            .top(px(top))
            .w(px(plot_width))
            .h(px(plot_height))
            .border_1()
            .border_color(rgb(theme::BORDER))
            .child(crate::plotline::element(lines)),
    )
    .into_any_element()
}

/// The form, over the page, as `frm.Show()` leaves the page beside it: its caption and close box,
/// the buttons, the progress bar, the graph and the tree; and what it is asking over it.
/// `// C#: Log/MavlinkLog.Designer.cs:29-164`
pub fn window(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = this.mavlink_log.as_ref()?;
    let header = div()
        .flex()
        .items_center()
        .justify_between()
        .child(
            crate::probe::measured("mavlinklog-title", div())
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(form.title.clone()),
        )
        .child(crate::ui::action(
            "mavlinklog-close",
            "\u{2715}",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.mavlink_log = None;
                cx.notify();
            }),
        ));
    let filled = PROGRESS.2 * f32::from(form.progress.min(100)) / 100.0;
    let mut client = div()
        .relative()
        .w(px(FORM_WIDTH))
        .h(px(FORM_HEIGHT))
        .child(
            crate::probe::measured("mavlinklog-progress", div())
                .absolute()
                .left(px(PROGRESS.0))
                .top(px(PROGRESS.1))
                .w(px(PROGRESS.2))
                .h(px(PROGRESS.3))
                .bg(rgb(theme::BG))
                .border_1()
                .border_color(rgb(theme::BORDER))
                .child(div().h_full().w(px(filled)).bg(rgb(theme::OK))),
        )
        .child(chart(&form.graph))
        .child(tree(form, cx));
    for button in Button::ALL {
        client = client.child(form_button(button, cx));
    }
    let body = crate::probe::measured("mavlinklog", div())
        .id("mavlinklog")
        .flex()
        .flex_col()
        .gap_1()
        .p_2()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_md()
        .occlude()
        .child(header)
        .child(client);
    let asking = asking_box(form, window, cx);
    let size = window.viewport_size();
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(
                    ((size.width - px(FORM_WIDTH + 16.0)) / 2.0).max(px(0.0)),
                    px(80.0),
                ))
                .child(div().relative().child(body).children(asking)),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

/// The question the form is asking, over it.
fn asking_box(
    form: &MavlinkLogForm,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let focus = form.focus.as_ref()?;
    match form.asking.as_ref()? {
        Asking::Open { path, .. } => Some(crate::config::firmware::path_box(
            IDS,
            path,
            focus,
            window,
            |this, event| {
                let outcome = match this
                    .mavlink_log
                    .as_mut()
                    .and_then(|form| form.asking.as_mut())
                {
                    Some(Asking::Open { path, .. }) => path.field.key(event),
                    _ => return false,
                };
                match outcome {
                    crate::textfield::KeyOutcome::Submitted => path_answered(this, true),
                    crate::textfield::KeyOutcome::Cancelled => path_answered(this, false),
                    crate::textfield::KeyOutcome::Changed => {}
                    crate::textfield::KeyOutcome::Ignored => return false,
                }
                true
            },
            path_answered,
            cx,
        )),
        Asking::Offset { input, .. } | Asking::HomeAlt { input, .. } => {
            Some(crate::config::optional::input_box(
                "mavlinklog-input-box",
                input,
                focus,
                window,
                |this, event| {
                    let outcome = match this
                        .mavlink_log
                        .as_mut()
                        .and_then(|form| form.asking.as_mut())
                    {
                        Some(Asking::Offset { input, .. } | Asking::HomeAlt { input, .. }) => {
                            input.field.key(event)
                        }
                        _ => return false,
                    };
                    match outcome {
                        crate::textfield::KeyOutcome::Submitted => input_answered(this, true),
                        crate::textfield::KeyOutcome::Cancelled => input_answered(this, false),
                        crate::textfield::KeyOutcome::Changed => {}
                        crate::textfield::KeyOutcome::Ignored => return false,
                    }
                    true
                },
                |this| input_answered(this, true),
                |this| input_answered(this, false),
                cx,
            ))
        }
    }
}

/// `mavlinklog.*`: whether the form shows, its caption, the progress bar, the work running, the
/// question showing, what the last job wrote, the tree - its messages, each one's fields and
/// whether it is expanded, the fields ticked - and the curves, each with its points and axis,
/// and the Y axes drawn on either side.
pub fn record_facts(form: Option<&MavlinkLogForm>) {
    use crate::facts::record;
    record("mavlinklog.open", form.is_some());
    let Some(form) = form else {
        return;
    };
    record("mavlinklog.title", &form.title);
    record("mavlinklog.progress", form.progress);
    let busy = form.busy();
    record(
        "mavlinklog.busy",
        if busy.is_empty() {
            "none".to_owned()
        } else {
            busy.join(",")
        },
    );
    record(
        "mavlinklog.asking",
        match form.asking.as_ref() {
            None => "none".to_owned(),
            Some(Asking::Open { button, .. }) => format!("open: {}", button.text()),
            Some(Asking::Offset { input, .. } | Asking::HomeAlt { input, .. }) => {
                format!("{}: {}", input.title, input.prompt)
            }
        },
    );
    record(
        "mavlinklog.written",
        form.written
            .iter()
            .filter_map(|path| path.file_name())
            .map(|name| name.to_string_lossy().into_owned())
            .collect::<Vec<_>>()
            .join(","),
    );
    let graph = &form.graph;
    record("mavlinklog.tree.roots", graph.roots.len());
    record(
        "mavlinklog.tree",
        graph
            .roots
            .iter()
            .map(|root| root.text.as_str())
            .collect::<Vec<_>>()
            .join(","),
    );
    for root in &graph.roots {
        record(
            format!("mavlinklog.node.{}", root.text),
            root.children
                .iter()
                .map(|node| node.text.as_str())
                .collect::<Vec<_>>()
                .join(","),
        );
        record(
            format!("mavlinklog.node.{}.expanded", root.text),
            root.expanded,
        );
    }
    record(
        "mavlinklog.checked",
        graph
            .roots
            .iter()
            .flat_map(|root| {
                root.children
                    .iter()
                    .filter(|node| node.checked)
                    .map(move |node| format!("{}.{}", root.text, node.text))
            })
            .collect::<Vec<_>>()
            .join(","),
    );
    record("mavlinklog.curves", graph.curves.len());
    // Each curve's `Tag`, the name it was graphed by (MavlinkLog.cs:945), in the pane's order.
    record(
        "mavlinklog.curves.tags",
        graph
            .curves
            .iter()
            .map(|curve| curve.tag.as_str())
            .collect::<Vec<_>>()
            .join(","),
    );
    for (index, curve) in graph.curves.iter().enumerate() {
        let (is_y2, axis) = graph.axis_of(curve);
        let list = if is_y2 { &graph.y2_axes } else { &graph.y_axes };
        record(
            format!("mavlinklog.curve.{index}"),
            format!(
                "{} | {} points | {} {}",
                curve.label,
                curve.series.len(),
                if is_y2 { "right" } else { "left" },
                list.get(axis).map_or("", |axis| axis.title.as_str())
            ),
        );
    }
    let shown = |axes: &[Axis]| {
        axes.iter()
            .filter(|axis| axis.visible)
            .map(|axis| axis.title.as_str())
            .collect::<Vec<_>>()
            .join(",")
    };
    record("mavlinklog.axes.left", shown(&graph.y_axes));
    record("mavlinklog.axes.right", shown(&graph.y2_axes));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A graph of a log's options and series, as Graph Log reads them.
    fn loaded(
        options: &[&str],
        series: &[(&str, Vec<(f64, f64)>)],
        units: &[(&str, &str)],
    ) -> Graph {
        let mut graph = Graph::new();
        graph.load(GraphLog {
            options: options.iter().map(|option| (*option).to_owned()).collect(),
            series: series
                .iter()
                .map(|(key, points)| ((*key).to_owned(), points.clone()))
                .collect(),
            units: units
                .iter()
                .map(|(key, unit)| ((*key).to_owned(), (*unit).to_owned()))
                .collect(),
        });
        graph
    }

    /// The buttons as the `.resx` places and names them, the three not ported dimmed with why.
    #[test]
    fn the_buttons_are_the_resx_ones_in_its_order() {
        let texts: Vec<&str> = Button::ALL.iter().map(|button| button.text()).collect();
        assert_eq!(
            texts,
            [
                "Create KML + GPX",
                "Convert to Text",
                "Convert to CSV",
                "Graph Log",
                "Extract Params",
                "Extract WPs",
                "Extract CS",
                "Create Matlab file"
            ]
        );
        let xs: Vec<f32> = Button::ALL.iter().map(|button| button.x()).collect();
        assert_eq!(xs, [129.0, 203.0, 277.0, 351.0, 425.0, 499.0, 573.0, 647.0]);
        assert_eq!(Button::Matlab.filter(), "*.tlog|*.tlog");
        assert_eq!(Button::Graph.filter(), "Telemetry Log|*.tlog;*.tlog.*");
        assert!(!Button::Kml.in_log_directory());
        assert!(!Button::Graph.in_log_directory());
        assert!(Button::Text.in_log_directory());
        let dimmed: Vec<Button> = Button::ALL
            .into_iter()
            .filter(|button| button.not_ported().is_some())
            .collect();
        assert_eq!(dimmed, [Button::Params, Button::Wps, Button::Cs]);
    }

    /// The tree: the options sorted as the culture sorts them, a root per message - `ATTITUDE` for
    /// `mavlink_attitude_t` - its fields under it in that order, an array's elements each.
    #[test]
    fn the_tree_is_a_root_per_message_its_fields_under_it() {
        let graph = loaded(
            &[
                "mavlink_statustext_t.text[1]",
                "mavlink_attitude_t.roll",
                "mavlink_gps_raw_int_t.alt",
                "mavlink_attitude_t.pitch",
                "mavlink_statustext_t.text[10]",
                "mavlink_statustext_t.text[0]",
                "mavlink_gps2_raw_t.alt",
                "mavlink_custom_t.custom",
            ],
            &[],
            &[],
        );
        let roots: Vec<&str> = graph.roots.iter().map(|root| root.text.as_str()).collect();
        assert_eq!(
            roots,
            [
                "ATTITUDE",
                "CUSTOM",
                "GPS_RAW_INT",
                "GPS2_RAW",
                "STATUSTEXT"
            ]
        );
        let fields: Vec<&str> = graph.roots[4]
            .children
            .iter()
            .map(|node| node.text.as_str())
            .collect();
        assert_eq!(fields, ["text[0]", "text[1]", "text[10]"]);
        assert!(graph.roots.iter().all(|root| !root.expanded));
        assert_eq!(root_text("mavlink_attitude_target_t"), "ATTITUDE_TARGET");
        assert_eq!(root_text("mavlink_custom_t"), "CUSTOM");
    }

    /// A field's box ticked graphs it on the left axis of its units, made the first time: roll in
    /// degrees, labelled without `mavlink_`, in the first colour; a second field of the same units
    /// on the same axis, in the next colour.
    #[test]
    fn a_ticked_field_is_graphed_on_the_left_axis_of_its_units() {
        let mut graph = loaded(
            &["mavlink_attitude_t.roll", "mavlink_attitude_t.pitch"],
            &[
                ("roll mavlink_attitude_t", vec![(1.0, 0.5), (2.0, -0.5)]),
                ("pitch mavlink_attitude_t", vec![(1.0, 0.25)]),
            ],
            &[("ATTITUDE.roll", "[rad]"), ("ATTITUDE.pitch", "[rad]")],
        );
        // ATTITUDE's fields, sorted: pitch, roll.
        graph.click(0, Some(1), false, true);
        assert!(graph.roots[0].children[1].checked);
        assert_eq!(graph.curves.len(), 1);
        let roll = &graph.curves[0];
        assert_eq!(roll.label, "roll attitude_t");
        assert_eq!(roll.tag, "roll mavlink_attitude_t");
        assert_eq!(roll.colour, 0xFF_0000);
        // `ppl[a].Y * (180.0 / Math.PI)`.
        assert_eq!(
            roll.series.get(0).unwrap().value,
            0.5 * (180.0 / std::f64::consts::PI)
        );
        assert_eq!(graph.axis_of(roll), (false, 1));
        assert_eq!(
            graph.y_axes,
            [
                Axis {
                    title: "Y Axis".to_owned(),
                    visible: true
                },
                Axis {
                    title: "[rad]".to_owned(),
                    visible: true
                }
            ]
        );
        graph.click(0, Some(0), false, true);
        assert_eq!(graph.curves[1].colour, 0x00_FF00);
        assert_eq!(graph.axis_of(&graph.curves[1]), (false, 1));
        assert_eq!(graph.y_axes.len(), 2);
        assert_eq!(graph.selection.len(), 2);
    }

    /// A right click toggles a field and graphs it on the right: a new right axis of its units
    /// made hidden, as ZedGraph's are, and the default right axis shown; `-R` after its label.
    /// Unticked, its curve goes, and every axis no curve is drawn against is hidden.
    #[test]
    fn a_right_click_graphs_on_the_right_and_unticking_hides_unused_axes() {
        let mut graph = loaded(
            &["mavlink_attitude_t.roll", "mavlink_attitude_t.pitch"],
            &[
                ("roll mavlink_attitude_t", vec![(1.0, 0.5)]),
                ("pitch mavlink_attitude_t", vec![(1.0, 0.25)]),
            ],
            &[("ATTITUDE.roll", "[rad]"), ("ATTITUDE.pitch", "[rad]")],
        );
        graph.click(0, Some(1), false, true);
        graph.click(0, Some(0), true, false);
        assert!(graph.roots[0].children[0].checked);
        let pitch = &graph.curves[1];
        assert_eq!(pitch.label, "pitch attitude_t-R");
        assert_eq!(graph.axis_of(pitch), (true, 1));
        assert_eq!(
            graph.y2_axes,
            [
                Axis {
                    title: String::new(),
                    visible: true
                },
                Axis {
                    title: "[rad]".to_owned(),
                    visible: false
                }
            ]
        );
        // roll unticked: its curve gone; only pitch's right axis is drawn against, and it was
        // hidden, so nothing is shown.
        graph.click(0, Some(1), false, true);
        assert_eq!(graph.curves.len(), 1);
        assert!(graph.y_axes.iter().all(|axis| !axis.visible));
        assert!(!graph.y2_axes[0].visible);
    }

    /// The label tests the C# makes: ticking `roll` beside a graphed `rollspeed` graphs nothing,
    /// its label starting `roll`; unticking `roll` takes `rollspeed` too.
    #[test]
    fn roll_and_rollspeed_are_one_to_the_labels_test() {
        let mut graph = loaded(
            &["mavlink_attitude_t.roll", "mavlink_attitude_t.rollspeed"],
            &[
                ("roll mavlink_attitude_t", vec![(1.0, 0.5)]),
                ("rollspeed mavlink_attitude_t", vec![(1.0, 0.1)]),
            ],
            &[],
        );
        graph.click(0, Some(1), false, true);
        assert_eq!(graph.curves.len(), 1);
        graph.click(0, Some(0), false, true);
        assert!(graph.roots[0].children[0].checked);
        assert_eq!(
            graph.curves.len(),
            1,
            "already graphed, to the label's test"
        );
        graph.click(0, Some(0), false, true);
        assert!(graph.curves.is_empty());
    }

    /// A message's box does not stay ticked; a label's left click graphs nothing; a unitless field
    /// on the left gets an axis titled "" of its own, as the default is titled "Y Axis".
    #[test]
    fn a_message_box_unticks_and_a_unitless_field_gets_its_own_axis() {
        let mut graph = loaded(
            &["mavlink_custom_t.custom"],
            &[("custom mavlink_custom_t", vec![(1.0, 2.0)])],
            &[],
        );
        graph.click(0, None, false, true);
        assert!(!graph.roots[0].checked);
        graph.click(0, Some(0), false, false);
        assert!(graph.curves.is_empty());
        graph.click(0, Some(0), false, true);
        assert_eq!(graph.axis_of(&graph.curves[0]), (false, 1));
        assert_eq!(graph.y_axes[1].title, "");
        let ((x0, x1), left, _) = graph.ranges();
        assert_eq!((x0, x1), (1.0, 1.0));
        assert_eq!(left[1], (2.0, 2.0));
        assert_eq!(left[0], (0.0, 1.0));
    }

    /// A new Graph Log clears the curves and begins the colours again; the axes stay.
    #[test]
    fn a_new_graph_log_clears_the_curves_and_keeps_the_axes() {
        let mut graph = loaded(
            &["mavlink_attitude_t.roll"],
            &[("roll mavlink_attitude_t", vec![(1.0, 0.5)])],
            &[("ATTITUDE.roll", "[rad]")],
        );
        graph.click(0, Some(0), false, true);
        graph.load(GraphLog {
            options: vec!["mavlink_vfr_hud_t.alt".to_owned()],
            series: std::collections::HashMap::new(),
            units: std::collections::HashMap::new(),
        });
        assert!(graph.curves.is_empty());
        assert_eq!(graph.roots[0].text, "VFR_HUD");
        assert_eq!(graph.y_axes.len(), 2);
        assert_eq!(graph.colour_step, 0);
    }

    /// Files beside the log, as `GetFileNameWithoutExtension` names them; Create KML's home
    /// altitude to a float's seven digits.
    #[test]
    fn files_are_named_beside_the_log() {
        assert_eq!(
            beside(Path::new("/logs/flight.tlog"), ".txt"),
            PathBuf::from("/logs/flight.txt")
        );
        assert_eq!(
            beside(Path::new("/logs/flight.tlog.1"), ".csv"),
            PathBuf::from("/logs/flight.tlog.csv")
        );
        assert_eq!(without_extension(".tlog"), "");
        assert_eq!(base_altitude(584.25, "0"), 584.25);
        assert_eq!(base_altitude(584.25, "10.5"), 594.75);
        assert_eq!(base_altitude(0.1, "0.2"), 0.3);
    }
}
