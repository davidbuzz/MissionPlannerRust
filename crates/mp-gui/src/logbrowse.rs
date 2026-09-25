//! Reviewing a dataflash log.
//!
//! Ported from `Log/LogBrowse.cs` @ efb0801 (GPL-3.0-or-later). Mission Planner opens it as a
//! separate window from a button on the flight screen; a single-window application makes it a tab.
//! What it holds is the same: a list of the fields the log declares, a chart of the chosen ones on
//! two axes, the map of where the vehicle went beside the chart, and the log's records in a grid
//! under it, whose current cell the Graph Left and Graph Right buttons put on the chart.
//!
//! Over the chart go the labels the check boxes under it choose - mode changes, errors, messages,
//! events, and minutes when the x axis is line numbers - and a double click on it moves a cursor
//! to the record under the pointer, a marker on the map to where the nearest position record
//! says the vehicle was, and the grid to that record.
//!
//! The strip's Show Params lists the parameters the log carries in the Full Parameter List's
//! columns, and its drop-down graphs the sets Mission Planner ships in its `graphs` directory.
//! The chart zooms and pans as ZedGraph does, and its context menu turns on the value of the
//! point under the pointer; the grid's menu exports its rows, and the files the log carries.
//!
//! The extraction is in `mp_log::plot`, the record index in `mp_log::index`, the routes in
//! `mp_log::track`, the labels and the cursor's lookups in `mp_log::overlay`, the parameters in
//! `mp_log::logparams`, the preselected graphs in `mp_log::mavgraph` and `mp_log::expression`,
//! and the reduction in `mp_chart`, all of which have their own tests and no gpui in them. The
//! grid's model is [`grid`], the chart's zoom [`view`], a field's scaler [`modifier`], the grid's
//! menu [`export`], and [`coverage`] ledgers the whole window against its designer. This is the
//! screen.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

pub mod coverage;
mod export;
mod grid;
pub mod metadata;
mod modifier;
#[cfg(test)]
mod ported_tests;
// The point search and its tooltip, which the FFT window's graphs use too.
pub(crate) mod view;

use std::cell::{Cell, RefCell};
use std::collections::BTreeMap;
use std::rc::Rc;
use std::sync::Arc;

use gpui::{
    AnyElement, Bounds, Context, MouseButton, Pixels, SharedString, canvas, div, prelude::*, px,
    relative, rgb, rgba,
};
use mp_chart::Series;
use mp_log::logparams::LogParam;
use mp_log::mavgraph::DisplayList;
use mp_log::overlay::{Firmware, LineAtTime, Mark, Overlays, Positions};
use mp_log::plot::{FieldUnit, PlottableField, UnitTable};
use mp_log::track::{RouteKind, Routes};
use mp_tiles::store::TileStore;
use mp_units::LatLon;

use crate::MissionPlanner;
use crate::mapview::MapViewport;
use crate::ui::{action, panel, theme};
use grid::{Grid, ROW_HEIGHT, TYPE_COLUMN};
use modifier::Modifier;
use view::{Scales, Zoom};

/// How many fields the list shows before it stops.
///
/// A real log declares six hundred; showing all of them makes a list nobody scrolls to the bottom
/// of. The search box is how a field is found, which is the same answer the parameter screen
/// reached for the same reason.
const SHOWN_FIELDS: usize = 200;

/// How wide the plot is, in columns.
const COLUMNS: usize = 240;

/// Which axis a series is drawn against.
///
/// `LogBrowse` has two. A field goes on the left with a left click in its tree and on the right
/// with a right click - `BUT_Graphit` and `BUT_Graphit_R` do the same for the data grid - and
/// the difference matters as soon as two fields have different magnitudes: roll in degrees beside
/// a battery voltage flattens both on one axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    /// The left axis, of which there is one per unit.
    Left,
    /// The right axis, shared by everything on it.
    Right,
}

/// What the chart's x axis counts: `chk_time`.
///
/// Ticked, as the C# opens, every record sits at its time; unticked, at its line number - its
/// place among the log's records, which is the grid's row. Changing it clears the chart, since a
/// curve drawn against one cannot be read against the other.
/// `// C#: Log/LogBrowse.cs:3534-3570`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum XAxis {
    /// `Time (sec)`: seconds from the log's first timestamp.
    Time,
    /// `Line Number`.
    Line,
}

impl XAxis {
    /// The axis title the C# gives it.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::Time => "Time (sec)",
            Self::Line => "Line Number",
        }
    }
}

/// A check box in the strip between the chart and the grid.
///
/// The strip's boxes in the C#'s order - by `Location.X` in the resx, left to right - and with
/// its text. Not here: Show Params, which is a box only in looks - it unticks itself and shows
/// the parameters, a button in all but name - and the preselected graphs' drop-down between it
/// and Mode; the strip draws both in their places.
/// `// C#: Log/LogBrowse.designer.cs:261-332; Log/LogBrowse.resx (CHK_map 401, chk_time 454,
/// chk_datagrid 509, chk_params 594, CMB_preselect 691, chk_mode 797, chk_errors 856,
/// chk_msg 915, chk_events 971)`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Check {
    /// `CHK_map`: shows the map beside the chart.
    Map,
    /// `chk_time`: time or line numbers along the x axis.
    Time,
    /// `chk_datagrid`: shows the grid under the strip.
    DataTable,
    /// `chk_mode`: `DrawModes`.
    Mode,
    /// `chk_errors`: `DrawErrors`.
    Errors,
    /// `chk_msg`: `DrawMSG`.
    Msg,
    /// `chk_events`: `DrawEV`.
    Events,
}

impl Check {
    /// Every box, left to right.
    pub const STRIP: [Self; 7] = [
        Self::Map,
        Self::Time,
        Self::DataTable,
        Self::Mode,
        Self::Errors,
        Self::Msg,
        Self::Events,
    ];

    /// Its text. `// C#: Log/LogBrowse.resx`
    #[must_use]
    pub const fn text(self) -> &'static str {
        match self {
            Self::Map => "Map",
            Self::Time => "Time",
            Self::DataTable => "Data Table",
            Self::Mode => "Mode",
            Self::Errors => "Errors",
            Self::Msg => "MSG",
            Self::Events => "Events",
        }
    }

    /// The name a test clicks it by, and publishes its state under.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Map => "log-chk-map",
            Self::Time => "log-chk-time",
            Self::DataTable => "log-chk-datagrid",
            Self::Mode => "log-chk-mode",
            Self::Errors => "log-chk-errors",
            Self::Msg => "log-chk-msg",
            Self::Events => "log-chk-events",
        }
    }

    /// Whether it is ticked when the log browser opens: the designer's `Checked`, and the
    /// defaults `LoadLog2` reads the remembered settings with.
    /// `// C#: Log/LogBrowse.designer.cs:271, 287, 296, 305, 321; Log/LogBrowse.cs:444-449`
    const fn default(self) -> bool {
        !matches!(self, Self::Map | Self::DataTable)
    }

    /// The key `LoadLog2` reads the box from and its handler writes it to, for the six it
    /// remembers; Events is not remembered.
    /// `// C#: Log/LogBrowse.cs:444-460`
    #[must_use]
    pub const fn setting(self) -> Option<&'static str> {
        Some(match self {
            Self::Map => "LB_Map",
            Self::Time => "LB_Time",
            Self::DataTable => "LB_Grid",
            Self::Mode => "LB_Mode",
            Self::Errors => "LB_Error",
            Self::Msg => "LB_MSG",
            Self::Events => return None,
        })
    }
}

/// `Settings.GetBoolean`: `bool.TryParse`, which takes `True` or `False` in any case with space
/// around it, else the default.
/// `// C#: ExtLibs/Utilities/Settings.cs:223-232`
fn setting_bool(text: Option<&str>, default: bool) -> bool {
    match text.map(|text| text.trim().to_ascii_lowercase()).as_deref() {
        Some("true") => true,
        Some("false") => false,
        _ => default,
    }
}

/// `bool.ToString()`, which is what the handlers write.
const fn setting_text(on: bool) -> &'static str {
    if on { "True" } else { "False" }
}

/// Which of the strip's boxes are ticked.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Strip([bool; 7]);

impl Default for Strip {
    fn default() -> Self {
        Self(Check::STRIP.map(Check::default))
    }
}

impl Strip {
    /// Whether a box is ticked.
    #[must_use]
    pub fn get(self, check: Check) -> bool {
        self.0.get(check as usize).copied().unwrap_or(false)
    }

    fn set(&mut self, check: Check, on: bool) {
        if let Some(ticked) = self.0.get_mut(check as usize) {
            *ticked = on;
        }
    }
}

/// One series on the chart.
#[derive(Debug)]
pub struct Plotted {
    /// The field it came from; `None` for a preselected graph's expression.
    pub field: Option<PlottableField>,
    /// Its samples, scaled into its unit.
    pub series: Series,
    /// Its samples' order along x, for Show Point Values' search.
    pub order: view::TimeOrder,
    /// Which axis it is drawn against.
    pub axis: Axis,
    /// The unit the left axis groups by; empty when the log declares none.
    pub unit: String,
    /// What the legend calls it: `ATT.Roll (deg)`, with ` R` appended on the right axis.
    pub label: String,
    /// The bit of a bitmask field it is, by its node's text: `None` for the field itself.
    pub bit: Option<String>,
}

/// One child node of a bitmask field in the tree: `add_field_node`'s `new_bit_node`.
/// `// C#: Log/LogBrowse.cs:687-696`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BitNode {
    /// `Text`: `bit.name`, which is null - an empty node - for a bit without one.
    pub text: String,
    /// `ToolTipText`: `bit.description`, empty without one.
    pub tip: String,
    /// The mask `GraphItem` finds for the node: the first of the field's bits whose `name` is
    /// the node's text. `None` for a nameless bit, whose empty text no null name equals - the C#
    /// then graphs the field's raw value under the bit's label.
    /// `// C#: Log/LogBrowse.cs:1237-1248`
    pub mask: Option<u32>,
}

/// The bits of a field whose metadata is a bitmask, as `add_field_node` adds them: one node for
/// each bit, in the metadata's order; nothing for a field that is not a bitmask.
/// `// C#: Log/LogBrowse.cs:687-696, 1237-1248`
#[must_use]
pub fn bit_nodes(meta: &metadata::MetaData, message: &str, field: &str) -> Vec<BitNode> {
    let Some(bits) = meta
        .field(message, field)
        .and_then(|known| known.bitmask.as_ref())
    else {
        return Vec::new();
    };
    bits.iter()
        .map(|bit| {
            let text = bit.name.clone().unwrap_or_default();
            BitNode {
                mask: bits
                    .iter()
                    .find(|item| item.name.as_deref() == Some(text.as_str()))
                    .map(|item| item.mask),
                tip: bit.description.clone().unwrap_or_default(),
                text,
            }
        })
        .collect()
}

impl Plotted {
    /// Builds the series, scaled into its unit, labelled as `LogBrowse` labels a curve.
    ///
    /// The multiplier is applied here rather than in the extraction, as the C# applies it in
    /// `GraphItem_AddCurve` rather than in the log reader, so the raw values stay raw for anything
    /// that exports them. The label is `MSG.Field (unit)`, and a right-axis curve gets ` R` -
    /// which is also how the C# finds a curve to remove, by that prefix. Each sample goes at its
    /// time or its line, as the Time box says.
    /// `// C#: Log/LogBrowse.cs:1551-1561, 1605-1630, 2787-2792`
    #[cfg(test)]
    fn new(
        field: PlottableField,
        points: &[mp_log::plot::Point],
        unit: &FieldUnit,
        axis: Axis,
        x_axis: XAxis,
    ) -> Self {
        Self::modified(field, points, unit, axis, x_axis, None)
    }

    /// [`Self::new`] with the field's scaler and offset, if it has one: applied to each value
    /// before the unit's multiplier, as `GraphItem_GetList` applies it before
    /// `GraphItem_AddCurve` multiplies, and its text added to the label after the unit.
    /// `// C#: Log/LogBrowse.cs:1236-1241, 1530-1556, 1605-1630`
    fn modified(
        field: PlottableField,
        points: &[mp_log::plot::Point],
        unit: &FieldUnit,
        axis: Axis,
        x_axis: XAxis,
        modifier: Option<&Modifier>,
    ) -> Self {
        let extra = modifier.map_or("", |modifier| modifier.command.as_str());
        Self::built(field, points, unit, axis, x_axis, modifier, extra, None)
    }

    /// A bit of a bitmask field: its raw values through the bit's mask, `DataModifer(mask)` -
    /// shifted down to the mask's lowest bit, so a one-bit mask graphs 0 or 1 - then the unit's
    /// multiplier as for any field; labelled `MSG.Field (unit)` then `extra_label`, which for a
    /// bit is `"." + bitmask` and the node's tooltip after a space.
    ///
    /// Divergence: the C#'s label ends in that space when the bit has no description; ours
    /// does not, as a field's label here drops the space before its own empty tooltip.
    /// `// C#: Log/LogBrowse.cs:1210-1224, 1237-1248, 1528-1538, 1616-1627`
    fn of_bit(
        field: PlottableField,
        points: &[mp_log::plot::Point],
        unit: &FieldUnit,
        axis: Axis,
        x_axis: XAxis,
        node: &BitNode,
    ) -> Self {
        let mut extra = format!(".{}", node.text);
        if !node.tip.is_empty() {
            extra.push(' ');
            extra.push_str(&node.tip);
        }
        let modifier = node.mask.map(Modifier::mask);
        Self::built(
            field,
            points,
            unit,
            axis,
            x_axis,
            modifier.as_ref(),
            &extra,
            Some(node.text.clone()),
        )
    }

    /// The curve, with `extra_label` after the unit and before the right axis's ` R`.
    /// `// C#: Log/LogBrowse.cs:1530-1556, 1605-1630, 2786-2792`
    #[allow(clippy::too_many_arguments)] // `GraphItem_AddCurve`'s own seven, and the bit
    fn built(
        field: PlottableField,
        points: &[mp_log::plot::Point],
        unit: &FieldUnit,
        axis: Axis,
        x_axis: XAxis,
        modifier: Option<&Modifier>,
        extra: &str,
        bit: Option<String>,
    ) -> Self {
        let mut label = field.to_string();
        if !unit.unit.is_empty() {
            label.push_str(&format!(" ({})", unit.unit));
        }
        label.push_str(extra);
        if axis == Axis::Right {
            label.push_str(" R");
        }
        let mut series = Series::new(label.clone(), points.len().max(1));
        for point in points {
            #[allow(clippy::cast_precision_loss)] // a line number is far below 2^53
            let x = match x_axis {
                XAxis::Time => point.seconds,
                XAxis::Line => point.line as f64,
            };
            let value = modifier.map_or(point.value, |modifier| modifier.apply(point.value));
            series.push(x, value * unit.multiplier);
        }
        Self {
            field: Some(field),
            order: view::TimeOrder::of(&series),
            series,
            axis,
            unit: unit.unit.clone(),
            label,
            bit,
        }
    }

    /// A preselected graph's piece, evaluated: labelled as `GraphItem_AddCurve` labels an
    /// expression - its text and a full stop, ` R` on the right - on the unit-less axis, since
    /// `isexpression` forces the unit empty "so precaned graphs draw on a singel axis", and
    /// unscaled, since an expression's text is no field `GetUnit` knows.
    /// `// C#: Log/LogBrowse.cs:1283-1296, 1597-1630`
    fn expression(
        item: &mp_log::mavgraph::DisplayItem,
        samples: &[mp_log::expression::Sample],
        origin: Option<f64>,
        x_axis: XAxis,
    ) -> Self {
        let label = item.label();
        let mut series = Series::new(label.clone(), samples.len().max(1));
        for sample in samples {
            #[allow(clippy::cast_precision_loss)] // a line number is far below 2^53
            let x = match x_axis {
                XAxis::Time => match (origin, sample.time_us) {
                    (Some(origin), Some(time)) => mp_log::plot::seconds_since(origin, time),
                    _ => continue,
                },
                XAxis::Line => sample.line as f64,
            };
            series.push(x, sample.value);
        }
        Self {
            field: None,
            order: view::TimeOrder::of(&series),
            series,
            axis: if item.left { Axis::Left } else { Axis::Right },
            unit: String::new(),
            label,
            bit: None,
        }
    }
}

/// What the log's map shows, counted, for the facts and the legend.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct MapContents {
    /// Which route is drawn as the track: `GPS`, `POS`, or nothing.
    pub source: &'static str,
    /// Points of that route.
    pub points: usize,
    /// Waypoints of the mission the vehicle logged.
    pub waypoints: usize,
}

/// What the log screen keeps between frames.
pub struct LogBrowse {
    /// The log open, read and indexed: `logdata`, the C#'s `DFLogBuffer`.
    log: Option<Rc<mp_log::logfile::LogFile>>,
    /// The file that was opened, if one was.
    path: Option<std::path::PathBuf>,
    /// Everything that log declares as plottable.
    fields: Vec<PlottableField>,
    /// The units it declares for them.
    units: UnitTable,
    /// The series being plotted.
    plotted: Vec<Plotted>,
    /// The last thing that happened, so a refusal is never silent.
    status: Option<String>,
    /// The last refusal, until something succeeds: the C#'s message box, kept for a test to read.
    refused: Option<String>,
    /// The records, a screenful at a time: `dataGridView1`.
    grid: Option<Grid>,
    /// The map beside the chart: `myGMAP1`. The flight screen's map widget, a second instance.
    map: Rc<RefCell<MapViewport>>,
    /// What is on it.
    map_contents: MapContents,
    /// The imagery it shows, the flight map's provider as `myGMAP1.MapProvider` is.
    tiles: Option<Arc<TileStore>>,
    /// The check boxes under the chart.
    strip: Strip,
    /// Whether the grid has been shown since the log was opened: until `chk_datagrid` is ticked
    /// the C#'s grid has no rows at all.
    grid_filled: bool,
    /// Everything the chart can be labelled with, read when the log is opened.
    overlays: Overlays,
    /// The records a double click can find a place in.
    positions: Positions,
    /// The log's first `TimeUS`, where the time axis starts.
    origin: Option<f64>,
    /// Whether the chart holds its labels: `GraphObjList` after `zg1_ZoomEvent` drew them and
    /// before anything cleared it.
    labelled: bool,
    /// The cursor: the line `GoToSample` last put `m_cursorLine` on, until the labels are redrawn
    /// or the chart cleared, either of which empties `GraphObjList` and takes it with them.
    cursor: Option<usize>,
    /// `markeroverlay`'s one marker: where the record nearest the cursor says the vehicle was.
    marker: Option<LatLon>,
    /// Where the chart's plotting area was last laid out, in window coordinates, so a double
    /// click can be turned into a place on its axis.
    chart_bounds: Rc<Cell<Option<Bounds<Pixels>>>>,
    /// Every route the log holds, read when it is opened.
    routes: Routes,
    /// What `DrawMap` last put on the map: the routes between the lines the chart shows.
    drawn: Rc<Routes>,
    /// `ZoomAndCenterRoutes` asked for and not yet done: the map fits the drawn routes the next
    /// time it is painted, when it knows its size.
    fit_routes: Rc<Cell<bool>>,
    /// The log's parameters, as Show Params collects them.
    params: Vec<LogParam>,
    /// The parameter list on screen, if Show Params has put it there.
    params_view: Option<ParamsView>,
    /// `CMB_preselect`'s items: `mavgraph.graphs`, sorted.
    graphs: Vec<DisplayList>,
    /// `CMB_preselect.SelectedIndex`.
    preselect: Option<usize>,
    /// Whether the drop-down's list is open, and the first item it shows.
    preselect_list: Option<usize>,
    /// What the last selection graphed and what it could not.
    preselect_outcome: Option<(usize, Vec<String>)>,
    /// The chart's zoom.
    zoom: Zoom,
    /// A rectangle being dragged, from and to, as fractions of the plotting area.
    zoom_drag: Option<((f64, f64), (f64, f64))>,
    /// `IsShowPointValues`: off, as ZedGraph starts.
    point_values: bool,
    /// The pointer over the plotting area, as a fraction of it.
    pointer: Option<(f64, f64)>,
    /// The chart's context menu, open at a fraction of the plotting area.
    chart_menu: Option<(f64, f64)>,
    /// The grid's context menu, `contextMenuStrip1`, open.
    grid_menu: bool,
    /// An `InputBox` or a file dialog, standing in the window.
    prompt: Option<Prompt>,
    /// `dataModifierHash`: each field's scaler and offset, by node name.
    modifiers: BTreeMap<String, Modifier>,
    /// What the last export wrote: rows of Export Visible, or files of Export Files.
    exported: Option<String>,
    /// The time of the record the cursor was last put on from the grid, which need not be a
    /// position record.
    cursor_time: Option<f64>,
    /// `txt_info`: the description of the field the pointer last rested on, in a multi-line box
    /// that can be selected, copied from and typed in - the C# never reads it back.
    info: crate::textfield::TextField,
    /// The child nodes `add_field_node` gave each bitmask field when the tree was built, by
    /// message and field: the same for every instance's node of the field.
    bits: BTreeMap<(String, String), Vec<BitNode>>,
    /// The field nodes expanded to show their bits, by the field's name (`MSG.Field` or
    /// `MSG[i].Field`): none when the tree is built, as a `TreeView`'s nodes start collapsed.
    expanded: std::collections::BTreeSet<String>,
}

/// Pixels a wheel notch is taken as, over the chart.
const WHEEL_NOTCH: f32 = 20.0;

/// The parameter list Show Params shows.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamsView {
    /// The rows, sorted as the grid sorts them.
    rows: Vec<LogParam>,
    /// Whether the Default column shows.
    defaults: bool,
    /// The first row on screen.
    first: usize,
}

/// Rows of the parameter list on screen at once.
const PARAM_ROWS: usize = 16;

/// Items of the preselected graphs' list on screen at once.
const PRESELECT_ROWS: usize = 20;

/// The caption of the dialog standing in for `SaveFileDialog`: its own default.
const SAVE_AS: &str = "Save As";

/// What a prompt asks for, and so what OK does with the answer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PromptKind {
    /// `ProcessCmdKey`'s Ctrl+G: `InputBox.Show("Line no", "Enter Line Number", ref lineno)`.
    GoToLine,
    /// `treeView1_DoubleClick`: a field's scaler and offset.
    Modifier(String),
    /// Export Visible's `SaveFileDialog`.
    ExportVisible,
    /// Export Files' `FolderBrowserDialog`.
    ExportFiles,
}

/// A dialog, drawn in the window: a title, what it asks, the answer being typed.
#[derive(Debug)]
pub struct Prompt {
    /// What it is for.
    pub kind: PromptKind,
    /// The caption.
    pub title: String,
    /// What it says.
    pub text: String,
    /// The answer.
    pub field: crate::textfield::TextField,
}

impl std::fmt::Debug for LogBrowse {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LogBrowse")
            .field("path", &self.path)
            .field("fields", &self.fields.len())
            .field("plotted", &self.plotted)
            .field("status", &self.status)
            .field("grid", &self.grid)
            .field("map", &self.map_contents)
            .field("strip", &self.strip)
            .field("cursor", &self.cursor)
            .field("marker", &self.marker)
            .finish_non_exhaustive()
    }
}

impl Default for LogBrowse {
    fn default() -> Self {
        Self::new()
    }
}

impl LogBrowse {
    /// Nothing open.
    #[must_use]
    pub fn new() -> Self {
        Self {
            log: None,
            path: None,
            fields: Vec::new(),
            units: UnitTable::default(),
            plotted: Vec::new(),
            status: None,
            refused: None,
            grid: None,
            map: Rc::new(RefCell::new(MapViewport::new(0, 0))),
            map_contents: MapContents::default(),
            tiles: None,
            strip: Strip::default(),
            grid_filled: false,
            overlays: Overlays::default(),
            positions: Positions::default(),
            origin: None,
            labelled: false,
            cursor: None,
            marker: None,
            chart_bounds: Rc::new(Cell::new(None)),
            routes: Routes::default(),
            drawn: Rc::new(Routes::default()),
            fit_routes: Rc::new(Cell::new(false)),
            params: Vec::new(),
            params_view: None,
            graphs: Vec::new(),
            preselect: None,
            preselect_list: None,
            preselect_outcome: None,
            zoom: Zoom::default(),
            zoom_drag: None,
            point_values: false,
            pointer: None,
            chart_menu: None,
            grid_menu: false,
            prompt: None,
            modifiers: BTreeMap::new(),
            exported: None,
            cursor_time: None,
            info: {
                let mut info = crate::textfield::TextField::new("");
                info.set_multiline(true);
                info
            },
            bits: BTreeMap::new(),
            expanded: std::collections::BTreeSet::new(),
        }
    }

    /// Opens a log: what it can plot, where it went, and the index its grid reads rows through.
    ///
    /// `new DFLogBuffer(filename)`: the file is read and walked once for every record's place and
    /// type, and everything after that - the field list, the units, the labels, the routes, the
    /// parameters, the grid's rows, each curve - reads only the records of the types it needs,
    /// through that index (`mp_log::logfile`). D14's budget for this is two seconds for a 1 GB
    /// log to its first plot; `crates/mp-log/benches/parse_1gb.rs` measures it.
    /// `// C#: Log/LogBrowse.cs:359-401; ExtLibs/Utilities/DFLogBuffer.cs:43-200`
    pub fn open(&mut self, path: &std::path::Path) {
        let log = match mp_log::logfile::LogFile::open(path) {
            Ok(log) => Rc::new(log),
            Err(err) => {
                self.status = Some(format!("could not read {}: {err}", path.display()));
                return;
            }
        };
        self.fields = log.plottable();
        self.units = log.units();
        // `ResetTreeView`: the tree built again, each bitmask field with its bits, from the
        // metadata as it is now - a log opened before `LogMetaData` was read has no bits.
        // `// C#: Log/LogBrowse.cs:440, 699-764`
        self.add_bit_nodes(metadata::shared());
        self.path = Some(path.to_path_buf());
        self.refused = None;
        // `LogBrowse_Load` empties the map's marker; `LoadLog2` clears the chart through
        // `chk_time_CheckedChanged` and labels it with `zg1_ZoomEvent` once the log is read.
        // `// C#: Log/LogBrowse.cs:237-240, 403-408, 479`
        self.marker = None;
        self.clear();
        self.overlays = log.overlays(flight_mode_name);
        self.positions = log.positions();
        self.origin = log.time_origin();
        self.grid_filled = self.strip.get(Check::DataTable);
        let parms: Vec<mp_log::dataflash::LogMessage> = log
            .messages(&["PARM"])
            .map(|(_, message)| message)
            .collect();
        self.params = mp_log::logparams::from_messages(&parms);
        self.params_view = None;
        self.prompt = None;
        self.chart_menu = None;
        self.grid_menu = false;
        self.exported = None;
        // `LogBrowse_Load` empties the map's overlay; `DrawMap` fills it when the chart is next
        // labelled with the map shown.
        self.routes = log.routes();
        self.drawn = Rc::new(Routes::default());
        self.map_contents = map_contents(&self.routes);
        self.map = Rc::new(RefCell::new(MapViewport::new(0, 0)));
        if let Some(store) = &self.tiles {
            self.map.borrow_mut().set_tiles(Arc::clone(store));
        }
        // `readmavgraphsxml` runs once, and `LoadLog2` hands the sorted list to `CMB_preselect`,
        // whose first item - "a/None" - is then selected and does nothing.
        // `// C#: Log/LogBrowse.cs:469-477; ExtLibs/Utilities/mavgraph.cs:209-215`
        if self.graphs.is_empty() {
            self.graphs = mp_log::mavgraph::graphs();
        }
        self.preselect = (!self.graphs.is_empty()).then_some(0);
        self.preselect_list = None;
        self.preselect_outcome = None;
        self.zoom_event();
        // The grid decodes its rows a screenful at a time from the log the index holds, as
        // `CellValueNeeded` asks `DFLogBuffer` for one row at a time.
        self.grid = Some(Grid::over(Rc::clone(&log), leap_seconds_now()));
        self.log = Some(log);
        self.status = Some(if self.fields.is_empty() {
            format!(
                "{} has nothing plottable - is it a dataflash log?",
                path.display()
            )
        } else {
            format!(
                "{} fields in {}",
                self.fields.len(),
                path.file_name().map_or_else(
                    || path.display().to_string(),
                    |n| n.to_string_lossy().into_owned()
                )
            )
        });
    }

    /// Adds a field to the plot on the left axis, or removes it if it is there.
    ///
    /// A left click in Mission Planner's field tree; see [`Self::graph`].
    pub fn toggle(&mut self, field: &PlottableField) {
        self.graph(field, Axis::Left);
    }

    /// Adds a field to the plot on one axis, or removes it if it is plotted on either.
    ///
    /// The tree in `LogBrowse` graphs a field when its box is checked - on the left axis for a
    /// left click, on the right for a right click - and unchecking removes the curve whichever
    /// axis it was on. So a click on a plotted field removes it regardless of button, and a click
    /// on an unplotted one adds it on the axis the button says.
    ///
    /// Adding a curve ends in `zg1_ZoomEvent`, which redraws the labels and so takes the cursor
    /// off the chart; removing one does not.
    /// `// C#: Log/LogBrowse.cs:1692-1694, 3079-3128, 3843-3855`
    pub fn graph(&mut self, field: &PlottableField, axis: Axis) {
        if let Some(position) = self
            .plotted
            .iter()
            .position(|shown| shown.field.as_ref() == Some(field) && shown.bit.is_none())
        {
            self.plotted.remove(position);
            return;
        }
        let Some(log) = self.log.as_ref() else {
            return;
        };
        // `GraphItem_GetList`: the type's records, through the index.
        // `// C#: Log/LogBrowse.cs:1488-1595`
        let points = log.extract_instance(&field.message, field.instance, &field.field);
        let unit = self.units.get(&field.message, &field.field);
        let node = modifier::node_name(&field.message, field.instance, &field.field);
        let plotted = Plotted::modified(
            field.clone(),
            &points,
            &unit,
            axis,
            self.x_axis(),
            self.modifiers.get(&node),
        );
        self.status = Some(format!("{}: {} samples", plotted.label, points.len()));
        self.plotted.push(plotted);
        // `GraphItem_AddCurve` zooms out all the way before it labels the chart.
        // `// C#: Log/LogBrowse.cs:1685-1690`
        self.zoom.reset();
        self.zoom_event();
    }

    /// Which axis a field is plotted on, if it is.
    #[must_use]
    pub fn axis_of(&self, field: &PlottableField) -> Option<Axis> {
        self.plotted
            .iter()
            .find(|shown| shown.field.as_ref() == Some(field) && shown.bit.is_none())
            .map(|shown| shown.axis)
    }

    /// `add_field_node`'s bit nodes for every field of the log, from `LogMetaData`: a field
    /// whose metadata is a bitmask gets a node for each of its bits, and with no metadata
    /// nothing does. Every field node starts collapsed.
    /// `// C#: Log/LogBrowse.cs:682-696`
    pub fn add_bit_nodes(&mut self, meta: Option<&metadata::MetaData>) {
        self.bits.clear();
        self.expanded.clear();
        let Some(meta) = meta else {
            return;
        };
        for field in &self.fields {
            let key = (field.message.clone(), field.field.clone());
            if self.bits.contains_key(&key) {
                continue;
            }
            let nodes = bit_nodes(meta, &field.message, &field.field);
            if !nodes.is_empty() {
                self.bits.insert(key, nodes);
            }
        }
    }

    /// A field's bit nodes: empty unless its metadata is a bitmask.
    #[must_use]
    pub fn bits_of(&self, field: &PlottableField) -> &[BitNode] {
        self.bits
            .get(&(field.message.clone(), field.field.clone()))
            .map_or(&[], Vec::as_slice)
    }

    /// Whether a field's node is expanded to show its bits.
    #[must_use]
    pub fn is_expanded(&self, field: &PlottableField) -> bool {
        self.expanded.contains(&field.to_string())
    }

    /// The node's plus or minus: a field with bits expanded or collapsed, as a `TreeView` node
    /// with children is. A field with none has no plus to click.
    pub fn toggle_expanded(&mut self, field: &PlottableField) {
        if self.bits_of(field).is_empty() {
            return;
        }
        let name = field.to_string();
        if !self.expanded.remove(&name) {
            self.expanded.insert(name);
        }
    }

    /// Which axis a bit of a field is plotted on, if it is.
    #[must_use]
    pub fn axis_of_bit(&self, field: &PlottableField, bit: &str) -> Option<Axis> {
        self.plotted
            .iter()
            .find(|shown| shown.field.as_ref() == Some(field) && shown.bit.as_deref() == Some(bit))
            .map(|shown| shown.axis)
    }

    /// A bit's node ticked or unticked: `treeView1_AfterCheck` on a node tagged `"bitmask"`,
    /// which is `GraphItem(type, field, left, ..., instance, bit)`. Ticked, the field's values
    /// are read and each goes through the bit's mask - `(value & mask) >> lowest set bit` - and
    /// is drawn on the axis the click says, labelled `MSG.Field.BIT` and the bit's description;
    /// unticked, that curve comes off whichever axis it is on.
    ///
    /// `GraphItem` first gives up on any curve whose label starts with the field's node name and
    /// a space. A curve of the field itself always does - its label ends in the space before its
    /// tooltip, which ours drops, so the name alone stands for it - and so does a bit of a field
    /// with a unit, whose label is `MSG.Field (unit).BIT`: then the bit is not graphed, and the
    /// status line says so where the C# says nothing.
    /// `// C#: Log/LogBrowse.cs:1129-1152, 1210-1248, 1488-1595, 3079-3128`
    pub fn graph_bit(&mut self, field: &PlottableField, bit: &str, axis: Axis) {
        if let Some(position) = self.plotted.iter().position(|shown| {
            shown.field.as_ref() == Some(field) && shown.bit.as_deref() == Some(bit)
        }) {
            self.plotted.remove(position);
            return;
        }
        let Some(node) = self
            .bits_of(field)
            .iter()
            .find(|node| node.text == bit)
            .cloned()
        else {
            return;
        };
        let Some(log) = self.log.as_ref() else {
            return;
        };
        let name = modifier::node_name(&field.message, field.instance, &field.field);
        let spaced = format!("{name} ");
        if self
            .plotted
            .iter()
            .any(|shown| shown.label == name || shown.label.starts_with(&spaced))
        {
            self.status = Some(format!("{name} is already on the graph"));
            return;
        }
        // `GraphItem_GetList`: the type's records, through the index, as for the field.
        // `// C#: Log/LogBrowse.cs:1488-1595`
        let points = log.extract_instance(&field.message, field.instance, &field.field);
        let unit = self.units.get(&field.message, &field.field);
        let plotted = Plotted::of_bit(field.clone(), &points, &unit, axis, self.x_axis(), &node);
        self.status = Some(format!("{}: {} samples", plotted.label, points.len()));
        self.plotted.push(plotted);
        // `GraphItem_AddCurve` zooms out all the way before it labels the chart.
        // `// C#: Log/LogBrowse.cs:1685-1690`
        self.zoom.reset();
        self.zoom_event();
    }

    /// The fields the open log declares.
    #[must_use]
    pub fn fields(&self) -> &[PlottableField] {
        &self.fields
    }

    /// The series being plotted.
    #[must_use]
    pub fn plotted(&self) -> &[Plotted] {
        &self.plotted
    }

    /// How many series are on the right axis.
    #[must_use]
    pub fn right_count(&self) -> usize {
        self.plotted
            .iter()
            .filter(|shown| shown.axis == Axis::Right)
            .count()
    }

    /// The units the left side is split into - one axis each.
    #[must_use]
    pub fn left_units(&self) -> std::collections::BTreeSet<&str> {
        self.plotted
            .iter()
            .filter(|shown| shown.axis == Axis::Left)
            .map(|shown| shown.unit.as_str())
            .collect()
    }

    /// The last thing that happened.
    #[must_use]
    pub fn status(&self) -> Option<&str> {
        self.status.as_deref()
    }

    /// Whether a log is open.
    #[must_use]
    pub const fn is_open(&self) -> bool {
        self.path.is_some()
    }

    /// Clear Graph: the curves, and everything drawn over them - the labels and the cursor.
    ///
    /// The labels come back with the next `zg1_ZoomEvent`, which adding a curve or ticking a box
    /// raises. The map's marker is not on the chart and stays.
    /// `// C#: Log/LogBrowse.cs:2801-2808`
    pub fn clear(&mut self) {
        self.plotted.clear();
        self.labelled = false;
        self.cursor = None;
        self.zoom.reset();
        self.zoom_drag = None;
    }

    /// `zg1_ZoomEvent`: the chart's drawn objects are emptied and the labels the ticked boxes
    /// ask for drawn again - which empties the cursor off it too - and, with the map shown,
    /// `DrawMap` puts on it the stretch of the routes the chart shows.
    /// `// C#: Log/LogBrowse.cs:2906-2978`
    fn zoom_event(&mut self) {
        self.labelled = true;
        self.cursor = None;
        if self.strip.get(Check::Map) {
            self.draw_map();
        }
    }

    /// `DrawMap`: the routes of the whole log with nothing plotted, else of the lines the x axis
    /// spans - on a line axis its ends as lines, on a time axis the first position record at or
    /// after each end (`GetLineNoFromTime`) - and then `ZoomAndCenterRoutes`.
    /// `// C#: Log/LogBrowse.cs:2938-2964, 2191-2520; ExtLibs/Utilities/DFLog.cs:736-753`
    fn draw_map(&mut self) {
        let drawn = match self.x_range() {
            None => self.routes.clone(),
            Some((min, max)) => {
                let (start, end) = match self.x_axis() {
                    // `(long)Scale.Min`: towards zero.
                    #[allow(clippy::cast_possible_truncation)]
                    XAxis::Line => (min as i64, max as i64),
                    XAxis::Time => (self.line_at_seconds(min), self.line_at_seconds(max)),
                };
                match (usize::try_from(start.max(0)), usize::try_from(end)) {
                    (Ok(start), Ok(end)) => self.routes.between(start, end),
                    _ => Routes::default(),
                }
            }
        };
        self.drawn = Rc::new(drawn);
        self.fit_routes.set(true);
    }

    /// `GetLineNoFromTime` for a place on the time axis, as a `long`.
    fn line_at_seconds(&self, seconds: f64) -> i64 {
        let Some(origin) = self.origin else {
            return 0;
        };
        match self
            .positions
            .line_at_time(seconds.mul_add(1_000_000.0, origin))
        {
            LineAtTime::Line(line) => i64::try_from(line).unwrap_or(i64::MAX),
            LineAtTime::AfterAll => i64::MAX,
            LineAtTime::NoPositions => 0,
        }
    }

    /// What `DrawMap` last put on the map.
    #[must_use]
    pub fn drawn(&self) -> &Routes {
        &self.drawn
    }

    /// `LoadLog2`'s reading of the six remembered boxes, in its order, each through its
    /// `CheckedChanged` when it changes, and then the chart labelled as `LoadLog2` ends.
    /// `// C#: Log/LogBrowse.cs:444-449, 479`
    pub fn apply_remembered(&mut self, get: impl Fn(&str) -> Option<String>) {
        for check in [
            Check::DataTable,
            Check::Time,
            Check::Map,
            Check::Errors,
            Check::Mode,
            Check::Msg,
        ] {
            let Some(key) = check.setting() else {
                continue;
            };
            let wanted = setting_bool(get(key).as_deref(), check.default());
            if wanted != self.strip.get(check) {
                self.toggle_check(check);
            }
        }
        if self.is_open() {
            self.zoom_event();
        }
    }

    /// What automatic scaling shows: every curve's whole extent, each axis fitted to it.
    fn automatic(&self) -> Option<Scales> {
        let (from, to) = self
            .plotted
            .iter()
            .filter_map(|shown| shown.series.extent())
            .reduce(|(low, high), (from, to)| (low.min(from), high.max(to)))?;
        let axes = Axes::over(&self.plotted, from, to);
        Some(Scales {
            x: (from, to),
            left: axes.left,
            right: axes.right,
        })
    }

    /// The ranges the chart shows: the zoom's, where the user has chosen, and automatic for any
    /// axis the zoom does not cover.
    #[must_use]
    pub fn scales(&self) -> Option<Scales> {
        let automatic = self.automatic()?;
        let mut scales = self.zoom.scales(automatic.clone());
        for (unit, range) in automatic.left {
            scales.left.entry(unit).or_insert(range);
        }
        if scales.right.is_none() {
            scales.right = automatic.right;
        }
        Some(scales)
    }

    /// The chart's zoom.
    #[must_use]
    pub const fn zoom(&self) -> &Zoom {
        &self.zoom
    }

    /// Where a window position is on the plotting area, as fractions across and down, as it was
    /// last laid out.
    #[must_use]
    pub fn chart_point(&self, window_x: f32, window_y: f32) -> Option<(f64, f64)> {
        let bounds = self.chart_bounds.get()?;
        let (width, height) = (f32::from(bounds.size.width), f32::from(bounds.size.height));
        (width > 0.0 && height > 0.0).then(|| {
            (
                f64::from((window_x - f32::from(bounds.origin.x)) / width),
                f64::from((window_y - f32::from(bounds.origin.y)) / height),
            )
        })
    }

    /// The plotting area's size in pixels, as last laid out.
    fn chart_size(&self) -> (f32, f32) {
        self.chart_bounds.get().map_or((0.0, 0.0), |bounds| {
            (f32::from(bounds.size.width), f32::from(bounds.size.height))
        })
    }

    /// A press on the chart that is not a double click: Ctrl with the left button, or the
    /// middle button, starts a pan; the left button alone starts a rectangle. Either closes an
    /// open context menu.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.Events.cs:393-480`
    pub fn chart_press(&mut self, at: (f64, f64), pan: bool) {
        self.chart_menu = None;
        if self.plotted.is_empty() {
            return;
        }
        if pan {
            self.zoom.begin_pan(at);
        } else {
            self.zoom_drag = Some((at, at));
        }
    }

    /// The pointer moved over the chart: the point values follow it, a rectangle stretches, a
    /// pan moves every axis.
    pub fn chart_move(&mut self, at: (f64, f64)) {
        self.pointer = Some(at);
        if self.zoom.is_panning() {
            self.chart_pan_to(at);
        }
        if let Some((_, to)) = self.zoom_drag.as_mut() {
            *to = at;
        }
    }

    /// The pointer left the chart.
    pub fn chart_leave(&mut self) {
        self.pointer = None;
    }

    /// The button came up: a rectangle zooms, a pan ends, and either raises `ZoomEvent`.
    pub fn chart_release(&mut self) {
        if self.zoom.is_panning() {
            if self.zoom.end_pan() {
                self.zoom_event();
            }
            return;
        }
        if let Some((from, to)) = self.zoom_drag.take() {
            let size = self.chart_size();
            self.chart_drag_zoom(from, to, size);
        }
    }

    /// `HandleZoomFinish`: every axis zoomed to a dragged rectangle, if it is one.
    pub fn chart_drag_zoom(&mut self, from: (f64, f64), to: (f64, f64), size: (f32, f32)) {
        let Some(automatic) = self.scales_for_zoom() else {
            return;
        };
        if self.zoom.drag_zoom(automatic, from, to, size) {
            self.zoom_event();
        }
    }

    /// A wheel event over the chart, by its pixel delta: one zoom step per event by the sign, as
    /// `ZedGraphControl_MouseWheel` reads only `e.Delta < 0`. Returns whether it zoomed.
    pub fn chart_wheel_pixels(&mut self, pixels: f32) -> bool {
        // `ZedGraphControl_MouseWheel` zooms once per event by the sign of `e.Delta`, whatever
        // its size - so one notch is one step, however many lines the desktop puts in a notch.
        // `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.Events.cs:835-846`
        if pixels == 0.0 {
            return false;
        }
        // Towards the user is a negative `Delta` in Windows and a negative y here.
        self.chart_wheel(pixels < 0.0);
        true
    }

    /// `ZedGraphControl_MouseWheel`: a notch, towards the user or away.
    pub fn chart_wheel(&mut self, towards_user: bool) {
        let Some(automatic) = self.scales_for_zoom() else {
            return;
        };
        self.zoom.wheel(automatic, towards_user);
        self.zoom_event();
    }

    /// `HandlePanDrag`.
    pub fn chart_pan_to(&mut self, at: (f64, f64)) {
        if let Some(automatic) = self.scales_for_zoom() {
            self.zoom.pan_to(automatic, at);
        }
    }

    /// The ranges a zoom starts from when it is the first: what is on screen.
    fn scales_for_zoom(&self) -> Option<Scales> {
        self.scales()
    }

    /// The rectangle being dragged, if one is.
    #[must_use]
    pub const fn zoom_drag(&self) -> Option<((f64, f64), (f64, f64))> {
        self.zoom_drag
    }

    /// Opens ZedGraph's context menu: a right click on the chart.
    pub fn open_chart_menu(&mut self, at: (f64, f64)) {
        self.chart_menu = Some(at);
        self.zoom_drag = None;
    }

    /// The chart's context menu, if it is open.
    #[must_use]
    pub const fn chart_menu(&self) -> Option<(f64, f64)> {
        self.chart_menu
    }

    /// One of the chart menu's items, by its ZedGraph name: `show_val`, `unzoom`, `undo_all`,
    /// `set_default`. Un-Zoom and Undo All do nothing with nothing to undo, where the C# draws
    /// them disabled. Each but Show Point Values ends in `ZoomEvent`.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.ContextMenu.cs:130-200, 615-800`
    pub fn chart_menu_item(&mut self, item: &str) {
        self.chart_menu = None;
        let changed = match item {
            "show_val" => {
                self.point_values = !self.point_values;
                false
            }
            "unzoom" => self.zoom.undo(),
            "undo_all" => self.zoom.undo_all(),
            "set_default" => {
                self.zoom.set_default();
                true
            }
            _ => false,
        };
        if changed {
            self.zoom_event();
        }
    }

    /// `treeView1_TreeNodeMouseHover`: the pointer resting on a field's node. Its path is
    /// `MSG\field`, or `MSG\instance\field`; a message and field `LogMetaData` knows puts
    /// the field's description in `txt_info`, and anything else leaves the box as it was. The
    /// list here has a node for each field and none for a message, so the C#'s other branch -
    /// a message's own description, for the pointer on its node - has no node to rest on.
    /// `// C#: Log/LogBrowse.cs:3778-3803`
    pub fn hover_field(&mut self, field: &PlottableField, meta: Option<&metadata::MetaData>) {
        if let Some(known) = meta.and_then(|meta| meta.field(&field.message, &field.field)) {
            self.info.set(known.description.clone());
        }
    }

    /// `treeView1_TreeNodeMouseHover` on a bit's node: its path is `MSG\field\bit`, or with the
    /// instance, and the C# looks its last part up as a field of the message - so a bit's node
    /// puts nothing in `txt_info` unless the message has a field of the bit's name. The bit's
    /// own description is its node's tooltip.
    /// `// C#: Log/LogBrowse.cs:3778-3803`
    pub fn hover_bit(
        &mut self,
        field: &PlottableField,
        bit: &str,
        meta: Option<&metadata::MetaData>,
    ) {
        if let Some(known) = meta.and_then(|meta| meta.field(&field.message, bit)) {
            self.info.set(known.description.clone());
        }
    }

    /// `txt_info.Text`.
    #[cfg(test)]
    #[must_use]
    pub fn info(&self) -> &str {
        self.info.value()
    }

    /// `txt_info` itself, for drawing.
    #[must_use]
    pub const fn info_field(&self) -> &crate::textfield::TextField {
        &self.info
    }

    /// A key in `txt_info`: an ordinary multi-line `TextBox`, so a key moves, selects, copies,
    /// pastes or types; Escape does nothing there. Returns whether the box changed.
    /// `// C#: Log/LogBrowse.designer.cs:391-396; Log/LogBrowse.resx (txt_info.Multiline)`
    pub fn info_key(&mut self, event: &gpui::KeyDownEvent) -> bool {
        self.info.key(event) == crate::textfield::KeyOutcome::Changed
    }

    /// Whether Show Point Values is on.
    #[must_use]
    pub const fn point_values(&self) -> bool {
        self.point_values
    }

    /// The tooltip for the point nearest the pointer, when Show Point Values is on:
    /// `HandlePointValues`.
    #[must_use]
    pub fn point_tooltip(&self, size: (f32, f32)) -> Option<String> {
        if !self.point_values {
            return None;
        }
        let pointer = self.pointer?;
        let scales = self.scales()?;
        let axes = Axes {
            left: scales.left.clone(),
            right: scales.right,
        };
        let curves: Vec<view::Curve<'_>> = self
            .plotted
            .iter()
            .filter_map(|shown| {
                axes.range_for(shown).map(|range| view::Curve {
                    series: &shown.series,
                    order: &shown.order,
                    range,
                })
            })
            .collect();
        let (_, sample) = view::nearest_point(&curves, scales.x, pointer, size)?;
        let x = match self.x_axis() {
            XAxis::Time => {
                let origin = self.origin?;
                let boot_ms = sample.at.mul_add(1_000_000.0, origin) / 1000.0;
                self.grid.as_ref()?.time_of_day(boot_ms)
            }
            XAxis::Line => mp_log::netfmt::double(sample.at),
        };
        Some(view::point_text(&x, sample.value))
    }

    /// The check boxes under the chart.
    #[must_use]
    pub const fn strip(&self) -> Strip {
        self.strip
    }

    /// What the x axis counts.
    #[must_use]
    pub fn x_axis(&self) -> XAxis {
        if self.strip.get(Check::Time) {
            XAxis::Time
        } else {
            XAxis::Line
        }
    }

    /// Ticks or unticks a box: a click on it, and its `CheckedChanged`.
    ///
    /// - Map shows or hides the map, and ticking it redraws the chart's labels.
    /// - Time clears the chart, as the axis it was drawn against is gone.
    /// - Data Table shows or hides the grid; the first showing fills it.
    /// - Mode, Errors, MSG and Events redraw the chart's labels with the box's say.
    ///
    /// `// C#: Log/LogBrowse.cs:2980-2996, 3534-3570, 3611-3624, 3642-3647, 3773-3776`
    pub fn toggle_check(&mut self, check: Check) {
        let on = !self.strip.get(check);
        self.strip.set(check, on);
        match check {
            Check::Map => {
                if on {
                    self.zoom_event();
                }
            }
            Check::Time => self.clear(),
            Check::DataTable => {
                if on {
                    self.grid_filled = true;
                }
            }
            Check::Mode | Check::Errors | Check::Msg | Check::Events => self.zoom_event(),
        }
    }

    /// The x range the chart spans: the lowest and highest x of anything plotted, as the C#'s
    /// `ZoomOutAll` fits the axis to its curves, or what the user zoomed or panned to.
    #[must_use]
    pub fn x_range(&self) -> Option<(f64, f64)> {
        self.scales().map(|scales| scales.x)
    }

    /// Where a record sits on the x axis, if it can be placed there.
    #[allow(clippy::cast_precision_loss)] // a line number is far below 2^53
    fn x_of(&self, line: usize, time_us: Option<f64>) -> Option<f64> {
        match self.x_axis() {
            XAxis::Line => Some(line as f64),
            XAxis::Time => Some(mp_log::plot::seconds_since(self.origin?, time_us?)),
        }
    }

    /// What the strip, the labels and the cursor come to, as facts a test can assert on.
    ///
    /// - `log.check.<box>`: each box, `true` when ticked - `map`, `time`, `datagrid`, `mode`,
    ///   `errors`, `msg`, `events`;
    /// - `log.info`: `txt_info`'s text, the description of the field last hovered;
    /// - `log.info.selection`: what is selected in it, `start,end` in characters, or `none`;
    /// - `log.axis`: `time` or `line`;
    /// - `log.overlays.<kind>`: how many labels of each kind the chart carries now;
    /// - `log.cursor.line`, `log.cursor.x`: the line the cursor is on and where on the axis it is
    ///   drawn, or `none`;
    /// - `log.cursor.marker`: the map marker's `latitude,longitude`, or `none`;
    /// - `log.cursor.centred`: whether the map is centred on the marker;
    /// - `log.cursor.grid`: whether the grid's current cell is the cursor line's time cell;
    /// - `log.grid.current`: the grid's current cell as `row,column`, or `none`.
    #[must_use]
    pub fn facts(&self) -> Vec<(String, String)> {
        let mut facts = Vec::new();
        for check in Check::STRIP {
            facts.push((
                format!("log.check.{}", check.id().trim_start_matches("log-chk-")),
                self.strip.get(check).to_string(),
            ));
        }
        facts.push(("log.info".to_owned(), self.info.value().to_owned()));
        facts.push(("log.info.selection".to_owned(), self.info.selection_fact()));
        facts.push((
            "log.axis".to_owned(),
            match self.x_axis() {
                XAxis::Time => "time",
                XAxis::Line => "line",
            }
            .to_owned(),
        ));
        for (kind, count) in self.overlay_counts() {
            facts.push((format!("log.overlays.{kind}"), count.to_string()));
        }
        let none = || "none".to_owned();
        facts.push((
            "log.cursor.line".to_owned(),
            self.cursor().map_or_else(none, |line| line.to_string()),
        ));
        facts.push((
            "log.cursor.x".to_owned(),
            self.cursor_x().map_or_else(none, |x| format!("{x:.3}")),
        ));
        facts.push((
            "log.cursor.marker".to_owned(),
            self.marker.map_or_else(none, |at| {
                format!("{:.7},{:.7}", at.latitude(), at.longitude())
            }),
        ));
        facts.push((
            "log.cursor.centred".to_owned(),
            self.map_centred_on_marker().to_string(),
        ));
        facts.push((
            "log.cursor.grid".to_owned(),
            self.cursor
                .is_some_and(|line| self.grid_current() == Some((line, 1)))
                .to_string(),
        ));
        facts.push((
            "log.grid.current".to_owned(),
            self.grid_current()
                .map_or_else(none, |(row, column)| format!("{row},{column}")),
        ));
        facts.extend(self.more_facts());
        facts
    }

    /// The facts of what this port adds to the strip, the chart, the map and the grid:
    ///
    /// - `log.check.params`: Show Params, which never stays ticked;
    /// - `log.params.shown`, `.count`, `.first`, `.defaults`: the parameter list, its first row
    ///   as `NAME=value`, and whether it has a Default column;
    /// - `log.preselect.items`, `.selected`, `.open`, `.graphed`, `.skipped`;
    /// - `log.map.drawn.<route>` for `gps`, `gps2`, `gpsb`, `pos` and `cmd`, and
    ///   `log.map.drawn.markers` and `.photos`: what `DrawMap` last drew;
    /// - `log.zoom.depth`, `log.zoom.x` (`auto` or `min,max`), `log.zoom.undo` (`Un-Zoom`,
    ///   `Un-Pan` or `none`), `log.zoom.dragging`;
    /// - `log.chart.pointvalues`, `log.chart.tooltip`, `log.chart.menu`, `log.grid.menu`;
    /// - `log.prompt`: the open prompt's title, or `none`; `log.exported`; `log.modifiers`;
    /// - `log.field.<MSG.Field>.bits`: a bitmask field's bit nodes, their texts in order and
    ///   comma separated, and `.expanded`, whether its node shows them;
    /// - `log.plotted.labels`: every curve's label, in the order drawn, ` | ` between them.
    #[must_use]
    pub fn more_facts(&self) -> Vec<(String, String)> {
        let none = || "none".to_owned();
        let mut facts = vec![("log.check.params".to_owned(), "false".to_owned())];
        for field in &self.fields {
            let bits = self.bits_of(field);
            if bits.is_empty() {
                continue;
            }
            let texts: Vec<&str> = bits.iter().map(|bit| bit.text.as_str()).collect();
            facts.push((format!("log.field.{field}.bits"), texts.join(",")));
            facts.push((
                format!("log.field.{field}.expanded"),
                self.is_expanded(field).to_string(),
            ));
        }
        let labels: Vec<&str> = self
            .plotted
            .iter()
            .map(|shown| shown.label.as_str())
            .collect();
        facts.push(("log.plotted.labels".to_owned(), labels.join(" | ")));
        let view = self.params_view.as_ref();
        facts.push(("log.params.shown".to_owned(), view.is_some().to_string()));
        facts.push((
            "log.params.count".to_owned(),
            view.map_or(0, |view| view.rows.len()).to_string(),
        ));
        facts.push((
            "log.params.first".to_owned(),
            view.and_then(|view| view.rows.first())
                .map_or_else(none, |param| {
                    format!("{}={}", param.name, param.value_text())
                }),
        ));
        facts.push((
            "log.params.top".to_owned(),
            view.map_or(0, |view| view.first).to_string(),
        ));
        facts.push((
            "log.params.defaults".to_owned(),
            view.is_some_and(|view| view.defaults).to_string(),
        ));
        facts.push((
            "log.preselect.items".to_owned(),
            self.graphs.len().to_string(),
        ));
        facts.push((
            "log.preselect.selected".to_owned(),
            self.preselected().map_or_else(none, str::to_owned),
        ));
        facts.push((
            "log.preselect.open".to_owned(),
            self.preselect_list.is_some().to_string(),
        ));
        facts.push((
            "log.preselect.graphed".to_owned(),
            self.preselect_outcome
                .as_ref()
                .map_or(0, |(graphed, _)| *graphed)
                .to_string(),
        ));
        facts.push((
            "log.preselect.skipped".to_owned(),
            self.preselect_outcome
                .as_ref()
                .map_or(0, |(_, skipped)| skipped.len())
                .to_string(),
        ));
        let drawn = self.drawn();
        for kind in RouteKind::ALL {
            facts.push((
                format!("log.map.drawn.{}", route_name(kind)),
                drawn.route(kind).len().to_string(),
            ));
        }
        facts.push((
            "log.map.drawn.markers".to_owned(),
            (drawn.commands.len() + drawn.repeats.len()).to_string(),
        ));
        facts.push((
            "log.map.drawn.photos".to_owned(),
            drawn.cameras.len().to_string(),
        ));
        facts.push(("log.zoom.depth".to_owned(), self.zoom.depth().to_string()));
        facts.push((
            "log.zoom.x".to_owned(),
            match (self.zoom.is_zoomed(), self.x_range()) {
                (true, Some((low, high))) => format!("{low:.3},{high:.3}"),
                _ => "auto".to_owned(),
            },
        ));
        facts.push((
            "log.zoom.undo".to_owned(),
            self.zoom
                .top()
                .map_or_else(none, |kind| kind.undo_text().to_owned()),
        ));
        facts.push((
            "log.zoom.dragging".to_owned(),
            self.zoom_drag.is_some().to_string(),
        ));
        facts.push((
            "log.chart.pointvalues".to_owned(),
            self.point_values.to_string(),
        ));
        facts.push((
            "log.chart.tooltip".to_owned(),
            self.point_tooltip(self.chart_size()).unwrap_or_else(none),
        ));
        facts.push((
            "log.chart.menu".to_owned(),
            self.chart_menu.is_some().to_string(),
        ));
        facts.push(("log.grid.menu".to_owned(), self.grid_menu.to_string()));
        facts.push((
            "log.prompt".to_owned(),
            self.prompt
                .as_ref()
                .map_or_else(none, |prompt| prompt.title.clone()),
        ));
        facts.push((
            "log.exported".to_owned(),
            self.exported.clone().unwrap_or_else(none),
        ));
        facts.push((
            "log.modifiers".to_owned(),
            self.modifiers().len().to_string(),
        ));
        // How much of `LogBrowse` this screen has, from its ledger, as the flight screen's
        // coverage is published.
        let (done, elsewhere, missing, _, _) = coverage::counts(coverage::LOGBROWSE);
        facts.push((
            "coverage.logbrowse.done".to_owned(),
            (done + elsewhere).to_string(),
        ));
        facts.push(("coverage.logbrowse.missing".to_owned(), missing.to_string()));
        facts.push((
            "coverage.logbrowse.total".to_owned(),
            coverage::LOGBROWSE.len().to_string(),
        ));
        let (done, elsewhere, missing, _, _) = coverage::counts(coverage::BEYOND);
        facts.push((
            "coverage.logbrowse.beyond.done".to_owned(),
            (done + elsewhere).to_string(),
        ));
        facts.push((
            "coverage.logbrowse.beyond.missing".to_owned(),
            missing.to_string(),
        ));
        facts
    }

    /// A double click on the chart, `fraction` of the way across it.
    ///
    /// `zg1_MouseDoubleClick`: the pointer's x on the axis is a line number, or on a time axis
    /// the line of the first `GPS`, `GPS2` or `POS` record at or after that time; and then
    /// `GoToSample` with the map moved and the grid moved, the chart left where it is. With
    /// nothing plotted the chart has no axis to read a place off, and nothing happens.
    /// `// C#: Log/LogBrowse.cs:3278-3297; ExtLibs/Utilities/DFLog.cs:736-753`
    pub fn double_click(&mut self, fraction: f64) {
        let Some((from, to)) = self.x_range() else {
            return;
        };
        let x = fraction.mul_add(to - from, from);
        let sample = match self.x_axis() {
            // `(int)x`: towards zero.
            #[allow(clippy::cast_possible_truncation)]
            XAxis::Line => x as i64,
            XAxis::Time => {
                let Some(origin) = self.origin else {
                    return;
                };
                match self.positions.line_at_time(x.mul_add(1_000_000.0, origin)) {
                    LineAtTime::Line(line) => i64::try_from(line).unwrap_or(i64::MAX),
                    // `(int)long.MaxValue` is -1.
                    LineAtTime::AfterAll => -1,
                    LineAtTime::NoPositions => 0,
                }
            }
        };
        self.go_to_sample(sample, true, false, true);
    }

    /// `dataGridView1_CellDoubleClick`: `GoToSample` on the row's record with the map and the
    /// chart moved and the grid left where it is.
    ///
    /// **One deliberate difference.** The C# hands `GoToSample` the row's index, which is the
    /// record's line only while the grid is unfiltered; filtered to one type, row 5 is the fifth
    /// record of that type and the C# goes to line 5. Here it goes to the row's record.
    /// `// C#: Log/LogBrowse.cs:3525-3532`
    pub fn grid_double_click(&mut self, row: usize) {
        let Some(grid) = self.filled_grid() else {
            return;
        };
        let (Some(line), time_us) = (grid.line_of_row(row), grid.time_of_row(row)) else {
            return;
        };
        self.cursor_time = time_us;
        self.go_to_sample(i64::try_from(line).unwrap_or(i64::MAX), true, true, false);
    }

    /// `GoToSample`: the map's marker, the chart's cursor and the grid's current row, all on one
    /// record.
    ///
    /// The marker goes where `GetGPSFromRow` says, and without a place there is no marker - the
    /// old one is gone either way. With `move_map` the map centres on it. The cursor is a dashed
    /// light grey line down the chart, behind the curves. With `move_grid` the grid scrolls the
    /// row to its middle and makes its time cell current - if the grid has ever been shown, since
    /// until then the C#'s has no rows and `Rows[SampleID]` throws.
    ///
    /// A negative sample - a click after every position record, on a time axis - reads line -1
    /// in `GetGPSFromRow`, which throws after the marker is cleared, so nothing else changes.
    ///
    /// **One deliberate difference.** The C# puts its cursor line at x = the line number on both
    /// axes, and on a time axis a line number is a date in 1900: the line is off the chart, which
    /// its own `//TODO - time fails` owns up to. Here, on a time axis, it is drawn at the time of
    /// the record it stands on, where the C# evidently means it to be. `move_graph` - the grid's
    /// double click - centres the x axis on the cursor keeping its span, which the C# does by
    /// setting the scale to the line number and so, on a time axis, likewise off the chart.
    /// `// C#: Log/LogBrowse.cs:3464-3523`
    pub fn go_to_sample(&mut self, sample: i64, move_map: bool, move_graph: bool, move_grid: bool) {
        self.marker = None;
        let Ok(line) = usize::try_from(sample) else {
            return;
        };
        if let Some((latitude, longitude)) = self.positions.from_row(line)
            && let Ok(place) = LatLon::new(latitude, longitude)
        {
            self.marker = Some(place);
            if move_map {
                self.map.borrow_mut().centre_on(place);
            }
        }
        self.cursor = Some(line);
        if !move_graph {
            self.cursor_time = None;
        }
        if move_graph && let (Some(x), Some(scales)) = (self.cursor_x(), self.scales()) {
            self.zoom.centre_x(scales, x);
        }
        if move_grid
            && self.grid_filled
            && let Some(grid) = self.grid.as_mut()
        {
            grid.go_to(line, 1);
        }
    }

    /// The line the cursor is on.
    #[must_use]
    pub const fn cursor(&self) -> Option<usize> {
        self.cursor
    }

    /// Where the cursor is drawn on the x axis, if it can be: its line, or the time of the record
    /// it was put on.
    #[must_use]
    pub fn cursor_x(&self) -> Option<f64> {
        let line = self.cursor?;
        let time = self.cursor_time.or_else(|| self.positions.time_of(line));
        self.x_of(line, time)
    }

    /// The map's marker.
    #[must_use]
    pub const fn marker(&self) -> Option<LatLon> {
        self.marker
    }

    /// Whether the map's view is centred on its marker.
    #[must_use]
    pub fn map_centred_on_marker(&self) -> bool {
        let (Some(marker), Some(camera)) = (self.marker, self.map.borrow().camera()) else {
            return false;
        };
        let at = marker.to_web_mercator();
        (camera.centre.x - at.x).abs() < 1e-12 && (camera.centre.y - at.y).abs() < 1e-12
    }

    /// Whether a box's labels are on the chart now.
    fn shows(&self, check: Check) -> bool {
        self.labelled && self.strip.get(check)
    }

    /// The mode changes `DrawModes` labels: every one, unless a mode number past the end of the
    /// C#'s colour table throws first - which it does building the band for the change after it,
    /// so the labels stop there.
    /// `// C#: Log/LogBrowse.cs:1953-1968`
    fn drawn_modes(&self) -> &[mp_log::overlay::ModeChange] {
        let modes = &self.overlays.modes;
        let stop = modes
            .windows(2)
            .position(|pair| {
                pair.first()
                    .is_some_and(|mode| pastel(mode.number).is_none())
            })
            .map_or(modes.len(), |before| before + 1);
        modes.get(..stop).unwrap_or(modes)
    }

    /// How many labels each box has on the chart now: `modes`, `errors`, `messages`, `events` and
    /// `minutes`. Minutes are drawn only on a line-number axis.
    #[must_use]
    pub fn overlay_counts(&self) -> [(&'static str, usize); 5] {
        let count = |check: Check, marks: &[Mark]| {
            if self.shows(check) { marks.len() } else { 0 }
        };
        [
            (
                "modes",
                if self.shows(Check::Mode) {
                    self.drawn_modes().len()
                } else {
                    0
                },
            ),
            ("errors", count(Check::Errors, &self.overlays.errors)),
            ("messages", count(Check::Msg, &self.overlays.messages)),
            ("events", count(Check::Events, &self.overlays.events)),
            (
                "minutes",
                if self.labelled && self.x_axis() == XAxis::Line {
                    self.overlays.minutes.len()
                } else {
                    0
                },
            ),
        ]
    }

    /// The grid, once a log is open.
    #[must_use]
    pub const fn grid(&self) -> Option<&Grid> {
        self.grid.as_ref()
    }

    /// The grid, once a log is open and the Data Table box has shown it.
    ///
    /// `chk_datagrid_CheckedChanged` is what gives the C#'s grid its rows, so until then
    /// `graphit_clickprocess` finds `RowCount` 0 and asks for a valid file, and `GoToSample`'s
    /// `Rows[SampleID]` throws.
    /// `// C#: Log/LogBrowse.cs:1070-1074, 3642-3700`
    fn filled_grid(&self) -> Option<&Grid> {
        self.grid.as_ref().filter(|_| self.grid_filled)
    }

    /// Makes a grid cell current: a click on it.
    pub fn select_cell(&mut self, row: usize, column: usize) {
        if let Some(grid) = self.grid.as_mut() {
            grid.select(row, column);
        }
    }

    /// Moves the grid by a wheel's movement.
    pub fn scroll_grid(&mut self, pixels: f32) {
        if let Some(grid) = self.grid.as_mut() {
            grid.scroll_pixels(pixels);
        }
    }

    /// Shows or hides the types the grid can be filtered to: a click on a column header.
    pub fn toggle_grid_chooser(&mut self) {
        if let Some(grid) = self.grid.as_mut() {
            grid.toggle_chooser();
        }
    }

    /// Filters the grid to one type, or clears the filter.
    pub fn filter_grid(&mut self, name: Option<&str>) {
        if let Some(grid) = self.grid.as_mut() {
            grid.set_filter(name);
        }
    }

    /// Graphs the grid's current cell: Graph Left and Graph Right.
    ///
    /// `graphit_clickprocess` and then `GraphItem`. The refusals are the C#'s, in its words, and
    /// go to the status line where the C# puts up a message box - one window, and a modal box in
    /// it would stop the operator reading the grid the message is about. Two differences, both
    /// forced by the chart this graphs onto:
    ///
    /// - a field the chart has no time series for - an `FMT` column, a text field, `TimeUS`
    ///   itself - is refused, where the C# would draw something against line numbers;
    /// - a field already on the chart stays there, as `GraphItem` aborts on it, rather than
    ///   coming off as a second click in the field list does.
    ///
    /// `// C#: Log/LogBrowse.cs:1062-1146, 2898-2901`
    pub fn graph_selected(&mut self, axis: Axis) {
        let resolved = self
            .filled_grid()
            .map_or(Err(grid::Refusal::NoLog), Grid::resolve);
        let target = match resolved {
            Ok(target) => target,
            Err(refusal) => {
                self.refuse(refusal.to_string());
                return;
            }
        };
        let Some(field) = self
            .fields
            .iter()
            .find(|field| {
                field.message == target.message
                    && field.instance == target.instance
                    && field.field == target.field
            })
            .cloned()
        else {
            self.refuse(format!("{target} cannot be plotted against time"));
            return;
        };
        self.refused = None;
        if self.axis_of(&field).is_some() {
            self.status = Some(format!("{target} is already on the graph"));
            return;
        }
        self.graph(&field, axis);
    }

    /// Says why something was refused, and remembers that it was.
    fn refuse(&mut self, why: String) {
        self.status = Some(why.clone());
        self.refused = Some(why);
    }

    /// The last refusal, if nothing has succeeded since.
    #[must_use]
    pub fn refused(&self) -> Option<&str> {
        self.refused.as_deref()
    }

    /// Rows the grid holds: none until the Data Table box has shown it.
    #[must_use]
    pub fn grid_rows(&self) -> usize {
        self.filled_grid().map_or(0, Grid::rows)
    }

    /// The grid's current cell, once it has been shown.
    #[must_use]
    pub fn grid_current(&self) -> Option<(usize, usize)> {
        self.filled_grid().and_then(Grid::current)
    }

    /// Records in the open log.
    #[must_use]
    pub fn grid_records(&self) -> usize {
        self.grid.as_ref().map_or(0, Grid::records)
    }

    /// The field the grid's current cell holds, or `none`.
    #[must_use]
    pub fn selected_field(&self) -> String {
        self.filled_grid()
            .and_then(|grid| grid.resolve().ok())
            .map_or_else(|| "none".to_owned(), |field| field.to_string())
    }

    /// Show Params: `chk_params_CheckedChanged`, which unticks the box straight away and shows
    /// the log's parameters in the Full Parameter List - here, a list in this window in its
    /// columns.
    ///
    /// **One deliberate difference.** The C# puts the log's parameters in
    /// `MainV2.comPort.MAV.param`, the connected vehicle's list, which is how its parameter
    /// screen gets them; with a vehicle connected that replaces the vehicle's parameters on the
    /// screen that writes them back. The list here is the log's and nothing else's.
    /// `// C#: Log/LogBrowse.cs:3805-3841`
    pub fn show_params(&mut self) {
        let mut rows = self.params.clone();
        mp_log::logparams::sort(&mut rows);
        self.status = Some(format!("{} parameters in the log", rows.len()));
        self.params_view = Some(ParamsView {
            defaults: mp_log::logparams::has_defaults(&rows),
            rows,
            first: 0,
        });
    }

    /// Closes the parameter list.
    pub fn close_params(&mut self) {
        self.params_view = None;
    }

    /// The parameter list on screen, if Show Params put it there.
    #[must_use]
    pub const fn params_view(&self) -> Option<&ParamsView> {
        self.params_view.as_ref()
    }

    /// Moves the parameter list by whole rows.
    pub fn scroll_params(&mut self, rows: isize) {
        if let Some(view) = self.params_view.as_mut() {
            let last = view.rows.len().saturating_sub(PARAM_ROWS);
            view.first = view.first.saturating_add_signed(rows).min(last);
        }
    }

    /// `CMB_preselect`'s items.
    #[must_use]
    pub fn graphs(&self) -> &[DisplayList] {
        &self.graphs
    }

    /// Opens or closes the drop-down's list, which opens with the selected item in view, half
    /// a page down where there is room.
    pub fn toggle_preselect_list(&mut self) {
        let last = self.graphs.len().saturating_sub(PRESELECT_ROWS);
        self.preselect_list = match self.preselect_list {
            Some(_) => None,
            None => Some(
                self.preselect
                    .unwrap_or(0)
                    .saturating_sub(PRESELECT_ROWS / 2)
                    .min(last),
            ),
        };
    }

    /// Moves the drop-down's list by whole items.
    pub fn scroll_preselect(&mut self, rows: isize) {
        if let Some(first) = self.preselect_list.as_mut() {
            let last = self.graphs.len().saturating_sub(PRESELECT_ROWS);
            *first = first.saturating_add_signed(rows).min(last);
        }
    }

    /// An item chosen from the drop-down: if it is another, `SelectedIndexChanged`.
    pub fn choose_preselect(&mut self, index: usize) {
        self.preselect_list = None;
        if self.preselect == Some(index) || index >= self.graphs.len() {
            return;
        }
        self.preselect = Some(index);
        self.apply_preselect();
    }

    /// `CMB_preselect_SelectedIndexChanged`: the graph cleared, then every piece of the chosen
    /// set graphed through `GraphItem`'s expression path, and the chart labelled.
    ///
    /// "a/None" has no items and the handler returns before clearing anything. A piece whose
    /// types the log does not have plots nothing, silently, as `GraphItem` is told not to show
    /// errors; one that needs Python this evaluator lacks is left out and named in the status
    /// line. A piece whose label, and a space, starts an existing curve's is skipped, as
    /// `GraphItem` aborts on it - which with a `.` and ` R` ending every label here is a repeat
    /// on the right axis.
    /// `// C#: Log/LogBrowse.cs:1129-1152, 1270-1298, 3137-3167`
    pub fn apply_preselect(&mut self) {
        let Some(list) = self
            .preselect
            .and_then(|index| self.graphs.get(index))
            .cloned()
        else {
            return;
        };
        let Some(items) = list.items else {
            return;
        };
        let Some(log) = self.log.clone() else {
            return;
        };
        self.clear();
        let parsed: Vec<(
            &mp_log::mavgraph::DisplayItem,
            Result<mp_log::expression::Expression, mp_log::expression::ExpressionError>,
        )> = items
            .iter()
            .map(|item| (item, mp_log::expression::Expression::parse(&item.graphed())))
            .collect();
        let mut names: Vec<String> = parsed
            .iter()
            .filter_map(|(_, expression)| expression.as_ref().ok())
            .flat_map(mp_log::expression::Expression::messages)
            .collect();
        names.sort();
        names.dedup();
        // `GetEnumeratorType` over every type the pieces name, once for the lot.
        let records = mp_log::expression::records_in(&log, &names);
        let mut graphed = 0;
        let mut skipped = Vec::new();
        for (item, expression) in parsed {
            let expression = match expression {
                Ok(expression) => expression,
                Err(why) => {
                    skipped.push(format!("{}: {why}", item.graphed()));
                    continue;
                }
            };
            let prefix = format!("{}. ", item.graphed());
            if self
                .plotted
                .iter()
                .any(|shown| shown.label.starts_with(&prefix))
            {
                continue;
            }
            let samples = expression.evaluate(
                records
                    .iter()
                    .map(|(line, message, instance)| (*line, message, *instance)),
            );
            // `GraphItem_AddCurve` returns on an empty list.
            if samples.is_empty() {
                continue;
            }
            self.plotted.push(Plotted::expression(
                item,
                &samples,
                self.origin,
                self.x_axis(),
            ));
            graphed += 1;
        }
        self.zoom.reset();
        self.zoom_event();
        self.status = Some(if skipped.is_empty() {
            format!("{}: {graphed} graphed", list.name)
        } else {
            format!(
                "{}: {graphed} graphed; not graphed: {}",
                list.name,
                skipped.join("; ")
            )
        });
        self.preselect_outcome = Some((graphed, skipped));
    }

    /// The selected preselected graph's name.
    #[must_use]
    pub fn preselected(&self) -> Option<&str> {
        self.preselect
            .and_then(|index| self.graphs.get(index))
            .map(|list| list.name.as_str())
    }

    /// The prompt standing in the window, if there is one.
    #[must_use]
    pub const fn prompt(&self) -> Option<&Prompt> {
        self.prompt.as_ref()
    }

    fn ask(&mut self, kind: PromptKind, title: String, text: &str, value: &str) {
        let mut field = crate::textfield::TextField::new("");
        field.set(value);
        self.prompt = Some(Prompt {
            kind,
            title,
            text: text.to_owned(),
            field,
        });
        self.chart_menu = None;
        self.grid_menu = false;
    }

    /// `ProcessCmdKey`'s Ctrl+G: `InputBox.Show("Line no", "Enter Line Number", ref lineno)`,
    /// starting from 0.
    /// `// C#: Log/LogBrowse.cs:183-205`
    pub fn ask_go_to_line(&mut self) {
        self.ask(
            PromptKind::GoToLine,
            "Line no".to_owned(),
            "Enter Line Number",
            "0",
        );
    }

    /// `treeView1_DoubleClick`: the field's scaler and offset, starting from the one it has.
    /// `// C#: Log/LogBrowse.cs:3037-3077`
    pub fn ask_modifier(&mut self, field: &PlottableField) {
        let node = modifier::node_name(&field.message, field.instance, &field.field);
        let current = self
            .modifiers
            .get(&node)
            .map(|modifier| modifier.command.clone())
            .unwrap_or_default();
        self.ask(
            PromptKind::Modifier(node.clone()),
            modifier::title(&node),
            modifier::INSTRUCTIONS,
            &current,
        );
    }

    /// Export Visible: `SaveFileDialog`, suggesting `output.csv`, a name in the log's folder.
    /// `// C#: Log/LogBrowse.cs:3588-3593`
    pub fn ask_export_visible(&mut self) {
        self.ask(
            PromptKind::ExportVisible,
            SAVE_AS.to_owned(),
            "A file in the log's folder",
            export::VISIBLE_NAME,
        );
    }

    /// Export Files: `FolderBrowserDialog`, "Where to save the files", a folder in the log's
    /// folder, made if it is not there - the dialog's New Folder.
    /// `// C#: Log/LogBrowse.cs:3860-3866`
    pub fn ask_export_files(&mut self) {
        self.ask(
            PromptKind::ExportFiles,
            export::FILES_DESCRIPTION.to_owned(),
            "A folder in the log's folder",
            "",
        );
    }

    /// A key in the prompt's field: Enter is OK and Escape Cancel, as `InputBox`'s buttons are.
    pub fn prompt_key(&mut self, event: &gpui::KeyDownEvent) -> bool {
        let Some(prompt) = self.prompt.as_mut() else {
            return false;
        };
        match prompt.field.key(event) {
            crate::textfield::KeyOutcome::Submitted => self.prompt_ok(),
            crate::textfield::KeyOutcome::Cancelled => self.prompt = None,
            crate::textfield::KeyOutcome::Ignored => return false,
            crate::textfield::KeyOutcome::Changed => {}
        }
        true
    }

    /// Cancel.
    pub fn prompt_cancel(&mut self) {
        self.prompt = None;
    }

    /// OK: what the prompt asked for, done with the answer.
    pub fn prompt_ok(&mut self) {
        let Some(prompt) = self.prompt.take() else {
            return;
        };
        let answer = prompt.field.value().trim().to_owned();
        match prompt.kind {
            PromptKind::GoToLine => self.go_to_line(&answer),
            PromptKind::Modifier(node) => self.set_modifier(&node, &answer),
            PromptKind::ExportVisible => self.export_visible(&answer),
            PromptKind::ExportFiles => self.export_files(&answer),
        }
    }

    /// Ctrl+G's answer: `dataGridView1.CurrentCell = dataGridView1[1, line - 1]`, or "Line
    /// Doesn't Exist". A line that is not a number, which `int.Parse` throws on and nothing
    /// catches, is refused the same way.
    /// `// C#: Log/LogBrowse.cs:187-201`
    pub fn go_to_line(&mut self, text: &str) {
        let row = mp_log::netfmt::parse_i32(text)
            .and_then(|line| usize::try_from(line.checked_sub(1)?).ok());
        let shown = match (row, self.grid.as_mut().filter(|_| self.grid_filled)) {
            (Some(row), Some(grid)) => grid.show(row, 1),
            _ => false,
        };
        if shown {
            self.refused = None;
        } else {
            self.refuse("Line Doesn't Exist".to_owned());
        }
    }

    /// The scaler and offset typed for a field: kept if it parses, the field's old one removed
    /// either way. It applies the next time the field is graphed.
    /// `// C#: Log/LogBrowse.cs:3071-3077`
    pub fn set_modifier(&mut self, node: &str, text: &str) {
        self.modifiers.remove(node);
        if let Some(modifier) = Modifier::parse(text) {
            self.modifiers.insert(node.to_owned(), modifier);
        }
    }

    /// The fields with a scaler or offset.
    #[must_use]
    pub fn modifiers(&self) -> &BTreeMap<String, Modifier> {
        &self.modifiers
    }

    /// Where a name typed in an export's prompt goes: its last part, in the log's folder.
    fn beside_log(&self, name: &str) -> Option<std::path::PathBuf> {
        let leaf = name
            .rsplit(['/', '\\'])
            .next()
            .filter(|part| !part.is_empty() && *part != "." && *part != "..")?;
        Some(self.path.as_ref()?.parent()?.join(leaf))
    }

    /// Export Visible: every row the grid holds, each cell and a comma, a line each.
    /// `// C#: Log/LogBrowse.cs:3588-3609`
    pub fn export_visible(&mut self, name: &str) {
        let Some(path) = self.beside_log(name) else {
            self.refuse(format!("{name} is not a file name"));
            return;
        };
        let mut text = String::new();
        let mut rows = 0usize;
        if let Some(grid) = self.grid.as_mut().filter(|_| self.grid_filled) {
            let columns = grid.csv_columns();
            grid.for_each_row(|row| {
                text.push_str(&export::csv_line(&row.cells, columns));
                text.push_str(export::NEWLINE);
                rows += 1;
            });
        }
        match std::fs::write(&path, text) {
            Ok(()) => {
                self.refused = None;
                self.exported = Some(format!("{rows} rows"));
                self.status = Some(format!("{rows} rows written to {}", path.display()));
            }
            Err(err) => self.refuse(format!("could not write {}: {err}", path.display())),
        }
    }

    /// Export Files: every file the log carries, written into a folder.
    /// `// C#: Log/LogBrowse.cs:3857-3911`
    pub fn export_files(&mut self, name: &str) {
        let (Some(folder), Some(log)) = (self.beside_log(name), self.log.clone()) else {
            self.refuse(format!("{name} is not a folder name"));
            return;
        };
        let written = std::fs::create_dir_all(&folder)
            .map_err(|err| err.to_string())
            .and_then(|()| {
                // `logdata.GetEnumeratorType("FILE")`.
                let records: Vec<mp_log::dataflash::LogMessage> = log
                    .messages(&["FILE"])
                    .map(|(_, message)| message)
                    .collect();
                export::export_files(&records, &folder).map_err(|err| err.to_string())
            });
        match written {
            Ok(exported) => {
                self.refused = None;
                self.exported = Some(format!("{} files", exported.files.len()));
                self.status = Some(format!(
                    "{} files written to {}{}",
                    exported.files.len(),
                    folder.display(),
                    if exported.refused.is_empty() {
                        String::new()
                    } else {
                        format!("; refused: {}", exported.refused.join(", "))
                    }
                ));
            }
            Err(err) => self.refuse(format!("could not export to {}: {err}", folder.display())),
        }
    }

    /// Opens or closes the grid's context menu: a right click on it.
    pub fn toggle_grid_menu(&mut self) {
        self.grid_menu = !self.grid_menu && self.grid.is_some();
        self.chart_menu = None;
    }

    /// Whether the grid's context menu is open.
    #[must_use]
    pub const fn grid_menu(&self) -> bool {
        self.grid_menu
    }

    /// The map beside the chart.
    #[must_use]
    pub fn map(&self) -> Rc<RefCell<MapViewport>> {
        Rc::clone(&self.map)
    }

    /// What the map shows.
    #[must_use]
    pub const fn map_contents(&self) -> MapContents {
        self.map_contents
    }

    /// Shows the flight map's imagery on this map too.
    ///
    /// `myGMAP1.MapProvider = GCSViews.FlightData.mymap.MapProvider`, which the C# does when the
    /// window opens and again whenever the map is shown. A store of its own rather than the flight
    /// map's, as `myGMAP1` is a control of its own; it is only replaced when the provider changes,
    /// so opening one log after another does not start a fetch thread each time.
    /// `// C#: Log/LogBrowse.cs:217-218, 2990`
    pub fn use_imagery_of(&mut self, flight: &MapViewport) {
        let wanted = flight.source_id();
        if wanted == self.tiles.as_ref().map(|store| store.source().id) {
            return;
        }
        self.tiles = wanted
            .and_then(mp_tiles::source::source_by_id)
            .map(|source| {
                let cache =
                    mp_tiles::cache::TileCache::new(mp_tiles::cache::TileCache::default_root());
                // The same switch the flight map is built with: offline means the cache only.
                Arc::new(if std::env::var("MP_OFFLINE").is_ok() {
                    TileStore::offline(source, cache)
                } else {
                    TileStore::new(source, cache)
                })
            });
        if let Some(store) = &self.tiles {
            self.map.borrow_mut().set_tiles(Arc::clone(store));
        }
    }
}

/// What a log's routes hold, counted: the first GPS's route, or the `POS` route for a log whose
/// GPS never had a fix, and the logged mission's waypoints.
fn map_contents(routes: &Routes) -> MapContents {
    let (points, source) = if !routes.gps.is_empty() {
        (routes.gps.len(), "GPS")
    } else if !routes.pos.is_empty() {
        (routes.pos.len(), "POS")
    } else {
        (0, "")
    };
    MapContents {
        source,
        points,
        waypoints: routes.commands.len(),
    }
}

/// `BinaryLog.onFlightMode` as `MainV2` wires it: the name the firmware's mode table gives a
/// number. The tables are the parameter metadata's, as `getModesList` reads them; there is none
/// here for a tracker, whose modes stay numbers.
/// `// C#: MainV2.cs:3394-3418`
fn flight_mode_name(firmware: Firmware, mode: u64) -> Option<String> {
    let family = match firmware {
        Firmware::Copter => mp_vehicle::VehicleFamily::Copter,
        Firmware::Plane => mp_vehicle::VehicleFamily::Plane,
        Firmware::Rover => mp_vehicle::VehicleFamily::Rover,
        Firmware::Tracker => return None,
    };
    family
        .mode_name(u32::try_from(mode).ok()?)
        .map(str::to_owned)
}

/// `colourspastal`: the band colour for each mode number, as RGBA.
///
/// Fifteen opaque pastels from `ConvertFromRange` - each channel `(int)(c * 127) + 127` - and
/// then fifty-six from `ConvertFromHex`, which gives every one an alpha of 20.
/// `// C#: Log/LogBrowse.cs:909-1016`
const PASTEL: [u32; 71] = [
    0xfe7f_7fff,
    0x7ffe_7fff,
    0x7f7f_feff,
    0x7ffe_feff,
    0xfe7f_feff,
    0xfefe_7fff,
    0xfebe_7fff,
    0xfe7f_beff,
    0xbefe_7fff,
    0x7ffe_beff,
    0xbe7f_feff,
    0x7fbe_feff,
    0xfebe_beff,
    0xbefe_beff,
    0xbebe_feff,
    0x5757_ff14,
    0x62a9_ff14,
    0x62d0_ff14,
    0x06dc_fb14,
    0x01fc_ef14,
    0x03eb_a614,
    0x01f3_3e14,
    0x6a6a_ff14,
    0x75b4_ff14,
    0x75d6_ff14,
    0x24e0_fb14,
    0x1ffe_f314,
    0x03f3_ab14,
    0x0afe_4714,
    0x7979_ff14,
    0x86bc_ff14,
    0x8adc_ff14,
    0x3de4_fc14,
    0x5ffe_f714,
    0x33fd_c014,
    0x4bfe_7814,
    0x8c8c_ff14,
    0x99c7_ff14,
    0x99e0_ff14,
    0x63e9_fc14,
    0x74fe_f814,
    0x62fd_ce14,
    0x72fe_9514,
    0x9999_ff14,
    0x99c7_ff14,
    0xa8e4_ff14,
    0x75ec_fd14,
    0x92fe_f914,
    0x7dfd_d714,
    0x8bfe_a814,
    0xaaaa_ff14,
    0xa8cf_ff14,
    0xbbeb_ff14,
    0x8cef_fd14,
    0xa5fe_fa14,
    0x8ffe_dd14,
    0xa3fe_ba14,
    0xbbbb_ff14,
    0xbbda_ff14,
    0xcef0_ff14,
    0xacf3_fd14,
    0xb5ff_fc14,
    0xa5fe_e314,
    0xb5ff_c814,
    0xcaca_ff14,
    0xd0e6_ff14,
    0xd9f3_ff14,
    0xc0f7_fe14,
    0xceff_fd14,
    0xbefe_eb14,
    0xcaff_d814,
];

/// A mode number's band colour, or `None` past the end of the table, where the C# throws.
fn pastel(number: i64) -> Option<u32> {
    usize::try_from(number)
        .ok()
        .and_then(|index| PASTEL.get(index).copied())
}

/// Seconds GPS time is ahead of UTC, as `gpsTimeToTime` asks: for today, not for the log.
fn leap_seconds_now() -> i64 {
    use chrono::Datelike as _;
    let today = chrono::Local::now();
    mp_log::index::leap_seconds_gps(today.year(), today.month())
}

/// The value ranges the chart's axes take over a time window.
///
/// One left axis per unit and one right axis for everything on the right. That asymmetry is the
/// C#'s: `GraphItem_AddCurve` creates a Y axis per unit on the left, and would on the right too,
/// but `leftorrightaxis` then forces every right-hand curve onto `Y2Axis` index 0, so the
/// per-unit right axes it just created sit empty and `CleanupYAxis` hides them. Reproduced as it
/// behaves rather than as it reads; the owner can rule otherwise, and this is the site.
/// `// C#: Log/LogBrowse.cs:1633-1660, 2787-2799, 1701-1725`
#[derive(Debug, Default, PartialEq)]
pub struct Axes {
    /// The left axes, keyed by unit.
    pub left: BTreeMap<String, mp_chart::Range>,
    /// The right axis, if anything is on it.
    pub right: Option<mp_chart::Range>,
}

impl Axes {
    /// Auto-ranges every axis over the series on it.
    #[must_use]
    pub fn over(plotted: &[Plotted], from: f64, to: f64) -> Self {
        let mut by_unit: BTreeMap<String, Vec<&Series>> = BTreeMap::new();
        let mut on_right: Vec<&Series> = Vec::new();
        for shown in plotted {
            match shown.axis {
                Axis::Left => by_unit
                    .entry(shown.unit.clone())
                    .or_default()
                    .push(&shown.series),
                Axis::Right => on_right.push(&shown.series),
            }
        }
        let left = by_unit
            .into_iter()
            .filter_map(|(unit, series)| {
                mp_chart::auto_range(&series, from, to).map(|range| (unit, range))
            })
            .collect();
        let right = if on_right.is_empty() {
            None
        } else {
            mp_chart::auto_range(&on_right, from, to)
        };
        Self { left, right }
    }

    /// Whether there is anything to draw against.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.left.is_empty() && self.right.is_none()
    }

    /// The range a series is drawn against.
    #[must_use]
    pub fn range_for(&self, shown: &Plotted) -> Option<mp_chart::Range> {
        match shown.axis {
            Axis::Left => self.left.get(&shown.unit).copied(),
            Axis::Right => self.right,
        }
    }
}

/// The colours traces get, by position. The same table the tuning graph uses, so a field looks the
/// same live and afterwards.
const TRACE_COLOURS: &[u32] = &[
    theme::ACCENT,
    theme::OK,
    theme::WARN,
    theme::ALERT,
    0x9f_7fff,
    0x7f_ffcc,
];

/// The whole screen.
///
/// Arranged as `LogBrowse` arranges it, as far as the widgets allow:
///
/// ```text
///   splitContainerAllTree
///   ├─ Panel1  splitContainerZgGrid
///   │          ├─ Panel1  zg1 (the chart) beside myGMAP1 (the map)
///   │          └─ Panel2  the button strip, then dataGridView1
///   └─ Panel2  treeView1 - the field tree, on the RIGHT
/// ```
///
/// So: the chart, with the map beside it taking half the width once the Map box shows it; under
/// both, the strip of buttons and check boxes, and then the grid once the Data Table box shows
/// it; the field list down the right-hand side. The log's name and open button sit above it all,
/// where a single window has room for them and the C# has a Load A Log button in the strip and a
/// file dialog.
///
/// Not ported from the strip: Remove Item (which the resx hides anyway), Show Params, and the
/// preselected graphs. The boxes start as the C# starts them but are not remembered between
/// sessions, where the C# keeps Map, Time, Data Table, Mode, Errors and MSG in its settings.
/// `// C#: Log/LogBrowse.designer.cs:136-390; Log/LogBrowse.resx (strip at y 3, x 3 to 971)`
pub fn screen(
    browse: &LogBrowse,
    name: &crate::textfield::TextField,
    focus: &Focus<'_>,
    search: &str,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    div()
        .id("log-screen")
        .flex()
        .flex_1()
        .min_h(px(0.0))
        // Never wider than the window. A flex item's automatic minimum width is its content's
        // min-content, and the data grid's rows made that wider than 1600 px, which pushed the
        // field list past the window's right edge where no click could reach it.
        .min_w(px(0.0))
        .gap_2()
        .p_2()
        // `ProcessCmdKey`: the form sees Ctrl+G whichever of its controls has the keyboard.
        .track_focus(focus.screen)
        .on_key_down(cx.listener(|this, event: &gpui::KeyDownEvent, window, cx| {
            let keystroke = &event.keystroke;
            if keystroke.modifiers.control
                && keystroke.key.eq_ignore_ascii_case("g")
                && this.log_browse.is_open()
            {
                this.log_browse.ask_go_to_line();
                window.focus(&this.log_prompt_focus, cx);
                cx.stop_propagation();
                cx.notify();
            }
        }))
        // Left: everything about the plot, top to bottom.
        .child(
            div()
                .flex()
                .flex_col()
                .flex_1()
                .min_w(px(0.0))
                .gap_2()
                .child(file_panel(browse, name, focus.name, focus.name_focused, cx))
                .children(
                    browse
                        .prompt()
                        .map(|prompt| prompt_panel(prompt, focus.prompt, focus.prompt_focused, cx)),
                )
                .children(browse.is_open().then(|| chart_row(browse, cx)))
                .children(browse.is_open().then(|| button_strip(browse, cx)))
                .children(
                    browse
                        .preselect_list
                        .filter(|_| browse.is_open())
                        .map(|first| preselect_list(browse, first, cx)),
                )
                .children(browse.params_view().map(|view| params_panel(view, cx)))
                .children(
                    browse
                        .grid()
                        .filter(|_| browse.strip().get(Check::DataTable))
                        .map(|grid| grid_panel(grid, browse.grid_menu(), cx)),
                ),
        )
        // Right: the field tree, which is where LogBrowse puts it.
        .children(
            browse
                .is_open()
                .then(|| field_panel(browse, search, focus, cx)),
        )
        .into_any_element()
}

/// The keyboard focus the screen's text fields and its own key handling take.
pub struct Focus<'a> {
    /// The log's name.
    pub name: &'a gpui::FocusHandle,
    /// Whether the name has it.
    pub name_focused: bool,
    /// A prompt's field.
    pub prompt: &'a gpui::FocusHandle,
    /// Whether the prompt's field has it.
    pub prompt_focused: bool,
    /// The screen itself, for Ctrl+G.
    pub screen: &'a gpui::FocusHandle,
    /// `txt_info`.
    pub info: &'a gpui::FocusHandle,
    /// Whether `txt_info` has it.
    pub info_focused: bool,
}

/// A dialog in the window: `InputBox`, `SaveFileDialog` or `FolderBrowserDialog`, as a title,
/// what it asks, a field, OK and Cancel.
/// `// C#: ExtLibs/Controls/InputBox.cs`
fn prompt_panel(
    prompt: &Prompt,
    focus: &gpui::FocusHandle,
    focused: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    panel(
        "prompt",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(
                div()
                    .text_sm()
                    .text_color(rgb(theme::TEXT))
                    .child(prompt.title.clone()),
            )
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(prompt.text.clone()),
            )
            .child(
                div()
                    .flex()
                    .items_center()
                    .gap_2()
                    .child(crate::textfield::text_field(
                        "log-prompt-field",
                        &prompt.field,
                        focus,
                        focused,
                        px(360.0),
                        cx.listener(|this, event: &gpui::KeyDownEvent, _window, cx| {
                            if this.log_browse.prompt_key(event) {
                                cx.stop_propagation();
                                cx.notify();
                            }
                        }),
                    ))
                    .child(action(
                        "log-prompt-ok",
                        "OK",
                        theme::ACCENT,
                        true,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.log_browse.prompt_ok();
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "log-prompt-cancel",
                        "Cancel",
                        theme::ACCENT,
                        true,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.log_browse.prompt_cancel();
                            cx.notify();
                        }),
                    )),
            ),
    )
    .into_any_element()
}

/// The drop-down's list: `CMB_preselect`, 300 pixels wide as its `DropDownWidth` says, a page of
/// names at a time; the wheel moves it.
/// `// C#: Log/LogBrowse.designer.cs:334-340`
fn preselect_list(
    browse: &LogBrowse,
    first: usize,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let mut list = div()
        .id("log-preselect-list")
        .flex()
        .flex_col()
        .w(px(300.0))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .on_scroll_wheel(
            cx.listener(|this, event: &gpui::ScrollWheelEvent, _window, cx| {
                let delta = event.delta.pixel_delta(px(ROW_HEIGHT));
                #[allow(clippy::cast_possible_truncation)] // a wheel event is a few rows
                let rows = -(f32::from(delta.y) / ROW_HEIGHT).round() as isize;
                this.log_browse.scroll_preselect(rows);
                cx.stop_propagation();
                cx.notify();
            }),
        );
    for (index, graph) in browse
        .graphs()
        .iter()
        .enumerate()
        .skip(first)
        .take(PRESELECT_ROWS)
    {
        let id = format!("log-preselect-item-{index}");
        let chosen = browse.preselect == Some(index);
        list = list.child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .px_2()
                .h(px(ROW_HEIGHT))
                .text_xs()
                .whitespace_nowrap()
                .overflow_hidden()
                .bg(rgb(if chosen { theme::ACCENT } else { theme::PANEL }))
                .text_color(rgb(if chosen { theme::BG } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(graph.name.clone())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.log_browse.choose_preselect(index);
                    cx.notify();
                })),
        );
    }
    list.into_any_element()
}

/// Show Params' list, in the Full Parameter List's columns: Command, Value, Default - when any
/// parameter logged one - Units, Options and Desc, the last three from the parameter
/// documentation. The favourites column is left out: it reads and writes the settings'
/// `fav_params`, and this list writes nothing.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.Designer.cs:224-231;
/// GCSViews/ConfigurationView/ConfigRawParams.cs:563-676`
fn params_panel(view: &ParamsView, cx: &mut Context<MissionPlanner>) -> AnyElement {
    const WIDTHS: [f32; 6] = [180.0, 90.0, 90.0, 70.0, 220.0, 400.0];
    let heads = ["Command", "Value", "Default", "Units", "Options", "Desc"];
    let shown = |column: usize| column != 2 || view.defaults;
    let cell = |column: usize, text: String, colour: u32| {
        div()
            .w(px(WIDTHS.get(column).copied().unwrap_or(80.0)))
            .flex_shrink_0()
            .px_1()
            .h(px(ROW_HEIGHT))
            .overflow_hidden()
            .whitespace_nowrap()
            .text_xs()
            .text_color(rgb(colour))
            .child(text)
    };
    let mut header = div().flex().border_b_1().border_color(rgb(theme::BORDER));
    for (column, head) in heads.iter().enumerate() {
        if shown(column) {
            header = header.child(cell(column, (*head).to_owned(), theme::DIM));
        }
    }
    let mut body = div().flex().flex_col();
    for param in view.rows.iter().skip(view.first).take(PARAM_ROWS) {
        let meta = crate::metadata::lookup(&param.name);
        let texts = [
            param.name.clone(),
            param.value_text(),
            param.default_text(),
            meta.map(|meta| meta.units.to_owned()).unwrap_or_default(),
            meta.map(|meta| {
                mp_log::logparams::options_text(meta.range, meta.values).replace('\n', "  ")
            })
            .unwrap_or_default(),
            meta.map(|meta| meta.description.to_owned())
                .unwrap_or_default(),
        ];
        let mut line = div().flex();
        for (column, text) in texts.into_iter().enumerate() {
            if shown(column) {
                line = line.child(cell(column, text, theme::TEXT));
            }
        }
        body = body.child(line);
    }
    let last = (view.first + PARAM_ROWS).min(view.rows.len());
    panel(
        "params",
        // Measured, so a script can put the wheel over it.
        crate::probe::measured("log-params", div())
            .id("log-params")
            .flex()
            .flex_col()
            .gap_1()
            .on_scroll_wheel(
                cx.listener(|this, event: &gpui::ScrollWheelEvent, _window, cx| {
                    let delta = event.delta.pixel_delta(px(ROW_HEIGHT));
                    #[allow(clippy::cast_possible_truncation)] // a wheel event is a few rows
                    let rows = -(f32::from(delta.y) / ROW_HEIGHT).round() as isize;
                    this.log_browse.scroll_params(rows);
                    cx.stop_propagation();
                    cx.notify();
                }),
            )
            .child(
                div()
                    .id("log-params-scroll")
                    .w_full()
                    .overflow_x_scroll()
                    .child(div().flex().flex_col().child(header).child(body)),
            )
            .child(
                div()
                    .flex()
                    .items_center()
                    .gap_2()
                    .child(div().text_xs().text_color(rgb(theme::DIM)).child(
                        if view.rows.is_empty() {
                            "the log has no PARM records".to_owned()
                        } else {
                            format!("rows {} to {last} of {}", view.first + 1, view.rows.len())
                        },
                    ))
                    .child(action(
                        "log-params-close",
                        "Close",
                        theme::ACCENT,
                        true,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.log_browse.close_params();
                            cx.notify();
                        }),
                    )),
            ),
    )
    .into_any_element()
}

/// Opening a log.
fn file_panel(
    browse: &LogBrowse,
    name: &crate::textfield::TextField,
    name_focus: &gpui::FocusHandle,
    focused: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    panel(
        "log",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(
                div()
                    .flex()
                    .items_center()
                    .gap_2()
                    .child(crate::textfield::text_field(
                        "log-name",
                        name,
                        name_focus,
                        focused,
                        px(320.0),
                        cx.listener(|this, event: &gpui::KeyDownEvent, _window, cx| {
                            match this.log_name.key(event) {
                                crate::textfield::KeyOutcome::Submitted => {
                                    this.log_browse.use_imagery_of(&this.map.borrow());
                                    this.open_log();
                                }
                                crate::textfield::KeyOutcome::Cancelled => this.log_name.clear(),
                                crate::textfield::KeyOutcome::Ignored => return,
                                crate::textfield::KeyOutcome::Changed => {}
                            }
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "log-open",
                        "open",
                        theme::ACCENT,
                        true,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.log_browse.use_imagery_of(&this.map.borrow());
                            this.open_log();
                            cx.notify();
                        }),
                    )),
            )
            .children(browse.status().map(|status| {
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(status.to_owned())
            })),
    )
    .into_any_element()
}

/// Height of the chart's plotting area.
const PLOT_HEIGHT: f32 = 260.0;

/// Height of the strips above and below the plotting area that labels drawn outside it use.
///
/// ZedGraph puts every other label on the far side of the edge it is anchored to - above the top
/// of the chart, below the bottom - so that neighbours do not sit on each other, and those land
/// over the title and the x axis's numbers.
const LABEL_LANE: f32 = 18.0;

/// The cursor line: `Color.LightGray`.
const CURSOR: u32 = 0xd3_d3_d3;

/// Where a label sits against the edge of the plotting area it is anchored to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Place {
    /// `AlignV.Bottom` at the top edge: above the chart.
    AboveTop,
    /// `AlignV.Top` at the top edge: hanging inside it.
    BelowTop,
    /// `AlignV.Bottom` at the bottom edge: standing inside it.
    AboveBottom,
    /// `AlignV.Top` at the bottom edge: below the chart.
    BelowBottom,
}

/// One label for the chart, placed.
#[derive(Debug, Clone, PartialEq)]
struct Label {
    /// Its text.
    text: String,
    /// Its x, the left edge of its box: `AlignH.Left`.
    x: f64,
    /// Above or below which edge.
    place: Place,
    /// Filled red, as errors and events are.
    alert: bool,
}

/// Every label the chart carries now, in the C#'s places.
///
/// Mode changes and messages stand on the bottom edge, alternately inside and below it; errors
/// and events hang from the top edge, alternately above and inside it, in red; minutes hang
/// inside the top edge. Each list alternates on its own, as each `Draw` method keeps its own
/// `top`. A label that cannot be placed on the axis - a record without a time on a time axis -
/// is left off.
/// `// C#: Log/LogBrowse.cs:1731-2183`
fn labels(browse: &LogBrowse) -> Vec<Label> {
    let mut labels = Vec::new();
    // The first of each list goes on the first side, the next on the second, and so on.
    let mut add = |marks: &mut dyn Iterator<Item = &Mark>, sides: (Place, Place), alert: bool| {
        for (index, mark) in marks.enumerate() {
            if let Some(x) = browse.x_of(mark.line, mark.time_us) {
                labels.push(Label {
                    text: mark.text.clone(),
                    x,
                    place: if index % 2 == 0 { sides.0 } else { sides.1 },
                    alert,
                });
            }
        }
    };
    let bottom = (Place::AboveBottom, Place::BelowBottom);
    let top = (Place::AboveTop, Place::BelowTop);
    if browse.shows(Check::Mode) {
        add(
            &mut browse.drawn_modes().iter().map(|change| &change.mark),
            bottom,
            false,
        );
    }
    if browse.shows(Check::Errors) {
        add(&mut browse.overlays.errors.iter(), top, true);
    }
    if browse.labelled && browse.x_axis() == XAxis::Line {
        add(
            &mut browse.overlays.minutes.iter(),
            (Place::BelowTop, Place::BelowTop),
            false,
        );
    }
    if browse.shows(Check::Events) {
        add(&mut browse.overlays.events.iter(), top, true);
    }
    if browse.shows(Check::Msg) {
        add(&mut browse.overlays.messages.iter(), bottom, false);
    }
    labels
}

/// The bands `DrawModes` draws along the top 2% of the chart, behind the curves, in the order it
/// adds them: start, end and colour.
///
/// Every band starts at the left edge - `prevx` is never moved on - and runs to its mode change,
/// in the colour of the mode before it; a last band runs the whole width in the colour of the
/// last mode. ZedGraph draws its list last first, so the first band added, the shortest, ends up
/// on top: what shows is each stretch between changes in the colour of the mode in force there,
/// and with no mode changes at all, one band the colour of mode 0. A mode number past the end of
/// the colour table throws, and nothing after it is drawn.
/// `// C#: Log/LogBrowse.cs:1900-2018`
fn mode_bands(browse: &LogBrowse, low: f64, high: f64) -> Vec<(f64, f64, u32)> {
    let mut bands = Vec::new();
    let mut previous = 0;
    for change in browse.drawn_modes() {
        let before = std::mem::replace(&mut previous, change.number);
        let Some(colour) = pastel(before) else {
            return bands;
        };
        let Some(at) = browse.x_of(change.mark.line, change.mark.time_us) else {
            continue;
        };
        if low < high && at > low {
            bands.push((low, at.max(low).min(high), colour));
        }
    }
    if let Some(colour) = pastel(previous) {
        bands.push((low.min(high), high, colour));
    }
    bands
}

/// A label's box.
fn chart_label(label: &Label, fraction: f32) -> AnyElement {
    let (fill, text) = if label.alert {
        (theme::ALERT, theme::BG)
    } else {
        (theme::PANEL, theme::TEXT)
    };
    let element = div()
        .absolute()
        .left(relative(fraction))
        .h(px(LABEL_LANE))
        .px_1()
        .flex()
        .items_center()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(fill))
        .text_xs()
        .text_color(rgb(text))
        .whitespace_nowrap()
        .child(label.text.clone());
    match label.place {
        Place::AboveTop => element.top(px(0.0)),
        Place::BelowTop => element.top(px(LABEL_LANE)),
        Place::AboveBottom => element.bottom(px(LABEL_LANE)),
        Place::BelowBottom => element.bottom(px(0.0)),
    }
    .into_any_element()
}

/// The chart: every series drawn against its own axis's range, with the labels and the cursor.
///
/// The axes have no tick marks yet; their ranges are written out above the plot, one entry per
/// axis, and the legend carries each series' own extent - a plot with an auto-scaled axis and no
/// numbers on it says only "this went up and down".
///
/// A double click anywhere on it is `zg1_MouseDoubleClick`. The rest is ZedGraph's, as `zg1`
/// keeps its defaults: a left drag zooms to the rectangle, Ctrl and a left drag or a middle drag
/// pans, the wheel zooms, a right click opens its context menu, and with that menu's Show Point
/// Values on - it starts off, and `LogBrowse` never turns it on - the pointer shows the value of
/// the point nearest it. `zg1_MouseMoveEvent` itself only debounces.
/// `// C#: Log/LogBrowse.cs:3278-3297, 3574-3586;
/// ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.Events.cs:393-480, 670-800, 839-876`
#[allow(clippy::too_many_lines)] // one element tree, drawn in the C#'s layers
fn plot_panel(browse: &LogBrowse, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let plotted = browse.plotted();
    let scales = browse.scales();
    let range = scales.as_ref().map(|scales| scales.x);
    let (from, to) = range.unwrap_or((f64::INFINITY, f64::NEG_INFINITY));
    let axes = scales.map_or_else(Axes::default, |scales| Axes {
        left: scales.left,
        right: scales.right,
    });
    // Where an x sits across the plotting area, if it is on it.
    let across = |x: f64| -> Option<f32> {
        let (from, to) = range?;
        if !(from..=to).contains(&x) {
            return None;
        }
        let span = to - from;
        #[allow(clippy::cast_possible_truncation)] // a fraction of a width
        Some(if span > 0.0 {
            ((x - from) / span) as f32
        } else {
            0.0
        })
    };

    let bounds = Rc::clone(&browse.chart_bounds);
    let mut plot = crate::probe::measured("log-chart", div())
        .absolute()
        .top(px(LABEL_LANE))
        .left_0()
        .right_0()
        .h(px(PLOT_HEIGHT))
        // Where it was laid out, for turning a double click into a place on the axis.
        .child(
            canvas(
                move |laid_out, _window, _cx| bounds.set(Some(laid_out)),
                |_bounds, (), _window, _cx| {},
            )
            .absolute()
            .size_full(),
        );
    if axes.is_empty() {
        plot = plot.child(
            div()
                .absolute()
                .top(px(110.0))
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child("choose a field below to plot it - left click for the left axis, right click for the right"),
        );
    }

    // Behind the curves: the mode bands, then the cursor.
    if let Some((from, to)) = range
        && browse.shows(Check::Mode)
    {
        for (start, end, colour) in mode_bands(browse, from, to).into_iter().rev() {
            let (Some(left), Some(right)) = (across(start), across(end)) else {
                continue;
            };
            plot = plot.child(
                div()
                    .absolute()
                    .top_0()
                    .left(relative(left))
                    .w(relative(right - left))
                    .h(relative(0.02))
                    .bg(rgba(colour)),
            );
        }
    }
    if let Some(left) = browse.cursor_x().and_then(across) {
        plot = plot.child(
            div()
                .absolute()
                .top_0()
                .left(relative(left))
                .h_full()
                .w(px(0.0))
                .border_l_2()
                .border_dashed()
                .border_color(rgb(CURSOR)),
        );
    }

    for (index, shown) in plotted.iter().enumerate() {
        let Some(range) = axes.range_for(shown) else {
            continue;
        };
        let colour = TRACE_COLOURS
            .get(index % TRACE_COLOURS.len())
            .copied()
            .unwrap_or(theme::TEXT);
        for column in mp_chart::reduce(&shown.series, from, to, COLUMNS) {
            // A zoomed axis leaves some of a curve above or below the plot; what is off it is
            // not drawn, as `IsClippedToChartRect` clips it.
            if range.fraction(column.low) > 1.0 || range.fraction(column.high) < 0.0 {
                continue;
            }
            #[allow(clippy::cast_precision_loss)]
            let left = column.index as f32 / COLUMNS as f32;
            #[allow(clippy::cast_possible_truncation)]
            let top = (1.0 - range.fraction(column.high)).clamp(0.0, 1.0) as f32;
            #[allow(clippy::cast_possible_truncation)]
            let bottom = (1.0 - range.fraction(column.low)).clamp(0.0, 1.0) as f32;
            let height = (bottom - top).max(0.004);
            plot = plot.child(
                div()
                    .absolute()
                    .left(relative(left))
                    .top(relative(top))
                    .w(px(1.5))
                    .h(relative(height))
                    .bg(rgb(colour)),
            );
        }
    }

    // The rectangle a left drag is drawing, dashed, as ZedGraph draws its reversible frame.
    if let Some((drag_from, drag_to)) = browse.zoom_drag() {
        #[allow(clippy::cast_possible_truncation)] // fractions of the plotting area
        let (left, top, right, bottom) = (
            drag_from.0.min(drag_to.0).clamp(0.0, 1.0) as f32,
            drag_from.1.min(drag_to.1).clamp(0.0, 1.0) as f32,
            drag_from.0.max(drag_to.0).clamp(0.0, 1.0) as f32,
            drag_from.1.max(drag_to.1).clamp(0.0, 1.0) as f32,
        );
        plot = plot.child(
            div()
                .absolute()
                .left(relative(left))
                .top(relative(top))
                .w(relative(right - left))
                .h(relative(bottom - top))
                .border_1()
                .border_dashed()
                .border_color(rgb(theme::TEXT)),
        );
    }
    // Show Point Values: the tooltip, beside the pointer.
    if let (Some(text), Some(pointer)) = (browse.point_tooltip(browse.chart_size()), browse.pointer)
    {
        #[allow(clippy::cast_possible_truncation)] // a fraction of the plotting area
        let (left, top) = (
            pointer.0.clamp(0.0, 0.8) as f32,
            pointer.1.clamp(0.0, 0.9) as f32,
        );
        plot = plot.child(
            crate::probe::measured("log-chart-tooltip", div())
                .absolute()
                .left(relative(left))
                .top(relative(top))
                .ml(px(12.0))
                .mt(px(12.0))
                .px_1()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::BG))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .whitespace_nowrap()
                .child(text),
        );
    }

    let chart = div()
        .id("log-chart-area")
        .relative()
        .w_full()
        .h(px(2.0f32.mul_add(LABEL_LANE, PLOT_HEIGHT)))
        .overflow_hidden()
        .child(plot)
        .children(
            labels(browse)
                .iter()
                .filter_map(|label| across(label.x).map(|at| chart_label(label, at))),
        )
        .children(browse.chart_menu().map(|at| chart_menu(browse, at, cx)))
        // `MouseDoubleClick`, which Windows raises on the second press; the first press starts a
        // rectangle, or with Ctrl a pan, as ZedGraph's `MouseDown` does.
        .on_mouse_down(
            MouseButton::Left,
            cx.listener(|this, event: &gpui::MouseDownEvent, _window, cx| {
                let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
                let Some(at) = this.log_browse.chart_point(x, y) else {
                    return;
                };
                if event.click_count == 2 {
                    this.log_browse.double_click(at.0);
                } else {
                    this.log_browse.chart_press(at, event.modifiers.control);
                }
                cx.notify();
            }),
        )
        .on_mouse_down(
            MouseButton::Middle,
            cx.listener(|this, event: &gpui::MouseDownEvent, _window, cx| {
                let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
                if let Some(at) = this.log_browse.chart_point(x, y) {
                    this.log_browse.chart_press(at, true);
                    cx.notify();
                }
            }),
        )
        .on_mouse_down(
            MouseButton::Right,
            cx.listener(|this, event: &gpui::MouseDownEvent, _window, cx| {
                let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
                if let Some(at) = this.log_browse.chart_point(x, y) {
                    this.log_browse.open_chart_menu(at);
                    cx.notify();
                }
            }),
        )
        .on_mouse_move(
            cx.listener(|this, event: &gpui::MouseMoveEvent, _window, cx| {
                let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
                if let Some(at) = this.log_browse.chart_point(x, y) {
                    this.log_browse.chart_move(at);
                    cx.notify();
                }
            }),
        )
        .on_mouse_up(
            MouseButton::Left,
            cx.listener(|this, _event: &gpui::MouseUpEvent, _window, cx| {
                this.log_browse.chart_release();
                cx.notify();
            }),
        )
        .on_mouse_up(
            MouseButton::Middle,
            cx.listener(|this, _event: &gpui::MouseUpEvent, _window, cx| {
                this.log_browse.chart_release();
                cx.notify();
            }),
        )
        .on_mouse_up_out(
            MouseButton::Left,
            cx.listener(|this, _event: &gpui::MouseUpEvent, _window, cx| {
                this.log_browse.chart_release();
                cx.notify();
            }),
        )
        .on_hover(cx.listener(|this, hovered: &bool, _window, cx| {
            if !*hovered {
                this.log_browse.chart_leave();
                cx.notify();
            }
        }))
        .on_scroll_wheel(
            cx.listener(|this, event: &gpui::ScrollWheelEvent, _window, cx| {
                let delta = event.delta.pixel_delta(px(WHEEL_NOTCH));
                if this.log_browse.chart_wheel_pixels(f32::from(delta.y)) {
                    cx.notify();
                }
                cx.stop_propagation();
            }),
        );

    // The axes, as text: which units the left side is split into and what each spans, and the
    // right axis's span.
    let mut readouts = div().flex().flex_wrap().gap_3();
    for (unit, range) in &axes.left {
        let name = if unit.is_empty() {
            "left".to_owned()
        } else {
            format!("left ({unit})")
        };
        readouts = readouts.child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(format!("{name}: {:.3} to {:.3}", range.low, range.high)),
        );
    }
    if let Some(range) = axes.right {
        readouts = readouts.child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(format!("right: {:.3} to {:.3}", range.low, range.high)),
        );
    }

    let mut legend = div().flex().flex_wrap().gap_3();
    for (index, shown) in plotted.iter().enumerate() {
        let colour = TRACE_COLOURS
            .get(index % TRACE_COLOURS.len())
            .copied()
            .unwrap_or(theme::TEXT);
        let low = shown
            .series
            .samples()
            .map(|sample| sample.value)
            .fold(f64::INFINITY, f64::min);
        let high = shown
            .series
            .samples()
            .map(|sample| sample.value)
            .fold(f64::NEG_INFINITY, f64::max);
        legend = legend.child(
            div()
                .flex()
                .items_center()
                .gap_1()
                .child(div().size_2().rounded_full().bg(rgb(colour)))
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .child(shown.label.clone()),
                )
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child(if low.is_finite() {
                            format!("{low:.3} to {high:.3}")
                        } else {
                            "no samples".to_owned()
                        }),
                ),
        );
    }

    panel(
        "plot",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(readouts)
            .child(chart)
            .child(div().text_xs().text_color(rgb(theme::DIM)).child(
                match (range, browse.x_axis()) {
                    (Some((from, to)), XAxis::Time) => {
                        format!("{}: {from:.1} to {to:.1}", XAxis::Time.title())
                    }
                    (Some((from, to)), XAxis::Line) => {
                        format!("{}: {from:.0} to {to:.0}", XAxis::Line.title())
                    }
                    (None, _) => String::new(),
                },
            ))
            .child(legend),
    )
    .into_any_element()
}

/// ZedGraph's context menu, as `contextMenuStrip1_Opening` builds it for `zg1`: Show Point
/// Values, ticked when on; the undo of the last zoom, pan or wheel, named for it; Undo All
/// Zoom/Pan; Set Scale to Default. The two undo items are drawn dimmed with nothing to undo, as
/// ZedGraph disables them. Copy and Save Image As are not here: gpui renders no image of a view
/// to copy or save. `LogBrowse`'s `Zg1_ContextMenuBuilder` adds nothing.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.ContextMenu.cs:97-205;
/// ExtLibs/ZedGraph/ZedGraph/ZedGraphLocale.resx; Log/LogBrowse.cs:324-357`
fn chart_menu(browse: &LogBrowse, at: (f64, f64), cx: &mut Context<MissionPlanner>) -> AnyElement {
    let undo = browse.zoom().top();
    let items: [(&'static str, String, bool); 4] = [
        ("show_val", "Show Point Values".to_owned(), true),
        (
            "unzoom",
            undo.map_or("Un-Zoom", view::Kind::undo_text).to_owned(),
            undo.is_some(),
        ),
        ("undo_all", "Undo All Zoom/Pan".to_owned(), undo.is_some()),
        ("set_default", "Set Scale to Default".to_owned(), true),
    ];
    let mut menu = div()
        .absolute()
        .flex()
        .flex_col()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL));
    for (name, text, enabled) in items {
        let id = match name {
            "show_val" => "log-chart-menu-show_val",
            "unzoom" => "log-chart-menu-unzoom",
            "undo_all" => "log-chart-menu-undo_all",
            _ => "log-chart-menu-set_default",
        };
        // `item.Checked`: a tick beside Show Point Values while it is on.
        let ticked = name == "show_val" && browse.point_values();
        let mut item = crate::probe::measured(id, div())
            .id(id)
            .flex()
            .items_center()
            .gap_1()
            .px_2()
            .py(px(2.0))
            .text_xs()
            .whitespace_nowrap()
            .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
            .child(div().size_2().rounded_sm().bg(rgb(if ticked {
                theme::ACCENT
            } else {
                theme::PANEL
            })))
            .child(text);
        if enabled {
            item = item
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.log_browse.chart_menu_item(name);
                    cx.notify();
                }));
        }
        menu = menu.child(item);
    }
    #[allow(clippy::cast_possible_truncation)] // a fraction of the plotting area
    let (left, down) = (at.0.clamp(0.0, 0.8) as f32, at.1.clamp(0.0, 0.7) as f32);
    menu.left(relative(left))
        .top(px(down.mul_add(PLOT_HEIGHT, LABEL_LANE)))
        .into_any_element()
}

/// `splitContainerZgMap`: the chart, and the map beside it once the Map box shows it.
///
/// `CHK_map_CheckedChanged` uncollapses the map's panel and sets the splitter to half the width,
/// and the designer puts the chart on the left; collapsed, the chart has the whole width.
/// `// C#: Log/LogBrowse.cs:246-247, 2980-2987`
fn chart_row(browse: &LogBrowse, cx: &mut Context<MissionPlanner>) -> AnyElement {
    div()
        .flex()
        .gap_2()
        .child(div().flex_1().min_w(px(0.0)).child(plot_panel(browse, cx)))
        .children(
            browse
                .strip()
                .get(Check::Map)
                .then(|| div().flex_1().min_w(px(0.0)).child(map_panel(browse))),
        )
        .into_any_element()
}

/// Height of the map, about the height of the chart panel beside it.
const MAP_HEIGHT: f32 = 300.0;

/// `myGMAP1`: where the log says the vehicle went.
///
/// Dragging pans and the wheel zooms about the cursor, as the designer sets `CanDragMap` and
/// `MousePositionWithoutCenter`. The routes `DrawMap` made are drawn over it in their colours,
/// with the waypoints and photos on top, and the four labels name the colours in the top-left
/// corner.
/// `// C#: Log/LogBrowse.designer.cs:153-232; Log/LogBrowse.cs:3735-3771`
fn map_panel(browse: &LogBrowse) -> AnyElement {
    let map = browse.map();
    let attribution = map.borrow().attribution();

    panel(
        "map",
        crate::probe::measured("log-map", div())
            .relative()
            .h(px(MAP_HEIGHT))
            .w_full()
            .overflow_hidden()
            .on_mouse_down(MouseButton::Left, {
                let map = map.clone();
                move |event: &gpui::MouseDownEvent, _window, _cx| {
                    map.borrow_mut()
                        .begin_drag(f32::from(event.position.x), f32::from(event.position.y));
                }
            })
            .on_mouse_move({
                let map = map.clone();
                move |event: &gpui::MouseMoveEvent, window, _cx| {
                    if event.pressed_button != Some(MouseButton::Left) {
                        return;
                    }
                    map.borrow_mut()
                        .drag_to(f32::from(event.position.x), f32::from(event.position.y));
                    window.refresh();
                }
            })
            .on_mouse_up(MouseButton::Left, {
                let map = map.clone();
                move |_event: &gpui::MouseUpEvent, _window, _cx| map.borrow_mut().end_drag()
            })
            .on_scroll_wheel({
                let map = map.clone();
                move |event: &gpui::ScrollWheelEvent, window, _cx| {
                    let delta = event.delta.pixel_delta(px(20.0));
                    map.borrow_mut().zoom(
                        f32::from(event.position.x),
                        f32::from(event.position.y),
                        f32::from(delta.y) / 20.0,
                    );
                    window.refresh();
                }
            })
            .child(crate::mapview::map_element(Rc::clone(&map)))
            // `mapoverlay`: the routes and markers `DrawMap` made, over the map.
            .child(routes_element(
                Rc::clone(&map),
                Rc::clone(&browse.drawn),
                Rc::clone(&browse.fit_routes),
            ))
            // `markeroverlay`, over the map and painted after it.
            .child(marker_element(Rc::clone(&map), browse.marker()))
            // `label1` to `label4`: each route's name in its colour, where the resx puts them.
            .children(ROUTE_LABELS.iter().map(|(text, colour, x, y)| {
                div()
                    .absolute()
                    .top(px(*y))
                    .left(px(*x))
                    .text_xs()
                    .text_color(rgb(*colour))
                    .child(*text)
            }))
            // Required by the imagery's licence wherever the imagery is shown.
            .children(attribution.map(|text| {
                div()
                    .absolute()
                    .bottom_1()
                    .right_1()
                    .px_1()
                    .rounded_sm()
                    .bg(rgb(theme::PANEL))
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(text)
            })),
    )
    .into_any_element()
}

/// `splitContainerButGrid.Panel1`: the strip of buttons and check boxes between the chart and
/// the grid.
///
/// In the C#'s order: Graph Left, Graph Right, Clear Graph, then the boxes - Map, Time, Data
/// Table, Show Params, the preselected graphs' drop-down, Mode, Errors, MSG, Events. The two
/// graph buttons act on the grid's current cell, not on the field list.
/// `// C#: Log/LogBrowse.designer.cs:240-347; Log/LogBrowse.resx (chk_params 594,
/// CMB_preselect 691)`
fn button_strip(browse: &LogBrowse, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let (before, after) = Check::STRIP.split_at(3);
    div()
        .flex()
        .flex_wrap()
        .items_center()
        .gap_2()
        .child(action(
            "log-graph-left",
            "Graph Left",
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.log_browse.graph_selected(Axis::Left);
                cx.notify();
            }),
        ))
        .child(action(
            "log-graph-right",
            "Graph Right",
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.log_browse.graph_selected(Axis::Right);
                cx.notify();
            }),
        ))
        .child(action(
            "log-clear",
            "Clear Graph",
            theme::ACCENT,
            !browse.plotted().is_empty(),
            cx.listener(|this, _event: &(), _window, cx| {
                this.log_browse.clear();
                cx.notify();
            }),
        ))
        .children(
            before
                .iter()
                .map(|check| check_box(*check, browse.strip().get(*check), cx)),
        )
        .child(params_box(cx))
        .child(preselect_box(browse, cx))
        .children(
            after
                .iter()
                .map(|check| check_box(*check, browse.strip().get(*check), cx)),
        )
        .into_any_element()
}

/// Show Params: a check box that unticks itself as it is ticked and shows the parameters, so it
/// is only ever drawn unticked.
/// `// C#: Log/LogBrowse.cs:3805-3810; Log/LogBrowse.resx (chk_params.Text)`
fn params_box(cx: &mut Context<MissionPlanner>) -> AnyElement {
    crate::probe::measured("log-chk-params", div())
        .id("log-chk-params")
        .flex()
        .items_center()
        .gap_1()
        .px_1()
        .cursor_pointer()
        .child(
            div()
                .size_3()
                .rounded_sm()
                .border_1()
                .border_color(rgb(theme::DIM)),
        )
        .child(
            div()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child("Show Params"),
        )
        .on_click(cx.listener(|this, _event, _window, cx| {
            this.log_browse.show_params();
            cx.notify();
        }))
        .into_any_element()
}

/// `CMB_preselect`: the selected graph's name, 100 pixels wide as the resx has it; a click opens
/// its list under the strip.
/// `// C#: Log/LogBrowse.designer.cs:334-340; Log/LogBrowse.resx (CMB_preselect.Size 100, 21)`
fn preselect_box(browse: &LogBrowse, cx: &mut Context<MissionPlanner>) -> AnyElement {
    crate::probe::measured("log-preselect", div())
        .id("log-preselect")
        .w(px(100.0))
        .h(px(21.0))
        .px_1()
        .flex()
        .items_center()
        .overflow_hidden()
        .whitespace_nowrap()
        .border_1()
        .border_color(rgb(if browse.preselect_list.is_some() {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(theme::ACTION))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .child(browse.preselected().unwrap_or_default().to_owned())
        .on_click(cx.listener(|this, _event, _window, cx| {
            this.log_browse.toggle_preselect_list();
            cx.notify();
        }))
        .into_any_element()
}

/// A check box: a square, filled when ticked, and its text; a click anywhere on either toggles
/// it, as a WinForms `CheckBox` does.
fn check_box(check: Check, ticked: bool, cx: &mut Context<MissionPlanner>) -> AnyElement {
    crate::probe::measured(check.id(), div())
        .id(check.id())
        .flex()
        .items_center()
        .gap_1()
        .px_1()
        .cursor_pointer()
        .child(
            div()
                .size_3()
                .flex()
                .items_center()
                .justify_center()
                .rounded_sm()
                .border_1()
                .border_color(rgb(if ticked { theme::ACCENT } else { theme::DIM }))
                .children(ticked.then(|| div().size_2().rounded_sm().bg(rgb(theme::ACCENT)))),
        )
        .child(
            div()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(check.text()),
        )
        .on_click(cx.listener(move |this, _event, _window, cx| {
            this.log_browse.toggle_check(check);
            // The handler `LoadLog2` adds: the box remembered as it now is.
            // `// C#: Log/LogBrowse.cs:451-460`
            if let Some(key) = check.setting() {
                this.persisted
                    .set(key, setting_text(this.log_browse.strip().get(check)));
            }
            cx.notify();
        }))
        .into_any_element()
}

/// `label1` to `label4`: text, `ForeColor`, and `Location` in the map's panel.
/// `// C#: Log/LogBrowse.designer.cs:178-204; Log/LogBrowse.resx (label1 4,8; label2 39,8;
/// label3 80,8; label4 115,9)`
const ROUTE_LABELS: [(&str, u32, f32, f32); 4] = [
    ("GPS", 0x00_00_ff, 4.0, 8.0),
    ("GPS2", 0x00_80_00, 39.0, 8.0),
    ("POS", 0xff_00_00, 80.0, 8.0),
    ("GPSB", 0xff_ff_00, 115.0, 9.0),
];

/// A route's name in the facts.
const fn route_name(kind: RouteKind) -> &'static str {
    match kind {
        RouteKind::Gps => "gps",
        RouteKind::Gps2 => "gps2",
        RouteKind::Gpsb => "gpsb",
        RouteKind::Pos => "pos",
        RouteKind::Cmd => "cmd",
    }
}

/// A route's pen as gpui paints it: its colour at `Color.FromArgb(127, ...)`.
fn route_pen(kind: RouteKind) -> gpui::Hsla {
    gpui::Hsla::from(rgba((kind.colour() << 8) | u32::from(RouteKind::ALPHA)))
}

/// `mapoverlay`: every route `DrawMap` made, two pixels wide in its pen, then the waypoint
/// markers - numbered, one for every `CMD` record - and the photo markers on top, as GMap draws
/// an overlay's routes and then its markers.
///
/// Painted after the map, from where the map says it put each place in the same frame, so the
/// routes move with it. `ZoomAndCenterRoutes` is done here too, the first time the map is
/// painted after `DrawMap`, because only then does the map know how big it is.
/// `// C#: Log/LogBrowse.cs:2437-2520; ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapOverlay.cs`
fn routes_element(
    map: Rc<RefCell<MapViewport>>,
    drawn: Rc<Routes>,
    fit: Rc<Cell<bool>>,
) -> impl IntoElement {
    canvas(
        |_bounds, _window, _cx| (),
        move |_bounds, (), window, cx| {
            if fit.replace(false) {
                let places: Vec<LatLon> = drawn
                    .places()
                    .filter_map(|(latitude, longitude)| LatLon::new(latitude, longitude).ok())
                    .collect();
                if map.borrow_mut().zoom_to_fit(&places) {
                    window.refresh();
                }
            }
            let map = map.borrow();
            let screen = |latitude: f64, longitude: f64| {
                LatLon::new(latitude, longitude)
                    .ok()
                    .and_then(|place| map.screen_of(place))
            };
            for kind in RouteKind::ALL {
                let points: Vec<(f32, f32)> = drawn
                    .route(kind)
                    .iter()
                    .filter_map(|point| screen(point.latitude, point.longitude))
                    .collect();
                paint_polyline(window, &points, route_pen(kind));
            }
            for command in drawn.commands.iter().chain(&drawn.repeats) {
                if let Some((x, y)) = screen(command.latitude, command.longitude) {
                    paint_waypoint(window, cx, x, y, &command.seq.to_string());
                }
            }
            for photo in &drawn.cameras {
                if let Some((x, y)) = screen(photo.latitude, photo.longitude) {
                    paint_photo(window, x, y);
                }
            }
        },
    )
    .absolute()
    .size_full()
}

/// A line through screen points, in chunks a gpui path can hold.
fn paint_polyline(window: &mut gpui::Window, points: &[(f32, f32)], colour: gpui::Hsla) {
    use gpui::{PathBuilder, point};
    for chunk in points.chunks(60_000) {
        if chunk.len() < 2 {
            continue;
        }
        let mut builder = PathBuilder::stroke(px(2.0));
        let mut chunk = chunk.iter();
        if let Some((x, y)) = chunk.next() {
            builder.move_to(point(px(*x), px(*y)));
        }
        for (x, y) in chunk {
            builder.line_to(point(px(*x), px(*y)));
        }
        if let Ok(path) = builder.build() {
            window.paint_path(path, colour);
        }
    }
}

/// `GMapMarkerWP`: a waypoint's marker, a round green head on a point, with its number.
fn paint_waypoint(window: &mut gpui::Window, cx: &mut gpui::App, x: f32, y: f32, label: &str) {
    use gpui::{BorderStyle, Corners, Edges, point, quad, size};
    window.paint_quad(quad(
        Bounds {
            origin: point(px(x - 8.0), px(y - 20.0)),
            size: size(px(16.0), px(16.0)),
        },
        Corners::all(px(8.0)),
        rgb(WAYPOINT),
        Edges::all(px(1.0)),
        rgb(0x00_00_00),
        BorderStyle::Solid,
    ));
    let run = gpui::TextRun {
        len: label.len(),
        font: window.text_style().font(),
        color: gpui::Hsla::from(rgb(0x00_00_00)),
        background_color: None,
        underline: None,
        strikethrough: None,
    };
    let line = window.text_system().shape_line(
        SharedString::from(label.to_owned()),
        px(10.0),
        &[run],
        None,
    );
    let width = f32::from(line.width());
    let _ = line.paint(
        point(px(x - width / 2.0), px(y - 18.0)),
        px(12.0),
        gpui::TextAlign::Left,
        None,
        window,
        cx,
    );
}

/// `GMapMarkerPhoto`: where a photo was taken, a small square.
fn paint_photo(window: &mut gpui::Window, x: f32, y: f32) {
    use gpui::{BorderStyle, Corners, Edges, point, quad, size};
    window.paint_quad(quad(
        Bounds {
            origin: point(px(x - 4.0), px(y - 4.0)),
            size: size(px(8.0), px(8.0)),
        },
        Corners::all(px(1.0)),
        rgb(0xff_ff_ff),
        Edges::all(px(1.0)),
        rgb(0x00_00_00),
        BorderStyle::Solid,
    ));
}

/// A waypoint marker's head.
const WAYPOINT: u32 = 0x3f_b9_50;

/// The marker's colour: `GMarkerGoogleType.pink_dot`.
const PIN: u32 = 0xff_69_b4;

/// `markeroverlay`: `GoToSample`'s `GMarkerGoogle`, a pink pin whose point is on the place.
///
/// A canvas over the map, painted after it, asking the map where it put the place in the same
/// frame - so the pin moves with the map when it is dragged or zoomed, as a marker on a GMap
/// overlay does.
/// `// C#: Log/LogBrowse.cs:59-60, 214-223, 3466-3478`
fn marker_element(map: Rc<RefCell<MapViewport>>, marker: Option<LatLon>) -> impl IntoElement {
    canvas(
        |_bounds, _window, _cx| (),
        move |_bounds, (), window, _cx| {
            let Some(place) = marker else {
                return;
            };
            let Some((x, y)) = map.borrow().screen_of(place) else {
                return;
            };
            paint_pin(window, x, y);
        },
    )
    .absolute()
    .size_full()
}

/// A map pin with its point at `x`, `y`: a round head over a tapering tail, a dark dot in the
/// head.
fn paint_pin(window: &mut gpui::Window, x: f32, y: f32) {
    use gpui::{BorderStyle, Corners, Edges, PathBuilder, point, quad, size};
    let mut tail = PathBuilder::fill();
    tail.move_to(point(px(x - 5.0), px(y - 15.0)));
    tail.line_to(point(px(x + 5.0), px(y - 15.0)));
    tail.line_to(point(px(x), px(y)));
    tail.line_to(point(px(x - 5.0), px(y - 15.0)));
    if let Ok(path) = tail.build() {
        window.paint_path(path, gpui::Hsla::from(rgb(PIN)));
    }
    window.paint_quad(quad(
        Bounds {
            origin: point(px(x - 7.0), px(y - 25.0)),
            size: size(px(14.0), px(14.0)),
        },
        Corners::all(px(7.0)),
        rgb(PIN),
        Edges::all(px(1.0)),
        rgb(0x00_00_00),
        BorderStyle::Solid,
    ));
    window.paint_quad(quad(
        Bounds {
            origin: point(px(x - 2.0), px(y - 20.0)),
            size: size(px(4.0), px(4.0)),
        },
        Corners::all(px(2.0)),
        rgb(0x3a_0a_2a),
        Edges::default(),
        rgb(0x00_00_00),
        BorderStyle::default(),
    ));
}

/// Width of a grid column: the line number, the time, the type, and then the fields.
///
/// A field is ten characters or so once a float is written as the single it was logged as, so a
/// message of a dozen fields fits beside the field list in a 1600-pixel window, and a wider one
/// scrolls sideways.
fn column_width(column: usize) -> f32 {
    match column {
        0 => 56.0,
        1 => 160.0,
        TYPE_COLUMN => 48.0,
        _ => 76.0,
    }
}

/// `dataGridView1`: the log's records, a screenful at a time.
///
/// A click on a cell makes it current - one cell, as `SelectionMode = CellSelect` and
/// `MultiSelect = false` have it. The wheel moves the rows. A click on a column header offers
/// the log's message types to filter the grid to, with Cancel to show every record again, which
/// is `dataGridView1_ColumnHeaderMouseClick`'s dialog laid out in the panel instead.
///
/// Every visible cell reports its position as `loggrid-<row>-<column>`, the row counted in the
/// grid as it is filtered, so a test can click one by name. A double click on a cell is
/// `CellDoubleClick`, and a right click anywhere on the grid opens its context menu,
/// `contextMenuStrip1`, at the top of the panel.
/// `// C#: Log/LogBrowse.designer.cs:87-105, 349-363; Log/LogBrowse.cs:2820-2896, 3525-3532`
fn grid_panel(grid: &Grid, menu_open: bool, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let columns = grid.columns();
    let current = grid.current();
    let width: f32 = (0..columns).map(column_width).sum();

    // Each cell's text sits in a child the full size of the cell, because the probe measures a
    // control's children: an empty cell would otherwise report a rectangle of no height on its
    // top edge, and a click there lands on the row above.
    let mut header = div().flex().h(px(ROW_HEIGHT));
    for (column, text) in grid.headers().into_iter().enumerate() {
        let id = format!("loggrid-head-{column}");
        header = header.child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .w(px(column_width(column)))
                .h_full()
                .flex_shrink_0()
                .px_1()
                .border_r_1()
                .border_b_1()
                .border_color(rgb(theme::BORDER))
                .text_xs()
                .text_color(rgb(theme::DIM))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(div().size_full().overflow_hidden().child(text))
                .on_click(cx.listener(|this, _event, _window, cx| {
                    this.log_browse.toggle_grid_chooser();
                    cx.notify();
                })),
        );
    }

    let mut body = div()
        .id("loggrid")
        .flex()
        .flex_col()
        .on_scroll_wheel(
            cx.listener(|this, event: &gpui::ScrollWheelEvent, _window, cx| {
                let delta = event.delta.pixel_delta(px(ROW_HEIGHT));
                this.log_browse.scroll_grid(f32::from(delta.y));
                // The panel around the grid would scroll too; the rows are what moved.
                cx.stop_propagation();
                cx.notify();
            }),
        );
    for row in grid.window() {
        let mut line = div().flex().h(px(ROW_HEIGHT));
        for column in 0..columns {
            let chosen = current == Some((row.row, column));
            let id = format!("loggrid-{}-{column}", row.row);
            let text = row.cells.get(column).cloned().unwrap_or_default();
            let at = (row.row, column);
            line = line.child(
                crate::probe::measured(id.clone(), div())
                    .id(SharedString::from(id))
                    .w(px(column_width(column)))
                    .h_full()
                    .flex_shrink_0()
                    .px_1()
                    .border_r_1()
                    .border_color(rgb(theme::BORDER))
                    .text_xs()
                    .bg(rgb(if chosen { theme::ACCENT } else { theme::PANEL }))
                    .text_color(rgb(if chosen { theme::BG } else { theme::TEXT }))
                    .cursor_pointer()
                    .child(div().size_full().overflow_hidden().child(text))
                    .on_click(
                        cx.listener(move |this, event: &gpui::ClickEvent, _window, cx| {
                            this.log_browse.select_cell(at.0, at.1);
                            if event.click_count() == 2 {
                                this.log_browse.grid_double_click(at.0);
                            }
                            cx.notify();
                        }),
                    ),
            );
        }
        body = body.child(line);
    }

    // `contextMenuStrip1`: Export Visible and Export Files.
    let menu = menu_open.then(|| {
        div()
            .flex()
            .gap_2()
            .child(action(
                "loggrid-menu-visible",
                "Export Visible",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), window, cx| {
                    this.log_browse.toggle_grid_menu();
                    this.log_browse.ask_export_visible();
                    window.focus(&this.log_prompt_focus, cx);
                    cx.notify();
                }),
            ))
            .child(action(
                "loggrid-menu-files",
                "Export Files",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), window, cx| {
                    this.log_browse.toggle_grid_menu();
                    this.log_browse.ask_export_files();
                    window.focus(&this.log_prompt_focus, cx);
                    cx.notify();
                }),
            ))
    });

    // The types to filter by, when a header has been clicked.
    let chooser = grid.is_choosing().then(|| {
        let mut chips = div().flex().flex_wrap().gap_1();
        for name in grid.types() {
            let id = format!("loggrid-type-{name}");
            let chosen = name.clone();
            chips = chips.child(
                crate::probe::measured(id.clone(), div())
                    .id(SharedString::from(id))
                    .px_2()
                    .py(px(1.0))
                    .rounded_sm()
                    .border_1()
                    .border_color(rgb(if grid.filter() == Some(name.as_str()) {
                        theme::ACCENT
                    } else {
                        theme::BORDER
                    }))
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .cursor_pointer()
                    .hover(|style| style.bg(rgb(theme::BORDER)))
                    .child(name)
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        this.log_browse.filter_grid(Some(&chosen));
                        cx.notify();
                    })),
            );
        }
        div()
            .flex()
            .items_start()
            .gap_2()
            .child(chips)
            .child(action(
                "loggrid-type-cancel",
                "Cancel",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.log_browse.filter_grid(None);
                    cx.notify();
                }),
            ))
    });

    // Where the window is in the log - what the C# grid's scroll bar shows by its thumb.
    let position = match grid.window().last() {
        Some(last) => format!(
            "rows {} to {} of {}{}",
            grid.first() + 1,
            last.row + 1,
            grid.rows(),
            grid.filter()
                .map(|name| format!(" {name}, of {} records", grid.records()))
                .unwrap_or_default()
        ),
        _ => format!(
            "no rows{}",
            grid.filter()
                .map(|name| format!(" of {name}"))
                .unwrap_or_default()
        ),
    };

    panel(
        "data",
        div()
            .id("loggrid-panel")
            .flex()
            .flex_col()
            .gap_1()
            .on_mouse_down(
                MouseButton::Right,
                cx.listener(|this, _event: &gpui::MouseDownEvent, _window, cx| {
                    this.log_browse.toggle_grid_menu();
                    cx.notify();
                }),
            )
            .children(menu)
            .children(chooser)
            .child(
                div()
                    .id("loggrid-scroll")
                    // Its own width comes from the column, never from the rows: without this
                    // the widest message's columns pushed the whole screen wider than the
                    // window and the field list off its right edge.
                    .w_full()
                    .min_w(px(0.0))
                    .overflow_x_scroll()
                    // The wheel moves rows; only a sideways movement scrolls the columns.
                    .restrict_scroll_to_axis()
                    .child(
                        div()
                            .flex()
                            .flex_col()
                            .w(px(width))
                            .child(header)
                            .child(body),
                    ),
            )
            .child(div().text_xs().text_color(rgb(theme::DIM)).child(position)),
    )
    .into_any_element()
}

/// The field list, filtered by the search box.
///
/// A left click graphs a field on the left axis and a right click on the right, as the tree in
/// `LogBrowse` does; either click on a plotted field removes it. A field on the right axis shows
/// ` R` after its name, which is how the C# marks the curve too.
fn field_panel(
    browse: &LogBrowse,
    search: &str,
    focus: &Focus<'_>,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let needle = search.trim().to_uppercase();
    let matching: Vec<&PlottableField> = browse
        .fields()
        .iter()
        .filter(|field| needle.is_empty() || field.to_string().to_uppercase().contains(&needle))
        .collect();
    let total = matching.len();

    let mut list = div().flex().flex_wrap().gap_1();
    for field in matching.into_iter().take(SHOWN_FIELDS) {
        let bits = browse.bits_of(field);
        if bits.is_empty() {
            list = list.child(field_chip(browse, field, cx));
            continue;
        }
        // A bitmask field's node, with its plus or minus and, expanded, its bits under it on a
        // line of their own, indented as a `TreeView` indents a child.
        // `// C#: Log/LogBrowse.cs:687-696`
        let label = field.to_string();
        let expanded = browse.is_expanded(field);
        let toggled = field.clone();
        let expander = crate::probe::measured(format!("logfield-{label}-expand"), div())
            .id(gpui::SharedString::from(format!("logfield-{label}-expand")))
            .w(px(14.0))
            .flex()
            .justify_center()
            .text_xs()
            .text_color(rgb(theme::DIM))
            .cursor_pointer()
            .child(if expanded { "-" } else { "+" })
            .on_click(
                cx.listener(move |this, _event: &gpui::ClickEvent, _window, cx| {
                    this.log_browse.toggle_expanded(&toggled);
                    cx.notify();
                }),
            );
        let mut node = div().w_full().flex().flex_col().gap_1().child(
            div()
                .flex()
                .items_center()
                .gap_1()
                .child(expander)
                .child(field_chip(browse, field, cx)),
        );
        if expanded {
            let mut children = div().pl(px(16.0)).flex().flex_wrap().gap_1();
            for bit in bits {
                children = children.child(bit_chip(browse, field, bit, cx));
            }
            node = node.child(children);
        }
        list = list.child(node);
    }

    // `txt_info`: docked along the bottom of the tree's panel, 40 pixels tall, multiline, a
    // `TextBox` like any other - selectable, copyable, editable.
    // `// C#: Log/LogBrowse.designer.cs:391-396; Log/LogBrowse.resx (txt_info)`
    let info = div().flex_shrink_0().child(crate::textfield::text_area(
        "log-txt-info",
        browse.info_field(),
        focus.info,
        focus.info_focused,
        gpui::relative(1.0),
        px(40.0),
        cx.listener(|this, event: &gpui::KeyDownEvent, _window, cx| {
            if this.log_browse.info_key(event) {
                cx.notify();
            }
        }),
    ));

    panel(
        "fields",
        div()
            .flex()
            .flex_col()
            .w(px(300.0))
            .flex_shrink_0()
            .gap_2()
            .min_h(px(0.0))
            .child(
                // Measured, so a script can wheel it until a field is inside its box.
                crate::probe::measured("log-fields", div())
                    .id("log-fields")
                    .flex()
                    .flex_col()
                    .flex_1()
                    .min_h(px(0.0))
                    .gap_2()
                    .overflow_y_scroll()
                    .child(div().text_xs().text_color(rgb(theme::DIM)).child(
                        if total > SHOWN_FIELDS {
                            format!(
                                "{total} fields, showing {SHOWN_FIELDS} - type in the box \
                                     above to narrow"
                            )
                        } else {
                            format!("{total} fields")
                        },
                    ))
                    .child(list),
            )
            .child(info),
    )
    // The list scrolls inside the window rather than making the screen as tall as itself: a
    // flex item's automatic minimum height is its content's, and two hundred chips are taller
    // than any window, which stretched the whole row and put the scroll bar out of reach.
    .min_h(px(0.0))
    .into_any_element()
}

/// One field's chip in the list: a node of the C#'s tree. A left click graphs it on the left
/// axis and a right click on the right, and either removes it when it is plotted.
fn field_chip(
    browse: &LogBrowse,
    field: &PlottableField,
    cx: &mut Context<MissionPlanner>,
) -> gpui::Stateful<gpui::Div> {
    let axis = browse.axis_of(field);
    let shown = axis.is_some();
    let label = field.to_string();
    let text = match axis {
        Some(Axis::Right) => format!("{label} R"),
        _ => label.clone(),
    };
    let chosen_left = field.clone();
    let chosen_right = field.clone();
    let hovered = field.clone();
    crate::probe::measured(format!("logfield-{label}"), div())
        .id(gpui::SharedString::from(format!("logfield-{label}")))
        .px_2()
        .py(px(1.0))
        .rounded_sm()
        .border_1()
        .border_color(rgb(if shown { theme::ACCENT } else { theme::BORDER }))
        .text_xs()
        .text_color(rgb(if shown { theme::ACCENT } else { theme::TEXT }))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .child(text)
        // `treeView1.NodeMouseHover`. `// C#: Log/LogBrowse.designer.cs:375`
        .on_hover(cx.listener(move |this, over: &bool, _window, cx| {
            if *over {
                this.log_browse.hover_field(&hovered, metadata::shared());
                cx.notify();
            }
        }))
        .on_click(
            cx.listener(move |this, event: &gpui::ClickEvent, window, cx| {
                // The first press of a double click ticked or unticked the field; the
                // second undoes that and asks for its scaler and offset, as a double click
                // on a node's text in the C#'s tree changes no tick.
                // `// C#: Log/LogBrowse.cs:3037-3077`
                this.log_browse.toggle(&chosen_left);
                if event.click_count() == 2 {
                    this.log_browse.ask_modifier(&chosen_left);
                    window.focus(&this.log_prompt_focus, cx);
                }
                cx.notify();
            }),
        )
        // gpui's `on_click` is the primary button only; the others arrive here. Only the
        // right one means anything, as in the C# tree, where `wasrightclick` is what
        // decides the axis.
        .on_aux_click(
            cx.listener(move |this, event: &gpui::ClickEvent, _window, cx| {
                if event.is_right_click() {
                    this.log_browse.graph(&chosen_right, Axis::Right);
                    cx.notify();
                }
            }),
        )
}

/// A bit's chip under its field: `add_field_node`'s `new_bit_node`, its text the bit's name and
/// its tooltip the bit's description. A left click graphs the bit on the left axis and a right
/// click on the right, and either removes it when it is plotted, as ticking and unticking the
/// C#'s node does.
///
/// A double click asks nothing: `treeView1_DoubleClick` on a bit's node reads the field's name
/// as an instance number, `int.Parse`, which throws.
/// `// C#: Log/LogBrowse.cs:687-696, 3037-3077, 3079-3128`
fn bit_chip(
    browse: &LogBrowse,
    field: &PlottableField,
    bit: &BitNode,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let axis = browse.axis_of_bit(field, &bit.text);
    let shown = axis.is_some();
    let id = format!("logfield-{field}.{}", bit.text);
    let text = match axis {
        Some(Axis::Right) => format!("{} R", bit.text),
        _ => bit.text.clone(),
    };
    let (left, right, hovered) = (field.clone(), field.clone(), field.clone());
    let (left_bit, right_bit, hovered_bit) = (bit.text.clone(), bit.text.clone(), bit.text.clone());
    let chip = crate::probe::measured(id.clone(), div())
        .id(gpui::SharedString::from(id))
        .px_2()
        .py(px(1.0))
        .rounded_sm()
        .border_1()
        .border_color(rgb(if shown { theme::ACCENT } else { theme::BORDER }))
        .text_xs()
        .text_color(rgb(if shown { theme::ACCENT } else { theme::TEXT }))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .child(text)
        // `treeView1.NodeMouseHover`. `// C#: Log/LogBrowse.designer.cs:375`
        .on_hover(cx.listener(move |this, over: &bool, _window, cx| {
            if *over {
                this.log_browse
                    .hover_bit(&hovered, &hovered_bit, metadata::shared());
                cx.notify();
            }
        }))
        .on_click(
            cx.listener(move |this, _event: &gpui::ClickEvent, _window, cx| {
                this.log_browse.graph_bit(&left, &left_bit, Axis::Left);
                cx.notify();
            }),
        )
        .on_aux_click(
            cx.listener(move |this, event: &gpui::ClickEvent, _window, cx| {
                if event.is_right_click() {
                    this.log_browse.graph_bit(&right, &right_bit, Axis::Right);
                    cx.notify();
                }
            }),
        );
    if bit.tip.is_empty() {
        return chip.into_any_element();
    }
    // `treeView1.ShowNodeToolTips = true`: the node's `ToolTipText`. `// C#: Log/LogBrowse.cs:702`
    let tip = gpui::SharedString::from(bit.tip.clone());
    chip.tooltip(move |_window, cx| -> gpui::AnyView {
        let tip = tip.clone();
        cx.new(|_| crate::config::rover_tuning::Tip(tip)).into()
    })
    .into_any_element()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/dataflash.bin")
    }

    fn field(message: &str, name: &str) -> PlottableField {
        PlottableField {
            message: message.to_owned(),
            instance: None,
            field: name.to_owned(),
            samples: 0,
        }
    }

    fn points(values: &[f64]) -> Vec<mp_log::plot::Point> {
        values
            .iter()
            .enumerate()
            .map(|(index, value)| mp_log::plot::Point {
                #[allow(clippy::cast_precision_loss)]
                seconds: index as f64,
                line: 100 + index,
                value: *value,
            })
            .collect()
    }

    fn unit(name: &str, multiplier: f64) -> FieldUnit {
        FieldUnit {
            unit: name.to_owned(),
            multiplier,
        }
    }

    /// `treeView1_TreeNodeMouseHover`: the pointer resting on a field puts the description
    /// `LogMetaData` has for its message and field in `txt_info`, instance or not; a field it
    /// does not know, or no metadata yet, leaves the box as it was.
    /// `// C#: Log/LogBrowse.cs:3778-3803`
    #[test]
    fn resting_on_a_field_shows_its_description() {
        let mut meta = metadata::MetaData::default();
        meta.parse(
            "<loggermessagefile><logformat name=\"ATT\">\
             <description>Canonical vehicle attitude</description><fields>\
             <field name=\"Roll\"><description>achieved vehicle roll</description></field>\
             </fields></logformat><logformat name=\"IMU\"><description>Inertial</description>\
             <fields><field name=\"AccX\"><description>acceleration along X axis</description>\
             </field></fields></logformat></loggermessagefile>",
        );
        let mut browse = LogBrowse::new();
        browse.hover_field(&field("ATT", "Roll"), None);
        assert_eq!(browse.info(), "");
        browse.hover_field(&field("ATT", "Roll"), Some(&meta));
        assert_eq!(browse.info(), "achieved vehicle roll");
        browse.hover_field(&field("ATT", "Pitch"), Some(&meta));
        browse.hover_field(&field("GPS", "Roll"), Some(&meta));
        assert_eq!(browse.info(), "achieved vehicle roll");
        let mut second = field("IMU", "AccX");
        second.instance = Some(1);
        browse.hover_field(&second, Some(&meta));
        assert_eq!(browse.info(), "acceleration along X axis");
        assert!(browse.facts().contains(&(
            "log.info".to_owned(),
            "acceleration along X axis".to_owned()
        )));
    }

    /// `txt_info` is a multi-line `TextBox`: the description in it can be selected and copied,
    /// and typed over, which the C# never reads back. The selection is a fact.
    /// `// C#: Log/LogBrowse.designer.cs:391-396; Log/LogBrowse.resx (txt_info.Multiline)`
    #[test]
    fn the_description_box_selects_copies_and_takes_typing() {
        fn key(
            name: &str,
            character: Option<&str>,
            control: bool,
            shift: bool,
        ) -> gpui::KeyDownEvent {
            let mut event = gpui::KeyDownEvent {
                keystroke: gpui::Keystroke {
                    modifiers: gpui::Modifiers::default(),
                    key: name.to_owned(),
                    key_char: character.map(ToOwned::to_owned),
                },
                is_held: false,
                prefer_character_input: false,
            };
            event.keystroke.modifiers.control = control;
            event.keystroke.modifiers.shift = shift;
            event
        }
        let mut meta = metadata::MetaData::default();
        meta.parse(
            "<loggermessagefile><logformat name=\"ATT\"><description>Attitude</description>\
             <fields><field name=\"Roll\"><description>achieved vehicle roll</description>\
             </field></fields></logformat></loggermessagefile>",
        );
        let mut browse = LogBrowse::new();
        browse.hover_field(&field("ATT", "Roll"), Some(&meta));
        assert!(browse.info_field().is_multiline());
        // Drawn, as the screen draws it.
        browse.info_field().show_caret();
        let selection = |browse: &LogBrowse| {
            browse
                .facts()
                .into_iter()
                .find(|(name, _)| name == "log.info.selection")
                .map(|(_, value)| value)
        };
        assert_eq!(selection(&browse).as_deref(), Some("none"));
        assert!(
            !browse.info_key(&key("a", Some("a"), true, false)),
            "select all"
        );
        assert_eq!(selection(&browse).as_deref(), Some("0,21"));
        assert_eq!(browse.info_field().selected_text(), "achieved vehicle roll");
        // A word, from the end, with Ctrl+Shift+Left.
        assert!(!browse.info_key(&key("end", None, true, false)));
        assert!(!browse.info_key(&key("left", None, true, true)));
        assert_eq!(browse.info_field().selected_text(), "roll");
        assert!(
            browse.info_key(&key("p", Some("p"), false, false)),
            "typed over"
        );
        assert_eq!(browse.info(), "achieved vehicle p");
        assert!(!browse.info_key(&key("escape", None, false, false)));
        assert_eq!(browse.info(), "achieved vehicle p");
    }

    /// The label is `MSG.Field (unit)`, and ` R` marks the right axis, as the C# names a curve.
    #[test]
    fn a_curve_is_labelled_with_its_unit_and_its_axis() {
        let left = Plotted::new(
            field("ATT", "Roll"),
            &points(&[1.0]),
            &unit("deg", 1.0),
            Axis::Left,
            XAxis::Time,
        );
        assert_eq!(left.label, "ATT.Roll (deg)");
        let right = Plotted::new(
            field("ATT", "Roll"),
            &points(&[1.0]),
            &unit("deg", 1.0),
            Axis::Right,
            XAxis::Time,
        );
        assert_eq!(right.label, "ATT.Roll (deg) R");
        let plain = Plotted::new(
            field("X", "Y"),
            &points(&[1.0]),
            &FieldUnit::default(),
            Axis::Left,
            XAxis::Time,
        );
        assert_eq!(plain.label, "X.Y", "no unit, no brackets");
    }

    /// The multiplier puts the stored value into its unit, once, here, not in the extraction.
    #[test]
    fn a_multiplier_scales_the_series_into_its_unit() {
        let shown = Plotted::new(
            field("GPS", "Spd"),
            &points(&[100.0, 250.0]),
            &unit("m/s", 0.1),
            Axis::Left,
            XAxis::Time,
        );
        let values: Vec<f64> = shown.series.samples().map(|sample| sample.value).collect();
        assert_eq!(values, vec![10.0, 25.0]);
        assert_eq!(shown.unit, "m/s");
    }

    /// One left axis per unit, and the right axis shared - the C#'s behaviour, not its intent.
    #[test]
    fn the_left_axis_is_one_per_unit_and_the_right_is_shared() {
        let plotted = vec![
            Plotted::new(
                field("ATT", "Roll"),
                &points(&[-10.0, 10.0]),
                &unit("deg", 1.0),
                Axis::Left,
                XAxis::Time,
            ),
            Plotted::new(
                field("BAT", "Volt"),
                &points(&[11.8, 12.6]),
                &unit("V", 1.0),
                Axis::Left,
                XAxis::Time,
            ),
            Plotted::new(
                field("RCIN", "C3"),
                &points(&[1000.0, 2000.0]),
                &unit("PWM", 1.0),
                Axis::Right,
                XAxis::Time,
            ),
            Plotted::new(
                field("BARO", "Alt"),
                &points(&[0.0, 50.0]),
                &unit("m", 1.0),
                Axis::Right,
                XAxis::Time,
            ),
        ];
        let axes = Axes::over(&plotted, 0.0, 1.0);

        assert_eq!(
            axes.left.len(),
            2,
            "degrees and volts each get an axis: {axes:?}"
        );
        let degrees = axes.left.get("deg").expect("a degrees axis");
        assert!(degrees.low <= -10.0 && degrees.high >= 10.0, "{degrees:?}");
        assert!(
            degrees.high < 100.0,
            "the degrees axis must not be stretched by the volts"
        );
        let volts = axes.left.get("V").expect("a volts axis");
        assert!(
            volts.low <= 11.8 && volts.high >= 12.6 && volts.low > 0.0,
            "{volts:?}"
        );

        let right = axes.right.expect("a right axis");
        assert!(
            right.low <= 0.0 && right.high >= 2000.0,
            "one axis spans both: {right:?}"
        );
        assert_eq!(axes.range_for(&plotted[2]), Some(right));
        assert_eq!(axes.range_for(&plotted[3]), Some(right));
        assert_eq!(axes.range_for(&plotted[0]), Some(*degrees));
    }

    /// With nothing plotted there is nothing to draw against.
    #[test]
    fn no_series_means_no_axes() {
        assert!(Axes::over(&[], 0.0, 1.0).is_empty());
        assert_eq!(Axes::default(), Axes::over(&[], 0.0, 1.0));
    }

    /// The real log: a right click puts a field on the right axis, labelled, and counts as such.
    #[test]
    fn a_right_click_graphs_a_field_on_the_right_axis() {
        let mut browse = LogBrowse::new();
        browse.open(&fixture());
        assert!(browse.is_open());
        let roll = browse
            .fields()
            .iter()
            .find(|field| field.message == "ATT" && field.field == "Roll")
            .expect("ATT.Roll is in the fixture")
            .clone();
        let pitch = browse
            .fields()
            .iter()
            .find(|field| field.message == "ATT" && field.field == "Pitch")
            .expect("ATT.Pitch is in the fixture")
            .clone();

        browse.toggle(&roll);
        browse.graph(&pitch, Axis::Right);

        assert_eq!(browse.plotted().len(), 2);
        assert_eq!(browse.axis_of(&roll), Some(Axis::Left));
        assert_eq!(browse.axis_of(&pitch), Some(Axis::Right));
        assert_eq!(browse.right_count(), 1);
        assert_eq!(browse.plotted()[1].label, "ATT.Pitch (deg) R");
        assert_eq!(
            browse.left_units().into_iter().collect::<Vec<_>>(),
            vec!["deg"]
        );
        assert!(
            browse.plotted()[0].series.len() > 100,
            "the samples were extracted"
        );
    }

    /// A click on a plotted field removes it, whichever button - unchecking in the C# tree.
    #[test]
    fn a_click_on_a_plotted_field_removes_it_whichever_button() {
        let mut browse = LogBrowse::new();
        browse.open(&fixture());
        let roll = browse
            .fields()
            .iter()
            .find(|field| field.message == "ATT" && field.field == "Roll")
            .expect("ATT.Roll")
            .clone();

        browse.graph(&roll, Axis::Left);
        assert!(browse.axis_of(&roll).is_some());
        browse.graph(&roll, Axis::Right);
        assert!(
            browse.axis_of(&roll).is_none(),
            "a right click on a left-plotted field removes it"
        );

        browse.graph(&roll, Axis::Right);
        assert_eq!(browse.axis_of(&roll), Some(Axis::Right));
        browse.toggle(&roll);
        assert!(
            browse.axis_of(&roll).is_none(),
            "a left click on a right-plotted field removes it"
        );
    }

    /// Nothing open, nothing plotted - and no panic.
    #[test]
    fn graphing_with_no_log_open_does_nothing() {
        let mut browse = LogBrowse::new();
        browse.graph(&field("ATT", "Roll"), Axis::Right);
        assert!(browse.plotted().is_empty());
        assert_eq!(browse.right_count(), 0);
    }

    /// The healthy fixture open, with the Data Table box ticked so the grid has its rows.
    fn opened() -> LogBrowse {
        let mut browse = LogBrowse::new();
        browse.open(&fixture());
        browse.toggle_check(Check::DataTable);
        browse
    }

    /// Graph Left and Graph Right put the grid's current cell on their axis.
    #[test]
    fn the_graph_buttons_graph_the_current_cell() {
        let mut browse = opened();
        browse.filter_grid(Some("ATT"));
        browse.select_cell(0, 5);
        assert_eq!(browse.selected_field(), "ATT.Roll");
        browse.graph_selected(Axis::Left);
        assert_eq!(browse.plotted().len(), 1);
        assert_eq!(browse.plotted()[0].label, "ATT.Roll (deg)");
        assert_eq!(browse.right_count(), 0);
        assert_eq!(browse.refused(), None);

        browse.select_cell(0, 7);
        browse.graph_selected(Axis::Right);
        assert_eq!(browse.plotted().len(), 2);
        assert_eq!(browse.right_count(), 1);
        assert_eq!(browse.plotted()[1].label, "ATT.Pitch (deg) R");
    }

    /// A cell of a type with instances graphs that instance, as the field list would.
    #[test]
    fn a_cell_graphs_its_rows_instance() {
        let mut browse = opened();
        browse.filter_grid(Some("VIBE"));
        browse.select_cell(0, 5);
        browse.graph_selected(Axis::Left);
        assert_eq!(browse.plotted().len(), 1);
        assert_eq!(
            browse.plotted()[0]
                .field
                .as_ref()
                .map(ToString::to_string)
                .unwrap_or_default(),
            "VIBE[0].VibeX"
        );
    }

    /// Straight after opening, the first cell is current, and graphing it is refused in words.
    #[test]
    fn the_line_number_column_is_refused_in_the_status_line() {
        let mut browse = opened();
        assert_eq!(browse.selected_field(), "none");
        browse.graph_selected(Axis::Left);
        let words = "Please pick another column, Highlight the cell you wish to graph";
        assert!(browse.plotted().is_empty());
        assert_eq!(browse.refused(), Some(words));
        assert_eq!(browse.status(), Some(words), "the status line says it");
    }

    /// A column that is a field but not one the chart can plot is refused, and a success after a
    /// refusal clears it.
    #[test]
    fn a_field_with_no_time_series_is_refused_and_a_success_clears_it() {
        let mut browse = opened();
        browse.select_cell(0, 3); // FMT.Type
        browse.graph_selected(Axis::Left);
        assert_eq!(
            browse.refused(),
            Some("FMT.Type cannot be plotted against time")
        );
        assert!(browse.plotted().is_empty());

        browse.filter_grid(Some("ATT"));
        browse.select_cell(0, 5);
        browse.graph_selected(Axis::Left);
        assert_eq!(browse.refused(), None);
        assert_eq!(browse.plotted().len(), 1);
    }

    /// Graphing a plotted cell again leaves it on the chart, as `GraphItem` aborts on it.
    #[test]
    fn graphing_a_plotted_cell_again_leaves_it_there() {
        let mut browse = opened();
        browse.filter_grid(Some("ATT"));
        browse.select_cell(0, 5);
        browse.graph_selected(Axis::Left);
        browse.graph_selected(Axis::Right);
        assert_eq!(browse.plotted().len(), 1);
        assert_eq!(
            browse.right_count(),
            0,
            "not moved to the other axis either"
        );
        assert_eq!(browse.status(), Some("ATT.Roll is already on the graph"));
    }

    /// No log open, the buttons refuse as the C# does with an empty grid.
    #[test]
    fn the_graph_buttons_with_no_log_refuse() {
        let mut browse = LogBrowse::new();
        browse.graph_selected(Axis::Right);
        assert_eq!(browse.refused(), Some("Please load a valid file"));
        assert_eq!(browse.grid_rows(), 0);
    }

    /// The grid holds every record of the log, and a filter narrows it.
    #[test]
    fn the_grid_counts_its_rows_and_the_logs_records() {
        let mut browse = opened();
        assert_eq!(browse.grid_rows(), 11_439);
        assert_eq!(browse.grid_records(), 11_439);
        browse.filter_grid(Some("GPS"));
        assert_eq!(browse.grid_rows(), 91);
        assert_eq!(browse.grid_records(), 11_439);
        browse.filter_grid(None);
        assert_eq!(browse.grid_rows(), 11_439);
    }

    /// The fixture's GPS never had a fix: no track, and the mission it logged in its place.
    #[test]
    fn a_log_with_no_fix_maps_its_mission() {
        let mut browse = opened();
        let contents = browse.map_contents();
        assert_eq!(contents.points, 0);
        assert_eq!(contents.source, "");
        assert_eq!(contents.waypoints, 6);
        // `DrawMap` runs when the map is shown, and draws the logged mission.
        assert!(browse.drawn().commands.is_empty());
        browse.toggle_check(Check::Map);
        assert_eq!(browse.drawn().commands.len(), 6);
        assert!(browse.drawn().gps.is_empty());
        assert_eq!(
            browse.map().borrow().path_len(),
            0,
            "the widget draws no track of its own"
        );
    }

    /// A log with a fix maps its first GPS's route, and its EKF's.
    #[test]
    fn a_log_with_a_fix_maps_its_gps_route() {
        let mut browse = LogBrowse::new();
        browse.open(
            &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../testdata/dataflash_damaged.bin"),
        );
        let contents = browse.map_contents();
        assert_eq!(contents.source, "GPS");
        assert_eq!(contents.points, 63);
        browse.toggle_check(Check::Map);
        assert_eq!(browse.drawn().gps.len(), 63);
        assert_eq!(browse.drawn().pos.len(), 119);
    }

    /// Opening another log replaces the map rather than adding to it.
    #[test]
    fn opening_another_log_replaces_its_map() {
        let mut browse = LogBrowse::new();
        browse.open(
            &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../testdata/dataflash_damaged.bin"),
        );
        browse.toggle_check(Check::Map);
        assert_eq!(browse.drawn().gps.len(), 63);
        browse.open(&fixture());
        assert!(browse.drawn().gps.is_empty());
        assert_eq!(browse.drawn().commands.len(), 6);
        assert_eq!(browse.map_contents().points, 0);
    }

    fn damaged() -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/dataflash_damaged.bin")
    }

    /// A log open with a field plotted, the grid shown, on the axis asked for.
    fn plotting(path: &std::path::Path, x_axis: XAxis) -> LogBrowse {
        let mut browse = LogBrowse::new();
        browse.open(path);
        browse.toggle_check(Check::DataTable);
        if x_axis == XAxis::Line {
            browse.toggle_check(Check::Time);
        }
        let roll = browse
            .fields()
            .iter()
            .find(|field| field.message == "ATT" && field.field == "Roll")
            .expect("ATT.Roll")
            .clone();
        browse.toggle(&roll);
        browse
    }

    fn count(browse: &LogBrowse, kind: &str) -> usize {
        browse
            .overlay_counts()
            .iter()
            .find(|(name, _)| *name == kind)
            .map(|(_, count)| *count)
            .unwrap_or_else(|| panic!("no count for {kind}"))
    }

    fn fact(browse: &LogBrowse, key: &str) -> String {
        browse
            .facts()
            .into_iter()
            .find(|(name, _)| name == key)
            .map(|(_, value)| value)
            .unwrap_or_else(|| panic!("no fact {key}"))
    }

    /// The strip's boxes are the C#'s, in its order, with its text, ticked as it ticks them.
    #[test]
    fn the_strip_is_the_c_sharps_boxes_in_its_order() {
        let texts: Vec<&str> = Check::STRIP.iter().map(|check| check.text()).collect();
        assert_eq!(
            texts,
            vec![
                "Map",
                "Time",
                "Data Table",
                "Mode",
                "Errors",
                "MSG",
                "Events"
            ]
        );
        let strip = Strip::default();
        let ticked: Vec<bool> = Check::STRIP.iter().map(|check| strip.get(*check)).collect();
        assert_eq!(ticked, vec![false, true, false, true, true, true, true]);
        let browse = LogBrowse::new();
        assert_eq!(browse.x_axis(), XAxis::Time);
    }

    /// Opening a log labels the chart with what the ticked boxes ask for.
    #[test]
    fn opening_a_log_labels_the_chart() {
        let browse = opened();
        assert_eq!(count(&browse, "modes"), 1);
        assert_eq!(count(&browse, "messages"), 14);
        assert_eq!(count(&browse, "errors"), 0, "the fixture logs no ERR");
        assert_eq!(count(&browse, "events"), 0, "nor any EV");
        assert_eq!(count(&browse, "minutes"), 0, "minutes are for a line axis");

        let mut browse = LogBrowse::new();
        browse.open(&damaged());
        assert_eq!(count(&browse, "events"), 5);
        assert_eq!(count(&browse, "messages"), 144);
    }

    /// Unticking a box takes its labels off, and ticking it puts them back.
    #[test]
    fn a_box_puts_its_labels_on_and_off() {
        let mut browse = opened();
        browse.toggle_check(Check::Mode);
        assert_eq!(count(&browse, "modes"), 0);
        assert_eq!(count(&browse, "messages"), 14, "the others stay");
        browse.toggle_check(Check::Msg);
        assert_eq!(count(&browse, "messages"), 0);
        browse.toggle_check(Check::Mode);
        assert_eq!(count(&browse, "modes"), 1);
        assert_eq!(fact(&browse, "log.check.mode"), "true");
        assert_eq!(fact(&browse, "log.check.msg"), "false");
    }

    /// Clear Graph empties the chart of its labels too, until a curve brings them back.
    #[test]
    fn clearing_the_graph_takes_the_labels_until_a_curve_is_added() {
        let mut browse = plotting(&fixture(), XAxis::Time);
        assert_eq!(count(&browse, "messages"), 14);
        browse.clear();
        assert_eq!(count(&browse, "messages"), 0);
        assert_eq!(count(&browse, "modes"), 0);
        let pitch = browse
            .fields()
            .iter()
            .find(|field| field.message == "ATT" && field.field == "Pitch")
            .expect("ATT.Pitch")
            .clone();
        browse.toggle(&pitch);
        assert_eq!(count(&browse, "messages"), 14);
    }

    /// Unticking Time plots against line numbers, and clears what was plotted against time.
    #[test]
    fn the_time_box_switches_to_line_numbers_and_clears_the_chart() {
        let mut browse = plotting(&fixture(), XAxis::Time);
        assert_eq!(browse.plotted().len(), 1);
        browse.toggle_check(Check::Time);
        assert_eq!(browse.x_axis(), XAxis::Line);
        assert!(
            browse.plotted().is_empty(),
            "chk_time_CheckedChanged clears it"
        );
        assert_eq!(count(&browse, "messages"), 0, "and its labels");
        assert_eq!(fact(&browse, "log.axis"), "line");

        let browse = plotting(&fixture(), XAxis::Line);
        let index = mp_log::index::RecordIndex::build(&std::fs::read(fixture()).expect("fixture"));
        let rows = index.rows_named("ATT");
        let (from, to) = browse.x_range().expect("a range");
        assert!((from - f64::from(rows[0])).abs() < f64::EPSILON);
        assert!((to - f64::from(*rows.last().expect("rows"))).abs() < f64::EPSILON);
        assert_eq!(count(&browse, "modes"), 1);
        assert_eq!(
            count(&browse, "minutes"),
            0,
            "eighteen seconds of GPS has no minute"
        );
    }

    /// A label is at its record's line on a line axis, at its time on a time axis, and each kind
    /// alternates sides on its own.
    #[test]
    fn labels_sit_at_their_records_and_alternate() {
        let browse = plotting(&fixture(), XAxis::Line);
        let placed = labels(&browse);
        let messages: Vec<&Label> = placed.iter().filter(|label| !label.alert).skip(1).collect();
        assert_eq!(placed[0].text, "Auto", "the mode comes first");
        assert_eq!(placed[0].place, Place::AboveBottom);
        #[allow(clippy::cast_precision_loss)]
        let first_message = browse.overlays.messages[0].line as f64;
        assert!((messages[0].x - first_message).abs() < f64::EPSILON);
        assert_eq!(messages[0].place, Place::AboveBottom);
        assert_eq!(messages[1].place, Place::BelowBottom);
        assert_eq!(messages[2].place, Place::AboveBottom);

        let browse = plotting(&fixture(), XAxis::Time);
        let placed = labels(&browse);
        let origin = browse.origin.expect("an origin");
        let time = browse.overlays.messages[0].time_us.expect("a time");
        assert!((placed[1].x - mp_log::plot::seconds_since(origin, time)).abs() < 1e-9);
    }

    /// Errors and events hang from the top edge in red, alternately above and inside it.
    #[test]
    fn errors_and_events_hang_from_the_top_in_red() {
        let mut browse = plotting(&damaged(), XAxis::Time);
        browse.toggle_check(Check::Msg);
        browse.toggle_check(Check::Mode);
        let placed = labels(&browse);
        assert_eq!(placed.len(), 5, "{placed:?}");
        assert!(
            placed
                .iter()
                .all(|label| label.alert && label.text == "EV: EKF_YAW_RESET")
        );
        let places: Vec<Place> = placed.iter().map(|label| label.place).collect();
        assert_eq!(
            places,
            vec![
                Place::AboveTop,
                Place::BelowTop,
                Place::AboveTop,
                Place::BelowTop,
                Place::AboveTop
            ]
        );
    }

    fn mode_at(line: usize, number: i64) -> mp_log::overlay::ModeChange {
        mp_log::overlay::ModeChange {
            mark: Mark {
                line,
                time_us: None,
                text: number.to_string(),
            },
            number,
        }
    }

    /// Every band starts at the left edge, in the colour of the mode before its change; the last
    /// runs the whole width in the last mode's colour.
    #[test]
    fn the_mode_bands_are_drawmodes_bands() {
        let mut browse = plotting(&fixture(), XAxis::Line);
        browse.overlays.modes = vec![mode_at(10, 5), mode_at(20, 3)];
        assert_eq!(
            mode_bands(&browse, 0.0, 30.0),
            vec![
                (0.0, 10.0, PASTEL[0]),
                (0.0, 20.0, PASTEL[5]),
                (0.0, 30.0, PASTEL[3])
            ]
        );
        // A change before the chart's left edge has no band of its own.
        assert_eq!(
            mode_bands(&browse, 15.0, 30.0),
            vec![(15.0, 20.0, PASTEL[5]), (15.0, 30.0, PASTEL[3])]
        );
        // With no changes at all, one band in mode 0's colour.
        browse.overlays.modes.clear();
        assert_eq!(mode_bands(&browse, 0.0, 30.0), vec![(0.0, 30.0, PASTEL[0])]);
        // A mode past the colour table throws building the next band: its label is drawn, the
        // next change's is not, and there is no last band.
        browse.overlays.modes = vec![mode_at(10, 80), mode_at(20, 3)];
        assert_eq!(browse.drawn_modes().len(), 1);
        assert_eq!(mode_bands(&browse, 0.0, 30.0), vec![(0.0, 10.0, PASTEL[0])]);
        assert_eq!(count(&browse, "modes"), 1);
    }

    /// The colour table is the C#'s: fifteen opaque pastels, then fifty-six faint ones.
    #[test]
    fn the_band_colours_are_colourspastal() {
        assert_eq!(PASTEL.len(), 71);
        assert_eq!(PASTEL[0], 0xfe7f_7fff, "ConvertFromRange(1, 0, 0)");
        assert_eq!(PASTEL[6], 0xfebe_7fff, "ConvertFromRange(1, 0.5, 0)");
        assert_eq!(PASTEL[15], 0x5757_ff14, "#5757FF at alpha 20");
        assert_eq!(PASTEL[70], 0xcaff_d814, "#CAFFD8");
        assert_eq!(pastel(70), Some(PASTEL[70]));
        assert_eq!(pastel(71), None);
        assert_eq!(pastel(-1), None);
    }

    /// On a line axis a double click goes to the line under the pointer: cursor, marker, map and
    /// grid all on it.
    #[test]
    fn a_double_click_on_a_line_axis_goes_to_the_line_under_it() {
        let mut browse = plotting(&damaged(), XAxis::Line);
        let (from, to) = browse.x_range().expect("a range");
        browse.double_click(0.5);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let line = (from + 0.5 * (to - from)) as usize;
        assert_eq!(browse.cursor(), Some(line));
        #[allow(clippy::cast_precision_loss)]
        let expected = line as f64;
        assert_eq!(browse.cursor_x(), Some(expected));
        let marker = browse.marker().expect("the damaged log has a fix");
        assert!(
            (marker.latitude() + 27.5134).abs() < 0.01
                && (marker.longitude() - 153.0094).abs() < 0.01,
            "{marker:?}"
        );
        assert!(browse.map_centred_on_marker());
        assert_eq!(browse.grid_current(), Some((line, 1)));
        assert_eq!(fact(&browse, "log.cursor.grid"), "true");
        assert_eq!(fact(&browse, "log.cursor.centred"), "true");
        assert_eq!(fact(&browse, "log.cursor.line"), line.to_string());
        assert!(fact(&browse, "log.cursor.marker").starts_with("-27.51"));
    }

    /// On a time axis it goes to the first position record at or after the time under the
    /// pointer, and the cursor is drawn at that record's time.
    #[test]
    fn a_double_click_on_a_time_axis_goes_to_the_next_position_record() {
        let mut browse = plotting(&damaged(), XAxis::Time);
        let (from, to) = browse.x_range().expect("a range");
        let origin = browse.origin.expect("an origin");
        browse.double_click(0.5);
        let x = 0.5f64.mul_add(to - from, from);
        let LineAtTime::Line(line) = browse.positions.line_at_time(x.mul_add(1e6, origin)) else {
            panic!("the middle of the chart is before the last position record");
        };
        assert_eq!(browse.cursor(), Some(line));
        let record = browse
            .positions
            .records()
            .iter()
            .find(|record| record.line == line)
            .expect("a position record");
        assert!(
            matches!(record.name.as_str(), "GPS" | "GPS2" | "POS"),
            "{record:?}"
        );
        let at = mp_log::plot::seconds_since(origin, record.time_us.expect("a time"));
        assert_eq!(browse.cursor_x(), Some(at));
        assert!(at >= x, "at or after the pointer");
        assert!(browse.marker().is_some());
        assert_eq!(browse.grid_current(), Some((line, 1)));
    }

    /// A log whose GPS never had a fix gets a cursor and no marker.
    #[test]
    fn without_a_fix_there_is_a_cursor_and_no_marker() {
        let mut browse = plotting(&fixture(), XAxis::Line);
        browse.double_click(0.5);
        assert!(browse.cursor().is_some());
        assert_eq!(browse.marker(), None);
        assert!(!browse.map_centred_on_marker());
        assert_eq!(fact(&browse, "log.cursor.marker"), "none");
    }

    /// A negative sample - after every position record on a time axis - clears the marker and
    /// changes nothing else, as the C#'s exception leaves it.
    #[test]
    fn a_sample_before_the_log_clears_the_marker_only() {
        let mut browse = plotting(&damaged(), XAxis::Line);
        browse.double_click(0.5);
        let cursor = browse.cursor();
        let current = browse.grid_current();
        assert!(browse.marker().is_some());
        browse.go_to_sample(-1, true, false, true);
        assert_eq!(browse.marker(), None);
        assert_eq!(browse.cursor(), cursor);
        assert_eq!(browse.grid_current(), current);
    }

    /// Redrawing the labels empties the cursor off the chart; removing a curve does not; the
    /// marker is on the map and stays through both.
    #[test]
    fn redrawing_the_labels_takes_the_cursor_off_but_not_the_marker() {
        let mut browse = plotting(&damaged(), XAxis::Line);
        let pitch = browse
            .fields()
            .iter()
            .find(|field| field.message == "ATT" && field.field == "Pitch")
            .expect("ATT.Pitch")
            .clone();
        browse.toggle(&pitch);
        browse.double_click(0.5);
        assert!(browse.cursor().is_some());
        browse.toggle(&pitch);
        assert!(
            browse.cursor().is_some(),
            "removing a curve raises no zoom event"
        );
        browse.toggle_check(Check::Events);
        assert_eq!(browse.cursor(), None);
        assert!(browse.marker().is_some());

        browse.double_click(0.5);
        browse.toggle_check(Check::Map);
        assert_eq!(browse.cursor(), None, "ticking Map redraws the labels");
        browse.double_click(0.5);
        browse.toggle_check(Check::Map);
        assert!(browse.cursor().is_some(), "unticking it does not");
    }

    /// Until the Data Table box has shown the grid it has no rows: the graph buttons ask for a
    /// valid file, and a double click leaves it alone.
    #[test]
    fn a_grid_never_shown_has_no_rows() {
        let mut browse = LogBrowse::new();
        browse.open(&damaged());
        assert_eq!(browse.grid_rows(), 0);
        browse.graph_selected(Axis::Left);
        assert_eq!(browse.refused(), Some("Please load a valid file"));
        let roll = browse
            .fields()
            .iter()
            .find(|field| field.message == "ATT" && field.field == "Roll")
            .expect("ATT.Roll")
            .clone();
        browse.toggle(&roll);
        browse.double_click(0.5);
        assert!(browse.cursor().is_some());
        assert_eq!(browse.grid_current(), None);

        browse.toggle_check(Check::DataTable);
        assert!(browse.grid_rows() > 0);
        browse.toggle_check(Check::DataTable);
        assert!(browse.grid_rows() > 0, "hidden again, it keeps its rows");
    }

    /// With nothing plotted there is no axis to read a place off.
    #[test]
    fn a_double_click_on_an_empty_chart_does_nothing() {
        let mut browse = opened();
        browse.double_click(0.5);
        assert_eq!(browse.cursor(), None);
        assert_eq!(fact(&browse, "log.cursor.line"), "none");
    }

    /// A pointer is turned into a place across the chart from where it was laid out.
    #[test]
    fn a_pointer_is_a_fraction_of_the_charts_width() {
        let browse = LogBrowse::new();
        assert_eq!(browse.chart_point(100.0, 20.0), None, "not laid out yet");
        browse.chart_bounds.set(Some(Bounds {
            origin: gpui::point(px(40.0), px(10.0)),
            size: gpui::size(px(400.0), px(260.0)),
        }));
        assert_eq!(browse.chart_point(240.0, 140.0), Some((0.5, 0.5)));
        assert_eq!(browse.chart_point(40.0, 10.0), Some((0.0, 0.0)));
        assert_eq!(browse.chart_size(), (400.0, 260.0));
    }

    /// Opening a log takes the last one's marker and cursor away.
    #[test]
    fn opening_a_log_clears_the_cursor_and_the_marker() {
        let mut browse = plotting(&damaged(), XAxis::Time);
        browse.double_click(0.5);
        assert!(browse.marker().is_some());
        browse.open(&fixture());
        assert_eq!(browse.marker(), None);
        assert_eq!(browse.cursor(), None);
        assert!(browse.plotted().is_empty());
        assert_eq!(count(&browse, "modes"), 1, "the new log is labelled");
    }

    /// `tests/gui/log-cursor.gui`, step for step, against the model: what the script expects of
    /// the facts, checked here where it can be run without a window.
    #[test]
    fn the_log_cursor_script_holds_without_a_window() {
        let facts_of = |browse: &LogBrowse| -> BTreeMap<String, String> {
            browse.facts().into_iter().collect()
        };
        let mut browse = LogBrowse::new();
        browse.open(&damaged());
        let facts = facts_of(&browse);
        for (key, value) in [
            ("log.check.map", "false"),
            ("log.check.time", "true"),
            ("log.check.datagrid", "false"),
            ("log.check.mode", "true"),
            ("log.check.errors", "true"),
            ("log.check.msg", "true"),
            ("log.check.events", "true"),
            ("log.axis", "time"),
            ("log.overlays.modes", "1"),
            ("log.overlays.messages", "144"),
            ("log.overlays.events", "5"),
            ("log.overlays.errors", "0"),
            ("log.overlays.minutes", "0"),
            ("log.cursor.line", "none"),
            ("log.cursor.marker", "none"),
        ] {
            assert_eq!(facts.get(key).map(String::as_str), Some(value), "{key}");
        }

        browse.toggle_check(Check::Map);
        browse.toggle_check(Check::DataTable);
        assert!(browse.grid_rows() > 0);
        let roll = browse
            .fields()
            .iter()
            .find(|field| field.message == "ATT" && field.field == "Roll")
            .expect("ATT.Roll")
            .clone();
        browse.toggle(&roll);
        browse.double_click(0.5);
        let facts = facts_of(&browse);
        assert!(
            facts["log.cursor.line"]
                .parse::<usize>()
                .is_ok_and(|line| line > 0)
        );
        assert!(facts["log.cursor.marker"].contains("-27.51"));
        assert!(facts["log.cursor.marker"].contains(",153.00"));
        assert_eq!(facts["log.cursor.centred"], "true");
        assert_eq!(facts["log.cursor.grid"], "true");

        browse.toggle_check(Check::Events);
        let facts = facts_of(&browse);
        assert_eq!(facts["log.check.events"], "false");
        assert_eq!(facts["log.overlays.events"], "0");
        assert_eq!(facts["log.overlays.messages"], "144");
        assert_eq!(facts["log.cursor.line"], "none");
        assert!(facts["log.cursor.marker"].contains("-27.51"));

        browse.toggle_check(Check::Time);
        let facts = facts_of(&browse);
        assert_eq!(facts["log.axis"], "line");
        assert!(browse.plotted().is_empty());
        assert_eq!(facts["log.overlays.messages"], "0");

        browse.toggle(&roll);
        browse.double_click(0.5);
        let facts = facts_of(&browse);
        assert_eq!(facts["log.overlays.messages"], "144");
        assert_eq!(facts["log.overlays.minutes"], "0");
        assert!(
            facts["log.cursor.line"]
                .parse::<usize>()
                .is_ok_and(|line| line > 0)
        );
        assert_eq!(facts["log.cursor.grid"], "true");
        assert!(facts["log.cursor.marker"].contains("-27.51"));

        browse.open(&fixture());
        let facts = facts_of(&browse);
        assert_eq!(facts["log.cursor.marker"], "none");
        assert_eq!(facts["log.cursor.line"], "none");
        assert_eq!(facts["log.overlays.modes"], "1");
        assert_eq!(facts["log.overlays.messages"], "14");
        let roll = browse
            .fields()
            .iter()
            .find(|field| field.message == "ATT" && field.field == "Roll")
            .expect("ATT.Roll")
            .clone();
        browse.toggle(&roll);
        browse.double_click(0.5);
        let facts = facts_of(&browse);
        assert!(
            facts["log.cursor.line"]
                .parse::<usize>()
                .is_ok_and(|line| line > 0)
        );
        assert_eq!(facts["log.cursor.marker"], "none");
        assert_eq!(facts["log.cursor.centred"], "false");
        assert_eq!(facts["log.cursor.grid"], "true");
    }

    /// A mode's name comes from its vehicle's table; a tracker has none here.
    #[test]
    fn a_mode_is_named_from_its_vehicles_table() {
        assert_eq!(
            flight_mode_name(Firmware::Copter, 3).as_deref(),
            Some("Auto")
        );
        assert_eq!(
            flight_mode_name(Firmware::Copter, 5).as_deref(),
            Some("Loiter")
        );
        assert_eq!(flight_mode_name(Firmware::Tracker, 3), None);
    }

    /// `MAV` and `POWR` from ArduPilot's `LogMessagesCopter.xml`, as autotest publishes it:
    /// `MAV.flags` and `POWR.Flags` are bitmasks, and both are in the healthy fixture.
    const LOG_MESSAGES: &str = include_str!("../../../testdata/logbrowse/LogMessagesCopter.xml");

    fn powr_meta() -> metadata::MetaData {
        let mut meta = metadata::MetaData::default();
        meta.parse(LOG_MESSAGES);
        meta
    }

    fn named(browse: &LogBrowse, message: &str, name: &str) -> PlottableField {
        browse
            .fields()
            .iter()
            .find(|field| field.message == message && field.field == name)
            .unwrap_or_else(|| panic!("{message}.{name}"))
            .clone()
    }

    fn maybe_fact(browse: &LogBrowse, key: &str) -> Option<String> {
        browse
            .facts()
            .into_iter()
            .find(|(name, _)| name == key)
            .map(|(_, value)| value)
    }

    /// A field whose metadata is a bitmask gets a child node for each bit, its text the bit's
    /// name and its tooltip the bit's description, in the metadata's order; a field that is not
    /// a bitmask gets none; with no metadata no field does. Each instance's field has the bits,
    /// and expands on its own. The nodes start collapsed and the plus opens them.
    /// `// C#: Log/LogBrowse.cs:682-696, 1237-1248`
    #[test]
    fn a_bitmask_fields_bits_are_its_child_nodes() {
        let mut browse = LogBrowse::new();
        browse.open(&fixture());
        let flags = named(&browse, "POWR", "Flags");
        let vcc = named(&browse, "POWR", "Vcc");
        assert!(browse.bits_of(&flags).is_empty(), "no metadata, no bits");

        browse.add_bit_nodes(Some(&powr_meta()));
        let texts: Vec<&str> = browse
            .bits_of(&flags)
            .iter()
            .map(|b| b.text.as_str())
            .collect();
        assert_eq!(
            texts,
            [
                "BRICK_VALID",
                "SERVO_VALID",
                "USB_CONNECTED",
                "PERIPH_OVERCURRENT",
                "PERIPH_HIPOWER_OVERCURRENT",
                "CHANGED"
            ]
        );
        let bits = browse.bits_of(&flags);
        assert_eq!(bits[0].tip, "main brick power supply valid");
        assert_eq!(bits[2].mask, Some(4));
        assert_eq!(bits[5].mask, Some(32));
        assert!(browse.bits_of(&vcc).is_empty());

        assert_eq!(
            maybe_fact(&browse, "log.field.POWR.Flags.bits").as_deref(),
            Some(
                "BRICK_VALID,SERVO_VALID,USB_CONNECTED,PERIPH_OVERCURRENT,\
                  PERIPH_HIPOWER_OVERCURRENT,CHANGED"
            )
        );
        let mav_flags = named(&browse, "MAV", "flags");
        let channel = |instance| PlottableField {
            instance: Some(instance),
            ..mav_flags.clone()
        };
        let mav: Vec<&str> = browse
            .bits_of(&channel(0))
            .iter()
            .map(|b| b.text.as_str())
            .collect();
        assert_eq!(
            mav,
            ["USING_SIGNING", "ACTIVE", "STREAMING", "PRIVATE", "LOCKED"]
        );
        assert_eq!(
            browse.bits_of(&channel(3)).len(),
            5,
            "every instance's node has them"
        );
        assert_eq!(
            browse.bits_of(&channel(0))[0].tip,
            "",
            "no description, no tooltip"
        );
        assert_eq!(
            maybe_fact(&browse, "log.field.MAV[3].flags.bits").as_deref(),
            Some("USING_SIGNING,ACTIVE,STREAMING,PRIVATE,LOCKED")
        );
        assert_eq!(maybe_fact(&browse, "log.field.POWR.Vcc.bits"), None);
        assert_eq!(
            maybe_fact(&browse, "log.field.POWR.Flags.expanded").as_deref(),
            Some("false")
        );
        browse.toggle_expanded(&flags);
        assert!(browse.is_expanded(&flags));
        assert_eq!(
            maybe_fact(&browse, "log.field.POWR.Flags.expanded").as_deref(),
            Some("true")
        );
        browse.toggle_expanded(&vcc);
        assert!(
            !browse.is_expanded(&vcc),
            "a field with no bits has nothing to expand"
        );
        browse.toggle_expanded(&channel(0));
        assert!(browse.is_expanded(&channel(0)));
        assert!(
            !browse.is_expanded(&channel(3)),
            "an instance expands on its own"
        );
        browse.toggle_expanded(&flags);
        assert!(!browse.is_expanded(&flags));

        browse.add_bit_nodes(None);
        assert!(browse.bits_of(&flags).is_empty());
    }

    /// The real log's `POWR.Flags` is 4 throughout - USB connected, nothing else - so its USB
    /// bit graphs 1 at every record and its brick bit 0, labelled `POWR.Flags.BIT` and the
    /// bit's description; `MAV.flags` is 6 on channel 0 and 2 on channel 3, so STREAMING is 1
    /// on the one and 0 on the other, labelled with the instance and no description.
    /// `// C#: Log/LogBrowse.cs:1237-1248, 1528-1538, 1616-1627`
    #[test]
    fn a_bit_of_the_fixtures_flags_is_graphed_as_its_bit() {
        let mut browse = LogBrowse::new();
        browse.open(&fixture());
        browse.add_bit_nodes(Some(&powr_meta()));
        let flags = named(&browse, "POWR", "Flags");
        browse.graph_bit(&flags, "USB_CONNECTED", Axis::Left);
        browse.graph_bit(&flags, "BRICK_VALID", Axis::Right);
        let plotted = browse.plotted();
        assert_eq!(plotted.len(), 2);
        assert_eq!(
            plotted[0].label,
            "POWR.Flags.USB_CONNECTED USB power is connected"
        );
        assert_eq!(
            plotted[1].label,
            "POWR.Flags.BRICK_VALID main brick power supply valid R"
        );
        let usb: Vec<f64> = plotted[0].series.samples().map(|s| s.value).collect();
        let brick: Vec<f64> = plotted[1].series.samples().map(|s| s.value).collect();
        assert_eq!(usb.len(), 182);
        assert!(usb.iter().all(|value| (*value - 1.0).abs() < 1e-9));
        assert_eq!(brick.len(), 182);
        assert!(brick.iter().all(|value| value.abs() < 1e-9));
        assert_eq!(browse.axis_of_bit(&flags, "BRICK_VALID"), Some(Axis::Right));
        assert_eq!(browse.axis_of(&flags), None, "a bit is not its field");
        assert_eq!(
            maybe_fact(&browse, "log.plotted.labels").as_deref(),
            Some(
                "POWR.Flags.USB_CONNECTED USB power is connected | \
                 POWR.Flags.BRICK_VALID main brick power supply valid R"
            )
        );

        let mav_flags = named(&browse, "MAV", "flags");
        let channel = |instance| PlottableField {
            instance: Some(instance),
            ..mav_flags.clone()
        };
        let (zero, three) = (channel(0), channel(3));
        browse.clear();
        browse.graph_bit(&zero, "STREAMING", Axis::Left);
        browse.graph_bit(&three, "STREAMING", Axis::Left);
        let plotted = browse.plotted();
        assert_eq!(plotted[0].label, "MAV[0].flags.STREAMING");
        assert_eq!(plotted[1].label, "MAV[3].flags.STREAMING");
        let streaming: Vec<f64> = plotted[0].series.samples().map(|s| s.value).collect();
        let idle: Vec<f64> = plotted[1].series.samples().map(|s| s.value).collect();
        assert_eq!((streaming.len(), idle.len()), (18, 18));
        assert!(streaming.iter().all(|value| (*value - 1.0).abs() < 1e-9));
        assert!(idle.iter().all(|value| value.abs() < 1e-9));
    }

    /// A log written for the test: `TEST` is `QI`, `TimeUS` and `Flags`, one record for each
    /// value.
    fn flags_log(values: &[u32]) -> std::path::PathBuf {
        let mut log = Vec::new();
        let mut payload = vec![150, 15];
        for (text, width) in [("TEST", 4), ("QI", 16), ("TimeUS,Flags", 64)] {
            let mut field = text.as_bytes().to_vec();
            field.resize(width, 0);
            payload.extend(field);
        }
        log.extend([0xA3, 0x95, 0x80]);
        log.extend(payload);
        for (index, value) in (1u64..).zip(values) {
            log.extend([0xA3, 0x95, 150]);
            log.extend((index * 1_000_000).to_le_bytes());
            log.extend(value.to_le_bytes());
        }
        let path = std::env::temp_dir().join(format!(
            "mp-gui-logbits-{}-{}.bin",
            std::process::id(),
            values.len()
        ));
        std::fs::write(&path, log).expect("write the log");
        path
    }

    /// Each value goes through the bit's mask, `(value & mask) >> lowest set bit`: a one-bit
    /// mask graphs 0 or 1, a mask of two bits their value; a nameless bit finds no mask and
    /// graphs the field's raw value. A second click takes the bit off, whichever button.
    /// `// C#: Log/LogBrowse.cs:1237-1248, 1528-1538, 3079-3128`
    #[test]
    fn a_bit_graphs_each_value_through_its_mask() {
        let path = flags_log(&[0, 1, 4, 5, 6, 7, 12]);
        let mut browse = LogBrowse::new();
        browse.open(&path);
        let _ = std::fs::remove_file(&path);
        let mut meta = metadata::MetaData::default();
        meta.parse(
            "<loggermessagefile><logformat name=\"TEST\"><description>t</description><fields>\
             <field name=\"Flags\"><description>f</description><bitmask name=\"M\">\
             <bit name=\"FOUR\"><value>4</value></bit>\
             <bit name=\"TWO_BITS\"><description>four and eight</description><value>12</value></bit>\
             <bit><value>1</value></bit>\
             </bitmask></field></fields></logformat></loggermessagefile>",
        );
        browse.add_bit_nodes(Some(&meta));
        let flags = named(&browse, "TEST", "Flags");
        let nameless = &browse.bits_of(&flags)[2];
        assert_eq!(nameless.text, "", "a bit with no name is an empty node");
        assert_eq!(nameless.mask, None, "no name is equal to a null one");
        let values = |browse: &LogBrowse, index: usize| -> Vec<f64> {
            browse.plotted()[index]
                .series
                .samples()
                .map(|s| s.value)
                .collect()
        };

        browse.graph_bit(&flags, "FOUR", Axis::Left);
        assert_eq!(values(&browse, 0), [0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0]);
        assert_eq!(browse.plotted()[0].label, "TEST.Flags.FOUR");
        browse.graph_bit(&flags, "TWO_BITS", Axis::Left);
        assert_eq!(values(&browse, 1), [0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 3.0]);
        assert_eq!(
            browse.plotted()[1].label,
            "TEST.Flags.TWO_BITS four and eight"
        );
        browse.graph_bit(&flags, "", Axis::Left);
        assert_eq!(values(&browse, 2), [0.0, 1.0, 4.0, 5.0, 6.0, 7.0, 12.0]);
        assert_eq!(browse.plotted()[2].label, "TEST.Flags.");

        browse.graph_bit(&flags, "FOUR", Axis::Right);
        assert_eq!(browse.plotted().len(), 2, "a plotted bit comes off");
        assert_eq!(browse.axis_of_bit(&flags, "FOUR"), None);
        browse.graph_bit(&flags, "NOT_A_BIT", Axis::Left);
        assert_eq!(browse.plotted().len(), 2, "no node, nothing graphed");
    }

    /// `GraphItem` gives up when a curve's label starts with the field's name and a space: the
    /// field's own curve does, so with the field plotted none of its bits is graphed - the
    /// status line says why - and the field is still toggled on its own.
    /// `// C#: Log/LogBrowse.cs:1135-1147`
    #[test]
    fn with_its_field_plotted_a_bit_is_not_graphed() {
        let path = flags_log(&[0, 4]);
        let mut browse = LogBrowse::new();
        browse.open(&path);
        let _ = std::fs::remove_file(&path);
        let mut meta = metadata::MetaData::default();
        meta.parse(
            "<loggermessagefile><logformat name=\"TEST\"><description>t</description><fields>\
             <field name=\"Flags\"><description>f</description><bitmask name=\"M\">\
             <bit name=\"FOUR\"><value>4</value></bit></bitmask></field></fields></logformat>\
             </loggermessagefile>",
        );
        browse.add_bit_nodes(Some(&meta));
        let flags = named(&browse, "TEST", "Flags");
        browse.graph_bit(&flags, "FOUR", Axis::Left);
        browse.graph(&flags, Axis::Right);
        assert_eq!(
            browse.plotted().len(),
            2,
            "a bit's curve does not stop its field"
        );
        browse.graph_bit(&flags, "FOUR", Axis::Left);
        assert_eq!(browse.plotted().len(), 1, "the bit comes off");
        browse.graph_bit(&flags, "FOUR", Axis::Left);
        assert_eq!(browse.plotted().len(), 1, "the field's curve stops the bit");
        assert_eq!(browse.status(), Some("TEST.Flags is already on the graph"));
        assert_eq!(browse.axis_of(&flags), Some(Axis::Right));
    }

    /// A bit of a field with a unit: `MSG.Field (unit).BIT` and its description, the bit's
    /// value then the unit's multiplier, as `GraphItem_AddCurve` scales any curve.
    /// `// C#: Log/LogBrowse.cs:1237-1248, 1597-1627`
    #[test]
    fn a_bits_label_follows_its_fields_unit() {
        let node = BitNode {
            text: "B".to_owned(),
            tip: "the b bit".to_owned(),
            mask: Some(2),
        };
        let shown = Plotted::of_bit(
            field("X", "Y"),
            &points(&[2.0, 1.0]),
            &unit("V", 0.5),
            Axis::Right,
            XAxis::Time,
            &node,
        );
        assert_eq!(shown.label, "X.Y (V).B the b bit R");
        assert_eq!(shown.bit.as_deref(), Some("B"));
        let values: Vec<f64> = shown.series.samples().map(|s| s.value).collect();
        assert_eq!(values, [0.5, 0.0]);
    }

    /// A bit's node on hover: the C# looks its text up as a field of the message, which it
    /// is not, so `txt_info` keeps what it had.
    /// `// C#: Log/LogBrowse.cs:3778-3803`
    #[test]
    fn resting_on_a_bit_looks_it_up_as_a_field() {
        let meta = powr_meta();
        let mut browse = LogBrowse::new();
        let flags = field("POWR", "Flags");
        browse.hover_field(&flags, Some(&meta));
        assert_eq!(browse.info(), "System power flags");
        browse.hover_bit(&flags, "USB_CONNECTED", Some(&meta));
        assert_eq!(browse.info(), "System power flags");
        browse.hover_bit(&flags, "Vcc", Some(&meta));
        assert_eq!(browse.info(), "Flight board voltage");
    }
}
