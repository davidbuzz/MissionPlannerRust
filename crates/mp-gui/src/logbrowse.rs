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
//! The extraction is in `mp_log::plot`, the record index in `mp_log::index`, the routes in
//! `mp_log::track`, the labels and the cursor's lookups in `mp_log::overlay`, and the reduction in
//! `mp_chart`, all of which have their own tests and no gpui in them. The grid's model is
//! [`grid`]. This is the screen.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

mod grid;

use std::cell::{Cell, RefCell};
use std::collections::BTreeMap;
use std::rc::Rc;
use std::sync::Arc;

use gpui::{
    AnyElement, Bounds, Context, MouseButton, Pixels, SharedString, canvas, div, prelude::*, px,
    relative, rgb, rgba,
};
use mp_chart::Series;
use mp_log::overlay::{Firmware, LineAtTime, Mark, Overlays, Positions};
use mp_log::plot::{FieldUnit, PlottableField, UnitTable};
use mp_tiles::store::TileStore;
use mp_units::LatLon;

use crate::MissionPlanner;
use crate::mapview::MapViewport;
use crate::ui::{action, panel, theme};
use grid::{Grid, ROW_HEIGHT, TYPE_COLUMN};

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
/// its text. Not here: Show Params, which loads the log's parameters into the parameter screen,
/// and the preselected graphs' drop-down between it and Mode; neither is ported.
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
    /// The field it came from.
    pub field: PlottableField,
    /// Its samples, scaled into its unit.
    pub series: Series,
    /// Which axis it is drawn against.
    pub axis: Axis,
    /// The unit the left axis groups by; empty when the log declares none.
    pub unit: String,
    /// What the legend calls it: `ATT.Roll (deg)`, with ` R` appended on the right axis.
    pub label: String,
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
    fn new(
        field: PlottableField,
        points: &[mp_log::plot::Point],
        unit: &FieldUnit,
        axis: Axis,
        x_axis: XAxis,
    ) -> Self {
        let mut label = field.to_string();
        if !unit.unit.is_empty() {
            label.push_str(&format!(" ({})", unit.unit));
        }
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
            series.push(x, point.value * unit.multiplier);
        }
        Self {
            field,
            series,
            axis,
            unit: unit.unit.clone(),
            label,
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
        }
    }

    /// Opens a log: what it can plot, where it went, and the index its grid reads rows through.
    ///
    /// The whole file is read, and walked once for each of those. A 1 GB log is not something to
    /// do on the render thread, and D14 budgets two seconds for it with a memory-mapped columnar
    /// parse - this is the straightforward version, and the place that gets replaced when that
    /// lands.
    pub fn open(&mut self, path: &std::path::Path) {
        let data = match std::fs::read(path) {
            Ok(data) => data,
            Err(err) => {
                self.status = Some(format!("could not read {}: {err}", path.display()));
                return;
            }
        };
        self.fields = mp_log::plot::plottable(&data);
        self.units = mp_log::plot::units(&data);
        self.path = Some(path.to_path_buf());
        self.refused = None;
        // `LogBrowse_Load` empties the map's marker; `LoadLog2` clears the chart through
        // `chk_time_CheckedChanged` and labels it with `zg1_ZoomEvent` once the log is read.
        // `// C#: Log/LogBrowse.cs:237-240, 403-408, 479`
        self.marker = None;
        self.clear();
        self.overlays = mp_log::overlay::overlays(&data, flight_mode_name);
        self.positions = Positions::read(&data);
        self.origin = mp_log::plot::time_origin(&data);
        self.grid_filled = self.strip.get(Check::DataTable);
        self.zoom_event();
        self.show_routes(&mp_log::track::routes(&data));
        // The grid keeps the file open and reads its rows back a screenful at a time, as
        // `DFLogBuffer` keeps its stream; the bytes read here are dropped when this returns.
        self.grid = None;
        match std::fs::File::open(path) {
            Ok(file) => {
                self.grid = Some(Grid::new(
                    mp_log::index::RecordIndex::build(&data),
                    Box::new(file),
                    leap_seconds_now(),
                ));
            }
            Err(err) => {
                self.status = Some(format!("could not reopen {}: {err}", path.display()));
                return;
            }
        }
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
        if let Some(position) = self.plotted.iter().position(|shown| shown.field == *field) {
            self.plotted.remove(position);
            return;
        }
        let Some(path) = self.path.clone() else {
            return;
        };
        let Ok(data) = std::fs::read(&path) else {
            self.status = Some(format!("{} could not be re-read", path.display()));
            return;
        };
        let points =
            mp_log::plot::extract_instance(&data, &field.message, field.instance, &field.field);
        let unit = self.units.get(&field.message, &field.field);
        let plotted = Plotted::new(field.clone(), &points, &unit, axis, self.x_axis());
        self.status = Some(format!("{}: {} samples", plotted.label, points.len()));
        self.plotted.push(plotted);
        self.zoom_event();
    }

    /// Which axis a field is plotted on, if it is.
    #[must_use]
    pub fn axis_of(&self, field: &PlottableField) -> Option<Axis> {
        self.plotted
            .iter()
            .find(|shown| shown.field == *field)
            .map(|shown| shown.axis)
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
    }

    /// `zg1_ZoomEvent`: the chart's drawn objects are emptied and the labels the ticked boxes
    /// ask for drawn again - which empties the cursor off it too.
    /// `// C#: Log/LogBrowse.cs:2906-2978`
    fn zoom_event(&mut self) {
        self.labelled = true;
        self.cursor = None;
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
    /// `ZoomOutAll` fits the axis to its curves.
    #[must_use]
    pub fn x_range(&self) -> Option<(f64, f64)> {
        self.plotted
            .iter()
            .filter_map(|shown| shown.series.extent())
            .reduce(|(low, high), (from, to)| (low.min(from), high.max(to)))
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
        facts
    }

    /// How far across the chart's plotting area a window x is, as it was last laid out.
    #[must_use]
    pub fn chart_fraction(&self, window_x: f32) -> Option<f64> {
        let bounds = self.chart_bounds.get()?;
        let width = f32::from(bounds.size.width);
        (width > 0.0).then(|| f64::from((window_x - f32::from(bounds.origin.x)) / width))
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
        self.go_to_sample(sample, true, true);
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
    /// the record it stands on, where the C# evidently means it to be.
    /// `// C#: Log/LogBrowse.cs:3464-3523`
    pub fn go_to_sample(&mut self, sample: i64, move_map: bool, move_grid: bool) {
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

    /// Where the cursor is drawn on the x axis, if it can be.
    #[must_use]
    pub fn cursor_x(&self) -> Option<f64> {
        let line = self.cursor?;
        self.x_of(line, self.positions.time_of(line))
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

    /// Puts a log's routes on the map, replacing whatever the last log put there.
    ///
    /// `DrawMap` draws every route at once, each its own colour, over the mission the vehicle
    /// logged. The map widget here draws one track, so it draws the first GPS's route - the
    /// C#'s blue one - and the `POS` route only for a log whose GPS never had a fix; the second
    /// GPS and the `GPSB` blend are not drawn. The logged mission goes on as a mission, which is
    /// what the widget draws `CMD`'s route and markers as anyway.
    ///
    /// The widget's track is the path a vehicle has flown, so it ends at a vehicle symbol; here
    /// that sits on the route's last point, pointing along the last course the GPS logged.
    /// `// C#: Log/LogBrowse.cs:2191-2540`
    fn show_routes(&mut self, routes: &mp_log::track::Routes) {
        let (points, source) = if routes.gps.is_empty() && !routes.pos.is_empty() {
            (&routes.pos, "POS")
        } else if routes.gps.is_empty() {
            (&routes.gps, "")
        } else {
            (&routes.gps, "GPS")
        };

        // A fresh widget, because a track only ever grows: the last log's would otherwise run
        // straight into this one's.
        let mut map = MapViewport::new(0, 0);
        if let Some(store) = &self.tiles {
            map.set_tiles(Arc::clone(store));
        }
        let mut drawn = 0;
        for point in points {
            if let Ok(position) = mp_units::LatLon::new(point.latitude, point.longitude) {
                let course = mp_units::Bearing(mp_units::Degrees(point.course.unwrap_or(0.0)));
                map.observe(position, course);
                drawn += 1;
            }
        }
        let mission: Vec<mp_mission::MissionItem> = routes
            .commands
            .iter()
            .map(|command| {
                let [param1, param2, param3, param4] = command.params;
                mp_mission::MissionItem {
                    seq: command.seq,
                    current: 0,
                    frame: command.frame.unwrap_or(0),
                    command: command.command,
                    param1,
                    param2,
                    param3,
                    param4,
                    x: command.latitude,
                    y: command.longitude,
                    z: command.altitude,
                    autocontinue: 1,
                }
            })
            .collect();
        map.set_mission(&mission);
        *self.map.borrow_mut() = map;
        self.map_contents = MapContents {
            source,
            points: drawn,
            waypoints: routes.commands.len(),
        };
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
    name_focus: &gpui::FocusHandle,
    focused: bool,
    search: &str,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    div()
        .flex()
        .flex_1()
        .min_h(px(0.0))
        // Never wider than the window. A flex item's automatic minimum width is its content's
        // min-content, and the data grid's rows made that wider than 1600 px, which pushed the
        // field list past the window's right edge where no click could reach it.
        .min_w(px(0.0))
        .gap_2()
        .p_2()
        // Left: everything about the plot, top to bottom.
        .child(
            div()
                .flex()
                .flex_col()
                .flex_1()
                .min_w(px(0.0))
                .gap_2()
                .child(file_panel(browse, name, name_focus, focused, cx))
                .children(browse.is_open().then(|| chart_row(browse, cx)))
                .children(browse.is_open().then(|| button_strip(browse, cx)))
                .children(
                    browse
                        .grid()
                        .filter(|_| browse.strip().get(Check::DataTable))
                        .map(|grid| grid_panel(grid, cx)),
                ),
        )
        // Right: the field tree, which is where LogBrowse puts it.
        .children(browse.is_open().then(|| field_panel(browse, search, cx)))
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
/// A double click anywhere on it is `zg1_MouseDoubleClick`. Moving the pointer over it does
/// nothing else: `zg1_MouseMoveEvent` only debounces, and ZedGraph then shows point values only
/// if its Show Point Values is on, which `LogBrowse` never turns on.
/// `// C#: Log/LogBrowse.cs:3278-3297, 3574-3586;
/// ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.Events.cs:670-701`
fn plot_panel(browse: &LogBrowse, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let plotted = browse.plotted();
    let range = browse.x_range();
    let (from, to) = range.unwrap_or((f64::INFINITY, f64::NEG_INFINITY));
    let axes = if range.is_some() {
        Axes::over(plotted, from, to)
    } else {
        Axes::default()
    };
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
            #[allow(clippy::cast_precision_loss)]
            let left = column.index as f32 / COLUMNS as f32;
            #[allow(clippy::cast_possible_truncation)]
            let top = (1.0 - range.fraction(column.high)) as f32;
            #[allow(clippy::cast_possible_truncation)]
            let bottom = (1.0 - range.fraction(column.low)) as f32;
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

    let chart = div()
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
        // `MouseDoubleClick`, which Windows raises on the second press.
        .on_mouse_down(
            MouseButton::Left,
            cx.listener(|this, event: &gpui::MouseDownEvent, _window, cx| {
                if event.click_count != 2 {
                    return;
                }
                if let Some(fraction) = this.log_browse.chart_fraction(f32::from(event.position.x))
                {
                    this.log_browse.double_click(fraction);
                    cx.notify();
                }
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
/// `MousePositionWithoutCenter`. The C# labels its routes in their colours in the top-left
/// corner; the one route drawn here is labelled the same way, in the colour it is drawn.
/// `// C#: Log/LogBrowse.designer.cs:153-232; Log/LogBrowse.cs:3735-3771`
fn map_panel(browse: &LogBrowse) -> AnyElement {
    let map = browse.map();
    let contents = browse.map_contents();
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
            // `markeroverlay`, over the map and painted after it.
            .child(marker_element(Rc::clone(&map), browse.marker()))
            // `label1`, "GPS", in the corner, in the colour of its route - which here is the
            // colour the map widget strokes any track in.
            .children((!contents.source.is_empty()).then(|| {
                div()
                    .absolute()
                    .top_1()
                    .left_1()
                    .px_1()
                    .rounded_sm()
                    .bg(rgb(theme::PANEL))
                    .text_xs()
                    .text_color(rgb(theme::OK))
                    .child(contents.source)
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
/// Table, Mode, Errors, MSG, Events. The two graph buttons act on the grid's current cell, not on
/// the field list.
/// `// C#: Log/LogBrowse.designer.cs:240-332; Log/LogBrowse.resx:159-160, 186-187, 675-676`
fn button_strip(browse: &LogBrowse, cx: &mut Context<MissionPlanner>) -> AnyElement {
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
            Check::STRIP
                .into_iter()
                .map(|check| check_box(check, browse.strip().get(check), cx)),
        )
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
            cx.notify();
        }))
        .into_any_element()
}

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
/// grid as it is filtered, so a test can click one by name.
/// `// C#: Log/LogBrowse.designer.cs:349-363; Log/LogBrowse.cs:2820-2896`
fn grid_panel(grid: &Grid, cx: &mut Context<MissionPlanner>) -> AnyElement {
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
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        this.log_browse.select_cell(at.0, at.1);
                        cx.notify();
                    })),
            );
        }
        body = body.child(line);
    }

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
            .flex()
            .flex_col()
            .gap_1()
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
fn field_panel(browse: &LogBrowse, search: &str, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let needle = search.trim().to_uppercase();
    let matching: Vec<&PlottableField> = browse
        .fields()
        .iter()
        .filter(|field| needle.is_empty() || field.to_string().to_uppercase().contains(&needle))
        .collect();
    let total = matching.len();

    let mut list = div().flex().flex_wrap().gap_1();
    for field in matching.into_iter().take(SHOWN_FIELDS) {
        let axis = browse.axis_of(field);
        let shown = axis.is_some();
        let label = field.to_string();
        let text = match axis {
            Some(Axis::Right) => format!("{label} R"),
            _ => label.clone(),
        };
        let chosen_left = field.clone();
        let chosen_right = field.clone();
        list = list.child(
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
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.log_browse.toggle(&chosen_left);
                    cx.notify();
                }))
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
                ),
        );
    }

    panel(
        "fields",
        div()
            .id("log-fields")
            .flex()
            .flex_col()
            .w(px(300.0))
            .flex_shrink_0()
            .gap_2()
            .overflow_y_scroll()
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(if total > SHOWN_FIELDS {
                        format!("{total} fields, showing {SHOWN_FIELDS} - type in the box above to narrow")
                    } else {
                        format!("{total} fields")
                    }),
            )
            .child(list),
    )
    // The list scrolls inside the window rather than making the screen as tall as itself: a
    // flex item's automatic minimum height is its content's, and two hundred chips are taller
    // than any window, which stretched the whole row and put the scroll bar out of reach.
    .min_h(px(0.0))
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
        assert_eq!(browse.plotted()[0].field.to_string(), "VIBE[0].VibeX");
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
        let browse = opened();
        let contents = browse.map_contents();
        assert_eq!(contents.points, 0);
        assert_eq!(contents.source, "");
        assert_eq!(contents.waypoints, 6);
        assert_eq!(browse.map().borrow().mission_len(), 6);
        assert_eq!(browse.map().borrow().path_len(), 0);
    }

    /// A log with a fix maps its first GPS's route.
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
        // The widget keeps a point only when the vehicle has moved; on a bench it barely does.
        let kept = browse.map().borrow().path_len();
        assert!((1..=63).contains(&kept), "{kept}");
        assert!(browse.map().borrow().has_fix());
    }

    /// Opening another log replaces the map rather than adding to it.
    #[test]
    fn opening_another_log_replaces_its_map() {
        let mut browse = LogBrowse::new();
        browse.open(
            &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../testdata/dataflash_damaged.bin"),
        );
        assert!(browse.map().borrow().path_len() > 0);
        browse.open(&fixture());
        assert_eq!(browse.map().borrow().path_len(), 0);
        assert!(!browse.map().borrow().has_fix());
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
        browse.go_to_sample(-1, true, true);
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
        assert_eq!(browse.chart_fraction(100.0), None, "not laid out yet");
        browse.chart_bounds.set(Some(Bounds {
            origin: gpui::point(px(40.0), px(10.0)),
            size: gpui::size(px(400.0), px(260.0)),
        }));
        assert_eq!(browse.chart_fraction(240.0), Some(0.5));
        assert_eq!(browse.chart_fraction(40.0), Some(0.0));
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
}
