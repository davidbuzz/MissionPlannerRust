//! Reviewing a dataflash log.
//!
//! Ported from `Log/LogBrowse.cs` @ efb0801 (GPL-3.0-or-later). Mission Planner opens it as a
//! separate window from a button on the flight screen; a single-window application makes it a tab.
//! What it holds is the same: a list of the fields the log declares, a chart of the chosen ones on
//! two axes, the map of where the vehicle went beside the chart, and the log's records in a grid
//! under it, whose current cell the Graph Left and Graph Right buttons put on the chart.
//!
//! The extraction is in `mp_log::plot`, the record index in `mp_log::index`, the routes in
//! `mp_log::track` and the reduction in `mp_chart`, all of which have their own tests and no gpui
//! in them. The grid's model is [`grid`]. This is the screen.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

mod grid;

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::rc::Rc;
use std::sync::Arc;

use gpui::{AnyElement, Context, MouseButton, SharedString, div, prelude::*, px, rgb};
use mp_chart::Series;
use mp_log::plot::{FieldUnit, PlottableField, UnitTable};
use mp_tiles::store::TileStore;

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
    /// which is also how the C# finds a curve to remove, by that prefix.
    /// `// C#: Log/LogBrowse.cs:1605-1630, 2787-2792`
    fn new(
        field: PlottableField,
        points: &[mp_log::plot::Point],
        unit: &FieldUnit,
        axis: Axis,
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
            series.push(point.seconds, point.value * unit.multiplier);
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
        self.plotted.clear();
        self.path = Some(path.to_path_buf());
        self.refused = None;
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
    /// `// C#: Log/LogBrowse.cs:3079-3128, 3843-3855`
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
        let plotted = Plotted::new(field.clone(), &points, &unit, axis);
        self.status = Some(format!("{}: {} samples", plotted.label, points.len()));
        self.plotted.push(plotted);
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

    /// Forgets the plot but keeps the log open.
    pub fn clear(&mut self) {
        self.plotted.clear();
    }

    /// The grid, once a log is open.
    #[must_use]
    pub const fn grid(&self) -> Option<&Grid> {
        self.grid.as_ref()
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
            .grid
            .as_ref()
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

    /// Rows the grid holds.
    #[must_use]
    pub fn grid_rows(&self) -> usize {
        self.grid.as_ref().map_or(0, Grid::rows)
    }

    /// Records in the open log.
    #[must_use]
    pub fn grid_records(&self) -> usize {
        self.grid.as_ref().map_or(0, Grid::records)
    }

    /// The field the grid's current cell holds, or `none`.
    #[must_use]
    pub fn selected_field(&self) -> String {
        self.grid
            .as_ref()
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
/// So: the chart with the map beside it, taking half the width each as `CHK_map` splits them;
/// under both, the strip of buttons and then the grid; the field list down the right-hand side.
/// The log's name and open button sit above it all, where a single window has room for them and
/// the C# has a Load A Log button in the strip and a file dialog.
///
/// Not ported from the strip: Remove Item, the preselected graphs, and the Time, Map, Data Table,
/// Show Params, Mode, Errors, MSG and Events check boxes. The map and the grid are always shown,
/// where the C# hides each behind its box - Map, Data Table - and remembers the boxes between
/// sessions.
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
                .children(browse.is_open().then(|| chart_row(browse)))
                .children(browse.is_open().then(|| button_strip(browse, cx)))
                .children(browse.grid().map(|grid| grid_panel(grid, cx))),
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

/// The chart: every series drawn against its own axis's range.
///
/// The axes have no tick marks yet; their ranges are written out above the plot, one entry per
/// axis, and the legend carries each series' own extent - a plot with an auto-scaled axis and no
/// numbers on it says only "this went up and down".
fn plot_panel(browse: &LogBrowse) -> AnyElement {
    let plotted = browse.plotted();
    let from = plotted
        .iter()
        .filter_map(|shown| shown.series.samples().next().map(|sample| sample.at))
        .fold(f64::INFINITY, f64::min);
    let to = plotted
        .iter()
        .filter_map(|shown| shown.series.latest_time())
        .fold(f64::NEG_INFINITY, f64::max);
    let axes = if from.is_finite() && to.is_finite() {
        Axes::over(plotted, from, to)
    } else {
        Axes::default()
    };

    let mut plot = div().relative().h(px(260.0)).w_full();
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
                    .left(gpui::relative(left))
                    .top(gpui::relative(top))
                    .w(px(1.5))
                    .h(gpui::relative(height))
                    .bg(rgb(colour)),
            );
        }
    }

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
            .child(plot)
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(if from.is_finite() {
                        format!("{from:.1}s to {to:.1}s")
                    } else {
                        String::new()
                    }),
            )
            .child(legend),
    )
    .into_any_element()
}

/// `splitContainerZgMap`: the chart and the map, side by side, half the width each.
///
/// `CHK_map_CheckedChanged` sets the splitter to half the width when the map is shown, and the
/// designer puts the chart on the left. `// C#: Log/LogBrowse.cs:2980-2987`
fn chart_row(browse: &LogBrowse) -> AnyElement {
    div()
        .flex()
        .gap_2()
        .child(div().flex_1().min_w(px(0.0)).child(plot_panel(browse)))
        .child(div().flex_1().min_w(px(0.0)).child(map_panel(browse)))
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
            .child(crate::mapview::map_element(map))
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

/// `splitContainerButGrid.Panel1`: the strip of buttons between the chart and the grid.
///
/// In the C#'s order: Graph Left, Graph Right, Clear Graph. The two graph buttons act on the
/// grid's current cell, not on the field list.
/// `// C#: Log/LogBrowse.designer.cs:240-255; Log/LogBrowse.resx:159-160, 186-187, 675-676`
fn button_strip(browse: &LogBrowse, cx: &mut Context<MissionPlanner>) -> AnyElement {
    div()
        .flex()
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
        .into_any_element()
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
        );
        assert_eq!(left.label, "ATT.Roll (deg)");
        let right = Plotted::new(
            field("ATT", "Roll"),
            &points(&[1.0]),
            &unit("deg", 1.0),
            Axis::Right,
        );
        assert_eq!(right.label, "ATT.Roll (deg) R");
        let plain = Plotted::new(
            field("X", "Y"),
            &points(&[1.0]),
            &FieldUnit::default(),
            Axis::Left,
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
            ),
            Plotted::new(
                field("BAT", "Volt"),
                &points(&[11.8, 12.6]),
                &unit("V", 1.0),
                Axis::Left,
            ),
            Plotted::new(
                field("RCIN", "C3"),
                &points(&[1000.0, 2000.0]),
                &unit("PWM", 1.0),
                Axis::Right,
            ),
            Plotted::new(
                field("BARO", "Alt"),
                &points(&[0.0, 50.0]),
                &unit("m", 1.0),
                Axis::Right,
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

    fn opened() -> LogBrowse {
        let mut browse = LogBrowse::new();
        browse.open(&fixture());
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
}
