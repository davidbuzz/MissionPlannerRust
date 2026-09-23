//! Reviewing a dataflash log.
//!
//! Ported from `Log/LogBrowse.cs` @ efb0801 (GPL-3.0-or-later). Mission Planner opens it as a
//! separate window from a button on the flight screen; a single-window application makes it a tab.
//! What it holds is the same: a list of the fields the log declares, and a chart of the chosen
//! ones on two axes.
//!
//! The extraction is in `mp_log::plot` and the reduction in `mp_chart`, both of which have their
//! own tests and no gpui in them. This is the screen.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeMap;

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_chart::Series;
use mp_log::plot::{FieldUnit, PlottableField, UnitTable};

use crate::MissionPlanner;
use crate::ui::{action, panel, theme};

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

/// What the log screen keeps between frames.
#[derive(Debug, Default)]
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
}

impl LogBrowse {
    /// Nothing open.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Opens a log and reads what it can plot.
    ///
    /// The whole file is read and walked once. A 1 GB log is not something to do on the render
    /// thread, and D14 budgets two seconds for it with a memory-mapped columnar parse - this is
    /// the straightforward version, and the place that gets replaced when that lands.
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
/// So: chart on the left with its controls under it, field list down the right-hand side. The map
/// beside the chart and the raw data grid under it are not built yet and are named here so the
/// shape is not mistaken for finished. `// C#: Log/LogBrowse.designer.cs:136-390`
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
                .children(browse.is_open().then(|| plot_panel(browse, cx))),
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
                                crate::textfield::KeyOutcome::Submitted => this.open_log(),
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
fn plot_panel(browse: &LogBrowse, cx: &mut Context<MissionPlanner>) -> AnyElement {
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
                    .flex()
                    .items_center()
                    .justify_between()
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
                    .child(action(
                        "log-clear",
                        "clear",
                        theme::ACCENT,
                        !browse.plotted().is_empty(),
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.log_browse.clear();
                            cx.notify();
                        }),
                    )),
            )
            .child(legend),
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
}
