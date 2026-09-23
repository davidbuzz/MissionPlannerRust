//! Reviewing a dataflash log.
//!
//! Ported from `Log/LogBrowse.cs` @ efb0801 (GPL-3.0-or-later). Mission Planner opens it as a
//! separate window from a button on the flight screen; a single-window application makes it a tab.
//! What it holds is the same: a list of the fields the log declares, and a chart of the chosen
//! ones.
//!
//! The extraction is in `mp_log::plot` and the reduction in `mp_chart`, both of which have their
//! own tests and no gpui in them. This is the screen.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_chart::Series;
use mp_log::plot::PlottableField;

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

/// What the log screen keeps between frames.
#[derive(Debug, Default)]
pub struct LogBrowse {
    /// The file that was opened, if one was.
    path: Option<std::path::PathBuf>,
    /// Everything that log declares as plottable.
    fields: Vec<PlottableField>,
    /// The series being plotted, with the field each came from.
    plotted: Vec<(PlottableField, Series)>,
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

    /// Adds or removes a field from the plot.
    pub fn toggle(&mut self, field: &PlottableField) {
        if let Some(position) = self.plotted.iter().position(|(shown, _)| shown == field) {
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
        let mut series = Series::new(field.to_string(), points.len().max(1));
        for point in &points {
            series.push(point.seconds, point.value);
        }
        self.plotted.push((field.clone(), series));
        self.status = Some(format!("{field}: {} samples", points.len()));
    }

    /// Whether a field is being plotted.
    #[must_use]
    pub fn shows(&self, field: &PlottableField) -> bool {
        self.plotted.iter().any(|(shown, _)| shown == field)
    }

    /// The fields the open log declares.
    #[must_use]
    pub fn fields(&self) -> &[PlottableField] {
        &self.fields
    }

    /// The series being plotted.
    #[must_use]
    pub fn plotted(&self) -> &[(PlottableField, Series)] {
        &self.plotted
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

/// The chart.
///
/// One Y axis, auto-ranged over every series shown. `LogBrowse` has two - `BUT_Graphit` puts a
/// field on the left axis and `BUT_Graphit_R` on the right - which matters as soon as two fields
/// have different magnitudes: roll in degrees beside a battery voltage flattens both. Not built
/// yet, and named here so the single axis is not mistaken for a decision.
/// `// C#: Log/LogBrowse.designer.cs:245,251`
fn plot_panel(browse: &LogBrowse, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let borrowed: Vec<&Series> = browse.plotted().iter().map(|(_, series)| series).collect();
    let from = borrowed
        .iter()
        .filter_map(|series| series.samples().next().map(|sample| sample.at))
        .fold(f64::INFINITY, f64::min);
    let to = borrowed
        .iter()
        .filter_map(|series| series.latest_time())
        .fold(f64::NEG_INFINITY, f64::max);
    let range = if from.is_finite() && to.is_finite() {
        mp_chart::auto_range(&borrowed, from, to)
    } else {
        None
    };

    let mut plot = div().relative().h(px(260.0)).w_full();
    match range {
        Some(range) => {
            for (index, (_, series)) in browse.plotted().iter().enumerate() {
                let colour = TRACE_COLOURS
                    .get(index % TRACE_COLOURS.len())
                    .copied()
                    .unwrap_or(theme::TEXT);
                for column in mp_chart::reduce(series, from, to, COLUMNS) {
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
        }
        None => {
            plot = plot.child(
                div()
                    .absolute()
                    .top(px(110.0))
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child("choose a field below to plot it"),
            );
        }
    }

    // The legend carries the range, because a plot with an auto-scaled axis and no numbers on it
    // says only "this went up and down".
    let mut legend = div().flex().flex_wrap().gap_3();
    for (index, (field, series)) in browse.plotted().iter().enumerate() {
        let colour = TRACE_COLOURS
            .get(index % TRACE_COLOURS.len())
            .copied()
            .unwrap_or(theme::TEXT);
        let low = series
            .samples()
            .map(|sample| sample.value)
            .fold(f64::INFINITY, f64::min);
        let high = series
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
                        .child(field.to_string()),
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
        let shown = browse.shows(field);
        let label = field.to_string();
        let chosen = field.clone();
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
                .child(label)
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.log_browse.toggle(&chosen);
                    cx.notify();
                })),
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
