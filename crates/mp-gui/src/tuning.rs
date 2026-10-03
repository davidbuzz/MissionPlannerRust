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

//! The live tuning graph.
//!
//! Watching a value against time is how tuning is done, and how a pilot answers "is that
//! oscillation in the vehicle or in my head". Mission Planner puts one on the flight screen; this
//! is the same thing.
//!
//! The arithmetic is in `mp_chart`, which has no gpui in it and its own tests. This module is the
//! drawing: each series the line through it, as the C#'s `AddCurve(..., SymbolType.None)` draws
//! it (`crate::plotline`). It was first a min/max bar a column, cheaper to lay out than a path is
//! to tessellate, but ten seconds at 10 Hz is a hundred samples over 180 columns, and a bar each
//! is a row of dots. The line goes through the same min/max reduction, so it cannot hide a
//! one-sample spike either.
//! `// C#: GCSViews/FlightData.cs:1940-2020`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_chart::{Range, Series, auto_range, window};

use crate::MissionPlanner;
use crate::telemetry::TelemetryView;
use crate::ui::{action, panel, theme};

/// How many samples a series keeps.
///
/// Ten seconds are shown and telemetry arrives at around 10 Hz, so a hundred would do - but a
/// vehicle streaming attitude at 50 Hz fills that in two seconds and the trace becomes a stub
/// whenever the link speeds up. Six hundred is ten seconds of the fastest stream anybody runs.
const SAMPLES: usize = 600;

/// How many columns a series is reduced to across the plot.
const COLUMNS: usize = 180;

/// A field that can be plotted, and how to read it from a snapshot.
type Reader = fn(&mp_vehicle::VehicleState) -> f64;

/// The fields offered, in the order Mission Planner's default selection lists them.
///
/// Deliberately short. The C# offers every property on `CurrentState` through a double-click
/// dialog of about two hundred entries, which is a list nobody reads; these are the ones a tune is
/// actually watched on.
const FIELDS: &[(&str, Reader)] = &[
    ("roll", |state| state.attitude.roll.0.to_degrees()),
    ("pitch", |state| state.attitude.pitch.0.to_degrees()),
    ("yaw", |state| state.attitude.yaw.0.to_degrees()),
    ("roll rate", |state| {
        f64::from(state.attitude.roll_rate).to_degrees()
    }),
    ("pitch rate", |state| {
        f64::from(state.attitude.pitch_rate).to_degrees()
    }),
    ("altitude", |state| state.altitude_relative.0),
    ("climb", |state| state.climb_rate.0),
    ("ground speed", |state| state.ground_speed.0),
    ("throttle", |state| f64::from(state.throttle_percent)),
    ("battery", |state| f64::from(state.battery.voltage)),
    ("vibration z", |state| f64::from(state.vibration.z)),
];

/// Everything the tuning graph keeps between frames.
#[derive(Debug)]
pub struct Tuning {
    /// One series per chosen field, in the order they were chosen.
    series: Vec<Series>,
    /// Which fields are shown, by index into [`FIELDS`].
    chosen: Vec<usize>,
    /// When the graph started, so the x axis is seconds rather than a wall clock.
    started: std::time::Instant,
    /// The snapshot count last sampled, so a stalled link does not draw a flat line that looks
    /// like a real measurement of zero.
    last_seen: u64,
    /// Whether the graph is showing.
    ///
    /// Off until asked for, as in the C#: `splitContainer1.Panel1` holds the chart and starts
    /// collapsed, and `CB_tuning` is what uncollapses it.
    /// `// C#: GCSViews/FlightData.cs:1902-1918`
    visible: bool,
}

impl Default for Tuning {
    fn default() -> Self {
        Self::new()
    }
}

impl Tuning {
    /// Roll and pitch, which is what a tune is usually watched on.
    #[must_use]
    pub fn new() -> Self {
        let mut tuning = Self {
            series: Vec::new(),
            chosen: Vec::new(),
            started: std::time::Instant::now(),
            last_seen: 0,
            visible: false,
        };
        tuning.toggle(0);
        tuning.toggle(1);
        tuning
    }

    /// Adds or removes a field.
    pub fn toggle(&mut self, field: usize) {
        let Some((name, _)) = FIELDS.get(field) else {
            return;
        };
        if let Some(position) = self.chosen.iter().position(|chosen| *chosen == field) {
            self.chosen.remove(position);
            self.series.remove(position);
        } else {
            self.chosen.push(field);
            self.series.push(Series::new(*name, SAMPLES));
        }
    }

    /// Whether a field is being shown.
    #[must_use]
    pub fn shows(&self, field: usize) -> bool {
        self.chosen.contains(&field)
    }

    /// Takes a sample from a snapshot, if it is a new one.
    ///
    /// Only on a new snapshot. Sampling the same state repeatedly draws a horizontal line that
    /// looks exactly like a measurement of a steady value, when what it means is that nothing has
    /// arrived - and on a tuning graph those two readings lead to opposite conclusions.
    pub fn sample(&mut self, view: &TelemetryView) {
        // Nothing is sampled while the graph is hidden. The C# stops `ZedGraphTimer` when the
        // panel collapses, and doing the same means a session that never opens the graph does no
        // work for it at all.
        if !self.visible {
            return;
        }
        let Some(state) = view.state.as_ref() else {
            return;
        };
        if state.messages_applied == self.last_seen {
            return;
        }
        self.last_seen = state.messages_applied;
        let at = self.started.elapsed().as_secs_f64();
        for (slot, field) in self.chosen.iter().enumerate() {
            if let Some((_, read)) = FIELDS.get(*field)
                && let Some(series) = self.series.get_mut(slot)
            {
                series.push(at, read(state));
            }
        }
    }

    /// The series being shown.
    pub fn series(&self) -> &[Series] {
        &self.series
    }

    /// Whether the graph is showing.
    #[must_use]
    pub const fn is_visible(&self) -> bool {
        self.visible
    }

    /// Shows or hides the graph.
    ///
    /// Showing restarts the clock, so the ten seconds on screen are the ten seconds since it was
    /// asked for rather than a window into a buffer filled while nobody was looking - which is
    /// what the C# does by only running `ZedGraphTimer` while the panel is uncollapsed.
    pub fn set_visible(&mut self, visible: bool) {
        if visible && !self.visible {
            self.clear();
        }
        self.visible = visible;
    }

    /// Forgets everything drawn so far.
    pub fn clear(&mut self) {
        self.started = std::time::Instant::now();
        for series in &mut self.series {
            series.clear();
        }
    }
}

/// The colour each series gets, by position.
const TRACE_COLOURS: &[u32] = &[
    theme::ACCENT,
    theme::OK,
    theme::WARN,
    theme::ALERT,
    0x9f_7fff,
    0x7f_ffcc,
];

/// Each series the line through it over `from..to`, in its colour.
fn lines(series: &[Series], range: Range, from: f64, to: f64) -> Vec<crate::plotline::Line> {
    series
        .iter()
        .enumerate()
        .map(|(index, series)| crate::plotline::Line {
            points: crate::plotline::curve(series, range, from, to, COLUMNS),
            colour: rgb(TRACE_COLOURS
                .get(index % TRACE_COLOURS.len())
                .copied()
                .unwrap_or(theme::TEXT))
            .into(),
            diamonds: false,
        })
        .collect()
}

/// The tuning panel.
pub fn panel_for(tuning: &Tuning, cx: &mut Context<MissionPlanner>) -> AnyElement {
    // Hidden until asked for, so a screen that already scrolls does not carry a plot nobody is
    // watching. `// C#: GCSViews/FlightData.cs:1902-1918`
    if !tuning.is_visible() {
        return panel(
            "tuning",
            div().child(action(
                "tuning-show",
                "show tuning graph",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.tuning.set_visible(true);
                    cx.notify();
                }),
            )),
        )
        .into_any_element();
    }

    let latest = tuning
        .series()
        .iter()
        .filter_map(mp_chart::Series::latest_time)
        .fold(0.0_f64, f64::max);
    let (from, to) = window(latest);
    let borrowed: Vec<&Series> = tuning.series().iter().collect();
    let range = auto_range(&borrowed, from, to);

    let mut plot = div().relative().h(px(120.0)).w_full();
    if let Some(range) = range {
        plot = plot.child(crate::plotline::element(lines(
            tuning.series(),
            range,
            from,
            to,
        )));
    } else {
        plot = plot.child(
            div()
                .absolute()
                .top(px(50.0))
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child("waiting for telemetry"),
        );
    }

    // The field buttons, which are how a different value gets watched.
    let mut chooser = div().flex().flex_wrap().gap_1();
    for (index, (name, _)) in FIELDS.iter().enumerate() {
        let shown = tuning.shows(index);
        chooser = chooser.child(
            crate::probe::measured(format!("tuning-{name}"), div())
                .id(gpui::SharedString::from(format!("tuning-{name}")))
                .px_2()
                .py(px(1.0))
                .rounded_sm()
                .border_1()
                .border_color(rgb(if shown { theme::ACCENT } else { theme::BORDER }))
                .text_xs()
                .text_color(rgb(if shown { theme::ACCENT } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child((*name).to_owned())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.tuning.toggle(index);
                    cx.notify();
                })),
        );
    }

    // The current value of each trace, in its own colour, because reading a number off a plot is
    // guesswork and the number is the thing being tuned to.
    let mut legend = div().flex().flex_wrap().gap_3();
    for (index, series) in tuning.series().iter().enumerate() {
        let colour = TRACE_COLOURS
            .get(index % TRACE_COLOURS.len())
            .copied()
            .unwrap_or(theme::TEXT);
        legend = legend.child(
            div()
                .flex()
                .items_center()
                .gap_1()
                .child(div().size_2().rounded_full().bg(rgb(colour)))
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child(series.name.clone()),
                )
                .child(
                    div().text_xs().text_color(rgb(colour)).child(
                        series
                            .latest()
                            .map_or_else(|| "--".to_owned(), |value| format!("{value:.2}")),
                    ),
                ),
        );
    }

    panel(
        "tuning",
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
                            .child(format!("{from:.0}s to {to:.0}s")),
                    )
                    .child(
                        div()
                            .flex()
                            .gap_2()
                            .child(action(
                                "tuning-clear",
                                "clear",
                                theme::ACCENT,
                                true,
                                cx.listener(|this, _event: &(), _window, cx| {
                                    this.tuning.clear();
                                    cx.notify();
                                }),
                            ))
                            .child(action(
                                "tuning-hide",
                                "hide",
                                theme::DIM,
                                true,
                                cx.listener(|this, _event: &(), _window, cx| {
                                    this.tuning.set_visible(false);
                                    cx.notify();
                                }),
                            )),
                    ),
            )
            .child(legend)
            .child(chooser),
    )
    .into_any_element()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Off until asked for, as the C# panel starts collapsed.
    #[test]
    fn the_graph_is_hidden_until_it_is_asked_for() {
        let mut tuning = Tuning::new();
        assert!(!tuning.is_visible());
        tuning.set_visible(true);
        assert!(tuning.is_visible());
        tuning.set_visible(false);
        assert!(!tuning.is_visible());
    }

    /// A hidden graph does no work, as the C# stops its timer when the panel collapses.
    #[test]
    fn nothing_is_sampled_while_it_is_hidden() {
        let mut tuning = Tuning::new();
        let view = crate::telemetry::TelemetryView::disconnected(String::new());
        tuning.sample(&view);
        assert!(tuning.series().iter().all(mp_chart::Series::is_empty));
    }

    /// The default is the pair a tune is usually watched on.
    #[test]
    fn it_starts_showing_roll_and_pitch() {
        let tuning = Tuning::new();
        assert_eq!(tuning.series().len(), 2);
        assert!(tuning.shows(0));
        assert!(tuning.shows(1));
        assert!(!tuning.shows(2));
    }

    /// Choosing and unchoosing keeps the series and the choice in step.
    #[test]
    fn toggling_a_field_adds_and_removes_its_series() {
        let mut tuning = Tuning::new();
        tuning.toggle(5);
        assert_eq!(tuning.series().len(), 3);
        assert_eq!(tuning.series()[2].name, "altitude");

        tuning.toggle(0);
        assert_eq!(tuning.series().len(), 2);
        assert!(!tuning.shows(0));
        // And the remaining series are still the ones that were chosen, in order.
        let names: Vec<&str> = tuning.series().iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, vec!["pitch", "altitude"]);
    }

    /// A field that does not exist must not panic or desynchronise the two lists.
    #[test]
    fn a_field_outside_the_list_is_ignored() {
        let mut tuning = Tuning::new();
        tuning.toggle(999);
        assert_eq!(tuning.series().len(), 2);
        assert_eq!(tuning.series().len(), tuning.chosen.len());
    }

    /// Ten seconds of 10 Hz telemetry is a hundred samples over 180 columns: one line through
    /// all of them, left to right, not a bar a column - which drew a row of dots.
    #[test]
    fn a_series_is_one_line_through_its_samples() {
        let mut tuning = Tuning::new();
        for step in 0..100 {
            let at = f64::from(step) / 10.0;
            tuning.series[0].push(at, at.sin());
            tuning.series[1].push(at, at.cos());
        }
        let (from, to) = window(9.9);
        let borrowed: Vec<&Series> = tuning.series().iter().collect();
        let range = auto_range(&borrowed, from, to).expect("a range");
        let drawn = lines(tuning.series(), range, from, to);
        assert_eq!(drawn.len(), 2);
        for line in &drawn {
            assert_eq!(line.points.len(), 100, "a point a sample");
            assert!(line.points.windows(2).all(|pair| pair[0].0 < pair[1].0));
            assert!(
                line.points
                    .iter()
                    .all(|(_, y)| (0.0..=1.0).contains(y)),
                "the range holds the line"
            );
        }
        assert_ne!(drawn[0].colour, drawn[1].colour);
    }

    /// Every field has a colour, including past the end of the table.
    #[test]
    fn the_colour_table_covers_every_field() {
        for index in 0..FIELDS.len() {
            let colour = TRACE_COLOURS
                .get(index % TRACE_COLOURS.len())
                .copied()
                .unwrap_or(theme::TEXT);
            assert_ne!(colour, 0, "field {index} has no colour");
        }
    }
}
