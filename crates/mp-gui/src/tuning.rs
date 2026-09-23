//! The live tuning graph.
//!
//! Watching a value against time is how tuning is done, and how a pilot answers "is that
//! oscillation in the vehicle or in my head". Mission Planner puts one on the flight screen; this
//! is the same thing.
//!
//! The arithmetic is in `mp_chart`, which has no gpui in it and its own tests. This module is the
//! drawing, and it draws min/max bars rather than a polyline - gpui has no line primitive that
//! does not go through CPU tessellation, and a bar per pixel column is both cheaper and, as
//! `mp_chart` argues, incapable of hiding a one-sample spike.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_chart::{Series, auto_range, reduce, window};

use crate::MissionPlanner;
use crate::telemetry::TelemetryView;
use crate::ui::{action, panel, theme};

/// How many samples a series keeps.
///
/// Ten seconds are shown and telemetry arrives at around 10 Hz, so a hundred would do - but a
/// vehicle streaming attitude at 50 Hz fills that in two seconds and the trace becomes a stub
/// whenever the link speeds up. Six hundred is ten seconds of the fastest stream anybody runs.
const SAMPLES: usize = 600;

/// How wide the plot is, in pixel columns.
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

/// The tuning panel.
pub fn panel_for(tuning: &Tuning, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let latest = tuning
        .series()
        .iter()
        .filter_map(mp_chart::Series::latest_time)
        .fold(0.0_f64, f64::max);
    let (from, to) = window(latest);
    let borrowed: Vec<&Series> = tuning.series().iter().collect();
    let range = auto_range(&borrowed, from, to);

    // One absolutely positioned bar per column per series. At 180 columns and two series that is
    // 360 elements, which gpui lays out without noticing; a polyline would go through lyon's CPU
    // tessellator every frame for the same picture.
    let mut plot = div().relative().h(px(120.0)).w_full();
    if let Some(range) = range {
        for (index, series) in tuning.series().iter().enumerate() {
            let colour = TRACE_COLOURS
                .get(index % TRACE_COLOURS.len())
                .copied()
                .unwrap_or(theme::TEXT);
            for column in reduce(series, from, to, COLUMNS) {
                // Fractions of the plot box, computed in f64 and narrowed once. `fraction` is
                // clamped to 0..1, so every value here is inside a range f32 represents exactly
                // enough for a pixel position.
                #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
                let left = column.index as f32 / COLUMNS as f32;
                #[allow(clippy::cast_possible_truncation)]
                let top = (1.0 - range.fraction(column.high)) as f32;
                #[allow(clippy::cast_possible_truncation)]
                let bottom = (1.0 - range.fraction(column.low)) as f32;
                // A bar is given a floor, or a flat stretch of the trace vanishes entirely.
                let height = (bottom - top).max(0.009);
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
                    .child(action(
                        "tuning-clear",
                        "clear",
                        theme::ACCENT,
                        true,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.tuning.clear();
                            cx.notify();
                        }),
                    )),
            )
            .child(legend)
            .child(chooser),
    )
    .into_any_element()
}

#[cfg(test)]
mod tests {
    use super::*;

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
