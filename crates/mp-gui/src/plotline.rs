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

//! A chart's curves as ZedGraph draws a `LineItem`: a line joining each point to the next, clipped
//! to the plot, with a symbol at each point when the curve has one.
//!
//! Every graph Mission Planner draws with ZedGraph - the log browser's (`AddCurve(...,
//! SymbolType.None)`), the flight screen's tuning graph and the FFT screen's - is a line. Each
//! was first drawn here as a bar a pixel column, which a curve with fewer samples than columns
//! turns into "a left-to-right string of dots" with white space between (the owner's report of
//! Logs > PLOT, 2026-09-26). The line goes through `mp_chart::trace`, so it still costs the plot's
//! width however long the series.
//! `// C#: ExtLibs/ZedGraph/ZedGraph/Line.cs:641-830; ExtLibs/ZedGraph/ZedGraph/Symbol.cs:95-145`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{ContentMask, Hsla, IntoElement, PathBuilder, Styled, Window, canvas, point, px};
use mp_chart::{Range, Series};

/// `LineBase.Default.Width`: the pen a curve is drawn with unless it says otherwise.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/LineBase.cs:114`
pub const PEN: f32 = 1.0;

/// `Symbol.Default.Size`: a symbol's width and height.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/Symbol.cs:99`
const SYMBOL_SIZE: f32 = 7.0;

/// The most points one gpui path is given; a longer line is painted in pieces that overlap by a
/// point.
const CHUNK: usize = 60_000;

/// One curve to paint: its points as fractions of the plot - `x` from the left, `y` from the top,
/// beyond `0..=1` where the line leaves the plot - its colour, and whether each point carries a
/// diamond, `SymbolType.Diamond`.
#[derive(Debug, Clone, PartialEq)]
pub struct Line {
    /// The points, joined in order.
    pub points: Vec<(f32, f32)>,
    /// The line's colour, and its symbols'.
    pub colour: Hsla,
    /// A hollow diamond at each point.
    pub diamonds: bool,
}

/// A series over `from..to` against `range` as the line through it, in fractions of the plot,
/// reduced to `columns` columns - one a pixel. A value off the range is placed beyond the edge,
/// for the clip to cut the line where it leaves.
#[must_use]
#[allow(clippy::cast_possible_truncation)] // fractions of a plot
pub fn curve(series: &Series, range: Range, from: f64, to: f64, columns: usize) -> Vec<(f32, f32)> {
    mp_chart::trace(series, from, to, columns)
        .into_iter()
        .map(|(x, value)| (x as f32, (1.0 - range.place(value)) as f32))
        .collect()
}

/// The curves, painted over the whole of the element they are put in and clipped to it, as
/// `IsClippedToChartRect` clips ZedGraph's: each line of [`PEN`] in its colour, then its
/// diamonds.
pub fn element(lines: Vec<Line>) -> impl IntoElement {
    canvas(
        |_bounds, _window, _cx| (),
        move |bounds, (), window, _cx| {
            let (left, top) = (f32::from(bounds.origin.x), f32::from(bounds.origin.y));
            let (width, height) = (f32::from(bounds.size.width), f32::from(bounds.size.height));
            window.with_content_mask(Some(ContentMask { bounds }), |window| {
                for line in &lines {
                    let pixels: Vec<(f32, f32)> = line
                        .points
                        .iter()
                        .map(|(x, y)| (left + x * width, top + y * height))
                        .collect();
                    paint_polyline(window, &pixels, line.colour, PEN);
                    if line.diamonds {
                        for (x, y) in &pixels {
                            paint_diamond(window, *x, *y, line.colour);
                        }
                    }
                }
            });
        },
    )
    .absolute()
    .size_full()
}

/// A line `width` pixels wide through screen points, in chunks a gpui path can hold, each
/// starting where the one before ended so the line has no break at the join.
pub fn paint_polyline(window: &mut Window, points: &[(f32, f32)], colour: Hsla, width: f32) {
    let mut start = 0;
    while start + 1 < points.len() {
        let end = (start + CHUNK).min(points.len());
        let Some(chunk) = points.get(start..end) else {
            break;
        };
        start = end - 1;
        let mut builder = PathBuilder::stroke(px(width));
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

/// `SymbolType.Diamond` at ZedGraph's defaults: [`SYMBOL_SIZE`] across, its border a pen of one
/// in the curve's colour, not filled (`Symbol.Default.FillType` is `None`).
/// `// C#: ExtLibs/ZedGraph/ZedGraph/Symbol.cs:99-142`
fn paint_diamond(window: &mut Window, x: f32, y: f32, colour: Hsla) {
    let half = SYMBOL_SIZE / 2.0;
    let mut builder = PathBuilder::stroke(px(PEN));
    builder.move_to(point(px(x), px(y - half)));
    builder.line_to(point(px(x + half), px(y)));
    builder.line_to(point(px(x), px(y + half)));
    builder.line_to(point(px(x - half), px(y)));
    builder.close();
    if let Ok(path) = builder.build() {
        window.paint_path(path, colour);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Fractions of the plot, `y` from the top: the range's low at the bottom, its high at the
    /// top, and a value off it beyond the edge rather than on it.
    #[test]
    fn a_curve_is_placed_in_the_plot_from_the_top() {
        let mut series = Series::new("s", 10);
        for (at, value) in [(0.0, 0.0), (5.0, 10.0), (10.0, 20.0)] {
            series.push(at, value);
        }
        let range = Range {
            low: 0.0,
            high: 10.0,
        };
        let points = curve(&series, range, 0.0, 10.0, 10);
        assert_eq!(points.len(), 3, "{points:?}");
        assert!((points[0].1 - 1.0).abs() < 1e-6, "{points:?}");
        assert!(points[1].1.abs() < 1e-6, "{points:?}");
        assert!((points[2].1 + 1.0).abs() < 1e-6, "{points:?}");
        assert!(points.windows(2).all(|pair| pair[0].0 < pair[1].0));
    }
}
