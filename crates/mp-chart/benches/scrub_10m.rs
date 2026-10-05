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

//! Deliverable 14's scrub budget: a cursor dragged across eight series of ten million samples each, every
//! step inside a 120 fps frame (8.33 ms), and the work provably the plot's width, not the log's
//! length.
//!
//! **What a step is.** What the log browser does when the cursor moves and the chart is drawn
//! again (`crates/mp-gui/src/logbrowse.rs`), on the same crates:
//! - *the cursor* - `double_click`: the x under the pointer, and on a time axis the first
//!   `GPS`/`GPS2`/`POS` record at or after it (`Positions::line_at_time`); then `go_to_sample`:
//!   the map marker from the position record nearest that line (`Positions::from_row`), and
//!   `cursor_x`, the record's time (`Positions::time_of`);
//! - *the frame* - `plot_panel`: `scales()`, which is every series' `extent()` and an
//!   `auto_range` per axis (`Axes::over`; here four series on the left and four on the right),
//!   then `mp_chart::reduce` of every series to the browser's 240 columns.
//!
//! The grid's scroll and the map's redraw are the GUI's own and are not measured.
//!
//! **The plot** is built as the browser builds one: a generated dataflash log - one message of
//! eight float fields logged at 400 Hz, `GPS` at 5 Hz - opened with `LogFile`, each field taken
//! with `extract` and pushed into a `Series` of its length. Ten million records is a 430 MB log
//! and seven hours at 400 Hz.
//!
//! **The scrub** is 1,000 steps from the left edge to the right, twice: with the whole log in
//! view, and zoomed to one minute centred on the cursor, as the grid's double click centres the
//! axis. A minute rather than a fraction of the log, so that both sizes put the same 24,000
//! samples a series on screen and the only difference between them is the length of the log.
//! Each step is timed on its own; the p50 and p99 are reported for the cursor, the frame, and the
//! two together, with the reads the reductions made (`mp_chart::reduce_counted`: every sample,
//! block summary and block start read, one each).
//!
//! **The gate**, in an optimised build (`cargo bench -p mp-chart --bench scrub_10m`):
//! - the whole step's p99 at 10 M under 8.33 ms, in both views;
//! - the work the width's, not the length's: the most reads any frame made at 10 M within 1.5
//!   times the most at 1 M - where the scan it replaced reads ten times as many - and under
//!   `MAX_READS_PER_COLUMN` a column. A count, so it is the same on every machine at every load,
//!   and is the proof; the 1.5 is the pyramid's one extra level at 10 M (four levels to three);
//! - the time following the work: the step's p50 at 10 M within `MAX_TIME_RATIO` of its p50 at
//!   1 M. Looser than the reads, because a read is not a fixed cost: ten million samples a series
//!   (1.28 GB for eight) miss every cache, a million a series (128 MB) sits partly in the last
//!   level of it, and the other agents' builds on this machine move the times by half again from
//!   run to run. The scan was ten times.
//!
//! Before timing, the reduction at 10 M is checked against `reduce_scan`, the pass over every
//! sample it replaced, at 240 and 1,920 columns: the same columns, lows and highs, so the same
//! pixels.
//!
//! In an unoptimised build (`cargo test --benches`) it builds a 20,000-record plot, checks the
//! reduction against the scan, and times nothing.
//!
//! Measured on the development machine, an i7-10875H (8 cores, 16 threads, 32 GB, swap full),
//! release profile, with other agents' builds running beside it (see the run's own output for
//! the load): see DELIVERABLES.md Deliverable 14 for the figures of the last run.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::print_stdout, clippy::cast_precision_loss)]
#![allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]

use std::hint::black_box;
use web_time::{Duration, Instant};

use mp_chart::{Series, auto_range, reduce, reduce_counted, reduce_scan};
use mp_log::logfile::LogFile;
use mp_log::overlay::{LineAtTime, Positions};

/// The browser's chart width in columns: `logbrowse.rs`'s `COLUMNS`.
const COLUMNS: usize = 240;

/// A frame at 120 fps.
const BUDGET: Duration = Duration::from_micros(8_333);

/// How many more reads a frame at 10 M may make than one at 1 M.
const MAX_READS_RATIO: f64 = 1.5;

/// How many reads a column may cost: the searches and the min/max down four levels, with room.
const MAX_READS_PER_COLUMN: usize = 800;

/// How much slower a step at 10 M may be than at 1 M, in time.
const MAX_TIME_RATIO: f64 = 2.5;

/// Steps across the plot.
const STEPS: usize = 1_000;

/// The eight fields.
const FIELDS: [&str; 8] = ["A", "B", "C", "D", "E", "F", "G", "H"];

/// Dataflash head bytes and the `FMT` type.
const HEAD: [u8; 2] = [0xA3, 0x95];
const FMT: u8 = 0x80;
const SCRB: u8 = 200;
const GPS: u8 = 201;

fn fixed(text: &str, width: usize) -> Vec<u8> {
    let mut out = text.as_bytes().to_vec();
    out.resize(width, 0);
    out
}

fn fmt(msg_type: u8, length: u8, name: &str, format: &str, columns: &str) -> Vec<u8> {
    let mut out = vec![HEAD[0], HEAD[1], FMT, msg_type, length];
    out.extend(fixed(name, 4));
    out.extend(fixed(format, 16));
    out.extend(fixed(columns, 64));
    out
}

/// A log of `records` eight-field records at 400 Hz with a `GPS` fix every eightieth.
///
/// The fields are sines of different periods with noise and, now and then, a one-sample spike:
/// the shape of IMU data, and something for min/max to keep.
fn generate(records: usize) -> Vec<u8> {
    let mut log = fmt(
        SCRB,
        3 + 8 + 32,
        "SCRB",
        "Qffffffff",
        "TimeUS,A,B,C,D,E,F,G,H",
    );
    log.extend(fmt(GPS, 20, "GPS", "QBLL", "TimeUS,Status,Lat,Lng"));
    log.reserve(records * 43 + records / 80 * 20);
    let mut state = 0x2545_f491_4f6c_dd1du64;
    for i in 0..records {
        let time_us = 60_000_000u64 + i as u64 * 2_500;
        log.extend([HEAD[0], HEAD[1], SCRB]);
        log.extend(time_us.to_le_bytes());
        let t = i as f64 / 400.0;
        for (k, _) in FIELDS.iter().enumerate() {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let noise = (state >> 40) as f64 / (1u64 << 24) as f64 - 0.5;
            let period = 2.0 + k as f64 * 7.0;
            let spike = if (i + k * 977) % 1_000_003 == 0 {
                50.0
            } else {
                0.0
            };
            let value = (t * std::f64::consts::TAU / period).sin() * (k + 1) as f64 + noise + spike;
            log.extend((value as f32).to_le_bytes());
        }
        if i % 80 == 0 {
            log.extend([HEAD[0], HEAD[1], GPS]);
            log.extend(time_us.to_le_bytes());
            log.push(3);
            let lat = -353_000_000i32 + (i / 80) as i32;
            let lng = 1_491_000_000i32 + (i / 80) as i32;
            log.extend(lat.to_le_bytes());
            log.extend(lng.to_le_bytes());
        }
    }
    log
}

/// What the browser holds once the log is open and the eight fields are plotted.
struct Plot {
    series: Vec<Series>,
    positions: Positions,
    origin: f64,
}

/// Opens the log and plots every field, as `LogBrowse::open` and `graph` do.
fn plot(records: usize) -> Plot {
    let started = Instant::now();
    let log = LogFile::from_bytes(generate(records));
    let origin = log.time_origin().expect("a timed log");
    let positions = log.positions();
    let series = FIELDS
        .iter()
        .map(|field| {
            let points = log.extract("SCRB", field);
            assert_eq!(points.len(), records);
            // `Plotted::modified`: one sample a point, the unit's multiplier (1 here) applied.
            let mut series = Series::new(format!("SCRB.{field}"), points.len().max(1));
            for point in &points {
                series.push(point.seconds, point.value);
            }
            series
        })
        .collect();
    println!(
        "scrub_10m: {records} records x {} fields plotted in {:.2?}",
        FIELDS.len(),
        started.elapsed()
    );
    Plot {
        series,
        positions,
        origin,
    }
}

/// The x range every curve spans: `LogBrowse::automatic`.
fn whole(plot: &Plot) -> (f64, f64) {
    plot.series
        .iter()
        .filter_map(Series::extent)
        .reduce(|(low, high), (from, to)| (low.min(from), high.max(to)))
        .expect("an extent")
}

/// The cursor: `double_click` at `fraction` across `(from, to)`, then `go_to_sample` and
/// `cursor_x`. Returns the cursor's x, which a zoom centres on.
fn cursor(plot: &Plot, (from, to): (f64, f64), fraction: f64) -> Option<f64> {
    let x = fraction.mul_add(to - from, from);
    let line = match plot
        .positions
        .line_at_time(x.mul_add(1_000_000.0, plot.origin))
    {
        LineAtTime::Line(line) => line,
        LineAtTime::AfterAll | LineAtTime::NoPositions => return None,
    };
    black_box(plot.positions.from_row(line));
    let time = plot.positions.time_of(line)?;
    Some(mp_log::plot::seconds_since(plot.origin, time))
}

/// The frame: `scales()` - every extent and each axis's range - and every series reduced.
/// Returns the reductions' reads.
fn frame(plot: &Plot, zoom: Option<(f64, f64)>) -> usize {
    let automatic = whole(plot);
    let (from, to) = zoom.unwrap_or(automatic);
    let (left, right) = plot.series.split_at(4);
    let left: Vec<&Series> = left.iter().collect();
    let right: Vec<&Series> = right.iter().collect();
    black_box(auto_range(&left, from, to));
    black_box(auto_range(&right, from, to));
    plot.series
        .iter()
        .map(|series| black_box(reduce_counted(series, from, to, COLUMNS)).1)
        .sum()
}

/// The frame as it was: every extent, range and reduction a pass over every sample.
fn frame_scan(plot: &Plot) -> usize {
    let extent = plot
        .series
        .iter()
        .filter_map(|series| {
            let mut samples = series.samples();
            let first = samples.next()?.at;
            Some(samples.fold((first, first), |(l, h), s| (l.min(s.at), h.max(s.at))))
        })
        .reduce(|(low, high), (from, to)| (low.min(from), high.max(to)))
        .expect("an extent");
    for half in plot.series.chunks(4) {
        let mut low = f64::INFINITY;
        let mut high = f64::NEG_INFINITY;
        for series in half {
            for sample in series.samples() {
                if sample.at >= extent.0 && sample.at <= extent.1 {
                    low = low.min(sample.value);
                    high = high.max(sample.value);
                }
            }
        }
        black_box((low, high));
    }
    plot.series
        .iter()
        .map(|series| black_box(reduce_scan(series, extent.0, extent.1, COLUMNS)).len())
        .sum()
}

/// The reduction through the index against the scan, over the whole log and a zoomed window.
fn check_pixels(plot: &Plot) {
    let (from, to) = whole(plot);
    let zoomed = (from + (to - from) * 0.37, from + (to - from) * 0.38);
    for series in &plot.series {
        for (from, to) in [(from, to), zoomed] {
            for columns in [COLUMNS, 1_920] {
                assert_eq!(
                    reduce(series, from, to, columns),
                    reduce_scan(series, from, to, columns),
                    "{} at {columns} columns over {from}..{to}",
                    series.name
                );
            }
        }
    }
}

/// Per-step times, sorted, and the most reads any frame made.
struct Timings {
    cursor: Vec<Duration>,
    frame: Vec<Duration>,
    step: Vec<Duration>,
    reads: usize,
}

fn percentile(sorted: &[Duration], p: f64) -> Duration {
    let rank = ((sorted.len() as f64 * p).ceil() as usize).clamp(1, sorted.len());
    sorted[rank - 1]
}

/// The zoomed view's span, seconds.
const ZOOM_SECONDS: f64 = 60.0;

/// Drags the cursor across the plot, `STEPS` steps, whole or zoomed to a minute.
fn scrub(plot: &Plot, zoomed: bool) -> Timings {
    let whole = whole(plot);
    let span = ZOOM_SECONDS;
    let mut view = whole;
    let mut timings = Timings {
        cursor: Vec::with_capacity(STEPS),
        frame: Vec::with_capacity(STEPS),
        step: Vec::with_capacity(STEPS),
        reads: 0,
    };
    for step in 0..STEPS {
        let fraction = (step as f64 + 0.5) / STEPS as f64;
        let started = Instant::now();
        let x = cursor(plot, whole, fraction);
        let cursor_done = Instant::now();
        if zoomed && let Some(x) = x {
            view = (x - span / 2.0, x + span / 2.0);
        }
        let reads = frame(plot, zoomed.then_some(view));
        let ended = Instant::now();
        timings.reads = timings.reads.max(reads);
        timings.cursor.push(cursor_done - started);
        timings.frame.push(ended - cursor_done);
        timings.step.push(ended - started);
    }
    timings.cursor.sort_unstable();
    timings.frame.sort_unstable();
    timings.step.sort_unstable();
    timings
}

fn report(label: &str, timings: &Timings) {
    let ms = |d: Duration| d.as_secs_f64() * 1000.0;
    println!(
        "scrub_10m: {label:<18} cursor p50 {:.4} ms p99 {:.4} ms | frame p50 {:.3} ms p99 {:.3} ms | step p50 {:.3} ms p99 {:.3} ms, max {:.3} ms | reads <= {}",
        ms(percentile(&timings.cursor, 0.5)),
        ms(percentile(&timings.cursor, 0.99)),
        ms(percentile(&timings.frame, 0.5)),
        ms(percentile(&timings.frame, 0.99)),
        ms(percentile(&timings.step, 0.5)),
        ms(percentile(&timings.step, 0.99)),
        ms(percentile(&timings.step, 1.0)),
        timings.reads,
    );
}

fn load() -> String {
    std::fs::read_to_string("/proc/loadavg")
        .ok()
        .and_then(|text| text.split_whitespace().next().map(str::to_owned))
        .unwrap_or_else(|| "unknown".to_owned())
}

fn main() {
    if cfg!(debug_assertions) {
        let plot = plot(20_000);
        check_pixels(&plot);
        let timings = scrub(&plot, true);
        assert_eq!(timings.step.len(), STEPS);
        println!("scrub_10m: unoptimised build - checked the reduction, timed nothing");
        return;
    }
    println!("scrub_10m: load average {}", load());
    let mut results = Vec::new();
    for records in [1_000_000, 10_000_000] {
        let plot = plot(records);
        check_pixels(&plot);
        // Warm: the first pass pages the samples in.
        let _ = scrub(&plot, false);
        let full = scrub(&plot, false);
        let zoomed = scrub(&plot, true);
        report(&format!("{}M whole log", records / 1_000_000), &full);
        report(&format!("{}M zoomed 1 min", records / 1_000_000), &zoomed);
        // The frame as it was, for the record: a handful of steps is enough to see it.
        let started = Instant::now();
        for _ in 0..3 {
            black_box(frame_scan(&plot));
        }
        println!(
            "scrub_10m: {}M before (every frame a pass over every sample): {:.1} ms a frame",
            records / 1_000_000,
            started.elapsed().as_secs_f64() * 1000.0 / 3.0
        );
        results.push((records, full, zoomed));
    }
    println!("scrub_10m: load average {}", load());

    let [(_, small_full, small_zoomed), (_, big_full, big_zoomed)] = &results[..] else {
        unreachable!("two sizes were run");
    };
    for (label, small, big) in [
        ("whole log", small_full, big_full),
        ("zoomed", small_zoomed, big_zoomed),
    ] {
        let p99 = percentile(&big.step, 0.99);
        let ratio =
            percentile(&big.step, 0.5).as_secs_f64() / percentile(&small.step, 0.5).as_secs_f64();
        let reads_ratio = big.reads as f64 / small.reads as f64;
        let per_column = big.reads / (COLUMNS * FIELDS.len());
        println!(
            "scrub_10m: {label}: 10M/1M reads ratio {reads_ratio:.2} ({} to {}, {per_column} a column), \
             step p50 ratio {ratio:.2}, 10M p99 {p99:.2?}",
            small.reads, big.reads
        );
        assert!(
            p99 <= BUDGET,
            "{label}: a step's p99 at 10 M is {p99:.2?}, over the {BUDGET:.2?} of a 120 fps frame"
        );
        assert!(
            reads_ratio <= MAX_READS_RATIO && per_column <= MAX_READS_PER_COLUMN,
            "{label}: a frame at 10 M reads {reads_ratio:.2} times what one at 1 M does, \
             {per_column} a column; the work is not the width's"
        );
        assert!(
            ratio <= MAX_TIME_RATIO,
            "{label}: a step at 10 M takes {ratio:.2} times one at 1 M"
        );
    }
}
