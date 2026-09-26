//! The index is invisible: every query through it gives what a pass over every sample gives.
//!
//! The chart draws a series as the line through [`reduce`]'s columns - each column's first
//! value, its low and high, and its last, at its index across the plot - and fits its axes with
//! [`auto_range`] over the window [`Series::extent`] gives. So if those three give the scan's
//! answer, the line drawn is the same to the pixel: the same columns, the same four values each,
//! the same range to place them in. These tests hold the
//! indexed queries to the scan - [`reduce_scan`], the reduction as it was first written, and the
//! range and extent as they were first written, copied below - over series that roll, that
//! repeat times, whose clock restarts, at the widths the log browser (240 columns) and the tuning
//! graph use and at screen widths, over whole, zoomed and off-the-end windows.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(clippy::cast_precision_loss)]

use mp_chart::{Range, Series, auto_range, reduce, reduce_scan};
use proptest::prelude::*;

/// The widths asked for: the log browser's `COLUMNS`, and a screen's worth either side.
const WIDTHS: [usize; 9] = [1, 2, 3, 7, 240, 800, 1024, 1920, 2560];

/// `Series::extent` as it was first written: a fold over every sample.
fn extent_scan(series: &Series) -> Option<(f64, f64)> {
    let mut samples = series.samples();
    let first = samples.next()?.at;
    Some(samples.fold((first, first), |(low, high), sample| {
        (low.min(sample.at), high.max(sample.at))
    }))
}

/// `auto_range` as it was first written: a pass over every sample of every series.
fn auto_range_scan(series: &[&Series], from: f64, to: f64) -> Option<Range> {
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    for one in series {
        for sample in one.samples() {
            if sample.at >= from && sample.at <= to {
                low = low.min(sample.value);
                high = high.max(sample.value);
            }
        }
    }
    if !low.is_finite() || !high.is_finite() {
        return None;
    }
    let padding = (high - low).abs() * 0.05;
    let padding = if padding < f64::EPSILON {
        high.abs().max(1.0) * 0.05
    } else {
        padding
    };
    Some(Range {
        low: low - padding,
        high: high + padding,
    })
}

/// Every query through the index against the scan, over the whole extent and the windows given.
fn check(series: &Series, windows: &[(f64, f64)]) {
    assert_eq!(series.extent(), extent_scan(series), "extent");
    let mut all = windows.to_vec();
    if let Some(whole) = extent_scan(series) {
        all.push(whole);
    }
    for (from, to) in all {
        assert_eq!(
            auto_range(&[series], from, to),
            auto_range_scan(&[series], from, to),
            "range over {from}..{to}"
        );
        for width in WIDTHS {
            assert_eq!(
                reduce(series, from, to, width),
                reduce_scan(series, from, to, width),
                "{width} columns over {from}..{to}"
            );
        }
    }
}

/// A series from `(step, value)` pairs: each time `step` on from the last, a negative step being
/// a clock that restarted.
fn series_of(capacity: usize, steps: &[(f64, f64)]) -> Series {
    let mut series = Series::new("s", capacity);
    let mut at = 0.0;
    for (step, value) in steps {
        at += step;
        series.push(at, *value);
    }
    series
}

/// A deterministic pseudo-random sequence in `[0, 1)`.
fn noise(seed: u64) -> impl FnMut() -> f64 {
    let mut state = seed | 1;
    move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1u64 << 53) as f64
    }
}

#[test]
fn a_long_log_reduces_as_the_scan_does() {
    // 300,000 samples at 400 Hz: four levels of pyramid.
    let mut next = noise(7);
    let mut series = Series::new("imu", 300_000);
    for i in 0..300_000u32 {
        let value = (f64::from(i) * 0.01).sin() * 10.0 + next();
        series.push(f64::from(i) / 400.0, value);
    }
    check(
        &series,
        &[
            (0.0, 750.0),
            (100.0, 100.5),
            (123.456, 400.0),
            (-50.0, 10.0),
            (749.0, 900.0),
            (1000.0, 2000.0),
            (0.0, 0.0025),
        ],
    );
}

#[test]
fn a_log_whose_clock_restarts_reduces_as_the_scan_does() {
    // Three flights in one file: the time axis clamps each reboot to the origin and starts again.
    let mut next = noise(11);
    let mut series = Series::new("att", 90_000);
    for flight in 0..3 {
        for i in 0..30_000u32 {
            let at = f64::from(i) / 50.0 + f64::from(flight) * 13.0;
            series.push(at, next() * 100.0 - 50.0);
        }
    }
    check(&series, &[(0.0, 50.0), (300.0, 700.0), (13.0, 13.02)]);
}

#[test]
fn a_rolling_series_reduces_as_the_scan_does() {
    // The tuning graph's series: bounded, the oldest dropped, far more pushed than held.
    let mut next = noise(3);
    let mut series = Series::new("roll", 5_000);
    for i in 0..123_457u32 {
        series.push(f64::from(i) / 100.0, next());
        if i % 20_011 == 0 {
            check(
                &series,
                &[(f64::from(i) / 100.0 - 10.0, f64::from(i) / 100.0)],
            );
        }
    }
    let end = 123_456.0 / 100.0;
    check(&series, &[(end - 10.0, end), (end - 30.0, end - 20.0)]);
}

#[test]
fn repeated_times_and_signed_zeros_reduce_as_the_scan_does() {
    let mut series = Series::new("steps", 10_000);
    for i in 0..10_000u32 {
        // Ten samples at every time, values that are zero of either sign now and then.
        let at = f64::from(i / 10);
        let value = match i % 7 {
            0 => 0.0,
            1 => -0.0,
            _ => f64::from(i % 13) - 6.0,
        };
        series.push(at, value);
    }
    check(&series, &[(0.0, 999.0), (10.0, 10.0), (5.0, 6.0)]);
}

#[test]
fn a_clock_that_goes_back_at_every_sample_is_scanned() {
    // Past the index's run limit: every query falls back to the scan, and still agrees.
    let mut next = noise(5);
    let mut series = Series::new("chaos", 2_000);
    for _ in 0..2_000 {
        series.push(next() * 100.0, next());
    }
    check(&series, &[(10.0, 20.0), (0.0, 100.0)]);
}

#[test]
fn unplaceable_windows_are_scanned() {
    let series = series_of(100, &[(1.0, 1.0); 100]);
    for (from, to) in [
        (f64::NEG_INFINITY, 50.0),
        (0.0, f64::INFINITY),
        (f64::NAN, 50.0),
        (-f64::MAX, f64::MAX),
    ] {
        assert_eq!(
            reduce(&series, from, to, 240),
            reduce_scan(&series, from, to, 240)
        );
        assert_eq!(
            auto_range(&[&series], from, to),
            auto_range_scan(&[&series], from, to)
        );
    }
}

#[test]
fn a_time_that_is_not_a_number_is_not_stored() {
    let mut series = Series::new("s", 10);
    series.push(f64::NAN, 1.0);
    series.push(f64::INFINITY, 1.0);
    series.push(1.0, 2.0);
    assert_eq!(series.len(), 1);
    assert_eq!(series.extent(), Some((1.0, 1.0)));
}

#[test]
fn a_cleared_series_indexes_afresh() {
    let mut series = series_of(3_000, &[(0.5, 3.0); 2_500]);
    series.clear();
    assert_eq!(series.extent(), None);
    let mut next = noise(9);
    for i in 0..2_000u32 {
        series.push(f64::from(i), next());
    }
    check(&series, &[(100.0, 1_500.0)]);
}

/// The work is the width's: a sixteen times longer series costs a frame at most one more level of
/// the pyramid's reads - a column's min/max descends one level further for every 32 times as many
/// samples in it - whole or zoomed to the same time on screen, and a few hundred reads a column
/// either way. The scan it replaced read sixteen times as much.
#[test]
fn a_frame_reads_the_same_whatever_the_length() {
    let build = |samples: u32| {
        let mut next = noise(u64::from(samples));
        let mut series = Series::new("long", samples as usize);
        for i in 0..samples {
            series.push(f64::from(i) / 400.0, next());
        }
        series
    };
    let short = build(1 << 16);
    let long = build(1 << 20);
    for (label, window) in [("whole", None), ("one minute", Some((100.0, 160.0)))] {
        let reads = |series: &Series| {
            let (from, to) = window.or_else(|| series.extent()).unwrap();
            let (columns, reads) = mp_chart::reduce_counted(series, from, to, 240);
            assert_eq!(columns, reduce_scan(series, from, to, 240));
            reads
        };
        let (few, many) = (reads(&short), reads(&long));
        assert!(
            many as f64 <= few as f64 * 1.6,
            "{label}: {many} reads at 2^20 samples against {few} at 2^16"
        );
        assert!(many <= 240 * 800, "{label}: {many} reads for 240 columns");
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    /// Any series - rolling or not, with repeats and restarts - and any window.
    #[test]
    fn any_series_reduces_as_the_scan_does(
        capacity in 1usize..4_000,
        steps in prop::collection::vec(
            (prop_oneof![
                200 => 0.0f64..2.0,
                20 => Just(0.0),
                1 => -500.0f64..0.0,
            ], -1e3f64..1e3),
            0..6_000,
        ),
        windows in prop::collection::vec((-100.0f64..3_000.0, 0.0f64..3_000.0), 1..4),
    ) {
        let series = series_of(capacity, &steps);
        let windows: Vec<(f64, f64)> =
            windows.into_iter().map(|(from, span)| (from, from + span)).collect();
        check(&series, &windows);
    }
}
