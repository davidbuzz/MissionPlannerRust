//! Time series, and turning them into something drawable.
//!
//! The data half of the tuning graph and, later, of log plotting. No gpui here: PLAN.md §5.1's L6
//! rule keeps the framework out of anything that does not need it, and the arithmetic that decides
//! where a line goes is worth testing without a window.
//!
//! **The reduction is min/max per pixel column, not sampling.** A chart that picks one sample per
//! column is drawing a different signal from the one that was recorded: it hides the single
//! clipped accelerometer reading and the one-frame GPS glitch, which are exactly the samples
//! somebody opens a tuning graph to find. Two values per column costs nothing and cannot hide a
//! spike. PLAN.md §8.1 makes the same argument for log plotting at ten million points, and this is
//! the same shape at a smaller scale so the two do not end up with different behaviour.
//!
//! **A frame costs the plot's width, not the log's length.** Each series keeps an index - its
//! runs of non-decreasing time and a min/max pyramid over its values (`lod`) - so the extent,
//! [`auto_range`] and [`reduce`] search and read summaries instead of visiting every sample, and
//! give exactly what a pass over every sample gives ([`reduce_scan`], kept as the reference).
//! D14's scrub budget, ten million samples in each of eight series at 120 frames a second, is
//! `benches/scrub_10m.rs`.

use std::collections::VecDeque;

mod lod;

/// How long the tuning graph shows, in seconds.
///
/// Ten, matching `xScale.Min = xScale.Max - 10.0`. `// C#: GCSViews/FlightData.cs:5346`
pub const WINDOW_SECONDS: f64 = 10.0;

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Seconds since the series started.
    pub at: f64,
    /// The value.
    pub value: f64,
}

/// A bounded series of samples, oldest dropped.
#[derive(Debug, Clone)]
pub struct Series {
    /// What to call it on screen.
    pub name: String,
    samples: VecDeque<Sample>,
    capacity: usize,
    /// The runs and the min/max pyramid every query reads instead of the samples.
    index: lod::Index,
}

impl Series {
    /// A series holding at most `capacity` samples.
    #[must_use]
    pub fn new(name: impl Into<String>, capacity: usize) -> Self {
        Self {
            name: name.into(),
            samples: VecDeque::with_capacity(capacity.min(4096)),
            capacity: capacity.max(1),
            index: lod::Index::new(capacity.max(1)),
        }
    }

    /// Adds a sample, dropping the oldest if the series is full.
    ///
    /// Bounded because this runs for the length of a flight. An unbounded series on a four-hour
    /// mission at 10 Hz is 144,000 samples that nothing will ever draw, held so that the last ten
    /// seconds can be.
    pub fn push(&mut self, at: f64, value: f64) {
        // A value that is not a number cannot be drawn and poisons every min and max it touches.
        // Dropped rather than stored: a telemetry field is absent often enough that this is a
        // normal event, not an error. A time that is not a number has no place on the axis
        // either, and would make the series' order meaningless to search.
        if !value.is_finite() || !at.is_finite() {
            return;
        }
        if self.samples.len() >= self.capacity && self.samples.pop_front().is_some() {
            self.index.pop_front();
        }
        let sample = Sample { at, value };
        self.index
            .push(self.samples.len(), self.samples.back(), sample);
        self.samples.push_back(sample);
    }

    /// Everything in the series, oldest first.
    pub fn samples(&self) -> impl Iterator<Item = &Sample> {
        self.samples.iter()
    }

    /// The sample at `index`, oldest first: the `index`-th of [`Series::samples`].
    #[must_use]
    pub fn get(&self, index: usize) -> Option<&Sample> {
        self.samples.get(index)
    }

    /// How many samples are held.
    #[must_use]
    pub fn len(&self) -> usize {
        self.samples.len()
    }

    /// Whether the series is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    /// The newest sample's value, which is what a numeric readout shows.
    #[must_use]
    pub fn latest(&self) -> Option<f64> {
        self.samples.back().map(|sample| sample.value)
    }

    /// The time of the newest sample.
    #[must_use]
    pub fn latest_time(&self) -> Option<f64> {
        self.samples.back().map(|sample| sample.at)
    }

    /// The earliest and latest `at` of any sample: the x range an axis fitted to it spans.
    ///
    /// Not the first and last sample's: a log's clock can restart part way through, and the
    /// samples after the restart would then fall outside a range read off the ends.
    ///
    /// Each run's first and last sample, which within a run are its earliest and latest.
    #[must_use]
    pub fn extent(&self) -> Option<(f64, f64)> {
        if self.index.searchable() {
            return self
                .index
                .runs(self.samples.len())
                .filter_map(|(start, end)| {
                    let first = self.samples.get(start)?.at;
                    let last = self.samples.get(end.checked_sub(1)?)?.at;
                    Some((first, last))
                })
                .reduce(|(low, high), (first, last)| (low.min(first), high.max(last)));
        }
        let mut samples = self.samples.iter();
        let first = samples.next()?.at;
        Some(samples.fold((first, first), |(low, high), sample| {
            (low.min(sample.at), high.max(sample.at))
        }))
    }

    /// Forgets everything.
    pub fn clear(&mut self) {
        self.samples.clear();
        self.index.clear();
    }

    /// The lowest and highest value of the samples from `from` to `to` inclusive, by the index.
    fn value_range(&self, from: f64, to: f64) -> Option<(f64, f64)> {
        if !self.index.searchable() {
            return self
                .samples
                .iter()
                .filter(|sample| sample.at >= from && sample.at <= to)
                .fold(None, |acc, sample| {
                    Some(
                        acc.map_or((sample.value, sample.value), |(low, high): (f64, f64)| {
                            (low.min(sample.value), high.max(sample.value))
                        }),
                    )
                });
        }
        let mut found: Option<(f64, f64)> = None;
        let mut reads = 0;
        for (start, end) in self.index.runs(self.samples.len()) {
            let (lo, hi) = self.window(start, end, from, to, &mut reads);
            if let Some((low, high)) = self.index.min_max(&self.samples, lo, hi, &mut reads) {
                found = Some(found.map_or((low, high), |(l, h)| (l.min(low), h.max(high))));
            }
        }
        found
    }

    /// The positions `[lo, hi)` of a run's samples from `from` to `to` inclusive.
    fn window(
        &self,
        start: usize,
        end: usize,
        from: f64,
        to: f64,
        reads: &mut usize,
    ) -> (usize, usize) {
        let lo = self
            .index
            .search(&self.samples, start, end, |at| at >= from, reads);
        let hi = self
            .index
            .search(&self.samples, lo, end, |at| at > to, reads);
        (lo, hi)
    }
}

/// The value range a set of series occupies over a time window.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Range {
    /// The lowest value.
    pub low: f64,
    /// The highest value.
    pub high: f64,
}

impl Range {
    /// How tall the range is, never zero.
    ///
    /// A flat signal has no range at all, and dividing by it puts every point at infinity. Given a
    /// floor so a constant line draws through the middle rather than disappearing.
    #[must_use]
    pub fn span(self) -> f64 {
        let span = self.high - self.low;
        if span.abs() < f64::EPSILON { 1.0 } else { span }
    }

    /// Where a value sits in the range, 0 at the bottom and 1 at the top.
    #[must_use]
    pub fn fraction(self, value: f64) -> f64 {
        ((value - self.low) / self.span()).clamp(0.0, 1.0)
    }
}

/// The range a set of series occupies over a window, with a little room above and below.
///
/// Padded by a twentieth, because a trace that touches the top and bottom pixel of its box reads
/// as clipped even when it is not - and on a tuning graph "is this clipping" is usually the
/// question being asked.
#[must_use]
pub fn auto_range(series: &[&Series], from: f64, to: f64) -> Option<Range> {
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    for one in series {
        if let Some((one_low, one_high)) = one.value_range(from, to) {
            low = low.min(one_low);
            high = high.max(one_high);
        }
    }
    if !low.is_finite() || !high.is_finite() {
        return None;
    }
    let padding = (high - low).abs() * 0.05;
    // A flat trace gets its own padding, or the range stays zero and it lands on one edge.
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

/// One pixel column of a reduced series: the lowest and highest value in it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Column {
    /// Which column, from the left.
    pub index: usize,
    /// The lowest value in this column.
    pub low: f64,
    /// The highest value in this column.
    pub high: f64,
}

/// Reduces a series to at most `columns` min/max pairs over a time window.
///
/// Columns with no samples are absent rather than zero, so a gap in telemetry draws as a gap
/// instead of as a line to the axis - which is what it would mean.
///
/// Through the series' index: each run's stretch inside the window found by search, then each
/// column's stretch within it by search on the same expression [`reduce_scan`] places a sample
/// with, and its min/max read from the pyramid - a few hundred reads a column however many
/// samples it holds. The result is [`reduce_scan`]'s.
#[must_use]
pub fn reduce(series: &Series, from: f64, to: f64, columns: usize) -> Vec<Column> {
    reduce_counted(series, from, to, columns).0
}

/// [`reduce`], and how many things it read to get there: sample times and values, block
/// summaries and block start times, one each. The measure D14's "work provably O(width)" is
/// held to (`benches/scrub_10m.rs`): a count, not a time, so it is the same on every machine at
/// every load. A scan reads every sample; the index reads a few hundred things a column.
#[must_use]
pub fn reduce_counted(series: &Series, from: f64, to: f64, columns: usize) -> (Vec<Column>, usize) {
    if columns == 0 || to <= from {
        return (Vec::new(), 0);
    }
    let span = to - from;
    // Bounds that are not numbers, or a span too wide to be one, place samples in ways that are
    // not monotonic in time, and so cannot be searched for; the scan places them as it always has.
    if !series.index.searchable() || !from.is_finite() || !span.is_finite() {
        return (reduce_scan(series, from, to, columns), series.len());
    }
    #[allow(clippy::cast_precision_loss)] // a column count is a viewport width
    let width = columns as f64;
    // The scan's expression, exactly.
    let column_of = |at: f64| -> usize {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let index = (((at - from) / span) * width) as usize;
        index.min(columns - 1)
    };
    let mut out: Vec<Option<Column>> = vec![None; columns];
    let samples = &series.samples;
    let mut reads = 0;
    for (start, end) in series.index.runs(samples.len()) {
        let (lo, hi) = series.window(start, end, from, to, &mut reads);
        let mut first = lo;
        while let Some(sample) = samples.get(first).filter(|_| first < hi) {
            reads += 1;
            let index = column_of(sample.at);
            let last = series.index.search(
                samples,
                first + 1,
                hi,
                |at| column_of(at) > index,
                &mut reads,
            );
            if let (Some((low, high)), Some(slot)) = (
                series.index.min_max(samples, first, last, &mut reads),
                out.get_mut(index),
            ) {
                match slot {
                    Some(column) => {
                        column.low = column.low.min(low);
                        column.high = column.high.max(high);
                    }
                    None => *slot = Some(Column { index, low, high }),
                }
            }
            first = last;
        }
    }
    (out.into_iter().flatten().collect(), reads)
}

/// [`reduce`] by a pass over every sample: the reduction as it was first written, kept as the
/// reference the index is held to and for a series whose clock goes backwards too often to
/// search.
#[must_use]
pub fn reduce_scan(series: &Series, from: f64, to: f64, columns: usize) -> Vec<Column> {
    if columns == 0 || to <= from {
        return Vec::new();
    }
    let mut out: Vec<Option<Column>> = vec![None; columns];
    let span = to - from;
    for sample in series.samples() {
        if sample.at < from || sample.at > to {
            continue;
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        // The fraction is clamped to 0..1 and columns is bounded by a viewport width, so this
        // cannot exceed usize.
        let index = (((sample.at - from) / span) * (columns as f64)) as usize;
        let index = index.min(columns - 1);
        match out.get_mut(index) {
            Some(Some(column)) => {
                column.low = column.low.min(sample.value);
                column.high = column.high.max(sample.value);
            }
            Some(slot) => {
                *slot = Some(Column {
                    index,
                    low: sample.value,
                    high: sample.value,
                });
            }
            None => {}
        }
    }
    out.into_iter().flatten().collect()
}

/// The window a rolling chart should show at a given moment.
///
/// Rolls forward once the newest sample passes the right-hand edge, as the C# does - it moves the
/// axis rather than scrolling every frame, so a trace stays still until it reaches the edge.
/// `// C#: GCSViews/FlightData.cs:5342-5347`
#[must_use]
pub fn window(latest: f64) -> (f64, f64) {
    let end = latest.max(WINDOW_SECONDS);
    (end - WINDOW_SECONDS, end)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn series_of(values: &[(f64, f64)]) -> Series {
        let mut series = Series::new("test", 1000);
        for (at, value) in values {
            series.push(*at, *value);
        }
        series
    }

    /// Bounded, because this runs for the length of a flight.
    #[test]
    fn a_full_series_drops_its_oldest_sample() {
        let mut series = Series::new("s", 3);
        for index in 0..5 {
            #[allow(clippy::cast_precision_loss)]
            series.push(index as f64, index as f64);
        }
        assert_eq!(series.len(), 3);
        assert_eq!(series.latest(), Some(4.0));
        let times: Vec<f64> = series.samples().map(|s| s.at).collect();
        assert_eq!(times, vec![2.0, 3.0, 4.0]);
    }

    /// `get` counts from the oldest sample held, as `samples` does, after the oldest is dropped.
    #[test]
    fn get_is_the_nth_of_samples() {
        let mut series = Series::new("s", 3);
        for index in 0..5 {
            #[allow(clippy::cast_precision_loss)]
            series.push(index as f64, index as f64);
        }
        assert_eq!(series.get(0).map(|s| s.at), Some(2.0));
        assert_eq!(series.get(2).map(|s| s.at), Some(4.0));
        assert_eq!(series.get(3), None);
    }

    /// A NaN cannot be drawn and poisons every min and max it touches.
    #[test]
    fn a_value_that_is_not_a_number_is_not_stored() {
        let mut series = Series::new("s", 10);
        series.push(0.0, 1.0);
        series.push(1.0, f64::NAN);
        series.push(2.0, f64::INFINITY);
        series.push(3.0, 2.0);
        assert_eq!(series.len(), 2);
        assert_eq!(series.latest(), Some(2.0));
    }

    /// The reduction must never hide a spike. This is the whole argument for min/max.
    #[test]
    fn a_one_sample_spike_survives_the_reduction() {
        // A flat signal with one sample far above it, reduced to four columns.
        let mut values: Vec<(f64, f64)> = (0..100).map(|i| (f64::from(i) / 10.0, 1.0)).collect();
        values[55] = (5.5, 99.0);
        let series = series_of(&values);

        let columns = reduce(&series, 0.0, 10.0, 4);
        let highest = columns
            .iter()
            .map(|column| column.high)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (highest - 99.0).abs() < f64::EPSILON,
            "the spike was reduced away, which is the one thing this must not do"
        );
    }

    /// A gap in telemetry is a gap, not a line to the axis.
    #[test]
    fn a_column_with_no_samples_is_absent_rather_than_zero() {
        // Samples at the start and the end, nothing in the middle.
        let series = series_of(&[(0.0, 5.0), (0.1, 5.0), (9.9, 5.0), (10.0, 5.0)]);
        let columns = reduce(&series, 0.0, 10.0, 10);
        assert!(columns.len() < 10, "the empty middle should not be filled");
        assert!(columns.iter().all(|column| column.low > 0.0));
        // And the ones that exist are at the ends.
        assert_eq!(columns.first().map(|c| c.index), Some(0));
        assert_eq!(columns.last().map(|c| c.index), Some(9));
    }

    /// A flat trace has no range, and dividing by it puts every point at infinity.
    #[test]
    fn a_constant_signal_still_has_somewhere_to_draw() {
        let series = series_of(&[(0.0, 7.0), (1.0, 7.0), (2.0, 7.0)]);
        let range = auto_range(&[&series], 0.0, 10.0).expect("a range");
        assert!(
            range.high > range.low,
            "a flat trace needs a range to sit in"
        );
        let fraction = range.fraction(7.0);
        assert!(
            (0.2..=0.8).contains(&fraction),
            "a constant line should sit away from the edges, got {fraction}"
        );
        assert!(fraction.is_finite());
    }

    /// The padding is there so a trace that reaches its extreme does not read as clipped.
    #[test]
    fn the_range_leaves_room_above_and_below() {
        let series = series_of(&[(0.0, 0.0), (1.0, 100.0)]);
        let range = auto_range(&[&series], 0.0, 10.0).expect("a range");
        assert!(range.low < 0.0, "the minimum should not touch the floor");
        assert!(
            range.high > 100.0,
            "the maximum should not touch the ceiling"
        );
        assert!(range.fraction(0.0) > 0.0);
        assert!(range.fraction(100.0) < 1.0);
    }

    /// Several series share one axis, so the range covers all of them.
    #[test]
    fn the_range_covers_every_series_shown() {
        let low = series_of(&[(0.0, -5.0), (1.0, 0.0)]);
        let high = series_of(&[(0.0, 10.0), (1.0, 20.0)]);
        let range = auto_range(&[&low, &high], 0.0, 10.0).expect("a range");
        assert!(range.low < -5.0);
        assert!(range.high > 20.0);
    }

    /// Nothing in the window is not a range of zero, it is no range at all.
    #[test]
    fn a_window_with_no_samples_has_no_range() {
        let series = series_of(&[(0.0, 1.0)]);
        assert!(auto_range(&[&series], 100.0, 110.0).is_none());
        assert!(auto_range(&[], 0.0, 10.0).is_none());
    }

    /// The window is ten seconds and does not roll until it has to.
    #[test]
    fn the_window_holds_still_until_the_trace_reaches_the_edge() {
        assert_eq!(window(0.0), (0.0, 10.0));
        assert_eq!(window(5.0), (0.0, 10.0));
        assert_eq!(window(10.0), (0.0, 10.0));
        assert_eq!(window(12.5), (2.5, 12.5));
        assert_eq!(window(100.0), (90.0, 100.0));
    }

    /// The extent is the lowest and highest time, not the first and last sample's.
    #[test]
    fn the_extent_covers_samples_that_go_back_in_time() {
        let series = series_of(&[(5.0, 1.0), (9.0, 1.0), (2.0, 1.0), (7.0, 1.0)]);
        assert_eq!(series.extent(), Some((2.0, 9.0)));
        assert_eq!(series_of(&[(3.0, 1.0)]).extent(), Some((3.0, 3.0)));
        assert_eq!(Series::new("empty", 1).extent(), None);
    }

    /// Degenerate inputs must not panic; this runs on the flight screen.
    #[test]
    fn the_reduction_refuses_impossible_windows_rather_than_panicking() {
        let series = series_of(&[(0.0, 1.0)]);
        assert!(reduce(&series, 0.0, 10.0, 0).is_empty());
        assert!(reduce(&series, 10.0, 0.0, 10).is_empty());
        assert!(reduce(&series, 5.0, 5.0, 10).is_empty());
        assert!(reduce(&Series::new("empty", 10), 0.0, 10.0, 10).is_empty());
    }
}
