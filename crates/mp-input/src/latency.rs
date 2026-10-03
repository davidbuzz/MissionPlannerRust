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

//! A small fixed-bucket latency histogram, for stick-to-wire timing.
//!
//! DELIVERABLES.md Deliverable 15 sets the bar as a percentile - p99 under 5 ms - and a percentile needs a
//! distribution, not an average: an average of 0.2 ms hides the one frame in fifty that waited
//! 40 ms behind a repaint, and that one frame is the pilot's input arriving late. So the thread
//! that sends records every change it delivers here, and the screen can show p50 and p99 over a
//! real device.
//!
//! Fixed buckets rather than a dependency such as `hdrhistogram`: the question is only ever
//! "which side of a few milliseconds", the buckets make that readable at a glance, and recording
//! is a comparison and an increment with no allocation - it runs on the thread flying the
//! aircraft, once per stick movement.

use std::fmt;
use std::time::Duration;

/// Upper edges of the buckets, in microseconds, exclusive. The last bucket holds everything above.
///
/// A 1-2-5 series, so the edges are round numbers a person can read, with 5 ms - the target - as an
/// edge rather than inside a bucket: "under 5 ms" has to be a question the histogram can answer.
const EDGES_US: [u64; 9] = [100, 200, 500, 1_000, 2_000, 5_000, 10_000, 20_000, 50_000];

/// How many buckets there are: one per edge, and one for everything past the last.
const BUCKETS: usize = EDGES_US.len() + 1;

/// How wide the `Display` bars are at their longest.
const BAR_WIDTH: u64 = 40;

/// Counts of latencies in fixed buckets.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LatencyHistogram {
    counts: [u64; BUCKETS],
    total: u64,
    max: Duration,
}

impl LatencyHistogram {
    /// An empty histogram.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            counts: [0; BUCKETS],
            total: 0,
            max: Duration::ZERO,
        }
    }

    /// Adds one sample.
    pub fn record(&mut self, latency: Duration) {
        let bucket = EDGES_US
            .iter()
            .position(|&edge| latency < Duration::from_micros(edge))
            .unwrap_or(EDGES_US.len());
        if let Some(count) = self.counts.get_mut(bucket) {
            *count = count.saturating_add(1);
        }
        self.total = self.total.saturating_add(1);
        self.max = self.max.max(latency);
    }

    /// How many samples have been recorded.
    #[must_use]
    pub const fn count(&self) -> u64 {
        self.total
    }

    /// The longest sample, or `None` if there are none.
    #[must_use]
    pub fn max(&self) -> Option<Duration> {
        (self.total > 0).then_some(self.max)
    }

    /// An upper bound on a percentile, or `None` if there are no samples.
    ///
    /// The upper edge of the bucket the percentile falls in, or the longest sample if that is
    /// smaller - so the true value is never above what this returns. An upper bound rather than an
    /// interpolation because the question it answers is a pass mark: "p99 is at most 0.5 ms" is
    /// something the buckets know, "p99 is 0.37 ms" is something they would be guessing.
    ///
    /// `percent` above 100 is taken as 100.
    #[must_use]
    pub fn percentile(&self, percent: u8) -> Option<Duration> {
        if self.total == 0 {
            return None;
        }
        // The 1-based rank of the sample at this percentile, rounded up: the 99th percentile of
        // 1,000 samples is the 990th, and of 1,001 it is the 991st.
        let rank = self
            .total
            .saturating_mul(u64::from(percent.min(100)))
            .div_ceil(100)
            .max(1);
        let mut seen = 0u64;
        for (bucket, count) in self.counts.iter().enumerate() {
            seen = seen.saturating_add(*count);
            if seen >= rank {
                return Some(
                    EDGES_US
                        .get(bucket)
                        .map_or(self.max, |&edge| Duration::from_micros(edge).min(self.max)),
                );
            }
        }
        Some(self.max)
    }

    /// An upper bound on the median.
    #[must_use]
    pub fn p50(&self) -> Option<Duration> {
        self.percentile(50)
    }

    /// An upper bound on the 99th percentile: the number Deliverable 15 is judged by.
    #[must_use]
    pub fn p99(&self) -> Option<Duration> {
        self.percentile(99)
    }

    /// Each bucket's exclusive upper edge (`None` for the last, which is unbounded) and its count.
    pub fn buckets(&self) -> impl Iterator<Item = (Option<Duration>, u64)> + '_ {
        self.counts.iter().enumerate().map(|(bucket, count)| {
            (
                EDGES_US
                    .get(bucket)
                    .map(|&edge| Duration::from_micros(edge)),
                *count,
            )
        })
    }
}

/// Milliseconds with three decimals, which is microsecond resolution without the unit changing.
fn millis(duration: Duration) -> String {
    format!("{:.3} ms", duration.as_secs_f64() * 1_000.0)
}

/// A bucket edge as a person writes it: `0.1`, `2`, `50`. Integer arithmetic so the label does
/// not depend on how a float happens to round.
fn edge_label(edge_us: u64) -> String {
    let whole = edge_us / 1_000;
    let tenths = (edge_us % 1_000) / 100;
    if tenths == 0 {
        format!("{whole}")
    } else {
        format!("{whole}.{tenths}")
    }
}

impl fmt::Display for LatencyHistogram {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (Some(p50), Some(p99), Some(max)) = (self.p50(), self.p99(), self.max()) else {
            return write!(f, "no samples");
        };
        writeln!(
            f,
            "{} samples: p50 <= {}, p99 <= {}, max {}",
            self.total,
            millis(p50),
            millis(p99),
            millis(max)
        )?;
        let largest = self.counts.iter().copied().max().unwrap_or(0).max(1);
        let last = EDGES_US.last().copied().unwrap_or(0);
        for (bucket, count) in self.counts.iter().enumerate() {
            let label = EDGES_US.get(bucket).map_or_else(
                || format!(">= {} ms", edge_label(last)),
                |&edge| format!("<  {} ms", edge_label(edge)),
            );
            // Rounded up, so a bucket with one sample in it shows a mark rather than looking empty:
            // the rare slow frame is the one this exists to make visible.
            let bar = count.saturating_mul(BAR_WIDTH).div_ceil(largest);
            let bar = "#".repeat(usize::try_from(bar).unwrap_or(0));
            let separator = if bucket + 1 == BUCKETS { "" } else { "\n" };
            write!(f, "  {label:>10} {count:>7} {bar}{separator}")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn micros(us: u64) -> Duration {
        Duration::from_micros(us)
    }

    /// Nothing recorded is nothing claimed. A p99 of zero from an empty histogram would read as a
    /// perfect result on a device that was never moved.
    #[test]
    fn an_empty_histogram_claims_nothing() {
        let histogram = LatencyHistogram::new();
        assert_eq!(histogram.count(), 0);
        assert_eq!(histogram.p50(), None);
        assert_eq!(histogram.p99(), None);
        assert_eq!(histogram.max(), None);
        assert_eq!(histogram.to_string(), "no samples");
    }

    /// Edges are exclusive: exactly 5 ms is not "under 5 ms", which is the side the target needs.
    #[test]
    fn a_sample_on_an_edge_goes_in_the_bucket_above() {
        let mut histogram = LatencyHistogram::new();
        histogram.record(Duration::from_millis(5));
        let counts: Vec<(Option<Duration>, u64)> = histogram.buckets().collect();
        let under_five = counts
            .iter()
            .find(|(edge, _)| *edge == Some(Duration::from_millis(5)))
            .expect("a 5 ms edge");
        assert_eq!(under_five.1, 0, "5 ms is not under 5 ms");
        let under_ten = counts
            .iter()
            .find(|(edge, _)| *edge == Some(Duration::from_millis(10)))
            .expect("a 10 ms edge");
        assert_eq!(under_ten.1, 1);
    }

    /// The 99th percentile of 1,000 samples is the 990th, and the answer moves the moment the
    /// slow tail reaches past it - which is what makes it a pass mark rather than a mood.
    #[test]
    fn the_p99_rank_is_exact_at_the_boundary() {
        let mut fast_enough = LatencyHistogram::new();
        for _ in 0..990 {
            fast_enough.record(micros(300));
        }
        for _ in 0..10 {
            fast_enough.record(Duration::from_millis(30));
        }
        assert_eq!(
            fast_enough.p99(),
            Some(micros(500)),
            "the 990th sample is fast"
        );

        let mut too_slow = LatencyHistogram::new();
        for _ in 0..989 {
            too_slow.record(micros(300));
        }
        for _ in 0..11 {
            too_slow.record(Duration::from_millis(30));
        }
        // Past the 20 ms edge, inside the 50 ms bucket, and reported as the longest sample
        // because that is a tighter bound than the bucket's edge.
        assert_eq!(too_slow.p99(), Some(Duration::from_millis(30)));
    }

    /// A reported percentile is never below the truth, and never above the longest sample.
    #[test]
    fn a_percentile_is_an_upper_bound_clamped_to_the_longest_sample() {
        let mut histogram = LatencyHistogram::new();
        histogram.record(micros(120));
        histogram.record(micros(130));
        // Both are in the "under 0.2 ms" bucket, but nothing took longer than 0.13 ms.
        assert_eq!(histogram.p50(), Some(micros(130)));
        assert_eq!(histogram.p99(), Some(micros(130)));
        assert_eq!(histogram.percentile(0), Some(micros(130)));
        assert_eq!(histogram.percentile(250), histogram.percentile(100));
    }

    /// Past the last edge there is no edge to report, so the longest sample is the bound.
    #[test]
    fn the_open_bucket_reports_the_longest_sample() {
        let mut histogram = LatencyHistogram::new();
        histogram.record(Duration::from_millis(80));
        histogram.record(Duration::from_millis(200));
        assert_eq!(histogram.p99(), Some(Duration::from_millis(200)));
        assert_eq!(histogram.max(), Some(Duration::from_millis(200)));
        let (edge, count) = histogram.buckets().last().expect("an open bucket");
        assert_eq!((edge, count), (None, 2));
    }

    /// The printout has every bucket, a mark for even a single slow sample, and the summary line.
    #[test]
    fn the_display_shows_every_bucket_and_marks_a_lone_outlier() {
        let mut histogram = LatencyHistogram::new();
        for _ in 0..1_000 {
            histogram.record(micros(150));
        }
        histogram.record(Duration::from_millis(12));
        let text = histogram.to_string();
        // 0.2 ms, not 0.15: the bound is the bucket's edge, because the longest sample is not
        // in the median's bucket and so cannot tighten it.
        assert!(text.starts_with("1001 samples: p50 <= 0.200 ms"), "{text}");
        assert!(text.contains("<  0.1 ms"), "{text}");
        assert!(text.contains("<  5 ms"), "{text}");
        assert!(text.contains(">= 50 ms"), "{text}");
        assert_eq!(text.lines().count(), 1 + BUCKETS, "{text}");
        let outlier = text
            .lines()
            .find(|line| line.contains("<  20 ms"))
            .expect("a 20 ms row");
        assert!(
            outlier.ends_with(" 1 #"),
            "one slow frame must show: {outlier}"
        );
    }
}
