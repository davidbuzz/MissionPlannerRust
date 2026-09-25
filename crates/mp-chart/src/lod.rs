//! The index a [`crate::Series`] keeps so that a frame costs the width of the plot, not the
//! length of the log.
//!
//! Two things make every query the chart asks - the extent, the value range over a window, the
//! min/max of each pixel column - independent of how many samples there are:
//!
//! - **Runs.** A series is cut wherever time goes backwards (a log's clock restarting across a
//!   reboot), and within a run time never decreases, so the samples in any time window are one
//!   contiguous stretch of each run, found by searching rather than by looking at every sample.
//!   The column a sample falls in is a monotonic function of its time - every step of
//!   `((at - from) / span) * columns` rounds monotonically - so each column's samples are a
//!   contiguous stretch too, found the same way, with the very expression the scan uses.
//! - **A min/max pyramid.** Level `k` holds the lowest and highest value of every block of
//!   `FANOUT^(k+1)` samples, so the min/max of any stretch is its ragged ends read sample by
//!   sample and the middle read a block at a time, a few dozen reads a level whatever the length.
//!
//! A column's low and high are therefore the minimum and maximum of exactly the samples the scan
//! would have put in it, and minimum and maximum do not depend on the order they are taken in:
//! the reduction is the scan's to the bit (zero's sign aside, which no pixel can see), which
//! `tests/lod.rs` holds it to.
//!
//! The index follows the series as it rolls: a sample dropped off the front leaves its block's
//! summary stale, but a block is only ever read when every sample in it is inside the stretch
//! asked about, and a stretch never reaches back past the oldest sample held.

use std::collections::VecDeque;

use crate::Sample;

/// How many samples a first-level block summarises, and how many blocks a block of the next
/// level up does.
const FANOUT: u64 = 32;

/// How many runs a series may be cut into before a query is cheaper as a pass over every sample:
/// a clock that goes backwards at every sample is not a timeline, and searching each of its runs
/// would cost more than the scan it replaces.
const MAX_RUNS: usize = 64;

/// One level of the pyramid.
#[derive(Debug, Clone)]
struct Level {
    /// How many samples one block covers.
    size: u64,
    /// The number of the first block held.
    first: u64,
    /// Each block's lowest and highest value.
    blocks: VecDeque<(f64, f64)>,
    /// Each block's first sample's time: the level's thinned copy of the time axis, which a
    /// search reads instead of the samples, so that it touches a few cache lines a level rather
    /// than a cache miss a probe across the whole series.
    starts: VecDeque<f64>,
}

/// The runs and the pyramid of one series.
#[derive(Debug, Clone)]
pub(crate) struct Index {
    /// How many samples have ever been dropped off the front: the number of the oldest held.
    base: u64,
    /// Where each run but the first starts, by sample number.
    runs: VecDeque<u64>,
    /// The pyramid, finest first.
    levels: Vec<Level>,
}

impl Index {
    /// An index for a series that will hold at most `capacity` samples: enough levels that the
    /// coarsest has at most `FANOUT` blocks across the whole series.
    pub(crate) fn new(capacity: usize) -> Self {
        let capacity = u64::try_from(capacity).unwrap_or(u64::MAX);
        let mut levels = Vec::new();
        let mut size = FANOUT;
        // A level whose one block would cover the whole series is never read.
        while size < capacity {
            levels.push(Level {
                size,
                first: 0,
                blocks: VecDeque::new(),
                starts: VecDeque::new(),
            });
            match size.checked_mul(FANOUT) {
                Some(next) => size = next,
                None => break,
            }
        }
        Self {
            base: 0,
            runs: VecDeque::new(),
            levels,
        }
    }

    /// Forgets everything, keeping the levels.
    pub(crate) fn clear(&mut self) {
        self.base = 0;
        self.runs.clear();
        for level in &mut self.levels {
            level.first = 0;
            level.blocks.clear();
            level.starts.clear();
        }
    }

    /// Records a sample appended to the series, which held `held` before it and whose newest was
    /// `previous`.
    pub(crate) fn push(&mut self, held: usize, previous: Option<&Sample>, sample: Sample) {
        let number = self.base + held as u64;
        if previous.is_some_and(|previous| sample.at < previous.at) {
            self.runs.push_back(number);
        }
        for level in &mut self.levels {
            let block = number / level.size;
            let newest = level.first + level.blocks.len() as u64;
            match level.blocks.back_mut() {
                Some(last) if newest - 1 == block => {
                    last.0 = last.0.min(sample.value);
                    last.1 = last.1.max(sample.value);
                }
                Some(_) => {
                    level.blocks.push_back((sample.value, sample.value));
                    level.starts.push_back(sample.at);
                }
                None => {
                    level.first = block;
                    level.blocks.push_back((sample.value, sample.value));
                    level.starts.push_back(sample.at);
                }
            }
        }
    }

    /// Records the oldest sample dropped.
    pub(crate) fn pop_front(&mut self) {
        self.base += 1;
        while self.runs.front().is_some_and(|start| *start <= self.base) {
            self.runs.pop_front();
        }
        for level in &mut self.levels {
            while !level.blocks.is_empty() && (level.first + 1) * level.size <= self.base {
                level.blocks.pop_front();
                level.starts.pop_front();
                level.first += 1;
            }
        }
    }

    /// Whether queries should search the runs rather than scan every sample.
    pub(crate) fn searchable(&self) -> bool {
        self.runs.len() < MAX_RUNS
    }

    /// The first position in `[lo, hi)` whose sample's time `ahead` holds for, or `hi`: `ahead`
    /// must be false and then true across the stretch, as any threshold on time is within a
    /// run.
    ///
    /// Down the pyramid: at each level, the blocks whose first sample lies in the stretch still
    /// open are searched by their first times, which narrows the stretch to between two
    /// neighbouring block starts, one block of the level; at the bottom, at most `FANOUT`
    /// samples are searched. Each level reads a few neighbouring entries of its `starts`.
    ///
    /// Every time and start read is counted into `reads`.
    pub(crate) fn search(
        &self,
        samples: &VecDeque<Sample>,
        lo: usize,
        hi: usize,
        ahead: impl Fn(f64) -> bool,
        reads: &mut usize,
    ) -> usize {
        if lo >= hi {
            return hi;
        }
        // The answer is in [lower, upper]; `ahead` holds at `upper` unless it is `hi`.
        let mut lower = self.base + lo as u64;
        let mut upper = self.base + hi as u64;
        for level in self.levels.iter().rev() {
            // The blocks whose first sample is in [lower, upper).
            let (first, last) = (lower.div_ceil(level.size), upper.div_ceil(level.size));
            if first >= last {
                continue;
            }
            let start = |block: u64| {
                block
                    .checked_sub(level.first)
                    .and_then(|index| usize::try_from(index).ok())
                    .and_then(|index| level.starts.get(index))
                    .copied()
            };
            // The first of them whose start `ahead` holds for.
            let (mut left, mut right) = (first, last);
            while left < right {
                let middle = left + (right - left) / 2;
                *reads += 1;
                let Some(at) = start(middle) else {
                    // Not held, which the invariants rule out: search the samples instead.
                    return partition(samples, lo, hi, |sample| ahead(sample.at), reads);
                };
                if ahead(at) {
                    right = middle;
                } else {
                    left = middle + 1;
                }
            }
            if left < last {
                upper = left * level.size;
            }
            if left > first {
                lower = (left - 1) * level.size + 1;
            }
        }
        let (lower, upper) = (self.position(lower), self.position(upper));
        partition(samples, lower, upper, |sample| ahead(sample.at), reads)
    }

    /// Each run as `[start, end)` positions in a series holding `len` samples, oldest first.
    pub(crate) fn runs(&self, len: usize) -> impl Iterator<Item = (usize, usize)> + '_ {
        let starts = std::iter::once(0).chain(self.runs.iter().map(|start| self.position(*start)));
        let ends = self
            .runs
            .iter()
            .map(|start| self.position(*start))
            .chain(std::iter::once(len));
        starts.zip(ends)
    }

    /// A sample number as a position in the series.
    fn position(&self, number: u64) -> usize {
        usize::try_from(number - self.base).unwrap_or(usize::MAX)
    }

    /// The lowest and highest value of the samples at positions `[lo, hi)`, `None` for none;
    /// every sample and block summary read is counted into `reads`.
    pub(crate) fn min_max(
        &self,
        samples: &VecDeque<Sample>,
        lo: usize,
        hi: usize,
        reads: &mut usize,
    ) -> Option<(f64, f64)> {
        if lo >= hi {
            return None;
        }
        self.pyramid_min_max(samples, lo, hi, reads).or_else(|| {
            *reads += hi - lo;
            scan(samples, lo, hi)
        })
    }

    /// [`Self::min_max`] through the pyramid; `None` if a block it needed is not held, which the
    /// invariants above rule out and the caller answers with a scan.
    fn pyramid_min_max(
        &self,
        samples: &VecDeque<Sample>,
        lo: usize,
        hi: usize,
        reads: &mut usize,
    ) -> Option<(f64, f64)> {
        let mut acc = (f64::INFINITY, f64::NEG_INFINITY);
        let mut fold = |low: f64, high: f64| {
            *reads += 1;
            acc.0 = acc.0.min(low);
            acc.1 = acc.1.max(high);
        };
        // The stretch in sample numbers, and then in blocks of each level in turn.
        let mut a = self.base + lo as u64;
        let mut b = self.base + hi as u64;
        let raw = |from: u64, to: u64, fold: &mut dyn FnMut(f64, f64)| {
            for sample in samples.range(self.position(from)..self.position(to)) {
                fold(sample.value, sample.value);
            }
        };
        let Some(first) = self.levels.first() else {
            raw(a, b, &mut fold);
            return Some(acc);
        };
        let (ca, fb) = (a.div_ceil(first.size), b / first.size);
        if ca >= fb {
            raw(a, b, &mut fold);
            return Some(acc);
        }
        raw(a, ca * first.size, &mut fold);
        raw(fb * first.size, b, &mut fold);
        (a, b) = (ca, fb);
        for (depth, level) in self.levels.iter().enumerate() {
            let block = |number: u64| {
                number
                    .checked_sub(level.first)
                    .and_then(|index| usize::try_from(index).ok())
                    .and_then(|index| level.blocks.get(index))
                    .copied()
            };
            let blocks = |from: u64, to: u64, fold: &mut dyn FnMut(f64, f64)| -> Option<()> {
                for number in from..to {
                    let (low, high) = block(number)?;
                    fold(low, high);
                }
                Some(())
            };
            let coarsest = depth + 1 == self.levels.len();
            let (ca, fb) = (a.div_ceil(FANOUT), b / FANOUT);
            if coarsest || ca >= fb {
                blocks(a, b, &mut fold)?;
                return Some(acc);
            }
            blocks(a, ca * FANOUT, &mut fold)?;
            blocks(fb * FANOUT, b, &mut fold)?;
            (a, b) = (ca, fb);
        }
        Some(acc)
    }
}

/// The lowest and highest value of the samples at positions `[lo, hi)`, one by one.
fn scan(samples: &VecDeque<Sample>, lo: usize, hi: usize) -> Option<(f64, f64)> {
    samples.range(lo..hi).fold(None, |acc, sample| {
        Some(
            acc.map_or((sample.value, sample.value), |(low, high): (f64, f64)| {
                (low.min(sample.value), high.max(sample.value))
            }),
        )
    })
}

/// The first position in `[lo, hi)` whose sample `ahead` holds for, or `hi`: `ahead` must be
/// false and then true across the stretch, as it is for any threshold on time within a run.
///
/// Galloping from `lo`, so a boundary a few samples on costs a few reads, and one far off costs
/// its logarithm.
fn partition(
    samples: &VecDeque<Sample>,
    lo: usize,
    hi: usize,
    ahead: impl Fn(&Sample) -> bool,
    reads: &mut usize,
) -> usize {
    let mut holds = |position: usize| {
        *reads += 1;
        samples.get(position).is_none_or(&ahead)
    };
    let mut left = lo;
    let mut right = hi;
    let mut step = 1usize;
    while left < hi {
        let probe = left.saturating_add(step - 1).min(hi - 1);
        if holds(probe) {
            right = probe;
            break;
        }
        left = probe + 1;
        step = step.saturating_mul(2);
    }
    while left < right {
        let middle = left + (right - left) / 2;
        if holds(middle) {
            right = middle;
        } else {
            left = middle + 1;
        }
    }
    left
}
