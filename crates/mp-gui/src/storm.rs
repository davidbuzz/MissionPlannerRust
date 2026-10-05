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

//! What a frame costs during a telemetry storm: DELIVERABLES.md Deliverable 10's "no UI stall > 8 ms during
//! a 200 Hz telemetry storm", measured in the window, where the claim is made.
//!
//! `crates/mp-link/tests/telemetry_storm.rs` shows that the link and the snapshot bus hold up
//! under a writer running flat out. It cannot say what a frame costs, because it has no frame.
//! This module measures that half. It is development scaffolding, off unless `MP_STORM` gives a
//! rate, and a normal run does none of it: there is no storm, no timing and no extra element.
//!
//! # The storm
//!
//! With `MP_STORM=200` the application does not open the link it was given, or the one it
//! remembers. Its link runs over an in-memory transport to [`mp_link::testing::Storm`], a vehicle
//! that writes ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD and HEARTBEAT 200 times a second each while
//! flying a circle, so the map, the HUD and every readout change on every frame. The frames pass
//! through the real link thread and the real snapshot bus, and the screens read them as they
//! would read a vehicle's. The screen repaints at display rate ([`REFRESH`]) instead of the
//! usual 10 Hz, which gives enough frames for a 99th percentile to mean something.
//!
//! # What is measured
//!
//! A frame's cost is the time the UI thread spends on it. That runs from the top of `render` to
//! the paint of [`marker`], an empty element placed as the root's last child, so it covers
//! render, layout, prepaint and paint of everything on screen. A stall would happen in that
//! span. Presenting to the GPU afterwards is not counted, because it waits for the display
//! rather than working. The test harness's own work inside a frame is subtracted, because a
//! normal run does none of it: the facts written with `MP_FACTS`, and the probe file written
//! with `MP_PROBE` ([`exclude`]).
//!
//! The gap is the time from one frame's start to the next frame's start. A frame that is late
//! because the thread was busy between frames shows up in the gap.
//!
//! Frames from the first [`WARM_UP`], and any frame drawn before the vehicle is heard, are not
//! counted. Those frames pay once for fonts, glyph atlases and shader pipelines, and they are
//! not steady state under a storm. The facts appear once [`ENOUGH`] frames have been counted,
//! so a script that reads them too early fails on a missing fact rather than passing on three
//! samples:
//!
//! | fact | what |
//! |---|---|
//! | `frame.count` | frames counted |
//! | `frame.p50`, `frame.p99`, `frame.max` | frame cost, whole milliseconds rounded down, so `frame.p99 < 8` holds exactly when the 99th percentile is under 8 ms |
//! | `frame.p50_us`, `frame.p99_us`, `frame.max_us` | the same in microseconds, for the record |
//! | `frame.stalls` | frames that cost more than [`STALL`] |
//! | `frame.gap.p50_us`, `frame.gap.p99_us`, `frame.gap.max_us` | the gap, in microseconds |
//! | `storm.rate` | the storm as it arrived, in Hz: the link's frame counter, per second, over the four frames each tick writes |
//! | `storm.frames` | frames the link counted while the frames were measured |
//! | `storm.latency.count` | packets whose journey to the screen was measured |
//! | `storm.latency.p99` | packet-to-pixel, whole milliseconds rounded down, so `storm.latency.p99 < 16` holds exactly when the 99th percentile is under D9's 16 ms |
//! | `storm.latency.p50_us`, `storm.latency.p99_us`, `storm.latency.max_us` | the same in microseconds |
//! | `storm.latency.over` | packets that took more than [`LATENCY_BUDGET`] |
//!
//! # Packet to pixel
//!
//! DELIVERABLES.md Deliverable 9's "< 16 ms packet-to-pixel at the 99th percentile" runs from a packet
//! arriving at the link to the frame showing it being presented. The storm's link is asked to
//! stamp arrivals (`LinkConfig::stamp_arrivals`), so the snapshot a frame reads carries when its
//! newest packet came in (`VehicleState::packet_in`); the product's links never are, and the
//! field stays empty. The frame hands the stamp to [`marker`], and the measurement ends when
//! gpui has handed the frame to the platform: its `draw` and `present` run inside one update of
//! the application, and a callback deferred from the marker's paint runs when that update ends.
//! So the journey covers the link's read and decode, the wait for the next snapshot publish
//! (every 5 ms, `mp_link::DEFAULT_PUBLISH_INTERVAL`), the wait for the next frame, and the
//! frame's render, layout, paint and present. The harness's own work inside the frame is subtracted, as it is from a frame's cost.
//!
//! Each packet is measured once, at the first frame that shows it: a frame drawn before the
//! next snapshot shows the same packet again, and its age then is not a packet's latency.
//!
//! `tests/gui/storm.gui` runs it.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::RefCell;
use std::sync::OnceLock;
use web_time::{Duration, Instant};

use gpui::{IntoElement, Styled as _};
use mp_link::testing::{STORM_FRAMES_PER_TICK, Storm};
use mp_link::{Link, LinkConfig};

use crate::telemetry::Telemetry;

/// The repaint interval during a storm: about 60 Hz, the display rate. The 8 ms budget is half
/// of one of these frames.
pub const REFRESH: Duration = Duration::from_millis(16);

/// A frame costing more than this is a stall: Deliverable 10's figure.
pub const STALL: Duration = Duration::from_millis(8);

/// Packet-to-pixel at the 99th percentile must be under this: Deliverable 9's figure.
pub const LATENCY_BUDGET: Duration = Duration::from_millis(16);

/// Frames drawn in this long after the first are not counted.
pub const WARM_UP: Duration = Duration::from_secs(2);

/// Frames counted before any fact is published.
pub const ENOUGH: usize = 100;

/// Once there are enough frames, the facts are refreshed every this many frames. The sort behind
/// a percentile is done outside the measured part of a frame, but it is still the UI thread's
/// time.
const PUBLISH_EVERY: usize = 10;

/// Frames counted at most: five minutes at 60 Hz. After that the facts stay as they were, so a
/// storm left running does not grow a sample buffer forever.
const MAX_FRAMES: usize = 18_000;

/// The highest rate `MP_STORM` accepts. A rate above this is a typo.
const MAX_RATE: u32 = 5_000;

/// A rate from the value of `MP_STORM`: a whole number of ticks a second, from 1 to [`MAX_RATE`].
fn rate_from(value: Option<&str>) -> Option<u32> {
    let rate: u32 = value?.trim().parse().ok()?;
    (1..=MAX_RATE).contains(&rate).then_some(rate)
}

/// The storm's rate, or `None` when there is no storm.
#[must_use]
pub fn rate() -> Option<u32> {
    static RATE: OnceLock<Option<u32>> = OnceLock::new();
    *RATE.get_or_init(|| {
        let value = std::env::var("MP_STORM").ok();
        let rate = rate_from(value.as_deref());
        if rate.is_none()
            && let Some(value) = value
        {
            eprintln!(
                "MP_STORM should be a rate in Hz from 1 to {MAX_RATE}, not {value:?}; no storm"
            );
        }
        rate
    })
}

/// Whether a storm is running, and so whether frames are measured.
#[must_use]
pub fn enabled() -> bool {
    rate().is_some()
}

/// A storm at `rate`, and the telemetry the screens read it through.
///
/// The storm has to be kept: dropping it stops the vehicle.
#[must_use]
pub fn source(rate: u32) -> (Storm, Telemetry) {
    let (storm, end) = Storm::start(rate);
    // Stamped, so a frame can tell when the packet it shows arrived.
    let config = LinkConfig {
        stamp_arrivals: true,
        ..LinkConfig::default()
    };
    let link = Link::from_transport(Box::new(end), config);
    (storm, Telemetry::over(link, &format!("storm:{rate}")))
}

/// The storm's telemetry when `MP_STORM` asks for one, in place of the link the application would
/// have opened. The storm runs for as long as the process does.
#[must_use]
pub fn telemetry() -> Option<Telemetry> {
    static RUNNING: OnceLock<Storm> = OnceLock::new();
    let (storm, telemetry) = source(rate()?);
    // Set once; a second call's storm is dropped, and stops.
    let _ = RUNNING.set(storm);
    Some(telemetry)
}

/// Where the 99th and other percentiles come from: every counted frame's cost and gap.
#[derive(Debug, Default, Clone)]
pub struct Meter {
    costs: Vec<Duration>,
    gaps: Vec<Duration>,
    stalls: usize,
}

/// A meter's findings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Summary {
    /// Frames counted.
    pub count: usize,
    /// Median frame cost.
    pub p50: Duration,
    /// 99th percentile frame cost.
    pub p99: Duration,
    /// Worst frame cost.
    pub max: Duration,
    /// Frames costing more than [`STALL`].
    pub stalls: usize,
    /// Median gap between frame starts.
    pub gap_p50: Duration,
    /// 99th percentile gap.
    pub gap_p99: Duration,
    /// Longest gap.
    pub gap_max: Duration,
}

/// The `percent`th percentile of `sorted`, by nearest rank: the smallest value with at least
/// that share of the values at or below it. Zero for no values.
#[must_use]
pub fn percentile(sorted: &[Duration], percent: usize) -> Duration {
    let rank = (percent * sorted.len()).div_ceil(100).max(1);
    sorted.get(rank - 1).copied().unwrap_or_default()
}

impl Meter {
    /// Counts one frame: what it cost, and the gap since the one before, if there was one.
    /// Returns whether it was counted; past [`MAX_FRAMES`] it is not.
    pub fn record(&mut self, cost: Duration, gap: Option<Duration>) -> bool {
        if self.costs.len() >= MAX_FRAMES {
            return false;
        }
        if cost > STALL {
            self.stalls += 1;
        }
        self.costs.push(cost);
        self.gaps.extend(gap);
        true
    }

    /// Frames counted.
    #[must_use]
    pub fn count(&self) -> usize {
        self.costs.len()
    }

    /// The percentiles, the worst and the stalls.
    #[must_use]
    pub fn summary(&self) -> Summary {
        let mut costs = self.costs.clone();
        costs.sort_unstable();
        let mut gaps = self.gaps.clone();
        gaps.sort_unstable();
        Summary {
            count: costs.len(),
            p50: percentile(&costs, 50),
            p99: percentile(&costs, 99),
            max: costs.last().copied().unwrap_or_default(),
            stalls: self.stalls,
            gap_p50: percentile(&gaps, 50),
            gap_p99: percentile(&gaps, 99),
            gap_max: gaps.last().copied().unwrap_or_default(),
        }
    }
}

/// Every measured packet's journey from the link to the screen.
#[derive(Debug, Default, Clone)]
pub struct Latencies {
    samples: Vec<Duration>,
    over: usize,
}

/// What the latencies came to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LatencySummary {
    /// Packets measured.
    pub count: usize,
    /// Median packet-to-pixel.
    pub p50: Duration,
    /// 99th percentile.
    pub p99: Duration,
    /// Slowest.
    pub max: Duration,
    /// Packets slower than [`LATENCY_BUDGET`].
    pub over: usize,
}

impl Latencies {
    /// Counts one packet's journey. Returns whether it was counted; past [`MAX_FRAMES`] it is not.
    pub fn record(&mut self, latency: Duration) -> bool {
        if self.samples.len() >= MAX_FRAMES {
            return false;
        }
        if latency > LATENCY_BUDGET {
            self.over += 1;
        }
        self.samples.push(latency);
        true
    }

    /// The percentiles, the slowest and the count over budget.
    #[must_use]
    pub fn summary(&self) -> LatencySummary {
        let mut sorted = self.samples.clone();
        sorted.sort_unstable();
        LatencySummary {
            count: sorted.len(),
            p50: percentile(&sorted, 50),
            p99: percentile(&sorted, 99),
            max: sorted.last().copied().unwrap_or_default(),
            over: self.over,
        }
    }
}

/// The storm's rate as it arrived: `frames` counted by the link over `elapsed`, a second, over the
/// frames in each tick. Rounded down, so `storm.rate >= 190` holds exactly when it was.
#[must_use]
pub fn arrived_rate(frames: u64, elapsed: Duration) -> u128 {
    let per_tick_micros = elapsed.as_micros() * u128::from(STORM_FRAMES_PER_TICK);
    (u128::from(frames) * 1_000_000)
        .checked_div(per_tick_micros)
        .unwrap_or(0)
}

/// How many facts [`facts`] gives.
const FACT_COUNT: usize = 19;

/// The facts for a summary, the packets' journeys and the storm behind them.
#[must_use]
pub fn facts(
    summary: &Summary,
    latency: &LatencySummary,
    link_frames: u64,
    elapsed: Duration,
) -> [(&'static str, u128); FACT_COUNT] {
    [
        ("frame.count", summary.count as u128),
        ("frame.p50", summary.p50.as_millis()),
        ("frame.p99", summary.p99.as_millis()),
        ("frame.max", summary.max.as_millis()),
        ("frame.p50_us", summary.p50.as_micros()),
        ("frame.p99_us", summary.p99.as_micros()),
        ("frame.max_us", summary.max.as_micros()),
        ("frame.stalls", summary.stalls as u128),
        ("frame.gap.p50_us", summary.gap_p50.as_micros()),
        ("frame.gap.p99_us", summary.gap_p99.as_micros()),
        ("frame.gap.max_us", summary.gap_max.as_micros()),
        ("storm.rate", arrived_rate(link_frames, elapsed)),
        ("storm.frames", u128::from(link_frames)),
        ("storm.latency.count", latency.count as u128),
        ("storm.latency.p99", latency.p99.as_millis()),
        ("storm.latency.p50_us", latency.p50.as_micros()),
        ("storm.latency.p99_us", latency.p99.as_micros()),
        ("storm.latency.max_us", latency.max.as_micros()),
        ("storm.latency.over", latency.over as u128),
    ]
}

/// The frame being drawn, and the frames counted so far.
#[derive(Debug, Default)]
struct Clock {
    /// When the frame being drawn began, until its marker is painted.
    started: Option<Instant>,
    /// The gap from the frame before to this one.
    gap: Option<Duration>,
    /// The test harness's time inside this frame.
    harness: Duration,
    /// When the frame before began.
    previous: Option<Instant>,
    /// When the first frame began, which the warm-up counts from.
    first: Option<Instant>,
    /// When counting began, and the link's frame count then.
    baseline: Option<(Instant, u64)>,
    /// The last frame counted: when it began and the link's frame count.
    last: Option<(Instant, u64)>,
    meter: Meter,
    /// The counted frame painted and waiting to be presented, if it shows a packet not measured
    /// yet: when that packet arrived at the link, and the harness's time inside the frame.
    presenting: Option<(Instant, Duration)>,
    /// The packet the last presented frame showed, which a later frame showing it again does
    /// not measure.
    last_packet: Option<Instant>,
    latencies: Latencies,
}

impl Clock {
    /// A frame begins.
    fn begin(&mut self, now: Instant) {
        self.first.get_or_insert(now);
        self.gap = self
            .previous
            .replace(now)
            .map(|previous| now.saturating_duration_since(previous));
        self.started = Some(now);
        self.harness = Duration::ZERO;
    }

    /// Harness work inside the frame, which is not the frame's cost.
    fn exclude(&mut self, spent: Duration) {
        if self.started.is_some() {
            self.harness += spent;
        }
    }

    /// The frame's marker is painted, with the link's frame count and the arrival of the newest
    /// packet in the snapshot as the frame read them. Returns whether the facts are due.
    fn end(&mut self, now: Instant, link_frames: u64, packet_in: Option<Instant>) -> bool {
        self.presenting = None;
        let Some(started) = self.started.take() else {
            return false;
        };
        let warm = self
            .first
            .is_some_and(|first| started.saturating_duration_since(first) >= WARM_UP);
        if !warm || link_frames == 0 {
            return false;
        }
        let cost = now
            .saturating_duration_since(started)
            .saturating_sub(self.harness);
        if !self.meter.record(cost, self.gap) {
            return false;
        }
        self.presenting = packet_in
            .filter(|packet| self.last_packet != Some(*packet))
            .map(|packet| (packet, self.harness));
        self.baseline.get_or_insert((started, link_frames));
        self.last = Some((started, link_frames));
        let count = self.meter.count();
        count >= ENOUGH && (count - ENOUGH).is_multiple_of(PUBLISH_EVERY)
    }

    /// Harness work after the frame's marker and before it is presented - the facts written as
    /// it ends - which is not the packet's journey.
    fn exclude_presenting(&mut self, spent: Duration) {
        if let Some((_, harness)) = self.presenting.as_mut() {
            *harness += spent;
        }
    }

    /// The frame painted last has been presented: the packet it shows has reached the screen.
    fn presented(&mut self, now: Instant) {
        if let Some((packet, harness)) = self.presenting.take() {
            let latency = now
                .saturating_duration_since(packet)
                .saturating_sub(harness);
            if self.latencies.record(latency) {
                self.last_packet = Some(packet);
            }
        }
    }

    /// The facts, as of the last frame counted.
    fn facts(&self) -> Option<[(&'static str, u128); FACT_COUNT]> {
        let (since, frames_then) = self.baseline?;
        let (at, frames_now) = self.last?;
        Some(facts(
            &self.meter.summary(),
            &self.latencies.summary(),
            frames_now.saturating_sub(frames_then),
            at.saturating_duration_since(since),
        ))
    }
}

thread_local! {
    /// The UI thread's clock. Thread-local because frames are drawn on one thread, and so the
    /// measurement needs no lock.
    static CLOCK: RefCell<Clock> = RefCell::new(Clock::default());
}

/// A frame begins: called first thing in `render`.
pub fn frame_started() {
    if !enabled() {
        return;
    }
    let now = Instant::now();
    CLOCK.with_borrow_mut(|clock| clock.begin(now));
}

/// Time the test harness spent inside the frame being drawn, to leave out of its cost.
pub fn exclude(spent: Duration) {
    if !enabled() {
        return;
    }
    CLOCK.with_borrow_mut(|clock| clock.exclude(spent));
}

/// The element that ends a frame's measurement, for the root's last child; `None` without a
/// storm, so a normal run's element tree is untouched.
///
/// Positioned absolutely and empty, so it takes no part in the layout it is measuring.
/// `link_frames` is the link's frame count as this frame read it, for `storm.rate`, and
/// `packet_in` when the newest packet in the snapshot it read arrived at the link
/// (`VehicleState::packet_in`), for the packet-to-pixel latency.
#[must_use]
pub fn marker(link_frames: u64, packet_in: Option<Instant>) -> Option<impl IntoElement> {
    enabled().then(|| {
        gpui::canvas(
            |_bounds, _window, _cx| (),
            move |_bounds, (), _window, cx| painted(link_frames, packet_in, cx),
        )
        .absolute()
    })
}

/// The marker is painted: the frame's work is done, and it is presented once the update that
/// draws it ends.
fn painted(link_frames: u64, packet_in: Option<Instant>, cx: &mut gpui::App) {
    let now = Instant::now();
    let presenting = CLOCK.with_borrow_mut(|clock| {
        if clock.end(now, link_frames, packet_in)
            && crate::facts::enabled()
            && let Some(facts) = clock.facts()
        {
            let writing = Instant::now();
            for (key, value) in facts {
                crate::facts::record(key, value);
            }
            clock.exclude_presenting(writing.elapsed());
        }
        clock.presenting.is_some()
    });
    // gpui draws the frame and presents it inside one update of the application, and runs what
    // was deferred when the update ends: after the present.
    if presenting {
        cx.defer(|_| {
            let now = Instant::now();
            CLOCK.with_borrow_mut(|clock| clock.presented(now));
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    #[test]
    fn the_percentiles_and_the_stalls_of_a_known_sequence() {
        // 1 ms to 100 ms, shuffled so the meter has to sort: the median is 50, the 99th is 99,
        // and 9 ms to 100 ms - 92 frames - are over the 8 ms budget.
        let mut meter = Meter::default();
        for n in (1..=100).map(|n| (n * 37) % 100 + 1) {
            assert!(meter.record(ms(n), Some(ms(16))));
        }
        let summary = meter.summary();
        assert_eq!(summary.count, 100);
        assert_eq!(summary.p50, ms(50));
        assert_eq!(summary.p99, ms(99));
        assert_eq!(summary.max, ms(100));
        assert_eq!(summary.stalls, 92);
        assert_eq!(summary.gap_p99, ms(16));
    }

    #[test]
    fn a_frame_of_exactly_the_budget_is_not_a_stall() {
        // "Over 8 ms": 8 ms is inside the budget, a microsecond more is not.
        let mut meter = Meter::default();
        meter.record(ms(8), None);
        meter.record(ms(8) + Duration::from_micros(1), None);
        assert_eq!(meter.summary().stalls, 1);
    }

    #[test]
    fn a_percentile_is_a_value_that_was_measured() {
        // Nearest rank, not interpolation: the 99th of ten frames is the worst of them, and of
        // one frame it is that frame.
        let ten: Vec<Duration> = (1..=10).map(ms).collect();
        assert_eq!(percentile(&ten, 99), ms(10));
        assert_eq!(percentile(&ten, 50), ms(5));
        assert_eq!(percentile(&[ms(3)], 99), ms(3));
        assert_eq!(percentile(&[], 99), Duration::ZERO);
    }

    #[test]
    fn milliseconds_are_rounded_down_so_under_eight_means_under_eight() {
        let summary = Summary {
            count: 100,
            p50: Duration::from_micros(1_234),
            p99: Duration::from_micros(7_999),
            max: Duration::from_micros(8_000),
            stalls: 0,
            gap_p50: ms(16),
            gap_p99: ms(17),
            gap_max: ms(20),
        };
        let facts = facts(
            &summary,
            &LatencySummary::default(),
            8_000,
            Duration::from_secs(10),
        );
        let value = |key: &str| facts.iter().find(|(k, _)| *k == key).map(|(_, v)| *v);
        assert_eq!(value("frame.p99"), Some(7));
        assert_eq!(value("frame.p99_us"), Some(7_999));
        assert_eq!(value("frame.max"), Some(8));
        assert_eq!(value("frame.p50"), Some(1));
        assert_eq!(value("frame.gap.p99_us"), Some(17_000));
        // 8,000 frames in ten seconds is 800 a second, and four to a tick: 200 Hz.
        assert_eq!(value("storm.rate"), Some(200));
        assert_eq!(value("storm.frames"), Some(8_000));
    }

    #[test]
    fn the_arrived_rate_rounds_down_and_survives_no_time() {
        assert_eq!(arrived_rate(7_599, Duration::from_secs(10)), 189);
        assert_eq!(arrived_rate(7_600, Duration::from_secs(10)), 190);
        assert_eq!(arrived_rate(100, Duration::ZERO), 0);
    }

    #[test]
    fn a_frame_costs_from_render_to_marker_less_the_harness() {
        let t0 = Instant::now();
        let mut clock = Clock::default();
        // A frame long after warm-up, whose facts file took 3 ms of its 5.
        clock.begin(t0);
        clock.end(t0 + ms(1), 10, None);
        let start = t0 + WARM_UP;
        clock.begin(start);
        clock.exclude(ms(3));
        clock.end(start + ms(5), 10, None);
        let summary = clock.meter.summary();
        assert_eq!(summary.count, 1);
        assert_eq!(summary.max, ms(2));
        // The gap is start to start, harness and all.
        assert_eq!(summary.gap_max, WARM_UP);
    }

    #[test]
    fn warm_up_and_frames_before_the_vehicle_are_not_counted() {
        let t0 = Instant::now();
        let mut clock = Clock::default();
        // Slow first frames, as shaders compile: not steady state, not counted.
        for n in 0..10 {
            let start = t0 + ms(100 * n);
            clock.begin(start);
            clock.end(start + ms(50), 1, None);
        }
        // Past the warm-up, but nothing heard from the vehicle yet.
        clock.begin(t0 + WARM_UP);
        clock.end(t0 + WARM_UP + ms(50), 0, None);
        assert_eq!(clock.meter.count(), 0);
        // A marker painted with no frame begun counts nothing either.
        assert!(!clock.end(t0 + WARM_UP + ms(60), 5, None));
        assert_eq!(clock.meter.count(), 0);
    }

    #[test]
    fn the_facts_wait_for_enough_frames_and_report_the_storm() {
        let t0 = Instant::now();
        let mut clock = Clock::default();
        clock.begin(t0);
        clock.end(t0, 0, None);
        // 60 Hz frames of 2 ms each, with the link counting 800 frames a second.
        let mut due = Vec::new();
        for n in 0..=(ENOUGH + PUBLISH_EVERY) {
            let offset = ms(16) * u32::try_from(n).expect("small");
            let start = t0 + WARM_UP + offset;
            let link_frames = 1_000 + u64::try_from(offset.as_millis() * 8 / 10).expect("small");
            clock.begin(start);
            if clock.end(start + ms(2), link_frames, None) {
                due.push(clock.meter.count());
            }
        }
        assert_eq!(due, [ENOUGH, ENOUGH + PUBLISH_EVERY]);
        let facts = clock.facts().expect("facts once enough frames are counted");
        let value = |key: &str| facts.iter().find(|(k, _)| *k == key).map(|(_, v)| *v);
        assert_eq!(value("frame.count"), Some(111));
        assert_eq!(value("frame.p99_us"), Some(2_000));
        assert_eq!(value("frame.stalls"), Some(0));
        assert_eq!(value("frame.gap.p50_us"), Some(16_000));
        assert_eq!(value("storm.rate"), Some(200));
    }

    #[test]
    fn a_rate_is_a_whole_number_of_hertz_or_no_storm() {
        assert_eq!(rate_from(Some("200")), Some(200));
        assert_eq!(rate_from(Some(" 50 ")), Some(50));
        assert_eq!(rate_from(Some("0")), None);
        assert_eq!(rate_from(Some("fast")), None);
        assert_eq!(rate_from(Some("200.5")), None);
        assert_eq!(rate_from(Some("1000000")), None);
        assert_eq!(rate_from(None), None);
    }

    #[test]
    fn without_the_switch_there_is_no_storm_and_nothing_measured() {
        // `MP_STORM` is not set under `cargo test`. The cost of the measurement in a normal run
        // has to be nothing: no storm, no element and no clock.
        if std::env::var_os("MP_STORM").is_some() {
            return;
        }
        assert!(!enabled());
        assert!(telemetry().is_none());
        assert!(marker(1_000, None).is_none());
        frame_started();
        exclude(ms(1));
        CLOCK.with_borrow(|clock| assert!(clock.started.is_none() && clock.first.is_none()));
    }

    #[test]
    fn the_script_asks_for_facts_this_publishes_against_d10s_numbers() {
        // `tests/gui/storm.gui` is run by hand, with a window; this is the part of it that can be
        // checked without one. A renamed fact would fail it only once somebody ran it.
        let script = include_str!("../../../tests/gui/storm.gui");
        let summary = Meter::default().summary();
        let published: Vec<&str> = facts(&summary, &LatencySummary::default(), 0, Duration::ZERO)
            .iter()
            .map(|(key, _)| *key)
            .collect();
        let expects: Vec<Vec<&str>> = script
            .lines()
            .map(|line| line.split('#').next().unwrap_or_default())
            .map(|line| line.split_whitespace().collect::<Vec<_>>())
            .filter(|words| words.first() == Some(&"expect"))
            .collect();
        for words in &expects {
            let key = words[1];
            if key.starts_with("frame.") || key.starts_with("storm.") {
                assert!(
                    published.contains(&key),
                    "the script expects {key}, never published"
                );
            }
        }
        let has = |line: &[&str]| expects.iter().any(|words| words[1..] == *line);
        assert!(script.lines().any(|line| line.trim() == "env MP_STORM 200"));
        assert!(has(&["frame.stalls", "0"]));
        assert!(has(&["frame.p99", "<", "8"]));
        assert_eq!(STALL, ms(8));
        assert!(has(&["storm.rate", ">=", "190"]));
        assert!(has(&["frame.count", ">=", &ENOUGH.to_string()]));
        // Deliverable 9: packet-to-pixel under 16 ms at the 99th percentile, from enough packets.
        assert!(has(&["storm.latency.p99", "<", "16"]));
        assert_eq!(LATENCY_BUDGET, ms(16));
        assert!(has(&["storm.latency.count", ">=", "50"]));
    }

    /// A frame long after warm-up, to measure latency on.
    fn warm_clock(t0: Instant) -> Clock {
        let mut clock = Clock::default();
        clock.begin(t0);
        clock.end(t0 + ms(1), 10, None);
        clock
    }

    #[test]
    fn a_packet_is_timed_from_the_link_to_the_present_less_the_harness() {
        // The packet arrived 3 ms before the frame began; the frame took 5 ms, 1 of it the
        // harness's; its facts took 1 ms more after the marker; it was presented 2 ms after the
        // marker. The packet's journey: 3 + 5 + 2 less the harness's 2.
        let t0 = Instant::now();
        let mut clock = warm_clock(t0);
        let start = t0 + WARM_UP;
        clock.begin(start);
        clock.exclude(ms(1));
        clock.end(start + ms(5), 20, Some(start - ms(3)));
        clock.exclude_presenting(ms(1));
        clock.presented(start + ms(7));
        let summary = clock.latencies.summary();
        assert_eq!(summary.count, 1);
        assert_eq!(summary.max, ms(8));
        assert_eq!(summary.over, 0);
        // Presenting again measures nothing more.
        clock.presented(start + ms(9));
        assert_eq!(clock.latencies.summary().count, 1);
    }

    #[test]
    fn a_packet_is_measured_once_at_the_first_frame_that_shows_it() {
        let t0 = Instant::now();
        let mut clock = warm_clock(t0);
        let packet = t0 + WARM_UP;
        for n in 0..3 {
            // Three frames 16 ms apart, all showing the one packet: only the first counts.
            let start = packet + ms(1 + 16 * n);
            clock.begin(start);
            clock.end(start + ms(2), 20, Some(packet));
            clock.presented(start + ms(3));
        }
        assert_eq!(clock.latencies.summary().count, 1);
        assert_eq!(clock.latencies.summary().max, ms(4));
        // A new packet is measured.
        let start = packet + ms(60);
        clock.begin(start);
        clock.end(start + ms(2), 20, Some(start - ms(15)));
        clock.presented(start + ms(3));
        let summary = clock.latencies.summary();
        assert_eq!(summary.count, 2);
        assert_eq!(summary.max, ms(18));
        assert_eq!(summary.over, 1, "18 ms is over the 16 ms budget");
    }

    #[test]
    fn no_latency_without_a_stamp_or_before_warm_up() {
        let t0 = Instant::now();
        let mut clock = Clock::default();
        // Before warm-up: not counted, though stamped.
        clock.begin(t0);
        clock.end(t0 + ms(2), 10, Some(t0));
        clock.presented(t0 + ms(3));
        // Warm, but the snapshot carries no stamp: a link that was not asked for one.
        let start = t0 + WARM_UP;
        clock.begin(start);
        clock.end(start + ms(2), 10, None);
        clock.presented(start + ms(3));
        // A marker painted with no frame begun measures nothing either.
        clock.end(start + ms(4), 10, Some(start));
        clock.presented(start + ms(5));
        assert_eq!(clock.latencies.summary().count, 0);
    }

    #[test]
    fn the_latency_facts_round_down_so_under_sixteen_means_under_sixteen() {
        let mut latencies = Latencies::default();
        for micros in [2_000, 9_000, 15_999] {
            assert!(latencies.record(Duration::from_micros(micros)));
        }
        let latency = latencies.summary();
        let facts = facts(&Meter::default().summary(), &latency, 0, Duration::ZERO);
        let value = |key: &str| facts.iter().find(|(k, _)| *k == key).map(|(_, v)| *v);
        assert_eq!(value("storm.latency.count"), Some(3));
        assert_eq!(value("storm.latency.p99"), Some(15));
        assert_eq!(value("storm.latency.p99_us"), Some(15_999));
        assert_eq!(value("storm.latency.p50_us"), Some(9_000));
        assert_eq!(value("storm.latency.max_us"), Some(15_999));
        assert_eq!(value("storm.latency.over"), Some(0));
        latencies.record(LATENCY_BUDGET + Duration::from_micros(1));
        assert_eq!(latencies.summary().over, 1);
        assert_eq!(latencies.summary().p99.as_millis(), 16);
    }

    // Unix only: tools/gui-test.sh is the Linux and macOS runner (Windows runs the scripts with
    // tools/win10/gui-test.ps1), and on Windows the `bash` a process finds first can be the WSL
    // launcher with no Linux behind it, which fails every call without running the comparison
    // (the hosted runner, 2026-10-04).
    #[cfg(unix)]
    #[test]
    fn the_test_runner_compares_less_than_as_the_script_needs() {
        // `expect frame.p99 < 8` is the runner's only `<`. Its comparison is run here, as the
        // runner defines it, rather than trusted.
        let runner = include_str!("../../../tools/gui-test.sh");
        assert!(
            runner.contains(r#"[ "$OP" = "<" ]"#),
            "the runner does not dispatch `<` to its comparison"
        );
        let start = runner.find("holds() {").expect("the runner's comparison");
        let length = runner[start..].find("\n}\n").expect("its end") + 2;
        let holds = &runner[start..start + length];
        let check = |args: &str| {
            std::process::Command::new("bash")
                .arg("-c")
                .arg(format!("{holds}\nholds {args}"))
                .status()
                .map(|status| status.success())
        };
        let Ok(true) = check("7 '<' 8") else {
            // No bash to run it with is not a failure of the comparison; say so rather than pass.
            if check("1 '<' 2").is_err() {
                eprintln!("skipped: no bash to run the runner's comparison with");
                return;
            }
            panic!("7 < 8 does not hold in the runner");
        };
        assert_eq!(check("8 '<' 8").ok(), Some(false), "8 < 8 held");
        assert_eq!(check("9 '<' 8").ok(), Some(false), "9 < 8 held");
        assert_eq!(
            check("none '<' 8").ok(),
            Some(false),
            "a value that is not a number held"
        );
        assert_eq!(check("-1 '<' 0").ok(), Some(true), "-1 < 0 did not hold");
    }

    #[test]
    fn the_storm_reaches_the_screens_through_the_real_link() {
        // What `MP_STORM` puts behind the screens: the vehicle appears, the link counts its
        // frames, and the snapshot the screens read has a position and a moving attitude.
        let (storm, telemetry) = source(200);
        let deadline = Instant::now() + Duration::from_secs(10);
        let mut rolls = std::collections::BTreeSet::new();
        loop {
            let view = telemetry.view();
            if let Some(state) = view.state.as_ref()
                && state.position.is_some()
            {
                #[allow(clippy::cast_possible_truncation)] // milliradians, a few hundred
                rolls.insert((state.attitude.roll.0 * 1000.0).round() as i64);
                if rolls.len() > 3 && view.frames > 40 {
                    break;
                }
            }
            assert!(
                Instant::now() < deadline,
                "the storm did not reach the screens: {} frames, {} rolls",
                view.frames,
                rolls.len()
            );
            wasm_thread::sleep(Duration::from_millis(20));
        }
        assert!(storm.ticks() > 10);
        // The storm's link stamps each packet's arrival into the snapshot, for the latency, and
        // the stamp is recent: 200 Hz packets and a 5 ms publish.
        let stamped = telemetry
            .view()
            .state
            .as_ref()
            .and_then(|state| state.packet_in)
            .expect("the storm's link stamps arrivals");
        let age = Instant::now().saturating_duration_since(stamped);
        assert!(age < Duration::from_secs(1), "{age:?}");
    }
}
