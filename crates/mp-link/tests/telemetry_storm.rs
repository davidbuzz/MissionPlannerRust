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

//! What a render pass costs while telemetry is pouring in (DELIVERABLES.md D10).
//!
//! The deliverable asks for no UI stall over 8 ms during a 200 Hz telemetry storm. The part of
//! that which can be measured without a window - and the part the design rests on - is the
//! snapshot bus: a render pass takes one immutable snapshot per vehicle per frame and does nothing
//! else that can block. If taking a snapshot stays cheap while the link thread is writing at full
//! rate, the UI cannot stall on telemetry; if it does not, no amount of rendering work will save
//! it.
//!
//! This is a test rather than a benchmark so it runs in CI. The thresholds are generous compared
//! with the measured figures, because a shared machine under load is the environment it has to
//! pass in, and a timing test that fails on a busy runner teaches people to ignore it. For the same
//! reason each sample is the fastest of a few loads taken back to back: a thread descheduled in
//! the middle of one load is the scheduler, not the code, and it does not happen three times in a
//! row, while a load that blocks on the writer blocks every time (PLAN.md §13.4 row 6 has the
//! sample that failed at load 30 and passed alone a minute later).
//!
//! The frame itself - render, layout and paint under the same storm - is measured in the window,
//! by `planner` with `MP_STORM` set (`crates/mp-gui/src/storm.rs`, `tests/gui/storm.gui`), from
//! the storm [`mp_link::testing::Storm`] writes; the last test here holds that storm to its rate.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::time::{Duration, Instant};

use mp_link::testing::{STORM_FRAMES_PER_TICK, Storm, attitude, heartbeat, position};
use mp_link::{Link, LinkConfig};
use mp_transport::Transport;
use mp_transport::testing::Loopback;

/// A frame budget. 8 ms is the deliverable's figure, which is half of a 60 Hz frame.
const FRAME_BUDGET: Duration = Duration::from_millis(8);

/// How many snapshots to take. At 60 Hz this is about eight seconds of rendering.
const SNAPSHOTS: usize = 500;

/// Loads per sample, the fastest of which is the sample.
const TRIES: usize = 3;

#[test]
fn a_render_pass_stays_cheap_while_telemetry_pours_in() {
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    // Wait for the vehicle to exist before measuring, so discovery is not counted as a stall.
    vehicle_side.write_all(&heartbeat(0)).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while link.primary_vehicle().is_none() {
        assert!(Instant::now() < deadline, "no vehicle appeared");
        std::thread::sleep(Duration::from_millis(10));
    }
    let (_, handle) = link.primary_vehicle().expect("a vehicle");

    // A writer running as fast as it can, which is harder than 200 Hz. If the snapshot path holds
    // up against an unthrottled writer it holds up against a real vehicle.
    let writer = std::thread::spawn(move || {
        let mut seq = 0u8;
        let until = Instant::now() + Duration::from_secs(3);
        while Instant::now() < until {
            let roll = f32::from(seq) / 100.0;
            if vehicle_side.write_all(&attitude(seq, roll)).is_err()
                || vehicle_side.write_all(&position(seq)).is_err()
                || vehicle_side.write_all(&heartbeat(seq)).is_err()
            {
                break;
            }
            seq = seq.wrapping_add(1);
        }
    });

    // What a render pass does: take one snapshot and read from it.
    let mut worst = Duration::ZERO;
    let mut total = Duration::ZERO;
    for _ in 0..SNAPSHOTS {
        let elapsed = (0..TRIES)
            .map(|_| {
                let started = Instant::now();
                let state = handle.load();
                // Touch the fields a frame actually reads, so the compiler cannot elide the load.
                let sum = state.attitude.roll.0 + state.altitude_relative.0;
                std::hint::black_box(sum);
                started.elapsed()
            })
            .min()
            .unwrap_or_default();
        worst = worst.max(elapsed);
        total += elapsed;

        // Roughly 60 Hz, so this measures a renderer's pace rather than a tight loop.
        std::thread::sleep(Duration::from_millis(16));
    }

    writer.join().expect("the writer thread should finish");

    let average = total / u32::try_from(SNAPSHOTS).unwrap_or(1);
    assert!(
        worst < FRAME_BUDGET,
        "worst snapshot took {worst:?}, over the {FRAME_BUDGET:?} frame budget \
         (average {average:?})"
    );
    // The design claim is that this is nanoseconds, not milliseconds. A microsecond average would
    // still pass the budget while meaning something had gone badly wrong.
    assert!(
        average < Duration::from_micros(100),
        "average snapshot took {average:?}, which is far above the expected nanoseconds"
    );
}

#[test]
fn snapshots_keep_arriving_while_the_link_is_saturated() {
    // A cheap snapshot that never changes would pass the timing test and be useless. The point of
    // the bus is fresh data, so this asserts the reader actually sees the writer's work.
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    vehicle_side.write_all(&heartbeat(0)).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while link.primary_vehicle().is_none() {
        assert!(Instant::now() < deadline, "no vehicle appeared");
        std::thread::sleep(Duration::from_millis(10));
    }
    let (_, handle) = link.primary_vehicle().expect("a vehicle");

    let mut seen = std::collections::BTreeSet::new();
    for seq in 0..60u8 {
        let roll = f32::from(seq) / 10.0;
        vehicle_side.write_all(&attitude(seq, roll)).unwrap();
        std::thread::sleep(Duration::from_millis(5));
        let state = handle.load();
        // Rounded, because the wire carries f32 and the state keeps radians as f64.
        #[allow(clippy::cast_possible_truncation)] // roll in milliradians fits an i64 easily
        let milliradians = (state.attitude.roll.0 * 1000.0).round() as i64;
        seen.insert(milliradians);
    }

    assert!(
        seen.len() > 10,
        "the snapshot barely changed across 60 updates: {} distinct values",
        seen.len()
    );
}

#[test]
fn a_paced_storm_arrives_through_the_link_at_its_rate() {
    // What `planner` reports as `storm.rate`: frames the link counted, a second, over the frames
    // in each tick. A storm that fell short would make the frame measurement a measurement of a
    // lighter load than D10 names; one that ran unthrottled, of a heavier one.
    let (storm, end) = Storm::start(200);
    let link = Link::from_transport(Box::new(end), LinkConfig::default());
    let deadline = Instant::now() + Duration::from_secs(10);
    while link.primary_vehicle().is_none() {
        assert!(Instant::now() < deadline, "no vehicle appeared");
        std::thread::sleep(Duration::from_millis(10));
    }

    let (frames_before, started) = (link.frames_received(), Instant::now());
    std::thread::sleep(Duration::from_secs(2));
    let frames = link.frames_received() - frames_before;
    let elapsed = started.elapsed().as_secs_f64();
    drop(storm);

    #[allow(clippy::cast_precision_loss)] // a few thousand frames
    let rate = frames as f64 / elapsed / f64::from(STORM_FRAMES_PER_TICK);
    // Generous for a loaded machine: the writer catches up after a deschedule, so a real
    // shortfall is the pacing broken, not the scheduler.
    assert!(
        (170.0..=230.0).contains(&rate),
        "the storm arrived at {rate:.1} Hz, asked for 200 ({frames} frames in {elapsed:.2} s)"
    );
    let (_, handle) = link.primary_vehicle().expect("a vehicle");
    assert!(
        handle.load().position.is_some(),
        "the storm's positions did not reach the vehicle state"
    );
}
