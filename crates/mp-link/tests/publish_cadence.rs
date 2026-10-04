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

//! The snapshot cadence: how often the link thread publishes each vehicle's state for a screen
//! to read. A frame reads the newest snapshot, so a cadence longer than the frame shows some
//! frames the snapshot the frame before had shown - which the 20 ms cadence did on a 60 Hz
//! display (`tests/gui/storm.gui`, 2026-09-26: 570 frames, 469 distinct snapshots). The default
//! is now 5 ms, shorter than any frame up to 200 Hz.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use web_time::{Duration, Instant};

use mp_link::testing::{attitude, heartbeat};
use mp_link::{DEFAULT_PUBLISH_INTERVAL, Link, LinkConfig};
use mp_transport::Transport as _;
use mp_transport::testing::Loopback;
use mp_vehicle::VehicleId;

/// A 120 Hz display's frame: the fastest laptop panel in ordinary use.
const FASTEST_FRAME: Duration = Duration::from_micros(8_333);

/// A 60 Hz display's frame, which a screen samples the snapshot at.
const FRAME_60_HZ: Duration = Duration::from_micros(16_667);

#[test]
fn the_default_cadence_is_shorter_than_the_fastest_frame() {
    let config = LinkConfig::default();
    assert_eq!(config.publish_interval, DEFAULT_PUBLISH_INTERVAL);
    assert!(
        config.publish_interval <= FASTEST_FRAME,
        "a {:?} cadence would show a 120 Hz display stale snapshots",
        config.publish_interval
    );
}

/// Samples the snapshot at a 60 Hz display's pace while the vehicle sends attitude at 200 Hz,
/// and counts the samples that show nothing newer than the sample before: the frames that would
/// have repainted a stale state. Also the frame the sampling actually achieved, which a sleep
/// stretches past 16.7 ms by whatever the machine's timer adds.
fn stale_frames(publish_interval: Duration, samples: usize) -> (usize, Duration) {
    let (gcs_side, mut vehicle) = Loopback::pair();
    let config = LinkConfig {
        stream_rate_hz: 0,
        send_heartbeat: false,
        publish_interval,
        ..LinkConfig::default()
    };
    let link = Link::from_transport(Box::new(gcs_side), config);
    vehicle.write_all(&heartbeat(0)).unwrap();

    // The vehicle's 200 Hz, on its own thread, for as long as the sampling takes.
    let sending = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let sender = {
        let sending = sending.clone();
        wasm_thread::spawn(move || {
            let mut roll = 0.0f32;
            while sending.load(std::sync::atomic::Ordering::Relaxed) {
                roll += 0.001;
                vehicle.write_all(&attitude(1, roll)).unwrap();
                wasm_thread::sleep(Duration::from_millis(5));
            }
        })
    };

    let deadline = Instant::now() + Duration::from_secs(10);
    let handle = loop {
        if let Some(handle) = link.vehicle(VehicleId::new(1, 1))
            && handle.load().messages_applied > 2
        {
            break handle;
        }
        assert!(Instant::now() < deadline, "the vehicle never appeared");
        wasm_thread::sleep(Duration::from_millis(2));
    };

    let mut stale = 0;
    let mut last = handle.load().messages_applied;
    let sampling = Instant::now();
    for _ in 0..samples {
        wasm_thread::sleep(FRAME_60_HZ);
        let applied = handle.load().messages_applied;
        if applied <= last {
            stale += 1;
        }
        last = applied;
    }
    let frame = sampling.elapsed() / u32::try_from(samples).unwrap();
    sending.store(false, std::sync::atomic::Ordering::Relaxed);
    sender.join().unwrap();
    (stale, frame)
}

#[test]
fn a_60_hz_screen_sees_a_fresh_snapshot_every_frame_at_the_default_cadence() {
    // The two cadences under the same load, so a busy machine moves both counts and not the
    // comparison: the old 20 ms cadence repeats a snapshot about one frame in six; the default
    // must do clearly better than that, and a quiet machine gives it none at all.
    let samples = 120;
    let twenty = Duration::from_millis(20);
    let (old, frame) = stale_frames(twenty, samples);
    let (now, _) = stale_frames(DEFAULT_PUBLISH_INTERVAL, samples);
    // A 16.7 ms frame against a 20 ms cadence repeats one snapshot in six. A frame the machine's
    // timer stretches repeats fewer - macOS sleeps 16.7 ms as about 22 ms (the owner's Mac,
    // 2026-10-03), where a floor of one in twelve failed one run in three - and a frame longer
    // than the cadence repeats none, so the floor is half of what this run's own frame predicts,
    // and nothing when it predicts nothing.
    let predicted = f64::from(u32::try_from(samples).unwrap())
        * (1.0 - frame.as_secs_f64() / twenty.as_secs_f64());
    if predicted >= 2.0 {
        assert!(
            f64::from(u32::try_from(old).unwrap()) >= predicted / 2.0,
            "the 20 ms cadence should repeat snapshots at a {frame:?} frame, about {predicted:.0} \
             of {samples}; only {old} did"
        );
        // The comparison the cadence was changed for.
        assert!(
            now * 4 < old.max(1),
            "the default cadence repeated {now} of {samples} snapshots against the 20 ms \
             cadence's {old}"
        );
    } else {
        // A frame longer than the 20 ms cadence shows nothing about either: a repeat here means
        // the machine starved the link thread, not that the cadence was too slow, and the counts
        // are then noise on both sides (the hosted macOS runner, 2026-10-03: a 75 ms frame, 15
        // repeats at 20 ms and 10 at the default).
        println!(
            "a {frame:?} frame is no shorter than the 20 ms cadence here, so neither cadence is \
             judged: {old} repeats at 20 ms, {now} at the default, of {samples}"
        );
    }
}
