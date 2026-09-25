//! The snapshot cadence: how often the link thread publishes each vehicle's state for a screen
//! to read. A frame reads the newest snapshot, so a cadence longer than the frame shows some
//! frames the snapshot the frame before had shown - which the 20 ms cadence did on a 60 Hz
//! display (`tests/gui/storm.gui`, 2026-09-26: 570 frames, 469 distinct snapshots). The default
//! is now 5 ms, shorter than any frame up to 200 Hz.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::time::{Duration, Instant};

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
/// have repainted a stale state.
fn stale_frames(publish_interval: Duration, samples: usize) -> usize {
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
        std::thread::spawn(move || {
            let mut roll = 0.0f32;
            while sending.load(std::sync::atomic::Ordering::Relaxed) {
                roll += 0.001;
                vehicle.write_all(&attitude(1, roll)).unwrap();
                std::thread::sleep(Duration::from_millis(5));
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
        std::thread::sleep(Duration::from_millis(2));
    };

    let mut stale = 0;
    let mut last = handle.load().messages_applied;
    for _ in 0..samples {
        std::thread::sleep(FRAME_60_HZ);
        let applied = handle.load().messages_applied;
        if applied <= last {
            stale += 1;
        }
        last = applied;
    }
    sending.store(false, std::sync::atomic::Ordering::Relaxed);
    sender.join().unwrap();
    stale
}

#[test]
fn a_60_hz_screen_sees_a_fresh_snapshot_every_frame_at_the_default_cadence() {
    // The two cadences under the same load, so a busy machine moves both counts and not the
    // comparison: the old 20 ms cadence repeats a snapshot about one frame in six; the default
    // must do clearly better than that, and a quiet machine gives it none at all.
    let samples = 120;
    let old = stale_frames(Duration::from_millis(20), samples);
    let now = stale_frames(DEFAULT_PUBLISH_INTERVAL, samples);
    assert!(
        old >= samples / 12,
        "the 20 ms cadence should repeat snapshots at 60 Hz; only {old} of {samples} did"
    );
    assert!(
        now * 4 < old.max(1),
        "the default cadence repeated {now} of {samples} snapshots against the 20 ms cadence's {old}"
    );
}
