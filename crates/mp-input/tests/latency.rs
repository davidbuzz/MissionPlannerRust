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

//! Stick-to-wire latency, measured against a fake device.
//!
//! DELIVERABLES.md D15: stick input to packet on the wire, p99 under 5 ms. The device here is one
//! end of a Unix socket pair, which the reader blocks on exactly as it blocks on `/dev/input/js*`:
//! the kernel wakes the blocked `read` when bytes arrive. Each test writes `js_event`s at recorded
//! instants and times how long each takes to reach the sink as a change, so what is measured is
//! the whole path this crate owns - a byte becoming readable, the read thread waking, decoding,
//! publishing, the send thread waking, mapping, the failsafe, and the sink being called.
//!
//! The assertions are on p99, not the maximum, because a machine running a parallel build will
//! occasionally deschedule a thread for longer than 5 ms and that is not what D15 measures. The
//! bound is the real target, not a loosened one: on an idle machine the p99 is two orders of
//! magnitude under it, so a failure here means the design has regressed - a poll interval crept
//! back in, or a send started waiting for a tick - rather than that the machine was busy.
//!
//! Run with `--nocapture` to see the histograms.

#![cfg(unix)]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::collections::HashMap;
use std::io::Write;
use std::os::unix::net::UnixStream;
use std::sync::mpsc::{self, Receiver};
use std::thread;
use std::time::{Duration, Instant};

use mp_input::mapping::{SAFE_MAX_US, SAFE_MIN_US};
use mp_input::{
    Cause, Frame, JoystickAxis, LatencyHistogram, MIN_INTERVAL, Mapping, RESEND, RcRanges, Reading,
    Runtime, StickReader,
};

/// Channel 1 driven by axis 0 - `X` - with the vehicle's ranges given.
fn channel_one_on_x(ranges: RcRanges) -> Mapping {
    let mut mapping = Mapping {
        ranges,
        ..Mapping::default()
    };
    mapping.config.set_axis(1, JoystickAxis::X);
    mapping
}

/// D15's bar.
const TARGET: Duration = Duration::from_millis(5);

/// Long enough to wait for something that must happen, on a machine that is busy.
const PATIENCE: Duration = Duration::from_secs(5);

/// A reader on one end of a socket pair, flying, with every frame the sink sees timestamped.
fn flying(mapping: Mapping) -> (StickReader, UnixStream, Receiver<(Instant, Frame)>) {
    let (device, feed) = UnixStream::pair().expect("a socket pair");
    let (wire, frames) = mpsc::channel();
    let reader = StickReader::spawn(device, mapping, move |frame| {
        // Stamped first thing in the sink: this is the moment the link would be handed the frame.
        let delivered = Instant::now();
        let _ = wire.send((delivered, *frame));
        true
    })
    .expect("threads start");
    // The device opens as the kernel's does, saying where its sticks are; nothing is sent before
    // that is read.
    let mut feed = feed;
    feed.write_all(&mp_input::event::encode(
        0,
        mp_input::event::AXIS | mp_input::event::INIT,
        0,
        0,
    ))
    .expect("the opening burst");
    let until = Instant::now() + PATIENCE;
    while reader.reading().axes.is_empty() {
        assert!(Instant::now() < until, "the opening burst never arrived");
        std::thread::sleep(Duration::from_millis(1));
    }
    assert!(reader.set_enabled(true));
    // Switching on sends the current position at once; take it, so the first event's change is
    // not mistaken for it.
    let (_, first) = frames.recv_timeout(PATIENCE).expect("a first frame");
    assert_eq!(first.cause, Cause::Refresh);
    (reader, feed, frames)
}

/// The next change on the wire, skipping the resends that a held stick gets every 50 ms.
fn next_change(frames: &Receiver<(Instant, Frame)>) -> (Instant, Frame) {
    loop {
        let (at, frame) = frames.recv_timeout(PATIENCE).expect("a change on the wire");
        if frame.cause == Cause::Changed {
            return (at, frame);
        }
    }
}

fn axis_event(value: i16) -> [u8; mp_input::event::LEN] {
    mp_input::event::encode(0, mp_input::event::AXIS, 0, value)
}

/// Exact percentiles from the raw samples, alongside the histogram's bucketed bound.
fn report(title: &str, samples: &mut [Duration], histogram: &LatencyHistogram) -> Duration {
    samples.sort_unstable();
    let at = |percent: usize| samples[(samples.len() * percent).div_ceil(100) - 1];
    let (p50, p99, max) = (at(50), at(99), samples[samples.len() - 1]);
    println!("\n{title}");
    println!(
        "exact: p50 {:.3} ms, p99 {:.3} ms, max {:.3} ms",
        p50.as_secs_f64() * 1e3,
        p99.as_secs_f64() * 1e3,
        max.as_secs_f64() * 1e3
    );
    println!("{histogram}");
    p99
}

/// One event at a time, each waited for before the next: the latency of a stick movement on its
/// own, with both threads asleep in between. This is D15's number.
///
/// "On its own" means nothing was sent in the last `MIN_INTERVAL`, which is what lets a movement
/// go out at once rather than wait for the rate floor - so each event is written a floor and a
/// millisecond after the previous frame, plus up to 1.6 ms more that varies, so that events do
/// not all land at the same point in the threads' sleep. It makes this test take about twenty
/// seconds for a thousand events; fewer events would make p99 the tenth-slowest of a handful.
#[test]
fn an_isolated_movement_reaches_the_wire_within_five_milliseconds() {
    const EVENTS: usize = 1_000;
    let mapping = channel_one_on_x(RcRanges::default());
    let (reader, mut feed, frames) = flying(mapping);

    let mut histogram = LatencyHistogram::new();
    let mut samples = Vec::with_capacity(EVENTS);
    // The switch-on frame has been taken, so the last send was before now.
    let mut last_sent = Instant::now();
    for index in 0..EVENTS {
        // Either side of centre in turn, so every event moves channel 1.
        let size = 8_000 + i16::try_from(index % 1_000).unwrap() * 20;
        let value = if index % 2 == 0 { size } else { -size };
        let isolated = last_sent
            + MIN_INTERVAL
            + Duration::from_millis(1)
            + Duration::from_micros(400 * u64::try_from(index % 5).unwrap());
        thread::sleep(isolated.saturating_duration_since(Instant::now()));

        let written = Instant::now();
        feed.write_all(&axis_event(value)).expect("write an event");
        let (delivered, frame) = next_change(&frames);
        last_sent = delivered;

        let channel = frame.channels.get(1).unwrap();
        assert_eq!(
            value > 0,
            channel > mp_input::mapping::CENTRE_US,
            "event {index} ({value}) produced {channel}: the change on the wire is not this event"
        );
        let latency = delivered.saturating_duration_since(written);
        histogram.record(latency);
        samples.push(latency);
    }

    let p99 = report(
        &format!("isolated movements: {EVENTS} events, written -> delivered to the sink"),
        &mut samples,
        &histogram,
    );
    println!(
        "\nread -> sink returned, as the reader records it:\n{}",
        reader.latency()
    );
    assert_eq!(histogram.count(), u64::try_from(EVENTS).unwrap());
    assert!(
        p99 <= TARGET,
        "p99 {p99:?} is over D15's {TARGET:?}\n{histogram}"
    );
    reader.close();
}

/// A stick being stirred, one event a millisecond whether or not the last has been sent - a
/// device polled at 1 kHz. Nothing waits for anything, so this is the case the rate floor is for:
/// the wire must see no more than one frame per `MIN_INTERVAL`, no fewer than one per `RESEND`,
/// never an old position after a newer one, and every position within a floor (plus 5 ms) of
/// being written.
///
/// Every event is a distinct channel value, so each frame on the wire says exactly which event it
/// carries. An event's latency is from its write to the first frame carrying it *or anything
/// newer*: an event overtaken by the next one before it could be sent has been delivered the moment
/// the newer position is, which is what coalescing means.
#[test]
fn a_stick_stirred_at_one_kilohertz_is_sent_at_most_every_floor_and_never_stale() {
    // `pickchannel` puts every axis through -500 to 500 in whole steps, truncated - so the step
    // either side of centre is twice as wide as the rest - and a sweep has at most 1001 distinct
    // values. The events are the first `js` position of each of 990 of them, in order.
    const EVENTS: usize = 990;
    // The widest range a channel may have, so each of those steps is a distinct microsecond value.
    let mapping = channel_one_on_x(RcRanges {
        listed: true,
        channels: vec![
            None,
            Some((
                i32::from(SAFE_MIN_US),
                i32::from(SAFE_MAX_US),
                i32::from(SAFE_MAX_US) / 2,
            )),
        ],
    });
    let channel_of = |value: i16| {
        let reading = Reading {
            axes: vec![f32::from(value) / 32_767.0],
            buttons: Vec::new(),
        };
        mapping
            .overrides(&reading, Runtime::default())
            .channels()
            .get(1)
            .unwrap()
    };
    let mut values: Vec<i16> = Vec::with_capacity(EVENTS);
    let mut previous = None;
    for position in -32_767..=32_767i16 {
        let channel = channel_of(position);
        if previous != Some(channel) {
            previous = Some(channel);
            values.push(position);
            if values.len() == EVENTS {
                break;
            }
        }
    }
    // The channel value each event produces, through the same decoding the reader applies.
    let index_of: HashMap<u16, usize> = values
        .iter()
        .enumerate()
        .map(|(index, &value)| (channel_of(value), index))
        .collect();
    assert_eq!(index_of.len(), EVENTS, "two events share a channel value");
    let last_value = channel_of(values[EVENTS - 1]);

    let (reader, mut feed, frames) = flying(mapping.clone());

    let writer = thread::spawn(move || {
        let start = Instant::now();
        let mut written = Vec::with_capacity(values.len());
        for (index, value) in values.into_iter().enumerate() {
            // On an absolute schedule, so a late wake-up does not push every later event back.
            let due = start + Duration::from_millis(u64::try_from(index).unwrap());
            thread::sleep(due.saturating_duration_since(Instant::now()));
            written.push(Instant::now());
            feed.write_all(&axis_event(value)).expect("write an event");
        }
        (written, feed)
    });

    // Every frame, resends included: the floor is about what the link is asked to carry.
    let mut on_wire: Vec<(Instant, Frame)> = Vec::new();
    loop {
        let (at, frame) = frames.recv_timeout(PATIENCE).expect("a frame");
        on_wire.push((at, frame));
        if frame.cause == Cause::Changed && frame.channels.get(1) == Some(last_value) {
            break;
        }
    }
    let (written, feed) = writer.join().expect("the writer");

    let first_write = written[0];
    on_wire.retain(|(at, _)| *at >= first_write);
    let delivered: Vec<(Instant, usize)> = on_wire
        .iter()
        .filter(|(_, frame)| frame.cause == Cause::Changed)
        .map(|(at, frame)| {
            let channel = frame.channels.get(1).unwrap();
            let index = *index_of
                .get(&channel)
                .unwrap_or_else(|| panic!("{channel} is not a value any event produces"));
            (*at, index)
        })
        .collect();

    // At most one frame per floor across the run, and at least one per resend.
    let span = on_wire.last().unwrap().0.duration_since(first_write);
    let most = span.as_micros() / MIN_INTERVAL.as_micros() + 2;
    let least = span.as_micros() / RESEND.as_micros();
    let count = u128::try_from(on_wire.len()).unwrap();
    println!(
        "\nstirred at 1 kHz: {} frames on the wire over {:.0} ms ({:.1} frames/s); \
         allowed {least}..={most}",
        on_wire.len(),
        span.as_secs_f64() * 1e3,
        on_wire.len() as f64 / span.as_secs_f64(),
    );
    assert!(
        count <= most,
        "{count} frames in {span:?} is more than one per {MIN_INTERVAL:?}"
    );
    assert!(
        count >= least,
        "{count} frames in {span:?} is fewer than one per {RESEND:?}"
    );

    // Never a stale position after a newer one: the sweep only goes one way, so neither may the
    // wire.
    assert!(
        delivered.windows(2).all(|pair| pair[0].1 < pair[1].1),
        "a position was sent after a newer one"
    );

    let mut histogram = LatencyHistogram::new();
    let mut samples = Vec::with_capacity(EVENTS);
    let mut frame = 0;
    for (index, written_at) in written.iter().enumerate() {
        while delivered[frame].1 < index {
            frame += 1;
        }
        let latency = delivered[frame].0.saturating_duration_since(*written_at);
        histogram.record(latency);
        samples.push(latency);
    }

    let p99 = report(
        &format!(
            "stirred at 1 kHz: {EVENTS} events in {} changes on the wire, written -> delivered",
            delivered.len()
        ),
        &mut samples,
        &histogram,
    );
    // Held by the floor, so not D15's 5 ms: a floor, plus the same 5 ms for getting there.
    let bound = MIN_INTERVAL + TARGET;
    assert!(
        p99 <= bound,
        "p99 {p99:?} is over {bound:?}: a stirred stick is staler than one floor\n{histogram}"
    );
    drop(feed);
    reader.close();
}
