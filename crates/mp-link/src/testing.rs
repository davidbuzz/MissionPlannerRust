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

//! A vehicle's side of a link, for tests and for measuring the screens under load.
//!
//! Two things live here because more than one crate needs them. The frame builders encode a
//! message as a vehicle puts it on the wire; `tests/telemetry_storm.rs` had its own until the GUI
//! needed the same frames. [`Storm`] writes those frames at a fixed rate into an in-memory
//! [`Loopback`], so that a link, and a screen reading one, can be measured under DELIVERABLES.md
//! D10's "200 Hz telemetry storm" without a simulator. `planner` runs one when `MP_STORM` is set
//! (`crates/mp-gui/src/storm.rs`).
//!
//! A link in normal operation uses nothing here.

use std::f64::consts::TAU;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use mp_mavlink::encode_v2;
use mp_mavlink_dialects::all::{Attitude, GlobalPositionInt, Heartbeat, MavMessage, VfrHud};
use mp_transport::Transport as _;
use mp_transport::testing::{Loopback, LoopbackEnd};

/// The system the frames come from.
pub const SYSID: u8 = 1;
/// The component the frames come from: the autopilot.
pub const COMPID: u8 = 1;

/// Encodes one message as MAVLink 2 frame `seq` from [`SYSID`] and [`COMPID`].
#[must_use]
pub fn frame(seq: u8, message: &MavMessage) -> Vec<u8> {
    let mut payload = [0u8; 255];
    let len = message.encode(&mut payload);
    let mut out = [0u8; mp_mavlink::MAX_FRAME_LEN];
    let written = payload.get(..len).and_then(|payload| {
        encode_v2(
            &mut out,
            seq,
            SYSID,
            COMPID,
            message.id(),
            payload,
            message.crc_extra(),
            0,
        )
        .ok()
    });
    written
        .and_then(|n| out.get(..n))
        .map(<[u8]>::to_vec)
        .unwrap_or_default()
}

/// A quadcopter's heartbeat: ArduPilot, disarmed, in its first mode.
#[must_use]
pub fn heartbeat(seq: u8) -> Vec<u8> {
    frame(
        seq,
        &MavMessage::Heartbeat(Heartbeat {
            custom_mode: 0,
            r#type: 2,
            autopilot: 3,
            base_mode: 81,
            system_status: 3,
            mavlink_version: 3,
        }),
    )
}

/// ATTITUDE with a roll, in radians, and everything else level.
#[must_use]
pub fn attitude(seq: u8, roll: f32) -> Vec<u8> {
    frame(
        seq,
        &MavMessage::Attitude(Attitude {
            time_boot_ms: u32::from(seq),
            roll,
            pitch: 0.0,
            yaw: 0.0,
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
        }),
    )
}

/// GLOBAL_POSITION_INT at ArduPilot's SITL home, 10 m up and still.
#[must_use]
pub fn position(seq: u8) -> Vec<u8> {
    frame(
        seq,
        &MavMessage::GlobalPositionInt(GlobalPositionInt {
            time_boot_ms: u32::from(seq),
            lat: -353_632_620,
            lon: 1_491_652_370,
            alt: 600_000,
            relative_alt: 10_000,
            vx: 0,
            vy: 0,
            vz: 0,
            hdg: 18_000,
        }),
    )
}

/// The frames each tick of a [`Storm`] writes: ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD and
/// HEARTBEAT. A storm at 200 Hz is 200 of each a second, 800 frames.
pub const STORM_FRAMES_PER_TICK: u32 = 4;

/// ArduPilot's SITL home at CMAC, which the storm's vehicle circles.
const HOME: (f64, f64) = (-35.363_261, 149.165_230);
/// Home's height above sea level, metres.
const HOME_ALT: f64 = 584.0;
/// How high the circle is flown above home, metres.
const HEIGHT: f64 = 50.0;
/// The circle's radius, metres.
const RADIUS: f64 = 150.0;
/// One lap, seconds: about 16 m/s round a 150 m circle.
const LAP: f64 = 60.0;
/// Metres in a degree of latitude.
const METRES_PER_DEGREE: f64 = 111_319.5;

/// The vehicle behind a [`Storm`]: it flies a circle round CMAC, banking and pitching as it goes,
/// so every value a screen shows changes on every tick.
///
/// The circle rather than a hover because a hovering vehicle's position never moves, and the
/// map's track, its automatic fit and the HUD's heading tape would be measured doing nothing.
#[derive(Debug, Clone)]
pub struct StormVehicle {
    /// Ticks a second.
    rate: u32,
    /// Ticks written so far.
    tick: u64,
    /// The next frame's sequence number.
    seq: u8,
}

impl StormVehicle {
    /// A vehicle that will be ticked `rate` times a second.
    #[must_use]
    pub fn new(rate: u32) -> Self {
        Self {
            rate: rate.max(1),
            tick: 0,
            seq: 0,
        }
    }

    /// Ticks written so far.
    #[must_use]
    pub const fn ticks(&self) -> u64 {
        self.tick
    }

    /// The next tick's four frames, one after the other, as one write.
    #[must_use]
    pub fn next_tick(&mut self) -> Vec<u8> {
        let messages = self.messages();
        let mut bytes = Vec::with_capacity(160);
        for message in &messages {
            bytes.extend_from_slice(&frame(self.seq, message));
            self.seq = self.seq.wrapping_add(1);
        }
        self.tick += 1;
        bytes
    }

    /// Where the vehicle is at this tick, and what it is doing.
    ///
    /// The narrowing casts are onto the wire's own types, from values bounded by construction:
    /// a position within 150 m of CMAC, speeds under 20 m/s, angles under a radian.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn messages(&self) -> [MavMessage; 4] {
        // Seconds since the storm began, and how far round the circle that is.
        #[allow(clippy::cast_precision_loss)] // exact below 2^52 ticks
        let t = self.tick as f64 / f64::from(self.rate);
        let angle = TAU * t / LAP;
        let omega = TAU / LAP;
        let (north, east) = (RADIUS * angle.cos(), RADIUS * angle.sin());
        let (v_north, v_east) = (-RADIUS * omega * angle.sin(), RADIUS * omega * angle.cos());
        let track = v_east.atan2(v_north);
        let heading = track.to_degrees().rem_euclid(360.0);
        let speed = RADIUS * omega;
        let climb = 0.5 * (TAU * t / 10.0).sin();
        let latitude = HOME.0 + north / METRES_PER_DEGREE;
        let longitude = HOME.1 + east / (METRES_PER_DEGREE * HOME.0.to_radians().cos());
        let time_boot_ms = (t * 1000.0) as u32;

        [
            MavMessage::Attitude(Attitude {
                time_boot_ms,
                roll: (0.35 * (TAU * t / 4.0).sin()) as f32,
                pitch: (0.1 * (TAU * t / 3.0).sin()) as f32,
                yaw: track as f32,
                rollspeed: 0.0,
                pitchspeed: 0.0,
                yawspeed: omega as f32,
            }),
            MavMessage::GlobalPositionInt(GlobalPositionInt {
                time_boot_ms,
                lat: (latitude * 1e7).round() as i32,
                lon: (longitude * 1e7).round() as i32,
                alt: ((HOME_ALT + HEIGHT) * 1000.0) as i32,
                relative_alt: (HEIGHT * 1000.0) as i32,
                vx: (v_north * 100.0) as i16,
                vy: (v_east * 100.0) as i16,
                vz: (-climb * 100.0) as i16,
                hdg: (heading * 100.0) as u16,
            }),
            MavMessage::VfrHud(VfrHud {
                airspeed: speed as f32,
                groundspeed: speed as f32,
                alt: (HOME_ALT + HEIGHT) as f32,
                climb: climb as f32,
                heading: heading as i16,
                throttle: (50.0 + 5.0 * (TAU * t / 7.0).sin()) as u16,
            }),
            MavMessage::Heartbeat(Heartbeat {
                custom_mode: 0,
                r#type: 2,
                autopilot: 3,
                base_mode: 81,
                system_status: 3,
                mavlink_version: 3,
            }),
        ]
    }
}

/// When each tick of a [`Storm`] is due.
///
/// Deadline-paced, so the rate is the rate asked for and not the rate less the time the writes
/// and the sleeps' overshoot take: a writer that sleeps a period after each tick runs slow by
/// exactly that. A writer that falls behind catches up, so a brief deschedule costs no ticks;
/// one that falls far behind starts again from now, rather than writing a second's worth of
/// ticks at once.
#[derive(Debug, Clone)]
struct Pacer {
    period: Duration,
    /// When the next tick is due.
    next: Instant,
}

impl Pacer {
    /// Behind by more than this and the schedule starts again from now.
    const GIVE_UP_CATCHING_UP: Duration = Duration::from_millis(250);

    fn new(rate: u32, now: Instant) -> Self {
        Self {
            period: Duration::from_secs(1) / rate.max(1),
            next: now,
        }
    }

    /// How long to wait, from `now`, before writing the next tick. Advances the schedule by one.
    fn wait(&mut self, now: Instant) -> Duration {
        let due = self.next;
        self.next += self.period;
        if due > now {
            return due - now;
        }
        if now - due > Self::GIVE_UP_CATCHING_UP {
            self.next = now + self.period;
        }
        Duration::ZERO
    }
}

/// A vehicle writing telemetry at a fixed rate into an in-memory link, on a thread of its own.
///
/// Stops, and joins its thread, when dropped.
#[derive(Debug)]
pub struct Storm {
    /// Ticks a second.
    rate: u32,
    stop: Arc<AtomicBool>,
    ticks: Arc<AtomicU64>,
    thread: Option<JoinHandle<()>>,
}

impl Storm {
    /// Starts a storm of `rate` ticks a second, each tick [`STORM_FRAMES_PER_TICK`] frames, and
    /// returns it with the end a link reads from.
    #[must_use]
    pub fn start(rate: u32) -> (Self, LoopbackEnd) {
        let rate = rate.max(1);
        let (mut vehicle_end, link_end) = Loopback::pair();
        let stop = Arc::new(AtomicBool::new(false));
        let ticks = Arc::new(AtomicU64::new(0));
        let thread = {
            let stop = Arc::clone(&stop);
            let ticks = Arc::clone(&ticks);
            std::thread::Builder::new()
                .name("mp-storm".to_owned())
                .spawn(move || {
                    let mut vehicle = StormVehicle::new(rate);
                    let mut pacer = Pacer::new(rate, Instant::now());
                    let mut sink = [0u8; 4096];
                    while !stop.load(Ordering::Acquire) {
                        let wait = pacer.wait(Instant::now());
                        if !wait.is_zero() {
                            std::thread::sleep(wait);
                        }
                        // What the link sends the vehicle - its heartbeat, a banner request - is
                        // read and dropped, so the direction nobody else reads does not grow for
                        // as long as the storm runs.
                        while vehicle_end.read(&mut sink).is_ok_and(|n| n > 0) {}
                        if vehicle_end.write_all(&vehicle.next_tick()).is_err() {
                            break;
                        }
                        ticks.store(vehicle.ticks(), Ordering::Release);
                    }
                })
                .ok()
        };
        (
            Self {
                rate,
                stop,
                ticks,
                thread,
            },
            link_end,
        )
    }

    /// Ticks a second, as asked for.
    #[must_use]
    pub const fn rate(&self) -> u32 {
        self.rate
    }

    /// Ticks written so far.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.ticks.load(Ordering::Acquire)
    }
}

impl Drop for Storm {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mp_mavlink::FrameDecoder;
    use mp_mavlink_dialects::all::DIALECT;

    /// Decodes everything in `bytes`, as the link would.
    fn decode(bytes: &[u8]) -> Vec<(u8, MavMessage)> {
        let mut decoder = FrameDecoder::new();
        let mut out = Vec::new();
        decoder.push_and_drain(bytes, &DIALECT, |frame| {
            if let Some(message) = MavMessage::decode(frame.msgid, frame.payload) {
                out.push((frame.seq, message));
            }
        });
        out
    }

    #[test]
    fn a_tick_is_the_four_messages_in_sequence() {
        let mut vehicle = StormVehicle::new(200);
        let mut bytes = vehicle.next_tick();
        bytes.extend(vehicle.next_tick());
        let frames = decode(&bytes);
        let ids: Vec<u32> = frames.iter().map(|(_, message)| message.id()).collect();
        assert_eq!(ids, [30, 33, 74, 0, 30, 33, 74, 0]);
        // One sequence across every frame, so the link counts no loss.
        let seqs: Vec<u8> = frames.iter().map(|(seq, _)| *seq).collect();
        assert_eq!(seqs, (0..8).collect::<Vec<u8>>());
        assert_eq!(vehicle.ticks(), 2);
        assert_eq!(STORM_FRAMES_PER_TICK, 4);
    }

    #[test]
    fn the_vehicle_moves_on_every_tick() {
        // A storm whose values never change would measure a screen redrawing the same thing.
        let mut vehicle = StormVehicle::new(200);
        let mut positions = Vec::new();
        let mut rolls = Vec::new();
        for _ in 0..50 {
            for (_, message) in decode(&vehicle.next_tick()) {
                match message {
                    MavMessage::GlobalPositionInt(p) => positions.push((p.lat, p.lon)),
                    MavMessage::Attitude(a) => rolls.push(a.roll),
                    _ => {}
                }
            }
        }
        positions.dedup();
        assert!(positions.len() > 40, "{positions:?}");
        assert!(rolls.windows(2).all(|pair| pair[0] != pair[1]));
        // Within the circle's radius of home, in degrees E7.
        for (lat, lon) in positions {
            assert!((lat + 353_632_610).abs() < 20_000, "{lat}");
            assert!((lon - 1_491_652_300).abs() < 20_000, "{lon}");
        }
    }

    #[test]
    fn the_pacer_holds_the_rate_it_was_given() {
        // Driven by a clock that only moves when the writer sleeps: 200 Hz for a second is 200
        // ticks, exactly, and none of them early.
        let start = Instant::now();
        let end = start + Duration::from_secs(1);
        let mut pacer = Pacer::new(200, start);
        let (mut now, mut ticks) = (start, 0);
        loop {
            let at = now + pacer.wait(now);
            if at >= end {
                break;
            }
            (now, ticks) = (at, ticks + 1);
        }
        assert_eq!(ticks, 200);
    }

    #[test]
    fn a_late_writer_catches_up_without_losing_ticks() {
        // A 40 ms deschedule is eight ticks late; they are written at once, and the second still
        // holds 200.
        let start = Instant::now();
        let end = start + Duration::from_secs(1);
        let mut pacer = Pacer::new(200, start);
        let (mut now, mut ticks) = (start, 0);
        loop {
            if ticks == 100 {
                now += Duration::from_millis(40);
            }
            let at = now + pacer.wait(now);
            if at >= end {
                break;
            }
            (now, ticks) = (at, ticks + 1);
        }
        assert_eq!(ticks, 200);
    }

    #[test]
    fn a_writer_far_behind_starts_again_rather_than_bursting() {
        // Stopped for two seconds, a catching-up writer would put 400 ticks on the wire at once:
        // a storm the rate never asked for.
        let start = Instant::now();
        let mut pacer = Pacer::new(200, start);
        let late = start + Duration::from_secs(2);
        assert_eq!(pacer.wait(start), Duration::ZERO);
        assert_eq!(pacer.wait(late), Duration::ZERO);
        let next = pacer.wait(late);
        assert!(
            next > Duration::ZERO && next <= Duration::from_millis(5),
            "{next:?}"
        );
    }

    #[test]
    fn a_storm_writes_at_its_rate_and_stops_when_dropped() {
        let (storm, mut end) = Storm::start(200);
        std::thread::sleep(Duration::from_millis(500));
        let ticks = storm.ticks();
        drop(storm);
        // Half a second at 200 Hz is 100 ticks. Generous either side for a loaded machine, and
        // still far from an unthrottled writer's thousands.
        assert!((60..=140).contains(&ticks), "{ticks} ticks in 500 ms");
        let mut buf = vec![0u8; 1 << 20];
        let mut bytes = Vec::new();
        while let Ok(n) = end.read(&mut buf) {
            if n == 0 {
                break;
            }
            bytes.extend_from_slice(&buf[..n]);
        }
        let frames = u64::try_from(decode(&bytes).len()).unwrap_or(u64::MAX);
        assert!(
            frames >= ticks * u64::from(STORM_FRAMES_PER_TICK),
            "{frames}"
        );
        // Dropped means stopped: nothing more arrives.
        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(end.read(&mut buf).ok(), Some(0));
    }
}
