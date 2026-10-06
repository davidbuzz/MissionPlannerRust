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

//! What a vehicle's proximity sensors report: `ExtLibs/ArduPilot/Proximity.cs`, the object
//! `MAVState` makes for every vehicle it hears (`MAVState.Proximity`,
//! `ExtLibs/ArduPilot/Mavlink/MAVState.cs:105-106`) and `Controls/ProximityControl.cs` draws.
//!
//! It holds `directionState`'s list: one entry per `DISTANCE_SENSOR` sensor id and orientation,
//! and one per `OBSTACLE_DISTANCE` sensor type and angle, each kept for a while after it came -
//! three seconds for a `DISTANCE_SENSOR`, a fifth of one for an `OBSTACLE_DISTANCE` - and then
//! dropped (`Proximity.cs:48-106, 114-246`).
//!
//! Where this differs from the C#, and why:
//!
//! * the list is a fixed array of [`CAPACITY`] entries, because [`crate::VehicleState`] is `Copy`
//!   (a snapshot is a memcpy); the C#'s `List<data>` grows without bound. When it is full the
//!   oldest entry - the first in the list - makes room. 128 is every `DISTANCE_SENSOR` orientation
//!   ArduPilot reports, and one `OBSTACLE_DISTANCE`'s 72 angles, several times over;
//! * an entry is stamped with the clock the vehicle state keeps, `CurrentState.datetime`: the time
//!   it was read on a live link, as `DateTime.Now` is, and the recording's time in a replay, where
//!   the C# stamps it with the wall clock; a reader expires the list against the time it passes;
//! * `DataAvailable`, `GetClosest` and `GetWarnings` are not ported: `GetClosest` and `GetWarnings`
//!   have no callers in the C# tree, and `DataAvailable`'s one reader guards a block whose only
//!   line is commented out (`GCSViews/FlightData.cs:3870-3873`).
//!
//! The subscription is the C#'s exactly: `SubscribeToPacketType(..., sysid, compid)` hands it the
//! messages of its own system and component only (`MAVLinkInterface.cs:5508-5533`), which is the
//! [`crate::VehicleState`] they are applied to here.

use mp_mavlink_dialects::all::{DistanceSensor, ObstacleDistance};

use crate::clock::DateTime;

/// How many entries the list holds; see the module documentation.
pub const CAPACITY: usize = 128;

/// `MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_CUSTOM`: what an `OBSTACLE_DISTANCE` entry's
/// orientation is. `// C#: ExtLibs/Mavlink/Mavlink.cs:4382`
pub const ROTATION_CUSTOM: u8 = 100;

/// `MAV_FRAME.GLOBAL`. `// C#: ExtLibs/Mavlink/Mavlink.cs:3020`
const FRAME_GLOBAL: u8 = 0;

/// How long a `DISTANCE_SENSOR` reading is kept, seconds. `// C#: ExtLibs/ArduPilot/Proximity.cs:64`
pub const DISTANCE_SENSOR_AGE: f32 = 3.0;

/// How long an `OBSTACLE_DISTANCE` reading is kept, seconds.
/// `// C#: ExtLibs/ArduPilot/Proximity.cs:99`
pub const OBSTACLE_DISTANCE_AGE: f32 = 0.2;

/// `directionState.data`: one reading.
/// `// C#: ExtLibs/ArduPilot/Proximity.cs:118-150`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reading {
    /// `SensorId`: a `DISTANCE_SENSOR`'s `id`, an `OBSTACLE_DISTANCE`'s `sensor_type`.
    pub sensor_id: u8,
    /// `Orientation`: the `MAV_SENSOR_ORIENTATION` it faces, [`ROTATION_CUSTOM`] for an
    /// `OBSTACLE_DISTANCE` angle.
    pub orientation: u8,
    /// `Angle`, degrees clockwise from the front: 0 for a `DISTANCE_SENSOR`. A `double` in the
    /// C# made from `float` arithmetic, which a `f32` holds exactly.
    pub angle: f32,
    /// `Size`, the arc it covers, degrees: 45 for a `DISTANCE_SENSOR`.
    pub size: f32,
    /// `Distance`, centimetres.
    pub distance: f32,
    /// `Received`.
    pub received: DateTime,
    /// `Age`, seconds.
    pub age: f32,
}

impl Reading {
    /// `ExpireTime`: `Received.AddSeconds(Age)`, which rounds to whole milliseconds as .NET's
    /// `AddSeconds` does.
    /// `// C#: ExtLibs/ArduPilot/Proximity.cs:128`
    #[must_use]
    #[allow(clippy::cast_possible_truncation)] // whole milliseconds of a few seconds
    pub fn expires(&self) -> DateTime {
        let value = f64::from(self.age);
        let millis = (value * 1000.0 + if value >= 0.0 { 0.5 } else { -0.5 }) as i64;
        DateTime::from_ticks(self.received.ticks() + millis * 10_000)
    }
}

const EMPTY: Reading = Reading {
    sensor_id: 0,
    orientation: 0,
    angle: 0.0,
    size: 0.0,
    distance: 0.0,
    received: DateTime::MIN,
    age: 0.0,
};

/// `Proximity` and its `directionState`: the readings, oldest first.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Proximity {
    readings: [Reading; CAPACITY],
    len: usize,
}

impl Default for Proximity {
    fn default() -> Self {
        Self {
            readings: [EMPTY; CAPACITY],
            len: 0,
        }
    }
}

impl Proximity {
    /// The readings held, oldest first, expired ones included until a reader or the next
    /// message drops them.
    #[must_use]
    pub fn held(&self) -> &[Reading] {
        self.readings.get(..self.len).unwrap_or(&[])
    }

    /// `GetRaw`: the readings not expired at `now`, in the list's order.
    /// `// C#: ExtLibs/ArduPilot/Proximity.cs:220-225, 227-246`
    pub fn raw(&self, now: DateTime) -> impl Iterator<Item = &Reading> {
        // `if (expireat < DateTime.Now)` removes it.
        self.held()
            .iter()
            .filter(move |reading| reading.expires() >= now)
    }

    /// `messageReceived` for a `DISTANCE_SENSOR`: kept unless it reads at or past either end of
    /// its range, by its id and orientation, for three seconds.
    /// `// C#: ExtLibs/ArduPilot/Proximity.cs:54-67`
    pub fn distance_sensor(&mut self, m: &DistanceSensor, now: DateTime) {
        if m.current_distance >= m.max_distance || m.current_distance <= m.min_distance {
            return;
        }
        // `Add(uint id, MAV_SENSOR_ORIENTATION orientation, ...)`: any entry of this id and
        // orientation goes, an OBSTACLE_DISTANCE one included if the numbers meet.
        // C#: ExtLibs/ArduPilot/Proximity.cs:152-164
        self.remove(|reading| reading.sensor_id == m.id && reading.orientation == m.orientation);
        self.push(
            Reading {
                sensor_id: m.id,
                orientation: m.orientation,
                angle: 0.0,
                size: 45.0,
                distance: f32::from(m.current_distance),
                received: now,
                age: DISTANCE_SENSOR_AGE,
            },
            now,
        );
    }

    /// `messageReceived` for an `OBSTACLE_DISTANCE`: each distance that is used and within the
    /// range, at its angle - from the front for `MAV_FRAME_BODY_FRD`, from north for
    /// `MAV_FRAME_GLOBAL`, which adds `yaw` (degrees, 0 to 360, as `cs.yaw` holds it) - by the
    /// sensor type and the angle, for a fifth of a second.
    /// `// C#: ExtLibs/ArduPilot/Proximity.cs:68-103`
    pub fn obstacle_distance(&mut self, m: &ObstacleDistance, yaw: f32, now: DateTime) {
        // `dists.increment == 0 ? dists.increment_f : dists.increment`: a float either way.
        let inc = if m.increment == 0 {
            m.increment_f
        } else {
            f32::from(m.increment)
        };
        let mut rangestart = m.angle_offset;
        // BODY_FRD: no action; GLOBAL: north-aligned; anything else as BODY_FRD.
        if m.frame == FRAME_GLOBAL {
            rangestart += yaw;
        }
        for (a, &distance) in m.distances.iter().enumerate() {
            // `not used`, past the far end, short of the near end.
            if distance == u16::MAX || distance > m.max_distance || distance < m.min_distance {
                continue;
            }
            let dist = distance.max(m.min_distance).min(m.max_distance);
            #[allow(clippy::cast_precision_loss)] // a = 0 to 71
            let angle = rangestart + inc * a as f32;
            // `Add(uint id, double angle, ...)`: any entry of this id at this angle goes.
            // C#: ExtLibs/ArduPilot/Proximity.cs:166-178
            self.remove(|reading| reading.sensor_id == m.sensor_type && reading.angle == angle);
            self.push(
                Reading {
                    sensor_id: m.sensor_type,
                    orientation: ROTATION_CUSTOM,
                    angle,
                    size: inc,
                    distance: f32::from(dist),
                    received: now,
                    age: OBSTACLE_DISTANCE_AGE,
                },
                now,
            );
        }
    }

    /// The entries `matches` picks out removed, the rest kept in their order.
    fn remove(&mut self, matches: impl Fn(&Reading) -> bool) {
        let mut kept = 0;
        for index in 0..self.len {
            let Some(reading) = self.readings.get(index).copied() else {
                break;
            };
            if !matches(&reading) {
                if let Some(slot) = self.readings.get_mut(kept) {
                    *slot = reading;
                }
                kept += 1;
            }
        }
        self.len = kept;
    }

    /// `_dists.Add(...)` then `expire()`, the oldest entry making room when the list is full.
    /// `// C#: ExtLibs/ArduPilot/Proximity.cs:161-163, 175-177, 227-246`
    fn push(&mut self, reading: Reading, now: DateTime) {
        self.remove(|held| held.expires() < now);
        if self.len == CAPACITY {
            self.readings.copy_within(1.., 0);
            self.len -= 1;
        }
        if let Some(slot) = self.readings.get_mut(self.len) {
            *slot = reading;
            self.len += 1;
        }
        // The new one expired already only if its clock ran backwards; `expire()` drops it then.
        self.remove(|held| held.expires() < now);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A second's worth of ticks.
    const SECOND: i64 = 10_000_000;

    fn at(seconds: f64) -> DateTime {
        #[allow(clippy::cast_possible_truncation)]
        DateTime::from_ticks(630_000_000_000_000_000 + (seconds * 1e7) as i64)
    }

    fn sensor(id: u8, orientation: u8, distance: u16) -> DistanceSensor {
        DistanceSensor {
            time_boot_ms: 0,
            min_distance: 20,
            max_distance: 700,
            current_distance: distance,
            r#type: 0,
            id,
            orientation,
            covariance: 0,
            horizontal_fov: 0.0,
            vertical_fov: 0.0,
            quaternion: [0.0; 4],
            signal_quality: 0,
        }
    }

    fn obstacles(frame: u8, increment: u8, increment_f: f32, offset: f32) -> ObstacleDistance {
        ObstacleDistance {
            time_usec: 0,
            distances: [u16::MAX; 72],
            min_distance: 10,
            max_distance: 1000,
            sensor_type: 3,
            increment,
            increment_f,
            angle_offset: offset,
            frame,
        }
    }

    /// A reading at or past either end of its range is not kept; one inside is, by its id and
    /// orientation, replacing the last of that pair and no other.
    /// `// C#: ExtLibs/ArduPilot/Proximity.cs:54-67, 152-164`
    #[test]
    fn a_distance_sensor_reading_is_kept_by_its_id_and_orientation() {
        let mut p = Proximity::default();
        p.distance_sensor(&sensor(0, 0, 700), at(0.0));
        p.distance_sensor(&sensor(0, 0, 20), at(0.0));
        assert!(p.held().is_empty(), "at max and at min: dropped");
        p.distance_sensor(&sensor(0, 0, 150), at(0.0));
        p.distance_sensor(&sensor(0, 2, 300), at(0.0));
        p.distance_sensor(&sensor(1, 0, 400), at(0.0));
        p.distance_sensor(&sensor(0, 0, 250), at(0.5));
        let held: Vec<(u8, u8, f32)> = p
            .raw(at(0.5))
            .map(|r| (r.sensor_id, r.orientation, r.distance))
            .collect();
        assert_eq!(held, [(0, 2, 300.0), (1, 0, 400.0), (0, 0, 250.0)]);
        let first = p.held()[0];
        assert_eq!((first.angle, first.size, first.age), (0.0, 45.0, 3.0));
    }

    /// Kept three seconds: still there at exactly three - `expireat < DateTime.Now` - and gone
    /// a millisecond later, whether a reader asks or the next message clears it.
    #[test]
    fn a_distance_sensor_reading_expires_after_three_seconds() {
        let mut p = Proximity::default();
        p.distance_sensor(&sensor(0, 0, 150), at(0.0));
        assert_eq!(p.held()[0].expires().ticks(), at(0.0).ticks() + 3 * SECOND);
        assert_eq!(p.raw(at(3.0)).count(), 1);
        assert_eq!(p.raw(at(3.001)).count(), 0);
        p.distance_sensor(&sensor(1, 4, 150), at(3.001));
        assert_eq!(p.held().len(), 1, "the expired one dropped by the next add");
        assert_eq!(p.held()[0].sensor_id, 1);
    }

    /// `OBSTACLE_DISTANCE`: unused, too far and too near dropped; the rest at `angle_offset +
    /// increment * index`, in the body frame as they are and in the global frame turned by the
    /// yaw; `increment_f` when `increment` is 0; kept a fifth of a second.
    /// `// C#: ExtLibs/ArduPilot/Proximity.cs:68-103`
    #[test]
    fn obstacle_distances_are_kept_by_angle() {
        let mut p = Proximity::default();
        let mut m = obstacles(12, 5, 0.0, 10.0);
        m.distances[0] = 100;
        m.distances[1] = 1001; // past max
        m.distances[2] = 9; // short of min
        m.distances[3] = 10; // at min: kept
        m.distances[4] = 1000; // at max: kept
        p.obstacle_distance(&m, 90.0, at(0.0));
        let held: Vec<(f32, f32, f32, u8)> = p
            .raw(at(0.0))
            .map(|r| (r.angle, r.size, r.distance, r.orientation))
            .collect();
        assert_eq!(
            held,
            [
                (10.0, 5.0, 100.0, ROTATION_CUSTOM),
                (25.0, 5.0, 10.0, ROTATION_CUSTOM),
                (30.0, 5.0, 1000.0, ROTATION_CUSTOM),
            ]
        );
        assert_eq!(p.held()[0].sensor_id, 3);
        assert_eq!(p.raw(at(0.2)).count(), 3);
        assert_eq!(p.raw(at(0.201)).count(), 0);

        // The global frame adds the yaw; increment_f stands in for a zero increment.
        let mut p = Proximity::default();
        let mut m = obstacles(0, 0, 2.5, -5.0);
        m.distances[2] = 500;
        p.obstacle_distance(&m, 90.0, at(0.0));
        let reading = p.held()[0];
        assert_eq!((reading.angle, reading.size), (90.0, 2.5));

        // The same angle again replaces it.
        m.distances[2] = 600;
        p.obstacle_distance(&m, 90.0, at(0.1));
        assert_eq!(p.held().len(), 1);
        assert_eq!(p.held()[0].distance, 600.0);
    }

    /// The C#'s matching is by the fields it compares and no others: an `OBSTACLE_DISTANCE` at
    /// angle 0 whose sensor type is a `DISTANCE_SENSOR`'s id replaces that reading too.
    #[test]
    fn the_two_adds_match_as_the_csharp_matches() {
        let mut p = Proximity::default();
        p.distance_sensor(&sensor(3, 1, 150), at(0.0));
        let mut m = obstacles(12, 5, 0.0, 0.0);
        m.distances[0] = 100;
        p.obstacle_distance(&m, 0.0, at(0.0));
        assert_eq!(p.held().len(), 1);
        assert_eq!(p.held()[0].orientation, ROTATION_CUSTOM);
    }

    /// The vehicle state takes both messages on the path the link applies them, stamped with
    /// its clock, a north-aligned `OBSTACLE_DISTANCE` turned by `cs.yaw` - 0 to 360 - from the
    /// last `ATTITUDE`.
    #[test]
    fn the_vehicle_state_applies_both_messages() {
        use mp_mavlink_dialects::all::{Attitude, MavMessage};
        let mut state = crate::VehicleState::new(1, 1);
        state.datetime = at(0.0);
        state.apply(&MavMessage::Attitude(Attitude {
            time_boot_ms: 0,
            roll: 0.0,
            pitch: 0.0,
            yaw: -std::f32::consts::FRAC_PI_2,
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
        }));
        state.apply(&MavMessage::DistanceSensor(sensor(0, 2, 150)));
        let mut m = obstacles(0, 10, 0.0, 0.0);
        m.distances[1] = 300;
        state.apply(&MavMessage::ObstacleDistance(m));
        let held: Vec<(u8, f32, f32)> = state
            .proximity
            .raw(at(0.0))
            .map(|r| (r.orientation, r.angle, r.distance))
            .collect();
        assert_eq!(held, [(2, 0.0, 150.0), (ROTATION_CUSTOM, 280.0, 300.0)]);
        assert_eq!(state.proximity.held()[0].received, at(0.0));
    }

    /// Full, the oldest makes room.
    #[test]
    fn a_full_list_drops_its_oldest() {
        let mut p = Proximity::default();
        for index in 0..=CAPACITY {
            let mut m = obstacles(12, 1, 0.0, 0.0);
            m.sensor_type = u8::try_from(index / 72).unwrap();
            m.angle_offset = 0.0;
            m.distances = [u16::MAX; 72];
            m.distances[index % 72] = 100;
            p.obstacle_distance(&m, 0.0, at(0.0));
        }
        assert_eq!(p.held().len(), CAPACITY);
        assert_eq!((p.held()[0].sensor_id, p.held()[0].angle), (0, 1.0));
        let last = p.held()[CAPACITY - 1];
        assert_eq!((last.sensor_id, last.angle), (1, 56.0));
    }
}
