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

//! Converting between mission items and their MAVLink wire form.
//!
//! `MISSION_ITEM_INT` carries latitude and longitude as 1e7 fixed-point integers and altitude as
//! an `f32`. The older `MISSION_ITEM` used floats for all three and is deprecated precisely
//! because an `f32` degree has about a metre of resolution at the equator - enough to move a
//! waypoint off a runway.
//!
//! One asymmetry to know: for **non-navigation** commands, x and y are not coordinates at all but
//! command parameters that happen to occupy those fields. Scaling those by 1e7 corrupts them, so
//! the conversion checks what kind of command it is.

use crate::item::MissionItem;

/// `MAV_MISSION_TYPE_MISSION`; fences and rally points use other values on the same protocol.
pub const MISSION_TYPE_MISSION: u8 = 0;

/// Fields of a `MISSION_ITEM_INT`, in wire units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WireItem {
    /// Sequence number.
    pub seq: u16,
    /// `MAV_FRAME`.
    pub frame: u8,
    /// `MAV_CMD`.
    pub command: u16,
    /// Whether this is the active item.
    pub current: u8,
    /// Whether to continue automatically.
    pub autocontinue: u8,
    /// Command parameter 1.
    pub param1: f32,
    /// Command parameter 2.
    pub param2: f32,
    /// Command parameter 3.
    pub param3: f32,
    /// Command parameter 4.
    pub param4: f32,
    /// Latitude in 1e7 degrees, or a raw parameter for non-navigation commands.
    pub x: i32,
    /// Longitude in 1e7 degrees, or a raw parameter.
    pub y: i32,
    /// Altitude in metres.
    pub z: f32,
}

impl MissionItem {
    /// Converts to wire form.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)] // guarded by the clamps below
    pub fn to_wire(&self) -> WireItem {
        let (x, y) = if self.is_navigation() {
            (scale_degrees(self.x), scale_degrees(self.y))
        } else {
            // Raw parameters, carried without scaling.
            (
                self.x
                    .clamp(f64::from(i32::MIN), f64::from(i32::MAX))
                    .round() as i32,
                self.y
                    .clamp(f64::from(i32::MIN), f64::from(i32::MAX))
                    .round() as i32,
            )
        };

        WireItem {
            seq: self.seq,
            frame: self.frame,
            command: self.command,
            current: self.current,
            autocontinue: self.autocontinue,
            param1: self.param1 as f32,
            param2: self.param2 as f32,
            param3: self.param3 as f32,
            param4: self.param4 as f32,
            x,
            y,
            z: self.z as f32,
        }
    }

    /// Converts from wire form.
    #[must_use]
    pub fn from_wire(wire: &WireItem) -> Self {
        // Same rule as MissionItem::is_navigation; fences and rally points carry coordinates
        // despite sitting outside the classic navigation command block.
        let navigation = matches!(wire.command, 16..=95 | 5000..=5006 | 5100);
        let (x, y) = if navigation {
            (f64::from(wire.x) / 1e7, f64::from(wire.y) / 1e7)
        } else {
            (f64::from(wire.x), f64::from(wire.y))
        };

        Self {
            seq: wire.seq,
            current: wire.current,
            frame: wire.frame,
            command: wire.command,
            param1: f64::from(wire.param1),
            param2: f64::from(wire.param2),
            param3: f64::from(wire.param3),
            param4: f64::from(wire.param4),
            x,
            y,
            z: f64::from(wire.z),
            autocontinue: wire.autocontinue,
        }
    }
}

/// Degrees to 1e7 fixed point, saturating at the representable range.
#[allow(clippy::cast_possible_truncation)]
fn scale_degrees(degrees: f64) -> i32 {
    (degrees * 1e7)
        .clamp(f64::from(i32::MIN), f64::from(i32::MAX))
        .round() as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn navigation_coordinates_scale_by_1e7() {
        let item = MissionItem {
            seq: 1,
            command: 16,
            x: -35.363_262,
            y: 149.165_237,
            z: 40.0,
            ..MissionItem::default()
        };
        let wire = item.to_wire();
        assert_eq!(wire.x, -353_632_620);
        assert_eq!(wire.y, 1_491_652_370);

        let back = MissionItem::from_wire(&wire);
        assert!(
            (back.x - item.x).abs() < 1e-7,
            "latitude drifted to {}",
            back.x
        );
        assert!(
            (back.y - item.y).abs() < 1e-7,
            "longitude drifted to {}",
            back.y
        );
    }

    #[test]
    fn non_navigation_parameters_are_not_scaled() {
        // DO_SET_SERVO carries a channel and a PWM value in the fields a waypoint uses for
        // coordinates. Multiplying those by 1e7 sends nonsense to the vehicle.
        let item = MissionItem {
            seq: 2,
            command: 183,
            x: 9.0,
            y: 1500.0,
            ..MissionItem::default()
        };
        let wire = item.to_wire();
        assert_eq!(wire.x, 9, "servo channel must survive unscaled");
        assert_eq!(wire.y, 1500, "pwm value must survive unscaled");

        let back = MissionItem::from_wire(&wire);
        assert!((back.x - 9.0).abs() < f64::EPSILON);
        assert!((back.y - 1500.0).abs() < f64::EPSILON);
    }

    #[test]
    fn a_full_mission_round_trips_through_the_wire() {
        let mission = vec![
            MissionItem {
                seq: 0,
                command: 16,
                x: -35.362_938,
                y: 149.165_085,
                z: 584.4,
                ..MissionItem::default()
            },
            MissionItem {
                seq: 1,
                command: 22,
                param1: 15.0,
                x: -35.361_164,
                y: 149.163_986,
                z: 28.11,
                ..MissionItem::default()
            },
            MissionItem {
                seq: 2,
                command: 183,
                param1: 9.0,
                x: 1500.0,
                ..MissionItem::default()
            },
            MissionItem {
                seq: 3,
                command: 20,
                ..MissionItem::default()
            },
        ];

        for item in &mission {
            let back = MissionItem::from_wire(&item.to_wire());
            assert_eq!(back.seq, item.seq);
            assert_eq!(back.command, item.command);
            assert_eq!(back.frame, item.frame);
            assert!(
                (back.x - item.x).abs() < 1e-6,
                "{:?} x moved to {}",
                item.command,
                back.x
            );
            assert!(
                (back.y - item.y).abs() < 1e-6,
                "{:?} y moved to {}",
                item.command,
                back.y
            );
            assert!(
                (back.z - item.z).abs() < 1e-3,
                "{:?} z moved to {}",
                item.command,
                back.z
            );
        }
    }

    #[test]
    fn fence_and_rally_coordinates_are_scaled_like_waypoints() {
        // These commands sit outside the 16-95 navigation block but carry real positions.
        // Sending them unscaled puts a geofence off the coast of Africa.
        for command in [5001u16, 5004, 5100] {
            let item = MissionItem {
                seq: 0,
                command,
                x: -35.363_262,
                y: 149.165_237,
                ..MissionItem::default()
            };
            let wire = item.to_wire();
            assert_eq!(
                wire.x, -353_632_620,
                "command {command} latitude was not scaled"
            );
            assert_eq!(
                wire.y, 1_491_652_370,
                "command {command} longitude was not scaled"
            );

            let back = MissionItem::from_wire(&wire);
            assert!(
                (back.x - item.x).abs() < 1e-7,
                "command {command} did not round trip"
            );
        }
    }

    #[test]
    fn coordinates_saturate_rather_than_wrapping() {
        // A corrupt latitude must not wrap to the opposite hemisphere.
        let item = MissionItem {
            seq: 1,
            command: 16,
            x: 1e6,
            y: 0.0,
            ..MissionItem::default()
        };
        assert_eq!(item.to_wire().x, i32::MAX);
    }
}
