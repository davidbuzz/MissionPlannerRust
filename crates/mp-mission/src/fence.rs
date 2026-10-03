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

//! Geofences and rally points.
//!
//! Replaces the fence and rally handling in `ExtLibs/ArduPilot` and `GCSViews/FlightPlanner`.
//!
//! Both travel over the **same** mission protocol as waypoints, distinguished only by
//! `MAV_MISSION_TYPE`. That is convenient and dangerous in equal measure: sending a fence with the
//! mission type left at zero overwrites the flight plan, and the vehicle will accept it.

use mp_units::LatLon;

use crate::item::MissionItem;

/// `MAV_MISSION_TYPE_FENCE`.
pub const MISSION_TYPE_FENCE: u8 = 1;
/// `MAV_MISSION_TYPE_RALLY`.
pub const MISSION_TYPE_RALLY: u8 = 2;

/// `MAV_CMD_NAV_FENCE_RETURN_POINT`.
pub const CMD_FENCE_RETURN_POINT: u16 = 5000;
/// `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION`: stay inside.
pub const CMD_FENCE_POLYGON_INCLUSION: u16 = 5001;
/// `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION`: stay outside.
pub const CMD_FENCE_POLYGON_EXCLUSION: u16 = 5002;
/// `MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION`.
pub const CMD_FENCE_CIRCLE_INCLUSION: u16 = 5003;
/// `MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION`.
pub const CMD_FENCE_CIRCLE_EXCLUSION: u16 = 5004;
/// `MAV_CMD_NAV_RALLY_POINT`.
pub const CMD_RALLY_POINT: u16 = 5100;

/// A geofence element.
#[derive(Debug, Clone, PartialEq)]
pub enum FenceItem {
    /// Where the vehicle returns to on a breach.
    ReturnPoint {
        /// Position.
        position: LatLon,
    },
    /// A polygon the vehicle must stay inside or outside.
    Polygon {
        /// Whether the vehicle must remain inside.
        inclusion: bool,
        /// Vertices, in order.
        vertices: Vec<LatLon>,
    },
    /// A circle the vehicle must stay inside or outside.
    Circle {
        /// Whether the vehicle must remain inside.
        inclusion: bool,
        /// Centre.
        centre: LatLon,
        /// Radius in metres.
        radius: f64,
    },
}

/// Why a fence is not valid.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum FenceError {
    /// A polygon needs at least three vertices to enclose anything.
    #[error("a polygon fence needs at least 3 vertices, found {found}")]
    PolygonTooSmall {
        /// How many vertices were given.
        found: usize,
    },
    /// A circle with no radius encloses nothing.
    #[error("a circle fence needs a positive radius, found {radius}")]
    CircleRadius {
        /// The radius given, rendered.
        radius: String,
    },
    /// The vertex count declared by the first vertex disagrees with how many arrived.
    #[error("polygon declared {declared} vertices but {found} arrived")]
    VertexCountMismatch {
        /// What the items claimed.
        declared: usize,
        /// What was present.
        found: usize,
    },
}

impl FenceItem {
    /// Checks the fence describes an actual region.
    ///
    /// A two-vertex "polygon" and a zero-radius circle are both accepted by the wire format and
    /// enclose nothing. A vehicle given one has a fence that can never be breached, which is
    /// indistinguishable from having no fence at all - except that the pilot believes otherwise.
    pub fn validate(&self) -> Result<(), FenceError> {
        match self {
            Self::ReturnPoint { .. } => Ok(()),
            Self::Polygon { vertices, .. } => {
                if vertices.len() < 3 {
                    Err(FenceError::PolygonTooSmall {
                        found: vertices.len(),
                    })
                } else {
                    Ok(())
                }
            }
            Self::Circle { radius, .. } => {
                if *radius > 0.0 && radius.is_finite() {
                    Ok(())
                } else {
                    Err(FenceError::CircleRadius {
                        radius: format!("{radius}"),
                    })
                }
            }
        }
    }

    /// Converts to the mission items that carry this fence over the wire.
    ///
    /// Polygon vertices each repeat the total vertex count in `param1`, which is how the vehicle
    /// knows where one polygon ends and the next begins.
    #[must_use]
    pub fn to_items(&self, start_seq: u16) -> Vec<MissionItem> {
        match self {
            Self::ReturnPoint { position } => vec![MissionItem {
                seq: start_seq,
                command: CMD_FENCE_RETURN_POINT,
                x: position.latitude(),
                y: position.longitude(),
                ..MissionItem::default()
            }],
            Self::Polygon {
                inclusion,
                vertices,
            } => {
                let command = if *inclusion {
                    CMD_FENCE_POLYGON_INCLUSION
                } else {
                    CMD_FENCE_POLYGON_EXCLUSION
                };
                #[allow(clippy::cast_precision_loss)] // vertex counts are small
                let count = vertices.len() as f64;
                vertices
                    .iter()
                    .enumerate()
                    .map(|(index, vertex)| MissionItem {
                        seq: start_seq + u16::try_from(index).unwrap_or(u16::MAX),
                        command,
                        param1: count,
                        x: vertex.latitude(),
                        y: vertex.longitude(),
                        ..MissionItem::default()
                    })
                    .collect()
            }
            Self::Circle {
                inclusion,
                centre,
                radius,
            } => {
                let command = if *inclusion {
                    CMD_FENCE_CIRCLE_INCLUSION
                } else {
                    CMD_FENCE_CIRCLE_EXCLUSION
                };
                vec![MissionItem {
                    seq: start_seq,
                    command,
                    param1: *radius,
                    x: centre.latitude(),
                    y: centre.longitude(),
                    ..MissionItem::default()
                }]
            }
        }
    }
}

/// A place the vehicle can divert to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RallyPoint {
    /// Position.
    pub position: LatLon,
    /// Altitude above home, in metres.
    pub altitude: f64,
    /// Preferred approach heading in degrees, or `None` for any.
    pub break_altitude: Option<f64>,
}

impl RallyPoint {
    /// Converts to the mission item that carries it.
    #[must_use]
    pub fn to_item(&self, seq: u16) -> MissionItem {
        MissionItem {
            seq,
            command: CMD_RALLY_POINT,
            x: self.position.latitude(),
            y: self.position.longitude(),
            z: self.altitude,
            param2: self.break_altitude.unwrap_or(0.0),
            ..MissionItem::default()
        }
    }
}

/// Rebuilds fence items from the flat list the protocol delivers.
///
/// Polygons arrive as runs of vertices that each declare the total; the run ends when that many
/// have been seen. Trusting the declared count without checking how many actually arrived is how
/// a truncated transfer becomes a fence with a missing side.
pub fn fences_from_items(items: &[MissionItem]) -> Result<Vec<FenceItem>, FenceError> {
    let mut out = Vec::new();
    let mut index = 0usize;

    while index < items.len() {
        let Some(item) = items.get(index) else { break };
        match item.command {
            CMD_FENCE_RETURN_POINT => {
                if let Ok(position) = LatLon::new(item.x, item.y) {
                    out.push(FenceItem::ReturnPoint { position });
                }
                index += 1;
            }
            CMD_FENCE_POLYGON_INCLUSION | CMD_FENCE_POLYGON_EXCLUSION => {
                let inclusion = item.command == CMD_FENCE_POLYGON_INCLUSION;
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let declared = item.param1.max(0.0) as usize;
                let mut vertices = Vec::with_capacity(declared);

                while index < items.len() {
                    let Some(vertex) = items.get(index) else {
                        break;
                    };
                    if vertex.command != item.command || vertices.len() >= declared {
                        break;
                    }
                    if let Ok(position) = LatLon::new(vertex.x, vertex.y) {
                        vertices.push(position);
                    }
                    index += 1;
                }

                if vertices.len() != declared {
                    return Err(FenceError::VertexCountMismatch {
                        declared,
                        found: vertices.len(),
                    });
                }
                let fence = FenceItem::Polygon {
                    inclusion,
                    vertices,
                };
                fence.validate()?;
                out.push(fence);
            }
            CMD_FENCE_CIRCLE_INCLUSION | CMD_FENCE_CIRCLE_EXCLUSION => {
                if let Ok(centre) = LatLon::new(item.x, item.y) {
                    let fence = FenceItem::Circle {
                        inclusion: item.command == CMD_FENCE_CIRCLE_INCLUSION,
                        centre,
                        radius: item.param1,
                    };
                    fence.validate()?;
                    out.push(fence);
                }
                index += 1;
            }
            _ => index += 1,
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(lat: f64, lon: f64) -> LatLon {
        LatLon::new(lat, lon).expect("valid")
    }

    #[test]
    fn a_polygon_round_trips_through_mission_items() {
        let fence = FenceItem::Polygon {
            inclusion: true,
            vertices: vec![
                at(-35.0, 149.0),
                at(-35.0, 149.1),
                at(-35.1, 149.1),
                at(-35.1, 149.0),
            ],
        };
        let items = fence.to_items(0);
        assert_eq!(items.len(), 4);
        // Every vertex declares the total, which is how the vehicle finds the end of the run.
        assert!(items.iter().all(|i| (i.param1 - 4.0).abs() < f64::EPSILON));

        let rebuilt = fences_from_items(&items).expect("valid");
        assert_eq!(rebuilt, vec![fence]);
    }

    #[test]
    fn two_polygons_in_one_transfer_are_separated() {
        let a = FenceItem::Polygon {
            inclusion: true,
            vertices: vec![at(-35.0, 149.0), at(-35.0, 149.1), at(-35.1, 149.1)],
        };
        let b = FenceItem::Polygon {
            inclusion: false,
            vertices: vec![at(-36.0, 149.0), at(-36.0, 149.1), at(-36.1, 149.1)],
        };
        let mut items = a.to_items(0);
        items.extend(b.to_items(3));

        let rebuilt = fences_from_items(&items).expect("valid");
        assert_eq!(
            rebuilt,
            vec![a, b],
            "the inclusion and exclusion polygons must stay distinct"
        );
    }

    #[test]
    fn a_truncated_polygon_is_an_error_rather_than_a_fence_with_a_missing_side() {
        let fence = FenceItem::Polygon {
            inclusion: true,
            vertices: vec![
                at(-35.0, 149.0),
                at(-35.0, 149.1),
                at(-35.1, 149.1),
                at(-35.1, 149.0),
            ],
        };
        let mut items = fence.to_items(0);
        items.pop(); // a transfer that stopped early

        assert_eq!(
            fences_from_items(&items),
            Err(FenceError::VertexCountMismatch {
                declared: 4,
                found: 3
            })
        );
    }

    #[test]
    fn degenerate_fences_are_refused() {
        // Both are accepted by the wire format and enclose nothing, which is worse than no fence
        // because the pilot believes there is one.
        let two_sided = FenceItem::Polygon {
            inclusion: true,
            vertices: vec![at(-35.0, 149.0), at(-35.0, 149.1)],
        };
        assert_eq!(
            two_sided.validate(),
            Err(FenceError::PolygonTooSmall { found: 2 })
        );

        let no_radius = FenceItem::Circle {
            inclusion: true,
            centre: at(-35.0, 149.0),
            radius: 0.0,
        };
        assert!(no_radius.validate().is_err());

        let nan_radius = FenceItem::Circle {
            inclusion: true,
            centre: at(-35.0, 149.0),
            radius: f64::NAN,
        };
        assert!(
            nan_radius.validate().is_err(),
            "a NaN radius must not pass as positive"
        );
    }

    #[test]
    fn a_circle_round_trips_with_its_radius() {
        let fence = FenceItem::Circle {
            inclusion: false,
            centre: at(-35.36, 149.16),
            radius: 150.0,
        };
        let items = fence.to_items(7);
        assert_eq!(items[0].command, CMD_FENCE_CIRCLE_EXCLUSION);
        assert!(
            (items[0].param1 - 150.0).abs() < f64::EPSILON,
            "radius travels in param1"
        );

        assert_eq!(fences_from_items(&items).expect("valid"), vec![fence]);
    }

    #[test]
    fn a_rally_point_carries_its_altitude() {
        let rally = RallyPoint {
            position: at(-35.36, 149.16),
            altitude: 80.0,
            break_altitude: Some(40.0),
        };
        let item = rally.to_item(2);
        assert_eq!(item.command, CMD_RALLY_POINT);
        assert!((item.z - 80.0).abs() < f64::EPSILON);
        assert!((item.param2 - 40.0).abs() < f64::EPSILON);
        assert!(item.is_navigation(), "rally points are navigation commands");
    }
}
