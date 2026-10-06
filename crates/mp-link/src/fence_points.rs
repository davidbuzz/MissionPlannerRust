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

//! `MAVState.fencepoints`: each vehicle's geofence as the traffic on the link has shown it, which
//! `CurrentState.GeoFenceDist` measures from (`CurrentState.cs:1632`).
//!
//! The C# fills it from two places, and so does this:
//!
//! * whatever passes on the link, whoever asked for it - `processInfoFromStream` files every
//!   fence `MISSION_ITEM` and `MISSION_ITEM_INT` it reads under the vehicle it is about, clears
//!   that vehicle's fence on a fence `MISSION_COUNT`, and turns the old protocol's `FENCE_POINT`
//!   into an item (`MAVLinkInterface.cs:5605-5673, 5715-5728`). A download - this link's or
//!   another ground station's - fills it that way, and so does a recording, whose items this
//!   link sent are read back like any other;
//! * this link's own uploads: `setWPTotalAsync` clears it when the vehicle asks for the first
//!   item, and `setWPAsync` files each item it sent once the vehicle asks for the next with
//!   `MISSION_REQUEST` or answers with `MISSION_ACK` - whatever the answer; a `MISSION_REQUEST_INT`
//!   files nothing, as the C# has no branch that does (`MAVLinkInterface.cs:3794-3866,
//!   4090-4213, 4264-4346`). The link thread does this half (`file_fence_upload` in `lib.rs`).
//!
//! An item is filed as the C# files it: a `MISSION_ITEM_INT` as it came, and everything else
//! through `Locationwp`, whose round trip through degrees can move a coordinate by one unit of
//! 1e-7 degrees (`ExtLibs/Utilities/locationwp.cs:77-119, 153-178`).

use mp_mavlink_dialects::all::MavMessage;
use mp_mission::{MissionItem, WireItem};
use mp_vehicle::{FenceItem, VehicleId};

/// `MAV_MISSION_TYPE_FENCE`.
const FENCE: u8 = mp_mission::fence::MISSION_TYPE_FENCE;
/// `MAV_CMD_FENCE_RETURN_POINT`.
const FENCE_RETURN_POINT: u16 = 5000;
/// `MAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION`.
const FENCE_POLYGON_VERTEX_INCLUSION: u16 = 5001;

/// Every vehicle's fence points, each list in sequence order.
///
/// A list per vehicle, sorted by sequence number, rather than a map: clearing one keeps its
/// storage, so a vehicle whose fence is read again and again allocates nothing after the first
/// time, which the link's per-packet budget needs (DELIVERABLES.md Deliverable 5).
#[derive(Debug, Default)]
pub struct FencePoints {
    by_vehicle: Vec<(VehicleId, Vec<(u16, FenceItem)>)>,
}

impl FencePoints {
    /// A vehicle's fence, in sequence order: what `parent.fencepoints` enumerates, a
    /// `ConcurrentDictionary<int, ...>` whose small integer keys come out in order.
    #[must_use]
    pub fn items(&self, id: VehicleId) -> Vec<FenceItem> {
        self.list(id)
            .map(|list| list.iter().map(|(_, item)| *item).collect())
            .unwrap_or_default()
    }

    fn list(&self, id: VehicleId) -> Option<&Vec<(u16, FenceItem)>> {
        self.by_vehicle
            .iter()
            .find(|(vehicle, _)| *vehicle == id)
            .map(|(_, list)| list)
    }

    fn list_mut(&mut self, id: VehicleId) -> &mut Vec<(u16, FenceItem)> {
        let index = match self
            .by_vehicle
            .iter()
            .position(|(vehicle, _)| *vehicle == id)
        {
            Some(index) => index,
            None => {
                self.by_vehicle.push((id, Vec::new()));
                self.by_vehicle.len() - 1
            }
        };
        // Just found or just pushed.
        #[allow(clippy::indexing_slicing)]
        &mut self.by_vehicle[index].1
    }

    /// `fencepoints.Clear()`.
    pub fn clear(&mut self, id: VehicleId) {
        self.list_mut(id).clear();
    }

    /// `fencepoints[seq] = item`: replaced if the sequence number is there, added in order if not.
    pub fn store(&mut self, id: VehicleId, seq: u16, item: FenceItem) {
        let list = self.list_mut(id);
        match list.binary_search_by_key(&seq, |(key, _)| *key) {
            Ok(index) => {
                if let Some(slot) = list.get_mut(index) {
                    slot.1 = item;
                }
            }
            Err(index) => list.insert(index, (seq, item)),
        }
    }

    /// `processInfoFromStream`'s fence half, for one message from `sysid`/`compid` whose header
    /// names `header_target` (`TARGET32`) or none. A message addressed to this ground station
    /// (`gcs_sysid`) or to 0 is about its sender; any other is about the vehicle it is addressed
    /// to - the header's target first, then the payload's - which is how a recording's own
    /// uploads land on the vehicle.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5605-5673, 5715-5728`
    pub fn observe(
        &mut self,
        sysid: u32,
        compid: u8,
        gcs_sysid: u32,
        header_target: Option<u32>,
        message: &MavMessage,
    ) {
        // `GetTargetSystem() ?? sysid`, and the sender for this ground station or 0.
        // C#: MAVLinkInterface.cs:5607-5614
        let about = |target_system: u8, target_component: u8| {
            let system = header_target.unwrap_or(u32::from(target_system));
            if system == gcs_sysid || system == 0 {
                VehicleId::new(sysid, compid)
            } else {
                VehicleId::new(system, target_component)
            }
        };
        match message {
            // C#: MAVLinkInterface.cs:5615-5627
            MavMessage::MissionCount(m) if m.mission_type == FENCE => {
                self.clear(about(m.target_system, m.target_component));
            }
            // C#: MAVLinkInterface.cs:5628-5650, `(Locationwp) wp` from the float item.
            MavMessage::MissionItem(m) if m.mission_type == FENCE && m.current != 2 => {
                let item = FenceItem {
                    command: m.command,
                    param1: m.param1,
                    x: from_degrees(m.command, f64::from(m.x)),
                    y: from_degrees(m.command, f64::from(m.y)),
                };
                self.store(about(m.target_system, m.target_component), m.seq, item);
            }
            // C#: MAVLinkInterface.cs:5651-5673, filed as it came.
            MavMessage::MissionItemInt(m) if m.mission_type == FENCE && m.current != 2 => {
                let item = FenceItem {
                    command: m.command,
                    param1: m.param1,
                    x: m.x,
                    y: m.y,
                };
                self.store(about(m.target_system, m.target_component), m.seq, item);
            }
            // C#: MAVLinkInterface.cs:5715-5728, the old protocol's point as an item: the return
            // point at index 0, an inclusion vertex after it, the count less one as `param1`.
            MavMessage::FencePoint(m) => {
                let command = if m.idx == 0 {
                    FENCE_RETURN_POINT
                } else {
                    FENCE_POLYGON_VERTEX_INCLUSION
                };
                let item = FenceItem {
                    command,
                    param1: count_less_one(m.count),
                    x: truncate(f64::from(m.lat) * 1e7),
                    y: truncate(f64::from(m.lng) * 1e7),
                };
                self.store(
                    about(m.target_system, m.target_component),
                    u16::from(m.idx),
                    item,
                );
            }
            _ => {}
        }
    }
}

/// An item this link uploaded, as `setWPAsync` files it: `(Locationwp) req` of the
/// `mavlink_mission_item_int_t` it sent.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4282-4292, 4319-4329`
#[must_use]
pub fn uploaded(item: &MissionItem) -> FenceItem {
    let wire: WireItem = item.to_wire();
    FenceItem {
        command: wire.command,
        param1: wire.param1,
        x: through_degrees(wire.command, wire.x),
        y: through_degrees(wire.command, wire.y),
    }
}

/// An item this link sent on its own with `setWP` - a `MISSION_ITEM` or a `MISSION_ITEM_INT` -
/// as `setWPAsync` files it in `fencepoints` once the vehicle has taken it: `(Locationwp) req`
/// under its sequence number, for an item of the fence list that is neither a guided target
/// (current 2) nor an altitude change (current 3). `None` for anything else.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4107-4117, 4145-4155, 4282-4292, 4319-4329`
#[must_use]
pub fn set_wp_item(message: &MavMessage) -> Option<(u16, FenceItem)> {
    match message {
        MavMessage::MissionItem(m) if m.mission_type == FENCE && !matches!(m.current, 2 | 3) => {
            Some((
                m.seq,
                FenceItem {
                    command: m.command,
                    param1: m.param1,
                    x: from_degrees(m.command, f64::from(m.x)),
                    y: from_degrees(m.command, f64::from(m.y)),
                },
            ))
        }
        MavMessage::MissionItemInt(m) if m.mission_type == FENCE && !matches!(m.current, 2 | 3) => {
            Some((
                m.seq,
                FenceItem {
                    command: m.command,
                    param1: m.param1,
                    x: through_degrees(m.command, m.x),
                    y: through_degrees(m.command, m.y),
                },
            ))
        }
        _ => None,
    }
}

/// `Locationwp.isLocationCommand`: whether `x` and `y` are a position, scaled by 1e7 on the way
/// in and out of `Locationwp`. `// C#: ExtLibs/Utilities/locationwp.cs:39-56`
pub(crate) fn is_location(command: u16) -> bool {
    MissionItem {
        command,
        ..MissionItem::default()
    }
    .is_navigation()
}

/// A `mavlink_mission_item_int_t`'s coordinate into `Locationwp` and back: times 1e-7, then times
/// 1e7 and truncated, for a position. `// C#: ExtLibs/Utilities/locationwp.cs:96-119, 153-176`
pub(crate) fn through_degrees(command: u16, value: i32) -> i32 {
    let degrees = if is_location(command) {
        f64::from(value) * 1.0e-7
    } else {
        f64::from(value)
    };
    from_degrees(command, degrees)
}

/// `Locationwp`'s coordinate as `Convert(.., isint: true)` writes it: times 1e7 for a position,
/// then `(int)`. `// C#: ExtLibs/Utilities/locationwp.cs:153-176`
pub(crate) fn from_degrees(command: u16, degrees: f64) -> i32 {
    if is_location(command) {
        truncate(degrees * 1.0e7)
    } else {
        truncate(degrees)
    }
}

/// C#'s `(int)` of a double: toward zero; saturating here where the C# leaves it unspecified.
#[allow(clippy::cast_possible_truncation)]
const fn truncate(value: f64) -> i32 {
    value as i32
}

/// `fencept.count - 1`, an `int` in a `float` parameter.
#[allow(clippy::cast_precision_loss)]
fn count_less_one(count: u8) -> f32 {
    (i32::from(count) - 1) as f32
}

#[cfg(test)]
mod tests {
    use mp_mavlink_dialects::all::{FencePoint, MissionCount, MissionItemInt};

    use super::*;

    const VEHICLE: VehicleId = VehicleId::new(1, 1);
    const GCS: u8 = 255;

    fn item_int(seq: u16, target_system: u8, command: u16, x: i32) -> MavMessage {
        MavMessage::MissionItemInt(MissionItemInt {
            param1: 3.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x,
            y: x + 1,
            z: 0.0,
            seq,
            command,
            target_system,
            target_component: 190,
            frame: 3,
            current: 0,
            autocontinue: 1,
            mission_type: FENCE,
        })
    }

    #[test]
    fn items_are_filed_under_the_vehicle_they_are_about_in_sequence_order() {
        let mut fence = FencePoints::default();
        // The vehicle's answers to a download, addressed to this ground station, out of order.
        fence.observe(1, 1, u32::from(GCS), None, &item_int(2, GCS, 5001, 30));
        fence.observe(1, 1, u32::from(GCS), None, &item_int(0, GCS, 5001, 10));
        fence.observe(1, 1, u32::from(GCS), None, &item_int(1, GCS, 5001, 20));
        let xs: Vec<i32> = fence.items(VEHICLE).iter().map(|item| item.x).collect();
        assert_eq!(xs, [10, 20, 30]);
        // Filed again: replaced, not added.
        fence.observe(1, 1, u32::from(GCS), None, &item_int(1, GCS, 5001, 21));
        assert_eq!(fence.items(VEHICLE).len(), 3);
        assert_eq!(fence.items(VEHICLE)[1].x, 21);
        // A ground station's item, as a recording holds it, is about the vehicle it addresses.
        fence.observe(
            u32::from(GCS),
            190,
            u32::from(GCS),
            None,
            &item_int(0, 7, 5003, 40),
        );
        assert_eq!(fence.items(VehicleId::new(7, 190))[0].x, 40);
        // A fence MISSION_COUNT starts that vehicle's fence again.
        fence.observe(
            1,
            1,
            u32::from(GCS),
            None,
            &MavMessage::MissionCount(MissionCount {
                count: 3,
                target_system: GCS,
                target_component: 190,
                mission_type: FENCE,
            }),
        );
        assert!(fence.items(VEHICLE).is_empty());
        assert_eq!(fence.items(VehicleId::new(7, 190)).len(), 1);
    }

    #[test]
    fn a_point_to_no_one_is_its_senders_and_a_header_target_comes_before_the_payloads() {
        // `processInfoFromStream` as of 5dbb2b0 (e6454ccdd). C#: MAVLinkInterface.cs:5607-5614
        let mut fence = FencePoints::default();
        fence.observe(1, 1, u32::from(GCS), None, &item_int(0, 0, 5001, 10));
        assert_eq!(fence.items(VEHICLE).len(), 1);
        assert!(fence.items(VehicleId::new(0, 190)).is_empty());
        let wide = VehicleId::new(70_000, 190);
        fence.observe(
            u32::from(GCS),
            190,
            u32::from(GCS),
            Some(wide.sysid),
            &MavMessage::FencePoint(FencePoint {
                lat: -35.5,
                lng: 149.25,
                target_system: GCS,
                target_component: 190,
                idx: 0,
                count: 2,
            }),
        );
        assert_eq!(fence.items(wide).len(), 1);
        assert_eq!(fence.items(VEHICLE).len(), 1);
    }

    #[test]
    fn an_old_fence_point_is_a_return_point_then_inclusion_vertices() {
        let mut fence = FencePoints::default();
        for idx in 0..3 {
            fence.observe(
                1,
                1,
                u32::from(GCS),
                None,
                &MavMessage::FencePoint(FencePoint {
                    lat: -35.5,
                    lng: 149.25,
                    target_system: GCS,
                    target_component: 190,
                    idx,
                    count: 3,
                }),
            );
        }
        let items = fence.items(VEHICLE);
        assert_eq!(
            items.iter().map(|item| item.command).collect::<Vec<_>>(),
            [5000, 5001, 5001]
        );
        assert_eq!(items[0].param1, 2.0);
        assert_eq!((items[0].x, items[0].y), (-355_000_000, 1_492_500_000));
    }

    #[test]
    fn an_uploaded_item_goes_through_locationwp_as_the_csharp_files_it() {
        // A coordinate whose trip through degrees and back does not land on itself: the C#
        // truncates what comes back.
        let mut item = MissionItem {
            command: 5001,
            ..MissionItem::default()
        };
        item.x = -35.365;
        item.y = 149.17;
        let wire = item.to_wire();
        assert_eq!((wire.x, wire.y), (-353_650_000, 1_491_700_000));
        // -353650000 * 1e-7 * 1e7 is -353649999.99999994, and 1491700000's trip is
        // 1491699999.9999998: `(int)` takes each a unit toward zero.
        let filed = uploaded(&item);
        assert_eq!((filed.x, filed.y), (-353_649_999, 1_491_699_999));
        // A coordinate whose trip lands on itself stays.
        item.x = -35.363_262_1;
        assert_eq!(uploaded(&item).x, item.to_wire().x);
        // Not a position: the raw value, untouched.
        assert_eq!(through_degrees(177, 12_345), 12_345);
    }
}
