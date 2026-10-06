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

//! `MAVState.wps` and `MAVState.rallypoints`: each vehicle's mission and rally points as the
//! traffic on the link has shown them - "a snapshot of what is loaded on the ap atm. - derived
//! from the stream" (`MAVState.cs:310-315`). The flight screen draws the first as its mission
//! overlay and the second as its rally markers (`FlightData.cs:3924-3957, 4012-4019`), and counts
//! the first for its Set WP list (`:2581-2586`).
//!
//! Filled as `fencepoints` is (see [`crate::fence_points`]), from the same three places:
//!
//! * whatever passes on the link, whoever asked for it - `processInfoFromStream` clears a
//!   vehicle's list on a `MISSION_COUNT` naming it, files every `MISSION_ITEM` and
//!   `MISSION_ITEM_INT` that is not a guided target (`current` 2, which is `GuidedMode`'s) under
//!   the list its `mission_type` names, and turns the old protocol's `RALLY_POINT` into an item of
//!   the rally list (`MAVLinkInterface.cs:5605-5673, 5691-5702`). A download - this link's or
//!   another ground station's - fills a list that way, and so does a recording;
//! * this link's own uploads: `setWPTotalAsync` clears the list when the vehicle asks for the
//!   first item, and `setWPAsync` files each item it sent once the vehicle asks for the next with
//!   `MISSION_REQUEST` or answers with `MISSION_ACK` - whatever the answer (`file_list_upload` in
//!   `lib.rs`);
//! * a single `setWP` - a script's: filed by the message that ended it, see [`set_wp_item`].
//!
//! An item is filed as the C# files it, a `mavlink_mission_item_int_t`: an `_INT` item as it
//! came, a float item through `Locationwp` - `(int)(lat * 1e7)` for a position - and an item this
//! link sent through `Locationwp` and back, which can move a coordinate by one unit of 1e-7
//! degrees (`ExtLibs/Utilities/locationwp.cs:77-119, 153-178`). Read out as [`MissionItem`]s -
//! `(Locationwp) a` of each, the position back in degrees - which is how the flight screen reads
//! them.

use mp_mavlink_dialects::all::{MavMessage, MissionItem as MissionItemMessage, MissionItemInt};
use mp_mission::{MISSION_TYPE_MISSION, MissionItem};
use mp_vehicle::VehicleId;

use crate::fence_points::{from_degrees, is_location, through_degrees};

/// `MAV_MISSION_TYPE_RALLY`.
pub const MISSION_TYPE_RALLY: u8 = mp_mission::fence::MISSION_TYPE_RALLY;
/// `MAV_CMD_NAV_RALLY_POINT`, the command a `RALLY_POINT` is filed under.
const RALLY_POINT: u16 = 5100;
/// `MAV_FRAME_GLOBAL_RELATIVE_ALT`, the frame it is filed in.
const GLOBAL_RELATIVE_ALT: u8 = 3;

/// Every vehicle's mission and rally points, each list in sequence order.
///
/// Lists sorted by sequence number rather than maps, as [`crate::fence_points::FencePoints`] are:
/// clearing one keeps its storage, so a mission read again and again allocates nothing after the
/// first time, which the link's per-packet budget needs (DELIVERABLES.md Deliverable 5).
#[derive(Debug, Default)]
pub struct MissionPoints {
    by_vehicle: Vec<(VehicleId, Lists)>,
}

/// One vehicle's two lists.
#[derive(Debug, Default)]
struct Lists {
    /// `MAVState.wps`.
    wps: Vec<(u16, MissionItem)>,
    /// `MAVState.rallypoints`.
    rally: Vec<(u16, MissionItem)>,
}

impl Lists {
    /// The list a `MAV_MISSION_TYPE` names; none for the fence, which `fence_points` keeps, or
    /// a type the C# has no dictionary for.
    fn of_mut(&mut self, mission_type: u8) -> Option<&mut Vec<(u16, MissionItem)>> {
        match mission_type {
            MISSION_TYPE_MISSION => Some(&mut self.wps),
            MISSION_TYPE_RALLY => Some(&mut self.rally),
            _ => None,
        }
    }

    fn of(&self, mission_type: u8) -> Option<&Vec<(u16, MissionItem)>> {
        match mission_type {
            MISSION_TYPE_MISSION => Some(&self.wps),
            MISSION_TYPE_RALLY => Some(&self.rally),
            _ => None,
        }
    }
}

impl MissionPoints {
    /// A vehicle's mission, in sequence order: what `MAV.wps.Values` enumerates, a
    /// `ConcurrentDictionary<int, ...>` whose small integer keys come out in order.
    #[must_use]
    pub fn wps(&self, id: VehicleId) -> Vec<MissionItem> {
        self.items(id, MISSION_TYPE_MISSION)
    }

    /// A vehicle's rally points, in sequence order: `MAV.rallypoints.Values`.
    #[must_use]
    pub fn rally_points(&self, id: VehicleId) -> Vec<MissionItem> {
        self.items(id, MISSION_TYPE_RALLY)
    }

    /// The list a `MAV_MISSION_TYPE` names, in sequence order; empty for a type kept elsewhere.
    #[must_use]
    pub fn items(&self, id: VehicleId, mission_type: u8) -> Vec<MissionItem> {
        self.lists(id)
            .and_then(|lists| lists.of(mission_type))
            .map(|list| list.iter().map(|(_, item)| *item).collect())
            .unwrap_or_default()
    }

    fn lists(&self, id: VehicleId) -> Option<&Lists> {
        self.by_vehicle
            .iter()
            .find(|(vehicle, _)| *vehicle == id)
            .map(|(_, lists)| lists)
    }

    fn lists_mut(&mut self, id: VehicleId) -> &mut Lists {
        let index = match self
            .by_vehicle
            .iter()
            .position(|(vehicle, _)| *vehicle == id)
        {
            Some(index) => index,
            None => {
                self.by_vehicle.push((id, Lists::default()));
                self.by_vehicle.len() - 1
            }
        };
        // Just found or just pushed.
        #[allow(clippy::indexing_slicing)]
        &mut self.by_vehicle[index].1
    }

    /// `wps.Clear()` or `rallypoints.Clear()`, whichever `mission_type` names; nothing for any
    /// other type.
    pub fn clear(&mut self, id: VehicleId, mission_type: u8) {
        if let Some(list) = self.lists_mut(id).of_mut(mission_type) {
            list.clear();
        }
    }

    /// `wps[seq] = item` or `rallypoints[seq] = item`, whichever `mission_type` names: replaced
    /// if the sequence number is there, added in order if not. Nothing for any other type.
    pub fn store(&mut self, id: VehicleId, mission_type: u8, seq: u16, item: MissionItem) {
        let Some(list) = self.lists_mut(id).of_mut(mission_type) else {
            return;
        };
        match list.binary_search_by_key(&seq, |(key, _)| *key) {
            Ok(index) => {
                if let Some(slot) = list.get_mut(index) {
                    slot.1 = item;
                }
            }
            Err(index) => list.insert(index, (seq, item)),
        }
    }

    /// `processInfoFromStream`'s mission and rally half, for one message from `sysid`/`compid`
    /// whose header names `header_target` (`TARGET32`) or none. A message addressed to this
    /// ground station (`gcs_sysid`) or to 0 is about its sender; any other is about the vehicle
    /// it is addressed to - the header's target first, then the payload's - which is how a
    /// recording's own uploads land on the vehicle.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5605-5673, 5691-5702`
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
            // C#: MAVLinkInterface.cs:5615-5627, the list the count names started again.
            MavMessage::MissionCount(m) => {
                self.clear(about(m.target_system, m.target_component), m.mission_type);
            }
            // C#: MAVLinkInterface.cs:5628-5650, `(Locationwp) wp` from the float item; a guided
            // target (current 2) is `GuidedMode`'s, not a list's.
            MavMessage::MissionItem(m) if m.current != 2 => {
                self.store(
                    about(m.target_system, m.target_component),
                    m.mission_type,
                    m.seq,
                    from_float(m),
                );
            }
            // C#: MAVLinkInterface.cs:5651-5673, filed as it came.
            MavMessage::MissionItemInt(m) if m.current != 2 => {
                self.store(
                    about(m.target_system, m.target_component),
                    m.mission_type,
                    m.seq,
                    from_int(m),
                );
            }
            // C#: MAVLinkInterface.cs:5691-5702, the old protocol's point as a RALLY_POINT item in
            // GLOBAL_RELATIVE_ALT under its index, its break altitude, land direction and flags
            // not carried.
            MavMessage::RallyPoint(m) => {
                let item = MissionItem {
                    seq: u16::from(m.idx),
                    current: 0,
                    frame: GLOBAL_RELATIVE_ALT,
                    command: RALLY_POINT,
                    param1: 0.0,
                    param2: 0.0,
                    param3: 0.0,
                    param4: 0.0,
                    x: read_out(RALLY_POINT, m.lat),
                    y: read_out(RALLY_POINT, m.lng),
                    z: f64::from(m.alt),
                    autocontinue: 0,
                };
                self.store(
                    about(m.target_system, m.target_component),
                    MISSION_TYPE_RALLY,
                    u16::from(m.idx),
                    item,
                );
            }
            _ => {}
        }
    }
}

/// An item this link uploaded, as `setWPAsync` files it: `(Locationwp) req` of the
/// `mavlink_mission_item_int_t` it sent, read out again.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4282-4292, 4319-4329`
#[must_use]
pub fn uploaded(item: &MissionItem) -> MissionItem {
    let wire = item.to_wire();
    MissionItem {
        seq: wire.seq,
        current: wire.current,
        frame: wire.frame,
        command: wire.command,
        param1: f64::from(wire.param1),
        param2: f64::from(wire.param2),
        param3: f64::from(wire.param3),
        param4: f64::from(wire.param4),
        x: read_out(wire.command, through_degrees(wire.command, wire.x)),
        y: read_out(wire.command, through_degrees(wire.command, wire.y)),
        z: f64::from(wire.z),
        autocontinue: wire.autocontinue,
    }
}

/// The message that ended a single `setWP`, which decides what the C# files and where.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Finish {
    /// A `MISSION_ACK`, whatever its result: filed under the item's own list (`:4090-4124` for a
    /// float item, `:4264-4299` for an `_INT`).
    Ack,
    /// A `MISSION_REQUEST` for the item after: filed under the item's own list (`:4125-4172`,
    /// `:4300-4346`).
    Request,
    /// A `MISSION_REQUEST_INT` for the item after: a float item is filed in `wps` whatever list
    /// it belongs to - the branch checks no `mission_type` (`:4173-4213`); an `_INT` item has no
    /// such branch and is filed nowhere.
    RequestInt,
}

/// What a single `setWP` files once the vehicle has taken its item - a `MISSION_ITEM` or a
/// `MISSION_ITEM_INT` that is neither a guided target (current 2) nor an altitude change (current
/// 3): the list it goes to, its sequence number and `(Locationwp) req`. `None` for anything else,
/// the fence included, which [`crate::fence_points::set_wp_item`] files.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4107-4117, 4145-4155, 4193-4196,
/// 4282-4292, 4319-4329`
#[must_use]
pub fn set_wp_item(message: &MavMessage, finish: Finish) -> Option<(u8, u16, MissionItem)> {
    match message {
        MavMessage::MissionItem(m) if !matches!(m.current, 2 | 3) => {
            let list = match finish {
                Finish::Ack | Finish::Request => m.mission_type,
                Finish::RequestInt => MISSION_TYPE_MISSION,
            };
            matches!(list, MISSION_TYPE_MISSION | MISSION_TYPE_RALLY)
                .then(|| (list, m.seq, from_float(m)))
        }
        MavMessage::MissionItemInt(m)
            if !matches!(m.current, 2 | 3)
                && finish != Finish::RequestInt
                && matches!(m.mission_type, MISSION_TYPE_MISSION | MISSION_TYPE_RALLY) =>
        {
            Some((m.mission_type, m.seq, uploaded(&from_int(m))))
        }
        _ => None,
    }
}

/// `(Locationwp) wp` of a float item, filed as an `_INT` and read out: a position through
/// `(int)(lat * 1e7)` and back, anything else through `(int)`.
fn from_float(m: &MissionItemMessage) -> MissionItem {
    MissionItem {
        seq: m.seq,
        current: m.current,
        frame: m.frame,
        command: m.command,
        param1: f64::from(m.param1),
        param2: f64::from(m.param2),
        param3: f64::from(m.param3),
        param4: f64::from(m.param4),
        x: read_out(m.command, from_degrees(m.command, f64::from(m.x))),
        y: read_out(m.command, from_degrees(m.command, f64::from(m.y))),
        z: f64::from(m.z),
        autocontinue: m.autocontinue,
    }
}

/// An `_INT` item as it came, read out.
fn from_int(m: &MissionItemInt) -> MissionItem {
    MissionItem {
        seq: m.seq,
        current: m.current,
        frame: m.frame,
        command: m.command,
        param1: f64::from(m.param1),
        param2: f64::from(m.param2),
        param3: f64::from(m.param3),
        param4: f64::from(m.param4),
        x: read_out(m.command, m.x),
        y: read_out(m.command, m.y),
        z: f64::from(m.z),
        autocontinue: m.autocontinue,
    }
}

/// `(Locationwp) item`'s coordinate from the filed `mavlink_mission_item_int_t`: a position's
/// divided by 1e7, anything else as it is. `// C#: ExtLibs/Utilities/locationwp.cs:96-119`
fn read_out(command: u16, value: i32) -> f64 {
    if is_location(command) {
        f64::from(value) / 1e7
    } else {
        f64::from(value)
    }
}

#[cfg(test)]
mod tests {
    use mp_mavlink_dialects::all::{MissionCount, RallyPoint};

    use super::*;

    const VEHICLE: VehicleId = VehicleId::new(1, 1);
    const GCS: u8 = 255;
    const WAYPOINT: u16 = 16;

    fn item_int(seq: u16, mission_type: u8, x: i32) -> MavMessage {
        MavMessage::MissionItemInt(MissionItemInt {
            param1: 3.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x,
            y: x + 1,
            z: 50.0,
            seq,
            command: WAYPOINT,
            target_system: GCS,
            target_component: 190,
            frame: 3,
            current: 0,
            autocontinue: 1,
            mission_type,
        })
    }

    fn item_float(seq: u16, mission_type: u8, current: u8) -> MavMessage {
        MavMessage::MissionItem(MissionItemMessage {
            param1: 15.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: -35.36,
            y: 149.16,
            z: 20.0,
            seq,
            command: WAYPOINT,
            target_system: GCS,
            target_component: 190,
            frame: 3,
            current,
            autocontinue: 1,
            mission_type,
        })
    }

    #[test]
    fn stream_items_are_filed_by_list_under_the_vehicle_they_are_about_in_order() {
        let mut points = MissionPoints::default();
        // A download's answers, addressed to this ground station, out of order.
        points.observe(
            1,
            1,
            u32::from(GCS),
            None,
            &item_int(2, MISSION_TYPE_MISSION, -353_630_000),
        );
        points.observe(
            1,
            1,
            u32::from(GCS),
            None,
            &item_int(0, MISSION_TYPE_MISSION, -353_610_000),
        );
        points.observe(
            1,
            1,
            u32::from(GCS),
            None,
            &item_int(1, MISSION_TYPE_MISSION, -353_620_000),
        );
        let xs: Vec<f64> = points.wps(VEHICLE).iter().map(|item| item.x).collect();
        assert_eq!(xs, [-35.361, -35.362, -35.363]);
        assert_eq!(points.wps(VEHICLE)[1].z, 50.0);
        // Filed again: replaced, not added.
        points.observe(
            1,
            1,
            u32::from(GCS),
            None,
            &item_int(1, MISSION_TYPE_MISSION, -353_625_000),
        );
        assert_eq!(points.wps(VEHICLE).len(), 3);
        assert_eq!(points.wps(VEHICLE)[1].x, -35.3625);
        // A rally item goes to the rally list, a fence item to neither (fence_points has it).
        points.observe(
            1,
            1,
            u32::from(GCS),
            None,
            &item_int(0, MISSION_TYPE_RALLY, -353_000_000),
        );
        points.observe(1, 1, u32::from(GCS), None, &item_int(0, 1, -352_000_000));
        assert_eq!(points.rally_points(VEHICLE).len(), 1);
        assert_eq!(points.rally_points(VEHICLE)[0].x, -35.3);
        assert_eq!(points.wps(VEHICLE).len(), 3);
        assert!(points.items(VEHICLE, 1).is_empty());
        // A guided target is GuidedMode's.
        let MavMessage::MissionItemInt(mut guided) = item_int(7, MISSION_TYPE_MISSION, 1) else {
            unreachable!()
        };
        guided.current = 2;
        points.observe(
            1,
            1,
            u32::from(GCS),
            None,
            &MavMessage::MissionItemInt(guided),
        );
        assert_eq!(points.wps(VEHICLE).len(), 3);
        // A ground station's item, as a recording holds it, is about the vehicle it addresses.
        let MavMessage::MissionItemInt(mut theirs) = item_int(0, MISSION_TYPE_MISSION, 40) else {
            unreachable!()
        };
        theirs.target_system = 7;
        points.observe(
            u32::from(GCS),
            190,
            u32::from(GCS),
            None,
            &MavMessage::MissionItemInt(theirs),
        );
        assert_eq!(points.wps(VehicleId::new(7, 190)).len(), 1);
        // A MISSION_COUNT starts the list it names again, and only that one.
        let count = |mission_type: u8| {
            MavMessage::MissionCount(MissionCount {
                count: 3,
                target_system: GCS,
                target_component: 190,
                mission_type,
            })
        };
        points.observe(1, 1, u32::from(GCS), None, &count(1));
        assert_eq!(points.wps(VEHICLE).len(), 3);
        points.observe(1, 1, u32::from(GCS), None, &count(MISSION_TYPE_MISSION));
        assert!(points.wps(VEHICLE).is_empty());
        assert_eq!(points.rally_points(VEHICLE).len(), 1);
        assert_eq!(points.wps(VehicleId::new(7, 190)).len(), 1);
        points.observe(1, 1, u32::from(GCS), None, &count(MISSION_TYPE_RALLY));
        assert!(points.rally_points(VEHICLE).is_empty());
    }

    #[test]
    fn an_item_to_no_one_is_its_senders_and_a_header_target_comes_before_the_payloads() {
        // `processInfoFromStream` as of 5dbb2b0 (e6454ccdd): 0 is the sender, as this ground
        // station is, not a vehicle 0. C#: MAVLinkInterface.cs:5607-5614
        let mut points = MissionPoints::default();
        let MavMessage::MissionItemInt(mut to_no_one) = item_int(0, MISSION_TYPE_MISSION, 40)
        else {
            unreachable!()
        };
        to_no_one.target_system = 0;
        points.observe(
            1,
            1,
            u32::from(GCS),
            None,
            &MavMessage::MissionItemInt(to_no_one),
        );
        assert_eq!(points.wps(VEHICLE).len(), 1);
        assert!(points.wps(VehicleId::new(0, 190)).is_empty());
        // A recorded upload to a vehicle whose id is over 255: the header's target, not the
        // payload's 255, which would otherwise be this ground station and so the sender.
        let wide = VehicleId::new(70_000, 190);
        points.observe(
            u32::from(GCS),
            190,
            u32::from(GCS),
            Some(wide.sysid),
            &item_int(0, MISSION_TYPE_MISSION, 41),
        );
        assert_eq!(points.wps(wide).len(), 1);
        assert!(points.wps(VehicleId::new(u32::from(GCS), 190)).is_empty());
        // A count in the header's name starts that vehicle's list again.
        points.observe(
            u32::from(GCS),
            190,
            u32::from(GCS),
            Some(wide.sysid),
            &MavMessage::MissionCount(MissionCount {
                count: 0,
                target_system: GCS,
                target_component: 190,
                mission_type: MISSION_TYPE_MISSION,
            }),
        );
        assert!(points.wps(wide).is_empty());
        assert_eq!(points.wps(VEHICLE).len(), 1);
    }

    #[test]
    fn a_float_item_goes_through_locationwp_and_an_old_rally_point_becomes_an_item() {
        let mut points = MissionPoints::default();
        points.observe(
            1,
            1,
            u32::from(GCS),
            None,
            &item_float(0, MISSION_TYPE_MISSION, 0),
        );
        let [filed] = points.wps(VEHICLE)[..] else {
            panic!("one item")
        };
        // `(int)(lat * 1e7)` of the float as a double, then back out over 1e7.
        #[allow(clippy::cast_possible_truncation)]
        let expected = f64::from((f64::from(-35.36_f32) * 1e7) as i32) / 1e7;
        assert_eq!(filed.x, expected);
        assert_eq!(
            (filed.command, filed.param1, filed.z),
            (WAYPOINT, 15.0, 20.0)
        );
        // A guided target or an altitude change from the stream: current 2 is GuidedMode's and
        // not filed; current 3 the stream files, as its `else` takes everything but 2.
        points.observe(
            1,
            1,
            u32::from(GCS),
            None,
            &item_float(1, MISSION_TYPE_MISSION, 2),
        );
        assert_eq!(points.wps(VEHICLE).len(), 1);
        points.observe(
            1,
            1,
            u32::from(GCS),
            None,
            &item_float(1, MISSION_TYPE_MISSION, 3),
        );
        assert_eq!(points.wps(VEHICLE).len(), 2);

        points.observe(
            1,
            1,
            u32::from(GCS),
            None,
            &MavMessage::RallyPoint(RallyPoint {
                lat: -353_632_621,
                lng: 1_491_652_374,
                alt: 100,
                break_alt: 40,
                land_dir: 0,
                target_system: GCS,
                target_component: 190,
                idx: 1,
                count: 2,
                flags: 0,
            }),
        );
        let [rally] = points.rally_points(VEHICLE)[..] else {
            panic!("one rally point")
        };
        assert_eq!((rally.seq, rally.command, rally.frame), (1, RALLY_POINT, 3));
        assert_eq!(
            (rally.x, rally.y, rally.z),
            (-35.3632621, 149.1652374, 100.0)
        );
        assert_eq!((rally.param1, rally.autocontinue), (0.0, 0));
    }

    #[test]
    fn a_single_set_wp_is_filed_by_the_message_that_ended_it() {
        // A float mission item: its own list on an ack or a request, wps on a request-int.
        let mission = item_float(4, MISSION_TYPE_MISSION, 0);
        let rally = item_float(2, MISSION_TYPE_RALLY, 0);
        let fence = item_float(0, 1, 0);
        let list = |filed: Option<(u8, u16, MissionItem)>| filed.map(|(list, seq, _)| (list, seq));
        assert_eq!(list(set_wp_item(&mission, Finish::Ack)), Some((0, 4)));
        assert_eq!(list(set_wp_item(&mission, Finish::Request)), Some((0, 4)));
        assert_eq!(
            list(set_wp_item(&mission, Finish::RequestInt)),
            Some((0, 4))
        );
        assert_eq!(list(set_wp_item(&rally, Finish::Ack)), Some((2, 2)));
        assert_eq!(list(set_wp_item(&rally, Finish::Request)), Some((2, 2)));
        // The float path's MISSION_REQUEST_INT branch files wps whatever the list (:4206).
        assert_eq!(list(set_wp_item(&rally, Finish::RequestInt)), Some((0, 2)));
        // The fence is fence_points' - but that same branch puts a fence item in wps.
        assert_eq!(list(set_wp_item(&fence, Finish::Ack)), None);
        assert_eq!(list(set_wp_item(&fence, Finish::RequestInt)), Some((0, 0)));
        // A guided target and an altitude change are filed nowhere.
        assert_eq!(set_wp_item(&item_float(1, 0, 2), Finish::Ack), None);
        assert_eq!(set_wp_item(&item_float(1, 0, 3), Finish::Request), None);
        // An _INT item: its own list on an ack or a request, nowhere on a request-int.
        let int = item_int(3, MISSION_TYPE_MISSION, -353_650_000);
        assert_eq!(list(set_wp_item(&int, Finish::Ack)), Some((0, 3)));
        assert_eq!(list(set_wp_item(&int, Finish::Request)), Some((0, 3)));
        assert_eq!(set_wp_item(&int, Finish::RequestInt), None);
        // `(Locationwp) req` of the _INT item and back: a coordinate whose trip through degrees
        // does not land on itself moves a unit toward zero (fence_points's example).
        let (_, _, filed) = set_wp_item(&int, Finish::Ack).expect("filed");
        assert_eq!(filed.x, -35.364_999_9);
    }

    #[test]
    fn an_uploaded_item_is_filed_as_its_wire_form_read_back() {
        let mut item = MissionItem {
            command: WAYPOINT,
            seq: 2,
            ..MissionItem::default()
        };
        item.x = -35.363_262_1;
        item.y = 149.165_237_4;
        item.z = 584.09;
        let filed = uploaded(&item);
        assert_eq!((filed.seq, filed.command), (2, WAYPOINT));
        assert_eq!((filed.x, filed.y), (-35.363_262_1, 149.165_237_4));
        // z is a float on the wire.
        assert_eq!(filed.z, f64::from(584.09_f32));
        // Not a position: the raw value, untouched.
        let mut servo = MissionItem {
            command: 183,
            ..MissionItem::default()
        };
        servo.x = 12_345.0;
        assert_eq!(uploaded(&servo).x, 12_345.0);
    }
}
