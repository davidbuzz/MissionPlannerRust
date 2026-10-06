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

//! The mission items the planning screen's map menu creates, as the C# fills their rows.
//!
//! Each constructor is one handler in `GCSViews/FlightPlanner.cs`: which `MAV_CMD` it puts in the
//! Command column, which parameter columns it writes, and where the position and altitude come
//! from. What the handler leaves alone is what `Commands_RowsAdded` put in a new row - `"0"` in
//! every text cell and `CMB_altmode`'s frame in the Frame column
//! (`GCSViews/FlightPlanner.cs:2407-2428`) - so it is zero here, and the frame is the caller's.
//!
//! A position is written only where the C#'s `setfromMap` writes one. It writes latitude and
//! longitude when the column headers `ChangeColumnHeader` has just set for that command read
//! "Lat" and "Long", and altitude when the header starts with "Alt" (`FlightPlanner.cs:1150-1170`).
//! Those headers come from `mavcmd.xml`, and every command here has the same three in all three
//! of its vehicle sections - except the ones that never call `setfromMap` at all.

use mp_units::LatLon;

use crate::item::MissionItem;

/// `MAV_CMD_NAV_WAYPOINT`.
pub const WAYPOINT: u16 = 16;
/// `MAV_CMD_NAV_LOITER_UNLIM`.
pub const LOITER_UNLIM: u16 = 17;
/// `MAV_CMD_NAV_LOITER_TURNS`.
pub const LOITER_TURNS: u16 = 18;
/// `MAV_CMD_NAV_LOITER_TIME`.
pub const LOITER_TIME: u16 = 19;
/// `MAV_CMD_NAV_RETURN_TO_LAUNCH`.
pub const RETURN_TO_LAUNCH: u16 = 20;
/// `MAV_CMD_NAV_LAND`.
pub const LAND: u16 = 21;
/// `MAV_CMD_NAV_TAKEOFF`.
pub const TAKEOFF: u16 = 22;
/// `MAV_CMD_NAV_SPLINE_WAYPOINT`.
pub const SPLINE_WAYPOINT: u16 = 82;
/// `MAV_CMD_DO_JUMP`.
pub const DO_JUMP: u16 = 177;
/// `MAV_CMD_DO_SET_ROI`.
pub const DO_SET_ROI: u16 = 201;

/// The altitude `landToolStripMenuItem_Click` hands `setfromMap`.
///
/// `setfromMap(MouseDownEnd.Lat, MouseDownEnd.Lng, 1)`, and a non-zero altitude is written as
/// given (`if (alt != 0) cell.Value = alt.ToString();`), so a land from the menu is at 1.
/// `// C#: GCSViews/FlightPlanner.cs:4313, 1195-1196`
pub const LAND_ALTITUDE: f64 = 1.0;

/// A row as `Commands_RowsAdded` leaves it: this command, zeroes, the given frame.
const fn row(command: u16, frame: u8) -> MissionItem {
    MissionItem {
        seq: 0,
        current: 0,
        frame,
        command,
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
        autocontinue: 1,
    }
}

/// A row with a position and altitude, as `setfromMap` fills one.
fn placed(command: u16, frame: u8, position: LatLon, altitude: f64) -> MissionItem {
    MissionItem {
        x: position.latitude(),
        y: position.longitude(),
        z: altitude,
        ..row(command, frame)
    }
}

/// `WAYPOINT` at a position: a click on the map, or Insert Wp.
///
/// `// C#: GCSViews/FlightPlanner.cs:593-598 (AddWPToMap), 4070-4091 (insertWpToolStripMenuItem_Click)`
#[must_use]
pub fn waypoint(position: LatLon, altitude: f64, frame: u8) -> MissionItem {
    placed(WAYPOINT, frame, position, altitude)
}

/// `SPLINE_WAYPOINT` at a position: Insert Spline WP.
///
/// `// C#: GCSViews/FlightPlanner.cs:4035-4067`
#[must_use]
pub fn spline_waypoint(position: LatLon, altitude: f64, frame: u8) -> MissionItem {
    placed(SPLINE_WAYPOINT, frame, position, altitude)
}

/// `LOITER_UNLIM` at a position: Loiter > Forever. No prompt, no parameters.
///
/// `// C#: GCSViews/FlightPlanner.cs:4763-4774`
#[must_use]
pub fn loiter_unlimited(position: LatLon, altitude: f64, frame: u8) -> MissionItem {
    placed(LOITER_UNLIM, frame, position, altitude)
}

/// `LOITER_TIME` at a position, `Param1` the time asked for: Loiter > Time.
///
/// `// C#: GCSViews/FlightPlanner.cs:4776-4794`
#[must_use]
pub fn loiter_time(position: LatLon, altitude: f64, frame: u8, seconds: f64) -> MissionItem {
    MissionItem {
        param1: seconds,
        ..placed(LOITER_TIME, frame, position, altitude)
    }
}

/// `LOITER_TURNS` at a position, `Param1` the turns asked for: Loiter > Circles.
///
/// `// C#: GCSViews/FlightPlanner.cs:4743-4761`
#[must_use]
pub fn loiter_turns(position: LatLon, altitude: f64, frame: u8, turns: f64) -> MissionItem {
    MissionItem {
        param1: turns,
        ..placed(LOITER_TURNS, frame, position, altitude)
    }
}

/// `DO_JUMP`, `Param1` the item to jump to and `Param2` the repeat count: Jump > Start (to item
/// 1) and Jump > WP #. No position: neither handler calls `setfromMap`.
///
/// `// C#: GCSViews/FlightPlanner.cs:4094-4109 (Start), 4111-4129 (WP #)`
#[must_use]
pub const fn do_jump(target: f64, repeat: f64, frame: u8) -> MissionItem {
    MissionItem {
        param1: target,
        param2: repeat,
        ..row(DO_JUMP, frame)
    }
}

/// `RETURN_TO_LAUNCH`: RTL. The handler sets the command and nothing else.
///
/// `// C#: GCSViews/FlightPlanner.cs:5873-5884`
#[must_use]
pub const fn return_to_launch(frame: u8) -> MissionItem {
    row(RETURN_TO_LAUNCH, frame)
}

/// `LAND` at a position, at [`LAND_ALTITUDE`]: Land.
///
/// `// C#: GCSViews/FlightPlanner.cs:4302-4316`
#[must_use]
pub fn land(position: LatLon, frame: u8) -> MissionItem {
    placed(LAND, frame, position, LAND_ALTITUDE)
}

/// `TAKEOFF` with the altitude and pitch asked for: Takeoff. The handler writes `Param1` and the
/// Alt column directly and never calls `setfromMap`, so there is no position.
///
/// `// C#: GCSViews/FlightPlanner.cs:6777-6834`
#[must_use]
pub const fn takeoff(altitude: f64, pitch: f64, frame: u8) -> MissionItem {
    MissionItem {
        param1: pitch,
        z: altitude,
        ..row(TAKEOFF, frame)
    }
}

/// `DO_SET_ROI` at a position: the region of interest.
///
/// `// C#: GCSViews/FlightPlanner.cs:6670-6690`
#[must_use]
pub fn set_roi(position: LatLon, altitude: f64, frame: u8) -> MissionItem {
    placed(DO_SET_ROI, frame, position, altitude)
}

#[cfg(test)]
mod tests {
    use super::*;

    const RELATIVE: u8 = 3;

    fn canberra() -> LatLon {
        LatLon::new(-35.363_262, 149.165_237).expect("valid")
    }

    /// A placed item carries the position, the altitude and the frame, and no parameters.
    #[test]
    fn a_placed_item_is_where_it_was_put_and_nothing_else() {
        for (item, command) in [
            (waypoint(canberra(), 100.0, RELATIVE), WAYPOINT),
            (
                spline_waypoint(canberra(), 100.0, RELATIVE),
                SPLINE_WAYPOINT,
            ),
            (loiter_unlimited(canberra(), 100.0, RELATIVE), LOITER_UNLIM),
            (set_roi(canberra(), 100.0, RELATIVE), DO_SET_ROI),
        ] {
            assert_eq!(item.command, command);
            assert_eq!(item.position(), Ok(Some(canberra())));
            assert!((item.z - 100.0).abs() < f64::EPSILON);
            assert_eq!(item.frame, RELATIVE);
            assert_eq!(
                (item.param1, item.param2, item.param3, item.param4),
                (0.0, 0.0, 0.0, 0.0),
                "command {command} should carry no parameters"
            );
            assert_eq!(item.autocontinue, 1);
        }
    }

    /// Loiter > Time and Loiter > Circles put what was typed in `Param1`, and only there.
    #[test]
    fn a_loiter_carries_its_time_or_turns_in_param1() {
        let time = loiter_time(canberra(), 60.0, RELATIVE, 5.0);
        assert_eq!(time.command, LOITER_TIME);
        assert!((time.param1 - 5.0).abs() < f64::EPSILON);
        assert_eq!((time.param2, time.param3, time.param4), (0.0, 0.0, 0.0));
        assert_eq!(time.position(), Ok(Some(canberra())));

        let turns = loiter_turns(canberra(), 60.0, RELATIVE, 3.0);
        assert_eq!(turns.command, LOITER_TURNS);
        assert!((turns.param1 - 3.0).abs() < f64::EPSILON);
        assert_eq!((turns.param2, turns.param3, turns.param4), (0.0, 0.0, 0.0));
    }

    /// A jump names its target in `Param1` and its repeat count in `Param2`, and has no position -
    /// a jump drawn on the map at 0,0 would be a waypoint off the coast of Africa.
    #[test]
    fn a_jump_has_a_target_and_a_repeat_count_and_no_position() {
        let jump = do_jump(1.0, 5.0, RELATIVE);
        assert_eq!(jump.command, DO_JUMP);
        assert!((jump.param1 - 1.0).abs() < f64::EPSILON);
        assert!((jump.param2 - 5.0).abs() < f64::EPSILON);
        assert_eq!(jump.position(), Ok(None));
        assert_eq!((jump.x, jump.y, jump.z), (0.0, 0.0, 0.0));
    }

    /// RTL is the command and nothing else.
    #[test]
    fn a_return_to_launch_is_only_its_command() {
        let rtl = return_to_launch(RELATIVE);
        assert_eq!(rtl.command, RETURN_TO_LAUNCH);
        assert_eq!(rtl.position(), Ok(None));
        assert_eq!(rtl.z, 0.0);
        assert_eq!(rtl.frame, RELATIVE);
    }

    /// Land from the menu is at the click, one metre up, as `setfromMap(..., 1)` leaves it.
    #[test]
    fn a_land_from_the_menu_is_at_one_metre() {
        let item = land(canberra(), RELATIVE);
        assert_eq!(item.command, LAND);
        assert_eq!(item.position(), Ok(Some(canberra())));
        assert!((item.z - 1.0).abs() < f64::EPSILON);
    }

    /// Takeoff writes the altitude and the pitch and never a position.
    #[test]
    fn a_takeoff_has_an_altitude_and_a_pitch_and_no_position() {
        let item = takeoff(10.0, 15.0, RELATIVE);
        assert_eq!(item.command, TAKEOFF);
        assert!((item.z - 10.0).abs() < f64::EPSILON);
        assert!((item.param1 - 15.0).abs() < f64::EPSILON);
        assert_eq!(item.position(), Ok(None));
    }

    /// Every item a menu makes survives a `.waypoints` round trip with its command and parameters.
    #[test]
    fn menu_items_survive_a_waypoints_file() {
        let items = vec![
            waypoint(canberra(), 100.0, RELATIVE),
            loiter_time(canberra(), 100.0, RELATIVE, 5.0),
            loiter_turns(canberra(), 100.0, RELATIVE, 3.0),
            do_jump(2.0, 5.0, RELATIVE),
            takeoff(10.0, 0.0, RELATIVE),
            return_to_launch(RELATIVE),
        ]
        .into_iter()
        .enumerate()
        .map(|(seq, item)| MissionItem {
            seq: u16::try_from(seq).unwrap_or(u16::MAX),
            ..item
        })
        .collect::<Vec<_>>();
        let read = crate::read_waypoints(&crate::write_waypoints(&items)).expect("parses");
        assert_eq!(read.len(), items.len());
        for (read, written) in read.iter().zip(&items) {
            assert_eq!(read.command, written.command);
            assert!((read.param1 - written.param1).abs() < 1e-6);
            assert!((read.param2 - written.param2).abs() < 1e-6);
        }
    }
}
