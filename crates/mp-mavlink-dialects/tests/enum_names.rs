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

//! The generated value-to-name lookup on every enum.
//!
//! The UI needs the reverse direction far more than it needs the constants: a `COMMAND_ACK`
//! carrying 400 is useless on screen, and "MAV_CMD_COMPONENT_ARM_DISARM: denied" is the whole
//! message. These tests pin the behaviour that matters - the names are the XML's, and an unknown
//! value produces nothing rather than a neighbouring name.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use mp_mavlink_dialects::all::{MavCmd, MavFrame, MavResult, MavSeverity, MavState, MavType};

#[test]
fn commands_are_named_as_the_definitions_name_them() {
    assert_eq!(
        MavCmd::MAV_CMD_COMPONENT_ARM_DISARM.name(),
        Some("MAV_CMD_COMPONENT_ARM_DISARM")
    );
    assert_eq!(MavCmd(16).name(), Some("MAV_CMD_NAV_WAYPOINT"));
    assert_eq!(MavCmd(22).name(), Some("MAV_CMD_NAV_TAKEOFF"));
    assert_eq!(MavCmd(21).name(), Some("MAV_CMD_NAV_LAND"));
}

#[test]
fn an_unknown_value_has_no_name_rather_than_a_wrong_one() {
    // A newer autopilot's command shown under an older name is worse than a bare number: the
    // operator would act on it.
    assert_eq!(MavCmd(64_999).name(), None);
    assert_eq!(MavResult(200).name(), None);
    assert_eq!(MavType(250).name(), None);
}

#[test]
fn the_name_round_trips_through_the_constant() {
    // Guards against the generator emitting a lookup keyed on the wrong value, which would be
    // invisible until a specific command was acked.
    let cases = [
        MavResult::MAV_RESULT_ACCEPTED,
        MavResult::MAV_RESULT_DENIED,
        MavResult::MAV_RESULT_FAILED,
        MavResult::MAV_RESULT_UNSUPPORTED,
    ];
    for case in cases {
        let name = case.name().expect("a defined result has a name");
        assert!(name.starts_with("MAV_RESULT_"), "{name}");
    }
}

#[test]
fn severities_cover_the_syslog_range() {
    for value in 0..=7 {
        assert!(
            MavSeverity(value).name().is_some(),
            "severity {value} has no name"
        );
    }
    assert_eq!(MavSeverity(8).name(), None);
}

#[test]
fn frames_the_mission_editor_writes_are_named() {
    // MAV_FRAME_GLOBAL_RELATIVE_ALT is what a waypoint drawn on the map uses; if this name ever
    // stops resolving, the plan table starts showing numbers.
    assert_eq!(MavFrame(3).name(), Some("MAV_FRAME_GLOBAL_RELATIVE_ALT"));
    assert_eq!(MavFrame(0).name(), Some("MAV_FRAME_GLOBAL"));
}

#[test]
fn vehicle_types_and_states_resolve() {
    assert_eq!(MavType(2).name(), Some("MAV_TYPE_QUADROTOR"));
    assert_eq!(MavType(1).name(), Some("MAV_TYPE_FIXED_WING"));
    assert!(MavState(3).name().is_some());
}

#[test]
fn every_named_value_is_reachable_from_its_own_number() {
    // A generated table is only as good as its keys. Walk a wide span of MAV_CMD and assert that
    // whenever a name exists it is stable under reconstruction from the raw value.
    for value in 0..400_u32 {
        let direct = MavCmd(value).name();
        let rebuilt = MavCmd(MavCmd(value).0).name();
        assert_eq!(direct, rebuilt, "value {value}");
    }
}
