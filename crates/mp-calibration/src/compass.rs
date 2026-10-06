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

//! Compass calibration: the commands the Compass page sends, and what the vehicle says back.
//!
//! The protocol half of `GCSViews/ConfigurationView/ConfigHWCompass2.cs` (and of the older
//! `ConfigHWCompass.cs`, whose onboard calibration is the same code): the three commands of the
//! Onboard Mag Calibration group, Large Vehicle MagCal's one, and the two lists the page's timer
//! reads - every `MAG_CAL_PROGRESS` and every `MAG_CAL_REPORT` since Start - reduced to what the
//! timer takes from them. The link thread feeds a [`MagCalLog`] as the messages arrive; the page
//! reads it.

use mp_mavlink_dialects::all::{MagCalProgress, MagCalReport, MavMessage};
use mp_vehicle::VehicleId;

use super::command;

/// `MAV_CMD_FIXED_MAG_CAL_YAW`.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:1429`
pub const CMD_FIXED_MAG_CAL_YAW: u16 = 42_006;
/// `MAV_CMD_DO_START_MAG_CAL`.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:1435`
pub const CMD_DO_START_MAG_CAL: u16 = 42_424;
/// `MAV_CMD_DO_ACCEPT_MAG_CAL`.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:1438`
pub const CMD_DO_ACCEPT_MAG_CAL: u16 = 42_425;
/// `MAV_CMD_DO_CANCEL_MAG_CAL`.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:1441`
pub const CMD_DO_CANCEL_MAG_CAL: u16 = 42_426;

/// Start: an onboard calibration of every compass, `0, 1, 1, 0, 0, 0, 0`.
///
/// `MAV_CMD_DO_START_MAG_CAL`, not `MAV_CMD_PREFLIGHT_CALIBRATION`. The latter's magnetometer
/// parameter is the legacy offset calibration; it is accepted by the firmware and never produces a
/// `MAG_CAL_PROGRESS` message, so a page that sends it shows progress that never moves.
///
/// param1 = 0 calibrates every compass; param2 = 1 retries on failure; param3 = 1 saves the result
/// without waiting for Accept.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:273`
#[must_use]
pub fn start_compass(target: VehicleId) -> MavMessage {
    command(
        target,
        CMD_DO_START_MAG_CAL,
        [0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    )
}

/// Accept: `MAV_CMD_DO_ACCEPT_MAG_CAL`, `0, 0, 1, 0, 0, 0, 0`.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:327`
#[must_use]
pub fn accept_compass(target: VehicleId) -> MavMessage {
    command(
        target,
        CMD_DO_ACCEPT_MAG_CAL,
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    )
}

/// Cancel: `MAV_CMD_DO_CANCEL_MAG_CAL`, `0, 0, 1, 0, 0, 0, 0` - param3 is 1, as the C# sends it.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:345`
#[must_use]
pub fn cancel_compass(target: VehicleId) -> MavMessage {
    command(
        target,
        CMD_DO_CANCEL_MAG_CAL,
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    )
}

/// Large Vehicle MagCal: `MAV_CMD_FIXED_MAG_CAL_YAW` with the heading the operator typed, in
/// degrees, true rather than magnetic, and the rest zero.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:483-484`
#[must_use]
pub fn fixed_mag_cal_yaw(target: VehicleId, yaw_degrees: f32) -> MavMessage {
    command(
        target,
        CMD_FIXED_MAG_CAL_YAW,
        [yaw_degrees, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
}

/// A `MAG_CAL_STATUS` as `((MAVLink.MAG_CAL_STATUS)value).ToString()` writes it: the enum's name,
/// or the number for a value the C#'s enum does not name (ArduPilot's `FAILED_OFFSETS` and later).
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:6500-6511; GCSViews/ConfigurationView/ConfigHWCompass2.cs:418`
#[must_use]
pub fn mag_cal_status_name(value: u8) -> String {
    match value {
        0 => "MAG_CAL_NOT_STARTED",
        1 => "MAG_CAL_WAITING_TO_START",
        2 => "MAG_CAL_RUNNING_STEP_ONE",
        3 => "MAG_CAL_RUNNING_STEP_TWO",
        4 => "MAG_CAL_SUCCESS",
        5 => "MAG_CAL_FAILED",
        6 => "MAG_CAL_BAD_ORIENTATION",
        7 => "MAG_CAL_BAD_RADIUS",
        other => return other.to_string(),
    }
    .to_owned()
}

/// `MAG_CAL_SUCCESS`.
pub const MAG_CAL_SUCCESS: u8 = 4;

/// The last `MAG_CAL_PROGRESS` heard for one compass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompassProgress {
    /// Which compass, as the vehicle numbers them.
    pub compass_id: u8,
    /// `cal_status`, a `MAG_CAL_STATUS`.
    pub status: u8,
    /// `attempt`.
    pub attempt: u8,
    /// `completion_pct`, zero to a hundred.
    pub percent: u8,
}

/// The last `MAG_CAL_REPORT` heard for one compass: what the page's text box says of it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompassReport {
    /// Which compass.
    pub compass_id: u8,
    /// `ofs_x`, `ofs_y`, `ofs_z`.
    pub offsets: [f32; 3],
    /// `fitness`: the residual after fitting, lower being better.
    pub fitness: f32,
    /// `cal_status`, a `MAG_CAL_STATUS`.
    pub status: u8,
    /// `autosaved`: 1 when the vehicle saved the result without waiting for Accept.
    pub autosaved: bool,
}

/// What the vehicle has said about a compass calibration since it was last cleared: the C#'s
/// `mprog` and `mrep` lists, as `timer1_Tick` reads them.
///
/// The tick keeps the last message per compass in a `Dictionary<byte, ...>` built afresh from the
/// whole list, and a dictionary that is only added to enumerates in insertion order - so each list
/// here holds one entry per compass, in the order each compass was first heard, holding the last
/// message heard. A report for compass 0 whose `ofs_x` is zero is passed over, as the tick passes
/// it over, before it can take a place or replace one.
/// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:282-283, 296-321, 364-371, 396-408`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MagCalLog {
    /// The last progress message per compass, first-heard order.
    pub progress: Vec<CompassProgress>,
    /// The last report per compass, first-heard order.
    pub reports: Vec<CompassReport>,
}

impl MagCalLog {
    /// Takes a `MAG_CAL_PROGRESS`.
    pub fn observe_progress(&mut self, message: &MagCalProgress) {
        let entry = CompassProgress {
            compass_id: message.compass_id,
            status: message.cal_status,
            attempt: message.attempt,
            percent: message.completion_pct,
        };
        match self
            .progress
            .iter_mut()
            .find(|held| held.compass_id == entry.compass_id)
        {
            Some(held) => *held = entry,
            None => self.progress.push(entry),
        }
    }

    /// Takes a `MAG_CAL_REPORT`.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWCompass2.cs:404-407`
    pub fn observe_report(&mut self, message: &MagCalReport) {
        if message.compass_id == 0 && message.ofs_x == 0.0 {
            return;
        }
        let entry = CompassReport {
            compass_id: message.compass_id,
            offsets: [message.ofs_x, message.ofs_y, message.ofs_z],
            fitness: message.fitness,
            status: message.cal_status,
            autosaved: message.autosaved == 1,
        };
        match self
            .reports
            .iter_mut()
            .find(|held| held.compass_id == entry.compass_id)
        {
            Some(held) => *held = entry,
            None => self.reports.push(entry),
        }
    }

    /// Whether nothing has been heard.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.progress.is_empty() && self.reports.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The seven parameters of a built command.
    fn params(message: &MavMessage) -> [f32; 7] {
        match message {
            MavMessage::CommandLong(c) => [
                c.param1, c.param2, c.param3, c.param4, c.param5, c.param6, c.param7,
            ],
            other => panic!("expected a COMMAND_LONG, got {}", other.name()),
        }
    }

    /// The command id and target of a built command.
    fn command_of(message: &MavMessage) -> (u16, u8, u8) {
        match message {
            MavMessage::CommandLong(c) => (c.command, c.target_system, c.target_component),
            other => panic!("expected a COMMAND_LONG, got {}", other.name()),
        }
    }

    fn progress(compass_id: u8, status: u8, percent: u8) -> MagCalProgress {
        MagCalProgress {
            direction_x: 0.0,
            direction_y: 0.0,
            direction_z: 0.0,
            compass_id,
            cal_mask: 7,
            cal_status: status,
            attempt: 1,
            completion_pct: percent,
            completion_mask: [0; 10],
        }
    }

    fn report(compass_id: u8, ofs_x: f32, status: u8, autosaved: u8) -> MagCalReport {
        MagCalReport {
            fitness: 3.25,
            ofs_x,
            ofs_y: -2.0,
            ofs_z: 7.5,
            diag_x: 1.0,
            diag_y: 1.0,
            diag_z: 1.0,
            offdiag_x: 0.0,
            offdiag_y: 0.0,
            offdiag_z: 0.0,
            compass_id,
            cal_mask: 7,
            cal_status: status,
            autosaved,
            orientation_confidence: 0.0,
            old_orientation: 0,
            new_orientation: 0,
            scale_factor: 1.0,
        }
    }

    /// Start, Accept and Cancel are the C#'s three `doCommand`s, parameter for parameter.
    #[test]
    fn the_onboard_calibration_commands_are_the_csharps() {
        let id = VehicleId::new(1, 1);
        let start = start_compass(id);
        assert_eq!(command_of(&start), (CMD_DO_START_MAG_CAL, 1, 1));
        assert_eq!(params(&start), [0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]);

        let accept = accept_compass(id);
        assert_eq!(command_of(&accept), (CMD_DO_ACCEPT_MAG_CAL, 1, 1));
        assert_eq!(params(&accept), [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]);
    }

    /// Cancel sends param3 = 1, as `BUT_OBmagcalcancel_Click` does. This sent 0 until the page was
    /// ported whole, which the C# never sends.
    #[test]
    fn cancel_sends_param3_as_one() {
        let id = VehicleId::new(3, 1);
        let cancel = cancel_compass(id);
        assert_eq!(command_of(&cancel), (CMD_DO_CANCEL_MAG_CAL, 3, 1));
        assert_eq!(params(&cancel), [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]);
    }

    /// Large Vehicle MagCal carries the typed heading in param1 and nothing else.
    #[test]
    fn large_vehicle_magcal_sends_the_heading_in_param1() {
        let id = VehicleId::new(1, 1);
        let built = fixed_mag_cal_yaw(id, 272.5);
        assert_eq!(command_of(&built), (CMD_FIXED_MAG_CAL_YAW, 1, 1));
        assert_eq!(params(&built), [272.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    }

    /// The command numbers are the dialect's.
    #[test]
    fn the_command_numbers_are_the_dialects() {
        use mp_mavlink_dialects::all::MavCmd;
        assert_eq!(
            MavCmd(u32::from(CMD_FIXED_MAG_CAL_YAW)).name(),
            Some("MAV_CMD_FIXED_MAG_CAL_YAW")
        );
        assert_eq!(
            MavCmd(u32::from(CMD_DO_START_MAG_CAL)).name(),
            Some("MAV_CMD_DO_START_MAG_CAL")
        );
        assert_eq!(
            MavCmd(u32::from(CMD_DO_ACCEPT_MAG_CAL)).name(),
            Some("MAV_CMD_DO_ACCEPT_MAG_CAL")
        );
        assert_eq!(
            MavCmd(u32::from(CMD_DO_CANCEL_MAG_CAL)).name(),
            Some("MAV_CMD_DO_CANCEL_MAG_CAL")
        );
    }

    /// Each compass keeps its first-heard place and its last message: SITL reports compass 1,
    /// then 2, then 0, and the text box lists them in that order.
    #[test]
    fn progress_keeps_the_first_heard_order_and_the_last_message() {
        let mut log = MagCalLog::default();
        assert!(log.is_empty());
        log.observe_progress(&progress(1, 2, 0));
        log.observe_progress(&progress(2, 2, 0));
        log.observe_progress(&progress(0, 2, 0));
        log.observe_progress(&progress(1, 3, 40));
        let ids: Vec<u8> = log.progress.iter().map(|entry| entry.compass_id).collect();
        assert_eq!(ids, vec![1, 2, 0]);
        assert_eq!(log.progress[0].percent, 40);
        assert_eq!(log.progress[0].status, 3);
        assert!(log.reports.is_empty());
    }

    /// A report for compass 0 with a zero X offset is passed over, and does not replace one heard
    /// before it; any other compass's is kept whatever its offsets.
    #[test]
    fn a_compass_zero_report_with_no_x_offset_is_passed_over() {
        let mut log = MagCalLog::default();
        log.observe_report(&report(0, 0.0, MAG_CAL_SUCCESS, 1));
        assert!(log.reports.is_empty());

        log.observe_report(&report(0, 12.5, MAG_CAL_SUCCESS, 1));
        log.observe_report(&report(0, 0.0, 5, 0));
        assert_eq!(log.reports.len(), 1);
        assert!((log.reports[0].offsets[0] - 12.5).abs() < f32::EPSILON);
        assert!(log.reports[0].autosaved);

        log.observe_report(&report(1, 0.0, 5, 0));
        assert_eq!(log.reports.len(), 2);
        assert_eq!(log.reports[1].status, 5);
        assert!(!log.reports[1].autosaved);
    }

    /// The status is written as the C#'s enum writes it: its name, or its number past the eight it
    /// names.
    #[test]
    fn statuses_are_named_as_the_csharps_enum_names_them() {
        assert_eq!(mag_cal_status_name(4), "MAG_CAL_SUCCESS");
        assert_eq!(mag_cal_status_name(6), "MAG_CAL_BAD_ORIENTATION");
        assert_eq!(mag_cal_status_name(7), "MAG_CAL_BAD_RADIUS");
        assert_eq!(mag_cal_status_name(8), "8");
        assert_eq!(mag_cal_status_name(10), "10");
    }
}
