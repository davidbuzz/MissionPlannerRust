//! Sensor calibration.
//!
//! Accelerometer calibration is a conversation, not a command. The ground station asks the vehicle
//! to start; the vehicle then asks, one at a time, for the airframe to be put in six orientations,
//! and waits for the ground station to say it has been done. The exchange runs over
//! `MAV_CMD_ACCELCAL_VEHICLE_POS` in both directions - the vehicle sends it to ask, the ground
//! station sends it back to confirm - which is unusual enough to be worth saying out loud.
//!
//! Levelling and compass calibration are single commands by comparison, and are here because they
//! are the same `MAV_CMD_PREFLIGHT_CALIBRATION` with a different parameter set.

use mp_mavlink_dialects::all::MavMessage;
use mp_vehicle::VehicleId;

use crate::commands::command;

/// `MAV_CMD_PREFLIGHT_CALIBRATION`.
pub const CMD_PREFLIGHT_CALIBRATION: u16 = 241;

/// `MAV_CMD_ACCELCAL_VEHICLE_POS`, an ArduPilot extension.
pub const CMD_ACCELCAL_VEHICLE_POS: u16 = 42_429;

/// Where the vehicle wants the airframe put next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccelPosition {
    /// Sitting normally.
    Level,
    /// Rolled onto its left side.
    Left,
    /// Rolled onto its right side.
    Right,
    /// Nose pointing at the ground.
    NoseDown,
    /// Nose pointing at the sky.
    NoseUp,
    /// Upside down.
    Back,
}

impl AccelPosition {
    /// Reads a position from the wire, or `None` if it is not one.
    ///
    /// The success and failure values share this field and are handled by the caller, because they
    /// are outcomes rather than positions and treating them as positions would ask the operator to
    /// place the vehicle "success".
    #[must_use]
    pub const fn from_wire(value: u32) -> Option<Self> {
        Some(match value {
            1 => Self::Level,
            2 => Self::Left,
            3 => Self::Right,
            4 => Self::NoseDown,
            5 => Self::NoseUp,
            6 => Self::Back,
            _ => return None,
        })
    }

    /// The wire value.
    #[must_use]
    pub const fn to_wire(self) -> u32 {
        match self {
            Self::Level => 1,
            Self::Left => 2,
            Self::Right => 3,
            Self::NoseDown => 4,
            Self::NoseUp => 5,
            Self::Back => 6,
        }
    }

    /// What to tell the operator to do.
    ///
    /// Written as an instruction rather than a name. "LEFT" is a label the operator has to
    /// interpret; "place the vehicle on its left side" is what to do, and getting an orientation
    /// wrong means the calibration is wrong in a way that only shows up in flight.
    #[must_use]
    pub const fn instruction(self) -> &'static str {
        match self {
            Self::Level => "place the vehicle level",
            Self::Left => "place the vehicle on its LEFT side",
            Self::Right => "place the vehicle on its RIGHT side",
            Self::NoseDown => "place the vehicle NOSE DOWN",
            Self::NoseUp => "place the vehicle NOSE UP",
            Self::Back => "place the vehicle on its BACK",
        }
    }

    /// A short name, for a progress list.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Level => "level",
            Self::Left => "left",
            Self::Right => "right",
            Self::NoseDown => "nose down",
            Self::NoseUp => "nose up",
            Self::Back => "back",
        }
    }

    /// The six, in the order ArduPilot asks for them.
    pub const ALL: [Self; 6] = [
        Self::Level,
        Self::Left,
        Self::Right,
        Self::NoseDown,
        Self::NoseUp,
        Self::Back,
    ];
}

/// `ACCELCAL_VEHICLE_POS_SUCCESS`.
pub const ACCELCAL_SUCCESS: u32 = 16_777_215;
/// `ACCELCAL_VEHICLE_POS_FAILED`.
pub const ACCELCAL_FAILED: u32 = 16_777_216;

/// What the vehicle last said about an accelerometer calibration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AccelCalibration {
    /// Not started, or finished and cleared.
    #[default]
    Idle,
    /// Waiting for the airframe to be put in this position, and for the operator to confirm it.
    Waiting(AccelPosition),
    /// The vehicle accepted the calibration.
    Succeeded,
    /// The vehicle rejected it. Usually because the airframe moved during a sample.
    Failed,
}

impl AccelCalibration {
    /// Interprets the value the vehicle sent in `MAV_CMD_ACCELCAL_VEHICLE_POS`.
    #[must_use]
    pub const fn from_wire(value: u32) -> Self {
        if value == ACCELCAL_SUCCESS {
            return Self::Succeeded;
        }
        if value == ACCELCAL_FAILED {
            return Self::Failed;
        }
        match AccelPosition::from_wire(value) {
            Some(position) => Self::Waiting(position),
            // A value that is neither a position nor an outcome. Staying idle is better than
            // inventing a step: the operator would be asked to place the vehicle in a position
            // that does not exist.
            None => Self::Idle,
        }
    }

    /// Whether the calibration is still running.
    #[must_use]
    pub const fn in_progress(self) -> bool {
        matches!(self, Self::Waiting(_))
    }
}

/// Starts a six-position accelerometer calibration.
///
/// param5 = 1. The other calibrations share this command and must be zero, which the definitions
/// say explicitly: "only one sensor should be set in a single message".
#[must_use]
pub fn start_accelerometer(target: VehicleId) -> MavMessage {
    command(
        target,
        CMD_PREFLIGHT_CALIBRATION,
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
    )
}

/// Tells the vehicle the airframe is now in the position it asked for.
///
/// The same command the vehicle used to ask, sent back with the same value. That is how ArduPilot
/// expects the handshake to work, and it is what Mission Planner does.
#[must_use]
pub fn accelerometer_position_reached(target: VehicleId, position: AccelPosition) -> MavMessage {
    #[allow(clippy::cast_precision_loss)] // positions are 1 to 6
    let value = position.to_wire() as f32;
    command(
        target,
        CMD_ACCELCAL_VEHICLE_POS,
        [value, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
}

/// Levels the vehicle: tells it that however it is sitting now is level.
///
/// param5 = 2. A single sample rather than a sequence, and the thing to run after mounting the
/// autopilot slightly askew. It is not a substitute for the six-position calibration: it corrects
/// the attitude reference, not the accelerometer scale factors.
#[must_use]
pub fn level(target: VehicleId) -> MavMessage {
    command(
        target,
        CMD_PREFLIGHT_CALIBRATION,
        [0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0],
    )
}

/// Starts an onboard compass calibration.
///
/// param2 = 1. The vehicle samples while the airframe is rotated through every orientation and
/// reports progress in `MAG_CAL_PROGRESS`.
#[must_use]
pub fn start_compass(target: VehicleId) -> MavMessage {
    command(
        target,
        CMD_PREFLIGHT_CALIBRATION,
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
}

/// Calibrates the barometer's ground pressure reference.
///
/// param3 = 1. Quick, and worth doing before a flight when the weather has changed: the altitude
/// the vehicle reports is relative to whatever pressure it last called zero.
#[must_use]
pub fn ground_pressure(target: VehicleId) -> MavMessage {
    command(
        target,
        CMD_PREFLIGHT_CALIBRATION,
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    )
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

    /// The command id of a built command.
    fn command_id(message: &MavMessage) -> u16 {
        match message {
            MavMessage::CommandLong(c) => c.command,
            other => panic!("expected a COMMAND_LONG, got {}", other.name()),
        }
    }

    #[test]
    fn positions_round_trip_through_the_wire() {
        for position in AccelPosition::ALL {
            assert_eq!(AccelPosition::from_wire(position.to_wire()), Some(position));
        }
    }

    #[test]
    fn the_six_positions_are_the_ones_ardupilot_asks_for() {
        // 1 to 6, in order. ArduPilot asks for them by number and a mismatch would have the
        // operator place the vehicle one way and confirm another.
        let values: Vec<u32> = AccelPosition::ALL.iter().map(|p| p.to_wire()).collect();
        assert_eq!(values, vec![1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn success_and_failure_are_outcomes_not_positions() {
        // They arrive in the same field. Read as positions they would ask the operator to place
        // the vehicle in one that does not exist.
        assert_eq!(AccelPosition::from_wire(ACCELCAL_SUCCESS), None);
        assert_eq!(AccelPosition::from_wire(ACCELCAL_FAILED), None);
        assert_eq!(
            AccelCalibration::from_wire(ACCELCAL_SUCCESS),
            AccelCalibration::Succeeded
        );
        assert_eq!(
            AccelCalibration::from_wire(ACCELCAL_FAILED),
            AccelCalibration::Failed
        );
    }

    #[test]
    fn an_unknown_value_leaves_the_calibration_idle() {
        assert_eq!(AccelCalibration::from_wire(99), AccelCalibration::Idle);
        assert!(!AccelCalibration::from_wire(99).in_progress());
    }

    #[test]
    fn a_position_request_is_in_progress() {
        let state = AccelCalibration::from_wire(3);
        assert_eq!(state, AccelCalibration::Waiting(AccelPosition::Right));
        assert!(state.in_progress());
    }

    #[test]
    fn each_calibration_sets_exactly_one_parameter() {
        // The definitions are explicit: "only one sensor should be set in a single message and all
        // others should be zero". Setting two asks the vehicle for two calibrations at once, which
        // it refuses.
        let id = VehicleId::new(1, 1);
        for (what, built) in [
            ("accelerometer", start_accelerometer(id)),
            ("level", level(id)),
            ("compass", start_compass(id)),
            ("ground pressure", ground_pressure(id)),
        ] {
            let set = params(&built)
                .iter()
                .filter(|p| p.abs() > f32::EPSILON)
                .count();
            assert_eq!(set, 1, "{what} sets {set} parameters, not 1");
            assert_eq!(command_id(&built), CMD_PREFLIGHT_CALIBRATION);
        }
    }

    #[test]
    fn the_accelerometer_and_level_calibrations_are_different_requests() {
        // Both are param5, with different values - 1 for the six-position sequence and 2 for a
        // single level sample. Confusing them would run the wrong one silently.
        let id = VehicleId::new(1, 1);
        assert!((params(&start_accelerometer(id))[4] - 1.0).abs() < f32::EPSILON);
        assert!((params(&level(id))[4] - 2.0).abs() < f32::EPSILON);
    }

    #[test]
    fn confirming_a_position_sends_back_what_was_asked_for() {
        let id = VehicleId::new(1, 1);
        let built = accelerometer_position_reached(id, AccelPosition::NoseUp);
        assert_eq!(command_id(&built), CMD_ACCELCAL_VEHICLE_POS);
        assert!((params(&built)[0] - 5.0).abs() < f32::EPSILON);
    }

    #[test]
    fn every_instruction_says_what_to_do_rather_than_naming_a_state() {
        // "LEFT" is a label an operator has to interpret; getting an orientation wrong means a
        // calibration that is wrong in a way which only shows up in flight.
        for position in AccelPosition::ALL {
            let instruction = position.instruction();
            assert!(
                instruction.starts_with("place the vehicle"),
                "{instruction}"
            );
            assert!(!position.label().is_empty());
        }
    }
}
