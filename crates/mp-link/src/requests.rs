//! Send one message, wait for its answer, send again: Mission Planner's request loops.
//!
//! `setParamAsync`, `GetParamAsync`, `doCommandAsync` and `setWPCurrentAsync` in
//! `ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs` are four copies of one loop: send, wait a fixed
//! time for a message that matches, send again while retries remain, then throw
//! `TimeoutException`. They differ only in what they send, what answer they wait for, and their
//! numbers. So here they are one machine, [`Request`], with the differences in [`RequestKind`] and
//! the numbers in [`ProtocolTimeouts`].
//!
//! The C# blocks the calling thread inside each loop - `AwaitSync()` on the UI thread, for most
//! callers. A [`Request`] instead lives on the link thread, which feeds it every message and every
//! tick, and the caller reads its [`RequestState`] when it wants to know.

use std::time::Instant;

use mp_mavlink_dialects::all::{MavMessage, MissionSetCurrent, ParamSet};
use mp_params::{ParamTable, ParamType, ParamValue};
use mp_vehicle::VehicleId;

use crate::commands;
use crate::timeouts::{ProtocolTimeouts, Retry};

/// `MAV_CMD_PREFLIGHT_CALIBRATION`.
pub const CMD_PREFLIGHT_CALIBRATION: u16 = 241;
/// `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`.
pub const CMD_PREFLIGHT_REBOOT_SHUTDOWN: u16 = 246;
/// `MAV_CMD_GET_HOME_POSITION`.
pub const CMD_GET_HOME_POSITION: u16 = 410;
/// `MAV_CMD_FLASH_BOOTLOADER`.
pub const CMD_FLASH_BOOTLOADER: u16 = 42_650;

/// `MAV_RESULT_ACCEPTED`.
pub const MAV_RESULT_ACCEPTED: u8 = 0;
/// `MAV_RESULT_TEMPORARILY_REJECTED`.
pub const MAV_RESULT_TEMPORARILY_REJECTED: u8 = 1;
/// `MAV_RESULT_IN_PROGRESS`.
pub const MAV_RESULT_IN_PROGRESS: u8 = 5;

/// A parameter, by name or by its place in the list.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParamKey {
    /// By name, which is how nearly everything asks.
    Name(String),
    /// By `param_index`.
    Index(u16),
}

/// What is being asked, and so what answer ends the wait.
#[derive(Debug, Clone, PartialEq)]
pub enum RequestKind {
    /// `setParamAsync`: `PARAM_SET` until the vehicle echoes a `PARAM_VALUE` of that name.
    SetParam {
        /// The parameter.
        name: String,
        /// The value asked for.
        value: f64,
        /// Send even when the table already holds this value (the C#'s `force`).
        force: bool,
    },
    /// `GetParamAsync`: `PARAM_REQUEST_READ` until the parameter arrives.
    ReadParam(ParamKey),
    /// `doCommandAsync`: `COMMAND_LONG` until a `COMMAND_ACK` for this command.
    Command {
        /// `MAV_CMD`.
        command: u16,
        /// param1 to param7.
        params: [f32; 7],
        /// Whether to wait for the ack at all (the C#'s `requireack`).
        require_ack: bool,
    },
    /// `setWPCurrentAsync`: `MISSION_SET_CURRENT` until a `MISSION_CURRENT` arrives.
    SetCurrent {
        /// The mission item to make current.
        seq: u16,
    },
}

/// How a request ended.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RequestOutcome {
    /// The vehicle said yes. For a parameter, `value` is what it reported back - which is what it
    /// now holds, and not necessarily what was asked for (see [`RequestKind::SetParam`]).
    Accepted {
        /// The parameter's value as the vehicle reported it, for a parameter request.
        value: Option<ParamValue>,
    },
    /// The vehicle said no: a command's `MAV_RESULT`, anything but accepted and in progress.
    ///
    /// The C# returns false on the first such ack and does not send again - including for
    /// `MAV_RESULT_TEMPORARILY_REJECTED`, whose name invites a retry the C# does not make
    /// (C#: MAVLinkInterface.cs:2829-2833).
    Rejected(u8),
    /// Every retry went unanswered: the C#'s `TimeoutException`.
    TimedOut,
    /// Sent, and by the C#'s rule not waited for: `requireack` false, or a command whose answer
    /// would come too late or not at all - a reboot, `GET_HOME_POSITION`, the calibrations that
    /// block (C#: MAVLinkInterface.cs:2720-2724, 2734-2747, 2758-2763, 2769-2773). The C#
    /// returns true for these.
    Sent,
    /// Not sent: the vehicle has not listed a parameter of that name. The C# logs "Trying to set
    /// Param that doesnt exist" and returns false (C#: MAVLinkInterface.cs:1640-1644).
    UnknownParameter,
    /// Not sent: the parameter already holds that value. The C# logs "not modified as same" and
    /// returns true (C#: MAVLinkInterface.cs:1647-1651).
    Unchanged,
}

/// Where a request is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RequestState {
    /// Queued by the caller, not yet picked up by the link thread.
    Queued,
    /// Sent, waiting for the answer.
    Waiting,
    /// Over.
    Finished(RequestOutcome),
}

/// What a request wants put on the wire.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Outgoing {
    /// Nothing.
    Nothing,
    /// This message, once.
    Once(MavMessage),
    /// This message, twice back to back: how the C# sends a reboot and a compassmot, "just
    /// incase" (C#: MAVLinkInterface.cs:2743-2744, 2760-2761).
    Twice(MavMessage),
}

/// One request and its retries.
#[derive(Debug, Clone)]
pub struct Request {
    /// Which vehicle.
    pub target: VehicleId,
    /// What is being asked.
    pub kind: RequestKind,
    state: RequestState,
    /// What goes on the wire, built once so every retry sends the same bytes - bar a command's
    /// confirmation, which the C# counts up per retry (C#: MAVLinkInterface.cs:2789).
    message: Option<MavMessage>,
    policy: Retry,
    retries_left: u8,
    deadline: Instant,
    sends: u16,
}

impl Request {
    /// A request, not yet sent. The link thread sends it when it picks it up.
    #[must_use]
    pub fn new(target: VehicleId, kind: RequestKind) -> Self {
        Self {
            target,
            kind,
            state: RequestState::Queued,
            message: None,
            policy: Retry {
                timeout: std::time::Duration::ZERO,
                retries: 0,
            },
            retries_left: 0,
            deadline: Instant::now(),
            sends: 0,
        }
    }

    /// Where it is.
    #[must_use]
    pub const fn state(&self) -> RequestState {
        self.state
    }

    /// How it ended, once it has.
    #[must_use]
    pub const fn outcome(&self) -> Option<RequestOutcome> {
        match self.state {
            RequestState::Finished(outcome) => Some(outcome),
            RequestState::Queued | RequestState::Waiting => None,
        }
    }

    /// Whether it has ended.
    #[must_use]
    pub const fn is_finished(&self) -> bool {
        matches!(self.state, RequestState::Finished(_))
    }

    /// How many times it has gone on the wire, counting a double send as two.
    #[must_use]
    pub const fn sends(&self) -> u16 {
        self.sends
    }

    fn finish(&mut self, outcome: RequestOutcome) {
        self.state = RequestState::Finished(outcome);
    }

    fn arm(&mut self, message: MavMessage, policy: Retry, now: Instant) {
        self.message = Some(message);
        self.policy = policy;
        self.retries_left = policy.retries;
        self.deadline = now + policy.timeout;
        self.state = RequestState::Waiting;
    }

    fn count(&mut self, send: Outgoing) -> Outgoing {
        self.sends += match send {
            Outgoing::Nothing => 0,
            Outgoing::Once(_) => 1,
            Outgoing::Twice(_) => 2,
        };
        send
    }

    /// Picks the request up: the checks the C# makes before sending, and the first send.
    ///
    /// `table` is the vehicle's parameters, for a parameter set; `ardupilot` whether the vehicle
    /// is ArduPilot, which decides how a parameter value is put in the float field.
    pub fn begin(
        &mut self,
        timeouts: &ProtocolTimeouts,
        table: Option<&ParamTable>,
        ardupilot: bool,
        now: Instant,
    ) -> Outgoing {
        let target = self.target;
        let send = match &self.kind {
            RequestKind::SetParam { name, value, force } => {
                let Some(current) = table.and_then(|table| table.get(name)) else {
                    self.finish(RequestOutcome::UnknownParameter);
                    return Outgoing::Nothing;
                };
                // An exact comparison, because the C#'s is (`param.Value == value`, :1647), and
                // both sides are the table's rounded reading of the same bytes.
                #[allow(clippy::float_cmp)]
                let unchanged = current.as_f64() == *value;
                if unchanged && !force {
                    self.finish(RequestOutcome::Unchanged);
                    return Outgoing::Nothing;
                }
                let message = param_set(target, name, *value, current.param_type(), ardupilot);
                self.arm(message, timeouts.param_set, now);
                Outgoing::Once(message)
            }
            RequestKind::ReadParam(key) => {
                let message = match key {
                    ParamKey::Name(name) => commands::request_param_by_name(target, name),
                    ParamKey::Index(index) => commands::request_param_by_index(target, *index),
                };
                self.arm(message, timeouts.param_read, now);
                Outgoing::Once(message)
            }
            RequestKind::Command {
                command,
                params,
                require_ack,
            } => {
                let message = commands::command(target, *command, *params);
                let [.., p5, p6, _] = *params;
                let calibration = *command == CMD_PREFLIGHT_CALIBRATION;
                // doCommandAsync's special cases, in its order (:2720-2773).
                if !require_ack || (calibration && is_one(p5)) || *command == CMD_GET_HOME_POSITION
                {
                    self.finish(RequestOutcome::Sent);
                    Outgoing::Once(message)
                } else if (calibration && is_one(p6)) || *command == CMD_PREFLIGHT_REBOOT_SHUTDOWN {
                    self.finish(RequestOutcome::Sent);
                    Outgoing::Twice(message)
                } else {
                    let policy = if calibration || *command == CMD_FLASH_BOOTLOADER {
                        timeouts.command_slow
                    } else if *command == commands::CMD_COMPONENT_ARM_DISARM {
                        timeouts.command_arm
                    } else {
                        timeouts.command
                    };
                    self.arm(message, policy, now);
                    Outgoing::Once(message)
                }
            }
            RequestKind::SetCurrent { seq } => {
                let message = MavMessage::MissionSetCurrent(MissionSetCurrent {
                    seq: *seq,
                    target_system: target.sysid,
                    target_component: target.compid,
                });
                self.arm(message, timeouts.set_current, now);
                Outgoing::Once(message)
            }
        };
        self.count(send)
    }

    /// A `PARAM_VALUE` arrived from `from`.
    ///
    /// For a set, only the name has to match: whatever value comes back is the answer, and the
    /// set succeeded. The vehicle may have clamped or refused the value, and the C# still returns
    /// true and stores what came back (C#: MAVLinkInterface.cs:1693-1731); so does this, carrying
    /// the reported value in [`RequestOutcome::Accepted`] for a caller that wants to compare.
    pub fn on_param_value(&mut self, from: VehicleId, name: &str, index: u16, value: ParamValue) {
        if self.state != RequestState::Waiting || from != self.target {
            return;
        }
        let matches = match &self.kind {
            // "MAVLINK bad param response" otherwise, and keep waiting (:1699-1703).
            RequestKind::SetParam { name: wanted, .. } => wanted == name,
            // "Wrong Answer" otherwise, and keep waiting (:2365-2371).
            RequestKind::ReadParam(ParamKey::Name(wanted)) => wanted == name,
            RequestKind::ReadParam(ParamKey::Index(wanted)) => *wanted == index,
            RequestKind::Command { .. } | RequestKind::SetCurrent { .. } => false,
        };
        if matches {
            self.finish(RequestOutcome::Accepted { value: Some(value) });
        }
    }

    /// A `COMMAND_ACK` arrived from `from`. C#: MAVLinkInterface.cs:2800-2834.
    ///
    /// Returns whether this request took it, so one ack answers one command.
    pub fn on_command_ack(
        &mut self,
        from: VehicleId,
        command: u16,
        result: u8,
        now: Instant,
    ) -> bool {
        if self.state != RequestState::Waiting || from != self.target {
            return false;
        }
        let RequestKind::Command {
            command: wanted, ..
        } = self.kind
        else {
            return false;
        };
        if wanted != command {
            // "Commands dont match", and keep waiting (:2808-2813).
            return false;
        }
        match result {
            // Wait a whole timeout again, with no retries left: the vehicle has the command, and
            // sending it again would start it again (:2818-2823).
            MAV_RESULT_IN_PROGRESS => {
                self.deadline = now + self.policy.timeout;
                self.retries_left = 0;
            }
            MAV_RESULT_ACCEPTED => self.finish(RequestOutcome::Accepted { value: None }),
            other => self.finish(RequestOutcome::Rejected(other)),
        }
        true
    }

    /// A `MISSION_CURRENT` arrived from `from`: any one ends a set-current (:2482-2487).
    pub fn on_mission_current(&mut self, from: VehicleId) {
        if self.state == RequestState::Waiting
            && from == self.target
            && matches!(self.kind, RequestKind::SetCurrent { .. })
        {
            self.finish(RequestOutcome::Accepted { value: None });
        }
    }

    /// Called every pass of the link loop: sends again, or gives up.
    pub fn on_tick(&mut self, now: Instant) -> Outgoing {
        if self.state != RequestState::Waiting || now < self.deadline {
            return Outgoing::Nothing;
        }
        let Some(mut message) = self.message else {
            self.finish(RequestOutcome::TimedOut);
            return Outgoing::Nothing;
        };
        if self.retries_left == 0 {
            self.finish(RequestOutcome::TimedOut);
            return Outgoing::Nothing;
        }
        self.retries_left -= 1;
        self.deadline = now + self.policy.timeout;
        if let MavMessage::CommandLong(long) = &mut message {
            long.confirmation = long.confirmation.wrapping_add(1);
            self.message = Some(message);
        }
        self.count(Outgoing::Once(message))
    }
}

/// `p5 == 1` in the C#, on a float that arrived as a float.
#[allow(clippy::float_cmp)]
fn is_one(value: f32) -> bool {
    value == 1.0
}

/// A `PARAM_SET` built as `setParamAsync` builds it (C#: MAVLinkInterface.cs:1657-1678).
///
/// The type field carries the parameter's declared type whatever the autopilot. The value field
/// carries the number as a float for ArduPilot - which always sends floats and converts on its
/// side - and the declared type's bytes for anything that follows the specification.
fn param_set(
    target: VehicleId,
    name: &str,
    value: f64,
    declared: ParamType,
    ardupilot: bool,
) -> MavMessage {
    let field = if ardupilot {
        ParamValue::from_f64(value, ParamType::Real32)
    } else {
        ParamValue::from_f64(value, declared)
    };
    MavMessage::ParamSet(ParamSet {
        param_value: field.to_param_value_field(),
        target_system: target.sysid,
        target_component: target.compid,
        param_id: mp_params::encode_param_id(name),
        param_type: declared.to_wire(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn target() -> VehicleId {
        VehicleId::new(1, 1)
    }

    fn table() -> ParamTable {
        let mut table = ParamTable::new();
        table.insert(
            "RTL_ALT".to_owned(),
            ParamValue::from_ardupilot(1500.0, ParamType::Int32),
            0,
            1,
        );
        table
    }

    #[test]
    fn a_command_is_sent_four_times_then_times_out_with_its_confirmation_counting_up() {
        let t = ProtocolTimeouts::default();
        let t0 = Instant::now();
        let mut request = Request::new(
            target(),
            RequestKind::Command {
                command: 22,
                params: [0.0; 7],
                require_ack: true,
            },
        );
        let mut confirmations = Vec::new();
        let mut record = |send: Outgoing| {
            if let Outgoing::Once(MavMessage::CommandLong(long)) = send {
                confirmations.push(long.confirmation);
            }
        };
        record(request.begin(&t, None, true, t0));
        for step in 1..=4u32 {
            record(request.on_tick(t0 + t.command.timeout * step));
        }
        assert_eq!(confirmations, vec![0, 1, 2, 3]);
        assert_eq!(request.outcome(), Some(RequestOutcome::TimedOut));
        assert_eq!(request.sends(), t.command.sends());
    }

    #[test]
    fn a_set_param_that_would_change_nothing_is_not_sent() {
        let t = ProtocolTimeouts::default();
        let mut request = Request::new(
            target(),
            RequestKind::SetParam {
                name: "RTL_ALT".to_owned(),
                value: 1500.0,
                force: false,
            },
        );
        assert_eq!(
            request.begin(&t, Some(&table()), true, Instant::now()),
            Outgoing::Nothing
        );
        assert_eq!(request.outcome(), Some(RequestOutcome::Unchanged));
    }

    #[test]
    fn a_set_param_of_a_name_the_vehicle_never_listed_is_refused() {
        let t = ProtocolTimeouts::default();
        let mut request = Request::new(
            target(),
            RequestKind::SetParam {
                name: "NO_SUCH".to_owned(),
                value: 1.0,
                force: false,
            },
        );
        assert_eq!(
            request.begin(&t, Some(&table()), true, Instant::now()),
            Outgoing::Nothing
        );
        assert_eq!(request.outcome(), Some(RequestOutcome::UnknownParameter));
    }

    #[test]
    fn a_set_param_carries_the_declared_type_and_the_value_as_a_float_for_ardupilot() {
        let t = ProtocolTimeouts::default();
        let mut request = Request::new(
            target(),
            RequestKind::SetParam {
                name: "RTL_ALT".to_owned(),
                value: 2000.0,
                force: false,
            },
        );
        let Outgoing::Once(MavMessage::ParamSet(set)) =
            request.begin(&t, Some(&table()), true, Instant::now())
        else {
            panic!("a PARAM_SET");
        };
        assert_eq!(set.param_type, ParamType::Int32.to_wire());
        assert!((set.param_value - 2000.0).abs() < f32::EPSILON);
    }

    #[test]
    fn a_read_by_index_is_answered_by_that_index_whatever_its_name() {
        // `par.param_index == index || st == name` (C#: MAVLinkInterface.cs:2365), with the name
        // empty when reading by index: only the index can match.
        let t = ProtocolTimeouts::default();
        let mut request = Request::new(target(), RequestKind::ReadParam(ParamKey::Index(7)));
        let Outgoing::Once(MavMessage::ParamRequestRead(read)) =
            request.begin(&t, None, true, Instant::now())
        else {
            panic!("a PARAM_REQUEST_READ");
        };
        assert_eq!(read.param_index, 7);
        assert_eq!(read.param_id, [0u8; 16]);

        let value = ParamValue::from_ardupilot(3.0, ParamType::Real32);
        request.on_param_value(target(), "OTHER", 8, value);
        assert_eq!(request.outcome(), None, "index 8 is not the answer");
        request.on_param_value(target(), "WPNAV_SPEED", 7, value);
        assert_eq!(
            request.outcome(),
            Some(RequestOutcome::Accepted { value: Some(value) })
        );
    }

    #[test]
    fn in_progress_waits_one_more_timeout_and_never_resends() {
        let t = ProtocolTimeouts::default();
        let t0 = Instant::now();
        let mut request = Request::new(
            target(),
            RequestKind::Command {
                command: 22,
                params: [0.0; 7],
                require_ack: true,
            },
        );
        let _ = request.begin(&t, None, true, t0);
        let later = t0 + Duration::from_millis(1500);
        assert!(request.on_command_ack(target(), 22, MAV_RESULT_IN_PROGRESS, later));
        assert_eq!(
            request.on_tick(later + t.command.timeout),
            Outgoing::Nothing
        );
        assert_eq!(request.outcome(), Some(RequestOutcome::TimedOut));
        assert_eq!(request.sends(), 1);
    }
}
