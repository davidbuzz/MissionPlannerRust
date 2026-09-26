//! Send one message, wait for its answer, send again: Mission Planner's request loops.
//!
//! `setParamAsync`, `GetParamAsync`, `doCommandAsync`, `doCommandIntAsync`, `setWPCurrentAsync`,
//! `setWPAsync` and `getHomePositionAsync` in `ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs` are
//! copies of one loop: send, wait a fixed
//! time for a message that matches, send again while retries remain, then throw
//! `TimeoutException`. They differ only in what they send, what answer they wait for, and their
//! numbers. So here they are one machine, [`Request`], with the differences in [`RequestKind`] and
//! the numbers in [`ProtocolTimeouts`].
//!
//! The C# blocks the calling thread inside each loop - `AwaitSync()` on the UI thread, for most
//! callers. A [`Request`] instead lives on the link thread, which feeds it every message and every
//! tick, and the caller reads its [`RequestState`] when it wants to know.

use std::time::Instant;

use mp_mavlink_dialects::all::{
    FenceFetchPoint, FencePoint, MavMessage, MissionSetCurrent, ParamSet, RallyFetchPoint,
    RallyPoint,
};
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
    /// `setRallyPoint`: `RALLY_POINT`, then `getRallyPoint` - `RALLY_FETCH_POINT` until the
    /// vehicle sends that point back - and the point again if what came back is not what was
    /// sent, three times in all. See [`RallyPointSet`].
    SetRallyPoint(RallyPointSet),
    /// `doCommandIntAsync`: `COMMAND_INT` until a `COMMAND_ACK` for this command, three more
    /// times two seconds apart as `doCommandAsync` sends, with none of its special cases - and
    /// none of its `IN_PROGRESS` patience: anything but `ACCEPTED` is `false` (`:2940-2949`).
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2847-2951`
    CommandInt {
        /// `MAV_CMD`.
        command: u16,
        /// `MAV_FRAME`, which the C# defaults to `GLOBAL`.
        frame: u8,
        /// param1 to param4.
        params: [f32; 4],
        /// `x`, param5 as an integer.
        x: i32,
        /// `y`, param6 as an integer.
        y: i32,
        /// `z`, param7.
        z: f32,
        /// Whether to wait for the ack at all (the C#'s `requireack`).
        require_ack: bool,
    },
    /// `setWPAsync` for one item: a `MISSION_ITEM` or `MISSION_ITEM_INT` until the vehicle
    /// acknowledges it or asks for the item after it, ten more times 450 ms apart - Change Alt,
    /// and ArduPlane's guided target.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4061-4235, 4236-4380`
    SetWp {
        /// The item as built, sent as it is. Boxed: a message is the largest thing a kind holds.
        item: Box<MavMessage>,
        /// Its sequence number: the vehicle asking for `seq + 1` is an acceptance.
        seq: u16,
    },
    /// `getHomePositionAsync`: `GET_HOME_POSITION` - `doCommand` with `requireack` false each
    /// time - until a `HOME_POSITION` arrives, three more times 700 ms apart.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3343-3387`
    GetHomePosition,
    /// `setFencePoint`: `FENCE_POINT`, then `getFencePoint` - `FENCE_FETCH_POINT` until the
    /// vehicle sends that point back - and the point again if what came back is five metres or
    /// more from what was sent, three times in all. See [`FencePointSet`].
    SetFencePoint(FencePointSet),
    /// `getFencePoint`: `FENCE_FETCH_POINT` for point `idx` until the vehicle sends it, three
    /// more times 700 ms apart; what came is [`Request::fence_point`].
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5908-5967`
    GetFencePoint {
        /// The point's place, from 0: the return point, then the polygon's corners.
        idx: u8,
    },
}

/// One geofence point as `setFencePoint` puts it in a `mavlink_fence_point_t`: the position as
/// `(float)` degrees, the count as the C#'s `byte`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6415-6439`
///
/// What the machine does (`setFencePoint` `:6415-6439`, `getFencePoint` `:5908-5967`):
///
/// * `FENCE_POINT` goes out (`:6430`), then `FENCE_FETCH_POINT` for its index (`:5923`), waiting
///   700 ms for a `FENCE_POINT` from the vehicle, of that index, addressed to this ground station
///   and to `MAV_COMP_ID_MISSIONPLANNER` (`:5948-5957`), sending the fetch three more times
///   (`:5926-5942`). A `FENCE_POINT` that is not that one is read past, not answered
///   (`continue`, `:5954-5957`).
/// * Every fetch unanswered ends [`RequestOutcome::TimedOut`]: `getFencePoint`'s
///   `TimeoutException`, which `setFencePoint` does not catch, so no further round is made.
/// * A point that came back within five metres of the one sent (`GetDistance`, `:6433`) ends
///   [`RequestOutcome::Accepted`]; one further off sends the point and the fetch again, three
///   times in all (`retry = 3`, `:6426-6436`), and then ends [`RequestOutcome::Sent`] - sent, not
///   confirmed: the C#'s `throw new Exception("Could not verify GeoFence Point")` (`:6438`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FencePointSet {
    /// `idx`: the point's place, from 0.
    pub idx: u8,
    /// `count`: how many points there are, return and closing point included.
    pub count: u8,
    /// Latitude, degrees, as the caller has it; the wire carries it as a `float`.
    pub lat: f64,
    /// Longitude, degrees, likewise.
    pub lng: f64,
}

/// A `FENCE_POINT` that answered a `getFencePoint`: its position, and `count`, the C#'s `total`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5959-5965`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FencePointRead {
    /// `fp.lat`.
    pub lat: f32,
    /// `fp.lng`.
    pub lng: f32,
    /// `fp.count`: how many points the vehicle holds.
    pub count: u8,
}

/// How many times `setFencePoint` sends a point before it gives up on reading it back the same:
/// `int retry = 3; while (retry > 0)`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6426-6436`
pub const FENCE_POINT_SENDS: u8 = 3;

/// How close, in metres, a point read back must be to the one sent: `GetDistance(plla) < 5`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6433`
pub const FENCE_POINT_TOLERANCE: f64 = 5.0;

/// The `FENCE_POINT` that sets `set` on `target`: `(float) plla.Lat`, `(float) plla.Lng`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6417-6424`
#[allow(clippy::cast_possible_truncation)] // the C#'s `(float)`
fn fence_point(target: VehicleId, set: &FencePointSet) -> MavMessage {
    MavMessage::FencePoint(FencePoint {
        lat: set.lat as f32,
        lng: set.lng as f32,
        target_system: target.sysid,
        target_component: target.compid,
        idx: set.idx,
        count: set.count,
    })
}

/// The `FENCE_FETCH_POINT` that reads point `idx` of `target` back.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5916-5923`
const fn fence_fetch(target: VehicleId, idx: u8) -> MavMessage {
    MavMessage::FenceFetchPoint(FenceFetchPoint {
        target_system: target.sysid,
        target_component: target.compid,
        idx,
    })
}

/// Whether a point read back is the one sent: `newfp.plla.GetDistance(plla) < 5`. A position
/// that is not one - out of range, not a number - is not.
fn fence_point_matches(read: &FencePoint, set: &FencePointSet) -> bool {
    let (Ok(back), Ok(sent)) = (
        mp_units::LatLon::new(f64::from(read.lat), f64::from(read.lng)),
        mp_units::LatLon::new(set.lat, set.lng),
    ) else {
        return false;
    };
    back.distance_to(sent).0 < FENCE_POINT_TOLERANCE
}

/// One rally point as `setRallyPoint` puts it in a `mavlink_rally_point_t`: the position as
/// `(int)(degrees * 1e7)`, the altitude as `(short)`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6445-6456`
///
/// What the machine does, and where it departs from the C# (`setRallyPoint` `:6441-6476`,
/// `getRallyPoint` `:6346-6412`):
///
/// * `RALLY_POINT` goes out (`:6462`), then `RALLY_FETCH_POINT` for its index (`:6360`), waiting
///   700 ms for a `RALLY_POINT` of that index addressed to this ground station, three more times
///   (`:6363-6378`).
/// * **Divergence:** the C#'s retries send `FENCE_FETCH_POINT` - the rally request's bytes under
///   the fence message's id (`:6372`, `:6397`) - which no vehicle answers with a rally point, so
///   its retries can never succeed. Here they send `RALLY_FETCH_POINT`, as the first ask does.
/// * A `RALLY_POINT` of another index asks again at once without spending a retry (`:6395-6399`).
/// * **Divergence:** the C# takes the point as set (`:6466`) when `newfp.plla.Lat == plla.Lat &&
///   newfp.plla.Lng == rp.lng` - the second half compares degrees with `degrees * 1e7`, and the
///   first a double with what came back through 1e7 - so it is all but never true, and every
///   point is sent three times and `false` returned, which its caller ignores. Here the point is
///   set when the latitude and longitude that come back are the ones sent; otherwise it goes
///   again, three times in all, as the C#'s loop does, and ends [`RequestOutcome::Sent`] - sent,
///   not confirmed - as the C#'s `false`.
/// * Every retry unanswered ends [`RequestOutcome::TimedOut`]: the C#'s `TimeoutException`,
///   which `saveRallyPointsToolStripMenuItem_Click` turns into "Failed to save rally point".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RallyPointSet {
    /// `idx`: the point's place, from 0.
    pub idx: u8,
    /// `count`: how many there are.
    pub count: u8,
    /// Latitude, degrees times 1e7.
    pub lat: i32,
    /// Longitude, degrees times 1e7.
    pub lng: i32,
    /// Altitude, metres.
    pub alt: i16,
    /// `break_alt`, metres.
    pub break_alt: i16,
    /// `land_dir`, centidegrees.
    pub land_dir: u16,
    /// `flags`.
    pub flags: u8,
}

/// How many times `setRallyPoint` sends a point before it gives up on reading it back:
/// `int retry = 3; while (retry > 0)`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6458-6473`
pub const RALLY_POINT_SENDS: u8 = 3;

/// The `RALLY_POINT` that sets `set` on `target`.
fn rally_point(target: VehicleId, set: &RallyPointSet) -> MavMessage {
    MavMessage::RallyPoint(RallyPoint {
        lat: set.lat,
        lng: set.lng,
        alt: set.alt,
        break_alt: set.break_alt,
        land_dir: set.land_dir,
        target_system: target.sysid,
        target_component: target.compid,
        idx: set.idx,
        count: set.count,
        flags: set.flags,
    })
}

/// The `RALLY_FETCH_POINT` that reads point `idx` of `target` back.
const fn rally_fetch(target: VehicleId, idx: u8) -> MavMessage {
    MavMessage::RallyFetchPoint(RallyFetchPoint {
        target_system: target.sysid,
        target_component: target.compid,
        idx,
    })
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
///
/// A rally point's pair makes this two messages wide. It lives for one pass of the link loop, in a
/// list reused every pass, so the size is paid once and a box on every pair would not be.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum Outgoing {
    /// Nothing.
    Nothing,
    /// This message, once.
    Once(MavMessage),
    /// This message, twice back to back: how the C# sends a reboot and a compassmot, "just
    /// incase" (C#: MAVLinkInterface.cs:2743-2744, 2760-2761).
    Twice(MavMessage),
    /// These two, in this order: a rally point and the fetch that reads it back
    /// (C#: MAVLinkInterface.cs:6462-6464, 6360).
    Pair(MavMessage, MavMessage),
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
    /// A rally point's sends left after this one: `setRallyPoint`'s outer loop; a fence
    /// point's likewise, `setFencePoint`'s.
    attempts_left: u8,
    /// The `FENCE_POINT` that answered a fence point's fetch.
    fence_read: Option<FencePointRead>,
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
            attempts_left: 0,
            fence_read: None,
        }
    }

    /// The `FENCE_POINT` that answered a [`RequestKind::GetFencePoint`] or a
    /// [`RequestKind::SetFencePoint`]'s read-back: the last one read, whether or not it matched.
    #[must_use]
    pub const fn fence_point(&self) -> Option<FencePointRead> {
        self.fence_read
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
            Outgoing::Twice(_) | Outgoing::Pair(..) => 2,
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
            RequestKind::SetRallyPoint(set) => {
                let fetch = rally_fetch(target, set.idx);
                let point = rally_point(target, set);
                self.arm(fetch, timeouts.rally_fetch, now);
                self.attempts_left = RALLY_POINT_SENDS - 1;
                Outgoing::Pair(point, fetch)
            }
            RequestKind::CommandInt {
                command,
                frame,
                params,
                x,
                y,
                z,
                require_ack,
            } => {
                let message = commands::command_int(target, *command, *frame, *params, *x, *y, *z);
                if *require_ack {
                    // `retrys = 3; timeout = 2000` (:2884-2885), the same as doCommandAsync's.
                    self.arm(message, timeouts.command, now);
                } else {
                    self.finish(RequestOutcome::Sent);
                }
                Outgoing::Once(message)
            }
            RequestKind::SetWp { item, .. } => {
                let item = **item;
                self.arm(item, timeouts.mission_item_send, now);
                Outgoing::Once(item)
            }
            RequestKind::GetHomePosition => {
                let message = commands::command(target, CMD_GET_HOME_POSITION, [0.0; 7]);
                self.arm(message, timeouts.home_position, now);
                Outgoing::Once(message)
            }
            RequestKind::SetFencePoint(set) => {
                let fetch = fence_fetch(target, set.idx);
                let point = fence_point(target, set);
                self.arm(fetch, timeouts.fence_fetch, now);
                self.attempts_left = FENCE_POINT_SENDS - 1;
                Outgoing::Pair(point, fetch)
            }
            RequestKind::GetFencePoint { idx } => {
                let fetch = fence_fetch(target, *idx);
                self.arm(fetch, timeouts.fence_fetch, now);
                Outgoing::Once(fetch)
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
            RequestKind::Command { .. }
            | RequestKind::SetCurrent { .. }
            | RequestKind::SetRallyPoint(_)
            | RequestKind::CommandInt { .. }
            | RequestKind::SetWp { .. }
            | RequestKind::GetHomePosition
            | RequestKind::SetFencePoint(_)
            | RequestKind::GetFencePoint { .. } => false,
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
        let (wanted, int) = match self.kind {
            RequestKind::Command { command, .. } => (command, false),
            RequestKind::CommandInt { command, .. } => (command, true),
            _ => return false,
        };
        if wanted != command {
            // "Commands dont match", and keep waiting (:2808-2813; :2927-2934).
            return false;
        }
        match result {
            // Wait a whole timeout again, with no retries left: the vehicle has the command, and
            // sending it again would start it again (:2818-2823). doCommandIntAsync has no such
            // branch - anything but ACCEPTED is its `false` (:2940-2949).
            MAV_RESULT_IN_PROGRESS if !int => {
                self.deadline = now + self.policy.timeout;
                self.retries_left = 0;
            }
            MAV_RESULT_ACCEPTED => self.finish(RequestOutcome::Accepted { value: None }),
            other => self.finish(RequestOutcome::Rejected(other)),
        }
        true
    }

    /// A `MISSION_ACK` arrived from `from`, addressed to this ground station or not. `setWPAsync`
    /// takes only one addressed to it ("check this gcs sent it", :4084-4087) and ends with its
    /// result: accepted, or the `MAV_MISSION_RESULT` the vehicle refused with (:4108-4112).
    ///
    /// Returns whether a set-WP took it, so the transfer machines do not read it as theirs.
    pub fn on_mission_ack(&mut self, from: VehicleId, to_us: bool, result: u8) -> bool {
        if self.state != RequestState::Waiting
            || from != self.target
            || !matches!(self.kind, RequestKind::SetWp { .. })
            || !to_us
        {
            return false;
        }
        if result == crate::mission_transfer::MISSION_ACCEPTED {
            self.finish(RequestOutcome::Accepted { value: None });
        } else {
            self.finish(RequestOutcome::Rejected(result));
        }
        true
    }

    /// A `MISSION_REQUEST` or `MISSION_REQUEST_INT` for `seq` arrived from `from`. For a set-WP,
    /// one addressed to this ground station asking for the item after the one sent is an
    /// acceptance (:4115-4143); any other says the vehicle is on another item, and the point goes
    /// again at once, spending a retry - `start = DateTime.MinValue` (:4177-4182) and the loop's
    /// top resends or throws.
    ///
    /// Returns whether a set-WP took it, and what to send.
    pub fn on_mission_request(
        &mut self,
        from: VehicleId,
        to_us: bool,
        seq: u16,
        now: Instant,
    ) -> (bool, Outgoing) {
        if self.state != RequestState::Waiting || from != self.target || !to_us {
            return (false, Outgoing::Nothing);
        }
        let RequestKind::SetWp { item, seq: sent } = &self.kind else {
            return (false, Outgoing::Nothing);
        };
        let (item, sent) = (**item, *sent);
        if seq == sent.wrapping_add(1) {
            self.finish(RequestOutcome::Accepted { value: None });
            return (true, Outgoing::Nothing);
        }
        if self.retries_left == 0 {
            self.finish(RequestOutcome::TimedOut);
            return (true, Outgoing::Nothing);
        }
        self.retries_left -= 1;
        self.deadline = now + self.policy.timeout;
        (true, self.count(Outgoing::Once(item)))
    }

    /// A `HOME_POSITION` arrived from `from`: it answers a `getHomePosition` (:3346-3355).
    pub fn on_home_position(&mut self, from: VehicleId) {
        if self.state == RequestState::Waiting
            && from == self.target
            && matches!(self.kind, RequestKind::GetHomePosition)
        {
            self.finish(RequestOutcome::Accepted { value: None });
        }
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

    /// A `RALLY_POINT` arrived from `from`; `to_us` is whether it is addressed to this ground
    /// station, which `getRallyPoint` requires ("check this gcs sent it", `:6390-6393`). Returns
    /// what to send: the fetch again for a point of another index, the point and the fetch again
    /// for one that came back different. See [`RallyPointSet`].
    pub fn on_rally_point(
        &mut self,
        from: VehicleId,
        to_us: bool,
        point: &RallyPoint,
        now: Instant,
    ) -> Outgoing {
        if self.state != RequestState::Waiting || from != self.target || !to_us {
            return Outgoing::Nothing;
        }
        let RequestKind::SetRallyPoint(set) = self.kind else {
            return Outgoing::Nothing;
        };
        let target = self.target;
        if point.idx != set.idx {
            // `generatePacket(FENCE_FETCH_POINT, req); continue;` - the fetch, not the fence's.
            return self.count(Outgoing::Once(rally_fetch(target, set.idx)));
        }
        if point.lat == set.lat && point.lng == set.lng {
            self.finish(RequestOutcome::Accepted { value: None });
            return Outgoing::Nothing;
        }
        if self.attempts_left == 0 {
            self.finish(RequestOutcome::Sent);
            return Outgoing::Nothing;
        }
        // `retry--` and round again: the point, and a fresh `getRallyPoint`.
        self.attempts_left -= 1;
        self.retries_left = self.policy.retries;
        self.deadline = now + self.policy.timeout;
        self.count(Outgoing::Pair(
            rally_point(target, &set),
            rally_fetch(target, set.idx),
        ))
    }

    /// A `FENCE_POINT` arrived from `from`; `to_us` is whether it is addressed to this ground
    /// station and to `MAV_COMP_ID_MISSIONPLANNER`, which `getFencePoint` requires ("check this
    /// gcs sent it", `:5953-5957`). Returns what to send: the point and the fetch again for a set
    /// whose point came back five metres or more away. See [`FencePointSet`].
    pub fn on_fence_point(
        &mut self,
        from: VehicleId,
        to_us: bool,
        point: &FencePoint,
        now: Instant,
    ) -> Outgoing {
        if self.state != RequestState::Waiting || from != self.target || !to_us {
            return Outgoing::Nothing;
        }
        let target = self.target;
        let read = FencePointRead {
            lat: point.lat,
            lng: point.lng,
            count: point.count,
        };
        match self.kind {
            RequestKind::GetFencePoint { idx } if point.idx == idx => {
                self.fence_read = Some(read);
                self.finish(RequestOutcome::Accepted { value: None });
                Outgoing::Nothing
            }
            RequestKind::SetFencePoint(set) if point.idx == set.idx => {
                self.fence_read = Some(read);
                if fence_point_matches(point, &set) {
                    self.finish(RequestOutcome::Accepted { value: None });
                    return Outgoing::Nothing;
                }
                if self.attempts_left == 0 {
                    // "Could not verify GeoFence Point".
                    self.finish(RequestOutcome::Sent);
                    return Outgoing::Nothing;
                }
                // `retry--` and round again: the point, and a fresh `getFencePoint`.
                self.attempts_left -= 1;
                self.retries_left = self.policy.retries;
                self.deadline = now + self.policy.timeout;
                self.count(Outgoing::Pair(
                    fence_point(target, &set),
                    fence_fetch(target, set.idx),
                ))
            }
            _ => Outgoing::Nothing,
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
        // doCommandAsync's retries send `req` again with its confirmation counted up; a
        // getHomePosition retry is a fresh `doCommand`, at zero (:3369).
        if let MavMessage::CommandLong(long) = &mut message
            && !matches!(self.kind, RequestKind::GetHomePosition)
        {
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

    fn rally_set() -> RallyPointSet {
        RallyPointSet {
            idx: 1,
            count: 2,
            lat: -353_632_621,
            lng: 1_491_652_374,
            alt: 60,
            break_alt: 0,
            land_dir: 0,
            flags: 0,
        }
    }

    /// The vehicle's answer to a fetch: the point at `idx`, addressed to the ground station.
    fn echo(idx: u8, lat: i32, lng: i32) -> RallyPoint {
        RallyPoint {
            lat,
            lng,
            alt: 60,
            break_alt: 0,
            land_dir: 0,
            target_system: 255,
            target_component: 190,
            idx,
            count: 2,
            flags: 0,
        }
    }

    /// `setRallyPoint`: the point, then its fetch; the same point read back sets it.
    #[test]
    fn a_rally_point_is_sent_then_fetched_and_set_when_it_reads_back_the_same() {
        let t = ProtocolTimeouts::default();
        let t0 = Instant::now();
        let mut request = Request::new(target(), RequestKind::SetRallyPoint(rally_set()));
        let Outgoing::Pair(MavMessage::RallyPoint(point), MavMessage::RallyFetchPoint(fetch)) =
            request.begin(&t, None, true, t0)
        else {
            panic!("the point and its fetch");
        };
        assert_eq!(
            (point.idx, point.count, point.lat, point.lng, point.alt),
            (1, 2, -353_632_621, 1_491_652_374, 60)
        );
        assert_eq!((point.target_system, point.target_component), (1, 1));
        assert_eq!(fetch.idx, 1);
        // Another GCS's answer, and another index's, do not end it; the second asks again.
        let answer = echo(1, -353_632_621, 1_491_652_374);
        assert_eq!(
            request.on_rally_point(target(), false, &answer, t0),
            Outgoing::Nothing
        );
        assert!(matches!(
            request.on_rally_point(target(), true, &echo(0, 0, 0), t0),
            Outgoing::Once(MavMessage::RallyFetchPoint(RallyFetchPoint { idx: 1, .. }))
        ));
        assert_eq!(request.outcome(), None);
        assert_eq!(
            request.on_rally_point(target(), true, &answer, t0),
            Outgoing::Nothing
        );
        assert_eq!(
            request.outcome(),
            Some(RequestOutcome::Accepted { value: None })
        );
        assert_eq!(request.sends(), 3);
    }

    /// Unanswered, the fetch goes four times, 700 ms apart - as `RALLY_FETCH_POINT`, where the
    /// C#'s retries send `FENCE_FETCH_POINT` (MAVLinkInterface.cs:6372) - and then it has timed
    /// out: "Failed to save rally point" to the handler.
    #[test]
    fn a_rally_point_never_read_back_times_out_after_four_fetches() {
        let t = ProtocolTimeouts::default();
        let t0 = Instant::now();
        let mut request = Request::new(target(), RequestKind::SetRallyPoint(rally_set()));
        let _ = request.begin(&t, None, true, t0);
        for step in 1..=3u32 {
            assert!(matches!(
                request.on_tick(t0 + t.rally_fetch.timeout * step),
                Outgoing::Once(MavMessage::RallyFetchPoint(_))
            ));
        }
        assert_eq!(
            request.on_tick(t0 + t.rally_fetch.timeout * 4),
            Outgoing::Nothing
        );
        assert_eq!(request.outcome(), Some(RequestOutcome::TimedOut));
        assert_eq!(request.sends(), 1 + t.rally_fetch.sends());
    }

    /// A point that comes back different is sent again, three times in all, and then left as
    /// sent: `setRallyPoint`'s `false`, which its caller ignores.
    #[test]
    fn a_rally_point_that_reads_back_different_is_sent_three_times() {
        let t = ProtocolTimeouts::default();
        let t0 = Instant::now();
        let mut request = Request::new(target(), RequestKind::SetRallyPoint(rally_set()));
        let _ = request.begin(&t, None, true, t0);
        let wrong = echo(1, -353_000_000, 1_490_000_000);
        for _ in 0..2 {
            assert!(matches!(
                request.on_rally_point(target(), true, &wrong, t0),
                Outgoing::Pair(MavMessage::RallyPoint(_), MavMessage::RallyFetchPoint(_))
            ));
        }
        assert_eq!(
            request.on_rally_point(target(), true, &wrong, t0),
            Outgoing::Nothing
        );
        assert_eq!(request.outcome(), Some(RequestOutcome::Sent));
        assert_eq!(request.sends(), 6);
    }

    fn fence_set() -> FencePointSet {
        FencePointSet {
            idx: 1,
            count: 5,
            lat: -35.363_262_1,
            lng: 149.165_237_4,
        }
    }

    /// A `FENCE_POINT` from the vehicle: point `idx` at `lat`, `lng`, addressed to the ground
    /// station.
    fn fence_echo(idx: u8, lat: f32, lng: f32) -> FencePoint {
        FencePoint {
            lat,
            lng,
            target_system: 255,
            target_component: 190,
            idx,
            count: 5,
        }
    }

    /// `setFencePoint`: the point as `(float)` degrees, then `getFencePoint`'s fetch of its index;
    /// an answer for another ground station or another index is read past without a send, and the
    /// point read back within five metres sets it.
    #[test]
    #[allow(clippy::cast_possible_truncation)]
    fn a_fence_point_is_sent_then_fetched_and_set_when_it_reads_back_within_five_metres() {
        let t = ProtocolTimeouts::default();
        let t0 = Instant::now();
        let set = fence_set();
        let mut request = Request::new(target(), RequestKind::SetFencePoint(set));
        let Outgoing::Pair(MavMessage::FencePoint(point), MavMessage::FenceFetchPoint(fetch)) =
            request.begin(&t, None, true, t0)
        else {
            panic!("the point and its fetch");
        };
        assert_eq!(
            (point.idx, point.count, point.lat, point.lng),
            (1, 5, set.lat as f32, set.lng as f32)
        );
        assert_eq!((point.target_system, point.target_component), (1, 1));
        assert_eq!((fetch.idx, fetch.target_system), (1, 1));
        let answer = fence_echo(1, set.lat as f32, set.lng as f32);
        assert_eq!(
            request.on_fence_point(target(), false, &answer, t0),
            Outgoing::Nothing
        );
        assert_eq!(
            request.on_fence_point(target(), true, &fence_echo(0, 0.0, 0.0), t0),
            Outgoing::Nothing
        );
        assert_eq!(
            request.on_fence_point(VehicleId::new(2, 1), true, &answer, t0),
            Outgoing::Nothing
        );
        assert_eq!(request.outcome(), None);
        // Three metres north is within five.
        let near = fence_echo(1, (set.lat + 0.000_027) as f32, set.lng as f32);
        assert_eq!(
            request.on_fence_point(target(), true, &near, t0),
            Outgoing::Nothing
        );
        assert_eq!(
            request.outcome(),
            Some(RequestOutcome::Accepted { value: None })
        );
        assert_eq!(request.fence_point().map(|read| read.count), Some(5));
        assert_eq!(request.sends(), 2);
    }

    /// A point read back five metres or more away is sent again with a fresh fetch, three sends
    /// in all, and then left as sent: "Could not verify GeoFence Point".
    #[test]
    #[allow(clippy::cast_possible_truncation)]
    fn a_fence_point_that_reads_back_elsewhere_is_sent_three_times() {
        let t = ProtocolTimeouts::default();
        let t0 = Instant::now();
        let set = fence_set();
        let mut request = Request::new(target(), RequestKind::SetFencePoint(set));
        let _ = request.begin(&t, None, true, t0);
        // Ten metres north.
        let far = fence_echo(1, (set.lat + 0.000_09) as f32, set.lng as f32);
        for _ in 0..2 {
            assert!(matches!(
                request.on_fence_point(target(), true, &far, t0),
                Outgoing::Pair(MavMessage::FencePoint(_), MavMessage::FenceFetchPoint(_))
            ));
        }
        assert_eq!(
            request.on_fence_point(target(), true, &far, t0),
            Outgoing::Nothing
        );
        assert_eq!(request.outcome(), Some(RequestOutcome::Sent));
        assert_eq!(request.sends(), 6);
    }

    /// Unanswered, the fetch goes four times, 700 ms apart, and the point has timed out:
    /// `getFencePoint`'s "Timeout on read - getFencePoint", which ends `setFencePoint` too.
    #[test]
    fn a_fence_point_never_read_back_times_out_after_four_fetches() {
        let t = ProtocolTimeouts::default();
        assert_eq!(t.fence_fetch.timeout, Duration::from_millis(700));
        let t0 = Instant::now();
        for kind in [
            RequestKind::SetFencePoint(fence_set()),
            RequestKind::GetFencePoint { idx: 1 },
        ] {
            let first_sends: u16 = if matches!(kind, RequestKind::SetFencePoint(_)) {
                2
            } else {
                1
            };
            let mut request = Request::new(target(), kind);
            let _ = request.begin(&t, None, true, t0);
            for step in 1..=3u32 {
                assert!(matches!(
                    request.on_tick(t0 + t.fence_fetch.timeout * step),
                    Outgoing::Once(MavMessage::FenceFetchPoint(FenceFetchPoint { idx: 1, .. }))
                ));
            }
            assert_eq!(
                request.on_tick(t0 + t.fence_fetch.timeout * 4),
                Outgoing::Nothing
            );
            assert_eq!(request.outcome(), Some(RequestOutcome::TimedOut));
            assert_eq!(request.sends(), first_sends + 3);
        }
    }

    /// `getFencePoint` alone: the fetch, and the point of that index addressed to us ends it with
    /// the position and the count the vehicle holds.
    #[test]
    fn a_fence_point_fetch_takes_the_point_of_its_index() {
        let t = ProtocolTimeouts::default();
        let t0 = Instant::now();
        let mut request = Request::new(target(), RequestKind::GetFencePoint { idx: 2 });
        assert!(matches!(
            request.begin(&t, None, true, t0),
            Outgoing::Once(MavMessage::FenceFetchPoint(FenceFetchPoint { idx: 2, .. }))
        ));
        let _ = request.on_fence_point(target(), true, &fence_echo(1, -35.0, 149.0), t0);
        assert_eq!(request.outcome(), None);
        let _ = request.on_fence_point(target(), true, &fence_echo(2, -35.5, 149.25), t0);
        assert_eq!(
            request.outcome(),
            Some(RequestOutcome::Accepted { value: None })
        );
        assert_eq!(
            request.fence_point(),
            Some(FencePointRead {
                lat: -35.5,
                lng: 149.25,
                count: 5
            })
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

    fn command_int() -> RequestKind {
        RequestKind::CommandInt {
            command: commands::CMD_DO_SET_HOME,
            frame: commands::FRAME_GLOBAL,
            params: [0.0; 4],
            x: -353_632_620,
            y: 1_491_652_370,
            z: 584.0,
            require_ack: true,
        }
    }

    /// `doCommandIntAsync`: `retrys = 3; timeout = 2000` (:2884-2885), the `COMMAND_INT` sent
    /// again as it was - it has no confirmation field - then "Timeout on read - doCommand".
    #[test]
    fn a_command_int_is_sent_four_times_then_times_out() {
        let t = ProtocolTimeouts::default();
        let t0 = Instant::now();
        let mut request = Request::new(target(), command_int());
        let mut sent = 0;
        let mut record = |send: Outgoing| {
            if let Outgoing::Once(MavMessage::CommandInt(int)) = send {
                assert_eq!(int.command, commands::CMD_DO_SET_HOME);
                assert_eq!((int.x, int.y), (-353_632_620, 1_491_652_370));
                sent += 1;
            }
        };
        record(request.begin(&t, None, true, t0));
        for step in 1..=4u32 {
            record(request.on_tick(t0 + t.command.timeout * step));
        }
        assert_eq!(sent, 4);
        assert_eq!(request.outcome(), Some(RequestOutcome::TimedOut));
        assert_eq!(request.sends(), t.command.sends());
    }

    /// Anything but ACCEPTED is `false` for doCommandIntAsync (:2940-2949): IN_PROGRESS, which
    /// doCommandAsync waits on, ends a command-int as a refusal.
    #[test]
    fn a_command_int_takes_only_its_own_ack_and_in_progress_is_a_refusal() {
        let t = ProtocolTimeouts::default();
        let t0 = Instant::now();
        let mut request = Request::new(target(), command_int());
        let _ = request.begin(&t, None, true, t0);
        assert!(!request.on_command_ack(target(), 22, MAV_RESULT_ACCEPTED, t0));
        assert!(!request.on_command_ack(
            VehicleId::new(2, 1),
            commands::CMD_DO_SET_HOME,
            MAV_RESULT_ACCEPTED,
            t0
        ));
        assert_eq!(request.outcome(), None);
        assert!(request.on_command_ack(
            target(),
            commands::CMD_DO_SET_HOME,
            MAV_RESULT_IN_PROGRESS,
            t0
        ));
        assert_eq!(
            request.outcome(),
            Some(RequestOutcome::Rejected(MAV_RESULT_IN_PROGRESS))
        );

        let mut accepted = Request::new(target(), command_int());
        let _ = accepted.begin(&t, None, true, t0);
        assert!(accepted.on_command_ack(
            target(),
            commands::CMD_DO_SET_HOME,
            MAV_RESULT_ACCEPTED,
            t0
        ));
        assert_eq!(
            accepted.outcome(),
            Some(RequestOutcome::Accepted { value: None })
        );
    }

    /// `requireack` false: sent once, not waited for (:2874-2878).
    #[test]
    fn a_command_int_without_an_ack_wanted_is_sent_once_and_done() {
        let t = ProtocolTimeouts::default();
        let RequestKind::CommandInt {
            command,
            frame,
            params,
            x,
            y,
            z,
            ..
        } = command_int()
        else {
            unreachable!()
        };
        let mut request = Request::new(
            target(),
            RequestKind::CommandInt {
                command,
                frame,
                params,
                x,
                y,
                z,
                require_ack: false,
            },
        );
        assert!(matches!(
            request.begin(&t, None, true, Instant::now()),
            Outgoing::Once(MavMessage::CommandInt(_))
        ));
        assert_eq!(request.outcome(), Some(RequestOutcome::Sent));
    }

    fn set_wp() -> RequestKind {
        RequestKind::SetWp {
            item: Box::new(commands::change_alt(target(), 25.0)),
            seq: 0,
        }
    }

    /// `setWPAsync`: `retrys = 10`, 450 ms (:4250-4254), eleven sends, then "Timeout on read -
    /// setWP" (:4258).
    #[test]
    fn a_set_wp_is_sent_eleven_times_then_times_out() {
        let t = ProtocolTimeouts::default();
        let t0 = Instant::now();
        let mut request = Request::new(target(), set_wp());
        let mut sent = 0;
        let mut record = |send: Outgoing| {
            if let Outgoing::Once(MavMessage::MissionItem(item)) = send {
                assert_eq!(item.current, 3);
                sent += 1;
            }
        };
        record(request.begin(&t, None, true, t0));
        for step in 1..=11u32 {
            record(request.on_tick(t0 + t.mission_item_send.timeout * step));
        }
        assert_eq!(sent, 11);
        assert_eq!(request.outcome(), Some(RequestOutcome::TimedOut));
        assert_eq!(request.sends(), t.mission_item_send.sends());
    }

    /// A `MISSION_ACK` to another ground station is somebody else's (:4084-4087); one to this
    /// one ends the set with its result (:4108-4112).
    #[test]
    fn a_set_wp_takes_only_an_ack_addressed_to_us() {
        let t = ProtocolTimeouts::default();
        let t0 = Instant::now();
        let mut request = Request::new(target(), set_wp());
        let _ = request.begin(&t, None, true, t0);
        assert!(!request.on_mission_ack(target(), false, 0));
        assert!(!request.on_mission_ack(VehicleId::new(2, 1), true, 0));
        assert_eq!(request.outcome(), None);
        assert!(request.on_mission_ack(target(), true, 13));
        assert_eq!(request.outcome(), Some(RequestOutcome::Rejected(13)));

        let mut accepted = Request::new(target(), set_wp());
        let _ = accepted.begin(&t, None, true, t0);
        assert!(accepted.on_mission_ack(target(), true, 0));
        assert_eq!(
            accepted.outcome(),
            Some(RequestOutcome::Accepted { value: None })
        );
    }

    /// The vehicle asking for the item after the one sent is an acceptance (:4115-4143); asking
    /// for any other sends the item again at once, spending a retry (:4177-4182), and with none
    /// left the loop's top throws.
    #[test]
    fn a_set_wp_is_accepted_by_a_request_for_the_next_item_and_resent_at_once_for_any_other() {
        let t = ProtocolTimeouts::default();
        let t0 = Instant::now();
        let mut request = Request::new(target(), set_wp());
        let _ = request.begin(&t, None, true, t0);
        assert_eq!(
            request.on_mission_request(target(), false, 1, t0),
            (false, Outgoing::Nothing)
        );
        let (took, send) = request.on_mission_request(target(), true, 7, t0);
        assert!(took);
        assert!(matches!(send, Outgoing::Once(MavMessage::MissionItem(_))));
        assert_eq!(request.sends(), 2);
        assert_eq!(request.outcome(), None);
        assert_eq!(
            request.on_mission_request(target(), true, 1, t0),
            (true, Outgoing::Nothing)
        );
        assert_eq!(
            request.outcome(),
            Some(RequestOutcome::Accepted { value: None })
        );

        let mut worn = Request::new(target(), set_wp());
        let _ = worn.begin(&t, None, true, t0);
        for _ in 0..10 {
            let (took, send) = worn.on_mission_request(target(), true, 7, t0);
            assert!(took && send != Outgoing::Nothing);
        }
        assert_eq!(worn.sends(), 11);
        assert_eq!(
            worn.on_mission_request(target(), true, 7, t0),
            (true, Outgoing::Nothing)
        );
        assert_eq!(worn.outcome(), Some(RequestOutcome::TimedOut));
    }

    /// `getHomePositionAsync`: `doCommand(GET_HOME_POSITION, ..., false)` then again three
    /// times 700 ms apart (:3357-3372), each a fresh command at confirmation zero, then
    /// "Timeout on read - getHomePosition"; any `HOME_POSITION` from the vehicle ends it.
    #[test]
    fn get_home_position_asks_four_times_at_confirmation_zero_then_times_out() {
        let t = ProtocolTimeouts::default();
        let t0 = Instant::now();
        let mut request = Request::new(target(), RequestKind::GetHomePosition);
        let mut confirmations = Vec::new();
        let mut record = |send: Outgoing| {
            if let Outgoing::Once(MavMessage::CommandLong(long)) = send {
                assert_eq!(long.command, CMD_GET_HOME_POSITION);
                confirmations.push(long.confirmation);
            }
        };
        record(request.begin(&t, None, true, t0));
        for step in 1..=4u32 {
            record(request.on_tick(t0 + t.home_position.timeout * step));
        }
        assert_eq!(confirmations, vec![0, 0, 0, 0]);
        assert_eq!(request.outcome(), Some(RequestOutcome::TimedOut));

        let mut answered = Request::new(target(), RequestKind::GetHomePosition);
        let _ = answered.begin(&t, None, true, t0);
        answered.on_home_position(VehicleId::new(2, 1));
        assert_eq!(answered.outcome(), None);
        answered.on_home_position(target());
        assert_eq!(
            answered.outcome(),
            Some(RequestOutcome::Accepted { value: None })
        );
    }
}
