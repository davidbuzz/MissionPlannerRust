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

//! Ctrl+Z: `new Camera().test(MainV2.comPort)` (`MainV2.cs:4161-4166`), `ExtLibs/ArduPilot/
//! Camera.cs`'s `test`: the first component on the link that is a camera (`MAV_COMP_ID_CAMERA`)
//! asked, one `doCommand` after another, for its information, its video stream's, its settings,
//! its mode, its storage's, and to start streaming - each with its parameters all 0. No camera,
//! nothing. The whole is in a `try` whose `catch` says nothing: a command that goes unanswered
//! through every retry throws `doCommand`'s `TimeoutException` and ends the test there; one the
//! camera refuses returns false and the next is sent.
//!
//! What differs: each `doCommand` blocks the C#'s window until its acknowledgement; here the link
//! sends and retries it, and the next goes once the last has ended, the window running meanwhile.
//! A second Ctrl+Z while one runs is kept and run after it, as the C#'s window takes the key it
//! queued once the first returns. `MAVlist` is enumerated in the order the C#'s dictionary was
//! filled; here the link's vehicles come in their stable order.
//!
//! `CameraInfo` (`Camera.cs:37-49`) has no caller in the C# and is not ported.
//!
//! `// C#: ExtLibs/ArduPilot/Camera.cs:12-35`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_link::RequestId;
use mp_link::requests::RequestOutcome;
use mp_mavlink_dialects::all::MavCmd;
use mp_vehicle::VehicleId;
use web_time::Instant;

use crate::telemetry::{Lookup, Report, Telemetry};

/// `MAV_COMPONENT.MAV_COMP_ID_CAMERA`, the component `test` looks for.
/// `// C#: ExtLibs/ArduPilot/Camera.cs:16`
pub const CAMERA_COMPONENT: u8 = 100;

/// The six commands, in the order `test` sends them; `VIDEO_STOP_STREAMING` and the four after it
/// are commented out in the C#.
/// `// C#: ExtLibs/ArduPilot/Camera.cs:21-31`
pub const COMMANDS: [MavCmd; 6] = [
    MavCmd::MAV_CMD_REQUEST_CAMERA_INFORMATION,
    MavCmd::MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION,
    MavCmd::MAV_CMD_REQUEST_CAMERA_SETTINGS,
    MavCmd::MAV_CMD_SET_CAMERA_MODE,
    MavCmd::MAV_CMD_REQUEST_STORAGE_INFORMATION,
    MavCmd::MAV_CMD_VIDEO_START_STREAMING,
];

/// How the last test ended, for the facts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Ended {
    /// None has run.
    #[default]
    None,
    /// `if (mav == null) return;`: no camera on the link.
    NoCamera,
    /// All six sent and answered, accepted or not.
    Done,
    /// One went unanswered through every retry: `doCommand`'s throw, caught.
    TimedOut,
    /// The link went away under it: nothing more can be sent.
    LinkGone,
}

impl Ended {
    /// The word a fact carries.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::NoCamera => "no-camera",
            Self::Done => "done",
            Self::TimedOut => "timed-out",
            Self::LinkGone => "link-gone",
        }
    }
}

/// A test under way: the camera, the next command's place in [`COMMANDS`], and the one waited on.
#[derive(Debug, Clone, Copy)]
struct Run {
    camera: VehicleId,
    next: usize,
    waiting: Option<(RequestId, Instant)>,
}

/// `Camera.test`, as its commands go.
#[derive(Debug, Default)]
pub struct CameraTest {
    /// The test running, if one is.
    run: Option<Run>,
    /// Presses made while one ran, each run after it.
    queued: usize,
    /// Tests started, and commands sent.
    pub started: usize,
    pub sent: usize,
    /// The camera the last test asked.
    pub camera: Option<VehicleId>,
    /// How the last test ended.
    pub ended: Ended,
}

/// `mavint.MAVlist.FirstOrDefault(a => a.compid == MAV_COMP_ID_CAMERA)`.
/// `// C#: ExtLibs/ArduPilot/Camera.cs:16`
#[must_use]
pub fn camera_of(vehicles: &[VehicleId]) -> Option<VehicleId> {
    vehicles
        .iter()
        .copied()
        .find(|id| id.compid == CAMERA_COMPONENT)
}

impl CameraTest {
    /// Ctrl+Z: the test started, or kept for when the running one ends.
    pub fn press(&mut self, telemetry: &mut Telemetry) {
        if self.run.is_some() {
            self.queued += 1;
            return;
        }
        self.start(telemetry);
    }

    /// Whether a test is under way.
    #[must_use]
    pub const fn running(&self) -> bool {
        self.run.is_some()
    }

    /// `test`: the camera found, else nothing; then its first command.
    /// `// C#: ExtLibs/ArduPilot/Camera.cs:16-21`
    fn start(&mut self, telemetry: &mut Telemetry) {
        self.started += 1;
        let Some(camera) = camera_of(&telemetry.vehicles()) else {
            self.camera = None;
            self.ended = Ended::NoCamera;
            return;
        };
        self.camera = Some(camera);
        self.run = Some(Run {
            camera,
            next: 0,
            waiting: None,
        });
        self.advance(telemetry);
    }

    /// The next command sent, or the test ended when there is none.
    fn advance(&mut self, telemetry: &mut Telemetry) {
        let Some(mut run) = self.run.take() else {
            return;
        };
        let Some(command) = COMMANDS.get(run.next) else {
            self.finish(Ended::Done, telemetry);
            return;
        };
        let command = u16::try_from(command.0).unwrap_or(u16::MAX);
        // `doCommand(mav.sysid, mav.compid, cmd, 0, 0, 0, 0, 0, 0, 0)`; the `catch` says nothing.
        let Some(id) = telemetry.command(run.camera, command, [0.0; 7], Report::default()) else {
            self.finish(Ended::LinkGone, telemetry);
            return;
        };
        self.sent += 1;
        run.next += 1;
        run.waiting = Some((id, Instant::now()));
        self.run = Some(run);
    }

    /// The test ended; a press kept meanwhile starts the next.
    fn finish(&mut self, ended: Ended, telemetry: &mut Telemetry) {
        self.run = None;
        self.ended = ended;
        if self.queued > 0 {
            self.queued -= 1;
            self.start(telemetry);
        }
    }

    /// Once a frame: the command waited on, when it has ended, followed by the next - unless it
    /// went unanswered, which is `doCommand`'s throw and the end of the test.
    /// `// C#: ExtLibs/ArduPilot/Camera.cs:14-34; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2780-2795`
    pub fn tick(&mut self, telemetry: &mut Telemetry) {
        let Some((id, made)) = self.run.and_then(|run| run.waiting) else {
            return;
        };
        let outcome = match telemetry.lookup(id, made) {
            Lookup::Found(request) => request.outcome(),
            Lookup::PickingUp => return,
            Lookup::Gone => {
                self.finish(Ended::LinkGone, telemetry);
                return;
            }
        };
        match outcome {
            None => {}
            Some(RequestOutcome::TimedOut) => self.finish(Ended::TimedOut, telemetry),
            Some(_) => self.advance(telemetry),
        }
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(test: &CameraTest) {
    use crate::facts::record;
    record("camera.test.started", test.started);
    record("camera.test.sent", test.sent);
    record("camera.test.running", test.running());
    record("camera.test.ended", test.ended.key());
    record(
        "camera.test.camera",
        test.camera.map_or_else(
            || "none".to_owned(),
            |id| format!("{}:{}", id.sysid, id.compid),
        ),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::scripted::{Vehicle, ack, until};
    use mp_link::ProtocolTimeouts;
    use mp_mavlink_dialects::all::{Heartbeat, MavMessage};

    fn fast() -> ProtocolTimeouts {
        ProtocolTimeouts::default().faster(20)
    }

    /// A camera's `HEARTBEAT`: `MAV_TYPE_CAMERA`, no autopilot.
    fn camera_heartbeat() -> MavMessage {
        MavMessage::Heartbeat(Heartbeat {
            custom_mode: 0,
            r#type: 30,
            autopilot: 8,
            base_mode: 0,
            system_status: 4,
            mavlink_version: 3,
        })
    }

    /// The `COMMAND_LONG`s the vehicle end heard, as (target, command), a resend of the one
    /// before counted once: the link sends a command again while its answer is awaited.
    fn commands(vehicle: &Vehicle) -> Vec<(u8, u8, u16)> {
        let mut heard: Vec<(u8, u8, u16)> = vehicle
            .heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::CommandLong(long) => {
                    Some((long.target_system, long.target_component, long.command))
                }
                _ => None,
            })
            .collect();
        heard.dedup();
        heard
    }

    fn command(index: usize) -> u16 {
        u16::try_from(COMMANDS[index].0).expect("a command number")
    }

    /// The six, as the C# lists them: 521, 2504, 522, 530, 525, 2502.
    #[test]
    fn the_commands_are_the_csharps_in_its_order() {
        let numbers: Vec<u16> = (0..COMMANDS.len()).map(command).collect();
        assert_eq!(numbers, [521, 2504, 522, 530, 525, 2502]);
        assert_eq!(CAMERA_COMPONENT, 100);
        let Some(source) = crate::config_coverage::source::csharp("ExtLibs/ArduPilot/Camera.cs")
        else {
            eprintln!(
                "skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner"
            );
            return;
        };
        // Each sent in this order, outside the commented-out lines.
        let live: Vec<&str> = source
            .lines()
            .filter(|line| line.contains("doCommand") && !line.trim_start().starts_with("//"))
            .collect();
        let names = [
            "REQUEST_CAMERA_INFORMATION",
            "REQUEST_VIDEO_STREAM_INFORMATION",
            "REQUEST_CAMERA_SETTINGS",
            "SET_CAMERA_MODE",
            "REQUEST_STORAGE_INFORMATION",
            "VIDEO_START_STREAMING",
        ];
        for (line, name) in live.iter().zip(names) {
            assert!(line.contains(&format!("MAV_CMD.{name},")), "{line}");
        }
    }

    /// The first component that is a camera; none, none.
    #[test]
    fn the_camera_is_the_first_component_100() {
        let autopilot = VehicleId::new(1, 1);
        let camera = VehicleId::new(1, 100);
        let other = VehicleId::new(2, 100);
        assert_eq!(camera_of(&[autopilot, camera, other]), Some(camera));
        assert_eq!(camera_of(&[autopilot]), None);
        assert_eq!(camera_of(&[]), None);
    }

    /// No camera on the link: nothing sent.
    #[test]
    fn without_a_camera_nothing_is_sent() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut test = CameraTest::default();
        test.press(&mut telemetry);
        assert_eq!(test.ended, Ended::NoCamera);
        assert_eq!(test.sent, 0);
        assert!(!test.running());
        vehicle.read();
        assert!(commands(&vehicle).is_empty());
    }

    /// A camera that answers each: the six go to it one after another, each after the last's
    /// acknowledgement - a refusal moving on as an acceptance does.
    #[test]
    fn the_six_go_one_after_another_to_the_camera() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let camera = VehicleId::new(1, CAMERA_COMPONENT);
        vehicle.send_from(camera, &camera_heartbeat());
        until("the camera to be seen", || {
            telemetry.vehicles().contains(&camera)
        });
        let mut test = CameraTest::default();
        test.press(&mut telemetry);
        assert_eq!(test.camera, Some(camera));
        for index in 0..COMMANDS.len() {
            until("the command", || {
                vehicle.read();
                commands(&vehicle).len() > index
            });
            // Only the one: the next waits for this one's answer.
            assert_eq!(commands(&vehicle).len(), index + 1);
            assert_eq!(
                commands(&vehicle)[index],
                (1, CAMERA_COMPONENT, command(index))
            );
            // The second refused (`MAV_RESULT_DENIED`), the rest accepted.
            let result = if index == 1 { 2 } else { 0 };
            vehicle.send_from(camera, &ack(command(index), result));
            until("the answer to be taken", || {
                test.tick(&mut telemetry);
                test.sent > index + 1 || !test.running()
            });
        }
        assert_eq!(test.ended, Ended::Done);
        assert_eq!(test.sent, 6);
        assert!(!test.running());
    }

    /// One unanswered through every retry is `doCommand`'s throw: the test ends there.
    #[test]
    fn an_unanswered_command_ends_the_test() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let camera = VehicleId::new(1, CAMERA_COMPONENT);
        vehicle.send_from(camera, &camera_heartbeat());
        until("the camera to be seen", || {
            telemetry.vehicles().contains(&camera)
        });
        let mut test = CameraTest::default();
        test.press(&mut telemetry);
        until("the test to give up", || {
            test.tick(&mut telemetry);
            !test.running()
        });
        assert_eq!(test.ended, Ended::TimedOut);
        assert_eq!(test.sent, 1);
        vehicle.read();
        // Sent again by the link, never moved on to the second.
        assert!(
            commands(&vehicle)
                .iter()
                .all(|&(_, _, number)| number == command(0))
        );
    }

    /// A press while one runs is run after it.
    #[test]
    fn a_press_while_one_runs_runs_after_it() {
        let (mut telemetry, _vehicle) = Vehicle::connect(fast());
        let camera = VehicleId::new(1, CAMERA_COMPONENT);
        let mut test = CameraTest {
            run: Some(Run {
                camera,
                next: COMMANDS.len(),
                waiting: None,
            }),
            ..CameraTest::default()
        };
        test.press(&mut telemetry);
        assert_eq!(test.queued, 1);
        assert_eq!(test.started, 0);
        // The running one ends: the kept press starts, and finds no camera on this link.
        test.advance(&mut telemetry);
        assert_eq!(test.started, 1);
        assert_eq!(test.queued, 0);
        assert_eq!(test.ended, Ended::NoCamera);
    }
}
