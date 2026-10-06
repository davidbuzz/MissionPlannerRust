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

//! The flight screen's Scripts tab: `tabScripts` of `FlightData`, ported from
//! `GCSViews/FlightData.cs:788-814, 1014-1019, 1225-1235, 1632-1643, 4841-4890` and
//! `FlightData.resx`, with `Script.cs` run by `mp_script`'s engine (PLAN.md §13.6 row 94, §12
//! D20).
//!
//! What it shows, at the `.resx`'s places in a 290 x 175 page: "Script Status: ..." and
//! "Selected Script: ..." labels, the "Redirect Program Output" box, Select Script, Run Script
//! (once one is selected), Abort Running Script (while one runs) and Edit Selected Script. Select
//! Script is an `OpenFileDialog` for `*.py`, a typed path here as every file dialog is; Run
//! Script reads the file and starts it on "Script Thread (new)"; Abort stops it; Edit opens the
//! file with the desktop's editor, as `Process.Start` with `UseShellExecute` does.
//!
//! The console: with the box ticked, the C# opens a `ScriptConsole` form the first time the
//! thread has an output writer, and its timer appends what the writer has written
//! (`Controls/ScriptConsole.cs`). Here the console is drawn under the buttons on the same page
//! rather than in a form of its own, with the form's Clear button and autoscroll box - a
//! divergence written here.
//!
//! The host the script calls into runs on the script's thread and cannot touch the window, so it
//! sends [`Request`]s over a channel that [`ScriptsTab::serve`] answers from the window's
//! telemetry, every 5 ms while a run lasts (a task the Run button starts, [`serve_while_running`]):
//! `GetParam` from the parameters held, `ChangeParam` as a write followed to its end,
//! `ChangeMode` through the mode table, `cs` fields from the vehicle's state, `WaitFor` from the
//! messages - a view of the telemetry taken only for a request that reads one - and for `MAV` the
//! link's own machines, each a request the link retries as the C# does, followed to its end:
//! `setParam`, `doCommand` (`doARM`), `doReboot`, `setWPTotal`, `setWP`, `setWPCurrent`, and
//! `getWP`, the one item read on its own (`MISSION_REQUEST_INT`, or `MISSION_REQUEST` to a
//! vehicle without the capability), never a mission transfer, so a script's reads and the Plan
//! screen's Read and Write keep out of each other's way. What the C# sends without waiting goes
//! straight out through the link's sender from the script's thread, as the C#'s does from its
//! own: `SendRC`, `setWPACK`, `setGuidedModeWP`'s position target, `BaseStream.Write`. The link
//! and the vehicle `MAV` talks to are the window's as they stand - handed to the script's thread
//! at each serve - so `MAV.sysidcurrent` and every member follow a vehicle heard after Run, a
//! reconnect, or another vehicle chosen, as the C#'s `MainV2.comPort` does. An error a script
//! ends in goes to the console and the status line, not a box (the owner's ruling on avoidable
//! error boxes; the C# shows "Error running script" in a box as well as in the writer).
//!
//! What the C# keeps that this does not, or keeps otherwise:
//!
//! * `MAVState.wps`, `rallypoints` and `fencepoints`, the lists as the link's traffic has shown
//!   them, which a script's `setWPTotal` empties and each `setWP` the vehicle takes refills, as
//!   the C#'s do, and the first two of which the flight map draws while no plan is being edited:
//!   kept in the link (mp-link's `mission_points` and `fence_points`, `wp_total_answered` and
//!   `file_set_wp`), with `setWPTotal`'s `WP_TOTAL`, `CMD_TOTAL` and `MIS_TOTAL`.
//! * `GuidedMode`, which `setGuidedModeWP` and `setWP` with current 2 update, is the flight
//!   screen's own (`fly::Actions::guided`), updated from here when the C# updates it.
//! * A link that stops while a script waits on it ends the wait at once in the member's
//!   `TimeoutException`, where the C#'s loop runs its retries into the closed port first.
//! * `MainV2.speechEnable` and `speech_armed_only` start from the user's settings at each Run; a
//!   script that changes them changes them for its run, not for the session as the C#'s statics.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::Lock as _;
use mp_os::RecvTimeout as _;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender, channel};
use std::sync::{Arc, Mutex, PoisonError};
use web_time::{Duration, Instant};

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_link::RequestId;
use mp_link::requests::{RequestOutcome, WpRead};
use mp_mavlink_dialects::all::{MavMessage, MissionItem as WireMissionItem};
use mp_script::api::RC_RESEND_GAP_MS;
use mp_script::{
    CsValue, Locationwp, PositionTarget, ScriptApi, ScriptHost, ScriptRun, Timeout, WpItem,
};
use mp_vehicle::{VehicleFamily, VehicleId, VehicleState};

use crate::MissionPlanner;
use crate::config::optional::{at, button, label};
use crate::config::servo_output::{Check, CheckState, check_box};
use crate::fly::GuidedMode;
use crate::telemetry::{Report, Telemetry, TelemetryView};
use crate::ui::theme;

/// `tabScripts.Size`.
/// `// C#: GCSViews/FlightData.resx (tabScripts.Size)`
pub const PAGE_SIZE: (f32, f32) = (290.0, 175.0);

/// The controls' `Location` and `Size`, from the `.resx`.
pub const STATUS_LABEL_AT: (f32, f32) = (19.0, 10.0);
pub const SELECTED_LABEL_AT: (f32, f32) = (19.0, 29.0);
pub const REDIRECT_AT: (f32, f32) = (19.0, 48.0);
pub const SELECT_AT: (f32, f32, f32, f32) = (16.0, 80.0, 71.0, 23.0);
pub const RUN_AT: (f32, f32, f32, f32) = (92.0, 80.0, 71.0, 23.0);
pub const ABORT_AT: (f32, f32, f32, f32) = (169.0, 80.0, 80.0, 23.0);
pub const EDIT_AT: (f32, f32, f32, f32) = (255.0, 79.0, 80.0, 23.0);

/// The labels' texts.
/// `// C#: GCSViews/FlightData.resx; GCSViews/FlightData.cs:797, 1638, 4873`
pub const STATUS_NONE: &str = "Script Status: No Script Running";
pub const STATUS_RUNNING: &str = "Script Status: Running";
pub const STATUS_FINISHED: &str = "Script Status: Finished (or aborted)";
pub const SELECTED_NONE: &str = "Selected Script: None";
pub const REDIRECT_TEXT: &str = "Redirect Program Output";

/// The buttons' texts.
pub const SELECT_TEXT: &str = "Select Script";
pub const RUN_TEXT: &str = "Run Script";
pub const ABORT_TEXT: &str = "Abort Running Script";
pub const EDIT_TEXT: &str = "Edit Selected Script";

/// How long the script's thread waits for the window to answer one request before it gives up
/// on it: a window that has gone is a request never answered.
const ANSWER_WAIT: Duration = Duration::from_secs(10);

/// How long it waits for the end of a request the link retries: longer than the longest of
/// them - arming's ten seconds four times over - so the link's own timeout, the C#'s, is always
/// the one that ends it; and a link that stops ends every request it holds at once.
const LINK_WAIT: Duration = Duration::from_secs(300);

/// How often a wait on the window looks at the Abort button.
const ABORT_LOOK: Duration = Duration::from_millis(20);

/// How often the window answers while a script runs: `WaitFor`'s own poll.
/// `// C#: Script.cs:161`
pub const SERVE_INTERVAL: Duration = Duration::from_millis(5);

/// What a script asks the window for, from its own thread.
#[derive(Debug, Clone, PartialEq)]
pub enum Request {
    /// `Script.GetParam`.
    GetParam(String),
    /// `Script.ChangeParam`.
    ChangeParam(String, f32),
    /// `Script.ChangeMode`.
    ChangeMode(String),
    /// `Script.WaitFor`'s poll.
    HasMessage(String),
    /// `cs.messages.Clear()`.
    ClearMessages,
    /// `cs.<field>`.
    CsField(String),
    /// `MAVlist[sysid, compid].cs.<field>`: a named vehicle's.
    CsFieldOf { target: VehicleId, name: String },
    /// `setMode(sysid, compid, mode)`: a named vehicle's mode.
    SetModeOf { target: VehicleId, mode: String },
    /// `MAV.BaseStream.IsOpen`.
    IsOpen,
    /// `MAV.setParam(sysid, compid, name, value, force)`.
    SetParam {
        target: VehicleId,
        name: String,
        value: f64,
        force: bool,
    },
    /// `MAV.doCommand` with its acknowledgement waited for.
    Command {
        target: VehicleId,
        command: u16,
        params: [f32; 7],
    },
    /// `MAV.doReboot(bootloadermode)`.
    Reboot { bootloader: bool },
    /// `MAV.setWPTotal`: `MISSION_COUNT` until the vehicle asks for the first item.
    SetWpTotal {
        target: VehicleId,
        total: u16,
        mission_type: u8,
    },
    /// `MAV.setWP`: the `MISSION_ITEM`.
    SetWp { target: VehicleId, item: WpItem },
    /// `MAV.setWPCurrent`.
    SetWpCurrent { target: VehicleId, seq: u16 },
    /// `MAV.getWP(sysid, compid, index, type)`.
    GetWp {
        target: VehicleId,
        index: u16,
        mission_type: u8,
    },
    /// What `setPositionTargetGlobalInt` does to `GuidedMode` as it sends: told, not answered.
    Guided(GuidedUpdate),
    /// `MainV2.speechEngine.SpeakAsync`'s text.
    Speak(String),
}

/// The window's answer.
#[derive(Debug, Clone, PartialEq)]
pub enum Reply {
    Bool(bool),
    Param(Option<f32>),
    Field(Option<CsValue>),
    Done,
    /// A `MAV_MISSION_RESULT`.
    Result(u8),
    /// A `getWP`'s item.
    Wp(Locationwp),
    /// The C#'s `TimeoutException`, its message.
    TimedOut(String),
}

/// One request with the channel its answer comes back on.
pub type Asked = (Request, Sender<Reply>);

/// The link and the vehicle being flown as the window last saw them, for the script's thread.
type LinkSlot = Arc<Mutex<Option<(mp_link::LinkSender, VehicleId)>>>;

/// A change to `MAV.GuidedMode`: `setPositionTargetGlobalInt`'s - the position where it is not
/// zero, the height always, the frame left - or a guided `setWP`'s, the whole item.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4527-4534, 4100-4103, 4138-4141`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidedUpdate {
    /// `GuidedMode.x`, degrees x 1e7, when it changes.
    pub x: Option<i32>,
    /// `GuidedMode.y`, likewise.
    pub y: Option<i32>,
    /// `GuidedMode.z`.
    pub z: f32,
    /// `GuidedMode.frame`, when it changes.
    pub frame: Option<u8>,
}

impl GuidedUpdate {
    /// `MAVlist[sysid, compid].GuidedMode.x = (int) (lat * 1e7)` and the rest, as
    /// `setPositionTargetGlobalInt` writes them for a position.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4527-4534`
    #[allow(clippy::cast_possible_truncation)] // `(int)`, `(float)`
    fn position_target(position: &PositionTarget) -> Self {
        Self {
            x: (position.lat != 0.0).then_some((position.lat * 1e7) as i32),
            y: (position.lng != 0.0).then_some((position.lng * 1e7) as i32),
            z: position.alt as f32,
            frame: None,
        }
    }

    /// `GuidedMode = (Locationwp) req` for the `MISSION_ITEM` a guided `setWP` sent: the float
    /// position into `Locationwp` and out into `GuidedMode`'s `mavlink_mission_item_int_t`, times
    /// 1e7 for a location command.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4100-4103; ExtLibs/Utilities/locationwp.cs:77-96, 153-178`
    #[allow(clippy::cast_possible_truncation)] // `(int)`
    fn guided_item(item: &WpItem) -> Self {
        let location = mp_mission::MissionItem {
            command: item.command,
            ..mp_mission::MissionItem::default()
        }
        .is_navigation();
        let scale = if location { 1e7 } else { 1.0 };
        Self {
            x: Some((f64::from(item.x) * scale) as i32),
            y: Some((f64::from(item.y) * scale) as i32),
            z: item.z,
            frame: Some(item.frame),
        }
    }

    /// Onto the flight screen's `GuidedMode`.
    pub fn apply(&self, guided: &mut GuidedMode) {
        if let Some(x) = self.x {
            guided.x = x;
        }
        if let Some(y) = self.y {
            guided.y = y;
        }
        guided.z = self.z;
        if let Some(frame) = self.frame {
            guided.frame = frame;
        }
    }
}

/// The [`ScriptHost`] on the script's thread: everything but `SendRC` and `Sleep` goes to the
/// window over the channel.
pub struct GuiScriptHost {
    asks: Sender<Asked>,
    /// `MainV2.comPort` as the window last handed it over: see [`ScriptsTab::serve`].
    link: LinkSlot,
    api: ScriptApi,
    /// The Abort button: a wait on the window is given up when it is pressed.
    abort: Arc<AtomicBool>,
    /// "speechenable" and "speech_armed_only" as the run started.
    speech: (bool, bool),
}

impl std::fmt::Debug for GuiScriptHost {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GuiScriptHost")
            .field("has_link", &self.current().is_some())
            .finish_non_exhaustive()
    }
}

impl GuiScriptHost {
    /// The link and the vehicle being flown, as the window last saw them.
    fn current(&self) -> Option<(mp_link::LinkSender, VehicleId)> {
        self.link
            .os_lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    fn ask(&self, request: Request) -> Option<Reply> {
        self.ask_within(request, ANSWER_WAIT)
    }

    /// The window's answer, or none: not within `wait`, the window gone, or the Abort button
    /// pressed meanwhile.
    fn ask_within(&self, request: Request, wait: Duration) -> Option<Reply> {
        let (tx, rx) = channel();
        self.asks.send((request, tx)).ok()?;
        let deadline = Instant::now() + wait;
        loop {
            if self.abort.load(Ordering::Relaxed) {
                return None;
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return None;
            }
            match rx.os_recv_timeout(left.min(ABORT_LOOK)) {
                Ok(reply) => return Some(reply),
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return None,
            }
        }
    }

    /// Tells the window something it answers nothing to.
    fn tell(&self, request: Request) {
        let (tx, _rx) = channel();
        let _ = self.asks.send((request, tx));
    }

    /// A request the link retries, to its end: true or false, or the member's timeout - also
    /// when the window never answers, as a port that has gone never does.
    fn ask_link(&self, request: Request, member: &str) -> Result<Reply, Timeout> {
        match self.ask_within(request, LINK_WAIT) {
            Some(Reply::TimedOut(text)) => Err(Timeout(text)),
            Some(reply) => Ok(reply),
            None => Err(Timeout::on(member)),
        }
    }

    /// A message the C# puts on the wire without waiting, sent from this thread as the C#'s
    /// `generatePacket` is. False with no link.
    fn send(&self, message: &MavMessage) -> bool {
        self.current()
            .is_some_and(|(sender, _)| sender.send(message))
    }
}

/// The ids a script names as a vehicle.
fn vehicle((sysid, compid): (u8, u8)) -> VehicleId {
    VehicleId::new(sysid, compid)
}

impl ScriptHost for GuiScriptHost {
    fn get_parameter(&self, name: &str) -> Option<f32> {
        match self.ask(Request::GetParam(name.to_owned())) {
            Some(Reply::Param(value)) => value,
            _ => None,
        }
    }

    fn change_param(&mut self, name: &str, value: f32) -> bool {
        matches!(
            self.ask(Request::ChangeParam(name.to_owned(), value)),
            Some(Reply::Bool(true))
        )
    }

    fn change_mode(&mut self, mode: &str) -> bool {
        // `setMode` and a literal true, whatever the vehicle did. `// C#: Script.cs:149-153`
        let _ = self.ask(Request::ChangeMode(mode.to_owned()));
        true
    }

    fn has_message(&self, text: &str) -> bool {
        matches!(
            self.ask(Request::HasMessage(text.to_owned())),
            Some(Reply::Bool(true))
        )
    }

    fn clear_messages(&mut self) {
        let _ = self.ask(Request::ClearMessages);
    }

    fn cs_field(&self, name: &str) -> Option<CsValue> {
        match self.ask(Request::CsField(name.to_owned())) {
            Some(Reply::Field(value)) => value,
            _ => None,
        }
    }

    /// The override the object holds, the channel set, sent twice 20 ms apart when asked.
    /// `// C#: Script.cs:169-215`
    fn send_rc(&mut self, channel: u8, pwm: u16, send_now: bool) -> bool {
        let known = self.api.set_channel(channel, pwm);
        if !send_now {
            return known;
        }
        let Some((sender, vehicle)) = self.current() else {
            return false;
        };
        let message = mp_link::commands::rc_override(vehicle, self.api.overrides());
        let first = sender.send(&message);
        wasm_thread::sleep(Duration::from_millis(u64::from(RC_RESEND_GAP_MS)));
        let second = sender.send(&message);
        first && second
    }

    fn sleep(&mut self, milliseconds: u32) {
        wasm_thread::sleep(Duration::from_millis(u64::from(milliseconds)));
    }

    fn cs_field_of(&self, target: (u8, u8), name: &str) -> Option<CsValue> {
        let request = Request::CsFieldOf {
            target: vehicle(target),
            name: name.to_owned(),
        };
        match self.ask(request) {
            Some(Reply::Field(value)) => value,
            _ => None,
        }
    }

    fn set_mode_of(&mut self, target: (u8, u8), mode: &str) {
        let _ = self.ask(Request::SetModeOf {
            target: vehicle(target),
            mode: mode.to_owned(),
        });
    }

    /// `sysidcurrent`, `compidcurrent`: the vehicle being flown now, (0, 0) before one is heard.
    fn link_target(&self) -> (u8, u8) {
        self.current()
            .map_or((0, 0), |(_, id)| (id.sysid, id.compid))
    }

    fn is_open(&self) -> bool {
        matches!(self.ask(Request::IsOpen), Some(Reply::Bool(true)))
    }

    fn set_param(
        &mut self,
        target: (u8, u8),
        name: &str,
        value: f64,
        force: bool,
    ) -> Result<bool, Timeout> {
        let request = Request::SetParam {
            target: vehicle(target),
            name: name.to_owned(),
            value,
            force,
        };
        let reply = self.ask_link(request, &format!("setParam {name}"))?;
        Ok(reply == Reply::Bool(true))
    }

    /// Waited for through the link's `doCommand`; not waited for, sent once from here and
    /// true, as `doCommandAsync` returns after its `generatePacket` (`:2712-2716`) - false with
    /// the port closed (`:2693-2694`).
    fn command(
        &mut self,
        target: (u8, u8),
        command: u16,
        params: [f32; 7],
        require_ack: bool,
    ) -> Result<bool, Timeout> {
        if !require_ack {
            let message = mp_link::commands::command_long(vehicle(target), command, params);
            return Ok(self.send(&message));
        }
        let request = Request::Command {
            target: vehicle(target),
            command,
            params,
        };
        Ok(self.ask_link(request, "doCommand")? == Reply::Bool(true))
    }

    /// The window's reboot: `Telemetry::reboot`, which also looks at a serial port afterwards
    /// as the C# does.
    fn reboot(&mut self, bootloader: bool) -> Result<bool, Timeout> {
        Ok(self.ask_link(Request::Reboot { bootloader }, "doCommand")? == Reply::Bool(true))
    }

    /// `MISSION_COUNT` through the link's `setWPTotal`, to the vehicle's first request.
    fn set_wp_total(&mut self, target: (u8, u8), total: u16, kind: u8) -> Result<(), Timeout> {
        let request = Request::SetWpTotal {
            target: vehicle(target),
            total,
            mission_type: kind,
        };
        self.ask_link(request, "setWPTotal").map(|_| ())
    }

    fn set_wp(&mut self, target: (u8, u8), item: &WpItem) -> Result<u8, Timeout> {
        let request = Request::SetWp {
            target: vehicle(target),
            item: *item,
        };
        match self.ask_link(request, "setWP")? {
            Reply::Result(result) => Ok(result),
            _ => Err(Timeout::on("setWP")),
        }
    }

    fn set_wp_ack(&mut self, target: (u8, u8), kind: u8) {
        // `type = 0`, accepted. `// C#: MAVLinkInterface.cs:2441-2449`
        self.send(&mp_link::commands::send_mission_ack(
            vehicle(target),
            0,
            kind,
        ));
    }

    fn set_wp_current(&mut self, target: (u8, u8), seq: u16) -> Result<bool, Timeout> {
        let request = Request::SetWpCurrent {
            target: vehicle(target),
            seq,
        };
        Ok(self.ask_link(request, "setWPCurrent")? == Reply::Bool(true))
    }

    /// The one item, of the list asked for, through the link's `getWP`.
    fn get_wp(&mut self, target: (u8, u8), index: u16, kind: u8) -> Result<Locationwp, Timeout> {
        let request = Request::GetWp {
            target: vehicle(target),
            index,
            mission_type: kind,
        };
        match self.ask_link(request, "getWP")? {
            Reply::Wp(wp) => Ok(wp),
            _ => Err(Timeout::on("getWP")),
        }
    }

    /// Sent from here, and `GuidedMode` told, as `setPositionTargetGlobalInt` writes it before
    /// it sends. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4500-4555`
    fn set_position_target(&mut self, target: (u8, u8), position: &PositionTarget) -> bool {
        self.tell(Request::Guided(GuidedUpdate::position_target(position)));
        self.send(&mp_link::commands::guided_position_target(
            vehicle(target),
            position.frame,
            position.lat,
            position.lng,
            position.alt,
        ))
    }

    fn write_raw(&mut self, bytes: &[u8]) -> bool {
        self.current()
            .is_some_and(|(sender, _)| sender.write_raw(bytes))
    }

    fn speak(&mut self, text: &str) {
        let _ = self.ask(Request::Speak(text.to_owned()));
    }

    fn speech_settings(&self) -> (bool, bool) {
        self.speech
    }
}

/// `setParam(sysid, compid, name, value, force)` with the port closed: what `setParamAsync`
/// decides before it sends - false for a name the vehicle has not listed, true for a value it
/// already holds unless `force` - and otherwise its retries into the closed port and its
/// timeout. `held` is that vehicle's value, compared exactly as the C# compares it.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1637-1648, 1757-1763`
fn closed_port_set_param(held: Option<f64>, name: &str, value: f64, force: bool) -> Reply {
    match held {
        None => Reply::Bool(false),
        #[allow(clippy::float_cmp)] // `param.Value == value`
        Some(held) if held == value && !force => Reply::Bool(true),
        Some(_) => Reply::TimedOut(Timeout::on(&format!("setParam {name}")).0),
    }
}

/// `setWPAsync`'s `mavlink_mission_item_t`, `use_int` false.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4020-4039`
fn mission_item(target: VehicleId, item: &WpItem) -> MavMessage {
    MavMessage::MissionItem(WireMissionItem {
        param1: item.params[0],
        param2: item.params[1],
        param3: item.params[2],
        param4: item.params[3],
        x: item.x,
        y: item.y,
        z: item.z,
        seq: item.seq,
        command: item.command,
        target_system: target.sysid,
        target_component: target.compid,
        frame: item.frame,
        current: item.current,
        autocontinue: item.autocontinue,
        mission_type: item.mission_type,
    })
}

/// `getWPAsync`'s `Locationwp` from the item the link read (see [`WpRead`]).
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3494-3540`
const fn locationwp(read: &WpRead) -> Locationwp {
    let [p1, p2, p3, p4] = read.params;
    Locationwp {
        id: read.id,
        p1,
        p2,
        p3,
        p4,
        lat: read.lat,
        lng: read.lng,
        alt: read.alt,
        frame: read.frame,
    }
}

/// What answering a request needs of the window, apart from the writes and the mode it sets:
/// the parameters held, the messages so far and the vehicle's state.
pub struct Answers<'a> {
    pub parameters: &'a [(String, f64)],
    pub messages: &'a [mp_link::messages::LogMessage],
    pub state: Option<&'a VehicleState>,
    /// Whether the link is up: `BaseStream.IsOpen`, `cs.connected`.
    pub connected: bool,
}

/// What a request asks the window to do beyond answering.
#[derive(Debug, Clone, PartialEq)]
pub enum Effect {
    /// `setParam`: a write, answered when it ends.
    Write(String, f32),
    /// `setMode`.
    SetMode(u32),
    /// `cs.messages.Clear()`.
    ClearMessages,
    /// `SpeakAsync`: the text, for the facts.
    Speak(String),
}

/// `Host.cs.<name>`: the fields the Scripts tab reads by the C#'s names, the quick view's whole
/// table, and `connected` and `firmware`.
#[must_use]
pub fn cs_value(state: Option<&VehicleState>, connected: bool, name: &str) -> Option<CsValue> {
    match name {
        "connected" => return Some(CsValue::Flag(connected)),
        // `cs.firmware`: the C#'s `Firmwares` name of the vehicle's family.
        "firmware" => {
            let family = state.and_then(|state| VehicleFamily::from_mav_type(state.vehicle_type));
            return Some(CsValue::Text(
                match family {
                    Some(VehicleFamily::Copter) => "ArduCopter2",
                    Some(VehicleFamily::Plane) => "ArduPlane",
                    Some(VehicleFamily::Rover) => "ArduRover",
                    None => "Other",
                }
                .to_owned(),
            ));
        }
        _ => {}
    }
    let state = state?;
    if let Some(value) = cs_field(state, name) {
        return Some(value);
    }
    crate::quick::value(name, state).map(CsValue::Number)
}

/// `cs.<field>` from the vehicle's state, by the C#'s names, for the fields the corpus and the
/// Status tab read most; a name not listed is an `AttributeError` in the script.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs`
pub fn cs_field(state: &VehicleState, name: &str) -> Option<CsValue> {
    Some(match name {
        // `target_bearing - yaw`, the C#'s yaw wrapped into 0 to 360, as example3 reads it.
        // `// C#: ExtLibs/ArduPilot/CurrentState.cs:1111-1118, 274-283`
        "ber_error" => CsValue::Number(
            f64::from(state.nav.target_bearing)
                - crate::quick::value("yaw", state)
                    .unwrap_or_else(|| state.attitude.yaw.0.to_degrees()),
        ),
        // `cs.lat`/`cs.lng` are 0 until a fix, as the C#'s doubles start at 0.
        "lat" => CsValue::Number(state.position.map_or(0.0, |p| p.latitude())),
        "lng" => CsValue::Number(state.position.map_or(0.0, |p| p.longitude())),
        // `cs.alt` is the altitude above home, `cs.altasl` above sea level.
        "alt" => CsValue::Number(state.altitude_relative.0),
        "altasl" => CsValue::Number(state.altitude_msl.0),
        "roll" => CsValue::Number(state.attitude.roll.0.to_degrees()),
        "pitch" => CsValue::Number(state.attitude.pitch.0.to_degrees()),
        "yaw" => CsValue::Number(state.attitude.yaw.0.to_degrees()),
        "groundspeed" => CsValue::Number(state.ground_speed.0),
        "airspeed" => CsValue::Number(state.air_speed.0),
        "satcount" => CsValue::Number(f64::from(state.gps.satellites_visible)),
        "gpshdop" => CsValue::Number(f64::from(state.gps.hdop)),
        "battery_voltage" => CsValue::Number(f64::from(state.battery.voltage)),
        "armed" => CsValue::Flag(state.armed),
        "mode" => CsValue::Text(crate::fly::mode_name(state).unwrap_or("").to_owned()),
        _ => return None,
    })
}

/// Answers one request from what the window knows, and says what the window must do for it.
/// A `ChangeParam` is not answered here: its answer is the write's end.
pub fn service(request: &Request, answers: &Answers<'_>) -> (Option<Reply>, Option<Effect>) {
    match request {
        Request::GetParam(name) => {
            // `(float)MainV2.comPort.MAV.param[param].Value`: the C#'s own narrowing.
            #[allow(clippy::cast_possible_truncation)]
            let value = answers
                .parameters
                .iter()
                .find(|(param, _)| param == name)
                .map(|(_, value)| *value as f32);
            (Some(Reply::Param(value)), None)
        }
        Request::ChangeParam(name, value) => (None, Some(Effect::Write(name.clone(), *value))),
        Request::ChangeMode(mode) => {
            let number = answers
                .state
                .and_then(|state| VehicleFamily::from_mav_type(state.vehicle_type))
                .and_then(|family| family.mode_number(mode));
            (Some(Reply::Bool(true)), number.map(Effect::SetMode))
        }
        Request::HasMessage(text) => (
            Some(Reply::Bool(
                answers
                    .messages
                    .iter()
                    .any(|message| message.text.contains(text.as_str())),
            )),
            None,
        ),
        Request::ClearMessages => (Some(Reply::Done), Some(Effect::ClearMessages)),
        // The Plugins host's `cs`: these fields, `connected`, `firmware` and the quick view's
        // table - `setGuidedModeWP` reads `cs.firmware`.
        Request::CsField(name) => (
            Some(Reply::Field(cs_value(
                answers.state,
                answers.connected,
                name,
            ))),
            None,
        ),
        Request::IsOpen => (Some(Reply::Bool(answers.connected)), None),
        Request::Speak(text) => (Some(Reply::Done), Some(Effect::Speak(text.clone()))),
        // The link's: answered by `ScriptsTab::serve` when they end; `GuidedMode`'s, by no one.
        Request::SetParam { .. }
        | Request::Command { .. }
        | Request::Reboot { .. }
        | Request::SetWpTotal { .. }
        | Request::SetWp { .. }
        | Request::SetWpCurrent { .. }
        | Request::GetWp { .. }
        | Request::CsFieldOf { .. }
        | Request::SetModeOf { .. }
        | Request::Guided(_) => (None, None),
    }
}

/// Whether answering a request reads the window's telemetry: the view is taken for these alone.
const fn reads_the_view(request: &Request) -> bool {
    matches!(
        request,
        Request::GetParam(_)
            | Request::ChangeMode(_)
            | Request::HasMessage(_)
            | Request::CsField(_)
    )
}

/// The tab's state between frames.
#[derive(Debug)]
pub struct ScriptsTab {
    /// `selectedscript`.
    pub selected: Option<PathBuf>,
    /// `checkBoxRedirectOutput.Checked`.
    pub redirect: bool,
    /// `labelScriptStatus.Text`.
    pub status: &'static str,
    /// The run, while there is one.
    run: Option<ScriptRun>,
    /// The requests the run's host sends.
    asks: Option<Receiver<Asked>>,
    /// The link the run's host sends on, handed over at each serve.
    link: LinkSlot,
    /// The run's Abort button, which its host sees too.
    abort: Arc<AtomicBool>,
    /// Requests the link has under way, and who is waiting for each.
    waiting: Vec<Waiting>,
    /// Link requests that need the telemetry to themselves to start - a command, a set-WP, a
    /// set-current - held for the next [`ScriptsTab::serve`].
    to_start: Vec<Asked>,
    /// How many `setWP`s of this run the vehicle accepted: a fact.
    pub wps_accepted: usize,
    /// What `SpeakAsync` was last asked to say: a fact, as speech is DELIVERABLES Deliverable 15.
    pub spoken: Option<String>,
    /// `textOutput.Text`.
    pub console: String,
    /// Whether the console is drawn: the C#'s form, opened when the run has a writer.
    pub console_shown: bool,
    /// `autoscrollCheckbox.Checked`.
    pub autoscroll: bool,
    /// How the last run ended.
    pub result: Option<Result<(), String>>,
    /// What the status line should say this frame, once.
    status_line: Option<String>,
    /// How many views of the telemetry answering has taken for itself.
    views_taken: u64,
}

impl Default for ScriptsTab {
    fn default() -> Self {
        Self {
            selected: None,
            redirect: false,
            status: STATUS_NONE,
            run: None,
            asks: None,
            link: LinkSlot::default(),
            abort: Arc::default(),
            waiting: Vec::new(),
            to_start: Vec::new(),
            wps_accepted: 0,
            spoken: None,
            console: String::new(),
            console_shown: false,
            autoscroll: true,
            result: None,
            status_line: None,
            views_taken: 0,
        }
    }
}

impl ScriptsTab {
    /// `labelSelectedScript.Text`.
    #[must_use]
    pub fn selected_text(&self) -> String {
        self.selected.as_ref().map_or_else(
            || SELECTED_NONE.to_owned(),
            |path| format!("Selected Script: {}", path.display()),
        )
    }

    /// `scriptrunning`.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.run.as_ref().is_some_and(ScriptRun::is_running)
    }

    /// `BUT_select_script_Click`: the dialog's answer, a path; an empty one is Cancel, which
    /// clears the selection as the C# does.
    /// `// C#: GCSViews/FlightData.cs:1632-1643`
    pub fn select(&mut self, path: &str) {
        let path = path.trim();
        self.selected = (!path.is_empty()).then(|| PathBuf::from(path));
    }

    /// `BUT_run_script_Click`: nothing unless the file exists; else the run started, the status
    /// "Running", and the console opened when the output is redirected. `MainV2`'s speech
    /// starts from the user's "speechenable" and "speech_armed_only" in `settings`.
    /// `// C#: GCSViews/FlightData.cs:788-814, 4841-4846; MainV2.cs:658, 1007-1008`
    pub fn run_pressed(&mut self, telemetry: &Telemetry, settings: &crate::settings::Persisted) {
        let speech = (
            crate::raw_params::get_boolean(settings.get("speechenable")),
            crate::raw_params::get_boolean(settings.get("speech_armed_only")),
        );
        if self.is_running() {
            return;
        }
        let Some(path) = self.selected.clone() else {
            return;
        };
        let source = match mp_os::fs::read_to_string(&path) {
            Ok(source) => source,
            Err(err) => {
                self.status_line = Some(format!("could not read {}: {err}", path.display()));
                return;
            }
        };
        self.start(&path, source, telemetry.send_handle(), speech);
    }

    /// The run itself, from a source already read.
    fn start(
        &mut self,
        path: &Path,
        source: String,
        sender: Option<(mp_link::LinkSender, VehicleId)>,
        speech: (bool, bool),
    ) {
        let (asks_tx, asks_rx) = channel();
        self.link = Arc::new(Mutex::new(sender));
        self.abort = Arc::default();
        let host = GuiScriptHost {
            asks: asks_tx,
            link: Arc::clone(&self.link),
            api: ScriptApi::new(),
            abort: Arc::clone(&self.abort),
            speech,
        };
        let name = path.file_name().map_or_else(
            || path.display().to_string(),
            |n| n.to_string_lossy().into_owned(),
        );
        self.console.clear();
        self.console_shown = self.redirect;
        self.result = None;
        self.status = STATUS_RUNNING;
        self.waiting.clear();
        self.to_start.clear();
        self.wps_accepted = 0;
        self.spoken = None;
        self.asks = Some(asks_rx);
        self.run = Some(ScriptRun::start_with_abort(
            &name,
            source,
            Box::new(host),
            Arc::clone(&self.abort),
        ));
    }

    /// `BUT_abort_script_Click`: the script stopped at its next wait, or in the one it is in -
    /// a `Sleep`, a `WaitFor`, a `MAV` member waiting on the link.
    /// `// C#: GCSViews/FlightData.cs:1014-1019`
    pub fn abort_pressed(&mut self) {
        if let Some(run) = &self.run {
            run.abort();
        }
    }

    /// `BUT_edit_selected_Click`: the file opened with the desktop's editor, `Process.Start`
    /// with `UseShellExecute`; the C# swallows every failure, this puts it on the status line.
    /// `// C#: GCSViews/FlightData.cs:1225-1235`
    pub fn edit_pressed(&mut self) {
        let Some(path) = &self.selected else {
            return;
        };
        if let Err(err) = open_with_shell(path) {
            self.status_line = Some(format!("could not open {}: {err}", path.display()));
        }
    }

    /// `BUT_clear_Click` on the console.
    pub fn clear_console(&mut self) {
        self.console.clear();
    }

    /// While a run lasts, every [`SERVE_INTERVAL`] from the task the Run button starts: the
    /// link handed to the script's thread as it stands, every request answered - a view of the
    /// telemetry taken only for one that reads it - the link's requests started, the ones that
    /// need the telemetry to themselves too, and followed; the flight screen's `GuidedMode`
    /// changed where a script changes it. Whether the run still wants serving.
    pub fn serve(&mut self, telemetry: &mut Telemetry, guided: &mut GuidedMode) -> bool {
        self.answer_requests(telemetry, None, guided);
        for (request, reply) in std::mem::take(&mut self.to_start) {
            self.start_link(telemetry, request, reply);
        }
        self.is_running() || !self.waiting.is_empty() || !self.to_start.is_empty()
    }

    /// A link request that needs `&mut Telemetry` to start, started.
    fn start_link(&mut self, telemetry: &mut Telemetry, request: Request, reply: Sender<Reply>) {
        // With no link, or its port closed: what each does then - a command false
        // (`doCommandAsync`, `:2693-2694`), a set-WP and a set-current the timeout they reach.
        let refused = match &request {
            Request::SetWp { .. } => Reply::TimedOut(Timeout::on("setWP").0),
            Request::SetWpCurrent { .. } => Reply::TimedOut(Timeout::on("setWPCurrent").0),
            _ => Reply::Bool(false),
        };
        if matches!(request, Request::Command { .. } | Request::Reboot { .. })
            && !telemetry.is_open()
        {
            let _ = reply.send(refused);
            return;
        }
        let started = match request {
            Request::Command {
                target,
                command,
                params,
            } => telemetry
                .command(target, command, params, Report::default())
                .map(|id| Waiting::Command {
                    id,
                    reply: reply.clone(),
                }),
            // `doReboot(true)`: 3, into the bootloader, through `doCommand` as a plain reboot
            // goes. `// C#: MAVLinkInterface.cs:2555-2564`
            Request::Reboot { .. } => telemetry
                .send_handle()
                .and_then(|(_, id)| {
                    telemetry.command(
                        id,
                        mp_script::api::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                        [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        Report::default(),
                    )
                })
                .map(|id| Waiting::Command {
                    id,
                    reply: reply.clone(),
                }),
            Request::SetWp { target, item } => telemetry
                .set_wp(target, mission_item(target, &item), Report::default())
                .map(|id| Waiting::SetWp {
                    id,
                    guided: (item.current == 2).then(|| GuidedUpdate::guided_item(&item)),
                    reply: reply.clone(),
                }),
            Request::SetWpCurrent { target, seq } => telemetry
                .set_current_waypoint(target, seq, Report::default())
                .map(|id| Waiting::SetCurrent {
                    id,
                    reply: reply.clone(),
                }),
            _ => None,
        };
        match started {
            Some(waiting) => self.waiting.push(waiting),
            None => {
                let _ = reply.send(refused);
            }
        }
    }

    /// Once a frame: the run's output appended, the run's end noticed, and every request the
    /// script has made answered from the telemetry. Returns a line for the status line, if any.
    /// `// C#: GCSViews/FlightData.cs:4871-4890 (scriptChecker_Tick)`
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        guided: &mut GuidedMode,
    ) -> Option<String> {
        self.answer_requests(telemetry, Some(view), guided);
        if let Some(run) = &mut self.run {
            let fresh = run.take_output();
            self.console.push_str(&fresh);
            if !run.is_running() && self.result.is_none() {
                let result = run.result().unwrap_or(Ok(()));
                if let Err(text) = &result {
                    // `OutputWriter.Write(FormatException(e))` and the box, here the console and
                    // the status line.
                    self.console.push_str("Error running script ");
                    self.console.push_str(text);
                    let first = text.lines().last().unwrap_or("").trim();
                    self.status_line = Some(format!("Error running script: {first}"));
                }
                self.result = Some(result);
                self.status = STATUS_FINISHED;
            }
        }
        self.status_line.take()
    }

    /// The link handed over, the requests answered or started, and the ones under way followed.
    /// What needs the telemetry to itself to start is kept for [`ScriptsTab::serve`]. `view` is
    /// the frame's, when there is one; otherwise one is taken if a request reads it, and only
    /// then.
    fn answer_requests(
        &mut self,
        telemetry: &Telemetry,
        view: Option<&TelemetryView>,
        guided: &mut GuidedMode,
    ) {
        let Some(asks) = &self.asks else {
            return;
        };
        // A script running: its output and requests looked for as the timer looked for them.
        crate::repaint::in_flight();
        // `MainV2.comPort` and `sysidcurrent` as they stand, for the script's thread.
        *self.link.os_lock().unwrap_or_else(PoisonError::into_inner) = telemetry.send_handle();
        let mut taken: Option<TelemetryView> = None;
        let mut started = Vec::new();
        while let Ok((request, reply)) = asks.try_recv() {
            match request {
                Request::SetParam {
                    target,
                    ref name,
                    value,
                    force,
                } => {
                    if !telemetry.is_open() {
                        let _ = reply.send(closed_port_set_param(
                            telemetry.parameter_of(target, name),
                            name,
                            value,
                            force,
                        ));
                        continue;
                    }
                    match telemetry.write_parameter_on(target, name, value, force) {
                        Some(id) => started.push(Waiting::SetParam {
                            id,
                            name: name.clone(),
                            reply,
                        }),
                        // No link, no table: "Trying to set Param that doesnt exist".
                        None => {
                            let _ = reply.send(Reply::Bool(false));
                        }
                    }
                    continue;
                }
                Request::Reboot { bootloader: false } => {
                    // `Telemetry::reboot`: `doCommand`'s two sends, not waited for, and the
                    // serial port looked at afterwards; false with the port closed, as
                    // `doCommand` is. `// C#: MAVLinkInterface.cs:2550-2586, 2690-2691`
                    let _ = reply.send(Reply::Bool(telemetry.is_open() && telemetry.reboot()));
                    continue;
                }
                Request::GetWp {
                    target,
                    index,
                    mission_type,
                } => {
                    match telemetry.get_wp(target, index, mission_type) {
                        Some(id) => started.push(Waiting::GetWp { id, reply }),
                        // No port: the C#'s asks go nowhere and end in its timeout.
                        None => {
                            let _ = reply.send(Reply::TimedOut(Timeout::on("getWP").0));
                        }
                    }
                    continue;
                }
                Request::SetWpTotal {
                    target,
                    total,
                    mission_type,
                } => {
                    match telemetry.set_wp_total(target, total, mission_type) {
                        Some(id) => started.push(Waiting::SetWpTotal { id, reply }),
                        None => {
                            let _ = reply.send(Reply::TimedOut(Timeout::on("setWPTotal").0));
                        }
                    }
                    continue;
                }
                Request::Guided(update) => {
                    update.apply(guided);
                    continue;
                }
                Request::CsFieldOf { target, ref name } => {
                    // `MAVlist[sysid, compid].cs`, from that vehicle's own state.
                    let state = telemetry.vehicle_state(target);
                    let value = cs_value(state.as_deref(), telemetry.is_open(), name);
                    let _ = reply.send(Reply::Field(value));
                    continue;
                }
                Request::SetModeOf { target, ref mode } => {
                    // `setMode(sysid, compid, modein)`: `translateMode` by that vehicle's own
                    // type, then `DO_SET_MODE` not waited for and `SET_MODE` twice, all to it.
                    // `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4614-4642`
                    let family = telemetry
                        .vehicle_state(target)
                        .and_then(|state| VehicleFamily::from_mav_type(state.vehicle_type));
                    for message in crate::fly::set_mode_messages(target, family, mode) {
                        telemetry.send(&message);
                    }
                    let _ = reply.send(Reply::Done);
                    continue;
                }
                Request::Command { .. }
                | Request::Reboot { .. }
                | Request::SetWp { .. }
                | Request::SetWpCurrent { .. } => {
                    self.to_start.push((request, reply));
                    continue;
                }
                _ => {}
            }
            let seen = if reads_the_view(&request) {
                Some(match view {
                    Some(view) => view,
                    None => &*taken.get_or_insert_with(|| {
                        self.views_taken += 1;
                        telemetry.view()
                    }),
                })
            } else {
                None
            };
            let answers = seen.map_or(
                Answers {
                    parameters: &[],
                    messages: &[],
                    state: None,
                    connected: telemetry.is_open(),
                },
                |view| Answers {
                    parameters: &view.parameters,
                    messages: &view.messages,
                    state: view.state.as_deref(),
                    connected: view.connected,
                },
            );
            let (answer, effect) = service(&request, &answers);
            match effect {
                Some(Effect::Write(name, value)) => {
                    match telemetry.write_parameter(&name, f64::from(value), false) {
                        Some(id) => started.push(Waiting::Write {
                            id,
                            reply: reply.clone(),
                        }),
                        None => {
                            let _ = reply.send(Reply::Bool(false));
                        }
                    }
                }
                Some(Effect::SetMode(number)) => telemetry.set_mode(number),
                Some(Effect::ClearMessages) => telemetry.clear_messages(),
                Some(Effect::Speak(text)) => self.spoken = Some(text),
                None => {}
            }
            if let Some(answer) = answer {
                let _ = reply.send(answer);
            }
        }
        self.waiting.extend(started);
        let mut accepted = 0;
        self.waiting
            .retain_mut(|waiting| match waiting.follow(telemetry) {
                Some((answer, update)) => {
                    if matches!(waiting, Waiting::SetWp { .. }) && answer == Reply::Result(0) {
                        accepted += 1;
                    }
                    if let Some(update) = update {
                        update.apply(guided);
                    }
                    let _ = waiting.reply().send(answer);
                    false
                }
                None => true,
            });
        self.wps_accepted += accepted;
    }

    /// Facts a UI test asserts on.
    pub fn record_facts(&self) {
        use crate::facts::record;
        record("fly.script.status", self.status);
        record("fly.script.selected", self.selected_text());
        record("fly.script.running", self.is_running());
        record("fly.script.redirect", self.redirect);
        // "none" while empty: a script cannot expect an empty value.
        record(
            "fly.script.console",
            if self.console.is_empty() {
                "none".to_owned()
            } else {
                self.console.replace('\n', " | ")
            },
        );
        record(
            "fly.script.result",
            match &self.result {
                None => "none".to_owned(),
                Some(Ok(())) => "ok".to_owned(),
                Some(Err(text)) => format!("error: {}", text.lines().last().unwrap_or("").trim()),
            },
        );
        // `MAV.setWP`s the vehicle accepted this run, and what `SpeakAsync` last had to say.
        record("fly.script.wps", self.wps_accepted);
        record(
            "fly.script.speech",
            self.spoken.as_deref().unwrap_or("none"),
        );
    }
}

/// A link request under way for a script, and who is waiting for its end.
#[derive(Debug)]
enum Waiting {
    /// `Script.ChangeParam`: true when accepted.
    Write { id: RequestId, reply: Sender<Reply> },
    /// `MAV.setParam`.
    SetParam {
        id: RequestId,
        name: String,
        reply: Sender<Reply>,
    },
    /// `MAV.doCommand`, `doARM`, `doReboot(true)`.
    Command { id: RequestId, reply: Sender<Reply> },
    /// `MAV.setWPTotal`.
    SetWpTotal { id: RequestId, reply: Sender<Reply> },
    /// `MAV.setWP`, and for a guided item (current 2) what it does to `GuidedMode` once the
    /// vehicle answers.
    SetWp {
        id: RequestId,
        guided: Option<GuidedUpdate>,
        reply: Sender<Reply>,
    },
    /// `MAV.setWPCurrent`.
    SetCurrent { id: RequestId, reply: Sender<Reply> },
    /// `MAV.getWP`.
    GetWp { id: RequestId, reply: Sender<Reply> },
}

impl Waiting {
    fn reply(&self) -> &Sender<Reply> {
        match self {
            Self::Write { reply, .. }
            | Self::SetParam { reply, .. }
            | Self::Command { reply, .. }
            | Self::SetWpTotal { reply, .. }
            | Self::SetWp { reply, .. }
            | Self::SetCurrent { reply, .. }
            | Self::GetWp { reply, .. } => reply,
        }
    }

    /// The answer once the request has ended, as the C#'s member returns or throws, and what it
    /// does to `GuidedMode`. A request the link no longer knows - the link closed and gone - is
    /// the member's timeout, and so is one still under way on a link whose port has closed,
    /// which nothing will end now: the C#'s loop sends into the closed port (`generatePacket`
    /// returns at once, `MAVLinkInterface.cs:1265-1268`), reads nothing (`readPacketAsync`'s
    /// empty read, `:4697`, `:4932`, `:4968-4969`) and throws its `TimeoutException` when its
    /// retries are spent - here at once.
    fn follow(&self, telemetry: &Telemetry) -> Option<(Reply, Option<GuidedUpdate>)> {
        let timed_out = |member: &str| Reply::TimedOut(Timeout::on(member).0);
        let (id, member) = match self {
            Self::Write { id, .. } => (*id, String::new()),
            Self::SetParam { id, name, .. } => (*id, format!("setParam {name}")),
            Self::Command { id, .. } => (*id, "doCommand".to_owned()),
            Self::SetWpTotal { id, .. } => (*id, "setWPTotal".to_owned()),
            Self::SetWp { id, .. } => (*id, "setWP".to_owned()),
            Self::SetCurrent { id, .. } => (*id, "setWPCurrent".to_owned()),
            Self::GetWp { id, .. } => (*id, "getWP".to_owned()),
        };
        let request = telemetry.request(id);
        let outcome = match request.as_ref().map(mp_link::requests::Request::outcome) {
            Some(Some(outcome)) => Some(outcome),
            // Under way, the port still open: not yet.
            Some(None) if telemetry.is_open() => return None,
            Some(None) | None => None,
        };
        let mut guided = None;
        let answer = match (self, outcome) {
            // `Script.ChangeParam`: as before, true only when the vehicle took it.
            (Self::Write { .. }, outcome) => {
                Reply::Bool(matches!(outcome, Some(RequestOutcome::Accepted { .. })))
            }
            (_, Some(RequestOutcome::TimedOut) | None) => timed_out(&member),
            // `getWPAsync`: the item the vehicle sent. `// C#: MAVLinkInterface.cs:3494-3550`
            (Self::GetWp { .. }, Some(RequestOutcome::Accepted { .. })) => request
                .as_ref()
                .and_then(mp_link::requests::Request::wp)
                .map_or_else(|| timed_out(&member), |read| Reply::Wp(locationwp(&read))),
            (Self::GetWp { .. }, Some(_)) => timed_out(&member),
            // `setWPTotalAsync` returns once the vehicle asks for the first item or acks.
            (Self::SetWpTotal { .. }, Some(_)) => Reply::Done,
            // `setWPAsync`: the `MAV_MISSION_RESULT`, accepted when the vehicle asked for the
            // next item; a guided item is `GuidedMode` either way. `// C#: MAVLinkInterface.cs:4099-4142`
            (Self::SetWp { guided: update, .. }, Some(outcome)) => {
                guided = *update;
                match outcome {
                    RequestOutcome::Rejected(result) => Reply::Result(result),
                    _ => Reply::Result(0),
                }
            }
            // `setParamAsync`: false for a name not listed, true for a value already held.
            // `// C#: MAVLinkInterface.cs:1637-1648`
            (_, Some(RequestOutcome::UnknownParameter | RequestOutcome::Rejected(_))) => {
                Reply::Bool(false)
            }
            (_, Some(_)) => Reply::Bool(true),
        };
        Some((answer, guided))
    }
}

/// The task that serves a run: [`ScriptsTab::serve`] every [`SERVE_INTERVAL`] until the run
/// has ended and nothing is left under way. Started by the Run button, beside the window's own
/// frames, so a script's requests are answered at `WaitFor`'s pace whatever the window draws.
pub fn serve_while_running(cx: &mut Context<MissionPlanner>) {
    cx.spawn(async move |this, cx| {
        loop {
            cx.background_executor().timer(SERVE_INTERVAL).await;
            let serving = this.update(cx, |this, _| {
                let telemetry = &mut this.telemetry;
                let guided = &mut this.fly_actions.guided;
                this.fly_data.scripts.serve(telemetry, guided)
            });
            if !matches!(serving, Ok(true)) {
                break;
            }
        }
    })
    .detach();
}

/// `Process.Start(new ProcessStartInfo(path) { UseShellExecute = true })`: the desktop's
/// association for the file. Under the harness, `MP_OPEN_WITH_SHELL_LOG` names a file the path is
/// appended to instead, so a test can see what would have opened without a browser or an editor
/// coming up on the test display.
pub(crate) fn open_with_shell(path: &Path) -> Result<(), String> {
    if let Some(log) = std::env::var_os("MP_OPEN_WITH_SHELL_LOG") {
        use std::io::Write as _;
        return mp_os::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(log)
            .and_then(|mut file| writeln!(file, "{}", path.display()))
            .map_err(|err| err.to_string());
    }
    let mut command = if cfg!(target_os = "windows") {
        let mut command = std::process::Command::new("cmd");
        command.args(["/C", "start", ""]).arg(path);
        command
    } else if cfg!(target_os = "macos") {
        let mut command = std::process::Command::new("open");
        command.arg(path);
        command
    } else {
        let mut command = std::process::Command::new("xdg-open");
        command.arg(path);
        command
    };
    command
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .map(|_| ())
        .map_err(|err| err.to_string())
}

/// The page: the labels, the box and the four buttons at the `.resx`'s places, and the console
/// under them while it shows.
/// `// C#: GCSViews/FlightData.Designer.cs:2016-2022; FlightData.resx; Controls/ScriptConsole.Designer.cs`
pub fn page(tab: &ScriptsTab, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let running = tab.is_running();
    let selected = tab.selected.is_some();
    let mut body = crate::probe::measured("fly-scripts", div())
        .relative()
        .w(px(PAGE_SIZE.0))
        .h(px(if tab.console_shown {
            PAGE_SIZE.1 + 230.0
        } else {
            PAGE_SIZE.1
        }))
        .child(label(
            STATUS_LABEL_AT.0,
            STATUS_LABEL_AT.1,
            tab.status,
            true,
        ))
        .child(label(
            SELECTED_LABEL_AT.0,
            SELECTED_LABEL_AT.1,
            tab.selected_text(),
            true,
        ));
    let mut redirect = Check::default();
    redirect.enabled = !running;
    redirect.state = if tab.redirect {
        CheckState::Checked
    } else {
        CheckState::Unchecked
    };
    body = body.child(check_box(
        "fly-script-redirect".to_owned(),
        &redirect,
        REDIRECT_TEXT,
        REDIRECT_AT,
        |this| {
            let tab = &mut this.fly_data.scripts;
            if !tab.is_running() {
                tab.redirect = !tab.redirect;
            }
        },
        cx,
    ));
    body = body.child(button(
        "fly-script-select",
        SELECT_TEXT,
        SELECT_AT,
        !running,
        |this, window, cx| {
            this.fly_actions.ask(crate::fly::Prompt::SelectScript, "");
            // The box takes the typed path: the field has the focus, as the other typed
            // questions give it (found by fly-scripts.gui, 2026-09-25).
            this.fly_focus.prompt.focus(window, cx);
        },
        cx,
    ));
    if selected {
        body = body
            .child(button(
                "fly-script-run",
                RUN_TEXT,
                RUN_AT,
                !running,
                |this, _window, cx| {
                    let telemetry = &this.telemetry;
                    this.fly_data
                        .scripts
                        .run_pressed(telemetry, &this.persisted);
                    if this.fly_data.scripts.is_running() {
                        serve_while_running(cx);
                    }
                },
                cx,
            ))
            .child(button(
                "fly-script-edit",
                EDIT_TEXT,
                EDIT_AT,
                !running,
                |this, _window, _cx| this.fly_data.scripts.edit_pressed(),
                cx,
            ));
    }
    if running {
        body = body.child(button(
            "fly-script-abort",
            ABORT_TEXT,
            ABORT_AT,
            true,
            |this, _window, _cx| this.fly_data.scripts.abort_pressed(),
            cx,
        ));
    }
    if tab.console_shown {
        body = body.child(console(tab, cx));
    }
    body.into_any_element()
}

/// `ScriptConsole`: the output, Clear, and autoscroll.
/// `// C#: Controls/ScriptConsole.cs; Controls/ScriptConsole.Designer.cs`
fn console(tab: &ScriptsTab, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let top = PAGE_SIZE.1 + 4.0;
    let mut text = crate::probe::measured(
        "fly-script-console",
        at(0.0, top + 26.0, PAGE_SIZE.0, 200.0),
    )
    .id("fly-script-console")
    .flex()
    .flex_col()
    .px_1()
    .overflow_y_scroll()
    .rounded_sm()
    .border_1()
    .border_color(rgb(theme::BORDER))
    .bg(rgb(theme::BG))
    .text_xs()
    .text_color(rgb(theme::TEXT));
    for line in tab.console.lines() {
        text = text.child(div().whitespace_nowrap().child(line.to_owned()));
    }
    let mut autoscroll = Check::default();
    autoscroll.enabled = true;
    autoscroll.state = if tab.autoscroll {
        CheckState::Checked
    } else {
        CheckState::Unchecked
    };
    at(0.0, top, PAGE_SIZE.0, 230.0)
        .child(button(
            "fly-script-console-clear",
            "Clear",
            (0.0, 0.0, 60.0, 22.0),
            true,
            |this, _window, _cx| this.fly_data.scripts.clear_console(),
            cx,
        ))
        .child(check_box(
            "fly-script-console-autoscroll".to_owned(),
            &autoscroll,
            "autoscroll",
            (70.0, 3.0),
            |this| {
                let tab = &mut this.fly_data.scripts;
                tab.autoscroll = !tab.autoscroll;
            },
            cx,
        ))
        .child(text)
        .into_any_element()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn message(text: &str) -> mp_link::messages::LogMessage {
        mp_link::messages::LogMessage {
            from: VehicleId::new(1, 1),
            severity: mp_link::messages::Severity::Info,
            text: text.to_owned(),
            seq: 1,
            received: 0,
        }
    }

    /// The window's answers: a parameter held or not, a message present or not, a mode by name
    /// through the vehicle's table, `cs` fields by the C#'s names.
    #[test]
    fn requests_are_answered_from_what_the_window_knows() {
        let parameters = vec![("RC3_MIN".to_owned(), 1100.0)];
        let messages = vec![message("ARMING MOTORS")];
        let mut state = VehicleState::default();
        state.vehicle_type = 2;
        state.position = mp_units::LatLon::new(-35.36, 149.16).ok();
        state.gps.satellites_visible = 9;
        state.armed = true;
        let answers = Answers {
            parameters: &parameters,
            messages: &messages,
            state: Some(&state),
            connected: true,
        };
        assert_eq!(
            service(&Request::GetParam("RC3_MIN".to_owned()), &answers),
            (Some(Reply::Param(Some(1100.0))), None)
        );
        // `cs.firmware`, which `setGuidedModeWP` reads, and `BaseStream.IsOpen`.
        assert_eq!(
            service(&Request::CsField("firmware".to_owned()), &answers),
            (
                Some(Reply::Field(Some(CsValue::Text("ArduCopter2".to_owned())))),
                None
            )
        );
        assert_eq!(
            service(&Request::IsOpen, &answers),
            (Some(Reply::Bool(true)), None)
        );
        assert_eq!(
            service(&Request::Speak("test 0".to_owned()), &answers),
            (Some(Reply::Done), Some(Effect::Speak("test 0".to_owned())))
        );
        assert_eq!(
            service(&Request::GetParam("NOPE".to_owned()), &answers),
            (Some(Reply::Param(None)), None)
        );
        assert_eq!(
            service(&Request::HasMessage("ARMING".to_owned()), &answers),
            (Some(Reply::Bool(true)), None)
        );
        assert_eq!(
            service(&Request::HasMessage("DISARM".to_owned()), &answers),
            (Some(Reply::Bool(false)), None)
        );
        assert_eq!(
            service(&Request::ChangeMode("auto".to_owned()), &answers),
            (Some(Reply::Bool(true)), Some(Effect::SetMode(3)))
        );
        assert_eq!(
            service(&Request::ChangeMode("Nonsense".to_owned()), &answers),
            (Some(Reply::Bool(true)), None)
        );
        assert_eq!(
            service(&Request::ChangeParam("X".to_owned(), 2.0), &answers),
            (None, Some(Effect::Write("X".to_owned(), 2.0)))
        );
        assert_eq!(
            service(&Request::ClearMessages, &answers),
            (Some(Reply::Done), Some(Effect::ClearMessages))
        );
        assert_eq!(
            service(&Request::CsField("lat".to_owned()), &answers),
            (Some(Reply::Field(Some(CsValue::Number(-35.36)))), None)
        );
        assert_eq!(
            service(&Request::CsField("satcount".to_owned()), &answers),
            (Some(Reply::Field(Some(CsValue::Number(9.0)))), None)
        );
        assert_eq!(
            service(&Request::CsField("armed".to_owned()), &answers),
            (Some(Reply::Field(Some(CsValue::Flag(true)))), None)
        );
        assert_eq!(
            service(&Request::CsField("nosuch".to_owned()), &answers),
            (Some(Reply::Field(None)), None)
        );
    }

    /// The tab's texts and states: nothing selected, then a script, the C#'s label texts, an
    /// empty answer clearing the selection.
    #[test]
    fn the_labels_follow_the_selection() {
        let mut tab = ScriptsTab::default();
        assert_eq!(tab.status, STATUS_NONE);
        assert_eq!(tab.selected_text(), SELECTED_NONE);
        tab.select("/tmp/example1.py");
        assert_eq!(tab.selected_text(), "Selected Script: /tmp/example1.py");
        tab.select("");
        assert_eq!(tab.selected_text(), SELECTED_NONE);
        assert!(!tab.is_running());
    }

    /// A script run from the tab with no link: its `print` reaches the console, `SendRC` with
    /// no link is false, the status runs and finishes, and the result is recorded; a script
    /// that throws puts "Error running script" in the console and its last line on the status
    /// line.
    #[test]
    fn a_run_from_the_tab_fills_the_console_and_ends() {
        let telemetry = Telemetry::idle();
        let view = TelemetryView::disconnected("");
        let mut guided = GuidedMode::default();
        let mut tab = ScriptsTab {
            redirect: true,
            ..Default::default()
        };
        tab.start(
            Path::new("/scripts/ok.py"),
            "print('hello')\nprint(Script.SendRC(3, 1500, True))\n".to_owned(),
            None,
            (false, false),
        );
        assert_eq!(tab.status, STATUS_RUNNING);
        assert!(tab.console_shown);
        let deadline = web_time::Instant::now() + Duration::from_secs(30);
        while tab.result.is_none() && web_time::Instant::now() < deadline {
            tab.tick(&telemetry, &view, &mut guided);
            wasm_thread::sleep(Duration::from_millis(10));
        }
        assert_eq!(tab.result, Some(Ok(())));
        assert_eq!(tab.status, STATUS_FINISHED);
        assert_eq!(tab.console, "hello\nFalse\n");

        let mut tab = ScriptsTab::default();
        tab.start(
            Path::new("bad.py"),
            "MAV.getParamList()\n".to_owned(),
            None,
            (false, false),
        );
        let deadline = web_time::Instant::now() + Duration::from_secs(30);
        let mut status = None;
        while tab.result.is_none() && web_time::Instant::now() < deadline {
            status = tab.tick(&telemetry, &view, &mut guided).or(status);
            wasm_thread::sleep(Duration::from_millis(10));
        }
        assert!(tab.result.as_ref().is_some_and(Result::is_err));
        assert!(
            tab.console.starts_with("Error running script "),
            "{}",
            tab.console
        );
        assert!(
            status
                .as_deref()
                .is_some_and(|s| s.contains("MAV.getParamList is not available")),
            "{status:?}"
        );
    }

    /// Serves and ticks `tab`'s run until it ends, as the Run button's task and the window's
    /// frames do, `vehicle` answering the link after each serve.
    fn serve_until_ended(
        tab: &mut ScriptsTab,
        telemetry: &mut Telemetry,
        guided: &mut GuidedMode,
        mut vehicle: impl FnMut(&mut ScriptsTab),
    ) {
        let deadline = web_time::Instant::now() + Duration::from_secs(60);
        while tab.result.is_none() {
            assert!(web_time::Instant::now() < deadline, "{}", tab.console);
            tab.serve(telemetry, guided);
            vehicle(tab);
            let view = telemetry.view();
            tab.tick(telemetry, &view, guided);
            wasm_thread::sleep(SERVE_INTERVAL);
        }
    }

    /// Runs `source` from the tab until it ends, started with the link as Run starts it.
    fn run_served(
        tab: &mut ScriptsTab,
        telemetry: &mut Telemetry,
        guided: &mut GuidedMode,
        source: &str,
        mut vehicle: impl FnMut(),
    ) {
        tab.start(
            Path::new("served.py"),
            source.to_owned(),
            telemetry.send_handle(),
            (false, false),
        );
        serve_until_ended(tab, telemetry, guided, |_| vehicle());
    }

    /// With no link, `MAV` does what `MAVLinkInterface` does with its port closed: a command is
    /// false, `setWPTotal` and `getWP` their timeouts, the port not open.
    #[test]
    fn with_no_link_mav_is_a_closed_port() {
        let mut telemetry = Telemetry::idle();
        let mut tab = ScriptsTab::default();
        run_served(
            &mut tab,
            &mut telemetry,
            &mut GuidedMode::default(),
            concat!(
                "import System\n",
                "print(MAV.BaseStream.IsOpen, MAV.doARM(True), MAV.sysidcurrent)\n",
                "for call in (lambda: MAV.setWPTotal(2), lambda: MAV.getWP(0)):\n",
                "    try:\n",
                "        call()\n",
                "    except System.TimeoutException as e:\n",
                "        print(e)\n",
            ),
            || {},
        );
        assert_eq!(tab.result, Some(Ok(())));
        assert_eq!(
            tab.console,
            "False False 0\nTimeout on read - setWPTotal\nTimeout on read - getWP\n"
        );
    }

    /// The shipped `example4 wp.py` through the tab and the real link to a scripted copter that
    /// takes a mission item by item as ArduPilot does - `MISSION_REQUEST` for the first item
    /// after `MISSION_COUNT`, then for each next one, `MISSION_ACK` after the last: `setWPTotal`
    /// waits for that first request, so every item reaches the vehicle once - item 0 not sent a
    /// second time for the request `setWPTotal` took (ArduPilot's INVALID_SEQUENCE) - in the frame
    /// the script passed, `setWPACK`'s ack after them, and the facts say five were accepted.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2431-2447, 3746-4224`
    #[test]
    fn example4_writes_its_mission_to_the_vehicle_item_by_item() {
        use mp_mavlink_dialects::all::{MissionAck, MissionRequest};

        use crate::telemetry::scripted::Vehicle;

        let (mut telemetry, vehicle) = Vehicle::connect(mp_link::ProtocolTimeouts::default());
        let vehicle = std::cell::RefCell::new(vehicle);
        let items = std::cell::RefCell::new(Vec::new());
        let gcs = |seq: u16| MissionRequest {
            seq,
            target_system: 255,
            target_component: 190,
            mission_type: 0,
        };
        let source = mp_os::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../testdata/scripts/example4 wp.py"
        ))
        .unwrap();
        let mut tab = ScriptsTab::default();
        run_served(
            &mut tab,
            &mut telemetry,
            &mut GuidedMode::default(),
            &source,
            || {
                let mut vehicle = vehicle.borrow_mut();
                for message in vehicle.read() {
                    match message {
                        MavMessage::MissionCount(count) => {
                            assert_eq!(count.count, 5);
                            vehicle.send(&MavMessage::MissionRequest(gcs(0)));
                        }
                        MavMessage::MissionItem(item) => {
                            items.borrow_mut().push(item);
                            if item.seq + 1 < 5 {
                                vehicle.send(&MavMessage::MissionRequest(gcs(item.seq + 1)));
                            } else {
                                vehicle.send(&MavMessage::MissionAck(MissionAck {
                                    target_system: 255,
                                    target_component: 190,
                                    r#type: 0,
                                    mission_type: 0,
                                }));
                            }
                        }
                        _ => {}
                    }
                }
            },
        );
        assert_eq!(tab.result, Some(Ok(())), "{}", tab.console);
        assert_eq!(
            tab.console,
            concat!(
                "set wp total\nupload home - reset on arm\nupload to\nupload wp1\n",
                "upload wp2\nupload wp3\nfinal ack\ndone\n"
            )
        );
        let summary: Vec<(u16, u16, u8, f32, f32, f32, f32)> = items
            .into_inner()
            .iter()
            .map(|item| {
                (
                    item.seq,
                    item.command,
                    item.frame,
                    item.param1,
                    item.x,
                    item.y,
                    item.z,
                )
            })
            .collect();
        // Each item once, in order: none sent again.
        assert_eq!(
            summary,
            [
                (0, 16, 3, 0.0, -34.9805, 117.8518, 0.0),
                (1, 22, 3, 15.0, 0.0, 0.0, 50.0),
                (2, 16, 3, 0.0, -35.0, 117.8, 50.0),
                (3, 16, 3, 0.0, -35.0, 117.89, 50.0),
                (4, 16, 3, 0.0, -35.0, 117.85, 20.0),
            ]
        );
        // `setWPACK` after the last item, and one `MISSION_COUNT`.
        let mut vehicle = vehicle.into_inner();
        crate::telemetry::scripted::until("the final ack", || {
            vehicle.read();
            vehicle.count(|message| matches!(message, MavMessage::MissionAck(_))) == 1
        });
        assert_eq!(
            vehicle.count(|message| matches!(message, MavMessage::MissionCount(_))),
            1
        );
        assert_eq!(tab.wps_accepted, 5);
    }

    /// `doARM` through the link's `doCommand`, answered; a refused command false; `getWP(0)` and
    /// a fence point `getWP(1, FENCE)` each read on its own with `MISSION_REQUEST` - the copter
    /// has not said it speaks `MISSION_INT` - and answered with the item, while an operator's
    /// Plan-screen Write is under way: the script's reads start no mission download and leave
    /// the Write where it is.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2619-2833, 3393-3558`
    #[test]
    fn commands_and_get_wp_go_through_the_link() {
        use mp_mavlink_dialects::all::MissionItem;

        use crate::telemetry::scripted::{Vehicle, ack};

        let (mut telemetry, vehicle) = Vehicle::connect(mp_link::ProtocolTimeouts::default());
        let vehicle = std::cell::RefCell::new(vehicle);
        // The operator's Write, which the copter has not answered yet.
        telemetry.upload_mission(vec![mp_mission::MissionItem::default(); 3]);
        let mut tab = ScriptsTab::default();
        run_served(
            &mut tab,
            &mut telemetry,
            &mut GuidedMode::default(),
            concat!(
                "import MAVLink\n",
                "print(MAV.doARM(True))\n",
                "print(MAV.doCommand(MAVLink.MAV_CMD.TAKEOFF, 0, 0, 0, 0, 0, 0, 100))\n",
                "home = MAV.getWP(0)\n",
                "print(home.id, round(home.lat, 6), round(home.lng, 6), home.alt, home.frame)\n",
                "fence = MAV.getWP(1, MAVLink.MAV_MISSION_TYPE.FENCE)\n",
                "print(fence.id, round(fence.lat, 4), fence.p1)\n",
            ),
            || {
                let mut vehicle = vehicle.borrow_mut();
                for message in vehicle.read() {
                    match message {
                        MavMessage::CommandLong(long) => {
                            // Arming accepted, the take-off refused (MAV_RESULT_DENIED).
                            let result = if long.command == 400 { 0 } else { 2 };
                            vehicle.send(&ack(long.command, result));
                        }
                        MavMessage::MissionRequest(request) => {
                            let fence = request.mission_type == 1;
                            vehicle.send(&MavMessage::MissionItem(MissionItem {
                                param1: if fence { 4.0 } else { 0.0 },
                                param2: 0.0,
                                param3: 0.0,
                                param4: 0.0,
                                x: if fence { -35.36 } else { -35.363_262 },
                                y: 149.165_24,
                                z: 584.0,
                                seq: request.seq,
                                command: if fence { 5001 } else { 16 },
                                target_system: 255,
                                target_component: 190,
                                frame: 0,
                                current: 0,
                                autocontinue: 1,
                                mission_type: request.mission_type,
                            }));
                        }
                        _ => {}
                    }
                }
            },
        );
        assert_eq!(tab.result, Some(Ok(())), "{}", tab.console);
        assert_eq!(
            tab.console,
            "True\nFalse\n16 -35.363262 149.165237 584.0 0\n5001 -35.36 4.0\n"
        );
        let vehicle = vehicle.into_inner();
        let asked: Vec<(u16, u8)> = vehicle
            .heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::MissionRequest(request) => Some((request.seq, request.mission_type)),
                _ => None,
            })
            .collect();
        assert_eq!(asked, [(0, 0), (1, 1)]);
        assert_eq!(
            vehicle.count(|message| matches!(message, MavMessage::MissionRequestList(_))),
            0
        );
        let transfer = telemetry.view().transfer.expect("the operator's Write");
        assert!(
            transfer.label.starts_with("writing item")
                || transfer.label.starts_with("transfer failed"),
            "{}",
            transfer.label
        );
    }

    /// `getWP(sysid, compid, index)` asks the vehicle the script names, and only its answer
    /// counts: the copter answering for another is read past, and the read times out.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3403-3407, 3480-3486`
    #[test]
    fn get_wp_asks_the_vehicle_the_script_names() {
        use mp_mavlink_dialects::all::MissionItem;

        use crate::telemetry::scripted::Vehicle;

        let timeouts = mp_link::ProtocolTimeouts::default().faster(50);
        let (mut telemetry, vehicle) = Vehicle::connect(timeouts);
        let vehicle = std::cell::RefCell::new(vehicle);
        let mut tab = ScriptsTab::default();
        run_served(
            &mut tab,
            &mut telemetry,
            &mut GuidedMode::default(),
            concat!(
                "import System\n",
                "try:\n",
                "    MAV.getWP(2, 1, 0)\n",
                "except System.TimeoutException as e:\n",
                "    print(e)\n",
            ),
            || {
                let mut vehicle = vehicle.borrow_mut();
                for message in vehicle.read() {
                    if let MavMessage::MissionRequest(request) = message {
                        vehicle.send(&MavMessage::MissionItem(MissionItem {
                            param1: 0.0,
                            param2: 0.0,
                            param3: 0.0,
                            param4: 0.0,
                            x: -35.0,
                            y: 149.0,
                            z: 10.0,
                            seq: request.seq,
                            command: 16,
                            target_system: 255,
                            target_component: 190,
                            frame: 0,
                            current: 0,
                            autocontinue: 1,
                            mission_type: 0,
                        }));
                    }
                }
            },
        );
        assert_eq!(tab.console, "Timeout on read - getWP\n");
        let mut vehicle = vehicle.into_inner();
        vehicle.read();
        let targets: Vec<(u8, u8)> = vehicle
            .heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::MissionRequest(request) => {
                    Some((request.target_system, request.target_component))
                }
                _ => None,
            })
            .collect();
        assert_eq!(
            targets,
            vec![(2, 1); usize::from(timeouts.mission_item_request.sends())]
        );
    }

    /// The link stopping - the cable pulled - while a script waits in `setParam` ends the wait
    /// in its `TimeoutException`, at once; `doARM` afterwards is false, as `doCommand` is with
    /// the port closed, and the port is not open.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1265-1268, 1757-1763, 2690-2691`
    #[test]
    fn a_link_that_stops_ends_the_scripts_wait() {
        use crate::telemetry::scripted::{Vehicle, param};

        let (mut telemetry, vehicle) = Vehicle::connect(mp_link::ProtocolTimeouts::default());
        let vehicle = std::cell::RefCell::new(vehicle);
        vehicle.borrow_mut().send(&param("RTL_ALT", 1500.0, 6));
        crate::telemetry::scripted::until("RTL_ALT held", || telemetry.holds_parameter("RTL_ALT"));
        let mut tab = ScriptsTab::default();
        let started = Instant::now();
        run_served(
            &mut tab,
            &mut telemetry,
            &mut GuidedMode::default(),
            concat!(
                "import System\n",
                "try:\n",
                "    MAV.setParam('RTL_ALT', 2000)\n",
                "except System.TimeoutException as e:\n",
                "    print(e)\n",
                "print(MAV.doARM(True), MAV.BaseStream.IsOpen)\n",
            ),
            || {
                let mut vehicle = vehicle.borrow_mut();
                if vehicle
                    .read()
                    .iter()
                    .any(|message| matches!(message, MavMessage::ParamSet(_)))
                {
                    vehicle.unplug();
                }
            },
        );
        assert_eq!(tab.result, Some(Ok(())), "{}", tab.console);
        assert_eq!(
            tab.console,
            "Timeout on read - setParam RTL_ALT\nFalse False\n"
        );
        // Not the C#'s four 700 ms waits, nor the tab's own five minutes.
        assert!(
            started.elapsed() < Duration::from_secs(2),
            "{:?}",
            started.elapsed()
        );
    }

    /// The Abort button ends a script waiting on the link: the vehicle never answers its
    /// `getWP`, Abort is pressed, and the script stops there - nothing after it runs.
    /// `// C#: GCSViews/FlightData.cs:1014-1019`
    #[test]
    fn abort_ends_a_script_waiting_on_the_link() {
        use crate::telemetry::scripted::Vehicle;

        let (mut telemetry, mut vehicle) = Vehicle::connect(mp_link::ProtocolTimeouts::default());
        let mut tab = ScriptsTab::default();
        tab.start(
            Path::new("wait.py"),
            "MAV.getWP(0)\nprint('after')\n".to_owned(),
            telemetry.send_handle(),
            (false, false),
        );
        let mut pressed = None;
        serve_until_ended(
            &mut tab,
            &mut telemetry,
            &mut GuidedMode::default(),
            |tab| {
                let asked = vehicle
                    .read()
                    .iter()
                    .any(|message| matches!(message, MavMessage::MissionRequest(_)));
                if asked && pressed.is_none() {
                    tab.abort_pressed();
                    pressed = Some(Instant::now());
                }
            },
        );
        assert_eq!(tab.result, Some(Ok(())));
        assert_eq!(tab.console, "");
        let pressed = pressed.expect("the getWP asked");
        assert!(
            pressed.elapsed() < Duration::from_secs(2),
            "{:?}",
            pressed.elapsed()
        );
    }

    /// `MAV` follows the window's link as it stands, not as it stood at Run: a script started
    /// before the vehicle was handed over finds it at its next call, and its command reaches it.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:289-315`
    #[test]
    fn mav_follows_the_link_as_it_stands() {
        use crate::telemetry::scripted::{Vehicle, ack};

        let (mut telemetry, vehicle) = Vehicle::connect(mp_link::ProtocolTimeouts::default());
        let vehicle = std::cell::RefCell::new(vehicle);
        let mut tab = ScriptsTab::default();
        // Run pressed before any vehicle was heard: no link handed over.
        tab.start(
            Path::new("late.py"),
            "Script.Sleep(50)\nprint(MAV.sysidcurrent, MAV.doARM(True))\n".to_owned(),
            None,
            (false, false),
        );
        serve_until_ended(&mut tab, &mut telemetry, &mut GuidedMode::default(), |_| {
            let mut vehicle = vehicle.borrow_mut();
            for message in vehicle.read() {
                if let MavMessage::CommandLong(long) = message {
                    vehicle.send(&ack(long.command, 0));
                }
            }
        });
        assert_eq!(tab.result, Some(Ok(())), "{}", tab.console);
        assert_eq!(tab.console, "1 True\n");
    }

    /// `setGuidedModeWP` on a copter: the position target sent, and the flight screen's
    /// `GuidedMode` given its position and height as `setPositionTargetGlobalInt` gives them,
    /// its frame left as it was.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4423-4461, 4527-4534`
    #[test]
    fn set_guided_mode_wp_moves_the_flight_screens_guided_mode() {
        use crate::telemetry::scripted::Vehicle;

        let (mut telemetry, vehicle) = Vehicle::connect(mp_link::ProtocolTimeouts::default());
        let vehicle = std::cell::RefCell::new(vehicle);
        let mut guided = GuidedMode {
            frame: 10,
            ..GuidedMode::default()
        };
        let mut tab = ScriptsTab::default();
        run_served(
            &mut tab,
            &mut telemetry,
            &mut guided,
            concat!(
                "from MissionPlanner.Utilities import Locationwp\n",
                "MAV.setGuidedModeWP(Locationwp().Set(-35.36, 149.16, 60, 16))\n",
            ),
            || {
                vehicle.borrow_mut().read();
            },
        );
        assert_eq!(tab.result, Some(Ok(())), "{}", tab.console);
        assert_eq!(
            guided,
            GuidedMode {
                x: -353_600_000,
                y: 1_491_600_000,
                z: 60.0,
                frame: 10,
            }
        );
        let mut vehicle = vehicle.into_inner();
        crate::telemetry::scripted::until("the position target", || {
            vehicle.read();
            vehicle.count(|message| matches!(message, MavMessage::SetPositionTargetGlobalInt(_)))
                == 1
        });
    }

    /// A guided `setWP` (current 2) sets the whole `GuidedMode` from the item, through
    /// `Locationwp`: the position times 1e7 for a location command, as it came for another; a
    /// position target keeps what it does not set.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4100-4103, 4527-4534; ExtLibs/Utilities/locationwp.cs:153-178`
    #[test]
    fn guided_updates_are_the_c_sharp_ones() {
        let item = WpItem {
            seq: 0,
            frame: 3,
            command: 16,
            current: 2,
            autocontinue: 1,
            params: [0.0; 4],
            x: -35.5,
            y: 149.25,
            z: 80.0,
            mission_type: 0,
        };
        let mut guided = GuidedMode::default();
        GuidedUpdate::guided_item(&item).apply(&mut guided);
        assert_eq!(
            guided,
            GuidedMode {
                x: -355_000_000,
                y: 1_492_500_000,
                z: 80.0,
                frame: 3,
            }
        );
        // `DO_SET_SERVO` is no location: its numbers as they came.
        GuidedUpdate::guided_item(&WpItem {
            command: 183,
            x: 5.0,
            ..item
        })
        .apply(&mut guided);
        assert_eq!((guided.x, guided.y), (5, 149));
        // A zero latitude leaves `x`, and the frame is not a position target's.
        GuidedUpdate::position_target(&PositionTarget {
            frame: 0,
            lat: 0.0,
            lng: 150.0,
            alt: 12.0,
        })
        .apply(&mut guided);
        assert_eq!(
            guided,
            GuidedMode {
                x: 5,
                y: 1_500_000_000,
                z: 12.0,
                frame: 3,
            }
        );
    }

    /// Serving a script that asks nothing takes no view of the telemetry, however long it
    /// runs; one that asks what a view holds - `WaitFor` - takes one a poll.
    #[test]
    fn serving_a_script_that_asks_nothing_takes_no_view() {
        use crate::telemetry::scripted::Vehicle;

        let (mut telemetry, _vehicle) = Vehicle::connect(mp_link::ProtocolTimeouts::default());
        let mut guided = GuidedMode::default();
        let mut tab = ScriptsTab::default();
        tab.start(
            Path::new("idle.py"),
            "Script.Sleep(300)\n".to_owned(),
            telemetry.send_handle(),
            (false, false),
        );
        let deadline = Instant::now() + Duration::from_secs(30);
        let mut serves = 0;
        while tab.is_running() {
            assert!(Instant::now() < deadline);
            tab.serve(&mut telemetry, &mut guided);
            serves += 1;
            wasm_thread::sleep(SERVE_INTERVAL);
        }
        assert!(serves > 10, "{serves}");
        assert_eq!(tab.views_taken, 0);

        tab.start(
            Path::new("wait.py"),
            "print(Script.WaitFor('never', 100))\n".to_owned(),
            telemetry.send_handle(),
            (false, false),
        );
        while tab.is_running() {
            assert!(Instant::now() < deadline);
            tab.serve(&mut telemetry, &mut guided);
            wasm_thread::sleep(SERVE_INTERVAL);
        }
        assert!(tab.views_taken > 0);
    }

    /// Speech starts as the user's settings have it, handed over at Run.
    #[test]
    fn speech_starts_from_the_settings_handed_over_at_run() {
        let telemetry = Telemetry::idle();
        let view = TelemetryView::disconnected("");
        let mut tab = ScriptsTab::default();
        tab.start(
            Path::new("speak.py"),
            "print(MainV2.speechEnable)\nMainV2.speechEngine.SpeakAsync('hello')\n".to_owned(),
            None,
            (true, false),
        );
        let deadline = Instant::now() + Duration::from_secs(30);
        while tab.result.is_none() && Instant::now() < deadline {
            tab.tick(&telemetry, &view, &mut GuidedMode::default());
            wasm_thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(tab.result, Some(Ok(())));
        assert_eq!(tab.console, "True\n");
        assert_eq!(tab.spoken.as_deref(), Some("hello"));
    }

    /// A copter's `HEARTBEAT` in a custom mode, of `mav_type`.
    fn heartbeat(mav_type: u8, custom_mode: u32) -> MavMessage {
        MavMessage::Heartbeat(mp_mavlink_dialects::all::Heartbeat {
            custom_mode,
            r#type: mav_type,
            autopilot: 3,
            base_mode: 81,
            system_status: 3,
            mavlink_version: 3,
        })
    }

    /// The `SET_MODE`s and `DO_SET_MODE`s a vehicle heard: (target system, custom mode).
    fn modes_heard(heard: &[MavMessage]) -> Vec<(u8, u32)> {
        heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::SetMode(mode) => Some((mode.target_system, mode.custom_mode)),
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                MavMessage::CommandLong(long) if long.command == 176 => {
                    Some((long.target_system, long.param2 as u32))
                }
                _ => None,
            })
            .collect()
    }

    /// `setGuidedModeWP(sysid, compid, ...)` reads the named vehicle's mode and puts that
    /// vehicle in Guided - `DO_SET_MODE` and `SET_MODE` twice, all to it - and sends it the
    /// target; the vehicle shown, in Auto, is not touched.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4423-4461, 4614-4642`
    #[test]
    fn set_guided_mode_wp_on_a_named_vehicle_changes_that_vehicles_mode() {
        use crate::telemetry::scripted::{VEHICLE, Vehicle};

        const SECOND: VehicleId = VehicleId::new(2, 1);
        let (mut telemetry, vehicle) = Vehicle::connect(mp_link::ProtocolTimeouts::default());
        let vehicle = std::cell::RefCell::new(vehicle);
        // The shown vehicle in Auto, a second copter in Stabilize.
        vehicle.borrow_mut().send(&heartbeat(2, 3));
        vehicle.borrow_mut().send_from(SECOND, &heartbeat(2, 0));
        crate::telemetry::scripted::until("both vehicles", || {
            telemetry.vehicle_state(SECOND).is_some()
                && telemetry
                    .vehicle_state(VEHICLE)
                    .is_some_and(|state| state.custom_mode == 3)
        });
        assert_eq!(telemetry.view().vehicle, Some(VEHICLE));
        let mut tab = ScriptsTab::default();
        run_served(
            &mut tab,
            &mut telemetry,
            &mut GuidedMode::default(),
            concat!(
                "from MissionPlanner.Utilities import Locationwp\n",
                "MAV.setGuidedModeWP(2, 1, Locationwp().Set(-35.36, 149.16, 60, 16))\n",
            ),
            || {
                vehicle.borrow_mut().read();
            },
        );
        assert_eq!(tab.result, Some(Ok(())), "{}", tab.console);
        let mut vehicle = vehicle.into_inner();
        crate::telemetry::scripted::until("the target", || {
            vehicle.read();
            vehicle.count(|message| matches!(message, MavMessage::SetPositionTargetGlobalInt(_)))
                == 1
        });
        // Copter's GUIDED is 4: to the second vehicle, three times; nothing to the first.
        assert_eq!(modes_heard(&vehicle.heard), [(2, 4), (2, 4), (2, 4)]);
        let target = vehicle.heard.iter().find_map(|message| match message {
            MavMessage::SetPositionTargetGlobalInt(target) => Some(target.target_system),
            _ => None,
        });
        assert_eq!(target, Some(2));
    }

    /// A guided `setWP` (current 2) from a script, answered with an ack, sets the flight
    /// screen's `GuidedMode` to its item - a refusal too, as the C# sets it before it returns
    /// the result - and `setGuidedModeWP` on a plane goes that way: Guided (15) asked of it,
    /// then the item with current 2.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4098-4112, 4122-4141, 4430-4438`
    #[test]
    fn a_guided_set_wp_moves_the_flight_screens_guided_mode() {
        use mp_mavlink_dialects::all::MissionAck;

        use crate::telemetry::scripted::{VEHICLE, Vehicle};

        let answer = |vehicle: &std::cell::RefCell<Vehicle>, results: &mut Vec<u8>| {
            let mut vehicle = vehicle.borrow_mut();
            for message in vehicle.read() {
                if let MavMessage::MissionItem(item) = message {
                    assert_eq!(item.current, 2);
                    let result = if results.is_empty() {
                        0
                    } else {
                        results.remove(0)
                    };
                    vehicle.send(&MavMessage::MissionAck(MissionAck {
                        target_system: 255,
                        target_component: 190,
                        r#type: result,
                        mission_type: 0,
                    }));
                }
            }
        };

        // A copter: `setWP(loc, 0, frame, 2)` straight from the script, accepted, then refused.
        let (mut telemetry, vehicle) = Vehicle::connect(mp_link::ProtocolTimeouts::default());
        let vehicle = std::cell::RefCell::new(vehicle);
        let mut results = vec![0, 1];
        let mut guided = GuidedMode::default();
        let mut tab = ScriptsTab::default();
        run_served(
            &mut tab,
            &mut telemetry,
            &mut guided,
            concat!(
                "import MAVLink\n",
                "from MissionPlanner.Utilities import Locationwp\n",
                "frame = MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT\n",
                "print(MAV.setWP(Locationwp().Set(-35.5, 149.25, 80, 16), 0, frame, 2))\n",
                "print(MAV.setWP(Locationwp().Set(-35.25, 149.5, 90, 16), 0, frame, 2))\n",
            ),
            || answer(&vehicle, &mut results),
        );
        assert_eq!(tab.result, Some(Ok(())), "{}", tab.console);
        assert_eq!(tab.console, "MAV_MISSION_ACCEPTED\nMAV_MISSION_ERROR\n");
        assert_eq!(
            guided,
            GuidedMode {
                x: -352_500_000,
                y: 1_495_000_000,
                z: 90.0,
                frame: 3,
            }
        );

        // A plane, in Manual: `setGuidedModeWP` puts it in Guided and sends the item.
        let (mut telemetry, vehicle) = Vehicle::connect(mp_link::ProtocolTimeouts::default());
        let vehicle = std::cell::RefCell::new(vehicle);
        vehicle.borrow_mut().send(&heartbeat(1, 0));
        crate::telemetry::scripted::until("a plane", || {
            telemetry
                .vehicle_state(VEHICLE)
                .is_some_and(|state| state.vehicle_type == 1)
        });
        let mut results = Vec::new();
        let mut guided = GuidedMode::default();
        let mut tab = ScriptsTab::default();
        run_served(
            &mut tab,
            &mut telemetry,
            &mut guided,
            concat!(
                "from MissionPlanner.Utilities import Locationwp\n",
                "MAV.setGuidedModeWP(Locationwp().Set(-35.5, 149.25, 80, 16))\n",
            ),
            || answer(&vehicle, &mut results),
        );
        assert_eq!(tab.result, Some(Ok(())), "{}", tab.console);
        assert_eq!(
            guided,
            GuidedMode {
                x: -355_000_000,
                y: 1_492_500_000,
                z: 80.0,
                frame: 3,
            }
        );
        assert_eq!(
            modes_heard(&vehicle.into_inner().heard),
            [(1, 15), (1, 15), (1, 15)]
        );
    }

    /// With the port closed, `setParam` decides as `setParamAsync` does before it sends, from
    /// the named vehicle's parameters: false for one it has not listed, true for a value it
    /// holds already and not forced, and otherwise the timeout of its sends into the closed
    /// port.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1637-1648, 1757-1763`
    #[test]
    fn set_param_with_the_port_closed_decides_as_the_c_sharp_does() {
        use crate::telemetry::scripted::{VEHICLE, Vehicle, param};

        const SECOND: VehicleId = VehicleId::new(2, 1);
        let (mut telemetry, mut vehicle) = Vehicle::connect(mp_link::ProtocolTimeouts::default());
        vehicle.send(&param("RTL_ALT", 1500.0, 6));
        vehicle.send_from(SECOND, &heartbeat(2, 0));
        vehicle.send_from(SECOND, &param("Q_ONLY", 5.0, 9));
        crate::telemetry::scripted::until("both tables", || {
            telemetry.parameter_of(VEHICLE, "RTL_ALT").is_some()
                && telemetry.parameter_of(SECOND, "Q_ONLY").is_some()
        });
        vehicle.unplug();
        crate::telemetry::scripted::until("the port closed", || !telemetry.is_open());
        let mut tab = ScriptsTab::default();
        run_served(
            &mut tab,
            &mut telemetry,
            &mut GuidedMode::default(),
            concat!(
                "import System\n",
                "print(MAV.setParam(2, 1, 'Q_ONLY', 5))\n",
                "print(MAV.setParam(1, 1, 'Q_ONLY', 5))\n",
                "print(MAV.setParam('RTL_ALT', 1500))\n",
                "for call in (lambda: MAV.setParam(2, 1, 'Q_ONLY', 5, True),\n",
                "             lambda: MAV.setParam('RTL_ALT', 2000)):\n",
                "    try:\n",
                "        call()\n",
                "    except System.TimeoutException as e:\n",
                "        print(e)\n",
            ),
            || {},
        );
        assert_eq!(tab.result, Some(Ok(())), "{}", tab.console);
        assert_eq!(
            tab.console,
            concat!(
                "True\nFalse\nTrue\n",
                "Timeout on read - setParam Q_ONLY\nTimeout on read - setParam RTL_ALT\n"
            )
        );
    }

    /// Run reads `MainV2`'s speech from the settings it is read from: "speechenable" and
    /// "speech_armed_only" as the user saved them, both off when absent.
    /// `// C#: MainV2.cs:658, 1007-1008`
    #[test]
    fn run_reads_the_speech_settings() {
        let path = mp_os::temp_dir().join(format!("mp-scripts-speech-{}.py", mp_os::process_id()));
        mp_os::fs::write(
            &path,
            "print(MainV2.speechEnable, MainV2.speech_armed_only)\n",
        )
        .unwrap();
        let telemetry = Telemetry::idle();
        let view = TelemetryView::disconnected("");
        let mut saved = crate::settings::Persisted::at(None);
        saved.set("speechenable", "True");
        saved.set("speech_armed_only", " true ");
        for (settings, printed) in [
            (saved, "True True\n"),
            (crate::settings::Persisted::at(None), "False False\n"),
        ] {
            let mut tab = ScriptsTab::default();
            tab.select(&path.display().to_string());
            tab.run_pressed(&telemetry, &settings);
            let deadline = Instant::now() + Duration::from_secs(30);
            while tab.result.is_none() && Instant::now() < deadline {
                tab.tick(&telemetry, &view, &mut GuidedMode::default());
                wasm_thread::sleep(Duration::from_millis(5));
            }
            assert_eq!(tab.result, Some(Ok(())));
            assert_eq!(tab.console, printed);
        }
        let _ = mp_os::fs::remove_file(&path);
    }

    /// The vehicle's mission, read as the Plan screen's Read reads it.
    fn read_back(telemetry: &Telemetry) -> Vec<mp_mission::MissionItem> {
        // A finished transfer is taken to be this one once it has been seen under way, or half a
        // second on: a short mission can be read between two looks.
        const PICKUP: Duration = Duration::from_millis(500);
        telemetry.request_mission();
        let asked = web_time::Instant::now();
        let mut seen_running = false;
        loop {
            let view = telemetry.view();
            if let Some(transfer) = &view.transfer {
                seen_running |= !transfer.finished;
                if transfer.finished && (seen_running || asked.elapsed() > PICKUP) {
                    assert!(!transfer.failed, "{}", transfer.label);
                    return view.mission;
                }
            }
            assert!(asked.elapsed() < Duration::from_secs(30), "no mission read");
            wasm_thread::sleep(Duration::from_millis(20));
        }
    }

    /// Against ArduCopter SITL: the shipped `example4 wp.py` and `TAKEOFF.py` run from the tab
    /// through `GuiScriptHost` and the application's link, and what they wrote is on the
    /// vehicle when its mission is read back. example4 ends with "done"; TAKEOFF.py reads home
    /// with `getWP(0)`, writes its six items and then stops at `setWPCurrent(1)`, IronPython's
    /// TypeError, as it does under Mission Planner - its mission is on the vehicle all the same.
    /// Run with `--ignored` while SITL is up and settled - TAKEOFF.py checks the GPS first and
    /// prints "GPS PASSED." only with a fix, which a SITL seconds old has not got (passed
    /// 2026-09-26 on one given 45 s: all six items written, `getWP(0)` answered during the
    /// upload); it replaces the vehicle's mission and nothing else.
    #[test]
    #[ignore = "needs ArduCopter SITL on tcp:127.0.0.1:5760"]
    fn sitl_example4_and_takeoff_write_their_missions() {
        let link = mp_link::Link::connect("tcp:127.0.0.1:5760", mp_link::LinkConfig::default())
            .expect("SITL on 5760");
        let mut telemetry = Telemetry::over(link, "tcp:127.0.0.1:5760");
        let deadline = web_time::Instant::now() + Duration::from_secs(20);
        while telemetry.send_handle().is_none() || telemetry.view().state.is_none() {
            assert!(web_time::Instant::now() < deadline, "no vehicle on 5760");
            wasm_thread::sleep(Duration::from_millis(50));
        }
        let script = |name: &str| {
            mp_os::fs::read_to_string(format!(
                "{}/../../testdata/scripts/{name}",
                env!("CARGO_MANIFEST_DIR")
            ))
            .unwrap()
        };
        let mut guided = GuidedMode::default();

        let mut tab = ScriptsTab::default();
        run_served(
            &mut tab,
            &mut telemetry,
            &mut guided,
            &script("example4 wp.py"),
            || {},
        );
        eprintln!("example4: {:?}\n{}", tab.result, tab.console);
        assert_eq!(tab.result, Some(Ok(())));
        assert!(
            tab.console.ends_with("final ack\ndone\n"),
            "{}",
            tab.console
        );
        assert_eq!(tab.wps_accepted, 5);
        let mission = read_back(&telemetry);
        eprintln!("read back: {mission:#?}");
        assert_eq!(mission.len(), 5);
        let takeoff = &mission[1];
        assert_eq!(
            (takeoff.command, takeoff.param1, takeoff.z),
            (22, 15.0, 50.0)
        );
        for (item, (lat, lng, alt)) in mission[2..].iter().zip([
            (-35.0, 117.8, 50.0),
            (-35.0, 117.89, 50.0),
            (-35.0, 117.85, 20.0),
        ]) {
            assert_eq!(item.command, 16);
            assert_eq!(item.frame, 3);
            assert!(
                (item.x - lat).abs() < 1e-5 && (item.y - lng).abs() < 1e-5,
                "{item:?}"
            );
            assert!((item.z - alt).abs() < 1e-3, "{item:?}");
        }

        let mut tab = ScriptsTab::default();
        run_served(
            &mut tab,
            &mut telemetry,
            &mut guided,
            &script("TAKEOFF.py"),
            || {},
        );
        eprintln!("TAKEOFF: {:?}\n{}", tab.result, tab.console);
        assert!(tab.console.starts_with("GPS PASSED."), "{}", tab.console);
        let error = tab
            .result
            .clone()
            .expect("ended")
            .expect_err("the TypeError");
        assert!(
            error.contains("TypeError: setWPCurrent() takes exactly 3 arguments (1 given)"),
            "{error}"
        );
        assert_eq!(tab.wps_accepted, 6);
        let mission = read_back(&telemetry);
        eprintln!("read back: {mission:#?}");
        assert_eq!(mission.len(), 6);
        assert_eq!(
            (mission[1].command, mission[1].param1, mission[1].z),
            (22, 10.0, 100.0)
        );
        assert!((mission[5].z - 200.0).abs() < 1e-3, "{:?}", mission[5]);
    }

    /// A script's `GetParam` and `WaitFor` are answered by the window's frames from the
    /// telemetry it holds: here an idle telemetry, so no parameter (0.0) and no message.
    #[test]
    fn requests_from_the_scripts_thread_are_answered_each_frame() {
        let telemetry = Telemetry::idle();
        let view = TelemetryView::disconnected("");
        let mut tab = ScriptsTab::default();
        tab.start(
            Path::new("ask.py"),
            "print(Script.GetParam('RC3_MIN'))\nprint(Script.WaitFor('ARMING', 20))\nprint(cs.lat)\n".to_owned(),
            None,
            (false, false),
        );
        let deadline = web_time::Instant::now() + Duration::from_secs(30);
        while tab.result.is_none() && web_time::Instant::now() < deadline {
            tab.tick(&telemetry, &view, &mut GuidedMode::default());
            wasm_thread::sleep(Duration::from_millis(5));
        }
        assert!(tab.console.starts_with("0.0\nFalse\n"), "{}", tab.console);
        // `cs.lat` with no vehicle: the field is not there, an AttributeError.
        assert!(tab.result.as_ref().is_some_and(Result::is_err));
        assert!(
            tab.console.contains("cs has no field 'lat'"),
            "{}",
            tab.console
        );
    }
}
