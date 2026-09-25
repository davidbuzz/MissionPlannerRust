//! The flight screen's Scripts tab: `tabScripts` of `FlightData`, ported from
//! `GCSViews/FlightData.cs:786-812, 1012-1017, 1223-1233, 1630-1641, 4727-4776` and
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
//! sends [`Request`]s over a channel that [`ScriptsTab::tick`] answers once a frame from the
//! window's telemetry: `GetParam` from the parameters held, `ChangeParam` as a write the frame
//! follows to its end, `ChangeMode` through the mode table, `cs` fields from the vehicle's state,
//! `WaitFor` from the messages; `SendRC` goes straight out through the link's sender, as the
//! C#'s does from its thread. An error a script ends in goes to the console and the status line,
//! not a box (the owner's ruling on avoidable error boxes; the C# shows "Error running script"
//! in a box as well as in the writer).

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::path::{Path, PathBuf};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::time::Duration;

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_link::RequestId;
use mp_link::requests::RequestOutcome;
use mp_script::api::RC_RESEND_GAP_MS;
use mp_script::{CsValue, ScriptApi, ScriptHost, ScriptRun};
use mp_vehicle::{VehicleFamily, VehicleId, VehicleState};

use crate::MissionPlanner;
use crate::config::flight_modes::{ParamWriter, Progress};
use crate::config::optional::{at, button, label};
use crate::config::servo_output::{Check, CheckState, check_box};
use crate::telemetry::{Telemetry, TelemetryView};
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
/// `// C#: GCSViews/FlightData.resx; GCSViews/FlightData.cs:795, 1636, 4759`
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
}

/// The window's answer.
#[derive(Debug, Clone, PartialEq)]
pub enum Reply {
    Bool(bool),
    Param(Option<f32>),
    Field(Option<CsValue>),
    Done,
}

/// One request with the channel its answer comes back on.
pub type Asked = (Request, Sender<Reply>);

/// The [`ScriptHost`] on the script's thread: everything but `SendRC` and `Sleep` goes to the
/// window over the channel.
pub struct GuiScriptHost {
    asks: Sender<Asked>,
    sender: Option<(mp_link::LinkSender, VehicleId)>,
    api: ScriptApi,
}

impl std::fmt::Debug for GuiScriptHost {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GuiScriptHost")
            .field("has_link", &self.sender.is_some())
            .finish_non_exhaustive()
    }
}

impl GuiScriptHost {
    fn ask(&self, request: Request) -> Option<Reply> {
        let (tx, rx) = channel();
        self.asks.send((request, tx)).ok()?;
        rx.recv_timeout(ANSWER_WAIT).ok()
    }
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
        let Some((sender, vehicle)) = &self.sender else {
            return false;
        };
        let message = mp_link::commands::rc_override(*vehicle, self.api.overrides());
        let first = sender.send(&message);
        std::thread::sleep(Duration::from_millis(u64::from(RC_RESEND_GAP_MS)));
        let second = sender.send(&message);
        first && second
    }

    fn sleep(&mut self, milliseconds: u32) {
        std::thread::sleep(Duration::from_millis(u64::from(milliseconds)));
    }
}

/// What answering a request needs of the window, apart from the writes and the mode it sets:
/// the parameters held, the messages so far and the vehicle's state.
pub struct Answers<'a> {
    pub parameters: &'a [(String, f64)],
    pub messages: &'a [mp_link::messages::LogMessage],
    pub state: Option<&'a VehicleState>,
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
}

/// `cs.<field>` from the vehicle's state, by the C#'s names, for the fields the corpus and the
/// Status tab read most; a name not listed is an `AttributeError` in the script.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs`
pub fn cs_field(state: &VehicleState, name: &str) -> Option<CsValue> {
    Some(match name {
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
        Request::CsField(name) => (
            Some(Reply::Field(
                answers.state.and_then(|state| cs_field(state, name)),
            )),
            None,
        ),
    }
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
    /// `ChangeParam`s under way: the write and who is waiting for it.
    pending_writes: Vec<(RequestId, Sender<Reply>)>,
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
}

impl Default for ScriptsTab {
    fn default() -> Self {
        Self {
            selected: None,
            redirect: false,
            status: STATUS_NONE,
            run: None,
            asks: None,
            pending_writes: Vec::new(),
            console: String::new(),
            console_shown: false,
            autoscroll: true,
            result: None,
            status_line: None,
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
    /// `// C#: GCSViews/FlightData.cs:1630-1641`
    pub fn select(&mut self, path: &str) {
        let path = path.trim();
        self.selected = (!path.is_empty()).then(|| PathBuf::from(path));
    }

    /// `BUT_run_script_Click`: nothing unless the file exists; else the run started, the status
    /// "Running", and the console opened when the output is redirected.
    /// `// C#: GCSViews/FlightData.cs:786-812, 4727-4732`
    pub fn run_pressed(&mut self, telemetry: &Telemetry) {
        if self.is_running() {
            return;
        }
        let Some(path) = self.selected.clone() else {
            return;
        };
        let source = match std::fs::read_to_string(&path) {
            Ok(source) => source,
            Err(err) => {
                self.status_line = Some(format!("could not read {}: {err}", path.display()));
                return;
            }
        };
        self.start(&path, source, telemetry.send_handle());
    }

    /// The run itself, from a source already read.
    fn start(
        &mut self,
        path: &Path,
        source: String,
        sender: Option<(mp_link::LinkSender, VehicleId)>,
    ) {
        let (asks_tx, asks_rx) = channel();
        let host = GuiScriptHost {
            asks: asks_tx,
            sender,
            api: ScriptApi::new(),
        };
        let name = path
            .file_name()
            .map_or_else(|| path.display().to_string(), |n| n.to_string_lossy().into_owned());
        self.console.clear();
        self.console_shown = self.redirect;
        self.result = None;
        self.status = STATUS_RUNNING;
        self.pending_writes.clear();
        self.asks = Some(asks_rx);
        self.run = Some(ScriptRun::start(&name, source, Box::new(host)));
    }

    /// `BUT_abort_script_Click`.
    /// `// C#: GCSViews/FlightData.cs:1012-1017`
    pub fn abort_pressed(&mut self) {
        if let Some(run) = &self.run {
            run.abort();
        }
    }

    /// `BUT_edit_selected_Click`: the file opened with the desktop's editor, `Process.Start`
    /// with `UseShellExecute`; the C# swallows every failure, this puts it on the status line.
    /// `// C#: GCSViews/FlightData.cs:1223-1233`
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

    /// Once a frame: the run's output appended, the run's end noticed, and every request the
    /// script has made answered from the telemetry. Returns a line for the status line, if any.
    /// `// C#: GCSViews/FlightData.cs:4757-4776 (scriptChecker_Tick)`
    pub fn tick(&mut self, telemetry: &Telemetry, view: &TelemetryView) -> Option<String> {
        self.answer_requests(telemetry, view);
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

    /// The requests answered, the writes followed.
    fn answer_requests(&mut self, telemetry: &Telemetry, view: &TelemetryView) {
        let Some(asks) = &self.asks else {
            return;
        };
        let answers = Answers {
            parameters: &view.parameters,
            messages: &view.messages,
            state: view.state.as_deref(),
        };
        let mut writes = Vec::new();
        while let Ok((request, reply)) = asks.try_recv() {
            let (answer, effect) = service(&request, &answers);
            match effect {
                Some(Effect::Write(name, value)) => match telemetry.write_parameter(&name, f64::from(value), false) {
                    Some(id) => writes.push((id, reply.clone())),
                    None => {
                        let _ = reply.send(Reply::Bool(false));
                    }
                },
                Some(Effect::SetMode(number)) => telemetry.set_mode(number),
                Some(Effect::ClearMessages) => telemetry.clear_messages(),
                None => {}
            }
            if let Some(answer) = answer {
                let _ = reply.send(answer);
            }
        }
        self.pending_writes.extend(writes);
        self.pending_writes.retain(|(id, reply)| match telemetry.progress(*id) {
            Progress::Waiting => true,
            Progress::Finished(outcome) => {
                let _ = reply.send(Reply::Bool(matches!(
                    outcome,
                    RequestOutcome::Accepted { .. }
                )));
                false
            }
            Progress::Lost => {
                let _ = reply.send(Reply::Bool(false));
                false
            }
        });
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
    }
}

/// `Process.Start(new ProcessStartInfo(path) { UseShellExecute = true })`: the desktop's
/// association for the file.
fn open_with_shell(path: &Path) -> Result<(), String> {
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
        .child(label(STATUS_LABEL_AT.0, STATUS_LABEL_AT.1, tab.status, true))
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
        |this, _window, _cx| {
            this.fly_actions.ask(crate::fly::Prompt::SelectScript, "");
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
                |this, _window, _cx| {
                    let telemetry = &this.telemetry;
                    this.fly_data.scripts.run_pressed(telemetry);
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
        };
        assert_eq!(
            service(&Request::GetParam("RC3_MIN".to_owned()), &answers),
            (Some(Reply::Param(Some(1100.0))), None)
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
        let mut tab = ScriptsTab {
            redirect: true,
            ..Default::default()
        };
        tab.start(
            Path::new("/scripts/ok.py"),
            "print('hello')\nprint(Script.SendRC(3, 1500, True))\n".to_owned(),
            None,
        );
        assert_eq!(tab.status, STATUS_RUNNING);
        assert!(tab.console_shown);
        let deadline = std::time::Instant::now() + Duration::from_secs(30);
        while tab.result.is_none() && std::time::Instant::now() < deadline {
            tab.tick(&telemetry, &view);
            std::thread::sleep(Duration::from_millis(10));
        }
        assert_eq!(tab.result, Some(Ok(())));
        assert_eq!(tab.status, STATUS_FINISHED);
        assert_eq!(tab.console, "hello\nFalse\n");

        let mut tab = ScriptsTab::default();
        tab.start(Path::new("bad.py"), "MAV.getWP(1)\n".to_owned(), None);
        let deadline = std::time::Instant::now() + Duration::from_secs(30);
        let mut status = None;
        while tab.result.is_none() && std::time::Instant::now() < deadline {
            status = tab.tick(&telemetry, &view).or(status);
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(tab.result.as_ref().is_some_and(Result::is_err));
        assert!(tab.console.starts_with("Error running script "), "{}", tab.console);
        assert!(
            status.as_deref().is_some_and(|s| s.contains("MAV is not available")),
            "{status:?}"
        );
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
        );
        let deadline = std::time::Instant::now() + Duration::from_secs(30);
        while tab.result.is_none() && std::time::Instant::now() < deadline {
            tab.tick(&telemetry, &view);
            std::thread::sleep(Duration::from_millis(5));
        }
        assert!(tab.console.starts_with("0.0\nFalse\n"), "{}", tab.console);
        // `cs.lat` with no vehicle: the field is not there, an AttributeError.
        assert!(tab.result.as_ref().is_some_and(Result::is_err));
        assert!(tab.console.contains("cs has no field 'lat'"), "{}", tab.console);
    }
}
