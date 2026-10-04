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

//! The HELP screen - `MainV2`'s HELP - ported from `GCSViews/Help.cs`, `Help.Designer.cs` and
//! `Help.resx`: the help text (`Resources.help_text`), "Check for Updates", "Check for BETA
//! Updates", "Show Console Window (restart)" and the Change Log link; with `Update.CheckForUpdate`
//! and `Update.DoUpdate`'s conversation around `mp_update` - the "Update Found / Update Now"
//! question, the "Check for Updates" progress dialog with its Cancel, "No update available.", and
//! the handover to the updater - and `MainV2`'s once-a-day check at startup, `Program.
//! CleanupFiles`' copy of a new updater into place, and the `/update` and `/updatebeta` command
//! lines.
//!
//! Divergences, each at its site: the check's and the update's failures go on the status line
//! (the owner's ruling of 2026-09-25; the C# boxes them); the question's ChangeLog link is a line
//! of the question; "Show Console Window" writes its setting and nothing more (the console
//! window is Windows', allocated at the next start); the updater is `headless-planner
//! update-apply`, which ships beside the planner as `Updater.exe` ships beside Mission Planner;
//! and `MP_UPDATE_DIR` is the harness's door - the directory the update is written to, in place
//! of the program's own, and with it the updater is named rather than started.
//! `// C#: GCSViews/Help.cs; GCSViews/Help.Designer.cs; GCSViews/Help.resx; Utilities/Update.cs;
//! MainV2.cs:821, 3661-3671, 4030-4044; Program.cs:187-203, 614-626`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, TryRecvError, channel};
use std::time::Instant;

use gpui::{AnyElement, Context, MouseButton, Window, div, prelude::*, px, rgb};
use mp_update::check::Check;
use mp_update::{
    CHECK_FOR_UPDATES, Channel, Fetch, MASTER_WARNING, UPDATE_FAILED, UPDATE_FOUND,
    UPDATE_NOT_FOUND, UPDATE_NOW, Which,
};

use crate::MissionPlanner;
use mp_firmware::flow::Buttons;

use crate::config::firmware::{BoxIds, Waiting, link_label, message_box, question_box};
use crate::config::optional::button;
use crate::config::serial_ports::{Bar, ProgressIds, progress_dialog};
use crate::settings::Persisted;
use crate::ui::theme;

/// `$this.Size`. `// C#: GCSViews/Help.resx`
pub const PAGE: (f32, f32) = (578.0, 339.0);
/// `richTextBox1`: `Location` and `Size`, anchored on every side.
const TEXT_BOX: (f32, f32, f32, f32) = (106.0, 80.0, 348.0, 161.0);
/// `BUT_updatecheck`, anchored to the bottom.
const UPDATE_CHECK: (f32, f32, f32, f32) = (146.0, 268.0, 123.0, 29.0);
/// `BUT_betaupdate`, anchored to the bottom.
const BETA_UPDATE: (f32, f32, f32, f32) = (275.0, 268.0, 123.0, 29.0);
/// `CHK_showconsole`, anchored to the bottom.
const SHOW_CONSOLE: (f32, f32, f32, f32) = (188.0, 303.0, 174.0, 17.0);
/// `linkLabel1`, anchored to the bottom.
const CHANGE_LOG: (f32, f32) = (244.0, 323.0);
/// The texts. `// C#: GCSViews/Help.resx`
pub const UPDATE_CHECK_TEXT: &str = "Check for Updates";
pub const BETA_UPDATE_TEXT: &str = "Check for BETA Updates";
pub const SHOW_CONSOLE_TEXT: &str = "Show Console Window (restart)";
pub const CHANGE_LOG_TEXT: &str = "Change Log";
/// Where the Change Log link goes - Mission Planner's own, as the C# has it.
/// `// C#: GCSViews/Help.cs:62`
pub const CHANGE_LOG_URL: &str =
    "https://firmware.ardupilot.org/Tools/MissionPlanner/upgrade/ChangeLog.txt";
/// `CHK_showconsole`'s setting. `// C#: GCSViews/Help.cs:20, 52; MainV2.cs:821`
pub const SHOW_CONSOLE_KEY: &str = "showconsole";
/// The settings the once-a-day check keeps and reads. `// C#: MainV2.cs:3661-3669`
pub const UPDATE_CHECK_KEY: &str = "update_check";
pub const BETA_UPDATES_KEY: &str = "beta_updates";
/// The harness's door: the directory the update is written to in place of the program's.
pub const INSTALL_DIR_ENV: &str = "MP_UPDATE_DIR";
/// The updater beside the planner. `// C#: Utilities/Update.cs:62-72`
const UPDATER: &str = "headless-planner";

/// `Resources.help_text`, the RTF's words: its paragraphs, a tab where it has one, the links as
/// their addresses - with "Mission Planner" read "MissionPlannerRust" (the owner, 2026-10-04).
/// `// C#: Properties/Resources.resx (help_text)`
pub const HELP_TEXT: &str = "\n\
    \tWelcome to the MissionPlannerRust, mission planning for Unmanned Aerial Vehicles (UAV).\n\
    \tHelp:\n\
    Arduplane: http://ardupilot.org/plane\n\
    ArduCopter: http://ardupilot.org/copter\n\
    ArduRover: http://ardupilot.org/rover\n\
    \n\
    \tPlease visit http://ardupilot.org/planner  further information.\n\
    \tMain Developer: Michael Oborne\n\
    \tDesign: Samantha Nelson\n\
    \tContributions from: Andrew Radford,  Marooned, Ryan Beall, Tom Pittenger and https://github.com/ArduPilot/MissionPlanner/graphs/contributors\n\
    \tArduCopter Illustrations: Max Levine\n\
    Librarys: Gmap.net, Sharpkml, SharpZipLib, IronPython, KMLib, OpenTK, ZedGraph, alglib, BouncyCastle, DotSpatial, LibVLC, netDXF\n\
    \n\
    Companys that have contributed to MissionPlannerRusts development: 3D Robotics, Falcon Unmanned, UAV Solutions\n\
    \n\
    ShortCuts\n\
    \n\
    Main Screen\n\
    F2 - FlightData\n\
    F3 - FlightPlanner\n\
    F4 - Tuning\n\
    F5 - Refresh full param list\n\
    Control-F - Temp screen\n\
    Control-T - Blind connect\n\
    \n\
    FlightData\n\
    Control-1 - tab 1\n\
    Control-2 - tab 2\n\
    Control-3 - tab 3\n\
    Control-4 - tab 4\n\
    Control-5 - tab 5\n\
    Control-6 - tab 6\n\
    Control-7 - tab 7\n\
    Control-8 - tab 8\n\
    Control-9 - tab 9\n\
    Control-0 - tab 10\n\
    \n\
    FlightPlanner\n\
    Control-Z - undo\n\
    Control-O - open wp file\n\
    Control-S - save wp file\n\
    \n\
    AutoWP Grid\n\
    Control-O - open grid file\n\
    Control-S - save grid file\n";

/// The ids of the question and the boxes.
const BOXES: BoxIds = BoxIds {
    question: "help-question",
    yes: "help-question-yes",
    no: "help-question-no",
    message: "help-message",
    ok: "help-message-ok",
    path: "help-path",
    path_value: "help-path-value",
    path_ok: "help-path-ok",
    path_cancel: "help-path-cancel",
};

/// The ids of the progress dialog.
const PROGRESS: ProgressIds = ProgressIds {
    frame: "help-progress",
    bar: "help-progress-bar",
    cancel: "help-progress-cancel",
    backdrop: "help-progress-backdrop",
};

/// What the update is doing, as the screen shows it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Flow {
    /// Nothing.
    Idle,
    /// `CheckForUpdate` on its thread.
    Checking,
    /// "Update Found" - "Update Now", Yes or No. `// C#: Utilities/Update.cs:181-199`
    Question {
        /// The box's text, the ChangeLog link a line of it.
        text: String,
    },
    /// `DoUpdate`'s progress dialog, its worker running. `// C#: Utilities/Update.cs:199-215`
    Updating,
    /// A box with OK: "No update available.", or "This will update to MASTER release" before
    /// that update.
    Message {
        /// The box's text.
        text: String,
        /// Whether `DoUpdate` follows OK.
        then_update: bool,
    },
    /// The update is downloaded and the updater named: the application exits.
    Restarting,
}

/// What `DoUpdate`'s worker says: `UpdateProgressAndStatus`, then its end.
enum Progress {
    Said(i32, String),
    Done(Result<Vec<PathBuf>, String>),
}

/// `DoUpdate`'s worker, running.
struct Updating {
    progress: Receiver<Progress>,
    cancel: Arc<AtomicBool>,
}

/// The HELP page and `Update`'s state.
pub struct Help {
    /// `CHK_showconsole.Checked`.
    show_console: bool,
    /// What the update is doing.
    flow: Flow,
    /// `Update.dobeta` and `Update.domaster`, for the check or update in hand.
    which: Which,
    /// `CheckForUpdate(NotifyNoUpdate)`.
    notify_no_update: bool,
    /// `CheckForUpdate` on its thread.
    check: Option<Receiver<Result<Check, String>>>,
    /// `DoUpdate`'s worker.
    updating: Option<Updating>,
    /// The progress dialog's bar and its text.
    bar: Bar,
    progress_text: String,
    started: Instant,
    /// What the last check or update came to, for the facts.
    last: String,
    /// The `.new` files the last update wrote.
    written: usize,
    /// The updater's command line, once the update is downloaded.
    restart: Option<String>,
    /// The network.
    fetch: Arc<dyn Fetch + Send + Sync>,
    /// `Settings.GetRunningDirectory()`: where the program and its `version.txt` are.
    install_dir: PathBuf,
}

/// Where the program is, or the harness's door. `// C#: Settings.GetRunningDirectory`
#[must_use]
pub fn install_dir() -> PathBuf {
    if let Ok(door) = std::env::var(INSTALL_DIR_ENV) {
        return PathBuf::from(door);
    }
    std::env::current_exe()
        .ok()
        .and_then(|exe| exe.parent().map(Path::to_path_buf))
        .unwrap_or_else(|| PathBuf::from("."))
}

/// The updater's binary beside the program. `// C#: Utilities/Update.cs:62-72`
fn updater_path(install_dir: &Path) -> PathBuf {
    install_dir.join(format!("{UPDATER}{}", std::env::consts::EXE_SUFFIX))
}

/// `Settings.GetBoolean(key)`: "True" (as `bool.ToString()` writes it), without case.
fn setting_is_true(persisted: &Persisted, key: &str) -> bool {
    persisted
        .get(key)
        .is_some_and(|value| value.eq_ignore_ascii_case("true"))
}

/// `Program.CleanupFiles` and `updateCheckMain`: every `<updater>*.new` beside the program
/// copied over the updater and deleted - the updater cannot replace itself.
/// `// C#: Program.cs:614-626; Utilities/Update.cs:74-84`
pub fn cleanup_updater_files(install_dir: &Path) {
    let Ok(entries) = std::fs::read_dir(install_dir) else {
        return;
    };
    for path in entries.flatten().map(|e| e.path()) {
        let name = path
            .file_name()
            .map_or_else(String::new, |n| n.to_string_lossy().into_owned());
        if name.starts_with(UPDATER) && name.ends_with(".new") {
            let target = install_dir.join(name.trim_end_matches(".new"));
            if std::fs::copy(&path, &target).is_ok() {
                let _ = std::fs::remove_file(&path);
            }
        }
    }
}

/// `Program.CleanupFiles` at startup, in the program's own directory.
pub fn cleanup_files() {
    cleanup_updater_files(&install_dir());
}

impl Default for Help {
    fn default() -> Self {
        Self::new()
    }
}

impl Help {
    /// The page, with the network and the program's directory.
    #[must_use]
    pub fn new() -> Self {
        Self::with(Arc::new(mp_firmware::manifest::Http), install_dir())
    }

    /// The page over another network and directory: the tests'.
    #[must_use]
    pub fn with(fetch: Arc<dyn Fetch + Send + Sync>, install_dir: PathBuf) -> Self {
        Self {
            show_console: false,
            flow: Flow::Idle,
            which: Which::Stable,
            notify_no_update: false,
            check: None,
            updating: None,
            bar: Bar::default(),
            progress_text: String::new(),
            started: Instant::now(),
            last: "none".to_owned(),
            written: 0,
            restart: None,
            fetch,
            install_dir,
        }
    }

    /// `Help.Activate`: the check box from the setting. `// C#: GCSViews/Help.cs:17-31`
    pub fn activate(&mut self, persisted: &Persisted) {
        self.show_console = setting_is_true(persisted, SHOW_CONSOLE_KEY);
    }

    /// `CHK_showconsole_CheckedChanged`: the setting written as `Checked.ToString()`.
    /// `// C#: GCSViews/Help.cs:50-53`
    pub fn toggle_console(&mut self, persisted: &mut Persisted) {
        self.show_console = !self.show_console;
        persisted.set(
            SHOW_CONSOLE_KEY,
            if self.show_console { "True" } else { "False" },
        );
    }

    /// `CHK_showconsole.Checked`.
    #[cfg(test)]
    #[must_use]
    pub const fn show_console(&self) -> bool {
        self.show_console
    }

    /// Whether a check or an update is running, when the buttons wait (the C#'s dialogs are
    /// modal).
    #[must_use]
    pub const fn busy(&self) -> bool {
        !matches!(self.flow, Flow::Idle)
    }

    /// The channel `which` names, out of the settings.
    fn channel(which: Which, persisted: &Persisted) -> Channel {
        which.channel(&|key| persisted.get(key).map(str::to_owned))
    }

    /// `Update.CheckForUpdate(NotifyNoUpdate)` on a thread of its own: nothing without a
    /// version URL; else the channel's `version.txt` against the program's, and then the
    /// question, or "No update available." when asked to say so.
    /// `// C#: Utilities/Update.cs:118-178; GCSViews/Help.cs:33-48; MainV2.cs:4030-4044`
    pub fn check_for_update(
        &mut self,
        which: Which,
        notify_no_update: bool,
        persisted: &Persisted,
    ) {
        if self.busy() {
            return;
        }
        let channel = Self::channel(which, persisted);
        if channel.version_url.is_empty() {
            self.last = "no channel".to_owned();
            return;
        }
        self.which = which;
        self.notify_no_update = notify_no_update;
        self.flow = Flow::Checking;
        let (sender, receiver) = channel_pair();
        let fetch = Arc::clone(&self.fetch);
        let install_dir = self.install_dir.clone();
        let url = channel.version_url;
        let spawned = std::thread::Builder::new()
            .name("mp-update-check".to_owned())
            .spawn(move || {
                let outcome =
                    mp_update::check::check_for_update(fetch.as_ref(), &url, &install_dir);
                let _ = sender.send(outcome);
            });
        if spawned.is_err() {
            self.flow = Flow::Idle;
            return;
        }
        self.check = Some(receiver);
    }

    /// `BUT_updatecheck_Click`: the stable channel checked, "No update available." said.
    /// `// C#: GCSViews/Help.cs:33-48`
    pub fn update_check_clicked(&mut self, persisted: &Persisted) {
        self.check_for_update(Which::Stable, true, persisted);
    }

    /// `BUT_betaupdate_Click`: `dobeta`, and with Control held `domaster` behind "This will
    /// update to MASTER release"; then `DoUpdate`, no version asked.
    /// `// C#: GCSViews/Help.cs:66-82`
    pub fn beta_update_clicked(&mut self, control_held: bool, persisted: &Persisted) {
        if self.busy() {
            return;
        }
        self.which = Which::Beta;
        if control_held {
            self.which = Which::Master;
            self.flow = Flow::Message {
                text: MASTER_WARNING.to_owned(),
                then_update: true,
            };
            return;
        }
        self.start_update(persisted);
    }

    /// `Update.DoUpdate`: the "Check for Updates" progress dialog and its worker, on a thread.
    /// `// C#: Utilities/Update.cs:199-217, 684-738`
    pub fn start_update(&mut self, persisted: &Persisted) {
        let channel = Self::channel(self.which, persisted);
        self.flow = Flow::Updating;
        self.bar = Bar::default();
        self.progress_text = "Checking for Updates".to_owned();
        self.started = Instant::now();
        let (sender, receiver) = channel_pair();
        let cancel = Arc::new(AtomicBool::new(false));
        let cancelled = Arc::clone(&cancel);
        let fetch = Arc::clone(&self.fetch);
        let install_dir = self.install_dir.clone();
        let spawned = std::thread::Builder::new()
            .name("mp-update".to_owned())
            .spawn(move || {
                let reporter = sender.clone();
                let outcome = mp_update::check::do_update(
                    fetch.as_ref(),
                    &channel,
                    &install_dir,
                    &mut |percent, text| {
                        let _ = reporter.send(Progress::Said(percent, text.to_owned()));
                    },
                    &|| cancelled.load(Ordering::Relaxed),
                );
                let _ = sender.send(Progress::Done(outcome));
            });
        if spawned.is_err() {
            self.flow = Flow::Idle;
            return;
        }
        self.updating = Some(Updating {
            progress: receiver,
            cancel,
        });
    }

    /// The progress dialog's Cancel: `CancelRequested`, which the worker sees at its next file.
    /// `// C#: Utilities/Update.cs:209, 311-315`
    pub fn cancel_update(&self) {
        if let Some(updating) = &self.updating {
            updating.cancel.store(true, Ordering::Relaxed);
        }
    }

    /// The question's answer: Yes is `DoUpdate`. `// C#: Utilities/Update.cs:190-198`
    pub fn answer(&mut self, yes: bool, persisted: &Persisted) {
        if !matches!(self.flow, Flow::Question { .. }) {
            return;
        }
        if yes {
            self.start_update(persisted);
        } else {
            self.flow = Flow::Idle;
        }
    }

    /// A message's OK: the master update follows its warning.
    pub fn message_ok(&mut self, persisted: &Persisted) {
        let Flow::Message { then_update, .. } = &self.flow else {
            return;
        };
        if *then_update {
            self.start_update(persisted);
        } else {
            self.flow = Flow::Idle;
        }
    }

    /// `MainV2`'s check at startup: once a day on the stable channel, the day kept as
    /// `update_check`; on a day already checked, the beta channel when `beta_updates` is set.
    /// `// C#: MainV2.cs:3661-3671`
    pub fn startup_check(&mut self, persisted: &mut Persisted) {
        let today = crate::settings::short_date_today();
        if persisted.get(UPDATE_CHECK_KEY) != Some(today.as_str()) {
            self.check_for_update(Which::Stable, false, persisted);
            persisted.set(UPDATE_CHECK_KEY, today);
        } else if setting_is_true(persisted, BETA_UPDATES_KEY) {
            self.check_for_update(Which::Beta, false, persisted);
        }
    }

    /// The threads' news, once a frame: what to put on the status line, if anything.
    pub fn tick(&mut self) -> Option<String> {
        let mut status = None;
        if let Some(receiver) = &self.check {
            match receiver.try_recv() {
                Ok(outcome) => {
                    self.check = None;
                    status = self.checked(outcome);
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => {
                    self.check = None;
                    self.flow = Flow::Idle;
                }
            }
        }
        if let Some(updating) = &self.updating {
            loop {
                match updating.progress.try_recv() {
                    Ok(Progress::Said(percent, text)) => {
                        self.bar = Bar {
                            marquee: percent < 0,
                            value: percent,
                        };
                        self.progress_text = text;
                    }
                    Ok(Progress::Done(outcome)) => {
                        self.updating = None;
                        status = self.updated(outcome);
                        break;
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        self.updating = None;
                        self.flow = Flow::Idle;
                        break;
                    }
                }
            }
        }
        status
    }

    /// `CheckForUpdate`'s finding: the question with "BETA " before it for the beta channel, "No
    /// update available." when asked, or the failure, which the C# logs ("Update check failed")
    /// or boxes (the Help page's Error box) and the status line says.
    /// `// C#: Utilities/Update.cs:180-203; GCSViews/Help.cs:44-47; MainV2.cs:4040-4043`
    fn checked(&mut self, outcome: Result<Check, String>) -> Option<String> {
        match outcome {
            Ok(Check::UpdateFound { changelog_url }) => {
                let extra = if self.which == Which::Beta {
                    "BETA "
                } else {
                    ""
                };
                self.last = "update found".to_owned();
                self.flow = Flow::Question {
                    text: format!("{extra}{UPDATE_FOUND}\nChangeLog: {changelog_url}"),
                };
                None
            }
            Ok(Check::UpToDate) => {
                self.last = "up to date".to_owned();
                self.flow = if self.notify_no_update {
                    Flow::Message {
                        text: UPDATE_NOT_FOUND.to_owned(),
                        then_update: false,
                    }
                } else {
                    Flow::Idle
                };
                None
            }
            Ok(Check::NoChannel) => {
                self.last = "no channel".to_owned();
                self.flow = Flow::Idle;
                None
            }
            Err(why) => {
                self.last = format!("Update check failed: {why}");
                self.flow = Flow::Idle;
                Some(self.last.clone())
            }
        }
    }

    /// `updateCheckMain`'s end: the files downloaded, a new updater copied into place, "Starting
    /// Updater" and the application's exit; or "Update Failed " and why.
    /// `// C#: Utilities/Update.cs:60-109`
    fn updated(&mut self, outcome: Result<Vec<PathBuf>, String>) -> Option<String> {
        match outcome {
            Ok(written) => {
                self.written = written.len();
                cleanup_updater_files(&self.install_dir);
                let planner = std::env::current_exe().map_or_else(
                    |_| String::from("planner"),
                    |exe| exe.to_string_lossy().into_owned(),
                );
                self.restart = Some(format!(
                    "{} update-apply {planner}",
                    updater_path(&self.install_dir).display()
                ));
                self.last = "Starting Updater".to_owned();
                self.flow = Flow::Restarting;
                Some(self.last.clone())
            }
            Err(why) => {
                self.last = format!("{UPDATE_FAILED}{why}");
                self.flow = Flow::Idle;
                Some(self.last.clone())
            }
        }
    }

    /// The updater's command, once the update is downloaded.
    #[must_use]
    pub fn restart_command(&self) -> Option<std::process::Command> {
        if !matches!(self.flow, Flow::Restarting) {
            return None;
        }
        let planner = std::env::current_exe().ok()?;
        let mut command = std::process::Command::new(updater_path(&self.install_dir));
        command
            .arg("update-apply")
            .arg(planner)
            .current_dir(&self.install_dir);
        Some(command)
    }

    /// The restart taken: under the harness's door the updater is named, not started, and the
    /// page goes back to idle.
    pub fn restart_taken(&mut self) {
        self.flow = Flow::Idle;
    }

    /// The facts a UI test asserts on.
    pub fn record_facts(&self) {
        use crate::facts::record;
        record("help.console", self.show_console);
        record(
            "help.flow",
            match &self.flow {
                Flow::Idle => "idle",
                Flow::Checking => "checking",
                Flow::Question { .. } => "question",
                Flow::Updating => "updating",
                Flow::Message { .. } => "message",
                Flow::Restarting => "restarting",
            },
        );
        record(
            "help.question",
            match &self.flow {
                Flow::Question { text } | Flow::Message { text, .. } => text.as_str(),
                _ => "none",
            },
        );
        record("help.progress", &self.progress_text);
        record("help.last", &self.last);
        record("help.written", self.written);
        record("help.restart", self.restart.as_deref().unwrap_or("none"));
    }
}

/// A channel pair, typed by use.
fn channel_pair<T>() -> (std::sync::mpsc::Sender<T>, Receiver<T>) {
    channel()
}

/// `/update` and `/updatebeta`: `DoUpdate` without a window - its progress on the console, the
/// updater started and the exit code the shell sees. `// C#: Program.cs:192-203`
pub fn update_from_command_line(beta: bool) -> i32 {
    let persisted = Persisted::load();
    let which = if beta { Which::Beta } else { Which::Stable };
    let channel = which.channel(&|key| persisted.get(key).map(str::to_owned));
    let install_dir = install_dir();
    println!("{CHECK_FOR_UPDATES}");
    let outcome = mp_update::check::do_update(
        &mp_firmware::manifest::Http,
        &channel,
        &install_dir,
        &mut |_percent, text| println!("{text}"),
        &|| false,
    );
    match outcome {
        Ok(_) => {
            cleanup_updater_files(&install_dir);
            println!("Starting Updater");
            let planner = std::env::current_exe().unwrap_or_else(|_| PathBuf::from("planner"));
            match std::process::Command::new(updater_path(&install_dir))
                .arg("update-apply")
                .arg(planner)
                .current_dir(&install_dir)
                .spawn()
            {
                Ok(_) => 0,
                Err(why) => {
                    eprintln!("{UPDATE_FAILED}{why}");
                    1
                }
            }
        }
        Err(why) => {
            eprintln!("{UPDATE_FAILED}{why}");
            1
        }
    }
}

impl MissionPlanner {
    /// Once a frame: the update's threads, their news on the status line, and the restart - the
    /// updater started and the application quit, or, under the harness's door, the updater
    /// named.
    pub(crate) fn help_tick(&mut self, cx: &mut Context<Self>) {
        if let Some(status) = self.help.tick() {
            self.file_status = Some(status);
        }
        if let Some(mut command) = self.help.restart_command() {
            if std::env::var_os(INSTALL_DIR_ENV).is_some() {
                self.help.restart_taken();
                return;
            }
            match command.spawn() {
                // "Quitting existing process"
                Ok(_) => cx.quit(),
                Err(why) => {
                    self.file_status = Some(format!("{UPDATE_FAILED}{why}"));
                    self.help.restart_taken();
                }
            }
        }
    }
}

/// The page, in the Designer's arrangement: the help text filling the middle and anchored on
/// every side, the two buttons, the check box and the link held to the bottom.
/// `// C#: GCSViews/Help.Designer.cs; GCSViews/Help.resx`
pub fn screen(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let help = &this.help;
    let enabled = !help.busy();
    let (tx, ty, tw, th) = TEXT_BOX;
    let text_box = crate::probe::measured("help-text", div())
        .id("help-text")
        .absolute()
        .left(px(tx))
        .top(px(ty))
        .right(px(PAGE.0 - tx - tw))
        .bottom(px(PAGE.1 - ty - th))
        .p_1()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .overflow_y_scroll()
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .child(
            div()
                .flex()
                .flex_col()
                .children(HELP_TEXT.lines().map(|line| {
                    let text = line.replace('\t', "    ");
                    div()
                        .min_h(px(14.0))
                        .whitespace_nowrap()
                        .child(if text.is_empty() {
                            " ".to_owned()
                        } else {
                            text
                        })
                })),
        );
    // Anchored to the bottom: placed from it, where the Designer's page is 339 high.
    let from_bottom = |(x, y, w, h): (f32, f32, f32, f32)| {
        div()
            .absolute()
            .left(px(x))
            .bottom(px(PAGE.1 - y - h))
            .w(px(w))
            .h(px(h))
    };
    let (_, _, bw, bh) = UPDATE_CHECK;
    let update_check = from_bottom(UPDATE_CHECK).child(button(
        "help-updatecheck",
        UPDATE_CHECK_TEXT,
        (0.0, 0.0, bw, bh),
        enabled,
        |this, _window, _cx| this.help.update_check_clicked(&this.persisted),
        cx,
    ));
    let beta_update = from_bottom(BETA_UPDATE).child(button(
        "help-betaupdate",
        BETA_UPDATE_TEXT,
        (0.0, 0.0, bw, bh),
        enabled,
        |this, window, _cx| {
            let control = window.modifiers().control;
            this.help.beta_update_clicked(control, &this.persisted);
        },
        cx,
    ));
    let (_, _, cw, ch) = SHOW_CONSOLE;
    let mark_colour = if enabled { theme::ACCENT } else { theme::DIM };
    let show_console = from_bottom(SHOW_CONSOLE).child(
        crate::probe::measured("help-showconsole", div())
            .id("help-showconsole")
            .w(px(cw))
            .h(px(ch))
            .flex()
            .items_center()
            .gap_1()
            .text_xs()
            .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
            .cursor_pointer()
            .child(
                div()
                    .size(px(12.0))
                    .border_1()
                    .border_color(rgb(mark_colour))
                    .flex()
                    .items_center()
                    .justify_center()
                    .children(
                        help.show_console
                            .then(|| div().size(px(6.0)).bg(rgb(mark_colour))),
                    ),
            )
            .child(SHOW_CONSOLE_TEXT)
            .on_mouse_down(
                MouseButton::Left,
                cx.listener(|this, _event, _window, cx| {
                    if !this.help.busy() {
                        this.help.toggle_console(&mut this.persisted);
                        cx.notify();
                    }
                }),
            ),
    );
    let change_log = div()
        .absolute()
        .left(px(0.0))
        .bottom(px(PAGE.1 - CHANGE_LOG.1 - 13.0))
        .w(px(PAGE.0))
        .h(px(13.0))
        .child(link_label(
            "help-changelog",
            CHANGE_LOG_TEXT,
            (CHANGE_LOG.0, 0.0),
            true,
            |this, _window, _cx| {
                // `Process.Start(url)`: the desktop's browser.
                if let Err(why) = crate::scripts_tab::open_with_shell(Path::new(CHANGE_LOG_URL)) {
                    this.file_status = Some(format!("could not open the Change Log: {why}"));
                }
            },
            cx,
        ));
    let _ = window;
    crate::probe::measured("help-body", div())
        .id("help-body")
        .relative()
        .flex_1()
        .min_h(px(PAGE.1))
        .min_w(px(PAGE.0))
        .bg(rgb(theme::BG))
        .child(text_box)
        .child(update_check)
        .child(beta_update)
        .child(show_console)
        .child(change_log)
        .into_any_element()
}

/// The update's question, progress dialog or message, over whatever screen is showing.
pub fn overlay(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let help = &this.help;
    Some(match &help.flow {
        Flow::Idle | Flow::Checking | Flow::Restarting => return None,
        Flow::Question { text } => question_box(
            BOXES,
            UPDATE_NOW,
            text,
            Buttons::YesNo,
            window,
            |this, yes| this.help.answer(yes, &this.persisted),
            cx,
        ),
        Flow::Message { text, .. } => {
            let waiting = Waiting {
                text: text.clone(),
                caption: String::new(),
                buttons: None,
            };
            message_box(
                BOXES,
                &waiting,
                window,
                |this| this.help.message_ok(&this.persisted),
                cx,
            )
        }
        Flow::Updating => progress_dialog(
            PROGRESS,
            &format!("{CHECK_FOR_UPDATES}\n{}", help.progress_text),
            help.bar,
            help.started,
            Some(|this: &mut MissionPlanner| this.help.cancel_update()),
            window,
            cx,
        ),
    })
}

/// The facts a UI test asserts on.
pub fn record_facts(help: &Help) {
    help.record_facts();
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The help screen names this program: "Mission Planner" read "MissionPlannerRust" (the owner,
    /// 2026-10-04), the rest the C#'s words.
    #[test]
    fn the_help_text_names_missionplannerrust() {
        assert!(!HELP_TEXT.contains("Mission Planner"), "{HELP_TEXT}");
        assert!(HELP_TEXT.contains(
            "Welcome to the MissionPlannerRust, mission planning for Unmanned Aerial Vehicles (UAV)."
        ));
        assert!(HELP_TEXT.contains("Control-T - Blind connect"));
    }
    use std::collections::HashMap;

    struct Server(HashMap<String, Vec<u8>>);

    impl Fetch for Server {
        fn get(&self, url: &str) -> Result<Vec<u8>, String> {
            let bare = url.split('?').next().unwrap_or(url);
            self.0
                .get(bare)
                .cloned()
                .ok_or_else(|| format!("http status 404 for {bare}"))
        }
    }

    fn scratch(test: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("mp-gui-help-{test}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("a scratch directory");
        dir
    }

    fn settings(pairs: &[(&str, &str)]) -> Persisted {
        let mut persisted = Persisted::at(None);
        for (key, value) in pairs {
            persisted.set(key, *value);
        }
        persisted
    }

    fn wait(help: &mut Help) -> Option<String> {
        let deadline = Instant::now() + std::time::Duration::from_secs(10);
        loop {
            let status = help.tick();
            if !matches!(help.flow, Flow::Checking | Flow::Updating) || Instant::now() > deadline {
                return status;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    }

    fn server(dir: &Path, version: &str) -> Arc<Server> {
        let probe = b"probe content";
        let checksums = format!("{}  ./update-probe.txt\n", mp_update::md5::hex(probe));
        let _ = dir;
        Arc::new(Server(HashMap::from([
            (
                "https://x/version.txt".to_owned(),
                version.as_bytes().to_vec(),
            ),
            (
                "https://x/beta/version.txt".to_owned(),
                b"9.0.0.0\n".to_vec(),
            ),
            (
                "https://x/checksums.txt".to_owned(),
                checksums.clone().into_bytes(),
            ),
            (
                "https://x/beta/checksums.txt".to_owned(),
                checksums.into_bytes(),
            ),
            (
                "https://x/files//update-probe.txt".to_owned(),
                probe.to_vec(),
            ),
        ])))
    }

    const KEYS: [(&str, &str); 6] = [
        ("UpdateLocationVersion", "https://x/version.txt"),
        ("UpdateLocationMD5", "https://x/checksums.txt"),
        ("UpdateLocation", "https://x/files/"),
        ("BetaUpdateLocationVersion", "https://x/beta/version.txt"),
        ("BetaUpdateLocationMD5", "https://x/beta/checksums.txt"),
        ("BetaUpdateLocationZip", "https://x/files/"),
    ];

    #[test]
    fn no_channel_does_nothing_and_a_channel_asks_the_question() {
        let dir = scratch("question");
        let mut help = Help::with(server(&dir, "2.0.0.0\n"), dir.clone());
        help.update_check_clicked(&settings(&[]));
        assert_eq!(help.flow, Flow::Idle);
        assert_eq!(help.last, "no channel");

        let persisted = settings(&KEYS);
        help.update_check_clicked(&persisted);
        assert_eq!(help.flow, Flow::Checking);
        assert!(wait(&mut help).is_none());
        assert_eq!(
            help.flow,
            Flow::Question {
                text: "Update Found\nChangeLog: https://x/ChangeLog.txt".to_owned()
            }
        );
        help.answer(false, &persisted);
        assert_eq!(help.flow, Flow::Idle);

        // Up to date, and asked to say so: "No update available."; the startup check says nothing.
        std::fs::write(dir.join("version.txt"), "2.0.0.0\n").expect("version.txt");
        help.update_check_clicked(&persisted);
        wait(&mut help);
        assert_eq!(
            help.flow,
            Flow::Message {
                text: UPDATE_NOT_FOUND.to_owned(),
                then_update: false
            }
        );
        help.message_ok(&persisted);
        assert_eq!(help.flow, Flow::Idle);
        help.check_for_update(Which::Stable, false, &persisted);
        wait(&mut help);
        assert_eq!(help.flow, Flow::Idle);
        assert_eq!(help.last, "up to date");

        // A channel that does not answer: the failure on the status line.
        let broken = settings(&[("UpdateLocationVersion", "https://x/none.txt")]);
        help.update_check_clicked(&broken);
        let status = wait(&mut help).expect("a status");
        assert!(status.starts_with("Update check failed: "), "{status}");
        assert_eq!(help.flow, Flow::Idle);
        std::fs::remove_dir_all(&dir).expect("cleanup");
    }

    #[test]
    fn yes_downloads_and_names_the_updater_and_the_beta_button_skips_the_question() {
        let dir = scratch("update");
        let mut help = Help::with(server(&dir, "2.0.0.0\n"), dir.clone());
        let persisted = settings(&KEYS);
        help.update_check_clicked(&persisted);
        wait(&mut help);
        help.answer(true, &persisted);
        assert_eq!(help.flow, Flow::Updating);
        let status = wait(&mut help);
        assert_eq!(status.as_deref(), Some("Starting Updater"));
        assert_eq!(help.flow, Flow::Restarting);
        assert_eq!(help.written, 1);
        assert_eq!(
            std::fs::read(dir.join("update-probe.txt.new")).expect("the new file"),
            b"probe content"
        );
        assert!(
            help.restart
                .as_deref()
                .is_some_and(|r| r.contains("update-apply")),
            "{:?}",
            help.restart
        );
        assert!(help.restart_command().is_some());
        help.restart_taken();
        assert_eq!(help.flow, Flow::Idle);

        // The beta button: no question, the update at once; with Control, the warning first.
        std::fs::remove_file(dir.join("update-probe.txt.new")).expect("removed");
        help.beta_update_clicked(false, &persisted);
        assert_eq!(help.flow, Flow::Updating);
        assert_eq!(help.which, Which::Beta);
        wait(&mut help);
        assert_eq!(help.flow, Flow::Restarting);
        help.restart_taken();
        help.beta_update_clicked(true, &persisted);
        assert_eq!(
            help.flow,
            Flow::Message {
                text: MASTER_WARNING.to_owned(),
                then_update: true
            }
        );
        assert_eq!(help.which, Which::Master);
        // The master channel has no addresses here: its update fails and says so.
        help.message_ok(&persisted);
        assert_eq!(help.flow, Flow::Updating);
        let status = wait(&mut help).expect("a status");
        assert!(status.starts_with(UPDATE_FAILED), "{status}");
        assert_eq!(help.flow, Flow::Idle);
        std::fs::remove_dir_all(&dir).expect("cleanup");
    }

    #[test]
    fn cancel_is_the_users_request() {
        let dir = scratch("cancel");
        let mut help = Help::with(server(&dir, "2.0.0.0\n"), dir.clone());
        let persisted = settings(&KEYS);
        help.which = Which::Stable;
        help.start_update(&persisted);
        help.cancel_update();
        let status = wait(&mut help).expect("a status");
        // Cancelled before the file, or after it was already fetched: either way the C#'s words.
        assert!(
            status == format!("{UPDATE_FAILED}User Request") || status == "Starting Updater",
            "{status}"
        );
        std::fs::remove_dir_all(&dir).expect("cleanup");
    }

    #[test]
    fn the_console_setting_and_the_daily_check_follow_the_settings() {
        let dir = scratch("daily");
        let mut help = Help::with(server(&dir, "2.0.0.0\n"), dir.clone());
        let mut persisted = settings(&KEYS);
        help.activate(&persisted);
        assert!(!help.show_console());
        help.toggle_console(&mut persisted);
        assert!(help.show_console());
        assert_eq!(persisted.get(SHOW_CONSOLE_KEY), Some("True"));
        help.toggle_console(&mut persisted);
        assert_eq!(persisted.get(SHOW_CONSOLE_KEY), Some("False"));
        persisted.set(SHOW_CONSOLE_KEY, "true");
        help.activate(&persisted);
        assert!(help.show_console());

        // A new day: the stable check, and the day kept.
        help.startup_check(&mut persisted);
        assert_eq!(help.flow, Flow::Checking);
        assert_eq!(help.which, Which::Stable);
        assert_eq!(
            persisted.get(UPDATE_CHECK_KEY),
            Some(crate::settings::short_date_today().as_str())
        );
        wait(&mut help);
        help.answer(false, &persisted);
        // The same day again: nothing, unless beta updates are on, which checks the beta channel.
        help.startup_check(&mut persisted);
        assert_eq!(help.flow, Flow::Idle);
        persisted.set(BETA_UPDATES_KEY, "True");
        help.startup_check(&mut persisted);
        assert_eq!(help.flow, Flow::Checking);
        assert_eq!(help.which, Which::Beta);
        wait(&mut help);
        assert!(
            matches!(&help.flow, Flow::Question { text } if text.starts_with("BETA Update Found"))
        );
        std::fs::remove_dir_all(&dir).expect("cleanup");
    }

    #[test]
    fn a_new_updater_is_copied_into_place() {
        let dir = scratch("updater");
        let updater = updater_path(&dir);
        std::fs::write(&updater, b"old").expect("old updater");
        let new = dir.join(format!(
            "{}.new",
            updater.file_name().expect("a name").to_string_lossy()
        ));
        std::fs::write(&new, b"new").expect("new updater");
        cleanup_updater_files(&dir);
        assert_eq!(std::fs::read(&updater).expect("updater"), b"new");
        assert!(!new.exists());
        std::fs::remove_dir_all(&dir).expect("cleanup");
    }
}
