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
//! Updates", "Show Console Window (restart)" and the Change Log link; with the conversation of
//! `Update.CheckForUpdate` and `Update.DoUpdate` - the "Update Found / Update Now" question, the
//! "Check for Updates" progress dialog with its Cancel, "No update available." - `MainV2`'s
//! once-a-day check at startup, `Program.CleanupFiles`, and the `/update` and `/updatebeta`
//! command lines.
//!
//! The update behind that conversation is this planner's own, not the C#'s, on the owner's word
//! of 2026-11-18 (updates.md, and `crates/mp-update`): a release's manifest read from GitHub, the
//! release's archive fetched, checked, tried and moved into place by the planner itself, with no
//! `Updater.exe`. What differs from the C# for the user, each at its site: the question says what
//! is offered ("0.0.1 to 0.0.2"); the beta button asks as the other does, where the C#'s updates
//! at once, and Control on it is nothing, there being no master channel; a planner with a link
//! open installs and does not restart; the Debian package's install fetches the new package and
//! shows its command; a development build is told so; CONFIG > Planner has a box that turns the
//! check at start off; and in a web page both buttons are hidden, as the C# hides them for a
//! Windows Store install (`Help.cs:27-31`).
//!
//! Divergences kept from before: the check's and the update's failures go on the status line
//! (the owner's ruling of 2026-09-25; the C# boxes them); "Show Console Window" writes its
//! setting and nothing more (the console window is Windows', allocated at the next start); and
//! `MP_UPDATE_DIR` is the harness's door - the directory the update is installed in, in place of
//! the program's own, and with it the new planner is named rather than started.
//! `// C#: GCSViews/Help.cs; GCSViews/Help.Designer.cs; GCSViews/Help.resx; Utilities/Update.cs;
//! MainV2.cs:823, 3668-3678, 4037-4051; Program.cs:189-205, 616-628`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]
// In a web page nothing is updated, so what the update's threads use is unused there.
#![cfg_attr(target_family = "wasm", allow(dead_code, unused_imports, unused_variables))]

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, TryRecvError, channel};
use web_time::Instant;

use gpui::{AnyElement, Context, MouseButton, Window, div, prelude::*, px, rgb};
use mp_update::identity::{Channel, Identity};
use mp_update::manifest::{Manifest, manifest_url};
use mp_update::{
    CHECK_FOR_UPDATES, DEVELOPMENT_BUILD, UPDATE_FAILED, UPDATE_FOUND, UPDATE_NOT_FOUND,
    UPDATE_NOW,
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
/// The guide to Tailscale, on Change Log's line to its right: not in the C#, the owner's plan of
/// 2026-10-14 ("using_tailscale.md reachable from HELP"), shown while Tailscale is enabled in the
/// PLUGINS tab (`plugin_manager::TAILSCALE`).
const TAILSCALE_GUIDE: (f32, f32) = (344.0, 323.0);
/// Its text.
pub const TAILSCALE_GUIDE_TEXT: &str = "Using Tailscale";
/// Where it goes: the guide, as the repository shows it.
pub const TAILSCALE_GUIDE_URL: &str =
    "https://github.com/davidbuzz/MissionPlannerRust/blob/main/using_tailscale.md";
/// The texts. `// C#: GCSViews/Help.resx`
pub const UPDATE_CHECK_TEXT: &str = "Check for Updates";
pub const BETA_UPDATE_TEXT: &str = "Check for BETA Updates";
pub const SHOW_CONSOLE_TEXT: &str = "Show Console Window (restart)";
pub const CHANGE_LOG_TEXT: &str = "Change Log";
/// Where the Change Log link goes - Mission Planner's own, as the C# has it.
/// `// C#: GCSViews/Help.cs:62`
pub const CHANGE_LOG_URL: &str =
    "https://firmware.ardupilot.org/Tools/MissionPlanner/upgrade/ChangeLog.txt";
/// `CHK_showconsole`'s setting. `// C#: GCSViews/Help.cs:20, 52; MainV2.cs:823`
pub const SHOW_CONSOLE_KEY: &str = "showconsole";
/// The settings the once-a-day check keeps and reads. `// C#: MainV2.cs:3668-3676`
pub const UPDATE_CHECK_KEY: &str = "update_check";
pub const BETA_UPDATES_KEY: &str = "beta_updates";
/// CONFIG > Planner's "Check for updates at start": "False" turns the once-a-day check off. Not
/// in the C#, whose check has no switch: the owner's (2026-11-18).
pub const UPDATE_AT_START_KEY: &str = "update_check_at_start";
/// The harness's door: the directory the update is installed in, in place of the program's.
pub const INSTALL_DIR_ENV: &str = "MP_UPDATE_DIR";
/// What the buttons say once an update is installed and this planner is still the old one.
pub const RESTART_TO_FINISH: &str = "The update is installed: restart the planner to use it";
/// The status line when an update has been moved into place.
pub const UPDATE_INSTALLED: &str = "Update installed";
/// The status line when it was installed with a link open: no restart.
pub const INSTALLED_LATER: &str =
    "Update installed: it takes effect when the planner is next started";
/// A platform the release workflow builds nothing for.
pub const NO_RELEASE_HERE: &str = "no release is published for this kind of computer";

/// `Resources.help_text`, the RTF's words: its paragraphs, a tab where it has one, the links as
/// their addresses - with "Mission Planner" read "MissionPlannerRust" (the owner, 2026-10-04).
/// The companies are Mission Planner's, not this port's: none yet, the owner asks them to get in
/// touch (2026-10-04). The C#'s "Main Developer: Michael Oborne" is two lines, the owner's words
/// (2026-10-14): this port's developer, and Mission Planner's as its honorary one. The libraries
/// are this port's, not the C#'s twelve, none of which it uses (the owner, 2026-10-14): the
/// notable crates, GStreamer for video, and the count `THIRD_PARTY_LICENSES` lists. The
/// contributions are this port's: none yet, as with the companies (the owner, 2026-10-14). Where
/// to find more is this port's page, where the C#'s points at Mission Planner's manual,
/// ardupilot.org/planner (the owner's bug of 2026-10-14; the fix proposed then, his word of
/// 2026-10-16), and with the C#'s missing "for" put back.
/// `// C#: Properties/Resources.resx (help_text)`
pub const HELP_TEXT: &str = "\n\
    \tWelcome to the MissionPlannerRust, mission planning for Unmanned Aerial Vehicles (UAV).\n\
    \tHelp:\n\
    Arduplane: http://ardupilot.org/plane\n\
    ArduCopter: http://ardupilot.org/copter\n\
    ArduRover: http://ardupilot.org/rover\n\
    \n\
    \tPlease visit https://github.com/davidbuzz/MissionPlannerRust for further information.\n\
    \tMain Developer:David Buzz  <davidbuzz@gmail.com>\n\
    \tHonourary Developer:Michael Oborne , the OG that this is inspired by/ripped off from.\n\
    \tDesign: Samantha Nelson\n\
    \tContributions to MissionPlannerRust from: none yet, pls get in touch - https://github.com/davidbuzz/MissionPlannerRust\n\
    \tArduCopter Illustrations: Max Levine\n\
    Libraries: gpui (Zed Industries), RustPython, wasmtime, serialport, ureq, image, tiny-skia, nalgebra, rustfft, dxf, roxmltree, flate2, ring, GStreamer - 1,128 crates in all, listed in THIRD_PARTY_LICENSES\n\
    \n\
    Companies that have contributed to MissionPlannerRust's development: none yet, pls get in touch\n\
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
        /// The box's text, the C#'s with what is offered after it, and its
        /// `[link;...;ChangeLog]`, which the box draws as a link under the words
        /// (`crate::ui::box_text`).
        text: String,
    },
    /// `DoUpdate`'s progress dialog, its worker running. `// C#: Utilities/Update.cs:199-215`
    Updating,
    /// A box with OK: "No update available.", or where the new package is and its command.
    Message {
        /// The box's text.
        text: String,
    },
    /// The update is installed: the new planner is started and this one exits.
    Restarting,
}

/// What an update came to.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Done {
    /// Installed in place: the files moved in.
    Installed(usize),
    /// The Debian package, fetched for the package manager to install.
    Package(PathBuf),
}

/// What `DoUpdate`'s worker says: `UpdateProgressAndStatus`, then its end.
enum Progress {
    Said(i32, String),
    Done(Result<Done, String>),
}

/// `DoUpdate`'s worker, running.
struct Updating {
    progress: Receiver<Progress>,
    cancel: Arc<AtomicBool>,
}

/// How this planner is installed, which decides what "Update Now" does (updates.md, 4.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// An archive unpacked into a folder: updated in place.
    Archive,
    /// The Debian package: the new package fetched and its command shown.
    Deb,
}

/// What follows an update installed in place.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AfterInstall {
    /// Under the harness's door: the new planner is named, not started.
    Named,
    /// A link is open: no restart, the update takes effect at the next start. A ground station
    /// does not end itself while it is connected to a vehicle (the owner, 2026-11-18).
    Later,
    /// The new planner started and this one ended, as the C#'s updater restarts it.
    Restart,
}

/// What follows, by the door and the link.
#[must_use]
pub const fn after_install(door: bool, linked: bool) -> AfterInstall {
    if door {
        AfterInstall::Named
    } else if linked {
        AfterInstall::Later
    } else {
        AfterInstall::Restart
    }
}

/// The network the updater asks; a web page has no updater.
#[cfg(not(target_family = "wasm"))]
type Network = Arc<dyn mp_update::download::Source>;
#[cfg(target_family = "wasm")]
type Network = ();
/// The new planner's trial before it is installed (`mp_update::install::trial`).
type Trial = fn(&Path, &Manifest) -> Result<(), String>;

/// The HELP page and the update's state.
pub struct Help {
    /// `CHK_showconsole.Checked`.
    show_console: bool,
    /// What the update is doing.
    flow: Flow,
    /// The channel of the check or update in hand: `Update.dobeta`.
    which: Channel,
    /// `CheckForUpdate(NotifyNoUpdate)`.
    notify_no_update: bool,
    /// `CheckForUpdate` on its thread: the release it offers, if any.
    check: Option<Receiver<Result<Option<Box<Manifest>>, String>>>,
    /// The release the question is about, and "Update Now" installs.
    offered: Option<Manifest>,
    /// `DoUpdate`'s worker.
    updating: Option<Updating>,
    /// The progress dialog's bar and its text.
    bar: Bar,
    progress_text: String,
    started: Instant,
    /// What the last check or update came to, for the facts.
    last: String,
    /// The files the last update moved into place.
    written: usize,
    /// The new planner's command line, once the update is installed.
    restart: Option<String>,
    /// An update is installed and this planner is still the old one: the buttons say to restart.
    installed: bool,
    /// The network.
    network: Network,
    /// Where the planner is installed. `// C#: Settings.GetRunningDirectory`
    install_dir: PathBuf,
    /// What this build is; `None` a development build, which is never updated from a release.
    build: Option<Identity>,
    /// How it is installed.
    kind: Kind,
    /// Where the Debian package's new package is fetched to.
    downloads: PathBuf,
    /// The new planner's trial.
    trial: Trial,
    /// How many times a button has been refused, for the facts.
    refused: usize,
    /// The last address a link of this page handed to the desktop's browser, for the facts.
    opened: Option<&'static str>,
    /// Whether the Tailscale guide's link showed in the last frame, for the facts.
    tailscale_link: std::cell::Cell<bool>,
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

/// The planner's file name, as a release's archive holds it.
fn program() -> String {
    format!("planner{}", std::env::consts::EXE_SUFFIX)
}

/// `Settings.GetBoolean(key)`: "True" (as `bool.ToString()` writes it), without case.
fn setting_is_true(persisted: &Persisted, key: &str) -> bool {
    persisted
        .get(key)
        .is_some_and(|value| value.eq_ignore_ascii_case("true"))
}

/// Whether the check at start is wanted: yes until CONFIG > Planner's box says "False".
fn check_at_start(persisted: &Persisted) -> bool {
    !persisted
        .get(UPDATE_AT_START_KEY)
        .is_some_and(|value| value.eq_ignore_ascii_case("false"))
}

/// At startup, in the program's own directory: the files the last update moved aside deleted,
/// as `Program.CleanupFiles` clears what the C#'s updater left. `// C#: Program.cs:616-628`
pub fn cleanup_files() {
    #[cfg(not(target_family = "wasm"))]
    mp_update::install::cleanup(&install_dir());
}

/// How the planner at the program's own place is installed; an archive under the harness's
/// door, whose install is a scratch folder.
fn kind_here() -> Kind {
    #[cfg(not(target_family = "wasm"))]
    if std::env::var_os(INSTALL_DIR_ENV).is_none()
        && let Ok(exe) = std::env::current_exe()
        && mp_update::install::install_kind(&exe) == mp_update::install::InstallKind::Deb
    {
        return Kind::Deb;
    }
    Kind::Archive
}

/// No trial: a web page installs nothing.
#[cfg(target_family = "wasm")]
fn no_trial(_: &Path, _: &Manifest) -> Result<(), String> {
    Ok(())
}

impl Default for Help {
    fn default() -> Self {
        Self::new()
    }
}

impl Help {
    /// The page, with the network, the program's directory and what this build is.
    #[must_use]
    pub fn new() -> Self {
        #[cfg(not(target_family = "wasm"))]
        let mut help = Self::with(
            Arc::new(mp_update::download::Http),
            install_dir(),
            mp_update::identity::this_build(),
        );
        #[cfg(target_family = "wasm")]
        let mut help = Self::with((), install_dir(), None);
        help.kind = kind_here();
        help
    }

    /// The page over another network, directory and build: the tests'.
    #[must_use]
    pub fn with(network: Network, install_dir: PathBuf, build: Option<Identity>) -> Self {
        Self {
            show_console: false,
            flow: Flow::Idle,
            which: Channel::Stable,
            notify_no_update: false,
            check: None,
            offered: None,
            updating: None,
            bar: Bar::default(),
            progress_text: String::new(),
            started: Instant::now(),
            last: "none".to_owned(),
            written: 0,
            restart: None,
            installed: false,
            network,
            install_dir,
            build,
            kind: Kind::Archive,
            #[cfg(not(target_family = "wasm"))]
            downloads: mp_update::install::downloads_dir(),
            #[cfg(target_family = "wasm")]
            downloads: PathBuf::new(),
            #[cfg(not(target_family = "wasm"))]
            trial: mp_update::install::trial,
            #[cfg(target_family = "wasm")]
            trial: no_trial,
            refused: 0,
            opened: None,
            tailscale_link: std::cell::Cell::new(false),
        }
    }

    /// Whether the two update buttons are shown: not in a web page, which is whatever its site
    /// serves - as the C# hides them for a Windows Store install. `// C#: GCSViews/Help.cs:27-31`
    #[must_use]
    pub const fn updates_shown() -> bool {
        cfg!(not(target_family = "wasm"))
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

    /// The check's thread: the channel's manifest fetched and held against this build.
    #[cfg(not(target_family = "wasm"))]
    fn spawn_check(
        &self,
        url: String,
        channel: Channel,
        build: Identity,
    ) -> Option<Receiver<Result<Option<Box<Manifest>>, String>>> {
        use mp_update::check::{Check, check_for_update};
        let (sender, receiver) = channel_pair();
        let network = Arc::clone(&self.network);
        wasm_thread::Builder::new()
            .name("mp-update-check".to_owned())
            .spawn(move || {
                let found = check_for_update(network.as_ref(), &url, channel, &build).map(|check| {
                    match check {
                        Check::UpdateFound(manifest) => Some(manifest),
                        Check::UpToDate => None,
                    }
                });
                let _ = sender.send(found);
            })
            .ok()?;
        Some(receiver)
    }

    #[cfg(target_family = "wasm")]
    fn spawn_check(
        &self,
        _url: String,
        _channel: Channel,
        _build: Identity,
    ) -> Option<Receiver<Result<Option<Box<Manifest>>, String>>> {
        None
    }

    /// `Update.CheckForUpdate(NotifyNoUpdate)` on a thread of its own: nothing for a development
    /// build; else the channel's manifest against this build, and then the question, or "No
    /// update available." when asked to say so.
    /// `// C#: Utilities/Update.cs:118-178; GCSViews/Help.cs:33-48; MainV2.cs:4037-4051`
    pub fn check_for_update(
        &mut self,
        which: Channel,
        notify_no_update: bool,
        persisted: &Persisted,
    ) {
        if self.busy() {
            return;
        }
        let Some(build) = self.build.clone() else {
            return;
        };
        let url = manifest_url(which, &|key| persisted.get(key).map(str::to_owned));
        if let Some(receiver) = self.spawn_check(url, which, build) {
            self.which = which;
            self.notify_no_update = notify_no_update;
            self.flow = Flow::Checking;
            self.check = Some(receiver);
        }
    }

    /// A button's click: the channel checked and "No update available." said when there is
    /// none - or, for a build that is no release or one already updated and waiting for its
    /// restart, only the words for the status line.
    fn clicked(&mut self, which: Channel, persisted: &Persisted) -> Option<&'static str> {
        if self.build.is_none() {
            self.refused += 1;
            return Some(DEVELOPMENT_BUILD);
        }
        if self.installed {
            self.refused += 1;
            return Some(RESTART_TO_FINISH);
        }
        self.check_for_update(which, true, persisted);
        None
    }

    /// `BUT_updatecheck_Click`: the stable channel. `// C#: GCSViews/Help.cs:33-48`
    pub fn update_check_clicked(&mut self, persisted: &Persisted) -> Option<&'static str> {
        self.clicked(Channel::Stable, persisted)
    }

    /// `BUT_betaupdate_Click`: the beta channel. The C# updates at once and asks nothing, and
    /// with Control held goes to its master channel; here the beta is asked about as a release
    /// is, saying what is offered, and Control is nothing - there is no master channel (the
    /// owner, 2026-11-18). `// C#: GCSViews/Help.cs:66-82`
    pub fn beta_update_clicked(&mut self, persisted: &Persisted) -> Option<&'static str> {
        self.clicked(Channel::Beta, persisted)
    }

    /// The update's thread: in place for an archive's install, the new package fetched for the
    /// Debian package's.
    #[cfg(not(target_family = "wasm"))]
    fn spawn_update(&self, manifest: Manifest) -> Option<Updating> {
        use mp_update::install::{fetch_deb, install};
        let (sender, receiver) = channel_pair();
        let cancel = Arc::new(AtomicBool::new(false));
        let cancelled = Arc::clone(&cancel);
        let network = Arc::clone(&self.network);
        let install_dir = self.install_dir.clone();
        let downloads = self.downloads.clone();
        let kind = self.kind;
        let trial = self.trial;
        wasm_thread::Builder::new()
            .name("mp-update".to_owned())
            .spawn(move || {
                let reporter = sender.clone();
                let mut progress = |percent: i32, text: &str| {
                    let _ = reporter.send(Progress::Said(percent, text.to_owned()));
                };
                let cancel_requested = || cancelled.load(Ordering::Relaxed);
                let outcome = match (kind, mp_update::manifest::platform()) {
                    (Kind::Deb, _) => fetch_deb(
                        network.as_ref(),
                        &manifest,
                        &downloads,
                        &mut progress,
                        &cancel_requested,
                    )
                    .map(Done::Package),
                    (Kind::Archive, Some(platform)) => install(
                        network.as_ref(),
                        &manifest,
                        platform,
                        &install_dir,
                        &program(),
                        &trial,
                        &mut progress,
                        &cancel_requested,
                    )
                    .map(|done| Done::Installed(done.files)),
                    (Kind::Archive, None) => Err(NO_RELEASE_HERE.to_owned()),
                };
                let _ = sender.send(Progress::Done(outcome));
            })
            .ok()?;
        Some(Updating {
            progress: receiver,
            cancel,
        })
    }

    #[cfg(target_family = "wasm")]
    fn spawn_update(&self, _manifest: Manifest) -> Option<Updating> {
        None
    }

    /// `Update.DoUpdate`: the "Check for Updates" progress dialog and its worker, on a thread,
    /// for the release the question offered.
    /// `// C#: Utilities/Update.cs:199-217, 684-738`
    pub fn start_update(&mut self) {
        self.flow = Flow::Idle;
        let Some(manifest) = self.offered.take() else {
            return;
        };
        if let Some(updating) = self.spawn_update(manifest) {
            self.flow = Flow::Updating;
            self.bar = Bar::default();
            self.progress_text = "Checking for Updates".to_owned();
            self.started = Instant::now();
            self.updating = Some(updating);
        }
    }

    /// The progress dialog's Cancel: `CancelRequested`, which the worker sees at its next read.
    /// `// C#: Utilities/Update.cs:209, 311-315`
    pub fn cancel_update(&self) {
        if let Some(updating) = &self.updating {
            updating.cancel.store(true, Ordering::Relaxed);
        }
    }

    /// The question's answer: Yes is `DoUpdate`. `// C#: Utilities/Update.cs:190-198`
    pub fn answer(&mut self, yes: bool) {
        if !matches!(self.flow, Flow::Question { .. }) {
            return;
        }
        if yes {
            self.start_update();
        } else {
            self.offered = None;
            self.flow = Flow::Idle;
        }
    }

    /// A message's OK.
    pub fn message_ok(&mut self) {
        if matches!(self.flow, Flow::Message { .. }) {
            self.flow = Flow::Idle;
        }
    }

    /// `MainV2`'s check at startup: once a day on the stable channel, the day kept as
    /// `update_check`; on a day already checked, the beta channel when `beta_updates` is set.
    /// Not for a development build, and not when CONFIG > Planner's "Check for updates at
    /// start" is unticked - a box the C# has not got, the owner's (2026-11-18).
    /// `// C#: MainV2.cs:3668-3678`
    pub fn startup_check(&mut self, persisted: &mut Persisted) {
        if self.build.is_none() || !check_at_start(persisted) {
            return;
        }
        let today = crate::settings::short_date_today();
        if persisted.get(UPDATE_CHECK_KEY) != Some(today.as_str()) {
            self.check_for_update(Channel::Stable, false, persisted);
            persisted.set(UPDATE_CHECK_KEY, today);
        } else if setting_is_true(persisted, BETA_UPDATES_KEY) {
            self.check_for_update(Channel::Beta, false, persisted);
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
                Err(TryRecvError::Empty) => crate::repaint::in_flight(),
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
                    Err(TryRecvError::Empty) => {
                        crate::repaint::in_flight();
                        break;
                    }
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

    /// `CheckForUpdate`'s finding: the question with "BETA " before it for the beta channel and
    /// what is offered after it, "No update available." when asked, or the failure, which the C#
    /// logs ("Update check failed") or boxes (the Help page's Error box) and the status line
    /// says. `// C#: Utilities/Update.cs:180-203; GCSViews/Help.cs:44-47; MainV2.cs:4047-4050`
    fn checked(&mut self, outcome: Result<Option<Box<Manifest>>, String>) -> Option<String> {
        match (outcome, &self.build) {
            (Ok(Some(manifest)), Some(build)) => {
                let extra = if self.which == Channel::Beta {
                    "BETA "
                } else {
                    ""
                };
                self.last = "update found".to_owned();
                // `extra + Strings.UpdateFound + " [link;" + ... + ";ChangeLog]"`: the box draws
                // the markup as `CustomMessageBox` does, a link under the words - to the
                // release's page here. `// C#: Utilities/Update.cs:192-195`
                self.flow = Flow::Question {
                    text: format!(
                        "{extra}{UPDATE_FOUND}: {} [link;{};ChangeLog]",
                        manifest.described_for(build),
                        manifest.notes_url
                    ),
                };
                self.offered = Some(*manifest);
                None
            }
            (Ok(_), _) => {
                self.last = "up to date".to_owned();
                self.flow = if self.notify_no_update {
                    Flow::Message {
                        text: UPDATE_NOT_FOUND.to_owned(),
                    }
                } else {
                    Flow::Idle
                };
                None
            }
            (Err(why), _) => {
                self.last = format!("Update check failed: {why}");
                self.flow = Flow::Idle;
                Some(self.last.clone())
            }
        }
    }

    /// The update's end: installed in place, and the restart; the new package fetched, and where
    /// it is and its command in a box; or "Update Failed " and why.
    /// `// C#: Utilities/Update.cs:60-109`
    fn updated(&mut self, outcome: Result<Done, String>) -> Option<String> {
        match outcome {
            Ok(Done::Installed(files)) => {
                self.written = files;
                self.installed = true;
                self.restart = Some(self.install_dir.join(program()).display().to_string());
                self.last = UPDATE_INSTALLED.to_owned();
                self.flow = Flow::Restarting;
                Some(self.last.clone())
            }
            Ok(Done::Package(package)) => {
                self.last = format!("Downloaded {}", package.display());
                self.flow = Flow::Message {
                    text: package_text(&package),
                };
                Some(self.last.clone())
            }
            Err(why) => {
                self.last = format!("{UPDATE_FAILED}{why}");
                self.flow = Flow::Idle;
                Some(self.last.clone())
            }
        }
    }

    /// The new planner's command, once the update is installed.
    #[cfg(not(target_family = "wasm"))]
    #[must_use]
    pub fn restart_command(&self) -> Option<std::process::Command> {
        if !matches!(self.flow, Flow::Restarting) {
            return None;
        }
        let mut command = std::process::Command::new(self.install_dir.join(program()));
        command.current_dir(&self.install_dir);
        Some(command)
    }

    /// Whether an update has just been installed and the restart is to be decided.
    #[must_use]
    pub const fn restarting(&self) -> bool {
        matches!(self.flow, Flow::Restarting)
    }

    /// The restart decided: the page goes back to idle.
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
                Flow::Question { text } | Flow::Message { text } => text.as_str(),
                _ => "none",
            },
        );
        record("help.progress", &self.progress_text);
        record("help.last", &self.last);
        record("help.written", self.written);
        record("help.refused", self.refused);
        record("help.opened", self.opened.unwrap_or("none"));
        record("help.tailscale", self.tailscale_link.get());
        record("help.restart", self.restart.as_deref().unwrap_or("none"));
        record("help.installed", self.installed);
        record("help.updates", Self::updates_shown());
        record(
            "help.build",
            mp_update::identity::version_line(
                "planner",
                env!("CARGO_PKG_VERSION"),
                self.build.as_ref(),
            ),
        );
    }
}

/// What the box says of a fetched package: where it is, and the command that installs it.
fn package_text(package: &Path) -> String {
    #[cfg(not(target_family = "wasm"))]
    let command = mp_update::install::deb_command(package);
    #[cfg(target_family = "wasm")]
    let command = String::new();
    format!(
        "The new package is in {}.\nClose the planner and install it with:\n{command}",
        package.display()
    )
}

/// A channel pair, typed by use.
fn channel_pair<T>() -> (std::sync::mpsc::Sender<T>, Receiver<T>) {
    channel()
}

/// `/update` and `/updatebeta`: the check and the update without a window - what is found and
/// the update's progress on the console, the new planner started, and the exit code the shell
/// sees. What an earlier update left is cleared first, as at any start; under the harness's door
/// the new planner is named, not started. `tools/release/self-update-test.py` drives this on
/// each platform with a planner built as a release. `// C#: Program.cs:194-205`
#[cfg(not(target_family = "wasm"))]
pub fn update_from_command_line(beta: bool) -> i32 {
    use mp_update::check::{Check, check_for_update};
    use mp_update::install::{downloads_dir, fetch_deb, install, trial};
    let Some(build) = mp_update::identity::this_build() else {
        eprintln!("{DEVELOPMENT_BUILD}");
        return 1;
    };
    cleanup_files();
    let persisted = Persisted::load();
    let which = if beta { Channel::Beta } else { Channel::Stable };
    let url = manifest_url(which, &|key| persisted.get(key).map(str::to_owned));
    let network = mp_update::download::Http;
    println!("{CHECK_FOR_UPDATES}");
    let manifest = match check_for_update(&network, &url, which, &build) {
        Ok(Check::UpdateFound(manifest)) => manifest,
        Ok(Check::UpToDate) => {
            println!("{UPDATE_NOT_FOUND}");
            return 0;
        }
        Err(why) => {
            eprintln!("Update check failed: {why}");
            return 1;
        }
    };
    println!("{UPDATE_FOUND}: {}", manifest.described_for(&build));
    // A line a step, and a download's every tenth percent.
    let mut said = String::new();
    let mut progress = |percent: i32, text: &str| {
        let step = text.split(':').next().unwrap_or(text);
        let line = if percent >= 10 {
            format!("{step}: {}%", percent / 10 * 10)
        } else {
            step.to_owned()
        };
        if line != said {
            println!("{line}");
            said = line;
        }
    };
    let install_dir = install_dir();
    let outcome = match (kind_here(), mp_update::manifest::platform()) {
        (Kind::Deb, _) => fetch_deb(&network, &manifest, &downloads_dir(), &mut progress, &|| false)
            .map(|package| println!("{}", package_text(&package))),
        (Kind::Archive, Some(platform)) => install(
            &network,
            &manifest,
            platform,
            &install_dir,
            &program(),
            &trial,
            &mut progress,
            &|| false,
        )
        .and_then(|done| {
            println!("{UPDATE_INSTALLED}: {} files", done.files);
            let planner = install_dir.join(program());
            if std::env::var_os(INSTALL_DIR_ENV).is_some() {
                println!("The new planner is {}", planner.display());
                return Ok(());
            }
            std::process::Command::new(planner)
                .current_dir(&install_dir)
                .spawn()
                .map(|_| ())
                .map_err(|why| why.to_string())
        }),
        (Kind::Archive, None) => Err(NO_RELEASE_HERE.to_owned()),
    };
    match outcome {
        Ok(()) => 0,
        Err(why) => {
            eprintln!("{UPDATE_FAILED}{why}");
            1
        }
    }
}

/// A web page has no update: it is whatever its site serves.
#[cfg(target_family = "wasm")]
pub fn update_from_command_line(_beta: bool) -> i32 {
    1
}

impl MissionPlanner {
    /// Once a frame: the update's threads, their news on the status line, and what follows an
    /// update installed in place ([`after_install`]) - the new planner started and this one
    /// ended; with a link open, no restart, said on the status line; under the harness's door,
    /// the new planner named.
    pub(crate) fn help_tick(&mut self, cx: &mut Context<Self>) {
        if let Some(status) = self.help.tick() {
            self.file_status = Some(status);
        }
        if !self.help.restarting() {
            return;
        }
        let door = std::env::var_os(INSTALL_DIR_ENV).is_some();
        match after_install(door, self.telemetry.has_link()) {
            AfterInstall::Named => self.help.restart_taken(),
            AfterInstall::Later => {
                self.file_status = Some(INSTALLED_LATER.to_owned());
                self.help.restart_taken();
            }
            AfterInstall::Restart => {
                #[cfg(not(target_family = "wasm"))]
                if let Some(mut command) = self.help.restart_command() {
                    // The settings written before the new planner reads them.
                    self.save_config(crate::settings::SaveEvent::Close);
                    match command.spawn() {
                        // "Quitting existing process"
                        Ok(_) => cx.quit(),
                        Err(why) => self.file_status = Some(format!("{UPDATE_FAILED}{why}")),
                    }
                }
                let _ = &cx;
                self.help.restart_taken();
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
        |this, _window, _cx| {
            if let Some(refused) = this.help.update_check_clicked(&this.persisted) {
                this.file_status = Some(refused.to_owned());
            }
        },
        cx,
    ));
    let beta_update = from_bottom(BETA_UPDATE).child(button(
        "help-betaupdate",
        BETA_UPDATE_TEXT,
        (0.0, 0.0, bw, bh),
        enabled,
        |this, _window, _cx| {
            if let Some(refused) = this.help.beta_update_clicked(&this.persisted) {
                this.file_status = Some(refused.to_owned());
            }
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
    // A box as high as the text's line, up from the Designer's bottom edge: a fixed 13 left the
    // 19-high text 3 pixels below a 920-high window (the layout guard on the owner's Mac,
    // 2026-10-04).
    let tailscale = crate::plugins_ui::native_enabled(
        &this.persisted,
        &crate::plugin_manager::TAILSCALE,
    );
    help.tailscale_link.set(tailscale);
    let change_log = div()
        .absolute()
        .left(px(0.0))
        .bottom(px(PAGE.1 - CHANGE_LOG.1 - 13.0))
        .w(px(PAGE.0))
        .h(px(20.0))
        .child(link_label(
            "help-changelog",
            CHANGE_LOG_TEXT,
            (CHANGE_LOG.0, 0.0),
            true,
            |this, _window, _cx| {
                // `Process.Start(url)`: the desktop's browser.
                match crate::scripts_tab::open_with_shell(Path::new(CHANGE_LOG_URL)) {
                    Ok(()) => this.help.opened = Some(CHANGE_LOG_URL),
                    Err(why) => {
                        this.file_status = Some(format!("could not open the Change Log: {why}"));
                    }
                }
            },
            cx,
        ))
        .children(tailscale.then(|| {
            link_label(
                "help-tailscale",
                TAILSCALE_GUIDE_TEXT,
                (TAILSCALE_GUIDE.0, 0.0),
                true,
                |this, _window, _cx| {
                    match crate::scripts_tab::open_with_shell(Path::new(TAILSCALE_GUIDE_URL)) {
                        Ok(()) => this.help.opened = Some(TAILSCALE_GUIDE_URL),
                        Err(why) => {
                            this.file_status =
                                Some(format!("could not open the Tailscale guide: {why}"));
                        }
                    }
                },
                cx,
            )
        }));
    let _ = window;
    crate::probe::measured("help-body", div())
        .id("help-body")
        .relative()
        .flex_1()
        .min_h(px(PAGE.1))
        .min_w(px(PAGE.0))
        .bg(rgb(theme::BG))
        .child(text_box)
        // Not in a web page: `BUT_betaupdate.Visible = false`. `// C#: GCSViews/Help.cs:27-31`
        .children(Help::updates_shown().then_some(update_check))
        .children(Help::updates_shown().then_some(beta_update))
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
            |this, yes| this.help.answer(yes),
            cx,
        ),
        Flow::Message { text } => {
            let waiting = Waiting {
                text: text.clone(),
                caption: String::new(),
                buttons: None,
            };
            message_box(
                BOXES,
                &waiting,
                window,
                |this| this.help.message_ok(),
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
    use mp_os::fs::FsExt as _;
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
        // No company has contributed to this port yet; Mission Planner's three are its own.
        assert!(HELP_TEXT.contains(
            "Companies that have contributed to MissionPlannerRust's development: none yet, pls get in touch"
        ));
        assert!(!HELP_TEXT.contains("3D Robotics"), "{HELP_TEXT}");
        // The developers, the owner's two lines of 2026-10-14 in place of the C#'s one.
        assert!(HELP_TEXT.contains("\tMain Developer:David Buzz  <davidbuzz@gmail.com>\n"));
        assert!(HELP_TEXT.contains(
            "\tHonourary Developer:Michael Oborne , the OG that this is inspired by/ripped off from.\n"
        ));
        assert!(
            !HELP_TEXT.contains("Main Developer: Michael Oborne"),
            "{HELP_TEXT}"
        );
        // This port's contributors: none yet, the owner's words of 2026-10-14.
        assert!(HELP_TEXT.contains(
            "\tContributions to MissionPlannerRust from: none yet, pls get in touch - https://github.com/davidbuzz/MissionPlannerRust\n"
        ));
        assert!(!HELP_TEXT.contains("Contributions from:"), "{HELP_TEXT}");
        // Further information is this port's page, not Mission Planner's manual (the owner,
        // 2026-10-16).
        assert!(HELP_TEXT.contains(
            "\tPlease visit https://github.com/davidbuzz/MissionPlannerRust for further information.\n"
        ));
        assert!(!HELP_TEXT.contains("ardupilot.org/planner"), "{HELP_TEXT}");
    }

    /// The libraries line names crates this port is built from - each a row of
    /// `THIRD_PARTY_LICENSES`' table - and that file's count of them, so a change of dependencies
    /// that regenerates the file (`cargo xtask licences`) fails here until the line says so too.
    #[test]
    fn the_libraries_line_is_this_ports() {
        const LICENCES: &str = include_str!("../../../THIRD_PARTY_LICENSES");
        let line = HELP_TEXT
            .lines()
            .find(|line| line.starts_with("Libraries: "))
            .expect("the libraries line");
        assert!(!HELP_TEXT.contains("Librarys"), "{HELP_TEXT}");
        let named = line
            .trim_start_matches("Libraries: ")
            .split(" - ")
            .next()
            .unwrap_or_default();
        for name in named.split(", ") {
            // "gpui (Zed Industries)" is the crate gpui; GStreamer is a runtime, not a crate.
            let crate_name = name.split(" (").next().unwrap_or(name).to_lowercase();
            if crate_name == "gstreamer" {
                continue;
            }
            assert!(
                LICENCES.contains(&format!("\n| {crate_name} | ")),
                "{crate_name} is not among THIRD_PARTY_LICENSES' crates"
            );
        }
        let counted = LICENCES
            .lines()
            .find_map(|l| l.split_once(" crates, every target").map(|(n, _)| n))
            .and_then(|n| n.trim().parse::<usize>().ok())
            .expect("THIRD_PARTY_LICENSES' count of crates");
        let said = line
            .split(" - ")
            .nth(1)
            .and_then(|rest| rest.split(" crates in all").next())
            .map(|n| n.replace(',', ""))
            .and_then(|n| n.parse::<usize>().ok())
            .expect("the line's count of crates");
        assert_eq!(
            said, counted,
            "the libraries line's count is THIRD_PARTY_LICENSES'"
        );
    }
    use std::collections::HashMap;

    use mp_update::download::{Body, Source, sha256_hex};

    /// A network of files held in memory.
    struct Server(HashMap<String, Vec<u8>>);

    impl Source for Server {
        fn open(&self, url: &str) -> Result<Body, String> {
            let bytes = self
                .0
                .get(url)
                .cloned()
                .ok_or_else(|| format!("{url}: http status: 404"))?;
            Ok(Body {
                length: Some(bytes.len() as u64),
                reader: Box::new(std::io::Cursor::new(bytes)),
            })
        }
    }

    fn scratch(test: &str) -> PathBuf {
        let dir = mp_os::temp_dir().join(format!("mp-gui-help-{test}-{}", mp_os::process_id()));
        let _ = mp_os::fs::remove_dir_all(&dir);
        mp_os::fs::create_dir_all(&dir).expect("a scratch directory");
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
            wasm_thread::sleep(std::time::Duration::from_millis(5));
        }
    }

    const OLD_COMMIT: &str = "0570c61aa1b2c3d4e5f60718293a4b5c6d7e8f90";
    const NEW_COMMIT: &str = "42992d086a789143682f2997adc737545e3d8c2e";
    const ARCHIVE_URL: &str = "https://x/files/missionplanner-rust_0_0_2.zip";
    const DEB_URL: &str = "https://x/files/missionplanner-rust_0.0.2_amd64.deb";
    const DEB_BYTES: &[u8] = b"!<arch>\ndebian-binary";

    fn build(version: &str, channel: &str, commit: &str, built: &str) -> Option<Identity> {
        Identity::from_values(Some(version), Some(commit), Some(channel), Some(built))
    }

    /// A release's archive: the planner and its licence in the release's own folder. A zip, which
    /// the updater unpacks by the download's name whatever the platform.
    fn archive() -> Vec<u8> {
        let entry = |name: String, data: &[u8]| mp_log::zip::Entry {
            name,
            data: data.to_vec(),
        };
        mp_log::zip::write(
            &[
                entry(format!("missionplanner-rust_0_0_2/{}", program()), b"the new planner"),
                entry("missionplanner-rust_0_0_2/LICENSE".to_owned(), b"new licence"),
            ],
            mp_log::zip::DosTime {
                year: 2026,
                month: 11,
                day: 20,
                hour: 3,
                minute: 10,
                second: 0,
            },
        )
        .expect("a zip")
    }

    /// A channel's manifest: release 0.0.2 at `commit`, with this platform's archive and the
    /// Debian package.
    fn manifest(channel: &str, commit: &str, built: &str) -> Vec<u8> {
        let platform = mp_update::manifest::platform().expect("a platform with releases");
        let asset = |name: &str, url: &str, bytes: &[u8]| {
            format!(
                r#"{{"name": "{name}", "url": "{url}", "size": {}, "sha256": "{}"}}"#,
                bytes.len(),
                sha256_hex(bytes)
            )
        };
        format!(
            r#"{{"schema": 1, "channel": "{channel}", "version": "0.0.2", "tag": "v0_0_2",
                "commit": "{commit}", "built": "{built}",
                "notes_url": "https://x/releases/tag/v0_0_2",
                "assets": {{"{platform}": {}, "linux-x86_64-deb": {}}}}}"#,
            asset("missionplanner-rust_0_0_2.zip", ARCHIVE_URL, &archive()),
            asset("missionplanner-rust_0.0.2_amd64.deb", DEB_URL, DEB_BYTES),
        )
        .into_bytes()
    }

    fn server() -> Arc<Server> {
        Arc::new(Server(HashMap::from([
            (
                "https://x/update.json".to_owned(),
                manifest("stable", NEW_COMMIT, "2026-11-20T03:10:00Z"),
            ),
            (
                "https://x/beta/update.json".to_owned(),
                manifest("beta", NEW_COMMIT, "2026-11-25T02:00:00Z"),
            ),
            (ARCHIVE_URL.to_owned(), archive()),
            (DEB_URL.to_owned(), DEB_BYTES.to_vec()),
        ])))
    }

    const KEYS: [(&str, &str); 2] = [
        ("UpdateManifest", "https://x/update.json"),
        ("BetaUpdateManifest", "https://x/beta/update.json"),
    ];

    fn passes(_: &Path, _: &Manifest) -> Result<(), String> {
        Ok(())
    }

    /// Release 0.0.1, installed from an archive in `dir`, with a new planner that passes its trial.
    fn release_0_0_1(dir: &Path) -> Help {
        mp_os::fs::write(dir.join(program()), b"the old planner").expect("the old planner");
        mp_os::fs::write(dir.join("LICENSE"), b"old licence").expect("the old licence");
        let mut help = Help::with(
            server(),
            dir.to_path_buf(),
            build("0.0.1", "stable", OLD_COMMIT, "2026-10-06T11:00:00Z"),
        );
        help.trial = passes;
        help
    }

    /// A build that is no release - `cargo build`'s - is never updated from one: both buttons
    /// only say so, and nothing is asked of the network at start either.
    #[test]
    fn a_development_build_only_says_what_it_is() {
        let dir = scratch("development");
        let mut help = Help::with(server(), dir.clone(), None);
        let mut persisted = settings(&KEYS);
        persisted.set(BETA_UPDATES_KEY, "True");
        assert_eq!(help.update_check_clicked(&persisted), Some(DEVELOPMENT_BUILD));
        assert_eq!(help.beta_update_clicked(&persisted), Some(DEVELOPMENT_BUILD));
        assert_eq!(help.refused, 2);
        help.startup_check(&mut persisted);
        assert_eq!(persisted.get(UPDATE_CHECK_KEY), None, "no day kept: no check made");
        help.check_for_update(Channel::Beta, true, &persisted);
        help.start_update();
        assert_eq!(help.flow, Flow::Idle);
        assert!(help.check.is_none() && help.updating.is_none());
        assert_eq!(help.written, 0);
        assert!(help.restart.is_none());
        assert!(Help::updates_shown());
        mp_os::fs::remove_dir_all(&dir).expect("cleanup");
    }

    #[test]
    fn a_release_is_asked_about_with_what_is_offered_and_its_change_log() {
        let dir = scratch("question");
        let mut help = release_0_0_1(&dir);
        let persisted = settings(&KEYS);
        assert_eq!(help.update_check_clicked(&persisted), None);
        assert_eq!(help.flow, Flow::Checking);
        assert!(wait(&mut help).is_none());
        // The C#'s words, what is offered after them, and the ChangeLog its box's link
        // (crate::ui::split_link) - to the release's page.
        assert_eq!(
            help.flow,
            Flow::Question {
                text: "Update Found: 0.0.1 to 0.0.2 [link;https://x/releases/tag/v0_0_2;ChangeLog]"
                    .to_owned()
            }
        );
        if let Flow::Question { text } = &help.flow {
            let (words, link) = crate::ui::split_link(text);
            assert_eq!(words, "Update Found: 0.0.1 to 0.0.2 ");
            assert_eq!(
                link,
                Some(crate::ui::BoxLink {
                    url: "https://x/releases/tag/v0_0_2".to_owned(),
                    text: "ChangeLog".to_owned(),
                })
            );
        }
        // No: nothing fetched, nothing changed.
        help.answer(false);
        assert_eq!(help.flow, Flow::Idle);
        assert!(help.offered.is_none());
        assert_eq!(mp_os::fs::read(dir.join(program())).expect("planner"), b"the old planner");

        // Up to date, and asked to say so: "No update available."; the startup check says nothing.
        help.build = build("0.0.2", "stable", NEW_COMMIT, "2026-11-20T03:10:00Z");
        help.update_check_clicked(&persisted);
        wait(&mut help);
        assert_eq!(
            help.flow,
            Flow::Message {
                text: UPDATE_NOT_FOUND.to_owned()
            }
        );
        help.message_ok();
        assert_eq!(help.flow, Flow::Idle);
        help.check_for_update(Channel::Stable, false, &persisted);
        wait(&mut help);
        assert_eq!(help.flow, Flow::Idle);
        assert_eq!(help.last, "up to date");

        // A channel that does not answer: the failure on the status line.
        let broken = settings(&[("UpdateManifest", "https://x/none.json")]);
        help.update_check_clicked(&broken);
        let status = wait(&mut help).expect("a status");
        assert!(status.starts_with("Update check failed: "), "{status}");
        assert_eq!(help.flow, Flow::Idle);
        mp_os::fs::remove_dir_all(&dir).expect("cleanup");
    }

    #[test]
    fn yes_installs_the_release_in_place_and_names_the_new_planner() {
        let dir = scratch("update");
        let mut help = release_0_0_1(&dir);
        let persisted = settings(&KEYS);
        help.update_check_clicked(&persisted);
        wait(&mut help);
        help.answer(true);
        assert_eq!(help.flow, Flow::Updating);
        let status = wait(&mut help);
        assert_eq!(status.as_deref(), Some(UPDATE_INSTALLED));
        assert_eq!(help.flow, Flow::Restarting);
        assert!(help.restarting());
        assert_eq!(help.written, 2);
        // The new files in place, the old ones beside them for the next start to delete.
        assert_eq!(mp_os::fs::read(dir.join(program())).expect("planner"), b"the new planner");
        assert_eq!(mp_os::fs::read(dir.join("LICENSE")).expect("licence"), b"new licence");
        assert_eq!(
            mp_os::fs::read(dir.join(format!("{}.old", program()))).expect("the old planner"),
            b"the old planner"
        );
        assert!(!dir.join(".update").os_exists());
        assert_eq!(
            help.restart.as_deref(),
            Some(dir.join(program()).display().to_string().as_str())
        );
        assert!(help.restart_command().is_some());
        help.restart_taken();
        assert_eq!(help.flow, Flow::Idle);
        // Until it is restarted this is still the old planner: the buttons say so.
        assert_eq!(help.update_check_clicked(&persisted), Some(RESTART_TO_FINISH));
        assert_eq!(help.beta_update_clicked(&persisted), Some(RESTART_TO_FINISH));
        assert_eq!(help.flow, Flow::Idle);
        // The next start.
        assert_eq!(mp_update::install::cleanup(&dir), 2);
        assert!(!dir.join(format!("{}.old", program())).os_exists());
        mp_os::fs::remove_dir_all(&dir).expect("cleanup");
    }

    #[test]
    fn a_new_planner_that_fails_its_trial_is_not_installed() {
        fn refuses(_: &Path, _: &Manifest) -> Result<(), String> {
            Err("the new planner does not start on this computer: exit status: 1".to_owned())
        }
        let dir = scratch("trial");
        let mut help = release_0_0_1(&dir);
        help.trial = refuses;
        let persisted = settings(&KEYS);
        help.update_check_clicked(&persisted);
        wait(&mut help);
        help.answer(true);
        let status = wait(&mut help).expect("a status");
        assert_eq!(
            status,
            "Update Failed the new planner does not start on this computer: exit status: 1"
        );
        assert_eq!(help.flow, Flow::Idle);
        assert!(!help.installed);
        assert_eq!(mp_os::fs::read(dir.join(program())).expect("planner"), b"the old planner");
        assert!(!dir.join(format!("{}.old", program())).os_exists());
        assert!(!dir.join(".update").os_exists());
        mp_os::fs::remove_dir_all(&dir).expect("cleanup");
    }

    #[test]
    fn the_beta_button_asks_about_the_beta_and_only_a_later_one() {
        let dir = scratch("beta");
        let mut help = release_0_0_1(&dir);
        help.build = build("0.0.2", "beta", OLD_COMMIT, "2026-11-24T02:00:00Z");
        let persisted = settings(&KEYS);
        assert_eq!(help.beta_update_clicked(&persisted), None);
        assert_eq!(help.which, Channel::Beta);
        wait(&mut help);
        assert_eq!(
            help.flow,
            Flow::Question {
                text: "BETA Update Found: 0570c61aa1b2 built 2026-11-24 to 42992d086a78 built \
                       2026-11-25 [link;https://x/releases/tag/v0_0_2;ChangeLog]"
                    .to_owned()
            }
        );
        help.answer(false);
        // The beta this build already is: nothing to offer.
        help.build = build("0.0.2", "beta", NEW_COMMIT, "2026-11-25T02:00:00Z");
        help.beta_update_clicked(&persisted);
        wait(&mut help);
        assert_eq!(
            help.flow,
            Flow::Message {
                text: UPDATE_NOT_FOUND.to_owned()
            }
        );
        mp_os::fs::remove_dir_all(&dir).expect("cleanup");
    }

    /// The Debian package's install is the package manager's to replace: the new package is
    /// fetched and checked, and the box says where it is and the command that installs it (the
    /// owner, 2026-11-18). Nothing beside the planner is touched.
    #[test]
    fn the_debian_package_is_fetched_and_its_command_said() {
        let dir = scratch("deb");
        let downloads = dir.join("Downloads");
        mp_os::fs::create_dir_all(&downloads).expect("downloads");
        let mut help = release_0_0_1(&dir);
        help.kind = Kind::Deb;
        help.downloads = downloads.clone();
        let persisted = settings(&KEYS);
        help.update_check_clicked(&persisted);
        wait(&mut help);
        help.answer(true);
        let status = wait(&mut help).expect("a status");
        let package = downloads.join("missionplanner-rust_0.0.2_amd64.deb");
        assert_eq!(status, format!("Downloaded {}", package.display()));
        assert_eq!(mp_os::fs::read(&package).expect("the package"), DEB_BYTES);
        let Flow::Message { text } = &help.flow else {
            panic!("a message: {:?}", help.flow);
        };
        assert!(text.starts_with(&format!("The new package is in {}.", package.display())), "{text}");
        assert!(text.contains("sudo apt install "), "{text}");
        assert!(!help.installed && !help.restarting());
        assert_eq!(mp_os::fs::read(dir.join(program())).expect("planner"), b"the old planner");
        help.message_ok();
        assert_eq!(help.flow, Flow::Idle);
        mp_os::fs::remove_dir_all(&dir).expect("cleanup");
    }

    #[test]
    fn cancel_is_the_users_request() {
        let dir = scratch("cancel");
        let mut help = release_0_0_1(&dir);
        let persisted = settings(&KEYS);
        help.update_check_clicked(&persisted);
        wait(&mut help);
        help.answer(true);
        help.cancel_update();
        let status = wait(&mut help).expect("a status");
        // Cancelled before the swap, or after it had begun: either way the C#'s words.
        assert!(
            status == format!("{UPDATE_FAILED}User Request") || status == UPDATE_INSTALLED,
            "{status}"
        );
        mp_os::fs::remove_dir_all(&dir).expect("cleanup");
    }

    /// What follows an install: under the harness's door the new planner is only named; with a
    /// link open nothing restarts; otherwise the new planner is started.
    #[test]
    fn a_planner_with_a_link_open_does_not_restart_itself() {
        assert_eq!(after_install(false, false), AfterInstall::Restart);
        assert_eq!(after_install(false, true), AfterInstall::Later);
        assert_eq!(after_install(true, false), AfterInstall::Named);
        assert_eq!(after_install(true, true), AfterInstall::Named);
    }

    #[test]
    fn the_console_setting_and_the_daily_check_follow_the_settings() {
        let dir = scratch("daily");
        let mut help = release_0_0_1(&dir);
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

        // CONFIG > Planner's box unticked: no check at start, and no day kept.
        persisted.set(UPDATE_AT_START_KEY, "False");
        help.startup_check(&mut persisted);
        assert_eq!(help.flow, Flow::Idle);
        assert_eq!(persisted.get(UPDATE_CHECK_KEY), None);
        persisted.set(UPDATE_AT_START_KEY, "True");

        // A new day: the stable check, and the day kept.
        help.startup_check(&mut persisted);
        assert_eq!(help.flow, Flow::Checking);
        assert_eq!(help.which, Channel::Stable);
        assert_eq!(
            persisted.get(UPDATE_CHECK_KEY),
            Some(crate::settings::short_date_today().as_str())
        );
        wait(&mut help);
        assert!(matches!(&help.flow, Flow::Question { text } if text.starts_with("Update Found: ")));
        help.answer(false);
        // The same day again: nothing, unless beta updates are on, which checks the beta channel.
        help.startup_check(&mut persisted);
        assert_eq!(help.flow, Flow::Idle);
        persisted.set(BETA_UPDATES_KEY, "True");
        help.startup_check(&mut persisted);
        assert_eq!(help.flow, Flow::Checking);
        assert_eq!(help.which, Channel::Beta);
        wait(&mut help);
        assert!(
            matches!(&help.flow, Flow::Question { text } if text.starts_with("BETA Update Found: "))
        );
        mp_os::fs::remove_dir_all(&dir).expect("cleanup");
    }
}
