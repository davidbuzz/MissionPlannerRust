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

//! Import from Mission Planner: the first start's offer of Mission Planner's settings and logs.
//!
//! Not in the C#. The owner's feature of 2026-10-14 - "a first-time-use offer to import all a
//! user/s logs from 'Mission Planner', and an offer to "import settings from `MissionPlanner`"
//! that copies the config.xml, modifies the local one, and fixes its paths" - made, by his word of
//! 2026-10-15, a native row of the PLUGINS tab like Tailscale's (`plugin_manager::MP_IMPORT`), on
//! until it is unticked. It replaces the silent import of Mission Planner's files that a start
//! finding this application's data directory empty made (`mp_settings::migrate::import`, which
//! only `headless-planner` still runs): without the offer - the row unticked, or Don't ask again -
//! this application starts with its own defaults.
//!
//! **When it is made.** At a start where the row is ticked; this application's `config.xml` is
//! its own, the one in its user data directory, not one `MP_CONFIG_XML` names elsewhere (a test's
//! copy); and either that file held no settings when the start read it - it was missing or empty:
//! the first start - or the offer's setting, [`ANSWER_KEY`], says to ask again. And Mission
//! Planner's user data directory has to be there with a `config.xml`, or a log under its log
//! folder (`mp_settings::migrate::look`).
//!
//! **What it offers**, each on a button of its own. Import settings: Mission Planner's
//! `config.xml` laid over this application's dictionary - every key of Mission Planner's as it is,
//! the keys only this application holds kept - with what then points into Mission Planner's folder
//! pointed into this application's (`mp_settings::rehome`), and its other artefacts, `poi.txt`,
//! `History` and the rest, copied where this application has none
//! (`mp_settings::migrate::import_settings`). Copy logs: the `.tlog`, `.rlog`, `.bin` and `.log`
//! files under Mission Planner's log folder copied into this application's, the folders kept,
//! never over a file already there, on a thread of its own - its progress in the box, its count on
//! the status line when it is done (`mp_settings::migrate::copy_logs`). Not now: asked again at
//! the next start. Don't ask again. Nothing in Mission Planner's folder is written, renamed or
//! deleted.
//!
//! **The settings in this session.** The start-up reads the settings before the offer can be
//! answered, so the import goes into the dictionary and the file at once, and what this session
//! read before it and would write back unasked - the planning screen's boxes, the port and baud -
//! or shows - the quick views, the points of interest - is read again; the rest (the map's
//! provider, the units, the language, the plugins) is read at the next start, which the status line
//! says.
//!
//! **The setting.** [`ANSWER_KEY`] in this application's `config.xml`: absent until the offer is
//! first made; `later` while it is to be made again - Not now, or no answer yet: it is written as
//! the offer shows, so a start that ends unanswered asks again; `logs` while a log copy runs, so
//! one cut short by closing the application is offered again, Copy logs alone; `done` once
//! something came across; `never` for Don't ask again. Saved at once each time, as a plugin saves
//! the settings.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{Receiver, TryRecvError, channel};
use std::time::Duration;

use gpui::{AnyElement, Context, SharedString, Window, div, prelude::*, px, rgb};
use mp_settings::migrate::{self, Directories, Found, LogCopy, SettingsImport};

use crate::MissionPlanner;
use crate::settings::{Persisted, SaveEvent};
use crate::ui::{action, theme};

/// The setting in this application's `config.xml` that remembers the offer's answer
/// ([`Answer`]).
pub(crate) const ANSWER_KEY: &str = "mpimport_offer";

/// The box, by the id a script finds it under.
const BOX: &str = "mpimport-offer";
/// Import settings' id.
const SETTINGS_BUTTON: &str = "mpimport-settings";
/// Copy logs' id.
const LOGS_BUTTON: &str = "mpimport-logs";
/// Not now's id.
const LATER_BUTTON: &str = "mpimport-later";
/// Don't ask again's id.
const NEVER_BUTTON: &str = "mpimport-never";
/// Close's id: the one button left once something was asked for.
const CLOSE_BUTTON: &str = "mpimport-close";

/// Import settings' words.
const IMPORT_SETTINGS: &str = "Import settings";
/// Copy logs' words.
const COPY_LOGS: &str = "Copy logs";
/// Not now's words.
const NOT_NOW: &str = "Not now";
/// Don't ask again's words.
const DONT_ASK_AGAIN: &str = "Don't ask again";
/// Close's words.
const CLOSE: &str = "Close";

/// The line over where Mission Planner's data was found.
const FOUND_IN: &str = "Mission Planner's data was found in";
/// The question, under it.
const ASK: &str = "MissionPlannerRust keeps settings and logs of its own. Bring Mission Planner's across? Its folder is only read: nothing in it is changed or deleted.";
/// Why Copy logs had nowhere to copy to.
const NO_LOG_FOLDER: &str = "no home directory to keep this application's logs under";

/// The box's width: room for a data folder's path on a line.
const WIDTH: f32 = 520.0;

/// How often a copy is looked at while it runs.
const POLL: Duration = Duration::from_millis(100);

/// The most copied logs the `mpimport.logs.files` fact names.
const FILES_NAMED: usize = 20;

/// What [`ANSWER_KEY`] says.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Answer {
    /// Not there, or empty: the offer has not been made.
    Unasked,
    /// To be made again at the next start: Not now, or no answer yet.
    Later,
    /// A log copy runs, or was cut short: Copy logs offered again.
    Logs,
    /// Something came across.
    Done,
    /// Don't ask again.
    Never,
}

impl Answer {
    /// The setting read: a word this application does not write is taken as an answer given.
    #[must_use]
    pub(crate) fn read(value: Option<&str>) -> Self {
        match value {
            None | Some("") => Self::Unasked,
            Some("later") => Self::Later,
            Some("logs") => Self::Logs,
            Some("never") => Self::Never,
            Some(_) => Self::Done,
        }
    }

    /// The word written, and `none` for the setting not there.
    #[must_use]
    pub(crate) const fn word(self) -> &'static str {
        match self {
            Self::Unasked => "none",
            Self::Later => "later",
            Self::Logs => "logs",
            Self::Done => "done",
            Self::Never => "never",
        }
    }
}

/// Whether this start makes the offer, before Mission Planner's folders are looked in: the row
/// ticked, the `config.xml` this application's own, and either a first start - the file held no
/// settings - or an answer that asks again.
#[must_use]
pub(crate) const fn asked_for(
    enabled: bool,
    own_config: bool,
    had_settings: bool,
    answer: Answer,
) -> bool {
    enabled
        && own_config
        && match answer {
            Answer::Unasked => !had_settings,
            Answer::Later | Answer::Logs => true,
            Answer::Done | Answer::Never => false,
        }
}

/// Whether this start's `config.xml` is the application's own, in its user data directory - not
/// one `MP_CONFIG_XML` names elsewhere, as the GUI runners' copies for their scripts are.
fn own_config(persisted: &Persisted) -> bool {
    let own = mp_settings::user_data_directory()
        .map(|directory| directory.join(mp_settings::config::FILE_NAME));
    own.is_some() && persisted.path() == own.as_deref()
}

/// This application's log folder as its recorder finds it: `MP_LOG_DIR`, else the `logdirectory`
/// setting, else `logs` in the user data directory (`Settings.LogDir`).
/// `// C#: ExtLibs/Utilities/Settings.cs:127-158`
fn our_log_directory(config: &mp_settings::Config) -> Option<PathBuf> {
    std::env::var_os("MP_LOG_DIR")
        .map(PathBuf::from)
        .or_else(|| config.log_directory())
        .or_else(mp_settings::default_log_directory)
}

/// The settings half of the offer.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
enum SettingsPart {
    /// Offered, not asked for.
    #[default]
    Offered,
    /// Not offered at this start, which offers a log copy cut short again, Copy logs alone.
    NotOffered,
    /// Imported: what came across.
    Imported(String),
    /// Nothing came across, and why.
    Failed(String),
}

/// The logs half of the offer.
#[derive(Debug, Default)]
enum LogsPart {
    /// Offered, not asked for.
    #[default]
    Offered,
    /// Copying, on its thread.
    Copying(LogJob),
    /// Copied: what the copy did.
    Copied(LogCopy),
    /// Not copied, and why.
    Failed(String),
}

/// A copy's progress, shared with its thread: how many of how many logs are done.
#[derive(Debug, Default)]
struct Progress {
    /// Done.
    done: AtomicUsize,
    /// Found: none until the walk is over.
    of: AtomicUsize,
}

/// A log copy on a thread of its own (`mp_settings::migrate::copy_logs`).
#[derive(Debug)]
struct LogJob {
    /// How far it is, as the thread says.
    progress: Arc<Progress>,
    /// What it did, once.
    receiver: Receiver<LogCopy>,
    /// Mission Planner's log folder.
    from: PathBuf,
    /// This application's.
    to: PathBuf,
}

impl LogJob {
    /// The copy, started on its thread; why not, when the thread would not start.
    fn start(from: PathBuf, to: PathBuf) -> Result<Self, String> {
        let progress = Arc::new(Progress::default());
        let (sender, receiver) = channel();
        let shared = Arc::clone(&progress);
        let (source, target) = (from.clone(), to.clone());
        wasm_thread::Builder::new()
            .name("mp-import-logs".to_owned())
            .spawn(move || {
                let copy = migrate::copy_logs(&source, &target, |done, of| {
                    shared.of.store(of, Ordering::Relaxed);
                    shared.done.store(done, Ordering::Relaxed);
                });
                let _ = sender.send(copy);
            })
            .map(drop)
            .map_err(|err| err.to_string())?;
        Ok(Self {
            progress,
            receiver,
            from,
            to,
        })
    }

    /// How many of how many are done.
    fn progress(&self) -> (usize, usize) {
        (
            self.progress.done.load(Ordering::Relaxed),
            self.progress.of.load(Ordering::Relaxed),
        )
    }
}

/// The offer, and what came of it this session.
#[derive(Debug, Default)]
pub struct MpImport {
    /// What Mission Planner's folders held when this start made the offer; `None` without one.
    found: Option<Found>,
    /// Whether the box shows.
    open: bool,
    /// The settings half.
    settings: SettingsPart,
    /// The logs half.
    logs: LogsPart,
    /// Where Copy logs copies to, once asked.
    logs_to: Option<PathBuf>,
}

impl MpImport {
    /// The offer of what Mission Planner's folders hold. Made again for a log copy cut short
    /// ([`Answer::Logs`]), it offers Copy logs alone.
    fn offered(found: Found, answer: Answer) -> Self {
        Self {
            found: Some(found),
            open: true,
            settings: if answer == Answer::Logs {
                SettingsPart::NotOffered
            } else {
                SettingsPart::Offered
            },
            ..Self::default()
        }
    }

    /// Whether Import settings or Copy logs was asked for: the box then has Close alone.
    fn acted(&self) -> bool {
        !matches!(
            self.settings,
            SettingsPart::Offered | SettingsPart::NotOffered
        ) || !matches!(self.logs, LogsPart::Offered)
    }

    /// Whether the box shows: Tailscale's first-start offer waits for it to be answered, so the
    /// two never stand one over the other.
    pub(crate) const fn is_open(&self) -> bool {
        self.open
    }

    /// Whether a copy runs.
    const fn copying(&self) -> bool {
        matches!(self.logs, LogsPart::Copying(_))
    }

    /// The copy's end, if it has come since the last look: what the status line says of it.
    fn poll(&mut self) -> Option<String> {
        let LogsPart::Copying(job) = &self.logs else {
            return None;
        };
        let copy = match job.receiver.try_recv() {
            Ok(copy) => copy,
            Err(TryRecvError::Empty) => return None,
            Err(TryRecvError::Disconnected) => {
                let why = "the copy stopped without saying what it did".to_owned();
                let said = format!("Mission Planner's logs were not all copied: {why}");
                self.logs = LogsPart::Failed(why);
                return Some(said);
            }
        };
        let said = copied_status(&copy, &job.from, &job.to);
        self.logs = LogsPart::Copied(copy);
        Some(said)
    }

    /// What the copy did, once it is done.
    const fn copied(&self) -> Option<&LogCopy> {
        match &self.logs {
            LogsPart::Copied(copy) => Some(copy),
            _ => None,
        }
    }

    /// What the box says of the settings.
    fn settings_line(&self) -> String {
        match &self.settings {
            SettingsPart::Offered => match self.found.as_ref() {
                Some(found) if !found.settings.is_empty() => {
                    format!("Settings: {}", found.settings.join(", "))
                }
                _ => "Settings: none found".to_owned(),
            },
            SettingsPart::NotOffered => "Settings: offered at an earlier start".to_owned(),
            SettingsPart::Imported(what) => format!("Settings: imported {what}"),
            SettingsPart::Failed(why) => format!("Settings: not imported - {why}"),
        }
    }

    /// What the box says of the logs.
    fn logs_line(&self) -> String {
        match &self.logs {
            LogsPart::Offered => match self.found.as_ref() {
                Some(found) if found.has_logs => format!(
                    "Logs: the .tlog, .rlog, .bin and .log files in {}",
                    found.logs.display()
                ),
                Some(found) => format!("Logs: none found in {}", found.logs.display()),
                None => "Logs: none found".to_owned(),
            },
            LogsPart::Copying(job) => match job.progress() {
                (_, 0) => "Logs: looking for them".to_owned(),
                (done, of) => format!("Logs: copying, {done} of {of}"),
            },
            LogsPart::Copied(copy) => format!("Logs: {}", copied_words(copy)),
            LogsPart::Failed(why) => format!("Logs: not copied - {why}"),
        }
    }

    /// The `mpimport.settings` fact: none, what came across, or why nothing did.
    fn settings_fact(&self) -> String {
        match &self.settings {
            SettingsPart::Offered | SettingsPart::NotOffered => "none".to_owned(),
            SettingsPart::Imported(what) => what.clone(),
            SettingsPart::Failed(why) => format!("failed: {why}"),
        }
    }

    /// The `mpimport.logs` fact: none, copying, or how many were copied.
    fn logs_fact(&self) -> String {
        match &self.logs {
            LogsPart::Offered => "none".to_owned(),
            LogsPart::Copying(_) => "copying".to_owned(),
            LogsPart::Copied(copy) => copy.copied.len().to_string(),
            LogsPart::Failed(why) => format!("failed: {why}"),
        }
    }
}

/// "it was" for one, "they were" for more.
const fn left_as(count: usize) -> &'static str {
    if count == 1 { "it was" } else { "they were" }
}

/// How many logs a copy copied, and how many were there already or could not be copied.
fn copied_words(copy: &LogCopy) -> String {
    let mut words = format!("{} copied", copy.copied.len());
    if copy.already > 0 {
        let _ = write!(
            words,
            ", {} already there, left as {}",
            copy.already,
            left_as(copy.already)
        );
    }
    if !copy.failed.is_empty() {
        let _ = write!(words, ", {} could not be copied", copy.failed.len());
    }
    words
}

/// What the status line says when a copy ends.
fn copied_status(copy: &LogCopy, from: &Path, to: &Path) -> String {
    let count = copy.copied.len();
    let mut said = format!(
        "Copied {count} {} from Mission Planner's {} into {}",
        if count == 1 { "log" } else { "logs" },
        from.display(),
        to.display()
    );
    if copy.already > 0 {
        let _ = write!(
            said,
            " ({} already there, left as {})",
            copy.already,
            left_as(copy.already)
        );
    }
    if let Some(first) = copy.failed.first() {
        let _ = write!(
            said,
            "; {} could not be copied, the first {first}",
            copy.failed.len()
        );
    }
    said
}

/// What the status line says of Import settings.
fn imported_status(outcome: &SettingsImport, from: &Path) -> String {
    let mut said = if outcome.names.is_empty() {
        format!("Nothing was imported from Mission Planner's {}", from.display())
    } else {
        format!(
            "Imported Mission Planner's {} from {} - every screen uses them from the next start",
            outcome.summary(),
            from.display()
        )
    };
    if let Some(first) = outcome.failed.first() {
        let _ = write!(
            said,
            "; {} not imported, the first {first}",
            outcome.failed.len()
        );
    }
    said
}

/// The copied logs the `mpimport.logs.files` fact names: the first [`FILES_NAMED`] of them, `/`
/// between folders on every platform.
fn files_named(copy: &LogCopy) -> String {
    if copy.copied.is_empty() {
        return "none".to_owned();
    }
    let mut named: Vec<String> = copy
        .copied
        .iter()
        .take(FILES_NAMED)
        .map(|path| path.to_string_lossy().replace('\\', "/"))
        .collect();
    if copy.copied.len() > FILES_NAMED {
        named.push(format!("and {} more", copy.copied.len() - FILES_NAMED));
    }
    named.join(", ")
}

impl MissionPlanner {
    /// The offer at start, made when it is due (the module's comment): called once the start-up
    /// has read the settings and saved them. Its setting is written `later` as it shows, and saved,
    /// so a start that ends before it is answered asks again.
    pub(crate) fn mp_import_start(&mut self) {
        let answer = Answer::read(self.persisted.get(ANSWER_KEY));
        let enabled = crate::plugins_ui::native_enabled(
            &self.persisted,
            &crate::plugin_manager::MP_IMPORT,
        );
        if !asked_for(
            enabled,
            own_config(&self.persisted),
            self.persisted.had_settings(),
            answer,
        ) {
            return;
        }
        let Some(found) = mp_settings::Folders::from_environment()
            .and_then(|folders| migrate::look(&Directories::of(&folders)))
        else {
            return;
        };
        // A copy cut short is offered again while there are logs to copy, and then no more.
        let offered = if answer == Answer::Logs {
            found.has_logs
        } else {
            found.anything()
        };
        if !offered {
            if answer == Answer::Logs {
                self.mp_import_answer(Answer::Done);
            }
            return;
        }
        self.mp_import = MpImport::offered(found, answer);
        if answer != Answer::Logs {
            self.mp_import_answer(Answer::Later);
        }
    }

    /// The offer's setting written and, when that changed it, saved at once, as a plugin's
    /// `Settings.Instance.Save()` saves.
    fn mp_import_answer(&mut self, answer: Answer) {
        if self.persisted.get(ANSWER_KEY) != Some(answer.word()) {
            self.persisted.set(ANSWER_KEY, answer.word());
            self.save_config(SaveEvent::Plugin);
        }
    }

    /// Import settings: Mission Planner's files read and copied, its `config.xml` laid over this
    /// application's dictionary, what the start-up read of the settings read again, and the file
    /// saved at once; what came across on the status line.
    fn mp_import_settings(&mut self) {
        let Some(folders) = mp_settings::Folders::from_environment() else {
            self.mp_import.settings =
                SettingsPart::Failed("no home directory to find Mission Planner's under".to_owned());
            return;
        };
        let directories = Directories::of(&folders);
        let outcome = migrate::import_settings(&directories);
        self.persisted.import_mission_planner(
            outcome.config.as_ref(),
            Some(&folders),
            outcome.summary(),
        );
        if outcome.config.is_some() {
            self.mp_import_read_again();
        }
        if outcome
            .names
            .iter()
            .any(|name| name == crate::poi::FILE_NAME)
        {
            self.fly_data.pois = crate::poi::Pois::load_default();
        }
        self.mp_import.settings = match outcome.failed.first() {
            Some(why) if outcome.names.is_empty() => SettingsPart::Failed(why.clone()),
            _ => SettingsPart::Imported(outcome.summary()),
        };
        self.file_status = Some(imported_status(&outcome, &directories.from_user));
        let answer = if self.mp_import.copying() {
            Answer::Logs
        } else {
            Answer::Done
        };
        // Saved even when the setting was already this, for the import's sake.
        self.persisted.set(ANSWER_KEY, answer.word());
        self.save_config(SaveEvent::Plugin);
    }

    /// What the start-up read of the settings, read again from the imported ones where this
    /// session would otherwise write back what it read before them, or show it: the planning
    /// screen's home, boxes and altitude frame - which leaving that screen saves - and its
    /// `UseMissionMAVFTP` box; the planned home `CurrentState` keeps; the quick views; the
    /// connection box's port and baud; and the Planner page, when it shows. The rest - the map's
    /// provider, the units, the language, the plugins - is read at the next start.
    /// `// C#: MainV2.cs:963-977, 1012-1030; GCSViews/FlightPlanner.cs:2592-2612;
    /// GCSViews/FlightData.cs:459-496`
    fn mp_import_read_again(&mut self) {
        let home = crate::plan::planned_home_from_config(Some(self.persisted.config()));
        self.plan.set_planned_home(home);
        mp_vehicle::VehicleState::set_planned_home(mp_vehicle::LatLngAlt {
            lat: home.lat,
            lng: home.lng,
            alt: home.alt,
        });
        self.plan.apply_panel_config(Some(self.persisted.config()));
        self.plan.apply_mavftp_config(Some(self.persisted.config()));
        if let Some(frame) = self.persisted.altitude_frame() {
            self.altitude_frame = frame;
        }
        if self.screen == crate::Screen::Plan {
            crate::plan::activate(self);
        }
        self.persisted.restore_quick_views(&mut self.fly_data.quick);
        self.connect_box = crate::connect::ConnectBox::new(
            self.persisted.get("comport").unwrap_or_default(),
            self.persisted.baud(),
        );
        if self.planner.is_active() {
            self.planner_activate();
        }
    }

    /// Copy logs: Mission Planner's log folder copied on a thread of its own ([`LogJob`]) into
    /// this application's, as the settings name it when the button is pressed - after Import
    /// settings, the imported `logdirectory` pointed home - and looked at from here until it ends;
    /// the offer's setting `logs` meanwhile, so a copy cut short by closing the application is
    /// offered again.
    fn mp_import_logs(&mut self, cx: &mut Context<Self>) {
        let Some(from) = self.mp_import.found.as_ref().map(|found| found.logs.clone()) else {
            return;
        };
        let Some(to) = our_log_directory(self.persisted.config()) else {
            self.mp_import.logs = LogsPart::Failed(NO_LOG_FOLDER.to_owned());
            return;
        };
        match LogJob::start(from, to.clone()) {
            Ok(job) => self.mp_import.logs = LogsPart::Copying(job),
            Err(why) => {
                self.file_status = Some(format!("Mission Planner's logs were not copied: {why}"));
                self.mp_import.logs = LogsPart::Failed(why);
                return;
            }
        }
        self.mp_import.logs_to = Some(to);
        self.mp_import_answer(Answer::Logs);
        cx.spawn(async move |this, cx| {
            loop {
                cx.background_executor().timer(POLL).await;
                let ended = this.update(cx, |this, cx| {
                    let ended = this.mp_import_poll();
                    cx.notify();
                    ended
                });
                if ended.unwrap_or(true) {
                    break;
                }
            }
        })
        .detach();
    }

    /// The copy's end, if it has come: said on the status line, and the offer's setting `done`,
    /// saved. True once there is nothing more to look at.
    fn mp_import_poll(&mut self) -> bool {
        match self.mp_import.poll() {
            Some(said) => {
                self.file_status = Some(said);
                self.mp_import_answer(Answer::Done);
                true
            }
            None => !self.mp_import.copying(),
        }
    }
}

/// One line of the box's words, held to its width so that a long one wraps.
fn line(text: String, colour: u32) -> gpui::Div {
    div()
        .w_full()
        .min_w_0()
        .text_sm()
        .text_color(rgb(colour))
        .child(text)
}

/// One half of the offer: what it says, and its button at the right.
fn half(text: String, button: AnyElement) -> gpui::Div {
    div()
        .w_full()
        .min_w_0()
        .flex()
        .items_center()
        .justify_between()
        .gap_2()
        .child(
            div()
                .flex_1()
                .min_w_0()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(text),
        )
        .child(button)
}

/// Import settings: live while the settings are offered and something of them is there.
fn settings_button(state: &MpImport, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let enabled = state.settings == SettingsPart::Offered
        && state
            .found
            .as_ref()
            .is_some_and(|found| !found.settings.is_empty());
    action(
        SETTINGS_BUTTON,
        IMPORT_SETTINGS,
        theme::ACCENT,
        enabled,
        cx.listener(|this, _event: &(), _window, cx| {
            this.mp_import_settings();
            cx.notify();
        }),
    )
}

/// Copy logs: live while the logs are offered and there are some to copy.
fn logs_button(state: &MpImport, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let enabled = matches!(state.logs, LogsPart::Offered)
        && state.found.as_ref().is_some_and(|found| found.has_logs);
    action(
        LOGS_BUTTON,
        COPY_LOGS,
        theme::ACCENT,
        enabled,
        cx.listener(|this, _event: &(), _window, cx| {
            this.mp_import_logs(cx);
            cx.notify();
        }),
    )
}

/// Not now and Don't ask again; or, once Import settings or Copy logs was asked for, Close - a
/// copy then goes on to its end with the box closed.
fn answer_buttons(state: &MpImport, cx: &mut Context<MissionPlanner>) -> Vec<AnyElement> {
    if state.acted() {
        return vec![action(
            CLOSE_BUTTON,
            CLOSE,
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.mp_import.open = false;
                cx.notify();
            }),
        )];
    }
    vec![
        action(
            LATER_BUTTON,
            NOT_NOW,
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                // The setting already asks again: `later`, or `logs` for a copy cut short.
                this.mp_import.open = false;
                cx.notify();
            }),
        ),
        action(
            NEVER_BUTTON,
            DONT_ASK_AGAIN,
            theme::DIM,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.mp_import.open = false;
                this.mp_import_answer(Answer::Never);
                cx.notify();
            }),
        ),
    ]
}

/// The offer over whatever screen shows, while it is open: where Mission Planner's data was
/// found, the settings and the logs each with its button, and the answers.
pub fn overlay(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let state = &this.mp_import;
    if !state.open {
        return None;
    }
    let found = state.found.as_ref()?;
    let size = window.viewport_size();
    let dialog = crate::probe::measured(BOX, div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(WIDTH))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::WARN))
        .rounded_md()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(crate::plugin_manager::MP_IMPORT.name),
        )
        .child(line(FOUND_IN.to_owned(), theme::TEXT))
        .child(line(found.from.display().to_string(), theme::ACCENT))
        .child(line(ASK.to_owned(), theme::DIM))
        .child(half(state.settings_line(), settings_button(state, cx)))
        .child(half(state.logs_line(), logs_button(state, cx)))
        .child(
            div()
                .w_full()
                .min_w_0()
                .flex()
                .gap_2()
                .justify_end()
                .children(answer_buttons(state, cx)),
        );
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(
                    div()
                        .id(SharedString::from(format!("{BOX}-backdrop")))
                        .w(size.width)
                        .h(size.height)
                        .flex()
                        .items_center()
                        .justify_center()
                        .occlude()
                        .child(dialog),
                ),
        )
        .with_priority(2)
        .into_any_element(),
    )
}

/// The facts a UI test asserts on: whether the offer shows, its setting, where Mission Planner's
/// data was found, what each half did, and where the logs went.
pub fn record_facts(state: &MpImport, persisted: &Persisted) {
    use crate::facts::record;
    record("mpimport.offer", if state.open { "shown" } else { "none" });
    record(
        "mpimport.answer",
        persisted.get(ANSWER_KEY).unwrap_or("none"),
    );
    record(
        "mpimport.found",
        state
            .found
            .as_ref()
            .map_or_else(|| "none".to_owned(), |found| found.from.display().to_string()),
    );
    record("mpimport.settings", state.settings_fact());
    record("mpimport.logs", state.logs_fact());
    let copied = state.copied();
    record(
        "mpimport.logs.already",
        copied.map_or(0, |copy| copy.already),
    );
    record(
        "mpimport.logs.files",
        copied.map_or_else(|| "none".to_owned(), files_named),
    );
    record(
        "mpimport.logs.to",
        state
            .logs_to
            .as_ref()
            .map_or_else(|| "none".to_owned(), |to| to.display().to_string()),
    );
}

#[cfg(test)]
mod tests {
    use mp_os::fs::FsExt as _;
    use super::*;

    /// A directory of its own for one test, removed afterwards.
    struct Scratch(PathBuf);

    impl Scratch {
        fn new(name: &str) -> Self {
            let path =
                mp_os::temp_dir().join(format!("mp-gui-mp-import-{name}-{}", mp_os::process_id()));
            let _ = mp_os::fs::remove_dir_all(&path);
            mp_os::fs::create_dir_all(&path).expect("scratch directory");
            Self(path)
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = mp_os::fs::remove_dir_all(&self.0);
        }
    }

    /// Writes a file, making its folder.
    fn put(path: &Path, bytes: &[u8]) {
        mp_os::fs::create_dir_all(path.parent().expect("a folder")).expect("folder");
        mp_os::fs::write(path, bytes).expect("written");
    }

    /// What Mission Planner's folders hold on a machine that has flown.
    fn found() -> Found {
        let from = PathBuf::from("/home/pilot/.local/share/Mission Planner");
        Found {
            config: Some(from.join("config.xml")),
            settings: vec!["config.xml".to_owned(), "poi.txt".to_owned()],
            logs: from.join("logs"),
            has_logs: true,
            from,
        }
    }

    /// The first start asks; a start with settings of its own was never asked and is not - an
    /// existing user's; Not now and a copy cut short ask again, settings or not; an answer given
    /// does not; and neither the row unticked nor a `config.xml` named elsewhere ever does.
    #[test]
    fn the_offer_is_made_at_a_first_start_or_when_it_was_put_off() {
        assert!(asked_for(true, true, false, Answer::Unasked));
        assert!(!asked_for(true, true, true, Answer::Unasked));
        assert!(asked_for(true, true, true, Answer::Later));
        assert!(asked_for(true, true, false, Answer::Later));
        assert!(asked_for(true, true, true, Answer::Logs));
        assert!(!asked_for(true, true, false, Answer::Done));
        assert!(!asked_for(true, true, false, Answer::Never));
        assert!(!asked_for(true, true, true, Answer::Never));
        for answer in [Answer::Unasked, Answer::Later, Answer::Logs] {
            assert!(!asked_for(false, true, false, answer), "{answer:?}");
            assert!(!asked_for(true, false, false, answer), "{answer:?}");
        }
    }

    #[test]
    fn the_answer_reads_back_as_written() {
        for answer in [Answer::Later, Answer::Logs, Answer::Done, Answer::Never] {
            assert_eq!(Answer::read(Some(answer.word())), answer);
        }
        assert_eq!(Answer::read(None), Answer::Unasked);
        assert_eq!(Answer::read(Some("")), Answer::Unasked);
        assert_eq!(Answer::Unasked.word(), "none");
        // A word this application never writes is an answer given, not a question to ask.
        assert_eq!(Answer::read(Some("yes")), Answer::Done);
    }

    /// The row is named as the owner named it, and the scripts untick it and set the offer's
    /// setting by the names this module uses.
    #[test]
    fn the_row_and_the_setting_are_the_ones_the_scripts_name() {
        assert_eq!(crate::plugin_manager::MP_IMPORT.name, "Import from Mission Planner");
        assert_eq!(crate::plugin_manager::MP_IMPORT.file, "mpimport.native");
        let disabled = include_str!("../../../tests/gui/mp-import-disabled.gui");
        assert!(disabled.contains("<DisabledPlugins>mpimport.native</DisabledPlugins>"));
        assert!(disabled.contains(&format!("<{ANSWER_KEY}>later</{ANSWER_KEY}>")));
    }

    /// Every button the box has is clicked by a script, by its id.
    #[test]
    fn the_scripts_click_the_offers_buttons() {
        let both = include_str!("../../../tests/gui/mp-import.gui");
        let later = include_str!("../../../tests/gui/mp-import-later.gui");
        let settings = include_str!("../../../tests/gui/config-import.gui");
        for (script, id) in [
            (both, SETTINGS_BUTTON),
            (both, LOGS_BUTTON),
            (both, CLOSE_BUTTON),
            (settings, SETTINGS_BUTTON),
            (later, LATER_BUTTON),
            (later, NEVER_BUTTON),
        ] {
            assert!(script.contains(&format!("\nclick {id}\n")), "{id}");
        }
        for script in [both, later, settings] {
            assert!(script.contains("\nexpect mpimport.offer shown\n"));
        }
    }

    /// The box says what was found, offers what is there and says what came of it; a copy cut
    /// short, offered again, offers Copy logs alone.
    #[test]
    fn the_box_says_what_was_found_and_what_came_of_it() {
        let found = found();
        let offer = MpImport::offered(found.clone(), Answer::Unasked);
        assert!(offer.open);
        assert!(!offer.acted());
        assert_eq!(offer.settings_line(), "Settings: config.xml, poi.txt");
        assert_eq!(
            offer.logs_line(),
            format!(
                "Logs: the .tlog, .rlog, .bin and .log files in {}",
                found.logs.display()
            )
        );
        assert_eq!(offer.settings_fact(), "none");
        assert_eq!(offer.logs_fact(), "none");

        let again = MpImport::offered(found.clone(), Answer::Logs);
        assert_eq!(again.settings, SettingsPart::NotOffered);
        assert_eq!(again.settings_line(), "Settings: offered at an earlier start");
        assert!(!again.acted());

        let bare = MpImport::offered(
            Found {
                config: None,
                settings: Vec::new(),
                has_logs: false,
                ..found.clone()
            },
            Answer::Later,
        );
        assert_eq!(bare.settings_line(), "Settings: none found");
        assert_eq!(
            bare.logs_line(),
            format!("Logs: none found in {}", found.logs.display())
        );

        let mut done = MpImport::offered(found, Answer::Unasked);
        done.settings = SettingsPart::Imported("config.xml, poi.txt".to_owned());
        assert!(done.acted());
        assert_eq!(done.settings_line(), "Settings: imported config.xml, poi.txt");
        assert_eq!(done.settings_fact(), "config.xml, poi.txt");
        done.settings = SettingsPart::Failed("config.xml: not well-formed".to_owned());
        assert_eq!(done.settings_fact(), "failed: config.xml: not well-formed");
    }

    /// What the status line says of Import settings: what came across and from where, and what
    /// did not.
    #[test]
    fn the_status_line_says_what_the_settings_import_brought() {
        let from = Path::new("/home/pilot/.local/share/Mission Planner");
        let outcome = SettingsImport {
            config: None,
            names: vec!["config.xml".to_owned(), "History".to_owned()],
            failed: vec!["poi.txt: denied".to_owned()],
        };
        assert_eq!(
            imported_status(&outcome, from),
            format!(
                "Imported Mission Planner's config.xml, History from {} - every screen uses them from the next start; 1 not imported, the first poi.txt: denied",
                from.display()
            )
        );
        assert_eq!(
            imported_status(&SettingsImport::default(), from),
            format!("Nothing was imported from Mission Planner's {}", from.display())
        );
    }

    /// Copy logs runs on a thread of its own: the box's line follows it, its end is said once for
    /// the status line, the log already here is left as it is, the folders are kept and a file
    /// that is not a log stays behind.
    #[test]
    fn a_copy_runs_on_its_own_thread_and_says_what_it_did() {
        let scratch = Scratch::new("copy");
        let from = scratch.0.join("Mission Planner").join("logs");
        let to = scratch.0.join("MissionPlannerRust").join("logs");
        put(&from.join("QUADROTOR").join("1").join("a.tlog"), b"theirs");
        put(&from.join("QUADROTOR").join("1").join("a.rlog"), b"raw");
        put(&from.join("HELI").join("00000001.BIN"), b"df");
        put(&from.join("notes.txt"), b"notes");
        put(&to.join("QUADROTOR").join("1").join("a.tlog"), b"ours");
        let mut state = MpImport {
            found: Some(found()),
            open: true,
            logs: LogsPart::Copying(LogJob::start(from.clone(), to.clone()).expect("started")),
            logs_to: Some(to.clone()),
            ..MpImport::default()
        };
        assert!(state.acted());
        assert!(state.copying());
        assert_eq!(state.logs_fact(), "copying");
        assert!(state.logs_line().starts_with("Logs: "));
        let deadline = web_time::Instant::now() + Duration::from_secs(10);
        let said = loop {
            if let Some(said) = state.poll() {
                break said;
            }
            assert!(web_time::Instant::now() < deadline, "the copy never ended");
            wasm_thread::sleep(Duration::from_millis(5));
        };
        assert_eq!(
            said,
            format!(
                "Copied 2 logs from Mission Planner's {} into {} (1 already there, left as it was)",
                from.display(),
                to.display()
            )
        );
        assert!(!state.copying());
        assert_eq!(state.poll(), None);
        assert_eq!(state.logs_fact(), "2");
        assert_eq!(
            state.logs_line(),
            "Logs: 2 copied, 1 already there, left as it was"
        );
        assert_eq!(
            state.copied().map(files_named).as_deref(),
            Some("HELI/00000001.BIN, QUADROTOR/1/a.rlog")
        );
        assert_eq!(state.copied().map(|copy| copy.already), Some(1));
        assert_eq!(
            mp_os::fs::read(to.join("QUADROTOR").join("1").join("a.tlog")).expect("ours"),
            b"ours"
        );
        assert_eq!(
            mp_os::fs::read(to.join("HELI").join("00000001.BIN")).expect("copied"),
            b"df"
        );
        assert!(!to.join("notes.txt").os_exists());
        assert_eq!(
            mp_os::fs::read(from.join("QUADROTOR").join("1").join("a.tlog")).expect("theirs"),
            b"theirs"
        );
    }

    /// The fact names twenty copied logs at most, `/` between folders.
    #[test]
    fn the_files_fact_names_twenty_at_most() {
        let copy = LogCopy {
            found: 25,
            copied: (0..25).map(|n| PathBuf::from("QUADROTOR").join(format!("{n}.tlog"))).collect(),
            already: 0,
            failed: Vec::new(),
        };
        let named = files_named(&copy);
        assert!(named.starts_with("QUADROTOR/0.tlog, QUADROTOR/1.tlog, "), "{named}");
        assert!(named.ends_with(", QUADROTOR/19.tlog, and 5 more"), "{named}");
        assert_eq!(files_named(&LogCopy::default()), "none");
        assert_eq!(copied_words(&copy), "25 copied");
    }
}
