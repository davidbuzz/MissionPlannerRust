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

//! The Log Downloader: `Log/LogDownloadMavLink.cs`, the window the flight screen's DataFlash Logs
//! page opens with Download DataFlash Log Via Mavlink.
//!
//! The form, as its Designer lays it out: "Log files:" over `CHK_logs`, a checked list of every
//! log the vehicle lists with its date and size; under it the buttons - Download All Logs,
//! Download Selected Logs and Clear Logs in the first column, First Person KML, Create KML and
//! .bin to .log in the second; under those the progress bar and its byte count. On the right,
//! "Output:" over `TXT_seriallog`, where each step says what it did, and the note about posting
//! the `.bin`. `Show()`, not `ShowDialog()`: the rest of the application stays usable while it
//! is open, and a download carries on when it is closed.
//! `// C#: Log/LogDownloadMavLink.cs, Log/LogDownloadMavLink.Designer.cs:116-180, Log/LogDownloadMavLink.resx`
//!
//! The second column holds the log tools. Create KML and First Person KML ask for a log (`*.log`),
//! say in the output that they process it, read it line by line as text whatever its name, and
//! write what `LogOutput` writes ([`mp_kml::dflog`]) - Create KML through the writer of the
//! DataFlash page's Create KML + gpx, First Person KML `writeKMLFirstPerson`'s tour as
//! `<name>-fp.kmz` - then say "Done". .bin to .log asks for a DataFlash log, then where to save
//! its text, and converts it as the DataFlash page's Convert .Bin to .Log does
//! ([`mp_log::convert`]). Clear Logs - which `LoadLogList` allows only with a link open - asks "Are
//! you sure?", sends `LOG_ERASE` twice ([`mp_link::commands::log_erase`]), says to allow for the
//! erase and empties the list.
//! `// C#: Log/LogDownloadMavLink.cs:450-514, 525-600`
//!
//! Where these differ from the C#:
//! - the file dialogs are this application's: a typed path over the list of the folder it names
//!   ([`crate::config::firmware::path_box`]), one file each, where the C#'s Create KML and First
//!   Person KML take several (`Multiselect`); a Save As asks nothing about a file already there;
//! - a tool works on a thread of its own and the window goes on, where the C#'s handler holds the
//!   form until it is done; the tools wait while one works;
//! - what a handler throws, which the form does not catch, is said on the status line as the C#'s
//!   error box says it; .bin to .log, which says nothing, says there where its `.log` went.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use web_time::Instant;

use gpui::{AnyElement, Context, Div, KeyDownEvent, Window, div, prelude::*, px, rgb};
use mp_firmware::flow::Buttons;
use mp_ftp::logs::LogListing;
use mp_kml::dflog::LogOutput;

use crate::MissionPlanner;
use crate::config::firmware::{BoxIds, PathBox};
use crate::fly::error_box;
use crate::textfield::KeyOutcome;
use crate::ui::{action, progress, theme};

/// `LogStrings`, the words the downloader says. `// C#: Log/LogStrings.resx`
pub mod strings {
    /// `LogStrings.NotConnected`.
    pub const NOT_CONNECTED: &str = "Please connect your drone";
    /// `LogStrings.FetchingLogfileList`.
    pub const FETCHING_LIST: &str = "Getting list of log files...";
    /// `LogStrings.SomeLogsFound`, with `{0}`.
    pub const SOME_LOGS: &str = "Found {0} log files, note: item sizes are just an estimate.";
    /// `LogStrings.FetchingLog`, with `{0}`.
    pub const FETCHING_LOG: &str = "Fetching log file {0} ...";
    /// `LogStrings.DownloadStarting`, with `{0}`.
    pub const DOWNLOAD_STARTING: &str = "Downloading log files to: {0}";
    /// `LogStrings.NothingSelected`.
    pub const NOTHING_SELECTED: &str =
        "Please select some logs by checking the check boxes in the list";
    /// `LogStrings.CancelDownload`.
    pub const CANCEL_DOWNLOAD: &str = "You are downloading a log file, do you want to cancel?";
    /// What `DownloadThread` says at the end. `// C#: Log/LogDownloadMavLink.cs:356`
    pub const COMPLETE: &str = "Download complete.";
    /// `Log_Load`'s warning to an armed vehicle. `// C#: Log/LogDownloadMavLink.cs:55-56`
    pub const DISARM: &str = "Please disarm the drone before downloading logs!";
    /// `LabelStatus.Text`. `// C#: Log/LogDownloadMavLink.resx`
    pub const NOTE: &str = "NOTE: When posting support querys, please send the .bin file";
    /// `LogStrings.Confirmation`, Clear Logs' question.
    pub const CONFIRMATION: &str = "Are you sure?";
    /// The caption of Clear Logs' question. `// C#: Log/LogDownloadMavLink.cs:452`
    pub const SURE: &str = "sure";
    /// `LogStrings.EraseComplete`.
    pub const ERASE_COMPLETE: &str = "!!Allow 30-90 seconds for erase";
    /// `LogStrings.ProcessingLog`, with `{0}`.
    pub const PROCESSING: &str = "Processing {0}";
    /// `LogStrings.ErrorProcessingLogfile`.
    pub const ERROR_PROCESSING: &str = "Error processing file. Make sure the file is not in use";
    /// `LogStrings.Done`.
    pub const DONE: &str = "Done";
}

/// Create KML's and First Person KML's `OpenFileDialog.Filter`.
/// `// C#: Log/LogDownloadMavLink.cs:473, 529`
pub const LOG_FILTER: &str = "*.log|*.log";

/// .bin to .log's `OpenFileDialog.Filter`. `// C#: Log/LogDownloadMavLink.cs:581`
pub const BIN_FILTER: &str = "Binary Log|*.bin;*.BIN";

/// .bin to .log's `SaveFileDialog.Filter`, whose `log` its `AddExtension` puts on a name typed
/// without an extension. `// C#: Log/LogDownloadMavLink.cs:589`
pub const SAVE_FILTER: &str = "log|*.log;*.LOG";

/// The extension `AddExtension` puts on .bin to .log's Save As: its filter's first.
const SAVE_EXTENSION: &str = "log";

/// The window's boxes, for a script to click them by.
const IDS: BoxIds = BoxIds {
    question: "fly-logs-question",
    yes: "fly-logs-question-yes",
    no: "fly-logs-question-no",
    message: "fly-logs-message",
    ok: "fly-logs-message-ok",
    path: "fly-logs-path",
    path_value: "fly-logs-path-value",
    path_ok: "fly-logs-path-ok",
    path_cancel: "fly-logs-path-cancel",
};

/// One of the log tools, the buttons' second column.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tool {
    /// `BUT_firstperson`: First Person KML.
    FirstPerson,
    /// `BUT_redokml`: Create KML.
    RedoKml,
    /// `BUT_bintolog`: .bin to .log.
    BinToLog,
}

impl Tool {
    /// The id a script clicks its button by.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::FirstPerson => "fly-logs-firstperson",
            Self::RedoKml => "fly-logs-redokml",
            Self::BinToLog => "fly-logs-bintolog",
        }
    }

    /// Its button's `Text`. `// C#: Log/LogDownloadMavLink.resx`
    #[must_use]
    pub const fn text(self) -> &'static str {
        match self {
            Self::FirstPerson => "First Person KML",
            Self::RedoKml => "Create KML",
            Self::BinToLog => ".bin to .log",
        }
    }

    /// The name the facts use.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::FirstPerson => "firstperson",
            Self::RedoKml => "redokml",
            Self::BinToLog => "bintolog",
        }
    }

    /// Its `OpenFileDialog`'s filter.
    #[must_use]
    pub const fn filter(self) -> &'static str {
        match self {
            Self::FirstPerson | Self::RedoKml => LOG_FILTER,
            Self::BinToLog => BIN_FILTER,
        }
    }
}

/// What the window is asking, over everything as the C#'s modal boxes are.
#[derive(Debug)]
enum Asking {
    /// Clear Logs' `CustomMessageBox.Show(LogStrings.Confirmation, "sure", YesNo)`.
    /// `// C#: Log/LogDownloadMavLink.cs:452-453`
    Clear,
    /// A tool's `OpenFileDialog`.
    Open(Tool, PathBox),
    /// .bin to .log's `SaveFileDialog`, for the log chosen.
    Save(PathBuf, PathBox),
}

/// A tool's work, once its dialogs have answered.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Work {
    /// Create KML of the log.
    RedoKml(PathBuf),
    /// First Person KML of the log.
    FirstPerson(PathBuf),
    /// .bin to .log: the log, and the file its text goes to.
    BinToLog(PathBuf, PathBuf),
}

impl Work {
    /// The tool doing it.
    #[must_use]
    pub const fn tool(&self) -> Tool {
        match self {
            Self::RedoKml(_) => Tool::RedoKml,
            Self::FirstPerson(_) => Tool::FirstPerson,
            Self::BinToLog(..) => Tool::BinToLog,
        }
    }

    /// The work as its handler does it, once its dialogs have answered.
    #[must_use]
    pub fn run(&self) -> Ran {
        match self {
            Self::RedoKml(log) => redo_kml(log),
            Self::FirstPerson(log) => first_person(log),
            Self::BinToLog(bin, out) => bin_to_log(bin, out),
        }
    }
}

/// What a tool's work came to.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Ran {
    /// `AppendSerialLog`'s lines, in order.
    pub lines: Vec<String>,
    /// The status line's words: what the handler throws, which the form does not catch, as the
    /// C#'s error box says it; or where .bin to .log's file went.
    pub status: Option<String>,
    /// The files written, each with its size.
    pub files: Vec<(PathBuf, u64)>,
}

/// The files, each with its size as it lies on the disk.
fn sized(paths: Vec<PathBuf>) -> Vec<(PathBuf, u64)> {
    paths
        .into_iter()
        .map(|path| {
            let size = mp_os::fs::metadata(&path).map_or(0, |meta| meta.len());
            (path, size)
        })
        .collect()
}

/// The `.kml` a tool names after a log: the log's name with `suffix` added.
fn beside(log: &Path, suffix: &str) -> PathBuf {
    let mut name = log.as_os_str().to_owned();
    name.push(suffix);
    PathBuf::from(name)
}

/// `BUT_redokml_Click`'s work for one log: its lines read as text into a `LogOutput` - a log that
/// cannot be read said in the output, and its KML written all the same, of nothing - then
/// `writeKML(<log>.kml)`, then "Done". What `writeKML` throws ends the handler with no "Done":
/// the status line's error. `// C#: Log/LogDownloadMavLink.cs:487-511`
fn redo_kml(log: &Path) -> Ran {
    let mut ran = Ran::default();
    let mut output = LogOutput::new();
    match mp_os::fs::read(log) {
        Ok(data) => mp_kml::dflog::process_text(&data, &mut output),
        // `ErrorProcessingLogfile + Environment.NewLine + ex.ToString()`.
        Err(err) => ran
            .lines
            .extend([strings::ERROR_PROCESSING.to_owned(), err.to_string()]),
    }
    match mp_kml::dflog::write_kml_files(
        &mut output,
        &beside(log, ".kml"),
        &mp_kml::dflog::local_zone,
    ) {
        Ok(paths) => {
            ran.lines.push(strings::DONE.to_owned());
            ran.files = sized(paths);
        }
        Err(err) => ran.status = Some(error_box(err)),
    }
    ran
}

/// `BUT_firstperson_Click`'s work for one log: its lines read as text into a `LogOutput` - a log
/// that cannot be read said in the output and left there, as the handler's `continue` leaves it -
/// then `writeKMLFirstPerson(<log>-fp.kml)`, then "Done". What that throws - a log not named
/// `.log` has no `.kmz` of its own - ends the handler with no "Done": the status line's error.
/// `// C#: Log/LogDownloadMavLink.cs:544-572`
fn first_person(log: &Path) -> Ran {
    let mut ran = Ran::default();
    let data = match mp_os::fs::read(log) {
        Ok(data) => data,
        // `ErrorProcessingLogfile + Environment.NewLine + ex.Message`, and `continue`.
        Err(err) => {
            ran.lines
                .extend([strings::ERROR_PROCESSING.to_owned(), err.to_string()]);
            return ran;
        }
    };
    let mut output = LogOutput::new();
    mp_kml::dflog::process_text(&data, &mut output);
    match mp_kml::dflog::write_first_person_files(
        &output,
        &beside(log, "-fp.kml"),
        &mp_kml::dflog::local_zone,
    ) {
        Ok(paths) => {
            ran.lines.push(strings::DONE.to_owned());
            ran.files = sized(paths);
        }
        Err(err) => ran.status = Some(error_box(err)),
    }
    ran
}

/// `BinaryLog.ConvertBin(ofd.FileName, sfd.FileName)`: the log's text written where the Save As
/// named, the DataFlash page's conversion ([`mp_log::convert::convert_bin_file`]). The output
/// says nothing, as the C# says nothing; the status line says where it went, or what
/// `ConvertBin` threw. `// C#: Log/LogDownloadMavLink.cs:595`
fn bin_to_log(bin: &Path, out: &Path) -> Ran {
    match mp_log::convert::convert_bin_file(bin, out, &mp_log::convert::flight_mode_name) {
        Ok(()) => Ran {
            lines: Vec::new(),
            status: Some(format!("{}: {}", Tool::BinToLog.text(), out.display())),
            files: sized(vec![out.to_path_buf()]),
        },
        Err(err) => Ran {
            lines: Vec::new(),
            status: Some(error_box(format!("{}: {err}", bin.display()))),
            files: Vec::new(),
        },
    }
}

/// `ConnectionStats.ToHumanReadableByteCount`: Gb, Mb and Kb to two places, whole bytes below.
/// `// C#: Controls/ConnectionStats.cs:165-174`
#[must_use]
pub fn human_bytes(bytes: u32) -> String {
    const K: u32 = 1024;
    let scaled = |divisor: u32| {
        #[allow(clippy::cast_precision_loss)] // a log's size, as the C#'s float division
        let value = bytes as f32 / divisor as f32;
        value
    };
    if bytes > K * K * K {
        format!("{:.2}Gb", scaled(K * K * K))
    } else if bytes > K * K {
        format!("{:.2}Mb", scaled(K * K))
    } else if bytes > K {
        format!("{:.2}Kb", scaled(K))
    } else if bytes == 0 {
        // "####" writes nothing for zero.
        "b".to_owned()
    } else {
        format!("{bytes}b")
    }
}

/// A log's line in `CHK_logs`: its number, when it started in local time as `DateTime.ToString`
/// writes it in an English culture, and its size. `// C#: Log/LogDownloadMavLink.cs:91-125`
#[must_use]
pub fn caption(listing: &LogListing) -> String {
    format!(
        "{} {}  ({})",
        listing.id,
        item_caption(listing),
        human_bytes(listing.size)
    )
}

/// `GetItemCaption`: the Unix time made local. `// C#: Log/LogDownloadMavLink.cs:121-124`
fn item_caption(listing: &LogListing) -> String {
    use chrono::TimeZone as _;
    chrono::Local
        .timestamp_opt(i64::from(listing.time_utc), 0)
        .single()
        .map_or_else(String::new, |time| {
            time.format("%-m/%-d/%Y %-I:%M:%S %p").to_string()
        })
}

/// The window's state.
#[derive(Debug, Default)]
pub struct LogDownloader {
    /// Whether it is showing.
    open: bool,
    /// The logs checked in `CHK_logs`, by number.
    checked: BTreeSet<u16>,
    /// `TXT_seriallog`, oldest first.
    output: Vec<String>,
    /// How many logs had been listed when the output last said so.
    reported: usize,
    /// Whether a list was asked for since the window opened.
    listing: bool,
    /// The logs `DownloadThread` has still to fetch, in order, with their sizes and captions.
    queue: Vec<(u16, u32, String)>,
    /// The log being fetched.
    current: Option<u16>,
    /// `totalBytes`: the batch's size.
    total: u32,
    /// `tallyBytes`: what the logs finished so far in the batch held.
    tally: u32,
    /// When the batch's first byte arrived, for the rate.
    started: Option<Instant>,
    /// "Cancel Download" is being asked.
    confirming_close: bool,
    /// The progress bar and `labelBytes`, as this frame's `UpdateProgress` left them.
    bar: Option<(f32, String)>,
    /// `BUT_clearlogs.Enabled`, as `LoadLogList` last left it: whether a link was open.
    clear_enabled: bool,
    /// `CHK_logs.Items.Clear()` after Clear Logs: the list shows nothing until it is asked for
    /// again, the link's list of what was erased left as it is.
    cleared: bool,
    /// The question being asked.
    asking: Option<Asking>,
    /// The file dialogs' keyboard, made the first time one shows.
    focus: Option<gpui::FocusHandle>,
    /// The tool at work, and where its answer arrives.
    tool: Option<(Tool, std::sync::mpsc::Receiver<Ran>)>,
    /// The tool that finished last, and the files it wrote.
    last: Option<(Tool, Vec<(PathBuf, u64)>)>,
}

impl LogDownloader {
    /// Whether the window is showing.
    #[must_use]
    pub const fn is_open(&self) -> bool {
        self.open
    }

    /// `AppendSerialLog`.
    fn say(&mut self, line: impl Into<String>) {
        self.output.push(line.into());
    }

    /// Whether `DownloadThread` is running: `status == SerialStatus.Reading`.
    #[must_use]
    pub const fn downloading(&self) -> bool {
        self.current.is_some()
    }

    /// The last line of the output.
    #[must_use]
    pub fn last_output(&self) -> &str {
        self.output.last().map_or("", String::as_str)
    }

    /// Toggles a log's box.
    pub fn toggle(&mut self, id: u16) {
        if !self.checked.remove(&id) {
            self.checked.insert(id);
        }
    }

    /// The logs checked.
    #[must_use]
    pub fn checked(&self) -> Vec<u16> {
        self.checked.iter().copied().collect()
    }

    /// The link has `count` logs listed: once a list was asked for, the output says how many -
    /// `LoadCheckedList`'s "Found" line. The entries arrive one `LOG_ENTRY` at a time, where the
    /// C# waits for the whole list, so the line is said again, in place, as the count grows.
    /// Nothing once Clear Logs has emptied the list, until it is asked for again.
    /// `// C#: Log/LogDownloadMavLink.cs:91-119`
    pub fn listed(&mut self, count: usize) {
        if self.cleared || !self.listing || count == self.reported || count == 0 {
            return;
        }
        if self.reported != usize::MAX
            && self
                .output
                .last()
                .is_some_and(|line| line.starts_with("Found "))
        {
            self.output.pop();
        }
        self.say(strings::SOME_LOGS.replace("{0}", &count.to_string()));
        self.reported = count;
    }

    /// `LoadLogList`, but for the sending: with no link, "Please connect your drone" and Clear
    /// Logs dimmed; with one, Clear Logs back and the list emptied - its checks with it - to be
    /// asked for again. Returns whether to ask. `// C#: Log/LogDownloadMavLink.cs:59-89`
    fn load_list(&mut self, connected: bool) -> bool {
        self.clear_enabled = connected;
        if !connected {
            self.say(strings::NOT_CONNECTED);
            return false;
        }
        self.cleared = false;
        self.checked.clear();
        self.say(strings::FETCHING_LIST);
        self.listing = true;
        self.reported = usize::MAX;
        true
    }

    /// Clear Logs answered Yes, `LOG_ERASE` on its way: `LogStrings.EraseComplete` said, and
    /// `CHK_logs` emptied. `// C#: Log/LogDownloadMavLink.cs:457-460`
    fn erased(&mut self) {
        self.say(strings::ERASE_COMPLETE);
        self.cleared = true;
        self.checked.clear();
    }

    /// The logs `CHK_logs` lists of the link's: all of them, or none once Clear Logs has emptied
    /// it.
    #[must_use]
    pub fn shown<'a>(&self, listings: &'a [LogListing]) -> &'a [LogListing] {
        if self.cleared { &[] } else { listings }
    }

    /// Whether a tool is at work.
    #[must_use]
    pub const fn working(&self) -> bool {
        self.tool.is_some()
    }

    /// A tool's file dialog answered. OK on an `OpenFileDialog` takes a file that is there - any
    /// other name keeps it asking, as the platform's dialog does - and .bin to .log's goes on to
    /// its `SaveFileDialog`, opening in the log's folder, where the platform's dialogs open after
    /// the last; OK on that takes the name typed, `.log` put on one with no extension
    /// (`AddExtension`), a name that is none or a folder asked again. Cancel ends the press.
    /// Returns the work to start. `// C#: Log/LogDownloadMavLink.cs:485-487, 542-544, 583-595`
    fn answer_path(&mut self, ok: bool) -> Option<Work> {
        match self.asking.take() {
            Some(Asking::Open(tool, path)) => {
                if !ok {
                    return None;
                }
                let Some(file) = path.chosen() else {
                    self.asking = Some(Asking::Open(tool, path));
                    return None;
                };
                match tool {
                    Tool::RedoKml => Some(Work::RedoKml(file)),
                    Tool::FirstPerson => Some(Work::FirstPerson(file)),
                    Tool::BinToLog => {
                        let folder = file
                            .parent()
                            .map(|dir| dir.display().to_string())
                            .unwrap_or_default();
                        let mut save = PathBox::new(&folder, SAVE_FILTER);
                        save.caption = crate::joystick::SAVE_AS;
                        self.asking = Some(Asking::Save(file, save));
                        None
                    }
                }
            }
            Some(Asking::Save(bin, path)) => {
                if !ok {
                    return None;
                }
                let Some(out) = crate::plan::dialog_path(
                    path.field.value(),
                    SAVE_EXTENSION,
                    &crate::file_list::start_folder(),
                ) else {
                    self.asking = Some(Asking::Save(bin, path));
                    return None;
                };
                Some(Work::BinToLog(bin, out))
            }
            other => {
                self.asking = other;
                None
            }
        }
    }

    /// The work started on a thread of its own, the KML tools' `ProcessingLog` said first after
    /// the two empty lines the C# puts before it. Nothing starts while a tool works. `Err` with
    /// what the status line says when no thread could be had.
    /// `// C#: Log/LogDownloadMavLink.cs:489-491, 546-548`
    fn start(&mut self, work: Work) -> Result<(), String> {
        if self.tool.is_some() {
            return Ok(());
        }
        let tool = work.tool();
        if let Work::RedoKml(log) | Work::FirstPerson(log) = &work {
            // `AppendSerialLog(Environment.NewLine + Environment.NewLine + string.Format(...))`.
            self.say("");
            self.say("");
            self.say(strings::PROCESSING.replace("{0}", &log.display().to_string()));
        }
        let (sender, receiver) = std::sync::mpsc::channel();
        let spawned = wasm_thread::Builder::new()
            .name(format!("mp-logs-{}", tool.name()))
            .spawn(move || {
                let _ = sender.send(work.run());
            });
        if let Err(err) = spawned {
            return Err(error_box(err));
        }
        self.tool = Some((tool, receiver));
        Ok(())
    }

    /// Once a frame: the tool's answer, when it comes - its lines said in the output, the files it
    /// wrote kept for the facts - returning what the status line says.
    fn poll(&mut self) -> Option<String> {
        let (tool, receiver) = self.tool.as_ref()?;
        let tool = *tool;
        let ran = match receiver.try_recv() {
            Ok(ran) => ran,
            Err(std::sync::mpsc::TryRecvError::Empty) => {
                crate::repaint::in_flight();
                return None;
            }
            Err(std::sync::mpsc::TryRecvError::Disconnected) => Ran {
                status: Some(error_box(format!(
                    "{}: stopped without an answer",
                    tool.text()
                ))),
                ..Ran::default()
            },
        };
        self.tool = None;
        for line in ran.lines {
            self.say(line);
        }
        self.last = Some((tool, ran.files));
        ran.status
    }

    /// Starts a batch: `DownloadThread(selected)`'s opening - the sizes added up, then the
    /// first log asked for. Returns the log to ask for.
    fn start_batch(&mut self, logs: Vec<(u16, u32, String)>) -> Option<(u16, u32)> {
        self.total = logs.iter().map(|(_, size, _)| *size).sum();
        self.tally = 0;
        self.started = None;
        self.queue = logs;
        self.next()
    }

    /// The next log of the batch, said as `FetchingLog` says it.
    fn next(&mut self) -> Option<(u16, u32)> {
        if self.queue.is_empty() {
            self.current = None;
            return None;
        }
        let (id, size, caption) = self.queue.remove(0);
        self.say(strings::FETCHING_LOG.replace("{0}", &caption));
        self.current = Some(id);
        Some((id, size))
    }

    /// A log of the batch arrived whole: counted, and the next asked for - or the batch is
    /// done. `// C#: Log/LogDownloadMavLink.cs:340-357`
    pub fn finished(&mut self, id: u16, bytes: u32) -> Option<(u16, u32)> {
        if self.current != Some(id) {
            return None;
        }
        self.tally = self.tally.saturating_add(bytes);
        let next = self.next();
        if next.is_none() {
            self.say(strings::COMPLETE);
        }
        next
    }

    /// `UpdateProgress(0, totalBytes, tallyBytes + received)`: the bar's fraction and
    /// `labelBytes` - what has arrived, the percentage, the rate and the time left - or `None`
    /// once the bar is hidden, when everything has arrived.
    /// `// C#: Log/LogDownloadMavLink.cs:390-426`
    #[must_use]
    pub fn progress(&mut self, received: u32, now: Instant) -> Option<(f32, String)> {
        let current = self.tally.saturating_add(received);
        let max = self.total;
        if self.current.is_none() || current >= max {
            return None;
        }
        if current == 0 {
            self.started = Some(now);
            return Some((0.0, String::new()));
        }
        let started = *self.started.get_or_insert(now);
        let elapsed = now.duration_since(started).as_secs_f64();
        let elapsed = if elapsed == 0.0 { 1.0 } else { elapsed };
        let per = f64::from(current) / f64::from(max) * 100.0;
        let rate = f64::from(current) / elapsed;
        let rate = if rate == 0.0 { 1.0 } else { rate };
        let left = f64::from(max - current) / rate;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let (left_seconds, rate_bytes) = (left as u64, rate as u32);
        let hours = left_seconds / 3600;
        let clock = if hours > 0 {
            format!(
                "{hours}:{:02}:{:02}",
                left_seconds / 60 % 60,
                left_seconds % 60
            )
        } else {
            format!("{:02}:{:02}", left_seconds / 60 % 60, left_seconds % 60)
        };
        #[allow(clippy::cast_possible_truncation)]
        let fraction = (f64::from(current) / f64::from(max)) as f32;
        Some((
            fraction,
            format!(
                "{} {per:.1}% {}/s {clock} left",
                human_bytes(current),
                human_bytes(rate_bytes)
            ),
        ))
    }

    /// Publishes what a UI test asserts on: besides the list, the downloads and the output - its
    /// last line, and every line, which the facts' file runs together - Clear Logs' state, the
    /// question asked, the tool at work, and the last tool to finish with the files it wrote.
    pub fn record_facts(&self, listed: usize) {
        crate::facts::record("fly.logs.open", self.is_open());
        crate::facts::record("fly.logs.items", if self.cleared { 0 } else { listed });
        crate::facts::record("fly.logs.output.text", self.output.join("\n"));
        crate::facts::record("fly.logs.clear.enabled", self.clear_enabled);
        crate::facts::record(
            "fly.logs.asking",
            match &self.asking {
                None => "none",
                Some(Asking::Clear) => "clear",
                Some(Asking::Open(tool, _)) => tool.name(),
                Some(Asking::Save(..)) => "bintolog-save",
            },
        );
        crate::facts::record(
            "fly.logs.asking.path",
            match &self.asking {
                Some(Asking::Open(_, path) | Asking::Save(_, path)) => {
                    format!("{}: {}", path.caption, path.field.value())
                }
                _ => "none".to_owned(),
            },
        );
        crate::facts::record(
            "fly.logs.tool",
            self.tool.as_ref().map_or("none", |(tool, _)| tool.name()),
        );
        crate::facts::record(
            "fly.logs.tool.last",
            self.last.as_ref().map_or("none", |(tool, _)| tool.name()),
        );
        crate::facts::record(
            "fly.logs.tool.files",
            match &self.last {
                Some((_, files)) if !files.is_empty() => files
                    .iter()
                    .map(|(path, size)| {
                        let name = path
                            .file_name()
                            .map_or_else(String::new, |name| name.to_string_lossy().into_owned());
                        format!("{name}:{size}")
                    })
                    .collect::<Vec<_>>()
                    .join(","),
                _ => "none".to_owned(),
            },
        );
        crate::facts::record(
            "fly.logs.checked",
            self.checked()
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(","),
        );
        crate::facts::record("fly.logs.output", self.last_output());
        crate::facts::record(
            "fly.logs.fetching",
            self.current
                .map_or_else(|| "none".to_owned(), |id| id.to_string()),
        );
        crate::facts::record("fly.logs.queued", self.queue.len());
        crate::facts::record("fly.logs.confirm", self.confirming_close);
    }
}

impl MissionPlanner {
    /// `BUT_DFMavlink_Click`, then the form's `Log_Load`: the list asked for, or why not, and
    /// the warning to an armed vehicle. `// C#: GCSViews/FlightData.cs:1206-1211,
    /// Log/LogDownloadMavLink.cs:51-89`
    pub(crate) fn logs_open(&mut self) {
        let view = self.telemetry.view();
        self.fly_data.logs.open = true;
        self.fly_data.logs.confirming_close = false;
        self.logs_load_list(&view);
        if view.state.as_ref().is_some_and(|state| state.armed) {
            self.file_status = Some(error_box(strings::DISARM));
        }
    }

    /// `LoadLogList`: with a link open - a vehicle on a port, not a log played - the list asked
    /// for, Clear Logs allowed; without, why not, and Clear Logs dimmed.
    /// `// C#: Log/LogDownloadMavLink.cs:59-89`
    fn logs_load_list(&mut self, view: &crate::telemetry::TelemetryView) {
        let connected = crate::fly::port_open(view) && view.vehicle.is_some();
        if self.fly_data.logs.load_list(connected) {
            self.telemetry.request_log_list();
        }
    }

    /// `BUT_DLall_Click`: with nothing listed - nothing yet, or the list Clear Logs emptied -
    /// `LoadLogList` again; otherwise every log, in the list's order, into the log directory.
    /// `// C#: Log/LogDownloadMavLink.cs:158-197`
    fn logs_download_all(&mut self) {
        if self.fly_data.logs.downloading() {
            return;
        }
        let all = self.telemetry.log_listings();
        let listings = self.fly_data.logs.shown(&all);
        if listings.is_empty() {
            // "try again..."
            let view = self.telemetry.view();
            self.logs_load_list(&view);
            return;
        }
        let directory = Self::plan_directory();
        self.fly_data
            .logs
            .say(strings::DOWNLOAD_STARTING.replace("{0}", &directory.display().to_string()));
        let batch = listings
            .iter()
            .map(|listing| (listing.id, listing.size, item_caption(listing)))
            .collect();
        if let Some((id, size)) = self.fly_data.logs.start_batch(batch) {
            self.telemetry.download_log(id, size);
        }
    }

    /// `BUT_DLthese_Click`: the checked logs, or a reminder to check some.
    /// `// C#: Log/LogDownloadMavLink.cs:428-448`
    fn logs_download_selected(&mut self) {
        if self.fly_data.logs.downloading() {
            return;
        }
        let listings = self.telemetry.log_listings();
        let batch: Vec<(u16, u32, String)> = listings
            .iter()
            .filter(|listing| self.fly_data.logs.checked.contains(&listing.id))
            .map(|listing| (listing.id, listing.size, item_caption(listing)))
            .collect();
        if batch.is_empty() {
            self.fly_data.logs.say(strings::NOTHING_SELECTED);
            return;
        }
        if let Some((id, size)) = self.fly_data.logs.start_batch(batch) {
            self.telemetry.download_log(id, size);
        }
    }

    /// The form's close box: `OnClosing` asks first while a log is being read.
    /// `// C#: Log/LogDownloadMavLink.cs:272-285`
    fn logs_close(&mut self) {
        if self.fly_data.logs.downloading() {
            self.fly_data.logs.confirming_close = true;
        } else {
            self.fly_data.logs.open = false;
        }
    }

    /// A log tool's button: its `OpenFileDialog`, a typed path, with the keyboard, opening in the
    /// log directory - which First Person KML makes first. .bin to .log's sets no folder, so the
    /// platform's dialog opens where it last was: here the log directory too, as the DataFlash
    /// page's Convert .Bin to .Log opens. Nothing while a tool works.
    /// `// C#: Log/LogDownloadMavLink.cs:471-485, 527-542, 579-583`
    fn logs_ask_file(&mut self, tool: Tool, window: &mut Window, cx: &mut Context<Self>) {
        let logs = &mut self.fly_data.logs;
        if logs.working() {
            return;
        }
        let folder = crate::fly::log_directory();
        if tool == Tool::FirstPerson
            && let Some(folder) = &folder
        {
            // `Directory.CreateDirectory(Settings.Instance.LogDir)`, in its `try`.
            let _ = mp_os::fs::create_dir_all(folder);
        }
        let folder = folder
            .map(|folder| folder.display().to_string())
            .unwrap_or_default();
        logs.asking = Some(Asking::Open(tool, PathBox::new(&folder, tool.filter())));
        let focus = logs.focus.get_or_insert_with(|| cx.focus_handle()).clone();
        focus.focus(window, cx);
    }

    /// Clear Logs answered Yes: `EraseLog` - `LOG_ERASE` twice to the vehicle being flown, which
    /// answers nothing - then "!!Allow 30-90 seconds for erase" said and the list emptied. With no
    /// vehicle nothing is sent, as `generatePacket` writes nothing to a closed port, and the rest
    /// is the same. `// C#: Log/LogDownloadMavLink.cs:450-467; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6269-6279`
    fn logs_clear(&mut self) {
        if let Some((_, target)) = self.telemetry.send_handle() {
            let erase = mp_link::commands::log_erase(target);
            self.telemetry.send(&erase);
            self.telemetry.send(&erase);
        }
        self.fly_data.logs.erased();
    }

    /// Once a frame: a log tool's answer said, the list's arrival said in the output, and a
    /// download kept moving, written out when it finishes, and followed by the next of its batch.
    ///
    /// Each log is written where this application has always put a downloaded log, the plan
    /// directory, as `log_<n>.bin`. The C# writes `<log dir>/<vehicle type>/<sysid>/<n> <date>.bin`
    /// and renames it by the log's first GPS time; that naming is not ported.
    pub(crate) fn logs_tick(&mut self) {
        if let Some(status) = self.fly_data.logs.poll() {
            self.file_status = Some(status);
        }
        let listed = self.telemetry.log_listings().len();
        self.fly_data.logs.listed(listed);
        let received = self.telemetry.log_progress().map(|(_, got, _)| got);
        self.fly_data.logs.bar = self
            .fly_data
            .logs
            .progress(received.unwrap_or(0), Instant::now());
        if received.is_none() {
            return;
        }
        if let Some((id, bytes)) = self.telemetry.finished_log() {
            self.telemetry.clear_log_download();
            let path = Self::plan_directory().join(format!("log_{id}.bin"));
            self.file_status = Some(match mp_os::fs::write(&path, &bytes) {
                Ok(()) => {
                    crate::page_files::saved(&path);
                    format!("wrote {} ({} bytes)", path.display(), bytes.len())
                }
                Err(err) => format!("could not write {}: {err}", path.display()),
            });
            let size = u32::try_from(bytes.len()).unwrap_or(u32::MAX);
            if let Some((next, next_size)) = self.fly_data.logs.finished(id, size) {
                self.telemetry.download_log(next, next_size);
            }
        } else {
            self.telemetry.nudge_log_download();
        }
    }
}

/// The window, over the screen without a backdrop: `Show()` leaves everything behind it usable.
pub fn window(
    logs: &LogDownloader,
    listings: &[LogListing],
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if !logs.open {
        return None;
    }
    let size = window.viewport_size();

    let mut list = div()
        .id("fly-logs-list")
        .flex()
        .flex_col()
        .h(px(244.0))
        .overflow_y_scroll()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG));
    for listing in logs.shown(listings) {
        let id = listing.id;
        let checked = logs.checked.contains(&id);
        let key = format!("fly-log-{id}");
        list = list.child(
            crate::probe::measured(key.clone(), div())
                .id(gpui::SharedString::from(key))
                .flex()
                .gap_1()
                .px_1()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(if checked { "\u{2611}" } else { "\u{2610}" })
                .child(caption(listing))
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.fly_data.logs.toggle(id);
                    cx.notify();
                })),
        );
    }

    let grid = buttons(logs, cx);

    let mut bar = div().flex().items_center().gap_2().h(px(20.0));
    if let Some((fraction, text)) = &logs.bar {
        bar = bar
            .child(div().flex_1().child(progress(*fraction, theme::ACCENT)))
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .child(text.clone()),
            );
    }

    let mut output = div()
        .id("fly-logs-output")
        .flex()
        .flex_col()
        .flex_1()
        .min_h(px(0.0))
        .overflow_y_scroll()
        .p_1()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG));
    for line in &logs.output {
        output = output.child(
            div()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(line.clone()),
        );
    }

    let mut body = crate::probe::measured("fly-logs", div())
        .id("fly-logs")
        .flex()
        .flex_col()
        .gap_2()
        .w(px(784.0).min(size.width - px(20.0)))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_md()
        .occlude()
        .child(
            div()
                .flex()
                .justify_between()
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child("Log Downloader"),
                )
                .child(action(
                    "fly-logs-close",
                    "\u{2715}",
                    theme::TEXT,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.logs_close();
                        cx.notify();
                    }),
                )),
        )
        .child(
            div()
                .flex()
                .gap_2()
                .h(px(390.0))
                .child(
                    div()
                        .flex()
                        .flex_col()
                        .gap_1()
                        .w(px(321.0))
                        .child(
                            div()
                                .text_xs()
                                .text_color(rgb(theme::DIM))
                                .child("Log files:"),
                        )
                        .child(list)
                        .child(grid)
                        .child(bar),
                )
                .child(
                    div()
                        .flex()
                        .flex_col()
                        .gap_1()
                        .flex_1()
                        .min_w(px(0.0))
                        .child(div().text_xs().text_color(rgb(theme::DIM)).child("Output:"))
                        .child(output)
                        .child(
                            div()
                                .text_xs()
                                .text_color(rgb(theme::DIM))
                                .child(strings::NOTE),
                        ),
                ),
        );
    if logs.confirming_close {
        body = body.child(
            div()
                .flex()
                .items_center()
                .gap_2()
                .p_2()
                .border_1()
                .border_color(rgb(theme::WARN))
                .rounded_md()
                .child(
                    div()
                        .flex_1()
                        .text_sm()
                        .text_color(rgb(theme::TEXT))
                        .child(strings::CANCEL_DOWNLOAD),
                )
                .child(action(
                    "fly-logs-cancel-yes",
                    "Yes",
                    theme::OK,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        // The form closes; `DownloadThread` carries on without it.
                        this.fly_data.logs.confirming_close = false;
                        this.fly_data.logs.open = false;
                        cx.notify();
                    }),
                ))
                .child(action(
                    "fly-logs-cancel-no",
                    "No",
                    theme::TEXT,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.fly_data.logs.confirming_close = false;
                        cx.notify();
                    }),
                )),
        );
    }

    let asking = asking_box(logs, window, cx);
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(
                    ((size.width - px(784.0)) / 2.0).max(px(0.0)),
                    px(60.0),
                ))
                .child(div().relative().child(body).children(asking)),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

/// `tableLayoutPanel3`, two columns of three: the downloads and Clear Logs, then the log tools.
/// The downloads wait while one runs; Clear Logs as `LoadLogList` left it; the tools while one
/// works. `// C#: Log/LogDownloadMavLink.Designer.cs:160-170`
fn buttons(logs: &LogDownloader, cx: &mut Context<MissionPlanner>) -> Div {
    let busy = logs.downloading();
    let idle = !logs.working();
    div()
        .grid()
        .grid_cols(2)
        .gap_1()
        .child(action(
            "fly-logs-dlall",
            "Download All Logs",
            theme::ACCENT,
            !busy,
            cx.listener(|this, _event: &(), _window, cx| {
                this.logs_download_all();
                cx.notify();
            }),
        ))
        .child(tool_button(Tool::FirstPerson, idle, cx))
        .child(action(
            "fly-logs-dlthese",
            "Download Selected Logs",
            theme::ACCENT,
            !busy,
            cx.listener(|this, _event: &(), _window, cx| {
                this.logs_download_selected();
                cx.notify();
            }),
        ))
        .child(tool_button(Tool::RedoKml, idle, cx))
        .child(action(
            "fly-logs-clear",
            "Clear Logs",
            theme::ACCENT,
            logs.clear_enabled,
            cx.listener(|this, _event: &(), _window, cx| {
                this.fly_data.logs.asking = Some(Asking::Clear);
                cx.notify();
            }),
        ))
        .child(tool_button(Tool::BinToLog, idle, cx))
}

/// A log tool's button: its dialog asked for, while no tool works.
fn tool_button(tool: Tool, enabled: bool, cx: &mut Context<MissionPlanner>) -> AnyElement {
    action(
        tool.id(),
        tool.text(),
        theme::ACCENT,
        enabled,
        cx.listener(move |this, _event: &(), window, cx| {
            this.logs_ask_file(tool, window, cx);
            cx.notify();
        }),
    )
}

/// The question the window is asking, over it and everything else as the C#'s modal boxes are:
/// Clear Logs' "Are you sure?", or a tool's file dialog with the keyboard.
fn asking_box(
    logs: &LogDownloader,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    match logs.asking.as_ref()? {
        Asking::Clear => Some(crate::config::firmware::question_box(
            IDS,
            strings::SURE,
            strings::CONFIRMATION,
            Buttons::YesNo,
            window,
            clear_answered,
            cx,
        )),
        Asking::Open(_, path) | Asking::Save(_, path) => Some(crate::config::firmware::path_box(
            IDS,
            path,
            logs.focus.as_ref()?,
            window,
            path_key,
            path_answered,
            cx,
        )),
    }
}

/// Clear Logs' question answered: Yes erases, No leaves everything as it was.
/// `// C#: Log/LogDownloadMavLink.cs:452-466`
fn clear_answered(this: &mut MissionPlanner, yes: bool) {
    if !matches!(this.fly_data.logs.asking, Some(Asking::Clear)) {
        return;
    }
    this.fly_data.logs.asking = None;
    if yes {
        this.logs_clear();
    }
}

/// A key in a file dialog's box: typing, Enter for OK and Escape for Cancel. False for a key the
/// box takes no notice of.
fn path_key(this: &mut MissionPlanner, event: &KeyDownEvent) -> bool {
    let outcome = match this.fly_data.logs.asking.as_mut() {
        Some(Asking::Open(_, path) | Asking::Save(_, path)) => path.field.key(event),
        _ => return false,
    };
    match outcome {
        KeyOutcome::Submitted => path_answered(this, true),
        KeyOutcome::Cancelled => path_answered(this, false),
        KeyOutcome::Changed => {}
        KeyOutcome::Ignored => return false,
    }
    true
}

/// A file dialog answered: the tool's work started, or its next dialog shown; a thread that could
/// not be had said on the status line.
fn path_answered(this: &mut MissionPlanner, ok: bool) {
    let Some(work) = this.fly_data.logs.answer_path(ok) else {
        return;
    };
    if let Err(why) = this.fly_data.logs.start(work) {
        this.file_status = Some(why);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn sizes_read_as_the_csharp_writes_them() {
        assert_eq!(human_bytes(0), "b");
        assert_eq!(human_bytes(900), "900b");
        assert_eq!(human_bytes(1024), "1024b");
        assert_eq!(human_bytes(1536), "1.50Kb");
        assert_eq!(human_bytes(5 * 1024 * 1024 + 1), "5.00Mb");
    }

    #[test]
    fn a_log_is_listed_with_its_number_date_and_size() {
        let listing = LogListing {
            id: 3,
            size: 2048 + 512,
            time_utc: 0,
        };
        let text = caption(&listing);
        assert!(text.starts_with("3 "), "{text}");
        assert!(text.ends_with("  (2.50Kb)"), "{text}");
        // 1970, in whatever zone this machine keeps: "12/31/1969 ..." west of Greenwich.
        assert!(text.contains("/19"), "{text}");
    }

    #[test]
    fn a_batch_fetches_each_log_in_turn_and_says_so() {
        let mut logs = LogDownloader::default();
        let first = logs.start_batch(vec![(1, 100, "one".to_owned()), (2, 300, "two".to_owned())]);
        assert_eq!(first, Some((1, 100)));
        assert_eq!(logs.last_output(), "Fetching log file one ...");
        assert!(logs.downloading());
        // A finish for a log not being fetched changes nothing.
        assert_eq!(logs.finished(7, 5), None);
        assert_eq!(logs.finished(1, 100), Some((2, 300)));
        assert_eq!(logs.last_output(), "Fetching log file two ...");
        assert_eq!(logs.finished(2, 300), None);
        assert_eq!(logs.last_output(), "Download complete.");
        assert!(!logs.downloading());
    }

    #[test]
    fn the_progress_line_is_bytes_percent_rate_and_time_left() {
        let mut logs = LogDownloader::default();
        logs.start_batch(vec![(1, 20_480, "one".to_owned())]);
        let start = Instant::now();
        assert_eq!(logs.progress(0, start), Some((0.0, String::new())));
        let (fraction, text) = logs
            .progress(10_240, start + Duration::from_secs(10))
            .unwrap();
        assert!((fraction - 0.5).abs() < 1e-6);
        assert_eq!(text, "10.00Kb 50.0% 1024b/s 00:10 left");
        // Everything arrived: the bar goes.
        assert_eq!(logs.progress(20_480, start), None);
    }

    #[test]
    fn the_found_line_counts_the_list_as_it_arrives() {
        let mut logs = LogDownloader::default();
        // Nothing asked for: nothing said.
        logs.listed(3);
        assert_eq!(logs.last_output(), "");
        logs.listing = true;
        logs.reported = usize::MAX;
        logs.say(strings::FETCHING_LIST);
        logs.listed(0);
        assert_eq!(logs.last_output(), "Getting list of log files...");
        logs.listed(1);
        logs.listed(4);
        assert_eq!(
            logs.output,
            vec![
                "Getting list of log files...".to_owned(),
                "Found 4 log files, note: item sizes are just an estimate.".to_owned(),
            ]
        );
        logs.listed(4);
        assert_eq!(logs.output.len(), 2);
    }

    #[test]
    fn checking_a_log_twice_unchecks_it() {
        let mut logs = LogDownloader::default();
        logs.toggle(4);
        logs.toggle(2);
        assert_eq!(logs.checked(), vec![2, 4]);
        logs.toggle(4);
        assert_eq!(logs.checked(), vec![2]);
    }

    // --- The log tools and Clear Logs ------------------------------------------------------------

    use mp_os::fs::FsExt as _;

    fn testdata(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name)
    }

    /// A directory of its own for one test, empty.
    fn scratch(test: &str) -> PathBuf {
        let dir =
            mp_os::temp_dir().join(format!("mp-gui-logdownload-{test}-{}", mp_os::process_id()));
        let _ = mp_os::fs::remove_dir_all(&dir);
        mp_os::fs::create_dir_all(&dir).expect("a scratch directory");
        dir
    }

    /// The names of the files a tool wrote.
    fn names(ran: &Ran) -> Vec<String> {
        ran.files
            .iter()
            .map(|(path, _)| {
                path.file_name()
                    .expect("a file")
                    .to_string_lossy()
                    .into_owned()
            })
            .collect()
    }

    /// `LoadLogList`: with no link, why not, and Clear Logs dimmed; with one, Clear Logs allowed,
    /// the checks gone with the list, and the list to be asked for.
    /// `// C#: Log/LogDownloadMavLink.cs:59-89`
    #[test]
    fn clear_logs_is_allowed_only_with_a_link() {
        let mut logs = LogDownloader::default();
        assert!(!logs.load_list(false));
        assert_eq!(logs.last_output(), strings::NOT_CONNECTED);
        assert!(!logs.clear_enabled);
        logs.toggle(3);
        assert!(logs.load_list(true));
        assert_eq!(logs.last_output(), strings::FETCHING_LIST);
        assert!(logs.clear_enabled);
        assert!(logs.checked().is_empty());
        assert!(!logs.load_list(false));
        assert!(!logs.clear_enabled);
    }

    /// Clear Logs answered Yes: "!!Allow 30-90 seconds for erase", and `CHK_logs` empty - no rows,
    /// no checks, no "Found" line - until the list is asked for again.
    /// `// C#: Log/LogDownloadMavLink.cs:457-460`
    #[test]
    fn clearing_the_logs_empties_the_list_until_it_is_asked_for_again() {
        let listing = LogListing {
            id: 1,
            size: 100,
            time_utc: 0,
        };
        let mut logs = LogDownloader::default();
        assert!(logs.load_list(true));
        logs.listed(1);
        logs.toggle(1);
        assert_eq!(logs.shown(&[listing]).len(), 1);
        logs.erased();
        assert_eq!(logs.last_output(), "!!Allow 30-90 seconds for erase");
        assert!(logs.shown(&[listing]).is_empty());
        assert!(logs.checked().is_empty());
        logs.listed(2);
        assert_eq!(logs.last_output(), "!!Allow 30-90 seconds for erase");
        assert!(logs.load_list(true));
        assert_eq!(logs.shown(&[listing]).len(), 1);
        logs.listed(1);
        assert_eq!(
            logs.last_output(),
            "Found 1 log files, note: item sizes are just an estimate."
        );
    }

    /// A tool's `OpenFileDialog`: OK on a name that is no file asks again; on a file, the work;
    /// Cancel ends the press. `// C#: Log/LogDownloadMavLink.cs:485-487, 542-544`
    #[test]
    fn a_tool_asks_until_it_is_given_a_file_that_is_there() {
        let dir = scratch("open");
        let log = dir.join("flight.log");
        mp_os::fs::write(&log, b"").expect("a log");
        let mut logs = LogDownloader::default();
        for tool in [Tool::RedoKml, Tool::FirstPerson] {
            let path = PathBox::new(&dir.display().to_string(), tool.filter());
            logs.asking = Some(Asking::Open(tool, path));
            assert_eq!(logs.answer_path(true), None, "the folder is no file");
            assert!(matches!(logs.asking, Some(Asking::Open(asking, _)) if asking == tool));
            if let Some(Asking::Open(_, path)) = logs.asking.as_mut() {
                path.field.set(log.display().to_string());
            }
            let work = if tool == Tool::RedoKml {
                Work::RedoKml(log.clone())
            } else {
                Work::FirstPerson(log.clone())
            };
            assert_eq!(logs.answer_path(true), Some(work));
            assert!(logs.asking.is_none());
        }
        logs.asking = Some(Asking::Open(Tool::RedoKml, PathBox::new("", LOG_FILTER)));
        assert_eq!(logs.answer_path(false), None);
        assert!(logs.asking.is_none(), "Cancel ends the press");
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    /// .bin to .log: the log chosen, then its Save As, opening in the log's folder; a folder for a
    /// name asks again, and a name with no extension is given `.log`.
    /// `// C#: Log/LogDownloadMavLink.cs:579-596`
    #[test]
    fn bin_to_log_asks_where_to_save_in_the_logs_folder() {
        let dir = scratch("save");
        let bin = dir.join("flight.bin");
        mp_os::fs::write(&bin, b"").expect("a log");
        let mut logs = LogDownloader::default();
        let mut open = PathBox::new("", BIN_FILTER);
        open.field.set(bin.display().to_string());
        logs.asking = Some(Asking::Open(Tool::BinToLog, open));
        assert_eq!(logs.answer_path(true), None);
        let Some(Asking::Save(chosen, save)) = logs.asking.as_ref() else {
            panic!("no Save As");
        };
        assert_eq!(chosen, &bin);
        assert_eq!(save.caption, "Save As");
        assert_eq!(save.filter, SAVE_FILTER);
        assert_eq!(
            save.field.value(),
            format!("{}{}", dir.display(), std::path::MAIN_SEPARATOR)
        );
        assert_eq!(logs.answer_path(true), None, "a folder is no name");
        assert!(matches!(logs.asking, Some(Asking::Save(..))));
        if let Some(Asking::Save(_, save)) = logs.asking.as_mut() {
            save.field.set(dir.join("converted").display().to_string());
        }
        assert_eq!(
            logs.answer_path(true),
            Some(Work::BinToLog(bin.clone(), dir.join("converted.log")))
        );
        assert!(logs.asking.is_none());
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    /// Create KML over a log read as text: Create KML + gpx's files beside it - the `.gpx` the
    /// golden's size, whatever zone its times are in - and "Done".
    /// `// C#: Log/LogDownloadMavLink.cs:487-511`
    #[test]
    fn create_kml_writes_create_kml_and_gpxs_files_and_says_done() {
        let dir = scratch("redokml");
        let log = dir.join("flight.log");
        mp_os::fs::copy(testdata("dataflash/synthetic.log"), &log).expect("the log copied");
        let ran = Work::RedoKml(log).run();
        assert_eq!(ran.lines, ["Done"]);
        assert_eq!(ran.status, None);
        let names = names(&ran);
        for name in [
            "flight.log.gpx",
            "flight.log.obs",
            "flight.log0wp.txt",
            "flight.log0rally.txt",
            "flight.log.param",
            "flight.kmz",
        ] {
            assert!(
                names.iter().any(|written| written == name),
                "{name}: {names:?}"
            );
            assert!(dir.join(name).os_is_file(), "{name}");
        }
        assert!(ran.files.contains(&(dir.join("flight.log.gpx"), 770)));
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    /// Create KML of a log that is not there: the C#'s error said, then the KML written all the
    /// same, of nothing, and "Done". `// C#: Log/LogDownloadMavLink.cs:493-510`
    #[test]
    fn create_kml_of_a_log_not_there_says_so_and_writes_it_of_nothing() {
        let dir = scratch("redokml-gone");
        let ran = Work::RedoKml(dir.join("gone.log")).run();
        assert_eq!(ran.lines.len(), 3, "{:?}", ran.lines);
        assert_eq!(ran.lines[0], strings::ERROR_PROCESSING);
        assert_eq!(ran.lines[2], "Done");
        assert_eq!(names(&ran), ["gone.log.gpx", "gone.kmz"]);
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    /// First Person KML: the tour zipped as `<name>-fp.kmz` beside the log - the `.kml` in it, not
    /// beside it - and "Done". `// C#: Log/LogDownloadMavLink.cs:550-571`
    #[test]
    fn first_person_kml_writes_its_kmz_and_says_done() {
        let dir = scratch("firstperson");
        let log = dir.join("flight.log");
        mp_os::fs::copy(testdata("dataflash/synthetic.log"), &log).expect("the log copied");
        let ran = Work::FirstPerson(log).run();
        assert_eq!(ran.lines, ["Done"]);
        assert_eq!(ran.status, None);
        assert_eq!(names(&ran), ["flight-fp.kmz"]);
        assert!(!dir.join("flight.log-fp.kml").os_is_file());
        let kmz = mp_os::fs::read(dir.join("flight-fp.kmz")).expect("the .kmz");
        let entries = mp_log::zip::read(&kmz).expect("a zip");
        let names: Vec<&str> = entries.iter().map(|entry| entry.name.as_str()).collect();
        assert_eq!(names, ["flight.log-fp.kml", "block_plane_0.dae"]);
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    /// First Person KML of a log that is not there: the C#'s error said, and nothing more - its
    /// `continue` - nor any file. `// C#: Log/LogDownloadMavLink.cs:552-567`
    #[test]
    fn first_person_kml_of_a_log_not_there_says_so_and_stops() {
        let dir = scratch("firstperson-gone");
        let ran = Work::FirstPerson(dir.join("gone.log")).run();
        assert_eq!(ran.lines.len(), 2, "{:?}", ran.lines);
        assert_eq!(ran.lines[0], strings::ERROR_PROCESSING);
        assert_eq!(ran.status, None);
        assert!(ran.files.is_empty());
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    /// First Person KML of a log not named `.log`: `writeKMLFirstPerson` throws out of the
    /// handler, which says no "Done" - the status line's error box - and nothing is written.
    /// `// C#: ExtLibs/Utilities/LogOutput.cs:314-332`
    #[test]
    fn first_person_kml_of_a_log_not_named_log_is_an_error() {
        let dir = scratch("firstperson-upper");
        let log = dir.join("FLIGHT.LOG");
        mp_os::fs::copy(testdata("dataflash/synthetic.log"), &log).expect("the log copied");
        let ran = Work::FirstPerson(log).run();
        assert!(ran.lines.is_empty(), "{:?}", ran.lines);
        assert_eq!(
            ran.status,
            Some(format!(
                "Error: The process cannot access the file '{}' because it is being used by \
                 another process.",
                dir.join("FLIGHT.LOG-fp.kml").display()
            ))
        );
        assert!(ran.files.is_empty());
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    /// .bin to .log: the DataFlash page's conversion - the golden's bytes - to the file the Save
    /// As named, nothing in the output, and the status line saying where it went.
    /// `// C#: Log/LogDownloadMavLink.cs:595`
    #[test]
    fn bin_to_log_writes_the_text_where_save_as_named() {
        let dir = scratch("bintolog");
        let bin = dir.join("flight.bin");
        mp_os::fs::copy(testdata("dataflash.bin"), &bin).expect("the log copied");
        let out = dir.join("converted.log");
        let ran = Work::BinToLog(bin, out.clone()).run();
        assert!(ran.lines.is_empty(), "{:?}", ran.lines);
        assert_eq!(ran.status, Some(format!(".bin to .log: {}", out.display())));
        assert_eq!(ran.files, [(out.clone(), 868_884)]);
        assert_eq!(
            mp_os::fs::read(&out).expect("the .log"),
            mp_os::fs::read(testdata("dataflash/golden/dataflash.log")).expect("the golden")
        );
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    /// A tool works on a thread of its own: the KML tools' "Processing" line said first, after
    /// two empty lines; nothing else starts while it works; its lines are said when its answer
    /// comes. `// C#: Log/LogDownloadMavLink.cs:489-491, 546-548`
    #[test]
    fn a_tool_works_on_a_thread_and_says_what_it_did_when_done() {
        let dir = scratch("thread");
        let gone = dir.join("gone.log");
        let mut logs = LogDownloader::default();
        assert_eq!(logs.start(Work::FirstPerson(gone.clone())), Ok(()));
        assert!(logs.working());
        let processing = format!("Processing {}", gone.display());
        assert_eq!(logs.output, [String::new(), String::new(), processing]);
        assert_eq!(logs.start(Work::RedoKml(gone)), Ok(()));
        assert_eq!(logs.output.len(), 3, "nothing starts while one works");
        let deadline = Instant::now() + Duration::from_secs(60);
        while logs.working() {
            assert_eq!(logs.poll(), None);
            assert!(Instant::now() < deadline, "the tool never finished");
            wasm_thread::sleep(Duration::from_millis(10));
        }
        assert_eq!(logs.output.len(), 5, "{:?}", logs.output);
        assert_eq!(logs.output[3], strings::ERROR_PROCESSING);
        assert_eq!(
            logs.last.as_ref().map(|(tool, _)| *tool),
            Some(Tool::FirstPerson)
        );
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    /// The tools' and Clear Logs' words are the C#'s: the buttons' from the form's `.resx`, the
    /// lines from `LogStrings.resx`, the filters and the question from the handlers.
    #[test]
    fn the_tools_words_are_the_csharps() {
        use crate::config_coverage::source::{csharp, resx};
        let (Some(form), Some(words), Some(source)) = (
            csharp("Log/LogDownloadMavLink.resx"),
            csharp("Log/LogStrings.resx"),
            csharp("Log/LogDownloadMavLink.cs"),
        ) else {
            eprintln!("skipped: MP_SRC does not name the C# tree");
            return;
        };
        let form = resx(&form);
        for (tool, name) in [
            (Tool::FirstPerson, "BUT_firstperson"),
            (Tool::RedoKml, "BUT_redokml"),
            (Tool::BinToLog, "BUT_bintolog"),
        ] {
            assert_eq!(
                form.get(&format!("{name}.Text")).map(String::as_str),
                Some(tool.text())
            );
        }
        assert_eq!(
            form.get("BUT_clearlogs.Text").map(String::as_str),
            Some("Clear Logs")
        );
        let words = resx(&words);
        for (name, ours) in [
            ("Confirmation", strings::CONFIRMATION),
            ("EraseComplete", strings::ERASE_COMPLETE),
            ("ProcessingLog", strings::PROCESSING),
            ("ErrorProcessingLogfile", strings::ERROR_PROCESSING),
            ("Done", strings::DONE),
        ] {
            assert_eq!(words.get(name).map(String::as_str), Some(ours), "{name}");
        }
        for filter in [LOG_FILTER, BIN_FILTER, SAVE_FILTER] {
            assert!(
                source.contains(&format!("Filter = \"{filter}\";")),
                "{filter}"
            );
        }
        assert!(source.contains(&format!(
            "CustomMessageBox.Show(LogStrings.Confirmation, \"{}\", MessageBoxButtons.YesNo)",
            strings::SURE
        )));
    }
}
