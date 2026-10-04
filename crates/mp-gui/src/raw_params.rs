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

//! The Full Parameter List's remainder: `ConfigRawParams`'s Reset to Default, Load Presaved and
//! its file list, Commit Params, the Modified filter, Refresh Table and the tree's collapse.
//!
//! The screen itself - the group list, the rows, the editor and the file panel - is `params.rs`;
//! this is the rest of the C#'s right-hand column and the state behind it.
//!
//! * Reset to Default asks "Reset all parameters to default\nAre you sure!!" under "Reset", then
//!   `setParam(new[] {"FORMAT_VERSION", "SYSID_SW_MREV"}, 0)` - the first of the two the vehicle
//!   has listed -, a second's wait, `doReboot(false, true)`, the port closed, and "Your board is
//!   now rebooting, You will be required to reconnect to the autopilot." Here the write is the
//!   link's retrying `setParam`, the wait a clock read each frame rather than a blocked thread,
//!   and the `catch`'s `Strings.ErrorCommunicating` box goes on the status line, as the owner
//!   ruled on 2026-09-25 for an error the window can show as state. The question and the report
//!   keep their boxes.
//! * Load Presaved: on `Activate`, `updatedefaultlist` lists ArduPilot's
//!   `Tools/Frame_params` (`QuadPlanes/` for a vehicle with `Q_ENABLE` set) through GitHub's
//!   contents API into `CMB_paramfiles`, once a process; the button fetches the one chosen,
//!   saves it in the user data directory, reads it with `loadParamFile` and opens `ParamCompare`
//!   over the vehicle's table and it.
//! * Commit Params: `PREFLIGHT_STORAGE` with 1, shown only when the display view's
//!   `displayParamCommitButton` is - which Mission Planner's own views never set.
//! * Modified: only the rows in `_changes`, the edits not yet written.
//! * Refresh Table (shown with the Planner page's Slow Machine ticked): the rows made again.
//! * `but_collapse`: the tree hidden, and its prefix dropped, so every row shows.
//!
//! `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeMap;
use std::path::Path;
use std::sync::mpsc::{Receiver, TryRecvError};
use web_time::{Duration, Instant};

use gpui::{AnyElement, Context, SharedString, Window, div, prelude::*, px, rgb};
use mp_firmware::manifest::Fetch;
use mp_link::RequestId;
use mp_link::requests::RequestOutcome;

use crate::MissionPlanner;
use crate::config::param_compare::{self, ParamCompare};
use crate::config::servo_output::{self, Combo};
use crate::fly::{error_box, strings};
use crate::params::Written;
use crate::telemetry::{Lookup, Report, Telemetry};
use crate::ui::{action, theme};

/// `BUT_reset_params.Text`. `// C#: GCSViews/ConfigurationView/ConfigRawParams.resx`
pub const RESET_TO_DEFAULT: &str = "Reset to Default";
/// The question's caption. `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:983`
pub const RESET_CAPTION: &str = "Reset";
/// The question. `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:983`
pub const RESET_QUESTION: &str = "Reset all parameters to default\nAre you sure!!";
/// What is said once the reboot has gone and the port is closed.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:994-995`
pub const REBOOTING: &str =
    "Your board is now rebooting, You will be required to reconnect to the autopilot.";
/// The names `setParam` is given, in order; the first the vehicle has is written.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:988`
pub const RESET_NAMES: [&str; 2] = ["FORMAT_VERSION", "SYSID_SW_MREV"];
/// `Thread.Sleep(1000)` between the write and the reboot.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:989`
pub const RESET_WAIT: Duration = Duration::from_secs(1);
/// How long the port stays open after the reboot is queued. `doCommand` has written both of its
/// frames before `BaseStream.Close()` runs; here the link thread writes them on its next pass,
/// and a link closed before then drops them with the queue.
pub const CLOSE_GRACE: Duration = Duration::from_millis(250);

/// `BUT_commitToFlash.Text`. `// C#: GCSViews/ConfigurationView/ConfigRawParams.resx`
pub const COMMIT_PARAMS: &str = "Commit Params";
/// `MAV_CMD.PREFLIGHT_STORAGE`. `// C#: ExtLibs/Mavlink/Mavlink.cs:1099`
pub const PREFLIGHT_STORAGE: u16 = 245;
/// `doCommand(..., PREFLIGHT_STORAGE, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f)`.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1097`
pub const COMMIT: [f32; 7] = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
/// Said when the command was answered - or refused, whose `false` the C# does not look at.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1105`
pub const COMMITTED: &str = "Parameters committed to non-volatile memory";
/// The `catch`'s box. `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1099-1103`
pub const INVALID_COMMAND: &str = "Invalid command";
/// The display view's switch Commit Params is shown by. `// C#: ConfigRawParams.cs:61`
pub const COMMIT_FLAG: &str = "displayParamCommitButton";

/// `chk_modified.Text`. `// C#: GCSViews/ConfigurationView/ConfigRawParams.resx`
pub const MODIFIED: &str = "Modified";
/// `BUT_refreshTable.Text`. `// C#: GCSViews/ConfigurationView/ConfigRawParams.resx`
pub const REFRESH_TABLE: &str = "Refresh Table";
/// The setting Refresh Table is shown by, the Planner page's Slow Machine.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:62`
pub const SLOW_MACHINE: &str = "SlowMachine";
/// `BUT_paramfileload.Text`. `// C#: GCSViews/ConfigurationView/ConfigRawParams.resx`
pub const LOAD_PRESAVED: &str = "Load Presaved";
/// What the load says when it fails, before the exception's text.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:970-973`
pub const LOAD_FAILED: &str = "Failed to load file.";
/// Where the tree's state is kept. `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:85, 113`
pub const COLLAPSED_KEY: &str = "rawparam_panel1collapsed";
/// `but_collapse.Text` with the tree showing, and with it collapsed.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.resx; ConfigRawParams.cs:1136-1146`
pub const COLLAPSE: [&str; 2] = ["<", ">"];

/// `GitHubContent.githubapiurl`. `// C#: ExtLibs/Utilities/GitHubContent.cs:21`
pub const GITHUB_API: &str = "https://api.github.com/repos";

/// `Settings.GetBoolean`: `bool.TryParse` of the value - "true" or "false" in any case, with
/// white space around it - or `false`.
/// `// C#: ExtLibs/Utilities/Settings.cs:223-232`
#[must_use]
pub fn get_boolean(value: Option<&str>) -> bool {
    value.is_some_and(|value| value.trim().eq_ignore_ascii_case("true"))
}

// --- The screen's state ------------------------------------------------------------------------

/// What the C#'s right-hand column holds between presses.
#[derive(Debug, Default)]
pub struct RawParams {
    /// Whether the screen is showing: between its `Activate` and its `Deactivate`.
    active: bool,
    /// `_changes`.
    changes: BTreeMap<String, f64>,
    /// `chk_modified.Checked`.
    modified: bool,
    /// `splitContainer1.Panel1Collapsed`.
    collapsed: bool,
    /// Reset to Default's question, showing.
    question: bool,
    /// A box showing: the reboot's report.
    message: Option<&'static str>,
    /// A reset under way.
    reset: Option<Reset>,
    /// `CMB_paramfiles` and Load Presaved.
    presets: Presets,
    /// `ParamCompare`, open over a presaved file.
    compare: Option<ParamCompare>,
}

impl RawParams {
    /// `Activate`'s part: `_changes` cleared, the tree as it was left, and the presaved list
    /// asked for - the combo box and its button disabled until it comes.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:50-97`
    pub fn activate(&mut self, collapsed: Option<&str>, parameters: &[(String, f64)]) {
        self.active = true;
        self.changes.clear();
        self.collapsed = get_boolean(collapsed);
        self.presets.activate(parameters);
    }

    /// `Deactivate`'s part: the tree's state, for the next `Activate`, as `bool.ToString()`.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:99-114`
    pub fn deactivate(&mut self) -> (&'static str, &'static str) {
        self.active = false;
        self.presets.open = false;
        (COLLAPSED_KEY, if self.collapsed { "True" } else { "False" })
    }

    /// Whether the screen is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// `_changes`: every value edited whose write the vehicle has not answered, or never did.
    ///
    /// In the C# a value edited in the grid waits there, green, until Write Params sends it, and
    /// leaves `_changes` once its `setParam` returns; a `setParam` that throws leaves it in
    /// (`ConfigRawParams.cs:313-364, 520`). This screen writes a value as soon as it is edited -
    /// the editor's step, a file's differences, a presaved file's rows - so the same set is the
    /// writes queued or in flight and the ones that timed out. It holds a name from the edit
    /// until the vehicle echoes it, which is the C#'s "not yet written", and keeps one whose
    /// every retry went unanswered, as the C#'s `catch` does.
    #[must_use]
    pub const fn changes(&self) -> &BTreeMap<String, f64> {
        &self.changes
    }

    /// Writes started: each in `_changes` with the value asked for, `_changes[name] = newvalue`.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:520`
    pub fn queued<'a>(&mut self, writes: impl Iterator<Item = (&'a str, f64)>) {
        for (name, value) in writes {
            self.changes.insert(name.to_owned(), value);
        }
    }

    /// A write ended: out of `_changes` unless it timed out, which is `setParam`'s exception and
    /// skips the `_changes.Remove`.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:323-363`
    pub fn written(&mut self, written: &Written) {
        if written.outcome != RequestOutcome::TimedOut {
            self.changes.remove(&written.name);
        }
    }

    /// `chk_modified.Checked`.
    #[must_use]
    pub const fn modified(&self) -> bool {
        self.modified
    }

    /// `splitContainer1.Panel1Collapsed`.
    #[must_use]
    pub const fn collapsed(&self) -> bool {
        self.collapsed
    }

    /// The presaved files' list and its button.
    #[must_use]
    pub const fn presets(&self) -> &Presets {
        &self.presets
    }

    /// `ParamCompare`, when it is open.
    #[must_use]
    pub const fn compare(&self) -> Option<&ParamCompare> {
        self.compare.as_ref()
    }

    /// Whether a reset is under way.
    #[must_use]
    pub const fn resetting(&self) -> bool {
        self.reset.is_some()
    }

    /// The question showing, if it is.
    #[must_use]
    pub const fn question(&self) -> bool {
        self.question
    }

    /// The box showing, if one is.
    #[must_use]
    pub const fn message(&self) -> Option<&'static str> {
        self.message
    }

    /// `but_collapse_Click`: the tree hidden or shown. Hiding it sets `filterPrefix = ""`, which
    /// the button's handler does by dropping the group chosen; showing it again runs `BuildTree`,
    /// which starts with nothing chosen.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1134-1149`
    pub fn click_collapse(&mut self) {
        self.collapsed = !self.collapsed;
    }

    /// `chk_modified` clicked.
    pub fn click_modified(&mut self) {
        self.modified = !self.modified;
    }
}

// --- Reset to Default ----------------------------------------------------------------------------

/// Where a reset is.
#[derive(Debug, Clone, Copy)]
enum Step {
    /// `setParam(name, 0)`, retried by the link until the vehicle echoes it.
    Writing {
        /// The name written.
        name: &'static str,
        /// The link's request.
        id: RequestId,
        /// When it was made, for [`Telemetry::lookup`].
        made: Instant,
    },
    /// `Thread.Sleep(1000)`.
    Sleeping {
        /// When it ends.
        until: Instant,
    },
    /// The reboot queued; the port closes once the link has written it.
    Closing {
        /// When the port closes.
        at: Instant,
    },
}

/// What one turn of a reset has to say.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ResetProgress {
    /// Still going.
    Waiting,
    /// The reboot has gone: close the port and say so.
    Close,
    /// The `catch`: what the status line says.
    Failed(String),
}

/// Reset to Default, from the Yes to the closed port.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:980-1003`
#[derive(Debug)]
pub struct Reset {
    step: Step,
}

/// The `catch`'s box, `Strings.ErrorCommunicating + "\n" + ex` under `Strings.ERROR`, on the
/// status line on one line.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:997-1001`
fn communicating(why: &str) -> String {
    error_box(format!("{} {why}", strings::ERROR_COMMUNICATING))
}

impl Reset {
    /// The Yes: `setParam(new[] {"FORMAT_VERSION", "SYSID_SW_MREV"}, 0)`. That overload tries
    /// each name in turn and stops at the first `setParam` that returns true; `setParam` returns
    /// false, sending nothing, for a name the vehicle has not listed - so the first name listed
    /// is the one written. A vehicle with neither returns false for both, which is no
    /// exception, and the reset goes on to the wait and the reboot.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:988; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1609-1620, 1640-1644`
    ///
    /// # Errors
    /// No link to write on: the `catch`'s text.
    pub fn start(telemetry: &Telemetry, now: Instant) -> Result<Self, String> {
        let Some(name) = RESET_NAMES
            .into_iter()
            .find(|name| telemetry.holds_parameter(name))
        else {
            return Ok(Self {
                step: Step::Sleeping {
                    until: now + RESET_WAIT,
                },
            });
        };
        let id = telemetry
            .write_parameter(name, 0.0, false)
            .ok_or_else(|| communicating("The port is closed."))?;
        Ok(Self {
            step: Step::Writing {
                name,
                id,
                made: now,
            },
        })
    }

    /// Moves on as far as the link and the clock allow.
    pub fn advance(&mut self, telemetry: &Telemetry, now: Instant) -> ResetProgress {
        match self.step {
            Step::Writing { name, id, made } => {
                let outcome = match telemetry.lookup(id, made) {
                    Lookup::PickingUp => return ResetProgress::Waiting,
                    Lookup::Gone => RequestOutcome::TimedOut,
                    Lookup::Found(request) => match request.outcome() {
                        None => return ResetProgress::Waiting,
                        Some(outcome) => outcome,
                    },
                };
                // Every retry unanswered is `setParam`'s `TimeoutException`; any other end is
                // a return, true or false, and the reset goes on.
                // `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1748-1770`
                if outcome == RequestOutcome::TimedOut {
                    return ResetProgress::Failed(communicating(&format!(
                        "Timeout on read - setParam {name}"
                    )));
                }
                self.step = Step::Sleeping {
                    until: now + RESET_WAIT,
                };
                ResetProgress::Waiting
            }
            Step::Sleeping { until } if now < until => ResetProgress::Waiting,
            // `doReboot(false, true)`: PREFLIGHT_REBOOT_SHUTDOWN with 1 to the vehicle shown.
            // `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2553-2567`
            Step::Sleeping { .. } => {
                if telemetry.reboot() {
                    self.step = Step::Closing {
                        at: now + CLOSE_GRACE,
                    };
                    ResetProgress::Waiting
                } else {
                    ResetProgress::Failed(communicating("The port is closed."))
                }
            }
            Step::Closing { at } if now < at => ResetProgress::Waiting,
            Step::Closing { .. } => ResetProgress::Close,
        }
    }
}

// --- Commit Params -------------------------------------------------------------------------------

/// What Commit Params says: "Invalid command" when every retry goes unanswered - `doCommand`'s
/// `TimeoutException` - and "Parameters committed to non-volatile memory" otherwise, the
/// command refused included, since the C# does not look at `doCommand`'s `false`. Both are
/// boxes in the C#; here they go on the status line as every `doCommand`'s words do
/// ([`Report`]), and "Invalid command" is an error the owner ruled (2026-09-25) is never a box.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1093-1107`
#[must_use]
pub fn commit_report() -> Report {
    Report {
        timed_out: Some(INVALID_COMMAND.to_owned()),
        refused: Some(COMMITTED.to_owned()),
        accepted: Some(COMMITTED.to_owned()),
        fallback: None,
    }
}

/// `BUT_commitToFlash_Click`: `PREFLIGHT_STORAGE` with 1 to the vehicle shown, through the
/// link's retrying `doCommand`. `None` with no vehicle, which the C#'s `doCommand` meets as an
/// exception on the closed port: "Invalid command".
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1093-1107`
pub fn commit_params(telemetry: &mut Telemetry) -> Option<RequestId> {
    let (_, vehicle) = telemetry.send_handle()?;
    telemetry.command(vehicle, PREFLIGHT_STORAGE, COMMIT, commit_report())
}

// --- Load Presaved -------------------------------------------------------------------------------

/// One of `paramfiles`: a `GitHubContent.FileInfo`'s `name` and `path`.
/// `// C#: ExtLibs/Utilities/GitHubContent.cs:26-38`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PresetFile {
    /// `name`, which the combo box shows and the file is saved as.
    pub name: String,
    /// `path` in the repository, which the file is fetched by.
    pub path: String,
}

/// `updatedefaultlist`'s URL: `GetDirContent("ardupilot", "ardupilot", "/Tools/Frame_params/" +
/// subdir, ".param")`, `subdir` "QuadPlanes/" when the vehicle has `Q_ENABLE` at 1 or more;
/// `GetDirContent` puts "/contents" before the path and trims its trailing slash.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:864-873; ExtLibs/Utilities/GitHubContent.cs:51-61`
#[must_use]
pub fn list_url(parameters: &[(String, f64)]) -> String {
    let quadplane = parameters
        .iter()
        .any(|(name, value)| name == "Q_ENABLE" && *value >= 1.0);
    let subdir = if quadplane { "QuadPlanes/" } else { "" };
    let path = format!("/contents/Tools/Frame_params/{subdir}");
    format!(
        "{GITHUB_API}/ardupilot/ardupilot{}",
        path.trim_end_matches(['/', '\\'])
    )
}

/// `GetDirContent`'s answer: each entry whose name holds ".param", in any case, in GitHub's
/// order.
/// `// C#: ExtLibs/Utilities/GitHubContent.cs:71-80`
///
/// # Errors
/// Not a JSON list.
pub fn parse_list(json: &[u8]) -> Result<Vec<PresetFile>, String> {
    let value: serde_json::Value = serde_json::from_slice(json).map_err(|err| err.to_string())?;
    let entries = value
        .as_array()
        .ok_or_else(|| "the answer is not a list".to_owned())?;
    Ok(entries
        .iter()
        .filter_map(|entry| {
            let name = entry.get("name")?.as_str()?;
            let path = entry
                .get("path")
                .and_then(serde_json::Value::as_str)
                .unwrap_or_default();
            name.to_lowercase().contains(".param").then(|| PresetFile {
                name: name.to_owned(),
                path: path.to_owned(),
            })
        })
        .collect())
}

/// `BUT_paramfileload_Click`'s URL: `GetFileContent("ArduPilot", "ardupilot", path)`.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:952-953; ExtLibs/Utilities/GitHubContent.cs:85-93`
#[must_use]
pub fn file_url(path: &str) -> String {
    if path.is_empty() {
        format!("{GITHUB_API}/ArduPilot/ardupilot/")
    } else {
        format!("{GITHUB_API}/ArduPilot/ardupilot/contents/{path}")
    }
}

/// `GetFileContent`'s answer: its `content`, `Convert.FromBase64String` - which passes over the
/// line breaks GitHub puts in it.
/// `// C#: ExtLibs/Utilities/GitHubContent.cs:101-110`
///
/// # Errors
/// Not a JSON object with a base64 `content`.
pub fn parse_file(json: &[u8]) -> Result<Vec<u8>, String> {
    use base64::Engine as _;
    let value: serde_json::Value = serde_json::from_slice(json).map_err(|err| err.to_string())?;
    let content = value
        .get("content")
        .and_then(serde_json::Value::as_str)
        .ok_or_else(|| "the answer has no content".to_owned())?;
    let compact: String = content.chars().filter(|c| !c.is_whitespace()).collect();
    base64::engine::general_purpose::STANDARD
        .decode(compact)
        .map_err(|err| err.to_string())
}

/// A presaved file read, `loadParamFile`'s names and values; or why not, as the `catch` has it.
pub type Loaded = Result<Vec<(String, f64)>, String>;

/// `updatedefaultlist`'s fetch.
///
/// # Errors
/// The fetch, or its answer.
pub fn fetch_list(fetch: &dyn Fetch, url: &str) -> Result<Vec<PresetFile>, String> {
    parse_list(&fetch.get(url)?)
}

/// `BUT_paramfileload_Click`'s `try`: the file fetched, `File.WriteAllBytes` to
/// `Settings.GetUserDataDirectory() + CMB_paramfiles.Text`, and `loadParamFile` of it.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:946-957`
///
/// # Errors
/// Anything the C#'s `catch` would meet, as text.
pub fn load_preset(fetch: &dyn Fetch, file: &PresetFile, directory: Option<&Path>) -> Loaded {
    let data = parse_file(&fetch.get(&file_url(&file.path))?)?;
    let directory = directory.ok_or_else(|| "no user data directory".to_owned())?;
    // Only the name's last part: GitHub's names have no separators, and a name that did would
    // otherwise be written outside the directory.
    let name = Path::new(&file.name)
        .file_name()
        .ok_or_else(|| format!("{} is not a file name", file.name))?;
    let path = directory.join(name);
    std::fs::write(&path, &data).map_err(|err| format!("{}: {err}", path.display()))?;
    let loaded = mp_params::param_file::ParamFile::load(&path)
        .map_err(|err| format!("{}: {err}", path.display()))?;
    Ok(loaded
        .iter()
        .map(|(name, value)| (name.to_owned(), value))
        .collect())
}

/// `CMB_paramfiles` and `BUT_paramfileload`, and the fetches behind them.
#[derive(Debug, Default)]
pub struct Presets {
    /// `paramfiles`: static, so fetched once a process - a failed fetch leaves it `null` and the
    /// next `Activate` tries again.
    files: Option<Vec<PresetFile>>,
    /// The list's fetch, under way.
    listing: Option<Receiver<Result<Vec<PresetFile>, String>>>,
    /// The combo box: `DataSource = paramfiles`, `DisplayMember = "name"`.
    combo: Combo,
    /// `BUT_paramfileload.Enabled`.
    button: bool,
    /// The combo box's list, dropped down.
    open: bool,
    /// A file's fetch, under way.
    loading: Option<Receiver<Loaded>>,
}

impl Presets {
    /// `Activate`'s part: both disabled, and `updatedefaultlist` queued - the fetch when there
    /// is no list yet, else the list bound again at once.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:64-66, 860-887`
    fn activate(&mut self, parameters: &[(String, f64)]) {
        self.combo.enabled = false;
        self.button = false;
        self.open = false;
        if self.files.is_some() {
            self.bind();
        } else if self.listing.is_none() {
            let url = list_url(parameters);
            let (send, receive) = std::sync::mpsc::channel();
            let spawned = wasm_thread::Builder::new()
                .name("presaved-params".to_owned())
                .spawn(move || {
                    let _ = send.send(fetch_list(&*mp_firmware::manifest::fetcher(), &url));
                });
            if spawned.is_ok() {
                self.listing = Some(receive);
            }
        }
    }

    /// Takes a finished list. `Ok` is `paramfiles` bound; a failure is logged, as the C#'s
    /// `catch` logs it, and leaves both disabled.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:875-886`
    fn take_list(&mut self, result: Result<Vec<PresetFile>, String>) {
        match result {
            Ok(files) => {
                self.files = Some(files);
                self.bind();
            }
            Err(why) => eprintln!("presaved parameter files: {why}"),
        }
    }

    /// The `BeginInvoke`: the list bound - its first row selected, as a bound combo box's is -
    /// and both enabled.
    fn bind(&mut self) {
        let options: Vec<(i64, String)> = self
            .files
            .iter()
            .flatten()
            .enumerate()
            .map(|(index, file)| (i64::try_from(index).unwrap_or(i64::MAX), file.name.clone()))
            .collect();
        self.combo = Combo {
            param: "CMB_paramfiles".to_owned(),
            selected: options.first().map(|(key, _)| *key),
            options,
            enabled: true,
            top_index: 0,
        };
        self.button = true;
    }

    /// The files listed, once they have come.
    #[must_use]
    pub fn files(&self) -> &[PresetFile] {
        self.files.as_deref().unwrap_or_default()
    }

    /// The combo box.
    #[must_use]
    pub const fn combo(&self) -> &Combo {
        &self.combo
    }

    /// Whether Load Presaved can be pressed: enabled, and no file on its way.
    #[must_use]
    pub const fn can_load(&self) -> bool {
        self.button && self.loading.is_none()
    }

    /// Whether a file is on its way.
    #[must_use]
    pub const fn is_loading(&self) -> bool {
        self.loading.is_some()
    }

    /// The file the combo box shows.
    #[must_use]
    pub fn selected(&self) -> Option<&PresetFile> {
        let index = usize::try_from(self.combo.selected?).ok()?;
        self.files().get(index)
    }
}

// --- The screen's part in the application -------------------------------------------------------

impl MissionPlanner {
    /// Whether the Full Parameter List is showing: the Params tab, or CONFIG's page.
    fn raw_params_showing(&self) -> bool {
        self.screen == crate::Screen::Params
            || (self.screen == crate::Screen::Config
                && self
                    .config_list
                    .page()
                    .is_some_and(|entry| entry.class == "ConfigRawParams"))
    }

    /// Once a frame: `Activate` when the screen comes into view and `Deactivate` when it leaves,
    /// the fetches taken when they finish, and a reset moved on.
    pub(crate) fn raw_params_tick(&mut self) {
        let showing = self.raw_params_showing();
        if showing && !self.raw_params.is_active() {
            let view = self.telemetry.view();
            let collapsed = self.persisted.get(COLLAPSED_KEY).map(str::to_owned);
            self.raw_params
                .activate(collapsed.as_deref(), &view.parameters);
            // ---- ConfigRawParams remainder ----
            self.param_grid_activate();
            // ---- end ConfigRawParams remainder ----
        } else if !showing && self.raw_params.is_active() {
            let (key, value) = self.raw_params.deactivate();
            self.persisted.set(key, value);
            // ---- ConfigRawParams remainder ----
            self.param_grid_deactivate();
            // ---- end ConfigRawParams remainder ----
        }
        // ---- ConfigRawParams remainder ----
        self.param_grid_tick();
        // ---- end ConfigRawParams remainder ----

        let presets = &mut self.raw_params.presets;
        if let Some(listing) = &presets.listing {
            match listing.try_recv() {
                Ok(result) => {
                    presets.listing = None;
                    presets.take_list(result);
                }
                Err(TryRecvError::Disconnected) => presets.listing = None,
                Err(TryRecvError::Empty) => {}
            }
        }
        if let Some(loading) = &presets.loading {
            let result = match loading.try_recv() {
                Ok(result) => Some(result),
                Err(TryRecvError::Disconnected) => Some(Err("the fetch ended".to_owned())),
                Err(TryRecvError::Empty) => None,
            };
            if let Some(result) = result {
                presets.loading = None;
                match result {
                    // `new ParamCompare(Params, MainV2.comPort.MAV.param, param2)`.
                    // C#: GCSViews/ConfigurationView/ConfigRawParams.cs:959-962
                    Ok(values) => {
                        let view = self.telemetry.view();
                        self.raw_params.compare =
                            Some(ParamCompare::new(&view.parameters, &values));
                    }
                    // A box in the C#; an error, so on the status line (the owner's ruling,
                    // 2026-09-25).
                    // C#: GCSViews/ConfigurationView/ConfigRawParams.cs:970-973
                    Err(why) => self.file_status = Some(format!("{LOAD_FAILED} {why}")),
                }
            }
        }

        let progress = self
            .raw_params
            .reset
            .as_mut()
            .map(|reset| reset.advance(&self.telemetry, Instant::now()));
        match progress {
            None | Some(ResetProgress::Waiting) => {}
            // `MainV2.comPort.BaseStream.Close()`, then the box.
            // C#: GCSViews/ConfigurationView/ConfigRawParams.cs:991-995
            Some(ResetProgress::Close) => {
                self.raw_params.reset = None;
                self.telemetry = Telemetry::idle();
                self.mission_requested = false;
                self.params_requested = false;
                self.raw_params.message = Some(REBOOTING);
            }
            Some(ResetProgress::Failed(text)) => {
                self.raw_params.reset = None;
                self.file_status = Some(text);
            }
        }
    }

    /// The question answered: on Yes the reset starts.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:982-988`
    fn answer_reset(&mut self, yes: bool) {
        self.raw_params.question = false;
        if !yes {
            return;
        }
        match Reset::start(&self.telemetry, Instant::now()) {
            Ok(reset) => self.raw_params.reset = Some(reset),
            Err(text) => self.file_status = Some(text),
        }
    }

    /// Commit Params pressed.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1093-1107`
    fn commit_to_flash(&mut self) {
        if commit_params(&mut self.telemetry).is_none() {
            self.file_status = Some(INVALID_COMMAND.to_owned());
        }
    }

    /// Load Presaved pressed: the file chosen fetched on a thread of its own, the C#'s UI thread
    /// blocked in `GetFileContent` being this one's frames.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:946-974`
    fn load_presaved(&mut self) {
        let presets = &mut self.raw_params.presets;
        let Some(file) = presets.selected().cloned() else {
            return;
        };
        presets.open = false;
        let directory = mp_settings::user_data_directory();
        let (send, receive) = std::sync::mpsc::channel();
        let spawned = wasm_thread::Builder::new()
            .name("presaved-param-file".to_owned())
            .spawn(move || {
                let fetch = mp_firmware::manifest::fetcher();
                let _ = send.send(load_preset(&*fetch, &file, directory.as_deref()));
            });
        match spawned {
            Ok(_) => presets.loading = Some(receive),
            Err(err) => self.file_status = Some(format!("{LOAD_FAILED} {err}")),
        }
    }

    /// `ParamCompare`'s button over the screen's grid: each row ticked put into the grid, which
    /// is the grid's `CellValueChanged` - and on this screen a value put in is a value written,
    /// so the rows ticked are written, one after another, into `_changes` until each is heard
    /// back. The form then closes with OK.
    ///
    /// The C# then shows "Loaded parameters, please make sure you write them!": the values wait
    /// in its grid for Write Params. Here they are already being written, and there is no Write
    /// Params to press, so the box would tell the operator to do something that does not exist;
    /// the writes' own summary goes on the status line instead.
    /// `// C#: Controls/paramcompare.cs:90-109; GCSViews/ConfigurationView/ConfigRawParams.cs:962-965`
    /// `BUT_save_Click` with the grid handed in (`dgv`): each ticked row's new value is put in
    /// the grid's Value cell, which is `Params_CellValueChanged` for it - the ReadOnly box, the
    /// out-of-range question, the write through `_changes` - one row after another, so the
    /// questions come in turn. A value that is no number is the C#'s `double.Parse` throwing
    /// into the `ErrorSettingParameter` box: the status line's here (the owner's ruling).
    /// `// C#: Controls/paramcompare.cs:65-108; GCSViews/ConfigurationView/ConfigRawParams.cs:453-524`
    fn save_compare(&mut self) {
        let Some(form) = self.raw_params.compare.take() else {
            return;
        };
        for row in form.rows().iter().filter(|row| row.used) {
            let name = row.name.trim().to_owned();
            if mp_log::netfmt::parse_double(&row.new_value).is_none() {
                self.file_status = Some(crate::config::compass::ERROR_SETTING_PARAMETER.to_owned());
                continue;
            }
            self.param_grid_edit(&name, row.new_value.trim());
        }
    }

    /// The facts a script reads.
    pub(crate) fn raw_params_facts(&self, parameters: &[crate::params::Parameter]) {
        use crate::facts::record;
        let raw = &self.raw_params;
        record("params.modified", raw.modified);
        record("params.changes", raw.changes.len());
        record(
            "params.changes.names",
            raw.changes.keys().cloned().collect::<Vec<_>>().join(","),
        );
        record("params.collapsed", raw.collapsed);
        record("params.active", raw.active);
        record(
            "params.refresh_table.visible",
            get_boolean(self.persisted.get(SLOW_MACHINE)),
        );
        record("params.commit.visible", crate::setup::display(COMMIT_FLAG));
        let (built, refreshes) = crate::params::tables_built();
        record("params.table.built", built);
        record("params.table.refreshes", refreshes);
        record("params.presets", raw.presets.files().len());
        record("params.presets.enabled", raw.presets.can_load());
        record(
            "params.presets.selected",
            raw.presets.selected().map_or("", |file| file.name.as_str()),
        );
        record("params.reset.question", raw.question);
        record("params.reset.running", raw.reset.is_some());
        record("params.message", raw.message.unwrap_or(""));
        record("params.compare.open", raw.compare.is_some());
        let filters = crate::params::Filters {
            none_default: self.param_none_default,
            modified: raw.modified,
            changes: &raw.changes,
            collapsed: raw.collapsed,
        };
        record(
            "params.shown",
            crate::params::shown(
                parameters,
                self.selected_param_group.as_deref(),
                self.param_search.value(),
                &filters,
            )
            .map_or(0, |shown| shown.len()),
        );
    }
}

// --- Drawing -------------------------------------------------------------------------------------

/// A check box and its text, as `chk_none_default` is drawn beside it.
fn check_box(
    id: &'static str,
    text: &'static str,
    checked: bool,
    on_click: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    crate::probe::measured(id, div())
        .id(id)
        .flex()
        .items_center()
        .gap_1()
        .text_xs()
        .cursor_pointer()
        .text_color(rgb(if checked { theme::ACCENT } else { theme::TEXT }))
        .child(
            div()
                .size(px(13.0))
                .border_1()
                .border_color(rgb(theme::BORDER))
                .flex()
                .items_center()
                .justify_center()
                .children(checked.then(|| div().size(px(7.0)).bg(rgb(theme::ACCENT)))),
        )
        .child(text)
        .on_click(cx.listener(move |this, _event, _window, cx| {
            on_click(this);
            cx.notify();
        }))
        .into_any_element()
}

/// `but_collapse`: "<" with the tree showing, ">" with it collapsed. Collapsing drops the tree's
/// prefix, which here is the group chosen.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1134-1149`
pub fn collapse_button(collapsed: bool, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let [open, closed] = COLLAPSE;
    crate::probe::measured("param-collapse", div())
        .id("param-collapse")
        .w(px(18.0))
        .h(px(18.0))
        .flex()
        .items_center()
        .justify_center()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::ACTION))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .hover(|style| style.border_color(rgb(theme::ACCENT)))
        .child(if collapsed { closed } else { open })
        .on_click(cx.listener(|this, _event, _window, cx| {
            this.raw_params.click_collapse();
            if this.raw_params.collapsed() {
                this.selected_param_group = None;
            }
            cx.notify();
        }))
        .into_any_element()
}

/// The combo box's place: `CMB_paramfiles.Size`, 110 x 21.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.resx`
const COMBO_SIZE: (f32, f32) = (110.0, 21.0);

/// The rest of the C#'s right-hand column: Commit Params (when the display view shows it), the
/// presaved files and Load Presaved, Reset to Default, Modified, and Refresh Table (with Slow
/// Machine ticked), in the Designer's top-to-bottom order.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.resx; ConfigRawParams.cs:58-66`
pub fn controls(
    raw: &RawParams,
    connected: bool,
    slow_machine: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (width, height) = COMBO_SIZE;
    let presets = raw.presets();
    let mut combo = div()
        .relative()
        .w(px(width))
        .h(px(height))
        .child(servo_output::combo_box(
            "param-presets".to_owned(),
            presets.combo(),
            (0.0, 0.0, width, height),
            |this: &mut MissionPlanner| {
                let presets = &mut this.raw_params.presets;
                presets.open = !presets.open;
                if presets.open {
                    presets.combo.open_list();
                }
            },
            cx,
        ));
    if presets.open {
        combo = combo.child(servo_output::dropdown(
            "param-presets",
            presets.combo(),
            (0.0, height, width),
            |this: &mut MissionPlanner, key| {
                let presets = &mut this.raw_params.presets;
                presets.combo.select(key);
                presets.open = false;
            },
            |this: &mut MissionPlanner, lines| this.raw_params.presets.combo.scroll_list(lines),
            cx,
        ));
    }

    let mut row = div().flex().flex_wrap().items_center().gap_2();
    if crate::setup::display(COMMIT_FLAG) {
        row = row.child(action(
            "param-commit",
            COMMIT_PARAMS,
            theme::WARN,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.commit_to_flash();
                cx.notify();
            }),
        ));
    }
    row = row
        .child(combo)
        .child(action(
            "param-load-presaved",
            if presets.is_loading() {
                SharedString::from(format!("{LOAD_PRESAVED}..."))
            } else {
                SharedString::from(LOAD_PRESAVED)
            },
            theme::ACCENT,
            presets.can_load(),
            cx.listener(|this, _event: &(), _window, cx| {
                this.load_presaved();
                cx.notify();
            }),
        ))
        // `BUT_reset_params.Enabled = MainV2.comPort.BaseStream.IsOpen`.
        // C#: GCSViews/ConfigurationView/ConfigRawParams.cs:60
        .child(action(
            "param-reset",
            RESET_TO_DEFAULT,
            theme::ALERT,
            connected && !raw.resetting(),
            cx.listener(|this, _event: &(), _window, cx| {
                this.raw_params.question = true;
                cx.notify();
            }),
        ))
        .child(check_box(
            "param-modified",
            MODIFIED,
            raw.modified(),
            |this| this.raw_params.click_modified(),
            cx,
        ));
    // `BUT_refreshTable.Visible = Settings.Instance.GetBoolean("SlowMachine", false)`.
    // C#: GCSViews/ConfigurationView/ConfigRawParams.cs:62
    if slow_machine {
        row = row.child(action(
            "param-refresh-table",
            REFRESH_TABLE,
            theme::ACCENT,
            true,
            cx.listener(|_this, _event: &(), _window, cx| {
                crate::params::refresh_table();
                cx.notify();
            }),
        ));
    }
    row.into_any_element()
}

/// The boxes over the window: Reset to Default's question, the reboot's report, and
/// `ParamCompare` over a presaved file.
pub fn overlays(
    raw: &RawParams,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    let mut shown = Vec::new();
    if raw.question() {
        let buttons = vec![
            action(
                "param-reset-yes",
                "Yes",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.answer_reset(true);
                    cx.notify();
                }),
            ),
            action(
                "param-reset-no",
                "No",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.answer_reset(false);
                    cx.notify();
                }),
            ),
        ];
        shown.push(servo_output::modal(
            "param-reset-question",
            RESET_CAPTION,
            RESET_QUESTION,
            false,
            buttons,
            window,
        ));
    }
    if let Some(text) = raw.message() {
        let ok = action(
            "param-message-ok",
            "OK",
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.raw_params.message = None;
                cx.notify();
            }),
        );
        // `CustomMessageBox.Show(text)`: no caption.
        shown.push(servo_output::modal(
            "param-message",
            "",
            text,
            false,
            vec![ok],
            window,
        ));
    }
    if let Some(form) = raw.compare() {
        shown.push(param_compare::dialog(
            form,
            false,
            param_compare::Handlers {
                toggle_all: |this: &mut MissionPlanner| {
                    if let Some(form) = &mut this.raw_params.compare {
                        form.click_toggle_all();
                    }
                },
                toggle_row: |this: &mut MissionPlanner, index| {
                    if let Some(form) = &mut this.raw_params.compare {
                        form.toggle_row(index);
                    }
                },
                save: |this: &mut MissionPlanner| this.save_compare(),
                close: |this: &mut MissionPlanner| this.raw_params.compare = None,
            },
            window,
            cx,
        ));
    }
    shown
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::scripted::{INT32, VEHICLE, Vehicle, ack, param, until};
    use mp_link::ProtocolTimeouts;
    use mp_mavlink_dialects::all::MavMessage;

    /// The link's waits, divided so a test runs the C#'s whole retry ladder in a blink.
    fn fast() -> ProtocolTimeouts {
        ProtocolTimeouts::default().faster(20)
    }

    /// Runs a reset to its end, the vehicle answering with `answer`, the clock moved on by
    /// `step` each turn so the second's wait passes at once.
    fn run_reset(
        reset: &mut Reset,
        telemetry: &Telemetry,
        vehicle: &mut Vehicle,
        mut answer: impl FnMut(&mut Vehicle, &MavMessage),
    ) -> ResetProgress {
        let mut now = Instant::now();
        let mut last = ResetProgress::Waiting;
        until("the reset to end", || {
            for message in vehicle.read() {
                answer(&mut *vehicle, &message);
            }
            now += Duration::from_millis(100);
            last = reset.advance(telemetry, now);
            last != ResetProgress::Waiting
        });
        // What the link wrote before the port closed.
        wasm_thread::sleep(Duration::from_millis(50));
        vehicle.read();
        last
    }

    /// The commands the vehicle heard: their number and first parameter.
    fn commands(vehicle: &Vehicle) -> Vec<(u16, f32)> {
        vehicle
            .heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::CommandLong(long) => Some((long.command, long.param1)),
                _ => None,
            })
            .collect()
    }

    /// The names the vehicle was asked to set, and to what.
    fn sets(vehicle: &Vehicle) -> Vec<(String, f32)> {
        vehicle
            .heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::ParamSet(set) => {
                    Some((mp_params::decode_param_id(&set.param_id), set.param_value))
                }
                _ => None,
            })
            .collect()
    }

    /// Yes: `FORMAT_VERSION` set to 0 and echoed, the wait, `PREFLIGHT_REBOOT_SHUTDOWN` with 1 on
    /// the wire twice - `doCommand`'s second send for a reboot - and then the port to close.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:986-995; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2758-2763`
    #[test]
    fn a_reset_zeroes_format_version_waits_and_reboots() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&param("FORMAT_VERSION", 120.0, INT32));
        until("FORMAT_VERSION", || {
            telemetry.holds_parameter("FORMAT_VERSION")
        });

        let mut reset = Reset::start(&telemetry, Instant::now()).expect("a link");
        let end = run_reset(&mut reset, &telemetry, &mut vehicle, |vehicle, message| {
            if let MavMessage::ParamSet(set) = message {
                vehicle.send(&param("FORMAT_VERSION", set.param_value, INT32));
            }
        });

        assert_eq!(end, ResetProgress::Close);
        assert_eq!(sets(&vehicle), [("FORMAT_VERSION".to_owned(), 0.0)]);
        assert_eq!(commands(&vehicle), [(246, 1.0), (246, 1.0)]);
    }

    /// A vehicle without `FORMAT_VERSION` has `SYSID_SW_MREV` written: the overload's second name.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1609-1620`
    #[test]
    fn a_reset_writes_the_second_name_when_the_first_is_not_listed() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&param("SYSID_SW_MREV", 120.0, INT32));
        until("SYSID_SW_MREV", || {
            telemetry.holds_parameter("SYSID_SW_MREV")
        });

        let mut reset = Reset::start(&telemetry, Instant::now()).expect("a link");
        let end = run_reset(&mut reset, &telemetry, &mut vehicle, |vehicle, message| {
            if let MavMessage::ParamSet(set) = message {
                vehicle.send(&param("SYSID_SW_MREV", set.param_value, INT32));
            }
        });

        assert_eq!(end, ResetProgress::Close);
        assert_eq!(sets(&vehicle), [("SYSID_SW_MREV".to_owned(), 0.0)]);
        assert_eq!(commands(&vehicle), [(246, 1.0), (246, 1.0)]);
    }

    /// Neither name listed: `setParam` returns false twice, which is no exception, and the
    /// board is rebooted all the same.
    #[test]
    fn a_reset_with_neither_name_still_reboots() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut reset = Reset::start(&telemetry, Instant::now()).expect("a link");
        let end = run_reset(&mut reset, &telemetry, &mut vehicle, |_, _| {});
        assert_eq!(end, ResetProgress::Close);
        assert!(sets(&vehicle).is_empty());
        assert_eq!(commands(&vehicle), [(246, 1.0), (246, 1.0)]);
    }

    /// A write never echoed is `setParam`'s `TimeoutException`: the `catch`'s words, and no
    /// reboot.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:997-1001`
    #[test]
    fn a_reset_whose_write_is_never_echoed_says_so_and_does_not_reboot() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&param("FORMAT_VERSION", 120.0, INT32));
        until("FORMAT_VERSION", || {
            telemetry.holds_parameter("FORMAT_VERSION")
        });

        let mut reset = Reset::start(&telemetry, Instant::now()).expect("a link");
        let end = run_reset(&mut reset, &telemetry, &mut vehicle, |_, _| {});

        assert_eq!(
            end,
            ResetProgress::Failed(
                "Error: Error communicating with the autopilot Timeout on read - setParam \
                 FORMAT_VERSION"
                    .to_owned()
            )
        );
        assert_eq!(sets(&vehicle).len(), 4, "the first send and three retries");
        assert!(commands(&vehicle).is_empty());
    }

    /// No link: the Yes meets a closed port.
    #[test]
    fn a_reset_without_a_link_fails_at_once() {
        let telemetry = Telemetry::idle();
        let mut reset = Reset::start(&telemetry, Instant::now()).expect("no name to write");
        let later = Instant::now() + Duration::from_secs(2);
        assert_eq!(
            reset.advance(&telemetry, later),
            ResetProgress::Failed(
                "Error: Error communicating with the autopilot The port is closed.".to_owned()
            )
        );
    }

    /// Commit Params: `PREFLIGHT_STORAGE` with 1 and zeros, and the C#'s words for the answer
    /// and for none.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1093-1107`
    #[test]
    fn commit_sends_preflight_storage_and_says_what_the_csharp_says() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        commit_params(&mut telemetry).expect("a vehicle");
        let mut said = Vec::new();
        until("the commit to be answered", || {
            for message in vehicle.read() {
                if let MavMessage::CommandLong(long) = message {
                    vehicle.send(&ack(long.command, 0));
                }
            }
            said.extend(telemetry.take_reports());
            !said.is_empty()
        });
        assert_eq!(said, [COMMITTED]);
        let MavMessage::CommandLong(long) = vehicle
            .heard
            .iter()
            .find(|message| matches!(message, MavMessage::CommandLong(_)))
            .expect("the command")
        else {
            unreachable!()
        };
        assert_eq!(long.command, PREFLIGHT_STORAGE);
        assert_eq!(
            [
                long.param1,
                long.param2,
                long.param3,
                long.param4,
                long.param5,
                long.param6,
                long.param7
            ],
            COMMIT
        );
        assert_eq!(
            (long.target_system, long.target_component),
            (VEHICLE.sysid, VEHICLE.compid)
        );

        // Refused: `doCommand`'s false, which the C# does not look at.
        commit_params(&mut telemetry).expect("a vehicle");
        let mut said = Vec::new();
        until("the refusal", || {
            for message in vehicle.read() {
                if let MavMessage::CommandLong(long) = message {
                    vehicle.send(&ack(long.command, 4));
                }
            }
            said.extend(telemetry.take_reports());
            !said.is_empty()
        });
        assert_eq!(said, [COMMITTED]);

        // Never answered: the `catch`.
        commit_params(&mut telemetry).expect("a vehicle");
        let mut said = Vec::new();
        until("the retries to run out", || {
            vehicle.read();
            said.extend(telemetry.take_reports());
            !said.is_empty()
        });
        assert_eq!(said, [INVALID_COMMAND]);
        assert!(commit_params(&mut Telemetry::idle()).is_none());
    }

    /// Commit Params shows only under `displayParamCommitButton`, which the Advanced view -
    /// the only one this application has - leaves false.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:61; ExtLibs/Utilities/DisplayView.cs:432`
    #[test]
    fn commit_params_is_hidden_in_the_advanced_view() {
        assert!(!crate::setup::display(COMMIT_FLAG));
    }

    /// `_changes`: in when a write starts, out when it is heard back or refused, kept when it
    /// timed out; `Activate` clears it.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:56, 323-363, 520`
    #[test]
    fn changes_hold_what_is_not_yet_written() {
        let mut raw = RawParams::default();
        raw.queued([("RTL_ALT", 2000.0), ("WPNAV_SPEED", 750.0)].into_iter());
        assert_eq!(raw.changes().len(), 2);
        let ended = |name: &str, outcome| Written {
            name: name.to_owned(),
            outcome,
            sends: 1,
        };
        raw.written(&ended("RTL_ALT", RequestOutcome::Accepted { value: None }));
        raw.written(&ended("WPNAV_SPEED", RequestOutcome::TimedOut));
        assert_eq!(
            raw.changes().iter().collect::<Vec<_>>(),
            [(&"WPNAV_SPEED".to_owned(), &750.0)]
        );
        raw.written(&ended("WPNAV_SPEED", RequestOutcome::Unchanged));
        assert!(raw.changes().is_empty());
        raw.queued([("RTL_ALT", 1.0)].into_iter());
        raw.activate(None, &[]);
        assert!(raw.changes().is_empty());
    }

    /// The screen's own writes feed `_changes`: a write never echoed stays in it, one echoed
    /// leaves it - through the list of writes and the real link.
    #[test]
    fn a_write_never_echoed_stays_modified() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&param("RTL_ALT", 1500.0, INT32));
        until("RTL_ALT", || telemetry.holds_parameter("RTL_ALT"));
        vehicle.send(&param("WPNAV_SPEED", 500.0, INT32));
        until("WPNAV_SPEED", || telemetry.holds_parameter("WPNAV_SPEED"));

        let mut raw = RawParams::default();
        let mut writes = crate::params::ParamWrites::apply(
            [
                ("RTL_ALT".to_owned(), 2000.0),
                ("WPNAV_SPEED".to_owned(), 750.0),
            ],
            0,
        );
        raw.queued(writes.queued());
        until("the writes to finish", || {
            for message in vehicle.read() {
                if let MavMessage::ParamSet(set) = message
                    && mp_params::decode_param_id(&set.param_id) == "WPNAV_SPEED"
                {
                    vehicle.send(&param("WPNAV_SPEED", set.param_value, INT32));
                }
            }
            if let Some(written) = writes.advance(&telemetry).written {
                raw.written(&written);
            }
            writes.is_finished()
        });
        assert_eq!(raw.changes().keys().collect::<Vec<_>>(), ["RTL_ALT"]);
    }

    fn parameter(name: &str, value: f64, default: Option<f64>) -> crate::params::Parameter {
        crate::params::Parameter {
            name: name.to_owned(),
            value,
            meta: None,
            default,
        }
    }

    fn names(shown: Option<Vec<&crate::params::Parameter>>) -> Option<Vec<String>> {
        shown.map(|shown| shown.iter().map(|p| p.name.clone()).collect())
    }

    /// `filterList`: Modified is every row in `_changes` whatever the search; None Default every
    /// row off its default whatever came before; the tree collapsed is every row.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:889-944`
    #[test]
    fn the_filters_are_applied_in_the_csharps_order() {
        use crate::params::{Filters, shown};
        let parameters = [
            parameter("RTL_ALT", 1500.0, Some(1500.0)),
            parameter("RTL_SPEED", 0.0, Some(0.0)),
            parameter("WPNAV_SPEED", 750.0, Some(500.0)),
        ];
        let mut changes = BTreeMap::new();
        changes.insert("RTL_SPEED".to_owned(), 5.0);
        let none = Filters {
            none_default: false,
            modified: false,
            changes: &changes,
            collapsed: false,
        };
        // Nothing chosen: the prompt.
        assert_eq!(names(shown(&parameters, None, "", &none)), None);
        // A group.
        assert_eq!(
            names(shown(&parameters, Some("RTL"), "", &none)),
            Some(vec!["RTL_ALT".to_owned(), "RTL_SPEED".to_owned()])
        );
        // The tree collapsed: the prefix "", every row.
        let collapsed = Filters {
            collapsed: true,
            ..none
        };
        assert_eq!(
            names(shown(&parameters, None, "", &collapsed)).map(|n| n.len()),
            Some(3)
        );
        // Modified: only `_changes`, the search set aside.
        let modified = Filters {
            modified: true,
            ..none
        };
        assert_eq!(
            names(shown(&parameters, None, "WPNAV", &modified)),
            Some(vec!["RTL_SPEED".to_owned()])
        );
        assert_eq!(
            names(shown(
                &parameters,
                None,
                "",
                &Filters {
                    changes: &BTreeMap::new(),
                    ..modified
                }
            )),
            Some(vec![])
        );
        // None Default last, over every row: Modified set aside too.
        let both = Filters {
            none_default: true,
            ..modified
        };
        assert_eq!(
            names(shown(&parameters, Some("RTL"), "", &both)),
            Some(vec!["WPNAV_SPEED".to_owned()])
        );
    }

    /// Refresh Table makes the rows again from the vehicle's table; without it the same table
    /// gives the same rows.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1119-1124`
    #[test]
    fn refresh_table_makes_the_rows_again() {
        let mut view = Telemetry::idle().view();
        view.parameters = std::sync::Arc::from(vec![("RTL_ALT".to_owned(), 1500.0)]);
        let first = crate::params::collect(&view);
        let again = crate::params::collect(&view);
        assert!(std::sync::Arc::ptr_eq(&first, &again));
        let (built, refreshes) = crate::params::tables_built();
        crate::params::refresh_table();
        let fresh = crate::params::collect(&view);
        assert!(!std::sync::Arc::ptr_eq(&first, &fresh));
        assert_eq!(crate::params::tables_built(), (built + 1, refreshes + 1));
    }

    /// Refresh Table shows with Slow Machine ticked: `GetBoolean`'s `bool.TryParse`.
    #[test]
    fn slow_machine_is_read_as_bool_try_parse_reads_it() {
        assert!(get_boolean(Some("True")));
        assert!(get_boolean(Some(" true ")));
        assert!(!get_boolean(Some("False")));
        assert!(!get_boolean(Some("1")));
        assert!(!get_boolean(None));
    }

    /// The tree's collapse: kept as `bool.ToString()` on leaving and read back on returning.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:85-86, 113, 1134-1149`
    #[test]
    fn the_collapse_is_kept_between_visits() {
        let mut raw = RawParams::default();
        raw.activate(None, &[]);
        assert!(!raw.collapsed());
        raw.click_collapse();
        assert_eq!(raw.deactivate(), (COLLAPSED_KEY, "True"));
        let mut again = RawParams::default();
        again.activate(Some("True"), &[]);
        assert!(again.collapsed());
        again.click_collapse();
        assert_eq!(again.deactivate(), (COLLAPSED_KEY, "False"));
    }

    /// `updatedefaultlist`'s URL, with and without `Q_ENABLE`, and the file's.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:864-873, 952-953`
    #[test]
    fn the_presaved_urls_are_the_csharps() {
        assert_eq!(
            list_url(&[]),
            "https://api.github.com/repos/ardupilot/ardupilot/contents/Tools/Frame_params"
        );
        assert_eq!(
            list_url(&[("Q_ENABLE".to_owned(), 1.0)]),
            "https://api.github.com/repos/ardupilot/ardupilot/contents/Tools/Frame_params/QuadPlanes"
        );
        assert_eq!(
            list_url(&[("Q_ENABLE".to_owned(), 0.0)]),
            "https://api.github.com/repos/ardupilot/ardupilot/contents/Tools/Frame_params"
        );
        assert_eq!(
            file_url("Tools/Frame_params/Solo.param"),
            "https://api.github.com/repos/ArduPilot/ardupilot/contents/Tools/Frame_params/Solo.param"
        );
    }

    /// A stand-in for GitHub: each URL's answer.
    struct Canned(Vec<(String, Vec<u8>)>);

    impl Fetch for Canned {
        fn get(&self, url: &str) -> Result<Vec<u8>, String> {
            self.0
                .iter()
                .find(|(held, _)| held == url)
                .map(|(_, body)| body.clone())
                .ok_or_else(|| format!("404 {url}"))
        }
    }

    /// The listing keeps the names holding ".param", in any case, and the file is fetched,
    /// decoded past GitHub's line breaks, saved under its name and read as a `.param` file.
    /// `// C#: ExtLibs/Utilities/GitHubContent.cs:71-110; GCSViews/ConfigurationView/ConfigRawParams.cs:946-957`
    #[test]
    fn a_presaved_file_is_listed_fetched_saved_and_read() {
        let listing = br#"[
            {"name": "3DR_Iris+.param", "path": "Tools/Frame_params/3DR_Iris+.param", "type": "file"},
            {"name": "QuadPlanes", "path": "Tools/Frame_params/QuadPlanes", "type": "dir"},
            {"name": "Solo.PARAM", "path": "Tools/Frame_params/Solo.PARAM", "type": "file"},
            {"name": "README.md", "path": "Tools/Frame_params/README.md", "type": "file"}
        ]"#;
        let text = "RTL_ALT,2000\nWPNAV_SPEED 750\n";
        let encoded = {
            use base64::Engine as _;
            let full = base64::engine::general_purpose::STANDARD.encode(text);
            // GitHub breaks the content every 60 characters.
            let (head, tail) = full.split_at(10);
            format!("{head}\n{tail}\n")
        };
        let file = format!(r#"{{"name": "3DR_Iris+.param", "content": {encoded:?}}}"#);
        let fetch = Canned(vec![
            (list_url(&[]), listing.to_vec()),
            (
                file_url("Tools/Frame_params/3DR_Iris+.param"),
                file.into_bytes(),
            ),
        ]);

        let files = fetch_list(&fetch, &list_url(&[])).expect("the list");
        assert_eq!(
            files.iter().map(|f| f.name.as_str()).collect::<Vec<_>>(),
            ["3DR_Iris+.param", "Solo.PARAM"]
        );

        let directory = mp_os::temp_dir().join(format!(
            "mp-gui-presaved-{}-{:?}",
            mp_os::process_id(),
            wasm_thread::current().id()
        ));
        std::fs::create_dir_all(&directory).expect("scratch directory");
        let first = files.first().expect("a file");
        let values = load_preset(&fetch, first, Some(&directory)).expect("the file");
        assert_eq!(
            std::fs::read_to_string(directory.join("3DR_Iris+.param")).expect("saved"),
            text
        );
        let _ = std::fs::remove_dir_all(&directory);
        let mut values = values;
        values.sort_by(|a, b| a.0.cmp(&b.0));
        assert_eq!(
            values,
            [
                ("RTL_ALT".to_owned(), 2000.0),
                ("WPNAV_SPEED".to_owned(), 750.0)
            ]
        );

        // A file GitHub does not have is the `catch`.
        let missing = PresetFile {
            name: "Nope.param".to_owned(),
            path: "Tools/Frame_params/Nope.param".to_owned(),
        };
        assert!(load_preset(&fetch, &missing, Some(&directory)).is_err());
    }

    /// Bound, the list selects its first file and enables its button; a failed fetch leaves
    /// both disabled; a second `Activate` binds the list it already holds without fetching.
    #[test]
    fn the_presaved_list_binds_as_the_csharps_does() {
        let mut presets = Presets::default();
        assert!(!presets.can_load());
        presets.take_list(Err("offline".to_owned()));
        assert!(!presets.can_load());
        presets.take_list(Ok(vec![
            PresetFile {
                name: "a.param".to_owned(),
                path: "p/a.param".to_owned(),
            },
            PresetFile {
                name: "b.param".to_owned(),
                path: "p/b.param".to_owned(),
            },
        ]));
        assert!(presets.can_load());
        assert_eq!(presets.combo().text(), "a.param");
        assert_eq!(
            presets.selected().map(|f| f.path.as_str()),
            Some("p/a.param")
        );
        presets.combo.select(1);
        assert_eq!(presets.selected().map(|f| f.name.as_str()), Some("b.param"));
        presets.activate(&[]);
        assert!(presets.listing.is_none(), "fetched once a process");
        assert!(presets.can_load());
    }

    /// Every control and fact the script names is one this module or the screen draws or
    /// records.
    #[test]
    fn the_scripts_names_exist() {
        let script = include_str!("../../../tests/gui/params-list-remainder.gui");
        let sources = [
            include_str!("raw_params.rs"),
            include_str!("params.rs"),
            include_str!("main.rs"),
            // ---- ConfigRawParams remainder ----
            include_str!("raw_params_grid.rs"),
            // ---- end ConfigRawParams remainder ----
        ];
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or_default().trim();
            let mut words = line.split_whitespace();
            let (Some(verb), Some(name)) = (words.next(), words.next()) else {
                continue;
            };
            let name = name.split(['@', ':']).next().unwrap_or_default();
            // A row or a group is named by a `format!`: `param-RTL_ALT_M` is `"param-{name}"`.
            // Any stem up to a dash will do: `param-step--` is `"param-step-{label}"`.
            let made: Vec<String> = name
                .match_indices('-')
                .filter_map(|(at, _)| name.get(..=at))
                .map(|stem| format!("\"{stem}{{"))
                .collect();
            match verb {
                "click" | "expect" => assert!(
                    sources.iter().any(|source| {
                        source.contains(&format!("\"{name}\""))
                            || (verb == "click" && made.iter().any(|stem| source.contains(stem)))
                    }),
                    "{verb} {name}: not in the sources"
                ),
                _ => {}
            }
        }
    }
}
