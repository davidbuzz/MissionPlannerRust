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

//! Bringing Mission Planner's files into this application's own directory.
//!
//! This application keeps its files under [`crate::APP_CONFIG_NAME`], not under the C#'s
//! `Mission Planner` directory, by the owner's ruling of 2026-09-25 (PLAN.md section 12, D11): a
//! user running both applications must not lose data to the other one's writes. A pilot moving
//! over should not have to set everything up again either. Two ways bring the C#'s files across,
//! each artefact by name ([`ARTEFACTS`]), with a marker file ([`MARKER`]) recording what came:
//!
//! * The planner's first start offers them - the owner's feature of 2026-10-14, a native row of
//!   its PLUGINS tab (`crates/mp-gui/src/mp_import.rs` says when the offer is made). [`look`] says
//!   what the C#'s folders hold; Import settings is [`import_settings`], which reads the C#'s
//!   `config.xml` for the planner to lay over its own and copies every other artefact; Copy logs is
//!   [`copy_logs`], the C#'s log files copied into this application's log folder, the folders kept.
//!   The planner imports nothing unasked: its silent import is gone (the owner, 2026-10-14).
//! * [`import`], the one-shot import: on the first start that finds this application's user data
//!   directory missing or empty while the C#'s exists, every artefact copied across once, and the
//!   marker making the directory non-empty, so it never runs a second time - not even when nothing
//!   was found to copy. Only `headless-planner` still runs it.
//!
//! The C#'s directory is only ever read: nothing in it is written, renamed or deleted, and a file
//! this application already has is never overwritten.
//!
//! What is not imported, on purpose: the map tile cache (`gmapcache`), the terrain tiles (`srtm`)
//! and the parameter metadata (`*.pdef.xml` and the `LogMessages*.xml` files). They are large -
//! gigabytes of tiles on a machine that has flown - and every one of them is re-created on demand:
//! tiles and terrain are downloaded again, the metadata is fetched again. Nor are the flight logs
//! and recordings (`logs`) part of [`import`]: they stay where the C# wrote them, readable from
//! there, and the planner's offer copies them only when Copy logs is chosen.
//!
//! Each artefact is copied from the C# directory the C# keeps it in to the same directory under
//! this application's name: the user data directory for most, the shared data directory for
//! `UserAlerts.json` and `History` (the two differ on Windows only). `logo.png` and `logo.txt` the
//! C# reads beside its executable (`GetRunningDirectory`, `Program.cs:211-223`), which is not a
//! data directory; they are copied from the user data directory if a user put them there.

use mp_os::fs::FsExt as _;
use std::fmt::Write as _;
use std::io;
use std::path::{Path, PathBuf};

use crate::{Config, Folders};

/// Which of an application's two data directories an artefact is kept in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kept {
    /// `Settings.GetUserDataDirectory`.
    User,
    /// `Settings.GetDataDirectory`.
    Shared,
}

/// What is imported, by name, and where the C# keeps each.
pub const ARTEFACTS: &[(&str, Kept)] = &[
    // `// C#: ExtLibs/Utilities/Settings.cs:370-411`
    ("config.xml", Kept::User),
    // `// C#: Utilities/POI.cs:40`
    ("poi.txt", Kept::User),
    // `// C#: Grid/GridUI.cs:124`
    ("cameras.xml", Kept::User),
    // `// C#: Controls/PreFlight/CheckListControl.cs:14`
    ("checklist.xml", Kept::User),
    // `// C#: ExtLibs/Utilities/Warnings/WarningEngine.cs:14`
    ("warnings.xml", Kept::User),
    // `// C#: ExtLibs/ArduPilot/UserAlert.cs:25`
    ("UserAlerts.json", Kept::Shared),
    // `// C#: ExtLibs/ArduPilot/Mavlink/MAVAuthKeys.cs:18`
    ("authkeys.xml", Kept::User),
    // `// C#: Program.cs:222` (beside the executable in the C#)
    ("logo.png", Kept::User),
    // `// C#: Program.cs:211-213` (beside the executable in the C#)
    ("logo.txt", Kept::User),
    // A folder. `// C#: temp.cs:250`
    ("History", Kept::Shared),
];

/// The file in this application's user data directory that records the import.
pub const MARKER: &str = "imported-from-Mission-Planner.txt";

/// The four directories an import reads from and writes to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Directories {
    /// The C#'s user data directory.
    pub from_user: PathBuf,
    /// The C#'s shared data directory.
    pub from_shared: PathBuf,
    /// This application's user data directory, which must be missing or empty.
    pub to_user: PathBuf,
    /// This application's shared data directory.
    pub to_shared: PathBuf,
}

impl Directories {
    /// Both applications' directories, by the C#'s rules under each one's name.
    #[must_use]
    pub fn of(folders: &Folders) -> Self {
        Self {
            from_user: folders.csharp_user_data_directory(),
            from_shared: folders.csharp_data_directory(),
            to_user: folders.user_data_directory(),
            to_shared: folders.data_directory(),
        }
    }

    const fn from(&self, kept: Kept) -> &PathBuf {
        match kept {
            Kept::User => &self.from_user,
            Kept::Shared => &self.from_shared,
        }
    }

    const fn to(&self, kept: Kept) -> &PathBuf {
        match kept {
            Kept::User => &self.to_user,
            Kept::Shared => &self.to_shared,
        }
    }
}

/// What a start's import did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Import {
    /// The import ran: these artefacts were copied (possibly none), and these could not be.
    Imported {
        /// The C#'s user data directory they came from.
        from: PathBuf,
        /// The artefacts copied, in [`ARTEFACTS`] order.
        names: Vec<String>,
        /// The artefacts that were there and could not be copied, each with why.
        failed: Vec<String>,
    },
    /// This application's directory already holds something - an earlier import's marker, or
    /// files of its own - so nothing was done.
    AlreadyHere,
    /// There is no C# directory to import from, so nothing was done.
    NothingToImport,
    /// No home directory to find either directory under.
    NoHome,
    /// This application's directory could not be made, or the marker not written.
    Failed(String),
}

impl Import {
    /// The artefacts copied by this start, if any.
    #[must_use]
    pub fn names(&self) -> &[String] {
        match self {
            Self::Imported { names, .. } => names,
            _ => &[],
        }
    }

    /// The artefacts copied by this start, comma-separated, or `none`.
    #[must_use]
    pub fn summary(&self) -> String {
        if self.names().is_empty() {
            "none".to_owned()
        } else {
            self.names().join(", ")
        }
    }
}

/// Imports the C#'s artefacts once, if this application's user data directory is missing or
/// empty and the C#'s exists.
#[must_use]
pub fn import(directories: &Directories) -> Import {
    if !is_missing_or_empty(&directories.to_user) {
        return Import::AlreadyHere;
    }
    if !directories.from_user.os_is_dir() && !directories.from_shared.os_is_dir() {
        return Import::NothingToImport;
    }
    if let Err(err) = mp_os::fs::create_dir_all(&directories.to_user) {
        return Import::Failed(format!("{}: {err}", directories.to_user.display()));
    }
    let mut names = Vec::new();
    let mut failed = Vec::new();
    for &(name, kept) in ARTEFACTS {
        let source = directories.from(kept).join(name);
        let target = directories.to(kept).join(name);
        // A file this application already has (a shared directory on Windows can) is its own.
        if mp_os::fs::symlink_metadata(&source).is_err() || mp_os::fs::symlink_metadata(&target).is_ok()
        {
            continue;
        }
        match copy(&source, &target) {
            Ok(()) => names.push(name.to_owned()),
            Err(err) => failed.push(format!("{name}: {err}")),
        }
    }
    let marker = marker_text(directories, &names, &failed);
    if let Err(err) = mp_os::fs::write(directories.to_user.join(MARKER), marker) {
        return Import::Failed(format!("{MARKER}: {err}"));
    }
    Import::Imported {
        from: directories.from_user.clone(),
        names,
        failed,
    }
}

/// Whether a directory is absent or has nothing in it. One that cannot be read is neither: it is
/// left alone.
fn is_missing_or_empty(directory: &Path) -> bool {
    match mp_os::fs::read_dir(directory) {
        Ok(mut entries) => entries.next().is_none(),
        Err(err) => err.kind() == io::ErrorKind::NotFound,
    }
}

/// Copies a file, or a folder and everything in it. Reads the source only.
fn copy(source: &Path, target: &Path) -> io::Result<()> {
    if mp_os::fs::metadata(source)?.is_dir() {
        copy_tree(source, target)
    } else {
        if let Some(parent) = target.parent() {
            mp_os::fs::create_dir_all(parent)?;
        }
        mp_os::fs::copy(source, target).map(|_| ())
    }
}

/// Copies a folder's contents. A link inside it is copied as what it points at when that is a
/// file, and is an error when it is a folder, so a loop of links cannot recurse for ever.
fn copy_tree(source: &Path, target: &Path) -> io::Result<()> {
    mp_os::fs::create_dir_all(target)?;
    for entry in mp_os::fs::read_dir(source)? {
        let entry = entry?;
        let to = target.join(entry.file_name());
        if entry.file_type()?.is_dir() {
            copy_tree(&entry.path(), &to)?;
        } else {
            mp_os::fs::copy(entry.path(), &to)?;
        }
    }
    Ok(())
}

/// The marker's text: where from, then one artefact copied a line.
fn marker_text(directories: &Directories, names: &[String], failed: &[String]) -> String {
    let mut text =
        String::from("Imported once from Mission Planner's directory, which was left as it was.\n");
    let _ = writeln!(text, "from {}", directories.from_user.display());
    if directories.from_shared != directories.from_user {
        let _ = writeln!(text, "from {}", directories.from_shared.display());
    }
    for name in names {
        let _ = writeln!(text, "{name}");
    }
    for failure in failed {
        let _ = writeln!(text, "not copied: {failure}");
    }
    text
}

// --- The planner's first-start offer -------------------------------------------------------------

/// The extensions of the files [`copy_logs`] copies, compared without regard to case: telemetry
/// logs and the raw recordings beside them (`.tlog`, `.rlog`), and dataflash logs, binary and text
/// (`.bin`, `.log`).
pub const LOG_EXTENSIONS: [&str; 4] = ["tlog", "rlog", "bin", "log"];

/// What Mission Planner's folders hold, for the offer to say.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Found {
    /// The C#'s user data directory.
    pub from: PathBuf,
    /// Its `config.xml`, when it has one.
    pub config: Option<PathBuf>,
    /// The artefacts of [`ARTEFACTS`] that are there, `config.xml` among them, in that order.
    pub settings: Vec<String>,
    /// The C#'s log folder, `Settings.LogDir`: its `config.xml`'s `logdirectory` when that is set,
    /// else `logs` in its user data directory.
    pub logs: PathBuf,
    /// Whether a log file ([`is_log`]) is anywhere under that folder.
    pub has_logs: bool,
}

impl Found {
    /// Whether there is anything to offer: a `config.xml`, or logs.
    #[must_use]
    pub const fn anything(&self) -> bool {
        self.config.is_some() || self.has_logs
    }
}

/// What Mission Planner's folders hold, or `None` when its user data directory is not there. Only
/// looks: the log folder is walked until its first log.
#[must_use]
pub fn look(directories: &Directories) -> Option<Found> {
    let from = &directories.from_user;
    if !from.os_is_dir() {
        return None;
    }
    let config = Some(from.join(crate::config::FILE_NAME)).filter(|path| path.os_is_file());
    let settings = ARTEFACTS
        .iter()
        .filter(|&&(name, kept)| {
            mp_os::fs::symlink_metadata(directories.from(kept).join(name)).is_ok()
        })
        .map(|&(name, _)| name.to_owned())
        .collect();
    let logs = csharp_log_directory(from, config.as_deref());
    let has_logs = holds_a_log(&logs);
    Some(Found {
        from: from.clone(),
        config,
        settings,
        logs,
        has_logs,
    })
}

/// `Settings.LogDir` for the C#: its `config.xml`'s `logdirectory` when that is set, else `logs`
/// in its user data directory (`GetDefaultLogDir`). A file that cannot be read names none.
/// `// C#: ExtLibs/Utilities/Settings.cs:127-158`
fn csharp_log_directory(from_user: &Path, config: Option<&Path>) -> PathBuf {
    config
        .and_then(|path| read_config(path).ok())
        .and_then(|config| config.log_directory())
        .unwrap_or_else(|| from_user.join("logs"))
}

/// A `config.xml` read as `Settings.Load` reads one: a file with nothing in it is no settings,
/// and any other failure is the reason.
fn read_config(path: &Path) -> Result<Config, String> {
    let text = mp_os::fs::read_to_string(path).map_err(|err| err.to_string())?;
    if text.trim_start_matches('\u{feff}').trim().is_empty() {
        return Ok(Config::default());
    }
    Config::parse(&text).map_err(|err| err.to_string())
}

/// Whether a file is a log, by its extension ([`LOG_EXTENSIONS`]).
#[must_use]
pub fn is_log(path: &Path) -> bool {
    path.extension()
        .and_then(|extension| extension.to_str())
        .is_some_and(|extension| {
            LOG_EXTENSIONS
                .iter()
                .any(|log| extension.eq_ignore_ascii_case(log))
        })
}

/// Each log file under `dir` handed to `each` by its path relative to `root`, the entries of a
/// folder in name order; `each` answering false ends the walk, and the walk then answers false
/// too. A folder that cannot be read is passed over; a link to a folder is not followed, so a loop
/// of links cannot recurse for ever; and `skip`, a folder the walk must not enter, is not entered.
fn walk_logs(
    root: &Path,
    dir: &Path,
    skip: Option<&Path>,
    each: &mut dyn FnMut(PathBuf) -> bool,
) -> bool {
    let Ok(entries) = mp_os::fs::read_dir(dir) else {
        return true;
    };
    let mut entries: Vec<mp_os::fs::DirEntry> = entries.flatten().collect();
    entries.sort_by_cached_key(mp_os::fs::DirEntry::file_name);
    for entry in entries {
        let path = entry.path();
        let Ok(kind) = entry.file_type() else {
            continue;
        };
        if kind.is_dir() {
            if skip != Some(path.as_path()) && !walk_logs(root, &path, skip, each) {
                return false;
            }
        } else if is_log(&path) {
            let relative = path
                .strip_prefix(root)
                .map_or_else(|_| path.clone(), Path::to_path_buf);
            if !each(relative) {
                return false;
            }
        }
    }
    true
}

/// The log files under `root`, by their paths relative to it, folders in name order; `skip` is a
/// folder not walked.
#[must_use]
pub fn log_files(root: &Path, skip: Option<&Path>) -> Vec<PathBuf> {
    let mut files = Vec::new();
    walk_logs(root, root, skip, &mut |relative| {
        files.push(relative);
        true
    });
    files
}

/// Whether a log file is anywhere under `root`: the walk ends at the first.
fn holds_a_log(root: &Path) -> bool {
    !walk_logs(root, root, None, &mut |_| false)
}

/// What [`copy_logs`] did.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LogCopy {
    /// How many log files were found.
    pub found: usize,
    /// Those copied, by their paths relative to the C#'s log folder, in the walk's order.
    pub copied: Vec<PathBuf>,
    /// How many this application already had: left as they were.
    pub already: usize,
    /// Those that could not be copied, each with why.
    pub failed: Vec<String>,
}

/// Copies every log file under `from` ([`is_log`]) to the same place under `to`, the folders
/// kept. A file already there is this application's and is left as it is - never overwritten -
/// and `from` is only read. Each file lands under a name of its own first and is then renamed into
/// place, so a copy cut short leaves no half a log under a log's name for the next copy to take as
/// done. `progress` hears how many of how many are done: once the files are found, and after each.
/// A `to` inside `from` is not walked, so this application's logs are not copied into themselves.
#[must_use]
pub fn copy_logs(from: &Path, to: &Path, mut progress: impl FnMut(usize, usize)) -> LogCopy {
    let files = log_files(from, Some(to));
    let mut outcome = LogCopy {
        found: files.len(),
        ..LogCopy::default()
    };
    progress(0, files.len());
    for (done, relative) in files.iter().enumerate() {
        let target = to.join(relative);
        if mp_os::fs::symlink_metadata(&target).is_ok() {
            outcome.already += 1;
        } else {
            match copy_log(&from.join(relative), &target) {
                Ok(()) => outcome.copied.push(relative.clone()),
                Err(err) => outcome
                    .failed
                    .push(format!("{}: {err}", relative.display())),
            }
        }
        progress(done + 1, files.len());
    }
    outcome
}

/// One log copied: to `<name>.part` beside where it goes, then renamed into place; what a failure
/// left of the partial file removed.
fn copy_log(source: &Path, target: &Path) -> io::Result<()> {
    if let Some(parent) = target.parent() {
        mp_os::fs::create_dir_all(parent)?;
    }
    let mut partial = target.as_os_str().to_owned();
    partial.push(".part");
    let partial = PathBuf::from(partial);
    let copied =
        mp_os::fs::copy(source, &partial).and_then(|_| mp_os::fs::rename(&partial, target));
    if copied.is_err() {
        let _ = mp_os::fs::remove_file(&partial);
    }
    copied
}

/// What the offer's Import settings did with Mission Planner's files.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SettingsImport {
    /// Mission Planner's `config.xml`, read: for the planner to lay over its own dictionary and
    /// save, since this application's `config.xml` is its own to write - and by the time the
    /// offer is answered, the start-up has written one.
    pub config: Option<Config>,
    /// What came across, in [`ARTEFACTS`] order: `config.xml` when it was read, and each other
    /// artefact copied.
    pub names: Vec<String>,
    /// What was there and could not be read or copied, each with why.
    pub failed: Vec<String>,
}

impl SettingsImport {
    /// What came across, comma-separated, or `none`.
    #[must_use]
    pub fn summary(&self) -> String {
        if self.names.is_empty() {
            "none".to_owned()
        } else {
            self.names.join(", ")
        }
    }
}

/// The offer's Import settings, the files: Mission Planner's `config.xml` read, and each other
/// artefact ([`ARTEFACTS`]) copied from the C# directory that keeps it to the same directory under
/// this application's name, by [`import`]'s rules - a file this application already has is its own
/// and is not overwritten, and the C#'s directory is only read. The marker records what came
/// across, after whatever an earlier import wrote in it.
#[must_use]
pub fn import_settings(directories: &Directories) -> SettingsImport {
    let mut outcome = SettingsImport::default();
    for &(name, kept) in ARTEFACTS {
        let source = directories.from(kept).join(name);
        if mp_os::fs::symlink_metadata(&source).is_err() {
            continue;
        }
        if name == crate::config::FILE_NAME {
            match read_config(&source) {
                Ok(config) => {
                    outcome.config = Some(config);
                    outcome.names.push(name.to_owned());
                }
                Err(why) => outcome.failed.push(format!("{name}: {why}")),
            }
            continue;
        }
        let target = directories.to(kept).join(name);
        if mp_os::fs::symlink_metadata(&target).is_ok() {
            continue;
        }
        match copy(&source, &target) {
            Ok(()) => outcome.names.push(name.to_owned()),
            Err(err) => outcome.failed.push(format!("{name}: {err}")),
        }
    }
    if (!outcome.names.is_empty() || !outcome.failed.is_empty())
        && let Err(err) = add_to_marker(directories, &outcome.names, &outcome.failed)
    {
        outcome.failed.push(format!("{MARKER}: {err}"));
    }
    outcome
}

/// The marker with this import's lines added after what it held, if anything.
fn add_to_marker(directories: &Directories, names: &[String], failed: &[String]) -> io::Result<()> {
    mp_os::fs::create_dir_all(&directories.to_user)?;
    let path = directories.to_user.join(MARKER);
    let mut text = mp_os::fs::read_to_string(&path).unwrap_or_default();
    text.push_str(&marker_text(directories, names, failed));
    mp_os::fs::write(&path, text)
}

#[cfg(test)]
mod tests {
    use mp_os::fs::FsExt as _;
    use super::*;
    use std::collections::BTreeMap;

    /// A temporary directory that removes itself.
    struct Scratch(PathBuf);

    impl Scratch {
        fn new(name: &str) -> Self {
            let path = mp_os::temp_dir().join(format!(
                "mp-settings-migrate-{name}-{}-{:?}",
                mp_os::process_id(),
                wasm_thread::current().id()
            ));
            let _ = mp_os::fs::remove_dir_all(&path);
            mp_os::fs::create_dir_all(&path).expect("scratch");
            Self(path)
        }

        /// A mono machine whose home is in the scratch directory.
        fn mono(&self) -> Folders {
            Folders {
                my_documents: self.0.join("home"),
                local_application_data: self.0.join("home").join(".local").join("share"),
                common_application_data: PathBuf::from("/usr/share"),
                unix: true,
            }
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = mp_os::fs::remove_dir_all(&self.0);
        }
    }

    /// Every file under a directory, by path relative to it, with its bytes; folders as `None`.
    fn snapshot(root: &Path) -> BTreeMap<PathBuf, Option<Vec<u8>>> {
        fn walk(root: &Path, dir: &Path, into: &mut BTreeMap<PathBuf, Option<Vec<u8>>>) {
            for entry in mp_os::fs::read_dir(dir).expect("readable") {
                let path = entry.expect("entry").path();
                let relative = path.strip_prefix(root).expect("under root").to_path_buf();
                if path.os_is_dir() {
                    into.insert(relative, None);
                    walk(root, &path, into);
                } else {
                    into.insert(relative, Some(mp_os::fs::read(&path).expect("readable")));
                }
            }
        }
        let mut files = BTreeMap::new();
        walk(root, root, &mut files);
        files
    }

    /// Bytes that differ per artefact, so a copy of the wrong file is caught.
    fn content(name: &str) -> Vec<u8> {
        let mut bytes = format!("\u{feff}the C#'s {name}\r\n").into_bytes();
        bytes.extend([0, 1, 2, 255]);
        bytes
    }

    /// Puts one artefact in the C#'s directory (a folder with two files, nested, for `History`),
    /// alongside things that must not be imported, imports, and checks the copy and that the
    /// C#'s directory is byte for byte what it was.
    fn imports_one(name: &str) {
        let scratch = Scratch::new(&name.replace('.', "-"));
        let folders = scratch.mono();
        let directories = Directories::of(&folders);
        let theirs = &directories.from_user;
        mp_os::fs::create_dir_all(theirs.join("gmapcache")).expect("C# directory");
        mp_os::fs::write(theirs.join("gmapcache").join("tile.png"), b"tile").expect("tile");
        mp_os::fs::write(theirs.join("ArduCopter.apm.pdef.xml"), b"<params/>").expect("pdef");
        if name == "History" {
            let history = theirs.join("History");
            mp_os::fs::create_dir_all(history.join("ArduCopter")).expect("History");
            mp_os::fs::write(history.join("firmware.hex"), content("firmware.hex")).expect("hex");
            mp_os::fs::write(
                history.join("ArduCopter").join("arducopter.apj"),
                content("apj"),
            )
            .expect("apj");
        } else {
            mp_os::fs::write(theirs.join(name), content(name)).expect("artefact");
        }
        let before = snapshot(theirs);

        let outcome = import(&directories);

        assert_eq!(outcome.names(), [name.to_owned()], "{outcome:?}");
        assert_eq!(outcome.summary(), name);
        assert_eq!(snapshot(theirs), before, "the C#'s directory was changed");
        let ours = snapshot(&directories.to_user);
        for (path, bytes) in &before {
            if path.starts_with(name) {
                assert_eq!(ours.get(path), Some(bytes), "{} not copied", path.display());
            }
        }
        assert!(!directories.to_user.join("gmapcache").os_exists());
        assert!(!directories.to_user.join("ArduCopter.apm.pdef.xml").os_exists());
        let marker = mp_os::fs::read_to_string(directories.to_user.join(MARKER)).expect("the marker");
        assert!(marker.lines().any(|line| line == name), "{marker}");
    }

    #[test]
    fn imports_config_xml() {
        imports_one("config.xml");
    }

    #[test]
    fn imports_poi_txt() {
        imports_one("poi.txt");
    }

    #[test]
    fn imports_cameras_xml() {
        imports_one("cameras.xml");
    }

    #[test]
    fn imports_checklist_xml() {
        imports_one("checklist.xml");
    }

    #[test]
    fn imports_warnings_xml() {
        imports_one("warnings.xml");
    }

    #[test]
    fn imports_user_alerts_json() {
        imports_one("UserAlerts.json");
    }

    #[test]
    fn imports_authkeys_xml() {
        imports_one("authkeys.xml");
    }

    #[test]
    fn imports_logo_png() {
        imports_one("logo.png");
    }

    #[test]
    fn imports_logo_txt() {
        imports_one("logo.txt");
    }

    #[test]
    fn imports_the_history_folder() {
        imports_one("History");
    }

    #[test]
    fn every_artefact_at_once_in_order() {
        let scratch = Scratch::new("all");
        let directories = Directories::of(&scratch.mono());
        mp_os::fs::create_dir_all(&directories.from_user).expect("C# directory");
        for &(name, _) in ARTEFACTS {
            if name == "History" {
                mp_os::fs::create_dir_all(directories.from_user.join(name)).expect("History");
            } else {
                mp_os::fs::write(directories.from_user.join(name), content(name)).expect("file");
            }
        }
        let outcome = import(&directories);
        let expected: Vec<String> = ARTEFACTS
            .iter()
            .map(|(name, _)| (*name).to_owned())
            .collect();
        assert_eq!(outcome.names(), expected);
        assert_eq!(outcome.summary(), expected.join(", "));
    }

    #[test]
    fn a_second_start_imports_nothing() {
        let scratch = Scratch::new("second");
        let directories = Directories::of(&scratch.mono());
        mp_os::fs::create_dir_all(&directories.from_user).expect("C# directory");
        mp_os::fs::write(directories.from_user.join("config.xml"), b"first").expect("config");
        assert_eq!(import(&directories).names(), ["config.xml".to_owned()]);

        // The C# goes on writing its own file, and a new one; this application keeps what it has.
        mp_os::fs::write(directories.from_user.join("config.xml"), b"second").expect("config");
        mp_os::fs::write(directories.from_user.join("poi.txt"), b"poi").expect("poi");
        let again = import(&directories);
        assert_eq!(again, Import::AlreadyHere);
        assert_eq!(again.summary(), "none");
        assert_eq!(
            mp_os::fs::read(directories.to_user.join("config.xml")).expect("ours"),
            b"first"
        );
        assert!(!directories.to_user.join("poi.txt").os_exists());
    }

    #[test]
    fn a_start_that_found_nothing_to_copy_still_never_runs_again() {
        let scratch = Scratch::new("found-nothing");
        let directories = Directories::of(&scratch.mono());
        mp_os::fs::create_dir_all(&directories.from_user).expect("an empty C# directory");
        let first = import(&directories);
        assert!(matches!(first, Import::Imported { .. }), "{first:?}");
        assert_eq!(first.summary(), "none");
        assert!(directories.to_user.join(MARKER).os_is_file());
        mp_os::fs::write(directories.from_user.join("config.xml"), b"later").expect("config");
        assert_eq!(import(&directories), Import::AlreadyHere);
    }

    #[test]
    fn an_existing_directory_with_anything_in_it_is_left_alone() {
        let scratch = Scratch::new("not-empty");
        let directories = Directories::of(&scratch.mono());
        mp_os::fs::create_dir_all(&directories.from_user).expect("C# directory");
        mp_os::fs::write(directories.from_user.join("config.xml"), b"theirs").expect("config");
        mp_os::fs::create_dir_all(&directories.to_user).expect("ours");
        mp_os::fs::write(directories.to_user.join("poi.txt"), b"ours").expect("poi");
        let before = snapshot(&directories.to_user);

        assert_eq!(import(&directories), Import::AlreadyHere);
        assert_eq!(snapshot(&directories.to_user), before);
    }

    #[test]
    fn an_existing_empty_directory_is_imported_into() {
        let scratch = Scratch::new("empty");
        let directories = Directories::of(&scratch.mono());
        mp_os::fs::create_dir_all(&directories.from_user).expect("C# directory");
        mp_os::fs::write(directories.from_user.join("config.xml"), b"theirs").expect("config");
        mp_os::fs::create_dir_all(&directories.to_user).expect("ours, empty");
        assert_eq!(import(&directories).names(), ["config.xml".to_owned()]);
    }

    #[test]
    fn no_csharp_directory_is_fine_and_makes_nothing() {
        let scratch = Scratch::new("no-csharp");
        let directories = Directories::of(&scratch.mono());
        let outcome = import(&directories);
        assert_eq!(outcome, Import::NothingToImport);
        assert_eq!(outcome.summary(), "none");
        assert!(!directories.to_user.os_exists());
        assert!(!directories.from_user.os_exists());
    }

    #[test]
    fn on_windows_shared_artefacts_go_from_program_data_to_program_data() {
        let scratch = Scratch::new("windows");
        let folders = Folders {
            my_documents: scratch.0.join("Documents"),
            local_application_data: scratch.0.join("AppData").join("Local"),
            common_application_data: scratch.0.join("ProgramData"),
            unix: false,
        };
        let directories = Directories::of(&folders);
        assert_eq!(
            directories.from_shared,
            scratch.0.join("ProgramData").join("Mission Planner")
        );
        assert_eq!(
            directories.to_shared,
            scratch.0.join("ProgramData").join("MissionPlannerRust")
        );
        mp_os::fs::create_dir_all(&directories.from_user).expect("C# user directory");
        mp_os::fs::create_dir_all(&directories.from_shared).expect("C# shared directory");
        mp_os::fs::write(directories.from_user.join("config.xml"), b"config").expect("config");
        mp_os::fs::write(directories.from_shared.join("UserAlerts.json"), b"{}").expect("alerts");
        // A shared file this application already has is not overwritten.
        mp_os::fs::create_dir_all(directories.from_shared.join("History")).expect("History");
        mp_os::fs::create_dir_all(directories.to_shared.join("History")).expect("our History");

        let outcome = import(&directories);
        assert_eq!(
            outcome.names(),
            ["config.xml".to_owned(), "UserAlerts.json".to_owned()]
        );
        assert_eq!(
            mp_os::fs::read(directories.to_shared.join("UserAlerts.json")).expect("copied"),
            b"{}"
        );
        assert!(!directories.to_user.join("UserAlerts.json").os_exists());
    }

    // --- The planner's first-start offer ---------------------------------------------------------

    /// Writes a file, making its folder.
    fn put(path: &Path, bytes: &[u8]) {
        mp_os::fs::create_dir_all(path.parent().expect("a folder")).expect("folder");
        mp_os::fs::write(path, bytes).expect("written");
    }

    /// What the offer is told: nothing without the C#'s directory; with it, its `config.xml`, the
    /// other artefacts there, and its log folder - `logdirectory` when the file names one - with
    /// whether a log is under it, a `.txt` not being one.
    #[test]
    fn the_offer_looks_for_a_config_and_logs() {
        let scratch = Scratch::new("look");
        let directories = Directories::of(&scratch.mono());
        assert_eq!(look(&directories), None);

        let theirs = &directories.from_user;
        put(&theirs.join("logs").join("notes.txt"), b"not a log");
        let found = look(&directories).expect("the C#'s directory");
        assert_eq!(found.from, *theirs);
        assert_eq!(found.config, None);
        assert_eq!(found.logs, theirs.join("logs"));
        assert!(!found.has_logs);
        assert!(!found.anything());
        assert!(found.settings.is_empty());

        // A log, deep down: found.
        put(&theirs.join("logs").join("QUADROTOR").join("1").join("a.tlog"), b"t");
        assert!(look(&directories).expect("there").has_logs);

        // A config.xml naming a log folder of its own: that folder, and only its logs count.
        let elsewhere = scratch.0.join("drive").join("flights");
        put(&elsewhere.join("HELI").join("00000001.BIN"), b"df");
        put(
            &theirs.join("config.xml"),
            format!(
                "\u{feff}<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Config>\n  <logdirectory>{}</logdirectory>\n</Config>",
                elsewhere.display()
            )
            .as_bytes(),
        );
        put(&theirs.join("poi.txt"), b"poi");
        let found = look(&directories).expect("there");
        assert_eq!(found.config, Some(theirs.join("config.xml")));
        assert_eq!(found.logs, elsewhere);
        assert!(found.has_logs);
        assert!(found.anything());
        assert_eq!(found.settings, ["config.xml".to_owned(), "poi.txt".to_owned()]);
    }

    #[test]
    fn a_log_is_known_by_its_extension_whatever_its_case() {
        for name in ["a.tlog", "a.TLOG", "a.rlog", "00000001.BIN", "1.bin", "old.log"] {
            assert!(is_log(Path::new(name)), "{name}");
        }
        for name in ["a.txt", "a.param", "tlog", "a.tlog.part", "a.bin.gz"] {
            assert!(!is_log(Path::new(name)), "{name}");
        }
    }

    /// Copy logs: every log under the C#'s folder lands at the same place under this
    /// application's, the folders kept; a log already there is left as it is; a file that is not
    /// a log stays where it is; the C#'s folder is byte for byte what it was; no partial file is
    /// left behind; the progress counts to the end; and a second copy copies nothing.
    #[test]
    fn copy_logs_keeps_the_folders_and_never_overwrites() {
        let scratch = Scratch::new("copy-logs");
        let from = scratch.0.join("Mission Planner").join("logs");
        let to = scratch.0.join("MissionPlannerRust").join("logs");
        put(&from.join("QUADROTOR/1/a.tlog"), &content("a.tlog"));
        put(&from.join("QUADROTOR/1/a.rlog"), &content("a.rlog"));
        put(&from.join("QUADROTOR/1/notes.txt"), b"notes");
        put(&from.join("HELI/00000001.BIN"), &content("bin"));
        put(&from.join("old.log"), &content("old.log"));
        put(&to.join("QUADROTOR/1/a.tlog"), b"ours");
        let before = snapshot(&from);

        let mut heard = Vec::new();
        let outcome = copy_logs(&from, &to, |done, of| heard.push((done, of)));

        assert_eq!(outcome.found, 4);
        assert_eq!(
            outcome.copied,
            [
                PathBuf::from("HELI/00000001.BIN"),
                PathBuf::from("QUADROTOR/1/a.rlog"),
                PathBuf::from("old.log"),
            ]
        );
        assert_eq!(outcome.already, 1);
        assert!(outcome.failed.is_empty(), "{:?}", outcome.failed);
        assert_eq!(heard.first(), Some(&(0, 4)));
        assert_eq!(heard.last(), Some(&(4, 4)));
        assert_eq!(heard.len(), 5);
        for copied in &outcome.copied {
            assert_eq!(
                mp_os::fs::read(to.join(copied)).expect("copied"),
                mp_os::fs::read(from.join(copied)).expect("theirs"),
                "{}",
                copied.display()
            );
        }
        assert_eq!(
            mp_os::fs::read(to.join("QUADROTOR/1/a.tlog")).expect("ours"),
            b"ours"
        );
        assert!(!to.join("QUADROTOR/1/notes.txt").os_exists());
        assert!(
            snapshot(&to)
                .keys()
                .all(|path| path.extension().is_none_or(|extension| extension != "part")),
            "a partial file was left"
        );
        assert_eq!(snapshot(&from), before, "the C#'s logs were changed");

        let again = copy_logs(&from, &to, |_, _| {});
        assert!(again.copied.is_empty());
        assert_eq!(again.already, 4);
    }

    /// This application's log folder inside the C#'s is not walked, so its own logs are not
    /// copied into themselves; one folder for both copies nothing.
    #[test]
    fn a_log_folder_inside_the_csharps_is_not_copied_into_itself() {
        let scratch = Scratch::new("copy-inside");
        let from = scratch.0.join("logs");
        let to = from.join("MissionPlannerRust");
        put(&from.join("theirs.tlog"), b"theirs");
        put(&to.join("ours.tlog"), b"ours");
        let outcome = copy_logs(&from, &to, |_, _| {});
        assert_eq!(outcome.found, 1);
        assert_eq!(outcome.copied, [PathBuf::from("theirs.tlog")]);
        assert!(!to.join("MissionPlannerRust").os_exists());

        let same = copy_logs(&from, &from, |_, _| {});
        assert!(same.copied.is_empty());
        assert_eq!(same.already, same.found);
    }

    /// Import settings: the C#'s `config.xml` is read, not copied - this application's is its own
    /// to write, and the start-up has written one - and every other artefact comes across unless
    /// this application has it already; the marker keeps what an earlier import wrote and adds
    /// this one's lines; the C#'s directory is byte for byte what it was.
    #[test]
    fn import_settings_reads_the_config_and_copies_the_rest() {
        let scratch = Scratch::new("import-settings");
        let directories = Directories::of(&scratch.mono());
        let theirs = &directories.from_user;
        put(
            &theirs.join("config.xml"),
            b"\xef\xbb\xbf<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Config>\n  <distunits>Feet</distunits>\n</Config>",
        );
        put(&theirs.join("poi.txt"), &content("poi.txt"));
        put(&theirs.join("cameras.xml"), &content("cameras.xml"));
        put(&theirs.join("History").join("firmware.hex"), &content("hex"));
        let ours = &directories.to_user;
        put(&ours.join("config.xml"), b"the start-up's");
        put(&ours.join("cameras.xml"), b"ours");
        put(&ours.join(MARKER), b"earlier\n");
        let before = snapshot(theirs);

        let outcome = import_settings(&directories);

        assert_eq!(
            outcome.names,
            [
                "config.xml".to_owned(),
                "poi.txt".to_owned(),
                "History".to_owned()
            ]
        );
        assert_eq!(outcome.summary(), "config.xml, poi.txt, History");
        assert!(outcome.failed.is_empty(), "{:?}", outcome.failed);
        let config = outcome.config.expect("their config.xml, read");
        assert_eq!(config.get("distunits"), Some("Feet"));
        assert_eq!(
            mp_os::fs::read(ours.join("config.xml")).expect("ours"),
            b"the start-up's"
        );
        assert_eq!(mp_os::fs::read(ours.join("cameras.xml")).expect("ours"), b"ours");
        assert_eq!(
            mp_os::fs::read(ours.join("poi.txt")).expect("copied"),
            content("poi.txt")
        );
        assert_eq!(
            mp_os::fs::read(ours.join("History").join("firmware.hex")).expect("copied"),
            content("hex")
        );
        let marker = mp_os::fs::read_to_string(ours.join(MARKER)).expect("the marker");
        assert!(marker.starts_with("earlier\n"), "{marker}");
        assert!(marker.lines().any(|line| line == "poi.txt"), "{marker}");
        assert!(marker.lines().any(|line| line == "config.xml"), "{marker}");
        assert_eq!(snapshot(theirs), before, "the C#'s directory was changed");
    }

    /// A `config.xml` of the C#'s that does not read is said, and the other artefacts still come.
    #[test]
    fn an_unreadable_csharp_config_is_said_and_the_rest_still_comes() {
        let scratch = Scratch::new("import-unreadable");
        let directories = Directories::of(&scratch.mono());
        put(
            &directories.from_user.join("config.xml"),
            b"<Config><unclosed></Config>",
        );
        put(&directories.from_user.join("poi.txt"), b"poi");
        let outcome = import_settings(&directories);
        assert_eq!(outcome.config, None);
        assert_eq!(outcome.names, ["poi.txt".to_owned()]);
        assert_eq!(outcome.failed.len(), 1);
        assert!(outcome.failed[0].starts_with("config.xml: "), "{:?}", outcome.failed);
        assert!(directories.to_user.join("poi.txt").os_is_file());

        // An empty one is no settings, as `Load` reads it.
        put(&directories.from_user.join("config.xml"), b"");
        let outcome = import_settings(&directories);
        assert_eq!(outcome.config, Some(Config::default()));
        assert!(outcome.failed.is_empty(), "{:?}", outcome.failed);
    }
}
