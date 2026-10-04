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

//! The one-shot import of Mission Planner's files into this application's own directory.
//!
//! This application keeps its files under [`crate::APP_CONFIG_NAME`], not under the C#'s
//! `Mission Planner` directory, by the owner's ruling of 2026-09-25 (PLAN.md section 12, D11): a
//! user running both applications must not lose data to the other one's writes. A pilot moving
//! over should not have to set everything up again either, so on the first start that finds this
//! application's user data directory missing or empty while the C#'s exists, the C#'s files are
//! copied across once, each artefact by name ([`ARTEFACTS`]), and a marker file ([`MARKER`])
//! records what was copied. The marker makes the directory non-empty, so the import never runs a
//! second time - not even when nothing was found to copy.
//!
//! The C#'s directory is only ever read: nothing in it is written, renamed or deleted, and a file
//! this application already has is never overwritten.
//!
//! What is not imported, on purpose: the map tile cache (`gmapcache`), the terrain tiles (`srtm`),
//! the flight logs and recordings (`logs`), and the parameter metadata (`*.pdef.xml` and the
//! `LogMessages*.xml` files). They are large - gigabytes of tiles and logs on a machine that has
//! flown - and every one of them is re-created on demand: tiles and terrain are downloaded again,
//! the metadata is fetched again, and the logs stay where the C# wrote them, readable from there.
//!
//! Each artefact is copied from the C# directory the C# keeps it in to the same directory under
//! this application's name: the user data directory for most, the shared data directory for
//! `UserAlerts.json` and `History` (the two differ on Windows only). `logo.png` and `logo.txt` the
//! C# reads beside its executable (`GetRunningDirectory`, `Program.cs:209-221`), which is not a
//! data directory; they are copied from the user data directory if a user put them there.

use std::fmt::Write as _;
use std::io;
use std::path::{Path, PathBuf};

use crate::Folders;

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
    // `// C#: Program.cs:220` (beside the executable in the C#)
    ("logo.png", Kept::User),
    // `// C#: Program.cs:209-211` (beside the executable in the C#)
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

/// Imports for this process's directories: what the applications call at start-up, before
/// anything reads or writes the data directory.
#[must_use]
pub fn import_at_start() -> Import {
    Folders::from_environment().map_or(Import::NoHome, |folders| import(&Directories::of(&folders)))
}

/// Imports the C#'s artefacts once, if this application's user data directory is missing or
/// empty and the C#'s exists.
#[must_use]
pub fn import(directories: &Directories) -> Import {
    if !is_missing_or_empty(&directories.to_user) {
        return Import::AlreadyHere;
    }
    if !directories.from_user.is_dir() && !directories.from_shared.is_dir() {
        return Import::NothingToImport;
    }
    if let Err(err) = std::fs::create_dir_all(&directories.to_user) {
        return Import::Failed(format!("{}: {err}", directories.to_user.display()));
    }
    let mut names = Vec::new();
    let mut failed = Vec::new();
    for &(name, kept) in ARTEFACTS {
        let source = directories.from(kept).join(name);
        let target = directories.to(kept).join(name);
        // A file this application already has (a shared directory on Windows can) is its own.
        if std::fs::symlink_metadata(&source).is_err() || std::fs::symlink_metadata(&target).is_ok()
        {
            continue;
        }
        match copy(&source, &target) {
            Ok(()) => names.push(name.to_owned()),
            Err(err) => failed.push(format!("{name}: {err}")),
        }
    }
    let marker = marker_text(directories, &names, &failed);
    if let Err(err) = std::fs::write(directories.to_user.join(MARKER), marker) {
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
    match std::fs::read_dir(directory) {
        Ok(mut entries) => entries.next().is_none(),
        Err(err) => err.kind() == io::ErrorKind::NotFound,
    }
}

/// Copies a file, or a folder and everything in it. Reads the source only.
fn copy(source: &Path, target: &Path) -> io::Result<()> {
    if std::fs::metadata(source)?.is_dir() {
        copy_tree(source, target)
    } else {
        if let Some(parent) = target.parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::copy(source, target).map(|_| ())
    }
}

/// Copies a folder's contents. A link inside it is copied as what it points at when that is a
/// file, and is an error when it is a folder, so a loop of links cannot recurse for ever.
fn copy_tree(source: &Path, target: &Path) -> io::Result<()> {
    std::fs::create_dir_all(target)?;
    for entry in std::fs::read_dir(source)? {
        let entry = entry?;
        let to = target.join(entry.file_name());
        if entry.file_type()?.is_dir() {
            copy_tree(&entry.path(), &to)?;
        } else {
            std::fs::copy(entry.path(), &to)?;
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

#[cfg(test)]
mod tests {
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
            let _ = std::fs::remove_dir_all(&path);
            std::fs::create_dir_all(&path).expect("scratch");
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
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// Every file under a directory, by path relative to it, with its bytes; folders as `None`.
    fn snapshot(root: &Path) -> BTreeMap<PathBuf, Option<Vec<u8>>> {
        fn walk(root: &Path, dir: &Path, into: &mut BTreeMap<PathBuf, Option<Vec<u8>>>) {
            for entry in std::fs::read_dir(dir).expect("readable") {
                let path = entry.expect("entry").path();
                let relative = path.strip_prefix(root).expect("under root").to_path_buf();
                if path.is_dir() {
                    into.insert(relative, None);
                    walk(root, &path, into);
                } else {
                    into.insert(relative, Some(std::fs::read(&path).expect("readable")));
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
        std::fs::create_dir_all(theirs.join("gmapcache")).expect("C# directory");
        std::fs::write(theirs.join("gmapcache").join("tile.png"), b"tile").expect("tile");
        std::fs::write(theirs.join("ArduCopter.apm.pdef.xml"), b"<params/>").expect("pdef");
        if name == "History" {
            let history = theirs.join("History");
            std::fs::create_dir_all(history.join("ArduCopter")).expect("History");
            std::fs::write(history.join("firmware.hex"), content("firmware.hex")).expect("hex");
            std::fs::write(
                history.join("ArduCopter").join("arducopter.apj"),
                content("apj"),
            )
            .expect("apj");
        } else {
            std::fs::write(theirs.join(name), content(name)).expect("artefact");
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
        assert!(!directories.to_user.join("gmapcache").exists());
        assert!(!directories.to_user.join("ArduCopter.apm.pdef.xml").exists());
        let marker = std::fs::read_to_string(directories.to_user.join(MARKER)).expect("the marker");
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
        std::fs::create_dir_all(&directories.from_user).expect("C# directory");
        for &(name, _) in ARTEFACTS {
            if name == "History" {
                std::fs::create_dir_all(directories.from_user.join(name)).expect("History");
            } else {
                std::fs::write(directories.from_user.join(name), content(name)).expect("file");
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
        std::fs::create_dir_all(&directories.from_user).expect("C# directory");
        std::fs::write(directories.from_user.join("config.xml"), b"first").expect("config");
        assert_eq!(import(&directories).names(), ["config.xml".to_owned()]);

        // The C# goes on writing its own file, and a new one; this application keeps what it has.
        std::fs::write(directories.from_user.join("config.xml"), b"second").expect("config");
        std::fs::write(directories.from_user.join("poi.txt"), b"poi").expect("poi");
        let again = import(&directories);
        assert_eq!(again, Import::AlreadyHere);
        assert_eq!(again.summary(), "none");
        assert_eq!(
            std::fs::read(directories.to_user.join("config.xml")).expect("ours"),
            b"first"
        );
        assert!(!directories.to_user.join("poi.txt").exists());
    }

    #[test]
    fn a_start_that_found_nothing_to_copy_still_never_runs_again() {
        let scratch = Scratch::new("found-nothing");
        let directories = Directories::of(&scratch.mono());
        std::fs::create_dir_all(&directories.from_user).expect("an empty C# directory");
        let first = import(&directories);
        assert!(matches!(first, Import::Imported { .. }), "{first:?}");
        assert_eq!(first.summary(), "none");
        assert!(directories.to_user.join(MARKER).is_file());
        std::fs::write(directories.from_user.join("config.xml"), b"later").expect("config");
        assert_eq!(import(&directories), Import::AlreadyHere);
    }

    #[test]
    fn an_existing_directory_with_anything_in_it_is_left_alone() {
        let scratch = Scratch::new("not-empty");
        let directories = Directories::of(&scratch.mono());
        std::fs::create_dir_all(&directories.from_user).expect("C# directory");
        std::fs::write(directories.from_user.join("config.xml"), b"theirs").expect("config");
        std::fs::create_dir_all(&directories.to_user).expect("ours");
        std::fs::write(directories.to_user.join("poi.txt"), b"ours").expect("poi");
        let before = snapshot(&directories.to_user);

        assert_eq!(import(&directories), Import::AlreadyHere);
        assert_eq!(snapshot(&directories.to_user), before);
    }

    #[test]
    fn an_existing_empty_directory_is_imported_into() {
        let scratch = Scratch::new("empty");
        let directories = Directories::of(&scratch.mono());
        std::fs::create_dir_all(&directories.from_user).expect("C# directory");
        std::fs::write(directories.from_user.join("config.xml"), b"theirs").expect("config");
        std::fs::create_dir_all(&directories.to_user).expect("ours, empty");
        assert_eq!(import(&directories).names(), ["config.xml".to_owned()]);
    }

    #[test]
    fn no_csharp_directory_is_fine_and_makes_nothing() {
        let scratch = Scratch::new("no-csharp");
        let directories = Directories::of(&scratch.mono());
        let outcome = import(&directories);
        assert_eq!(outcome, Import::NothingToImport);
        assert_eq!(outcome.summary(), "none");
        assert!(!directories.to_user.exists());
        assert!(!directories.from_user.exists());
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
        std::fs::create_dir_all(&directories.from_user).expect("C# user directory");
        std::fs::create_dir_all(&directories.from_shared).expect("C# shared directory");
        std::fs::write(directories.from_user.join("config.xml"), b"config").expect("config");
        std::fs::write(directories.from_shared.join("UserAlerts.json"), b"{}").expect("alerts");
        // A shared file this application already has is not overwritten.
        std::fs::create_dir_all(directories.from_shared.join("History")).expect("History");
        std::fs::create_dir_all(directories.to_shared.join("History")).expect("our History");

        let outcome = import(&directories);
        assert_eq!(
            outcome.names(),
            ["config.xml".to_owned(), "UserAlerts.json".to_owned()]
        );
        assert_eq!(
            std::fs::read(directories.to_shared.join("UserAlerts.json")).expect("copied"),
            b"{}"
        );
        assert!(!directories.to_user.join("UserAlerts.json").exists());
    }
}
