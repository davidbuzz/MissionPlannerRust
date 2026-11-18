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

//! Installing an update in place, with no second program: the release's archive fetched and
//! checked, unpacked beside the installed planner, the new planner tried, and then each installed
//! file moved aside as `<file>.old` and its new one moved into its name - any failure moving
//! everything back. The old files go at the next start. Where the C# downloads each changed file
//! as `<file>.new` and starts `Updater.exe` to move them (`Update.cs`, `Updater/Program.cs`),
//! this moves them itself: updates.md, section 4.6, and section 9 for why it is a move.
//!
//! The rules the tests of 2026-11-18 left, kept here: a file is replaced by moving a fresh one
//! into its name, never by writing into it (macOS kills a program whose file was written into
//! while it ran); the old file cannot be deleted in the run that replaced it (Windows), so it is
//! listed and deleted by the next.
//! `// C#: Utilities/Update.cs:684-738; Updater/Program.cs:84-178; Program.cs:616-628`

use std::path::{Component, Path, PathBuf};

use mp_os::fs::FsExt as _;

use crate::download::{Source, USER_REQUEST, fetch_file};
use crate::manifest::{DEB, Manifest};

/// The folder beside the planner an update is fetched and unpacked in; gone when it ends.
pub const WORK: &str = ".update";
/// The list, beside the planner, of the `.old` files the last update left for the next start.
pub const OLD_LIST: &str = ".update-old.txt";
/// `// C#: Utilities/Update.cs:711`
pub const UNWRITABLE: &str = "Unable to write to the install directory";

/// How this planner came to be where it is, which decides what an update may do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InstallKind {
    /// An archive unpacked into a folder: updated in place.
    Archive,
    /// The Debian package: the package manager's to replace, so the new package is fetched and
    /// its command shown.
    Deb,
}

/// The kind of install a planner at `exe` is: the package's when it is under `/usr` on Linux.
#[must_use]
pub fn install_kind(exe: &Path) -> InstallKind {
    if cfg!(target_os = "linux") && exe.starts_with("/usr") {
        InstallKind::Deb
    } else {
        InstallKind::Archive
    }
}

/// `DoUpdateWorker_DoWork`'s write test: a file written into the install directory and removed.
///
/// # Errors
///
/// [`UNWRITABLE`], with why. `// C#: Utilities/Update.cs:700-722`
pub fn write_test(install_dir: &Path) -> Result<(), String> {
    let probe = install_dir.join("writetest.txt");
    let outcome = mp_os::fs::write(&probe, "this is a test");
    let _ = mp_os::fs::remove_file(&probe);
    outcome.map_err(|why| format!("{UNWRITABLE}: {why}"))
}

/// The new planner tried before anything is replaced: `--version` must run and name the
/// manifest's release, which a build this machine cannot start (an older C library, another
/// processor), or a download that is not that release, does not.
///
/// # Errors
///
/// It did not start, or it is not that release.
#[cfg(not(target_family = "wasm"))]
pub fn trial(planner: &Path, manifest: &Manifest) -> Result<(), String> {
    let output = std::process::Command::new(planner)
        .arg("--version")
        // The harness's door is this planner's, not the new one's.
        .env_remove(crate::identity::DOOR)
        .output()
        .map_err(|why| format!("the new planner does not start on this computer: {why}"))?;
    if !output.status.success() {
        return Err(format!(
            "the new planner does not start on this computer: {}",
            output.status
        ));
    }
    let said = String::from_utf8_lossy(&output.stdout);
    let said = said.trim();
    let version = format!(" {} (", manifest.version);
    if said.contains(&version) && said.contains(crate::identity::short(&manifest.commit)) {
        Ok(())
    } else {
        Err(format!(
            "the download is not release {}: it says {said:?}",
            manifest.tag
        ))
    }
}

/// What an update did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Installed {
    /// The files moved into place.
    pub files: usize,
    /// The installed files moved aside, for the next start to delete.
    pub old: usize,
}

/// `<file>.old`, beside it.
fn aside(target: &Path) -> PathBuf {
    let mut name = target.file_name().unwrap_or_default().to_os_string();
    name.push(".old");
    target.with_file_name(name)
}

/// Each staged file moved into its installed name, the installed one moved aside first; on any
/// failure every move is undone, so the install is the old one whole or the new one whole.
/// Returns the files moved aside.
fn swap(staged: &Path, files: &[PathBuf], install_dir: &Path) -> Result<Vec<PathBuf>, String> {
    // What has been done: a file now in place, and the old one beside it where there was one.
    let mut done: Vec<(PathBuf, Option<PathBuf>)> = Vec::new();
    let mut step = || -> Result<(), String> {
        for relative in files {
            let target = install_dir.join(relative);
            let old = aside(&target);
            let failed = |why: std::io::Error| format!("{}: {why}", target.display());
            if let Some(parent) = target.parent() {
                mp_os::fs::create_dir_all(parent).map_err(failed)?;
            }
            let had = target.symlink_metadata().is_ok();
            if had {
                // One left by an update before this one, which its program no longer runs from.
                let _ = mp_os::fs::remove_file(&old);
                mp_os::fs::rename(&target, &old).map_err(failed)?;
            }
            done.push((target.clone(), had.then(|| old.clone())));
            mp_os::fs::rename(staged.join(relative), &target).map_err(failed)?;
        }
        Ok(())
    };
    if let Err(why) = step() {
        for (target, old) in done.iter().rev() {
            let _ = mp_os::fs::remove_file(target);
            if let Some(old) = old {
                let _ = mp_os::fs::rename(old, target);
            }
        }
        return Err(why);
    }
    Ok(done.into_iter().filter_map(|(_, old)| old).collect())
}

/// The update, whole: the write test, the platform's download fetched into [`WORK`] and checked
/// against the manifest, unpacked, the new planner tried with `trial`, the files swapped, and the
/// old ones listed in [`OLD_LIST`] for [`cleanup`]. Until the swap nothing installed is touched,
/// and a failure or Cancel at any point leaves the install as it was.
///
/// `program` is the planner's file name in the archive, `planner` or `planner.exe`; `progress`
/// is the dialog's `UpdateProgressAndStatus(percent, text)`, -1 a marquee; `cancel_requested`
/// its Cancel, seen until the swap begins.
///
/// # Errors
///
/// [`UNWRITABLE`]; no download for the platform; the download's errors
/// ([`crate::download::fetch_file`]); the archive's ([`crate::unpack::unpack`]); the trial's; a
/// file that could not be moved.
#[allow(clippy::too_many_arguments)]
pub fn install(
    source: &dyn Source,
    manifest: &Manifest,
    platform: &str,
    install_dir: &Path,
    program: &str,
    trial: &dyn Fn(&Path, &Manifest) -> Result<(), String>,
    progress: &mut dyn FnMut(i32, &str),
    cancel_requested: &dyn Fn() -> bool,
) -> Result<Installed, String> {
    progress(-1, "Checking the install directory");
    write_test(install_dir)?;
    let asset = manifest.asset(platform)?;
    let work = install_dir.join(WORK);
    let _ = mp_os::fs::remove_dir_all(&work);
    mp_os::fs::create_dir_all(&work).map_err(|why| format!("{UNWRITABLE}: {why}"))?;
    let outcome = (|| {
        let archive = work.join(format!("{}.part", asset.name));
        fetch_file(source, asset, &archive, progress, cancel_requested)?;
        progress(-1, &format!("Unpacking {}", asset.name));
        let staged = work.join("staged");
        let files = crate::unpack::unpack(&archive, &staged)?;
        if !files.iter().any(|file| file == Path::new(program)) {
            return Err(format!("{} holds no {program}", asset.name));
        }
        progress(-1, "Checking the new planner");
        trial(&staged.join(program), manifest)?;
        if cancel_requested() {
            return Err(USER_REQUEST.to_owned());
        }
        progress(-1, "Installing");
        let old = swap(&staged, &files, install_dir)?;
        let listed: Vec<String> = old
            .iter()
            .filter_map(|path| path.strip_prefix(install_dir).ok())
            .map(|path| path.to_string_lossy().replace('\\', "/"))
            .collect();
        // Without the list the old files stay, which costs disk and breaks nothing.
        let _ = mp_os::fs::write(install_dir.join(OLD_LIST), listed.join("\n"));
        Ok(Installed {
            files: files.len(),
            old: old.len(),
        })
    })();
    let _ = mp_os::fs::remove_dir_all(&work);
    outcome
}

/// At the next start: the files the last update moved aside deleted - those [`OLD_LIST`] names
/// and no others, so nothing of the user's that happens to end in `.old` is touched - and what an
/// interrupted update left in [`WORK`]. Returns how many were deleted; one still in use is kept
/// on the list for the start after.
/// `// C#: Updater/Program.cs:45-57, 96-118; Program.cs:616-628`
pub fn cleanup(install_dir: &Path) -> usize {
    let _ = mp_os::fs::remove_dir_all(install_dir.join(WORK));
    let list = install_dir.join(OLD_LIST);
    let Ok(text) = mp_os::fs::read_to_string(&list) else {
        return 0;
    };
    let mut deleted = 0;
    let mut kept = Vec::new();
    for line in text.lines().map(str::trim).filter(|line| !line.is_empty()) {
        let relative = Path::new(line);
        let inside = relative
            .components()
            .all(|c| matches!(c, Component::Normal(_)));
        if !inside || !line.ends_with(".old") {
            continue;
        }
        let path = install_dir.join(relative);
        match mp_os::fs::remove_file(&path) {
            Ok(()) => deleted += 1,
            Err(_) if path.symlink_metadata().is_ok() => kept.push(line),
            Err(_) => {}
        }
    }
    if kept.is_empty() {
        let _ = mp_os::fs::remove_file(&list);
    } else {
        let _ = mp_os::fs::write(&list, kept.join("\n"));
    }
    deleted
}

/// The user's Downloads folder, their home where there is none, the temporary folder where
/// there is neither.
#[must_use]
pub fn downloads_dir() -> PathBuf {
    let home = std::env::var_os("HOME")
        .or_else(|| std::env::var_os("USERPROFILE"))
        .map(PathBuf::from);
    match home {
        Some(home) if home.join("Downloads").os_is_dir() => home.join("Downloads"),
        Some(home) if home.os_is_dir() => home,
        _ => mp_os::temp_dir(),
    }
}

/// For the Debian package's install: the release's `.deb` fetched into `downloads` and checked
/// against the manifest. The package manager installs it, with [`deb_command`].
///
/// # Errors
///
/// The release has no `.deb`; the download's errors.
pub fn fetch_deb(
    source: &dyn Source,
    manifest: &Manifest,
    downloads: &Path,
    progress: &mut dyn FnMut(i32, &str),
    cancel_requested: &dyn Fn() -> bool,
) -> Result<PathBuf, String> {
    let asset = manifest.asset(DEB)?;
    let target = downloads.join(&asset.name);
    let part = downloads.join(format!("{}.part", asset.name));
    fetch_file(source, asset, &part, progress, cancel_requested)?;
    mp_os::fs::rename(&part, &target).map_err(|why| {
        let _ = mp_os::fs::remove_file(&part);
        format!("{}: {why}", target.display())
    })?;
    Ok(target)
}

/// The command that installs a fetched package, its path quoted where a shell would split it.
#[must_use]
pub fn deb_command(package: &Path) -> String {
    let path = package.to_string_lossy();
    let plain = path
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || matches!(c, '/' | '.' | '_' | '-' | '+' | '~'));
    if plain {
        format!("sudo apt install {path}")
    } else {
        format!("sudo apt install '{}'", path.replace('\'', "'\\''"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::download::tests::{Server, asset_of, scratch};
    use crate::manifest::tests::COMMIT;
    use crate::unpack::tests::tar_gz;

    const URL: &str = "https://x/missionplanner-rust_0_0_2_linux-x86_64.tar.gz";
    const NAME: &str = "missionplanner-rust_0_0_2_linux-x86_64.tar.gz";

    /// A planner that answers `--version` as release `version` would: a shell script.
    fn planner_saying(version: &str) -> Vec<u8> {
        format!("#!/bin/sh\necho 'planner {version} (stable, commit 42992d086a78, built 2026-11-20T03:10:00Z)'\n")
            .into_bytes()
    }

    /// A release 0.0.2 whose platform download is those bytes.
    fn release(platform: &str, name: &str, url: &str, bytes: &[u8]) -> Manifest {
        let mut manifest = Manifest::parse(
            crate::manifest::tests::text("stable", "0.0.2", COMMIT, "2026-11-20T03:10:00Z")
                .as_bytes(),
        )
        .unwrap();
        manifest
            .assets
            .insert(platform.to_owned(), asset_of(name, url, bytes));
        manifest
    }

    /// An install of the old planner, with a file of the user's that ends in `.old`.
    fn installed(dir: &Path) {
        mp_os::fs::create_dir_all(dir.join("sitl-wasm")).unwrap();
        mp_os::fs::write(dir.join("planner"), b"the old planner").unwrap();
        mp_os::fs::write(dir.join("LICENSE"), b"old licence").unwrap();
        mp_os::fs::write(dir.join("sitl-wasm/arducopter.wasm"), b"old wasm").unwrap();
        mp_os::fs::write(dir.join("notes.old"), b"the user's own").unwrap();
    }

    fn archive(planner: &[u8]) -> Vec<u8> {
        tar_gz(&[
            (
                "missionplanner-rust_0_0_2_linux-x86_64/planner",
                planner,
                0o755,
            ),
            (
                "missionplanner-rust_0_0_2_linux-x86_64/LICENSE",
                b"new licence",
                0o644,
            ),
            (
                "missionplanner-rust_0_0_2_linux-x86_64/sitl-wasm/arducopter.wasm",
                b"new wasm",
                0o644,
            ),
            (
                "missionplanner-rust_0_0_2_linux-x86_64/sitl-wasm/arduplane.wasm",
                b"a new file",
                0o644,
            ),
        ])
    }

    fn passes(_: &Path, _: &Manifest) -> Result<(), String> {
        Ok(())
    }

    /// The install as it was before any update: the old files, and nothing of an update's.
    fn untouched(dir: &Path) {
        assert_eq!(
            mp_os::fs::read(dir.join("planner")).unwrap(),
            b"the old planner"
        );
        assert_eq!(
            mp_os::fs::read(dir.join("LICENSE")).unwrap(),
            b"old licence"
        );
        assert_eq!(
            mp_os::fs::read(dir.join("sitl-wasm/arducopter.wasm")).unwrap(),
            b"old wasm"
        );
        assert!(!dir.join("planner.old").os_exists());
        assert!(!dir.join("sitl-wasm/arduplane.wasm").os_exists());
        assert!(!dir.join(WORK).os_exists());
        assert!(!dir.join(OLD_LIST).os_exists());
    }

    #[test]
    fn an_update_moves_the_new_files_in_and_the_old_aside_and_the_next_start_deletes_them() {
        let dir = scratch("install");
        installed(&dir);
        let bytes = archive(&planner_saying("0.0.2"));
        let server = Server::with(&[(URL, &bytes)]);
        let manifest = release("linux-x86_64", NAME, URL, &bytes);
        let mut said = Vec::new();
        // The real trial where a script can stand for the planner.
        let try_it: &dyn Fn(&Path, &Manifest) -> Result<(), String> =
            if cfg!(unix) { &trial } else { &passes };
        let done = install(
            &server,
            &manifest,
            "linux-x86_64",
            &dir,
            "planner",
            try_it,
            &mut |_, text| said.push(text.to_owned()),
            &|| false,
        )
        .unwrap();
        assert_eq!(done, Installed { files: 4, old: 3 });
        assert_eq!(
            mp_os::fs::read(dir.join("planner")).unwrap(),
            planner_saying("0.0.2")
        );
        assert_eq!(
            mp_os::fs::read(dir.join("LICENSE")).unwrap(),
            b"new licence"
        );
        assert_eq!(
            mp_os::fs::read(dir.join("sitl-wasm/arducopter.wasm")).unwrap(),
            b"new wasm"
        );
        assert_eq!(
            mp_os::fs::read(dir.join("sitl-wasm/arduplane.wasm")).unwrap(),
            b"a new file"
        );
        // The old ones beside them, listed; the work folder gone.
        assert_eq!(
            mp_os::fs::read(dir.join("planner.old")).unwrap(),
            b"the old planner"
        );
        assert_eq!(
            mp_os::fs::read(dir.join("sitl-wasm/arducopter.wasm.old")).unwrap(),
            b"old wasm"
        );
        let mut listed: Vec<String> = mp_os::fs::read_to_string(dir.join(OLD_LIST))
            .unwrap()
            .lines()
            .map(str::to_owned)
            .collect();
        listed.sort();
        assert_eq!(
            listed,
            [
                "LICENSE.old",
                "planner.old",
                "sitl-wasm/arducopter.wasm.old"
            ]
        );
        assert!(!dir.join(WORK).os_exists());
        assert_eq!(said.first().unwrap(), "Checking the install directory");
        assert_eq!(said.last().unwrap(), "Installing");
        assert!(said.contains(&format!("Unpacking {NAME}")));

        // The next start: the listed files deleted, the list with them, the user's file left.
        assert_eq!(cleanup(&dir), 3);
        assert!(!dir.join("planner.old").os_exists());
        assert!(!dir.join("sitl-wasm/arducopter.wasm.old").os_exists());
        assert!(!dir.join(OLD_LIST).os_exists());
        assert_eq!(
            mp_os::fs::read(dir.join("notes.old")).unwrap(),
            b"the user's own"
        );
        assert_eq!(cleanup(&dir), 0);
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_failure_before_the_swap_leaves_the_install_as_it_was() {
        let dir = scratch("install-fails");
        installed(&dir);
        let bytes = archive(&planner_saying("0.0.2"));
        let server = Server::with(&[(URL, &bytes)]);
        let manifest = release("linux-x86_64", NAME, URL, &bytes);
        let run = |manifest: &Manifest,
                   platform: &str,
                   program: &str,
                   try_it: &dyn Fn(&Path, &Manifest) -> Result<(), String>,
                   cancel: &dyn Fn() -> bool| {
            install(
                &server,
                manifest,
                platform,
                &dir,
                program,
                try_it,
                &mut |_, _| {},
                cancel,
            )
            .unwrap_err()
        };

        // A download that is not the manifest's.
        let mut other = manifest.clone();
        other.assets.insert(
            "linux-x86_64".to_owned(),
            asset_of(NAME, URL, b"something else"),
        );
        assert_eq!(
            run(&other, "linux-x86_64", "planner", &passes, &|| false),
            format!("File downloaded does not match hash: {NAME}")
        );
        untouched(&dir);
        // No download for this platform, and an archive without the planner in it.
        assert_eq!(
            run(&manifest, "windows-x86_64", "planner", &passes, &|| false),
            "release v0_0_2 has no download for windows-x86_64"
        );
        untouched(&dir);
        assert_eq!(
            run(&manifest, "linux-x86_64", "planner.exe", &passes, &|| false),
            format!("{NAME} holds no planner.exe")
        );
        untouched(&dir);
        // The new planner refused by its trial.
        let refuses = |_: &Path, _: &Manifest| {
            Err("the new planner does not start on this computer".to_owned())
        };
        assert_eq!(
            run(&manifest, "linux-x86_64", "planner", &refuses, &|| false),
            "the new planner does not start on this computer"
        );
        untouched(&dir);
        // Cancel.
        assert_eq!(
            run(&manifest, "linux-x86_64", "planner", &passes, &|| true),
            USER_REQUEST
        );
        untouched(&dir);
        // And a directory that cannot be written.
        let error = install(
            &server,
            &manifest,
            "linux-x86_64",
            &dir.join("no-such-folder"),
            "planner",
            &passes,
            &mut |_, _| {},
            &|| false,
        )
        .unwrap_err();
        assert!(error.starts_with(UNWRITABLE), "{error}");
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    #[cfg(unix)]
    #[test]
    fn the_trial_runs_the_new_planner_and_holds_it_to_the_manifest() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = scratch("trial");
        let manifest = release("linux-x86_64", NAME, URL, b"x");
        let planner = dir.join("planner");
        let write = |bytes: &[u8]| {
            mp_os::fs::write(&planner, bytes).unwrap();
            mp_os::fs::set_permissions(&planner, mp_os::fs::Permissions::from_mode(0o755)).unwrap();
        };
        write(&planner_saying("0.0.2"));
        assert_eq!(trial(&planner, &manifest), Ok(()));
        // Another release than the manifest's, by its version and by its commit.
        write(&planner_saying("0.0.1"));
        assert!(
            trial(&planner, &manifest)
                .unwrap_err()
                .starts_with("the download is not release v0_0_2")
        );
        write(b"#!/bin/sh\necho 'planner 0.0.2 (stable, commit 000000000000)'\n");
        assert!(
            trial(&planner, &manifest)
                .unwrap_err()
                .starts_with("the download is not release v0_0_2")
        );
        // One that fails, and one that is no program at all.
        write(b"#!/bin/sh\nexit 3\n");
        assert!(
            trial(&planner, &manifest)
                .unwrap_err()
                .starts_with("the new planner does not start")
        );
        mp_os::fs::write(&planner, b"not a program").unwrap();
        mp_os::fs::set_permissions(&planner, mp_os::fs::Permissions::from_mode(0o644)).unwrap();
        assert!(
            trial(&planner, &manifest)
                .unwrap_err()
                .starts_with("the new planner does not start")
        );
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_swap_that_fails_part_way_is_undone() {
        let dir = scratch("swap");
        installed(&dir);
        let staged = dir.join(WORK).join("staged");
        mp_os::fs::create_dir_all(&staged).unwrap();
        mp_os::fs::write(staged.join("planner"), b"the new planner").unwrap();
        mp_os::fs::write(staged.join("LICENSE"), b"new licence").unwrap();
        // The third file is not there to be moved: the two before it go back.
        let files = [
            PathBuf::from("planner"),
            PathBuf::from("LICENSE"),
            PathBuf::from("sitl-wasm/arducopter.wasm"),
        ];
        assert!(swap(&staged, &files, &dir).is_err());
        assert_eq!(
            mp_os::fs::read(dir.join("planner")).unwrap(),
            b"the old planner"
        );
        assert_eq!(
            mp_os::fs::read(dir.join("LICENSE")).unwrap(),
            b"old licence"
        );
        assert_eq!(
            mp_os::fs::read(dir.join("sitl-wasm/arducopter.wasm")).unwrap(),
            b"old wasm"
        );
        assert!(!dir.join("planner.old").os_exists());
        assert!(!dir.join("LICENSE.old").os_exists());
        assert!(!dir.join("sitl-wasm/arducopter.wasm.old").os_exists());
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    #[test]
    fn cleanup_deletes_only_what_the_list_names_inside_the_install() {
        let dir = scratch("cleanup");
        installed(&dir);
        mp_os::fs::write(dir.join("planner.old"), b"old").unwrap();
        mp_os::fs::write(dir.join("planner"), b"kept").unwrap();
        let outside = dir
            .parent()
            .unwrap()
            .join(format!("mp-update-outside-{}.old", mp_os::process_id()));
        mp_os::fs::write(&outside, b"not ours").unwrap();
        mp_os::fs::create_dir_all(dir.join(WORK).join("staged")).unwrap();
        mp_os::fs::write(
            dir.join(OLD_LIST),
            format!(
                "planner.old\nplanner\n../{}\n/etc/passwd.old\ngone.old\n",
                outside.file_name().unwrap().to_string_lossy()
            ),
        )
        .unwrap();
        assert_eq!(cleanup(&dir), 1);
        assert!(!dir.join("planner.old").os_exists());
        assert_eq!(mp_os::fs::read(dir.join("planner")).unwrap(), b"kept");
        assert_eq!(mp_os::fs::read(&outside).unwrap(), b"not ours");
        assert_eq!(
            mp_os::fs::read(dir.join("notes.old")).unwrap(),
            b"the user's own"
        );
        assert!(!dir.join(WORK).os_exists());
        assert!(!dir.join(OLD_LIST).os_exists());
        let _ = mp_os::fs::remove_file(&outside);
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_debian_package_is_fetched_checked_and_its_command_said() {
        let dir = scratch("deb");
        let bytes = b"!<arch>\ndebian-binary".to_vec();
        let url = "https://x/missionplanner-rust_0.0.2_amd64.deb";
        let server = Server::with(&[(url, &bytes)]);
        let manifest = release(DEB, "missionplanner-rust_0.0.2_amd64.deb", url, &bytes);
        let package = fetch_deb(&server, &manifest, &dir, &mut |_, _| {}, &|| false).unwrap();
        assert_eq!(package, dir.join("missionplanner-rust_0.0.2_amd64.deb"));
        assert_eq!(mp_os::fs::read(&package).unwrap(), bytes);
        assert!(
            !dir.join("missionplanner-rust_0.0.2_amd64.deb.part")
                .os_exists()
        );
        // Not the manifest's bytes: nothing kept.
        let _ = mp_os::fs::remove_file(&package);
        let wrong = release(DEB, "missionplanner-rust_0.0.2_amd64.deb", url, b"other");
        assert!(fetch_deb(&server, &wrong, &dir, &mut |_, _| {}, &|| false).is_err());
        assert!(!package.os_exists());

        assert_eq!(
            deb_command(Path::new(
                "/home/buzz/Downloads/missionplanner-rust_0.0.2_amd64.deb"
            )),
            "sudo apt install /home/buzz/Downloads/missionplanner-rust_0.0.2_amd64.deb"
        );
        assert_eq!(
            deb_command(Path::new("/home/a b/it's.deb")),
            "sudo apt install '/home/a b/it'\\''s.deb'"
        );
        assert_eq!(
            install_kind(Path::new("/home/buzz/planner/planner")),
            InstallKind::Archive
        );
        if cfg!(target_os = "linux") {
            assert_eq!(
                install_kind(Path::new("/usr/bin/planner")),
                InstallKind::Deb
            );
        }
        let _ = mp_os::fs::remove_dir_all(&dir);
    }
}
