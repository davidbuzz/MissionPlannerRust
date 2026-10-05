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

//! `Updater/Program.cs`: the separate program the planner starts and exits for, which waits five
//! seconds, moves every `<file>.new` under its directory into place - the file there first to
//! `<file>.old`, ten tries half a second apart - deletes the `.old` files, and starts the planner
//! again. It never replaces itself: the planner copies `<updater>.new` over the updater at its
//! next start (`Program.CleanupFiles`). `// C#: Updater/Program.cs`

use mp_os::fs::FsExt as _;
use std::path::Path;
use std::time::Duration;

/// "give 4 seconds grace" - five, in the code. `// C#: Updater/Program.cs:30`
pub const GRACE: Duration = Duration::from_secs(5);
/// Between the tries at a file in use. `// C#: Updater/Program.cs:149`
pub const RETRY: Duration = Duration::from_millis(500);
/// What the console says when a file could not be moved. `// C#: Updater/Program.cs:37-38`
pub const FAILED: &str = "Update failed, please try it later.\nPress any key to continue.";

/// `File.SetAttributes(file, FileAttributes.Normal)` then `File.Delete`: a read-only file made
/// writable and removed.
fn force_remove(file: &Path) {
    if mp_os::fs::remove_file(file).is_err() {
        if let Ok(metadata) = mp_os::fs::metadata(file) {
            let mut permissions = metadata.permissions();
            #[allow(clippy::permissions_set_readonly_false)] // the C#'s FileAttributes.Normal
            permissions.set_readonly(false);
            let _ = mp_os::fs::set_permissions(file, permissions);
        }
        let _ = mp_os::fs::remove_file(file);
    }
}

/// A name's lower-case form has `needle` in it.
fn lower_contains(path: &Path, needle: &str) -> bool {
    path.to_string_lossy().to_lowercase().contains(needle)
}

/// `UpdateFiles(directory)`: the `.old` files deleted, each `.new` moved into place with the
/// file there kept as `.old`, ten tries each, then the subdirectories; `updater_name` (the C#'s
/// "updater.exe") is the file this program never replaces, being itself. Returns whether every
/// file went. `// C#: Updater/Program.cs:81-174`
#[must_use]
pub fn update_files(directory: &Path, updater_name: &str) -> bool {
    let mut all_done = true;
    let needle = updater_name.to_lowercase();
    if let Ok(entries) = mp_os::fs::read_dir(directory) {
        let files: Vec<_> = entries
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.os_is_file())
            .collect();
        // cleanup old
        for file in files.iter().filter(|f| {
            lower_contains(f, "") && f.to_string_lossy().to_lowercase().ends_with(".old")
        }) {
            force_remove(file);
        }
        for file in &files {
            let lower = file.to_string_lossy().to_lowercase();
            if !lower.ends_with(".new") || lower.ends_with("/.new") || lower == ".new" {
                continue;
            }
            let mut done = false;
            for _ in 0..10 {
                if done {
                    break;
                }
                if lower_contains(file, &needle) {
                    // cant self update on windows
                    done = true;
                    break;
                }
                let text = file.to_string_lossy();
                let stem = &text[..text.len().saturating_sub(4)];
                let newfile = Path::new(stem);
                let oldfile = std::path::PathBuf::from(format!("{stem}.old"));
                let moved = (|| -> std::io::Result<()> {
                    // move existing to .old
                    if newfile.os_exists() {
                        mp_os::fs::rename(newfile, &oldfile)?;
                    }
                    // move .new to existing
                    mp_os::fs::rename(file, newfile)
                })();
                match moved {
                    Ok(()) => done = true,
                    Err(_) => {
                        wasm_thread::sleep(RETRY);
                        // normally in use by explorer.exe
                        if lower_contains(file, "tlogthumbnailhandler") {
                            done = true;
                        }
                    }
                }
            }
            all_done = all_done && done;
        }
    }
    if let Ok(entries) = mp_os::fs::read_dir(directory) {
        for sub in entries.flatten().map(|e| e.path()).filter(|p| p.os_is_dir()) {
            all_done = all_done && update_files(&sub, updater_name);
        }
    }
    all_done
}

/// `Main`, after the grace: the files moved, the `.old` files in the directory itself deleted,
/// and the planner started again from that directory. `wait` is whether to sleep the grace first
/// (the real run does; tests do not).
///
/// # Errors
///
/// [`FAILED`] when a file could not be moved; the planner's start error when it would not start.
/// `// C#: Updater/Program.cs:16-79`
pub fn run(directory: &Path, planner: &Path, updater_name: &str, wait: bool) -> Result<(), String> {
    if wait {
        wasm_thread::sleep(GRACE);
    }
    if !update_files(directory, updater_name) {
        return Err(FAILED.to_owned());
    }
    if let Ok(entries) = mp_os::fs::read_dir(directory) {
        for file in entries.flatten().map(|e| e.path()) {
            if file.to_string_lossy().to_lowercase().ends_with(".old") {
                force_remove(&file);
            }
        }
    }
    std::process::Command::new(planner)
        .current_dir(directory)
        .spawn()
        .map(|_| ())
        .map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use mp_os::fs::FsExt as _;
    use super::*;

    fn scratch(test: &str) -> std::path::PathBuf {
        let dir = mp_os::temp_dir().join(format!("mp-update-apply-{test}-{}", mp_os::process_id()));
        let _ = mp_os::fs::remove_dir_all(&dir);
        mp_os::fs::create_dir_all(dir.join("sub")).unwrap();
        dir
    }

    #[test]
    fn new_files_move_into_place_and_old_ones_go() {
        let dir = scratch("move");
        mp_os::fs::write(dir.join("a.txt"), b"old a").unwrap();
        mp_os::fs::write(dir.join("a.txt.new"), b"new a").unwrap();
        mp_os::fs::write(dir.join("b.bin.new"), b"new b").unwrap();
        mp_os::fs::write(dir.join("stale.old"), b"stale").unwrap();
        mp_os::fs::write(dir.join("sub/c.new"), b"new c").unwrap();
        mp_os::fs::write(dir.join("headless-planner.new"), b"updater").unwrap();
        assert!(update_files(&dir, "headless-planner"));
        assert_eq!(mp_os::fs::read(dir.join("a.txt")).unwrap(), b"new a");
        assert_eq!(mp_os::fs::read(dir.join("a.txt.old")).unwrap(), b"old a");
        assert_eq!(mp_os::fs::read(dir.join("b.bin")).unwrap(), b"new b");
        assert!(!dir.join("b.bin.new").os_exists());
        assert!(!dir.join("stale.old").os_exists());
        assert_eq!(mp_os::fs::read(dir.join("sub/c")).unwrap(), b"new c");
        // The updater itself is left for the planner to copy over.
        assert!(dir.join("headless-planner.new").os_exists());
        mp_os::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn run_clears_the_old_files_and_starts_the_planner() {
        let dir = scratch("run");
        mp_os::fs::write(dir.join("a.txt"), b"old").unwrap();
        mp_os::fs::write(dir.join("a.txt.new"), b"new").unwrap();
        // "The planner" here is `true`, which starts and exits.
        run(&dir, Path::new("true"), "headless-planner", false).unwrap();
        assert_eq!(mp_os::fs::read(dir.join("a.txt")).unwrap(), b"new");
        assert!(!dir.join("a.txt.old").os_exists());
        let missing = run(
            &dir,
            &dir.join("no-such-planner"),
            "headless-planner",
            false,
        );
        assert!(missing.is_err());
        // The planner was started standing in `dir` and not waited for, as the C# starts it, and
        // Windows will not remove a folder a running process stands in: the cleanup waits for
        // `true` to have gone (CI run 37173996196, "being used by another process").
        let until = web_time::Instant::now() + std::time::Duration::from_secs(10);
        while let Err(e) = mp_os::fs::remove_dir_all(&dir) {
            assert!(web_time::Instant::now() < until, "{}: {e}", dir.display());
            wasm_thread::sleep(std::time::Duration::from_millis(50));
        }
    }
}
