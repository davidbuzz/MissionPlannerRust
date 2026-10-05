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

//! `std::fs` on the desktop; in a web page, the page's files ([`mem`]), kept between visits in the
//! browser's own storage (the owner's request of 2026-10-05: settings, missions and logs to
//! survive a reload).
//!
//! The crates call `mp_os::fs::...` where they called `std::fs::...` (web/tools/port_fs.py), and
//! `.os_exists()`, `.os_is_file()` and `.os_is_dir()` ([`FsExt`]) where they asked a path, a
//! `Metadata` or a `FileType` - std's own `Path::exists` asks the operating system, which a page
//! has not got, and a method a type has of its own cannot be taken over by a trait, so the calls
//! are renamed.

pub mod mem;

#[cfg(not(target_family = "wasm"))]
pub use std::fs::*;

#[cfg(target_family = "wasm")]
pub use mem::{
    DirEntry, File, FileType, Metadata, OpenOptions, Permissions, ReadDir, canonicalize, copy,
    create_dir, create_dir_all, exists, metadata, read, read_dir, read_to_string, remove_dir,
    remove_dir_all, remove_file, rename, set_permissions, symlink_metadata, write,
};

use std::path::Path;

/// In a page, the files the last visit kept, which storage.js read from the browser's storage
/// into `globalThis.mpStorageFiles` before the planner started: taken into the page's store, as
/// the browser has them. To be called first thing, on the page's main thread, before anything
/// reads a file. Nothing on the desktop.
pub fn preload_from_page() {
    #[cfg(target_family = "wasm")]
    {
        let global = js_sys::global();
        let Ok(files) = js_sys::Reflect::get(&global, &"mpStorageFiles".into()) else {
            return;
        };
        if !js_sys::Array::is_array(&files) {
            return;
        }
        let files = js_sys::Array::from(&files);
        mem::STORE.preload(files.iter().filter_map(|pair| {
            let pair = js_sys::Array::from(&pair);
            let path = pair.get(0).as_string()?;
            let bytes = js_sys::Uint8Array::new(&pair.get(1)).to_vec();
            Some((std::path::PathBuf::from(path), bytes))
        }));
        // The page's copy is not needed again.
        let _ = js_sys::Reflect::delete_property(&global, &"mpStorageFiles".into());
        // The logs storage.js left unread, past what a visit loads: [count, bytes, budget].
        if let Ok(left) = js_sys::Reflect::get(&global, &"mpStorageLeft".into())
            && js_sys::Array::is_array(&left)
        {
            let left = js_sys::Array::from(&left);
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let number = |at: u32| left.get(at).as_f64().unwrap_or(0.0) as u64;
            let _ = LEFT.set(LeftInPage {
                logs: number(0),
                bytes: number(1),
                budget: number(2),
            });
        }
    }
}

/// The logs the browser keeps for the page that this visit did not load, past the most of them a
/// visit loads (web/www/storage.js's `LOG_BUDGET`): they stay in the browser's storage, unread.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LeftInPage {
    /// How many.
    pub logs: u64,
    /// Their bytes.
    pub bytes: u64,
    /// The most of the logs a visit loads, in bytes.
    pub budget: u64,
}

/// What [`preload_from_page`] heard was left unread.
static LEFT: std::sync::OnceLock<LeftInPage> = std::sync::OnceLock::new();

/// The logs this visit left unread, if any - never on the desktop, where every file is read as it
/// is asked for.
#[must_use]
pub fn left_in_page() -> Option<LeftInPage> {
    LEFT.get().copied()
}

/// What is there, asked of a path - std's `Path::exists`, `is_file` and `is_dir` on the desktop,
/// the page's files in a page - or of a `Metadata` or `FileType`, under the same names, so one
/// import serves a module whatever it asks.
pub trait FsExt {
    /// `Path::exists`; for a `Metadata` or `FileType`, which describe something there, true.
    fn os_exists(&self) -> bool;
    /// `is_file`.
    fn os_is_file(&self) -> bool;
    /// `is_dir`.
    fn os_is_dir(&self) -> bool;
}

impl FsExt for Path {
    fn os_exists(&self) -> bool {
        metadata(self).is_ok()
    }

    fn os_is_file(&self) -> bool {
        metadata(self).is_ok_and(|held| held.is_file())
    }

    fn os_is_dir(&self) -> bool {
        metadata(self).is_ok_and(|held| held.is_dir())
    }
}

impl FsExt for Metadata {
    fn os_exists(&self) -> bool {
        true
    }

    fn os_is_file(&self) -> bool {
        self.is_file()
    }

    fn os_is_dir(&self) -> bool {
        self.is_dir()
    }
}

impl FsExt for FileType {
    fn os_exists(&self) -> bool {
        true
    }

    fn os_is_file(&self) -> bool {
        self.is_file()
    }

    fn os_is_dir(&self) -> bool {
        self.is_dir()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// On the desktop `mp_os::fs` is std's: a file written through it is on the disk, and the
    /// renamed questions answer as std's own.
    #[test]
    fn on_the_desktop_the_calls_are_stds() {
        let dir = std::env::temp_dir().join(format!("mp-os-fs-{}", std::process::id()));
        let file = dir.join("a.txt");
        create_dir_all(&dir).unwrap_or_default();
        write(&file, b"on disk").unwrap_or_default();
        assert_eq!(std::fs::read(&file).ok().as_deref(), Some(&b"on disk"[..]));
        assert!(file.os_exists() && file.os_is_file() && !file.os_is_dir());
        assert!(dir.os_is_dir());
        assert!(metadata(&file).is_ok_and(|held| held.os_is_file()));
        assert!(!dir.join("none").os_exists());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
