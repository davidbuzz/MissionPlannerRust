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

//! The computer's files in a web page: the browser's own file picker for a file box that opens a
//! file, and a download for a file a box saved (the browser build's storage row: "the browser's
//! file picker and downloads for Load and Save").
//!
//! A page's files are the planner's own, held in memory and kept in the browser's storage
//! (crates/mp-os/src/fs/mem.rs, web/www/storage.js), and a file box lists only those: the
//! computer's files are out of a page's reach, and what the planner saves stays in the browser.
//! So in a page:
//!
//! - a file box that opens a file shows "From this computer..." ([`browse_button`]): the
//!   browser's picker, filtered to the box's file types. The file chosen is put in the folder the
//!   box shows - so it is listed there from then on, and kept between visits - and its path is
//!   typed into the box and Enter pressed, as a double click on a listed file takes it
//!   ([`deliver`]). Every file box takes a typed path and Enter, so one way serves them all;
//! - a file a box saved is handed to the browser as a download as well ([`saved`]), under its
//!   name.
//!
//! web/www/files.js is the page's half. On the desktop there is nothing to add: a box reaches
//! every file, and the button is not shown.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::RefCell;
use std::path::{Path, PathBuf};

use gpui::{div, prelude::*, px, rgb};

use crate::ui::theme;

/// What the browser's picker filters to: the box's file types as an `<input type="file">`'s
/// `accept` takes them, `.waypoints,.txt`. None: any file.
#[must_use]
pub fn accept(types: &[&str]) -> String {
    types
        .iter()
        .map(|kind| format!(".{kind}"))
        .collect::<Vec<_>>()
        .join(",")
}

/// What the browser's picker filters to for a file dialog's `Filter`, `"Firmware|*.apj;*.px4"` or
/// `"*.bin;*.apj"`: each `*.ext` in it. None - any file - when it has `*.*` or no pattern.
#[must_use]
pub fn accept_filter(filter: &str) -> String {
    let mut kinds = Vec::new();
    for pattern in filter.split(['|', ';', ',', ' ']) {
        let Some(kind) = pattern.trim().strip_prefix("*.") else {
            continue;
        };
        if kind.is_empty() || kind.contains(['*', '?']) {
            return String::new();
        }
        if !kinds.contains(&kind) {
            kinds.push(kind);
        }
    }
    accept(&kinds)
}

/// Where a file chosen in the browser's picker goes: `folder`, under the file's own name - its
/// last part only, so a name cannot place it anywhere else. None for a name that is no file's.
#[must_use]
#[cfg_attr(not(target_family = "wasm"), allow(dead_code))]
pub fn destination(folder: &Path, name: &str) -> Option<PathBuf> {
    let leaf = name.rsplit(['/', '\\']).next()?.trim();
    if leaf.is_empty() || leaf == "." || leaf == ".." {
        return None;
    }
    Some(folder.join(leaf))
}

/// The folder a file box shows for what is typed in it: the typed path if it is a folder, else
/// the folder it is in, else `fallback` - the folder the box opened in.
#[must_use]
pub fn folder_of(typed: &str, fallback: &Path) -> PathBuf {
    use mp_os::fs::FsExt as _;
    let typed = typed.trim();
    if typed.is_empty() {
        return fallback.to_path_buf();
    }
    let path = Path::new(typed);
    let path = if path.is_absolute() {
        path.to_path_buf()
    } else {
        fallback.join(path)
    };
    if path.os_is_dir() {
        return path;
    }
    path.parent()
        .filter(|parent| parent.os_is_dir())
        .map_or_else(|| fallback.to_path_buf(), Path::to_path_buf)
}

/// Whether the page offers the browser's picker: in a page whose script provides it.
#[must_use]
pub fn available() -> bool {
    page::available()
}

/// "From this computer...", for a file box that opens a file: the browser's picker filtered to
/// `accept` ([`accept`], [`accept_filter`]), its file put in `folder`. Nothing on the desktop. The press keeps the keyboard where
/// it is - in the box - for the path typed into it once the file is chosen.
#[must_use]
pub fn browse_button(
    id: impl Into<gpui::SharedString>,
    accept: String,
    folder: PathBuf,
) -> Option<gpui::AnyElement> {
    if !available() {
        return None;
    }
    let id = id.into();
    Some(
        crate::probe::measured(id.to_string(), div())
            .id(id)
            .px_2()
            .py_1()
            .rounded_sm()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .text_xs()
            .text_color(rgb(theme::ACCENT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .min_w(px(0.0))
            .child(BROWSE)
            .on_mouse_down(gpui::MouseButton::Left, |_event, window, _cx| {
                window.prevent_default();
            })
            .on_click(move |_event, _window, _cx| pick(&accept, &folder))
            .into_any_element(),
    )
}

/// The button's text.
pub const BROWSE: &str = "From this computer...";

thread_local! {
    /// Where the file the picker gives goes: the folder of the box that opened it.
    static FOLDER: RefCell<Option<PathBuf>> = const { RefCell::new(None) };
    /// The file the picker gave, once it is in its folder, for [`deliver`].
    static PLACED: RefCell<Option<PathBuf>> = const { RefCell::new(None) };
}

/// Opens the browser's picker, filtered to `accept`, for a file to go into `folder`.
pub fn pick(accept: &str, folder: &Path) {
    FOLDER.with(|held| *held.borrow_mut() = Some(folder.to_path_buf()));
    page::open_picker(accept);
}

/// The file the browser's picker gave: put in the folder of the box that opened the picker,
/// under its name, and the frame that types its path into the box asked for. Its path, or why it
/// could not be kept; nothing when no picker was opened.
///
/// # Errors
/// A name that is no file's, or the write's.
#[cfg_attr(not(target_family = "wasm"), allow(dead_code))]
pub fn place(name: &str, bytes: &[u8]) -> std::io::Result<Option<PathBuf>> {
    let Some(folder) = FOLDER.with(|held| held.borrow().clone()) else {
        return Ok(None);
    };
    let path = destination(&folder, name).ok_or_else(|| {
        std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            format!("{name:?} is no file's name"),
        )
    })?;
    mp_os::fs::write(&path, bytes)?;
    PLACED.with(|placed| *placed.borrow_mut() = Some(path.clone()));
    crate::repaint::again_in(std::time::Duration::ZERO);
    Ok(Some(path))
}

/// Hands the file at `path`, which a box has just saved, to the browser as a download, in a page;
/// nothing on the desktop.
pub fn saved(path: &Path) {
    page::download(path);
}

/// Called each frame: once the browser's picker has given a file and it is in its folder, its
/// path typed into the box that holds the keyboard and Enter pressed, after this frame - the box
/// takes them as its own keys. What the box held is replaced: Ctrl+A, then the path pasted.
pub fn deliver(window: &mut gpui::Window, cx: &mut gpui::App) {
    let Some(path) = PLACED.with(|placed| placed.borrow_mut().take()) else {
        return;
    };
    window.defer(cx, move |window, cx| {
        crate::textfield::type_into_focused(&path.display().to_string(), window, cx);
    });
}

#[cfg(target_family = "wasm")]
mod page {
    use std::path::Path;

    use wasm_bindgen::JsCast as _;
    use wasm_bindgen::prelude::wasm_bindgen;

    /// One of the functions web/www/files.js gives the page.
    fn function(name: &str) -> Option<js_sys::Function> {
        js_sys::Reflect::get(&js_sys::global(), &name.into())
            .ok()?
            .dyn_into()
            .ok()
    }

    pub(super) fn available() -> bool {
        function("mprPickFile").is_some()
    }

    pub(super) fn open_picker(accept: &str) {
        let Some(open) = function("mprPickFile") else {
            return;
        };
        if let Err(err) = open.call1(&wasm_bindgen::JsValue::NULL, &accept.into()) {
            log::warn!("the browser's file picker: {err:?}");
        }
    }

    pub(super) fn download(path: &Path) {
        let Some(offer) = function("mprDownload") else {
            return;
        };
        let Ok(bytes) = mp_os::fs::read(path) else {
            return;
        };
        let name = path
            .file_name()
            .map_or_else(String::new, |name| name.to_string_lossy().into_owned());
        // Copied out of the planner's memory, which a Blob may not be made from (it is shared
        // between the planner's threads).
        let bytes = js_sys::Uint8Array::from(bytes.as_slice());
        if let Err(err) = offer.call2(&wasm_bindgen::JsValue::NULL, &name.into(), &bytes) {
            log::warn!("the browser's download: {err:?}");
        }
    }

    /// The page (files.js): the file chosen in the browser's picker, its name and bytes
    /// ([`super::place`]).
    #[wasm_bindgen]
    #[allow(unreachable_pub)]
    pub fn planner_file_picked(name: &str, bytes: &[u8]) {
        if let Err(err) = super::place(name, bytes) {
            log::warn!("the file chosen, {name}: {err}");
        }
    }
}

#[cfg(not(target_family = "wasm"))]
mod page {
    use std::path::Path;

    pub(super) const fn available() -> bool {
        false
    }

    pub(super) const fn open_picker(_accept: &str) {}

    pub(super) const fn download(_path: &Path) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_picker_is_filtered_to_the_boxs_types() {
        assert_eq!(accept(&["waypoints", "txt"]), ".waypoints,.txt");
        assert_eq!(accept(&[]), "");
    }

    #[test]
    fn a_dialogs_filter_is_the_pickers_too() {
        assert_eq!(accept_filter("Firmware|*.apj;*.px4"), ".apj,.px4");
        assert_eq!(accept_filter("*.bin;*.apj"), ".bin,.apj");
        assert_eq!(
            accept_filter("Log Files|*.log;*.bin|Bin Files|*.bin"),
            ".log,.bin"
        );
        assert_eq!(accept_filter("All files|*.*"), "");
        assert_eq!(accept_filter(""), "");
    }

    #[test]
    fn a_chosen_file_goes_into_the_boxs_folder_under_its_own_name() {
        let folder = Path::new("/home/web/missions");
        assert_eq!(
            destination(folder, "survey.plan"),
            Some(folder.join("survey.plan"))
        );
        // A name with a path in it keeps only its last part.
        assert_eq!(
            destination(folder, "../../etc/passwd"),
            Some(folder.join("passwd"))
        );
        assert_eq!(
            destination(folder, "C:\\Users\\me\\a.fen"),
            Some(folder.join("a.fen"))
        );
        for nothing in ["", "..", ".", "dir/"] {
            assert_eq!(destination(folder, nothing), None, "{nothing:?}");
        }
    }

    #[test]
    fn the_folder_a_box_shows_follows_what_is_typed() {
        let dir = std::env::temp_dir().join(format!("page-files-{}", std::process::id()));
        let sub = dir.join("sub");
        mp_os::fs::create_dir_all(&sub).unwrap();
        assert_eq!(folder_of("", &dir), dir);
        assert_eq!(folder_of("sub", &dir), sub);
        assert_eq!(folder_of(&sub.display().to_string(), &dir), sub);
        assert_eq!(folder_of("sub/mission.waypoints", &dir), sub);
        assert_eq!(folder_of("nowhere/mission.waypoints", &dir), dir);
        mp_os::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_chosen_file_is_kept_in_the_folder_of_the_box_that_asked() {
        // No picker opened: nothing is placed.
        FOLDER.with(|held| *held.borrow_mut() = None);
        assert!(place("a.plan", b"{}").unwrap().is_none());
        let dir = std::env::temp_dir().join(format!("page-files-place-{}", std::process::id()));
        mp_os::fs::create_dir_all(&dir).unwrap();
        pick(".plan", &dir);
        let path = place("survey.plan", b"{ }").unwrap().unwrap();
        assert_eq!(path, dir.join("survey.plan"));
        assert_eq!(mp_os::fs::read(&path).unwrap(), b"{ }");
        assert_eq!(
            PLACED.with(|placed| placed.borrow_mut().take()),
            Some(path),
            "the path is waiting to be typed into the box"
        );
        assert!(place("..", b"").is_err());
        mp_os::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn the_desktop_shows_no_button_and_downloads_nothing() {
        assert!(!available());
        assert!(browse_button("x", accept(&["txt"]), PathBuf::from("/")).is_none());
        saved(Path::new("/nonexistent"));
    }
}
