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

//! The file dialogs' list of a folder, to browse: what the folder a dialog's box names holds,
//! under the box.
//!
//! Mission Planner's `OpenFileDialog` and `SaveFileDialog` are the platform's own - a folder's
//! contents to click through, beside the box that names the file - and Mission Planner's code
//! only sets their `Filter`, and at times their folder, and reads the `FileName` chosen (Shp to
//! Poly's, `temp.cs:417-421`; Load A Log's, `Log/LogBrowse.cs:271-280`). This application draws
//! no platform dialog, and its file dialogs were a typed path alone, which could not be browsed
//! (the owner's reports of 2026-10-14: Shp to Poly's "is supposed to be a file chooser that lets
//! you browse the file system and choose a *.shp, but its not"; Review a Log's open "should be a
//! filesystem chooser"). So each draws this list under its box, standing for the platform
//! dialog's browsing, with FLIGHT PLAN's file dialog's listing ([`crate::plan::dialog_listing`])
//! and look:
//!
//! - the folder the box names, or the one the path typed in it sits in, else [`start_folder`];
//! - `..` first, then its folders, then its files of the dialog's `Filter` - the entry chosen
//!   under the list, the first as the dialogs open ([`Filter`]), each pattern matched as the
//!   platform matches one ([`matches`]) - hidden ones left out;
//! - a folder clicked is opened - the box holds its path and a separator - a file clicked is put
//!   in the box, and a file double-clicked is the dialog's OK ([`Choice`]).
//!
//! The dialogs that draw it: every typed-path box ([`crate::config::firmware::path_box`]) and the
//! flight screen's and LOGS' file questions ([`crate::fly::prompt_dialog`]), Review a Log's open
//! among them.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::RefCell;
use std::path::{Path, PathBuf};
use std::rc::Rc;

use gpui::{
    AnyElement, ClickEvent, Context, KeyDownEvent, Keystroke, Modifiers, SharedString, Window,
    div, prelude::*, px, rgb,
};
use mp_os::fs::FsExt as _;

use crate::MissionPlanner;
use crate::plan::DialogEntry;
use crate::ui::theme;

/// The most rows a list holds: a folder of logs runs to hundreds, and each row is drawn every
/// frame the dialog shows.
const LIST_MAX: usize = 1000;

/// The list's height, FLIGHT PLAN's dialog's: about thirteen rows, the rest scrolled to. Fixed,
/// so the dialog keeps its size as the folders it shows are opened.
const LIST_HEIGHT: f32 = 220.0;

/// The list's probe name, for a script's `scroll` and `reveal`.
pub const LIST: &str = "chooser-list";

/// A row's probe name before its place in the list: `chooser-file-0` is `..` where the folder has
/// one. The rows scroll in the list as the platform dialog's do, so only their prefix is let off
/// the cut-off guard (`crate::layout_guard::MAY_SCROLL`).
pub const ROW: &str = "chooser-file-";

/// A "Files of type" button's probe name before the filter entry's place.
pub const TYPE: &str = "chooser-type-";

/// The words before the filter's entries, the platform dialog's.
const FILES_OF_TYPE: &str = "Files of type:";

/// What a dialog does with a click in its list.
type OnChoice = dyn Fn(&mut MissionPlanner, Choice, &mut Window, &mut Context<MissionPlanner>);

/// One entry of a dialog's `Filter`: what its "Files of type" box says, and the patterns of the
/// files it lists.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FilterEntry {
    /// The words, `Shape file`.
    pub description: String,
    /// The patterns, `*.shp`.
    pub patterns: Vec<String>,
}

/// A dialog's `Filter`, `"Words|*.a;*.b|Words|*.c"`: its entries in order. A filter of patterns
/// alone, `"*.bin;*.apj"`, is one entry worded by them; no filter, as RAW Sensor's Save CSV sets
/// none (`Controls/RAW_Sensor.cs:280-284`), lists every file.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Filter {
    /// The entries.
    entries: Vec<FilterEntry>,
}

impl Filter {
    /// The entries of a `Filter` string. A description with no patterns after it - which .NET
    /// refuses - is taken as its own patterns, and an entry with no pattern is left out.
    #[must_use]
    pub fn parse(filter: &str) -> Self {
        let parts: Vec<&str> = filter.split('|').collect();
        let entries = if parts.len() == 1 {
            entry(filter, filter).into_iter().collect()
        } else {
            parts
                .chunks(2)
                .filter_map(|pair| match pair {
                    [description, patterns] => entry(description, patterns),
                    [description] => entry(description, description),
                    _ => None,
                })
                .collect()
        };
        Self { entries }
    }

    /// The entries, in order.
    #[must_use]
    pub fn entries(&self) -> &[FilterEntry] {
        &self.entries
    }

    /// The entry `index` lists by: itself, or the first for an index past the last - as the
    /// dialogs that set `FilterIndex = 2` on a filter of one entry show it
    /// (`Log/LogBrowse.cs:273-274`, `Log/MatLabForms.cs:47-48`).
    #[must_use]
    pub fn chosen(&self, index: usize) -> usize {
        if index < self.entries.len() { index } else { 0 }
    }

    /// The entry listed by for `index`; none for a dialog with no filter.
    #[must_use]
    pub fn entry(&self, index: usize) -> Option<&FilterEntry> {
        self.entries.get(self.chosen(index))
    }

    /// Whether a file of this name is listed under entry `index`: one of its patterns matches it,
    /// or the dialog has no filter.
    #[must_use]
    pub fn admits(&self, index: usize, name: &str) -> bool {
        self.entry(index).is_none_or(|entry| {
            entry
                .patterns
                .iter()
                .any(|pattern| matches(pattern, name))
        })
    }
}

/// An entry of `description`'s words and the `;`-separated `patterns`, worded by its patterns
/// when it has no words; none when it has no pattern.
fn entry(description: &str, patterns: &str) -> Option<FilterEntry> {
    let patterns: Vec<String> = patterns
        .split(';')
        .map(str::trim)
        .filter(|pattern| !pattern.is_empty())
        .map(ToOwned::to_owned)
        .collect();
    if patterns.is_empty() {
        return None;
    }
    let description = match description.trim() {
        "" => patterns.join(";"),
        words => words.to_owned(),
    };
    Some(FilterEntry {
        description,
        patterns,
    })
}

/// Whether a file name matches a filter's pattern as the platform dialog matches one: `*` any
/// run of characters, `?` any one, a letter of either case alike; and a pattern ending `.*` also
/// matches the name without that part, so `*.*` is every file, a dot in its name or not.
#[must_use]
pub fn matches(pattern: &str, name: &str) -> bool {
    let pattern: Vec<char> = pattern.trim().to_lowercase().chars().collect();
    let name: Vec<char> = name.to_lowercase().chars().collect();
    wild(&pattern, &name)
        || pattern
            .strip_suffix(&['.', '*'])
            .is_some_and(|stem| wild(stem, &name))
}

/// `*` and `?` over characters, the last `*` taking one more character each time what follows
/// it fails.
fn wild(pattern: &[char], name: &[char]) -> bool {
    let (mut at, mut of) = (0, 0);
    // The last `*` met, and where in the name what follows it was last tried.
    let mut star: Option<(usize, usize)> = None;
    while of < name.len() {
        match pattern.get(at) {
            Some('*') => {
                star = Some((at, of));
                at += 1;
            }
            Some(&wanted) if wanted == '?' || name.get(of) == Some(&wanted) => {
                at += 1;
                of += 1;
            }
            _ => match star {
                Some((star_at, tried)) => {
                    at = star_at + 1;
                    of = tried + 1;
                    star = Some((star_at, tried + 1));
                }
                None => return false,
            },
        }
    }
    pattern
        .get(at..)
        .is_some_and(|rest| rest.iter().all(|wanted| *wanted == '*'))
}

/// The folder a dialog lists while its box names none: the one a relative path typed in the box
/// names a file in - the folder the planner was started in, as every dialog's handler reads a
/// relative path - or, in a page, which has none, the data folder, where the browser's picker
/// puts the files it gives (`page_files.rs`).
#[must_use]
pub fn start_folder() -> PathBuf {
    std::env::current_dir()
        .ok()
        .or_else(mp_settings::data_directory)
        .unwrap_or_default()
}

/// What a dialog's list shows: the folder, its rows, and the filter it lists by.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Chooser {
    /// The dialog's probe name, for the facts.
    pub id: &'static str,
    /// What the dialog's box holds.
    pub typed: String,
    /// The dialog's filter.
    pub filter: Filter,
    /// Its entry listed by, `FilterIndex` less one.
    pub index: usize,
    /// The folder listed.
    pub folder: PathBuf,
    /// Its rows: `..`, the folders, the files.
    pub entries: Vec<DialogEntry>,
}

impl Chooser {
    /// The list of the dialog `id` whose box holds `typed`, its `filter` listed by entry `index`:
    /// a relative path is taken from [`start_folder`].
    #[must_use]
    pub fn new(id: &'static str, typed: &str, filter: &str, index: usize) -> Self {
        Self::within(id, typed, &start_folder(), filter, index)
    }

    /// [`Chooser::new`], a relative path taken from `directory`.
    #[must_use]
    pub fn within(
        id: &'static str,
        typed: &str,
        directory: &Path,
        filter: &str,
        index: usize,
    ) -> Self {
        let filter = Filter::parse(filter);
        // The folder, `..` and its folders, as FLIGHT PLAN's dialog finds them: no extension asks
        // it for no file. The files follow, by the filter's patterns.
        let (folder, mut entries) = crate::plan::dialog_listing(typed, directory, &[]);
        let mut files: Vec<DialogEntry> = mp_os::fs::read_dir(&folder)
            .into_iter()
            .flatten()
            .flatten()
            .filter_map(|entry| {
                let name = entry.file_name().to_string_lossy().into_owned();
                let path = entry.path();
                let listed =
                    !name.starts_with('.') && !path.os_is_dir() && filter.admits(index, &name);
                listed.then_some(DialogEntry {
                    name,
                    path,
                    folder: false,
                })
            })
            .collect();
        files.sort_by_cached_key(|file| file.name.to_lowercase());
        entries.extend(files);
        entries.truncate(LIST_MAX);
        Self {
            id,
            typed: typed.to_owned(),
            filter,
            index,
            folder,
            entries,
        }
    }

    /// Each row as it reads: a folder's name and a `/`, `../` first; a file's name.
    #[must_use]
    pub fn rows(&self) -> Vec<String> {
        self.entries.iter().map(row_text).collect()
    }
}

/// A row as it reads.
fn row_text(entry: &DialogEntry) -> String {
    if entry.folder {
        format!("{}/", entry.name)
    } else {
        entry.name.clone()
    }
}

/// What a row puts in the dialog's box: a folder's path with a separator after it, so the list
/// opens it; a file's path, whichever folder the dialog opened in.
#[must_use]
pub fn answer(entry: &DialogEntry) -> String {
    let path = entry.path.display().to_string();
    if entry.folder && !path.ends_with(['/', '\\']) {
        format!("{path}{}", std::path::MAIN_SEPARATOR)
    } else {
        path
    }
}

/// What a click in a dialog's list asks of the dialog.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Choice {
    /// A row: the box to hold `text`, and for a file double-clicked the dialog's OK.
    Row {
        /// What the box is to hold ([`answer`]).
        text: String,
        /// The dialog's OK as well.
        open: bool,
    },
    /// The filter's entry to list by, as the platform dialog's "Files of type" box picks one.
    Type(usize),
}

thread_local! {
    /// What the last row clicked put in a box, for the click after it.
    static PICKED: RefCell<Option<String>> = const { RefCell::new(None) };
}

/// A row of `text` clicked, the click `clicks` of a run: whether it is the dialog's OK - a file's
/// second click, its first having picked the same file. A double click on a folder opens the
/// folder on its first click, and its second lands on whatever row is under the pointer in the
/// folder opened: that row is picked, never opened.
#[must_use]
pub fn opens(text: &str, folder: bool, clicks: usize) -> bool {
    let before = PICKED.with(|picked| picked.replace(Some(text.to_owned())));
    !folder && clicks >= 2 && before.as_deref() == Some(text)
}

/// The keys that put `text` in a box in place of what it holds, for a dialog whose box only its
/// owner can change ([`crate::config::firmware::path_box`]): Ctrl+U, which empties a box
/// ([`crate::textfield::TextField::key`]), then `text` as one keystroke's characters. The owner's
/// key handler takes a row's path as it takes typing.
#[must_use]
pub fn typing(text: &str) -> [KeyDownEvent; 2] {
    let key = |modifiers: Modifiers, key: &str, key_char: Option<&str>| KeyDownEvent {
        keystroke: Keystroke {
            modifiers,
            key: key.to_owned(),
            key_char: key_char.map(ToOwned::to_owned),
        },
        is_held: false,
        prefer_character_input: false,
    };
    [
        key(Modifiers::control(), "u", None),
        key(Modifiers::none(), "", Some(text)),
    ]
}

thread_local! {
    /// The facts the list drawn last recorded, to tell when they change.
    static RECORDED: RefCell<String> = const { RefCell::new(String::new()) };
}

/// Publishes the list drawn last, for a script: the dialog's probe name, the folder, the rows as
/// they read, comma-separated, and the filter's entry listed by - `none` for a dialog with no
/// filter.
///
/// They are recorded as the list is drawn, after the frame's facts have gone out, so a list that
/// changed asks for one more frame to take them, as a control moved does (`probe.rs`), rather
/// than waiting on the next repaint.
pub fn record_facts(chooser: &Chooser) {
    if !crate::facts::enabled() {
        return;
    }
    let folder = chooser.folder.display().to_string();
    let files = chooser.rows().join(",");
    let kind = chooser
        .filter
        .entry(chooser.index)
        .map_or("none", |entry| entry.description.as_str());
    crate::facts::record("chooser.id", chooser.id);
    crate::facts::record("chooser.folder", &folder);
    crate::facts::record("chooser.files", &files);
    crate::facts::record("chooser.type", kind);
    let now = format!("{}|{folder}|{files}|{kind}", chooser.id);
    if RECORDED.with(|recorded| recorded.replace(now.clone()) != now) {
        crate::repaint::again_in(std::time::Duration::ZERO);
    }
}

/// The list under a dialog's box: the folder listed, with `browse` beside it - the browser's
/// picker, in a page - its rows in a box of their own that scrolls, and the filter's entries to
/// list by. A click is handed to `on_choice`.
pub fn view(
    chooser: &Chooser,
    browse: Option<AnyElement>,
    on_choice: impl Fn(&mut MissionPlanner, Choice, &mut Window, &mut Context<MissionPlanner>)
    + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    record_facts(chooser);
    let on_choice: Rc<OnChoice> = Rc::new(on_choice);
    let heading = div()
        .flex()
        .items_center()
        .justify_between()
        .gap_2()
        .child(
            div()
                .min_w(px(0.0))
                .overflow_hidden()
                .whitespace_nowrap()
                .text_ellipsis()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(format!("in {}", chooser.folder.display())),
        )
        .children(browse);
    // Measured by its scrolling box - the rows in view, not the union of every row, which would
    // run past the dialog - as the log browser's field list is, so a script can wheel it. The box
    // has its own name for each folder, so a folder opened shows from its top rather than where
    // the last one was scrolled to.
    let list = crate::probe::measured(LIST, div()).flex().flex_col().child(
        div()
            .id(SharedString::from(format!(
                "{LIST}:{}",
                chooser.folder.display()
            )))
            .flex()
            .flex_col()
            .h(px(LIST_HEIGHT))
            .overflow_y_scroll()
            .bg(rgb(theme::BG))
            .border_1()
            .border_color(rgb(theme::BORDER))
            .rounded_sm()
            .children(rows(chooser, &on_choice, cx)),
    );
    div()
        .flex()
        .flex_col()
        .gap_1()
        .child(heading)
        .child(list)
        .children(types(chooser, &on_choice, cx))
        .into_any_element()
}

/// The rows: a folder's in the accent colour, the one the box holds marked; each click handed to
/// `on_choice` with what the row puts in the box, and whether it is the dialog's OK ([`opens`]).
fn rows(
    chooser: &Chooser,
    on_choice: &Rc<OnChoice>,
    cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    let typed = chooser.typed.trim();
    chooser
        .entries
        .iter()
        .enumerate()
        .map(|(index, entry)| {
            let text = answer(entry);
            let folder = entry.folder;
            let held = text == typed;
            let on_choice = Rc::clone(on_choice);
            crate::probe::measured(format!("{ROW}{index}"), div())
                .id(("chooser-file", index))
                .flex_shrink_0()
                .px_2()
                .text_xs()
                .whitespace_nowrap()
                .overflow_hidden()
                .text_ellipsis()
                .cursor_pointer()
                .bg(rgb(if held { theme::SELECTION } else { theme::BG }))
                .text_color(rgb(if folder { theme::ACCENT } else { theme::TEXT }))
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(row_text(entry))
                .on_click(cx.listener(move |this, event: &ClickEvent, window, cx| {
                    let open = opens(&text, folder, event.click_count());
                    let choice = Choice::Row {
                        text: text.clone(),
                        open,
                    };
                    on_choice(this, choice, window, cx);
                    cx.notify();
                }))
                .into_any_element()
        })
        .collect()
}

/// The filter's entries to list by, the one listed by marked, after the platform dialog's words;
/// nothing for a dialog with no filter.
fn types(
    chooser: &Chooser,
    on_choice: &Rc<OnChoice>,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let entries = chooser.filter.entries();
    if entries.is_empty() {
        return None;
    }
    let chosen = chooser.filter.chosen(chooser.index);
    let buttons: Vec<AnyElement> = entries
        .iter()
        .enumerate()
        .map(|(index, entry)| {
            let name = format!("{TYPE}{index}");
            let on_choice = Rc::clone(on_choice);
            let picked = index == chosen;
            crate::probe::measured(name.clone(), div())
                .id(SharedString::from(name))
                .px_2()
                .py(px(1.0))
                .rounded_sm()
                .border_1()
                .border_color(rgb(if picked { theme::ACCENT } else { theme::BORDER }))
                .text_xs()
                .text_color(rgb(if picked { theme::ACCENT } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(entry.description.clone())
                .on_click(cx.listener(move |this, _event: &ClickEvent, window, cx| {
                    on_choice(this, Choice::Type(index), window, cx);
                    cx.notify();
                }))
                .into_any_element()
        })
        .collect();
    Some(
        div()
            .flex()
            .flex_wrap()
            .items_center()
            .gap_1()
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(FILES_OF_TYPE),
            )
            .children(buttons)
            .into_any_element(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::textfield::{KeyOutcome, TextField};

    /// Each entry of a filter as `words=patterns`.
    fn described(filter: &str) -> Vec<String> {
        Filter::parse(filter)
            .entries()
            .iter()
            .map(|entry| format!("{}={}", entry.description, entry.patterns.join(";")))
            .collect()
    }

    /// A filter's entries as the dialogs write them: words and patterns in pairs, patterns
    /// alone, and none.
    #[test]
    fn a_filter_is_its_entries() {
        assert_eq!(described("Shape file|*.shp"), ["Shape file=*.shp"]);
        assert_eq!(
            described(crate::config::firmware::MANIFEST_FILTER),
            [
                "Firmware (*.hex;*.px4;*.vrx;*.apj)=*.hex;*.px4;*.vrx;*.apj",
                "DFU=*.hex;*.bin;*.dfu",
                "All files (*.*)=*.*",
            ]
        );
        assert_eq!(
            described("*.log;*.bin|*.log;*.bin;*.BIN;*.LOG"),
            ["*.log;*.bin=*.log;*.bin;*.BIN;*.LOG"]
        );
        // Patterns alone, as DroneCAN's firmware dialog has them.
        assert_eq!(described("*.bin;*.apj"), ["*.bin;*.apj=*.bin;*.apj"]);
        assert!(described("").is_empty());
        // A description left without its patterns is taken as its own.
        assert_eq!(
            described("tlog|*.tlog|*.rlog"),
            ["tlog=*.tlog", "*.rlog=*.rlog"]
        );
    }

    /// The entry listed by: the one chosen, the first for one past the last, every file for no
    /// filter.
    #[test]
    fn a_filter_lists_by_the_entry_chosen() {
        let filter = Filter::parse("Python Files|*.py|All files|*.*");
        assert!(filter.admits(0, "script.py"));
        assert!(!filter.admits(0, "notes.txt"));
        assert!(filter.admits(1, "notes.txt"));
        assert!(filter.admits(1, "README"));
        // `FilterIndex = 2` on one entry, as LogBrowse sets it: the first.
        let logs = Filter::parse("Log Files|*.log;*.bin;*.BIN;*.LOG");
        assert_eq!(logs.chosen(1), 0);
        assert!(logs.admits(1, "00000001.BIN"));
        assert!(!logs.admits(1, "flight.tlog"));
        assert_eq!(
            logs.entry(5).map(|entry| entry.description.as_str()),
            Some("Log Files")
        );
        let none = Filter::parse("");
        assert!(none.admits(0, "anything.csv"));
        assert!(none.entry(0).is_none());
    }

    /// The patterns, as the platform dialog matches them.
    #[test]
    fn a_pattern_matches_as_the_platforms() {
        assert!(matches("*.shp", "field.shp"));
        assert!(matches("*.shp", "FIELD.SHP"), "either case");
        assert!(matches("*.BIN", "00000012.bin"));
        assert!(!matches("*.shp", "field.prj"));
        assert!(!matches("*.shp", "field.shp.bak"));
        assert!(matches("*.*", "notes.txt"));
        assert!(matches("*.*", "README"), "*.* is every file");
        assert!(matches("*", "README"));
        assert!(matches("*.tlog.*", "flight.tlog.zip"));
        assert!(matches("*.tlog.*", "flight.tlog"));
        assert!(!matches("*.tlog", "flight.tlog.zip"));
        assert!(matches("log?.bin", "log1.bin"));
        assert!(!matches("log?.bin", "log12.bin"));
        assert!(matches("*a*b*", "xxaxxbxx"));
        assert!(!matches("*a*b", "xxbxxa"));
        assert!(!matches("", "x"));
    }

    /// A folder of the test's own, emptied first.
    fn scratch(name: &str) -> PathBuf {
        let folder = mp_os::temp_dir().join(format!("mp-chooser-{name}-{}", mp_os::process_id()));
        let _ = mp_os::fs::remove_dir_all(&folder);
        mp_os::fs::create_dir_all(&folder).unwrap();
        folder
    }

    /// The list: `..`, the folders, then the files of the filter's entry, each by name whatever
    /// its case, hidden ones left out; every file for `*.*` and for no filter.
    #[test]
    fn a_dialog_lists_its_folder_by_its_filter() {
        let folder = scratch("list");
        for sub in ["sub", ".hidden"] {
            mp_os::fs::create_dir_all(folder.join(sub)).unwrap();
        }
        for file in ["b.SHP", "a.shp", "a.prj", ".x.shp", "README"] {
            mp_os::fs::write(folder.join(file), "").unwrap();
        }
        let shp = Chooser::within("test", "", &folder, "Shape file|*.shp", 0);
        assert_eq!(shp.folder, folder);
        assert_eq!(shp.rows(), ["../", "sub/", "a.shp", "b.SHP"]);
        assert_eq!(
            shp.entries
                .iter()
                .map(|entry| entry.folder)
                .collect::<Vec<_>>(),
            [true, true, false, false]
        );
        let all = Chooser::within("test", "", &folder, "Shape file|*.shp|All files|*.*", 1);
        assert_eq!(
            all.rows(),
            ["../", "sub/", "a.prj", "a.shp", "b.SHP", "README"]
        );
        let none = Chooser::within("test", "", &folder, "", 0);
        assert_eq!(none.rows(), all.rows());
        mp_os::fs::remove_dir_all(&folder).unwrap();
    }

    /// A folder's row opens it, `..` goes back up, and a path typed lists the folder it names or
    /// sits in; a file's row is its path.
    #[test]
    fn a_folders_row_opens_it_and_a_files_row_names_it() {
        let folder = scratch("open");
        mp_os::fs::create_dir_all(folder.join("sub")).unwrap();
        mp_os::fs::write(folder.join("sub").join("deep.shp"), "").unwrap();
        mp_os::fs::write(folder.join("top.shp"), "").unwrap();
        let top = Chooser::within("test", "", &folder, "Shape file|*.shp", 0);
        assert_eq!(top.rows(), ["../", "sub/", "top.shp"]);
        let opened = answer(&top.entries[1]);
        assert_eq!(
            opened,
            format!(
                "{}{}",
                folder.join("sub").display(),
                std::path::MAIN_SEPARATOR
            )
        );
        let inside = Chooser::within("test", &opened, &folder, "Shape file|*.shp", 0);
        assert_eq!(inside.folder, folder.join("sub"));
        assert_eq!(inside.rows(), ["../", "deep.shp"]);
        let up = answer(&inside.entries[0]);
        let back = Chooser::within("test", &up, &folder, "Shape file|*.shp", 0);
        assert_eq!(back.folder, folder);
        assert_eq!(back.rows(), top.rows());
        // A file's row is its path, wherever the dialog opened.
        assert_eq!(
            answer(&top.entries[2]),
            folder.join("top.shp").display().to_string()
        );
        // A file typed, there or not yet, lists its own folder.
        let typed = folder.join("sub").join("new.shp").display().to_string();
        let beside = Chooser::within("test", &typed, &folder, "Shape file|*.shp", 0);
        assert_eq!(beside.folder, folder.join("sub"));
        // A relative path is taken from the folder given.
        let relative = Chooser::within("test", "sub/", &folder, "Shape file|*.shp", 0);
        assert_eq!(relative.folder, folder.join("sub"));
        mp_os::fs::remove_dir_all(&folder).unwrap();
    }

    /// The root has no `..`, and a row of it does not double its separator.
    #[test]
    fn the_root_has_no_way_up() {
        let root = PathBuf::from(std::path::MAIN_SEPARATOR.to_string());
        let listed = Chooser::within("test", &root.display().to_string(), &root, "", 0);
        assert!(
            listed.entries.iter().all(|entry| entry.name != ".."),
            "{:?}",
            listed.rows()
        );
        let entry = DialogEntry {
            name: "..".to_owned(),
            path: root.clone(),
            folder: true,
        };
        assert_eq!(answer(&entry), root.display().to_string());
    }

    /// A row's keys put its path in a box in place of what it held, whether the box was ever
    /// drawn with its caret or not, as typing would: each a change the owner hears of.
    #[test]
    fn a_rows_keys_replace_what_a_box_holds() {
        for drawn in [false, true] {
            let mut field = TextField::new("");
            field.set("/somewhere/else/old.shp");
            if drawn {
                field.show_caret();
            }
            let outcomes: Vec<KeyOutcome> = typing("/home/me/field.shp")
                .iter()
                .map(|event| field.key(event))
                .collect();
            assert_eq!(field.value(), "/home/me/field.shp", "drawn: {drawn}");
            assert_eq!(outcomes, [KeyOutcome::Changed, KeyOutcome::Changed]);
        }
    }

    /// A double click opens a file only when its first click picked that file: a folder's first
    /// click opens the folder, and the second lands on another row.
    #[test]
    fn a_double_click_opens_the_file_its_first_click_picked() {
        assert!(!opens("/logs/a.bin", false, 1));
        assert!(opens("/logs/a.bin", false, 2));
        assert!(!opens("/logs/sub/", true, 1));
        assert!(
            !opens("/logs/sub/b.bin", false, 2),
            "the first click was the folder's"
        );
        assert!(!opens("/logs/sub/", true, 1));
        assert!(!opens("/logs/sub/", true, 2), "a folder never opens as a file");
        assert!(!opens("/logs/sub/b.bin", false, 1));
        assert!(opens("/logs/sub/b.bin", false, 2));
    }
}
