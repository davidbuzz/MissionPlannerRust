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

//! Ctrl+X's map cache window: `Controls/GMAPCache.cs`, which `MainV2.ProcessCmdKey` opens with
//! `new GMAPCache().ShowUserControl()` (`MainV2.cs:4139-4143`) - the control in a form of its own
//! size, captioned with its `Text`, which nothing sets.
//!
//! What it shows: `myDataGridView1`, read only and filling the control (675 x 534), one row for
//! each directory under the tile cache's `TileDBv3/en` - a map provider's tiles - and a last row,
//! "Total", for the whole of it: the name, the size in megabytes rounded half away from zero with
//! "MB" after it, and how many files; then two button columns with no header, "Remove 30+days"
//! and "Remove All" (`Activate`, `GMAPCache.cs:29-93`; `DirSize`, `:125-146`).
//!
//! What it does: a button cell pressed (`myDataGridView1_CellClick`, `:95-123`) takes the row's
//! name as a provider - `GMapProviders.List.First(a => a.Name == dir)`, which throws for a name
//! that is none, "Total" among them - and deletes every `.jpg` and `.png` under that provider's
//! directory created before thirty days ago, or before now (`MyImageCache.DeleteOlderThan`,
//! `ExtLibs/Maps/MyImageCache.cs:132-184`), says "Removed N images" in a box, and fills the grid
//! again once the box is closed.
//!
//! Where this is not the C#, each at its site:
//!
//! * the providers are this application's and the C#'s: [`mp_tiles::SOURCES`] (its
//!   `GMapProviders.List`, as `source_by_name` reads it) and [`mp_tiles::CSHARP_LIST`], so the
//!   tiles of either can be removed from a shared cache;
//! * the throw for a name that is no provider - an unhandled exception, the C#'s error box - is
//!   said on the status line, by the owner's ruling of 2026-09-25;
//! * a tile root that is not there reads as empty: the C# makes it when the map starts
//!   (`MyImageCache.CacheLocation`, `:46-57`), which this application leaves to its first tile;
//!   the providers are listed by name, as Windows lists a directory's; a directory or file that
//!   cannot be read counts as nothing, where the C#'s `GetFiles` throws;
//! * a file's creation time is its modification time where the file system keeps none;
//! * the form is drawn over the window, modal, as the other windows here are; a second Ctrl+X
//!   opens a fresh one in its place;
//! * the colours are this application's.
//!
//! `// C#: Controls/GMAPCache.cs:1-150; Controls/GMAPCache.Designer.cs:29-62`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::fs::FsExt as _;
use std::path::{Path, PathBuf};
use web_time::{Duration, SystemTime};

use gpui::{AnyElement, Context, SharedString, Window, div, prelude::*, px, rgb};

use crate::MissionPlanner;
use crate::config::servo_output::Message;
use crate::ui::theme;

/// The control's `Size`, which `ShowUserControl` gives its form.
/// `// C#: Controls/GMAPCache.Designer.cs:54; Utilities/ExtensionsMP.cs:110-131`
pub const SIZE: (f32, f32) = (675.0, 534.0);

/// The form's caption: the control's `Text`, which its Designer leaves empty.
/// `// C#: Utilities/ExtensionsMP.cs:114`
pub const FORM_TEXT: &str = "";

/// The columns `AutoGenerateColumns` makes of the anonymous type's properties, in their order.
/// `// C#: Controls/GMAPCache.cs:36-41, 49-54`
pub const COLUMNS: [&str; 3] = ["Name", "Size", "Count"];

/// The two button columns' `Text`; their `HeaderText` is empty.
/// `// C#: Controls/GMAPCache.cs:69-89`
pub const REMOVE_OLD: &str = "Remove 30+days";
pub const REMOVE_ALL: &str = "Remove All";

/// The last row's name. `// C#: Controls/GMAPCache.cs:38`
pub const TOTAL: &str = "Total";

/// `DateTime.Now.AddDays(-30)`. `// C#: Controls/GMAPCache.cs:104`
pub const THIRTY_DAYS: Duration = Duration::from_secs(30 * 24 * 60 * 60);

/// `Enumerable.First`'s `InvalidOperationException` when no provider has the row's name.
/// `// C#: Controls/GMAPCache.cs:105, 117`
pub const NO_MATCH: &str = "Sequence contains no matching element";

/// The two kinds of `.jpg` and `.png` `DeleteOlderThan` looks for, in its order.
/// `// C#: ExtLibs/Maps/MyImageCache.cs:147, 165`
pub const TILE_EXTENSIONS: [&str; 2] = ["jpg", "png"];

/// One row: the anonymous type's `Name`, `Size` and `Count`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Row {
    /// `Path.GetFileName(s)`, or "Total".
    pub name: String,
    /// `Math.Round(bytes / 1024.0 / 1024.0, MidpointRounding.AwayFromZero) + "MB"`.
    pub size: String,
    /// The files under it.
    pub count: u64,
}

/// `DirSize`: the bytes and the files under a directory, its subdirectories' included.
/// `// C#: Controls/GMAPCache.cs:125-146`
#[must_use]
pub fn dir_size(directory: &Path) -> (u64, u64) {
    let (mut size, mut count) = (0, 0);
    let Ok(entries) = mp_os::fs::read_dir(directory) else {
        return (0, 0);
    };
    for entry in entries.flatten() {
        let Ok(kind) = entry.file_type() else {
            continue;
        };
        if kind.os_is_dir() {
            let (bytes, files) = dir_size(&entry.path());
            size += bytes;
            count += files;
        } else if let Ok(metadata) = entry.metadata() {
            size += metadata.len();
            count += 1;
        }
    }
    (size, count)
}

/// `Math.Round(bytes / 1024.0 / 1024.0, MidpointRounding.AwayFromZero) + "MB"`: a whole number,
/// which `double.ToString()` writes without a point.
/// `// C#: Controls/GMAPCache.cs:39, 52`
#[must_use]
pub fn megabytes(bytes: u64) -> String {
    #[allow(clippy::cast_precision_loss)] // a cache's size, far below 2^53 bytes
    let megabytes = (bytes as f64 / 1024.0 / 1024.0).round();
    format!("{megabytes}MB")
}

/// `Activate`'s list: a row for each provider's directory, by name, then "Total".
/// `// C#: Controls/GMAPCache.cs:31-57`
#[must_use]
pub fn rows(tile_root: &Path) -> Vec<Row> {
    let mut directories: Vec<PathBuf> = mp_os::fs::read_dir(tile_root)
        .map(|entries| {
            entries
                .flatten()
                .filter(|entry| entry.file_type().is_ok_and(|kind| kind.os_is_dir()))
                .map(|entry| entry.path())
                .collect()
        })
        .unwrap_or_default();
    directories.sort();
    let mut list: Vec<Row> = directories
        .iter()
        .map(|directory| {
            let (bytes, count) = dir_size(directory);
            Row {
                name: directory
                    .file_name()
                    .map_or_else(String::new, |name| name.to_string_lossy().into_owned()),
                size: megabytes(bytes),
                count,
            }
        })
        .collect();
    let (bytes, count) = dir_size(tile_root);
    list.push(Row {
        name: TOTAL.to_owned(),
        size: megabytes(bytes),
        count,
    });
    list
}

/// `GMapProviders.List.First(a => a.Name == dir)` found one: a provider of this application's or
/// of the C#'s by that name.
#[must_use]
pub fn is_provider(name: &str) -> bool {
    mp_tiles::source_by_name(name).is_some() || mp_tiles::CSHARP_LIST.contains(&name)
}

/// `File.GetCreationTime`, else the modification time where the file system keeps no creation
/// time.
fn created(metadata: &mp_os::fs::Metadata) -> Option<SystemTime> {
    let written = metadata.created().or_else(|_| metadata.modified()).ok()?;
    // A file's time is std's, the clock here the page's in a browser: the same instant, counted
    // from the same epoch (one type on the desktop).
    let epoch = web_time::UNIX_EPOCH;
    Some(SystemTime::UNIX_EPOCH + written.duration_since(epoch).unwrap_or_default())
}

/// Every file under `directory` with one of the extensions, as `Directory.GetFiles(dir, "*.jpg",
/// SearchOption.AllDirectories)` finds them - the extension in any case, as Windows matches it.
fn tiles_under(directory: &Path, extension: &str, found: &mut Vec<PathBuf>) {
    let Ok(entries) = mp_os::fs::read_dir(directory) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if entry.file_type().is_ok_and(|kind| kind.os_is_dir()) {
            tiles_under(&path, extension, found);
        } else if path
            .extension()
            .is_some_and(|ext| ext.to_string_lossy().eq_ignore_ascii_case(extension))
        {
            found.push(path);
        }
    }
}

/// `MyImageCache.DeleteOlderThan(date, type)`: each `.jpg`, then each `.png`, under the
/// provider's directory created before `date` deleted, and how many were; a file that cannot be
/// deleted is passed over. A provider with no directory, none.
/// `// C#: ExtLibs/Maps/MyImageCache.cs:132-184`
#[must_use]
pub fn delete_older_than(tile_root: &Path, provider: &str, date: SystemTime) -> usize {
    let directory = tile_root.join(provider);
    if !directory.os_is_dir() {
        return 0;
    }
    let mut removed = 0;
    for extension in TILE_EXTENSIONS {
        let mut found = Vec::new();
        tiles_under(&directory, extension, &mut found);
        for file in found {
            let old = mp_os::fs::metadata(&file)
                .ok()
                .as_ref()
                .and_then(created)
                .is_some_and(|made| made < date);
            if old && mp_os::fs::remove_file(&file).is_ok() {
                removed += 1;
            }
        }
    }
    removed
}

/// Which button column was pressed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Removal {
    /// `buttonColumn`, "Remove 30+days".
    Old,
    /// `buttonColumn2`, "Remove All".
    All,
}

impl Removal {
    /// The date before which a file goes: thirty days before now, or now.
    /// `// C#: Controls/GMAPCache.cs:104, 116`
    #[must_use]
    pub fn date(self, now: SystemTime) -> SystemTime {
        match self {
            Self::Old => now
                .checked_sub(THIRTY_DAYS)
                .unwrap_or(SystemTime::UNIX_EPOCH),
            Self::All => now,
        }
    }

    /// The probe id of its cell in a row.
    fn id(self, row: usize) -> String {
        match self {
            Self::Old => format!("mapcache-remove-old-{row}"),
            Self::All => format!("mapcache-remove-all-{row}"),
        }
    }
}

/// The form, while it is open.
#[derive(Debug)]
pub struct Form {
    /// `CacheLocator.Location + "TileDBv3/en"`.
    tile_root: PathBuf,
    /// The grid's rows.
    pub rows: Vec<Row>,
    /// "Removed N images", while its box shows.
    pub message: Option<String>,
}

impl Form {
    /// `new GMAPCache()` and `Activate` on the form's `Load`.
    /// `// C#: Utilities/ExtensionsMP.cs:126, 161-167; Controls/GMAPCache.cs:29-93`
    #[must_use]
    pub fn activate(tile_root: PathBuf) -> Self {
        let rows = rows(&tile_root);
        Self {
            tile_root,
            rows,
            message: None,
        }
    }
}

/// The window, held for the application.
#[derive(Debug, Default)]
pub struct MapCache {
    /// The form, while it is open.
    pub window: Option<Form>,
    /// How many times it has been opened.
    pub opened: usize,
}

impl MapCache {
    /// Ctrl+X: a fresh form over the cache at `cache_root` (`CacheLocator.Location`).
    pub fn show(&mut self, cache_root: &Path) {
        let tile_root = mp_tiles::TileCache::new(cache_root).tile_root();
        self.window = Some(Form::activate(tile_root));
        self.opened += 1;
    }

    /// The close box.
    pub fn close(&mut self) {
        self.window = None;
    }

    /// `myDataGridView1_CellClick` on a button cell: the row's provider's files removed and the
    /// box saying how many, or what the C# throws for a row that names none.
    /// `// C#: Controls/GMAPCache.cs:95-123`
    pub fn remove(&mut self, row: usize, removal: Removal, now: SystemTime) -> Result<(), String> {
        let Some(form) = self.window.as_mut() else {
            return Ok(());
        };
        // `if (e.RowIndex < 0) return;`
        let Some(name) = form.rows.get(row).map(|row| row.name.clone()) else {
            return Ok(());
        };
        if !is_provider(&name) {
            return Err(NO_MATCH.to_owned());
        }
        let removed = delete_older_than(&form.tile_root, &name, removal.date(now));
        form.message = Some(format!("Removed {removed} images"));
        Ok(())
    }

    /// The box's OK: `Activate()` again.
    /// `// C#: Controls/GMAPCache.cs:107-109, 119-121`
    pub fn dismiss(&mut self) {
        if let Some(form) = self.window.as_mut() {
            form.message = None;
            form.rows = rows(&form.tile_root);
        }
    }
}

/// Facts a UI test asserts on, under `mapcache.`.
pub fn record_facts(holder: &MapCache) {
    use crate::facts::record;
    record("mapcache.window", holder.window.is_some());
    record("mapcache.opened", holder.opened);
    let Some(form) = holder.window.as_ref() else {
        return;
    };
    record("mapcache.rows", form.rows.len());
    for (index, row) in form.rows.iter().enumerate() {
        record(
            format!("mapcache.row.{index}"),
            format!("{}|{}|{}", row.name, row.size, row.count),
        );
    }
    record(
        "mapcache.message",
        form.message.as_deref().unwrap_or("none"),
    );
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// `RowTemplate.Height` and the header's, as `DataGridView` makes them.
const ROW_HEIGHT: f32 = 22.0;
/// A character's width and a cell's padding, for sizing a column to what it shows
/// (`AutoSizeColumnsMode.DisplayedCells`).
const CHAR_WIDTH: f32 = 7.0;
const CELL_PADDING: f32 = 12.0;

/// A column's width: its widest text, header included.
fn width_of<'a>(texts: impl Iterator<Item = &'a str>) -> f32 {
    #[allow(clippy::cast_precision_loss)] // a few dozen characters
    let longest = texts.map(|text| text.chars().count()).max().unwrap_or(0) as f32;
    longest * CHAR_WIDTH + CELL_PADDING
}

/// One grid cell.
fn cell(width: f32, text: impl Into<SharedString>) -> gpui::Div {
    div()
        .w(px(width))
        .h_full()
        .px_1()
        .flex()
        .items_center()
        .overflow_hidden()
        .whitespace_nowrap()
        .text_xs()
        .child(text.into())
}

/// A button cell: the column's text on a button filling it; a press anywhere in the cell is the
/// grid's `CellClick`.
fn button_cell(
    row: usize,
    removal: Removal,
    width: f32,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let id = removal.id(row);
    let text = match removal {
        Removal::Old => REMOVE_OLD,
        Removal::All => REMOVE_ALL,
    };
    cell(width, "")
        .p(px(2.0))
        .child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .size_full()
                .flex()
                .items_center()
                .justify_center()
                .rounded_sm()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::ACTION))
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.border_color(rgb(theme::ACCENT)))
                .child(text)
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    let now = SystemTime::now();
                    if let Err(why) = this.key_forms.map_cache.remove(row, removal, now) {
                        this.file_status = Some(why);
                    }
                    cx.notify();
                })),
        )
        .into_any_element()
}

/// The form over the window, its grid filling it, and the box over that.
/// `// C#: Controls/GMAPCache.Designer.cs:29-58`
pub fn overlay(
    holder: &MapCache,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = holder.window.as_ref()?;
    let size = window.viewport_size();
    let [name, size_header, count] = COLUMNS;
    let widths = [
        width_of(std::iter::once(name).chain(form.rows.iter().map(|row| row.name.as_str()))),
        width_of(std::iter::once(size_header).chain(form.rows.iter().map(|row| row.size.as_str()))),
        // The counts' widest, a cache of millions of tiles.
        width_of(std::iter::once(count).chain(form.rows.iter().map(|_| "0000000"))),
        width_of(std::iter::once(REMOVE_OLD)),
        width_of(std::iter::once(REMOVE_ALL)),
    ];
    let mut header = div()
        .flex()
        .flex_shrink_0()
        .h(px(ROW_HEIGHT))
        .bg(rgb(theme::BG))
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .text_color(rgb(theme::DIM));
    for (index, width) in widths.iter().enumerate() {
        header = header.child(cell(*width, COLUMNS.get(index).copied().unwrap_or("")));
    }
    let mut grid = div()
        .id("mapcache-grid")
        .flex()
        .flex_col()
        .size_full()
        .overflow_y_scroll()
        .child(header);
    let [name_width, size_width, count_width, old_width, all_width] = widths;
    for (index, row) in form.rows.iter().enumerate() {
        grid = grid.child(
            crate::probe::measured(format!("mapcache-row-{index}"), div())
                .flex()
                .flex_shrink_0()
                .h(px(ROW_HEIGHT))
                .border_b_1()
                .border_color(rgb(theme::BORDER))
                .text_color(rgb(theme::TEXT))
                .child(cell(name_width, row.name.clone()))
                .child(cell(size_width, row.size.clone()))
                .child(cell(count_width, row.count.to_string()))
                .child(button_cell(index, Removal::Old, old_width, cx))
                .child(button_cell(index, Removal::All, all_width, cx)),
        );
    }
    let client = div()
        .w(px(SIZE.0))
        .h(px(SIZE.1))
        .bg(rgb(theme::BG))
        .child(grid);
    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(div().text_xs().text_color(rgb(theme::DIM)).child(FORM_TEXT))
        .child(crate::ui::action(
            "mapcache-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.key_forms.map_cache.close();
                cx.notify();
            }),
        ));
    let body = crate::probe::measured("mapcache", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    let over = div()
        .id("mapcache-backdrop")
        .w(size.width)
        .h(size.height)
        .flex()
        .items_center()
        .justify_center()
        .occlude()
        .child(body);
    let mut layers = div().child(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(over),
        )
        .with_priority(1),
    );
    // `CustomMessageBox.Show("Removed " + removed + " images")`: no caption, OK.
    if let Some(text) = form.message.as_ref() {
        layers = layers.child(crate::config::optional::message_box(
            "mapcache-message",
            "mapcache-message-ok",
            &Message {
                title: "",
                text: text.clone(),
            },
            window,
            |this| this.key_forms.map_cache.dismiss(),
            cx,
        ));
    }
    Some(layers.into_any_element())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A temporary directory that removes itself.
    struct Scratch(PathBuf);

    impl Scratch {
        fn new(name: &str) -> Self {
            let path = mp_os::temp_dir().join(format!(
                "mp-gmapcache-{name}-{}-{:?}",
                mp_os::process_id(),
                wasm_thread::current().id()
            ));
            let _ = mp_os::fs::remove_dir_all(&path);
            mp_os::fs::create_dir_all(&path).expect("a scratch directory");
            Self(path)
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = mp_os::fs::remove_dir_all(&self.0);
        }
    }

    /// A file of `bytes` bytes at `path`, its directories made.
    fn file(path: &Path, bytes: usize) {
        mp_os::fs::create_dir_all(path.parent().expect("a parent")).expect("directories");
        mp_os::fs::write(path, vec![0x42; bytes]).expect("a file");
    }

    /// A cache of two providers: OpenStreetMap with three tiles, one a `.png`, and a half-written
    /// `.part`; Bing with one tile of 1.5 MB.
    fn cache(scratch: &Scratch) -> PathBuf {
        let root = scratch.0.join("gmapcache");
        let tiles = mp_tiles::TileCache::new(&root).tile_root();
        file(&tiles.join("OpenStreetMap/16/39658/59922.jpg"), 1000);
        file(&tiles.join("OpenStreetMap/16/39658/59923.jpg"), 1000);
        file(&tiles.join("OpenStreetMap/16/39659/59922.png"), 1000);
        file(&tiles.join("OpenStreetMap/16/39659/59923.jpg.part"), 10);
        file(&tiles.join("BingMap/3/1/2.jpg"), 1024 * 1024 + 512 * 1024);
        root
    }

    /// `Math.Round(x, MidpointRounding.AwayFromZero)`: a half goes up; no point in a whole number.
    #[test]
    fn megabytes_round_half_away_from_zero() {
        assert_eq!(megabytes(0), "0MB");
        assert_eq!(megabytes(512 * 1024 - 1), "0MB");
        assert_eq!(megabytes(512 * 1024), "1MB");
        assert_eq!(megabytes(1024 * 1024 + 512 * 1024), "2MB");
        assert_eq!(megabytes(2 * 1024 * 1024 + 512 * 1024), "3MB");
        assert_eq!(megabytes(1024 * 1024 * 1024), "1024MB");
    }

    /// `Activate`: a row for each provider by name, every file counted, then Total.
    #[test]
    fn activate_lists_each_provider_then_the_total() {
        let scratch = Scratch::new("rows");
        let root = cache(&scratch);
        let mut holder = MapCache::default();
        holder.show(&root);
        let form = holder.window.as_ref().expect("the form");
        assert_eq!(
            form.rows,
            [
                Row {
                    name: "BingMap".to_owned(),
                    size: "2MB".to_owned(),
                    count: 1,
                },
                Row {
                    name: "OpenStreetMap".to_owned(),
                    size: "0MB".to_owned(),
                    count: 4,
                },
                Row {
                    name: TOTAL.to_owned(),
                    size: "2MB".to_owned(),
                    count: 5,
                },
            ]
        );
        assert_eq!(holder.opened, 1);
        assert!(form.message.is_none());
    }

    /// No cache yet: Total alone, nothing.
    #[test]
    fn an_empty_cache_is_the_total_alone() {
        let scratch = Scratch::new("empty");
        let mut holder = MapCache::default();
        holder.show(&scratch.0.join("gmapcache"));
        let form = holder.window.as_ref().expect("the form");
        assert_eq!(
            form.rows,
            [Row {
                name: TOTAL.to_owned(),
                size: "0MB".to_owned(),
                count: 0,
            }]
        );
    }

    /// `DeleteOlderThan`: only `.jpg` and `.png` made before the date; how many.
    #[test]
    fn delete_older_than_takes_the_tiles_made_before_the_date() {
        let scratch = Scratch::new("delete");
        let root = cache(&scratch);
        let tiles = mp_tiles::TileCache::new(&root).tile_root();
        let past = SystemTime::now() - THIRTY_DAYS;
        assert_eq!(delete_older_than(&tiles, "OpenStreetMap", past), 0);
        let future = SystemTime::now() + Duration::from_secs(60);
        assert_eq!(delete_older_than(&tiles, "OpenStreetMap", future), 3);
        // The `.part` is not a tile, and stays; Bing is untouched.
        assert_eq!(dir_size(&tiles.join("OpenStreetMap")).1, 1);
        assert_eq!(dir_size(&tiles.join("BingMap")).1, 1);
        assert_eq!(delete_older_than(&tiles, "NoSuchProvider", future), 0);
    }

    /// The buttons: Remove 30+days takes nothing just made; Remove All takes it all; the box
    /// says how many, and its OK fills the grid again. Total, and a name no provider has, are
    /// `First`'s throw, on the status line.
    #[test]
    fn the_buttons_remove_and_say_how_many() {
        let scratch = Scratch::new("buttons");
        let root = cache(&scratch);
        let mut holder = MapCache::default();
        holder.show(&root);
        let now = SystemTime::now() + Duration::from_secs(1);
        assert_eq!(holder.remove(1, Removal::Old, now), Ok(()));
        let form = holder.window.as_ref().expect("the form");
        assert_eq!(form.message.as_deref(), Some("Removed 0 images"));
        holder.dismiss();
        assert_eq!(holder.remove(1, Removal::All, now), Ok(()));
        let form = holder.window.as_ref().expect("the form");
        assert_eq!(form.message.as_deref(), Some("Removed 3 images"));
        // The grid is filled again only once the box is closed.
        assert_eq!(form.rows[1].count, 4);
        holder.dismiss();
        let form = holder.window.as_ref().expect("the form");
        assert!(form.message.is_none());
        assert_eq!(form.rows[1].count, 1);
        assert_eq!(form.rows[2].count, 2);
        // Total names no provider.
        assert_eq!(
            holder.remove(2, Removal::All, now),
            Err(NO_MATCH.to_owned())
        );
        assert!(
            holder
                .window
                .as_ref()
                .is_some_and(|form| form.message.is_none())
        );
        // A row past the end is no row.
        assert_eq!(holder.remove(9, Removal::All, now), Ok(()));
        holder.close();
        assert!(holder.window.is_none());
    }

    /// `GMapProviders.List`: the C#'s names and this application's.
    #[test]
    fn providers_are_the_csharps_and_this_applications() {
        assert!(is_provider("OpenStreetMap"));
        assert!(is_provider("OpenCycleMap"));
        assert!(is_provider("OpenTopoMap"));
        assert!(!is_provider(TOTAL));
        assert!(!is_provider("openstreetmap"));
    }

    /// The thirty days, and now.
    #[test]
    fn the_dates_are_thirty_days_ago_and_now() {
        let now = SystemTime::now();
        assert_eq!(Removal::All.date(now), now);
        assert_eq!(
            now.duration_since(Removal::Old.date(now)).ok(),
            Some(Duration::from_secs(2_592_000))
        );
    }

    /// The Designer's grid and size, and the button columns' words, read from the tree when it
    /// is here.
    #[test]
    fn the_words_and_size_are_the_csharps() {
        let (Some(code), Some(designer)) = (
            crate::config_coverage::source::csharp("Controls/GMAPCache.cs"),
            crate::config_coverage::source::csharp("Controls/GMAPCache.Designer.cs"),
        ) else {
            eprintln!(
                "skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner"
            );
            return;
        };
        assert!(code.contains(&format!("Text = \"{REMOVE_OLD}\"")));
        assert!(code.contains(&format!("Text = \"{REMOVE_ALL}\"")));
        assert!(code.contains(&format!("Name = \"{TOTAL}\"")));
        assert!(code.contains("DateTime.Now.AddDays(-30)"));
        assert!(code.contains("DateTime.Now.AddDays(0)"));
        assert!(code.contains("\"Removed \" + removed + \" images\""));
        assert!(designer.contains(&format!(
            "this.Size = new System.Drawing.Size({}, {});",
            SIZE.0, SIZE.1
        )));
        assert!(designer.contains("this.myDataGridView1.ReadOnly = true;"));
    }
}
