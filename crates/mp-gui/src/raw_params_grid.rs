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

//! The Full Parameter List's grid, the rest of it: `ConfigRawParams`'s Fav column and the sort
//! that puts favourites first, the Options column and the control drawn in it when a row is
//! entered, the Units and Desc columns and the Desc cell's link, the typed Value cell with its
//! ReadOnly and out-of-range questions, Ctrl+S, the columns' widths and the splitter's distance
//! kept between visits, and the RawParamWarning box.
//!
//! The screen is `params.rs` (the group list, the rows, the editor and the file panel) and
//! `raw_params.rs` (row 82's right-hand column and `_changes`); this is the grid's own behaviour
//! and the state behind it.
//!
//! * **Fav** (`:595-596, 1045-1063, 833-858`): each row's check box is ticked from the
//!   `fav_params` setting when the rows are made; clicking it appends the name to the list or
//!   sets the list without it, and sorts the grid again - favourites first, then by name in
//!   `NaturalStringComparer`'s order.
//! * **Options** (`:613-642, 1151-1320`): the cell's text is the documentation's range and its
//!   values, one to a line. Entering a row puts a control over that cell: Set Bitmask for a
//!   documented bitmask, a drop-down list of the values for a documented list, a `NumericUpDown`
//!   over the range for a documented range, and nothing otherwise. Each writes the Value cell,
//!   which is an edit like any other (below).
//! * **Units** and **Desc** (`:604-645, 1027-1091`): filled from the documentation when it has a
//!   description; clicking the Desc cell opens the first `http`/`https` address in it.
//! * **An edit** (`Params_CellValueChanged`, `:454-533`): `RCn_REV`/`HSn_REV` 0 is made -1, the
//!   text is read as a number (a comma taken for a point), a parameter the documentation marks
//!   ReadOnly gets "NAME is marked as ReadOnly, and will not be changed" and keeps its value, and
//!   a value outside the documented range asks "NAME value is out of range. Do you want to
//!   continue?" - No keeps the old value. A text that is no number turns the cell red.
//! * **Ctrl+S** (`ProcessCmdKey`, `:116-125`) is Write Params (`BUT_writePIDS_Click`,
//!   `:257-394`) over `_changes`.
//! * **The widths and the splitter** (`:70-86, 99-114`): `Activate` sets each resizable column's
//!   width from `rawparam_<column>_width` and the splitter from `rawparam_splitterdistance`;
//!   `Deactivate` saves them. (No `SplitterMoved` or `ColumnWidthChanged` handler saves anything:
//!   `Params_ColumnWidthChanged` only moves the Options control, `:1353-1359`.)
//! * **RawParamWarning** (`:92`): `Common.MessageShowAgain(Strings.RawParamWarning,
//!   Strings.RawParamWarningi)` on every `Activate`, with its "Show me again?" box kept under
//!   `SHOWAGAIN_Raw_Param_Warning`.
//!
//! This screen writes a value as it is edited (see `raw_params.rs`, `RawParams::changes`), so an
//! edit that passes its questions is written at once rather than waiting for Write Params, and
//! `_changes` holds what is on its way or timed out; Ctrl+S writes those again.
//!
//! **Where the Options control is drawn.** In the row, over the entered row's Options cell: the
//! C# adds it to the grid's own controls at `GetCellDisplayRectangle(Options.Index, row)`
//! (`:1203, 1247, 1313`) and moves it with the cell as the grid scrolls (`:1323-1359`). The
//! parameter panel under the grid (`params::editor_panel`) is this port's own and has no C#
//! counterpart, so the control is not put there; that panel is left as it was.
//!
//! **Where this differs, and why** (each also at its site):
//!
//! * a typed value is read as arithmetic - `+ - * / ^`, parentheses, a unary minus - and not
//!   mXparser's whole grammar of functions and constants (`calculate`);
//! * the `NumericUpDown`'s box, clicked, begins the Value cell's edit with its text: what is
//!   typed shows in the Value cell, where the C# mirrors each keystroke into it (`options_cell`);
//! * a bit clicked in Set Bitmask's window edits that window's parameter, where the C# writes
//!   whichever row is current by then (`param_grid_click_bit`);
//! * the ReadOnly box and the out-of-range question keep their boxes (questions, per the owner's
//!   ruling of 2026-09-25); Write Params' closed-port refusals go on the status line, and its
//!   "N parameters successfully saved." with them (`RawGrid::save`, `RawGrid::saved`).
//!
//! `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, VecDeque};

use gpui::{
    AnyElement, App, Context, FocusHandle, KeyDownEvent, MouseButton, SharedString, Window, div,
    prelude::*, px, rgb,
};
use mp_link::requests::RequestOutcome;
use mp_params::ParamMeta;

use crate::MissionPlanner;
use crate::config::servo_output::{self, Combo};
use crate::params::{Parameter, Written};
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action, theme};

// --- The columns -------------------------------------------------------------------------------

/// The grid's columns, in the Designer's order.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.Designer.cs:224-231`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Column {
    /// `Command`: the name.
    Command,
    /// `Value`.
    Value,
    /// `Default_value`: shown only when the vehicle gave defaults.
    DefaultValue,
    /// `Units`.
    Units,
    /// `Options`.
    Options,
    /// `Desc`: `AutoSizeMode = Fill`.
    Desc,
    /// `Fav`: a check box.
    Fav,
}

impl Column {
    /// Every column, in order.
    pub const ALL: [Self; 7] = [
        Self::Command,
        Self::Value,
        Self::DefaultValue,
        Self::Units,
        Self::Options,
        Self::Desc,
        Self::Fav,
    ];

    /// Its place in [`Column::ALL`].
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::Command => 0,
            Self::Value => 1,
            Self::DefaultValue => 2,
            Self::Units => 3,
            Self::Options => 4,
            Self::Desc => 5,
            Self::Fav => 6,
        }
    }

    /// `col.Name`, which the settings' keys are made of.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.Designer.cs:264-309`
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Command => "Command",
            Self::Value => "Value",
            Self::DefaultValue => "Default_value",
            Self::Units => "Units",
            Self::Options => "Options",
            Self::Desc => "Desc",
            Self::Fav => "Fav",
        }
    }

    /// `HeaderText`. `// C#: GCSViews/ConfigurationView/ConfigRawParams.resx`
    #[must_use]
    pub const fn header(self) -> &'static str {
        match self {
            Self::Command => "Name",
            Self::Value => "Value",
            Self::DefaultValue => "Default",
            Self::Units => "Units",
            Self::Options => "Options",
            Self::Desc => "Desc",
            Self::Fav => "Fav",
        }
    }

    /// `Width` in the `.resx`; the Desc column fills what is left and has none.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.resx`
    #[must_use]
    pub const fn width(self) -> i32 {
        match self {
            Self::Command => 130,
            Self::Value | Self::DefaultValue => 70,
            Self::Units => 60,
            Self::Options => 150,
            Self::Desc => 0,
            Self::Fav => 30,
        }
    }

    /// `MinimumWidth`, below which WinForms does not set a column.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.resx`
    #[must_use]
    pub const fn minimum(self) -> i32 {
        match self {
            Self::Fav => 30,
            _ => 50,
        }
    }

    /// Whether `Activate` and `Deactivate` keep its width: every column but the fill column
    /// (all are resizable - `Fav.Resizable` is set True, the rest inherit it).
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:72-76, 103-107`
    #[must_use]
    pub const fn kept(self) -> bool {
        !matches!(self, Self::Desc)
    }

    /// `"rawparam_" + col.Name + "_width"`.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:78, 109`
    #[must_use]
    pub fn key(self) -> String {
        format!("rawparam_{}_width", self.name())
    }
}

/// `new DataGridViewRow() { Height = 36 }`. `// C#: ConfigRawParams.cs:589`
pub const ROW_HEIGHT: f32 = 36.0;

/// How the grid is sorted: a column header clicked sorts by it, ascending, and clicked again
/// descending, as a `DataGridView`'s automatic sort on a text column does (the Fav check-box
/// column has no automatic sort). `Params.Sort(Command, Ascending)` at start and after each
/// load; `OnParamsOnSortCompare` keeps favourites first either way.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:674-676, 833-858, 1062`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sort {
    /// The column sorted by.
    pub column: Column,
    /// Ascending, else descending.
    pub ascending: bool,
}

impl Default for Sort {
    fn default() -> Self {
        Self {
            column: Column::Command,
            ascending: true,
        }
    }
}

impl Sort {
    /// For the fact: `Command asc`.
    #[must_use]
    pub fn label(self) -> String {
        format!(
            "{} {}",
            self.column.name(),
            if self.ascending { "asc" } else { "desc" }
        )
    }

    /// A header clicked: the same column turns the order round, another sorts ascending;
    /// the Fav column is `NotSortable` and changes nothing.
    pub fn click(&mut self, column: Column) {
        if column == Column::Fav {
            return;
        }
        if self.column == column {
            self.ascending = !self.ascending;
        } else {
            *self = Self {
                column,
                ascending: true,
            };
        }
    }
}
/// Where the splitter's distance is kept. `// C#: ConfigRawParams.cs:84, 112`
pub const SPLITTER_KEY: &str = "rawparam_splitterdistance";
/// `GetInt32("rawparam_splitterdistance", 180)`'s default, the Designer's distance.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:84; ConfigRawParams.resx`
pub const SPLITTER_DEFAULT: i32 = 180;
/// A `SplitContainer`'s `SplitterWidth` and `Panel1MinSize` and `Panel2MinSize`: WinForms'
/// defaults, which the Designer leaves.
pub const SPLITTER_WIDTH: f32 = 4.0;
/// See [`SPLITTER_WIDTH`].
pub const PANEL_MIN: i32 = 25;
/// `but_collapse.Size`, 18 by 18, docked at the grid's left. `// C#: ConfigRawParams.resx`
pub const COLLAPSE_SIZE: f32 = 18.0;

/// `Settings.GetInt32(key, default)`: `int.TryParse` of the value - white space around it and
/// a sign allowed - or the default.
/// `// C#: ExtLibs/Utilities/Settings.cs:198-207`
#[must_use]
pub fn get_int32(value: Option<&str>, default: i32) -> i32 {
    value
        .and_then(|value| value.trim().parse::<i32>().ok())
        .unwrap_or(default)
}

/// The columns' widths and the splitter's distance.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Layout {
    /// By [`Column::index`]; the Desc column's is unused.
    widths: [i32; 7],
    /// `splitContainer1.SplitterDistance`: the tree's width.
    splitter: i32,
}

impl Default for Layout {
    /// The Designer's.
    fn default() -> Self {
        Self {
            widths: Column::ALL.map(Column::width),
            splitter: SPLITTER_DEFAULT,
        }
    }
}

impl Layout {
    /// `Activate`'s part: each kept column whose setting is not empty set to
    /// `Math.Max(5, GetInt32(key))` - which WinForms raises to the column's `MinimumWidth` -
    /// and the splitter to `GetInt32("rawparam_splitterdistance", 180)`. A column without the
    /// setting keeps the width it had.
    ///
    /// The C# sets `SplitterDistance` as read, and WinForms throws for a negative one; here
    /// anything under `Panel1MinSize` is `Panel1MinSize`.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:70-84`
    pub fn restore<'a>(&mut self, get: impl Fn(&str) -> Option<&'a str>) {
        for column in Column::ALL.into_iter().filter(|column| column.kept()) {
            if let Some(text) = get(&column.key()).filter(|text| !text.is_empty()) {
                self.set_width(column, get_int32(Some(text), 0).max(5));
            }
        }
        self.splitter = get_int32(get(SPLITTER_KEY), SPLITTER_DEFAULT).max(PANEL_MIN);
    }

    /// `Deactivate`'s part: each kept column's width as `ToString("0")`, and the splitter's
    /// distance.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:99-112`
    #[must_use]
    pub fn saved(&self) -> Vec<(String, String)> {
        let mut saved: Vec<(String, String)> = Column::ALL
            .into_iter()
            .filter(|column| column.kept())
            .map(|column| (column.key(), self.width(column).to_string()))
            .collect();
        saved.push((SPLITTER_KEY.to_owned(), self.splitter.to_string()));
        saved
    }

    /// A column's width.
    #[must_use]
    pub fn width(&self, column: Column) -> i32 {
        self.widths
            .get(column.index())
            .copied()
            .unwrap_or_else(|| column.width())
    }

    /// `col.Width = width`: never under the column's `MinimumWidth`.
    pub fn set_width(&mut self, column: Column, width: i32) {
        if let Some(kept) = self.widths.get_mut(column.index()) {
            *kept = width.max(column.minimum());
        }
    }

    /// The splitter's distance.
    #[must_use]
    pub const fn splitter(&self) -> i32 {
        self.splitter
    }

    /// The splitter dragged: never under `Panel1MinSize`, nor so far that Panel2 is under its
    /// own, in a split `total` wide.
    pub fn set_splitter(&mut self, distance: i32, total: i32) {
        let most = total - PANEL_MIN - 4;
        self.splitter = distance.min(most).max(PANEL_MIN);
    }
}

/// What is being dragged.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dragged {
    /// A column's right edge in the header.
    Column(Column),
    /// The splitter between the tree and the grid.
    Splitter,
}

/// A drag under way: what, where the pointer was pressed, and the width or distance then.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Drag {
    what: Dragged,
    from: f32,
    start: i32,
}

// --- Fav -----------------------------------------------------------------------------------------

/// Where the favourites are kept. `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:595, 1053`
pub const FAV_KEY: &str = "fav_params";

/// `WebUtility.UrlDecode`: `+` is a space and `%XX` a byte; anything else, a `%` without two
/// hex digits after it included, is kept.
fn url_decode(text: &str) -> String {
    let bytes = text.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut at = 0;
    while let Some(&byte) = bytes.get(at) {
        let hex = |offset: usize| {
            bytes
                .get(at + offset)
                .and_then(|digit| char::from(*digit).to_digit(16))
        };
        match byte {
            b'+' => out.push(b' '),
            b'%' => {
                if let (Some(high), Some(low)) = (hex(1), hex(2)) {
                    out.push(u8::try_from(high * 16 + low).unwrap_or(b'%'));
                    at += 3;
                    continue;
                }
                out.push(byte);
            }
            _ => out.push(byte),
        }
        at += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// `WebUtility.UrlEncode`: letters, digits and `-_.!*()` as they are, a space `+`, every other
/// byte `%XX` in capitals.
fn url_encode(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for byte in text.bytes() {
        match byte {
            b'A'..=b'Z'
            | b'a'..=b'z'
            | b'0'..=b'9'
            | b'-'
            | b'_'
            | b'.'
            | b'!'
            | b'*'
            | b'('
            | b')' => out.push(char::from(byte)),
            b' ' => out.push('+'),
            _ => out.push_str(&format!("%{byte:02X}")),
        }
    }
    out
}

/// `Settings.GetList(key)`: the value split at `;`, each decoded, the first of each kept; an
/// absent key is an empty list.
/// `// C#: ExtLibs/Utilities/Settings.cs:164-169`
#[must_use]
pub fn get_list(value: Option<&str>) -> Vec<String> {
    let mut list: Vec<String> = Vec::new();
    for item in value.into_iter().flat_map(|value| value.split(';')) {
        let item = url_decode(item);
        if !list.contains(&item) {
            list.push(item);
        }
    }
    list
}

/// `Settings.SetList(key, list)`'s value: each once, encoded, joined with `;`. `None` for an
/// empty list, which `SetList` returns on without touching the setting.
/// `// C#: ExtLibs/Utilities/Settings.cs:171-176`
#[must_use]
pub fn set_list(list: &[String]) -> Option<String> {
    if list.is_empty() {
        return None;
    }
    let mut seen: Vec<String> = Vec::new();
    for item in list {
        let encoded = url_encode(item);
        if !seen.contains(&encoded) {
            seen.push(encoded);
        }
    }
    Some(seen.join(";"))
}

/// The Fav cell clicked: the setting's new value, if it changes. Ticked is
/// `AppendList("fav_params", name)`; unticked is `SetList("fav_params", list without name)` -
/// which leaves the setting as it was when that list is empty, so the last favourite unticked
/// is still in `fav_params` and comes back ticked the next time the rows are made. That is the
/// C#'s, kept.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1045-1060; ExtLibs/Utilities/Settings.cs:171-183`
#[must_use]
pub fn fav_clicked(setting: Option<&str>, name: &str, ticked: bool) -> Option<String> {
    let mut list = get_list(setting);
    if ticked {
        list.push(name.to_owned());
    } else {
        list.retain(|item| item != name);
    }
    set_list(&list)
}

/// `NaturalStringComparer.NaturalCompare`: runs of digits compared as numbers - leading zeros
/// skipped, the longer run the larger, then digit by digit - and everything else character by
/// character, upper-cased. (`char.IsDigit` is any Unicode decimal digit; parameter names are
/// ASCII, and only ASCII digits are taken as digits here.)
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:746-829`
#[must_use]
pub fn natural_compare(x: &str, y: &str) -> Ordering {
    // A parameter's name is ASCII, and is compared without collecting it; anything else is
    // compared by its characters.
    if x.is_ascii() && y.is_ascii() {
        let (x, y) = (x.as_bytes(), y.as_bytes());
        natural_in(
            |at| x.get(at).map(|byte| char::from(*byte)),
            |at| y.get(at).map(|byte| char::from(*byte)),
        )
    } else {
        let (x, y): (Vec<char>, Vec<char>) = (x.chars().collect(), y.chars().collect());
        natural_in(|at| x.get(at).copied(), |at| y.get(at).copied())
    }
}

/// [`natural_compare`] over two texts' characters, by position.
fn natural_in(x: impl Fn(usize) -> Option<char>, y: impl Fn(usize) -> Option<char>) -> Ordering {
    let (mut ix, mut iy) = (0, 0);
    loop {
        // One string has ended: equal if both have, else the shorter first.
        let (cx, cy) = match (x(ix), y(iy)) {
            (Some(cx), Some(cy)) => (cx, cy),
            (None, None) => return Ordering::Equal,
            (None, Some(_)) => return Ordering::Less,
            (Some(_), None) => return Ordering::Greater,
        };
        if cx.is_ascii_digit() && cy.is_ascii_digit() {
            // Leading zeroes skipped, then the longer number is the bigger, then digit by digit.
            while x(ix) == Some('0') {
                ix += 1;
            }
            while y(iy) == Some('0') {
                iy += 1;
            }
            let end = |text: &dyn Fn(usize) -> Option<char>, mut at: usize| {
                while text(at).is_some_and(|c| c.is_ascii_digit()) {
                    at += 1;
                }
                at
            };
            let (ex, ey) = (end(&x, ix), end(&y, iy));
            match (ex - ix).cmp(&(ey - iy)) {
                Ordering::Equal => {}
                longer => return longer,
            }
            while ix < ex {
                match x(ix).cmp(&y(iy)) {
                    Ordering::Equal => {}
                    other => return other,
                }
                ix += 1;
                iy += 1;
            }
        } else {
            let upper = |c: char| c.to_uppercase().next().unwrap_or(c);
            match upper(cx).cmp(&upper(cy)) {
                Ordering::Equal => {}
                other => return other,
            }
            ix += 1;
            iy += 1;
        }
    }
}

/// `OnParamsOnSortCompare` under `Params.Sort(Command, Ascending)`: favourites first, and by
/// name in natural order within each.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:674-676, 833-858, 1062`
pub fn sort_rows(
    rows: &mut [&Parameter],
    favourites: &BTreeSet<String>,
    sort: Sort,
    changes: &BTreeMap<String, f64>,
) {
    // The cell's text in the sorted column, as the comparer reads `CellValue1.ToString()`.
    let text = |parameter: &Parameter| -> String {
        match sort.column {
            Column::Command | Column::Fav => parameter.name.clone(),
            Column::Value => value_text(parameter, changes),
            Column::DefaultValue => parameter.default_shown(),
            Column::Units => cells(parameter.meta).units,
            Column::Options => cells(parameter.meta).options,
            Column::Desc => cells(parameter.meta).desc,
        }
    };
    rows.sort_by(|a, b| {
        let (fa, fb) = (favourites.contains(&a.name), favourites.contains(&b.name));
        fb.cmp(&fa).then_with(|| {
            let order = natural_compare(&text(a), &text(b))
                .then_with(|| natural_compare(&a.name, &b.name));
            if sort.ascending { order } else { order.reverse() }
        })
    });
}

// --- The cells' tooltips ---------------------------------------------------------------------------

/// `maximumSingleLineTooltipLength`: a description shorter than this is one line.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:27`
const MAX_SINGLE_LINE_TOOLTIP: usize = 50;

/// `AddNewLinesForTooltip`: the Name, Value and Desc cells' tooltip. Text under fifty characters
/// stays as it is; longer text is broken into lines of about `2 * sqrt(length)` characters -
/// a break at the first whitespace once a line is that long, the whitespace after a break
/// dropped - so a long description reads as a block rather than one line across the screen.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:535-560`
#[must_use]
pub fn tooltip_lines(text: &str) -> String {
    let chars: Vec<char> = text.chars().collect();
    if chars.len() < MAX_SINGLE_LINE_TOOLTIP {
        return text.to_owned();
    }
    // `(int)Math.Sqrt(text.Length) * 2`: the root truncated, then doubled.
    #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let line_length = (chars.len() as f64).sqrt() as usize * 2;
    let mut out = String::new();
    let mut position = 0;
    let mut index = 0;
    while let Some(&current) = chars.get(index) {
        if position >= line_length && current.is_whitespace() {
            out.push('\n');
            position = 0;
        }
        if position == 0 {
            while chars.get(index).is_some_and(|c| c.is_whitespace()) {
                index += 1;
            }
        }
        if let Some(&c) = chars.get(index) {
            out.push(c);
        }
        position += 1;
        index += 1;
    }
    out
}

/// The Options cell's tooltip: the values one to a line, and past fifty of them in columns -
/// `(N - 1) / 50 + 1` values a line, each followed by ", " - as `processToScreen` lays them out.
/// None without values.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:622-642`
#[must_use]
pub fn options_tooltip(options: &str) -> Option<String> {
    if options.is_empty() {
        return None;
    }
    let commas = options.matches(',').count();
    if commas <= 50 {
        return Some(options.replace(',', "\n"));
    }
    let columns = (commas - 1) / 50 + 1;
    let mut opts = options.split(',');
    let mut out = String::new();
    let mut i = 0;
    'lines: loop {
        for _ in 0..columns {
            out.push_str(opts.next().unwrap_or_default());
            out.push_str(", ");
            i += 1;
            if i >= commas {
                break 'lines;
            }
        }
        out.push('\n');
    }
    Some(out)
}

// --- A typed value's expression --------------------------------------------------------------------

/// `new Expression(value).calculate()`: the Value cell's text is an arithmetic expression, not
/// only a number - `2*3.5`, `(1+2)/4`, `-0.5`, `2^10`. mXparser's grammar for these: `+ - * /`
/// and `^` (right to left), parentheses, a unary minus, decimal numbers; whitespace ignored.
/// Anything else mXparser also knows - its functions and constants, `pi`, `sqrt(2)` - is not
/// read here and is NaN, which the C# throws on and the cell refuses (the divergence at the
/// top of the file). `None` for what does not parse, NaN or infinity.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:471-478`
#[must_use]
pub fn calculate(text: &str) -> Option<f64> {
    let chars: Vec<char> = text.chars().filter(|c| !c.is_whitespace()).collect();
    let mut at = 0;
    let value = expr_sum(&chars, &mut at)?;
    (at == chars.len() && value.is_finite()).then_some(value)
}

fn expr_sum(chars: &[char], at: &mut usize) -> Option<f64> {
    let mut left = expr_product(chars, at)?;
    while let Some(&op) = chars.get(*at)
        && (op == '+' || op == '-')
    {
        *at += 1;
        let right = expr_product(chars, at)?;
        left = if op == '+' { left + right } else { left - right };
    }
    Some(left)
}

fn expr_product(chars: &[char], at: &mut usize) -> Option<f64> {
    let mut left = expr_power(chars, at)?;
    while let Some(&op) = chars.get(*at)
        && (op == '*' || op == '/')
    {
        *at += 1;
        let right = expr_power(chars, at)?;
        left = if op == '*' { left * right } else { left / right };
    }
    Some(left)
}

fn expr_power(chars: &[char], at: &mut usize) -> Option<f64> {
    let base = expr_unary(chars, at)?;
    if chars.get(*at) == Some(&'^') {
        *at += 1;
        // Right to left: `2^3^2` is `2^9`.
        let exponent = expr_power(chars, at)?;
        return Some(base.powf(exponent));
    }
    Some(base)
}

fn expr_unary(chars: &[char], at: &mut usize) -> Option<f64> {
    match chars.get(*at) {
        Some('-') => {
            *at += 1;
            expr_unary(chars, at).map(|value| -value)
        }
        Some('+') => {
            *at += 1;
            expr_unary(chars, at)
        }
        Some('(') => {
            *at += 1;
            let inner = expr_sum(chars, at)?;
            if chars.get(*at) != Some(&')') {
                return None;
            }
            *at += 1;
            Some(inner)
        }
        _ => expr_number(chars, at),
    }
}

/// A number: digits with a point, and mXparser's `1e3` / `1.5E-2` exponent form.
fn expr_number(chars: &[char], at: &mut usize) -> Option<f64> {
    let start = *at;
    while chars.get(*at).is_some_and(|c| c.is_ascii_digit() || *c == '.') {
        *at += 1;
    }
    if *at == start {
        return None;
    }
    if chars.get(*at).is_some_and(|c| *c == 'e' || *c == 'E') {
        let mut after = *at + 1;
        if chars.get(after).is_some_and(|c| *c == '+' || *c == '-') {
            after += 1;
        }
        let digits = after;
        while chars.get(after).is_some_and(char::is_ascii_digit) {
            after += 1;
        }
        if after > digits {
            *at = after;
        }
    }
    chars.get(start..*at)?.iter().collect::<String>().parse().ok()
}

// --- The documentation's cells -------------------------------------------------------------------

/// The Units, Options and Desc cells, as `processToScreen` fills them: only for a parameter the
/// documentation describes.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Cells {
    /// `Units`.
    pub units: String,
    /// `Options`: `(range + "\n" + options.Replace(",", "\n")).Trim()`.
    pub options: String,
    /// `Desc`.
    pub desc: String,
}

/// The documentation's range as `GetParameterMetaData(..., Range, ...)` returns it: the file's
/// own text, `0.0 1.0` as written; from the two numbers where a table carries only those.
fn range_text(meta: &ParamMeta) -> String {
    if !meta.range_text.is_empty() {
        return meta.range_text.to_owned();
    }
    meta.range
        .map(|(low, high)| format!("{low} {high}"))
        .unwrap_or_default()
}

/// `GetParameterMetaData(..., Values, ...)` as the fetched file's reader makes it: `code:text,`
/// for each value.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:236-248`
fn values_text(meta: &ParamMeta) -> String {
    meta.values
        .iter()
        .map(|(code, text)| format!("{code}:{text},"))
        .collect()
}

/// The cells `processToScreen` fills from the documentation - nothing without a description.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:604-645`
#[must_use]
pub fn cells(meta: Option<&ParamMeta>) -> Cells {
    let Some(meta) = meta.filter(|meta| !meta.description.is_empty()) else {
        return Cells::default();
    };
    let options = format!(
        "{}\n{}",
        range_text(meta),
        values_text(meta).replace(',', "\n")
    );
    Cells {
        units: meta.units.to_owned(),
        options: options.trim().to_owned(),
        desc: meta.description.to_owned(),
    }
}

/// `CheckForUrlAndLaunchInBrowser`: the first word of the text that is an absolute `http` or
/// `https` address. (`Uri.TryCreate` is taken as a scheme, `://` and a host after it.)
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1066-1091`
#[must_use]
pub fn first_url(text: &str) -> Option<&str> {
    text.split(' ').find(|word| {
        let lower = word.to_ascii_lowercase();
        ["http://", "https://"].iter().any(|scheme| {
            lower.starts_with(scheme)
                && word
                    .get(scheme.len()..)
                    .is_some_and(|rest| rest.chars().next().is_some_and(|c| c != '/'))
        })
    })
}

// --- An edit -------------------------------------------------------------------------------------

/// The ReadOnly box's caption. `// C#: ConfigRawParams.cs:488-491`
pub const READ_ONLY_CAPTION: &str = "ReadOnly";
/// The out-of-range question's caption. `// C#: ConfigRawParams.cs:505-508`
pub const OUT_OF_RANGE_CAPTION: &str = "Out of range";

/// The ReadOnly box's text. `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:488-491`
#[must_use]
pub fn read_only_text(name: &str) -> String {
    format!("{name} is marked as ReadOnly, and will not be changed")
}

/// The out-of-range question. `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:504-508`
#[must_use]
pub fn out_of_range_text(name: &str) -> String {
    format!("{name} value is out of range. Do you want to continue?")
}

/// What an edit of the Value cell comes to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Edit {
    /// Every check passed: `_changes[name] = newvalue` - here, written.
    Write(f64),
    /// Marked ReadOnly: the box, and the old value kept.
    ReadOnly,
    /// Outside the documented range: the question, and on Yes this value written.
    OutOfRange(f64),
    /// Not a number, or a ReadOnly text `bool.Parse` refuses: the `catch`, the cell red.
    Invalid,
}

/// `bool.Parse`: "True" or "False" in any case, white space around it allowed; anything else
/// throws.
fn bool_parse(text: &str) -> Option<bool> {
    let text = text.trim();
    if text.eq_ignore_ascii_case("true") {
        Some(true)
    } else if text.eq_ignore_ascii_case("false") {
        Some(false)
    } else {
        None
    }
}

/// `Params_CellValueChanged` over the Value cell's new text.
///
/// The C# reads the text with mXparser's `Expression.calculate`, so `2*3` is 6; here the text is
/// read as a number, and an expression is a text that is no number - the red cell.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:454-533`
#[must_use]
pub fn cell_value_changed(
    name: &str,
    text: &str,
    read_only: Option<&str>,
    range: Option<(f64, f64)>,
) -> Edit {
    // An RC or HS reverse of 0 is -1.
    let text = if name.ends_with("_REV")
        && (name.starts_with("RC") || name.starts_with("HS"))
        && text == "0"
    {
        "-1"
    } else {
        text
    };
    // `new Expression(value).calculate()`: arithmetic, not only a number; NaN or infinity
    // throws in the C#, which is the refusal here.
    let Some(value) = calculate(&text.replace(',', ".")) else {
        return Edit::Invalid;
    };
    if let Some(read_only) = read_only.filter(|text| !text.is_empty()) {
        match bool_parse(read_only) {
            Some(true) => return Edit::ReadOnly,
            Some(false) => {}
            None => return Edit::Invalid,
        }
    }
    if let Some((min, max)) = range
        && (value > max || value < min)
    {
        return Edit::OutOfRange(value);
    }
    Edit::Write(value)
}

/// The Value cell's text: the value waiting in `_changes` as `newvalue.ToString()`, else the
/// vehicle's as `MAVLinkParam.ToString()` - a REAL32's `float.ToString()`, which for a whole
/// number of an integer type reads as `double.ToString()` does.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:594, 523; ExtLibs/Mavlink/MAVLinkParam.cs:219-224`
#[must_use]
pub fn value_text(parameter: &Parameter, changes: &BTreeMap<String, f64>) -> String {
    match changes.get(&parameter.name) {
        Some(value) => mp_log::netfmt::double(*value),
        #[allow(clippy::cast_possible_truncation)] // a REAL32 on the wire
        None => mp_log::netfmt::single(parameter.value as f32),
    }
}

// --- The Options control -------------------------------------------------------------------------

/// `NumericUpDown` over a documented range.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1270-1316`
#[derive(Debug, Clone, PartialEq)]
pub struct Numeric {
    /// `Minimum`, rounded to the places.
    pub minimum: f64,
    /// `Maximum`, rounded to the places.
    pub maximum: f64,
    /// `Increment`, rounded to the places.
    pub increment: f64,
    /// `DecimalPlaces`.
    pub places: usize,
    /// `Value`.
    pub value: f64,
}

/// `Math.Round(value, places)`: to even at the half.
fn round_to(value: f64, places: usize) -> f64 {
    let scale = 10f64.powi(i32::try_from(places).unwrap_or(0));
    (value * scale).round_ties_even() / scale
}

/// `decimal.TryParse(text, out val)`: a plain decimal number - a sign, digits and a point, no
/// exponent - or 0, which is what the `out` holds after a failed parse.
fn decimal_try_parse(text: &str) -> f64 {
    let text = text.trim();
    let digits = text.strip_prefix(['-', '+']).unwrap_or(text);
    let plain = !digits.is_empty()
        && digits.chars().all(|c| c.is_ascii_digit() || c == '.')
        && digits.chars().filter(|c| *c == '.').count() <= 1
        && digits.chars().any(|c| c.is_ascii_digit());
    if plain {
        text.parse().unwrap_or(0.0)
    } else {
        0.0
    }
}

impl Numeric {
    /// The `NumericUpDown` `Params_RowEnter` makes: the increment the documentation's, else the
    /// range over a thousand rounded down to a power of ten; the places from the increment, or
    /// from the minimum when it is smaller and not zero; the cell's value clamped into the range.
    ///
    /// `None` where the C# would set places WinForms refuses - an increment or a range of zero -
    /// and throw out of `RowEnter`.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1272-1299; ExtLibs/Utilities/ParameterMetaDataRepository.cs:178-188`
    #[must_use]
    pub fn new(range: (f64, f64), increment: Option<f64>, cell: &str) -> Option<Self> {
        let (min, max) = range;
        #[allow(clippy::cast_possible_truncation)] // `float.TryParse`
        let inc = increment.map_or_else(
            || 10f64.powf(((max - min) / 1000.0).log10().floor()),
            |inc| f64::from(inc as f32),
        );
        let places_of = |of: f64| (-of.abs().log10()).max(0.0).round_ties_even();
        let mut places = places_of(inc);
        if min.abs() < inc && min.abs() >= 1e-9 {
            places = places_of(min);
        }
        if !places.is_finite() || !(0.0..=99.0).contains(&places) {
            return None;
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // 0 to 99
        let places = places as usize;
        let minimum = round_to(min, places);
        let maximum = round_to(max, places);
        let value = decimal_try_parse(cell).min(maximum).max(minimum);
        Some(Self {
            minimum,
            maximum,
            increment: round_to(inc, places),
            places,
            value: round_to(value, places),
        })
    }

    /// `Text`: the value to its places, `ToString("F" + DecimalPlaces)`.
    #[must_use]
    pub fn text(&self) -> String {
        let value = if self.value == 0.0 { 0.0 } else { self.value };
        format!("{value:.*}", self.places)
    }

    /// The up or down arrow: `Value` moved by `Increment`, held at `Maximum` or `Minimum`. The
    /// new text when it changed - `TextChanged`, which puts it in the Value cell - else `None`.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1302-1306`
    pub fn step(&mut self, up: bool) -> Option<String> {
        let before = self.text();
        self.value = if up {
            (self.value + self.increment).min(self.maximum)
        } else {
            (self.value - self.increment).max(self.minimum)
        };
        let after = self.text();
        (after != before).then_some(after)
    }
}

/// `MavlinkCheckBoxBitMask` in `ShowUserControl`'s window: the display name, the description
/// and a box per documented bit.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1174-1200; Controls/MavlinkCheckBoxBitMask.cs:74-141`
#[derive(Debug, Clone, PartialEq)]
pub struct BitmaskWindow {
    /// `ParamName`.
    pub param: String,
    /// `myLabel1`: the display name.
    pub title: String,
    /// `label1`: the description.
    pub description: String,
    /// Each bit, its text, and whether it is ticked.
    pub bits: Vec<(u32, String, bool)>,
}

impl BitmaskWindow {
    /// `setup` over the Value cell's number, `double.Parse(cell, InvariantCulture)` - `None`
    /// when it is none, which throws in the C#'s click handler and opens nothing.
    #[must_use]
    pub fn new(param: &str, cell: &str, meta: &ParamMeta) -> Option<Self> {
        let value: f64 = cell.trim().parse().ok()?;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // `(uint)`
        let value = value as u32;
        Some(Self {
            param: param.to_owned(),
            title: meta.display_name.to_owned(),
            description: meta.description.to_owned(),
            bits: meta
                .bitmask
                .iter()
                .map(|(bit, text)| (*bit, (*text).to_owned(), value & (1 << (bit & 31)) > 0))
                .collect(),
        })
    }

    /// `Value.ToString()`: each ticked bit added up in a `float`, then `(int)` - the type the C#
    /// assumes when the vehicle's table has no type for the name, as this one's never does.
    /// `// C#: Controls/MavlinkCheckBoxBitMask.cs:24-51; ConfigRawParams.cs:1183-1187`
    #[must_use]
    pub fn value_text(&self) -> String {
        #[allow(clippy::cast_precision_loss)] // the C#'s float
        let sum: f32 = self
            .bits
            .iter()
            .filter(|(.., ticked)| *ticked)
            .map(|(bit, ..)| (1_u32 << (bit & 31)) as f32)
            .sum();
        #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)] // `(int)answer`
        let value = sum as i32 as f32;
        mp_log::netfmt::single(value)
    }
}

/// The control `Params_RowEnter` puts over the entered row's Options cell.
#[derive(Debug, Clone, PartialEq)]
pub enum OptionsControl {
    /// A "Set Bitmask" button.
    Bitmask,
    /// A `DropDownList` of the documented values.
    Values {
        /// The list, its selection the cell's value when it is one of them.
        combo: Combo,
        /// Dropped down.
        open: bool,
    },
    /// A `NumericUpDown` over the range.
    Range(Numeric),
}

/// `Params_RowEnter`: a bitmask gets its button, else a list of values its drop-down, else a
/// range its `NumericUpDown`, else nothing.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1153-1320`
#[must_use]
pub fn options_control(name: &str, cell: &str, meta: Option<&ParamMeta>) -> Option<OptionsControl> {
    let meta = meta?;
    if !meta.bitmask.is_empty() {
        return Some(OptionsControl::Bitmask);
    }
    if !meta.values.is_empty() {
        let options: Vec<(i64, String)> = meta
            .values
            .iter()
            .map(|(code, text)| (*code, text.trim().to_owned()))
            .collect();
        // `int.TryParse(cell)`: `SelectedValue = val`, which selects nothing for a value the
        // list does not hold; else `SelectedIndex = -1`.
        let selected = cell
            .trim()
            .parse::<i32>()
            .ok()
            .map(i64::from)
            .filter(|value| options.iter().any(|(code, _)| code == value));
        return Some(OptionsControl::Values {
            combo: Combo {
                param: name.to_owned(),
                options,
                selected,
                enabled: true,
                top_index: 0,
            },
            open: false,
        });
    }
    let range = meta.range?;
    Numeric::new(range, meta.increment, cell).map(OptionsControl::Range)
}

// --- Write Params -------------------------------------------------------------------------------

/// How many changes the question lists one by one. `// C#: ConfigRawParams.cs:268`
pub const MAX_DISPLAY: usize = 20;
/// The question's caption. `// C#: ConfigRawParams.cs:298, 306`
pub const CONFIRM_CAPTION: &str = "Confirm Parameter Changes";
/// Said with nothing to write, under "No changes". `// C#: ConfigRawParams.cs:370-371`
pub const NO_CHANGES: (&str, &str) = ("No changes", "No parameters were changed.");
/// Said when a parameter written needs a reboot. `// C#: ConfigRawParams.cs:374-377`
pub const REBOOT_REQUIRED: (&str, &str) = (
    "Reboot Required",
    "Reboot is required for some parameters to take effect.",
);
/// Said when the count of parameters changed, armed. `// C#: ConfigRawParams.cs:381-386`
pub const COUNT_CHANGED_ARMED: (&str, &str) = (
    "Params",
    "The number of available parameters changed, until full param refresh is done, some parameters will not be available.",
);
/// Said when the count of parameters changed, disarmed; the list is fetched after it.
/// `// C#: ConfigRawParams.cs:387-392`
pub const COUNT_CHANGED: (&str, &str) = (
    "Params",
    "The number of available parameters changed. A full param refresh will be done to show all params.",
);
/// The first loop's refusal with no link. `// C#: ConfigRawParams.cs:277-281`
pub const NOT_CONNECTED: &str = "You are not connected";
/// The second loop's. `// C#: ConfigRawParams.cs:317-321`
pub const NOT_CONNECTED_2: &str = "Your are not connected";

/// `_changes`' names as `BUT_writePIDS_Click` orders them, `SortENABLE`.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:260-262`
#[must_use]
pub fn save_order(changes: &BTreeMap<String, f64>) -> Vec<String> {
    let mut names: Vec<String> = changes.keys().cloned().collect();
    crate::config::adsb::sort_enable(&mut names);
    names
}

/// The question before the writes: each change as `NAME: previous -> new` when there are at
/// most twenty, else only how many. `None` with nothing to write, which asks nothing. The
/// previous value is the vehicle's `MAVLinkParam.ToString()`, the new `double.ToString()`.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:270-310`
#[must_use]
pub fn save_question(
    names: &[String],
    changes: &BTreeMap<String, f64>,
    held: impl Fn(&str) -> Option<f64>,
) -> Option<String> {
    if names.is_empty() {
        return None;
    }
    if names.len() > MAX_DISPLAY {
        return Some(format!(
            "You are about to change {} parameters. Are you sure you want to proceed?",
            names.len()
        ));
    }
    let lines: Vec<String> = names
        .iter()
        .map(|name| {
            #[allow(clippy::cast_possible_truncation)] // a REAL32 on the wire
            let previous =
                held(name).map_or_else(String::new, |value| mp_log::netfmt::single(value as f32));
            let new = changes
                .get(name)
                .map_or_else(String::new, |value| mp_log::netfmt::double(*value));
            format!("{name}: {previous} -> {new}")
        })
        .collect();
    Some(format!(
        "You are about to change {} parameters. Please review the changes below:\n\n{}\n\nDo you want to proceed?",
        names.len(),
        lines.join("\n")
    ))
}

/// Write Params under way: how many it writes, the names not yet heard back, those whose
/// documentation asks for a reboot, and whether one of those was written.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Saving {
    total: usize,
    waiting: BTreeSet<String>,
    reboot_names: BTreeSet<String>,
    reboot: bool,
}

impl Saving {
    /// A write ended: `GetParameterRebootRequired` is asked only after a `setParam` that did not
    /// throw.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:323-328`
    pub fn written(&mut self, written: &Written) {
        if self.waiting.remove(&written.name)
            && written.outcome != RequestOutcome::TimedOut
            && self.reboot_names.contains(&written.name)
        {
            self.reboot = true;
        }
    }

    /// Whether every write has been heard back.
    #[must_use]
    pub fn done(&self) -> bool {
        self.waiting.is_empty()
    }
}

// --- The state -----------------------------------------------------------------------------------

/// What a box is for, and what its answer does.
#[derive(Debug, Clone, PartialEq)]
pub enum BoxKind {
    /// A box with OK that does nothing more.
    Notice,
    /// The out-of-range question: Yes writes the value.
    OutOfRange {
        /// The parameter.
        name: String,
        /// The value asked for.
        value: f64,
    },
    /// Write Params' question: Yes writes these.
    Confirm {
        /// The names, in their order.
        names: Vec<String>,
    },
    /// "A full param refresh will be done": OK fetches the list.
    Refresh,
}

/// A box over the window: its caption, its text, and what it is for.
#[derive(Debug, Clone, PartialEq)]
pub struct GridBox {
    /// The caption.
    pub caption: String,
    /// The text.
    pub text: String,
    /// What it is for.
    pub kind: BoxKind,
}

impl GridBox {
    fn notice(caption: &str, text: impl Into<String>) -> Self {
        Self {
            caption: caption.to_owned(),
            text: text.into(),
            kind: BoxKind::Notice,
        }
    }

    /// Whether it asks Yes or No.
    #[must_use]
    pub const fn is_question(&self) -> bool {
        matches!(
            self.kind,
            BoxKind::OutOfRange { .. } | BoxKind::Confirm { .. }
        )
    }
}

/// `Strings.RawParamWarning`, the box's title. `// C#: ExtLibs/Strings/Strings.resx:434-436`
pub const RAW_PARAM_WARNING: &str = "Raw Param Warning";
/// `Strings.RawParamWarningi`, its text. `// C#: ExtLibs/Strings/Strings.resx:437-440`
pub const RAW_PARAM_WARNING_TEXT: &str = "All values on this screen are not min/max checked. Please double check your input.\nPlease use Standard/Advanced Params for the safe settings";
/// `MessageShowAgain`'s key for it: `SHOWAGAIN_` and the title with its spaces made underscores.
/// `// C#: Common.cs:264-268`
pub const WARNING_KEY: &str = "SHOWAGAIN_Raw_Param_Warning";

/// Refresh Params' question on an armed vehicle: `MessageShowAgain("Refresh Params",
/// Strings.WarningUpdateParamList, true)`, its tick kept under `SHOWAGAIN_Refresh_Params` - the
/// key ADSB's and the Standard and Advanced pages' Refresh share, as the title is the tag.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:426-427; ExtLibs/Strings/Strings.resx:221-223`
pub const REFRESH_TITLE: &str = "Refresh Params";
/// `Strings.WarningUpdateParamList`, with the newline the `.resx` value ends in.
pub const REFRESH_TEXT: &str = "Update Params\nDON'T DO THIS IF YOU ARE IN THE AIR\n";

/// The grid's state between frames.
#[derive(Debug, Default)]
pub struct RawGrid {
    /// The Fav cells ticked: from `fav_params` when the rows are made, then as clicked.
    favourites: BTreeSet<String>,
    /// The column sorted by and its direction.
    sort: Sort,
    /// The widths and the splitter.
    layout: Layout,
    /// A header edge or the splitter being dragged.
    drag: Option<Drag>,
    /// The Value cell being typed into: its row's name and the text.
    editing: Option<(String, TextField)>,
    /// Rows whose Value cell is red: the last edit's text was no number.
    red: BTreeSet<String>,
    /// The entered row's Options control, and its row.
    control: Option<(String, OptionsControl)>,
    /// The row the control was made for, `None` for none: the row last entered.
    entered: Option<String>,
    /// Set Bitmask's window.
    bitmask: Option<BitmaskWindow>,
    /// The boxes, the first showing.
    boxes: VecDeque<GridBox>,
    /// RawParamWarning showing, and its "Show me again?" tick.
    warning: Option<bool>,
    /// Refresh Params' armed-only question showing, and its "Show me again?" tick.
    refresh: Option<bool>,
    /// Write Params under way.
    saving: Option<Saving>,
    /// The last address a Desc cell opened, for the facts.
    opened: Option<String>,
}

impl RawGrid {
    /// `Activate`'s part: the widths and the splitter from the settings, the Fav cells from
    /// `fav_params` as `processToScreen` makes the rows, anything being typed dropped, and
    /// RawParamWarning shown unless its "Show me again?" was unticked.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:70-92, 595-596; Common.cs:260-270`
    pub fn activate<'a>(&mut self, get: impl Fn(&str) -> Option<&'a str>) {
        self.layout.restore(&get);
        self.favourites = get_list(get(FAV_KEY)).into_iter().collect();
        self.editing = None;
        self.red.clear();
        self.control = None;
        self.entered = None;
        self.drag = None;
        // `ContainsKey(key) && GetBoolean(key) == false`: a value `bool.TryParse` refuses is
        // false too.
        let suppressed =
            get(WARNING_KEY).is_some_and(|value| !crate::raw_params::get_boolean(Some(value)));
        self.warning = (!suppressed).then_some(true);
    }

    /// `Deactivate`'s part: the settings to keep.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:99-112`
    pub fn deactivate(&mut self) -> Vec<(String, String)> {
        self.drag = None;
        self.editing = None;
        self.bitmask = None;
        self.layout.saved()
    }

    /// The widths and the splitter.
    #[must_use]
    pub const fn layout(&self) -> &Layout {
        &self.layout
    }

    /// How the rows are sorted.
    #[must_use]
    pub const fn sort(&self) -> Sort {
        self.sort
    }

    /// A column header clicked.
    pub fn click_header(&mut self, column: Column) {
        self.sort.click(column);
    }

    /// The Fav cells ticked.
    #[must_use]
    pub const fn favourites(&self) -> &BTreeSet<String> {
        &self.favourites
    }

    /// The row being typed into.
    #[must_use]
    pub fn editing(&self) -> Option<(&str, &TextField)> {
        self.editing
            .as_ref()
            .map(|(name, field)| (name.as_str(), field))
    }

    /// Whether a row's Value cell is red.
    #[must_use]
    pub fn is_red(&self, name: &str) -> bool {
        self.red.contains(name)
    }

    /// The entered row's control.
    #[must_use]
    pub fn control(&self) -> Option<(&str, &OptionsControl)> {
        self.control
            .as_ref()
            .map(|(name, control)| (name.as_str(), control))
    }

    /// The first box waiting: the one showing once RawParamWarning ([`RawGrid::warning`]) is
    /// closed.
    #[must_use]
    pub fn front(&self) -> Option<&GridBox> {
        self.boxes.front()
    }

    /// RawParamWarning's "Show me again?", while it shows.
    #[must_use]
    pub const fn warning(&self) -> Option<bool> {
        self.warning
    }

    /// Refresh Params' question's "Show me again?", while it shows.
    #[must_use]
    pub const fn refresh(&self) -> Option<bool> {
        self.refresh
    }

    /// `BUT_rerequestparams_Click` up to the fetch: nothing without a link; on a vehicle that is
    /// not armed, the fetch; armed, the question - unless its tick was cleared, when
    /// `MessageShowAgain` is OK at once. Returns whether to fetch now.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:420-428; Common.cs:267-270`
    pub fn press_refresh<'a>(
        &mut self,
        open: bool,
        armed: bool,
        get: impl Fn(&str) -> Option<&'a str>,
    ) -> bool {
        if !open {
            return false;
        }
        let suppressed = get(crate::config::adsb::SHOW_AGAIN_KEY)
            .is_some_and(|value| !crate::raw_params::get_boolean(Some(value)));
        if !armed || suppressed {
            return true;
        }
        self.refresh = Some(true);
        false
    }

    /// The question's "Show me again?" clicked: the setting's new value, `Checked.ToString()`.
    /// `// C#: Common.cs:446-449`
    pub fn toggle_refresh(&mut self) -> Option<&'static str> {
        let ticked = self.refresh.as_mut()?;
        *ticked = !*ticked;
        Some(if *ticked { "True" } else { "False" })
    }

    /// The question answered: OK fetches, Cancel does not.
    pub fn answer_refresh(&mut self, ok: bool) -> bool {
        self.refresh.take().is_some() && ok
    }

    /// Set Bitmask's window.
    #[must_use]
    pub const fn bitmask(&self) -> Option<&BitmaskWindow> {
        self.bitmask.as_ref()
    }

    /// A header edge or the splitter pressed.
    pub fn begin_drag(&mut self, what: Dragged, at: f32) {
        let start = match what {
            Dragged::Column(column) => self.layout.width(column),
            Dragged::Splitter => self.layout.splitter(),
        };
        self.drag = Some(Drag {
            what,
            from: at,
            start,
        });
    }

    /// The pointer moved with the button down: the width or the distance follows it. `total` is
    /// the split's width, which the splitter stays inside.
    pub fn drag_to(&mut self, at: f32, total: f32) -> bool {
        let Some(drag) = self.drag else {
            return false;
        };
        #[allow(clippy::cast_possible_truncation)] // pixels
        let moved = (at - drag.from).round() as i32;
        match drag.what {
            Dragged::Column(column) => self.layout.set_width(column, drag.start + moved),
            #[allow(clippy::cast_possible_truncation)] // pixels
            Dragged::Splitter => self
                .layout
                .set_splitter(drag.start + moved, total.round() as i32),
        }
        true
    }

    /// The button let go.
    pub fn end_drag(&mut self) {
        self.drag = None;
    }

    /// Whether something is being dragged.
    #[must_use]
    pub const fn dragging(&self) -> bool {
        self.drag.is_some()
    }

    /// The Fav cell clicked: ticked or unticked, and the setting's new value if it changes.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1045-1063`
    pub fn click_fav(&mut self, name: &str, setting: Option<&str>) -> Option<String> {
        let ticked = !self.favourites.contains(name);
        if ticked {
            self.favourites.insert(name.to_owned());
        } else {
            self.favourites.remove(name);
        }
        fav_clicked(setting, name, ticked)
    }

    /// `Params_RowEnter`: the control for a row entered, made from its cell as it is now; the
    /// row left behind loses its own. Nothing is made again for the row already entered.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1153-1170`
    pub fn enter_row(&mut self, name: Option<&str>, cell: &str, meta: Option<&ParamMeta>) {
        if self.entered.as_deref() == name {
            return;
        }
        self.entered = name.map(str::to_owned);
        self.control = name.and_then(|name| {
            options_control(name, cell, meta).map(|control| (name.to_owned(), control))
        });
    }

    /// Typing begins in a row's Value cell, with this text.
    pub fn begin_edit(&mut self, name: &str, text: &str) {
        let mut field = TextField::new("");
        field.set(text);
        self.editing = Some((name.to_owned(), field));
    }

    /// A key in the Value cell being typed into: the row and its text when Enter ends it.
    pub fn edit_key(&mut self, event: &KeyDownEvent) -> (KeyOutcome, Option<(String, String)>) {
        let Some((_, field)) = &mut self.editing else {
            return (KeyOutcome::Ignored, None);
        };
        let outcome = field.key(event);
        match outcome {
            KeyOutcome::Submitted => (outcome, self.end_edit()),
            KeyOutcome::Cancelled => {
                self.editing = None;
                (outcome, None)
            }
            _ => (outcome, None),
        }
    }

    /// The typing ended - Enter, or the current cell changed: the row and what was typed.
    pub fn end_edit(&mut self) -> Option<(String, String)> {
        self.editing
            .take()
            .map(|(name, field)| (name, field.value().to_owned()))
    }

    /// An edit's red cell: set by a text that is no number, cleared by one that is.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:518, 526-529`
    fn mark(&mut self, name: &str, red: bool) {
        if red {
            self.red.insert(name.to_owned());
        } else {
            self.red.remove(name);
        }
    }

    /// RawParamWarning's "Show me again?" clicked: the setting's new value,
    /// `Checked.ToString()`.
    /// `// C#: Common.cs:446-449`
    pub fn toggle_warning(&mut self) -> Option<&'static str> {
        let ticked = self.warning.as_mut()?;
        *ticked = !*ticked;
        Some(if *ticked { "True" } else { "False" })
    }

    /// RawParamWarning's OK.
    pub fn close_warning(&mut self) {
        self.warning = None;
    }

    /// A write ended: Write Params' count of what is still to hear back.
    pub fn written(&mut self, written: &Written) {
        if let Some(saving) = &mut self.saving {
            saving.written(written);
        }
    }
}

/// What answering the box showing comes to.
#[derive(Debug, Clone, PartialEq)]
pub enum Answered {
    /// Nothing more.
    Nothing,
    /// The out-of-range question's Yes: this written.
    Write(String, f64),
    /// Write Params' Yes: these of `_changes` written.
    WriteChanges(Vec<String>),
    /// The count's box: the list fetched again.
    Refresh,
}

impl RawGrid {
    /// `Params_CellValueChanged` over a Value cell's new text: the value to write when every
    /// check passes; else the ReadOnly box, the out-of-range question or the red cell.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:454-533`
    pub fn edit(
        &mut self,
        name: &str,
        text: &str,
        read_only: Option<&str>,
        range: Option<(f64, f64)>,
    ) -> Option<(String, f64)> {
        match cell_value_changed(name, text, read_only, range) {
            Edit::Write(value) => {
                self.mark(name, false);
                return Some((name.to_owned(), value));
            }
            Edit::ReadOnly => self
                .boxes
                .push_back(GridBox::notice(READ_ONLY_CAPTION, read_only_text(name))),
            Edit::OutOfRange(value) => self.boxes.push_back(GridBox {
                caption: OUT_OF_RANGE_CAPTION.to_owned(),
                text: out_of_range_text(name),
                kind: BoxKind::OutOfRange {
                    name: name.to_owned(),
                    value,
                },
            }),
            Edit::Invalid => self.mark(name, true),
        }
        None
    }

    /// The first box answered: OK and Yes are `true`, No `false`. No to the out-of-range
    /// question keeps the old value, `cellEditValue`; Yes goes on to the write.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:504-516, 298-309`
    pub fn answer(&mut self, yes: bool) -> Answered {
        let Some(answered) = self.boxes.pop_front() else {
            return Answered::Nothing;
        };
        match answered.kind {
            BoxKind::Notice => Answered::Nothing,
            BoxKind::Refresh => Answered::Refresh,
            BoxKind::OutOfRange { name, value } if yes => {
                self.mark(&name, false);
                Answered::Write(name, value)
            }
            BoxKind::Confirm { names } if yes => Answered::WriteChanges(names),
            BoxKind::OutOfRange { .. } | BoxKind::Confirm { .. } => Answered::Nothing,
        }
    }

    /// Ctrl+S, `BUT_writePIDS_Click`, over `_changes`: `Ok(true)` with nothing to write, which
    /// asks nothing and goes straight to its end; `Ok(false)` with the question asked; `Err`
    /// with twenty or fewer and no link, the first loop's refusal - on the status line, as the
    /// owner ruled (2026-09-25) for an error the window shows as state.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:257-310`
    ///
    /// # Errors
    /// The refusal's text.
    pub fn save(
        &mut self,
        changes: &BTreeMap<String, f64>,
        connected: bool,
        held: impl Fn(&str) -> Option<f64>,
    ) -> Result<bool, &'static str> {
        let names = save_order(changes);
        if names.is_empty() {
            return Ok(true);
        }
        if names.len() <= MAX_DISPLAY && !connected {
            return Err(NOT_CONNECTED);
        }
        if let Some(text) = save_question(&names, changes, held) {
            self.boxes.push_back(GridBox {
                caption: CONFIRM_CAPTION.to_owned(),
                text,
                kind: BoxKind::Confirm { names },
            });
        }
        Ok(false)
    }

    /// The question's Yes: the writes, in `SortENABLE`'s order, one after another - each "Set
    /// NAME Failed" if never heard back and the summary after them on the status line, as the
    /// file panel's writes say theirs - and Write Params' end kept for when they are heard back.
    /// A closed port is the second loop's refusal.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:313-364`
    ///
    /// # Errors
    /// The refusal's text.
    pub fn write_changes(
        &mut self,
        names: &[String],
        changes: &BTreeMap<String, f64>,
        connected: bool,
        reboot: impl Fn(&str) -> bool,
    ) -> Result<Vec<(String, f64)>, &'static str> {
        if !connected {
            return Err(NOT_CONNECTED_2);
        }
        let writes: Vec<(String, f64)> = names
            .iter()
            .filter_map(|name| Some((name.clone(), *changes.get(name)?)))
            .collect();
        self.saving = Some(Saving {
            total: writes.len(),
            waiting: writes.iter().map(|(name, _)| name.clone()).collect(),
            reboot_names: writes
                .iter()
                .map(|(name, _)| name.clone())
                .filter(|name| reboot(name))
                .collect(),
            reboot: false,
        });
        Ok(writes)
    }

    /// Write Params over, when every write is heard back or none is running any more.
    pub fn take_saved(&mut self, writes_running: bool) -> Option<Saving> {
        let done = self
            .saving
            .as_ref()
            .is_some_and(|saving| saving.done() || !writes_running);
        if done { self.saving.take() } else { None }
    }

    /// Write Params' end: "No parameters were changed." when there was nothing, the reboot's
    /// box when a parameter written asks for one, and the count's box when the vehicle's count
    /// of parameters is not the count held - armed, only said (the C#'s `TotalReported` made
    /// `TotalReceived` is not ported: the count here is the vehicle's own, which its next
    /// `PARAM_VALUE` sets); disarmed, said and the list fetched again after OK.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:366-393`
    pub fn saved(&mut self, saving: &Saving, held: usize, expected: usize, armed: bool) {
        // `temp.Count == 0`; the other two ends, "N parameters successfully saved." and "Not all
        // parameters successfully saved.", are the writes' summary on the status line.
        if saving.total == 0 {
            self.boxes
                .push_back(GridBox::notice(NO_CHANGES.0, NO_CHANGES.1));
        }
        if saving.reboot {
            self.boxes
                .push_back(GridBox::notice(REBOOT_REQUIRED.0, REBOOT_REQUIRED.1));
        }
        if expected > 0 && held != expected {
            if armed {
                self.boxes.push_back(GridBox::notice(
                    COUNT_CHANGED_ARMED.0,
                    COUNT_CHANGED_ARMED.1,
                ));
            } else {
                self.boxes.push_back(GridBox {
                    caption: COUNT_CHANGED.0.to_owned(),
                    text: COUNT_CHANGED.1.to_owned(),
                    kind: BoxKind::Refresh,
                });
            }
        }
    }
}

// --- The screen's part in the application -------------------------------------------------------
impl MissionPlanner {
    /// `Activate`: the grid's part, run by `raw_params_tick` as the screen comes into view.
    pub(crate) fn param_grid_activate(&mut self) {
        let persisted = &self.persisted;
        self.param_grid.activate(|key| persisted.get(key));
    }

    /// `Deactivate`: the widths and the splitter into the settings.
    pub(crate) fn param_grid_deactivate(&mut self) {
        for (key, value) in self.param_grid.deactivate() {
            self.persisted.set(&key, value);
        }
    }

    /// Once a frame: the entered row's control made when the row changes, and Write Params'
    /// end when its last write is heard back.
    pub(crate) fn param_grid_tick(&mut self) {
        let running = !self.param_writes.is_empty();
        if let Some(saving) = self.param_grid.take_saved(running) {
            self.param_grid_saved(&saving);
        }
        let selected = self.selected_param.clone();
        if !self.raw_params.is_active() || self.param_grid.entered.as_deref() == selected.as_deref()
        {
            return;
        }
        let view = self.telemetry.view();
        let parameter = selected.as_deref().and_then(|name| {
            view.parameters
                .iter()
                .find(|(held, _)| held == name)
                .map(|(name, value)| Parameter {
                    name: name.clone(),
                    value: *value,
                    meta: crate::metadata::lookup(name),
                    default: None,
                })
        });
        let cell = parameter
            .as_ref()
            .map(|parameter| value_text(parameter, self.raw_params.changes()))
            .unwrap_or_default();
        self.param_grid.enter_row(
            parameter.as_ref().map(|parameter| parameter.name.as_str()),
            &cell,
            parameter.as_ref().and_then(|parameter| parameter.meta),
        );
    }

    /// `Params_CellValueChanged`: a Value cell's new text - typed, or put there by the Options
    /// control - checked, and written when it passes.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:454-533`
    pub(crate) fn param_grid_edit(&mut self, name: &str, text: &str) {
        let range = crate::metadata::lookup(name).and_then(|meta| meta.range);
        let read_only = crate::metadata::read_only(name);
        if let Some((name, value)) = self
            .param_grid
            .edit(name, text, read_only.as_deref(), range)
        {
            self.param_grid_write(&name, value);
        }
    }

    /// An edit that passed: `_changes[name] = newvalue`, which on this screen is the write.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:518-524`
    fn param_grid_write(&mut self, name: &str, value: f64) {
        self.file_status = Some(format!("{name} = {value}"));
        self.start_param_writes(crate::params::ParamWrites::nudge(name, value));
    }

    /// The box showing answered: OK, Yes or No.
    pub(crate) fn param_grid_answer(&mut self, yes: bool) {
        match self.param_grid.answer(yes) {
            Answered::Nothing => {}
            Answered::Refresh => self.telemetry.download_parameters(),
            Answered::Write(name, value) => self.param_grid_write(&name, value),
            Answered::WriteChanges(names) => {
                let connected = self.telemetry.view().connected;
                let reboot = |name: &str| {
                    crate::metadata::lookup(name).is_some_and(|meta| meta.reboot_required)
                };
                match self.param_grid.write_changes(
                    &names,
                    self.raw_params.changes(),
                    connected,
                    reboot,
                ) {
                    Ok(writes) => {
                        self.start_param_writes(crate::params::ParamWrites::apply(writes, 0));
                    }
                    Err(said) => self.file_status = Some(crate::fly::error_box(said)),
                }
            }
        }
    }

    /// `ProcessCmdKey`'s Ctrl+S: `BUT_writePIDS_Click` over `_changes`.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:116-125, 257-310`
    pub(crate) fn param_grid_save(&mut self) {
        let view = self.telemetry.view();
        let held = |name: &str| {
            view.parameters
                .iter()
                .find(|(held, _)| held == name)
                .map(|(_, value)| *value)
        };
        match self
            .param_grid
            .save(self.raw_params.changes(), view.connected, held)
        {
            Ok(true) => self.param_grid_saved(&Saving::default()),
            Ok(false) => {}
            Err(said) => self.file_status = Some(crate::fly::error_box(said)),
        }
    }

    /// Write Params' end, with the vehicle's count of parameters and whether it is armed.
    fn param_grid_saved(&mut self, saving: &Saving) {
        let view = self.telemetry.view();
        let armed = view.state.as_ref().is_some_and(|state| state.armed);
        self.param_grid.saved(
            saving,
            view.parameters.len(),
            usize::from(view.parameters_expected),
            armed,
        );
    }

    /// A Desc cell clicked: the first address in it opened, `CheckForUrlAndLaunchInBrowser`.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1033-1043, 1066-1091`
    pub(crate) fn param_grid_desc(&mut self, text: &str, cx: &mut Context<Self>) {
        if let Some(url) = first_url(text) {
            self.param_grid.opened = Some(url.to_owned());
            // A test run opens no browser; the fact says what would have been opened.
            if !crate::facts::enabled() {
                cx.open_url(url);
            }
        }
    }

    /// The Fav cell clicked, the setting changed as the C# changes it.
    fn param_grid_fav(&mut self, name: &str) {
        let setting = self.persisted.get(FAV_KEY).map(str::to_owned);
        if let Some(value) = self.param_grid.click_fav(name, setting.as_deref()) {
            self.persisted.set(FAV_KEY, value);
        }
    }

    /// Set Bitmask pressed: the window over the entered row's value.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1176-1200`
    fn param_grid_open_bitmask(&mut self, name: &str) {
        let view = self.telemetry.view();
        let Some(parameter) =
            view.parameters
                .iter()
                .find(|(held, _)| held == name)
                .map(|(name, value)| Parameter {
                    name: name.clone(),
                    value: *value,
                    meta: crate::metadata::lookup(name),
                    default: None,
                })
        else {
            return;
        };
        let Some(meta) = parameter.meta else {
            return;
        };
        let cell = value_text(&parameter, self.raw_params.changes());
        self.param_grid.bitmask = BitmaskWindow::new(name, &cell, meta);
    }

    /// A bit's box clicked: `ValueChanged`, the value into the Value cell - the window's own
    /// parameter's, where the C# writes whichever row is current by then.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1192-1197; Controls/MavlinkCheckBoxBitMask.cs:143-150`
    fn param_grid_click_bit(&mut self, bit: usize) {
        let Some(window) = &mut self.param_grid.bitmask else {
            return;
        };
        let Some((.., ticked)) = window.bits.get_mut(bit) else {
            return;
        };
        *ticked = !*ticked;
        let (name, text) = (window.param.clone(), window.value_text());
        self.param_grid_edit(&name, &text);
    }

    /// The `NumericUpDown`'s arrow: its text into the Value cell when it changed.
    fn param_grid_step(&mut self, up: bool) {
        let Some((name, OptionsControl::Range(numeric))) = &mut self.param_grid.control else {
            return;
        };
        if let Some(text) = numeric.step(up) {
            let name = name.clone();
            self.param_grid_edit(&name, &text);
        }
    }

    /// A value chosen from the drop-down: `SelectedValue.ToString()` into the Value cell.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1262-1266`
    fn param_grid_choose(&mut self, key: i64) {
        let Some((name, OptionsControl::Values { combo, open })) = &mut self.param_grid.control
        else {
            return;
        };
        *open = false;
        if combo.select(key) {
            let name = name.clone();
            self.param_grid_edit(&name, &key.to_string());
        }
    }

    /// A key while the Value cell is typed into: Enter ends the typing and is the edit, Escape
    /// drops it; either gives the grid the keyboard back. Whether the field took the key - a
    /// chord it does not (Ctrl+S) goes on to the screen.
    fn param_grid_edit_key(
        &mut self,
        event: &KeyDownEvent,
        window: &mut Window,
        cx: &mut App,
    ) -> bool {
        let (outcome, done) = self.param_grid.edit_key(event);
        if let Some((name, text)) = done {
            self.param_grid_edit(&name, &text);
        }
        if matches!(outcome, KeyOutcome::Submitted | KeyOutcome::Cancelled) {
            window.focus(&self.param_grid_focus, cx);
        }
        outcome != KeyOutcome::Ignored
    }

    /// The facts a script reads; `none` for nothing.
    pub(crate) fn param_grid_facts(&self, parameters: &[Parameter]) {
        use crate::facts::record;
        let or_none = |text: &str| {
            if text.is_empty() {
                "none".to_owned()
            } else {
                text.to_owned()
            }
        };
        let grid = &self.param_grid;
        for column in Column::ALL.into_iter().filter(|column| column.kept()) {
            record(
                format!("params.width.{}", column.name()),
                grid.layout.width(column),
            );
        }
        record("params.splitter", grid.layout.splitter());
        // The settings as the dictionary holds them.
        let mut keys: Vec<String> = Column::ALL
            .into_iter()
            .filter(|column| column.kept())
            .map(Column::key)
            .collect();
        keys.extend([SPLITTER_KEY, FAV_KEY, WARNING_KEY].map(str::to_owned));
        for key in keys {
            record(
                format!("params.config.{key}"),
                self.persisted.get(&key).unwrap_or("none"),
            );
        }
        let favourites: Vec<&str> = grid.favourites.iter().map(String::as_str).collect();
        record("params.fav", or_none(&favourites.join(",")));
        let filters = crate::params::Filters {
            none_default: self.param_none_default,
            modified: self.raw_params.modified(),
            changes: self.raw_params.changes(),
            collapsed: self.raw_params.collapsed(),
        };
        // The grid's first row, as the grid orders them.
        let order = crate::params::grid_order(
            parameters,
            self.selected_param_group.as_deref(),
            self.param_search.value(),
            &filters,
            &grid.favourites,
            grid.sort,
        )
        .unwrap_or_default();
        record(
            "params.rows.first",
            order
                .first()
                .and_then(|&at| parameters.get(at))
                .map_or("none", |parameter| parameter.name.as_str()),
        );
        // How many rows the grid built for its box, the last time it drew: the rows in view,
        // not every row shown.
        record("params.rows.drawn", crate::params::rows_drawn());
        record("params.sort", self.param_grid.sort().label());
        let selected = self
            .selected_param
            .as_deref()
            .and_then(|name| parameters.iter().find(|parameter| parameter.name == name));
        let selected_cells = cells(selected.and_then(|parameter| parameter.meta));
        record("params.selected.desc", or_none(&selected_cells.desc));
        record("params.selected.options", or_none(&selected_cells.options));
        record("params.selected.units", or_none(&selected_cells.units));
        record(
            "params.selected.value",
            selected.map_or_else(
                || "none".to_owned(),
                |parameter| value_text(parameter, self.raw_params.changes()),
            ),
        );
        record(
            "params.control",
            match grid.control() {
                None => "none",
                Some((_, OptionsControl::Bitmask)) => "bitmask",
                Some((_, OptionsControl::Values { .. })) => "values",
                Some((_, OptionsControl::Range(_))) => "range",
            },
        );
        record(
            "params.control.text",
            match grid.control() {
                Some((_, OptionsControl::Range(numeric))) => numeric.text(),
                Some((_, OptionsControl::Values { combo, .. })) => or_none(combo.text()),
                _ => "none".to_owned(),
            },
        );
        record(
            "params.editing",
            grid.editing().map_or("none", |(name, _)| name),
        );
        let red: Vec<&str> = grid.red.iter().map(String::as_str).collect();
        record("params.red", or_none(&red.join(",")));
        record(
            "params.box",
            grid.front().map_or("none", |shown| shown.caption.as_str()),
        );
        record(
            "params.box.text",
            grid.front().map_or("none", |shown| shown.text.as_str()),
        );
        record("params.warning", grid.warning.is_some());
        record(
            "params.refresh",
            grid.refresh
                .map_or("none", |ticked| if ticked { "ticked" } else { "unticked" }),
        );
        record(
            "params.bitmask",
            grid.bitmask
                .as_ref()
                .map_or("none", |window| window.param.as_str()),
        );
        record("params.saving", grid.saving.is_some());
        record(
            "params.desc.opened",
            grid.opened.as_deref().unwrap_or("none"),
        );
    }
}

// --- Drawing -------------------------------------------------------------------------------------

/// What drawing a row needs besides the parameter.
pub struct GridView<'a> {
    /// The grid's state.
    pub grid: &'a RawGrid,
    /// `_changes`, whose Value cells show the value asked for.
    pub changes: &'a BTreeMap<String, f64>,
    /// The grid's keyboard focus.
    pub grid_focus: &'a FocusHandle,
    /// The Value cell's being typed into.
    pub edit_focus: &'a FocusHandle,
    /// Whether that has the keyboard.
    pub edit_focused: bool,
}

impl MissionPlanner {
    /// What drawing a row needs, from the screen's state as it is now.
    pub(crate) fn param_grid_view(&self, window: &Window) -> GridView<'_> {
        GridView {
            grid: &self.param_grid,
            changes: self.raw_params.changes(),
            grid_focus: &self.param_grid_focus,
            edit_focus: &self.param_edit_focus,
            edit_focused: self.param_edit_focus.is_focused(window),
        }
    }
}

/// A column's width in pixels.
#[allow(clippy::cast_precision_loss)] // widths are a few hundred pixels
fn width(layout: &Layout, column: Column) -> gpui::Pixels {
    px(layout.width(column) as f32)
}

/// The grid's header row: each column's `HeaderText`, the Default column only with defaults, and
/// a handle on each kept column's right edge that drags its width.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.resx; ConfigRawParams.Designer.cs:224-310`
pub fn header(
    layout: &Layout,
    with_defaults: bool,
    sort: Sort,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let mut row = div()
        .flex()
        .py(px(1.0))
        .text_xs()
        .text_color(rgb(theme::DIM));
    for column in Column::ALL {
        if column == Column::DefaultValue && !with_defaults {
            continue;
        }
        // The header's text, with the sort glyph a `DataGridView` draws on the sorted column;
        // a click sorts by the column (the Fav check-box column is `NotSortable`).
        let glyph = if sort.column == column {
            if sort.ascending { " \u{25b2}" } else { " \u{25bc}" }
        } else {
            ""
        };
        let id = format!("param-col-{}", column.name());
        let mut cell = crate::probe::measured(id.clone(), div())
            .id(SharedString::from(id))
            .relative()
            .flex_shrink_0()
            .overflow_hidden()
            .child(format!("{}{glyph}", column.header()));
        if column != Column::Fav {
            cell = cell.cursor_pointer().on_click(cx.listener(
                move |this, _event: &gpui::ClickEvent, _window, cx| {
                    this.param_grid.click_header(column);
                    cx.notify();
                },
            ));
        }
        cell = if column == Column::Desc {
            cell.flex_1().min_w(px(0.0))
        } else {
            cell.w(width(layout, column))
        };
        if column.kept() {
            let id = format!("param-col-{}-edge", column.name());
            cell = cell.child(
                crate::probe::measured(id.clone(), div())
                    .id(SharedString::from(id))
                    .absolute()
                    .top_0()
                    .right_0()
                    .w(px(4.0))
                    .h_full()
                    .bg(rgb(theme::BORDER))
                    .cursor(gpui::CursorStyle::ResizeLeftRight)
                    .on_mouse_down(
                        MouseButton::Left,
                        cx.listener(move |this, event: &gpui::MouseDownEvent, _window, cx| {
                            this.param_grid
                                .begin_drag(Dragged::Column(column), f32::from(event.position.x));
                            cx.stop_propagation();
                        }),
                    ),
            );
        }
        row = row.child(cell);
    }
    row.into_any_element()
}

/// A cell with its `ToolTipText`, when it has one: shown as the log browser's chips show
/// theirs, in the panel's colours.
fn with_tip(cell: gpui::Stateful<gpui::Div>, tip: Option<String>) -> gpui::Stateful<gpui::Div> {
    let Some(tip) = tip else { return cell };
    let tip = SharedString::from(tip);
    cell.tooltip(move |_window, cx| -> gpui::AnyView {
        let tip = tip.clone();
        cx.new(|_| crate::config::rover_tuning::Tip(tip)).into()
    })
}

/// A text cell of a fixed width, as tall as the row, clipped.
fn cell(layout: &Layout, column: Column) -> gpui::Div {
    div()
        .w(width(layout, column))
        .flex_shrink_0()
        .h_full()
        .overflow_hidden()
}

/// One row: Name, Value, Default (with defaults), Units, Options, Desc and Fav, 36 high.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:589-645`
pub fn row(
    parameter: &Parameter,
    view: &GridView<'_>,
    with_defaults: bool,
    selected: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let layout = view.grid.layout();
    let name = parameter.name.clone();
    let texts = cells(parameter.meta);
    let value = value_text(parameter, view.changes);
    // The description, broken into lines, as the Name, Value and Desc cells' tooltip.
    // `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:610-611, 644`
    let tip_text = (!texts.desc.is_empty()).then(|| tooltip_lines(&texts.desc));

    // `Value`: typed into, or the value - green while it waits in `_changes`, red after a text
    // that was no number.
    let value_cell: AnyElement = match view.grid.editing() {
        Some((editing, field)) if editing == name => crate::textfield::text_field(
            "param-value-edit",
            field,
            view.edit_focus,
            view.edit_focused,
            width(layout, Column::Value),
            cx.listener(|this, event: &KeyDownEvent, window, cx| {
                if this.param_grid_edit_key(event, window, cx) {
                    cx.stop_propagation();
                }
                cx.notify();
            }),
        )
        .into_any_element(),
        _ => {
            let pending = view.changes.contains_key(&name);
            let red = view.grid.is_red(&name);
            let mut shown = cell(layout, Column::Value).child(value);
            if red {
                shown = shown.bg(rgb(theme::ALERT)).text_color(rgb(theme::BG));
            } else if pending {
                shown = shown.bg(rgb(theme::OK)).text_color(rgb(theme::BG));
            }
            let clicked = name.clone();
            with_tip(
                crate::probe::measured(format!("param-value-{name}"), shown)
                    .id(SharedString::from(format!("value-{name}")))
                    .cursor_text()
                    .on_click(
                        cx.listener(move |this, event: &gpui::ClickEvent, window, cx| {
                            this.param_grid_select(&clicked, false, window, cx);
                            // A double click begins the edit, as the grid's does.
                            if event.click_count() >= 2 {
                                this.param_grid_begin_edit(None, window, cx);
                            }
                            cx.stop_propagation();
                            cx.notify();
                        }),
                    ),
                tip_text.clone(),
            )
            .into_any_element()
        }
    };

    // `Options`: the control over the entered row's cell, else the cell's text.
    let options_cell: AnyElement = match view.grid.control() {
        Some((entered, control)) if selected && entered == name => {
            options_cell(control, layout, cx)
        }
        // The text form's tooltip lists the values, in columns past fifty.
        // `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:622-642`
        _ => with_tip(
            cell(layout, Column::Options)
                .id(SharedString::from(format!("options-{name}")))
                .text_color(rgb(theme::DIM))
                .child(texts.options.clone()),
            options_tooltip(&texts.options),
        )
        .into_any_element(),
    };

    let fav = view.grid.favourites().contains(&name);
    let fav_name = name.clone();
    let desc_name = name.clone();
    let desc = texts.desc.clone();
    let row_name = name.clone();
    #[allow(clippy::cast_precision_loss)] // pixels
    let desc_minimum = Column::Desc.minimum() as f32;
    crate::probe::measured(format!("param-{name}"), div())
        .id(SharedString::from(format!("row-{name}")))
        .flex()
        .h(px(ROW_HEIGHT))
        .flex_shrink_0()
        .py(px(1.0))
        .text_xs()
        .cursor_pointer()
        .bg(rgb(if selected {
            theme::ACTION
        } else {
            theme::PANEL
        }))
        .text_color(rgb(if selected { theme::ACCENT } else { theme::TEXT }))
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .child(with_tip(
            cell(layout, Column::Command)
                .id(SharedString::from(format!("name-{name}")))
                .child(name.clone()),
            tip_text.clone(),
        ))
        .child(value_cell)
        // `Default_value`: `default_value_to_string`, "NaN" without one.
        // C#: GCSViews/ConfigurationView/ConfigRawParams.cs:598-603
        .children(with_defaults.then(|| {
            cell(layout, Column::DefaultValue)
                .text_color(rgb(if parameter.differs_from_default() {
                    theme::WARN
                } else {
                    theme::DIM
                }))
                .child(parameter.default_shown())
        }))
        .child(
            cell(layout, Column::Units)
                .text_color(rgb(theme::DIM))
                .child(texts.units.clone()),
        )
        .child(options_cell)
        // `Desc`: the fill column; a click opens the first address in it.
        // C#: GCSViews/ConfigurationView/ConfigRawParams.cs:643, 1033-1043
        .child(with_tip(
            crate::probe::measured(format!("param-desc-{name}"), div())
                .id(SharedString::from(format!("desc-{name}")))
                .flex_1()
                .min_w(px(desc_minimum))
                .h_full()
                .overflow_hidden()
                .text_color(rgb(theme::DIM))
                .child(texts.desc)
                .on_click(cx.listener(move |this, _event, window, cx| {
                    this.param_grid_select(&desc_name, false, window, cx);
                    this.param_grid_desc(&desc, cx);
                    cx.stop_propagation();
                    cx.notify();
                })),
            tip_text,
        ))
        // `Fav`: a check box.
        // C#: GCSViews/ConfigurationView/ConfigRawParams.cs:596, 1045-1063
        .child(
            crate::probe::measured(format!("param-fav-{name}"), cell(layout, Column::Fav))
                .id(SharedString::from(format!("fav-{name}")))
                .flex()
                .items_center()
                .justify_center()
                .child(
                    div()
                        .size(px(13.0))
                        .border_1()
                        .border_color(rgb(theme::BORDER))
                        .flex()
                        .items_center()
                        .justify_center()
                        .children(fav.then(|| div().size(px(7.0)).bg(rgb(theme::ACCENT)))),
                )
                .on_click(cx.listener(move |this, _event, window, cx| {
                    this.param_grid_select(&fav_name, false, window, cx);
                    this.param_grid_fav(&fav_name);
                    cx.stop_propagation();
                    cx.notify();
                })),
        )
        .on_click(cx.listener(move |this, _event, window, cx| {
            this.param_grid_select(&row_name, true, window, cx);
            cx.notify();
        }))
        .into_any_element()
}

/// The entered row's Options control, over its cell.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1174-1316`
fn options_cell(
    control: &OptionsControl,
    layout: &Layout,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    #[allow(clippy::cast_precision_loss)] // pixels
    let w = layout.width(Column::Options) as f32;
    match control {
        // `new MyButton() { Text = "Set Bitmask" }`.
        OptionsControl::Bitmask => cell(layout, Column::Options)
            .flex()
            .items_center()
            .child(action(
                "param-options-bitmask",
                SET_BITMASK,
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    if let Some(name) = this.param_grid.control().map(|(name, _)| name.to_owned()) {
                        this.param_grid_open_bitmask(&name);
                    }
                    cx.notify();
                }),
            ))
            .into_any_element(),
        // The `DropDownList`, its list dropped over the rows below.
        OptionsControl::Values { combo, open } => {
            let height = 22.0;
            let mut holder = div()
                .relative()
                .w(px(w))
                .h(px(height))
                .flex_shrink_0()
                .child(servo_output::combo_box(
                    "param-options-combo".to_owned(),
                    combo,
                    (0.0, 0.0, w, height),
                    |this: &mut MissionPlanner| {
                        if let Some((_, OptionsControl::Values { combo, open })) =
                            &mut this.param_grid.control
                        {
                            *open = !*open;
                            if *open {
                                combo.open_list();
                            }
                        }
                    },
                    cx,
                ));
            if *open {
                holder = holder.child(servo_output::dropdown(
                    "param-options-combo",
                    combo,
                    (0.0, height, w),
                    |this: &mut MissionPlanner, key| this.param_grid_choose(key),
                    |this: &mut MissionPlanner, lines| {
                        if let Some((_, OptionsControl::Values { combo, .. })) =
                            &mut this.param_grid.control
                        {
                            combo.scroll_list(lines);
                        }
                    },
                    cx,
                ));
            }
            holder.into_any_element()
        }
        // The `NumericUpDown`: its text and its arrows. Its own box is typed into: every
        // `TextChanged`, key by key, is the Value cell's text, so a click in the box begins the
        // Value cell's edit with the box's text, and what is typed shows there.
        // C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1281-1314
        OptionsControl::Range(numeric) => cell(layout, Column::Options)
            .flex()
            .items_center()
            .gap_1()
            .child({
                let text = numeric.text();
                crate::probe::measured("param-options-number", div())
                    .id("param-options-number")
                    .flex_1()
                    .px_1()
                    .border_1()
                    .border_color(rgb(theme::BORDER))
                    .text_color(rgb(theme::TEXT))
                    .cursor_text()
                    .child(numeric.text())
                    .on_click(cx.listener(move |this, _event, window, cx| {
                        this.param_grid_begin_edit(Some(&text), window, cx);
                        cx.stop_propagation();
                        cx.notify();
                    }))
            })
            .child(arrow("param-options-up", "▲", true, cx))
            .child(arrow("param-options-down", "▼", false, cx))
            .into_any_element(),
    }
}

/// One of the `NumericUpDown`'s arrows.
fn arrow(
    id: &'static str,
    text: &'static str,
    up: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    crate::probe::measured(id, div())
        .id(id)
        .px_1()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::ACTION))
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .hover(|style| style.border_color(rgb(theme::ACCENT)))
        .child(text)
        .on_click(cx.listener(move |this, _event, _window, cx| {
            this.param_grid_step(up);
            cx.stop_propagation();
            cx.notify();
        }))
        .into_any_element()
}

/// The Set Bitmask button's text. `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1176`
pub const SET_BITMASK: &str = "Set Bitmask";

/// `splitContainer1`: the tree at the splitter's distance, the splitter, `but_collapse` docked
/// at the grid's left, and the rest; with the tree collapsed only the button and the rest.
/// Ctrl+S is taken here from whichever of the screen's controls has the keyboard, as
/// `ProcessCmdKey` takes it, and the grid's own keys when the grid has it.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.resx (splitContainer1, but_collapse); ConfigRawParams.cs:116-125`
pub fn split(
    tree: Option<AnyElement>,
    collapse: AnyElement,
    main: AnyElement,
    layout: &Layout,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let showing_tree = tree.is_some();
    #[allow(clippy::cast_precision_loss)] // pixels
    let distance = layout.splitter() as f32;
    div()
        .id("params-split")
        .flex()
        .flex_1()
        .min_h(px(0.0))
        .min_w(px(0.0))
        .on_key_down(cx.listener(|this, event: &KeyDownEvent, window, cx| {
            if this.param_grid_key(event, window, cx) {
                cx.stop_propagation();
                cx.notify();
            }
        }))
        .on_mouse_move(
            cx.listener(|this, event: &gpui::MouseMoveEvent, window, cx| {
                if !this.param_grid.dragging() {
                    return;
                }
                if event.pressed_button == Some(MouseButton::Left) {
                    let total = f32::from(window.viewport_size().width);
                    if this.param_grid.drag_to(f32::from(event.position.x), total) {
                        cx.notify();
                    }
                } else {
                    this.param_grid.end_drag();
                }
            }),
        )
        .on_mouse_up(
            MouseButton::Left,
            cx.listener(|this, _event: &gpui::MouseUpEvent, _window, _cx| {
                this.param_grid.end_drag();
            }),
        )
        .children(tree.map(|tree| {
            // Measured by its scrolling box - the rows in view, not the union of every row,
            // which reached below the window - so a script can `reveal` a group below it.
            crate::probe::measured("params-tree", div())
                .w(px(distance))
                .flex_shrink_0()
                .h_full()
                .child(
                    div()
                        .id("params-tree")
                        .size_full()
                        .p_2()
                        .overflow_y_scroll()
                        .child(tree),
                )
        }))
        .children(showing_tree.then(|| {
            crate::probe::measured("param-splitter", div())
                .id("param-splitter")
                .w(px(SPLITTER_WIDTH))
                .flex_shrink_0()
                .h_full()
                .bg(rgb(theme::BORDER))
                .cursor(gpui::CursorStyle::ResizeLeftRight)
                .on_mouse_down(
                    MouseButton::Left,
                    cx.listener(|this, event: &gpui::MouseDownEvent, _window, cx| {
                        this.param_grid
                            .begin_drag(Dragged::Splitter, f32::from(event.position.x));
                        cx.stop_propagation();
                    }),
                )
        }))
        .child(
            div()
                .w(px(COLLAPSE_SIZE))
                .flex_shrink_0()
                .pt_2()
                .child(collapse),
        )
        .child(main)
        .into_any_element()
}

/// A check box and its text, for the warning's "Show me again?".
fn check(
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
        // Takes the row's free space so OK sits at the right edge. Not `mr_auto`: under the
        // row's `justify_end` the layout counted the free space twice - once as the margin and
        // once as the end offset - and drew OK 140 px outside the box (the owner, 2026-09-25).
        .flex_grow(1.0)
        .text_xs()
        .cursor_pointer()
        .text_color(rgb(theme::TEXT))
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

/// The boxes over the window - RawParamWarning, then the first of the rest - and Set Bitmask's
/// window.
pub fn overlays(
    grid: &RawGrid,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    let mut shown = Vec::new();
    if let Some(again) = grid.warning() {
        // `CreateMessageShowAgainForm`: the text, "Show me again?" ticked at the left, OK.
        // C#: Common.cs:296-443
        let buttons = vec![
            check(
                "raw-param-warning-again",
                crate::config::adsb::SHOW_ME_AGAIN,
                again,
                |this| {
                    if let Some(value) = this.param_grid.toggle_warning() {
                        this.persisted.set(WARNING_KEY, value);
                    }
                },
                cx,
            ),
            action(
                "raw-param-warning-ok",
                "OK",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.param_grid.close_warning();
                    cx.notify();
                }),
            ),
        ];
        shown.push(servo_output::modal(
            "raw-param-warning",
            RAW_PARAM_WARNING,
            RAW_PARAM_WARNING_TEXT,
            false,
            buttons,
            window,
        ));
    } else if let Some(again) = grid.refresh() {
        // `MessageShowAgain("Refresh Params", ..., show_cancel: true)`: the text, "Show me
        // again?" ticked at the left, OK and Cancel. C#: Common.cs:296-443
        let buttons = vec![
            check(
                "param-refresh-again",
                crate::config::adsb::SHOW_ME_AGAIN,
                again,
                |this| {
                    if let Some(value) = this.param_grid.toggle_refresh() {
                        this.persisted
                            .set(crate::config::adsb::SHOW_AGAIN_KEY, value);
                    }
                },
                cx,
            ),
            action(
                "param-refresh-ok",
                "OK",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    if this.param_grid.answer_refresh(true) {
                        this.telemetry.download_parameters();
                    }
                    cx.notify();
                }),
            ),
            action(
                "param-refresh-cancel",
                "Cancel",
                theme::TEXT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.param_grid.answer_refresh(false);
                    cx.notify();
                }),
            ),
        ];
        shown.push(servo_output::modal(
            "param-refresh",
            REFRESH_TITLE,
            REFRESH_TEXT,
            false,
            buttons,
            window,
        ));
    } else if let Some(front) = grid.front() {
        let buttons = if front.is_question() {
            vec![
                answer("param-box-yes", "Yes", true, cx),
                answer("param-box-no", "No", false, cx),
            ]
        } else {
            vec![answer("param-box-ok", "OK", true, cx)]
        };
        shown.push(servo_output::modal(
            "param-box",
            &front.caption,
            &front.text,
            false,
            buttons,
            window,
        ));
    }
    if let Some(bitmask) = grid.bitmask() {
        shown.push(bitmask_window(bitmask, window, cx));
    }
    shown
}

/// A box's button: OK and Yes are `true`, No `false`.
fn answer(
    id: &'static str,
    text: &'static str,
    yes: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    action(
        id,
        text,
        theme::ACCENT,
        true,
        cx.listener(move |this, _event: &(), _window, cx| {
            this.param_grid_answer(yes);
            cx.notify();
        }),
    )
}

/// Set Bitmask's window: `ShowUserControl`'s form, 700 wide, not modal, over the screen.
/// `// C#: Controls/MavlinkCheckBoxBitMask.cs:66-141; Utilities/ExtensionsMP.cs:110-132`
fn bitmask_window(
    bitmask: &BitmaskWindow,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let size = window.viewport_size();
    let mut boxes = div()
        .flex()
        .flex_wrap()
        .gap_x(px(5.0))
        .gap_y(px(5.0))
        .w(px(520.0));
    for (index, (bit, text, ticked)) in bitmask.bits.iter().enumerate() {
        let id = format!("param-bitmask-{bit}");
        boxes = boxes.child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .flex()
                .items_center()
                .gap_1()
                .cursor_pointer()
                .child(
                    div()
                        .size(px(13.0))
                        .flex()
                        .items_center()
                        .justify_center()
                        .border_1()
                        .border_color(rgb(theme::DIM))
                        .bg(rgb(theme::BG))
                        .children(ticked.then(|| div().size(px(6.0)).bg(rgb(theme::ACCENT)))),
                )
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .child(text.clone()),
                )
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.param_grid_click_bit(index);
                    cx.notify();
                })),
        );
    }
    let frame = crate::probe::measured("param-bitmask", div())
        .w(px(700.0))
        .flex()
        .flex_col()
        .gap_2()
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .occlude()
        .child(
            div()
                .flex()
                .justify_between()
                .child(
                    div()
                        .text_sm()
                        .text_color(rgb(theme::TEXT))
                        .child(bitmask.title.clone()),
                )
                .child(
                    crate::probe::measured("param-bitmask-close", div())
                        .id("param-bitmask-close")
                        .px_1()
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .cursor_pointer()
                        .hover(|style| style.text_color(rgb(theme::TEXT)))
                        .child("X")
                        .on_click(cx.listener(|this, _event, _window, cx| {
                            this.param_grid.bitmask = None;
                            cx.notify();
                        })),
                ),
        )
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(bitmask.description.clone()),
        )
        .child(boxes);
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(
                (size.width - px(700.0)) / 2.0,
                size.height / 4.0,
            ))
            .child(frame),
    )
    .with_priority(1)
    .into_any_element()
}

impl MissionPlanner {
    /// A row clicked - on its Value, Desc or Fav cell, or elsewhere in it: the typing in another
    /// row's Value cell ends and is its edit (the grid ends an edit when the current cell
    /// changes), the row becomes the current one, and the grid takes the keyboard. A click
    /// elsewhere in the row chooses it again or lets it go, as the screen's rows always have.
    pub(crate) fn param_grid_select(
        &mut self,
        name: &str,
        toggle: bool,
        window: &mut Window,
        cx: &mut App,
    ) {
        if self
            .param_grid
            .editing()
            .is_some_and(|(editing, _)| editing != name)
            && let Some((edited, text)) = self.param_grid.end_edit()
        {
            self.param_grid_edit(&edited, &text);
        }
        let already = self.selected_param.as_deref() == Some(name);
        self.selected_param = if toggle && already {
            None
        } else {
            Some(name.to_owned())
        };
        window.focus(&self.param_grid_focus, cx);
    }

    /// Typing begins in the current row's Value cell: with the cell's text (a double click, F2)
    /// or with the key that began it, which replaces it.
    pub(crate) fn param_grid_begin_edit(
        &mut self,
        typed: Option<&str>,
        window: &mut Window,
        cx: &mut App,
    ) {
        let Some(name) = self.selected_param.clone() else {
            return;
        };
        let text = match typed {
            Some(typed) => typed.to_owned(),
            None => {
                let view = self.telemetry.view();
                let Some(parameter) =
                    view.parameters
                        .iter()
                        .find(|(held, _)| *held == name)
                        .map(|(name, value)| Parameter {
                            name: name.clone(),
                            value: *value,
                            meta: None,
                            default: None,
                        })
                else {
                    return;
                };
                value_text(&parameter, self.raw_params.changes())
            }
        };
        self.param_grid.begin_edit(&name, &text);
        window.focus(&self.param_edit_focus, cx);
    }

    /// A key anywhere on the screen: Ctrl+S from any control; F2 or a character with the grid
    /// holding the keyboard begins typing in the current row's Value cell. Whether it was taken.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:116-125`
    fn param_grid_key(&mut self, event: &KeyDownEvent, window: &mut Window, cx: &mut App) -> bool {
        let keystroke = &event.keystroke;
        if keystroke.modifiers.control && keystroke.key.eq_ignore_ascii_case("s") {
            self.param_grid_save();
            return true;
        }
        if !self.param_grid_focus.is_focused(window) || self.param_grid.editing().is_some() {
            return false;
        }
        if keystroke.key == "f2" {
            self.param_grid_begin_edit(None, window, cx);
            return true;
        }
        if keystroke.modifiers.control || keystroke.modifiers.platform {
            return false;
        }
        match keystroke.key_char.as_deref() {
            Some(text) if !text.chars().any(char::is_control) => {
                self.param_grid_begin_edit(Some(text), window, cx);
                true
            }
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::scripted::{INT32, Vehicle, param, until};
    use mp_link::ProtocolTimeouts;
    use mp_mavlink_dialects::all::MavMessage;
    use mp_params::UserLevel;

    /// Documentation for a test: a range, values, bits and a description as given.
    fn meta(
        range: Option<(f64, f64)>,
        values: &'static [(i64, &'static str)],
        bitmask: &'static [(u32, &'static str)],
        description: &'static str,
    ) -> &'static ParamMeta {
        Box::leak(Box::new(ParamMeta {
            name: "TEST",
            display_name: "Test parameter",
            description,
            units: "cm",
            range,
            range_text: "",
            increment: None,
            values,
            bitmask,
            user_level: UserLevel::Standard,
            reboot_required: false,
        }))
    }

    fn parameter(name: &str, value: f64) -> Parameter {
        Parameter {
            name: name.to_owned(),
            value,
            meta: None,
            default: None,
        }
    }

    /// The columns' names, headers, widths and minimum widths are the Designer's and the
    /// `.resx`'s; the Desc column fills and is the only one not kept.
    #[test]
    fn the_columns_are_the_designers() {
        let (Some(resx), Some(designer)) = (
            crate::config_coverage::source::csharp(
                "GCSViews/ConfigurationView/ConfigRawParams.resx",
            ),
            crate::config_coverage::source::csharp(
                "GCSViews/ConfigurationView/ConfigRawParams.Designer.cs",
            ),
        ) else {
            eprintln!("SKIP: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        for column in Column::ALL {
            let name = column.name();
            assert_eq!(
                values
                    .get(&format!("{name}.HeaderText"))
                    .map(String::as_str),
                Some(column.header()),
                "{name}"
            );
            assert_eq!(
                values.get(&format!("{name}.MinimumWidth")),
                Some(&column.minimum().to_string()),
                "{name}"
            );
            if column.kept() {
                assert_eq!(
                    values.get(&format!("{name}.Width")),
                    Some(&column.width().to_string()),
                    "{name}"
                );
            }
        }
        assert!(designer.contains(
            "this.Desc.AutoSizeMode = System.Windows.Forms.DataGridViewAutoSizeColumnMode.Fill;"
        ));
        assert_eq!(
            values.get("splitContainer1.SplitterDistance"),
            Some(&SPLITTER_DEFAULT.to_string())
        );
        assert_eq!(
            values.get("but_collapse.Size").map(String::as_str),
            Some("18, 18")
        );
    }

    /// `Activate` reads each kept column's width - `Math.Max(5, GetInt32)`, raised to its
    /// minimum - and the splitter's distance, 180 without one; `Deactivate` writes them all.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:70-84, 99-112`
    #[test]
    fn widths_and_the_splitter_are_read_and_saved_as_the_csharp_does() {
        let settings: BTreeMap<&str, &str> = [
            ("rawparam_Options_width", "222"),
            ("rawparam_Value_width", "3"),
            ("rawparam_Units_width", "wide"),
            ("rawparam_Command_width", ""),
            ("rawparam_Fav_width", " 40 "),
            ("rawparam_Desc_width", "999"),
        ]
        .into_iter()
        .collect();
        let mut layout = Layout::default();
        layout.set_width(Column::Command, 140);
        layout.restore(|key| settings.get(key).copied());
        assert_eq!(layout.width(Column::Options), 222);
        assert_eq!(layout.width(Column::Value), 50, "5, raised to MinimumWidth");
        assert_eq!(
            layout.width(Column::Units),
            50,
            "GetInt32's 0, then 5, then 50"
        );
        assert_eq!(
            layout.width(Column::Command),
            140,
            "an empty setting leaves it"
        );
        assert_eq!(layout.width(Column::Fav), 40);
        assert_eq!(layout.width(Column::DefaultValue), 70);
        assert_eq!(layout.splitter(), 180);

        let saved: BTreeMap<String, String> = layout.saved().into_iter().collect();
        assert_eq!(saved.len(), 7, "six columns and the splitter");
        assert_eq!(
            saved.get("rawparam_Options_width").map(String::as_str),
            Some("222")
        );
        assert_eq!(
            saved
                .get("rawparam_Default_value_width")
                .map(String::as_str),
            Some("70")
        );
        assert_eq!(
            saved.get("rawparam_splitterdistance").map(String::as_str),
            Some("180")
        );
        assert!(
            !saved.contains_key("rawparam_Desc_width"),
            "the fill column is not kept"
        );

        let mut again = Layout::default();
        again.restore(|key| match key {
            SPLITTER_KEY => Some("260"),
            _ => saved.get(key).map(String::as_str),
        });
        assert_eq!(again.width(Column::Options), 222);
        assert_eq!(again.splitter(), 260);
        again.restore(|key| (key == SPLITTER_KEY).then_some("-40"));
        assert_eq!(again.splitter(), PANEL_MIN);
    }

    /// Dragging a header's edge or the splitter moves it with the pointer, never under the
    /// column's minimum or the panel's.
    #[test]
    fn a_drag_moves_a_width_and_the_splitter() {
        let mut grid = RawGrid::default();
        grid.begin_drag(Dragged::Column(Column::Options), 500.0);
        assert!(grid.drag_to(560.4, 1600.0));
        assert_eq!(grid.layout().width(Column::Options), 210);
        grid.drag_to(300.0, 1600.0);
        assert_eq!(grid.layout().width(Column::Options), 50);
        grid.end_drag();
        assert!(!grid.drag_to(900.0, 1600.0), "no drag, nothing moves");

        grid.begin_drag(Dragged::Splitter, 180.0);
        grid.drag_to(240.0, 1600.0);
        assert_eq!(grid.layout().splitter(), 240);
        grid.drag_to(-500.0, 1600.0);
        assert_eq!(grid.layout().splitter(), PANEL_MIN);
        grid.drag_to(5000.0, 1600.0);
        assert_eq!(grid.layout().splitter(), 1600 - PANEL_MIN - 4);
    }

    /// `GetList`, `SetList` and `AppendList` as `Settings` has them: decoded, each once, and an
    /// empty list leaving the setting alone - so the last favourite unticked is still there.
    /// `// C#: ExtLibs/Utilities/Settings.cs:164-183; ConfigRawParams.cs:1045-1060`
    #[test]
    fn favourites_are_kept_as_settings_keeps_lists() {
        assert!(get_list(None).is_empty());
        assert_eq!(
            get_list(Some("A%20B;RTL_ALT;RTL_ALT;C+D")),
            ["A B", "RTL_ALT", "C D"]
        );
        assert_eq!(
            set_list(&["RTL_ALT".to_owned(), "A B/".to_owned()]).as_deref(),
            Some("RTL_ALT;A+B%2F")
        );
        assert_eq!(set_list(&[]), None);
        assert_eq!(
            fav_clicked(Some("RTL_ALT"), "WPNAV_SPEED", true).as_deref(),
            Some("RTL_ALT;WPNAV_SPEED")
        );
        assert_eq!(
            fav_clicked(Some("RTL_ALT;WPNAV_SPEED"), "RTL_ALT", false).as_deref(),
            Some("WPNAV_SPEED")
        );
        assert_eq!(fav_clicked(Some("RTL_ALT"), "RTL_ALT", false), None);
        assert_eq!(
            fav_clicked(None, "RTL_ALT", true).as_deref(),
            Some("RTL_ALT")
        );

        let mut grid = RawGrid::default();
        grid.activate(|key| (key == FAV_KEY).then_some("RTL_ALT"));
        assert!(grid.favourites().contains("RTL_ALT"));
        assert_eq!(grid.click_fav("RTL_ALT", Some("RTL_ALT")), None);
        assert!(!grid.favourites().contains("RTL_ALT"), "the cell unticks");
        assert_eq!(
            grid.click_fav("SERVO1_MIN", None).as_deref(),
            Some("SERVO1_MIN")
        );
    }

    /// `NaturalStringComparer`: numbers as numbers, leading zeros skipped, case ignored.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:757-828`
    #[test]
    fn names_sort_in_natural_order() {
        let mut names = vec![
            "SERVO10_MIN",
            "SERVO2_MIN",
            "SERVO1_MIN",
            "servo1_max",
            "SERVO01_TRIM",
        ];
        names.sort_by(|a, b| natural_compare(a, b));
        assert_eq!(
            names,
            [
                "servo1_max",
                "SERVO1_MIN",
                "SERVO01_TRIM",
                "SERVO2_MIN",
                "SERVO10_MIN"
            ]
        );
        assert_eq!(natural_compare("A01", "A1"), Ordering::Equal);
        assert_eq!(
            natural_compare("Q_PLT_Y_RATE", "Q_PLT_Y_RATE_TC"),
            Ordering::Less
        );
        assert_eq!(natural_compare("RC9", "RC10"), Ordering::Less);
        assert_eq!(natural_compare("É1", "É2"), Ordering::Less);
    }

    /// `OnParamsOnSortCompare`: favourites first, each part in natural order.
    #[test]
    fn favourites_sort_first() {
        let rows = [
            parameter("RTL_ALT", 1.0),
            parameter("RTL_SPEED", 1.0),
            parameter("RTL_CLIMB_MIN", 1.0),
            parameter("RTL_LOIT_TIME", 1.0),
        ];
        let mut shown: Vec<&Parameter> = rows.iter().collect();
        let favourites: BTreeSet<String> = ["RTL_SPEED".to_owned(), "RTL_LOIT_TIME".to_owned()]
            .into_iter()
            .collect();
        sort_rows(&mut shown, &favourites, Sort::default(), &BTreeMap::new());
        let names: Vec<&str> = shown.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(
            names,
            ["RTL_LOIT_TIME", "RTL_SPEED", "RTL_ALT", "RTL_CLIMB_MIN"]
        );
    }

    /// The Units, Options and Desc cells come from the documentation only when it describes
    /// the parameter; Options is the range, then each value `code:text` on a line of its own.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:604-645`
    #[test]
    fn the_documentation_fills_units_options_and_desc() {
        let both = meta(
            Some((0.0, 3.0)),
            &[(0, "Disabled"), (1, "Enabled")],
            &[],
            "Turns it on",
        );
        let filled = cells(Some(both));
        assert_eq!(filled.units, "cm");
        assert_eq!(filled.options, "0 3\n0:Disabled\n1:Enabled");
        assert_eq!(filled.desc, "Turns it on");
        assert_eq!(
            cells(Some(meta(Some((-0.5, 0.95)), &[], &[], "d"))).options,
            "-0.5 0.95"
        );
        assert_eq!(
            cells(Some(meta(None, &[(2, "Two")], &[], "d"))).options,
            "2:Two"
        );
        assert_eq!(
            cells(Some(meta(Some((0.0, 1.0)), &[], &[], ""))),
            Cells::default()
        );
        assert_eq!(cells(None), Cells::default());
    }

    /// `CheckForUrlAndLaunchInBrowser`: the first `http` or `https` word, nothing without one.
    #[test]
    fn the_desc_link_is_the_first_web_address() {
        assert_eq!(
            first_url("See https://ardupilot.org/copter/ and http://example.com"),
            Some("https://ardupilot.org/copter/")
        );
        assert_eq!(
            first_url("HTTP://Example.com/x"),
            Some("HTTP://Example.com/x")
        );
        assert_eq!(first_url("ftp://host/file and https:// alone"), None);
        assert_eq!(first_url("no address here"), None);
    }

    /// `Params_CellValueChanged`'s checks, in its order.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:454-533`
    #[test]
    fn an_edit_is_checked_as_cell_value_changed_checks_it() {
        let range = Some((0.0, 100.0));
        assert_eq!(
            cell_value_changed("RTL_ALT", "50", None, range),
            Edit::Write(50.0)
        );
        assert_eq!(
            cell_value_changed("RTL_ALT", "2,5", None, range),
            Edit::Write(2.5)
        );
        assert_eq!(
            cell_value_changed("RTL_ALT", "150", None, range),
            Edit::OutOfRange(150.0)
        );
        assert_eq!(
            cell_value_changed("RTL_ALT", "-1", None, range),
            Edit::OutOfRange(-1.0)
        );
        assert_eq!(
            cell_value_changed("RTL_ALT", "abc", None, range),
            Edit::Invalid
        );
        assert_eq!(
            cell_value_changed("RTL_ALT", "NaN", None, range),
            Edit::Invalid
        );
        assert_eq!(
            cell_value_changed("RTL_ALT", "inf", None, None),
            Edit::Invalid
        );
        assert_eq!(
            cell_value_changed("RTL_ALT", "1e3", None, None),
            Edit::Write(1000.0)
        );
        // ReadOnly before the range; a text `bool.Parse` refuses is the red cell.
        assert_eq!(
            cell_value_changed("STAT_BOOTCNT", "150", Some("True"), range),
            Edit::ReadOnly
        );
        assert_eq!(
            cell_value_changed("STAT_BOOTCNT", "5", Some(" false "), range),
            Edit::Write(5.0)
        );
        assert_eq!(
            cell_value_changed("STAT_BOOTCNT", "5", Some("yes"), range),
            Edit::Invalid
        );
        assert_eq!(
            cell_value_changed("STAT_BOOTCNT", "5", Some(""), range),
            Edit::Write(5.0)
        );
        // An RC or HS reverse of 0 is -1.
        assert_eq!(
            cell_value_changed("RC3_REV", "0", None, None),
            Edit::Write(-1.0)
        );
        assert_eq!(
            cell_value_changed("HS2_REV", "0", None, None),
            Edit::Write(-1.0)
        );
        assert_eq!(
            cell_value_changed("SERVO3_REV", "0", None, None),
            Edit::Write(0.0)
        );
    }

    /// An edit's outcome in the grid: the write, the ReadOnly box, the question - No keeps the
    /// value, Yes writes it - and the red cell, cleared by the next good edit.
    #[test]
    fn an_edit_writes_or_asks() {
        let mut grid = RawGrid::default();
        assert_eq!(
            grid.edit("RTL_ALT", "50", None, Some((0.0, 100.0))),
            Some(("RTL_ALT".to_owned(), 50.0))
        );
        assert!(grid.front().is_none());

        assert_eq!(grid.edit("RTL_ALT", "500", None, Some((0.0, 100.0))), None);
        let asked = grid.front().expect("the question");
        assert_eq!(asked.caption, "Out of range");
        assert_eq!(
            asked.text,
            "RTL_ALT value is out of range. Do you want to continue?"
        );
        assert!(asked.is_question());
        assert_eq!(grid.answer(false), Answered::Nothing);
        assert!(grid.front().is_none());

        grid.edit("RTL_ALT", "500", None, Some((0.0, 100.0)));
        assert_eq!(
            grid.answer(true),
            Answered::Write("RTL_ALT".to_owned(), 500.0)
        );

        assert_eq!(grid.edit("FORMAT_VERSION", "1", Some("True"), None), None);
        let told = grid.front().expect("the box");
        assert_eq!(
            (
                told.caption.as_str(),
                told.text.as_str(),
                told.is_question()
            ),
            (
                "ReadOnly",
                "FORMAT_VERSION is marked as ReadOnly, and will not be changed",
                false
            )
        );
        assert_eq!(grid.answer(true), Answered::Nothing);

        assert_eq!(grid.edit("RTL_ALT", "1+", None, None), None);
        assert!(grid.is_red("RTL_ALT"));
        grid.edit("RTL_ALT", "7", None, None);
        assert!(!grid.is_red("RTL_ALT"));
    }

    /// The Value cell typed into: Enter ends it with the text, Escape drops it.
    #[test]
    fn the_value_cell_is_typed_into() {
        let key = |text: &str| KeyDownEvent {
            keystroke: gpui::Keystroke {
                modifiers: gpui::Modifiers::default(),
                key: text.to_owned(),
                key_char: (text.chars().count() == 1).then(|| text.to_owned()),
            },
            is_held: false,
            prefer_character_input: false,
        };
        let mut grid = RawGrid::default();
        grid.begin_edit("RTL_ALT", "1");
        grid.edit_key(&key("5"));
        assert_eq!(grid.editing().map(|(_, field)| field.value()), Some("15"));
        assert_eq!(
            grid.edit_key(&key("enter")),
            (
                KeyOutcome::Submitted,
                Some(("RTL_ALT".to_owned(), "15".to_owned()))
            )
        );
        assert!(grid.editing().is_none());
        grid.begin_edit("RTL_ALT", "1");
        assert_eq!(grid.edit_key(&key("escape")), (KeyOutcome::Cancelled, None));
        assert!(grid.editing().is_none());
    }

    /// The `NumericUpDown` over a range: places from the increment, or from a small minimum;
    /// the cell's value clamped; the arrows held at the ends.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:1272-1306`
    #[test]
    fn the_numeric_up_down_is_the_csharps() {
        // Range 0..100: increment 10^floor(log10(0.1)) = 0.1, one place.
        let mut number = Numeric::new((0.0, 100.0), None, "15").expect("a range");
        assert_eq!(
            (number.places, number.increment, number.text()),
            (1, 0.1, "15.0".to_owned())
        );
        assert_eq!(number.step(true).as_deref(), Some("15.1"));
        // Out of range is clamped; a text with an exponent is decimal.TryParse's 0.
        assert_eq!(
            Numeric::new((0.0, 100.0), None, "250")
                .expect("a range")
                .text(),
            "100.0"
        );
        assert_eq!(
            Numeric::new((5.0, 100.0), None, "1E-05")
                .expect("a range")
                .text(),
            "5.00",
            "a range of 95: increment 0.01, two places"
        );
        // The documentation's increment; the maximum held.
        let mut stepped = Numeric::new((0.0, 10.0), Some(5.0), "9").expect("a range");
        assert_eq!(stepped.places, 0);
        assert_eq!(stepped.step(true).as_deref(), Some("10"));
        assert_eq!(stepped.step(true), None, "at the maximum nothing changes");
        assert_eq!(stepped.step(false).as_deref(), Some("5"));
        // A minimum smaller than the increment, not zero, sets the places.
        // Range 0.001..1000: increment 0.1, but the minimum is under it - three places.
        let small = Numeric::new((0.001, 1000.0), None, "1").expect("a range");
        assert_eq!(small.places, 3);
        assert_eq!(small.increment, 0.1);
        assert_eq!(small.text(), "1.000");
        // A range of nothing makes places WinForms refuses.
        assert_eq!(Numeric::new((3.0, 3.0), None, "3"), None);
    }

    /// `Params_RowEnter`'s control: a bitmask first, then values, then a range, else none.
    #[test]
    fn a_row_entered_gets_the_control_its_documentation_makes() {
        let bits = meta(
            Some((0.0, 7.0)),
            &[(0, "A")],
            &[(0, "One"), (2, "Four")],
            "d",
        );
        let values = meta(
            Some((0.0, 1.0)),
            &[(0, "Disabled"), (1, "Enabled")],
            &[],
            "d",
        );
        let range = meta(Some((0.0, 100.0)), &[], &[], "d");
        let plain = meta(None, &[], &[], "d");
        assert_eq!(
            options_control("X", "5", Some(bits)),
            Some(OptionsControl::Bitmask)
        );
        let Some(OptionsControl::Values { combo, open }) = options_control("X", "1", Some(values))
        else {
            panic!("a list of values");
        };
        assert_eq!(
            (combo.selected, combo.text(), open),
            (Some(1), "Enabled", false)
        );
        let Some(OptionsControl::Values { combo, .. }) = options_control("X", "0.5", Some(values))
        else {
            panic!("a list of values");
        };
        assert_eq!(combo.selected, None, "int.TryParse fails: SelectedIndex -1");
        assert!(matches!(
            options_control("X", "5", Some(range)),
            Some(OptionsControl::Range(_))
        ));
        assert_eq!(options_control("X", "5", Some(plain)), None);
        assert_eq!(options_control("X", "5", None), None);

        let mut grid = RawGrid::default();
        grid.enter_row(Some("X"), "1", Some(values));
        assert!(matches!(
            grid.control(),
            Some(("X", OptionsControl::Values { .. }))
        ));
        grid.enter_row(None, "", None);
        assert!(grid.control().is_none());
    }

    /// Set Bitmask's window: a box per bit, ticked from the cell's value, and its value as the
    /// C# adds it up.
    #[test]
    fn the_bitmask_window_reads_and_makes_the_value() {
        let bits = meta(
            None,
            &[],
            &[(0, "One"), (2, "Four"), (5, "ThirtyTwo")],
            "The bits",
        );
        let mut window = BitmaskWindow::new("X", "5", bits).expect("a number");
        assert_eq!(
            window
                .bits
                .iter()
                .map(|(_, _, ticked)| *ticked)
                .collect::<Vec<_>>(),
            [true, true, false]
        );
        assert_eq!(window.value_text(), "5");
        if let Some(bit) = window.bits.get_mut(2) {
            bit.2 = true;
        }
        assert_eq!(window.value_text(), "37");
        assert_eq!(
            (window.title.as_str(), window.description.as_str()),
            ("Test parameter", "The bits")
        );
        assert_eq!(BitmaskWindow::new("X", "lots", bits), None);
    }

    /// Write Params' question: each change listed with twenty or fewer, the count alone above.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:257-310`
    #[test]
    fn write_params_asks_first() {
        let changes: BTreeMap<String, f64> = [
            ("RTL_ALT".to_owned(), 2000.0),
            ("FENCE_ENABLE".to_owned(), 1.0),
        ]
        .into_iter()
        .collect();
        let names = save_order(&changes);
        assert_eq!(names, ["FENCE_ENABLE", "RTL_ALT"], "SortENABLE");
        let held = |name: &str| (name == "RTL_ALT").then_some(1500.0);
        assert_eq!(
            save_question(&names, &changes, held).as_deref(),
            Some(
                "You are about to change 2 parameters. Please review the changes below:\n\nFENCE_ENABLE:  -> 1\nRTL_ALT: 1500 -> 2000\n\nDo you want to proceed?"
            )
        );
        let many: BTreeMap<String, f64> = (0..21).map(|n| (format!("P{n}"), 1.0)).collect();
        assert_eq!(
            save_question(&save_order(&many), &many, |_| None).as_deref(),
            Some("You are about to change 21 parameters. Are you sure you want to proceed?")
        );
        assert_eq!(save_question(&[], &changes, held), None);

        let mut grid = RawGrid::default();
        assert_eq!(
            grid.save(&BTreeMap::new(), false, |_| None),
            Ok(true),
            "nothing to write"
        );
        assert_eq!(grid.save(&changes, false, held), Err(NOT_CONNECTED));
        assert_eq!(
            grid.save(&many, false, |_| None),
            Ok(false),
            "over twenty asks first"
        );
        assert_eq!(
            grid.front().map(|shown| shown.caption.as_str()),
            Some(CONFIRM_CAPTION)
        );
        assert!(matches!(grid.answer(false), Answered::Nothing));
        assert_eq!(grid.save(&changes, true, held), Ok(false));
        assert_eq!(grid.answer(true), Answered::WriteChanges(names.clone()));
        assert_eq!(
            grid.write_changes(&names, &changes, false, |_| false),
            Err(NOT_CONNECTED_2)
        );

        // Nothing to write: the "No changes" box.
        grid.saved(&Saving::default(), 10, 10, false);
        assert_eq!(
            grid.front()
                .map(|shown| (shown.caption.as_str(), shown.text.as_str())),
            Some(NO_CHANGES)
        );
        grid.answer(true);
        // The vehicle's count not the count held: disarmed, the box and then the fetch.
        grid.saved(
            &Saving {
                total: 1,
                ..Saving::default()
            },
            9,
            10,
            false,
        );
        assert_eq!(
            grid.front().map(|shown| shown.text.as_str()),
            Some(COUNT_CHANGED.1)
        );
        assert_eq!(grid.answer(true), Answered::Refresh);
        grid.saved(
            &Saving {
                total: 1,
                ..Saving::default()
            },
            9,
            10,
            true,
        );
        assert_eq!(
            grid.front().map(|shown| shown.text.as_str()),
            Some(COUNT_CHANGED_ARMED.1)
        );
        assert_eq!(grid.answer(true), Answered::Nothing);
    }

    /// Ctrl+S's Yes over a real link: every change written one after another, and the reboot's
    /// box when one the documentation marks is heard back - not when it timed out.
    #[test]
    fn write_params_writes_and_says_a_reboot_is_required() {
        let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default().faster(20));
        for (name, value) in [
            ("RTL_ALT", 1500.0),
            ("INS_GYRO_FILTER", 20.0),
            ("WPNAV_SPEED", 500.0),
        ] {
            vehicle.send(&param(name, value, INT32));
            until(name, || telemetry.holds_parameter(name));
        }
        let changes: BTreeMap<String, f64> = [
            ("RTL_ALT".to_owned(), 2000.0),
            ("INS_GYRO_FILTER".to_owned(), 40.0),
            ("WPNAV_SPEED".to_owned(), 750.0),
        ]
        .into_iter()
        .collect();
        let mut grid = RawGrid::default();
        assert_eq!(grid.save(&changes, true, |_| None), Ok(false));
        let Answered::WriteChanges(names) = grid.answer(true) else {
            panic!("Yes writes the changes");
        };
        // WPNAV_SPEED also asks for a reboot here, and is never echoed.
        let writes = grid
            .write_changes(&names, &changes, true, |name| name != "RTL_ALT")
            .expect("connected");
        assert_eq!(writes.len(), 3);
        let mut list = crate::params::ParamWrites::apply(writes, 0);
        until("the writes to finish", || {
            for message in vehicle.read() {
                if let MavMessage::ParamSet(set) = message {
                    let name = mp_params::decode_param_id(&set.param_id);
                    if name != "WPNAV_SPEED" {
                        vehicle.send(&param(&name, set.param_value, INT32));
                    }
                }
            }
            if let Some(written) = list.advance(&telemetry).written {
                grid.written(&written);
            }
            list.is_finished()
        });
        let saving = grid.take_saved(false).expect("Write Params is over");
        assert!(grid.take_saved(false).is_none());
        grid.saved(&saving, 3, 3, false);
        assert_eq!(
            grid.front()
                .map(|shown| (shown.caption.as_str(), shown.text.as_str())),
            Some(REBOOT_REQUIRED)
        );
        assert_eq!(
            telemetry
                .view()
                .parameters
                .iter()
                .find(|(name, _)| name == "RTL_ALT")
                .map(|(_, v)| *v),
            Some(2000.0)
        );
    }

    /// An edit that passes is written through the link and heard back.
    #[test]
    fn an_edit_is_written_to_the_vehicle() {
        let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default().faster(20));
        vehicle.send(&param("RTL_ALT", 1500.0, INT32));
        until("RTL_ALT", || telemetry.holds_parameter("RTL_ALT"));
        let mut grid = RawGrid::default();
        let (name, value) = grid
            .edit("RTL_ALT", "2500", None, Some((200.0, 300_000.0)))
            .expect("in range");
        let mut write = crate::params::ParamWrites::nudge(&name, value);
        let mut outcome = None;
        until("the write to be heard back", || {
            for message in vehicle.read() {
                if let MavMessage::ParamSet(set) = message {
                    vehicle.send(&param("RTL_ALT", set.param_value, INT32));
                }
            }
            if let Some(written) = write.advance(&telemetry).written {
                outcome = Some(written.outcome);
            }
            write.is_finished()
        });
        assert!(matches!(outcome, Some(RequestOutcome::Accepted { .. })));
        let set = vehicle.count(|message| matches!(message, MavMessage::ParamSet(_)));
        assert_eq!(set, 1);
    }

    /// RawParamWarning shows on every `Activate` until its "Show me again?" is unticked, which
    /// is saved as `False`; any other value `bool.TryParse` refuses hides it too.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:92; Common.cs:260-270, 446-449`
    #[test]
    fn the_raw_param_warning_shows_until_unticked() {
        let mut grid = RawGrid::default();
        grid.activate(|_| None);
        assert_eq!(grid.warning(), Some(true));
        assert_eq!(grid.toggle_warning(), Some("False"));
        assert_eq!(grid.toggle_warning(), Some("True"));
        assert_eq!(grid.toggle_warning(), Some("False"));
        grid.close_warning();
        assert_eq!(grid.warning(), None);
        assert_eq!(grid.toggle_warning(), None);
        grid.activate(|key| (key == WARNING_KEY).then_some("True"));
        assert_eq!(grid.warning(), Some(true));
        grid.activate(|key| (key == WARNING_KEY).then_some("False"));
        assert_eq!(grid.warning(), None);
        grid.activate(|key| (key == WARNING_KEY).then_some("maybe"));
        assert_eq!(grid.warning(), None);
        assert_eq!(WARNING_KEY, "SHOWAGAIN_Raw_Param_Warning");
    }

    /// Refresh Params asks only on an armed vehicle whose tick is not cleared: not armed, the
    /// fetch at once; armed, the question, OK fetching and Cancel not; the tick cleared, kept
    /// as `SHOWAGAIN_Refresh_Params` and the question not asked again; no link, nothing.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:420-428`
    #[test]
    fn refresh_params_asks_only_when_armed_and_not_suppressed() {
        let mut grid = RawGrid::default();
        assert!(!grid.press_refresh(false, true, |_| None));
        assert_eq!(grid.refresh(), None);
        assert!(grid.press_refresh(true, false, |_| None));
        assert_eq!(grid.refresh(), None);
        assert!(!grid.press_refresh(true, true, |_| None));
        assert_eq!(grid.refresh(), Some(true));
        assert!(!grid.answer_refresh(false));
        assert_eq!(grid.refresh(), None);
        assert!(!grid.press_refresh(true, true, |_| None));
        assert_eq!(grid.toggle_refresh(), Some("False"));
        assert_eq!(grid.toggle_refresh(), Some("True"));
        assert_eq!(grid.toggle_refresh(), Some("False"));
        assert!(grid.answer_refresh(true));
        assert_eq!(grid.toggle_refresh(), None);
        let key = crate::config::adsb::SHOW_AGAIN_KEY;
        assert!(grid.press_refresh(true, true, |name| (name == key).then_some("False")));
        assert_eq!(grid.refresh(), None);
        assert!(!grid.press_refresh(true, true, |name| (name == key).then_some("True")));
        assert_eq!(grid.refresh(), Some(true));
        assert_eq!(key, "SHOWAGAIN_Refresh_Params");
    }

    /// The warning's text and title are `Strings.resx`'s.
    #[test]
    fn the_warning_is_the_strings_resx() {
        let Some(strings) = crate::config_coverage::source::csharp("ExtLibs/Strings/Strings.resx")
        else {
            eprintln!("SKIP: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let values = crate::config_coverage::source::resx(&strings);
        assert_eq!(
            values.get("WarningUpdateParamList").map(String::as_str),
            Some(REFRESH_TEXT)
        );
        assert_eq!(
            values.get("RawParamWarning").map(String::as_str),
            Some(RAW_PARAM_WARNING)
        );
        assert_eq!(
            values
                .get("RawParamWarningi")
                .map(|text| text.replace("\r\n", "\n")),
            Some(RAW_PARAM_WARNING_TEXT.to_owned())
        );
    }

    /// The Value cell reads the value waiting in `_changes` as a double, else the vehicle's as
    /// a float.
    #[test]
    fn the_value_cell_shows_what_is_waiting() {
        let rtl = parameter("RTL_ALT", 0.1);
        let mut changes = BTreeMap::new();
        assert_eq!(value_text(&rtl, &changes), "0.1");
        changes.insert("RTL_ALT".to_owned(), 0.25);
        assert_eq!(value_text(&rtl, &changes), "0.25");
    }

    /// Every control and fact the script names is one this module or the screen draws or
    /// records.
    #[test]
    fn the_scripts_names_exist() {
        let script = include_str!("../../../tests/gui/params-list-columns.gui");
        let sources = [
            include_str!("raw_params_grid.rs"),
            include_str!("raw_params.rs"),
            include_str!("params.rs"),
            include_str!("main.rs"),
        ];
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or_default().trim();
            let mut words = line.split_whitespace();
            let (Some(verb), Some(name)) = (words.next(), words.next()) else {
                continue;
            };
            let name = name.split(['@', ':']).next().unwrap_or_default();
            let made: Vec<String> = name
                .match_indices(['-', '.'])
                .filter_map(|(at, _)| name.get(..=at))
                .map(|stem| format!("\"{stem}{{"))
                .collect();
            match verb {
                "click" | "doubleclick" | "expect" => assert!(
                    name.starts_with("config.")
                        || sources.iter().any(|source| {
                            source.contains(&format!("\"{name}\""))
                                || made.iter().any(|stem| source.contains(stem))
                        }),
                    "{verb} {name}: not in the sources"
                ),
                _ => {}
            }
        }
    }

    /// `AddNewLinesForTooltip`: short text as it is; long text in lines of `2 * (int)sqrt(n)`
    /// characters, broken at whitespace, the whitespace after a break dropped.
    #[test]
    fn a_long_tooltip_is_broken_into_lines_at_whitespace() {
        assert_eq!(tooltip_lines("short"), "short");
        let text = "a".repeat(20) + " " + &"b".repeat(20) + "  " + &"c".repeat(20) + " tail";
        // 66 characters: (int)sqrt(66) = 8, lines of 16. The first break comes at the first
        // whitespace at or past position 16 (after the a's), the next after the b's.
        let broken = tooltip_lines(&text);
        assert_eq!(
            broken,
            "a".repeat(20) + "\n" + &"b".repeat(20) + "\n" + &"c".repeat(20) + "\ntail"
        );
        // Exactly 49 characters is still one line; 50 is broken.
        assert_eq!(tooltip_lines("x ".repeat(24).trim_end()), "x ".repeat(24).trim_end());
        assert!(tooltip_lines(&"x ".repeat(25)).contains('\n'));
    }

    /// The Options tooltip: the values one a line, and past fifty of them in columns.
    #[test]
    fn the_options_tooltip_lists_values_and_columns_many() {
        assert_eq!(options_tooltip(""), None);
        assert_eq!(
            options_tooltip("0:Off,1:On,").as_deref(),
            Some("0:Off\n1:On\n")
        );
        let many: String = (0..60).map(|i| format!("{i}:v{i},")).collect();
        let tip = options_tooltip(&many).expect("a tooltip");
        // 60 commas: (60 - 1) / 50 + 1 = 2 columns; the first line holds two values.
        assert!(tip.starts_with("0:v0, 1:v1, \n2:v2, 3:v3, "), "{tip}");
        assert_eq!(tip.matches('\n').count(), 29);
    }

    /// `new Expression(value).calculate()`: arithmetic with precedence and parentheses; what
    /// does not parse, or is not finite, is refused.
    #[test]
    fn a_typed_value_may_be_an_expression() {
        assert_eq!(calculate("2*3.5"), Some(7.0));
        assert_eq!(calculate(" (1 + 2) / 4 "), Some(0.75));
        assert_eq!(calculate("-0.5"), Some(-0.5));
        assert_eq!(calculate("2^10"), Some(1024.0));
        assert_eq!(calculate("2^3^2"), Some(512.0), "right to left");
        assert_eq!(calculate("1-2-3"), Some(-4.0), "left to right");
        assert_eq!(calculate("--2"), Some(2.0));
        assert_eq!(calculate("1e3"), Some(1000.0));
        assert_eq!(calculate("1.5E-2*2"), Some(0.03));
        assert_eq!(calculate("1/0"), None, "infinity is refused");
        assert_eq!(calculate("abc"), None);
        assert_eq!(calculate("2*"), None);
        assert_eq!(calculate("(2"), None);
        assert_eq!(calculate("sqrt(4)"), None, "mXparser's functions are not read here");
        assert_eq!(calculate(""), None);
    }

    /// A header clicked sorts by its column, ascending, then descending; the Fav column is
    /// `NotSortable`; favourites stay first either way, and the rows tie-break by name.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:674-676, 833-858`
    #[test]
    fn a_header_click_sorts_by_its_column_with_favourites_first() {
        let mut sort = Sort::default();
        assert_eq!(sort.label(), "Command asc");
        sort.click(Column::Value);
        assert_eq!(sort.label(), "Value asc");
        sort.click(Column::Value);
        assert_eq!(sort.label(), "Value desc");
        sort.click(Column::Fav);
        assert_eq!(sort.label(), "Value desc", "Fav is NotSortable");
        sort.click(Column::Command);
        assert_eq!(sort.label(), "Command asc");

        let a = parameter("B_ONE", 3.0);
        let b = parameter("A_TWO", 1.0);
        let c = parameter("C_TEN", 10.0);
        let d = parameter("D_TWO", 1.0);
        let favourites: BTreeSet<String> = ["C_TEN".to_owned()].into_iter().collect();
        let changes = BTreeMap::new();
        fn names<'a>(rows: &[&'a Parameter]) -> Vec<&'a str> {
            rows.iter().map(|p| p.name.as_str()).collect()
        }

        let mut rows = vec![&a, &b, &c, &d];
        sort_rows(&mut rows, &favourites, Sort::default(), &changes);
        assert_eq!(names(&rows), ["C_TEN", "A_TWO", "B_ONE", "D_TWO"]);

        let by_value = Sort { column: Column::Value, ascending: true };
        sort_rows(&mut rows, &favourites, by_value, &changes);
        assert_eq!(names(&rows), ["C_TEN", "A_TWO", "D_TWO", "B_ONE"], "ties by name");

        let by_value_desc = Sort { column: Column::Value, ascending: false };
        sort_rows(&mut rows, &favourites, by_value_desc, &changes);
        assert_eq!(names(&rows), ["C_TEN", "B_ONE", "D_TWO", "A_TWO"], "the favourite still first");

        // A value waiting in `_changes` sorts by what the cell shows.
        let changed: BTreeMap<String, f64> = [("A_TWO".to_owned(), 100.0)].into_iter().collect();
        sort_rows(&mut rows, &favourites, by_value, &changed);
        assert_eq!(names(&rows), ["C_TEN", "D_TWO", "B_ONE", "A_TWO"]);
    }
}
