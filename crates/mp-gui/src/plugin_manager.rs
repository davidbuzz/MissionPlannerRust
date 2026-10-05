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

//! The plugin manager, `Plugin/PluginUI.cs`: Ctrl+P's form (`MainV2.ProcessCmdKey`), a grid of
//! the plugins with each one's Enabled box, Save && Close writing `DisabledPlugins` for the next
//! start, and Show Errors.
//!
//! * The rows are `PopulateGridView`'s: first every plugin `PluginLoader.Plugins` holds - the
//!   ones whose `Init` and `Loaded` both said yes, `Running` here - green when enabled, dark orange
//!   when on the disabled list (loaded this run, disabled for the next); then every name on the
//!   disabled list no loaded plugin's file name contains and whose file is in the plugins folder,
//!   dark red, "Not loaded".
//! * Divergence: the planner carries Mission Planner's shipped plugins built in (`plugins_ui`), so
//!   a built-in plugin's name counts as a file in the folder - without that a shipped plugin once
//!   disabled could never be enabled again from here.
//! * Save && Close is `UpdateDisabledPlugins`: the file names of the rows not ticked, lower-cased,
//!   as `DisabledPlugins` (`SetList`), or the setting removed when none; then
//!   `bRestartRequired`, which shows the warning label on every later opening this run.
//! * Show Errors is `PluginLoader.ErrorInfo`, every value and a newline, or "No Errors": the C#
//!   records there the scripts that would not compile; here, the plugins that could not be loaded
//!   at all (not loaded for any reason but their `Init` saying no).
//! * `btnLoadPlugin_Click` is not ported: no control in the designer calls it.
//! * Not in the C#, the owner's addition (2026-10-04): an Exercise column, where each loaded
//!   plugin's Try lists what it added to the two maps' menus, and a click on one runs it as the
//!   map's menu would - at the flight map's last right-click, or its centre - so every working
//!   plugin can be tried from here.
//! * Divergence, the owner's (2026-10-04): not a form of its own but the PLUGINS tab, between LOGS
//!   and HELP, and Ctrl+P shows that tab; filled anew each time it is chosen, and again after
//!   Save && Close. Its buttons are sized to their text, where the designer's 73 by 31 cut theirs
//!   in half.
//!
//! `// C#: Plugin/PluginUI.cs:14-149; Plugin/PluginUI.Designer.cs:31-178; MainV2.cs:4118-4122`

use gpui::{
    AnyElement, Context, InteractiveElement, IntoElement, ParentElement, SharedString,
    StatefulInteractiveElement, Styled, div, px, rgb,
};
use mp_plugin_host::{INIT_REFUSED, PluginState, PluginStatus};

use crate::{MissionPlanner, facts, theme};

/// The form's caption.
const FORM_TEXT: &str = "PluginManager";
/// `bSave`'s text: "Save && Close" in the designer, an escaped ampersand.
const SAVE_TEXT: &str = "Save & Close";
/// `but_errors`'s text.
const ERRORS_TEXT: &str = "Show Errors";
/// `labelWarning`'s text, its `\r` a line break.
const WARNING_TEXT: &str =
    "Enable/Disable settings changed, till restart \nnot loaded but enabled plugins will not shown!";
/// The grid's columns' header texts, `pluginName` to `pluginEnabled`.
const HEADERS: [&str; 5] = ["Plugin Name", "Author", "Version", "FileName", "Enabled"];
/// What the C# puts in a not-loaded row's name, author and version.
const NOT_LOADED: &str = "Not loaded";
const DASHES: &str = "--";

/// A row's back colour, `row.DefaultCellStyle.BackColor`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RowColour {
    /// Loaded and enabled: `Color.Green`.
    Green,
    /// Loaded, and on the disabled list: `Color.DarkOrange`.
    DarkOrange,
    /// On the disabled list and not loaded: `Color.DarkRed`.
    DarkRed,
}

impl RowColour {
    /// The .NET colour.
    const fn rgb(self) -> u32 {
        match self {
            Self::Green => 0x00_80_00,
            Self::DarkOrange => 0xFF_8C_00,
            Self::DarkRed => 0x8B_00_00,
        }
    }

    /// One word, as a fact carries it.
    const fn word(self) -> &'static str {
        match self {
            Self::Green => "green",
            Self::DarkOrange => "dark-orange",
            Self::DarkRed => "dark-red",
        }
    }
}

/// One row of `dgvPlugins`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Row {
    /// `pluginName`.
    pub name: String,
    /// `pluginAuthor`.
    pub author: String,
    /// `pluginVersion`.
    pub version: String,
    /// `pluginDll`: the file's name, lower-cased.
    pub dll: String,
    /// `pluginEnabled`, the one cell the user changes.
    pub enabled: bool,
    pub colour: RowColour,
    /// The loaded plugin's index among the host's, for Exercise; `None` for a "Not loaded" row.
    pub plugin: Option<usize>,
}

/// `PopulateGridView`: the loaded plugins, then the disabled names not loaded whose file is
/// `present` (in the plugins folder, or built in).
/// `// C#: Plugin/PluginUI.cs:27-70`
pub(crate) fn rows(plugins: &[PluginStatus], disabled: &[String], present: impl Fn(&str) -> bool) -> Vec<Row> {
    let mut out = Vec::new();
    // `foreach (Plugin.Plugin p in Plugin.PluginLoader.Plugins)`: Init and Loaded said yes.
    let loaded: Vec<(usize, &PluginStatus)> = plugins
        .iter()
        .enumerate()
        .filter(|(_, status)| status.state == PluginState::Running)
        .collect();
    for &(index, status) in &loaded {
        let Some(info) = &status.info else { continue };
        // `!DisabledPluginNames.Contains(p.FileName, StringComparer.OrdinalIgnoreCase)`
        let enabled = !disabled
            .iter()
            .any(|off| off.eq_ignore_ascii_case(&status.file));
        out.push(Row {
            name: info.name.clone(),
            author: info.author.clone(),
            version: info.version.clone(),
            dll: status.file.to_lowercase(),
            enabled,
            colour: if enabled {
                RowColour::Green
            } else {
                RowColour::DarkOrange
            },
            plugin: Some(index),
        });
    }
    for off in disabled {
        // `if (p.FileName.ToLower().Contains(s)) isLoaded = true;`
        let is_loaded = loaded
            .iter()
            .any(|(_, status)| status.file.to_lowercase().contains(off.as_str()));
        if present(off) && !is_loaded {
            out.push(Row {
                name: NOT_LOADED.to_owned(),
                author: DASHES.to_owned(),
                version: DASHES.to_owned(),
                dll: off.clone(),
                enabled: false,
                colour: RowColour::DarkRed,
                plugin: None,
            });
        }
    }
    out
}

/// `UpdateDisabledPlugins`'s list: every row not ticked, its file name lower-cased.
/// `// C#: Plugin/PluginUI.cs:73-84`
pub(crate) fn disabled_after(rows: &[Row]) -> Vec<String> {
    rows.iter()
        .filter(|row| !row.enabled)
        .map(|row| row.dll.to_lowercase())
        .collect()
}

/// `PluginLoader.ErrorInfo` as `but_errors_Click` shows it: every value and a newline, or
/// "No Errors". Here the values are why each plugin that could not be loaded was not.
/// `// C#: Plugin/PluginUI.cs:140-147`
pub(crate) fn error_text(plugins: &[PluginStatus]) -> String {
    let message: String = plugins
        .iter()
        .filter_map(|status| match &status.state {
            PluginState::NotLoaded(reason) if reason != INIT_REFUSED => {
                Some(format!("{}\n", reason))
            }
            _ => None,
        })
        .collect();
    if message.is_empty() {
        "No Errors".to_owned()
    } else {
        message
    }
}

/// The form, while it is open.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Form {
    rows: Vec<Row>,
    /// Show Errors' message box, while it shows: `CustomMessageBox.Show(msg, "Errors")`.
    errors: Option<String>,
    /// What Show Errors will say, read when the form opened.
    error_text: String,
}

/// Ctrl+P's form and what outlives it.
#[derive(Debug, Default)]
pub(crate) struct PluginManager {
    form: Option<Form>,
    /// `PluginLoader.bRestartRequired`: set by Save && Close, for the rest of the run.
    restart_required: bool,
    /// The row whose Try is open: its plugin's map menu entries listed below the grid.
    exercising: Option<usize>,
    /// How many times it has been opened.
    pub opened: usize,
}

impl PluginManager {
    /// `new PluginUI().Show()`: a fresh form over the plugins as they are now.
    /// `// C#: Plugin/PluginUI.cs:18-24; MainV2.cs:4118-4122`
    pub(crate) fn show(
        &mut self,
        plugins: &[PluginStatus],
        disabled: &[String],
        present: impl Fn(&str) -> bool,
    ) {
        self.opened += 1;
        self.exercising = None;
        self.form = Some(Form {
            rows: rows(plugins, disabled, present),
            errors: None,
            error_text: error_text(plugins),
        });
    }

    #[must_use]
    pub(crate) fn is_open(&self) -> bool {
        self.form.is_some()
    }

    /// The form's close box.
    pub(crate) fn close(&mut self) {
        self.form = None;
        self.exercising = None;
    }

    /// A row's Try: its plugin's entries listed, or put away when they already are; nothing
    /// for a row with no loaded plugin.
    pub(crate) fn toggle_exercise(&mut self, row: usize) {
        let loaded = self
            .form
            .as_ref()
            .and_then(|form| form.rows.get(row))
            .is_some_and(|row| row.plugin.is_some());
        self.exercising = if loaded && self.exercising != Some(row) {
            Some(row)
        } else {
            None
        };
    }

    /// The plugin being exercised: its row's name and its index among the host's.
    pub(crate) fn exercised(&self) -> Option<(&str, usize)> {
        let row = self.form.as_ref()?.rows.get(self.exercising?)?;
        Some((row.name.as_str(), row.plugin?))
    }

    /// A click on a row's Enabled box.
    pub(crate) fn toggle(&mut self, index: usize) {
        if let Some(row) = self.form.as_mut().and_then(|form| form.rows.get_mut(index)) {
            row.enabled = !row.enabled;
        }
    }

    /// `bSave_Click`: the disabled list for `DisabledPlugins` (`None` when the form is not open),
    /// a restart required, and the form closed.
    /// `// C#: Plugin/PluginUI.cs:86-91`
    pub(crate) fn save(&mut self) -> Option<Vec<String>> {
        let form = self.form.take()?;
        self.restart_required = true;
        Some(disabled_after(&form.rows))
    }

    /// `but_errors_Click`.
    pub(crate) fn show_errors(&mut self) {
        if let Some(form) = self.form.as_mut() {
            form.errors = Some(form.error_text.clone());
        }
    }

    /// The errors box's OK.
    pub(crate) fn dismiss_errors(&mut self) {
        if let Some(form) = self.form.as_mut() {
            form.errors = None;
        }
    }

    /// The facts a script asserts on: whether the form is open, the warning's visibility, each
    /// row's name, file, Enabled box and colour, and the errors box.
    pub(crate) fn record_facts(&self) {
        facts::record("plugin-manager.open", self.is_open());
        facts::record("plugin-manager.opened", self.opened);
        facts::record("plugin-manager.restart-required", self.restart_required);
        let Some(form) = &self.form else { return };
        facts::record("plugin-manager.rows", form.rows.len());
        facts::record(
            "plugin-manager.exercising",
            self.exercised().map_or("none", |(name, _)| name),
        );
        for (index, row) in form.rows.iter().enumerate() {
            facts::record(
                format!("plugin-manager.row.{index}"),
                format!(
                    "{}|{}|{}|{}",
                    row.name,
                    row.dll,
                    if row.enabled { "enabled" } else { "disabled" },
                    row.colour.word()
                ),
            );
        }
        facts::record(
            "plugin-manager.errors",
            form.errors.as_deref().unwrap_or("none"),
        );
    }
}

/// The grid's rows' height (`RowTemplate.Height`), and its columns' designer widths, the least
/// each takes (`AutoSizeMode.AllCells` widens them to their text).
const ROW_HEIGHT: f32 = 24.0;
const COLUMN_WIDTHS: [f32; 5] = [92.0, 63.0, 67.0, 76.0, 52.0];
/// A character's width and a cell's padding, for sizing a column to its text.
const CHAR_WIDTH: f32 = 7.0;
const CELL_PADDING: f32 = 12.0;
/// The Exercise column's header, and its width.
const EXERCISE_TEXT: &str = "Exercise";
const EXERCISE_WIDTH: f32 = 70.0;
/// What a loaded row's Exercise cell says, closed and open; and the list's words.
const TRY_TEXT: &str = "Try";
const TRY_OPEN_TEXT: &str = "Close";
const NO_ENTRIES_TEXT: &str = "adds nothing to the map menus";
const RUNS_AT_TEXT: &str = "runs at the flight map's last right-click, else its centre";

/// Each column's width: the designer's, or its widest text (`AllCells`).
fn column_widths(rows: &[Row]) -> [f32; 5] {
    let mut widths = COLUMN_WIDTHS;
    for (column, header) in HEADERS.iter().enumerate() {
        let longest = rows
            .iter()
            .map(|row| match column {
                0 => row.name.chars().count(),
                1 => row.author.chars().count(),
                2 => row.version.chars().count(),
                3 => row.dll.chars().count(),
                _ => 0,
            })
            .chain(std::iter::once(header.chars().count()))
            .max()
            .unwrap_or(0);
        #[allow(clippy::cast_precision_loss)] // a few dozen characters
        let wanted = longest as f32 * CHAR_WIDTH + CELL_PADDING;
        if let Some(width) = widths.get_mut(column) {
            *width = width.max(wanted);
        }
    }
    widths
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
        .text_xs()
        .child(text.into())
}

/// Where a map menu click lands: the flight map's last right-click, as its menu has it, else the
/// map's centre, else (0, 0).
fn exercise_point(this: &MissionPlanner) -> (f64, f64) {
    this.fly_data
        .mouse_down_start
        .map(|(at, _)| at)
        .or_else(|| this.map.borrow().position_and_zoom().map(|(centre, _)| centre))
        .map_or((0.0, 0.0), |at| (at.latitude(), at.longitude()))
}

/// The form, while it is open, with Show Errors' box over it.
/// `// C#: Plugin/PluginUI.Designer.cs:31-178`
fn panel(this: &MissionPlanner, cx: &mut Context<MissionPlanner>) -> Option<AnyElement> {
    let manager = &this.plugins.manager;
    let form = manager.form.as_ref()?;
    let widths = column_widths(&form.rows);
    let grid_width: f32 = widths.iter().sum::<f32>() + EXERCISE_WIDTH;

    let mut header = div()
        .flex()
        .h(px(ROW_HEIGHT))
        .bg(rgb(theme::PANEL))
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .text_color(rgb(theme::DIM));
    for (text, width) in HEADERS.iter().zip(widths) {
        header = header.child(cell(width, *text));
    }
    header = header.child(cell(EXERCISE_WIDTH, EXERCISE_TEXT));
    let mut grid = div()
        .w(px(grid_width))
        .flex()
        .flex_col()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .child(header);
    for (index, row) in form.rows.iter().enumerate() {
        let id = SharedString::from(format!("plugin-manager-enabled-{index}"));
        let tick = crate::probe::measured(id.clone(), div())
            .id(id)
            .size(px(13.0))
            .flex()
            .items_center()
            .justify_center()
            .border_1()
            .border_color(rgb(theme::TEXT))
            .bg(rgb(theme::BG))
            .cursor_pointer()
            // The mark always drawn, in the box's own colour when not ticked: the probe measures a
            // control by its children, and an empty box was no control a script could click.
            .child(div().size(px(7.0)).bg(rgb(if row.enabled {
                theme::TEXT
            } else {
                theme::BG
            })))
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.plugins.manager.toggle(index);
                cx.notify();
            }));
        let mut exercise = cell(EXERCISE_WIDTH, "").justify_center();
        if row.plugin.is_some() {
            let id = SharedString::from(format!("plugin-manager-try-{index}"));
            let open = manager.exercising == Some(index);
            exercise = exercise.child(
                crate::probe::measured(id.clone(), div())
                    .id(id)
                    .px_2()
                    .border_1()
                    .border_color(rgb(theme::TEXT))
                    .rounded_sm()
                    .bg(rgb(if open { theme::ACCENT } else { theme::BG }))
                    .cursor_pointer()
                    .child(if open { TRY_OPEN_TEXT } else { TRY_TEXT })
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        this.plugins.manager.toggle_exercise(index);
                        cx.notify();
                    })),
            );
        }
        grid = grid.child(
            div()
                .flex()
                .h(px(ROW_HEIGHT))
                .bg(rgb(row.colour.rgb()))
                .text_color(rgb(theme::TEXT))
                .child(cell(widths[0], row.name.clone()))
                .child(cell(widths[1], row.author.clone()))
                .child(cell(widths[2], row.version.clone()))
                .child(cell(widths[3], row.dll.clone()))
                // The box named by its file too, so a plugin finds its own row: the
                // Welcome-Demo-Sitl unticks itself (the owner, 2026-10-05).
                .child(
                    cell(widths[4], "")
                        .justify_center()
                        .child(crate::probe::measured(
                            format!("plugin-manager-enabled-{}", row.dll),
                            div().child(tick),
                        )),
                )
                .child(exercise),
        );
    }

    // `bSave`, `labelWarning` (`Visible = bRestartRequired`) and `but_errors`, left to right.
    let mut top = div().flex().items_center().gap_3().child(crate::ui::action(
        "plugin-manager-save",
        SAVE_TEXT,
        theme::ACCENT,
        true,
        cx.listener(|this, _event: &(), _window, cx| {
            this.plugin_manager_save();
            cx.notify();
        }),
    ));
    if manager.restart_required {
        top = top.child(
            crate::probe::measured("plugin-manager-warning", div())
                .max_w(px(260.0))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(WARNING_TEXT),
        );
    }
    top = top.child(crate::ui::action(
        "plugin-manager-errors",
        ERRORS_TEXT,
        theme::TEXT,
        true,
        cx.listener(|this, _event: &(), _window, cx| {
            this.plugins.manager.show_errors();
            cx.notify();
        }),
    ));
    let mut client = div().flex().flex_col().gap_3().p(px(9.0)).child(top).child(grid);

    // The owner's Exercise: the plugin's map menu entries, each run as the map's menu runs it.
    if let Some((name, plugin)) = manager.exercised() {
        let entries = this.plugins.entries_of(plugin);
        let mut list = crate::probe::measured("plugin-manager-exercise", div())
            .flex()
            .flex_col()
            .gap_1()
            .p_2()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .rounded_sm()
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(format!("{name}: {RUNS_AT_TEXT}")),
            );
        if entries.is_empty() {
            list = list.child(div().text_sm().child(NO_ENTRIES_TEXT));
        }
        for entry in entries {
            let (plugin, id) = (entry.plugin, entry.id);
            let which = match entry.menu {
                mp_plugin_host::MapMenu::FlightData => "Flight Data map",
                mp_plugin_host::MapMenu::FlightPlanner => "Flight Plan map",
            };
            let probe = SharedString::from(format!("plugin-manager-run-{plugin}-{id}"));
            list = list.child(
                crate::probe::measured(probe.clone(), div())
                    .id(probe)
                    .px_2()
                    .py_1()
                    .rounded_sm()
                    .text_sm()
                    .cursor_pointer()
                    .hover(|style| style.bg(rgb(theme::BG)))
                    .child(format!("{which}: {}", entry.label()))
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        let (lat, lng) = exercise_point(this);
                        this.plugins.run_entry(plugin, id, lat, lng);
                        cx.notify();
                    })),
            );
        }
        client = client.child(list);
    }

    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(div().text_xs().text_color(rgb(theme::DIM)).child(FORM_TEXT));
    let mut body = crate::probe::measured("plugin-manager", div())
        .id("plugin-manager-form")
        .relative()
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .occlude()
        .child(caption)
        .child(client);
    // Show Errors: `CustomMessageBox.Show(msg, "Errors")`, modal over the form.
    if let Some(text) = &form.errors {
        body = body.child(
            div()
                .absolute()
                .inset_0()
                .flex()
                .items_center()
                .justify_center()
                .child(
                    crate::probe::measured("plugin-manager-errors-box", div())
                        .flex()
                        .flex_col()
                        .gap_2()
                        .max_w(px(grid_width - 40.0))
                        .p_3()
                        .bg(rgb(theme::PANEL))
                        .border_1()
                        .border_color(rgb(theme::ACCENT))
                        .rounded_md()
                        .child(div().text_xs().text_color(rgb(theme::DIM)).child("Errors"))
                        .children(text.lines().map(|line| {
                            div()
                                .text_sm()
                                .text_color(rgb(theme::TEXT))
                                .child(line.to_owned())
                        }))
                        .child(div().flex().justify_end().child(crate::ui::action(
                            "plugin-manager-errors-ok",
                            "OK",
                            theme::ACCENT,
                            true,
                            cx.listener(|this, _event: &(), _window, cx| {
                                this.plugins.manager.dismiss_errors();
                                cx.notify();
                            }),
                        ))),
                ),
        );
    }
    Some(body.into_any_element())
}

/// The PLUGINS tab: the plugin manager in the screen (the owner's addition, 2026-10-04).
pub(crate) fn screen(this: &MissionPlanner, cx: &mut Context<MissionPlanner>) -> AnyElement {
    div()
        .id("plugins-body")
        .flex_1()
        .min_h(px(0.0))
        .overflow_y_scroll()
        .p_4()
        .children(panel(this, cx))
        .into_any_element()
}

#[cfg(test)]
mod tests {
    use super::*;
    use mp_plugin_host::Info;

    fn running(name: &str, file: &str) -> PluginStatus {
        PluginStatus {
            file: file.to_owned(),
            info: Some(Info {
                name: name.to_owned(),
                version: "0.1".to_owned(),
                author: "Michael Oborne".to_owned(),
                file: file.to_owned(),
            }),
            state: PluginState::Running,
        }
    }

    fn not_loaded(file: &str, reason: &str) -> PluginStatus {
        PluginStatus {
            file: file.to_owned(),
            info: None,
            state: PluginState::NotLoaded(reason.to_owned()),
        }
    }

    /// Green for a loaded plugin; dark orange for one loaded and on the disabled list (matched
    /// without regard to case); a plugin whose Init or Loaded said no is not in `Plugins`, so not
    /// a row; dark red "Not loaded" for a disabled name with a file and no loaded plugin; nothing
    /// for a disabled name with no file.
    #[test]
    fn the_rows_are_populate_grid_views() {
        let plugins = [
            running("Dowding", "Dowding.wasm"),
            running("Fence Dist", "fencedist.wasm"),
            not_loaded("example.wasm", INIT_REFUSED),
        ];
        let disabled = [
            "fencedist.wasm".to_owned(),
            "menu.wasm".to_owned(),
            "gone.wasm".to_owned(),
        ];
        let rows = rows(&plugins, &disabled, |name| name != "gone.wasm");
        assert_eq!(
            rows,
            [
                Row {
                    name: "Dowding".to_owned(),
                    author: "Michael Oborne".to_owned(),
                    version: "0.1".to_owned(),
                    dll: "dowding.wasm".to_owned(),
                    enabled: true,
                    colour: RowColour::Green,
                    plugin: Some(0),
                },
                Row {
                    name: "Fence Dist".to_owned(),
                    author: "Michael Oborne".to_owned(),
                    version: "0.1".to_owned(),
                    dll: "fencedist.wasm".to_owned(),
                    enabled: false,
                    colour: RowColour::DarkOrange,
                    plugin: Some(1),
                },
                Row {
                    name: "Not loaded".to_owned(),
                    author: "--".to_owned(),
                    version: "--".to_owned(),
                    dll: "menu.wasm".to_owned(),
                    enabled: false,
                    colour: RowColour::DarkRed,
                    plugin: None,
                },
            ]
        );
    }

    /// `UpdateDisabledPlugins`: the rows not ticked, lower-cased; none, an empty list (which the
    /// window writes by removing the setting).
    #[test]
    fn save_lists_the_rows_not_ticked() {
        let plugins = [running("Dowding", "Dowding.wasm"), running("Menu", "menu.wasm")];
        let mut rows = rows(&plugins, &[], |_| true);
        assert!(disabled_after(&rows).is_empty());
        rows[0].enabled = false;
        assert_eq!(disabled_after(&rows), ["dowding.wasm"]);
    }

    /// Show Errors: the plugins that could not be loaded, a line each; one whose Init said no is
    /// not an error; none, "No Errors".
    #[test]
    fn show_errors_lists_what_could_not_be_loaded() {
        let clean = [running("Dowding", "dowding.wasm"), not_loaded("example.wasm", INIT_REFUSED)];
        assert_eq!(error_text(&clean), "No Errors");
        let broken = [
            not_loaded("bad.wasm", "bad.wasm is not a plugin: magic header not detected"),
            not_loaded("worse.wasm", "worse.wasm: permission denied"),
        ];
        assert_eq!(
            error_text(&broken),
            "bad.wasm is not a plugin: magic header not detected\nworse.wasm: permission denied\n"
        );
    }

    /// The form's life: opened fresh, a box ticked off, Save && Close gives the list and closes,
    /// and the restart warning stays for the next opening; Show Errors opens its box and OK
    /// closes it; the close box saves nothing.
    #[test]
    fn open_untick_save_and_the_warning_after() {
        let plugins = [running("Dowding", "dowding.wasm"), running("Menu", "menu.wasm")];
        let mut manager = PluginManager::default();
        assert!(!manager.is_open());
        assert_eq!(manager.save(), None);
        manager.show(&plugins, &[], |_| true);
        assert!(manager.is_open());
        assert!(!manager.restart_required);
        manager.toggle(1);
        manager.toggle(7);
        assert_eq!(manager.save(), Some(vec!["menu.wasm".to_owned()]));
        assert!(!manager.is_open());
        assert!(manager.restart_required);

        manager.show(&plugins, &["menu.wasm".to_owned()], |_| true);
        assert_eq!(manager.opened, 2);
        let form = manager.form.as_ref().expect("open");
        assert_eq!(form.rows[1].colour, RowColour::DarkOrange);
        manager.show_errors();
        assert_eq!(
            manager.form.as_ref().and_then(|form| form.errors.as_deref()),
            Some("No Errors")
        );
        manager.dismiss_errors();
        assert_eq!(manager.form.as_ref().and_then(|form| form.errors.as_deref()), None);
        manager.close();
        assert!(!manager.is_open());
        assert!(manager.restart_required);
    }

    /// Exercise: a loaded row's Try opens its plugin's list and closes it again; another row's
    /// moves the list there; a "Not loaded" row has none; closing the form puts it away.
    #[test]
    fn try_lists_a_loaded_plugins_entries() {
        let plugins = [
            running("Dowding", "dowding.wasm"),
            running("Menu", "menu.wasm"),
        ];
        let mut manager = PluginManager::default();
        manager.show(&plugins, &["gone.wasm".to_owned()], |_| true);
        assert_eq!(manager.exercised(), None);
        manager.toggle_exercise(1);
        assert_eq!(manager.exercised(), Some(("Menu", 1)));
        manager.toggle_exercise(0);
        assert_eq!(manager.exercised(), Some(("Dowding", 0)));
        manager.toggle_exercise(0);
        assert_eq!(manager.exercised(), None);
        manager.toggle_exercise(2);
        assert_eq!(manager.exercised(), None, "a Not loaded row");
        manager.toggle_exercise(1);
        manager.close();
        assert_eq!(manager.exercised(), None);
    }

    /// The columns are at least the designer's widths, and wider for longer text.
    #[test]
    fn columns_fit_their_text() {
        let plugins = [running("A very long plugin name indeed", "x.wasm")];
        let widths = column_widths(&rows(&plugins, &[], |_| true));
        // "0.1" and "x.wasm" are narrower than Version's and FileName's designer widths.
        assert_eq!(widths[2], COLUMN_WIDTHS[2]);
        assert_eq!(widths[3], COLUMN_WIDTHS[3]);
        // The name and "Michael Oborne" are wider than theirs.
        assert!(widths[0] > COLUMN_WIDTHS[0]);
        assert!(widths[1] > COLUMN_WIDTHS[1]);
    }
}
