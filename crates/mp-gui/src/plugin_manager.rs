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
//! * Not in the C#, the owner's complaint of 2026-10-16 - "we appear to have *two* tailscale
//!   plugins, and i still have NO IDEA HOW TO USE THEM": one native row, Tailscale, and under the
//!   grid while it is ticked the Tailscale panel - how to use it in three numbered lines and a
//!   link to the guide, this computer's Tailscale as the connection area's indicator says it, and
//!   the tick of the planner's built-in node, which had been a row of its own.
//!
//! `// C#: Plugin/PluginUI.cs:14-149; Plugin/PluginUI.Designer.cs:31-178; MainV2.cs:4125-4129`

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

/// A feature of this application listed among the plugins and ticked or unticked as they are,
/// but native code, not a wasm plugin: the owner's word of 2026-10-14, "its going to look like a
/// plugin, smell like a plugin, but not be an actual wasm plugin". A plugin's sandbox gives it no
/// processes, files or sockets of its own (mp-plugin-host's `plugin.wit`), which these need. Not
/// in the C#. Its key in `DisabledPlugins` is its `file`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct NativePlugin {
    /// The Plugin Name column.
    pub name: &'static str,
    /// The Author column.
    pub author: &'static str,
    /// The Version column.
    pub version: &'static str,
    /// The FileName column, and the key `DisabledPlugins` names it by.
    pub file: &'static str,
    /// Off until a user ticks it and saves, where the others are on until unticked: an
    /// experimental feature, which a `DisabledPlugins` saved before it existed must not turn on
    /// (`plugins_ui`'s `EnabledNativePlugins`).
    pub opt_in: bool,
}

/// Tailscale: the connection area's indicator of this computer's tailnet, the first start's offer
/// of the official client and the status line's reason when a tailnet address fails - the owner's
/// plan of 2026-10-14. On until it is unticked, as plugins are.
pub(crate) const TAILSCALE: NativePlugin = NativePlugin {
    name: "Tailscale",
    author: "MissionPlannerRust",
    version: "built in",
    file: "tailscale.native",
    opt_in: false,
};

/// The built-in Tailscale node, experimental - the owner's of 2026-10-14: tailscale-rs on the
/// planner's own thread, a node of its own on a tailnet with no Tailscale client installed, which
/// the links to the tailnet's hosts dial through, its names (`white`, `buzz55`) as well as its
/// addresses (mp-tailnet; mp-transport's `tailnet`). Since the owner's complaint of 2026-10-16
/// ("we appear to have *two* tailscale plugins") no row of the grid but the Tailscale panel's
/// tick, `name` its words; its key in `DisabledPlugins` is the one it had as a row, so a choice
/// saved then carries over. Off until it is ticked and saved; saving it ticked starts it at once,
/// unticked stops it - and it runs only while the Tailscale row is on as well (`plugins_ui`). A
/// build without mp-gui's `tailnet-node` feature has the tick, and once it is ticked and saved
/// says it has no node. Not in a web page, whose tailnet is the page's own.
#[cfg(not(target_family = "wasm"))]
pub(crate) const TAILSCALE_NODE: NativePlugin = NativePlugin {
    name: "Use the planner's built-in node (experimental)",
    author: "MissionPlannerRust",
    version: "built in",
    file: "tailscale-node.native",
    opt_in: true,
};

/// Import from Mission Planner: the first start's offer of Mission Planner's settings and logs
/// (mp_import.rs) - the owner's proposal of 2026-10-14, "this first-time-user-migrate *MIGHT* be
/// best done as a disableable plugin", approved the same day as a row like Tailscale's. On until
/// it is unticked.
pub(crate) const MP_IMPORT: NativePlugin = NativePlugin {
    name: "Import from Mission Planner",
    author: "MissionPlannerRust",
    version: "built in",
    file: "mpimport.native",
    opt_in: false,
};

/// The native features the manager lists as rows, after the plugins: Tailscale and the import
/// from Mission Planner. The built-in node is none (the owner's complaint of 2026-10-16): its
/// tick is the Tailscale panel's, under the grid.
#[cfg(not(target_family = "wasm"))]
pub(crate) const NATIVE: &[NativePlugin] = &[TAILSCALE, MP_IMPORT];
/// The native features the manager lists, after the plugins: in a web page Tailscale alone - a
/// page has no Mission Planner folder to import from, and no built-in node.
#[cfg(target_family = "wasm")]
pub(crate) const NATIVE: &[NativePlugin] = &[TAILSCALE];

/// Every native feature `DisabledPlugins` keys: the grid's rows ([`NATIVE`]) and the built-in
/// node, whose switch is the Tailscale panel's tick.
#[cfg(not(target_family = "wasm"))]
pub(crate) const NATIVE_SWITCHES: &[NativePlugin] = &[TAILSCALE, MP_IMPORT, TAILSCALE_NODE];
/// Every native feature `DisabledPlugins` keys: in a web page the grid's rows alone.
#[cfg(target_family = "wasm")]
pub(crate) const NATIVE_SWITCHES: &[NativePlugin] = NATIVE;

/// What the built-in node's sign-in link says, under its tick.
#[cfg(not(target_family = "wasm"))]
const SIGN_IN_TEXT: &str = "Sign in to the tailnet (opens the browser)";

/// The Tailscale panel under the grid, the owner's complaint of 2026-10-16 ("i still have NO IDEA
/// HOW TO USE THEM"): how to use it, in three numbered lines.
#[cfg(not(target_family = "wasm"))]
pub(crate) const TAILSCALE_STEPS: [&str; 3] = [
    "1. Install Tailscale on this computer and on the vehicle's computer, and sign both into your \
     tailnet (or tick the built-in node below and open its sign-in link).",
    "2. In the port box choose Tailscale.",
    "3. CONNECT and type the vehicle's tailnet name or 100.x address.",
];
/// The panel's link to the guide, using_tailscale.md, opened as HELP's "Using Tailscale" opens it.
#[cfg(not(target_family = "wasm"))]
const GUIDE_TEXT: &str = "Guide";
/// What heads the panel's line of this computer's Tailscale.
#[cfg(not(target_family = "wasm"))]
const COMPUTER_TEXT: &str = "This computer:";
/// What the node's line adds while its tick is ticked and no node runs yet: the tick takes effect
/// at Save && Close, as the grid's do.
#[cfg(not(target_family = "wasm"))]
const NODE_AT_SAVE_TEXT: &str = "Save & Close starts it";

/// Whether a native feature is on: not in the disabled list, compared as plugins' files are.
#[must_use]
pub(crate) fn native_enabled(feature: &NativePlugin, disabled: &[String]) -> bool {
    !disabled
        .iter()
        .any(|off| off.eq_ignore_ascii_case(feature.file))
}

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
/// `present` (in the plugins folder, or built in); and last, not the C#'s, the native features
/// ([`NATIVE`]), so a plugin's row is where it always was.
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
        // A native feature's key is no plugin's: the built-in node's included, which has no row.
        if NATIVE_SWITCHES
            .iter()
            .any(|feature| feature.file.eq_ignore_ascii_case(off))
        {
            continue;
        }
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
    for feature in NATIVE {
        let enabled = native_enabled(feature, disabled);
        out.push(Row {
            name: feature.name.to_owned(),
            author: feature.author.to_owned(),
            version: feature.version.to_owned(),
            dll: feature.file.to_owned(),
            enabled,
            colour: if enabled {
                RowColour::Green
            } else {
                RowColour::DarkOrange
            },
            plugin: None,
        });
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
    /// The Tailscale panel's tick of the built-in node ([`TAILSCALE_NODE`]): as the disabled list
    /// in force had it when the form opened, and turned by a click as a row's Enabled box is.
    #[cfg(not(target_family = "wasm"))]
    node: bool,
    /// Show Errors' message box, while it shows: `CustomMessageBox.Show(msg, "Errors")`.
    errors: Option<String>,
    /// What Show Errors will say, read when the form opened.
    error_text: String,
}

impl Form {
    /// What Save && Close writes: the rows not ticked ([`disabled_after`]), and the built-in
    /// node's key while its tick is not - as its row's was, when it had one.
    #[cfg_attr(target_family = "wasm", allow(unused_mut))]
    fn disabled_after(&self) -> Vec<String> {
        let mut list = disabled_after(&self.rows);
        #[cfg(not(target_family = "wasm"))]
        if !self.node {
            list.push(TAILSCALE_NODE.file.to_owned());
        }
        list
    }
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
    /// The last page the Tailscale panel handed to the desktop's browser - the guide, or the
    /// node's sign-in page - for the facts.
    tailscale_opened: Option<String>,
}

impl PluginManager {
    /// `new PluginUI().Show()`: a fresh form over the plugins as they are now.
    /// `// C#: Plugin/PluginUI.cs:18-24; MainV2.cs:4125-4129`
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
            #[cfg(not(target_family = "wasm"))]
            node: native_enabled(&TAILSCALE_NODE, disabled),
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

    /// Whether the open form's Tailscale row is ticked: the Tailscale panel shows under the grid
    /// while it is, and goes as soon as it is unticked, before Save && Close.
    #[cfg(not(target_family = "wasm"))]
    #[must_use]
    pub(crate) fn tailscale_ticked(&self) -> bool {
        self.form.as_ref().is_some_and(|form| {
            form.rows
                .iter()
                .any(|row| row.enabled && row.dll.eq_ignore_ascii_case(TAILSCALE.file))
        })
    }

    /// The Tailscale panel's tick of the built-in node, while the form is open.
    #[cfg(not(target_family = "wasm"))]
    #[must_use]
    pub(crate) fn node_ticked(&self) -> Option<bool> {
        self.form.as_ref().map(|form| form.node)
    }

    /// A click on the Tailscale panel's tick of the built-in node.
    #[cfg(not(target_family = "wasm"))]
    pub(crate) fn toggle_node(&mut self) {
        if let Some(form) = self.form.as_mut() {
            form.node = !form.node;
        }
    }

    /// `bSave_Click`: the disabled list for `DisabledPlugins` (`None` when the form is not open),
    /// a restart required, and the form closed. The list names the built-in node while its tick
    /// is not ticked, as it named its row.
    /// `// C#: Plugin/PluginUI.cs:86-91`
    pub(crate) fn save(&mut self) -> Option<Vec<String>> {
        let form = self.form.take()?;
        self.restart_required = true;
        Some(form.disabled_after())
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
    /// row's name, file, Enabled box and colour, the built-in node's tick, the errors box, and the
    /// last page the Tailscale panel opened.
    pub(crate) fn record_facts(&self) {
        facts::record("plugin-manager.open", self.is_open());
        facts::record("plugin-manager.opened", self.opened);
        facts::record("plugin-manager.restart-required", self.restart_required);
        facts::record(
            "plugin-manager.tailscale.opened",
            self.tailscale_opened.as_deref().unwrap_or("none"),
        );
        let Some(form) = &self.form else { return };
        facts::record("plugin-manager.rows", form.rows.len());
        #[cfg(not(target_family = "wasm"))]
        facts::record("plugin-manager.tailnet-node.tick", form.node);
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
    // Next to the grid whose Try opened it, above the Tailscale panel.
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

    // The Tailscale panel, while the Tailscale row is ticked (the owner's complaint of
    // 2026-10-16): as wide as the grid, its lines wrapped to it.
    #[cfg(not(target_family = "wasm"))]
    if let Some(tailscale) = this.tailscale_panel() {
        client = client.child(tailscale_panel_view(tailscale, grid_width, cx));
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

/// The Tailscale panel's words as they stand: drawn under the grid, and given to the facts.
#[cfg(not(target_family = "wasm"))]
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct TailscalePanel {
    /// This computer's Tailscale - the official client's - as the connection area's indicator
    /// says it: its dot, its words and its hover's account (`tailscale::shown`). The built-in node
    /// has a line of its own under its tick, so here it does not speak for this computer.
    pub computer: crate::tailscale::Shown,
    /// The built-in node's tick.
    pub node_ticked: bool,
    /// Under the tick while it is ticked: how the node stands, as its line under the grid said it
    /// when it was a row - or, no node running yet, that Save && Close starts it.
    pub node_line: Option<String>,
    /// The page the node waits for its sign-in at, while it does and its tick is ticked.
    pub sign_in: Option<String>,
}

/// The Tailscale panel's words: from this computer's Tailscale as the indicator's poller has it,
/// the node's tick, and the node while one runs (`plugins_ui`'s `tailnet_state`).
#[cfg(not(target_family = "wasm"))]
#[must_use]
pub(crate) fn panel_words(
    computer: &crate::tailscale::TailnetState,
    node_ticked: bool,
    node: Option<&mp_tailnet::State>,
) -> TailscalePanel {
    let (node_line, sign_in) = match (node_ticked, node) {
        (false, _) => (None, None),
        (true, None) => (
            Some(format!(
                "{} ({NODE_AT_SAVE_TEXT})",
                mp_tailnet::State::Stopped
            )),
            None,
        ),
        (true, Some(state)) => {
            let sign_in = match state {
                mp_tailnet::State::NeedsSignIn { url } => Some(url.clone()),
                _ => None,
            };
            (Some(state.to_string()), sign_in)
        }
    };
    TailscalePanel {
        computer: crate::tailscale::shown(None, computer),
        node_ticked,
        node_line,
        sign_in,
    }
}

#[cfg(not(target_family = "wasm"))]
impl MissionPlanner {
    /// The Tailscale panel, while the plugin manager is open with its Tailscale row ticked.
    pub(crate) fn tailscale_panel(&self) -> Option<TailscalePanel> {
        let manager = &self.plugins.manager;
        let node_ticked = manager.node_ticked()?;
        manager.tailscale_ticked().then(|| {
            panel_words(
                &self.tailnet_state(),
                node_ticked,
                self.plugins.tailnet_state().as_ref(),
            )
        })
    }

    /// A page of the Tailscale panel handed to the desktop's browser, as HELP's links hand
    /// theirs: kept for the facts once opened; a failure, "could not open `what`: why", on the
    /// status line.
    fn tailscale_panel_open(&mut self, url: &str, what: &str) {
        match crate::scripts_tab::open_with_shell(std::path::Path::new(url)) {
            Ok(()) => self.plugins.manager.tailscale_opened = Some(url.to_owned()),
            Err(why) => self.file_status = Some(format!("could not open {what}: {why}")),
        }
    }

    /// The Tailscale panel's facts while the plugin manager is open: whether it shows
    /// (`plugin-manager.tailscale.panel`), and while it does its three lines
    /// (`plugin-manager.tailscale.step.1` to `.3`), this computer's words and account
    /// (`plugin-manager.tailscale.computer`, `.computer.details`), the node's line under its tick
    /// (`plugin-manager.tailnet-node.line`) and its sign-in page
    /// (`plugin-manager.tailnet-node.sign-in`), each `none` while it is not drawn.
    pub(crate) fn tailscale_panel_facts(&self) {
        if !self.plugins.manager.is_open() {
            return;
        }
        let panel = self.tailscale_panel();
        facts::record("plugin-manager.tailscale.panel", panel.is_some());
        let Some(panel) = panel else { return };
        for (index, step) in TAILSCALE_STEPS.iter().enumerate() {
            facts::record(format!("plugin-manager.tailscale.step.{}", index + 1), step);
        }
        facts::record("plugin-manager.tailscale.computer", &panel.computer.text);
        facts::record(
            "plugin-manager.tailscale.computer.details",
            &panel.computer.details,
        );
        facts::record(
            "plugin-manager.tailnet-node.line",
            panel.node_line.as_deref().unwrap_or("none"),
        );
        facts::record(
            "plugin-manager.tailnet-node.sign-in",
            panel.sign_in.as_deref().unwrap_or("none"),
        );
    }
}

/// One of the panel's links, drawn as this application's are (`ui::box_text`'s): the link blue,
/// underlined and lighter under the pointer, in a row of its own so that only its words take the
/// click.
#[cfg(not(target_family = "wasm"))]
fn panel_link(
    id: &'static str,
    text: &'static str,
    on_click: impl Fn(&gpui::ClickEvent, &mut gpui::Window, &mut gpui::App) + 'static,
) -> gpui::Div {
    div().flex().child(
        crate::probe::measured(id, div())
            .id(id)
            .text_sm()
            .text_color(rgb(theme::LINK))
            .underline()
            .cursor_pointer()
            .hover(|style| style.text_color(rgb(theme::LINK_HOVER)))
            .child(text)
            .on_click(on_click),
    )
}

/// The Tailscale panel under the grid, while the Tailscale row is ticked (the owner's complaint
/// of 2026-10-16): its name and the guide's link; how to use it, in three numbered lines; this
/// computer's Tailscale as the connection area's indicator says it, its account beneath; and the
/// built-in node's tick - saved by Save && Close, as the grid's ticks are - with how the node
/// stands under it while it is ticked, and its sign-in link while it waits for one. `width` is
/// the grid's, which the lines wrap to.
#[cfg(not(target_family = "wasm"))]
fn tailscale_panel_view(
    panel: TailscalePanel,
    width: f32,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    // A line held to the panel's width: a flex item's least width is its content's, and a long
    // line would widen the form past the grid.
    let line = |text: String| {
        div()
            .w_full()
            .min_w_0()
            .text_sm()
            .text_color(rgb(theme::TEXT))
            .child(text)
    };
    let heading = div()
        .flex()
        .items_center()
        .justify_between()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(TAILSCALE.name),
        )
        .child(panel_link(
            "plugin-manager-tailscale-guide",
            GUIDE_TEXT,
            cx.listener(|this, _event: &gpui::ClickEvent, _window, cx| {
                this.tailscale_panel_open(crate::help::TAILSCALE_GUIDE_URL, "the Tailscale guide");
                cx.notify();
            }),
        ));
    let steps = div()
        .flex()
        .flex_col()
        .gap_1()
        .children(TAILSCALE_STEPS.iter().map(|step| line((*step).to_owned())));
    let computer = crate::probe::measured("plugin-manager-tailscale-computer", div())
        .flex()
        .flex_col()
        .gap_1()
        .child(
            div()
                .flex()
                .items_center()
                .gap_2()
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child(COMPUTER_TEXT),
                )
                .child(
                    div()
                        .size_2()
                        .flex_shrink_0()
                        .rounded_full()
                        .bg(rgb(panel.computer.colour)),
                )
                .child(
                    div()
                        .min_w_0()
                        .text_sm()
                        .text_color(rgb(theme::TEXT))
                        .child(panel.computer.text),
                ),
        )
        .child(
            div()
                .w_full()
                .min_w_0()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(panel.computer.details),
        );
    // The tick drawn as the grid's Enabled boxes are - its mark always there, in the box's own
    // colour when not ticked - the box and its words one control.
    let tick = crate::probe::measured("plugin-manager-tailnet-node-tick", div())
        .id("plugin-manager-tailnet-node-tick")
        .flex()
        .items_center()
        .gap_2()
        .cursor_pointer()
        .child(
            div()
                .size(px(13.0))
                .flex_shrink_0()
                .flex()
                .items_center()
                .justify_center()
                .border_1()
                .border_color(rgb(theme::TEXT))
                .bg(rgb(theme::BG))
                .child(div().size(px(7.0)).bg(rgb(if panel.node_ticked {
                    theme::TEXT
                } else {
                    theme::BG
                }))),
        )
        .child(
            div()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(TAILSCALE_NODE.name),
        )
        .on_click(cx.listener(|this, _event: &gpui::ClickEvent, _window, cx| {
            this.plugins.manager.toggle_node();
            cx.notify();
        }));
    let mut body = crate::probe::measured("plugin-manager-tailscale", div())
        .w(px(width))
        .flex()
        .flex_col()
        .gap_2()
        .p_2()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_sm()
        .child(heading)
        .child(steps)
        .child(computer)
        .child(div().flex().child(tick));
    // Under the tick, in line with its words: how the node stands, and its sign-in page.
    if let Some(state) = panel.node_line {
        let mut node = crate::probe::measured("plugin-manager-tailnet-node", div())
            .flex()
            .flex_col()
            .gap_1()
            .pl(px(21.0))
            .child(line(state));
        if let Some(url) = panel.sign_in {
            node = node.child(panel_link(
                "plugin-manager-tailnet-sign-in",
                SIGN_IN_TEXT,
                cx.listener(move |this, _event: &gpui::ClickEvent, _window, cx| {
                    this.tailscale_panel_open(&url, "the Tailscale sign-in page");
                    cx.notify();
                }),
            ));
        }
        body = body.child(node);
    }
    body.into_any_element()
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
    /// for a disabled name with no file. Last the two native rows - and none for the built-in
    /// node's key, in the disabled list as it is until its tick is ticked: neither a row of its
    /// own (its tick is the Tailscale panel's) nor a "Not loaded" one.
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
            "tailscale-node.native".to_owned(),
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
                Row {
                    name: "Tailscale".to_owned(),
                    author: "MissionPlannerRust".to_owned(),
                    version: "built in".to_owned(),
                    dll: "tailscale.native".to_owned(),
                    enabled: true,
                    colour: RowColour::Green,
                    plugin: None,
                },
                Row {
                    name: "Import from Mission Planner".to_owned(),
                    author: "MissionPlannerRust".to_owned(),
                    version: "built in".to_owned(),
                    dll: "mpimport.native".to_owned(),
                    enabled: true,
                    colour: RowColour::Green,
                    plugin: None,
                },
            ]
        );
    }

    /// A native row of the grid, by its key.
    fn native_row<'a>(rows: &'a [Row], file: &str) -> &'a Row {
        rows.iter()
            .find(|row| row.dll == file)
            .unwrap_or_else(|| panic!("no row for {file}"))
    }

    /// The owner's native Tailscale row (2026-10-14): after the plugins, so a plugin's row is
    /// where it was; on until unticked; unticked, a disabled row of its own name - not "Not
    /// loaded" - and its key in what Save writes; with no Try, as it is no plugin.
    #[test]
    fn tailscale_is_a_row_like_a_plugins_but_native() {
        let plugins = [running("Dowding", "Dowding.wasm")];
        let on = rows(&plugins, &[], |_| true);
        assert_eq!(on.len(), 1 + NATIVE.len());
        assert_eq!(on.first().map(|row| row.dll.as_str()), Some("dowding.wasm"));
        let tailscale = native_row(&on, TAILSCALE.file);
        assert_eq!(
            (tailscale.name.as_str(), tailscale.enabled, tailscale.plugin),
            ("Tailscale", true, None)
        );
        assert!(native_enabled(&TAILSCALE, &[]));
        const { assert!(!TAILSCALE.opt_in, "on until unticked") };
        // Import from Mission Planner, the other row on until unticked.
        assert!(native_enabled(&MP_IMPORT, &[]));
        const { assert!(!MP_IMPORT.opt_in) };
        assert_eq!(native_row(&on, MP_IMPORT.file).name, "Import from Mission Planner");

        let disabled = ["Tailscale.Native".to_owned()];
        assert!(!native_enabled(&TAILSCALE, &disabled));
        assert!(native_enabled(&MP_IMPORT, &disabled));
        let off = rows(&plugins, &disabled, |_| true);
        assert_eq!(off.len(), 1 + NATIVE.len(), "no Not loaded row for it");
        let tailscale = native_row(&off, TAILSCALE.file);
        assert_eq!(
            (tailscale.name.as_str(), tailscale.enabled, tailscale.colour),
            ("Tailscale", false, RowColour::DarkOrange)
        );

        let mut ticked = on;
        for row in &mut ticked {
            row.enabled = row.dll != TAILSCALE.file;
        }
        assert_eq!(disabled_after(&ticked), ["tailscale.native"]);
    }

    /// The owner's complaint of 2026-10-16, "we appear to have *two* tailscale plugins": the grid
    /// lists one Tailscale row and the import, two native rows; the built-in node (the owner's of
    /// 2026-10-14) is no row, whether the disabled list names it or not, and its key no "Not
    /// loaded" row - but the key is the one it had as a row, opt-in, still read as its switch, so
    /// a choice saved then carries over.
    #[test]
    fn the_tailscale_node_is_the_panels_tick_not_a_row() {
        assert_eq!(NATIVE, [TAILSCALE, MP_IMPORT]);
        assert_eq!(NATIVE_SWITCHES, [TAILSCALE, MP_IMPORT, TAILSCALE_NODE]);
        const { assert!(TAILSCALE_NODE.opt_in) };
        assert_eq!(TAILSCALE_NODE.file, "tailscale-node.native");
        assert_eq!(
            TAILSCALE_NODE.name,
            "Use the planner's built-in node (experimental)"
        );

        let plugins = [running("Dowding", "Dowding.wasm")];
        for disabled in [vec![], vec![TAILSCALE_NODE.file.to_owned()]] {
            let rows = rows(&plugins, &disabled, |_| true);
            assert_eq!(rows.len(), 1 + 2, "{disabled:?}");
            assert!(
                rows.iter().all(|row| row.dll != TAILSCALE_NODE.file),
                "{disabled:?}"
            );
            let tailscale: Vec<&str> = rows
                .iter()
                .filter(|row| row.name.contains("Tailscale"))
                .map(|row| row.dll.as_str())
                .collect();
            assert_eq!(tailscale, ["tailscale.native"], "one Tailscale row");
        }
        // The key, read as it was.
        assert!(!native_enabled(
            &TAILSCALE_NODE,
            &["Tailscale-Node.Native".to_owned()]
        ));
        assert!(native_enabled(
            &TAILSCALE_NODE,
            &["tailscale.native".to_owned()]
        ));
    }

    /// The Tailscale panel's tick is the node's key: ticked as the disabled list in force has it
    /// when the form opens - unticked while the list names the key, as it does until the tick is
    /// saved (`plugins_ui`); a click turns it, as a row's Enabled box; Save && Close writes the key
    /// into the list while it is unticked and leaves it out while it is ticked, beside the rows
    /// not ticked; and a click with no form open does nothing.
    #[test]
    fn the_node_tick_is_saved_as_the_nodes_key() {
        let plugins = [running("Dowding", "dowding.wasm")];
        let node = TAILSCALE_NODE.file.to_owned();
        let mut manager = PluginManager::default();
        assert_eq!(manager.node_ticked(), None);
        manager.toggle_node();
        assert_eq!(manager.node_ticked(), None);

        // Never ticked: unticked, and saved so.
        manager.show(&plugins, std::slice::from_ref(&node), |_| true);
        assert_eq!(manager.node_ticked(), Some(false));
        assert_eq!(manager.save(), Some(vec![node.clone()]));

        // Ticked and saved: the key out of the list, so the node is on.
        manager.show(&plugins, std::slice::from_ref(&node), |_| true);
        manager.toggle_node();
        assert_eq!(manager.node_ticked(), Some(true));
        let saved = manager.save().expect("open");
        assert!(saved.is_empty());
        assert!(native_enabled(&TAILSCALE_NODE, &saved));

        // Opened again over that: ticked; unticked with Dowding's row, both in the list.
        manager.show(&plugins, &saved, |_| true);
        assert_eq!(manager.node_ticked(), Some(true));
        manager.toggle_node();
        manager.toggle(0);
        let saved = manager.save().expect("open");
        assert_eq!(saved, ["dowding.wasm".to_owned(), node]);
        assert!(!native_enabled(&TAILSCALE_NODE, &saved));

        // The close box saves nothing.
        manager.show(&plugins, &saved, |_| true);
        manager.toggle_node();
        manager.close();
        assert_eq!(manager.save(), None);
    }

    /// The panel's three lines, word for word (the owner's complaint of 2026-10-16: "i still have
    /// NO IDEA HOW TO USE THEM"), numbered in order; its Guide link opens the guide HELP's "Using
    /// Tailscale" opens.
    #[test]
    fn the_panel_says_how_to_use_it_in_three_lines() {
        assert_eq!(
            TAILSCALE_STEPS,
            [
                "1. Install Tailscale on this computer and on the vehicle's computer, and sign \
                 both into your tailnet (or tick the built-in node below and open its sign-in \
                 link).",
                "2. In the port box choose Tailscale.",
                "3. CONNECT and type the vehicle's tailnet name or 100.x address.",
            ]
        );
        for (index, step) in TAILSCALE_STEPS.iter().enumerate() {
            assert!(step.starts_with(&format!("{}. ", index + 1)), "{step}");
        }
        assert_eq!(GUIDE_TEXT, "Guide");
        assert!(
            crate::help::TAILSCALE_GUIDE_URL.ends_with("/using_tailscale.md"),
            "{}",
            crate::help::TAILSCALE_GUIDE_URL
        );
    }

    /// The guide the Guide link opens starts with the panel's three lines, word for word, in its
    /// "The short version" before "What Tailscale does here", and names the port box's Tailscale
    /// kind and its two questions as CONNECT asks them.
    #[test]
    fn the_guide_opens_with_the_panels_three_lines() {
        let guide = include_str!("../../../using_tailscale.md");
        let short_at = guide
            .find("\n## The short version\n")
            .expect("its short version");
        let what_at = guide
            .find("\n## What Tailscale does here\n")
            .expect("its first long section");
        assert!(short_at < what_at, "the short version first");
        let short: String = guide
            .get(short_at..what_at)
            .unwrap_or_default()
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" ");
        for step in TAILSCALE_STEPS {
            assert!(short.contains(step), "{step}\nnot in\n{short}");
        }
        for words in ["**Tailscale**", "**tailnet host**", "**remote Port**"] {
            assert!(short.contains(words), "{words}");
        }
    }

    /// The panel shows while the open form's Tailscale row is ticked, and goes as soon as it is
    /// unticked, before Save && Close; a form opened with Tailscale off has none; no form, none.
    #[test]
    fn the_panel_follows_the_tailscale_rows_tick() {
        let mut manager = PluginManager::default();
        assert!(!manager.tailscale_ticked());
        manager.show(&[], &[], |_| true);
        assert!(manager.tailscale_ticked());
        let row = manager
            .form
            .as_ref()
            .and_then(|form| form.rows.iter().position(|row| row.dll == TAILSCALE.file))
            .expect("its row");
        manager.toggle(row);
        assert!(!manager.tailscale_ticked());
        manager.toggle(row);
        assert!(manager.tailscale_ticked());
        manager.show(&[], &[TAILSCALE.file.to_owned()], |_| true);
        assert!(!manager.tailscale_ticked());
        // The import's row is not Tailscale's.
        manager.show(&[], &[MP_IMPORT.file.to_owned()], |_| true);
        assert!(manager.tailscale_ticked());
    }

    /// The panel's words: this computer's line is what the connection area's indicator shows for
    /// the official client - its words, account and dot, `tailscale::shown`'s - whatever the node
    /// does, as the node has its own line; that line shows only while the node's tick is ticked:
    /// Save && Close starts it while none runs, then how it stands, with its sign-in page while it
    /// waits for one; and a build with no node says so once it has been saved ticked.
    #[test]
    fn the_panel_says_this_computers_tailscale_and_the_nodes() {
        use crate::tailscale::TailnetState;
        const URL: &str = "https://login.tailscale.com/a/1a2b3c";
        let signed_out = TailnetState::SignedOut { login_url: None };
        let sign_in = mp_tailnet::State::NeedsSignIn {
            url: URL.to_owned(),
        };

        let unticked = panel_words(&signed_out, false, Some(&sign_in));
        assert_eq!(
            unticked.computer,
            crate::tailscale::shown(None, &signed_out)
        );
        assert_eq!(unticked.computer.text, "Tailscale: signed out");
        assert_eq!(unticked.computer.details, signed_out.details());
        assert!(!unticked.node_ticked);
        assert_eq!((unticked.node_line, unticked.sign_in), (None, None));

        let before_save = panel_words(&signed_out, true, None);
        assert_eq!(
            before_save.node_line.as_deref(),
            Some("stopped (Save & Close starts it)")
        );
        assert_eq!(before_save.sign_in, None);

        let waiting = panel_words(&signed_out, true, Some(&sign_in));
        assert_eq!(
            waiting.computer.text, "Tailscale: signed out",
            "the node in use does not speak for this computer here"
        );
        assert_eq!(
            waiting.node_line.as_deref(),
            Some(format!("needs a sign-in at {URL}").as_str())
        );
        assert_eq!(waiting.sign_in.as_deref(), Some(URL));

        let running = mp_tailnet::State::Running {
            name: "mpr-buzz55.tail1234.ts.net".to_owned(),
            ipv4: std::net::Ipv4Addr::new(100, 101, 102, 104),
            ipv6: "fd7a:115c:a1e0::4".parse().expect("the test's own address"),
        };
        let on = panel_words(&TailnetState::NotInstalled, true, Some(&running));
        assert_eq!(on.computer.text, "Tailscale: not installed");
        assert_eq!(
            on.node_line.as_deref(),
            Some("running as mpr-buzz55.tail1234.ts.net (100.101.102.104, fd7a:115c:a1e0::4)")
        );
        assert_eq!(on.sign_in, None);

        let no_node = mp_tailnet::State::Failed {
            why: mp_tailnet::NO_NODE.to_owned(),
        };
        let failed = panel_words(&TailnetState::NotInstalled, true, Some(&no_node));
        assert_eq!(
            failed.node_line.as_deref(),
            Some("failed: this build has no built-in Tailscale node")
        );
        assert_eq!(failed.sign_in, None);
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
        let plugin_rows: Vec<Row> = rows(&plugins, &[], |_| true)
            .into_iter()
            .filter(|row| row.plugin.is_some())
            .collect();
        let widths = column_widths(&plugin_rows);
        // "0.1" and "x.wasm" are narrower than Version's and FileName's designer widths.
        assert_eq!(widths[2], COLUMN_WIDTHS[2]);
        assert_eq!(widths[3], COLUMN_WIDTHS[3]);
        // The name and "Michael Oborne" are wider than theirs.
        assert!(widths[0] > COLUMN_WIDTHS[0]);
        assert!(widths[1] > COLUMN_WIDTHS[1]);
    }
}
