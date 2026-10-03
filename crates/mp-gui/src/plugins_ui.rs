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

//! The WebAssembly plugins in the window: `PluginLoader.LoadAll` at start, and each frame what
//! the plugins did and asked, served through the application's own paths (PLAN.md §13.6 row 96).
//!
//! `mp_plugin_host` loads the plugins Mission Planner ships - built into the planner by
//! `build.rs`, as the C#'s build puts them in its plugins folder - and every `*.wasm` in
//! `plugins/` beside the executable, a file there replacing a built-in of its name; each runs on
//! a thread of its own. This module is the window's half. Once a frame [`MissionPlanner::plugins_tick`]
//! hands the plugins a fresh snapshot to read - the vehicle's state as `Host.cs`, the parameters,
//! the fence, the planning map's view - and serves what they asked since the last frame:
//!
//! * a parameter write through the link's `setParam` ([`crate::telemetry::Telemetry::write_parameter`]),
//!   a mode through the command the fly screen's mode buttons send, a packet through the link's
//!   queue - no second path to the vehicle;
//! * `AddWPtoList` and `InsertWP` through the plan's `AddCommand`, `GetWPs` through the planning
//!   screen's Read;
//! * `CustomMessageBox.Show`, `InputBox.Show` and the file dialogs as a modal box over the window,
//!   one at a time in the order asked, the plugin waiting on its thread for the answer (a file
//!   dialog is a typed path here, as every file dialog in this application is);
//! * the map menus' entries - the planning map's drawn at the end of its right-click menu
//!   (`plan.rs`), the flight map's in a panel of their own on the flight screen, since that map's
//!   `contextMenuStripMap` entries are the Actions tab's chips here;
//! * the form a plugin describes, drawn as a panel of labels, text boxes, check boxes, combo boxes
//!   and buttons, its changes sent back to the plugin;
//! * the status line, and `Host.config` as `Settings.Instance`.
//!
//! `MP_PLUGINS` names another plugins folder, for a test that must not put a plugin beside the
//! shared binary.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

/// The plugins Mission Planner ships, built for WebAssembly by `build.rs` and carried here.
mod builtin {
    include!(concat!(env!("OUT_DIR"), "/builtin_plugins.rs"));
}

use std::collections::{HashMap, VecDeque};
use std::path::{Component, Path, PathBuf};
use std::sync::{Arc, Mutex, OnceLock, PoisonError};

use gpui::{
    AnyElement, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px, rgb,
};
use mp_plugin_host::{
    Area, Control, ControlKind, CsValue, DialogResult, FencePoint, Limits, MapMenu, MessageButtons,
    OpenedFile, PluginHost, PluginState, Reply, RequestBody, Waypoint,
};
use mp_vehicle::{VehicleFamily, VehicleState};

use crate::MissionPlanner;
use crate::facts;
use crate::telemetry::TelemetryView;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action, theme};

/// The variable naming another plugins folder.
pub const ENV: &str = "MP_PLUGINS";

/// The setting `PluginLoader.DisabledPluginNames` is read from. `// C#: MainV2.cs:3192-3194`
const DISABLED: &str = "DisabledPlugins";

/// Where the plugins are: `MP_PLUGINS`, else `plugins` beside the executable.
/// `// C#: Plugin/PluginLoader.cs:205-206`
#[must_use]
pub fn folder() -> Option<PathBuf> {
    std::env::var_os(ENV)
        .map(PathBuf::from)
        .or_else(mp_plugin_host::plugins_dir)
}

/// A map menu entry a plugin added.
#[derive(Debug, Clone, PartialEq)]
pub struct Entry {
    /// The plugin's index.
    pub plugin: usize,
    /// The plugin's id for it.
    pub id: u32,
    /// Which map's menu.
    pub menu: MapMenu,
    /// The entry it drops down from.
    pub parent: Option<u32>,
    /// Its text.
    pub text: String,
}

impl Entry {
    /// Its probe id: `plugin-menu-<plugin>-<id>`.
    #[must_use]
    pub fn probe_id(&self) -> &'static str {
        static_id(format!("plugin-menu-{}-{}", self.plugin, self.id))
    }

    /// Its text, indented under its parent: a `DropDownItems` entry is drawn below the entry it
    /// drops from, not in a drop-down of its own.
    #[must_use]
    pub fn label(&self) -> String {
        if self.parent.is_some() {
            format!("    {}", self.text)
        } else {
            self.text.clone()
        }
    }
}

/// An id made once for the process and kept: gpui's and the probe's ids for controls whose
/// names come from a plugin. Bounded by the plugins' control ids, which a plugin names.
fn static_id(id: String) -> &'static str {
    static IDS: OnceLock<Mutex<HashMap<String, &'static str>>> = OnceLock::new();
    let mut ids = IDS
        .get_or_init(Mutex::default)
        .lock()
        .unwrap_or_else(PoisonError::into_inner);
    if let Some(id) = ids.get(&id) {
        return id;
    }
    let leaked: &'static str = Box::leak(id.clone().into_boxed_str());
    ids.insert(id, leaked);
    leaked
}

/// What a question's answer goes back as.
#[derive(Debug)]
enum Answer {
    Message(Reply<DialogResult>),
    Input(Reply<Option<String>>),
    Open(Reply<Option<OpenedFile>>),
    Save {
        data: Vec<u8>,
        reply: Reply<Option<String>>,
    },
}

/// A box a plugin is waiting on.
#[derive(Debug)]
struct Question {
    plugin: usize,
    caption: String,
    text: String,
    /// A message box's buttons; the others have OK and Cancel.
    buttons: MessageButtons,
    /// The text its box starts with, for the boxes that have one.
    value: Option<String>,
    answer: Answer,
}

/// A plugin's form, as the window holds it.
#[derive(Debug)]
struct Form {
    plugin: usize,
    title: String,
    controls: Vec<Control>,
    /// The text boxes' state, by control id.
    fields: HashMap<String, TextField>,
    /// Their focus.
    focus: HashMap<String, FocusHandle>,
    /// The combo box whose list is open.
    open_combo: Option<String>,
}

/// The plugins, and what the window shows of them.
pub struct Plugins {
    host: PluginHost,
    entries: Vec<Entry>,
    questions: VecDeque<Question>,
    /// The showing question's text box.
    field: TextField,
    focus: FocusHandle,
    /// Whether the showing question has been given the focus.
    focused: bool,
    forms: Vec<Form>,
    /// The last status line a plugin said.
    status: Option<String>,
}

impl std::fmt::Debug for Plugins {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Plugins")
            .field("plugins", &self.host.plugins())
            .field("entries", &self.entries)
            .field("questions", &self.questions.len())
            .field("forms", &self.forms.len())
            .finish_non_exhaustive()
    }
}

impl Plugins {
    /// `PluginLoader.LoadAll` over [`folder`], less `DisabledPlugins`: each plugin on its own
    /// thread, its terrain and parameter documentation readers set once.
    /// `// C#: MainV2.cs:3185-3196`
    pub fn start(persisted: &crate::settings::Persisted, cx: &mut gpui::App) -> Self {
        let disabled = crate::raw_params_grid::get_list(persisted.get(DISABLED));
        // `PluginLoader.LoadAll` at start: the shipped plugins and the folder's, less the
        // disabled ones. `// C#: Plugin/PluginLoader.cs:203-311`
        let host = PluginHost::load_with_builtins(
            builtin::BUILTIN,
            folder().as_deref(),
            &disabled,
            Limits::default(),
        );
        host.update_snapshot(|snapshot| {
            snapshot.terrain = Arc::new(|lat, lng| {
                let answer = crate::srtm::altitude(lat, lng);
                (answer.current_type == crate::srtm::TileType::Valid).then_some(answer.alt)
            });
            snapshot.options = Arc::new(|name| {
                crate::metadata::lookup(name).map_or_else(Vec::new, |meta| {
                    meta.values
                        .iter()
                        .filter_map(|(value, text)| {
                            Some((i32::try_from(*value).ok()?, (*text).to_owned()))
                        })
                        .collect()
                })
            });
        });
        Self {
            host,
            entries: Vec::new(),
            questions: VecDeque::new(),
            field: TextField::new(""),
            focus: cx.focus_handle(),
            focused: false,
            forms: Vec::new(),
            status: None,
        }
    }

    /// Whether any plugin file was found.
    fn any(&self) -> bool {
        !self.host.plugins().is_empty()
    }

    /// The planning map's entries, for its menu.
    #[must_use]
    pub fn planner_entries(&self) -> Vec<Entry> {
        self.entries
            .iter()
            .filter(|entry| entry.menu == MapMenu::FlightPlanner)
            .cloned()
            .collect()
    }

    /// The name a plugin goes by in facts and on the status line.
    fn name(&self, plugin: usize) -> String {
        self.host
            .plugins()
            .get(plugin)
            .map_or_else(String::new, |status| status.name().to_owned())
    }

    /// A question queued behind any showing; the first to show has its box filled.
    fn ask(&mut self, question: Question) {
        if self.questions.is_empty() {
            self.field.set(question.value.clone().unwrap_or_default());
            self.focused = false;
        }
        self.questions.push_back(question);
    }

    /// Everything of a plugin that has gone: its entries, its form, its questions.
    fn forget(&mut self, plugin: usize) {
        self.entries.retain(|entry| entry.plugin != plugin);
        self.forms.retain(|form| form.plugin != plugin);
        let showing = self.questions.front().map(|question| question.plugin);
        self.questions.retain(|question| question.plugin != plugin);
        if showing == Some(plugin) {
            let next = self
                .questions
                .front()
                .and_then(|question| question.value.clone());
            self.field.set(next.unwrap_or_default());
            self.focused = false;
        }
    }

    /// A plugin's form shown, or replaced with its text boxes' state kept.
    fn show_form(&mut self, plugin: usize, title: String, controls: Vec<Control>) {
        let slot = self.forms.iter().position(|form| form.plugin == plugin);
        let mut form = match slot {
            Some(index) => self.forms.remove(index),
            None => Form {
                plugin,
                title: String::new(),
                controls: Vec::new(),
                fields: HashMap::new(),
                focus: HashMap::new(),
                open_combo: None,
            },
        };
        form.title = title;
        for control in &controls {
            if control.kind == ControlKind::TextBox {
                form.fields
                    .entry(control.id.clone())
                    .or_insert_with(|| TextField::new(""))
                    .set(control.value.clone());
            }
        }
        form.controls = controls;
        match slot {
            Some(index) => self.forms.insert(index, form),
            None => self.forms.push(form),
        }
    }

    /// The facts a script asserts on: how many plugin files, each plugin's state and version by
    /// its name (spaces as `_`), the two menus' entries, the question showing, the forms, the
    /// last status line.
    fn record_facts(&self) {
        facts::record("plugins.count", self.host.plugins().len());
        for status in self.host.plugins() {
            let name = status.name().replace(' ', "_");
            facts::record(format!("plugins.{name}.state"), status.state.word());
            if let Some(info) = &status.info {
                facts::record(format!("plugins.{name}.version"), &info.version);
                facts::record(format!("plugins.{name}.author"), &info.author);
            }
            if let PluginState::Unloaded(reason) | PluginState::NotLoaded(reason) = &status.state {
                facts::record(format!("plugins.{name}.reason"), reason);
            }
        }
        for (menu, key) in [
            (MapMenu::FlightData, "plugins.menu.flight-data"),
            (MapMenu::FlightPlanner, "plugins.menu.flight-planner"),
        ] {
            let texts: Vec<&str> = self
                .entries
                .iter()
                .filter(|entry| entry.menu == menu)
                .map(|entry| entry.text.as_str())
                .collect();
            facts::record(
                key,
                if texts.is_empty() {
                    "none".to_owned()
                } else {
                    texts.join(",")
                },
            );
        }
        facts::record(
            "plugins.question",
            self.questions.front().map_or_else(
                || "none".to_owned(),
                |q| format!("{}: {}", q.caption, q.text),
            ),
        );
        let forms: Vec<&str> = self.forms.iter().map(|form| form.title.as_str()).collect();
        facts::record(
            "plugins.forms",
            if forms.is_empty() {
                "none".to_owned()
            } else {
                forms.join(",")
            },
        );
        facts::record("plugins.status", self.status.as_deref().unwrap_or("none"));
    }
}

/// `Host.cs.<name>`: the fields the Scripts tab reads by the C#'s names, the quick view's whole
/// table, and `connected` and `firmware`.
#[must_use]
pub fn cs_value(state: Option<&VehicleState>, connected: bool, name: &str) -> Option<CsValue> {
    match name {
        "connected" => return Some(CsValue::Flag(connected)),
        // `cs.firmware`: the C#'s `Firmwares` name of the vehicle's family.
        "firmware" => {
            let family = state.and_then(|state| VehicleFamily::from_mav_type(state.vehicle_type));
            return Some(CsValue::Text(
                match family {
                    Some(VehicleFamily::Copter) => "ArduCopter2",
                    Some(VehicleFamily::Plane) => "ArduPlane",
                    Some(VehicleFamily::Rover) => "ArduRover",
                    None => "Other",
                }
                .to_owned(),
            ));
        }
        _ => {}
    }
    let state = state?;
    if let Some(value) = crate::scripts_tab::cs_field(state, name) {
        return Some(match value {
            mp_script::CsValue::Number(number) => CsValue::Number(number),
            mp_script::CsValue::Text(text) => CsValue::Text(text),
            mp_script::CsValue::Flag(flag) => CsValue::Flag(flag),
        });
    }
    crate::quick::value(name, state).map(CsValue::Number)
}

/// `WriteUserData`'s path under the user data directory: relative, and not climbing out of it.
fn user_path(root: &Path, path: &str) -> Result<PathBuf, String> {
    let relative = Path::new(path);
    if path.is_empty()
        || relative
            .components()
            .any(|part| !matches!(part, Component::Normal(_)))
    {
        return Err(format!("not a path under the user data directory: {path}"));
    }
    Ok(root.join(relative))
}

/// A file written under the user data directory, its folders made: the path written.
fn write_user_data(root: Option<PathBuf>, path: &str, data: &[u8]) -> Result<String, String> {
    let root = root.ok_or_else(|| "no user data directory".to_owned())?;
    let full = user_path(&root, path)?;
    if let Some(parent) = full.parent() {
        std::fs::create_dir_all(parent).map_err(|err| format!("{}: {err}", parent.display()))?;
    }
    std::fs::write(&full, data).map_err(|err| format!("{}: {err}", full.display()))?;
    Ok(full.display().to_string())
}

/// A fence item as the world carries it: degrees, not degrees times 1e7.
fn fence_point(item: &mp_vehicle::FenceItem) -> FencePoint {
    FencePoint {
        command: item.command,
        param1: item.param1,
        lat: f64::from(item.x) / 1e7,
        lng: f64::from(item.y) / 1e7,
    }
}

impl MissionPlanner {
    /// Once a frame: the plugins' snapshot refreshed, what they did and asked served, a menu
    /// click on the planning map passed on, and the facts recorded.
    pub(crate) fn plugins_tick(
        &mut self,
        view: &TelemetryView,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        if !self.plugins.any() {
            if facts::enabled() {
                self.plugins.record_facts();
            }
            return;
        }
        let state = view.state.clone();
        let connected = view.connected && view.state.is_some();
        let fence: Vec<FencePoint> = self
            .telemetry
            .fence_points()
            .iter()
            .map(fence_point)
            .collect();
        let view_area = self
            .map
            .borrow()
            .view_corners()
            .map(|(top_left, bottom_right)| Area {
                top: top_left.latitude(),
                bottom: bottom_right.latitude(),
                left: top_left.longitude(),
                right: bottom_right.longitude(),
            });
        let parameters = Arc::clone(&view.parameters);
        self.plugins.host.update_snapshot(move |snapshot| {
            snapshot.cs = Arc::new(move |name| cs_value(state.as_deref(), connected, name));
            snapshot.params = parameters;
            snapshot.fence = Arc::from(fence);
            snapshot.fp_view_area = view_area;
        });
        for request in self.plugins.host.drain() {
            self.serve(request.plugin, request.body, view);
        }
        // A planning map entry chosen: `Host.FPMenuMapPosition` with it.
        if let Some((plugin, id, at)) = self.plan_menus.plugin_click.take() {
            self.plugins
                .host
                .menu_click(plugin, id, at.latitude(), at.longitude());
        }
        self.plan_menus.plugin_entries = self.plugins.planner_entries();
        // The first question takes the focus when it shows.
        if self
            .plugins
            .questions
            .front()
            .is_some_and(|q| q.value.is_some())
            && !self.plugins.focused
        {
            self.plugins.focus.focus(window, cx);
            self.plugins.focused = true;
        }
        for form in &mut self.plugins.forms {
            for control in &form.controls {
                if control.kind == ControlKind::TextBox && !form.focus.contains_key(&control.id) {
                    form.focus.insert(control.id.clone(), cx.focus_handle());
                }
            }
        }
        if facts::enabled() {
            self.plugins.record_facts();
        }
    }

    /// One thing a plugin did or asked, served through the application's own paths.
    fn serve(&mut self, plugin: usize, body: RequestBody, view: &TelemetryView) {
        let name = self.plugins.name(plugin);
        match body {
            // "Plugin Init {name} {version} by {author}". `// C#: Plugin/PluginLoader.cs:177`
            RequestBody::Started(info) => {
                self.file_status = Some(format!(
                    "Plugin Init {} {} by {}",
                    info.name, info.version, info.author
                ));
            }
            RequestBody::NotLoaded(reason) => {
                // `Init` false is a plugin choosing not to run, which the C# does not log.
                if reason != "Init returned false" {
                    self.file_status = Some(format!("Plugin {name}: {reason}"));
                }
                self.plugins.forget(plugin);
            }
            RequestBody::Unloaded(reason) => {
                self.file_status = Some(format!("Plugin {name} unloaded: {reason}"));
                self.plugins.forget(plugin);
            }
            RequestBody::Idle | RequestBody::Exited => {}
            RequestBody::Status(text) => {
                self.plugins.status = Some(text.clone());
                self.file_status = Some(text);
            }
            // `log.Info`: the application's log is its standard error.
            RequestBody::Log(text) => eprintln!("plugin {name}: {text}"),
            RequestBody::MenuAdd {
                id,
                menu,
                parent,
                text,
            } => self.plugins.entries.push(Entry {
                plugin,
                id,
                menu,
                parent,
                text,
            }),
            // `setParam`: the link's parameter write, as the parameter pages send it.
            RequestBody::SetParam { name, value, reply } => {
                reply.send(
                    self.telemetry
                        .write_parameter(&name, value, false)
                        .is_some(),
                );
            }
            // `setMode(name)`: the number of the vehicle's mode of that name, sent as the fly
            // screen's mode buttons send it.
            RequestBody::SetMode { mode, reply } => {
                let number = crate::fly::family(view).and_then(|family| family.mode_number(&mode));
                if let Some(number) = number {
                    self.telemetry.set_mode(number);
                }
                reply.send(number.is_some() && self.telemetry.send_handle().is_some());
            }
            // `sendPacket`: the message decoded as the dialect has it, queued on the link.
            RequestBody::SendPacket {
                message_id,
                payload,
                reply,
            } => {
                let message = mp_mavlink_dialects::MavMessage::decode(message_id, &payload);
                let sent = match (message, self.telemetry.send_handle()) {
                    (Some(message), Some((sender, _))) => sender.send(&message),
                    _ => false,
                };
                reply.send(sent);
            }
            RequestBody::MessageBox {
                text,
                caption,
                buttons,
                reply,
            } => self.plugins.ask(Question {
                plugin,
                caption,
                text,
                buttons,
                value: None,
                answer: Answer::Message(reply),
            }),
            RequestBody::InputBox {
                title,
                prompt,
                value,
                reply,
            } => self.plugins.ask(Question {
                plugin,
                caption: title,
                text: prompt,
                buttons: MessageButtons::Ok,
                value: Some(value),
                answer: Answer::Input(reply),
            }),
            RequestBody::OpenFile {
                title,
                filter,
                reply,
            } => self.plugins.ask(Question {
                plugin,
                caption: title,
                text: format!("File to open ({filter})"),
                buttons: MessageButtons::Ok,
                value: Some(String::new()),
                answer: Answer::Open(reply),
            }),
            RequestBody::SaveFile {
                title,
                filter,
                name,
                data,
                reply,
            } => self.plugins.ask(Question {
                plugin,
                caption: title,
                text: format!("File to save ({filter})"),
                buttons: MessageButtons::Ok,
                value: Some(name),
                answer: Answer::Save { data, reply },
            }),
            RequestBody::AddWp { wp, reply } => reply.send(self.plugin_add_wp(&wp)),
            RequestBody::InsertWp { index, wp } => self.plugin_insert_wp(index, &wp),
            // `GetWPs`: `BUT_read_Click`. `// C#: Plugin/Plugin.cs:241-244`
            RequestBody::GetWps => crate::plan::read_from_vehicle(self),
            RequestBody::ConfigGet { key, reply } => {
                reply.send(self.persisted.get(&key).map(str::to_owned));
            }
            RequestBody::ConfigSet { key, value } => self.persisted.set(&key, value),
            RequestBody::WriteUserData { path, data, reply } => {
                reply.send(write_user_data(
                    mp_settings::user_data_directory(),
                    &path,
                    &data,
                ));
            }
            RequestBody::FormShow { title, controls } => {
                self.plugins.show_form(plugin, title, controls);
            }
            RequestBody::FormClose => self.plugins.forms.retain(|form| form.plugin != plugin),
        }
    }

    /// `AddWPtoList`: `AddCommand` on the planning screen, the new row's index back, -1 when
    /// the plan refused it (its reason on the status line).
    /// `// C#: Plugin/Plugin.cs:213-217; GCSViews/FlightPlanner.cs:540-550`
    fn plugin_add_wp(&mut self, wp: &Waypoint) -> i32 {
        let context = crate::plan::menu_context(self);
        match self.plan.add_command(
            wp.command,
            [wp.p1, wp.p2, wp.p3, wp.p4],
            wp.x,
            wp.y,
            wp.z,
            &context,
        ) {
            Ok(()) => i32::try_from(self.plan.items().len()).map_or(i32::MAX, |rows| rows - 1),
            Err(why) => {
                self.file_status = Some(why.to_owned());
                -1
            }
        }
    }

    /// `InsertWP`: `InsertCommand`, `AddCommand` past the last row, else the row made as
    /// `AddCommand` makes it and put at `index`.
    /// `// C#: Plugin/Plugin.cs:219-224; GCSViews/FlightPlanner.cs:971-988`
    fn plugin_insert_wp(&mut self, index: i32, wp: &Waypoint) {
        let rows = self.plan.items().len();
        let at = usize::try_from(index).unwrap_or(0);
        if self.plugin_add_wp(wp) < 0 || at >= rows {
            return;
        }
        let Some(row) = self.plan.items().last().cloned() else {
            return;
        };
        self.plan.remove(row.seq);
        if self.plan.insert_after(&at.to_string(), row).is_err() {
            self.plan.append(row);
        }
    }

    /// The showing question answered: `button` the one pressed, the box's text read for the
    /// boxes that have one.
    fn plugin_answer(&mut self, button: DialogResult) {
        let Some(question) = self.plugins.questions.pop_front() else {
            return;
        };
        self.plugins.focused = false;
        let text = self.plugins.field.value().to_owned();
        let accepted = matches!(button, DialogResult::Ok | DialogResult::Yes);
        match question.answer {
            Answer::Message(reply) => reply.send(button),
            Answer::Input(reply) => reply.send(accepted.then_some(text)),
            Answer::Open(reply) => {
                let opened = if accepted {
                    match std::fs::read(&text) {
                        Ok(data) => Some(OpenedFile {
                            name: Path::new(&text)
                                .file_name()
                                .map_or_else(String::new, |name| {
                                    name.to_string_lossy().into_owned()
                                }),
                            data,
                        }),
                        Err(err) => {
                            self.file_status = Some(format!("{text}: {err}"));
                            None
                        }
                    }
                } else {
                    None
                };
                reply.send(opened);
            }
            Answer::Save { data, reply } => {
                let saved = if accepted {
                    match std::fs::write(&text, &data) {
                        Ok(()) => Some(text),
                        Err(err) => {
                            self.file_status = Some(format!("{text}: {err}"));
                            None
                        }
                    }
                } else {
                    None
                };
                reply.send(saved);
            }
        }
        if let Some(next) = self.plugins.questions.front() {
            self.plugins
                .field
                .set(next.value.clone().unwrap_or_default());
        }
    }

    /// A key in the showing question's box: Enter is OK, Escape Cancel.
    fn plugin_question_key(&mut self, event: &KeyDownEvent) -> bool {
        match self.plugins.field.key(event) {
            KeyOutcome::Submitted => self.plugin_answer(DialogResult::Ok),
            KeyOutcome::Cancelled => self.plugin_answer(DialogResult::Cancel),
            KeyOutcome::Changed => {}
            KeyOutcome::Ignored => return false,
        }
        true
    }

    /// A control of a plugin's form changed or was pressed: sent to the plugin.
    fn plugin_form_event(&mut self, plugin: usize, id: &str, value: &str) {
        if let Some(form) = self
            .plugins
            .forms
            .iter_mut()
            .find(|form| form.plugin == plugin)
        {
            if let Some(control) = form.controls.iter_mut().find(|control| control.id == id)
                && control.kind != ControlKind::Button
            {
                control.value = value.to_owned();
            }
            form.open_combo = None;
        }
        self.plugins.host.form_event(plugin, id, value);
    }

    /// A key in a form's text box: each change sent to the plugin.
    fn plugin_form_key(&mut self, plugin: usize, id: &str, event: &KeyDownEvent) -> bool {
        let Some(form) = self
            .plugins
            .forms
            .iter_mut()
            .find(|form| form.plugin == plugin)
        else {
            return false;
        };
        let Some(field) = form.fields.get_mut(id) else {
            return false;
        };
        match field.key(event) {
            KeyOutcome::Changed | KeyOutcome::Submitted => {
                let value = field.value().to_owned();
                self.plugin_form_event(plugin, id, &value);
                true
            }
            KeyOutcome::Cancelled | KeyOutcome::Ignored => false,
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// The width of a plugin's form.
const FORM_WIDTH: f32 = 340.0;
/// The width of the flight map's entries' panel.
const MENU_WIDTH: f32 = 200.0;
/// One entry's height.
const MENU_ROW: f32 = 22.0;

/// The plugins' part of the window: the question showing, the forms, and on the flight screen the
/// flight map's entries.
pub fn overlay(
    this: &MissionPlanner,
    fly_screen: bool,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    let plugins = &this.plugins;
    let mut out = Vec::new();
    if fly_screen {
        out.extend(flight_menu(this, window, cx));
    }
    let size = window.viewport_size();
    let mut top = 72.0;
    for form in &plugins.forms {
        out.push(form_panel(
            form,
            (f32::from(size.width) - FORM_WIDTH - 16.0, top),
            window,
            cx,
        ));
        top += 40.0;
    }
    if let Some(question) = plugins.questions.front() {
        out.push(question_box(
            question,
            &plugins.field,
            &plugins.focus,
            window,
            cx,
        ));
    }
    out
}

/// A clickable row.
fn row(
    id: &'static str,
    text: String,
    cx: &mut Context<MissionPlanner>,
) -> gpui::Stateful<gpui::Div> {
    let _ = cx;
    crate::probe::measured(id, div())
        .id(id)
        .h(px(MENU_ROW))
        .px_3()
        .flex()
        .items_center()
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .child(text)
}

/// The flight map's entries: `FDMenuMap`'s items a plugin added, clicked with the flight map's
/// last press as `Host.FDMenuMapPosition`.
fn flight_menu(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let entries: Vec<&Entry> = this
        .plugins
        .entries
        .iter()
        .filter(|entry| entry.menu == MapMenu::FlightData)
        .collect();
    if entries.is_empty() {
        return None;
    }
    #[allow(clippy::cast_precision_loss)]
    let height = MENU_ROW * entries.len() as f32 + 28.0;
    let size = window.viewport_size();
    let rows: Vec<AnyElement> = entries
        .into_iter()
        .map(|entry| {
            let (plugin, id) = (entry.plugin, entry.id);
            row(entry.probe_id(), entry.label(), cx)
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    let at = this
                        .fly_data
                        .mouse_down_start
                        .map_or((0.0, 0.0), |(at, _)| (at.latitude(), at.longitude()));
                    this.plugins.host.menu_click(plugin, id, at.0, at.1);
                    cx.notify();
                }))
                .into_any_element()
        })
        .collect();
    let panel = crate::probe::measured("plugin-flight-menu", div())
        .flex()
        .flex_col()
        .w(px(MENU_WIDTH))
        .py_1()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_sm()
        .child(
            div()
                .px_3()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child("Map menu - plugins"),
        )
        .children(rows);
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(
                    px(f32::from(size.width) - MENU_WIDTH - 16.0),
                    px(f32::from(size.height) - height - 40.0),
                ))
                .child(panel),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

/// One control of a form.
fn control_row(
    form: &Form,
    control: &Control,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let plugin = form.plugin;
    let id = control.id.clone();
    let probe = static_id(format!("plugin-form-{plugin}-{}", control.id));
    let label = div()
        .w(px(110.0))
        .text_xs()
        .text_color(rgb(theme::DIM))
        .child(control.label.clone());
    let widget: AnyElement = match control.kind {
        ControlKind::Label => div()
            .text_sm()
            .text_color(rgb(theme::TEXT))
            .child(control.value.clone())
            .into_any_element(),
        ControlKind::Button => action(
            probe,
            control.value.clone(),
            theme::ACCENT,
            true,
            cx.listener(move |this, _event: &(), _window, cx| {
                this.plugin_form_event(plugin, &id, "");
                cx.notify();
            }),
        ),
        ControlKind::CheckBox => {
            let checked = control.value == "true";
            crate::probe::measured(probe, div())
                .id(probe)
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .child(if checked { "[x]" } else { "[ ]" })
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.plugin_form_event(plugin, &id, if checked { "false" } else { "true" });
                    cx.notify();
                }))
                .into_any_element()
        }
        ControlKind::TextBox => match (form.fields.get(&control.id), form.focus.get(&control.id)) {
            (Some(field), Some(handle)) => crate::textfield::text_field(
                probe,
                field,
                handle,
                handle.is_focused(window),
                px(190.0),
                cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                    if this.plugin_form_key(plugin, &id, event) {
                        cx.notify();
                    }
                }),
            )
            .into_any_element(),
            _ => div().child(control.value.clone()).into_any_element(),
        },
        ControlKind::ComboBox => combo(form, control, probe, cx),
    };
    div()
        .flex()
        .items_center()
        .gap_2()
        .child(label)
        .child(widget)
        .into_any_element()
}

/// A combo box: its value, and while open its options below it, each a row to choose.
fn combo(
    form: &Form,
    control: &Control,
    probe: &'static str,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let plugin = form.plugin;
    let id = control.id.clone();
    let open = form.open_combo.as_deref() == Some(control.id.as_str());
    let head = crate::probe::measured(probe, div())
        .id(probe)
        .w(px(190.0))
        .px_2()
        .py_1()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_sm()
        .text_sm()
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .child(format!("{} \u{25be}", control.value))
        .on_click(cx.listener(move |this, _event, _window, cx| {
            if let Some(form) = this
                .plugins
                .forms
                .iter_mut()
                .find(|form| form.plugin == plugin)
            {
                form.open_combo = if form.open_combo.as_deref() == Some(id.as_str()) {
                    None
                } else {
                    Some(id.clone())
                };
            }
            cx.notify();
        }));
    let mut column = div().flex().flex_col().child(head);
    if open {
        // Thirty rows at most, as every drop-down here shows.
        let options: Vec<AnyElement> = control
            .options
            .iter()
            .take(30)
            .enumerate()
            .map(|(index, option)| {
                let id = control.id.clone();
                let value = option.clone();
                row(static_id(format!("{probe}-{index}")), option.clone(), cx)
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        this.plugin_form_event(plugin, &id, &value);
                        cx.notify();
                    }))
                    .into_any_element()
            })
            .collect();
        column = column.child(
            div()
                .flex()
                .flex_col()
                .bg(rgb(theme::PANEL))
                .border_1()
                .border_color(rgb(theme::BORDER))
                .children(options),
        );
    }
    column.into_any_element()
}

/// A plugin's form, as a panel at `at`, with a close box that hides it.
fn form_panel(
    form: &Form,
    at: (f32, f32),
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let plugin = form.plugin;
    let close = static_id(format!("plugin-form-{plugin}-close"));
    let title = div()
        .flex()
        .justify_between()
        .items_center()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(form.title.to_uppercase()),
        )
        .child(
            crate::probe::measured(close, div())
                .id(close)
                .px_1()
                .text_sm()
                .text_color(rgb(theme::DIM))
                .cursor_pointer()
                .child("\u{d7}")
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.plugins.forms.retain(|form| form.plugin != plugin);
                    cx.notify();
                })),
        );
    let rows: Vec<AnyElement> = form
        .controls
        .iter()
        .map(|control| control_row(form, control, window, cx))
        .collect();
    let panel = crate::probe::measured(static_id(format!("plugin-form-{plugin}")), div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(FORM_WIDTH))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_md()
        .occlude()
        .child(title)
        .children(rows);
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(at.0), px(at.1)))
            .snap_to_window()
            .child(panel),
    )
    .with_priority(1)
    .into_any_element()
}

/// The question showing, over the whole window as `ShowDialog` holds it: its caption, its text,
/// its box for the ones that have one, and its buttons.
fn question_box(
    question: &Question,
    field: &TextField,
    focus: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let button = |id: &'static str,
                  text: &'static str,
                  answer: DialogResult,
                  cx: &mut Context<MissionPlanner>| {
        action(
            id,
            text,
            theme::ACCENT,
            true,
            cx.listener(move |this, _event: &(), _window, cx| {
                this.plugin_answer(answer);
                cx.notify();
            }),
        )
    };
    let buttons = match (&question.answer, question.buttons) {
        (Answer::Message(_), MessageButtons::YesNo) => vec![
            button("plugin-question-yes", "Yes", DialogResult::Yes, cx),
            button("plugin-question-no", "No", DialogResult::No, cx),
        ],
        (Answer::Message(_), MessageButtons::Ok) => {
            vec![button("plugin-question-ok", "OK", DialogResult::Ok, cx)]
        }
        _ => vec![
            button("plugin-question-ok", "OK", DialogResult::Ok, cx),
            button("plugin-question-cancel", "Cancel", DialogResult::Cancel, cx),
        ],
    };
    let mut dialog = crate::probe::measured("plugin-question", div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(380.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(question.caption.clone()),
        )
        .children(question.text.lines().map(|line| {
            div()
                .w_full()
                .min_w_0()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(line.to_owned())
        }));
    if question.value.is_some() {
        dialog = dialog.child(crate::textfield::text_field(
            "plugin-question-value",
            field,
            focus,
            focus.is_focused(window),
            px(350.0),
            cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                if this.plugin_question_key(event) {
                    cx.notify();
                }
            }),
        ));
    }
    let dialog = dialog.child(
        div()
            .w_full()
            .flex()
            .gap_2()
            .justify_end()
            .children(buttons),
    );
    let size = window.viewport_size();
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .id(SharedString::from("plugin-question-backdrop"))
                    .w(size.width)
                    .h(size.height)
                    .flex()
                    .items_center()
                    .justify_center()
                    .occlude()
                    .child(dialog),
            ),
    )
    .with_priority(2)
    .into_any_element()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The planner carries the plugins Mission Planner ships, so a plain start loads them with
    /// nothing beside the executable: the owner's bug of 2026-10-03 was a start that loaded none,
    /// because the plugins were only ever built into a test's folder.
    #[test]
    fn the_planner_carries_the_plugins_mission_planner_ships() {
        let names: Vec<&str> = super::builtin::BUILTIN
            .iter()
            .map(|(name, _)| *name)
            .collect();
        assert_eq!(
            names,
            [
                "anonymizebinlog.wasm",
                "dowding.wasm",
                "example.wasm",
                "fencedist.wasm",
                "mapicondesc.wasm",
                "menu.wasm",
                "modechange.wasm",
                "opendroneid.wasm",
                "persistentsimple.wasm",
                "terrainmaker.wasm",
            ]
        );
        for (name, bytes) in super::builtin::BUILTIN {
            assert!(
                bytes.starts_with(b"\0asm"),
                "{name} is not WebAssembly ({} bytes)",
                bytes.len()
            );
        }
    }

    /// A plugin writes under the user data directory only: a relative path, folders made.
    #[test]
    fn user_data_stays_under_its_directory() {
        let root = std::env::temp_dir().join(format!("plugins-ui-{}", std::process::id()));
        let written = write_user_data(Some(root.clone()), "TerrainData/S36E149.DAT", b"dat")
            .expect("written");
        assert_eq!(
            std::fs::read(&written).expect("read back"),
            b"dat",
            "{written}"
        );
        for bad in ["", "/etc/passwd", "../outside", "TerrainData/../../x"] {
            assert!(
                write_user_data(Some(root.clone()), bad, b"x").is_err(),
                "{bad}"
            );
        }
        assert!(write_user_data(None, "a", b"x").is_err());
        let _ = std::fs::remove_dir_all(root);
    }

    /// `Host.cs`: `connected` from the link, `firmware` by the C#'s names, the Scripts tab's
    /// fields, the quick view's table, nothing for a name neither has.
    #[test]
    fn cs_reads_the_csharps_names() {
        assert_eq!(
            cs_value(None, false, "connected"),
            Some(CsValue::Flag(false))
        );
        assert_eq!(
            cs_value(None, false, "firmware"),
            Some(CsValue::Text("Other".to_owned()))
        );
        assert_eq!(cs_value(None, true, "lat"), None);
        let mut state = VehicleState::default();
        state.vehicle_type = 2;
        state.armed = true;
        assert_eq!(
            cs_value(Some(&state), true, "firmware"),
            Some(CsValue::Text("ArduCopter2".to_owned()))
        );
        assert_eq!(
            cs_value(Some(&state), true, "armed"),
            Some(CsValue::Flag(true))
        );
        assert!(matches!(
            cs_value(Some(&state), true, "groundspeed"),
            Some(CsValue::Number(_))
        ));
        assert_eq!(cs_value(Some(&state), true, "no_such_field"), None);
    }

    /// The folder: `MP_PLUGINS` when set, else `plugins` beside the executable.
    #[test]
    fn the_folder_is_beside_the_executable() {
        if std::env::var_os(ENV).is_none() {
            let exe = std::env::current_exe().expect("exe");
            assert_eq!(folder(), exe.parent().map(|dir| dir.join("plugins")));
        }
        assert!(crate::USAGE.contains(ENV), "usage does not mention {ENV}");
    }

    /// A menu entry's id and label: an entry under a parent is indented.
    #[test]
    fn entries_name_themselves() {
        let entry = Entry {
            plugin: 2,
            id: 5,
            menu: MapMenu::FlightData,
            parent: Some(1),
            text: "Arm".to_owned(),
        };
        assert_eq!(entry.probe_id(), "plugin-menu-2-5");
        assert_eq!(entry.label(), "    Arm");
        // The same id twice is the same leaked string.
        assert!(std::ptr::eq(entry.probe_id(), entry.probe_id()));
    }
}
