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

//! `PluginLoader.LoadAll` and `MainV2.PluginThread`: every `*.wasm` in the plugins folder
//! loaded, each on a thread of its own that runs its `Loop` at its `loopratehz` and serves its
//! menu clicks and form events.
//!
//! The C# runs every plugin's `Loop` on one shared thread and its menu clicks on the window's
//! thread (`// C#: MainV2.cs:2506-2574`); a plugin that blocks there blocks them all, or the
//! window. Here each plugin has its own thread, which also takes its clicks, so a plugin waits
//! only on itself; the window hears from it through [`Request`]s it drains once a frame.

use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, PoisonError, RwLock};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use wasmtime::Engine;

use crate::channel::{ChannelSurface, Request, RequestBody, Snapshot};
use crate::plugin::{Fault, Info, Limits, Plugin, engine};

/// `Settings.GetRunningDirectory() + "plugins"`: the `plugins` folder beside the executable.
/// `// C#: Plugin/PluginLoader.cs:205-206`
#[must_use]
pub fn plugins_dir() -> Option<PathBuf> {
    let exe = std::env::current_exe().ok()?;
    Some(exe.parent()?.join("plugins"))
}

/// The reason a plugin is [`PluginState::NotLoaded`] when its `Init` said no - as against a file
/// that could not be loaded at all, which is what the plugin manager's Show Errors lists.
pub const INIT_REFUSED: &str = "Init returned false";

/// Where a plugin is in its life.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PluginState {
    /// Its thread is compiling it or running `Init`.
    Loading,
    /// `Init` and `Loaded` said yes: its `Loop` runs at its rate.
    Running,
    /// `Loaded` said no: the C# does not add it to the running plugins, so its `Loop` never
    /// runs; what it added in `Init` and `Loaded` (a menu entry) still works.
    Idle,
    /// `Init` said no, or the file is not a plugin (the reason).
    NotLoaded(String),
    /// It faulted and was unloaded (the fault).
    Unloaded(String),
    /// `Exit` ran.
    Exited,
}

impl PluginState {
    /// One word for the state, as a fact carries it.
    #[must_use]
    pub const fn word(&self) -> &'static str {
        match self {
            Self::Loading => "loading",
            Self::Running => "running",
            Self::Idle => "idle",
            Self::NotLoaded(_) => "not-loaded",
            Self::Unloaded(_) => "unloaded",
            Self::Exited => "exited",
        }
    }
}

/// One plugin file and what became of it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PluginStatus {
    /// The file's name.
    pub file: String,
    /// Who the plugin says it is, once loaded.
    pub info: Option<Info>,
    /// Where it is in its life.
    pub state: PluginState,
}

impl PluginStatus {
    /// The plugin's `Name`, or its file's name before it has said.
    #[must_use]
    pub fn name(&self) -> &str {
        self.info
            .as_ref()
            .map_or(self.file.as_str(), |info| info.name.as_str())
    }
}

/// What the window asks of a plugin's thread.
#[derive(Debug)]
enum Command {
    MenuClick { id: u32, lat: f64, lng: f64 },
    FormEvent { id: String, value: String },
    Exit,
}

/// A plugin's thread, as the host holds it.
#[derive(Debug)]
struct Handle {
    commands: Sender<Command>,
    thread: Option<JoinHandle<()>>,
}

/// How long closing waits for the plugins' `Exit`s: the C# waits for its plugin thread's
/// (`// C#: MainV2.cs:2084`); a plugin blocked on a question nobody will answer is not waited
/// for past this.
const EXIT_WAIT: Duration = Duration::from_secs(2);

/// The plugins, loaded and running.
#[derive(Debug)]
pub struct PluginHost {
    handles: Vec<Handle>,
    status: Vec<PluginStatus>,
    requests: Receiver<Request>,
    snapshot: Arc<RwLock<Snapshot>>,
}

impl Default for PluginHost {
    fn default() -> Self {
        let (_, requests) = mpsc::channel();
        Self {
            handles: Vec::new(),
            status: Vec::new(),
            requests,
            snapshot: Arc::default(),
        }
    }
}

/// Where a plugin's WebAssembly comes from.
#[derive(Debug, Clone)]
pub enum PluginSource {
    /// A `*.wasm` file, read on the plugin's thread.
    File(PathBuf),
    /// A plugin the application carries in itself, under the file name it would have in
    /// `plugins/`.
    Builtin {
        /// The file name it is known by: the disable list and the window name it by this.
        file: String,
        /// The component's bytes.
        bytes: &'static [u8],
    },
}

impl PluginSource {
    /// The file name the plugin is known by.
    #[must_use]
    pub fn file(&self) -> String {
        match self {
            Self::File(path) => file_name(path),
            Self::Builtin { file, .. } => file.clone(),
        }
    }
}

/// Every `*.wasm` file in `dir`, in name order; none when there is no such folder.
fn wasm_files(dir: &Path) -> Vec<PathBuf> {
    let mut files: Vec<PathBuf> = std::fs::read_dir(dir)
        .map(|entries| {
            entries
                .filter_map(|entry| entry.ok().map(|entry| entry.path()))
                .filter(|path| {
                    path.is_file()
                        && path
                            .extension()
                            .is_some_and(|ext| ext.eq_ignore_ascii_case("wasm"))
                })
                .collect()
        })
        .unwrap_or_default();
    files.sort();
    files
}

/// Whether `file` is on the disable list (`DisabledPluginNames`, compared without case).
fn is_disabled(file: &str, disabled: &[String]) -> bool {
    let file = file.to_lowercase();
    disabled.iter().any(|off| off.to_lowercase() == file)
}

impl PluginHost {
    /// `PluginLoader.LoadAll`: every `*.wasm` in `dir`, in name order, less the ones `disabled`
    /// names (`DisabledPluginNames`, lower-case file names), each loaded on its own thread. No
    /// folder, no plugins. `// C#: Plugin/PluginLoader.cs:99-120, 203-311`
    #[must_use]
    pub fn load_all(dir: &Path, disabled: &[String], limits: Limits) -> Self {
        Self::load_with_builtins(&[], Some(dir), disabled, limits)
    }

    /// `PluginLoader.LoadAll` over the plugins the application ships and the plugins folder.
    ///
    /// Mission Planner's build puts its plugins in the `plugins` folder beside the executable -
    /// every `plugins\*.cs` copied there (`MissionPlanner.csproj`, `CopyToOutputDirectory`), and
    /// the Dowding, OpenDroneID and TerrainMaker projects built into it - so every install starts
    /// with them loaded unless `DisabledPluginNames` names one. Here the shipped plugins are
    /// `builtins`, carried in the application, so that no install can be without them; a file in
    /// `dir` with a built-in's name is loaded in its place, as replacing a file in the C#'s folder
    /// does, and any other file in `dir` is loaded as well. The disable list applies to both.
    /// Name order over the whole set. `// C#: Plugin/PluginLoader.cs:99-120, 203-311`
    #[must_use]
    pub fn load_with_builtins(
        builtins: &[(&str, &'static [u8])],
        dir: Option<&Path>,
        disabled: &[String],
        limits: Limits,
    ) -> Self {
        let files = dir.map(wasm_files).unwrap_or_default();
        let replaced = |name: &str| {
            files
                .iter()
                .any(|path| file_name(path).eq_ignore_ascii_case(name))
        };
        let mut sources: Vec<PluginSource> = builtins
            .iter()
            .filter(|(name, _)| !replaced(name))
            .map(|(name, bytes)| PluginSource::Builtin {
                file: (*name).to_owned(),
                bytes,
            })
            .chain(files.iter().cloned().map(PluginSource::File))
            .filter(|source| !is_disabled(&source.file(), disabled))
            .collect();
        sources.sort_by_key(|source| source.file().to_lowercase());
        Self::load_sources(sources, limits)
    }

    /// The plugins in `files`, each loaded on its own thread.
    #[must_use]
    pub fn load_files(files: &[PathBuf], limits: Limits) -> Self {
        Self::load_sources(
            files.iter().cloned().map(PluginSource::File).collect(),
            limits,
        )
    }

    /// The plugins in `sources`, each loaded on its own thread.
    #[must_use]
    pub fn load_sources(sources: Vec<PluginSource>, limits: Limits) -> Self {
        let (tx, requests) = mpsc::channel();
        let snapshot = Arc::new(RwLock::new(Snapshot::default()));
        let engine = engine();
        let mut host = Self {
            handles: Vec::new(),
            status: Vec::new(),
            requests,
            snapshot: Arc::clone(&snapshot),
        };
        for (index, source) in sources.into_iter().enumerate() {
            let (commands, inbox) = mpsc::channel();
            let file = source.file();
            host.status.push(PluginStatus {
                file: file.clone(),
                info: None,
                state: PluginState::Loading,
            });
            let surface = ChannelSurface::new(index, tx.clone(), Arc::clone(&snapshot));
            let job = Job {
                index,
                source,
                file,
                engine: engine.clone(),
                limits,
                requests: tx.clone(),
            };
            let thread = std::thread::Builder::new()
                .name(format!("plugin {index}"))
                .spawn(move || job.run(surface, &inbox))
                .ok();
            if thread.is_none()
                && let Some(status) = host.status.last_mut()
            {
                status.state = PluginState::NotLoaded("no thread for it".to_owned());
            }
            host.handles.push(Handle { commands, thread });
        }
        host
    }

    /// Every plugin file and what became of it, in load order.
    #[must_use]
    pub fn plugins(&self) -> &[PluginStatus] {
        &self.status
    }

    /// Replaces what the plugins read: called by the window once a frame.
    pub fn set_snapshot(&self, snapshot: Snapshot) {
        *self
            .snapshot
            .write()
            .unwrap_or_else(PoisonError::into_inner) = snapshot;
    }

    /// Changes part of what the plugins read.
    pub fn update_snapshot(&self, change: impl FnOnce(&mut Snapshot)) {
        change(
            &mut self
                .snapshot
                .write()
                .unwrap_or_else(PoisonError::into_inner),
        );
    }

    /// Everything the plugins did or asked since the last call, in order; the plugins' states
    /// follow what it says.
    pub fn drain(&mut self) -> Vec<Request> {
        let mut out = Vec::new();
        while let Ok(request) = self.requests.try_recv() {
            if let Some(status) = self.status.get_mut(request.plugin) {
                match &request.body {
                    RequestBody::Started(info) => {
                        status.info = Some(info.clone());
                        status.state = PluginState::Running;
                    }
                    RequestBody::NotLoaded(reason) => {
                        status.state = PluginState::NotLoaded(reason.clone());
                    }
                    RequestBody::Unloaded(reason) => {
                        status.state = PluginState::Unloaded(reason.clone());
                    }
                    RequestBody::Idle => status.state = PluginState::Idle,
                    RequestBody::Exited => status.state = PluginState::Exited,
                    _ => {}
                }
            }
            out.push(request);
        }
        out
    }

    /// Whether any plugin is still loading.
    #[must_use]
    pub fn loading(&self) -> bool {
        self.status
            .iter()
            .any(|status| status.state == PluginState::Loading)
    }

    /// A menu entry plugin `plugin` added was chosen, the menu opened at `lat`, `lng`.
    pub fn menu_click(&self, plugin: usize, id: u32, lat: f64, lng: f64) {
        self.command(plugin, Command::MenuClick { id, lat, lng });
    }

    /// A control of plugin `plugin`'s form changed or was clicked.
    pub fn form_event(&self, plugin: usize, id: &str, value: &str) {
        self.command(
            plugin,
            Command::FormEvent {
                id: id.to_owned(),
                value: value.to_owned(),
            },
        );
    }

    fn command(&self, plugin: usize, command: Command) {
        if let Some(handle) = self.handles.get(plugin) {
            let _ = handle.commands.send(command);
        }
    }

    /// `Exit` for every running plugin, and their threads waited for up to [`EXIT_WAIT`].
    /// `// C#: MainV2.cs:2556-2574`
    pub fn shutdown(&mut self) {
        for handle in &self.handles {
            let _ = handle.commands.send(Command::Exit);
        }
        let deadline = Instant::now() + EXIT_WAIT;
        for handle in &mut self.handles {
            if let Some(thread) = handle.thread.take() {
                while !thread.is_finished() && Instant::now() < deadline {
                    std::thread::sleep(Duration::from_millis(5));
                }
                if thread.is_finished() {
                    let _ = thread.join();
                }
            }
        }
    }
}

impl Drop for PluginHost {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// The file's name, without its folder.
fn file_name(path: &Path) -> String {
    path.file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default()
}

/// One plugin's thread.
struct Job {
    index: usize,
    source: PluginSource,
    file: String,
    engine: Result<Engine, Fault>,
    limits: Limits,
    requests: Sender<Request>,
}

impl Job {
    fn tell(&self, body: RequestBody) {
        let _ = self.requests.send(Request {
            plugin: self.index,
            body,
        });
    }

    /// Load, `Init`, `Loaded`, then `Loop` at the rate and the window's commands until `Exit`;
    /// a fault anywhere unloads the plugin.
    fn run(self, surface: ChannelSurface, inbox: &Receiver<Command>) {
        let loaded = self.engine.clone().and_then(|engine| {
            let read;
            let bytes: &[u8] = match &self.source {
                PluginSource::File(path) => {
                    read = std::fs::read(path)
                        .map_err(|err| Fault::Load(format!("{}: {err}", path.display())))?;
                    &read
                }
                PluginSource::Builtin { bytes, .. } => bytes,
            };
            Plugin::load(&engine, bytes, &self.file, Box::new(surface), self.limits)
        });
        let mut plugin = match loaded {
            Ok(plugin) => plugin,
            Err(fault) => return self.tell(RequestBody::NotLoaded(fault.to_string())),
        };
        // `if (plugin.Init())`: false and nothing of it is kept.
        // `// C#: Plugin/PluginLoader.cs:175-182`
        match plugin.init() {
            Ok(true) => self.tell(RequestBody::Started(plugin.info().clone())),
            Ok(false) => {
                return self.tell(RequestBody::NotLoaded(INIT_REFUSED.to_owned()));
            }
            Err(fault) => return self.tell(RequestBody::Unloaded(fault.to_string())),
        }
        // `if (p.Loaded()) Plugins.Add(p)`: only an added plugin loops and exits.
        // `// C#: Plugin/PluginLoader.cs:323-336`
        let running = match plugin.loaded() {
            Ok(running) => running,
            Err(fault) => return self.tell(RequestBody::Unloaded(fault.to_string())),
        };
        if !running {
            self.tell(RequestBody::Idle);
        }
        loop {
            let next = if running { plugin.next_run() } else { None };
            let command = match next {
                Some(next) => inbox.recv_timeout(next.saturating_duration_since(Instant::now())),
                None => inbox.recv().map_err(|_| RecvTimeoutError::Disconnected),
            };
            let result = match command {
                Ok(Command::MenuClick { id, lat, lng }) => plugin.menu_click(id, lat, lng),
                Ok(Command::FormEvent { id, value }) => plugin.form_event(&id, &value),
                Ok(Command::Exit) | Err(RecvTimeoutError::Disconnected) => {
                    if running {
                        match plugin.exit() {
                            Ok(_) => self.tell(RequestBody::Exited),
                            Err(fault) => self.tell(RequestBody::Unloaded(fault.to_string())),
                        }
                    }
                    return;
                }
                Err(RecvTimeoutError::Timeout) => plugin.tick(Instant::now()).map(|_| ()),
            };
            if let Err(fault) = result {
                return self.tell(RequestBody::Unloaded(fault.to_string()));
            }
        }
    }
}
