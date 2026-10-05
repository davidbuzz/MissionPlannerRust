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

//! What the plugin tests share: the plugins built for WebAssembly, and a scripted application
//! surface that records what a plugin does and answers its questions from a script.

#![allow(dead_code, unreachable_pub, clippy::unwrap_used, clippy::expect_used)]

use mp_os::Lock as _;
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::path::PathBuf;
use std::process::Command;
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};

use mp_plugin_host::{
    Area, Control, CsValue, DialogResult, FencePoint, Limits, MapMenu, MessageButtons, OpenedFile,
    Plugin, Surface, Waypoint, engine,
};

/// The folder the plugins are built into, once per test binary: `None`, and each test says SKIP
/// and passes, on a machine without the `wasm32-unknown-unknown` target.
///
/// Built into a target folder of their own under the tests' scratch folder, so the build does
/// not wait on the lock of the one running these tests.
pub fn plugins() -> Option<&'static PathBuf> {
    static BUILT: OnceLock<Option<PathBuf>> = OnceLock::new();
    BUILT
        .get_or_init(|| {
            let installed = Command::new("rustup")
                .args(["target", "list", "--installed"])
                .output()
                .ok()
                .is_some_and(|out| {
                    String::from_utf8_lossy(&out.stdout).contains("wasm32-unknown-unknown")
                });
            if !installed {
                eprintln!("SKIP: rustup target add wasm32-unknown-unknown");
                return None;
            }
            let target = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("plugins");
            let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
            let status = Command::new(env!("CARGO"))
                .args([
                    "build",
                    "--release",
                    "-p",
                    "mp-plugins",
                    "--target",
                    "wasm32-unknown-unknown",
                    "--examples",
                ])
                .env("CARGO_TARGET_DIR", &target)
                .current_dir(&root)
                .status()
                .expect("cargo runs");
            assert!(status.success(), "the plugins build");
            Some(target.join("wasm32-unknown-unknown/release/examples"))
        })
        .as_ref()
}

/// The built plugin `name`'s path.
pub fn path(name: &str) -> Option<PathBuf> {
    Some(plugins()?.join(format!("{name}.wasm")))
}

/// Everything the scripted application saw and will answer.
#[derive(Debug, Default)]
pub struct Record {
    // What the application holds.
    pub cs: BTreeMap<String, CsValue>,
    pub params: BTreeMap<String, f64>,
    pub options: BTreeMap<String, Vec<(i32, String)>>,
    pub fence: Vec<FencePoint>,
    pub config: BTreeMap<String, String>,
    pub view_area: Option<Area>,
    /// The controls on screen, by the planner's names, for the demo pointer; `map` on screen
    /// takes a click anywhere on the map.
    pub shown: BTreeSet<String>,
    /// A demo gesture under way.
    pub demo_busy: bool,
    /// The terrain's height everywhere, when set.
    pub terrain: Option<f64>,
    pub set_param_ok: bool,
    pub set_mode_ok: bool,
    // The script's answers, in order.
    pub message_answers: VecDeque<DialogResult>,
    pub input_answers: VecDeque<Option<String>>,
    pub open_answers: VecDeque<Option<OpenedFile>>,
    // What the plugin did.
    pub menu: Vec<(MapMenu, Option<u32>, String)>,
    pub messages: Vec<(String, String, MessageButtons)>,
    pub inputs: Vec<(String, String, String)>,
    pub status: Vec<String>,
    pub log: Vec<String>,
    pub param_writes: Vec<(String, f64)>,
    pub modes: Vec<String>,
    pub packets: Vec<(u32, Vec<u8>)>,
    pub added: Vec<Waypoint>,
    pub inserted: Vec<(i32, Waypoint)>,
    pub reads: usize,
    pub saved: Vec<(String, Vec<u8>)>,
    pub written: Vec<(String, Vec<u8>)>,
    pub form: Option<(String, Vec<Control>)>,
    pub forms_closed: usize,
    pub terrain_reads: usize,
    /// The demo pointer's clicks, in order: a control's name, or `map lat,lng`.
    pub demo_clicks: Vec<String>,
    pub demo_typed: Vec<String>,
}

/// A scripted application: a [`Surface`] over a shared [`Record`].
#[derive(Debug, Clone, Default)]
pub struct Scripted(pub Arc<Mutex<Record>>);

impl Scripted {
    pub fn new() -> Self {
        let scripted = Self::default();
        scripted.with(|r| {
            r.set_param_ok = true;
            r.set_mode_ok = true;
        });
        scripted
    }

    pub fn record(&self) -> MutexGuard<'_, Record> {
        self.0.os_lock().unwrap()
    }

    pub fn with<R>(&self, change: impl FnOnce(&mut Record) -> R) -> R {
        change(&mut self.record())
    }

    /// The form's control `id`'s value.
    pub fn control(&self, id: &str) -> Option<Control> {
        self.record()
            .form
            .as_ref()
            .and_then(|(_, controls)| controls.iter().find(|c| c.id == id).cloned())
    }
}

impl Surface for Scripted {
    fn cs(&mut self, name: &str) -> Option<CsValue> {
        self.record().cs.get(name).cloned()
    }
    fn get_param(&mut self, name: &str) -> Option<f64> {
        self.record().params.get(name).copied()
    }
    fn param_options(&mut self, name: &str) -> Vec<(i32, String)> {
        self.record().options.get(name).cloned().unwrap_or_default()
    }
    fn set_param(&mut self, name: &str, value: f64) -> bool {
        let mut r = self.record();
        r.param_writes.push((name.to_owned(), value));
        r.set_param_ok
    }
    fn set_mode(&mut self, mode: &str) -> bool {
        let mut r = self.record();
        r.modes.push(mode.to_owned());
        r.set_mode_ok
    }
    fn send_packet(&mut self, message_id: u32, payload: Vec<u8>) -> bool {
        self.record().packets.push((message_id, payload));
        true
    }
    fn menu_add(&mut self, menu: MapMenu, parent: Option<u32>, text: &str) -> u32 {
        let mut r = self.record();
        r.menu.push((menu, parent, text.to_owned()));
        u32::try_from(r.menu.len() - 1).unwrap()
    }
    fn message_box(&mut self, text: &str, caption: &str, buttons: MessageButtons) -> DialogResult {
        let mut r = self.record();
        r.messages
            .push((text.to_owned(), caption.to_owned(), buttons));
        r.message_answers.pop_front().unwrap_or(DialogResult::Ok)
    }
    fn input_box(&mut self, title: &str, prompt: &str, value: &str) -> Option<String> {
        let mut r = self.record();
        r.inputs
            .push((title.to_owned(), prompt.to_owned(), value.to_owned()));
        r.input_answers
            .pop_front()
            .unwrap_or_else(|| Some(value.to_owned()))
    }
    fn status(&mut self, text: &str) {
        self.record().status.push(text.to_owned());
    }
    fn log(&mut self, text: &str) {
        self.record().log.push(text.to_owned());
    }
    fn add_wp(&mut self, wp: Waypoint) -> i32 {
        let mut r = self.record();
        r.added.push(wp);
        i32::try_from(r.added.len()).unwrap()
    }
    fn insert_wp(&mut self, index: i32, wp: Waypoint) {
        self.record().inserted.push((index, wp));
    }
    fn get_wps(&mut self) {
        self.record().reads += 1;
    }
    fn fence_points(&mut self) -> Vec<FencePoint> {
        self.record().fence.clone()
    }
    fn config_get(&mut self, key: &str) -> Option<String> {
        self.record().config.get(key).cloned()
    }
    fn config_set(&mut self, key: &str, value: &str) {
        self.record()
            .config
            .insert(key.to_owned(), value.to_owned());
    }
    fn save_tab_control_actions(&mut self) {
        self.record()
            .config
            .insert("tabcontrolactions".to_owned(), "tabQuick;".to_owned());
    }
    // The demo pointer over the script's screen: a click lands on a control shown, and is
    // recorded.
    fn demo_click(&mut self, control: &str, _millis: u32) -> bool {
        let mut r = self.record();
        let shown = r.shown.contains(control);
        if shown {
            r.demo_clicks.push(control.to_owned());
        }
        shown
    }
    fn demo_click_map(&mut self, lat: f64, lng: f64, _millis: u32) -> bool {
        let mut r = self.record();
        let shown = r.shown.contains("map");
        if shown {
            r.demo_clicks.push(format!("map {lat:.6},{lng:.6}"));
        }
        shown
    }
    fn demo_type(&mut self, text: &str) {
        self.record().demo_typed.push(text.to_owned());
    }
    fn demo_busy(&mut self) -> bool {
        self.record().demo_busy
    }
    fn demo_visible(&mut self, control: &str) -> bool {
        self.record().shown.contains(control)
    }
    fn fp_selected_area(&mut self) -> Option<Area> {
        None
    }
    fn fp_view_area(&mut self) -> Option<Area> {
        self.record().view_area
    }
    fn terrain_altitude(&mut self, _lat: f64, _lng: f64) -> Option<f64> {
        let mut r = self.record();
        r.terrain_reads += 1;
        r.terrain
    }
    fn open_file(&mut self, _title: &str, _filter: &str) -> Option<OpenedFile> {
        self.record().open_answers.pop_front().flatten()
    }
    fn save_file(
        &mut self,
        _title: &str,
        _filter: &str,
        name: &str,
        data: Vec<u8>,
    ) -> Option<String> {
        let path = format!("/saved/{name}");
        self.record().saved.push((path.clone(), data));
        Some(path)
    }
    fn write_user_data(&mut self, path: &str, data: Vec<u8>) -> Result<String, String> {
        let full = format!("/user/{path}");
        self.record().written.push((full.clone(), data));
        Ok(full)
    }
    fn form_show(&mut self, title: &str, controls: Vec<Control>) {
        self.record().form = Some((title.to_owned(), controls));
    }
    fn form_close(&mut self) {
        let mut r = self.record();
        r.form = None;
        r.forms_closed += 1;
    }
}

/// Plugin `name` loaded against a fresh scripted surface: the plugin and the script.
pub fn load(name: &str) -> Option<(Plugin, Scripted)> {
    load_with(name, Scripted::new(), Limits::default())
}

/// Plugin `name` loaded against `scripted` with `limits`.
pub fn load_with(name: &str, scripted: Scripted, limits: Limits) -> Option<(Plugin, Scripted)> {
    let path = path(name)?;
    let bytes = std::fs::read(&path).expect("the plugin was built");
    let engine = engine().expect("an engine");
    let plugin = Plugin::load(
        &engine,
        &bytes,
        &format!("{name}.wasm"),
        Box::new(scripted.clone()),
        limits,
    )
    .expect("loads");
    Some((plugin, scripted))
}

/// A fence point.
pub fn fence(command: u16, param1: f32, lat: f64, lng: f64) -> FencePoint {
    FencePoint {
        command,
        param1,
        lat,
        lng,
    }
}
