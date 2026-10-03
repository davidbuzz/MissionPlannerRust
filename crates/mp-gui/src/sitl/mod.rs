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

//! The SITL screen - MainV2's SIMULATION - ported from `GCSViews/SITL.cs`, `SITL.Designer.cs` and
//! `SITL.resx` @ efb0801 (GPL-3.0-only), PLAN.md §13.6 row 78 under the owner's ruling D14.
//!
//! The page: a map with the home marker to drag (`groupBox1`); the heading and the version to
//! download (`groupBox3`); the speed-up, the model, an extra command line, Wipe and the swarm
//! buttons (`groupBox4`); and the four vehicle pictures (`groupBox2`), a click on which fetches
//! that vehicle's simulator, starts it with the command line the page's choices make, shows
//! FLIGHT DATA and connects to it on TCP 5760.
//!
//! - `model.rs`: the texts, lists and command lines.
//! - `launcher.rs`: the download and the processes, per desktop.
//! - `wasm.rs`: the probe for a WebAssembly SITL and the note for desktops without a simulator.
//! - `view.rs`: the drawing.
//!
//! This module holds the page object and its part in the application. Every failure the C#
//! boxes - a failed download, a failed start, a failed connection - goes on the status line
//! (the owner's ruling of 2026-09-25); "how many?" keeps its box.
//!
//! Not ported, each for its reason: the three Multilink swarm buttons and Ctrl+D
//! (`StartSwarmSeperate`, `SITL.cs:829-991`) connect one link per vehicle
//! (`MainV2.Comports.Add`), which the application has not got - they are drawn dimmed;
//! `SITLSEND`, the UDP port 5501 the C# opens for `rcinput`'s joystick overrides
//! (`SITL.cs:728, 741-760`), as the joystick's RC override path through it is not ported. The
//! vehicle pictures are the `.resx`'s bitmaps, `ImageNormal` and `ImageOver` under the pointer
//! (`crate::pictures`).

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

pub mod launcher;
pub mod model;
pub mod view;
pub mod wasm;

use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::Arc;
use std::sync::mpsc::{Receiver, TryRecvError, channel};

use gpui::{Context, FocusHandle, KeyDownEvent, Window};
use mp_units::LatLon;

use crate::MissionPlanner;
use crate::config::optional::InputBox;
use crate::config::servo_output::Combo;
use crate::mapview::MapViewport;
use crate::textfield::{KeyOutcome, TextField};
use launcher::{Launcher, Outcome};
use model::Vehicle;

/// A `NumericUpDown` of whole numbers: its value, its range, and the text being typed.
#[derive(Debug)]
pub struct Spin {
    /// `Value`.
    value: i64,
    /// `Minimum`.
    minimum: i64,
    /// `Maximum`.
    maximum: i64,
    /// The text while it is typed into.
    field: TextField,
    /// Whether it is being typed into.
    edited: bool,
}

impl Spin {
    /// A box as the Designer leaves it.
    #[must_use]
    pub const fn new(minimum: i64, maximum: i64, value: i64) -> Self {
        Self {
            value,
            minimum,
            maximum,
            field: TextField::new(""),
            edited: false,
        }
    }

    /// `Value`.
    #[must_use]
    pub const fn value(&self) -> i64 {
        self.value
    }

    /// What the box shows.
    #[must_use]
    pub fn shown(&self) -> String {
        if self.edited {
            self.field.value().to_owned()
        } else {
            self.value.to_string()
        }
    }

    /// Clicked into: its text, to type after.
    pub fn begin(&mut self) {
        if !self.edited {
            self.field.set(self.value.to_string());
            self.edited = true;
        }
    }

    /// The text read, as the box does when it loses the focus or takes Enter: a number is
    /// brought into the range; anything else puts the value back.
    pub fn commit(&mut self) {
        if self.edited {
            if let Ok(typed) = self.field.value().trim().parse::<i64>() {
                self.value = typed.clamp(self.minimum, self.maximum);
            }
            self.edited = false;
        }
    }

    /// An arrow: one up or down, within the range.
    pub fn step(&mut self, up: bool) {
        self.commit();
        let next = if up {
            self.value.saturating_add(1)
        } else {
            self.value.saturating_sub(1)
        };
        self.value = next.clamp(self.minimum, self.maximum);
    }

    /// A key while it is typed into: digits and a sign, the arrows, Enter. Whether anything
    /// changed.
    pub fn key(&mut self, event: &KeyDownEvent) -> bool {
        match event.keystroke.key.as_str() {
            "up" => {
                self.step(true);
                return true;
            }
            "down" => {
                self.step(false);
                return true;
            }
            _ => {}
        }
        let typed = event.keystroke.key_char.as_deref().unwrap_or_default();
        if !typed.is_empty() && !typed.chars().all(|c| c.is_ascii_digit() || c == '-') {
            return false;
        }
        match self.field.key(event) {
            KeyOutcome::Changed => true,
            KeyOutcome::Submitted | KeyOutcome::Cancelled => {
                self.commit();
                true
            }
            KeyOutcome::Ignored => false,
        }
    }
}

/// The boxes that are typed into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Field {
    /// `NUM_heading`.
    Heading,
    /// `num_simspeed`.
    Speed,
    /// `cmb_model`'s text.
    Model,
    /// `txt_cmdline`.
    Cmdline,
}

/// What a press on the map took hold of.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Press {
    /// The home marker: it follows the pointer (`onmarker`).
    Marker,
    /// The map: it pans (`mousedown`).
    Map,
}

/// What the start's thread says.
#[derive(Debug)]
enum Progress {
    /// The loading box's text.
    Say(String),
    /// It has ended.
    Done(Outcome),
}

/// `GCSViews.SITL`, the page object: made once, as `MainV2` makes it at start-up, and activated
/// each time it is shown.
pub struct Sitl {
    /// Whether the constructor has run.
    made: bool,
    /// Whether the page is showing.
    active: bool,
    /// `cmb_version.SelectedIndex`.
    version: usize,
    /// Whether `cmb_version`'s list is down.
    version_open: bool,
    /// `cmb_model.Text`.
    model: TextField,
    /// Whether `cmb_model`'s list is down.
    model_open: bool,
    /// `cmb_model`'s list's first row showing.
    model_top: usize,
    /// `NUM_heading`.
    heading: Spin,
    /// `num_simspeed`.
    speed: Spin,
    /// `txt_cmdline`.
    cmdline: TextField,
    /// `chk_wipe.Checked`.
    wipe: bool,
    /// The box being typed into.
    typing: Option<Field>,
    /// `myGMAP1`.
    map: Rc<RefCell<MapViewport>>,
    /// `homemarker.Position`.
    home: LatLon,
    /// What the left button holds on the map.
    press: Option<Press>,
    /// `myGMAP1.Zoom = 16` waiting for the map to have a width to zoom at.
    zoom_pending: bool,
    /// Where simulators come from on this desktop.
    launcher: Arc<dyn Launcher>,
    /// A start on its way.
    worker: Option<Receiver<Progress>>,
    /// The loading box's text while a start downloads.
    saying: Option<String>,
    /// The note for a desktop without a simulator.
    note: Option<String>,
    /// The WebAssembly probe on its way.
    probe: Option<Receiver<wasm::Probe>>,
    /// Whether the probe has been started.
    probed: bool,
    /// "how many?", while it asks.
    how_many: Option<InputBox>,
    /// The command line the last start gave the simulator.
    arguments: Option<String>,
    /// How the last start ended, for the facts.
    outcome: Option<String>,
    /// A line for the status line.
    status: Option<String>,
    /// A link to open, once a start has given the simulator its two seconds.
    connect: Option<String>,
    /// The picture under the pointer, which shows its `ImageOver`.
    hovered: Option<Vehicle>,
}

/// The page's keyboard focus.
pub struct Focus {
    /// The box being typed into, and "how many?"'s answer.
    pub field: FocusHandle,
    /// The page itself, for Ctrl+S and Ctrl+D (`ProcessCmdKey`).
    pub page: FocusHandle,
}

impl Focus {
    /// New handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            field: cx.focus_handle(),
            page: cx.focus_handle(),
        }
    }
}

impl Default for Sitl {
    fn default() -> Self {
        Self::new()
    }
}

impl Sitl {
    /// The page object before its constructor has run, with this desktop's launcher.
    #[must_use]
    pub fn new() -> Self {
        Self::with_launcher(Arc::from(launcher::for_this_desktop()))
    }

    /// The same with the launcher given.
    fn with_launcher(launcher: Arc<dyn Launcher>) -> Self {
        let (lat, lng) = model::DEFAULT_HOME;
        Self {
            made: false,
            active: false,
            version: 0,
            version_open: false,
            model: TextField::new(""),
            model_open: false,
            model_top: 0,
            heading: Spin::new(0, model::HEADING_MAX, 0),
            speed: Spin::new(model::SPEED_MIN, model::SPEED_MAX, model::SPEED_MIN),
            cmdline: TextField::new(""),
            wipe: false,
            typing: None,
            map: Rc::new(RefCell::new(MapViewport::new(0, 0))),
            home: LatLon::new(lat, lng).unwrap_or_default(),
            press: None,
            zoom_pending: false,
            launcher,
            worker: None,
            saying: None,
            note: None,
            probe: None,
            probed: false,
            how_many: None,
            arguments: None,
            outcome: None,
            status: None,
            connect: None,
            hovered: None,
        }
    }

    /// The pointer entering or leaving a picture: `OnMouseEnter` and `OnMouseLeave`.
    /// `// C#: ExtLibs/Controls/PictureBoxMouseOver.cs:19-35`
    pub fn hover(&mut self, vehicle: Vehicle, over: bool) {
        if over {
            self.hovered = Some(vehicle);
        } else if self.hovered == Some(vehicle) {
            self.hovered = None;
        }
    }

    /// The bitmap a picture shows: `ImageOver` while the pointer is on it, else `ImageNormal`.
    /// `// C#: ExtLibs/Controls/PictureBoxMouseOver.cs:37-50`
    #[must_use]
    pub fn picture(&self, vehicle: Vehicle) -> &'static str {
        vehicle.image(self.hovered == Some(vehicle))
    }

    /// The constructor, once, and then `Activate`: the home marker where the planner's home is
    /// (or near Canberra when it has none), the map on it at zoom 16 with the flight map's
    /// imagery. On a desktop with no simulator, the note, and the probe started.
    /// `// C#: GCSViews/SITL.cs:107-152`
    pub fn activate(
        &mut self,
        version_setting: Option<&str>,
        planned_home: (f64, f64),
        imagery: Option<&str>,
    ) {
        if !self.made {
            if let Some(dir) = sitl_directory() {
                let _ = std::fs::create_dir_all(dir);
            }
            self.version = model::version_index(version_setting);
            self.made = true;
        }
        let (lat, lng) = if planned_home == (0.0, 0.0) {
            model::DEFAULT_HOME
        } else {
            planned_home
        };
        if let Ok(home) = LatLon::new(lat, lng) {
            self.home = home;
        }
        {
            let mut map = self.map.borrow_mut();
            map.set_home(Some(self.home));
            map.centre_on(self.home);
        }
        self.zoom_pending = true;
        use_imagery(&self.map, imagery);
        self.active = true;
        if self.note.is_none() {
            self.note = self.launcher.note();
        }
        if self.note.is_some() && !self.probed {
            self.start_probe();
        }
    }

    /// Another screen shown: the box being typed into read, the lists up.
    pub fn deactivate(&mut self) {
        self.stop_typing();
        self.version_open = false;
        self.model_open = false;
        self.press = None;
        self.active = false;
    }

    /// The probe, on a thread, through the firmware pages' client.
    fn start_probe(&mut self) {
        self.probed = true;
        let (sender, receiver) = channel();
        let started = std::thread::Builder::new()
            .name("mp-sitl-wasm-probe".to_owned())
            .spawn(move || {
                let fetch = mp_firmware::manifest::fetcher();
                let _ = sender.send(wasm::probe(fetch.as_ref()));
            });
        if started.is_ok() {
            self.probe = Some(receiver);
        }
    }

    /// Once a frame: a box the focus has left read, the start's and the probe's news, and the
    /// zoom once the map has a width.
    pub fn tick(&mut self, focused: bool) {
        if self.typing.is_some() && !focused {
            self.stop_typing();
        }
        if let Some(receiver) = &self.worker {
            loop {
                match receiver.try_recv() {
                    Ok(Progress::Say(text)) => self.saying = Some(text),
                    Ok(Progress::Done(outcome)) => {
                        self.worker = None;
                        self.saying = None;
                        self.finished(outcome);
                        break;
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        self.worker = None;
                        self.saying = None;
                        break;
                    }
                }
            }
        }
        if let Some(receiver) = &self.probe {
            match receiver.try_recv() {
                Ok(found) => {
                    self.note = Some(wasm::note(&found));
                    self.probe = None;
                }
                Err(TryRecvError::Disconnected) => self.probe = None,
                Err(TryRecvError::Empty) => {}
            }
        }
        if self.zoom_pending && self.map.borrow().painted() {
            let mut map = self.map.borrow_mut();
            map.centre_on(self.home);
            map.set_zoom(model::MAP_ZOOM);
            self.zoom_pending = false;
        }
    }

    /// A start's end: connect, or the note, or the status line.
    fn finished(&mut self, outcome: Outcome) {
        match outcome {
            Outcome::Connect { link, arguments } => {
                self.outcome = Some(format!("connect {link}"));
                self.arguments = Some(arguments);
                self.connect = Some(link);
            }
            Outcome::NotAvailable(note) => {
                self.outcome = Some("not available".to_owned());
                self.status = Some(note.clone());
                self.note = Some(note);
            }
            Outcome::Failed(text) => {
                let line = text.replace('\n', " ");
                self.outcome = Some(format!("failed: {line}"));
                self.status = Some(line);
            }
        }
    }

    /// The box being typed into read, and none typed into.
    fn stop_typing(&mut self) {
        match self.typing.take() {
            Some(Field::Heading) => self.heading.commit(),
            Some(Field::Speed) => self.speed.commit(),
            Some(Field::Model | Field::Cmdline) | None => {}
        }
    }

    /// A box clicked into.
    pub fn begin_typing(&mut self, field: Field) {
        if self.typing != Some(field) {
            self.stop_typing();
        }
        self.version_open = false;
        self.model_open = false;
        match field {
            Field::Heading => self.heading.begin(),
            Field::Speed => self.speed.begin(),
            Field::Model | Field::Cmdline => {}
        }
        self.typing = Some(field);
    }

    /// A key in the box being typed into.
    pub fn key(&mut self, event: &KeyDownEvent) -> bool {
        match self.typing {
            Some(Field::Heading) => self.heading.key(event),
            Some(Field::Speed) => self.speed.key(event),
            Some(Field::Model) => self.model.key(event) != KeyOutcome::Ignored,
            Some(Field::Cmdline) => self.cmdline.key(event) != KeyOutcome::Ignored,
            None => false,
        }
    }

    /// An arrow of `NUM_heading` or `num_simspeed`.
    pub fn step(&mut self, field: Field, up: bool) {
        self.stop_typing();
        match field {
            Field::Heading => self.heading.step(up),
            Field::Speed => self.speed.step(up),
            Field::Model | Field::Cmdline => {}
        }
    }

    /// `cmb_version`'s list dropped down, or back up.
    pub fn toggle_version(&mut self) {
        self.stop_typing();
        self.model_open = false;
        self.version_open = !self.version_open;
    }

    /// A version chosen from the list.
    pub fn choose_version(&mut self, index: usize) {
        if index < model::VERSIONS.len() {
            self.version = index;
        }
        self.version_open = false;
    }

    /// `cmb_model`'s list dropped down, or back up.
    pub fn toggle_model(&mut self) {
        self.stop_typing();
        self.version_open = false;
        self.model_open = !self.model_open;
        if self.model_open {
            let mut combo = self.model_combo();
            combo.open_list();
            self.model_top = combo.top_index;
        }
    }

    /// A model chosen from the list: the box's text.
    pub fn choose_model(&mut self, index: usize) {
        if let Some(name) = model::MODELS.get(index) {
            self.model.set(*name);
        }
        self.model_open = false;
    }

    /// The wheel over `cmb_model`'s list.
    pub fn scroll_models(&mut self, lines: i32) {
        let mut combo = self.model_combo();
        combo.top_index = self.model_top;
        combo.scroll_list(lines);
        self.model_top = combo.top_index;
    }

    /// `cmb_version` as a list control.
    #[must_use]
    pub fn version_combo(&self) -> Combo {
        Combo {
            param: String::new(),
            options: model::VERSIONS
                .iter()
                .enumerate()
                .map(|(index, (name, _))| (index_key(index), (*name).to_owned()))
                .collect(),
            selected: Some(index_key(self.version)),
            enabled: true,
            top_index: 0,
        }
    }

    /// `cmb_model` as a list control, the row its text names selected.
    #[must_use]
    pub fn model_combo(&self) -> Combo {
        Combo {
            param: String::new(),
            options: model::MODELS
                .iter()
                .enumerate()
                .map(|(index, name)| (index_key(index), (*name).to_owned()))
                .collect(),
            selected: model::MODELS
                .iter()
                .position(|name| *name == self.model.value())
                .map(index_key),
            enabled: true,
            top_index: self.model_top,
        }
    }

    /// `chk_wipe` clicked.
    pub fn toggle_wipe(&mut self) {
        self.stop_typing();
        self.wipe = !self.wipe;
    }

    /// The release `cmb_version` names; `None` for "Skip Download".
    #[must_use]
    pub fn release(&self) -> Option<mp_firmware::manifest::ReleaseType> {
        model::VERSIONS
            .get(self.version)
            .and_then(|(_, release)| *release)
    }

    /// Whether a start is on its way.
    #[must_use]
    pub const fn running(&self) -> bool {
        self.worker.is_some()
    }

    /// The map's left button down: on the home marker it takes hold of it, elsewhere of the map.
    /// `// C#: GCSViews/SITL.cs:762-772, 806-810`
    pub fn map_down(&mut self, x: f32, y: f32) {
        self.stop_typing();
        let on_marker = self.map.borrow().screen_of(self.home).is_some_and(|at| {
            let (dx, dy) = (x - at.0, y - at.1);
            let (left, top, width, height) = view::PIN_AREA;
            dx >= left && dx < left + width && dy >= top && dy < top + height
        });
        if on_marker {
            self.press = Some(Press::Marker);
        } else {
            self.press = Some(Press::Map);
            self.map.borrow_mut().begin_drag(x, y);
        }
    }

    /// The pointer moved with the left button down: the marker follows it, or the map pans.
    /// `// C#: GCSViews/SITL.cs:774-798`
    pub fn map_move(&mut self, x: f32, y: f32) {
        match self.press {
            Some(Press::Marker) => {
                let at = self.map.borrow().position_at(x, y);
                if let Some(at) = at {
                    self.home = at;
                    self.map.borrow_mut().set_home(Some(at));
                }
            }
            Some(Press::Map) => self.map.borrow_mut().drag_to(x, y),
            None => {}
        }
    }

    /// The button up. `// C#: GCSViews/SITL.cs:800-804`
    pub fn map_up(&mut self) {
        if self.press.take() == Some(Press::Map) {
            self.map.borrow_mut().end_drag();
        }
    }

    /// The map, for drawing.
    #[must_use]
    pub fn map(&self) -> Rc<RefCell<MapViewport>> {
        Rc::clone(&self.map)
    }

    /// "how many?"'s answer box, while it asks.
    #[must_use]
    pub const fn how_many(&self) -> Option<&InputBox> {
        self.how_many.as_ref()
    }

    /// A key in "how many?"'s box.
    pub fn how_many_key(&mut self, event: &KeyDownEvent) -> KeyOutcome {
        self.how_many
            .as_mut()
            .map_or(KeyOutcome::Ignored, |input| input.field.key(event))
    }

    /// "how many?" cancelled: nothing is started. `// C#: GCSViews/SITL.cs:997-998`
    pub fn how_many_cancel(&mut self) {
        self.how_many = None;
    }

    /// "how many?"'s OK: the box closed, its answer kept as `InputBox` keeps every titled answer
    /// on OK - `InputBoxhowmanyhowmany` - and the answer, for the caller to parse.
    /// `// C#: GCSViews/SITL.cs:997; ExtLibs/Controls/InputBox.cs:21-27, 73-84, 178-184`
    pub fn how_many_ok(&mut self, settings: &mut crate::settings::Persisted) -> Option<String> {
        let input = self.how_many.take()?;
        input.remember(settings);
        Some(input.field.value().to_owned())
    }

    /// The home's command-line form for a point, with SRTM's height there.
    fn home_at(&self, at: (f64, f64)) -> String {
        let alt = crate::srtm::altitude(at.0, at.1).alt;
        model::home_location(at.0, at.1, alt, self.heading.value())
    }
}

/// `cmb_version`'s and `cmb_model`'s rows by index, as the list control keys them.
fn index_key(index: usize) -> i64 {
    i64::try_from(index).unwrap_or(i64::MAX)
}

/// `sitldirectory`: `sitl` in the data directory. `// C#: GCSViews/SITL.cs:41-42`
#[must_use]
pub fn sitl_directory() -> Option<PathBuf> {
    mp_settings::user_data_directory().map(|dir| dir.join("sitl"))
}

/// `myGMAP1.MapProvider = FlightData.mymap.MapProvider`: a tile store of the page's own for the
/// flight map's source, none when the flight map has none. `// C#: GCSViews/SITL.cs:137`
fn use_imagery(map: &Rc<RefCell<MapViewport>>, source: Option<&str>) {
    let Some(source) = source.and_then(mp_tiles::source::source_by_id) else {
        return;
    };
    if map.borrow().source_id() == Some(source.id) {
        return;
    }
    let cache = mp_tiles::cache::TileCache::new(mp_tiles::cache::TileCache::default_root());
    let store = if std::env::var("MP_OFFLINE").is_ok() {
        mp_tiles::TileStore::offline(source, cache)
    } else {
        mp_tiles::TileStore::new(source, cache)
    };
    map.borrow_mut().set_tiles(Arc::new(store));
}

impl MissionPlanner {
    /// The page shown: `Activate`, with the planner's home and the flight map's imagery.
    pub(crate) fn sitl_activate(&mut self) {
        let setting = self
            .persisted
            .get(model::VERSION_SETTING)
            .map(str::to_owned);
        let planned = mp_vehicle::VehicleState::planned_home();
        let imagery = self.map.borrow().source_id();
        self.sitl
            .activate(setting.as_deref(), (planned.lat, planned.lng), imagery);
    }

    /// Once a frame: the page's tick; its status line; and, once a start has given the
    /// simulator its two seconds, FLIGHT DATA shown and the link opened to it.
    /// `// C#: GCSViews/SITL.cs:713-738`
    pub(crate) fn sitl_tick(&mut self, window: &Window) {
        let focused = self.sitl_focus.field.is_focused(window);
        self.sitl.tick(focused);
        if let Some(status) = self.sitl.status.take() {
            self.file_status = Some(status);
        }
        let Some(link) = self.sitl.connect.take() else {
            return;
        };
        // `MainV2.View.ShowScreen(MainV2.View.screens[0].Name)`, as the tab strip shows it.
        if self.screen == crate::Screen::Sitl {
            self.sitl.deactivate();
            self.screen = crate::Screen::Fly;
            self.remember();
            self.save_config(crate::settings::SaveEvent::FlightData);
        }
        // `MainV2.comPort.BaseStream = client; doConnect(comPort, "preset", "5760")`.
        self.telemetry = crate::telemetry::Telemetry::connect(&link);
        self.mission_requested = false;
        self.file_status = Some(match self.telemetry.error() {
            Some(err) => format!("{}: {err}", model::FAILED_TO_CONNECT),
            None => format!("connected to {link}"),
        });
    }

    /// A picture clicked: the version remembered, the home built, and the start on a thread.
    /// `// C#: GCSViews/SITL.cs:154-238, 269-272`
    pub(crate) fn sitl_click_picture(&mut self, vehicle: Vehicle) {
        if self.sitl.running() {
            return;
        }
        self.sitl.stop_typing();
        self.sitl.version_open = false;
        self.sitl.model_open = false;
        // `Settings.Instance["sitl_download_version"] = cmb_version.SelectedIndex.ToString()`.
        self.persisted
            .set(model::VERSION_SETTING, self.sitl.version.to_string());
        let Some(dir) = sitl_directory() else {
            self.file_status = Some("no data directory to keep SITL in".to_owned());
            return;
        };
        let home = (self.sitl.home.latitude(), self.sitl.home.longitude());
        let request = launcher::Request {
            vehicle,
            release: self.sitl.release(),
            model_text: self.sitl.model.value().to_owned(),
            home: self.sitl.home_at(home),
            speedup: self.sitl.speed.value(),
            cmdline: self.sitl.cmdline.value().to_owned(),
            wipe: self.sitl.wipe,
            dir,
            path: std::env::var("PATH").unwrap_or_default(),
        };
        self.sitl.outcome = None;
        let launcher = Arc::clone(&self.sitl.launcher);
        self.sitl.worker = run(move |fetch, say| {
            launcher::start(launcher.as_ref(), fetch, &request, say, &std::thread::sleep)
        });
    }

    /// "Copter Swarm - Single link" or Ctrl+S: "how many?", starting at 10.
    /// `// C#: GCSViews/SITL.cs:812-818, 995-998, 1127-1130`
    pub(crate) fn sitl_ask_how_many(&mut self) {
        if self.sitl.running() {
            return;
        }
        self.sitl.stop_typing();
        self.sitl.how_many = Some(InputBox::new(
            model::HOW_MANY,
            model::HOW_MANY,
            &model::HOW_MANY_DEFAULT.to_string(),
        ));
    }

    /// "how many?" answered: the chain swarm on a thread. The answer is kept first, as
    /// `InputBox` keeps every titled answer on OK (under `InputBoxhowmanyhowmany`), before the
    /// `ref int` overload parses it - so an answer that is not a number is kept too. The C#'s
    /// `int.Parse` then throws, out of the button's handler; here it is a status line.
    /// `// C#: GCSViews/SITL.cs:993-1125; ExtLibs/Controls/InputBox.cs:21-27, 73-84, 178-184`
    pub(crate) fn sitl_how_many_ok(&mut self) {
        let Some(answer) = self.sitl.how_many_ok(&mut self.persisted) else {
            return;
        };
        let Ok(how_many) = answer.trim().parse::<i32>() else {
            self.file_status = Some(format!("how many? wants a whole number, not \"{answer}\""));
            return;
        };
        self.persisted
            .set(model::VERSION_SETTING, self.sitl.version.to_string());
        let (Some(dir), Some(data_dir)) = (sitl_directory(), mp_settings::user_data_directory())
        else {
            self.file_status = Some("no data directory to keep SITL in".to_owned());
            return;
        };
        // Each instance 4 m further along the heading: `newpos(NUM_heading, a * 4)`.
        #[allow(clippy::cast_precision_loss)] // the heading is 0 to 360
        let heading = self.sitl.heading.value() as f64;
        let (lat, lng) = (self.sitl.home.latitude(), self.sitl.home.longitude());
        let homes = (0..how_many.max(0))
            .map(|a| {
                let at = mp_mission::circle_survey::newpos(lat, lng, heading, f64::from(a) * 4.0);
                self.sitl.home_at(at)
            })
            .collect();
        let request = launcher::ChainRequest {
            how_many,
            release: self.sitl.release(),
            homes,
            dir,
            data_dir,
            path: std::env::var("PATH").unwrap_or_default(),
        };
        self.sitl.outcome = None;
        let launcher = Arc::clone(&self.sitl.launcher);
        self.sitl.worker = run(move |fetch, say| {
            launcher::start_chain(launcher.as_ref(), fetch, &request, say, &std::thread::sleep)
        });
    }

    /// A Multilink swarm button, or Ctrl+D: not ported, for want of one link per vehicle.
    /// `// C#: GCSViews/SITL.cs:820-824, 829-991, 1132-1145`
    pub(crate) fn sitl_multilink(&mut self) {
        self.file_status = Some(view::MULTILINK_REASON.to_owned());
    }

    /// `ProcessCmdKey`: Ctrl+S the chain swarm, Ctrl+D the Multilink copters.
    /// `// C#: GCSViews/SITL.cs:812-827`
    pub(crate) fn sitl_command_key(&mut self, event: &KeyDownEvent) -> bool {
        let keystroke = &event.keystroke;
        if !keystroke.modifiers.control || keystroke.modifiers.alt || keystroke.modifiers.shift {
            return false;
        }
        match keystroke.key.as_str() {
            "s" => {
                self.sitl_ask_how_many();
                true
            }
            "d" => {
                self.sitl_multilink();
                true
            }
            _ => false,
        }
    }
}

/// A start on a thread of its own, with the firmware pages' client, its loading-box text and its
/// end sent back.
fn run(
    work: impl FnOnce(&dyn mp_firmware::manifest::Fetch, &dyn Fn(&str)) -> Outcome + Send + 'static,
) -> Option<Receiver<Progress>> {
    let (sender, receiver) = channel();
    std::thread::Builder::new()
        .name("mp-sitl-start".to_owned())
        .spawn(move || {
            let fetch = mp_firmware::manifest::fetcher();
            let say_to = sender.clone();
            let say = move |text: &str| {
                let _ = say_to.send(Progress::Say(text.to_owned()));
            };
            let outcome = work(fetch.as_ref(), &say);
            let _ = sender.send(Progress::Done(outcome));
        })
        .ok()
        .map(|_| receiver)
}

/// What the page shows and holds, for a script.
pub fn record_facts(sitl: &Sitl, persisted: &crate::settings::Persisted) {
    use crate::facts::record;
    let or_none = |text: &str| {
        if text.is_empty() {
            "none".to_owned()
        } else {
            text.to_owned()
        }
    };
    record("sitl.active", sitl.active);
    record("sitl.group.home", model::GROUP_HOME);
    record("sitl.group.firmware", model::GROUP_FIRMWARE);
    record("sitl.group.options", model::GROUP_OPTIONS);
    record("sitl.group.advanced", model::GROUP_ADVANCED);
    record(
        "sitl.pictures",
        Vehicle::ALL
            .iter()
            .map(|vehicle| vehicle.label())
            .collect::<Vec<_>>()
            .join(","),
    );
    record(
        "sitl.version",
        model::VERSIONS
            .get(sitl.version)
            .map_or("", |(name, _)| *name),
    );
    record("sitl.version.open", sitl.version_open);
    record(
        "sitl.version.setting",
        persisted
            .get(model::VERSION_SETTING)
            .map_or_else(|| "none".to_owned(), str::to_owned),
    );
    record("sitl.model", or_none(sitl.model.value()));
    record("sitl.model.open", sitl.model_open);
    record("sitl.models", model::MODELS.len());
    record("sitl.heading", sitl.heading.shown());
    record("sitl.speedup", sitl.speed.shown());
    record("sitl.cmdline", or_none(sitl.cmdline.value()));
    record("sitl.wipe", sitl.wipe);
    record(
        "sitl.typing",
        sitl.typing.map_or_else(
            || "none".to_owned(),
            |field| format!("{field:?}").to_lowercase(),
        ),
    );
    record(
        "sitl.home",
        format!("{},{}", sitl.home.latitude(), sitl.home.longitude()),
    );
    record(
        "sitl.map.home",
        sitl.map.borrow().home().map_or_else(
            || "none".to_owned(),
            |home| format!("{},{}", home.latitude(), home.longitude()),
        ),
    );
    record("sitl.launcher", sitl.launcher.name());
    record(
        "sitl.note",
        sitl.note.clone().unwrap_or_else(|| "none".to_owned()),
    );
    record("sitl.running", sitl.running());
    record(
        "sitl.saying",
        sitl.saying.clone().unwrap_or_else(|| "none".to_owned()),
    );
    record(
        "sitl.arguments",
        sitl.arguments.clone().unwrap_or_else(|| "none".to_owned()),
    );
    record(
        "sitl.outcome",
        sitl.outcome.clone().unwrap_or_else(|| "none".to_owned()),
    );
    record("sitl.howmany.open", sitl.how_many.is_some());
    record("sitl.multilink", view::MULTILINK_REASON);
    // `sitl.picture.<plane|rover|quad|heli>`: the bitmap each picture shows, or `none` where it
    // is not carried and the named box is drawn.
    for vehicle in Vehicle::ALL {
        let shown = sitl.picture(vehicle);
        record(
            format!(
                "sitl.picture.{}",
                vehicle.control().trim_start_matches("pictureBox")
            ),
            if crate::pictures::bytes(shown).is_some() {
                shown
            } else {
                "none"
            },
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl::launcher::tests::StubLauncher;
    use launcher::Image;

    fn key(key: &str, key_char: Option<&str>) -> KeyDownEvent {
        KeyDownEvent {
            keystroke: gpui::Keystroke {
                modifiers: gpui::Modifiers::default(),
                key: key.to_owned(),
                key_char: key_char.map(ToOwned::to_owned),
            },
            is_held: false,
            prefer_character_input: false,
        }
    }

    /// `NumericUpDown`: typed, clamped on leaving, a bad number put back, the arrows within the
    /// range.
    #[test]
    fn the_number_boxes_behave_as_numeric_up_downs() {
        let mut heading = Spin::new(0, model::HEADING_MAX, 0);
        heading.begin();
        assert!(heading.key(&key("9", Some("9"))));
        assert!(!heading.key(&key("a", Some("a"))));
        assert!(heading.key(&key("9", Some("9"))));
        assert_eq!(heading.shown(), "099");
        heading.commit();
        assert_eq!(heading.value(), 99);
        heading.begin();
        assert!(heading.key(&key("9", Some("9"))));
        assert!(heading.key(&key("enter", None)));
        assert_eq!(heading.value(), 360, "999 clamped to the maximum");
        heading.step(true);
        assert_eq!(heading.value(), 360);
        heading.step(false);
        assert_eq!(heading.value(), 359);
        let mut speed = Spin::new(model::SPEED_MIN, model::SPEED_MAX, model::SPEED_MIN);
        speed.step(false);
        assert_eq!(speed.value(), 1);
        speed.begin();
        speed.key(&key("backspace", None));
        speed.commit();
        assert_eq!(speed.value(), 1, "an empty box puts the value back");
    }

    /// "how many?"'s OK keeps the answer as `InputBox` does, before anything parses it - a
    /// word as well as a number, since the C#'s `int.Parse` runs after the box has kept it -
    /// and Cancel keeps nothing.
    /// `// C#: GCSViews/SITL.cs:997; ExtLibs/Controls/InputBox.cs:21-27, 73-84, 178-184`
    #[test]
    fn how_many_ok_keeps_the_answer_under_the_input_box_key() {
        let key_name = crate::config::optional::answers_key(model::HOW_MANY, model::HOW_MANY);
        assert_eq!(key_name, "InputBoxhowmanyhowmany");
        assert!(crate::settings::PUBLISHED.contains(&key_name.as_str()));
        let mut settings = crate::settings::Persisted::at(None);
        let mut sitl = Sitl::with_launcher(Arc::new(StubLauncher::new(Image::NotAvailable(
            "n".to_owned(),
        ))));
        assert_eq!(
            sitl.how_many_ok(&mut settings),
            None,
            "no box, nothing kept"
        );
        let ask = |sitl: &mut Sitl| {
            sitl.how_many = Some(InputBox::new(
                model::HOW_MANY,
                model::HOW_MANY,
                &model::HOW_MANY_DEFAULT.to_string(),
            ));
        };
        ask(&mut sitl);
        sitl.how_many_cancel();
        assert_eq!(settings.get(&key_name), None, "Cancel keeps nothing");
        ask(&mut sitl);
        assert_eq!(
            sitl.how_many_key(&key("backspace", None)),
            KeyOutcome::Changed
        );
        assert_eq!(sitl.how_many_ok(&mut settings).as_deref(), Some("1"));
        assert!(sitl.how_many().is_none(), "OK closes the box");
        assert_eq!(settings.get(&key_name), Some("1"));
        ask(&mut sitl);
        if let Some(input) = sitl.how_many.as_mut() {
            input.field.set("some");
        }
        assert_eq!(sitl.how_many_ok(&mut settings).as_deref(), Some("some"));
        assert_eq!(
            settings.get(&key_name),
            Some("some"),
            "kept before the parse"
        );
    }

    /// `PictureBoxMouseOver`: each picture shows its `ImageNormal`, and its `ImageOver` while the
    /// pointer is on it; leaving another picture does not take it off. Every bitmap is carried.
    /// `// C#: ExtLibs/Controls/PictureBoxMouseOver.cs:19-50`
    #[test]
    fn a_picture_shows_its_over_image_under_the_pointer() {
        let mut sitl = Sitl::with_launcher(Arc::new(StubLauncher::new(Image::NotAvailable(
            "n".to_owned(),
        ))));
        let shown = |sitl: &Sitl| Vehicle::ALL.map(|vehicle| sitl.picture(vehicle));
        assert_eq!(
            shown(&sitl),
            [
                "SITL.pictureBoxplane.ImageNormal",
                "SITL.pictureBoxrover.ImageNormal",
                "SITL.pictureBoxquad.ImageNormal",
                "SITL.pictureBoxheli.ImageNormal",
            ]
        );
        sitl.hover(Vehicle::Multirotor, true);
        assert_eq!(
            sitl.picture(Vehicle::Multirotor),
            "SITL.pictureBoxquad.ImageOver"
        );
        assert_eq!(
            sitl.picture(Vehicle::Plane),
            "SITL.pictureBoxplane.ImageNormal"
        );
        sitl.hover(Vehicle::Plane, false);
        assert_eq!(
            sitl.picture(Vehicle::Multirotor),
            "SITL.pictureBoxquad.ImageOver"
        );
        sitl.hover(Vehicle::Helicopter, true);
        assert_eq!(
            sitl.picture(Vehicle::Multirotor),
            "SITL.pictureBoxquad.ImageNormal"
        );
        assert_eq!(
            sitl.picture(Vehicle::Helicopter),
            "SITL.pictureBoxheli.ImageOver"
        );
        sitl.hover(Vehicle::Helicopter, false);
        assert!(
            shown(&sitl)
                .iter()
                .all(|name| name.ends_with(".ImageNormal"))
        );
        for vehicle in Vehicle::ALL {
            for over in [false, true] {
                assert!(crate::pictures::bytes(vehicle.image(over)).is_some());
            }
        }
    }

    /// The page's lists and boxes as the model uses them.
    #[test]
    fn the_page_holds_the_choices_a_start_is_built_from() {
        let mut sitl = Sitl::with_launcher(Arc::new(StubLauncher::new(Image::NotAvailable(
            "n".to_owned(),
        ))));
        sitl.activate(Some("2"), (0.0, 0.0), None);
        assert_eq!(
            sitl.release(),
            Some(mp_firmware::manifest::ReleaseType::Official)
        );
        assert_eq!(
            (sitl.home.latitude(), sitl.home.longitude()),
            model::DEFAULT_HOME
        );
        sitl.toggle_version();
        assert!(sitl.version_open);
        sitl.choose_version(3);
        assert_eq!(sitl.release(), None);
        assert!(!sitl.version_open);
        // A second activation keeps the constructor's choice and takes the planner's home.
        sitl.activate(Some("0"), (51.5, -0.1), None);
        assert_eq!(sitl.version, 3);
        assert_eq!((sitl.home.latitude(), sitl.home.longitude()), (51.5, -0.1));
        sitl.toggle_model();
        sitl.scroll_models(3);
        assert_eq!(sitl.model_top, 3, "a row a notch");
        sitl.scroll_models(3);
        assert_eq!(sitl.model_top, 4, "34 rows, 30 shown: the wheel stops at 4");
        sitl.choose_model(13);
        assert_eq!(sitl.model.value(), "heli");
        assert_eq!(sitl.model_combo().selected, Some(13));
        sitl.toggle_wipe();
        assert!(sitl.wipe);
        sitl.step(Field::Speed, true);
        assert_eq!(sitl.speed.value(), 2);
        sitl.begin_typing(Field::Cmdline);
        assert!(sitl.key(&key("x", Some("x"))));
        assert_eq!(sitl.cmdline.value(), "x");
        sitl.tick(false);
        assert_eq!(sitl.typing, None, "the focus gone, the box is left");
        // The stub's note is not the page's until a start says so.
        assert_eq!(sitl.note, None);
        sitl.finished(Outcome::NotAvailable("n".to_owned()));
        assert_eq!(sitl.note.as_deref(), Some("n"));
        sitl.finished(Outcome::Failed(
            "Failed to start the simulator\nx".to_owned(),
        ));
        assert_eq!(
            sitl.status.as_deref(),
            Some("Failed to start the simulator x")
        );
        sitl.finished(Outcome::Connect {
            link: model::SITL_LINK.to_owned(),
            arguments: "-M+".to_owned(),
        });
        assert_eq!(sitl.connect.as_deref(), Some("tcp:127.0.0.1:5760"));
    }
}
