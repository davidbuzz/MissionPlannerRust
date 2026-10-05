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

//! The application's [`Surface`]: reads from a snapshot the window refreshes every frame, and
//! everything else carried to the window's thread as a [`Request`] and answered there.
//!
//! The C#'s plugin reads `Host.cs` and `Host.comPort.MAV` directly, on whatever thread it is on,
//! and writes through the same objects. Here the plugin's thread never touches the application:
//! what it reads is a snapshot (the telemetry view, the parameters, the fence), and what it
//! writes or asks goes to the window as a request, which the window serves through its own
//! paths - the link's parameter write, the fly screen's mode command, the plan's list, its
//! prompts - so a plugin opens no second path to the vehicle.

use mp_os::RecvTimeout as _;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, PoisonError, RwLock};
use std::time::Duration;

use crate::{
    Area, Control, CsValue, DialogResult, FencePoint, Info, MapMenu, MessageButtons, OpenedFile,
    Surface, Waypoint,
};

/// How long a plugin waits for the window to serve a request that needs no one: a parameter
/// write, a mode change, a setting. A window that has not answered in this time is gone or
/// stuck, and the plugin gets the answer "no" rather than hanging with it. Questions for the user
/// wait as long as the user takes.
pub const ANSWER_WAIT: Duration = Duration::from_secs(10);

/// A `CurrentState` reader: a field's value by its C# name.
pub type CsReader = Arc<dyn Fn(&str) -> Option<CsValue> + Send + Sync>;

/// A terrain reader: `srtm.getAltitude`.
pub type TerrainReader = Arc<dyn Fn(f64, f64) -> Option<f64> + Send + Sync>;

/// A parameter documentation reader: `GetParameterOptionsInt`.
pub type OptionsReader = Arc<dyn Fn(&str) -> Vec<(i32, String)> + Send + Sync>;

/// What a plugin reads of the application: refreshed by the window every frame, read by the
/// plugins' threads whenever they like.
#[derive(Clone)]
pub struct Snapshot {
    /// `Host.cs`: the vehicle's state as the window last saw it.
    pub cs: CsReader,
    /// `MAV.param`: the parameter list as last read.
    pub params: Arc<[(String, f64)]>,
    /// The parameters' documented values, for the connected vehicle's firmware.
    pub options: OptionsReader,
    /// `MAV.fencepoints`.
    pub fence: Arc<[FencePoint]>,
    /// `FPGMapControl.ViewArea`: the planning screen's map as last drawn.
    pub fp_view_area: Option<Area>,
    /// `srtm.getAltitude`: read on the plugin's thread, which may ask it a million times.
    pub terrain: TerrainReader,
}

impl Default for Snapshot {
    fn default() -> Self {
        Self {
            cs: Arc::new(|_| None),
            params: Arc::from(Vec::new()),
            options: Arc::new(|_| Vec::new()),
            fence: Arc::from(Vec::new()),
            fp_view_area: None,
            terrain: Arc::new(|_, _| None),
        }
    }
}

impl std::fmt::Debug for Snapshot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Snapshot")
            .field("params", &self.params.len())
            .field("fence", &self.fence.len())
            .field("fp_view_area", &self.fp_view_area)
            .finish_non_exhaustive()
    }
}

/// The answer to a request, sent back to the plugin's thread that waits for it.
#[derive(Debug)]
pub struct Reply<T>(Sender<T>);

impl<T> Reply<T> {
    /// Answers. A plugin that has stopped waiting (unloaded, timed out) is not an error.
    pub fn send(self, answer: T) {
        let _ = self.0.send(answer);
    }

    /// A reply and the receiver its answer arrives on: for a test standing in for the window.
    #[must_use]
    pub fn pair() -> (Self, Receiver<T>) {
        let (tx, rx) = mpsc::channel();
        (Self(tx), rx)
    }
}

/// Something a plugin did or asked, for the window to serve.
#[derive(Debug)]
pub struct Request {
    /// Which plugin: its index in [`crate::PluginHost::plugins`].
    pub plugin: usize,
    /// What.
    pub body: RequestBody,
}

/// What a plugin did or asked.
#[derive(Debug)]
pub enum RequestBody {
    /// `Init` said yes: the plugin is kept, and this is who it is. The C# logs "Plugin Init
    /// {name} {version} by {author}". `// C#: Plugin/PluginLoader.cs:175-182`
    Started(Info),
    /// `Init` said no, or the file is not a plugin: nothing of it is kept.
    NotLoaded(String),
    /// `Loaded` said no: the plugin stays, but its `Loop` never runs.
    Idle,
    /// The plugin faulted and is unloaded: its menu entries and its form go with it.
    Unloaded(String),
    /// `Exit` ran: the application is closing.
    Exited,
    /// The status line.
    Status(String),
    /// `log.Info`.
    Log(String),
    /// A menu entry added.
    MenuAdd {
        /// The entry's id, the plugin's own.
        id: u32,
        /// Which map's menu.
        menu: MapMenu,
        /// The entry it drops down from.
        parent: Option<u32>,
        /// Its text.
        text: String,
    },
    /// `setParam`.
    SetParam {
        /// The parameter.
        name: String,
        /// Its new value.
        value: f64,
        /// Whether the write went to the vehicle.
        reply: Reply<bool>,
    },
    /// `setMode`.
    SetMode {
        /// The mode's name.
        mode: String,
        /// Whether the change went to the vehicle.
        reply: Reply<bool>,
    },
    /// `sendPacket`.
    SendPacket {
        /// The MAVLink message id.
        message_id: u32,
        /// Its payload.
        payload: Vec<u8>,
        /// Whether it went.
        reply: Reply<bool>,
    },
    /// `CustomMessageBox.Show`.
    MessageBox {
        /// The text.
        text: String,
        /// The caption.
        caption: String,
        /// The buttons.
        buttons: MessageButtons,
        /// The button pressed.
        reply: Reply<DialogResult>,
    },
    /// `InputBox.Show`.
    InputBox {
        /// The caption.
        title: String,
        /// The question.
        prompt: String,
        /// The value the box starts with.
        value: String,
        /// The answer, none for Cancel.
        reply: Reply<Option<String>>,
    },
    /// `AddWPtoList`.
    AddWp {
        /// The item.
        wp: Waypoint,
        /// The new row's index.
        reply: Reply<i32>,
    },
    /// `InsertWP`.
    InsertWp {
        /// Where.
        index: i32,
        /// The item.
        wp: Waypoint,
    },
    /// `GetWPs`.
    GetWps,
    /// `config[key]`.
    ConfigGet {
        /// The key.
        key: String,
        /// Its value.
        reply: Reply<Option<String>>,
    },
    /// `config[key] = value`.
    ConfigSet {
        /// The key.
        key: String,
        /// The value.
        value: String,
    },
    /// `FlightData.saveTabControlActions()`, then `Settings.Instance.Save()`.
    SaveTabControlActions,
    /// The demo pointer to a named control, and its click: whether the control is on screen.
    /// The owner's Welcome-Demo-Sitl, not in the C#.
    DemoClick {
        /// The control's probe name.
        control: String,
        /// How long the pointer takes to get there.
        millis: u32,
        /// Whether it is on screen.
        reply: Reply<bool>,
    },
    /// The demo pointer to a place on the map, and its click: whether the map shows it.
    DemoClickMap {
        /// Latitude, degrees.
        lat: f64,
        /// Longitude, degrees.
        lng: f64,
        /// How long the pointer takes to get there.
        millis: u32,
        /// Whether the map is on screen and shows the place.
        reply: Reply<bool>,
    },
    /// Text typed into what has the keyboard, then Enter.
    DemoType(String),
    /// Whether a demo gesture is under way.
    DemoBusy(Reply<bool>),
    /// Whether a named control is on screen.
    DemoVisible {
        /// The control's probe name.
        control: String,
        /// Whether it is.
        reply: Reply<bool>,
    },
    /// `OpenFileDialog` and the read.
    OpenFile {
        /// The dialog's title.
        title: String,
        /// Its filter.
        filter: String,
        /// The file, none when cancelled.
        reply: Reply<Option<OpenedFile>>,
    },
    /// `SaveFileDialog` and the write.
    SaveFile {
        /// The dialog's title.
        title: String,
        /// Its filter.
        filter: String,
        /// The name it starts with.
        name: String,
        /// What to write.
        data: Vec<u8>,
        /// The path written, none when cancelled.
        reply: Reply<Option<String>>,
    },
    /// A file under the user data directory.
    WriteUserData {
        /// The path under it.
        path: String,
        /// What to write.
        data: Vec<u8>,
        /// The full path written, or why not.
        reply: Reply<Result<String, String>>,
    },
    /// The plugin's form, shown or replaced.
    FormShow {
        /// Its title.
        title: String,
        /// Its controls.
        controls: Vec<Control>,
    },
    /// The plugin's form closed.
    FormClose,
}

/// The application's [`Surface`] for one plugin.
#[derive(Debug)]
pub struct ChannelSurface {
    plugin: usize,
    requests: Sender<Request>,
    snapshot: Arc<RwLock<Snapshot>>,
    next_menu_id: u32,
}

impl ChannelSurface {
    /// The surface of plugin `plugin`: requests go to `requests`, reads come from `snapshot`.
    #[must_use]
    pub const fn new(
        plugin: usize,
        requests: Sender<Request>,
        snapshot: Arc<RwLock<Snapshot>>,
    ) -> Self {
        Self {
            plugin,
            requests,
            snapshot,
            next_menu_id: 0,
        }
    }

    fn read(&self) -> Snapshot {
        self.snapshot
            .read()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    fn tell(&self, body: RequestBody) {
        let _ = self.requests.send(Request {
            plugin: self.plugin,
            body,
        });
    }

    /// A request that waits for its answer: `default` when the window is gone, or, with `wait`,
    /// when it has not answered in that time.
    fn ask<T>(
        &self,
        body: impl FnOnce(Reply<T>) -> RequestBody,
        default: T,
        wait: Option<Duration>,
    ) -> T {
        let (reply, answer) = Reply::pair();
        self.tell(body(reply));
        match wait {
            Some(wait) => match answer.os_recv_timeout(wait) {
                Ok(value) => value,
                Err(RecvTimeoutError::Timeout | RecvTimeoutError::Disconnected) => default,
            },
            None => answer.recv().unwrap_or(default),
        }
    }
}

impl Surface for ChannelSurface {
    fn cs(&mut self, name: &str) -> Option<CsValue> {
        let reader = self.read().cs;
        reader(name)
    }

    fn get_param(&mut self, name: &str) -> Option<f64> {
        self.read()
            .params
            .iter()
            .find(|(param, _)| param == name)
            .map(|(_, value)| *value)
    }

    fn param_options(&mut self, name: &str) -> Vec<(i32, String)> {
        let reader = self.read().options;
        reader(name)
    }

    fn set_param(&mut self, name: &str, value: f64) -> bool {
        let name = name.to_owned();
        self.ask(
            |reply| RequestBody::SetParam { name, value, reply },
            false,
            Some(ANSWER_WAIT),
        )
    }

    fn set_mode(&mut self, mode: &str) -> bool {
        let mode = mode.to_owned();
        self.ask(
            |reply| RequestBody::SetMode { mode, reply },
            false,
            Some(ANSWER_WAIT),
        )
    }

    fn send_packet(&mut self, message_id: u32, payload: Vec<u8>) -> bool {
        self.ask(
            |reply| RequestBody::SendPacket {
                message_id,
                payload,
                reply,
            },
            false,
            Some(ANSWER_WAIT),
        )
    }

    fn menu_add(&mut self, menu: MapMenu, parent: Option<u32>, text: &str) -> u32 {
        let id = self.next_menu_id;
        self.next_menu_id = self.next_menu_id.wrapping_add(1);
        self.tell(RequestBody::MenuAdd {
            id,
            menu,
            parent,
            text: text.to_owned(),
        });
        id
    }

    fn message_box(&mut self, text: &str, caption: &str, buttons: MessageButtons) -> DialogResult {
        let (text, caption) = (text.to_owned(), caption.to_owned());
        self.ask(
            |reply| RequestBody::MessageBox {
                text,
                caption,
                buttons,
                reply,
            },
            DialogResult::Cancel,
            None,
        )
    }

    fn input_box(&mut self, title: &str, prompt: &str, value: &str) -> Option<String> {
        let (title, prompt, value) = (title.to_owned(), prompt.to_owned(), value.to_owned());
        self.ask(
            |reply| RequestBody::InputBox {
                title,
                prompt,
                value,
                reply,
            },
            None,
            None,
        )
    }

    fn status(&mut self, text: &str) {
        self.tell(RequestBody::Status(text.to_owned()));
    }

    fn log(&mut self, text: &str) {
        self.tell(RequestBody::Log(text.to_owned()));
    }

    fn add_wp(&mut self, wp: Waypoint) -> i32 {
        self.ask(
            |reply| RequestBody::AddWp { wp, reply },
            -1,
            Some(ANSWER_WAIT),
        )
    }

    fn insert_wp(&mut self, index: i32, wp: Waypoint) {
        self.tell(RequestBody::InsertWp { index, wp });
    }

    fn get_wps(&mut self) {
        self.tell(RequestBody::GetWps);
    }

    fn fence_points(&mut self) -> Vec<FencePoint> {
        self.read().fence.to_vec()
    }

    fn config_get(&mut self, key: &str) -> Option<String> {
        let key = key.to_owned();
        self.ask(
            |reply| RequestBody::ConfigGet { key, reply },
            None,
            Some(ANSWER_WAIT),
        )
    }

    fn config_set(&mut self, key: &str, value: &str) {
        self.tell(RequestBody::ConfigSet {
            key: key.to_owned(),
            value: value.to_owned(),
        });
    }

    fn save_tab_control_actions(&mut self) {
        self.tell(RequestBody::SaveTabControlActions);
    }

    fn demo_click(&mut self, control: &str, millis: u32) -> bool {
        let control = control.to_owned();
        self.ask(
            |reply| RequestBody::DemoClick {
                control,
                millis,
                reply,
            },
            false,
            Some(ANSWER_WAIT),
        )
    }

    fn demo_click_map(&mut self, lat: f64, lng: f64, millis: u32) -> bool {
        self.ask(
            |reply| RequestBody::DemoClickMap {
                lat,
                lng,
                millis,
                reply,
            },
            false,
            Some(ANSWER_WAIT),
        )
    }

    fn demo_type(&mut self, text: &str) {
        self.tell(RequestBody::DemoType(text.to_owned()));
    }

    fn demo_busy(&mut self) -> bool {
        self.ask(RequestBody::DemoBusy, false, Some(ANSWER_WAIT))
    }

    fn demo_visible(&mut self, control: &str) -> bool {
        let control = control.to_owned();
        self.ask(
            |reply| RequestBody::DemoVisible { control, reply },
            false,
            Some(ANSWER_WAIT),
        )
    }

    // This map has no rubber band: `SelectedArea` is always empty (plan.rs's Prefetch says so).
    fn fp_selected_area(&mut self) -> Option<Area> {
        None
    }

    fn fp_view_area(&mut self) -> Option<Area> {
        self.read().fp_view_area
    }

    fn terrain_altitude(&mut self, lat: f64, lng: f64) -> Option<f64> {
        let reader = self.read().terrain;
        reader(lat, lng)
    }

    fn open_file(&mut self, title: &str, filter: &str) -> Option<OpenedFile> {
        let (title, filter) = (title.to_owned(), filter.to_owned());
        self.ask(
            |reply| RequestBody::OpenFile {
                title,
                filter,
                reply,
            },
            None,
            None,
        )
    }

    fn save_file(
        &mut self,
        title: &str,
        filter: &str,
        name: &str,
        data: Vec<u8>,
    ) -> Option<String> {
        let (title, filter, name) = (title.to_owned(), filter.to_owned(), name.to_owned());
        self.ask(
            |reply| RequestBody::SaveFile {
                title,
                filter,
                name,
                data,
                reply,
            },
            None,
            None,
        )
    }

    fn write_user_data(&mut self, path: &str, data: Vec<u8>) -> Result<String, String> {
        let path = path.to_owned();
        self.ask(
            |reply| RequestBody::WriteUserData { path, data, reply },
            Err("the application did not answer".to_owned()),
            Some(ANSWER_WAIT),
        )
    }

    fn form_show(&mut self, title: &str, controls: Vec<Control>) {
        self.tell(RequestBody::FormShow {
            title: title.to_owned(),
            controls,
        });
    }

    fn form_close(&mut self) {
        self.tell(RequestBody::FormClose);
    }
}
